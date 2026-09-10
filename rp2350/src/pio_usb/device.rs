// SPDX-License-Identifier: MIT
//! What the device does when a token arrives.
//!
//! This is the decision layer between [`super::packet`] (which knows about
//! PIDs) and the hardware (which has to act inside the bus turnaround window).
//! Deliberately pure: the interrupt handler has a few hundred nanoseconds to
//! decide and act, so the deciding is separated from the acting and tested on
//! the host, where it costs nothing.
//!
//! # Isochronous is the whole point
//!
//! Every rule below is shared with any other USB device except one, and that
//! one is why this project exists. An isochronous transaction has **no
//! handshake phase**: the host sends no ACK after an iso IN, and the device
//! must send none after an iso OUT. Pico-PIO-USB waits for that handshake
//! unconditionally, which is what makes it unusable here — see [`super`].
//!
//! Iso endpoints also never retry and never toggle DATA0/DATA1: a late packet
//! is a lost packet, and that is the trade audio makes for bounded latency.

use super::packet::{Pid, Token};

/// The kind of transfer an endpoint carries.
///
/// Mirrors `embassy_usb_driver::EndpointType`, kept separate so this module
/// stays host-testable without dragging the USB stack in.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum EpType {
    Control,
    Isochronous,
    Bulk,
    Interrupt,
}

impl EpType {
    /// Whether transactions on this endpoint have a handshake phase.
    ///
    /// The single rule that separates isochronous from everything else.
    pub const fn has_handshake(self) -> bool {
        !matches!(self, EpType::Isochronous)
    }

    /// Whether this endpoint alternates DATA0/DATA1. Isochronous full-speed
    /// endpoints are always DATA0.
    pub const fn toggles_data_pid(self) -> bool {
        !matches!(self, EpType::Isochronous)
    }
}

/// What one endpoint can do right now.
#[derive(Copy, Clone, Debug)]
pub struct EpState {
    pub ep_type: EpType,
    /// A packet is queued and ready to go out (IN), or a buffer is free to
    /// receive into (OUT).
    pub ready: bool,
    /// The endpoint is halted and must answer STALL.
    pub stalled: bool,
    /// Next data PID to use. Meaningless for isochronous.
    pub data1: bool,
}

impl EpState {
    pub const fn new(ep_type: EpType) -> Self {
        Self {
            ep_type,
            ready: false,
            stalled: false,
            data1: false,
        }
    }

    /// The PID a data packet from this endpoint should carry.
    pub const fn data_pid(&self) -> Pid {
        if self.data1 && self.ep_type.toggles_data_pid() {
            Pid::Data1
        } else {
            Pid::Data0
        }
    }
}

/// What the hardware should do about a token.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Response {
    /// Not addressed to us, or not something we act on. Say nothing at all —
    /// answering a token meant for another device is worse than silence.
    Ignore,
    /// Transmit the endpoint's queued packet with this PID. If `handshake` is
    /// false the transaction ends when the packet does, with nothing to wait
    /// for — that is the isochronous case.
    SendData { pid: Pid, expect_handshake: bool },
    /// Send a handshake and nothing else: NAK when there is no data ready,
    /// STALL when the endpoint is halted.
    SendHandshake(Pid),
    /// Receive a data packet, then reply with this handshake. `None` means send
    /// nothing back, which is the isochronous OUT case.
    Receive { reply: Option<Pid> },
    /// A start of frame. Not addressed to an endpoint; the frame number is the
    /// payload, and it is what the bridge paces against.
    Sof,
}

/// Decide what to do about a received token.
///
/// `address` is the address this device currently answers to — zero until the
/// host has assigned one, which is why it is a parameter rather than a
/// constant.
pub fn respond(token: Token, address: u8, ep: Option<&EpState>) -> Response {
    if token.pid == Pid::Sof {
        return Response::Sof;
    }
    if !token.pid.is_token() || token.address != address {
        return Response::Ignore;
    }
    let Some(ep) = ep else {
        // A token for an endpoint we do not have. Silence rather than STALL:
        // the host is entitled to probe, and a STALL from a non-existent
        // endpoint is a different (and wrong) answer.
        return Response::Ignore;
    };

    match token.pid {
        // SETUP is never stalled and never NAKed. A device that refuses a
        // SETUP cannot be configured at all, so this is deliberately checked
        // before `stalled` and before `ready`: a control endpoint must always
        // accept the setup stage, even one that is currently halted, since
        // that is how the host clears the halt.
        Pid::Setup => Response::Receive {
            reply: Some(Pid::Ack),
        },

        Pid::In => {
            if !ep.ep_type.has_handshake() {
                // Isochronous, and the handshake check comes first on purpose.
                // There is no NAK in isochronous, so nothing to send means
                // silence: the host records a missed frame and moves on.
                //
                // A *halted* iso endpoint is silent for the same reason. STALL
                // is a handshake, so answering one here would break the rule
                // this endpoint type is defined by. (The Halt feature is only
                // defined for bulk and interrupt endpoints in the first place —
                // USB 2.0 section 9.4.5 — so this is a state the host should
                // never put us in, but silence is the answer that stays legal
                // if it does.)
                if ep.ready && !ep.stalled {
                    Response::SendData {
                        pid: ep.data_pid(),
                        expect_handshake: false,
                    }
                } else {
                    Response::Ignore
                }
            } else if ep.stalled {
                Response::SendHandshake(Pid::Stall)
            } else if ep.ready {
                Response::SendData {
                    pid: ep.data_pid(),
                    expect_handshake: true,
                }
            } else {
                Response::SendHandshake(Pid::Nak)
            }
        }

        Pid::Out => {
            if !ep.ep_type.has_handshake() {
                // Isochronous OUT: take the data, acknowledge nothing.
                Response::Receive { reply: None }
            } else if ep.stalled {
                Response::Receive {
                    reply: Some(Pid::Stall),
                }
            } else if ep.ready {
                Response::Receive {
                    reply: Some(Pid::Ack),
                }
            } else {
                Response::Receive {
                    reply: Some(Pid::Nak),
                }
            }
        }

        _ => Response::Ignore,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ADDR: u8 = 7;

    fn token(pid: Pid, address: u8, endpoint: u8) -> Token {
        Token {
            pid,
            address,
            endpoint,
        }
    }

    fn ep(ep_type: EpType, ready: bool) -> EpState {
        EpState {
            ready,
            ..EpState::new(ep_type)
        }
    }

    #[test]
    fn tokens_for_other_devices_are_ignored() {
        for pid in [Pid::In, Pid::Out, Pid::Setup] {
            let e = ep(EpType::Bulk, true);
            assert_eq!(
                respond(token(pid, ADDR + 1, 0), ADDR, Some(&e)),
                Response::Ignore,
                "{pid:?} for another address"
            );
        }
    }

    #[test]
    fn tokens_for_unknown_endpoints_are_ignored_not_stalled() {
        assert_eq!(respond(token(Pid::In, ADDR, 5), ADDR, None), Response::Ignore);
    }

    #[test]
    fn sof_is_recognised_regardless_of_address() {
        // A SOF's eleven bits are a frame number, not an address, so the
        // address check must not be applied to it.
        assert_eq!(respond(token(Pid::Sof, 0x2A, 0), ADDR, None), Response::Sof);
    }

    #[test]
    fn in_with_data_sends_it() {
        let e = ep(EpType::Bulk, true);
        assert_eq!(
            respond(token(Pid::In, ADDR, 1), ADDR, Some(&e)),
            Response::SendData {
                pid: Pid::Data0,
                expect_handshake: true
            }
        );
    }

    #[test]
    fn in_without_data_naks_except_isochronous() {
        for t in [EpType::Bulk, EpType::Interrupt, EpType::Control] {
            let e = ep(t, false);
            assert_eq!(
                respond(token(Pid::In, ADDR, 1), ADDR, Some(&e)),
                Response::SendHandshake(Pid::Nak),
                "{t:?}"
            );
        }
    }

    /// The rule that Pico-PIO-USB does not implement, and the reason this
    /// driver exists: isochronous has no handshake, in either direction.
    #[test]
    fn isochronous_never_handshakes() {
        // An iso IN with nothing ready must be silent, not a NAK.
        let empty = ep(EpType::Isochronous, false);
        assert_eq!(
            respond(token(Pid::In, ADDR, 1), ADDR, Some(&empty)),
            Response::Ignore
        );

        // An iso IN with data expects nothing back.
        let ready = ep(EpType::Isochronous, true);
        assert_eq!(
            respond(token(Pid::In, ADDR, 1), ADDR, Some(&ready)),
            Response::SendData {
                pid: Pid::Data0,
                expect_handshake: false
            }
        );

        // An iso OUT is received and not acknowledged.
        assert_eq!(
            respond(token(Pid::Out, ADDR, 1), ADDR, Some(&ready)),
            Response::Receive { reply: None }
        );
    }

    /// A halted isochronous endpoint must still emit no handshake, because
    /// STALL *is* a handshake. This test was originally written asserting the
    /// opposite, which contradicted its own name — the stall check ran before
    /// the isochronous check, so a halted iso endpoint answered STALL.
    #[test]
    fn stalled_isochronous_still_never_handshakes() {
        let mut e = ep(EpType::Isochronous, true);
        e.stalled = true;
        assert_eq!(
            respond(token(Pid::In, ADDR, 1), ADDR, Some(&e)),
            Response::Ignore,
            "STALL is a handshake, and isochronous has none"
        );
        assert_eq!(
            respond(token(Pid::Out, ADDR, 1), ADDR, Some(&e)),
            Response::Receive { reply: None },
            "nor in the OUT direction"
        );
    }

    #[test]
    fn stalled_endpoints_report_it() {
        let mut e = ep(EpType::Bulk, true);
        e.stalled = true;
        assert_eq!(
            respond(token(Pid::In, ADDR, 1), ADDR, Some(&e)),
            Response::SendHandshake(Pid::Stall)
        );
        assert_eq!(
            respond(token(Pid::Out, ADDR, 1), ADDR, Some(&e)),
            Response::Receive {
                reply: Some(Pid::Stall)
            }
        );
    }

    /// A device that refuses a SETUP cannot be configured, so it is accepted
    /// even on a halted endpoint — that is how the host clears the halt.
    #[test]
    fn setup_is_always_accepted() {
        let mut e = ep(EpType::Control, false);
        e.stalled = true;
        assert_eq!(
            respond(token(Pid::Setup, ADDR, 0), ADDR, Some(&e)),
            Response::Receive {
                reply: Some(Pid::Ack)
            }
        );
    }

    #[test]
    fn data_pid_toggles_except_on_isochronous() {
        let mut e = ep(EpType::Bulk, true);
        assert_eq!(e.data_pid(), Pid::Data0);
        e.data1 = true;
        assert_eq!(e.data_pid(), Pid::Data1);

        let mut iso = ep(EpType::Isochronous, true);
        iso.data1 = true;
        assert_eq!(iso.data_pid(), Pid::Data0, "full-speed iso is always DATA0");
    }

    #[test]
    fn address_zero_is_answered_before_one_is_assigned() {
        // Enumeration starts here: until SET_ADDRESS lands, the device must
        // answer on address 0.
        let e = ep(EpType::Control, false);
        assert_eq!(
            respond(token(Pid::Setup, 0, 0), 0, Some(&e)),
            Response::Receive {
                reply: Some(Pid::Ack)
            }
        );
    }
}

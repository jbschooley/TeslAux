// SPDX-License-Identifier: MIT
//! Running one USB transaction, from token to handshake.
//!
//! This is the part that has a deadline. The edge detector raises
//! `IRQ_RX_START` when it sees a start of packet; from there the sequence is
//! always the same — collect the token, decide, act — and the deciding is
//! [`super::device::respond`], already tested.
//!
//! # The timing budget
//!
//! A token is four decoded bytes (SYNC, PID, and two of address, endpoint and
//! CRC5), so about 2.7 µs elapses between the interrupt and the last of them
//! arriving. That wait is a busy spin with interrupts off, and it is
//! unavoidable: the token has to be complete before it can be answered.
//!
//! What follows it is not. Once the token's EOP passes, the device has between
//! 2 and 7.5 bit times — 167 to 625 ns — to put the first bit of its reply on
//! the wire. At 96 MHz that is 16 to 60 cycles, which is why nothing here
//! builds a packet: [`super::endpoint`] has already encoded them, and the reply
//! is a DMA pointed at bytes that are finished.
//!
//! # Why the bus is behind a trait
//!
//! [`Port`] exists so this sequencing can be tested on the host against a
//! scripted byte stream. The hardware implementation is thin by design — every
//! decision worth getting wrong is made here or in `device`, where it can be
//! checked without a logic analyser. That is a deliberate response to how this
//! driver's bugs have actually been found: six so far, none in the PIO
//! programs, all in the scaffolding around them.

use super::device::{respond, EpState, Response};
use super::endpoint::Handshakes;
use super::packet::{check_data, decode_sof, decode_token, Pid, SYNC};

/// The bus operations a transaction needs.
pub trait Port {
    /// The next decoded byte, if one has arrived yet.
    fn rx(&mut self) -> Option<u8>;
    /// Whether the receiver has seen the end of the current packet.
    fn eop(&self) -> bool;
    /// Take the bus and transmit an already-encoded packet.
    fn transmit(&mut self, symbols: &[u32]);
}

/// How a transaction ended.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Outcome {
    /// A start of frame, carrying its frame number. What the bridge paces
    /// against, and the only "transaction" with no endpoint.
    Sof(u16),
    /// A data packet went out.
    Sent,
    /// A handshake went out and nothing else.
    Handshake(Pid),
    /// A data packet arrived and was accepted, with this many payload bytes.
    Received(usize),
    /// A data packet arrived and its CRC16 failed, so nothing was
    /// acknowledged. The host will retry; an isochronous host will not, and
    /// that is the trade.
    Corrupt,
    /// Addressed elsewhere, or an endpoint we do not have.
    Ignored,
    /// The token did not survive its own checks — bad SYNC, bad PID check
    /// nibble, or a failed CRC5. Answering it would mean replying on another
    /// device's behalf.
    BadToken,
    /// The bus went quiet mid-packet.
    Timeout,
}

/// How many spins to wait for each byte before giving up.
///
/// A byte is eight bit times, about 667 ns. This is generous enough to cover
/// a slow host and short enough that a disconnected bus does not wedge the
/// interrupt handler — which, with interrupts off, would take the whole device
/// down rather than just one transfer.
const SPIN_LIMIT: u32 = 20_000;

/// Device-wide state a transaction needs.
pub struct Engine {
    /// The address this device answers to. Zero until SET_ADDRESS, which is
    /// why enumeration works at all.
    pub address: u8,
    /// The most recent frame number seen in a SOF.
    pub frame: u16,
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}

impl Engine {
    pub const fn new() -> Self {
        Self {
            address: 0,
            frame: 0,
        }
    }

    /// Read one byte, spinning until it arrives or the bus goes quiet.
    fn byte<P: Port>(port: &mut P) -> Option<u8> {
        for _ in 0..SPIN_LIMIT {
            if let Some(b) = port.rx() {
                return Some(b);
            }
        }
        None
    }

    /// Handle one transaction, having just been told a packet started.
    ///
    /// `endpoint` looks up an endpoint's state by its number and direction;
    /// returning `None` means we do not have it. `rx_buf` receives the payload
    /// of an OUT or SETUP packet.
    pub fn handle<'p, P: Port>(
        &mut self,
        port: &mut P,
        handshakes: &Handshakes,
        endpoint: impl Fn(u8, bool) -> Option<EpState>,
        packet: impl Fn(u8) -> Option<&'p [u32]>,
        rx_buf: &mut [u8],
    ) -> Outcome {
        // --- collect the token ---
        let Some(sync) = Self::byte(port) else {
            return Outcome::Timeout;
        };
        if sync != SYNC {
            return Outcome::BadToken;
        }
        let (Some(pid_byte), Some(b0), Some(b1)) =
            (Self::byte(port), Self::byte(port), Self::byte(port))
        else {
            return Outcome::Timeout;
        };
        let Some(pid) = Pid::from_byte(pid_byte) else {
            return Outcome::BadToken;
        };

        if pid == Pid::Sof {
            return match decode_sof(b0, b1) {
                Some(frame) => {
                    self.frame = frame;
                    Outcome::Sof(frame)
                }
                None => Outcome::BadToken,
            };
        }

        let Some(token) = decode_token(pid, b0, b1) else {
            return Outcome::BadToken;
        };

        // --- decide ---
        let is_in = token.pid == Pid::In;
        let state = endpoint(token.endpoint, is_in);
        match respond(token, self.address, state.as_ref()) {
            // `Sof` cannot reach here — it returned above — but matching it
            // explicitly keeps this exhaustive, so a new variant becomes a
            // compile error rather than being swallowed by a catch-all.
            Response::Ignore | Response::Sof => Outcome::Ignored,

            Response::SendHandshake(pid) => {
                port.transmit(handshakes.get(pid));
                Outcome::Handshake(pid)
            }

            Response::SendData { .. } => {
                // The packet was encoded when the application wrote it; this
                // only has to point the DMA at it. Returning "sent" without
                // sending would be a lie, so the lookup is a parameter rather
                // than the caller's job afterwards.
                match packet(token.endpoint) {
                    Some(symbols) => {
                        port.transmit(symbols);
                        Outcome::Sent
                    }
                    // `respond` said the endpoint was ready and the buffer
                    // disagrees. Nothing safe can go on the wire, and a
                    // half-truth is worse than silence.
                    None => Outcome::Ignored,
                }
            }

            Response::Receive { reply } => {
                let n = match Self::receive(port, rx_buf) {
                    Some(n) => n,
                    None => return Outcome::Timeout,
                };
                let Some(payload) = check_data(&rx_buf[..n]) else {
                    // A corrupt packet is never acknowledged: the host must be
                    // free to retry, and acknowledging damage is worse than
                    // staying silent.
                    return Outcome::Corrupt;
                };
                let len = payload.len();
                if let Some(pid) = reply {
                    port.transmit(handshakes.get(pid));
                }
                Outcome::Received(len)
            }
        }
    }

    /// Read a data packet's body — everything after its PID, including the two
    /// CRC bytes — until the receiver reports the end of the packet.
    fn receive<P: Port>(port: &mut P, buf: &mut [u8]) -> Option<usize> {
        // Skip the data packet's own SYNC and PID.
        let _sync = Self::byte(port)?;
        let _pid = Self::byte(port)?;

        let mut n = 0;
        for _ in 0..SPIN_LIMIT {
            while let Some(b) = port.rx() {
                if n < buf.len() {
                    buf[n] = b;
                }
                n += 1;
            }
            if port.eop() {
                // Drain whatever landed between the last read and the flag.
                while let Some(b) = port.rx() {
                    if n < buf.len() {
                        buf[n] = b;
                    }
                    n += 1;
                }
                return (n <= buf.len()).then_some(n);
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pio_usb::device::EpType;
    use crate::pio_usb::packet::{build_data, encode_token};

    /// A scripted bus: hands out bytes in order, records what was sent, and
    /// reports EOP once the script is exhausted.
    struct Fake {
        rx: std::collections::VecDeque<u8>,
        sent: Vec<Vec<u32>>,
    }

    impl Fake {
        fn new(bytes: &[u8]) -> Self {
            Self {
                rx: bytes.iter().copied().collect(),
                sent: Vec::new(),
            }
        }
    }

    impl Port for Fake {
        fn rx(&mut self) -> Option<u8> {
            self.rx.pop_front()
        }
        fn eop(&self) -> bool {
            self.rx.is_empty()
        }
        fn transmit(&mut self, symbols: &[u32]) {
            self.sent.push(symbols.to_vec());
        }
    }

    const ADDR: u8 = 7;

    fn token_bytes(pid: Pid, address: u8, ep: u8) -> Vec<u8> {
        let [b0, b1] = encode_token(address, ep);
        vec![SYNC, pid.to_byte(), b0, b1]
    }

    fn engine() -> Engine {
        Engine {
            address: ADDR,
            frame: 0,
        }
    }

    fn no_endpoints(_: u8, _: bool) -> Option<EpState> {
        None
    }

    fn no_packet(_: u8) -> Option<&'static [u32]> {
        None
    }

    fn one(ep_type: EpType, ready: bool) -> impl Fn(u8, bool) -> Option<EpState> {
        move |_, _| {
            Some(EpState {
                ready,
                ..EpState::new(ep_type)
            })
        }
    }

    #[test]
    fn a_sof_updates_the_frame_number() {
        let frame = 0x2AB;
        let [b0, b1] = {
            let raw = frame | ((crate::pio_usb::codec::crc5(frame) as u16) << 11);
            [raw as u8, (raw >> 8) as u8]
        };
        let mut port = Fake::new(&[SYNC, Pid::Sof.to_byte(), b0, b1]);
        let mut e = engine();
        let h = Handshakes::new();
        assert_eq!(
            e.handle(&mut port, &h, no_endpoints, no_packet, &mut [0u8; 8]),
            Outcome::Sof(frame)
        );
        assert_eq!(e.frame, frame);
        assert!(port.sent.is_empty(), "a SOF is never answered");
    }

    #[test]
    fn a_bad_sync_is_rejected_without_replying() {
        let mut bytes = token_bytes(Pid::In, ADDR, 1);
        bytes[0] = 0x81;
        let mut port = Fake::new(&bytes);
        let h = Handshakes::new();
        assert_eq!(
            engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]),
            Outcome::BadToken
        );
        assert!(port.sent.is_empty());
    }

    #[test]
    fn a_corrupt_token_is_never_answered() {
        // Flipping a bit in the address must fail CRC5, and a device that
        // answered would be replying for whoever the corrupted address names.
        let mut bytes = token_bytes(Pid::In, ADDR, 1);
        bytes[2] ^= 0x01;
        let mut port = Fake::new(&bytes);
        let h = Handshakes::new();
        assert_eq!(
            engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]),
            Outcome::BadToken
        );
        assert!(port.sent.is_empty(), "silence, not a NAK");
    }

    #[test]
    fn a_token_for_another_device_is_ignored_silently() {
        let mut port = Fake::new(&token_bytes(Pid::In, ADDR + 1, 1));
        let h = Handshakes::new();
        assert_eq!(
            engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]),
            Outcome::Ignored
        );
        assert!(port.sent.is_empty());
    }

    #[test]
    fn an_in_with_no_data_naks() {
        let mut port = Fake::new(&token_bytes(Pid::In, ADDR, 1));
        let h = Handshakes::new();
        assert_eq!(
            engine().handle(&mut port, &h, one(EpType::Bulk, false), no_packet, &mut [0u8; 8]),
            Outcome::Handshake(Pid::Nak)
        );
        assert_eq!(port.sent.len(), 1);
        assert_eq!(port.sent[0], h.get(Pid::Nak));
    }

    #[test]
    fn an_in_with_data_transmits_the_prepared_packet() {
        use crate::pio_usb::endpoint::{words_for_payload, PacketBuf};

        let payload = [0xA5u8; 8];
        let mut buf: PacketBuf<12, { words_for_payload(8) }> = PacketBuf::new();
        buf.prepare(Pid::Data0, &payload).unwrap();
        let expected = buf.symbols().to_vec();

        let mut port = Fake::new(&token_bytes(Pid::In, ADDR, 1));
        let h = Handshakes::new();
        assert_eq!(
            engine().handle(
                &mut port,
                &h,
                one(EpType::Bulk, true),
                |_| Some(buf.symbols()),
                &mut [0u8; 8]
            ),
            Outcome::Sent
        );
        assert_eq!(port.sent.len(), 1, "exactly one packet on the wire");
        assert_eq!(port.sent[0], expected, "and it is the prepared one");
    }

    /// If `respond` says ready but no buffer turns up, nothing goes out. A
    /// half-formed packet on the wire is worse than a missed frame.
    #[test]
    fn an_in_with_no_prepared_buffer_stays_silent() {
        let mut port = Fake::new(&token_bytes(Pid::In, ADDR, 1));
        let h = Handshakes::new();
        assert_eq!(
            engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]),
            Outcome::Ignored
        );
        assert!(port.sent.is_empty());
    }

    /// The isochronous rule, at the level that actually puts bits on the wire.
    #[test]
    fn an_isochronous_in_with_no_data_says_nothing() {
        let mut port = Fake::new(&token_bytes(Pid::In, ADDR, 1));
        let h = Handshakes::new();
        assert_eq!(
            engine().handle(&mut port, &h, one(EpType::Isochronous, false), no_packet, &mut [0u8; 8]),
            Outcome::Ignored
        );
        assert!(port.sent.is_empty(), "there is no NAK in isochronous");
    }

    #[test]
    fn an_out_is_received_and_acked() {
        let payload = [0xDEu8, 0xAD, 0xBE, 0xEF];
        let mut data = [0u8; 16];
        let n = build_data(Pid::Data0, &payload, &mut data).unwrap();

        let mut bytes = token_bytes(Pid::Out, ADDR, 2);
        bytes.extend_from_slice(&data[..n]);

        let mut port = Fake::new(&bytes);
        let h = Handshakes::new();
        let mut buf = [0u8; 64];
        assert_eq!(
            engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut buf),
            Outcome::Received(payload.len())
        );
        assert_eq!(&buf[..payload.len()], &payload);
        assert_eq!(port.sent[0], h.get(Pid::Ack));
    }

    #[test]
    fn an_isochronous_out_is_received_and_not_acked() {
        let payload = [1u8, 2, 3, 4];
        let mut data = [0u8; 16];
        let n = build_data(Pid::Data0, &payload, &mut data).unwrap();
        let mut bytes = token_bytes(Pid::Out, ADDR, 2);
        bytes.extend_from_slice(&data[..n]);

        let mut port = Fake::new(&bytes);
        let h = Handshakes::new();
        let mut buf = [0u8; 64];
        assert_eq!(
            engine().handle(&mut port, &h, one(EpType::Isochronous, true), no_packet, &mut buf),
            Outcome::Received(payload.len())
        );
        assert!(port.sent.is_empty(), "isochronous acknowledges nothing");
    }

    #[test]
    fn a_corrupt_data_packet_is_not_acknowledged() {
        let payload = [1u8, 2, 3, 4];
        let mut data = [0u8; 16];
        let n = build_data(Pid::Data0, &payload, &mut data).unwrap();
        data[3] ^= 0x01;

        let mut bytes = token_bytes(Pid::Out, ADDR, 2);
        bytes.extend_from_slice(&data[..n]);

        let mut port = Fake::new(&bytes);
        let h = Handshakes::new();
        assert_eq!(
            engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 64]),
            Outcome::Corrupt
        );
        assert!(
            port.sent.is_empty(),
            "acknowledging damage is worse than silence — the host must retry"
        );
    }

    #[test]
    fn a_setup_is_accepted_even_on_a_halted_endpoint() {
        let mut data = [0u8; 16];
        // A SETUP always carries eight bytes.
        let n = build_data(Pid::Data0, &[0u8; 8], &mut data).unwrap();
        let mut bytes = token_bytes(Pid::Setup, ADDR, 0);
        bytes.extend_from_slice(&data[..n]);

        let mut port = Fake::new(&bytes);
        let h = Handshakes::new();
        let halted = |_: u8, _: bool| {
            Some(EpState {
                stalled: true,
                ..EpState::new(EpType::Control)
            })
        };
        assert_eq!(
            engine().handle(&mut port, &h, halted, no_packet, &mut [0u8; 64]),
            Outcome::Received(8)
        );
        assert_eq!(port.sent[0], h.get(Pid::Ack), "that is how a halt is cleared");
    }

    #[test]
    fn a_bus_that_goes_quiet_mid_token_times_out_rather_than_wedging() {
        let mut port = Fake::new(&[SYNC, Pid::In.to_byte()]);
        let h = Handshakes::new();
        assert_eq!(
            engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]),
            Outcome::Timeout
        );
        assert!(port.sent.is_empty());
    }

    #[test]
    fn address_zero_answers_before_set_address() {
        let mut e = Engine::new();
        assert_eq!(e.address, 0);
        let mut port = Fake::new(&token_bytes(Pid::In, 0, 0));
        let h = Handshakes::new();
        assert_eq!(
            e.handle(&mut port, &h, one(EpType::Control, false), no_packet, &mut [0u8; 8]),
            Outcome::Handshake(Pid::Nak)
        );
        // And the same token is not ours once an address has been assigned.
        e.address = ADDR;
        let mut port = Fake::new(&token_bytes(Pid::In, 0, 0));
        assert_eq!(
            e.handle(&mut port, &h, one(EpType::Control, false), no_packet, &mut [0u8; 8]),
            Outcome::Ignored
        );
    }

    #[test]
    fn handshakes_are_never_sent_for_a_sof() {
        // Regression guard: an early version treated SOF as a token, decoded it
        // as an address, and could answer one.
        let mut port = Fake::new(&[SYNC, Pid::Sof.to_byte(), 0, 0]);
        let h = Handshakes::new();
        let out = engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]);
        assert!(matches!(out, Outcome::Sof(_) | Outcome::BadToken));
        assert!(port.sent.is_empty());
    }
}

// SPDX-License-Identifier: MIT
//! The endpoint table: state both the interrupt handler and the application
//! touch.
//!
//! # No locking in here
//!
//! Every method below assumes the caller has already excluded the other side.
//! That is deliberate. Locking primitives are hard to test on a development
//! machine — `critical_section` needs a platform implementation — and a table
//! that cannot be tested is the last place to put the handoff between an
//! interrupt and an async task.
//!
//! So this is plain data with plain methods, exercised by host tests, and the
//! target wraps the whole thing in one critical section. The rule is simply
//! that no method here may be called from both contexts at once.
//!
//! # Who owns a buffer, and when
//!
//! Transmission is fire-and-forget: [`super::hw::HwPort`] arms a DMA and
//! returns, so a packet's bytes must stay put until it has gone out. That rules
//! out lending the application's slice to the hardware. Instead every endpoint
//! owns its buffer for the life of the driver, the application copies into it,
//! and `ready` says whose it is:
//!
//! * `ready == false` — the application's. It may fill the buffer.
//! * `ready == true` — the interrupt's. It may transmit from the buffer.
//!
//! One flag, one direction of travel each way, and no moment where both sides
//! believe they hold it.

use super::device::{EpState, EpType};
use super::endpoint::{words_for_payload, PacketBuf};
use super::packet::Pid;

/// The largest packet any endpoint carries.
///
/// Sized for the car-facing audio endpoint's 196-byte maximum — 48 frames of
/// 16-bit stereo plus room for a 49-frame correction. Every endpoint gets the
/// same, which wastes a little on the control endpoint's 64 and is not worth
/// the const-generic machinery to avoid.
///
/// It is not free, though. A slot is about **2.7 KB** — the assembled bytes,
/// the packed symbols, the expanded symbol words, and the receive buffer — so
/// a `Table<N>` costs roughly `N * 5.5 KB` across both directions. Four
/// endpoint numbers is 22 KB, eight is 43 KB. Comfortable on 512 KB, but worth
/// knowing before declaring a table larger than the device needs.
pub const MAX_PACKET: usize = 196;

const RAW: usize = 2 + MAX_PACKET + 2;
const WORDS: usize = words_for_payload(MAX_PACKET);

/// One endpoint's buffers and state.
pub struct Slot {
    pub state: EpState,
    /// Encoded outgoing packet. Meaningful only for an IN endpoint.
    tx: PacketBuf<RAW, WORDS>,
    /// Most recent incoming payload. Meaningful only for an OUT endpoint.
    rx: [u8; MAX_PACKET],
    rx_len: usize,
    /// Whether the endpoint is configured at all. An unallocated slot must be
    /// invisible, not merely idle — a token for an endpoint we do not have is
    /// ignored, and answering it would be wrong.
    pub allocated: bool,
    /// Set by the interrupt when a transfer completes, cleared by the
    /// application when it notices. What a waker is armed against.
    pub complete: bool,
    /// Whether the host has selected this endpoint. An alternate setting that
    /// is not current must not answer, or a device with a zero-bandwidth alt 0
    /// would stream audio the host never asked for.
    pub enabled: bool,
    /// Whether the packet in `rx` arrived on a SETUP token rather than an OUT.
    /// The control pipe needs to tell them apart, and the PID is gone by then.
    pub was_setup: bool,
}

impl Default for Slot {
    fn default() -> Self {
        Self::new()
    }
}

impl Slot {
    pub const fn new() -> Self {
        Self {
            state: EpState::new(EpType::Bulk),
            tx: PacketBuf::new(),
            rx: [0; MAX_PACKET],
            rx_len: 0,
            allocated: false,
            complete: false,
            enabled: true,
            was_setup: false,
        }
    }

    /// Application side: fill the outgoing buffer and hand it to the hardware.
    ///
    /// Returns `false` if the previous packet has not gone yet — the caller
    /// must wait rather than overwrite it, because the DMA may be reading it
    /// right now.
    pub fn queue(&mut self, payload: &[u8]) -> bool {
        if self.state.ready || payload.len() > MAX_PACKET {
            return false;
        }
        let pid = self.state.data_pid();
        if self.tx.prepare(pid, payload).is_none() {
            return false;
        }
        self.complete = false;
        self.state.ready = true;
        true
    }

    /// Interrupt side: the encoded packet to transmit, if there is one.
    pub fn packet(&self) -> Option<&[u32]> {
        self.state.ready.then(|| self.tx.symbols())
    }

    /// Interrupt side: a transmitted packet is finished with.
    ///
    /// `acked` is whether the host confirmed it. The data toggle only advances
    /// on a confirmed packet — an unacknowledged one will be sent again with
    /// the same PID, which is how the host tells a retransmission from a new
    /// packet. Isochronous passes `false` and does not toggle at all.
    pub fn sent(&mut self, acked: bool) {
        self.state.ready = false;
        self.complete = true;
        if acked && self.state.ep_type.toggles_data_pid() {
            self.state.data1 = !self.state.data1;
        }
    }

    /// Interrupt side: store a received payload.
    ///
    /// Returns `false` if the previous one has not been collected. The packet
    /// is dropped in that case, which for isochronous is the correct outcome
    /// and for everything else means the endpoint should have been NAKing.
    pub fn deliver(&mut self, payload: &[u8]) -> bool {
        if self.complete || payload.len() > MAX_PACKET {
            return false;
        }
        self.rx[..payload.len()].copy_from_slice(payload);
        self.rx_len = payload.len();
        self.complete = true;
        true
    }

    /// Application side: collect a received payload, freeing the buffer.
    pub fn take(&mut self, out: &mut [u8]) -> Option<usize> {
        if !self.complete {
            return None;
        }
        let n = self.rx_len.min(out.len());
        out[..n].copy_from_slice(&self.rx[..n]);
        self.complete = false;
        // An OUT endpoint is "ready" when it has somewhere to put the next
        // packet, which is now.
        self.state.ready = true;
        Some(n)
    }
}

/// Every endpoint, indexed by number and direction.
///
/// `N` is the number of endpoint *numbers*; each has an IN and an OUT slot,
/// which USB treats as independent endpoints that merely share a number.
pub struct Table<const N: usize> {
    ins: [Slot; N],
    outs: [Slot; N],
}

impl<const N: usize> Default for Table<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Table<N> {
    pub const fn new() -> Self {
        Self {
            ins: [const { Slot::new() }; N],
            outs: [const { Slot::new() }; N],
        }
    }

    pub fn slot(&self, ep: u8, is_in: bool) -> Option<&Slot> {
        let s = if is_in {
            self.ins.get(ep as usize)?
        } else {
            self.outs.get(ep as usize)?
        };
        s.allocated.then_some(s)
    }

    pub fn slot_mut(&mut self, ep: u8, is_in: bool) -> Option<&mut Slot> {
        let s = if is_in {
            self.ins.get_mut(ep as usize)?
        } else {
            self.outs.get_mut(ep as usize)?
        };
        s.allocated.then_some(s)
    }

    /// Configure an endpoint. Returns `false` if the number is out of range or
    /// already taken — silently reusing one would give two parts of the
    /// application the same buffer.
    pub fn allocate(&mut self, ep: u8, is_in: bool, ep_type: EpType) -> bool {
        let slots = if is_in { &mut self.ins } else { &mut self.outs };
        let Some(slot) = slots.get_mut(ep as usize) else {
            return false;
        };
        if slot.allocated {
            return false;
        }
        *slot = Slot::new();
        slot.allocated = true;
        slot.state = EpState::new(ep_type);
        // An OUT endpoint starts able to receive; an IN endpoint starts with
        // nothing to send.
        slot.state.ready = !is_in;
        true
    }

    /// The state the transaction engine asks about.
    /// The state the transaction engine asks about.
    ///
    /// A disabled endpoint is invisible, exactly like an unallocated one. That
    /// is what makes a zero-bandwidth alternate setting mean anything: the host
    /// selects alt 0 to stop the stream, and an endpoint that kept answering
    /// would keep streaming.
    pub fn state(&self, ep: u8, is_in: bool) -> Option<EpState> {
        self.slot(ep, is_in).filter(|s| s.enabled).map(|s| s.state)
    }
}

/// The PID a slot's next packet will carry. Exposed for diagnostics; the
/// transmit path reads it through [`Slot::queue`].
pub fn next_pid(slot: &Slot) -> Pid {
    slot.state.data_pid()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn table() -> Table<4> {
        Table::new()
    }

    #[test]
    fn an_unallocated_endpoint_is_invisible() {
        let t = table();
        assert!(t.state(1, true).is_none());
        assert!(t.slot(1, true).is_none());
    }

    #[test]
    fn in_and_out_with_the_same_number_are_separate_endpoints() {
        let mut t = table();
        assert!(t.allocate(1, true, EpType::Bulk));
        assert!(t.state(1, true).is_some());
        assert!(t.state(1, false).is_none(), "OUT 1 is a different endpoint");
        assert!(t.allocate(1, false, EpType::Bulk));
        assert!(t.state(1, false).is_some());
    }

    #[test]
    fn allocating_the_same_endpoint_twice_is_refused() {
        let mut t = table();
        assert!(t.allocate(2, true, EpType::Bulk));
        assert!(
            !t.allocate(2, true, EpType::Interrupt),
            "two owners would share one buffer"
        );
    }

    #[test]
    fn an_out_endpoint_starts_ready_and_an_in_endpoint_does_not() {
        let mut t = table();
        t.allocate(1, true, EpType::Bulk);
        t.allocate(1, false, EpType::Bulk);
        assert!(!t.state(1, true).unwrap().ready, "nothing to send yet");
        assert!(t.state(1, false).unwrap().ready, "somewhere to receive into");
    }

    #[test]
    fn queueing_hands_the_buffer_over_and_sending_hands_it_back() {
        let mut t = table();
        t.allocate(1, true, EpType::Bulk);
        let s = t.slot_mut(1, true).unwrap();

        assert!(s.packet().is_none(), "nothing to send before queueing");
        assert!(s.queue(&[1, 2, 3]));
        assert!(s.state.ready, "the buffer is the hardware's now");
        assert!(s.packet().is_some());

        assert!(
            !s.queue(&[4, 5, 6]),
            "overwriting a packet the DMA may be reading"
        );

        s.sent(true);
        assert!(!s.state.ready, "and back to the application");
        assert!(s.packet().is_none());
        assert!(s.queue(&[4, 5, 6]));
    }

    #[test]
    fn the_data_toggle_only_advances_on_an_ack() {
        let mut t = table();
        t.allocate(1, true, EpType::Bulk);
        let s = t.slot_mut(1, true).unwrap();

        assert_eq!(next_pid(s), Pid::Data0);
        s.queue(&[0]);
        s.sent(false);
        assert_eq!(
            next_pid(s),
            Pid::Data0,
            "an unconfirmed packet is sent again with the same PID"
        );
        s.queue(&[0]);
        s.sent(true);
        assert_eq!(next_pid(s), Pid::Data1);
    }

    #[test]
    fn isochronous_never_toggles() {
        let mut t = table();
        t.allocate(1, true, EpType::Isochronous);
        let s = t.slot_mut(1, true).unwrap();
        for _ in 0..4 {
            assert_eq!(next_pid(s), Pid::Data0);
            s.queue(&[0]);
            // Even told it was acknowledged, which cannot happen on an
            // isochronous endpoint but must not corrupt the PID if it did.
            s.sent(true);
        }
    }

    #[test]
    fn a_received_packet_is_held_until_it_is_collected() {
        let mut t = table();
        t.allocate(2, false, EpType::Bulk);
        let s = t.slot_mut(2, false).unwrap();

        assert!(s.deliver(&[9, 8, 7]));
        assert!(
            !s.deliver(&[1, 2, 3]),
            "the previous packet has not been read"
        );

        let mut buf = [0u8; 8];
        assert_eq!(s.take(&mut buf), Some(3));
        assert_eq!(&buf[..3], &[9, 8, 7]);
        assert_eq!(s.take(&mut buf), None, "and only once");
        assert!(s.deliver(&[1, 2, 3]), "now there is room again");
    }

    #[test]
    fn collecting_makes_the_endpoint_ready_again() {
        let mut t = table();
        t.allocate(2, false, EpType::Bulk);
        let s = t.slot_mut(2, false).unwrap();
        s.deliver(&[1]);
        s.state.ready = false; // as the engine would leave it
        let mut buf = [0u8; 4];
        s.take(&mut buf);
        assert!(s.state.ready, "there is somewhere to put the next packet");
    }

    #[test]
    fn an_oversized_payload_is_refused_rather_than_truncated() {
        let mut t = table();
        t.allocate(1, true, EpType::Bulk);
        let s = t.slot_mut(1, true).unwrap();
        assert!(!s.queue(&[0u8; MAX_PACKET + 1]));
        assert!(!s.state.ready, "and the endpoint is untouched");
    }

    #[test]
    fn the_audio_packet_size_fits() {
        let mut t = table();
        t.allocate(1, true, EpType::Isochronous);
        let s = t.slot_mut(1, true).unwrap();
        assert!(
            s.queue(&[0xFFu8; 192]),
            "192 bytes of all-ones is the real endpoint's worst case"
        );
    }
}

/// Run one transaction and record what it did.
///
/// The bridge between [`super::txn::Engine`], which knows the protocol, and
/// [`Table`], which owns the buffers. Everything here is bookkeeping the
/// interrupt does after the deadline has passed — the reply is already on the
/// wire by the time this updates anything.
///
/// The caller holds whatever lock keeps the application out; see the module
/// docs for why there is none inside.
pub fn service<P: super::txn::Port, const N: usize>(
    engine: &mut super::txn::Engine,
    port: &mut P,
    handshakes: &super::endpoint::Handshakes,
    table: &mut Table<N>,
    rx_buf: &mut [u8],
) -> super::txn::Outcome {
    use super::txn::Outcome;

    let outcome = {
        // Split the borrow so the engine can hold the table immutably through
        // both closures while it runs, and this can take it mutably after.
        let t = &*table;
        engine.handle(
            port,
            handshakes,
            |ep, is_in| t.state(ep, is_in),
            |ep| t.slot(ep, true).and_then(|s| s.packet()),
            rx_buf,
        )
    };

    match outcome {
        Outcome::Sent { ep, acked } => {
            if let Some(slot) = table.slot_mut(ep, true) {
                slot.sent(acked);
            }
        }
        Outcome::Received { ep, len, setup } => {
            if let Some(slot) = table.slot_mut(ep, false) {
                slot.was_setup = setup;
                // A packet the application has not collected yet is dropped.
                // For isochronous that is the correct outcome — a late frame is
                // a lost frame — and for anything else the endpoint should have
                // been NAKing, so it is a bug upstream rather than here.
                slot.deliver(&rx_buf[..len]);
                // The endpoint has nowhere to put the next one until the
                // application reads this, so stop accepting.
                slot.state.ready = false;
            }
        }
        // Nothing to record. A corrupt packet in particular must leave the
        // endpoint exactly as it was: the host will send it again, and a slot
        // that had advanced would put the retransmission in the wrong place.
        Outcome::Sof(_)
        | Outcome::Handshake(_)
        | Outcome::Corrupt
        | Outcome::Ignored { .. }
        | Outcome::BadToken { .. }
        | Outcome::Timeout => {}
    }
    outcome
}

#[cfg(test)]
mod service_tests {
    use super::*;
    use crate::pio_usb::endpoint::Handshakes;
    use crate::pio_usb::packet::{build_data, encode_token, Pid, SYNC};
    use crate::pio_usb::txn::{Engine, Outcome, Port};

    const ADDR: u8 = 7;

    struct Fake {
        rx: std::collections::VecDeque<u8>,
        sent: Vec<Vec<u32>>,
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
        fn tx_busy(&self) -> bool {
            false
        }
        fn reset_receiver(&mut self) {}
        fn clear_eop(&mut self) {}
    }

    fn fake(bytes: &[u8]) -> Fake {
        Fake {
            rx: bytes.iter().copied().collect(),
            sent: Vec::new(),
        }
    }

    fn token(pid: Pid, ep: u8) -> Vec<u8> {
        let [b0, b1] = encode_token(ADDR, ep);
        vec![SYNC, pid.to_byte(), b0, b1]
    }

    fn engine() -> Engine {
        Engine {
            address: ADDR,
            frame: 0,
        }
    }

    #[test]
    fn a_sent_packet_frees_the_endpoint_for_the_application() {
        let mut t: Table<4> = Table::new();
        t.allocate(1, true, EpType::Bulk);
        t.slot_mut(1, true).unwrap().queue(&[1, 2, 3]);

        let mut bytes = token(Pid::In, 1);
        bytes.extend_from_slice(&[SYNC, Pid::Ack.to_byte()]);
        let mut port = fake(&bytes);

        let out = service(
            &mut engine(),
            &mut port,
            &Handshakes::new(),
            &mut t,
            &mut [0u8; 64],
        );
        assert_eq!(out, Outcome::Sent { ep: 1, acked: true });

        let s = t.slot_mut(1, true).unwrap();
        assert!(!s.state.ready, "the buffer is the application's again");
        assert!(s.complete, "and it can see the transfer finished");
        assert_eq!(next_pid(s), Pid::Data1, "acknowledged, so the toggle moved");
    }

    #[test]
    fn an_unacknowledged_packet_stays_on_data0() {
        let mut t: Table<4> = Table::new();
        t.allocate(1, true, EpType::Bulk);
        t.slot_mut(1, true).unwrap().queue(&[1]);

        // No ACK behind the token.
        let mut port = fake(&token(Pid::In, 1));
        service(
            &mut engine(),
            &mut port,
            &Handshakes::new(),
            &mut t,
            &mut [0u8; 64],
        );
        assert_eq!(next_pid(t.slot(1, true).unwrap()), Pid::Data0);
    }

    #[test]
    fn a_received_packet_lands_in_the_right_slot() {
        let mut t: Table<4> = Table::new();
        t.allocate(2, false, EpType::Bulk);

        let payload = [0xAAu8, 0xBB, 0xCC];
        let mut data = [0u8; 16];
        let n = build_data(Pid::Data0, &payload, &mut data).unwrap();
        let mut bytes = token(Pid::Out, 2);
        bytes.extend_from_slice(&data[..n]);
        let mut port = fake(&bytes);

        let out = service(
            &mut engine(),
            &mut port,
            &Handshakes::new(),
            &mut t,
            &mut [0u8; 64],
        );
        assert_eq!(out, Outcome::Received { ep: 2, len: 3, setup: false });

        let s = t.slot_mut(2, false).unwrap();
        assert!(!s.state.ready, "nowhere to put another until this is read");
        let mut buf = [0u8; 8];
        assert_eq!(s.take(&mut buf), Some(3));
        assert_eq!(&buf[..3], &payload);
        assert!(s.state.ready, "and now there is");
    }

    /// The IN slot and the OUT slot with the same number are different
    /// endpoints, so a transfer must not be recorded against the wrong one.
    #[test]
    fn a_transfer_is_recorded_against_the_matching_direction() {
        let mut t: Table<4> = Table::new();
        t.allocate(1, true, EpType::Bulk);
        t.allocate(1, false, EpType::Bulk);
        t.slot_mut(1, true).unwrap().queue(&[1]);

        let mut bytes = token(Pid::In, 1);
        bytes.extend_from_slice(&[SYNC, Pid::Ack.to_byte()]);
        let mut port = fake(&bytes);
        service(
            &mut engine(),
            &mut port,
            &Handshakes::new(),
            &mut t,
            &mut [0u8; 64],
        );

        assert!(t.slot(1, true).unwrap().complete, "the IN endpoint sent");
        assert!(
            !t.slot(1, false).unwrap().complete,
            "the OUT endpoint did nothing"
        );
    }

    /// A corrupt packet must leave the endpoint untouched: the host will send
    /// it again, and a slot that had advanced would misplace the retry.
    #[test]
    fn a_corrupt_packet_changes_nothing() {
        let mut t: Table<4> = Table::new();
        t.allocate(2, false, EpType::Bulk);

        let mut data = [0u8; 16];
        let n = build_data(Pid::Data0, &[1, 2, 3], &mut data).unwrap();
        data[3] ^= 0x01;
        let mut bytes = token(Pid::Out, 2);
        bytes.extend_from_slice(&data[..n]);
        let mut port = fake(&bytes);

        let out = service(
            &mut engine(),
            &mut port,
            &Handshakes::new(),
            &mut t,
            &mut [0u8; 64],
        );
        assert_eq!(out, Outcome::Corrupt);
        let s = t.slot(2, false).unwrap();
        assert!(!s.complete, "nothing was delivered");
        assert!(s.state.ready, "and the endpoint still has room");
    }

    #[test]
    fn a_sof_touches_no_endpoint() {
        let mut t: Table<4> = Table::new();
        t.allocate(1, true, EpType::Bulk);
        t.slot_mut(1, true).unwrap().queue(&[1]);

        let frame = 0x123u16;
        let raw = frame | ((crate::pio_usb::codec::crc5(frame) as u16) << 11);
        let mut port = fake(&[SYNC, Pid::Sof.to_byte(), raw as u8, (raw >> 8) as u8]);
        let mut e = engine();
        let out = service(&mut e, &mut port, &Handshakes::new(), &mut t, &mut [0u8; 64]);

        assert_eq!(out, Outcome::Sof(frame));
        assert_eq!(e.frame, frame);
        assert!(
            t.slot(1, true).unwrap().state.ready,
            "the queued packet is still queued"
        );
    }
}

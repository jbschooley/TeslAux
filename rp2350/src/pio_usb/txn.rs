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
    /// Take the bus and transmit an already-encoded packet. Returns as soon
    /// as the transfer is armed, not when it has finished.
    fn transmit(&mut self, symbols: &[u32]);
    /// Whether a transmission is still going out.
    fn tx_busy(&self) -> bool;
    /// Put the receiver back to waiting for a start of packet.
    ///
    /// Needed after transmitting: the receiver watches the same two wires the
    /// transmitter drives, so without this it decodes the device's own packet
    /// and is out of step by the time the host answers.
    fn reset_receiver(&mut self);
    /// Clear the end-of-packet marker.
    ///
    /// A SETUP is a token immediately followed by its DATA0. With a
    /// non-blocking EOP the token's end-of-packet flag is still set when the
    /// data stage begins, and the data reader would stop on it before a single
    /// byte arrived. Clearing it between the two stages is what lets a
    /// two-packet transaction be read as one.
    fn clear_eop(&mut self);
    /// Realign the decoder to the next packet's byte boundary — its shift
    /// counter and NRZI state only, not the edge detector. Light enough to run
    /// in the ~167 ns gap before a SETUP's data packet, which the full reset is
    /// not. Mirrors the reference's `restart_usb_receiver`.
    fn realign_decoder(&mut self);
}

/// How a transaction ended.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Outcome {
    /// A start of frame, carrying its frame number. What the bridge paces
    /// against, and the only "transaction" with no endpoint.
    Sof(u16),
    /// A data packet went out. `acked` says whether the host acknowledged it,
    /// and is always false for isochronous, which has no handshake to wait
    /// for. Only an acknowledged packet may advance the data toggle.
    ///
    /// Carries the endpoint number because the caller has to record the result
    /// against the right slot, and by the time it sees this the token is gone.
    Sent { ep: u8, acked: bool },
    /// A handshake went out and nothing else.
    Handshake(Pid),
    /// A data packet arrived and was accepted, with this many payload bytes
    /// now in the caller's buffer.
    ///
    /// `setup` distinguishes a SETUP from an OUT. The control pipe has to tell
    /// them apart and the PID is gone by the time it looks.
    Received { ep: u8, len: usize, setup: bool },
    /// A data packet arrived and its CRC16 failed, so nothing was
    /// acknowledged. The host will retry; an isochronous host will not, and
    /// that is the trade.
    Corrupt,
    /// A well-formed token this device did not act on: addressed elsewhere, an
    /// endpoint we do not have, a PID that is not a token, or an isochronous
    /// endpoint with nothing to send.
    ///
    /// Carries the token because "ignored" covers several situations that need
    /// different fixes, and a bare counter cannot tell them apart. A host
    /// sending SETUPs we discard and a host sending nothing look identical
    /// until this says which.
    Ignored { pid: u8, address: u8, endpoint: u8 },
    /// The token did not survive its own checks — bad SYNC, bad PID check
    /// nibble, or a failed CRC5. Answering it would mean replying on another
    /// device's behalf.
    ///
    /// Carries the four bytes read, because what they are matters. A mangled
    /// token and a *data* packet mistaken for one look identical in a counter
    /// and mean completely different things: the first is a decode fault, the
    /// second means the receiver joined a transaction late and is reading the
    /// data stage as if it were the start of a packet.
    BadToken { bytes: [u8; 4] },
    /// The bus went quiet mid-packet.
    Timeout,
}

/// Diagnostic counters, target-only so the engine stays pure under test.
///
/// They answer the question the outcome counters cannot: which token *types*
/// the host is actually sending us, decoded and by PID, before any of the
/// downstream success or failure. If SETUP never increments, the host is not
/// getting far enough to send one; if it does, the fault is what we do with it.
#[cfg(target_os = "none")]
pub mod trace {
    use core::sync::atomic::{AtomicU32, Ordering};
    pub static SETUP: AtomicU32 = AtomicU32::new(0);
    pub static IN: AtomicU32 = AtomicU32::new(0);
    pub static OUT: AtomicU32 = AtomicU32::new(0);
    pub static DATA: AtomicU32 = AtomicU32::new(0);
    /// The four bytes of the last non-SOF packet the engine looked at, latched.
    pub static LAST: AtomicU32 = AtomicU32::new(0);

    pub(super) fn token(pid: super::Pid) {
        let c = match pid {
            super::Pid::Setup => &SETUP,
            super::Pid::In => &IN,
            super::Pid::Out => &OUT,
            super::Pid::Data0 | super::Pid::Data1 => &DATA,
            _ => return,
        };
        c.fetch_add(1, Ordering::Relaxed);
    }
    pub(super) fn last(b: [u8; 4]) {
        LAST.store(u32::from_be_bytes(b), Ordering::Relaxed);
    }

    /// How the last data-stage read ended, and its shape: bytes read in the low
    /// byte, DataEnd tag in the next, and the first four bytes read above.
    pub static DATA_READ: AtomicU32 = AtomicU32::new(0);
    pub static DATA_HEAD: AtomicU32 = AtomicU32::new(0);

    pub(super) fn data_read(n: u8, head: [u8; 4], end: super::DataEnd) {
        DATA_READ.store((n as u32) | ((end as u32) << 8), Ordering::Relaxed);
        DATA_HEAD.store(u32::from_be_bytes(head), Ordering::Relaxed);
    }
}

/// How a data-stage read ended, for the trace.
#[cfg(target_os = "none")]
#[derive(Copy, Clone)]
pub enum DataEnd {
    Timeout = 0,
    BadCrc = 1,
    Ok = 2,
}

/// Spins to wait for one byte before giving up.
///
/// This runs with interrupts disabled, so the arithmetic matters. A byte is
/// eight bit times — 667 ns, about 64 cycles at 96 MHz — and the loop body is a
/// register read and a test, call it a dozen cycles. Five iterations covers a
/// byte; 128 covers twenty-five of them, roughly 17 µs.
///
/// The first version of this was 20,000, which is two to three milliseconds of
/// disabled interrupts every time a byte failed to arrive. With a host plugged
/// in sending a start of frame every millisecond, the CPU never left the
/// handler: the device stopped answering and the *other* USB port — the one
/// carrying the diagnostics — went dead with it. A timeout long enough to be
/// safe is not the same as one long enough to be harmless.
const BYTE_SPINS: u32 = 128;

/// Spins without progress before abandoning a packet mid-flight.
///
/// Bounded by inactivity rather than by total time, because a 192-byte packet
/// legitimately takes 128 µs and a fixed budget would either cut it short or be
/// far too generous for a bus that has simply gone quiet. The counter resets on
/// every byte, so this is "nothing for about 17 µs", the same as above.
const IDLE_SPINS: u32 = 128;

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

    /// An engine already answering to `addr`, for tests and harnesses that skip
    /// enumeration.
    pub const fn at_address(addr: u8) -> Self {
        Self {
            address: addr,
            frame: 0,
        }
    }

    /// Read one byte, spinning until it arrives or the bus goes quiet.
    fn byte<P: Port>(port: &mut P) -> Option<u8> {
        for _ in 0..BYTE_SPINS {
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
    ///
    /// **Leaves the receiver armed for the next packet**, whatever happened to
    /// this one — including the paths that give up early on a bad token or a
    /// silent bus. That is what makes this usable from an interrupt: a host
    /// sends when it likes, and there is no other moment at which the reset
    /// could be done. Until this, the receiver was reset by whichever code
    /// scheduled the packet, which only worked because the tests knew when one
    /// was coming.
    pub fn handle<'p, P: Port>(
        &mut self,
        port: &mut P,
        handshakes: &Handshakes,
        endpoint: impl Fn(u8, bool) -> Option<EpState>,
        packet: impl Fn(u8) -> Option<&'p [u32]>,
        rx_buf: &mut [u8],
    ) -> Outcome {
        let outcome = self.handle_inner(port, handshakes, endpoint, packet, rx_buf);
        // Unconditional: a transaction that ended badly is exactly when the
        // receiver is most likely to be mid-packet and least likely to recover
        // on its own.
        port.reset_receiver();
        outcome
    }

    #[allow(clippy::too_many_arguments)]
    fn handle_inner<'p, P: Port>(
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
        let (Some(pid_byte), Some(b0), Some(b1)) =
            (Self::byte(port), Self::byte(port), Self::byte(port))
        else {
            return Outcome::Timeout;
        };
        let seen = [sync, pid_byte, b0, b1];
        if sync != SYNC {
            return Outcome::BadToken { bytes: seen };
        }
        let Some(pid) = Pid::from_byte(pid_byte) else {
            return Outcome::BadToken { bytes: seen };
        };

        #[cfg(target_os = "none")]
        {
            if pid != Pid::Sof {
                trace::token(pid);
                trace::last(seen);
            }
        }

        if pid == Pid::Sof {
            return match decode_sof(b0, b1) {
                Some(frame) => {
                    self.frame = frame;
                    Outcome::Sof(frame)
                }
                None => Outcome::BadToken { bytes: seen },
            };
        }

        let Some(token) = decode_token(pid, b0, b1) else {
            return Outcome::BadToken { bytes: seen };
        };

        // --- decide ---
        let is_in = token.pid == Pid::In;
        let state = endpoint(token.endpoint, is_in);
        match respond(token, self.address, state.as_ref()) {
            // `Sof` cannot reach here — it returned above — but matching it
            // explicitly keeps this exhaustive, so a new variant becomes a
            // compile error rather than being swallowed by a catch-all.
            Response::Ignore | Response::Sof => Outcome::Ignored {
                pid: pid_byte,
                address: token.address,
                endpoint: token.endpoint,
            },

            Response::SendHandshake(pid) => {
                port.transmit(handshakes.get(pid));
                Outcome::Handshake(pid)
            }

            Response::SendData {
                expect_handshake, ..
            } => {
                // The packet was encoded when the application wrote it; this
                // only has to point the DMA at it. Returning "sent" without
                // sending would be a lie, so the lookup is a parameter rather
                // than the caller's job afterwards.
                let Some(symbols) = packet(token.endpoint) else {
                    // `respond` said the endpoint was ready and the buffer
                    // disagrees. Nothing safe can go on the wire, and a
                    // half-truth is worse than silence.
                    return Outcome::Ignored {
                        pid: pid_byte,
                        address: token.address,
                        endpoint: token.endpoint,
                    };
                };
                port.transmit(symbols);

                if !expect_handshake {
                    // Isochronous. The transaction is over when the packet is;
                    // there is nothing to wait for, and waiting would burn the
                    // budget for the next frame.
                    return Outcome::Sent {
                        ep: token.endpoint,
                        acked: false,
                    };
                }

                // Everything else: the host answers. The receiver has to be
                // put back first, because it has just decoded our own packet —
                // it watches the same wires the transmitter drives.
                while port.tx_busy() {}
                port.reset_receiver();
                let acked = match (Self::byte(port), Self::byte(port)) {
                    (Some(SYNC), Some(pid)) => Pid::from_byte(pid) == Some(Pid::Ack),
                    // No answer, or a malformed one. Not an error: the host is
                    // entitled to stay silent, and the packet simply has not
                    // been confirmed.
                    _ => false,
                };
                Outcome::Sent {
                    ep: token.endpoint,
                    acked,
                }
            }

            Response::Receive { reply } => {
                // A SETUP or OUT token is immediately followed by its DATA
                // packet, ~167 ns behind — far too close for the CPU to return
                // from this interrupt and catch the data on a fresh one. So the
                // data is read here, in the same pass, exactly as the reference
                // does: realign the decoder to the data's byte boundary, wait
                // for its start, and read it.
                //
                // The realign is deliberately light — the decoder's shift
                // counter and NRZI state, nothing more. The heavy reset that
                // also tears down the edge detector is far too slow to fit the
                // gap, and tearing down the detector would lose the data's start
                // of packet outright. This is the difference between the `01`
                // (near-byte) misalignment of an un-realigned read and a clean
                // one.
                // The realign is required to get the decoder emitting the data
                // to this inline reader at all — without it the read times out
                // with zero bytes. It does not fix alignment, though: the data
                // packet rides ~167 ns behind the token's EOP, too close for a
                // clean start-of-packet, so its SYNC decodes as 01 rather than
                // 80 every time. That is the back-to-back boundary problem the
                // `piopair` harness isolates, surfacing here as the SETUP data.
                port.realign_decoder();
                port.clear_eop();
                let n = match Self::receive(port, rx_buf) {
                    Some(n) => n,
                    None => {
                        #[cfg(target_os = "none")]
                        trace::data_read(0, [0; 4], DataEnd::Timeout);
                        return Outcome::Timeout;
                    }
                };
                #[cfg(target_os = "none")]
                {
                    let mut head = [0u8; 4];
                    for (i, b) in rx_buf.iter().take(4).enumerate() {
                        head[i] = *b;
                    }
                    let ok = check_data(&rx_buf[..n]).is_some();
                    trace::data_read(
                        n as u8,
                        head,
                        if ok { DataEnd::Ok } else { DataEnd::BadCrc },
                    );
                }
                let Some(payload) = check_data(&rx_buf[..n]) else {
                    return Outcome::Corrupt;
                };
                let len = payload.len();
                if let Some(pid) = reply {
                    port.transmit(handshakes.get(pid));
                }
                Outcome::Received {
                    ep: token.endpoint,
                    len,
                    setup: token.pid == Pid::Setup,
                }
            }
        }
    }

    /// Read a data packet's body — everything after its PID, including the two
    /// CRC bytes — until the receiver reports the end of the packet.
    fn receive<P: Port>(port: &mut P, buf: &mut [u8]) -> Option<usize> {
        // Read the whole packet as the decoder gives it — bytes and all,
        // including its SYNC and PID — into a scratch, then realign in
        // software.
        //
        // A data packet riding ~167 ns behind its token never gets a clean
        // start of packet, so the decoder's byte boundary is off and its SYNC
        // comes out as 01, not 80. Core 1 made that offset the *same* every
        // time, which is what makes a software fix sound: reconstruct the bit
        // stream, find where the SYNC really falls, and slide the byte boundary
        // there. See [`realign`]. The hardware fix — a self-aligning receiver —
        // is better but is a from-scratch PIO receiver; this gets enumeration
        // moving without it.
        let mut raw = [0u8; RX_SCRATCH];
        let mut n = 0;
        let mut idle = 0u32;
        while idle < IDLE_SPINS {
            let mut progressed = false;
            while let Some(b) = port.rx() {
                if n < raw.len() {
                    raw[n] = b;
                }
                n += 1;
                progressed = true;
            }
            idle = if progressed { 0 } else { idle + 1 };
            if port.eop() {
                while let Some(b) = port.rx() {
                    if n < raw.len() {
                        raw[n] = b;
                    }
                    n += 1;
                }
                break;
            }
        }
        if n == 0 || n > raw.len() {
            return None;
        }
        realign(&raw[..n], buf)
    }
}

/// Scratch for a raw, possibly-misaligned received packet before realignment.
const RX_SCRATCH: usize = 80;

/// Recover a data packet from a bit-misaligned byte stream.
///
/// The decoder assembles bytes on whatever boundary it was on when the packet
/// began; a data packet that came in right behind its token is off by a fixed
/// number of bits, so its SYNC (chronologically seven 0-bits then a 1) does not
/// land on a byte boundary. This rebuilds the chronological bit stream, tries
/// every bit offset, and keeps the one whose SYNC, PID and CRC16 all check out.
/// Writes the payload-plus-CRC (everything after SYNC and PID) into `out` and
/// returns its length, so the caller's `check_data` validates exactly as for an
/// aligned read.
///
/// When no offset validates, it falls back to the byte-aligned reading — SYNC
/// and PID as `raw[0]` and `raw[1]`, body from `raw[2]`. That is what a
/// correctly aligned but genuinely corrupt packet looks like, and the caller's
/// own `check_data` must see its body to report [`Outcome::Corrupt`]: a damaged
/// packet must not come back as an empty one, or the host is never told to
/// retry.
///
/// A decoder byte holds its eight bits least-significant-first — the earliest
/// bit is bit 0 — because the decoder shifts right. So the SYNC byte 0x80
/// (bit 7 set) is the chronological pattern 0,0,0,0,0,0,0,1.
fn realign(raw: &[u8], out: &mut [u8]) -> Option<usize> {
    let total_bits = raw.len() * 8;
    let bit = |i: usize| (raw[i / 8] >> (i % 8)) & 1;
    let byte_at = |o: usize| -> u8 {
        let mut b = 0u8;
        for k in 0..8 {
            b |= bit(o + k) << k;
        }
        b
    };

    // Search bit offsets for a SYNC, then confirm the packet at that offset.
    for off in 0..total_bits.saturating_sub(8) {
        if byte_at(off) != crate::pio_usb::packet::SYNC {
            continue;
        }
        // Bytes from just after the SYNC: PID, then payload and CRC.
        let start = off + 8;
        let avail = (total_bits - start) / 8;
        if avail < 3 {
            continue; // need at least PID + 2 CRC bytes
        }
        let body_len = avail - 1; // drop the PID
        if body_len > out.len() {
            continue;
        }
        for j in 0..body_len {
            out[j] = byte_at(start + 8 + j * 8);
        }
        if check_data(&out[..body_len]).is_some() {
            return Some(body_len);
        }
    }

    // Nothing validated. Hand back the byte-aligned body so the caller reports
    // Corrupt rather than Timeout.
    let body = raw.get(2..)?;
    if body.is_empty() || body.len() > out.len() {
        return None;
    }
    out[..body.len()].copy_from_slice(body);
    Some(body.len())
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
        resets: usize,
    }

    impl Fake {
        fn new(bytes: &[u8]) -> Self {
            Self {
                rx: bytes.iter().copied().collect(),
                sent: Vec::new(),
                resets: 0,
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
        fn tx_busy(&self) -> bool {
            // The fake transmits instantly; nothing here is about timing.
            false
        }
        fn reset_receiver(&mut self) {
            self.resets += 1;
        }
        fn clear_eop(&mut self) {}
        fn realign_decoder(&mut self) {}
    }

    const ADDR: u8 = 7;

    fn token_bytes(pid: Pid, address: u8, ep: u8) -> Vec<u8> {
        let [b0, b1] = encode_token(address, ep);
        vec![SYNC, pid.to_byte(), b0, b1]
    }

    fn engine() -> Engine {
        Engine::at_address(ADDR)
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
        assert!(matches!(
            engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]),
            Outcome::BadToken { .. }
        ));
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
        assert!(matches!(
            engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]),
            Outcome::BadToken { .. }
        ));
        assert!(port.sent.is_empty(), "silence, not a NAK");
    }

    #[test]
    fn a_token_for_another_device_is_ignored_silently() {
        let mut port = Fake::new(&token_bytes(Pid::In, ADDR + 1, 1));
        let h = Handshakes::new();
        assert!(matches!(
            engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]),
            Outcome::Ignored { .. }
        ));
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
            Outcome::Sent { ep: 1, acked: false }
        );
        assert_eq!(port.sent.len(), 1, "exactly one packet on the wire");
        assert_eq!(port.sent[0], expected, "and it is the prepared one");
    }

    /// A non-isochronous data packet is only confirmed when the host answers,
    /// and only a confirmed packet may advance the data toggle. This was the
    /// gap that made bulk behave exactly like isochronous — the wire captures
    /// for the two cases were identical, which is how it was noticed.
    #[test]
    fn a_bulk_in_waits_for_the_hosts_ack() {
        use crate::pio_usb::endpoint::{words_for_payload, PacketBuf};
        let mut buf: PacketBuf<12, { words_for_payload(8) }> = PacketBuf::new();
        buf.prepare(Pid::Data0, &[0u8; 8]).unwrap();

        // Token, then the host's ACK waiting behind it.
        let mut bytes = token_bytes(Pid::In, ADDR, 1);
        bytes.extend_from_slice(&[SYNC, Pid::Ack.to_byte()]);

        let mut port = Fake::new(&bytes);
        let h = Handshakes::new();
        assert_eq!(
            engine().handle(
                &mut port,
                &h,
                one(EpType::Bulk, true),
                |_| Some(buf.symbols()),
                &mut [0u8; 8]
            ),
            Outcome::Sent { ep: 1, acked: true }
        );
        // One to listen for the ACK, one to re-arm for the next packet.
        assert_eq!(port.resets, 2, "the receiver must be put back before waiting");
    }

    #[test]
    fn a_bulk_in_the_host_never_answers_is_not_acked() {
        use crate::pio_usb::endpoint::{words_for_payload, PacketBuf};
        let mut buf: PacketBuf<12, { words_for_payload(8) }> = PacketBuf::new();
        buf.prepare(Pid::Data0, &[0u8; 8]).unwrap();

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
            Outcome::Sent { ep: 1, acked: false },
            "silence is not an error, just an unconfirmed packet"
        );
    }

    /// Isochronous must not wait, and must not reset the receiver to do so:
    /// the next frame's budget is the thing being protected.
    #[test]
    fn an_isochronous_in_does_not_wait_for_anything() {
        use crate::pio_usb::endpoint::{words_for_payload, PacketBuf};
        let mut buf: PacketBuf<12, { words_for_payload(8) }> = PacketBuf::new();
        buf.prepare(Pid::Data0, &[0u8; 8]).unwrap();

        // An ACK is sitting there; isochronous must ignore it entirely.
        let mut bytes = token_bytes(Pid::In, ADDR, 1);
        bytes.extend_from_slice(&[SYNC, Pid::Ack.to_byte()]);

        let mut port = Fake::new(&bytes);
        let h = Handshakes::new();
        assert_eq!(
            engine().handle(
                &mut port,
                &h,
                one(EpType::Isochronous, true),
                |_| Some(buf.symbols()),
                &mut [0u8; 8]
            ),
            Outcome::Sent { ep: 1, acked: false }
        );
        // Only the re-arm at the end: isochronous never stops to listen.
        assert_eq!(port.resets, 1, "no wait means no extra reset");
    }

    /// If `respond` says ready but no buffer turns up, nothing goes out. A
    /// half-formed packet on the wire is worse than a missed frame.
    #[test]
    fn an_in_with_no_prepared_buffer_stays_silent() {
        let mut port = Fake::new(&token_bytes(Pid::In, ADDR, 1));
        let h = Handshakes::new();
        assert!(matches!(
            engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]),
            Outcome::Ignored { .. }
        ));
        assert!(port.sent.is_empty());
    }

    /// The isochronous rule, at the level that actually puts bits on the wire.
    #[test]
    fn an_isochronous_in_with_no_data_says_nothing() {
        let mut port = Fake::new(&token_bytes(Pid::In, ADDR, 1));
        let h = Handshakes::new();
        assert!(matches!(
            engine().handle(&mut port, &h, one(EpType::Isochronous, false), no_packet, &mut [0u8; 8]),
            Outcome::Ignored { .. }
        ));
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
            Outcome::Received { ep: 2, len: payload.len(), setup: false }
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
            Outcome::Received { ep: 2, len: payload.len(), setup: false }
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

    /// The reason `realign` exists: a data packet that came in behind its token
    /// is byte-misaligned by a fixed number of bits, its SYNC no longer on a
    /// byte boundary. Rebuild that: take a real packet's chronological bit
    /// stream, insert `shift` leading bits (as a late start-of-packet does),
    /// repack, and confirm `realign` slides the boundary back and recovers the
    /// exact body `check_data` accepts.
    #[test]
    fn realign_recovers_a_bit_shifted_packet() {
        let payload = [0x12u8, 0x34, 0x56, 0x78, 0x9a];
        let mut data = [0u8; 16];
        let n = build_data(Pid::Data0, &payload, &mut data).unwrap();

        // The decoder packs bits least-significant-first (it shifts right), so
        // bit i of the stream is raw[i/8] >> (i%8). Expand to a flat bit vector.
        let mut bits = [0u8; 16 * 8];
        for i in 0..n * 8 {
            bits[i] = (data[i / 8] >> (i % 8)) & 1;
        }

        for shift in 1..8usize {
            // Prepend `shift` idle-ish bits, then the packet's bits, and repack
            // into bytes the same LSB-first way.
            let mut shifted = [0u8; 20];
            let total = shift + n * 8;
            for i in 0..n * 8 {
                if bits[i] == 1 {
                    shifted[(shift + i) / 8] |= 1 << ((shift + i) % 8);
                }
            }
            let raw_len = total.div_ceil(8);

            let mut out = [0u8; 64];
            let len = realign(&shifted[..raw_len], &mut out)
                .unwrap_or_else(|| panic!("shift {shift} did not realign"));
            let recovered = check_data(&out[..len])
                .unwrap_or_else(|| panic!("shift {shift} realigned but failed CRC"));
            assert_eq!(recovered, &payload, "shift {shift}");
        }
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
            Outcome::Received { ep: 0, len: 8, setup: true }
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
        assert!(matches!(
            e.handle(&mut port, &h, one(EpType::Control, false), no_packet, &mut [0u8; 8]),
            Outcome::Ignored { .. }
        ));
    }

    /// The receiver has to be armed again however the transaction ended. A
    /// transaction that failed is exactly when the receiver is most likely to
    /// be mid-packet, and there is no later moment to do it — a host sends when
    /// it likes.
    #[test]
    fn every_ending_leaves_the_receiver_armed() {
        let h = Handshakes::new();

        // Bad sync.
        let mut bytes = token_bytes(Pid::In, ADDR, 1);
        bytes[0] = 0x81;
        let mut port = Fake::new(&bytes);
        engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]);
        assert_eq!(port.resets, 1, "bad sync");

        // Failed CRC5.
        let mut bytes = token_bytes(Pid::In, ADDR, 1);
        bytes[2] ^= 0x01;
        let mut port = Fake::new(&bytes);
        engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]);
        assert_eq!(port.resets, 1, "bad crc5");

        // A bus that went quiet mid-token.
        let mut port = Fake::new(&[SYNC, Pid::In.to_byte()]);
        engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]);
        assert_eq!(port.resets, 1, "timeout");

        // Someone else's token.
        let mut port = Fake::new(&token_bytes(Pid::In, ADDR + 1, 1));
        engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]);
        assert_eq!(port.resets, 1, "another device");

        // A SOF.
        let mut port = Fake::new(&[SYNC, Pid::Sof.to_byte(), 0, 0]);
        engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]);
        assert_eq!(port.resets, 1, "sof");
    }

    #[test]
    fn handshakes_are_never_sent_for_a_sof() {
        // Regression guard: an early version treated SOF as a token, decoded it
        // as an address, and could answer one.
        let mut port = Fake::new(&[SYNC, Pid::Sof.to_byte(), 0, 0]);
        let h = Handshakes::new();
        let out = engine().handle(&mut port, &h, one(EpType::Bulk, true), no_packet, &mut [0u8; 8]);
        assert!(matches!(out, Outcome::Sof(_) | Outcome::BadToken { .. }));
        assert!(port.sent.is_empty());
    }
}

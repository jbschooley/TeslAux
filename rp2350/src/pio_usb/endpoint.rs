// SPDX-License-Identifier: MIT
//! Endpoint buffers, holding packets already encoded for the wire.
//!
//! # Why packets are encoded early
//!
//! When an IN token arrives the device has between 2 and 7.5 bit times to
//! start replying — 167 to 625 ns, or 16 to 60 cycles at 96 MHz. Encoding a
//! 192-byte packet is a loop over sixteen hundred bits; it does not fit, and it
//! is not close.
//!
//! So the whole packet is built and NRZI-encoded when the application writes
//! it, and the interrupt handler only has to arm a DMA that is already pointed
//! at finished bytes. The reference does the same thing for the same reason
//! (`prepare_tx_data`, called from the transfer-start path rather than the IRQ).
//!
//! The handshakes are pre-encoded once at startup for the same reason: ACK, NAK
//! and STALL are two-byte packets, but two bytes is still an encode loop, and
//! it would be inside the deadline every single time.
//!
//! # Cost
//!
//! Symbols are stored one per word rather than packed four to a byte, because
//! that is the layout [`super::bus::dma_symbols`] needs and it is the layout
//! verified against a captured waveform. With the packed form kept alongside
//! it, a 192-byte audio endpoint costs about 2.5 KB. On a chip with 512 KB
//! that is a fair trade for a transmit path that cannot miss its deadline, but
//! it is worth knowing before allocating many of them.

use super::codec::{encode, SymbolBuf};
use super::packet::{build_data, build_handshake, Pid};

/// Worst-case symbol count for a packet of `raw` bytes on the wire.
///
/// Every bit becomes a symbol, bit stuffing adds one per six consecutive
/// ones, and the EOP adds an SE0, a release and up to three symbols of padding
/// to reach a word boundary. Sized for the worst case because a buffer that is
/// merely usually big enough fails on the one payload that is all ones.
pub const fn symbols_for(raw: usize) -> usize {
    let bits = raw * 8;
    // +6 rather than +5 so integer division cannot round the stuffing down.
    bits + (bits + 5) / 6 + 5
}

/// Symbol **words** needed for a payload of `payload` bytes, including SYNC,
/// PID and CRC16.
///
/// One word carries one symbol *byte*, which is four symbols — the packing the
/// transmitter's autopull expects. Confusing the two over-allocates fourfold.
pub const fn words_for_payload(payload: usize) -> usize {
    symbols_for(2 + payload + 2).div_ceil(4)
}

/// A packet buffer for one endpoint, holding both the assembled bytes and the
/// encoded symbols.
///
/// `RAW` is SYNC + PID + payload + CRC16; `WORDS` is [`words_for_payload`] for
/// the same payload size. They are separate parameters because Rust cannot yet
/// compute one from the other in a type position.
pub struct PacketBuf<const RAW: usize, const WORDS: usize> {
    raw: [u8; RAW],
    /// Symbols as the encoder writes them, four to a byte.
    packed: [u8; WORDS],
    /// The same symbols one per word, which is what the DMA sends.
    words: [u32; WORDS],
    /// Number of symbol words that are actually part of the packet.
    len: usize,
}

impl<const RAW: usize, const WORDS: usize> Default for PacketBuf<RAW, WORDS> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const RAW: usize, const WORDS: usize> PacketBuf<RAW, WORDS> {
    pub const fn new() -> Self {
        Self {
            raw: [0; RAW],
            packed: [0; WORDS],
            words: [0; WORDS],
            len: 0,
        }
    }

    /// Encode packet bytes that are already assembled.
    ///
    /// For packets that are not data packets — a token carries a CRC5 inside
    /// its two bytes rather than a trailing CRC16, so it cannot go through
    /// [`Self::prepare`]. The driver itself never sends one; this exists for
    /// test harnesses that have to impersonate a host.
    pub fn encode_raw(&mut self, raw: &[u8]) -> Option<usize> {
        if raw.len() > RAW {
            return None;
        }
        self.raw[..raw.len()].copy_from_slice(raw);
        self.encode(raw.len())
    }

    /// Assemble and encode a data packet. Returns the number of symbol words,
    /// or `None` if the payload does not fit.
    ///
    /// Everything expensive happens here, away from the interrupt handler.
    pub fn prepare(&mut self, pid: Pid, payload: &[u8]) -> Option<usize> {
        let n = build_data(pid, payload, &mut self.raw)?;
        self.encode(n)
    }

    /// Encode the first `n` bytes of `raw` into the symbol words.
    fn encode(&mut self, n: usize) -> Option<usize> {

        // Encode packed, then expand. An earlier version encoded into the
        // `words` storage and expanded in place from the back, which works but
        // needs an unaliased byte view of a `[u32]` — unsafe, and subtle enough
        // to need a proof, to save the quarter-kilobyte `packed` costs. Not a
        // trade worth making in something that ends up in a car.
        let sym_bytes = {
            let mut buf = SymbolBuf::new(&mut self.packed);
            encode(&self.raw[..n], &mut buf)?;
            buf.bytes()
        };
        if sym_bytes > WORDS {
            return None;
        }
        for i in 0..sym_bytes {
            self.words[i] = (self.packed[i] as u32) << 24;
        }
        self.len = sym_bytes;
        Some(sym_bytes)
    }

    /// The encoded packet, ready for [`super::bus::dma_symbols`].
    pub fn symbols(&self) -> &[u32] {
        &self.words[..self.len]
    }

    /// The assembled bytes, for tests and diagnostics.
    pub fn raw(&self) -> &[u8] {
        &self.raw
    }
}

/// The three handshake packets, encoded once so the interrupt path never has
/// to build one inside the turnaround window.
pub struct Handshakes {
    ack: [u32; HANDSHAKE_WORDS],
    nak: [u32; HANDSHAKE_WORDS],
    stall: [u32; HANDSHAKE_WORDS],
    len: usize,
}

/// A handshake is SYNC plus a PID: two bytes.
pub const HANDSHAKE_WORDS: usize = symbols_for(2);

impl Default for Handshakes {
    fn default() -> Self {
        Self::new()
    }
}

impl Handshakes {
    pub fn new() -> Self {
        let mut me = Self {
            ack: [0; HANDSHAKE_WORDS],
            nak: [0; HANDSHAKE_WORDS],
            stall: [0; HANDSHAKE_WORDS],
            len: 0,
        };
        for (pid, slot) in [
            (Pid::Ack, 0usize),
            (Pid::Nak, 1),
            (Pid::Stall, 2),
        ] {
            let raw = build_handshake(pid);
            let mut scratch = [0u8; HANDSHAKE_WORDS];
            let mut buf = SymbolBuf::new(&mut scratch);
            encode(&raw, &mut buf).expect("a handshake always fits");
            let n = buf.bytes();
            let dst = match slot {
                0 => &mut me.ack,
                1 => &mut me.nak,
                _ => &mut me.stall,
            };
            for i in 0..n {
                dst[i] = (scratch[i] as u32) << 24;
            }
            me.len = n;
        }
        me
    }

    /// The encoded form of one handshake.
    pub fn get(&self, pid: Pid) -> &[u32] {
        let src = match pid {
            Pid::Ack => &self.ack,
            Pid::Nak => &self.nak,
            Pid::Stall => &self.stall,
            // Nothing else is a handshake; answering with an ACK would be
            // wrong, so this is a programming error rather than a bus state.
            _ => panic!("not a handshake PID"),
        };
        &src[..self.len]
    }
}

/// The payload this project's car-facing endpoint carries.
///
/// 192 bytes: 48 frames of 16-bit stereo, which is the real TeslaMic's
/// wMaxPacketSize exactly — and three times what Pico-PIO-USB's buffers hold,
/// which is the reason this driver exists rather than a wrapper around it.
///
/// The shipping firmware declares 196 on the endpoint so a corrected packet of
/// 49 frames still fits; that is the endpoint's advertised maximum, not the
/// nominal payload, and it belongs with the descriptor rather than here.
pub const AUDIO_PAYLOAD: usize = 192;
/// Raw bytes for [`AUDIO_PAYLOAD`], including SYNC, PID and CRC16.
pub const AUDIO_RAW: usize = 2 + AUDIO_PAYLOAD + 2;
/// Symbol words for [`AUDIO_PAYLOAD`].
pub const AUDIO_WORDS: usize = words_for_payload(AUDIO_PAYLOAD);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pio_usb::packet::check_data;
    use crate::pio_usb::tx::Symbol;

    /// Decode a symbol-word stream back to bytes, from the USB spec's
    /// definition rather than by inverting the encoder.
    fn decode(words: &[u32]) -> Vec<u8> {
        let mut syms = Vec::new();
        for w in words {
            let byte = (w >> 24) as u8;
            for i in 0..4 {
                let shift = 6 - 2 * i;
                syms.push(match (byte >> shift) & 0b11 {
                    0 => Symbol::Se0,
                    1 => Symbol::K,
                    2 => Symbol::Release,
                    _ => Symbol::J,
                });
            }
        }
        let mut out = Vec::new();
        let (mut k, mut ones, mut acc, mut n) = (true, 0u8, 0u8, 0u8);
        for s in syms {
            let now = match s {
                Symbol::K => true,
                Symbol::J => false,
                _ => break,
            };
            if ones == 6 {
                k = now;
                ones = 0;
                continue;
            }
            let bit = u8::from(now == k);
            ones = if bit == 1 { ones + 1 } else { 0 };
            k = now;
            acc |= bit << n;
            n += 1;
            if n == 8 {
                out.push(acc);
                acc = 0;
                n = 0;
            }
        }
        out
    }

    #[test]
    fn a_prepared_packet_decodes_back_to_its_payload() {
        let payload: Vec<u8> = (0..192u32).map(|i| (i * 31 + 5) as u8).collect();
        let mut buf: PacketBuf<AUDIO_RAW, AUDIO_WORDS> = PacketBuf::new();
        let words = buf.prepare(Pid::Data0, &payload).expect("fits");
        assert!(words > 0);

        let bytes = decode(buf.symbols());
        assert_eq!(bytes[0], 0x80, "sync");
        assert_eq!(bytes[1], Pid::Data0.to_byte());
        assert_eq!(
            check_data(&bytes[2..2 + payload.len() + 2]),
            Some(&payload[..]),
            "payload and CRC survive the round trip"
        );
    }

    #[test]
    fn the_all_ones_payload_still_fits() {
        // The bit-stuffing worst case, and the reason `symbols_for` rounds up.
        let payload = [0xFFu8; 192];
        let mut buf: PacketBuf<AUDIO_RAW, AUDIO_WORDS> = PacketBuf::new();
        let words = buf.prepare(Pid::Data0, &payload).expect("worst case must fit");
        assert!(
            words <= AUDIO_WORDS,
            "{words} words exceeds the {AUDIO_WORDS} allocated"
        );
        assert_eq!(check_data(&decode(buf.symbols())[2..]), Some(&payload[..]));
    }

    #[test]
    fn a_zero_length_packet_is_fine() {
        // The status stage of a control transfer.
        let mut buf: PacketBuf<4, { words_for_payload(0) }> = PacketBuf::new();
        buf.prepare(Pid::Data1, &[]).expect("fits");
        let bytes = decode(buf.symbols());
        assert_eq!(bytes[1], Pid::Data1.to_byte());
        assert_eq!(check_data(&bytes[2..4]), Some(&[][..]));
    }

    #[test]
    fn an_oversized_payload_is_refused_not_truncated() {
        let mut buf: PacketBuf<8, { words_for_payload(4) }> = PacketBuf::new();
        assert!(buf.prepare(Pid::Data0, &[0u8; 64]).is_none());
    }

    /// Guards a bug this had: `words_for_payload` returned a symbol count
    /// where a symbol-*byte* count was wanted, over-allocating fourfold. Both
    /// halves matter — too small fails on the worst-case payload, too large is
    /// invisible except as wasted RAM, which is why it went unnoticed.
    #[test]
    fn the_audio_buffer_is_sized_for_the_worst_case_and_no_larger() {
        let payload = [0xFFu8; AUDIO_PAYLOAD];
        let mut buf: PacketBuf<AUDIO_RAW, AUDIO_WORDS> = PacketBuf::new();
        let used = buf.prepare(Pid::Data0, &payload).expect("worst case must fit");
        assert!(used <= AUDIO_WORDS, "{used} words needed, {AUDIO_WORDS} allocated");
        assert!(
            AUDIO_WORDS < used * 2,
            "{AUDIO_WORDS} allocated for {used} used — a symbol/symbol-byte mixup"
        );
    }

    #[test]
    fn handshakes_encode_and_decode() {
        let h = Handshakes::new();
        for pid in [Pid::Ack, Pid::Nak, Pid::Stall] {
            let bytes = decode(h.get(pid));
            assert_eq!(bytes[0], 0x80, "sync for {pid:?}");
            assert_eq!(bytes[1], pid.to_byte(), "{pid:?}");
        }
    }

    #[test]
    fn handshakes_are_distinct() {
        let h = Handshakes::new();
        assert_ne!(h.get(Pid::Ack), h.get(Pid::Nak));
        assert_ne!(h.get(Pid::Nak), h.get(Pid::Stall));
    }
}

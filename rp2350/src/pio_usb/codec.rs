// SPDX-License-Identifier: MIT
//! Line coding for the PIO USB port: NRZI, bit stuffing, and the two CRCs.
//!
//! All of this is pure logic with no hardware in it, which makes it the only
//! part of this driver that can be tested without a logic analyser. It is
//! therefore where the effort goes: everything below is exercised by the tests
//! at the bottom of this file, on the host, with `cargo test`.
//!
//! Ported from Pico-PIO-USB (MIT, © 2021-2024 sekigon-gonnoc) — specifically
//! `pio_usb_ll_encode_tx_data` and `usb_crc.c`.

use super::tx::Symbol;

/// USB bit stuffing: a zero is inserted after this many consecutive 1 bits, to
/// guarantee the receiver sees a transition often enough to stay in sync.
const STUFF_AFTER: u8 = 6;

/// Packs [`Symbol`]s four to a byte, most significant pair first.
///
/// This is the exact layout the TX state machine's `out pc, 2` consumes with an
/// 8-bit left-shifting autopull, so the packing is wire format.
pub struct SymbolBuf<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> SymbolBuf<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, len: 0 }
    }

    /// Append one symbol. Returns `false` if the buffer is full, in which case
    /// nothing is written — a truncated packet is never emitted.
    pub fn push(&mut self, s: Symbol) -> bool {
        let byte = self.len >> 2;
        if byte >= self.buf.len() {
            return false;
        }
        // Shift left first, matching the reference encoder: the buffer is built
        // MSB-pair first, so the symbol written first is the one the state
        // machine pulls first.
        if self.len % 4 == 0 {
            self.buf[byte] = 0;
        }
        self.buf[byte] = (self.buf[byte] << 2) | (s as u8);
        self.len += 1;
        true
    }

    /// Number of symbols written.
    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Number of whole bytes the DMA should send.
    ///
    /// [`encode`] always pads the tail to a symbol-quad boundary, so this is
    /// exact rather than a round-up that would emit stale symbols.
    pub fn bytes(&self) -> usize {
        self.len >> 2
    }
}

/// NRZI-encode `data` with bit stuffing into `out`, appending the EOP.
///
/// `data` is the packet as it goes on the wire — SYNC, PID, payload, CRC —
/// already assembled. Returns the number of symbols written, or `None` if `out`
/// was too small.
///
/// NRZI here is the USB convention: a data **0** causes a transition, a data
/// **1** leaves the line alone. Bits go out least significant first.
pub fn encode(data: &[u8], out: &mut SymbolBuf) -> Option<usize> {
    // `k` tracks the current line state: true means the line is sitting in K.
    // The reference encoder calls this `current_state` and starts it at 1.
    let mut k = true;
    let mut ones = 0u8;

    for &byte in data {
        for b in 0..8 {
            if byte & (1 << b) != 0 {
                // A 1: no transition.
                if !out.push(if k { Symbol::K } else { Symbol::J }) {
                    return None;
                }
                ones += 1;
            } else {
                // A 0: transition.
                k = !k;
                if !out.push(if k { Symbol::K } else { Symbol::J }) {
                    return None;
                }
                ones = 0;
            }

            if ones == STUFF_AFTER {
                // Six 1s in a row: stuff a 0, which is a transition.
                k = !k;
                if !out.push(if k { Symbol::K } else { Symbol::J }) {
                    return None;
                }
                ones = 0;
            }
        }
    }

    // EOP: SE0, then release the drivers, then pad with K to fill the final
    // quad so the DMA never sends a partial byte.
    if !out.push(Symbol::Se0) || !out.push(Symbol::Release) {
        return None;
    }
    while out.len() % 4 != 0 {
        if !out.push(Symbol::K) {
            return None;
        }
    }
    Some(out.len())
}

/// The USB CRC16 used on data packets, as it goes on the wire.
///
/// Polynomial 0x8005 reflected (0xA001), init 0xFFFF, final complement.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &byte in data {
        crc = crc16_update(crc, byte);
    }
    !crc
}

/// Fold one byte into a running CRC16. Exposed because the receive path checks
/// the CRC as bytes arrive, without buffering the packet first.
pub fn crc16_update(mut crc: u16, byte: u8) -> u16 {
    crc ^= byte as u16;
    for _ in 0..8 {
        crc = if crc & 1 != 0 { (crc >> 1) ^ 0xA001 } else { crc >> 1 };
    }
    crc
}

/// The USB CRC5 used on token packets, over the 11 bits of address and
/// endpoint. Polynomial 0x05 reflected (0x14), init 0x1F, final complement.
pub fn crc5(value: u16) -> u8 {
    let mut crc = 0x1Fu8;
    for i in 0..11 {
        let bit = ((value >> i) & 1) as u8;
        let top = (crc & 1) ^ bit;
        crc >>= 1;
        if top != 0 {
            crc ^= 0x14;
        }
    }
    (!crc) & 0x1F
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Decode a symbol stream back to bytes: NRZI, de-stuffing, LSB first.
    ///
    /// Written from the USB spec's definition rather than by inverting
    /// [`encode`], so a round trip is a real check and not the same mistake
    /// made twice. Stops at the EOP.
    fn decode(syms: &[Symbol]) -> Vec<u8> {
        let mut out = Vec::new();
        let mut k = true;
        let mut ones = 0u8;
        let mut acc = 0u8;
        let mut n = 0u8;
        for &s in syms {
            let now = match s {
                Symbol::K => true,
                Symbol::J => false,
                _ => break, // SE0 / Release: end of packet
            };
            if ones == STUFF_AFTER {
                // This symbol is a stuffed zero. Consume it and drop it.
                assert_ne!(now, k, "a stuffed bit must be a transition");
                k = now;
                ones = 0;
                continue;
            }
            let bit = u8::from(now == k);
            if bit == 1 {
                ones += 1;
            } else {
                ones = 0;
            }
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

    /// Unpack the quad-packed buffer the way the state machine's autopull does.
    fn symbols(buf: &[u8], len: usize) -> Vec<Symbol> {
        (0..len)
            .map(|i| {
                let byte = buf[i >> 2];
                let shift = 6 - 2 * (i % 4);
                match (byte >> shift) & 0b11 {
                    0 => Symbol::Se0,
                    1 => Symbol::K,
                    2 => Symbol::Release,
                    _ => Symbol::J,
                }
            })
            .collect()
    }

    fn round_trip(data: &[u8]) {
        let mut raw = [0u8; 2048];
        let mut buf = SymbolBuf::new(&mut raw);
        let n = encode(data, &mut buf).expect("buffer large enough");
        assert_eq!(n % 4, 0, "encoder must pad to a whole byte");
        let got = decode(&symbols(&raw, n));
        assert_eq!(&got[..data.len()], data, "round trip failed for {data:02x?}");
    }

    #[test]
    fn round_trips_a_sync_and_pid() {
        // SYNC (0x80) then a DATA0 PID, the start of every data packet.
        round_trip(&[0x80, 0xC3]);
    }

    #[test]
    fn round_trips_all_zeroes_and_all_ones() {
        // All 0xFF is the bit-stuffing worst case: a transition only ever
        // appears because one was stuffed in.
        round_trip(&[0x80, 0xC3, 0xFF, 0xFF, 0xFF, 0xFF]);
        round_trip(&[0x80, 0xC3, 0x00, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn round_trips_a_full_audio_packet() {
        // 192 bytes is this project's iso IN endpoint: 48 frames, 2 channels,
        // 16 bits. Exceeding 64 is the whole reason Pico-PIO-USB could not be
        // used as it stands.
        let mut pkt = vec![0x80, 0xC3];
        pkt.extend((0..192u32).map(|i| (i * 37) as u8));
        round_trip(&pkt);
    }

    /// The bus must never go more than seven bit times without an edge, which
    /// is what the receiver's clock recovery is specified against.
    ///
    /// Note the bound is **seven symbols, not six**, and the difference is not
    /// slack. The rule is six consecutive *1 bits*, and in NRZI a 1 is the
    /// absence of a transition. So the worst case is a 0 bit transitioning the
    /// line into a state (one symbol), six 1 bits holding it there (six more),
    /// and only then the stuffed 0 that forces an edge — seven identical
    /// symbols between two transitions. Asserting six here fails against a
    /// correct encoder, which is how this test was originally written.
    #[test]
    fn never_goes_more_than_seven_bit_times_without_an_edge() {
        let mut raw = [0u8; 2048];
        let mut buf = SymbolBuf::new(&mut raw);
        let n = encode(&[0x80, 0xC3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF], &mut buf).unwrap();
        let mut run = 0;
        let mut last = None;
        for s in symbols(&raw, n) {
            if !matches!(s, Symbol::K | Symbol::J) {
                break;
            }
            if Some(s) == last {
                run += 1;
                assert!(
                    run <= STUFF_AFTER + 1,
                    "bit stuffing failed: {run} bit times with no edge"
                );
            } else {
                run = 1;
            }
            last = Some(s);
        }
    }

    /// The invariant underneath the one above, stated on the data rather than
    /// the line: six 1 bits are always followed by a stuffed 0.
    #[test]
    fn stuffs_a_zero_after_six_ones() {
        // 0xFF repeated is nothing but 1 bits, so every seventh bit on the wire
        // must be a stuffed transition.
        let mut raw = [0u8; 2048];
        let mut buf = SymbolBuf::new(&mut raw);
        let n = encode(&[0xFF; 8], &mut buf).unwrap();
        let syms = symbols(&raw, n);
        // 64 data bits -> one stuffed bit per six -> 10 stuffed bits.
        let data_and_stuffed = syms
            .iter()
            .take_while(|s| matches!(s, Symbol::K | Symbol::J))
            .count();
        assert_eq!(data_and_stuffed, 64 + 64 / 6, "wrong number of stuffed bits");
    }

    #[test]
    fn encode_refuses_to_truncate() {
        let mut raw = [0u8; 4];
        let mut buf = SymbolBuf::new(&mut raw);
        assert!(encode(&[0u8; 64], &mut buf).is_none());
    }

    /// The optimised CRC16 must agree with the textbook bitwise definition.
    #[test]
    fn crc16_matches_the_bitwise_definition() {
        fn reference(data: &[u8]) -> u16 {
            let mut crc = 0xFFFFu16;
            for &byte in data {
                for b in 0..8 {
                    let bit = (byte >> b) & 1;
                    let top = ((crc & 1) as u8) ^ bit;
                    crc >>= 1;
                    if top != 0 {
                        crc ^= 0xA001;
                    }
                }
            }
            !crc
        }
        for len in 0..40usize {
            let data: Vec<u8> = (0..len).map(|i| (i * 91 + 7) as u8).collect();
            assert_eq!(crc16(&data), reference(&data), "len {len}");
        }
    }

    /// A receiver folds the received CRC back in and expects a fixed residue.
    /// That is the property the RX path will actually rely on.
    #[test]
    fn crc16_residue_is_constant() {
        let mut residues = std::collections::HashSet::new();
        for len in 1..40usize {
            let data: Vec<u8> = (0..len).map(|i| (i * 53 + 11) as u8).collect();
            let c = crc16(&data);
            let mut with_crc = data.clone();
            with_crc.push((c & 0xFF) as u8);
            with_crc.push((c >> 8) as u8);
            let mut crc = 0xFFFFu16;
            for &b in &with_crc {
                crc = crc16_update(crc, b);
            }
            residues.insert(crc);
        }
        assert_eq!(residues.len(), 1, "residue must not depend on the payload");
    }

    /// CRC5 is only ever computed over 11 bits, and the result must fit in 5.
    #[test]
    fn crc5_fits_in_five_bits() {
        for v in 0..=0x7FFu16 {
            assert!(crc5(v) <= 0x1F, "crc5({v:#x}) overflowed");
        }
    }

    /// A constant would pass the width check above while catching nothing.
    #[test]
    fn crc5_is_not_degenerate() {
        let distinct: std::collections::HashSet<u8> = (0..=0x7FFu16).map(crc5).collect();
        assert_eq!(distinct.len(), 32, "CRC5 should reach all 32 values");
    }
}

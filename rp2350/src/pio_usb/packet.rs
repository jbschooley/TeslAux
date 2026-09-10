// SPDX-License-Identifier: MIT
//! USB packet framing: PIDs, tokens, data packets and handshakes.
//!
//! The layer between [`super::codec`] (which knows about line states) and the
//! endpoint machinery (which knows about transfers). Like the codec this is
//! pure logic, so it is tested on the host — see the bottom of this file.
//!
//! Everything here is device-side. The port receives tokens and sends data and
//! handshakes; it never generates a token of its own, so token *encoding*
//! exists only to be tested against the decoder.

use super::codec::{crc16, crc5};

/// Every byte on the wire is preceded by this sync pattern.
///
/// On the wire it is 0x80 sent least-significant-bit first, which after NRZI
/// becomes the KJKJKJKK the receiver locks onto.
pub const SYNC: u8 = 0x80;

/// USB packet identifiers.
///
/// The discriminant is the 4-bit PID as it appears in the low nibble of the
/// packet's second byte. The high nibble on the wire is its complement, which
/// is what [`Pid::from_byte`] checks.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Pid {
    Out = 0b0001,
    In = 0b1001,
    Sof = 0b0101,
    Setup = 0b1101,
    Data0 = 0b0011,
    Data1 = 0b1011,
    Ack = 0b0010,
    Nak = 0b1010,
    Stall = 0b1110,
    Pre = 0b1100,
}

impl Pid {
    /// The full PID byte as transmitted: PID in the low nibble, its complement
    /// in the high nibble.
    pub const fn to_byte(self) -> u8 {
        let p = self as u8;
        p | ((!p & 0x0F) << 4)
    }

    /// Parse a received PID byte, rejecting anything whose check nibble is not
    /// the complement of its PID.
    ///
    /// That check is the only protection a token has beyond its CRC5, and a
    /// corrupted PID that still parses would be acted on as a different
    /// transaction entirely — so this is deliberately strict about unknown
    /// PIDs as well as malformed ones.
    pub fn from_byte(byte: u8) -> Option<Self> {
        if (byte >> 4) != (!byte & 0x0F) {
            return None;
        }
        Some(match byte & 0x0F {
            0b0001 => Pid::Out,
            0b1001 => Pid::In,
            0b0101 => Pid::Sof,
            0b1101 => Pid::Setup,
            0b0011 => Pid::Data0,
            0b1011 => Pid::Data1,
            0b0010 => Pid::Ack,
            0b1010 => Pid::Nak,
            0b1110 => Pid::Stall,
            0b1100 => Pid::Pre,
            _ => return None,
        })
    }

    /// Whether this PID introduces a data payload.
    pub const fn is_data(self) -> bool {
        matches!(self, Pid::Data0 | Pid::Data1)
    }

    /// Whether this PID introduces a token addressed to a device endpoint.
    pub const fn is_token(self) -> bool {
        matches!(self, Pid::Out | Pid::In | Pid::Setup)
    }
}

/// A decoded token: which device, which endpoint.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Token {
    pub pid: Pid,
    pub address: u8,
    pub endpoint: u8,
}

/// Decode the two bytes following a token PID.
///
/// Returns `None` if the CRC5 does not check out. A token that fails its CRC
/// must be ignored entirely rather than guessed at: acting on a corrupted
/// address means answering a token meant for another device on the bus.
pub fn decode_token(pid: Pid, b0: u8, b1: u8) -> Option<Token> {
    let raw = (b0 as u16) | ((b1 as u16) << 8);
    let fields = raw & 0x07FF;
    let received = ((raw >> 11) & 0x1F) as u8;
    if crc5(fields) != received {
        return None;
    }
    Some(Token {
        pid,
        address: (fields & 0x7F) as u8,
        endpoint: ((fields >> 7) & 0x0F) as u8,
    })
}

/// Encode a token's two payload bytes. Device-side this exists for the tests;
/// the port never transmits a token.
pub fn encode_token(address: u8, endpoint: u8) -> [u8; 2] {
    let fields = ((address as u16) & 0x7F) | (((endpoint as u16) & 0x0F) << 7);
    let raw = fields | ((crc5(fields) as u16) << 11);
    [raw as u8, (raw >> 8) as u8]
}

/// The frame number carried by a SOF token.
///
/// The 11 CRC-protected bits of a SOF are a frame counter rather than an
/// address and endpoint. This is what the bridge paces against — the same role
/// the car's SOF plays in the two-board build.
pub fn decode_sof(b0: u8, b1: u8) -> Option<u16> {
    let raw = (b0 as u16) | ((b1 as u16) << 8);
    let frame = raw & 0x07FF;
    if crc5(frame) != ((raw >> 11) & 0x1F) as u8 {
        return None;
    }
    Some(frame)
}

/// Build a data packet — SYNC, PID, payload, CRC16 — into `out`.
///
/// Returns the number of bytes written, or `None` if `out` is too small. This
/// is the buffer that then goes through [`super::codec::encode`].
pub fn build_data(pid: Pid, payload: &[u8], out: &mut [u8]) -> Option<usize> {
    debug_assert!(pid.is_data(), "build_data called with a non-data PID");
    let total = 2 + payload.len() + 2;
    if out.len() < total {
        return None;
    }
    out[0] = SYNC;
    out[1] = pid.to_byte();
    out[2..2 + payload.len()].copy_from_slice(payload);
    let crc = crc16(payload);
    out[2 + payload.len()] = crc as u8;
    out[3 + payload.len()] = (crc >> 8) as u8;
    Some(total)
}

/// Build a handshake packet — SYNC and a PID, nothing else.
pub fn build_handshake(pid: Pid) -> [u8; 2] {
    [SYNC, pid.to_byte()]
}

/// Check a received data packet and return its payload.
///
/// `body` is everything after the PID, including the two trailing CRC bytes.
pub fn check_data(body: &[u8]) -> Option<&[u8]> {
    if body.len() < 2 {
        return None;
    }
    let (payload, tail) = body.split_at(body.len() - 2);
    let received = (tail[0] as u16) | ((tail[1] as u16) << 8);
    (crc16(payload) == received).then_some(payload)
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [Pid; 10] = [
        Pid::Out,
        Pid::In,
        Pid::Sof,
        Pid::Setup,
        Pid::Data0,
        Pid::Data1,
        Pid::Ack,
        Pid::Nak,
        Pid::Stall,
        Pid::Pre,
    ];

    /// The PID bytes are fixed by the spec; these are the values a bus analyser
    /// shows. Written out literally so a typo in `to_byte` cannot hide behind
    /// the same arithmetic being used to check it.
    #[test]
    fn pid_bytes_match_the_spec() {
        assert_eq!(Pid::Out.to_byte(), 0xE1);
        assert_eq!(Pid::In.to_byte(), 0x69);
        assert_eq!(Pid::Sof.to_byte(), 0xA5);
        assert_eq!(Pid::Setup.to_byte(), 0x2D);
        assert_eq!(Pid::Data0.to_byte(), 0xC3);
        assert_eq!(Pid::Data1.to_byte(), 0x4B);
        assert_eq!(Pid::Ack.to_byte(), 0xD2);
        assert_eq!(Pid::Nak.to_byte(), 0x5A);
        assert_eq!(Pid::Stall.to_byte(), 0x1E);
        assert_eq!(Pid::Pre.to_byte(), 0x3C);
    }

    #[test]
    fn pid_round_trips() {
        for pid in ALL {
            assert_eq!(Pid::from_byte(pid.to_byte()), Some(pid), "{pid:?}");
        }
    }

    /// A single bit flip anywhere in the PID byte must be rejected, which is
    /// the entire point of the check nibble.
    #[test]
    fn pid_rejects_every_single_bit_flip() {
        for pid in ALL {
            let good = pid.to_byte();
            for bit in 0..8 {
                let bad = good ^ (1 << bit);
                assert_eq!(Pid::from_byte(bad), None, "{pid:?} bit {bit} accepted");
            }
        }
    }

    #[test]
    fn token_round_trips_over_every_address_and_endpoint() {
        for address in 0..=0x7Fu8 {
            for endpoint in 0..=0x0Fu8 {
                let [b0, b1] = encode_token(address, endpoint);
                let got = decode_token(Pid::In, b0, b1).expect("valid CRC");
                assert_eq!(got.address, address);
                assert_eq!(got.endpoint, endpoint);
                assert_eq!(got.pid, Pid::In);
            }
        }
    }

    /// CRC5 catches all single-bit errors in a token; a device that answered a
    /// corrupted address would be replying on another device's behalf.
    #[test]
    fn token_rejects_every_single_bit_flip() {
        let [b0, b1] = encode_token(0x2A, 3);
        for bit in 0..16 {
            let (x0, x1) = if bit < 8 {
                (b0 ^ (1 << bit), b1)
            } else {
                (b0, b1 ^ (1 << (bit - 8)))
            };
            assert_eq!(decode_token(Pid::In, x0, x1), None, "bit {bit} accepted");
        }
    }

    #[test]
    fn sof_frame_numbers_round_trip() {
        for frame in 0..=0x7FFu16 {
            let raw = frame | ((crc5(frame) as u16) << 11);
            assert_eq!(decode_sof(raw as u8, (raw >> 8) as u8), Some(frame));
        }
    }

    #[test]
    fn data_packet_builds_and_checks() {
        // 192 bytes: this project's iso IN endpoint.
        let payload: Vec<u8> = (0..192u32).map(|i| (i * 17 + 3) as u8).collect();
        let mut out = [0u8; 256];
        let n = build_data(Pid::Data0, &payload, &mut out).unwrap();
        assert_eq!(n, 2 + 192 + 2);
        assert_eq!(out[0], SYNC);
        assert_eq!(Pid::from_byte(out[1]), Some(Pid::Data0));
        assert_eq!(check_data(&out[2..n]), Some(&payload[..]));
    }

    #[test]
    fn data_packet_rejects_corruption() {
        let payload = [1u8, 2, 3, 4];
        let mut out = [0u8; 32];
        let n = build_data(Pid::Data1, &payload, &mut out).unwrap();
        out[3] ^= 0x01;
        assert_eq!(check_data(&out[2..n]), None);
    }

    #[test]
    fn zero_length_data_packet_is_valid() {
        // The status stage of a control transfer is a DATA1 with no payload.
        let mut out = [0u8; 8];
        let n = build_data(Pid::Data1, &[], &mut out).unwrap();
        assert_eq!(n, 4);
        assert_eq!(check_data(&out[2..n]), Some(&[][..]));
    }

    #[test]
    fn build_data_refuses_to_overflow() {
        let mut out = [0u8; 8];
        assert!(build_data(Pid::Data0, &[0u8; 16], &mut out).is_none());
    }

    #[test]
    fn handshakes_are_two_bytes() {
        assert_eq!(build_handshake(Pid::Ack), [0x80, 0xD2]);
        assert_eq!(build_handshake(Pid::Nak), [0x80, 0x5A]);
        assert_eq!(build_handshake(Pid::Stall), [0x80, 0x1E]);
    }
}

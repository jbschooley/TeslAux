// SPDX-License-Identifier: MIT
//! USB Mass Storage, Bulk-Only Transport, and the slice of SCSI a car needs.
//!
//! `embassy-usb` has no mass-storage class, so this is hand-rolled — the same
//! situation as the endpoint-less IF3 handler in `teslamic.rs`.
//!
//! Everything here is **pure logic over byte slices**: no HAL, no endpoints, no
//! async. That is deliberate. The transport is a dozen lines of "read 31 bytes,
//! maybe move data, write 13 bytes"; all the ways this can go wrong live in the
//! command handling, and that part is testable on the host. Every fault this
//! project has spent real time on was invisible until something measured it, so
//! the parts that *can* be measured cheaply are kept separate from the parts
//! that need hardware.
//!
//! Bulk-Only Transport is three phases:
//!
//! 1. the host writes a 31-byte **CBW** naming a SCSI command,
//! 2. data moves in the direction the CBW declared, if any,
//! 3. the device writes a 13-byte **CSW** with status and residue.
//!
//! Getting the residue or the direction wrong wedges the host — it will keep
//! waiting for bytes that never come — so those are what the tests are for.

#![allow(dead_code)]

pub const CBW_LEN: usize = 31;
pub const CSW_LEN: usize = 13;
const CBW_SIG: u32 = 0x4342_5355; // "USBC"
const CSW_SIG: u32 = 0x5342_5355; // "USBS"

/// Interface descriptor values for Bulk-Only Transport with transparent SCSI.
pub const CLASS_MASS_STORAGE: u8 = 0x08;
pub const SUBCLASS_SCSI: u8 = 0x06;
pub const PROTOCOL_BULK_ONLY: u8 = 0x50;

/// Class-specific control requests.
pub const REQ_GET_MAX_LUN: u8 = 0xFE;
pub const REQ_BULK_ONLY_RESET: u8 = 0xFF;

// The SCSI opcodes a host actually issues to a read-only disk.
const TEST_UNIT_READY: u8 = 0x00;
const REQUEST_SENSE: u8 = 0x03;
const INQUIRY: u8 = 0x12;
const MODE_SENSE_6: u8 = 0x1A;
const START_STOP_UNIT: u8 = 0x1B;
const PREVENT_ALLOW_REMOVAL: u8 = 0x1E;
const READ_FORMAT_CAPACITIES: u8 = 0x23;
const READ_CAPACITY_10: u8 = 0x25;
const READ_10: u8 = 0x28;
const READ_12: u8 = 0xA8;
const REPORT_LUNS: u8 = 0xA0;
const SERVICE_ACTION_IN_16: u8 = 0x9E;
const SAI_READ_CAPACITY_16: u8 = 0x10;
const WRITE_10: u8 = 0x2A;
const SYNCHRONIZE_CACHE: u8 = 0x35;
const MODE_SENSE_10: u8 = 0x5A;

/// SCSI sense data, as the three fields a host actually looks at.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Sense {
    pub key: u8,
    pub asc: u8,
    pub ascq: u8,
}

impl Sense {
    pub const GOOD: Sense = Sense { key: 0x00, asc: 0x00, ascq: 0x00 };
    /// Illegal request / invalid command operation code.
    pub const INVALID_COMMAND: Sense = Sense { key: 0x05, asc: 0x20, ascq: 0x00 };
    /// Illegal request / logical block address out of range.
    pub const LBA_OUT_OF_RANGE: Sense = Sense { key: 0x05, asc: 0x21, ascq: 0x00 };
    /// Illegal request / invalid field in the command block.
    pub const INVALID_FIELD: Sense = Sense { key: 0x05, asc: 0x24, ascq: 0x00 };
    /// Data protect / write protected.
    pub const WRITE_PROTECTED: Sense = Sense { key: 0x07, asc: 0x27, ascq: 0x00 };
}

/// A parsed Command Block Wrapper.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cbw {
    pub tag: u32,
    /// How many bytes the host expects to move in this transfer.
    pub data_len: u32,
    /// True when the host expects data *from* us.
    pub data_in: bool,
    pub lun: u8,
    /// The SCSI command block, `cb_len` bytes of it.
    pub cb: [u8; 16],
    pub cb_len: usize,
}

impl Cbw {
    /// Parse 31 bytes. `None` if it is not a valid CBW — the spec calls this
    /// "not meaningful", and the correct response is to stall both endpoints
    /// and wait for a reset, never to guess.
    pub fn parse(buf: &[u8]) -> Option<Cbw> {
        if buf.len() != CBW_LEN {
            return None;
        }
        if u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) != CBW_SIG {
            return None;
        }
        let cb_len = (buf[14] & 0x1F) as usize;
        if cb_len == 0 || cb_len > 16 {
            return None;
        }
        let mut cb = [0u8; 16];
        cb[..cb_len].copy_from_slice(&buf[15..15 + cb_len]);
        Some(Cbw {
            tag: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
            data_len: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            data_in: buf[12] & 0x80 != 0,
            lun: buf[13] & 0x0F,
            cb,
            cb_len,
        })
    }

    pub fn opcode(&self) -> u8 {
        self.cb[0]
    }

    /// The LBA field of a 10-byte command.
    pub fn lba(&self) -> u32 {
        u32::from_be_bytes([self.cb[2], self.cb[3], self.cb[4], self.cb[5]])
    }

    /// The transfer-length field of a 10-byte command, in blocks.
    pub fn blocks(&self) -> u16 {
        u16::from_be_bytes([self.cb[7], self.cb[8]])
    }
}

/// Status byte of a Command Status Wrapper.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Status {
    Passed = 0,
    Failed = 1,
    PhaseError = 2,
}

/// Build the 13-byte CSW that closes a transfer.
///
/// `residue` is how much of the host's expectation went unmet. Reporting it
/// wrongly is worse than failing the command: the host waits for bytes that
/// never arrive.
pub fn csw(tag: u32, residue: u32, status: Status, out: &mut [u8; CSW_LEN]) {
    out[0..4].copy_from_slice(&CSW_SIG.to_le_bytes());
    out[4..8].copy_from_slice(&tag.to_le_bytes());
    out[8..12].copy_from_slice(&residue.to_le_bytes());
    out[12] = status as u8;
}

/// What the transport should do after a command is dispatched.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Action {
    /// Send `len` bytes already placed in the reply buffer, then a CSW.
    Reply { len: usize },
    /// Stream `blocks` sectors starting at `lba`, then a CSW.
    ReadBlocks { lba: u32, blocks: u32 },
    /// No data phase; just a CSW.
    None,
    /// Fail with the recorded sense, stalling the data phase if one was
    /// expected.
    Fail,
}

/// A read-only disk that answers SCSI.
///
/// It holds no data: sector contents come from a callback, which is what lets
/// the whole volume be synthesised. See `fat.rs`.
pub struct Scsi {
    pub blocks: u32,
    pub block_size: u32,
    sense: Sense,
}

impl Scsi {
    pub const fn new(blocks: u32, block_size: u32) -> Self {
        Self { blocks, block_size, sense: Sense::GOOD }
    }

    pub fn sense(&self) -> Sense {
        self.sense
    }

    /// Dispatch one command. `reply` receives any data the command produces.
    ///
    /// Note the deliberate ordering: `REQUEST_SENSE` must report the sense left
    /// by the *previous* command, so it reads before the reset at the end.
    pub fn command(&mut self, cbw: &Cbw, reply: &mut [u8]) -> Action {
        let op = cbw.opcode();

        if op == REQUEST_SENSE {
            let n = self.request_sense(reply, cbw.data_len as usize);
            self.sense = Sense::GOOD;
            return Action::Reply { len: n };
        }

        // Every other command starts from a clean sense.
        self.sense = Sense::GOOD;

        match op {
            TEST_UNIT_READY | START_STOP_UNIT | PREVENT_ALLOW_REMOVAL | SYNCHRONIZE_CACHE => {
                Action::None
            }
            INQUIRY => {
                // Bit 0 of byte 1 is EVPD: the host wants a vital-product-data
                // page, named by byte 2, *not* the standard inquiry data.
                // Answering with standard data regardless is a protocol
                // violation, and a host that asked for a serial number and got
                // a device type back has every reason to distrust the device.
                if cbw.cb[1] & 0x01 != 0 {
                    match self.vpd(cbw.cb[2], reply, cbw.data_len as usize) {
                        Some(len) => Action::Reply { len },
                        None => {
                            // An unsupported page must be refused, not
                            // approximated.
                            self.sense = Sense::INVALID_FIELD;
                            Action::Fail
                        }
                    }
                } else {
                    Action::Reply { len: self.inquiry(reply, cbw.data_len as usize) }
                }
            }
            REPORT_LUNS => Action::Reply { len: self.report_luns(reply) },
            SERVICE_ACTION_IN_16 if cbw.cb[1] & 0x1F == SAI_READ_CAPACITY_16 => {
                Action::Reply { len: self.read_capacity_16(reply) }
            }
            READ_CAPACITY_10 => Action::Reply { len: self.read_capacity(reply) },
            READ_FORMAT_CAPACITIES => {
                Action::Reply { len: self.read_format_capacities(reply) }
            }
            MODE_SENSE_6 => Action::Reply { len: self.mode_sense_6(reply) },
            MODE_SENSE_10 => Action::Reply { len: self.mode_sense_10(reply) },
            READ_10 | READ_12 => {
                // READ(12) puts a 32-bit block count where READ(10) has 16.
                let blocks = if op == READ_12 {
                    u32::from_be_bytes([cbw.cb[6], cbw.cb[7], cbw.cb[8], cbw.cb[9]])
                } else {
                    cbw.blocks() as u32
                };
                let lba = cbw.lba();
                // A zero-length read is legal and means nothing to do.
                if blocks == 0 {
                    return Action::None;
                }
                // Check the end of the range as a u64: `lba + blocks` can wrap,
                // and a wrapped comparison would admit a read past the end.
                if lba as u64 + blocks as u64 > self.blocks as u64 {
                    self.sense = Sense::LBA_OUT_OF_RANGE;
                    return Action::Fail;
                }
                Action::ReadBlocks { lba, blocks }
            }
            WRITE_10 => {
                // Read-only by construction: there is nowhere to put the data.
                self.sense = Sense::WRITE_PROTECTED;
                Action::Fail
            }
            _ => {
                self.sense = Sense::INVALID_COMMAND;
                Action::Fail
            }
        }
    }

    /// Standard INQUIRY data. The strings are padded to their fixed widths,
    /// which hosts rely on.
    fn inquiry(&self, out: &mut [u8], want: usize) -> usize {
        const LEN: usize = 36;
        let n = LEN.min(out.len()).min(if want == 0 { LEN } else { want });
        let clear = LEN.min(out.len());
        out[..clear].fill(0);
        if out.len() >= LEN {
            out[0] = 0x00; // direct-access block device
            out[1] = 0x80; // removable
            out[2] = 0x04; // SPC-2
            out[3] = 0x02; // response data format 2
            out[4] = (LEN - 5) as u8; // additional length
            out[8..16].copy_from_slice(b"TeslAux ");
            out[16..32].copy_from_slice(b"Media Bridge    ");
            out[32..36].copy_from_slice(b"1.00");
        }
        n
    }

    /// Vital product data. `None` for a page we do not publish.
    fn vpd(&self, page: u8, out: &mut [u8], want: usize) -> Option<usize> {
        let len = match page {
            // 0x00: the list of pages we support, including itself.
            0x00 => {
                if out.len() < 7 {
                    return Some(0);
                }
                out[..7].fill(0);
                out[1] = 0x00;
                out[3] = 3; // three pages follow
                out[4] = 0x00;
                out[5] = 0x80;
                out[6] = 0x83;
                7
            }
            // 0x80: unit serial number.
            0x80 => {
                const SERIAL: &[u8] = b"TESLAUX0001     ";
                if out.len() < 4 + SERIAL.len() {
                    return Some(0);
                }
                out[0] = 0x00;
                out[1] = 0x80;
                out[2] = 0;
                out[3] = SERIAL.len() as u8;
                out[4..4 + SERIAL.len()].copy_from_slice(SERIAL);
                4 + SERIAL.len()
            }
            // 0x83: device identification, as a single vendor-specific
            // descriptor — the simplest form that is still well formed.
            0x83 => {
                const ID: &[u8] = b"TeslAux Media   ";
                let total = 4 + 4 + ID.len();
                if out.len() < total {
                    return Some(0);
                }
                out[..total].fill(0);
                out[1] = 0x83;
                out[3] = (4 + ID.len()) as u8;
                out[4] = 0x02; // ASCII
                out[5] = 0x00; // vendor specific
                out[7] = ID.len() as u8;
                out[8..8 + ID.len()].copy_from_slice(ID);
                total
            }
            _ => return None,
        };
        Some(len.min(out.len()).min(if want == 0 { len } else { want }))
    }

    /// REPORT LUNS: one LUN, numbered zero.
    fn report_luns(&self, out: &mut [u8]) -> usize {
        if out.len() < 16 {
            return 0;
        }
        out[..16].fill(0);
        out[3] = 8; // one 8-byte LUN entry follows
        16
    }

    /// READ CAPACITY(16), for hosts that ask the 16-byte way.
    fn read_capacity_16(&self, out: &mut [u8]) -> usize {
        if out.len() < 32 {
            return 0;
        }
        out[..32].fill(0);
        out[4..8].copy_from_slice(&(self.blocks - 1).to_be_bytes());
        out[8..12].copy_from_slice(&self.block_size.to_be_bytes());
        32
    }

    /// READ CAPACITY(10) reports the address of the **last** block, not the
    /// count. Off by one here and the host sees a disk one sector too large,
    /// then errors on the read that falls off the end.
    fn read_capacity(&self, out: &mut [u8]) -> usize {
        if out.len() < 8 {
            return 0;
        }
        out[0..4].copy_from_slice(&(self.blocks - 1).to_be_bytes());
        out[4..8].copy_from_slice(&self.block_size.to_be_bytes());
        8
    }

    fn read_format_capacities(&self, out: &mut [u8]) -> usize {
        if out.len() < 12 {
            return 0;
        }
        out[..12].fill(0);
        out[3] = 8; // capacity list length
        out[4..8].copy_from_slice(&self.blocks.to_be_bytes());
        out[8] = 0x02; // formatted media
        out[9..12].copy_from_slice(&self.block_size.to_be_bytes()[1..]);
        12
    }

    /// MODE SENSE(6) with no pages: enough to answer, and the write-protect bit
    /// is the part that matters — it stops a host trying to write.
    fn mode_sense_6(&self, out: &mut [u8]) -> usize {
        if out.len() < 4 {
            return 0;
        }
        out[0] = 3; // mode data length, excluding this byte
        out[1] = 0; // medium type
        out[2] = 0x80; // write protected
        out[3] = 0; // block descriptor length
        4
    }

    fn mode_sense_10(&self, out: &mut [u8]) -> usize {
        if out.len() < 8 {
            return 0;
        }
        out[..8].fill(0);
        out[1] = 6; // mode data length
        out[3] = 0x80; // write protected
        8
    }

    /// Fixed-format sense data.
    fn request_sense(&self, out: &mut [u8], want: usize) -> usize {
        const LEN: usize = 18;
        let n = LEN.min(out.len()).min(if want == 0 { LEN } else { want });
        if out.len() >= LEN {
            out[..LEN].fill(0);
            out[0] = 0x70; // current error, fixed format
            out[2] = self.sense.key;
            out[7] = (LEN - 8) as u8; // additional sense length
            out[12] = self.sense.asc;
            out[13] = self.sense.ascq;
        }
        n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a CBW the way a host would, so the tests exercise the parser too.
    fn cbw(tag: u32, data_len: u32, data_in: bool, cb: &[u8]) -> [u8; CBW_LEN] {
        let mut b = [0u8; CBW_LEN];
        b[0..4].copy_from_slice(&CBW_SIG.to_le_bytes());
        b[4..8].copy_from_slice(&tag.to_le_bytes());
        b[8..12].copy_from_slice(&data_len.to_le_bytes());
        b[12] = if data_in { 0x80 } else { 0x00 };
        b[13] = 0;
        b[14] = cb.len() as u8;
        b[15..15 + cb.len()].copy_from_slice(cb);
        b
    }

    fn disk() -> Scsi {
        Scsi::new(4_501_676, 512)
    }

    #[test]
    fn parses_a_host_cbw() {
        let raw = cbw(0xDEAD_BEEF, 512, true, &[READ_10, 0, 0, 0, 0x12, 0x34, 0, 0, 1, 0]);
        let c = Cbw::parse(&raw).expect("valid CBW rejected");
        assert_eq!(c.tag, 0xDEAD_BEEF);
        assert_eq!(c.data_len, 512);
        assert!(c.data_in);
        assert_eq!(c.opcode(), READ_10);
        assert_eq!(c.lba(), 0x1234);
        assert_eq!(c.blocks(), 1);
    }

    #[test]
    fn rejects_malformed_cbws() {
        // A bad signature is "not meaningful" and must never be guessed at.
        let mut bad = cbw(1, 0, false, &[TEST_UNIT_READY]);
        bad[0] ^= 0xFF;
        assert!(Cbw::parse(&bad).is_none(), "accepted a bad signature");

        // A zero-length command block has no opcode to dispatch.
        let mut zero = cbw(1, 0, false, &[TEST_UNIT_READY]);
        zero[14] = 0;
        assert!(Cbw::parse(&zero).is_none(), "accepted an empty command block");

        assert!(Cbw::parse(&[0u8; 30]).is_none(), "accepted a short CBW");
    }

    #[test]
    fn csw_round_trips() {
        let mut out = [0u8; CSW_LEN];
        csw(0x1234_5678, 42, Status::Failed, &mut out);
        assert_eq!(u32::from_le_bytes(out[0..4].try_into().unwrap()), CSW_SIG);
        assert_eq!(u32::from_le_bytes(out[4..8].try_into().unwrap()), 0x1234_5678);
        assert_eq!(u32::from_le_bytes(out[8..12].try_into().unwrap()), 42);
        assert_eq!(out[12], 1);
    }

    #[test]
    fn inquiry_is_well_formed() {
        let mut d = disk();
        let mut r = [0u8; 64];
        let c = Cbw::parse(&cbw(1, 36, true, &[INQUIRY, 0, 0, 0, 36, 0])).unwrap();
        let a = d.command(&c, &mut r);
        assert_eq!(a, Action::Reply { len: 36 });
        assert_eq!(r[0], 0x00, "must be a direct-access block device");
        assert_eq!(r[1] & 0x80, 0x80, "must report removable");
        assert_eq!(r[4], 31, "additional length must cover the 36-byte reply");
        assert_eq!(&r[8..16], b"TeslAux ");
    }

    #[test]
    fn capacity_reports_the_last_block_not_the_count() {
        // Off by one here shows up as a host error on the final sector, long
        // after the mistake.
        let mut d = disk();
        let mut r = [0u8; 64];
        let c = Cbw::parse(&cbw(1, 8, true, &[READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0])).unwrap();
        assert_eq!(d.command(&c, &mut r), Action::Reply { len: 8 });
        assert_eq!(u32::from_be_bytes(r[0..4].try_into().unwrap()), 4_501_675);
        assert_eq!(u32::from_be_bytes(r[4..8].try_into().unwrap()), 512);
    }

    #[test]
    fn read_within_range_streams_blocks() {
        let mut d = disk();
        let mut r = [0u8; 64];
        let c = Cbw::parse(&cbw(1, 4096, true, &[READ_10, 0, 0, 0, 0x10, 0x00, 0, 0, 8, 0])).unwrap();
        assert_eq!(d.command(&c, &mut r), Action::ReadBlocks { lba: 0x1000, blocks: 8 });
    }

    #[test]
    fn read_past_the_end_fails_without_wrapping() {
        // lba + blocks overflows u32 here; a wrapped comparison would let it
        // through and read off the end of the device.
        let mut d = disk();
        let mut r = [0u8; 64];
        let c = Cbw::parse(&cbw(
            1, 512, true,
            &[READ_10, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0xFF, 0xFF, 0],
        )).unwrap();
        assert_eq!(d.command(&c, &mut r), Action::Fail);
        assert_eq!(d.sense(), Sense::LBA_OUT_OF_RANGE);
    }

    #[test]
    fn zero_length_read_is_not_an_error() {
        let mut d = disk();
        let mut r = [0u8; 64];
        let c = Cbw::parse(&cbw(1, 0, true, &[READ_10, 0, 0, 0, 0, 0, 0, 0, 0, 0])).unwrap();
        assert_eq!(d.command(&c, &mut r), Action::None);
        assert_eq!(d.sense(), Sense::GOOD);
    }

    #[test]
    fn writes_are_refused_as_write_protected() {
        let mut d = disk();
        let mut r = [0u8; 64];
        let c = Cbw::parse(&cbw(1, 512, false, &[WRITE_10, 0, 0, 0, 0, 1, 0, 0, 1, 0])).unwrap();
        assert_eq!(d.command(&c, &mut r), Action::Fail);
        assert_eq!(d.sense(), Sense::WRITE_PROTECTED);
    }

    #[test]
    fn mode_sense_says_write_protected() {
        // A host that thinks the disk is writable will eventually try, and the
        // failure is uglier than declaring it up front.
        let mut d = disk();
        let mut r = [0u8; 64];
        let c = Cbw::parse(&cbw(1, 4, true, &[MODE_SENSE_6, 0, 0x3F, 0, 4, 0])).unwrap();
        assert_eq!(d.command(&c, &mut r), Action::Reply { len: 4 });
        assert_eq!(r[2] & 0x80, 0x80, "write-protect bit not set");
    }

    #[test]
    fn request_sense_reports_the_previous_failure_then_clears() {
        // The whole point of sense data: it survives exactly one command.
        let mut d = disk();
        let mut r = [0u8; 64];

        let bad = Cbw::parse(&cbw(1, 512, false, &[WRITE_10, 0, 0, 0, 0, 1, 0, 0, 1, 0])).unwrap();
        assert_eq!(d.command(&bad, &mut r), Action::Fail);

        let sense = Cbw::parse(&cbw(2, 18, true, &[REQUEST_SENSE, 0, 0, 0, 18, 0])).unwrap();
        assert_eq!(d.command(&sense, &mut r), Action::Reply { len: 18 });
        assert_eq!(r[0], 0x70);
        assert_eq!(r[2], Sense::WRITE_PROTECTED.key);
        assert_eq!(r[12], Sense::WRITE_PROTECTED.asc);

        // Asking again must report good, not repeat the stale failure.
        assert_eq!(d.command(&sense, &mut r), Action::Reply { len: 18 });
        assert_eq!(r[2], Sense::GOOD.key);
    }

    #[test]
    fn unknown_commands_fail_with_invalid_command() {
        let mut d = disk();
        let mut r = [0u8; 64];
        let c = Cbw::parse(&cbw(1, 0, false, &[0xEE, 0, 0, 0, 0, 0])).unwrap();
        assert_eq!(d.command(&c, &mut r), Action::Fail);
        assert_eq!(d.sense(), Sense::INVALID_COMMAND);
    }

    #[test]
    fn the_quiet_commands_need_no_data_phase() {
        let mut d = disk();
        let mut r = [0u8; 64];
        for op in [TEST_UNIT_READY, START_STOP_UNIT, PREVENT_ALLOW_REMOVAL, SYNCHRONIZE_CACHE] {
            let c = Cbw::parse(&cbw(1, 0, false, &[op, 0, 0, 0, 0, 0])).unwrap();
            assert_eq!(d.command(&c, &mut r), Action::None, "opcode {op:#04x}");
            assert_eq!(d.sense(), Sense::GOOD);
        }
    }

    #[test]
    fn evpd_inquiry_returns_the_requested_page_not_standard_data() {
        // The bug this test exists for: ignoring the EVPD bit and answering
        // every INQUIRY with standard data. A host that asked for a serial
        // number and got a device type back has reason to distrust the device.
        let mut d = disk();
        let mut r = [0u8; 64];

        // Page 0x00 lists the pages we support, and must list itself.
        let c = Cbw::parse(&cbw(1, 64, true, &[INQUIRY, 0x01, 0x00, 0, 64, 0])).unwrap();
        let a = d.command(&c, &mut r);
        assert!(matches!(a, Action::Reply { .. }), "page 00 refused: {a:?}");
        assert_eq!(r[1], 0x00, "page code must be echoed");
        let n = r[3] as usize;
        assert!(r[4..4 + n].contains(&0x00), "page list must include itself");
        assert!(r[4..4 + n].contains(&0x80), "page list must include the serial");
        assert!(r[4..4 + n].contains(&0x83), "page list must include device id");

        // Page 0x80 is the serial number, and must echo its own page code.
        let c = Cbw::parse(&cbw(2, 64, true, &[INQUIRY, 0x01, 0x80, 0, 64, 0])).unwrap();
        assert!(matches!(d.command(&c, &mut r), Action::Reply { .. }));
        assert_eq!(r[1], 0x80);
        assert!(r[3] > 0, "serial number must not be empty");

        // A page we do not publish must be refused rather than approximated.
        let c = Cbw::parse(&cbw(3, 64, true, &[INQUIRY, 0x01, 0xB2, 0, 64, 0])).unwrap();
        assert_eq!(d.command(&c, &mut r), Action::Fail);
        assert_eq!(d.sense(), Sense::INVALID_FIELD);
    }

    #[test]
    fn read_12_uses_its_wider_block_count() {
        // READ(12) puts a 32-bit count where READ(10) has 16. Reading it as
        // 16 bits truncates every large transfer, silently.
        let mut d = disk();
        let mut r = [0u8; 64];
        let c = Cbw::parse(&cbw(
            1, 0x20000, true,
            &[READ_12, 0, 0, 0, 0x10, 0x00, 0, 0, 0x01, 0x00, 0, 0],
        )).unwrap();
        assert_eq!(d.command(&c, &mut r), Action::ReadBlocks { lba: 0x1000, blocks: 256 });
    }

    #[test]
    fn report_luns_and_capacity_16_are_well_formed() {
        let mut d = disk();
        let mut r = [0u8; 64];

        let c = Cbw::parse(&cbw(1, 16, true, &[REPORT_LUNS, 0, 0, 0, 0, 0, 0, 0, 0, 16, 0, 0]))
            .unwrap();
        assert_eq!(d.command(&c, &mut r), Action::Reply { len: 16 });
        assert_eq!(r[3], 8, "one LUN entry expected");

        let c = Cbw::parse(&cbw(
            2, 32, true,
            &[SERVICE_ACTION_IN_16, SAI_READ_CAPACITY_16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 32, 0, 0, 0, 0],
        )).unwrap();
        assert_eq!(d.command(&c, &mut r), Action::Reply { len: 32 });
        // Same off-by-one rule as READ CAPACITY(10): the last block, not a count.
        assert_eq!(u32::from_be_bytes(r[4..8].try_into().unwrap()), 4_501_675);
    }

    #[test]
    fn a_full_enumeration_sequence_succeeds() {
        // The order a host actually uses. Any one of these failing leaves the
        // volume unmounted, so run them as a sequence rather than in isolation.
        let mut d = disk();
        let mut r = [0u8; 64];
        let steps: [(&[u8], bool); 4] = [
            (&[INQUIRY, 0, 0, 0, 36, 0], true),
            (&[TEST_UNIT_READY, 0, 0, 0, 0, 0], false),
            (&[READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0], true),
            (&[READ_10, 0, 0, 0, 0, 0, 0, 0, 1, 0], true),
        ];
        for (i, (cb, wants_data)) in steps.iter().enumerate() {
            let c = Cbw::parse(&cbw(i as u32, 512, *wants_data, cb)).unwrap();
            let a = d.command(&c, &mut r);
            assert_ne!(a, Action::Fail, "step {i} failed: {cb:02x?}");
        }
    }
}

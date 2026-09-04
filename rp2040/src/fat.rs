// SPDX-License-Identifier: MIT
//! A FAT32 volume that does not exist.
//!
//! Every sector is computed on demand, so the "drive" costs no storage at all:
//! the file data is silence, which is zeros, and the metadata is arithmetic over
//! a fixed layout. What it holds is a set of identical silent WAV tracks whose
//! only purpose is to be a **position counter the car moves for us**.
//!
//! The car's media player advances through these tracks when the steering wheel
//! next/prev buttons are pressed. Because we author the layout, a sector address
//! identifies a track, so watching which sectors are read tells us which track
//! the car moved to — and the difference from the last one is how many times the
//! button was pressed. Reading the *destination* rather than counting events is
//! what makes rapid presses safe: five presses land five tracks away whether the
//! player opens each file on the way or jumps straight there.
//!
//! Nothing here is audio. The tracks are silent by design; the real audio
//! arrives over the microphone endpoint, which the car mixes on top of whatever
//! media is playing.

#![allow(dead_code)]

pub const SECTOR: usize = 512;
/// 32 KB clusters. Larger clusters mean a smaller FAT to synthesise, and the
/// cluster count still has to stay above FAT32's minimum — see `_LAYOUT`.
pub const SECTORS_PER_CLUSTER: u32 = 64;
pub const CLUSTER_BYTES: u32 = SECTORS_PER_CLUSTER * SECTOR as u32;
pub const RESERVED_SECTORS: u32 = 32;

/// 48 kHz, 16-bit stereo: the format the rest of this project uses.
pub const SAMPLE_RATE: u32 = 48_000;
pub const CHANNELS: u16 = 2;
pub const BITS: u16 = 16;
pub const BYTES_PER_SECOND: u32 = SAMPLE_RATE * CHANNELS as u32 * (BITS as u32 / 8);

/// Ten minutes per track.
///
/// Short tracks keep the volume small, which matters because Tesla re-indexes
/// the drive **every time the car wakes**, not just on insertion. The cost is
/// that a track occasionally ends on its own; that is distinguishable from a
/// button press, because a natural advance happens only after the player has
/// read to the end of the file.
pub const TRACK_SECONDS: u32 = 600;
pub const WAV_HEADER: u32 = 44;
pub const TRACK_DATA_BYTES: u32 = TRACK_SECONDS * BYTES_PER_SECOND;
pub const TRACK_FILE_BYTES: u32 = TRACK_DATA_BYTES + WAV_HEADER;

/// How many tracks. This is the range of the counter, not a musical choice:
/// with N tracks a burst of presses can be resolved up to about N/2 in either
/// direction before wrap-around becomes ambiguous.
pub const N_TRACKS: u32 = 20;

pub const CLUSTERS_PER_FILE: u32 = TRACK_FILE_BYTES.div_ceil(CLUSTER_BYTES);
/// Cluster 2 is the root directory; the files follow it.
pub const ROOT_CLUSTER: u32 = 2;
pub const FIRST_FILE_CLUSTER: u32 = 3;
pub const DATA_CLUSTERS: u32 = 1 + N_TRACKS * CLUSTERS_PER_FILE;
pub const FAT_SECTORS: u32 = ((DATA_CLUSTERS + 2) * 4).div_ceil(SECTOR as u32);
pub const FAT_START: u32 = RESERVED_SECTORS;
pub const DATA_START: u32 = FAT_START + 2 * FAT_SECTORS;
pub const TOTAL_SECTORS: u32 = DATA_START + DATA_CLUSTERS * SECTORS_PER_CLUSTER;

const _LAYOUT: () = {
    // Below 65525 clusters a host is required to read the volume as FAT16, and
    // the BPB we emit is FAT32 — the mismatch makes the volume unreadable
    // rather than merely smaller.
    assert!(
        DATA_CLUSTERS > 65524,
        "too few clusters for FAT32; add tracks or shrink the cluster size"
    );
};

/// Where a data sector falls.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Position {
    /// 0-based track index.
    pub track: u32,
    /// Byte offset within that track's file, header included.
    pub offset: u32,
}

fn le16(buf: &mut [u8], at: usize, v: u16) {
    buf[at..at + 2].copy_from_slice(&v.to_le_bytes());
}

fn le32(buf: &mut [u8], at: usize, v: u32) {
    buf[at..at + 4].copy_from_slice(&v.to_le_bytes());
}

/// The first cluster of a track's file.
pub fn track_first_cluster(track: u32) -> u32 {
    FIRST_FILE_CLUSTER + track * CLUSTERS_PER_FILE
}

/// Which track and offset does `lba` address? `None` for metadata sectors and
/// for the root directory.
///
/// This is the whole point of synthesising the volume rather than storing one:
/// the mapping from a sector the car asks for to a position in a track is a
/// closed-form calculation, so watching reads is enough to know where the
/// player is.
pub fn locate(lba: u32) -> Option<Position> {
    if lba < DATA_START || lba >= TOTAL_SECTORS {
        return None;
    }
    let cluster = ROOT_CLUSTER + (lba - DATA_START) / SECTORS_PER_CLUSTER;
    if cluster < FIRST_FILE_CLUSTER {
        return None; // root directory
    }
    let track = (cluster - FIRST_FILE_CLUSTER) / CLUSTERS_PER_FILE;
    if track >= N_TRACKS {
        return None;
    }
    let first = track_first_cluster(track);
    let within_cluster = (lba - DATA_START) % SECTORS_PER_CLUSTER;
    let offset = (cluster - first) * CLUSTER_BYTES + within_cluster * SECTOR as u32;
    if offset >= TRACK_FILE_BYTES {
        return None; // slack at the end of the last cluster
    }
    Some(Position { track, offset })
}

/// 8.3 name for a track: `001     WAV`, which sorts in playback order.
///
/// Tesla orders by tag title where present and by filename otherwise, and offers
/// no way to impose an order beyond that — so zero-padded numeric names make
/// playback order equal track index, which is exactly what the counter needs.
fn short_name(track: u32, out: &mut [u8; 11]) {
    let n = track + 1;
    *out = *b"           ";
    out[0] = b'0' + ((n / 100) % 10) as u8;
    out[1] = b'0' + ((n / 10) % 10) as u8;
    out[2] = b'0' + (n % 10) as u8;
    out[8..11].copy_from_slice(b"WAV");
}

fn boot_sector(buf: &mut [u8; SECTOR]) {
    buf[0..3].copy_from_slice(&[0xEB, 0x58, 0x90]); // jmp, as hosts expect
    buf[3..11].copy_from_slice(b"MSDOS5.0");
    le16(buf, 11, SECTOR as u16);
    buf[13] = SECTORS_PER_CLUSTER as u8;
    le16(buf, 14, RESERVED_SECTORS as u16);
    buf[16] = 2; // two FATs, as every real volume has
    le16(buf, 17, 0); // FAT32 keeps no root entry count here
    le16(buf, 19, 0); // and no 16-bit sector count
    buf[21] = 0xF8; // fixed disk
    le16(buf, 22, 0); // FAT32 uses the 32-bit field below
    le16(buf, 24, 63);
    le16(buf, 26, 255);
    le32(buf, 28, 0); // no hidden sectors: this is a whole device, not a partition
    le32(buf, 32, TOTAL_SECTORS);
    le32(buf, 36, FAT_SECTORS);
    le16(buf, 40, 0); // no mirroring flags
    le16(buf, 42, 0); // version 0.0
    le32(buf, 44, ROOT_CLUSTER);
    le16(buf, 48, 1); // FSInfo sector
    le16(buf, 50, 6); // backup boot sector
    buf[64] = 0x80;
    buf[66] = 0x29; // extended boot signature
    le32(buf, 67, 0x1234_5678);
    buf[71..82].copy_from_slice(b"TESLAUX    ");
    buf[82..90].copy_from_slice(b"FAT32   ");
    buf[510] = 0x55;
    buf[511] = 0xAA;
}

fn fsinfo_sector(buf: &mut [u8; SECTOR]) {
    le32(buf, 0, 0x4161_5252);
    le32(buf, 484, 0x6141_7272);
    le32(buf, 488, 0xFFFF_FFFF); // free count unknown
    le32(buf, 492, 0xFFFF_FFFF); // next free unknown
    buf[510] = 0x55;
    buf[511] = 0xAA;
}

/// One FAT sector. Entry `c` holds the cluster that follows `c`, or an
/// end-of-chain marker. Every file is contiguous, so the chain is arithmetic.
fn fat_sector(index: u32, buf: &mut [u8; SECTOR]) {
    const PER_SECTOR: u32 = SECTOR as u32 / 4;
    let first = index * PER_SECTOR;
    for i in 0..PER_SECTOR {
        let cluster = first + i;
        let value = if cluster == 0 {
            0x0FFF_FFF8
        } else if cluster == 1 || cluster == ROOT_CLUSTER {
            0x0FFF_FFFF // reserved, and the root's single-cluster chain
        } else if cluster < FIRST_FILE_CLUSTER + N_TRACKS * CLUSTERS_PER_FILE {
            let within = (cluster - FIRST_FILE_CLUSTER) % CLUSTERS_PER_FILE;
            if within == CLUSTERS_PER_FILE - 1 {
                0x0FFF_FFFF // last cluster of this file
            } else {
                cluster + 1
            }
        } else {
            0 // free
        };
        le32(buf, (i * 4) as usize, value);
    }
}

/// One sector of the root directory: a volume label followed by the tracks.
fn root_sector(index: u32, buf: &mut [u8; SECTOR]) {
    const PER_SECTOR: u32 = SECTOR as u32 / 32;
    for slot in 0..PER_SECTOR {
        let entry = index * PER_SECTOR + slot;
        let at = (slot * 32) as usize;
        if entry == 0 {
            buf[at..at + 11].copy_from_slice(b"TESLAUX    ");
            buf[at + 11] = 0x08; // volume label
            continue;
        }
        let track = entry - 1;
        if track >= N_TRACKS {
            return; // rest stays zero: end of directory
        }
        let mut name = [0u8; 11];
        short_name(track, &mut name);
        buf[at..at + 11].copy_from_slice(&name);
        buf[at + 11] = 0x21; // read-only + archive
        let cluster = track_first_cluster(track);
        le16(buf, at + 20, (cluster >> 16) as u16);
        le16(buf, at + 26, cluster as u16);
        le32(buf, at + 28, TRACK_FILE_BYTES);
    }
}

/// The 44-byte canonical WAV header for one track.
fn wav_header(buf: &mut [u8]) {
    buf[0..4].copy_from_slice(b"RIFF");
    le32(buf, 4, TRACK_FILE_BYTES - 8);
    buf[8..12].copy_from_slice(b"WAVE");
    buf[12..16].copy_from_slice(b"fmt ");
    le32(buf, 16, 16);
    le16(buf, 20, 1); // PCM
    le16(buf, 22, CHANNELS);
    le32(buf, 24, SAMPLE_RATE);
    le32(buf, 28, BYTES_PER_SECOND);
    le16(buf, 32, CHANNELS * BITS / 8);
    le16(buf, 34, BITS);
    buf[36..40].copy_from_slice(b"data");
    le32(buf, 40, TRACK_DATA_BYTES);
}

/// Produce the contents of `lba`. Anything not explicitly described is zero,
/// which for the audio payload is exactly right: zeros are silence.
pub fn read_sector(lba: u32, buf: &mut [u8; SECTOR]) {
    buf.fill(0);
    if lba == 0 || lba == 6 {
        boot_sector(buf);
    } else if lba == 1 || lba == 7 {
        fsinfo_sector(buf);
    } else if (FAT_START..FAT_START + FAT_SECTORS).contains(&lba) {
        fat_sector(lba - FAT_START, buf);
    } else if (FAT_START + FAT_SECTORS..DATA_START).contains(&lba) {
        fat_sector(lba - FAT_START - FAT_SECTORS, buf); // the mirror
    } else if let Some(pos) = locate(lba) {
        if pos.offset == 0 {
            wav_header(buf);
        }
    } else if lba >= DATA_START && lba < DATA_START + SECTORS_PER_CLUSTER {
        root_sector(lba - DATA_START, buf);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sector(lba: u32) -> [u8; SECTOR] {
        let mut b = [0u8; SECTOR];
        read_sector(lba, &mut b);
        b
    }

    #[test]
    fn boot_sector_is_valid_fat32() {
        let b = sector(0);
        assert_eq!(&b[510..512], &[0x55, 0xAA], "missing boot signature");
        assert_eq!(u16::from_le_bytes([b[11], b[12]]), SECTOR as u16);
        assert_eq!(b[13] as u32, SECTORS_PER_CLUSTER);
        assert_eq!(b[16], 2, "hosts expect two FATs");
        assert_eq!(u16::from_le_bytes([b[17], b[18]]), 0, "FAT32 root entries must be 0");
        assert_eq!(u32::from_le_bytes(b[32..36].try_into().unwrap()), TOTAL_SECTORS);
        assert_eq!(u32::from_le_bytes(b[44..48].try_into().unwrap()), ROOT_CLUSTER);
        assert_eq!(sector(6), b, "backup boot sector must match");
    }

    #[test]
    fn every_file_chain_is_contiguous_and_terminated() {
        let read = |cluster: u32| -> u32 {
            let per = SECTOR as u32 / 4;
            let b = sector(FAT_START + cluster / per);
            let at = ((cluster % per) * 4) as usize;
            u32::from_le_bytes(b[at..at + 4].try_into().unwrap()) & 0x0FFF_FFFF
        };
        for track in 0..N_TRACKS {
            let first = track_first_cluster(track);
            for i in 0..CLUSTERS_PER_FILE - 1 {
                assert_eq!(read(first + i), first + i + 1, "track {track} chain broken");
            }
            assert_eq!(
                read(first + CLUSTERS_PER_FILE - 1),
                0x0FFF_FFFF,
                "track {track} chain not terminated"
            );
        }
        assert_eq!(read(ROOT_CLUSTER), 0x0FFF_FFFF, "root chain not terminated");
    }

    #[test]
    fn fat_mirror_matches() {
        for i in [0u32, 1, FAT_SECTORS / 2, FAT_SECTORS - 1] {
            assert_eq!(sector(FAT_START + i), sector(FAT_START + FAT_SECTORS + i));
        }
    }

    #[test]
    fn directory_lists_every_track_at_the_right_cluster() {
        let b = sector(DATA_START);
        assert_eq!(&b[0..11], b"TESLAUX    ", "volume label missing");
        assert_eq!(b[11], 0x08);
        for track in 0..N_TRACKS.min(15) {
            let at = ((track + 1) * 32) as usize;
            let mut want = [0u8; 11];
            short_name(track, &mut want);
            assert_eq!(&b[at..at + 11], &want, "wrong name for track {track}");
            let hi = u16::from_le_bytes([b[at + 20], b[at + 21]]) as u32;
            let lo = u16::from_le_bytes([b[at + 26], b[at + 27]]) as u32;
            assert_eq!((hi << 16) | lo, track_first_cluster(track));
            assert_eq!(
                u32::from_le_bytes(b[at + 28..at + 32].try_into().unwrap()),
                TRACK_FILE_BYTES
            );
        }
    }

    #[test]
    fn names_sort_into_playback_order() {
        // Playback order must equal track index, because the index *is* the
        // counter the whole scheme reads.
        let mut prev = [0u8; 11];
        short_name(0, &mut prev);
        for track in 1..N_TRACKS {
            let mut now = [0u8; 11];
            short_name(track, &mut now);
            assert!(now > prev, "track {track} sorts before its predecessor");
            prev = now;
        }
    }

    #[test]
    fn each_track_starts_with_a_wav_header() {
        for track in 0..N_TRACKS {
            let lba = DATA_START + (track_first_cluster(track) - ROOT_CLUSTER) * SECTORS_PER_CLUSTER;
            let b = sector(lba);
            assert_eq!(&b[0..4], b"RIFF", "track {track} has no header");
            assert_eq!(&b[8..12], b"WAVE");
            assert_eq!(u32::from_le_bytes(b[24..28].try_into().unwrap()), SAMPLE_RATE);
            assert_eq!(u32::from_le_bytes(b[40..44].try_into().unwrap()), TRACK_DATA_BYTES);
            assert_eq!(locate(lba), Some(Position { track, offset: 0 }));
        }
    }

    #[test]
    fn locate_round_trips_across_every_track() {
        for track in 0..N_TRACKS {
            let base = DATA_START + (track_first_cluster(track) - ROOT_CLUSTER) * SECTORS_PER_CLUSTER;
            for probe in [0u32, 1, 63, 64, 1000, CLUSTERS_PER_FILE * SECTORS_PER_CLUSTER - 1] {
                match locate(base + probe) {
                    Some(p) => {
                        assert_eq!(p.track, track, "sector {probe} attributed to the wrong track");
                        assert_eq!(p.offset, probe * SECTOR as u32);
                    }
                    // Only the slack past the end of the file may be unmapped.
                    None => assert!(probe * SECTOR as u32 >= TRACK_FILE_BYTES),
                }
            }
        }
    }

    #[test]
    fn metadata_sectors_are_not_mistaken_for_audio() {
        for lba in [0u32, 1, 6, FAT_START, FAT_START + FAT_SECTORS, DATA_START] {
            assert_eq!(locate(lba), None, "sector {lba} looks like track data");
        }
    }

    #[test]
    fn tracks_do_not_overlap() {
        for track in 0..N_TRACKS - 1 {
            assert!(
                track_first_cluster(track) + CLUSTERS_PER_FILE <= track_first_cluster(track + 1),
                "track {track} overlaps the next"
            );
        }
        let last = track_first_cluster(N_TRACKS - 1) + CLUSTERS_PER_FILE;
        assert!(last <= ROOT_CLUSTER + DATA_CLUSTERS, "last track runs past the volume");
    }
}

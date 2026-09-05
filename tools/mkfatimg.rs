//! Emit the synthetic volume as a file, using the firmware's own code.
//!
//! The point is that a host OS is a far better judge of the filesystem than any
//! test I write against my own assumptions:
//!
//!     rustc -O -o /tmp/mkfatimg tools/mkfatimg.rs
//!     /tmp/mkfatimg /tmp/fat.img
//!     hdiutil attach -imagekey diskimage-class=CRawDiskImage /tmp/fat.img
//!     ls -la /Volumes/TESLAUX/ && ffprobe /Volumes/TESLAUX/001.WAV
//!     hdiutil detach /Volumes/TESLAUX
//!
//! Written sparsely, which also demonstrates the claim the design rests on: a
//! 2.3 GB volume has under 900 KB of actual content.
#[path = "../rp2040/src/fat.rs"]
mod fat;
use fat::*;
use std::io::{Seek, SeekFrom, Write};
fn main() {
    let path = std::env::args().nth(1).unwrap();
    let mut f = std::fs::File::create(&path).unwrap();
    let mut buf = [0u8; SECTOR];
    let zero = [0u8; SECTOR];
    let mut written = 0u64;
    for lba in 0..TOTAL_SECTORS {
        read_sector(lba, &mut buf);
        if buf != zero {
            f.seek(SeekFrom::Start(lba as u64 * SECTOR as u64)).unwrap();
            f.write_all(&buf).unwrap();
            written += 1;
        }
    }
    f.set_len(TOTAL_SECTORS as u64 * SECTOR as u64).unwrap();
    eprintln!("{} sectors total, {} non-zero written", TOTAL_SECTORS, written);
}

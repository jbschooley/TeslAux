# Media controls from the car's own buttons

Goal: make the steering wheel's next / previous / play-pause control **whatever
is feeding the bridge** — a ScreenMate, a phone — rather than the car's own
media player.

Two obvious routes are ruled out by the use case rather than by difficulty:

* **Bluetooth.** The phone stays paired for *calls*, which is a large part of why
  this project exists: audio comes in over USB so Bluetooth is free for
  telephony. Pairing the source for AVRCP would fight that.
* **CAN.** The steering wheel already puts its button presses on the bus, and
  reading them is far less invasive than injecting. But it needs a physical tap
  at a point the OBD2 connector may not reach, and the frame IDs may not be in
  the community databases. Kept as a fallback, not a first move.

## The mechanism

Present the car with a **USB mass-storage device holding silent tracks**. The
car indexes it, plays it, and its transport buttons act on it. Because we author
the filesystem, a sector address identifies a track — so watching which sectors
the car reads says which track it moved to.

The tracks are not audio in any meaningful sense. They are a **position counter
that the car moves for us.** The real audio arrives over the microphone
endpoint, which the car mixes on top of whatever media is playing.

```
steering wheel -> CAN -> car's media player -> reads our synthetic drive
    -> we see the track change -> HID media key to the source -> track changes
    -> its audio arrives over the mic endpoint as before
```

Nothing leaves USB at either end.

### Why this reads the destination, not the presses

Press next five times quickly and the car ends up reading track 8 instead of
track 3. We see a jump of **+5** and send five media-next keys.

This is deliberately not an event counter. It does not matter whether the player
opens each intermediate file or jumps straight to the destination, and there is
no press to miss: the answer is always the difference from the last known
position. An event-counting scheme would be at the mercy of how the player
coalesces rapid input.

Drift between the two playlists does not matter either, because only *relative*
commands are ever sent. The drive's track 8 need not correspond to anything on
the source; it is a counter, not an index.

### Play/pause, and why it can be near-instant

The naive version fails: a player that reads in bursts and idles between them
looks identical to a paused one until the next read that never comes.

Both halves of the fix are ours to control:

* **Serve reads at just above real time.** The car cannot burst if we will not
  let it, so it reads steadily instead of in gulps.
* **Use a high-bitrate format.** 48 kHz/16-bit stereo is 192 KB/s, so a typical
  64 KB read lands about three times a second.

The car then reads continuously, and a pause stops the reads within roughly one
read interval — about 300 ms. Buffer depth stops mattering; what matters is that
reads are continuous rather than bursty.

## What Tesla actually does

Researched rather than assumed, though not yet confirmed on the car:

| Behaviour | Consequence |
|---|---|
| Plays WAV, MP3, FLAC; **not** AAC | WAV, which is also the high-bitrate option the read trick wants |
| Filesystem: "exFAT, MS-DOS FAT (for Mac), ext3, or ext4 (NTFS is currently not supported)" | FAT32 is "MS-DOS FAT" and is supported; exFAT is too, and is the community's choice for large music drives |
| **Re-indexes on every wake**, not just insertion | expect a scan burst each drive; the detector needs a settle period, and the volume should stay small |
| Orders by tag title, else filename; no other ordering | zero-padded numeric names make playback order equal track index |
| Missing tags fall back to filenames | no metadata needed; artwork simply does not appear |

**Confirmed on the car:** mic audio plays *over* other media, so the drive and
the bridge coexist. That was the assumption everything else rested on.

**Required settings:** shuffle **off** — a random next makes the delta
meaningless, and there is no workaround — and repeat-all **on**, so the playlist
never ends and stalls the mechanism.

## Edge cases

| Case | Handling |
|---|---|
| Rapid presses | inherent: the delta *is* the count |
| Wrap-around (track 18 + 5 → 3) | interpret modulo the track count, take the smaller magnitude |
| A track ending on its own | natural advance happens only after the player reads to the *end* of the file; a jump from the middle is a button press |
| Indexing at wake | settle period; act only once a sustained run is read from a track |
| First prev restarts the track | shows as a backwards seek *within* a file, not a jump — distinguishable |

## Why a separate device, not another interface

The car validates the TeslaMic descriptor set; the endpoint-less IF3 with its
exact 36-byte report descriptor is what stops the "unsupported USB microphone"
popup. Adding a mass-storage interface makes the device not-a-TeslaMic, and the
car may reject it.

A second USB port costs nothing by comparison. The mic stays a byte-for-byte
clone and the drive is just a drive.

## Status

**Built.** `rp2040/src/fat.rs` — the synthetic volume. Every sector is computed,
so a 2.3 GB volume has under 900 KB of actual content. `locate(lba)` maps a
sector to a track and offset, which is the function the detector is made of.

Verified by mounting it, which is a far better judge than tests written against
my own assumptions: macOS lists all 20 tracks and `ffprobe` reads each as a valid
600 s 48 kHz stereo WAV containing silence. Regenerate with
`tools/mkfatimg.rs`. Nine unit tests cover the layout invariants as well.

**Also built:** `rp2040/src/msc.rs` — USB Mass Storage, Bulk-Only Transport and
the SCSI subset a host actually issues to a read-only disk. `embassy-usb` has no
mass-storage class, so this is hand-rolled, like the IF3 handler.

It is pure logic over byte slices — no HAL, no endpoints, no async — because the
transport is a dozen lines of "read 31 bytes, maybe move data, write 13 bytes"
while every way this can go wrong lives in the command handling. 14 tests cover
the parts that bite: a malformed CBW must be rejected rather than guessed at,
`READ CAPACITY` reports the *last block* rather than the count, a read whose
`lba + blocks` overflows `u32` must not wrap into a valid range, sense data must
survive exactly one command, and a write must be refused as write-protected
rather than silently accepted.

**Also built:** `rp2040/src/bin/usbdrive.rs`, a standalone binary joining that to
`fat.rs`. Standalone deliberately — adding a mass-storage interface to the mic
would make it not-a-TeslaMic and risk the popup returning, and a second USB port
costs nothing by comparison.

One known divergence from the spec, noted rather than left to be rediscovered:
a failed command should stall the data endpoint, but `embassy-usb` exposes no
stall for bulk endpoints. Sending nothing and reporting the full amount as
residue tells the host the same thing.

**Not built:**

1. **The detector** — `locate()` across reads, natural-advance suppression,
   wrap-around modulo the track count.
2. **Read pacing** — serve just above real time so a pause shows in ~300 ms.
3. **A Consumer Control HID interface on the source side**, to send the media
   keys onward. Standard and well supported by any Android or Linux userspace;
   it touches only the source-facing descriptors, so the cloned mic side the car
   validates stays byte-identical.

**Verified on a Mac.** `teslamic-rp-USBDRIVE.uf2` enumerates and mounts as a
real disk:

```
/dev/disk4 (external, physical)   TESLAUX   2.3 GB
20 tracks; ffprobe reads 001.WAV as pcm_s16le 48000 Hz 2ch, 600.000000 s
4 KB read 50 MB into track 7: all zero, as it should be
sustained read: 957,643 bytes/sec
```

So the descriptors, Bulk-Only Transport, the SCSI subset and the synthetic FAT32
are all right, and a deep read exercises `locate()` across thousands of clusters
correctly. The car is now the only unknown.

### The volume is partitioned

It was originally a "superfloppy" — a FAT32 boot sector at LBA 0 with no
partition table. Desktop operating systems mount that happily, and macOS did,
but **real USB sticks are essentially always partitioned**, so an embedded media
player may never have been tested against one that is not.

There is now an MBR at LBA 0 with a single type `0x0C` (FAT32 LBA) partition
starting at sector 2048, the conventional 1 MiB alignment. macOS now sees
`FDisk_partition_scheme` -> `disk4s1 Windows_FAT_32` rather than a bare
filesystem, which is what a real stick looks like.

Two address spaces exist as a result, and conflating them is the obvious way to
break this: `read_sector` and `locate` take **device** addresses, while the
layout constants are **filesystem-relative**. `locate_fs` is the internal form
for code that has already subtracted the offset — subtracting it twice
attributes every sector to the wrong track, silently.

### Measured: the car reads the tracks and still will not play them

With the LED distinguishing metadata reads from track reads:

| Host | Result |
|---|---|
| Mac | amber — directory only, which is right: listing a folder opens no file |
| Car | **green — read inside a track** |

So the car parses the MBR, mounts FAT32, walks the directory, finds the files
and opens at least one. **The filesystem is accepted; exFAT is not the
problem**, and neither is the partition table, the mass-storage class or the
SCSI subset.

Whatever the car objects to is about the files or the volume, not about being
able to read them.

### If the car does not offer it as a media source

Owner reports converge on one cause that has nothing to do with the volume:
**a dashcam drive suppresses USB music.** When a drive carrying a `TeslaCam`
folder is present, the USB music option disappears — and a second drive in
another port does not bring it back. Tesla appears to pick a single storage
device for media, and dashcam wins.

Using a hub makes it worse on its own: the dashcam icon vanishing and the music
option disappearing are both commonly reported, needing a reset to recover.

So the first test is free: **unplug the Sentry drive and leave only this one.**

Two other things that are not the problem here, but are worth knowing because
they are cheap to get wrong:

* **No folder structure is needed.** Files at the root are indexed by tag or
  filename, which is what this volume does.
* **A `Lightshow` folder anywhere on the stick makes Tesla ignore all music.**
  Not present here, but it shows how easily the media source is suppressed by
  something unrelated to the audio.

### If the car rejects the volume

FAT32 is listed as supported by Tesla's own service documentation, so it is
worth testing before anything is rebuilt. Two things to try first, in order,
because both are far cheaper than the alternative:

**Grow the volume.** Ours is 2.3 GB, and Tesla's documentation mentions a 64 GB
minimum — that figure is for Dashcam rather than music, and small music drives
are used routinely, but it is the cheapest thing to rule out. Every sector is
synthesised, so more tracks or longer ones cost no flash at all: change
`N_TRACKS` or `TRACK_SECONDS` in `fat.rs`. More tracks is useful regardless,
since the track count *is* the range of the counter for rapid button presses.

**Then exFAT.** A real job rather than a tweak: an upcase table, an allocation
bitmap, and directory *entry sets* with checksums, against FAT32's flat table
and 32-byte entries. Worth doing only once FAT32 has actually been shown to
fail, since the documentation says it should not.

### Read pacing is load-bearing, not polish

That throughput measurement changes a design assumption. **958 KB/s is about
five times real time** — 48 kHz/16-bit stereo playback needs 192 KB/s. A car
reading that fast can buffer five seconds of audio for every second it plays, so
"reads stopped" would lag a pause by five seconds or more, against the ~300 ms
this design assumed.

Pacing the reads to just above real time is therefore what makes play/pause
detection work at all, and the ratio it has to correct is now measured rather
than guessed.

## Ruled out: the car tells the mic nothing

Tested directly on the STM32 build, with `embassy-usb` control tracing running
while the steering wheel's next, previous, play/pause and volume buttons were
pressed, and **with audio confirmed playing throughout** so the device was
certainly live: **zero control requests.** The car sends a connected microphone
nothing at all in response to its own transport buttons.

That matches what the enumeration trace shows about IF3. The car writes about 73
`A5 5A` frames when it first sees the mic, and the shape is now clear:

```
a5 5a | id | len | ff | payload... | 16
```

IDs `00`-`0d` then `81`-`92`+, all host-to-device, and **not one `GET_REPORT`** —
the car never reads from us. The payloads look like DSP parameters, gains and
thresholds: a one-time configuration dump, not an event channel. A microphone
receives settings and sends audio, and nothing in the protocol carries a button
press in either direction.

So the synthetic drive is not a workaround for a channel we failed to find. It
is the only channel there is.

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

**Not built:**

1. **USB Mass Storage class** — Bulk-Only Transport plus a SCSI subset
   (`INQUIRY`, `READ CAPACITY`, `READ(10)`, `TEST UNIT READY`, `REQUEST SENSE`,
   `MODE SENSE`). `embassy-usb` has no MSC class, so hand-rolled, much like the
   IF3 handler.
2. **A standalone `usbdrive` binary**, to plug into a second car port and
   confirm the car indexes it and the wheel drives it — before any of this goes
   near the TeslaMic. Testable on a Mac first: if it mounts as a *device* the way
   the image mounts as a *file*, the class implementation is right.
3. **The detector** — `locate()` across reads, natural-advance suppression,
   wrap-around modulo the track count.
4. **Read pacing** — serve just above real time so a pause shows in ~300 ms.
5. **A Consumer Control HID interface on the source side**, to send the media
   keys onward. Standard and well supported by any Android or Linux userspace;
   it touches only the source-facing descriptors, so the cloned mic side the car
   validates stays byte-identical.

## Still worth ruling out first

Latching any control traffic the car sends to IF2/IF3 and then pressing the
media buttons. The control-channel capture in `RESEARCH.md` shows the car sends
nothing after enumeration, but nobody was pressing buttons at the time.

Odds are low — in HID, media keys flow device-to-host, and the car is the host,
so it would be sending a keyboard LED state at most. But it is an hour of work
against a mass-storage stack.

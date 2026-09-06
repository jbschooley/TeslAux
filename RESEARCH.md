# TeslaMic reverse-engineering — findings & next steps

Everything we've learned about Tesla's USB cabin-mic (`TeslaMic`, VID `1235:0002`)
from emulating it on a Heltec T114, and the exact plan for finishing the job once
a real mic + USB sniffer are in hand. Source thread:
<https://www.reddit.com/r/hardwarehacking/comments/1ok256p/going_down_a_rabbit_hole_wondering_where_to_start/>

---

## 1. Device identity (from the Reddit dump, confirmed by our emulator)

| Field | Value |
|-------|-------|
| VID:PID | `1235:0002` (registered to Focusrite/Novation — TeslaMic borrows it) |
| Manufacturer string | `TeslaMic_T004_OTA_231008` |
| Product string | `TeslaMic` |
| Max power | 500 mA, bus-powered |
| Interface tree | **4 interfaces**: IF0 AudioControl, IF1 AudioStreaming, IF2 HID, IF3 HID |
| Audio | USB Audio Class 1.0, iso IN, 192 B / 1 ms → 48 kHz / 16-bit / stereo |
| IF2 HID | 8-byte interrupt IN, 1 ms — "status telemetry" |
| IF3 HID | second HID — the **settings / OTA config** channel |

The `_OTA_` in the manufacturer string + the config protocol below → IF3 is a
firmware/config channel.

---

## 2. What our emulator achieved (all working)

- **Mic icon appears** in the car; audio plays through the cabin speakers.
- Enumerates on macOS as a 2-ch / 48 kHz input, with the IF2/IF3 HID interfaces.
- **Format support (RETESTED 2026-09-02, after fixing the ISO-IN DMA race and
  applying nRF52840 erratum 166)** — Tesla is **stereo, but not rate-fussy**:
  32 k, 44.1 k, 48 k and 96 k all play clean. The earlier readings below were
  taken with those two bugs present and were measuring our own corruption:
  - 48 kHz & 96 kHz at 16-bit: clean.
  - **24-bit: NOT established.** The July "clean" reading was taken with both
    transport bugs present, using a 1 kHz tone at 48 kHz — one cycle per USB
    packet, and so blind to the packet rotation by construction. That same
    signal called 48 kHz clean while every packet was being corrupted, so it
    proves nothing about 24-bit either. Rebuilt 24-bit images with a 997 Hz tone
    are in `t114/` if it is worth settling.
  - 192 kHz: no output (beyond the car's max rate).
  - 44.1 kHz: **clean** (the July "buzzes" reading was our fractional-packet
    handling being corrupted, not the car resampling).
  - >2 channels (4ch/5ch): only the first 2 channels ever play → stereo-max.
  - Frequency sweep 50 Hz–20 kHz came through full-range → **not heavily
    band-limited** (good for real/music audio, not just voice).
  - **Recommended format: 48 kHz stereo 16-bit** — what the real mic uses.
- ~100 ms button-to-sound latency is Tesla's own audio buffering, not the device.

---

## 3. The "unsupported USB microphone" popup — SOLVED ✅ (2026-07-14)

**Root cause:** our HID descriptors didn't match a real TeslaMic. Dumping a
working clone over libusb (see `real_mic_dump.md`) showed IF2 is a HID **keyboard**
and IF3 is an **endpoint-less** vendor HID (Usage 0x55AA) with a specific 36-byte
report descriptor + 8-byte Feature report `0001000303000800` that the car
validates. Cloning those exactly (plus the serial) → the car accepts it with **no
popup**. History of the investigation is kept below for reference.

The car shows the icon + plays audio, then (before the fix) after **~60 s** popped
"unsupported USB microphone" and disconnected.

### 3.1 What we tried (all still popped)
1. Enumeration only.
2. + HID heartbeat on IF2 (rolling counter).
3. + HID request handler (answer GET_REPORT/SET_REPORT instead of STALL).
4. + **IF3** (second HID interface) — this changed the car's behavior a lot (see below), but still popped.

### 3.2 The full control-channel handshake (captured with our on-screen USB spy)

We built `teslamic-spy.uf2` (feature `usb-spy`) — it logs every USB **control**
request the car sends to the onboard TFT. Complete captured connect sequence:

```
SET_IF if0 alt0
SET_IF if1 alt0
SET_IF if2 alt0
SET_IF if3 alt0
CONFIGURED=1
O 21 r0a v0000 i2 l0        SET_IDLE → IF2
I 81 r06 v2200 i2 l121      GET_DESCRIPTOR(HID Report) IF2, wLength = 0x121 = 289
O 21 r0a v0000 i3 l0        SET_IDLE → IF3
I 81 r06 v2200 i3 l121      GET_DESCRIPTOR(HID Report) IF3, wLength = 289
  (SET_IDLE + GET-report-desc on IF3 sometimes repeats once)
S i3 l9  a5 5a fc 04 b0 b0 00 00 f1 (16?)   SET_REPORT → IF3 (Output report, wValue 0x0200)
S i3 l9  a5 5a fc 04 b0 b0 00 10 (01 16?)
S i3 l9  a5 5a fc 04 b0 b0 00 20 (01 16?)
S i3 l8  a5 5a fc 03 c0 aa 01 (16?)
S i3 l12 a5 5a 00 07 31 00 01 02 02 17 00 (16?)
S i3 l18 a5 5a 01 0d ff 00 00 00 00 01 00 80 00 10 00 00 (16?)
  ... then only AudioStreaming alt1/alt0 toggling forever.
  NO GET_REPORT read-back after the writes.
```
(Bytes in parens are uncertain — read off blurry photos of a 240×135 screen;
verify against a real capture. Several frames appear to end in `16`, possibly a
checksum/terminator.)

Request decoding: `bmRequestType` `0x21` = host→device/class/interface,
`0x81` = device→host/standard/interface, `0xA1` = device→host/class/interface.
`bReq` `0x0a`=SET_IDLE, `0x06`=GET_DESCRIPTOR, `0x09`=SET_REPORT, `0x01`=GET_REPORT.
`wValue` `0x2200` = HID Report descriptor; `0x0200` = Output report, ID 0.

### 3.3 The IF3 config protocol (`A5 5A` framed) — DECODED

Captured in the car with `--features if3-log`, by moving each control in turn.
534 frames. The frame is:

```
A5 5A | type | len | payload[len] | 16
```

`len` counts the payload only. Two earlier readings here were wrong and are
corrected below: the `fc 04 b0 b0 00 NN` frames are not chunked writes to an
offset, and the incrementing third byte is not a sequence number.

**The UI controls.** `A5 5A FC 04 B0 B0 <cc> <v> 16`, where `<v>` is 0x00-0x0F —
sixteen steps — and `<cc>` selects the control:

| `<cc>` | control |
|---|---|
| `0x00` | **volume**, default 0x0A |
| `0x01` + `0x02` | **reverb**, sent as a pair, one frame each per step |

`0x00` is the volume, not specifically the mic slider: the car's master volume
roller sends the same control, sweeping the same 0x00-0x0F. Whether the two UI
elements are one value or the car maps both onto it cannot be told from this
side — either way the device sees a single volume number.

So the car's mic volume slider is a 0-15 step sent here, not the audio class
Feature Unit, and the reverb slider drives two channels together. What was read
as an offset walking `00 → 10 → 20` was the slider being dragged.

**The effect presets.** Tapping an effect re-sends the entire DSP configuration:
types `0x00`, `0x01`, `0x03`, `0x04`, `0x06`-`0x0D`, then `A5 5A FC 05 FF` +
ASCII **`YASB`**, then a parameter sweep from `0x81` to `0xB7`, then
`FC 03 C0 AA 01` to commit. The incrementing byte is a **parameter ID**, not a
sequence number — the car walks the register map in order.

The block is identical every time except for the frames that carry the preset
itself. Those are the ones worth knowing:

| type | len | distinct values seen |
|---|---|---|
| `0x89` | 18 | 4 |
| `0x8B` | 22 | 4 |
| `0xA9`, `0xAA`, `0xAB` | 12 | 3-4 |

Four distinct values across four effects tapped. Everything else in the block is
constant, including eight 112-byte blobs at `0xA0`-`0xA7`.

`FC 03 C0 AA 01` / `... 00` bracket the block — start and commit.

**What this means.** Two things this project wanted are sitting in here:

* The **mic volume slider** can be honoured. Treating `<v>` as attenuation with
  0x0F as unity gives a working volume control that is bit-exact at maximum and
  never clips, unlike gain.
* The **effect buttons are a control channel**. Each tap is a distinct,
  recognisable frame from the car's own UI, which is what the media-control work
  went looking for and could not find over the mic channel.

**Media transport sends nothing.** Play, pause, next and previous, from the
steering wheel and from the screen, produce not one frame. The mic is never told
the track changed, which is reasonable — it has no reason to know — but it does
mean IF3 offers no transport events, and the effect buttons are the only UI
control that reaches the device.

It does **not** offer more volume. The real mic already reports maxed samples at
its top step, and this device sends full scale regardless, so both arrive at the
car's mixer at the same level and hit the same ceiling.

### 3.35 The volume ceiling is anti-howling, and it is not reachable

Playing through the mic tops out around 60% of the car's volume scale. Chased it
from both ends; it is not the mic path being mixed quietly.

**It clamps everything, not the mic.** With the mic connected, Spotify stops
getting louder past the same point. Unplug the mic and the car goes much louder.

**It is device presence, not a live mic.** The real receiver with both handhelds
switched off clamps just the same.

**It latches when the volume is set, not continuously.** Raise the volume first,
then connect: it stays loud, and keeps playing loud, until the volume is touched
again — at which point everything drops to the ceiling while the display carries
on reading higher.

Tesla calls the feature "anti-howling" on the CaraokeMic's product page.
Limiting the speakers whenever a microphone-class device is in the cabin is
exactly what that is for, and the car applies it bluntly: any CaraokeMic
present, live or not.

Things tried, all measured, none of which move it:

| tried | result |
|---|---|
| Feature Unit advertising +30 dB, 0 dB, then -60..-20 dB | No audible difference in any of the three |
| The real mic's own range | 0 to +16 dB, already pinned at maximum |
| Input Terminal as Line Connector (0x0603) rather than Microphone | Still recognised as a CaraokeMic, still clamped |
| Both handhelds powered off | Still clamped |
| Mic volume slider (IF3 `cc=0x00`) at maximum | Already there |

The Feature Unit is decorative here. The car queries the range, writes the
maximum to both channels and reads one back — faithfully, every time, whether
that maximum is +30 dB, 0 dB or -20 dB — and then does nothing with it. Claiming
headroom does not release any, claiming none does not make the car compensate,
and claiming to be 20 dB short of unity does not make it boost.

Two HID paths are dead as well: the car acts on neither Keyboard-page volume
usages (0x80/0x81) nor Consumer-page ones (0xE9), sent from the interrupt IN
endpoint that is the mic's own button.

The last one closes it from the other side: the real mic reports maxed samples
at its top step, and this device sends full scale regardless, so both arrive at
the mixer at the same level and meet the same ceiling.

**And it is not about being a CaraokeMic at all.** Taking the identity apart one
field at a time:

| build | recognised? | audio? | IF3 config block | clamp |
|---|---|---|---|---|
| Line Connector terminal type | yes | yes | sent | yes |
| IF3 refused (`c0 aa` stalled) | yes | yes | sent anyway | yes |
| IF3 omitted entirely | yes | yes | none to send | yes |
| VID/PID `1209:0001` | yes | **yes** | **none sent** | yes |

The last row is the finding. A generic test VID/PID, with the car sending no
karaoke configuration whatsoever — so not treated as a CaraokeMic — still gets
its audio played, and still clamps.

So recognition is keyed on the **descriptor shape**: a UAC1 input with the right
format, rates and endpoint. Not the identity, not IF3, not the terminal type. A
random USB audio device that the car ignores is being rejected for its format,
not for who it claims to be.

Two things in this table also correct earlier sections. IF3 is **not** what
defeats the "unsupported USB microphone" popup — audio works without it and no
popup appears — and it is not "the descriptor the car validates". Either that was
never the operative factor or the car's firmware has moved since it was written.

Which closes the search rather than narrowing it: **anti-howling applies to any
USB audio input the car accepts**, and there is no identity or descriptor that is
accepted without it. Looking for a different device to imitate is looking for
something that does not exist.

What remains is a workaround rather than a fix: **set the volume before the
bridge is connected**, and do not touch it afterwards. The clamp latches on
change, so a level set while nothing is plugged in survives.

Full volume means leaving the mic path for the media path, which is not clamped
because a mass-storage device is not an audio input. That trades away the
overlay — the reason the mic path was chosen — and inherits the car's read-ahead
as latency.

### 3.36 The car's USB scheduling is not costing us anything

Measured in the car with `--features usb-timing`, which buckets the gap between
successful isochronous writes. A packet should leave every 1 ms.

| | no Sentry drive | Sentry drive recording |
|---|---|---|
| packets per 5 s window | 5000 | 5000 |
| on time (<=1.5 ms) | **5000 / 5000** | **5000 / 5000** |
| worst gap | 1008-1009 us | 1007-1040 us |
| write timeouts | 8, from enumeration | 8, unchanged |

Not one late packet in either condition, over many windows. The worst case moved
by 31 microseconds against a 1000 microsecond budget.

This was worth measuring because there was a real mechanism to suspect: a
full-speed device behind a high-speed hub has its isochronous traffic carried as
split transactions through the hub's transaction translator, sharing a budget
with every other full-speed device — a known source of isochronous jitter, and
the argument for moving to a high-speed part such as an STM32F723 or a Teensy
4.1. It is not happening here. The car polls this device like a metronome
whether or not it is writing video to a drive on the same bus.

So high speed would buy nothing for jitter, and nothing meaningful for latency
either: the car's own pipeline adds about 100 ms and the phone another 10-50,
against a few milliseconds of ours.

What this does not cover is what the car does with the packets after collecting
them. Every bit-exact recording here was captured on a Mac, so the car's
downstream processing remains unmeasured — but that is also beyond anything a
faster bridge could change.

### 3.4 Why we're blocked (the wall)
Because there's **no `GET_REPORT` after the writes**, the accept/reject decision
happens on channels our device-side control spy **cannot observe**:
1. **The interrupt-IN endpoints** (IF2 8-byte telemetry, and possibly IF3). The
   car polls these over *interrupt* transfers, which are **not** control requests,
   so the spy can't see them. The real mic answers with specific `A5 5A`
   status/ACK reports; our rolling-counter heartbeat doesn't match → the ~60 s
   watchdog fails.
2. **The 289-byte HID report descriptor content**, which defines those report
   formats. Ours is a 21-byte generic stub; we can't reconstruct 289 bytes blind.
   (Note: the car requests 289 bytes even though our HID descriptor advertises 21
   → it has a **TeslaMic-specific driver with hardcoded expectations**.)

Both require observing a **real mic** — this is the limit of blind RE.

---

## 4. NEXT STEPS — when the mic + sniffer arrive

### 4.1 Buy
- A genuine or clone **TeslaMic / Caraoke mic / PureMic**.
- **Stage 1 (free):** any **Linux** box (Raspberry Pi 4/5 ideal) for usbmon/usbhid-dump.
- **Stage 2 (inline capture):** **Cynthion** (Great Scott Gadgets, ~$150) — USB 2.0
  full-speed analyzer, Wireshark support. (Or Total Phase Beagle USB 12, ~$400+.)
  Avoid generic $20 "USB analyzers".

### 4.2 Stage 1 — dump the mic on Linux (gets most of what we need)
```sh
lsusb -v -d 1235:0002                 # full descriptors, HID descriptor lengths
sudo usbhid-dump -d 1235:0002         # the actual 289-byte report descriptors (IF2 + IF3)  <-- KEY
sudo modprobe usbmon
# then capture in Wireshark on the usbmonN interface for the mic's bus while
# plugging the mic in; watch the interrupt-IN reports the mic sends unprompted.
```
Save the report-descriptor bytes and any interrupt-IN report payloads.

### 4.3 Stage 2 — sniff the car ↔ mic exchange (Cynthion inline)
Put the analyzer between the car's USB port and the real mic. Capture a full
session (connect → ~60 s). Extract, in order:
1. The **IF2 & IF3 interrupt-IN reports** the mic sends (format + cadence) — this
   is what our watchdog-failing heartbeat must match.
2. The car's `A5 5A` **SET_REPORT** writes to IF3 **and the mic's response** (does
   it ACK on IF3 interrupt-IN? with what?).
3. Whether the mic's reports are `A5 5A`-framed (likely) and their exact fields.

### 4.4 Then implement in this firmware
- Replace `HID_REPORT_DESCRIPTOR` (currently a 21-byte stub in `t114/src/main.rs`) with
  the real 289-byte descriptor(s) for IF2 and IF3.
- Replace the rolling-counter heartbeat with the mic's real IF2 telemetry report(s).
- Handle the IF3 `A5 5A` config writes and emit the correct ACK/status on IF3.
- Re-test in the car; the ~60 s watchdog should then be satisfied.

The `usb-spy` build stays useful throughout for confirming the car's control-side
behavior matches.

---

## 5. Emulator build reference (this repo)

Firmware for the Heltec T114 (nRF52840), Rust/embassy, flashes via UF2
(double-tap RESET → drag the `.uf2`). See `README.md` for the full build/flash/
format-matrix details. Key builds:

| UF2 | Purpose |
|-----|---------|
| `teslamic.uf2` | mic + silence |
| `teslamic-sine.uf2` | mic + button tone (L/R per press) |
| `teslamic-hid.uf2` | mic + 2 HID interfaces + heartbeat (popup attempt) |
| `teslamic-spy.uf2` | **USB control-request logger on the onboard TFT** (the RE tool) |
| `teslamic-{44k,96k,192k,mono,4ch,5ch,*-24bit,sweep}.uf2` | format tests |

Audio format is a build-time knob: `TESLAMIC_RATE/CHANNELS/BITS/CHMASK`.
`teslamic-spy.uf2` shows the newest ~15 control events; it suppresses the
audio-alt and SET_REPORT floods so the handshake stays visible.

Reminder of the physics limit: nRF52840 USB is **full-speed**, so one iso packet
is ≤ 1023 bytes/frame — `(rate/1000) × channels × bytes ≤ 1023` (enforced in
`build.rs`). embassy-nrf 0.10 can't write the iso endpoint via its driver, so the
firmware drives the `USBD.ISOIN` registers directly, armed once per SOF.

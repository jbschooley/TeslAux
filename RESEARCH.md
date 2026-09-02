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
- **Format support (tested in-car)** — Tesla is effectively **stereo, 48 kHz-family**:
  - 48 kHz & 96 kHz: clean (16- and 24-bit both fine).
  - 192 kHz: no output (beyond the car's max rate).
  - 44.1 kHz: plays but buzzes (only rate outside the 48 k family).
  - >2 channels (4ch/5ch): only the first 2 channels ever play → stereo-max.
  - Frequency sweep 50 Hz–20 kHz came through full-range → **not heavily
    band-limited** (good for real/music audio, not just voice).
  - **Recommended format: 48 kHz or 96 kHz, stereo, 16- or 24-bit.**
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

### 3.3 The IF3 config protocol (`A5 5A` framed)
- Magic header **`A5 5A`**, then a command/type byte (`fc`, `00`, `01`, …).
- The `fc 04 b0 b0 00 NN` frames look like **offset-addressed chunked writes**
  (offset `0x0000` → `0x0010` → `0x0020`, +0x10 each). Likely writing a config or
  parameter blob in 16-byte chunks.
- Variable report lengths (9, 8, 12, 18 bytes) → multiple message/command types.
- We `Accept` every write; the car never reads back over the control channel.

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
- Replace `HID_REPORT_DESCRIPTOR` (currently a 21-byte stub in `src/main.rs`) with
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

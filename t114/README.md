# T114 / nRF52840 — the original prototype

**This is not the firmware to use.** It is where the project started, and where
the hard problems were solved: the descriptor set the car validates, and both
isochronous transport bugs. The shipping firmware is in [`../rp2040`](../rp2040).

It still works, and it is still the most car-proven code here — it has been
accepted by a real Tesla since July. It is kept because:

* it is the reference the RP2040 descriptors were copied from, byte for byte;
* it has a hardware I2S slave peripheral, so it needs no PIO, which makes it a
  useful cross-check if the RP2040's I2S is ever suspect;
* it has an onboard TFT, which is what the `usb-spy` build draws the car's USB
  control requests on. That is how the IF3 config protocol was found.

What it does **not** have is an audio input path — it emulates the mic and
generates test tones, but nothing wires I2S capture into its isochronous pump.
That work went into the RP2040 firmware instead.

Hardware: Heltec Mesh Node T114 (nRF52840 + SX1262). Flash by double-tapping
RESET and dragging a `.uf2` onto the drive that appears.

---

Firmware that makes a **Heltec Mesh Node T114** enumerate over USB as Tesla's
cabin microphone, to test whether the car will show its **MIC overlay icon**.

Identity + descriptors are cloned from the TeslaMic dump in the [r/hardwarehacking
thread](https://www.reddit.com/r/hardwarehacking/comments/1ok256p/going_down_a_rabbit_hole_wondering_where_to_start/):

| Field | Value |
|-------|-------|
| VID:PID | `1235:0002` |
| Manufacturer | `TeslaMic_T004_OTA_231008` |
| Product | `TeslaMic` |
| Max power | 500 mA |
| Audio | USB Audio Class 1.0 — 48 kHz, 16-bit, **stereo**, 192 B/1 ms iso IN |

## Status — WORKING ✅ (popup solved 2026-07-14)

`teslamic-hid.uf2` runs in the car with the **mic icon, working audio, and no
"unsupported USB microphone" popup / disconnect**. The HID interfaces are a
byte-exact clone of a real TeslaMic (dumped over libusb — see `real_mic_dump.md`):
IF2 is a HID keyboard, IF3 is an endpoint-less vendor HID (Usage 0x55AA) whose
report descriptor + Feature report the car validates. It also enumerates on macOS
as a 2-channel / 48 kHz input named `TeslaMic`.

Remaining work is product polish (an I²S line-in front-end to stream real audio)
and the ~100 ms **Tesla-side** latency (not fixable on the device — our end is
~1–2 ms; it's the car's audio pipeline).

Four flashable builds:

| File | Behaviour |
|------|-----------|
| **`teslamic.uf2`** | Enumerates **and streams silence** — 192 B of zeros per USB frame out the iso endpoint, so it looks like a mic that's actively capturing. Try this in the car. |
| **`teslamic-sine.uf2`** | Same, but emits a **1 kHz sine while the USER button (P1_10) is held**, silence when released. Each new press **alternates the channel**: press 1 = LEFT, press 2 = RIGHT, press 3 = LEFT, … (other channel silent). Good for confirming the audio path and checking L/R routing. |
| **`teslamic-hid.uf2`** | Everything: the button-triggered L/R sine **plus a HID interface (IF2) streaming an 8-byte heartbeat** — an attempt to defeat the ~60 s "unsupported USB microphone" popup (see [The popup](#the-popup-hid-heartbeat)). Built with `--features hid-heartbeat,sine-button`. |
| `teslamic-enum-only.uf2` | Enumerates only (no iso data). Fallback if a streaming build misbehaves. Tested to enumerate on macOS. |

## The popup (HID heartbeat)

The genuine TeslaMic exposes two HID interfaces (an 8-byte interrupt-IN status
stream + a control/feature "settings" interface) that this project's audio-only
builds don't. The car shows the mic icon and plays audio, but after ~60 s pops
"unsupported USB microphone" — most likely a **keepalive/telemetry watchdog**
timing out (a static descriptor check would fail instantly, not after a minute).

`teslamic-hid.uf2` adds **IF2** (8-byte interrupt IN, `poll_ms = 1`, matching the
dump) and streams a report continuously, with a rolling counter in byte 0 so a
liveness watchdog sees changing data.

**This is a blind guess.** The dump doesn't include the HID report descriptors or
the report *contents*, so if the car validates specific values (rather than just
"is something reporting"), this won't be enough — it would need a `usbhid-dump` +
USB trace from a real or clone TeslaMic. The IF3 "settings" interface is also not
implemented yet. Combine with the tone build via
`--features hid-heartbeat,sine-button` if you want to test audio and the popup
together.

Tesla: Sees the device as a microphone input and shows the mic icon. The sine wave plays through the car's audio system.

However, there is a noticeable delay (estimating around 100ms) between pressing the button and hearing the sine wave. This is much better than Bluetooth, but not instantaneous enough for real-time audio (like playing an instrument). The only way to fix this would be to get Tesla's engineers to reduce the buffering. I'm surprised they haven't done this already because the delay is noticeable when using the official Tesla Caraoke Mic.

After a minute, it disconnects and the car shows an unsupported USB microphone popup. I assume this is because the TeslaMic firmware is sending some telemetry over the HID interface that this firmware does not implement. I don't have an official or clone mic to use to reverse engineer this interface yet.

## USB spy — on-screen request logger (`teslamic-spy.uf2`)

To reverse-engineer the popup without a real mic, this build turns the T114 into
its own USB analyzer. It's the full mic + HID device (`--features usb-spy`, which
implies `hid-heartbeat`), plus a spy that **logs every USB control request the car
sends to the onboard ST7789 TFT in real time**. Flash it, plug into the car, and
watch the screen (it lights ~1.5 s after boot).

Each line is one control request/event:

```
CONFIGURED=1            device configured
SET_IF if1 alt1         host selected AudioStreaming alt-1 (starts audio)
O 21 r09 v0200 i2 l8 d05   ctrl-OUT bmReqType=0x21 bReq=0x09(SET_REPORT)
                            wValue=0x0200 iface=2 len=8 firstbyte=0x05
I a1 r01 v0100 i2 l8       ctrl-IN  bmReqType=0xA1 bReq=0x01(GET_REPORT) ...
```

`bmReqType` decodes as: `0x21` = host→device, class, interface; `0xA1` = the
device→host read. Anything the car sends to **interface 2** (our HID) — especially
`SET_REPORT`/`GET_REPORT` with their `wValue` and data byte — is the handshake
we're missing. Read those off the screen (or photograph it) right up to the popup,
and we can implement the HID interface to match instead of guessing.

Notes/limits: the spy observes **class/vendor** requests (the interesting ones) plus
config + alt-setting changes; it doesn't see plain standard requests. The screen
shows the most recent ~11 events; the ring buffer holds 32.

## Format test builds

The audio format is a **build-time parameter** (`build.rs` reads env vars), so any
feasible rate / channel count / bit depth is one build away. All these use
`sine-button`, so the tone plays while the USER button is held; each press steps
the tone to the **next channel** (at 2ch = L/R; at 4ch = cycles all four), which
is how you check the car's channel mapping.

| File | Format | Car result | Env |
|------|--------|-----------|-----|
| `teslamic-48k-24bit.uf2` | 48 kHz / 2ch / 24   | ❔ July result is uninformative, not retested | `TESLAMIC_BITS=24` |
| `teslamic-96k.uf2`     | 96 kHz / 2ch / 16   | ✅ clean | `TESLAMIC_RATE=96000` |
| `teslamic-96k-24bit.uf2` | 96 kHz / 2ch / 24   | ❔ July result is uninformative, not retested | `TESLAMIC_RATE=96000 TESLAMIC_BITS=24` |
| `teslamic-mono.uf2`    | 48 kHz / 1ch / 16   | ✅ works | `TESLAMIC_CHANNELS=1` |
| `teslamic-sweep.uf2`   | 48 kHz / 2ch / 24, **freq sweep** | ❔ measured under the transport bugs | `--features sweep TESLAMIC_BITS=24` |
| `teslamic-44k.uf2`     | 44.1 kHz / 2ch / 16 | ✅ **clean** (retested 2026-09-02) | `TESLAMIC_RATE=44100` |
| `teslamic-32k.uf2`     | 32 kHz / 2ch / 16   | ✅ clean (retested 2026-09-02) | `TESLAMIC_RATE=32000` |
| `teslamic-44k-24bit.uf2` | 44.1 kHz / 2ch / 24 | ❔ not retested since the fixes | `TESLAMIC_RATE=44100 TESLAMIC_BITS=24` |
| `teslamic-4ch.uf2`     | 48 kHz / 4ch / 16   | ⚠️ only ch 1–2 play | `TESLAMIC_CHANNELS=4` |
| `teslamic-96k-4ch.uf2` | 96 kHz / 4ch / 16   | ⚠️ only ch 1–2 play | `TESLAMIC_RATE=96000 TESLAMIC_CHANNELS=4` |
| `teslamic-5ch.uf2`     | 48 kHz / **5.0 surround** / 16 | ⚠️ only ch 1–2 play | `TESLAMIC_CHANNELS=5 TESLAMIC_CHMASK=0x37` |
| `teslamic-192k.uf2`    | 192 kHz / 2ch / 16  | ❔ "no output" in July; not retested since the fixes | `TESLAMIC_RATE=192000` |

Column key: ✅ works cleanly · ⚠️ works with a caveat · ❌ no audio.

### What the results tell us

> **Corrected 2026-09-02.** The original matrix was measured with two firmware
> bugs present (an ISO-IN DMA race, and nRF52840 erratum 166 unapplied). Both
> corrupted the audio *we* transmitted, so several "Tesla can't do this"
> conclusions were actually our own faults. Rates marked ❔ were measured under
> those bugs and have not been retested.

- **Tesla is NOT 48 kHz-family only.** With the transport fixed, **32 kHz,
  44.1 kHz, 48 kHz and 96 kHz all play cleanly.** The old "44.1 kHz buzzes"
  finding was our bug: 44.1 is the one standard rate that doesn't divide into
  1 ms frames, so it alternates 44/45 samples per packet, and varying the packet
  size was exactly what the two bugs corrupted.
- **Variable packet sizes are fine.** `teslamic-stress-variable.uf2` changes the
  packet size on *every frame* — ~500x harsher than real clock drift — and plays
  clean. Elastic pacing against a drifting source is therefore viable.
- **Tesla is stereo-max.** Every multichannel build (4ch / 5ch) plays only the
  first two channels. Measured before the fixes, but the fixes cannot plausibly
  affect channel *mapping*, so this one still stands.
- **Not heavily processed.** The 50 Hz–20 kHz sweep comes through full-range with
  no obvious roll-off, so the car isn't aggressively band-limiting the mic input —
  promising for piping real/music audio, not just voice.
- **Recommended format: 48 kHz stereo 16-bit** — not because the others fail,
  but because it is exactly what the real TeslaMic advertises.

`TESLAMIC_CHMASK` overrides the `wChannelConfig` spatial-position bitmap (default =
low `channels` bits). The **sweep** build ignores per-channel stepping: while the
button is held it steps a tone through 50 Hz → 20 kHz on all channels (~0.35 s
each, restarting each hold), so you can hear where the car's processing rolls off.

Build any format: `TESLAMIC_RATE=… TESLAMIC_CHANNELS=… TESLAMIC_BITS=… cargo build --release --features sine-button` (defaults 48000 / 2 / 16).

**Hardware limit:** the nRF52840 USB is full-speed, so one isochronous packet is
≤ 1023 bytes/frame — i.e. `(rate/1000) × channels × (bits/8) ≤ 1023`. `build.rs`
rejects anything over that at compile time. Consequences: 192 kHz caps at 2ch/16-bit,
4ch caps at 96 kHz, and 24-bit caps at 96 kHz/2ch. (192 kHz/4ch, 96 kHz/8ch, and
192 kHz/24-bit are impossible on this chip.)

## Build

Requires the Rust toolchain (pinned in `rust-toolchain.toml`) and
`arm-none-eabi-objcopy` (from `arm-none-eabi-gcc` / `brew install arm-none-eabi-gcc`).

Each build is a Cargo feature of the same source. Compile, then wrap the ELF
into a UF2 at the app offset `0x26000`:

```sh
# pick ONE build:
cargo build --release                          # -> silence  (teslamic.uf2)
cargo build --release --features sine-button   # -> sine/button (teslamic-sine.uf2)
cargo build --release --features hid-heartbeat,sine-button # -> button sine + HID heartbeat (teslamic-hid.uf2)
cargo build --release --no-default-features     # -> enumerate only (teslamic-enum-only.uf2)

# then package the resulting ELF:
arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/teslamic teslamic.bin
python3 ../tools/uf2conv.py teslamic.bin -c -b 0x26000 -f 0xADA52840 -o teslamic.uf2
```

All four prebuilt `*.uf2` files are checked in.

## Flash it (no debug probe needed)

The T114 ships with the Adafruit/Nordic UF2 bootloader.

1. **Double-tap the RESET button** on the T114. A USB drive named something like
   `T114BOOT` / `NRF52BOOT` appears on your Mac.
2. **Drag `teslamic.uf2` onto that drive.** The board reboots into the firmware.
3. Plug it into the car's USB port (the data port — glovebox or center console).
   If the icon doesn't appear, try toggling the screen (scroll-wheel reboot) with
   it plugged in.

To check enumeration on your Mac first: plug the board into the Mac and run
`system_profiler SPUSBDataType | grep -A12 TeslaMic` — you should see
`TeslaMic`, `0x1235`, `0x0002`, and an "Audio" interface.

## Restore Meshtastic

The app lives at `0x26000` and never touches the bootloader or SoftDevice, so
recovery is just flashing the stock image back:

1. Double-tap RESET → bootloader drive.
2. Drag the **Meshtastic T114 UF2** back on (download from the Meshtastic web
   flasher / releases). Done.

Nothing here erases the bootloader, so the board can't be bricked by a bad app
UF2 — worst case you double-tap and flash something else.

## How the silence stream works (and how to make it real audio)

`embassy-nrf` 0.10 can *allocate* the nRF52840's isochronous endpoint (so the
mic enumerates), but its `EndpointIn::write` asserts `len <= 64` and drives the
regular EPIN registers — it **cannot** push 192-byte iso packets out of the ISO
endpoint. So `iso_silence_pump()` in `src/main.rs` drives the `USBD.ISOIN`
registers directly (via `embassy_nrf::pac`, behind the `unstable-pac` feature):

- Waits until the host selects AudioStreaming alt-1 (embassy sets `EPINEN` bit 8).
- Arms `ISOIN.PTR`/`ISOIN.MAXCNT` with a 192-byte buffer and triggers
  `TASKS_STARTISOIN` **exactly once per frame, right after `EVENTS_SOF`**.
  This timing is critical: a full-speed 192-byte packet takes ~128 µs to clock
  out, and firing `STARTISOIN` (which DMAs into the single ISO buffer) while
  that transmission is in flight corrupts the packet — audible as scratchiness.
  Arming just after SOF guarantees the DMA finishes before the host's IN token
  and never during a transmission. embassy's USB driver never touches these
  registers, so there's no conflict with its interrupt handler.

**To carry real audio** instead of silence: capture stereo 48 kHz/16-bit into a
RAM ring buffer with the nRF52840's **I²S** (external ADC/line-in — cleanest),
**PDM** (digital MEMS mics), or **SAADC** (analog line-in), and point the pump's
`arm_iso` at the ring buffer's read cursor instead of the zero buffer. The USB
side is already done; only the capture front-end and a few free header pins are
needed. (`ISOINCONFIG.RESPONSE` is set to `ZERO_DATA` so any late frame sends a
harmless zero-length packet rather than stalling.)

## Design notes

- **No SoftDevice.** We never call `sd_softdevice_enable`, so the stock SD stays
  dormant and the app owns CLOCK + POWER — letting us use embassy-nrf's ordinary
  USB driver + `HardwareVbusDetect`. We force HFXO (external crystal) because the
  USB peripheral needs the crystal-derived 48 MHz reference.
- **Endpoint address.** The genuine device uses iso IN `0x84`; the nRF only has
  one iso endpoint, so ours is `0x88`. Hosts key on interface class + audio
  format, not the endpoint number.
- All descriptor byte layouts are in `src/main.rs`, annotated against the USB
  Audio Class 1.0 spec.

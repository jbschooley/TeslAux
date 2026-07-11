# TeslaMic emulator (Heltec T114 / nRF52840)

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

## Status

Presents the full identity and a spec-correct UAC1 **microphone** (AudioControl
+ AudioStreaming interfaces, stereo 48 k/16-bit), verified: macOS enrolls it as
a 2-channel / 48 kHz USB input device named `TeslaMic`.

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
python3 tools/uf2conv.py teslamic.bin -c -b 0x26000 -f 0xADA52840 -o teslamic.uf2
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

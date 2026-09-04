# TeslAux

**Play audio from a phone through a Tesla's speakers, over USB.**

A Tesla has no aux input. It does have a USB port that accepts Tesla's own
cabin microphone — the CaraokeMic — and routes whatever that microphone hears
into the cabin speakers. TeslAux clones that microphone precisely enough that the
car accepts it, then feeds it real audio instead of a microphone.

The result is a wired audio input: anything that can play into a USB audio
output can play through the car. Wired, so unlike Bluetooth it is not
resampled, not re-encoded, and not subject to codec latency.

## How it works

```
phone ──USB──> [source board] ──I²S──> [car board] ──USB──> Tesla
               a USB audio output      a clone of the CaraokeMic
```

Two boards, because the phone and the car are both USB *hosts* — a device has to
be a USB *peripheral* to each, and one USB controller cannot do both. They are
joined by I²S, which carries the audio between them as plain PCM with no
conversion at any point.

The source board's clock is steered to follow the phone, so the phone and the
I²S link run on one clock. The only remaining clock difference — against the
car — is absorbed by varying the size of each USB packet, which loses no samples
at all.

## Build and flash

Two identical **RP2040-Zero** boards, about $3 each. Wire three signals and a
ground between them:

| source board | | car board |
|---|---|---|
| GPIO2 | → | GPIO2 (data) |
| GPIO3 | → | GPIO3 (bit clock) |
| GPIO4 | → | GPIO4 (word clock) |
| GND | → | GND |

Ground is not optional — the boards are powered from different USB ports.

```sh
cd rp2040 && ./build-uf2.sh
```

Then hold BOOTSEL while plugging each board into a computer and copy its image
across. Use `cp`, not Finder: the bootloader reboots the instant the last block
lands, which Finder treats as a disk being yanked out.

```sh
# the board the phone plugs into
cp rp2040/teslamic-rp-source-ultralow.uf2 /Volumes/RPI-RP2/

# the board that plugs into the car
cp rp2040/teslamic-rp-car-elastic-lowlat.uf2 /Volumes/RPI-RP2/
```

The phone will show a USB audio output called **TeslAux Bridge**. Plug the car
board into the car's data port and the mic icon appears.

Each board has an onboard RGB LED: **green** streaming, **blue** waiting for
audio, **red** a fault. [`rp2040/FLASHING.md`](rp2040/FLASHING.md) has every
image, the LED codes in full, and the diagnostic builds.

### A simpler version is coming

A **PCM2706** USB-to-I²S bridge chip can replace the source board entirely. It is
a fixed-function part with no firmware, and it recovers the host's clock in
hardware rather than in a software control loop — which should be both more
reliable and about 5 ms lower latency than the two-board version. That leaves one
RP2040 plus a ~$10 module. The car-side firmware already supports it and follows
whatever sample rate it delivers; it has not been tested yet.

## Verified bit-exact

The two-board rig has been measured, not just listened to: **8,352,000 frames
across three consecutive cold boots, delivered from an Android phone to a Mac
through both boards, identical to the source file, sample for sample.** See `rp2040/TESTING.md` for
the method and `tools/bitcompare.py` for the tool.

## What the car accepts

Findings from the car itself, not from documentation. Several contradict what
this project believed for months — the corrections are noted.

### The device must look like a CaraokeMic

The car runs a driver written for one specific device and checks for it. Getting
the audio right is not enough; get the descriptors wrong and it shows
"unsupported USB microphone" and disconnects after about 60 seconds.

What has to match, verified against a real unit dumped over libusb
(see [`real_mic_dump.md`](real_mic_dump.md)):

* **VID `0x1235`, PID `0x0002`**, and the manufacturer, product and serial
  strings. The serial is 40 bytes, which makes the string descriptor 162 bytes —
  a 128-byte control buffer silently breaks enumeration.
* **Four interfaces**: AudioControl, AudioStreaming, and *two* HID interfaces.
* **IF2 is a HID keyboard** with the standard 65-byte boot-keyboard report
  descriptor. Not telemetry, as was assumed for months — the mic's button sends
  keystrokes.
* **IF3 is an endpoint-less vendor HID**, usage page `0xFF00`, usage `0x55AA`,
  with an exact 36-byte report descriptor and an 8-byte Feature report reading
  `00 01 00 03 03 00 08 00`. **This is the one that matters.** The car writes
  `A5 5A`-framed configuration to it and validates what comes back. Cloning it
  byte for byte is what defeats the popup.

What does *not* have to match: the endpoint numbers, the audio topology (a plain
microphone → USB-streaming terminal pair works, without the real mic's feature
and selector units), `bcdUSB`, `bcdHID`, and `wMaxPacketSize`.

### Audio formats

Retested after fixing two transport bugs that had been corrupting everything we
sent. The earlier conclusion that the car was "48 kHz-family only" was wrong —
it was our own corruption, and 44.1 kHz was simply the rate most exposed to it.

| | |
|---|---|
| 32 kHz, 44.1 kHz, 48 kHz, 16-bit | clean — retested after the transport fixes |
| 96 kHz, 16-bit | clean — retested |
| 192 kHz, 16-bit | plays, with a click roughly every 2 s (not diagnosed) |
| stereo | yes |
| more than 2 channels | only the first two are ever played |
| **24-bit** | **unverified** — see below |

**24-bit is an open question, not a result.** It was tested in July and appeared
clean at 48 and 96 kHz, but that predates the two transport bugs being found, and
those July tests used a 1 kHz tone at 48 kHz — exactly one cycle per USB packet,
and therefore blind to the packet corruption by construction. The same test
signal also reported 48 kHz as "clean" while every packet was being mangled. So
the July result carries no information either way.

`t114/teslamic-48k-24bit.uf2` and `teslamic-96k-24bit.uf2` are rebuilt against
the fixed firmware with a 997 Hz tone (deliberately not packet-periodic) if you
want to settle it. Note 192 kHz at 24-bit is impossible regardless: 1152 bytes
per frame against the 1023-byte full-speed isochronous limit.

It does not currently matter for the shipping firmware, which is 16-bit
throughout — the PCM2706 is 16-bit only, and the RP2040 path is built around
16-bit samples.

### Variable packet sizes

**The car accepts isochronous packets that change size from frame to frame.**
Verified with a build that alternates 47 and 49 samples on *every* frame, ~500×
harsher than real clock drift, playing cleanly for ten minutes.

This is what makes the design work: the difference between our clock and the
car's is absorbed by sending one more or one fewer sample when needed, so no
sample is ever dropped, duplicated or resampled.

### Latency

About 8 ms through both boards — USB frame quantisation, and enough buffer to
cover one packet at each hop.

That is not what you will hear. **The car's own audio pipeline adds roughly
100 ms**, measured in July and not reachable from the device, and the phone adds
10–50 ms of its own buffering. Our share is a few percent of the total, which is
worth knowing before optimising it further.

## Repository layout

| | |
|---|---|
| [`rp2040/`](rp2040) | the firmware to use — both boards build from one crate |
| [`t114/`](t114) | the original nRF52840 prototype, where the descriptors and the transport bugs were worked out |
| [`tools/`](tools) | UF2 packaging, and analysis scripts for recorded audio |
| [`RESEARCH.md`](RESEARCH.md) | how the CaraokeMic was reverse engineered |
| [`real_mic_dump.md`](real_mic_dump.md) | raw descriptor dump from a real unit |

## Licence

MIT throughout — see [`LICENSE`](LICENSE).

[`NOTICE.md`](NOTICE.md) records the provenance of two pieces: the T114's ST7789
driver, ported from an AGPL project and relicensed MIT here by its copyright
holder, and `tools/uf2conv.py`, from Microsoft's uf2 project.

"Tesla", "TeslaMic" and "CaraokeMic" are trademarks of Tesla, Inc. This project
is independent and not affiliated with or endorsed by Tesla.

See [HARDWARE.md](HARDWARE.md) for why the bridge needs two USB device
controllers, what that rules out, and the high-speed options worth revisiting if
Tesla's own ~100 ms delay is ever fixed.

## Single-chip variant (experimental)

`stm32/` holds a port to one STM32F407, which has two USB device controllers and
so needs no I2S link between the two halves. It builds but has never been run on
hardware — see `stm32/README.md`. The two-board RP2040 rig in `rp2040/` is the
version that works in a car.

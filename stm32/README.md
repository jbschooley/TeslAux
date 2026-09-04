# TeslAux on a single STM32F407

> **Status: builds, never run on hardware.** Nothing in this directory has been
> tested on a board or in a car. The two-board RP2040 rig in `../rp2040/` is the
> version that works. Treat everything here as a port to be brought up, not as a
> drop-in replacement.

The two-board rig puts the phone on one RP2040 and the car on another, joined by
an I2S link. That link is the reason the RP2040 build needs a PIO driver, two
ring buffers, a clock-steering control loop, and three clock domains.

The STM32F407 has **two independent USB device controllers**, so both hosts
attach to one chip and the link disappears:

```
  phone --USB--> OTG_HS (PB14/PB15)  ->  Pipe  ->  OTG_FS (PA11/PA12) --USB--> car
                 UAC1 speaker                      TeslaMic
```

No I2S, one clock crossing instead of three, and no sample slipping — the
phone-vs-car difference is absorbed losslessly by varying the packet size to the
car, which is the mechanism already proven in the car on the RP2040 build.

## Parts

| Part | Notes |
|---|---|
| STM32F407VET6 core board ("STM32F4XX_M") | LQFP100, 8 MHz HSE, 32.768 kHz LSE, USB-C |
| SparkFun USB-C Breakout (BOB-15100) | Has 5.1 kΩ Rd on both CC lines — verified from its schematic |
| 3 wires | That is the entire build |

## Wiring

Three wires. The onboard USB-C connector is already the car-side port: this
board wires D− to PA11 and D+ to PA12 through 22 Ω series resistors.

| Breakout pad | Board pin | |
|---|---|---|
| **D+** | **PB15** | OTG_HS_DP |
| **D−** | **PB14** | OTG_HS_DM |
| **GND** | any GND | |
| VBUS | *leave unconnected* | see below |
| CC1, CC2 | *leave unconnected* | breakout's own resistors handle it |

**Do not wire VBUS.** This board's +5V pins tie directly to the USB VBUS pin
with no protection. Bridging the phone's VBUS to that rail connects the car's 5 V
supply to the phone's, two hosts pushing against each other. The board is powered
from the car port, and the D+ pull-up that makes the phone notice a device is
powered by the board's own 3.3 V rail — so the phone enumerates it with no VBUS
wire at all. `vbus_detection` is `false` on both ports for this reason.

Getting D+ and D− backwards fails silently as "device not recognised". The higher
pin number is the higher signal line: **PB15 = D+**.

## Check this before anything else

The reference documentation for this board describes a **Micro USB** connector;
the current revision ships **USB-C**. That means the one USB-C-specific detail —
whether the board carries its own 5.1 kΩ Rd resistors on the connector's CC pins
— is not covered by the reference and is **unverified**. It matters, because the
car is a USB-C host and will not supply VBUS to a port that does not present Rd.

**Test it on your desk:** plug the board into a computer with a **C-to-C** cable.
If it powers up, the resistors are fitted. If it only works with the bundled
A-to-C cable, they are missing and the car will not power it.

## Why the car is on the onboard port

Neither port can do high speed, so the assignment is decided by power, not speed.
`OTG_FS` (PA11/PA12, the onboard connector) is full-speed-only silicon.
`OTG_HS` (PB14/PB15) is the high-speed-capable core, but only with an external
ULPI PHY; we use its internal PHY, which is full speed. Real high speed would
cut the packet quantum from 48 frames to 6 and take end-to-end well under a
millisecond, but it needs a PHY chip on a PCB.

Given that, the onboard connector's VBUS decides it: it is hardwired to the +5V
rail with no protection, so whatever is plugged in there powers the board. Car
on the onboard port means the car powers the bridge and the phone's VBUS stays
disconnected. Phone there instead would drain the phone, leave the bridge dead
whenever the phone is unplugged, and offer no way to feed car power in without
bridging the two hosts' rails.

The one argument the other way is capacity — the busier device is on the smaller
core:

| Core | Endpoints | FIFO | Carries |
|---|---|---|---|
| OTG_FS | 4 | 1.25 KB | TeslaMic: EP0 + iso IN (196 B) + HID IN |
| OTG_HS | 6 | 4 KB | speaker: EP0 + iso OUT |

Three of four endpoints fits, and if the FIFO does not, embassy panics at init —
a loud failure on the bench, not a subtle artifact in the car. If that happens,
the fix is to swap the ports *and* cut the onboard VBUS trace. Do not take on the
power problem pre-emptively.

## Build

```sh
cargo build --release
```

No feature flags. The two-board build has them because its cushion is a guess
that different situations want sized differently; here the cushion is measured,
so there is one configuration.

## Flash

Either route works; neither is drag-and-drop like the RP2040's UF2.

**DFU, no programmer.** The F407's ROM bootloader speaks DFU over PA11/PA12, so
the onboard USB-C is enough. Pull **BOOT0** high, reset, then:

```sh
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/bridge bridge.bin
dfu-util -a 0 -s 0x08000000:leave -D bridge.bin
```

**SWD, with an ST-Link V2.** Wire SWDIO/PA13, SWCLK/PA14, GND, 3V3 to the SWD
header, then `cargo run --release` (the runner is already configured for
`STM32F407VETx`).

Use SWD for bring-up. Not for the flashing — for the visibility. This port has
real unknowns and `probe-rs` gives you RTT and breakpoints; the RP2040 LED bug
cost four wrong theories precisely because there was no way to see inside.

## What the RP2040's bit-exact verification means here

The two-board build was verified bit-exact — 8,352,000 frames across three cold
boots, every sample identical to the source. Of the faults that had to be fixed
to get there, three were in `audio_pipe.rs` or in behaviour this build shares,
and are present here; the rest were specific to the I2S link and cannot occur:

| Fault | Here? |
|---|---|
| unprimed pipe paced as drift, emitting short packets | **yes** — fixed |
| pipe never trimmed back to target, latency set by boot order | **yes** — fixed |
| I2S capture pushed samples, so frame alignment depended on FIFO parity | no — see below |
| capture DMA single-buffered, FIFO unattended | no — no DMA in the audio path |
| rate detector muted for a second at every stream start | no — no rate following |

**Frame alignment cannot be lost here**, and it is worth being precise about
why rather than assuming the absence of I2S is enough. A USB packet is
self-delimiting and a frame is four bytes, so `chunks_exact(4)` drops a trailing
partial frame and the next packet starts aligned again. The RP2040's capture had
no such boundary: it pushed single samples into a FIFO, nothing tied a DMA word
to a frame, and an odd number left in the FIFO rotated every frame for the whole
session. That is a property of the transport, not of the chip.

**None of this is a substitute for running the test here.** `tools/bitcompare.py`
applies unchanged once there is hardware, and given that the RP2040's worst
fault was invisible to every counter and passed by ear, it is the only thing
that should be believed.

## Status LED

D2 on PA1, wired in sink mode (lights when the pin is low).

| Pattern | Meaning |
|---|---|
| Solid | Streaming |
| Slow blink | Waiting for the phone to open the stream |
| Fast blink | Fault: a packet exceeded wMaxPacketSize |

## Latency

Computed from the constants, not measured end to end:

| Stage | ms |
|---|---:|
| Phone USB OUT, 1 ms frame quantisation | 1.00 |
| Pipe cushion (128 frames at 48 kHz) | 2.67 |
| Car USB IN, 1 ms frame quantisation | 1.00 |
| **Total** | **≈4.7** |

against ≈8.0 ms for the two-board rig. The two 1 ms terms are irreducible at
full speed.

The cushion is set by the phone, not the chip: the producer is a 48-frame USB
packet and hosts bunch several together. `RING = 256` comes from the
`measure-excursion` build on the RP2040 source board, which stayed green — peak
excursion under 64 frames — for a full session with an iPhone and again with an
Android. The cushion is twice that.

## Why this is simpler than the two-board build

Removing the I2S link removes more than the wire.

**No lossy corrections, ever.** The pipe has two ways to absorb a clock
difference. `plan_batch()` varies how many frames go in the next packet — free,
lossless, invisible. `slip()` duplicates or discards a frame — a real
discontinuity whose size scales with the sample value, which is why loud bass
was where it was audible. `slip()` exists for a sink whose rate cannot be
varied, i.e. a fixed I2S clock, and it is called only from the RP2040 source
board for exactly that reason. **This design has no fixed-rate sink**, so the
only consumer is the car's iso IN endpoint whose packet size we choose, and
`slip()` is never reached.

That is also why the deadband can be tight here without the caution the source
board needs: correcting often costs a byte rather than a click.

What else went away, and why it was only ever there for the link:

| Gone | Was for |
|---|---|
| PIO I2S driver, pull-downs, DMA timeout | driving and receiving the wire |
| One of two ring buffers | each board needed its own |
| Clock-steering control loop | pulling the I2S clock onto the host's |
| `RateDetect` + watchdog-scratch re-enumeration | a PCM2706 upstream could pick any rate |
| Two of three clock domains | phone, I2S, car — now just phone and car |
| `slip()` | a fixed-rate I2S sink |

## Going faster

≈4.7 ms is the full-speed floor, not a conservative choice — see
[../HARDWARE.md](../HARDWARE.md) for the high-speed alternatives (STM32F72x with
an integrated PHY, i.MX RT1062 with two of them) and for why Tesla's own ~100 ms
makes none of them worth doing yet.

## Shared code

`audio_pipe.rs` and `teslamic.rs` are compiled straight out of `../rp2040/src/`
via `#[path]` rather than copied:

- `audio_pipe.rs` is HAL-free and carries 27 host-run tests. Run them with
  `rustc --test --edition 2021 -o /tmp/ap ../rp2040/src/audio_pipe.rs && /tmp/ap`.
- `teslamic.rs` is generic over `embassy_usb::driver::Driver`, so the descriptors
  the car validates are the *same bytes* in both builds. Porting them changed
  nothing, which is the point — the IF3 report descriptor and its Feature report
  are what defeat the "unsupported USB microphone" popup, and they must not drift
  between firmwares.

A path include rather than a shared crate, so the car-proven RP2040 build stays
untouched.

## Shared with the two-board build

`audio_pipe.rs` is compiled from `../rp2040/src/`, so fixes there land here too —
but only once this branch has them. Two matter:

- **`plan()` must not pace an unprimed pipe.** Without the guard an empty buffer
  reads as maximum negative drift, so every packet is emitted one frame short.
- **`trim_to_target()` on stream open and on poll resume.** Nothing drains the
  pipe until the car polls, so it pegs at capacity; the pacer then sheds one
  frame per packet and stops as soon as the level is inside the deadband, never
  returning to target. Latency ends up set by whether the phone or the car came
  up first, for the whole session.

Removing the I2S link did not remove that second one. It is a property of a
producer that runs while the consumer is idle, which both designs have.

## Not ported

- **Rate following.** The RP2040 car board follows 32/44.1/48 kHz because a
  PCM2706 upstream might be at any of them. Here we own both ends and advertise
  only 48 kHz to the phone, refusing anything else explicitly, so there is no
  rate to follow and the watchdog-scratch re-enumeration machinery is gone. If
  multi-rate support is wanted later it needs a persistent store for the rate
  across the reset — the RTC backup registers are the natural place.
- **Analog input.** The F407's I2S peripherals are free, so a PCM1808 ADC is
  four traces away. Not started.

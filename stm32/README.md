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

## Build

```sh
cargo build --release                        # ring 512, 5.33 ms cushion
cargo build --release --features low-latency # ring 256, 2.67 ms cushion
```

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

## Status LED

D2 on PA1, wired in sink mode (lights when the pin is low).

| Pattern | Meaning |
|---|---|
| Solid | Streaming |
| Slow blink | Waiting for the phone to open the stream |
| Fast blink | Fault: a packet exceeded wMaxPacketSize |
| N blinks, pause | Post-drive fault report (below) |

The report only appears once the phone disconnects, so it never competes with
live status. Park, unplug the phone, and count:

| Blinks | Cause |
|---|---|
| 1 | more than 8 pacer slips |
| 2 | the phone's stream dropped mid-run |
| 3 | buffer over/underran — the cushion was too small |

## Latency

Computed from the constants, not measured:

| Stage | ms |
|---|---:|
| Phone USB OUT, 1 ms frame quantisation | 1.00 |
| Pipe cushion (`RING`/2 frames at 48 kHz) | 5.33 (2.67 with `low-latency`) |
| Car USB IN, 1 ms frame quantisation | 1.00 |
| **Total** | **≈7.3** (**≈4.7**) |

against ≈8.0 ms for the two-board rig. The two 1 ms terms are irreducible at
full speed.

**The cushion is the whole spread, and it is set by the phone, not the chip.**
The producer here is a 48-frame USB packet from the host, and hosts bunch
packets; on the RP2040 car board the producer was a 16-frame I2S DMA block. So
collapsing two boards into one removes an entire cushion but does not shrink the
one that remains. `HYSTERESIS = 192` is the figure the RP2040 source board runs
and has been proven with an iPhone. `low-latency` drops it to 64, which the
measured peak excursion (<64 frames, from the `MEASURE` build) suggests is
reachable — but that was measured with the I2S chain in place, so **run
`source-MEASURE` against your own phone before trusting it.**

The deadband rule that was violated three times during the RP2040 bring-up — the
deadband must exceed the producer's burst, and both must fit in the cushion — is
now a `const` assertion in `main.rs`, checked at compile time.

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

## Not ported

- **Rate following.** The RP2040 car board follows 32/44.1/48 kHz because a
  PCM2706 upstream might be at any of them. Here we own both ends and advertise
  only 48 kHz to the phone, refusing anything else explicitly, so there is no
  rate to follow and the watchdog-scratch re-enumeration machinery is gone. If
  multi-rate support is wanted later it needs a persistent store for the rate
  across the reset — the RTC backup registers are the natural place.
- **Analog input.** The F407's I2S peripherals are free, so a PCM1808 ADC is
  four traces away. Not started.

# Hardware platform options

Both the car and the phone are USB **hosts**. The bridge is therefore a USB
**peripheral to each of them**, which means it needs **two device controllers**.
That single constraint drives every choice below — it is why no ordinary
dual-USB-C board works, since those bring out one device port and one host port,
or one port and a charger.

## What we have built

| | Two RP2040 | One STM32F407 |
|---|---|---|
| Status | **Works in the car** | Builds, never run on hardware |
| Boards | 2x RP2040-Zero + I2S jumpers | 1x STM32F4XX_M + USB-C breakout |
| Clock domains | 3 (phone, I2S, car) | 2 (phone, car) |
| Lossy corrections | yes, `slip()` on the I2S sink | **none possible** |
| Latency (ours) | ≈8.0 ms | ≈4.7 ms |
| Where | `rp2040/` | `stm32/` |

The RP2040 pairing needs the I2S link only because neither chip can be a
peripheral to two hosts at once. The F407 has two USB device controllers, so the
link disappears and with it the PIO driver, one ring buffer, the clock-steering
loop, `RateDetect`, and the only code path that could ever duplicate or discard a
sample.

Rejected along the way: **RP2040 + PCM2706**, because the PCM2706's hardware
clock recovery solves a problem this architecture does not have, and it is a
legacy part. **ESP32 / RP2350 dual-USB-C boards**, because the second connector
is a host or power port, not a second device controller.

## The latency floor, and why it is where it is

At USB full speed a packet carries **48 frames** and arrives once per 1 ms
frame. That quantum sets everything:

- the deadband must exceed one packet (48) → 64
- the cushion must exceed deadband + one packet (112) → 128 frames = 2.67 ms
- plus 1 ms of frame quantisation at each end

≈4.7 ms, and **none of it is conservatism** — it is forced by the packet size.
The cushion is confirmed by measurement, not inference: the `measure-excursion`
build stayed green (peak excursion under 64 frames) for a full session on an
iPhone and again on an Android, against a cushion of 128.

The only lever left is **USB high speed**, where a microframe is 125 µs and a
packet carries 6 frames instead of 48. That shrinks the quantum eightfold and
collapses both terms at once, to well under 1 ms total.

## Why we are not chasing high speed

Tesla's own audio path is roughly **100 ms**, downstream of us and untouchable:

| Design | Ours | With the car |
|---|---:|---:|
| Two RP2040 | 8.0 ms | ~108 ms |
| One STM32F407 | 4.7 ms | ~105 ms |
| Dual-HS | ~0.5 ms | ~100 ms |

Every one of those is inaudible for playback, and high speed does not rescue live
playing either — a note through an analog input still returns ~100 ms late out of
the speakers, which is unplayable regardless of how fast the bridge is. Monitor
through headphones off the source rig instead.

**That ~100 ms is a working estimate carried through this project, not something
measured here.** It is the number that decides this question, so it is worth
measuring before spending anything on the options below.

## If Tesla's delay is ever fixed

These become worth revisiting, in ascending order of effort.

### STM32F723 / F733 / F730 — one HS port, integrated PHY

The pragmatic option. HS with a **built-in** PHY on one port plus an FS port,
and it stays inside `embassy-stm32`, so `teslamic.rs` and `audio_pipe.rs` port
for free exactly as they did to the F407.

Most of the win comes from putting the **phone** on the HS port, because the
cushion is set by the producer's burst. Car stays full speed. Estimated ≈1.8 ms.

Catch: small dev boards barely exist — mostly the STM32F723E-DISCO. On a custom
PCB this is the natural upgrade path from the F407.

Note the H7 family is **not** the answer here despite being newer: the
STM32H723 needs an external ULPI PHY. Within STM32, integrated HS PHY means
F72x/F73x, or the newer H7R / H7S.

### NXP i.MX RT1062 — two HS ports, both with integrated PHY

The only part that literally matches the requirement: dual high-speed USB OTG,
both PHYs on-chip, no ULPI. Available on the **Teensy 4.1** (~$32, small, well
supported), which brings the second controller out on a 5-pin header.

Two catches:

- Teensy exposes that second port as a USB **host**. The silicon is OTG so
  device mode is possible, but it is against the grain of the ecosystem.
- **There is no `embassy-usb` driver for i.MX RT.** Rust support is
  `imxrt-usbd`, which implements the `usb-device` crate's traits instead. The
  F407 port was nearly free because `embassy-usb` is chip-agnostic; this one is
  not. It would mean rewriting the descriptor stack against a different USB
  library — including the hand-rolled endpoint-less IF3 handler and its exact
  36-byte report descriptor, which is the single piece that must stay
  byte-for-byte identical or the "unsupported USB microphone" popup returns.

`usb-device` can express all of it (it has `control_in` / `control_out` hooks),
so it is portable — just not cheap, and the risk lands on the most
safety-critical part of the project.

### Also dual-HS, if a Linux userspace is ever acceptable

TI AM335x (PocketBeagle) and Rockchip RK3399 both have two USB OTG controllers.
Different project shape entirely — USB gadget mode, a kernel scheduler between
you and the audio, and a boot time. Noted for completeness, not recommended.

## Crystals: do not over-spec them

A natural assumption is that the F407's 8 MHz crystal is what makes the
single-chip build clean, and that the RP2040 needed workarounds because of its
oscillator. Neither is true, and acting on it would waste money on a PCB.

Both chips already derive an **exact** 48 MHz USB clock: 8 MHz on the F407 via
`PLLQ` (8/4*168/7), and 12 MHz on the RP2040 (12*4) from its own USB PLL. The
RP2040 crystal is not even a choice — the ROM bootloader requires 12 MHz.
Neither chip has ever had a USB clock problem.

The RP2040 workarounds were about a different clock entirely: the **I2S sample
clock**. Its PIO divider is 8.8 fixed point, so at the default 125 MHz sysclk
48 kHz quantises to 48003.07 Hz — a systematic +64 ppm, ~3.1 slips/sec forever,
audible as crustiness on loud bass because a slip's discontinuity scales with
sample value. Running at 124.8 MHz makes the divider land exactly. `source.rs`
says it outright: *"USB is unaffected — it runs from its own PLL."*

**The single-chip build needs none of that because it never synthesizes an audio
clock.** No I2S, no divider, no quantisation — audio moves by memcpy between two
USB endpoints, and the crystal only clocks the CPU and the transceivers.

So for a PCB: any oscillator with an integer path to 48 MHz is sufficient, and a
TCXO buys nothing. It would not help with the one clock difference that does
remain — the phone's crystal against the car's — because that is inherent to
bridging two independent hosts and is absorbed by varying the packet size.

The exception is the **analog input**: driving a PCM1808 means generating an I2S
clock again. The F4 has a dedicated `PLLI2S` with a fractional divider built for
hitting audio rates exactly, so that is a solved problem there rather than a
repeat of the PIO fight.

## External PHY route

Any STM32 with `OTG_HS` and ULPI pins can reach true high speed with an external
PHY (USB3300 or similar). That includes the F407 already in `stm32/`. It is a
PCB-only option — the dev board uses OTG_HS's internal full-speed PHY — but it
means the current firmware's chip choice does not foreclose high speed later.

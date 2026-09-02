# Builds, pairings, and the two open questions

## Pairings (flash one image per board)

| Build | Boards | Notes |
|---|---|---|
| **`car-elastic` + `source-adaptive`** | 2x RP2040 | **Recommended.** Source board is I2S master steering itself to the phone's SOF; car board is I2S slave pacing elastically to the car. Bit-exact, no feedback endpoint needed, 48 kHz-only descriptor. Needs Q1 to pass. |
| `car-pcm2706` + a PCM2706 module | 1x RP2040 | Same car binary as `car-elastic`. Fewest unproven parts; no rate control. Needs Q1 to pass. |
| `car-locked` + `source-locked` | 2x RP2040 | Car board is I2S master locked to the car's SOF, fixed 192-byte packets; source board asks the phone to follow via a feedback endpoint. The only option if Q1 fails. |
| `car-STRESS-TEST` | 1x RP2040 | Diagnostic only, see Q1. |


Both designs are built and ready. Which one to ship depends on two facts nobody
knows yet. Each has a firmware build that answers it.

## Q1. Does Tesla accept variable-size isochronous packets?

If yes, the **PCM2706 build** works and is the simpler product (one board, one
firmware, no second USB device to write). If no, only the clock-locked two-board
build can work, because it is the one that emits fixed 192-byte packets.

**Test: `teslamic-rp-car-STRESS-TEST.uf2`** — needs no PCM2706, no source board
and no wiring. It generates a 1 kHz tone internally and deliberately swings the
packet size between 47 and 49 samples on alternate frames, keeping the average
at exactly 48 so nothing else can be blamed. That exercises the variable-packet
mechanism about 500x harder than real drift ever will.

1. Flash it to one RP2040.
2. Plug into the car's data port.
3. Listen.

* **Clean steady 1 kHz tone** -> Tesla tolerates variable packets -> the
  PCM2706 build is safe, and it is the simpler thing to ship.
* **Clicking, warbling, dropouts, or the "unsupported USB microphone" popup**
  -> variable packets are not viable -> the two-board clock-locked build is the
  only correct option.

Note the mic icon should appear either way; this is a test of the *audio*, not
of enumeration.

## Q2. Does the ScreenMate (Android) honour USB audio feedback?

If yes, the two-board chain is fully clock-locked end to end and no sample is
ever dropped or repeated. If no, the source board falls back to a deliberate
sample slip roughly twice a second — inaudible in practice, but not free.

**Test: `teslamic-rp-source.uf2`**, which reports the answer on its own LED.
It needs the car board running as its I2S master to have a clock at all, so:

1. Flash `teslamic-rp-car-locked.uf2` to board A, `teslamic-rp-source.uf2` to
   board B. Wire GPIO2/3/4 between them plus GND (see `bin/car.rs` for pinout).
2. Power board A from the car (or any USB port — it only needs SOF).
3. Plug board B into the ScreenMate and play audio.
4. Watch board B's LED for ~30 s:

| LED | Meaning |
|-----|---------|
| **solid** | Feedback honoured. Fully locked chain, zero slips. |
| **double-blink** | Host is ignoring feedback; slipping ~2/s. Still works, slightly lossy. |
| **slow blink** | No I2S clock — board A unpowered, unwired, or its PIO is wrong. |

The LED samples the slip counters over 10 s windows, so give it at least that
long to settle before reading it.

## Deciding

| Q1 variable packets | Q2 feedback | Ship |
|---|---|---|
| OK | either | **PCM2706 build** — one board, simplest to distribute |
| Not OK | honoured | **Two-board locked** — fully locked, no compromises |
| Not OK | ignored | **Two-board locked** — fixed packets to the car, ~2 slips/s at the phone |

Q1 is the one that actually decides the architecture; Q2 only decides how good
the two-board version is. So run Q1 first — if it passes, you may never need the
second board at all.

## Caveat

None of this has run on hardware. The PIO I2S bit alignment is the most likely
thing to be wrong on first power-up; a channel-swapped or distorted signal points
there rather than at anything in this document. `teslamic-rp-car-STRESS-TEST.uf2`
is the exception — it generates audio internally and touches no PIO at all, so it
is the safest first thing to flash.

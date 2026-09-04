# Builds, pairings, and the two open questions

> ## Status 2026-09-03
>
> **The RP2040 car board is accepted by the car.** `teslamic-rp-car-STRESS-TEST.uf2`
> played a 997 Hz tone for **10 minutes with no dropout and no popup** — well past
> the ~60 s threshold at which the "unsupported USB microphone" popup used to
> fire. That validates, on real hardware in the real car:
>
> * the TeslaMic descriptor clone works from an RP2040 (the three descriptors the
>   car validates are byte-identical to the real mic — verified over libusb);
> * variable packet sizes are accepted, and this build changes size on *every*
>   frame, ~500x harsher than real clock drift.
>
> **The two-board I2S chain works on the bench**: source board as I2S master ->
> car board as I2S slave -> USB, clean audio, correct bit alignment. Resetting the
> source board cuts audio for about a second and recovers, which is the stall
> detection in `capture` doing its job.
>
> **Not yet tested: the two-board rig in the car.** Bench and car have each been
> proven separately, never together.

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

## I2S bring-up: `teslamic-rp-I2STEST.uf2`

Wire GPIO2 (DATA), GPIO3 (BCK), GPIO4 (LRCK) and **GND** between the two boards,
then flash `teslamic-rp-I2STEST.uf2` to the source-side board and
`teslamic-rp-car-elastic.uf2` to the car-side board, and listen on the car
board's USB input.

The test board drives the link with embassy-rp's **upstream** `PioI2sOut` rather
than any of my PIO code, configured `bit_depth = 32` so BCK is 64x fs — the same
framing `slave_rx` expects and the PCM2706 produces, with the 16-bit sample in
the top half of each word. So this isolates my `slave_rx` against known-good
master code:

| result | meaning |
|---|---|
| clean 997 Hz | wiring and `slave_rx` are both correct |
| distorted / channel-swapped | `slave_rx` bit alignment is wrong (1 bit or 1 slot out) |
| silence | no clock reaching the car board — check wiring, and check GND |

Do this before trusting the two-board pairings. Any fault it shows is in my code,
not upstream's, which is the point.

### Why upstream isn't used in the shipping firmware

`PioI2sIn`/`PioI2sOut` are controller-role only and expose no runtime clock
retuning (their state machine is private). Every pairing here needs a slave on
one side, and both master roles need a steerable clock — that is what adaptive
and clock-locked mean. So they fit as a test master and nowhere else.

## Known issue: the RP2040-Zero status LED

The Zero has no plain LED, only a WS2812 on GPIO16, and the status indicator on
it is **not working**. It shows a boot colour (magenta) and never updates.

What was tried, and what each attempt showed:

1. Status in a spawned task, using embassy-rp's `PioWs2812` (DMA) — LED stayed
   on the boot colour set from `main`.
2. Same, writing unconditionally instead of only on state change — no change, so
   it is not change-detection masking a stable state.
3. Own DMA-free driver pushing the PIO FIFO directly (`src/ws2812.rs`) — no
   change, so it is not the DMA.
4. Driven from the pump loop instead of a task — LED stopped lighting at all.

Attempt 4 is the confusing one and has no explanation I can support. `LoadedProgram`
has no `Drop`, so instruction memory is not being freed underneath it. Parked at
the state that at least shows a boot colour rather than left worse.

**Audio is unaffected by all of this** — the LED is diagnostics only. If it
becomes worth solving, the honest next step is a logic analyser on GPIO16 or RTT
over SWD, not more guessing.

Relevant beyond the LED: attempt 1 suggests spawned tasks may not be getting
polled, and the I2S capture path is also a spawned task doing DMA. Worth keeping
in mind if I2S produces nothing — but note `usb_task` is spawned and works, so it
cannot be a blanket failure.

## Caveat

None of this has run on hardware. The PIO I2S bit alignment is the most likely
thing to be wrong on first power-up; a channel-swapped or distorted signal points
there rather than at anything in this document. `teslamic-rp-car-STRESS-TEST.uf2`
is the exception — it generates audio internally and touches no PIO at all, so it
is the safest first thing to flash.

## Bit-exact end-to-end verification

`tools/bitcompare.py` compares a recording made through the bridge against the
file that was played. It is a far stronger test than listening or than any
status LED, because it proves a negative those cannot: that no sample was lost,
repeated, truncated, reordered, rescaled or mis-framed anywhere between the
source and the recorder.

```sh
tools/bitcompare.py reference.wav recording.wav
tools/bitcompare.py --self-test
```

It aligns the two by correlation, checks that alignment is real, then names what
it finds:

| Result | Means |
|---|---|
| bit-exact | nothing lost, repeated, rescaled or reordered |
| every sample off by 1 LSB | a truncating `int16 -> float -> int16` round-trip; nothing lost |
| N frames missing at a point | a discrete discard, located to the frame |
| repeated frame | a slip down: a starved batch was padded |
| channels swapped **and** one sample skewed | the capture lost frame alignment |
| a constant gain | a volume not at unity — the chain, not the bridge |
| not the same audio | wrong file, or the capture missed it |

The tool self-tests against synthesised versions of every fault it names, so a
result from it means something before it is pointed at real audio.

### Setting up a bit-exact chain

Most failures will be the chain rather than the bridge:

* The source file must **already** be 48 kHz, 16-bit. Anything else is resampled
  or dithered before it reaches us.
* The player needs a bit-perfect path to USB audio, volume at 100%, with every
  effect and normalisation off. On Android, USB Audio Player Pro does this;
  the stock mixer rescales and may drop buffers.
* **Record 32-bit float.** Recording 16-bit adds a truncating conversion that
  leaves ~9% of samples one LSB low. Harmless to listen to, but it hides
  everything else, because with every frame differing an exact comparison
  reports total corruption.

`ffmpeg -f avfoundation` is **not** a usable capture path: measured at about 10%
sustained frame loss (a 5 s capture returned 4.499 s), which destroys sample
alignment while leaving the spectrum intact — so it looks like a bridge fault.
`tools/record-mac.sh` is kept only as a record of that.

## Result: bit-exact, 2026-09-04

Android (USB Audio Player Pro) -> source RP2040 -> I2S -> car RP2040 -> USB ->
Mac, recorded at 32-bit float, compared against the file on the phone:

```
max fractional error: 0.0
compared 2,784,000 frames (58.0 s)
PASS  bit-exact: every sample matches
```

Every sample arrived identical. No loss, no repetition, no drift, no rescaling,
correct channel mapping, correct frame alignment — which verifies the elastic
pacing, the I2S framing, the channel order and the clock-domain crossing all at
once.

Two firmware faults had to be fixed to reach it, and **neither was visible to
any counter or by ear**: an unprimed pipe was paced as maximum negative drift
and emitted short packets, and the capture could start on the wrong half of a
frame, rotating every sample for the session.

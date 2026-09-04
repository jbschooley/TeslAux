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

## Reading the car board after a drive

You cannot watch an LED while driving, and "it sounded weird once or twice" does
not distinguish a loose wire from a firmware fault — they have opposite fixes.
The car board therefore latches the worst thing it saw since boot and reports it
on the LED **once the source goes away**, so live status is never affected. Park,
unplug the phone, and read the blink code:

| Blinks | Colour | Meaning | Where to look |
|---|---|---|---|
| none (solid blue) | blue | clean run | — |
| 1 | amber | level ran too **full** | source outran the car, or the stream restarted after alt 0 |
| 2 | blue | I2S link dropped for >250 ms | mechanical: jumpers, connector, vibration |
| 3 | purple | re-enumerated at a new rate | source changed sample rate mid-drive |
| 4 | red | buffer over/underran | cushion too small for the observed drift |
| 5 | white | **PIO RX FIFO overflowed — samples silently lost** | the capture DMA is single-buffered |
| 6 | cyan | level ran too **empty** | the car outran the source |

Code 1 reports **peak excursion**, not a count of corrections. Corrections on
this board are `plan_batch` — lossless changes to the next packet's size, and
the normal way the source-versus-car clock difference is absorbed. Counting them
reported the pacer working. It also produced a false code 1 every time the host
selected alt 1 after a gap: nothing drains the pipe on alt 0, so it pegs at
capacity and then sheds ~200 frames back to target, one per packet.

Peak excursion is measured **in the pump, once per USB packet, before any
frames are taken** — the same point the pacer acts on. Phase matters more than
it looks: the level sawtooths by a whole USB packet every millisecond, because
capture pushes an I2S block three times per packet while the pump removes ~48
frames once. Sampled just after a push, a perfectly healthy buffer parked at the
deadband edge reads `HYSTERESIS + 48` and trips any threshold set relative to the
deadband. That is what produced a false code 1 after activating the host
mid-playback.

Every counter here is gated on the pump actually writing packets, and peak
excursion additionally waits until the level has come inside the deadband at
least once in the current session — which is exactly "the pacer has the level
under control", and needs no time constant.

**A stall is counted only when the link comes back.** A stall that never
recovers is just you unplugging the phone to read the report, and one before the
first capture is this board powering up before the source — neither is a fault.
Counting stalls as they happen would also climb without limit, because the
capture timeout re-fires every 250 ms for as long as the source is absent. So
code 2 means the link dropped **and returned mid-drive**, which is the glitch you
would have heard.

Code 5 is the one to watch, because it is the only fault here that nothing else
can see. `slave_rx` uses `push noblock`, so a full RX FIFO discards the sample
instead of stalling the state machine, and downstream cannot tell the difference
between lost samples and a source running slightly slow. The capture DMA is
single-buffered, so between one `dma_pull` completing and the next being armed
only the 8-word FIFO — about 83 us at 48 kHz — is holding the stream. The `pump`
task takes `PIPE` under a critical section in that window, and USB interrupts
land in it too.

`RXSTALL` latches, and the state machine runs from the moment it is enabled
while the first DMA is not armed until the capture task first runs — so with a
source already clocking, every boot sets the flag. That transient is cleared
without being counted, as is the first block after a stall; otherwise code 5
would appear on every single run and say nothing about streaming.

**Code 5 was observed on a real run**, which is what prompted double-buffering
the capture: the next DMA is now armed before the pipe push and the bookkeeping,
so all of that overlaps a running transfer and the unattended window shrinks from
the whole processing pass to a memcpy plus DMA setup. The counter is kept, so a
clean run now *is* the evidence that the fix worked. This is the same fix the
nRF firmware needed, for the same reason.

Codes 2 and 4 are individually audible.

One limit worth knowing: a rate change (code 3) re-enumerates via a watchdog
reset, which clears RAM. Only the rate-change tally survives, carried in the
scratch registers; any slip or stall counts from before it are lost. Code 2 pointing at a mechanical fault is
the case a PCB fixes; the others are firmware and would follow the design onto
any new board.

The counts come from the pipe's own `Stats` rather than being tallied separately,
so the LED and the pipe cannot disagree. The rate-change count is carried through
the watchdog scratch registers, since the reset it records would otherwise clear
it.

## Reading the source board after a run

The source board has its own latch, with an important difference: **it is
powered by the host**, so unplugging the phone takes the latch with it. Read it
at a **pause** — stop playback and the LED reports while the board stays
powered.

| Blinks | Meaning | What it means |
|---|---|---|
| none | clean run | — |
| 1 | peak level excursion passed `PEAK_WARN` | margin being used up; nothing audible yet |
| 2 | sample slips occurred | audible: the steering loop fell behind |

Code 2 is the one that matters. This board's consumer is a fixed I2S clock, so
correcting means `slip()` — a real discontinuity whose size scales with sample
value, which is why it was audible on bass and not when quiet. On a
`clock-steered` build the loop should hold the level and `slip()` should return
0, so any slip at all means steering lost the plot.

`PEAK_WARN` sits halfway between the slip deadband and a dry buffer — 224 frames
on the low-latency build, where the deadband is 192 and the target 256. It has to
be **above** the deadband: inside it the level wanders freely by design and the
pacer does nothing, so a threshold there reports ordinary operation. A `const`
assertion enforces that now, because the first version used `RING / 4` = 128 and
flagged every run.

Both counters are gated on `HOST_LIVE` rather than `CLOCK_LIVE`, and slips are
rebased per streaming session. The difference matters: after the host stops, the
buffer keeps draining, the level dives toward empty and `slip()` fires on the way
down. Counting that would make every pause look like a fault — which is exactly
how the car board's latch twice reported nothing but its own shutdown.

The peak latch additionally requires **fresh frames from the host** in the same
block, because `HOST_LIVE` stays true through the gap between the host stopping
and the endpoint reporting it — and the level falls ~48 frames per millisecond in
that window.

Neither counter is cleared by resuming playback: they are a record of the whole
run. **Power-cycle the board for a fresh reading.**

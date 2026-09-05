# Audit: what else can interrupt the audio

Prompted by pops the buffer counters showed nothing for. Everything below is a
path that can produce a discontinuity, ranked by how easily it fires.

## Fixed here

**The rate detector could mute on one bad measurement.** `classify()` rejects
anything more than 2% from a standard rate, and an unclassifiable result muted
the output until the next window closed — up to a second of silence.

The window is easy to spoil, because the detector is clocked by *delivered
packets* rather than by time. `on_usb_frame()` only advances when the host takes
a packet, so a host that pauses polling stops the clock while the I2S side keeps
counting frames, and the ratio comes out inflated through no fault of the
source. Muting now needs three consecutive nonsense windows, and a rate change
two agreeing ones — the same rule the re-enumeration path already used.

## Remaining discontinuity sources, and why each is left alone

| Path | When it fires | Verdict |
|---|---|---|
| `trim_to_target()` on stream open | host selects alt 1 after a gap | discards down to target; **present in the build measured click-free**, so not the cause, but it is the only deliberately lossy path left |
| `push()` overrun | pipe full — the host is not draining | drops oldest, which is right: staying live beats keeping stale audio |
| `pop()` underrun | pipe dry | holds the last sample, a smaller discontinuity than jumping to zero |
| `reset()` on a 200 ms capture gap | source really stopped | deliberate: splicing stale audio onto new is worse |
| `MUTED` → `fill(0)` | rate mismatch confirmed | now needs three windows |

## Not a suspect: the elastic pacing

Varying the packet size between 47 and 49 frames **loses no samples**. It is the
mechanism that absorbs the clock difference without dropping or duplicating
anything, and it was verified bit-exact across three cold boots. Worth stating
plainly because it looks like the obvious culprit and is not.

## Measured, and clean

During a click, with logging on the single-chip build: zero underruns, overrun
and timeout counts unchanged, the car polling, the phone delivering, level inside
the deadband. **Whatever is heard does not pass through the buffer.** That is
what moved the search to the wiring and to the wire protocol.

## Cleanups made

* Removed unused imports and initialisers the loop immediately overwrites.
* `#![allow(dead_code)]` on `status.rs`, `teslamic.rs` and `i2s_pio.rs`, each with
  a note saying why: they are shared between binaries that use different subsets,
  so unused items are a property of the caller.
* The car build is now warning-free.

## What the measurements found

Everything below is measured, not inferred. Recordings were compared against the
file that was played (`tools/bitcompare.py`) or against the tone the firmware
generates (`tools/tonecompare.py`).

**The pops were line coupling, and they are fixed.** An eleven-minute recording
held 112 corrupted samples in its first two and a half minutes. All but one were
the right channel, and every one had been replaced by its own sign bit —
`(sample & 0xFFFF) >> 15` exactly, on all 54 checked by hand — with jumps up to
44% of full scale in a single sample. That is a right slot read fifteen
bit-clocks early: fifteen bits of the left slot's zero padding, then one bit of
the real sample, which happens when `wait 1 pin` on LRCK releases before the
true edge. LRCK idles low during the left slot, so only a positive glitch
corrupts the right channel, which is why the damage was one-sided.

Soldering the boards edge to edge with a driven shield between BCK and LRCK
removed it: **zero corrupted samples across five recordings since**, against 112
in two and a half minutes before.

**The remaining artifact is a 16-frame hold, and it is not the buffer.** Three
per ten minutes at first, each a sample held for 0.33 ms. Doubling the source's
cushion changed nothing (3 -> 3); doubling the car's changed nothing (3 -> 3).
The hold stayed exactly 16 frames while the source's I2S block moved 16 -> 32
and the car's moved 16 -> 64, so it is not either board's block size.

`packet-stress`, which generates a tone with no phone, no source board and no
I2S, ran **411 seconds without a single frame added, dropped or altered** —
while swinging packet sizes 47/49 every frame. That clears the USB transmit
path, the variable packet sizing, and the capture on the Mac.

`pipe-tone`, which substitutes a known tone for the captured samples while
keeping the capture timing, then put the holds in the car board's own pipe:
three in eight minutes with the I2S data discarded, so none of them could have
arrived from the source.

**Then it stopped reproducing.** Nearly thirty minutes across three
configurations, zero holds:

| car build | source build | duration | holds |
|---|---|---|---|
| `pipe-tone` | before the idle fix | 7.9 min | **3** |
| `pipe-tone` + `pipe-watch` | before the idle fix | 9.6 min | 0 |
| `pipe-tone` + `pipe-watch` | after | 10.1 min | 0 |
| `pipe-tone` | after | 10.2 min | 0 |

At the original rate, thirty clean minutes is about a 1-in-90,000 outcome, so
something did change. But no single variable explains it: the instrumentation
cannot be the cause, because the last run has none; and the idle fix cannot be
the whole cause, because the second run predates it.

The idle fix is still the best mechanism on offer. Before it, a momentary
starvation of the source's pipe un-primed it, the steering loop read a whole
target low, and the I2S clock wound down to 47488 Hz — at which the car's pipe
loses 512 frames a second and underruns about a quarter of a second later. That
chain produces exactly this artifact, from a cause on the source board, showing
up on the car board, with neither board's buffer able to prevent it. It is also
consistent with the counts falling as the source's cushion grew (6 -> 3) while
the car's made no difference.

What no hypothesis has explained is why the hold was **exactly 16 frames every
time**, across four different block-size configurations. That remains open.

## Where it ended

The shipping pairing — `car-elastic-lowlat` and `source-ultralow`, the same two
images that showed six holds in `cap2` — was recorded against the full
eleven-minute reference after the shield, the idle fix and the rate-detector
fix:

    compared 30876800 frames (643.3 s), lead-in included
    PASS  bit-exact: every sample matches

Every sample of the entire reference, including the two seconds the anchor used
to hide. No corrupted samples, no holds, no missing frames, no startup
transient.

One run does not prove a rare fault is gone, and the hold was always rare — but
at `cap2`'s rate this recording should have contained about five, and the
configuration is the one that produced them. Taken with the thirty clean minutes
of tone before it, the artifact is no longer reproducible by any means available
here.

Two of the three faults are closed by measurement rather than argument: the
sign-bit corruption (a physical fix, confirmed by its absence across six
recordings) and the idle buzz (a firmware fix with a mechanism). The third is
recorded here as unreproducible rather than solved, because that is what the
evidence supports.

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

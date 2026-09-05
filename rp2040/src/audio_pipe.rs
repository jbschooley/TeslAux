// SPDX-License-Identifier: MIT
//! Clock-domain bridge between an audio *source* (I2S capture, running on the
//! phone's clock) and an audio *sink* (the USB isochronous IN endpoint, running
//! on the car's clock), plus source sample-rate measurement.
//!
//! Pure logic: no HAL, no allocation, no `unsafe`. Host-testable — see the
//! `tests` module at the bottom, run via `tools/test_audio_pipe.sh`.
//!
//! # The problem
//!
//! The two ends are independent crystals. At a typical 20-50 ppm mismatch they
//! drift by roughly 1-5 audio samples per second, forever. Something has to
//! absorb that or the buffer eventually runs dry (click) or overflows (click).
//!
//! # The two modes
//!
//! [`PaceMode::Elastic`] is for the PCM2706 build, where the source clock is
//! the phone's and nothing can be done about the mismatch. The pipe keeps its
//! ring buffer near half full by handing the USB frame one *more* or one
//! *fewer* sample than nominal when the level drifts. This is the standard UAC
//! asynchronous-source mechanism and is inaudible — but it means the packet
//! size varies, which the car may or may not accept.
//!
//! [`PaceMode::Locked`] is for the two-board build, where the source's I2S
//! clock is derived from the car's SOF, so there is no drift to absorb. Every
//! USB frame gets exactly the nominal sample count, so every packet is exactly
//! the same size — matching the real TeslaMic's fixed 192-byte adaptive
//! endpoint. Any correction here would indicate a bug in the SOF-locked clock,
//! so underruns are counted rather than smoothed over.
//!
//! # Fractional rates
//!
//! Rates that aren't a multiple of 1000 (44100, 88200) don't divide evenly into
//! 1 ms USB frames. [`Pipe::nominal`] runs an accumulator so the long-run average
//! is exact: at 44100 it emits 44 samples for nine frames then 45 on the tenth,
//! totalling 441 samples per 10 ms.

#![allow(dead_code)]

/// One stereo sample pair. 16-bit is the TeslaMic format and the PCM2706's only
/// format, so this is deliberately not generic.
pub type Frame = [i16; 2];

/// How the sink decides its per-frame sample count.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PaceMode {
    /// Vary the packet size to track a free-running source clock.
    Elastic,
    /// Emit exactly the nominal count; the source clock is already locked to us.
    Locked,
    /// Diagnostic: swing the packet size between nominal-1 and nominal+1 on
    /// alternate frames, keeping the average exact.
    ///
    /// Answers the open question "does Tesla tolerate variable-size isochronous
    /// packets?" definitively and quickly. Elastic mode only varies a couple of
    /// times a second, so an intolerant host might merely sound slightly off;
    /// this exercises the same mechanism 500x harder, so a host that cannot cope
    /// fails obviously and immediately. If a tone plays cleanly under Stress,
    /// Elastic is safe. Never ship this.
    Stress,
}

/// Health counters. All saturate rather than wrap so a long run can't hide a
/// problem behind an overflow.
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct Stats {
    /// Source produced faster than the sink drained; frames were discarded.
    pub overruns: u32,
    /// Sink asked for frames that weren't there; the last frame was repeated.
    pub underruns: u32,
    /// Times an extra sample was sent to shed a too-full buffer.
    pub adj_up: u32,
    /// Times a sample was withheld to refill a too-empty buffer.
    pub adj_down: u32,
}

/// Ring buffer plus pacing, bridging the source and sink clock domains.
///
/// `N` is the capacity in frames. It must comfortably exceed one USB frame's
/// worth of samples; 512 frames (~10 ms at 48 kHz) is a sane default, giving
/// ~5 ms of slack in each direction.
pub struct Pipe<const N: usize> {
    ring: [Frame; N],
    head: usize,
    tail: usize,
    len: usize,
    /// Fill level the pacer steers toward.
    target: usize,
    /// Deadband around `target`.
    ///
    /// **Must exceed the producer's burst size.** The source delivers in bursts
    /// (a whole I2S DMA block at a time) while the sink drains once per USB
    /// frame, so the instantaneous fill level sawtooths by at least one burst
    /// even when the clocks match perfectly. A deadband narrower than that
    /// makes the pacer correct against its own sampling phase rather than
    /// against real drift — which it did, 16 times a minute at zero ppm, before
    /// this was widened.
    hyst: usize,
    /// Fractional-rate accumulator, in units of samples-per-1000-frames.
    accum: u32,
    rate: u32,
    /// Repeated on underrun so a gap is a held sample rather than a zero click.
    last: Frame,
    pub stats: Stats,
    /// Cleared until the buffer first reaches `target`, so playback doesn't
    /// start from an empty buffer and immediately underrun.
    primed: bool,
    /// Alternating flag for [`PaceMode::Stress`].
    stress: bool,
}

impl<const N: usize> Pipe<N> {
    /// `rate` is the source sample rate in Hz (48000, 44100, ...).
    ///
    /// Deadband defaults to two USB frames. Use [`Self::new_with_hysteresis`]
    /// where the host delivers in larger bursts than that.
    ///
    /// `const` so a `Pipe` can live in a `static` and be shared between the
    /// capture task and the USB pump without a lazy init.
    pub const fn new(rate: u32) -> Self {
        Self::new_with_hysteresis(rate, (rate as usize / 1000) * 2)
    }

    /// As [`Self::new`], but with an explicit deadband in frames.
    ///
    /// The deadband must exceed the largest level excursion the two sides can
    /// produce on their own — one producer burst, one consumer burst, and any
    /// bunching the host does. A host that delivers three USB packets at once
    /// moves the level by three frames' worth, and anything narrower than that
    /// corrects against delivery jitter instead of drift.
    ///
    /// It must also stay comfortably below `N / 2`, or the deadband edge sits
    /// too close to an end of the buffer to recover from.
    pub const fn new_with_hysteresis(rate: u32, hyst: usize) -> Self {
        Self {
            ring: [[0i16; 2]; N],
            head: 0,
            tail: 0,
            len: 0,
            target: N / 2,
            hyst,
            accum: 0,
            rate,
            last: [0, 0],
            // Written out rather than `Stats::default()` because `Default` is
            // not const.
            stats: Stats { overruns: 0, underruns: 0, adj_up: 0, adj_down: 0 },
            primed: false,
            stress: false,
        }
    }

    /// Frames currently buffered.
    pub fn fill(&self) -> usize {
        self.len
    }

    /// Fill level the pacer steers toward.
    pub fn target(&self) -> usize {
        self.target
    }

    /// True once enough has been captured to start handing out audio.
    pub fn primed(&self) -> bool {
        self.primed
    }

    /// The deadband. Callers wiring up I2S must keep their DMA block smaller
    /// than this or the pacer will chase its own sampling phase.
    pub fn hysteresis(&self) -> usize {
        self.hyst
    }

    /// How far the buffer has wandered from target, in frames.
    ///
    /// In [`PaceMode::Locked`] this is the early warning that the SOF-derived
    /// clock has come unlocked: the buffer takes tens of seconds to actually
    /// run dry, but it starts sliding immediately. Poll it rather than waiting
    /// for `stats.underruns`.
    pub fn off_target(&self) -> i32 {
        self.len as i32 - self.target as i32
    }

    /// True when the buffer has drifted outside the deadband — in Locked mode,
    /// a bug; in Elastic mode, normal and about to be corrected.
    pub fn drifting(&self) -> bool {
        self.off_target().unsigned_abs() as usize > self.hyst
    }

    /// Change the sample rate the pacer is working to.
    ///
    /// Needed because the rate is not known until the source has been measured,
    /// but the pipe lives in a `static` and so must be const-constructed with a
    /// provisional one. Resets the fractional accumulator: a rate change makes
    /// any partial frame meaningless.
    pub fn set_rate(&mut self, rate: u32) {
        self.rate = rate;
        self.accum = 0;
        self.hyst = (rate as usize / 1000) * 2;
    }

    /// The rate currently being paced to.
    pub fn rate(&self) -> u32 {
        self.rate
    }

    /// True when the buffer has run completely dry.
    ///
    /// Distinct from an underrun statistic: this means the source has stopped
    /// entirely (playback paused, host stopped delivering) rather than drifted.
    /// Treat it as "not streaming" and stop pacing — otherwise the pacer keeps
    /// trying to correct a buffer that has nothing in it, which shows up as a
    /// storm of slips for as long as the pause lasts.
    pub fn starved(&self) -> bool {
        self.len == 0
    }

    /// Drop everything — call when the source clock stops (phone unplugged) so
    /// stale audio isn't played when it comes back.
    pub fn reset(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.len = 0;
        self.accum = 0;
        self.primed = false;
        self.last = [0, 0];
    }

    /// Producer side: push one captured frame. Returns false if it was dropped
    /// because the buffer was full (an overrun).
    pub fn push(&mut self, f: Frame) -> bool {
        if self.len == N {
            // Drop the oldest rather than the newest: staying close to live is
            // worth more than the sample we throw away, and the pacer will
            // shed the excess within a few frames anyway.
            self.tail = (self.tail + 1) % N;
            self.len -= 1;
            self.stats.overruns = self.stats.overruns.saturating_add(1);
        }
        self.ring[self.head] = f;
        self.head = (self.head + 1) % N;
        self.len += 1;
        if !self.primed && self.len >= self.target {
            self.primed = true;
        }
        true
    }

    /// Discard the oldest frames until the level is back at target.
    ///
    /// For use when a stream starts or restarts. Nothing drains the pipe while
    /// the host sits on alt 0, so it pegs at capacity; the pacer then sheds only
    /// one frame per packet and *stops as soon as the level is inside the
    /// deadband*, so it never returns to target. The level parks at the deadband
    /// edge and stays there for the rest of the session, carrying extra latency
    /// that depends on nothing but the order things were plugged in — and the
    /// first thing the host hears is a backlog rather than live audio.
    ///
    /// Dropping it in one step is a single splice, at the one moment the stream
    /// is starting anyway. Not counted as overruns: this is deliberate.
    pub fn trim_to_target(&mut self) {
        while self.len > self.target {
            self.tail = (self.tail + 1) % N;
            self.len -= 1;
        }
    }

    /// Push interleaved stereo samples straight from an I2S DMA buffer.
    /// An odd trailing sample is ignored.
    pub fn push_interleaved(&mut self, samples: &[i16]) {
        for pair in samples.chunks_exact(2) {
            self.push([pair[0], pair[1]]);
        }
    }

    /// Nominal frames for the next USB frame, exact on average for fractional
    /// rates (44100 -> 44,44,44,44,44,44,44,44,44,45, repeating).
    fn nominal(&mut self) -> usize {
        self.accum += self.rate;
        let n = (self.accum / 1000) as usize;
        self.accum %= 1000;
        n
    }

    /// How many frames the next USB packet should carry, after drift control.
    fn plan(&mut self, mode: PaceMode) -> usize {
        let n = self.nominal();
        if mode == PaceMode::Locked || n == 0 {
            return n;
        }
        if mode == PaceMode::Stress {
            // Alternate -1/+1 so the long-run average stays exact and the buffer
            // does not walk; only the packet size moves.
            self.stress = !self.stress;
            return if self.stress { n + 1 } else { n - 1 };
        }
        // Pace whether or not the pipe has primed, but only *count* once it
        // has.
        //
        // Taking one frame fewer while the level is low is not a correction
        // against drift — it is the only mechanism that builds the cushion.
        // With producer and consumer both at exactly one packet per millisecond
        // and the buffer starting empty, the level stays at zero forever unless
        // the consumer takes slightly less, so the pipe never primes and `pop()`
        // returns held samples: audible as crustiness, in proportion to signal
        // level. That is what the single-chip build did in the car.
        //
        // An earlier version returned `n` while unprimed, to stop a pipe with no
        // source logging a thousand corrections a second. Right observation
        // about the counters, wrong conclusion about the pacing.
        if self.len > self.target + self.hyst {
            if self.primed {
                self.stats.adj_up = self.stats.adj_up.saturating_add(1);
            }
            n + 1
        } else if self.len < self.refill_floor() {
            if self.primed {
                self.stats.adj_down = self.stats.adj_down.saturating_add(1);
            }
            n - 1
        } else {
            n
        }
    }

    /// The level below which the pacer takes one frame fewer.
    ///
    /// Once primed this is the bottom of the deadband, so the pacer ignores
    /// ordinary jitter. Before priming it is the target itself, because the
    /// deadband would otherwise stop the refill at `target - hyst` — below the
    /// level priming requires. Against a source that matches the sink exactly,
    /// that difference is the whole game: the level parks just under the
    /// deadband, the pipe never primes, and `pop()` returns held samples
    /// forever, which is audible as crustiness in proportion to signal level.
    ///
    /// There is no jitter to ignore while the buffer is still filling, so there
    /// is nothing to trade away by removing the deadband there.
    fn refill_floor(&self) -> usize {
        if self.primed {
            self.target.saturating_sub(self.hyst)
        } else {
            self.target
        }
    }

    /// Frames to move when the natural batch is something other than one USB
    /// frame — an I2S DMA block, say. Same +/-1 drift control as [`Self::take`],
    /// anchored to `nominal` instead of the sample-rate accumulator.
    ///
    /// Used by the source board, where the consumer is I2S rather than USB.
    pub fn plan_batch(&mut self, nominal: usize, mode: PaceMode) -> usize {
        if mode == PaceMode::Locked || mode == PaceMode::Stress || nominal == 0 {
            return nominal;
        }
        if self.len > self.target + self.hyst {
            self.stats.adj_up = self.stats.adj_up.saturating_add(1);
            nominal + 1
        } else if self.len + self.hyst < self.target {
            self.stats.adj_down = self.stats.adj_down.saturating_add(1);
            nominal - 1
        } else {
            nominal
        }
    }

    /// Drift correction for a sink whose rate we **cannot** vary — a fixed I2S
    /// clock, as opposed to a USB endpoint whose packet size we choose.
    ///
    /// Returns how many *extra* frames to draw from the ring this batch:
    ///
    /// * `+1` — buffer too full: draw one extra and **discard** it.
    /// * `-1` — buffer too empty: draw one fewer and **repeat** one frame to
    ///   pad the batch back to size.
    /// * `0` — draw the batch as-is.
    ///
    /// Side-effect free on purpose: an earlier version popped the discarded
    /// frame itself *and* returned an adjustment, so callers double-counted it
    /// and the buffer walked off target anyway. Here the caller does all the
    /// moving and the arithmetic is impossible to get wrong.
    ///
    /// Unlike [`Self::plan_batch`], which only works when the consumer accepts a
    /// variable batch, this is a real sample slip — one duplicated or discarded
    /// frame, a ~20 us discontinuity at 48 kHz. At a typical 20-50 ppm mismatch
    /// it fires a couple of times a second and is inaudible on music or speech,
    /// but it is not free: the only way to avoid it entirely is an explicit
    /// feedback endpoint so the *host* adjusts its rate instead.
    pub fn slip(&mut self, mode: PaceMode) -> i32 {
        if mode == PaceMode::Locked || mode == PaceMode::Stress {
            return 0;
        }
        // The FULL deadband, not a fraction of it.
        //
        // Same rule as [`Self::hyst`]: the band must exceed the burst
        // granularity of both sides, or the correction fires on sampling phase
        // rather than on drift. On the source board the producer delivers one
        // USB packet (48 frames) at a time and the consumer takes one I2S DMA
        // block (64 frames) at a time, so the level jitters by up to 64 frames
        // with the clocks perfectly matched. An earlier version used hyst/4 = 24
        // and slipped continuously against that jitter — audible on loud
        // material, because a slip's discontinuity scales with sample value.
        let band = self.hyst as i32;
        let off = self.off_target();
        if off > band {
            self.stats.adj_up = self.stats.adj_up.saturating_add(1);
            1
        } else if off < -band {
            self.stats.adj_down = self.stats.adj_down.saturating_add(1);
            -1
        } else {
            0
        }
    }

    /// Pop one frame, holding the last sample on underrun.
    pub fn pop(&mut self) -> Frame {
        if self.len > 0 {
            let f = self.ring[self.tail];
            self.tail = (self.tail + 1) % N;
            self.len -= 1;
            self.last = f;
            f
        } else {
            if self.primed {
                self.stats.underruns = self.stats.underruns.saturating_add(1);
            }
            self.last
        }
    }

    /// Consumer side: fill one USB frame's worth of little-endian 16-bit stereo
    /// PCM into `out`, returning the number of **bytes** written.
    ///
    /// Before priming, emits nominal-length silence so the endpoint keeps
    /// streaming (the car sees a live mic) while the buffer fills.
    pub fn take(&mut self, out: &mut [u8], mode: PaceMode) -> usize {
        let want = self.plan(mode);
        let nbytes = want * 4; // 2 channels * 2 bytes
        debug_assert!(nbytes <= out.len(), "sink buffer too small for one frame");
        let nbytes = nbytes.min(out.len() & !3);

        for i in 0..nbytes / 4 {
            // Hold the last sample on underrun rather than emitting a zero: a
            // held value is a far smaller discontinuity than a jump to silence.
            let f = self.pop();
            let l = f[0].to_le_bytes();
            let r = f[1].to_le_bytes();
            out[i * 4] = l[0];
            out[i * 4 + 1] = l[1];
            out[i * 4 + 2] = r[0];
            out[i * 4 + 3] = r[1];
        }
        nbytes
    }
}

/// Measures the source's real sample rate by counting captured frames against
/// USB frames (which arrive at exactly 1 kHz from the host).
///
/// Exists because neither the PCM2706 nor any other class-compliant bridge can
/// be locked to one rate — the phone picks, and if it picks something we didn't
/// advertise to the car, the audio must be muted rather than played at the
/// wrong pitch.
pub struct RateDetect {
    frames: u32,
    ticks: u32,
    window: u32,
    /// Last completed measurement in Hz, `None` until the first window closes.
    pub measured: Option<u32>,
}

impl RateDetect {
    /// `window` is how many USB frames (milliseconds) to average over. 1000 (one
    /// second) resolves 1 Hz, far finer than needed to tell 44100 from 48000.
    pub fn new(window: u32) -> Self {
        Self { frames: 0, ticks: 0, window, measured: None }
    }

    /// Call with the number of frames captured since the last call.
    pub fn on_capture(&mut self, n: u32) {
        self.frames = self.frames.saturating_add(n);
    }

    /// Call once per USB SOF. Returns the measured rate when a window closes.
    pub fn on_usb_frame(&mut self) -> Option<u32> {
        self.ticks += 1;
        if self.ticks < self.window {
            return None;
        }
        let hz = self.frames * 1000 / self.ticks;
        self.frames = 0;
        self.ticks = 0;
        self.measured = Some(hz);
        Some(hz)
    }

    /// Clear state (source disappeared).
    pub fn reset(&mut self) {
        self.frames = 0;
        self.ticks = 0;
        self.measured = None;
    }
}

/// Every rate a class-compliant USB bridge might hand us.
pub const STANDARD_RATES: [u32; 7] = [8000, 16000, 32000, 44100, 48000, 88200, 96000];

/// Snap a measured rate to the nearest standard rate, within 2%.
///
/// 2% is far tighter than the gap between adjacent standard rates (the closest
/// pair, 44100 and 48000, differ by 8.8%) and far looser than any crystal error,
/// so this cannot misclassify a working source.
pub fn classify(hz: u32) -> Option<u32> {
    STANDARD_RATES
        .iter()
        .copied()
        .find(|&r| {
            let d = if hz > r { hz - r } else { r - hz };
            d * 50 <= r
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drive `ms` milliseconds of traffic with the source running `ppm` parts
    /// per million fast (positive) or slow (negative) relative to the sink.
    /// Returns the pipe so the caller can inspect stats and fill level.
    fn run(rate: u32, ppm: i32, ms: u32, mode: PaceMode) -> Pipe<512> {
        let mut p = Pipe::<512>::new(rate);
        let mut out = [0u8; 1024];
        // Source-side fractional accumulator, independent of the pipe's.
        let mut src_accum: i64 = 0;
        // Prime the buffer first, as the real firmware does.
        while !p.primed() {
            p.push([1, -1]);
        }
        let per_ms = rate as i64 * (1_000_000 + ppm as i64) / 1_000_000;
        for _ in 0..ms {
            src_accum += per_ms;
            let n = src_accum / 1000;
            src_accum %= 1000;
            for _ in 0..n {
                p.push([1, -1]);
            }
            p.take(&mut out, mode);
        }
        p
    }

    #[test]
    fn nominal_is_exact_for_integer_rates() {
        let mut p = Pipe::<512>::new(48000);
        for _ in 0..1000 {
            assert_eq!(p.nominal(), 48);
        }
    }

    #[test]
    fn nominal_averages_exactly_for_44100() {
        let mut p = Pipe::<512>::new(44100);
        let mut total = 0;
        let mut counts = [0usize; 2];
        for _ in 0..1000 {
            let n = p.nominal();
            total += n;
            match n {
                44 => counts[0] += 1,
                45 => counts[1] += 1,
                other => panic!("unexpected frame size {other}"),
            }
        }
        // Exactly 44100 samples per second, as 900 short frames and 100 long.
        assert_eq!(total, 44100);
        assert_eq!(counts, [900, 100]);
    }

    #[test]
    fn take_writes_expected_byte_counts() {
        let mut p = Pipe::<512>::new(48000);
        let mut out = [0u8; 1024];
        for _ in 0..64 {
            p.push([0x1234u16 as i16, 0x5678u16 as i16]);
        }
        assert_eq!(p.take(&mut out, PaceMode::Locked), 48 * 4);
        assert_eq!(&out[0..4], &[0x34, 0x12, 0x78, 0x56]);
    }

    #[test]
    fn matched_clocks_never_correct() {
        let p = run(48000, 0, 60_000, PaceMode::Elastic);
        assert_eq!(p.stats.overruns, 0);
        assert_eq!(p.stats.underruns, 0);
        assert_eq!(p.stats.adj_up, 0, "corrected with no drift to correct");
        assert_eq!(p.stats.adj_down, 0);
    }

    #[test]
    fn deadband_survives_bursty_delivery_at_zero_drift() {
        // Regression: real I2S hands over a whole DMA block at once, so fill
        // sawtooths by the block size. With a deadband narrower than the block,
        // the pacer corrects against its own sampling phase and mangles audio
        // that had nothing wrong with it.
        for block in [1usize, 16, 48, 64] {
            let mut p = Pipe::<512>::new(48000);
            let mut out = [0u8; 1024];
            while !p.primed() {
                p.push([0, 0]);
            }
            assert!(p.hysteresis() >= block, "deadband {} < block {block}", p.hysteresis());
            let mut owed = 0usize;
            for _ in 0..10_000 {
                owed += 48;
                while owed >= block {
                    for _ in 0..block {
                        p.push([0, 0]);
                    }
                    owed -= block;
                }
                p.take(&mut out, PaceMode::Elastic);
            }
            assert_eq!(p.stats.adj_up, 0, "block {block}: corrected on burst phase");
            assert_eq!(p.stats.adj_down, 0, "block {block}: corrected on burst phase");
        }
    }

    #[test]
    fn elastic_absorbs_fast_source_without_loss() {
        // +100 ppm for a minute is a worst-case pair of cheap crystals.
        let p = run(48000, 100, 60_000, PaceMode::Elastic);
        assert_eq!(p.stats.overruns, 0, "buffer overflowed instead of shedding");
        assert_eq!(p.stats.underruns, 0);
        assert!(p.stats.adj_up > 0, "never corrected for a fast source");
        assert!(p.fill() <= p.target() + p.hysteresis() + 96, "level ran away");
    }

    #[test]
    fn elastic_absorbs_slow_source_without_loss() {
        let p = run(48000, -100, 60_000, PaceMode::Elastic);
        assert_eq!(p.stats.overruns, 0);
        assert_eq!(p.stats.underruns, 0, "ran dry instead of refilling");
        assert!(p.stats.adj_down > 0, "never corrected for a slow source");
    }

    #[test]
    fn elastic_survives_an_hour_of_drift() {
        // The failure mode this whole module exists to prevent: slow leak.
        let p = run(48000, 50, 3_600_000, PaceMode::Elastic);
        assert_eq!(p.stats.overruns, 0);
        assert_eq!(p.stats.underruns, 0);
    }

    #[test]
    fn elastic_handles_fractional_rate_with_drift() {
        let p = run(44100, 80, 60_000, PaceMode::Elastic);
        assert_eq!(p.stats.overruns, 0);
        assert_eq!(p.stats.underruns, 0);
    }

    #[test]
    fn locked_mode_never_varies_packet_size() {
        let mut p = Pipe::<512>::new(48000);
        let mut out = [0u8; 1024];
        while !p.primed() {
            p.push([0, 0]);
        }
        for _ in 0..10_000 {
            for _ in 0..48 {
                p.push([0, 0]);
            }
            // Every packet identical — this is what the car's driver expects.
            assert_eq!(p.take(&mut out, PaceMode::Locked), 192);
        }
        assert_eq!(p.stats.adj_up, 0);
        assert_eq!(p.stats.adj_down, 0);
    }

    #[test]
    fn locked_mode_reports_underrun_rather_than_hiding_it() {
        // A drifting source in Locked mode means the SOF lock is broken; the
        // counters must show it instead of silently smoothing it away.
        // -200 ppm drains 9.6 frames/s, so a 256-frame cushion survives ~27 s
        // before the first underrun — hence a 60 s run, not 10.
        let p = run(48000, -200, 60_000, PaceMode::Locked);
        assert!(p.stats.underruns > 0, "starved silently in locked mode");
        assert_eq!(p.stats.adj_down, 0, "locked mode must not pace-correct");
    }

    #[test]
    fn underrun_holds_last_sample_not_silence() {
        let mut p = Pipe::<512>::new(48000);
        let mut out = [0u8; 1024];
        while !p.primed() {
            p.push([0x0100, 0x0200]);
        }
        // Drain far past what was buffered.
        for _ in 0..100 {
            p.take(&mut out, PaceMode::Locked);
        }
        assert_eq!(&out[0..4], &[0x00, 0x01, 0x00, 0x02], "emitted silence, not the held sample");
        assert!(p.stats.underruns > 0);
    }

    #[test]
    fn overrun_drops_oldest_and_stays_live() {
        let mut p = Pipe::<8>::new(48000);
        for i in 0..12 {
            p.push([i as i16, 0]);
        }
        assert_eq!(p.stats.overruns, 4);
        assert_eq!(p.fill(), 8);
        let mut out = [0u8; 1024];
        p.take(&mut out, PaceMode::Locked);
        // Oldest four were discarded, so playback resumes at sample 4.
        assert_eq!(i16::from_le_bytes([out[0], out[1]]), 4);
    }

    #[test]
    fn reset_clears_everything() {
        let mut p = Pipe::<512>::new(48000);
        for _ in 0..300 {
            p.push([5, 5]);
        }
        assert!(p.primed());
        p.reset();
        assert_eq!(p.fill(), 0);
        assert!(!p.primed());
    }

    #[test]
    fn plan_batch_tracks_drift_like_take_does() {
        let mut p = Pipe::<512>::new(48000);
        while !p.primed() {
            p.push([0, 0]);
        }
        // Sitting on target: no correction.
        assert_eq!(p.plan_batch(64, PaceMode::Elastic), 64);
        assert_eq!(p.plan_batch(64, PaceMode::Locked), 64);
        // Overfull: shed one.
        for _ in 0..300 {
            p.push([0, 0]);
        }
        assert_eq!(p.plan_batch(64, PaceMode::Elastic), 65);
        // Locked mode never corrects, however far off it is.
        assert_eq!(p.plan_batch(64, PaceMode::Locked), 64);
    }

    #[test]
    fn trim_drops_a_backlog_and_keeps_the_newest() {
        let mut p = Pipe::<512>::new(48000);
        // Fill past target, as an alt-0 period does.
        for i in 0..512 {
            p.push([i as i16, i as i16]);
        }
        assert_eq!(p.off_target(), 512 - p.target() as i32);
        p.trim_to_target();
        assert_eq!(p.off_target(), 0, "trim did not reach target");
        // The oldest go, so what remains is the most recent audio.
        assert_eq!(p.pop(), [256, 256], "trim kept the wrong end");
        // Deliberate, so not counted as a fault.
        assert_eq!(p.stats.overruns, 0, "trim counted as overruns");
    }

    #[test]
    fn trim_is_a_no_op_below_target() {
        let mut p = Pipe::<512>::new(48000);
        for _ in 0..10 {
            p.push([1, 1]);
        }
        let before = p.off_target();
        p.trim_to_target();
        assert_eq!(p.off_target(), before, "trim removed frames it should not have");
    }

    #[test]
    fn unprimed_pipe_paces_but_does_not_count() {
        // A host can stream before any source is connected. That is an empty
        // buffer, not drift, so nothing is counted — but the short packet must
        // still be emitted, because it is what lets the buffer fill once a
        // source does appear.
        let mut p = Pipe::<512>::new(48000);
        let mut out = [0u8; 256];
        let n = p.take(&mut out, PaceMode::Elastic) / 4;
        assert_eq!(n, p.nominal() - 1, "unprimed pipe must still shed a frame");
        for _ in 0..100 {
            p.take(&mut out, PaceMode::Elastic);
        }
        assert_eq!(p.stats.adj_up, 0, "counted while unprimed");
        assert_eq!(p.stats.adj_down, 0, "counted while unprimed");
    }

    #[test]
    fn a_matched_source_and_sink_still_prime() {
        // The case that made the single-chip build crackle: producer and
        // consumer both at exactly one packet per millisecond, buffer starting
        // empty. Taking one frame fewer while low is the only thing that builds
        // the cushion; without it the level never leaves zero.
        let mut p = Pipe::<256>::new_with_hysteresis(48000, 64);
        let mut out = [0u8; 256];
        for _ in 0..2000 {
            for _ in 0..48 {
                p.push([1234, -1234]);
            }
            p.take(&mut out, PaceMode::Elastic);
            if p.primed() {
                break;
            }
        }
        assert!(p.primed(), "never primed against a matched source");
        assert!(
            p.off_target().abs() <= 64,
            "primed but not near target: {}",
            p.off_target()
        );
    }

    #[test]
    fn primed_pipe_still_paces() {
        // The guard above must not disable pacing once audio is flowing.
        let mut p = Pipe::<512>::new(48000);
        for _ in 0..600 {
            p.push([0, 0]);
        }
        assert!(p.primed());
        let mut out = [0u8; 256];
        for _ in 0..10 {
            p.take(&mut out, PaceMode::Elastic);
        }
        assert!(
            p.stats.adj_up > 0,
            "an over-full primed pipe must still shed"
        );
    }

    #[test]
    fn pop_and_take_agree() {
        let mut a = Pipe::<512>::new(48000);
        let mut b = Pipe::<512>::new(48000);
        for i in 0..200i16 {
            a.push([i, -i]);
            b.push([i, -i]);
        }
        let mut out = [0u8; 1024];
        let n = b.take(&mut out, PaceMode::Locked) / 4;
        for i in 0..n {
            let f = a.pop();
            assert_eq!(i16::from_le_bytes([out[i * 4], out[i * 4 + 1]]), f[0]);
            assert_eq!(i16::from_le_bytes([out[i * 4 + 2], out[i * 4 + 3]]), f[1]);
        }
        assert_eq!(a.fill(), b.fill());
    }

    /// A fixed-rate sink (I2S) draining at exactly `nominal` per batch while the
    /// producer runs `ppm` off. Without `slip` this pins to an end of the buffer.
    fn run_fixed_sink(ppm: i32, batches: u32, use_slip: bool) -> Pipe<512> {
        let mut p = Pipe::<512>::new(48000);
        const BATCH: usize = 64;
        while !p.primed() {
            p.push([0, 0]);
        }
        let mut src: i64 = 0;
        // Frames the producer delivers per batch period, scaled by ppm.
        let per_batch = BATCH as i64 * (1_000_000 + ppm as i64);
        for _ in 0..batches {
            src += per_batch;
            let n = src / 1_000_000;
            src %= 1_000_000;
            for _ in 0..n {
                p.push([1, -1]);
            }
            // The sink always consumes exactly BATCH frames; `slip` says how
            // many extra to draw from the ring (discarding) or fewer (repeating).
            let adj = if use_slip { p.slip(PaceMode::Elastic) } else { 0 };
            let draw = BATCH as i32 + adj;
            for _ in 0..draw {
                p.pop();
            }
        }
        p
    }

    #[test]
    fn fixed_sink_without_slip_pins_to_an_end() {
        // Demonstrates the bug this exists to fix: with no slip correction the
        // buffer walks to an end and stays there, underrunning continuously.
        let p = run_fixed_sink(-100, 200_000, false);
        assert!(
            p.stats.underruns > 1000,
            "expected sustained underrun, got {}",
            p.stats.underruns
        );
    }

    #[test]
    fn fixed_sink_with_slip_stays_centred() {
        for ppm in [-200, -50, 0, 50, 200] {
            let p = run_fixed_sink(ppm, 200_000, true);
            assert_eq!(p.stats.underruns, 0, "{ppm} ppm: underran");
            assert_eq!(p.stats.overruns, 0, "{ppm} ppm: overran");
            // `slip` samples the level *before* the batch is popped, so the
            // level observed here reads one batch lower. The bound is therefore
            // the deadband plus a batch, not the deadband alone.
            const BATCH: usize = 64;
            assert!(
                p.off_target().unsigned_abs() as usize <= p.hysteresis() + BATCH + 4,
                "{ppm} ppm: drifted to {}",
                p.off_target()
            );
        }
    }

    #[test]
    fn slip_is_disabled_when_clock_locked() {
        let mut p = Pipe::<512>::new(48000);
        while !p.primed() {
            p.push([0, 0]);
        }
        for _ in 0..400 {
            p.push([0, 0]);
        }
        assert_eq!(p.slip(PaceMode::Locked), 0, "slipped despite a locked clock");
    }

    #[test]
    fn stress_mode_swings_packet_size_but_keeps_average_exact() {
        let mut p = Pipe::<512>::new(48000);
        let mut out = [0u8; 1024];
        while !p.primed() {
            p.push([0, 0]);
        }
        let (mut total, mut seen_small, mut seen_big) = (0usize, false, false);
        for _ in 0..1000 {
            for _ in 0..48 {
                p.push([0, 0]);
            }
            let n = p.take(&mut out, PaceMode::Stress) / 4;
            match n {
                47 => seen_small = true,
                49 => seen_big = true,
                other => panic!("stress emitted {other}, expected 47 or 49"),
            }
            total += n;
        }
        assert!(seen_small && seen_big, "did not exercise both packet sizes");
        // Average must stay exactly 48 so the test isolates packet size from drift.
        assert_eq!(total, 48_000);
        assert_eq!(p.stats.underruns, 0);
        assert_eq!(p.stats.overruns, 0);
    }

    #[test]
    fn slip_ignores_burst_phase_with_matched_clocks() {
        // Regression: producer and consumer both deliver in bursts, of different
        // sizes, with zero drift. Nothing should ever be slipped.
        for (in_burst, out_burst) in [(48, 64), (64, 48), (48, 48), (1, 64), (64, 1)] {
            let mut p = Pipe::<512>::new(48000);
            while !p.primed() {
                p.push([0, 0]);
            }
            let mut owed_in = 0usize;
            for _ in 0..20_000 {
                // Same number of frames in as out, just delivered in lumps.
                owed_in += out_burst;
                while owed_in >= in_burst {
                    for _ in 0..in_burst {
                        p.push([0, 0]);
                    }
                    owed_in -= in_burst;
                }
                let adj = p.slip(PaceMode::Elastic);
                for _ in 0..(out_burst as i32 + adj) {
                    p.pop();
                }
            }
            assert_eq!(
                p.stats.adj_up + p.stats.adj_down,
                0,
                "in {in_burst}/out {out_burst}: slipped on burst phase, not drift"
            );
        }
    }

    #[test]
    fn starved_reports_a_stopped_source() {
        let mut p = Pipe::<512>::new(48000);
        assert!(p.starved());
        while !p.primed() {
            p.push([1, 1]);
        }
        assert!(!p.starved());
        // Drain it the way a paused host would.
        for _ in 0..600 {
            p.pop();
        }
        assert!(p.starved(), "a dry buffer must report starved");
    }

    #[test]
    fn wider_hysteresis_tolerates_bigger_bursts() {
        // Four USB frames of deadband must survive a host bunching 3 packets.
        let mut p = Pipe::<1024>::new_with_hysteresis(48000, 192);
        assert_eq!(p.hysteresis(), 192);
        while !p.primed() {
            p.push([0, 0]);
        }
        for _ in 0..5000 {
            for _ in 0..3 {
                for _ in 0..48 {
                    p.push([0, 0]);
                }
            }
            for _ in 0..3 {
                let adj = p.slip(PaceMode::Elastic);
                for _ in 0..(48i32 + adj) {
                    p.pop();
                }
            }
        }
        assert_eq!(p.stats.adj_up + p.stats.adj_down, 0, "slipped on 3-packet bursts");
    }

    #[test]
    fn set_rate_repaces_and_clears_the_accumulator() {
        let mut p = Pipe::<512>::new(48000);
        let mut out = [0u8; 1024];
        while !p.primed() {
            p.push([0, 0]);
        }
        assert_eq!(p.take(&mut out, PaceMode::Locked) / 4, 48);

        p.set_rate(44100);
        assert_eq!(p.rate(), 44100);
        // 44.1 kHz must now pace 44/45, averaging exactly 44100 over a second.
        let mut total = 0;
        for _ in 0..1000 {
            for _ in 0..45 {
                p.push([0, 0]);
            }
            let n = p.take(&mut out, PaceMode::Locked) / 4;
            assert!(n == 44 || n == 45, "unexpected frame size {n} at 44.1 kHz");
            total += n;
        }
        assert_eq!(total, 44_100);
    }

    #[test]
    fn rate_detect_measures_each_standard_rate() {
        for rate in STANDARD_RATES {
            let mut d = RateDetect::new(1000);
            let mut accum = 0u32;
            let mut got = None;
            for _ in 0..1000 {
                accum += rate;
                d.on_capture(accum / 1000);
                accum %= 1000;
                if let Some(hz) = d.on_usb_frame() {
                    got = Some(hz);
                }
            }
            assert_eq!(got, Some(rate), "misread {rate}");
            assert_eq!(classify(got.unwrap()), Some(rate));
        }
    }

    #[test]
    fn classify_tolerates_crystal_error_but_rejects_nonsense() {
        assert_eq!(classify(48001), Some(48000));
        assert_eq!(classify(47990), Some(48000));
        assert_eq!(classify(44105), Some(44100));
        // Halfway between two standard rates is not a rate.
        assert_eq!(classify(46000), None);
        assert_eq!(classify(0), None);
    }

    #[test]
    fn classify_separates_44100_from_48000() {
        // The distinction the whole mute-on-mismatch behaviour depends on.
        assert_eq!(classify(44100), Some(44100));
        assert_eq!(classify(48000), Some(48000));
        assert_ne!(classify(44100), classify(48000));
    }
}

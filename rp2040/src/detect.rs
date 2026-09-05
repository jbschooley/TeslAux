// SPDX-License-Identifier: MIT
//! Turning the car's disk reads into button presses.
//!
//! The car tells a connected microphone nothing about its transport buttons —
//! measured, see `MEDIA-CONTROLS.md`. But it does play a playlist we authored,
//! and because we authored it, **which sectors it reads says which track it
//! moved to**. Press next and the car stops reading track 3 and starts reading
//! track 4.
//!
//! This reads the car's *destination*, never its keystrokes. Press next five
//! times quickly and the car lands on track 8; the answer is `+5` whether the
//! player opened each file on the way or jumped straight there, and there is no
//! press to miss. An event counter would be at the mercy of how the player
//! coalesces rapid input.
//!
//! Everything here is pure logic over `(track, offset)` pairs and a millisecond
//! clock — no USB, no HAL — so the judgements it makes are testable on the host.
//! That matters more than usual: every judgement is a chance to report a button
//! press that never happened, and a wrong one is directly audible to the user as
//! a track skipping by itself.

#![allow(dead_code)]

/// What the car did.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Event {
    /// The user pressed next `n` times.
    Next(u32),
    /// The user pressed previous `n` times.
    Prev(u32),
    /// Playback stopped: reads have gone quiet while a track was open.
    Paused,
    /// Playback resumed after a pause.
    Resumed,
}

/// How long reads must be quiet before playback counts as paused.
///
/// At 48 kHz stereo the car reads roughly three times a second, so a gap this
/// long is not jitter. Short enough to feel immediate on a button press, long
/// enough to ride out a slow read.
const PAUSE_AFTER_MS: u32 = 400;

/// Reads from a new track before believing the player moved there.
///
/// A host briefly touches other files while indexing — reading a header to
/// identify a format, for instance — and treating that as a track change would
/// report button presses nobody pressed. Playback produces a sustained run of
/// reads; a metadata peek does not.
const SETTLE_READS: u32 = 3;

/// How close to the end of a file counts as having played it out.
///
/// The car reads ahead of playback, so by the time a track ends its reads have
/// already reached the end of the file. A press in the last moments of a track
/// is indistinguishable from the track ending on its own — and treating a
/// natural advance as a press is the worse error, because it happens on every
/// track boundary rather than once.
const END_TOLERANCE_BYTES: u32 = 512 * 1024;

/// Where in the volume a read landed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Position {
    pub track: u32,
    pub offset: u32,
}

pub struct Detector {
    tracks: u32,
    track_bytes: u32,

    /// The track the player is believed to be on.
    current: Option<u32>,
    /// Furthest offset read within `current`, for deciding whether a track was
    /// played out or abandoned.
    high_water: u32,

    /// A track seen recently that is not yet believed.
    candidate: Option<u32>,
    candidate_reads: u32,

    last_read_ms: u32,
    paused: bool,
}

impl Detector {
    pub const fn new(tracks: u32, track_bytes: u32) -> Self {
        Self {
            tracks,
            track_bytes,
            current: None,
            high_water: 0,
            candidate: None,
            candidate_reads: 0,
            last_read_ms: 0,
            paused: false,
        }
    }

    /// The track the player is believed to be on, once one is established.
    pub fn track(&self) -> Option<u32> {
        self.current
    }

    pub fn is_paused(&self) -> bool {
        self.paused
    }

    /// Signed distance from `from` to `to`, the short way round.
    ///
    /// Twenty-four tracks and a jump from 23 to 0 is `+1`, not `-23`: the player
    /// wrapped. Ambiguous only at exactly half the playlist, which is a dozen
    /// presses in one burst.
    fn delta(&self, from: u32, to: u32) -> i32 {
        let n = self.tracks as i32;
        let raw = to as i32 - from as i32;
        if raw > n / 2 {
            raw - n
        } else if raw < -n / 2 {
            raw + n
        } else {
            raw
        }
    }

    /// Call for every sector read that lands inside a track.
    pub fn on_read(&mut self, pos: Position, now_ms: u32) -> Option<Event> {
        let resumed = self.paused;
        self.paused = false;
        self.last_read_ms = now_ms;

        // First read of the session: adopt the track without reporting a press.
        let Some(current) = self.current else {
            self.current = Some(pos.track);
            self.high_water = pos.offset;
            return None;
        };

        if pos.track == current {
            self.high_water = self.high_water.max(pos.offset);
            self.candidate = None;
            self.candidate_reads = 0;
            return if resumed { Some(Event::Resumed) } else { None };
        }

        // A different track. Wait for it to be confirmed by a sustained run
        // before believing it — see `SETTLE_READS`.
        if self.candidate != Some(pos.track) {
            self.candidate = Some(pos.track);
            self.candidate_reads = 1;
            return if resumed { Some(Event::Resumed) } else { None };
        }
        self.candidate_reads += 1;
        if self.candidate_reads < SETTLE_READS {
            return if resumed { Some(Event::Resumed) } else { None };
        }

        // Committed: the player really is on a new track.
        let delta = self.delta(current, pos.track);
        let played_out = self.high_water + END_TOLERANCE_BYTES >= self.track_bytes;

        self.current = Some(pos.track);
        self.high_water = pos.offset;
        self.candidate = None;
        self.candidate_reads = 0;

        // A track that was played to its end and advanced by one is the playlist
        // doing its job, not a button.
        if delta == 1 && played_out {
            return None;
        }

        match delta {
            0 => None,
            d if d > 0 => Some(Event::Next(d as u32)),
            d => Some(Event::Prev((-d) as u32)),
        }
    }

    // There is deliberately no restart detection.
    //
    // Most players make the first `previous` press restart the current track
    // rather than move back, and detecting that looked easy: a read landing
    // much earlier in the file than the furthest one so far. In the car it fired
    // constantly. Hosts read out of order as a matter of course — a header for
    // metadata, a seek to fill a buffer, a re-read after a gap — and every one
    // of those looks like a backwards seek.
    //
    // It reported a press on almost every genuine track change, because after
    // moving to a new track the car reads its header at offset zero. Since a
    // false press is directly audible on the source and the feature only saves
    // the user a second press, it is not worth having on those terms.

    /// Call periodically. Reports a pause once reads have been quiet.
    pub fn tick(&mut self, now_ms: u32) -> Option<Event> {
        if self.paused || self.current.is_none() {
            return None;
        }
        if now_ms.wrapping_sub(self.last_read_ms) >= PAUSE_AFTER_MS {
            self.paused = true;
            return Some(Event::Paused);
        }
        None
    }

    /// Forget the player's position, for when the host goes away.
    pub fn reset(&mut self) {
        self.current = None;
        self.high_water = 0;
        self.candidate = None;
        self.candidate_reads = 0;
        self.paused = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TRACKS: u32 = 24;
    const TRACK_BYTES: u32 = 600 * 48_000 * 4 + 44;

    fn det() -> Detector {
        Detector::new(TRACKS, TRACK_BYTES)
    }

    /// Read `n` times from `track` at `offset`, returning the last event.
    fn reads(d: &mut Detector, track: u32, offset: u32, n: u32, t0: u32) -> Option<Event> {
        let mut last = None;
        for i in 0..n {
            let e = d.on_read(Position { track, offset }, t0 + i * 10);
            if e.is_some() {
                last = e;
            }
        }
        last
    }

    /// Establish playback on a track, mid-file.
    fn playing(d: &mut Detector, track: u32, t0: u32) {
        reads(d, track, TRACK_BYTES / 2, SETTLE_READS + 2, t0);
    }

    #[test]
    fn the_first_read_is_not_a_button_press() {
        let mut d = det();
        assert_eq!(d.on_read(Position { track: 5, offset: 0 }, 0), None);
        assert_eq!(d.track(), Some(5));
    }

    #[test]
    fn one_press_forward() {
        let mut d = det();
        playing(&mut d, 3, 0);
        assert_eq!(reads(&mut d, 4, 0, SETTLE_READS, 1000), Some(Event::Next(1)));
        assert_eq!(d.track(), Some(4));
    }

    #[test]
    fn five_rapid_presses_are_read_as_five() {
        // The point of reading the destination: whether the player opened each
        // file on the way or jumped straight to track 8, the answer is +5.
        let mut d = det();
        playing(&mut d, 3, 0);
        assert_eq!(reads(&mut d, 8, 0, SETTLE_READS, 1000), Some(Event::Next(5)));
    }

    #[test]
    fn a_press_backwards() {
        let mut d = det();
        playing(&mut d, 10, 0);
        assert_eq!(reads(&mut d, 9, 0, SETTLE_READS, 1000), Some(Event::Prev(1)));
    }

    #[test]
    fn wrapping_forward_is_one_press_not_twenty_three_back() {
        let mut d = det();
        playing(&mut d, TRACKS - 1, 0);
        assert_eq!(reads(&mut d, 0, 0, SETTLE_READS, 1000), Some(Event::Next(1)));
    }

    #[test]
    fn wrapping_backward_is_one_press_not_twenty_three_forward() {
        let mut d = det();
        playing(&mut d, 0, 0);
        assert_eq!(
            reads(&mut d, TRACKS - 1, 0, SETTLE_READS, 1000),
            Some(Event::Prev(1))
        );
    }

    #[test]
    fn a_track_ending_on_its_own_is_not_a_press() {
        // This happens at every track boundary, so reporting it would skip a
        // track on the source every ten minutes.
        let mut d = det();
        reads(&mut d, 3, TRACK_BYTES - 1024, SETTLE_READS + 2, 0);
        assert_eq!(reads(&mut d, 4, 0, SETTLE_READS, 1000), None);
        assert_eq!(d.track(), Some(4));
    }

    #[test]
    fn a_press_near_the_end_of_a_track_still_counts_if_it_skips_further() {
        // Played out, but landing two tracks on is not the playlist advancing.
        let mut d = det();
        reads(&mut d, 3, TRACK_BYTES - 1024, SETTLE_READS + 2, 0);
        assert_eq!(reads(&mut d, 5, 0, SETTLE_READS, 1000), Some(Event::Next(2)));
    }

    #[test]
    fn abandoning_a_track_early_is_a_press() {
        // Same +1 as a natural advance, but from the middle of the file.
        let mut d = det();
        playing(&mut d, 3, 0);
        assert_eq!(reads(&mut d, 4, 0, SETTLE_READS, 1000), Some(Event::Next(1)));
    }

    #[test]
    fn a_metadata_peek_at_another_track_is_not_a_press() {
        // Hosts read a header here and there while indexing. Treating that as a
        // track change reports presses nobody made.
        let mut d = det();
        playing(&mut d, 3, 0);
        assert_eq!(d.on_read(Position { track: 17, offset: 0 }, 1000), None);
        assert_eq!(d.on_read(Position { track: 3, offset: TRACK_BYTES / 2 + 4096 }, 1010), None);
        assert_eq!(d.track(), Some(3), "a single stray read moved the position");
    }

    #[test]
    fn a_pause_is_reported_once_reads_go_quiet() {
        let mut d = det();
        playing(&mut d, 3, 0);
        assert_eq!(d.tick(100), None, "reported a pause while still reading");
        assert_eq!(d.tick(1000), Some(Event::Paused));
        assert_eq!(d.tick(2000), None, "reported the same pause twice");
        assert!(d.is_paused());
    }

    #[test]
    fn resuming_is_reported_and_is_not_a_press() {
        let mut d = det();
        playing(&mut d, 3, 0);
        assert_eq!(d.tick(1000), Some(Event::Paused));
        assert_eq!(
            d.on_read(Position { track: 3, offset: TRACK_BYTES / 2 + 4096 }, 1100),
            Some(Event::Resumed)
        );
        assert!(!d.is_paused());
    }

    #[test]
    fn reading_a_header_after_a_track_change_is_not_a_second_event() {
        // What restart detection got wrong: on moving to a new track the car
        // reads its header at offset zero, which looks exactly like seeking
        // backwards. Reporting that produced a `previous` alongside every
        // `next`.
        let mut d = det();
        playing(&mut d, 3, 0);
        assert_eq!(reads(&mut d, 4, 500_000, SETTLE_READS, 1000), Some(Event::Next(1)));
        assert_eq!(
            d.on_read(Position { track: 4, offset: 0 }, 1100),
            None,
            "a header read after a skip reported an event"
        );
    }

    #[test]
    fn a_full_lap_of_the_playlist_never_reports_a_press() {
        // Ten minutes a track, twenty-four tracks: a four-hour drive. Any false
        // positive here is a track skipping by itself on the source.
        let mut d = det();
        let mut t = 0u32;
        reads(&mut d, 0, TRACK_BYTES - 1024, SETTLE_READS + 2, t);
        for track in 1..TRACKS {
            t += 600_000;
            assert_eq!(
                reads(&mut d, track, TRACK_BYTES - 1024, SETTLE_READS + 2, t),
                None,
                "natural advance to track {track} reported as a press"
            );
        }
        // And the wrap back to the start is equally natural.
        t += 600_000;
        assert_eq!(reads(&mut d, 0, 0, SETTLE_READS, t), None, "wrap reported as a press");
    }
}

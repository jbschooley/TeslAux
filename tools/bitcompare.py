#!/usr/bin/env python3
"""Compare a recording made through the bridge against the file that was played.

This is the end-to-end test the LED codes only approximate. Everything the
firmware can get wrong shows up here as a specific, located difference:

  a repeated frame     -> a slip down: the pacer padded a starved batch
  a dropped frame      -> a slip up: the pacer discarded to shed drift
  a run of zeros       -> a dropout (muted, or the pipe reset)
  a constant offset    -> nothing wrong; just where the recording started
  channels exchanged   -> the I2S slot mapping is inverted
  everything shifted   -> a bit-alignment error in the I2S framing
  scaled samples       -> a gain stage somewhere; not bit-exact by definition

A clean run proves rather more than a clean LED: no sample was lost, repeated,
truncated, reordered or rescaled anywhere between the phone and the recorder.

Usage:
    bitcompare.py reference.wav recording.wav
    bitcompare.py --self-test

Both files must be 16-bit stereo PCM at the same rate. See PRECONDITIONS below —
most "failures" are the source or the recorder, not the bridge.
"""

import sys
import wave

import numpy as np

PRECONDITIONS = """\
For a bit-exact result the whole chain has to be bit-exact, which takes setup:

  * The source file must already be 48 kHz, 16-bit. Anything else is resampled
    or dithered before it reaches us, and no two resamplers agree bit for bit.
  * The player must have a bit-perfect path to USB audio, volume at 100%, with
    every effect, EQ and normalisation off. Android mixes and rescales by
    default, and a volume of 99% is a multiply.
  * The recorder must be at unity gain with no plugins, recording 48 kHz.
    Record 16-bit if you can; 24-bit is fine if the samples land in the top 16
    bits untouched.

If any of those is wrong the comparison fails everywhere at once, which is easy
to tell apart from the localised differences a firmware fault produces."""

# A frame is one stereo sample pair.
CHANNELS = 2


def read_float_wav(path):
    """Read an IEEE-float WAV, which `wave` refuses. Returns (data, rate) or None.

    Worth the trouble: CoreAudio hands applications float32, and a 16-bit sample
    converts to float32 exactly. So a float recording that comes back as whole
    numbers proves the samples reached the recorder untouched — the int16 they
    are eventually written as is a separate, later conversion, and the usual
    place a least significant bit goes missing.
    """
    with open(path, "rb") as f:
        raw = f.read()
    if raw[:4] != b"RIFF" or raw[8:12] != b"WAVE":
        return None
    pos, fmt, rate, ch, data = 12, None, None, None, None
    while pos + 8 <= len(raw):
        cid = raw[pos : pos + 4]
        size = int.from_bytes(raw[pos + 4 : pos + 8], "little")
        body = raw[pos + 8 : pos + 8 + size]
        if cid == b"fmt ":
            fmt = int.from_bytes(body[0:2], "little")
            ch = int.from_bytes(body[2:4], "little")
            rate = int.from_bytes(body[4:8], "little")
            bits = int.from_bytes(body[14:16], "little")
            if fmt == 0xFFFE and len(body) >= 26:
                fmt = int.from_bytes(body[24:26], "little")
        elif cid == b"data":
            data = body
        pos += 8 + size + (size & 1)
    if fmt != 3 or data is None or bits != 32:
        return None
    return np.frombuffer(data, dtype="<f4").reshape(-1, ch), rate


def read_wav(path):
    """Return (frames, rate) as an int16 array of shape (n, 2)."""
    fl = read_float_wav(path)
    if fl is not None:
        x, rate = fl
        scaled = x.astype(np.float64) * 32768.0
        err = np.abs(scaled - np.rint(scaled))
        integral = float(err.max()) < 1e-3
        print(
            f"note: {path} is float32; samples are "
            + (
                "exact integers -> nothing after the USB device altered them"
                if integral
                else f"NOT integral (max fractional error {err.max():.4f}) -> "
                "something scaled or resampled them"
            )
        )
        return np.clip(np.rint(scaled), -32768, 32767).astype(np.int16)[:, :CHANNELS], rate
    with wave.open(path, "rb") as w:
        if w.getnchannels() != CHANNELS:
            raise SystemExit(f"{path}: expected stereo, got {w.getnchannels()} channels")
        rate = w.getframerate()
        width = w.getsampwidth()
        raw = w.readframes(w.getnframes())
    if width == 2:
        data = np.frombuffer(raw, dtype="<i2")
    elif width == 3:
        # 24-bit packed little-endian: keep the top 16 bits, which is where a
        # 16-bit source sits untouched.
        b = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3)
        data = (b[:, 1].astype(np.int32) | (b[:, 2].astype(np.int8).astype(np.int32) << 8))
        data = data.astype(np.int16)
    elif width == 4:
        data = (np.frombuffer(raw, dtype="<i4") >> 16).astype(np.int16)
    else:
        raise SystemExit(f"{path}: unsupported sample width {width * 8}-bit")
    return data.reshape(-1, CHANNELS), rate


def find_offset(ref, rec, probe=48000, search=None):
    """Locate where `ref` begins inside `rec`.

    Correlates one channel over a probe window. Returns the offset in frames, or
    None if nothing correlates — which usually means the two files are not the
    same material rather than that the bridge failed.
    """
    if len(rec) < probe or len(ref) < probe:
        probe = min(len(rec), len(ref)) // 2
    if probe < 64:
        return None
    # Skip any leading digital silence in the reference; correlating against
    # zeros finds nothing.
    start = 0
    nz = np.nonzero(ref[:, 0])[0]
    if len(nz):
        start = int(nz[0])
    needle = ref[start : start + probe, 0].astype(np.float64)
    if not np.any(needle):
        return None
    hay = rec[: (search or len(rec)), 0].astype(np.float64)
    if len(hay) < len(needle):
        return None
    corr = _xcorr_valid(hay, needle)
    # Normalise by the energy under each window, which turns a dot product into
    # a correlation coefficient. Unnormalised, the largest product is simply the
    # loudest place in the recording rather than the matching one: this file
    # ends about three times louder than it begins, so a probe taken from the
    # quiet opening peaked in the finale and the whole comparison was reported
    # as different material. Short captures never showed it.
    energy = np.concatenate(([0.0], np.cumsum(hay * hay)))
    window = energy[len(needle) :] - energy[: -len(needle)]
    denom = np.sqrt(window) * np.linalg.norm(needle)
    denom[denom <= 0] = np.inf
    ncc = np.abs(corr / denom)
    # Take the EARLIEST offset that matches about as well as the best one, not
    # the best one itself. A recording left running past the end of the material
    # contains it twice, both copies correlate at 1.000, and which of them wins
    # is decided by whichever has fewer edits inside the one-second probe. That
    # is how a fourteen-minute capture aligned on its own last two minutes and
    # reported them as the whole result, silently discarding the eleven minutes
    # actually under test.
    best = float(ncc.max())
    if best > 0:
        # Take the earliest *place* that matches about as well as the best one,
        # then the exact peak within it. Taking the first index above the
        # threshold instead is not the same thing and is wrong: neighbouring
        # lags of smooth material correlate well above 0.98, so it lands a few
        # frames early — which then reads as a spurious edit at the anchor,
        # because the walk starts misaligned by exactly that much.
        good = np.nonzero(ncc >= 0.98 * best)[0]
        cluster = good[good <= good[0] + len(needle)]
        peak = int(cluster[np.argmax(ncc[cluster])])
    else:
        peak = int(np.argmax(ncc))
    return peak - start


def _xcorr_valid(hay, needle):
    """Cross-correlation of `needle` against `hay` at every valid offset.

    Identical in result to np.correlate(hay, needle, mode="valid"), but that
    does the sum directly, in O(len(hay) * len(needle)). Against a ten-minute
    recording — thirty million frames, a one-second probe — that is on the order
    of 10^12 multiply-adds and never finishes; the first attempt at this had to
    be killed. Overlap-save with an FFT gives the same numbers in O(n log n) and
    in memory bounded by the segment size rather than the recording.
    """
    m = len(needle)
    n_out = len(hay) - m + 1
    # Segments a few times the needle keep the discarded wrap-around region a
    # small fraction of each transform.
    seg = max(1 << (4 * m - 1).bit_length(), 1 << 16)
    step = seg - m + 1
    # Correlation is convolution with the needle reversed.
    fr = np.fft.rfft(needle[::-1], seg)
    out = np.empty(n_out, dtype=np.float64)
    pos = 0
    while pos < n_out:
        block = hay[pos : pos + seg]
        if len(block) < seg:
            block = np.pad(block, (0, seg - len(block)))
        prod = np.fft.irfft(np.fft.rfft(block, seg) * fr, seg)
        # The first m-1 samples of a circular convolution are corrupted by
        # wrap-around; everything after them is the linear result.
        take = min(step, n_out - pos)
        out[pos : pos + take] = prod[m - 1 : m - 1 + take]
        pos += take
    return out


def classify(ref, rec, limit=20):
    """Compare aligned arrays and describe the first differences."""
    n = min(len(ref), len(rec))
    a, b = ref[:n], rec[:n]
    diff = np.any(a != b, axis=1)
    idx = np.nonzero(diff)[0]
    if len(idx) == 0:
        return []

    notes = []
    for i in idx[:limit]:
        i = int(i)
        note = {"frame": i, "ref": tuple(int(x) for x in a[i]), "rec": tuple(int(x) for x in b[i])}
        if not b[i].any():
            note["kind"] = "silence"
        elif tuple(b[i]) == tuple(a[i][::-1]):
            note["kind"] = "channels swapped"
        elif i > 0 and tuple(b[i]) == tuple(b[i - 1]):
            note["kind"] = "repeated frame (slip down)"
        elif i + 1 < n and tuple(b[i]) == tuple(a[i + 1]):
            note["kind"] = "dropped frame (slip up)"
        else:
            note["kind"] = "mismatch"
        notes.append(note)
    return notes


def explain_bulk(ref, rec):
    """When nearly everything differs, say *how* — the chain and the firmware
    fail in completely different ways.

    A single multiply (a volume control not quite at maximum, a recorder gain
    stage) leaves the signal otherwise intact, so one constant explains every
    sample. A resample leaves no such constant. Naming which is which turns a
    useless "everything differs" into a specific thing to go and fix.
    """
    a = ref.astype(np.float64).ravel()
    b = rec.astype(np.float64).ravel()
    denom = float((a * a).sum())
    if denom == 0:
        return None
    k = float((a * b).sum()) / denom
    resid = b - k * a
    rms_b = float(np.sqrt((b * b).mean()))
    if rms_b == 0:
        return None
    rel = float(np.sqrt((resid * resid).mean())) / rms_b
    if rel < 0.02 and abs(k - 1.0) > 1e-4:
        db = 20.0 * np.log10(k) if k > 0 else float("nan")
        return (
            f"a constant gain of {k:.6f} ({db:+.3f} dB) explains the whole "
            f"recording to within {rel * 100:.2f}%.\n"
            "      Something applied a multiply: a volume control not at 100%, "
            "or a gain stage\n      in the recorder. Nothing here is a bridge "
            "fault."
        )
    if rel < 0.02:
        return "the signal matches to within a constant, but that constant is 1 — check alignment."
    return (
        f"no constant gain explains it (residual {rel * 100:.1f}%).\n"
        "      That is a resample or different material, not a level change."
    )


def walk(ref, rec, off, tol=1, blk=2400, search=8000):
    """Walk both streams from `off`, tolerating small sample error, and return
    (stats, edits).

    Two things have to be separated, because they mean completely different
    things:

      * a **level error** — every sample off by a fixed small amount, typically
        one LSB from a float round-trip that truncates instead of rounding.
        Nothing is lost; the values are just not identical.
      * a **splice** — a run of frames missing outright, after which the streams
        realign perfectly. That is lost audio.

    Comparing exactly cannot tell them apart: a one-LSB level error makes every
    frame "differ", which buries the splices in noise and reports a healthy
    stream as total corruption.
    """
    edits = []
    worst = 0
    diffs = 0
    total = 0
    i, j = 0, off
    while i + blk < len(ref) and j + blk < len(rec):
        d = np.abs(ref[i : i + blk].astype(np.int32) - rec[j : j + blk].astype(np.int32))
        if d.max() <= tol:
            worst = max(worst, int(d.max()))
            diffs += int(np.count_nonzero(d))
            total += d.size
            i += blk
            j += blk
            continue
        # Find where it stops matching, then how far to skip to resync.
        #
        # Which stream to advance depends on which way the edit went. Frames
        # missing from the recording means the *reference* has to skip ahead to
        # catch up; extra frames in the recording means the opposite. Searching
        # only one of them finds neither.
        bad = int(np.argmax(d.max(axis=1) > tol))
        pos = i + bad
        jb = j + bad
        found = None
        for sh in range(1, search):
            a = ref[pos + sh : pos + sh + 512].astype(np.int32)
            b = rec[jb : jb + 512].astype(np.int32)
            if a.shape == b.shape and a.size and np.abs(a - b).max() <= tol:
                found = -sh
                i, j = pos + sh, jb
                break
            a = ref[pos : pos + 512].astype(np.int32)
            b = rec[jb + sh : jb + sh + 512].astype(np.int32)
            if a.shape == b.shape and a.size and np.abs(a - b).max() <= tol:
                found = sh
                i, j = pos, jb + sh
                break
        if found is None:
            # No shift reconciles the streams, so nothing was inserted or lost.
            # Before giving up, check whether they simply carry on in step a few
            # frames later: that means the frames in between kept their place
            # and changed their value, which is a different fault entirely — a
            # sample corrupted in transit rather than a frame dropped, and the
            # signature of a bit error on the wire. Treating it as unresolved
            # ended the walk at the first occurrence and left the rest of an
            # eleven-minute recording unexamined.
            run = 0
            while (
                pos + run < len(ref)
                and jb + run < len(rec)
                and run < blk
                and np.abs(
                    ref[pos + run].astype(np.int32) - rec[jb + run].astype(np.int32)
                ).max()
                > tol
            ):
                run += 1
            a = ref[pos + run : pos + run + 512].astype(np.int32)
            b = rec[jb + run : jb + run + 512].astype(np.int32)
            if run and a.shape == b.shape and a.size and np.abs(a - b).max() <= tol:
                edits.append((pos, "corrupt", run))
                i, j = pos + run, jb + run
                continue
            edits.append((pos, "unresolved", 0))
            break
        edits.append((pos, "missing" if found < 0 else "extra", abs(found)))
    return {"worst_lsb": worst, "diff_samples": diffs, "compared": total}, edits


def report_lead_in(ref, rec, tol, rate):
    """Say what happened in the seconds before the alignment anchor.

    Reported apart from the main walk because the two answer different
    questions. The walk asks whether a settled stream stayed intact. This asks
    whether it started intact — which is where priming, the first packets and
    the pacer's first corrections live, and so where a fault heard "right at the
    beginning" actually is. Silence here used to be indistinguishable from
    having looked.
    """
    n = len(ref)
    if n == 0:
        print("      lead-in: none (recording starts at the reference)")
        return True, 0
    secs = n / rate
    if np.array_equal(ref, rec):
        print(f"      lead-in: first {secs:.1f} s match exactly")
        return True, 0
    if n > 2400:
        _, edits = walk(ref, rec, 0, tol=tol)
        if edits:
            kinds = ", ".join(f"{k} {c}" for _, k, c in edits[:4])
            print(f"      lead-in: {len(edits)} edit(s) in the first {secs:.1f} s ({kinds})")
            for pos, kind, cnt in edits[:4]:
                print(f"        ref frame {pos:>9}  {pos / rate:6.3f} s  {kind} {cnt} frames")
            net = sum(c for _, k, c in edits if k == "extra") - sum(
                c for _, k, c in edits if k == "missing"
            )
            return False, net
    # No splice explains it, so say how big the difference actually is: at the
    # start of a stream this is usually one file having signal where the other
    # still has silence, which is not a fault and should not read like one.
    d = np.abs(ref.astype(np.int32) - rec.astype(np.int32))
    bad = int(np.count_nonzero(d.max(axis=1)))
    worst = int(d.max())
    print(
        f"      lead-in: {bad} of {n} frames differ in the first {secs:.1f} s, "
        f"by at most {worst} LSB ({20 * np.log10(max(worst, 1) / 32768.0):+.0f} dBFS)"
    )
    # A handful of near-silent frames at a stream's edge is the recorder finding
    # the signal, not a fault. Anything louder is real.
    return worst <= 32, 0


def compare(ref_path, rec_path):
    ref, ref_rate = read_wav(ref_path)
    rec, rec_rate = read_wav(rec_path)
    if ref_rate != rec_rate:
        print(f"FAIL  sample rates differ: reference {ref_rate}, recording {rec_rate}")
        print("\n" + PRECONDITIONS)
        return 1

    # Anchor a little way in. The first moments are the least representative
    # part of any recording: the recorder's own lead-in silence, the stream
    # opening, the buffer priming. Aligning on them can leave the comparison
    # starting inside a region that exists in one file and not the other, which
    # no shift can reconcile.
    anchor = min(2 * ref_rate, len(ref) // 4)
    off = find_offset(ref[anchor:], rec)
    if off is None:
        print("FAIL  could not align the two files; are they the same material?")
        return 1
    off -= anchor
    # Confirm the alignment actually means something before trusting anything
    # built on it. Correlation always returns a peak; on music it will happily
    # match a musically similar passage. Without this check a recording of
    # entirely different material was reported as one unresolved splice, which
    # reads like a firmware fault instead of "wrong file".
    #
    # The offset is measured once, near the start, but it does not hold for the
    # whole file: the pacer inserts and drops frames to absorb the difference
    # between the player's clock and the recorder's, so the lag creeps. Over
    # eleven minutes it crept by about fifty frames. Requiring the single
    # starting offset to line up a third of the way in therefore compares music
    # against itself shifted by a fraction of a millisecond, which correlates at
    # about 0.14 and reads exactly like the wrong file — it rejected a recording
    # that turned out to correlate at 1.000 once the drift was allowed for.
    #
    # So search a window around the expected position and take the best match.
    # Wrong material still correlates with nothing at any lag, which is the
    # distinction this check exists to make.
    probe_n = min(48000, len(ref) // 4)
    slack = min(2 * ref_rate, probe_n)
    i = len(ref) // 3
    a = ref[i : i + probe_n, 0].astype(np.float64)
    lo = max(0, i + off - slack)
    hay = rec[lo : i + off + probe_n + slack, 0].astype(np.float64)
    if len(hay) > len(a) and a.size and a.any():
        a = a - a.mean()
        corr = _xcorr_valid(hay, a)
        energy = np.concatenate(([0.0], np.cumsum(hay * hay)))
        window = energy[len(a) :] - energy[: -len(a)]
        # Subtracting the window mean is what makes this a correlation
        # coefficient rather than a dot product biased by any DC offset.
        counts = float(len(a))
        sums = np.concatenate(([0.0], np.cumsum(hay)))
        wsum = sums[len(a) :] - sums[: -len(a)]
        var = np.maximum(window - wsum * wsum / counts, 0.0)
        den = np.sqrt(var * (a * a).sum())
        den[den <= 0] = np.inf
        ncc = float(np.abs(corr / den).max())
        if ncc < 0.9:
            print(f"FAIL  these are not the same audio (correlation {ncc:+.3f})")
            print("      The recording does not contain the reference material.")
            print("      Check that the player was playing the reference file, and")
            print("      that the capture covers it.")
            return 1
    print(f"aligned at frame {off} of the recording ({off / ref_rate:.3f} s)")

    if off < 0:
        ref = ref[-off:]
    else:
        rec = rec[off:]
    # The anchor is where alignment was MEASURED, not where comparison starts.
    # Skipping it entirely hid the opening seconds of every stream — priming,
    # the first packets, the moment a glitch is most likely — from every report:
    # a capture whose first two seconds were visibly wrong came back "bit-exact"
    # because the only region anyone doubted was the one never looked at.
    #
    # So hold the lead-in aside and compare it separately. It stays out of the
    # main walk because a stream's first moments are exactly where one file
    # holds material the other does not, which no shift reconciles; a failure
    # there should be reported, not allowed to derail the rest.
    lead_n0 = min(anchor, len(ref), len(rec))
    lead_ok, lead_net = report_lead_in(
        ref[:lead_n0], rec[:lead_n0], 1, ref_rate
    )
    # Carry what the lead-in found into the main comparison. Without this the
    # walk restarts at the anchor still using the original offset, is out by
    # however many frames the lead-in gained or lost, and rediscovers the same
    # edit as a second one — a single insertion at startup got reported twice,
    # once at frame 1 and again at the anchor.
    lead_ref, lead_rec = ref[:anchor], rec[:anchor]
    ref = ref[anchor:]
    rec = rec[anchor + lead_net :] if anchor + lead_net >= 0 else rec[anchor:]
    n = min(len(ref), len(rec))
    if n == 0:
        print("FAIL  no overlap after alignment")
        return 1

    lead_n = min(len(lead_ref), len(lead_rec))

    if lead_ok and np.array_equal(ref[:n], rec[:n]):
        total = n + lead_n
        print(f"compared {total} frames ({total / ref_rate:.1f} s), lead-in included")
        print("PASS  bit-exact: every sample matches")
        return 0

    # Not identical. Measure the level error first, and use it as the tolerance
    # for finding splices: a hardcoded tolerance either misses them (too tight,
    # every frame looks like an edit) or invents matches (too loose).
    probe = min(len(ref), len(rec), 20 * ref_rate)
    d = np.abs(ref[:probe].astype(np.int32) - rec[:probe].astype(np.int32))
    tol = int(np.percentile(d, 99.9)) if probe else 0
    if tol > 8:
        # Not a level error at all; fall through to the bulk explanation.
        tol = 0
    stats, edits = walk(ref, rec, 0, tol=max(tol, 1))
    lost = sum(e[2] for e in edits if e[1] == "missing")
    gained = sum(e[2] for e in edits if e[1] == "extra")
    corrupt = sum(e[2] for e in edits if e[1] == "corrupt")
    print(f"compared {stats['compared'] // CHANNELS} frames between edits")
    if stats["worst_lsb"]:
        pct = 100.0 * stats["diff_samples"] / max(stats["compared"], 1)
        print(
            f"      level: {pct:.1f}% of samples differ by at most "
            f"{stats['worst_lsb']} LSB"
        )
        # A multiply and a rounding error look alike at this scale, so say which:
        # a volume control shows up as a consistent scale factor, a truncating
        # conversion does not.
        a = ref[:probe].astype(np.float64).ravel()
        b = rec[:probe].astype(np.float64).ravel()
        den = float((a * a).sum())
        if den:
            k = float((a * b).sum()) / den
            db = 20.0 * np.log10(k) if k > 0 else float("nan")
            print(f"      best-fit scale {k:.8f} ({db:+.5f} dB)")
        print("      A truncating float round-trip does this — an audio stack that")
        print("      converts int16 -> float -> int16 without rounding. It loses no")
        print("      audio, but it is not bit-exact, so fix it before trusting a pass.")
    if edits:
        print(
            f"      {len(edits)} edit(s): {lost} frames missing, {gained} extra, "
            f"{corrupt} corrupted in place"
        )
        for pos, kind, cnt in edits[:12]:
            # The walk runs on the anchored slice; report where it is in the file.
            at = pos + anchor
            print(
                f"        ref frame {at:>9}  {at / ref_rate / 60:5.2f} min  "
                f"{kind} {cnt} frames ({1000.0 * cnt / ref_rate:.1f} ms)"
            )
        if len(edits) > 12:
            print(f"        ... and {len(edits) - 12} more")
        print("      Between splices the streams stay aligned exactly, so nothing is")
        print("      drifting: each is a discrete discard, not a clock mismatch.")
        return 1
    if stats["worst_lsb"]:
        return 1

    notes = classify(ref[:n], rec[:n])
    total = int(np.count_nonzero(np.any(ref[:n] != rec[:n], axis=1)))
    print(f"compared {n} frames ({n / ref_rate:.1f} s)")

    if not notes:
        if lead_ok:
            print("PASS  bit-exact: every sample matches")
            return 0
        # Saying PASS while having just printed edits is how the anchor hid them
        # in the first place.
        print("FAIL  bit-exact after the lead-in, but the lead-in has edits above")
        return 1

    pct = 100.0 * total / n
    print(f"FAIL  {total} frames differ ({pct:.4f}%)")
    if pct > 50:
        print("      Nearly everything differs, so this is the chain rather than")
        print("      the bridge. Specifically:")
        why = explain_bulk(ref[:n], rec[:n])
        if why:
            print(f"      {why}")
        print()
        print(PRECONDITIONS)
        return 1
    for note in notes:
        print(
            f"      frame {note['frame']:>9}  {note['kind']:<28}"
            f" ref={note['ref']} rec={note['rec']}"
        )
    if total > len(notes):
        print(f"      ... and {total - len(notes)} more")
    return 1


# --- self-test -------------------------------------------------------------
# The tool has to be trusted before a result from it means anything, so it is
# checked against faults it is meant to name, synthesised deliberately.


def _self_test():
    rng = np.random.default_rng(1)
    ref = (rng.integers(-20000, 20000, size=(48000, 2))).astype(np.int16)
    ok = True

    def check(name, rec, expect_clean, expect_kind=None, offset=0):
        nonlocal ok
        off = find_offset(ref, rec)
        if off is None:
            print(f"  FAIL {name}: alignment failed")
            ok = False
            return
        if off != offset:
            print(f"  FAIL {name}: offset {off}, expected {offset}")
            ok = False
            return
        aligned = rec[off:] if off > 0 else rec
        n = min(len(ref), len(aligned))
        notes = classify(ref[:n], aligned[:n])
        if expect_clean:
            if notes:
                print(f"  FAIL {name}: expected bit-exact, got {notes[0]}")
                ok = False
            else:
                print(f"  ok   {name}")
            return
        if not notes:
            print(f"  FAIL {name}: expected a difference, found none")
            ok = False
        elif expect_kind and notes[0]["kind"] != expect_kind:
            print(f"  FAIL {name}: got '{notes[0]['kind']}', expected '{expect_kind}'")
            ok = False
        else:
            print(f"  ok   {name} -> {notes[0]['kind']} at frame {notes[0]['frame']}")

    check("identical", ref.copy(), True)

    lead = np.zeros((1234, 2), dtype=np.int16)
    check("offset recording", np.vstack([lead, ref]), True, offset=1234)

    dup = ref.copy()
    dup[20000] = dup[19999]
    check("repeated frame", dup, False, "repeated frame (slip down)")

    drop = np.vstack([ref[:20000], ref[20001:], ref[-1:]])
    check("dropped frame", drop, False, "dropped frame (slip up)")

    gap = ref.copy()
    gap[30000:30100] = 0
    check("dropout", gap, False, "silence")

    swap = ref.copy()
    swap[25000] = swap[25000][::-1]
    check("channel swap", swap, False, "channels swapped")

    # A one-LSB level error must not be reported as lost audio, and a splice
    # must still be found underneath one.
    lsb = np.clip(ref.astype(np.int32) - (ref > 0), -32768, 32767).astype(np.int16)
    st, ed = walk(ref, lsb, 0)
    if st["worst_lsb"] == 1 and not ed:
        print("  ok   one-LSB level error -> named as level, no splices invented")
    else:
        print(f"  FAIL one-LSB level error: worst={st['worst_lsb']} edits={ed}")
        ok = False

    spliced = np.vstack([lsb[:20000], lsb[20384:]])
    st, ed = walk(ref, spliced, 0)
    if len(ed) == 1 and ed[0][1] == "missing" and ed[0][2] == 384:
        print("  ok   384-frame splice under a level error -> found exactly")
    else:
        print(f"  FAIL splice under level error: {ed[:3]}")
        ok = False

    # Bulk explanations: qualifying the player matters as much as the firmware.
    quiet = (ref.astype(np.float64) * 0.98).astype(np.int16)
    why = explain_bulk(ref, quiet)
    if why and "constant gain" in why:
        print("  ok   volume multiply -> named as a gain")
    else:
        print(f"  FAIL volume multiply: {why}")
        ok = False

    resampled = np.roll(ref, 1, axis=0) // 2 + ref // 2
    why = explain_bulk(ref, resampled)
    if why and "no constant gain" in why:
        print("  ok   resample-like -> named as not a level change")
    else:
        print(f"  FAIL resample-like: {why}")
        ok = False

    ok = _check_lead_in() and ok
    ok = _check_repeated_material() and ok

    print("\nself-test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def _check_repeated_material():
    """When the material appears twice, align on the FIRST copy.

    A recording left running past the end holds the reference twice. Both copies
    correlate at 1.000, so the winner is decided by which has fewer edits inside
    the one-second probe — and the later copy, being shorter and quieter, often
    wins. A fourteen-minute capture aligned on its own final two minutes and
    reported them as the whole result.
    """
    rng = np.random.default_rng(11)
    rate = 48000
    ref = rng.integers(-20000, 20000, size=(4 * rate, 2)).astype(np.int16)
    flawed = ref.copy()
    # A blemish inside the probe window, enough to lose a tie but not a match.
    flawed[1000:1100] = 0
    gap = np.zeros((rate, CHANNELS), dtype=np.int16)
    rec = np.concatenate([flawed, gap, ref])

    off = find_offset(ref, rec)
    if off is not None and abs(off) < 100:
        print("  ok   material recorded twice -> aligns on the first copy")
        return True
    print(f"  FAIL material recorded twice: aligned at {off}, expected ~0")
    return False


def _check_lead_in():
    """A fault inside the alignment anchor must be reported, not skipped.

    This is the regression the rest of the suite could not catch, because every
    other case drives find_offset and classify directly and never exercises what
    compare() chooses to look at. A recording whose opening was visibly wrong
    came back "bit-exact" while the anchor quietly excluded the only region in
    doubt, so the test has to go through compare() end to end.
    """
    import contextlib
    import io
    import os
    import tempfile
    import wave as wavemod

    rng = np.random.default_rng(7)
    rate = 48000
    ref = rng.integers(-20000, 20000, size=(10 * rate, 2)).astype(np.int16)
    cut = rate // 2  # half a second in: well inside the two-second anchor
    spliced = np.concatenate([ref[:cut], ref[cut + 64 :]])

    def run(rec):
        d = tempfile.mkdtemp()
        a, b = os.path.join(d, "ref.wav"), os.path.join(d, "rec.wav")
        for path, data in ((a, ref), (b, rec)):
            with wavemod.open(path, "wb") as w:
                w.setnchannels(CHANNELS)
                w.setsampwidth(2)
                w.setframerate(rate)
                w.writeframes(data.astype("<i2").tobytes())
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            compare(a, b)
        os.remove(a)
        os.remove(b)
        os.rmdir(d)
        return buf.getvalue()

    good = True

    out = run(spliced)
    if "lead-in" in out and "64 frames" in out:
        print("  ok   64-frame splice inside the anchor -> reported in the lead-in")
    else:
        good = False
        print("  FAIL splice inside the anchor was not reported")
        print("\n".join("        " + l for l in out.splitlines()))

    # A fault inside the lead-in must not leak into the main walk. It used to:
    # the walk restarted at the anchor with the original offset, was out by
    # however many frames the lead-in had gained, and rediscovered the same
    # insertion as a second edit of its own.
    at = 1000
    inserted = np.concatenate([ref[:at], np.repeat(ref[at - 1 : at], 8, axis=0), ref[at:]])
    out = run(inserted)
    main = [l for l in out.splitlines() if "edit(s):" in l]
    leaked = main and not main[0].strip().startswith("0 edit(s)")
    if "lead-in:" in out and not leaked:
        print("  ok   a fault inside the lead-in does not leak into the main walk")
    else:
        good = False
        print("  FAIL lead-in fault leaked into the main walk")
        print("\n".join("        " + l for l in out.splitlines()))

    out = run(ref.copy())
    if "PASS" in out and "lead-in included" in out:
        print("  ok   clean recording -> passes with the lead-in counted")
    else:
        good = False
        print("  FAIL clean recording did not report the lead-in as compared")
        print("\n".join("        " + l for l in out.splitlines()))

    return good


def main(argv):
    if len(argv) == 2 and argv[1] == "--self-test":
        return _self_test()
    if len(argv) != 3:
        print(__doc__)
        return 2
    return compare(argv[1], argv[2])


if __name__ == "__main__":
    sys.exit(main(sys.argv))

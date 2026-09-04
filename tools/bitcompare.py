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


def read_wav(path):
    """Return (frames, rate) as an int16 array of shape (n, 2)."""
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
    corr = np.correlate(hay, needle, mode="valid")
    peak = int(np.argmax(np.abs(corr)))
    return peak - start


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


def compare(ref_path, rec_path):
    ref, ref_rate = read_wav(ref_path)
    rec, rec_rate = read_wav(rec_path)
    if ref_rate != rec_rate:
        print(f"FAIL  sample rates differ: reference {ref_rate}, recording {rec_rate}")
        print("\n" + PRECONDITIONS)
        return 1

    off = find_offset(ref, rec)
    if off is None:
        print("FAIL  could not align the two files; are they the same material?")
        return 1
    print(f"aligned at frame {off} of the recording ({off / ref_rate:.3f} s)")

    if off < 0:
        ref = ref[-off:]
    else:
        rec = rec[off:]
    n = min(len(ref), len(rec))
    if n == 0:
        print("FAIL  no overlap after alignment")
        return 1

    notes = classify(ref[:n], rec[:n])
    total = int(np.count_nonzero(np.any(ref[:n] != rec[:n], axis=1)))
    print(f"compared {n} frames ({n / ref_rate:.1f} s)")

    if not notes:
        print("PASS  bit-exact: every sample matches")
        return 0

    pct = 100.0 * total / n
    print(f"FAIL  {total} frames differ ({pct:.4f}%)")
    if pct > 50:
        print("      Nearly everything differs, which is a chain problem rather")
        print("      than a firmware fault — a resample, a gain stage or a")
        print("      different master.\n")
        print(PRECONDITIONS)
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

    print("\nself-test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main(argv):
    if len(argv) == 2 and argv[1] == "--self-test":
        return _self_test()
    if len(argv) != 3:
        print(__doc__)
        return 2
    return compare(argv[1], argv[2])


if __name__ == "__main__":
    sys.exit(main(sys.argv))

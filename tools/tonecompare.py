"""Check a recording of the `packet-stress` tone against what the firmware emits.

The car board generates that tone itself, from a phase accumulator with no
input, so the expected samples can be regenerated here exactly rather than
compared against a file that was played. That is the point of the build: it
takes the phone, the source board and the I2S link out of the chain, so
anything wrong in the result belongs to the car board's USB path alone.

Alignment is by exact match, not correlation. A 997 Hz tone nearly repeats every
48.14 samples, so its autocorrelation has hundreds of near-equal peaks and the
usual aligner cannot tell them apart. An exact sixteen-frame match can.
"""
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bitcompare as bc

CAR_RS = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "rp2040", "src", "bin", "car.rs"
)


def _firmware_tone():
    """Read the sine table and phase increment out of the firmware source.

    Parsed rather than copied so the two cannot drift apart. A transcribed table
    that silently went stale would not fail: it would report a working board as
    subtly wrong, which is the least useful way for a checker to break.
    """
    src = open(CAR_RS).read()
    m = re.search(r"const SINE256: \[i16; 256\] = \[(.*?)\];", src, re.S)
    if not m:
        raise SystemExit(f"could not find SINE256 in {CAR_RS}")
    table = [int(x) for x in re.findall(r"-?\d+", m.group(1))]
    if len(table) != 256:
        raise SystemExit(f"SINE256 has {len(table)} entries, expected 256")
    m = re.search(
        r"const TONE_PHASE_INC: u32 = \(\(([\d_]+)u64 << 32\) / ([\d_]+)u64\)", src
    )
    if not m:
        raise SystemExit(f"could not find TONE_PHASE_INC in {CAR_RS}")
    hz = int(m.group(1).replace("_", ""))
    rate = int(m.group(2).replace("_", ""))
    return table, (hz << 32) // rate


SINE256, TONE_PHASE_INC = _firmware_tone()


def generate(n, phase0=0, first=0):
    """The firmware's tone, sample for sample.

    `phase0` carries the sub-quantum part of the board's accumulator. The phase
    is a 32-bit value advanced once per frame, so a recording that starts partway
    through cannot be expressed as an exact whole number of increments from zero;
    without this the reconstruction lands the wrong side of an interpolation
    rounding boundary on about 3% of samples and reports a bit-exact stream as
    differing by one or two LSB.
    """
    tbl = np.array(SINE256, dtype=np.int64)
    idx_ = np.arange(first, first + n, dtype=np.int64)
    phase = ((idx_ * TONE_PHASE_INC) + phase0) & 0xFFFFFFFF
    idx = (phase >> 24) & 0xFF
    frac = (phase >> 16) & 0xFF
    a = tbl[idx]
    b = tbl[(idx + 1) & 0xFF]
    # Rust's >> on i32 floors, and so does Python's on ints.
    v = (a + ((b - a) * frac >> 8)).astype(np.int16)
    return np.stack([v, v], axis=1)


def find_start(expected, rec, width=16):
    """Index into `expected` where `rec` begins, by exact window match."""
    keys = {}
    flat = expected[:, 0]
    for i in range(len(flat) - width):
        keys.setdefault(flat[i : i + width].tobytes(), i)
    # Try several places in the recording: the first may sit in a glitch.
    for probe in range(0, min(len(rec) - width, 400000), 997):
        hit = keys.get(rec[probe : probe + width, 0].tobytes())
        if hit is not None and hit >= probe:
            return hit - probe
    return None


def main(path):
    rec, rate = bc.read_wav(path)
    expected = generate(len(rec) + 4 * rate)
    off = find_start(expected, rec)
    if off is None:
        print("FAIL  could not locate the tone; is this the packet-stress build?")
        return 1
    # Recover the sub-quantum phase, then the reconstruction is exact.
    probe = min(len(rec), 200000)
    target = rec[:probe, 0].astype(np.int64)
    best, best_score = 0, -1
    for step in (256, 1):
        lo = max(0, best - 256) if step == 1 else 0
        hi = best + 257 if step == 1 else 65536
        for c in range(lo, hi, step):
            v = generate(probe, phase0=c, first=off)[:, 0].astype(np.int64)
            sc = int(np.count_nonzero(v == target))
            if sc > best_score:
                best, best_score = c, sc
    print(f"tone starts {off} frames ({off / rate:.2f} s) in, phase offset {best} "
          f"({100.0 * best_score / probe:.2f}% exact over the probe)")
    ref = generate(len(rec), phase0=best, first=off)
    n = min(len(ref), len(rec))
    if np.array_equal(ref[:n], rec[:n]):
        print(f"compared {n} frames ({n / rate:.1f} s)")
        print("PASS  every sample matches what the firmware generated")
        return 0
    stats, edits = bc.walk(ref[:n], rec[:n], 0, tol=1)
    print(f"compared {stats['compared'] // 2} frames between edits")
    held = sum(c for _, k, c in edits if k == "extra")
    print(f"{len(edits)} edit(s): "
          f"{sum(c for _, k, c in edits if k == 'missing')} missing, "
          f"{held} extra, "
          f"{sum(c for _, k, c in edits if k == 'corrupt')} corrupted")
    for pos, kind, cnt in edits[:20]:
        seg = rec[pos : pos + cnt]
        hold = "hold" if cnt and len(set(map(tuple, seg))) == 1 else ""
        print(f"   frame {pos:>9}  {pos / rate / 60:6.3f} min  {kind} {cnt} {hold}")
    if len(edits) > 20:
        print(f"   ... and {len(edits) - 20} more")
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))

#!/usr/bin/env python3
"""Analyse a recording of the `ramp` diagnostic build.

The firmware built with `--features ramp` fills every audio sample slot with a
16-bit counter that increments by exactly 1 per sample, identical on every
channel.  Record that stream bit-exactly on a host (16-bit PCM, no sample-rate
conversion, no processing) and run this script on the WAV.

In a perfect capture, consecutive samples differ by exactly 1.  Anything else
is a transport fault, and *where* it happens says what went wrong:

  delta 0   a sample was repeated      (host re-read a stale buffer)
  delta N   N-1 samples went missing   (a packet was dropped or truncated)
  spacing   gaps every 441 samples at 44.1 kHz means the 45-sample frame is
            the culprit; irregular spacing points at SOF timing instead.

Usage:  python3 tools/check_ramp.py recording.wav [--rate 44100]
"""
import argparse
import struct
import sys
import wave
from collections import Counter


def read_channel0(path):
    with wave.open(path, "rb") as w:
        nch, width, rate, nframes = (
            w.getnchannels(),
            w.getsampwidth(),
            w.getframerate(),
            w.getnframes(),
        )
        raw = w.readframes(nframes)
    if width != 2:
        sys.exit(
            f"{path}: {width * 8}-bit samples. Re-record as 16-bit PCM — "
            "anything else has been converted and won't be bit-exact."
        )
    vals = struct.unpack("<%dh" % (len(raw) // 2), raw)
    # Counter was written as u16-LE; the host reads it back as signed i16.
    return [v & 0xFFFF for v in vals[::nch]], rate, nch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wav")
    ap.add_argument("--rate", type=int, default=None, help="expected sample rate")
    ap.add_argument("--show", type=int, default=20, help="max anomalies to list")
    args = ap.parse_args()

    samples, rate, nch = read_channel0(args.wav)
    if not samples:
        sys.exit("empty recording")
    expected_rate = args.rate or rate
    print(f"{args.wav}: {len(samples)} frames, {nch}ch, {rate} Hz")

    # Per-frame sample counts the firmware should be emitting.
    lo = expected_rate // 1000
    hi = -(-expected_rate // 1000)  # ceil
    if lo == hi:
        print(f"format: {lo} samples/frame (integer — packet size never varies)")
    else:
        period = 1000 // (expected_rate % 1000)  # frames between odd-sized frames
        apart = expected_rate * period // 1000  # audio samples between them
        print(
            f"format: alternating {lo}/{hi} samples/frame; the {hi}-sample frame "
            f"lands every {period} frames ({apart} samples apart)"
        )

    bad, deltas = [], Counter()
    for i in range(1, len(samples)):
        d = (samples[i] - samples[i - 1]) & 0xFFFF
        deltas[d] += 1
        if d != 1:
            bad.append((i, samples[i - 1], samples[i], d))

    print(f"\nanomalies: {len(bad)} of {len(samples) - 1} transitions")
    if not bad:
        print("PERFECT RAMP — the iso transport is clean at this rate.")
        return

    print("\ndelta histogram (delta: count) — 1 is correct:")
    for d, n in sorted(deltas.items(), key=lambda kv: -kv[1])[:10]:
        note = "  <- correct" if d == 1 else ("  <- repeated sample" if d == 0 else f"  <- {d - 1} sample(s) lost")
        print(f"  {d:6d}: {n}{note}")

    print(f"\nfirst {min(args.show, len(bad))} anomalies:")
    for i, prev, cur, d in bad[:args.show]:
        print(f"  sample {i:9d}: {prev:5d} -> {cur:5d}  (delta {d})")

    if len(bad) > 1:
        gaps = [bad[i][0] - bad[i - 1][0] for i in range(1, len(bad))]
        g = Counter(gaps).most_common(5)
        print("\nspacing between anomalies (samples apart: count):")
        for spacing, n in g:
            ms = spacing / expected_rate * 1000
            print(f"  {spacing:6d}: {n}   ({ms:.2f} ms apart)")
        print(
            "\nIf the dominant spacing matches the odd-frame period printed "
            "above, the variable packet size is the cause."
        )


if __name__ == "__main__":
    main()

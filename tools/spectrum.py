#!/usr/bin/env python3
"""Find and identify spectral artifacts in a recording of a test tone.

Reports the fundamental, then every other peak with its level relative to the
fundamental and — the useful part — an interpretation of *where it came from*:

  harmonic of the tone      -> distortion in the generator or the analogue path
  multiple of 1000 Hz       -> a per-USB-frame artifact (packet boundary)
  tone +/- N * 1000 Hz      -> the tone modulated at the frame rate, i.e. a
                               periodic per-packet disturbance
  multiple of the odd-frame rate -> variable-packet-size correction artifacts

Usage:  python3 tools/spectrum.py recording.wav [--tone 997]
"""
import argparse
import cmath
import math
import struct
import sys
import wave

FRAME_HZ = 1000.0  # USB full-speed frame rate


def read_mono(path):
    with wave.open(path, "rb") as w:
        nch, width, sr, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    if width != 2:
        sys.exit(f"{path}: expected 16-bit PCM, got {width * 8}-bit")
    v = struct.unpack("<%dh" % (len(raw) // 2), raw)
    return [float(x) for x in v[::nch]], sr


def fft(x):
    """Iterative radix-2 FFT; input length must be a power of two."""
    n = len(x)
    j = 0
    x = list(x)
    for i in range(1, n):
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j |= bit
        if i < j:
            x[i], x[j] = x[j], x[i]
    length = 2
    while length <= n:
        ang = -2 * math.pi / length
        wl = cmath.exp(1j * ang)
        for i in range(0, n, length):
            w = 1 + 0j
            for k in range(i, i + length // 2):
                u = x[k]
                v = x[k + length // 2] * w
                x[k] = u + v
                x[k + length // 2] = u - v
                w *= wl
        length <<= 1
    return x


def classify(f, tone, sr):
    """Say where a peak most plausibly came from."""
    out = []
    for h in range(2, 12):
        if abs(f - tone * h) < 3:
            out.append(f"harmonic {h} of the tone (generator/path distortion)")
    if f > 1 and abs(f - round(f / FRAME_HZ) * FRAME_HZ) < 3 and round(f / FRAME_HZ) >= 1:
        out.append(f"{round(f / FRAME_HZ)}x the 1 kHz USB frame rate (per-packet artifact)")
    for k in range(1, 8):
        for sign, label in ((1, "+"), (-1, "-")):
            if abs(f - (tone + sign * k * FRAME_HZ)) < 3:
                out.append(f"tone {label} {k}x frame rate (tone modulated per packet)")
    for period, what in ((10.0, "44.1 kHz odd-frame rate"), (2.0, "gentle stress rate")):
        if abs(f - period) < 0.5:
            out.append(what)
    return "; ".join(out) if out else "unattributed"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wav")
    ap.add_argument("--tone", type=float, default=997.0, help="expected fundamental Hz")
    ap.add_argument("--peaks", type=int, default=12)
    ap.add_argument("--floor", type=float, default=-100.0, help="ignore peaks below this dBc")
    args = ap.parse_args()

    x, sr = read_mono(args.wav)
    n = 1 << (len(x).bit_length() - 1)
    n = min(n, 1 << 18)
    # Skip the first 0.5 s: alt-setting changes and stream startup live there.
    start = min(int(0.5 * sr), max(0, len(x) - n))
    x = x[start:start + n]
    if len(x) < n:
        sys.exit("recording too short")

    # Hann window, so a strong fundamental doesn't smear over everything else.
    win = [0.5 - 0.5 * math.cos(2 * math.pi * i / n) for i in range(n)]
    spec = fft([complex(a * b, 0) for a, b in zip(x, win)])
    mag = [abs(spec[k]) for k in range(n // 2)]

    # Local maxima only, so one peak isn't reported as five.
    peaks = []
    for k in range(2, n // 2 - 2):
        if mag[k] > mag[k - 1] and mag[k] >= mag[k + 1]:
            peaks.append((mag[k], k * sr / n))
    peaks.sort(reverse=True)
    if not peaks:
        sys.exit("no peaks found")

    fund_mag, fund_f = peaks[0]
    print(f"{args.wav}: {len(x)} samples @ {sr} Hz, {n}-point FFT "
          f"({sr / n:.2f} Hz/bin)")
    print(f"fundamental: {fund_f:.1f} Hz\n")
    print(f"{'freq (Hz)':>10}  {'dBc':>7}   origin")
    shown = 0
    for m, f in peaks[1:]:
        if abs(f - fund_f) < 6:
            continue
        db = 20 * math.log10(m / fund_mag)
        if db < args.floor:
            break
        print(f"{f:10.1f}  {db:7.1f}   {classify(f, fund_f, sr)}")
        shown += 1
        if shown >= args.peaks:
            break
    if shown == 0:
        print("  (nothing above the floor — the tone is clean)")


if __name__ == "__main__":
    main()

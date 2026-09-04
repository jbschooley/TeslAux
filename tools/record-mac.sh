#!/bin/sh
# Record the TeslaMic as float32, bypassing Gig Performer entirely.
#
# float32 is what CoreAudio hands applications, and a 16-bit sample converts to
# float32 exactly — so this captures whatever reached the Mac without adding a
# conversion of its own. bitcompare.py then reports whether the samples came
# back as whole numbers, which is the test for whether anything touched them.
set -e
OUT="${1:-$HOME/Documents/teslaux-capture.wav}"
SECS="${2:-70}"
echo "recording $SECS s from TeslaMic -> $OUT"
ffmpeg -hide_banner -loglevel warning \
  -f avfoundation -i ":6" \
  -t "$SECS" -ac 2 -ar 48000 -c:a pcm_f32le -y "$OUT"
echo "done: $OUT"

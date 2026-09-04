#!/bin/sh
# Record the TeslaMic as float32, bypassing Gig Performer entirely.
#
# float32 is what CoreAudio hands applications, and a 16-bit sample converts to
# float32 exactly — so this captures whatever reached the Mac without adding a
# conversion of its own. bitcompare.py then reports whether the samples came
# back as whole numbers, which is the test for whether anything touched them.
#
# WARNING: measured to drop about 10% of frames. A 5 s capture returned 4.499 s
# and a 70 s capture returned 63.8 s, sustained rather than a startup offset.
# Losses scattered through the stream destroy sample alignment completely, so a
# capture made this way cannot be compared against a reference even though its
# spectrum matches. Kept only as a record of a path that does not work.
#
# Record 32-bit float in the DAW instead: that removes the int16 conversion,
# which is the thing this script existed to rule out, and the DAW's capture is
# reliable.
set -e
echo "WARNING: this path drops ~10% of frames; see the comment above." >&2
OUT="${1:-$HOME/Documents/teslaux-capture.wav}"
SECS="${2:-70}"
echo "recording $SECS s from TeslaMic -> $OUT"
ffmpeg -hide_banner -loglevel warning \
  -f avfoundation -i ":6" \
  -t "$SECS" -ac 2 -ar 48000 -c:a pcm_f32le -y "$OUT"
echo "done: $OUT"

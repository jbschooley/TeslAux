#!/bin/sh
# Build every RP2040 image and package it as a drag-and-drop UF2.
# RP2040 UF2 family id 0xe48bff56, flash base 0x10000000.
set -e
cd "$(dirname "$0")"
UF2=../tools/uf2conv.py
OBJ=${OBJCOPY:-arm-none-eabi-objcopy}
# BOARD=rp2040-zero (default, what we have) or BOARD= for a Pico / RP2040-Plus.
# The Zero has no plain LED, so status goes to its WS2812 on GPIO16 instead.
BOARD=${BOARD-rp2040-zero}
build() { # <bin> <features> <out>
  FEATS=$(echo "$BOARD${2:+,$2}" | sed 's/^,//')
  cargo build --release --bin "$1" ${FEATS:+--features "$FEATS"}
  $OBJ -O binary "target/thumbv6m-none-eabi/release/$1" "/tmp/$3.bin"
  python3 $UF2 "/tmp/$3.bin" -c -b 0x10000000 -f 0xe48bff56 -o "$3.uf2"
}
# --- recommended pairing: 2x RP2040, adaptive source + elastic car ---
build car    ""              teslamic-rp-car-elastic
build car    low-latency     teslamic-rp-car-elastic-lowlat
build source ""              teslamic-rp-source-adaptive
build source low-latency     teslamic-rp-source-lowlat
build source clock-steered   teslamic-rp-source-steered
build source clock-steered,low-latency teslamic-rp-source-steered-lowlat
build source ultra-low       teslamic-rp-source-ultralow
# --- alternative pairing: clock-locked chain (fixed 192-B packets to the car) ---
# source clock-locked is deliberately not built — see the compile_error in
# src/bin/source.rs. It would make both boards drive the I2S clock lines.
# --- PCM2706 variant: same car binary as elastic, kept under its old name ---
build car    ""              teslamic-rp-car-pcm2706
build car    clock-locked    teslamic-rp-car-locked
build car    packet-stress   teslamic-rp-car-STRESS-TEST
# Same cushion and block size as the shipping car image, so it is the shipping
# pipe under test — only the sample VALUES are substituted.
build car    low-latency,pipe-tone teslamic-rp-car-PIPETONE
# Both together: known data AND the measurement, so one run answers both what
# the pipe did and how it got there.
build car    low-latency,pipe-tone,pipe-watch teslamic-rp-car-PIPEWATCH

# --- diagnostics ---
# These were built by hand and went stale: when the I2S pins moved they kept the
# old ones, so I2SSNIFF would have reported "no clock" on a working link. Build
# them with everything else so that cannot happen again.
#
# PANTEST and MEASURE take clock-steered because the shielded pinout only exists
# on that path; without it they fall back to upstream's driver and the original
# three wires. I2STEST is deliberately left on the old pinout — see the note in
# its source — so it needs rejumpering to GP2/3/4 at both ends to be useful.
build ledtest  ""                          teslamic-rp-LEDTEST
build i2ssniff ""                          teslamic-rp-I2SSNIFF
build i2srx    ""                          teslamic-rp-I2SRX
build i2stest  ""                          teslamic-rp-I2STEST
build source   pan-test,clock-steered      teslamic-rp-source-PANTEST
build source   measure-excursion,clock-steered teslamic-rp-source-MEASURE

ls -l ./*.uf2

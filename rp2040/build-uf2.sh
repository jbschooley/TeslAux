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
ls -l ./*.uf2

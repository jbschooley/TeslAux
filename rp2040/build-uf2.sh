#!/bin/sh
# Build every RP2040 image and package it as a drag-and-drop UF2.
# RP2040 UF2 family id 0xe48bff56, flash base 0x10000000.
set -e
cd "$(dirname "$0")"
UF2=../tools/uf2conv.py
OBJ=${OBJCOPY:-arm-none-eabi-objcopy}
build() { # <bin> <features> <out>
  cargo build --release --bin "$1" ${2:+--features "$2"}
  $OBJ -O binary "target/thumbv6m-none-eabi/release/$1" "/tmp/$3.bin"
  python3 $UF2 "/tmp/$3.bin" -c -b 0x10000000 -f 0xe48bff56 -o "$3.uf2"
}
# --- recommended pairing: 2x RP2040, adaptive source + elastic car ---
build car    ""              teslamic-rp-car-elastic
build source ""              teslamic-rp-source-adaptive
# --- alternative pairing: clock-locked chain (fixed 192-B packets to the car) ---
build source clock-locked    teslamic-rp-source-locked
# --- PCM2706 variant: same car binary as elastic, kept under its old name ---
build car    ""              teslamic-rp-car-pcm2706
build car    clock-locked    teslamic-rp-car-locked
build car    packet-stress   teslamic-rp-car-STRESS-TEST
ls -l ./*.uf2

#!/bin/sh
# Build every RP2350 image and package it as a drag-and-drop UF2.
# RP2350 ARM secure-image UF2 family id 0xe48bff59 (the RP2040's is ...56 —
# using the wrong one gives a board that accepts the file and does nothing).
# Flash base 0x10000000.
set -e
cd "$(dirname "$0")"
UF2=../tools/uf2conv.py
OBJ=${OBJCOPY:-arm-none-eabi-objcopy}
build() { # <bin> <features> <out>
  cargo build --release --bin "$1" ${2:+--features "$2"}
  $OBJ -O binary "target/thumbv8m.main-none-eabihf/release/$1" "/tmp/$3.bin"
  python3 $UF2 "/tmp/$3.bin" -c -b 0x10000000 -f 0xe48bff59 -o "$3.uf2"
}
build ledtest "" teslamic-rp2350-LEDTEST
# Loopback self-test for the PIO USB line coding. Green = every packet decoded
# back byte for byte; red = decoded but wrong; blue = nothing came back at all;
# amber = the system clock is not 96 MHz. See src/bin/pioloop.rs.
build pioloop "" teslamic-rp2350-PIOLOOP

ls -l ./*.uf2

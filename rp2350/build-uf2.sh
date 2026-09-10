#!/bin/sh
# Build every RP2350 image and package it as a drag-and-drop UF2.
# RP2350 ARM secure-image UF2 family id 0xe48bff59 (the RP2040's is ...56 —
# using the wrong one gives a board that accepts the file and does nothing).
# Flash base 0x10000000.
set -e
cd "$(dirname "$0")"
UF2=../tools/uf2conv.py
OBJ=${OBJCOPY:-arm-none-eabi-objcopy}
# BOARD=usb-c (default) or BOARD=pizero. The two differ in flash size, GPIO
# count and which pins carry D+/D-; see src/board.rs.
BOARD=${BOARD:-usb-c}
SUFFIX=""
[ "$BOARD" = "usb-c" ] || SUFFIX="-$BOARD"

build() { # <bin> <features> <out>
  cargo build --release --bin "$1" --no-default-features \
    --features "board-$BOARD${2:+,$2}"
  $OBJ -O binary "target/thumbv8m.main-none-eabihf/release/$1" "/tmp/$3.bin"
  python3 $UF2 "/tmp/$3.bin" -c -b 0x10000000 -f 0xe48bff59 -o "$3$SUFFIX.uf2"
}
# Measures which GPIO carries the device pull-up. Run this first on any board:
# it answers both "which pins are D+/D-" and "is this port wired as a device at
# all", and both have cost days when assumed. See src/bin/pinscan.rs.
build pinscan "" teslamic-rp2350-PINSCAN
build ledtest "" teslamic-rp2350-LEDTEST
# Loopback self-test for the PIO USB line coding. Green = every packet decoded
# back byte for byte; red = decoded but wrong; blue = nothing came back at all;
# amber = the system clock is not 96 MHz. See src/bin/pioloop.rs.
build pioloop "" teslamic-rp2350-PIOLOOP
# Logic analyser: samples D+/D- at 96 MHz with a spare PIO block and dumps the
# raw waveform over serial. See src/bin/piocap.rs.
build piocap "" teslamic-rp2350-PIOCAP
# Whole transactions: the CPU plays host, the engine replies, PIO2 watches.
# See src/bin/piotxn.rs.
build piotxn "" teslamic-rp2350-PIOTXN
# Stands the port up as a real device for a computer to enumerate. The first
# test with a host on the other end. See src/bin/pioenum.rs.
build pioenum "" teslamic-rp2350-PIOENUM

ls -l ./*.uf2

# What to flash on which board

Two RP2040-Zero boards. They are **not** interchangeable once flashed — one
faces the phone, one faces the car.

```
phone ──USB-C──> [SOURCE board] ──I²S──> [CAR board] ──USB-C──> Tesla
```

## Wiring between the boards

| source board | | car board |
|---|---|---|
| GPIO2 | → | GPIO2 (DATA) |
| GPIO3 | → | GPIO3 (BCK) |
| GPIO4 | → | GPIO4 (LRCK) |
| GND | → | GND |

**Ground is not optional.** The boards are powered from different USB ports;
without a common reference the I²S has nothing to switch against.

## Flash these

Hold BOOTSEL while plugging the board into your Mac, then:

```sh
# SOURCE board — the one the phone plugs into
cp ~/Projects/TeslaAudio/rp2040/teslamic-rp-source-steered-lowlat.uf2 /Volumes/RPI-RP2/

# CAR board — the one that plugs into the Tesla
cp ~/Projects/TeslaAudio/rp2040/teslamic-rp-car-elastic-lowlat.uf2 /Volumes/RPI-RP2/
```

Copy from the terminal, not Finder — Finder crashes on `RPI-RP2` because the
bootloader reboots and unmounts the volume the moment the last block lands.

## Reading the LED (WS2812 on GPIO16)

| colour | source board | car board |
|---|---|---|
| green | streaming | streaming |
| blue | no audio from the host yet, or paused | no I²S clock arriving |
| red | — | muted: source rate is not 48 kHz, or a packet exceeded wMaxPacketSize |
| amber | correcting drift more than expected | — |
| magenta | boot indicator, should turn green/blue quickly | same |

## All images

### Shipping

| image | board | notes |
|---|---|---|
| `teslamic-rp-source-steered-lowlat.uf2` | source | **recommended.** Clock steered to the host, 5.3 ms cushion |
| `teslamic-rp-source-steered.uf2` | source | same, 10.7 ms cushion — more margin |
| `teslamic-rp-source-adaptive.uf2` | source | fallback: upstream I²S master, free-running clock, uses slip correction |
| `teslamic-rp-source-lowlat.uf2` | source | as adaptive, halved cushion |
| `teslamic-rp-car-elastic-lowlat.uf2` | car | **recommended.** 2.7 ms cushion |
| `teslamic-rp-car-elastic.uf2` | car | same, 5.3 ms cushion — more margin |
| `teslamic-rp-car-pcm2706.uf2` | car | identical to `car-elastic`; kept under its own name for the PCM2706 build |
| `teslamic-rp-car-locked.uf2` | car | clock-locked pairing. Its source counterpart is deliberately not built |

### Diagnostics

Flash these only while chasing a fault. Several do not enumerate as USB at all,
which is expected.

| image | board | answers |
|---|---|---|
| `teslamic-rp-LEDTEST.uf2` | either | is the WS2812 driver, PIO program and pin correct? Cycles colours |
| `teslamic-rp-I2SSNIFF.uf2` | car | is a clock arriving? Plain GPIO edge counting, no PIO |
| `teslamic-rp-I2SRX.uf2` | car | does `slave_rx` produce samples? red stalled / amber all-zero / green real data |
| `teslamic-rp-I2STEST.uf2` | source | drives the link with embassy-rp's upstream I²S master, to test `slave_rx` against known-good code |
| `teslamic-rp-source-PANTEST.uf2` | source | which side does I²S slot 0 come out of? Tone in slot 0, silence in slot 1 |
| `teslamic-rp-car-STRESS-TEST.uf2` | car | does the car accept variable packet sizes? Self-contained, needs no wiring |

**Start with the isolation builds, not with a theory.** Every hard bug in this
project was found by taking one variable out of the path; several hours were
lost to reasoning first.

## Known non-issue

Gig Performer displays this device's two channels swapped. The firmware is
correct — `PANTEST` shows I²S slot 0 arriving on the left, and the car plays the
correct side. It is Gig Performer's channel labelling.

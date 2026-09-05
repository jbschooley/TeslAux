# TeslAux single-board PCB

An STM32F407VET6 with **both USB connectors on the board**, so there are no
jumper wires anywhere on a signal path. That is the whole point of the design:
the dev-board build works, but it reaches the phone through four inches of
unshielded jumper carrying a 90 Ω differential pair, and that is the leading
suspect for the occasional pops.

Everything else is stripped: no TFT header, no SD card, no GPIO combs. What
stays is what is needed to bring the board up and debug it.

> **Connect by pin name, not pin number.** The pin numbers below are
> deliberately absent — EasyEDA's `STM32F407VET6` symbol has named pins, which is
> what you wire, and a mis-remembered LQFP100 number is worse than no number at
> all. Every net here is given by signal name on both ends.

## What this is copying

The `STM32F4XX_M` board that already works, minus the peripherals, plus a second
USB-C. Two facts from bringing that board up carry over and are worth not
rediscovering:

* **The crystal is 8 MHz**, and the RCC config divides from it exactly
  (`8 / 4 * 168 / 7 = 48 MHz` for USB). Fit 8 MHz, not 12 or 25.
* **The onboard USB-C already has its CC resistors**, which is why the car can
  power it. Both connectors here need them — see below.

## Nets

### Power

| Net | Connects |
|---|---|
| `VBUS_CAR` | CAR USB-C `VBUS` (A4, B4, A9, B9) → regulator `IN`, TVS |
| `+5V` | regulator `IN`, 10 µF bulk, test point |
| `+3V3` | regulator `OUT`, 10 µF bulk, every `VDD`, `VDDA` (via ferrite), `VBAT`, test point |
| `GND` | everything |

The **car port powers the board**. The phone port's `VBUS` goes to the ESD
diode and **nowhere else** — connecting it would tie the car's 5 V rail to the
phone's, two hosts pushing against each other through the board. This is not a
detail to economise on.

### MCU support — all of this is required, none is optional

| Pin | Connect |
|---|---|
| every `VDD` | 100 nF to `GND`, one per pin, placed at the pin |
| `VDDA` | ferrite bead from `+3V3`, then 1 µF ∥ 100 nF to `GND` |
| `VREF+` | to `VDDA` (or its own 100 nF if the symbol separates it) |
| `VSSA`, every `VSS` | `GND` |
| `VBAT` | `+3V3` (no coin cell wanted) |
| **`VCAP_1`, `VCAP_2`** | **2.2 µF X7R to `GND` each, low ESR, right at the pin** |
| `NRST` | 100 nF to `GND`, plus reset button to `GND` |
| `BOOT0` | 10 kΩ to `GND`; 2-pin header to `+3V3` for DFU |
| `PH0-OSC_IN` / `PH1-OSC_OUT` | 8 MHz crystal, load caps per its datasheet (typically 12–20 pF each) |

`VCAP_1`/`VCAP_2` are the ones people leave off. They feed the internal
regulator; without them the part does not run, and the failure looks like a dead
board.

### USB — car side (OTG_FS)

| Net | From | To |
|---|---|---|
| `USB1_DP` | CAR USB-C `D+` (A6, B6 **tied**) | 22 Ω → `PA12` |
| `USB1_DM` | CAR USB-C `D−` (A7, B7 **tied**) | 22 Ω → `PA11` |
| `USB1_CC1` | CAR USB-C `CC1` | 5.1 kΩ → `GND` |
| `USB1_CC2` | CAR USB-C `CC2` | 5.1 kΩ → `GND` |
| `VBUS_CAR` | CAR USB-C `VBUS` | regulator, as above |

### USB — phone side (OTG_HS, internal full-speed PHY)

| Net | From | To |
|---|---|---|
| `USB2_DP` | PHONE USB-C `D+` (A6, B6 **tied**) | 22 Ω → `PB15` |
| `USB2_DM` | PHONE USB-C `D−` (A7, B7 **tied**) | 22 Ω → `PB14` |
| `USB2_CC1` | PHONE USB-C `CC1` | 5.1 kΩ → `GND` |
| `USB2_CC2` | PHONE USB-C `CC2` | 5.1 kΩ → `GND` |
| PHONE `VBUS` | — | **ESD diode only. Not the rail.** |

**Tie A6/B6 and A7/B7 together on both connectors.** That is what makes a USB 2.0
device work in either cable orientation; without it the cable works one way up
and not the other, which reads as an intermittent fault.

**The 5.1 kΩ CC pulldowns are what make a USB-C host see a device at all.** No
resistors, no VBUS, no enumeration — and the car is a USB-C host.

### Debug and status

| Net | Connect |
|---|---|
| `SWDIO` | `PA13` → SWD header pin 2 |
| `SWCLK` | `PA14` → SWD header pin 4 |
| `NRST` | SWD header pin 5 |
| `+3V3`, `GND` | SWD header pins 1, 3 |
| `LED` | `PA1` → 1 kΩ → LED anode; cathode to `GND` |

`PA1` drives the LED **low to light it**, matching the firmware, which was
written against the existing board's sink-mode LED.

Bring out `PB12`, `PB13`, `PC4`, `PC5` and a `GND` on a small pad field. They
cost nothing and they are the I²S pins a PCM1808 would need if the analogue
input ever happens.

## Bill of materials

| Ref | Part | Notes |
|---|---|---|
| U1 | STM32F407VET6, LQFP100 | 8 MHz HSE assumed by the firmware |
| U2 | 3.3 V LDO, ≥300 mA, SOT-223 | AMS1117-3.3 is fine; ~100 mA draw |
| U3, U4 | USBLC6-2SC6 | ESD protection, one per USB pair |
| J1, J2 | USB-C receptacle, 16-pin, USB 2.0 | both need CC pads broken out |
| J3 | 5-pin header, 2.54 mm | SWD |
| J4 | 2-pin header | BOOT0 |
| Y1 | 8 MHz crystal | load caps to its datasheet |
| SW1 | tactile switch | reset |
| — | 2× 2.2 µF X7R 0805 | `VCAP_1`, `VCAP_2` |
| — | 100 nF 0402 ×8–10 | one per `VDD`, at the pin |
| — | 10 µF ×2, 1 µF ×1 | bulk and `VDDA` |
| — | 22 Ω ×4 | USB series |
| — | 5.1 kΩ ×4 | CC pulldowns |
| — | 10 kΩ ×1 | BOOT0 |
| — | ferrite bead | `VDDA` |
| — | LED + 1 kΩ | status |

## Layout — the part that decides whether this was worth doing

The reason to build this board is signal integrity, so the layout is not a
formality.

1. **Four layers**: signal / **solid ground** / power / signal. A continuous
   ground plane under the USB pairs is what a jumper wire cannot give you, and
   it is the entire point.
2. **Route `D+`/`D−` as a differential pair**: 90 Ω differential, matched
   length within ~2 mm, kept together, over unbroken ground. Short.
3. **Put the 22 Ω resistors near the MCU**, not near the connector.
4. **Do not route anything else between or beneath the pairs.** No stubs.
5. **Crystal close to the pins**, its load caps closer, with a ground pour
   around it and nothing switching underneath.
6. **`VCAP` and `VDD` decoupling at the pins** — a via to the plane, not a
   trace across the board.
7. **Keep the two USB pairs apart from each other.** They are each other's
   nearest aggressor, which is the same crosstalk mechanism suspected of the
   jumper wires.
8. **Connector shields to `GND`** through a pad; leave a spot for a small
   capacitor in case that turns out better in the car.

## What this does not change

The firmware. It is the same STM32 build, same pins — `PA11`/`PA12` for the car,
`PB14`/`PB15` for the phone, `PA1` for the LED — so the board should run what is
already flashed with nothing recompiled.

That is also what makes it a clean experiment: **if the pops persist on a board
with no jumper wires at all, wiring was never the cause**, and the answer is
somewhere else entirely.

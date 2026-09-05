#!/usr/bin/env python3
"""Emit a KiCad 5.1 project for the TeslAux board.

EasyEDA Pro imports "KiCad 5.1 and KiCad 5.9", not the modern S-expression
schematic, so this writes the **legacy** format: a `.sch` plus a `-cache.lib`
holding every symbol. Both are plain text and self-contained.

Pin *numbers* come from the official KiCad STM32F407VETx symbol, downloaded
rather than recalled. Pin *geometry* is invented here, which is fine — position
on the page has no bearing on the netlist, and the alternative was translating
someone else's coordinate system for no gain.

Connectivity is by global label placed exactly on each pin's connection point.
In the legacy format a label at a pin's endpoint joins it, so there are no wire
segments to get subtly wrong.
"""
import re, os, shutil, zipfile

OUT = "kicad5"
NAME = "teslaux"

# ------------------------------------------------------------- MCU pin table
src = open("/tmp/f407base.sym").read()
MCU_PINS = []                                   # (number, name)
for m in re.finditer(r'\(name "([^"]+)"(?:.|\n)*?\(number "(\d+)"', src):
    MCU_PINS.append((int(m.group(2)), m.group(1)))
MCU_PINS.sort()
assert len(MCU_PINS) == 100, f"expected 100 MCU pins, got {len(MCU_PINS)}"

# ------------------------------------------------------------------- the design
# Every net in one table. Anything not named here is left unconnected on
# purpose, and ERC will say so.
MCU_NETS = {
    "PA11": "USB1_DM_R", "PA12": "USB1_DP_R",
    "PB14": "USB2_DM_R", "PB15": "USB2_DP_R",
    "PA13": "SWDIO", "PA14": "SWCLK", "PA1": "LED_A",
    "PH0": "OSC_IN", "PH1": "OSC_OUT",
    "NRST": "NRST", "BOOT0": "BOOT0",
    "VCAP_1": "VCAP_1", "VCAP_2": "VCAP_2",
    "VDDA": "VDDA", "VREF+": "VDDA", "VBAT": "+3V3",
    "VDD": "+3V3", "VSS": "GND", "VSSA": "GND",
}

PARTS = [  # ref, value, pin1 net, pin2 net, footprint
    ("R1","22","USB1_DP","USB1_DP_R","Resistor_SMD:R_0402_1005Metric"),
    ("R2","22","USB1_DM","USB1_DM_R","Resistor_SMD:R_0402_1005Metric"),
    ("R3","22","USB2_DP","USB2_DP_R","Resistor_SMD:R_0402_1005Metric"),
    ("R4","22","USB2_DM","USB2_DM_R","Resistor_SMD:R_0402_1005Metric"),
    ("R5","5.1k","USB1_CC1","GND","Resistor_SMD:R_0402_1005Metric"),
    ("R6","5.1k","USB1_CC2","GND","Resistor_SMD:R_0402_1005Metric"),
    ("R7","5.1k","USB2_CC1","GND","Resistor_SMD:R_0402_1005Metric"),
    ("R8","5.1k","USB2_CC2","GND","Resistor_SMD:R_0402_1005Metric"),
    ("R9","10k","BOOT0","GND","Resistor_SMD:R_0402_1005Metric"),
    ("R10","1k","LED_A","LED_K","Resistor_SMD:R_0402_1005Metric"),
    ("C1","2.2u","VCAP_1","GND","Capacitor_SMD:C_0805_2012Metric"),
    ("C2","2.2u","VCAP_2","GND","Capacitor_SMD:C_0805_2012Metric"),
    ("C3","100n","+3V3","GND","Capacitor_SMD:C_0402_1005Metric"),
    ("C4","100n","+3V3","GND","Capacitor_SMD:C_0402_1005Metric"),
    ("C5","100n","+3V3","GND","Capacitor_SMD:C_0402_1005Metric"),
    ("C6","100n","+3V3","GND","Capacitor_SMD:C_0402_1005Metric"),
    ("C7","100n","+3V3","GND","Capacitor_SMD:C_0402_1005Metric"),
    ("C8","100n","+3V3","GND","Capacitor_SMD:C_0402_1005Metric"),
    ("C13","10u","+3V3","GND","Capacitor_SMD:C_0805_2012Metric"),
    ("C14","10u","+5V","GND","Capacitor_SMD:C_0805_2012Metric"),
    ("C15","1u","VDDA","GND","Capacitor_SMD:C_0603_1608Metric"),
    ("C16","100n","VDDA","GND","Capacitor_SMD:C_0402_1005Metric"),
    ("C17","100n","NRST","GND","Capacitor_SMD:C_0402_1005Metric"),
    ("C18","18p","OSC_IN","GND","Capacitor_SMD:C_0402_1005Metric"),
    ("C19","18p","OSC_OUT","GND","Capacitor_SMD:C_0402_1005Metric"),
    ("FB1","600R","+3V3","VDDA","Inductor_SMD:L_0603_1608Metric"),
    ("Y1","8MHz","OSC_IN","OSC_OUT","Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm"),
    ("D1","LED","LED_K","GND","LED_SMD:LED_0603_1608Metric"),
    ("SW1","RESET","NRST","GND","Button_Switch_SMD:SW_SPST_TL3342"),
]

USB_PINS = [("A4","VBUS"),("A6","D+1"),("A7","D-1"),("A5","CC1"),
            ("B5","CC2"),("B6","D+2"),("B7","D-2"),("A1","GND")]
J1_NETS = ["+5V","USB1_DP","USB1_DM","USB1_CC1","USB1_CC2","USB1_DP","USB1_DM","GND"]
J2_NETS = ["VBUS_PHONE","USB2_DP","USB2_DM","USB2_CC1","USB2_CC2","USB2_DP","USB2_DM","GND"]
LDO_PINS = [("1","GND"),("2","OUT"),("3","IN")]
LDO_NETS = ["GND","+3V3","+5V"]
ESD_PINS = [("1","IO1"),("2","GND"),("3","IO2"),("4","IO2b"),("5","VBUS"),("6","IO1b")]
U3_NETS = ["USB1_DP","GND","USB1_DM","USB1_DM","+5V","USB1_DP"]
U4_NETS = ["USB2_DP","GND","USB2_DM","USB2_DM","VBUS_PHONE","USB2_DP"]
SWD_PINS = [("1","3V3"),("2","SWDIO"),("3","GND"),("4","SWCLK"),("5","NRST")]
SWD_NETS = ["+3V3","SWDIO","GND","SWCLK","NRST"]
BOOT_PINS = [("1","BOOT0"),("2","3V3")]
BOOT_NETS = ["BOOT0","+3V3"]

# ----------------------------------------------------------------- .lib parts
def lib_symbol(name, ref, pins, box_w=600, pitch=100):
    """pins: list of (number, name, side) with side 'L' or 'R'."""
    left  = [p for p in pins if p[2] == "L"]
    right = [p for p in pins if p[2] == "R"]
    rows = max(len(left), len(right))
    half_h = max(200, (rows * pitch) // 2 + 100)
    hw = box_w // 2
    out = [f"#\n# {name}\n#",
           f"DEF {name} {ref} 0 40 Y Y 1 F N",
           f'F0 "{ref}" {-hw} {half_h+50} 50 H V L CNN',
           f'F1 "{name}" {-hw} {-half_h-50} 50 H V L CNN',
           f'F2 "" 0 0 50 H I C CNN',
           f'F3 "" 0 0 50 H I C CNN',
           "DRAW",
           f"S {-hw} {half_h} {hw} {-half_h} 0 1 10 f"]
    for i, (num, pname, _) in enumerate(left):
        y = half_h - 100 - i * pitch
        out.append(f"X {pname} {num} {-hw-200} {y} 200 R 50 50 1 1 B")
    for i, (num, pname, _) in enumerate(right):
        y = half_h - 100 - i * pitch
        out.append(f"X {pname} {num} {hw+200} {y} 200 L 50 50 1 1 B")
    out += ["ENDDRAW", "ENDDEF"]
    return "\n".join(out), half_h, hw

def two_pin_lib(name, ref):
    return "\n".join([
        f"#\n# {name}\n#",
        f"DEF {name} {ref} 0 40 N N 1 F N",
        f'F0 "{ref}" 0 100 50 H V C CNN',
        f'F1 "{name}" 0 -100 50 H V C CNN',
        f'F2 "" 0 0 50 H I C CNN',
        f'F3 "" 0 0 50 H I C CNN',
        "DRAW",
        "S -40 60 40 -60 0 1 10 f",
        "X ~ 1 0 150 90 D 50 50 1 1 P",
        "X ~ 2 0 -150 90 U 50 50 1 1 P",
        "ENDDRAW", "ENDDEF"])

# MCU symbol: pins 1-50 down the left, 51-100 down the right.
mcu_pins = [(n, nm, "L" if n <= 50 else "R") for n, nm in MCU_PINS]
mcu_lib, MCU_HH, MCU_HW = lib_symbol("STM32F407VETx", "U", mcu_pins, box_w=1600)

usb_lib, USB_HH, USB_HW = lib_symbol("USB_C_Receptacle", "J",
    [(n, nm, "L") for n, nm in USB_PINS], box_w=600)
ldo_lib, LDO_HH, LDO_HW = lib_symbol("AMS1117-3.3", "U",
    [(n, nm, "L") for n, nm in LDO_PINS], box_w=600)
esd_lib, ESD_HH, ESD_HW = lib_symbol("USBLC6-2SC6", "U",
    [(n, nm, "L") for n, nm in ESD_PINS], box_w=600)
swd_lib, SWD_HH, SWD_HW = lib_symbol("Conn_01x05", "J",
    [(n, nm, "L") for n, nm in SWD_PINS], box_w=400)
boot_lib, BOOT_HH, BOOT_HW = lib_symbol("Conn_01x02", "J",
    [(n, nm, "L") for n, nm in BOOT_PINS], box_w=400)

lib = "\n".join(["EESchema-LIBRARY Version 2.4", "#encoding utf-8",
                 two_pin_lib("Device_2pin", "U"),
                 mcu_lib, usb_lib, ldo_lib, esd_lib, swd_lib, boot_lib,
                 "#", "#End Library", ""])

# ------------------------------------------------------------------ the sheet
comps, labels = [], []
_ser = [0]

def timestamp():
    _ser[0] += 1
    return f"{_ser[0]:08X}"

def comp(lib_name, ref, value, x, y, footprint=""):
    comps.append("\n".join([
        "$Comp",
        f"L {NAME}:{lib_name} {ref}",
        f"U 1 1 {timestamp()}",
        f"P {x} {y}",
        f'F 0 "{ref}" H {x} {y-50} 50  0000 C CNN',
        f'F 1 "{value}" H {x} {y+50} 50  0000 C CNN',
        f'F 2 "{footprint}" H {x} {y} 50  0001 C CNN',
        f'F 3 "" H {x} {y} 50  0001 C CNN',
        f"\t1    {x} {y}",
        "\t1    0    0    -1  ",
        "$EndComp"]))

def glabel(net, x, y, orient=2):
    """A global label at a pin's connection point joins that pin. orient 2 =
    text extends left, which suits a pin whose connection point is on the left
    of its symbol."""
    labels.append(f"Text GLabel {x} {y} {orient}    50   BiDi ~ 0\n{net}")

# In the legacy format a component's pin sits at (comp_x + px, comp_y - py).
def pin_xy(cx, cy, px, py):
    return cx + px, cy - py

# --- MCU -------------------------------------------------------------------
MX, MY = 6000, 5000
comp("STM32F407VETx", "U1", "STM32F407VET6", MX, MY,
     "Package_QFP:LQFP-100_14x14mm_P0.5mm")
for i, (num, pname, side) in enumerate(mcu_pins):
    net = MCU_NETS.get(pname)
    if not net:
        continue
    idx = [p for p in mcu_pins if p[2] == side].index((num, pname, side))
    py = MCU_HH - 100 - idx * 100
    px = -(MCU_HW + 200) if side == "L" else (MCU_HW + 200)
    x, y = pin_xy(MX, MY, px, py)
    glabel(net, x, y, 2 if side == "L" else 0)

# --- two-pin parts ---------------------------------------------------------
x, y = 1200, 1200
for ref, val, n1, n2, fp in PARTS:
    comp("Device_2pin", ref, val, x, y, fp)
    glabel(n1, *pin_xy(x, y, 0, 150), 1)
    glabel(n2, *pin_xy(x, y, 0, -150), 3)
    x += 700
    if x > 11000:
        x = 1200; y += 900

# --- connectors and ICs ----------------------------------------------------
def place_sided(lib_name, ref, value, cx, cy, pins, nets, half_h, half_w, fp=""):
    comp(lib_name, ref, value, cx, cy, fp)
    for i, ((num, pname), net) in enumerate(zip(pins, nets)):
        py = half_h - 100 - i * 100
        glabel(net, *pin_xy(cx, cy, -(half_w + 200), py), 2)

y += 1200
place_sided("USB_C_Receptacle", "J1", "USB-C CAR", 2500, y,
            USB_PINS, J1_NETS, USB_HH, USB_HW)
place_sided("USB_C_Receptacle", "J2", "USB-C PHONE", 5000, y,
            USB_PINS, J2_NETS, USB_HH, USB_HW)
place_sided("AMS1117-3.3", "U2", "AMS1117-3.3", 7500, y,
            LDO_PINS, LDO_NETS, LDO_HH, LDO_HW, "Package_TO_SOT_SMD:SOT-223-3_TabPin2")
place_sided("USBLC6-2SC6", "U3", "USBLC6-2SC6", 9500, y,
            ESD_PINS, U3_NETS, ESD_HH, ESD_HW, "Package_TO_SOT_SMD:SOT-23-6")

y += 1400
place_sided("USBLC6-2SC6", "U4", "USBLC6-2SC6", 2500, y,
            ESD_PINS, U4_NETS, ESD_HH, ESD_HW, "Package_TO_SOT_SMD:SOT-23-6")
place_sided("Conn_01x05", "J3", "SWD", 5000, y,
            SWD_PINS, SWD_NETS, SWD_HH, SWD_HW,
            "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical")
place_sided("Conn_01x02", "J4", "BOOT0", 7500, y,
            BOOT_PINS, BOOT_NETS, BOOT_HH, BOOT_HW,
            "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical")

sch = "\n".join([
    "EESchema Schematic File Version 4",
    f"LIBS:{NAME}-cache",
    "EELAYER 30 0",
    "EELAYER END",
    "$Descr A2 22047 15591",
    "encoding utf-8",
    "Sheet 1 1",
    'Title "TeslAux single-board bridge"',
    'Date ""',
    'Rev "A"',
    'Comp ""',
    'Comment1 "OTG_FS (PA11/PA12) to the car, OTG_HS (PB14/PB15) to the phone"',
    'Comment2 "Connectivity is by global label - see pcb/netlist.csv"',
    'Comment3 "Layout is NOT generated - see HARDWARE-PCB.md"',
    'Comment4 ""',
    "$EndDescr",
    "\n".join(comps),
    "\n".join(labels),
    "$EndSCHEMATC", ""])

os.makedirs(OUT, exist_ok=True)
open(f"{OUT}/{NAME}.sch", "w").write(sch)
open(f"{OUT}/{NAME}-cache.lib", "w").write(lib)
open(f"{OUT}/{NAME}.pro", "w").write(
    "update=Never\nversion=1\nlast_client=eeschema\n"
    "[general]\nversion=1\n[eeschema]\nversion=1\nLibDir=\n"
    "[eeschema/libraries]\nLibName1=" + NAME + "-cache\n")

with zipfile.ZipFile(f"{NAME}-kicad5.zip", "w", zipfile.ZIP_DEFLATED) as z:
    for f in (f"{NAME}.sch", f"{NAME}-cache.lib", f"{NAME}.pro"):
        z.write(f"{OUT}/{f}", f)

print(f"wrote {OUT}/{NAME}.sch, {OUT}/{NAME}-cache.lib, {NAME}-kicad5.zip")
print(f"  {len(comps)} components, {len(labels)} global labels")
print(f"  MCU pins: {len(MCU_PINS)}")

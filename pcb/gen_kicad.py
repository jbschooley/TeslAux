#!/usr/bin/env python3
"""Emit a KiCad 9 schematic for the TeslAux board.

Why generated rather than drawn: the value of a machine-written schematic is
that every net comes from one table, so the file and `netlist.csv` cannot drift
apart. The layout is still a human job — see ../HARDWARE-PCB.md.

Connectivity is expressed with **global labels on pin stubs** rather than routed
wires. Two reasons: a label carries the same netlist weight as a wire, and a
generated rat's nest of wire segments is far more likely to be subtly wrong than
a label whose text I can check against the table.

The MCU symbol is the official KiCad one, fetched rather than typed, so its 100
pin numbers are ST's rather than mine.
"""
import re, uuid, textwrap

MCU_LIB = "/tmp/f407base.sym"

def uid():
    return str(uuid.uuid4())

# ---------------------------------------------------------------- pin lookup
src = open(MCU_LIB).read()
PINS = {}                       # name -> [numbers]
for m in re.finditer(r'\(name "([^"]+)"(?:.|\n)*?\(number "(\d+)"', src):
    PINS.setdefault(m.group(1), []).append(m.group(2))

# The MCU symbol body, lifted whole and renamed. Keeping ST's own pin numbering
# is the entire point of using it.
body = src[src.index('(symbol "STM32F407V_E-G_Tx"'):]
body = body[:body.rindex(')')]          # drop the library's closing paren
body = body.replace('STM32F407V_E-G_Tx', 'STM32F407VETx')

# ------------------------------------------------------------- what connects
# One table. Every net in the design, by MCU pin name or by discrete part.
MCU_NETS = {
    "PA11": "USB1_DM_R", "PA12": "USB1_DP_R",       # car   (OTG_FS)
    "PB14": "USB2_DM_R", "PB15": "USB2_DP_R",       # phone (OTG_HS)
    "PA13": "SWDIO", "PA14": "SWCLK",
    "PA1": "LED_A",
    "PH0": "OSC_IN", "PH1": "OSC_OUT",
    "NRST": "NRST", "BOOT0": "BOOT0",
    "VCAP_1": "VCAP_1", "VCAP_2": "VCAP_2",
    "VDDA": "VDDA", "VREF+": "VDDA",
    "VBAT": "+3V3",
    "VDD": "+3V3", "VSS": "GND", "VSSA": "GND",
}

# Two-pin parts: ref, value, pin1 net, pin2 net, footprint hint
PARTS = [
    ("R1", "22",   "USB1_DP",  "USB1_DP_R", "R_0402"),
    ("R2", "22",   "USB1_DM",  "USB1_DM_R", "R_0402"),
    ("R3", "22",   "USB2_DP",  "USB2_DP_R", "R_0402"),
    ("R4", "22",   "USB2_DM",  "USB2_DM_R", "R_0402"),
    ("R5", "5.1k", "USB1_CC1", "GND",       "R_0402"),
    ("R6", "5.1k", "USB1_CC2", "GND",       "R_0402"),
    ("R7", "5.1k", "USB2_CC1", "GND",       "R_0402"),
    ("R8", "5.1k", "USB2_CC2", "GND",       "R_0402"),
    ("R9", "10k",  "BOOT0",    "GND",       "R_0402"),
    ("R10","1k",   "LED_A",    "LED_K",     "R_0402"),
    ("C1", "2.2u", "VCAP_1",   "GND",       "C_0805"),
    ("C2", "2.2u", "VCAP_2",   "GND",       "C_0805"),
    ("C3", "100n", "+3V3",     "GND",       "C_0402"),
    ("C4", "100n", "+3V3",     "GND",       "C_0402"),
    ("C5", "100n", "+3V3",     "GND",       "C_0402"),
    ("C6", "100n", "+3V3",     "GND",       "C_0402"),
    ("C7", "100n", "+3V3",     "GND",       "C_0402"),
    ("C8", "100n", "+3V3",     "GND",       "C_0402"),
    ("C13","10u",  "+3V3",     "GND",       "C_0805"),
    ("C14","10u",  "+5V",      "GND",       "C_0805"),
    ("C15","1u",   "VDDA",     "GND",       "C_0603"),
    ("C16","100n", "VDDA",     "GND",       "C_0402"),
    ("C17","100n", "NRST",     "GND",       "C_0402"),
    ("C18","18p",  "OSC_IN",   "GND",       "C_0402"),
    ("C19","18p",  "OSC_OUT",  "GND",       "C_0402"),
    ("FB1","600R", "+3V3",     "VDDA",      "L_0603"),
    ("Y1", "8MHz", "OSC_IN",   "OSC_OUT",   "Crystal_SMD_3225"),
    ("D1", "LED",  "LED_K",    "GND",       "LED_0603"),
    ("SW1","RESET","NRST",     "GND",       "SW_SPST"),
]

# ------------------------------------------------------------- symbol shapes
def two_pin_symbol(name, pin1="1", pin2="2"):
    return f'''
		(symbol "{name}"
			(pin_numbers (hide yes))
			(pin_names (offset 0) (hide yes))
			(exclude_from_sim no) (in_bom yes) (on_board yes)
			(property "Reference" "U" (at 0 3.81 0)
				(effects (font (size 1.27 1.27))))
			(property "Value" "{name}" (at 0 -3.81 0)
				(effects (font (size 1.27 1.27))))
			(property "Footprint" "" (at 0 0 0)
				(effects (font (size 1.27 1.27)) (hide yes)))
			(symbol "{name}_0_1"
				(rectangle (start -1.27 2.54) (end 1.27 -2.54)
					(stroke (width 0.254) (type default))
					(fill (type none))))
			(symbol "{name}_1_1"
				(pin passive line (at 0 5.08 270) (length 2.54)
					(name "~" (effects (font (size 1.27 1.27))))
					(number "{pin1}" (effects (font (size 1.27 1.27)))))
				(pin passive line (at 0 -5.08 90) (length 2.54)
					(name "~" (effects (font (size 1.27 1.27))))
					(number "{pin2}" (effects (font (size 1.27 1.27)))))))'''

def usbc_symbol():
    """Only the pins this design uses. The shield and SBU lines are left off
    deliberately: unused pins on a generated symbol invite being wired to
    something."""
    pins = [("A4","VBUS"),("A6","DP1"),("A7","DM1"),("A5","CC1"),
            ("B5","CC2"),("B6","DP2"),("B7","DM2"),("A1","GND")]
    out = []
    for i,(num,nm) in enumerate(pins):
        y = 10.16 - i*2.54
        out.append(f'''				(pin passive line (at -10.16 {y} 0) (length 2.54)
					(name "{nm}" (effects (font (size 1.27 1.27))))
					(number "{num}" (effects (font (size 1.27 1.27)))))''')
    return f'''
		(symbol "USB_C_Receptacle"
			(pin_names (offset 1.016))
			(exclude_from_sim no) (in_bom yes) (on_board yes)
			(property "Reference" "J" (at 0 12.7 0)
				(effects (font (size 1.27 1.27))))
			(property "Value" "USB_C_Receptacle" (at 0 -12.7 0)
				(effects (font (size 1.27 1.27))))
			(property "Footprint" "" (at 0 0 0)
				(effects (font (size 1.27 1.27)) (hide yes)))
			(symbol "USB_C_Receptacle_0_1"
				(rectangle (start -7.62 11.43) (end 7.62 -11.43)
					(stroke (width 0.254) (type default))
					(fill (type none))))
			(symbol "USB_C_Receptacle_1_1"
{chr(10).join(out)}))'''

def header_symbol(name, pins):
    out = []
    for i,(num,nm) in enumerate(pins):
        y = (len(pins)-1)*1.27 - i*2.54
        out.append(f'''				(pin passive line (at -7.62 {y:.2f} 0) (length 2.54)
					(name "{nm}" (effects (font (size 1.27 1.27))))
					(number "{num}" (effects (font (size 1.27 1.27)))))''')
    h = len(pins)*1.27 + 1.27
    return f'''
		(symbol "{name}"
			(pin_names (offset 1.016))
			(exclude_from_sim no) (in_bom yes) (on_board yes)
			(property "Reference" "J" (at 0 {h+1.27:.2f} 0)
				(effects (font (size 1.27 1.27))))
			(property "Value" "{name}" (at 0 {-h-1.27:.2f} 0)
				(effects (font (size 1.27 1.27))))
			(property "Footprint" "" (at 0 0 0)
				(effects (font (size 1.27 1.27)) (hide yes)))
			(symbol "{name}_0_1"
				(rectangle (start -5.08 {h:.2f}) (end 5.08 {-h:.2f})
					(stroke (width 0.254) (type default))
					(fill (type none))))
			(symbol "{name}_1_1"
{chr(10).join(out)}))'''

def ldo_symbol():
    pins = [("1","GND"),("2","OUT"),("3","IN")]
    out = []
    for i,(num,nm) in enumerate(pins):
        y = 2.54 - i*2.54
        out.append(f'''				(pin passive line (at -10.16 {y} 0) (length 2.54)
					(name "{nm}" (effects (font (size 1.27 1.27))))
					(number "{num}" (effects (font (size 1.27 1.27)))))''')
    return f'''
		(symbol "AMS1117-3.3"
			(pin_names (offset 1.016))
			(exclude_from_sim no) (in_bom yes) (on_board yes)
			(property "Reference" "U" (at 0 6.35 0)
				(effects (font (size 1.27 1.27))))
			(property "Value" "AMS1117-3.3" (at 0 -6.35 0)
				(effects (font (size 1.27 1.27))))
			(property "Footprint" "" (at 0 0 0)
				(effects (font (size 1.27 1.27)) (hide yes)))
			(symbol "AMS1117-3.3_0_1"
				(rectangle (start -7.62 5.08) (end 7.62 -5.08)
					(stroke (width 0.254) (type default))
					(fill (type none))))
			(symbol "AMS1117-3.3_1_1"
{chr(10).join(out)}))'''

def esd_symbol():
    pins = [("1","IO1"),("2","GND"),("3","IO2"),("4","IO2b"),("5","VBUS"),("6","IO1b")]
    out = []
    for i,(num,nm) in enumerate(pins):
        y = 6.35 - i*2.54
        out.append(f'''				(pin passive line (at -10.16 {y} 0) (length 2.54)
					(name "{nm}" (effects (font (size 1.27 1.27))))
					(number "{num}" (effects (font (size 1.27 1.27)))))''')
    return f'''
		(symbol "USBLC6-2SC6"
			(pin_names (offset 1.016))
			(exclude_from_sim no) (in_bom yes) (on_board yes)
			(property "Reference" "U" (at 0 10.16 0)
				(effects (font (size 1.27 1.27))))
			(property "Value" "USBLC6-2SC6" (at 0 -10.16 0)
				(effects (font (size 1.27 1.27))))
			(property "Footprint" "" (at 0 0 0)
				(effects (font (size 1.27 1.27)) (hide yes)))
			(symbol "USBLC6-2SC6_0_1"
				(rectangle (start -7.62 8.89) (end 7.62 -8.89)
					(stroke (width 0.254) (type default))
					(fill (type none))))
			(symbol "USBLC6-2SC6_1_1"
{chr(10).join(out)}))'''

# --------------------------------------------------------------- the sheet
placed, labels = [], []

def place(lib, ref, value, x, y, footprint=""):
    placed.append(f'''	(symbol
		(lib_id "teslaux:{lib}") (at {x} {y} 0) (unit 1)
		(exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)
		(uuid "{uid()}")
		(property "Reference" "{ref}" (at {x} {y-6.35} 0)
			(effects (font (size 1.27 1.27))))
		(property "Value" "{value}" (at {x} {y+6.35} 0)
			(effects (font (size 1.27 1.27))))
		(property "Footprint" "{footprint}" (at {x} {y} 0)
			(effects (font (size 1.27 1.27)) (hide yes)))
		(instances (project "teslaux"
			(path "/{SHEET}" (reference "{ref}") (unit 1)))))''')

def label(net, x, y, rot=0):
    labels.append(f'''	(global_label "{net}" (shape input) (at {x} {y} {rot})
		(effects (font (size 1.27 1.27)) (justify left))
		(uuid "{uid()}"))''')

def wire(x1, y1, x2, y2):
    labels.append(f'''	(wire (pts (xy {x1} {y1}) (xy {x2} {y2}))
		(stroke (width 0) (type default)) (uuid "{uid()}"))''')

SHEET = uid()

# MCU, with a stub and a label on every pin we use.
MCU_X, MCU_Y = 120.0, 100.0
place("STM32F407VETx", "U1", "STM32F407VET6", MCU_X, MCU_Y,
      "Package_QFP:LQFP-100_14x14mm_P0.5mm")

# Two-pin parts in a grid, each pin stubbed to a label.
x, y = 30.0, 30.0
for ref, val, n1, n2, fp in PARTS:
    lib = "R_C_2pin"
    place(lib, ref, val, x, y, fp)
    wire(x, y - 5.08, x, y - 7.62); label(n1, x, y - 7.62, 90)
    wire(x, y + 5.08, x, y + 7.62); label(n2, x, y + 7.62, 270)
    x += 25.4
    if x > 250:
        x = 30.0; y += 38.1

# Connectors, regulator, ESD.
y += 45.0
place("USB_C_Receptacle", "J1", "USB-C CAR", 40.0, y, "")
for i,(nm,net) in enumerate([("VBUS","+5V"),("DP1","USB1_DP"),("DM1","USB1_DM"),
                             ("CC1","USB1_CC1"),("CC2","USB1_CC2"),
                             ("DP2","USB1_DP"),("DM2","USB1_DM"),("GND","GND")]):
    yy = y + 10.16 - i*2.54
    wire(29.84, yy, 22.0, yy); label(net, 22.0, yy, 180)

place("USB_C_Receptacle", "J2", "USB-C PHONE", 120.0, y, "")
for i,(nm,net) in enumerate([("VBUS","VBUS_PHONE"),("DP1","USB2_DP"),("DM1","USB2_DM"),
                             ("CC1","USB2_CC1"),("CC2","USB2_CC2"),
                             ("DP2","USB2_DP"),("DM2","USB2_DM"),("GND","GND")]):
    yy = y + 10.16 - i*2.54
    wire(109.84, yy, 102.0, yy); label(net, 102.0, yy, 180)

place("AMS1117-3.3", "U2", "AMS1117-3.3", 200.0, y, "")
for i,(nm,net) in enumerate([("GND","GND"),("OUT","+3V3"),("IN","+5V")]):
    yy = y + 2.54 - i*2.54
    wire(189.84, yy, 182.0, yy); label(net, 182.0, yy, 180)

y += 45.0
place("USBLC6-2SC6", "U3", "USBLC6-2SC6", 40.0, y, "")
for i,(nm,net) in enumerate([("IO1","USB1_DP"),("GND","GND"),("IO2","USB1_DM"),
                             ("IO2b","USB1_DM"),("VBUS","+5V"),("IO1b","USB1_DP")]):
    yy = y + 6.35 - i*2.54
    wire(29.84, yy, 22.0, yy); label(net, 22.0, yy, 180)

place("USBLC6-2SC6", "U4", "USBLC6-2SC6", 120.0, y, "")
for i,(nm,net) in enumerate([("IO1","USB2_DP"),("GND","GND"),("IO2","USB2_DM"),
                             ("IO2b","USB2_DM"),("VBUS","VBUS_PHONE"),("IO1b","USB2_DP")]):
    yy = y + 6.35 - i*2.54
    wire(109.84, yy, 102.0, yy); label(net, 102.0, yy, 180)

place("Conn_1x05", "J3", "SWD", 200.0, y, "")
for i,(nm,net) in enumerate([("1","+3V3"),("2","SWDIO"),("3","GND"),("4","SWCLK"),("5","NRST")]):
    yy = y + 5.08 - i*2.54
    wire(192.38, yy, 184.0, yy); label(net, 184.0, yy, 180)

y += 40.0
place("Conn_1x02", "J4", "BOOT0", 200.0, y, "")
for i,(nm,net) in enumerate([("1","BOOT0"),("2","+3V3")]):
    yy = y + 1.27 - i*2.54
    wire(192.38, yy, 184.0, yy); label(net, 184.0, yy, 180)

# MCU pin labels: a stub and a label per used pin, on ST's own numbering.
lx, ly = 250.0, 30.0
for pin_name, net in sorted(MCU_NETS.items()):
    for num in PINS.get(pin_name, []):
        labels.append(f'''	(text "U1.{num} {pin_name} -> {net}" (at {lx} {ly} 0)
		(effects (font (size 1.27 1.27)) (justify left)) (uuid "{uid()}"))''')
        ly += 3.0
        if ly > 280:
            ly = 30.0; lx += 60.0

libs = "\n".join([two_pin_symbol("R_C_2pin"), usbc_symbol(), ldo_symbol(),
                  esd_symbol(), header_symbol("Conn_1x05",
                      [("1","1"),("2","2"),("3","3"),("4","4"),("5","5")]),
                  header_symbol("Conn_1x02", [("1","1"),("2","2")]),
                  "\n\t\t" + body.strip()])

out = f'''(kicad_sch
	(version 20231120)
	(generator "teslaux")
	(generator_version "8.0")
	(uuid "{SHEET}")
	(paper "A2")
	(title_block
		(title "TeslAux single-board bridge")
		(date "")
		(rev "A")
		(comment 1 "Two USB-C: OTG_FS to the car, OTG_HS to the phone")
		(comment 2 "Connectivity is by global label; see pcb/netlist.csv")
		(comment 3 "Layout is NOT generated - see HARDWARE-PCB.md")
	)
	(lib_symbols{libs}
	)
{chr(10).join(placed)}
{chr(10).join(labels)}
)
'''
open("teslaux.kicad_sch","w").write(out)
print(f"wrote teslaux.kicad_sch  ({len(out)} bytes)")
print(f"  {len(placed)} components, {len(labels)} labels/wires")
missing = [n for n in MCU_NETS if n not in PINS]
print(f"  MCU pin names not found in the symbol: {missing or 'none'}")

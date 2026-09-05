# PCB working files

`bom.csv` and `netlist.csv` are the parts of this board that can be written
down. The rest cannot, and it is worth being clear about which is which.

## What is here

* **`bom.csv`** — every component, value and package. Paste into EasyEDA's BOM
  view or use it as the shopping list. Nothing is optional: `C1`/`C2` feed the
  MCU's internal regulator, and without them the part does not run.
* **`netlist.csv`** — every net, by **pin name**. Wire these and the schematic is
  complete. Pin *numbers* are deliberately absent: EasyEDA's symbol has named
  pins, and a recalled LQFP100 number would be a guess.

See `../HARDWARE-PCB.md` for why each net is the way it is, and for the layout
rules that matter.

## What is not here, and will not be

**Gerbers and pick-and-place cannot be generated from this.** They are not
derived from a schematic — they *are* the layout: every component placed, every
trace routed, every pour drawn. There is no honest path from a netlist to a
Gerber without doing that work.

That matters more than usual here, because **the layout is the entire reason to
build this board**. The dev-board version already works; what it lacks is 90 Ω
differential pairs over an unbroken ground plane. A generated layout would be
exactly the part that cannot be generated.

A native EasyEDA schematic is likewise not included. Their format keys each
component to a UUID in their library, so an invented 100-pin symbol imports as
broken geometry — slower to repair than to draw.

## Suggested order

1. Draw the schematic in EasyEDA from `netlist.csv`, picking parts from their
   library so the footprints and JLCPCB part numbers come along.
2. Run their DRC/ERC. Every net in `netlist.csv` should appear, and nothing else
   should.
3. Lay out by hand, following the rules in `../HARDWARE-PCB.md` — four layers,
   solid ground under the pairs, 22 Ω resistors at the MCU, crystal tight.
4. Let EasyEDA produce the Gerbers and pick-and-place from that.

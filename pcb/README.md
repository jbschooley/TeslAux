# PCB working files

`bom.csv` and `netlist.csv` are the parts of this board that can be written
down. The rest cannot, and it is worth being clear about which is which.

## What is here

* **`teslaux.kicad_sch`** — a KiCad 8 schematic, importable via EasyEDA's
  **File > Import > KiCad**. It is self-contained: every symbol definition is
  embedded, so nothing depends on your libraries.

  The MCU symbol is the **official KiCad one**, downloaded rather than typed, so
  its 100 pin numbers are ST's and not my recollection. `gen_kicad.py` builds the
  file from one table of nets, which is the point of generating it — the
  schematic and `netlist.csv` are checked against each other and currently agree
  on all 27 nets.

  Connectivity is expressed with **global labels on pin stubs** rather than
  routed wires. A label carries the same netlist weight, and a generated tangle
  of wire segments is far likelier to be subtly wrong than a label whose text can
  be diffed against the table.


* **`gen_kicad.py`** — regenerates the schematic. Edit the net table there, not
  the `.kicad_sch`.
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

1. **Import `teslaux.kicad_sch`** into EasyEDA. Expect to reassign footprints:
   the symbols carry pin numbers and net names, which is what matters for
   connectivity, but their footprint fields are hints rather than EasyEDA library
   references.
2. Run their ERC and check it against `netlist.csv` — every net should appear
   and nothing else should.
3. Lay out by hand, following the rules in `../HARDWARE-PCB.md` — four layers,
   solid ground under the pairs, 22 Ω resistors at the MCU, crystal tight.
4. Let EasyEDA produce the Gerbers and pick-and-place from that.

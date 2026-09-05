# Case for the soldered pair

Two RP2040-Zeros soldered end to end — castellations butted, component sides up,
a USB-C port at each outer end, the I²S link running across the joint.

```
    USB-C -> [RP2040-Zero | RP2040-Zero] <- USB-C
     phone              ^ joint             car
```

| file | triangles | size |
|---|---|---|
| `rp2040_soldered_case.stl` | 2,746 | 53.2 x 24.2 x 7.8 mm |
| `rp2040_soldered_lid.stl` | 3,490 | 53.2 x 24.2 x 7.6 mm |

53.2 x 24.2 x 9.4 mm closed. The lid is flush with the base — no lip, so the
seam is the only purchase for opening it.

## Printing

Both parts print without supports.

- **Base:** flat on the bed as oriented.
- **Lid:** outer face down, so the tabs and posts point up.

The rounded edges sit against the bed either way. A quarter round leaves the face
horizontally, so the first few layers grow outward fast — about 0.24 mm per
0.2 mm layer at the steepest, within what a 0.4 mm extrusion bridges, but slow
the first few layers if the edge comes out rough.

## Fit notes

These are the second revision, after a test print. What changed:

- The board is **0.8 mm thick, not 1.6** — measured off the real thing, which is
  why the case got shorter despite everything else growing.
- A SOT-23 on the underside beside each USB port landed on the support ring; the
  ring is now cut away 8 mm across, 9 mm in from the cavity wall at each end.
- The USB sill was too low once the thickness was corrected, and sits 0.3 mm
  below flush so a sill printed slightly proud cannot foul the connector.
- Two ribs bracing the joint from underneath fouled the crystals, which sit 4–5 mm
  either side of it. Gone; the ring's long runs carry the board across the gap.

Only the two STLs are here. The generator that produced them, and the full write
-up of the revisions, live outside this repo — worth bringing in if the case ever
needs changing, since these files cannot be regenerated without it.

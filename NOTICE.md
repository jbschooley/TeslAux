# Licensing

The project is **MIT** (see [`LICENSE`](LICENSE)), with two exceptions worth
knowing about before you redistribute anything.

## AGPL: the T114's `usb-spy` display driver

`t114/src/st7789.rs` and `t114/src/st7789/framebuffer.rs` are
**AGPL-3.0-or-later**. They were ported from `wireless-performer-fw`, a
separate AGPL project, and keep their original licence.

Nothing includes them unless you build the T114 firmware with
`--features usb-spy`, which is the on-screen USB request logger. Because the
AGPL is copyleft, **that particular binary is a combined work and is AGPL**,
even though the rest of its sources are MIT. Every other T114 build, and the
whole of `rp2040/`, is MIT and contains no AGPL code.

If you want a fully MIT T114 build, do not enable `usb-spy`. The copyright
holder of both projects is the same person, so relicensing those two files for
this project is also an option, and would remove the exception entirely.

## Third party: `tools/uf2conv.py`

`tools/uf2conv.py` and `tools/uf2families.json` come from Microsoft's
[uf2](https://github.com/microsoft/uf2) repository and are MIT licensed there.
They are packaging tools, not part of any firmware image.

## Trademark

"Tesla", "TeslaMic" and "CaraokeMic" are trademarks of Tesla, Inc. This project
is not affiliated with, endorsed by, or connected to Tesla in any way. It is an
independent, interoperability-driven reimplementation.

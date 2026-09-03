# Licensing

The project is **MIT** throughout — see [`LICENSE`](LICENSE). Two pieces have
histories worth recording.

## `t114/src/st7789*`

The ST7789 display driver and its framebuffer were ported from
`wireless-performer-fw`, which is licensed AGPL-3.0-or-later. They have been
**relicensed MIT for this project by the copyright holder**, who owns both.

The originals in `wireless-performer-fw` remain AGPL-3.0-or-later. Only this
copy is MIT, and that grant does not extend to anything else in that project.

## `tools/uf2conv.py`

`tools/uf2conv.py` and `tools/uf2families.json` come from Microsoft's
[uf2](https://github.com/microsoft/uf2) repository and are MIT licensed there.
They are packaging tools and are not part of any firmware image.

## Trademark

"Tesla", "TeslaMic" and "CaraokeMic" are trademarks of Tesla, Inc. This project
is not affiliated with, endorsed by, or connected to Tesla in any way. It is an
independent, interoperability-driven reimplementation.

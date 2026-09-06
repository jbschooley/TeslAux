#!/usr/bin/env python3
"""Ask a USB audio device's Feature Unit what volume range it claims.

The range is not in the descriptor — the descriptor only says a volume control
exists. The numbers come back as class control requests, so the only way to know
what a mic claims it can do is to ask it.

Worth asking of the real mic. The car sets a mic's volume to its advertised
maximum at connect and never revisits it, so that one number is the whole of
what the car learns about a mic's capability.

Usage:
    fuquery.py [vid:pid] [unit] [interface]
    fuquery.py 1235:0002 5 0        # the TeslaMic's Feature Unit 5 on IF0
"""

import ctypes
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from usbdesc import declare, load_libusb  # noqa: E402

GET_CUR, GET_MIN, GET_MAX, GET_RES = 0x81, 0x82, 0x83, 0x84
MUTE, VOLUME = 0x01, 0x02
IN_CLASS_INTERFACE = 0xA1


def query(lu, h, unit, iface, request, selector, channel, length=2):
    buf = (ctypes.c_ubyte * length)()
    n = lu.libusb_control_transfer(
        h,
        IN_CLASS_INTERFACE,
        request,
        (selector << 8) | channel,
        (unit << 8) | iface,
        buf,
        length,
        1000,
    )
    if n < 0:
        return None, n
    return bytes(buf[:n]), n


def db(raw):
    """UAC1 volume is a signed 16-bit count of 1/256 dB."""
    v = int.from_bytes(raw[:2], "little", signed=True)
    if v == -32768:
        return v, "-inf (silence)"
    return v, f"{v / 256:+.2f} dB"


def main(argv):
    want = argv[1] if len(argv) > 1 else "1235:0002"
    unit = int(argv[2]) if len(argv) > 2 else 5
    iface = int(argv[3]) if len(argv) > 3 else 0
    vid, pid = (int(x, 16) for x in want.split(":"))

    lu = load_libusb()
    declare(lu)
    ctx = ctypes.c_void_p()
    if lu.libusb_init(ctypes.byref(ctx)) != 0:
        raise SystemExit("libusb_init failed")
    lst = ctypes.POINTER(ctypes.c_void_p)()
    n = lu.libusb_get_device_list(ctx, ctypes.byref(lst))

    from usbdesc import DeviceDescriptor

    handle = ctypes.c_void_p()
    opened = False
    for i in range(n):
        dd = DeviceDescriptor()
        if lu.libusb_get_device_descriptor(lst[i], ctypes.byref(dd)) != 0:
            continue
        if dd.idVendor == vid and dd.idProduct == pid:
            if lu.libusb_open(lst[i], ctypes.byref(handle)) == 0:
                opened = True
            break
    if not opened:
        raise SystemExit(f"no device {want} that could be opened")

    print(f"{want}  Feature Unit {unit} on interface {iface}")
    for ch, label in ((0, "master"), (1, "ch1"), (2, "ch2")):
        raw, rc = query(lu, handle, unit, iface, GET_CUR, MUTE, ch, 1)
        if raw:
            print(f"  {label:6s} mute   CUR = {raw[0]}")
    for ch, label in ((0, "master"), (1, "ch1"), (2, "ch2")):
        line = f"  {label:6s} volume"
        any_ok = False
        for name, req in (("CUR", GET_CUR), ("MIN", GET_MIN), ("MAX", GET_MAX), ("RES", GET_RES)):
            raw, rc = query(lu, handle, unit, iface, req, VOLUME, ch)
            if raw and len(raw) >= 2:
                v, pretty = db(raw)
                line += f"  {name} = {v:>6} ({pretty})"
                any_ok = True
        print(line if any_ok else f"  {label:6s} volume  — not supported")

    lu.libusb_close(handle)
    lu.libusb_free_device_list(lst, 1)
    lu.libusb_exit(ctx)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

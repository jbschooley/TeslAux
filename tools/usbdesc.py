#!/usr/bin/env python3
"""Dump the full configuration descriptor of every attached TeslaMic.

The car validates the descriptor set, and `teslamic.rs` is shared between the
RP2040 and STM32 builds — but *sharing the source is not the same as emitting
the same bytes*. Endpoint numbering, descriptor ordering and anything the HAL
inserts are outside our code. With two boards plugged in, this prints both and
diffs them, which either finds the difference or eliminates the whole class.

    tools/usbdesc.py            # dump and diff every TeslaMic found
    tools/usbdesc.py --raw      # bytes only, no parsing

macOS has no tool that shows this: `system_profiler` summarises, and `ioreg`
cannot reach a class-specific descriptor. A control transfer can.
"""

import ctypes
import ctypes.util
import sys

VID, PID = 0x1235, 0x0002


def load_libusb():
    for cand in (
        ctypes.util.find_library("usb-1.0"),
        "/opt/homebrew/lib/libusb-1.0.dylib",
        "/usr/local/lib/libusb-1.0.dylib",
    ):
        if cand:
            try:
                return ctypes.CDLL(cand)
            except OSError:
                pass
    raise SystemExit("libusb not found; try: brew install libusb")


class DeviceDescriptor(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_uint8),
        ("bDescriptorType", ctypes.c_uint8),
        ("bcdUSB", ctypes.c_uint16),
        ("bDeviceClass", ctypes.c_uint8),
        ("bDeviceSubClass", ctypes.c_uint8),
        ("bDeviceProtocol", ctypes.c_uint8),
        ("bMaxPacketSize0", ctypes.c_uint8),
        ("idVendor", ctypes.c_uint16),
        ("idProduct", ctypes.c_uint16),
        ("bcdDevice", ctypes.c_uint16),
        ("iManufacturer", ctypes.c_uint8),
        ("iProduct", ctypes.c_uint8),
        ("iSerialNumber", ctypes.c_uint8),
        ("bNumConfigurations", ctypes.c_uint8),
    ]


def declare(lu):
    """ctypes defaults every argument to int, which segfaults on 64-bit
    pointers. Declare each signature explicitly."""
    c, p, u8, u16 = ctypes.c_int, ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint16
    lu.libusb_init.argtypes = [ctypes.POINTER(p)]
    lu.libusb_init.restype = c
    lu.libusb_exit.argtypes = [p]
    lu.libusb_get_device_list.argtypes = [p, ctypes.POINTER(ctypes.POINTER(p))]
    lu.libusb_get_device_list.restype = ctypes.c_ssize_t
    lu.libusb_free_device_list.argtypes = [ctypes.POINTER(p), c]
    lu.libusb_get_device_descriptor.argtypes = [p, ctypes.POINTER(DeviceDescriptor)]
    lu.libusb_get_device_descriptor.restype = c
    lu.libusb_get_bus_number.argtypes = [p]
    lu.libusb_get_bus_number.restype = u8
    lu.libusb_get_device_address.argtypes = [p]
    lu.libusb_get_device_address.restype = u8
    lu.libusb_open.argtypes = [p, ctypes.POINTER(p)]
    lu.libusb_open.restype = c
    lu.libusb_close.argtypes = [p]
    lu.libusb_control_transfer.argtypes = [
        p, u8, u8, u16, u16, ctypes.POINTER(ctypes.c_ubyte), u16, ctypes.c_uint
    ]
    lu.libusb_control_transfer.restype = c


def dump_all():
    lu = load_libusb()
    declare(lu)
    ctx = ctypes.c_void_p()
    if lu.libusb_init(ctypes.byref(ctx)) != 0:
        raise SystemExit("libusb_init failed")
    lst = ctypes.POINTER(ctypes.c_void_p)()
    n = lu.libusb_get_device_list(ctx, ctypes.byref(lst))
    found = []
    for i in range(n):
        dev = lst[i]
        dd = DeviceDescriptor()
        if lu.libusb_get_device_descriptor(dev, ctypes.byref(dd)) != 0:
            continue
        if (dd.idVendor, dd.idProduct) != (VID, PID):
            continue
        bus = lu.libusb_get_bus_number(dev)
        addr = lu.libusb_get_device_address(dev)
        handle = ctypes.c_void_p()
        if lu.libusb_open(dev, ctypes.byref(handle)) != 0:
            print(f"bus {bus} addr {addr}: cannot open (permissions?)", file=sys.stderr)
            continue
        buf = (ctypes.c_ubyte * 512)()
        # GET_DESCRIPTOR(CONFIGURATION, 0) — ask for the whole tree.
        got = lu.libusb_control_transfer(
            handle, 0x80, 0x06, 0x0200, 0, buf, 512, 1000
        )
        lu.libusb_close(handle)
        if got > 0:
            found.append(((bus, addr), bytes(buf[:got])))
    lu.libusb_free_device_list(lst, 1)
    lu.libusb_exit(ctx)
    return found


CLASS = {1: "Audio", 3: "HID"}
SUB = {1: "AudioControl", 2: "AudioStreaming"}


def parse(data):
    """Walk the descriptor tree, one line per descriptor."""
    out, i = [], 0
    while i < len(data):
        length = data[i]
        if length == 0:
            break
        d = data[i : i + length]
        t = d[1]
        hexs = " ".join(f"{b:02x}" for b in d)
        if t == 0x02:
            note = f"CONFIG total={d[2] | d[3] << 8} ifaces={d[4]} power={d[8] * 2}mA"
        elif t == 0x04:
            note = (
                f"INTERFACE #{d[2]} alt={d[3]} eps={d[4]} "
                f"class={CLASS.get(d[5], d[5])} sub={SUB.get(d[6], d[6])}"
            )
        elif t == 0x05:
            note = (
                f"ENDPOINT addr=0x{d[2]:02x} attr=0x{d[3]:02x} "
                f"max={d[4] | d[5] << 8} interval={d[6]}"
            )
        elif t == 0x21:
            note = "HID descriptor"
        elif t == 0x24:
            note = f"CS_INTERFACE subtype={d[2]}"
        elif t == 0x25:
            note = f"CS_ENDPOINT subtype={d[2]} bmAttributes=0x{d[3]:02x}"
        elif t == 0x0B:
            note = "IAD"
        else:
            note = f"type 0x{t:02x}"
        out.append((note, hexs))
        i += length
    return out


def main(argv):
    devs = dump_all()
    if not devs:
        raise SystemExit("no TeslaMic found")
    raw_only = "--raw" in argv

    for (bus, addr), data in devs:
        print(f"=== bus {bus} addr {addr} — {len(data)} bytes ===")
        if raw_only:
            print(" ".join(f"{b:02x}" for b in data))
        else:
            for note, hexs in parse(data):
                print(f"  {note:<52} {hexs}")
        print()

    if len(devs) >= 2:
        a, b = devs[0][1], devs[1][1]
        print("=== diff ===")
        if a == b:
            print("  IDENTICAL — both devices present the same configuration bytes")
        else:
            print(f"  lengths: {len(a)} vs {len(b)}")
            pa, pb = parse(a), parse(b)
            for i in range(max(len(pa), len(pb))):
                x = pa[i] if i < len(pa) else ("<missing>", "")
                y = pb[i] if i < len(pb) else ("<missing>", "")
                if x != y:
                    print(f"  [{i}] A: {x[0]:<48} {x[1]}")
                    print(f"      B: {y[0]:<48} {y[1]}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

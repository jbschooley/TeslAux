# Real TeslaMic (clone) USB dump — 2026-07-14

Dumped from a working clone `TeslaMic` (VID `1235:0002`) on macOS via libusb.
This is the definitive replication target. Manufacturer string of this unit:
`TeslaMic_V004_FW_20220217Tes` (a different FW rev than the Reddit `T004_OTA_231008`).

## Raw descriptors (hex)

```
DEVICE  120110010000004035120200000101020301
CONFIG  0902a30004010080fa 090400000001010000 09240100012f000101
        0c2402040102000203000000 0a240605040101020200 07240506010500
        092403070101000600 090401000000010200 000904010101010200
        0007240107010100 0e2402010202100244ac0080bb00 09058409c000010000
        07250101000000 090402000103000000 09210102000122410007058103400001
        090403000003000000 092101020001222400
IF2_REPORT (65 B, HID keyboard)
        05010906a101050719e029e71500250175019508810295017508810195057501
        050819012905910295017503910195067508150026a400050719002aa4008100c0
IF3_REPORT (36 B, vendor)
        0600ff0aaa55a101150026ff007508960001090181029600010901910295080901b102c0
IF3 Feature report (GET_REPORT feature, 8 B)  0001000303000800
IF2 Input report (no key)  00
iSerialNumber (40 B, per-session random) e.g. 0E02070008181F0B8AFB76F93A8C1B1D472AA0AA0C2535898B4D6A4738FAD0057B6A12DB17320B75
```

## Decoded

### Device descriptor
bcdUSB **0x0110** · bMaxPacketSize0 64 · VID `1235` PID `0002` · bcdDevice `0x0100`
· iMfr/iProd/iSerial = 1/2/3 · 1 config

### Config (163 B, 4 interfaces, bus-powered, 500 mA)

**IF0 — AudioControl** (class 1/1, 0 ep). Topology:
Input Terminal **4** (Microphone, 2ch, L+R) → Feature Unit **5** (master **mute**,
ch1/ch2 **volume**) → Selector Unit **6** → Output Terminal **7** (USB streaming).
- HEADER: bcdADC 1.00, wTotalLength 47
- INPUT_TERMINAL: id 4, type 0x0201, 2ch, chcfg 0x0003
- FEATURE_UNIT: id 5, src 4, controlSize 1, controls [master 0x01, ch1 0x02, ch2 0x02]
- SELECTOR_UNIT: id 6, 1 pin, src 5
- OUTPUT_TERMINAL: id 7, type 0x0101, src 6

**IF1 — AudioStreaming** (class 1/2): alt0 zero-bandwidth, alt1 active.
- AS_GENERAL: bTerminalLink 7, bDelay 1, PCM
- FORMAT_TYPE_I: 2ch, 2 B/subframe, 16-bit, **2 sample rates: 44100 & 48000**
- EP `0x84` iso **adaptive** (attr 0x09), 192 B, interval 1, (bRefresh/bSynch 0)
- CS_EP: **sampling-frequency control** enabled (bmAttributes 0x01)

**IF2 — HID keyboard** (class 3/0): standard boot-keyboard report descriptor
(65 B), interrupt IN EP `0x81`, 64 B, interval 1. (The mic's physical button →
keystrokes.)

**IF3 — HID vendor** (class 3/0, **0 endpoints**): report descriptor 36 B —
Usage Page 0xFF00, top **Usage 0x55AA**, with a **256-B Input report**, **256-B
Output report**, and **8-B Feature report** (all Usage 0x01, report ID 0). This
is the config/OTA channel: the car writes `A5 5A`-framed data to the 256-B
**Output** report via `SET_REPORT (wValue 0x0200)`. Its **Feature report reads
back `00 01 00 03 03 00 08 00`**.

## What this means for the emulator (what we had WRONG)
- bcdUSB should be 0x0110 (we sent 0x0200) — minor.
- **IF2 must be a real HID keyboard** (65-B descriptor) — we sent an 8-B vendor blob.
- **IF3 must be endpoint-less** with the exact 36-B vendor descriptor + 256/256/8
  In/Out/Feature reports — we gave it an interrupt-IN endpoint + 21-B stub.
- Audio needs the **Feature Unit + Selector Unit** and terminal IDs 4/5/6/7, plus
  **two sample rates** and sampling-freq control — we had a bare mic→USB pair.
- Add a serial number string; return the IF3 Feature report `0001000303000800`.

The car validates these descriptors (esp. IF3's), so matching them exactly is the
path to defeating the popup. See the plan being implemented in `src/main.rs`.

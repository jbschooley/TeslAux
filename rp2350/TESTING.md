# Testing the RP2350 build

Three binaries, in the order they are worth running. Each one is only meaningful
if the one before it passed — they were written in this order for that reason,
and every bug found so far was found by the cheapest test that could see it.

Build them all with `./build-uf2.sh`, then drag the `.uf2` onto the board while
holding BOOT.

## Host tests first

```sh
cargo test --lib --target aarch64-apple-darwin
```

The codec and packet layers are pure logic, so they run on the development
machine: NRZI, bit stuffing, CRC5, CRC16, PID and token framing. 24 tests, and
they cost nothing to run.

They have already earned it once. The bit-stuffing test was originally written
to assert no more than six identical symbols in a row, and a *correct* encoder
emits seven — the rule is six consecutive 1 bits, and in NRZI a 1 is the absence
of a transition, so the worst case is a 0 transitioning the line in, six 1s
holding it, then the stuffed 0. That misunderstanding was heading for the
receive path.

## `LEDTEST` — does the chip port work?

Cycles the WS2812 on GPIO16. Proves thumbv8m, the Cortex-M33, the RP2350 memory
map, and above all the IMAGE_DEF block that `memory.x` places at the base of
flash. Those fail in ways that look identical from outside — the board sits in
BOOTSEL and says nothing — so they are worth separating from everything else.

Its colours are **not** reliable: this board's WS2812 appears to take RGB where
the RP2040-Zero's takes GRB, so `ledtest` has probably been showing swapped
colours all along. A cycle of four colours in the wrong order still looks like a
cycle of four colours, which is why nobody noticed. Nothing depends on this;
`pioloop` reports over serial precisely so it does not have to.

## `PIOLOOP` — does the line coding work?

The transmitter drives D+/D-; the receive state machines read the same pads.
Reports over the **native** USB port, which is free because the PIO port is the
one under test:

```sh
screen /dev/tty.usbmodem* 115200
```

Five payloads, chosen for what they do to the coding rather than their content:
empty (a control status stage), a short run, all-ones (the bit-stuffing worst
case), all-zeroes (a transition on every bit), and 192 bytes — this project's
iso IN endpoint size, and three times what Pico-PIO-USB's buffers hold.

All five should return `OK`. They do.

That covers the encoder, the PIO transmitter, the edge detector, the NRZI
decoder, de-stuffing and CRC. It does **not** cover clock recovery: in loopback
both ends derive from `clk_sys` and are perfectly synchronous, so the edge
detector's six-instruction resync window — the entire mechanism for tracking a
host's independent crystal — is never put under any strain.

## `PIOCAP` — what is actually on the wire?

A logic analyser built from the spare PIO block. PIO2 samples D+/D- at 96 MHz,
eight samples per full-speed bit, into RAM by DMA, and dumps the waveform over
the same serial port alongside what the receiver made of it.

Reach for this when `PIOLOOP` fails and the decoded bytes have stopped being
informative. Decoding it by hand is slow but it is ground truth, and it settled
three questions that no amount of reading the code had.

Read the capture **self-calibrating**: the bus is idle J for most of the window,
so whichever sample value dominates is J, the opposite is K, and a value with
both bits set is SE0 — because `configure_pins` inverts the input path on both
pads and that applies to the sampler too. Two bit times of SE0 is the EOP.

A note on trusting instruments: the first version of this polled the FIFO
instead of using DMA, and could not keep up with a word every 167 ns. It did not
fail visibly — it produced plausible-looking output with samples that were two
fragments either side of a dropped run. If you ever see a sample that is neither
J, K nor SE0, the capture is broken, not the bus.

## What is still untested

* **Clock recovery**, as above. The first real test is plugging the PIO port
  into a computer and seeing whether it enumerates.
* **Framing without foreknowledge.** `bus::prepare_decoder` is currently called
  by the same CPU that schedules the transmission. A host sends whenever it
  likes, so that reset has to move into the interrupt path off `IRQ_RX_START`.
* **Everything above the wire.** There is no endpoint machinery and no
  `embassy_usb_driver::Driver` implementation yet.

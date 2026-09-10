// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! Loopback self-test for the PIO USB line coding, reporting over serial.
//!
//! The transmitter drives D+/D-; the two receive state machines read those same
//! pads. Pointing them at each other on one chip exercises the whole coding
//! chain — symbol packing, the TX program, the edge detector, the NRZI decoder,
//! de-stuffing — with no instruments needed.
//!
//! # Why serial and not the LED
//!
//! The first version reported pass/fail as LED colours, and what came back was
//! "green, then white flashes" — a combination this code cannot produce, since
//! green means zero failures and the flashes only run when there are some. The
//! LED driver emits GRB, which is correct for the RP2040-Zero, so the likely
//! reading is that this board's part takes RGB and every colour has been
//! swapped since `ledtest`.
//!
//! Rather than guess a channel order and keep reading tea leaves, this reports
//! over the **native** USB port — which is free, the whole point being that the
//! PIO port is the one under test. Plug the Type-C_1 (programming) port into a
//! computer and open the serial device:
//!
//! ```text
//! screen /dev/tty.usbmodem* 115200
//! ```
//!
//! The LED is kept only as a crude "is it alive" indicator, showing white,
//! which is immune to channel order. Its colours are not load-bearing any more.
//!
//! # What a pass here would not mean
//!
//! TX and RX derive from the same `clk_sys`, so they are perfectly synchronous.
//! Recovering a host's *independent* crystal is what the edge detector's
//! resynchronisation is for, and this never puts it under any strain. It also
//! says nothing about the bus turnaround, the pull-up, or anything electrical.

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, PIO1, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use smart_leds::RGB8;

use teslamic_rp2350::pio_usb::{bus, codec, packet, SYSCLK_HZ};

#[path = "../../../rp2040/src/ws2812.rs"]
mod ws2812;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

/// Payloads to send. Chosen for what they do to the *coding*, not the content:
/// all-ones is the bit-stuffing worst case, all-zeroes transitions on every
/// bit, zero-length is a control status stage, and 192 bytes is this project's
/// actual iso IN endpoint.
const CASES: [&[u8]; 5] = [
    &[],
    &[0x00, 0x01, 0x02, 0x03],
    &[0xFF; 16],
    &[0x00; 16],
    &[0x5A; 192],
];

const SYM_BYTES: usize = 1024;
const RAW_BYTES: usize = 200;

/// A fixed-size line buffer, so `write!` works without an allocator.
struct Line {
    buf: [u8; 256],
    len: usize,
}

impl Line {
    const fn new() -> Self {
        Self {
            buf: [0; 256],
            len: 0,
        }
    }
    fn clear(&mut self) {
        self.len = 0;
    }
    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

impl core::fmt::Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            if self.len < self.buf.len() {
                self.buf[self.len] = b;
                self.len += 1;
            }
        }
        Ok(())
    }
}

/// Write a line to the host in 64-byte packets, dropping it if the host went
/// away. A diagnostic that blocks forever is worse than one that stops.
async fn emit<'d>(class: &mut CdcAcmClass<'d, UsbDriver<'d, USB>>, line: &Line) {
    let bytes = line.as_bytes();
    for chunk in bytes.chunks(64) {
        if class.write_packet(chunk).await.is_err() {
            return;
        }
    }
    // A full final packet needs a zero-length one after it or the host waits.
    if !bytes.is_empty() && bytes.len() % 64 == 0 {
        let _ = class.write_packet(&[]).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // The PIO dividers are computed against this; anything else and the bus
    // runs at the wrong bit rate.
    let p = embassy_rp::init(bus::clock_config());

    // Alive indicator on PIO1. PIO0 is filled exactly by the three USB
    // programs, with no room for a WS2812 driver beside them.
    let Pio {
        common: mut led_common,
        sm0: led_sm,
        ..
    } = Pio::new(p.PIO1, Irqs);
    let mut led = ws2812::Ws2812::new(&mut led_common, led_sm, p.PIN_16);
    led.set(RGB8::new(16, 16, 16)); // white: immune to channel order

    let driver = UsbDriver::new(p.USB, Irqs);
    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xca7e);
    usb_config.manufacturer = Some("TeslAux");
    usb_config.product = Some("PIO USB loopback");
    usb_config.serial_number = Some("pioloop");
    usb_config.max_power = 100;
    usb_config.max_packet_size_0 = 64;

    let mut config_descriptor = [0u8; 256];
    let mut bos_descriptor = [0u8; 256];
    let mut control_buf = [0u8; 64];
    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();

    let run_usb = usb.run();

    let test = async {
        // Nothing is worth saying until someone is listening.
        class.wait_connection().await;
        Timer::after_millis(100).await;

        let mut line = Line::new();

        let Pio {
            mut common,
            mut sm0,
            mut sm1,
            mut sm2,
            irq_flags,
            ..
        } = Pio::new(p.PIO0, Irqs);

        // D+ is gp13 and D- is gp12 — the opposite of the schematic's apparent
        // reading, settled by the board itself. See bus.rs.
        let mut dp = common.make_pio_pin(p.PIN_13);
        let mut dm = common.make_pio_pin(p.PIN_12);
        bus::configure_pins(&mut dp, &mut dm);

        let order = if dp.pin() < dm.pin() {
            teslamic_rp2350::pio_usb::tx::PinOrder::DpBelowDm
        } else {
            teslamic_rp2350::pio_usb::tx::PinOrder::DmBelowDp
        };
        let programs = bus::load(&mut common, order);

        line.clear();
        let _ = write!(
            line,
            "\r\n=== PIO USB loopback ===\r\nclk_sys {} Hz (want {})\r\ntx@{} edge@{} decoder@{}\r\nD+ gp{}  D- gp{}\r\n",
            embassy_rp::clocks::clk_sys_freq(),
            SYSCLK_HZ,
            programs.tx.origin,
            programs.edge.origin,
            programs.decoder.origin,
            dp.pin(),
            dm.pin(),
        );
        emit(&mut class, &line).await;

        // At rest the bus should be idle-J: D+ high (the board's 1.5k pull-up),
        // D- low. The edge detector reads D- *inverted*, so it should see high
        // and sit waiting. If it is instead free-running, the inversion is the
        // first thing to doubt — hence dumping the registers rather than
        // reasoning about them.
        for (name, pin) in [("D+", dp.pin()), ("D-", dm.pin())] {
            let (ctrl, status) = bus::pin_registers(pin);
            line.clear();
            let _ = write!(
                line,
                "{name} gp{pin}: ctrl {ctrl:08x} funcsel {} inover {}  status {status:08x} pad {}\r\n",
                ctrl & 0x1f,
                (ctrl >> 16) & 0x3,
                (status >> 17) & 1,
            );
            emit(&mut class, &line).await;
        }

        bus::init_tx(&mut sm0, &programs.tx, &dp, &dm);
        bus::init_edge_detector(&mut sm1, &programs.edge, &dp, &dm);
        bus::init_decoder(&mut sm2, &programs.decoder, &dp);
        bus::preload_decoder_osr(&mut sm2);

        sm1.set_enable(true);
        sm2.set_enable(true);
        sm0.set_enable(true);

        let mut raw = [0u8; RAW_BYTES];
        let mut syms = [0u8; SYM_BYTES];
        let mut back = [0u8; RAW_BYTES];

        for (idx, payload) in CASES.iter().enumerate() {
            let n = packet::build_data(packet::Pid::Data0, payload, &mut raw).unwrap();
            let sym_bytes = {
                let mut buf = codec::SymbolBuf::new(&mut syms);
                codec::encode(&raw[..n], &mut buf).unwrap();
                buf.bytes()
            };

            // Put the receiver in a known state: the decoder byte-aligned from
            // this packet's first bit, the edge detector waiting for its start,
            // and no stale trigger to consume a phantom bit.
            bus::prepare_decoder(&mut sm2, &programs.decoder);
            bus::restart_edge_detector(&mut sm1, &programs.edge);
            bus::clear_rx_irqs(&irq_flags);
            back.fill(0);
            // Enable the line drivers. Without this the symbols clock out onto
            // pins that are still inputs and the bus never moves.
            bus::tx_start(&mut sm0, &programs.tx, programs.order);

            let mut got = 0usize;
            let mut fed = 0usize;
            let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_millis(50);
            while embassy_time::Instant::now() < deadline {
                if fed < sym_bytes && bus::try_push_symbol(&mut sm0, syms[fed]) {
                    fed += 1;
                }
                while let Some(w) = sm2.rx().try_pull() {
                    if got < back.len() {
                        // A right-shifting ISR with an 8-bit threshold leaves
                        // the byte in the top octet.
                        back[got] = (w >> 24) as u8;
                    }
                    got += 1;
                }
                if fed >= sym_bytes && got >= n {
                    break;
                }
            }

            // A static bus has a signature. With no transitions the decoder
            // reads six bits as "unchanged" (a 1), then the stuff counter trips
            // and flips its state reference, so the seventh reads as a 0 — a
            // repeating seven-bit pattern, identical whatever was "sent". It is
            // worth naming, because it looks like data and it is what a
            // transmitter that never enabled its line drivers produces.
            const STATIC_BUS: [u8; 7] = [0x7e, 0xbf, 0xdf, 0xef, 0xf7, 0xfb, 0xfd];
            let bus_static =
                got > 8 && back[..got.min(back.len())].iter().all(|b| STATIC_BUS.contains(b));

            let first_bad = (0..n.min(got)).find(|&i| back[i] != raw[i]);
            line.clear();
            let _ = write!(
                line,
                "\r\ncase {} payload {:>3}  sent {:>3}B / {:>3} sym-bytes  got {:>3}B  ",
                idx,
                payload.len(),
                n,
                sym_bytes,
                got
            );
            match (got, first_bad) {
                (0, _) => {
                    let _ = write!(line, "SILENT\r\n");
                }
                _ if bus_static => {
                    let _ = write!(line, "BUS STATIC (transmitter never drove)\r\n");
                }
                (_, Some(i)) => {
                    let _ = write!(line, "MISMATCH at {i}\r\n");
                }
                (_, None) if got >= n => {
                    let _ = write!(line, "OK\r\n");
                }
                (_, None) => {
                    let _ = write!(line, "SHORT\r\n");
                }
            }
            emit(&mut class, &line).await;

            // Dump both in full (to a limit), not truncated to the expected
            // length. Bytes arriving *after* the expected end are as diagnostic
            // as the ones before it: they say the receiver kept decoding past
            // the EOP, which is a framing fault rather than a coding one.
            for (label, data) in [
                ("  sent", &raw[..n.min(32)]),
                ("  got ", &back[..got.min(back.len()).min(32)]),
            ] {
                line.clear();
                let _ = write!(line, "{label}");
                for b in data {
                    let _ = write!(line, " {b:02x}");
                }
                let _ = write!(line, "\r\n");
                emit(&mut class, &line).await;
            }

            // The first symbol-bytes actually handed to the state machine. The
            // encoder is unit-tested on the host, so this is not checking the
            // encoding — it is checking that what the FIFO received is what the
            // encoder produced, which the feed loop could get wrong on its own.
            line.clear();
            let _ = write!(line, "  syms");
            for b in syms.iter().take(12) {
                let _ = write!(line, " {b:02x}");
            }
            let _ = write!(line, "\r\n");
            emit(&mut class, &line).await;
        }

        line.clear();
        let _ = write!(line, "\r\n=== done ===\r\n");
        emit(&mut class, &line).await;

        loop {
            Timer::after_millis(1000).await;
        }
    };

    join(run_usb, test).await;
}

// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! Drive whole transactions against the device engine, on one chip.
//!
//! The CPU plays host: it transmits a token, and [`pio_usb::txn::Engine`] —
//! reading through [`pio_usb::hw::HwPort`], the one module no host test can
//! reach — decides and replies. PIO2 captures both halves of the exchange, so
//! the reply can be decoded by hand rather than inferred from an outcome code.
//!
//! This is as close to a real host as one board gets. What it exercises that
//! loopback did not: token decoding, the address filter, `respond`'s dispatch,
//! and a reply actually reaching the wire — including the two cases where the
//! correct reply is *nothing*, which an outcome code cannot distinguish from a
//! reply that was dropped.
//!
//! # What it cannot show
//!
//! **Timing.** A real host's token arrives unannounced and its turnaround
//! window is 167 to 625 ns; here the CPU sends the token and then waits for it
//! at its leisure.
//!
//! **The handshake wait.** A bulk IN waits for the host's ACK and an
//! isochronous IN does not, but a wait is silence and silence looks like idle,
//! so the two produce identical captures. Nor can this harness answer its own
//! question: the CPU is the host *and* the device, so it cannot send an ACK
//! while it is inside `handle` waiting for one. Both cases therefore report
//! `acked: false` — one because the host said nothing, the other because it
//! never listened.
//!
//! That difference is covered by the host tests in `txn`, which can script an
//! ACK into the byte stream. On hardware it needs a real host, which is the
//! next milestone regardless.
//!
//! ```text
//! screen /dev/tty.usbmodem* 115200
//! ```

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, PIO1, PIO2, USB};
use embassy_rp::pio::{
    Config as PioConfig, FifoJoin, InterruptHandler as PioInterruptHandler, Pio, ShiftConfig,
    ShiftDirection,
};
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use fixed::traits::ToFixed;
use smart_leds::RGB8;

use teslamic_rp2350::board;
use teslamic_rp2350::pio_usb::{
    bus,
    device::{EpState, EpType},
    endpoint::{words_for_payload, Handshakes, PacketBuf},
    hw::HwPort,
    packet::{encode_token, Pid, SYNC},
    txn::{Engine, Port},
};

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
    PIO2_IRQ_0 => PioInterruptHandler<PIO2>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

/// The address the device answers to during the test. Not zero, so the
/// address filter is actually doing something.
const ADDR: u8 = 7;
/// 16 samples per word at 96 MHz: 1024 words is about 170 µs.
///
/// The first version used 384 words — 64 µs — and closed before the reply.
/// Everything between the token and the reply has to fit inside the window,
/// and the gap is set by how long the CPU takes to notice the token, which is
/// not a quantity to be optimistic about.
const CAPTURE_WORDS: usize = 1024;

const PAYLOAD: [u8; 8] = [0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04];

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

async fn emit<'d>(class: &mut CdcAcmClass<'d, UsbDriver<'d, USB>>, line: &Line) {
    let bytes = line.as_bytes();
    for chunk in bytes.chunks(64) {
        if class.write_packet(chunk).await.is_err() {
            return;
        }
    }
    if !bytes.is_empty() && bytes.len() % 64 == 0 {
        let _ = class.write_packet(&[]).await;
    }
}

/// One thing to try: a token, and what the device should make of it.
struct Case {
    name: &'static str,
    pid: Pid,
    address: u8,
    endpoint: u8,
    /// Whether the endpoint has a packet queued.
    ready: bool,
    ep_type: EpType,
}

const CASES: [Case; 5] = [
    Case {
        name: "IN, nothing queued -> NAK",
        pid: Pid::In,
        address: ADDR,
        endpoint: 1,
        ready: false,
        ep_type: EpType::Bulk,
    },
    Case {
        name: "IN, packet queued -> DATA0",
        pid: Pid::In,
        address: ADDR,
        endpoint: 1,
        ready: true,
        ep_type: EpType::Bulk,
    },
    Case {
        name: "IN for another address -> silence",
        pid: Pid::In,
        address: ADDR + 1,
        endpoint: 1,
        ready: true,
        ep_type: EpType::Bulk,
    },
    Case {
        name: "iso IN, nothing queued -> silence",
        pid: Pid::In,
        address: ADDR,
        endpoint: 1,
        ready: false,
        ep_type: EpType::Isochronous,
    },
    Case {
        name: "iso IN, packet queued -> DATA0, no handshake",
        pid: Pid::In,
        address: ADDR,
        endpoint: 1,
        ready: true,
        ep_type: EpType::Isochronous,
    },
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(bus::clock_config());

    let Pio {
        common: mut led_common,
        sm0: led_sm,
        ..
    } = Pio::new(p.PIO1, Irqs);
    let mut led = ws2812::Ws2812::new(&mut led_common, led_sm, p.PIN_16);
    led.set(RGB8::new(16, 16, 16));

    let driver = UsbDriver::new(p.USB, Irqs);
    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xca80);
    usb_config.manufacturer = Some("TeslAux");
    usb_config.product = Some("PIO USB transactions");
    usb_config.serial_number = Some("piotxn");
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

    let work = async {
        class.wait_connection().await;
        Timer::after_millis(100).await;
        let mut line = Line::new();

        // --- the USB port itself ---
        let Pio {
            mut common,
            mut sm0,
            mut sm1,
            mut sm2,
            irq_flags,
            ..
        } = Pio::new(p.PIO0, Irqs);

        let (dp_pin, dm_pin) = teslamic_rp2350::usb_pins!(p);
        let mut dp = common.make_pio_pin(dp_pin);
        let mut dm = common.make_pio_pin(dm_pin);
        bus::configure_pins(&mut dp, &mut dm);
        let order = board::pin_order();
        let programs = bus::load(&mut common, order);
        programs.check_tx_offset();
        bus::init_tx(&mut sm0, &programs.tx, &dp, &dm);
        bus::init_edge_detector(&mut sm1, &programs.edge, &dp, &dm);
        bus::init_decoder(&mut sm2, &programs.decoder, &dp);
        sm0.set_enable(true);

        // --- sampler on PIO2, as in piocap ---
        let Pio {
            common: mut cap_common,
            sm0: mut cap_sm,
            ..
        } = Pio::new(p.PIO2, Irqs);
        let cap_prg = cap_common.load_program(
            &pio::pio_asm!(".wrap_target", "    in pins, 2", ".wrap").program,
        );
        let mut cfg = PioConfig::default();
        cfg.use_program(&cap_prg, &[]);
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Right,
        };
        cfg.fifo_join = FifoJoin::RxOnly;
        cfg.clock_divider = 1u8.to_fixed();
        cap_sm.set_config(&cfg);
        embassy_rp::pac::PIO2
            .sm(0)
            .pinctrl()
            .modify(|w| w.set_in_base(dm.pin()));

        // The engine talks to the hardware through this, not through embassy's
        // typed handles — it is the module a host test cannot reach, which is
        // the whole reason this binary exists.
        let mut port = HwPort::new(
            embassy_rp::pac::PIO0,
            0,
            0,
            2,
            1,
            1, // DMA channel 1; channel 0 is the sampler's
            programs.tx.origin as u16,
            programs.decoder.origin as u16,
            programs.edge.origin as u16,
            order,
        );

        let handshakes = Handshakes::new();
        let mut packet: PacketBuf<12, { words_for_payload(8) }> = PacketBuf::new();
        packet.prepare(Pid::Data0, &PAYLOAD).unwrap();

        line.clear();
        let _ = write!(
            line,
            "\r\n=== PIO USB transactions ===\r\nclk_sys {} Hz  device address {}\r\ntx@{} edge@{} decoder@{}\r\n",
            embassy_rp::clocks::clk_sys_freq(),
            ADDR,
            programs.tx.origin,
            programs.edge.origin,
            programs.decoder.origin,
        );
        emit(&mut class, &line).await;

        let mut capture = [0u32; CAPTURE_WORDS];
        // Reborrowed per case: each transfer borrows the channel for its life.
        let mut cap_dma = p.DMA_CH0;

        for case in &CASES {
            // The engine re-arms the receiver itself at the end of every
            // transaction now, so this is only needed to get the *first* one
            // started — before anything has run, nothing has armed it.
            bus::prepare_decoder(&mut sm2, &programs.decoder);
            bus::restart_edge_detector(&mut sm1, &programs.edge);
            bus::clear_rx_irqs(&irq_flags);

            // Build the token this "host" is about to send.
            let [b0, b1] = encode_token(case.address, case.endpoint);
            let token_raw = [SYNC, case.pid.to_byte(), b0, b1];
            let mut token: PacketBuf<4, { words_for_payload(0) }> = PacketBuf::new();
            // `prepare` builds a data packet; a token is raw bytes plus an EOP,
            // so encode it directly instead.
            let token_words = encode_raw(&token_raw, &mut token);

            cap_sm.set_enable(true);
            let cap = cap_sm.rx().dma_pull(cap_dma.reborrow(), &mut capture, false);

            port.transmit(token_words);
            // Let the token finish and be decoded before the engine looks for
            // it. A real host does not wait, but a real host is also not the
            // same CPU.
            //
            // Blocking, not `Timer::after`. Awaiting yields to the executor,
            // which runs the USB task, which can take far longer than the delay
            // asked for — long enough that the capture window closed before the
            // reply and every exchange looked like a token with no answer.
            while port.tx_busy() {}
            embassy_time::block_for(embassy_time::Duration::from_micros(10));

            let mut engine = Engine {
                address: ADDR,
                frame: 0,
            };
            let ep_type = case.ep_type;
            let ready = case.ready;
            let mut rx_buf = [0u8; 64];
            let outcome = engine.handle(
                &mut port,
                &handshakes,
                |_, _| {
                    Some(EpState {
                        ready,
                        ..EpState::new(ep_type)
                    })
                },
                |_| Some(packet.symbols()),
                &mut rx_buf,
            );

            // Let the reply drain, then stop sampling. Blocking for the same
            // reason as above: this is inside the capture window.
            while port.tx_busy() {}
            embassy_time::block_for(embassy_time::Duration::from_micros(20));
            let _ = embassy_time::with_timeout(embassy_time::Duration::from_millis(5), cap).await;
            cap_sm.set_enable(false);

            line.clear();
            let _ = write!(line, "\r\n{}\r\n  outcome {:?}\r\n", case.name, outcome);
            emit(&mut class, &line).await;

            // Only the part of the capture with anything in it: the idle either
            // side is not worth the serial time.
            let idle = capture[0];
            let first = capture.iter().position(|w| *w != idle).unwrap_or(0);
            let last = capture
                .iter()
                .rposition(|w| *w != idle)
                .unwrap_or(CAPTURE_WORDS - 1);
            let from = first.saturating_sub(1);
            let to = (last + 2).min(CAPTURE_WORDS);
            line.clear();
            let _ = write!(
                line,
                "  wire w{from}..w{to} of {CAPTURE_WORDS} (idle {idle:08x}){}\r\n",
                if to >= CAPTURE_WORDS - 2 {
                    "  TRUNCATED - window too short"
                } else {
                    ""
                }
            );
            emit(&mut class, &line).await;
            for chunk in capture[from..to].chunks(8) {
                line.clear();
                let _ = write!(line, "  ");
                for w in chunk {
                    let _ = write!(line, "{w:08x} ");
                }
                let _ = write!(line, "\r\n");
                emit(&mut class, &line).await;
            }
        }

        line.clear();
        let _ = write!(line, "\r\n=== done ===\r\n");
        emit(&mut class, &line).await;
        loop {
            Timer::after_millis(1000).await;
        }
    };

    join(run_usb, work).await;
}

/// Encode raw packet bytes — a token, which has no CRC16 — into a buffer's
/// symbol words, returning them.
///
/// [`PacketBuf::prepare`] builds *data* packets, which is not what a token is.
/// This only exists because this binary has to impersonate a host; the driver
/// itself never transmits a token.
fn encode_raw<'a, const RAW: usize, const WORDS: usize>(
    raw: &[u8],
    buf: &'a mut PacketBuf<RAW, WORDS>,
) -> &'a [u32] {
    buf.encode_raw(raw).expect("a token always fits");
    buf.symbols()
}

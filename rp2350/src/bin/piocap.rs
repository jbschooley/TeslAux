// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! A logic analyser for D+/D-, built from a spare PIO block.
//!
//! Four rounds of inferring the receiver's behaviour from the bytes it produced
//! found three real bugs and then stopped paying. The transmit path is now
//! known good — the symbol stream reaching the FIFO matches what the host-side
//! encoder produces, byte for byte — so the remaining fault is in the receive
//! programs or their configuration, and that is not something the decoded
//! output can localise.
//!
//! So: sample the two lines directly. PIO0 runs only the transmitter here; PIO2
//! samples D+ and D- at 96 MHz — eight samples per full-speed bit — into RAM,
//! and the capture is dumped over the native USB port for decoding off-chip.
//!
//! Two bits per sample, sixteen samples per 32-bit word, `CAPTURE_WORDS` words:
//! at 96 MHz that is about 42 µs, against roughly 3 µs for the smallest test
//! packet. Plenty of room either side of the packet to see the idle state, the
//! start of packet and the EOP.
//!
//! **The capture is DMA'd, not polled.** The FIFO fills at 6 MHz — a word every
//! 167 ns, about sixteen CPU cycles — so a polling loop cannot keep up and the
//! FIFO simply overflows. The first version of this did poll, and the giveaway
//! was samples reading `11`: both lines high is not a legal bus state, so those
//! were two fragments either side of a gap being read as one sample.
//!
//! Output is raw hex words, oldest first. Within a word the *oldest* sample is
//! in the low bits, because the ISR shifts right.
//!
//! Rather than depend on getting the bit order right, read the capture
//! self-calibrating: the bus is idle **J** for most of the window, so whichever
//! sample value dominates *is* J, the opposite value is K, and `00` is SE0.
//! A sample of both-bits-set cannot happen on a working bus.
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

use teslamic_rp2350::pio_usb::{bus, codec, packet, tx};

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

/// 16 samples per word at 96 MHz: 256 words is about 42 µs.
const CAPTURE_WORDS: usize = 256;

/// The packet to send while capturing. Deliberately the smallest case — its
/// whole transmission fits inside the capture window with idle either side.
const PAYLOAD: &[u8] = &[];

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
    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xca7f);
    usb_config.manufacturer = Some("TeslAux");
    usb_config.product = Some("PIO USB capture");
    usb_config.serial_number = Some("piocap");
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

        // --- transmitter on PIO0, nothing else ---
        let Pio {
            common: mut tx_common,
            sm0: mut tx_sm,
            sm1: mut edge_sm,
            sm2: mut dec_sm,
            irq_flags,
            ..
        } = Pio::new(p.PIO0, Irqs);

        let mut dp = tx_common.make_pio_pin(p.PIN_13);
        let mut dm = tx_common.make_pio_pin(p.PIN_12);
        bus::configure_pins(&mut dp, &mut dm);
        let order = tx::PinOrder::DmBelowDp;
        let programs = bus::load(&mut tx_common, order);
        let tx_prg = &programs.tx;
        bus::init_tx(&mut tx_sm, tx_prg, &dp, &dm);
        // The receiver runs too, so one packet yields both the wire and what
        // the decoder made of it. Aligning those is the only way left to find
        // where decoding diverges: the wire is known good, the framing flags
        // say the packet is delimited correctly, and the decoder still emits
        // more bits than there are bit times to carry them.
        bus::init_edge_detector(&mut edge_sm, &programs.edge, &dp, &dm);
        bus::init_decoder(&mut dec_sm, &programs.decoder, &dp);
        bus::prepare_decoder(&mut dec_sm, &programs.decoder);
        bus::restart_edge_detector(&mut edge_sm, &programs.edge);
        bus::clear_rx_irqs(&irq_flags);
        tx_sm.set_enable(true);

        // --- sampler on PIO2 ---
        //
        // A separate block so it cannot compete with the transmitter for
        // instruction space, and so this stays usable once the receive programs
        // come back into PIO0.
        let Pio {
            common: mut cap_common,
            sm0: mut cap_sm,
            ..
        } = Pio::new(p.PIO2, Irqs);

        // The pads belong to PIO0, which drives them. Every PIO block sees
        // every GPIO input regardless of which one owns the pad, so the sampler
        // can read them — but embassy's `set_in_pins` wants typed pin handles
        // that PIO0 already took. Setting this state machine's IN base through
        // the PAC is the way round it, and it is the same kind of poke the
        // input inversion needs.
        let cap_prg = cap_common.load_program(
            &pio::pio_asm!(
                // One sample per cycle. `in pins, 2` reads from the IN base
                // upwards, so with the base at D- this packs (dp << 1) | dm.
                ".wrap_target",
                "    in pins, 2",
                ".wrap",
            )
            .program,
        );

        let mut cfg = PioConfig::default();
        cfg.use_program(&cap_prg, &[]);
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Right,
        };
        cfg.fifo_join = FifoJoin::RxOnly;
        // Undivided: 96 MHz, eight samples per full-speed bit.
        cfg.clock_divider = 1u8.to_fixed();
        cap_sm.set_config(&cfg);
        // IN base = D- (the lower pin), so `in pins, 2` packs (dp << 1) | dm.
        // No pin dirs to set: a PIO reading an input never needs to own the pad.
        embassy_rp::pac::PIO2
            .sm(0)
            .pinctrl()
            .modify(|w| w.set_in_base(dm.pin()));

        // --- encode the packet before starting anything timing-sensitive ---
        let mut raw = [0u8; 64];
        let n = packet::build_data(packet::Pid::Data0, PAYLOAD, &mut raw).unwrap();
        let mut syms = [0u8; 256];
        let mut sym_words = [0u32; 256];
        let sym_bytes = {
            let mut buf = codec::SymbolBuf::new(&mut syms);
            codec::encode(&raw[..n], &mut buf).unwrap();
            buf.bytes()
        };

        bus::expand_symbols(&syms[..sym_bytes], &mut sym_words).unwrap();

        line.clear();
        let _ = write!(
            line,
            "\r\n=== D+/D- capture ===\r\nclk_sys {} Hz  sampling 96 MHz, 8/bit\r\nD+ gp{} D- gp{}\r\npacket {}B, {} symbol-bytes\r\n",
            embassy_rp::clocks::clk_sys_freq(),
            dp.pin(),
            dm.pin(),
            n,
            sym_bytes
        );
        emit(&mut class, &line).await;
        line.clear();
        let _ = write!(line, "raw ");
        for b in &raw[..n] {
            let _ = write!(line, "{b:02x} ");
        }
        let _ = write!(line, "\r\nsyms ");
        for b in syms.iter().take(sym_bytes.min(24)) {
            let _ = write!(line, "{b:02x} ");
        }
        let _ = write!(line, "\r\n");
        emit(&mut class, &line).await;

        // --- capture ---
        //
        // DMA, because the FIFO produces a word every 167 ns and the CPU cannot
        // drain that by polling. The state machine is enabled first: its FIFO is
        // eight deep, which is far more slack than the few instructions before
        // the transfer starts, so nothing is lost at the seam.
        let mut capture = [0u32; CAPTURE_WORDS];
        cap_sm.set_enable(true);
        let transfer = cap_sm.rx().dma_pull(p.DMA_CH0, &mut capture, false);

        bus::tx_start(&mut tx_sm, tx_prg, order);

        // Feed the whole packet without stopping to check anything. Blocking on
        // a full FIFO is fine here — the DMA is running independently.
        let mut decoded = [0u8; 64];
        let mut got = 0usize;
        // DMA, so no interrupt can starve the transmitter mid-packet.
        let feed = bus::dma_symbols(&mut tx_sm, p.DMA_CH1, &sym_words[..sym_bytes]);
        feed.await;
        // Let the tail of the packet and the EOP land.
        let drain_until = embassy_time::Instant::now() + embassy_time::Duration::from_micros(500);
        while embassy_time::Instant::now() < drain_until {
            while let Some(w) = dec_sm.rx().try_pull() {
                if got < decoded.len() {
                    decoded[got] = (w >> 24) as u8;
                }
                got += 1;
            }
        }

        // 256 words at 6 MHz is ~42 us; a timeout only matters if the sampler
        // never started.
        let filled =
            embassy_time::with_timeout(embassy_time::Duration::from_millis(100), transfer)
                .await
                .is_ok();
        cap_sm.set_enable(false);

        line.clear();
        let _ = write!(
            line,
            "captured {CAPTURE_WORDS} words{} (oldest sample in low bits)\r\nidle J is whichever value dominates\r\n",
            if filled { "" } else { " (TIMED OUT - sampler never ran)" }
        );
        emit(&mut class, &line).await;

        line.clear();
        let _ = write!(
            line,
            "decoded {}B  sop {} eop {}\r\n  ",
            got,
            irq_flags.check(teslamic_rp2350::pio_usb::rx::IRQ_RX_START as u8) as u8,
            irq_flags.check(teslamic_rp2350::pio_usb::rx::IRQ_RX_EOP as u8) as u8
        );
        for b in decoded.iter().take(got.min(24)) {
            let _ = write!(line, "{b:02x} ");
        }
        let _ = write!(line, "\r\n");
        emit(&mut class, &line).await;

        for chunk in capture[..CAPTURE_WORDS].chunks(8) {
            line.clear();
            for w in chunk {
                let _ = write!(line, "{w:08x} ");
            }
            let _ = write!(line, "\r\n");
            emit(&mut class, &line).await;
        }

        line.clear();
        let _ = write!(line, "=== done ===\r\n");
        emit(&mut class, &line).await;

        loop {
            Timer::after_millis(1000).await;
        }
    };

    join(run_usb, work).await;
}

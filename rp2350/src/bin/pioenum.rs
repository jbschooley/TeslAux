// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! Stand the PIO USB port up as a real device and let a computer enumerate it.
//!
//! Everything until now has been this board talking to itself. This is the
//! first test with a host on the other end, and it exercises at once the three
//! things a single board structurally could not:
//!
//! * **Clock recovery.** A real host runs on its own crystal. The edge
//!   detector's six-instruction resync window exists entirely for that, and
//!   loopback — where both ends share `clk_sys` — never put it under any
//!   strain at all.
//! * **The turnaround deadline.** A host's token arrives unannounced and the
//!   reply must start within 2 to 7.5 bit times. Here the CPU has always known
//!   when a token was coming.
//! * **The handshake difference.** A bulk IN waits for an ACK and an
//!   isochronous IN does not; only a host that actually sends one can tell
//!   those apart.
//!
//! The device itself is deliberately the smallest thing that can enumerate: a
//! vendor-class device with nothing but its control endpoint. If it fails, the
//! fault is in the driver rather than in a class implementation.
//!
//! # Two ports at once
//!
//! The **native** connector runs an ordinary CDC serial for diagnostics; the
//! **PIO** connector is the device under test. Plug both in — the native one to
//! read the log, the PIO one into whatever should enumerate it.
//!
//! ```text
//! screen /dev/tty.usbmodem* 115200
//! system_profiler SPUSBDataType | grep -A 8 TeslAux
//! ```

use core::fmt::Write as _;

use embassy_executor::Spawner;

use embassy_rp::bind_interrupts;
use embassy_rp::interrupt::typelevel::{Handler, Interrupt};
use embassy_rp::peripherals::{PIO0, PIO1, PIO2, USB};
use embassy_rp::pio::{
    Config as PioConfig, FifoJoin, InterruptHandler as PioInterruptHandler, Pio, ShiftConfig,
    ShiftDirection,
};
use fixed::traits::ToFixed;
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use smart_leds::RGB8;

use teslamic_rp2350::board;
use teslamic_rp2350::pio_usb::{
    bus,
    driver::{self, PioUsb},
    hw::HwPort,
    rx::IRQ_RX_START,
};

#[path = "../../../rp2040/src/ws2812.rs"]
mod ws2812;

/// Strobes the LED rather than parking silently. A panic that looks exactly
/// like a hang cost two round trips finding the one below.
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    teslamic_rp2350::panic_led::strobe()
}

/// Services a start of packet.
///
/// On its own vector, not the one embassy's PIO futures use: the transaction
/// engine must run the moment a packet begins, and sharing a handler with the
/// executor's own PIO bookkeeping would put a scheduler between the token and
/// the reply.
struct StartOfPacket;

impl Handler<embassy_rp::interrupt::typelevel::PIO0_IRQ_1> for StartOfPacket {
    unsafe fn on_interrupt() {
        // Acknowledge first. The flag is set by a plain `irq` rather than an
        // `irq wait`, so nothing is blocked on it — but leaving it set would
        // re-enter this handler the instant it returns.
        embassy_rp::pac::PIO0
            .irq()
            .write(|w| w.set_irq(1 << IRQ_RX_START));
        driver::on_packet_start();
    }
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    // The vector has to be declared here, not merely implemented: this macro is
    // what emits the `PIO0_IRQ_1` symbol the hardware jumps to. Implementing
    // `Handler` alone leaves the handler installed nowhere, and the device is
    // deaf while looking perfectly healthy.
    PIO0_IRQ_1 => StartOfPacket;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    PIO2_IRQ_0 => PioInterruptHandler<PIO2>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

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
    // Solid from the first instruction that can run: this says the image
    // booted, the clocks came up and PIO1 works, before any USB exists.
    led.set(RGB8::new(16, 16, 16));

    // --- diagnostics on the native port ---
    let native = UsbDriver::new(p.USB, Irqs);
    let mut log_config = embassy_usb::Config::new(0xc0de, 0xca81);
    log_config.manufacturer = Some("TeslAux");
    log_config.product = Some("PIO USB enumeration log");
    log_config.serial_number = Some("pioenum");
    log_config.max_power = 100;
    log_config.max_packet_size_0 = 64;

    let mut log_cfg_desc = [0u8; 256];
    let mut log_bos_desc = [0u8; 256];
    let mut log_ctrl_buf = [0u8; 64];
    let mut log_state = State::new();
    let mut log_builder = Builder::new(
        native,
        log_config,
        &mut log_cfg_desc,
        &mut log_bos_desc,
        &mut [],
        &mut log_ctrl_buf,
    );
    let mut log_class = CdcAcmClass::new(&mut log_builder, &mut log_state, 64);
    let mut log_usb = log_builder.build();

    // --- the port under test ---
    let Pio {
        mut common,
        mut sm0,
        mut sm1,
        mut sm2,
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
    bus::preload_decoder_osr(&mut sm2);
    sm0.set_enable(true);
    sm1.set_enable(true);
    sm2.set_enable(true);

    let port = HwPort::new(
        embassy_rp::pac::PIO0,
        0,
        0,
        2,
        1,
        1, // DMA channel 1
        programs.tx.origin as u16,
        programs.decoder.origin as u16,
        programs.edge.origin as u16,
        order,
    );

    // --- a logic analyser on the spare block ---
    //
    // The counters can say a packet was missed but not why, and guessing has
    // cost two rounds already. This samples both lines at 96 MHz so the host's
    // own signalling can be read directly.
    let Pio {
        common: mut cap_common,
        sm0: mut cap_sm,
        ..
    } = Pio::new(p.PIO2, Irqs);
    let cap_prg = cap_common
        .load_program(&pio::pio_asm!(".wrap_target", "    in pins, 2", ".wrap").program);
    let mut cap_cfg = PioConfig::default();
    cap_cfg.use_program(&cap_prg, &[]);
    cap_cfg.shift_in = ShiftConfig {
        auto_fill: true,
        threshold: 32,
        direction: ShiftDirection::Right,
    };
    cap_cfg.fifo_join = FifoJoin::RxOnly;
    cap_cfg.clock_divider = 1u8.to_fixed();
    cap_sm.set_config(&cap_cfg);
    embassy_rp::pac::PIO2
        .sm(0)
        .pinctrl()
        // The base is the *lower* pin: `in pins, 2` reads upward from it, so
        // bit 0 is always the lower-numbered line and bit 1 the higher. Using
        // D- unconditionally worked only because it happened to be lower on the
        // RP2350-USB-C; on the PiZero D+ is gp28 and D- gp29, so that sampled
        // gp29 and gp30 — one real line and one unrelated pin.
        .modify(|w| w.set_in_base(board::DP_PIN.min(board::DM_PIN)));
    let mut cap_dma = p.DMA_CH0;

    // --- the device the host will see ---
    let pio_driver = PioUsb::new(port);
    let mut dev_config = embassy_usb::Config::new(0x1209, 0x0001);
    dev_config.manufacturer = Some("TeslAux");
    dev_config.product = Some("PIO USB device");
    dev_config.serial_number = Some("0001");
    dev_config.max_power = 100;
    dev_config.max_packet_size_0 = 64;
    // Vendor class with no interfaces: the smallest thing that can enumerate,
    // so a failure is the driver rather than a class implementation.
    //
    // `composite_with_iads` defaults to *true*, and embassy-usb panics in
    // `Builder::new` unless the class is then 0xEF — the magic value from the
    // USB-IF interface-association ECN. Setting a vendor class without clearing
    // it is what stopped the first two attempts: the panic handler parked
    // silently and the board looked hung rather than crashed.
    dev_config.composite_with_iads = false;
    dev_config.device_class = 0xff;
    dev_config.device_sub_class = 0x00;
    dev_config.device_protocol = 0x00;

    let mut dev_cfg_desc = [0u8; 256];
    let mut dev_bos_desc = [0u8; 256];
    let mut dev_ctrl_buf = [0u8; 64];
    let dev_builder = Builder::new(
        pio_driver,
        dev_config,
        &mut dev_cfg_desc,
        &mut dev_bos_desc,
        &mut [],
        &mut dev_ctrl_buf,
    );
    let mut device = dev_builder.build();

    let report = async {
        log_class.wait_connection().await;
        Timer::after_millis(100).await;
        let mut line = Line::new();

        line.clear();
        let _ = write!(
            line,
            "\r\n=== PIO USB enumeration ===\r\nclk_sys {} Hz  D+ gp{} D- gp{}\r\ntx@{} edge@{} decoder@{}\r\nplug the PIO port into a host now\r\n\r\n  txns    sof answer    bad setup  addr\r\n",
            embassy_rp::clocks::clk_sys_freq(),
            dp.pin(),
            dm.pin(),
            programs.tx.origin,
            programs.edge.origin,
            programs.decoder.origin,
        );
        emit(&mut log_class, &line).await;

        // Only now route the start-of-packet flag to the CPU.
        //
        // Deliberately last. Servicing a transaction runs with interrupts
        // disabled, so a fault in that path takes the diagnostics down with it
        // — which is exactly what happened the first time this was run, and it
        // left nothing to look at on either port. Arming it after the log is up
        // and has printed means a board that goes quiet from here has told us
        // where it went quiet.
        //
        // `sm3` is the enable for PIO IRQ flag 3, which is IRQ_RX_START. The
        // register names them after the state machines that usually raise them;
        // here it is the edge detector, and the flag number is what matters.
        const _: () = assert!(IRQ_RX_START == 3, "the enable below names flag 3");
        embassy_rp::pac::PIO0
            .irqs(1)
            .inte()
            .modify(|w| w.set_sm3(true));
        unsafe {
            embassy_rp::interrupt::typelevel::PIO0_IRQ_1::enable();
        }
        line.clear();
        let _ = write!(line, "start-of-packet interrupt armed\r\n");
        emit(&mut log_class, &line).await;

        // The capture lives in the reporting loop, not before it.
        //
        // The first version ran once at startup, found four hundred
        // milliseconds of idle, and stopped — and the port was plugged in after
        // that, so it missed the only thing worth seeing. An instrument that
        // watches at the wrong moment is not an instrument.
        const CAP: usize = 4096; // ~680 us at 96 MHz
        let mut capture = [0u32; CAP];
        let mut dumped = false;

        let mut last = (u32::MAX, 0, 0, 0, 0, 0);
        let mut last_quiet = false;
        // A host sends a start of frame every millisecond, so the rate is a
        // direct measure of how much of the bus we are actually hearing. The
        // counters only ever showed a total, which looks healthy while ninety
        // per cent of the traffic goes past unnoticed — and a SETUP lost that
        // way leaves no trace at all, not even in `bad`.
        let started = embassy_time::Instant::now();
        loop {
            let now = driver::stats::read();
            // Only speak when something changed: a quiet line is a useful
            // signal in itself, and a scrolling wall of identical rows is not.
            // Sample the pads across the interval. All-zero counters mean one
            // of two very different things — the host has not detected us, or
            // it is talking and we are deaf — and the bus itself says which.
            //
            // Idle J is D+ high, D- low, unchanging. A host that has noticed us
            // drives a reset (both low) and then start-of-frame traffic every
            // millisecond, so *any* movement here means it is talking.
            // Release the receiver if a bus reset has stopped it. Nothing in
            // the packet path can do this: the detector is blocked precisely
            // because no packet is being handled. In a finished driver this
            // belongs in `Bus::poll`; here it runs in the reporter, which is
            // the only thing guaranteed to keep ticking.
            driver::service_bus();

            // One capture window per pass. Whenever the host does anything —
            // whenever the port is plugged in — this is watching.
            if !dumped {
                cap_sm.set_enable(true);
                let t = cap_sm
                    .rx()
                    .dma_pull(cap_dma.reborrow(), &mut capture, false);
                let _ = embassy_time::with_timeout(
                    embassy_time::Duration::from_millis(50),
                    t,
                )
                .await;
                cap_sm.set_enable(false);
                // Trigger on a *packet*, not on any non-idle sample.
                //
                // A bus reset is ten milliseconds of SE0 and the window is
                // under one, so "first thing that is not idle" caught the reset
                // every time and never a token. K only appears inside a packet:
                // with these pins inverted, J is 0b10, SE0 is 0b11, and K is
                // 0b01. Finding one of those means real data.
                let has_k = |w: &u32| (0..16).any(|i| (w >> (i * 2)) & 0b11 == 0b01);
                let idle = capture[0];
                if let Some(first) = capture.iter().position(has_k) {
                    dumped = true;
                    let from = first.saturating_sub(4);
                    let to = (from + 160).min(CAP);
                    line.clear();
                    let _ = write!(
                        line,
                        "\r\nwire w{from}..w{to} of {CAP} (idle {idle:08x})\r\n  sample bit0=gp{} bit1=gp{}, inverted (both bits set = SE0)\r\n",
                        board::DP_PIN.min(board::DM_PIN),
                        board::DP_PIN.max(board::DM_PIN),
                    );
                    emit(&mut log_class, &line).await;
                    for chunk in capture[from..to].chunks(8) {
                        line.clear();
                        let _ = write!(line, "  ");
                        for w in chunk {
                            let _ = write!(line, "{w:08x} ");
                        }
                        let _ = write!(line, "\r\n");
                        emit(&mut log_class, &line).await;
                    }
                }
            }

            // Sample in a tight loop rather than with awaits. A start of frame
            // is about a microsecond and arrives once a millisecond, so
            // sampling every 200 µs almost never lands on one — the previous
            // version could see a ten-millisecond reset and nothing else, and
            // reported a bus that was in fact busy as STATIC.
            // Short bursts with a yield between. A start of frame is about a
            // microsecond arriving once a millisecond, so the burst has to be
            // long enough to land on one — but a single 24,000-iteration loop
            // held the executor for tens of milliseconds at a time and stopped
            // the heartbeat, which is the one signal that says the executor is
            // alive at all. Do not starve the liveness indicator to measure
            // liveness.
            let mut seen = 0u8;
            for _ in 0..8 {
                for _ in 0..2_000 {
                    let (_, dp_status) = bus::pin_registers(board::DP_PIN);
                    let (_, dm_status) = bus::pin_registers(board::DM_PIN);
                    let dp = ((dp_status >> 17) & 1) as u8;
                    let dm = ((dm_status >> 17) & 1) as u8;
                    seen |= 1 << (dp * 2 + dm);
                }
                embassy_futures::yield_now().await;
            }
            // Which bus states appeared: bit 1 = SE0, bit 2 = K, bit 4 = J.
            let quiet = seen.count_ones() == 1;

            if now != last || quiet != last_quiet {
                last = now;
                last_quiet = quiet;
                line.clear();
                let _ = write!(
                    line,
                    "{:6} {:6} {:6} {:6} {:5} {:5} {:6} {:5} {:5}  bus {seen:04b} {}\r\n",
                    now.0,
                    now.1,
                    now.2,
                    now.3,
                    now.4,
                    now.5,
                    // Start-of-frames per second. About 1000 means we are
                    // hearing the whole bus; anything far below means packets
                    // are being missed rather than mis-decoded.
                    (now.1 as u64 * 1000)
                        .checked_div(started.elapsed().as_millis().max(1))
                        .unwrap_or(0),
                    started.elapsed().as_secs(),
                    driver::address(),
                    if quiet { "STATIC" } else { "moving" }
                );
                emit(&mut log_class, &line).await;
            }
        }
    };

    // A heartbeat on the executor itself. USB is the thing under suspicion, so
    // the liveness signal must not depend on it:
    //
    //   solid          — booted, but the executor never ran. Stuck in init.
    //   blinking       — the executor is running and USB is the problem.
    //   dark           — never got as far as the first instruction.
    //
    // Blink *count* rather than colour: this board's WS2812 appears to take RGB
    // where the RP2040-Zero's takes GRB, so colours here are not trustworthy.
    let heartbeat = async {
        loop {
            led.set(RGB8::new(24, 24, 24));
            Timer::after_millis(150).await;
            led.set(RGB8::new(0, 0, 0));
            Timer::after_millis(850).await;
        }
    };

    // The receiver is released by `Engine::handle` after each transaction and
    // by the reporter between passes. An earlier version spun on
    // `service_bus()` in its own task to test whether a stuck EOP explained the
    // missing packets. It did not — the rate did not move — and spinning made
    // it worse, because each call takes a critical section and so suppresses
    // the interrupt it was meant to unblock. Measuring a thing must not consume
    // it, which is the second time that has caught me here.
    embassy_futures::join::join4(log_usb.run(), device.run(), report, heartbeat).await;
}

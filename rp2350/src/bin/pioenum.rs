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
use embassy_futures::join::join3;
use embassy_rp::bind_interrupts;
use embassy_rp::interrupt::typelevel::{Handler, Interrupt};
use embassy_rp::peripherals::{PIO0, PIO1, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use smart_leds::RGB8;

use teslamic_rp2350::pio_usb::{
    bus,
    driver::{self, PioUsb},
    hw::HwPort,
    rx::IRQ_RX_START,
    tx,
};

#[path = "../../../rp2040/src/ws2812.rs"]
mod ws2812;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
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

    let mut dp = common.make_pio_pin(p.PIN_13);
    let mut dm = common.make_pio_pin(p.PIN_12);
    bus::configure_pins(&mut dp, &mut dm);
    let order = tx::PinOrder::DmBelowDp;
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
        1, // DMA channel 1
        programs.tx.origin as u16,
        programs.decoder.origin as u16,
        order,
    );

    // Route the start-of-packet flag to the CPU. Without this the handler is
    // never called and the device is deaf while looking perfectly healthy.
    // `sm3` is the enable for PIO IRQ flag 3, which is IRQ_RX_START. The
    // register names them sm0..sm3 after the state machines that usually raise
    // them; here it is the edge detector, and the flag number is what matters.
    const _: () = assert!(IRQ_RX_START == 3, "the enable below names flag 3");
    embassy_rp::pac::PIO0
        .irqs(1)
        .inte()
        .modify(|w| w.set_sm3(true));
    unsafe {
        embassy_rp::interrupt::typelevel::PIO0_IRQ_1::enable();
    }

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

        let mut last = (u32::MAX, 0, 0, 0, 0);
        loop {
            let now = driver::stats::read();
            // Only speak when something changed: a quiet line is a useful
            // signal in itself, and a scrolling wall of identical rows is not.
            if now != last {
                last = now;
                line.clear();
                let _ = write!(
                    line,
                    "{:6} {:6} {:6} {:6} {:5} {:5}\r\n",
                    now.0,
                    now.1,
                    now.2,
                    now.3,
                    now.4,
                    driver::address()
                );
                emit(&mut log_class, &line).await;
            }
            Timer::after_millis(250).await;
        }
    };

    join3(log_usb.run(), device.run(), report).await;
}

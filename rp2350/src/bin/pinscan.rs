// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! Find the USB data pins by measuring, not by reading a schematic.
//!
//! A full-speed device's D+ carries a 1.5 kΩ pull-up to 3V3, and that is the
//! one externally identifiable thing about it. Enable every GPIO's internal
//! pull-down — tens of kilohms — and read them: anything still high has a
//! strong external pull-up, and on a USB board that is D+. D- is its neighbour.
//!
//! This exists because reading it off the PDF got it wrong twice on the board
//! that already works, and because the *absence* of a pull-up is the fault that
//! stopped the current board being seen by any host at all. Both questions are
//! answered here in ten seconds:
//!
//! * one pin high, next to a low one — D+ found, and the port is wired as a
//!   device
//! * nothing high — no device pull-up anywhere, so no host will ever look at
//!   the data lines, whatever the marketing says
//!
//! Reports over the **native** USB port and touches nothing else, so it is safe
//! to run first on any board.
//!
//! ```text
//! screen /dev/tty.usbmodem* 115200
//! ```

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    teslamic_rp2350::panic_led::strobe()
}

bind_interrupts!(struct Irqs {
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

/// Read one pin with an internal pull-down applied.
///
/// Anything reading high against a pull-down of tens of kilohms has a strong
/// external pull-up. Done through the PAC rather than by claiming each pin as
/// an embassy peripheral, because the point is to sweep all of them without
/// caring what they are used for.
fn probe(pin: usize) -> bool {
    use embassy_rp::pac;
    let gpio = pac::IO_BANK0.gpio(pin);
    let pad = pac::PADS_BANK0.gpio(pin);

    let saved_ctrl = gpio.ctrl().read();
    let saved_pad = pad.read();

    // Plain input, pull-down on, nothing driving.
    gpio.ctrl().write(|w| {
        // Function 5 is SIO on every RP2350 GPIO; the PAC names it after
        // pin 0's own signal, which is an artefact of how the SVD is written.
        w.set_funcsel(pac::io::vals::Gpio0ctrlFuncsel::SIOB_PROC_0 as u8);
        w.set_oeover(pac::io::vals::Oeover::DISABLE);
    });
    pad.write(|w| {
        w.set_ie(true);
        w.set_od(false);
        w.set_pue(false);
        w.set_pde(true);
    });
    cortex_m::asm::delay(20_000); // let the pull settle

    let high = (gpio.status().read().0 >> 17) & 1 == 1;

    // Put it back. This runs before anything else claims these pins, but
    // leaving a board's pins reconfigured behind us is a bad habit.
    gpio.ctrl().write_value(saved_ctrl);
    pad.write_value(saved_pad);
    high
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let driver = UsbDriver::new(p.USB, Irqs);
    let mut config = embassy_usb::Config::new(0xc0de, 0xca82);
    config.manufacturer = Some("TeslAux");
    config.product = Some("Pin scan");
    config.serial_number = Some("pinscan");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut cfg_desc = [0u8; 256];
    let mut bos_desc = [0u8; 256];
    let mut ctrl_buf = [0u8; 64];
    let mut state = State::new();
    let mut builder = Builder::new(
        driver,
        config,
        &mut cfg_desc,
        &mut bos_desc,
        &mut [],
        &mut ctrl_buf,
    );
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();

    let scan = async {
        class.wait_connection().await;
        Timer::after_millis(100).await;
        let mut line = Line::new();

        // The A part has 30 GPIOs and the B part 48. Probing past the end of a
        // 60-pin package reads a pad that is not bonded out, which is harmless
        // and reports as low.
        const PINS: usize = 48;

        line.clear();
        let _ = write!(
            line,
            "\r\n=== pin scan ===\r\nevery GPIO with an internal pull-down; high means a strong external pull-up\r\n\r\n"
        );
        emit(&mut class, &line).await;

        let mut pulled = [false; PINS];
        for (pin, slot) in pulled.iter_mut().enumerate() {
            *slot = probe(pin);
        }

        for row in 0..(PINS / 8) {
            line.clear();
            let _ = write!(line, "  gp{:2}..{:2} ", row * 8, row * 8 + 7);
            for i in 0..8 {
                let _ = write!(line, "{}", if pulled[row * 8 + i] { '1' } else { '.' });
            }
            let _ = write!(line, "\r\n");
            emit(&mut class, &line).await;
        }

        let found = pulled.iter().filter(|p| **p).count();
        line.clear();
        if found == 0 {
            let _ = write!(
                line,
                "\r\nNo pin has an external pull-up.\r\n  There is no full-speed device pull-up on this board, so no host\r\n  will look at the data lines whatever the documentation says. The\r\n  PIO port is wired for host mode, or the resistor is unfitted.\r\n"
            );
        } else {
            let _ = write!(
                line,
                "\r\n{found} pin(s) pulled up.\r\n  On a USB board D+ is the one carrying the 1.5k, and D- is its\r\n  neighbour — check continuity to the connector to confirm which.\r\n  Note the native USB port's own D+/D- are not GPIOs and will not\r\n  appear here.\r\n"
            );
        }
        emit(&mut class, &line).await;

        loop {
            Timer::after_millis(1000).await;
        }
    };

    join(usb.run(), scan).await;
}

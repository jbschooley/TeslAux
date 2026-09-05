// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! Is a clock actually arriving? Plain GPIO, no PIO, no USB.
//!
//! Flash this to the CAR-side board while the other board runs
//! `teslamic-rp-I2STEST.uf2`. It reads GPIO3 (BCK) and GPIO4 (LRCK) as ordinary
//! inputs and counts transitions, so it answers the wiring question without any
//! of my PIO code in the path:
//!
//! | LED    | meaning |
//! |--------|---------|
//! | green  | both BCK and LRCK toggling — wiring is good, the fault is in `slave_rx` |
//! | amber  | BCK toggling but LRCK static — LRCK wire, or the master's word clock |
//! | blue   | LRCK toggling but BCK static — BCK wire |
//! | red    | neither toggling — no clock at all: check the source board, the wires and GND |
//!
//! It also proves whether the *source* board is running at all, which
//! `i2stest` alone cannot (it has no LED and does not enumerate).

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::{Instant, Timer};
use smart_leds::RGB8;

#[macro_use]
#[path = "../pins.rs"]
mod pins;
#[path = "../ws2812.rs"]
mod ws2812;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

bind_interrupts!(struct Pio1Irqs {
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Common must outlive the LED — dropping it NULLs the pin's funcsel.
    let (mut common, sm) = {
        let Pio { common, sm0, .. } = Pio::new(p.PIO1, Pio1Irqs);
        (common, sm0)
    };
    let mut led = ws2812::Ws2812::new(&mut common, sm, p.PIN_16);
    led.set(RGB8::new(8, 0, 8));

    // No pulls: we want to see what the other board is actually driving. A
    // floating input would read noise, which shows up as "both toggling" — so
    // treat green with nothing else working as suspect and check continuity.
    // Must match the sink board's real wiring, or this reports "no clock" on a
    // link that is working perfectly. See `pins`.
    let (_, bck, lrck) = sink_i2s_pins!(p);
    let bck = Input::new(bck, Pull::None);
    let lrck = Input::new(lrck, Pull::None);

    loop {
        let (mut b_edges, mut l_edges) = (0u32, 0u32);
        let (mut pb, mut pl) = (bck.is_high(), lrck.is_high());
        let t0 = Instant::now();
        // Poll for 100 ms. At 48 kHz this should see ~307k BCK and ~4800 LRCK
        // edges; we only care whether the counts are zero or not.
        while Instant::now().duration_since(t0).as_millis() < 100 {
            let (b, l) = (bck.is_high(), lrck.is_high());
            if b != pb {
                b_edges = b_edges.saturating_add(1);
                pb = b;
            }
            if l != pl {
                l_edges = l_edges.saturating_add(1);
                pl = l;
            }
        }
        led.set(match (b_edges > 10, l_edges > 2) {
            (true, true) => RGB8::new(0, 16, 0),   // both — wiring good
            (true, false) => RGB8::new(14, 6, 0),  // BCK only
            (false, true) => RGB8::new(0, 0, 16),  // LRCK only
            (false, false) => RGB8::new(16, 0, 0), // nothing
        });
        Timer::after_millis(300).await;
    }
}

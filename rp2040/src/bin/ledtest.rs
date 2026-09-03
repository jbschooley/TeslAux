// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! Minimal WS2812 test: no USB, no audio, no spawned tasks.
//!
//! Exists because the status LED failed in a way that resisted four rounds of
//! theorising. This strips everything else away so the answer is unambiguous:
//!
//! * **Cycles red -> green -> blue** — the driver, the PIO program, the clock
//!   divider and GPIO16 are all correct, and the fault is in how the LED
//!   interacts with USB or the executor in the real firmware.
//! * **Stuck on one colour** — the FIFO push works once but the state machine
//!   then stalls.
//! * **Nothing at all** — the driver, the program, or the pin is wrong, and the
//!   earlier "magenta" sightings must have come from something else.

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::Timer;
use smart_leds::RGB8;

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
    let Pio { mut common, sm0, .. } = Pio::new(p.PIO1, Pio1Irqs);
    let mut led = ws2812::Ws2812::new(&mut common, sm0, p.PIN_16);

    // Bright enough to be unmistakable — the real firmware runs these dim.
    let colors = [
        RGB8::new(64, 0, 0),
        RGB8::new(0, 64, 0),
        RGB8::new(0, 0, 64),
        RGB8::new(48, 48, 48),
    ];
    let mut i = 0usize;
    loop {
        led.set(colors[i % colors.len()]);
        i += 1;
        Timer::after_millis(500).await;
    }
}

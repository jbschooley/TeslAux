// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! Bring-up for the RP2350-USB-C: WS2812 on GPIO16, nothing else.
//!
//! This exists to prove the *chip port* before any USB work is built on top of
//! it — thumbv8m, the Cortex-M33, the RP2350 memory map, and above all the
//! IMAGE_DEF block that `memory.x` places at the base of flash. Those fail in
//! ways that look identical from outside (the board just sits in BOOTSEL), so
//! they are worth separating from everything else.
//!
//! * **Cycles red -> green -> blue -> white** — the image boots, the clocks
//!   come up, PIO1 and GPIO16 work. The chip port is good.
//! * **Nothing, and the board re-enumerates as RPI-RP2** — the bootrom rejected
//!   the image. Suspect `.start_block` placement in `memory.x`.
//!
//! The board carries the same WS2812-on-GPIO16 as the RP2040-Zero, so this
//! shares `ws2812.rs` with the rp2040 crate rather than duplicating it.

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::Timer;
use smart_leds::RGB8;

#[path = "../../../rp2040/src/ws2812.rs"]
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

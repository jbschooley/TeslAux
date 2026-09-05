// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! Does `slave_rx` actually produce samples? No USB, no audio pipeline.
//!
//! Flash to the CAR-side board with `teslamic-rp-I2STEST.uf2` on the other.
//! Runs exactly the same `i2s_pio::slave_rx` the real firmware uses, pulls one
//! DMA block, and reports what came back:
//!
//! | LED | meaning |
//! |-----|---------|
//! | red | the DMA never completes — the PIO state machine is stalled on a `wait` |
//! | amber | blocks arrive but every sample is zero — clocking, but sampling the wrong pin or slot |
//! | green | blocks arrive with plausible non-zero audio — `slave_rx` works, fault is downstream |
//! | blue | first block still pending at startup (transient) |
//!
//! The distinction between red and amber is the whole point: "silence" in the
//! real firmware covers both, and they have completely different causes.

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use smart_leds::RGB8;

#[path = "../i2s_pio.rs"]
mod i2s_pio;
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

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});
bind_interrupts!(struct Pio1Irqs {
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Common must outlive the peripherals it configured — dropping it resets
    // the pins' function-select to NULL.
    let (mut led_common, led_sm) = {
        let Pio { common, sm0, .. } = Pio::new(p.PIO1, Pio1Irqs);
        (common, sm0)
    };
    let mut led = ws2812::Ws2812::new(&mut led_common, led_sm, p.PIN_16);
    led.set(RGB8::new(0, 0, 16));

    let (mut common, mut sm) = {
        let Pio { common, sm0, .. } = Pio::new(p.PIO0, Irqs);
        (common, sm0)
    };
    let (data, bck, lrck) = sink_i2s_pins!(p);
    i2s_pio::slave_rx(&mut common, &mut sm, data, bck, lrck);
    sm.set_enable(true);

    let mut dma = p.DMA_CH0;
    let mut raw = [0u32; 128];
    loop {
        raw = [0u32; 128];
        // A stalled state machine blocks forever, so bound the wait.
        let got = match select(
            sm.rx().dma_pull(dma.reborrow(), &mut raw, false),
            Timer::after(Duration::from_millis(500)),
        )
        .await
        {
            Either::First(_) => true,
            Either::Second(_) => false,
        };

        // Each word is a whole frame now: left in the top half, right in the
        // bottom. Count a frame as live if either channel is non-zero.
        let nonzero = raw
            .iter()
            .filter(|&&w| (w >> 16) as u16 != 0 || w as u16 != 0)
            .count();
        led.set(if !got {
            RGB8::new(16, 0, 0) // stalled
        } else if nonzero < 4 {
            RGB8::new(14, 6, 0) // clocking but all zeros
        } else {
            RGB8::new(0, 16, 0) // real data
        });
        Timer::after_millis(300).await;
    }
}

// SPDX-License-Identifier: MIT
//! Status indicator, abstracted over what the board actually has.
//!
//! The RP2040-Zero has **no plain LED**. Its only indicator is a WS2812 on
//! GPIO16, which needs a PIO state machine rather than a GPIO write — so
//! `--features rp2040-zero` drives it over PIO1 (PIO0 belongs to I2S) and gets
//! colour, which is more legible than blink codes anyway. Every other board
//! (Pico, RP2040-Plus) keeps the simple GPIO25 LED and blinks.

// Shared by every binary, and each shows a different subset of the states:
// the drive has no notion of slipping, the car board none of counts. Dead
// code here is a property of the caller, not of the module.
#![allow(dead_code)]

/// What the firmware is currently doing. The board decides how to show it.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// Streaming normally.
    Ok,
    /// Waiting for something: no source clock, or the host has not started.
    Waiting,
    /// Something is wrong and audio is muted (e.g. source sample-rate mismatch).
    Fault,
    /// Streaming, but correcting drift more than expected (source not tracking).
    Slipping,
}

#[cfg(not(feature = "rp2040-zero"))]
mod imp {
    use super::State;
    use embassy_rp::gpio::Output;
    use embassy_time::Timer;

    /// Blink codes: solid = ok, slow = waiting, fast = fault, double = slipping.
    pub async fn run(led: &mut Output<'static>, state: impl Fn() -> State) -> ! {
        loop {
            match state() {
                State::Ok => {
                    led.set_high();
                    Timer::after_millis(200).await;
                }
                State::Waiting => {
                    led.toggle();
                    Timer::after_millis(600).await;
                }
                State::Fault => {
                    led.toggle();
                    Timer::after_millis(100).await;
                }
                State::Slipping => {
                    for _ in 0..2 {
                        led.set_high();
                        Timer::after_millis(80).await;
                        led.set_low();
                        Timer::after_millis(80).await;
                    }
                    Timer::after_millis(160).await;
                }
            }
        }
    }
}

#[cfg(feature = "rp2040-zero")]
mod imp {
    use super::State;
    use crate::ws2812::Ws2812;
    use embassy_rp::peripherals::PIO1;
    use embassy_time::Timer;
    use smart_leds::RGB8;

    /// Colour codes, dimmed hard — the Zero's WS2812 is startlingly bright and
    /// this thing sits on a car dashboard.
    ///
    /// green = streaming, blue = waiting, red = fault (muted),
    /// amber = streaming but slipping.
    /// Set the colour for a state. Synchronous — a WS2812 is stateless colour,
    /// so unlike a blinking GPIO it needs no timing and no task of its own.
    /// Callable from anywhere, including the pump loop.
    pub fn set(led: &mut Ws2812<'static, PIO1, 0>, s: State) {
        led.set(match s {
            State::Ok => RGB8::new(0, 12, 0),
            State::Waiting => RGB8::new(0, 0, 12),
            State::Fault => RGB8::new(16, 0, 0),
            State::Slipping => RGB8::new(14, 6, 0),
        });
    }

    pub async fn run(led: &mut Ws2812<'static, PIO1, 0>, state: impl Fn() -> State) -> ! {
        // Written unconditionally rather than only on change. Refreshing a
        // WS2812 every 200 ms costs nothing, and change-detection means a stuck
        // task is indistinguishable from a stable state — which is exactly the
        // ambiguity that made "the LED stays at its boot colour" hard to read.
        loop {
            let c = match state() {
                State::Ok => RGB8::new(0, 12, 0),
                State::Waiting => RGB8::new(0, 0, 12),
                State::Fault => RGB8::new(16, 0, 0),
                State::Slipping => RGB8::new(14, 6, 0),
            };
            led.set(c);
            Timer::after_millis(200).await;
        }
    }
}

pub use imp::run;
#[cfg(feature = "rp2040-zero")]
#[allow(unused_imports)]
pub use imp::set;

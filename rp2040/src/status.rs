// SPDX-License-Identifier: MIT
//! Status indicator, abstracted over what the board actually has.
//!
//! The RP2040-Zero has **no plain LED**. Its only indicator is a WS2812 on
//! GPIO16, which needs a PIO state machine rather than a GPIO write — so
//! `--features rp2040-zero` drives it over PIO1 (PIO0 belongs to I2S) and gets
//! colour, which is more legible than blink codes anyway. Every other board
//! (Pico, RP2040-Plus) keeps the simple GPIO25 LED and blinks.

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
    /// Idle, reporting the worst fault seen since boot as a blink count.
    ///
    /// 1 = slips, 2 = I2S link dropped, 3 = re-enumerated at a new rate,
    /// 4 = buffer over/underran. This only shows when no source is connected,
    /// so it never competes with live status: you read it after the drive.
    Report(u8),
}

#[cfg(not(feature = "rp2040-zero"))]
mod imp {
    use super::State;
    use embassy_rp::gpio::Output;
    use embassy_time::Timer;

    /// Blink codes: solid = ok, slow = waiting, fast = fault, double =
    /// slipping, and N-blinks-then-pause for a post-drive fault report.
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
                State::Report(n) => {
                    // Long gap after the group so the count can be read off
                    // without a stopwatch.
                    for _ in 0..n {
                        led.set_high();
                        Timer::after_millis(150).await;
                        led.set_low();
                        Timer::after_millis(150).await;
                    }
                    Timer::after_millis(900).await;
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
        led.set(colour(s));
    }

    /// Steady colour for a state. `Report` blinks instead, so it maps to the
    /// colour of the fault it is reporting and the caller does the blinking.
    fn colour(s: State) -> RGB8 {
        match s {
            State::Ok => RGB8::new(0, 12, 0),
            State::Waiting => RGB8::new(0, 0, 12),
            State::Fault => RGB8::new(16, 0, 0),
            State::Slipping => RGB8::new(14, 6, 0),
            State::Report(1) => RGB8::new(14, 6, 0),  // slips: amber
            State::Report(2) => RGB8::new(0, 6, 16),  // I2S link: blue
            State::Report(3) => RGB8::new(10, 0, 14), // rate change: purple
            State::Report(4) => RGB8::new(16, 0, 0),  // buffer over/underrun: red
            State::Report(6) => RGB8::new(0, 12, 12), // level ran too empty: cyan
            _ => RGB8::new(14, 14, 14),               // PIO overflow: white
        }
    }

    pub async fn run(led: &mut Ws2812<'static, PIO1, 0>, state: impl Fn() -> State) -> ! {
        // Written unconditionally rather than only on change. Refreshing a
        // WS2812 every 200 ms costs nothing, and change-detection means a stuck
        // task is indistinguishable from a stable state — which is exactly the
        // ambiguity that made "the LED stays at its boot colour" hard to read.
        loop {
            let s = state();
            if let State::Report(n) = s {
                let c = colour(s);
                for _ in 0..n {
                    led.set(c);
                    Timer::after_millis(150).await;
                    led.set(RGB8::new(0, 0, 0));
                    Timer::after_millis(150).await;
                }
                Timer::after_millis(900).await;
                continue;
            }
            led.set(colour(s));
            Timer::after_millis(200).await;
        }
    }
}

pub use imp::run;
#[cfg(feature = "rp2040-zero")]
pub use imp::set;

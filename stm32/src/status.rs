// SPDX-License-Identifier: MIT
//! Status LED, blink-coded.
//!
//! Same `State` vocabulary as the RP2040 build, but this board has one plain
//! LED on PA1 wired in sink mode, so there is no colour to work with — every
//! state is a blink pattern. See `../README.md` for the table.

use embassy_stm32::gpio::Output;
use embassy_time::Timer;

/// What the firmware is currently doing.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// Streaming normally.
    Ok,
    /// Waiting for something: the phone has not opened the stream yet.
    Waiting,
    /// Something is wrong and audio is muted.
    Fault,
    /// Idle, reporting the worst fault seen since boot as a blink count.
    ///
    /// 1 = slips, 2 = the phone's stream dropped, 3 = buffer over/underran.
    /// This only shows when no source is connected, so it never competes with
    /// live status: you read it after the drive.
    Report(u8),
}

/// D2 on this board is wired to sink current, so it lights when the pin is LOW.
/// Naming the two ends rather than calling `set_high`/`set_low` at each site
/// keeps that inversion in exactly one place.
fn on(led: &mut Output<'static>) {
    led.set_low();
}

fn off(led: &mut Output<'static>) {
    led.set_high();
}

pub async fn run(led: &mut Output<'static>, state: impl Fn() -> State) -> ! {
    loop {
        match state() {
            State::Ok => {
                on(led);
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
            State::Report(n) => {
                // Long gap after the group so the count can be read off without
                // a stopwatch.
                for _ in 0..n {
                    on(led);
                    Timer::after_millis(150).await;
                    off(led);
                    Timer::after_millis(150).await;
                }
                Timer::after_millis(900).await;
            }
        }
    }
}

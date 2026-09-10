// SPDX-License-Identifier: MIT
//! Make a panic visible on the WS2812, without PIO, USB or an allocator.
//!
//! A panic handler that only parks the core is invisible, and on a board whose
//! sole output is one LED that is indistinguishable from a hang. It cost two
//! full round trips to find a `Builder::new` panic that way — the LED sat
//! solid, which was exactly what "booted but the executor never ran" looks
//! like.
//!
//! So the handler strobes instead. The distinction that matters is not the
//! colour, which this board's LED reports unreliably, but the *rate*:
//!
//! * roughly once a second — the executor is running
//! * several times a second — panicked
//! * solid — stopped before either
//!
//! The driver here bit-bangs the WS2812 protocol with cycle delays, because a
//! panic handler cannot borrow the PIO state machine the rest of the firmware
//! is using. It is deliberately crude: nothing it touches needs to work
//! afterwards.

use embassy_rp::pac;
use embassy_rp::pac::io::vals::{Oeover, Outover};

/// The WS2812's pin on this board, shared with the RP2040-Zero.
const LED_PIN: usize = 16;

/// Cycle counts for the WS2812's bit encoding at [`crate::pio_usb::SYSCLK_HZ`].
///
/// A bit is about 1.25 µs; a zero is short-high then long-low, a one the
/// reverse. `delay` is approximately one cycle per count, and the part tolerates
/// a wide margin, which is what makes bit-banging it viable at all.
const T0H: u32 = 34; // ~0.35 us
const T0L: u32 = 77; // ~0.80 us
const T1H: u32 = 67; // ~0.70 us
const T1L: u32 = 58; // ~0.60 us
/// Longer than the part's 50 µs latch window.
const RESET: u32 = 8_000;

fn pin_high(high: bool) {
    // Drive the output override directly rather than going through the SIO
    // registers: this has to work whatever the pin was previously assigned to,
    // and the override is checked after function select.
    pac::IO_BANK0.gpio(LED_PIN).ctrl().modify(|w| {
        w.set_oeover(Oeover::ENABLE);
        w.set_outover(if high { Outover::HIGH } else { Outover::LOW });
    });
}

fn send_byte(b: u8) {
    for i in (0..8).rev() {
        let one = b & (1 << i) != 0;
        pin_high(true);
        cortex_m::asm::delay(if one { T1H } else { T0H });
        pin_high(false);
        cortex_m::asm::delay(if one { T1L } else { T0L });
    }
}

/// Show one colour. GRB order, as the RP2040-Zero's driver uses — if this
/// board's part is RGB the colour will be wrong, which does not matter: the
/// rate is the signal.
fn show(r: u8, g: u8, b: u8) {
    send_byte(g);
    send_byte(r);
    send_byte(b);
    pin_high(false);
    cortex_m::asm::delay(RESET);
}

/// Strobe forever. Call from a `#[panic_handler]`.
///
/// Does not return, does not unwind, and touches nothing the rest of the
/// firmware needs — by the time this runs there is no rest of the firmware.
pub fn strobe() -> ! {
    loop {
        show(48, 48, 48);
        cortex_m::asm::delay(12_000_000); // ~125 ms at 96 MHz
        show(0, 0, 0);
        cortex_m::asm::delay(12_000_000);
    }
}

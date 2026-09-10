// SPDX-License-Identifier: MIT
//! A full-speed USB **device** port built out of PIO, for the second USB-C
//! connector on the Waveshare RP2350-USB-C (D+ = GPIO12, D- = GPIO13).
//!
//! # Why this exists
//!
//! The bridge has to be a USB peripheral to the car and to the phone at once,
//! which needs two device controllers. The RP2350 has one. This is the second.
//!
//! The alternative was Pico-PIO-USB, and it cannot carry this project's audio:
//! it implements no isochronous path at all (its `EP_ATTR_ISOCHRONOUS` is
//! defined and never referenced; the device IN path unconditionally waits for a
//! handshake that an iso host never sends), and its packet buffers are capped
//! at 64 bytes against the 192 this project's endpoint needs. Its *PIO
//! programs*, though, are proven and are ported here directly — see `tx.rs`.
//!
//! # Shape
//!
//! The target is an implementation of [`embassy_usb_driver::Driver`], so that
//! everything above it — the descriptors, the endpoint-less IF3 handler, the
//! byte-exact report descriptor that defeats the "unsupported USB microphone"
//! popup — is reused rather than rewritten. That is the whole reason not to
//! fork a C library: the descriptor stack is the part that must not change.
//!
//! # Status
//!
//! **Nothing here has run on hardware.** The PIO programs assemble; that is
//! all that is currently true of them.

// Hardware bring-up needs the chip HAL, so it is target-only. The codec and
// packet layers below stay host-buildable so their tests can run.
#[cfg(target_os = "none")]
pub mod bus;
pub mod codec;
pub mod packet;
pub mod rx;
pub mod tx;

/// System clock this port requires, in Hz.
///
/// Not a preference. The transmitter runs at 48 MHz and the receiver's edge
/// detector at 96 MHz, and both want an *integer* PIO divider — a fractional
/// divider dithers the period, and on a bit cell that is 8 samples wide that
/// dithering eats directly into the sampling margin.
///
/// 96 MHz is the only clock that gives both an integer divider (TX 2, RX 1)
/// while staying inside the RP2350's 150 MHz rating. The tempting alternative,
/// 192 MHz, gives dividers of 4 and 2 and twice the CPU, but it is an
/// overclock, and this ends up in a car.
///
/// Note this is *not* the RP2040 build's 124.8 MHz, which existed to make an
/// I2S divider land exactly. There is no I2S here: the single-board bridge
/// moves audio between two USB endpoints and synthesizes no audio clock at all.
pub const SYSCLK_HZ: u32 = 96_000_000;

/// PIO clock for the transmitter: 4 cycles per full-speed bit.
pub const TX_CLOCK_HZ: u32 = 48_000_000;

/// PIO clock for the receiver's edge detector: 8 cycles per full-speed bit.
pub const RX_CLOCK_HZ: u32 = 96_000_000;

const _: () = assert!(
    SYSCLK_HZ % TX_CLOCK_HZ == 0 && SYSCLK_HZ % RX_CLOCK_HZ == 0,
    "SYSCLK_HZ must give both PIO clocks an integer divider"
);

#[cfg(test)]
mod tests {
    use super::*;

    /// The three programs have to share one PIO instruction memory, and they
    /// fill it exactly. This is not a comfortable budget: if a future change
    /// needs one more instruction, something else has to give.
    #[test]
    fn the_three_programs_fit_one_pio_block() {
        let tx = tx::program().code.len();
        let edge = rx::edge_detector().code.len();
        let dec = rx::nrzi_decoder().code.len();
        assert!(
            tx + edge + dec <= 32,
            "PIO block overflows: tx {tx} + edge {edge} + decoder {dec}"
        );
    }

    /// The TX symbol values are raw instruction addresses, so the transmitter
    /// must be loaded at offset 0 and must not grow past the four symbol slots
    /// plus its EOP entry point.
    #[test]
    fn tx_symbols_stay_within_the_program() {
        let tx = tx::program().code.len();
        for s in [
            tx::Symbol::Se0,
            tx::Symbol::K,
            tx::Symbol::Release,
            tx::Symbol::J,
        ] {
            assert!((s as usize) < tx, "symbol {s:?} addresses past the program");
        }
    }
}

// SPDX-License-Identifier: MIT
//! Which board this firmware is being built for.
//!
//! Everything board-specific lives here so a new one is a feature and a table
//! entry rather than an edit spread across five binaries. Selected with
//! `--features board-<name>`; exactly one must be active.
//!
//! # Confirm the pins before trusting them
//!
//! The D+/D- assignment on the RP2350-USB-C took three attempts to get right,
//! and the schematic PDF was wrong-looking on two of them — the text extraction
//! interleaves two columns of labels, and both readings of it disagreed with
//! the board. What settled it was measurement: with internal pull-downs on both
//! pins, only D+ reads high, because only it carries the 1.5 kOhm device
//! pull-up.
//!
//! `pinscan` does exactly that sweep and prints the answer in ten seconds. Run
//! it on any new board before believing the constants below, and especially
//! before writing code against them.

/// Human-readable name, for diagnostics.
pub const NAME: &str = NAME_;
/// D+ of the PIO USB port.
pub const DP_PIN: u8 = DP_;
/// D- of the PIO USB port.
pub const DM_PIN: u8 = DM_;
/// The onboard WS2812, used for status and by the panic handler.
pub const LED_PIN: u8 = LED_;
/// Whether the pin assignment above has been confirmed against hardware.
///
/// False means it is a plausible default and nothing more. Binaries print this
/// so a run against an unconfirmed board cannot be mistaken for a working one.
pub const PINS_CONFIRMED: bool = CONFIRMED_;

#[cfg(feature = "board-usb-c")]
mod sel {
    /// Waveshare RP2350-USB-C. RP2350A, 2 MB flash.
    ///
    /// D+ on gp13 and D- on gp12, established by measurement: with internal
    /// pull-downs applied, gp13 reads high and gp12 low, and only a 1.5 kOhm
    /// external pull-up does that. Note this is the opposite of what the
    /// schematic appears to say.
    pub const NAME_: &str = "RP2350-USB-C";
    pub const DP_: u8 = 13;
    pub const DM_: u8 = 12;
    pub const LED_: u8 = 16;
    pub const CONFIRMED_: bool = true;
}

#[cfg(feature = "board-pizero")]
mod sel {
    /// Waveshare RP2350-PiZero. RP2350B, 16 MB flash, Pi Zero form factor.
    ///
    /// D+ on gp28 and D- on gp29, each behind 22 Ohm (R8/R9), from the board's
    /// own schematic. Both are inside the first thirty GPIOs, so nothing here
    /// actually needs the B part's extra pins — the feature is selected to
    /// match the silicon rather than because the driver requires it.
    ///
    /// # Whether it can be a device is unresolved
    ///
    /// There is no 1.5 kOhm anywhere on the schematic, which is what a
    /// full-speed device normally uses to announce itself on D+. But R22 is
    /// fitted at **2.2 kOhm** with R23 unfitted beside it, between the data
    /// lines and 3V3/GND, and 2.2 kOhm against a host's 15 kOhm pull-down still
    /// gives 2.88 V — comfortably over the 2.0 V a host needs to see. So it
    /// could be a device pull-up in an unusual value, or a pull-down, or on the
    /// wrong line. The PDF does not say which, and reading pull-up orientation
    /// off these schematics has been wrong twice already.
    ///
    /// `pinscan` settles it in ten seconds: gp28 reading high against an
    /// internal pull-down means a device pull-up is fitted and this board can
    /// work. Nothing high means no host will ever look at the data lines,
    /// whatever the documentation claims, and it needs a resistor first.
    ///
    /// The CC pins carry 5.1 kOhm near VBUS, which is the source role, so a
    /// USB-C host may ignore the port regardless — the same thing that made the
    /// RP2350-USB-C invisible on a C-to-C cable while working through a USB-A
    /// adapter.
    pub const NAME_: &str = "RP2350-PiZero (pull-up unverified; run pinscan)";
    pub const DP_: u8 = 28;
    pub const DM_: u8 = 29;
    pub const LED_: u8 = 16;
    pub const CONFIRMED_: bool = false;
}

use sel::*;

/// The transmitter's side-set polarity, which depends on which line got the
/// lower GPIO. Derived rather than configured: getting it wrong inverts the
/// bus, and there is no reason for two sources of truth.
pub const fn pin_order() -> crate::pio_usb::tx::PinOrder {
    if DP_PIN < DM_PIN {
        crate::pio_usb::tx::PinOrder::DpBelowDm
    } else {
        crate::pio_usb::tx::PinOrder::DmBelowDp
    }
}

#[cfg(all(feature = "board-usb-c", feature = "board-pizero"))]
compile_error!("select exactly one board-* feature");

#[cfg(not(any(feature = "board-usb-c", feature = "board-pizero")))]
compile_error!("select a board: --features board-usb-c or board-pizero");

/// Hand back the PIO USB pins for the selected board.
///
/// embassy's peripherals are distinct types per pin, so a number in a constant
/// cannot be turned into `PIN_13` — the choice has to happen where the
/// peripherals struct is in scope. Yields `(dp, dm)`, in that order.
#[macro_export]
macro_rules! usb_pins {
    ($p:expr) => {{
        #[cfg(feature = "board-usb-c")]
        {
            ($p.PIN_13, $p.PIN_12)
        }
        #[cfg(feature = "board-pizero")]
        {
            ($p.PIN_28, $p.PIN_29)
        }
    }};
}

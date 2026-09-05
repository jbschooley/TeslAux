// SPDX-License-Identifier: MIT
//! Which GPIO carries which I2S signal, on both boards at once.
//!
//! The two RP2040-Zeros are soldered bottom edge to bottom edge with one
//! rotated 180 degrees, so their pads face each other in reverse order: GP14
//! meets GP8, GP13 meets GP9, and so on down the row. Every facing pair sums to
//! [`FACING_SUM`], which is the first thing the asserts at the bottom check.
//!
//! ```text
//!     signal    source   sink
//!     data        12      10
//!     bck         11      11
//!     shield      10      12
//!     lrck         9      13
//! ```
//!
//! Physical order along the joint is the same on both boards — data, bck,
//! shield, lrck — because the GPIO numbering runs one way on one board and the
//! other way on the other.
//!
//! # What SHIELD is for
//!
//! It carries nothing. The master board holds it low for the entire run so that
//! a driven conductor sits between BCK and LRCK.
//!
//! That is the whole point of this layout. An eleven-minute recording compared
//! against the file that was played showed 112 samples in the first two and a
//! half minutes whose right channel had been replaced by its own sign bit —
//! `(sample & 0xFFFF) >> 15`, 54 of 54 checked by hand, left channel never
//! touched, jumps up to 44% of full scale. That is what a right slot read
//! fifteen bit-clocks early looks like: fifteen bits of the left slot's zero
//! padding, then one bit of the real sample. It happens when `wait 1 pin` on
//! LRCK releases before the true edge, and BCK switching at 3 MHz on the
//! neighbouring wire is the obvious thing that would release it.
//!
//! # Why the order is not free
//!
//! Two hardware constraints, not preferences:
//!
//!  * The sink runs `i2s_pio::slave_rx`, whose `in pins, 1` samples from
//!    PINCTRL_IN_BASE and nowhere else. DATA must be the **lowest** of the
//!    sink's three pins, with BCK and LRCK at the fixed offsets below — those
//!    are the `wait ... pin N` indices in the program.
//!  * The source runs `i2s_pio::master_tx`, which drives LRCK, SHIELD and BCK
//!    from side-set, and side-set pins must be **consecutive and ascending**.
//!    This is also why SHIELD has to be a side-set bit rather than a plain
//!    output: a pin can only sit between BCK and LRCK if it takes the GPIO
//!    number between them, and then all three have to belong to the side-set.
//!
//! Editing this file moves the wiring. The asserts fail the build if an edit
//! breaks either constraint, rather than letting it through as a right channel
//! that is quietly wrong.
//!
//! # Two places, kept in step
//!
//! Each number appears twice: once as a constant the asserts check, and once
//! inside the macro that names the peripheral. embassy's pins are distinct
//! types and cannot be picked by value, so `p.PIN_10` has to be written out.
//! The macros assert against the constants, so changing one without the other
//! fails to compile and says so.

#![allow(dead_code)]

/// Pads facing each other across the soldered joint sum to this.
///
/// Bottom row is GP14..GP8 left to right; rotating the second board 180 degrees
/// reverses it, so pad *n* on one board meets pad `FACING_SUM - n` on the other.
pub const FACING_SUM: u8 = 22;

/// Offset from the sink's IN base at which `slave_rx` expects BCK.
///
/// Must match the `wait ... pin 1` indices in that program.
pub const SINK_BCK_OFFSET: u8 = 1;

/// Offset from the sink's IN base at which `slave_rx` expects LRCK.
///
/// Must match the `wait ... pin 3` indices in that program.
pub const SINK_LRCK_OFFSET: u8 = 3;

/// The phone-facing board: USB in, I2S out. Drives every clock, including the
/// shield.
pub mod source {
    pub const DATA: u8 = 12;
    pub const BCK: u8 = 11;
    pub const SHIELD: u8 = 10;
    pub const LRCK: u8 = 9;
}

/// The car-facing board: I2S in, USB out. Receives the shield without
/// configuring it — the source holds it low from the other end.
pub mod sink {
    pub const DATA: u8 = 10;
    pub const BCK: u8 = 11;
    pub const SHIELD: u8 = 12;
    pub const LRCK: u8 = 13;
}

/// The sink board's I2S pins, in the order `slave_rx` and `master_rx` take them.
///
/// SHIELD is deliberately absent: when the sink is the slave the source drives
/// that line, and the sink has no reason to claim the pin.
macro_rules! sink_i2s_pins {
    ($p:expr) => {{
        const _: () = assert!(
            pins::sink::DATA == 10 && pins::sink::BCK == 11 && pins::sink::LRCK == 13,
            "pins::sink and sink_i2s_pins! disagree; update both",
        );
        ($p.PIN_10, $p.PIN_11, $p.PIN_13)
    }};
}

/// The sink board's shield pin, for the clock-locked build where the sink is
/// master and has to drive it.
macro_rules! sink_shield_pin {
    ($p:expr) => {{
        const _: () = assert!(
            pins::sink::SHIELD == 12,
            "pins::sink::SHIELD and sink_shield_pin! disagree; update both",
        );
        $p.PIN_12
    }};
}

/// The source board's I2S pins: data, bck, shield, lrck.
macro_rules! source_i2s_pins {
    ($p:expr) => {{
        const _: () = assert!(
            pins::source::DATA == 12
                && pins::source::BCK == 11
                && pins::source::SHIELD == 10
                && pins::source::LRCK == 9,
            "pins::source and source_i2s_pins! disagree; update both",
        );
        ($p.PIN_12, $p.PIN_11, $p.PIN_10, $p.PIN_9)
    }};
}

// ── The wiring has to be physically possible ────────────────────────────────
const _: () = assert!(
    source::DATA + sink::DATA == FACING_SUM,
    "DATA does not land on facing pads",
);
const _: () = assert!(
    source::BCK + sink::BCK == FACING_SUM,
    "BCK does not land on facing pads",
);
const _: () = assert!(
    source::SHIELD + sink::SHIELD == FACING_SUM,
    "SHIELD does not land on facing pads",
);
const _: () = assert!(
    source::LRCK + sink::LRCK == FACING_SUM,
    "LRCK does not land on facing pads",
);

// ── The shield has to actually be between the clocks, on both boards ────────
const _: () = assert!(
    (source::SHIELD > source::BCK) != (source::SHIELD > source::LRCK),
    "source SHIELD must lie between BCK and LRCK, or it shields nothing",
);
const _: () = assert!(
    (sink::SHIELD > sink::BCK) != (sink::SHIELD > sink::LRCK),
    "sink SHIELD must lie between BCK and LRCK, or it shields nothing",
);

// ── slave_rx: data at the IN base, clocks at the program's fixed offsets ────
const _: () = assert!(
    sink::BCK == sink::DATA + SINK_BCK_OFFSET,
    "sink BCK must sit at SINK_BCK_OFFSET above DATA; see the wait indices in slave_rx",
);
const _: () = assert!(
    sink::LRCK == sink::DATA + SINK_LRCK_OFFSET,
    "sink LRCK must sit at SINK_LRCK_OFFSET above DATA; see the wait indices in slave_rx",
);

// ── master_tx: side-set is (LRCK, SHIELD, BCK) upward from the lowest pin ───
const _: () = assert!(
    source::SHIELD == source::LRCK + 1 && source::BCK == source::LRCK + 2,
    "source LRCK/SHIELD/BCK must be consecutive ascending: side-set cannot skip a pin",
);

// ── master_rx: side-set is (BCK, SHIELD, LRCK) upward from the lowest pin ───
const _: () = assert!(
    sink::SHIELD == sink::BCK + 1 && sink::LRCK == sink::BCK + 2,
    "sink BCK/SHIELD/LRCK must be consecutive ascending: side-set cannot skip a pin",
);

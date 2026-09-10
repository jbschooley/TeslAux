// SPDX-License-Identifier: MIT
//! The full-speed NRZI transmitter, as a PIO program.
//!
//! Ported from Pico-PIO-USB's `usb_tx.pio` (`usb_tx_dpdm`, MIT, © 2021-2024
//! sekigon-gonnoc). Deliberately a port and not a rewrite: this program is the
//! one part of a PIO USB port that is genuinely proven in the field, and the
//! places it looks strange are the places it is right.
//!
//! # How it drives the bus
//!
//! The CPU never hands this program *bits*. It hands it a pre-encoded stream of
//! 2-bit symbols, each of which is the address of one of the four instructions
//! below, and the program executes `out pc, 2` to jump to the symbol it was
//! given. So the NRZI encoding, the bit stuffing and the EOP are all done in
//! software when the packet is queued, and at transmit time the state machine
//! is a dumb symbol player. That is what makes the response to a host token
//! cheap enough to hit the turnaround window: the IRQ path only starts a DMA.
//!
//! The four symbols line up with `Symbol` below, whose discriminants are the
//! instruction offsets and must not be reordered.
//!
//! # Clock
//!
//! Runs at 48 MHz — 4x the 12 Mbit/s full-speed bit rate, one instruction per
//! quarter-bit. See [`super::SYSCLK_HZ`] for why that constrains the system
//! clock rather than being satisfied by any old divider.

use pio::Program;

/// IRQ raised as the EOP starts, so the driver can tell when the packet is
/// actually on the wire rather than merely handed to the DMA.
pub const IRQ_TX_EOP: u8 = 0;

/// The 2-bit symbols the encoder emits, one per PIO instruction slot.
///
/// The values are instruction offsets into [`program`] — `out pc, 2` jumps
/// straight to them — so these are wire format, not a free choice, and they
/// match Pico-PIO-USB's `PIO_USB_TX_ENCODED_DATA_*` exactly.
///
/// Note the symbol names describe *which instruction runs*, not which line is
/// high: whether [`Symbol::J`] physically drives D+ or D- depends on the
/// side-set pin order the state machine was configured with, and this program
/// is the D+/D- (`dpdm`) variant. Do not infer bus polarity from these names.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Symbol {
    /// Both lines driven low — a single-ended zero. Starts the EOP, and is the
    /// only symbol that also raises [`IRQ_TX_EOP`].
    Se0 = 0,
    /// Hold the bus in K for one bit time.
    K = 1,
    /// Release the line drivers (`pindirs` = 0), handing the bus back. Emitted
    /// once after the EOP's SE0, not as part of the data.
    ///
    /// Pico-PIO-USB calls this `COMP`, which reads as "complement" and is
    /// misleading — NRZI's transitions are encoded by alternating [`Symbol::J`]
    /// and [`Symbol::K`], and this symbol never appears in a data run.
    Release = 2,
    /// Hold the bus in J for one bit time.
    J = 3,
}

/// Which physical pin carries D+, relative to D-.
///
/// The bus is driven by a two-bit side-set whose bit 0 is the lower-numbered
/// pin, so the encoding of "drive J" depends on which line got the lower GPIO.
/// The reference ships two separate programs for this rather than computing it,
/// and so does this.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum PinOrder {
    /// D+ is the lower-numbered pin. J is `0b01`.
    DpBelowDm,
    /// D- is the lower-numbered pin. J is `0b10`.
    ///
    /// This is the Waveshare RP2350-USB-C: D+ is gp13 and D- is gp12, which the
    /// board reported itself — with internal pull-downs on both, gp13 still
    /// read high, and only the 1.5 kOhm device pull-up does that.
    DmBelowDp,
}

impl PinOrder {
    /// Side-set value that drives an idle J bus.
    pub const fn idle_j(self) -> u8 {
        match self {
            PinOrder::DpBelowDm => 0b01,
            PinOrder::DmBelowDp => 0b10,
        }
    }
}

/// Assemble the transmitter for a given pin order.
///
/// Placed at offset 0 in instruction memory: the symbol values above are raw
/// addresses, so the program cannot be relocated without rewriting the encoder.
///
/// The two variants differ only in the two side-set constants. They are written
/// out separately rather than generated because `pio_asm!` needs literals, and
/// because a side-set polarity is not something to compute in a macro argument.
pub fn program(order: PinOrder) -> Program<32> {
    match order {
        PinOrder::DpBelowDm => pio::pio_asm!(
            ".side_set 2",
            ".define FJ_LK 0b01",
            ".define FK_LJ 0b10",
            ".define SE0   0b00",
            "    irq 0             side SE0 [7]",
            "reset:",
            ".wrap_target",
            "    out pc, 2         side FJ_LK [3]",
            "    set pindirs, 0b00 side FJ_LK [3]",
            "    out pc, 2         side FK_LJ [3]",
            "start:",
            "    set pindirs, 0b11 side FJ_LK",
            ".wrap",
        )
        .program,
        PinOrder::DmBelowDp => pio::pio_asm!(
            ".side_set 2",
            ".define FJ_LK 0b10",
            ".define FK_LJ 0b01",
            ".define SE0   0b00",
            "    irq 0             side SE0 [7]",
            "reset:",
            ".wrap_target",
            "    out pc, 2         side FJ_LK [3]",
            "    set pindirs, 0b00 side FJ_LK [3]",
            "    out pc, 2         side FK_LJ [3]",
            "start:",
            "    set pindirs, 0b11 side FJ_LK",
            ".wrap",
        )
        .program,
    }
}

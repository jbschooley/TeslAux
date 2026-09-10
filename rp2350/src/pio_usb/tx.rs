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

/// Assemble the transmitter.
///
/// Placed at offset 0 in instruction memory: the symbol values above are raw
/// addresses, so the program cannot be relocated without rewriting the encoder.
pub fn program() -> Program<32> {
    pio::pio_asm!(
        ".side_set 2",
        // J while full-speed idle. (The low-speed variant swaps these; this
        // port is full-speed only, so the distinction is compiled away.)
        ".define FJ_LK 0b01",
        ".define FK_LJ 0b10",
        ".define SE0   0b00",
        // Entry point is `reset`, one past the IRQ: the state machine is
        // started at offset+1 so this fires only at the end of a packet.
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
    .program
}

// SPDX-License-Identifier: MIT
//! The full-speed receiver: an edge detector and an NRZI decoder, as two PIO
//! programs running on two state machines.
//!
//! Ported from Pico-PIO-USB's `usb_rx.pio` (MIT, © 2021-2024 sekigon-gonnoc).
//! As with [`super::tx`] this is deliberately a port rather than a rewrite —
//! recovering a 12 Mbit/s clock in nine PIO instructions is not something to
//! reinvent, and the parts that look odd are load-bearing.
//!
//! # How the two halves split the work
//!
//! [`edge_detector`] watches the bus at 96 MHz — eight samples per bit — and
//! does two things: it resynchronises on every edge, so sampling stays centred
//! in the bit cell no matter how the host's clock drifts against ours, and it
//! raises an IRQ once per bit time to tell the decoder when to look. It also
//! spots the EOP, which is the one bus state that is neither J nor K.
//!
//! [`nrzi_decoder`] does no timing at all. It waits on that IRQ, samples the
//! pin, and turns "same as last time" into a 1 and "changed" into a 0, shifting
//! the result into the ISR. It counts six bits between transitions and drops
//! the seventh as a stuffed bit.
//!
//! # Input inversion is required
//!
//! Both programs read the bus through `jmp pin` and `in pins`, and both assume
//! **D+ and D- are configured with their GPIO input inverted**. The EOP test in
//! particular ("both inputs high") only works because a real SE0 — both lines
//! low — reads as both-high after inversion. Without the inversion the receiver
//! locks up on the first packet rather than failing visibly, so the pin setup
//! and these programs are a matched pair.
//!
//! # Instruction budget
//!
//! [`super::tx::program`] is 5 instructions, [`edge_detector`] is 17 and
//! [`nrzi_decoder`] is 10: exactly 32, which is one whole PIO block with
//! nothing to spare. The status LED therefore cannot share this block — on the
//! RP2350 that is fine, since it has three.

use pio::Program;

/// Raised by the decoder's driver on a bit-stuffing error.
pub const IRQ_RX_BS_ERR: u8 = 1;
/// Raised by [`edge_detector`] when it sees the EOP.
pub const IRQ_RX_EOP: u8 = 2;
/// Raised by [`edge_detector`] on the falling edge that starts a packet.
pub const IRQ_RX_START: u8 = 3;
/// Raised once per bit time, telling [`nrzi_decoder`] to sample.
pub const DECODER_TRIGGER: u8 = 4;

/// Bits between stuffed zeroes — six 1s force a seventh, dropped, transition.
pub const BIT_REPEAT_COUNT: u8 = 6;

/// Edge detector and EOP detector. Runs at 96 MHz (8 cycles per bit).
///
/// `jmp_pin` must be D-, `in` base must be D+, both input-inverted.
///
/// The six `jmp pin` instructions in the low half are not a loop that could be
/// tightened: they are the bit cell, one instruction per sample, and the entry
/// point into that run is what re-centres sampling after an edge.
pub fn edge_detector() -> Program<32> {
    pio::pio_asm!(
        "eop:",
        "    irq wait 2",              // IRQ_RX_EOP
        "start:",
        "    jmp pin start",           // wait for the falling edge
        "    irq 3 [1]",               // IRQ_RX_START
        ".wrap_target",
        "pin_still_low:",
        "    irq 4 [1]",               // DECODER_TRIGGER
        "pin_low:",
        "    jmp pin pin_went_high",
        "    jmp pin pin_went_high",
        "pin_went_low:",
        "    jmp pin pin_went_high",
        "    jmp pin pin_went_high",
        "    jmp pin pin_went_high",
        "    jmp pin pin_went_high",
        ".wrap",
        "pin_still_high:",
        // Both inputs read high, and both are inverted, so both lines are
        // actually low: that is an SE0, i.e. the EOP.
        "    mov x, isr [1]",
        "    jmp x-- eop",
        "pin_went_high:",
        "    mov isr, null [1]",
        "    irq 4",                   // DECODER_TRIGGER
        "    in pins, 1",              // sample D+ to distinguish EOP from K
        "    jmp pin pin_still_high",
        // Re-enter at pin_went_low rather than pin_low: that is what keeps the
        // gap between decoder triggers one bit time after an edge.
        "    jmp pin_went_low",
    )
    .program
}

/// NRZI decoder and de-stuffer. Clocked entirely by the edge detector's IRQ,
/// so its own clock divider only has to be fast enough to keep up.
///
/// `jmp_pin` must be D+, input-inverted. The OSR must be preloaded with all
/// ones, since `in osr, 1` is how a 1 bit is shifted in.
///
/// X holds the previous line state as 0 or ~0, flipped with `mov x, ~x`. Y
/// counts down the six bits to the next stuffed one; when it hits zero the
/// `jmp !y flip` path consumes the stuffed bit without shifting anything in.
pub fn nrzi_decoder() -> Program<32> {
    pio::pio_asm!(
        "start:",
        ".wrap_target",
        "set_y:",
        "    set y, 6",                // BIT_REPEAT_COUNT
        "irq_wait:",
        "    wait 1 irq 4",            // DECODER_TRIGGER
        "    jmp !y flip",             // stuffed bit: drop it, just flip state
        "    jmp pin pin_high",
        // Line is low. If the previous state was also low (x == 0) nothing
        // changed, so this is a one — that lives in the pin_high block below.
        "pin_low:",
        "    jmp !x k1",
        // Fell through: the line changed, so this is a zero.
        "j1:",
        "    in null, 1",
        "flip:",
        "    mov x, ~x",
        ".wrap",
        // Line is high. If the previous state was low (x == 0) it changed, so
        // that is a zero and belongs on the `in null, 1` above.
        "pin_high:",
        "    jmp !x j1",
        // Fell through: the line held, so this is a one. Note there is no
        // `mov x, ~x` on this path — a one means the state did not change.
        "k1:",
        "    in osr, 1",
        // `.wrap` would reset the stuff counter; a one must not. Loop back to
        // the wait instead, decrementing the six-bit countdown as it goes.
        "    jmp y-- irq_wait",
    )
    .program
}

/// `set x, 0` — clears the decoder's previous-line-state reference, which is
/// meaningless across a gap in the stream.
///
/// Hardcoded rather than assembled, because the reset runs in the interrupt
/// path and `pio_asm!` builds a `Program` at every call. The test below checks
/// it against the assembler, which is the point of having it here rather than
/// next to the code that uses it: `hw` is target-only and its tests never run.
pub const SET_X_ZERO: u16 = 0xe020;

/// `mov osr, ~null` — the all-ones OSR the decoder shifts a 1 bit out of.
pub const MOV_OSR_NOT_NULL: u16 = 0xa0eb;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_hardcoded_reset_instructions_match_the_assembler() {
        assert_eq!(
            SET_X_ZERO,
            pio::pio_asm!("set x, 0").program.code[0],
            "set x, 0"
        );
        assert_eq!(
            MOV_OSR_NOT_NULL,
            pio::pio_asm!("mov osr, ~null").program.code[0],
            "mov osr, ~null"
        );
    }
}

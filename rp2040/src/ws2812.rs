// SPDX-License-Identifier: MIT
//! Minimal WS2812 driver: one LED, no DMA.
//!
//! embassy-rp ships `PioWs2812`, and its `write()` drives the pixel over DMA.
//! That worked when called from `main` but never took effect from a spawned
//! task, leaving the LED stuck on whatever colour `main` set last. Rather than
//! keep guessing at why, this drops DMA entirely and pushes the 24-bit word
//! straight into the PIO TX FIFO — for a single pixel updated a few times a
//! second, DMA was never buying anything anyway.
//!
//! The PIO program is the standard WS2812 one (timing lifted from embassy-rp's
//! `pio_programs::ws2812`): each bit is 10 state-machine cycles at 8 MHz =
//! 1.25 us. Low for 3, high for 2, then high for 5 more if the bit is 1 or low
//! for 5 if it is 0.

use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, PioPin, ShiftConfig, ShiftDirection,
    StateMachine,
};
use embassy_rp::Peri;
use fixed::traits::ToFixed;
use smart_leds::RGB8;

/// State-machine cycles per WS2812 bit (T1 + T2 + T3 in the program below).
const CYCLES_PER_BIT: u32 = 10;
/// WS2812 bit rate.
const BIT_HZ: u32 = 800_000;

pub struct Ws2812<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize> Ws2812<'d, P, S> {
    pub fn new(common: &mut Common<'d, P>, mut sm: StateMachine<'d, P, S>, pin: Peri<'d, impl PioPin>) -> Self {
        let prg = pio::pio_asm!(
            ".side_set 1",
            "    set pindirs, 1   side 0",
            ".wrap_target",
            "bitloop:",
            "    out x, 1         side 0 [2]", // low for T3
            "    jmp !x do_zero   side 1 [1]", // high for T1
            "    jmp bitloop      side 1 [4]", // bit 1: stay high for T2
            "do_zero:",
            "    nop              side 0 [4]", // bit 0: low for T2
            ".wrap",
        );

        let out_pin = common.make_pio_pin(pin);
        sm.set_pin_dirs(Direction::Out, &[&out_pin]);

        let mut cfg = Config::default();
        cfg.use_program(&common.load_program(&prg.program), &[&out_pin]);
        cfg.set_out_pins(&[&out_pin]);
        cfg.set_set_pins(&[&out_pin]);
        cfg.clock_divider = (clk_sys_freq() as f32 / (BIT_HZ * CYCLES_PER_BIT) as f32).to_fixed();
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 24,
            direction: ShiftDirection::Left,
        };
        sm.set_config(&cfg);
        sm.set_enable(true);
        Self { sm }
    }

    /// Set the pixel. Non-blocking: one 24-bit word into a FIFO that is always
    /// empty at the update rates this is used at.
    ///
    /// GRB order, left-shifted so the top 24 bits are what the program clocks
    /// out. The line idles low afterwards because the program stalls on `out`,
    /// which is exactly the >50 us reset the part needs.
    pub fn set(&mut self, c: RGB8) {
        let word = (u32::from(c.g) << 24) | (u32::from(c.r) << 16) | (u32::from(c.b) << 8);
        self.sm.tx().push(word);
    }
}

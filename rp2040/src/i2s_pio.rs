// SPDX-License-Identifier: MIT
//! I2S in PIO.
//!
//! The RP2040 has no I2S peripheral, so all three roles this project needs are
//! PIO programs:
//!
//! | role | used by | clocks |
//! |------|---------|--------|
//! | [`SlaveRx`]  | car board, PCM2706 build | supplied by the PCM2706 |
//! | [`MasterRx`] | car board, clock-locked build | generated here, SOF-trimmed |
//! | [`SlaveTx`]  | source board | supplied by the car board |
//!
//! **Pin layout is fixed and consecutive: `base+0` = DATA, `base+1` = BCK,
//! `base+2` = LRCK.** PIO `wait pin N` is relative to the state machine's pin
//! base, so the programs below address them as pins 0/1/2 and the three GPIOs
//! must be adjacent.
//!
//! Format assumptions, matching the PCM2706 (datasheet SLES081F): 16-bit
//! stereo, **BCK = 64 x fs** (16 data bits left-justified in a 32-bit slot),
//! MSB first, standard I2S alignment — data is delayed one BCK from the LRCK
//! edge, LRCK low = left.
//!
//! # Verification status
//!
//! These programs compile and the assembler accepts them, but **none has been
//! run against real hardware**. The bit alignment in particular (that one-BCK
//! delay, and which LRCK level is the left channel) is the kind of thing that
//! is either exactly right or off by one bit, and only a scope or a recognisable
//! test signal will tell you which. Check that first if audio comes out
//! distorted or channel-swapped.

use embassy_rp::dma::Channel;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, PioPin, ShiftConfig, ShiftDirection,
    StateMachine,
};
use embassy_rp::Peri;
use fixed::traits::ToFixed;

/// PIO cycles per bit clock for the master programs. Each bit of the loop below
/// costs 2 instructions, so the state machine runs at 2x BCK.
pub const MASTER_CYCLES_PER_BCK: u32 = 2;

/// BCK is 64x the sample rate (32 bits per channel slot).
pub const BCK_RATIO: u32 = 64;

/// Set up a state machine to receive I2S as a **slave**.
///
/// `pin_base` must be the DATA pin, with BCK and LRCK on the next two GPIOs.
pub fn slave_rx<'d, P: Instance, const SM: usize>(
    common: &mut Common<'d, P>,
    sm: &mut StateMachine<'d, P, SM>,
    data: Peri<'d, impl PioPin>,
    bck: Peri<'d, impl PioPin>,
    lrck: Peri<'d, impl PioPin>,
) {
    let prg = pio::pio_asm!(
        ".wrap_target",
        // LRCK low marks the left slot. Resync every frame so a glitch costs
        // one frame rather than desynchronising the stream permanently.
        "    wait 0 pin 2",
        // Standard I2S delays data by one BCK after the LRCK edge; burn that
        // edge before sampling.
        "    wait 1 pin 1",
        "    set x, 15",
        "left:",
        "    wait 0 pin 1",
        "    wait 1 pin 1",
        "    in pins, 1",
        "    jmp x-- left",
        "    push noblock",
        "    wait 1 pin 2",
        "    wait 1 pin 1",
        "    set x, 15",
        "right:",
        "    wait 0 pin 1",
        "    wait 1 pin 1",
        "    in pins, 1",
        "    jmp x-- right",
        "    push noblock",
        ".wrap",
    );

    let data = common.make_pio_pin(data);
    let bck = common.make_pio_pin(bck);
    let lrck = common.make_pio_pin(lrck);
    sm.set_pin_dirs(Direction::In, &[&data, &bck, &lrck]);

    let mut cfg = Config::default();
    cfg.use_program(&common.load_program(&prg.program), &[]);
    cfg.set_in_pins(&[&data, &bck, &lrck]);
    // MSB first: the first bit sampled must end up in bit 15.
    cfg.shift_in = ShiftConfig {
        auto_fill: false,
        threshold: 16,
        direction: ShiftDirection::Left,
    };
    cfg.fifo_join = FifoJoin::RxOnly;
    // Free-run: as a slave the SM only ever waits on external edges, so it just
    // needs to be fast enough to catch them. At 125 MHz it has ~40 cycles per
    // BCK period at 48 kHz.
    cfg.clock_divider = 1u8.to_fixed();
    sm.set_config(&cfg);
}

/// Set up a state machine to receive I2S as **master**, generating BCK and LRCK.
///
/// `sys_hz` is the current system clock; `rate` the target sample rate. Trim the
/// result at runtime with [`set_master_rate`] to track the car's SOF.
pub fn master_rx<'d, P: Instance, const SM: usize>(
    common: &mut Common<'d, P>,
    sm: &mut StateMachine<'d, P, SM>,
    data: Peri<'d, impl PioPin>,
    bck: Peri<'d, impl PioPin>,
    lrck: Peri<'d, impl PioPin>,
    sys_hz: u32,
    rate: u32,
) {
    // side-set drives BCK (bit 0) and LRCK (bit 1) so both clocks are emitted
    // by the same instructions that sample the data — they cannot skew.
    //
    // Every bit-clock period is exactly two PIO cycles, ordered (high, low), so
    // sampling happens on the BCK rising edge. Each channel slot is 32 BCK:
    // 1 delay (standard I2S puts the LRCK edge one BCK ahead of the MSB),
    // 16 data, 15 padding — 64 BCK per frame, matching BCK_RATIO and what the
    // PCM2706 emits.
    //
    // The earlier version of this program had no padding and produced only 32
    // BCK per frame while the divider was computed for 64, which would have run
    // the whole link at twice the intended sample rate.
    let prg = pio::pio_asm!(
        ".side_set 2",
        ".wrap_target",
        // ---- left slot: LRCK low ----
        "    set x, 15          side 0b01",
        "    nop                side 0b00",
        "lrx:",
        "    in pins, 1         side 0b01",
        "    jmp x-- lrx        side 0b00",
        "    push noblock       side 0b01",
        "    set y, 13          side 0b00",
        "lpad:",
        "    nop                side 0b01",
        "    jmp y-- lpad       side 0b00",
        // ---- right slot: LRCK high ----
        "    set x, 15          side 0b11",
        "    nop                side 0b10",
        "rrx:",
        "    in pins, 1         side 0b11",
        "    jmp x-- rrx        side 0b10",
        "    push noblock       side 0b11",
        "    set y, 13          side 0b10",
        "rpad:",
        "    nop                side 0b11",
        "    jmp y-- rpad       side 0b10",
        ".wrap",
    );

    let data = common.make_pio_pin(data);
    let bck = common.make_pio_pin(bck);
    let lrck = common.make_pio_pin(lrck);
    sm.set_pin_dirs(Direction::In, &[&data]);
    sm.set_pin_dirs(Direction::Out, &[&bck, &lrck]);

    let mut cfg = Config::default();
    cfg.use_program(&common.load_program(&prg.program), &[&bck, &lrck]);
    cfg.set_in_pins(&[&data]);
    cfg.shift_in = ShiftConfig {
        auto_fill: false,
        threshold: 16,
        direction: ShiftDirection::Left,
    };
    cfg.fifo_join = FifoJoin::RxOnly;
    cfg.clock_divider = master_divider(sys_hz, rate).to_fixed();
    sm.set_config(&cfg);
}

/// Transmit I2S as a slave: clocks in, data out.
pub fn slave_tx<'d, P: Instance, const SM: usize>(
    common: &mut Common<'d, P>,
    sm: &mut StateMachine<'d, P, SM>,
    data: Peri<'d, impl PioPin>,
    bck: Peri<'d, impl PioPin>,
    lrck: Peri<'d, impl PioPin>,
) {
    let prg = pio::pio_asm!(
        ".wrap_target",
        "    wait 0 pin 1",     // LRCK low = left slot
        "    wait 1 pin 0",     // one-BCK I2S delay
        "    set x, 15",
        "left:",
        "    wait 1 pin 0",
        "    out pins, 1",      // data changes on the rising edge...
        "    wait 0 pin 0",     // ...and is sampled by the receiver on falling
        "    jmp x-- left",
        "    wait 1 pin 1",
        "    wait 1 pin 0",
        "    set x, 15",
        "right:",
        "    wait 1 pin 0",
        "    out pins, 1",
        "    wait 0 pin 0",
        "    jmp x-- right",
        ".wrap",
    );

    let data = common.make_pio_pin(data);
    let bck = common.make_pio_pin(bck);
    let lrck = common.make_pio_pin(lrck);
    sm.set_pin_dirs(Direction::Out, &[&data]);
    sm.set_pin_dirs(Direction::In, &[&bck, &lrck]);

    let mut cfg = Config::default();
    cfg.use_program(&common.load_program(&prg.program), &[]);
    cfg.set_out_pins(&[&data]);
    cfg.set_in_pins(&[&bck, &lrck]);
    cfg.shift_out = ShiftConfig {
        auto_fill: true,
        threshold: 16,
        direction: ShiftDirection::Left,
    };
    cfg.fifo_join = FifoJoin::TxOnly;
    cfg.clock_divider = 1u8.to_fixed();
    sm.set_config(&cfg);
}

/// Set up a state machine to **transmit** I2S as master, generating BCK and
/// LRCK. Used by the source board in adaptive mode, where its clock is steered
/// to follow the phone's USB frame clock.
///
/// Mirror of [`master_rx`], but each bit-clock period is ordered (low, high) so
/// data changes while BCK is low and is stable across the rising edge the
/// receiver samples on. Same 1 + 16 + 15 = 32 BCK per slot, 64 per frame.
pub fn master_tx<'d, P: Instance, const SM: usize>(
    common: &mut Common<'d, P>,
    sm: &mut StateMachine<'d, P, SM>,
    data: Peri<'d, impl PioPin>,
    bck: Peri<'d, impl PioPin>,
    lrck: Peri<'d, impl PioPin>,
    sys_hz: u32,
    rate: u32,
) {
    let prg = pio::pio_asm!(
        ".side_set 2",
        ".wrap_target",
        // ---- left slot: LRCK low ----
        "    set x, 15          side 0b00",
        "    nop                side 0b01",
        "ltx:",
        "    out pins, 1        side 0b00",
        "    jmp x-- ltx        side 0b01",
        "    set y, 14          side 0b00",
        "lpad:",
        "    nop                side 0b00",
        "    jmp y-- lpad       side 0b01",
        // ---- right slot: LRCK high ----
        "    set x, 15          side 0b10",
        "    nop                side 0b11",
        "rtx:",
        "    out pins, 1        side 0b10",
        "    jmp x-- rtx        side 0b11",
        "    set y, 14          side 0b10",
        "rpad:",
        "    nop                side 0b10",
        "    jmp y-- rpad       side 0b11",
        ".wrap",
    );

    let data = common.make_pio_pin(data);
    let bck = common.make_pio_pin(bck);
    let lrck = common.make_pio_pin(lrck);
    sm.set_pin_dirs(Direction::Out, &[&data, &bck, &lrck]);

    let mut cfg = Config::default();
    cfg.use_program(&common.load_program(&prg.program), &[&bck, &lrck]);
    cfg.set_out_pins(&[&data]);
    cfg.shift_out = ShiftConfig {
        auto_fill: true,
        threshold: 16,
        direction: ShiftDirection::Left,
    };
    cfg.fifo_join = FifoJoin::TxOnly;
    cfg.clock_divider = master_divider(sys_hz, rate).to_fixed();
    sm.set_config(&cfg);
}

/// Clock divider that makes a master state machine produce `rate` Hz.
///
/// Each BCK costs [`MASTER_CYCLES_PER_BCK`] PIO cycles and there are
/// [`BCK_RATIO`] bit clocks per frame, so the SM runs at
/// `rate * BCK_RATIO * MASTER_CYCLES_PER_BCK` Hz.
///
/// Note this is exactly where the RP2040's default 125 MHz clock hurts:
/// 48000 * 64 * 2 = 6.144 MHz, and 125/6.144 = 20.345..., which the 8.8-bit
/// fractional divider can only approximate to about 300 ppm. That error is
/// irrelevant in the PCM2706 build (we're a slave, this divider is unused) and
/// is *corrected continuously* in the clock-locked build by [`set_master_rate`].
/// If you ever want it exact open-loop, run the system clock at 147.456 MHz.
pub fn master_divider(sys_hz: u32, rate: u32) -> f32 {
    sys_hz as f32 / (rate * BCK_RATIO * MASTER_CYCLES_PER_BCK) as f32
}

/// Retune a running master state machine.
///
/// This is the actuator for the SOF-locked loop: the car board measures how
/// many audio frames elapse per USB frame and nudges the divider until that is
/// exactly `rate/1000`, which makes the I2S clock a division of the *car's*
/// clock rather than of this board's crystal.
pub fn set_master_rate<'d, P: Instance, const SM: usize>(
    sm: &mut StateMachine<'d, P, SM>,
    sys_hz: u32,
    rate_millihz: u64,
) {
    let denom = (rate_millihz * (BCK_RATIO * MASTER_CYCLES_PER_BCK) as u64) / 1000;
    if denom == 0 {
        return;
    }
    let div = (sys_hz as f32) / (denom as f32);
    sm.set_clock_divider(div.to_fixed());
    sm.clkdiv_restart();
}

/// Start a DMA transfer from a receive state machine into `buf`.
pub async fn rx_dma<'d, P: Instance, const SM: usize, C: Channel>(
    sm: &mut StateMachine<'d, P, SM>,
    ch: Peri<'d, C>,
    buf: &mut [u32],
) {
    sm.rx().dma_pull(ch, buf, false).await;
}

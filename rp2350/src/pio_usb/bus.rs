// SPDX-License-Identifier: MIT
//! Bringing the three PIO programs up on real pins.
//!
//! This is the layer that turns [`super::tx`] and [`super::rx`] into a working
//! bus: pin configuration, program loading, and the three state machines.
//! Nothing above the wire lives here — no endpoints, no transfers.
//!
//! # Pin requirements
//!
//! On the Waveshare RP2350-USB-C the second connector is D+ on GPIO12 and D- on
//! GPIO13, each behind 27 Ω, with a 1.5 kΩ pull-up from D+ to 3V3 (R13) that
//! makes the port a full-speed *device*. Those defaults are in [`DP_PIN`] and
//! [`DM_PIN`], but nothing here assumes them.

use embassy_rp::pac;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, LoadedProgram, Pin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use fixed::traits::ToFixed;

use super::{RX_CLOCK_HZ, SYSCLK_HZ, TX_CLOCK_HZ};

/// D+ on the Waveshare RP2350-USB-C's PIO-USB connector.
pub const DP_PIN: u8 = 12;
/// D- on the same connector.
pub const DM_PIN: u8 = 13;

/// Invert a pin's *input* logic.
///
/// Both receive programs are written against inverted inputs, and the EOP test
/// depends on it: an SE0 is both lines low, and the edge detector spots it by
/// finding both lines reading *high*. Get this wrong and the receiver does not
/// fail visibly — it simply never recognises the end of a packet and wedges on
/// the first one that arrives.
///
/// embassy-rp exposes `set_output_inversion` but has no input equivalent, so
/// this goes through the PAC the same way embassy's own version does.
fn set_input_inversion(pin: u8, invert: bool) {
    pac::IO_BANK0.gpio(pin as usize).ctrl().modify(|w| {
        w.set_inover(if invert {
            pac::io::vals::Inover::INVERT
        } else {
            pac::io::vals::Inover::NORMAL
        })
    });
}

/// PIO clock divider for the transmitter. Integer by construction — see
/// [`super::SYSCLK_HZ`] for why that matters.
const TX_DIV: u32 = SYSCLK_HZ / TX_CLOCK_HZ;
/// PIO clock divider for the edge detector.
const RX_DIV: u32 = SYSCLK_HZ / RX_CLOCK_HZ;

/// The three loaded programs, kept together because they must share one PIO
/// block and their offsets are related.
pub struct Programs<'d, P: Instance> {
    pub tx: LoadedProgram<'d, P>,
    pub edge: LoadedProgram<'d, P>,
    pub decoder: LoadedProgram<'d, P>,
}

/// Load all three programs into one PIO block.
///
/// The transmitter **must** end up at instruction offset 0: its symbol values
/// are raw instruction addresses that the encoder bakes into every packet, so a
/// relocated transmitter silently plays the wrong instructions. It is loaded
/// first for that reason, and [`Programs::check_tx_offset`] verifies it rather
/// than trusting the allocator.
pub fn load<'d, P: Instance>(common: &mut Common<'d, P>) -> Programs<'d, P> {
    let tx = common.load_program(&super::tx::program());
    let edge = common.load_program(&super::rx::edge_detector());
    let decoder = common.load_program(&super::rx::nrzi_decoder());
    Programs { tx, edge, decoder }
}

impl<P: Instance> Programs<'_, P> {
    /// Panics unless the transmitter landed at offset 0.
    ///
    /// Deliberately a panic and not a `Result`: there is no recovering from it
    /// at runtime, and the failure it prevents is a bus that transmits
    /// plausible-looking garbage rather than nothing.
    pub fn check_tx_offset(&self) {
        assert_eq!(
            self.tx.origin,
            0,
            "the TX program must load at offset 0 — its symbols are instruction addresses"
        );
    }
}

/// Configure the transmitter's state machine.
///
/// `dp` and `dm` must be consecutive with D+ lower, since the side-set that
/// drives the bus writes both at once from a two-bit field.
pub fn init_tx<'d, P: Instance, const SM: usize>(
    sm: &mut StateMachine<'d, P, SM>,
    prg: &LoadedProgram<'d, P>,
    dp: &Pin<'d, P>,
    dm: &Pin<'d, P>,
) {
    let mut cfg = Config::default();
    // Side-set drives the line states; `set pindirs` releases them for the
    // bus turnaround. Both need the pair, D+ first.
    cfg.use_program(prg, &[dp, dm]);
    cfg.set_set_pins(&[dp, dm]);
    // The symbol stream is packed four to a byte, most significant pair first,
    // and `out pc, 2` consumes it two bits at a time.
    cfg.shift_out = ShiftConfig {
        auto_fill: true,
        threshold: 8,
        direction: ShiftDirection::Left,
    };
    cfg.fifo_join = FifoJoin::TxOnly;
    cfg.clock_divider = TX_DIV.to_fixed();
    sm.set_config(&cfg);
    sm.set_pin_dirs(Direction::Out, &[dp, dm]);
}

/// Configure the edge detector's state machine.
///
/// `jmp pin` reads D-, `in pins` reads D+, and both are input-inverted by
/// [`configure_pins`].
pub fn init_edge_detector<'d, P: Instance, const SM: usize>(
    sm: &mut StateMachine<'d, P, SM>,
    prg: &LoadedProgram<'d, P>,
    dp: &Pin<'d, P>,
    dm: &Pin<'d, P>,
) {
    let mut cfg = Config::default();
    cfg.use_program(prg, &[]);
    cfg.set_jmp_pin(dm);
    cfg.set_in_pins(&[dp]);
    cfg.clock_divider = RX_DIV.to_fixed();
    sm.set_config(&cfg);
    sm.set_pin_dirs(Direction::In, &[dp, dm]);
}

/// Configure the NRZI decoder's state machine.
///
/// This one does no timing of its own — it is driven entirely by the edge
/// detector's IRQ — so it runs undivided and only has to keep up.
///
/// The OSR is preloaded with all ones because `in osr, 1` is how the program
/// shifts in a 1 bit. Left unset it would shift in zeroes and every received
/// byte would be wrong in a way that looks like a line fault.
pub fn init_decoder<'d, P: Instance, const SM: usize>(
    sm: &mut StateMachine<'d, P, SM>,
    prg: &LoadedProgram<'d, P>,
    dp: &Pin<'d, P>,
) {
    let mut cfg = Config::default();
    cfg.use_program(prg, &[]);
    cfg.set_jmp_pin(dp);
    // Bytes are assembled least significant bit first, matching the wire.
    cfg.shift_in = ShiftConfig {
        auto_fill: true,
        threshold: 8,
        direction: ShiftDirection::Right,
    };
    cfg.fifo_join = FifoJoin::RxOnly;
    cfg.clock_divider = 1u8.to_fixed();
    sm.set_config(&cfg);
}

/// Put D+ and D- into the state the receive programs expect.
///
/// Call before enabling any state machine. Pull-downs matter for the same
/// reason they do on the I2S link in the two-board build: an unplugged port
/// left floating picks up noise, and a receiver that syncs on noise produces
/// packets that never existed.
pub fn configure_pins<'d, P: Instance>(dp: &mut Pin<'d, P>, dm: &mut Pin<'d, P>) {
    use embassy_rp::gpio::Pull;
    dp.set_pull(Pull::Down);
    dm.set_pull(Pull::Down);
    set_input_inversion(dp.pin(), true);
    set_input_inversion(dm.pin(), true);
}

/// The embassy-rp clock configuration this port requires.
///
/// Pass this to `embassy_rp::init` instead of `Default::default()`. The default
/// brings the RP2350 up at 150 MHz, which divides to 75 MHz and 150 MHz rather
/// than the 48 and 96 the programs are written for — and the failure is not a
/// build error or a panic, it is a bus running at the wrong bit rate, which
/// looks exactly like a wiring fault.
///
/// See [`super::SYSCLK_HZ`] for why 96 MHz specifically.
pub fn clock_config() -> embassy_rp::config::Config {
    let clocks = embassy_rp::clocks::ClockConfig::system_freq(SYSCLK_HZ)
        .expect("96 MHz must be reachable from the 12 MHz crystal");
    embassy_rp::config::Config::new(clocks)
}

/// Panic unless the chip is actually running at [`super::SYSCLK_HZ`].
///
/// Cheap insurance against the one mistake that cannot be caught at compile
/// time: the dividers above are computed from a constant, and if the running
/// clock is not that constant they are silently wrong.
pub fn check_sysclk() {
    let actual = embassy_rp::clocks::clk_sys_freq();
    assert_eq!(
        actual, SYSCLK_HZ,
        "PIO USB needs a {SYSCLK_HZ} Hz system clock; init with bus::clock_config()"
    );
}

/// Preload the decoder's OSR with all ones.
///
/// `in osr, 1` is how [`super::rx::nrzi_decoder`] shifts in a 1 bit, so an OSR
/// left at its reset value shifts in zeroes instead and every received byte is
/// wrong — in a way that looks like a line fault rather than a missing setup
/// step, which is why this is its own named function rather than a line in a
/// bring-up sequence.
///
/// Must be called after `set_config` and before the state machine is enabled.
pub fn preload_decoder_osr<'d, P: Instance, const SM: usize>(sm: &mut StateMachine<'d, P, SM>) {
    let instr = pio::pio_asm!("mov osr, ~null").program.code[0];
    // Safe in the sense that matters here: the state machine is stopped, so
    // this cannot interleave with the program's own execution.
    unsafe { sm.exec_instr(instr) };
}

/// Clear the EOP flag the edge detector raises at the end of a packet.
///
/// The program uses `irq wait`, which sets the flag and then blocks until
/// something clears it. In the finished driver that is the interrupt handler
/// acknowledging a completed packet; until then, anything driving the bus has
/// to clear it by hand or the detector receives exactly one packet and stops.
pub fn clear_eop_irq<P: Instance>(flags: &embassy_rp::pio::IrqFlags<'_, P>) {
    flags.clear(super::rx::IRQ_RX_EOP as usize);
}

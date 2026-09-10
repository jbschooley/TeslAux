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
    // Deliberately no `set_pin_dirs` here. The program owns the direction of
    // these pins — [`tx_start`] drives them and [`tx_release`] hands them back —
    // because D+/D- are a shared bus, and the edge detector configures the same
    // two pins as inputs a moment later. Setting a direction here would either
    // be immediately overridden or fight the receiver.
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


/// Side-set field for an idle-J bus, positioned for a two-bit side-set.
///
/// The RP2040/RP2350 side-set field occupies the top bits of the delay/side-set
/// field, so a 2-bit value is shifted by `13 - 2`. This mirrors the SDK's
/// `pio_encode_sideset(2, FJ_LK)` with `FJ_LK = 0b01` for a D+/D- pin order.
const SIDESET_J: u16 = 0b01 << 11;

/// Offset of `start:` in the TX program — the `set pindirs, 0b11` that turns
/// the line drivers on. Not reachable through the symbol stream, whose values
/// only address 0..=3, so it can only be entered by injecting a jump.
const TX_START_OFFSET: u16 = 4;

/// Offset of the `set pindirs, 0b00` that releases the drivers.
const TX_RELEASE_OFFSET: u16 = 2;

/// Take control of the bus, ready to transmit.
///
/// **Must be called before every packet.** The transmitter's symbol values
/// address instructions 0..=3 only, so the `set pindirs, 0b11` at offset 4 is
/// unreachable from the symbol stream — the only way in is to inject a jump,
/// which is what the reference driver does before each transfer
/// (`pio_usb.c:287`, called from `pio_usb_device.c:144`).
///
/// Skipping it does not fail loudly. The state machine happily clocks the whole
/// packet out onto pins that are still inputs, the bus stays static, and the
/// receiver decodes that silence into a repeating seven-bit pattern that looks
/// like data. That is precisely how this was found.
///
/// The injected jump carries a side-set of J so the bus is left idle-J before
/// the first symbol rather than wherever it happened to be.
pub fn tx_start<'d, P: Instance, const SM: usize>(
    sm: &mut StateMachine<'d, P, SM>,
    prg: &LoadedProgram<'d, P>,
) {
    let instr = SIDESET_J | (prg.origin as u16 + TX_START_OFFSET);
    // Injecting into a running state machine is exactly what this is for.
    unsafe { sm.exec_instr(instr) };
}

/// Release the bus, letting the pull-ups and the host take it back.
pub fn tx_release<'d, P: Instance, const SM: usize>(
    sm: &mut StateMachine<'d, P, SM>,
    prg: &LoadedProgram<'d, P>,
) {
    let instr = SIDESET_J | (prg.origin as u16 + TX_RELEASE_OFFSET);
    unsafe { sm.exec_instr(instr) };
}

/// Offset of `start:` in the edge detector — the instruction that waits for the
/// falling edge that begins a packet. Instruction 0 is the `irq wait` at EOP.
const EDGE_START_OFFSET: u16 = 1;

/// Reset the decoder so the next packet starts on a clean byte boundary.
///
/// **Must be called before every packet.** The decoder has no idea where a
/// packet begins — it shifts whatever the edge detector triggers it to sample
/// into an ISR that autopushes every eight bits. Left running across packets,
/// those eight-bit groups land wherever they happen to, so the SYNC byte
/// arrives split across two pushes and every byte after it is rotated. The
/// symptom is a stream that contains recognisable fragments but matches nothing
/// at offset 0.
///
/// Mirrors `pio_usb_bus_prepare_receive` (`pio_usb.c`): stop, clear the FIFO,
/// restart the shift counters, jump back to the program start, and zero X — the
/// previous-line-state reference, which is meaningless across a gap in the
/// stream.
pub fn prepare_decoder<'d, P: Instance, const SM: usize>(
    sm: &mut StateMachine<'d, P, SM>,
    prg: &LoadedProgram<'d, P>,
) {
    sm.set_enable(false);
    sm.clear_fifos();
    sm.restart();
    let jmp_to_start = prg.origin as u16;
    let set_x_zero = pio::pio_asm!("set x, 0").program.code[0];
    // Stopped, so nothing can interleave with these.
    unsafe {
        sm.exec_instr(jmp_to_start);
        sm.exec_instr(set_x_zero);
    }
    // `restart` clears the shift counters; put the all-ones OSR back rather
    // than depend on it having survived.
    preload_decoder_osr(sm);
    sm.set_enable(true);
}

/// Return the edge detector to waiting for a start of packet.
///
/// Its instruction 0 is an `irq wait` that blocks until the EOP flag is
/// acknowledged, so in the finished driver [`clear_rx_irqs`] is enough to
/// release it. This is the blunter version, for a caller firing discrete
/// packets that wants the detector in a known state regardless of where the
/// last one left it.
pub fn restart_edge_detector<'d, P: Instance, const SM: usize>(
    sm: &mut StateMachine<'d, P, SM>,
    prg: &LoadedProgram<'d, P>,
) {
    sm.set_enable(false);
    sm.restart();
    unsafe { sm.exec_instr(prg.origin as u16 + EDGE_START_OFFSET) };
    sm.set_enable(true);
}

/// Clear every receive-side IRQ flag.
///
/// A stale `DECODER_TRIGGER` left over from the previous packet makes the
/// decoder sample a bit that no edge detector asked for, which costs exactly
/// one bit of alignment — enough to corrupt everything after it.
pub fn clear_rx_irqs<P: Instance>(flags: &embassy_rp::pio::IrqFlags<'_, P>) {
    for irq in [
        super::rx::IRQ_RX_BS_ERR,
        super::rx::IRQ_RX_EOP,
        super::rx::IRQ_RX_START,
        super::rx::DECODER_TRIGGER,
    ] {
        flags.clear(irq as usize);
    }
}

/// Raw `GPIO_CTRL` and `GPIO_STATUS` for a pin, for bring-up diagnostics.
///
/// Returned unhelpfully as raw words on purpose: the RP2350 dropped the
/// `INTOPERI` status field the RP2040 had, so there is no way to read back what
/// the peripheral actually sees after inversion. The next best thing is the
/// configuration and the pad level, decoded off-chip.
///
/// In `CTRL`, bit 0..4 is `FUNCSEL` (PIO0 is 6) and bits 16..17 are `INOVER`
/// (1 = invert). In `STATUS`, bit 17 is `INFROMPAD`.
pub fn pin_registers(pin: u8) -> (u32, u32) {
    let gpio = pac::IO_BANK0.gpio(pin as usize);
    (gpio.ctrl().read().0, gpio.status().read().0)
}

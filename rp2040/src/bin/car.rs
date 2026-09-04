// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! Car-facing board: presents the TeslaMic to the Tesla and streams whatever
//! arrives on I2S.
//!
//! Two builds from one source:
//!
//! * **default** — I2S *slave*. Pair with a PCM2706 module: the phone plugs into
//!   the PCM2706, which is the I2S master with its clock PLL'd off the phone's
//!   USB SOF. Our clock and the car's are then independent, so the pipe runs in
//!   [`PaceMode::Elastic`] and the iso packet size varies by +/-1 sample to
//!   absorb drift. One board, no second firmware.
//!
//! * **`--features clock-locked`** — I2S *master*, with the clock divider
//!   trimmed continuously so exactly 48 audio frames elapse per USB SOF. The
//!   source board is the I2S slave, so the entire chain ends up running on the
//!   car's clock, there is no drift, and every iso packet is exactly 192 bytes —
//!   matching the real TeslaMic's fixed-size adaptive endpoint. Requires the
//!   `source` binary on a second board.
//!
//! Pins (must be consecutive — see `i2s_pio`): GPIO2 DATA, GPIO3 BCK, GPIO4 LRCK.
//! Status LED on GPIO25.

use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::class::hid::State as HidState;
use embassy_usb::driver::{EndpointError, EndpointIn};
use embassy_usb::Builder;

use core::cell::RefCell;

#[path = "../audio_pipe.rs"]
mod audio_pipe;
#[path = "../i2s_pio.rs"]
mod i2s_pio;
#[path = "../status.rs"]
mod status;
#[path = "../ws2812.rs"]
mod ws2812;
#[path = "../teslamic.rs"]
mod teslamic;

use audio_pipe::{classify, PaceMode, Pipe, RateDetect};

/// ~10 ms of slack at 48 kHz, half in each direction. Comfortably larger than
/// the I2S DMA block so burst delivery never trips the pacer (see the deadband
/// note in `audio_pipe`).
#[cfg(not(feature = "low-latency"))]
const RING: usize = 512;
/// Half the cushion. Target 128 must exceed deadband (64) + producer burst (16)
/// = 80, leaving 48 frames of margin.
#[cfg(feature = "low-latency")]
const RING: usize = 256;
/// Deadband. Must exceed the larger of the two burst sizes: the I2S DMA block
/// coming in, and one USB frame going out.
#[cfg(not(feature = "low-latency"))]
const HYSTERESIS: usize = 96;
#[cfg(feature = "low-latency")]
const HYSTERESIS: usize = 64;

/// Frames per I2S DMA block. Must stay under the pipe's deadband (96 frames at
/// 48 kHz) or the pacer chases its own sampling phase.
#[cfg(not(feature = "low-latency"))]
const I2S_BLOCK: usize = 64;
/// Smaller blocks mean a smaller producer burst, which is what lets the cushion
/// shrink. Costs more frequent DMA completions, which the RP2040 absorbs easily.
#[cfg(feature = "low-latency")]
const I2S_BLOCK: usize = 16;

#[cfg(all(not(feature = "clock-locked"), not(feature = "packet-stress")))]
const MODE: PaceMode = PaceMode::Elastic;
#[cfg(all(feature = "clock-locked", not(feature = "packet-stress")))]
const MODE: PaceMode = PaceMode::Locked;
#[cfg(feature = "packet-stress")]
const MODE: PaceMode = PaceMode::Stress;

/// 256-point sine, quarter scale. Only used by the `packet-stress` diagnostic.
///
/// Indexed by a phase accumulator at **997 Hz**, not 1000 Hz, and interpolated.
/// Both details matter: 1 kHz at 48 kHz is exactly 48 samples, i.e. exactly one
/// cycle per USB packet, so a fault that merely reorders a packet's contents is
/// completely inaudible — that blind spot hid a real corruption bug in the nRF
/// firmware for two months. 997 shares no factor with the frame rate. And
/// indexing by the top 8 bits alone gives only ~-40 dBFS of phase-truncation
/// error, audible as a spurious tone, so the low bits interpolate.
#[cfg(feature = "packet-stress")]
#[rustfmt::skip]
const SINE256: [i16; 256] = [
         0,    393,    785,   1177,   1568,   1959,   2348,   2735,
      3121,   3506,   3888,   4267,   4645,   5019,   5390,   5758,
      6123,   6484,   6841,   7194,   7542,   7886,   8226,   8560,
      8889,   9213,   9531,   9844,  10150,  10451,  10745,  11033,
     11314,  11588,  11855,  12115,  12368,  12614,  12851,  13081,
     13304,  13518,  13724,  13921,  14111,  14292,  14464,  14627,
     14782,  14928,  15065,  15192,  15311,  15420,  15521,  15611,
     15693,  15764,  15827,  15880,  15923,  15957,  15981,  15995,
     16000,  15995,  15981,  15957,  15923,  15880,  15827,  15764,
     15693,  15611,  15521,  15420,  15311,  15192,  15065,  14928,
     14782,  14627,  14464,  14292,  14111,  13921,  13724,  13518,
     13304,  13081,  12851,  12614,  12368,  12115,  11855,  11588,
     11314,  11033,  10745,  10451,  10150,   9844,   9531,   9213,
      8889,   8560,   8226,   7886,   7542,   7194,   6841,   6484,
      6123,   5758,   5390,   5019,   4645,   4267,   3888,   3506,
      3121,   2735,   2348,   1959,   1568,   1177,    785,    393,
         0,   -393,   -785,  -1177,  -1568,  -1959,  -2348,  -2735,
     -3121,  -3506,  -3888,  -4267,  -4645,  -5019,  -5390,  -5758,
     -6123,  -6484,  -6841,  -7194,  -7542,  -7886,  -8226,  -8560,
     -8889,  -9213,  -9531,  -9844, -10150, -10451, -10745, -11033,
    -11314, -11588, -11855, -12115, -12368, -12614, -12851, -13081,
    -13304, -13518, -13724, -13921, -14111, -14292, -14464, -14627,
    -14782, -14928, -15065, -15192, -15311, -15420, -15521, -15611,
    -15693, -15764, -15827, -15880, -15923, -15957, -15981, -15995,
    -16000, -15995, -15981, -15957, -15923, -15880, -15827, -15764,
    -15693, -15611, -15521, -15420, -15311, -15192, -15065, -14928,
    -14782, -14627, -14464, -14292, -14111, -13921, -13724, -13518,
    -13304, -13081, -12851, -12614, -12368, -12115, -11855, -11588,
    -11314, -11033, -10745, -10451, -10150,  -9844,  -9531,  -9213,
     -8889,  -8560,  -8226,  -7886,  -7542,  -7194,  -6841,  -6484,
     -6123,  -5758,  -5390,  -5019,  -4645,  -4267,  -3888,  -3506,
     -3121,  -2735,  -2348,  -1959,  -1568,  -1177,   -785,   -393,
];

/// Phase step per sample for the diagnostic tone.
#[cfg(feature = "packet-stress")]
const TONE_PHASE_INC: u32 = ((997u64 << 32) / 48_000u64) as u32;
/// Default advertised rate. The board follows the source instead of insisting on
/// this — see `boot_rate` and the renegotiation in `pump`.
const DEFAULT_RATE: u32 = teslamic::SAMPLE_RATE;

/// Rates we are willing to advertise to the car.
///
/// Retested in the car after the ISO transport fixes: 32 k, 44.1 k, 48 k and
/// 96 k all play cleanly, so following the source's rate is safe. The July
/// finding that 44.1 kHz buzzed was our own bug, not the car's.
/// The ring size caps this. `set_rate` scales the deadband with the rate, and a
/// deadband at or above the target means the pacer can never correct — the level
/// cannot travel far enough to leave the band, so drift accumulates unchecked
/// until the buffer hits an end. At 96 kHz the deadband is 192 frames, which
/// exceeds the low-latency build's 128-frame target, so that build must not
/// advertise 96 kHz. Checked below at compile time rather than trusted.
///
/// 96 kHz is deliberately absent even though the car plays it: its deadband is
/// 192 frames, which with a 64-frame I2S block does not fit inside either
/// build's target (256 and 128). Supporting it would mean a 1024-frame ring on
/// the car board — 21 ms of cushion — for a rate nothing in this chain produces:
/// the PCM2706 tops out at 48 kHz and the RP2040 source advertises 48 kHz only.
/// If a 96 kHz bridge is ever used, grow RING and the assertion below will stop
/// complaining.
const SUPPORTED_RATES: [u32; 3] = [32_000, 44_100, 48_000];

/// Every advertised rate must leave the pacer room to work: deadband strictly
/// below target, with at least one producer burst of slack.
const _: () = {
    let mut i = 0;
    while i < SUPPORTED_RATES.len() {
        let r = SUPPORTED_RATES[i] as usize;
        let deadband = (r / 1000) * 2;
        let target = RING / 2;
        assert!(
            deadband + I2S_BLOCK < target,
            "a supported rate's deadband does not fit this RING; drop the rate or grow the buffer"
        );
        i += 1;
    }
};

/// Marks scratch1 as holding a rate we put there, rather than power-on garbage.
const RATE_MAGIC: u32 = 0x5453_4C41; // "TSLA"

/// Bytes per audio frame: 2 channels x 2 bytes.
const CHANNELS_X_BYTES: usize = 4;

/// Read the rate chosen by a previous run before it reset itself.
///
/// embassy-usb builds its descriptors once, at startup, so changing the
/// advertised sample rate means re-enumerating. The watchdog scratch registers
/// survive a watchdog reset, which makes them the natural place to carry the
/// decision across.
fn boot_rate(wd: &mut embassy_rp::watchdog::Watchdog) -> u32 {
    if wd.get_scratch(0) == RATE_MAGIC {
        faults::RATE_CHANGES.store(wd.get_scratch(2), core::sync::atomic::Ordering::Relaxed);
        let r = wd.get_scratch(1);
        if SUPPORTED_RATES.contains(&r) {
            return r;
        }
    }
    DEFAULT_RATE
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

#[cfg(feature = "rp2040-zero")]
bind_interrupts!(struct Pio1Irqs {
    PIO1_IRQ_0 => PioInterruptHandler<embassy_rp::peripherals::PIO1>;
});

/// Shared between the capture task (producer) and the USB pump (consumer).
static PIPE: Mutex<CriticalSectionRawMutex, RefCell<Pipe<RING>>> =
    Mutex::new(RefCell::new(Pipe::new_with_hysteresis(DEFAULT_RATE, HYSTERESIS)));

/// Set once the measured source rate disagrees with what we told the car. The
/// pump then emits silence: wrong-pitch audio is worse than none, and it makes
/// a misconfigured source obvious instead of mysterious.
static MUTED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// The rate currently advertised to the car, set at boot.
static ADVERTISED: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(DEFAULT_RATE);

/// Latched if a packet was ever rejected for exceeding wMaxPacketSize — a
/// firmware bug, shown as a fault rather than left to look like an audio glitch.
static OVERSIZE: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// Audio frames captured from I2S, counted by the capture task and read by the
/// pump.
///
/// Rate detection needs both halves: how many frames arrived (capture) and how
/// many USB frames elapsed (pump). They live in different tasks, so the count
/// has to cross between them. An earlier version gave each task its own
/// `RateDetect`, so the pump's saw zero captured frames, computed 0 Hz, failed
/// `classify()` and muted the output one second after startup — with perfectly
/// good audio arriving.
///
/// Plain load/store: thumbv6m has no atomic read-modify-write, and the capture
/// task is the only writer.
static CAPTURED: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Latched record of what went wrong since boot, for reading back after a
/// drive.
///
/// You cannot watch an LED while driving, and "it sounded weird once or twice"
/// does not say whether the I2S link dropped, the pacer slipped, or the buffer
/// ran dry — which have completely different fixes. One is a loose wire, one is
/// firmware. These latch so the answer survives until you can look.
mod faults {
    use core::sync::atomic::{AtomicU32, Ordering};

    /// I2S clock vanished for >250 ms. Mechanical: a wire, a connector, EMI.
    pub static I2S_STALLS: AtomicU32 = AtomicU32::new(0);
    /// Pacer corrected drift, sampled from the pipe's own counters. Expected
    /// occasionally; frequent means the clock steering is struggling.
    pub static SLIPS: AtomicU32 = AtomicU32::new(0);
    /// Buffer ran dry or overflowed, likewise sampled from the pipe. The
    /// cushion was not big enough.
    pub static STARVED: AtomicU32 = AtomicU32::new(0);
    /// Re-enumerated because the source changed sample rate. ~1 s of silence.
    pub static RATE_CHANGES: AtomicU32 = AtomicU32::new(0);

    /// Plain load/store: thumbv6m has no atomic read-modify-write, and each of
    /// these has a single writer.
    pub fn bump(c: &AtomicU32) {
        c.store(c.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
    }

    /// Worst thing seen, most serious first, as a blink count. `None` means a
    /// clean run.
    ///
    /// A handful of slips over a drive is normal for a free-running source and
    /// is not worth reporting, so that one has a threshold; the rest are
    /// individually audible and are reported on first occurrence.
    pub fn worst() -> Option<u8> {
        if STARVED.load(Ordering::Relaxed) > 0 {
            Some(4)
        } else if RATE_CHANGES.load(Ordering::Relaxed) > 0 {
            Some(3)
        } else if I2S_STALLS.load(Ordering::Relaxed) > 0 {
            Some(2)
        } else if SLIPS.load(Ordering::Relaxed) > 8 {
            Some(1)
        } else {
            None
        }
    }
}

/// True while I2S frames are actually arriving.
static SOURCE_LIVE: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Which rate did a previous run decide to advertise? Defaults to 48 kHz on
    // a cold boot.
    let mut wd = embassy_rp::watchdog::Watchdog::new(p.WATCHDOG);
    let rate = boot_rate(&mut wd);
    ADVERTISED.store(rate, core::sync::atomic::Ordering::Relaxed);
    PIPE.lock(|p| p.borrow_mut().set_rate(rate));

    // ── USB: the TeslaMic ───────────────────────────────────────────────────
    let driver = Driver::new(p.USB, Irqs);

    static mut CONFIG_DESC: [u8; 256] = [0; 256];
    static mut BOS_DESC: [u8; 32] = [0; 32];
    static mut MSOS_DESC: [u8; 16] = [0; 16];
    // The 40-byte serial makes a 162-byte string descriptor; 128 is too small
    // and silently breaks enumeration.
    static mut CONTROL_BUF: [u8; 512] = [0; 512];
    static mut HID_STATE: HidState = HidState::new();
    static mut KBD: teslamic::KeyboardHandler = teslamic::KeyboardHandler;
    static mut IF3: teslamic::If3Handler = teslamic::If3Handler;

    // SAFETY: single-threaded startup, each static taken exactly once, and all
    // outlive `usb` (main never returns).
    let (config_desc, bos_desc, msos_desc, control_buf, hid_state, kbd, if3) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(CONFIG_DESC),
            &mut *core::ptr::addr_of_mut!(BOS_DESC),
            &mut *core::ptr::addr_of_mut!(MSOS_DESC),
            &mut *core::ptr::addr_of_mut!(CONTROL_BUF),
            &mut *core::ptr::addr_of_mut!(HID_STATE),
            &mut *core::ptr::addr_of_mut!(KBD),
            &mut *core::ptr::addr_of_mut!(IF3),
        )
    };

    let mut builder = Builder::new(
        driver,
        teslamic::config(),
        config_desc,
        bos_desc,
        msos_desc,
        control_buf,
    );
    // wMaxPacketSize must be computed from the rate we are about to advertise,
    // not fixed. A fractional rate needs the ceiling (44.1 kHz sends 44 or 45
    // frames), and elastic pacing adds one more when shedding drift.
    //
    // Getting this wrong is silent and destructive: embassy-rp rejects a write
    // longer than the endpoint's max, so an undersized endpoint drops every
    // oversized packet and the audio degrades with no error anywhere. A fixed
    // 196 would have been right for 48 kHz and wrong for 96 kHz, which needs 388.
    let frames_max = rate.div_ceil(1000) as usize;
    #[cfg(feature = "clock-locked")]
    let ep_max = (frames_max * CHANNELS_X_BYTES) as u16;
    #[cfg(not(feature = "clock-locked"))]
    let ep_max = ((frames_max + 1) * CHANNELS_X_BYTES) as u16;
    debug_assert!(ep_max <= 1023, "over the full-speed isochronous limit");
    let iso_in = teslamic::build(&mut builder, hid_state, kbd, if3, ep_max, rate);
    let usb = builder.build();

    // ── I2S capture ─────────────────────────────────────────────────────────
    // The packet-stress diagnostic generates its own audio, so it needs no PIO,
    // no PCM2706 and no wiring — just the board and the car.
    #[cfg(not(feature = "packet-stress"))]
    let Pio { mut common, mut sm0, .. } = Pio::new(p.PIO0, Irqs);

    #[cfg(all(not(feature = "clock-locked"), not(feature = "packet-stress")))]
    i2s_pio::slave_rx(&mut common, &mut sm0, p.PIN_2, p.PIN_3, p.PIN_4);
    #[cfg(all(feature = "clock-locked", not(feature = "packet-stress")))]
    i2s_pio::master_rx(
        &mut common,
        &mut sm0,
        p.PIN_2,
        p.PIN_3,
        p.PIN_4,
        embassy_rp::clocks::clk_sys_freq(),
        ADVERTISED.load(core::sync::atomic::Ordering::Relaxed),
    );
    #[cfg(not(feature = "packet-stress"))]
    sm0.set_enable(true);

    // The RP2040-Zero has no plain LED; its only indicator is a WS2812 on
    // GPIO16, driven from PIO1 (PIO0 is I2S).
    #[cfg(not(feature = "rp2040-zero"))]
    let led = Output::new(p.PIN_25, Level::Low);
    #[cfg(feature = "rp2040-zero")]
    // NOTE: `Common` must outlive the LED. embassy-rp's `Drop for Common` calls
    // on_pio_drop(), which resets every PIO-claimed pin's function-select to
    // NULL once the user count falls to 1 — disconnecting GPIO16 from PIO. When
    // this was built inside a `let led = { ... }` block, Common dropped at the
    // end of it: the boot colour clocked out and the WS2812 latched it, then
    // every later write went to a dead pin. That looked exactly like "the status
    // task never runs". Keeping Common bound in main (which never returns) holds
    // the pin.
    let (mut led_common, led_sm) = {
        let Pio { common, sm0, .. } = Pio::new(p.PIO1, Pio1Irqs);
        (common, sm0)
    };
    #[cfg(feature = "rp2040-zero")]
    let mut led = ws2812::Ws2812::new(&mut led_common, led_sm, p.PIN_16);
    #[cfg(feature = "rp2040-zero")]
    led.set(smart_leds::RGB8::new(8, 0, 8));

    spawner.spawn(usb_task(usb).unwrap());
    #[cfg(not(feature = "packet-stress"))]
    spawner.spawn(capture(sm0, p.DMA_CH0).unwrap());
    spawner.spawn(status_task(led).unwrap());
    pump(iso_in, wd).await;
}

type UsbDevice = embassy_usb::UsbDevice<'static, Driver<'static, USB>>;

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice) -> ! {
    usb.run().await
}

/// Producer: pull I2S frames and push them into the pipe.
#[embassy_executor::task]
async fn capture(
    mut sm: embassy_rp::pio::StateMachine<'static, PIO0, 0>,
    dma: embassy_rp::Peri<'static, embassy_rp::peripherals::DMA_CH0>,
) -> ! {
    use core::sync::atomic::Ordering;

    // Each PIO push is one 16-bit sample in the low half of a word; a block is
    // I2S_BLOCK stereo frames.
    let mut raw = [0u32; I2S_BLOCK * 2];
    let mut dma = dma;
    let mut last_block = Instant::now();
    // Stall bookkeeping. A stall is only worth reporting if the link came
    // *back* — see the counting rule where these are used.
    let (mut was_live, mut pending_stall) = (false, false);
    // Starve bookkeeping, per streaming session. `base` is the pipe's cumulative
    // count at the start of the current session; `acc` is what earlier sessions
    // contributed. See the accounting where these are used.
    let (mut starve_base, mut starve_acc) = (0u32, 0u32);
    #[cfg(feature = "clock-locked")]
    let (mut win_sofs, mut last_sof) =
        (0u32, USB_FRAMES.load(core::sync::atomic::Ordering::Relaxed));

    loop {
        // Bound the wait. With no I2S clock the DMA never completes, so without
        // a timeout this task blocks forever and the pipe keeps whatever it last
        // held — which the pump then streams to the car as DC, or as noise if
        // the floating inputs picked any up.
        if embassy_time::with_timeout(
            Duration::from_millis(250),
            sm.rx().dma_pull(dma.reborrow(), &mut raw, false),
        )
        .await
        .is_err()
        {
            // Source gone. Reset clears the held sample to zero and un-primes,
            // so the pump emits real silence and re-primes cleanly when audio
            // comes back.
            PIPE.lock(|p| p.borrow_mut().reset());
            SOURCE_LIVE.store(false, Ordering::Relaxed);
            // Note the stall; do NOT count it yet. Counting here would make the
            // number meaningless three separate ways: this branch re-fires every
            // 250 ms for as long as the source is absent, so it would climb
            // without limit rather than reaching 1; it fires at boot whenever
            // this board powers up before the source; and it fires when you
            // unplug the phone at the end, which is how you ASK to read the
            // report. Every run would show a stall.
            //
            // A stall is only evidence of a fault if the link came back — that
            // is the mid-drive dropout you would have heard. One that never
            // recovers is just the session ending.
            if was_live {
                pending_stall = true;
                was_live = false;
                // Close the session's starve accounting here, at the last moment
                // the source was known good. Everything from now until the next
                // recovery is the buffer draining because the source stopped —
                // which is the disconnect, not a fault.
                starve_acc = faults::STARVED.load(Ordering::Relaxed);
            }
            continue;
        }
        let now = Instant::now();

        // A stalled source (phone unplugged, PCM2706 suspended) leaves the DMA
        // pending; if a block took absurdly long, treat the stream as dead and
        // drop what we have rather than splicing stale audio onto new.
        if now.duration_since(last_block) > Duration::from_millis(200) {
            PIPE.lock(|p| p.borrow_mut().reset());
        }
        last_block = now;
        SOURCE_LIVE.store(true, Ordering::Relaxed);
        // The link recovered, so the stall that preceded it was real.
        if pending_stall {
            faults::bump(&faults::I2S_STALLS);
            pending_stall = false;
        }

        // Starve accounting, done here rather than in the pump because only this
        // task knows whether the source is actually delivering.
        //
        // The pump cannot tell a real underrun from a disconnect: when the
        // source stops, the car keeps asking for a packet every millisecond and
        // the buffer empties in ~5 ms, but the timeout above does not un-prime
        // the pipe for 250 ms. That leaves ~245 underruns generated by every
        // unplug — including the one you do deliberately to read the report,
        // which made code 4 fire on every single run.
        //
        // Rebasing at each session start discards exactly that drain, because
        // the total is only ever sampled at a moment the source is known good.
        let total = PIPE.lock(|p| {
            let st = p.borrow().stats;
            st.overruns.saturating_add(st.underruns)
        });
        if !was_live {
            starve_base = total;
        }
        faults::STARVED.store(
            starve_acc.saturating_add(total.saturating_sub(starve_base)),
            Ordering::Relaxed,
        );
        was_live = true;

        // ── SOF-locked clock trim (clock-locked build only) ─────────────────
        // Steer the PIO divider so the I2S clock becomes a division of the
        // *car's* clock rather than this board's crystal. That is what makes
        // PaceMode::Locked legitimate: fixed 192-byte packets, forever.
        //
        // We steer on **buffer level**, not on measured rate. Measured rate
        // cannot work: the PIO clock divider is 8.8 fixed point, one LSB is
        // ~192 ppm at 48 kHz, so 48000 Hz is not a reachable output and a
        // rate-locked loop limit-cycles at +/-9 frames/s forever — which drains
        // this cushion in under a minute. Buffer level is the integral of the
        // rate error, so a plain proportional law on it is immune to that
        // quantisation: the divider dithers between adjacent values and the
        // level settles at a small fixed offset.
        //
        // Kp = 2000 mHz per frame off target. Simulated across -2000..+2000 ppm
        // of crystal error, worst-case excursion is ~50 frames (1 ms) against a
        // 256-frame cushion, with no oscillation.
        //
        // A car restart needs no special case: the SOF counter simply stops and
        // restarts, and the loop re-converges from wherever it left off.
        #[cfg(feature = "clock-locked")]
        {
            let sof = USB_FRAMES.load(Ordering::Relaxed);
            win_sofs += sof.wrapping_sub(last_sof);
            last_sof = sof;
            if win_sofs >= 100 {
                win_sofs = 0;
                let off = PIPE.lock(|p| p.borrow().off_target()) as i64;
                // Buffer filling => I2S is outrunning the car => command slower.
                let r = ADVERTISED.load(Ordering::Relaxed) as i64;
                let cmd = (r * 1000 - off * 2000)
                    .clamp((r - 2000) * 1000, (r + 2000) * 1000);
                i2s_pio::set_master_rate(
                    &mut sm,
                    embassy_rp::clocks::clk_sys_freq(),
                    cmd as u64,
                );
            }
        }

        PIPE.lock(|p| {
            let mut pipe = p.borrow_mut();
            for f in raw.chunks_exact(2) {
                pipe.push([f[0] as u16 as i16, f[1] as u16 as i16]);
            }
        });
        CAPTURED.store(
            CAPTURED.load(Ordering::Relaxed).wrapping_add(I2S_BLOCK as u32),
            Ordering::Relaxed,
        );
    }
}

/// Consumer: hand the car exactly one packet per USB frame.
///
/// Pacing comes from `write().await` itself: an isochronous IN buffer is
/// consumed once per frame when the host polls it, so the await returns exactly
/// once per frame. Nothing else is needed, and nothing else should be used.
///
/// An earlier version gated each write on the USB frame counter (`SOF_RD`)
/// changing. That hung: embassy-rp never enables SOF tracking, so the counter
/// need not advance, and the loop span forever without ever sending a packet —
/// the device enumerated fine and then blocked any host that tried to open the
/// stream.
async fn pump(
    mut iso_in: impl EndpointIn,
    mut wd: embassy_rp::watchdog::Watchdog,
) -> ! {
    use core::sync::atomic::Ordering;
    let (mut agree_rate, mut agree_count) = (0u32, 0u32);

    let mut buf = [0u8; teslamic::BYTES_PER_FRAME + 8];
    let mut detect = RateDetect::new(1000);
    let mut last_captured = CAPTURED.load(Ordering::Relaxed);
    #[cfg(feature = "packet-stress")]
    let (mut phase, mut stress_hi) = (0u32, false);

    loop {
        iso_in.wait_enabled().await;

        // The car toggles AudioStreaming alt1/alt0 constantly (seen in the
        // on-screen USB spy capture), so do NOT tear anything down when the
        // stream goes idle — just stop writing and wait to be re-enabled.
        loop {
            // The packet-stress diagnostic generates its tone HERE, in the frame
            // it is filling, rather than in a producer task feeding the pipe.
            //
            // A producer on a 1 ms timer cannot work: the consumer runs at
            // exactly the host's 1 kHz frame rate while a timer loop is always
            // a little slower, so the pipe drains and every packet degrades to a
            // held sample. Measured: 97.9% repeated samples, in runs of exactly
            // 47 — one starved packet each.
            #[cfg(feature = "packet-stress")]
            let n = {
                stress_hi = !stress_hi;
                let frames = if stress_hi { 49 } else { 47 };
                for i in 0..frames {
                    let idx = ((phase >> 24) & 0xFF) as usize;
                    let frac = ((phase >> 16) & 0xFF) as i32;
                    let a = SINE256[idx] as i32;
                    let b = SINE256[(idx + 1) & 0xFF] as i32;
                    let v = ((a + (((b - a) * frac) >> 8)) as i16).to_le_bytes();
                    phase = phase.wrapping_add(TONE_PHASE_INC);
                    buf[i * 4] = v[0];
                    buf[i * 4 + 1] = v[1];
                    buf[i * 4 + 2] = v[0];
                    buf[i * 4 + 3] = v[1];
                }
                SOURCE_LIVE.store(true, Ordering::Relaxed);
                frames * 4
            };
            #[cfg(not(feature = "packet-stress"))]
            let n = PIPE.lock(|p| p.borrow_mut().take(&mut buf, MODE));
            if MUTED.load(Ordering::Relaxed) {
                buf[..n].fill(0);
            }
            match iso_in.write(&buf[..n]).await {
                Ok(()) => {}
                // The host deselected the stream; normal, just wait to be
                // re-enabled.
                Err(EndpointError::Disabled) => break,
                // We tried to send more than wMaxPacketSize. That is a firmware
                // bug, not a transient — the packet is silently dropped and the
                // audio degrades to a click at every boundary. Latch it as a
                // fault so the LED says so instead of it looking like drift.
                Err(EndpointError::BufferOverflow) => {
                    OVERSIZE.store(true, Ordering::Relaxed);
                    break;
                }
            }
            // One completed write == one USB frame. This is the frame clock the
            // rest of the firmware uses; see USB_FRAMES.
            // Plain load/store, not fetch_add: thumbv6m (Cortex-M0+) has no
            // atomic read-modify-write. Sound here because this is the only
            // writer, and the only reader just takes a wrapping difference.
            USB_FRAMES.store(USB_FRAMES.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);

            #[cfg(not(feature = "packet-stress"))]
            {
                // Feed the detector both halves: frames captured since the last
                // USB frame, then the USB frame tick itself.
                let c = CAPTURED.load(Ordering::Relaxed);
                detect.on_capture(c.wrapping_sub(last_captured));
                last_captured = c;

                // Mirror the pipe's own tallies into the latch. Sampling beats
                // counting at each site: the pipe already counts these exactly,
                // and this way the two can never disagree.
                // STARVED is owned by `capture`, which can tell a real underrun
                // from a source that has simply stopped. See the accounting there.
                let st = PIPE.lock(|p| p.borrow().stats);
                faults::SLIPS.store(st.adj_up + st.adj_down, Ordering::Relaxed);
                if let Some(hz) = detect.on_usb_frame() {
                    let advertised = ADVERTISED.load(Ordering::Relaxed);
                    match classify(hz) {
                        // The source is running a rate we can advertise, but not
                        // the one we are advertising. Re-enumerate at it rather
                        // than muting: the car handles 32k/44.1k/48k/96k, so
                        // following the source beats refusing it.
                        //
                        // Require several consecutive agreeing measurements —
                        // each is a one-second window — so a transient never
                        // triggers a reset, and never re-enumerate to the rate
                        // we are already on.
                        Some(r) if r != advertised && SUPPORTED_RATES.contains(&r) => {
                            if agree_rate == r {
                                agree_count += 1;
                            } else {
                                agree_rate = r;
                                agree_count = 1;
                            }
                            if agree_count >= 3 {
                                // Carried across the reset so the count survives
                                // the re-enumeration it is recording.
                                wd.set_scratch(0, RATE_MAGIC);
                                wd.set_scratch(1, r);
                                wd.set_scratch(
                                    2,
                                    faults::RATE_CHANGES.load(Ordering::Relaxed) + 1,
                                );
                                wd.trigger_reset();
                            }
                            MUTED.store(true, Ordering::Relaxed);
                        }
                        Some(_) => {
                            agree_count = 0;
                            MUTED.store(false, Ordering::Relaxed);
                        }
                        // Unclassifiable means absent or still starting, which is
                        // not a reason to latch silence.
                        None => {
                            agree_count = 0;
                            MUTED.store(hz > 1000, Ordering::Relaxed);
                        }
                    }
                }
            }
            let _ = &mut detect;
        }
    }
}

/// USB frames elapsed, counted by completed isochronous writes.
///
/// One iso IN write completes per frame, so this advances at exactly the host's
/// frame rate — i.e. it *is* a view of the car's clock, which is what the
/// clock-locked build's control loop needs. Derived from writes rather than the
/// `SOF_RD` register because embassy-rp never enables SOF tracking, so that
/// register cannot be relied on to advance.
static USB_FRAMES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Map the firmware's atomics onto a board-independent status. How it is shown
/// depends on the board: a blink code on a plain LED, or colour on the
/// RP2040-Zero's WS2812 (it has no plain LED — see `status.rs`).
fn current_state() -> status::State {
    use core::sync::atomic::Ordering;
    if MUTED.load(Ordering::Relaxed) || OVERSIZE.load(Ordering::Relaxed) {
        status::State::Fault
    } else if SOURCE_LIVE.load(Ordering::Relaxed) {
        status::State::Ok
    } else if let Some(code) = faults::worst() {
        // The source has gone away, so the live status has nothing to say and
        // the LED is free to report what went wrong while you were driving.
        status::State::Report(code)
    } else {
        status::State::Waiting
    }
}

#[cfg(not(feature = "rp2040-zero"))]
#[embassy_executor::task]
async fn status_task(mut led: Output<'static>) -> ! {
    status::run(&mut led, current_state).await
}

#[cfg(feature = "rp2040-zero")]
#[embassy_executor::task]
async fn status_task(mut led: ws2812::Ws2812<'static, embassy_rp::peripherals::PIO1, 0>) -> ! {
    status::run(&mut led, current_state).await
}

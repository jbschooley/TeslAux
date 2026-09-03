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
use embassy_usb::driver::EndpointIn;
use embassy_usb::Builder;

use core::cell::RefCell;

#[path = "../../../src/audio_pipe.rs"]
mod audio_pipe;
#[path = "../i2s_pio.rs"]
mod i2s_pio;
#[path = "../teslamic.rs"]
mod teslamic;

use audio_pipe::{classify, PaceMode, Pipe, RateDetect};

/// ~10 ms of slack at 48 kHz, half in each direction. Comfortably larger than
/// the I2S DMA block so burst delivery never trips the pacer (see the deadband
/// note in `audio_pipe`).
const RING: usize = 512;

/// Frames per I2S DMA block. Must stay under the pipe's deadband (96 frames at
/// 48 kHz) or the pacer chases its own sampling phase.
const I2S_BLOCK: usize = 64;

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
/// The one rate we advertise to the car. A source running anything else is
/// muted rather than played at the wrong pitch.
const RATE: u32 = teslamic::SAMPLE_RATE;

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

/// Shared between the capture task (producer) and the USB pump (consumer).
static PIPE: Mutex<CriticalSectionRawMutex, RefCell<Pipe<RING>>> =
    Mutex::new(RefCell::new(Pipe::new(RATE)));

/// Set once the measured source rate disagrees with what we told the car. The
/// pump then emits silence: wrong-pitch audio is worse than none, and it makes
/// a misconfigured source obvious instead of mysterious.
static MUTED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// True while I2S frames are actually arriving.
static SOURCE_LIVE: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

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
    let iso_in = teslamic::build(&mut builder, hid_state, kbd, if3);
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
        RATE,
    );
    #[cfg(not(feature = "packet-stress"))]
    sm0.set_enable(true);

    let led = Output::new(p.PIN_25, Level::Low);

    spawner.spawn(usb_task(usb).unwrap());
    #[cfg(not(feature = "packet-stress"))]
    spawner.spawn(capture(sm0, p.DMA_CH0).unwrap());
    #[cfg(feature = "packet-stress")]
    spawner.spawn(tone_source().unwrap());
    spawner.spawn(status(led).unwrap());
    pump(iso_in).await;
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
    let mut detect = RateDetect::new(1000);
    let mut last_block = Instant::now();
    #[cfg(feature = "clock-locked")]
    let (mut win_sofs, mut last_sof) = (0u32, sof_frame());

    loop {
        sm.rx().dma_pull(dma.reborrow(), &mut raw, false).await;
        let now = Instant::now();

        // A stalled source (phone unplugged, PCM2706 suspended) leaves the DMA
        // pending; if a block took absurdly long, treat the stream as dead and
        // drop what we have rather than splicing stale audio onto new.
        if now.duration_since(last_block) > Duration::from_millis(200) {
            PIPE.lock(|p| p.borrow_mut().reset());
            detect.reset();
        }
        last_block = now;
        SOURCE_LIVE.store(true, Ordering::Relaxed);

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
            let sof = sof_frame();
            win_sofs += (sof.wrapping_sub(last_sof) & 0x7ff) as u32;
            last_sof = sof;
            if win_sofs >= 100 {
                win_sofs = 0;
                let off = PIPE.lock(|p| p.borrow().off_target()) as i64;
                // Buffer filling => I2S is outrunning the car => command slower.
                let cmd = ((RATE as i64) * 1000 - off * 2000)
                    .clamp((RATE as i64 - 2000) * 1000, (RATE as i64 + 2000) * 1000);
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
        detect.on_capture(I2S_BLOCK as u32);
    }
}

/// `packet-stress` only: fill the pipe with a 1 kHz tone at exactly 48 frames
/// per millisecond, so the only thing varying is the iso packet size.
#[cfg(feature = "packet-stress")]
#[embassy_executor::task]
async fn tone_source() -> ! {
    use core::sync::atomic::Ordering;
    let mut phase: u32 = 0;
    loop {
        // 48 frames every millisecond: exactly the nominal rate, no drift.
        PIPE.lock(|p| {
            let mut pipe = p.borrow_mut();
            for _ in 0..48 {
                let idx = ((phase >> 24) & 0xFF) as usize;
                let frac = ((phase >> 16) & 0xFF) as i32;
                let a = SINE256[idx] as i32;
                let b = SINE256[(idx + 1) & 0xFF] as i32;
                let v = (a + (((b - a) * frac) >> 8)) as i16;
                phase = phase.wrapping_add(TONE_PHASE_INC);
                pipe.push([v, v]);
            }
        });
        SOURCE_LIVE.store(true, Ordering::Relaxed);
        Timer::after_millis(1).await;
    }
}

/// Consumer: hand the car exactly one packet per USB frame.
async fn pump(mut iso_in: impl EndpointIn) -> ! {
    use core::sync::atomic::Ordering;

    let mut buf = [0u8; teslamic::BYTES_PER_FRAME + 8];
    let mut detect = RateDetect::new(1000);

    loop {
        iso_in.wait_enabled().await;

        // The car toggles AudioStreaming alt1/alt0 constantly (seen in the
        // on-screen USB spy capture), so do NOT tear anything down when the
        // stream goes idle — just stop writing and wait to be re-enabled.
        let mut last_sof = sof_frame();
        loop {
            // Pace off the car's SOF, exactly like the nRF build. This is what
            // makes packets land in their frame rather than bunching.
            let sof = sof_frame();
            if sof == last_sof {
                yield_now().await;
                continue;
            }
            last_sof = sof;

            #[cfg(not(feature = "packet-stress"))]
            if let Some(hz) = detect.on_usb_frame() {
                let ok = classify(hz) == Some(RATE);
                MUTED.store(!ok, Ordering::Relaxed);
            }

            let n = PIPE.lock(|p| p.borrow_mut().take(&mut buf, MODE));
            if MUTED.load(Ordering::Relaxed) {
                buf[..n].fill(0);
            }
            if iso_in.write(&buf[..n]).await.is_err() {
                break; // endpoint disabled underneath us; re-arm
            }
        }
    }
}

/// Read the USB frame counter. Changes once per millisecond, driven by the
/// car's host controller — this is our only view of the car's clock.
fn sof_frame() -> u16 {
    embassy_rp::pac::USB.sof_rd().read().count()
}

/// LED: solid = streaming, slow blink = no source, fast blink = rate mismatch.
#[embassy_executor::task]
async fn status(mut led: Output<'static>) -> ! {
    use core::sync::atomic::Ordering;
    loop {
        if MUTED.load(Ordering::Relaxed) {
            led.toggle();
            Timer::after_millis(100).await;
        } else if SOURCE_LIVE.load(Ordering::Relaxed) {
            led.set_high();
            Timer::after_millis(250).await;
            // Cleared by the next capture; if none arrives the source is gone.
            SOURCE_LIVE.store(false, Ordering::Relaxed);
        } else {
            led.toggle();
            Timer::after_millis(700).await;
        }
    }
}

// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! Phone-facing board: a USB Audio Class 1.0 **speaker** that advertises
//! 48 kHz and nothing else, re-emitting what it receives as I2S.
//!
//! This is the half of the two-board build that the PCM2706 cannot do. Its
//! entire reason to exist is the descriptor: because we own it, we can offer
//! the host exactly one sample rate. A PCM2706 always advertises 32/44.1/48 and
//! its rate list is not in the 57 bytes of descriptor its EEPROM can override,
//! so the phone's choice is unappealable. Here the phone has no choice — it
//! resamples internally if the content is 44.1, which is normal and transparent
//! and, importantly, *its* problem rather than ours.
//!
//! I2S is **slave**: the car board supplies BCK and LRCK, having derived them
//! from the car's SOF. So this board's output is clocked by the car, and the
//! whole chain runs on one clock.
//!
//! # Sync mode
//!
//! The iso OUT endpoint is declared **adaptive**, meaning we accept whatever
//! rate the phone sends and absorb the phone-vs-car difference here, in the
//! elastic pipe. The purer alternative is an asynchronous sink with an explicit
//! feedback endpoint telling the phone our exact rate, which would remove even
//! this correction — but Android's support for UAC1 feedback is inconsistent,
//! and a ScreenMate is an Android box. Absorbing it here is the safe default:
//! a rare one-frame correction between the phone and our own buffer is
//! inaudible, and it keeps the *car* side perfectly fixed-size, which is the
//! side that matters.
//!
//! Pins: GPIO2 DATA (out), GPIO3 BCK (in), GPIO4 LRCK (in). LED on GPIO25.

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
#[cfg(not(feature = "clock-steered"))]
use embassy_rp::pio_programs::i2s::{PioI2sOut, PioI2sOutProgram};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{with_timeout, Duration, Timer};
use embassy_usb::descriptor::{SynchronizationType, UsageType};
use embassy_usb::driver::{Endpoint, EndpointIn, EndpointOut};
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::{Builder, Config, Handler, UsbVersion};

use core::cell::RefCell;

#[path = "../audio_pipe.rs"]
mod audio_pipe;
#[path = "../i2s_pio.rs"]
mod i2s_pio;
#[macro_use]
#[path = "../pins.rs"]
mod pins;
#[path = "../status.rs"]
mod status;
#[path = "../ws2812.rs"]
mod ws2812;

use audio_pipe::{PaceMode, Pipe};

const RATE: u32 = 48_000;
const CHANNELS: usize = 2;
const BYTES_PER_FRAME: usize = 192; // 48 * 2ch * 2B
/// 1024 frames = 21.3 ms, target 512 = 10.7 ms of cushion each way.
///
/// Larger than the car board's, because a *host* is the producer here and hosts
/// bunch packets. iOS in particular can deliver several USB frames at once, and
/// each one moves the level by 48 frames. The deadband below has to exceed that
/// bunching, and the cushion has to comfortably exceed the deadband.
///
/// The added latency is irrelevant: ~5 ms more against Tesla's own ~100 ms.
/// Sized from measurement, not inference: with the clock steered, the peak
/// buffer excursion measured over a full track stayed under 64 frames, so 128
/// gives 2x margin. Ring 128 would give none.
#[cfg(feature = "ultra-low")]
const RING: usize = 256;
#[cfg(all(not(feature = "low-latency"), not(feature = "ultra-low")))]
const RING: usize = 1024;
/// Half the cushion. Ring must stay above ~2x(deadband + one consumer burst) =
/// 2x(192+32) = 448, so 512 is the nearest safe power of two.
#[cfg(all(feature = "low-latency", not(feature = "ultra-low")))]
const RING: usize = 512;
/// Four USB frames. Tolerates a host bunching up to three packets.
const HYSTERESIS: usize = 192;

/// The free-running (non-steered) path paces with `slip`, whose deadband must
/// fit inside the buffer — a deadband at or above the target means the pacer can
/// never correct. The steered path does not slip at all, so it is exempt. This
/// is why `ultra-low` requires `clock-steered`.
#[cfg(not(feature = "clock-steered"))]
const _: () = assert!(
    HYSTERESIS + I2S_BLOCK < RING / 2,
    "slip deadband does not fit this RING; use clock-steered or grow the buffer"
);
#[cfg(all(not(feature = "low-latency"), not(feature = "ultra-low")))]
const I2S_BLOCK: usize = 64;
#[cfg(all(feature = "low-latency", not(feature = "ultra-low")))]
const I2S_BLOCK: usize = 32;
#[cfg(feature = "ultra-low")]
const I2S_BLOCK: usize = 16;
/// 32-bit I2S slots => BCK at 64x fs, matching `slave_rx` and the PCM2706. The
/// 16-bit sample rides in the top half of each word.
const I2S_BIT_DEPTH: u32 = 32;

/// Swap L/R when packing I2S slots.
///
/// **False.** `pan-test` settled this empirically: a tone written into I2S slot 0
/// arrives on the LEFT at the car board's USB output, so the mapping is correct
/// straight through and no swap is needed. An earlier build set this true on a
/// by-ear report of reversed stereo; the pan test showed that was misattributed.
///
/// If stereo ever does appear reversed, re-run `--features pan-test` before
/// changing this — it takes the host, the pipe and this constant out of the path
/// and answers the question directly.
const SWAP_LR: bool = false;

/// `pan-test` only: 997 Hz into I2S slot 0, silence into slot 1.
#[cfg(feature = "pan-test")]
const PAN_INC: u32 = ((997u64 << 32) / 48_000u64) as u32;
#[cfg(feature = "pan-test")]
#[rustfmt::skip]
static PAN_SINE: [i16; 256] = {
    let mut t = [0i16; 256];
    let mut i = 0;
    while i < 256 {
        // Quarter-wave table built at compile time; sin is not const, so this
        // uses a triangle approximation — good enough to hear which side it is.
        let q = (i % 128) as i32;
        let tri = if q < 64 { q * 250 } else { (128 - q) * 250 };
        t[i] = if i < 128 { tri as i16 } else { -(tri as i16) };
        i += 1;
    }
    t
};

const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;
const AUDIO_CLASS: u8 = 0x01;
const SUBCLASS_AUDIOCONTROL: u8 = 0x01;
const SUBCLASS_AUDIOSTREAMING: u8 = 0x02;
const PROTO_UNDEFINED: u8 = 0x00;

// Speaker topology, the mirror of the mic: USB streaming in -> speaker out.
const AC_HEADER: [u8; 7] = [0x01, 0x00, 0x01, 0x1E, 0x00, 0x01, 0x01];

const AC_INPUT_TERMINAL: [u8; 10] = [
    0x02, // INPUT_TERMINAL
    0x01, // bTerminalID = 1
    0x01, 0x01, // wTerminalType = 0x0101 (USB Streaming)
    0x00, CHANNELS as u8, 0x03, 0x00, 0x00, 0x00,
];

const AC_OUTPUT_TERMINAL: [u8; 7] = [
    0x03, // OUTPUT_TERMINAL
    0x02, // bTerminalID = 2
    0x01, 0x03, // wTerminalType = 0x0301 (Speaker)
    0x00, 0x01, 0x00,
];

const AS_GENERAL: [u8; 5] = [0x01, 0x01, 0x01, 0x01, 0x00];

/// One discrete rate: 48000. This is the whole point of this board.
const AS_FORMAT_TYPE_I: [u8; 9] = [
    0x02,
    0x01,
    CHANNELS as u8,
    2,  // bSubframeSize
    16, // bBitResolution
    0x01,
    (RATE & 0xff) as u8,
    ((RATE >> 8) & 0xff) as u8,
    ((RATE >> 16) & 0xff) as u8,
];

/// bmAttributes bit 0 set = **sampling-frequency control present**.
///
/// We only support one rate, so strictly this control is optional. It is here
/// because hosts issue `SET_CUR(SAMPLING_FREQ)` on the endpoint anyway when
/// they open a stream, and a device that STALLs it can be dropped instead of
/// opened. The real TeslaMic advertises this bit too (see `real_mic_dump.md`),
/// which is decent evidence that it is what hosts expect. `SampleRateHandler`
/// answers the request.
const AS_ISO_ENDPOINT: [u8; 5] = [0x01, 0x01, 0x00, 0x00, 0x00];

/// UAC1 endpoint control selector for sampling frequency (`wValue` high byte).
const SAMPLING_FREQ_CONTROL: u8 = 0x01;

/// Answers `SET_CUR` / `GET_CUR` for the endpoint's sampling-frequency control.
///
/// Only 48000 exists, so `SET_CUR` is accepted when it asks for 48000 and
/// rejected otherwise — an explicit refusal, so a host that wants 44.1 learns it
/// cannot have it rather than silently proceeding at the wrong rate.
struct SampleRateHandler;

impl Handler for SampleRateHandler {
    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        if req.request_type != RequestType::Class || req.recipient != Recipient::Endpoint {
            return None;
        }
        // bRequest 0x01 = SET_CUR; wValue high byte selects the control.
        if req.request != 0x01 || (req.value >> 8) as u8 != SAMPLING_FREQ_CONTROL {
            return None;
        }
        if data.len() < 3 {
            return Some(OutResponse::Rejected);
        }
        // 3-byte little-endian sample rate.
        let hz = u32::from(data[0]) | u32::from(data[1]) << 8 | u32::from(data[2]) << 16;
        if hz == RATE {
            Some(OutResponse::Accepted)
        } else {
            Some(OutResponse::Rejected)
        }
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.request_type != RequestType::Class || req.recipient != Recipient::Endpoint {
            return None;
        }
        // bRequest 0x81 = GET_CUR.
        if req.request != 0x81 || (req.value >> 8) as u8 != SAMPLING_FREQ_CONTROL {
            return None;
        }
        if buf.len() < 3 {
            return Some(InResponse::Rejected);
        }
        buf[..3].copy_from_slice(&RATE.to_le_bytes()[..3]);
        Some(InResponse::Accepted(&buf[..3]))
    }
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

static PIPE: Mutex<CriticalSectionRawMutex, RefCell<Pipe<RING>>> =
    Mutex::new(RefCell::new(Pipe::new_with_hysteresis(RATE, HYSTERESIS)));

/// Explicit-feedback value in 10.14 format (samples per USB frame << 14).
/// Starts at the nominal 48.000 and is corrected as soon as the I2S clock is
/// measured against the phone's frame clock.
static FEEDBACK: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new((RATE / 1000) << 14);

/// Blocks to ignore after the pipe primes, while the steering loop pulls the
/// level to target. ~1500 blocks at one per 1.33 ms is about two seconds.
#[cfg(feature = "measure-excursion")]
const SETTLE_BLOCKS: u32 = 1500;

/// `measure-excursion` only: the largest |off_target| seen while streaming.
#[cfg(feature = "measure-excursion")]
static PEAK_OFF: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// False while the car board's I2S clock is absent (car asleep or restarting).
static CLOCK_LIVE: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Run the system clock at 124.8 MHz, not the default 125 MHz.
    //
    // The PIO clock divider is 8.8 fixed point. At 125 MHz the divider needed
    // for 48 kHz is 20.345052, which quantises to 20.343750 — so the I2S runs at
    // 48003.07 Hz, a systematic +64 ppm. Against a host at 48000 that is ~3.1
    // dropped or duplicated frames per second, forever, and the size of each
    // discontinuity scales with signal level: inaudible when quiet, audibly
    // crusty on loud bass.
    //
    // 124.8 MHz makes the divider exactly 20.3125, so fs is exactly 48000.000
    // and only crystal tolerance remains (~1 slip/sec worst case, usually far
    // less). It is PLL-reachable from the 12 MHz crystal: FBDIV 52 -> VCO
    // 624 MHz, POSTDIV 5/1. USB is unaffected — it runs from its own PLL.
    let mut cfg = embassy_rp::config::Config::default();
    cfg.clocks = embassy_rp::clocks::ClockConfig::system_freq(124_800_000)
        .expect("124.8 MHz must be PLL-reachable");
    let p = embassy_rp::init(cfg);
    let driver = Driver::new(p.USB, Irqs);

    let mut config = Config::new(0x1209, 0x0001); // pid.codes test VID/PID
    config.manufacturer = Some("TeslAux");
    config.product = Some("TeslAux Bridge");
    config.serial_number = Some("0001");
    config.bcd_usb = UsbVersion::Two;
    config.max_power = 100;
    config.self_powered = false;
    config.composite_with_iads = false;
    config.max_packet_size_0 = 64;

    static mut CONFIG_DESC: [u8; 256] = [0; 256];
    static mut BOS_DESC: [u8; 32] = [0; 32];
    static mut MSOS_DESC: [u8; 16] = [0; 16];
    static mut CONTROL_BUF: [u8; 256] = [0; 256];

    // SAFETY: taken once, at startup, and outlive `usb` (main never returns).
    let (cd, bd, md, cb) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(CONFIG_DESC),
            &mut *core::ptr::addr_of_mut!(BOS_DESC),
            &mut *core::ptr::addr_of_mut!(MSOS_DESC),
            &mut *core::ptr::addr_of_mut!(CONTROL_BUF),
        )
    };

    let mut builder = Builder::new(driver, config, cd, bd, md, cb);

    static mut RATE_HANDLER: SampleRateHandler = SampleRateHandler;
    // SAFETY: taken once at startup; outlives `usb` (main never returns).
    builder.handler(unsafe { &mut *core::ptr::addr_of_mut!(RATE_HANDLER) });

    // Two topologies, mirror images of each other:
    //
    // * **default (adaptive)** — we are the I2S *master*, and we steer our own
    //   clock to follow the phone's USB frame clock. Nothing has to be
    //   negotiated, so this works with every host including ones that ignore
    //   feedback. This is exactly what a PCM2706 does in hardware; pair it with
    //   the default (elastic) car build.
    //
    // * **clock-locked** — we are the I2S *slave* to a car board that derives
    //   its clock from the car's SOF, and we ask the phone to follow us via an
    //   explicit feedback endpoint. Fixed-size packets reach the car, at the
    //   cost of depending on the host honouring feedback.
    #[cfg(not(feature = "clock-locked"))]
    let iso_out = {
        let mut func = builder.function(AUDIO_CLASS, SUBCLASS_AUDIOCONTROL, PROTO_UNDEFINED);
        {
            let mut ac = func.interface();
            let mut alt = ac.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOCONTROL, PROTO_UNDEFINED, None);
            alt.descriptor(CS_INTERFACE, &AC_HEADER);
            alt.descriptor(CS_INTERFACE, &AC_INPUT_TERMINAL);
            alt.descriptor(CS_INTERFACE, &AC_OUTPUT_TERMINAL);
        }
        let ep = {
            let mut stream = func.interface();
            stream.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOSTREAMING, PROTO_UNDEFINED, None);
            let mut alt1 =
                stream.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOSTREAMING, PROTO_UNDEFINED, None);
            alt1.descriptor(CS_INTERFACE, &AS_GENERAL);
            alt1.descriptor(CS_INTERFACE, &AS_FORMAT_TYPE_I);
            // Adaptive: "send at your rate, we will follow you." No feedback
            // endpoint, nothing for a host to get wrong.
            let ep = alt1.endpoint_isochronous_out(
                None,
                // Headroom for one extra frame. Nominally an adaptive sink
                // receives exactly 48 frames per packet, but hosts vary packet
                // size for their own drift management and a host that sends 49
                // into a 192-byte endpoint has its packet truncated. This is the
                // mirror of the bug that silently dropped every corrected packet
                // on the car board's IN endpoint.
                (BYTES_PER_FRAME + 4) as u16,
                1,
                SynchronizationType::Adaptive,
                UsageType::DataEndpoint,
                &[0x00, 0x00],
            );
            alt1.descriptor(CS_ENDPOINT, &AS_ISO_ENDPOINT);
            ep
        };
        drop(func);
        ep
    };

    #[cfg(feature = "clock-locked")]
    let (iso_out, feedback_ep) = {
        let mut func = builder.function(AUDIO_CLASS, SUBCLASS_AUDIOCONTROL, PROTO_UNDEFINED);
        {
            let mut ac = func.interface();
            let mut alt = ac.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOCONTROL, PROTO_UNDEFINED, None);
            alt.descriptor(CS_INTERFACE, &AC_HEADER);
            alt.descriptor(CS_INTERFACE, &AC_INPUT_TERMINAL);
            alt.descriptor(CS_INTERFACE, &AC_OUTPUT_TERMINAL);
        }
        let eps = {
            let mut stream = func.interface();
            stream.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOSTREAMING, PROTO_UNDEFINED, None);
            let mut alt1 =
                stream.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOSTREAMING, PROTO_UNDEFINED, None);
            alt1.descriptor(CS_INTERFACE, &AS_GENERAL);
            alt1.descriptor(CS_INTERFACE, &AS_FORMAT_TYPE_I);

            // The feedback endpoint is allocated FIRST because the data
            // endpoint's bSynchAddress has to name it, and embassy writes each
            // endpoint descriptor at allocation time — so its address must
            // already exist. That puts the feedback descriptor ahead of the data
            // descriptor, which the spec permits (it defines no ordering within
            // an alt setting, only that a class-specific endpoint descriptor
            // follows its own standard one, which still holds here). If a host
            // ever refuses to enumerate, this ordering is the first suspect.
            //
            // 3 bytes, 10.14 format: the full-speed explicit-feedback encoding.
            // bRefresh = 3 -> the host re-reads every 2^3 = 8 ms.
            let fb = alt1.endpoint_isochronous_in(
                None,
                3,
                1,
                SynchronizationType::NoSynchronization,
                UsageType::FeedbackEndpoint,
                &[0x03, 0x00],
            );
            let fb_addr: u8 = fb.info().addr.into();

            // Asynchronous, not Adaptive: we are telling the phone that our clock
            // is the authority and that it must follow the rate reported on
            // `fb_addr`. That rate is derived from the car board's I2S clock,
            // which is itself locked to the car's SOF — so the phone ends up
            // following the car, and no sample slips anywhere in the chain.
            let ep = alt1.endpoint_isochronous_out(
                None,
                // Room for the host to send one extra frame while it tracks us.
                (BYTES_PER_FRAME + 4) as u16,
                1,
                SynchronizationType::Asynchronous,
                UsageType::DataEndpoint,
                &[0x00, fb_addr],
            );
            alt1.descriptor(CS_ENDPOINT, &AS_ISO_ENDPOINT);
            (ep, fb)
        };
        drop(func);
        eps
    };

    let usb = builder.build();

    let Pio { mut common, mut sm0, .. } = Pio::new(p.PIO0, Irqs);
    // The clock-locked pairing needs this board to be an I2S *slave* (the car
    // board supplies the clock). Upstream provides controller-role drivers only,
    // so building it that way would leave BOTH boards driving BCK and LRCK into
    // each other — two push-pull outputs shorted together, which can damage
    // pins. Refuse to build it rather than let that be flashed by accident.
    //
    // The pairing is not currently needed: Tesla accepts variable packet sizes
    // (verified in the car, 10 minutes clean), so the elastic design works and
    // the clock-locked fallback is unnecessary. Reviving it means proving
    // `i2s_pio::slave_tx` on hardware first, the way `slave_rx` was proven with
    // `i2srx`.
    #[cfg(feature = "clock-locked")]
    compile_error!(
        "source --features clock-locked is disabled: it would drive BCK/LRCK \
         against the car board's outputs. See the comment here."
    );

    // Two ways to drive the I2S link, and they differ in how many clock
    // domains the chain has.
    //
    // `clock-steered` (what the design intends): our I2S clock is trimmed to
    // follow the host, so the phone and the I2S link share one clock. The only
    // crossing left is at the car, and that one is absorbed losslessly by
    // varying the USB packet size — no sample is ever dropped or repeated.
    //
    // Default: upstream's fixed-rate master. Proven, but it cannot retune its
    // divider, so this board free-runs on its crystal and *creates* a second
    // crossing between the host and us. That crossing is what the elastic
    // buffer, the slips and the 10.7 ms cushion are paying for.
    #[cfg(feature = "clock-steered")]
    let mut i2s_sm = {
        let (data, bck, shield, lrck) = source_i2s_pins!(p);
        i2s_pio::master_tx(
            &mut common,
            &mut sm0,
            data,
            bck,
            shield,
            lrck,
            embassy_rp::clocks::clk_sys_freq(),
            RATE,
        );
        sm0.set_enable(true);
        sm0
    };
    // NOTE: this fallback keeps the ORIGINAL three-wire pinout (GP2/3/4) and has
    // no shield. Upstream's driver owns its own side-set and needs BCK and LRCK
    // adjacent, so a pin cannot be placed between them. The soldered two-board
    // layout in `pins` therefore only applies to the `clock-steered` build —
    // build the source with that feature, or this binary will expect the old
    // jumpers.
    #[cfg(not(feature = "clock-steered"))]
    let program = PioI2sOutProgram::new(&mut common);
    #[cfg(not(feature = "clock-steered"))]
    let mut i2s = PioI2sOut::new(
        &mut common,
        sm0,
        p.DMA_CH0,
        p.PIN_2,
        p.PIN_3,
        p.PIN_4,
        RATE,
        I2S_BIT_DEPTH,
        &program,
    );

    // The RP2040-Zero has no plain LED; status goes to its WS2812 on GPIO16,
    // driven from PIO1 since PIO0 is I2S.
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
    #[cfg(not(feature = "clock-steered"))]
    spawner.spawn(i2s_out(i2s).unwrap());
    #[cfg(feature = "clock-steered")]
    spawner.spawn(i2s_out_steered(i2s_sm, p.DMA_CH0).unwrap());
    spawner.spawn(status_task(led).unwrap());
    #[cfg(feature = "clock-locked")]
    spawner.spawn(feedback(feedback_ep).unwrap());
    sink(iso_out).await;
}

type UsbDevice = embassy_usb::UsbDevice<'static, Driver<'static, USB>>;
/// The explicit-feedback endpoint. Named concretely because `#[task]` cannot be
/// generic.
#[cfg(feature = "clock-locked")]
type FeedbackEp = embassy_rp::usb::Endpoint<'static, USB, embassy_rp::usb::In>;

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice) -> ! {
    usb.run().await
}

#[cfg(feature = "clock-locked")]
/// Report our real sample rate to the phone, so it sends at the car's rate.
///
/// This is what makes the chain fully locked: the car board derives its I2S
/// clock from the car's SOF, we are its I2S slave, and this endpoint passes that
/// rate up to the phone. When the car restarts, the car board re-converges and
/// this value follows it automatically — nothing here needs to know.
#[cfg(feature = "clock-locked")]
#[embassy_executor::task]
async fn feedback(mut ep: FeedbackEp) -> ! {
    loop {
        ep.wait_enabled().await;
        loop {
            let v = FEEDBACK.load(core::sync::atomic::Ordering::Relaxed);
            // 10.14, 3 bytes little-endian.
            if ep.write(&v.to_le_bytes()[..3]).await.is_err() {
                break;
            }
        }
    }
}

/// Producer: take iso OUT packets from the phone into the pipe.
async fn sink(mut ep: impl EndpointOut) -> ! {
    let mut buf = [0u8; BYTES_PER_FRAME + 8];
    loop {
        ep.wait_enabled().await;
        loop {
            match ep.read(&mut buf).await {
                Ok(n) => {
                    // One completed read == one USB frame; this is our frame clock.
                    // Plain load/store: thumbv6m has no atomic read-modify-write,
                    // and this is the only writer.
                    USB_FRAMES.store(
                        USB_FRAMES.load(core::sync::atomic::Ordering::Relaxed).wrapping_add(1),
                        core::sync::atomic::Ordering::Relaxed,
                    );
                    PIPE.lock(|p| {
                    let mut pipe = p.borrow_mut();
                    for f in buf[..n].chunks_exact(4) {
                        pipe.push([
                            i16::from_le_bytes([f[0], f[1]]),
                            i16::from_le_bytes([f[2], f[3]]),
                        ]);
                    }
                    })
                }
                // Endpoint disabled (host went to alt 0); wait to be re-enabled
                // rather than tearing anything down.
                Err(_) => break,
            }
        }
        PIPE.lock(|p| p.borrow_mut().reset());
    }
}

/// Consumer: clock frames out over I2S, steering our clock to follow the host.
///
/// The pipe level *is* the phase error between the host's rate and ours, so
/// holding it at target locks our I2S clock to the host. That collapses the
/// phone/I2S crossing entirely — the only one left is at the car, where varying
/// the packet size absorbs it without dropping a sample.
///
/// Steering on level rather than on a measured rate is deliberate, and the same
/// reasoning as the car board's loop: the PIO divider is 8.8 fixed point, so
/// most rates are not reachable exactly and a rate-locked loop would limit-cycle
/// forever. Level is the integral of rate error, so proportional control on it
/// is immune to that quantisation — the divider dithers between adjacent values
/// and the level settles at a small offset.
///
/// Sign check: the pipe fills from USB and drains to I2S, so a *rising* level
/// means our clock is too slow and we command a higher rate. That is inverted
/// relative to the car board, where the pipe fills from I2S and drains to USB.
#[cfg(feature = "clock-steered")]
#[embassy_executor::task]
async fn i2s_out_steered(
    mut sm: embassy_rp::pio::StateMachine<'static, PIO0, 0>,
    dma: embassy_rp::Peri<'static, embassy_rp::peripherals::DMA_CH0>,
) -> ! {
    let mut dma = dma;
    let mut raw = [0u32; I2S_BLOCK * 2];
    let mut ticks = 0u32;
    #[cfg(feature = "pan-test")]
    let mut pan_phase: u32 = 0;
    #[cfg(feature = "measure-excursion")]
    let mut settle: u32 = 0;

    loop {
        let live = PIPE.lock(|p| {
            let mut pipe = p.borrow_mut();
            if pipe.starved() {
                pipe.reset();
            }
            #[cfg(not(feature = "pan-test"))]
            if !pipe.primed() {
                raw = [0u32; I2S_BLOCK * 2];
                return false;
            }
            for i in 0..I2S_BLOCK {
                let f = pipe.pop();
                // 16-bit sample in the top half of a 32-bit slot.
                let (a, b) = if SWAP_LR { (f[1], f[0]) } else { (f[0], f[1]) };
                raw[i * 2] = (a as u16 as u32) << 16;
                raw[i * 2 + 1] = (b as u16 as u32) << 16;

                // Diagnostic: overwrite with a tone in slot 0 and silence in
                // slot 1, ignoring everything upstream. Whichever side it comes
                // out of tells us how the I2S slots map, with no host, no pipe
                // and no SWAP_LR in the way.
                #[cfg(feature = "pan-test")]
                {
                    let idx = ((pan_phase >> 24) & 0xFF) as usize;
                    let v = PAN_SINE[idx];
                    pan_phase = pan_phase.wrapping_add(PAN_INC);
                    raw[i * 2] = (v as u16 as u32) << 16;
                    raw[i * 2 + 1] = 0;
                }
            }
            true
        });
        CLOCK_LIVE.store(live, core::sync::atomic::Ordering::Relaxed);

        // Retune roughly every 100 ms. Kp = 2000 mHz per frame off target,
        // simulated stable across +/-2000 ppm of crystal error.
        // Record how far the level actually wanders. The cushion has to exceed
        // this; every figure used to size it so far has been inferred rather
        // than measured.
        #[cfg(feature = "measure-excursion")]
        {
            use core::sync::atomic::Ordering;
            let (off, primed) = PIPE.lock(|p| {
                let b = p.borrow();
                (b.off_target().unsigned_abs(), b.primed())
            });
            if !primed {
                // Not streaming: an empty pipe reads a full target's worth off
                // target, which is the startup transient rather than anything
                // about the host. Hold the peak cleared until audio is flowing,
                // and clear it again on pause so each run measures fresh.
                PEAK_OFF.store(0, Ordering::Relaxed);
                settle = 0;
            } else if settle < SETTLE_BLOCKS {
                // Give the steering loop time to pull the level to target before
                // believing anything it does.
                settle += 1;
            } else if off > PEAK_OFF.load(Ordering::Relaxed) {
                PEAK_OFF.store(off, Ordering::Relaxed);
            }
        }

        // pan-test bypasses the pipe, so its level is meaningless — steering on
        // it drove the clock to the clamp at 46 kHz, which the car board rightly
        // muted as an unrecognised rate. Hold the nominal rate instead.
        #[cfg(feature = "pan-test")]
        let _ = &mut ticks;
        #[cfg(not(feature = "pan-test"))]
        {
        ticks += 1;
        if ticks >= 75 {
            ticks = 0;
            // Steering an unprimed pipe is meaningless, and not harmless.
            // `off_target` reads a whole target low when the buffer is empty,
            // so the loop winds the clock down to nearly its clamp — 47488 Hz
            // with this cushion — and holds it there for as long as nothing is
            // playing. `pan-test` already carries a note about this: steering on
            // a pipe it bypasses drove the clock to 46 kHz and the car board
            // rightly muted an unrecognised rate.
            //
            // The same thing happens on every pause, and it is audible: a buzz
            // while the phone is stopped, and a car-side pipe that drains at
            // 512 frames a second until it underruns about a quarter of a second
            // later. That underrun is not a fault, but it looks exactly like one
            // — it latched the pipe-watch verdict on a stop rather than on
            // anything that happened while audio was flowing.
            //
            // With nothing to track, hold the nominal rate.
            let (off, primed) = PIPE.lock(|p| {
                let b = p.borrow();
                (b.off_target() as i64, b.primed())
            });
            let cmd = if primed {
                ((RATE as i64) * 1000 + off * 2000)
                    .clamp((RATE as i64 - 2000) * 1000, (RATE as i64 + 2000) * 1000)
            } else {
                (RATE as i64) * 1000
            };
            i2s_pio::set_master_rate(&mut sm, embassy_rp::clocks::clk_sys_freq(), cmd as u64);
        }
        }

        sm.tx().dma_push(dma.reborrow(), &raw, false).await;
    }
}

/// Consumer: clock frames out over I2S.
///
/// The I2S clock is ours (upstream `PioI2sOut` is the controller), so the
/// phone's rate and ours differ by whatever the two crystals disagree on.
/// `Pipe::slip` absorbs that: one frame duplicated or dropped when the buffer
/// wanders off target, roughly twice a second at 20-50 ppm.
#[cfg(not(feature = "clock-steered"))]
#[embassy_executor::task]
async fn i2s_out(
    mut i2s: PioI2sOut<'static, PIO0, 0>,
) -> ! {
    // Two buffers so the DMA always has one in flight while the other is filled.
    let mut bufs = [[0u32; I2S_BLOCK * 2]; 2];
    let mut cur = 0usize;
    let (mut slip_ticks, mut last_slips) = (0u32, 0u32);

    loop {
        let live = PIPE.lock(|p| {
            let mut pipe = p.borrow_mut();

            // Do not start draining until the buffer has filled.
            //
            // The consumer takes a 64-frame block every 1.33 ms and USB delivers
            // 64 frames in the same time, so if we pop from the first block the
            // level hovers at zero and never reaches the priming threshold —
            // whether it ever primes depends on the host happening to burst at
            // stream start, which made it work once and not the next time.
            // Emitting silence until primed lets the buffer build its cushion.
            // A dry buffer means the host has stopped sending — paused, or
            // between tracks — not that we have drifted. Re-arm rather than
            // pacing against nothing: otherwise the pacer slips continuously for
            // as long as the pause lasts, which is what made the LED go amber
            // whenever music was paused.
            if pipe.starved() {
                pipe.reset();
            }
            if !pipe.primed() {
                bufs[cur] = [0u32; I2S_BLOCK * 2];
                return false;
            }

            let adj = pipe.slip(PaceMode::Elastic);

            slip_ticks += 1;
            if slip_ticks >= 750 {
                // ~1 s at one block per 1.33 ms. More than a couple of
                // corrections in that window means the host is not tracking us.
                let now = pipe.stats.adj_up + pipe.stats.adj_down;
                SLIPPING.store(
                    now.wrapping_sub(last_slips) > 2,
                    core::sync::atomic::Ordering::Relaxed,
                );
                last_slips = now;
                slip_ticks = 0;
            }

            // Always hand the DMA a full block; `adj` only changes how many
            // frames come out of the ring to fill it.
            let draw = (I2S_BLOCK as i32 + adj) as usize;
            let buf = &mut bufs[cur];
            let mut last = [0i16; 2];
            for i in 0..draw {
                last = pipe.pop();
                if i < I2S_BLOCK {
                    // 16-bit sample in the top half of a 32-bit slot.
                    let (a, b) = if SWAP_LR { (last[1], last[0]) } else { (last[0], last[1]) };
                    buf[i * 2] = (a as u16 as u32) << 16;
                    buf[i * 2 + 1] = (b as u16 as u32) << 16;
                }
            }
            if adj < 0 {
                let i = I2S_BLOCK - 1;
                let (a, b) = if SWAP_LR { (last[1], last[0]) } else { (last[0], last[1]) };
                buf[i * 2] = (a as u16 as u32) << 16;
                buf[i * 2 + 1] = (b as u16 as u32) << 16;
            }
            pipe.primed()
        });
        CLOCK_LIVE.store(live, core::sync::atomic::Ordering::Relaxed);

        i2s.write(&bufs[cur]).await;
        cur ^= 1;
    }
}

/// USB frames elapsed, counted by completed isochronous reads.
///
/// One iso OUT packet arrives per frame, so this advances at the *phone's* frame
/// rate — which is exactly what the feedback value must be expressed against.
/// Counted from reads rather than the `SOF_RD` register because embassy-rp never
/// enables SOF tracking, so that register cannot be relied on to advance; gating
/// on it hung the car board outright.
static USB_FRAMES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

fn sof_frame() -> u32 {
    USB_FRAMES.load(core::sync::atomic::Ordering::Relaxed)
}

/// Status, and incidentally the answer to "does this host honour USB feedback?".
///
/// If the host follows the rate we report, our buffer sits on target and slip()
/// never fires. If it free-runs, slip corrects a couple of times a second. So a
/// Slipping indication over a 10 s window *is* the test result.
fn current_state() -> status::State {
    use core::sync::atomic::Ordering;

    // In measurement mode the LED reports the peak buffer excursion instead of
    // health, so the cushion can be sized from what the host actually does:
    //   green <64 frames (1.3 ms) | amber <128 (2.7 ms) | red >=128
    #[cfg(feature = "measure-excursion")]
    {
        let peak = PEAK_OFF.load(Ordering::Relaxed);
        return if peak < 64 {
            status::State::Ok
        } else if peak < 128 {
            status::State::Slipping
        } else {
            status::State::Fault
        };
    }

    #[cfg(not(feature = "measure-excursion"))]
    if !CLOCK_LIVE.load(Ordering::Relaxed) {
        return status::State::Waiting;
    }
    if SLIPPING.load(Ordering::Relaxed) {
        status::State::Slipping
    } else {
        status::State::Ok
    }
}

/// Sampled from the slip counters once every 10 s by `status_task`.
static SLIPPING: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[cfg(not(feature = "rp2040-zero"))]
#[embassy_executor::task]
async fn status_task(mut led: Output<'static>) -> ! {
    spawn_slip_watch();
    status::run(&mut led, current_state).await
}

#[cfg(feature = "rp2040-zero")]
#[embassy_executor::task]
async fn status_task(mut led: ws2812::Ws2812<'static, embassy_rp::peripherals::PIO1, 0>) -> ! {
    spawn_slip_watch();
    status::run(&mut led, current_state).await
}

/// Track the slip rate in the background so `current_state` stays cheap.
fn spawn_slip_watch() {
    // Nothing to spawn: the sampling happens in `i2s_out`, which already owns
    // the pipe lock each batch. Kept as a named no-op so the intent is obvious.
}

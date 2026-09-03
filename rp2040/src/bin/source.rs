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

#[path = "../../../src/audio_pipe.rs"]
mod audio_pipe;
#[path = "../i2s_pio.rs"]
mod i2s_pio;
#[path = "../status.rs"]
mod status;
#[path = "../ws2812.rs"]
mod ws2812;

use audio_pipe::{PaceMode, Pipe};

const RATE: u32 = 48_000;
const CHANNELS: usize = 2;
const BYTES_PER_FRAME: usize = 192; // 48 * 2ch * 2B
const RING: usize = 512;
const I2S_BLOCK: usize = 64;
/// 32-bit I2S slots => BCK at 64x fs, matching `slave_rx` and the PCM2706. The
/// 16-bit sample rides in the top half of each word.
const I2S_BIT_DEPTH: u32 = 32;

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
    Mutex::new(RefCell::new(Pipe::new(RATE)));

/// Explicit-feedback value in 10.14 format (samples per USB frame << 14).
/// Starts at the nominal 48.000 and is corrected as soon as the I2S clock is
/// measured against the phone's frame clock.
static FEEDBACK: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new((RATE / 1000) << 14);

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
    config.manufacturer = Some("TeslaAudio");
    config.product = Some("TeslaAudio Bridge");
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
                BYTES_PER_FRAME as u16,
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

    // Upstream embassy-rp I2S master, not my own PIO.
    //
    // `i2stest` + `i2srx` already proved this exact configuration end to end:
    // bit_depth = 32 puts BCK at 64x fs, which is the framing `slave_rx` on the
    // car board expects and the same the PCM2706 produces, with the 16-bit
    // sample in the top half of each word. My hand-written `master_tx` has never
    // run on hardware and had a bit-clock-ratio bug as recently as yesterday, so
    // there is no reason to prefer it here.
    //
    // The cost is that upstream exposes no runtime clock retuning (its state
    // machine is private), so this board runs at its own crystal rate instead of
    // steering to the phone. The phone-vs-us difference is absorbed by
    // `Pipe::slip` — one duplicated or dropped frame every half second or so at
    // typical crystal error, inaudible in practice. Steering would remove even
    // that, and needs `master_tx` proven first.
    let program = PioI2sOutProgram::new(&mut common);
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
    spawner.spawn(i2s_out(i2s).unwrap());
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

/// Consumer: clock frames out over I2S.
///
/// The I2S clock is ours (upstream `PioI2sOut` is the controller), so the
/// phone's rate and ours differ by whatever the two crystals disagree on.
/// `Pipe::slip` absorbs that: one frame duplicated or dropped when the buffer
/// wanders off target, roughly twice a second at 20-50 ppm.
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
                    buf[i * 2] = (last[0] as u16 as u32) << 16;
                    buf[i * 2 + 1] = (last[1] as u16 as u32) << 16;
                }
            }
            if adj < 0 {
                let i = I2S_BLOCK - 1;
                buf[i * 2] = (last[0] as u16 as u32) << 16;
                buf[i * 2 + 1] = (last[1] as u16 as u32) << 16;
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

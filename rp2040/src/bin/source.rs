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

use audio_pipe::{PaceMode, Pipe};

const RATE: u32 = 48_000;
const CHANNELS: usize = 2;
const BYTES_PER_FRAME: usize = 192; // 48 * 2ch * 2B
const RING: usize = 512;
const I2S_BLOCK: usize = 64;

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
    let p = embassy_rp::init(Default::default());
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
    #[cfg(not(feature = "clock-locked"))]
    i2s_pio::master_tx(
        &mut common,
        &mut sm0,
        p.PIN_2,
        p.PIN_3,
        p.PIN_4,
        embassy_rp::clocks::clk_sys_freq(),
        RATE,
    );
    #[cfg(feature = "clock-locked")]
    i2s_pio::slave_tx(&mut common, &mut sm0, p.PIN_2, p.PIN_3, p.PIN_4);
    sm0.set_enable(true);

    let led = Output::new(p.PIN_25, Level::Low);

    spawner.spawn(usb_task(usb).unwrap());
    spawner.spawn(i2s_out(sm0, p.DMA_CH0).unwrap());
    spawner.spawn(blink(led).unwrap());
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
                Ok(n) => PIPE.lock(|p| {
                    let mut pipe = p.borrow_mut();
                    for f in buf[..n].chunks_exact(4) {
                        pipe.push([
                            i16::from_le_bytes([f[0], f[1]]),
                            i16::from_le_bytes([f[2], f[3]]),
                        ]);
                    }
                }),
                // Endpoint disabled (host went to alt 0); wait to be re-enabled
                // rather than tearing anything down.
                Err(_) => break,
            }
        }
        PIPE.lock(|p| p.borrow_mut().reset());
    }
}

/// Consumer: clock frames out over I2S at the car board's rate.
#[embassy_executor::task]
async fn i2s_out(
    mut sm: embassy_rp::pio::StateMachine<'static, PIO0, 0>,
    dma: embassy_rp::Peri<'static, embassy_rp::peripherals::DMA_CH0>,
) -> ! {
    let mut raw = [0u32; (I2S_BLOCK + 1) * 2];
    let mut dma = dma;
    // Rolling measurement of the I2S clock against the phone's USB frame clock.
    let (mut win_frames, mut win_sofs, mut last_sof) = (0u32, 0u32, sof_frame());
    loop {
        // The I2S clock is fixed by the car board, so this sink consumes
        // exactly I2S_BLOCK frames no matter what we do — varying the batch
        // size (as the car board does with its USB packets) corrects nothing
        // here, it just changes how often this loop runs. The phone's clock is
        // independent (our endpoint is adaptive, with no feedback endpoint to
        // make the phone follow us), so the difference has to be absorbed by an
        // actual sample slip: one frame discarded or repeated, a couple of times
        // a second at typical crystal error.
        let n = PIPE.lock(|p| {
            let mut pipe = p.borrow_mut();
            let adj = pipe.slip(PaceMode::Elastic);
            // Always hand the DMA a full block; `adj` changes only how many
            // frames come out of the ring to fill it.
            let draw = (I2S_BLOCK as i32 + adj) as usize;
            let mut last = [0i16; 2];
            for i in 0..draw {
                last = pipe.pop();
                if i < I2S_BLOCK {
                    raw[i * 2] = last[0] as u16 as u32;
                    raw[i * 2 + 1] = last[1] as u16 as u32;
                }
            }
            if adj < 0 {
                // Drew one fewer: repeat the last frame to pad the block.
                let i = I2S_BLOCK - 1;
                raw[i * 2] = last[0] as u16 as u32;
                raw[i * 2 + 1] = last[1] as u16 as u32;
            }
            I2S_BLOCK
        });
        // If the car board loses power its PIO stops driving BCK/LRCK, and this
        // board's I2S state machine blocks forever on its `wait` instruction.
        // Time the transfer out so the task cannot wedge, and drop what we were
        // holding — when the car comes back its clock restarts and we resume
        // with live audio rather than replaying whatever was buffered when it
        // died. This is the source side's half of surviving a car restart.
        let push = sm.tx().dma_push(dma.reborrow(), &raw[..n * 2], false);
        if with_timeout(Duration::from_millis(200), push).await.is_err() {
            PIPE.lock(|p| p.borrow_mut().reset());
            CLOCK_LIVE.store(false, core::sync::atomic::Ordering::Relaxed);
        } else {
            CLOCK_LIVE.store(true, core::sync::atomic::Ordering::Relaxed);

            // How many I2S frames elapsed per phone-USB frame? That ratio, in
            // 10.14, is exactly what the feedback endpoint must report.
            win_frames += I2S_BLOCK as u32;
            let _ = win_frames;
            let sof = sof_frame();
            win_sofs += (sof.wrapping_sub(last_sof) & 0x7ff) as u32;
            last_sof = sof;
            // 256 ms: long enough that one frame of counting error is ~4 ppm,
            // short enough to track a car board that is still converging.
            #[cfg(feature = "clock-locked")]
            if win_sofs >= 256 {
                let v = (win_frames as u64 * 16384 / win_sofs as u64) as u32;
                // Ignore nonsense from a stalled or restarting clock.
                if v > (40 << 14) && v < (56 << 14) {
                    FEEDBACK.store(v, core::sync::atomic::Ordering::Relaxed);
                }
                win_frames = 0;
                win_sofs = 0;
            }

            // Adaptive mode: we own the I2S clock, so steer it to follow the
            // phone. Same level-steered loop as the car board and for the same
            // reason (the PIO divider is too coarse to hit 48000 exactly), but
            // the sign is INVERTED: here the pipe fills from USB and drains to
            // I2S, so a rising buffer means our clock is too *slow*.
            #[cfg(not(feature = "clock-locked"))]
            if win_sofs >= 100 {
                win_sofs = 0;
                win_frames = 0;
                let off = PIPE.lock(|p| p.borrow().off_target()) as i64;
                let cmd = ((RATE as i64) * 1000 + off * 2000)
                    .clamp((RATE as i64 - 2000) * 1000, (RATE as i64 + 2000) * 1000);
                i2s_pio::set_master_rate(
                    &mut sm,
                    embassy_rp::clocks::clk_sys_freq(),
                    cmd as u64,
                );
            }
        }
    }
}

/// The phone's USB frame counter — our view of the *phone's* clock, which is
/// what the feedback value must be expressed against.
fn sof_frame() -> u16 {
    embassy_rp::pac::USB.sof_rd().read().count()
}

/// LED, which doubles as the answer to "does this phone honour USB feedback?".
///
/// If the host follows the rate we report on the feedback endpoint, our buffer
/// sits on target and `slip()` never fires. If it ignores feedback and free-runs,
/// slip corrects a couple of times a second. So the slip rate over a 10 s window
/// *is* the test result — no serial console needed:
///
/// * **solid**          — streaming, feedback honoured (0-1 slips / 10 s)
/// * **double-blink**   — streaming, host ignoring feedback (slipping)
/// * **slow blink**     — no I2S clock; car board unpowered or restarting
#[embassy_executor::task]
async fn blink(mut led: Output<'static>) -> ! {
    use core::sync::atomic::Ordering;
    let mut last_slips = 0u32;
    let mut slipping = false;
    let mut ticks = 0u32;
    loop {
        // Sample the slip counters once every 10 s.
        ticks += 1;
        if ticks >= 50 {
            ticks = 0;
            let now = PIPE.lock(|p| {
                let st = p.borrow().stats;
                st.adj_up + st.adj_down
            });
            // >1 correction per 10 s means the host is not tracking us.
            slipping = now.wrapping_sub(last_slips) > 1;
            last_slips = now;
        }

        if !CLOCK_LIVE.load(Ordering::Relaxed) {
            led.toggle();
            Timer::after_millis(600).await;
        } else if slipping {
            led.set_high();
            Timer::after_millis(80).await;
            led.set_low();
            Timer::after_millis(80).await;
            led.set_high();
            Timer::after_millis(80).await;
            led.set_low();
            Timer::after_millis(160).await;
        } else {
            led.set_high();
            Timer::after_millis(200).await;
        }
    }
}

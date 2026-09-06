// SPDX-License-Identifier: MIT
//! TeslAux, single chip.
//!
//! The two-board rig puts the phone on one RP2040 and the car on another, with
//! an I2S link between them. That link is the only reason the RP2040 build has
//! a PIO driver, two ring buffers, a clock-steering control loop, and three
//! separate clock domains to reconcile.
//!
//! The STM32F407 has **two independent USB device controllers**, so both hosts
//! attach to one chip and the link disappears:
//!
//! ```text
//!   phone --USB--> OTG_HS (PB14/PB15)  ->  Pipe  ->  OTG_FS (PA11/PA12) --USB--> car
//!                  UAC1 speaker                      TeslaMic
//! ```
//!
//! What that buys, concretely:
//!
//!   * **No I2S at all.** Both streams live in one address space, so audio moves
//!     by memcpy instead of over a wire. The whole class of bugs from the RP2040
//!     bring-up — floating inputs streaming noise, PIO divider quantisation,
//!     bit alignment, a jumper twitching in a moving car — cannot occur here.
//!   * **One clock crossing instead of three.** The phone's frame clock versus
//!     the car's frame clock, absorbed losslessly by varying the packet size to
//!     the car. That mechanism is already proven in the car on the RP2040 build.
//!   * **No sample slipping.** Elastic pacing alone covers it, so the deadband
//!     rule that bit the RP2040 three times has one producer to satisfy, not two.
//!
//! What it does *not* buy: the cushion. That is set by how much the phone bunches
//! packets, which is a property of iOS/Android and not of the chip. See `RING`.

#![no_std]
#![no_main]

// The clock-domain logic is HAL-free and has 27 host-run tests, so it is
// compiled straight out of the RP2040 tree rather than copied. Same for the
// TeslaMic descriptors, which are generic over `embassy_usb::driver::Driver`
// and therefore already chip-agnostic — porting them changed nothing.
//
// A path include rather than a shared crate: it leaves the car-proven RP2040
// build completely untouched, and there is exactly one consumer.
#[path = "../../rp2040/src/audio_pipe.rs"]
mod audio_pipe;
#[path = "../../rp2040/src/teslamic.rs"]
mod teslamic;

mod speaker;
mod status;

use defmt_rtt as _;

use audio_pipe::{PaceMode, Pipe};
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, peripherals, usb};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_usb::driver::{EndpointError, EndpointIn, EndpointOut};
use embassy_usb::{Builder, UsbDevice};

use embassy_usb::class::hid::State as HidState;
use teslamic::{If3Handler, KeyboardHandler, SampleRateHandler};
#[cfg(feature = "feature-unit")]
use teslamic::FeatureUnitHandler;

/// Frames in one USB packet at 48 kHz. This is the producer's burst size, and
/// the deadband below has to exceed it.
const FRAMES_PER_PACKET: usize = (teslamic::SAMPLE_RATE / 1000) as usize;

/// Ring capacity in frames. The pipe steers toward `RING / 2`, so half of this
/// is the steady-state latency: 128 frames = 2.67 ms at 48 kHz.
///
/// **Sized from measurement, not inference.** The `measure-excursion` build on
/// the RP2040 source board reported peak buffer excursion under 64 frames for a
/// whole session with an iPhone and again with an Android. The cushion is twice
/// that, which is the margin that actually prevents dropouts.
///
/// There is deliberately no "low latency" variant. On the two-board rig the
/// cushion was a guess with a safety factor bolted on, so a second, tighter
/// build made sense; here the number is measured, and a single configuration is
/// one less thing to get wrong.
const RING: usize = 256;

/// Deadband: how far off target the level must drift before the packet size
/// changes. Must exceed the measured excursion, or corrections fire on sampling
/// phase rather than on real drift.
///
/// A tight deadband is safe *here* in a way it was not on the source board.
/// There the consumer was a fixed I2S clock, so correcting meant `slip()` —
/// duplicating or discarding a frame, a real discontinuity that scales with
/// sample value and was audible on bass. This design has no fixed-rate sink at
/// all: the only consumer is the car's iso IN endpoint, whose packet size we
/// choose, so every correction is `plan_batch()` and is lossless. Correcting
/// often costs nothing but a byte.
const HYSTERESIS: usize = 64;

/// The rule that was violated three separate times during the RP2040 bring-up:
/// **the deadband must exceed the producer's burst, and both must fit inside the
/// cushion.** A deadband at or above the target means the pacer can never
/// correct; a deadband below the burst means every burst trips a correction.
/// Checked here at compile time so it cannot be got wrong again.
const _: () = {
    assert!(
        HYSTERESIS > FRAMES_PER_PACKET,
        "deadband must exceed one USB packet, or every packet trips a correction"
    );
    assert!(
        HYSTERESIS + FRAMES_PER_PACKET < RING / 2,
        "deadband plus one producer burst does not fit in the cushion; grow RING"
    );
    // The cushion is what prevents dropouts, so it carries the safety factor:
    // at least 2x the deadband, i.e. 2x the excursion we expect to see.
    assert!(
        RING / 2 >= 2 * HYSTERESIS,
        "cushion has under 2x margin over the deadband; grow RING"
    );
};

/// Elastic: absorb the phone-vs-car clock difference by varying how many frames
/// go into each packet to the car. Lossless, unlike slipping a sample, and the
/// mechanism already proven in the car on the RP2040 build.
const MODE: PaceMode = PaceMode::Elastic;

static PIPE: Mutex<CriticalSectionRawMutex, RefCell<Pipe<RING>>> =
    Mutex::new(RefCell::new(Pipe::new_with_hysteresis(
        teslamic::SAMPLE_RATE,
        HYSTERESIS,
    )));

/// True while the phone's stream is open and delivering.
static SOURCE_LIVE: AtomicBool = AtomicBool::new(false);
/// Latched: we tried to send more than wMaxPacketSize. A firmware bug, not a
/// transient — the packet is silently dropped and the audio degrades to a click
/// at every boundary, which does not sound like a drift problem at all.
static OVERSIZE: AtomicBool = AtomicBool::new(false);

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
    OTG_HS => usb::InterruptHandler<peripherals::USB_OTG_HS>;
});

/// Car-facing: the onboard USB-C connector, wired to PA11/PA12 through 22R.
type CarDriver = usb::Driver<'static, peripherals::USB_OTG_FS>;
/// Phone-facing: the SparkFun breakout on PB14/PB15, OTG_HS running its
/// internal full-speed PHY.
type PhoneDriver = usb::Driver<'static, peripherals::USB_OTG_HS>;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

#[embassy_executor::task]
async fn usb_car_task(mut d: UsbDevice<'static, CarDriver>) -> ! {
    d.run().await
}

#[embassy_executor::task]
async fn usb_phone_task(mut d: UsbDevice<'static, PhoneDriver>) -> ! {
    d.run().await
}

#[embassy_executor::task]
async fn status_task(mut led: Output<'static>) -> ! {
    status::run(&mut led, current_state).await
}

fn current_state() -> status::State {
    if OVERSIZE.load(Ordering::Relaxed) {
        status::State::Fault
    } else if SOURCE_LIVE.load(Ordering::Relaxed) {
        status::State::Ok
    } else {
        status::State::Waiting
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        // The board has a real 8 MHz crystal, so this is `Oscillator`, NOT the
        // `Bypass` in embassy's stm32f4 example — Bypass expects an external
        // clock signal, and with a crystal fitted the PLL never locks and
        // nothing enumerates at all.
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::Hse;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::Div4,
            mul: PllMul::Mul168,
            divp: Some(PllPDiv::Div2), // 8 / 4 * 168 / 2 = 168 MHz sysclk
            divq: Some(PllQDiv::Div7), // 8 / 4 * 168 / 7 = 48 MHz, exactly, for USB
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::Div1;
        config.rcc.apb1_pre = APBPrescaler::Div4;
        config.rcc.apb2_pre = APBPrescaler::Div2;
        config.rcc.sys = Sysclk::Pll1P;
        config.rcc.mux.clk48sel = mux::Clk48sel::Pll1Q;
    }
    let p = embassy_stm32::init(config);

    // D2 is wired in sink mode, so start HIGH = off.
    let led = Output::new(p.PA1, Level::High, Speed::Low);

    // Two USB stacks means two of everything. Neither `vbus_detection` is
    // enabled: the car port powers the board, and the phone port deliberately
    // has no VBUS wire — the board's +5V rail ties straight to the car's VBUS
    // with no protection, so bridging the phone's VBUS to it would connect two
    // hosts' supplies together.
    let mut car_cfg = usb::Config::default();
    car_cfg.vbus_detection = false;
    let mut phone_cfg = usb::Config::default();
    phone_cfg.vbus_detection = false;

    static mut EP_OUT_CAR: [u8; 512] = [0; 512];
    static mut EP_OUT_PHONE: [u8; 1024] = [0; 1024];
    // SAFETY: each taken once, at startup, and outlives its driver (main never
    // returns).
    let (ep_car, ep_phone) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(EP_OUT_CAR),
            &mut *core::ptr::addr_of_mut!(EP_OUT_PHONE),
        )
    };

    let car_drv = usb::Driver::new_fs(p.USB_OTG_FS, Irqs, p.PA12, p.PA11, ep_car, car_cfg);
    let phone_drv = usb::Driver::new_fs(p.USB_OTG_HS, Irqs, p.PB15, p.PB14, ep_phone, phone_cfg);

    // --- car side: the TeslaMic, byte-for-byte ---
    let iso_in = {
        // The 40-byte serial forces a 162-byte string descriptor, so the control
        // buffer must be >= 256. 128 broke enumeration on the nRF.
        static mut CD: [u8; 256] = [0; 256];
        static mut BD: [u8; 32] = [0; 32];
        static mut MD: [u8; 16] = [0; 16];
        static mut CB: [u8; 256] = [0; 256];
        static mut HID: HidState<'static> = HidState::new();
        static mut KBD: KeyboardHandler = KeyboardHandler;
        static mut IF3: If3Handler = If3Handler;
        static mut SRATE: SampleRateHandler = SampleRateHandler { rate: 0 };
        #[cfg(feature = "feature-unit")]
        static mut FU: FeatureUnitHandler = FeatureUnitHandler;
        #[cfg(feature = "feature-unit")]
        // SAFETY: as below; taken once at startup and outlives `usb`.
        let fu = unsafe { &mut *core::ptr::addr_of_mut!(FU) };
        // SAFETY: taken once at startup; all outlive `usb` (main never returns).
        let (cd, bd, md, cb, hid, kbd, if3, srate) = unsafe {
            (
                &mut *core::ptr::addr_of_mut!(CD),
                &mut *core::ptr::addr_of_mut!(BD),
                &mut *core::ptr::addr_of_mut!(MD),
                &mut *core::ptr::addr_of_mut!(CB),
                &mut *core::ptr::addr_of_mut!(HID),
                &mut *core::ptr::addr_of_mut!(KBD),
                &mut *core::ptr::addr_of_mut!(IF3),
                &mut *core::ptr::addr_of_mut!(SRATE),
            )
        };
        let mut builder = Builder::new(car_drv, teslamic::config(), cd, bd, md, cb);
        // Elastic pacing sends `nominal + 1` frames when shedding drift, so the
        // endpoint must be declared at 196, not the real mic's 192. An endpoint
        // declared at 192 silently drops every corrected packet.
        let (ep, kbd_writer) = teslamic::build(
            &mut builder,
            hid,
            kbd,
            if3,
            srate,
            #[cfg(feature = "feature-unit")]
            fu,
            teslamic::BYTES_PER_FRAME_ELASTIC as u16,
            teslamic::SAMPLE_RATE,
        );
        #[cfg(feature = "feature-unit")]
        spawner.spawn(feature_unit_task().unwrap());
        #[cfg(feature = "if3-log")]
        spawner.spawn(if3_log_task().unwrap());
        #[cfg(feature = "kbd-volume")]
        spawner.spawn(kbd_volume_task(kbd_writer).unwrap());
        #[cfg(not(feature = "kbd-volume"))]
        let _ = kbd_writer;
        spawner.spawn(usb_car_task(builder.build()).unwrap());
        ep
    };

    // --- phone side: a UAC1 speaker at 48 kHz ---
    let iso_out = {
        static mut CD: [u8; 256] = [0; 256];
        static mut BD: [u8; 32] = [0; 32];
        static mut MD: [u8; 16] = [0; 16];
        static mut CB: [u8; 256] = [0; 256];
        static mut RATE_HANDLER: speaker::SampleRateHandler = speaker::SampleRateHandler;
        // SAFETY: as above.
        let (cd, bd, md, cb, rh) = unsafe {
            (
                &mut *core::ptr::addr_of_mut!(CD),
                &mut *core::ptr::addr_of_mut!(BD),
                &mut *core::ptr::addr_of_mut!(MD),
                &mut *core::ptr::addr_of_mut!(CB),
                &mut *core::ptr::addr_of_mut!(RATE_HANDLER),
            )
        };
        let mut builder = Builder::new(phone_drv, speaker::config(), cd, bd, md, cb);
        builder.handler(rh);
        let ep = speaker::build(&mut builder);
        spawner.spawn(usb_phone_task(builder.build()).unwrap());
        ep
    };

    spawner.spawn(status_task(led).unwrap());


    // Producer and consumer both live here rather than in spawned tasks: an
    // `#[embassy_executor::task]` cannot be generic over the endpoint type, and
    // the two drivers have different ones.
    join(sink(iso_out), pump(iso_in)).await;
    {
        unreachable!()
    }
}

/// Producer: iso OUT packets from the phone into the pipe.
async fn sink(mut ep: impl EndpointOut) -> ! {
    let mut buf = [0u8; speaker::BYTES_PER_FRAME + 8];
    loop {
        ep.wait_enabled().await;
        loop {
            // Bound the wait.
            //
            // A host that stops sending without disabling its endpoint produces
            // no error at all — this read simply blocks forever. The pipe then
            // drains and `pop()` holds its last sample, so the car is fed DC
            // rather than silence, and nothing here ever notices the source is
            // gone.
            //
            // The two-board build hit exactly this on its I2S input and fixed it
            // with a timeout of the same length. Removing the I2S link removed
            // the symptom's usual cause, not the failure: any producer can stop
            // without saying so.
            match embassy_time::with_timeout(
                embassy_time::Duration::from_millis(250),
                ep.read(&mut buf),
            )
            .await
            {
                // Timed out: the source stopped without disabling the endpoint.
                // Reset clears the held sample and un-primes, so the pump emits
                // real silence and re-primes cleanly when audio returns. The
                // endpoint is still enabled, so keep waiting on it.
                Err(_) => {
                    SOURCE_LIVE.store(false, Ordering::Relaxed);
                    PIPE.lock(|p| p.borrow_mut().reset());
                }
                Ok(Ok(n)) => {
                    SOURCE_LIVE.store(true, Ordering::Relaxed);
                    PIPE.lock(|p| {
                        let mut pipe = p.borrow_mut();
                        // `chunks_exact` is what makes a frame-alignment error
                        // impossible here, and it is worth saying why: a USB
                        // packet is self-delimiting and a frame is four bytes,
                        // so a short or malformed packet loses its trailing
                        // partial frame and the next packet starts aligned
                        // again. The RP2040's I2S capture had no such boundary —
                        // it pushed single samples into a FIFO, so an odd number
                        // left there rotated every frame for the whole session.
                        for f in buf[..n].chunks_exact(4) {
                            pipe.push([
                                i16::from_le_bytes([f[0], f[1]]),
                                i16::from_le_bytes([f[2], f[3]]),
                            ]);
                        }
                    })
                }
                // Endpoint disabled (host went to alt 0). Normal between tracks
                // and on pause; wait to be re-enabled rather than tearing down.
                Ok(Err(_)) => break,
            }
        }
        SOURCE_LIVE.store(false, Ordering::Relaxed);
        PIPE.lock(|p| p.borrow_mut().reset());
    }
}

/// Discard anything staged in the car endpoint's TX FIFO.
///
/// `embassy-usb` has no API for this — it is a property of the Synopsys core
/// rather than of the USB model — so it goes through the PAC.
fn flush_car_tx_fifo() {
    use embassy_stm32::pac::USB_OTG_FS as R;
    R.grstctl().modify(|w| {
        w.set_txfnum(1);
        w.set_txfflsh(true);
    });
    // Bounded: never spin forever on a peripheral that is not answering.
    let mut spins = 0u32;
    while R.grstctl().read().txfflsh() && spins < 100_000 {
        spins += 1;
    }
}

/// Consumer: hand the car one packet per USB frame, sized to hold the buffer at
/// target.
async fn pump(mut iso_in: impl EndpointIn) -> ! {
    let mut buf = [0u8; teslamic::BYTES_PER_FRAME + 8];
    let mut last_write = embassy_time::Instant::now();
    loop {
        iso_in.wait_enabled().await;

        // Flush the endpoint's TX FIFO now that it has been (re-)enabled.
        //
        // This is the fix for the car, and it belongs here because here is where
        // the damage is done. The car sets alt 1 -> 0 -> 1 in rapid succession —
        // no host that works does this — and a packet staged in the TX FIFO when
        // the endpoint is disabled stays there. Re-enabling does not clear it,
        // the core cannot stage another behind it, and the endpoint goes silent
        // for good: active, correctly configured as isochronous, and never
        // transmitting again.
        //
        // It presents as the *host* refusing to poll us, which is what made it
        // hard to find: the bus keeps clocking, the frame counter advances, the
        // endpoint reports active, and every counter we own looks reasonable.
        // Only DIEPCTL.EPENA staying false through thousands of writes gave it
        // away.
        flush_car_tx_fifo();

        // Drop whatever piled up while the car was not listening.
        //
        // Nothing drains the pipe until the car polls, so it pegs at capacity;
        // the pacer then sheds one frame per packet and stops as soon as the
        // level is inside the deadband, so it never returns to target. The level
        // parks at the deadband edge for the rest of the session, which makes
        // latency depend on nothing but whether the phone or the car came up
        // first — and the car serves a backlog before live audio.
        //
        // Removing the I2S link did not remove this one: it is a property of a
        // producer that runs while the consumer is idle, which both designs have.
        PIPE.lock(|p| p.borrow_mut().trim_to_target());

        // The car toggles AudioStreaming alt1/alt0 constantly (seen in the
        // on-screen USB spy capture), so do NOT tear anything down when the
        // stream goes idle — just stop writing and wait to be re-enabled.
        loop {
            let n = PIPE.lock(|p| p.borrow_mut().take(&mut buf, MODE));
            // Bound the write.
            //
            // An isochronous IN packet is collected once per frame, so a write
            // that has not completed in 20 ms means the host is not collecting
            // — and a host that stops collecting reports nothing, exactly as a
            // host that stops polling reports nothing and a source that stops
            // sending reports nothing. Every one of those has cost this project
            // a bug. Never block on a host indefinitely.
            let attempt =
                embassy_time::with_timeout(embassy_time::Duration::from_millis(20), iso_in.write(&buf[..n]))
                    .await;
            let attempt = match attempt {
                Ok(r) => r,
                Err(_) => {
                    // Belt and braces: the same flush on the recovery path, so
                    // a stall from any other cause cannot become permanent.
                    flush_car_tx_fifo();
                    // Go back and re-arm rather than retrying in place. The car
                    // sets alt 1/0/1/0/1 in quick succession and then stops; if
                    // the driver's endpoint state ends up out of step with that,
                    // retrying the same queued packet forever cannot recover,
                    // whereas re-entering through wait_enabled() might.
                    break;
                }
            };
            match attempt {
                Ok(()) => {
                    // `wait_enabled()` above only returns on an alt-setting
                    // change. A host that keeps the interface selected but stops
                    // polling produces no error at all — the write simply blocks
                    // — so the trim there never re-runs while the pipe fills.
                    // Catch it where it actually shows: a gap between delivered
                    // packets far longer than the one-frame polling interval.
                    let now = embassy_time::Instant::now();
                    if now.duration_since(last_write) > embassy_time::Duration::from_millis(20) {
                        PIPE.lock(|p| p.borrow_mut().trim_to_target());
                    }
                    last_write = now;
                }
                Err(EndpointError::Disabled) => {
                    break;
                }
                Err(EndpointError::BufferOverflow) => {
                    OVERSIZE.store(true, Ordering::Relaxed);
                    break;
                }
            }
        }
    }
}

/// Report what the car does with the Feature Unit, over the ST-Link.
///
/// This is the whole point of running the experiment here rather than on the
/// RP2040: the RP2040's only channel is an LED, which can say "something
/// happened" but not what was asked for or what value was set.
#[cfg(feature = "feature-unit")]
#[embassy_executor::task]
async fn feature_unit_task() -> ! {
    use core::sync::atomic::Ordering;
    use teslamic::feature_unit as fu;
    let mut last = u32::MAX;
    loop {
        let n = fu::REQUESTS.load(Ordering::Relaxed);
        if n != last {
            last = n;
            let raw = fu::LAST_RAW.load(Ordering::Relaxed);
            defmt::info!(
                "feature unit: {} request(s), {} SET_CUR, last bRequest {=u8:#04x} wValue {=u16:#06x}, volume {} (1/256 dB), muted {}",
                n,
                fu::SET_CUR_SEEN.load(Ordering::Relaxed),
                (raw >> 16) as u8,
                raw as u16,
                fu::LAST_VOLUME.load(Ordering::Relaxed),
                fu::MUTED.load(Ordering::Relaxed),
            );
        }
        embassy_time::Timer::after_millis(250).await;
    }
}

/// Print the car's IF3 writes as they arrive.
#[cfg(feature = "if3-log")]
#[embassy_executor::task]
async fn if3_log_task() -> ! {
    use core::sync::atomic::Ordering;
    use teslamic::if3_log;
    let mut next = 0u32;
    loop {
        let count = if3_log::COUNT.load(Ordering::Relaxed);
        // Anything more than a ring behind is gone; say so rather than print it.
        if count.wrapping_sub(next) > if3_log::SLOTS as u32 {
            let lost = count.wrapping_sub(next) - if3_log::SLOTS as u32;
            defmt::warn!("IF3: {} frame(s) missed", lost);
            next = count.wrapping_sub(if3_log::SLOTS as u32);
        }
        while next != count {
            let (req, val, len, bytes) = if3_log::get(next);
            defmt::info!(
                "IF3 #{}: bRequest {=u8:#04x} wValue {=u16:#06x} len {} | {=[u8]:#04x}",
                next,
                req,
                val,
                len,
                bytes[..len.min(16)]
            );
            next = next.wrapping_add(1);
        }
        embassy_time::Timer::after_millis(100).await;
    }
}

/// Ask the car to turn itself up, and see whether it listens.
///
/// The clamp is computed when the volume is set, from what is connected at that
/// moment — unplugging does not lift it, but the next volume change does. So the
/// question is whether this device can cause a volume change at a moment of its
/// choosing. IF3 cannot: it is endpoint-less and the car has never read from it.
/// The keyboard can, if the car acts on Keyboard-page volume usages at all.
///
/// Ten presses, two seconds apart, starting well after enumeration so the car is
/// settled. Watch the car's volume display: if it climbs, this channel works and
/// the clamp becomes something we can time rather than something we suffer.
#[cfg(feature = "kbd-volume")]
#[embassy_executor::task]
async fn kbd_volume_task(
    mut w: embassy_usb::class::hid::HidWriter<
        'static,
        embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>,
        8,
    >,
) -> ! {
    embassy_time::Timer::after_secs(10).await;
    for i in 0..10u32 {
        defmt::info!("keyboard: Volume Up #{}", i + 1);
        #[cfg(not(feature = "kbd-consumer"))]
        teslamic::tap_key(&mut w, teslamic::KEY_VOLUME_UP).await;
        #[cfg(feature = "kbd-consumer")]
        teslamic::tap_consumer(&mut w, teslamic::CONSUMER_VOLUME_UP).await;
        embassy_time::Timer::after_secs(2).await;
    }
    defmt::info!("keyboard: done — did the car's volume move?");
    loop {
        embassy_time::Timer::after_secs(60).await;
    }
}

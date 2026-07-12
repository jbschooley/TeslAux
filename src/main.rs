// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! TeslaMic USB emulator for the Heltec T114 (nRF52840).
//!
//! Goal: make a Tesla show its cabin-microphone ("MIC") overlay icon by
//! enumerating this board as the same USB device the genuine TeslaMic
//! receiver presents.  Based on the descriptor dump from the r/hardwarehacking
//! thread:
//!
//! ```text
//! VID 0x1235  PID 0x0002
//! Manufacturer: "TeslaMic_T004_OTA_231008"
//! Product:      "TeslaMic"
//! Max power:    500 mA
//! IF0  Audio Control
//! IF1  Audio Streaming  (alt0 = idle, alt1 = active)
//!        iso IN endpoint, 192 bytes / 1 ms  =>  48 kHz * 2ch * 16-bit
//! IF2  HID  (interrupt IN, 8 bytes / 1 ms)   [status telemetry]
//! IF3  HID  (control/feature only)           [settings]
//! ```
//!
//! ## What this build does
//!
//! It presents the identity (VID/PID/strings) plus a complete, spec-correct
//! **USB Audio Class 1.0** microphone: an AudioControl interface (microphone
//! input terminal -> USB-streaming output terminal) and an AudioStreaming
//! interface whose active alt-setting advertises 48 kHz / 16-bit / 2-channel
//! PCM over a 192-byte/1 ms isochronous IN endpoint.
//!
//! It also **streams silence**: [`iso_silence_pump`] feeds a 192-byte zero
//! packet per USB frame out the iso endpoint, so to the host this looks like a
//! mic that is actively capturing.  This is done at the register level because
//! embassy-nrf 0.10's USB driver can *allocate* the nRF isochronous endpoint
//! (so it enumerates) but its `EndpointIn::write` asserts `len <= 64` and
//! drives the wrong registers — it cannot push 192-byte iso packets.  The pump
//! carries silence today; the same path carries live samples once an
//! I2S/PDM/SAADC capture fills the buffer.
//!
//! The two HID interfaces from the dump (status telemetry + settings) are
//! omitted for now; if the audio device alone doesn't trigger the icon, add
//! them next.
//!
//! ## Why no SoftDevice
//!
//! The T114 stock image keeps a SoftDevice in flash, but this firmware never
//! calls `sd_softdevice_enable`, so the SD stays dormant and the application
//! owns CLOCK + POWER.  That lets us use embassy-nrf's ordinary USB driver and
//! `HardwareVbusDetect` (VBUS via the POWER peripheral) with no SD plumbing.
//! We force HFXO (external crystal) because the nRF52840 USB peripheral needs
//! the 48 MHz reference derived from it; the internal RC is out of spec for USB.

use embassy_executor::Spawner;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_nrf::{bind_interrupts, peripherals, usb};
use embassy_usb::descriptor::{SynchronizationType, UsageType};
use embassy_usb::{Builder, Config, UsbVersion};

#[cfg(feature = "stream")]
use embassy_futures::yield_now;
#[cfg(feature = "stream")]
use embassy_nrf::gpio::{Input, Pull};
#[cfg(feature = "stream")]
use embassy_nrf::pac;
#[cfg(feature = "stream")]
use embassy_nrf::pac::usbd::vals::Response;
#[cfg(feature = "stream")]
use embassy_time::Timer;

#[cfg(feature = "hid-heartbeat")]
use embassy_futures::join::join3;
#[cfg(feature = "hid-heartbeat")]
use embassy_usb::class::hid::{
    Config as HidConfig, HidBootProtocol, HidSubclass, HidWriter, ReportId, RequestHandler,
    State as HidState,
};
#[cfg(feature = "hid-heartbeat")]
use embassy_usb::control::OutResponse;

// ── usb-spy: onboard-TFT USB control-request logger ─────────────────────────
#[cfg(feature = "usb-spy")]
mod st7789;
#[cfg(feature = "usb-spy")]
use core::cell::RefCell;
#[cfg(feature = "usb-spy")]
use core::fmt::Write as _;
#[cfg(feature = "usb-spy")]
use core::sync::atomic::{AtomicU32, Ordering};
#[cfg(feature = "usb-spy")]
use critical_section::Mutex as CsMutex;
#[cfg(feature = "usb-spy")]
use embassy_nrf::gpio::{Level, Output, OutputDrive};
#[cfg(feature = "usb-spy")]
use embassy_nrf::spim::{self, Spim};
#[cfg(feature = "usb-spy")]
use embassy_time::Delay;
#[cfg(feature = "usb-spy")]
use embassy_usb::control::{InResponse, Recipient, Request, RequestType};
#[cfg(feature = "usb-spy")]
use embassy_usb::driver::Direction;
#[cfg(feature = "usb-spy")]
use embassy_usb::types::InterfaceNumber;
#[cfg(feature = "usb-spy")]
use embassy_usb::Handler;
#[cfg(feature = "usb-spy")]
use embedded_graphics::{
    mono_font::{ascii::FONT_5X8, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
#[cfg(feature = "usb-spy")]
use heapless::{Deque, String, Vec};
#[cfg(feature = "usb-spy")]
use st7789::{Framebuffer, St7789};

// SPIM interrupt for the display bus (TWISPI1). Separate bind so it only exists
// in the usb-spy build.
#[cfg(feature = "usb-spy")]
embassy_nrf::bind_interrupts!(struct SpiIrqs {
    TWISPI1 => spim::InterruptHandler<embassy_nrf::peripherals::TWISPI1>;
});

// Audio format, parameterized at build time by build.rs from
// TESLAMIC_RATE / TESLAMIC_CHANNELS / TESLAMIC_BITS (defaults 48000 / 2 / 16).
// Brings in: SAMPLE_RATE, CHANNELS, BITS, BYTES_PER_SAMPLE,
//            MAX_SAMPLES_PER_FRAME, MAX_BYTES_PER_FRAME.
include!(concat!(env!("OUT_DIR"), "/format.rs"));

// Minimal panic handler: no probe is attached in the car, so there is nothing
// to print to — just idle the core.  (Avoids pulling in defmt / panic-probe.)
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    // POWER/CLOCK IRQ drives HardwareVbusDetect (USB-detected / removed /
    // power-ready events).  Available to us because the SoftDevice is dormant.
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
});

// ── USB Audio Class 1.0 descriptor constants ────────────────────────────────
// (Values from the USB Device Class Definition for Audio Devices, rel. 1.0.)

const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;

// Interface class/subclass codes.
const AUDIO_CLASS: u8 = 0x01;
const SUBCLASS_AUDIOCONTROL: u8 = 0x01;
const SUBCLASS_AUDIOSTREAMING: u8 = 0x02;
const PROTO_UNDEFINED: u8 = 0x00;

// AudioControl class-specific descriptors (payloads start at bDescriptorSubtype;
// embassy prepends bLength + bDescriptorType).
//
// AC interface header: bcdADC 1.00, wTotalLength = 9+12+9 = 30 (0x001E),
// 1 streaming interface in the collection: interface #1.
const AC_HEADER: [u8; 7] = [
    0x01, // HEADER
    0x00, 0x01, // bcdADC = 0x0100
    0x1E, 0x00, // wTotalLength = 30
    0x01, // bInCollection = 1
    0x01, // baInterfaceNr(1) = interface 1 (AudioStreaming)
];

// Input Terminal: ID 1, type 0x0201 (Microphone), CHANNELS channels.
// 12 bytes total once embassy prepends bLength (0x0C) + bDescriptorType.
const AC_INPUT_TERMINAL: [u8; 10] = [
    0x02, // INPUT_TERMINAL
    0x01, // bTerminalID = 1
    0x01, 0x02, // wTerminalType = 0x0201 (Microphone)
    0x00, // bAssocTerminal
    CHANNELS as u8, // bNrChannels
    (CHANNEL_MASK & 0xff) as u8,
    (CHANNEL_MASK >> 8) as u8, // wChannelConfig
    0x00, // iChannelNames
    0x00, // iTerminal
];

// Output Terminal: ID 2, type 0x0101 (USB Streaming), sourced from terminal 1.
const AC_OUTPUT_TERMINAL: [u8; 7] = [
    0x03, // OUTPUT_TERMINAL
    0x02, // bTerminalID = 2
    0x01, 0x01, // wTerminalType = 0x0101 (USB Streaming)
    0x00, // bAssocTerminal
    0x01, // bSourceID = 1 (the microphone input terminal)
    0x00, // iTerminal
];

// AudioStreaming (alt 1) class-specific general descriptor:
// links to terminal 2, 1 ms delay, PCM format tag (0x0001).
const AS_GENERAL: [u8; 5] = [
    0x01, // AS_GENERAL
    0x02, // bTerminalLink = 2 (USB streaming output terminal)
    0x01, // bDelay = 1 frame
    0x01, 0x00, // wFormatTag = 0x0001 (PCM)
];

// Format Type I: CHANNELS channels, BYTES_PER_SAMPLE bytes/subframe, BITS
// resolution, one discrete sample rate = SAMPLE_RATE (little-endian 3-byte).
const AS_FORMAT_TYPE_I: [u8; 9] = [
    0x02, // FORMAT_TYPE
    0x01, // bFormatType = FORMAT_TYPE_I
    CHANNELS as u8,         // bNrChannels
    BYTES_PER_SAMPLE as u8, // bSubframeSize
    BITS,                   // bBitResolution
    0x01,                   // bSamFreqType = 1 discrete frequency
    (SAMPLE_RATE & 0xff) as u8,
    ((SAMPLE_RATE >> 8) & 0xff) as u8,
    ((SAMPLE_RATE >> 16) & 0xff) as u8,
];

// Class-specific AS isochronous audio-data endpoint descriptor.
const AS_ISO_ENDPOINT: [u8; 5] = [
    0x01, // EP_GENERAL
    0x00, // bmAttributes (no sampling-freq / pitch control)
    0x00, // bLockDelayUnits
    0x00, 0x00, // wLockDelay
];

// Minimal vendor-defined HID report descriptor: one 8-byte input report.
// The genuine TeslaMic's real report layout is unknown (not in the dump), so
// this just makes IF2 a valid HID device that can push 8-byte reports; the
// heartbeat content is a separate guess (see the heartbeat loop in `main`).
#[cfg(feature = "hid-heartbeat")]
#[rustfmt::skip]
const HID_REPORT_DESCRIPTOR: [u8; 21] = [
    0x06, 0x00, 0xFF, // Usage Page (Vendor-Defined 0xFF00)
    0x09, 0x01,       // Usage (0x01)
    0xA1, 0x01,       // Collection (Application)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8 bits)
    0x95, 0x08,       //   Report Count (8 fields)
    0x09, 0x01,       //   Usage (0x01)
    0x81, 0x02,       //   Input (Data, Var, Abs)
    0xC0,             // End Collection
];

// HID control-request handler for the IF2 interface.  With no handler embassy
// STALLs GET_REPORT / SET_REPORT, which likely makes the car reject the device.
// We answer everything permissively (feature reads return zeros) — still a blind
// guess at the protocol, but "respond" beats "stall".
#[cfg(feature = "hid-heartbeat")]
struct TeslaHidHandler;

#[cfg(feature = "hid-heartbeat")]
impl RequestHandler for TeslaHidHandler {
    fn get_report(&mut self, _id: ReportId, buf: &mut [u8]) -> Option<usize> {
        let n = buf.len().min(8);
        buf[..n].fill(0);
        Some(n)
    }
    fn set_report(&mut self, _id: ReportId, _data: &[u8]) -> OutResponse {
        OutResponse::Accepted
    }
    fn get_idle_ms(&mut self, _id: Option<ReportId>) -> Option<u32> {
        Some(0) // indefinite — don't stall GET_IDLE
    }
}

// ── usb-spy: shared log + control-request spy + TFT render task ─────────────
#[cfg(feature = "usb-spy")]
const LOG_CAP: usize = 40;
#[cfg(feature = "usb-spy")]
static LOG: CsMutex<RefCell<Deque<String<64>, LOG_CAP>>> =
    CsMutex::new(RefCell::new(Deque::new()));
#[cfg(feature = "usb-spy")]
static LOG_SEQ: AtomicU32 = AtomicU32::new(0);

// Freeze the log once full: keep the FIRST LOG_CAP events (the connect-time
// handshake) instead of a rolling window, so the endless audio-alt toggling
// can't evict the interesting early requests.
#[cfg(feature = "usb-spy")]
fn log_push(line: String<64>) {
    let mut added = false;
    critical_section::with(|cs| {
        let mut q = LOG.borrow_ref_mut(cs);
        if !q.is_full() {
            let _ = q.push_back(line);
            added = true;
        }
    });
    if added {
        LOG_SEQ.fetch_add(1, Ordering::Relaxed);
    }
}

// The car toggles AudioStreaming alt1/alt0 forever; log only the first few so
// they don't crowd out the HID handshake.
#[cfg(feature = "usb-spy")]
static SET_ALT_COUNT: AtomicU32 = AtomicU32::new(0);
// The car writes many chunked SET_REPORTs to IF3; log the first few (to see the
// payload structure) then suppress so we can see what it does AFTER the writes.
#[cfg(feature = "usb-spy")]
static SET_REPORT_COUNT: AtomicU32 = AtomicU32::new(0);

// Reconstruct the bmRequestType byte for a compact, readable log line.
#[cfg(feature = "usb-spy")]
fn bm_request_type(req: &Request) -> u8 {
    let dir = match req.direction {
        Direction::In => 0x80,
        Direction::Out => 0x00,
    };
    let ty = match req.request_type {
        RequestType::Standard => 0,
        RequestType::Class => 1,
        RequestType::Vendor => 2,
        RequestType::Reserved => 3,
    } << 5;
    let rec = match req.recipient {
        Recipient::Device => 0,
        Recipient::Interface => 1,
        Recipient::Endpoint => 2,
        Recipient::Other => 3,
        _ => 4,
    };
    dir | ty | rec
}

/// USB spy: observes (never consumes — always returns `None`) every delegated
/// control request plus config / alt-setting changes, pushing a one-line
/// summary to the on-screen log.  Registered before the HID handler so it sees
/// requests the HID class will go on to answer.
#[cfg(feature = "usb-spy")]
struct SpyHandler;
#[cfg(feature = "usb-spy")]
impl Handler for SpyHandler {
    fn configured(&mut self, configured: bool) {
        let mut s = String::new();
        let _ = write!(s, "CONFIGURED={}", configured as u8);
        log_push(s);
    }
    fn set_alternate_setting(&mut self, iface: InterfaceNumber, alt: u8) {
        // Only log the first few — the car toggles this forever.
        if SET_ALT_COUNT.fetch_add(1, Ordering::Relaxed) < 4 {
            let mut s = String::new();
            let _ = write!(s, "SET_IF if{} alt{}", iface.0, alt);
            log_push(s);
        }
    }
    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        let mut s = String::new();
        if req.request == 0x09 {
            // HID SET_REPORT: compact form so the payload fits on screen; log only
            // the first several so the chunked-write flood doesn't hide what the
            // car does afterward.
            if SET_REPORT_COUNT.fetch_add(1, Ordering::Relaxed) < 6 {
                let _ = write!(s, "S i{} l{} ", req.index, req.length);
                for b in data.iter().take(20) {
                    let _ = write!(s, "{:02x}", b);
                }
                log_push(s);
            }
        } else {
            let _ = write!(
                s,
                "O {:02x} r{:02x} v{:04x} i{} l{} ",
                bm_request_type(&req),
                req.request,
                req.value,
                req.index,
                req.length
            );
            for b in data.iter().take(9) {
                let _ = write!(s, "{:02x}", b);
            }
            log_push(s);
        }
        None
    }
    fn control_in<'a>(&'a mut self, req: Request, _buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        let mut s = String::new();
        let _ = write!(
            s,
            "I {:02x} r{:02x} v{:04x} i{} l{}",
            bm_request_type(&req),
            req.request,
            req.value,
            req.index,
            req.length
        );
        log_push(s);
        None
    }
}

#[cfg(feature = "usb-spy")]
type DisplayDriver = St7789<
    Spim<'static>,
    Output<'static>,
    Output<'static>,
    Output<'static>,
    Output<'static>,
    Delay,
>;

/// Bring up the TFT and continuously render the control-request log.
#[cfg(feature = "usb-spy")]
#[embassy_executor::task]
async fn display_task(mut disp: DisplayDriver, mut backlight: Output<'static>) {
    // 64 KB framebuffer in a static so it doesn't blow the task's stack.
    static mut FB: Framebuffer = Framebuffer::new();
    let fb: &mut Framebuffer = unsafe { &mut *core::ptr::addr_of_mut!(FB) };

    disp.init().await;
    let _ = fb.clear(Rgb565::BLACK);
    disp.flush(&mut *fb).await;
    backlight.set_low(); // active-low: backlight on

    let title = MonoTextStyle::new(&FONT_5X8, Rgb565::YELLOW);
    let bodyc = MonoTextStyle::new(&FONT_5X8, Rgb565::GREEN);

    let mut last_seq = u32::MAX;
    loop {
        let seq = LOG_SEQ.load(Ordering::Relaxed);
        if seq != last_seq {
            last_seq = seq;

            // Snapshot the log out of the critical section, then draw.
            let mut lines: Vec<String<64>, LOG_CAP> = Vec::new();
            critical_section::with(|cs| {
                for l in LOG.borrow_ref(cs).iter() {
                    let _ = lines.push(l.clone());
                }
            });

            let _ = fb.clear(Rgb565::BLACK);
            let _ = Text::new("TeslaMic USB spy", Point::new(2, 7), title).draw(&mut *fb);
            // Show the NEWEST ~15 events (the tail) — with the SET_REPORT and
            // audio-alt floods suppressed, this reveals what the car does AFTER
            // the config writes (e.g. any GET_REPORT read-back).
            let start = lines.len().saturating_sub(15);
            let mut y = 17;
            for line in &lines[start..] {
                let _ = Text::new(line, Point::new(2, y), bodyc).draw(&mut *fb);
                y += 8;
            }
            disp.flush(&mut *fb).await;
        }
        Timer::after_millis(150).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Force HFXO: USB needs the crystal-derived 48 MHz reference.  Legal to set
    // here because the SoftDevice is dormant and we own the CLOCK peripheral.
    let mut nrf_config = embassy_nrf::config::Config::default();
    nrf_config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(nrf_config);

    // USB driver + hardware VBUS detection (POWER peripheral).
    let vbus = HardwareVbusDetect::new(Irqs);
    let driver = Driver::new(p.USBD, Irqs, vbus);

    // ── Device identity: clone the TeslaMic ────────────────────────────────
    let mut config = Config::new(0x1235, 0x0002);
    config.manufacturer = Some("TeslaMic_T004_OTA_231008");
    config.product = Some("TeslaMic");
    config.bcd_usb = UsbVersion::Two; // report USB 2.00
    config.max_power = 500; // mA, matches the dump
    config.self_powered = false;
    // Plain (non-IAD) composite device: class codes live on the interfaces.
    config.composite_with_iads = false;
    config.device_class = 0x00;
    config.device_sub_class = 0x00;
    config.device_protocol = 0x00;
    config.max_packet_size_0 = 64;

    // HID class state + request handler — declared before the builder/buffers so
    // they outlive `usb` (which holds handlers that borrow them).
    #[cfg(feature = "hid-heartbeat")]
    let mut hid_state = HidState::new();
    #[cfg(feature = "hid-heartbeat")]
    let mut hid_handler = TeslaHidHandler;
    // Second HID interface (IF3) — the genuine TeslaMic exposes two.
    #[cfg(feature = "hid-heartbeat")]
    let mut hid_state3 = HidState::new();
    #[cfg(feature = "hid-heartbeat")]
    let mut hid_handler3 = TeslaHidHandler;
    #[cfg(feature = "usb-spy")]
    let mut spy = SpyHandler;

    // Descriptor / control buffers.  Live on main's stack; main never returns
    // (usb.run() loops forever) so they outlive every use.
    let mut config_descriptor = [0u8; 256];
    let mut bos_descriptor = [0u8; 32];
    let mut msos_descriptor = [0u8; 16];
    let mut control_buf = [0u8; 128];

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // ── Build the Audio function (IF0 = control, IF1 = streaming) ──────────
    let mut func = builder.function(AUDIO_CLASS, SUBCLASS_AUDIOCONTROL, PROTO_UNDEFINED);

    // IF0: AudioControl (topology only, no endpoints).
    {
        let mut ac = func.interface();
        let mut alt = ac.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOCONTROL, PROTO_UNDEFINED, None);
        alt.descriptor(CS_INTERFACE, &AC_HEADER);
        alt.descriptor(CS_INTERFACE, &AC_INPUT_TERMINAL);
        alt.descriptor(CS_INTERFACE, &AC_OUTPUT_TERMINAL);
    }

    // IF1: AudioStreaming.
    {
        let mut stream = func.interface();
        // alt 0 — zero-bandwidth idle setting (no endpoints).
        stream.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOSTREAMING, PROTO_UNDEFINED, None);
        // alt 1 — active setting with the isochronous IN endpoint.
        let mut alt1 =
            stream.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOSTREAMING, PROTO_UNDEFINED, None);
        alt1.descriptor(CS_INTERFACE, &AS_GENERAL);
        alt1.descriptor(CS_INTERFACE, &AS_FORMAT_TYPE_I);
        // Iso IN, 1 ms interval, wMaxPacketSize = the largest frame this format
        // needs (ceil(rate/1000) * channels * bytes).  On the nRF this lands on
        // EP8 (0x88); the exact number doesn't match the genuine 0x84 but hosts
        // key on the class/format, not the address.  extra_fields = [bRefresh,
        // bSynchAddress] makes this the 9-byte UAC audio-endpoint form.
        let _iso_in = alt1.endpoint_isochronous_in(
            None,
            MAX_BYTES_PER_FRAME as u16,
            1,
            SynchronizationType::Asynchronous,
            UsageType::DataEndpoint,
            &[0x00, 0x00],
        );
        // Class-specific AS iso endpoint descriptor follows the standard one.
        alt1.descriptor(CS_ENDPOINT, &AS_ISO_ENDPOINT);
    }

    drop(func);

    // Register the spy FIRST so it observes every delegated control request
    // before the HID class handler (added next) answers it.
    #[cfg(feature = "usb-spy")]
    builder.handler(&mut spy);

    // HID interface (IF2): 8-byte interrupt IN, matching the genuine TeslaMic's
    // telemetry endpoint.  Added under `hid-heartbeat`; the heartbeat loop below
    // streams reports the car's status watchdog may be waiting for.  (`hid_state`
    // is declared up top so it outlives `usb`, which holds a handler borrowing it.)
    #[cfg(feature = "hid-heartbeat")]
    let mut hid_writer: HidWriter<'_, _, 8> = HidWriter::new(
        &mut builder,
        &mut hid_state,
        HidConfig {
            report_descriptor: &HID_REPORT_DESCRIPTOR,
            request_handler: Some(&mut hid_handler),
            poll_ms: 1,
            max_packet_size: 8,
            hid_subclass: HidSubclass::No,
            hid_boot_protocol: HidBootProtocol::None,
        },
    );
    // IF3: second HID interface (matching the genuine device's interface tree).
    #[cfg(feature = "hid-heartbeat")]
    let mut hid_writer3: HidWriter<'_, _, 8> = HidWriter::new(
        &mut builder,
        &mut hid_state3,
        HidConfig {
            report_descriptor: &HID_REPORT_DESCRIPTOR,
            request_handler: Some(&mut hid_handler3),
            poll_ms: 1,
            max_packet_size: 8,
            hid_subclass: HidSubclass::No,
            hid_boot_protocol: HidBootProtocol::None,
        },
    );

    let mut usb = builder.build();

    // Feed the isochronous IN endpoint at the register level.  embassy-nrf's
    // driver can't write the iso endpoint (see module docs), so this task
    // drives USBD.ISOIN directly.  It waits until the host activates
    // AudioStreaming alt-1 before sending anything.
    //
    // The user button (P1_10, active-low) only matters in the `sine-button`
    // build; we configure it unconditionally (harmless) so one task serves all
    // stream variants.  Task macro returns a Result; Err only if the pool is
    // already occupied, impossible on first spawn.
    #[cfg(feature = "stream")]
    {
        let button = Input::new(p.P1_10, Pull::Up);
        spawner.spawn(iso_pump(button).unwrap());
    }
    #[cfg(not(feature = "stream"))]
    let _ = &spawner;

    // Bring up the onboard ST7789 TFT and stream the spy log to it.
    #[cfg(feature = "usb-spy")]
    {
        let mut spicfg = spim::Config::default();
        spicfg.frequency = spim::Frequency::M8;
        spicfg.mode = spim::MODE_0;
        let dspi = Spim::new_txonly(p.TWISPI1, SpiIrqs, p.P1_08, p.P1_09, spicfg);
        // nRF52840 SPIM-SCK PIN_CNF fixup for P1.08 (see wireless-performer-fw):
        // force PIN_CNF[8].INPUT so SCK toggles correctly.
        unsafe {
            core::ptr::write_volatile(0x5000_0A20 as *mut u32, 0x301);
        }
        let cs = Output::new(p.P0_11, Level::High, OutputDrive::HighDrive);
        let dc = Output::new(p.P0_12, Level::Low, OutputDrive::HighDrive);
        let rst = Output::new(p.P0_02, Level::High, OutputDrive::HighDrive);
        let vtft = Output::new(p.P0_03, Level::High, OutputDrive::Standard); // active-low gate
        let backlight = Output::new(p.P0_15, Level::High, OutputDrive::Standard); // active-low
        let disp = St7789::new(dspi, cs, dc, rst, vtft, Delay);
        spawner.spawn(display_task(disp, backlight).unwrap());
    }

    // Run the device: handles enumeration + control transfers forever.  With
    // the HID heartbeat enabled, run it concurrently with the report stream.
    #[cfg(feature = "hid-heartbeat")]
    {
        // Heartbeat on each HID interface. Content is a GUESS (real 8-byte
        // layout isn't in the dump); a rolling counter keeps the stream visibly
        // "alive" for any liveness watchdog.
        let hb2 = async {
            let mut report = [0u8; 8];
            let mut n: u8 = 0;
            loop {
                hid_writer.ready().await;
                report[0] = n;
                n = n.wrapping_add(1);
                if hid_writer.write(&report).await.is_err() {
                    Timer::after_millis(2).await;
                }
            }
        };
        let hb3 = async {
            let mut report = [0u8; 8];
            let mut n: u8 = 0;
            loop {
                hid_writer3.ready().await;
                report[0] = n;
                n = n.wrapping_add(1);
                if hid_writer3.write(&report).await.is_err() {
                    Timer::after_millis(2).await;
                }
            }
        };
        join3(usb.run(), hb2, hb3).await;
    }
    #[cfg(not(feature = "hid-heartbeat"))]
    usb.run().await;
}

// 256-point 1-cycle sine, i16, ~0.49 full-scale.  Indexed by the top 8 bits of
// a 32-bit phase accumulator so the tone stays continuous at any sample rate.
// Generated; see README.
#[cfg(feature = "stream")]
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

// Test-tone frequency and its per-sample phase increment (freq * 2^32 / rate).
#[cfg(feature = "stream")]
const TONE_HZ: u32 = 1000;
#[cfg(feature = "stream")]
const PHASE_INC: u32 = (((TONE_HZ as u64) << 32) / SAMPLE_RATE as u64) as u32;

// `sweep` build: log-spaced tones, each held SWEEP_STEP_FRAMES USB frames
// (~1 ms each) before advancing, restarting from the bottom on each button hold.
#[cfg(feature = "stream")]
const SWEEP_FREQS: [u32; 10] = [50, 100, 200, 400, 800, 1600, 3150, 6300, 12500, 20000];
#[cfg(feature = "stream")]
const SWEEP_STEP_FRAMES: u32 = 350; // ~0.35 s per tone

/// Arm the ISO IN endpoint's EasyDMA with a `len`-byte packet at `ptr` and kick
/// the transfer.  The hardware sends it on the next IN token, then raises
/// EVENTS_ENDISOIN.
#[cfg(feature = "stream")]
#[inline]
fn arm_iso(usbd: pac::usbd::Usbd, ptr: u32, len: u16) {
    usbd.isoin().ptr().write_value(ptr);
    usbd.isoin().maxcnt().write(|w| w.set_maxcnt(len));
    usbd.events_endisoin().write_value(0);
    // Ensure PTR/MAXCNT land before STARTISOIN reads them.
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    usbd.tasks_startisoin().write_value(1);
}

/// Stream one PCM packet per USB frame out the isochronous IN endpoint, making
/// this behave like a mic that is actively capturing at the built-in format
/// (SAMPLE_RATE / CHANNELS / BITS).
///
/// - Default (`stream`): silence.
/// - `sine-button`: a 1 kHz tone plays on ONE channel while the user button
///   (P1_10, active-low) is held, silence when released.  Each new press
///   advances the channel (0 -> 1 -> ... -> CHANNELS-1 -> 0); at 2ch that's the
///   left/right alternation, and with more channels it steps through them so you
///   can check how the car maps each one.
///
/// A 32-bit phase accumulator keeps the tone continuous at any sample rate, and
/// a sample-count accumulator handles fractional rates (e.g. 44/45 samples per
/// frame for 44.1 kHz).  We own the ISO registers outright — embassy-nrf's USB
/// driver never touches ISOIN / ISOINCONFIG / STARTISOIN / ENDISOIN — and arm
/// exactly once per frame right after SOF so a re-arm never corrupts an
/// in-flight packet.
///
/// To carry *real* audio, fill `buf` from an I2S / PDM / SAADC DMA capture
/// instead of the tone generator.
#[cfg(feature = "stream")]
#[embassy_executor::task]
async fn iso_pump(button: Input<'static>) {
    let usbd = pac::USBD;

    // One frame's PCM, sized for the biggest frame this format needs.  Lives in
    // the task future (RAM) — a stable address EasyDMA can read.
    let mut buf = [0u8; MAX_BYTES_PER_FRAME];

    let mut phase: u32 = 0; // tone phase accumulator
    let mut samp_accum: u32 = 0; // fractional-rate sample-count accumulator

    // Debounced button; each fresh press advances the channel the tone plays on.
    let mut stable_pressed = false;
    let mut debounce: u8 = 0;
    let mut sel: usize = CHANNELS - 1; // first press advances to channel 0

    // `sweep` build state: current frequency step + frames spent on it.
    let mut sweep_frame: u32 = 0;
    let mut sweep_idx: usize = 0;

    // Late frames answer the IN token with a zero-length packet (valid "no
    // samples this frame" for isochronous) instead of not responding.
    usbd.isoinconfig().write(|w| w.set_response(Response::ZERO_DATA));

    loop {
        // Idle until the host selects AudioStreaming alt-setting 1 — embassy
        // enables the iso IN endpoint, setting EPINEN bit 8 (`isoin`).
        while !usbd.epinen().read().isoin() {
            Timer::after_millis(4).await;
        }

        // Arm exactly ONE packet per USB frame, immediately after Start-of-Frame,
        // never mid-frame: a re-arm DMAs into the single ISO buffer, and doing
        // that while the packet is clocking out (~128 us for a full one) corrupts
        // it (audible scratchiness).  SOF-arming keeps the DMA before the IN
        // token and clear of the previous transmission.
        usbd.events_sof().write_value(0);
        while usbd.epinen().read().isoin() {
            if usbd.events_sof().read() != 0 {
                usbd.events_sof().write_value(0);

                // Debounce the button once per frame; a fresh press steps the
                // active channel.
                let raw = button.is_low(); // active-low
                if raw == stable_pressed {
                    debounce = 0;
                } else {
                    debounce += 1;
                    if debounce >= 12 {
                        stable_pressed = raw;
                        debounce = 0;
                        if stable_pressed {
                            sel = (sel + 1) % CHANNELS;
                        }
                    }
                }
                let tone =
                    (cfg!(feature = "sine-button") || cfg!(feature = "sweep")) && stable_pressed;

                // Per-frame phase increment: fixed 1 kHz, or the current sweep
                // step (restarting from the bottom each time the button is held).
                let inc = if cfg!(feature = "sweep") {
                    if stable_pressed {
                        sweep_frame += 1;
                        if sweep_frame >= SWEEP_STEP_FRAMES {
                            sweep_frame = 0;
                            sweep_idx = (sweep_idx + 1) % SWEEP_FREQS.len();
                        }
                    } else {
                        sweep_frame = 0;
                        sweep_idx = 0;
                    }
                    (((SWEEP_FREQS[sweep_idx] as u64) << 32) / SAMPLE_RATE as u64) as u32
                } else {
                    PHASE_INC
                };

                // sweep -> all channels; sine-button -> just the selected channel.
                let (ch_lo, ch_hi) = if cfg!(feature = "sweep") {
                    (0, CHANNELS)
                } else {
                    (sel, sel + 1)
                };

                // Audio frames this USB frame (alternates for fractional rates).
                samp_accum += SAMPLE_RATE;
                let nsamp = (samp_accum / 1000) as usize;
                samp_accum %= 1000;
                let nbytes = nsamp * CHANNELS * BYTES_PER_SAMPLE;

                // Zero the frame, then lay the tone into the active channel(s).
                // Phase advances every sample (even when silent) so the tone
                // resumes phase-continuous.
                for b in buf[..nbytes].iter_mut() {
                    *b = 0;
                }
                for s in 0..nsamp {
                    let v = SINE256[((phase >> 24) & 0xFF) as usize];
                    phase = phase.wrapping_add(inc);
                    if tone {
                        for ch in ch_lo..ch_hi {
                            let off = (s * CHANNELS + ch) * BYTES_PER_SAMPLE;
                            if BYTES_PER_SAMPLE == 2 {
                                let b = v.to_le_bytes();
                                buf[off] = b[0];
                                buf[off + 1] = b[1];
                            } else {
                                // 24-bit: scale the 16-bit sample up by 8 bits.
                                let b = ((v as i32) << 8).to_le_bytes();
                                buf[off] = b[0];
                                buf[off + 1] = b[1];
                                buf[off + 2] = b[2];
                            }
                        }
                    }
                }

                arm_iso(usbd, buf.as_ptr() as u32, nbytes as u16);
            }
            // Tight poll so we catch SOF within microseconds (before the frame's
            // IN token). The device has nothing else to do; usb.run() still runs
            // between yields.
            yield_now().await;
        }
    }
}

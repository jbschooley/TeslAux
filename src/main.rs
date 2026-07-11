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
use embassy_futures::join::join;
#[cfg(feature = "hid-heartbeat")]
use embassy_usb::class::hid::{Config as HidConfig, HidBootProtocol, HidSubclass, HidWriter, State as HidState};

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

// Input Terminal: ID 1, type 0x0201 (Microphone), 2 channels, L+R.
// 12 bytes total once embassy prepends bLength (0x0C) + bDescriptorType.
const AC_INPUT_TERMINAL: [u8; 10] = [
    0x02, // INPUT_TERMINAL
    0x01, // bTerminalID = 1
    0x01, 0x02, // wTerminalType = 0x0201 (Microphone)
    0x00, // bAssocTerminal
    0x02, // bNrChannels = 2
    0x03, 0x00, // wChannelConfig = 0x0003 (Left + Right Front)
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

// Format Type I: 2 channels, 2 bytes/subframe, 16-bit, one discrete sample
// rate = 48000 Hz (0x00BB80, little-endian 3-byte).
const AS_FORMAT_TYPE_I: [u8; 9] = [
    0x02, // FORMAT_TYPE
    0x01, // bFormatType = FORMAT_TYPE_I
    0x02, // bNrChannels = 2
    0x02, // bSubframeSize = 2 bytes
    0x10, // bBitResolution = 16
    0x01, // bSamFreqType = 1 discrete frequency
    0x80, 0xBB, 0x00, // tSamFreq[1] = 48000
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

    // HID class state — declared before the builder/buffers so it outlives
    // `usb` (which holds a handler that borrows it).
    #[cfg(feature = "hid-heartbeat")]
    let mut hid_state = HidState::new();

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
        // 192-byte iso IN, 1 ms interval.  On the nRF this lands on EP8 (0x88);
        // the exact number doesn't match the genuine 0x84 but hosts key on the
        // class/format, not the address.  extra_fields = [bRefresh, bSynchAddress]
        // makes this the 9-byte UAC audio-endpoint form.
        let _iso_in = alt1.endpoint_isochronous_in(
            None,
            192,
            1,
            SynchronizationType::Asynchronous,
            UsageType::DataEndpoint,
            &[0x00, 0x00],
        );
        // Class-specific AS iso endpoint descriptor follows the standard one.
        alt1.descriptor(CS_ENDPOINT, &AS_ISO_ENDPOINT);
    }

    drop(func);

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
            request_handler: None,
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

    // Run the device: handles enumeration + control transfers forever.  With
    // the HID heartbeat enabled, run it concurrently with the report stream.
    #[cfg(feature = "hid-heartbeat")]
    {
        let heartbeat = async {
            let mut report = [0u8; 8];
            let mut n: u8 = 0;
            loop {
                // Blocks until the host has the interface configured/open.
                hid_writer.ready().await;
                // Content is a GUESS — the real 8-byte layout isn't in the dump.
                // A rolling counter in byte 0 makes the stream visibly "alive"
                // so a liveness/keepalive watchdog sees changing data.  Revisit
                // once a genuine mic's HID reports are captured.
                report[0] = n;
                n = n.wrapping_add(1);
                if hid_writer.write(&report).await.is_err() {
                    // Endpoint not ready (host closed it); back off and re-arm.
                    Timer::after_millis(2).await;
                }
            }
        };
        join(usb.run(), heartbeat).await;
    }
    #[cfg(not(feature = "hid-heartbeat"))]
    usb.run().await;
}

// One cycle of a 1 kHz sine at 48 kHz = 48 samples, 16-bit, ~0.49 full-scale.
// A 192-byte iso packet holds exactly these 48 stereo frames, so a packet is
// exactly one cycle (starts/ends at the zero crossing) — packets are
// phase-continuous and switching buffers at a packet boundary is glitch-free.
// At task start this mono table is expanded into left-only / right-only stereo
// RAM buffers for EasyDMA. Generated; see README.
#[cfg(feature = "sine-button")]
#[rustfmt::skip]
const SINE_MONO: [i16; 48] = [
    0, 2088, 4141, 6123, 8000, 9740, 11314, 12694,
    13856, 14782, 15455, 15863, 16000, 15863, 15455, 14782,
    13856, 12694, 11314, 9740, 8000, 6123, 4141, 2088,
    0, -2088, -4141, -6123, -8000, -9740, -11314, -12694,
    -13856, -14782, -15455, -15863, -16000, -15863, -15455, -14782,
    -13856, -12694, -11314, -9740, -8000, -6123, -4141, -2088,
];

/// Arm the ISO IN endpoint's EasyDMA with a 192-byte packet at `ptr` and kick
/// the transfer.  The hardware sends it on the next IN token, then raises
/// EVENTS_ENDISOIN.
#[cfg(feature = "stream")]
#[inline]
fn arm_iso(usbd: pac::usbd::Usbd, ptr: u32) {
    usbd.isoin().ptr().write_value(ptr);
    usbd.isoin().maxcnt().write(|w| w.set_maxcnt(192));
    usbd.events_endisoin().write_value(0);
    // Ensure PTR/MAXCNT land before STARTISOIN reads them.
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    usbd.tasks_startisoin().write_value(1);
}

/// Stream one 192-byte packet per USB frame out the isochronous IN endpoint,
/// making this behave like a mic that is actively capturing.
///
/// - Default (`stream`): the packet is silence.
/// - `sine-button`: a 1 kHz sine plays while the user button (P1_10, active-low)
///   is held, and silence the instant it's released.  Each new press alternates
///   the channel it plays on — press 1 = LEFT, press 2 = RIGHT, press 3 = LEFT,
///   ... (the other channel stays silent).
///
/// We own the ISO registers outright: embassy-nrf's USB driver never touches
/// ISOIN / ISOINCONFIG / STARTISOIN / ENDISOIN, so there is no race with its
/// interrupt handler.  Pacing is self-clocked off EVENTS_ENDISOIN — one packet
/// armed per packet sent — which naturally tracks the host's 1 ms polling with
/// no drift.  The buffer choice is re-evaluated at every re-arm, so button
/// changes take effect within one frame.
///
/// To carry *real* audio, fill a RAM ring buffer from an I2S / PDM / SAADC DMA
/// capture and point `arm_iso` at its read cursor instead of these buffers.
#[cfg(feature = "stream")]
#[embassy_executor::task]
async fn iso_pump(button: Input<'static>) {
    let usbd = pac::USBD;

    // 192-byte packet buffers live in the task's future storage (RAM, embassy
    // arena) for the whole program — stable addresses EasyDMA can read.
    let silence = [0u8; 192];

    // Sine-button build only: left-only and right-only stereo buffers (the
    // opposite channel stays zero), plus the debounced button state and the
    // channel toggle.  Each fresh press flips the channel: press 1 -> LEFT,
    // press 2 -> RIGHT, press 3 -> LEFT, ...
    #[cfg(feature = "sine-button")]
    let mut left = [0u8; 192];
    #[cfg(feature = "sine-button")]
    let mut right = [0u8; 192];
    #[cfg(feature = "sine-button")]
    for (i, &s) in SINE_MONO.iter().enumerate() {
        let b = s.to_le_bytes(); // frame i = [L_lo, L_hi, R_lo, R_hi]
        left[4 * i] = b[0];
        left[4 * i + 1] = b[1];
        right[4 * i + 2] = b[0];
        right[4 * i + 3] = b[1];
    }
    #[cfg(feature = "sine-button")]
    let mut stable_pressed = false;
    #[cfg(feature = "sine-button")]
    let mut debounce: u8 = 0;
    #[cfg(feature = "sine-button")]
    let mut use_right = true; // first press flips this to false => LEFT
    #[cfg(not(feature = "sine-button"))]
    let _ = &button; // silence/default build never reads the button

    // Late frames answer the IN token with a zero-length packet (valid "no
    // samples this frame" for isochronous) instead of not responding.
    usbd.isoinconfig().write(|w| w.set_response(Response::ZERO_DATA));

    loop {
        // Idle until the host selects AudioStreaming alt-setting 1 — embassy
        // enables the iso IN endpoint, setting EPINEN bit 8 (`isoin`).
        while !usbd.epinen().read().isoin() {
            Timer::after_millis(4).await;
        }

        // Stream: arm exactly ONE packet per USB frame, immediately after the
        // Start-of-Frame, and never re-arm mid-frame.  This is the critical
        // timing rule for iso IN on the nRF: a full-speed 192-byte packet takes
        // ~128 us to clock out, and firing STARTISOIN (which DMAs into the
        // single ISO buffer) while that transmission is in flight corrupts the
        // packet — audible as scratchiness.  Arming right after SOF guarantees
        // the DMA (a few us) completes before the host's IN token and long
        // before the next frame.
        usbd.events_sof().write_value(0);
        while usbd.epinen().read().isoin() {
            if usbd.events_sof().read() != 0 {
                usbd.events_sof().write_value(0);

                // Pick this frame's packet.  Button is sampled once per frame
                // and debounced over ~12 ms so a single physical press toggles
                // the channel exactly once.
                #[cfg(feature = "sine-button")]
                let ptr = {
                    let raw = button.is_low(); // active-low
                    if raw == stable_pressed {
                        debounce = 0;
                    } else {
                        debounce += 1;
                        if debounce >= 12 {
                            stable_pressed = raw;
                            debounce = 0;
                            if stable_pressed {
                                use_right = !use_right; // new press => switch channel
                            }
                        }
                    }
                    if stable_pressed {
                        if use_right { right.as_ptr() } else { left.as_ptr() }
                    } else {
                        silence.as_ptr()
                    }
                } as u32;
                #[cfg(not(feature = "sine-button"))]
                let ptr = silence.as_ptr() as u32;

                arm_iso(usbd, ptr);
            }
            // Tight poll so we catch SOF within microseconds (before the frame's
            // IN token). The device has nothing else to do; usb.run() still runs
            // between yields.
            yield_now().await;
        }
    }
}

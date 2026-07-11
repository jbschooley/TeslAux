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
use embassy_nrf::pac;
use embassy_nrf::pac::usbd::vals::Response;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_nrf::{bind_interrupts, peripherals, usb};
use embassy_time::Timer;
use embassy_usb::descriptor::{SynchronizationType, UsageType};
use embassy_usb::{Builder, Config, UsbVersion};

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

    let mut usb = builder.build();

    // Feed the isochronous IN endpoint with silence at the register level.
    // embassy-nrf's driver can't write the iso endpoint (see module docs), so
    // this task drives USBD.ISOIN directly.  Started before usb.run() but it
    // simply waits until the host activates AudioStreaming alt-1.
    // (This embassy-executor's task macro returns a Result; the token is only
    // Err if the pool is already occupied, which can't happen on first spawn.)
    spawner.spawn(iso_silence_pump().unwrap());

    // Run the device: handles enumeration + control transfers forever.
    usb.run().await;
}

/// Arm the ISO IN endpoint's EasyDMA with a 192-byte packet at `ptr` and kick
/// the transfer.  The hardware sends it on the next IN token, then raises
/// EVENTS_ENDISOIN.
#[inline]
fn arm_iso(usbd: pac::usbd::Usbd, ptr: u32) {
    usbd.isoin().ptr().write_value(ptr);
    usbd.isoin().maxcnt().write(|w| w.set_maxcnt(192));
    usbd.events_endisoin().write_value(0);
    // Ensure PTR/MAXCNT land before STARTISOIN reads them.
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    usbd.tasks_startisoin().write_value(1);
}

/// Stream 192 bytes of silence per USB frame out the isochronous IN endpoint,
/// making this behave like a mic that is actively capturing (48 kHz * 2ch *
/// 16-bit = 192 B/ms of zeros).
///
/// We own the ISO registers outright: embassy-nrf's USB driver never touches
/// ISOIN / ISOINCONFIG / STARTISOIN / ENDISOIN, so there is no race with its
/// interrupt handler.  Pacing is self-clocked off EVENTS_ENDISOIN — one packet
/// armed per packet sent — which naturally tracks the host's 1 ms polling with
/// no drift.
///
/// To carry *real* audio instead of silence, replace the `silence` buffer with
/// a ring buffer filled by an I2S / PDM / SAADC DMA capture and arm from its
/// read cursor here.
#[embassy_executor::task]
async fn iso_silence_pump() {
    let usbd = pac::USBD;

    // 192 zero bytes.  Lives in the task's future storage (RAM, in the embassy
    // arena) for the whole program — a stable RAM address EasyDMA can read.
    let silence = [0u8; 192];
    let ptr = silence.as_ptr() as u32;

    // If a frame's buffer isn't ready in time, answer the IN token with a
    // zero-length packet (a valid "no samples this frame" for isochronous)
    // rather than not responding at all.
    usbd.isoinconfig().write(|w| w.set_response(Response::ZERO_DATA));

    loop {
        // Idle until the host selects AudioStreaming alt-setting 1 — embassy
        // enables the iso IN endpoint, setting EPINEN bit 8 (`isoin`).
        while !usbd.epinen().read().isoin() {
            Timer::after_millis(4).await;
        }

        // Prime the first packet, then re-arm as each one completes.
        arm_iso(usbd, ptr);
        while usbd.epinen().read().isoin() {
            if usbd.events_endisoin().read() != 0 {
                usbd.events_endisoin().write_value(0);
                arm_iso(usbd, ptr);
            }
            // Poll comfortably inside the 1 ms frame; the CPU has nothing else
            // to do.  (SOF-locked timing is a later refinement.)
            Timer::after_micros(150).await;
        }
    }
}

// SPDX-License-Identifier: MIT
//! The phone-facing side: a UAC1 stereo speaker at 48 kHz.
//!
//! Lifted from the RP2040 `source` binary, which is the version that has played
//! music from both an iPhone and an Android phone. Everything here is
//! `embassy-usb`, so the only thing that changed is who allocates the endpoint.
//!
//! This is the mirror image of `teslamic.rs`: USB streaming *in* to a speaker
//! terminal, where the mic is a microphone terminal out to USB streaming.

use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::descriptor::{SynchronizationType, UsageType};
use embassy_usb::driver::Driver;
use embassy_usb::{Builder, Config, Handler, UsbVersion};

/// One rate, and only one. The car is stereo-48k, so advertising anything else
/// to the phone would only create a rate we then have to refuse downstream.
pub const RATE: u32 = 48_000;
pub const CHANNELS: usize = 2;
/// 48 frames * 2ch * 2B.
pub const BYTES_PER_FRAME: usize = 192;

const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;
const AUDIO_CLASS: u8 = 0x01;
const SUBCLASS_AUDIOCONTROL: u8 = 0x01;
const SUBCLASS_AUDIOSTREAMING: u8 = 0x02;
const PROTO_UNDEFINED: u8 = 0x00;

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

/// bmAttributes bit 0 set = sampling-frequency control present. Hosts issue
/// `SET_CUR(SAMPLING_FREQ)` when opening a stream whether or not the device has
/// a choice to offer, and one that STALLs it can be dropped instead of opened.
const AS_ISO_ENDPOINT: [u8; 5] = [0x01, 0x01, 0x00, 0x00, 0x00];

/// UAC1 endpoint control selector for sampling frequency (`wValue` high byte).
const SAMPLING_FREQ_CONTROL: u8 = 0x01;

/// Answers `SET_CUR` / `GET_CUR` for the endpoint's sampling-frequency control.
///
/// 48000 is accepted and everything else is refused explicitly, so a host that
/// wants 44.1 learns it cannot have it rather than silently proceeding at the
/// wrong rate.
pub struct SampleRateHandler;

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

/// What the phone sees. Unlike the car side, nothing here is being cloned, so
/// these values are ours to choose.
pub fn config() -> Config<'static> {
    let mut c = Config::new(0x1209, 0x0001);
    c.manufacturer = Some("TeslAux");
    c.product = Some("TeslAux Bridge");
    c.serial_number = Some("0001");
    c.bcd_usb = UsbVersion::Two;
    c.max_power = 100;
    c.self_powered = true;
    c.composite_with_iads = false;
    c.device_class = 0x00;
    c.device_sub_class = 0x00;
    c.device_protocol = 0x00;
    c.max_packet_size_0 = 64;
    c
}

/// Build the speaker onto `builder` and hand back the iso OUT endpoint.
pub fn build<'d, D: Driver<'d>>(builder: &mut Builder<'d, D>) -> D::EndpointOut {
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
        // alt 0 is the zero-bandwidth setting the host parks on between uses.
        stream.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOSTREAMING, PROTO_UNDEFINED, None);
        let mut alt1 =
            stream.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOSTREAMING, PROTO_UNDEFINED, None);
        alt1.descriptor(CS_INTERFACE, &AS_GENERAL);
        alt1.descriptor(CS_INTERFACE, &AS_FORMAT_TYPE_I);
        // Adaptive: "send at your rate, we will follow you." No feedback
        // endpoint, so there is nothing for a host to get wrong.
        let ep = alt1.endpoint_isochronous_out(
            None,
            // Headroom for one extra frame. Hosts vary packet size for their own
            // drift management, and a host that sends 49 frames into a 192-byte
            // endpoint has its packet truncated — the mirror of the bug that
            // silently dropped every corrected packet on the car board.
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
}

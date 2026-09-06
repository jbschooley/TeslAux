// SPDX-License-Identifier: MIT
//! The TeslaMic USB device, byte-for-byte.
//!
//! Lifted from the nRF52840 firmware's `src/main.rs`, which is the version the
//! car actually accepts (mic icon, audio, no "unsupported USB microphone"
//! popup). Everything here is `embassy-usb`, which is chip-agnostic — the only
//! thing that changed porting it to RP2040 is who allocates the endpoints.
//!
//! Descriptor values come from `real_mic_dump.md` (a working clone dumped over
//! libusb). The car validates these, so **do not "clean them up"**: the
//! endpoint-less IF3 with its exact 36-byte report descriptor and its
//! `00 01 00 03 03 00 08 00` Feature report is what defeats the popup.
//!
//! Deliberate, known-good divergences from the real mic (all present in the
//! nRF build that works in the car):
//!   * Audio topology is Input Terminal 1 -> Output Terminal 2. The real mic has
//!     Input 4 -> Feature Unit 5 -> Selector 6 -> Output 7 and advertises two
//!     sample rates. The car does not care.
//!   * `bcd_usb` is 0x0200; the real mic is 0x0110.
//!   * The iso IN endpoint number differs (the host keys on class + format).

// Shared by the car binary and the single-chip build, which size the
// isochronous endpoint differently — a constant unused by one is used by
// the other.
#![allow(dead_code)]

use embassy_usb::class::hid::{
    Config as HidConfig, HidBootProtocol, HidSubclass, HidWriter, ReportId, RequestHandler,
    State as HidState,
};
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::descriptor::{SynchronizationType, UsageType};
use embassy_usb::driver::Driver;
use embassy_usb::{Builder, Config, Handler, UsbVersion};

/// The shipping format: exactly the real TeslaMic's. Not a build-time knob —
/// the car is stereo-48k and the PCM2706 is 16-bit only, so there is one
/// correct answer.
pub const SAMPLE_RATE: u32 = 48_000;
pub const CHANNELS: usize = 2;
pub const BYTES_PER_SAMPLE: usize = 2;
/// 48 frames * 2ch * 2B. The real mic's wMaxPacketSize exactly.
pub const BYTES_PER_FRAME: usize = 192;

/// One extra stereo frame of headroom.
///
/// Any build that paces elastically sends `nominal + 1` samples when shedding
/// drift, i.e. 49 frames = 196 bytes. embassy-rp rejects a write longer than the
/// endpoint's `max_packet_size` with `BufferOverflow`, so an endpoint declared
/// at 192 silently drops every corrected packet — which sounds like a click at
/// every packet boundary, not like a drift problem at all.
///
/// Deviating from the real mic's 192 is safe: verified in the car with the nRF
/// `packet-stress-control` build, which advertises 196 with constant packets and
/// plays clean.
pub const BYTES_PER_FRAME_ELASTIC: usize = BYTES_PER_FRAME + 4;

const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;
const AUDIO_CLASS: u8 = 0x01;
const SUBCLASS_AUDIOCONTROL: u8 = 0x01;
const SUBCLASS_AUDIOSTREAMING: u8 = 0x02;
const PROTO_UNDEFINED: u8 = 0x00;

// The audio topology, in two versions.
//
// The default is the bare Input -> Output pair that has worked in the car since
// the beginning. `feature-unit` replaces it with the real mic's full topology —
// Input 4 -> Feature Unit 5 -> Selector 6 -> Output 7, terminal IDs and all —
// because the real mic gets a volume control in the car's UI and this one does
// not, and the Feature Unit is the only thing that could be driving it.
//
// The earlier control capture showed the car issuing no audio-class requests at
// all, which was taken as proof it ignores volume. It proves nothing of the
// kind: that capture was of THIS device, which advertises no Feature Unit for
// the car to talk to.

#[cfg(not(feature = "feature-unit"))]
const AC_HEADER: [u8; 7] = [
    0x01, // HEADER
    0x00, 0x01, // bcdADC = 1.00
    0x1E, 0x00, // wTotalLength = 30
    0x01, // bInCollection
    0x01, // baInterfaceNr(1) = the AudioStreaming interface
];
/// 9 header + 12 input + 10 feature + 7 selector + 9 output = 47, which is what
/// the real mic reports.
#[cfg(feature = "feature-unit")]
const AC_HEADER: [u8; 7] = [
    0x01, // HEADER
    0x00, 0x01, // bcdADC = 1.00
    0x2F, 0x00, // wTotalLength = 47
    0x01, // bInCollection
    0x01, // baInterfaceNr(1) = the AudioStreaming interface
];

#[cfg(not(feature = "feature-unit"))]
const AC_INPUT_TERMINAL: [u8; 10] = [
    0x02, // INPUT_TERMINAL
    0x01, // bTerminalID = 1
    0x01, 0x02, // wTerminalType = 0x0201 (Microphone)
    0x00, // bAssocTerminal
    CHANNELS as u8,
    0x03, 0x00, // wChannelConfig = L+R
    0x00, // iChannelNames
    0x00, // iTerminal
];
#[cfg(feature = "feature-unit")]
const AC_INPUT_TERMINAL: [u8; 10] = [
    0x02, // INPUT_TERMINAL
    0x04, // bTerminalID = 4, as the real mic
    0x01, 0x02, // wTerminalType = 0x0201 (Microphone)
    0x00, // bAssocTerminal
    CHANNELS as u8,
    0x03, 0x00, // wChannelConfig = L+R
    0x00, // iChannelNames
    0x00, // iTerminal
];

/// Master mute plus per-channel volume, exactly the real mic's control bitmap.
#[cfg(feature = "feature-unit")]
const AC_FEATURE_UNIT: [u8; 8] = [
    0x06, // FEATURE_UNIT
    0x05, // bUnitID = 5
    0x04, // bSourceID = the input terminal
    0x01, // bControlSize = 1 byte per channel
    0x01, // bmaControls(0) master: mute
    0x02, // bmaControls(1) ch1: volume
    0x02, // bmaControls(2) ch2: volume
    0x00, // iFeature
];

#[cfg(feature = "feature-unit")]
const AC_SELECTOR_UNIT: [u8; 5] = [
    0x05, // SELECTOR_UNIT
    0x06, // bUnitID = 6
    0x01, // bNrInPins
    0x05, // baSourceID(1) = the feature unit
    0x00, // iSelector
];

#[cfg(not(feature = "feature-unit"))]
const AC_OUTPUT_TERMINAL: [u8; 7] = [
    0x03, // OUTPUT_TERMINAL
    0x02, // bTerminalID = 2
    0x01, 0x01, // wTerminalType = 0x0101 (USB Streaming)
    0x00, // bAssocTerminal
    0x01, // bSourceID = the microphone input terminal
    0x00, // iTerminal
];
#[cfg(feature = "feature-unit")]
const AC_OUTPUT_TERMINAL: [u8; 7] = [
    0x03, // OUTPUT_TERMINAL
    0x07, // bTerminalID = 7, as the real mic
    0x01, 0x01, // wTerminalType = 0x0101 (USB Streaming)
    0x00, // bAssocTerminal
    0x06, // bSourceID = the selector unit
    0x00, // iTerminal
];

#[cfg(not(feature = "feature-unit"))]
const AS_GENERAL: [u8; 5] = [
    0x01, // AS_GENERAL
    0x02, // bTerminalLink = 2
    0x01, // bDelay
    0x01, 0x00, // wFormatTag = PCM
];
#[cfg(feature = "feature-unit")]
const AS_GENERAL: [u8; 5] = [
    0x01, // AS_GENERAL
    0x07, // bTerminalLink = 7, the real mic's output terminal
    0x01, // bDelay
    0x01, 0x00, // wFormatTag = PCM
];

/// Built at runtime because the advertised rate follows the source: the board
/// re-enumerates at whatever the attached bridge is actually delivering, rather
/// than muting when it is not 48 kHz. The real mic advertises two rates for the
/// same reason.
fn as_format_type_i(rate: u32) -> [u8; 9] {
    [
        0x02, // FORMAT_TYPE
        0x01, // FORMAT_TYPE_I
        CHANNELS as u8,
        BYTES_PER_SAMPLE as u8,
        16,   // bBitResolution
        0x01, // one discrete frequency
        (rate & 0xff) as u8,
        ((rate >> 8) & 0xff) as u8,
        ((rate >> 16) & 0xff) as u8,
    ]
}

/// bmAttributes bit 0 set = **sampling-frequency control present**, which is
/// what the real mic advertises (`real_mic_dump.md`: "CS_EP: sampling-frequency
/// control enabled (bmAttributes 0x01)").
///
/// We shipped 0x00 for a long time and the car tolerated it on the RP2040 and
/// the nRF. It does not tolerate it everywhere: the car sends
/// `SET_CUR(SAMPLING_FREQ)` on this endpoint regardless of what we advertise,
/// and a device that STALLs it can be abandoned — on the STM32 the car set alt
/// 1, had its request refused, cycled alt 1/0 a few times and then stopped
/// polling for audio altogether.
const AS_ISO_ENDPOINT: [u8; 5] = [
    0x01, // EP_GENERAL
    0x01, // bmAttributes: sampling-frequency control
    0x00, // bLockDelayUnits
    0x00, 0x00, // wLockDelay
];

/// UAC1 endpoint control selector for sampling frequency (`wValue` high byte).
const SAMPLING_FREQ_CONTROL: u8 = 0x01;

/// Answers the car's `SET_CUR` / `GET_CUR` for the endpoint's sampling
/// frequency.
///
/// The real mic answers these; ours refused them, which is a divergence from
/// the device we are cloning rather than a simplification of it.
pub struct SampleRateHandler {
    pub rate: u32,
}

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
        if hz == self.rate {
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
        buf[..3].copy_from_slice(&self.rate.to_le_bytes()[..3]);
        Some(InResponse::Accepted(&buf[..3]))
    }
}

/// IF2: a standard HID boot keyboard, 65 bytes. The real mic's physical button
/// sends keystrokes; ours never presses a key.
#[rustfmt::skip]
const HID_REPORT_KEYBOARD: [u8; 65] = [
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7,
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01, 0x05, 0x08, 0x19, 0x01,
    0x29, 0x05, 0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91, 0x01, 0x95, 0x06,
    0x75, 0x08, 0x15, 0x00, 0x26, 0xa4, 0x00, 0x05, 0x07, 0x19, 0x00, 0x2a,
    0xa4, 0x00, 0x81, 0x00, 0xc0,
];

/// IF3: endpoint-less vendor HID, Usage Page 0xFF00 / Usage 0x55AA, with 256-B
/// In, 256-B Out and 8-B Feature reports. The car writes `A5 5A`-framed config
/// to the Output report. **This descriptor is what the car validates.**
#[rustfmt::skip]
const HID_REPORT_IF3: [u8; 36] = [
    0x06, 0x00, 0xff, 0x0a, 0xaa, 0x55, 0xa1, 0x01, 0x15, 0x00, 0x26, 0xff,
    0x00, 0x75, 0x08, 0x96, 0x00, 0x01, 0x09, 0x01, 0x81, 0x02, 0x96, 0x00,
    0x01, 0x09, 0x01, 0x91, 0x02, 0x95, 0x08, 0x09, 0x01, 0xb1, 0x02, 0xc0,
];

const HID_DESC_IF3: [u8; 9] = [0x09, 0x21, 0x01, 0x02, 0x00, 0x01, 0x22, 0x24, 0x00];

/// What the real mic returns on GET_REPORT(Feature) for IF3.
const IF3_FEATURE: [u8; 8] = [0x00, 0x01, 0x00, 0x03, 0x03, 0x00, 0x08, 0x00];

pub struct KeyboardHandler;

impl RequestHandler for KeyboardHandler {
    fn get_report(&mut self, _id: ReportId, buf: &mut [u8]) -> Option<usize> {
        let n = buf.len().min(8);
        buf[..n].fill(0);
        Some(n)
    }
    fn set_report(&mut self, _id: ReportId, _data: &[u8]) -> OutResponse {
        OutResponse::Accepted
    }
    fn get_idle_ms(&mut self, _id: Option<ReportId>) -> Option<u32> {
        Some(0)
    }
}

/// IF3 is endpoint-less, so `HidWriter` can't build it (it always allocates an
/// endpoint). Hand-rolled: serve the report/HID descriptors, return the Feature
/// report, and accept the car's SET_REPORT config writes.
pub struct If3Handler;

impl Handler for If3Handler {
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.index != 3 {
            return None;
        }
        match (req.request_type, req.request) {
            (RequestType::Standard, 0x06) => match (req.value >> 8) as u8 {
                0x22 => Some(InResponse::Accepted(&HID_REPORT_IF3)),
                0x21 => Some(InResponse::Accepted(&HID_DESC_IF3)),
                _ => Some(InResponse::Rejected),
            },
            (RequestType::Class, 0x01) => {
                let n = IF3_FEATURE.len().min(buf.len());
                buf[..n].copy_from_slice(&IF3_FEATURE[..n]);
                Some(InResponse::Accepted(&buf[..n]))
            }
            _ => None,
        }
    }

    fn control_out(&mut self, req: Request, _data: &[u8]) -> Option<OutResponse> {
        if req.index != 3 {
            return None;
        }
        match req.request_type {
            RequestType::Class => Some(OutResponse::Accepted),
            _ => None,
        }
    }
}

/// Answers the car's volume and mute queries, and records what it asked for.
///
/// The point of the build is the record, not the answers. If these counters stay
/// at zero the car never touches the Feature Unit and the ceiling is entirely
/// its own mixing; if they move, the car drives mic volume over USB and what we
/// report here is a lever.
///
/// Ranges are deliberately generous: 30 dB of gain is offered above unity, so if
/// the car maps its slider onto what the device advertises there is somewhere
/// for it to go. Nothing here changes a sample — `CUR` is what we *claim*, and
/// applying it would be a separate decision.
#[cfg(feature = "feature-unit")]
pub mod feature_unit {
    use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

    /// UAC1 volume is a signed 16-bit count of 1/256 dB.
    const DB: i32 = 256;

    pub static REQUESTS: AtomicU32 = AtomicU32::new(0);
    pub static SET_CUR_SEEN: AtomicU32 = AtomicU32::new(0);
    /// The last volume the car asked for, in 1/256 dB.
    pub static LAST_VOLUME: AtomicI32 = AtomicI32::new(0);
    /// Packed (bRequest << 16) | wValue of the last request, for anything the
    /// cases below do not name.
    pub static LAST_RAW: AtomicU32 = AtomicU32::new(0);
    pub static MUTED: AtomicU32 = AtomicU32::new(0);

    pub const CUR: i16 = 0; // 0 dB
    pub const MIN: i16 = (-30 * DB) as i16;
    pub const RES: i16 = (DB / 2) as i16; // 0.5 dB

    /// The advertised ceiling, and the experiment.
    ///
    /// The car reads this once at connect and immediately sets CUR to it on
    /// both channels — then never touches the unit again, however far its own
    /// slider moves. So the slider is the car's internal mixer, and this is the
    /// only number about us it takes any notice of.
    ///
    /// Which raises the inverted question. If the car *normalises* for what a
    /// mic claims it can do — "this one has 30 dB in hand, so I will pull back
    /// downstream" — then advertising LESS should make it attenuate less.
    /// `fu-max-unity` claims no gain at all and is how that gets tested. It is
    /// the opposite of the intuitive move, which is why it is worth measuring
    /// rather than reasoning about.
    ///
    /// On the real mic this control is a preamp gain for an analog capsule, and
    /// running it at maximum is exactly right. For a source already at full
    /// scale there is nothing underneath for it to amplify, so nothing here
    /// applies gain to a sample either way.
    #[cfg(not(feature = "fu-max-unity"))]
    pub const MAX: i16 = (30 * DB) as i16;
    #[cfg(feature = "fu-max-unity")]
    pub const MAX: i16 = 0; // 0 dB: we claim no gain available at all

    pub fn note(request: u8, value: u16) {
        REQUESTS.store(REQUESTS.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
        LAST_RAW.store(((request as u32) << 16) | value as u32, Ordering::Relaxed);
    }
}

/// The Feature Unit's control endpoint. Addressed as wIndex = (unit << 8) |
/// interface, so unit 5 on IF0 is 0x0500.
#[cfg(feature = "feature-unit")]
pub struct FeatureUnitHandler;

#[cfg(feature = "feature-unit")]
impl FeatureUnitHandler {
    const UNIT: u8 = 5;
    const MUTE: u8 = 0x01;
    const VOLUME: u8 = 0x02;
}

#[cfg(feature = "feature-unit")]
impl Handler for FeatureUnitHandler {
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        use core::sync::atomic::Ordering;
        if req.request_type != RequestType::Class
            || (req.index >> 8) as u8 != Self::UNIT
            || (req.index & 0xFF) != 0
        {
            return None;
        }
        feature_unit::note(req.request, req.value);
        let selector = (req.value >> 8) as u8;
        // 0x81 GET_CUR, 0x82 GET_MIN, 0x83 GET_MAX, 0x84 GET_RES.
        let v: i16 = match (req.request, selector) {
            (0x81, Self::VOLUME) => feature_unit::CUR,
            (0x82, Self::VOLUME) => feature_unit::MIN,
            (0x83, Self::VOLUME) => feature_unit::MAX,
            (0x84, Self::VOLUME) => feature_unit::RES,
            (0x81, Self::MUTE) => {
                let m = feature_unit::MUTED.load(Ordering::Relaxed) as u8;
                if buf.is_empty() {
                    return Some(InResponse::Rejected);
                }
                buf[0] = m;
                return Some(InResponse::Accepted(&buf[..1]));
            }
            _ => return Some(InResponse::Rejected),
        };
        if buf.len() < 2 {
            return Some(InResponse::Rejected);
        }
        buf[..2].copy_from_slice(&v.to_le_bytes());
        Some(InResponse::Accepted(&buf[..2]))
    }

    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        use core::sync::atomic::Ordering;
        if req.request_type != RequestType::Class
            || (req.index >> 8) as u8 != Self::UNIT
            || (req.index & 0xFF) != 0
        {
            return None;
        }
        feature_unit::note(req.request, req.value);
        // 0x01 = SET_CUR.
        if req.request == 0x01 {
            feature_unit::SET_CUR_SEEN.store(
                feature_unit::SET_CUR_SEEN.load(Ordering::Relaxed).wrapping_add(1),
                Ordering::Relaxed,
            );
            match ((req.value >> 8) as u8, data.len()) {
                (Self::VOLUME, n) if n >= 2 => {
                    let v = i16::from_le_bytes([data[0], data[1]]);
                    feature_unit::LAST_VOLUME.store(v as i32, Ordering::Relaxed);
                }
                (Self::MUTE, n) if n >= 1 => {
                    feature_unit::MUTED.store(data[0] as u32, Ordering::Relaxed);
                }
                _ => {}
            }
        }
        Some(OutResponse::Accepted)
    }
}

/// Device identity. The 40-byte serial forces a 162-byte string descriptor, so
/// the control buffer must be >= 256 (128 broke enumeration on nRF).
pub fn config() -> Config<'static> {
    let mut c = Config::new(0x1235, 0x0002);
    c.manufacturer = Some("TeslaMic_V004_FW_20220217Tes");
    c.product = Some("TeslaMic");
    c.serial_number =
        Some("0E02070008181F0B8AFB76F93A8C1B1D472AA0AA0C2535898B4D6A4738FAD0057B6A12DB17320B75");
    c.bcd_usb = UsbVersion::Two;
    c.max_power = 500;
    c.self_powered = false;
    c.composite_with_iads = false;
    c.device_class = 0x00;
    c.device_sub_class = 0x00;
    c.device_protocol = 0x00;
    c.max_packet_size_0 = 64;
    c
}

/// Build all four interfaces onto `builder` and hand back the iso IN endpoint.
///
/// Interface order is load-bearing — the car addresses IF2/IF3 by index.
/// `ep_max_bytes` is the iso IN endpoint's wMaxPacketSize. Pass
/// [`BYTES_PER_FRAME`] for a build that always sends exactly 48 frames
/// (clock-locked), or [`BYTES_PER_FRAME_ELASTIC`] for anything that varies the
/// packet size.
pub fn build<'d, D: Driver<'d>>(
    builder: &mut Builder<'d, D>,
    hid_state: &'d mut HidState<'d>,
    kbd: &'d mut KeyboardHandler,
    if3: &'d mut If3Handler,
    srate: &'d mut SampleRateHandler,
    #[cfg(feature = "feature-unit")] fu: &'d mut FeatureUnitHandler,
    ep_max_bytes: u16,
    rate: u32,
) -> D::EndpointIn {
    srate.rate = rate;
    // IF0 AudioControl + IF1 AudioStreaming.
    let mut func = builder.function(AUDIO_CLASS, SUBCLASS_AUDIOCONTROL, PROTO_UNDEFINED);
    {
        let mut ac = func.interface();
        let mut alt = ac.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOCONTROL, PROTO_UNDEFINED, None);
        alt.descriptor(CS_INTERFACE, &AC_HEADER);
        alt.descriptor(CS_INTERFACE, &AC_INPUT_TERMINAL);
        #[cfg(feature = "feature-unit")]
        {
            alt.descriptor(CS_INTERFACE, &AC_FEATURE_UNIT);
            alt.descriptor(CS_INTERFACE, &AC_SELECTOR_UNIT);
        }
        alt.descriptor(CS_INTERFACE, &AC_OUTPUT_TERMINAL);
    }
    let iso_in = {
        let mut stream = func.interface();
        // alt 0 is the zero-bandwidth setting the car parks on between uses.
        stream.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOSTREAMING, PROTO_UNDEFINED, None);
        let mut alt1 =
            stream.alt_setting(AUDIO_CLASS, SUBCLASS_AUDIOSTREAMING, PROTO_UNDEFINED, None);
        alt1.descriptor(CS_INTERFACE, &AS_GENERAL);
        alt1.descriptor(CS_INTERFACE, &as_format_type_i(rate));
        let ep = alt1.endpoint_isochronous_in(
            None,
            ep_max_bytes,
            1,
            SynchronizationType::Asynchronous,
            UsageType::DataEndpoint,
            &[0x00, 0x00], // bRefresh, bSynchAddress -> the 9-byte UAC form
        );
        alt1.descriptor(CS_ENDPOINT, &AS_ISO_ENDPOINT);
        ep
    };
    drop(func);

    // IF2: the real keyboard descriptor. Kept alive so the endpoint stays
    // allocated; no key is ever pressed.
    let _kbd: HidWriter<'_, D, 8> = HidWriter::new(
        builder,
        hid_state,
        HidConfig {
            report_descriptor: &HID_REPORT_KEYBOARD,
            request_handler: Some(kbd),
            poll_ms: 1,
            max_packet_size: 8,
            hid_subclass: HidSubclass::No,
            hid_boot_protocol: HidBootProtocol::None,
        },
    );

    // IF3: HID class interface with a HID descriptor and no endpoint.
    {
        let mut f3 = builder.function(0x03, 0x00, 0x00);
        let mut i3 = f3.interface();
        let mut a3 = i3.alt_setting(0x03, 0x00, 0x00, None);
        a3.descriptor(0x21, &HID_DESC_IF3[2..]);
    }
    builder.handler(if3);
    builder.handler(srate);
    // Global like the others; it filters on wIndex itself.
    #[cfg(feature = "feature-unit")]
    builder.handler(fu);

    iso_in
}

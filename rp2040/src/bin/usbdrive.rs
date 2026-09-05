// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! A USB drive that isn't: 20 silent tracks the car can play.
//!
//! Standalone on purpose. The car validates the TeslaMic descriptor set, and
//! the endpoint-less IF3 is what stops the "unsupported USB microphone" popup —
//! adding a mass-storage interface to that device makes it not-a-TeslaMic and
//! risks the popup returning. A second USB port costs nothing by comparison, so
//! the mic stays a byte-for-byte clone and this is just a drive.
//!
//! What it is for: the car tells a connected microphone nothing about its own
//! transport buttons (measured — see `MEDIA-CONTROLS.md`), so the only way to
//! see a button press is to watch the car's media player move through a
//! playlist we author. Which sectors it reads says which track it moved to.
//!
//! Bring-up order, cheapest first:
//!
//! | LED | meaning |
//! |-----|---------|
//! | blue, slow | enumerated, nothing read yet |
//! | amber | sectors read, but **metadata only** — no track was opened |
//! | green | exactly **one** track was opened |
//! | cyan | **several** tracks were opened |
//! | white | **many** (16+) tracks were opened |
//! | red | a SCSI command was refused mid-scan, or the transport gave up |
//!
//! Once the car is playing, the LED reports what the detector makes of its
//! reads, which is the thing worth watching:
//!
//! | LED | meaning |
//! |-----|---------|
//! | white flash | **next** pressed |
//! | purple flash | **previous** pressed |
//! | blue | playback paused — reads have stopped |
//!
//! Press a button and the colour should change immediately. A flash with no
//! press is the error that matters: it becomes a track skipping by itself on
//! the source.
//!
//! The count is the useful part once a host mounts the volume but will not play
//! it. One track means it opened a file and rejected something about it; all
//! twenty means it indexed the volume and is declining it for a reason that has
//! nothing to do with the files. Those need opposite next steps.
//!
//! Plug it into a Mac first. If `TESLAUX` mounts and `001.WAV` plays, the class
//! implementation is right and the car is the only remaining unknown.

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_usb::control::{InResponse, OutResponse, Request, RequestType};
use embassy_usb::driver::{Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::{Builder, Config, Handler, UsbDevice};

#[path = "../detect.rs"]
mod detect;
#[path = "../fat.rs"]
mod fat;
#[path = "../msc.rs"]
mod msc;
#[path = "../status.rs"]
mod status;
#[cfg(feature = "rp2040-zero")]
#[path = "../ws2812.rs"]
mod ws2812;

use detect::{Detector, Event};
use fat::SECTOR;
use msc::{Action, Cbw, Scsi, Status as CswStatus};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

#[cfg(feature = "rp2040-zero")]
bind_interrupts!(struct Pio1Irqs {
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO1>;
});

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

/// Sectors served since boot. Non-zero means a host is actually reading us,
/// which is the only evidence that matters at this stage.
static SECTORS_READ: AtomicU32 = AtomicU32::new(0);
/// True once the host has read a sector that belongs to a **track**, rather
/// than to the partition table, the FAT or the directory.
///
/// This is the question that matters when a host mounts the volume but does not
/// offer it as a media source: did it parse the filesystem and go looking at the
/// files, or did it give up after the metadata? `locate()` already answers it —
/// it is the same function the media-control detector will be built on, so this
/// exercises it against a real host at the same time.
static READ_TRACK_DATA: AtomicBool = AtomicBool::new(false);
/// Bitmask of which tracks the host has read from, one bit per track.
///
/// The count is the useful number: a host that opens one file and stops has
/// rejected something about it, while one that opens all twenty has indexed the
/// volume and is declining it for a reason that has nothing to do with the
/// files. Those need completely different next steps.
static TRACKS_SEEN: AtomicU32 = AtomicU32::new(0);
/// Button presses inferred from the car's reads: `next` in the low half,
/// `prev` in the high half.
///
/// Until there is somewhere to send them, this is the whole output of the
/// detector — and it is the number worth watching in the car, because a press
/// nobody made is directly audible as a track skipping by itself.
static PRESSES: AtomicU32 = AtomicU32::new(0);
/// True while the car has stopped reading, i.e. playback is paused.
static PLAYBACK_PAUSED: AtomicBool = AtomicBool::new(false);
/// The most recent detector event and when it happened, so the LED can show it
/// while it is fresh. 1 = next, 2 = previous.
static LAST_EVENT: AtomicU32 = AtomicU32::new(0);
static LAST_EVENT_MS: AtomicU32 = AtomicU32::new(0);
/// When a sector inside a track was last read. Pause is derived from this
/// rather than from the detector, so the status task needs no access to it.
static LAST_READ_MS: AtomicU32 = AtomicU32::new(0);

/// How long an event stays on the LED.
const FLASH_MS: u32 = 1200;

/// Bytes of track data a second of playback consumes: 48 kHz, 16-bit, stereo.
const PLAYBACK_BYTES_PER_SEC: u64 = 48_000 * 2 * 2;
/// How much faster than real time track data is served.
///
/// Just above 1, and that is the whole point. Served at full speed the car
/// fills its buffer, stops reading for seconds at a time, and plays from
/// memory — so *reads stopping* means nothing, and a pause is invisible until
/// the buffer drains. Held to barely above real time it can never get ahead,
/// so it reads continuously and a pause shows up within one read.
const PACE_NUMERATOR: u64 = 11;
const PACE_DENOMINATOR: u64 = 10;
/// Track data servable at full speed before pacing applies, per track.
///
/// A player wants a buffer before it starts, and throttling from the first byte
/// risks a stutter at the start of every track. This is enough to prime one and
/// not enough to hide a pause behind.
const BURST_BYTES: u64 = 2 * PLAYBACK_BYTES_PER_SEC;
/// Quiet time that counts as playback having stopped. Matches the detector.
const PAUSE_AFTER_MS: u32 = 400;
/// A SCSI command was refused *after* the host had started reading tracks.
///
/// Failures before that are normal: hosts probe for optional commands and
/// expect to be told no. One that arrives mid-scan is different — it may be the
/// reason a host stops. Kept separate from the early probing so the signal is
/// not buried in it.
static LATE_CMD_FAILED: AtomicBool = AtomicBool::new(false);
/// The transport could not continue: a malformed CBW, or an endpoint error.
static WEDGED: AtomicBool = AtomicBool::new(false);

fn current_state() -> status::State {
    if WEDGED.load(Ordering::Relaxed) {
        // red: the transport gave up
        status::State::Fault
    } else if LATE_CMD_FAILED.load(Ordering::Relaxed) {
        // A command refused while the host was scanning is a better lead than
        // any count, so it outranks them.
        status::State::Fault
    } else if {
        // A fresh event outranks everything below: it is the thing being
        // watched for, and it is gone in a moment.
        let now = embassy_time::Instant::now().as_millis() as u32;
        LAST_EVENT.load(Ordering::Relaxed) != 0
            && now.wrapping_sub(LAST_EVENT_MS.load(Ordering::Relaxed)) < FLASH_MS
    } {
        return status::State::Flash(LAST_EVENT.load(Ordering::Relaxed) as u8);
    } else if READ_TRACK_DATA.load(Ordering::Relaxed)
        && embassy_time::Instant::now()
            .as_millis()
            .wrapping_sub(LAST_READ_MS.load(Ordering::Relaxed) as u64)
            >= PAUSE_AFTER_MS as u64
    {
        // Reads have stopped while a track was open: playback is paused.
        status::State::Flash(3)
    } else if READ_TRACK_DATA.load(Ordering::Relaxed) {
        // The host parsed the filesystem and opened files; how many it opened
        // is the question that decides what to try next.
        status::State::Count(TRACKS_SEEN.load(Ordering::Relaxed).count_ones() as u8)
    } else if SECTORS_READ.load(Ordering::Relaxed) > 0 {
        // amber: sectors were read, but only metadata — the host looked at the
        // volume and stopped before any track
        status::State::Slipping
    } else {
        // blue: nothing read at all
        status::State::Waiting
    }
}

#[cfg(not(feature = "rp2040-zero"))]
#[embassy_executor::task]
async fn status_task(mut led: embassy_rp::gpio::Output<'static>) -> ! {
    status::run(&mut led, current_state).await
}

#[cfg(feature = "rp2040-zero")]
#[embassy_executor::task]
async fn status_task(mut led: ws2812::Ws2812<'static, embassy_rp::peripherals::PIO1, 0>) -> ! {
    status::run(&mut led, current_state).await
}

/// Answers the two class-specific control requests Bulk-Only Transport defines.
///
/// `embassy-usb` knows nothing about mass storage, so without this the host's
/// GET_MAX_LUN is STALLed. Some hosts read a STALL as "one LUN" and continue;
/// others treat it as a broken device and never mount. Answering is one byte
/// and removes the question.
struct MscControl;

impl Handler for MscControl {
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.request_type != RequestType::Class || req.request != msc::REQ_GET_MAX_LUN {
            return None;
        }
        // Highest LUN number, not a count: a single-LUN device reports 0.
        buf[0] = 0;
        Some(InResponse::Accepted(&buf[..1]))
    }

    fn control_out(&mut self, req: Request, _data: &[u8]) -> Option<OutResponse> {
        if req.request_type != RequestType::Class || req.request != msc::REQ_BULK_ONLY_RESET {
            return None;
        }
        // Nothing to tear down: the transport loop re-reads a CBW every pass,
        // so accepting the reset is enough to get back in step.
        Some(OutResponse::Accepted)
    }
}

/// Record what the detector reported.
///
/// Presses accumulate in one word — `next` in the low half, `prev` in the high
/// half — because until there is somewhere to send them, the count *is* the
/// output. It is also the number worth watching in the car: a press nobody made
/// is directly audible later as a track skipping by itself.
fn flash(code: u32) {
    LAST_EVENT.store(code, Ordering::Relaxed);
    LAST_EVENT_MS.store(embassy_time::Instant::now().as_millis() as u32, Ordering::Relaxed);
}

fn note_event(ev: Option<Event>) {
    let bump = |shift: u32, by: u32| {
        let cur = PRESSES.load(Ordering::Relaxed);
        let half = (cur >> shift) & 0xFFFF;
        let other = cur & (0xFFFF << (16 - shift));
        PRESSES.store(other | (half.saturating_add(by).min(0xFFFF) << shift), Ordering::Relaxed);
    };
    match ev {
        Some(Event::Next(n)) => {
            bump(0, n);
            flash(1);
        }
        Some(Event::Prev(n)) => {
            bump(16, n);
            flash(2);
        }
        Some(Event::Paused) => PLAYBACK_PAUSED.store(true, Ordering::Relaxed),
        Some(Event::Resumed) => PLAYBACK_PAUSED.store(false, Ordering::Relaxed),
        None => {}
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);

    // Our own identity: nothing is being cloned here, unlike the mic.
    let mut config = Config::new(0x1209, 0x0002);
    config.manufacturer = Some("TeslAux");
    config.product = Some("TeslAux Media");
    // The serial encodes the volume layout, so changing the layout presents a
    // *different* drive.
    //
    // The car caches its media index against the drive identity: after the
    // track length changed from 30 seconds to 10 minutes, it went on showing
    // 30-second tracks while happily playing the longer files. Harmless, but
    // anyone comparing the car's display against the volume would be reading a
    // stale answer.
    config.serial_number = Some(fat::LAYOUT_ID);
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static mut CONFIG_DESC: [u8; 256] = [0; 256];
    static mut BOS_DESC: [u8; 32] = [0; 32];
    static mut MSOS_DESC: [u8; 16] = [0; 16];
    static mut CONTROL_BUF: [u8; 128] = [0; 128];
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

    static mut MSC_CONTROL: MscControl = MscControl;
    // SAFETY: taken once at startup; outlives `usb` (main never returns).
    builder.handler(unsafe { &mut *core::ptr::addr_of_mut!(MSC_CONTROL) });

    // One interface, two bulk endpoints. Bulk-Only Transport needs nothing
    // else — no class-specific descriptors at all.
    let (mut ep_in, mut ep_out) = {
        let mut func = builder.function(
            msc::CLASS_MASS_STORAGE,
            msc::SUBCLASS_SCSI,
            msc::PROTOCOL_BULK_ONLY,
        );
        let mut iface = func.interface();
        let mut alt = iface.alt_setting(
            msc::CLASS_MASS_STORAGE,
            msc::SUBCLASS_SCSI,
            msc::PROTOCOL_BULK_ONLY,
            None,
        );
        // 64 bytes is the full-speed bulk maximum.
        let ep_in = alt.endpoint_bulk_in(None, 64);
        let ep_out = alt.endpoint_bulk_out(None, 64);
        drop(func);
        (ep_in, ep_out)
    };

    spawner.spawn(usb_task(builder.build()).unwrap());

    #[cfg(not(feature = "rp2040-zero"))]
    spawner.spawn(
        status_task(embassy_rp::gpio::Output::new(
            p.PIN_25,
            embassy_rp::gpio::Level::Low,
        ))
        .unwrap(),
    );
    // `Common` must outlive the LED, so it is bound here in main — which never
    // returns — rather than inside a block.
    //
    // Dropping it resets GPIO16's pin function, so the boot colour clocks out
    // and latches and every later write goes to a dead pin. That presents as
    // "the status task never runs", and it is the same fault `car.rs` documents
    // at length. Reintroduced here by declaring `common` inside a block, and
    // found only because the LED was dark in a car where a power problem was
    // the obvious suspect.
    #[cfg(feature = "rp2040-zero")]
    let (mut led_common, led_sm) = {
        use embassy_rp::pio::Pio;
        let Pio { common, sm0, .. } = Pio::new(p.PIO1, Pio1Irqs);
        (common, sm0)
    };
    #[cfg(feature = "rp2040-zero")]
    {
        let led = ws2812::Ws2812::new(&mut led_common, led_sm, p.PIN_16);
        spawner.spawn(status_task(led).unwrap());
    }

    let mut scsi = Scsi::new(fat::TOTAL_SECTORS, SECTOR as u32);
    let mut detector = Detector::new(fat::N_TRACKS, fat::TRACK_FILE_BYTES);
    // Pacing state: a credit of bytes that refills at the playback rate.
    let mut credit: u64 = BURST_BYTES;
    let mut credit_ms: u64 = embassy_time::Instant::now().as_millis();
    let mut cbw_buf = [0u8; 64];
    let mut reply = [0u8; 64];
    let mut sector = [0u8; SECTOR];

    loop {
        ep_out.wait_enabled().await;
        WEDGED.store(false, Ordering::Relaxed);

        loop {
            // --- command phase ---
            let n = match ep_out.read(&mut cbw_buf).await {
                Ok(n) => n,
                Err(EndpointError::Disabled) => break,
                Err(_) => {
                    WEDGED.store(true, Ordering::Relaxed);
                    break;
                }
            };
            let Some(cbw) = Cbw::parse(&cbw_buf[..n]) else {
                // "Not meaningful". The spec wants both endpoints stalled until
                // a Bulk-Only Reset arrives, but `embassy-usb` exposes no stall
                // on a bulk endpoint, so the best available is to stop
                // responding and wait to be re-enabled. A host that sends a
                // malformed CBW is already in trouble; what matters is that we
                // never guess at what it meant.
                WEDGED.store(true, Ordering::Relaxed);
                break;
            };

            let action = scsi.command(&cbw, &mut reply);

            // --- data phase ---
            let (sent, status) = match action {
                Action::None => (0u32, CswStatus::Passed),
                Action::Reply { len } => {
                    // Never send more than the host asked for; a host that
                    // receives extra bytes treats the transfer as a phase error.
                    let len = len.min(cbw.data_len as usize);
                    match write_all(&mut ep_in, &reply[..len]).await {
                        Ok(()) => (len as u32, CswStatus::Passed),
                        Err(_) => {
                            WEDGED.store(true, Ordering::Relaxed);
                            break;
                        }
                    }
                }
                Action::ReadBlocks { lba, blocks } => {
                    // Pace reads of track data, and only those. Metadata — the
                    // FAT, the directory, the partition table — is read while
                    // indexing, and throttling it would make every wake slow.
                    if fat::locate(lba).is_some() {
                        let now = embassy_time::Instant::now().as_millis();
                        let elapsed = now.saturating_sub(credit_ms);
                        credit_ms = now;
                        credit = (credit + elapsed * PLAYBACK_BYTES_PER_SEC * PACE_NUMERATOR
                            / (1000 * PACE_DENOMINATOR))
                            .min(BURST_BYTES);

                        let want = blocks as u64 * SECTOR as u64;
                        if credit < want {
                            // Wait for the credit this read needs rather than
                            // refusing it: the host is entitled to its data,
                            // just not to it early.
                            let short = want - credit;
                            let ms = short * 1000 * PACE_DENOMINATOR
                                / (PLAYBACK_BYTES_PER_SEC * PACE_NUMERATOR);
                            embassy_time::Timer::after_millis(ms.min(2000)).await;
                            credit = want;
                            credit_ms = embassy_time::Instant::now().as_millis();
                        }
                        credit -= want;
                    }

                    let mut sent = 0u32;
                    let mut failed = false;
                    for i in 0..blocks {
                        if let Some(pos) = fat::locate(lba + i) {
                            READ_TRACK_DATA.store(true, Ordering::Relaxed);
                            if pos.track < 32 {
                                TRACKS_SEEN.store(
                                    TRACKS_SEEN.load(Ordering::Relaxed) | (1 << pos.track),
                                    Ordering::Relaxed,
                                );
                            }
                            let now = embassy_time::Instant::now().as_millis() as u32;
                            LAST_READ_MS.store(now, Ordering::Relaxed);
                            let p = detect::Position { track: pos.track, offset: pos.offset };
                            note_event(detector.on_read(p, now));
                        }
                        fat::read_sector(lba + i, &mut sector);
                        if write_all(&mut ep_in, &sector).await.is_err() {
                            failed = true;
                            break;
                        }
                        sent += SECTOR as u32;
                    }
                    SECTORS_READ.store(
                        SECTORS_READ.load(Ordering::Relaxed).wrapping_add(blocks),
                        Ordering::Relaxed,
                    );
                    if failed {
                        WEDGED.store(true, Ordering::Relaxed);
                        break;
                    }
                    (sent, CswStatus::Passed)
                }
                Action::DiscardBlocks { blocks } => {
                    // Read and throw away. The host must see the bytes consumed
                    // — leaving them queued stalls the transfer just as surely
                    // as refusing it would.
                    let mut got = 0u32;
                    let want = blocks * SECTOR as u32;
                    let mut failed = false;
                    while got < want {
                        match ep_out.read(&mut cbw_buf).await {
                            Ok(n) => got += n as u32,
                            Err(_) => {
                                failed = true;
                                break;
                            }
                        }
                    }
                    if failed {
                        WEDGED.store(true, Ordering::Relaxed);
                        break;
                    }
                    (want.min(cbw.data_len), CswStatus::Passed)
                }
                Action::Fail => {
                    if READ_TRACK_DATA.load(Ordering::Relaxed) {
                        LATE_CMD_FAILED.store(true, Ordering::Relaxed);
                    }
                    // The spec would have us stall the data endpoint here, but
                    // `embassy-usb` exposes no stall for bulk. Sending nothing
                    // and reporting the full amount as residue tells the host
                    // the same thing — it asked for N bytes and got none —
                    // and every host tested accepts it. Noted as a divergence
                    // rather than left to be rediscovered.
                    (0, CswStatus::Failed)
                }
            };

            // --- status phase ---
            //
            // Residue is what the host asked for minus what it got. Reporting
            // it wrongly is worse than failing outright.
            let mut out = [0u8; msc::CSW_LEN];
            msc::csw(cbw.tag, cbw.data_len.saturating_sub(sent), status, &mut out);
            if ep_in.write(&out).await.is_err() {
                WEDGED.store(true, Ordering::Relaxed);
                break;
            }
        }
    }
}

/// Write a buffer as a sequence of max-size bulk packets.
///
/// A transfer whose length is an exact multiple of the endpoint size needs no
/// zero-length terminator here: every reply is either shorter than the packet
/// size or a whole number of 512-byte sectors, and the CSW that follows
/// delimits it either way.
async fn write_all<E: EndpointIn>(ep: &mut E, data: &[u8]) -> Result<(), EndpointError> {
    if data.is_empty() {
        return Ok(());
    }
    for chunk in data.chunks(64) {
        ep.write(chunk).await?;
    }
    Ok(())
}

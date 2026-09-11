// SPDX-License-Identifier: MIT
//! `embassy_usb_driver::Driver` over the PIO port.
//!
//! This is the layer the whole architecture was chosen for. With these traits
//! implemented, everything above them is reused rather than rewritten —
//! `teslamic.rs`, the endpoint-less IF3 handler, and the byte-exact 36-byte
//! report descriptor that defeats the "unsupported USB microphone" popup.
//! Rewriting that stack is the one thing this port must not require, and the
//! reason it is a Rust driver rather than a fork of a C library.
//!
//! # Structure
//!
//! One `static` holds everything both sides touch, inside a critical section.
//! [`super::table`] is plain data with no locking of its own precisely so it
//! can be tested; this is where the locking lives, and it is deliberately the
//! only place.
//!
//! The interrupt runs [`super::table::service`] and wakes whichever endpoint
//! changed. The async side polls the same table and registers a waker. Nothing
//! here decides anything about USB — by the time any of it runs, the reply is
//! already on the wire.
//!
//! # Where the receiver runs (Option 2)
//!
//! The timing-critical receive — token, decoder realign, data, response — runs
//! on **core 1**, in [`core1_receive`], and nowhere else. This is the reference
//! driver's architecture and it is here for the reason the reference chose it:
//! a SETUP's data packet follows its token by ~167 ns, and the realign that
//! aligns the decoder to the data has to land inside that window. On core 0 the
//! sequence shares the core with the executor and the native USB interrupt, so
//! the realign arrives at a jittering time relative to the fixed data deadline
//! — which misaligned every SETUP and stalled enumeration. On core 1 there is
//! nothing to preempt it, so the interval is fixed by the code and the wire.
//!
//! The split is: **core 1** polls the start-of-packet flag and runs each
//! transaction, updating the endpoint [`Table`]; **core 0** runs everything
//! else — the executor, the native USB, and the async `Driver`/`Endpoint`/
//! `ControlPipe` methods that read and write those same endpoints. The two
//! share [`STATE`] under a `critical_section`, which on the RP2350 is a
//! hardware spinlock and so is safe across cores. Wakes cross cores too: an
//! [`AtomicWaker`] woken on core 1 signals core 0's executor.
//!
//! `on_packet_start`, the old interrupt entry, still exists and still works;
//! nothing calls it once core 1 is running. It stays as the single-core path
//! for reference and for boards where core 1 is spoken for.
//!
//! # Why not one core
//!
//! Keeping the receiver on core 0 needs the decoder to re-align itself to each
//! packet's SYNC in hardware, so no bit-timed CPU step is on the critical path.
//! That is Option 1, and it founders on a PIO limit: a state machine has one
//! `jmp pin` and cannot conditionally branch on an IRQ or a second pin, so the
//! decoder cannot both branch on D+ to decode and branch on a reset signal to
//! re-align. It would take merging the edge detector and decoder into one
//! state machine that sees the whole packet including its EOP — a from-scratch
//! receiver that has to fit 32 instructions and hold its timing. Worth
//! revisiting; not the path taken here. Core 1 costs no latency (the cross-core
//! audio buffer adds a memory write, not a buffering stage), so nothing is lost
//! by it.
//!
//! # Status
//!
//! **Core 1 does what it was for: it removed the timing jitter.** On hardware,
//! against a real host, the SETUP-data misalignment that used to vary run to
//! run (01, then 05, then 27, then zeros) is now the *same* value every time —
//! 01 4b f9 6e. That determinism is the whole reason for the second core, and
//! it is the difference between an unfixable moving target and a bug that can
//! be read off a single capture.
//!
//! It has **not** carried enumeration through yet. `setup` and `addr` stay at
//! zero. The residual is now a specific, repeatable thing rather than a timing
//! mystery: the deterministic bad token 01 4b f9 6e is a DATA1 packet whose PID
//! (4b) decodes correctly but which arrives with a spurious 01 where its SYNC
//! belongs, and is serviced as its own token rather than as a data stage. The
//! control-transfer sequencing — which packet is the setup data, which is a
//! status stage, when a data packet should be consumed inline versus serviced
//! fresh — is the work left, and it is now debuggable because the hardware
//! timing underneath it is finally steady.
//!
//! **None of this has run.** It is the largest piece of the port that no host
//! test can reach and no single-board harness exercises: it only comes alive
//! against a real host, where enumeration is the test. Expect the first attempt
//! not to work and expect `piocap` to be how the reason is found.
//!
//! Two things are known-incomplete rather than merely untested:
//!
//! * **Bus events are synthesised.** A real reset is the host holding SE0 for
//!   10 ms, and nothing here measures that; [`Bus::poll`] reports a power-on and
//!   a reset once and then waits forever. Suspend and resume are not detected at
//!   all.
//! * **There is no timeout on a stalled transfer.** An endpoint whose host stops
//!   collecting will simply never complete.

use core::cell::RefCell;
use core::future::poll_fn;
use core::task::Poll;

use critical_section::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType,
    Event, Unsupported,
};

use super::device::EpType;
use super::endpoint::Handshakes;
use super::hw::HwPort;
use super::table::{service, Table, MAX_PACKET};
use super::txn::Engine;

/// Endpoint numbers this driver supports.
///
/// Four is enough for this project — control, the audio stream, and the two HID
/// interfaces — and each number costs about 5.5 KB across both directions. See
/// [`super::table::MAX_PACKET`].
pub const NUM_ENDPOINTS: usize = 4;

/// Everything the interrupt and the application share.
struct Inner {
    table: Table<NUM_ENDPOINTS>,
    engine: Engine,
    handshakes: Handshakes,
    port: HwPort,
    /// Scratch for a received packet, owned here so the interrupt never
    /// allocates and never borrows from a caller.
    rx: [u8; MAX_PACKET + 4],
}

static STATE: Mutex<RefCell<Option<Inner>>> = Mutex::new(RefCell::new(None));

/// One waker per endpoint and direction: IN endpoints occupy the low half.
static WAKERS: [AtomicWaker; NUM_ENDPOINTS * 2] =
    [const { AtomicWaker::new() }; NUM_ENDPOINTS * 2];

/// Counters, so a device that does not enumerate can say how far it got.
///
/// Plain atomics rather than anything in the critical section: the interrupt
/// writes them and the diagnostic reads them, and a torn read of a counter is
/// not worth excluding the interrupt for.
pub mod stats {
    use core::sync::atomic::{AtomicU32, Ordering};

    /// Every time the start-of-packet interrupt handler was entered, counted
    /// before anything else happens. Compared against TRANSACTIONS this says
    /// whether packets are being lost before the handler runs or inside it.
    pub static IRQ_ENTRIES: AtomicU32 = AtomicU32::new(0);
    /// Every transaction the interrupt has serviced.
    pub static TRANSACTIONS: AtomicU32 = AtomicU32::new(0);
    /// Start-of-frame tokens. A host that is talking at all sends one every
    /// millisecond, so this separates "nothing is arriving" from "nothing is
    /// arriving *for us*".
    pub static SOF: AtomicU32 = AtomicU32::new(0);
    /// Tokens addressed to this device that produced a reply.
    pub static ANSWERED: AtomicU32 = AtomicU32::new(0);
    /// Tokens that failed their own checks — bad sync, bad PID, failed CRC5.
    /// A steady trickle of these against a live host points at clock recovery
    /// rather than at anything above the wire.
    pub static BAD: AtomicU32 = AtomicU32::new(0);
    /// SETUP packets received. The first one is the host asking what we are.
    pub static SETUPS: AtomicU32 = AtomicU32::new(0);
    /// Times the receiver had to be released from a stuck EOP. A bus reset
    /// causes one; a steady stream of them means something else is stopping it.
    pub static REARMS: AtomicU32 = AtomicU32::new(0);

    /// The last well-formed token this device declined to act on, packed as
    /// `pid << 16 | address << 8 | endpoint`, with the top byte set once
    /// anything has been recorded.
    ///
    /// "Ignored" covers a token for another device, an endpoint we do not have,
    /// a PID that is not a token at all, and an isochronous endpoint with
    /// nothing to send. Those need different fixes, and a count cannot tell
    /// them apart — a host sending SETUPs we discard looks exactly like a host
    /// sending nothing until the PID is visible.
    pub static LAST_IGNORED: AtomicU32 = AtomicU32::new(0);
    /// How many distinct PIDs have been ignored, as a bitmask over the low
    /// nibble. A single stray fragment reads very differently from a host
    /// retrying the same request.
    pub static IGNORED_PIDS: AtomicU32 = AtomicU32::new(0);

    /// The four bytes of the last packet that failed its checks, packed big
    /// end first, with the top bit of a separate flag word set once valid.
    ///
    /// What they are is the whole question. `80 2d ..` is a SETUP token we
    /// mangled — a decode fault. `80 c3 ..` is a DATA packet, which means the
    /// receiver joined a transaction late and read the data stage as though it
    /// were the start of a packet, and the token before it was missed entirely.
    pub static LAST_BAD: AtomicU32 = AtomicU32::new(0);
    pub static LAST_BAD_SEEN: AtomicU32 = AtomicU32::new(0);

    pub(super) fn bump(c: &AtomicU32) {
        c.fetch_add(1, Ordering::Relaxed);
    }

    /// A snapshot: transactions, SOFs, answered, bad, setups, re-arms.
    pub fn read() -> (u32, u32, u32, u32, u32, u32) {
        (
            TRANSACTIONS.load(Ordering::Relaxed),
            SOF.load(Ordering::Relaxed),
            ANSWERED.load(Ordering::Relaxed),
            BAD.load(Ordering::Relaxed),
            SETUPS.load(Ordering::Relaxed),
            REARMS.load(Ordering::Relaxed),
        )
    }
}

/// Re-arm the receiver if it has stopped, returning whether it had.
///
/// The edge detector ends a packet with `irq wait`, which sets the EOP flag and
/// blocks until something acknowledges it. Normally that something is the end
/// of a transaction. A **bus reset** is not a transaction: it is ten
/// milliseconds of SE0, which the detector reads as an EOP and stops on, and
/// nothing downstream ever runs to release it. The device goes deaf at exactly
/// the moment the host is trying to talk to it, and stays that way.
///
/// So this has to be called periodically by something that is not the packet
/// path. It is cheap — one register read when there is nothing to do.
pub fn service_bus() -> bool {
    critical_section::with(|cs| {
        let mut guard = STATE.borrow_ref_mut(cs);
        let Some(inner) = guard.as_mut() else {
            return false;
        };
        use super::txn::Port as _;
        if !inner.port.eop() {
            return false;
        }
        inner.port.reset_receiver();
        stats::bump(&stats::REARMS);
        true
    })
}

/// The address the device currently answers to. Zero until SET_ADDRESS lands,
/// so a non-zero value proves enumeration got at least that far.
pub fn address() -> u8 {
    with(|inner| inner.engine.address)
}

const fn waker_index(ep: u8, is_in: bool) -> usize {
    ep as usize + if is_in { 0 } else { NUM_ENDPOINTS }
}

/// The dedicated receive loop, to run on core 1 and nowhere else.
///
/// This is Option 2, and the reason it exists is timing. A SETUP's data packet
/// follows its token by ~167 ns; the receiver has to read the token, realign
/// the decoder, and catch the data inside that window. On core 0 the sequence
/// shares the core with the executor and the native USB interrupt, and the
/// realign lands at a jittering time relative to the data — which misaligned
/// every SETUP and stalled enumeration.
///
/// On core 1 there is nothing else. The loop polls the start-of-packet flag and
/// runs the whole transaction with no preemption, so the interval from token to
/// realign is fixed by the code path and the wire, not by whatever else wanted
/// the core. That determinism is the entire point — it is the reference's
/// architecture, which runs its receiver on core 1 for exactly this reason.
///
/// Never returns. `spawn_core1` requires that.
pub fn core1_receive() -> ! {
    loop {
        // Poll the start-of-packet flag directly. No interrupt: an interrupt
        // would hand control to core 1's vector table with its own latency, and
        // the point is a tight, predictable path.
        let raised = embassy_rp::pac::PIO0.irq().read().irq() & (1 << super::rx::IRQ_RX_START) != 0;
        if raised {
            // Acknowledge before servicing, so a packet that starts while we are
            // still in `service` sets the flag again and is not lost.
            embassy_rp::pac::PIO0
                .irq()
                .write(|w| w.set_irq(1 << super::rx::IRQ_RX_START));
            service_one();
        }
    }
}

/// Service one transaction. Call from the `IRQ_RX_START` handler.
///
/// Everything inside the critical section is bookkeeping; the packet the host
/// asked for went out during [`service`], before any of this.
pub fn on_packet_start() {
    embassy_rp::pac::PIO0
        .irq()
        .write(|w| w.set_irq(1 << super::rx::IRQ_RX_START));
    service_one();
}

/// The body of one transaction: decide, act, record, wake. Shared by the core-1
/// loop and the (legacy) interrupt path.
///
/// The wake is done outside the critical section it takes, because a woken
/// core-0 task may run immediately and will want the lock this holds — and the
/// wake crosses cores, which is why it is an atomic waker rather than anything
/// core-local.
fn service_one() {
    stats::bump(&stats::IRQ_ENTRIES);
    let woken = critical_section::with(|cs| {
        let mut guard = STATE.borrow_ref_mut(cs);
        // Not started yet: core 1 may be polling before Driver::start has run.
        // Nothing to do, and certainly nothing to panic about.
        let Some(inner) = guard.as_mut() else {
            return None;
        };
        // Split the borrow: the engine, the port and the table are all needed
        // at once and they are separate fields.
        let Inner {
            table,
            engine,
            handshakes,
            port,
            rx,
            ..
        } = inner;
        let outcome = service(engine, port, handshakes, table, rx);
        use super::txn::Outcome;
        stats::bump(&stats::TRANSACTIONS);
        match outcome {
            Outcome::Sent { ep, .. } => {
                stats::bump(&stats::ANSWERED);
                Some(waker_index(ep, true))
            }
            Outcome::Received { ep, setup, .. } => {
                stats::bump(&stats::ANSWERED);
                if setup {
                    stats::bump(&stats::SETUPS);
                }
                Some(waker_index(ep, false))
            }
            Outcome::Handshake(_) => {
                stats::bump(&stats::ANSWERED);
                None
            }
            Outcome::Sof(_) => {
                stats::bump(&stats::SOF);
                None
            }
            Outcome::BadToken { bytes } => {
                stats::bump(&stats::BAD);
                stats::LAST_BAD.store(
                    u32::from_be_bytes(bytes),
                    core::sync::atomic::Ordering::Relaxed,
                );
                stats::LAST_BAD_SEEN.store(1, core::sync::atomic::Ordering::Relaxed);
                None
            }
            Outcome::Corrupt => {
                stats::bump(&stats::BAD);
                None
            }
            Outcome::Ignored {
                pid,
                address,
                endpoint,
            } => {
                stats::LAST_IGNORED.store(
                    1 << 24 | (pid as u32) << 16 | (address as u32) << 8 | endpoint as u32,
                    core::sync::atomic::Ordering::Relaxed,
                );
                stats::IGNORED_PIDS.fetch_or(
                    1 << (pid & 0x0f),
                    core::sync::atomic::Ordering::Relaxed,
                );
                None
            }
            Outcome::Timeout => None,
        }
    });
    // Waking outside the critical section: a woken task may run immediately,
    // and it will want the lock this was holding.
    if let Some(i) = woken {
        WAKERS[i].wake();
    }
}

/// Borrow the shared state. Panics if the driver has not been started, which
/// is a programming error rather than a bus condition.
fn with<R>(f: impl FnOnce(&mut Inner) -> R) -> R {
    critical_section::with(|cs| {
        let mut guard = STATE.borrow_ref_mut(cs);
        f(guard.as_mut().expect("PIO USB driver not started"))
    })
}

/// The driver, before [`Driver::start`] consumes it.
pub struct PioUsb {
    port: HwPort,
    /// The type each allocated endpoint was asked for, kept until [`start`]
    /// builds the table. Losing it here would default an isochronous endpoint
    /// to bulk, which inverts its handshake behaviour — the one rule this
    /// driver exists to get right.
    allocated: [Option<EpType>; NUM_ENDPOINTS * 2],
}

impl PioUsb {
    /// `port` must already be configured — programs loaded, state machines
    /// running, pins set up. See [`super::bus`].
    pub fn new(port: HwPort) -> Self {
        Self {
            port,
            allocated: [None; NUM_ENDPOINTS * 2],
        }
    }

    fn alloc(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
        is_in: bool,
    ) -> Result<Ep, EndpointAllocError> {
        if max_packet_size as usize > MAX_PACKET {
            return Err(EndpointAllocError);
        }
        // Honour a requested address, or take the lowest free number. Endpoint
        // zero is the control pipe and is never handed out this way.
        let number = match ep_addr {
            Some(a) => {
                let n = a.index();
                if n == 0
                    || n >= NUM_ENDPOINTS
                    || self.allocated[waker_index(n as u8, is_in)].is_some()
                {
                    return Err(EndpointAllocError);
                }
                n
            }
            None => (1..NUM_ENDPOINTS)
                .find(|n| self.allocated[waker_index(*n as u8, is_in)].is_none())
                .ok_or(EndpointAllocError)?,
        };
        self.allocated[waker_index(number as u8, is_in)] = Some(ep_type_of(ep_type));
        Ok(Ep {
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(
                    number,
                    if is_in {
                        Direction::In
                    } else {
                        Direction::Out
                    },
                ),
                ep_type,
                max_packet_size,
                interval_ms,
            },
        })
    }
}

/// One allocated endpoint. Holds no buffers: those live in the table, because
/// the DMA reads them after the call that filled them has returned.
pub struct Ep {
    info: EndpointInfo,
}

fn ep_type_of(t: EndpointType) -> EpType {
    match t {
        EndpointType::Control => EpType::Control,
        EndpointType::Isochronous => EpType::Isochronous,
        EndpointType::Bulk => EpType::Bulk,
        EndpointType::Interrupt => EpType::Interrupt,
    }
}

impl embassy_usb_driver::Endpoint for Ep {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let n = self.info.addr.index() as u8;
        let is_in = self.info.addr.direction() == Direction::In;
        poll_fn(|cx| {
            WAKERS[waker_index(n, is_in)].register(cx.waker());
            with(|inner| {
                match inner.table.slot(n, is_in) {
                    Some(s) if s.enabled => Poll::Ready(()),
                    _ => Poll::Pending,
                }
            })
        })
        .await
    }
}

impl embassy_usb_driver::EndpointIn for Ep {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        let n = self.info.addr.index() as u8;
        if buf.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        poll_fn(|cx| {
            WAKERS[waker_index(n, true)].register(cx.waker());
            with(|inner| {
                let Some(slot) = inner.table.slot_mut(n, true) else {
                    return Poll::Ready(Err(EndpointError::Disabled));
                };
                if !slot.enabled {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                // `queue` refuses while the previous packet is still the
                // hardware's, which is exactly the condition to wait on.
                if slot.queue(buf) {
                    Poll::Ready(Ok(()))
                } else {
                    Poll::Pending
                }
            })
        })
        .await
    }
}

impl embassy_usb_driver::EndpointOut for Ep {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let n = self.info.addr.index() as u8;
        poll_fn(|cx| {
            WAKERS[waker_index(n, false)].register(cx.waker());
            with(|inner| {
                let Some(slot) = inner.table.slot_mut(n, false) else {
                    return Poll::Ready(Err(EndpointError::Disabled));
                };
                if !slot.enabled {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                match slot.take(buf) {
                    Some(len) => Poll::Ready(Ok(len)),
                    None => Poll::Pending,
                }
            })
        })
        .await
    }
}

/// The control pipe: endpoint zero, both directions.
pub struct Control {
    max_packet_size: u16,
}

impl embassy_usb_driver::ControlPipe for Control {
    fn max_packet_size(&self) -> usize {
        self.max_packet_size as usize
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|cx| {
            WAKERS[waker_index(0, false)].register(cx.waker());
            with(|inner| {
                let Some(slot) = inner.table.slot_mut(0, false) else {
                    return Poll::Pending;
                };
                // Only a SETUP starts a control transfer. An OUT on endpoint
                // zero belongs to the data stage of one already in progress and
                // must not be mistaken for the start of another.
                if !slot.complete || !slot.was_setup {
                    return Poll::Pending;
                }
                let mut out = [0u8; 8];
                let n = slot.take(&mut out).unwrap_or(0);
                if n != 8 {
                    // A SETUP is always eight bytes. Anything else is damage.
                    return Poll::Pending;
                }
                Poll::Ready(out)
            })
        })
        .await
    }

    async fn data_out(
        &mut self,
        buf: &mut [u8],
        _first: bool,
        _last: bool,
    ) -> Result<usize, EndpointError> {
        poll_fn(|cx| {
            WAKERS[waker_index(0, false)].register(cx.waker());
            with(|inner| {
                let Some(slot) = inner.table.slot_mut(0, false) else {
                    return Poll::Ready(Err(EndpointError::Disabled));
                };
                if slot.complete && slot.was_setup {
                    // A new transfer has started; this one is abandoned.
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                match slot.take(buf) {
                    Some(len) => Poll::Ready(Ok(len)),
                    None => Poll::Pending,
                }
            })
        })
        .await
    }

    async fn data_in(&mut self, data: &[u8], _first: bool, _last: bool) -> Result<(), EndpointError> {
        write_ep0(data).await
    }

    async fn accept(&mut self) {
        // The status stage of a control-write transfer: a zero-length DATA1.
        let _ = write_ep0(&[]).await;
    }

    async fn reject(&mut self) {
        with(|inner| {
            for is_in in [true, false] {
                if let Some(slot) = inner.table.slot_mut(0, is_in) {
                    slot.state.stalled = true;
                }
            }
        });
    }

    async fn accept_set_address(&mut self, addr: u8) {
        // Order matters and is the whole reason this is a separate method: the
        // status stage still belongs to the old address. Answering it from the
        // new one would mean answering nothing, and enumeration would stall
        // here every time.
        let _ = write_ep0(&[]).await;
        with(|inner| inner.engine.address = addr);
    }
}

/// Queue a packet on endpoint zero IN and wait for the hardware to take it.
async fn write_ep0(data: &[u8]) -> Result<(), EndpointError> {
    poll_fn(|cx| {
        WAKERS[waker_index(0, true)].register(cx.waker());
        with(|inner| {
            let Some(slot) = inner.table.slot_mut(0, true) else {
                return Poll::Ready(Err(EndpointError::Disabled));
            };
            if slot.queue(data) {
                Poll::Ready(Ok(()))
            } else {
                Poll::Pending
            }
        })
    })
    .await
}

/// Bus-level control.
pub struct PioBus {
    /// Which synthetic event to report next. See the module's status note:
    /// real reset detection is not implemented.
    stage: u8,
}

impl embassy_usb_driver::Bus for PioBus {
    async fn enable(&mut self) {}

    async fn disable(&mut self) {}

    async fn poll(&mut self) -> Event {
        // Synthesised, not observed. A real reset is the host holding SE0 for
        // 10 ms and nothing here measures that; this reports a plug-in once so
        // the stack above reaches its configured state, then never returns.
        match self.stage {
            0 => {
                self.stage = 1;
                Event::PowerDetected
            }
            1 => {
                self.stage = 2;
                Event::Reset
            }
            _ => core::future::pending().await,
        }
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let n = ep_addr.index() as u8;
        let is_in = ep_addr.direction() == Direction::In;
        with(|inner| {
            if let Some(slot) = inner.table.slot_mut(n, is_in) {
                slot.enabled = enabled;
            }
        });
        // A task may be waiting for exactly this.
        WAKERS[waker_index(n, is_in)].wake();
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        let n = ep_addr.index() as u8;
        let is_in = ep_addr.direction() == Direction::In;
        with(|inner| {
            if let Some(slot) = inner.table.slot_mut(n, is_in) {
                slot.state.stalled = stalled;
                if !stalled {
                    // Clearing a halt resets the toggle: the host expects the
                    // next packet on DATA0, and carrying on from wherever the
                    // stall happened would look like a retransmission.
                    slot.state.data1 = false;
                }
            }
        });
        WAKERS[waker_index(n, is_in)].wake();
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        let n = ep_addr.index() as u8;
        let is_in = ep_addr.direction() == Direction::In;
        with(|inner| {
            inner
                .table
                .slot(n, is_in)
                .map(|s| s.state.stalled)
                .unwrap_or(false)
        })
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        // Driving a K for 1 to 15 ms. Not implemented, and not needed: this
        // device is bus-powered and never suspends the host.
        Err(Unsupported)
    }
}

impl<'a> embassy_usb_driver::Driver<'a> for PioUsb {
    type EndpointOut = Ep;
    type EndpointIn = Ep;
    type ControlPipe = Control;
    type Bus = PioBus;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.alloc(ep_type, ep_addr, max_packet_size, interval_ms, false)
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.alloc(ep_type, ep_addr, max_packet_size, interval_ms, true)
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let mut table: Table<NUM_ENDPOINTS> = Table::new();
        // Endpoint zero always exists, in both directions, before anything is
        // configured: it is how the host asks what this device is.
        table.allocate(0, true, EpType::Control);
        table.allocate(0, false, EpType::Control);
        for (i, ep_type) in self.allocated.iter().enumerate() {
            let Some(ep_type) = ep_type else { continue };
            let is_in = i < NUM_ENDPOINTS;
            let n = (i % NUM_ENDPOINTS) as u8;
            if n != 0 {
                table.allocate(n, is_in, *ep_type);
            }
        }

        critical_section::with(|cs| {
            *STATE.borrow_ref_mut(cs) = Some(Inner {
                table,
                engine: Engine::new(),
                handshakes: Handshakes::new(),
                port: self.port,
                rx: [0; MAX_PACKET + 4],
            });
        });

        (
            PioBus { stage: 0 },
            Control {
                max_packet_size: control_max_packet_size,
            },
        )
    }
}

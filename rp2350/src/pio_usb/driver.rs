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
//! # Status
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

const fn waker_index(ep: u8, is_in: bool) -> usize {
    ep as usize + if is_in { 0 } else { NUM_ENDPOINTS }
}

/// Service one transaction. Call from the `IRQ_RX_START` handler.
///
/// Everything inside the critical section is bookkeeping; the packet the host
/// asked for went out during [`service`], before any of this.
pub fn on_packet_start() {
    let woken = critical_section::with(|cs| {
        let mut guard = STATE.borrow_ref_mut(cs);
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
        match outcome {
            Outcome::Sent { ep, .. } => Some(waker_index(ep, true)),
            Outcome::Received { ep, .. } => Some(waker_index(ep, false)),
            _ => None,
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

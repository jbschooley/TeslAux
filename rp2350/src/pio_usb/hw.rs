// SPDX-License-Identifier: MIT
//! The hardware behind [`super::txn::Port`].
//!
//! Deliberately the thinnest thing that can work. Everything worth getting
//! wrong lives in `txn` and `device`, where a host test can reach it; this
//! moves bytes and starts DMAs and makes no decisions at all.
//!
//! # Why this talks to the PAC directly
//!
//! An interrupt handler cannot hold embassy's typed state-machine handles
//! without a good deal of lifetime and `static` machinery, and it cannot use
//! `dma_push` at all: that returns a `Transfer` whose `Drop` **aborts the
//! channel**, so firing one and returning would cancel the packet on the way
//! out. What is needed instead is fire-and-forget — arm the DMA, let the
//! transfer finish on its own — and the register sequence for that is short
//! and fixed.
//!
//! So this holds raw PAC handles and plain indices. It is `Copy`, `Send` and
//! has no lifetimes, which is what makes it usable from a `static` in an
//! interrupt handler.

use embassy_rp::pac;

use super::rx::{DECODER_TRIGGER, IRQ_RX_EOP, IRQ_RX_START, MOV_OSR_NOT_NULL, SET_X_ZERO};

/// Spins waiting for the data packet's start of packet before reading it.
///
/// Interrupts are disabled here, so this is bounded small — a few microseconds,
/// comfortably longer than the token-to-data gap. The loop body is a register
/// read and a test, a handful of cycles, so this is roughly 5 µs at 96 MHz.
const START_SPINS: u32 = 1024;

/// Offset of `start:` in the edge detector: instruction 0 is the `irq wait`
/// that ends a packet, so waiting for the next one begins at 1.
const EDGE_START_OFFSET: u16 = 1;
use super::tx::PinOrder;
use super::txn::Port;

/// Which PIO block and state machines the USB port occupies.
#[derive(Copy, Clone)]
pub struct HwPort {
    pio: pac::pio::Pio,
    /// Index of the PIO block, 0..=2. Needed for the DMA data request, which
    /// is `block * 8 + state machine`.
    pio_no: u8,
    tx_sm: usize,
    rx_sm: usize,
    /// The edge detector's state machine. Restarting the decoder alone is not
    /// enough — see [`HwPort::reset_receiver`].
    edge_sm: usize,
    /// DMA channel used to send packets.
    dma: usize,
    /// Instruction offset the transmitter was loaded at. Zero in practice —
    /// its symbols are absolute addresses — but carried rather than assumed.
    tx_origin: u16,
    /// Instruction offset the NRZI decoder was loaded at, for resetting it.
    dec_origin: u16,
    /// Instruction offset the edge detector was loaded at. Its `start:` is one
    /// past this — instruction 0 is the `irq wait` that ends a packet.
    edge_origin: u16,
    order: PinOrder,
}


impl HwPort {
    /// `pio_no` is 0, 1 or 2 and must match `pio`; they are separate because
    /// the PAC handle does not carry its own index and the DMA needs it.
    pub const fn new(
        pio: pac::pio::Pio,
        pio_no: u8,
        tx_sm: usize,
        rx_sm: usize,
        edge_sm: usize,
        dma: usize,
        tx_origin: u16,
        dec_origin: u16,
        edge_origin: u16,
        order: PinOrder,
    ) -> Self {
        Self {
            pio,
            pio_no,
            tx_sm,
            rx_sm,
            edge_sm,
            dma,
            tx_origin,
            dec_origin,
            edge_origin,
            order,
        }
    }

    /// Take the bus, driving it to idle J.
    ///
    /// The transmitter's symbol values only address instructions 0..=3, so the
    /// `set pindirs` at offset 4 is unreachable from the symbol stream and has
    /// to be injected. Without it the packet clocks out onto pins that are
    /// still inputs and the bus never moves — silently.
    fn take_bus(&self) {
        let instr = super::bus::sideset_j(self.order) | (self.tx_origin + super::bus::TX_START_OFFSET);
        self.pio
            .sm(self.tx_sm)
            .instr()
            .write(|w| w.set_instr(instr));
    }

    /// Arm the DMA and return without waiting.
    ///
    /// Fire-and-forget on purpose: this runs inside the bus turnaround window,
    /// and the transfer has to outlive the call. `symbols` must therefore stay
    /// valid until the packet has gone out, which is why endpoint buffers are
    /// owned for the life of the driver rather than borrowed per transfer.
    fn fire(&self, symbols: &[u32]) {
        let ch = pac::DMA.ch(self.dma);
        ch.read_addr().write_value(symbols.as_ptr() as u32);
        ch.write_addr()
            .write_value(self.pio.txf(self.tx_sm).as_ptr() as u32);
        ch.trans_count().write(|w| w.set_count(symbols.len() as u32));
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        ch.ctrl_trig().write(|w| {
            // One data request per FIFO slot, from this state machine's TX.
            w.set_treq_sel(pac::dma::vals::TreqSel::from(
                self.pio_no * 8 + self.tx_sm as u8,
            ));
            w.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
            w.set_incr_read(true);
            w.set_incr_write(false);
            w.set_chain_to(self.dma as u8);
            w.set_en(true);
        });
    }

}

impl Port for HwPort {
    fn rx(&mut self) -> Option<u8> {
        // One bit per state machine; set means that FIFO is empty.
        if self.pio.fstat().read().rxempty() & (1 << self.rx_sm) != 0 {
            return None;
        }
        // The decoder shifts right with an 8-bit threshold, so the byte
        // arrives in the top octet.
        Some((self.pio.rxf(self.rx_sm).read() >> 24) as u8)
    }

    fn eop(&self) -> bool {
        self.pio.irq().read().irq() & (1 << IRQ_RX_EOP) != 0
    }



    fn transmit(&mut self, symbols: &[u32]) {
        self.take_bus();
        self.fire(symbols);
    }

    fn tx_busy(&self) -> bool {
        pac::DMA.ch(self.dma).ctrl_trig().read().busy()
    }

    fn reset_receiver(&mut self) {
        HwPort::do_reset(self)
    }

    fn clear_eop(&mut self) {
        self.pio.irq().write(|w| w.set_irq(1 << IRQ_RX_EOP));
    }

    fn realign_decoder(&mut self) {
        // Reset the decoder for the data packet that immediately follows a
        // token, leaving the free-running edge detector locked on the incoming
        // start of packet.
        //
        // The wire proved the data packet is clean and our clock recovery
        // decodes it correctly — so this is purely about resetting the decoder's
        // state (NRZI previous level, FIFO, byte boundary) cleanly and at the
        // right time. Only the full disable/clear-FIFO/restart does that: an
        // in-place reset while the state machine runs neither clears the FIFO
        // (the token's trailing bytes leak into the data read) nor reliably
        // resets the NRZI state, and the data decodes as continuous garbage.
        //
        // Clear the token's leftover end-of-packet AND decoder-trigger flags —
        // but NOT the start-of-packet flag. The interrupt that got us here
        // already cleared the token's start of packet on entry, so the next
        // start flag to appear is the data packet's own, and
        // [`Port::wait_rx_start`] must be left to see it.
        //
        // Clearing the decoder-trigger flag is what keeps the data packet from
        // decoding one bit late. The edge detector raises it once per bit and
        // the decoder consumes it with `wait 1 irq`; a trigger left set from the
        // token makes the freshly-armed decoder's first wait return at once and
        // decode a phantom bit before the real SYNC, shifting the whole packet
        // by one (SYNC read as 81 rather than 80).
        self.pio
            .irq()
            .write(|w| w.set_irq((1 << IRQ_RX_EOP) | (1 << DECODER_TRIGGER)));

        // A fast disable/reset/enable. The forced `set x, 0` and `jmp` only take
        // effect while the state machine is stopped — done in place, live, the
        // decoder kept its stale NRZI previous-line-state and decoded the data's
        // first bit wrong (SYNC read as 81 instead of 80, shifting the whole
        // packet). So disable is required. But the full reset's FIFO clear (two
        // read-modify-writes of the join state) made it ~1 µs, longer than the
        // ~300-480 ns before the data packet's start of packet, so it re-armed
        // partway through. Skip it: the FIFO is already empty (the token's four
        // bytes were just read out), and the OSR stays all-ones on its own
        // (`in osr, 1` reads it without consuming it). That leaves disable,
        // `sm_restart`, forced `jmp` to the decoder start, `set x, 0`, enable —
        // five register writes, short enough to finish inside the gap.
        let ctrl_bit = 1u8 << self.rx_sm;
        self.pio
            .ctrl()
            .modify(|w| w.set_sm_enable(w.sm_enable() & !ctrl_bit));
        self.pio.ctrl().modify(|w| w.set_sm_restart(ctrl_bit));
        let sm = self.pio.sm(self.rx_sm);
        sm.instr().write(|w| w.set_instr(self.dec_origin));
        sm.instr().write(|w| w.set_instr(SET_X_ZERO));
        self.pio
            .ctrl()
            .modify(|w| w.set_sm_enable(w.sm_enable() | ctrl_bit));
    }

    fn wait_rx_start(&self) -> bool {
        // Spin until the edge detector flags the data packet's start of packet,
        // or give up. Bounded well under the old byte timeout: at 96 MHz this is
        // roughly 5 µs, longer than the token-to-data gap and short enough not
        // to stall the interrupt. Interrupts are off here, so like the byte
        // spin this stays deliberately small.
        for _ in 0..START_SPINS {
            if self.pio.irq().read().irq() & (1 << IRQ_RX_START) != 0 {
                return true;
            }
        }
        false
    }

}

impl HwPort {
    /// Reset the NRZI decoder state machine to a clean packet start, leaving the
    /// edge detector running.
    ///
    /// Stop it, clear the FIFO, restart the clock and shift counters, jump the
    /// program counter back to the decoder's `start`, zero X (the NRZI
    /// previous-line-state), reload the all-ones OSR the decoder shifts a 1 out
    /// of, then re-enable. Every step matters: without the FIFO clear the token's
    /// trailing bytes are read as data; without the jump the decoder resumes
    /// mid-loop on the wrong bit-stuffing phase; without the OSR reload `in osr,
    /// 1` shifts zeros and the payload reads as 00 00 ...
    fn reset_decoder_sm(&self) {
        let sm = self.pio.sm(self.rx_sm);
        let ctrl_bit = 1u8 << self.rx_sm;
        self.pio
            .ctrl()
            .modify(|w| w.set_sm_enable(w.sm_enable() & !ctrl_bit));

        // The SDK's `pio_sm_clear_fifos` is a join-state toggle: changing FJOIN
        // discards whatever is queued, and toggling it back restores the
        // configuration. `sm_restart` alone resets the shift counters but leaves
        // the FIFO contents alone.
        for _ in 0..2 {
            sm.shiftctrl().modify(|w| w.set_fjoin_rx(!w.fjoin_rx()));
        }
        self.pio.ctrl().modify(|w| w.set_clkdiv_restart(ctrl_bit));
        self.pio.ctrl().modify(|w| w.set_sm_restart(ctrl_bit));
        for instr in [self.dec_origin, SET_X_ZERO, MOV_OSR_NOT_NULL] {
            sm.instr().write(|w| w.set_instr(instr));
        }
        self.pio
            .ctrl()
            .modify(|w| w.set_sm_enable(w.sm_enable() | ctrl_bit));
    }

    /// Revive the receiver after a packet: reset the decoder and put the edge
    /// detector back to waiting for a start of packet.
    ///
    /// Callable from the EOP interrupt as well as the transaction path, because
    /// the edge detector blocks on `irq wait` at every EOP and something must
    /// clear it promptly — the bottleneck the stub handler exposed was this
    /// running only at transaction or report-loop rate.
    pub fn do_reset(&mut self) {
        self.reset_decoder_sm();

        // And the edge detector, which clearing the flag alone does not put
        // right. Releasing it from `irq wait` resumes it wherever it happened
        // to be; what is wanted is `start:`, waiting for the next start of
        // packet. Without this the port answers one packet and then goes quiet
        // until something outside re-arms it — which reads on a counter as a
        // host that has stopped talking.
        let edge_bit = 1u8 << self.edge_sm;
        self.pio
            .ctrl()
            .modify(|w| w.set_sm_enable(w.sm_enable() & !edge_bit));
        self.pio.ctrl().modify(|w| w.set_sm_restart(edge_bit));
        self.pio
            .sm(self.edge_sm)
            .instr()
            .write(|w| w.set_instr(self.edge_origin + EDGE_START_OFFSET));
        self.pio
            .ctrl()
            .modify(|w| w.set_sm_enable(w.sm_enable() | edge_bit));

        // Clear every receive flag last, so nothing left over from the packet
        // just ended triggers the freshly restarted machines.
        self.pio.irq().write(|w| w.set_irq(0xf << 1));
    }
}

// The raw PAC handle is a bare pointer, so this is not automatically `Send`.
// It is safe to move between contexts for the same reason the PAC's own
// handles are: it names a fixed peripheral, and every access here is a single
// volatile register operation.
unsafe impl Send for HwPort {}

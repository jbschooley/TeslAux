// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! Reproduce the token-then-data alignment problem on the bench.
//!
//! A SETUP is two packets back to back: a token, then a DATA0 ~2 bit times
//! behind it. Against a real host the second packet decoded misaligned, and the
//! misalignment jittered — the reason to suspect the CPU's timing rather than
//! the decoder. This settles which it is, deterministically and without a host.
//!
//! It transmits two packets in one continuous DMA — packet A, an EOP, a short
//! idle, packet B — and reads the decoder straight through, **with no CPU
//! realign between them**. If packet B comes back starting 0x80 (its SYNC), the
//! decoder handles back-to-back packets on its own and the host-side fault is
//! purely the async realign timing. If packet B is shifted, the decoder cannot
//! hold alignment across a packet boundary, and that is the thing a
//! SYNC-aligning decoder has to fix.
//!
//! The drivers stay engaged across the gap: the transmitter's "release" symbol
//! would drop them mid-stream and only a CPU-injected instruction re-engages
//! them, so the gap is driven SE0 then J instead, which is what the wire does
//! anyway between two same-direction packets.
//!
//! ```text
//! screen /dev/tty.usbmodem* 115200
//! ```

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, PIO1, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;

use teslamic_rp2350::board;
use teslamic_rp2350::pio_usb::{
    bus,
    packet::{build_data, Pid},
    tx::Symbol,
};

#[path = "../../../rp2040/src/ws2812.rs"]
mod ws2812;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    teslamic_rp2350::panic_led::strobe()
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

struct Line {
    buf: [u8; 256],
    len: usize,
}
impl Line {
    const fn new() -> Self {
        Self { buf: [0; 256], len: 0 }
    }
    fn clear(&mut self) {
        self.len = 0;
    }
    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}
impl core::fmt::Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            if self.len < self.buf.len() {
                self.buf[self.len] = b;
                self.len += 1;
            }
        }
        Ok(())
    }
}
async fn emit<'d>(c: &mut CdcAcmClass<'d, UsbDriver<'d, USB>>, l: &Line) {
    for chunk in l.as_bytes().chunks(64) {
        if c.write_packet(chunk).await.is_err() {
            return;
        }
    }
    if !l.as_bytes().is_empty() && l.as_bytes().len() % 64 == 0 {
        let _ = c.write_packet(&[]).await;
    }
}

/// NRZI-encode `data`'s bits into symbols, appending nothing — no EOP, no
/// release. The caller stitches the boundaries. Returns symbols pushed.
///
/// Mirrors `codec::encode`'s bit rule exactly; kept separate so the EOP between
/// the two packets can be controlled by hand.
fn nrzi(data: &[u8], out: &mut SymVec) {
    let mut k = true;
    let mut ones = 0u8;
    for &byte in data {
        for b in 0..8 {
            if byte & (1 << b) != 0 {
                out.push(if k { Symbol::K } else { Symbol::J });
                ones += 1;
            } else {
                k = !k;
                out.push(if k { Symbol::K } else { Symbol::J });
                ones = 0;
            }
            if ones == 6 {
                k = !k;
                out.push(if k { Symbol::K } else { Symbol::J });
                ones = 0;
            }
        }
    }
}

/// A tiny fixed-capacity symbol vector, packed four to a word for the DMA.
struct SymVec {
    words: [u32; 512],
    n: usize,
}
impl SymVec {
    fn new() -> Self {
        Self { words: [0; 512], n: 0 }
    }
    fn push(&mut self, s: Symbol) {
        if self.n < self.words.len() {
            self.words[self.n] = (s as u32) << 24;
            self.n += 1;
        }
    }
    fn words(&self) -> &[u32] {
        &self.words[..self.n]
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(bus::clock_config());

    let Pio {
        common: mut led_common,
        sm0: led_sm,
        ..
    } = Pio::new(p.PIO1, Irqs);
    let mut led = ws2812::Ws2812::new(&mut led_common, led_sm, p.PIN_16);
    led.set(smart_leds::RGB8::new(16, 16, 16));

    let driver = UsbDriver::new(p.USB, Irqs);
    let mut cfg = embassy_usb::Config::new(0xc0de, 0xca83);
    cfg.manufacturer = Some("TeslAux");
    cfg.product = Some("PIO USB pair");
    cfg.serial_number = Some("piopair");
    cfg.max_power = 100;
    cfg.max_packet_size_0 = 64;
    let mut d1 = [0u8; 256];
    let mut d2 = [0u8; 256];
    let mut d3 = [0u8; 64];
    let mut st = State::new();
    let mut b = Builder::new(driver, cfg, &mut d1, &mut d2, &mut [], &mut d3);
    let mut class = CdcAcmClass::new(&mut b, &mut st, 64);
    let mut usb = b.build();

    let work = async {
        class.wait_connection().await;
        Timer::after_millis(100).await;
        let mut line = Line::new();

        let Pio {
            mut common,
            mut sm0,
            mut sm1,
            mut sm2,
            ..
        } = Pio::new(p.PIO0, Irqs);
        let (dp_pin, dm_pin) = teslamic_rp2350::usb_pins!(p);
        let mut dp = common.make_pio_pin(dp_pin);
        let mut dm = common.make_pio_pin(dm_pin);
        bus::configure_pins(&mut dp, &mut dm);
        let order = board::pin_order();
        let programs = bus::load(&mut common, order);
        bus::init_tx(&mut sm0, &programs.tx, &dp, &dm);
        bus::init_edge_detector(&mut sm1, &programs.edge, &dp, &dm);
        bus::init_decoder(&mut sm2, &programs.decoder, &dp);
        bus::preload_decoder_osr(&mut sm2);

        // Two packets: a short one standing in for a token, and a data packet.
        let mut raw_a = [0u8; 8];
        let na = build_data(Pid::Data0, &[0x11, 0x22], &mut raw_a).unwrap();
        let mut raw_b = [0u8; 16];
        let nb = build_data(Pid::Data1, &[0xAA, 0xBB, 0xCC, 0xDD], &mut raw_b).unwrap();

        // Stitch: packet A bits, SE0 EOP, a couple of idle J, packet B bits,
        // SE0 EOP, release, pad to a word boundary.
        let mut v = SymVec::new();
        nrzi(&raw_a[..na], &mut v);
        v.push(Symbol::Se0);
        v.push(Symbol::Se0);
        v.push(Symbol::J);
        v.push(Symbol::J);
        let boundary = v.n; // where packet B begins, for reference
        nrzi(&raw_b[..nb], &mut v);
        v.push(Symbol::Se0);
        v.push(Symbol::Se0);
        v.push(Symbol::Release);
        while v.n % 4 != 0 {
            v.push(Symbol::K);
        }

        line.clear();
        let _ = write!(
            line,
            "\r\n=== back-to-back pair ===\r\npacket A {na}B, gap, packet B {nb}B\r\nA = {:02x?}\r\nB = {:02x?}\r\nB begins at symbol {boundary}\r\n",
            &raw_a[..na],
            &raw_b[..nb]
        );
        emit(&mut class, &line).await;

        // Fresh receiver, then fire the whole stream in one DMA and read the
        // decoder straight through — no realign anywhere.
        bus::prepare_decoder(&mut sm2, &programs.decoder);
        bus::restart_edge_detector(&mut sm1, &programs.edge);
        sm0.set_enable(true);
        sm1.set_enable(true);
        sm2.set_enable(true);

        let mut got = [0u8; 64];
        let mut n = 0usize;
        bus::tx_start(&mut sm0, &programs.tx, order);
        let feed = bus::dma_symbols(&mut sm0, p.DMA_CH0, v.words());
        let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_millis(20);
        while embassy_time::Instant::now() < deadline {
            while let Some(w) = sm2.rx().try_pull() {
                if n < got.len() {
                    got[n] = (w >> 24) as u8;
                }
                n += 1;
            }
        }
        feed.await;

        line.clear();
        let _ = write!(line, "decoded {n} bytes:\r\n  ");
        for byte in got.iter().take(n.min(48)) {
            let _ = write!(line, "{byte:02x} ");
        }
        let _ = write!(line, "\r\n");
        emit(&mut class, &line).await;

        // Find where packet A's SYNC and packet B's SYNC land in the output.
        let syncs: SyncPos = {
            let mut s = SyncPos::new();
            for (i, byte) in got.iter().take(n).enumerate() {
                if *byte == 0x80 {
                    s.push(i);
                }
            }
            s
        };
        line.clear();
        let _ = write!(line, "0x80 (SYNC) at byte offsets: ");
        for i in syncs.slice() {
            let _ = write!(line, "{i} ");
        }
        if syncs.len >= 2 {
            let _ = write!(line, " -> both packets aligned\r\n");
        } else {
            let _ = write!(line, " -> packet B did NOT align\r\n");
        }
        emit(&mut class, &line).await;

        line.clear();
        let _ = write!(line, "=== done ===\r\n");
        emit(&mut class, &line).await;
        loop {
            Timer::after_millis(1000).await;
        }
    };

    join(usb.run(), work).await;
}

struct SyncPos {
    v: [usize; 8],
    len: usize,
}
impl SyncPos {
    fn new() -> Self {
        Self { v: [0; 8], len: 0 }
    }
    fn push(&mut self, x: usize) {
        if self.len < self.v.len() {
            self.v[self.len] = x;
            self.len += 1;
        }
    }
    fn slice(&self) -> &[usize] {
        &self.v[..self.len]
    }
}

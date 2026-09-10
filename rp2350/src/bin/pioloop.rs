// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! Loopback self-test for the PIO USB line coding. No instruments needed.
//!
//! The transmitter drives D+/D-; the two receive state machines read those same
//! pads. Pointing them at each other on one chip exercises the whole chain —
//! symbol packing, the TX program, the edge detector, the NRZI decoder,
//! de-stuffing — and the board reports the answer on its own LED.
//!
//! * **Green** — every packet decoded back byte for byte. The line coding and
//!   all three PIO programs are right.
//! * **Red** — packets came back, but wrong. The decode path is broken; the
//!   first mismatching packet's length is blinked out in white afterwards.
//! * **Blue** — nothing arrived at all. The transmitter never drove the bus, or
//!   the edge detector never triggered. Suspect the input inversion, or a state
//!   machine that was never enabled.
//! * **Amber** — the system clock is not 96 MHz, so nothing else is meaningful.
//!
//! # What this cannot tell you
//!
//! TX and RX derive from the same `clk_sys`, so they are perfectly synchronous.
//! A real host runs on its own crystal, and recovering *that* clock is exactly
//! what the edge detector's resynchronisation exists for — which this test
//! never puts under any strain. Green here means the coding is right, not that
//! the timing margin is.
//!
//! It also says nothing about the bus turnaround, the 1.5 kΩ pull-up, or
//! anything electrical. Those need a real host on the other end of the cable.

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::Timer;
use smart_leds::RGB8;

use teslamic_rp2350::pio_usb::{bus, codec, packet};

#[path = "../../../rp2040/src/ws2812.rs"]
mod ws2812;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
});

/// Payloads to send. Chosen for what they do to the *coding*, not the content:
/// all-ones is the bit-stuffing worst case, all-zeroes transitions on every
/// bit, and 192 bytes is this project's actual iso IN endpoint size.
const CASES: [&[u8]; 5] = [
    &[],
    &[0x00, 0x01, 0x02, 0x03],
    &[0xFF; 16],
    &[0x00; 16],
    &[0x5A; 192],
];

/// Encoded symbols for the largest case, four to a byte, plus the EOP.
const SYM_BYTES: usize = 1024;
/// SYNC + PID + payload + CRC16 for the largest case.
const RAW_BYTES: usize = 200;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // The dividers in `bus` are computed against this. Anything else and the
    // bus runs at the wrong bit rate.
    let p = embassy_rp::init(bus::clock_config());

    // Status first, on PIO1: the USB programs fill PIO0 exactly, with no room
    // for a WS2812 driver beside them.
    let Pio {
        common: mut led_common,
        sm0: led_sm,
        ..
    } = Pio::new(p.PIO1, Irqs);
    let mut led = ws2812::Ws2812::new(&mut led_common, led_sm, p.PIN_16);

    if embassy_rp::clocks::clk_sys_freq() != teslamic_rp2350::pio_usb::SYSCLK_HZ {
        loop {
            led.set(RGB8::new(48, 24, 0)); // amber
            Timer::after_millis(500).await;
        }
    }

    let Pio {
        mut common,
        mut sm0,
        mut sm1,
        mut sm2,
        irq_flags,
        ..
    } = Pio::new(p.PIO0, Irqs);

    let mut dp = common.make_pio_pin(p.PIN_12);
    let mut dm = common.make_pio_pin(p.PIN_13);
    bus::configure_pins(&mut dp, &mut dm);

    let programs = bus::load(&mut common);
    programs.check_tx_offset();

    bus::init_tx(&mut sm0, &programs.tx, &dp, &dm);
    bus::init_edge_detector(&mut sm1, &programs.edge, &dp, &dm);
    bus::init_decoder(&mut sm2, &programs.decoder, &dp);

    // `in osr, 1` is how the decoder shifts in a 1 bit, so the OSR has to be
    // all ones before it starts. Left at zero every received byte comes back
    // wrong in a way that looks like a line fault.
    bus::preload_decoder_osr(&mut sm2);

    sm1.set_enable(true);
    sm2.set_enable(true);
    sm0.set_enable(true);

    let mut raw = [0u8; RAW_BYTES];
    let mut syms = [0u8; SYM_BYTES];
    let mut back = [0u8; RAW_BYTES];

    let mut failures = 0usize;
    let mut silent = 0usize;
    let mut first_bad_len = 0usize;

    for payload in CASES {
        let n = packet::build_data(packet::Pid::Data0, payload, &mut raw).unwrap();

        let mut buf = codec::SymbolBuf::new(&mut syms);
        let syms_len = codec::encode(&raw[..n], &mut buf).unwrap();
        let sym_bytes = syms_len / 4;

        bus::clear_eop_irq(&irq_flags);

        // Feed the transmitter and drain the decoder together: the RX FIFO is
        // four words deep and a 192-byte packet will overrun it many times over
        // if nothing is reading while it fills.
        let mut got = 0usize;
        let mut fed = 0usize;
        let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_millis(50);
        while embassy_time::Instant::now() < deadline {
            if fed < sym_bytes {
                let mut word = 0u32;
                let mut count = 0;
                while count < 4 && fed < sym_bytes {
                    word = (word << 8) | syms[fed] as u32;
                    fed += 1;
                    count += 1;
                }
                // A short final group still has to sit in the top bits, since
                // the state machine shifts left out of the MSB.
                word <<= 8 * (4 - count);
                while !sm0.tx().try_push(word) {}
            }
            while let Some(w) = sm2.rx().try_pull() {
                if got < back.len() {
                    // Right-shifting ISR with an 8-bit threshold leaves the
                    // byte in the top octet.
                    back[got] = (w >> 24) as u8;
                }
                got += 1;
            }
            if fed >= sym_bytes && got >= n {
                break;
            }
        }

        if got == 0 {
            silent += 1;
        } else if got < n || back[..n] != raw[..n] {
            if failures == 0 {
                first_bad_len = got;
            }
            failures += 1;
        }
    }

    loop {
        let colour = if silent == CASES.len() {
            RGB8::new(0, 0, 48) // blue: nothing came back at all
        } else if failures > 0 || silent > 0 {
            RGB8::new(48, 0, 0) // red: decoded, but wrong
        } else {
            RGB8::new(0, 48, 0) // green
        };
        led.set(colour);
        Timer::after_millis(700).await;

        // On a failure, blink out how many bytes did come back, so there is
        // something to go on beyond "red".
        if failures > 0 {
            led.set(RGB8::new(0, 0, 0));
            Timer::after_millis(400).await;
            for _ in 0..(first_bad_len.min(20)) {
                led.set(RGB8::new(32, 32, 32));
                Timer::after_millis(120).await;
                led.set(RGB8::new(0, 0, 0));
                Timer::after_millis(120).await;
            }
            Timer::after_millis(600).await;
        }
    }
}

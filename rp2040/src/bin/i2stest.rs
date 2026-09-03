// SPDX-License-Identifier: MIT
#![no_std]
#![no_main]

//! I2S bring-up: drive the link with embassy-rp's *upstream* I2S master.
//!
//! embassy-rp's `PioI2sIn`/`PioI2sOut` are controller-role only and expose no
//! runtime clock retuning, so they cannot serve either shipping design — both
//! need a slave on one side and a steerable clock on the other. But they are
//! maintained, tested code, which makes them exactly the right thing to test my
//! hand-written `slave_rx` against.
//!
//! Configured with `bit_depth = 32`, upstream emits BCK at 64x fs — the same
//! framing `slave_rx` expects and the same the PCM2706 produces. The 16-bit
//! sample sits in the top half of each 32-bit word, which is where `slave_rx`
//! samples.
//!
//! Flash this to one board and `teslamic-rp-car-elastic.uf2` to the other, wire
//! GPIO2/3/4 + GND between them, and listen on the car board's USB:
//!
//! * clean 997 Hz  -> the wiring and `slave_rx` are both correct
//! * distorted or channel-swapped -> `slave_rx` bit alignment is wrong
//! * silence -> no clock reaching the car board; check wiring and ground
//!
//! Any fault here is mine, not upstream's — which is the point of using it.
//!
//! Pins: GPIO2 DATA out, GPIO3 BCK out, GPIO4 LRCK out.

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::i2s::{PioI2sOut, PioI2sOutProgram};

#[path = "../ws2812.rs"]
mod ws2812;

const RATE: u32 = 48_000;
const BIT_DEPTH: u32 = 32;
/// One millisecond of stereo frames.
const BLOCK: usize = 48;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    // Common must outlive the peripheral, or embassy-rp resets the pins'
    // function-select to NULL when it drops. See src/status.rs.
    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
    let program = PioI2sOutProgram::new(&mut common);
    let mut i2s = PioI2sOut::new(
        &mut common, sm0, p.DMA_CH0, p.PIN_2, p.PIN_3, p.PIN_4, RATE, BIT_DEPTH, &program,
    );

    let mut buf = [0u32; BLOCK * 2];
    let mut phase: u32 = 0;
    let inc: u32 = ((997u64 << 32) / RATE as u64) as u32;
    loop {
        for f in 0..BLOCK {
            let idx = ((phase >> 24) & 0xFF) as usize;
            let frac = ((phase >> 16) & 0xFF) as i32;
            let a = sine(idx) as i32;
            let b = sine((idx + 1) & 0xFF) as i32;
            let v = (a + (((b - a) * frac) >> 8)) as i16;
            phase = phase.wrapping_add(inc);
            // 16-bit sample in the top half of the 32-bit word.
            let w = (v as u16 as u32) << 16;
            buf[f * 2] = w;
            buf[f * 2 + 1] = w;
        }
        i2s.write(&buf).await;
    }
}

/// Quarter-scale sine, computed rather than tabulated to keep this file short.
fn sine(i: usize) -> i16 {
    const T: [i16; 65] = [
        0, 393, 785, 1177, 1568, 1959, 2348, 2735, 3121, 3506, 3888, 4267, 4645, 5019, 5390, 5758,
        6123, 6484, 6841, 7194, 7542, 7886, 8226, 8560, 8889, 9213, 9531, 9844, 10150, 10451,
        10745, 11033, 11314, 11588, 11855, 12115, 12368, 12614, 12851, 13081, 13304, 13518, 13724,
        13921, 14111, 14292, 14464, 14627, 14782, 14928, 15065, 15192, 15311, 15420, 15521, 15611,
        15693, 15764, 15827, 15880, 15923, 15957, 15981, 15995, 16000,
    ];
    match i {
        0..=64 => T[i],
        65..=128 => T[128 - i],
        129..=192 => -T[i - 128],
        _ => -T[256 - i],
    }
}

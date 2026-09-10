// SPDX-License-Identifier: MIT
// `std` under `cfg(test)` so the codec's tests can run on the development
// machine — see `pio_usb::codec`. The firmware build is unaffected.
#![cfg_attr(not(test), no_std)]

//! Single-board TeslAux bridge on the RP2350.
//!
//! Run the host-side tests with:
//!
//! ```text
//! cargo test --lib --target aarch64-apple-darwin
//! ```
//!
//! The explicit target is needed because `.cargo/config.toml` pins the build to
//! thumbv8m for the firmware.
//!
//! Unlike the rp2040 crate, which shares code between its binaries with
//! `#[path]` includes, this one is a library: [`pio_usb`] is a device driver
//! with real internal structure, and it wants a module tree and unit tests
//! rather than to be textually pasted into each binary.

pub mod pio_usb;

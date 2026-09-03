// SPDX-License-Identifier: MIT
// Ported from wireless-performer-fw's osrf-driver-display-st7789, and
// relicensed MIT for this project by the copyright holder. The original in
// wireless-performer-fw remains AGPL-3.0-or-later; this copy does not.
#![allow(async_fn_in_trait, dead_code, unexpected_cfgs, unused_imports)]

//! Hand-rolled ST7789 driver tuned for the T114's 1.14″ 240×135
//! colour TFT.  Generic over SPI bus / output pins / delay — no
//! vendor HAL — so it works under any embassy port (or with a
//! Zephyr-equivalent wrapper) that provides the embedded-hal
//! traits.
//!
//! Geometry, MADCTL, and the power / gamma init sequence are
//! hardcoded for the T114 panel.  A different ST7789-based LCM
//! would need either: (a) override the [`PanelConfig`] consts, or
//! (b) fork this crate.  We took the simpler path until a second
//! panel actually shows up.
//!
//! ## Why not mipidsi
//!
//! `mipidsi::Builder::init` hangs on the T114 v2.0 hardware for
//! reasons not yet root-caused.  Hand-rolling the init sequence
//! sidesteps that entirely in ~150 lines of code with full control
//! over command timing.
//!
//! ## Architecture
//!
//! Two pieces:
//!
//!   - [`St7789`] — the controller driver.  Owns the SPI + four
//!     GPIO pins (`CS`, `DC`, `RESET`, `VTFT_CTRL`) + a delay
//!     provider.  All operations are async so the executor can
//!     yield during the long init delays and per-row pixel-data
//!     bursts.
//!
//!   - [`framebuffer::Framebuffer`] — in-RAM RGB565 buffer with
//!     dirty-rectangle tracking.  Implements
//!     `embedded_graphics_core::DrawTarget` so renderers paint
//!     into it synchronously.  Caller flushes the dirty region
//!     to the panel via [`St7789::flush`] (async).

pub mod framebuffer;
pub use framebuffer::{DirtyBox, Framebuffer, FB_H, FB_W};

use embedded_graphics_core::pixelcolor::raw::RawU16;
use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiBus;

/// Visible width of the panel in pixels (landscape).  Re-exported
/// from [`framebuffer`].
pub const WIDTH: u16 = FB_W;
/// Visible height of the panel in pixels (landscape).  Re-exported
/// from [`framebuffer`].
pub const HEIGHT: u16 = FB_H;

/// X offset into the ST7789's 240-column controller RAM where the
/// visible 1.14″ panel's first column lives, with our chosen
/// MADCTL / rotation.
const X_OFFSET: u16 = 40;
/// Y offset into the ST7789's 320-row controller RAM where the
/// visible 1.14″ panel's first row lives, with our chosen MADCTL /
/// rotation.
const Y_OFFSET: u16 = 53;

// ── ST7789 commands we emit ─────────────────────────────────────
const CMD_SWRESET: u8 = 0x01;
const CMD_SLPIN: u8 = 0x10;
const CMD_SLPOUT: u8 = 0x11;
const CMD_DISPOFF: u8 = 0x28;
const CMD_NORON: u8 = 0x13;
const CMD_INVON: u8 = 0x21;
const CMD_DISPON: u8 = 0x29;
const CMD_CASET: u8 = 0x2A;
const CMD_RASET: u8 = 0x2B;
const CMD_RAMWR: u8 = 0x2C;
const CMD_MADCTL: u8 = 0x36;
const CMD_COLMOD: u8 = 0x3A;

// Power / gamma commands.  Many ST7789 LCMs come from POR with
// safe defaults and don't need these; others ship with defaults
// that produce a black panel until VCOM / gamma are programmed
// explicitly.  Heltec sources LCMs from multiple suppliers within
// the T114 v2.1 SKU, so we send the Adafruit-compatible block
// unconditionally — harmless on the permissive panels, mandatory
// on the strict ones.  See ST7789V datasheet §6.2 for register
// definitions.
const CMD_PORCTRL: u8 = 0xB2;
const CMD_GCTRL: u8 = 0xB7;
const CMD_VCOMS: u8 = 0xBB;
const CMD_LCMCTRL: u8 = 0xC0;
const CMD_VDVVRHEN: u8 = 0xC2;
const CMD_VRHS: u8 = 0xC3;
const CMD_VDVS: u8 = 0xC4;
const CMD_FRCTRL2: u8 = 0xC6;
const CMD_PWCTRL1: u8 = 0xD0;
const CMD_PVGAMCTRL: u8 = 0xE0;
const CMD_NVGAMCTRL: u8 = 0xE1;

/// 1.14″ ST7789 colour TFT driver.
///
/// Owns the SPI bus + 4 GPIO outputs:
///   - `CS` — chip-select.
///   - `DC` — data/command select (low for command, high for data).
///   - `RST` — hardware reset.
///   - `PWR` — VTFT_CTRL panel-power gate, **active LOW** on the
///     T114 (drive LOW to enable LCM power, HIGH to power-down).
///
/// Plus a delay provider for the datasheet-mandated init delays
/// and a SLPIN settling pause.
///
/// Backlight is **not** owned by the display.  Backlight needs to
/// come on after the first clear-to-background to avoid showing
/// junk pixels; the profile keeps its own `OutputPin` handle to
/// the backlight GPIO and toggles it independently.
pub struct St7789<S, CS, DC, RST, PWR, D> {
    spi: S,
    cs: CS,
    dc: DC,
    reset: RST,
    vtft: PWR,
    delay: D,
}

impl<S, CS, DC, RST, PWR, D> St7789<S, CS, DC, RST, PWR, D>
where
    S: SpiBus<u8>,
    CS: OutputPin,
    DC: OutputPin,
    RST: OutputPin,
    PWR: OutputPin,
    D: DelayNs,
{
    /// Construct a new display.  Does **not** run the init sequence;
    /// call [`init`](Self::init) for that.  Separated so init can be
    /// async (uses `DelayNs` for datasheet-mandated delays) while
    /// the constructor stays sync — matters for boards that build
    /// their `Resources` outside an async context.
    pub fn new(spi: S, cs: CS, dc: DC, reset: RST, vtft: PWR, delay: D) -> Self {
        Self {
            spi,
            cs,
            dc,
            reset,
            vtft,
            delay,
        }
    }

    /// Run the ST7789 power-on sequence: rail warmup → hardware
    /// reset → SWRESET → SLPOUT → COLMOD (RGB565) → MADCTL (landscape,
    /// RGB order) → power/gamma block → INVON → NORON → DISPON.
    /// Datasheet-mandated delays observed via the provided `DelayNs`.
    /// Returns when the panel is ready for [`flush`](Self::flush) calls.
    pub async fn init(&mut self) {
        #[cfg(feature = "defmt")]
        defmt::info!("st7789: init begin");

        // Power up the LCM rail via VTFT_CTRL (active LOW).
        let _ = self.vtft.set_low();

        // Rail warmup — 1 s.  Meshtastic's reference uses the same.
        // The LCM's internal POR completes well before that; this
        // is the "works reliably across all units" floor.
        self.delay.delay_ms(1000).await;
        #[cfg(feature = "defmt")]
        defmt::info!("st7789: VTFT enabled, rail warm");

        // Hardware reset.  ST7789 datasheet says min 10 µs LOW, but
        // Meshtastic's working ST7789 driver uses 10 ms LOW for this
        // exact panel — many ST7789 revisions don't fully reset on a
        // sub-millisecond pulse.  HIGH 1 ms → LOW 10 ms → HIGH +
        // 120 ms post-reset.
        let _ = self.reset.set_high();
        self.delay.delay_ms(1).await;
        let _ = self.reset.set_low();
        self.delay.delay_ms(10).await;
        let _ = self.reset.set_high();
        self.delay.delay_ms(120).await;
        #[cfg(feature = "defmt")]
        defmt::info!("st7789: hw reset done");

        // Software reset (max 120 ms before any further command;
        // 150 ms for margin).
        self.write_command(CMD_SWRESET).await;
        self.delay.delay_ms(150).await;
        #[cfg(feature = "defmt")]
        defmt::info!("st7789: SWRESET done");

        // Sleep out — 120 ms before next command.
        self.write_command(CMD_SLPOUT).await;
        self.delay.delay_ms(120).await;
        #[cfg(feature = "defmt")]
        defmt::info!("st7789: SLPOUT done");

        // 16-bit/pixel RGB565 (0x55 = 65k colours over SPI).
        self.write_command_data(CMD_COLMOD, &[0x55]).await;
        self.delay.delay_ms(10).await;

        // MADCTL 0x60 = MX + MV, RGB order → landscape, origin at
        // upper-left of the visible window once X/Y_OFFSET applied.
        self.write_command_data(CMD_MADCTL, &[0x60]).await;

        // Adafruit-compatible power/gamma block — see the constant
        // table comment above.
        self.write_command_data(CMD_PORCTRL, &[0x0C, 0x0C, 0x00, 0x33, 0x33])
            .await;
        self.write_command_data(CMD_GCTRL, &[0x35]).await;
        self.write_command_data(CMD_VCOMS, &[0x19]).await;
        self.write_command_data(CMD_LCMCTRL, &[0x2C]).await;
        self.write_command_data(CMD_VDVVRHEN, &[0x01]).await;
        self.write_command_data(CMD_VRHS, &[0x12]).await;
        self.write_command_data(CMD_VDVS, &[0x20]).await;
        self.write_command_data(CMD_FRCTRL2, &[0x0F]).await;
        self.write_command_data(CMD_PWCTRL1, &[0xA4, 0xA1]).await;
        self.write_command_data(
            CMD_PVGAMCTRL,
            &[
                0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23,
            ],
        )
        .await;
        self.write_command_data(
            CMD_NVGAMCTRL,
            &[
                0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23,
            ],
        )
        .await;
        self.delay.delay_ms(10).await;
        #[cfg(feature = "defmt")]
        defmt::info!("st7789: power+gamma programmed");

        // Display inversion ON — required for ST7789 to render normal
        // (non-inverted) colours.  Backwards from earlier ST77xx parts.
        self.write_command(CMD_INVON).await;
        self.delay.delay_ms(10).await;

        // Normal display mode (vs partial / idle).
        self.write_command(CMD_NORON).await;
        self.delay.delay_ms(10).await;

        // Display ON — RAMWR will start showing pixels after this.
        self.write_command(CMD_DISPON).await;
        self.delay.delay_ms(10).await;

        #[cfg(feature = "defmt")]
        defmt::info!("st7789: init complete");
    }

    /// Diagnostic: send an arbitrary single-byte command with no
    /// data argument.  Public for SPI-viability smoke tests; not
    /// intended for normal use.
    pub async fn send_raw_command(&mut self, cmd: u8) {
        self.write_command(cmd).await;
    }

    /// Put the panel into its lowest-power state: `DISPOFF` +
    /// `SLPIN` (controller down to ~10 µA from ~1 mA normal-mode),
    /// then VTFT_CTRL HIGH so the panel's VDD rail goes off
    /// entirely (panel chip then draws 0).  Called from the deep
    /// soft-off path just before `sd_power_system_off`.
    ///
    /// Datasheet sequence requires SLPIN before cutting power; we
    /// wait 5 ms after SLPIN so the panel's charge-pumps settle
    /// before the rail goes away.
    pub async fn power_off(&mut self) {
        self.write_command(CMD_DISPOFF).await;
        self.write_command(CMD_SLPIN).await;
        self.delay.delay_ms(5).await;
        // Gate VTFT_CTRL HIGH = TFT VDD off.  Held until next init().
        let _ = self.vtft.set_high();
    }

    /// Push the dirty region of a [`Framebuffer`] to the panel,
    /// row by row, via async SPI.  Yields during each DMA burst
    /// so other tasks (notably the radio's RX loop) can run between
    /// bursts — without this, a sync render of ~30 ms of SPI work
    /// blocks `run_rx` long enough that the SX1262 RX FIFO overflows
    /// and 5–12 % of inbound packets get dropped.
    ///
    /// The dirty bounding box is cleared on completion.  `set_window`
    /// stays "fast" — its bursts are tiny (a handful of 1-5-byte
    /// commands, ~50 µs total) and not worth the extra plumbing.
    /// The big-data path — per-row pixel stream — is what matters
    /// and that's where we get the async-yield benefit.
    pub async fn flush(&mut self, fb: &mut Framebuffer) {
        let Some(b) = fb.dirty_box() else { return };
        self.set_window(b.x0, b.y0, b.x1, b.y1).await;
        let _ = self.dc.set_high();
        let _ = self.cs.set_low();

        let span = (b.x1 - b.x0 + 1) as usize;
        let mut row_buf = [0u8; FB_W as usize * 2];
        let pixels = fb.pixels();
        for y in b.y0..=b.y1 {
            let row_start = y as usize * FB_W as usize + b.x0 as usize;
            let row = &pixels[row_start..row_start + span];
            for (i, raw) in row.iter().enumerate() {
                row_buf[i * 2] = (raw >> 8) as u8;
                row_buf[i * 2 + 1] = (raw & 0xFF) as u8;
            }
            let _ = self.spi.write(&row_buf[..span * 2]).await;
        }

        let _ = self.cs.set_high();
        fb.clear_dirty();
    }

    // ── Low-level helpers ───────────────────────────────────────

    async fn write_command(&mut self, cmd: u8) {
        let _ = self.dc.set_low();
        let _ = self.cs.set_low();
        let _ = self.spi.write(&[cmd]).await;
        let _ = self.cs.set_high();
    }

    async fn write_command_data(&mut self, cmd: u8, data: &[u8]) {
        let _ = self.dc.set_low();
        let _ = self.cs.set_low();
        let _ = self.spi.write(&[cmd]).await;
        if !data.is_empty() {
            let _ = self.dc.set_high();
            let _ = self.spi.write(data).await;
        }
        let _ = self.cs.set_high();
    }

    /// Set the active drawing window via CASET / RASET and issue
    /// RAMWR.  Caller continues with the pixel-data stream.
    async fn set_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        let x0 = x0 + X_OFFSET;
        let x1 = x1 + X_OFFSET;
        let y0 = y0 + Y_OFFSET;
        let y1 = y1 + Y_OFFSET;
        self.write_command_data(
            CMD_CASET,
            &[
                (x0 >> 8) as u8,
                (x0 & 0xFF) as u8,
                (x1 >> 8) as u8,
                (x1 & 0xFF) as u8,
            ],
        )
        .await;
        self.write_command_data(
            CMD_RASET,
            &[
                (y0 >> 8) as u8,
                (y0 & 0xFF) as u8,
                (y1 >> 8) as u8,
                (y1 & 0xFF) as u8,
            ],
        )
        .await;
        self.write_command(CMD_RAMWR).await;
    }
}

// Silence unused-import warnings — `RawU16` is referenced by
// framebuffer::Framebuffer's DrawTarget impl, but only after that
// path is re-exported.  Touching it here keeps the import alive
// if the framebuffer module ever changes.
const _: fn() = || {
    let _ = RawU16::from(Rgb565::new(0, 0, 0));
};

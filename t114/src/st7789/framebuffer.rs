// SPDX-License-Identifier: AGPL-3.0-or-later

//! In-RAM framebuffer for the T114's 240×135 ST7789 TFT.
//!
//! Implements [`DrawTarget`] so the renderer (or any embedded-
//! graphics-compatible code) can paint into it synchronously.
//! Pixels accumulate in `pixels`; the affected rectangle is
//! tracked in `dirty`.  After a render completes, call
//! [`crate::St7789::flush`] (async) to stream the dirty region
//! to the panel — that path uses async SPI and yields during
//! each DMA burst so other tasks (notably the radio's RX loop)
//! can run between bursts.
//!
//! ## Why a framebuffer instead of writing direct
//!
//! `embedded_graphics::DrawTarget` is a sync trait — its methods
//! can't `.await`.  The only way to keep them sync while also
//! letting the SPI bursts yield is to break the work in two:
//! sync write into RAM here, async push to SPI there.  64 KB of
//! BSS is the cost of admission; the nRF52840 has enough headroom
//! for it (~240 KB available after SoftDevice reservations).

use core::convert::Infallible;
use embedded_graphics_core::pixelcolor::raw::RawU16;
use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::prelude::*;
use embedded_graphics_core::primitives::Rectangle;
use embedded_graphics_core::Pixel;

/// Panel width in pixels.
pub const FB_W: u16 = 240;
/// Panel height in pixels.
pub const FB_H: u16 = 135;

const FB_PIXELS: usize = FB_W as usize * FB_H as usize;

/// In-RAM RGB565 framebuffer + dirty-box tracker.  Sized for the
/// T114's 240×135 panel (~64 KB).  `const fn new()` so callers can
/// place an instance in `static` rather than blowing the task
/// stack with 64 KB.
pub struct Framebuffer {
    pixels: [u16; FB_PIXELS],
    dirty: Option<DirtyBox>,
}

/// Inclusive dirty bounding box.  `(x0,y0)..(x1,y1)`.
#[derive(Clone, Copy, Debug)]
pub struct DirtyBox {
    pub x0: u16,
    pub y0: u16,
    pub x1: u16,
    pub y1: u16,
}

impl Framebuffer {
    /// Construct an all-black framebuffer with no dirty region.
    /// `const fn` so callers can place it in `static`.
    pub const fn new() -> Self {
        Self {
            pixels: [0; FB_PIXELS],
            dirty: None,
        }
    }

    /// Read-only view of the packed RGB565 pixel array, row-major
    /// from `(0,0)` to `(FB_W-1, FB_H-1)`.  Used by
    /// [`crate::St7789::flush`] to stream out via SPI.
    pub fn pixels(&self) -> &[u16; FB_PIXELS] {
        &self.pixels
    }

    /// Current dirty bounding box (inclusive), or `None` if nothing
    /// has been drawn since the last [`clear_dirty`].
    pub fn dirty_box(&self) -> Option<DirtyBox> {
        self.dirty
    }

    /// Mark the framebuffer as fully clean (caller has flushed
    /// the dirty region to the panel).
    pub fn clear_dirty(&mut self) {
        self.dirty = None;
    }

    fn mark_dirty(&mut self, x: u16, y: u16) {
        self.dirty = Some(match self.dirty {
            None => DirtyBox {
                x0: x,
                y0: y,
                x1: x,
                y1: y,
            },
            Some(b) => DirtyBox {
                x0: b.x0.min(x),
                y0: b.y0.min(y),
                x1: b.x1.max(x),
                y1: b.y1.max(y),
            },
        });
    }

    fn put_pixel(&mut self, x: i32, y: i32, color: u16) {
        if x < 0 || y < 0 {
            return;
        }
        let xu = x as u16;
        let yu = y as u16;
        if xu >= FB_W || yu >= FB_H {
            return;
        }
        self.pixels[yu as usize * FB_W as usize + xu as usize] = color;
        self.mark_dirty(xu, yu);
    }
}

impl Default for Framebuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl Dimensions for Framebuffer {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::zero(), Size::new(FB_W as u32, FB_H as u32))
    }
}

impl DrawTarget for Framebuffer {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            let raw = RawU16::from(color).into_inner();
            self.put_pixel(point.x, point.y, raw);
        }
        Ok(())
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let mut iter = colors.into_iter();
        let x0 = area.top_left.x;
        let y0 = area.top_left.y;
        let x1 = x0 + area.size.width as i32;
        let y1 = y0 + area.size.height as i32;
        for y in y0..y1 {
            for x in x0..x1 {
                match iter.next() {
                    Some(c) => self.put_pixel(x, y, RawU16::from(c).into_inner()),
                    None => return Ok(()),
                }
            }
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let raw = RawU16::from(color).into_inner();
        let x0 = area.top_left.x;
        let y0 = area.top_left.y;
        let x1 = x0 + area.size.width as i32;
        let y1 = y0 + area.size.height as i32;
        for y in y0..y1 {
            for x in x0..x1 {
                self.put_pixel(x, y, raw);
            }
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let raw = RawU16::from(color).into_inner();
        for p in self.pixels.iter_mut() {
            *p = raw;
        }
        self.dirty = Some(DirtyBox {
            x0: 0,
            y0: 0,
            x1: FB_W - 1,
            y1: FB_H - 1,
        });
        Ok(())
    }
}

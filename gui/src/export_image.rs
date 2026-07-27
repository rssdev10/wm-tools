//! PNG image export for the oscilloscope capture viewer.
//!
//! Generates high-quality anti-aliased PNG images via `tiny-skia` for vector
//! paths (grid, traces) and renders text either with a built-in 3×5 bitmap
//! font or via system monospace fonts (`ab_glyph` + `font-kit`).

use std::path::Path;

use dso_parser::{ADC_VALUE_MID, Capture};
use crate::canvas::format_duration_ms_scale;
use crate::canvas::{GRID_HEIGHT_PX, PX_PER_DIV, T_CELLS, V_CELLS};
use crate::settings::{Settings, ViewMode};

// ── Colour palette (mirrors canvas.rs constants) ───────────────────────────

const PNG_BG: [u8; 3] = [10, 15, 26];
const PNG_GRID: [u8; 3] = [51, 61, 77];
const PNG_CH1: [u8; 3] = [255, 217, 25];
const PNG_CH2: [u8; 3] = [242, 25, 242];
const PNG_SCALE_TEXT: [u8; 3] = [153, 166, 179];

// ── Configuration ──────────────────────────────────────────────────────────

/// Configuration for PNG export resolution and rendering quality.
pub struct PngExportConfig {
    /// Target graph area width in pixels. 0 = default small (600 px).
    pub target_width: u32,
}

impl PngExportConfig {
    fn scale(&self) -> f64 {
        const BASE: u32 = T_CELLS as u32 * 50;
        if self.target_width > 0 {
            self.target_width as f64 / BASE as f64
        } else {
            1.0
        }
    }

    fn graph_w(&self) -> u32 {
        const BASE: u32 = T_CELLS as u32 * 50;
        (BASE as f64 * self.scale()).round() as u32
    }

    fn graph_h(&self) -> u32 {
        const BASE: u32 = V_CELLS as u32 * 50;
        (BASE as f64 * self.scale()).round() as u32
    }

    fn scale_margin_left(&self, show: bool) -> u32 {
        if show {
            ((65.0 * self.scale()).round() as u32).max(52)
        } else {
            0
        }
    }

    fn scale_margin_bottom(&self, show: bool) -> u32 {
        if show { (16.0 * self.scale()).round() as u32 } else { 0 }
    }

    fn grid_width(&self) -> f32 {
        (0.75 * self.scale().sqrt() as f32).clamp(0.75, 2.0)
    }

    /// Full line width for the trace in pixels.
    fn trace_width(&self) -> f32 {
        (1.25 * self.scale().sqrt() as f32).clamp(1.0, 4.0)
    }

    fn use_system_font(&self) -> bool {
        self.target_width > 0
    }

    fn font_size(&self) -> f32 {
        (11.0 * self.scale() as f32).max(11.0)
    }
}

// ── Main entry point ───────────────────────────────────────────────────────

/// Export the current capture as a PNG image.
pub fn export_png(path: &Path, cap: &Capture, settings: &Settings) -> anyhow::Result<()> {
    let config = PngExportConfig { target_width: settings.png_export_width };

    let graph_w = config.graph_w();
    let graph_h = config.graph_h();
    let scale_margin_left = config.scale_margin_left(settings.show_scales);
    let scale_margin_bottom = config.scale_margin_bottom(settings.show_scales);

    let has_ch2 = cap.ch2.is_some();
    let do_split = settings.split_png && settings.show_ch1 && settings.show_ch2 && has_ch2;

    let width = graph_w + scale_margin_left;
    let height = if do_split {
        (graph_h + scale_margin_bottom) * 2
    } else {
        graph_h + scale_margin_bottom
    };

    // RGBA pixel buffer for tiny-skia.
    let mut pixels = vec![0u8; (width * height * 4) as usize];
    for chunk in pixels.chunks_exact_mut(4) {
        chunk.copy_from_slice(&[PNG_BG[0], PNG_BG[1], PNG_BG[2], 255]);
    }

    // Disable anti-aliasing at very low resolution to keep a crisp look.
    let antialias = graph_w >= 400;
    // Tie system-font choice to antialias: use the aliased bitmap glyph when
    // anti-aliasing is off, so text stays crisp and fits the narrow margins.
    let use_system_font = config.use_system_font() && antialias;
    let font_size = config.font_size();
    let trace_w = config.trace_width();
    // When anti-aliasing is off, sub-pixel strokes may not render; ensure a
    // minimum visible width.
    let grid_w = if antialias { config.grid_width() } else { config.grid_width().max(1.0) };
    let smooth = settings.view_mode == ViewMode::Smooth;
    let dot_mode = settings.view_mode == ViewMode::Dot;
    let dot_radius = (trace_w * 0.6).max(1.0);

    // ── Grid helper via tiny-skia ────────────────────────────────────
    let draw_grid_on = |pixels: &mut [u8], y_off: u32| {
        let Some(grid_path) = build_grid_path(scale_margin_left, y_off, graph_w, graph_h) else {
            return;
        };
        let Some(mut pixmap) = tiny_skia::PixmapMut::from_bytes(pixels, width, height) else {
            return;
        };
        let mut paint = tiny_skia::Paint::default();
        paint.set_color(rgb_color(PNG_GRID));
        paint.anti_alias = antialias;
        let stroke = tiny_skia::Stroke {
            width: grid_w,
            ..tiny_skia::Stroke::default()
        };
        pixmap.stroke_path(&grid_path, &paint, &stroke, tiny_skia::Transform::identity(), None);
    };

    // ── Scale labels ─────────────────────────────────────────────────
    let t_per_div_ms = settings.t_per_div_ms;
    let draw_scales_on_region =
        |pixels: &mut [u8], x_off: u32, y_off: u32, w: u32, h: u32, _n_samples: usize,
         vpd_ch1: f64, vo_ch1: f64, ch1_color: Option<[u8; 3]>,
         vpd_ch2: f64, vo_ch2: f64, ch2_color: Option<[u8; 3]>| {
            let cols = T_CELLS as u32;
            let total_time_ms = T_CELLS as f64 * t_per_div_ms;
            for i in 0..cols {
                let frac = i as f64 / cols as f64;
                let time_ms = frac * total_time_ms;
                let label = format_duration_ms_scale(time_ms);
                let x_pos = x_off + i * w / cols + 2;
                let y_pos = y_off + h + 2;
                draw_text_tiny(pixels, width, &label, x_pos, y_pos, PNG_SCALE_TEXT, use_system_font, font_size);
            }

            // Y axis: voltage labels on left margin, per-channel.
            let rows = V_CELLS as u32;
            let mut draw_voltage_label = |vpd: f64, vo: f64, color: [u8; 3], x_lbl: u32| {
                for i in 0..=rows {
                    let v = ((i + 1) as f64 / rows as f64) * GRID_HEIGHT_PX;
                    let voltage = vo + (ADC_VALUE_MID as f64 - v) * vpd / PX_PER_DIV;
                    let y_pos = y_off + (i * h / rows);
                    draw_text_tiny(pixels, width, &format!("{voltage:.1}"), x_lbl, y_pos, color, use_system_font, font_size);
                }
            };

            let x_ch1 = 2u32;
            let x_ch2 = if ch1_color.is_some() { (scale_margin_left as f64 * 0.4).round() as u32 } else { 2u32 };
            if let Some(c) = ch1_color { draw_voltage_label(vpd_ch1, vo_ch1, c, x_ch1); }
            if let Some(c) = ch2_color { draw_voltage_label(vpd_ch2, vo_ch2, c, x_ch2); }
        };

    // ── Draw one subimage region ─────────────────────────────────────
    let draw_one = |pixels: &mut [u8], samples: &[u8], trace_color: [u8; 3],
                    y_off: u32, vpd: f64, vo: f64, ch_col: Option<[u8; 3]>| {
        draw_grid_on(pixels, y_off);
        if !samples.is_empty() {
            if dot_mode {
                draw_dots(pixels, width, height, samples, trace_color,
                          scale_margin_left, y_off, graph_w, graph_h, dot_radius, antialias);
            } else {
                draw_trace(pixels, width, height, samples, trace_color,
                           scale_margin_left, y_off, graph_w, graph_h, trace_w, smooth, antialias);
            }
        }
        if let Some(c) = ch_col {
            draw_scales_on_region(
                pixels, scale_margin_left, y_off, graph_w, graph_h, samples.len(),
                vpd, vo, Some(c), 0.0, 0.0, None,
            );
        }
    };

    if do_split {
        let ch1_col = settings.show_scales.then_some(PNG_CH1);
        draw_one(&mut pixels, &cap.ch1, PNG_CH1, 0,
                 settings.v_per_div_ch1, settings.v_offset_ch1, ch1_col);

        let ch2_col = settings.show_scales.then_some(PNG_CH2);
        if let Some(ch2) = &cap.ch2 {
            draw_one(&mut pixels, ch2, PNG_CH2, graph_h + scale_margin_bottom,
                     settings.v_per_div_ch2, settings.v_offset_ch2, ch2_col);
        }
    } else {
        draw_grid_on(&mut pixels, 0);
        if settings.show_ch1 {
            if dot_mode {
                draw_dots(&mut pixels, width, height, &cap.ch1, PNG_CH1,
                          scale_margin_left, 0, graph_w, graph_h, dot_radius, antialias);
            } else {
                draw_trace(&mut pixels, width, height, &cap.ch1, PNG_CH1,
                           scale_margin_left, 0, graph_w, graph_h, trace_w, smooth, antialias);
            }
        }
        if settings.show_ch2 {
            if let Some(ch2) = &cap.ch2 {
                if dot_mode {
                    draw_dots(&mut pixels, width, height, ch2, PNG_CH2,
                              scale_margin_left, 0, graph_w, graph_h, dot_radius, antialias);
                } else {
                    draw_trace(&mut pixels, width, height, ch2, PNG_CH2,
                               scale_margin_left, 0, graph_w, graph_h, trace_w, smooth, antialias);
                }
            }
        }
        if settings.show_scales {
            let ch1_col = if settings.show_ch1 { Some(PNG_CH1) } else { None };
            let ch2_col = if settings.show_ch2 && has_ch2 { Some(PNG_CH2) } else { None };
            draw_scales_on_region(
                &mut pixels, scale_margin_left, 0, graph_w, graph_h, cap.ch1.len(),
                settings.v_per_div_ch1, settings.v_offset_ch1, ch1_col,
                settings.v_per_div_ch2, settings.v_offset_ch2, ch2_col,
            );
        }
    }

    write_png(path, width, height, &pixels)?;
    Ok(())
}

// ── tiny-skia helper functions ─────────────────────────────────────────────

fn rgb_color(rgb: [u8; 3]) -> tiny_skia::Color {
    tiny_skia::Color::from_rgba8(rgb[0], rgb[1], rgb[2], 255)
}

#[inline]
fn waveform_y(sample: f64, height: u32) -> f32 {
    let y_scale = (height - 1) as f64 / GRID_HEIGHT_PX;
    ((sample - PX_PER_DIV) * y_scale) as f32
}

#[inline]
fn is_visible_y(y: f32, height: u32) -> bool {
    (0.0..=(height - 1) as f32).contains(&y)
}

#[inline]
fn border_y(y: f32, height: u32) -> f32 {
    if y < 0.0 {
        0.0
    } else {
        (height - 1) as f32
    }
}

/// Resample waveform samples into unclamped `(x, y)` points for the output
/// graph region. Keeping Y unclamped allows each renderer to distinguish real
/// border values from samples that are outside the visible voltage range.
///
/// Upscaling uses linear interpolation. Downscaling preserves the min/max
/// envelope per output column.
fn resample_waveform(
    samples: &[u8],
    output_width: u32,
    output_height: u32,
) -> Vec<(f32, f32)> {
    let n = samples.len();
    if n == 0 || output_width == 0 || output_height == 0 {
        return Vec::new();
    }

    // Keep the unclamped Y coordinate here. draw_trace() needs to know whether
    // each resampled point is genuinely inside or outside the visible mesh.
    let map_y = |sample: f64| waveform_y(sample, output_height);

    // Upscaling: linear interpolation between source samples.
    if output_width as usize >= n {
        let mut points = Vec::with_capacity(output_width as usize);
        for x in 0..output_width {
            let source_pos = x as f64 * (n - 1) as f64 / (output_width - 1).max(1) as f64;
            let left = source_pos.floor() as usize;
            let right = (left + 1).min(n - 1);
            let t = source_pos - left as f64;
            let sample = samples[left] as f64 * (1.0 - t) + samples[right] as f64 * t;
            points.push((x as f32, map_y(sample)));
        }
        return points;
    }

    // Downscaling: min/max envelope per column.
    let mut points = Vec::with_capacity(output_width as usize * 2);
    for x in 0..output_width {
        let start = x as usize * n / output_width as usize;
        let end = ((x + 1) as usize * n / output_width as usize)
            .max(start + 1)
            .min(n);
        let col = &samples[start..end];
        let min = *col.iter().min().unwrap() as f64;
        let max = *col.iter().max().unwrap() as f64;
        let y_min = map_y(min);
        let y_max = map_y(max);

        // Preserve temporal direction.
        if col.first() <= col.last() {
            points.push((x as f32, y_min));
            points.push((x as f32, y_max));
        } else {
            points.push((x as f32, y_max));
            points.push((x as f32, y_min));
        }
    }
    points
}

/// Build an anti-aliased grid path via tiny-skia.
fn build_grid_path(
    x_off: u32,
    y_off: u32,
    width: u32,
    height: u32,
) -> Option<tiny_skia::Path> {
    let mut path = tiny_skia::PathBuilder::new();
    for col in 1..T_CELLS as u32 {
        let x = x_off as f32 + col as f32 * width as f32 / T_CELLS as f32;
        path.move_to(x, y_off as f32);
        path.line_to(x, (y_off + height) as f32);
    }
    for row in 1..V_CELLS as u32 {
        let y = y_off as f32 + row as f32 * height as f32 / V_CELLS as f32;
        path.move_to(x_off as f32, y);
        path.line_to((x_off + width) as f32, y);
    }
    path.finish()
}

/// Append a smooth Catmull–Rom curve segment (converted to cubic Bézier)
/// for each consecutive pair of points.
fn append_catmull_rom_path(
    path: &mut tiny_skia::PathBuilder,
    points: &[(f32, f32)],
    x_off: u32,
    y_off: u32,
) {
    let ox = x_off as f32;
    let oy = y_off as f32;
    for i in 0..points.len() - 1 {
        let p0 = if i == 0 { points[i] } else { points[i - 1] };
        let p1 = points[i];
        let p2 = points[i + 1];
        let p3 = if i + 2 < points.len() { points[i + 2] } else { p2 };
        // Catmull-Rom → cubic Bézier with tension 0.5.
        let c1 = (p1.0 + (p2.0 - p0.0) / 6.0, p1.1 + (p2.1 - p0.1) / 6.0);
        let c2 = (p2.0 - (p3.0 - p1.0) / 6.0, p2.1 - (p3.1 - p1.1) / 6.0);
        path.cubic_to(ox + c1.0, oy + c1.1, ox + c2.0, oy + c2.1, ox + p2.0, oy + p2.1);
    }
}

/// Draw an anti-aliased trace via tiny-skia.
#[allow(clippy::too_many_arguments)]
fn draw_trace(
    pixels: &mut [u8],
    image_width: u32,
    image_height: u32,
    samples: &[u8],
    color: [u8; 3],
    x_off: u32,
    y_off: u32,
    width: u32,
    height: u32,
    line_width: f32,
    smooth: bool,
    antialias: bool,
) {
    if samples.len() < 2 || width < 2 || height < 2 {
        return;
    }
    let Some(mut pixmap) = tiny_skia::PixmapMut::from_bytes(pixels, image_width, image_height) else {
        return;
    };
    let points = resample_waveform(samples, width, height);
    if points.len() < 2 {
        return;
    }

    // An outside point is retained only when it is adjacent to an inside
    // point. In that case it is moved onto the corresponding top/bottom mesh
    // border. Outside runs therefore remain hidden, while the visible line or
    // smooth curve still reaches the border before disappearing/reappearing.
    let clipped_points = points.iter().enumerate().map(|(i, &(x, y))| {
        if is_visible_y(y, height) {
            return Some((x, y));
        }

        let previous_inside = i > 0 && is_visible_y(points[i - 1].1, height);
        let next_inside = i + 1 < points.len()
            && is_visible_y(points[i + 1].1, height);

        (previous_inside || next_inside).then(|| (x, border_y(y, height)))
    });

    let mut paint = tiny_skia::Paint::default();
    paint.set_color(rgb_color(color));
    paint.anti_alias = antialias;
    let stroke = tiny_skia::Stroke {
        width: line_width,
        line_cap: tiny_skia::LineCap::Round,
        line_join: tiny_skia::LineJoin::Round,
        ..tiny_skia::Stroke::default()
    };

    let draw_run = |pixmap: &mut tiny_skia::PixmapMut<'_>, run: &[(f32, f32)]| {
        if run.len() < 2 {
            return;
        }

        let mut path_builder = tiny_skia::PathBuilder::new();
        let first = run[0];
        path_builder.move_to(x_off as f32 + first.0, y_off as f32 + first.1);

        if smooth && run.len() >= 4 {
            append_catmull_rom_path(&mut path_builder, run, x_off, y_off);
        } else {
            for &(x, y) in &run[1..] {
                path_builder.line_to(x_off as f32 + x, y_off as f32 + y);
            }
        }

        if let Some(path) = path_builder.finish() {
            pixmap.stroke_path(
                &path,
                &paint,
                &stroke,
                tiny_skia::Transform::identity(),
                None,
            );
        }
    };

    let mut run = Vec::new();
    for point in clipped_points {
        match point {
            Some(point) => run.push(point),
            None => {
                draw_run(&mut pixmap, &run);
                run.clear();
            }
        }
    }
    draw_run(&mut pixmap, &run);
}

/// Draw dots via tiny-skia (Dot mode). Each visible sample is rendered as a
/// small filled circle. Samples outside the vertical mesh range are skipped;
/// unlike line/smooth modes, dot mode never creates synthetic border points.
///
/// When anti-aliasing is disabled, dots are drawn as individual pixels
/// directly into the RGBA buffer — no scaling, no smoothing, no sub-pixel
/// artefacts — to keep the dot series crisp at low resolution.
#[allow(clippy::too_many_arguments)]
fn draw_dots(
    pixels: &mut [u8],
    image_width: u32,
    image_height: u32,
    samples: &[u8],
    color: [u8; 3],
    x_off: u32,
    y_off: u32,
    width: u32,
    height: u32,
    dot_radius: f32,
    antialias: bool,
) {
    let n = samples.len();
    if n == 0 || width < 2 || height < 2 {
        return;
    }

    let step = n.div_ceil(width as usize).max(1);
    let sample_points = (0..n).step_by(step).filter_map(|i| {
        let y = waveform_y(samples[i] as f64, height);
        is_visible_y(y, height).then(|| {
            let x = i as f64 / (n - 1).max(1) as f64 * width as f64;
            (x as f32, y)
        })
    });

    if !antialias {
        for (x, y) in sample_points {
            let px = x_off + x.round() as u32;
            let py = y_off + y.round() as u32;
            let idx = (py as usize * image_width as usize + px as usize) * 4;
            if let Some(pixel) = pixels.get_mut(idx..idx + 4) {
                pixel[..3].copy_from_slice(&color);
                pixel[3] = 255;
            }
        }
        return;
    }

    let Some(mut pixmap) = tiny_skia::PixmapMut::from_bytes(
        pixels,
        image_width,
        image_height,
    ) else {
        return;
    };
    let mut paint = tiny_skia::Paint::default();
    paint.set_color(rgb_color(color));
    paint.anti_alias = true;

    for (x, y) in sample_points {
        let Some(circle) = tiny_skia::PathBuilder::from_circle(
            x_off as f32 + x,
            y_off as f32 + y,
            dot_radius,
        ) else {
            continue;
        };
        pixmap.fill_path(
            &circle,
            &paint,
            tiny_skia::FillRule::Winding,
            tiny_skia::Transform::identity(),
            None,
        );
    }
}

// ── Text rendering ─────────────────────────────────────────────────────────

/// Draw text onto a raw RGBA pixel buffer.
///
/// Uses the built-in 3×5 bitmap font when `force_system_font` is false,
/// otherwise renders with the system monospace font via `ab_glyph` at the
/// given `font_size`.
#[allow(clippy::too_many_arguments)]
fn draw_text_tiny(
    pixels: &mut [u8],
    img_width: u32,
    text: &str,
    x: u32,
    y: u32,
    color: [u8; 3],
    force_system_font: bool,
    font_size: f32,
) {
    if force_system_font {
        draw_system_font(pixels, img_width, text, x, y, color, font_size);
    } else {
        draw_tiny_glyph(pixels, img_width, text, x, y, color);
    }
}

/// Render text with the built-in 3×5 bitmap font.  Each font pixel maps to
/// exactly one output pixel — no scaling, no anti-aliasing.
fn draw_tiny_glyph(pixels: &mut [u8], img_width: u32, text: &str, x: u32, y: u32, color: [u8; 3]) {
    let mut cx = x;
    for ch in text.chars() {
        let glyph = tiny_glyph(ch);
        for row in 0..5u32 {
            for col in 0..3u32 {
                if glyph[row as usize] & (1 << (2 - col)) != 0 {
                    let px = cx + col;
                    let py = y + row;
                    let idx = ((py * img_width + px) * 4) as usize;
                    if idx + 3 < pixels.len() {
                        pixels[idx..idx + 3].copy_from_slice(&color);
                        pixels[idx + 3] = 255;
                    }
                }
            }
        }
        cx += 4;
    }
}

/// Render text using a system monospace font via `ab_glyph`.
/// Render text into an RGB pixel buffer using the operating system's
/// default monospace font family.
#[allow(clippy::too_many_arguments)]
fn draw_system_font(
    pixels: &mut [u8],
    img_width: u32,
    text: &str,
    x: u32,
    y: u32,
    color: [u8; 3],
    font_size: f32,
) {
    use ab_glyph::{Font, FontArc, PxScale, ScaleFont};
    use font_kit::{
        family_name::FamilyName,
        properties::Properties,
        source::SystemSource,
    };
    use std::sync::OnceLock;

    /// Resolve the default system monospace family and convert it to an
    /// `ab_glyph` font.
    fn load_system_monospace() -> Option<FontArc> {
        let handle = SystemSource::new()
            .select_best_match(
                &[FamilyName::Monospace],
                &Properties::new(),
            )
            .ok()?;

        // Extract font data and index from the handle, then load via ab_glyph.
        let (font_data, font_index) = match &handle {
            font_kit::handle::Handle::Memory { bytes, font_index } => {
                ((**bytes).clone(), *font_index)
            }
            font_kit::handle::Handle::Path { path, font_index } => {
                (std::fs::read(path).ok()?, *font_index)
            }
        };

        let font = ab_glyph::FontVec::try_from_vec_and_index(font_data, font_index).ok()?;
        Some(FontArc::new(font))
    }

    static FONT: OnceLock<Option<FontArc>> = OnceLock::new();

    let Some(font) = FONT.get_or_init(load_system_monospace).as_ref() else {
        return;
    };

    let scaled_font = font.as_scaled(PxScale::from(font_size));
    let baseline = y as f32 + scaled_font.ascent();

    let mut cursor_x = x as f32;

    for ch in text.chars() {
        let glyph_id = font.glyph_id(ch);

        let glyph = glyph_id.with_scale_and_position(
            font_size,
            ab_glyph::point(cursor_x, baseline),
        );

        if let Some(outlined) = font.outline_glyph(glyph) {
            let bounds = outlined.px_bounds();

            outlined.draw(|local_x, local_y, coverage| {
                let pixel_x = bounds.min.x.floor() as i32 + local_x as i32;
                let pixel_y = bounds.min.y.floor() as i32 + local_y as i32;

                if pixel_x < 0 || pixel_y < 0 {
                    return;
                }

                let pixel_x = pixel_x as u32;
                let pixel_y = pixel_y as u32;

                if pixel_x >= img_width {
                    return;
                }

                let index =
                    (pixel_y as usize * img_width as usize + pixel_x as usize) * 4;

                let Some(destination) = pixels.get_mut(index..index + 4) else {
                    return;
                };

                blend_rgb(destination, color, coverage);
            });
        }

        cursor_x += scaled_font.h_advance(glyph_id);
    }
}

/// Alpha-blend one RGB color over an RGBA destination pixel.
fn blend_rgb(destination: &mut [u8], foreground: [u8; 3], alpha: f32) {
    let alpha = alpha.clamp(0.0, 1.0);
    let inverse_alpha = 1.0 - alpha;

    for channel in 0..3 {
        destination[channel] = (
            foreground[channel] as f32 * alpha
                + destination[channel] as f32 * inverse_alpha
        )
            .round()
            .clamp(0.0, 255.0) as u8;
    }
    destination[3] = 255;
}

/// Returns a 5-row bitmask (3 bits wide) for a character.
fn tiny_glyph(ch: char) -> [u8; 5] {
    match ch {
        '0' => [0b111, 0b101, 0b101, 0b101, 0b111],
        '1' => [0b010, 0b110, 0b010, 0b010, 0b111],
        '2' => [0b111, 0b001, 0b111, 0b100, 0b111],
        '3' => [0b111, 0b001, 0b111, 0b001, 0b111],
        '4' => [0b101, 0b101, 0b111, 0b001, 0b001],
        '5' => [0b111, 0b100, 0b111, 0b001, 0b111],
        '6' => [0b111, 0b100, 0b111, 0b101, 0b111],
        '7' => [0b111, 0b001, 0b001, 0b001, 0b001],
        '8' => [0b111, 0b101, 0b111, 0b101, 0b111],
        '9' => [0b111, 0b101, 0b111, 0b001, 0b111],
        '.' => [0b000, 0b000, 0b000, 0b000, 0b010],
        '-' => [0b000, 0b000, 0b111, 0b000, 0b000],
        'V' => [0b101, 0b101, 0b101, 0b101, 0b010],
        'm' => [0b101, 0b111, 0b101, 0b101, 0b101],
        's' => [0b011, 0b100, 0b010, 0b001, 0b110],
        'n' => [0b000, 0b110, 0b101, 0b101, 0b101],
        'µ' => [0b000, 0b101, 0b101, 0b111, 0b100],
        ' ' => [0b000, 0b000, 0b000, 0b000, 0b000],
        _ => [0b000, 0b000, 0b000, 0b000, 0b000],
    }
}

/// Write an RGBA pixel buffer as a PNG file using the `png` crate with
/// maximum compression (no quality loss).
fn write_png(path: &Path, width: u32, height: u32, rgba: &[u8]) -> anyhow::Result<()> {
    use std::io::BufWriter;

    let file = std::fs::File::create(path)?;
    let w = BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.set_compression(png::Compression::Best);

    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba)?;
    Ok(())
}

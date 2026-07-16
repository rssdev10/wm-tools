//! Oscilloscope-style graph canvas (iced::widget::canvas).

use crate::app::{format_duration_ms_scale};
use crate::settings::ViewMode;
use dso_parser::{ADC_VALUE_MAX, ADC_VALUE_MID, Capture};
use iced::widget::canvas::{self, Frame, Geometry, Path, Stroke, Text};
use iced::{mouse, Color, Point, Rectangle, Renderer, Size, Theme};

/// Drawable scope view backed by an optional `Capture`.
pub struct Scope<'a, Message = ()> {
    pub capture: Option<&'a Capture>,
    pub show_ch1: bool,
    pub show_ch2: bool,
    pub view_mode: ViewMode,
    /// User-positioned measurement cursors: X range (low, high) in 0.0..1.0.
    pub meas_cursor_x: Option<(f32, f32)>,
    /// User-positioned measurement cursors: Y range (low, high) in 0.0..1.0.
    pub meas_cursor_y: Option<(f32, f32)>,
    /// Device-supplied X cursors (read-only, from capture data).
    pub device_cursor_x: Option<(f32, f32)>,
    /// Device-supplied Y cursors (read-only, from capture data).
    pub device_cursor_y: Option<(f32, f32)>,
    /// Show numeric axis scales on the graph.
    pub show_scales: bool,
    /// Number of samples for X scale calculation.
    pub n_samples: usize,
    /// Whether CH1 data is present in the capture (for scale labelling).
    pub has_ch1: bool,
    /// Whether CH2 data is present in the capture (for scale labelling).
    pub has_ch2: bool,
    /// Volts per division for CH1. 8 divisions total.
    pub v_per_div_ch1: f64,
    /// Volts per division for CH2. 8 divisions total.
    pub v_per_div_ch2: f64,
    /// Time per division in milliseconds. 12 divisions total.
    pub t_per_div_ms: f64,
    /// Voltage offset for CH1 (voltage at graph center).
    pub v_offset_ch1: f64,
    /// Voltage offset for CH2 (voltage at graph center).
    pub v_offset_ch2: f64,
    /// Active channel (0 = CH1, 1 = CH2). Used for device cursor voltage.
    pub active_channel: u8,
    /// Callback message factory for graph clicks: (frac_x, frac_y).
    pub on_click: Option<fn(f32, f32) -> Message>,
}

/// State for cursor drag interaction on the scope canvas.
#[derive(Default)]
pub struct ScopeState {
    dragging: Option<DragTarget>,
}

#[derive(Debug, Clone, Copy)]
enum DragTarget {
    XLow,
    XHigh,
    YLow,
    YHigh,
}

const BG: Color = Color::from_rgb(0.04, 0.06, 0.10);
const GRID: Color = Color::from_rgb(0.20, 0.24, 0.30);
const GRID_AXIS: Color = Color::from_rgb(0.35, 0.40, 0.46);
const CH1_COLOR: Color = Color::from_rgb(1.0, 0.85, 0.10);
const CH2_COLOR: Color = Color::from_rgb(0.95, 0.10, 0.95);
/// User-driven measurement cursors (interactive).
const MEAS_CURSOR: Color = Color::from_rgb(1.0, 0.30, 0.50);
const MEAS_FILL: Color = Color::from_rgba(1.0, 0.30, 0.50, 0.08);
/// Device-supplied cursors (read-only).
const DEV_CURSOR: Color = Color::from_rgb(0.40, 1.0, 0.40);

/// Build a `Text` label for a device cursor with consistent styling.
fn dev_label(content: String, position: Point) -> Text {
    Text {
        content,
        position,
        color: DEV_CURSOR,
        size: 10.0.into(),
        ..Text::default()
    }
}

impl<Message: Clone> canvas::Program<Message> for Scope<'_, Message> {
    type State = ScopeState;

    fn update(
        &self,
        state: &mut Self::State,
        event: &canvas::Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> Option<canvas::Action<Message>> {
        let on_click = self.on_click?;
        let pos = cursor.position_in(bounds)?;
        let frac_x = (pos.x / bounds.width).clamp(0.0, 1.0);
        let frac_y = (pos.y / bounds.height).clamp(0.0, 1.0);

        match event {
            canvas::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                // Determine which cursor line is nearest and start drag
                let mut best_dist = f32::MAX;
                let mut target: Option<DragTarget> = None;

                if let Some((lo, hi)) = self.meas_cursor_x {
                    let d_lo = (frac_x - lo).abs();
                    let d_hi = (frac_x - hi).abs();
                    if d_lo < best_dist {
                        best_dist = d_lo;
                        target = Some(DragTarget::XLow);
                    }
                    if d_hi < best_dist {
                        best_dist = d_hi;
                        target = Some(DragTarget::XHigh);
                    }
                }
                if let Some((lo, hi)) = self.meas_cursor_y {
                    let d_lo = (frac_y - lo).abs();
                    let d_hi = (frac_y - hi).abs();
                    if d_lo < best_dist {
                        best_dist = d_lo;
                        target = Some(DragTarget::YLow);
                    }
                    if d_hi < best_dist {
                        // best_dist = d_hi; // not needed, last comparison
                        target = Some(DragTarget::YHigh);
                    }
                }

                let _ = best_dist;
                state.dragging = target;
                let msg = on_click(frac_x, frac_y);
                Some(canvas::Action::publish(msg).and_capture())
            }
            canvas::Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                if state.dragging.is_some() {
                    let msg = on_click(frac_x, frac_y);
                    Some(canvas::Action::publish(msg).and_capture())
                } else {
                    None
                }
            }
            canvas::Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                if state.dragging.take().is_some() {
                    Some(canvas::Action::request_redraw())
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<Geometry> {
        let mut frame = Frame::new(renderer, bounds.size());

        // Background.
        let area = Path::rectangle(Point::ORIGIN, frame.size());
        frame.fill(&area, BG);

        draw_grid(&mut frame, bounds.size());

        if let Some(cap) = self.capture {
            // Draw traces for each enabled channel.
            let channels = [
                (self.show_ch1, Some(&cap.ch1[..]) as Option<&[u8]>, CH1_COLOR),
                (self.show_ch2, cap.ch2.as_deref(), CH2_COLOR),
            ];
            for &(show, data, color) in &channels {
                if let Some(samples) = show.then_some(data).flatten() {
                    draw_trace(&mut frame, samples, color, self.view_mode);
                }
            }
        } else {
            let text = Text {
                content: crate::i18n::t!("app.no_capture_loaded").to_string(),
                position: Point::new(frame.width() / 2.0, frame.height() / 2.0),
                color: Color::from_rgb(0.7, 0.7, 0.7),
                size: 16.0.into(),
                align_x: iced::alignment::Horizontal::Center.into(),
                align_y: iced::alignment::Vertical::Center,
                ..Text::default()
            };
            frame.fill_text(text);
        }

        // Measurement cursors (user-controlled, with shaded range).
        if let Some((xl, xr)) = self.meas_cursor_x {
            let w = frame.width();
            let h = frame.height();
            let px_l = xl.clamp(0.0, 1.0) * w;
            let px_r = xr.clamp(0.0, 1.0) * w;
            let region = Path::rectangle(
                Point::new(px_l, 0.0),
                Size::new((px_r - px_l).max(0.0), h),
            );
            frame.fill(&region, MEAS_FILL);
            let stroke = Stroke::default().with_color(MEAS_CURSOR).with_width(1.0);
            frame.stroke(&Path::line(Point::new(px_l, 0.0), Point::new(px_l, h)), stroke);
            frame.stroke(&Path::line(Point::new(px_r, 0.0), Point::new(px_r, h)), stroke);
        }
        if let Some((yu, yl)) = self.meas_cursor_y {
            let w = frame.width();
            let h = frame.height();
            let py_u = yu.clamp(0.0, 1.0) * h;
            let py_l = yl.clamp(0.0, 1.0) * h;
            let region = Path::rectangle(
                Point::new(0.0, py_u),
                Size::new(w, (py_l - py_u).max(0.0)),
            );
            frame.fill(&region, MEAS_FILL);
            let stroke = Stroke::default().with_color(MEAS_CURSOR).with_width(1.0);
            frame.stroke(&Path::line(Point::new(0.0, py_u), Point::new(w, py_u)), stroke);
            frame.stroke(&Path::line(Point::new(0.0, py_l), Point::new(w, py_l)), stroke);
        }

        // Device cursors (read-only, dashed-look using thinner line, different colour).
        let dev_stroke = Stroke::default().with_color(DEV_CURSOR).with_width(1.0);
        if let Some((a, b)) = self.device_cursor_x {
            let w = frame.width();
            let h = frame.height();
            for x in [a, b] {
                let px = x.clamp(0.0, 1.0) * w;
                frame.stroke(
                    &Path::line(Point::new(px, 0.0), Point::new(px, h)),
                    dev_stroke,
                );
            }
            // Show time values next to device X cursors.
            let total_time_ms = T_CELLS as f64 * self.t_per_div_ms;
            let fmt_time = |frac: f32| -> String {
                let t_ms = frac as f64 * total_time_ms;
                format_duration_ms_scale(t_ms)
            };
            frame.fill_text(dev_label(fmt_time(a), Point::new(a.clamp(0.0, 1.0) * w + 2.0, 12.0)));
            frame.fill_text(dev_label(fmt_time(b), Point::new(b.clamp(0.0, 1.0) * w + 2.0, 12.0)));
            // Also show Δt between cursors.
            let dt_ms = (b - a).abs() as f64 * total_time_ms;
            let dt_label = format!("Δt={}", format_duration_ms_scale(dt_ms));
            frame.fill_text(dev_label(dt_label, Point::new(4.0, 22.0)));
        }
        if let Some((a, b)) = self.device_cursor_y {
            let w = frame.width();
            let h = frame.height();
            for y in [a, b] {
                let py = y.clamp(0.0, 1.0) * h;
                frame.stroke(
                    &Path::line(Point::new(0.0, py), Point::new(w, py)),
                    dev_stroke,
                );
            }
            // Show numeric voltage values next to device Y cursors.
            // Use the active channel's V/div and V_offset for correct voltage.
            let (v_per_div, v_offset) = if self.active_channel == 0 {
                (self.v_per_div_ch1, self.v_offset_ch1)
            } else {
                (self.v_per_div_ch2, self.v_offset_ch2)
            };
            let cursor_to_voltage = |frac: f32| -> f64 {
                // Map fraction (0=top, 1=bottom) to pixel_val (255=top, 0=bottom)
                // using the full 0..255 ADC range to match pixel_to_volts.
                let pixel_val = (1.0 - frac as f64) * ADC_VALUE_MAX as f64;
                v_offset + (pixel_val - ADC_VALUE_MID as f64) * v_per_div / PX_PER_DIV
            };
            let v_a = cursor_to_voltage(a);
            let v_b = cursor_to_voltage(b);
            frame.fill_text(dev_label(format!("{v_a:.2}V"), Point::new(4.0, a.clamp(0.0, 1.0) * h + 2.0)));
            frame.fill_text(dev_label(format!("{v_b:.2}V"), Point::new(4.0, b.clamp(0.0, 1.0) * h + 2.0)));
        }

        // Axis scales (sample numbers on X, voltage on Y per-channel)
        if self.show_scales {
            draw_scales(
                &mut frame,
                self.n_samples,
                self.has_ch1,
                self.has_ch2,
                self.v_per_div_ch1,
                self.v_per_div_ch2,
                self.t_per_div_ms,
                self.v_offset_ch1,
                self.v_offset_ch2,
            );
        }

        vec![frame.into_geometry()]
    }
}

fn draw_grid(frame: &mut Frame, size: Size) {
    // 12 horizontal x 8 vertical divisions, like a real scope.
    let cols = T_CELLS;
    let rows = V_CELLS;
    let dx = size.width / cols as f32;
    let dy = size.height / rows as f32;

    let stroke = Stroke::default().with_color(GRID).with_width(1.0);
    for i in 1..cols {
        let x = i as f32 * dx;
        let p = Path::line(Point::new(x, 0.0), Point::new(x, size.height));
        frame.stroke(&p, stroke);
    }
    for i in 1..rows {
        let y = i as f32 * dy;
        let p = Path::line(Point::new(0.0, y), Point::new(size.width, y));
        frame.stroke(&p, stroke);
    }
    // Centerlines.
    let axis = Stroke::default().with_color(GRID_AXIS).with_width(1.5);
    let cx = size.width / 2.0;
    let cy = size.height / 2.0;
    frame.stroke(
        &Path::line(Point::new(cx, 0.0), Point::new(cx, size.height)),
        axis,
    );
    frame.stroke(
        &Path::line(Point::new(0.0, cy), Point::new(size.width, cy)),
        axis,
    );
}

const SCALE_COLOR: Color = Color::from_rgb(0.60, 0.65, 0.70);

/// Number of vertical divisions on the oscilloscope screen for voltage.
pub const V_CELLS: usize = 8;
/// Number of horizontal divisions on the oscilloscope screen for time.
pub const T_CELLS: usize = 12;
/// Device pixel density: 25 px/division. Re-exported from dso_parser.
pub use dso_parser::PIXELS_PER_DIV as PX_PER_DIV;
/// Total pixels for the full grid height: V_CELLS × PX_PER_DIV.
pub const GRID_HEIGHT_PX: f64 = V_CELLS as f64 * PX_PER_DIV;

#[allow(clippy::too_many_arguments)]
fn draw_scales(
    frame: &mut Frame,
    _n_samples: usize,
    has_ch1: bool,
    has_ch2: bool,
    v_per_div_ch1: f64,
    v_per_div_ch2: f64,
    t_per_div_ms: f64,
    v_offset_ch1: f64,
    v_offset_ch2: f64,
) {
    let size = frame.size();
    let rows = V_CELLS;
    let cols = T_CELLS; // grid visual divisions

    // X axis: time labels at each visual division.
    let total_time_ms = T_CELLS as f64 * t_per_div_ms;
    let dx = size.width / cols as f32;
    for i in 0..=cols {
        let frac = i as f64 / cols as f64;
        let time_ms = frac * total_time_ms;
        let x = i as f32 * dx;
        let content = format_duration_ms_scale(time_ms);
        let label = Text {
            content,
            position: Point::new(x + 2.0, size.height - 12.0),
            color: SCALE_COLOR,
            size: 9.0.into(),
            ..Text::default()
        };
        frame.fill_text(label);
    }

    // Y axis: voltage scale per channel.
    // Formula from pixel_to_volts: V = V_offset + (ADC_MID - v) × V/div / PIXELS_PER_DIV,
    // where v = i × PX_PER_DIV is the stored value at grid line i.
    let ch1_x_offset = 2.0;
    let ch2_x_offset = if has_ch1 { 42.0 } else { 2.0 };

    let mut label_ch = |v_per_div: f64, v_offset: f64, color: Color, x_off: f32| {
        for i in 0..=rows {
            let v = ((i + 1) as f64 / rows as f64) * GRID_HEIGHT_PX;
            let voltage = v_offset + (ADC_VALUE_MID as f64 - v) * v_per_div / PX_PER_DIV;
            let y = (i as f32 / rows as f32) * size.height;
            let label = Text {
                content: format!("{voltage:.2}V"),
                position: Point::new(x_off, y + 2.0),
                color,
                size: 9.0.into(),
                ..Text::default()
            };
            frame.fill_text(label);
        }
    };

    if has_ch1 {
        label_ch(v_per_div_ch1, v_offset_ch1, CH1_COLOR, ch1_x_offset);
    }
    if has_ch2 {
        label_ch(v_per_div_ch2, v_offset_ch2, CH2_COLOR, ch2_x_offset);
    }
}

fn draw_trace(frame: &mut Frame, samples: &[u8], color: Color, mode: ViewMode) {
    if samples.is_empty() {
        return;
    }
    let size = frame.size();
    let n = samples.len();
    let stroke = Stroke::default().with_color(color).with_width(1.2);

    // x maps sample i -> pixel; downsample so we never draw more vertices than pixels.
    let max_pts = (size.width as usize).max(2);
    let step = n.div_ceil(max_pts).max(1);

    let value_at = |i: usize| -> f32 {
        match mode {
            ViewMode::Smooth => {
                // Moving average with a small window relative to downsampling step.
                let half = (step / 2).max(2);
                let lo = i.saturating_sub(half);
                let hi = (i + half + 1).min(n);
                let sum: u32 = samples[lo..hi].iter().map(|&b| b as u32).sum();
                sum as f32 / (hi - lo) as f32
            }
            _ => samples[i] as f32,
        }
    };

    let to_xy = |i: usize, v: f32| -> Point {
        let x = (i as f32 / (n - 1).max(1) as f32) * size.width;
        // Capture values use screen-coordinate convention: low value = top
        // of screen = positive voltage, high value = bottom = negative voltage.
        // Use device pixel density: GRID_HEIGHT_PX = V_CELLS × PX_PER_DIV.
        let y = ((v - PX_PER_DIV as f32) / GRID_HEIGHT_PX as f32) * size.height;
        Point::new(x, y)
    };

    match mode {
        ViewMode::Dot => {
            let r = 1.2;
            for i in (0..n).step_by(step) {
                let p = to_xy(i, value_at(i));
                let dot = Path::circle(p, r);
                frame.fill(&dot, color);
            }
        }
        ViewMode::Line | ViewMode::Smooth => {
            let path = Path::new(|builder| {
                let mut started = false;
                for i in (0..n).step_by(step) {
                    let p = to_xy(i, value_at(i));
                    if started {
                        builder.line_to(p);
                    } else {
                        builder.move_to(p);
                        started = true;
                    }
                }
            });
            frame.stroke(&path, stroke);
        }
    }
}

// ── Thumbnail mini-scope ───────────────────────────────────────────────────

const THUMB_BG: Color = Color::from_rgb(0.06, 0.08, 0.12);
const THUMB_BORDER: Color = Color::from_rgb(0.30, 0.35, 0.42);
const THUMB_SELECTED: Color = Color::from_rgb(1.0, 0.30, 0.50);

/// A tiny waveform thumbnail for the captures strip.
pub struct Thumbnail<'a> {
    pub capture: &'a Capture,
    pub selected: bool,
}

impl<Message> canvas::Program<Message> for Thumbnail<'_> {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<Geometry> {
        let mut frame = Frame::new(renderer, bounds.size());
        let size = frame.size();

        // Background
        let bg = Path::rectangle(Point::ORIGIN, size);
        frame.fill(&bg, THUMB_BG);

        // Border (highlight if selected)
        let border_color = if self.selected { THUMB_SELECTED } else { THUMB_BORDER };
        let border = Path::rectangle(Point::ORIGIN, size);
        frame.stroke(&border, Stroke::default().with_color(border_color).with_width(if self.selected { 2.0 } else { 1.0 }));

        // Draw miniature traces for each channel.
        let mut draw_mini = |samples: &[u8], color: Color| {
            if samples.is_empty() {
                return;
            }
            let n = samples.len();
            let max_pts = (size.width as usize).max(2);
            let step = n.div_ceil(max_pts).max(1);
            let path = Path::new(|builder| {
                let mut started = false;
                for i in (0..n).step_by(step) {
                    let x = (i as f32 / (n - 1).max(1) as f32) * size.width;
                    let y = ((samples[i] - PX_PER_DIV as u8) as f32 / GRID_HEIGHT_PX as f32) * size.height;
                    let p = Point::new(x, y);
                    if started {
                        builder.line_to(p);
                    } else {
                        builder.move_to(p);
                        started = true;
                    }
                }
            });
            frame.stroke(&path, Stroke::default().with_color(color).with_width(1.0));
        };

        draw_mini(&self.capture.ch1, CH1_COLOR);
        if let Some(ch2) = &self.capture.ch2 {
            draw_mini(ch2, CH2_COLOR);
        }

        vec![frame.into_geometry()]
    }

    fn mouse_interaction(
        &self,
        _state: &Self::State,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        if cursor.is_over(bounds) {
            mouse::Interaction::Pointer
        } else {
            mouse::Interaction::default()
        }
    }
}

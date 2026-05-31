//! Oscilloscope-style graph canvas (iced::widget::canvas).

use crate::settings::ViewMode;
use dso3d12_parser::Capture;
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
    /// Volts per cell (vertical division). 8 cells total.
    pub v_per_cell: f64,
    /// Time per cell (horizontal division) in milliseconds. 12 cells total.
    pub t_per_cell_ms: f64,
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
const CH2_COLOR: Color = Color::from_rgb(0.15, 0.85, 1.0);
/// User-driven measurement cursors (interactive).
const MEAS_CURSOR: Color = Color::from_rgb(1.0, 0.30, 0.50);
const MEAS_FILL: Color = Color::from_rgba(1.0, 0.30, 0.50, 0.08);
/// Device-supplied cursors (read-only).
const DEV_CURSOR: Color = Color::from_rgb(0.40, 1.0, 0.40);

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
            if self.show_ch1 {
                draw_trace(&mut frame, &cap.ch1, CH1_COLOR, self.view_mode);
            }
            if self.show_ch2 {
                if let Some(ch2) = cap.ch2.as_ref() {
                    // CH2 is stored inverted in the debug dump; flip to match device display.
                    let flipped: Vec<u8> = ch2.iter().map(|&v| 255 - v).collect();
                    draw_trace(&mut frame, &flipped, CH2_COLOR, self.view_mode);
                }
            }
        } else {
            let text = Text {
                content: "No capture loaded — use Load Capture…".to_string(),
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
            // Show numeric values next to device X cursors
            let sample_a = (a * self.n_samples as f32) as usize;
            let sample_b = (b * self.n_samples as f32) as usize;
            let label_a = Text {
                content: format!("{sample_a}"),
                position: Point::new(a.clamp(0.0, 1.0) * w + 2.0, 12.0),
                color: DEV_CURSOR,
                size: 10.0.into(),
                ..Text::default()
            };
            let label_b = Text {
                content: format!("{sample_b}"),
                position: Point::new(b.clamp(0.0, 1.0) * w + 2.0, 12.0),
                color: DEV_CURSOR,
                size: 10.0.into(),
                ..Text::default()
            };
            frame.fill_text(label_a);
            frame.fill_text(label_b);
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
            // Show numeric voltage values next to device Y cursors
            let v_a = (a * 255.0 - 128.0) * 5.0 / 128.0;
            let v_b = (b * 255.0 - 128.0) * 5.0 / 128.0;
            let label_a = Text {
                content: format!("{v_a:.2}V"),
                position: Point::new(4.0, a.clamp(0.0, 1.0) * h + 2.0),
                color: DEV_CURSOR,
                size: 10.0.into(),
                ..Text::default()
            };
            let label_b = Text {
                content: format!("{v_b:.2}V"),
                position: Point::new(4.0, b.clamp(0.0, 1.0) * h + 2.0),
                color: DEV_CURSOR,
                size: 10.0.into(),
                ..Text::default()
            };
            frame.fill_text(label_a);
            frame.fill_text(label_b);
        }

        // Axis scales (sample numbers on X, voltage on Y per-channel)
        if self.show_scales {
            draw_scales(
                &mut frame,
                self.n_samples,
                self.has_ch1,
                self.has_ch2,
                self.v_per_cell,
                self.t_per_cell_ms,
            );
        }

        vec![frame.into_geometry()]
    }
}

fn draw_grid(frame: &mut Frame, size: Size) {
    // 12 horizontal x 8 vertical divisions, like a real scope.
    let cols = 12;
    let rows = 8;
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

/// Number of vertical divisions (cells) on the oscilloscope screen for voltage.
const V_CELLS: usize = 8;
/// Number of horizontal divisions (cells) on the oscilloscope screen for time.
const T_CELLS: usize = 12;

fn draw_scales(
    frame: &mut Frame,
    _n_samples: usize,
    has_ch1: bool,
    has_ch2: bool,
    v_per_cell: f64,
    t_per_cell_ms: f64,
) {
    let size = frame.size();
    let rows = V_CELLS;
    let cols = 10; // grid visual divisions (drawn grid is 10 columns)

    // X axis: time labels at each visual division.
    // Total time span = T_CELLS * t_per_cell_ms. We distribute over the sample window.
    let total_time_ms = T_CELLS as f64 * t_per_cell_ms;
    let dx = size.width / cols as f32;
    for i in 0..=cols {
        let frac = i as f64 / cols as f64;
        let time_ms = frac * total_time_ms;
        let x = i as f32 * dx;
        let content = if time_ms >= 1.0 {
            format!("{time_ms:.1}ms")
        } else {
            format!("{:.0}µs", time_ms * 1000.0)
        };
        let label = Text {
            content,
            position: Point::new(x + 2.0, size.height - 12.0),
            color: SCALE_COLOR,
            size: 9.0.into(),
            ..Text::default()
        };
        frame.fill_text(label);
    }

    // Y axis: voltage scale. Total range = V_CELLS * v_per_cell, centered around 0.
    // Top = +half_range, bottom = -half_range (AC mode).
    let half_range_v = (rows as f64 * v_per_cell) / 2.0;
    let ch1_x_offset = 2.0;
    let ch2_x_offset = if has_ch1 { 42.0 } else { 2.0 };

    if has_ch1 {
        for i in 0..=rows {
            let frac = i as f32 / rows as f32;
            // frac=0 → top → +half_range; frac=1 → bottom → -half_range
            let voltage = half_range_v * (1.0 - 2.0 * frac as f64);
            let y = frac * size.height;
            let label = Text {
                content: format!("{voltage:.2}V"),
                position: Point::new(ch1_x_offset, y + 2.0),
                color: CH1_COLOR,
                size: 9.0.into(),
                ..Text::default()
            };
            frame.fill_text(label);
        }
    }

    if has_ch2 {
        for i in 0..=rows {
            let frac = i as f32 / rows as f32;
            let voltage = half_range_v * (1.0 - 2.0 * frac as f64);
            let y = frac * size.height;
            let label = Text {
                content: format!("{voltage:.2}V"),
                position: Point::new(ch2_x_offset, y + 2.0),
                color: CH2_COLOR,
                size: 9.0.into(),
                ..Text::default()
            };
            frame.fill_text(label);
        }
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
        // Invert Y: higher ADC value (higher voltage) at top of screen.
        let y = (1.0 - v / 255.0) * size.height;
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

        // Draw CH1 trace
        let samples = &self.capture.ch1;
        if !samples.is_empty() {
            let n = samples.len();
            let max_pts = (size.width as usize).max(2);
            let step = n.div_ceil(max_pts).max(1);
            let path = Path::new(|builder| {
                let mut started = false;
                for i in (0..n).step_by(step) {
                    let x = (i as f32 / (n - 1).max(1) as f32) * size.width;
                    let y = (samples[i] as f32 / 255.0) * size.height;
                    let p = Point::new(x, y);
                    if started {
                        builder.line_to(p);
                    } else {
                        builder.move_to(p);
                        started = true;
                    }
                }
            });
            frame.stroke(&path, Stroke::default().with_color(CH1_COLOR).with_width(1.0));
        }

        // Draw CH2 if present
        if let Some(ch2) = &self.capture.ch2 {
            if !ch2.is_empty() {
                let n = ch2.len();
                let max_pts = (size.width as usize).max(2);
                let step = n.div_ceil(max_pts).max(1);
                let path = Path::new(|builder| {
                    let mut started = false;
                    for i in (0..n).step_by(step) {
                        let x = (i as f32 / (n - 1).max(1) as f32) * size.width;
                        let y = (ch2[i] as f32 / 255.0) * size.height;
                        let p = Point::new(x, y);
                        if started {
                            builder.line_to(p);
                        } else {
                            builder.move_to(p);
                            started = true;
                        }
                    }
                });
                frame.stroke(&path, Stroke::default().with_color(CH2_COLOR).with_width(1.0));
            }
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

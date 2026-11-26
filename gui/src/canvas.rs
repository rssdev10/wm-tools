//! Oscilloscope-style graph canvas (iced::widget::canvas).
use crate::settings::ViewMode;
use dso_parser::{ADC_VALUE_MID, Capture};
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
    /// Message factory for measurement-cursor drags on the graph.
    pub on_cursor_drag: Option<fn(CursorDrag) -> Message>,
}

/// State of the scope canvas: cursor drag interaction and the cached
/// trace layer.
#[derive(Default)]
pub struct ScopeState {
    dragging: Option<DragTarget>,
    /// Tessellated traces. Re-tessellating 60 000 samples on every mouse
    /// move of a cursor drag is what made the GUI slow; the traces only
    /// change when the data or the trace options do.
    traces: canvas::Cache,
    /// Fingerprint of everything the cached traces depend on.
    traces_key: std::cell::Cell<Option<u64>>,
}

/// One knob of a measurement-cursor pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Knob {
    /// First value of the `(low, high)` range.
    Low,
    /// Second value of the `(low, high)` range.
    High,
}

/// The measurement-cursor lines grabbed by a drag: one line, or one X and
/// one Y line together when the press was near where they cross.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DragTarget {
    pub x: Option<Knob>,
    pub y: Option<Knob>,
}

/// One step of a measurement-cursor drag: move the target lines to the
/// pointer position, given as fractions (0.0..1.0) of the plot area.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CursorDrag {
    pub target: DragTarget,
    pub frac_x: f32,
    pub frac_y: f32,
}

/// A press within this distance (pixels) of the crossing of the nearest X
/// and Y cursor lines drags both lines together.
pub const CROSS_GRAB_RADIUS_PX: f32 = 12.0;
/// The pointer within this distance (pixels) of a cursor line shows a
/// resize pointer.
const LINE_HOVER_PX: f32 = 6.0;

/// Nearest knob of a cursor pair to `pos` along one axis, and its distance
/// in pixels. When both knobs sit on the same pixel, the knob on the side the
/// pointer is on wins, so the pair can be pulled apart in either direction.
fn nearest_knob(pos: f32, origin: f32, extent: f32, (lo, hi): (f32, f32)) -> (Knob, f32) {
    let p_lo = origin + lo.clamp(0.0, 1.0) * extent;
    let p_hi = origin + hi.clamp(0.0, 1.0) * extent;
    let d_lo = (pos - p_lo).abs();
    let d_hi = (pos - p_hi).abs();
    if d_lo < d_hi || (d_lo == d_hi && pos <= p_lo) {
        (Knob::Low, d_lo)
    } else {
        (Knob::High, d_hi)
    }
}

/// Decide which measurement-cursor lines a press at `pos` grabs.
///
/// Near the crossing of the nearest X and Y lines (within
/// [`CROSS_GRAB_RADIUS_PX`]) both lines are grabbed; otherwise only the
/// single nearest line, measured in pixels. Returns `None` when no
/// measurement cursor is enabled.
pub fn pick_drag_target(
    pos: Point,
    plot: Rectangle,
    cursor_x: Option<(f32, f32)>,
    cursor_y: Option<(f32, f32)>,
) -> Option<DragTarget> {
    let x = cursor_x.map(|r| nearest_knob(pos.x, plot.x, plot.width, r));
    let y = cursor_y.map(|r| nearest_knob(pos.y, plot.y, plot.height, r));
    let target = match (x, y) {
        (None, None) => return None,
        (Some((kx, _)), None) => DragTarget { x: Some(kx), y: None },
        (None, Some((ky, _))) => DragTarget { x: None, y: Some(ky) },
        (Some((kx, dx)), Some((ky, dy))) => {
            if dx.hypot(dy) <= CROSS_GRAB_RADIUS_PX {
                DragTarget { x: Some(kx), y: Some(ky) }
            } else if dx <= dy {
                DragTarget { x: Some(kx), y: None }
            } else {
                DragTarget { x: None, y: Some(ky) }
            }
        }
    };
    Some(target)
}

/// Convert a position (canvas coordinates) to plot fractions, clamped to
/// the plot area so a drag that leaves the canvas pins the line to the edge.
fn plot_fraction(pos: Point, plot: Rectangle) -> (f32, f32) {
    let fx = if plot.width > 0.0 { (pos.x - plot.x) / plot.width } else { 0.0 };
    let fy = if plot.height > 0.0 { (pos.y - plot.y) / plot.height } else { 0.0 };
    (fx.clamp(0.0, 1.0), fy.clamp(0.0, 1.0))
}

/// Pointer shape for a drag target.
fn interaction_for(target: DragTarget) -> mouse::Interaction {
    match (target.x, target.y) {
        (Some(_), Some(_)) => mouse::Interaction::Move,
        (Some(_), None) => mouse::Interaction::ResizingHorizontally,
        (None, Some(_)) => mouse::Interaction::ResizingVertically,
        (None, None) => mouse::Interaction::default(),
    }
}

/// The plot area (grid and traces) inside a canvas of `size`, leaving room
/// for the scale labels when they are shown.
pub fn plot_rect(size: Size, show_scales: bool, has_ch1: bool, has_ch2: bool) -> Rectangle {
    let (left, bottom) = if show_scales {
        let label_columns = u8::from(has_ch1) + u8::from(has_ch2);
        (SCALE_MARGIN_LEFT * f32::from(label_columns), SCALE_MARGIN_BOTTOM)
    } else {
        (0.0, 0.0)
    };
    Rectangle {
        x: left,
        y: 0.0,
        width: (size.width - left).max(0.0),
        height: (size.height - bottom).max(0.0),
    }
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
        let on_cursor_drag = self.on_cursor_drag?;
        let plot = plot_rect(bounds.size(), self.show_scales, self.has_ch1, self.has_ch2);
        let step = |target: DragTarget, pos: Point| {
            let (frac_x, frac_y) = plot_fraction(pos, plot);
            canvas::Action::publish(on_cursor_drag(CursorDrag { target, frac_x, frac_y }))
                .and_capture()
        };

        match event {
            canvas::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                // Only a press inside the canvas starts a drag.
                let pos = cursor.position_in(bounds)?;
                let target =
                    pick_drag_target(pos, plot, self.meas_cursor_x, self.meas_cursor_y)?;
                state.dragging = Some(target);
                Some(step(target, pos))
            }
            canvas::Event::Mouse(mouse::Event::CursorMoved { position }) => {
                // Keep following the pointer after it leaves the canvas; the
                // position is clamped to the plot edge.
                let target = state.dragging?;
                let pos = Point::new(position.x - bounds.x, position.y - bounds.y);
                Some(step(target, pos))
            }
            canvas::Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                // Handled wherever the pointer is, so a release outside the
                // canvas cannot leave a drag stuck.
                state
                    .dragging
                    .take()
                    .map(|_| canvas::Action::request_redraw())
            }
            _ => None,
        }
    }

    fn mouse_interaction(
        &self,
        state: &Self::State,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        if let Some(target) = state.dragging {
            return interaction_for(target);
        }
        if self.on_cursor_drag.is_none() {
            return mouse::Interaction::default();
        }
        let Some(pos) = cursor.position_in(bounds) else {
            return mouse::Interaction::default();
        };
        let plot = plot_rect(bounds.size(), self.show_scales, self.has_ch1, self.has_ch2);
        let Some(target) = pick_drag_target(pos, plot, self.meas_cursor_x, self.meas_cursor_y)
        else {
            return mouse::Interaction::default();
        };
        // A press anywhere moves the nearest line; show the resize pointer
        // only when the pointer is actually on a line or the crossing.
        let near = |range: Option<(f32, f32)>, p: f32, o: f32, e: f32| {
            range.is_some_and(|r| nearest_knob(p, o, e, r).1 <= LINE_HOVER_PX)
        };
        let on_x = near(self.meas_cursor_x, pos.x, plot.x, plot.width);
        let on_y = near(self.meas_cursor_y, pos.y, plot.y, plot.height);
        let on_target = match (target.x, target.y) {
            (Some(_), Some(_)) => true,
            (Some(_), None) => on_x,
            (None, Some(_)) => on_y,
            (None, None) => false,
        };
        if on_target {
            interaction_for(target)
        } else {
            mouse::Interaction::default()
        }
    }

    fn draw(
        &self,
        state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<Geometry> {
        // Three layers, drawn in order: background and grid, traces (cached),
        // then cursors and labels on top.
        let mut base = Frame::new(renderer, bounds.size());

        // Background.
        let area = Path::rectangle(Point::ORIGIN, base.size());
        base.fill(&area, BG);

        let plot = plot_rect(bounds.size(), self.show_scales, self.has_ch1, self.has_ch2);
        let gx = plot.x;
        let gw = plot.width;
        let gh = plot.height;

        draw_grid(&mut base, bounds.size(), gx, bounds.height - gh);

        let key = self.traces_key(plot);
        if state.traces_key.get() != Some(key) {
            state.traces.clear();
            state.traces_key.set(Some(key));
        }
        let traces = state.traces.draw(renderer, bounds.size(), |frame| {
            let Some(cap) = self.capture else { return };
            // Draw traces for each enabled channel. draw_trace() clips them
            // to the plot area so off-screen samples do not run over the
            // scale labels.
            let channels = [
                (self.show_ch1, Some(&cap.ch1[..]) as Option<&[u8]>, CH1_COLOR),
                (self.show_ch2, cap.ch2.as_deref(), CH2_COLOR),
            ];
            for &(show, data, color) in &channels {
                if let Some(samples) = show.then_some(data).flatten() {
                    draw_trace(frame, samples, color, self.view_mode, plot);
                }
            }
        });

        let mut frame = Frame::new(renderer, bounds.size());
        if self.capture.is_none() {
            let text = Text {
                content: crate::i18n::t!("app.no_capture_loaded").to_string(),
                position: Point::new(gx + gw / 2.0, gh / 2.0),
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
            let h = gh;
            let px_l = gx + xl.clamp(0.0, 1.0) * gw;
            let px_r = gx + xr.clamp(0.0, 1.0) * gw;
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
            let h = gh;
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
            let h = gh;
            for x in [a, b] {
                let px = gx + x.clamp(0.0, 1.0) * gw;
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
            frame.fill_text(dev_label(fmt_time(a), Point::new(gx + a.clamp(0.0, 1.0) * gw + 2.0, 12.0)));
            frame.fill_text(dev_label(fmt_time(b), Point::new(gx + b.clamp(0.0, 1.0) * gw + 2.0, 12.0)));
            // Also show Δt between cursors.
            let dt_ms = (b - a).abs() as f64 * total_time_ms;
            let dt_label = format!("Δt={}", format_duration_ms_scale(dt_ms));
            frame.fill_text(dev_label(dt_label, Point::new(gx + 4.0, 22.0)));
        }
        if let Some((a, b)) = self.device_cursor_y {
            let w = frame.width();
            let h = gh;
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
                // Match pixel_to_volts / draw_scales: v = frac × GRID_HEIGHT_PX + PX_PER_DIV
                // so that frac=0 (top) maps to v=PX_PER_DIV, aligning with the grid labels.
                let v = frac as f64 * GRID_HEIGHT_PX + PX_PER_DIV;
                v_offset + (ADC_VALUE_MID as f64 - v) * v_per_div / PX_PER_DIV
            };
            let v_a = cursor_to_voltage(a);
            let v_b = cursor_to_voltage(b);
            frame.fill_text(dev_label(format!("{v_a:.2}V"), Point::new(gx + 4.0, a.clamp(0.0, 1.0) * h + 2.0)));
            frame.fill_text(dev_label(format!("{v_b:.2}V"), Point::new(gx + 4.0, b.clamp(0.0, 1.0) * h + 2.0)));
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
                gx,
                gh,
            );
        }

        vec![base.into_geometry(), traces, frame.into_geometry()]
    }
}

impl<Message> Scope<'_, Message> {
    /// Fingerprint of everything the trace layer depends on: the samples,
    /// the trace options and the plot area. Hashing 2 × 60 000 bytes takes
    /// microseconds, far less than tessellating them.
    fn traces_key(&self, plot: Rectangle) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut h = std::collections::hash_map::DefaultHasher::new();
        if let Some(cap) = self.capture {
            cap.ch1.hash(&mut h);
            cap.ch2.hash(&mut h);
        } else {
            0u8.hash(&mut h);
        }
        (self.show_ch1, self.show_ch2, self.view_mode as u8).hash(&mut h);
        for v in [plot.x, plot.y, plot.width, plot.height] {
            v.to_bits().hash(&mut h);
        }
        h.finish()
    }
}

fn draw_grid(frame: &mut Frame, size: Size, margin_left: f32, margin_bottom: f32) {
    // 12 horizontal x 8 vertical divisions, like a real scope.
    let cols = T_CELLS;
    let rows = V_CELLS;
    let gw = size.width - margin_left;
    let dx = gw / cols as f32;
    let graph_height = size.height - margin_bottom;
    let dy = graph_height / rows as f32;

    let stroke = Stroke::default().with_color(GRID).with_width(1.0);
    for i in 0..cols {
        let x = margin_left + i as f32 * dx;
        let p = Path::line(Point::new(x, 0.0), Point::new(x, graph_height));
        frame.stroke(&p, stroke);
    }
    for i in 1..rows+1 {
        let y = i as f32 * dy;
        let p = Path::line(Point::new(0.0, y), Point::new(size.width, y));
        frame.stroke(&p, stroke);
    }
    // Centerlines.
    let axis = Stroke::default().with_color(GRID_AXIS).with_width(1.5);
    let cx = margin_left + gw / 2.0;
    let cy = graph_height / 2.0;
    frame.stroke(
        &Path::line(Point::new(cx, 0.0), Point::new(cx, graph_height)),
        axis,
    );
    frame.stroke(
        &Path::line(Point::new(0.0, cy), Point::new(size.width, cy)),
        axis,
    );
}

/// Left margin reserved for voltage scale labels (pixels).
const SCALE_MARGIN_LEFT: f32 = 40.0;
const SCALE_MARGIN_BOTTOM: f32 = 15.0;

const SCALE_COLOR: Color = Color::from_rgb(0.60, 0.65, 0.70);

/// Number of vertical divisions on the oscilloscope screen for voltage.
pub const V_CELLS: usize = 8;
/// Number of horizontal divisions on the oscilloscope screen for time.
pub const T_CELLS: usize = 12;
/// Device pixel density: 25 px/division. Re-exported from dso_parser.
pub use dso_parser::PIXELS_PER_DIV as PX_PER_DIV;
/// Total pixels for the full grid height: V_CELLS × PX_PER_DIV.
pub const GRID_HEIGHT_PX: f64 = (V_CELLS * PX_PER_DIV as usize) as f64;

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
    margin_left: f32,
    graph_height: f32,
) {
    let size = frame.size();
    let rows = V_CELLS;
    let cols = T_CELLS; // grid visual divisions
    let gw = size.width - margin_left;

    // X axis: time labels at each visual division, in the bottom margin.
    let total_time_ms = T_CELLS as f64 * t_per_div_ms;
    let dx = gw / cols as f32;
    for i in 0..=cols {
        let frac = i as f64 / cols as f64;
        let time_ms = frac * total_time_ms;
        let x = margin_left + i as f32 * dx;
        let content = format_duration_ms_scale(time_ms);
        let label = Text {
            content,
            position: Point::new(x + 2.0, graph_height + 2.0),
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
            let y = (i as f32 / rows as f32) * graph_height;
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

/// Draw one channel's trace into `plot`.
///
/// Line and dot modes keep every sample's contribution: with more samples
/// than pixel columns, each column draws the min/max of its samples (line)
/// or each distinct value once (dot), so a single-sample spike stays
/// visible. Smooth mode is a deliberate visual filter and averages.
fn draw_trace(frame: &mut Frame, samples: &[u8], color: Color, mode: ViewMode, plot: Rectangle) {
    if samples.is_empty() {
        return;
    }
    let n = samples.len();
    let stroke = Stroke::default().with_color(color).with_width(1.2);
    let columns = (plot.width as usize).max(2);

    // Sample index (possibly fractional) -> x pixel, so that sample 0 is on
    // the left edge and sample n-1 on the right edge.
    let x_at = |i: f32| plot.x + i / (n - 1).max(1) as f32 * plot.width;
    // Capture values use screen-coordinate convention: low value = top
    // of screen = positive voltage, high value = bottom = negative voltage.
    // Use device pixel density: GRID_HEIGHT_PX = V_CELLS × PX_PER_DIV.
    let y_at =
        |v: f32| plot.y + (((v - PX_PER_DIV as f32) as f64 / GRID_HEIGHT_PX) * plot.height as f64) as f32;
    // Representative x of a column: the middle of its sample range.
    let column_x = |s: &crate::plot::ColumnSpan| x_at((s.start + s.end - 1) as f32 / 2.0);
    let (top, bottom) = (plot.y, plot.y + plot.height);

    // Stroke the parts of a polyline that are inside the plot area.
    let mut stroke_clipped = |points: Vec<(f32, f32)>| {
        let runs = crate::plot::clip_polyline_y(&points, top, bottom);
        let path = Path::new(|builder| {
            for run in runs.iter().filter(|r| r.len() >= 2) {
                builder.move_to(Point::new(run[0].0, run[0].1));
                for &(x, y) in &run[1..] {
                    builder.line_to(Point::new(x, y));
                }
            }
        });
        frame.stroke(&path, stroke);
    };

    match mode {
        ViewMode::Dot => {
            const R: f32 = 1.2;
            let mut dots: Vec<(f32, f32)> = Vec::new();
            if n <= columns {
                dots.extend(samples.iter().enumerate().map(|(i, &v)| (x_at(i as f32), y_at(v as f32))));
            } else {
                for span in crate::plot::column_spans(samples, columns) {
                    let x = column_x(&span);
                    dots.extend(
                        crate::plot::distinct_values(&samples[span.start..span.end])
                            .map(|v| (x, y_at(v as f32))),
                    );
                }
            }
            // Off-screen samples get no dot (same as the PNG export). Each
            // dot is a small square through iced's rectangle fast path:
            // at this size it looks like a round dot. Putting thousands of
            // overlapping circles in one path is very slow, because the fill
            // tessellator must resolve every overlap.
            let size = Size::new(2.0 * R, 2.0 * R);
            for &(x, y) in dots.iter().filter(|d| (top..=bottom).contains(&d.1)) {
                frame.fill_rectangle(Point::new(x - R, y - R), size, color);
            }
        }
        ViewMode::Line => {
            let points = if n <= columns {
                samples.iter().enumerate().map(|(i, &v)| (x_at(i as f32), y_at(v as f32))).collect()
            } else {
                crate::plot::column_spans(samples, columns)
                    .iter()
                    .flat_map(|s| {
                        let x = column_x(s);
                        s.ordered().map(|v| (x, y_at(v as f32)))
                    })
                    .collect()
            };
            stroke_clipped(points);
        }
        ViewMode::Smooth => {
            // Moving average with a small window relative to the decimation step.
            let step = n.div_ceil(columns).max(1);
            let half = (step / 2).max(2);
            let value_at = |i: usize| -> f32 {
                let lo = i.saturating_sub(half);
                let hi = (i + half + 1).min(n);
                let sum: u32 = samples[lo..hi].iter().map(|&b| b as u32).sum();
                sum as f32 / (hi - lo) as f32
            };
            stroke_clipped((0..n).step_by(step).map(|i| (x_at(i as f32), y_at(value_at(i)))).collect());
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
                    let y = (((samples[i] as f64 - PX_PER_DIV) / GRID_HEIGHT_PX) * size.height as f64) as f32;
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

// ── Time formatting utilities ──────────────────────────────────────────────

/// Compact time-duration formatting for scale labels: ms with 0.1 precision,
/// µs/ns with .0.
pub fn format_duration_ms_scale(ms: f64) -> String {
    if ms >= 1.0 {
        format!("{ms:.1}ms")
    } else if ms >= 0.001 {
        format!("{:.0}µs", ms * 1000.0)
    } else {
        format!("{:.0}ns", ms * 1_000_000.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 1000 × 800 plot at the canvas origin: fractions map to round pixels.
    const PLOT: Rectangle = Rectangle { x: 0.0, y: 0.0, width: 1000.0, height: 800.0 };
    const X: Option<(f32, f32)> = Some((0.2, 0.6)); // lines at x = 200, 600
    const Y: Option<(f32, f32)> = Some((0.25, 0.75)); // lines at y = 200, 600

    fn pick(x: f32, y: f32) -> Option<DragTarget> {
        pick_drag_target(Point::new(x, y), PLOT, X, Y)
    }

    #[test]
    fn press_near_crossing_grabs_both_lines() {
        let both = DragTarget { x: Some(Knob::High), y: Some(Knob::Low) };
        assert_eq!(pick(600.0, 200.0), Some(both));
        assert_eq!(pick(608.0, 208.0), Some(both)); // 11.3 px away
    }

    #[test]
    fn press_outside_crossing_radius_grabs_one_line() {
        // On the X line but 50 px below the crossing.
        assert_eq!(pick(601.0, 250.0), Some(DragTarget { x: Some(Knob::High), y: None }));
        // On the Y line, far from any X line.
        assert_eq!(pick(400.0, 598.0), Some(DragTarget { x: None, y: Some(Knob::High) }));
        // Just outside the radius diagonally: the closer line wins.
        assert_eq!(pick(609.0, 210.0), Some(DragTarget { x: Some(Knob::High), y: None }));
    }

    #[test]
    fn distance_is_compared_in_pixels_not_fractions() {
        // 30 px from the X line (0.03 of the width) and 40 px from the Y line
        // (0.05 of the height): the X line is nearer in pixels and in fractions.
        assert_eq!(pick(230.0, 240.0).unwrap(), DragTarget { x: Some(Knob::Low), y: None });
        // Wide plot: 50 px from the X line is 0.025 of 2000 px, 40 px from the
        // Y line is 0.05 of 800 px. Fractions would pick X; pixels pick Y.
        let wide = Rectangle { width: 2000.0, ..PLOT };
        let t = pick_drag_target(Point::new(450.0, 240.0), wide, X, Y).unwrap();
        assert_eq!(t, DragTarget { x: None, y: Some(Knob::Low) });
    }

    #[test]
    fn single_axis_and_disabled_cursors() {
        let p = Point::new(10.0, 10.0);
        assert_eq!(pick_drag_target(p, PLOT, None, None), None);
        assert_eq!(
            pick_drag_target(p, PLOT, X, None),
            Some(DragTarget { x: Some(Knob::Low), y: None })
        );
        assert_eq!(
            pick_drag_target(p, PLOT, None, Y),
            Some(DragTarget { x: None, y: Some(Knob::Low) })
        );
    }

    #[test]
    fn coincident_knobs_split_towards_the_pointer() {
        let same = Some((0.5, 0.5)); // both X lines at x = 500
        let right = pick_drag_target(Point::new(520.0, 0.0), PLOT, same, None).unwrap();
        assert_eq!(right.x, Some(Knob::High));
        let left = pick_drag_target(Point::new(480.0, 0.0), PLOT, same, None).unwrap();
        assert_eq!(left.x, Some(Knob::Low));
    }

    #[test]
    fn plot_fraction_clamps_outside_the_plot() {
        let plot = Rectangle { x: 80.0, y: 0.0, width: 400.0, height: 200.0 };
        assert_eq!(plot_fraction(Point::new(280.0, 50.0), plot), (0.5, 0.25));
        assert_eq!(plot_fraction(Point::new(-30.0, 900.0), plot), (0.0, 1.0));
    }

    #[test]
    fn plot_rect_reserves_scale_margins() {
        let size = Size::new(600.0, 400.0);
        let full = plot_rect(size, false, true, true);
        assert_eq!(full, Rectangle { x: 0.0, y: 0.0, width: 600.0, height: 400.0 });
        let one = plot_rect(size, true, true, false);
        assert_eq!(one.x, SCALE_MARGIN_LEFT);
        assert_eq!(one.height, 400.0 - SCALE_MARGIN_BOTTOM);
        let two = plot_rect(size, true, true, true);
        assert_eq!(two.x, SCALE_MARGIN_LEFT * 2.0);
        assert_eq!(two.width, 600.0 - SCALE_MARGIN_LEFT * 2.0);
    }
}

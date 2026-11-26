//! Oscilloscope-style graph canvas (iced::widget::canvas).
use crate::plot::{AxisLimits, ClipRect, View, X_LIMITS};
use crate::settings::ViewMode;
use dso_parser::{ADC_VALUE_MID, Capture};
use iced::widget::canvas::{self, Frame, Geometry, Path, Stroke, Text};
use iced::advanced::mouse::{click::Kind as ClickKind, Click as MouseClick};
use iced::{keyboard, mouse, Color, Point, Rectangle, Renderer, Size, Theme};

/// Drawable scope view backed by an optional `Capture`.
pub struct Scope<'a, Message = ()> {
    pub capture: Option<&'a Capture>,
    pub show_ch1: bool,
    pub show_ch2: bool,
    pub view_mode: ViewMode,
    /// Visible part of the capture on both axes (zoom and pan).
    pub view: View,
    /// User-positioned measurement cursors: X range (low, high) in 0.0..1.0
    /// of the whole capture (not of the visible window), so they stay on
    /// the signal while zooming.
    pub meas_cursor_x: Option<(f32, f32)>,
    /// User-positioned measurement cursors: Y range (low, high) in 0.0..1.0
    /// of the device screen height (not of the zoomed view).
    pub meas_cursor_y: Option<(f32, f32)>,
    /// Device-supplied X cursors (read-only, from capture data).
    pub device_cursor_x: Option<(f32, f32)>,
    /// Device-supplied Y cursors (read-only, from capture data).
    pub device_cursor_y: Option<(f32, f32)>,
    /// Show numeric axis scales on the graph.
    pub show_scales: bool,
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
    /// Message factory for zoom/pan changes (mouse wheel, right-drag).
    pub on_view_change: Option<fn(View) -> Message>,
    /// Message factory for a double-click: reset the zoom.
    pub on_view_reset: Option<fn(ViewReset) -> Message>,
}

/// Double-click on the graph: reset the zoom on both axes.
///
/// The first click of the double-click already moved the nearest cursor
/// line (a click places it). The cursor ranges from before that click are
/// carried here so the app can put them back: a double-click only resets
/// the zoom.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ViewReset {
    pub cursor_x: Option<(f32, f32)>,
    pub cursor_y: Option<(f32, f32)>,
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
    /// Keyboard modifiers: Shift+wheel pans, Option/Alt+wheel zooms Y.
    modifiers: keyboard::Modifiers,
    /// Right/middle-button pan in progress: press position and the view
    /// at that moment.
    panning: Option<(Point, View)>,
    /// Last left press, to detect double-clicks.
    last_click: Option<MouseClick>,
    /// Measurement cursor ranges just before the last single left press,
    /// restored by a double-click.
    before_click: ViewReset,
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
/// pointer position. `frac_x` is a fraction of the whole capture and
/// `frac_y` a fraction of the device screen height (the zoom is already
/// applied to both).
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

/// Nearest visible knob of a cursor pair to `pos` along one axis, and its
/// distance in pixels. `lo`/`hi` are fractions of the visible plot; a knob
/// outside `0..=1` is off-screen (zoomed out of view) and cannot be grabbed.
/// When both knobs sit on the same pixel, the knob on the side the pointer
/// is on wins, so the pair can be pulled apart in either direction.
fn nearest_knob(pos: f32, origin: f32, extent: f32, (lo, hi): (f32, f32)) -> Option<(Knob, f32)> {
    let candidate = |f: f32| {
        (0.0..=1.0).contains(&f).then(|| {
            let p = origin + f * extent;
            ((pos - p).abs(), p)
        })
    };
    match (candidate(lo), candidate(hi)) {
        (None, None) => None,
        (Some((d, _)), None) => Some((Knob::Low, d)),
        (None, Some((d, _))) => Some((Knob::High, d)),
        (Some((d_lo, p_lo)), Some((d_hi, _))) => {
            if d_lo < d_hi || (d_lo == d_hi && pos <= p_lo) {
                Some((Knob::Low, d_lo))
            } else {
                Some((Knob::High, d_hi))
            }
        }
    }
}

/// Decide which measurement-cursor lines a press at `pos` grabs.
///
/// Near the crossing of the nearest X and Y lines (within
/// [`CROSS_GRAB_RADIUS_PX`]) both lines are grabbed; otherwise only the
/// single nearest line, measured in pixels. `cursor_x` and `cursor_y` are
/// fractions of the visible plot. Returns `None` when no measurement cursor
/// line is enabled and on screen.
pub fn pick_drag_target(
    pos: Point,
    plot: Rectangle,
    cursor_x: Option<(f32, f32)>,
    cursor_y: Option<(f32, f32)>,
) -> Option<DragTarget> {
    let x = cursor_x.and_then(|r| nearest_knob(pos.x, plot.x, plot.width, r));
    let y = cursor_y.and_then(|r| nearest_knob(pos.y, plot.y, plot.height, r));
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

/// Zoom factor per wheel notch (line-based scrolling, mouse wheels).
const ZOOM_PER_LINE: f64 = 1.25;
/// Pixel-based scrolling (trackpads) that doubles or halves the zoom.
const ZOOM_PIXELS_PER_DOUBLING: f64 = 200.0;
/// Fraction of the visible window panned per wheel notch.
const PAN_PER_LINE: f64 = 0.1;
/// Zooming in stops when this many samples span the plot width.
const MIN_VISIBLE_SAMPLES: f64 = 10.0;
/// Voltage zoom-in stops when this many ADC levels span the plot height.
const MIN_VISIBLE_LEVELS: f64 = 8.0;

/// Voltage axis: the whole ADC range 0..=255, which reaches beyond the
/// device screen (values 25..=225, unit coordinates 0..=1), so zooming out
/// shows the parts of the signal the oscilloscope clipped off its screen.
pub const Y_LIMITS: AxisLimits = AxisLimits {
    lo: (0.0 - PX_PER_DIV) / GRID_HEIGHT_PX,
    hi: (255.0 - PX_PER_DIV) / GRID_HEIGHT_PX,
};
/// Smallest voltage-axis span.
const Y_MIN_SPAN: f64 = MIN_VISIBLE_LEVELS / GRID_HEIGHT_PX;

/// Smallest time-axis span for a capture of `n` samples.
pub fn min_view_span(n: usize) -> f64 {
    if n > 1 {
        (MIN_VISIBLE_SAMPLES / (n - 1) as f64).min(1.0)
    } else {
        1.0
    }
}

/// New view after a wheel event over the graph.
///
/// Vertical scrolling zooms around `anchor` (pointer position as fractions
/// of the plot): scrolling up/away zooms in. It zooms the time axis, or the
/// voltage axis when `zoom_y` is set (Option/Alt held, or the pointer over
/// the voltage labels). Horizontal scrolling (trackpad swipe, tilt wheel,
/// or Shift + vertical wheel) pans the time axis; with a trackpad the signal
/// follows the fingers.
pub fn wheel_view(
    view: View,
    delta: mouse::ScrollDelta,
    shift: bool,
    zoom_y: bool,
    anchor: (f64, f64),
    plot_width: f32,
    min_span_x: f64,
) -> View {
    let (dx, dy, pixels) = match delta {
        mouse::ScrollDelta::Lines { x, y } => (x as f64, y as f64, false),
        mouse::ScrollDelta::Pixels { x, y } => (x as f64, y as f64, true),
    };
    // Shift turns a plain vertical wheel into horizontal panning (macOS
    // already reports Shift+wheel as horizontal).
    let (dx, dy) = if shift && dx == 0.0 { (dy, 0.0) } else { (dx, dy) };
    let mut v = view;
    if dy != 0.0 {
        let factor = if pixels {
            2f64.powf(-dy / ZOOM_PIXELS_PER_DOUBLING)
        } else {
            ZOOM_PER_LINE.powf(-dy)
        };
        if zoom_y {
            v.y = v.y.zoom_at(anchor.1, factor, Y_MIN_SPAN, Y_LIMITS);
        } else {
            v.x = v.x.zoom_at(anchor.0, factor, min_span_x, X_LIMITS);
        }
    }
    if dx != 0.0 {
        let delta = if pixels {
            -dx / f64::from(plot_width.max(1.0)) * v.x.span()
        } else {
            -dx * PAN_PER_LINE * v.x.span()
        };
        v.x = v.x.pan(delta, X_LIMITS);
    }
    v
}

/// New view while dragging with the right/middle button: the signal
/// follows the pointer on both axes. `moved` is the pointer movement since
/// the press (pixels) and `start` the view at the press.
pub fn drag_view(start: View, moved: (f32, f32), plot: Rectangle) -> View {
    let (mx, my) = moved;
    let dx = -f64::from(mx) / f64::from(plot.width.max(1.0)) * start.x.span();
    let dy = -f64::from(my) / f64::from(plot.height.max(1.0)) * start.y.span();
    View {
        x: start.x.pan(dx, X_LIMITS),
        y: start.y.pan(dy, Y_LIMITS),
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
/// Top and bottom edges of the device screen, shown when the voltage axis
/// is zoomed out past them.
const SCREEN_EDGE: Color = Color::from_rgb(0.75, 0.55, 0.20);

/// Zoom indicator text: the zoom factor of each zoomed axis, e.g.
/// `X ×4.0  Y ×0.8`.
fn zoom_indicator(view: View) -> String {
    let mut parts = Vec::new();
    if !view.x.is_unit() {
        parts.push(format!("X ×{:.1}", 1.0 / view.x.span()));
    }
    if !view.y.is_unit() {
        parts.push(format!("Y ×{:.1}", 1.0 / view.y.span()));
    }
    parts.join("  ")
}

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
        let plot = plot_rect(bounds.size(), self.show_scales, self.has_ch1, self.has_ch2);
        let view = self.view;
        let step = |on_cursor_drag: fn(CursorDrag) -> Message, target: DragTarget, pos: Point| {
            let (fx, fy) = plot_fraction(pos, plot);
            let frac_x = view.x.to_data(f64::from(fx)) as f32;
            let frac_y = view.y.to_data(f64::from(fy)) as f32;
            canvas::Action::publish(on_cursor_drag(CursorDrag { target, frac_x, frac_y }))
                .and_capture()
        };

        match event {
            canvas::Event::Keyboard(keyboard::Event::ModifiersChanged(modifiers)) => {
                state.modifiers = *modifiers;
                None
            }
            canvas::Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                let on_view_change = self.on_view_change?;
                let cap = self.capture?;
                let pos = cursor.position_in(bounds)?;
                let (ax, ay) = plot_fraction(pos, plot);
                // Option/Alt, or the pointer over the voltage labels left of
                // the plot, zooms the voltage axis. Over the time labels
                // below the plot the wheel zooms time, as over the plot.
                let zoom_y = state.modifiers.alt() || pos.x < plot.x;
                let new_view = wheel_view(
                    view,
                    *delta,
                    state.modifiers.shift(),
                    zoom_y,
                    (f64::from(ax), f64::from(ay)),
                    plot.width,
                    min_view_span(cap.sample_count()),
                );
                // Capture the wheel even at the zoom limits so it does not
                // scroll a surrounding container.
                Some(if new_view == view {
                    canvas::Action::capture()
                } else {
                    canvas::Action::publish(on_view_change(new_view)).and_capture()
                })
            }
            canvas::Event::Mouse(mouse::Event::ButtonPressed(
                mouse::Button::Right | mouse::Button::Middle,
            )) => {
                // Right/middle-drag pans both axes.
                self.on_view_change?;
                self.capture?;
                let pos = cursor.position_in(bounds)?;
                state.panning = Some((pos, view));
                Some(canvas::Action::capture())
            }
            canvas::Event::Mouse(mouse::Event::ButtonReleased(
                mouse::Button::Right | mouse::Button::Middle,
            )) => state
                .panning
                .take()
                .map(|_| canvas::Action::request_redraw()),
            canvas::Event::Mouse(mouse::Event::CursorMoved { position })
                if state.panning.is_some() =>
            {
                let on_view_change = self.on_view_change?;
                let (start_pos, start_view) = state.panning?;
                let pos = Point::new(position.x - bounds.x, position.y - bounds.y);
                let moved = (pos.x - start_pos.x, pos.y - start_pos.y);
                let new_view = drag_view(start_view, moved, plot);
                Some(if new_view == view {
                    canvas::Action::capture()
                } else {
                    canvas::Action::publish(on_view_change(new_view)).and_capture()
                })
            }
            canvas::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                // Only a press inside the canvas counts.
                let pos = cursor.position_in(bounds)?;
                let click = MouseClick::new(pos, mouse::Button::Left, state.last_click);
                state.last_click = Some(click);
                if click.kind() == ClickKind::Single {
                    state.before_click = ViewReset {
                        cursor_x: self.meas_cursor_x,
                        cursor_y: self.meas_cursor_y,
                    };
                } else if let Some(on_view_reset) = self.on_view_reset {
                    // Double-click: reset the zoom and undo the cursor move
                    // made by the first click.
                    state.dragging = None;
                    return Some(
                        canvas::Action::publish(on_view_reset(state.before_click)).and_capture(),
                    );
                }
                let on_cursor_drag = self.on_cursor_drag?;
                let target =
                    pick_drag_target(pos, plot, self.screen_cursor_x(), self.screen_cursor_y())?;
                state.dragging = Some(target);
                Some(step(on_cursor_drag, target, pos))
            }
            canvas::Event::Mouse(mouse::Event::CursorMoved { position }) => {
                let on_cursor_drag = self.on_cursor_drag?;
                // Keep following the pointer after it leaves the canvas; the
                // position is clamped to the plot edge.
                let target = state.dragging?;
                let pos = Point::new(position.x - bounds.x, position.y - bounds.y);
                Some(step(on_cursor_drag, target, pos))
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
        if state.panning.is_some() {
            return mouse::Interaction::Grabbing;
        }
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
        let cursor_x = self.screen_cursor_x();
        let cursor_y = self.screen_cursor_y();
        let Some(target) = pick_drag_target(pos, plot, cursor_x, cursor_y)
        else {
            return mouse::Interaction::default();
        };
        // A press anywhere moves the nearest line; show the resize pointer
        // only when the pointer is actually on a line or the crossing.
        let near = |range: Option<(f32, f32)>, p: f32, o: f32, e: f32| {
            range
                .and_then(|r| nearest_knob(p, o, e, r))
                .is_some_and(|(_, d)| d <= LINE_HOVER_PX)
        };
        let on_x = near(cursor_x, pos.x, plot.x, plot.width);
        let on_y = near(cursor_y, pos.y, plot.y, plot.height);
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
                    draw_trace(frame, samples, color, self.view_mode, plot, self.view);
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
        // X positions follow the zoom; a line outside the view is not drawn
        // and the shaded band is cut at the plot edges.
        if let Some((sl, sr)) = self.screen_cursor_x() {
            let h = gh;
            let (band_l, band_r) = (sl.max(0.0), sr.min(1.0));
            if band_l <= band_r {
                let region = Path::rectangle(
                    Point::new(gx + band_l * gw, 0.0),
                    Size::new((band_r - band_l) * gw, h),
                );
                frame.fill(&region, MEAS_FILL);
            }
            let stroke = Stroke::default().with_color(MEAS_CURSOR).with_width(1.0);
            for s in [sl, sr].into_iter().filter(|s| (0.0..=1.0).contains(s)) {
                let px = gx + s * gw;
                frame.stroke(&Path::line(Point::new(px, 0.0), Point::new(px, h)), stroke);
            }
        }
        // Y positions follow the voltage zoom the same way.
        if let Some((su, sl)) = self.screen_cursor_y() {
            let w = frame.width();
            let h = gh;
            let (band_u, band_l) = (su.max(0.0), sl.min(1.0));
            if band_u <= band_l {
                let region = Path::rectangle(
                    Point::new(0.0, band_u * h),
                    Size::new(w, (band_l - band_u) * h),
                );
                frame.fill(&region, MEAS_FILL);
            }
            let stroke = Stroke::default().with_color(MEAS_CURSOR).with_width(1.0);
            for s in [su, sl].into_iter().filter(|s| (0.0..=1.0).contains(s)) {
                let py = s * h;
                frame.stroke(&Path::line(Point::new(0.0, py), Point::new(w, py)), stroke);
            }
        }

        // Device cursors (read-only, dashed-look using thinner line, different colour).
        let dev_stroke = Stroke::default().with_color(DEV_CURSOR).with_width(1.0);
        if let Some((a, b)) = self.device_cursor_x {
            let h = gh;
            // Show time values next to device X cursors that are in view.
            let total_time_ms = T_CELLS as f64 * self.t_per_div_ms;
            for x in [a, b] {
                let s = self.view.x.to_screen(f64::from(x.clamp(0.0, 1.0))) as f32;
                if !(0.0..=1.0).contains(&s) {
                    continue;
                }
                let px = gx + s * gw;
                frame.stroke(
                    &Path::line(Point::new(px, 0.0), Point::new(px, h)),
                    dev_stroke,
                );
                let t_ms = f64::from(x) * total_time_ms;
                frame.fill_text(dev_label(format_duration_ms_scale(t_ms), Point::new(px + 2.0, 12.0)));
            }
            // Also show Δt between cursors.
            let dt_ms = (b - a).abs() as f64 * total_time_ms;
            let dt_label = format!("Δt={}", format_duration_ms_scale(dt_ms));
            frame.fill_text(dev_label(dt_label, Point::new(gx + 4.0, 22.0)));
        }
        if let Some((a, b)) = self.device_cursor_y {
            let w = frame.width();
            let h = gh;
            let to_screen = |y: f32| self.view.y.to_screen(f64::from(y.clamp(0.0, 1.0))) as f32;
            for y in [a, b] {
                let s = to_screen(y);
                if !(0.0..=1.0).contains(&s) {
                    continue;
                }
                let py = s * h;
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
            for (y, v) in [(a, v_a), (b, v_b)] {
                let s = to_screen(y);
                if (0.0..=1.0).contains(&s) {
                    frame.fill_text(dev_label(format!("{v:.2}V"), Point::new(gx + 4.0, s * h + 2.0)));
                }
            }
        }

        // Axis scales (sample numbers on X, voltage on Y per-channel)
        // When the voltage axis is zoomed out past the device screen, mark
        // the screen's top and bottom edges: beyond them the oscilloscope
        // itself clipped the signal.
        for edge in [0.0, 1.0] {
            let s = self.view.y.to_screen(edge) as f32;
            if s > 0.0 && s < 1.0 {
                let py = s * gh;
                frame.stroke(
                    &Path::line(Point::new(gx, py), Point::new(gx + gw, py)),
                    Stroke::default().with_color(SCREEN_EDGE).with_width(1.0),
                );
            }
        }

        if !self.view.is_unit() {
            frame.fill_text(Text {
                content: zoom_indicator(self.view),
                position: Point::new(gx + gw - 6.0, 6.0),
                color: SCALE_COLOR,
                size: 11.0.into(),
                align_x: iced::alignment::Horizontal::Right.into(),
                ..Text::default()
            });
        }

        if self.show_scales {
            draw_scales(
                &mut frame,
                self.view,
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
        for a in [self.view.x, self.view.y] {
            (a.lo.to_bits(), a.hi.to_bits()).hash(&mut h);
        }
        for v in [plot.x, plot.y, plot.width, plot.height] {
            v.to_bits().hash(&mut h);
        }
        h.finish()
    }

    /// Measurement X cursors as fractions of the visible plot (outside
    /// `0..=1` when zoomed out of view).
    fn screen_cursor_x(&self) -> Option<(f32, f32)> {
        self.meas_cursor_x.map(|(lo, hi)| {
            let s = |f: f32| self.view.x.to_screen(f64::from(f)) as f32;
            (s(lo), s(hi))
        })
    }

    /// Measurement Y cursors as fractions of the visible plot height.
    fn screen_cursor_y(&self) -> Option<(f32, f32)> {
        self.meas_cursor_y.map(|(lo, hi)| {
            let s = |f: f32| self.view.y.to_screen(f64::from(f)) as f32;
            (s(lo), s(hi))
        })
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
    view: View,
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
    // With zoom, the 12 divisions cover only the visible window.
    let total_time_ms = T_CELLS as f64 * t_per_div_ms;
    let step_ms = view.x.span() * total_time_ms / cols as f64;
    let dx = gw / cols as f32;
    for i in 0..=cols {
        let frac = view.x.to_data(i as f64 / cols as f64);
        let time_ms = frac * total_time_ms;
        let x = margin_left + i as f32 * dx;
        let content = format_time_label(time_ms, step_ms);
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

    // With voltage zoom the grid lines stay put and the labels follow the
    // visible window; more decimals appear once 0.01 V no longer tells
    // neighbouring lines apart.
    let mut label_ch = |v_per_div: f64, v_offset: f64, color: Color, x_off: f32| {
        let step_v = v_per_div * view.y.span();
        let decimals = if step_v > 0.0 {
            ((-step_v.log10()).ceil() as i32).clamp(2, 6) as usize
        } else {
            2
        };
        for i in 0..=rows {
            // Grid line i at unit coordinate f holds ADC value f × 200 + 25.
            let f = view.y.to_data(i as f64 / rows as f64);
            let v = f * GRID_HEIGHT_PX + PX_PER_DIV;
            let voltage = v_offset + (ADC_VALUE_MID as f64 - v) * v_per_div / PX_PER_DIV;
            let y = (i as f32 / rows as f32) * graph_height;
            let label = Text {
                content: format!("{voltage:.decimals$}V"),
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

/// Draw one channel's trace into `plot`, showing the part of the capture
/// selected by `view`.
///
/// Only the visible samples (plus one on each side, so lines reach the
/// edges) are processed. Line and dot modes keep every sample's
/// contribution: with more visible samples than pixel columns, each column
/// draws the min/max of its samples (line) or each distinct value once
/// (dot), so a single-sample spike stays visible. Smooth mode is a
/// deliberate visual filter and averages.
fn draw_trace(
    frame: &mut Frame,
    samples: &[u8],
    color: Color,
    mode: ViewMode,
    plot: Rectangle,
    view: View,
) {
    if samples.is_empty() {
        return;
    }
    let n = samples.len();
    let last = (n - 1).max(1) as f64;
    let stroke = Stroke::default().with_color(color).with_width(1.2);
    let columns = (plot.width as usize).max(2);

    // Sample index (possibly fractional) -> x pixel: sample 0 is the start
    // and sample n-1 the end of the capture; the view selects the window.
    let x_at = |i: f64| (f64::from(plot.x) + view.x.to_screen(i / last) * f64::from(plot.width)) as f32;
    // Capture values use screen-coordinate convention: low value = top
    // of screen = positive voltage, high value = bottom = negative voltage.
    // Use device pixel density: GRID_HEIGHT_PX = V_CELLS × PX_PER_DIV.
    // The voltage view maps that screen fraction into the visible window.
    let y_at = |v: f32| {
        let f = (f64::from(v) - PX_PER_DIV) / GRID_HEIGHT_PX;
        (f64::from(plot.y) + view.y.to_screen(f) * f64::from(plot.height)) as f32
    };

    // Visible sample range, widened to the neighbouring samples so the
    // trace runs to the plot edges instead of stopping short of them.
    let i0 = ((view.x.lo * last).floor() as usize).min(n - 1);
    let i1 = ((view.x.hi * last).ceil() as usize).min(n - 1);
    let visible = &samples[i0..=i1];
    // Representative x of a column: the middle of its sample range.
    let column_x = |s: &crate::plot::ColumnSpan| x_at(i0 as f64 + (s.start + s.end - 1) as f64 / 2.0);
    let clip = ClipRect {
        left: plot.x,
        right: plot.x + plot.width,
        top: plot.y,
        bottom: plot.y + plot.height,
    };

    // Stroke the parts of a polyline that are inside the plot area.
    let mut stroke_clipped = |points: Vec<(f32, f32)>| {
        let runs = crate::plot::clip_polyline(&points, clip);
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
    let each_sample = || {
        visible
            .iter()
            .enumerate()
            .map(move |(k, &v)| (x_at((i0 + k) as f64), y_at(v as f32)))
    };

    match mode {
        ViewMode::Dot => {
            const R: f32 = 1.2;
            let mut dots: Vec<(f32, f32)> = Vec::new();
            if visible.len() <= columns {
                dots.extend(each_sample());
            } else {
                for span in crate::plot::column_spans(visible, columns) {
                    let x = column_x(&span);
                    dots.extend(
                        crate::plot::distinct_values(&visible[span.start..span.end])
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
            for &(x, y) in dots.iter().filter(|&&d| clip.contains(d)) {
                frame.fill_rectangle(Point::new(x - R, y - R), size, color);
            }
        }
        ViewMode::Line => {
            let points = if visible.len() <= columns {
                each_sample().collect()
            } else {
                crate::plot::column_spans(visible, columns)
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
            let step = visible.len().div_ceil(columns).max(1);
            let half = (step / 2).max(2);
            let value_at = |i: usize| -> f32 {
                let lo = i.saturating_sub(half);
                let hi = (i + half + 1).min(n);
                let sum: u32 = samples[lo..hi].iter().map(|&b| b as u32).sum();
                sum as f32 / (hi - lo) as f32
            };
            stroke_clipped(
                (i0..=i1)
                    .step_by(step)
                    .map(|i| (x_at(i as f64), y_at(value_at(i))))
                    .collect(),
            );
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

/// Time-axis label for `ms`, with enough decimals to tell labels `step_ms`
/// apart (needed when zoomed in). The unit follows the value, as in
/// [`format_duration_ms_scale`].
pub fn format_time_label(ms: f64, step_ms: f64) -> String {
    let (scale, unit) = if ms.abs() >= 1.0 {
        (1.0, "ms")
    } else if ms.abs() >= 1e-3 {
        (1e3, "µs")
    } else {
        (1e6, "ns")
    };
    let step = step_ms.abs() * scale;
    let decimals = if step > 0.0 {
        (-step.log10()).ceil().clamp(0.0, 6.0) as usize
    } else {
        0
    };
    format!("{:.*}{unit}", decimals, ms * scale)
}

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
    fn off_screen_cursor_lines_cannot_be_grabbed() {
        // Zoomed so that the low X line is left of the view (-0.5) and the
        // high one is on screen at 0.3.
        let x = Some((-0.5, 0.3));
        let t = pick_drag_target(Point::new(10.0, 0.0), PLOT, x, None).unwrap();
        assert_eq!(t.x, Some(Knob::High));
        // Both X lines out of view: a press does nothing on X.
        assert_eq!(pick_drag_target(Point::new(10.0, 0.0), PLOT, Some((-0.5, 1.5)), None), None);
        // ...but still reaches a Y line.
        let t = pick_drag_target(Point::new(10.0, 0.0), PLOT, Some((-0.5, 1.5)), Y).unwrap();
        assert_eq!(t, DragTarget { x: None, y: Some(Knob::Low) });
    }

    use crate::plot::AxisView;

    const UP: mouse::ScrollDelta = mouse::ScrollDelta::Lines { x: 0.0, y: 1.0 };
    const DOWN: mouse::ScrollDelta = mouse::ScrollDelta::Lines { x: 0.0, y: -1.0 };

    /// Wheel over the plot at window fractions `(ax, ay)`, 1000 px wide.
    fn wheel(v: View, delta: mouse::ScrollDelta, shift: bool, zoom_y: bool, at: (f64, f64)) -> View {
        wheel_view(v, delta, shift, zoom_y, at, 1000.0, 0.0)
    }

    #[test]
    fn wheel_up_zooms_in_around_the_pointer() {
        let v = wheel(View::UNIT, UP, false, false, (0.25, 0.5));
        assert!((v.x.span() - 0.8).abs() < 1e-12);
        assert!(v.y.is_unit());
        // The capture point under the pointer stays under the pointer.
        assert!((v.x.to_data(0.25) - 0.25).abs() < 1e-12);
        // Scrolling back down returns to the full view.
        assert!(wheel(v, DOWN, false, false, (0.25, 0.5)).is_unit());
    }

    #[test]
    fn option_wheel_zooms_the_voltage_axis_around_the_pointer() {
        let v = wheel(View::UNIT, UP, false, true, (0.5, 0.25));
        assert!(v.x.is_unit());
        assert!((v.y.span() - 0.8).abs() < 1e-12);
        assert!((v.y.to_data(0.25) - 0.25).abs() < 1e-12);
    }

    #[test]
    fn voltage_zoom_out_goes_past_the_screen_to_the_adc_range() {
        let mut v = View::UNIT;
        for _ in 0..10 {
            v = wheel(v, DOWN, false, true, (0.5, 0.5));
        }
        // ADC 0..=255 in screen units (screen = 25..=225).
        assert!((v.y.lo - (-0.125)).abs() < 1e-12);
        assert!((v.y.hi - 1.15).abs() < 1e-12);
        // Zooming back in passes through the device screen view exactly.
        let mut seen_unit = false;
        for _ in 0..10 {
            v = wheel(v, UP, false, true, (0.5, 0.5));
            seen_unit |= v.y.is_unit();
        }
        assert!(seen_unit);
    }

    #[test]
    fn voltage_zoom_in_stops_at_eight_adc_levels() {
        let mut v = View::UNIT;
        for _ in 0..40 {
            v = wheel(v, UP, false, true, (0.5, 0.5));
        }
        assert!((v.y.span() * GRID_HEIGHT_PX - 8.0).abs() < 1e-9);
    }

    #[test]
    fn trackpad_pixels_zoom_smoothly() {
        let v = wheel(View::UNIT, mouse::ScrollDelta::Pixels { x: 0.0, y: 200.0 }, false, false, (0.5, 0.5));
        assert!((v.x.span() - 0.5).abs() < 1e-12);
    }

    #[test]
    fn horizontal_scroll_and_shift_wheel_pan() {
        let v = View { x: AxisView { lo: 0.4, hi: 0.6 }, y: AxisView::UNIT };
        // Trackpad swipe of 100 px on a 1000 px plot moves by 1/10 of the view.
        let swiped = wheel(v, mouse::ScrollDelta::Pixels { x: -100.0, y: 0.0 }, false, false, (0.5, 0.5));
        assert!((swiped.x.lo - 0.42).abs() < 1e-12 && (swiped.x.span() - 0.2).abs() < 1e-12);
        // Shift + wheel pans instead of zooming.
        let shifted = wheel(v, DOWN, true, false, (0.5, 0.5));
        assert!((shifted.x.span() - 0.2).abs() < 1e-12);
        assert!((shifted.x.lo - 0.42).abs() < 1e-12);
    }

    #[test]
    fn right_drag_moves_the_signal_with_the_pointer() {
        let plot = Rectangle { x: 0.0, y: 0.0, width: 1000.0, height: 800.0 };
        let start = View {
            x: AxisView { lo: 0.4, hi: 0.6 },
            y: AxisView { lo: 0.25, hi: 0.75 },
        };
        // Dragging right by 100 px shows earlier samples (view moves left by
        // 100/1000 of its 0.2 span); dragging down by 80 px shows higher
        // positions (view moves up by 80/800 of its 0.5 span).
        let v = drag_view(start, (100.0, 80.0), plot);
        assert!((v.x.lo - 0.38).abs() < 1e-12);
        assert!((v.y.lo - 0.2).abs() < 1e-12);
        // Unzoomed time axis cannot pan; the voltage axis stops at its limits.
        let v = drag_view(View::UNIT, (300.0, 10_000.0), plot);
        assert!(v.x.is_unit());
        assert!((v.y.lo - Y_LIMITS.lo).abs() < 1e-12);
    }

    #[test]
    fn zoom_in_stops_at_min_visible_samples() {
        let mut v = View::UNIT;
        let up = mouse::ScrollDelta::Lines { x: 0.0, y: 10.0 };
        for _ in 0..20 {
            v = wheel_view(v, up, false, false, (0.5, 0.5), 1000.0, min_view_span(60_000));
        }
        assert!((v.x.span() * 59_999.0 - 10.0).abs() < 1e-6);
        assert_eq!(min_view_span(5), 1.0);
    }

    #[derive(Debug, Clone, PartialEq)]
    enum TestMsg {
        Drag(CursorDrag),
        View(View),
        Reset(ViewReset),
    }

    fn scope_for_clicks(cap: &Capture, cursor_x: Option<(f32, f32)>) -> Scope<'_, TestMsg> {
        Scope {
            capture: Some(cap),
            show_ch1: true,
            show_ch2: false,
            view_mode: ViewMode::Line,
            view: View::UNIT,
            meas_cursor_x: cursor_x,
            meas_cursor_y: None,
            device_cursor_x: None,
            device_cursor_y: None,
            show_scales: false,
            has_ch1: true,
            has_ch2: false,
            v_per_div_ch1: 1.0,
            v_per_div_ch2: 1.0,
            t_per_div_ms: 1.0,
            v_offset_ch1: 0.0,
            v_offset_ch2: 0.0,
            active_channel: 0,
            on_cursor_drag: Some(TestMsg::Drag),
            on_view_change: Some(TestMsg::View),
            on_view_reset: Some(TestMsg::Reset),
        }
    }

    /// Send a left press at `pos` through the canvas and return its message.
    fn press(scope: &Scope<'_, TestMsg>, state: &mut ScopeState, pos: Point) -> Option<TestMsg> {
        use canvas::Program;
        let event = canvas::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left));
        let action = Program::<TestMsg>::update(scope, state, &event, PLOT, mouse::Cursor::Available(pos))?;
        action.into_inner().0
    }

    #[test]
    fn double_click_resets_the_zoom_and_keeps_the_cursors() {
        let cap = Capture { ch1: vec![128; 100], ch2: None };
        let mut state = ScopeState::default();
        let pos = Point::new(500.0, 400.0);

        // First click: places the nearest X cursor line, as before.
        let scope = scope_for_clicks(&cap, Some((0.2, 0.6)));
        assert!(matches!(press(&scope, &mut state, pos), Some(TestMsg::Drag(_))));

        // Second click right after: reset, restoring the cursors from
        // before the first click (the app has moved the line meanwhile).
        let moved = scope_for_clicks(&cap, Some((0.2, 0.5)));
        let reset = press(&moved, &mut state, pos);
        assert_eq!(
            reset,
            Some(TestMsg::Reset(ViewReset { cursor_x: Some((0.2, 0.6)), cursor_y: None }))
        );
        assert!(state.dragging.is_none());
    }

    #[test]
    fn clicks_far_apart_are_not_a_double_click() {
        let cap = Capture { ch1: vec![128; 100], ch2: None };
        let mut state = ScopeState::default();
        let scope = scope_for_clicks(&cap, Some((0.2, 0.6)));
        assert!(matches!(press(&scope, &mut state, Point::new(100.0, 100.0)), Some(TestMsg::Drag(_))));
        assert!(matches!(press(&scope, &mut state, Point::new(700.0, 100.0)), Some(TestMsg::Drag(_))));
    }

    #[test]
    fn zoom_indicator_names_the_zoomed_axes() {
        assert_eq!(zoom_indicator(View::UNIT), "");
        let v = View { x: AxisView { lo: 0.0, hi: 0.25 }, y: AxisView::UNIT };
        assert_eq!(zoom_indicator(v), "X ×4.0");
        let v = View { x: AxisView { lo: 0.0, hi: 0.25 }, y: AxisView { lo: -0.125, hi: 1.125 } };
        assert_eq!(zoom_indicator(v), "X ×4.0  Y ×0.8");
    }

    #[test]
    fn time_labels_keep_the_old_look_and_resolve_zoom_steps() {
        // Unzoomed 0.2 ms/div labels are unchanged.
        assert_eq!(format_time_label(0.0, 0.2), "0ns");
        assert_eq!(format_time_label(0.2, 0.2), "200µs");
        assert_eq!(format_time_label(1.2, 0.2), "1.2ms");
        // Zoomed in to 0.5 µs/div: neighbouring labels differ.
        assert_eq!(format_time_label(1.0005, 0.0005), "1.0005ms");
        assert_ne!(format_time_label(0.5120, 0.0005), format_time_label(0.5125, 0.0005));
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

//! Renderer-neutral waveform helpers shared by the screen canvas
//! (`canvas.rs`) and the PNG export (`export_image.rs`).
//!
//! When a trace has more samples than the output has pixel columns, both
//! renderers must keep every sample's contribution: a single-sample spike
//! must stay visible. [`column_spans`] reduces the samples to one min/max
//! span per output column instead of picking every N-th sample.

/// The samples that fall into one output pixel column.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ColumnSpan {
    /// Index of the first sample in this column.
    pub start: usize,
    /// One past the index of the last sample in this column.
    pub end: usize,
    /// Smallest sample value in the column.
    pub min: u8,
    /// Largest sample value in the column.
    pub max: u8,
    /// `true` when the column's first sample is not greater than its last,
    /// i.e. the signal runs from `min` to `max` within the column.
    pub rising: bool,
}

impl ColumnSpan {
    /// The column's extremes in temporal order, so that a polyline drawn
    /// through consecutive columns follows the signal's direction.
    pub fn ordered(&self) -> [u8; 2] {
        if self.rising {
            [self.min, self.max]
        } else {
            [self.max, self.min]
        }
    }
}

/// Split `samples` into `columns` consecutive, non-empty groups and return
/// the min/max span of each. Every sample belongs to exactly one column.
///
/// Returns an empty vector when there are no samples or no columns. When
/// there are fewer samples than columns, each sample gets its own column
/// and the result has `samples.len()` entries.
pub fn column_spans(samples: &[u8], columns: usize) -> Vec<ColumnSpan> {
    let n = samples.len();
    if n == 0 || columns == 0 {
        return Vec::new();
    }
    let columns = columns.min(n);
    (0..columns)
        .map(|c| {
            let start = c * n / columns;
            let end = ((c + 1) * n / columns).max(start + 1).min(n);
            let col = &samples[start..end];
            let (min, max) = col
                .iter()
                .fold((u8::MAX, u8::MIN), |(lo, hi), &v| (lo.min(v), hi.max(v)));
            ColumnSpan {
                start,
                end,
                min,
                max,
                rising: col[0] <= col[col.len() - 1],
            }
        })
        .collect()
}

/// Distinct sample values inside `samples`, in ascending order.
///
/// Used by dot mode: in a column holding many samples, dots with the same
/// value land on the same pixel, so drawing each distinct value once shows
/// exactly the same picture with far fewer shapes.
pub fn distinct_values(samples: &[u8]) -> impl Iterator<Item = u8> {
    let mut seen = [false; 256];
    for &v in samples {
        seen[v as usize] = true;
    }
    (0..=255u8).filter(move |&v| seen[v as usize])
}

/// Axis-aligned clip rectangle (y grows downward).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClipRect {
    pub left: f32,
    pub right: f32,
    pub top: f32,
    pub bottom: f32,
}

impl ClipRect {
    pub fn contains(&self, p: (f32, f32)) -> bool {
        (self.left..=self.right).contains(&p.0) && (self.top..=self.bottom).contains(&p.1)
    }
}

/// Clip a polyline to `rect`.
///
/// Returns the visible runs: each run is a polyline wholly inside the
/// rectangle. A segment that crosses an edge is cut exactly at the edge, so
/// the trace visibly reaches the border before it leaves the screen, like on
/// the oscilloscope. Clipping is done on the geometry itself so it does not
/// depend on renderer clip support.
pub fn clip_polyline(points: &[(f32, f32)], rect: ClipRect) -> Vec<Vec<(f32, f32)>> {
    let mut runs = Vec::new();
    let mut run: Vec<(f32, f32)> = Vec::new();
    if let [only] = points {
        if rect.contains(*only) {
            runs.push(vec![*only]);
        }
        return runs;
    }
    for seg in points.windows(2) {
        match clip_segment(seg[0], seg[1], rect) {
            Some((a, b)) => {
                // A new run starts when the segment does not continue the
                // previous one (the trace left the rectangle in between).
                if run.last() != Some(&a) {
                    if run.len() >= 2 {
                        runs.push(std::mem::take(&mut run));
                    }
                    run.clear();
                    run.push(a);
                }
                run.push(b);
            }
            None => {
                if run.len() >= 2 {
                    runs.push(std::mem::take(&mut run));
                }
                run.clear();
            }
        }
    }
    if run.len() >= 2 {
        runs.push(run);
    }
    runs
}

/// The part of segment `p → q` inside `rect`, if any (Liang–Barsky).
/// Endpoints that are already inside are returned unchanged (bit-exact), so
/// consecutive segments can be joined by comparing points.
fn clip_segment(p: (f32, f32), q: (f32, f32), rect: ClipRect) -> Option<((f32, f32), (f32, f32))> {
    let (dx, dy) = (q.0 - p.0, q.1 - p.1);
    let (mut t0, mut t1) = (0.0f32, 1.0f32);
    // For each edge: (-direction, distance from p to the edge, inward).
    for (pk, qk) in [
        (-dx, p.0 - rect.left),
        (dx, rect.right - p.0),
        (-dy, p.1 - rect.top),
        (dy, rect.bottom - p.1),
    ] {
        if pk == 0.0 {
            // Parallel to this edge: entirely outside or not limited by it.
            if qk < 0.0 {
                return None;
            }
        } else {
            let r = qk / pk;
            if pk < 0.0 {
                t0 = t0.max(r);
            } else {
                t1 = t1.min(r);
            }
            if t0 > t1 {
                return None;
            }
        }
    }
    let at = |t: f32| {
        if t == 0.0 {
            p
        } else if t == 1.0 {
            q
        } else {
            // A cut point lies on an edge; clamp it so that rounding cannot
            // leave it just outside the rectangle.
            (
                (p.0 + dx * t).clamp(rect.left, rect.right),
                (p.1 + dy * t).clamp(rect.top, rect.bottom),
            )
        }
    };
    Some((at(t0), at(t1)))
}

/// The outermost range one axis of the view may show, in the same units
/// as [`AxisView`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisLimits {
    pub lo: f64,
    pub hi: f64,
}

impl AxisLimits {
    fn width(self) -> f64 {
        self.hi - self.lo
    }
}

/// Time axis: the whole capture and nothing beyond it.
pub const X_LIMITS: AxisLimits = AxisLimits { lo: 0.0, hi: 1.0 };

/// The visible part of one axis, `lo..hi`, in "unit" coordinates: on the
/// time axis a fraction of the whole capture, on the voltage axis a fraction
/// of the device screen height (0 = top grid line, 1 = bottom grid line).
/// [`AxisView::UNIT`] is the unzoomed view.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisView {
    pub lo: f64,
    pub hi: f64,
}

impl Default for AxisView {
    fn default() -> Self {
        Self::UNIT
    }
}

impl AxisView {
    /// Unzoomed: the whole capture (time), the device screen (voltage).
    pub const UNIT: Self = Self { lo: 0.0, hi: 1.0 };
    /// Largest zoom-in, regardless of other limits.
    const MIN_SPAN: f64 = 1e-6;

    pub fn span(self) -> f64 {
        self.hi - self.lo
    }

    pub fn is_unit(self) -> bool {
        self == Self::UNIT
    }

    /// Unit coordinate -> fraction of the visible window (outside `0..=1`
    /// for positions outside the view).
    pub fn to_screen(self, data: f64) -> f64 {
        (data - self.lo) / self.span()
    }

    /// Fraction of the visible window -> unit coordinate.
    pub fn to_data(self, screen: f64) -> f64 {
        self.lo + screen * self.span()
    }

    /// Zoom by `factor` (< 1 zooms in) keeping the point at window
    /// fraction `anchor` fixed on screen. The span stays within
    /// `min_span..=limits` and the window inside `limits`.
    ///
    /// Zooming through the unzoomed span stops exactly at [`Self::UNIT`]
    /// (a detent), so the normal view is always easy to get back to even
    /// after zooming around different points.
    pub fn zoom_at(self, anchor: f64, factor: f64, min_span: f64, limits: AxisLimits) -> Self {
        let anchor = anchor.clamp(0.0, 1.0);
        let pivot = self.to_data(anchor);
        let max_span = limits.width();
        let span = (self.span() * factor).clamp(min_span.clamp(Self::MIN_SPAN, max_span), max_span);
        let old = self.span();
        let unit_allowed = limits.lo <= 0.0 && limits.hi >= 1.0;
        if unit_allowed && ((old < 1.0 && span >= 1.0) || (old > 1.0 && span <= 1.0)) {
            return Self::UNIT;
        }
        Self::within(pivot - anchor * span, span, limits)
    }

    /// Shift the window by `delta` (unit coordinates), stopping at the
    /// limits.
    pub fn pan(self, delta: f64, limits: AxisLimits) -> Self {
        Self::within(self.lo + delta, self.span(), limits)
    }

    /// A window of `span` starting at `lo`, moved inside `limits`.
    fn within(lo: f64, span: f64, limits: AxisLimits) -> Self {
        if span >= limits.width() {
            return Self {
                lo: limits.lo,
                hi: limits.hi,
            };
        }
        let lo = lo.clamp(limits.lo, limits.hi - span);
        Self { lo, hi: lo + span }
    }
}

/// Zoom and pan of both axes.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct View {
    /// Time axis.
    pub x: AxisView,
    /// Voltage axis.
    pub y: AxisView,
}

impl View {
    /// Unzoomed on both axes.
    pub const UNIT: Self = Self {
        x: AxisView::UNIT,
        y: AxisView::UNIT,
    };

    pub fn is_unit(self) -> bool {
        self == Self::UNIT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A horizontal band with no left/right limit in practice.
    fn band(top: f32, bottom: f32) -> ClipRect {
        ClipRect {
            left: -1e6,
            right: 1e6,
            top,
            bottom,
        }
    }

    #[test]
    fn polyline_is_cut_at_left_and_right_edges() {
        let rect = ClipRect {
            left: 10.0,
            right: 20.0,
            top: 0.0,
            bottom: 100.0,
        };
        let runs = clip_polyline(&[(0.0, 50.0), (15.0, 50.0), (30.0, 80.0)], rect);
        assert_eq!(runs, vec![vec![(10.0, 50.0), (15.0, 50.0), (20.0, 60.0)]]);
    }

    #[test]
    fn diagonal_through_a_corner_region_is_cut_on_both_axes() {
        let rect = ClipRect {
            left: 0.0,
            right: 10.0,
            top: 0.0,
            bottom: 10.0,
        };
        // From (-5, 5) to (5, 15): enters at x = 0 (y = 10), i.e. the corner.
        let runs = clip_polyline(&[(-5.0, 5.0), (5.0, 15.0)], rect);
        assert!(runs.is_empty() || runs[0].iter().all(|&p| rect.contains(p)));
        // From (-5, 2) to (5, 12): visible from (0, 7) to (3, 10).
        let runs = clip_polyline(&[(-5.0, 2.0), (5.0, 12.0)], rect);
        assert_eq!(runs, vec![vec![(0.0, 7.0), (3.0, 10.0)]]);
    }

    #[test]
    fn zoom_keeps_the_anchor_fixed() {
        let v = AxisView::UNIT.zoom_at(0.25, 0.5, 0.0, X_LIMITS);
        assert_eq!(
            v,
            AxisView {
                lo: 0.125,
                hi: 0.625
            }
        );
        // The capture point under the anchor is the same before and after.
        assert_eq!(AxisView::UNIT.to_data(0.25), v.to_data(0.25));
        let v2 = v.zoom_at(0.5, 0.5, 0.0, X_LIMITS);
        assert!((v.to_data(0.5) - v2.to_data(0.5)).abs() < 1e-12);
    }

    #[test]
    fn zoom_out_stops_at_full_view_and_stays_inside() {
        let v = AxisView { lo: 0.8, hi: 0.9 };
        assert_eq!(v.zoom_at(1.0, 100.0, 0.0, X_LIMITS), AxisView::UNIT);
        assert!(AxisView::UNIT.zoom_at(0.0, 1.5, 0.0, X_LIMITS).is_unit());
        // Zooming out near the end shifts the window back inside the capture.
        let w = v.zoom_at(1.0, 3.0, 0.0, X_LIMITS);
        assert!(w.hi <= 1.0 && w.lo >= 0.0);
        assert!((w.span() - 0.3).abs() < 1e-12);
    }

    #[test]
    fn zoom_in_is_limited_by_min_span() {
        let v = AxisView::UNIT.zoom_at(0.5, 1e-9, 0.01, X_LIMITS);
        assert!((v.span() - 0.01).abs() < 1e-12);
    }

    /// Voltage-axis limits wider than the screen (as for the ADC range).
    const WIDE: AxisLimits = AxisLimits {
        lo: -0.125,
        hi: 1.15,
    };

    #[test]
    fn axis_can_zoom_out_past_the_unit_view_up_to_its_limits() {
        let out = AxisView::UNIT.zoom_at(0.5, 1.2, 0.0, WIDE);
        assert!((out.span() - 1.2).abs() < 1e-12);
        assert!(out.lo >= WIDE.lo && out.hi <= WIDE.hi);
        let max = out.zoom_at(0.5, 100.0, 0.0, WIDE);
        assert_eq!(
            max,
            AxisView {
                lo: WIDE.lo,
                hi: WIDE.hi
            }
        );
    }

    #[test]
    fn zooming_through_the_unit_span_stops_at_the_unit_view() {
        // Zoomed in off-centre, then zoomed out past 1: lands exactly on UNIT.
        let zoomed = AxisView { lo: 0.6, hi: 0.9 };
        assert_eq!(zoomed.zoom_at(0.9, 4.0, 0.0, WIDE), AxisView::UNIT);
        // Zoomed out, then back in past 1: also UNIT.
        let wide = AxisView { lo: -0.1, hi: 1.1 };
        assert_eq!(wide.zoom_at(0.2, 0.8, 0.0, WIDE), AxisView::UNIT);
        // The next step continues normally.
        assert!(AxisView::UNIT.zoom_at(0.5, 0.8, 0.0, WIDE).span() < 1.0);
    }

    #[test]
    fn pan_stops_at_capture_ends() {
        let v = AxisView { lo: 0.4, hi: 0.6 };
        assert_eq!(v.pan(0.1, X_LIMITS), AxisView { lo: 0.5, hi: 0.7 });
        let end = v.pan(5.0, X_LIMITS);
        assert!((end.hi - 1.0).abs() < 1e-12 && (end.span() - 0.2).abs() < 1e-12);
        assert_eq!(v.pan(-5.0, X_LIMITS).lo, 0.0);
        assert_eq!(AxisView::UNIT.pan(0.3, X_LIMITS), AxisView::UNIT);
    }

    #[test]
    fn screen_and_data_fractions_round_trip() {
        let v = AxisView { lo: 0.2, hi: 0.4 };
        assert!((v.to_screen(0.3) - 0.5).abs() < 1e-12);
        assert!((v.to_data(0.5) - 0.3).abs() < 1e-12);
        assert!(v.to_screen(0.1) < 0.0);
    }

    #[test]
    fn polyline_inside_band_is_one_run() {
        let pts = [(0.0, 10.0), (1.0, 20.0), (2.0, 15.0)];
        assert_eq!(clip_polyline(&pts, band(0.0, 100.0)), vec![pts.to_vec()]);
    }

    #[test]
    fn polyline_leaving_and_returning_is_split_at_the_edges() {
        // Goes below the band (y > 100) between x = 1 and x = 3.
        let pts = [
            (0.0, 50.0),
            (1.0, 90.0),
            (2.0, 130.0),
            (3.0, 90.0),
            (4.0, 50.0),
        ];
        let runs = clip_polyline(&pts, band(0.0, 100.0));
        assert_eq!(runs.len(), 2);
        assert_eq!(runs[0], vec![(0.0, 50.0), (1.0, 90.0), (1.25, 100.0)]);
        assert_eq!(runs[1], vec![(2.75, 100.0), (3.0, 90.0), (4.0, 50.0)]);
    }

    #[test]
    fn polyline_entirely_outside_band_is_dropped() {
        let pts = [(0.0, -10.0), (1.0, -20.0), (2.0, 300.0)];
        // The last segment crosses the whole band, so only that piece remains.
        let runs = clip_polyline(&pts, band(0.0, 100.0));
        assert_eq!(runs.len(), 1);
        assert_eq!(runs[0].len(), 2);
        assert!(runs[0].iter().all(|p| (0.0..=100.0).contains(&p.1)));
        assert!(clip_polyline(&[(0.0, -5.0), (1.0, -6.0)], band(0.0, 100.0)).is_empty());
    }

    #[test]
    fn single_point_and_empty_input() {
        assert_eq!(
            clip_polyline(&[(3.0, 4.0)], band(0.0, 10.0)),
            vec![vec![(3.0, 4.0)]]
        );
        assert!(clip_polyline(&[(3.0, 40.0)], band(0.0, 10.0)).is_empty());
        assert!(clip_polyline(&[], band(0.0, 10.0)).is_empty());
    }

    #[test]
    fn spans_cover_every_sample_once() {
        let samples: Vec<u8> = (0..1000).map(|i| (i % 256) as u8).collect();
        let spans = column_spans(&samples, 7);
        assert_eq!(spans.len(), 7);
        assert_eq!(spans[0].start, 0);
        assert_eq!(spans.last().unwrap().end, samples.len());
        for pair in spans.windows(2) {
            assert_eq!(pair[0].end, pair[1].start);
        }
    }

    #[test]
    fn single_sample_spike_is_kept() {
        // 60 000 flat samples with one spike, reduced to 600 columns: picking
        // every 100th sample would miss index 12 345.
        let mut samples = vec![128u8; 60_000];
        samples[12_345] = 30;
        let spans = column_spans(&samples, 600);
        let hit = spans
            .iter()
            .find(|s| (s.start..s.end).contains(&12_345))
            .unwrap();
        assert_eq!(hit.min, 30);
        assert_eq!(hit.max, 128);
        assert_eq!(spans.iter().filter(|s| s.min != 128).count(), 1);
    }

    #[test]
    fn fewer_samples_than_columns_gives_one_span_per_sample() {
        let spans = column_spans(&[5, 9, 7], 100);
        assert_eq!(spans.len(), 3);
        assert_eq!(
            spans.iter().map(|s| (s.min, s.max)).collect::<Vec<_>>(),
            vec![(5, 5), (9, 9), (7, 7)]
        );
    }

    #[test]
    fn empty_inputs_give_no_spans() {
        assert!(column_spans(&[], 10).is_empty());
        assert!(column_spans(&[1, 2], 0).is_empty());
    }

    #[test]
    fn ordered_follows_signal_direction() {
        let rising = column_spans(&[10, 50, 20, 40], 1)[0];
        assert_eq!(rising.ordered(), [10, 50]);
        let falling = column_spans(&[40, 50, 10, 20], 1)[0];
        assert_eq!(falling.ordered(), [50, 10]);
    }

    #[test]
    fn distinct_values_are_sorted_and_unique() {
        let v: Vec<u8> = distinct_values(&[7, 3, 7, 200, 3]).collect();
        assert_eq!(v, vec![3, 7, 200]);
    }
}

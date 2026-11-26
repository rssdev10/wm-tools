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

/// Clip a polyline to the horizontal band `top..=bottom` (y grows downward).
///
/// Returns the visible runs: each run is a polyline wholly inside the band.
/// A segment that crosses the band edge is cut exactly at the edge, so the
/// trace visibly reaches the border before it leaves the screen, like on
/// the oscilloscope. Clipping is done on the geometry itself so it does not
/// depend on renderer clip support.
pub fn clip_polyline_y(points: &[(f32, f32)], top: f32, bottom: f32) -> Vec<Vec<(f32, f32)>> {
    let inside = |p: (f32, f32)| (top..=bottom).contains(&p.1);
    let mut runs = Vec::new();
    let mut run: Vec<(f32, f32)> = Vec::new();
    if let [only] = points {
        if inside(*only) {
            runs.push(vec![*only]);
        }
        return runs;
    }
    for seg in points.windows(2) {
        match clip_segment_y(seg[0], seg[1], top, bottom) {
            Some((a, b)) => {
                // A new run starts when the segment does not continue the
                // previous one (the trace left the band in between).
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

/// The part of segment `p → q` inside the band `top..=bottom`, if any.
/// Endpoints that are already inside are returned unchanged (bit-exact), so
/// consecutive segments can be joined by comparing points.
fn clip_segment_y(
    p: (f32, f32),
    q: (f32, f32),
    top: f32,
    bottom: f32,
) -> Option<((f32, f32), (f32, f32))> {
    let dy = q.1 - p.1;
    if dy == 0.0 {
        return (top..=bottom).contains(&p.1).then_some((p, q));
    }
    let t_top = (top - p.1) / dy;
    let t_bottom = (bottom - p.1) / dy;
    let t0 = t_top.min(t_bottom).max(0.0);
    let t1 = t_top.max(t_bottom).min(1.0);
    if t0 > t1 {
        return None;
    }
    let at = |t: f32| {
        if t == 0.0 {
            p
        } else if t == 1.0 {
            q
        } else {
            // A cut point lies on an edge; snap y onto it exactly so that
            // rounding cannot leave it just outside the band.
            let y = p.1 + dy * t;
            let y = if (y - top).abs() < (y - bottom).abs() {
                top
            } else {
                bottom
            };
            (p.0 + (q.0 - p.0) * t, y)
        }
    };
    Some((at(t0), at(t1)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polyline_inside_band_is_one_run() {
        let pts = [(0.0, 10.0), (1.0, 20.0), (2.0, 15.0)];
        assert_eq!(clip_polyline_y(&pts, 0.0, 100.0), vec![pts.to_vec()]);
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
        let runs = clip_polyline_y(&pts, 0.0, 100.0);
        assert_eq!(runs.len(), 2);
        assert_eq!(runs[0], vec![(0.0, 50.0), (1.0, 90.0), (1.25, 100.0)]);
        assert_eq!(runs[1], vec![(2.75, 100.0), (3.0, 90.0), (4.0, 50.0)]);
    }

    #[test]
    fn polyline_entirely_outside_band_is_dropped() {
        let pts = [(0.0, -10.0), (1.0, -20.0), (2.0, 300.0)];
        // The last segment crosses the whole band, so only that piece remains.
        let runs = clip_polyline_y(&pts, 0.0, 100.0);
        assert_eq!(runs.len(), 1);
        assert_eq!(runs[0].len(), 2);
        assert!(runs[0].iter().all(|p| (0.0..=100.0).contains(&p.1)));
        assert!(clip_polyline_y(&[(0.0, -5.0), (1.0, -6.0)], 0.0, 100.0).is_empty());
    }

    #[test]
    fn single_point_and_empty_input() {
        assert_eq!(
            clip_polyline_y(&[(3.0, 4.0)], 0.0, 10.0),
            vec![vec![(3.0, 4.0)]]
        );
        assert!(clip_polyline_y(&[(3.0, 40.0)], 0.0, 10.0).is_empty());
        assert!(clip_polyline_y(&[], 0.0, 10.0).is_empty());
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

//! Display scale (V/div, V offset, t/div) of one capture.
//!
//! Each capture is shown with its own scale:
//!
//! * A capture **with metadata** (a screenshot packet, or a `.zwcap` record
//!   that carries a `scope_state`) uses the scale the oscilloscope reported.
//!   It is fixed and cannot be edited.
//! * A capture **without metadata** (an ASCII debug dump) uses the manual
//!   scale the user entered in the measurement panel, stored in settings.
//!
//! Receiving or selecting a capture never changes the manual scale, so the
//! scale of one capture cannot leak into another.

use crate::settings::Settings;
use dso_parser::ScopeState;

/// Where a capture's scale comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScaleSource {
    /// Reported by the oscilloscope with the capture.
    Metadata,
    /// Entered by the user (the capture has no metadata).
    Manual,
}

/// The scale a capture is drawn and measured with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CaptureScale {
    /// Volts per division for CH1.
    pub v_per_div_ch1: f64,
    /// Volts per division for CH2.
    pub v_per_div_ch2: f64,
    /// Voltage at the graph centre for CH1.
    pub v_offset_ch1: f64,
    /// Voltage at the graph centre for CH2.
    pub v_offset_ch2: f64,
    /// Time per division in milliseconds.
    pub t_per_div_ms: f64,
    pub source: ScaleSource,
}

impl CaptureScale {
    /// The manual scale from settings.
    pub fn manual(settings: &Settings) -> Self {
        Self {
            v_per_div_ch1: settings.v_per_div_ch1,
            v_per_div_ch2: settings.v_per_div_ch2,
            v_offset_ch1: settings.v_offset_ch1,
            v_offset_ch2: settings.v_offset_ch2,
            t_per_div_ms: settings.t_per_div_ms,
            source: ScaleSource::Manual,
        }
    }

    /// The scale of a capture: from its metadata when it has any, otherwise
    /// the manual scale.
    ///
    /// A value the metadata does not carry (a channel that was off, or a
    /// zero timebase) falls back to the manual value; such a channel has no
    /// samples to draw, so the fallback only affects empty scale labels.
    pub fn for_capture(scope_state: Option<&ScopeState>, settings: &Settings) -> Self {
        let manual = Self::manual(settings);
        let Some(state) = scope_state else {
            return manual;
        };
        let mut scale = Self {
            source: ScaleSource::Metadata,
            ..manual
        };
        // The record stores the offset with the device's `zero_volt` sign;
        // the voltage at the graph centre has the opposite sign.
        if let Some(ch1) = &state.ch1 {
            scale.v_per_div_ch1 = ch1.vdiv_v;
            scale.v_offset_ch1 = -ch1.v_offset_v;
        }
        if let Some(ch2) = &state.ch2 {
            scale.v_per_div_ch2 = ch2.vdiv_v;
            scale.v_offset_ch2 = -ch2.v_offset_v;
        }
        if state.timebase_ns_per_div > 0 {
            scale.t_per_div_ms = state.timebase_ns_per_div as f64 / 1_000_000.0;
        }
        scale
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use dso_parser::ChannelState;

    /// A metadata record: CH1 2 V/div, offset stored as +0.5 V, 50 µs/div.
    pub(crate) fn scope_state(ch2: Option<ChannelState>) -> ScopeState {
        ScopeState {
            timebase_label: "50us".into(),
            timebase_ns_per_div: 50_000,
            sample_rate_hz: 1_000_000.0,
            buffer_depth: 60_000,
            trigger_source: "CH1".into(),
            trigger_mode: "Auto".into(),
            trigger_edge: "Rising".into(),
            trigger_level_v: 0.0,
            trigger_delay_s: 0.0,
            ch1: Some(channel(2.0, 0.5)),
            ch2,
            cursors_x_enable: false,
            cursors_y_enable: false,
            cursor_x1_frac: 0.0,
            cursor_x2_frac: 0.0,
            cursor_y1_frac: 0.0,
            cursor_y2_frac: 0.0,
            active_channel: 0,
        }
    }

    pub(crate) fn channel(vdiv_v: f64, v_offset_v: f64) -> ChannelState {
        ChannelState {
            vdiv_v,
            coupling: "DC".into(),
            probe: "×1".into(),
            v_offset_v,
            measurements: None,
        }
    }

    fn manual_settings() -> Settings {
        Settings {
            v_per_div_ch1: 0.1,
            v_per_div_ch2: 0.2,
            v_offset_ch1: 1.0,
            v_offset_ch2: -1.0,
            t_per_div_ms: 5.0,
            ..Settings::default()
        }
    }

    #[test]
    fn capture_without_metadata_uses_manual_scale() {
        let s = manual_settings();
        let scale = CaptureScale::for_capture(None, &s);
        assert_eq!(scale.source, ScaleSource::Manual);
        assert_eq!(scale, CaptureScale::manual(&s));
        assert_eq!(scale.t_per_div_ms, 5.0);
    }

    #[test]
    fn capture_with_metadata_uses_its_own_scale() {
        let s = manual_settings();
        let state = scope_state(Some(channel(0.5, -0.25)));
        let scale = CaptureScale::for_capture(Some(&state), &s);
        assert_eq!(scale.source, ScaleSource::Metadata);
        assert_eq!(scale.v_per_div_ch1, 2.0);
        assert_eq!(scale.v_offset_ch1, -0.5);
        assert_eq!(scale.v_per_div_ch2, 0.5);
        assert_eq!(scale.v_offset_ch2, 0.25);
        assert_eq!(scale.t_per_div_ms, 0.05);
    }

    #[test]
    fn missing_metadata_values_fall_back_to_manual() {
        let s = manual_settings();
        let mut state = scope_state(None);
        state.timebase_ns_per_div = 0;
        let scale = CaptureScale::for_capture(Some(&state), &s);
        assert_eq!(scale.source, ScaleSource::Metadata);
        assert_eq!(scale.v_per_div_ch1, 2.0);
        assert_eq!(scale.v_per_div_ch2, 0.2);
        assert_eq!(scale.v_offset_ch2, -1.0);
        assert_eq!(scale.t_per_div_ms, 5.0);
    }
}

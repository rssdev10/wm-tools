//! `.zwcap` container format — serializes captures (both debug dumps and
//! screenshot packets) into a single CBOR-encoded binary file.
//!
//! ## File layout
//! ```text
//! Bytes 0–7 : Magic header  b"ZWCAP\x00\x01\x00"  (version 1)
//! Bytes 8.. : CBOR-encoded Vec<CaptureRecord>
//! ```

use chrono::{DateTime, Local};
use serde::{Deserialize, Serialize};

/// Magic header for `.zwcap` files.
pub const ZWCAP_MAGIC: &[u8; 8] = b"ZWCAP\x00\x01\x00";

/// Top-level capture record stored in the container.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CaptureRecord {
    /// Schema version (currently 1).
    pub schema_version: u16,
    /// How the capture was originally acquired.
    pub source_format: SourceFormat,
    /// Device model string (e.g. "DSO3D12").
    pub device_model: Option<String>,
    /// ISO 8601 timestamp with timezone when capture was received.
    pub captured_at: Option<String>,
    /// Raw binary payload (the original bytes as received).
    pub raw_payload: Vec<u8>,
    /// Oscilloscope state at time of capture (from screenshot metadata).
    pub scope_state: Option<ScopeState>,
    /// Decoded waveforms ready for display.
    pub waveforms: Vec<Waveform>,
}

/// How the capture was originally obtained.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SourceFormat {
    /// ASCII debug dump (long-press Save, 60000 samples/channel).
    HiddenAsciiDump,
    /// Binary 2048-byte screenshot packet (Gallery → select waveform).
    ScreenshotBinary,
}

/// Oscilloscope state metadata (from screenshot packet settings section).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScopeState {
    pub timebase_label: String,
    pub timebase_ns_per_div: u64,
    pub sample_rate_hz: f64,
    pub buffer_depth: i64,
    pub trigger_source: String,
    pub trigger_mode: String,
    pub trigger_edge: String,
    pub trigger_level_v: f64,
    pub trigger_delay_s: f64,
    pub ch1: Option<ChannelState>,
    pub ch2: Option<ChannelState>,
    pub cursors_x_enable: bool,
    pub cursors_y_enable: bool,
    /// Cursor X1 position as fraction 0.0–1.0 of graph width.
    pub cursor_x1_frac: f64,
    /// Cursor X2 position as fraction 0.0–1.0 of graph width.
    pub cursor_x2_frac: f64,
    /// Cursor Y1 position as fraction 0.0–1.0 of graph height.
    pub cursor_y1_frac: f64,
    /// Cursor Y2 position as fraction 0.0–1.0 of graph height.
    pub cursor_y2_frac: f64,
}

/// Per-channel state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChannelState {
    pub vdiv_v: f64,
    pub coupling: String,
    pub probe: String,
    pub v_offset_v: f64,
    pub measurements: Option<ChannelMeasurements>,
}

/// Auto-measurement values in physical units.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChannelMeasurements {
    pub freq_hz: f64,
    pub period_s: f64,
    pub pk_pk_v: f64,
    pub amplitude_v: f64,
    pub max_v: f64,
    pub min_v: f64,
    pub top_v: f64,
    pub base_v: f64,
    pub avg_v: f64,
    pub rms_v: f64,
    pub pos_width_s: f64,
    pub neg_width_s: f64,
    pub pos_duty_pct: f64,
    pub neg_duty_pct: f64,
}

/// A waveform stored in the capture.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Waveform {
    /// Full-buffer raw ADC samples (from debug dump).
    RawSamples {
        channel: u8,
        /// ADC values 0–255 (stored as i16 for future headroom).
        counts: Vec<i16>,
        /// Seconds per sample (if known from metadata).
        sample_interval_s: Option<f64>,
        /// Volts per ADC count (if calibrated).
        volts_per_count: Option<f64>,
    },
    /// Display-resolution envelope (from screenshot packet).
    DisplayEnvelope {
        channel: u8,
        /// Minimum ADC value per display pixel.
        min_counts: Vec<u8>,
        /// Maximum ADC value per display pixel.
        max_counts: Vec<u8>,
        /// Seconds per pixel.
        time_per_pixel_s: Option<f64>,
        /// µV per pixel (volt_scale_uv / 25).
        volts_per_pixel: Option<f64>,
        /// Zero-volt reference offset in pixels.
        zero_pixel: Option<i16>,
    },
}

// ── Serialization ──────────────────────────────────────────────────────────

/// Serialize a list of capture records to the `.zwcap` binary format.
pub fn serialize_zwcap(records: &[CaptureRecord]) -> Vec<u8> {
    let mut buf = Vec::new();
    buf.extend_from_slice(ZWCAP_MAGIC);
    ciborium::into_writer(records, &mut buf).expect("CBOR serialize should not fail");
    buf
}

/// Deserialize a `.zwcap` file. Returns an error if the magic header is wrong
/// or CBOR decoding fails.
pub fn deserialize_zwcap(data: &[u8]) -> Result<Vec<CaptureRecord>, String> {
    if data.len() < 8 || &data[..8] != ZWCAP_MAGIC {
        return Err("Not a .zwcap file (bad magic header)".to_string());
    }
    ciborium::from_reader(&data[8..]).map_err(|e| format!("CBOR decode error: {e}"))
}

/// Returns true if the buffer starts with the `.zwcap` magic header.
pub fn is_zwcap(data: &[u8]) -> bool {
    data.len() >= 8 && &data[..8] == ZWCAP_MAGIC
}

// ── Conversion helpers ─────────────────────────────────────────────────────

use crate::screenshot::{self, ScreenshotPacket};
use crate::Capture;

/// Convert an ASCII debug dump `Capture` into a `CaptureRecord`.
pub fn capture_to_record(
    cap: &Capture,
    timestamp: DateTime<Local>,
    raw_bytes: Option<&[u8]>,
) -> CaptureRecord {
    let mut waveforms = vec![Waveform::RawSamples {
        channel: 1,
        counts: cap.ch1.iter().map(|&b| b as i16).collect(),
        sample_interval_s: None,
        volts_per_count: None,
    }];
    if let Some(ch2) = &cap.ch2 {
        waveforms.push(Waveform::RawSamples {
            channel: 2,
            counts: ch2.iter().map(|&b| b as i16).collect(),
            sample_interval_s: None,
            volts_per_count: None,
        });
    }
    CaptureRecord {
        schema_version: 1,
        source_format: SourceFormat::HiddenAsciiDump,
        device_model: Some("DSO3D12".to_string()),
        captured_at: Some(timestamp.format("%Y-%m-%dT%H:%M:%S%:z").to_string()),
        raw_payload: raw_bytes.map(|b| b.to_vec()).unwrap_or_default(),
        scope_state: None,
        waveforms,
    }
}

/// Convert a parsed screenshot packet into a `CaptureRecord`.
pub fn screenshot_to_record(
    pkt: &ScreenshotPacket,
    timestamp: DateTime<Local>,
    raw_bytes: &[u8],
) -> CaptureRecord {
    let s = &pkt.settings;
    let probe_mult = |p: u8| 10f64.powi(p as i32);

    let ch1_state = ChannelState {
        vdiv_v: s.ch1.volt_scale_uv as f64 / 1_000_000.0 * probe_mult(s.ch1.probe_mode),
        coupling: match s.ch1.coupling { 0 => "DC", _ => "AC" }.to_string(),
        probe: match s.ch1.probe_mode { 0 => "×1", 1 => "×10", _ => "×100" }.to_string(),
        v_offset_v: s.ch1.zero_volt_uv as f64 / 1_000_000.0 * probe_mult(s.ch1.probe_mode),
        measurements: pkt.ch1_meas.as_ref().map(|m| measurements_to_state(m, s.ch1.probe_mode)),
    };

    let ch2_state = if s.ch2.enabled {
        Some(ChannelState {
            vdiv_v: s.ch2.volt_scale_uv as f64 / 1_000_000.0 * probe_mult(s.ch2.probe_mode),
            coupling: match s.ch2.coupling { 0 => "DC", _ => "AC" }.to_string(),
            probe: match s.ch2.probe_mode { 0 => "×1", 1 => "×10", _ => "×100" }.to_string(),
            v_offset_v: s.ch2.zero_volt_uv as f64 / 1_000_000.0 * probe_mult(s.ch2.probe_mode),
            measurements: pkt.ch2_meas.as_ref().map(|m| measurements_to_state(m, s.ch2.probe_mode)),
        })
    } else {
        None
    };

    // Cursor fractions: device uses pixel offsets from center (150 for X, 128 for Y)
    let cx1 = (s.cursor_x1 as f64 + 150.0) / 300.0;
    let cx2 = (s.cursor_x2 as f64 + 150.0) / 300.0;
    let cy1 = (s.cursor_y1 as f64 + 128.0) / 256.0;
    let cy2 = (s.cursor_y2 as f64 + 128.0) / 256.0;

    let scope_state = ScopeState {
        timebase_label: s.timebase_label().to_string(),
        timebase_ns_per_div: s.timebase_ns_per_div,
        sample_rate_hz: s.sample_rate_meas_hz,
        buffer_depth: s.buffer_depth,
        trigger_source: match s.trigger_source { 0 => "CH1", _ => "CH2" }.to_string(),
        trigger_mode: match s.trigger_mode { 0 => "Auto", _ => "Normal" }.to_string(),
        trigger_edge: match s.trigger_edge { 0 => "Rise", _ => "Fall" }.to_string(),
        trigger_level_v: if s.trigger_source == 0 {
            s.ch1.trigger_level_uv as f64 / 1_000_000.0 * probe_mult(s.ch1.probe_mode)
        } else {
            s.ch2.trigger_level_uv as f64 / 1_000_000.0 * probe_mult(s.ch2.probe_mode)
        },
        trigger_delay_s: s.horiz_trigger_delay_ps as f64 / 1e12,
        ch1: Some(ch1_state),
        ch2: ch2_state,
        cursors_x_enable: s.cursors_x_enable,
        cursors_y_enable: s.cursors_y_enable,
        cursor_x1_frac: cx1.clamp(0.0, 1.0),
        cursor_x2_frac: cx2.clamp(0.0, 1.0),
        cursor_y1_frac: cy1.clamp(0.0, 1.0),
        cursor_y2_frac: cy2.clamp(0.0, 1.0),
    };

    let mut waveforms = vec![Waveform::DisplayEnvelope {
        channel: 1,
        min_counts: pkt.ch1_min.to_vec(),
        max_counts: pkt.ch1_max.to_vec(),
        time_per_pixel_s: Some(s.time_per_pixel_s()),
        volts_per_pixel: Some(s.ch1.volt_scale_uv as f64 / 25.0),
        zero_pixel: Some(s.ch1.zero_volt_pixels as i16),
    }];
    if let (Some(mn), Some(mx)) = (&pkt.ch2_min, &pkt.ch2_max) {
        waveforms.push(Waveform::DisplayEnvelope {
            channel: 2,
            min_counts: mn.to_vec(),
            max_counts: mx.to_vec(),
            time_per_pixel_s: Some(s.time_per_pixel_s()),
            volts_per_pixel: Some(s.ch2.volt_scale_uv as f64 / 25.0),
            zero_pixel: Some(s.ch2.zero_volt_pixels as i16),
        });
    }

    CaptureRecord {
        schema_version: 1,
        source_format: SourceFormat::ScreenshotBinary,
        device_model: Some(pkt.device.to_string()),
        captured_at: Some(timestamp.format("%Y-%m-%dT%H:%M:%S%:z").to_string()),
        raw_payload: raw_bytes.to_vec(),
        scope_state: Some(scope_state),
        waveforms,
    }
}

/// Maximum plausible voltage (100 kV) — device leaves uninitialized fields
/// with garbage that can reach hundreds of megavolts.
const MEAS_MAX_V: f64 = 100_000.0;
/// Maximum plausible pulse/period width in seconds.
/// The DSO3D12's widest timebase is 10 s/div × 12 divs = 120 s.
const MEAS_MAX_T: f64 = 200.0;

/// Return `v` unchanged, or `NaN` if the value looks like uninitialised data.
#[inline]
fn plausible_v(v: f64) -> f64 {
    if v.is_finite() && v.abs() <= MEAS_MAX_V { v } else { f64::NAN }
}

/// Return `v` unchanged, or `NaN` if outside the device's possible time range.
#[inline]
fn plausible_t(t: f64) -> f64 {
    if t.is_finite() && (0.0..=MEAS_MAX_T).contains(&t) { t } else { f64::NAN }
}

fn measurements_to_state(m: &screenshot::Measurements, probe: u8) -> ChannelMeasurements {
    let mult = 10f64.powi(probe as i32);
    let uv = |raw: i64| plausible_v(raw as f64 / 1_000_000.0 * mult);
    ChannelMeasurements {
        freq_hz:      { let v = m.freq_hz();   if v.is_finite() && v >= 0.0 { v } else { f64::NAN } },
        period_s:     plausible_t(m.period_s()),
        pk_pk_v:      uv(m.pk_pk_uv),
        amplitude_v:  uv(m.amplitude_uv),
        max_v:        uv(m.max_uv),
        min_v:        uv(m.min_uv),
        top_v:        uv(m.top_uv),
        base_v:       uv(m.base_uv),
        avg_v:        uv(m.avg_uv),
        rms_v:        uv(m.rms_uv),
        pos_width_s:  plausible_t(m.pos_width_ps as f64 / 1e12),
        neg_width_s:  plausible_t(m.neg_width_ps as f64 / 1e12),
        pos_duty_pct: { let v = m.pos_duty_pct();               if v.is_finite() && (0.0..=100.0).contains(&v) { v } else { f64::NAN } },
        neg_duty_pct: { let v = m.neg_duty_tenth_pct as f64 / 10.0; if v.is_finite() && (0.0..=100.0).contains(&v) { v } else { f64::NAN } },
    }
}

/// Convert a `CaptureRecord` back to the legacy `Capture` struct for display.
/// This allows the GUI to use a unified rendering pipeline.
pub fn record_to_capture(rec: &CaptureRecord) -> Capture {
    match rec.source_format {
        SourceFormat::HiddenAsciiDump => {
            let mut ch1 = Vec::new();
            let mut ch2: Option<Vec<u8>> = None;
            for w in &rec.waveforms {
                match w {
                    Waveform::RawSamples { channel: 1, counts, .. } => {
                        ch1 = counts.iter().map(|&c| c.clamp(0, 255) as u8).collect();
                    }
                    Waveform::RawSamples { channel: 2, counts, .. } => {
                        ch2 = Some(counts.iter().map(|&c| c.clamp(0, 255) as u8).collect());
                    }
                    _ => {}
                }
            }
            Capture { ch1, ch2 }
        }
        SourceFormat::ScreenshotBinary => {
            // For screenshot packets, construct a Capture from the envelope average.
            let mut ch1 = Vec::new();
            let mut ch2: Option<Vec<u8>> = None;
            for w in &rec.waveforms {
                match w {
                    Waveform::DisplayEnvelope { channel: 1, min_counts, max_counts, .. } => {
                        ch1 = min_counts.iter().zip(max_counts.iter())
                            .map(|(&mn, &mx)| ((mn as u16 + mx as u16) / 2) as u8)
                            .collect();
                    }
                    Waveform::DisplayEnvelope { channel: 2, min_counts, max_counts, .. } => {
                        ch2 = Some(min_counts.iter().zip(max_counts.iter())
                            .map(|(&mn, &mx)| ((mn as u16 + mx as u16) / 2) as u8)
                            .collect());
                    }
                    _ => {}
                }
            }
            Capture { ch1, ch2 }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_zwcap() {
        let rec = CaptureRecord {
            schema_version: 1,
            source_format: SourceFormat::HiddenAsciiDump,
            device_model: Some("DSO3D12".to_string()),
            captured_at: Some("2025-05-31T12:00:00+03:00".to_string()),
            raw_payload: vec![1, 2, 3],
            scope_state: None,
            waveforms: vec![Waveform::RawSamples {
                channel: 1,
                counts: vec![100, 110, 120],
                sample_interval_s: None,
                volts_per_count: None,
            }],
        };
        let data = serialize_zwcap(&[rec.clone()]);
        assert!(is_zwcap(&data));
        let decoded = deserialize_zwcap(&data).unwrap();
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].device_model, rec.device_model);
        assert_eq!(decoded[0].raw_payload, rec.raw_payload);
    }

    #[test]
    fn record_to_capture_roundtrip() {
        let rec = CaptureRecord {
            schema_version: 1,
            source_format: SourceFormat::HiddenAsciiDump,
            device_model: None,
            captured_at: None,
            raw_payload: vec![],
            scope_state: None,
            waveforms: vec![
                Waveform::RawSamples { channel: 1, counts: vec![10, 20, 30], sample_interval_s: None, volts_per_count: None },
                Waveform::RawSamples { channel: 2, counts: vec![40, 50, 60], sample_interval_s: None, volts_per_count: None },
            ],
        };
        let cap = record_to_capture(&rec);
        assert_eq!(cap.ch1, vec![10, 20, 30]);
        assert_eq!(cap.ch2, Some(vec![40, 50, 60]));
    }

    #[test]
    fn is_zwcap_detection() {
        assert!(!is_zwcap(b"not a zwcap file"));
        assert!(!is_zwcap(b""));
        let data = serialize_zwcap(&[]);
        assert!(is_zwcap(&data));
    }
}

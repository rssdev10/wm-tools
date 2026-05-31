//! Parser for ZeeWeii DSO3D12 oscilloscope raw debug dumps.
//!
//! Format (as transmitted by the device after the "Long-press Save" debug
//! dump action, captured directly from the serial port):
//!
//! ```text
//! --------CH1--------\r\n
//! NNN \0NNN \0NNN \0...   (Buffer Depth samples; default 60000 per channel)
//! [optional]
//! --------CH2--------\r\n
//! NNN \0NNN \0...
//! ```
//!
//! Each sample is an ASCII decimal integer 0..=255 (one byte of ADC data),
//! followed by a single space (`0x20`) and a NUL byte (`0x00`).

use thiserror::Error;

pub mod screenshot;
pub use screenshot::{
    parse_screenshot, parse_screenshot_as, is_screenshot_packet,
    DeviceVariant, ScreenshotPacket, OscSettings, ChannelSettings,
    Measurements, PixelPoint,
    format_uv, format_ps, format_hz, format_sample_rate,
};

pub mod container;
pub use container::{
    CaptureRecord, SourceFormat, ScopeState, ChannelState, ChannelMeasurements,
    Waveform, serialize_zwcap, deserialize_zwcap, is_zwcap,
    capture_to_record, screenshot_to_record, record_to_capture,
    ZWCAP_MAGIC,
};

/// Default total buffer depth in samples that the DSO3D12 transmits per
/// channel for a complete debug dump.
pub const DEFAULT_BUFFER_DEPTH: usize = 60_000;

/// ASCII header marking the start of channel 1 samples.
pub const CH1_MARKER: &[u8] = b"--------CH1--------";

/// ASCII header marking the start of channel 2 samples.
pub const CH2_MARKER: &[u8] = b"--------CH2--------";

/// A single decoded oscilloscope capture.
///
/// Sample values are raw 8-bit ADC counts (0..=255).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Capture {
    /// Channel 1 samples (always present).
    pub ch1: Vec<u8>,
    /// Channel 2 samples (present only if the dump contained `--CH2--`).
    pub ch2: Option<Vec<u8>>,
}

impl Capture {
    /// Returns the number of samples in the longest channel.
    pub fn sample_count(&self) -> usize {
        let n1 = self.ch1.len();
        let n2 = self.ch2.as_ref().map(|v| v.len()).unwrap_or(0);
        n1.max(n2)
    }
}

/// Errors returned by the parser.
#[derive(Debug, Error)]
pub enum ParseError {
    #[error("no CH1 marker found in input")]
    MissingCh1Marker,

    #[error("malformed sample at byte offset {offset}: {reason}")]
    MalformedSample { offset: usize, reason: &'static str },

    #[error("buffer too short: got {got} bytes, expected at least {expected}")]
    TooShort { got: usize, expected: usize },
}

/// Parse every capture contained in `data`.
///
/// A new capture starts at every occurrence of [`CH1_MARKER`]. The parser is
/// tolerant: it will skip over any non-sample bytes (extra whitespace, status
/// strings printed by the device between captures, etc.) until it sees the
/// next ASCII digit or marker.
pub fn parse_captures(data: &[u8]) -> Result<Vec<Capture>, ParseError> {
    let mut captures = Vec::new();
    let mut cursor = 0;

    while let Some(start) = find_subslice(&data[cursor..], CH1_MARKER) {
        let abs = cursor + start;
        let ch1_body = skip_marker_eol(data, abs + CH1_MARKER.len());

        // CH1 samples end either at the CH2 marker or at the next CH1 marker
        // (next capture) or at end of file.
        let ch1_end = find_next_boundary(data, ch1_body);
        let ch1 = decode_ascii_samples(&data[ch1_body..ch1_end]);

        // Check whether the next boundary is a CH2 within the same capture.
        let mut next_cursor = ch1_end;
        let ch2 = if data[ch1_end..].starts_with(CH2_MARKER) {
            let ch2_body = skip_marker_eol(data, ch1_end + CH2_MARKER.len());
            let ch2_end = find_next_boundary(data, ch2_body);
            // The device sends CH2 ADC values with inverted polarity in the ASCII
            // dump (a hardware quirk). Flip here so Capture.ch2 uses the same
            // visual-screen-coordinate convention as CH1 and screenshot data:
            // low value = near top of screen = positive voltage.
            let samples: Vec<u8> = decode_ascii_samples(&data[ch2_body..ch2_end])
                .into_iter().map(|v| 255 - v).collect();
            next_cursor = ch2_end;
            Some(samples)
        } else {
            None
        };

        captures.push(Capture { ch1, ch2 });
        cursor = next_cursor;
    }

    if captures.is_empty() {
        return Err(ParseError::MissingCh1Marker);
    }
    Ok(captures)
}

/// Parse exactly one capture: the first `--------CH1--------` block found in
/// `data` plus its optional companion `--------CH2--------` block.
pub fn parse_first(data: &[u8]) -> Result<Capture, ParseError> {
    parse_captures(data).map(|mut v| v.remove(0))
}

/// Returns `true` if `data` looks like (or could become) the start of a valid
/// DSO3D12 raw debug dump — used by the live-serial state machine to decide
/// whether to keep buffering or discard a noise stream.
pub fn looks_like_dump_start(data: &[u8]) -> bool {
    if data.is_empty() {
        return false;
    }
    // Accept any prefix of the marker as a partial header.
    let limit = data.len().min(CH1_MARKER.len());
    CH1_MARKER.starts_with(&data[..limit])
}

/// Convert a raw ADC byte (0..=255) into a normalized signed value in `[-1, 1]`,
/// matching the way the device places "screen center" at code 128.
///
/// This is the simplest scaling we can apply without device-side metadata
/// (volt-scale, probe mode, zero offset). Callers that have access to the
/// settings buffer can substitute their own conversion.
pub fn adc_to_normalized(byte: u8) -> f32 {
    // Match ZeeTweak: y_screen = 255 - raw; center = 128.
    let centered = (255i16 - byte as i16) - 128;
    centered as f32 / 128.0
}

// ---------- internals ----------

fn find_subslice(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() || haystack.len() < needle.len() {
        return None;
    }
    haystack
        .windows(needle.len())
        .position(|w| w == needle)
}

/// Advance past `\r`, `\n`, or `\r\n` line endings immediately after a marker.
fn skip_marker_eol(data: &[u8], mut i: usize) -> usize {
    if i < data.len() && data[i] == b'\r' {
        i += 1;
    }
    if i < data.len() && data[i] == b'\n' {
        i += 1;
    }
    i
}

/// Find the next CH1 or CH2 marker in `data` starting at `from`, or end of
/// data if none.
fn find_next_boundary(data: &[u8], from: usize) -> usize {
    let mut best = data.len();
    if let Some(p) = find_subslice(&data[from..], CH1_MARKER) {
        best = best.min(from + p);
    }
    if let Some(p) = find_subslice(&data[from..], CH2_MARKER) {
        best = best.min(from + p);
    }
    best
}

/// Decode a sequence of ASCII samples of the form `"NN \0NNN \0..."` into
/// byte values. Malformed tokens are silently skipped; this keeps the parser
/// robust against the occasional dropped serial byte.
fn decode_ascii_samples(body: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(DEFAULT_BUFFER_DEPTH);
    let mut acc: u32 = 0;
    let mut have_digit = false;

    for &b in body {
        match b {
            b'0'..=b'9' => {
                acc = acc.saturating_mul(10).saturating_add((b - b'0') as u32);
                have_digit = true;
            }
            // Terminators that complete a token.
            b' ' | 0x00 | b'\r' | b'\n' | b'\t' => {
                if have_digit {
                    if acc <= 255 {
                        out.push(acc as u8);
                    }
                    acc = 0;
                    have_digit = false;
                }
            }
            _ => {
                // Unknown byte: drop the in-flight token to avoid pollution.
                acc = 0;
                have_digit = false;
            }
        }
    }
    if have_digit && acc <= 255 {
        out.push(acc as u8);
    }
    out
}

// ── Metadata (timestamps appended after capture data) ──────────────────────

use chrono::{DateTime, Local, TimeZone};

/// Metadata marker appended after all capture data. ZeeTweak and our parser
/// both ignore this because it doesn't start with `--------CH1--------`.
pub const METADATA_MARKER: &[u8] = b"\n---DSO3D12-META---\n";

/// A capture paired with a timestamp indicating when it was received.
#[derive(Debug, Clone)]
pub struct CaptureEntry {
    pub capture: Capture,
    pub timestamp: DateTime<Local>,
}

/// Parse the metadata section (timestamps) from file bytes.
/// Returns `None` if no metadata marker is found or if parsing fails.
pub fn parse_metadata(data: &[u8]) -> Option<Vec<DateTime<Local>>> {
    let marker = METADATA_MARKER;
    // Find the metadata marker (search from end for efficiency)
    let pos = data.windows(marker.len()).rposition(|w| w == marker)?;
    let json_start = pos + marker.len();
    let json_bytes = &data[json_start..];
    let json_end = json_bytes.iter().position(|&b| b == b'\n').unwrap_or(json_bytes.len());
    let json_str = std::str::from_utf8(&json_bytes[..json_end]).ok()?;
    let ts_strings: Vec<String> = serde_json::from_str(json_str).ok()?;
    let timestamps: Vec<DateTime<Local>> = ts_strings
        .iter()
        .filter_map(|s| {
            chrono::DateTime::parse_from_str(s, "%Y-%m-%dT%H:%M:%S%:z")
                .ok()
                .map(|dt| Local.from_utc_datetime(&dt.naive_utc()))
        })
        .collect();
    if timestamps.is_empty() {
        None
    } else {
        Some(timestamps)
    }
}

/// Load captures from file bytes with timestamps.
/// Parses both the raw capture data and optional metadata section.
pub fn load_captures_with_metadata(data: &[u8]) -> Result<Vec<CaptureEntry>, ParseError> {
    let captures = parse_captures(data)?;
    let timestamps = parse_metadata(data);
    let now = Local::now();
    let entries = captures
        .into_iter()
        .enumerate()
        .map(|(i, capture)| {
            let timestamp = timestamps
                .as_ref()
                .and_then(|ts| ts.get(i).copied())
                .unwrap_or(now);
            CaptureEntry { capture, timestamp }
        })
        .collect();
    Ok(entries)
}

/// Serialize captures with metadata (timestamps) into the DSO3D12 file format.
/// The output is compatible with ZeeTweak and other parsers — metadata is appended
/// at the end after a marker they won't recognize as a channel header.
pub fn serialize_captures(entries: &[CaptureEntry]) -> Vec<u8> {
    use std::io::Write;
    let mut buf = Vec::new();

    for entry in entries {
        // Write CH1
        buf.extend_from_slice(b"--------CH1--------\r\n");
        for (i, &sample) in entry.capture.ch1.iter().enumerate() {
            if i > 0 {
                buf.extend_from_slice(b" \0");
            }
            write!(buf, "{sample}").unwrap();
        }
        buf.extend_from_slice(b" \0");

        // Write CH2 if present
        if let Some(ref ch2) = entry.capture.ch2 {
            buf.extend_from_slice(b"--------CH2--------\r\n");
            for (i, &sample) in ch2.iter().enumerate() {
                if i > 0 {
                    buf.extend_from_slice(b" \0");
                }
                write!(buf, "{sample}").unwrap();
            }
            buf.extend_from_slice(b" \0");
        }
    }

    // Append metadata (timestamps)
    buf.extend_from_slice(METADATA_MARKER);
    let timestamps: Vec<String> = entries
        .iter()
        .map(|e| e.timestamp.format("%Y-%m-%dT%H:%M:%S%:z").to_string())
        .collect();
    if let Ok(meta_json) = serde_json::to_string(&timestamps) {
        buf.extend_from_slice(meta_json.as_bytes());
    }
    buf.push(b'\n');
    buf
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_dump(ch1: &[u8], ch2: Option<&[u8]>) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(CH1_MARKER);
        out.extend_from_slice(b"\r\n");
        for s in ch1 {
            out.extend_from_slice(format!("{} ", s).as_bytes());
            out.push(0);
        }
        if let Some(c2) = ch2 {
            out.extend_from_slice(CH2_MARKER);
            out.extend_from_slice(b"\r\n");
            for s in c2 {
                out.extend_from_slice(format!("{} ", s).as_bytes());
                out.push(0);
            }
        }
        out
    }

    #[test]
    fn parses_single_ch1_capture() {
        let dump = make_dump(&[137, 138, 137, 139], None);
        let cap = parse_first(&dump).unwrap();
        assert_eq!(cap.ch1, vec![137, 138, 137, 139]);
        assert!(cap.ch2.is_none());
    }

    #[test]
    fn parses_dual_channel_capture() {
        let dump = make_dump(&[10, 20, 30], Some(&[40, 50, 60]));
        let cap = parse_first(&dump).unwrap();
        assert_eq!(cap.ch1, vec![10, 20, 30]);
        // ASCII dump CH2 is inverted at parse time so it matches the
        // screen-coordinate convention used by screenshot data (low value = top).
        assert_eq!(cap.ch2.as_deref(), Some(&[215u8, 205, 195][..]));
    }

    #[test]
    fn parses_multiple_consecutive_captures() {
        let mut data = make_dump(&[1, 2, 3], None);
        data.extend_from_slice(&make_dump(&[4, 5, 6], Some(&[7, 8])));
        let caps = parse_captures(&data).unwrap();
        assert_eq!(caps.len(), 2);
        assert_eq!(caps[0].ch1, vec![1, 2, 3]);
        assert_eq!(caps[1].ch1, vec![4, 5, 6]);
        // CH2 is inverted at parse time: 255-7=248, 255-8=247
        assert_eq!(caps[1].ch2.as_deref(), Some(&[248u8, 247][..]));
    }

    #[test]
    fn missing_marker_is_error() {
        let err = parse_captures(b"no markers here").unwrap_err();
        matches!(err, ParseError::MissingCh1Marker);
    }

    #[test]
    fn skips_malformed_tokens() {
        let mut data = Vec::new();
        data.extend_from_slice(CH1_MARKER);
        data.extend_from_slice(b"\r\n");
        data.extend_from_slice(b"137 \x00garbage \x00200 \x00");
        let cap = parse_first(&data).unwrap();
        assert_eq!(cap.ch1, vec![137, 200]);
    }

    #[test]
    fn clamps_values_above_255() {
        let mut data = Vec::new();
        data.extend_from_slice(CH1_MARKER);
        data.extend_from_slice(b"\r\n");
        data.extend_from_slice(b"300 \x00128 \x00");
        let cap = parse_first(&data).unwrap();
        // 300 is out-of-range for u8 and gets dropped.
        assert_eq!(cap.ch1, vec![128]);
    }

    #[test]
    fn adc_normalization_centers_at_zero() {
        assert!((adc_to_normalized(127) - (0.0_f32)).abs() < 0.01);
        assert!(adc_to_normalized(0) > 0.9);
        assert!(adc_to_normalized(255) < -0.9);
    }

    #[test]
    fn looks_like_dump_start_accepts_prefix() {
        assert!(looks_like_dump_start(b"--"));
        assert!(looks_like_dump_start(b"--------CH1"));
        assert!(looks_like_dump_start(b"--------CH1--------"));
        assert!(!looks_like_dump_start(b"hello"));
        assert!(!looks_like_dump_start(b""));
    }

    #[test]
    fn sample_count_is_max_of_channels() {
        let c = Capture {
            ch1: vec![0; 100],
            ch2: Some(vec![0; 250]),
        };
        assert_eq!(c.sample_count(), 250);
    }

    #[test]
    fn parses_real_example_file_first_capture() {
        // Smoke-test: if the example file is present, make sure we get
        // ~60000 samples in CH1.
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../scripts/captures/dso3d12_raw_115200_20260530_183115.bin");
        if !path.exists() {
            return;
        }
        let bytes = std::fs::read(&path).expect("read example");
        let cap = parse_first(&bytes).expect("parse");
        assert_eq!(cap.ch1.len(), DEFAULT_BUFFER_DEPTH);
        assert!(cap.ch2.is_none());
    }
}

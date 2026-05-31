//! Parser for the ZeeWeii "screenshot" binary protocol.
//!
//! Based on https://github.com/taligentx/ZeeTweak/blob/main/zeetweak.py
//! 
//! When the user opens a saved waveform in the device's Screenshot/Gallery view
//! (or presses Save while live), the device transmits a **2048-byte binary packet**
//! at 115200 baud over the USB-serial port.
//!
//! ## Packet layout
//! ```text
//! Bytes    0 –  299  : CH1 min ADC values  (300 × u8, inverted: real = 255 − raw)
//! Bytes  300 –  599  : CH1 max ADC values  (300 × u8, inverted)
//! Bytes  600 –  899  : CH2 min ADC values  (300 × u8, inverted)
//! Bytes  900 – 1199  : CH2 max ADC values  (300 × u8, inverted)
//! Bytes 1200 – 1539  : Settings / state (DATA_DSO3D12 map, 340 bytes)
//! Bytes 1540 – 1639  : CH1 auto-measurements (100 bytes = 12×i64 + 2×i16)
//! Bytes 1640 – 1739  : CH2 auto-measurements (100 bytes)
//! Bytes 1740 – 2047  : Reserved / padding
//! ```
//! 300 x-pixels = 12 horizontal divisions × 25 pixels/div.
//!
//! DSO2512G firmware uses slightly different field offsets (see [`DeviceVariant`]).

use crate::{Capture, ParseError};

/// Which firmware / hardware variant we're parsing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DeviceVariant {
    /// ZeeWeii DSO3D12 (default, firmware v3.x).
    #[default]
    Dso3d12,
    /// ZeeWeii DSO2512G firmware v2.x.
    Dso2512gV2,
    /// ZeeWeii DSO2512G firmware v1.x.
    Dso2512gV1,
}

impl std::fmt::Display for DeviceVariant {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            DeviceVariant::Dso3d12 => "DSO3D12",
            DeviceVariant::Dso2512gV2 => "DSO2512G v2",
            DeviceVariant::Dso2512gV1 => "DSO2512G v1",
        })
    }
}


/// Expected total screenshot packet size.
pub const SCREENSHOT_SIZE: usize = 2048;

/// Number of waveform x-pixels (12 divisions × 25 px/div).
pub const WAVEFORM_PIXELS: usize = 300;

// ── Low-level read helpers ────────────────────────────────────────────────

#[inline]
fn read_u8(buf: &[u8], off: usize) -> u8 {
    buf.get(off).copied().unwrap_or(0)
}

#[inline]
fn read_u16(buf: &[u8], off: usize) -> u16 {
    if off + 2 > buf.len() { return 0; }
    u16::from_le_bytes([buf[off], buf[off + 1]])
}

#[inline]
fn read_i16(buf: &[u8], off: usize) -> i16 {
    if off + 2 > buf.len() { return 0; }
    i16::from_le_bytes([buf[off], buf[off + 1]])
}

#[inline]
fn read_u32(buf: &[u8], off: usize) -> u32 {
    if off + 4 > buf.len() { return 0; }
    u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

#[inline]
fn read_i32(buf: &[u8], off: usize) -> i32 {
    if off + 4 > buf.len() { return 0; }
    i32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

#[inline]
fn read_u64(buf: &[u8], off: usize) -> u64 {
    if off + 8 > buf.len() { return 0; }
    u64::from_le_bytes(buf[off..off + 8].try_into().unwrap())
}

#[inline]
fn read_i64(buf: &[u8], off: usize) -> i64 {
    if off + 8 > buf.len() { return 0; }
    i64::from_le_bytes(buf[off..off + 8].try_into().unwrap())
}

// ── Lookup tables ─────────────────────────────────────────────────────────

/// Timebase index → label (2=5ns … 30=10s). Index 0/1 reserved.
pub const TIMEBASE_LABELS: &[(u8, &str)] = &[
    (0, "1ns"), (1, "2ns"), (2, "5ns"), (3, "10ns"), (4, "20ns"), (5, "50ns"),
    (6, "100ns"), (7, "200ns"), (8, "500ns"), (9, "1µs"), (10, "2µs"), (11, "5µs"),
    (12, "10µs"), (13, "20µs"), (14, "50µs"), (15, "100µs"), (16, "200µs"), (17, "500µs"),
    (18, "1ms"), (19, "2ms"), (20, "5ms"), (21, "10ms"), (22, "20ms"), (23, "50ms"),
    (24, "100ms"), (25, "200ms"), (26, "500ms"), (27, "1s"), (28, "2s"), (29, "5s"),
    (30, "10s"),
];

/// V/Div index → base scale in µV (x1 probe). Keys: 1–13.
pub const VDIV_BASE_UV: &[(u8, u32)] = &[
    (1, 1_000), (2, 2_000), (3, 5_000), (4, 10_000), (5, 20_000), (6, 50_000),
    (7, 100_000), (8, 200_000), (9, 500_000), (10, 1_000_000), (11, 2_000_000),
    (12, 5_000_000), (13, 10_000_000),
];

fn timebase_label(idx: u8) -> &'static str {
    TIMEBASE_LABELS.iter().find(|(i, _)| *i == idx).map(|(_, s)| *s).unwrap_or("?")
}

fn vdiv_label_uv(idx: u8) -> u32 {
    VDIV_BASE_UV.iter().find(|(i, _)| *i == idx).map(|(_, v)| *v).unwrap_or(0)
}

fn probe_label(mode: u8) -> &'static str {
    match mode { 0 => "×1", 1 => "×10", 2 => "×100", _ => "?" }
}

fn run_state_label(s: u8) -> &'static str {
    match s { 0 => "Stop", 1 => "Run", 2 => "Wait", _ => "?" }
}

fn coupling_label(c: u8) -> &'static str {
    match c { 0 => "DC", 1 => "AC", _ => "?" }
}

fn trigger_source_label(s: u8) -> &'static str {
    match s { 0 => "CH1", 1 => "CH2", _ => "?" }
}

fn trigger_mode_label(m: u8) -> &'static str {
    match m { 0 => "Auto", 1 => "Normal", _ => "?" }
}

fn trigger_edge_label(e: u8) -> &'static str {
    match e { 0 => "Rise", 1 => "Fall", _ => "?" }
}

// ── Formatting helpers (mirror ZeeTweak Python) ───────────────────────────

/// Format microvolts → human-readable voltage string.
pub fn format_uv(uv: i64, probe_mode: u8) -> String {
    let volts = uv as f64 / 1_000_000.0 * (10f64.powi(probe_mode as i32));
    let abs_v = volts.abs();
    if abs_v == 0.0 { return "0.00V".to_string(); }
    if abs_v < 0.99995e-3 { return format!("{:.1}µV", volts * 1e6); }
    if abs_v < 0.99995    { return format!("{:.1}mV", volts * 1e3); }
    if abs_v < 10.0       { return format!("{:.2}V",  volts); }
    if abs_v < 100.0      { return format!("{:.1}V",  volts); }
    format!("{:.0}V", volts)
}

/// Format picoseconds → human-readable time string.
pub fn format_ps(ps: i64) -> String {
    let secs = ps as f64 / 1e12;
    let val = secs.abs();
    if val == 0.0 { return "0.00s".to_string(); }
    if val < 999.5e-9  { let ns = secs * 1e9;  return if ns.abs() >= 100.0 { format!("{:.0}ns", ns) } else if ns.abs() >= 10.0 { format!("{:.1}ns", ns) } else { format!("{:.2}ns", ns) }; }
    if val < 999.5e-6  { let us = secs * 1e6;  return if us.abs() >= 100.0 { format!("{:.0}µs", us) } else if us.abs() >= 10.0 { format!("{:.1}µs", us) } else { format!("{:.2}µs", us) }; }
    if val < 0.9995    { let ms = secs * 1e3;  return if ms.abs() >= 100.0 { format!("{:.0}ms", ms) } else if ms.abs() >= 10.0 { format!("{:.1}ms", ms) } else { format!("{:.2}ms", ms) }; }
    if val >= 100.0 { return format!("{:.0}s", secs); }
    if val >= 10.0  { return format!("{:.1}s", secs); }
    format!("{:.2}s", secs)
}

/// Format Hz → human-readable frequency string.
pub fn format_hz(hz: f64) -> String {
    if hz < 0.0 { return format_hz(0.0); }
    let (val, unit) = if hz >= 999_500.0 {
        (hz / 1_000_000.0, "MHz")
    } else if hz >= 999.5 {
        (hz / 1_000.0, "kHz")
    } else {
        (hz, "Hz")
    };
    if val >= 100.0 { format!("{:.0}{}", val, unit) }
    else if val >= 10.0 { format!("{:.1}{}", val, unit) }
    else { format!("{:.2}{}", val, unit) }
}

/// Format Hz → sample-rate string (Sa/s, kSa/s, MSa/s).
pub fn format_sample_rate(hz: f64) -> String {
    if hz < 0.0 { return format_sample_rate(0.0); }
    let (val, unit) = if hz >= 999_500.0 {
        (hz / 1_000_000.0, "MSa/s")
    } else if hz >= 999.5 {
        (hz / 1_000.0, "kSa/s")
    } else {
        (hz, "Sa/s")
    };
    if val >= 100.0 { format!("{:.0}{}", val, unit) }
    else if val >= 10.0 { format!("{:.1}{}", val, unit) }
    else { format!("{:.2}{}", val, unit) }
}

// ── Settings structures ────────────────────────────────────────────────────

/// Parsed per-channel settings.
#[derive(Debug, Clone)]
pub struct ChannelSettings {
    pub enabled: bool,
    pub vdiv_index: u8,
    /// Voltage scale in µV per pixel (scale_uv / 25).
    pub volt_scale_uv: u32,
    /// Zero-volt reference in screen pixels (signed, relative to ADC midpoint 128).
    pub zero_volt_pixels: i32,
    /// Zero-volt reference in µV.
    pub zero_volt_uv: i64,
    /// Trigger level in µV.
    pub trigger_level_uv: i64,
    /// Trigger level in screen pixels.
    pub trigger_level_pixels: i32,
    /// 0=DC, 1=AC.
    pub coupling: u8,
    /// 0=×1, 1=×10, 2=×100.
    pub probe_mode: u8,
    /// Bitmask of enabled auto-measurements.
    pub enabled_measurements: u32,
}

impl ChannelSettings {
    /// Converts a raw (inverted) ADC pixel value to voltage in volts.
    pub fn pixel_to_volts(&self, raw: u8) -> f64 {
        let pixel_val = (255 - raw) as f64;
        let vpp = self.volt_scale_uv as f64 / 25.0 / 1_000_000.0;
        let multiplier = 10f64.powi(self.probe_mode as i32);
        (pixel_val - 128.0 - self.zero_volt_pixels as f64) * vpp * multiplier
    }

    pub fn vdiv_str(&self) -> String {
        format_uv(vdiv_label_uv(self.vdiv_index) as i64, self.probe_mode)
    }
}

/// Parsed oscilloscope settings (instrument state).
#[derive(Debug, Clone)]
pub struct OscSettings {
    pub run_state: u8,
    pub system_state: u8,
    pub ch1: ChannelSettings,
    pub ch2: ChannelSettings,
    /// Timebase index (2=5ns … 30=10s).
    pub timebase_index: u8,
    /// Timebase value in ns/div.
    pub timebase_ns_per_div: u64,
    /// Horizontal trigger delay in picoseconds.
    pub horiz_trigger_delay_ps: i64,
    /// Hardware sample rate in Hz.
    pub sample_rate_hw_hz: f64,
    /// Measurement / display sample rate in Hz.
    pub sample_rate_meas_hz: f64,
    /// Pre-trigger sample count.
    pub pre_trigger_samples: u32,
    /// Post-trigger sample count.
    pub post_trigger_samples: u32,
    /// Total buffer depth in samples.
    pub buffer_depth: i64,
    /// Trigger source: 0=CH1, 1=CH2.
    pub trigger_source: u8,
    /// Trigger mode: 0=Auto, 1=Normal.
    pub trigger_mode: u8,
    /// Trigger edge: 0=Rise, 1=Fall.
    pub trigger_edge: u8,
    /// Trigger level in ADC counts.
    pub trigger_level_adc: u16,
    /// Cursor X enable.
    pub cursors_x_enable: bool,
    /// Cursor Y enable.
    pub cursors_y_enable: bool,
    /// Cursor positions in pixels (signed, relative to centre pixel 150).
    pub cursor_x1: i16,
    pub cursor_x2: i16,
    /// Cursor Y positions in pixels (signed, relative to ADC zero).
    pub cursor_y1: i16,
    pub cursor_y2: i16,
    /// Currently active channel.
    pub active_channel: u8,
    /// Display start/end column (viewport into the buffer).
    pub display_start_col: u16,
    pub display_end_col: u16,
}

impl OscSettings {
    /// Time per pixel in seconds (timebase_ns / 25 / 1e9).
    pub fn time_per_pixel_s(&self) -> f64 {
        self.timebase_ns_per_div as f64 / 25.0 / 1e9
    }

    pub fn timebase_label(&self) -> &'static str {
        timebase_label(self.timebase_index)
    }
}

// ── Measurement structures ─────────────────────────────────────────────────

/// Auto-measurement values decoded from a 100-byte measurement block.
/// Layout: 12 × i64 + 2 × i16 (little-endian).
#[derive(Debug, Clone)]
pub struct Measurements {
    /// Peak-to-peak voltage (µV).
    pub pk_pk_uv: i64,
    /// Amplitude (µV).
    pub amplitude_uv: i64,
    /// Top voltage (µV).
    pub top_uv: i64,
    /// Base voltage (µV).
    pub base_uv: i64,
    /// Max voltage (µV).
    pub max_uv: i64,
    /// Min voltage (µV).
    pub min_uv: i64,
    /// Average voltage (µV).
    pub avg_uv: i64,
    /// RMS voltage (µV).
    pub rms_uv: i64,
    /// Positive pulse width (ps).
    pub pos_width_ps: i64,
    /// Negative pulse width (ps).
    pub neg_width_ps: i64,
    /// Period (ps).
    pub period_ps: i64,
    /// Frequency (µHz — divide by 1e6 to get Hz).
    pub freq_uhz: i64,
    /// Positive duty cycle (0.1%).
    pub pos_duty_tenth_pct: i16,
    /// Negative duty cycle (0.1%).
    pub neg_duty_tenth_pct: i16,
}

impl Measurements {
    fn parse(buf: &[u8], offset: usize, probe_mode: u8) -> Option<Self> {
        let chunk = buf.get(offset..offset + 100)?;
        if chunk.len() < 100 { return None; }
        let v: Vec<i64> = (0..12).map(|i| read_i64(chunk, i * 8)).collect();
        let d0 = read_i16(chunk, 96);
        let d1 = read_i16(chunk, 98);
        let _ = probe_mode; // available for probe-adjusted display
        Some(Measurements {
            pk_pk_uv:          v[0],
            amplitude_uv:      v[1],
            top_uv:            v[2],
            base_uv:           v[3],
            max_uv:            v[4],
            min_uv:            v[5],
            avg_uv:            v[6],
            rms_uv:            v[7],
            pos_width_ps:      v[8],
            neg_width_ps:      v[9],
            period_ps:         v[10],
            freq_uhz:          v[11],
            pos_duty_tenth_pct: d0,
            neg_duty_tenth_pct: d1,
        })
    }

    /// Frequency in Hz.
    pub fn freq_hz(&self) -> f64 {
        self.freq_uhz as f64 / 1_000_000.0
    }

    /// Period in seconds.
    pub fn period_s(&self) -> f64 {
        self.period_ps as f64 / 1e12
    }

    /// Positive duty cycle (0–100%).
    pub fn pos_duty_pct(&self) -> f64 {
        self.pos_duty_tenth_pct as f64 / 10.0
    }
}

// ── Main packet structure ─────────────────────────────────────────────────

/// A fully-parsed ZeeWeii screenshot packet.
#[derive(Debug, Clone)]
pub struct ScreenshotPacket {
    pub device: DeviceVariant,
    /// CH1 minimum ADC values per pixel (raw, inverted: real = 255 - raw).
    pub ch1_min: [u8; WAVEFORM_PIXELS],
    /// CH1 maximum ADC values per pixel.
    pub ch1_max: [u8; WAVEFORM_PIXELS],
    /// CH2 minimum ADC values (present only when CH2 was enabled on capture).
    pub ch2_min: Option<[u8; WAVEFORM_PIXELS]>,
    /// CH2 maximum ADC values.
    pub ch2_max: Option<[u8; WAVEFORM_PIXELS]>,
    /// Oscilloscope settings extracted from the packet.
    pub settings: OscSettings,
    /// CH1 auto-measurements.
    pub ch1_meas: Option<Measurements>,
    /// CH2 auto-measurements (only when CH2 enabled).
    pub ch2_meas: Option<Measurements>,
    /// Length of the original buffer (should be 2048).
    pub raw_len: usize,
}

impl ScreenshotPacket {
    /// Convert the min ADC values to a `Capture` compatible with the rest of
    /// the GUI pipeline.  We use the per-pixel average of min/max for best fidelity.
    /// Values are re-mapped from the ZeeTweak pixel coordinate system
    /// (128 = zero-volt, inverted) back into the 0–255 range the GUI expects.
    pub fn to_capture(&self) -> Capture {
        let ch1: Vec<u8> = (0..WAVEFORM_PIXELS)
            .map(|i| {
                let avg = (self.ch1_min[i] as u16 + self.ch1_max[i] as u16) / 2;
                avg as u8
            })
            .collect();

        let ch2: Option<Vec<u8>> = self.ch2_min.as_ref().map(|mn| {
            let mx = self.ch2_max.as_ref().unwrap();
            (0..WAVEFORM_PIXELS)
                .map(|i| {
                    let avg = (mn[i] as u16 + mx[i] as u16) / 2;
                    avg as u8
                })
                .collect()
        });

        Capture { ch1, ch2 }
    }

    /// Convert to physical waveform: returns (time_s, ch1_volts, ch2_volts_opt)
    /// for every x pixel, using the calibrated scale from settings.
    pub fn to_physical_waveform(&self) -> Vec<PixelPoint> {
        let dt = self.settings.time_per_pixel_s();
        (0..WAVEFORM_PIXELS).map(|i| {
            let t = i as f64 * dt;
            let ch1_min_v = self.settings.ch1.pixel_to_volts(self.ch1_min[i]);
            let ch1_max_v = self.settings.ch1.pixel_to_volts(self.ch1_max[i]);
            let ch2 = if self.settings.ch2.enabled {
                let mn = self.ch2_min.as_ref().map(|a| a[i]).unwrap_or(128);
                let mx = self.ch2_max.as_ref().map(|a| a[i]).unwrap_or(128);
                Some((
                    self.settings.ch2.pixel_to_volts(mn),
                    self.settings.ch2.pixel_to_volts(mx),
                ))
            } else {
                None
            };
            PixelPoint { time_s: t, ch1_min_v, ch1_max_v, ch2 }
        }).collect()
    }

    /// Build a verbose, human-readable report of the entire packet.
    pub fn report_verbose(&self) -> String {
        let s = &self.settings;
        let mut out = String::new();

        let bar = "═".repeat(64);
        out.push_str(&format!("╔{bar}╗\n"));
        out.push_str(&format!("║  ZeeWeii Screenshot Packet — {}  \n", self.device));
        out.push_str(&format!("║  Raw buffer: {} bytes\n", self.raw_len));
        out.push_str(&format!("╚{bar}╝\n\n"));

        // Instrument state
        out.push_str("── Instrument State ──────────────────────────────────────\n");
        out.push_str(&format!("  Run state      : {}\n", run_state_label(s.run_state)));
        out.push_str(&format!("  System state   : {}\n", s.system_state));
        out.push_str(&format!("  Timebase       : {}/div (index {})\n",
            timebase_label(s.timebase_index), s.timebase_index));
        out.push_str(&format!("  Timebase raw   : {} ns/div\n", s.timebase_ns_per_div));
        out.push_str(&format!("  H trigger delay: {}\n",
            format_ps(s.horiz_trigger_delay_ps)));
        out.push_str(&format!("  Sample rate HW : {}\n",
            format_sample_rate(s.sample_rate_hw_hz)));
        out.push_str(&format!("  Sample rate M  : {}\n",
            format_sample_rate(s.sample_rate_meas_hz)));
        out.push_str(&format!("  Buffer depth   : {} samples\n", s.buffer_depth));
        out.push_str(&format!("  Pre-trigger    : {} samples\n", s.pre_trigger_samples));
        out.push_str(&format!("  Post-trigger   : {} samples\n", s.post_trigger_samples));
        out.push_str(&format!("  Display cols   : {} – {}\n",
            s.display_start_col, s.display_end_col));
        out.push_str(&format!("  Time/pixel     : {:.3e} s  ({} total for 300 pixels)\n",
            s.time_per_pixel_s(),
            format_ps((s.time_per_pixel_s() * 300.0 * 1e12) as i64)));

        // Trigger
        out.push_str("\n── Trigger ────────────────────────────────────────────────\n");
        out.push_str(&format!("  Source         : {}\n",
            trigger_source_label(s.trigger_source)));
        out.push_str(&format!("  Mode           : {}\n",
            trigger_mode_label(s.trigger_mode)));
        out.push_str(&format!("  Edge           : {}\n",
            trigger_edge_label(s.trigger_edge)));
        out.push_str(&format!("  Level ADC      : {} (raw count)\n", s.trigger_level_adc));
        let trig_ch = if s.trigger_source == 0 { &s.ch1 } else { &s.ch2 };
        out.push_str(&format!("  Level voltage  : {}\n",
            format_uv(trig_ch.trigger_level_uv, trig_ch.probe_mode)));

        // CH1
        out.push_str("\n── Channel 1 ──────────────────────────────────────────────\n");
        out.push_str(&format!("  Enabled        : {}\n", s.ch1.enabled));
        out.push_str(&format!("  V/Div          : {}/div (index {})\n",
            s.ch1.vdiv_str(), s.ch1.vdiv_index));
        out.push_str(&format!("  Volt scale     : {} µV/px ({}/pixel)\n",
            s.ch1.volt_scale_uv as f64 / 25.0,
            format_uv((s.ch1.volt_scale_uv as f64 / 25.0) as i64, 0)));
        out.push_str(&format!("  Zero V (pixels): {} px\n", s.ch1.zero_volt_pixels));
        out.push_str(&format!("  Zero V (µV)    : {}\n",
            format_uv(s.ch1.zero_volt_uv, 0)));
        out.push_str(&format!("  Coupling       : {}\n", coupling_label(s.ch1.coupling)));
        out.push_str(&format!("  Probe          : {}\n", probe_label(s.ch1.probe_mode)));
        out.push_str(&format!("  Meas flags     : 0x{:04X}\n", s.ch1.enabled_measurements));

        // CH2
        out.push_str("\n── Channel 2 ──────────────────────────────────────────────\n");
        out.push_str(&format!("  Enabled        : {}\n", s.ch2.enabled));
        if s.ch2.enabled {
            out.push_str(&format!("  V/Div          : {}/div (index {})\n",
                s.ch2.vdiv_str(), s.ch2.vdiv_index));
            out.push_str(&format!("  Volt scale     : {} µV/px\n",
                s.ch2.volt_scale_uv as f64 / 25.0));
            out.push_str(&format!("  Zero V (pixels): {} px\n", s.ch2.zero_volt_pixels));
            out.push_str(&format!("  Zero V (µV)    : {}\n",
                format_uv(s.ch2.zero_volt_uv, 0)));
            out.push_str(&format!("  Coupling       : {}\n", coupling_label(s.ch2.coupling)));
            out.push_str(&format!("  Probe          : {}\n", probe_label(s.ch2.probe_mode)));
        }

        // Cursors
        if s.cursors_x_enable || s.cursors_y_enable {
            out.push_str("\n── Cursors ────────────────────────────────────────────────\n");
        }
        if s.cursors_x_enable {
            let dx_px = (s.cursor_x2 - s.cursor_x1).abs() as f64;
            let dt_s = dx_px * s.time_per_pixel_s();
            let dt_ps = (dt_s * 1e12) as i64;
            let freq_hz = if dt_s > 0.0 { 1.0 / dt_s } else { 0.0 };
            out.push_str(&format!("  Cursor X1      : {} px (from centre)\n", s.cursor_x1));
            out.push_str(&format!("  Cursor X2      : {} px\n", s.cursor_x2));
            out.push_str(&format!("  Δt             : {}\n", format_ps(dt_ps)));
            out.push_str(&format!("  Freq (1/Δt)    : {}\n", format_hz(freq_hz)));
        }
        if s.cursors_y_enable {
            let act_ch = if s.active_channel == 0 { &s.ch1 } else { &s.ch2 };
            let uv_per_px = act_ch.volt_scale_uv as f64 / 25.0;
            let dy_px = (s.cursor_y2 - s.cursor_y1).abs() as f64;
            let dy_uv = (dy_px * uv_per_px) as i64;
            let y1_uv = (s.cursor_y1 as f64 * uv_per_px) as i64;
            let y2_uv = (s.cursor_y2 as f64 * uv_per_px) as i64;
            out.push_str(&format!("  Cursor Y1      : {} ({} px)\n",
                format_uv(y1_uv, act_ch.probe_mode), s.cursor_y1));
            out.push_str(&format!("  Cursor Y2      : {} ({} px)\n",
                format_uv(y2_uv, act_ch.probe_mode), s.cursor_y2));
            out.push_str(&format!("  ΔV             : {}\n",
                format_uv(dy_uv, act_ch.probe_mode)));
        }

        // Waveform summary
        out.push_str("\n── Waveform Data Summary ───────────────────────────────────\n");
        let points = self.to_physical_waveform();
        let ch1_v: Vec<f64> = points.iter().map(|p| (p.ch1_min_v + p.ch1_max_v) / 2.0).collect();
        let ch1_min_v = ch1_v.iter().cloned().fold(f64::INFINITY, f64::min);
        let ch1_max_v = ch1_v.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let ch1_avg_v = ch1_v.iter().sum::<f64>() / ch1_v.len() as f64;
        out.push_str(&format!("  CH1  300 pixels, min={:.4}V  max={:.4}V  avg={:.4}V\n",
            ch1_min_v, ch1_max_v, ch1_avg_v));
        if s.ch2.enabled {
            let ch2_v: Vec<f64> = points.iter().filter_map(|p| p.ch2.map(|(mn, mx)| (mn + mx) / 2.0)).collect();
            if !ch2_v.is_empty() {
                let ch2_min_v = ch2_v.iter().cloned().fold(f64::INFINITY, f64::min);
                let ch2_max_v = ch2_v.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                let ch2_avg_v = ch2_v.iter().sum::<f64>() / ch2_v.len() as f64;
                out.push_str(&format!("  CH2  300 pixels, min={:.4}V  max={:.4}V  avg={:.4}V\n",
                    ch2_min_v, ch2_max_v, ch2_avg_v));
            }
        }
        out.push_str(&format!("  Total time span: {}\n",
            format_ps((s.time_per_pixel_s() * WAVEFORM_PIXELS as f64 * 1e12) as i64)));

        // Auto-measurements
        if let Some(ref m) = self.ch1_meas {
            out.push_str("\n── CH1 Auto-Measurements ───────────────────────────────────\n");
            report_measurements(&mut out, m, s.ch1.probe_mode);
        }
        if let Some(ref m) = self.ch2_meas {
            out.push_str("\n── CH2 Auto-Measurements ───────────────────────────────────\n");
            report_measurements(&mut out, m, s.ch2.probe_mode);
        }

        // Raw waveform data (first 16 bytes each channel, hex)
        out.push_str("\n── Raw Waveform Bytes (first 16, hex) ──────────────────────\n");
        let hex_row = |label: &str, data: &[u8]| {
            let hex: Vec<String> = data.iter().take(16).map(|b| format!("{:02X}", b)).collect();
            format!("  {:<8}: {}\n", label, hex.join(" "))
        };
        out.push_str(&hex_row("CH1 min", &self.ch1_min));
        out.push_str(&hex_row("CH1 max", &self.ch1_max));
        if let (Some(mn), Some(mx)) = (&self.ch2_min, &self.ch2_max) {
            out.push_str(&hex_row("CH2 min", mn));
            out.push_str(&hex_row("CH2 max", mx));
        }

        out
    }
}

/// Max plausible measurement voltage (same threshold as container.rs).
const REPORT_MAX_V_UV: i64 = 100_000_000_000; // 100 kV in µV
/// Max plausible pulse width / period (ps).
const REPORT_MAX_T_PS: i64 = 200_000_000_000_000; // 200 s in ps

fn report_measurements(out: &mut String, m: &Measurements, probe: u8) {
    // Format a voltage field, showing "—" for uninitialized garbage.
    let fv = |uv: i64| {
        let mult = 10i64.pow(probe as u32);
        if uv.saturating_mul(mult).abs() > REPORT_MAX_V_UV {
            "—".to_string()
        } else {
            format_uv(uv, probe)
        }
    };
    // Format a time field (ps), showing "—" for uninitialized garbage.
    let ft = |ps: i64| {
        if !(0..=REPORT_MAX_T_PS).contains(&ps) { "—".to_string() } else { format_ps(ps) }
    };

    out.push_str(&format!("  Freq           : {}\n", format_hz(m.freq_hz())));
    out.push_str(&format!("  Period         : {}\n", ft(m.period_ps)));
    out.push_str(&format!("  +Width         : {}\n", ft(m.pos_width_ps)));
    out.push_str(&format!("  -Width         : {}\n", ft(m.neg_width_ps)));
    out.push_str(&format!("  +Duty          : {:.1}%\n", m.pos_duty_pct()));
    out.push_str(&format!("  PkPk           : {}\n", fv(m.pk_pk_uv)));
    out.push_str(&format!("  Amplitude      : {}\n", fv(m.amplitude_uv)));
    out.push_str(&format!("  Max            : {}\n", fv(m.max_uv)));
    out.push_str(&format!("  Min            : {}\n", fv(m.min_uv)));
    out.push_str(&format!("  Top            : {}\n", fv(m.top_uv)));
    out.push_str(&format!("  Base           : {}\n", fv(m.base_uv)));
    out.push_str(&format!("  Avg            : {}\n", fv(m.avg_uv)));
    out.push_str(&format!("  RMS            : {}\n", fv(m.rms_uv)));
}

/// A single calibrated point in the waveform.
#[derive(Debug, Clone)]
pub struct PixelPoint {
    pub time_s: f64,
    pub ch1_min_v: f64,
    pub ch1_max_v: f64,
    /// (min_v, max_v) for CH2, or None if CH2 disabled.
    pub ch2: Option<(f64, f64)>,
}

// ── Parsing functions ──────────────────────────────────────────────────────

/// Parse a 2048-byte screenshot buffer for a DSO3D12.
pub fn parse_screenshot(buf: &[u8]) -> Result<ScreenshotPacket, ParseError> {
    parse_screenshot_as(buf, DeviceVariant::Dso3d12)
}

/// Parse a screenshot buffer with an explicit device variant.
pub fn parse_screenshot_as(buf: &[u8], device: DeviceVariant) -> Result<ScreenshotPacket, ParseError> {
    if buf.len() < 1740 {
        return Err(ParseError::TooShort { got: buf.len(), expected: SCREENSHOT_SIZE });
    }

    let mut ch1_min = [0u8; WAVEFORM_PIXELS];
    let mut ch1_max = [0u8; WAVEFORM_PIXELS];
    ch1_min.copy_from_slice(&buf[0..WAVEFORM_PIXELS]);
    ch1_max.copy_from_slice(&buf[WAVEFORM_PIXELS..2 * WAVEFORM_PIXELS]);

    let settings = match device {
        DeviceVariant::Dso3d12   => parse_settings_dso3d12(buf),
        DeviceVariant::Dso2512gV2 => parse_settings_dso2512g_v2(buf),
        DeviceVariant::Dso2512gV1 => parse_settings_dso2512g_v1(buf),
    };

    let (ch2_min, ch2_max) = if settings.ch2.enabled {
        let mut mn = [0u8; WAVEFORM_PIXELS];
        let mut mx = [0u8; WAVEFORM_PIXELS];
        mn.copy_from_slice(&buf[2 * WAVEFORM_PIXELS..3 * WAVEFORM_PIXELS]);
        mx.copy_from_slice(&buf[3 * WAVEFORM_PIXELS..4 * WAVEFORM_PIXELS]);
        (Some(mn), Some(mx))
    } else {
        (None, None)
    };

    let (meas_ch1_off, meas_ch2_off) = match device {
        DeviceVariant::Dso3d12    => (1540, 1640),
        DeviceVariant::Dso2512gV2 => (1544, 1648),
        DeviceVariant::Dso2512gV1 => (1520, 1624),
    };

    let ch1_meas = Measurements::parse(buf, meas_ch1_off, settings.ch1.probe_mode);
    let ch2_meas = if settings.ch2.enabled {
        Measurements::parse(buf, meas_ch2_off, settings.ch2.probe_mode)
    } else {
        None
    };

    Ok(ScreenshotPacket {
        device,
        ch1_min,
        ch1_max,
        ch2_min,
        ch2_max,
        settings,
        ch1_meas,
        ch2_meas,
        raw_len: buf.len(),
    })
}

fn parse_settings_dso3d12(buf: &[u8]) -> OscSettings {
    let ch2_enabled = read_u8(buf, 1202) != 0;
    OscSettings {
        run_state:              read_u8(buf, 1200),
        system_state:           read_u8(buf, 1201),
        buffer_depth:           read_i64(buf, 1204),
        timebase_index:         read_u8(buf, 1216),
        timebase_ns_per_div:    read_u64(buf, 1220),
        horiz_trigger_delay_ps: read_i64(buf, 1228),
        sample_rate_hw_hz:      read_u32(buf, 1252) as f64 * 100.0,
        trigger_source:         read_u8(buf, 1256),
        trigger_mode:           read_u8(buf, 1257),
        trigger_edge:           read_u8(buf, 1258),
        trigger_level_adc:      read_u16(buf, 1264),
        ch1: ChannelSettings {
            enabled:              true,
            vdiv_index:           read_u8(buf, 1266),
            volt_scale_uv:        read_u32(buf, 1268),
            zero_volt_pixels:     read_i32(buf, 1272),
            zero_volt_uv:         read_i64(buf, 1276),
            trigger_level_uv:     read_i64(buf, 1292),
            trigger_level_pixels: read_i32(buf, 1300),
            coupling:             read_u8(buf, 1304),
            probe_mode:           read_u8(buf, 1305),
            enabled_measurements: read_u32(buf, 1308),
        },
        ch2: ChannelSettings {
            enabled:              ch2_enabled,
            vdiv_index:           read_u8(buf, 1328),
            volt_scale_uv:        read_u32(buf, 1332),
            zero_volt_pixels:     read_i32(buf, 1336),
            zero_volt_uv:         read_i64(buf, 1340),
            trigger_level_uv:     read_i64(buf, 1356),
            trigger_level_pixels: read_i32(buf, 1364),
            coupling:             read_u8(buf, 1368),
            probe_mode:           read_u8(buf, 1369),
            enabled_measurements: read_u32(buf, 1372),
        },
        sample_rate_meas_hz:    read_u64(buf, 1392) as f64,
        pre_trigger_samples:    read_u32(buf, 1400),
        post_trigger_samples:   read_u32(buf, 1404),
        display_start_col:      read_u16(buf, 1408),
        display_end_col:        read_u16(buf, 1410),
        cursors_x_enable:       read_u8(buf, 1453) != 0,
        cursors_y_enable:       read_u8(buf, 1454) != 0,
        cursor_x1:              read_i16(buf, 1456),
        cursor_x2:              read_i16(buf, 1458),
        cursor_y1:              read_i16(buf, 1460),
        cursor_y2:              read_i16(buf, 1462),
        active_channel:         read_u8(buf, 1486),
    }
}

fn parse_settings_dso2512g_v2(buf: &[u8]) -> OscSettings {
    let ch2_enabled = read_u8(buf, 1202) != 0;
    OscSettings {
        run_state:              read_u8(buf, 1200),
        system_state:           read_u8(buf, 1201),
        buffer_depth:           read_u64(buf, 1208) as i64,
        timebase_index:         read_u8(buf, 1220),
        timebase_ns_per_div:    read_u64(buf, 1224),
        horiz_trigger_delay_ps: read_i64(buf, 1232),
        sample_rate_hw_hz:      read_u64(buf, 1256) as f64 * 100.0,
        trigger_source:         read_u8(buf, 1264),
        trigger_mode:           read_u8(buf, 1265),
        trigger_edge:           read_u8(buf, 1266),
        trigger_level_adc:      read_u16(buf, 1272),
        ch1: ChannelSettings {
            enabled:              true,
            vdiv_index:           read_u8(buf, 1274),
            volt_scale_uv:        read_u32(buf, 1276),
            zero_volt_pixels:     read_i32(buf, 1280),
            zero_volt_uv:         read_i64(buf, 1288),
            trigger_level_uv:     read_i64(buf, 1304),
            trigger_level_pixels: read_i32(buf, 1312),
            coupling:             read_u8(buf, 1316),
            probe_mode:           read_u8(buf, 1317),
            enabled_measurements: read_u32(buf, 1320),
        },
        ch2: ChannelSettings {
            enabled:              ch2_enabled,
            vdiv_index:           read_u8(buf, 1340),
            volt_scale_uv:        read_u32(buf, 1344),
            zero_volt_pixels:     read_i32(buf, 1348),
            zero_volt_uv:         read_i64(buf, 1352),
            trigger_level_uv:     read_i64(buf, 1368),
            trigger_level_pixels: read_i32(buf, 1376),
            coupling:             read_u8(buf, 1380),
            probe_mode:           read_u8(buf, 1381),
            enabled_measurements: read_u32(buf, 1384),
        },
        sample_rate_meas_hz:    read_u64(buf, 1408) as f64,
        pre_trigger_samples:    read_u32(buf, 1416),
        post_trigger_samples:   read_u32(buf, 1420),
        display_start_col:      read_u16(buf, 1424),
        display_end_col:        read_u16(buf, 1426),
        cursors_x_enable:       read_u8(buf, 1469) != 0,
        cursors_y_enable:       read_u8(buf, 1470) != 0,
        cursor_x1:              read_i16(buf, 1472),
        cursor_x2:              read_i16(buf, 1474),
        cursor_y1:              read_i16(buf, 1476),
        cursor_y2:              read_i16(buf, 1478),
        active_channel:         read_u8(buf, 1502),
    }
}

fn parse_settings_dso2512g_v1(buf: &[u8]) -> OscSettings {
    // V1 has the same layout as V2 except some minor offsets.
    let ch2_enabled = read_u8(buf, 1202) != 0;
    OscSettings {
        run_state:              read_u8(buf, 1200),
        system_state:           read_u8(buf, 1201),
        buffer_depth:           read_u64(buf, 1208) as i64,
        timebase_index:         read_u8(buf, 1220),
        timebase_ns_per_div:    read_u64(buf, 1224),
        horiz_trigger_delay_ps: read_i64(buf, 1232),
        sample_rate_hw_hz:      read_u64(buf, 1256) as f64 * 100.0,
        trigger_source:         read_u8(buf, 1264),
        trigger_mode:           read_u8(buf, 1265),
        trigger_edge:           read_u8(buf, 1266),
        trigger_level_adc:      read_u16(buf, 1272),
        ch1: ChannelSettings {
            enabled:              true,
            vdiv_index:           read_u8(buf, 1274),
            volt_scale_uv:        read_u32(buf, 1276),
            zero_volt_pixels:     read_i32(buf, 1280),
            zero_volt_uv:         read_i64(buf, 1288),
            trigger_level_uv:     read_i64(buf, 1304),
            trigger_level_pixels: read_i32(buf, 1312),
            coupling:             read_u8(buf, 1316),
            probe_mode:           read_u8(buf, 1317),
            enabled_measurements: read_u32(buf, 1320),
        },
        ch2: ChannelSettings {
            enabled:              ch2_enabled,
            vdiv_index:           read_u8(buf, 1340),
            volt_scale_uv:        read_u32(buf, 1344),
            zero_volt_pixels:     read_i32(buf, 1348),
            zero_volt_uv:         read_i64(buf, 1352),
            trigger_level_uv:     read_i64(buf, 1368),
            trigger_level_pixels: read_i32(buf, 1376),
            coupling:             read_u8(buf, 1380),
            probe_mode:           read_u8(buf, 1381),
            enabled_measurements: read_u32(buf, 1384),
        },
        sample_rate_meas_hz:    read_u64(buf, 1408) as f64 * 10.0,  // V1: ×10
        pre_trigger_samples:    read_u32(buf, 1416),
        post_trigger_samples:   read_u32(buf, 1420),
        display_start_col:      read_u16(buf, 1424),
        display_end_col:        read_u16(buf, 1426),
        cursors_x_enable:       read_u8(buf, 1465) != 0,
        cursors_y_enable:       read_u8(buf, 1466) != 0,
        cursor_x1:              read_i16(buf, 1468),
        cursor_x2:              read_i16(buf, 1470),
        cursor_y1:              read_i16(buf, 1472),
        cursor_y2:              read_i16(buf, 1474),
        active_channel:         read_u8(buf, 1484),
    }
}

/// Returns true if the buffer looks like a screenshot packet (size + rough sanity check).
pub fn is_screenshot_packet(buf: &[u8]) -> bool {
    // ZeeTweak accepts 2040–2056 bytes as valid screenshot packets.
    (2034..=2056).contains(&buf.len())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_uv_cases() {
        assert_eq!(format_uv(0, 0), "0.00V");
        assert_eq!(format_uv(1_000_000, 0), "1.00V");
        assert_eq!(format_uv(500_000, 0), "500.0mV");
        assert_eq!(format_uv(1_000, 0), "1.0mV");
        assert_eq!(format_uv(500, 0), "500.0µV");
    }

    #[test]
    fn format_ps_cases() {
        assert_eq!(format_ps(0), "0.00s");
        assert_eq!(format_ps(1_000_000_000), "1.00ms");
        assert_eq!(format_ps(1_000_000_000_000), "1.00s");
    }

    #[test]
    fn format_hz_cases() {
        assert_eq!(format_hz(1000.0), "1.00kHz");
        assert_eq!(format_hz(1_000_000.0), "1.00MHz");
        assert_eq!(format_hz(50.0), "50.0Hz");
    }

    #[test]
    fn screenshot_too_short_error() {
        let buf = vec![0u8; 100];
        assert!(parse_screenshot(&buf).is_err());
    }

    #[test]
    fn is_screenshot_packet_size_check() {
        assert!(is_screenshot_packet(&vec![0u8; 2048]));
        assert!(is_screenshot_packet(&vec![0u8; 2040]));
        assert!(!is_screenshot_packet(&vec![0u8; 1200]));
        assert!(!is_screenshot_packet(&[0u8; 100]));
    }

    #[test]
    fn parse_zeroed_packet() {
        // A zeroed 2048-byte buffer should parse without panic.
        let buf = vec![0u8; 2048];
        let pkt = parse_screenshot(&buf).unwrap();
        assert!(!pkt.settings.ch2.enabled);
        assert_eq!(pkt.ch2_min, None);
        let report = pkt.report_verbose();
        assert!(report.contains("DSO3D12"));
    }

    #[test]
    fn pixel_to_volts_identity() {
        // A channel with 100000 µV scale, zero at pixel 0, ×1 probe.
        let ch = ChannelSettings {
            enabled: true,
            vdiv_index: 7,
            volt_scale_uv: 100_000,     // 100mV/div
            zero_volt_pixels: 0,
            zero_volt_uv: 0,
            trigger_level_uv: 0,
            trigger_level_pixels: 0,
            coupling: 0,
            probe_mode: 0,
            enabled_measurements: 0,
        };
        // raw=128 → pixel_val=127 → (127-128-0)*scale = -1 × (100000/25/1e6) ≈ -4mV
        let v = ch.pixel_to_volts(128);
        assert!((v - -0.004).abs() < 1e-6, "v={v}");
    }
}

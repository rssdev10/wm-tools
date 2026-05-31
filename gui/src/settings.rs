//! Persisted user settings, stored as JSON under the OS config dir.

use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Settings {
    /// Last-used serial port (persisted as default for next launch).
    pub serial_port: Option<String>,
    pub baud_rate: u32,
    pub show_instructions: bool,
    pub show_ch1: bool,
    pub show_ch2: bool,
    /// Show user-controlled measurement cursors on the X axis.
    pub measurement_cursor_x_enabled: bool,
    /// Show user-controlled measurement cursors on the Y axis.
    pub measurement_cursor_y_enabled: bool,
    /// Show device-supplied cursors (if present in capture data).
    pub device_cursor_x_enabled: bool,
    pub device_cursor_y_enabled: bool,
    pub view_mode: ViewMode,
    /// X measurement cursor range (low, high), each 0.0..1.0 of sample window.
    pub cursor_x_range: (f32, f32),
    /// Y measurement cursor range (low, high), each 0.0..1.0 of vertical axis.
    pub cursor_y_range: (f32, f32),
    /// Show numeric axis scales on the graph.
    pub show_scales: bool,
    /// Split CH1/CH2 into separate subimages in PNG export.
    pub split_png: bool,
    /// Play a beep when serial data arrives.
    pub alert_on_data: bool,
    /// Automatically start listening when the configured port appears.
    pub auto_listen: bool,
    /// Volts per division for the scope grid. Default 1.0 V/div.
    #[serde(default = "default_v_per_div")]
    pub v_per_div: f64,
    /// Time per division in milliseconds. Default 1.0 ms/div.
    #[serde(default = "default_t_per_div_ms")]
    pub t_per_div_ms: f64,
    /// Voltage offset (zero-level of graph) in volts. Default 0.0 V.
    #[serde(default)]
    pub v_offset: f64,
}

fn default_v_per_div() -> f64 {
    1.0
}

fn default_t_per_div_ms() -> f64 {
    1.0
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ViewMode {
    Line,
    Dot,
    Smooth,
}

impl ViewMode {
    pub const ALL: [ViewMode; 3] = [ViewMode::Line, ViewMode::Dot, ViewMode::Smooth];
}

impl std::fmt::Display for ViewMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            ViewMode::Line => "Line",
            ViewMode::Dot => "Dot",
            ViewMode::Smooth => "Smooth",
        })
    }
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            serial_port: None,
            baud_rate: 115200,
            show_instructions: true,
            show_ch1: true,
            show_ch2: true,
            measurement_cursor_x_enabled: false,
            measurement_cursor_y_enabled: false,
            device_cursor_x_enabled: false,
            device_cursor_y_enabled: false,
            view_mode: ViewMode::Line,
            cursor_x_range: (0.3, 0.7),
            cursor_y_range: (0.3, 0.7),
            show_scales: false,
            split_png: false,
            alert_on_data: true,
            auto_listen: false,
            v_per_div: 1.0,
            t_per_div_ms: 1.0,
            v_offset: 0.0,
        }
    }
}

pub fn config_dir_path() -> Option<PathBuf> {
    let dir = dirs::config_dir()?.join("dso3d12-gui");
    std::fs::create_dir_all(&dir).ok()?;
    Some(dir)
}

fn config_path() -> Option<PathBuf> {
    config_dir_path().map(|d| d.join("settings.json"))
}

impl Settings {
    pub fn load() -> Self {
        let Some(path) = config_path() else {
            return Self::default();
        };
        match std::fs::read_to_string(&path) {
            Ok(text) => serde_json::from_str(&text).unwrap_or_default(),
            Err(_) => Self::default(),
        }
    }

    pub fn save(&self) {
        let Some(path) = config_path() else { return };
        if let Ok(text) = serde_json::to_string_pretty(self) {
            let _ = std::fs::write(path, text);
        }
    }
}

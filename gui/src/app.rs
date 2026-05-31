//! Application state, messages, update and view (iced 0.14).

use std::path::PathBuf;

use chrono::{DateTime, Local};
use dso_parser::{Capture, DEFAULT_BUFFER_DEPTH};
use iced::widget::{
    button, canvas, checkbox, column, container, pick_list, progress_bar, row, rule,
    scrollable, text, text_input, Space,
};
use iced::{Element, Length, Subscription, Task, Theme};

use crate::canvas::{Scope, Thumbnail};
use crate::range_slider;
use crate::serial::{self, SerialConfig, SerialEvent, SerialHandle};
use crate::settings::{Settings, ViewMode};

// ── Messages ───────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub enum Message {
    // File actions
    LoadClicked,
    FilePicked(Option<PathBuf>),
    Loaded(Result<LoadedDump, String>),
    SaveCaptureClicked,
    SaveCapturePicked(Option<PathBuf>),
    SavePngClicked,
    PngPathPicked(Option<PathBuf>),
    ExportCsvClicked,
    ExportCsvPicked(Option<PathBuf>),
    // Capture selection
    SelectCapture(usize),
    // Serial port
    PortSelected(String),
    StartListening,
    StopListening,
    SerialEvent(#[allow(dead_code)] SerialEvent),
    // Channel & view (below-graph control panel)
    ToggleCh1(bool),
    ToggleCh2(bool),
    ToggleDeviceCursorX(bool),
    ToggleDeviceCursorY(bool),
    ToggleShowScales(bool),
    ToggleSplitPng(bool),
    ViewModeSelected(ViewMode),
    // Measurement cursors (right panel)
    ToggleMeasCursorX(bool),
    ToggleMeasCursorY(bool),
    MeasCursorXChanged(f32, f32),
    MeasCursorYChanged(f32, f32),
    // Scale inputs (V/div, time/div, V offset)
    VPerCellChanged(String),
    TPerCellChanged(String),
    VOffsetChanged(String),
    // Graph click — move nearest cursor
    GraphClicked(f32, f32),
    // Misc
    ToggleInstructions(bool),
    ToggleAlertOnData(bool),
    ToggleAutoListen(bool),
    // Settings dialog
    OpenSettings,
    CloseSettings,
    SettingsBaudChanged(String),
    SettingsPortChanged(String),
    // Firmware dialog
    OpenFirmware,
    CloseFirmware,
    FirmwarePortSelected(String),
    FirmwareRefreshPorts,
    FirmwareFileClicked,
    FirmwareFilePicked(Option<PathBuf>),
    FirmwareConfirmStart,
    FirmwareStart,
    FirmwareEvent(#[allow(dead_code)] crate::flash::FlashEvent),
    // Port scan tick (auto-refresh)
    PortScanTick,
    // Open a URL in the default browser
    OpenUrl(String),
    // Context menu on thumbnails (right-click via iced_aw::ContextMenu)
    ContextMenuDeleteAt(usize),
    ContextMenuExportPngAt(usize),
    ContextMenuExportPngPicked(Option<PathBuf>),
    ContextMenuExportCsvAt(usize),
    ContextMenuExportCsvPicked(Option<PathBuf>),
}

/// A capture paired with the timestamp of when it was received/loaded.
/// Holds both the legacy `Capture` (for rendering) and the rich `CaptureRecord`
/// (for metadata, measurements, and serialization).
#[derive(Debug, Clone)]
pub struct CaptureEntry {
    pub capture: Capture,
    pub timestamp: DateTime<Local>,
    /// Full record with metadata (always present for new captures; may be
    /// synthesized from legacy data when loading old files).
    pub record: dso_parser::CaptureRecord,
}

#[derive(Debug, Clone)]
pub struct LoadedDump {
    pub path: PathBuf,
    pub captures: Vec<CaptureEntry>,
}

// ── State ──────────────────────────────────────────────────────────────────

pub struct App {
    pub(crate) settings: Settings,
    pub(crate) dump: Option<LoadedDump>,
    pub(crate) current: usize,
    pub(crate) status: String,
    pub(crate) available_ports: Vec<String>,
    pub(crate) listening: bool,
    serial_handle: Option<SerialHandle>,
    serial_rx: Option<tokio::sync::mpsc::UnboundedReceiver<SerialEvent>>,
    /// Progress bytes received so far during a live capture.
    capture_progress: usize,
    /// Settings dialog open flag.
    show_settings: bool,
    /// Editable fields in settings dialog.
    settings_baud_input: String,
    settings_port_input: String,
    /// Firmware dialog state.
    show_firmware: bool,
    firmware_port: String,
    firmware_file: Option<PathBuf>,
    firmware_progress: Option<(u64, u64)>,
    firmware_status: String,
    firmware_handle: Option<crate::flash::FlashHandle>,
    firmware_rx: Option<tokio::sync::mpsc::UnboundedReceiver<crate::flash::FlashEvent>>,
    firmware_confirming: bool,
    /// Whether we were listening before firmware update (to auto-reconnect after).
    was_listening_before_flash: bool,
    /// Target capture index for context menu export operations.
    context_export_idx: Option<usize>,
    /// Text input state for V/div scale.
    v_per_div_input: String,
    /// Text input state for t/div scale.
    t_per_div_input: String,
    /// Text input state for V/offset.
    v_offset_input: String,
}

impl App {
    pub fn new() -> (Self, Task<Message>) {
        let settings = Settings::load();
        let ports = serial::list_ports();
        log::info!(
            "starting GUI; persisted port = {:?}; available ports = {:?}",
            settings.serial_port,
            ports
        );
        let baud_str = settings.baud_rate.to_string();
        let port_str = settings.serial_port.clone().unwrap_or_default();
        let v_per_div_str = format_float(settings.v_per_div);
        let t_per_div_str = format_float(settings.t_per_div_ms);
        let v_offset_str = format_float(settings.v_offset);
        (
            Self {
                settings,
                dump: None,
                current: 0,
                status: "Ready.".to_string(),
                available_ports: ports,
                listening: false,
                serial_handle: None,
                serial_rx: None,
                capture_progress: 0,
                show_settings: false,
                settings_baud_input: baud_str,
                settings_port_input: port_str,
                show_firmware: false,
                firmware_port: String::new(),
                firmware_file: None,
                firmware_progress: None,
                firmware_status: String::new(),
                firmware_handle: None,
                firmware_rx: None,
                firmware_confirming: false,
                was_listening_before_flash: false,
                context_export_idx: None,
                v_per_div_input: v_per_div_str,
                t_per_div_input: t_per_div_str,
                v_offset_input: v_offset_str,
            },
            Task::none(),
        )
    }

    pub fn title(&self) -> String {
        let ver = env!("CARGO_PKG_VERSION");
        match &self.dump {
            Some(d) => format!(
                "DSO3D12 Viewer v{ver} — {} ({} capture{})",
                d.path.file_name().and_then(|s| s.to_str()).unwrap_or("?"),
                d.captures.len(),
                if d.captures.len() == 1 { "" } else { "s" }
            ),
            None => format!("DSO3D12 Viewer v{ver}"),
        }
    }

    pub fn theme(&self) -> Theme {
        Theme::Dark
    }

    pub(crate) fn current_capture(&self) -> Option<&Capture> {
        self.dump
            .as_ref()
            .and_then(|d| d.captures.get(self.current))
            .map(|e| &e.capture)
    }

    pub(crate) fn current_entry(&self) -> Option<&CaptureEntry> {
        self.dump
            .as_ref()
            .and_then(|d| d.captures.get(self.current))
    }

    pub(crate) fn capture_has_device_cursor_x(&self) -> bool {
        self.current_entry()
            .and_then(|e| e.record.scope_state.as_ref())
            .map(|s| s.cursors_x_enable)
            .unwrap_or(false)
    }
    pub(crate) fn capture_has_device_cursor_y(&self) -> bool {
        self.current_entry()
            .and_then(|e| e.record.scope_state.as_ref())
            .map(|s| s.cursors_y_enable)
            .unwrap_or(false)
    }

    // ── Subscription for serial events ─────────────────────────────────

    pub fn subscription(&self) -> Subscription<Message> {
        let mut subs = vec![];

        // Poll serial receiver
        if self.listening && self.serial_rx.is_some() {
            subs.push(
                iced::time::every(std::time::Duration::from_millis(50))
                    .map(|_| Message::SerialEvent(SerialEvent::Progress(0))),
            );
        }

        // Auto port scan every 10s
        subs.push(
            iced::time::every(std::time::Duration::from_secs(10))
                .map(|_| Message::PortScanTick),
        );

        // Poll firmware events
        if self.firmware_rx.is_some() {
            subs.push(
                iced::time::every(std::time::Duration::from_millis(100))
                    .map(|_| Message::FirmwareEvent(crate::flash::FlashEvent::Status(String::new()))),
            );
        }

        Subscription::batch(subs)
    }

    // ── Update ─────────────────────────────────────────────────────────

    pub fn update(&mut self, msg: Message) -> Task<Message> {
        match msg {
            Message::LoadClicked => Task::perform(pick_file(), Message::FilePicked),
            Message::FilePicked(None) => Task::none(),
            Message::FilePicked(Some(path)) => Task::perform(load_dump(path), |r| {
                Message::Loaded(r.map_err(|e| e.to_string()))
            }),
            Message::Loaded(Ok(d)) => {
                self.status = format!(
                    "Loaded {} capture{} from {}",
                    d.captures.len(),
                    if d.captures.len() == 1 { "" } else { "s" },
                    d.path.display()
                );
                log::info!("{}", self.status);
                self.current = 0;
                self.dump = Some(d);
                Task::none()
            }
            Message::Loaded(Err(e)) => {
                self.status = format!("Load error: {e}");
                log::error!("load error: {e}");
                Task::none()
            }
            Message::SaveCaptureClicked => {
                Task::perform(pick_save_bin(), Message::SaveCapturePicked)
            }
            Message::SaveCapturePicked(None) => Task::none(),
            Message::SaveCapturePicked(Some(path)) => {
                if let Some(dump) = &self.dump {
                    match save_all_captures(&path, &dump.captures) {
                        Ok(()) => self.status = format!("Saved {} capture(s) to {}", dump.captures.len(), path.display()),
                        Err(e) => {
                            self.status = format!("Save error: {e}");
                            log::error!("save error: {e}");
                        }
                    }
                }
                Task::none()
            }
            Message::SavePngClicked => {
                let ts = self.current_entry().map(|e| e.timestamp);
                Task::perform(pick_save_png(ts), Message::PngPathPicked)
            }
            Message::PngPathPicked(None) => Task::none(),
            Message::PngPathPicked(Some(path)) => {
                if let Some(cap) = self.current_capture().cloned() {
                    match export_png(&path, &cap, &self.settings) {
                        Ok(()) => self.status = format!("PNG saved to {}", path.display()),
                        Err(e) => {
                            self.status = format!("PNG error: {e}");
                            log::error!("PNG error: {e}");
                        }
                    }
                } else {
                    self.status = "No capture to export.".to_string();
                }
                Task::none()
            },
            Message::ExportCsvClicked => {
                let ts = self.current_entry().map(|e| e.timestamp);
                Task::perform(pick_save_csv(ts), Message::ExportCsvPicked)
            }
            Message::ExportCsvPicked(None) => Task::none(),
            Message::ExportCsvPicked(Some(path)) => {
                if let Some(cap) = self.current_capture() {
                    match export_csv(&path, cap) {
                        Ok(()) => self.status = format!("Wrote {}", path.display()),
                        Err(e) => {
                            self.status = format!("CSV error: {e}");
                            log::error!("CSV error: {e}");
                        }
                    }
                }
                Task::none()
            }
            Message::SelectCapture(i) => {
                if let Some(d) = &self.dump {
                    if i < d.captures.len() {
                        self.current = i;
                    }
                }
                Task::none()
            }
            Message::PortSelected(p) => {
                log::info!("port selected: {p}");
                self.settings.serial_port = Some(p);
                self.settings.save();
                // Refresh ports
                self.available_ports = serial::list_ports();
                Task::none()
            }
            Message::StartListening => {
                let port = match &self.settings.serial_port {
                    Some(p) if !p.is_empty() => p.clone(),
                    _ => {
                        self.status = "Select a serial port first.".to_string();
                        return Task::none();
                    }
                };
                let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
                let config = SerialConfig {
                    port,
                    baud_rate: self.settings.baud_rate,
                };
                let handle = serial::start_listening(config, tx);
                self.serial_handle = Some(handle);
                self.serial_rx = Some(rx);
                self.listening = true;
                self.capture_progress = 0;
                self.status = format!("Listening on {}…", self.settings.serial_port.as_deref().unwrap_or("?"));
                Task::none()
            }
            Message::StopListening => {
                if let Some(mut h) = self.serial_handle.take() {
                    h.stop();
                }
                self.serial_rx = None;
                self.listening = false;
                self.status = "Stopped.".to_string();
                Task::none()
            }
            Message::SerialEvent(_) => {
                // Drain all pending events from the channel.
                if let Some(rx) = &mut self.serial_rx {
                    while let Ok(evt) = rx.try_recv() {
                        match evt {
                            SerialEvent::Connected => {
                                self.status = format!(
                                    "Connected to {}.",
                                    self.settings.serial_port.as_deref().unwrap_or("?")
                                );
                                log::info!("{}", self.status);
                            }
                            SerialEvent::Disconnected => {
                                self.status = "Port disconnected; reconnecting…".to_string();
                                log::warn!("{}", self.status);
                            }
                            SerialEvent::Progress(n) => {
                                if n > 0 {
                                    // Beep when data first starts arriving
                                    if self.capture_progress == 0 && self.settings.alert_on_data {
                                        play_beep_start();
                                    }
                                    self.capture_progress = n;
                                    self.status = format!("Receiving… {n} bytes");
                                }
                            }
                            SerialEvent::CaptureReceived(captures) => {
                                let count = captures.len();
                                log::info!(
                                    "received {count} capture(s) from serial, ch1 len = {}",
                                    captures.first().map(|c| c.ch1.len()).unwrap_or(0)
                                );
                                // Beep when capture is complete
                                if self.settings.alert_on_data {
                                    play_beep_end();
                                }
                                // Wrap captures with timestamps
                                let now = Local::now();
                                let entries: Vec<CaptureEntry> = captures
                                    .into_iter()
                                    .map(|capture| {
                                        let record = dso_parser::capture_to_record(&capture, now, None);
                                        CaptureEntry { capture, timestamp: now, record }
                                    })
                                    .collect();
                                // Append to existing dump rather than replacing.
                                if let Some(ref mut dump) = self.dump {
                                    let prev_len = dump.captures.len();
                                    dump.captures.extend(entries);
                                    // Auto-select the newly received capture.
                                    self.current = prev_len;
                                } else {
                                    self.dump = Some(LoadedDump {
                                        path: PathBuf::from("<serial>"),
                                        captures: entries,
                                    });
                                    self.current = 0;
                                }
                                self.capture_progress = 0;
                                self.status = format!(
                                    "Received {count} capture(s). Total: {}",
                                    self.dump.as_ref().map(|d| d.captures.len()).unwrap_or(0)
                                );
                            }
                            SerialEvent::ScreenshotReceived(raw_bytes) => {
                                log::info!("received screenshot packet ({} bytes)", raw_bytes.len());
                                if self.settings.alert_on_data {
                                    play_beep_end();
                                }
                                let now = Local::now();
                                match dso_parser::parse_screenshot(&raw_bytes) {
                                    Ok(pkt) => {
                                        let record = dso_parser::screenshot_to_record(&pkt, now, &raw_bytes);
                                        let capture = pkt.to_capture();
                                        // Auto-fill scale from screenshot metadata
                                        let s = &pkt.settings;
                                        let probe1 = 10f64.powi(s.ch1.probe_mode as i32);
                                        self.settings.v_per_div = s.ch1.volt_scale_uv as f64 / 1_000_000.0 * probe1;
                                        self.settings.t_per_div_ms = s.timebase_ns_per_div as f64 / 1_000_000.0;
                                        self.settings.v_offset = s.ch1.zero_volt_uv as f64 / 1_000_000.0 * probe1;
                                        self.v_per_div_input = format_float(self.settings.v_per_div);
                                        self.t_per_div_input = format_float(self.settings.t_per_div_ms);
                                        self.v_offset_input = format_float(self.settings.v_offset);
                                        self.settings.save();

                                        let entry = CaptureEntry { capture, timestamp: now, record };
                                        if let Some(ref mut dump) = self.dump {
                                            let prev_len = dump.captures.len();
                                            dump.captures.push(entry);
                                            self.current = prev_len;
                                        } else {
                                            self.dump = Some(LoadedDump {
                                                path: PathBuf::from("<serial>"),
                                                captures: vec![entry],
                                            });
                                            self.current = 0;
                                        }
                                        self.capture_progress = 0;
                                        self.status = format!(
                                            "Screenshot received ({}/div, {}/div). Total: {}",
                                            dso_parser::format_uv(
                                                (self.settings.v_per_div * 1_000_000.0) as i64, 0
                                            ),
                                            s.timebase_label(),
                                            self.dump.as_ref().map(|d| d.captures.len()).unwrap_or(0)
                                        );
                                    }
                                    Err(e) => {
                                        self.status = format!("Screenshot parse error: {e}");
                                        log::error!("screenshot parse: {e}");
                                    }
                                }
                            }
                            SerialEvent::Error(e) => {
                                self.status = format!("Serial: {e}");
                                log::error!("serial error: {e}");
                            }
                        }
                    }
                }
                Task::none()
            }
            Message::ToggleCh1(v) => {
                self.settings.show_ch1 = v;
                self.settings.save();
                Task::none()
            }
            Message::ToggleCh2(v) => {
                self.settings.show_ch2 = v;
                self.settings.save();
                Task::none()
            }
            Message::ToggleDeviceCursorX(v) => {
                self.settings.device_cursor_x_enabled = v;
                self.settings.save();
                Task::none()
            }
            Message::ToggleDeviceCursorY(v) => {
                self.settings.device_cursor_y_enabled = v;
                self.settings.save();
                Task::none()
            }
            Message::ViewModeSelected(m) => {
                self.settings.view_mode = m;
                self.settings.save();
                Task::none()
            }
            Message::ToggleMeasCursorX(v) => {
                self.settings.measurement_cursor_x_enabled = v;
                self.settings.save();
                Task::none()
            }
            Message::ToggleMeasCursorY(v) => {
                self.settings.measurement_cursor_y_enabled = v;
                self.settings.save();
                Task::none()
            }
            Message::MeasCursorXChanged(lo, hi) => {
                self.settings.cursor_x_range = (lo, hi);
                Task::none()
            }
            Message::MeasCursorYChanged(lo, hi) => {
                self.settings.cursor_y_range = (lo, hi);
                Task::none()
            }
            Message::VPerCellChanged(s) => {
                self.v_per_div_input = s.clone();
                // Accept both decimal ("0.001") and scientific notation ("1e-3")
                if let Ok(v) = s.parse::<f64>() {
                    if v > 0.0 && v.is_finite() {
                        self.settings.v_per_div = v;
                        self.settings.save();
                    }
                }
                Task::none()
            }
            Message::TPerCellChanged(s) => {
                self.t_per_div_input = s.clone();
                if let Ok(v) = s.parse::<f64>() {
                    if v > 0.0 && v.is_finite() {
                        self.settings.t_per_div_ms = v;
                        self.settings.save();
                    }
                }
                Task::none()
            }
            Message::VOffsetChanged(s) => {
                self.v_offset_input = s.clone();
                if let Ok(v) = s.parse::<f64>() {
                    if v.is_finite() {
                        self.settings.v_offset = v;
                        self.settings.save();
                    }
                }
                Task::none()
            }
            Message::GraphClicked(frac_x, frac_y) => {
                // Move nearest cursor to the click position.
                // If X cursors enabled, move nearest X cursor knob.
                if self.settings.measurement_cursor_x_enabled {
                    let (lo, hi) = self.settings.cursor_x_range;
                    let dist_lo = (frac_x - lo).abs();
                    let dist_hi = (frac_x - hi).abs();
                    if dist_lo <= dist_hi {
                        self.settings.cursor_x_range.0 = frac_x.clamp(0.0, hi);
                    } else {
                        self.settings.cursor_x_range.1 = frac_x.clamp(lo, 1.0);
                    }
                }
                // If Y cursors enabled, move nearest Y cursor knob.
                if self.settings.measurement_cursor_y_enabled {
                    let (lo, hi) = self.settings.cursor_y_range;
                    let dist_lo = (frac_y - lo).abs();
                    let dist_hi = (frac_y - hi).abs();
                    if dist_lo <= dist_hi {
                        self.settings.cursor_y_range.0 = frac_y.clamp(0.0, hi);
                    } else {
                        self.settings.cursor_y_range.1 = frac_y.clamp(lo, 1.0);
                    }
                }
                Task::none()
            }
            Message::ToggleInstructions(v) => {
                self.settings.show_instructions = v;
                self.settings.save();
                Task::none()
            }
            Message::OpenSettings => {
                self.show_settings = true;
                self.settings_baud_input = self.settings.baud_rate.to_string();
                self.settings_port_input = self.settings.serial_port.clone().unwrap_or_default();
                Task::none()
            }
            Message::CloseSettings => {
                // Apply settings
                if let Ok(b) = self.settings_baud_input.parse::<u32>() {
                    if b > 0 {
                        self.settings.baud_rate = b;
                    }
                }
                let p = self.settings_port_input.trim().to_string();
                self.settings.serial_port = if p.is_empty() { None } else { Some(p) };
                self.settings.save();
                self.show_settings = false;
                self.status = "Settings saved.".to_string();
                Task::none()
            }
            Message::SettingsBaudChanged(s) => {
                self.settings_baud_input = s;
                Task::none()
            }
            Message::SettingsPortChanged(s) => {
                self.settings_port_input = s;
                Task::none()
            }
            Message::ToggleShowScales(v) => {
                self.settings.show_scales = v;
                self.settings.save();
                Task::none()
            }
            Message::ToggleSplitPng(v) => {
                self.settings.split_png = v;
                self.settings.save();
                Task::none()
            }
            Message::ToggleAlertOnData(v) => {
                self.settings.alert_on_data = v;
                self.settings.save();
                Task::none()
            }
            Message::ToggleAutoListen(v) => {
                self.settings.auto_listen = v;
                self.settings.save();
                Task::none()
            }
            Message::PortScanTick => {
                self.available_ports = serial::list_ports();
                // Auto-listen: if enabled and not currently listening and a port is available
                if self.settings.auto_listen && !self.listening {
                    if let Some(port) = &self.settings.serial_port {
                        if self.available_ports.contains(port) {
                            return self.update(Message::StartListening);
                        }
                    }
                }
                Task::none()
            }
            Message::OpenUrl(url) => {
                let _ = open::that(url);
                Task::none()
            }
            Message::ContextMenuDeleteAt(idx) => {
                if let Some(ref mut dump) = self.dump {
                    if idx < dump.captures.len() {
                        dump.captures.remove(idx);
                        if self.current >= dump.captures.len() && self.current > 0 {
                            self.current -= 1;
                        }
                        self.status = format!("Deleted capture #{}", idx + 1);
                    }
                }
                Task::none()
            }
            Message::ContextMenuExportPngAt(idx) => {
                let ts = self.dump.as_ref()
                    .and_then(|d| d.captures.get(idx))
                    .map(|e| e.timestamp);
                self.context_export_idx = Some(idx);
                Task::perform(pick_save_png(ts), Message::ContextMenuExportPngPicked)
            }
            Message::ContextMenuExportPngPicked(None) => {
                self.context_export_idx = None;
                Task::none()
            }
            Message::ContextMenuExportPngPicked(Some(path)) => {
                let idx = self.context_export_idx.take().unwrap_or(self.current);
                if let Some(cap) = self.dump.as_ref().and_then(|d| d.captures.get(idx)).map(|e| &e.capture).cloned() {
                    match export_png(&path, &cap, &self.settings) {
                        Ok(()) => self.status = format!("PNG saved to {}", path.display()),
                        Err(e) => self.status = format!("PNG error: {e}"),
                    }
                }
                Task::none()
            }
            Message::ContextMenuExportCsvAt(idx) => {
                let ts = self.dump.as_ref()
                    .and_then(|d| d.captures.get(idx))
                    .map(|e| e.timestamp);
                self.context_export_idx = Some(idx);
                Task::perform(pick_save_csv(ts), Message::ContextMenuExportCsvPicked)
            }
            Message::ContextMenuExportCsvPicked(None) => {
                self.context_export_idx = None;
                Task::none()
            }
            Message::ContextMenuExportCsvPicked(Some(path)) => {
                let idx = self.context_export_idx.take().unwrap_or(self.current);
                if let Some(cap) = self.dump.as_ref().and_then(|d| d.captures.get(idx)).map(|e| &e.capture) {
                    match export_csv(&path, cap) {
                        Ok(()) => self.status = format!("CSV saved to {}", path.display()),
                        Err(e) => self.status = format!("CSV error: {e}"),
                    }
                }
                Task::none()
            }
            Message::OpenFirmware => {
                self.show_firmware = true;
                self.firmware_port = self.settings.serial_port.clone().unwrap_or_default();
                self.firmware_file = None;
                self.firmware_progress = None;
                self.firmware_status = "Select firmware file and port, then click Update.".to_string();
                self.firmware_handle = None;
                self.firmware_rx = None;
                self.firmware_confirming = false;
                self.was_listening_before_flash = self.listening;
                Task::none()
            }
            Message::CloseFirmware => {
                if let Some(mut h) = self.firmware_handle.take() {
                    h.cancel();
                }
                self.firmware_rx = None;
                self.show_firmware = false;
                self.firmware_confirming = false;
                // Auto-reconnect if we were listening before
                if self.was_listening_before_flash && !self.listening {
                    if let Some(port) = &self.settings.serial_port {
                        if self.available_ports.contains(port) {
                            return self.update(Message::StartListening);
                        }
                    }
                }
                Task::none()
            }
            Message::FirmwarePortSelected(p) => {
                self.firmware_port = p;
                Task::none()
            }
            Message::FirmwareRefreshPorts => {
                self.available_ports = serial::list_ports();
                Task::none()
            }
            Message::FirmwareFileClicked => {
                Task::perform(pick_firmware_file(), Message::FirmwareFilePicked)
            }
            Message::FirmwareFilePicked(path) => {
                self.firmware_file = path;
                Task::none()
            }
            Message::FirmwareConfirmStart => {
                self.firmware_confirming = true;
                self.firmware_status = "⚠ Are you sure? This may brick the device. Click Update again to confirm.".to_string();
                Task::none()
            }
            Message::FirmwareStart => {
                self.firmware_confirming = false;
                let port = self.firmware_port.clone();
                let file = match &self.firmware_file {
                    Some(p) => p.to_string_lossy().to_string(),
                    None => {
                        self.firmware_status = "No firmware file selected.".to_string();
                        return Task::none();
                    }
                };
                if port.is_empty() {
                    self.firmware_status = "No port selected.".to_string();
                    return Task::none();
                }
                // Stop listening if active (can't share port)
                if self.listening {
                    if let Some(mut h) = self.serial_handle.take() {
                        h.stop();
                    }
                    self.serial_rx = None;
                    self.listening = false;
                }
                let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
                let handle = crate::flash::start_flash(port, file, tx);
                self.firmware_handle = Some(handle);
                self.firmware_rx = Some(rx);
                self.firmware_progress = Some((0, 1));
                self.firmware_status = "Starting…".to_string();
                Task::none()
            }
            Message::FirmwareEvent(_) => {
                // Drain firmware channel
                let mut finished = false;
                if let Some(rx) = &mut self.firmware_rx {
                    while let Ok(evt) = rx.try_recv() {
                        match evt {
                            crate::flash::FlashEvent::Status(s) => {
                                if !s.is_empty() {
                                    self.firmware_status = s;
                                }
                            }
                            crate::flash::FlashEvent::Progress { current, total } => {
                                self.firmware_progress = Some((current, total));
                            }
                            crate::flash::FlashEvent::Finished(result) => {
                                match result {
                                    Ok(()) => {
                                        self.firmware_status = "Flash complete! You may close this dialog.".to_string();
                                    }
                                    Err(e) => {
                                        self.firmware_status = format!("Flash error: {e}");
                                    }
                                }
                                finished = true;
                            }
                        }
                    }
                }
                if finished {
                    self.firmware_handle = None;
                    self.firmware_rx = None;
                }
                Task::none()
            }
        }
    }

    // ── View ───────────────────────────────────────────────────────────

    pub fn view(&self) -> Element<'_, Message> {
        if self.show_firmware {
            return self.firmware_dialog();
        }
        if self.show_settings {
            return self.settings_dialog();
        }

        let menu_bar = self.menu_bar();

        // Capture title with timestamp
        let title_bar: Element<'_, Message> = if let Some(entry) = self.current_entry() {
            let ts_str = entry.timestamp.format("%Y-%m-%d %H:%M:%S").to_string();
            container(text(ts_str).size(12))
                .width(Length::Fill)
                .center_x(Length::Fill)
                .padding(2)
                .into()
        } else {
            Space::new().height(Length::Fixed(0.0)).into()
        };

        let scope = self.scope_widget();
        let right = self.measurement_panel();
        let graph_row: Element<'_, Message> = container(row![scope, right].spacing(4))
            .width(Length::Fill)
            .height(Length::FillPortion(1))
            .into();
        let controls = self.controls_panel();
        let thumbs = thumbnails(&self.dump, self.current);
        let actions = self.action_buttons();

        let instructions: Element<'_, Message> = if self.settings.show_instructions {
            instructions_panel()
        } else {
            Space::new().height(Length::Fixed(0.0)).into()
        };

        let bottom = column![
            controls,
            rule::horizontal(1),
            thumbs,
            rule::horizontal(1),
            actions,
            rule::horizontal(1),
            checkbox(self.settings.show_instructions)
                .label("Show instructions")
                .on_toggle(Message::ToggleInstructions),
            instructions,
        ]
        .spacing(4)
        .padding(4);

        container(column![menu_bar, title_bar, graph_row, bottom].spacing(2))
            .padding(4)
            .into()
    }

    fn settings_dialog(&self) -> Element<'_, Message> {
        let title = text("Settings").size(22);

        // ── Serial section ──
        let serial_section = container(
            column![
                text("Serial").size(15),
                rule::horizontal(1),
                text("Default port:").size(13),
                text_input("e.g. /dev/tty.usbserial-1110", &self.settings_port_input)
                    .on_input(Message::SettingsPortChanged)
                    .size(13),
                if self.available_ports.is_empty() {
                    text("No serial ports detected.").size(11)
                } else {
                    text(format!("Available: {}", self.available_ports.join(", "))).size(11)
                },
                text("Baud rate:").size(13),
                text_input("115200", &self.settings_baud_input)
                    .on_input(Message::SettingsBaudChanged)
                    .size(13),
                checkbox(self.settings.auto_listen)
                    .label("Auto-listen on startup")
                    .on_toggle(Message::ToggleAutoListen),
                checkbox(self.settings.alert_on_data)
                    .label("Beep on new capture")
                    .on_toggle(Message::ToggleAlertOnData),
            ]
            .spacing(4),
        )
        .padding(16)
        .style(container::bordered_box);

        // ── Display section ──
        let display_section = container(
            column![
                text("Display").size(15),
                rule::horizontal(1),
                checkbox(self.settings.show_instructions)
                    .label("Show instructions panel")
                    .on_toggle(Message::ToggleInstructions),
            ]
            .spacing(4),
        )
        .padding(16)
        .style(container::bordered_box);

        // ── About section ──
        let ver = env!("CARGO_PKG_VERSION");
        let repo = env!("CARGO_PKG_REPOSITORY");
        let config_path = crate::settings::config_dir_path()
            .map(|p| p.display().to_string())
            .unwrap_or_else(|| "unknown".to_string());
        let about_section = container(
            column![
                text("About").size(15),
                rule::horizontal(1),
                text(format!("DSO3D12 GUI v{ver}")).size(13),
                button(text(repo).size(11))
                    .on_press(Message::OpenUrl(repo.to_string()))
                    .style(button::text),
                text(format!("Config: {config_path}")).size(11),
                Space::new().height(Length::Fixed(8.0)),
                button(text("Firmware Update…").size(13)).on_press(Message::OpenFirmware),
            ]
            .spacing(4),
        )
        .padding(16)
        .style(container::bordered_box);

        let close_btn = button(text("Save & Close").size(13)).on_press(Message::CloseSettings);

        container(
            column![
                title,
                Space::new().height(Length::Fixed(8.0)),
                serial_section,
                Space::new().height(Length::Fixed(8.0)),
                display_section,
                Space::new().height(Length::Fixed(8.0)),
                about_section,
                Space::new().height(Length::Fixed(16.0)),
                close_btn,
            ]
            .spacing(4)
            .padding(20)
            .max_width(500),
        )
        .center_x(Length::Fill)
        .center_y(Length::Fill)
        .into()
    }

    fn firmware_dialog(&self) -> Element<'_, Message> {
        let title = text("Firmware Update").size(22);
        let warning = container(
            text("⚠ WARNING: Flashing incorrect firmware may brick your device. Ensure the device is in boot mode and the correct firmware file is selected.")
                .size(12),
        )
        .padding(8)
        .style(container::bordered_box);

        let boot_instructions = container(
            column![
                text("Switching to Boot Mode (DSO3D12)").size(13),
                Space::new().height(Length::Fixed(4.0)),
                text("1. Connect the supplied USB cable to your PC while the oscilloscope is powered off.").size(11),
                text("2. Press and hold the power button. The scope may enter a power-cycling loop — keep the button held down continuously until flashing reaches 100%.").size(11),
            ]
            .spacing(2),
        )
        .padding(8)
        .style(container::bordered_box);

        let port_picker = pick_list(
            self.available_ports.clone(),
            if self.firmware_port.is_empty() {
                None
            } else {
                Some(self.firmware_port.clone())
            },
            Message::FirmwarePortSelected,
        )
        .placeholder("Select port…")
        .text_size(13);
        let refresh_btn = button(text("Refresh").size(12)).on_press(Message::FirmwareRefreshPorts);

        let file_label = match &self.firmware_file {
            Some(p) => text(p.file_name().and_then(|n| n.to_str()).unwrap_or("?")).size(13),
            None => text("No file selected").size(13),
        };
        let file_btn = button(text("Browse…").size(13)).on_press(Message::FirmwareFileClicked);

        let progress_bar_el: Element<'_, Message> = if let Some((cur, total)) = self.firmware_progress
        {
            let pct = if total > 0 {
                (cur as f32 / total as f32) * 100.0
            } else {
                0.0
            };
            column![
                text(format!("{pct:.0}% ({cur}/{total} blocks)")).size(12),
                progress_bar(0.0..=total as f32, cur as f32),
            ]
            .spacing(2)
            .into()
        } else {
            Space::new().height(Length::Fixed(0.0)).into()
        };

        let status_text = text(self.firmware_status.clone()).size(12);

        let is_flashing = self.firmware_handle.is_some();
        let update_btn = if is_flashing {
            button(text("Updating…").size(13))
        } else if self.firmware_confirming {
            button(text("Confirm Update").size(13)).on_press(Message::FirmwareStart)
        } else {
            button(text("Update").size(13)).on_press(Message::FirmwareConfirmStart)
        };
        let close_btn = button(text("Close").size(13)).on_press(Message::CloseFirmware);

        container(
            column![
                title,
                Space::new().height(Length::Fixed(8.0)),
                warning,
                Space::new().height(Length::Fixed(8.0)),
                boot_instructions,
                Space::new().height(Length::Fixed(8.0)),
                text("Port:").size(13),
                row![port_picker, refresh_btn].spacing(8).align_y(iced::Alignment::Center),
                Space::new().height(Length::Fixed(8.0)),
                text("Firmware file (.fls):").size(13),
                row![file_btn, file_label].spacing(8).align_y(iced::Alignment::Center),
                Space::new().height(Length::Fixed(8.0)),
                progress_bar_el,
                status_text,
                Space::new().height(Length::Fixed(16.0)),
                row![update_btn, close_btn].spacing(8),
            ]
            .spacing(4)
            .padding(20)
            .max_width(500),
        )
        .center_x(Length::Fill)
        .center_y(Length::Fill)
        .into()
    }

    fn menu_bar(&self) -> Element<'_, Message> {
        let port_display = match &self.settings.serial_port {
            Some(p) => text(p.clone()).size(12),
            None => text("No port").size(12),
        };

        let listen_btn = if self.listening {
            button(text("Stop").size(12)).on_press(Message::StopListening)
        } else {
            button(text("Listen").size(12)).on_press(Message::StartListening)
        };

        let port_picker = pick_list(
            self.available_ports.clone(),
            self.settings.serial_port.clone(),
            Message::PortSelected,
        )
        .placeholder("Port…")
        .text_size(12);

        container(
            row![
                text("DSO3D12").size(14),
                Space::new().width(Length::Fixed(12.0)),
                port_picker,
                port_display,
                listen_btn,
                Space::new().width(Length::Fill),
                text(self.status.clone()).size(12),
            ]
            .spacing(6)
            .align_y(iced::Alignment::Center),
        )
        .padding(4)
        .width(Length::Fill)
        .style(container::rounded_box)
        .into()
    }

    fn scope_widget(&self) -> Element<'_, Message> {
        let cap = self.current_capture();
        log::trace!(
            "scope_widget: capture present={}, ch1 len={}",
            cap.is_some(),
            cap.map(|c| c.ch1.len()).unwrap_or(0)
        );
        let n_samples = cap.map(|c| c.ch1.len()).unwrap_or(DEFAULT_BUFFER_DEPTH);
        let has_ch2_data = cap.and_then(|c| c.ch2.as_ref()).is_some();
        let scope = Scope {
            capture: cap,
            show_ch1: self.settings.show_ch1,
            show_ch2: self.settings.show_ch2,
            view_mode: self.settings.view_mode,
            show_scales: self.settings.show_scales,
            n_samples,
            has_ch1: self.settings.show_ch1,
            has_ch2: self.settings.show_ch2 && has_ch2_data,
            meas_cursor_x: if self.settings.measurement_cursor_x_enabled {
                Some(self.settings.cursor_x_range)
            } else {
                None
            },
            meas_cursor_y: if self.settings.measurement_cursor_y_enabled {
                Some(self.settings.cursor_y_range)
            } else {
                None
            },
            device_cursor_x: if self.settings.device_cursor_x_enabled {
                self.current_entry()
                    .and_then(|e| e.record.scope_state.as_ref())
                    .filter(|s| s.cursors_x_enable)
                    .map(|s| (s.cursor_x1_frac as f32, s.cursor_x2_frac as f32))
            } else {
                None
            },
            device_cursor_y: if self.settings.device_cursor_y_enabled {
                self.current_entry()
                    .and_then(|e| e.record.scope_state.as_ref())
                    .filter(|s| s.cursors_y_enable)
                    .map(|s| (s.cursor_y1_frac as f32, s.cursor_y2_frac as f32))
            } else {
                None
            },
            v_per_div: self.settings.v_per_div,
            t_per_div_ms: self.settings.t_per_div_ms,
            v_offset: self.settings.v_offset,
            on_click: Some(Message::GraphClicked),
        };
        canvas(scope)
            .width(Length::Fill)
            .height(Length::FillPortion(1))
            .into()
    }

    /// Right-hand measurement panel — signal info + two range sliders with
    /// computed voltage/time/frequency measurements.
    fn measurement_panel(&self) -> Element<'_, Message> {
        let cap = self.current_capture();

        let (xlo, xhi) = self.settings.cursor_x_range;
        let (ylo, yhi) = self.settings.cursor_y_range;

        // Scale parameters
        let v_per_div = self.settings.v_per_div;
        let t_per_div_ms = self.settings.t_per_div_ms;
        let v_offset = self.settings.v_offset;
        let half_v_range = 4.0 * v_per_div; // 8 div / 2
        let total_time_ms = 12.0 * t_per_div_ms;

        // ── Signal information ──
        let mut items: Vec<Element<'_, Message>> = vec![
            text("Measurement").size(13).into(),
            rule::horizontal(1).into(),
        ];

        // ── Scale inputs ──
        items.push(
            row![
                text("V/div:").size(11),
                text_input("1.0", &self.v_per_div_input)
                    .on_input(Message::VPerCellChanged)
                    .size(11)
                    .width(Length::Fixed(50.0)),
                text("V").size(11),
            ]
            .spacing(4)
            .align_y(iced::Alignment::Center)
            .into(),
        );
        items.push(
            row![
                text("t/div:").size(11),
                text_input("1.0", &self.t_per_div_input)
                    .on_input(Message::TPerCellChanged)
                    .size(11)
                    .width(Length::Fixed(50.0)),
                text("ms").size(11),
            ]
            .spacing(4)
            .align_y(iced::Alignment::Center)
            .into(),
        );
        items.push(
            row![
                text("V/off:").size(11),
                text_input("0.0", &self.v_offset_input)
                    .on_input(Message::VOffsetChanged)
                    .size(11)
                    .width(Length::Fixed(50.0)),
                text("V").size(11),
            ]
            .spacing(4)
            .align_y(iced::Alignment::Center)
            .into(),
        );
        items.push(
            text("AC mode: 8×V, 12×t div").size(9).into(),
        );
        items.push(rule::horizontal(1).into());

        if let Some(cap) = cap {
            let v_min = frac_to_voltage(1.0, half_v_range, v_offset); // bottom
            let v_max = frac_to_voltage(0.0, half_v_range, v_offset); // top
            items.push(
                text(format!("CH1 range: {v_min:.2}V … {v_max:.2}V")).size(11).into(),
            );
            items.push(
                text(format!("Time range: {total_time_ms:.2} ms ({} samp)", cap.ch1.len()))
                    .size(11)
                    .into(),
            );
            if cap.ch2.is_some() {
                items.push(
                    text(format!("CH2 range: {v_min:.2}V … {v_max:.2}V"))
                        .size(11)
                        .into(),
                );
            }
            items.push(rule::horizontal(1).into());
        }

        // ── X cursor (time) ──
        items.push(
            checkbox(self.settings.measurement_cursor_x_enabled)
                .label("Cursor X (time)")
                .on_toggle(Message::ToggleMeasCursorX)
                .into(),
        );

        if self.settings.measurement_cursor_x_enabled {
            items.push(range_slider::horizontal(xlo, xhi, Message::MeasCursorXChanged));
            let t_lo = xlo as f64 * total_time_ms;
            let t_hi = xhi as f64 * total_time_ms;
            let dt_ms = t_hi - t_lo;
            let freq = if dt_ms > 0.0 {
                format!("{:.1} Hz", 1000.0 / dt_ms)
            } else {
                "—".to_string()
            };
            items.push(
                column![
                    text(format!("Δt = {dt_ms:.3} ms")).size(11),
                    text(format!("≈ freq: {freq}")).size(11),
                ]
                .spacing(2)
                .into(),
            );
        }

        items.push(Space::new().height(Length::Fixed(6.0)).into());

        // ── Y cursor (voltage) ──
        items.push(
            checkbox(self.settings.measurement_cursor_y_enabled)
                .label("Cursor Y (voltage)")
                .on_toggle(Message::ToggleMeasCursorY)
                .into(),
        );

        if self.settings.measurement_cursor_y_enabled {
            items.push(range_slider::horizontal(ylo, yhi, Message::MeasCursorYChanged));
            let v_lo = frac_to_voltage(ylo as f64, half_v_range, v_offset);
            let v_hi = frac_to_voltage(yhi as f64, half_v_range, v_offset);
            let dv = (v_hi - v_lo).abs();
            items.push(
                column![
                    text(format!("ΔV = {dv:.3} V")).size(11),
                    text(format!("({v_lo:.2}V … {v_hi:.2}V)")).size(11),
                ]
                .spacing(2)
                .into(),
            );
        }

        items.push(Space::new().height(Length::Fixed(6.0)).into());
        items.push(rule::horizontal(1).into());

        // ── Per-channel stats ──
        if let Some(cap) = cap {
            let (mn, mx, av) = stats(&cap.ch1);
            items.push(
                text(format!(
                    "CH1: n={} min={mn} max={mx} avg={av:.1}",
                    cap.ch1.len()
                ))
                .size(10)
                .into(),
            );
            if let Some(ch2) = &cap.ch2 {
                let (mn, mx, av) = stats(ch2);
                items.push(
                    text(format!("CH2: n={} min={mn} max={mx} avg={av:.1}", ch2.len()))
                        .size(10)
                        .into(),
                );
            }
        }

        // ── Trigger + measurements from screenshot metadata ──
        if let Some(scope_state) = self.current_entry().and_then(|e| e.record.scope_state.as_ref()) {
            items.push(rule::horizontal(1).into());
            items.push(text("Trigger").size(11).into());
            items.push(
                text(format!(
                    "Mode: {} | Edge: {} | Src: {}",
                    scope_state.trigger_mode, scope_state.trigger_edge, scope_state.trigger_source
                ))
                .size(10)
                .into(),
            );
            items.push(
                text(format!("Level: {:.3} V", scope_state.trigger_level_v))
                    .size(10)
                    .into(),
            );
            items.push(
                text(format!(
                    "Delay: {:.6} s | Rate: {:.0} Sa/s",
                    scope_state.trigger_delay_s, scope_state.sample_rate_hz
                ))
                .size(10)
                .into(),
            );

            // ── Auto-measurements table ──
            let has_m1 = scope_state.ch1.as_ref().and_then(|c| c.measurements.as_ref());
            let has_m2 = scope_state.ch2.as_ref().and_then(|c| c.measurements.as_ref());
            if has_m1.is_some() || has_m2.is_some() {
                items.push(rule::horizontal(1).into());
                items.push(text("Auto Measurements").size(11).into());

                let fmt_v = |v: f64| -> String {
                    if v.abs() >= 1.0 { format!("{v:.3} V") }
                    else { format!("{:.2} mV", v * 1000.0) }
                };
                let fmt_t = |s: f64| -> String {
                    if s >= 1.0 { format!("{s:.3} s") }
                    else if s >= 0.001 { format!("{:.3} ms", s * 1000.0) }
                    else { format!("{:.2} µs", s * 1_000_000.0) }
                };
                let fmt_hz = |hz: f64| -> String {
                    if hz >= 1_000_000.0 { format!("{:.3} MHz", hz / 1_000_000.0) }
                    else if hz >= 1000.0 { format!("{:.3} kHz", hz / 1000.0) }
                    else { format!("{hz:.2} Hz") }
                };

                // Build measurement rows: label | CH1 | CH2
                let labels = [
                    "Freq", "PkPk", "Avg", "RMS", "Amp", "+Duty", "-Duty",
                    "+T", "-T", "T", "Max", "Min", "Top", "Base",
                ];
                let ch_val = |m: Option<&dso_parser::ChannelMeasurements>, idx: usize| -> String {
                    match m {
                        None => "—".to_string(),
                        Some(m) => {
                            let fv = |v: f64| if v.is_nan() { "—".to_string() } else { fmt_v(v) };
                            let ft = |v: f64| if v.is_nan() { "—".to_string() } else { fmt_t(v) };
                            let fd = |v: f64| if v.is_nan() { "—".to_string() } else { format!("{:.1}%", v) };
                            let fh = |v: f64| if v.is_nan() { "—".to_string() } else { fmt_hz(v) };
                            match idx {
                                0 => fh(m.freq_hz),
                                1 => fv(m.pk_pk_v),
                                2 => fv(m.avg_v),
                                3 => fv(m.rms_v),
                                4 => fv(m.amplitude_v),
                                5 => fd(m.pos_duty_pct),
                                6 => fd(m.neg_duty_pct),
                                7 => ft(m.pos_width_s),
                                8 => ft(m.neg_width_s),
                                9 => ft(m.period_s),
                                10 => fv(m.max_v),
                                11 => fv(m.min_v),
                                12 => fv(m.top_v),
                                13 => fv(m.base_v),
                                _ => "—".to_string(),
                            }
                        }
                    }
                };

                // Header row
                items.push(
                    row![
                        text("").size(9).width(Length::Fixed(36.0)),
                        text("CH1").size(9).width(Length::Fixed(72.0)),
                        text("CH2").size(9).width(Length::Fixed(72.0)),
                    ]
                    .spacing(2)
                    .into(),
                );

                for (idx, lbl) in labels.iter().enumerate() {
                    let v1 = ch_val(has_m1, idx);
                    let v2 = ch_val(has_m2, idx);
                    items.push(
                        row![
                            text(*lbl).size(9).width(Length::Fixed(36.0)),
                            text(v1).size(9).width(Length::Fixed(72.0)),
                            text(v2).size(9).width(Length::Fixed(72.0)),
                        ]
                        .spacing(2)
                        .into(),
                    );
                }
            }
        }

        container(column(items).spacing(4).padding(6))
            .width(Length::Fixed(220.0))
            .style(container::rounded_box)
            .into()
    }

    fn controls_panel(&self) -> Element<'_, Message> {
        let has_dev_x = self.capture_has_device_cursor_x();
        let has_dev_y = self.capture_has_device_cursor_y();

        let dev_x = {
            let cb = checkbox(self.settings.device_cursor_x_enabled).label("Device X");
            if has_dev_x {
                cb.on_toggle(Message::ToggleDeviceCursorX)
            } else {
                cb
            }
        };
        let dev_y = {
            let cb = checkbox(self.settings.device_cursor_y_enabled).label("Device Y");
            if has_dev_y {
                cb.on_toggle(Message::ToggleDeviceCursorY)
            } else {
                cb
            }
        };

        row![
            checkbox(self.settings.show_ch1)
                .label("CH1")
                .on_toggle(Message::ToggleCh1),
            checkbox(self.settings.show_ch2)
                .label("CH2")
                .on_toggle(Message::ToggleCh2),
            Space::new().width(Length::Fixed(16.0)),
            dev_x,
            dev_y,
            Space::new().width(Length::Fixed(16.0)),
            checkbox(self.settings.show_scales)
                .label("Scales")
                .on_toggle(Message::ToggleShowScales),
            Space::new().width(Length::Fixed(16.0)),
            text("View:").size(13),
            pick_list(
                ViewMode::ALL.as_slice(),
                Some(self.settings.view_mode),
                Message::ViewModeSelected,
            )
            .text_size(13),
        ]
        .spacing(8)
        .padding(4)
        .align_y(iced::Alignment::Center)
        .into()
    }

    fn action_buttons(&self) -> Element<'_, Message> {
        row![
            button(text("Load Capture").size(12)).on_press(Message::LoadClicked),
            button(text("Save Capture").size(12)).on_press(Message::SaveCaptureClicked),
            Space::new().width(Length::Fixed(12.0)),
            text("│").size(12),
            Space::new().width(Length::Fixed(12.0)),
            button(text("Save PNG").size(12)).on_press(Message::SavePngClicked),
            checkbox(self.settings.split_png)
                .label("Split CH1/CH2")
                .on_toggle(Message::ToggleSplitPng),
            Space::new().width(Length::Fixed(12.0)),
            text("│").size(12),
            Space::new().width(Length::Fixed(12.0)),
            button(text("Export Data").size(12)).on_press(Message::ExportCsvClicked),
            Space::new().width(Length::Fill),
            button(text("Settings").size(12)).on_press(Message::OpenSettings),
        ]
        .spacing(6)
        .padding(4)
        .align_y(iced::Alignment::Center)
        .into()
    }
}

// ── Helpers ────────────────────────────────────────────────────────────────

fn instructions_panel<'a>() -> Element<'a, Message> {
    let standard_fw = "Standard firmware (raw debug dump):\n\n\
        1. Feed a signal into the scope.\n\
        2. Press Stop to freeze the sample buffer.\n\
        3. Press Menu → Stop so Channel 1\n   Measurements menu is on screen.\n\
        4. Long-press the Save button to start\n   the debug data transmission.\n\
        5. Wait until the transfer completes\n   (~300 kB at 115200 baud).";

    let screenshot_fw = "Customized firmware:\n\
        1. Capture a waveform on the device\n   (press Save while running or stopped).\n\
        2. Press Menu → Gallery to open\n   the saved screenshots list.\n\
        3. Select the waveform to transmit.\n\
        4. The device sends a 2048-byte binary\n   packet automatically over USB-serial.\n\
        5. Includes full metadata: V/div, timebase,\n   trigger, cursors, measurements.";

    let left = container(text(standard_fw).size(11))
        .padding(6)
        .width(Length::FillPortion(1));
    let right = container(
        column![
            button(text("ZeeTweak (GitHub)").size(11))
                .on_press(Message::OpenUrl("https://github.com/taligentx/ZeeTweak".to_string()))
                .style(button::text),
            text(screenshot_fw).size(11),
        ]
        .spacing(2),
    )
        .padding(6)
        .width(Length::FillPortion(1));

    container(
        row![left, rule::vertical(1), right]
            .spacing(4)
    )
    .padding(4)
    .style(container::rounded_box)
    .into()
}

pub(crate) fn stats(samples: &[u8]) -> (u8, u8, f32) {
    if samples.is_empty() {
        return (0, 0, 0.0);
    }
    let mut mn = u8::MAX;
    let mut mx = 0u8;
    let mut sum: u64 = 0;
    for &b in samples {
        mn = mn.min(b);
        mx = mx.max(b);
        sum += b as u64;
    }
    (mn, mx, sum as f32 / samples.len() as f32)
}

/// Convert raw ADC count (0-255) to approximate voltage using the default ±5V range.
/// Used in tests.
#[cfg(test)]
fn adc_to_voltage(adc: u8) -> f32 {
    (adc as f32 - 128.0) * 5.0 / 128.0
}

/// Convert a fractional position (0.0..1.0) on the Y axis to voltage
/// using the user-configured V/div scale.
/// frac=0 → top → +half_range; frac=1 → bottom → -half_range.
/// Format a float for display in a scale input field.
/// Always shows a decimal point so the field looks like a float.
/// Strips trailing zeros but keeps at least one decimal digit (e.g. "1.0", "0.5", "0.001").
fn format_float(v: f64) -> String {
    // Use enough precision to round-trip small values
    let s = format!("{:.9}", v);
    let trimmed = s.trim_end_matches('0');
    if trimmed.ends_with('.') {
        format!("{}0", trimmed)
    } else {
        trimmed.to_string()
    }
}

fn frac_to_voltage(frac: f64, half_v_range: f64, v_offset: f64) -> f64 {
    v_offset + half_v_range * (1.0 - 2.0 * frac)
}

/// Convert sample count to time in milliseconds at the default 1 MHz sample rate.
/// Used in tests.
#[cfg(test)]
const SAMPLE_RATE_HZ: f64 = 1_000_000.0;

#[cfg(test)]
fn samples_to_time_ms(n: usize) -> f64 {
    (n as f64 / SAMPLE_RATE_HZ) * 1000.0
}

/// Thumbnails strip: scrollable row of mini waveform canvases.
/// Each thumbnail shows the signal shape and is clickable.
fn thumbnails<'a>(dump: &'a Option<LoadedDump>, current: usize) -> Element<'a, Message> {
    let Some(dump) = dump else {
        return Space::new().height(Length::Fixed(0.0)).into();
    };
    if dump.captures.is_empty() {
        return Space::new().height(Length::Fixed(0.0)).into();
    }
    let row_items = dump.captures.iter().enumerate().map(|(i, entry)| {
        let thumb = Thumbnail {
            capture: &entry.capture,
            selected: i == current,
        };
        let thumb_canvas: Element<'a, Message> = canvas(thumb)
            .width(Length::Fixed(100.0))
            .height(Length::Fixed(50.0))
            .into();

        let btn: Element<'a, Message> = button(thumb_canvas)
            .on_press(Message::SelectCapture(i))
            .padding(2)
            .style(if i == current {
                button::primary
            } else {
                button::secondary
            })
            .into();

        // Wrap in ContextMenu for right-click popup
        let tooltip_ts = entry.timestamp.format("%Y-%m-%d %H:%M:%S").to_string();
        let underlay: Element<'a, Message> = iced::widget::tooltip(
            btn,
            text(tooltip_ts).size(11),
            iced::widget::tooltip::Position::Top,
        )
        .into();

        let ctx_idx = i;
        iced_aw::ContextMenu::new(underlay, move || {
            iced::widget::column![
                button(text("Export PNG…").size(12))
                    .on_press(Message::ContextMenuExportPngAt(ctx_idx))
                    .width(Length::Fill),
                button(text("Export CSV…").size(12))
                    .on_press(Message::ContextMenuExportCsvAt(ctx_idx))
                    .width(Length::Fill),
                button(text("Delete").size(12))
                    .on_press(Message::ContextMenuDeleteAt(ctx_idx))
                    .width(Length::Fill),
            ]
            .spacing(2)
            .padding(4)
            .width(Length::Fixed(140.0))
            .into()
        })
        .into()
    });
    container(
        scrollable(row(row_items).spacing(4))
            .direction(scrollable::Direction::Horizontal(
                scrollable::Scrollbar::default(),
            )),
    )
    .height(Length::Fixed(62.0))
    .into()
}

// ── Async file pickers ─────────────────────────────────────────────────────

async fn pick_file() -> Option<PathBuf> {
    rfd::AsyncFileDialog::new()
        .add_filter("ZeeWeii Capture", &["zwcap", "bin", "dat", "raw"])
        .add_filter("All files", &["*"])
        .pick_file()
        .await
        .map(|h| h.path().to_path_buf())
}

async fn pick_save_bin() -> Option<PathBuf> {
    let now = chrono::Local::now();
    let name = format!("capture_{}.zwcap", now.format("%Y%m%d_%H%M%S"));
    rfd::AsyncFileDialog::new()
        .add_filter("ZeeWeii Capture", &["zwcap"])
        .add_filter("All files", &["*"])
        .set_file_name(&name)
        .save_file()
        .await
        .map(|h| h.path().to_path_buf())
}

async fn pick_save_csv(ts: Option<DateTime<Local>>) -> Option<PathBuf> {
    let t = ts.unwrap_or_else(Local::now);
    let name = format!("capture_{}.csv", t.format("%Y%m%d_%H%M%S"));
    rfd::AsyncFileDialog::new()
        .add_filter("CSV", &["csv"])
        .set_file_name(&name)
        .save_file()
        .await
        .map(|h| h.path().to_path_buf())
}

async fn pick_save_png(ts: Option<DateTime<Local>>) -> Option<PathBuf> {
    let t = ts.unwrap_or_else(Local::now);
    let name = format!("capture_{}.png", t.format("%Y%m%d_%H%M%S"));
    rfd::AsyncFileDialog::new()
        .add_filter("PNG Image", &["png"])
        .set_file_name(&name)
        .save_file()
        .await
        .map(|h| h.path().to_path_buf())
}

async fn pick_firmware_file() -> Option<PathBuf> {
    rfd::AsyncFileDialog::new()
        .add_filter("Firmware", &["fls", "bin"])
        .add_filter("All files", &["*"])
        .pick_file()
        .await
        .map(|h| h.path().to_path_buf())
}

async fn load_dump(path: PathBuf) -> Result<LoadedDump, anyhow::Error> {
    log::info!("loading {}", path.display());
    let bytes = std::fs::read(&path)?;

    // Detect format: .zwcap container, binary screenshot, or ASCII debug dump
    let entries = if dso_parser::is_zwcap(&bytes) {
        // New container format
        let records = dso_parser::deserialize_zwcap(&bytes)
            .map_err(|e| anyhow::anyhow!("{e}"))?;
        records.into_iter().map(|rec| {
            let capture = dso_parser::record_to_capture(&rec);
            let timestamp = rec.captured_at.as_ref()
                .and_then(|s| chrono::DateTime::parse_from_str(s, "%Y-%m-%dT%H:%M:%S%:z").ok())
                .map(|dt| dt.with_timezone(&Local))
                .unwrap_or_else(Local::now);
            CaptureEntry { capture, timestamp, record: rec }
        }).collect()
    } else if dso_parser::is_screenshot_packet(&bytes) {
        // Single binary screenshot packet
        let pkt = dso_parser::parse_screenshot(&bytes)?;
        let now = Local::now();
        let record = dso_parser::screenshot_to_record(&pkt, now, &bytes);
        let capture = pkt.to_capture();
        vec![CaptureEntry { capture, timestamp: now, record }]
    } else {
        // Try ASCII debug dump (legacy format)
        let parser_entries = dso_parser::load_captures_with_metadata(&bytes)?;
        parser_entries.into_iter().map(|e| {
            let record = dso_parser::capture_to_record(&e.capture, e.timestamp, None);
            CaptureEntry { capture: e.capture, timestamp: e.timestamp, record }
        }).collect()
    };

    Ok(LoadedDump { path, captures: entries })
}

// ── Beep helpers ───────────────────────────────────────────────────────────

/// Play a short "start receiving" beep sound.
fn play_beep_start() {
    #[cfg(target_os = "macos")]
    {
        std::thread::spawn(|| {
            let _ = std::process::Command::new("afplay")
                .arg("/System/Library/Sounds/Tink.aiff")
                .output();
        });
    }
    #[cfg(not(target_os = "macos"))]
    {
        print!("\x07");
    }
}

/// Play a "capture complete" beep sound (different from start).
fn play_beep_end() {
    #[cfg(target_os = "macos")]
    {
        std::thread::spawn(|| {
            let _ = std::process::Command::new("afplay")
                .arg("/System/Library/Sounds/Glass.aiff")
                .output();
        });
    }
    #[cfg(not(target_os = "macos"))]
    {
        print!("\x07");
    }
}

// ── File save/load helpers ─────────────────────────────────────────────────

/// Save all captures to a `.zwcap` container file.
fn save_all_captures(path: &std::path::Path, entries: &[CaptureEntry]) -> anyhow::Result<()> {
    let records: Vec<dso_parser::CaptureRecord> = entries
        .iter()
        .map(|e| e.record.clone())
        .collect();
    let data = dso_parser::serialize_zwcap(&records);
    std::fs::write(path, data)?;
    Ok(())
}

fn export_csv(path: &std::path::Path, cap: &Capture) -> anyhow::Result<()> {
    let mut wtr = csv::Writer::from_path(path)?;
    if cap.ch2.is_some() {
        wtr.write_record(["index", "ch1", "ch2"])?;
    } else {
        wtr.write_record(["index", "ch1"])?;
    }
    let n = cap.sample_count();
    for i in 0..n {
        let v1 = cap.ch1.get(i).map(|b| b.to_string()).unwrap_or_default();
        if let Some(ch2) = &cap.ch2 {
            let v2 = ch2.get(i).map(|b| b.to_string()).unwrap_or_default();
            wtr.write_record([i.to_string(), v1, v2])?;
        } else {
            wtr.write_record([i.to_string(), v1])?;
        }
    }
    wtr.flush()?;
    Ok(())
}

/// Export the current capture as a PNG image.
fn export_png(path: &std::path::Path, cap: &Capture, settings: &Settings) -> anyhow::Result<()> {
    let graph_w: u32 = 1200;
    let graph_h: u32 = 600;
    let scale_margin_left: u32 = if settings.show_scales { 50 } else { 0 };
    let scale_margin_bottom: u32 = if settings.show_scales { 16 } else { 0 };

    let has_ch2 = cap.ch2.is_some();
    let do_split = settings.split_png && settings.show_ch1 && settings.show_ch2 && has_ch2;

    let width = graph_w + scale_margin_left;
    let height = if do_split {
        (graph_h + scale_margin_bottom) * 2
    } else {
        graph_h + scale_margin_bottom
    };

    let mut pixels = vec![0u8; (width * height * 3) as usize];
    let bg = [10u8, 15, 26];
    for chunk in pixels.chunks_exact_mut(3) {
        chunk.copy_from_slice(&bg);
    }

    // Helper to draw a grid into a sub-region of the pixel buffer.
    let draw_grid = |pixels: &mut Vec<u8>, x_off: u32, y_off: u32, w: u32, h: u32| {
        let grid_color = [51u8, 61, 77];
        for col in 1..10u32 {
            let x = x_off + col * w / 10;
            for y in y_off..y_off + h {
                let idx = ((y * width + x) * 3) as usize;
                pixels[idx..idx + 3].copy_from_slice(&grid_color);
            }
        }
        for r in 1..8u32 {
            let y = y_off + r * h / 8;
            for x in x_off..x_off + w {
                let idx = ((y * width + x) * 3) as usize;
                pixels[idx..idx + 3].copy_from_slice(&grid_color);
            }
        }
    };

    // Helper to draw a trace into a sub-region.
    let draw_ch =
        |pixels: &mut Vec<u8>, samples: &[u8], color: [u8; 3], x_off: u32, y_off: u32, w: u32, h: u32| {
            let n = samples.len();
            if n == 0 {
                return;
            }
            let sample_y = |px: u32| -> u32 {
                let i = (px as f64 / w as f64 * (n - 1) as f64) as usize;
                let v = samples[i.min(n - 1)];
                // Invert Y: higher ADC value (higher voltage) at top of image.
                let py = ((1.0 - v as f64 / 255.0) * (h - 1) as f64) as u32;
                py.min(h - 1)
            };

            let mut prev_y = sample_y(0);
            let idx = (((y_off + prev_y) * width + x_off) * 3) as usize;
            if idx + 2 < pixels.len() {
                pixels[idx..idx + 3].copy_from_slice(&color);
            }

            for px in 1..w {
                let cur_y = sample_y(px);
                let y_start = prev_y.min(cur_y);
                let y_end = prev_y.max(cur_y);
                for y in y_start..=y_end {
                    let idx = (((y_off + y) * width + x_off + px) * 3) as usize;
                    if idx + 2 < pixels.len() {
                        pixels[idx..idx + 3].copy_from_slice(&color);
                    }
                }
                if cur_y > 0 {
                    let idx = (((y_off + cur_y - 1) * width + x_off + px) * 3) as usize;
                    if idx + 2 < pixels.len() {
                        pixels[idx..idx + 3].copy_from_slice(&color);
                    }
                }
                if cur_y + 1 < h {
                    let idx = (((y_off + cur_y + 1) * width + x_off + px) * 3) as usize;
                    if idx + 2 < pixels.len() {
                        pixels[idx..idx + 3].copy_from_slice(&color);
                    }
                }
                prev_y = cur_y;
            }
        };

    // Helper to draw scale labels (simple 3x5 digit font).
    let v_per_div = settings.v_per_div;
    let t_per_div_ms = settings.t_per_div_ms;
    let v_offset = settings.v_offset;
    let draw_scales_on_region =
        |pixels: &mut Vec<u8>, x_off: u32, y_off: u32, w: u32, h: u32, _n_samples: usize, ch1_color: Option<[u8; 3]>, ch2_color: Option<[u8; 3]>| {
            // X axis: time labels at bottom
            let cols = 10u32;
            let total_time_ms = 12.0 * t_per_div_ms;
            for i in 0..=cols {
                let frac = i as f64 / cols as f64;
                let time_ms = frac * total_time_ms;
                let label = if time_ms >= 1.0 {
                    format!("{time_ms:.1}")
                } else {
                    format!("{:.0}u", time_ms * 1000.0)
                };
                let x_pos = x_off + i * w / cols + 2;
                let y_pos = y_off + h + 2;
                draw_text_tiny(pixels, width, &label, x_pos, y_pos, [153, 166, 179]);
            }

            // Y axis: voltage labels on left margin, per-channel
            let rows = 8u32;
            let half_v_range = 4.0 * v_per_div;
            let x_ch1 = 2u32;
            let x_ch2 = if ch1_color.is_some() { 26u32 } else { 2u32 };
            for i in 0..=rows {
                let frac = i as f64 / rows as f64;
                let voltage = v_offset + half_v_range * (1.0 - 2.0 * frac);
                let label = format!("{voltage:.1}");
                let y_pos = y_off + (i * h / rows);
                if let Some(c) = ch1_color {
                    draw_text_tiny(pixels, width, &label, x_ch1, y_pos, c);
                }
                if let Some(c) = ch2_color {
                    draw_text_tiny(pixels, width, &label, x_ch2, y_pos, c);
                }
            }
        };

    if do_split {
        let sub_h = graph_h;
        // Upper subimage: CH1
        let y_off_1 = 0u32;
        draw_grid(&mut pixels, scale_margin_left, y_off_1, graph_w, sub_h);
        draw_ch(&mut pixels, &cap.ch1, [255, 217, 25], scale_margin_left, y_off_1, graph_w, sub_h);
        if settings.show_scales {
            let ch1_col = Some([255u8, 217, 25]);
            draw_scales_on_region(&mut pixels, scale_margin_left, y_off_1, graph_w, sub_h, cap.ch1.len(), ch1_col, None);
        }

        // Lower subimage: CH2
        let y_off_2 = graph_h + scale_margin_bottom;
        draw_grid(&mut pixels, scale_margin_left, y_off_2, graph_w, sub_h);
        if let Some(ch2) = &cap.ch2 {
            draw_ch(&mut pixels, ch2, [38, 217, 255], scale_margin_left, y_off_2, graph_w, sub_h);
        }
        if settings.show_scales {
            let ch2_col = Some([38u8, 217, 255]);
            let n = cap.ch2.as_ref().map(|c| c.len()).unwrap_or(cap.ch1.len());
            draw_scales_on_region(&mut pixels, scale_margin_left, y_off_2, graph_w, sub_h, n, None, ch2_col);
        }
    } else {
        // Single combined image
        draw_grid(&mut pixels, scale_margin_left, 0, graph_w, graph_h);
        if settings.show_ch1 {
            draw_ch(&mut pixels, &cap.ch1, [255, 217, 25], scale_margin_left, 0, graph_w, graph_h);
        }
        if settings.show_ch2 {
            if let Some(ch2) = &cap.ch2 {
                draw_ch(&mut pixels, ch2, [38, 217, 255], scale_margin_left, 0, graph_w, graph_h);
            }
        }
        if settings.show_scales {
            let ch1_col = if settings.show_ch1 { Some([255u8, 217, 25]) } else { None };
            let ch2_col = if settings.show_ch2 && has_ch2 { Some([38u8, 217, 255]) } else { None };
            draw_scales_on_region(&mut pixels, scale_margin_left, 0, graph_w, graph_h, cap.ch1.len(), ch1_col, ch2_col);
        }
    }

    write_png(path, width, height, &pixels)?;
    Ok(())
}

/// Draw a string using a tiny 3x5 pixel font. Each character is 4px wide (3+1 spacing).
fn draw_text_tiny(pixels: &mut [u8], img_width: u32, text: &str, x: u32, y: u32, color: [u8; 3]) {
    let mut cx = x;
    for ch in text.chars() {
        let glyph = tiny_glyph(ch);
        for row in 0..5u32 {
            for col in 0..3u32 {
                if glyph[row as usize] & (1 << (2 - col)) != 0 {
                    let px = cx + col;
                    let py = y + row;
                    let idx = ((py * img_width + px) * 3) as usize;
                    if idx + 2 < pixels.len() {
                        pixels[idx..idx + 3].copy_from_slice(&color);
                    }
                }
            }
        }
        cx += 4;
    }
}

/// Returns a 5-row bitmask (3 bits wide) for a character.
fn tiny_glyph(ch: char) -> [u8; 5] {
    match ch {
        '0' => [0b111, 0b101, 0b101, 0b101, 0b111],
        '1' => [0b010, 0b110, 0b010, 0b010, 0b111],
        '2' => [0b111, 0b001, 0b111, 0b100, 0b111],
        '3' => [0b111, 0b001, 0b111, 0b001, 0b111],
        '4' => [0b101, 0b101, 0b111, 0b001, 0b001],
        '5' => [0b111, 0b100, 0b111, 0b001, 0b111],
        '6' => [0b111, 0b100, 0b111, 0b101, 0b111],
        '7' => [0b111, 0b001, 0b001, 0b001, 0b001],
        '8' => [0b111, 0b101, 0b111, 0b101, 0b111],
        '9' => [0b111, 0b101, 0b111, 0b001, 0b111],
        '.' => [0b000, 0b000, 0b000, 0b000, 0b010],
        '-' => [0b000, 0b000, 0b111, 0b000, 0b000],
        'V' => [0b101, 0b101, 0b101, 0b101, 0b010],
        ' ' => [0b000, 0b000, 0b000, 0b000, 0b000],
        _ => [0b000, 0b000, 0b000, 0b000, 0b000],
    }
}

/// Write an RGB pixel buffer as a PNG file.
/// Minimal implementation using uncompressed deflate (store blocks).
fn write_png(path: &std::path::Path, width: u32, height: u32, rgb: &[u8]) -> anyhow::Result<()> {
    use std::io::Write;

    let mut file = std::fs::File::create(path)?;

    // PNG signature
    file.write_all(&[137, 80, 78, 71, 13, 10, 26, 10])?;

    // IHDR chunk
    let mut ihdr = Vec::with_capacity(13);
    ihdr.extend_from_slice(&width.to_be_bytes());
    ihdr.extend_from_slice(&height.to_be_bytes());
    ihdr.push(8); // bit depth
    ihdr.push(2); // color type: RGB
    ihdr.push(0); // compression
    ihdr.push(0); // filter
    ihdr.push(0); // interlace
    write_png_chunk(&mut file, b"IHDR", &ihdr)?;

    // IDAT chunk — raw image data compressed with deflate (store only).
    // Each row is: filter_byte(0) + RGB pixels.
    let row_len = 1 + width as usize * 3;
    let raw_size = row_len * height as usize;

    // Build the unfiltered image data
    let mut raw = Vec::with_capacity(raw_size);
    for y in 0..height as usize {
        raw.push(0); // filter byte: None
        let start = y * width as usize * 3;
        let end = start + width as usize * 3;
        raw.extend_from_slice(&rgb[start..end]);
    }

    // Wrap in zlib format (store blocks, no compression)
    let deflated = zlib_store(&raw);
    write_png_chunk(&mut file, b"IDAT", &deflated)?;

    // IEND chunk
    write_png_chunk(&mut file, b"IEND", &[])?;

    Ok(())
}

fn write_png_chunk(w: &mut dyn std::io::Write, chunk_type: &[u8; 4], data: &[u8]) -> std::io::Result<()> {
    let len = data.len() as u32;
    w.write_all(&len.to_be_bytes())?;
    w.write_all(chunk_type)?;
    w.write_all(data)?;
    // CRC32 over type + data
    let crc = png_crc32(chunk_type, data);
    w.write_all(&crc.to_be_bytes())?;
    Ok(())
}

fn png_crc32(chunk_type: &[u8; 4], data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in chunk_type.iter().chain(data.iter()) {
        let idx = ((crc ^ b as u32) & 0xFF) as usize;
        crc = CRC_TABLE[idx] ^ (crc >> 8);
    }
    crc ^ 0xFFFF_FFFF
}

/// Wraps data in zlib format using only store (no compression) deflate blocks.
fn zlib_store(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len() + 64);
    // Zlib header: CM=8 (deflate), CINFO=7 (32K window), FCHECK so header%31==0
    out.push(0x78); // CMF
    out.push(0x01); // FLG (no dict, level 0; 0x7801 % 31 == 0)

    // Deflate store blocks — max 65535 bytes per block
    let mut remaining = data;
    while !remaining.is_empty() {
        let block_size = remaining.len().min(65535);
        let is_last = block_size == remaining.len();
        out.push(if is_last { 0x01 } else { 0x00 }); // BFINAL + BTYPE=00
        let len = block_size as u16;
        let nlen = !len;
        out.extend_from_slice(&len.to_le_bytes());
        out.extend_from_slice(&nlen.to_le_bytes());
        out.extend_from_slice(&remaining[..block_size]);
        remaining = &remaining[block_size..];
    }

    // Adler-32 checksum
    let adler = adler32(data);
    out.extend_from_slice(&adler.to_be_bytes());
    out
}

fn adler32(data: &[u8]) -> u32 {
    let mut a: u32 = 1;
    let mut b: u32 = 0;
    for &byte in data {
        a = (a + byte as u32) % 65521;
        b = (b + a) % 65521;
    }
    (b << 16) | a
}

// CRC32 lookup table for PNG
const CRC_TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut n = 0usize;
    while n < 256 {
        let mut c = n as u32;
        let mut k = 0;
        while k < 8 {
            if c & 1 != 0 {
                c = 0xEDB8_8320 ^ (c >> 1);
            } else {
                c >>= 1;
            }
            k += 1;
        }
        table[n] = c;
        n += 1;
    }
    table
};

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use dso_parser::Capture;

    /// Helper to construct a test CaptureEntry from a Capture.
    fn test_entry(capture: Capture) -> CaptureEntry {
        let now = Local::now();
        let record = dso_parser::capture_to_record(&capture, now, None);
        CaptureEntry { capture, timestamp: now, record }
    }

    #[test]
    fn stats_handles_empty() {
        assert_eq!(stats(&[]), (0, 0, 0.0));
    }

    #[test]
    fn stats_known_values() {
        let (mn, mx, av) = stats(&[10, 20, 30, 40]);
        assert_eq!(mn, 10);
        assert_eq!(mx, 40);
        assert!((av - 25.0).abs() < 1e-3);
    }

    #[test]
    fn app_default_no_capture() {
        let (app, _) = App::new();
        assert!(app.current_capture().is_none());
        assert_eq!(app.current, 0);
        assert!(!app.listening);
    }

    #[test]
    fn app_updates_measurement_range() {
        let (mut app, _) = App::new();
        let _ = app.update(Message::MeasCursorXChanged(0.1, 0.9));
        assert_eq!(app.settings.cursor_x_range, (0.1, 0.9));
    }

    #[test]
    fn app_select_capture_clamps() {
        let (mut app, _) = App::new();
        app.dump = Some(LoadedDump {
            path: PathBuf::from("test.bin"),
            captures: vec![test_entry(Capture {
                ch1: vec![1, 2, 3],
                ch2: None,
            })],
        });
        let _ = app.update(Message::SelectCapture(99));
        assert_eq!(app.current, 0);
        let _ = app.update(Message::SelectCapture(0));
        assert_eq!(app.current, 0);
    }

    #[test]
    fn toggling_channels_persists_in_settings() {
        let (mut app, _) = App::new();
        let before = app.settings.show_ch1;
        let _ = app.update(Message::ToggleCh1(!before));
        assert_eq!(app.settings.show_ch1, !before);
    }

    #[test]
    fn device_cursor_hooks_default_false() {
        let (app, _) = App::new();
        assert!(!app.capture_has_device_cursor_x());
        assert!(!app.capture_has_device_cursor_y());
    }

    #[test]
    fn view_renders_without_panic_when_empty() {
        let (app, _) = App::new();
        let _ = app.view();
    }

    #[test]
    fn view_renders_with_capture() {
        let (mut app, _) = App::new();
        app.dump = Some(LoadedDump {
            path: PathBuf::from("test.bin"),
            captures: vec![test_entry(Capture {
                ch1: (0..100).map(|i| i as u8).collect(),
                ch2: None,
            })],
        });
        let _ = app.view();
    }

    #[test]
    fn adc_voltage_conversion() {
        // 128 → 0V (center), 0 → -5V, 255 → ~+5V
        assert!((adc_to_voltage(128) - 0.0).abs() < 0.05);
        assert!((adc_to_voltage(0) - (-5.0)).abs() < 0.05);
        assert!((adc_to_voltage(255) - 4.96).abs() < 0.1);
    }

    #[test]
    fn samples_to_time_conversion() {
        let ms = samples_to_time_ms(1000);
        assert!((ms - 1.0).abs() < 0.01);
    }

    #[test]
    fn png_crc32_is_deterministic() {
        let crc1 = png_crc32(b"IHDR", &[0u8; 13]);
        let crc2 = png_crc32(b"IHDR", &[0u8; 13]);
        assert_eq!(crc1, crc2);
    }

    #[test]
    fn adler32_known_value() {
        // adler32 of "Wikipedia" = 0x11E60398
        let result = adler32(b"Wikipedia");
        assert_eq!(result, 0x11E6_0398);
    }

    #[test]
    fn settings_dialog_opens_and_closes() {
        let (mut app, _) = App::new();
        let _ = app.update(Message::OpenSettings);
        assert!(app.show_settings);
        let _ = app.update(Message::CloseSettings);
        assert!(!app.show_settings);
    }

    #[test]
    fn serial_start_without_port_shows_error() {
        let (mut app, _) = App::new();
        app.settings.serial_port = None;
        let _ = app.update(Message::StartListening);
        assert!(!app.listening);
        assert!(app.status.contains("Select a serial port"));
    }
}

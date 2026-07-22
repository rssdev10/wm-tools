//! Application state, messages, update and view (iced 0.14).

use std::path::PathBuf;

use chrono::{DateTime, Local};
use dso_parser::{ADC_VALUE_MID, Capture, DEFAULT_BUFFER_DEPTH};
#[cfg(test)]
use dso_parser::ADC_VALUE_MAX;
use iced::widget::{
    button, canvas, checkbox, column, container, pick_list, progress_bar, row, rule,
    scrollable, text, text_input, Space,
};
use iced::{
    event,
    keyboard,
    mouse,
    Color, Event,
    Element, Length, Subscription, Task, Theme,
};

use crate::canvas::{Scope, T_CELLS, Thumbnail, V_CELLS};
use crate::i18n::t;
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
    ModifiersChanged(keyboard::Modifiers),
    LeftMouseReleased,
    MeasCursorYChanged(f32, f32),
    // Scale inputs (V/div, time/div, V offset)
    VPerCellChangedCh1(String),
    VPerCellChangedCh2(String),
    TPerCellChanged(String),
    VOffsetChangedCh1(String),
    VOffsetChangedCh2(String),
    // Graph click — move nearest cursor
    GraphClicked(f32, f32),
    // Misc
    OpenInstructions,
    CloseInstructions,
    ToggleAlertOnData(bool),
    ToggleAutoListen(bool),
    // Settings dialog
    OpenSettings,
    CloseSettings,
    SettingsBaudChanged(String),
    SettingsPortChanged(String),
    SettingsPngWidthChanged(String),
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
    // Language selection
    ChangeLanguage(String),
    // Port scan tick (auto-refresh)
    PortScanTick,
    // Window resized
    WindowResized(f32, f32),
    // Open a URL in the default browser
    OpenUrl(String),
    // Context menu on thumbnails (right-click via iced_aw::ContextMenu)
    ContextMenuDeleteAt(usize),
    UndoDelete,
    UndoExpiredTick,
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

/// Wrapper for language display in pick_list.
#[derive(Debug, Clone, PartialEq, Eq)]
struct LangOption {
    code: String,
    display: String,
}

impl std::fmt::Display for LangOption {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.display)
    }
}

/// Factor applied to cursor movement when Ctrl is held (fine-drag mode).
const FINE_DRAG_FACTOR: f32 = 0.1;

/// Anchor state for fine-dragging a measurement cursor range.
/// Stores the logical and raw slider values at the moment Ctrl+drag began,
/// so that only 1/10 of the raw delta is applied to the logical value.
#[derive(Debug, Clone, Copy)]
struct FineRangeDrag {
    /// Slider values when the Ctrl-drag began.
    logical_start: (f32, f32),
    /// First raw values emitted by the range slider.
    raw_start: (f32, f32),
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
    /// Instructions overlay open flag.
    show_instructions: bool,
    /// Editable fields in settings dialog.
    settings_baud_input: String,
    settings_port_input: String,
    /// Editable PNG export width in settings dialog.
    png_width_input: String,
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
    /// Last deleted capture, available for undo (index it was at, entry).
    deleted_capture: Option<(usize, CaptureEntry)>,
    /// Timestamp of the last deletion, for 30-second undo expiry.
    deleted_at: Option<std::time::Instant>,
    ctrl_pressed: bool,
    measurement_cursor_x_fine_drag: Option<FineRangeDrag>,
    measurement_cursor_y_fine_drag: Option<FineRangeDrag>,
    /// Text input state for V/div scale (CH1).
    v_per_div_ch1_input: String,
    /// Text input state for V/div scale (CH2).
    v_per_div_ch2_input: String,
    /// Text input state for t/div scale.
    t_per_div_input: String,
    /// Text input state for V/offset (CH1).
    v_offset_ch1_input: String,
    /// Text input state for V/offset (CH2).
    v_offset_ch2_input: String,
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
        let v_per_div_ch1_str = format_float(settings.v_per_div_ch1);
        let v_per_div_ch2_str = format_float(settings.v_per_div_ch2);
        let t_per_div_str = format_float(settings.t_per_div_ms);
        let v_offset_ch1_str = format_float(settings.v_offset_ch1);
        let v_offset_ch2_str = format_float(settings.v_offset_ch2);
        let png_width_str = settings.png_export_width.to_string();
        (
            Self {
                settings,
                dump: None,
                current: 0,
                status: t!("app.ready").to_string(),
                available_ports: ports,
                listening: false,
                serial_handle: None,
                serial_rx: None,
                capture_progress: 0,
                show_settings: false,
                show_instructions: false,
                settings_baud_input: baud_str,
                settings_port_input: port_str,
                png_width_input: png_width_str,
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
                deleted_capture: None,
                deleted_at: None,
                ctrl_pressed: false,
                measurement_cursor_x_fine_drag: None,
                measurement_cursor_y_fine_drag: None,
                v_per_div_ch1_input: v_per_div_ch1_str,
                v_per_div_ch2_input: v_per_div_ch2_str,
                t_per_div_input: t_per_div_str,
                v_offset_ch1_input: v_offset_ch1_str,
                v_offset_ch2_input: v_offset_ch2_str,
            },
            Task::none(),
        )
    }

    pub fn title(&self) -> String {
        let ver = env!("CARGO_PKG_VERSION");
        match &self.dump {
            Some(d) => format!(
                "{} v{ver} — {} ({} {})",
                t!("app.title"),
                d.path.file_name().and_then(|s| s.to_str()).unwrap_or("?"),
                d.captures.len(),
                if d.captures.len() == 1 { "capture" } else { "captures" }
            ),
            None => format!("{} v{ver}", t!("app.title")),
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

        // Track window resize to persist size
        subs.push(
            iced::window::resize_events()
                .map(|(_id, size)| Message::WindowResized(size.width, size.height)),
        );

        // Poll firmware events
        if self.firmware_rx.is_some() {
            subs.push(
                iced::time::every(std::time::Duration::from_millis(100))
                    .map(|_| Message::FirmwareEvent(crate::flash::FlashEvent::Status(String::new()))),
            );
        }

        // Expire undo buffer after 30 seconds
        if self.deleted_capture.is_some() {
            subs.push(
                iced::time::every(std::time::Duration::from_secs(1))
                    .map(|_| Message::UndoExpiredTick),
            );
        }

        // Keyboard modifier and mouse-release events for fine-drag
        subs.push(event::listen_with(|event, _status, _window| match event {
            Event::Keyboard(keyboard::Event::ModifiersChanged(modifiers)) => {
                Some(Message::ModifiersChanged(modifiers))
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                Some(Message::LeftMouseReleased)
            }
            _ => None,
        }));

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
                self.deleted_capture = None;
                self.deleted_at = None;
                let n = d.captures.len();
                let path_s = d.path.display().to_string();
                self.status = t!("label.loaded_ok", n = n, path = path_s).to_string();
                log::info!("{}", self.status);
                self.current = 0;
                self.dump = Some(d);
                // Auto-populate from the first record's scope_state only if it
                // is the record being shown (single-capture files). For multi-
                // capture files the user must select a record to apply its
                // scope_state via the thumbnail strip.
                if self.dump.as_ref().map(|d| d.captures.len() <= 1).unwrap_or(false) {
                    if let Some(state) = self.current_entry()
                        .and_then(|e| e.record.scope_state.clone())
                    {
                        self.apply_scope_state(&state);
                    }
                }
                Task::none()
            }
            Message::Loaded(Err(e)) => {
                self.status = t!("label.load_error", e = format!("{e}")).to_string();
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
                        Ok(()) => self.status = t!("label.saved_ok", n = dump.captures.len(), path = path.display().to_string()).to_string(),
                        Err(e) => {
                            self.status = t!("label.save_error", e = format!("{e}")).to_string();
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
                        Ok(()) => self.status = t!("label.png_saved", path = path.display().to_string()).to_string(),
                        Err(e) => {
                            self.status = t!("label.png_error", e = format!("{e}")).to_string();
                            log::error!("PNG error: {e}");
                        }
                    }
                } else {
                    self.status = t!("app.no_capture").to_string();
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
                        Ok(()) => self.status = t!("label.csv_written", path = path.display().to_string()).to_string(),
                        Err(e) => {
                            self.status = t!("label.csv_error", e = format!("{e}")).to_string();
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
                        // Clone scope_state before calling apply_scope_state
                        // to avoid borrow conflict with self.dump.
                        let scope_state = d.captures[i].record.scope_state.clone();
                        if let Some(ref state) = scope_state {
                            self.apply_scope_state(state);
                        }
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
                        self.status = t!("app.select_port").to_string();
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
                let port = self.settings.serial_port.as_deref().unwrap_or("?").to_string();
                self.status = t!("label.listening_on", port = port).to_string();
                Task::none()
            }
            Message::StopListening => {
                if let Some(mut h) = self.serial_handle.take() {
                    h.stop();
                }
                self.serial_rx = None;
                self.listening = false;
                self.status = t!("label.stopped").to_string();
                Task::none()
            }
            Message::SerialEvent(_) => {
                // Drain all pending events from the channel.
                if let Some(rx) = &mut self.serial_rx {
                    while let Ok(evt) = rx.try_recv() {
                        match evt {
                            SerialEvent::Connected => {
                                let port = self.settings.serial_port.as_deref().unwrap_or("?").to_string();
                                self.status = t!("label.connected_to", port = port).to_string();
                                log::info!("{}", self.status);
                            }
                            SerialEvent::Disconnected => {
                                self.status = t!("label.disconnected").to_string();
                                log::warn!("{}", self.status);
                            }
                            SerialEvent::Progress(n) => {
                                if n > 0 {
                                    // Beep when data first starts arriving
                                    if self.capture_progress == 0 && self.settings.alert_on_data {
                                        play_beep_start();
                                    }
                                    self.capture_progress = n;
                                    self.status = t!("label.receiving", n = n).to_string();
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
                                self.deleted_capture = None;
                                self.deleted_at = None;
                                let total = self.dump.as_ref().map(|d| d.captures.len()).unwrap_or(0);
                                self.status = t!("label.received_captures", count = count, total = total).to_string();
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
                                        let probe2 = 10f64.powi(s.ch2.probe_mode as i32);
                                        self.settings.v_per_div_ch1 = s.ch1.volt_scale_uv as f64 / 1_000_000.0 * probe1;
                                        self.settings.v_per_div_ch2 = s.ch2.volt_scale_uv as f64 / 1_000_000.0 * probe2;
                                        self.settings.t_per_div_ms = s.timebase_ns_per_div as f64 / 1_000_000.0;
                                        // pixel_to_volts at screen centre gives  −zero_volt_pixels × vpp × mult,
                                        // but zero_volt_uv stores the value with opposite sign → negate.
                                        self.settings.v_offset_ch1 = -(s.ch1.zero_volt_uv as f64) / 1_000_000.0 * probe1;
                                        self.settings.v_offset_ch2 = -(s.ch2.zero_volt_uv as f64) / 1_000_000.0 * probe2;
                                        self.v_per_div_ch1_input = format_float(self.settings.v_per_div_ch1);
                                        self.v_per_div_ch2_input = format_float(self.settings.v_per_div_ch2);
                                        self.t_per_div_input = format_float(self.settings.t_per_div_ms);
                                        self.v_offset_ch1_input = format_float(self.settings.v_offset_ch1);
                                        self.v_offset_ch2_input = format_float(self.settings.v_offset_ch2);
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
                                        self.deleted_capture = None;
                                        self.deleted_at = None;
                                        let total = self.dump.as_ref().map(|d| d.captures.len()).unwrap_or(0);
                                        self.status = t!("label.screenshot_received",
                                            ch1 = dso_parser::format_uv((self.settings.v_per_div_ch1 * 1_000_000.0) as i64, 0),
                                            ch2 = dso_parser::format_uv((self.settings.v_per_div_ch2 * 1_000_000.0) as i64, 0),
                                            timebase = s.timebase_label(),
                                            total = total,
                                        ).to_string();
                                    }
                                    Err(e) => {
                                        self.status = t!("label.load_error", e = format!("{e}")).to_string();
                                        log::error!("screenshot parse: {e}");
                                    }
                                }
                            }
                            SerialEvent::Error(e) => {
                                self.status = format!("Serial: {e}");
                                // Not translated — raw device error message
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
            Message::MeasCursorXChanged(raw_lo, raw_hi) => {
                let ctrl = self.ctrl_pressed;
                let anchor = &mut self.measurement_cursor_x_fine_drag;
                let range = &mut self.settings.cursor_x_range;
                App::apply_fine_drag(ctrl, anchor, range, raw_lo, raw_hi);
                Task::none()
            }
            Message::ModifiersChanged(modifiers) => {
                self.ctrl_pressed = modifiers.control();
                if !self.ctrl_pressed {
                    self.measurement_cursor_x_fine_drag = None;
                    self.measurement_cursor_y_fine_drag = None;
                }
                Task::none()
            }
            Message::LeftMouseReleased => {
                self.measurement_cursor_x_fine_drag = None;
                self.measurement_cursor_y_fine_drag = None;
                Task::none()
            }
            Message::MeasCursorYChanged(raw_lo, raw_hi) => {
                let ctrl = self.ctrl_pressed;
                let anchor = &mut self.measurement_cursor_y_fine_drag;
                let range = &mut self.settings.cursor_y_range;
                App::apply_fine_drag(ctrl, anchor, range, raw_lo, raw_hi);
                Task::none()
            }
            Message::VPerCellChangedCh1(s) => {
                self.v_per_div_ch1_input = s.clone();
                if let Ok(v) = s.parse::<f64>() {
                    if v > 0.0 && v.is_finite() {
                        self.settings.v_per_div_ch1 = v;
                        self.settings.save();
                    }
                }
                Task::none()
            }
            Message::VPerCellChangedCh2(s) => {
                self.v_per_div_ch2_input = s.clone();
                if let Ok(v) = s.parse::<f64>() {
                    if v > 0.0 && v.is_finite() {
                        self.settings.v_per_div_ch2 = v;
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
            Message::VOffsetChangedCh1(s) => {
                self.v_offset_ch1_input = s.clone();
                if let Ok(v) = s.parse::<f64>() {
                    if v.is_finite() {
                        self.settings.v_offset_ch1 = v;
                        self.settings.save();
                    }
                }
                Task::none()
            }
            Message::VOffsetChangedCh2(s) => {
                self.v_offset_ch2_input = s.clone();
                if let Ok(v) = s.parse::<f64>() {
                    if v.is_finite() {
                        self.settings.v_offset_ch2 = v;
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
            Message::OpenInstructions => {
                self.show_instructions = true;
                Task::none()
            }
            Message::CloseInstructions => {
                self.show_instructions = false;
                Task::none()
            }
            Message::OpenSettings => {
                self.show_settings = true;
                self.settings_baud_input = self.settings.baud_rate.to_string();
                self.settings_port_input = self.settings.serial_port.clone().unwrap_or_default();
                self.png_width_input = self.settings.png_export_width.to_string();
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
                // PNG export width: minimum 300 px, 0 = default small.
                if let Ok(w) = self.png_width_input.trim().parse::<u32>() {
                    if w == 0 || w >= 300 {
                        self.settings.png_export_width = w;
                    }
                }
                self.settings.save();
                self.show_settings = false;
                self.status = t!("app.settings_saved").to_string();
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
            Message::SettingsPngWidthChanged(s) => {
                self.png_width_input = s;
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
            Message::ChangeLanguage(l) => {
                log::info!("language -> {l}");
                self.settings.language = l;
                self.settings.save();
                crate::i18n::set_language(&self.settings.language);
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
            Message::WindowResized(w, h) => {
                self.settings.window_size = (w, h);
                self.settings.save();
                Task::none()
            }
            Message::OpenUrl(url) => {
                let _ = open::that(url);
                Task::none()
            }
            Message::ContextMenuDeleteAt(idx) => {
                if let Some(ref mut dump) = self.dump {
                    if idx < dump.captures.len() {
                        let entry = dump.captures.remove(idx);
                        self.deleted_capture = Some((idx, entry));
                        self.deleted_at = Some(std::time::Instant::now());
                        if self.current >= dump.captures.len() && self.current > 0 {
                            self.current -= 1;
                        }
                        self.status = t!("label.deleted", n = idx + 1).to_string();
                    }
                }
                Task::none()
            }
            Message::UndoDelete => {
                self.deleted_at = None;
                if let Some((idx, entry)) = self.deleted_capture.take() {
                    let dump = self.dump.get_or_insert_with(|| LoadedDump {
                        path: PathBuf::from("<serial>"),
                        captures: vec![],
                    });
                    let insert_at = idx.min(dump.captures.len());
                    dump.captures.insert(insert_at, entry);
                    self.current = insert_at;
                    self.status = t!("label.restored", n = insert_at + 1).to_string();
                }
                Task::none()
            }
            Message::UndoExpiredTick => {
                if let Some(at) = self.deleted_at {
                    if at.elapsed().as_secs() >= 30 {
                        self.deleted_capture = None;
                        self.deleted_at = None;
                        // Clear the status hint only if it still mentions undo
                        if self.status.contains("Undo") {
                            self.status = t!("label.undo_expired").to_string();
                        }
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
                        Ok(()) => self.status = t!("label.png_saved", path = path.display().to_string()).to_string(),
                        Err(e) => self.status = t!("label.png_error", e = format!("{e}")).to_string(),
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
                        Ok(()) => self.status = t!("label.csv_written", path = path.display().to_string()).to_string(),
                        Err(e) => self.status = t!("label.csv_error", e = format!("{e}")).to_string(),
                    }
                }
                Task::none()
            }
            Message::OpenFirmware => {
                self.show_firmware = true;
                self.firmware_port = self.settings.serial_port.clone().unwrap_or_default();
                self.firmware_file = None;
                self.firmware_progress = None;
                self.firmware_status = t!("label.firmware_status_ready").to_string();
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
                self.firmware_status = t!("label.firmware_confirm").to_string();
                Task::none()
            }
            Message::FirmwareStart => {
                self.firmware_confirming = false;
                let port = self.firmware_port.clone();
                let file = match &self.firmware_file {
                    Some(p) => p.to_string_lossy().to_string(),
                    None => {
                        self.firmware_status = t!("label.firmware_no_file").to_string();
                        return Task::none();
                    }
                };
                if port.is_empty() {
                    self.firmware_status = t!("label.firmware_no_port").to_string();
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
                self.firmware_status = t!("label.firmware_starting").to_string();
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
                                        self.firmware_status = t!("label.firmware_complete").to_string();
                                    }
                                    Err(e) => {
                                        self.firmware_status = t!("label.firmware_error", e = format!("{e}")).to_string();
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
        if self.show_instructions {
            return self.instructions_overlay();
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
        let thumbs = thumbnails(&self.dump, self.current, self.deleted_capture.is_some());
        let actions = self.action_buttons();

        let bottom = column![
            controls,
            rule::horizontal(1),
            thumbs,
            rule::horizontal(1),
            actions,
        ]
        .spacing(4)
        .padding(4);

        container(column![menu_bar, title_bar, graph_row, bottom].spacing(2))
            .padding(4)
            .into()
    }

    fn settings_dialog(&self) -> Element<'_, Message> {
        let title = text(t!("label.settings_title")).size(22);

        // ── Language / Display card ──
        let lang_display_names = crate::i18n::language_display_names();
        let lang_options: Vec<LangOption> = lang_display_names
            .iter()
            .map(|(code, display)| LangOption {
                code: code.clone(),
                display: display.clone(),
            })
            .collect();
        let current_lang = lang_options
            .iter()
            .find(|o| o.code == self.settings.language)
            .cloned();
        let lang_pick = pick_list(
            lang_options,
            current_lang,
            |opt: LangOption| Message::ChangeLanguage(opt.code),
        );

        let display_card = container(
            column![
                row![
                    text(t!("label.display_section")).size(15),
                    Space::new().width(10.0),
                    text("🎨").size(20),
                ]
                .align_y(iced::Alignment::Center),
                Space::new().height(10.0),
                row![
                    text(t!("label.language")).size(13),
                    Space::new().width(Length::Fill),
                    lang_pick,
                ]
                .align_y(iced::Alignment::Center),
                Space::new().height(10.0),
                row![
                    text(t!("label.png_width")).size(13),
                    Space::new().width(Length::Fill),
                    text_input("0", &self.png_width_input)
                        .on_input(Message::SettingsPngWidthChanged)
                        .size(13)
                        .width(Length::Fixed(70.0)),
                ]
                .align_y(iced::Alignment::Center),
            ]
            .spacing(8),
        )
        .padding(16)
        .style(container::bordered_box)
        .width(Length::Fill);

        // ── Serial port card ──
        let serial_card = container(
            column![
                row![
                    text(t!("label.serial_section")).size(15),
                    Space::new().width(10.0),
                    text("🔌").size(20),
                ]
                .align_y(iced::Alignment::Center),
                Space::new().height(10.0),
                text(t!("label.default_port")).size(13),
                text_input("e.g. /dev/tty.usbserial-1110", &self.settings_port_input)
                    .on_input(Message::SettingsPortChanged)
                    .size(13),
                if self.available_ports.is_empty() {
                    text(t!("label.no_ports")).size(11)
                } else {
                    text(t!("label.available_ports", ports = self.available_ports.join(", "))).size(11)
                },
                text(t!("label.baud_rate")).size(13),
                text_input("115200", &self.settings_baud_input)
                    .on_input(Message::SettingsBaudChanged)
                    .size(13),
                checkbox(self.settings.auto_listen)
                    .label(t!("label.auto_listen"))
                    .on_toggle(Message::ToggleAutoListen),
                checkbox(self.settings.alert_on_data)
                    .label(t!("label.beep_on_capture"))
                    .on_toggle(Message::ToggleAlertOnData),
            ]
            .spacing(8),
        )
        .padding(16)
        .style(container::bordered_box)
        .width(Length::Fill);

        // ── Maintenance card (firmware updates) ──
        let maintenance_card = container(
            column![
                row![
                    text(t!("label.firmware_title")).size(15),
                    Space::new().width(10.0),
                    text("🛠️").size(20),
                ]
                .align_y(iced::Alignment::Center),
                Space::new().height(10.0),
                text(t!("label.firmware_warning")).size(10),
                Space::new().height(8.0),
                button(text(t!("btn.firmware_update")).size(13))
                    .padding([6, 14])
                    .on_press(Message::OpenFirmware),
            ]
            .spacing(4),
        )
        .padding(16)
        .style(container::bordered_box)
        .width(Length::Fill);

        // ── About card ──
        let ver = env!("CARGO_PKG_VERSION");
        let repo = env!("CARGO_PKG_REPOSITORY");
        let config_path = crate::settings::config_dir_path()
            .map(|p| p.display().to_string())
            .unwrap_or_else(|| "unknown".to_string());
        let about_card = container(
            column![
                row![
                    text(t!("label.about_section")).size(15),
                    Space::new().width(10.0),
                    text("ⓘ").size(20),
                ]
                .align_y(iced::Alignment::Center),
                Space::new().height(10.0),
                text(format!("DSO3D12 GUI v{ver}")).size(13),
                button(text(repo).size(11))
                    .on_press(Message::OpenUrl(repo.to_string()))
                    .style(button::text),
                text(format!("{}{}", t!("label.config_path"), config_path)).size(11),
            ]
            .spacing(6),
        )
        .padding(16)
        .style(container::bordered_box)
        .width(Length::Fill);

        let close_btn = button(text(t!("btn.save")).size(13)).on_press(Message::CloseSettings);

        scrollable(
            container(
                column![
                    title,
                    Space::new().height(Length::Fixed(8.0)),
                    row![display_card, serial_card].spacing(8).width(Length::Fill),
                    Space::new().height(Length::Fixed(8.0)),
                    row![maintenance_card, about_card].spacing(8).width(Length::Fill),
                    Space::new().height(Length::Fixed(16.0)),
                    close_btn,
                ]
                .spacing(4)
                .padding(20)
                .max_width(700),
            )
            .center_x(Length::Fill)
            .center_y(Length::Fill),
        )
        .direction(scrollable::Direction::Vertical(
            scrollable::Scrollbar::new().scroller_width(0.0).width(0.0),
        ))
        .width(Length::Fill)
        .into()
    }

    fn firmware_dialog(&self) -> Element<'_, Message> {
        let title = text(t!("label.firmware_title")).size(22);
        let warning = container(
            text(t!("label.firmware_warning"))
                .size(12),
        )
        .padding(8)
        .style(container::bordered_box);

        let boot_instructions = container(
            column![
                text(t!("label.boot_title")).size(13),
                Space::new().height(Length::Fixed(4.0)),
                text(t!("label.boot_1")).size(11),
                text(t!("label.boot_2")).size(11),
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
        .placeholder(t!("label.port"))
        .text_size(13);
        let refresh_btn = button(text(t!("btn.refresh")).size(12)).on_press(Message::FirmwareRefreshPorts);

        let file_label = match &self.firmware_file {
            Some(p) => text(p.file_name().and_then(|n| n.to_str()).unwrap_or("?")).size(13),
            None => text(t!("label.no_file")).size(13),
        };
        let file_btn = button(text(t!("btn.browse")).size(13)).on_press(Message::FirmwareFileClicked);

        let progress_bar_el: Element<'_, Message> = if let Some((cur, total)) = self.firmware_progress
        {
            let pct = if total > 0 {
                (cur as f32 / total as f32) * 100.0
            } else {
                0.0
            };
            column![
                text(t!("label.pct_progress", pct = format!("{:.0}", pct), cur = cur, total = total)).size(12),
                progress_bar(0.0..=total as f32, cur as f32),
            ]
            .spacing(2)
            .into()
        } else {
            Space::new().height(Length::Fixed(0.0)).into()
        };

        let status_text = text(self.firmware_status.clone()).size(12);
        // status_text is dynamically set, not translated here

        let is_flashing = self.firmware_handle.is_some();
        let update_btn = if is_flashing {
            button(text(t!("btn.updating")).size(13))
        } else if self.firmware_confirming {
            button(text(t!("btn.confirm_update")).size(13)).on_press(Message::FirmwareStart)
        } else {
            button(text(t!("btn.update")).size(13)).on_press(Message::FirmwareConfirmStart)
        };
        let close_btn = button(text(t!("btn.close")).size(13)).on_press(Message::CloseFirmware);

        container(
            column![
                title,
                Space::new().height(Length::Fixed(8.0)),
                warning,
                Space::new().height(Length::Fixed(8.0)),
                boot_instructions,
                Space::new().height(Length::Fixed(8.0)),
                text(t!("label.port")).size(13),
                row![port_picker, refresh_btn].spacing(8).align_y(iced::Alignment::Center),
                Space::new().height(Length::Fixed(8.0)),
                text(t!("label.firmware_file")).size(13),
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

    fn instructions_overlay(&self) -> Element<'_, Message> {
        let title = text(t!("label.instructions_title")).size(22);

        let content = instructions_panel();

        let close_btn = button(text(t!("btn.close")).size(13)).on_press(Message::CloseInstructions);

        container(
            column![
                title,
                Space::new().height(Length::Fixed(8.0)),
                content,
                Space::new().height(Length::Fixed(16.0)),
                close_btn,
            ]
            .spacing(4)
            .padding(20)
            .max_width(700),
        )
        .center_x(Length::Fill)
        .center_y(Length::Fill)
        .into()
    }

    fn menu_bar(&self) -> Element<'_, Message> {
        let port_display = match &self.settings.serial_port {
            Some(p) => text(p.clone()).size(12),
            None => text(t!("app.no_port")).size(12),
        };

        let listen_btn = if self.listening {
            button(text(t!("btn.stop")).size(12)).on_press(Message::StopListening)
        } else {
            button(text(t!("btn.listen")).size(12)).on_press(Message::StartListening)
        };

        let port_picker = pick_list(
            self.available_ports.clone(),
            self.settings.serial_port.clone(),
            Message::PortSelected,
        )
        .placeholder(t!("label.port"))
        .text_size(12);

        let help_btn = button(text(t!("btn.help")).size(12)).on_press(Message::OpenInstructions);

        let status_row = row![
                text(t!("app.title")).size(14),
                Space::new().width(Length::Fixed(12.0)),
                port_picker,
                port_display,
                listen_btn,
                Space::new().width(Length::Fill),
                text(self.status.clone()).size(12),
                Space::new().width(Length::Fixed(8.0)),
                help_btn
            ]
            .spacing(6)
            .align_y(iced::Alignment::Center);

        container(status_row)
        .padding(4)
        .width(Length::Fill)
        .style(container::rounded_box)
        .into()
    }

    /// Apply oscilloscope state metadata (V/div, V/offset, t/div) to the
    /// current settings and input fields. Used when loading captures that
    /// contain embedded scope_state (screenshot packets or .zwcap files).
    ///
    /// Note: `v_offset_v` is stored with the same sign as the device's
    /// `zero_volt_uv` field.  The device's `pixel_to_volts` formula at
    /// screen centre gives `−zero_volt_pixels × vpp × mult`, i.e. the
    /// *opposite* sign, so we negate here.
    fn apply_scope_state(&mut self, state: &dso_parser::ScopeState) {
        if let Some(ref ch1) = state.ch1 {
            self.settings.v_per_div_ch1 = ch1.vdiv_v;
            self.settings.v_offset_ch1 = -ch1.v_offset_v;
        }
        if let Some(ref ch2) = state.ch2 {
            self.settings.v_per_div_ch2 = ch2.vdiv_v;
            self.settings.v_offset_ch2 = -ch2.v_offset_v;
        }
        self.settings.t_per_div_ms = state.timebase_ns_per_div as f64 / 1_000_000.0;
        self.v_per_div_ch1_input = format_float(self.settings.v_per_div_ch1);
        self.v_per_div_ch2_input = format_float(self.settings.v_per_div_ch2);
        self.t_per_div_input = format_float(self.settings.t_per_div_ms);
        self.v_offset_ch1_input = format_float(self.settings.v_offset_ch1);
        self.v_offset_ch2_input = format_float(self.settings.v_offset_ch2);
        self.settings.save();
    }

    /// Apply fine-drag scaling when Ctrl is held: only 1/10 of the raw
    /// slider delta is applied to the logical cursor range.
    fn apply_fine_drag(
        ctrl_pressed: bool,
        drag_anchor: &mut Option<FineRangeDrag>,
        target_range: &mut (f32, f32),
        raw_lo: f32,
        raw_hi: f32,
    ) {
        if ctrl_pressed {
            let drag = drag_anchor.get_or_insert(FineRangeDrag {
                logical_start: *target_range,
                raw_start: (raw_lo, raw_hi),
            });

            let lo =
                drag.logical_start.0 + (raw_lo - drag.raw_start.0) * FINE_DRAG_FACTOR;
            let hi =
                drag.logical_start.1 + (raw_hi - drag.raw_start.1) * FINE_DRAG_FACTOR;

            *target_range = (lo.clamp(0.0, 1.0), hi.clamp(0.0, 1.0));
        } else {
            *drag_anchor = None;
            *target_range = (raw_lo, raw_hi);
        }
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
            v_per_div_ch1: self.settings.v_per_div_ch1,
            v_per_div_ch2: self.settings.v_per_div_ch2,
            t_per_div_ms: self.settings.t_per_div_ms,
            v_offset_ch1: self.settings.v_offset_ch1,
            v_offset_ch2: self.settings.v_offset_ch2,
            active_channel: self.current_entry()
                .and_then(|e| e.record.scope_state.as_ref())
                .map(|s| s.active_channel)
                .unwrap_or(0),
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

        // Scale parameters (per-channel)
        let v_per_div_ch1 = self.settings.v_per_div_ch1;
        let v_per_div_ch2 = self.settings.v_per_div_ch2;
        let t_per_div_ms = self.settings.t_per_div_ms;
        let v_offset_ch1 = self.settings.v_offset_ch1;
        let v_offset_ch2 = self.settings.v_offset_ch2;
        let total_time_ms = 12.0 * t_per_div_ms;

        let mut items: Vec<Element<'_, Message>> = vec![
            text(t!("label.measurement")).size(13).into(),
            rule::horizontal(1).into(),
        ];

        // ── X cursor (time) ──
        items.push(
            checkbox(self.settings.measurement_cursor_x_enabled)
                .label(t!("label.cursor_x"))
                .on_toggle(Message::ToggleMeasCursorX)
                .into(),
        );

        if self.settings.measurement_cursor_x_enabled {
            items.push(range_slider::horizontal(xlo, xhi, Message::MeasCursorXChanged));
            items.push(text(t!("label.ctrl_fine_drag")).size(9).color(Color::from_rgb(0.6, 0.6, 0.6)).into());
            let t_lo = xlo as f64 * total_time_ms;
            let t_hi = xhi as f64 * total_time_ms;
            let dt_ms = t_hi - t_lo;
            let freq = if dt_ms > 0.0 {
                format_frequency_hz(1000.0 / dt_ms)
            } else {
                "—".to_string()
            };
            items.push(
                column![
                    text(format!("Δt = {}", format_duration_ms(dt_ms))).size(11),
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
                .label(t!("label.cursor_y"))
                .on_toggle(Message::ToggleMeasCursorY)
                .into(),
        );

        if self.settings.measurement_cursor_y_enabled {
            items.push(range_slider::horizontal(ylo, yhi, Message::MeasCursorYChanged));
            // items.push(text(t!("label.ctrl_fine_drag")).size(9).color(Color::from_rgb(0.6, 0.6, 0.6)).into());
            let v_ch1_lo = frac_to_voltage(ylo as f64, v_per_div_ch1, v_offset_ch1);
            let v_ch1_hi = frac_to_voltage(yhi as f64, v_per_div_ch1, v_offset_ch1);
            let v_ch2_lo = frac_to_voltage(ylo as f64, v_per_div_ch2, v_offset_ch2);
            let v_ch2_hi = frac_to_voltage(yhi as f64, v_per_div_ch2, v_offset_ch2);
            let dv_ch1 = (v_ch1_hi - v_ch1_lo).abs();
            let dv_ch2 = (v_ch2_hi - v_ch2_lo).abs();
            items.push(
                column![
                    text(t!("label.ch1_dv", v = format!("{:.3}", dv_ch1))).size(11),
                    text(format!("  ({v_ch1_lo:.2}V … {v_ch1_hi:.2}V)")).size(10),
                    text(t!("label.ch2_dv", v = format!("{:.3}", dv_ch2))).size(11),
                    text(format!("  ({v_ch2_lo:.2}V … {v_ch2_hi:.2}V)")).size(10),
                ]
                .spacing(0)
                .into(),
            );
        }

        items.push(Space::new().height(Length::Fixed(6.0)).into());
        items.push(rule::horizontal(1).into());

        // ── Scale inputs (two-column layout: CH1 | CH2) ──
        let col_w = Length::Fixed(85.0);
        let input_w = Length::Fixed(42.0);
        items.push(
            row![
                text("").size(11).width(col_w),
                text("CH1").size(11).width(input_w),
                text("CH2").size(11).width(input_w),
            ]
            .spacing(2)
            .into(),
        );
        // V/div row
        items.push(
            row![
                text(t!("label.vdiv")).size(11).width(col_w),
                text_input("1.0", &self.v_per_div_ch1_input)
                    .on_input(Message::VPerCellChangedCh1)
                    .size(11)
                    .width(input_w),
                text_input("1.0", &self.v_per_div_ch2_input)
                    .on_input(Message::VPerCellChangedCh2)
                    .size(11)
                    .width(input_w),
            ]
            .spacing(2)
            .align_y(iced::Alignment::Center)
            .into(),
        );
        // Probe row (from device metadata, if available)
        let probe_ch1 = self.current_entry()
            .and_then(|e| e.record.scope_state.as_ref())
            .and_then(|s| s.ch1.as_ref())
            .map(|c| c.probe.clone())
            .unwrap_or_default();
        let probe_ch2 = self.current_entry()
            .and_then(|e| e.record.scope_state.as_ref())
            .and_then(|s| s.ch2.as_ref())
            .map(|c| c.probe.clone())
            .unwrap_or_default();
        if !probe_ch1.is_empty() || !probe_ch2.is_empty() {
            let p1 = if probe_ch1.is_empty() { "—".to_string() } else { probe_ch1 };
            let p2 = if probe_ch2.is_empty() { "—".to_string() } else { probe_ch2 };
            items.push(
                row![
                    text(t!("label.probe")).size(11).width(col_w),
                    text(p1).size(11).width(input_w),
                    text(p2).size(11).width(input_w),
                ]
                .spacing(2)
                .into(),
            );
        }
        // V/off row
        items.push(
            row![
                text(t!("label.voff")).size(11).width(col_w),
                text_input("0.0", &self.v_offset_ch1_input)
                    .on_input(Message::VOffsetChangedCh1)
                    .size(11)
                    .width(input_w),
                text_input("0.0", &self.v_offset_ch2_input)
                    .on_input(Message::VOffsetChangedCh2)
                    .size(11)
                    .width(input_w),
            ]
            .spacing(2)
            .align_y(iced::Alignment::Center)
            .into(),
        );
        // t/div row (shared, spans both channels)
        items.push(
            row![
                text(t!("label.tdiv")).size(11).width(col_w),
                text_input("1.0", &self.t_per_div_input)
                    .on_input(Message::TPerCellChanged)
                    .size(11)
                    .width(Length::Fixed(90.0))
            ]
            .spacing(2)
            .align_y(iced::Alignment::Center)
            .into(),
        );
        items.push(text(t!("label.ac_mode")).size(9).into());
        items.push(rule::horizontal(1).into());

        // ── Signal information (per-channel) ──
        if let Some(cap) = cap {
            let (ch1_mn, ch1_mx, _) = stats(&cap.ch1);
            let v_ch1_neg = v_offset_ch1 + (ADC_VALUE_MID as f64 - ch1_mx as f64) * v_per_div_ch1 / PX_PER_DIV;
            let v_ch1_pos = v_offset_ch1 + (ADC_VALUE_MID as f64 - ch1_mn as f64) * v_per_div_ch1 / PX_PER_DIV;
            items.push(
                text(format!("CH1 range: {v_ch1_neg:.2}V … {v_ch1_pos:.2}V")).size(11).into(),
            );
            if let Some(ch2) = &cap.ch2 {
                let (ch2_mn, ch2_mx, _) = stats(ch2);
                let v_ch2_neg = v_offset_ch2 + (ADC_VALUE_MID as f64 - ch2_mx as f64) * v_per_div_ch2 / PX_PER_DIV;
                let v_ch2_pos = v_offset_ch2 + (ADC_VALUE_MID as f64 - ch2_mn as f64) * v_per_div_ch2 / PX_PER_DIV;
                items.push(
                    text(format!("CH2 range: {v_ch2_neg:.2}V … {v_ch2_pos:.2}V"))
                        .size(11)
                        .into(),
                );
            }
            items.push(
                text(format!("Time range: {total_time_ms:.2} ms ({} samp)", cap.ch1.len()))
                    .size(11)
                    .into(),
            );
            items.push(rule::horizontal(1).into());
        }

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
            items.push(text(t!("label.trigger")).size(11).into());
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
            // Channel probe/coupling info
            if let Some(ref ch1) = scope_state.ch1 {
                items.push(
                    text(format!("CH1: {} probe, {}", ch1.probe, ch1.coupling))
                        .size(10)
                        .into(),
                );
            }
            if let Some(ref ch2) = scope_state.ch2 {
                items.push(
                    text(format!("CH2: {} probe, {}", ch2.probe, ch2.coupling))
                        .size(10)
                        .into(),
                );
            }
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
                items.push(text(t!("label.auto_meas")).size(11).into());

                let fmt_v = |v: f64| -> String {
                    if v.abs() >= 1.0 { format!("{v:.3} V") }
                    else { format!("{:.2} mV", v * 1000.0) }
                };
                let fmt_t = |s: f64| -> String {
                    if s >= 1.0 { format!("{s:.3} s") }
                    else if s >= 0.001 { format!("{:.3} ms", s * 1000.0) }
                    else if s >= 0.000_001 { format!("{:.2} µs", s * 1_000_000.0) }
                    else { format!("{:.2} ns", s * 1_000_000_000.0) }
                };
                let fmt_hz = |hz: f64| -> String {
                    if hz >= 1_000_000.0 { format!("{:.3} MHz", hz / 1_000_000.0) }
                    else if hz >= 1000.0 { format!("{:.3} kHz", hz / 1000.0) }
                    else { format!("{hz:.2} Hz") }
                };

                // Build measurement rows: label | CH1 | CH2
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
                        text(t!("label.ch1")).size(9).width(Length::Fixed(72.0)),
                        text(t!("label.ch2")).size(9).width(Length::Fixed(72.0)),
                    ]
                    .spacing(2)
                    .into(),
                );

                let label_data: &[(u8, &str)] = &[
                    (0, "Freq"), (1, "PkPk"), (2, "Avg"), (3, "RMS"), (4, "Amp"),
                    (5, "+Duty"), (6, "-Duty"), (7, "+T"), (8, "-T"), (9, "T"),
                    (10, "Max"), (11, "Min"), (12, "Top"), (13, "Base"),
                ];
                for &(idx, _key) in label_data {
                    let v1 = ch_val(has_m1, idx as usize);
                    let v2 = ch_val(has_m2, idx as usize);
                    let label = match idx {
                        0 => t!("label.freq_name").to_string(),
                        1 => t!("label.pkpk_name").to_string(),
                        2 => t!("label.avg_name").to_string(),
                        3 => t!("label.rms_name").to_string(),
                        4 => t!("label.amp_name").to_string(),
                        5 => t!("label.duty_plus_name").to_string(),
                        6 => t!("label.duty_minus_name").to_string(),
                        7 => t!("label.t_plus_name").to_string(),
                        8 => t!("label.t_minus_name").to_string(),
                        9 => t!("label.t_name").to_string(),
                        10 => t!("label.max_name").to_string(),
                        11 => t!("label.min_name").to_string(),
                        12 => t!("label.top_name").to_string(),
                        13 => t!("label.base_name").to_string(),
                        _ => unreachable!(),
                    };
                    items.push(
                        row![
                            text(label).size(9).width(Length::Fixed(36.0)),
                            text(v1).size(9).width(Length::Fixed(72.0)),
                            text(v2).size(9).width(Length::Fixed(72.0)),
                        ]
                        .spacing(2)
                        .into(),
                    );
                }
            }
        }

        scrollable(
            container(column(items).spacing(4).padding(6))
                .width(Length::Fixed(240.0))
                .style(container::rounded_box),
        )
        .direction(scrollable::Direction::Vertical(
            scrollable::Scrollbar::new(),
        ))
        .width(Length::Fixed(240.0))
        .into()
    }

    fn controls_panel(&self) -> Element<'_, Message> {
        let has_dev_x = self.capture_has_device_cursor_x();
        let has_dev_y = self.capture_has_device_cursor_y();

        let dev_x = {
            let cb = checkbox(self.settings.device_cursor_x_enabled).label(t!("label.device_x"));
            if has_dev_x {
                cb.on_toggle(Message::ToggleDeviceCursorX)
            } else {
                cb
            }
        };
        let dev_y = {
            let cb = checkbox(self.settings.device_cursor_y_enabled).label(t!("label.device_y"));
            if has_dev_y {
                cb.on_toggle(Message::ToggleDeviceCursorY)
            } else {
                cb
            }
        };

        row![
            checkbox(self.settings.show_ch1)
                .label(t!("label.ch1"))
                .on_toggle(Message::ToggleCh1),
            checkbox(self.settings.show_ch2)
                .label(t!("label.ch2"))
                .on_toggle(Message::ToggleCh2),
            Space::new().width(Length::Fixed(16.0)),
            dev_x,
            dev_y,
            Space::new().width(Length::Fixed(16.0)),
            checkbox(self.settings.show_scales)
                .label(t!("label.scales"))
                .on_toggle(Message::ToggleShowScales),
            Space::new().width(Length::Fixed(16.0)),
            text(t!("label.view")).size(13),
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
            button(text(t!("btn.load_capture")).size(12)).on_press(Message::LoadClicked),
            button(text(t!("btn.save_capture")).size(12)).on_press(Message::SaveCaptureClicked),
            Space::new().width(Length::Fixed(12.0)),
            text("│").size(12),
            Space::new().width(Length::Fixed(12.0)),
            button(text(t!("btn.save_png")).size(12)).on_press(Message::SavePngClicked),
            checkbox(self.settings.split_png)
                .label(t!("label.split_ch"))
                .on_toggle(Message::ToggleSplitPng),
            Space::new().width(Length::Fixed(12.0)),
            text("│").size(12),
            Space::new().width(Length::Fixed(12.0)),
            button(text(t!("btn.export_data")).size(12)).on_press(Message::ExportCsvClicked),
            Space::new().width(Length::Fill),
            button(text(t!("btn.settings")).size(12)).on_press(Message::OpenSettings),
        ]
        .spacing(6)
        .padding(4)
        .align_y(iced::Alignment::Center)
        .into()
    }
}

// ── Helpers ────────────────────────────────────────────────────────────────
fn instructions_panel<'a>() -> Element<'a, Message> {
    // Keep each complete instruction block in a separate string so it can be
    // moved into locale files later without restructuring the widget tree.
    let standard_firmware_instructions = format!(
        "{}\n\n{}\n{}\n{}\n{}\n{}\n{}\n{}",
        t!("label.standard_fw_title"),
        t!("label.standard_fw_1"),
        t!("label.standard_fw_2"),
        t!("label.standard_fw_3"),
        t!("label.standard_fw_4"),
        t!("label.standard_fw_5"),
        t!("label.standard_fw_6"),
        t!("label.standard_fw_7"),
    );

    let customized_firmware_instructions = format!(
        "{}\n\n{}\n{}\n{}\n{}\n{}\n{}\n{}",
        t!("label.customized_fw_title"),
        t!("label.customized_fw_1"),
        t!("label.customized_fw_2"),
        t!("label.customized_fw_3"),
        t!("label.customized_fw_4"),
        t!("label.customized_fw_5"),
        t!("label.customized_fw_6"),
        t!("label.customized_fw_7"),
    );

    let application_instructions = format!(
        "{}\n\n{}\n\n{}\n\n{}",
        t!("label.using_viewer_title"),
        t!("label.using_viewer_1"),
        t!("label.using_viewer_2"),
        t!("label.using_viewer_3"),
    );

    let standard_firmware = container(
        text(standard_firmware_instructions)
            .size(11)
            .width(Length::Fill),
    )
    .padding(6)
    .width(Length::FillPortion(1));

    let customized_firmware = container(
        column![
            button(text(t!("btn.help")).size(11))
                .on_press(Message::OpenUrl(
                    "https://github.com/taligentx/ZeeTweak".to_owned(),
                ))
                .style(button::text),
            text(customized_firmware_instructions)
                .size(11)
                .width(Length::Fill),
        ]
        .spacing(6),
    )
    .padding(6)
    .width(Length::FillPortion(1));

    let application = container(
        text(application_instructions)
            .size(11)
            .width(Length::Fill),
    )
    .padding(6)
    .width(Length::Fill);

    container(
        column![
            row![
                standard_firmware,
                rule::vertical(1),
                customized_firmware,
            ]
            .spacing(4)
            .width(Length::Fill),
            rule::horizontal(1),
            application,
        ]
        .spacing(4)
        .width(Length::Fill),
    )
    .padding(4)
    .width(Length::Fill)
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
    (adc as f32 - ADC_VALUE_MID as f32) * 5.0 / ADC_VALUE_MID as f32
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

/// Format a time duration given in milliseconds into a human-readable string
/// with automatic unit switching (ms / µs / ns).
pub fn format_duration_ms(ms: f64) -> String {
    if ms >= 1.0 {
        format!("{ms:.3} ms")
    } else if ms >= 0.001 {
        format!("{:.3} µs", ms * 1000.0)
    } else {
        format!("{:.3} ns", ms * 1_000_000.0)
    }
}

/// Compact variant for scale labels: ms with 0.1 precision, µs/ns with .0.
pub fn format_duration_ms_scale(ms: f64) -> String {
    if ms >= 1.0 {
        format!("{ms:.1}ms")
    } else if ms >= 0.001 {
        format!("{:.0}µs", ms * 1000.0)
    } else {
        format!("{:.0}ns", ms * 1_000_000.0)
    }
}

/// Format a frequency in Hz into a human-readable string with automatic unit
/// switching (Hz / kHz / MHz).
fn format_frequency_hz(hz: f64) -> String {
    if hz >= 1_000_000.0 {
        format!("{:.3} MHz", hz / 1_000_000.0)
    } else if hz >= 1000.0 {
        format!("{:.3} kHz", hz / 1000.0)
    } else {
        format!("{hz:.2} Hz")
    }
}

/// Grid constants from canvas.rs: 8 divs × 25 px/div = 200 px grid height.
use crate::canvas::{GRID_HEIGHT_PX, PX_PER_DIV};

/// Convert a fractional Y position on the graph (0.0 = top, 1.0 = bottom)
/// to voltage.  Formula from pixel_to_volts: V = V_offset + (ADC_MID - v) × V/div / PX_PER_DIV,
/// where v = frac × GRID_HEIGHT_PX + PX_PER_DIV matches draw_trace's offset
/// (v=25 maps to y=0, the top of the visible grid).
fn frac_to_voltage(frac: f64, v_per_div: f64, v_offset: f64) -> f64 {
    let v = frac * GRID_HEIGHT_PX + PX_PER_DIV;
    v_offset + (ADC_VALUE_MID as f64 - v) * v_per_div / PX_PER_DIV
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
fn thumbnails<'a>(dump: &'a Option<LoadedDump>, current: usize, can_undo: bool) -> Element<'a, Message> {
    let is_empty = dump.as_ref().is_none_or(|d| d.captures.is_empty());
    if is_empty {
        if can_undo {
            return container(
                button(text(t!("btn.undo_delete")).size(12))
                    .on_press(Message::UndoDelete)
                    .style(button::secondary),
            )
            .height(Length::Fixed(62.0))
            .width(Length::Fill)
            .align_x(iced::Alignment::Start)
            .align_y(iced::Alignment::Center)
            .padding(4)
            .into();
        }
        return Space::new().height(Length::Fixed(0.0)).into();
    }
    let dump = dump.as_ref().unwrap();
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
            text(tooltip_ts.clone()).size(11),
            iced::widget::tooltip::Position::Top,
        )
        .into();

        let ctx_idx = i;
        let menu_ts = tooltip_ts.clone();
        iced_aw::ContextMenu::new(underlay, move || {
            container(
                iced::widget::column![
                    text(menu_ts.clone()).size(10),
                    rule::horizontal(1),
                    button(text(t!("btn.export_png")).size(12))
                        .on_press(Message::ContextMenuExportPngAt(ctx_idx))
                        .width(Length::Fill)
                        .style(button::text),
                    button(text(t!("btn.export_csv")).size(12))
                        .on_press(Message::ContextMenuExportCsvAt(ctx_idx))
                        .width(Length::Fill)
                        .style(button::text),
                    rule::horizontal(1),
                    button(text(t!("btn.delete")).size(12))
                        .on_press(Message::ContextMenuDeleteAt(ctx_idx))
                        .width(Length::Fill)
                        .style(button::danger),
                    rule::horizontal(1),
                    {
                        let undo_btn = button(text(t!("btn.undo_delete")).size(12))
                            .width(Length::Fill)
                            .style(button::text);
                        if can_undo {
                            undo_btn.on_press(Message::UndoDelete)
                        } else {
                            undo_btn
                        }
                    },
                ]
                .spacing(1)
                .padding(4)
                .width(Length::Fixed(160.0)),
            )
            .style(container::bordered_box)
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
        .add_filter(t!("label.zwcap_filter").to_string(), &["zwcap", "bin", "dat", "raw"])
        .add_filter(t!("label.all_files").to_string(), &["*"])
        .pick_file()
        .await
        .map(|h| h.path().to_path_buf())
}

async fn pick_save_bin() -> Option<PathBuf> {
    let now = chrono::Local::now();
    let name = format!("capture_{}.zwcap", now.format("%Y%m%d_%H%M%S"));
    rfd::AsyncFileDialog::new()
        .add_filter(t!("label.zwcap_filter").to_string(), &["zwcap"])
        .add_filter(t!("label.all_files").to_string(), &["*"])
        .set_file_name(&name)
        .save_file()
        .await
        .map(|h| h.path().to_path_buf())
}

async fn pick_save_csv(ts: Option<DateTime<Local>>) -> Option<PathBuf> {
    let t = ts.unwrap_or_else(Local::now);
    let name = format!("capture_{}.csv", t.format("%Y%m%d_%H%M%S"));
    rfd::AsyncFileDialog::new()
        .add_filter(t!("label.csv_filter").to_string(), &["csv"])
        .set_file_name(&name)
        .save_file()
        .await
        .map(|h| h.path().to_path_buf())
}

async fn pick_save_png(ts: Option<DateTime<Local>>) -> Option<PathBuf> {
    let t = ts.unwrap_or_else(Local::now);
    let name = format!("capture_{}.png", t.format("%Y%m%d_%H%M%S"));
    rfd::AsyncFileDialog::new()
        .add_filter(t!("label.png_filter").to_string(), &["png"])
        .set_file_name(&name)
        .save_file()
        .await
        .map(|h| h.path().to_path_buf())
}

async fn pick_firmware_file() -> Option<PathBuf> {
    rfd::AsyncFileDialog::new()
        .add_filter(t!("label.fls_filter").to_string(), &["fls", "bin"])
        .add_filter(t!("label.all_files").to_string(), &["*"])
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
    std::thread::spawn(|| {
        let _ = std::process::Command::new("afplay")
            .arg("/System/Library/Sounds/Tink.aiff")
            .output();
    });

    #[cfg(target_os = "linux")]
    std::thread::spawn(|| {
        // Try paplay (PulseAudio/PipeWire) with freedesktop sound theme.
        let ok = std::process::Command::new("paplay")
            .arg("/usr/share/sounds/freedesktop/stereo/bell.oga")
            .status()
            .map(|s| s.success())
            .unwrap_or(false);
        if !ok {
            print!("\x07");
        }
    });

    #[cfg(target_os = "windows")]
    std::thread::spawn(|| {
        // Asterisk is the standard Windows "notification" system sound.
        let _ = std::process::Command::new("powershell")
            .args(["-NoProfile", "-Command", "[System.Media.SystemSounds]::Asterisk.Play()"])
            .output();
    });

    #[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
    print!("\x07");
}

/// Play a "capture complete" beep sound (different from start).
fn play_beep_end() {
    #[cfg(target_os = "macos")]
    std::thread::spawn(|| {
        let _ = std::process::Command::new("afplay")
            .arg("/System/Library/Sounds/Glass.aiff")
            .output();
    });

    #[cfg(target_os = "linux")]
    std::thread::spawn(|| {
        // "complete" is the standard task-done sound in the freedesktop theme.
        let ok = std::process::Command::new("paplay")
            .arg("/usr/share/sounds/freedesktop/stereo/complete.oga")
            .status()
            .map(|s| s.success())
            .unwrap_or(false);
        if !ok {
            print!("\x07");
        }
    });

    #[cfg(target_os = "windows")]
    std::thread::spawn(|| {
        // Exclamation is the standard Windows "action complete" system sound.
        let _ = std::process::Command::new("powershell")
            .args(["-NoProfile", "-Command", "[System.Media.SystemSounds]::Exclamation.Play()"])
            .output();
    });

    #[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
    print!("\x07");
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

// ── PNG export colour palette (mirrors canvas.rs constants) ────────────────
const PNG_BG: [u8; 3] = [10, 15, 26];
const PNG_GRID: [u8; 3] = [51, 61, 77];
const PNG_CH1: [u8; 3] = [255, 217, 25];
const PNG_CH2: [u8; 3] = [242, 25, 242];
const PNG_SCALE_TEXT: [u8; 3] = [153, 166, 179];

/// Configuration for PNG export resolution and rendering quality.
pub struct PngExportConfig {
    /// Target graph area width in pixels. 0 = default small (600 px).
    pub target_width: u32,
}

impl PngExportConfig {
    fn scale(&self) -> f64 {
        const BASE: u32 = T_CELLS as u32 * 50;
        if self.target_width > 0 {
            self.target_width as f64 / BASE as f64
        } else {
            1.0
        }
    }

    fn graph_w(&self) -> u32 {
        const BASE: u32 = T_CELLS as u32 * 50;
        (BASE as f64 * self.scale()).round() as u32
    }

    fn graph_h(&self) -> u32 {
        const BASE: u32 = V_CELLS as u32 * 50;
        (BASE as f64 * self.scale()).round() as u32
    }

    fn scale_margin_left(&self, show: bool) -> u32 {
        if show {
            ((65.0 * self.scale()).round() as u32).max(52)
        } else {
            0
        }
    }

    fn scale_margin_bottom(&self, show: bool) -> u32 {
        if show { (16.0 * self.scale()).round() as u32 } else { 0 }
    }

    fn grid_width(&self) -> f32 {
        (0.75 * self.scale().sqrt() as f32).clamp(0.75, 2.0)
    }

    /// Full line width for the trace in pixels.
    fn trace_width(&self) -> f32 {
        (1.25 * self.scale().sqrt() as f32).clamp(1.0, 4.0)
    }

    fn use_system_font(&self) -> bool {
        self.target_width > 0
    }

    fn font_size(&self) -> f32 {
        (11.0 * self.scale() as f32).max(11.0)
    }
}

/// Export the current capture as a PNG image.
fn export_png(path: &std::path::Path, cap: &Capture, settings: &Settings) -> anyhow::Result<()> {
    let config = PngExportConfig { target_width: settings.png_export_width };

    let graph_w = config.graph_w();
    let graph_h = config.graph_h();
    let scale_margin_left = config.scale_margin_left(settings.show_scales);
    let scale_margin_bottom = config.scale_margin_bottom(settings.show_scales);

    let has_ch2 = cap.ch2.is_some();
    let do_split = settings.split_png && settings.show_ch1 && settings.show_ch2 && has_ch2;

    let width = graph_w + scale_margin_left;
    let height = if do_split {
        (graph_h + scale_margin_bottom) * 2
    } else {
        graph_h + scale_margin_bottom
    };

    // RGBA pixel buffer for tiny-skia.
    let mut pixels = vec![0u8; (width * height * 4) as usize];
    for chunk in pixels.chunks_exact_mut(4) {
        chunk.copy_from_slice(&[PNG_BG[0], PNG_BG[1], PNG_BG[2], 255]);
    }

    // Disable anti-aliasing at very low resolution to keep a crisp look.
    let antialias = graph_w >= 400;
    // Tie system-font choice to antialias: use the aliased bitmap glyph when
    // anti-aliasing is off, so text stays crisp and fits the narrow margins.
    let use_system_font = config.use_system_font() && antialias;
    let font_size = config.font_size();
    let trace_w = config.trace_width();
    // When anti-aliasing is off, sub-pixel strokes may not render; ensure a
    // minimum visible width.
    let grid_w = if antialias { config.grid_width() } else { config.grid_width().max(1.0) };
    let smooth = settings.view_mode == ViewMode::Smooth;
    let dot_mode = settings.view_mode == ViewMode::Dot;
    let dot_radius = (trace_w * 0.6).max(1.0);

    // ── Grid helper via tiny-skia ────────────────────────────────────
    let draw_grid_on = |pixels: &mut [u8], y_off: u32| {
        let Some(grid_path) = build_grid_path(scale_margin_left, y_off, graph_w, graph_h) else {
            return;
        };
        let Some(mut pixmap) = tiny_skia::PixmapMut::from_bytes(pixels, width, height) else {
            return;
        };
        let mut paint = tiny_skia::Paint::default();
        paint.set_color(rgb_color(PNG_GRID));
        paint.anti_alias = antialias;
        let stroke = tiny_skia::Stroke {
            width: grid_w,
            ..tiny_skia::Stroke::default()
        };
        pixmap.stroke_path(&grid_path, &paint, &stroke, tiny_skia::Transform::identity(), None);
    };

    // ── Scale labels (unchanged, uses existing text renderer) ─────────
    let t_per_div_ms = settings.t_per_div_ms;
    let draw_scales_on_region =
        |pixels: &mut [u8], x_off: u32, y_off: u32, w: u32, h: u32, _n_samples: usize,
         vpd_ch1: f64, vo_ch1: f64, ch1_color: Option<[u8; 3]>,
         vpd_ch2: f64, vo_ch2: f64, ch2_color: Option<[u8; 3]>| {
            let cols = T_CELLS as u32;
            let total_time_ms = T_CELLS as f64 * t_per_div_ms;
            for i in 0..cols {
                let frac = i as f64 / cols as f64;
                let time_ms = frac * total_time_ms;
                let label = format_duration_ms_scale(time_ms);
                let x_pos = x_off + i * w / cols + 2;
                let y_pos = y_off + h + 2;
                draw_text_tiny(pixels, width, &label, x_pos, y_pos, PNG_SCALE_TEXT, use_system_font, font_size);
            }

            // Y axis: voltage labels on left margin, per-channel.
            let rows = V_CELLS as u32;
            let mut draw_voltage_label = |vpd: f64, vo: f64, color: [u8; 3], x_lbl: u32| {
                for i in 0..=rows {
                    let v = ((i + 1) as f64 / rows as f64) * GRID_HEIGHT_PX;
                    let voltage = vo + (ADC_VALUE_MID as f64 - v) * vpd / PX_PER_DIV;
                    let y_pos = y_off + (i * h / rows);
                    draw_text_tiny(pixels, width, &format!("{voltage:.1}"), x_lbl, y_pos, color, use_system_font, font_size);
                }
            };

            let x_ch1 = 2u32;
            let x_ch2 = if ch1_color.is_some() { (scale_margin_left as f64 * 0.4).round() as u32 } else { 2u32 };
            if let Some(c) = ch1_color { draw_voltage_label(vpd_ch1, vo_ch1, c, x_ch1); }
            if let Some(c) = ch2_color { draw_voltage_label(vpd_ch2, vo_ch2, c, x_ch2); }
        };

    // ── Draw one subimage region ─────────────────────────────────────
    let draw_one = |pixels: &mut [u8], samples: &[u8], trace_color: [u8; 3],
                    y_off: u32, vpd: f64, vo: f64, ch_col: Option<[u8; 3]>| {
        draw_grid_on(pixels, y_off);
        if !samples.is_empty() {
            if dot_mode {
                draw_dots(pixels, width, height, samples, trace_color,
                          scale_margin_left, y_off, graph_w, graph_h, dot_radius, antialias);
            } else {
                draw_trace(pixels, width, height, samples, trace_color,
                           scale_margin_left, y_off, graph_w, graph_h, trace_w, smooth, antialias);
            }
        }
        if let Some(c) = ch_col {
            draw_scales_on_region(
                pixels, scale_margin_left, y_off, graph_w, graph_h, samples.len(),
                vpd, vo, Some(c), 0.0, 0.0, None,
            );
        }
    };

    if do_split {
        let ch1_col = settings.show_scales.then_some(PNG_CH1);
        draw_one(&mut pixels, &cap.ch1, PNG_CH1, 0,
                 settings.v_per_div_ch1, settings.v_offset_ch1, ch1_col);

        let ch2_col = settings.show_scales.then_some(PNG_CH2);
        if let Some(ch2) = &cap.ch2 {
            draw_one(&mut pixels, ch2, PNG_CH2, graph_h + scale_margin_bottom,
                     settings.v_per_div_ch2, settings.v_offset_ch2, ch2_col);
        }
    } else {
        draw_grid_on(&mut pixels, 0);
        if settings.show_ch1 {
            if dot_mode {
                draw_dots(&mut pixels, width, height, &cap.ch1, PNG_CH1,
                          scale_margin_left, 0, graph_w, graph_h, dot_radius, antialias);
            } else {
                draw_trace(&mut pixels, width, height, &cap.ch1, PNG_CH1,
                           scale_margin_left, 0, graph_w, graph_h, trace_w, smooth, antialias);
            }
        }
        if settings.show_ch2 {
            if let Some(ch2) = &cap.ch2 {
                if dot_mode {
                    draw_dots(&mut pixels, width, height, ch2, PNG_CH2,
                              scale_margin_left, 0, graph_w, graph_h, dot_radius, antialias);
                } else {
                    draw_trace(&mut pixels, width, height, ch2, PNG_CH2,
                               scale_margin_left, 0, graph_w, graph_h, trace_w, smooth, antialias);
                }
            }
        }
        if settings.show_scales {
            let ch1_col = if settings.show_ch1 { Some(PNG_CH1) } else { None };
            let ch2_col = if settings.show_ch2 && has_ch2 { Some(PNG_CH2) } else { None };
            draw_scales_on_region(
                &mut pixels, scale_margin_left, 0, graph_w, graph_h, cap.ch1.len(),
                settings.v_per_div_ch1, settings.v_offset_ch1, ch1_col,
                settings.v_per_div_ch2, settings.v_offset_ch2, ch2_col,
            );
        }
    }

    write_png(path, width, height, &pixels)?;
    Ok(())
}

// ── tiny-skia helper functions ─────────────────────────────────────────────

fn rgb_color(rgb: [u8; 3]) -> tiny_skia::Color {
    tiny_skia::Color::from_rgba8(rgb[0], rgb[1], rgb[2], 255)
}

/// Resample waveform samples into a sequence of (x, y) points for the output
/// graph region.  Upscaling uses linear interpolation; downscaling preserves
/// the min/max envelope per output column.
fn resample_waveform(
    samples: &[u8],
    output_width: u32,
    output_height: u32,
) -> Vec<(f32, f32)> {
    let n = samples.len();
    if n == 0 || output_width == 0 || output_height == 0 {
        return Vec::new();
    }

    let y_scale = (output_height - 1) as f64 / GRID_HEIGHT_PX;

    let map_y = |sample: f64| -> f32 {
        let y = ((sample - PX_PER_DIV) * y_scale)
            .clamp(0.0, (output_height - 1) as f64);
        y as f32
    };

    // Upscaling: linear interpolation between source samples.
    if output_width as usize >= n {
        let mut points = Vec::with_capacity(output_width as usize);
        for x in 0..output_width {
            let source_pos = x as f64 * (n - 1) as f64 / (output_width - 1).max(1) as f64;
            let left = source_pos.floor() as usize;
            let right = (left + 1).min(n - 1);
            let t = source_pos - left as f64;
            let sample = samples[left] as f64 * (1.0 - t) + samples[right] as f64 * t;
            points.push((x as f32, map_y(sample)));
        }
        return points;
    }

    // Downscaling: min/max envelope per column.
    let mut points = Vec::with_capacity(output_width as usize * 2);
    for x in 0..output_width {
        let start = x as usize * n / output_width as usize;
        let end = ((x + 1) as usize * n / output_width as usize)
            .max(start + 1)
            .min(n);
        let col = &samples[start..end];
        let min = *col.iter().min().unwrap() as f64;
        let max = *col.iter().max().unwrap() as f64;
        let y_min = map_y(min);
        let y_max = map_y(max);
        // Preserve temporal direction.
        if col.first() <= col.last() {
            points.push((x as f32, y_min));
            points.push((x as f32, y_max));
        } else {
            points.push((x as f32, y_max));
            points.push((x as f32, y_min));
        }
    }
    points
}

/// Build an anti-aliased grid path via tiny-skia.
fn build_grid_path(
    x_off: u32,
    y_off: u32,
    width: u32,
    height: u32,
) -> Option<tiny_skia::Path> {
    let mut path = tiny_skia::PathBuilder::new();
    for col in 1..T_CELLS as u32 {
        let x = x_off as f32 + col as f32 * width as f32 / T_CELLS as f32;
        path.move_to(x, y_off as f32);
        path.line_to(x, (y_off + height) as f32);
    }
    for row in 1..V_CELLS as u32 {
        let y = y_off as f32 + row as f32 * height as f32 / V_CELLS as f32;
        path.move_to(x_off as f32, y);
        path.line_to((x_off + width) as f32, y);
    }
    path.finish()
}

/// Append a smooth Catmull–Rom curve segment (converted to cubic Bézier)
/// for each consecutive pair of points.
fn append_catmull_rom_path(
    path: &mut tiny_skia::PathBuilder,
    points: &[(f32, f32)],
    x_off: u32,
    y_off: u32,
) {
    let ox = x_off as f32;
    let oy = y_off as f32;
    for i in 0..points.len() - 1 {
        let p0 = if i == 0 { points[i] } else { points[i - 1] };
        let p1 = points[i];
        let p2 = points[i + 1];
        let p3 = if i + 2 < points.len() { points[i + 2] } else { p2 };
        // Catmull-Rom → cubic Bézier with tension 0.5.
        let c1 = (p1.0 + (p2.0 - p0.0) / 6.0, p1.1 + (p2.1 - p0.1) / 6.0);
        let c2 = (p2.0 - (p3.0 - p1.0) / 6.0, p2.1 - (p3.1 - p1.1) / 6.0);
        path.cubic_to(ox + c1.0, oy + c1.1, ox + c2.0, oy + c2.1, ox + p2.0, oy + p2.1);
    }
}

/// Draw an anti-aliased trace via tiny-skia.
#[allow(clippy::too_many_arguments)]
fn draw_trace(
    pixels: &mut [u8],
    image_width: u32,
    image_height: u32,
    samples: &[u8],
    color: [u8; 3],
    x_off: u32,
    y_off: u32,
    width: u32,
    height: u32,
    line_width: f32,
    smooth: bool,
    antialias: bool,
) {
    if samples.len() < 2 || width < 2 || height < 2 {
        return;
    }
    let Some(mut pixmap) = tiny_skia::PixmapMut::from_bytes(pixels, image_width, image_height) else {
        return;
    };
    let points = resample_waveform(samples, width, height);
    if points.len() < 2 {
        return;
    }
    let mut path_builder = tiny_skia::PathBuilder::new();
    let first = points[0];
    path_builder.move_to(x_off as f32 + first.0, y_off as f32 + first.1);
    if smooth && points.len() >= 4 {
        append_catmull_rom_path(&mut path_builder, &points, x_off, y_off);
    } else {
        for &(x, y) in &points[1..] {
            path_builder.line_to(x_off as f32 + x, y_off as f32 + y);
        }
    }
    let Some(path) = path_builder.finish() else { return; };
    let mut paint = tiny_skia::Paint::default();
    paint.set_color(rgb_color(color));
    paint.anti_alias = antialias;
    let stroke = tiny_skia::Stroke {
        width: line_width,
        line_cap: tiny_skia::LineCap::Round,
        line_join: tiny_skia::LineJoin::Round,
        ..tiny_skia::Stroke::default()
    };
    pixmap.stroke_path(&path, &paint, &stroke, tiny_skia::Transform::identity(), None);
}

/// Draw dots via tiny-skia (Dot mode). Each dot is a small filled circle.
#[allow(clippy::too_many_arguments)]
fn draw_dots(
    pixels: &mut [u8],
    image_width: u32,
    image_height: u32,
    samples: &[u8],
    color: [u8; 3],
    x_off: u32,
    y_off: u32,
    width: u32,
    height: u32,
    dot_radius: f32,
    antialias: bool,
) {
    let n = samples.len();
    if n == 0 || width < 2 || height < 2 {
        return;
    }
    let Some(mut pixmap) = tiny_skia::PixmapMut::from_bytes(pixels, image_width, image_height) else {
        return;
    };
    let y_scale = (height - 1) as f64 / GRID_HEIGHT_PX;
    let step = n.div_ceil(width.max(1) as usize).max(1);
    let mut paint = tiny_skia::Paint::default();
    paint.set_color(rgb_color(color));
    paint.anti_alias = antialias;
    for i in (0..n).step_by(step) {
        let x = x_off as f32 + (i as f64 / (n - 1).max(1) as f64 * width as f64) as f32;
        let sample = samples[i] as f64;
        let y = y_off as f32 + ((sample - PX_PER_DIV) * y_scale)
            .clamp(0.0, (height - 1) as f64) as f32;
        let Some(circle) = tiny_skia::PathBuilder::from_circle(x, y, dot_radius) else { continue; };
        pixmap.fill_path(&circle, &paint, tiny_skia::FillRule::Winding,
                         tiny_skia::Transform::identity(), None);
    }
}

/// Draw text onto a raw RGBA pixel buffer.
///
/// Uses the built-in 3×5 bitmap font when `force_system_font` is false,
/// otherwise renders with the system monospace font via `ab_glyph` at the
/// given `font_size`.
#[allow(clippy::too_many_arguments)]
fn draw_text_tiny(
    pixels: &mut [u8],
    img_width: u32,
    text: &str,
    x: u32,
    y: u32,
    color: [u8; 3],
    force_system_font: bool,
    font_size: f32,
) {
    if force_system_font {
        draw_system_font(pixels, img_width, text, x, y, color, font_size);
    } else {
        draw_tiny_glyph(pixels, img_width, text, x, y, color);
    }
}

/// Render text with the built-in 3×5 bitmap font.  Each font pixel maps to
/// exactly one output pixel — no scaling, no anti-aliasing.
fn draw_tiny_glyph(pixels: &mut [u8], img_width: u32, text: &str, x: u32, y: u32, color: [u8; 3]) {
    let mut cx = x;
    for ch in text.chars() {
        let glyph = tiny_glyph(ch);
        for row in 0..5u32 {
            for col in 0..3u32 {
                if glyph[row as usize] & (1 << (2 - col)) != 0 {
                    let px = cx + col;
                    let py = y + row;
                    let idx = ((py * img_width + px) * 4) as usize;
                    if idx + 3 < pixels.len() {
                        pixels[idx..idx + 3].copy_from_slice(&color);
                        pixels[idx + 3] = 255;
                    }
                }
            }
        }
        cx += 4;
    }
}

/// Render text using a system monospace font via `ab_glyph`.
/// Render text into an RGB pixel buffer using the operating system's
/// default monospace font family.
#[allow(clippy::too_many_arguments)]
fn draw_system_font(
    pixels: &mut [u8],
    img_width: u32,
    text: &str,
    x: u32,
    y: u32,
    color: [u8; 3],
    font_size: f32,
) {
    use ab_glyph::{Font, FontArc, PxScale, ScaleFont};
    use font_kit::{
        family_name::FamilyName,
        properties::Properties,
        source::SystemSource,
    };
    use std::sync::OnceLock;

    /// Resolve the default system monospace family and convert it to an
    /// `ab_glyph` font.
    fn load_system_monospace() -> Option<FontArc> {
        let handle = SystemSource::new()
            .select_best_match(
                &[FamilyName::Monospace],
                &Properties::new(),
            )
            .ok()?;

        // Extract font data and index from the handle, then load via ab_glyph.
        let (font_data, font_index) = match &handle {
            font_kit::handle::Handle::Memory { bytes, font_index } => {
                ((**bytes).clone(), *font_index)
            }
            font_kit::handle::Handle::Path { path, font_index } => {
                (std::fs::read(path).ok()?, *font_index)
            }
        };

        let font = ab_glyph::FontVec::try_from_vec_and_index(font_data, font_index).ok()?;
        Some(FontArc::new(font))
    }

    static FONT: OnceLock<Option<FontArc>> = OnceLock::new();

    let Some(font) = FONT.get_or_init(load_system_monospace).as_ref() else {
        return;
    };

    let scaled_font = font.as_scaled(PxScale::from(font_size));
    let baseline = y as f32 + scaled_font.ascent();

    let mut cursor_x = x as f32;

    for ch in text.chars() {
        let glyph_id = font.glyph_id(ch);

        let glyph = glyph_id.with_scale_and_position(
            font_size,
            ab_glyph::point(cursor_x, baseline),
        );

        if let Some(outlined) = font.outline_glyph(glyph) {
            let bounds = outlined.px_bounds();

            outlined.draw(|local_x, local_y, coverage| {
                let pixel_x = bounds.min.x.floor() as i32 + local_x as i32;
                let pixel_y = bounds.min.y.floor() as i32 + local_y as i32;

                if pixel_x < 0 || pixel_y < 0 {
                    return;
                }

                let pixel_x = pixel_x as u32;
                let pixel_y = pixel_y as u32;

                if pixel_x >= img_width {
                    return;
                }

                let index =
                    (pixel_y as usize * img_width as usize + pixel_x as usize) * 4;

                let Some(destination) = pixels.get_mut(index..index + 4) else {
                    return;
                };

                blend_rgb(destination, color, coverage);
            });
        }

        cursor_x += scaled_font.h_advance(glyph_id);
    }
}

/// Alpha-blend one RGB color over an RGBA destination pixel.
fn blend_rgb(destination: &mut [u8], foreground: [u8; 3], alpha: f32) {
    let alpha = alpha.clamp(0.0, 1.0);
    let inverse_alpha = 1.0 - alpha;

    for channel in 0..3 {
        destination[channel] = (
            foreground[channel] as f32 * alpha
                + destination[channel] as f32 * inverse_alpha
        )
            .round()
            .clamp(0.0, 255.0) as u8;
    }
    destination[3] = 255;
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
        'm' => [0b101, 0b111, 0b101, 0b101, 0b101],
        's' => [0b011, 0b100, 0b010, 0b001, 0b110],
        'n' => [0b000, 0b110, 0b101, 0b101, 0b101],
        'µ' => [0b000, 0b101, 0b101, 0b111, 0b100],
        ' ' => [0b000, 0b000, 0b000, 0b000, 0b000],
        _ => [0b000, 0b000, 0b000, 0b000, 0b000],
    }
}

/// Write an RGBA pixel buffer as a PNG file using the `png` crate with
/// maximum compression (no quality loss).
fn write_png(path: &std::path::Path, width: u32, height: u32, rgba: &[u8]) -> anyhow::Result<()> {
    use std::io::BufWriter;

    let file = std::fs::File::create(path)?;
    let w = BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.set_compression(png::Compression::Best);

    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba)?;
    Ok(())
}

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
        // ADC_VALUE_MID → 0V (center), 0 → -5V, ADC_VALUE_MAX → ~+5V
        assert!((adc_to_voltage(ADC_VALUE_MID) - 0.0).abs() < 0.05);
        assert!((adc_to_voltage(0) - (-5.0)).abs() < 0.05);
        assert!((adc_to_voltage(ADC_VALUE_MAX) - 4.96).abs() < 0.1);
    }

    #[test]
    fn samples_to_time_conversion() {
        let ms = samples_to_time_ms(1000);
        assert!((ms - 1.0).abs() < 0.01);
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

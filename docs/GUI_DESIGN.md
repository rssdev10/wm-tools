# DSO3D12 Viewer — GUI Design

This document describes the GUI layout, interaction model and state of the
`dso3d12-gui` application. **Update this document whenever the GUI changes.**

## Window layout

```
┌───────────────────────────────────────────────────────────────┐
│  DSO3D12 │ Port: [▾]  [Start Listening]            STATUS     │  ← menu bar
├──────────────────────────────────────────────┬────────────────┤
│                                              │  Measurement   │
│                                              │  ────────────  │
│              GRAPH CANVAS                    │  CH1 V: …      │
│              (oscilloscope grid +            │  Time range: … │
│               traces + cursors)              │  ────────────  │
│                                              │  ☐ Cursor X    │
│                                              │  [═══●═════●═] │
│                                              │  Δt, freq Hz   │
│                                              │                │
│                                              │  ☐ Cursor Y    │
│                                              │  [═══●═════●═] │
│                                              │  ΔV in volts   │
│                                              │  ────────────  │
│                                              │  CH1/2 stats   │
├──────────────────────────────────────────────┴────────────────┤
│ ☑ CH1  ☑ CH2  │  ☐ Device X  ☐ Device Y  │  View: [Line ▾]   │ ← data controls
├───────────────────────────────────────────────────────────────┤
│  [#1 (60000s)] [#2 (60000s)] [#3] …                           │ ← thumbnails
├───────────────────────────────────────────────────────────────┤
│ [Load Capture] [Save Capture] │ [Save PNG] [Export Data] │    │ ← actions
│                                            [App Settings]     │
├───────────────────────────────────────────────────────────────┤
│ ☑ Show instructions                                           │
│ ┌── How to capture from a ZeeWeii DSO3D12 ──────────────────┐ │
│ └───────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────┘
```

## Panels

### 1. Menu bar (top)
* Application label.
* `Port` `pick_list` of detected serial ports (auto-scanned every 10 s).
* Last-used port shown as text.
* `Listen` / `Stop` toggle button — starts/stops the background serial
  listener thread that auto-detects captures.
* Right-aligned **status text** ("Ready.", "Listening…", "Received N captures",
  "Receiving… 1234 bytes", error messages).
* Version number shown in window title.

### 2. Graph canvas
Custom `iced::widget::canvas` (`canvas.rs`, `Scope`). 10×8 grid, centred axes,
yellow CH1, cyan CH2. Renders:
* Traces in **Line / Dot / Smooth** mode (smooth = moving average).
* Pink interactive cursors (range-shaded) for **measurement cursors**.
  Cursors can also be moved by clicking/dragging directly on the graph
  (nearest cursor line snaps to click position).
* Green thin lines for **device cursors** (data-supplied, read-only) with
  numeric labels (sample numbers for X, voltage for Y).
* Optional **axis scales** toggled via "Show scales" setting. Scale values
  are derived from user-configurable V/div and time/div parameters.

### 3. Measurement panel (right, 220 px)
Vertical column inside a rounded container.

**Scale configuration** (always visible):
* **V/div** text input — volts per division (default 1.0 V). With "V" unit.
* **t/div** text input — time per division in ms (default 1.0 ms). With "ms" unit.
* **V/off** text input — voltage offset at graph center (default 0.0 V). With "V" unit.
* Label: "AC mode: 8×V, 12×t divs" — documents the scope grid geometry.

These values drive all voltage/time calculations throughout the panel and
graph scales. The oscilloscope screen has 8 vertical and 12 horizontal divs.
When a capture arrives via the screenshot protocol, V/div, t/div, and V/offset
are **automatically filled** from the packet metadata.

**Signal information** (shown when a capture is loaded):
* Voltage range for CH1/CH2 (computed from V/div × 8 divs, centered at 0).
* Time range (computed from t/div × 12 divs) + sample count.

**Two range sliders**, one per axis, each a custom widget (`range_slider.rs`)
drawn with `canvas::Program`. Each slider has **two knobs** (low, high); the
user drags whichever knob is nearer.

Below each slider the panel shows cursor measurements:
* X cursors → Δt in ms + frequency in Hz (using real t/div scale).
* Y cursors → ΔV in volts + absolute voltage pair (using real V/div scale).

A divider, then per-channel basic stats (`n`, `min`, `max`, `avg`).

**Trigger info** (shown when capture comes from screenshot format):
* Mode (Auto/Normal/Single), Edge, Source.
* Level in volts.
* Delay + sample rate.

**Auto-measurements table** (shown when screenshot measurements available):
* Two-column table (CH1, CH2).
* Rows: Freq, PkPk, Avg, RMS, Amp, +Duty, -Duty, +T, -T, T, Max, Min, Top, Base.
* Values formatted in physical units (V, Hz, s).

### 4. Data-controls panel (below graph)
* CH1 / CH2 visibility toggles.
* **Device cursor** toggles — **disabled** when no device-cursor data in capture.
* View-mode `pick_list`: `Line`, `Dot`, `Smooth`.

### 5. Thumbnails strip
`scrollable` horizontal row of buttons, one per loaded capture showing sample
count. The active capture uses `primary` style. Clicking loads that capture
into the main graph.

### 6. Action buttons row
`Load Capture` · `Save Capture` │ `Save PNG` · `Export Data` │
`Application Settings`.

### 7. Instructions panel
Toggleable two-column panel. Visibility persisted in settings.
* **Left column** — Standard firmware: how to trigger a raw debug dump
  (long-press Save in measurement view).
* **Right column** — Screenshot firmware: how to transmit a screenshot packet
  (Gallery → select waveform).

### 8. Settings dialog
Full-page replacement view (el15 card-based style). Sections:

**Serial** (bordered card):
* Default serial port (text input, shows available ports).
* Baud rate (text input, default 115200).
* Auto-listen on startup toggle.
* Beep on new capture toggle.

**Display** (bordered card):
* Show axis scales toggle.
* Show instructions panel toggle.

**About** (bordered card):
* App name + version.
* GitHub link.
* Config directory path.
* "Firmware Update…" button opens firmware dialog.

"Save & Close" button writes to JSON and returns to main view.

### 9. Firmware update dialog
Full-page replacement view with:
* Warning banner about bricking risk.
* Port picker (pick_list of available ports).
* File browser button for .bin firmware selection.
* Progress bar during flash.
* Status text with step-by-step updates.
* Start Flash / Close buttons.
* Uses XMODEM-1K protocol (same as CLI flasher) in a background thread.

## Cursor model

| Kind         | Source          | Editable | UI location         |
|--------------|-----------------|----------|---------------------|
| Measurement  | User (sliders)  | Yes      | Right-hand panel    |
| Device       | Capture data    | No       | Data-controls panel |

Never share state between the two.

## Serial port

* Uses `serialport` crate: 115200 baud, 8N1, no flow control.
* Background thread (`serial.rs`) reads continuously.
* **Two capture protocols are auto-detected:**
  1. **ASCII debug dump** — triggered by long-press Save on the device.
     Variable size (~150–500 kB); detected by `--------CH1--------` marker.
     Contains 60 000 raw samples per channel.
  2. **Binary screenshot packet** — triggered by opening a saved waveform in
     the device Gallery. Fixed size 2048 bytes; detected by size (2034–2056 bytes)
     after a 200 ms of serial silence.
     Contains 300 display pixels (min + max ADC per pixel) plus rich settings.
* Capture detection: 500 ms timeout with no new bytes → parse accumulated buffer.
* Auto-reconnect: on disconnect, retries every 2 s until stopped.
* Selected port persisted in settings; app tries to reconnect on startup.

## PNG export

`Save PNG` renders a 1200×600 pixel image of the current capture with grid +
traces, written as a valid PNG file using a minimal built-in encoder (no
external image crate needed). Filename is auto-generated as
`capture_YYYYMMDD_HHMMSS.png` saved to the Downloads directory (no file picker).

## Persisted state (`settings.json`)

```jsonc
{
  "serial_port": "/dev/tty.usbserial-1110",
  "baud_rate": 115200,
  "show_instructions": true,
  "show_ch1": true, "show_ch2": true,
  "measurement_cursor_x_enabled": false,
  "measurement_cursor_y_enabled": false,
  "device_cursor_x_enabled": false,
  "device_cursor_y_enabled": false,
  "view_mode": "Line",
  "cursor_x_range": [0.3, 0.7],
  "cursor_y_range": [0.3, 0.7],
  "show_scales": false,
  "alert_on_data": true,
  "auto_listen": false,
  "v_per_div": 1.0,
  "t_per_div_ms": 1.0,
  "v_offset": 0.0,
  "split_png": false
}
```

## Logging

`logging.rs` — console (stderr) logger only by default. File logging to
`<config_dir>/dso3d12-gui/dso3d12-gui.log` can be enabled by setting
environment variable `DSO_LOG_FILE=1`. Rotates at ~1 MB.
Default level: info; override with `RUST_LOG`.

## Voltage & time assumptions

* ADC 0..255 mapped to ±5 V (0 = −5 V, 128 = 0 V, 255 ≈ +5 V) for raw debug dumps.
* Scale labels are derived from V/div and V/offset settings:
  - Top of graph = V/offset + 4×V/div
  - Bottom of graph = V/offset − 4×V/div
* Time axis = 12 divisions × t/div (ms).
* When receiving screenshot packets, these are auto-filled from device metadata.

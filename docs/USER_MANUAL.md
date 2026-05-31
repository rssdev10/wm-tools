# DSO3D12 Viewer — User Manual

A desktop application to view, measure and export waveform captures from the
ZeeWeii DSO3D12 pocket oscilloscope.

## Installing

```bash
cargo install --path gui
```

Or during development:

```bash
cargo run -p dso3d12-gui
```

## Connecting to the oscilloscope

1. Connect the DSO3D12 via USB (a USB-to-serial adapter appears as e.g.
   `/dev/tty.usbserial-1110` on macOS).
2. In the top menu bar, select the port from the **Port** dropdown.
3. Click **Start Listening**.

The status bar shows "Listening on …". The app will now automatically detect
and display captures when you trigger a dump from the device.

### Triggering a capture on the device

There are two independent capture methods:

#### Method 1 — ASCII debug dump (full buffer)

1. Feed a signal into the scope.
2. Press **Stop** to freeze the sample buffer.
3. Press **Menu**, then **Stop** so the Channel 1 Measurements menu is visible.
4. **Long-press Save** to initiate the debug data transmission.

Transfers ~150–500 kB at 115200 baud. Delivers 60 000 raw ADC samples per
channel. Use `V/div` and `t/div` in the Measurement panel to set the scale.

#### Method 2 — Screenshot binary packet (display pixels)

1. Save a waveform on the device (short-press Save while viewing a signal).
2. Navigate to **Gallery** (Screenshots view) on the device.
3. Select the saved waveform — the device automatically transmits a **2048-byte
   binary packet** containing the 300 display pixels plus all instrument settings.

The app detects this packet by its size and decodes the embedded calibration
data to set `V/div` and `t/div` automatically.

Use the `capture` CLI tool to capture and analyze screenshot packets from the
command line (see [CLI tools](#cli-tools) below).

### Auto-reconnect

If the USB cable is disconnected, the app automatically retries every 2 seconds.
Re-plug and the connection resumes without manual intervention.

## Loading captures from files

Click **Load Capture** and select a `.bin` / `.dat` / `.raw` file previously
saved from the device. Files can contain one or many consecutive captures.

## Viewing the graph

* **Grid**: 10 × 8 divisions (oscilloscope style).
* **CH1** (yellow), **CH2** (cyan) — toggle visibility in the controls panel.
* **View mode** dropdown: Line, Dot, or Smooth (moving-average filter).

## Scale configuration

At the top of the **Measurement** panel you can set the real-world scale:

| Field    | Default | Meaning                                         |
|----------|---------|-------------------------------------------------|
| V/div   | 1.0 V   | Volts per vertical division on the scope screen |
| t/div   | 1.0 ms  | Time per horizontal division                    |
| V/off   | 0.0 V   | Voltage at graph center (offset)                |

The DSO3D12 screen has **8 vertical** and **12 horizontal** divisions (AC mode).
All voltage and time calculations, axis labels, and PNG scales use these values.

When a capture arrives via the **screenshot protocol**, V/div, t/div, and V/offset
are automatically filled from the packet's embedded settings metadata.

## Measurement cursors

In the right-hand **Measurement** panel:

| Toggle           | Slider                     | Readings                          |
|------------------|----------------------------|-----------------------------------|
| ☐ Cursor X (time) | Two knobs on horizontal range | Δt (ms), frequency (Hz)         |
| ☐ Cursor Y (voltage) | Two knobs on vertical range | ΔV (volts), absolute voltages |

**Drag the knob closest to your click** in the slider — or **click/drag
directly on the graph** to move the nearest cursor line. The shaded band
between cursors is drawn on the canvas in pink and updates in real time.

### Signal information

When a capture is loaded the panel also shows:
* Voltage range for each channel (derived from V/div setting).
* Total time range (derived from t/div setting) + sample count.
* Per-channel statistics (sample count, min, max, average ADC).

## Device cursors

If a capture contains device-set cursor data (from the screenshot protocol),
the **Device X** / **Device Y** toggles in the controls panel become enabled.
These cursors are drawn as green lines and **cannot be moved** — they reflect
the device's own cursor state at the moment of capture.

## Trigger & auto-measurements

When a capture comes from the screenshot protocol, the measurement panel shows:
* **Trigger info**: mode (Auto/Normal/Single), edge, source, level, delay,
  sample rate.
* **Auto-measurements table**: two columns (CH1, CH2) with Freq, PkPk, Avg,
  RMS, Amp, ±Duty, ±T, T, Max, Min, Top, Base — all in physical units.

## Thumbnails

When multiple captures are loaded (from a multi-capture file or via serial),
a scrollable strip of buttons appears below the controls panel. Click any
thumbnail to switch the active capture.

## Saving & exporting

| Button          | Action                                                        |
|-----------------|---------------------------------------------------------------|
| Save Capture    | Writes all captures in `.zwcap` container format (CBOR).      |
| Save PNG        | Exports a 1200×600 px PNG image of the graph with grid+traces.|
| Export Data     | Writes a CSV file with columns `index, ch1 [, ch2]`.          |

## Application Settings

Click **Application Settings** to open the settings dialog:
* **Default serial port** — pre-filled on startup.
* **Baud rate** — default 115200 (matches the DSO3D12 debug mode).

Settings are persisted to:
```
~/Library/Application Support/dso3d12-gui/settings.json   (macOS)
~/.config/dso3d12-gui/settings.json                       (Linux)
%APPDATA%\dso3d12-gui\settings.json                       (Windows)
```

## Logs

Activity is logged to stderr (console) by default. File logging to
`<config_dir>/dso3d12-gui/dso3d12-gui.log` can be enabled by setting
`DSO_LOG_FILE=1`. Rotation at ~1 MB. Increase verbosity:

```bash
RUST_LOG=debug dso3d12-gui
# Or with file logging:
DSO_LOG_FILE=1 RUST_LOG=debug dso3d12-gui
```

---

## CLI tools

### `capture` — screenshot packet capture

```bash
# List available serial ports:
capture -l

# Wait for a screenshot packet on the default port and print a verbose report:
capture

# Specify port, save raw binary and timeout:
capture --port /dev/tty.usbserial-1110 --save capture.bin --timeout 60

# Parse as a different device variant:
capture --device dso2512g-v2
```

**Steps:**
1. Connect the DSO3D12 via USB.
2. Run `capture` (it will wait for the device).
3. On the scope: **Gallery → select a saved waveform**.
4. The device transmits a 2048-byte packet; `capture` parses and prints the
   full verbose report including calibrated waveform points and auto-measurements.

If `--save path.bin` is specified the raw packet is written to disk for later
re-analysis or use with custom tooling.

## Troubleshooting

| Symptom                              | Fix                                                     |
|--------------------------------------|---------------------------------------------------------|
| Port not listed                      | Check USB connection; try `ls /dev/tty.*` in terminal.  |
| "Cannot open port" error             | Another process may hold the port; close other apps.    |
| Listening but no capture appears     | Ensure you long-press Save on the device (not short).   |
| Graph empty after load               | File may not be DSO3D12 format; check status bar error. |
| Cursors don't appear                 | Enable them in the Measurement panel checkboxes.        |
| Device cursor toggles greyed out     | Capture has no device-cursor metadata (normal today).   |

## Voltage & time reference

* Voltage range = V/div × 8 divs (centered at 0 → ±4 × V/div).
* Time range = t/div × 12 divs.
* Set V/div and t/div in the Measurement panel to match your scope settings.
* These defaults will be refined when device metadata parsing is extended.

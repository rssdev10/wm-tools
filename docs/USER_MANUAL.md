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

1. Feed a signal into the scope.
2. Press **Stop** to freeze the sample buffer.
3. Press **Menu**, then **Stop** so the Channel 1 Measurements menu is visible.
4. **Long-press Save** to initiate the debug data transmission.

The app detects end-of-transmission after 500 ms of silence and parses the
received data.

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
| V/cell   | 1.0 V   | Volts per vertical division on the scope screen |
| t/cell   | 1.0 ms  | Time per horizontal division                    |

The DSO3D12 screen has **8 vertical** and **12 horizontal** divisions (AC mode).
All voltage and time calculations, axis labels, and PNG scales use these values.

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
* Voltage range for each channel (derived from V/cell setting).
* Total time range (derived from t/cell setting) + sample count.
* Per-channel statistics (sample count, min, max, average ADC).

## Device cursors

If a capture contains device-set cursor data, the **Device X** / **Device Y**
toggles in the controls panel become enabled. These cursors are drawn as green
lines and **cannot be moved** — they reflect the device's own measurement state.

Currently the parser does not extract device-cursor metadata, so these toggles
remain disabled.

## Thumbnails

When multiple captures are loaded (from a multi-capture file or via serial),
a scrollable strip of buttons appears below the controls panel. Click any
thumbnail to switch the active capture.

## Saving & exporting

| Button          | Action                                                        |
|-----------------|---------------------------------------------------------------|
| Save Capture    | Writes the raw dump to a `.bin` file.                         |
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

Activity is logged to `<config_dir>/dso3d12-gui/dso3d12-gui.log` and stderr.
Rotation at ~1 MB. Increase verbosity:

```bash
RUST_LOG=debug dso3d12-gui
```

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

* Voltage range = V/cell × 8 cells (centered at 0 → ±4 × V/cell).
* Time range = t/cell × 12 cells.
* Set V/cell and t/cell in the Measurement panel to match your scope settings.
* These defaults will be refined when device metadata parsing is extended.

# wm-tools — Flash Utility for WinnerMicro MCUs

A native Rust implementation of a flash utility for WinnerMicro MCUs (W800/W801 family), including the DSO3D12 oscilloscope.

Based on the reference Python tool from the WinnerMicro IoT SDK:
https://github.com/winnermicro/wm_iot_sdk/tree/master

## Features

### CLI flasher (`flash`)

- Auto-detects the highest supported baud rate (up to 2 Mbps)
- XMODEM-1K protocol with CRC-16 verification
- Progress bar with ETA during flashing
- Lists available serial ports with USB product info
- Optional flash erase before writing
- DTR/RTS-based automatic device reset

### GUI viewer (`dso3d12-gui`)

A graphical application that allows you to visualize and save dumps and screenshots from the ZeeWeii DSO3D12 oscilloscope.

Once connected, the application automatically retrieves debug dumps and snapshot view data from the device, provided these features have been activated via the device buttons. These graphs can be saved as PNG files or exported as CSV data.

To use the snapshot view feature, you must install the modified firmware. For more details, visit [ZeeTweak](https://github.com/taligentx/ZeeTweak). The debug dump export feature is available by default.

## Requirements

- Rust toolchain (stable, 2021 edition or later)
- A USB-to-serial adapter connected to the target device (or integrated one)

## How to build

```bash
cargo build --release
```

The binaries are produced at:

| Binary | Path |
|--------|------|
| CLI flasher | `target/release/flash` |
| CLI capture | `target/release/capture` |
| GUI viewer  | `target/release/dso3d12-gui` |

### Run the GUI from source

```bash
cargo run --bin dso3d12-gui
```

## How to use

### List available serial ports

```bash
./target/release/flash -l
```

### Flash a firmware image

Auto-selects the port and negotiates the highest available baud rate:

```bash
./target/release/flash -i dso3d12_v3.0.6_III.fls
```

Specify the port explicitly:

```bash
./target/release/flash -p /dev/tty.usbserial-1110 -i dso3d12_v3.0.6_III.fls
```

Flash multiple images in one pass:

```bash
./target/release/flash -p /dev/tty.usbserial-1110 \
    -i bootloader.fls -i app.fls
```

### Erase flash only (no image)

```bash
./target/release/flash -p /dev/tty.usbserial-1110 -e
```

### Erase then flash

```bash
./target/release/flash -p /dev/tty.usbserial-1110 -e -i firmware.fls
```

### Lock in a specific baud rate

Supported rates: `2000000`, `1000000`, `921600`, `460800`, `115200`.

```bash
./target/release/flash -p /dev/tty.usbserial-1110 -b 460800 -i firmware.fls
```

## Switching to Boot Mode (DSO3D12)

1. **Connect** the supplied USB cable to your PC while the oscilloscope is **powered off**.
2. **Press and hold** the power button. The scope may enter a power-cycling loop — keep the button held down **continuously** until flashing reaches 100%.
3. The tool resets the device automatically via DTR/RTS. Use `-m` / `--manual-reset` to suppress the automatic reboot after flashing.

## Command-line reference

```
Flash tool for Winner Micro MCU

Usage: flash [OPTIONS]

Options:
  -p, --port <PORT>          serial port
  -b, --baudrate <BAUDRATE>  serial baudrate
  -i, --image <IMAGE>        image file paths (repeatable)
  -n, --name <NAME>          firmware names to burn. Ex: app,bootloader,partition_table,custom...
  -e, --erase                erase device flash
  -l, --list                 list serial ports
  -m, --manual-reset         skip automatic reboot after burning
  -h, --help                 Print help
  -V, --version              Print version

Repository: https://github.com/rssdev10/wm-tools
```

## Capture a screenshot packet (`capture`)

Listens for the 2048-byte screenshot packet the oscilloscope sends when you open
*Gallery* and select a saved waveform, then prints the parsed settings and samples:

```bash
./target/release/capture -p /dev/tty.usbserial-1110 -s screen.bin
./target/release/capture --load screen.bin -d dso3d12     # re-parse a saved packet
```

Options: `-p/--port`, `-l/--list`, `-d/--device` (`dso3d12`, `dso2512g-v2`, `dso2512g-v1`),
`-s/--save`, `--load`, `-t/--timeout` (seconds, `0` waits forever).

## Development

### Workspace layout

Cargo workspace (`resolver = "2"`). Shared dependencies are in `[workspace.dependencies]`.

| Package | Path | Purpose |
|---------|------|---------|
| `wm-tools` | `flasher/` | CLI binaries `flash` (XMODEM-1K flasher for W800/W801) and `capture` (screenshot packet listener). Has its own version. |
| `dso-parser` | `dso-parser/` | Library: parses the ASCII debug dump and the binary screenshot packet into typed structs; `.zwcap` container. |
| `dso3d12-gui` | `gui/` | Desktop viewer and firmware updater (iced 0.14). |

### GUI modules (`gui/src/`)

| File | Responsibility |
|------|----------------|
| `main.rs` | Boots `iced::application(App::new, App::update, App::view)` with `.subscription()`. |
| `app.rs` | `App` state, messages, view composition, subscriptions; headless unit tests. |
| `canvas.rs` | `Scope` canvas program: grid, traces, measurement and device cursors. |
| `range_slider.rs` | Custom two-knob range slider widget (`canvas::Program`). |
| `serial.rs` | Background serial listener: port list, auto-reconnect, capture detection. |
| `flash.rs` | Background firmware update over XMODEM-1K. |
| `export_image.rs` | PNG export (tiny-skia, antialiased text). |
| `settings.rs` | JSON settings in `<config dir>/dso3d12-gui/settings.json`. |
| `logging.rs` | Logger: stderr by default, file on request. |
| `i18n.rs` | `rust-i18n` setup and system-locale detection. |

### Threads and the serial port

- The serial listener (`serial::start_listening`) and the firmware update
  (`flash::start_flash`) each run on their own `std::thread`. They report to the GUI
  through a `tokio::sync::mpsc` unbounded channel, which `App::subscription` polls.
- Each thread is stopped through an `Arc<AtomicBool>` flag in its handle
  (`SerialHandle`, `FlashHandle`); dropping a handle sets the flag and joins the thread.
- The listener treats 500 ms without new bytes as the end of a capture, reads with a 100 ms
  timeout so it notices the stop flag, and retries a lost port every 2 s.
- A serial port has one owner at a time: the GUI stops the listener before it starts a
  firmware update.
- Opening a port can pulse DTR/RTS, which the DSO3D12 uses as reset lines. Where the platform
  allows, the listener opens the port natively and clears both lines at once.

### Cursor model

The GUI has two independent kinds of cursors; never share state, sliders or toggles between
them:

1. **Measurement cursors**: set by the user with the right-hand two-knob range sliders, one
   per axis. Drawn pink.
2. **Device cursors**: read-only, taken from the capture's metadata and toggled from the
   data-controls panel. The toggles are disabled when the capture has no device-cursor data.
   Drawn green.

### iced 0.14 notes

- `iced::application(boot, update, view).title(...).theme(...).run()`: `boot` takes no arguments.
- `Space::new().width(...).height(...)`, not `with_width`.
- `checkbox(bool).label(s).on_toggle(fn)`; leave out `on_toggle` to render it disabled.
- `iced::widget::rule::horizontal(px)` and `rule::vertical(px)`.
- `canvas::Text { align_x, align_y, .. }`, not `horizontal_alignment`.
- `canvas::Program::update(&self, &mut State, &Event, Rectangle, mouse::Cursor) -> Option<canvas::Action<Message>>`.
  Emit messages with `canvas::Action::publish(msg).and_capture()` and request redraws with
  `Action::request_redraw()`.
- `canvas::Event` is `iced_core::event::Event` (mouse, keyboard, touch, window).

### Device facts

- Capture baud rate: 115200. The flasher negotiates up to 2 Mbps (see `-b`).
- Acquisition buffer: 60 000 samples per channel.
- Device variants with different packet fields: `dso3d12`, `dso2512g-v2`, `dso2512g-v1`.
- The screenshot packet needs the ZeeTweak firmware; the debug dump works on stock firmware.

### Documentation map

| Document | Contents |
|----------|----------|
| [docs/GUI_DESIGN.md](docs/GUI_DESIGN.md) | GUI layout, panels, cursor model, serial behaviour, PNG export, settings, logging. |
| [docs/USER_MANUAL.md](docs/USER_MANUAL.md) | End-user guide for the viewer. |
| [docs/SCREENSHOT_PROTOCOL.md](docs/SCREENSHOT_PROTOCOL.md) | Screenshot packet layout, ADC-to-voltage conversion, settings offsets, variant differences. |
| [packaging/README.md](packaging/README.md) | Files used to assemble the release packages. |

### Testing and linting

```bash
cargo test --workspace                                  # unit tests
cargo clippy --workspace --all-targets -- -D warnings   # lint (must be warning-free)
```

Tests live in the crate they test and need no serial device.

### Localisation

The GUI's strings are in `gui/locales/` (`en`, `ru`, `zh`, `es`, `hi`). Every key exists in
all five files.

### Diagnostics

- `RUST_LOG` sets the GUI's stderr log level.
- `DSO_LOG_FILE=1` also writes `<config dir>/dso3d12-gui/dso3d12-gui.log` (rotated at 1 MB).

### Continuous integration and releases

- `.github/workflows/tests.yml` builds, tests and runs clippy on Linux, Windows and macOS for
  every pull request and every push to `main`.
- `.github/workflows/release.yml` runs when a GitHub release is created. It builds `wm-tools`
  and `dso3d12-gui` for each platform and attaches the packages assembled from `packaging/`.

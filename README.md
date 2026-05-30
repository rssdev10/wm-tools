# wm-tools — Flash Utility for WinnerMicro MCUs

A native Rust implementation of a flash utility for WinnerMicro MCUs (W800/W801 family), including the DSO3D12 oscilloscope.

Based on the reference Python tool from the WinnerMicro IoT SDK:
https://github.com/winnermicro/wm_iot_sdk/tree/master

## Features

- Auto-detects the highest supported baud rate (up to 2 Mbps)
- XMODEM-1K protocol with CRC-16 verification
- Progress bar with ETA during flashing
- Lists available serial ports with USB product info
- Optional flash erase before writing
- DTR/RTS-based automatic device reset

## Requirements

- Rust toolchain (stable, 2021 edition or later)
- A USB-to-serial adapter connected to the target device (or integrated one)

## How to build

```bash
cargo build --release
```

The binary is produced at `target/release/flash`.

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

## Running tests

```bash
cargo test --workspace
```

## Linting

```bash
cargo clippy --workspace
```

//! Serial capture tool for ZeeWeii DSO3D12 screenshot packets.
//!
//! # Usage
//!     capture [--port /dev/ttyXXX] [--device dso3d12|dso2512g-v2|dso2512g-v1]
//!             [--save path.bin] [--timeout 30]
//!     capture --load path.bin [--device dso3d12]
//!
//! On the oscilloscope: open *Gallery* → select a saved waveform.
//! The device transmits a 2048-byte binary screenshot packet automatically.

use anyhow::{Context, Result};
use clap::Parser;
use dso_parser::{
    is_screenshot_packet, parse_screenshot_as, DeviceVariant,
};
use serialport::SerialPortType;
use std::{
    io::{Read, Write},
    path::PathBuf,
    time::{Duration, Instant},
};

const DEFAULT_PORT: &str = "/dev/tty.usbserial-1110";
const BAUD: u32 = 115200;

#[derive(Parser, Debug)]
#[clap(
    name = "capture",
    about = "Listen for a ZeeWeii DSO3D12 screenshot packet over serial",
    after_help = "On the oscilloscope: open Gallery ▸ select a saved waveform to trigger upload."
)]
struct Args {
    /// Serial port to open (use -l to list available ports).
    #[clap(short, long, default_value = DEFAULT_PORT)]
    port: String,

    /// List available serial ports and exit.
    #[clap(short, long)]
    list: bool,

    /// Device firmware variant: dso3d12 | dso2512g-v2 | dso2512g-v1.
    #[clap(short, long, default_value = "dso3d12")]
    device: String,

    /// Save the raw binary packet to this file.
    #[clap(short, long)]
    save: Option<PathBuf>,

    /// Load and re-parse a previously saved raw binary file (skips serial capture).
    #[clap(long)]
    load: Option<PathBuf>,

    /// Receive timeout in seconds (0 = wait forever).
    #[clap(short, long, default_value = "60")]
    timeout: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();

    if args.list {
        list_ports();
        return Ok(());
    }

    let device = parse_device_variant(&args.device)?;

    // Re-parse mode: load from file instead of serial.
    let buf = if let Some(ref path) = args.load {
        println!("Loading {} …", path.display());
        std::fs::read(path)
            .with_context(|| format!("Cannot read '{}'", path.display()))?
    } else {
        println!("Opening {} at {} baud …", args.port, BAUD);
        let mut port = serialport::new(&args.port, BAUD)
            .timeout(Duration::from_millis(200))
            .open()
            .with_context(|| format!("Cannot open serial port '{}'", args.port))?;

        println!("Waiting for screenshot packet.");
        println!();
        println!("  ► On the oscilloscope: open Gallery ▸ select a saved waveform.");
        println!("  ► The device will transmit a 2048-byte binary packet automatically.");
        println!();
        if args.timeout > 0 {
            println!("  (Timeout: {} s)", args.timeout);
        } else {
            println!("  (Waiting indefinitely — Ctrl-C to abort)");
        }
        println!();

        let buf = receive_packet(&mut *port, args.timeout)?;

        println!("  Received {} bytes.", buf.len());

        if let Some(ref path) = args.save {
            std::fs::write(path, &buf)
                .with_context(|| format!("Cannot save to {}", path.display()))?;
            println!("  Saved raw packet → {}", path.display());
        }

        buf
    };

    println!();
    println!("{}", "═".repeat(66));
    println!("  Parsing as {} …", device);
    println!("{}", "═".repeat(66));

    match parse_screenshot_as(&buf, device) {
        Ok(pkt) => {
            println!("{}", pkt.report_verbose());

            // Additional graph-usage notes
            println!("{}", "═".repeat(66));
            println!("  GRAPH USAGE SUMMARY");
            println!("{}", "═".repeat(66));
            let s = &pkt.settings;
            let points = pkt.to_physical_waveform();
            println!("  Samples         : {} (300 pixels = 12 div × 25 px/div)", points.len());
            println!(
                "  X axis          : 0 … {} (each step = {:.3e} s)",
                dso_parser::format_ps(
                    (s.time_per_pixel_s() * 299.0 * 1e12) as i64
                ),
                s.time_per_pixel_s()
            );
            let half_range_v = s.ch1.volt_scale_uv as f64 * 4.0 / 1_000_000.0
                * 10f64.powi(s.ch1.probe_mode as i32);
            println!(
                "  Y axis (CH1)    : ≈ {:.4}V … {:.4}V  ({}/div)",
                -half_range_v,
                half_range_v,
                s.ch1.vdiv_str()
            );
            if s.ch2.enabled {
                let half_range_v2 = s.ch2.volt_scale_uv as f64 * 4.0 / 1_000_000.0
                    * 10f64.powi(s.ch2.probe_mode as i32);
                println!(
                    "  Y axis (CH2)    : ≈ {:.4}V … {:.4}V  ({}/div)",
                    -half_range_v2,
                    half_range_v2,
                    s.ch2.vdiv_str()
                );
            }

            println!();
            println!("  Data structure for plotting:");
            println!("    Each PixelPoint {{ time_s, ch1_min_v, ch1_max_v, ch2: Option<(min_v, max_v)> }}");
            println!("    Fill-between ch1_min_v and ch1_max_v for waveform envelope.");
            println!("    Average (min+max)/2 per pixel for a clean single trace.");
            println!();
            println!("  First 8 calibrated points:");
            for (i, p) in points.iter().take(8).enumerate() {
                if let Some((c2min, c2max)) = p.ch2 {
                    println!(
                        "    [{:3}] t={:.3e}s  CH1 {:.4}…{:.4}V  CH2 {:.4}…{:.4}V",
                        i, p.time_s, p.ch1_min_v, p.ch1_max_v, c2min, c2max
                    );
                } else {
                    println!(
                        "    [{:3}] t={:.3e}s  CH1 {:.4}…{:.4}V",
                        i, p.time_s, p.ch1_min_v, p.ch1_max_v
                    );
                }
            }
        }
        Err(e) => {
            eprintln!("Parse error: {e}");
            eprintln!("Raw hex dump (first 64 bytes):");
            for (i, b) in buf.iter().take(64).enumerate() {
                if i % 16 == 0 { eprint!("\n  {:04X}  ", i); }
                eprint!("{:02X} ", b);
            }
            eprintln!();
            std::process::exit(1);
        }
    }

    Ok(())
}

/// Read bytes from the serial port until we have a complete screenshot packet
/// or the timeout expires. Returns the raw buffer.
fn receive_packet(port: &mut dyn Read, timeout_secs: u64) -> Result<Vec<u8>> {
    let mut buf: Vec<u8> = Vec::with_capacity(2048);
    let mut read_buf = [0u8; 512];
    let deadline = if timeout_secs > 0 {
        Some(Instant::now() + Duration::from_secs(timeout_secs))
    } else {
        None
    };
    let mut last_recv = Instant::now();

    loop {
        // Timeout check
        if let Some(d) = deadline {
            if Instant::now() > d {
                anyhow::bail!("Timeout: no packet received within {} s", timeout_secs);
            }
        }

        match port.read(&mut read_buf) {
            Ok(0) => {}
            Ok(n) => {
                buf.extend_from_slice(&read_buf[..n]);
                last_recv = Instant::now();

                // Flush progress indicator
                print!("\r  Buffered {} bytes … ", buf.len());
                let _ = std::io::stdout().flush();

                // Once we have at least 2034 bytes and 200 ms have elapsed
                // with no new data, consider the packet complete.
                // ZeeTweak accepts 2034-2056 bytes.
                if buf.len() >= 2034
                    && last_recv.elapsed() >= Duration::from_millis(200)
                {
                    println!();
                    break;
                }

                // Hard upper bound — if we received more than 2200 bytes,
                // something went wrong (debug dump mixed in?).
                if buf.len() > 2200 {
                    println!();
                    eprintln!("Warning: buffer overflow ({} bytes); truncating to 2048", buf.len());
                    buf.truncate(2048);
                    break;
                }
            }
            Err(ref e)
                if e.kind() == std::io::ErrorKind::TimedOut
                    || e.kind() == std::io::ErrorKind::WouldBlock =>
            {
                // 200 ms read timeout — if we have enough bytes and no recent
                // activity, declare the packet complete.
                if buf.len() >= 2034
                    && last_recv.elapsed() >= Duration::from_millis(200)
                {
                    println!();
                    break;
                }
            }
            Err(e) => {
                return Err(e).context("Serial read error");
            }
        }
    }

    if !is_screenshot_packet(&buf) {
        anyhow::bail!(
            "Received {} bytes — does not match screenshot packet size (2034–2056).\n\
             This may be a debug dump or noise. Re-trigger on the device.",
            buf.len()
        );
    }

    Ok(buf)
}

fn parse_device_variant(s: &str) -> Result<DeviceVariant> {
    match s.to_lowercase().as_str() {
        "dso3d12" => Ok(DeviceVariant::Dso3d12),
        "dso2512g-v2" | "dso2512gv2" | "v2" => Ok(DeviceVariant::Dso2512gV2),
        "dso2512g-v1" | "dso2512gv1" | "v1" => Ok(DeviceVariant::Dso2512gV1),
        other => anyhow::bail!(
            "Unknown device variant '{}'. Use: dso3d12, dso2512g-v2, dso2512g-v1",
            other
        ),
    }
}

fn list_ports() {
    println!("Available serial ports:");
    match serialport::available_ports() {
        Ok(ports) => {
            if ports.is_empty() {
                println!("  (none found)");
                return;
            }
            for p in &ports {
                let info = match &p.port_type {
                    SerialPortType::UsbPort(i) => {
                        let mfr = i.manufacturer.as_deref().unwrap_or("?");
                        let prod = i.product.as_deref().unwrap_or("?");
                        format!("  USB  {mfr} / {prod}")
                    }
                    SerialPortType::BluetoothPort => "  Bluetooth".to_string(),
                    SerialPortType::PciPort => "  PCI".to_string(),
                    SerialPortType::Unknown => "  (unknown)".to_string(),
                };
                println!("  {}  {}", p.port_name, info);
            }
        }
        Err(e) => println!("  Error listing ports: {e}"),
    }
}

//! Serial port communication for live capture from the DSO3D12.
//!
//! Runs a background thread that reads from the configured serial port using
//! the `serialport` crate. Received bytes are buffered; when no data arrives
//! for 500 ms we treat the buffered bytes as a complete capture and send it
//! to the GUI via a channel.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use dso_parser::{parse_captures, is_screenshot_packet, Capture};
use tokio::sync::mpsc;

/// Configuration for serial capture.
#[derive(Debug, Clone)]
pub struct SerialConfig {
    pub port: String,
    pub baud_rate: u32,
}

impl Default for SerialConfig {
    fn default() -> Self {
        Self {
            port: String::new(),
            baud_rate: 115200,
        }
    }
}

/// Handle to a running serial listener. Drop or call `stop()` to end it.
pub struct SerialHandle {
    stop_flag: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl SerialHandle {
    pub fn stop(&mut self) {
        self.stop_flag.store(true, Ordering::Relaxed);
        if let Some(h) = self.thread.take() {
            let _ = h.join();
        }
    }
}

impl Drop for SerialHandle {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Events emitted by the serial listener.
#[derive(Debug, Clone)]
pub enum SerialEvent {
    /// A complete capture was received and parsed (ASCII debug dump).
    CaptureReceived(Vec<Capture>),
    /// A binary screenshot packet was received (2048 bytes).
    ScreenshotReceived(Vec<u8>),
    /// Bytes received so far (progress indicator: total bytes buffered).
    Progress(usize),
    /// An error occurred (port disconnected, parse failure, etc.).
    Error(String),
    /// Port opened successfully.
    Connected,
    /// Port was disconnected; will attempt to reconnect.
    Disconnected,
}

/// List available serial ports on the system, filtered to relevant ones.
/// On macOS, only `tty.usbserial*` ports are shown (matching the flasher tool).
pub fn list_ports() -> Vec<String> {
    let ports = serialport::available_ports().unwrap_or_default();
    ports
        .into_iter()
        .map(|p| p.port_name)
        .filter(|name| {
            if cfg!(target_os = "macos") {
                // On macOS, only show USB-serial adapters (tty.usbserial-*)
                name.contains("tty.usbserial")
            } else {
                true
            }
        })
        .collect()
}

/// Spawn a background thread that reads from the given serial port.
///
/// Capture detection: when 500 ms passes with no new bytes, the accumulated
/// buffer is treated as a complete dump and parsed. Successfully parsed
/// captures are sent via the returned channel.
///
/// If the port disconnects, the thread will retry every 2 seconds until
/// stopped.
pub fn start_listening(
    config: SerialConfig,
    tx: mpsc::UnboundedSender<SerialEvent>,
) -> SerialHandle {
    let stop_flag = Arc::new(AtomicBool::new(false));
    let flag = stop_flag.clone();

    let thread = std::thread::spawn(move || {
        listener_loop(&config, &tx, &flag);
    });

    SerialHandle {
        stop_flag,
        thread: Some(thread),
    }
}

const CAPTURE_TIMEOUT: Duration = Duration::from_millis(500);
const RECONNECT_DELAY: Duration = Duration::from_secs(2);
const READ_TIMEOUT: Duration = Duration::from_millis(100);

fn listener_loop(
    config: &SerialConfig,
    tx: &mpsc::UnboundedSender<SerialEvent>,
    stop: &AtomicBool,
) {
    while !stop.load(Ordering::Relaxed) {
        match open_port(config) {
            Ok(mut port) => {
                let _ = tx.send(SerialEvent::Connected);
                read_loop(&mut *port, tx, stop);
                let _ = tx.send(SerialEvent::Disconnected);
            }
            Err(e) => {
                log::warn!("cannot open {}: {e}", config.port);
                let _ = tx.send(SerialEvent::Error(format!(
                    "Cannot open {}: {e}",
                    config.port
                )));
            }
        }
        // Wait before reconnecting.
        let deadline = Instant::now() + RECONNECT_DELAY;
        while Instant::now() < deadline && !stop.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}

fn open_port(config: &SerialConfig) -> Result<Box<dyn serialport::SerialPort>, serialport::Error> {
    serialport::new(&config.port, config.baud_rate)
        .data_bits(serialport::DataBits::Eight)
        .stop_bits(serialport::StopBits::One)
        .parity(serialport::Parity::None)
        .flow_control(serialport::FlowControl::None)
        .timeout(READ_TIMEOUT)
        .open()
}

fn read_loop(
    port: &mut dyn serialport::SerialPort,
    tx: &mpsc::UnboundedSender<SerialEvent>,
    stop: &AtomicBool,
) {
    let mut buf = [0u8; 4096];
    let mut accum: Vec<u8> = Vec::with_capacity(256_000);
    let mut last_data = Instant::now();

    while !stop.load(Ordering::Relaxed) {
        match port.read(&mut buf) {
            Ok(n) if n > 0 => {
                accum.extend_from_slice(&buf[..n]);
                last_data = Instant::now();
                let _ = tx.send(SerialEvent::Progress(accum.len()));
            }
            Ok(_) => {}
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                // Normal — no data right now.
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::BrokenPipe
                || e.kind() == std::io::ErrorKind::PermissionDenied =>
            {
                log::warn!("serial port disconnected: {e}");
                return; // Will trigger reconnect.
            }
            Err(e) => {
                log::error!("serial read error: {e}");
                return;
            }
        }

        // Check if we have buffered data and the timeout has elapsed.
        if !accum.is_empty() && last_data.elapsed() >= CAPTURE_TIMEOUT {
            log::info!("capture timeout; {} bytes buffered, parsing…", accum.len());
            // Detect format: screenshot binary (2034–2056 bytes) vs ASCII debug dump
            if is_screenshot_packet(&accum) {
                log::info!("detected screenshot packet ({} bytes)", accum.len());
                let _ = tx.send(SerialEvent::ScreenshotReceived(accum.clone()));
            } else {
                match parse_captures(&accum) {
                    Ok(captures) if !captures.is_empty() => {
                        let _ = tx.send(SerialEvent::CaptureReceived(captures));
                    }
                    Ok(_) => {
                        let _ = tx.send(SerialEvent::Error(
                            "Received data but no valid captures found.".to_string(),
                        ));
                    }
                    Err(e) => {
                        let _ = tx.send(SerialEvent::Error(format!("Parse error: {e}")));
                    }
                }
            }
            accum.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn list_ports_does_not_panic() {
        // May return empty on CI, but must not crash.
        let _ = list_ports();
    }

    #[test]
    fn serial_config_default() {
        let cfg = SerialConfig::default();
        assert_eq!(cfg.baud_rate, 115200);
        assert!(cfg.port.is_empty());
    }
}

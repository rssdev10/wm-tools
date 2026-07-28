//! Firmware update module for the GUI.
//!
//! Reuses the XMODEM-1K protocol logic from `flasher/src/flash.rs`.
//! Runs the flash process in a background thread, reporting progress
//! via a channel.

use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crc::{Crc, CRC_16_XMODEM};
use tokio::sync::mpsc;

const DEFAULT_BAUDRATE: u32 = 115200;
const SYNC_THRESHOLD: usize = 5;

const BAUDRATE_LIST: &[(u32, &[u8])] = &[
    (
        2000000,
        b"\x21\x0a\x00\xef\x2a\x31\x00\x00\x00\x80\x84\x1e\x00",
    ),
    (
        1000000,
        b"\x21\x0a\x00\x5e\x3d\x31\x00\x00\x00\x40\x42\x0f\x00",
    ),
    (
        921600,
        b"\x21\x0a\x00\x5d\x50\x31\x00\x00\x00\x00\x10\x0e\x00",
    ),
    (
        460800,
        b"\x21\x0a\x00\x07\x00\x31\x00\x00\x00\x00\x08\x07\x00",
    ),
    (
        115200,
        b"\x21\x0a\x00\x97\x4b\x31\x00\x00\x00\x00\xc2\x01\x00",
    ),
];

const REBOOT_CMD: &[u8] = b"\x21\x06\x00\xc7\x7c\x3f\x00\x00\x00";

/// Progress events from the flash thread.
#[derive(Debug, Clone)]
pub enum FlashEvent {
    Status(String),
    Progress { current: u64, total: u64 },
    Finished(Result<(), String>),
}

/// Handle to a running flash operation.
pub struct FlashHandle {
    stop_flag: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl FlashHandle {
    pub fn cancel(&mut self) {
        self.stop_flag.store(true, Ordering::Relaxed);
    }
}

impl Drop for FlashHandle {
    fn drop(&mut self) {
        self.stop_flag.store(true, Ordering::Relaxed);
        if let Some(h) = self.thread.take() {
            let _ = h.join();
        }
    }
}

/// Start the firmware update process in a background thread.
pub fn start_flash(
    port_name: String,
    firmware_path: String,
    tx: mpsc::UnboundedSender<FlashEvent>,
) -> FlashHandle {
    let stop = Arc::new(AtomicBool::new(false));
    let flag = stop.clone();

    let thread = std::thread::spawn(move || {
        let result = run_flash(&port_name, &firmware_path, &tx, &flag);
        let _ = tx.send(FlashEvent::Finished(result.map_err(|e| e.to_string())));
    });

    FlashHandle {
        stop_flag: stop,
        thread: Some(thread),
    }
}

fn run_flash(
    port_name: &str,
    firmware_path: &str,
    tx: &mpsc::UnboundedSender<FlashEvent>,
    stop: &AtomicBool,
) -> Result<(), anyhow::Error> {
    let _ = tx.send(FlashEvent::Status(format!("Opening {port_name}…")));

    let mut port = serialport::new(port_name, DEFAULT_BAUDRATE)
        .timeout(Duration::from_secs(1))
        .open()
        .map_err(|e| anyhow::anyhow!("Cannot open port: {e}"))?;

    if stop.load(Ordering::Relaxed) {
        return Err(anyhow::anyhow!("Cancelled"));
    }

    // Reset device
    let _ = tx.send(FlashEvent::Status("Resetting device…".to_string()));
    for _ in 0..10 {
        reset_device(&mut *port)?;
        send_esc(&mut *port, 20)?;
        port.write_all(b"AT+RST\r\n")?;
        send_esc(&mut *port, 20)?;
        port.write_all(b"reboot\r\n")?;
        port.flush()?;
        send_esc(&mut *port, 50)?;
    }

    if stop.load(Ordering::Relaxed) {
        return Err(anyhow::anyhow!("Cancelled"));
    }

    // Sync
    let _ = tx.send(FlashEvent::Status("Syncing with device…".to_string()));
    sync_device(&mut *port)?;

    if stop.load(Ordering::Relaxed) {
        return Err(anyhow::anyhow!("Cancelled"));
    }

    // Negotiate baud rate
    let _ = tx.send(FlashEvent::Status("Negotiating baud rate…".to_string()));
    for &(baud, cmd) in BAUDRATE_LIST {
        if configure_baudrate(&mut *port, baud, cmd).is_ok() {
            let _ = tx.send(FlashEvent::Status(format!("Using {baud} baud.")));
            break;
        }
    }

    if stop.load(Ordering::Relaxed) {
        return Err(anyhow::anyhow!("Cancelled"));
    }

    // Read firmware file
    let file_data = std::fs::read(firmware_path)
        .map_err(|e| anyhow::anyhow!("Cannot read firmware: {e}"))?;
    let total_blocks = (file_data.len() as u64 / 1024) + 1;

    let _ = tx.send(FlashEvent::Status("Flashing…".to_string()));

    // XMODEM-1K send
    xmodem1k_send(&mut *port, &file_data, total_blocks, tx, stop)?;

    // Reboot
    let _ = tx.send(FlashEvent::Status("Rebooting device…".to_string()));
    port.write_all(REBOOT_CMD)?;
    port.flush()?;

    let _ = tx.send(FlashEvent::Status("Flash complete!".to_string()));
    Ok(())
}

fn reset_device(port: &mut dyn serialport::SerialPort) -> Result<(), anyhow::Error> {
    port.write_data_terminal_ready(false)?;
    port.write_request_to_send(true)?;
    std::thread::sleep(Duration::from_millis(10));
    port.write_data_terminal_ready(true)?;
    port.write_request_to_send(false)?;
    std::thread::sleep(Duration::from_millis(10));
    port.write_data_terminal_ready(false)?;
    Ok(())
}

fn send_esc(port: &mut dyn serialport::SerialPort, ms: u64) -> Result<(), anyhow::Error> {
    let delay = Duration::from_millis(10);
    let iterations = ms / 10;
    for _ in 0..iterations {
        port.write_all(&[0x1B])?;
        std::thread::sleep(delay);
    }
    Ok(())
}

fn sync_device(port: &mut dyn serialport::SerialPort) -> Result<(), anyhow::Error> {
    let mut cnt = 0;
    let start = Instant::now();
    let mut buf = [0; 1];

    loop {
        match port.read(&mut buf) {
            Ok(1) => {
                if buf[0] == b'C' {
                    cnt += 1;
                } else {
                    cnt = 0;
                }
                if cnt >= SYNC_THRESHOLD {
                    return Ok(());
                }
            }
            _ => {
                port.write_all(&[0x1B])?;
                std::thread::sleep(Duration::from_millis(30));
            }
        }
        if start.elapsed() > Duration::from_secs(60) {
            return Err(anyhow::anyhow!("Sync timeout — ensure device is in boot mode"));
        }
    }
}

fn configure_baudrate(
    port: &mut dyn serialport::SerialPort,
    baud: u32,
    cmd: &[u8],
) -> Result<(), anyhow::Error> {
    port.set_baud_rate(DEFAULT_BAUDRATE)?;
    if baud != DEFAULT_BAUDRATE {
        port.set_baud_rate(DEFAULT_BAUDRATE)?;
        std::thread::sleep(Duration::from_millis(20));
    }
    port.write_all(cmd)?;
    port.flush()?;
    std::thread::sleep(Duration::from_millis(20));

    // Drain
    let mut drain = [0u8; 64];
    while port.bytes_to_read()? > 0 {
        let _ = port.read(&mut drain);
    }

    port.set_baud_rate(baud)?;
    std::thread::sleep(Duration::from_millis(200));
    Ok(())
}

fn xmodem1k_send(
    port: &mut dyn serialport::SerialPort,
    data: &[u8],
    total_blocks: u64,
    tx: &mpsc::UnboundedSender<FlashEvent>,
    stop: &AtomicBool,
) -> Result<(), anyhow::Error> {
    let crc = Crc::<u16>::new(&CRC_16_XMODEM);
    let mut block_num = 1u8;
    let mut offset = 0;

    while offset < data.len() {
        if stop.load(Ordering::Relaxed) {
            return Err(anyhow::anyhow!("Cancelled"));
        }

        let end = (offset + 1024).min(data.len());
        let mut buffer = [0u8; 1024];
        let chunk_len = end - offset;
        buffer[..chunk_len].copy_from_slice(&data[offset..end]);
        if chunk_len < 1024 {
            buffer[chunk_len..].fill(0x1A);
        }

        let checksum = crc.checksum(&buffer);
        let mut success = false;

        for _ in 0..10 {
            port.write_all(&[0x02, block_num, 255 - block_num])?;
            port.write_all(&buffer)?;
            port.write_all(&checksum.to_be_bytes())?;
            port.flush()?;

            let mut ack = [0; 1];
            let start = Instant::now();
            while start.elapsed() < Duration::from_secs(3) {
                if let Ok(1) = port.read(&mut ack) {
                    if ack[0] == 0x06 {
                        success = true;
                        break;
                    } else if ack[0] == 0x15 {
                        break;
                    }
                }
            }
            if success {
                break;
            }
        }

        if !success {
            return Err(anyhow::anyhow!("Failed to send block {block_num}"));
        }

        let block_idx = (offset / 1024 + 1) as u64;
        let _ = tx.send(FlashEvent::Progress {
            current: block_idx,
            total: total_blocks,
        });
        block_num = block_num.wrapping_add(1);
        offset += 1024;
    }

    // Send EOT
    port.write_all(&[0x04])?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn baudrate_list_has_entries() {
        assert!(!BAUDRATE_LIST.is_empty());
    }

    #[test]
    fn baudrate_commands_are_13_bytes() {
        for &(_, cmd) in BAUDRATE_LIST {
            assert_eq!(cmd.len(), 13);
        }
    }

    #[test]
    fn reboot_cmd_is_9_bytes() {
        assert_eq!(REBOOT_CMD.len(), 9);
    }
}

//
// Flash tool for Winner Micro MCU
// based on https://github.com/winnermicro/wm_iot_sdk/blob/master/tools/wm/flash.py
//

use anyhow::{anyhow, Context, Result};
use clap::Parser;
use colored::*;
use crc::{Crc, CRC_16_XMODEM};

use indicatif::{ProgressBar, ProgressStyle};
use serialport::{SerialPort, SerialPortType};
use tabled::settings::Style;
use tabled::{Table, Tabled};

use std::{
    fs::File,
    io::{self, Read, Write},
    time::{Duration, Instant},
};

const DEFAULT_TIMEOUT: f64 = 1.0; // in seconds
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

const ERASE_CMD: &[u8] = b"\x21\x0a\x00\xc3\x35\x32\x00\x00\x00\x02\x00\xfe\x01";
const REBOOT_CMD: &[u8] = b"\x21\x06\x00\xc7\x7c\x3f\x00\x00\x00";

#[derive(Parser, Debug)]
#[clap(
    name = "flash",
    about = "Flash tool for Winner Micro MCU",
    version,
    long_about = None,
    after_help = "Repository: https://github.com/rssdev10/wm-tools"
)]
struct Args {
    #[clap(short = 'p', long = "port", help = "serial port")]
    port: Option<String>,

    #[clap(short = 'b', long = "baudrate", help = "serial baudrate")]
    baudrate: Option<u32>,

    #[clap(
        short = 'i',
        long = "image",
        multiple_occurrences(true),
        help = "image file paths"
    )]
    image: Vec<String>,

    #[clap(
        short = 'n',
        long = "name",
        multiple_occurrences(true),
        help = "firmware names to burn. Ex: app,bootloader,partition_table,custom..."
    )]
    name: Vec<String>,

    #[clap(short = 'e', long = "erase", help = "erase device flash")]
    erase: bool,

    #[clap(short = 'l', long = "list", help = "list serial ports")]
    list: bool,

    #[clap(
        short = 'm',
        long = "manual-reset",
        help = "manual reset after burning"
    )]
    manual_reset: bool,
}

struct FlashDownloader {
    port: Box<dyn SerialPort>,
}

impl FlashDownloader {
    fn new(port_name: &str, baud_rate: u32) -> Result<Self> {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_secs_f64(DEFAULT_TIMEOUT))
            .open()
            .with_context(|| format!("Failed to open port {}", port_name))?;
        Ok(Self { port })
    }

    fn set_port_baudrate(&mut self, baud_rate: u32) -> Result<()> {
        self.port
            .set_baud_rate(baud_rate)
            .map_err(|e| anyhow!("Failed to set baudrate: {}", e))
    }

    fn reset_device(&mut self) -> Result<()> {
        self.port.write_data_terminal_ready(false)?;
        self.port.write_request_to_send(true)?;
        std::thread::sleep(Duration::from_millis(10));
        self.port.write_data_terminal_ready(true)?;
        self.port.write_request_to_send(false)?;
        std::thread::sleep(Duration::from_millis(10));
        self.port.write_data_terminal_ready(false)?;

        Ok(())
    }

    fn send_esc(&mut self, ms: u64) -> Result<()> {
        let delay = Duration::from_millis(10);
        let iterations = ms / 10;
        for _ in 0..iterations {
            self.port.write(&[0x1B])?;
            std::thread::sleep(delay);
        }

        Ok(())
    }

    fn empty_read(&mut self) -> Result<()> {
        let mut buf: [u8; 1] = [0; 1];
        while self.port.bytes_to_read()? > 0 {
            self.port.read(&mut buf)?;
        }

        Ok(())
    }

    fn configure_protocol_baudrate(&mut self, baud_rate: u32, cmd: &[u8]) -> Result<()> {
        self.set_port_baudrate(baud_rate)?;

        if baud_rate != DEFAULT_BAUDRATE {
            self.set_port_baudrate(DEFAULT_BAUDRATE)?;
            std::thread::sleep(Duration::from_millis(20));
        }
        if let Err(e) = self.port.write_all(cmd) {
            return Err(anyhow!(
                "Failed to send command for baudrate {}: {}",
                baud_rate,
                e
            ));
        }
        self.port.flush()?;
        std::thread::sleep(Duration::from_millis(20));

        // println!("{} bytes to read", self.port.bytes_to_read()?);
        self.empty_read()?;

        if let Err(e) = self.set_port_baudrate(baud_rate) {
            return Err(anyhow!("Failed to reapply baudrate {}: {}", baud_rate, e));
        }
        std::thread::sleep(Duration::from_millis(200));

        Ok(())
    }

    fn perform_full_reset(&mut self) -> Result<()> {
        self.reset_device()?;
        self.send_esc(20)?;
        self.port.write_all(b"AT+RST\r\n")?;
        self.send_esc(20)?;
        self.port.write_all(b"reboot\r\n")?;
        self.port.flush()?;
        self.send_esc(50)?;

        Ok(())
    }

    fn sync_device(&mut self) -> Result<()> {
        let mut cnt = 0;
        let start = Instant::now();
        let mut buf = [0; 1];
        let mut note = true;

        loop {
            match self.port.read(&mut buf) {
                Ok(1) => {
                    if buf[0] == b'C' {
                        cnt += 1;
                    } else {
                        cnt = 0;
                    }
                    if cnt >= SYNC_THRESHOLD {
                        break;
                    }
                }
                _ => {
                    self.port.write(&[0x1B])?;
                    std::thread::sleep(Duration::from_millis(30));
                }
            }

            if note && start.elapsed() >= Duration::from_secs(2) {
                println!("{}", "Please reset device".cyan());
                note = false;
            }

            if start.elapsed() > Duration::from_secs(60) {
                return Err(anyhow!("Serial sync timeout"));
            }
        }
        Ok(())
    }

    fn erase_flash(&mut self) -> Result<()> {
        println!("Erasing device flash...");

        self.port.write_all(ERASE_CMD)?;
        self.port.flush()?;

        let mut cnt = 0;
        let start = Instant::now();
        let mut buf = [0; 1];

        loop {
            match self.port.read(&mut buf) {
                Ok(1) => {
                    if buf[0] == b'C' {
                        cnt += 1;
                    } else {
                        return Err(anyhow!("Erase failed: {}", buf[0] as char));
                    }
                    if cnt >= SYNC_THRESHOLD {
                        break;
                    }
                }
                _ => {
                    if start.elapsed() > Duration::from_secs(60) {
                        return Err(anyhow!("Erase timeout"));
                    }
                }
            }
        }
        println!("Erase complete");
        Ok(())
    }

    fn xmodem1k_send(&mut self, file_path: &str, progress: &ProgressBar) -> Result<()> {
        let mut file = File::open(file_path)?;
        let crc = Crc::<u16>::new(&CRC_16_XMODEM);
        let mut block_num = 1u8;
        let mut buffer = [0u8; 1024];

        loop {
            let bytes_read = file.read(&mut buffer)?;
            if bytes_read == 0 {
                break;
            }

            if bytes_read < 1024 {
                buffer[bytes_read..].fill(0x1A);
            }

            let checksum = crc.checksum(&buffer);
            let mut retries = 0;
            let mut success = false;

            while retries < 10 && !success {
                self.port.write(&[0x02, block_num, 255 - block_num])?;
                self.port.write_all(&buffer)?;
                self.port.write_all(&checksum.to_be_bytes())?;
                self.port.flush()?;

                let mut ack = [0; 1];
                let start = Instant::now();
                while start.elapsed() < Duration::from_secs(3) {
                    if self.port.read(&mut ack)? == 1 {
                        if ack[0] == 0x06 {
                            success = true;
                            break;
                        } else if ack[0] == 0x15 {
                            break;
                        }
                    }
                }

                retries += 1;
            }

            if !success {
                return Err(anyhow!("Failed to send block {}", block_num));
            }

            progress.inc(1);
            block_num = block_num.wrapping_add(1);
        }

        // Send EOT
        self.port.write(&[0x04])?;
        Ok(())
    }

    fn reboot_device(&mut self) -> Result<()> {
        if let Err(e) = self.port.write_all(REBOOT_CMD) {
            return Err(anyhow!("Failed to send reboot command: {}", e));
        }
        Ok(())
    }
}

fn list_serial_ports() -> io::Result<()> {
    println!("Available serial ports:");
    match serialport::available_ports() {
        Ok(ports) => {
            // Apply platform-specific filtering
            let filtered_ports = ports
                .into_iter()
                .filter(|port| {
                    #[cfg(not(windows))]
                    {
                        port.port_name.contains("/tty.")
                            || port.port_name.contains("/ttyUSB")
                            || port.port_name.contains("/ttyACM")
                    }
                    #[cfg(windows)]
                    {
                        true
                    }
                })
                .collect::<Vec<_>>();

            if filtered_ports.is_empty() {
                println!("  No ports found");
                return Ok(());
            }

            #[derive(Tabled)]
            struct Row {
                #[tabled(rename = "Port")]
                port: String,
                #[tabled(rename = "Type")]
                type_str: String,
                #[tabled(rename = "Info")]
                info: String,
            }

            let mut rows = Vec::new();
            for port in &filtered_ports {
                match &port.port_type {
                    SerialPortType::UsbPort(info) => {
                        let product = info
                            .product
                            .clone()
                            .unwrap_or_else(|| "Unknown".to_string());
                        rows.push(Row {
                            port: port.port_name.clone(),
                            type_str: "USB".to_string(),
                            info: format!("{} ({:04x}:{:04x})", product, info.vid, info.pid),
                        });
                    }
                    SerialPortType::BluetoothPort => {
                        rows.push(Row {
                            port: port.port_name.clone(),
                            type_str: "Bluetooth".to_string(),
                            info: "".to_string(),
                        });
                    }
                    SerialPortType::PciPort => {
                        rows.push(Row {
                            port: port.port_name.clone(),
                            type_str: "PCI".to_string(),
                            info: "".to_string(),
                        });
                    }
                    SerialPortType::Unknown => {
                        rows.push(Row {
                            port: port.port_name.clone(),
                            type_str: "Unknown".to_string(),
                            info: "".to_string(),
                        });
                    }
                }
            }

            let mut table = Table::new(rows);
            table.with(Style::psql());

            println!("{}", table);
            Ok(())
        }
        Err(e) => Err(io::Error::new(
            io::ErrorKind::Other,
            format!("Error listing ports: {}", e),
        )),
    }
}

fn main() -> Result<()> {
    let args = Args::parse();

    if args.list {
        list_serial_ports()?;
        return Ok(());
    }

    let port_name = match args.port {
        Some(p) => p,
        None => {
            let ports = serialport::available_ports()?;
            ports
                .first()
                .ok_or(anyhow!("No serial ports available"))?
                .port_name
                .clone()
        }
    };

    println!("Connecting to {}...", port_name);
    let mut download = FlashDownloader::new(&port_name, DEFAULT_BAUDRATE)?;

    println!("Resetting device...");
    for _ in 0..10 {
        download.perform_full_reset()?;
    }

    println!("Syncing...");
    download.sync_device()?;
    println!("{}", "Sync success".green());

    if args.erase {
        download.erase_flash()?;
        if args.image.is_empty() {
            return Ok(());
        }
    }

    if let Some(baud) = args.baudrate {
        if let Some((_, cmd)) = BAUDRATE_LIST.iter().find(|&&(b, _)| b == baud) {
            download.configure_protocol_baudrate(baud, cmd)?;
        } else {
            return Err(anyhow!("Baudrate {} not supported", baud));
        }
    } else {
        for &(b, cmd) in BAUDRATE_LIST {
            println!("Trying baudrate {}...", b);
            if download.configure_protocol_baudrate(b, cmd).is_ok() {
                println!(
                    "{}",
                    format!("Baudrate {} selected successfully.", b).green()
                );
                break;
            }
        }
    }

    println!("Starting download...");
    for image in &args.image {
        let style = ProgressStyle::default_bar()
            .template("{spinner} [{elapsed}] {bar:40} {pos}/{len} ({eta})")?
            .progress_chars("##-");
        let file_size = std::fs::metadata(image)?.len();
        let pb = ProgressBar::new((file_size / 1024 + 1) as u64).with_style(style);

        println!("Downloading {}...", image);

        download.xmodem1k_send(image, &pb)?;
        pb.finish();
    }

    if !args.manual_reset {
        download.reboot_device()?;
    }

    println!("{}", "Flash complete".green());
    Ok(())
}

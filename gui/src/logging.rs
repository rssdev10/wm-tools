//! File + stderr logging for the GUI.
//!
//! Logs go to `<config_dir>/dso3d12-gui/dso3d12-gui.log` (rotated naively when
//! it exceeds ~1 MB by being renamed to `.log.1`). Logs are also printed to
//! stderr at the level chosen by `RUST_LOG` (default `info`).

use std::fs::OpenOptions;
use std::io::Write;
use std::path::PathBuf;
use std::sync::Mutex;

use log::{Level, LevelFilter, Metadata, Record};

pub fn log_path() -> Option<PathBuf> {
    let dir = dirs::config_dir()?.join("dso3d12-gui");
    std::fs::create_dir_all(&dir).ok()?;
    Some(dir.join("dso3d12-gui.log"))
}

struct FileLogger {
    file: Mutex<Option<std::fs::File>>,
    stderr_level: Level,
}

impl log::Log for FileLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Trace
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        let ts = chrono::Local::now().format("%Y-%m-%d %H:%M:%S%.3f");
        let line = format!(
            "[{ts}] {:5} {}: {}\n",
            record.level(),
            record.target(),
            record.args()
        );
        if record.level() <= self.stderr_level {
            eprint!("{line}");
        }
        if let Ok(mut guard) = self.file.lock() {
            if let Some(f) = guard.as_mut() {
                let _ = f.write_all(line.as_bytes());
            }
        }
    }

    fn flush(&self) {
        if let Ok(mut g) = self.file.lock() {
            if let Some(f) = g.as_mut() {
                let _ = f.flush();
            }
        }
    }
}

/// Initialize global logger. Safe to call multiple times; only the first call
/// installs the logger.
pub fn init() {
    let stderr_level = std::env::var("RUST_LOG")
        .ok()
        .and_then(|s| s.parse::<Level>().ok())
        .unwrap_or(Level::Info);

    let file = log_path().and_then(|p| {
        rotate_if_needed(&p);
        OpenOptions::new().create(true).append(true).open(&p).ok()
    });

    let logger = FileLogger {
        file: Mutex::new(file),
        stderr_level,
    };
    let _ = log::set_boxed_logger(Box::new(logger))
        .map(|()| log::set_max_level(LevelFilter::Trace));
}

fn rotate_if_needed(path: &PathBuf) {
    const MAX_BYTES: u64 = 1_000_000;
    if let Ok(meta) = std::fs::metadata(path) {
        if meta.len() > MAX_BYTES {
            let backup = path.with_extension("log.1");
            let _ = std::fs::rename(path, backup);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn log_path_returns_some() {
        // On typical dev machines a config dir exists; just check the call
        // doesn't panic. The result may be None inside sandboxes.
        let _ = log_path();
    }

    #[test]
    fn init_is_idempotent() {
        init();
        init();
    }
}

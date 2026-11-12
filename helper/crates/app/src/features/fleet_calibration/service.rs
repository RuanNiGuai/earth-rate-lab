use crate::update::Progress;
use serde_json::Value;
use std::{
    fs::{self, File},
    io::{Read, Seek, SeekFrom},
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    sync::atomic::{AtomicBool, Ordering},
    thread,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

#[derive(Clone)]
pub(crate) struct Request {
    pub(crate) toolkit: PathBuf,
    pub(crate) python: PathBuf,
    pub(crate) payload: Value,
    pub(crate) hardware: bool,
}

pub(crate) struct Completion {
    pub(crate) result: Value,
    pub(crate) log: PathBuf,
}

// A child must never outlive its owning Job, including on read/status errors.
// Hardware cleanup is cooperative: killing an installer would skip its rollback.
struct OwnedChild {
    child: Child,
    cancel_path: PathBuf,
    hardware: bool,
}
impl Drop for OwnedChild {
    fn drop(&mut self) {
        if !matches!(self.child.try_wait(), Ok(Some(_))) {
            let _ = fs::write(&self.cancel_path, b"stop");
            if !self.hardware {
                let _ = self.child.kill();
            }
            let _ = self.child.wait();
        }
    }
}

fn bounded_text(path: &Path, limit: u64, tail: bool) -> Result<String, String> {
    let mut file = File::open(path).map_err(|e| e.to_string())?;
    let length = file.metadata().map_err(|e| e.to_string())?.len();
    if length > limit {
        if !tail {
            return Err("Toolkit result exceeds size limit".into());
        }
        file.seek(SeekFrom::End(-(limit as i64)))
            .map_err(|e| e.to_string())?;
    }
    let mut bytes = Vec::new();
    file.take(limit)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    Ok(String::from_utf8_lossy(&bytes).into_owned())
}

pub(crate) fn run(
    mut request: Request,
    cancel: &AtomicBool,
    progress: &mut dyn FnMut(Progress),
) -> Result<Completion, String> {
    if cancel.load(Ordering::Acquire) {
        return Err("Cancelled before starting".into());
    }
    let state_root = directories::ProjectDirs::from("org", "SlimeNRF", "SlimeNRF Helper")
        .ok_or("Cannot locate the calibration data folder")?
        .data_local_dir()
        .join("calibration");
    fs::create_dir_all(&state_root).map_err(|e| e.to_string())?;
    if request.payload["action"] == "workflow-resume" {
        let saved: Value = serde_json::from_str(
            &bounded_text(&state_root.join("last-session.json"), 8192, false).map_err(|_| {
                "没有可继续的会话。首次使用请点击准备；已有会话可在高级工具中选择目录。".to_owned()
            })?,
        )
        .map_err(|e| e.to_string())?;
        request.payload["session"] = saved["session"].clone();
        request.payload["action"] = Value::String("workflow-open".into());
    }
    let bundled = request.toolkit.as_os_str().is_empty();
    let toolkit = if bundled {
        std::env::current_exe()
            .map_err(|e| e.to_string())?
            .parent()
            .ok_or("Missing Helper directory")?
            .join("calibration")
    } else {
        request.toolkit.canonicalize().map_err(|e| e.to_string())?
    };
    let bridge = toolkit.join("tools/helper_fleet.py");
    let backend = toolkit.join(if cfg!(windows) {
        "earth-rate-calibration.exe"
    } else {
        "earth-rate-calibration"
    });
    if (bundled && !backend.is_file()) || (!bundled && !bridge.is_file()) {
        return Err("缺少配套校准组件。请解压完整 Helper 安装包，保留程序旁的 calibration 文件夹；开发者可在高级设置选择工具包。".into());
    }
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|e| e.to_string())?
        .as_nanos();
    let directory = state_root
        .join("jobs")
        .join(format!("{}-{nonce}", std::process::id()));
    fs::create_dir_all(&directory).map_err(|e| e.to_string())?;
    let input = directory.join("request.json");
    fs::write(
        &input,
        serde_json::to_vec_pretty(&request.payload).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    let log_path = directory.join("output.log");
    let log = File::create(&log_path).map_err(|e| e.to_string())?;
    let mut command = Command::new(if bundled { &backend } else { &request.python });
    if !bundled {
        command.arg("-u").arg(bridge);
    }
    command
        .arg("--request")
        .arg(&input)
        .current_dir(&toolkit)
        .env("PYTHONUTF8", "1")
        .env("EARTH_RATE_STATE_DIR", &state_root)
        .stdin(Stdio::null())
        .stdout(log.try_clone().map_err(|e| e.to_string())?)
        .stderr(log);
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt as _;
        command.creation_flags(0x0800_0000); // CREATE_NO_WINDOW
    }
    let child = command
        .spawn()
        .map_err(|e| format!("Cannot start the calibration component: {e}"))?;
    let mut owned = OwnedChild {
        child,
        cancel_path: directory.join("CANCEL"),
        hardware: request.hardware,
    };
    let mut cancelled = false;
    let mut previous = String::new();
    let status = loop {
        if cancel.load(Ordering::Acquire) && !cancelled {
            fs::write(&owned.cancel_path, b"stop").map_err(|e| e.to_string())?;
            if !request.hardware {
                owned.child.kill().map_err(|e| e.to_string())?;
            }
            cancelled = true;
        }
        if let Ok(text) = bounded_text(&directory.join("progress.json"), 64 * 1024, false)
            && text != previous
        {
            progress(Progress::stage(text.clone()));
            previous = text;
        }
        if let Some(status) = owned.child.try_wait().map_err(|e| e.to_string())? {
            break status;
        }
        thread::sleep(Duration::from_millis(150));
    };
    if !status.success() {
        let tail = bounded_text(&log_path, 8192, true).unwrap_or_default();
        return Err(format!(
            "Toolkit stopped ({status}).\n{tail}\n{}",
            log_path.display()
        ));
    }
    let text = bounded_text(&directory.join("result.json"), 256 * 1024, false)?;
    let result: Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
    Ok(Completion {
        result,
        log: log_path,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn oversized_results_reject_and_log_tail_stays_bounded() {
        let file = tempfile::NamedTempFile::new().unwrap();
        fs::write(file.path(), b"0123456789").unwrap();
        assert!(bounded_text(file.path(), 4, false).is_err());
        assert_eq!(bounded_text(file.path(), 4, true).unwrap(), "6789");
    }
    #[test]
    fn cancelled_job_never_opens_toolkit_or_starts_child() {
        let request = Request {
            toolkit: "nonexistent".into(),
            python: "nonexistent".into(),
            payload: Value::Null,
            hardware: true,
        };
        let result = run(request, &AtomicBool::new(true), &mut |_| {
            panic!("Unexpected progress")
        });
        assert!(result.err().unwrap().contains("Cancelled"));
    }
}

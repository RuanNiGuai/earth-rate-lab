//! ARC8-H2/H3 offline-only bridge. No HID, serial, NVS, OTA or flash commands.
//! The pinned ARC7 C solver and strict ARC8X1 decoder are embedded in the EXE;
//! Python 3 and a native C compiler are runtime prerequisites, not build requirements.
use std::{
    ffi::OsString,
    fs,
    path::{Path, PathBuf},
    process::{Command, Stdio},
    sync::atomic::{AtomicBool, Ordering},
    thread,
    time::{Duration, Instant},
};

#[derive(Clone, Debug)]
pub(crate) enum Request {
    Parse {
        log: PathBuf,
        output: PathBuf,
        tracker_uid: String,
        receiver_key: String,
        note: String,
    },
    Solve {
        capture: PathBuf,
        output: PathBuf,
    },
    Verify {
        bundle: PathBuf,
    },
}

#[derive(Debug)]
pub(crate) struct Completion {
    pub result: Result<String, String>,
    pub output: Option<PathBuf>,
    pub step: &'static str,
}

const SOURCES: [(&str, &str); 5] = [
    (
        "calibration.py",
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../tools/arc8_cal_pc/calibration.py"
        )),
    ),
    (
        "archive_transport.py",
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../tools/arc8_cal_pc/archive_transport.py"
        )),
    ),
    (
        "vendor/cal_archive_core.c",
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../tools/arc8_cal_pc/vendor/cal_archive_core.c"
        )),
    ),
    (
        "vendor/cal_archive_core.h",
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../tools/arc8_cal_pc/vendor/cal_archive_core.h"
        )),
    ),
    (
        "vendor/abi.c",
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../tools/arc8_cal_pc/vendor/abi.c"
        )),
    ),
];

fn bounded_file(path: &Path, max: u64) -> Result<(), String> {
    let info = fs::metadata(path).map_err(|e| format!("无法读取 {}: {e}", path.display()))?;
    if !info.is_file() || info.len() > max {
        return Err(format!("文件不是普通文件，或超过 {} 字节上限", max));
    }
    Ok(())
}

fn validate_identity(label: &str, value: &str) -> Result<(), String> {
    if !(8..=128).contains(&value.len())
        || value.contains("UNVERIFIED")
        || value.chars().any(|c| c.is_whitespace() || c.is_control())
    {
        return Err(format!(
            "{label} 需要已核对的 8–128 字符非空白标识，不能使用占位符"
        ));
    }
    Ok(())
}

fn validated_output(path: &Path) -> Result<(), String> {
    if path.exists() {
        return Err(format!("拒绝覆盖已有文件: {}", path.display()));
    }
    let parent = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .ok_or("输出文件必须指定有效目录")?;
    if !parent.is_dir() {
        return Err("输出目录不存在".into());
    }
    Ok(())
}

pub(crate) fn run(request: Request, cancel: &AtomicBool) -> Completion {
    let step = match &request {
        Request::Parse { .. } => "H3 归档解析",
        Request::Solve { .. } => "H2 联合求解",
        Request::Verify { .. } => "候选包校验",
    };
    let output = match &request {
        Request::Parse { output, .. } | Request::Solve { output, .. } => Some(output.clone()),
        Request::Verify { .. } => None,
    };
    Completion {
        result: execute(request, cancel),
        output,
        step,
    }
}

fn execute(request: Request, cancel: &AtomicBool) -> Result<String, String> {
    let (entry, args): (&str, Vec<OsString>) = match request {
        Request::Parse {
            log,
            output,
            tracker_uid,
            receiver_key,
            note,
        } => {
            bounded_file(&log, 2_000_000)?;
            validated_output(&output)?;
            validate_identity("Tracker UID", &tracker_uid)?;
            validate_identity("Receiver Key", &receiver_key)?;
            if note.trim().is_empty() || note.len() > 256 {
                return Err("采集来源说明不能为空，且不超过 256 字节".into());
            }
            (
                "archive_transport.py",
                vec![
                    "parse-log".into(),
                    log.into_os_string(),
                    "--tracker-uid".into(),
                    tracker_uid.into(),
                    "--receiver-key".into(),
                    receiver_key.into(),
                    "--note".into(),
                    note.into(),
                    "--out".into(),
                    output.into_os_string(),
                ],
            )
        }
        Request::Solve { capture, output } => {
            bounded_file(&capture, 1_000_000)?;
            validated_output(&output)?;
            (
                "calibration.py",
                vec![
                    "solve".into(),
                    capture.into_os_string(),
                    output.into_os_string(),
                ],
            )
        }
        Request::Verify { bundle } => {
            bounded_file(&bundle, 100_000)?;
            (
                "calibration.py",
                vec!["verify".into(), bundle.into_os_string()],
            )
        }
    };
    // Extraction is isolated from application configuration and user files.
    let dir = tempfile::tempdir().map_err(|e| format!("无法创建临时目录: {e}"))?;
    for (name, source) in SOURCES {
        let path = dir.path().join(name);
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }
        fs::write(&path, source).map_err(|e| format!("暂存脚本失败: {e}"))?;
    }
    let custom = std::env::var_os("ARC8_PYTHON");
    let program = custom.unwrap_or_else(|| {
        if cfg!(windows) {
            "py".into()
        } else {
            "python3".into()
        }
    });
    let mut process = Command::new(&program);
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        process.creation_flags(0x08000000); // CREATE_NO_WINDOW
        if program.as_os_str() == std::ffi::OsStr::new("py") {
            process.arg("-3");
        }
    }
    process
        .arg("-B")
        .arg(dir.path().join(entry))
        .args(args)
        .env("PYTHONDONTWRITEBYTECODE", "1")
        .current_dir(dir.path());
    let mut child = process
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| format!("无法运行 Python 3 ({program:?}): {e}; 可设置 ARC8_PYTHON"))?;
    let deadline = Instant::now() + Duration::from_secs(90);
    loop {
        if cancel.load(Ordering::Acquire) {
            let _ = child.kill();
            let _ = child.wait();
            return Err("已取消后台工具进程".into());
        }
        if child.try_wait().map_err(|e| e.to_string())?.is_some() {
            break;
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            return Err("工具超过 90 秒，已终止子进程；没有设备写入".into());
        }
        thread::sleep(Duration::from_millis(100));
    }
    let response = child
        .wait_with_output()
        .map_err(|e| format!("读取工具输出失败: {e}"))?;
    let stdout = String::from_utf8_lossy(&response.stdout);
    let stderr = String::from_utf8_lossy(&response.stderr);
    if !response.status.success() {
        return Err(format!(
            "{entry} 拒绝 (exit={}): {}",
            response.status,
            stderr.trim().chars().take(1200).collect::<String>()
        ));
    }
    Ok(format!(
        "{entry}: {}",
        stdout.trim().chars().take(1200).collect::<String>()
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rejects_placeholder_identity_and_existing_output() {
        assert!(validate_identity("uid", "UNVERIFIED-123").is_err());
        assert!(validate_identity("uid", "tracker 13").is_err());
        assert!(validate_identity("uid", "tracker-13-verified").is_ok());
        let d = tempfile::tempdir().unwrap();
        let p = d.path().join("already.json");
        fs::write(&p, b"original").unwrap();
        assert!(validated_output(&p).is_err());
        assert_eq!(fs::read(&p).unwrap(), b"original");
    }
    #[test]
    fn scripts_are_bundled_with_real_solver_and_decoder() {
        let c = SOURCES
            .iter()
            .find(|(name, _)| *name == "calibration.py")
            .unwrap()
            .1;
        let a = SOURCES
            .iter()
            .find(|(name, _)| *name == "archive_transport.py")
            .unwrap()
            .1;
        assert!(c.contains("cal_arc_solve"));
        assert!(a.contains("decode_lines"));
        assert!(a.contains("parse-log"));
    }
}

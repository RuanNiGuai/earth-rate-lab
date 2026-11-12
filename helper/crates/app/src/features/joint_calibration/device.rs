//! Identity-bound console protocol and journaled, non-atomic installation.
use super::service::{self, Candidate, Completion, DeviceKey, Snapshot, check};
use crate::{
    domain::SerialEndpoint,
    sessions::{RebootConsoleSource, ReservedConsole, ReservedConsoleSource},
    update::Progress,
};
use serde::Serialize;
use slimenrf_recording::{
    joint::{Coefficients, RuntimeLut},
    thermal_dump,
};
use std::{
    collections::BTreeMap,
    fs::{File, OpenOptions},
    io::Write,
    path::Path,
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};

pub(crate) trait Console {
    fn command(
        &mut self,
        command: &str,
        marker: &str,
        cancel: &AtomicBool,
    ) -> Result<String, String>;
    fn reboot(&mut self, cancel: &AtomicBool) -> Result<(), String>;
}
struct NativeConsole {
    source: RebootConsoleSource,
    console: Option<ReservedConsole>,
}
impl NativeConsole {
    fn connect(&mut self, cancel: &AtomicBool) -> Result<(), String> {
        let deadline = Instant::now() + Duration::from_secs(40);
        let mut error = String::new();
        while Instant::now() < deadline {
            check(!cancel.load(Ordering::Acquire), "Cancelled")?;
            match self.source.open() {
                Ok(console) => {
                    self.console = Some(console);
                    // CDC may announce DTR before its command loop is ready.
                    // Give a newly opened connection time to consume commands;
                    // never retry a possibly delivered mutating command.
                    std::thread::sleep(Duration::from_millis(350));
                    check(!cancel.load(Ordering::Acquire), "Cancelled")?;
                    return Ok(());
                }
                Err(e) => error = e,
            }
            std::thread::sleep(Duration::from_millis(200));
        }
        Err(format!("USB reconnect unconfirmed: {error}"))
    }
}
impl Console for NativeConsole {
    fn command(
        &mut self,
        command: &str,
        marker: &str,
        cancel: &AtomicBool,
    ) -> Result<String, String> {
        let result = self.exchange(command, marker, cancel);
        if result.is_err() {
            self.console = None;
        }
        result
    }
    fn reboot(&mut self, cancel: &AtomicBool) -> Result<(), String> {
        let result = self
            .console
            .as_mut()
            .ok_or("No console before reboot")?
            .write_command(b"reboot\r\n", cancel);
        self.console = None;
        // Delivery can be uncertain on reset. Always revalidate the returning
        // USB/RF identity and persisted values before reporting completion.
        std::thread::sleep(Duration::from_millis(1500));
        self.connect(cancel)
            .map_err(|e| format!("Reboot delivery {result:?}; {e}"))
    }
}
impl NativeConsole {
    fn exchange(
        &mut self,
        command: &str,
        marker: &str,
        cancel: &AtomicBool,
    ) -> Result<String, String> {
        if self.console.is_none() {
            self.connect(cancel)?;
        }
        let console = self.console.as_mut().expect("connected");
        let mut bytes = [0; 2048];
        // Drain earlier output; queued/echo text is never a save acknowledgement.
        let drain = Instant::now() + Duration::from_millis(300);
        while Instant::now() < drain {
            check(!cancel.load(Ordering::Acquire), "Cancelled")?;
            if console.read(&mut bytes)? == 0 {
                break;
            }
        }
        console.write_command(format!("{command}\r\n").as_bytes(), cancel)?;
        let mut response = String::new();
        let mut lines = crate::sessions::lines::Lines::default();
        let deadline = Instant::now() + Duration::from_secs(8);
        while Instant::now() < deadline {
            check(!cancel.load(Ordering::Acquire), "Cancelled")?;
            let n = console.read(&mut bytes)?;
            for &b in &bytes[..n] {
                if let Some(line) = lines.byte(b)? {
                    response.push_str(&line);
                    response.push('\n');
                }
            }
            check(
                response.len() <= 256 * 1024,
                "Console response exceeds limit",
            )?;
            if response.contains(marker) {
                return Ok(response);
            }
        }
        Err(format!("No complete acknowledgement for {command}"))
    }
}

struct Journal {
    file: File,
}
impl Journal {
    fn new(directory: &Path) -> Result<Self, String> {
        Ok(Self {
            file: OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(directory.join("journal.jsonl"))
                .map_err(|e| e.to_string())?,
        })
    }
    fn entry(&mut self, value: impl Serialize) -> Result<(), String> {
        serde_json::to_writer(&mut self.file, &value).map_err(|e| e.to_string())?;
        self.file
            .write_all(b"\n")
            .and_then(|()| self.file.sync_all())
            .map_err(|e| e.to_string())
    }
    fn send(
        &mut self,
        io: &mut impl Console,
        command: &str,
        marker: &str,
        cancel: &AtomicBool,
    ) -> Result<String, String> {
        self.entry(serde_json::json!({"command":command,"state":"about-to-send"}))?;
        let result = io.command(command, marker, cancel);
        self.entry(serde_json::json!({"command":command,"response":result.as_ref().ok(),"error":result.as_ref().err()}))?;
        result
    }
}
fn number(s: &str) -> Result<f64, String> {
    let v = s.parse::<f64>().map_err(|e| e.to_string())?;
    check(v.is_finite(), "Nonfinite console value")?;
    Ok(v)
}
fn numbers(s: &str, n: usize) -> Result<Vec<f64>, String> {
    let v: Vec<_> = s
        .split([' ', ',', '/'])
        .filter(|s| !s.is_empty())
        .map(number)
        .collect::<Result<_, _>>()?;
    check(v.len() == n, "Incomplete numeric console response")?;
    Ok(v)
}
fn field<'a>(text: &'a str, prefix: &str) -> Result<&'a str, String> {
    let rows: Vec<_> = text
        .lines()
        .filter_map(|line| line.trim().strip_prefix(prefix))
        .collect();
    check(rows.len() == 1, "Missing/ambiguous console field")?;
    Ok(rows[0])
}
fn identity(info: &str, endpoint: &SerialEndpoint) -> Result<(DeviceKey, u32), String> {
    check(
        info.lines()
            .any(|l| l.starts_with("SlimeVR SlimeNRF Tracker ")),
        "USB endpoint is not a tracker",
    )?;
    let build = info
        .split_once("(Commit ")
        .and_then(|(_, v)| v.split_once(','))
        .map(|(v, _)| v)
        .ok_or("Missing firmware build")?;
    check(
        service::supported_build(build),
        "Joint reconstruction needs a reviewed deterministic temperature LUT build",
    )?;
    let rf = field(info, "Device address: ")?.trim().to_uppercase();
    check(service::valid_address(&rf), "Invalid RF identity")?;
    let turns = info
        .split_once("degrees diff over ")
        .and_then(|(_, v)| v.split_once(" rev"))
        .map(|(v, _)| v)
        .ok_or("Missing sensitivity command convention")?
        .parse::<u32>()
        .map_err(|e| e.to_string())?;
    check(
        (1..=20).contains(&turns),
        "Unsupported sensitivity convention",
    )?;
    let serial = endpoint
        .serial_number
        .clone()
        .filter(|s| !s.is_empty())
        .ok_or("Missing USB serial")?;
    Ok((
        DeviceKey {
            serial,
            vid: endpoint.vid.ok_or("Missing VID")?,
            pid: endpoint.pid.ok_or("Missing PID")?,
            rf_address: rf,
            build: build.into(),
        },
        turns,
    ))
}
fn verify_identity(
    info: &str,
    endpoint: &SerialEndpoint,
    expected: &DeviceKey,
) -> Result<u32, String> {
    let (key, turns) = identity(info, endpoint)?;
    check(&key == expected, "Device identity/build changed")?;
    Ok(turns)
}
fn automatic(text: &str) -> Result<bool, String> {
    match field(text, "Auto-calibration: ")?.trim() {
        "enabled" => Ok(true),
        "disabled" => Ok(false),
        _ => Err("Unknown TCal automatic state".into()),
    }
}
fn archive_idle(text: &str) -> Result<(), String> {
    archive_state(text, false)
}
fn archive_state(text: &str, enabled: bool) -> Result<(), String> {
    let a = field(text, "ARC5 ")?;
    let b = field(text, "ARC7 ")?;
    let fields: BTreeMap<_, _> = a
        .split_whitespace()
        .chain(b.split_whitespace())
        .filter_map(|s| s.split_once('='))
        .collect();
    check(
        ["collecting", "save_pending", "pending"]
            .iter()
            .all(|k| fields.get(k) == Some(&"0"))
            && fields.get("capture") == Some(&"none")
            && fields.get("enabled") == Some(&if enabled { "1" } else { "0" }),
        "Archive calibration is active or pending",
    )
}
fn export_scales(
    io: &mut impl Console,
    journal: &mut Journal,
    endpoint: &SerialEndpoint,
    expected: &DeviceKey,
    cancel: &AtomicBool,
) -> Result<[String; 3], String> {
    // ARC8P1 exports only lazily loaded archive RAM. ON loads saved records;
    // it does not confirm a reference or start a capture. Always restore OFF.
    let result = (|| {
        journal.send(io, "cal archive on", "ARC4 ON:", cancel)?;
        archive_state(
            &journal.send(io, "cal archive status", "ARC7 ", cancel)?,
            true,
        )?;
        service::parse_scale_export(&journal.send(
            io,
            "cal archive export partial scale",
            "ARC8P1 END scale",
            cancel,
        )?)
    })();
    let cleanup_cancel = AtomicBool::new(false);
    let cleanup = (|| {
        verify_identity(
            &journal.send(io, "info", "degrees diff over", &cleanup_cancel)?,
            endpoint,
            expected,
        )?;
        journal.send(io, "cal archive off", "ARC4 OFF", &cleanup_cancel)?;
        archive_idle(&journal.send(io, "cal archive status", "ARC7 ", &cleanup_cancel)?)
    })();
    match (result, cleanup) {
        (Ok(records), Ok(())) => Ok(records),
        (Err(error), Ok(())) => Err(error),
        (result, Err(error)) => Err(format!(
            "Scale export {:?}; archive cleanup failed: {error}",
            result.err()
        )),
    }
}
fn read_settings(
    io: &mut impl Console,
    journal: &mut Journal,
    cancel: &AtomicBool,
) -> Result<Coefficients, String> {
    let s = journal.send(io, "gyro scale status", "armed=", cancel)?;
    check(
        s.contains("active=0 phase=0") && s.contains("armed=0"),
        "Scale capture is active",
    )?;
    let line = field(&s, "Gyro scale: ")?.split(';').next().unwrap();
    let tokens: Vec<_> = line.split_whitespace().collect();
    check(
        tokens.len() == 6 && tokens[0] == "X" && tokens[2] == "Y" && tokens[4] == "Z",
        "Malformed scale readback",
    )?;
    let scale = [number(tokens[1])?, number(tokens[3])?, number(tokens[5])?];
    let e = journal.send(io, "earth cal status", "Sync command:", cancel)?;
    check(
        e.contains("Earth cal: active=0;"),
        "Earth capture is active",
    )?;
    let earth = numbers(field(&e, "Reference body dps: ")?, 3)?;
    let g = journal.send(io, "gyro gsens status", "(0 = none)", cancel)?;
    let matrix = numbers(
        field(&g, "g-sens: ")?.split(" (0 = none)").next().unwrap(),
        9,
    )?;
    let c = Coefficients {
        scale,
        earth_dps: earth.try_into().map_err(|_| "Earth readback size")?,
        gsens_dps_per_g: std::array::from_fn(|i| {
            [matrix[3 * i], matrix[3 * i + 1], matrix[3 * i + 2]]
        }),
    };
    c.validate()?;
    Ok(c)
}
fn same_coefficients(actual: &Coefficients, expected: &Coefficients) -> Result<(), String> {
    check(
        actual
            .scale
            .iter()
            .zip(expected.scale)
            .all(|(a, b)| (a - b).abs() <= 2e-7)
            && actual
                .earth_dps
                .iter()
                .zip(expected.earth_dps)
                .all(|(a, b)| (a - b).abs() <= 1e-9)
            && actual
                .gsens_dps_per_g
                .iter()
                .flatten()
                .zip(expected.gsens_dps_per_g.iter().flatten())
                .all(|(a, b)| (a - b).abs() <= 1e-9),
        "Calibration readback mismatch",
    )
}
fn write_settings(
    io: &mut impl Console,
    journal: &mut Journal,
    c: &Coefficients,
    turns: u32,
    cancel: &AtomicBool,
) -> Result<(), String> {
    c.validate()?;
    let scale = c
        .scale
        .map(|s| format!("{:.9}", 360. * f64::from(turns) * (1. - 1. / s)))
        .join(",");
    journal.send(
        io,
        &format!("sens {scale}"),
        "Gyro scale saved and verified",
        cancel,
    )?;
    let matrix = c
        .gsens_dps_per_g
        .iter()
        .flatten()
        .map(|v| format!("{v:.9}"))
        .collect::<Vec<_>>()
        .join(" ");
    journal.send(
        io,
        &format!("gyro gsens set {matrix}"),
        "g-sens saved and read back",
        cancel,
    )?;
    let earth = c.earth_dps.map(|v| format!("{v:.9}")).join(" ");
    journal.send(
        io,
        &format!("earth ref set {earth}"),
        "Earth reference saved to NVS",
        cancel,
    )?;
    Ok(())
}
fn set_auto(
    io: &mut impl Console,
    journal: &mut Journal,
    enabled: bool,
    cancel: &AtomicBool,
) -> Result<(), String> {
    journal.send(
        io,
        if enabled {
            "tcal auto on"
        } else {
            "tcal auto off"
        },
        if enabled {
            "T-Cal start requested"
        } else {
            "T-Cal stop requested"
        },
        cancel,
    )?;
    let response = journal.send(io, "tcal status", "Auto-calibration:", cancel)?;
    check(
        automatic(&response)? == enabled,
        "TCal automatic state did not change",
    )
}
fn same_temperature(
    io: &mut impl Console,
    journal: &mut Journal,
    snapshot: &Snapshot,
    cancel: &AtomicBool,
) -> Result<(), String> {
    let dump = journal.send(
        io,
        "tcal dump",
        "End of dump. Total points printed:",
        cancel,
    )?;
    check(
        thermal_dump::parse_text(&dump).map_err(|e| e.to_string())?
            == thermal_dump::parse_text(snapshot.response("tcal dump")?)
                .map_err(|e| e.to_string())?,
        "Stored TCal observations changed; read inputs and reconstruct again",
    )?;
    let lut = RuntimeLut::parse(&journal.send(io, "tcal lut", "TCAL_LUT END", cancel)?)?;
    let expected = RuntimeLut::parse(snapshot.response("tcal lut")?)?;
    check(
        lut.points == expected.points
            && lut.nodes.len() == expected.nodes.len()
            && lut
                .nodes
                .iter()
                .zip(&expected.nodes)
                .all(|(a, b)| a.temperature_c == b.temperature_c && a.bias_dps == b.bias_dps),
        "Runtime TCal LUT changed",
    )
}
fn at_reference(debug: &str, face: &str) -> Result<bool, String> {
    check(
        debug.contains("Rest:Y") && !debug.contains("Rest:N"),
        "Tracker must remain at settled rest",
    )?;
    let rows: Vec<_> = debug
        .lines()
        .filter_map(|l| {
            l.split_once("RAW: A[")
                .and_then(|(_, r)| r.split_once(']'))
                .map(|(a, _)| a)
        })
        .collect();
    check(!rows.is_empty(), "Missing live gravity evidence")?;
    let axis = match face {
        "+X" | "-X" => 0,
        "+Y" | "-Y" => 1,
        "+Z" | "-Z" => 2,
        _ => return Err("Invalid first face".into()),
    };
    let sign = if face.starts_with('+') { 1. } else { -1. };
    for row in rows {
        let a = numbers(row, 3)?;
        let norm = a.iter().map(|v| v * v).sum::<f64>().sqrt();
        check(
            (0.96..=1.04).contains(&norm) && a[axis] * sign / norm >= 0.98,
            "Tracker is not in the first Earth reference face",
        )?;
    }
    check(
        debug.contains("EARTH:Y") || debug.contains("EARTH:N"),
        "Missing Earth runtime state",
    )?;
    Ok(debug.contains("EARTH:Y"))
}

fn wait_reference(
    io: &mut impl Console,
    journal: &mut Journal,
    face: &str,
    cancel: &AtomicBool,
) -> Result<(), String> {
    // Each native debug request observes one second. A fresh boot needs time
    // to establish rest and the Earth output; keep cancellation responsive.
    let mut last = String::new();
    for _ in 0..20 {
        check(!cancel.load(Ordering::Acquire), "Cancelled")?;
        let debug = journal.send(io, "debug 1", "Debug mode stopped", cancel)?;
        match at_reference(&debug, face) {
            Ok(true) => return Ok(()),
            Ok(false) => last = "Earth output is not enabled after reboot".into(),
            Err(error) => last = error,
        }
    }
    Err(format!("Post-reboot reference did not settle: {last}"))
}

pub(crate) fn run(
    source: ReservedConsoleSource,
    endpoint: SerialEndpoint,
    candidate: Option<Candidate>,
    cancel: &AtomicBool,
    progress: &mut dyn FnMut(Progress),
) -> Result<Completion, String> {
    let directory = service::output_directory()?;
    let mut journal = Journal::new(&directory)?;
    let mut io = NativeConsole {
        source: source.bind_reboot(endpoint.clone())?,
        console: None,
    };
    let result = match candidate {
        Some(candidate) => install(
            &mut io,
            &mut journal,
            &endpoint,
            &candidate,
            &directory,
            cancel,
            progress,
        )
        .map(|()| Completion::Installed(directory.clone())),
        None => read_snapshot(
            &mut io,
            &mut journal,
            &endpoint,
            &directory,
            cancel,
            progress,
        )
        .map(|s| Completion::Snapshot(s, directory.join("device-inputs.json"))),
    };
    if let Err(error) = &result {
        let _ = service::save_new(
            &directory.join("failure.json"),
            &serde_json::json!({"error":error}),
        );
    }
    result.map_err(|e| format!("{e}\n{}", directory.display()))
}
fn read_snapshot(
    io: &mut impl Console,
    journal: &mut Journal,
    endpoint: &SerialEndpoint,
    directory: &Path,
    cancel: &AtomicBool,
    progress: &mut dyn FnMut(Progress),
) -> Result<Snapshot, String> {
    progress(Progress::stage("joint.progress.reading"));
    let info = journal.send(io, "info", "degrees diff over", cancel)?;
    let (device, _) = identity(&info, endpoint)?;
    archive_idle(&journal.send(io, "cal archive status", "ARC7 ", cancel)?)?;
    let status = journal.send(io, "tcal status", "Auto-calibration:", cancel)?;
    let auto = automatic(&status)?;
    let result = (|| {
        set_auto(io, journal, false, cancel)?;
        let mut responses = BTreeMap::from([("info".into(), info), ("tcal status".into(), status)]);
        for (command, marker) in [
            ("tcal dump", "End of dump. Total points printed:"),
            ("tcal lut", "TCAL_LUT END"),
        ] {
            responses.insert(command.into(), journal.send(io, command, marker, cancel)?);
        }
        let scale_records = export_scales(io, journal, endpoint, &device, cancel)?;
        let snapshot = Snapshot {
            schema: 1,
            device: device.clone(),
            responses,
            scale_records,
        };
        snapshot.inputs(0., true)?;
        service::save_new(&directory.join("device-inputs.json"), &snapshot)?;
        Ok(snapshot)
    })();
    let cleanup_cancel = AtomicBool::new(false);
    let cleanup = (|| {
        verify_identity(
            &journal.send(io, "info", "degrees diff over", &cleanup_cancel)?,
            endpoint,
            &device,
        )?;
        set_auto(io, journal, auto, &cleanup_cancel)
    })();
    combine_cleanup(result, cleanup)
}
fn combine_cleanup<T>(result: Result<T, String>, cleanup: Result<(), String>) -> Result<T, String> {
    match (result, cleanup) {
        (Ok(v), Ok(())) => Ok(v),
        (Err(e), Ok(())) => Err(e),
        (Ok(_), Err(e)) => Err(format!(
            "Operation completed; TCal state restoration failed: {e}"
        )),
        (Err(e), Err(c)) => Err(format!("{e}; TCal state restoration failed: {c}")),
    }
}

fn install(
    io: &mut impl Console,
    journal: &mut Journal,
    endpoint: &SerialEndpoint,
    candidate: &Candidate,
    directory: &Path,
    cancel: &AtomicBool,
    progress: &mut dyn FnMut(Progress),
) -> Result<(), String> {
    let solution = &candidate.solution;
    solution.coefficients.validate()?;
    check(
        candidate.schema == 1
            && solution.installable
            && solution.earth_fit_rms_dps <= 0.001
            && solution.earth_holdout_rms_dps <= 0.001
            && solution.gsens_fit_rms_dps <= 0.006
            && solution.gsens_holdout_rms_dps <= 0.006,
        "Candidate quality gates failed",
    )?;
    check(
        candidate.snapshot_sha256
            == service::digest(
                &serde_json::to_vec(&candidate.snapshot).map_err(|e| e.to_string())?,
            ),
        "Candidate input snapshot changed",
    )?;
    let expected = &candidate.snapshot.device;
    progress(Progress::stage("joint.progress.backup"));
    let info = journal.send(io, "info", "degrees diff over", cancel)?;
    let turns = verify_identity(&info, endpoint, expected)?;
    archive_idle(&journal.send(io, "cal archive status", "ARC7 ", cancel)?)?;
    let auto = automatic(&journal.send(io, "tcal status", "Auto-calibration:", cancel)?)?;
    let (mut backup, mut mutated, mut prior_earth) = (None, false, false);
    let result = (|| {
        set_auto(io, journal, false, cancel)?;
        same_temperature(io, journal, &candidate.snapshot, cancel)?;
        let scale_records = export_scales(io, journal, endpoint, expected, cancel)?;
        check(
            scale_records == candidate.snapshot.scale_records,
            "Stored Scale inputs changed; reconstruct again",
        )?;
        let previous = read_settings(io, journal, cancel)?;
        let debug = journal.send(io, "debug 1", "Debug mode stopped", cancel)?;
        prior_earth = at_reference(
            &debug,
            &solution.poses.first().ok_or("Missing first pose")?.cardinal,
        )?;
        service::save_new(
            &directory.join("backup.json"),
            &serde_json::json!({"device":expected,"coefficients":previous,"auto_tcal":auto,"earth_enabled":prior_earth,"inputs":candidate.snapshot}),
        )?;
        service::save_new(&directory.join("candidate.json"), candidate)?;
        backup = Some(previous);
        mutated = true;
        progress(Progress::stage("joint.progress.writing"));
        journal.send(io, "earth off", "EARTH disabled:", cancel)?;
        write_settings(io, journal, &solution.coefficients, turns, cancel)?;
        same_coefficients(&read_settings(io, journal, cancel)?, &solution.coefficients)?;
        same_temperature(io, journal, &candidate.snapshot, cancel)?;
        progress(Progress::stage("joint.progress.reboot"));
        journal.entry(serde_json::json!({"stage":"reboot"}))?;
        io.reboot(cancel)?;
        let info = journal.send(io, "info", "degrees diff over", cancel)?;
        verify_identity(&info, endpoint, expected)?;
        set_auto(io, journal, false, cancel)?;
        let after = read_settings(io, journal, cancel)?;
        same_coefficients(&after, &solution.coefficients)?;
        same_temperature(io, journal, &candidate.snapshot, cancel)?;
        let ota = journal.send(io, "ota status", "app confirmed=", cancel)?;
        check(
            ota.contains("app confirmed=1"),
            "Firmware activation is unconfirmed",
        )?;
        wait_reference(io, journal, &solution.poses[0].cardinal, cancel)?;
        service::save_new(
            &directory.join("readback.json"),
            &serde_json::json!({"device":expected,"coefficients":after,"reboot_readback":true,"temperature_preserved":true,"atomic":false}),
        )?;
        Ok(())
    })();
    let result = if let Err(error) = result {
        if mutated {
            progress(Progress::stage("joint.progress.rollback"));
            let cleanup_cancel = AtomicBool::new(false);
            let rollback = (|| {
                let info = journal.send(io, "info", "degrees diff over", &cleanup_cancel)?;
                verify_identity(&info, endpoint, expected)?;
                set_auto(io, journal, false, &cleanup_cancel)?;
                journal.send(io, "earth off", "EARTH disabled:", &cleanup_cancel)?;
                let previous = backup.as_ref().ok_or("Missing durable backup")?;
                write_settings(io, journal, previous, turns, &cleanup_cancel)?;
                io.reboot(&cleanup_cancel)?;
                verify_identity(
                    &journal.send(io, "info", "degrees diff over", &cleanup_cancel)?,
                    endpoint,
                    expected,
                )?;
                same_coefficients(&read_settings(io, journal, &cleanup_cancel)?, previous)?;
                same_temperature(io, journal, &candidate.snapshot, &cleanup_cancel)?;
                if !prior_earth {
                    journal.send(io, "earth off", "EARTH disabled:", &cleanup_cancel)?;
                }
                Ok::<(), String>(())
            })();
            journal.entry(serde_json::json!({"rollback_verified":rollback.is_ok(),"rollback_error":rollback.as_ref().err()}))?;
            Err(format!(
                "{error}; rollback {}",
                match rollback {
                    Ok(()) => "verified after reboot".into(),
                    Err(e) => format!("UNCONFIRMED: {e}"),
                }
            ))
        } else {
            Err(error)
        }
    } else {
        Ok(())
    };
    // Cleanup does not inherit cancellation. Verify RF again before any write
    // so a replacement device never receives even a TCal state restoration.
    let cleanup_cancel = AtomicBool::new(false);
    let cleanup = (|| {
        verify_identity(
            &journal.send(io, "info", "degrees diff over", &cleanup_cancel)?,
            endpoint,
            expected,
        )?;
        set_auto(io, journal, auto, &cleanup_cancel)
    })();
    let result = combine_cleanup(result, cleanup);
    journal.entry(
        serde_json::json!({"finished":true,"success":result.is_ok(),"error":result.as_ref().err()}),
    )?;
    result
}

#[cfg(test)]
mod tests;

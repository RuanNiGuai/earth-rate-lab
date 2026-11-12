use super::*;
use crate::domain::SerialPortKind;
use slimenrf_recording::joint::{Audit, EARTH_RATE_DPS, PoseReport, Solution};

#[test]
fn reviewed_wom_build_is_accepted_without_admitting_unknown_successors() {
    let endpoint = endpoint();
    let info = "SlimeVR SlimeNRF Tracker Fixture\nVersion (Commit 06ce018fa6d1_wom3_t02_on, Build fixture)\nDevice address: 010203040506\nGyroscope sensitivity (degrees diff over 5 rev): 0 0 0\n";
    assert!(identity(info, &endpoint).is_ok());
    assert!(identity(&info.replace("wom3", "wom4"), &endpoint).is_err());
    assert!(identity(&info.replace("06ce018fa6d1", "ffffffffffff"), &endpoint).is_err());
    assert!(service::supported_build("fixture_jointv2"));
    assert!(!service::supported_build("arbitrary_firmware"));
}

fn scale_records() -> [String; 3] {
    std::array::from_fn(|axis| {
        let mut b = vec![0u8; 584];
        for (i, v) in [0x41353252u32, 2, 1, 7, axis as u32 + 1]
            .into_iter()
            .enumerate()
        {
            b[4 * i..4 * i + 4].copy_from_slice(&v.to_le_bytes());
        }
        b[20] = 1;
        b[21] = axis as u8;
        b[576] = axis as u8;
        b[577] = 20;
        let mut f = [0f32; 22];
        f[3] = 1.;
        f[6] = 25.;
        f[7 + axis] = 7200.;
        f[10] = 100.;
        f[13] = 2500.;
        f[14] = 100.;
        f[17] = 25.;
        f[18] = 25.;
        for (i, v) in f.iter().enumerate() {
            b[488 + 4 * i..492 + 4 * i].copy_from_slice(&v.to_le_bytes());
        }
        let crc = crc32fast::hash(&b[..580]);
        b[580..].copy_from_slice(&crc.to_le_bytes());
        service::hex(&b)
    })
}
fn frame(records: &[String; 3]) -> String {
    let bytes: Vec<_> = records
        .iter()
        .flat_map(|h| service::unhex(h).unwrap())
        .collect();
    let sha = service::digest(&bytes);
    let mut s = format!("ARC8P1 BEGIN scale 3 3 7 {sha}\n");
    for (i, r) in records.iter().enumerate() {
        s.push_str(&format!("ARC8P1 DATA scale {i} 0 {r}\n"));
    }
    s.push_str(&format!("ARC8P1 END scale 1752 {sha}\n"));
    s
}
fn endpoint() -> SerialEndpoint {
    SerialEndpoint {
        port_name: "fixture-port".into(),
        label: "Fixture".into(),
        manufacturer: None,
        product: None,
        kind: SerialPortKind::Usb,
        serial_number: Some("synthetic-usb".into()),
        vid: Some(0x1209),
        pid: Some(0x7690),
        usb_interface: None,
    }
}
fn baseline() -> Coefficients {
    Coefficients {
        scale: [1.; 3],
        earth_dps: [EARTH_RATE_DPS, 0., 0.],
        gsens_dps_per_g: [[0.; 3]; 3],
    }
}
fn fixture() -> (Candidate, Fake) {
    let key = DeviceKey {
        serial: "synthetic-usb".into(),
        vid: 0x1209,
        pid: 0x7690,
        rf_address: "001122334455".into(),
        build: "fixture_jointv2".into(),
    };
    let dump="Dumping 2 collected temperature calibration points:\nIndex | Temp (C) | Bias X   | Bias Y   | Bias Z   |\n0 | 20 | 0 | 0 | 0\n1 | 30 | 0 | 0 | 0\nEnd of dump. Total points printed: 2\n".to_owned();
    let lut="TCAL_LUT valid=1 count=2 version=2 points=2\nLUT 0 20 0 0 0\nLUT 1 30 0 0 0\nTCAL_LUT END\n".to_owned();
    let snapshot = Snapshot {
        schema: 1,
        device: key,
        responses: BTreeMap::from([("tcal dump".into(), dump), ("tcal lut".into(), lut)]),
        scale_records: scale_records(),
    };
    let mut coefficients = baseline();
    coefficients.scale = [1.01, 1.02, 0.99];
    coefficients.gsens_dps_per_g = [[0.001, 0.002, 0.003]; 3];
    let solution = Solution {
        coefficients,
        audit: Audit {
            unique_samples: 160000,
            duplicates: 0,
            reordered: 0,
            missing: 0,
            duration_s: 400.,
            rate_hz: 400.,
            sequence_wraps: 2,
        },
        poses: vec![PoseReport {
            detected_interval_s: [0., 30.],
            cardinal: "+X".into(),
            training_samples: 6000,
            holdout_samples: 4000,
            excluded_blocks: 0,
        }],
        iterations: 5,
        earth_fit_rms_dps: 0.0001,
        earth_holdout_rms_dps: 0.0002,
        gsens_fit_rms_dps: 0.001,
        gsens_holdout_rms_dps: 0.002,
        earth_horizontal_ratio: 1.,
        earth_closure_deg: 0.1,
        gravity_mismatch_deg: vec![0.; 9],
        installable: true,
    };
    let candidate = Candidate {
        schema: 1,
        capture_sha256: "fixture".into(),
        snapshot_sha256: service::digest(&serde_json::to_vec(&snapshot).unwrap()),
        inputs: snapshot.inputs(30., true).unwrap(),
        snapshot: snapshot.clone(),
        solution,
    };
    let fake = Fake {
        snapshot,
        values: baseline(),
        auto: true,
        earth: true,
        commands: Vec::new(),
        fail_once: false,
        cancel_write: false,
        replacement: false,
        reboots: 0,
        archive_enabled: false,
        fail_command: None,
        cancel_export: false,
        boot_unsettled: 0,
    };
    (candidate, fake)
}
struct Fake {
    snapshot: Snapshot,
    values: Coefficients,
    auto: bool,
    earth: bool,
    commands: Vec<String>,
    fail_once: bool,
    cancel_write: bool,
    replacement: bool,
    reboots: usize,
    archive_enabled: bool,
    fail_command: Option<String>,
    cancel_export: bool,
    boot_unsettled: usize,
}
impl Console for Fake {
    fn command(
        &mut self,
        command: &str,
        _marker: &str,
        cancel: &AtomicBool,
    ) -> Result<String, String> {
        check(!cancel.load(Ordering::Acquire), "Cancelled")?;
        self.commands.push(command.into());
        if self.fail_command.as_deref() == Some(command) {
            return Err(format!("Injected failure: {command}"));
        }
        match command {
            "info" => Ok(format!(
                "SlimeVR SlimeNRF Tracker Fixture\nVersion (Commit fixture_jointv2, Build fixture)\nDevice address: {}\nGyroscope sensitivity (degrees diff over 5 rev): 0 0 0\n",
                if self.replacement && self.reboots > 0 {
                    "AABBCCDDEEFF"
                } else {
                    &self.snapshot.device.rf_address
                }
            )),
            "cal archive status" => Ok(format!(
                "ARC5 enabled={} collecting=0 save_pending=0\nARC7 capture=none pending=0\n",
                u8::from(self.archive_enabled)
            )),
            "cal archive on" => {
                self.archive_enabled = true;
                Ok("ARC4 ON:".into())
            }
            "cal archive off" => {
                self.archive_enabled = false;
                Ok("ARC4 OFF".into())
            }
            "tcal status" => Ok(format!(
                "Auto-calibration: {}\n",
                if self.auto { "enabled" } else { "disabled" }
            )),
            "tcal auto on" => {
                self.auto = true;
                Ok("T-Cal start requested".into())
            }
            "tcal auto off" => {
                self.auto = false;
                Ok("T-Cal stop requested".into())
            }
            "tcal dump" | "tcal lut" => Ok(self.snapshot.response(command)?.into()),
            "cal archive export partial scale" => {
                check(self.archive_enabled, "ARC8P1 ERROR busy_or_not_loaded")?;
                if self.cancel_export {
                    cancel.store(true, Ordering::Release);
                    return Err("Cancelled during export".into());
                }
                Ok(frame(&self.snapshot.scale_records))
            }
            "gyro scale status" => Ok(format!(
                "Gyro scale: X {:.9} Y {:.9} Z {:.9}; active=0 phase=0 signed angle=0; armed=0\n",
                self.values.scale[0], self.values.scale[1], self.values.scale[2]
            )),
            "gyro gsens status" => Ok(format!(
                "g-sens: {} (0 = none)\n",
                self.values
                    .gsens_dps_per_g
                    .iter()
                    .flatten()
                    .map(|v| format!("{v:.9}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            )),
            "earth cal status" => Ok(format!(
                "Earth cal: active=0; reference source=NVS\nReference body dps: {:.9} {:.9} {:.9}\nSync command: earth ref set\n",
                self.values.earth_dps[0], self.values.earth_dps[1], self.values.earth_dps[2]
            )),
            "debug 1" if self.reboots > 0 && self.boot_unsettled > 0 => {
                self.boot_unsettled -= 1;
                Ok("RAW: A[1,0,0] G[0,0,0]\nRest:N EARTH:N\nDebug mode stopped".into())
            }
            "debug 1" => Ok(format!(
                "RAW: A[1,0,0] G[0,0,0]\nRest:Y EARTH:{}\nDebug mode stopped\n",
                if self.earth { "Y" } else { "N" }
            )),
            "earth off" => {
                self.earth = false;
                Ok("EARTH disabled:".into())
            }
            "ota status" => Ok("app confirmed=1".into()),
            _ => {
                if let Some(s) = command.strip_prefix("sens ") {
                    let v = numbers(s, 3)?;
                    self.values.scale = std::array::from_fn(|i| 1. / (1. - v[i] / 1800.));
                    return Ok("Gyro scale saved and verified".into());
                }
                if let Some(s) = command.strip_prefix("gyro gsens set ") {
                    let v = numbers(s, 9)?;
                    self.values.gsens_dps_per_g =
                        std::array::from_fn(|i| [v[3 * i], v[3 * i + 1], v[3 * i + 2]]);
                    if self.cancel_write {
                        self.cancel_write = false;
                        cancel.store(true, Ordering::Release);
                        return Err("Cancelled after mutation".into());
                    }
                    if self.fail_once {
                        self.fail_once = false;
                        return Err("Lost acknowledgement after mutation".into());
                    }
                    return Ok("g-sens saved and read back".into());
                }
                if let Some(s) = command.strip_prefix("earth ref set ") {
                    self.values.earth_dps = numbers(s, 3)?.try_into().unwrap();
                    return Ok("Earth reference saved to NVS".into());
                }
                Err(format!("Unexpected command {command}"))
            }
        }
    }
    fn reboot(&mut self, _: &AtomicBool) -> Result<(), String> {
        self.reboots += 1;
        self.earth = true;
        Ok(())
    }
}
fn install_fixture(
    candidate: &Candidate,
    io: &mut Fake,
    cancel: &AtomicBool,
) -> (Result<(), String>, tempfile::TempDir) {
    let dir = tempfile::tempdir().unwrap();
    let mut journal = Journal::new(dir.path()).unwrap();
    let result = install(
        io,
        &mut journal,
        &endpoint(),
        candidate,
        dir.path(),
        cancel,
        &mut |_| {},
    );
    (result, dir)
}
#[test]
fn installation_verifies_reboot_and_restores_automatic_state() {
    let (c, mut io) = fixture();
    io.boot_unsettled = 2;
    let (result, dir) = install_fixture(&c, &mut io, &AtomicBool::new(false));
    result.unwrap();
    same_coefficients(&io.values, &c.solution.coefficients).unwrap();
    assert!(io.auto);
    assert!(!io.archive_enabled);
    assert_eq!(io.boot_unsettled, 0);
    assert_eq!(io.reboots, 1);
    assert!(dir.path().join("backup.json").exists());
    assert!(dir.path().join("readback.json").exists());
}

#[test]
fn export_cancel_restores_archive_and_auto_without_coefficient_writes() {
    let (_, mut io) = fixture();
    io.cancel_export = true;
    let dir = tempfile::tempdir().unwrap();
    let mut journal = Journal::new(dir.path()).unwrap();
    let result = read_snapshot(
        &mut io,
        &mut journal,
        &endpoint(),
        dir.path(),
        &AtomicBool::new(false),
        &mut |_| {},
    );
    assert!(result.unwrap_err().contains("Cancelled during export"));
    assert!(!io.archive_enabled);
    assert!(io.auto);
    same_coefficients(&io.values, &baseline()).unwrap();
    assert!(!dir.path().join("device-inputs.json").exists());
}

#[test]
fn archive_cleanup_and_prewrite_read_failures_never_write_coefficients() {
    for command in ["cal archive off", "gyro scale status"] {
        let (c, mut io) = fixture();
        io.fail_command = Some(command.into());
        let (result, dir) = install_fixture(&c, &mut io, &AtomicBool::new(false));
        assert!(result.unwrap_err().contains(command));
        same_coefficients(&io.values, &baseline()).unwrap();
        assert!(io.auto);
        assert_eq!(io.reboots, 0);
        assert!(!dir.path().join("backup.json").exists());
    }
}

#[test]
fn changed_temperature_or_scale_rejects_stale_candidate() {
    for temperature in [false, true] {
        let (c, mut io) = fixture();
        if temperature {
            let lut = io.snapshot.responses.get_mut("tcal lut").unwrap();
            *lut = lut.replace("LUT 0 20 0 0 0", "LUT 0 20 0.001 0 0");
        } else {
            let mut bytes = service::unhex(&io.snapshot.scale_records[0]).unwrap();
            bytes[16] += 1;
            let crc = crc32fast::hash(&bytes[..580]);
            bytes[580..].copy_from_slice(&crc.to_le_bytes());
            io.snapshot.scale_records[0] = service::hex(&bytes);
        }
        let (result, _) = install_fixture(&c, &mut io, &AtomicBool::new(false));
        assert!(result.unwrap_err().contains("changed"));
        same_coefficients(&io.values, &baseline()).unwrap();
        assert!(io.auto);
        assert!(!io.archive_enabled);
        assert_eq!(io.reboots, 0);
    }
}

#[test]
fn persistent_write_failure_reports_unconfirmed_rollback() {
    let (c, mut io) = fixture();
    let values = c
        .solution
        .coefficients
        .earth_dps
        .map(|v| format!("{v:.9}"))
        .join(" ");
    io.fail_command = Some(format!("earth ref set {values}"));
    let (result, dir) = install_fixture(&c, &mut io, &AtomicBool::new(false));
    assert!(result.unwrap_err().contains("rollback UNCONFIRMED"));
    assert!(dir.path().join("backup.json").exists());
    assert!(io.auto);
}
#[test]
fn lost_ack_and_cancel_restore_all_records_with_independent_cleanup() {
    for cancel_write in [false, true] {
        let (c, mut io) = fixture();
        io.fail_once = !cancel_write;
        io.cancel_write = cancel_write;
        let (result, _) = install_fixture(&c, &mut io, &AtomicBool::new(false));
        assert!(result.unwrap_err().contains("rollback verified"));
        same_coefficients(&io.values, &baseline()).unwrap();
        assert!(io.auto);
        assert_eq!(io.reboots, 1);
    }
}
#[test]
fn replacement_after_reboot_never_receives_restore_commands() {
    let (c, mut io) = fixture();
    io.replacement = true;
    let (result, _) = install_fixture(&c, &mut io, &AtomicBool::new(false));
    assert!(result.unwrap_err().contains("UNCONFIRMED"));
    let writes = io
        .commands
        .iter()
        .filter(|c| c.starts_with("sens "))
        .count();
    assert_eq!(writes, 1);
    assert_eq!(io.commands.last().unwrap(), "info");
}
#[test]
fn identity_and_quality_fail_before_any_mutation() {
    let (mut c, mut io) = fixture();
    c.solution.installable = false;
    assert!(
        install_fixture(&c, &mut io, &AtomicBool::new(false))
            .0
            .is_err()
    );
    assert!(io.commands.is_empty());
    c.solution.installable = true;
    c.snapshot.device.rf_address = "AABBCCDDEEFF".into();
    c.snapshot_sha256 = service::digest(&serde_json::to_vec(&c.snapshot).unwrap());
    assert!(
        install_fixture(&c, &mut io, &AtomicBool::new(false))
            .0
            .is_err()
    );
    assert_eq!(io.commands, ["info"]);
}
#[test]
fn partial_scale_export_checks_hash_crc_offsets_and_reference() {
    let records = scale_records();
    let text = frame(&records);
    assert_eq!(service::parse_scale_export(&text).unwrap(), records);
    assert!(
        service::parse_scale_export(&text.replace("DATA scale 0 0", "DATA scale 0 1")).is_err()
    );
    let mut bad = records.clone();
    let mut bytes = service::unhex(&bad[1]).unwrap();
    bytes[12] = 9;
    let crc = crc32fast::hash(&bytes[..580]);
    bytes[580..].copy_from_slice(&crc.to_le_bytes());
    bad[1] = service::hex(&bytes);
    assert!(service::parse_scale_export(&frame(&bad)).is_err());
}

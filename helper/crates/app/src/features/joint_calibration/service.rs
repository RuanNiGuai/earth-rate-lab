use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use slimenrf_recording::{
    joint::{self, Inputs, RuntimeLut, ScaleRecord, Solution},
    thermal_dump,
};
use std::{
    collections::BTreeMap,
    fs::{File, OpenOptions},
    io::{Read, Write},
    path::{Path, PathBuf},
    sync::atomic::AtomicBool,
};

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub(crate) struct DeviceKey {
    pub serial: String,
    pub vid: u16,
    pub pid: u16,
    pub rf_address: String,
    pub build: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct Snapshot {
    pub schema: u32,
    pub device: DeviceKey,
    pub responses: BTreeMap<String, String>,
    pub scale_records: [String; 3],
}
impl Snapshot {
    pub fn inputs(&self, latitude_deg: f64, confirmed: bool) -> Result<Inputs, String> {
        check(
            self.schema == 1
                && !self.device.serial.is_empty()
                && valid_address(&self.device.rf_address)
                && supported_build(&self.device.build),
            "Unsupported or unbound device snapshot (requires a reviewed deterministic LUT build)",
        )?;
        let dump =
            thermal_dump::parse_text(self.response("tcal dump")?).map_err(|e| e.to_string())?;
        let lut = RuntimeLut::parse(self.response("tcal lut")?)?;
        check(
            dump.points.len() == lut.points as usize,
            "TCal records and runtime LUT disagree",
        )?;
        let records: Vec<_> = self
            .scale_records
            .iter()
            .enumerate()
            .map(|(j, h)| {
                let b = unhex(h)?;
                check(b.len() == 584, "Scale record length")?;
                let reference = u32::from_le_bytes(b[12..16].try_into().expect("bounded record"));
                ScaleRecord::from_arc2(&b, j, reference)
            })
            .collect::<Result<_, String>>()?;
        check(
            records
                .iter()
                .all(|r| r.reference_id == records[0].reference_id),
            "Scale reference mismatch",
        )?;
        Ok(Inputs {
            lut,
            scales: records.try_into().map_err(|_| "Scale record count")?,
            latitude_deg,
            temperature_reference_is_first_pose: confirmed,
        })
    }
    pub fn response(&self, command: &str) -> Result<&str, String> {
        self.responses
            .get(command)
            .map(String::as_str)
            .ok_or_else(|| format!("Missing snapshot response: {command}"))
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct Candidate {
    pub schema: u32,
    pub capture_sha256: String,
    pub snapshot_sha256: String,
    pub snapshot: Snapshot,
    pub inputs: Inputs,
    pub solution: Solution,
}
pub(crate) enum Completion {
    Snapshot(Snapshot, PathBuf),
    Candidate(Box<Candidate>, PathBuf),
    Installed(PathBuf),
}
pub(crate) fn check(ok: bool, message: &str) -> Result<(), String> {
    if ok { Ok(()) } else { Err(message.into()) }
}
pub(crate) fn supported_build(build: &str) -> bool {
    // This exact deployed WoM repair retains jointv2's deterministic LUT and
    // console schemas. Do not admit arbitrary future WoM/nightwear builds.
    build.ends_with("_jointv2") || build == "06ce018fa6d1_wom3_t02_on"
}
pub(crate) fn valid_address(s: &str) -> bool {
    s.len() == 12 && s.bytes().all(|b| b.is_ascii_hexdigit())
}
pub(crate) fn digest(bytes: &[u8]) -> String {
    hex(&Sha256::digest(bytes))
}
pub(crate) fn unhex(s: &str) -> Result<Vec<u8>, String> {
    check(
        s.len() <= 4096 && s.len().is_multiple_of(2) && s.bytes().all(|b| b.is_ascii_hexdigit()),
        "Invalid bounded hex payload",
    )?;
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|e| e.to_string()))
        .collect()
}
pub(crate) fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}
pub(crate) fn save_new(path: &Path, value: &impl Serialize) -> Result<(), String> {
    let bytes = serde_json::to_vec_pretty(value).map_err(|e| e.to_string())?;
    let mut f = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|e| e.to_string())?;
    f.write_all(&bytes)
        .and_then(|()| f.sync_all())
        .map_err(|e| e.to_string())
}
pub(crate) fn output_directory() -> Result<PathBuf, String> {
    let root = crate::storage_path::preferences()?
        .parent()
        .ok_or("Missing private storage directory")?
        .join("joint-calibration");
    std::fs::create_dir_all(&root).map_err(|e| e.to_string())?;
    let path = root.join(format!(
        "run-{}",
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_err(|e| e.to_string())?
            .as_nanos()
    ));
    std::fs::create_dir(&path).map_err(|e| e.to_string())?;
    Ok(path)
}
pub(crate) fn import_snapshot(path: &Path) -> Result<Snapshot, String> {
    let mut bytes = Vec::new();
    File::open(path)
        .map_err(|e| e.to_string())?
        .take(2 * 1024 * 1024 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    check(bytes.len() <= 2 * 1024 * 1024, "Snapshot exceeds 2 MiB")?;
    let snapshot: Snapshot = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    snapshot.inputs(0., true)?;
    Ok(snapshot)
}
fn hash_file(path: &Path, cancel: &AtomicBool) -> Result<String, String> {
    let mut file = File::open(path).map_err(|e| e.to_string())?;
    let mut hash = Sha256::new();
    let mut buf = [0u8; 65536];
    let mut total = 0u64;
    loop {
        check(
            !cancel.load(std::sync::atomic::Ordering::Acquire),
            "Cancelled",
        )?;
        let n = file.read(&mut buf).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        total += n as u64;
        check(total <= 2 * 1024 * 1024 * 1024, "Recording exceeds 2 GiB")?;
        hash.update(&buf[..n]);
    }
    Ok(hex(&hash.finalize()))
}
pub(crate) fn solve(
    path: PathBuf,
    snapshot: Snapshot,
    latitude: f64,
    confirmed: bool,
    cancel: &AtomicBool,
) -> Result<Completion, String> {
    let inputs = snapshot.inputs(latitude, confirmed)?;
    let before = hash_file(&path, cancel)?;
    let recording = joint::load_recording(&path, cancel)?;
    check(
        recording.tracks[0]
            .identity
            .rf_address
            .as_deref()
            .is_some_and(|a| a.eq_ignore_ascii_case(&snapshot.device.rf_address)),
        "Recording RF identity does not match the device snapshot",
    )?;
    let solution = joint::reconstruct(&recording, &inputs, cancel)?;
    check(
        hash_file(&path, cancel)? == before,
        "Recording changed while reconstructing",
    )?;
    let candidate = Candidate {
        schema: 1,
        capture_sha256: before,
        snapshot_sha256: digest(&serde_json::to_vec(&snapshot).map_err(|e| e.to_string())?),
        snapshot,
        inputs,
        solution,
    };
    let output = output_directory()?.join("candidate.json");
    save_new(&output, &candidate)?;
    Ok(Completion::Candidate(Box::new(candidate), output))
}
pub(crate) fn parse_scale_export(text: &str) -> Result<[String; 3], String> {
    check(text.len() <= 128 * 1024, "Scale export exceeds limit")?;
    let mut reference = None;
    let mut hash = None;
    let mut ended = false;
    let mut records: [Vec<u8>; 3] = std::array::from_fn(|_| Vec::new());
    let mut previous = 0;
    for line in text
        .lines()
        .map(str::trim)
        .filter(|s| s.starts_with("ARC8P1"))
    {
        check(!ended, "Scale data after END")?;
        let w: Vec<_> = line.split_whitespace().collect();
        match w.get(1).copied() {
            Some("BEGIN") => {
                check(
                    w.len() == 7
                        && w[2] == "scale"
                        && w[3] == "3"
                        && w[4] == "3"
                        && reference.is_none(),
                    "Invalid Scale BEGIN",
                )?;
                reference = Some(w[5].parse::<u32>().map_err(|e| e.to_string())?);
                hash = Some(w[6].to_owned());
            }
            Some("DATA") => {
                check(
                    w.len() == 6 && w[2] == "scale" && reference.is_some(),
                    "Invalid Scale DATA",
                )?;
                let j = w[3].parse::<usize>().map_err(|e| e.to_string())?;
                check(j < 3 && j >= previous, "Scale slots out of order")?;
                previous = j;
                let offset = w[4].parse::<usize>().map_err(|e| e.to_string())?;
                check(offset == records[j].len(), "Scale offset discontinuity")?;
                let chunk = unhex(w[5])?;
                check(!chunk.is_empty(), "Empty Scale data chunk")?;
                records[j].extend(chunk);
                check(records[j].len() <= 584, "Oversized Scale record")?;
            }
            Some("END") => {
                check(
                    w.len() == 5
                        && w[2] == "scale"
                        && w[3] == "1752"
                        && hash.as_deref() == Some(w[4]),
                    "Invalid Scale END",
                )?;
                ended = true;
            }
            _ => return Err("Unknown Scale export line".into()),
        }
    }
    check(
        ended && records.iter().all(|r| r.len() == 584),
        "Incomplete Scale export",
    )?;
    check(
        hash.as_deref() == Some(digest(&records.concat()).as_str()),
        "Scale export SHA-256 mismatch",
    )?;
    let reference = reference.ok_or("Missing Scale reference")?;
    for (j, r) in records.iter().enumerate() {
        ScaleRecord::from_arc2(r, j, reference)?;
    }
    Ok(records.map(|r| hex(&r)))
}

//! Full-rate joint scale / Earth / gravity sensitivity reconstruction.
//! Stored temperature reference observations are used unchanged. This is not
//! an ARC archive import, nor independent movement accuracy validation.
mod math;
mod prepare;
mod solve;
#[cfg(test)]
mod tests;

use math::*;
pub use prepare::{Audit, PoseReport, load_recording};
use serde::{Deserialize, Serialize};
pub use solve::reconstruct;
use std::sync::atomic::{AtomicBool, Ordering};

pub const EARTH_RATE_DPS: f64 = 0.0041780746;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LutPoint {
    pub temperature_c: f64,
    pub bias_dps: [f64; 3],
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RuntimeLut {
    pub version: u32,
    pub points: u32,
    pub nodes: Vec<LutPoint>,
}
impl RuntimeLut {
    pub fn parse(text: &str) -> Result<Self, String> {
        require(text.len() <= 128 * 1024, "Runtime LUT exceeds size limit")?;
        let mut result = None;
        let mut count = 0;
        let mut ended = false;
        for line in text.lines().map(str::trim) {
            if line.starts_with("TCAL_LUT valid=") {
                require(result.is_none() && !ended, "Duplicate LUT header")?;
                let fields: std::collections::BTreeMap<_, _> = line
                    .split_whitespace()
                    .skip(1)
                    .filter_map(|s| s.split_once('='))
                    .collect();
                let number = |key| {
                    fields
                        .get(key)
                        .ok_or("Missing LUT header field")?
                        .parse::<u32>()
                        .map_err(|_| "Invalid LUT header")
                };
                require(number("valid")? == 1, "Runtime LUT is not valid")?;
                count = number("count")? as usize;
                require((2..=512).contains(&count), "Invalid LUT node count")?;
                result = Some(Self {
                    version: number("version")?,
                    points: number("points")?,
                    nodes: Vec::new(),
                });
            } else if line == "TCAL_LUT END" {
                require(result.is_some() && !ended, "Unexpected LUT end")?;
                ended = true;
            } else if line.starts_with("LUT ") {
                require(!ended, "LUT row after end")?;
                let lut = result.as_mut().ok_or("LUT row without header")?;
                let fields: Vec<_> = line.split_whitespace().collect();
                require(
                    fields.len() == 6
                        && fields[1].parse::<usize>().ok() == Some(lut.nodes.len())
                        && lut.nodes.len() < count,
                    "Invalid LUT row index/size",
                )?;
                let v: Vec<f64> = fields[2..]
                    .iter()
                    .map(|s| s.parse::<f64>().map_err(|_| "Invalid LUT number"))
                    .collect::<Result<_, _>>()?;
                lut.nodes.push(LutPoint {
                    temperature_c: v[0],
                    bias_dps: [v[1], v[2], v[3]],
                });
            }
        }
        let lut = result.ok_or("Missing runtime LUT")?;
        require(ended && lut.nodes.len() == count, "Incomplete runtime LUT")?;
        lut.validate()?;
        Ok(lut)
    }
    pub fn validate(&self) -> Result<(), String> {
        require(
            (2..=512).contains(&self.nodes.len()) && self.points > 0 && self.points <= 512,
            "Invalid runtime LUT size",
        )?;
        require(
            self.nodes
                .iter()
                .all(|p| p.temperature_c.is_finite() && p.bias_dps.iter().all(|v| v.is_finite()))
                && self
                    .nodes
                    .windows(2)
                    .all(|p| p[0].temperature_c < p[1].temperature_c),
            "Nonfinite or unordered LUT",
        )
    }
    fn at(&self, t: f64) -> Result<V, String> {
        require(
            t.is_finite()
                && t >= self.nodes[0].temperature_c
                && t <= self.nodes[self.nodes.len() - 1].temperature_c,
            "Temperature outside measured runtime LUT",
        )?;
        let i = self
            .nodes
            .partition_point(|p| p.temperature_c < t)
            .max(1)
            .min(self.nodes.len() - 1);
        let (a, b) = (&self.nodes[i - 1], &self.nodes[i]);
        Ok(add(
            a.bias_dps,
            mul(
                sub(b.bias_dps, a.bias_dps),
                (t - a.temperature_c) / (b.temperature_c - a.temperature_c),
            ),
        ))
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ScaleRecord {
    pub reference_id: u32,
    pub epoch: u32,
    pub axis: usize,
    pub initial_raw: [f64; 3],
    pub initial_accel: [f64; 3],
    pub initial_temperature_c: f64,
    pub raw_angle: [f64; 3],
    pub accel_time: [f64; 3],
    pub temperature_time: f64,
    pub duration_s: f64,
    pub cross_angle: f64,
    pub quality: f64,
    pub temperature_min_c: f64,
    pub temperature_max_c: f64,
}
impl ScaleRecord {
    /// ARC2 record CRC, identity, generation and quality are verified here.
    pub fn from_arc2(bytes: &[u8], axis: usize, reference_id: u32) -> Result<Self, String> {
        require(bytes.len() == 584 && axis < 3, "Scale record size/axis")?;
        let u = |i| u32::from_le_bytes(bytes[i..i + 4].try_into().expect("bounded field"));
        let mut crc = !0u32;
        for b in &bytes[..580] {
            crc ^= u32::from(*b);
            for _ in 0..8 {
                crc = (crc >> 1) ^ (0xedb88320u32.wrapping_mul(crc & 1));
            }
        }
        require(!crc == u(580), "Scale CRC mismatch")?;
        require(
            u(0) == 0x41353252
                && u(4) == 2
                && u(8) > 0
                && u(12) == reference_id
                && reference_id > 0
                && bytes[20] == 1
                && bytes[21] as usize == axis,
            "Scale ARC2 identity mismatch",
        )?;
        let f: Vec<_> = (0..22)
            .map(|i| {
                f32::from_le_bytes(
                    bytes[488 + 4 * i..492 + 4 * i]
                        .try_into()
                        .expect("bounded field"),
                ) as f64
            })
            .collect();
        require(
            f.iter().all(|v| v.is_finite()) && bytes[576] as usize == axis && bytes[577] == 20,
            "Scale finite values / 20-turn axis",
        )?;
        let v = |i| [f[i], f[i + 1], f[i + 2]];
        let result = Self {
            reference_id,
            epoch: u(16),
            axis,
            initial_raw: v(0),
            initial_accel: v(3),
            initial_temperature_c: f[6],
            raw_angle: v(7),
            accel_time: v(10),
            temperature_time: f[13],
            duration_s: f[14],
            cross_angle: f[15],
            quality: f[16],
            temperature_min_c: f[17],
            temperature_max_c: f[18],
        };
        result.validate()?;
        Ok(result)
    }
    fn validate(&self) -> Result<(), String> {
        require(
            self.axis < 3
                && self.reference_id > 0
                && (10. ..=600.).contains(&self.duration_s)
                && (0. ..=1100.).contains(&self.cross_angle)
                && (0. ..=0.16).contains(&self.quality),
            "Scale quality gate",
        )?;
        require(
            self.initial_raw
                .iter()
                .chain(self.initial_accel.iter())
                .chain(self.raw_angle.iter())
                .chain(self.accel_time.iter())
                .chain([
                    &self.initial_temperature_c,
                    &self.temperature_time,
                    &self.temperature_min_c,
                    &self.temperature_max_c,
                ])
                .all(|x| x.is_finite())
                && self.temperature_min_c <= self.temperature_max_c,
            "Invalid Scale values",
        )
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Inputs {
    pub lut: RuntimeLut,
    pub scales: [ScaleRecord; 3],
    pub latitude_deg: f64,
    /// Explicit user declaration; temperature reference yaw cannot be inferred.
    pub temperature_reference_is_first_pose: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Coefficients {
    pub scale: [f64; 3],
    pub earth_dps: [f64; 3],
    pub gsens_dps_per_g: [[f64; 3]; 3],
}
impl Coefficients {
    pub fn validate(&self) -> Result<(), String> {
        require(
            self.scale.iter().all(|s| (0.9..=1.1).contains(s))
                && self.earth_dps.iter().all(|x| x.is_finite())
                && (norm(self.earth_dps) - EARTH_RATE_DPS).abs() < 1e-8
                && self
                    .gsens_dps_per_g
                    .iter()
                    .flatten()
                    .all(|v| v.is_finite() && v.abs() <= 0.05),
            "Coefficient physical bounds",
        )
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Solution {
    pub coefficients: Coefficients,
    pub audit: Audit,
    pub poses: Vec<PoseReport>,
    pub iterations: usize,
    pub earth_fit_rms_dps: f64,
    pub earth_holdout_rms_dps: f64,
    pub gsens_fit_rms_dps: f64,
    pub gsens_holdout_rms_dps: f64,
    pub earth_horizontal_ratio: f64,
    pub earth_closure_deg: f64,
    pub gravity_mismatch_deg: Vec<f64>,
    pub installable: bool,
}
fn require(ok: bool, message: &str) -> Result<(), String> {
    if ok { Ok(()) } else { Err(message.into()) }
}
fn cancelled(cancel: &AtomicBool) -> Result<(), String> {
    require(
        !cancel.load(Ordering::Acquire),
        "Joint calibration cancelled",
    )
}

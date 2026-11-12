use super::*;
use crate::capture::{Gyro, Recording, Sample, Track};
use std::{
    collections::BTreeMap,
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};

/// Load only a durably finished, single-track native recording. The general
/// replay loader supports prefix recovery and changing metadata; calibration
/// deliberately refuses those cases rather than hiding their uncertainty.
pub fn load_recording(path: &Path, cancel: &AtomicBool) -> Result<Recording, String> {
    require(
        path.extension()
            .is_some_and(|e| e.eq_ignore_ascii_case("slimecap")),
        "Select a native .slimecap recording",
    )?;
    let recording = crate::capture::format::load(path, cancel)?;
    require(
        recording.tracks.len() == 1,
        "Joint reconstruction requires exactly one track",
    )?;
    require(
        !recording
            .warnings
            .iter()
            .any(|w| w.starts_with("Recovered ")),
        "Incomplete/recovered recordings cannot calibrate a device",
    )?;
    let mut rate = None;
    let mut ended = false;
    let mut line = Vec::new();
    let mut reader = BufReader::new(File::open(path).map_err(|e| e.to_string())?);
    loop {
        cancelled(cancel)?;
        line.clear();
        // The native loader has already checked total size; still bound each
        // read so a concurrent replacement cannot allocate without a limit.
        use std::io::Read;
        let n = reader
            .by_ref()
            .take(1024 * 1024 + 1)
            .read_until(b'\n', &mut line)
            .map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        require(n <= 1024 * 1024, "Oversized native event")?;
        let value: serde_json::Value = serde_json::from_slice(&line).map_err(|e| e.to_string())?;
        require(!ended, "Event after durable end")?;
        require(
            value["event"] != "boundary",
            "Recording contains a session boundary",
        )?;
        if value["event"] == "end" {
            ended = true;
        }
        if let Some(metadata) = value.pointer("/report/Metadata/metadata") {
            let hz = metadata["raw_tx_hz"]
                .as_f64()
                .ok_or("Missing raw sample rate")?;
            if let Some(previous) = rate {
                require(hz == previous, "Sample rate changed within recording")?;
            }
            rate = Some(hz);
        }
    }
    require(
        ended && rate.is_some(),
        "Missing native end or sample clock",
    )?;
    Ok(recording)
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Audit {
    pub unique_samples: usize,
    pub duplicates: usize,
    pub reordered: usize,
    pub missing: usize,
    pub duration_s: f64,
    pub rate_hz: f64,
    pub sequence_wraps: i64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PoseReport {
    pub detected_interval_s: [f64; 2],
    pub cardinal: String,
    pub training_samples: usize,
    pub holdout_samples: usize,
    pub excluded_blocks: usize,
}
#[derive(Clone)]
pub(super) struct Row {
    pub seq: i64,
    pub t: f64,
    pub g: V,
    pub a: V,
    pub temp: f64,
    pub fresh: bool,
}
pub(super) struct Mean {
    pub g: V,
    pub a: V,
    pub table: V,
    pub t: f64,
}
pub(super) struct Prepared {
    pub rows: Vec<Row>,
    pub train: Vec<Mean>,
    pub hold: Vec<Mean>,
    pub poses: Vec<PoseReport>,
    pub motions: Vec<std::ops::Range<usize>>,
    pub audit: Audit,
}
fn same_sample(mut a: Sample, mut b: Sample) -> bool {
    a.receiver_ticks = None;
    b.receiver_ticks = None;
    a.rssi = None;
    b.rssi = None;
    a == b
}
pub(super) fn ordered(track: &Track, cancel: &AtomicBool) -> Result<(Vec<Row>, Audit), String> {
    let hz = f64::from(track.metadata.raw_tx_hz.ok_or("Missing raw sample rate")?);
    require(
        hz.is_finite() && (50. ..=4000.).contains(&hz),
        "Unsupported raw sample rate",
    )?;
    require(
        (2000..=5_000_000).contains(&track.samples.len()),
        "Recording sample count outside limits",
    )?;
    let mut sorted: BTreeMap<i64, Sample> = BTreeMap::new();
    let mut high = None;
    let (mut duplicates, mut reordered) = (0, 0);
    let mut tick = None;
    for s in &track.samples {
        cancelled(cancel)?;
        require(!s.discontinuity, "Discontinuity in recording")?;
        let delta = high.map_or(0, |h: i64| {
            (i64::from(s.sequence) - h.rem_euclid(65536) + 32768).rem_euclid(65536) - 32768
        });
        require(
            (-128..=4096).contains(&delta),
            "Ambiguous sequence/reset/long gap",
        )?;
        let absolute = high.map_or(i64::from(s.sequence), |h| h + delta);
        high = Some(high.map_or(absolute, |h| h.max(absolute)));
        reordered += usize::from(delta < 0);
        let now = s.receiver_ticks.ok_or("Missing receiver clock")?;
        if let Some(previous) = tick {
            require(
                now.wrapping_sub(previous) < 60 * 32768,
                "Ambiguous receiver clock / long outage",
            )?;
        }
        tick = Some(now);
        if let Some(old) = sorted.get(&absolute) {
            require(same_sample(*old, *s), "Conflicting duplicate sample")?;
            duplicates += 1;
        } else {
            sorted.insert(absolute, *s);
        }
    }
    let (&first, first_sample) = sorted.first_key_value().ok_or("Empty recording")?;
    let (&last, last_sample) = sorted.last_key_value().ok_or("Empty recording")?;
    let duration = (last - first) as f64 / hz;
    let ticks = last_sample
        .receiver_ticks
        .unwrap()
        .wrapping_sub(first_sample.receiver_ticks.unwrap());
    require(
        (f64::from(ticks) / 32768. - duration).abs() <= 1. && duration <= 3600.,
        "Sequence and receiver time disagree / recording exceeds one hour",
    )?;
    let audit = Audit {
        unique_samples: sorted.len(),
        duplicates,
        reordered,
        missing: (last - first + 1) as usize - sorted.len(),
        duration_s: duration,
        rate_hz: hz,
        sequence_wraps: last.div_euclid(65536) - first.div_euclid(65536),
    };
    let mut held = None;
    let mut rows = Vec::with_capacity(sorted.len());
    for (seq, s) in sorted {
        let Gyro::RatesDps(g) = s.gyro else {
            return Err("Joint calibration requires raw gyro rates".into());
        };
        let temp = f64::from(s.temperature_c.ok_or("Missing temperature")?);
        require(
            g.iter().all(|v| v.is_finite()) && temp.is_finite(),
            "Nonfinite raw measurements",
        )?;
        if s.acceleration_fresh {
            require(
                s.acceleration_g.iter().all(|v| v.is_finite()),
                "Nonfinite acceleration",
            )?;
            held = Some((s.acceleration_g.map(f64::from), seq));
        }
        // Leading rows before the first fresh acceleration are not silently
        // given zero acceleration. They cannot participate in any fit or move.
        let a = held
            .filter(|(_, last)| (seq - last) as f64 / hz <= 0.1)
            .map_or([f64::NAN; 3], |(a, _)| a);
        rows.push(Row {
            seq,
            t: (seq - first) as f64 / hz,
            g: g.map(f64::from),
            a,
            temp,
            fresh: s.acceleration_fresh,
        });
    }
    Ok((rows, audit))
}
fn mean(rows: &[Row], ids: &[usize], lut: &RuntimeLut) -> Result<Mean, String> {
    require(ids.len() > 1000, "Too few static samples in train/holdout")?;
    let mut g = [0.; 3];
    let mut a = g;
    let mut table = g;
    let mut t = 0.;
    let (mut tmin, mut tmax) = (f64::INFINITY, f64::NEG_INFINITY);
    for &i in ids {
        let r = &rows[i];
        require(
            r.a.iter().all(|v| v.is_finite()),
            "Missing/stale acceleration in static block",
        )?;
        g = add(g, r.g);
        a = add(a, r.a);
        table = add(table, lut.at(r.temp)?);
        t += r.t;
        tmin = tmin.min(r.temp);
        tmax = tmax.max(r.temp);
    }
    let n = ids.len() as f64;
    g = mul(g, 1. / n);
    a = mul(a, 1. / n);
    table = mul(table, 1. / n);
    let mut variance = [0.; 3];
    for &i in ids {
        let d = sub(rows[i].g, g);
        for j in 0..3 {
            variance[j] += d[j] * d[j];
        }
    }
    require(
        variance.iter().all(|v| v.sqrt() / n <= 0.003)
            && tmax - tmin <= 0.5
            && (0.96..=1.04).contains(&norm(a))
            && a.iter().map(|v| v.abs()).fold(0., f64::max) / norm(a) >= 0.9,
        "Static block quality gate (noise, temperature or gravity)",
    )?;
    Ok(Mean {
        g,
        a,
        table,
        t: t / n,
    })
}
fn cardinal(a: V) -> usize {
    let axis = (0..3)
        .max_by(|&i, &j| a[i].abs().total_cmp(&a[j].abs()))
        .unwrap();
    axis * 2 + usize::from(a[axis] < 0.)
}
pub(super) fn prepare(
    track: &Track,
    lut: &RuntimeLut,
    cancel: &AtomicBool,
) -> Result<Prepared, String> {
    let (rows, audit) = ordered(track, cancel)?;
    let index = |t: f64| rows.partition_point(|r| r.t < t);
    let mut ranges = Vec::new();
    let mut start = None;
    for second in 0..=audit.duration_s.floor() as usize {
        cancelled(cancel)?;
        let part = &rows[index(second as f64)..index(second as f64 + 1.)];
        let fresh: Vec<_> = part.iter().filter(|r| r.fresh).collect();
        let am = mul(
            fresh.iter().fold([0.; 3], |a, r| add(a, r.a)),
            1. / fresh.len().max(1) as f64,
        );
        let stable = second < (audit.duration_s.floor() as usize)
            && part.len() as f64 >= audit.rate_hz * 0.9
            && fresh.len() > 20
            && (part.iter().map(|r| dot(r.g, r.g)).sum::<f64>() / part.len().max(1) as f64).sqrt()
                < 3.
            && (0.95..1.05).contains(&norm(am));
        if stable {
            if start.is_none() {
                start = Some(second);
            }
        } else if let Some(s) = start.take()
            && second - s >= 22
        {
            ranges.push((s as f64, second as f64));
        }
    }
    require(
        ranges.len() == 10,
        &format!("Expected 10 stationary poses, detected {}", ranges.len()),
    )?;
    let mut train = Vec::new();
    let mut hold = Vec::new();
    let mut poses = Vec::new();
    for &(s, e) in &ranges {
        let lo = index(s + 2.);
        let hi = index(e - 1.);
        let stable_start = rows[lo].t;
        let stable_end = rows[hi - 1].t;
        let center = mul(
            rows[lo..hi].iter().fold([0.; 3], |a, r| add(a, r.g)),
            1. / (hi - lo) as f64,
        );
        let (mut training, mut validation) = (Vec::new(), Vec::new());
        let mut excluded = 0;
        for block in 0..((stable_end - stable_start) / 5.).floor() as usize {
            let a = index(stable_start + block as f64 * 5.);
            let b = index(stable_start + (block + 1) as f64 * 5.);
            if rows[a..b].iter().any(|r| norm(sub(r.g, center)) > 3.) {
                excluded += 1;
                continue;
            }
            require(
                (b - a) as f64 >= 4.5 * audit.rate_hz,
                "Excessive missing samples in static block",
            )?;
            if block % 2 == 0 {
                training.extend(a..b);
            } else {
                validation.extend(a..b);
            }
        }
        let tr = mean(&rows, &training, lut)?;
        let ho = mean(&rows, &validation, lut)?;
        let face = cardinal(tr.a);
        poses.push(PoseReport {
            detected_interval_s: [s, e],
            cardinal: format!(
                "{}{}",
                if face.is_multiple_of(2) { "+" } else { "-" },
                ["X", "Y", "Z"][face / 2]
            ),
            training_samples: training.len(),
            holdout_samples: validation.len(),
            excluded_blocks: excluded,
        });
        train.push(tr);
        hold.push(ho);
    }
    let first = cardinal(train[0].a);
    require(
        train[..5].iter().all(|m| cardinal(m.a) == first),
        "First five Earth poses must retain the same upward face",
    )?;
    let faces: std::collections::BTreeSet<_> = [0, 5, 6, 7, 8, 9]
        .into_iter()
        .map(|i| cardinal(train[i].a))
        .collect();
    require(faces.len() == 6, "GSens requires all six distinct faces")?;
    let mut motions = Vec::new();
    let up = unit(train[0].a);
    let mut first_sign = 0.;
    for k in 0..9 {
        let range = index(ranges[k].1 - 1.)..rows.partition_point(|r| r.t <= ranges[k + 1].0 + 1.);
        let part = &rows[range.clone()];
        require(
            !part.is_empty() && part.windows(2).all(|p| p[1].seq == p[0].seq + 1),
            "Missing samples in a movement interval",
        )?;
        require(
            part.iter().all(|r| r.a.iter().all(|v| v.is_finite())),
            "Missing/stale acceleration in motion",
        )?;
        if k < 4 {
            let angle = mul(
                part.iter().fold([0.; 3], |a, r| add(a, r.g)),
                1. / audit.rate_hz,
            );
            let along = dot(angle, up);
            if k == 0 {
                first_sign = along.signum();
            }
            require(
                (60. ..=120.).contains(&along.abs())
                    && norm(sub(angle, mul(up, along))) <= 40.
                    && along.signum() == if k < 2 { first_sign } else { -first_sign },
                "Earth motion must follow 0, 90, 180, 90, 0 degrees",
            )?;
        }
        motions.push(range);
    }
    Ok(Prepared {
        rows,
        train,
        hold,
        poses,
        motions,
        audit,
    })
}

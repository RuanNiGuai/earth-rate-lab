use super::{
    prepare::{Mean, Prepared},
    *,
};
use crate::capture::Recording;

struct Geometry {
    up: V,
    h1: V,
    h2: V,
    vertical: f64,
    horizontal: f64,
}
fn replay(
    p: &Prepared,
    c: &Coefficients,
    lut: &RuntimeLut,
    up: V,
    cancel: &AtomicBool,
) -> Result<(Vec<Q>, Vec<f64>), String> {
    let mut quats = vec![Q::ID];
    let mut mismatch = Vec::new();
    let e_over_s = std::array::from_fn(|i| c.earth_dps[i] / c.scale[i]);
    let b0 = sub(
        sub(
            sub(p.train[0].g, p.train[0].table),
            mv(c.gsens_dps_per_g, p.train[0].a),
        ),
        e_over_s,
    );
    for (k, range) in p.motions.iter().enumerate() {
        let mut q = quats[k];
        for i in range.clone() {
            if i % 512 == 0 {
                cancelled(cancel)?;
            }
            let r = &p.rows[i];
            let raw = sub(
                sub(sub(r.g, lut.at(r.temp)?), b0),
                mv(c.gsens_dps_per_g, r.a),
            );
            let corrected = std::array::from_fn(|j| raw[j] * c.scale[j]);
            let v = mul(
                sub(corrected, q.inverse_rotate(c.earth_dps)),
                std::f64::consts::PI / 180. / p.audit.rate_hz,
            );
            q = q.step(v);
        }
        let measured = unit(p.train[k + 1].a);
        let predicted = q.inverse_rotate(up);
        let cosine = dot(predicted, measured).clamp(-1., 1.);
        let angle = cosine.acos().to_degrees();
        require(angle <= 18., "Gravity mismatch exceeds 18 degrees")?;
        let axis = cross(measured, predicted);
        q = q.times(Q([1. + cosine, axis[0], axis[1], axis[2]]).normalized());
        quats.push(q);
        mismatch.push(angle);
    }
    Ok((quats, mismatch))
}
fn earth_design(q: &[Q], means: &[Mean], g: &Geometry, start: f64) -> Vec<Vec<f64>> {
    let mut a = Vec::new();
    for k in 0..5 {
        let h1 = q[k].inverse_rotate(g.h1);
        let h2 = q[k].inverse_rotate(g.h2);
        for j in 0..3 {
            let mut row = vec![0.; 8];
            row[j] = 1.;
            row[j + 3] = (means[k].t - start) / 100.;
            row[6] = h1[j];
            row[7] = h2[j];
            a.push(row);
        }
    }
    a
}
fn earth_target(q: &[Q], means: &[Mean], g: &Geometry, c: &Coefficients) -> Vec<f64> {
    let mut y = Vec::new();
    for k in 0..5 {
        let raw = sub(
            sub(means[k].g, means[k].table),
            mv(c.gsens_dps_per_g, means[k].a),
        );
        let vertical = mul(q[k].inverse_rotate(g.up), g.vertical);
        y.extend((0..3).map(|j| raw[j] * c.scale[j] - vertical[j]));
    }
    y
}
fn gsens_data(q: &[Q], means: &[Mean], c: &Coefficients) -> (Vec<Vec<f64>>, [Vec<f64>; 3]) {
    let mut a = Vec::new();
    let mut y: [Vec<f64>; 3] = std::array::from_fn(|_| Vec::new());
    for k in [0, 5, 6, 7, 8, 9] {
        let m = &means[k];
        a.push(vec![m.a[0], m.a[1], m.a[2], 1.]);
        let earth = q[k].inverse_rotate(c.earth_dps);
        for j in 0..3 {
            y[j].push(m.g[j] - m.table[j] - earth[j] / c.scale[j]);
        }
    }
    (a, y)
}
fn flat(c: &Coefficients) -> Vec<f64> {
    c.scale
        .into_iter()
        .chain(c.earth_dps)
        .chain(c.gsens_dps_per_g.into_iter().flatten())
        .collect()
}

/// Reconstruct from raw observations and verified device inputs. Call on a
/// worker thread. All fits use complete alternating five-second blocks; the
/// held-out means never change the fitted coefficients. Quality failures are
/// errors; holdout failure returns a preview with `installable == false`.
pub fn reconstruct(
    recording: &Recording,
    inputs: &Inputs,
    cancel: &AtomicBool,
) -> Result<Solution, String> {
    cancelled(cancel)?;
    require(
        recording.tracks.len() == 1,
        "Select a single-track recording",
    )?;
    require(
        inputs.temperature_reference_is_first_pose,
        "Confirm the stored temperature reference is Earth pose 1",
    )?;
    require(
        inputs.latitude_deg.is_finite() && inputs.latitude_deg.abs() < 85.,
        "Enter known site latitude (-85 to 85 degrees)",
    )?;
    inputs.lut.validate()?;
    for (j, s) in inputs.scales.iter().enumerate() {
        s.validate()?;
        // Capture epoch advances for each axis; reference_id binds the set.
        require(
            s.axis == j && s.reference_id == inputs.scales[0].reference_id && s.epoch > 0,
            "Scale reference/epoch/axis mismatch",
        )?;
        inputs.lut.at(s.temperature_min_c)?;
        inputs.lut.at(s.temperature_max_c)?;
    }
    let p = prepare::prepare(&recording.tracks[0], &inputs.lut, cancel)?;
    let up = unit(p.train[0].a);
    let axis = (0..3)
        .min_by(|&i, &j| up[i].abs().total_cmp(&up[j].abs()))
        .unwrap();
    let mut guide = [0.; 3];
    guide[axis] = 1.;
    let h1 = unit(sub(guide, mul(up, dot(guide, up))));
    let g = Geometry {
        up,
        h1,
        h2: cross(up, h1),
        vertical: EARTH_RATE_DPS * inputs.latitude_deg.to_radians().sin(),
        horizontal: EARTH_RATE_DPS * inputs.latitude_deg.to_radians().cos(),
    };
    let mut c = Coefficients {
        scale: [1.; 3],
        earth_dps: add(mul(up, g.vertical), mul(h1, g.horizontal)),
        gsens_dps_per_g: [[0.; 3]; 3],
    };
    let mut ecoef = vec![0.; 8];
    let mut gcoef: [Vec<f64>; 3] = std::array::from_fn(|_| vec![0.; 4]);
    let (mut erms, mut grms, mut magnitude, mut change) = (0., 0., 0., f64::INFINITY);
    let mut iterations = 0;
    for iteration in 0..20 {
        cancelled(cancel)?;
        let old = flat(&c);
        for (j, s) in inputs.scales.iter().enumerate() {
            let mut delta = sub(s.raw_angle, mul(s.initial_raw, s.duration_s));
            delta = sub(
                delta,
                mul(
                    sub(
                        inputs.lut.at(s.temperature_time / s.duration_s)?,
                        inputs.lut.at(s.initial_temperature_c)?,
                    ),
                    s.duration_s,
                ),
            );
            delta = sub(
                delta,
                mv(
                    c.gsens_dps_per_g,
                    sub(s.accel_time, mul(s.initial_accel, s.duration_s)),
                ),
            );
            let other = (0..3)
                .filter(|&i| i != j)
                .map(|i| (c.scale[i] * delta[i]).powi(2))
                .sum::<f64>();
            require(
                other <= 7200_f64.powi(2) * 0.1 && (6000. ..=8000.).contains(&delta[j].abs()),
                "Scale angle gate",
            )?;
            c.scale[j] = (7200_f64.powi(2) - other).sqrt() / delta[j].abs();
            require(
                (0.9..=1.1).contains(&c.scale[j]),
                "Scale factor outside 0.9..1.1",
            )?;
        }
        let (q, _) = replay(&p, &c, &inputs.lut, up, cancel)?;
        (ecoef, erms) = least_squares(
            &earth_design(&q, &p.train, &g, p.train[0].t),
            &earth_target(&q, &p.train, &g, &c),
        )?;
        magnitude = ecoef[6].hypot(ecoef[7]);
        require(
            erms <= 0.001 && (0.5 * g.horizontal..=1.5 * g.horizontal).contains(&magnitude),
            "Earth fit / horizontal magnitude gate",
        )?;
        c.earth_dps = add(
            mul(up, g.vertical),
            mul(
                add(mul(g.h1, ecoef[6]), mul(g.h2, ecoef[7])),
                g.horizontal / magnitude,
            ),
        );
        let (q, _) = replay(&p, &c, &inputs.lut, up, cancel)?;
        let (ga, gy) = gsens_data(&q, &p.train, &c);
        grms = 0.;
        for j in 0..3 {
            let (coef, rms) = least_squares(&ga, &gy[j])?;
            c.gsens_dps_per_g[j].copy_from_slice(&coef[..3]);
            gcoef[j] = coef;
            grms += rms * rms / 3.;
        }
        grms = grms.sqrt();
        require(
            grms <= 0.006 && c.gsens_dps_per_g.iter().flatten().all(|x| x.abs() <= 0.05),
            "GSens fit gate",
        )?;
        change = flat(&c)
            .iter()
            .zip(&old)
            .map(|(a, b)| (a - b).abs())
            .fold(0., f64::max);
        iterations = iteration + 1;
        if iteration >= 4 && change < 1e-9 {
            break;
        }
    }
    require(change < 1e-8, "Joint reconstruction did not converge")?;
    c.validate()?;
    let (q, mismatch) = replay(&p, &c, &inputs.lut, up, cancel)?;
    ecoef[6] *= g.horizontal / magnitude;
    ecoef[7] *= g.horizontal / magnitude;
    let eh = residual(
        &earth_design(&q, &p.hold, &g, p.train[0].t),
        &earth_target(&q, &p.hold, &g, &c),
        &ecoef,
    );
    let (ga, gy) = gsens_data(&q, &p.hold, &c);
    let gh = ((0..3)
        .map(|j| residual(&ga, &gy[j], &gcoef[j]).powi(2))
        .sum::<f64>()
        / 3.)
        .sqrt();
    Ok(Solution {
        coefficients: c,
        audit: p.audit,
        poses: p.poses,
        iterations,
        earth_fit_rms_dps: erms,
        earth_holdout_rms_dps: eh,
        gsens_fit_rms_dps: grms,
        gsens_holdout_rms_dps: gh,
        earth_horizontal_ratio: magnitude / g.horizontal,
        earth_closure_deg: q[4].angle(),
        gravity_mismatch_deg: mismatch,
        installable: eh <= 0.001 && gh <= 0.006,
    })
}

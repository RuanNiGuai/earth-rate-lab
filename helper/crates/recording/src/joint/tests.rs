use super::*;
#[test]
fn qr_recovers_coefficients_and_rejects_missing_geometry() {
    let a = vec![
        vec![1., 0., 1.],
        vec![0., 1., 1.],
        vec![-1., 0., 1.],
        vec![0., -1., 1.],
    ];
    let (x, r) = least_squares(&a, &[5., 6., 1., 0.]).unwrap();
    assert!(r < 1e-12);
    for (v, e) in x.iter().zip([2., 3., 3.]) {
        assert!((v - e).abs() < 1e-12);
    }
    assert!(least_squares(&vec![vec![1., 1.]; 4], &[0.; 4]).is_err());
}
#[test]
fn quaternion_preserves_yaw_and_tracks_body_earth() {
    let q = Q::ID.step([std::f64::consts::FRAC_PI_2, 0., 0.]);
    assert!(norm(sub(q.inverse_rotate([0., 1., 0.]), [0., 0., -1.])) < 1e-12);
    assert!((q.angle() - 90.).abs() < 1e-12);
}
#[test]
fn runtime_lut_requires_complete_ordered_finite_data() {
    let text = "TCAL_LUT valid=1 count=2 version=3 points=4\nLUT 0 20 1 2 3\nLUT 1 30 3 4 5\nTCAL_LUT END\n";
    let lut = RuntimeLut::parse(text).unwrap();
    assert_eq!(lut.at(25.).unwrap(), [2., 3., 4.]);
    assert!(lut.at(31.).is_err());
    for bad in [
        text.replace("TCAL_LUT END", ""),
        text.replace("LUT 1", "LUT 0"),
        text.replace("30 3", "19 3"),
        text.replace("30 3", "30 NaN"),
    ] {
        assert!(RuntimeLut::parse(&bad).is_err());
    }
}
#[test]
fn sequence_wrap_reorder_conflict_and_clock_are_checked() {
    use crate::capture::{Gyro, Sample, Track};
    let mut track = Track::default();
    track.metadata.raw_tx_hz = Some(400.);
    for i in 0..3000u32 {
        track.samples.push(Sample {
            sequence: (65000 + i) as u16,
            gyro: Gyro::RatesDps([0.; 3]),
            acceleration_g: [1., 0., 0.],
            acceleration_fresh: true,
            temperature_c: Some(25.),
            receiver_ticks: Some(i * 32768 / 400),
            ..Sample::default()
        });
    }
    let cancel = AtomicBool::new(false);
    let (_, audit) = prepare::ordered(&track, &cancel).unwrap();
    assert_eq!(audit.sequence_wraps, 1);
    track.samples.swap(20, 21);
    // Arrival ticks retain chronological arrival order even for late RF data.
    let ticks = track.samples[20].receiver_ticks;
    track.samples[20].receiver_ticks = track.samples[21].receiver_ticks;
    track.samples[21].receiver_ticks = ticks;
    assert_eq!(prepare::ordered(&track, &cancel).unwrap().1.reordered, 1);
    let mut duplicate = track.samples[2999];
    duplicate.gyro = Gyro::RatesDps([1., 0., 0.]);
    track.samples.push(duplicate);
    assert!(prepare::ordered(&track, &cancel).is_err());
    track.samples.pop();
    track.samples[2000].discontinuity = true;
    assert!(prepare::ordered(&track, &cancel).is_err());
}

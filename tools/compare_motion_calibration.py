"""Identity-bound RAWFIFO motion replay with recorded and installed calibration.

Read-only diagnostics. Fixed return windows are supplied before replay; this
tool never fits a parameter to the return, opens hardware, or authorizes writes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

import numpy as np

from calibration_coordinates import installed_accel
from calwire import unpack_snapshot
from fusion_replay import Replay, orientations
from validate_fusion import load_raw


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_binding(info, metadata, blob):
    """A slot or a matching build alone does not identify a physical sensor."""
    addresses = re.findall(r'^Device address: ([0-9A-Fa-f]{12})\s*$', info, re.M)
    require(len(addresses) == 1, 'Exactly one recorded tracker RF identity required')
    require(metadata.get('rf_address', '').upper() == addresses[0].upper(), 'Recorded/snapshot device identity mismatch')
    require(isinstance(metadata.get('receiver_serial'), str) and bool(metadata['receiver_serial']), 'Missing snapshot receiver identity')
    require(type(metadata.get('slot')) is int and 0 <= metadata['slot'] < 32, 'Missing snapshot slot')
    require(metadata.get('snapshot_sha256') == hashlib.sha256(blob).hexdigest(), 'Snapshot metadata hash mismatch')
    snapshot = unpack_snapshot(blob, require_scales=False)
    require(snapshot['build'] == metadata.get('build'), 'Snapshot build metadata mismatch')
    require(snapshot['accelerometer']['known'] and snapshot['accelerometer']['active_matches_persisted'],
            'Installed acceleration transform must be known and persisted')
    return snapshot


def recorded_scale(text):
    found = re.findall(r'Gyro scale: X ([-+\d.eE]+) Y ([-+\d.eE]+) Z ([-+\d.eE]+); active=0', text)
    require(len(found) == 1, 'One inactive recorded scale status required')
    scale = np.asarray(found[0], dtype=float)
    require(np.isfinite(scale).all() and np.all((scale > .8) & (scale < 1.2)), 'Invalid recorded scale')
    return scale


def compensate(raw, snapshot, rate_hz):
    """Apply (raw - T - M @ a_cal)*S, holding only past fresh accel tags.

    Temperature uses the recorded FIFO temperature and installed LUT directly.
    The original firmware's temperature smoother state is not recorded. This
    is a current-calibration comparison, not an exact historic runtime replay.
    """
    require(np.isfinite(rate_hz) and rate_hz > 0, 'Positive sensor rate required')
    gyro, accel, temp = [np.asarray(raw[k], dtype=float) for k in ('gyro', 'accel', 'temp')]
    n = len(gyro)
    require(gyro.shape == accel.shape == (n, 3) and temp.shape == (n,), 'Aligned sensor arrays required')
    require(np.isfinite(gyro).all() and np.isfinite(temp).all(), 'Nonfinite gyro/temperature')
    fresh = np.isfinite(accel).all(axis=1)
    require(np.all(fresh | np.isnan(accel).all(axis=1)) and fresh.sum() >= 2, 'Invalid acceleration freshness')
    ix = np.flatnonzero(fresh)
    require(ix[0] < 4 and n-1-ix[-1] < 4 and np.all(np.diff(ix) == 4), 'Missing/irregular fresh acceleration tags')
    calibrated = np.full_like(accel, np.nan)
    calibrated[fresh] = installed_accel(accel[fresh], snapshot)
    latest = np.maximum.accumulate(np.where(fresh, np.arange(n), -1))
    # The sensor owner processes gyro calibration before processing the accel
    # tag from the same FIFO packet. Match that ordering for M as well.
    last = np.r_[-1, latest[:-1]]
    available = last >= 0
    held = np.zeros_like(accel)
    held[available] = calibrated[last[available]]
    require(np.all((np.arange(n)[available]-last[available])/rate_hz <= .1), 'Stale acceleration')
    lut = np.asarray(snapshot['lut'], dtype=float)
    points = np.asarray(snapshot['thermal_points'], dtype=float)
    observed = points[points[:, 0] != 0, 0]
    require(lut.ndim == 2 and lut.shape[1] == 5 and len(lut) >= 2 and np.isfinite(lut).all()
            and np.all(np.diff(lut[:, 1]) > 0), 'Invalid installed temperature LUT')
    require(len(observed) >= 2 and temp.min() >= observed.min() and temp.max() <= observed.max()
            and temp.min() >= lut[0, 1] and temp.max() <= lut[-1, 1], 'Temperature outside observed calibration support')
    thermal = np.column_stack([np.interp(temp, lut[:, 1], lut[:, j]) for j in (2, 3, 4)])
    scale = np.asarray(snapshot['production']['scale'])
    matrix = np.asarray(snapshot['production']['gsens_dps_per_g']).reshape(3, 3)
    return (gyro - thermal - held @ matrix.T) * scale, calibrated


def endpoint_mask(raw, rate_hz, span):
    require(len(span) == 2 and np.isfinite(span).all() and 0 <= span[0] < span[1]
            and span[1]-span[0] >= 5, 'Return windows must last at least five seconds')
    time = np.arange(len(raw['gyro']))/rate_hz
    require(span[1] <= len(time)/rate_hz, 'Return window outside recorded support')
    mask = (time >= span[0]) & (time < span[1])
    accel = raw['accel'][mask]
    accel = accel[np.isfinite(accel).all(axis=1)]
    gyro = raw['gyro'][mask]
    require(len(accel) >= 10 and np.max(np.std(gyro, axis=0)) < .1, 'Return window contains gyro motion')
    norms = np.linalg.norm(accel, axis=1)
    require(abs(np.mean(norms)-1) < .1 and np.std(norms) < .02, 'Return window acceleration inconsistent with rest')
    return mask


def return_metrics(output, pre, post):
    q = orientations(output)
    delta = q[post].mean() * q[pre].mean().inv()
    return dict(return_deg=float(np.rad2deg(delta.magnitude())),
                world_rotvec_deg=np.rad2deg(delta.as_rotvec()).tolist(),
                pre_bias_dps=output[pre, 4:7].mean(axis=0).tolist(),
                post_bias_dps=output[post, 4:7].mean(axis=0).tolist())


def compare(config_path, out):
    config_path = Path(config_path).resolve()
    spec = json.loads(config_path.read_text(encoding='utf8'))
    require(spec.get('schema') == 'RAW_MOTION_COMPARE1', 'Unsupported motion comparison input')
    require(spec.get('physical_return_asserted') is True, 'Explicit physical return assertion required')
    paths = {}
    for key, entry in spec['files'].items():
        path = (config_path.parent / entry['path']).resolve()
        require(digest(path) == entry['sha256'], 'Input hash mismatch: ' + key)
        paths[key] = path
    required = {'serial', 'csv', 'summary', 'progress', 'info', 'recorded_scale', 'snapshot', 'metadata'}
    require(set(paths) == required, 'Complete explicit recording/calibration evidence required')
    capture = paths['serial'].parent
    require(paths['serial'].name == 'serial.txt' and paths['csv'] == capture/'decoded/samples.csv'
            and paths['summary'] == capture/'decoded/summary.json' and paths['progress'] == capture/'progress.json',
            'Capture inputs do not refer to one recording')
    require(paths['info'].parent == capture.parent and paths['recorded_scale'].parent == capture.parent,
            'Identity/scale evidence must be from the recording session')
    progress = json.loads(paths['progress'].read_text(encoding='utf8'))
    require(progress.get('cleanup_confirmed') is True and progress.get('device_serial') == spec.get('recorded_usb_serial')
            and bool(spec.get('recorded_usb_serial')), 'Recording device/cleanup evidence mismatch')
    metadata = json.loads(paths['metadata'].read_text(encoding='utf8'))
    snapshot = validate_binding(paths['info'].read_text(encoding='utf8'), metadata, paths['snapshot'].read_bytes())
    old_scale = recorded_scale(paths['recorded_scale'].read_text(encoding='utf8'))
    raw = load_raw(capture)
    rate = float(spec['gyro_rate_hz'])
    require(np.isfinite(rate) and 100 <= rate <= 4000, 'Invalid documented gyro rate')
    measured_rate = (len(raw['gyro'])-1)/(raw['batch_time'][-1]-raw['batch_time'][0])
    require(abs(measured_rate/rate-1) < .001, 'Documented rate disagrees with sensor batch timeline')
    pre = endpoint_mask(raw, rate, spec['before_s'])
    post = endpoint_mask(raw, rate, spec['after_s'])
    require(spec['before_s'][1] < spec['after_s'][0], 'Return windows must be ordered')
    gyro, accel = compensate(raw, snapshot, rate)
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    engine = Replay(out/'native', iir='production')
    cmsis = Replay(out/'native', iir='production', math_backend='cmsis')
    cases = {
        'recorded_scale_only': (raw['gyro']*old_scale, raw['accel'], engine, {}),
        'installed_scale_only': (raw['gyro']*snapshot['production']['scale'], raw['accel'], engine, {}),
        'installed_full': (gyro, accel, engine, {}),
        'installed_full_cmsis': (gyro, accel, cmsis, {}),
        'installed_full_no_motion_bias': (gyro, accel, engine, dict(motion_mode=1)),
        'installed_full_no_rate_gate': (gyro, accel, engine, dict(accel_gate=0)),
    }
    report = dict(schema='RAW_MOTION_COMPARISON1', input_sha256=digest(config_path),
        sources={key: digest(path) for key, path in paths.items()},
        installed_build=snapshot['build'], device_binding_verified=True,
        gyro_rate_hz=rate, measured_batch_rate_hz=measured_rate,
        before_s=spec['before_s'], after_s=spec['after_s'], samples=len(raw['gyro']),
        device_writes=0, parameter_fitting=False, independent_physical_accuracy_validated=False,
        cases={}, endpoint_sensitivity=[], engine_sources=engine.hashes,
        engine_library_sha256=digest(engine.library_path), cmsis_library_sha256=digest(cmsis.library_path))
    traces = dict(time=np.arange(len(gyro))/rate, temp=raw['temp'])
    for name, (g, a, replay, policy) in cases.items():
        output = replay.run(g, a, np.full(len(g), 1/rate), raw_accel_stride=4, **policy)
        report['cases'][name] = return_metrics(output, pre, post)
        traces[name] = output
        print(name, report['cases'][name]['return_deg'], flush=True)
    for before_shift in (-10, -5, 0):
        for after_shift in (-25, -12, 0):
            spans = [[v+before_shift for v in spec['before_s']], [v+after_shift for v in spec['after_s']]]
            try:
                masks = [endpoint_mask(raw, rate, span) for span in spans]
            except ValueError as error:
                report['endpoint_sensitivity'].append(dict(before_s=spans[0], after_s=spans[1], rejected=str(error)))
                continue
            report['endpoint_sensitivity'].append(dict(before_s=spans[0], after_s=spans[1],
                return_deg=return_metrics(traces['installed_full'], *masks)['return_deg']))
    report['limitations'] = [
        'Historic raw recording, replayed with a newer calibration of the same physical RF identity. No new physical accuracy test.',
        'Recorded-scale-only omits the historic thermal LUT; static bias is learned causally. It is a diagnostic comparator, not exact old firmware output.',
        'Current compensation uses recorded FIFO temperature and installed LUT, not the unrecorded firmware temperature smoother state.',
        'Acceleration freshness is preserved. M uses the previous processed acceleration; gyro precedes accel in each FIFO packet. No future interpolation.',
        'Earth is disabled in all cases because the historic geographic heading is not verified. No current Earth vector is invented.',
        'Endpoint low variance and gravity consistency cannot verify matching yaw. Physical return is an operator assertion.',
        'Sensor rate is the documented nominal sender rate; batch arrival times provide only a rate consistency check.',
        'No endpoint-derived scale, future bias correction, quaternion reset or parameter selection is applied.'
    ]
    np.savez_compressed(out/'traces.npz', **traces)
    (out/'comparison.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    compare(args.input, args.out)


if __name__ == '__main__':
    main()

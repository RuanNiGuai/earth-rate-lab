"""Matched component ablation on a frozen ten-device calibration recording.

This replays the LOCAL fusion core. Earth-off is a counterfactual, not an
upstream firmware emulator. No refitting, parameter selection or device I/O.
The output contains hashes and anonymous metrics, never writable coefficients.
"""
import argparse
import copy
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fusion_replay import Replay, metrics, resample_windows
import inputs
from calibration_coordinates import installed_accel
import joint_rigid_v3 as joint

PROFILES = {
    'all_on': frozenset('TSME'),
    'earth_off': frozenset('TSM'),
    'gsens_off': frozenset('TSE'),
    'earth_gsens_off': frozenset('TS'),
    'temperature_off': frozenset('SME'),
    'scale_off': frozenset('TME'),
    'all_off': frozenset(),
}


def corrected(gyro, accel, thermal, parameters, enabled):
    """Keep the firmware's order: subtract T and M*a, then multiply by S.

    E is deliberately NOT subtracted here: the paired fusion core owns its
    projection and bias-state handling. S=1 means no scale compensation.
    """
    if not set(enabled) <= set('TSME'):
        raise ValueError('Unknown component')
    gyro, accel, thermal = [np.asarray(x, dtype=float) for x in (gyro, accel, thermal)]
    if gyro.ndim != 2 or gyro.shape[1] != 3 or accel.shape != gyro.shape or thermal.shape != gyro.shape:
        raise ValueError('Expected matching N by 3 observations')
    s = np.asarray(parameters['scale'], dtype=np.float32).astype(float)
    m = np.asarray(parameters['gsens_dps_per_g'], dtype=np.float32).astype(float).reshape(3, 3)
    if s.shape != (3,) or not all(np.isfinite(x).all() for x in (gyro, accel, thermal, s, m)):
        raise ValueError('Nonfinite observations or invalid coefficients')
    return (gyro - (thermal if 'T' in enabled else 0)
            - (accel @ m.T if 'M' in enabled else 0)) * (s if 'S' in enabled else 1)


def recordings(data, parameters, enabled):
    result = []
    for slot, (d, values) in enumerate(zip(data, parameters)):
        e = np.asarray(values['earth_dps'], dtype=np.float32).astype(float)
        if e.shape != (3,) or not np.isfinite(e).all():
            raise ValueError('Invalid Earth vector')
        for name, rec in [('poses', d)] + [(name, d['scale_captures'][k]) for k, name in enumerate('xyz')]:
            accel = installed_accel(rec['accel'],d['backup'],allow_unknown=True)
            g = corrected(rec['gyro'], accel, joint.old.table(d, rec['temp']), values, enabled)
            duration = rec['duration'] * rec['clock_rate']
            error = float(np.ptp(rec['interval_end'] - np.cumsum(duration)))
            if error > .025:
                raise ValueError('Sensor timeline diverges from accumulated intervals')
            t, g, a, dt = resample_windows(g, accel, duration, rec['interval_end'][0] - duration[0])
            result.append(dict(device=f'T{slot+1:02d}', kind=name, time=t, gyro=g, accel=a, dt=dt,
                               earth=e.copy() if name == 'poses' and 'E' in enabled else np.zeros(3),
                               rests=joint.old.SEG if name == 'poses' else rec['rests']))
    return result


def statistics(values):
    a = np.asarray(values, dtype=float)
    if a.size == 0 or not np.isfinite(a).all():
        raise ValueError('Empty or nonfinite metrics')
    return dict(count=int(a.size), rms=float(np.sqrt(np.mean(a*a))), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), maximum=float(a.max()))


def summarize(rows):
    groups = dict(earth_first_five_rest_deg_per_min=[], poses_rest_deg_per_min=[],
                  axes_rest_deg_per_min=[], poses_return_deg=[], axes_return_deg=[])
    for row in rows:
        if row['kind'] == 'poses':
            groups['earth_first_five_rest_deg_per_min'] += row['rest_drift_deg_per_min'][:5]
            groups['poses_rest_deg_per_min'] += row['rest_drift_deg_per_min']
            groups['poses_return_deg'] += row['closure_deg']
        else:
            groups['axes_rest_deg_per_min'] += row['rest_drift_deg_per_min']
            groups['axes_return_deg'] += row['closure_deg']
    return {name: statistics(values) for name, values in groups.items()}


def evaluate(engine, records):
    rows = []
    for r in records:
        out = engine.run(r['gyro'], r['accel'], r['dt'], r['earth'], substeps=8)
        row = metrics(r['time'], out, r['rests'])
        # Without the optional final return, the last pose is a different face.
        if r['kind'] == 'poses' and len(r['rests']) == 10:
            row['closure_deg'] = row['closure_deg'][:1]
        row.update(device=r['device'], kind=r['kind'], windows=len(r['time']),
                   duration_s=float(r['dt'].sum()), earth_active_fraction=float(np.mean(out[:, 9])))
        expected_earth = bool(np.any(r['earth']))
        if expected_earth and row['earth_active_fraction'] < .99:
            raise ValueError('Earth-on arm did not activate')
        if not expected_earth and np.any(out[:, 9]):
            raise ValueError('Earth-off arm unexpectedly activated')
        rows.append(row)
    return dict(summary=summarize(rows), devices=[
        dict(device=f'T{i+1:02d}', metrics=summarize([r for r in rows if r['device'] == f'T{i+1:02d}']))
        for i in range(10)], recordings=rows)


def contrast(before, after):
    result = {}
    for key, value in before['summary'].items():
        a, b = value['rms'], after['summary'][key]['rms']
        changes = [x['metrics'][key]['rms'] - y['metrics'][key]['rms']
                   for x, y in zip(before['devices'], after['devices'])]
        result[key] = dict(disabled_rms=a, enabled_rms=b, absolute_reduction=a-b,
                           reduction_percent=(a-b)/a*100 if a else None,
                           devices_improved=sum(x > 1e-9 for x in changes),
                           devices_worsened=sum(x < -1e-9 for x in changes),
                           devices_tied=sum(abs(x) <= 1e-9 for x in changes))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--selection', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    config = inputs.configure(args.input, joint.old)
    data = inputs.load()
    fit = json.loads(args.result.read_text(encoding='utf8'))
    selection = json.loads(args.selection.read_text(encoding='utf8'))
    inputs.require(config['dataset_kind'] == 'measured' and config.get('transport_pass') is True,
                   'Complete measured recordings required')
    inputs.require(not config.get('transport_diagnostic_override'), 'Diagnostic transport override is not accepted')
    inputs.require(selection['input_sha256'] == inputs.digest(args.input)
                   and selection['result_sha256'] == inputs.digest(args.result), 'Frozen parameter/input mismatch')
    inputs.require(selection['snapshot_hashes'] == [d['backup']['snapshot_sha256'] for d in data],
                   'Frozen parameter/snapshot mismatch')
    inputs.require([r['slot'] for r in selection['decisions']] == list(range(10)), 'Incomplete parameter fleet')
    parameters = [copy.deepcopy(r['values']) for r in selection['decisions']]
    inputs.require(all(np.isfinite(np.r_[p['scale'], p['gsens_dps_per_g'], p['earth_dps']]).all()
                       for p in parameters), 'Nonfinite frozen parameters')
    inputs.require(fit['input_config']['config_sha256'] == inputs.digest(args.input), 'Fit/input mismatch')
    engine = Replay(args.out/'native')
    runs = {}
    for label, enabled in PROFILES.items():
        print('Replaying', label, flush=True)
        runs[label] = evaluate(engine, recordings(data, parameters, enabled))
        (args.out/(label+'.json')).write_text(json.dumps(runs[label], indent=2)+'\n', encoding='utf8')
    comparisons = {name+'_to_all_on': contrast(runs[name], runs['all_on'])
                   for name in PROFILES if name != 'all_on'}
    # Factorial control: Earth also compared with M off on both sides.
    comparisons['earth_gsens_off_to_gsens_off'] = contrast(runs['earth_gsens_off'], runs['gsens_off'])
    report = dict(schema='MATCHED_COMPONENT_ABLATION1', dataset_kind='measured',
        baseline_kind='local-core component counterfactual, not upstream firmware execution',
        input_sha256=inputs.digest(args.input), result_sha256=inputs.digest(args.result),
        selection_sha256=inputs.digest(args.selection), evaluator_sha256=inputs.digest(__file__),
        replay_source_sha256=engine.hashes, refitted=False, parameter_selection_performed=False,
        components={k:sorted(v) for k,v in PROFILES.items()}, substeps=8, gyro_hz=400, accel_hz=100,
        earth_enabled_only_in='poses with known initial heading; off in X/Y/Z in ALL profiles',
        initial_bias='identical VQF cold initialization and continuous online learning; no offline per-arm bias fit',
        parameters='same frozen recommended S/M/E in every enabled arm, quantized to float32',
        conclusions_scope='conditional within-recording ablation; not independent accuracy or upstream performance',
        runs=runs, comparisons=comparisons, device_writes=0,
        limitations=['F04 repeated means do not reconstruct raw high-frequency samples.',
                     'Installed acceleration matrix and MCU boot-bias handoff are absent.',
                     'A local temperature lookup is used, not the current upstream temperature implementation.',
                     'All variants use the same local VQF core, not the upstream experimental configuration.',
                     'Calibration used these recordings; returns were manual without external angle truth.',
                     'Missing physical restart, independent motion, thermal-cycle and overnight evidence.'])
    (args.out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf8')
    print(json.dumps(comparisons['earth_off_to_all_on'], indent=2), flush=True)


if __name__ == '__main__':
    main()

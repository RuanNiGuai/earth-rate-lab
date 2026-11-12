"""Prepare hashed measured inputs from a passive F04 recording and CALWIRE backups."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from inputs import digest, require
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from f04_summary import EndManifest, decode_frame_report
from capture_quality import audit_recording


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('recording', 'clocks', 'segments', 'snapshots', 'out'):
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--latitude', type=float, required=True)
    p.add_argument('--allow-legacy-transport', action='store_true',
                   help='Diagnostic use of old/incomplete recordings; never grants installation approval')
    a = p.parse_args()
    clocks = {r['tracker']: r for r in json.loads(a.clocks.read_text())['clock_models']}
    segments = json.loads(a.segments.read_text())['static_segments_s']
    require(set(clocks) == set(range(10)), 'Clock models must cover slots 0..9')
    transport = audit_recording(a.recording)
    require(transport['targets'] == list(range(10)), 'Frozen fleet must cover slots 0..9')
    require(transport['transport_pass'] or a.allow_legacy_transport,
            'Transport incomplete: inspect capture quality or use --allow-legacy-transport for diagnostics only')
    seen = [{} for _ in range(10)]
    identities = [set() for _ in range(10)]
    duplicates = [0] * 10
    with a.recording.open(encoding='utf8') as stream:
        for line in stream:
            row = json.loads(line)
            if 'report_hex' not in row:
                continue
            raw = bytes.fromhex(row['report_hex'])
            s, _, _ = decode_frame_report(raw)
            if isinstance(s, EndManifest):
                continue
            require(0 <= s.tracker < 10, 'Unexpected tracker')
            identities[s.tracker].add((s.boot, s.operation))
            require(len(identities[s.tracker]) == 1, 'Restart/session change: split recording first')
            old = seen[s.tracker].get(s.sequence)
            if old is not None:
                require(old == s, 'Conflicting duplicate summary')
                duplicates[s.tracker] += 1
            else:
                seen[s.tracker][s.sequence] = s
    a.out.mkdir(parents=True, exist_ok=False)
    config = dict(schema='JOINT10_INPUT1', dataset_kind='measured', latitude_deg=a.latitude,
                  static_segments_s=segments, recording=str(a.recording.resolve()),
                  recording_sha256=digest(a.recording), devices=[])
    config['transport_pass'] = transport['transport_pass']
    config['transport_diagnostic_override'] = a.allow_legacy_transport
    integrity = []
    for i in range(10):
        rows = sorted(seen[i].values(), key=lambda s: s.sequence)
        require(len(rows) > 100, 'Missing or short tracker stream')
        clock = clocks[i]
        rate = 1 + clock['clock_rate_correction_ppm'] * 1e-6
        end = np.array([((s.end_us - rows[0].end_us) & 0xffffffff) / 1e6 for s in rows])
        end = end * rate + clock['offset_seconds']
        duration = np.array([s.duration_us / 1e6 for s in rows])
        require(np.all(duration > 0), 'Zero-duration summary')
        arrays = a.out / f'tracker-{i}.npz'
        np.savez_compressed(arrays, time=end-duration*rate/2, interval_end=end, duration=duration,
                            gyro=np.array([np.array(s.gyro_delta_udeg)/s.duration_us for s in rows]),
                            accel=np.array([s.accel_mean_mg for s in rows])/1000,
                            temp=np.array([s.temperature_centi_c for s in rows])/100,
                            seq=np.array([s.sequence for s in rows]))
        snapshot = a.snapshots / f'slot-{i}/snapshot.bin'
        config['devices'].append(dict(slot=i, arrays=arrays.name, arrays_sha256=digest(arrays),
            snapshot=str(snapshot.resolve()), snapshot_sha256=digest(snapshot),
            clock_rate_correction_ppm=clock['clock_rate_correction_ppm']))
        integrity.append(dict(slot=i, unique=len(rows), duplicates=duplicates[i],
                              missing_within_observed_sequence=rows[-1].sequence-rows[0].sequence+1-len(rows)))
    (a.out/'input.json').write_text(json.dumps(config, indent=2), encoding='utf8')
    (a.out/'integrity.json').write_text(json.dumps(integrity, indent=2), encoding='utf8')
    (a.out/'transport-quality.json').write_text(json.dumps(transport, indent=2), encoding='utf8')
    print(a.out/'input.json')


if __name__ == '__main__':
    main()

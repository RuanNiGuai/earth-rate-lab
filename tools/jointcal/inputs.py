"""Explicit, hashed inputs for the ten-tracker diagnostic. No device I/O."""
import hashlib
import json
from pathlib import Path
import struct
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calwire import unpack_snapshot

_config = None


def require(ok, message):
    if not ok:
        raise ValueError(message)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def configure(path, backend):
    global _config
    path = Path(path).resolve()
    c = json.loads(path.read_text(encoding='utf-8-sig'))
    require(c.get('schema') == 'JOINT10_INPUT1', 'Unsupported input schema')
    require(c.get('dataset_kind') in ('measured', 'synthetic'), 'Specify dataset_kind')
    lat = float(c['latitude_deg'])
    require(np.isfinite(lat) and abs(lat) < 89.0, 'Latitude outside supported non-polar range')
    segments = np.asarray(c['static_segments_s'], dtype=float)
    count=11 if c.get('final_return_present') is True else 10
    require(segments.shape == (count, 2) and np.isfinite(segments).all(), 'Ten static segments, optionally followed by an explicit final return, required')
    require(np.all(segments[:, 1] - segments[:, 0] >= 12), 'Static segments must last at least 12 s')
    require(np.all(segments[1:, 0] > segments[:-1, 1] + .5), 'Static segments must be ordered with intervening motion')
    devices = c['devices']
    require(len(devices) == 10 and [d['slot'] for d in devices] == list(range(10)), 'Exactly slots 0..9 required')
    for d in devices:
        require(np.isfinite(d['clock_rate_correction_ppm']) and abs(d['clock_rate_correction_ppm']) < 1000,
                'Invalid initial clock rate')
        for field in ('arrays', 'snapshot'):
            p = (path.parent / d[field]).resolve()
            require(digest(p) == d[field + '_sha256'], 'Input hash mismatch: ' + field)
            d[field] = p
    recording = (path.parent / c['recording']).resolve()
    require(digest(recording) == c['recording_sha256'], 'Recording hash mismatch')
    for recording in c.get('recordings', []):
        require(digest(recording['path']) == recording['sha256'], 'Capture recording hash mismatch')
    if c.get('scale_captures'):
        require([r['axis'] for r in c['scale_captures']] == ['x','y','z'], 'Three continuous axes required')
        require([r['kind'] for r in c['recordings']] == ['x','y','z','poses'], 'Four source recordings required')
        for capture in c['scale_captures']:
            segments_axis=np.asarray(capture['static_segments_s'],dtype=float)
            require(segments_axis.shape==(3,2) and np.isfinite(segments_axis).all()
                    and np.all(segments_axis[:,1]-segments_axis[:,0]>=12)
                    and np.all(segments_axis[1:,0]>segments_axis[:-1,1]), 'Invalid axis rests')
            require(len(capture['devices'])==10,'Incomplete continuous scale fleet')
            for entry in capture['devices']:
                p=(path.parent/entry['arrays']).resolve()
                require(digest(p)==entry['arrays_sha256'],'Scale arrays hash mismatch')
                entry['arrays']=p
    c['public_description'] = dict(schema=c['schema'], dataset_kind=c['dataset_kind'],
                                  latitude_deg=lat, static_segments_s=segments.tolist(),
                                  config_sha256=digest(path), device_count=10)
    backend.SEG = segments.tolist()
    backend.MOV = list(zip(segments[:-1, 1], segments[1:, 0]))
    backend.LATITUDE_DEG = lat
    _config = c
    return c


def load():
    require(_config is not None, 'Call configure with an explicit input file')
    data = []
    for i,spec in enumerate(_config['devices']):
        b = unpack_snapshot(spec['snapshot'].read_bytes(), require_scales=not bool(_config.get('scale_captures')))
        b['snapshot_sha256'] = spec['snapshot_sha256']
        with np.load(spec['arrays'], allow_pickle=False) as archive:
            d = {k: archive[k] for k in ('time', 'interval_end', 'duration', 'gyro', 'accel', 'temp', 'seq')}
        n = len(d['time'])
        require(n > 100, 'Too few measured windows')
        for k, value in d.items():
            shape = (n, 3) if k in ('gyro', 'accel') else (n,)
            require(value.shape == shape and np.isfinite(value).all(), 'Invalid array: ' + k)
        require(np.all(np.diff(d['time']) > 0) and len(np.unique(d['seq'])) == n,
                'Non-monotonic or duplicate windows; split restarts and deduplicate first')
        require(np.all((d['duration'] > 0) & (d['duration'] < 1)), 'Invalid window duration')
        d['backup'] = b
        d['lut'] = np.array(b['lut'])
        d['M'] = np.array(b['production']['gsens_dps_per_g']).reshape(3, 3)
        d['S'] = np.array(b['production']['scale'])
        d['clock_rate'] = 1 + spec['clock_rate_correction_ppm'] * 1e-6
        d['interval_duration'] = d['duration'] * d['clock_rate']
        d['records'] = [struct.unpack_from('<22f4B', bytes.fromhex(x), 488) for x in b['scale_records_hex']]
        if _config.get('scale_captures'):
            d['scale_captures']=[]
            for capture in _config['scale_captures']:
                entry=capture['devices'][i]
                with np.load(entry['arrays'],allow_pickle=False) as archive:
                    row={k:archive[k] for k in ('time','interval_end','duration','gyro','accel','temp')}
                size=len(row['time'])
                require(size>100 and np.all(np.diff(row['time'])>0),'Invalid continuous scale time')
                for k,value in row.items():
                    require(value.shape==((size,3) if k in ('gyro','accel') else (size,))
                            and np.isfinite(value).all(),'Invalid continuous scale '+k)
                require(np.all((row['duration']>0)&(row['duration']<1)),'Invalid scale interval')
                row['rests']=capture['static_segments_s']
                row['clock_rate']=1+entry['clock_rate_correction_ppm']*1e-6
                require(np.isfinite(row['clock_rate']) and abs(row['clock_rate']-1)<.001,'Invalid axis clock rate')
                d['scale_captures'].append(row)
        data.append(d)
    return data

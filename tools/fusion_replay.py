"""Measured-window replay of the production VQF/Earth C core; no device I/O.

Window means cannot reconstruct original 400/100 Hz samples or coning. The
optional substeps repeat the same observation to audit filter-rate sensitivity.
All parameter comparisons must report this limitation and retain motion tests.
"""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation
from native_test import compiler

ROOT = Path(sys._MEIPASS)/'toolkit' if getattr(sys,'frozen',False) else Path(__file__).resolve().parents[1]
CORE = ROOT / 'integration/tracker/vqf-c/src'
FUSION = ROOT / 'integration/tracker/src/sensor/fusion'

# Host experiments, not firmware defaults or permission to install. Baseline
# retains the production policy; conservative does not speed up rest learning.
POLICIES = {
    'baseline': {},
    'fast_rest': {'rest_sigma': .005},
    'fast_rest_guard': {'rest_sigma': .005, 'rest_guard_dps': .03},
    'fast_rest_guard_norm': {'rest_sigma': .005, 'rest_guard_dps': .03, 'accel_gate': 3},
    'slow_anchor': {'rest_sigma': .005, 'rest_guard_dps': .03, 'rest_anchor_tau': 60, 'accel_gate': 3},
    'conservative': {'rest_guard_dps': .03, 'rest_anchor_tau': 60, 'accel_gate': 3},
    'adaptive_rest': {'adaptive_rest': True},
}


def extract_function(source, name):
    match = re.search(r'^(?:static )?(?:void|bool|vqf_real_t) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    if not match:
        raise ValueError('Missing production function: ' + name)
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end] + '\n'


class Replay:
    def __init__(self, build, iir='production'):
        build = Path(build).resolve()
        build.mkdir(parents=True, exist_ok=True)
        wrapper = (FUSION / 'vqf/vqf.c').read_text(encoding='utf8')
        functions = '\n'.join(extract_function(wrapper, n) for n in
                              ('set_params', 'motion_bias_gate', 'accel_observation_trusted'))
        macros = '\n'.join(line for line in wrapper.splitlines() if line.startswith('#define ') and
                           any(line.startswith('#define ' + key) for key in
                               ('MBE_GATE_', 'MBE_GYRO_GATE_', 'ACCEL_UPDATE_', 'ACCEL_RATE_', 'ACCEL_GATE_RECOVERY_')))
        generated = HEADER + macros + '\n' + functions + (ROOT/'tools/fusion_replay.c').read_text(encoding='utf8')
        sources = [CORE / 'vqf.c', CORE / 'vqf.h', FUSION / 'earth_pair.c', FUSION / 'earth_pair.h', FUSION / 'vqf/vqf.c', ROOT/'tools/fusion_replay.py', ROOT/'tools/fusion_replay.c']
        if iir not in ('production','stable'):raise ValueError('Unknown IIR realization')
        core_source=(CORE/'vqf.c').read_text(encoding='utf8')
        pair_source=(FUSION/'earth_pair.c').read_text(encoding='utf8')
        if iir=='stable':
            from fusion_stable_iir import transform
            core_source,pair_source=transform(core_source,pair_source,extract_function)
            sources.append(ROOT/'tools/fusion_stable_iir.py')
        self.hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
        self.iir=iir
        key = hashlib.sha256(generated.encode() + core_source.encode() + pair_source.encode() + json.dumps(self.hashes, sort_keys=True).encode()).hexdigest()[:16]
        source = build / ('replay-' + key + '.c')
        lib = build / ('replay-' + key + ('.dll' if os.name == 'nt' else '.so'))
        bundled = ROOT/'native/replay-manifest.json'
        if bundled.exists():
            entry=json.loads(bundled.read_text(encoding='utf8'))[iir]
            lib=ROOT/'native'/entry['library']
            if entry['sources']!=self.hashes or hashlib.sha256(lib.read_bytes()).hexdigest()!=entry['sha256']:
                raise ValueError('Bundled replay does not match its source manifest; reinstall the complete component')
        elif getattr(sys,'frozen',False):
            raise ValueError('Missing bundled replay engine; reinstall the complete component')
        if not lib.exists():
            source.write_text(generated, encoding='utf8')
            core_path=build/('core-'+key+'.c');core_path.write_text(core_source,encoding='utf8')
            pair_path=build/('pair-'+key+'.c');pair_path.write_text(pair_source,encoding='utf8')
            cc = compiler()
            env = dict(os.environ, PATH=str(Path(cc).parent) + os.pathsep + os.environ.get('PATH', ''), TEMP=str(build), TMP=str(build))
            command = [cc, '-std=c11', '-O2', '-ffp-contract=off', '-shared', '-static-libgcc', '-fPIC', '-I' + str(CORE), '-I' + str(FUSION), str(source), str(core_path), str(pair_path), '-lm', '-o', str(lib)]
            run = subprocess.run(command, env=env, capture_output=True, text=True, errors='replace')
            (build / ('compile-' + key + '.log')).write_text(run.stdout + run.stderr, encoding='utf8')
            if run.returncode:
                raise RuntimeError(run.stderr)
        self.library_path=lib
        self.lib = ctypes.CDLL(str(lib))
        ptr = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
        self.lib.replay.argtypes = [ctypes.c_int, ptr, ptr, ptr, ptr, ptr, ctypes.c_int, ctypes.c_int, ptr]
        self.lib.replay.restype = ctypes.c_int

    def run(self, gyro_dps, accel_g, dt, earth_body_dps=(0, 0, 0), *, substeps=1,
            tau=4.3, rest_sigma=.0153, forgetting=427, motion_mode=0,
            accel_gate=2, rest_min=.63, rest_tau=.66, rest_gyro=.68, rest_accel=.21,
            rest_guard_dps=0, rest_anchor_tau=0, raw_accel_stride=None, adaptive_rest=False):
        gyro, accel = [np.ascontiguousarray(x, dtype=np.float64) for x in (gyro_dps, accel_g)]
        durations = np.ascontiguousarray(dt, dtype=np.float64)
        n = len(durations)
        if gyro.shape != (n, 3) or accel.shape != (n, 3) or n < 2:
            raise ValueError('Expected aligned gyro/accel/duration arrays')
        raw_mode = raw_accel_stride is not None
        accel_valid = np.isfinite(accel).all(axis=1)
        if not np.isfinite(gyro).all() or not np.isfinite(durations).all() or np.any(durations <= 0):
            raise ValueError('Replay requires finite samples and positive durations')
        if raw_mode:
            if raw_accel_stride != 4 or substeps != 1 or np.any(~accel_valid & ~np.isnan(accel).all(axis=1)) or accel_valid.sum()<2:
                raise ValueError('Raw mode requires fresh 1:4 accel tags or all-NaN absent samples')
            fresh = np.flatnonzero(accel_valid)
            if fresh[0] >= 4 or n-1-fresh[-1] >= 4 or not np.all(np.diff(fresh) == 4):
                raise ValueError('Missing or irregular fresh acceleration tags')
        elif not accel_valid.all():
            raise ValueError('Window acceleration must be finite')
        if np.ptp(durations) > np.median(durations) * .01:
            raise ValueError('Variable-duration/gap input requires explicit resampling, not silent replay')
        if substeps not in (1, 8) or motion_mode not in (0, 1, 2) or accel_gate not in (0, 1, 2, 3):
            raise ValueError('Unsupported replay policy')
        options = np.array([tau, rest_sigma, forgetting, motion_mode, accel_gate, rest_min, rest_tau, rest_gyro, rest_accel, rest_guard_dps, rest_anchor_tau, bool(adaptive_rest)], dtype=np.float64)
        if not np.isfinite(options).all() or np.any(options[[0, 1, 2, 5, 6, 7, 8]] <= 0) or np.any(options[9:]<0):
            raise ValueError('Filter parameters must be positive')
        earth = np.ascontiguousarray(earth_body_dps, dtype=np.float64)
        if earth.shape != (3,) or not np.isfinite(earth).all():
            raise ValueError('Invalid Earth vector')
        output = np.empty((n, 10), dtype=np.float64)
        code = self.lib.replay(n, gyro, accel, durations, earth, options, substeps, -4 if raw_mode else (1 if substeps == 1 else 2), output)
        if code or not np.isfinite(output).all():
            raise RuntimeError('Native replay failed: ' + str(code))
        return output


def orientations(output):
    """Native quaternion wxyz to SciPy xyzw, normalized by Rotation."""
    return Rotation.from_quat(output[:, [1, 2, 3, 0]])


def resample_windows(gyro, accel, duration, start_time, period=.02):
    """Conservative overlap of constant-mean contiguous windows.

    Integrals are preserved on the returned support. The fractional final
    interval is omitted and must not be mistaken for a transport gap.
    """
    duration = np.asarray(duration, dtype=float)
    if period <= 0 or not np.isfinite(duration).all() or np.any(duration <= 0):
        raise ValueError('Invalid window timing')
    edges = np.r_[0, np.cumsum(duration)]
    target = np.arange(int(np.floor(edges[-1] / period)) + 1) * period
    outputs = []
    for values in (gyro, accel):
        values = np.asarray(values, dtype=float)
        if values.shape != (len(duration), 3) or not np.isfinite(values).all():
            raise ValueError('Invalid window measurements')
        integral = np.vstack([np.zeros(3), np.cumsum(values * duration[:, None], axis=0)])
        sampled = np.column_stack([np.interp(target, edges, integral[:, axis]) for axis in range(3)])
        outputs.append(np.diff(sampled, axis=0) / period)
    return np.asarray(start_time) + (target[:-1] + target[1:]) / 2, outputs[0], outputs[1], np.full(len(target) - 1, period)


def metrics(time, output, rests):
    """Endpoint closures use explicit physical-return assertions in the input.

    For the ten-pose recording only rest 0->4 and 0->last are return anchors.
    Each axis recording asserts every rest has the same physical orientation.
    """
    rotations = orientations(output)
    centers, drifts, rates = [], [], []
    for lo, hi in rests:
        if hi - lo < 20:
            raise ValueError('Rest too short for settled replay metrics')
        start = (time >= lo + 5) & (time < lo + 10)
        end = (time >= hi - 5) & (time < hi - 1)
        if start.sum() < 10 or end.sum() < 10:
            raise ValueError('Rest is outside replay support')
        qa, qb = rotations[start].mean(), rotations[end].mean()
        centers.append(qb)
        error = np.rad2deg((qb * qa.inv()).magnitude())
        drifts.append(float(error))
        rates.append(float(error / (time[end].mean() - time[start].mean()) * 60))
    pairs = [(0, 1), (1, 2), (0, 2)] if len(rests) == 3 else [(0, 4), (0, len(rests) - 1)]
    return dict(rest_drift_deg=drifts, rest_drift_deg_per_min=rates,
                closure_deg=[float(np.rad2deg((centers[b] * centers[a].inv()).magnitude())) for a, b in pairs],
                rest_fraction=float(np.mean(output[:, 7])), trusted_fraction=float(np.mean(output[:, 8])))


HEADER = r'''
#include <math.h>
#include <string.h>
#include "earth_pair.h"
#define DEG_TO_RAD 0.017453292519943295f
#define CONST_EARTH_GRAVITY 9.80665f
#define printk(...) ((void)0)
#define atomic_get(p) (*(p))
static int atomic_set(int *p, int v) { int old=*p; *p=v; return old; }
static vqf_params_t params;
static vqf_state_t state;
static vqf_coeffs_t coeffs;
static earth_pair_t earth_pair;
static float mbe_gyro_mag, accel_trusted_s;
static bool mbe_gate_enabled, accel_rate_allowed, accel_recovery_pending;
static int accel_gate_mode, accel_gate_request;
'''

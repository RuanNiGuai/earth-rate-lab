import copy
import hashlib
from pathlib import Path
import struct
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from compare_motion_calibration import compensate, endpoint_mask, recorded_scale, return_metrics, validate_binding
from fusion_replay import Replay, orientations


def fixture():
    profile = struct.pack('<15f4B', *([1.02, 1.01, .99]+[.1, .02, 0, 0, .2, 0, 0, 0, .3]+[0, 0, 0]), 0, 0, 0, 0)
    thermal = struct.pack('<5H2B', 2, 2, 2, 2, 7, 0, 0)
    thermal += struct.pack('<8f', 20, .1, .2, .3, 40, .3, .4, .5)*2
    acceleration = bytes([1, 1, 1, 1])+struct.pack('<12f', .1, -.2, .3, 2, 0, 0, 0, 3, 0, 0, 0, 4)
    sections = [(1, b'synthetic_cw6\0'), (2, profile), (3, thermal), (4, acceleration)]
    blob = b''.join(struct.pack('<HH', kind, len(data))+data for kind, data in sections)
    metadata = dict(build='synthetic_cw6', rf_address='112233445566', receiver_serial='SYNTHETIC', slot=2,
                    snapshot_sha256=hashlib.sha256(blob).hexdigest())
    snapshot = validate_binding('Device address: 112233445566\n', metadata, blob)
    return blob, metadata, snapshot


class MotionCalibrationTests(unittest.TestCase):
    def samples(self, n=16):
        accel = np.full((n, 3), np.nan)
        accel[::4] = [.1, -.2, .55]
        return dict(gyro=np.full((n, 3), 1.), accel=accel, temp=np.full(n, 30.))

    def test_correct_identity_is_bound_to_actual_snapshot_bytes(self):
        blob, metadata, snapshot = fixture()
        self.assertTrue(snapshot['accelerometer']['active_matches_persisted'])
        for changed in ({**metadata, 'rf_address':'AABBCCDDEEFF'}, {**metadata, 'build':'other'},
                        {**metadata, 'receiver_serial':''}, {**metadata, 'snapshot_sha256':'0'*64}):
            with self.assertRaises(ValueError):
                validate_binding('Device address: 112233445566\n', changed, blob)

    def test_recorded_scale_requires_inactive_unambiguous_status(self):
        np.testing.assert_allclose(recorded_scale('Gyro scale: X 1.01 Y .99 Z 1.02; active=0'), [1.01, .99, 1.02])
        for text in ('Gyro scale: X 1 Y 1 Z 1; active=1',
                     'Gyro scale: X 1 Y 1 Z 1; active=0\nGyro scale: X 1 Y 1 Z 1; active=0',
                     'Gyro scale: X -1 Y 1 Z 1; active=0'):
            with self.assertRaises(ValueError): recorded_scale(text)

    def test_acceleration_coordinates_and_compensation_order(self):
        _, _, snapshot = fixture()
        raw = self.samples()
        gyro, accel = compensate(raw, snapshot, 400)
        # The affine calibration maps [.1,-.2,.55] to [0,0,1].
        np.testing.assert_allclose(accel[::4], np.tile([0, 0, 1], (4, 1)), atol=1e-7)
        # T(30)=[.2,.3,.4], M*a=[0,0,.3], then multiply S.
        np.testing.assert_allclose(gyro[1], [.8*1.02, .7*1.01, .3*.99], atol=1e-7)
        self.assertTrue(np.isnan(accel[1]).all())

    def test_future_acceleration_cannot_change_past_compensation(self):
        _, _, snapshot = fixture()
        raw = self.samples()
        baseline, _ = compensate(raw, snapshot, 400)
        raw['accel'][4:] *= 5
        changed, _ = compensate(raw, snapshot, 400)
        np.testing.assert_array_equal(baseline[:5], changed[:5])
        self.assertGreater(np.max(abs(baseline[5:]-changed[5:])), 1)

    def test_no_future_acceleration_is_invented_at_start(self):
        _, _, snapshot = fixture()
        raw = self.samples()
        raw['accel'][:] = np.nan
        raw['accel'][3::4] = [.1, -.2, .55]
        gyro, _ = compensate(raw, snapshot, 400)
        np.testing.assert_allclose(gyro[0], [.8*1.02, .7*1.01, .6*.99], atol=1e-7)
        np.testing.assert_allclose(gyro[3], [.8*1.02, .7*1.01, .6*.99], atol=1e-7)
        np.testing.assert_allclose(gyro[4], [.8*1.02, .7*1.01, .3*.99], atol=1e-7)

    def test_unobserved_temperature_is_rejected_instead_of_clamped(self):
        _, _, snapshot = fixture()
        raw = self.samples()
        for value in (19., 41., float('nan')):
            raw['temp'][8] = value
            with self.assertRaises(ValueError): compensate(raw, snapshot, 400)

    def test_missing_accel_and_partial_nan_are_rejected(self):
        _, _, snapshot = fixture()
        for missing in ('tag', 'axis'):
            raw = self.samples()
            if missing == 'tag': raw['accel'][4] = np.nan
            else: raw['accel'][4, 1] = np.nan
            with self.assertRaises(ValueError): compensate(raw, snapshot, 400)

    def test_endpoint_support_and_actual_motion_are_checked(self):
        raw = self.samples(4000)
        raw['accel'][::4] = [0, 0, 1]
        self.assertEqual(endpoint_mask(raw, 400, [1, 6]).sum(), 2000)
        for span in ([1, 2], [-1, 6], [5, 11]):
            with self.assertRaises(ValueError): endpoint_mask(raw, 400, span)
        raw['gyro'][::2] = 5
        with self.assertRaises(ValueError): endpoint_mask(raw, 400, [1, 6])

    def test_installed_scale_recovers_known_turn_without_endpoint_fitting(self):
        _, _, snapshot = fixture()
        snapshot = copy.deepcopy(snapshot)
        snapshot['accelerometer'].update(offset_g=[0, 0, 0], matrix=np.eye(3).tolist())
        rate = 400
        raw = self.samples(40*rate)
        raw['accel'][::4] = [0, 0, 1]
        raw['gyro'][:] = [.2, .3, .7]  # T(30) + M @ [0,0,1]
        raw['gyro'][20*rate:24*rate, 2] += 90/.99
        calibrated, acceleration = compensate(raw, snapshot, rate)
        pre = endpoint_mask(raw, rate, [10, 18])
        post = endpoint_mask(raw, rate, [30, 38])
        engine = Replay(ROOT/'.local/native-tests/motion-calibration')
        corrected = engine.run(calibrated, acceleration, np.full(len(calibrated), 1/rate), raw_accel_stride=4)
        original = engine.run(raw['gyro'], raw['accel'], np.full(len(calibrated), 1/rate), raw_accel_stride=4)
        q = orientations(corrected)
        self.assertLess(np.rad2deg((q[24*rate-1]*q[20*rate-1].inv()).magnitude()), .01)
        # The production rest detector has a small low-pass-tail transient
        # after an abrupt stop. Keep that observable instead of fitting the
        # endpoint or claiming the calibration removes filter dynamics.
        self.assertLess(return_metrics(corrected, pre, post)['return_deg'], .15)
        self.assertGreater(return_metrics(original, pre, post)['return_deg'], 3)


if __name__ == '__main__':
    unittest.main()

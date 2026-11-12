import dataclasses
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from batch_orientation import classify
from f04_summary import Summary


def samples(accel=(0, 0, 1000)):
    return [Summary(2, 11, 5, i, i*20000, (1000, 0, 0), accel, 2500, 8, 2, 20000)
            for i in range(100)]


class OrientationTests(unittest.TestCase):
    def test_six_faces(self):
        for axis in range(3):
            for sign in (-1, 1):
                a = [0, 0, 0]
                a[axis] = sign*1000
                result = classify(samples(tuple(a)))
                self.assertEqual(result['axis'], 'xyz'[axis])
                self.assertEqual(result['face'], ('-' if sign < 0 else '+') + 'XYZ'[axis])

    def test_short_mixed_and_duplicate_windows_rejected(self):
        for bad in (samples()[:-1], samples()[:-1] + [samples()[0]],
                    samples()[:-1] + [dataclasses.replace(samples()[-1], boot=12)]):
            with self.assertRaises(ValueError):
                classify(bad)

    def test_motion_and_missing_fresh_accel_rejected(self):
        for change in ({'gyro_delta_udeg': (100000, 0, 0)}, {'accel_fresh': 0}):
            bad = samples()
            bad[-1] = dataclasses.replace(bad[-1], **change)
            with self.assertRaises(ValueError):
                classify(bad)

    def test_tilt_and_non_gravity_rejected(self):
        for accel in ((700, 0, 700), (0, 0, 800), (0, 0, 1200)):
            with self.assertRaises(ValueError):
                classify(samples(accel))


if __name__ == '__main__':
    unittest.main()

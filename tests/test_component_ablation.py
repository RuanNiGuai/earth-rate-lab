import copy
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/jointcal'))
from component_ablation import PROFILES, corrected, contrast, statistics


class ComponentAblationTests(unittest.TestCase):
    def test_controls_preserve_order_and_leave_earth_to_fusion(self):
        g = np.array([[5., 7., 11.]])
        a = np.array([[1., 2., 3.]])
        t = np.array([[.5, 1., 1.5]])
        p = dict(scale=[2., 3., 4.], gsens_dps_per_g=[1., 2., 0., 0., 1., 0., 0., 0., 1.],
                 earth_dps=[.003, 0., .0029])
        before = copy.deepcopy(p)
        np.testing.assert_array_equal(corrected(g, a, t, p, PROFILES['all_on']), [[-1., 12., 26.]])
        np.testing.assert_array_equal(corrected(g, a, t, p, PROFILES['earth_off']), [[-1., 12., 26.]])
        np.testing.assert_array_equal(corrected(g, a, t, p, PROFILES['earth_gsens_off']), [[9., 18., 38.]])
        np.testing.assert_array_equal(corrected(g, a, t, p, PROFILES['all_off']), g)
        np.testing.assert_array_equal(corrected(g, a, t, p, PROFILES['scale_off']), [[-.5, 4., 6.5]])
        self.assertEqual(p, before)

    def test_metrics_keep_worsening_devices_and_zero_baseline(self):
        before = dict(summary={'metric':statistics([0., 4.])}, devices=[
            {'metrics':{'metric':{'rms':0.}}}, {'metrics':{'metric':{'rms':4.}}}])
        after = dict(summary={'metric':statistics([1., 2.])}, devices=[
            {'metrics':{'metric':{'rms':1.}}}, {'metrics':{'metric':{'rms':2.}}}])
        row = contrast(before, after)['metric']
        self.assertEqual(row['devices_improved'], 1)
        self.assertEqual(row['devices_worsened'], 1)
        self.assertAlmostEqual(row['disabled_rms'], np.sqrt(8))
        self.assertAlmostEqual(row['enabled_rms'], np.sqrt(2.5))
        before['summary']['metric']['rms'] = 0.
        self.assertIsNone(contrast(before, after)['metric']['reduction_percent'])

    def test_invalid_observations_are_not_silently_filtered(self):
        p = dict(scale=[1.,1.,1.], gsens_dps_per_g=[0.]*9)
        with self.assertRaisesRegex(ValueError, 'Nonfinite'):
            corrected([[np.nan,0.,0.]], [[0.,0.,1.]], [[0.,0.,0.]], p, 'TSM')


if __name__ == '__main__':
    unittest.main()

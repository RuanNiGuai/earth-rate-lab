import unittest
from batch_snapshot import runtime_lut,restore_auto

class SnapshotTests(unittest.TestCase):
    def test_lut_real_size_and_reject_gaps(self):
        text='TCAL_LUT valid=1 count=3 version=48 points=48\nLUT 0 15 1 2 3\nLUT 1 15.25 1 2 3\nLUT 2 15.5 1 2 3\nTCAL_LUT END'
        self.assertEqual(len(runtime_lut(text)),3)
        for bad in (text.replace('LUT 1','LUT 2'),text.replace('version=48','version=47'),text.replace('TCAL_LUT END','')):
            with self.assertRaises(ValueError):runtime_lut(bad)
    def test_restore_failure_not_silenced(self):
        class Fake:
            def send(self,*args):return 'Auto-calibration: disabled'
        data={}
        with self.assertRaises(ValueError):restore_auto(Fake(),'enabled',data)
        self.assertNotIn('auto_restore_verified',data)
    def test_restore_preserves_disabled(self):
        class Fake:
            def send(self,cmd,wait):
                assert cmd=='tcal status'
                return 'Auto-calibration: disabled'
        data={};restore_auto(Fake(),'disabled',data)
        self.assertTrue(data['auto_restore_verified'])

if __name__=='__main__':unittest.main()

"""Windows replacement contention must not interrupt acquisition."""
import ctypes
from ctypes import wintypes
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import batch_calibration as b


class AtomicSaveTests(unittest.TestCase):
    def test_durable_failure_retains_original_and_cleans_pending_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'state.json';p.write_text('{"old":true}')
            error=PermissionError('busy');error.winerror=5
            with patch.object(b.os,'replace',side_effect=error),patch.object(b.time,'sleep') as sleep:
                with self.assertRaises(PermissionError):b.atomic_save(p,{'new':True})
                self.assertEqual(sleep.call_count,6)
            self.assertEqual(json.loads(p.read_text()),{'old':True})
            self.assertEqual(list(Path(tmp).iterdir()),[p])

    def test_telemetry_does_not_hide_disk_full_or_non_windows_permissions(self):
        with tempfile.TemporaryDirectory() as tmp:
            for error in [OSError(28,'disk full'),PermissionError(13,'denied')]:
                with patch.object(b.os,'replace',side_effect=error):
                    with self.assertRaises(OSError):b.atomic_save(Path(tmp)/'progress.json',{},best_effort=True)

    @unittest.skipUnless(os.name=='nt','Windows file sharing semantics')
    def test_real_windows_reader_contention_skips_telemetry_and_retries_state(self):
        kernel=ctypes.WinDLL('kernel32',use_last_error=True)
        kernel.CreateFileW.argtypes=[wintypes.LPCWSTR,wintypes.DWORD,wintypes.DWORD,ctypes.c_void_p,wintypes.DWORD,wintypes.DWORD,wintypes.HANDLE]
        kernel.CreateFileW.restype=wintypes.HANDLE
        kernel.CloseHandle.argtypes=[wintypes.HANDLE];kernel.CloseHandle.restype=wintypes.BOOL
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'progress.json';b.atomic_save(p,{'value':0})
            handle=kernel.CreateFileW(str(p),0x80000000,3,None,3,0,None) # no FILE_SHARE_DELETE
            self.assertNotEqual(handle,ctypes.c_void_p(-1).value)
            try:
                with patch.object(b.time,'sleep',side_effect=AssertionError('telemetry blocked pump')):
                    self.assertFalse(b.atomic_save(p,{'value':1},best_effort=True))
                self.assertEqual(json.loads(p.read_text()),{'value':0})
            finally:kernel.CloseHandle(handle)
            self.assertTrue(b.atomic_save(p,{'value':2},best_effort=True))
            handle=kernel.CreateFileW(str(p),0x80000000,3,None,3,0,None)
            release=threading.Thread(target=lambda:(time.sleep(.05),kernel.CloseHandle(handle)))
            release.start()
            try:self.assertTrue(b.atomic_save(p,{'value':3}))
            finally:release.join()
            self.assertEqual(json.loads(p.read_text()),{'value':3})
            self.assertEqual(list(Path(tmp).iterdir()),[p])


if __name__=='__main__':unittest.main()

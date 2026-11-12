"""Exercise the real recorder and Radio with a silent ordinary HID endpoint."""
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import batch_calibration as b
import batch_record_only as recorder
import capture_preflight
from test_capture_transport import report


class Clock:
    now=100.
    def monotonic(self):return self.now
    def monotonic_ns(self):return round(self.now*1e9)
    def time_ns(self):return round(self.now*1e9)
    def sleep(self,seconds):self.now+=seconds


class Rig:
    def __init__(self,clock,fail_mode=False):
        self.clock=clock;self.fail_mode=fail_mode
        self.targets={i:f'{i+1:012X}' for i in range(10)}
        self.devices=[];self.commands=[];self.started=None;self.stopped=False
        self.sent=0;self.total=0;self.ends=0;self.console_text=b'';self.main_slot=0
        self.silent_polls=0;self.console_closed=False

    def enumerate(self,*args):
        return [dict(serial_number='TEST',interface_number=2,usage_page=1,path=b'main'),
                dict(serial_number='TEST',interface_number=3,usage_page=0xff00,path=b'capture')]

    def device(self):
        rig=self
        class Device:
            nonblocking=False
            closed=False
            def open_path(self,path):self.path=path
            def set_nonblocking(self,value):
                # Reproduce the original constructor for the negative control.
                if not (rig.fail_mode and self.path==b'main'):self.nonblocking=value
            def close(self):self.closed=True
            def read(self,length,timeout_ms=0):
                if timeout_ms>0:rig.clock.sleep(timeout_ms/1000)
                if self.path==b'main':
                    if rig.started is not None:
                        if timeout_ms<=0 and not self.nonblocking:
                            raise RuntimeError('ordinary HID blocked during F04')
                        rig.silent_polls+=1
                        return []
                    slot=rig.main_slot%10;rig.main_slot+=1
                    address=bytes.fromhex(rig.targets[slot])[::-1]
                    return bytes([255,slot])+address+bytes(8)+bytes([1,slot])+bytes(46)
                if rig.started is None:return []
                count=rig.total if rig.stopped else int((rig.clock.now-rig.started)*50)*10
                if rig.sent<count:
                    index=rig.sent;rig.sent+=1
                    payload=bytearray(report(index//10,index%10))
                    payload[20:32]=bytes(12)
                    struct.pack_into('<I',payload,44,zlib.crc32(payload[:44]))
                    return bytes(payload)
                if rig.stopped and rig.ends<10:
                    slot=rig.ends;rig.ends+=1
                    return report(tracker=slot,end=dict(sequence_end=rig.total//10,valid_windows=rig.total//10,
                                  submitted=rig.total//10,partial_samples=0))
                return []
        device=Device();self.devices.append(device);return device

    def serial(self,*args,**kwargs):
        rig=self
        class Console:
            def read(self,length):
                data=rig.console_text;rig.console_text=b'';return data
            def write(self,payload):
                command=payload.decode().strip();rig.commands.append(command)
                if command=='collect':answer='Data collection inactive'
                elif command=='ota':answer='No active session'
                elif command=='list':answer='\n'.join(rig.targets.values())
                elif command=='collectall 50':
                    rig.started=rig.clock.now;answer='Batch collection STARTED'
                elif command=='collectstop':
                    rig.stopped=True;rig.total=int((rig.clock.now-rig.started)*50)*10
                    answer=f'Batch data collection STOPPED (received: {rig.total+10}, sent: {rig.total+10}, dropped: 0)'
                else:raise AssertionError('Unexpected hardware command: '+command)
                rig.console_text=(answer+'\n').encode();return len(payload)
            def close(self):rig.console_closed=True
        return Console()


class RecorderPumpTests(unittest.TestCase):
    def run_capture(self,directory,fail_mode=False,telemetry_conflict=False,quality_failure=False):
        clock=Clock();rig=Rig(clock,fail_mode);root=Path(directory)
        (root/'session.json').write_text(json.dumps(dict(receiver_serial='TEST',targets=rig.targets)))
        ports=SimpleNamespace(comports=lambda:[SimpleNamespace(vid=0x1209,pid=0x7690,serial_number='TEST',device='FAKE')])
        modules={'hid':SimpleNamespace(enumerate=rig.enumerate,device=rig.device),
                 'serial':SimpleNamespace(Serial=rig.serial),'serial.tools':SimpleNamespace(list_ports=ports),
                 'serial.tools.list_ports':ports}
        stages=[];ready_time=None;deferred=[];original_replace=b.os.replace
        def replace(src,dst):
            if quality_failure and Path(dst).name=='quality.json':
                raise OSError('quality disk full')
            if telemetry_conflict and Path(dst).name in ('progress.json','live.json'):
                deferred.append(str(dst));error=PermissionError('Windows reader holds target');error.winerror=5;raise error
            return original_replace(src,dst)
        def progress(row):
            nonlocal ready_time
            if 'stage' in row:stages.append(row['stage'])
            b.atomic_save(root/'progress.json',row,best_effort=True)
            if row.get('ready'):ready_time=clock.now
            if ready_time is not None and clock.now-ready_time>=3:
                output=Path(row.get('output',root));(output/'STOP_RECORDING').write_text('test complete')
        with patch.dict(sys.modules,modules),patch.dict(os.environ,{'EARTH_RATE_STATE_DIR':directory}),\
             patch.object(b,'time',clock),patch.object(recorder,'time',clock),\
             patch.object(capture_preflight,'windows_processes',return_value=[]),patch.object(b.os,'replace',side_effect=replace):
            if fail_mode:
                with self.assertRaisesRegex(RuntimeError,'ordinary HID blocked'):
                    recorder.record(root,progress=progress,kind='x')
                return None,rig,stages
            if quality_failure:
                with self.assertRaisesRegex(OSError,'quality disk full'):
                    recorder.record(root,progress=progress,kind='x')
                return None,rig,stages
            output=recorder.record(root,progress=progress,kind='x')
        if telemetry_conflict:self.assertGreater(len(deferred),5)
        return output,rig,stages

    def test_f04_ready_and_stop_work_while_ordinary_pose_endpoint_is_silent(self):
        with tempfile.TemporaryDirectory() as tmp:
            output,rig,stages=self.run_capture(tmp)
            quality=json.loads((output/'quality.json').read_text())
            self.assertTrue(quality['transport_pass'],quality['failures'])
            self.assertEqual(len(quality['streams']),10)
            self.assertIn('ready',stages);self.assertIn('stopping',stages)
            self.assertGreater(rig.silent_polls,200)
            self.assertTrue(all(d.closed for d in rig.devices));self.assertTrue(rig.console_closed)
            self.assertFalse(list(Path(tmp).rglob('*.lock')))
            self.assertEqual(rig.commands,['collect','ota','list','collectall 50','collectstop','collect'])

    def test_original_blocking_mode_reproduces_failure_in_same_capture(self):
        with tempfile.TemporaryDirectory() as tmp:
            _,rig,stages=self.run_capture(tmp,fail_mode=True)
            self.assertNotIn('ready',stages)
            self.assertTrue(all(d.closed for d in rig.devices));self.assertTrue(rig.console_closed)

    def test_repeated_progress_and_live_conflicts_preserve_full_recording_and_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            output,rig,stages=self.run_capture(tmp,telemetry_conflict=True)
            q=json.loads((output/'quality.json').read_text())
            self.assertTrue(q['transport_pass'],q['failures'])
            self.assertTrue(q['stop_confirmed'])
            self.assertIn('ready',stages);self.assertTrue(rig.stopped)
            self.assertTrue(all(d.closed for d in rig.devices))

    def test_metadata_write_failure_still_closes_all_device_handles(self):
        with tempfile.TemporaryDirectory() as tmp:
            _,rig,_=self.run_capture(tmp,quality_failure=True)
            self.assertTrue(rig.stopped)
            self.assertTrue(all(d.closed for d in rig.devices));self.assertTrue(rig.console_closed)
            self.assertFalse(list(Path(tmp).rglob('*.lock')))

    def test_slimevr_conflict_rejected_before_hardware_or_recording_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'session.json').write_text(json.dumps(dict(receiver_serial='TEST',targets={'0':'010203040506'})))
            with patch.object(capture_preflight,'windows_processes',return_value=[('SlimeVR.exe','')]),\
                 patch.object(b,'Radio',side_effect=AssertionError('hardware opened')):
                with self.assertRaisesRegex(ValueError,'请先完全退出 SlimeVR'):recorder.record(root,kind='x')
            self.assertEqual([p.name for p in root.iterdir()],['session.json'])

    def test_bundled_background_server_is_detected_but_unrelated_java_is_allowed(self):
        with patch.object(capture_preflight,'windows_processes',return_value=[('java.exe',r'C:\Apps\SlimeVR\jre\bin\java.exe')]):
            with self.assertRaisesRegex(ValueError,'后台服务器'):capture_preflight.check_capture_conflicts()
        with patch.object(capture_preflight,'windows_processes',return_value=[('java.exe',r'C:\Apps\Other\jre\bin\java.exe')]):
            capture_preflight.check_capture_conflicts()


if __name__=='__main__':unittest.main()

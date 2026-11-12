"""Transaction and identity regressions; these tests never open hardware."""
import dataclasses
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import batch_calibration as b


def status(phase=4, **changes):
    base = b.Status(2, 7, 0x4b, phase, 0, 0x3f, 12, 7, 5, 6, 9)
    return dataclasses.replace(base, **changes)


class ProtocolTests(unittest.TestCase):
    def test_idle_main_hid_poll_does_not_wait_for_suspended_pose_stream(self):
        class Device:
            nonblocking=False
            closed=False
            calls=[]
            def open_path(self,path):pass
            def set_nonblocking(self,value):self.nonblocking=value
            def close(self):self.closed=True
            def read(self,size,timeout_ms=0):
                if timeout_ms<=0 and not self.nonblocking:
                    raise AssertionError('hid_read would block until ordinary poses resume')
                self.calls.append(timeout_ms)
                return []
        device=Device()
        fake=SimpleNamespace(device=lambda:device,enumerate=lambda *args:[dict(serial_number='TEST',interface_number=2,usage_page=1,path=b'test')])
        with tempfile.TemporaryDirectory() as directory, patch.dict(sys.modules,{'hid':fake}), patch.dict(os.environ,{'EARTH_RATE_STATE_DIR':directory}):
            radio=b.Radio('TEST',{0:'010203040506'},b.Journal(Path(directory)/'events.jsonl'))
            try:
                self.assertEqual(radio.read(0),[])
                self.assertEqual(radio.read(100),[])  # Explicit positive timeout still works.
                self.assertTrue(radio.lock_path.exists())
            finally:radio.close()
            self.assertEqual(device.calls,[0,100])
            self.assertTrue(device.closed)
            self.assertFalse(radio.lock_path.exists())

    def test_failed_nonblocking_configuration_closes_handle_and_releases_lock(self):
        class Device:
            closed=False
            def open_path(self,path):pass
            def set_nonblocking(self,value):raise OSError('mode rejected')
            def close(self):self.closed=True
        device=Device()
        fake=SimpleNamespace(device=lambda:device,enumerate=lambda *args:[dict(serial_number='TEST',interface_number=2,usage_page=1,path=b'test')])
        with tempfile.TemporaryDirectory() as directory, patch.dict(sys.modules,{'hid':fake}), patch.dict(os.environ,{'EARTH_RATE_STATE_DIR':directory}):
            with self.assertRaisesRegex(OSError,'mode rejected'):
                b.Radio('TEST',{0:'010203040506'},b.Journal(Path(directory)/'events.jsonl'))
            self.assertTrue(device.closed)
            self.assertFalse(list(Path(directory).rglob('*.lock')))

    def test_crc_and_decode(self):
        packet = bytearray([0x16, 2, 1, 7, 0x4b, 4, 0, 63, 12, 7, 5, 6, 0, 0, 9, 0])
        packet[-1] = b.crc8(packet[:-1])
        self.assertEqual(b.Status.decode(packet), status())
        packet[3] ^= 1
        with self.assertRaises(ValueError):
            b.Status.decode(packet)

    def test_old_counts_and_execution_are_not_saved(self):
        tx = b.Transaction(2, 7, 0x4b, True)
        for phase in (1, 2):
            tx.observe(status(phase))
            self.assertFalse(tx.succeeded)
        tx.observe(status())
        self.assertTrue(tx.succeeded)

    def test_wrong_slot_sequence_or_command_never_qualify(self):
        for change in ({'slot': 3}, {'sequence': 6}, {'flag': 0x4c}):
            tx = b.Transaction(2, 7, 0x4b, True)
            self.assertFalse(tx.observe(status(**change)))
            self.assertFalse(tx.succeeded)

    def test_delayed_execute_cannot_replace_save(self):
        tx = b.Transaction(2, 7, 0x4b, True)
        tx.observe(status())
        tx.observe(status(2))
        self.assertTrue(tx.succeeded)

    def test_failure_is_sticky_even_after_delayed_success(self):
        tx = b.Transaction(2, 7, 0x4b, True)
        tx.observe(status(5, error=11))
        tx.observe(status())
        self.assertFalse(tx.succeeded)
        self.assertEqual(tx.failure, 'NVS')

    def test_read_only_status_needs_execution(self):
        tx = b.Transaction(2, 7, 0x42, False)
        tx.observe(status(1, flag=0x42))
        self.assertFalse(tx.succeeded)
        tx.observe(status(2, flag=0x42))
        self.assertTrue(tx.succeeded)

    def test_invalid_target_sets(self):
        for targets in ({}, {'32': '010203040506'}, {'0': '000000000000'},
                        {'0': '010203040506', '1': '010203040506'}, {'01': '010203040506'}):
            with self.assertRaises(ValueError):
                b.validate_targets(targets)

    def test_rf_registration_byte_order_and_replacement(self):
        radio = object.__new__(b.Radio)
        radio.targets = {2: '010203040506'}
        radio.registered, radio.live, radio.conflicts = {}, {}, set()
        class Device:
            def __init__(self, address):
                self.address = address
            def read(self, *args):
                return bytes([255, 2]) + self.address + bytes(56)
        radio.device = Device(bytes.fromhex('060504030201'))
        radio.read()
        self.assertIn(2, radio.registered)
        radio.device = Device(bytes.fromhex('060504030207'))
        with self.assertRaises(ValueError):
            radio.read()
        radio.device = Device(bytes.fromhex('060504030201'))
        with self.assertRaises(ValueError):
            radio.read()  # original identity returning does not undo conflict


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.state = {'devices': {'2': {'on': True, 'reference': True}}}

    def test_wrong_axis_stop_is_refused_before_io(self):
        self.state['devices']['2']['active'] = 'scale-x-start'
        with self.assertRaises(ValueError):
            b.stage_allowed(self.state, 'scale-y-stop', [2])
        b.stage_allowed(self.state, 'scale-x-stop', [2])

    def test_reference_cannot_change_mid_capture(self):
        self.state['devices']['2']['active'] = 'earth-start'
        with self.assertRaises(ValueError):
            b.stage_allowed(self.state, 'reference-new', [2])

    def test_partial_failure_does_not_advance_failed_device(self):
        self.state['devices']['3'] = {'on': True, 'reference': True}
        b.advance(self.state, 'earth-start', {'2': {'success': True}, '3': {'success': False}})
        self.assertEqual(self.state['devices']['2']['active'], 'earth-start')
        self.assertNotIn('active', self.state['devices']['3'])
        self.assertTrue(self.state['devices']['3']['uncertain'])

    def test_pose_count_is_this_run_not_old_firmware_count(self):
        b.advance(self.state, 'earth-start', {'2': {'success': True}})
        for i in range(5):
            b.advance(self.state, 'earth-pose', {'2': {'success': True, 'status': as_old_counts()}})
            self.assertEqual(self.state['devices']['2']['earth'], i + 1)
        self.assertIsNone(self.state['devices']['2']['active'])
        with self.assertRaises(ValueError):
            b.stage_allowed(self.state, 'earth-pose', [2])

    def test_status_does_not_clear_uncertainty(self):
        self.state['devices']['2']['uncertain'] = True
        b.advance(self.state, 'status', {'2': {'success': True}})
        self.assertTrue(self.state['devices']['2']['uncertain'])
        b.advance(self.state, 'cancel', {'2': {'success': True}})
        self.assertFalse(self.state['devices']['2']['uncertain'])

    def test_session_lock_and_durable_create(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'lock'
            with b.lock(path):
                with self.assertRaises(FileExistsError):
                    with b.lock(path):
                        self.fail('concurrent writer admitted')
            self.assertFalse(path.exists())
            path = Path(directory) / 'data.json'
            b.save_new(path, {'old': 1})
            with self.assertRaises(FileExistsError):
                b.save_new(path, {})
            b.atomic_save(path, {'new': 2})
            self.assertEqual(json.loads(path.read_text()), {'new': 2})

    def test_global_mailbox_is_serialized_but_capture_overlaps(self):
        class Radio:
            def __init__(self):
                self.pending = None
                self.started = []
                self.waiting_save = []
                self.clock = 0.
            def send(self, slot, sequence, flag):
                self.assert_free()
                self.pending = slot
                self.started.append(slot)
            def assert_free(self):
                if self.pending is not None:
                    raise AssertionError('second command before first execute')
            def read(self, *args):
                self.clock += .01
                if self.pending is not None:
                    slot, self.pending = self.pending, None
                    self.waiting_save.append(slot)
                    return [status(2, slot=slot)]
                saved, self.waiting_save = self.waiting_save, []
                return [status(4, slot=slot) for slot in saved]
        radio = Radio()
        with patch.object(b.time, 'monotonic', lambda: radio.clock):
            result = b.perform(radio, 'scale-x-stop', 7, [2, 3, 4])
        self.assertEqual(radio.started, [2, 3, 4])
        self.assertTrue(all(row['success'] for row in result.values()))


def as_old_counts():
    return dataclasses.asdict(status())


if __name__ == '__main__':
    unittest.main()

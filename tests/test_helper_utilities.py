"""Utility isolation, protocol receipts and thermal audit edge cases."""
import hashlib
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import helper_utilities as u
import helper_fleet as h
from calwire import unpack_snapshot
from thermal_coverage import audit


class Journal:
    def add(self, **row):
        pass


class UtilityTests(unittest.TestCase):
    def test_temperature_status_distinguishes_automatic_enable_from_active_sampling(self):
        radio = object.__new__(u.Radio)
        radio.targets = {3: '010203040506'}
        radio.conflicts = set()
        radio.registered = {3: 1}
        radio.live = {}
        radio.receipts = {}
        radio.temperatures = {}
        radio.journal = Journal()
        packet = bytearray(64)
        packet[:2] = bytes([6, 3])
        packet[2:4] = (80).to_bytes(2, 'little')
        packet[6:8] = (51).to_bytes(2, 'little', signed=True)
        packet[8] = 8  # Protocol bit 3 is auto collection, not compensation.
        radio.observe(packet)
        row = radio.temperatures[3]
        self.assertTrue(row['auto_collection_enabled'])
        self.assertFalse(row['collecting'])
        self.assertEqual(row['temperature_c'], 25.5)
        self.assertNotIn('compensation_enabled', row)
        packet[8] = 0
        radio.observe(packet)
        self.assertFalse(radio.temperatures[3]['auto_collection_enabled'])

    def test_command_needs_matching_completion_not_admission_or_foreign_ack(self):
        radio = object.__new__(u.Radio)
        radio.sequence = 40
        radio.receipts = {}
        radio.temperatures = {}
        radio.targets = {3: '010203040506'}
        radio.conflicts = set()
        radio.registered = {3: 1}
        radio.live = {}
        radio.journal = Journal()
        observed = []
        # Ignore a prior sequence, another opcode and STARTED; accept only OK.
        packets = [bytes([251,40,0x13,0]), bytes([251,41,0x14,0]),
                   bytes([251,41,0x13,7]), bytes([251,41,0x13,0])]
        radio.send = lambda *args: None
        def read(timeout):
            packet = packets.pop(0).ljust(64, b'\0')
            observed.append(packet)
            radio.observe(packet)
        radio.read = read
        radio.control(3, 0x13)
        self.assertEqual(len(observed), 4)
        # Same slot with changed RF address revokes authorization.
        with self.assertRaisesRegex(ValueError, 'RF identity changed'):
            radio.observe(bytes([255,3])+bytes.fromhex('999999999999')+bytes(56))

    def test_empty_thermal_table_can_be_inspected_but_cannot_feed_solver(self):
        def tlv(kind, data):
            return struct.pack('<HH', kind, len(data))+data
        profile = struct.pack('<15f4B', 1,1,1,*([0]*12),0,0,0,1)
        thermal = struct.pack('<5H2B',0,140,0,0,123,1,0)+bytes(140*16)
        raw = tlv(1,b'test\0')+tlv(2,profile)+tlv(3,thermal)
        with self.assertRaisesRegex(ValueError,'Thermal table not ready'):
            unpack_snapshot(raw, require_scales=False)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            dest = root/'slot-0'
            dest.mkdir()
            (dest/'snapshot.bin').write_bytes(raw)
            (dest/'backup.json').write_text(json.dumps(dict(rf_address='010203040506',
                snapshot_sha256=hashlib.sha256(raw).hexdigest())))
            result = audit(root, [0])
        self.assertFalse(result['complete'])
        self.assertEqual(result['any_device_missing_intervals_c'], [[15.0,43.0]])
        self.assertIsNone(result['devices'][0]['collected_temperature_centers_c'])

    def test_utility_completion_never_replaces_calibration_or_last_session(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session = root/'session'
            session.mkdir()
            job = root/'job'
            job.mkdir()
            original = b'{"original": "accepted captures"}'
            (session/'wizard.json').write_bytes(original)
            (root/'last-session.json').write_bytes(original)
            request = dict(schema='HELPER_FLEET1',action='utility-temperature-status',session=str(session),
                receiver_serial='TEST',targets={str(i):f'{i+1:012X}' for i in range(10)})
            with patch.dict('os.environ',EARTH_RATE_STATE_DIR=str(root)), patch.object(u,'execute',return_value=dict(output=str(job))):
                result = h.execute(request,job)
            self.assertTrue(result['utility'])
            self.assertNotIn('session',result)
            self.assertEqual((root/'last-session.json').read_bytes(),original)
            self.assertEqual((session/'wizard.json').read_bytes(),original)

    def test_start_failure_disables_every_possibly_enabled_tracker(self):
        class Partial:
            instance = None
            def __init__(self,*args):
                Partial.instance=self
                self.temperatures={}
                self.commands=[]
                self.closed=False
            def bind(self): pass
            def control(self,slot,flag):
                self.commands.append((slot,flag))
                if (slot,flag)==(1,0x13):raise TimeoutError('lost ack after enable')
                if flag==0x11:self.temperatures[slot]={'slot':slot,'auto_collection_enabled':True}
            def close(self):self.closed=True
        with tempfile.TemporaryDirectory() as directory,patch.object(u,'Radio',Partial):
            root=Path(directory)
            request=dict(action='utility-temperature-start',receiver_serial='TEST',targets={'0':'000000000001','1':'000000000002'})
            with self.assertRaises(TimeoutError):
                u.temperature(request,root/'output',root/'CANCEL',lambda **row:None)
            self.assertEqual(Partial.instance.commands[-2:],[(0,0x14),(1,0x14)])
            self.assertTrue(Partial.instance.closed)

    def test_command_echo_without_matching_auto_state_is_not_success(self):
        now = [0.0]
        class WrongState:
            instance = None
            def __init__(self, *args):
                WrongState.instance = self
                self.temperatures = {}
                self.commands = []
                self.closed = False
            def bind(self): pass
            def control(self, slot, flag):
                self.commands.append((slot, flag))
                if flag == 0x11:
                    self.temperatures[slot] = dict(slot=slot, auto_collection_enabled=False)
            def read(self, timeout): now[0] += 1
            def close(self): self.closed = True
        with tempfile.TemporaryDirectory() as directory, patch.object(u, 'Radio', WrongState), \
             patch.object(u.time, 'monotonic', side_effect=lambda: now[0]):
            root = Path(directory)
            request = dict(action='utility-temperature-start', receiver_serial='TEST',
                           targets={'0': '000000000001'})
            with self.assertRaisesRegex(ValueError, 'state did not match'):
                u.temperature(request, root/'output', root/'CANCEL', lambda **row: None)
            self.assertEqual(WrongState.instance.commands[-1], (0, 0x14))
            self.assertTrue(WrongState.instance.closed)

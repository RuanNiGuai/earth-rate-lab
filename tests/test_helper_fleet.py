"""Real bridge admission, saved-report audit and cancellation cleanup."""
import copy
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import helper_fleet as h
from calwire import Radio
from test_calwire_apply import snapshot
from test_capture_transport import report


class StatusRadio:
    """Read-only device acknowledgements; never opens a USB/HID handle."""
    instances = []
    failed_slot = None

    def __init__(self, *args):
        self.sent = []
        self.pending = None
        self.closed = False
        self.instances.append(self)

    def bind(self):
        pass

    def send(self, slot, sequence, flag):
        assert self.pending is None, 'Receiver mailbox overwritten'
        assert flag == h.b.FLAGS['status'], 'Initialization attempted a device mutation'
        self.pending = (slot, sequence, flag)
        self.sent.append(self.pending)

    def read(self, *args):
        slot, sequence, flag = self.pending
        self.pending = None
        failed = slot == self.failed_slot
        return [h.b.Status(slot, sequence, flag, 5 if failed else 2,
                           11 if failed else 0, 0, 0, 0, 0, 0, 0)]

    def close(self):
        self.closed = True


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.session = self.root/'session'
        self.session.mkdir()
        self.job = self.root/'job'
        self.job.mkdir()
        self.state = dict(receiver_serial='TEST', targets={str(i):f'{i+1:012X}' for i in range(10)})
        (self.session/'session.json').write_text(json.dumps(self.state))
        self.request = dict(schema='HELPER_FLEET1', action='record', session=str(self.session), **self.state)

    def test_reopen_restores_saved_diagnostics_without_device_access(self):
        import fleet_workflow as w
        self.state['initialized']=True
        (self.session/'session.json').write_text(json.dumps(self.state))
        config=self.session/'input.json';config.write_text('original hashed config')
        solved=self.session/'result.json'
        fit=dict(input_config=dict(config_sha256=w.digest(config)),static=dict(devices=[dict(slot=0)]),
                 iterations=[dict(scale=dict(direction_checks=[dict(slot=0,passed=False)]))])
        solved.write_text(json.dumps(fit))
        w.save(self.session,dict(schema='FLEET_WIZARD1',captures={},history=[],
                                artifacts=dict(input=str(config),result=str(solved))))
        with patch.object(h,'snapshot',side_effect=AssertionError('device access')):
            result=h.execute(dict(self.request,action='workflow-open'),self.job)
        self.assertEqual(result['devices'],fit['static']['devices'])
        self.assertFalse(result['installable'])
        self.assertFalse(result['direction_checks'][0]['passed'])
        config.write_text('changed config')
        retry=self.root/'retry';retry.mkdir()
        result=h.execute(dict(self.request,action='workflow-open'),retry)
        self.assertIn('不一致',result['failures'][0])
        self.assertNotIn('devices',result)

    def test_reopening_rejected_capture_preserves_failure_and_valid_axes(self):
        import fleet_workflow as w
        self.state['initialized']=True
        (self.session/'session.json').write_text(json.dumps(self.state))
        source=self.session/'capture'/'reports.jsonl'
        entry=dict(kind='poses',recording=str(source),accepted=False,error='#3 queue overflow',
                   analysis_version=w.continuous.POSE_ANALYSIS_VERSION)
        axes={k:dict(accepted=True,recording=k+'.jsonl') for k in ('x','y','z')}
        w.save(self.session,dict(schema='FLEET_WIZARD1',captures=axes,history=[entry],artifacts={}))
        before=(self.session/'wizard.json').read_bytes()
        with patch.object(h,'snapshot',side_effect=AssertionError('device access')):
            result=h.execute(dict(self.request,action='workflow-open'),self.job)
        self.assertEqual(result['state'],'CAPTURE_REJECTED')
        self.assertEqual(result['failures'],['#3 queue overflow'])
        self.assertEqual(result['workflow']['captures'],axes)
        self.assertEqual((self.session/'wizard.json').read_bytes(),before)

    def test_slimevr_conflict_preserves_previous_capture_and_plan(self):
        import capture_preflight
        import fleet_workflow
        fleet_workflow.save(self.session,dict(schema='FLEET_WIZARD1',captures={'x':{'accepted':True,'recording':'kept.jsonl'}},history=[],artifacts={'plan':'kept-plan.json'}))
        h.b.atomic_save(self.session/'helper-artifacts.json',{'plan':'kept-plan.json'})
        before={p.name:p.read_bytes() for p in self.session.iterdir()}
        with patch.object(capture_preflight,'windows_processes',return_value=[('SlimeVR.exe','')]),\
             patch.object(h.b,'Radio',side_effect=AssertionError('Opened hardware')):
            with self.assertRaisesRegex(ValueError,'SlimeVR'):
                h.execute(dict(self.request,action='workflow-record-x'),self.job)
        self.assertEqual(before,{p.name:p.read_bytes() for p in self.session.iterdir()})

    def test_init_reports_each_round_and_reuses_session_without_losing_data(self):
        updates = []
        save = h.b.atomic_save
        def observe(path, value, **options):
            if path.name == 'progress.json':
                updates.append(copy.deepcopy(value))
            return save(path, value, **options)
        with patch.object(h.b, 'Radio', StatusRadio), patch.object(h.b, 'atomic_save', observe):
            result = h.execute(dict(self.request, action='init', session=str(self.root)), self.job)
        session = Path(result['session'])
        self.assertEqual(session.parent, self.root)
        self.assertEqual(result['state'], 'SESSION_READY')
        self.assertFalse(result['reused'])
        self.assertEqual(result['output'], str(session))
        checks = [p for p in updates if p['stage'] == 'checking']
        for round_number in (1, 2, 3):
            self.assertEqual({p['checked'] for p in checks if p['round'] == round_number}, set(range(11)))
        self.assertTrue(all(not p['ready'] for p in updates))  # Never prompts motion.
        self.assertTrue(StatusRadio.instances[-1].closed)
        self.assertEqual(len(StatusRadio.instances[-1].sent), 30)

        state = h.read_json(session/'session.json')
        state['devices']['2'] = dict(on=True, reference=True, scale=['x'], uncertain=True)
        h.b.atomic_save(session/'session.json', state)
        h.b.save_new(session/'helper-artifacts.json', dict(recording='existing-reports.jsonl'))
        (session/'existing-reports.jsonl').write_bytes(b'original recording')
        index_before = (session/'helper-artifacts.json').read_bytes()
        repeat_job = self.root/'repeat-job'; repeat_job.mkdir()
        with patch.object(h.b, 'Radio', StatusRadio):
            repeated = h.execute(dict(self.request, action='init', session=str(session)), repeat_job)
        self.assertEqual(repeated['session'], str(session))
        self.assertTrue(repeated['reused'])
        self.assertEqual(h.read_json(session/'session.json')['devices'], state['devices'])
        self.assertEqual(h.read_json(session/'session.json')['next_sequence'], state['next_sequence']+1)
        self.assertEqual((session/'helper-artifacts.json').read_bytes(), index_before)
        self.assertEqual((session/'existing-reports.jsonl').read_bytes(), b'original recording')
        self.assertFalse(list(session.glob('fleet-*')))
        self.assertTrue(StatusRadio.instances[-1].closed)

    def test_reverify_refuses_wrong_binding_unfinished_init_or_inflight_before_io(self):
        for changes, message in [
            ({'receiver_serial':'OTHER'}, 'differs'),
            ({'targets':{str(i):f'{i+50:012X}' for i in range(10)}}, 'differs'),
            ({'initialized':False}, 'initialization did not finish'),
            ({'inflight':{'action':'scale-x-start'}}, 'Interrupted command'),
        ]:
            state = dict(self.state, initialized=True, inflight=None, next_sequence=5,
                         devices={str(i):{} for i in range(10)})
            state.update(changes)
            h.b.atomic_save(self.session/'session.json', state)
            before = (self.session/'session.json').read_bytes()
            with patch.object(h.b, 'Radio', side_effect=AssertionError('Opened hardware')):
                with self.assertRaisesRegex(ValueError, message):
                    h.execute(dict(self.request, action='init'), self.job)
            self.assertEqual((self.session/'session.json').read_bytes(), before)
            self.assertFalse((self.job/'result.json').exists())

    def test_failed_initial_verification_is_not_session_ready_and_closes_radio(self):
        with patch.object(h.b, 'Radio', StatusRadio), patch.object(StatusRadio, 'failed_slot', 7):
            with self.assertRaisesRegex(ValueError, 'Initial status incomplete'):
                h.execute(dict(self.request, action='init', session=str(self.root)), self.job)
        created, = self.root.glob('fleet-*')
        self.assertFalse(h.read_json(created/'session.json')['initialized'])
        self.assertFalse((self.job/'result.json').exists())
        self.assertTrue(StatusRadio.instances[-1].closed)

    def test_receiver_roster_and_confirmation_before_device_io(self):
        h.validate_request(self.request)
        for name, value in [('receiver_serial','OTHER'), ('targets',{str(i):f'{i+50:012X}' for i in range(10)})]:
            request = dict(self.request, **{name:value})
            with patch('calwire.Radio', side_effect=AssertionError('Opened hardware')):
                with self.assertRaisesRegex(ValueError, 'differs'):
                    h.execute(request, self.job)
        for action in ('apply','rollback','plan','reference-new'):
            with self.assertRaisesRegex(ValueError, 'confirmation'):
                h.validate_request(dict(self.request, action=action))
        for targets in ({'0':'000000000001'}, {str(i):'000000000001' for i in range(10)}):
            with self.assertRaises(ValueError):
                h.validate_request(dict(self.request, targets=targets))

    def test_cancelled_request_never_imports_hardware(self):
        (self.job/'CANCEL').touch()
        with patch('batch_calibration.Radio', side_effect=AssertionError('Opened hardware')):
            with self.assertRaisesRegex(ValueError, 'Cancelled'):
                h.execute(self.request, self.job)

    def test_backend_reaudits_actual_saved_frames_and_reports_tail_loss(self):
        recording = self.session/'reports.jsonl'
        with recording.open('w') as out:
            out.write(json.dumps(dict(format='F04HID1', **self.state))+'\n')
            for seq in range(250):
                for slot in range(10):
                    if (seq,slot) != (249,8):
                        out.write(json.dumps(dict(report_hex=report(seq,slot).hex(),host_monotonic_ns=seq*20_000_000))+'\n')
            for slot in range(10):
                out.write(json.dumps(dict(report_hex=report(tracker=slot,end={'deferred':1}).hex(),host_monotonic_ns=5_000_000_000))+'\n')
            out.write(json.dumps(dict(event='capture-stop',confirmed=True,receiver_dropped=0))+'\n')
        result=h.execute(dict(self.request,action='audit',recording=str(recording)),self.job)
        quality=json.loads((Path(result['output'])/'quality.json').read_text())
        self.assertFalse(quality['transport_pass'])
        self.assertIn('tracker 8:', ' '.join(quality['failures']))
        self.assertEqual(json.loads((self.job/'progress.json').read_text())['stage'],'finished')

    def test_failed_preparation_preserves_inputs_but_revokes_plan(self):
        index=dict(plan='reviewed-plan.json',input='previous-input.json')
        (self.session/'helper-artifacts.json').write_text(json.dumps(index))
        with self.assertRaisesRegex(ValueError,'recording'):
            h.execute(dict(self.request,action='prepare',latitude=36.6),self.job)
        self.assertEqual(json.loads((self.session/'helper-artifacts.json').read_text()),
                         {k:v for k,v in index.items() if k!='plan'})
        self.assertFalse((self.session/'LOCK').exists())

    def test_wireless_cancel_still_sends_end_and_closes_session(self):
        class Link:
            def write(self,line):
                request=bytes.fromhex(line.decode().split()[1]);self.op=request[3]
                raw=bytearray(request);raw[0]=0x27;raw[16]=4;raw[17]=0
                raw[18:44]=request[44:48]+bytes(22)
                struct.pack_into('<I',raw,44,zlib.crc32(raw[:44]))
                self.pending=b'CALWIRE '+raw.hex().encode()+b'\n'
            def flush(self):pass
            @property
            def in_waiting(self):return len(self.pending)
            def read(self,n):
                value,self.pending=self.pending[:n],self.pending[n:]
                return value
        radio=Radio.__new__(Radio)
        radio.cancel_file=self.job/'CANCEL';radio.cancel_file.touch()
        radio.journal=type('Journal',(),{'add':lambda self,**kw:None})()
        radio.link=Link()
        with self.assertRaises(InterruptedError):radio.request(0,'000000000001',123,1)
        self.assertEqual(radio.request(0,'000000000001',123,5),b'')
        self.assertEqual(radio.link.op,5)

    def test_reviewed_plan_change_rejected_before_installer_or_radio(self):
        import hashlib
        raw=snapshot();backup=self.root/'backup.bin';backup.write_bytes(raw)
        sha=hashlib.sha256(raw).hexdigest()
        values=dict(scale=[1.01,.99,1.005],gsens_dps_per_g=[.001]*9,earth_dps=[0,.0041780746,0])
        ev=dict(mode='QUALITY_ACCEPTED',quality_gates_passed=True,unmet_quality_gates=[],
                reference_pose_confirmed='original +X-up face and original heading',decisions={})
        plan=dict(schema='CALWIRE_APPLY2',receiver_serial='TEST',build='unit',devices=[])
        for slot in range(10):
            plan['devices'].append(dict(slot=slot,rf_address=self.state['targets'][str(slot)],
                snapshot_path=str(backup),snapshot_sha256=sha,values=values))
            ev['decisions'][str(slot)]=dict(install_authorized=True,reason='Reviewed',snapshot_sha256=sha,values=values)
        evidence=self.root/'evidence.json';evidence.write_text(json.dumps(ev))
        plan.update(evidence_path=str(evidence),evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
        path=self.root/'plan.json';path.write_text(json.dumps(plan))
        reviewed=h.review_plan(path,self.state)
        path.write_text(json.dumps(plan,indent=2))
        with patch('helper_fleet.script',side_effect=AssertionError('Started installer')):
            with self.assertRaisesRegex(ValueError,'changed since Helper review'):
                h.execute(dict(self.request,action='apply',confirmed=True,plan=str(path),
                               plan_sha256=reviewed['sha256']),self.job)
        self.assertFalse((self.session/'helper-artifacts.json').exists())

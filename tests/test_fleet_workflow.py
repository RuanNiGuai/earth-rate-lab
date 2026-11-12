import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'tools/jointcal'))
import fleet_workflow as w
from calwire import unpack_snapshot
from example import snapshot
from batch_record_only import pose_ready


class WorkflowTests(unittest.TestCase):
    def test_saved_installation_is_bound_to_exact_plan_and_replaced_by_rollback(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);plan=root/'plan.json';plan.write_text('reviewed plan')
            report=root/'installed.json'
            report.write_text(json.dumps(dict(state='APPLY_READBACK_VERIFIED',plan_sha256=w.digest(plan))))
            state=w.read(root);state['artifacts']=dict(plan=str(plan),installation=str(report));w.save(root,state)
            self.assertEqual(w.presentation(root)['installation']['state'],'APPLY_READBACK_VERIFIED')
            report.write_text(json.dumps(dict(state='ROLLBACK_READBACK_VERIFIED',plan_sha256=w.digest(plan))))
            self.assertEqual(w.presentation(root)['installation']['state'],'ROLLBACK_READBACK_VERIFIED')
            plan.write_text('different reviewed plan')
            self.assertIsNone(w.presentation(root)['installation'])

    def rejected_pose(self,root):
        source=root/'raw.jsonl';source.write_text('immutable capture')
        binding=dict(receiver_serial='test',targets={str(i):f'{i+1:012X}' for i in range(10)})
        (root/'session.json').write_text(json.dumps(binding))
        entry=dict(kind='poses',recording=str(source),sha256=w.digest(source),accepted=False,error='old order check')
        w.save(root,dict(schema='FLEET_WIZARD1',captures={k:dict(accepted=True,recording='old') for k in ('x','y','z')},
                        history=[entry],artifacts={'plan':'stale'}))
        return source,binding,entry

    def test_old_rejected_pose_is_reanalyzed_once_without_erasing_history(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source,binding,old=self.rejected_pose(root)
            with patch.object(w.continuous,'analyze',return_value=([],binding,{'observed_faces':['+X']})) as analyze:
                self.assertTrue(w.reanalyze_rejected_pose(root))
                self.assertFalse(w.reanalyze_rejected_pose(root))
                analyze.assert_called_once_with(source,'poses')
            state=w.read(root)
            self.assertEqual(state['history'][0],old)
            self.assertTrue(state['captures']['poses']['accepted'])
            self.assertEqual(state['history'][-1]['analysis_version'],w.continuous.POSE_ANALYSIS_VERSION)
            self.assertEqual(json.loads((root/'helper-artifacts.json').read_text()),{})
            self.assertEqual(w.digest(source),old['sha256'])

    def test_reanalysis_keeps_quality_failures_and_does_not_retry_each_open(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);self.rejected_pose(root)
            with patch.object(w.continuous,'analyze',side_effect=ValueError('missing face')) as analyze:
                with self.assertRaisesRegex(ValueError,'missing face'):w.reanalyze_rejected_pose(root)
                self.assertFalse(w.reanalyze_rejected_pose(root))
                self.assertEqual(analyze.call_count,1)
            self.assertEqual(len(w.read(root)['history']),2)
            self.assertNotIn('poses',w.read(root)['captures'])

    def test_changed_rejected_recording_is_not_reanalyzed(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source,_,_=self.rejected_pose(root)
            source.write_text('changed')
            with patch.object(w.continuous,'analyze') as analyze:
                with self.assertRaisesRegex(ValueError,'已改变'):w.reanalyze_rejected_pose(root)
                analyze.assert_not_called()
            self.assertEqual(len(w.read(root)['history']),1)

    def test_failed_preflight_diagnostic_is_utf8_even_under_legacy_windows_encoding(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);session=root/'session';session.mkdir()
            (session/'session.json').write_text(json.dumps({'initialized':False}))
            request=root/'request.json'
            request.write_text(json.dumps(dict(schema='HELPER_FLEET1',action='workflow-open',session=str(session))))
            run=subprocess.run([sys.executable,str(ROOT/'tools/helper_fleet.py'),'--request',str(request)],
                               env=dict(os.environ,PYTHONIOENCODING='gbk'),stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
            self.assertNotEqual(run.returncode,0)
            self.assertIn('请先准备并核验十台会话',run.stdout.decode('utf8'))

    def test_ready_requires_all_ten_still_in_the_selected_orientation(self):
        rows={i:((0,0,0),20000,(1000,0,0)) for i in range(10)}
        self.assertTrue(pose_ready(rows,'x'))
        self.assertTrue(pose_ready(rows,'poses'))
        self.assertFalse(pose_ready(rows,'y'))
        rows[7]=((30000,0,0),20000,(1000,0,0))
        self.assertFalse(pose_ready(rows,'x'))
        rows[7]=((0,0,0),20000,(-1000,0,0))
        self.assertFalse(pose_ready(rows,'poses'))
        del rows[7]
        self.assertFalse(pose_ready(rows,'x'))

    def test_failed_repeat_preserves_history_and_clears_installable_artifacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'record.jsonl';source.write_text('new capture')
            w.save(root,dict(schema='FLEET_WIZARD1',captures={'x':{'accepted':True,'recording':'old.jsonl'}},
                            history=[{'recording':'old.jsonl','accepted':True}],artifacts={'plan':'old-plan.json'}))
            with patch.object(w.continuous,'analyze',side_effect=ValueError('Missing end manifest')):
                with self.assertRaisesRegex(ValueError,'Missing end'):
                    w.accept_capture(root,'x',source)
            state=w.read(root)
            self.assertEqual(state['artifacts'],{})
            self.assertNotIn('x',state['captures'])
            self.assertEqual(len(state['history']),2)
            self.assertFalse(state['history'][-1]['accepted'])
            self.assertEqual(source.read_text(),'new capture')
            with self.assertRaisesRegex(ValueError,'X'):w.admit(state,'y')

    def test_changed_capture_refuses_prepare_before_processing(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'raw.jsonl';source.write_text('original')
            entry=dict(recording=str(source),sha256=w.digest(source))
            w.save(root,dict(schema='FLEET_WIZARD1',captures={k:dict(entry) for k in w.STAGES},history=[],artifacts={'result':'old'}))
            w.b.atomic_save(root/'helper-artifacts.json',{'plan':'old-plan'})
            source.write_text('altered')
            with patch.object(w.continuous,'prepare',side_effect=AssertionError('processed altered capture')):
                with self.assertRaisesRegex(ValueError,'改变'):w.prepare(root,root,45)
            self.assertEqual(w.read(root)['artifacts'],{})
            self.assertEqual(json.loads((root/'helper-artifacts.json').read_text()),{})

    def test_continuous_snapshot_does_not_require_old_endpoint_archives(self):
        blob=snapshot([1,1,1]);parts=[];position=0
        while position<len(blob):
            kind,size=struct.unpack_from('<HH',blob,position)
            if kind<100:parts.append(blob[position:position+4+size])
            position+=4+size
        no_archive=b''.join(parts)
        with self.assertRaisesRegex(ValueError,'metadata'):unpack_snapshot(no_archive)
        parsed=unpack_snapshot(no_archive,require_scales=False)
        self.assertEqual(parsed['scale_records_hex'],[])
        self.assertEqual(parsed['production']['scale'],[1,1,1])
        self.assertTrue(parsed['earth_valid'])
        with self.assertRaises(ValueError):unpack_snapshot(no_archive[:-1],require_scales=False)


if __name__=='__main__':unittest.main()

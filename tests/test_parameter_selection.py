"""Selected-field admission, stale evidence and transaction recovery, without hardware."""
import copy
import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import unittest
from unittest.mock import patch
import test_plan_review
import helper_fleet as h
import inputs
import select_parameters as selection
import calwire_apply as c
import fleet_workflow as workflow
from test_calwire_apply import Fake, Journal, snapshot


class SelectionTests(unittest.TestCase):
    setUp=test_plan_review.PlanReviewTests.setUp

    def selected(self):
        old=c.unpack_snapshot(snapshot())['production']
        for d in self.data:d['backup']['production']=copy.deepcopy(old)
        self.fit['static']['devices'][5]['earth_holdout_rms_dps']=.00329
        self.result.write_text(json.dumps(self.fit))
        decisions=[]
        for slot,new in enumerate(self.fit['static']['devices']):
            values=copy.deepcopy(old)
            if slot!=9:values['scale']=selection.np.asarray(new['scale'],dtype='float32').tolist()
            decisions.append(dict(slot=slot,values=values,scale_updated=slot!=9,
                turn_rms_deg=dict(old=10.,new=2.),replay_before=dict(closure_rms_deg=1.,rest_rms_deg_per_min=.2),
                replay_after=dict(closure_rms_deg=.9,rest_rms_deg_per_min=.19),reasons=['fixture']))
        report=dict(schema='JOINT_PARAMETER_SELECTION1',version=selection.VERSION,
                    input_sha256=inputs.digest(self.input),result_sha256=inputs.digest(self.result),
                    source_sha256=selection.source_hashes(),snapshot_hashes=self.fit['snapshot_hashes'],
                    decisions=decisions,limitations=[],runs={})
        self.selected_path=self.session/'selection.json';selection.save(self.selected_path,report)
        return report

    def plan(self):
        report=self.selected()
        self.artifacts['selection']=str(self.selected_path)
        h.b.atomic_save(self.session/'helper-artifacts.json',self.artifacts)
        w=workflow.read(self.session);w['artifacts']=self.artifacts.copy();workflow.save(self.session,w)
        job=self.root/'job';job.mkdir()
        with patch.object(inputs,'configure',return_value=self.config),patch.object(inputs,'load',return_value=self.data),redirect_stdout(io.StringIO()):
            result=h.execute(dict(schema='HELPER_FLEET1',action='plan',session=str(self.session),confirmed=True,
                                  selection_mode='validated-parts-v1'),job)
        return result,report

    def test_partial_plan_and_resume_keep_failed_earth_diagnostic(self):
        result,_=self.plan()
        self.assertTrue(result['installable'])
        self.assertFalse(result['quality_review']['full_candidate_quality_gates_passed'])
        plan=h.read_json(result['reviewed_plan']['path']);entries=c.prepare(plan,self.state)
        self.assertEqual([len(e['steps']) for e in entries],[1]*9+[0])
        for e in entries:
            at=c.profile_location(e['before'])
            self.assertEqual(e['before'][at+12:at+64],e['after'][at+12:at+64])
        job=self.root/'resume';job.mkdir()
        with patch('calwire.Radio',side_effect=AssertionError('hardware')):
            resumed=h.execute(dict(schema='HELPER_FLEET1',action='workflow-open',session=str(self.session)),job)
        self.assertTrue(resumed['installable'])
        self.assertEqual(resumed['reviewed_plan']['sha256'],result['reviewed_plan']['sha256'])

    def test_scale_only_lost_ack_restores_all_changed_devices(self):
        result,_=self.plan();entries=c.prepare(h.read_json(result['reviewed_plan']['path']),self.state)
        radio=Fake(entries,fail=(5,1),commit=True)
        with redirect_stdout(io.StringIO()),self.assertRaises(TimeoutError):c.apply_entries(radio,entries,Journal())
        self.assertTrue(all(radio.blobs[e['slot']]==e['before'] for e in entries))
        self.assertTrue(all(field==1 for _,field in radio.writes))
        radio=Fake(entries);radio.blobs[9]=radio.blobs[9][:-1]+b'!'
        with self.assertRaises(ValueError):c.apply_entries(radio,entries,Journal())
        self.assertEqual(radio.writes,[])

    def test_retained_field_cannot_change_even_with_matching_evidence(self):
        result,_=self.plan();plan=h.read_json(result['reviewed_plan']['path'])
        evidence=h.read_json(plan['evidence_path'])
        plan['devices'][0]['values']['gsens_dps_per_g'][0]=.001
        evidence['decisions']['0']['values']=plan['devices'][0]['values']
        selection.save(plan['evidence_path'],evidence);plan['evidence_sha256']=inputs.digest(plan['evidence_path'])
        with self.assertRaisesRegex(ValueError,'Retained parameter'):c.prepare(plan,self.state)

    def test_stale_source_nonfinite_and_direction_failure_reject_selection(self):
        report=self.selected()
        for alter in (lambda r:r.update(source_sha256={}),
                      lambda r:r.update(input_sha256='stale'),
                      lambda r:r['decisions'][0]['replay_after'].update(closure_rms_deg=float('nan')),
                      lambda r:r['decisions'][0]['values']['earth_dps'].__setitem__(0,.002)):
            changed=copy.deepcopy(report);alter(changed)
            self.selected_path.write_text(json.dumps(changed),encoding='utf8')
            with self.assertRaises(ValueError):selection.load_selection(self.selected_path,self.input,self.result,self.data,self.fit)
        selection.save(self.selected_path,report)
        self.fit['iterations'][-1]['scale']['direction_checks'][0]['passed']=False
        with self.assertRaisesRegex(ValueError,'bidirectional'):selection.load_selection(self.selected_path,self.input,self.result,self.data,self.fit)

    def test_failed_comparison_revokes_previous_plan_on_disk(self):
        result,_=self.plan()
        job=self.root/'retry';job.mkdir()
        with patch.object(selection,'compare',side_effect=ValueError('failed comparison')):
            with self.assertRaisesRegex(ValueError,'failed comparison'):
                h.execute(dict(schema='HELPER_FLEET1',action='compare',session=str(self.session)),job)
        for index in (h.read_json(self.session/'helper-artifacts.json'),workflow.read(self.session)['artifacts']):
            self.assertNotIn('plan',index);self.assertNotIn('selection',index)
            self.assertIn('result',index)


if __name__=='__main__':unittest.main()

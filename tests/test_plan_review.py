"""Quality rejection is a durable review, never an implicit trial installation."""
import copy
import hashlib
import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/jointcal')]
import helper_fleet as h
import fleet_workflow as workflow
import inputs
import make_plan
from refinement import LIMITS
from test_calwire_apply import snapshot


class PlanReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.session=self.root/'session';self.session.mkdir()
        self.state=dict(initialized=True,receiver_serial='TEST',targets={str(i):f'{i+1:012X}' for i in range(10)})
        (self.session/'session.json').write_text(json.dumps(self.state))
        self.input=self.session/'input.json';self.input.write_text('{}')
        self.result=self.session/'fit.json'
        self.config=dict(dataset_kind='measured',transport_pass=True,
                         public_description=dict(config_sha256=inputs.digest(self.input)),devices=[],scale_captures=True)
        self.data=[];raw=snapshot();sha=hashlib.sha256(raw).hexdigest()
        for slot in range(10):
            folder=self.session/f'slot-{slot}';folder.mkdir();path=folder/'snapshot.bin';path.write_bytes(raw)
            metadata=dict(slot=slot,rf_address=self.state['targets'][str(slot)],snapshot_sha256=sha)
            (folder/'backup.json').write_text(json.dumps(metadata))
            self.config['devices'].append(dict(snapshot=path,snapshot_sha256=sha))
            from calwire import unpack_snapshot
            self.data.append(dict(backup={**unpack_snapshot(raw,require_scales=False), 'snapshot_sha256':sha}))
        devices=[dict(slot=i,scale=[1.01,.99,1.005],gsens_dps_per_g=[[.001]*3]*3,earth_dps=[0,.0041780746,0],
                      earth_train_rms_dps=.0007,earth_holdout_rms_dps=.0008,
                      gsens_train_rms_dps=.001,gsens_holdout_rms_dps=.002,
                      design_rank=5,design_condition=20.) for i in range(10)]
        self.fit=dict(solver_version='adaptive-window-rigid-1',numerics_version='sensor-residual-window20ms-2',
                      convergence=dict(converged=True,iterations=4,history=[
                          dict(iteration=i+1,objective=1.,inner_solvers_converged=True,changes=dict.fromkeys(LIMITS,0.))
                          for i in range(4)]), input_config=self.config['public_description'],snapshot_hashes=[sha]*10,
                      static=dict(devices=devices,horizontal_free_magnitude_ratio=1.,
                                  earth_information=dict(projected_rank=2,conditional_phase_std_deg=1.)),
                      iterations=[dict(scale=dict(bidirectional=True,success=True,joint_angle_rms_deg=2.,
                          direction_checks=[dict(slot=i,relative_difference=[.0001]*3,passed=True) for i in range(10)]))],
                      earth_loop_closure_deg=.1,coverage=[dict(slot=i,covered=True) for i in range(10)])
        self.artifacts=dict(input=str(self.input),result=str(self.result),plan='old-plan.json')
        h.b.atomic_save(self.session/'helper-artifacts.json',self.artifacts)
        workflow.save(self.session,dict(schema='FLEET_WIZARD1',captures={},history=[],artifacts=self.artifacts.copy()))

    def bridge(self):
        self.result.write_text(json.dumps(self.fit))
        job=self.root/'job';job.mkdir()
        with patch.object(inputs,'configure',return_value=self.config),patch.object(inputs,'load',return_value=self.data),\
             patch('calwire.Radio',side_effect=AssertionError('Opened hardware')),redirect_stdout(io.StringIO()):
            return h.execute(dict(schema='HELPER_FLEET1',action='plan',session=str(self.session),confirmed=True),job)

    def test_failed_review_preserves_evidence_and_clears_stale_plan_across_resume(self):
        self.fit['static']['devices'][5]['earth_holdout_rms_dps']=.00329
        result=self.bridge()
        self.assertEqual(result['state'],'PLAN_REJECTED')
        self.assertNotIn('reviewed_plan',result)
        self.assertNotIn('plan',result['artifacts'])
        self.assertNotIn('plan',workflow.read(self.session)['artifacts'])
        self.assertIn('#5',result['failures'][1]);self.assertIn('0.00329',result['failures'][1])
        self.assertIn('0.001',result['failures'][1])
        folder=Path(result['output'])
        self.assertFalse((folder/'plan.json').exists());self.assertFalse((folder/'evidence.json').exists())
        self.assertTrue((folder/'review.md').exists())
        self.assertEqual(json.loads(self.result.read_text()),self.fit)
        resume=self.root/'resume';resume.mkdir()
        with patch('calwire.Radio',side_effect=AssertionError('Opened hardware')):
            restored=h.execute(dict(schema='HELPER_FLEET1',action='workflow-open',session=str(self.session)),resume)
        self.assertEqual(restored['state'],'PLAN_REJECTED')
        self.assertEqual(restored['failures'],result['failures'])
        self.assertFalse(restored['installable'])

    def test_full_raw_candidate_is_diagnostic_even_when_fit_quality_passes(self):
        result=self.bridge()
        self.assertEqual(result['state'],'PLAN_REJECTED')
        self.assertNotIn('reviewed_plan',result)
        self.assertTrue(any('M/E/B' in r for r in result['failures']))
        self.assertFalse(list(self.session.glob('helper-plan-*/plan.json')))

    def test_cli_without_review_option_retains_nonzero_failure_and_report(self):
        self.fit['static']['devices'][1]['earth_train_rms_dps']=.0014
        self.result.write_text(json.dumps(self.fit));out=self.root/'cli'
        args=['make_plan','--config',str(self.input),'--result',str(self.result),'--session',str(self.session),
              '--out',str(out),'--reference-confirmed','--authorization','Reviewed test']
        with patch.object(sys,'argv',args),patch.object(make_plan,'configure',return_value=self.config),\
             patch.object(make_plan,'load',return_value=self.data):
            with self.assertRaisesRegex(ValueError,'0.0014'):make_plan.main()
        self.assertTrue((out/'review.json').exists());self.assertFalse((out/'plan.json').exists())

    def test_quality_limits_remain_exact_and_nonfinite_is_rejected(self):
        fit=copy.deepcopy(self.fit)
        fit['static']['devices'][0].update(earth_train_rms_dps=.001,earth_holdout_rms_dps=.001,
                                        gsens_train_rms_dps=.006,gsens_holdout_rms_dps=.006)
        self.assertTrue(make_plan.assess_quality(fit,self.config)['quality_gates_passed'])
        fit['static']['devices'][0]['earth_train_rms_dps']=.001000001
        self.assertFalse(make_plan.assess_quality(fit,self.config)['quality_gates_passed'])
        fit['static']['devices'][0]['earth_train_rms_dps']=float('nan')
        report=make_plan.assess_quality(fit,self.config)
        self.assertFalse(report['quality_gates_passed'])
        json.dumps(report,allow_nan=False)

    def test_identity_mismatch_is_not_converted_to_quality_rejection(self):
        self.fit['snapshot_hashes'][0]='mismatch'
        with self.assertRaisesRegex(ValueError,'snapshot mismatch'):self.bridge()
        self.assertFalse(list(self.session.glob('helper-plan-*/review.json')))
        self.assertEqual(h.read_json(self.session/'helper-artifacts.json'),
                         {k:v for k,v in self.artifacts.items() if k!='plan'})


if __name__=='__main__':unittest.main()

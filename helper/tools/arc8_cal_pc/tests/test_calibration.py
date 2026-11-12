import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import calibration as m

class CalibrationPcTests(unittest.TestCase):
    def setUp(self): self.doc=m.make_template()

    def test_native_compiler_abi_and_pinned_provenance(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as d:
            library=m.compile_solver(d)
            self.assertEqual(library.arc8_version(),2)
            self.assertEqual(library.arc8_magic(),0x43414c31)

    def test_template_roundtrips_and_complete_counts(self):
        assembled=m.assemble(self.doc)
        self.assertEqual(assembled['earth'].count,5)
        self.assertEqual(assembled['gsens'].count,6)
        self.assertEqual(assembled['scale'].count,3)

    def test_sample_is_rejected_instead_of_faking_solution(self):
        with self.assertRaisesRegex(ValueError,'solver rejected'):
            m.solve(self.doc)

    def test_missing_one_pose_cannot_be_solved(self):
        self.doc['archive']['gsens']['count']=5
        with self.assertRaisesRegex(ValueError,'incomplete'):
            m.assemble(self.doc)

    def test_reference_mismatch_rejected(self):
        self.doc['archive']['earth']['reference_id']=9
        with self.assertRaisesRegex(ValueError,'reference_id mismatch'):
            m.assemble(self.doc)

    def test_wrong_hardware_refused(self):
        self.doc['target']['board']='unknown'
        with self.assertRaisesRegex(ValueError,'wrong target board'):
            m.assemble(self.doc)

    def test_missing_device_identity_refused(self):
        self.doc['target']['tracker_uid']=''
        with self.assertRaisesRegex(ValueError,'bad target'):
            m.assemble(self.doc)

    def test_bad_abi_version_refused(self):
        self.doc['archive']['scale']['version']=3
        with self.assertRaisesRegex(ValueError,'magic/version'):
            m.assemble(self.doc)

    def test_nan_and_infinity_rejected(self):
        for value in (float('nan'),float('inf'),-float('inf')):
            self.doc['archive']['earth']['latitude_deg']=value
            with self.assertRaisesRegex(ValueError,'finite'):
                m.assemble(self.doc)

    def test_truncated_motion_segment_refused(self):
        self.doc['archive']['earth']['travel'][0]['seg']=[]
        with self.assertRaisesRegex(ValueError,'exactly 12'):
            m.assemble(self.doc)

    def test_unknown_field_rejected(self):
        self.doc['archive']['earth']['_unsafe']=1
        with self.assertRaisesRegex(ValueError,'unexpected'):
            m.assemble(self.doc)

    def test_no_write_or_hid_in_module(self):
        code=Path(m.__file__).read_text(encoding='utf8')
        self.assertNotIn('hid.device(',code)
        self.assertNotIn('nvs_write(',code)
        self.assertNotIn('boot_request_upgrade(',code)
        self.assertNotIn('serial.Serial(',code)
        self.assertNotIn('subprocess.run(["west"',code)

    def test_bundle_integrity_tamper_and_state_gate(self):
        candidate={
            'schema':m.FORMAT,'target':self.doc['target'],
            'provenance':{'capture_sha256':'a'*64, 'solver_c_sha256':m.CORE_SHA256, 'reference_id':1,'capture_note':'fixture'},
            'solution':{'scale':[1.,1.,1.], 'earth_body':[0.004,0.,0.], 'gsens':[0.]*9,
                        'base_bias':[0.]*3, 'scale_error':[0.]*3,'earth_rms':0.,'gsens_rms':0.,'iterations':5},
            'deployment':{'state':'OFFLINE_CANDIDATE_UNAPPLIED','transport_enabled':False,
                          'needs_tracker_staged_import':True,'nvs_readback_verified':False,
                          'reboot_verified':False},
            'created_utc':'2026-10-31T00:00:00Z'}
        candidate['payload_sha256']=m.digest(candidate)
        self.assertTrue(m.verify_bundle(candidate))
        tampered=copy.deepcopy(candidate)
        tampered['solution']['gsens'][0]=0.01
        with self.assertRaisesRegex(ValueError,'SHA256'):
            m.verify_bundle(tampered)
        tampered=copy.deepcopy(candidate)
        tampered['deployment']['nvs_readback_verified']=True
        tampered['payload_sha256']=m.digest({k:v for k,v in tampered.items() if k!='payload_sha256'})
        with self.assertRaisesRegex(ValueError,'deploy state'):
            m.verify_bundle(tampered)

    def test_no_output_file_for_bad_capture(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as d:
            inp=Path(d)/'source.json'
            out=Path(d)/'bundle.json'
            inp.write_text(json.dumps(self.doc))
            self.assertEqual(m.main(['solve',str(inp),str(out)]),2)
            self.assertFalse(out.exists())

if __name__=='__main__': unittest.main()

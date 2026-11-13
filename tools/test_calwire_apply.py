"""Fault injection for actual multi-record plan, parser, readback and rollback."""
from contextlib import contextmanager
import copy,json,struct,tempfile,unittest,zlib
from pathlib import Path
import calwire_apply as c

def snapshot():
    def tlv(k,v):return struct.pack('<HH',k,len(v))+v
    def crc(v):return bytes(v[:-4])+struct.pack('<I',zlib.crc32(v[:-4]))
    profile=struct.pack('<15f4B',1,1,1,*([0]*9),.0041780746,0,0,1,0,0,1)
    thermal=struct.pack('<5H2B',2,140,113,2,123,1,0)
    thermal+=struct.pack('<8f',22.1,0,0,0,22.3,0,0,0)+bytes(138*16)
    thermal+=b''.join(struct.pack('<4f',15+.25*i,0,0,0) for i in range(113))
    meta=bytearray(32);struct.pack_into('<4I',meta,0,0x41524334,4,1,9)
    result=tlv(1,b'unit\0')+tlv(2,profile)+tlv(3,thermal)+tlv(4,bytes([1,1,1,1])+struct.pack('<12f',0,0,0,1,0,0,0,1,0,0,0,1))+tlv(100,crc(meta))
    for axis in range(3):
        raw=bytearray(584);struct.pack_into('<5I2B',raw,0,0x41353252,2,1,9,1,1,axis)
        vals=[0.]*22;vals[14]=60
        struct.pack_into('<22f4B',raw,488,*vals,axis,20,0,0)
        result+=tlv(104+2*axis,crc(raw))
    c.unpack_snapshot(result);return result

class Journal:
    def __init__(self):self.events=[]
    def add(self,**kw):self.events.append(kw)

class Fake:
    def __init__(self,entries,fail=None,commit=False):
        self.blobs={e['slot']:e['before'] for e in entries};self.fail=fail;self.commit=commit
        self.failed=False;self.writes=[];self.token=0;self.frozen={};self.bad_read=False
    @contextmanager
    def session(self,slot,address):
        self.token+=1;token=self.token;blob=self.blobs[slot];self.frozen[token]=blob
        try:yield token,len(blob),zlib.crc32(blob)
        finally:del self.frozen[token]
    def request(self,slot,address,token,op,offset):
        assert op==1
        raw=self.frozen[token][offset:offset+22]
        return bytes([raw[0]^1])+raw[1:] if self.bad_read else raw
    def stage_field(self,slot,address,token,before,field,values,apply=False):
        assert apply
        actual=c.unpack_snapshot(self.blobs[slot])
        assert actual['profile_bytes']==before['profile_bytes'] and actual['thermal_crc32']==before['thermal_crc32']
        fail=not self.failed and (slot,field)==self.fail
        if fail and not self.commit:self.failed=True;raise TimeoutError('before commit')
        _,start,size=next(v for v in c.FIELDS.values() if v[0]==field)
        blob=bytearray(self.blobs[slot]);p=c.profile_location(blob)+start
        blob[p:p+size]=struct.pack('<'+'f'*(size//4),*values);self.blobs[slot]=bytes(blob)
        self.writes.append((slot,field))
        if fail:self.failed=True;raise TimeoutError('commit completed; ACK lost')

class InstallTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);root=Path(self.tmp.name)
        raw=snapshot();snap=root/'before.bin';snap.write_bytes(raw)
        self.state={'receiver_serial':'TEST','targets':{'0':'000000000001','1':'000000000002'}}
        values=dict(scale=[1.01,.99,1.005],gsens_dps_per_g=[.001]*9,earth_dps=[0,.0041780746,0])
        self.evidence=dict(mode='USER_AUTHORIZED_TRIAL',user_authorization_quote='Install for trial',
            quality_gates_passed=False,unmet_quality_gates=['per-device Earth RMS'],
            reference_pose_confirmed='original +X-up face and original heading',decisions={})
        self.evpath=root/'evidence.json';self.plan=dict(schema='CALWIRE_APPLY2',receiver_serial='TEST',
            build='unit',evidence_path=str(self.evpath),devices=[])
        for slot in range(2):
            self.plan['devices'].append(dict(slot=slot,rf_address=self.state['targets'][str(slot)],
                snapshot_path=str(snap),snapshot_sha256=c.digest(raw),values=values))
            self.evidence['decisions'][str(slot)]=dict(install_authorized=True,reason='Trial',
                snapshot_sha256=c.digest(raw),values=values)
        self.refresh();self.entries=c.prepare(self.plan,self.state);self.journal=Journal()
    def refresh(self):
        self.evpath.write_text(json.dumps(self.evidence));self.plan['evidence_sha256']=c.digest(self.evpath.read_bytes())
    def test_all_three_fields_and_exact_readback(self):
        radio=Fake(self.entries);c.apply_entries(radio,self.entries,self.journal)
        self.assertEqual(len(radio.writes),6)
        for e in self.entries:self.assertEqual(radio.blobs[e['slot']],e['after'])
    def test_failure_before_commit_rolls_back_prior_devices_and_fields(self):
        radio=Fake(self.entries,(1,2))
        with self.assertRaises(TimeoutError):c.apply_entries(radio,self.entries,self.journal)
        for e in self.entries:self.assertEqual(radio.blobs[e['slot']],e['before'])
    def test_lost_ack_after_commit_rolls_back(self):
        radio=Fake(self.entries,(1,3),True)
        with self.assertRaises(TimeoutError):c.apply_entries(radio,self.entries,self.journal)
        for e in self.entries:self.assertEqual(radio.blobs[e['slot']],e['before'])
    def test_every_partial_prefix_restores(self):
        for count in range(4):
            radio=Fake(self.entries);e=self.entries[0];radio.blobs[0]=e['states'][count]
            c.restore(radio,e,self.journal);self.assertEqual(radio.blobs[0],e['before'])
    def test_foreign_change_never_clobbered(self):
        radio=Fake(self.entries);raw=bytearray(radio.blobs[0]);raw[-1]^=1;radio.blobs[0]=bytes(raw)
        with self.assertRaisesRegex(ValueError,'unrecognized'):c.restore(radio,self.entries[0],self.journal)
        self.assertEqual(radio.writes,[]);self.assertEqual(radio.blobs[0],bytes(raw))
    def test_preflight_whole_fleet_before_writes(self):
        radio=Fake(self.entries);raw=bytearray(radio.blobs[1]);raw[-1]^=1;radio.blobs[1]=bytes(raw)
        with self.assertRaises(ValueError):c.apply_entries(radio,self.entries,self.journal)
        self.assertEqual(radio.writes,[])
    def test_repeated_install_reads_back_without_writing_again(self):
        radio=Fake(self.entries);c.apply_entries(radio,self.entries,self.journal);radio.writes=[]
        result=c.apply_entries(radio,self.entries,self.journal)
        self.assertEqual(radio.writes,[])
        self.assertEqual(result,dict(already_applied_slots=[0,1],written_slots=[]))
    def test_completed_peers_are_not_rolled_back_when_retry_fails(self):
        radio=Fake(self.entries,fail=(1,2),commit=True);radio.blobs[0]=self.entries[0]['after']
        with self.assertRaises(TimeoutError):c.apply_entries(radio,self.entries,self.journal)
        self.assertEqual(radio.blobs[0],self.entries[0]['after'])
        self.assertEqual(radio.blobs[1],self.entries[1]['before'])
        self.assertTrue(all(slot==1 for slot,_ in radio.writes))
    def test_partial_prefix_requires_recovery_before_any_new_write(self):
        radio=Fake(self.entries);radio.blobs[1]=self.entries[1]['states'][1]
        with self.assertRaisesRegex(ValueError,'Tracker #1.*partial'):c.apply_entries(radio,self.entries,self.journal)
        self.assertEqual(radio.writes,[])
    def test_crc_alone_cannot_replace_actual_profile_read(self):
        radio=Fake(self.entries);radio.bad_read=True
        with self.assertRaisesRegex(ValueError,'readback'):c.apply_entries(radio,self.entries,self.journal)
        self.assertEqual(radio.writes,[])
    def test_trial_cannot_claim_quality_pass(self):
        self.evidence['quality_gates_passed']=True;self.refresh()
        with self.assertRaises(ValueError):c.prepare(self.plan,self.state)
    def test_trial_requires_authorization(self):
        self.evidence['user_authorization_quote']='';self.refresh()
        with self.assertRaises(ValueError):c.prepare(self.plan,self.state)
    def test_reference_pose_required(self):
        self.evidence['reference_pose_confirmed']='unknown';self.refresh()
        with self.assertRaises(ValueError):c.prepare(self.plan,self.state)
    def test_entire_fleet_required(self):
        self.plan['devices'].pop()
        with self.assertRaises(ValueError):c.prepare(self.plan,self.state)
    def test_evidence_digest_binds_decision(self):
        self.evpath.write_text('{}')
        with self.assertRaisesRegex(ValueError,'Evidence changed'):c.prepare(self.plan,self.state)
    def test_invalid_earth_norm(self):
        self.plan['devices'][0]['values']['earth_dps']=[.004,0,0];self.refresh()
        with self.assertRaisesRegex(ValueError,'magnitude'):c.prepare(self.plan,self.state)

if __name__=='__main__':unittest.main()

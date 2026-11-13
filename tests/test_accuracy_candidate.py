"""Production snapshots, affine calibration and crash recovery without devices."""
import copy
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/jointcal')]
from calwire import unpack_snapshot
from calibration_coordinates import installed_accel, raw_gsens, convert_raw_model
from test_calwire_apply import snapshot, Fake, Journal
import test_calwire_apply as install_fixture
import calwire_apply as c
from verify_persistence import compare
from native_test import compile_run
from fusion_replay import CORE, FUSION, extract_function, Replay, orientations
from test_continuous import axis_data
from continuous import align_clocks


def with_accel(blob, values=None):
    parts=[];p=0
    while p<len(blob):
        k,n=struct.unpack_from('<HH',blob,p)
        if k!=4:parts.append(blob[p:p+4+n])
        p+=4+n
    if values is not None:
        parts.append(struct.pack('<HH4B12f',4,52,1,1,1,1,*values))
    return b''.join(parts)


class CoordinateTests(unittest.TestCase):
    def test_cross_axis_affine_conversion_includes_constant_bias(self):
        a=np.array([[1.01,.02,-.01],[.003,.99,.004],[.002,-.007,1.02]])
        b=np.array([.013,-.006,.008])
        parsed=unpack_snapshot(with_accel(snapshot(),[*b,*a.ravel()]))
        rng=np.random.default_rng(12);raw=rng.normal(size=(80,3))
        # Use transmitted float32 coefficients for the expected affine map.
        a=np.asarray(parsed['accelerometer']['matrix']);b=np.asarray(parsed['accelerometer']['offset_g'])
        np.testing.assert_allclose(installed_accel(raw,parsed),(a@(raw-b).T).T)
        m=rng.normal(size=(3,3))*.003;constant=np.array([.01,-.02,.03])
        mc,bc=convert_raw_model(m,constant,parsed)
        np.testing.assert_allclose(installed_accel(raw,parsed)@mc.T+bc,raw@m.T+constant,atol=1e-15)
        parsed['production']['gsens_dps_per_g']=m.ravel().tolist()
        np.testing.assert_allclose(raw_gsens(parsed),m@a)

    def test_legacy_unknown_singular_and_nonfinite_are_not_installed_baselines(self):
        old=unpack_snapshot(with_accel(snapshot()))
        self.assertFalse(old['accelerometer']['known'])
        with self.assertRaisesRegex(ValueError,'unknown'):installed_accel([[0,0,1]],old)
        for vals in ([0]*12,[float('nan')]+[0]*11):
            with self.assertRaises(ValueError):unpack_snapshot(with_accel(snapshot(),vals))
        report=compare(with_accel(snapshot()),snapshot())
        self.assertTrue(report['parameters_match']);self.assertFalse(report['accelerometer_compared'])
        self.assertFalse(report['reboot_verified'])

    def test_persistence_detects_matrix_changes(self):
        changed=with_accel(snapshot(),[.01,0,0,1,0,0,0,1,0,0,0,1])
        self.assertEqual(compare(snapshot(),changed)['changed_fields'],['accelerometer'])


class ClockTests(unittest.TestCase):
    def test_batch_arrival_and_latency_do_not_become_sample_intervals(self):
        baseline=axis_data();delayed=copy.deepcopy(baseline)
        # 200 ms host batching plus per-device random positive latency. Keep
        # occasional low-latency arrivals for the existing envelope reference.
        rng=np.random.default_rng(3)
        for d in delayed:
            d['host']+=rng.uniform(0,.2,len(d['host']))
            d['host'][::10]=np.ceil(d['host'][::10]*5)/5
        first=align_clocks(baseline);second=align_clocks(delayed)
        for a,b in zip(baseline,delayed):
            np.testing.assert_array_equal(a['duration'],b['duration'])
            np.testing.assert_array_equal(a['gyro'],b['gyro'])
            np.testing.assert_array_equal(a['seq'],b['seq'])
        self.assertLess(max(abs(a['clock_rate_correction_ppm']-b['clock_rate_correction_ppm']) for a,b in zip(first,second)),100)
        self.assertLess(np.ptp([d['time'][2000] for d in delayed]),.015)


class SnapshotTests(unittest.TestCase):
    def test_versioned_pair_roundtrip_and_continuation_in_both_realizations(self):
        wrapper=(FUSION/'vqf/vqf.c').read_text(encoding='utf8')
        # Include the actual retained-ID function; the payload already fills
        # 512 bytes and is protected by the existing retained-data CRC.
        start=wrapper.index('uint8_t vqf_retained_id(void)');end=wrapper.index('\n}',start)+2
        code=r'''
#include <assert.h>
#include "vqf.c"
#include "earth_pair.c"
RETAINED_ID
int main(void) {
    assert(vqf_retained_id()==0x40+VQF_STATE_VERSION);
    assert(vqf_retained_id()!=2); /* unversioned legacy ID */
    vqf_params_t p;vqf_state_t s;vqf_coeffs_t c;init_params(&p);
    initVqf(&p,&s,&c,.0025f,.01f,-1);
    float g[3]={.001f,-.002f,.003f},a[3]={0,0,9.80665f};
    for(int i=0;i<24000;i++){updateGyr(&p,&s,&c,g);if(i%4==3)updateAcc(&p,&s,&c,a);}
    earth_pair_t live={0},restored={0};float earth[3]={.00005f,0,.00004f};
    assert(earth_pair_start(&live,&p,&s,&c,earth));
    unsigned char storage[EARTH_PAIR_SNAPSHOT_CAPACITY];
    assert(earth_pair_save(&live,false,true,24000,2,storage,sizeof(storage)));
    bool pending,gate;uint32_t samples;uint8_t mode;
    assert(earth_pair_load(&restored,&pending,&gate,&samples,&mode,storage,sizeof(storage)));
    assert(!pending&&gate&&samples==24000&&mode==2);
    for(int i=0;i<1000;i++){
        earth_pair_gyro(&live,g);earth_pair_gyro(&restored,g);
        if(i%4==3){earth_pair_accel(&live,a);earth_pair_accel(&restored,a);}
    }
    assert(!memcmp(&live.state,&restored.state,sizeof(live.state)));
    earth_pair_t before=restored;
    storage[30]^=1;
    assert(!earth_pair_load(&restored,&pending,&gate,&samples,&mode,storage,sizeof(storage)));
    assert(!memcmp(&before,&restored,sizeof(before)));storage[30]^=1;
    earth_pair_snapshot_t record;memcpy(&record,storage,sizeof(record));
    record.version=VQF_STATE_VERSION==4?3:4;record.checksum=snapshot_checksum(&record);
    memcpy(storage,&record,sizeof(record));
    assert(!earth_pair_load(&restored,&pending,&gate,&samples,&mode,storage,sizeof(storage)));
    assert(!memcmp(&before,&restored,sizeof(before)));
    return 0;
}
'''.replace('RETAINED_ID',wrapper[start:end])
        parent=ROOT/'.local/native-tests';parent.mkdir(exist_ok=True)
        for mode in (0,1):
            with tempfile.TemporaryDirectory(dir=parent) as d:
                compile_run('#define VQF_STABLE_IIR '+str(mode)+'\n'+code,d,[CORE,FUSION])

    def test_production_and_stable_mode_are_identical(self):
        prod=Replay(ROOT/'.local/native-tests/production-stable')
        stable=Replay(ROOT/'.local/native-tests/production-stable',iir='stable')
        t=np.arange(0,30,.0025);gyro=np.zeros((len(t),3));gyro[:,2]=.1*np.sin(t)
        acc=np.full_like(gyro,np.nan);acc[::4]=[0,0,1]
        np.testing.assert_array_equal(prod.run(gyro,acc,t*0+.0025,raw_accel_stride=4),
                                      stable.run(gyro,acc,t*0+.0025,raw_accel_stride=4))

    def test_cmsis_scalar_path_tracks_libm_for_rest_turn_and_recovery(self):
        engine=Replay(ROOT/'.local/native-tests/math-backends')
        cmsis=Replay(ROOT/'.local/native-tests/math-backends',math_backend='cmsis')
        t=np.arange(0,120,.0025);g=np.zeros((len(t),3));g[:]=[.13,-.05,.09]
        g[(t>=30)&(t<40),2]+=90
        a=np.full_like(g,np.nan);a[::4]=[0,0,1]
        results=[e.run(g,a,t*0+.0025,[.003,0,.0029],raw_accel_stride=4) for e in (engine,cmsis)]
        error=np.rad2deg((orientations(results[0])*orientations(results[1]).inv()).magnitude())
        self.assertLess(float(error.max()),.01)
        np.testing.assert_allclose(results[0][-1,4:7],results[1][-1,4:7],atol=2e-5)


class CrashTests(unittest.TestCase):
    def setUp(self):
        self.fixture=install_fixture.InstallTests('test_all_three_fields_and_exact_readback')
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)

    def test_full_group_rollback_preflight_rejects_foreign_state(self):
        entries=self.fixture.entries;radio=Fake(entries)
        radio.blobs[1]=entries[1]['after'];radio.blobs[0]=b'!'+radio.blobs[0][1:]
        with self.assertRaises(ValueError):c.rollback_entries(radio,entries,Journal())
        self.assertEqual(radio.writes,[])

    def test_kernel_lease_releases_after_hard_exit_without_stealing_live_owner(self):
        from hardware_lock import ProcessLock
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'LOCK'
            with ProcessLock(path):
                with self.assertRaises(FileExistsError):ProcessLock(path)
            self.assertFalse(path.exists())
            worker=Path(d)/'crash.py'
            worker.write_text('import sys,os\nfrom pathlib import Path\nsys.path.insert(0,sys.argv[1])\nfrom hardware_lock import ProcessLock\nlock=ProcessLock(Path(sys.argv[2]))\nos._exit(73)\n')
            result=subprocess.run([sys.executable,str(worker),str(ROOT/'tools'),str(path)],capture_output=True)
            self.assertEqual(result.returncode,73,result.stderr)
            self.assertTrue(path.exists())
            with ProcessLock(path):self.assertEqual(path.read_text(),str(os.getpid()))
            self.assertFalse(path.exists())
            path.write_text('unknown owner')
            with self.assertRaises(FileExistsError):ProcessLock(path)
            self.assertEqual(path.read_text(),'unknown owner')

    def test_hard_exit_after_each_commit_recovers_in_fresh_process(self):
        entries=self.fixture.entries
        # Serialize the reviewed states; simulated NVS lives in separate files.
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d);spec=folder/'entries.json'
            spec.write_text(json.dumps([{**e,'before':e['before'].hex(),'after':e['after'].hex(),
                                        'states':[v.hex() for v in e['states']]} for e in entries]))
            worker=folder/'worker.py'
            worker.write_text('''import json,os,sys
from pathlib import Path
sys.path.insert(0,sys.argv[1])
import calwire_apply as c
from test_calwire_apply import Fake
from batch_calibration import Journal
folder=Path(sys.argv[2]);mode=sys.argv[3]
entries=json.loads((folder/'entries.json').read_text())
for e in entries:
 e['before']=bytes.fromhex(e['before']);e['after']=bytes.fromhex(e['after']);e['states']=[bytes.fromhex(v) for v in e['states']]
class Disk(Fake):
 def __init__(self):
  super().__init__(entries)
  for slot in self.blobs:self.blobs[slot]=(folder/(str(slot)+'.bin')).read_bytes()
 def stage_field(self,*args,**kw):
  super().stage_field(*args,**kw)
  slot=args[0]
  with (folder/(str(slot)+'.bin')).open('wb') as f:f.write(self.blobs[slot]);f.flush();os.fsync(f.fileno())
  if mode!='recover' and len(self.writes)==int(mode):os._exit(73)
radio=Disk();journal=Journal(folder/'journal.jsonl')
if mode=='recover':c.rollback_entries(radio,entries,journal)
else:c.apply_entries(radio,entries,journal)
''',encoding='utf8')
            for boundary in range(1,7):
                for e in entries:(folder/(str(e['slot'])+'.bin')).write_bytes(e['before'])
                cmd=[sys.executable,str(worker),str(ROOT/'tools'),str(folder)]
                result=subprocess.run(cmd+[str(boundary)],capture_output=True)
                self.assertEqual(result.returncode,73,result.stderr)
                recovery=subprocess.run(cmd+['recover'],capture_output=True)
                self.assertEqual(recovery.returncode,0,recovery.stderr)
                for e in entries:self.assertEqual((folder/(str(e['slot'])+'.bin')).read_bytes(),e['before'])


if __name__=='__main__':unittest.main()

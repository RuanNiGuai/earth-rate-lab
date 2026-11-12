"""Behavioral checks of causal native replay and lossless window integration."""
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from fusion_replay import CORE,FUSION,POLICIES,Replay,extract_function,orientations,resample_windows
from native_test import compile_run
from fusion_stable_iir import REPLACEMENTS,transform
from scipy.signal import butter,lfilter,lfilter_zi


class FusionReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.engine=Replay(ROOT/'.local/native-tests/fusion-replay')

    def samples(self,seconds=60):
        n=seconds*400
        gyro=np.zeros((n,3));acc=np.full_like(gyro,np.nan);acc[::4]=[0,0,1]
        return gyro,acc,np.full(n,1/400)

    def test_window_resampling_preserves_integral(self):
        duration=np.array([.03,.01,.04]);g=np.array([[1,2,3],[-2,5,1],[4,3,2.]])
        t,go,ao,dt=resample_windows(g,g,duration,11,.02)
        np.testing.assert_allclose((go*dt[:,None]).sum(0),(g*duration[:,None]).sum(0),atol=1e-14)
        np.testing.assert_allclose(t,[11.01,11.03,11.05,11.07])
        np.testing.assert_equal(go,ao)

    def test_raw_absent_acceleration_is_not_a_new_sample(self):
        g,a,dt=self.samples();o=self.engine.run(g,a,dt,raw_accel_stride=4)
        self.assertGreater(o[-1,7],.9)
        self.assertLess(np.max(np.rad2deg(orientations(o).magnitude())),1e-4)
        with self.assertRaises(ValueError):self.engine.run(g,a,dt)

    def test_future_measurements_cannot_change_past(self):
        g,a,dt=self.samples();a_run=self.engine.run(g,a,dt,raw_accel_stride=4,rest_guard_dps=.03,rest_anchor_tau=60)
        g[16000:,2]=20
        b_run=self.engine.run(g,a,dt,raw_accel_stride=4,rest_guard_dps=.03,rest_anchor_tau=60)
        np.testing.assert_array_equal(a_run[:16000],b_run[:16000])

    def test_constant_bias_with_earth_does_not_double_subtract(self):
        g,a,dt=self.samples(120);earth=np.array([.003,0,.0029]);g[:]=[.13,-.05,.09]+earth
        o=self.engine.run(g,a,dt,earth,raw_accel_stride=4)
        self.assertEqual(o[-1,9],1)
        # The current single-precision DF2T low-pass has a measured DC error;
        # this bound still detects a second subtraction of the Earth vector.
        np.testing.assert_allclose(o[-1,4:7],[.13,-.05,.09],atol=.001)
        q=orientations(o)
        self.assertLess(np.rad2deg((q[-1]*q[24000].inv()).magnitude()),.04)

    def test_known_fast_yaw_does_not_get_learned_as_rest(self):
        g,a,dt=self.samples(40);g[8000:12000,2]=90
        o=self.engine.run(g,a,dt,raw_accel_stride=4,rest_sigma=.005,rest_guard_dps=.03,rest_anchor_tau=60,accel_gate=3)
        self.assertLess(np.mean(o[8500:11500,7]),.01)
        truth=Rotation.from_rotvec([0,0,np.deg2rad(900)])
        self.assertLess(np.rad2deg((orientations(o)[-1]*orientations(o)[7900].inv()*truth.inv()).magnitude()),.05)

    def test_stable_iir_recovers_small_bias_at_high_sample_rate(self):
        engine=Replay(ROOT/'.local/native-tests/fusion-replay',iir='stable')
        g,a,dt=self.samples(120);earth=np.array([.003,0,.0029]);g[:]=[.13,-.05,.09]+earth
        o=engine.run(g,a,dt,earth,raw_accel_stride=4)
        np.testing.assert_allclose(o[-1,4:7],[.13,-.05,.09],atol=.00002)
        q=orientations(o)
        self.assertLess(np.rad2deg((q[-1]*q[24000].inv()).magnitude()),.001)

    def test_adaptive_learning_is_delayed_and_preserves_fast_motion_after_rest(self):
        engine=Replay(ROOT/'.local/native-tests/fusion-replay',iir='stable')
        g,a,dt=self.samples(100);g[:]=[.13,-.05,.09]
        g[28000:30000,2]+=90
        baseline=engine.run(g,a,dt,raw_accel_stride=4)
        adaptive=engine.run(g,a,dt,raw_accel_stride=4,**POLICIES['adaptive_rest'])
        np.testing.assert_allclose(adaptive[:10000],baseline[:10000],atol=1e-8)
        q=orientations(adaptive)
        truth=Rotation.from_rotvec([0,0,np.deg2rad(450)])
        error=np.rad2deg((q[-1]*q[27999].inv()*truth.inv()).magnitude())
        original=orientations(baseline)
        baseline_error=np.rad2deg((original[-1]*original[27999].inv()*truth.inv()).magnitude())
        # Stay within one 90 deg/s sample, and do not degrade the same causal
        # initialization/turn sequence with the original learning schedule.
        self.assertLess(error,90*dt[0])
        self.assertLessEqual(error,baseline_error)
        self.assertLess(np.mean(adaptive[28500:29500,7]),.01)

    def test_linear_acceleration_rejection_and_recovery(self):
        g,a,dt=self.samples(60);a[8000:12000:4,0]=.5
        o=self.engine.run(g,a,dt,raw_accel_stride=4,accel_gate=3)
        self.assertLess(np.rad2deg(orientations(o).magnitude()).max(),.001)
        self.assertEqual(o[-1,7],1)
        self.assertEqual(o[-1,8],1)

    def test_missing_time_cannot_be_silently_compressed(self):
        g,a,dt=self.samples();dt[1000]=.1
        with self.assertRaises(ValueError):self.engine.run(g,a,dt,raw_accel_stride=4)

    def test_missing_acceleration_cannot_change_effective_update_rate(self):
        for index in (0,400,-4):
            g,a,dt=self.samples();a[index]=np.nan
            with self.assertRaisesRegex(ValueError,'irregular'):
                self.engine.run(g,a,dt,raw_accel_stride=4)

    def test_conservative_policy_retains_slow_motion_after_static_initialization(self):
        engine=Replay(ROOT/'.local/native-tests/fusion-replay',iir='stable')
        for rate in (.05,.2):
            g,a,dt=self.samples(120);g[12000:,2]=rate
            o=engine.run(g,a,dt,raw_accel_stride=4,**POLICIES['conservative'])
            q=orientations(o)
            truth=Rotation.from_rotvec([0,0,np.deg2rad(rate*90)])
            self.assertLess(np.rad2deg((q[-1]*q[12000].inv()*truth.inv()).magnitude()),.12)

    def test_stable_earth_realign_keeps_bias_and_lowpass_translation_consistent(self):
        core,pair=transform((CORE/'vqf.c').read_text(encoding='utf8'),
                            (FUSION/'earth_pair.c').read_text(encoding='utf8'),extract_function)
        code=r'''
#include <assert.h>
#include "stable_core.c"
#include "stable_pair.c"
int main(void) {
    vqf_params_t p; vqf_state_t s; vqf_coeffs_t c; init_params(&p);
    initVqf(&p,&s,&c,.0025f,.01f,-1);
    float g[3]={.001f,-.002f,.003f},a[3]={0,0,9.80665f};
    for(int i=0;i<24000;i++) {updateGyr(&p,&s,&c,g);if(i%4==3)updateAcc(&p,&s,&c,a);}
    earth_pair_t pair={0}; float e[3]={.00005f,0,.00004f};
    assert(earth_pair_start(&pair,&p,&s,&c,e));
    earth_pair_t before=pair; float q0[4],q1[4],e0[3],e1[3];
    getQuat6D(&pair.state,q0);earth_pair_project(q0,e,e0);
    assert(earth_pair_realign_yaw(&pair,.2f));
    getQuat6D(&pair.state,q1);earth_pair_project(q1,e,e1);
    for(int j=0;j<3;j++) {
        float delta=e1[j]-e0[j];
        assert(fabsf(pair.state.bias[j]+delta-before.state.bias[j])<1e-8f);
        for(int k=0;k<20;k++) {
            float y0=filterStep(g[j]-e0[j],c.restGyrLpB,c.restGyrLpA,&before.state.restGyrLpState[2*j]);
            float y1=filterStep(g[j]-e1[j],c.restGyrLpB,c.restGyrLpA,&pair.state.restGyrLpState[2*j]);
            assert(fabsf(y1+delta-y0)<1e-8f);
        }
    }
    return 0;
}
'''
        with tempfile.TemporaryDirectory(dir=ROOT/'.local/native-tests') as d:
            Path(d,'stable_core.c').write_text(core,encoding='utf8')
            Path(d,'stable_pair.c').write_text(pair,encoding='utf8')
            compile_run(code,d,[CORE,FUSION])

    def test_stable_realization_matches_double_precision_butterworth(self):
        # Independent scipy transfer function, with nonzero steady state and
        # changing signals. C uses the actual experimental float realization.
        n=8000;t=np.arange(n)/400
        x=.13+.04*np.sin(t*1.7)+.02*(t>8)
        b,a=butter(2,np.sqrt(2)/(2*np.pi*.66),fs=400)
        expected,_=lfilter(b,a,x,zi=lfilter_zi(b,a)*x[0])
        fmt=lambda v:','.join(f'{float(z):.10g}' for z in v)
        code='#include <math.h>\n#include <stddef.h>\n#include <stdbool.h>\n#include <assert.h>\ntypedef float vqf_real_t;typedef float vqf_double_t;\n#define M_SQRT2 1.4142135623730951f\n'
        code+='\n'.join(REPLACEMENTS.values())
        code+='\nstatic const double x[]={'+fmt(x)+'};\nstatic const double truth[]={'+fmt(expected)+'};\n'
        code+='int main(void){float b[3],a[2],s[2];filterCoeffs(.66f,1.0f/400,b,a);filterInitialState(x[0],b,a,s);'
        code+='float previous[1]={0};filterAdaptStateForCoeffChange(previous,1,b,a,b,a,s);'
        code+=f'for(int i=0;i<{n};i++){{float y=filterStep(x[i],b,a,s);assert(fabs(y-truth[i])<.00003);}}return 0;}}'
        parent=ROOT/'.local/native-tests'
        with tempfile.TemporaryDirectory(dir=parent) as d:compile_run(code,d)


if __name__=='__main__':unittest.main()

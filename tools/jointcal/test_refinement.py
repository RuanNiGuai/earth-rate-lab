import copy
import unittest
from unittest.mock import patch
import numpy as np
from scipy.sparse import eye,diags
from scipy.spatial.transform import Rotation as R
import joint_rigid_v3 as j
import refinement as f
from make_plan import require_convergence,information_failures,require_final_return


def state(scale=1.):
    return dict(scale=np.full((10,3),scale),gsens=np.zeros((10,3,3)),mount=np.tile(np.eye(3),(10,1,1)),
                clocks=np.zeros((40,2)),phase=0.,positions=np.zeros((10,3)))


def earth_fixture(earth=True):
    rng=np.random.default_rng(61);j.old.LATITUDE_DEG=45
    poses=[R.from_rotvec([a,0,0]) for a in np.deg2rad([0,90,180,270,360])]
    poses += [R.from_euler('z',180,degrees=True),R.from_euler('z',-90,degrees=True),R.from_euler('z',90,degrees=True),
              R.from_euler('y',90,degrees=True),R.from_euler('y',-90,degrees=True)]
    up=np.array([1.,0,0]);E=.0041780746*np.array([np.sin(np.pi/4),np.cos(np.pi/4)*np.cos(.7),np.cos(np.pi/4)*np.sin(.7)])
    data=[];st=[];truth=[];t=np.arange(10)*40.
    for _ in range(10):
        M=rng.normal(0,.002,(3,3));acc=np.array([p.inv().apply(up) for p in poses])
        y=acc@M.T+rng.normal(0,.01,3)+t[:,None]*rng.normal(0,1e-5,3)
        if earth:y+=np.array([p.inv().apply(E) for p in poses])
        data.append(dict(S=np.ones(3),Q=R.identity()));st.append((y,acc,t,None));truth.append(M)
    return data,st,poses,up,truth


class RefinementTests(unittest.TestCase):
    def test_return_rest_extends_fit_and_wrong_heading_fails_boundary(self):
        data,st,poses,up,truth=earth_fixture()
        for d,s in zip(data,st):
            y,a,t,_=s
            # Extend the same physical linear drift to the end of the run.
            D=np.c_[a,np.ones(10),(t-t[0])/100]
            E=.0041780746*np.array([np.sin(np.pi/4),np.cos(np.pi/4)*np.cos(.7),np.cos(np.pi/4)*np.sin(.7)])
            C=np.linalg.lstsq(D,y-np.array([p.inv().apply(E) for p in poses]),rcond=None)[0]
            final=y[0]+C[-1]*4
            d['extended']=(np.vstack([y,final]),np.vstack([a,a[0]]),np.r_[t,400.],None)
        st=[d.pop('extended') for d in data];poses=poses+[R.identity()]
        fit,matrices=j.earth_gsens(data,st,st,poses,up)
        np.testing.assert_allclose(matrices,truth,atol=1e-7)
        check=j.final_return_check(st,st,poses,fit)
        self.assertTrue(check['passed']);self.assertEqual(len(check['devices']),10)
        self.assertLess(max(r['return_holdout_rms_dps'] for r in check['devices']),1e-8)
        wrong=poses[:-1]+[R.from_rotvec(up*np.deg2rad(12))]
        rejected=j.final_return_check(st,st,wrong,fit)
        self.assertFalse(rejected['passed'])
        self.assertAlmostEqual(rejected['orientation_closure_deg'],12)
        self.assertTrue(all(r['gravity_return_deg']<1e-6 for r in rejected['devices']))
        require_final_return(dict(final_return=check))
        with self.assertRaisesRegex(ValueError,'heading'):require_final_return(dict(final_return=rejected))
        duplicate=copy.deepcopy(check);duplicate['devices'][-1]['slot']=0
        with self.assertRaisesRegex(ValueError,'diagnostic'):require_final_return(dict(final_return=duplicate))

    def test_clock_derivative_and_small_ppm_fit_on_long_turn(self):
        from scipy.optimize import least_squares
        knots=np.arange(200,270.01,.1)
        rate=np.c_[100+30*np.sin(knots*3),10*np.sin(knots),2*np.cos(knots)]
        a=np.arange(201.003,269,.02);b=a+.02
        target=np.array([.01,.05]);factor=1+target[1]*1e-6
        aa=a+target[0]*.001+(a-300)*target[1]*1e-6
        bb=b+target[0]*.001+(b-300)*target[1]*1e-6
        y=j.mean_curve(knots,rate,aa,bb)*factor
        pairs=[(dict(knots=knots,rate=rate),a,b,y)]
        # A macroscopic centered step is an independent derivative check;
        # default relative finite differences lose precision at sub-ppm shifts.
        x=np.array([.03,.1]);steps=[.001,.01];columns=[]
        for axis,step in enumerate(steps):
            delta=np.eye(2)[axis]*step
            columns.append((j.clock_residual(x+delta,pairs)-j.clock_residual(x-delta,pairs))/(2*step))
        np.testing.assert_allclose(j.clock_residual(x,pairs,True),np.column_stack(columns),atol=2e-6,rtol=3e-4)
        fit=least_squares(lambda x:j.clock_residual(x,pairs)[:-1],[0.,0.],
                          jac=lambda x:j.clock_residual(x,pairs,True)[:-1],max_nfev=25)
        self.assertTrue(fit.success)
        np.testing.assert_allclose(fit.x,target,atol=.0002)

    def test_inner_curve_iteration_limit_is_not_success(self):
        _,check=f.solve_curve(eye(4),np.zeros((4,3)))
        self.assertTrue(check['converged'])
        with patch.object(f,'lsmr',return_value=(np.zeros(4),7,800,0,0,0,1,0)):
            _,check=f.solve_curve(eye(4),np.zeros((4,3)))
        self.assertFalse(check['converged'])

    def test_window_grid_resolves_fast_rate_changes_during_long_turns(self):
        end=np.arange(.02,3,.02);begin=end-.02;frequency=2*np.pi*8
        averaged=120+16*(np.cos(frequency*begin)-np.cos(frequency*end))/(frequency*.02)
        gy=np.c_[averaged,np.zeros((len(end),2))];acc=np.tile([1.,0,0],(len(end),1))
        data=[dict(time=(begin+end)/2,interval_end=end,interval_duration=end-begin,
                   gyro=gy,accel=acc,temp=np.full(len(end),25.),S=np.ones(3),Q=R.identity(),M=np.zeros((3,3)),
                   clock_rate=1.,lut=np.array([[0,15,0,0,0],[1,43,0,0,0]])) for _ in range(10)]
        rests=[(np.zeros((2,3)),np.array([[1.,0,0]]*2),np.array([0.,3.]),None)]*10
        _,coarse,_,_=j.motion(data,rests,np.zeros((10,2)),ranges=[(.3,2.7)],step=.1)
        _,fine,_,_=j.motion(data,rests,np.zeros((10,2)),ranges=[(.3,2.7)],step=j.DT)
        self.assertGreater(coarse[0]['fit_rms_dps'],4.)
        self.assertLess(fine[0]['fit_rms_dps'],.1)
        self.assertTrue(fine[0]['curve_solve']['converged'])

    def test_interval_operator_handles_multicell_windows_exactly(self):
        knots=np.arange(0,2.01,.1);a=np.array([.003,.099,.105,.3]);b=np.array([.089,.151,.499,1.99])
        A=j.old.interval_operator(a,b,knots)
        np.testing.assert_allclose(A@knots,(a+b)/2,atol=1e-13)
        np.testing.assert_allclose(np.asarray(A.sum(axis=1)).ravel(),1,atol=1e-13)

    def test_continuous_constraints_do_not_lock_scale_to_previous_latent_curve(self):
        rng=np.random.default_rng(24);truth=rng.uniform(.995,1.005,(10,3));data=[];rows=[]
        for i in range(10):
            records=[]
            for axis in range(3):
                r=np.zeros(26);r[6]=25;r[7+axis]=7200/truth[i,axis];r[14]=60
                records.append(r)
            data.append(dict(S=np.ones(3),M=np.zeros((3,3)),Q=R.identity(),Qseed=R.identity(),records=records,
                             lut=np.array([[0,15,0,0,0],[1,43,0,0,0]])))
            omega=rng.normal(0,100,(1000,3));raw=omega/truth[i]
            rows.append(f.compress_rows(np.c_[raw,np.zeros_like(raw)],omega*1.01,np.ones(1000)*100))
        fit=j.joint_scales(data,rows)
        np.testing.assert_allclose(fit['scales'],truth,atol=2e-6)
        self.assertTrue(fit['window_constraints'])

    def test_qr_preserves_all_window_equations(self):
        rng=np.random.default_rng(21);a=rng.normal(size=(1000,6));y=rng.normal(size=(1000,3));w=rng.uniform(.2,2,1000)
        rows=f.compress_rows(a,y,w)
        for _ in range(4):
            coefficients=rng.normal(size=(6,3))
            expected=np.sum(w[:,None]*(a@coefficients-y)**2)
            got=np.sum((rows[:,:6]@coefficients-rows[:,6:])**2)
            self.assertAlmostEqual(expected,got,places=8)

    def test_unmodeled_window_motion_cannot_shrink_absolute_scale(self):
        rng=np.random.default_rng(442);truth=rng.uniform(.995,1.005,(10,3));data=[];rows=[]
        # The common curve cannot represent some high-frequency motion. Its
        # irreducible residual must not reward shrinking the sensor gain.
        omega=rng.normal(0,100,(2000,3));missing=rng.normal(0,8,omega.shape)
        missing-=omega@np.linalg.lstsq(omega,missing,rcond=None)[0]
        for i in range(10):
            records=[]
            for axis in range(3):
                r=np.zeros(26);r[6]=25;r[7+axis]=7200/truth[i,axis];r[14]=60;records.append(r)
            data.append(dict(S=np.ones(3),M=np.zeros((3,3)),Q=R.identity(),Qseed=R.identity(),records=records,
                             lut=np.array([[0,15,0,0,0],[1,43,0,0,0]])))
            rows.append(f.compress_rows(np.c_[omega/truth[i],np.zeros_like(omega)],omega+missing,np.ones(len(omega))*100))
        fit=j.joint_scales(data,rows)
        self.assertTrue(fit['success'])
        self.assertLess(fit['joint_angle_rms_deg'],.01)
        np.testing.assert_allclose(fit['scales'],truth,atol=2e-6)

    def test_stopping_requires_three_stable_changes_and_finite_parameters(self):
        c=f.Convergence()
        for _ in range(3):self.assertFalse(c.update(state(),1.))
        self.assertTrue(c.update(state(),1.))
        with self.assertRaisesRegex(ValueError,'convergence'):
            require_convergence(dict(solver_version='adaptive-window-rigid-1',numerics_version='analytic-clock-irls100-1',convergence=c.report()))
        require_convergence(dict(solver_version='adaptive-window-rigid-1',numerics_version='sensor-residual-window20ms-2',convergence=c.report()))
        broken_history=copy.deepcopy(c.report());broken_history['history'][-1]['objective']=float('nan')
        with self.assertRaisesRegex(ValueError,'objectives'):
            require_convergence(dict(solver_version='adaptive-window-rigid-1',numerics_version='sensor-residual-window20ms-2',convergence=broken_history))
        c2=f.Convergence(maximum=5)
        for i in range(5):self.assertFalse(c2.update(state(1+.001*(i%2)),1.))
        with self.assertRaisesRegex(ValueError,'convergence'):
            require_convergence(dict(solver_version='adaptive-window-rigid-1',numerics_version='sensor-residual-window20ms-2',convergence=c2.report()))
        broken=state();broken['gsens'][0,0,0]=np.nan
        with self.assertRaisesRegex(ValueError,'Non-finite'):c.update(broken,1.)

    def test_stable_outer_parameters_do_not_hide_failed_inner_solvers(self):
        c=f.Convergence(maximum=5)
        for _ in range(5):self.assertFalse(c.update(state(),1.,inner_ok=False))
        tampered=c.report();tampered['converged']=True
        with self.assertRaisesRegex(ValueError,'changes'):require_convergence(dict(solver_version='adaptive-window-rigid-1',numerics_version='sensor-residual-window20ms-2',convergence=tampered))

    def test_missing_earth_signal_cannot_pass_just_because_amplitude_is_constrained(self):
        data,st,poses,up,_=earth_fixture(False)
        fit,_=j.earth_gsens(data,st,st,poses,up)
        self.assertLess(fit['horizontal_free_magnitude_ratio'],.01)
        self.assertFalse(fit['earth_information']['identifiable'])
        self.assertTrue(information_failures(dict(static=fit)))

    def test_earth_identifiability_and_robust_single_pose_outlier(self):
        data,st,poses,up,truth=earth_fixture()
        held=copy.deepcopy(st)
        st[4][0][2]+=np.array([.01,-.008,.009])
        fit,matrices=j.earth_gsens(data,st,held,poses,up)
        self.assertTrue(fit['earth_information']['identifiable'])
        self.assertLess(fit['devices'][4]['static_robust_weights'][2],.2)
        self.assertLess(abs(fit['phase_rad']-.7),.03)
        self.assertLess(np.max(abs(np.array(matrices)-truth)),.001)
        self.assertTrue(fit['robust_converged'])

    def test_multisite_window_terms_recover_gsens_with_different_mounts(self):
        rng=np.random.default_rng(22);n=101;t=np.arange(n)*.02;positions=rng.uniform(-.1,.1,(10,3));positions[0]=0
        data=[dict(Q=R.from_rotvec(rng.normal(0,.01,3)),S=rng.uniform(.995,1.005,3),M=np.zeros((3,3))) for _ in range(10)]
        truth=rng.normal(0,.001,(10,3,3));accs=[[] for _ in data];gyros=[[] for _ in data];segments=[]
        for m in range(9):
            freq=np.array([1.2,1.9,2.3]);w=.7*np.sin(t[:,None]*freq+m*.7);alpha=.7*freq*np.cos(t[:,None]*freq+m*.7)
            common=np.c_[.2*np.sin(t+m),.2*np.cos(t*1.3+m),1+.1*np.sin(t*.7+m)]
            for i,d in enumerate(data):
                acc=common+(np.cross(alpha,positions[i])+np.cross(w,np.cross(w,positions[i])))/9.80665
                accs[i].append(acc);gyros[i].append(d['Q'].inv().apply(np.rad2deg(w))/d['S']+d['Q'].inv().apply(acc)@truth[i].T)
            D=diags([np.ones(n-2),-2*np.ones(n-2),np.ones(n-2)],[0,1,2],shape=(n-2,n))
            segments.append(dict(index=m,knots=t,rate=np.rad2deg(w),smooth=D,operators=[eye(n).tocsr()]*10,
                                 indexes=[np.arange(m*n,(m+1)*n)]*10))
        obs=[dict(start=np.arange(n*9)*.02,end=np.arange(n*9)*.02+.02,accel=np.concatenate(accs[i]),
                  gyro_delta=np.concatenate(gyros[i]),accel_base=np.zeros((n*9,3)),clock_factor=np.ones(n*9)) for i in range(10)]
        av=np.zeros((10,3));av[j.IDS]=np.vstack([np.eye(3),-np.eye(3)]);st=[(None,av,None,None)]*10
        rows,diagnostic=f.window_statistics(j,data,st,obs,segments,dict(relative_positions_m=positions,accelerometer_residual_bias_g=np.zeros((10,3))))
        matrices=[]
        for d,r in zip(data,rows):
            y=r[:,:3]-d['Q'].inv().apply(r[:,6:])/d['S']
            matrices.append(np.linalg.lstsq(r[:,3:6],y,rcond=None)[0].T)
        np.testing.assert_allclose(matrices,truth,atol=8e-5)
        self.assertEqual(len(diagnostic['blocks']),70)


if __name__=='__main__':unittest.main()

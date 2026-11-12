import unittest
import numpy as np
from scipy.sparse import eye,diags
from scipy.spatial.transform import Rotation as R
import joint_rigid_v3 as j

class PhysicsTests(unittest.TestCase):
    def setUp(self):
        j.old.MOV=[(123,135),(162,173),(203,214),(233,244),(273,286),(314,325),(353,367),(392,404),(433,449)]
        j.old.LATITUDE_DEG=45.0

    def test_relative_clock_offsets_recovery(self):
        rng=np.random.default_rng(923);truth=np.zeros((10,2));truth[1:,0]=rng.uniform(-.005,.005,9);truth[1:,1]=rng.uniform(-8,8,9)
        segments=[];ends=[];observed=[[] for _ in range(10)]
        for m,(a,b) in enumerate(j.old.MOV):
            knots=np.arange(a,b+j.DT/2,j.DT)
            rate=np.c_[20*np.sin(knots*2),15*np.cos(knots*1.3),10*np.sin(knots*2.7)]
            finish=np.linspace(a+.07,b-.07,100);start=finish-.02;ends.append(finish)
            segments.append(dict(index=m,knots=knots,rate=rate,indexes=[np.arange(m*100,(m+1)*100)]*10))
            for i,(offset,ppm) in enumerate(truth):
                aa=start+offset+(start-300)*ppm*1e-6;bb=finish+offset+(finish-300)*ppm*1e-6
                observed[i].append(j.mean_curve(knots,rate,aa,bb)*(1+ppm*1e-6))
        end=np.concatenate(ends);data=[];st=[]
        for i in range(10):
            data.append(dict(time=end-.01,interval_end=end,interval_duration=np.full(len(end),.02),
                clock_rate=1.,gyro=np.concatenate(observed[i]),accel=np.zeros((len(end),3)),M=np.zeros((3,3)),
                Q=R.identity(),S=np.ones(3),temp=np.full(len(end),25.),lut=np.array([[0,15,0,0,0],[1,43,0,0,0]])))
            st.append((np.zeros((10,3)),np.zeros((10,3)),np.linspace(90,480,10),None))
        got=j.refine_clocks(data,st,np.zeros((10,2)),segments)
        np.testing.assert_allclose(got[:,0],truth[:,0],atol=1e-4)
        np.testing.assert_allclose(got[:,1],truth[:,1],atol=.8)

    def test_joint_earth_and_gsens_recovery(self):
        rng=np.random.default_rng(850)
        poses=[R.from_rotvec([a,0,0]) for a in np.deg2rad([0,90,180,270,360])]
        poses += [R.from_euler('z',180,degrees=True),R.from_euler('z',-90,degrees=True),
                  R.from_euler('z',90,degrees=True),R.from_euler('y',90,degrees=True),R.from_euler('y',-90,degrees=True)]
        up=np.array([1.,0,0]);lat=np.deg2rad(j.old.LATITUDE_DEG);phase=.7
        E=.0041780746*np.array([np.sin(lat),np.cos(lat)*np.cos(phase),np.cos(lat)*np.sin(phase)])
        data=[];st=[];held=[];truth=[];t=np.arange(10)*40.
        for i in range(10):
            q=R.from_rotvec(rng.normal(0,.01,3));S=rng.uniform(.995,1.01,3);M=rng.normal(0,.003,(3,3))
            av=np.array([q.inv().apply(r.inv().apply(up)) for r in poses])
            earth=np.array([q.inv().apply(r.inv().apply(E))/S for r in poses])
            bias=rng.normal(0,.01,3);drift=rng.normal(0,.001,3)
            y=earth+av@M.T+bias+t[:,None]/100*drift
            st.append((y,av,t,None));held.append((y+.5/100*drift,av,t+.5,None))
            data.append(dict(Q=q,S=S));truth.append(M)
        result,Ms=j.earth_gsens(data,st,held,poses,up)
        self.assertAlmostEqual(result['phase_rad'],phase,places=4)
        np.testing.assert_allclose(Ms,truth,atol=1e-7)
        self.assertLess(max(r['earth_holdout_rms_dps'] for r in result['devices']),1e-8)

    def test_cross_and_centipetal_sign(self):
        w=np.array([[.2,-.3,.5],[0,0,2.]])
        alpha=np.array([[.3,.4,-.1],[0,0,.5]])
        r=np.array([.2,.1,-.05])
        B=j.cross_matrix(alpha)+j.cross_matrix(w)@j.cross_matrix(w)
        expected=np.cross(alpha,r)+np.cross(w,np.cross(w,r))
        np.testing.assert_allclose(B@r,expected,atol=1e-14)

    def test_true_interval_integrals(self):
        knots=np.arange(0,1.001,.02);rate=np.c_[knots,2+knots*3,-knots*2]
        a=np.array([.003,.128,.505]);b=np.array([.019,.175,.534])
        expected=np.c_[(a+b)/2,2+3*(a+b)/2,-(a+b)]
        np.testing.assert_allclose(j.mean_curve(knots,rate,a,b),expected,atol=1e-13)

    def test_joint_scale_recovery(self):
        rng=np.random.default_rng(842);S=rng.uniform(.995,1.01,(10,3));data=[]
        K=np.array([[1,.01,-.02],[.02,1,-.005],[.01,.02,1.]])
        K*=7200/np.linalg.norm(K,axis=1)[:,None]
        for i in range(10):
            q=R.from_rotvec(rng.normal(0,.007,3));delta=q.inv().apply(K)/S[i];records=[]
            for a in range(3):
                v=np.zeros(26);v[6]=25;v[7:10]=delta[a];v[13]=2500;v[14]=100;v[17:19]=25
                records.append(v)
            data.append(dict(S=np.ones(3),M=np.zeros((3,3)),Qseed=q,records=records,
                             lut=np.array([[0,15,0,0,0],[1,43,0,0,0]])))
        r=j.joint_scales(data)
        np.testing.assert_allclose(r['scales'],S,atol=2e-6)
        self.assertLess(r['joint_angle_rms_deg'],1e-3)

    def test_multi_position_acceleration_recovery(self):
        rng=np.random.default_rng(817);pos=rng.uniform(-.2,.2,(10,3));pos[0]=0
        n=101;t=np.arange(n)*j.DT;segments=[];all_acc=[[] for _ in range(10)]
        for m in range(9):
            phase=m*.7;freq=np.array([1.2,1.9,2.3])
            omega=.7*np.sin(t[:,None]*freq+phase)
            alpha=.7*freq*np.cos(t[:,None]*freq+phase)
            common=np.c_[.15*np.sin(t),.1*np.cos(t*1.3),1+.05*np.sin(t*.7)]
            for i in range(10):
                lever=(np.cross(alpha,pos[i])+np.cross(omega,np.cross(omega,pos[i])))/9.80665
                all_acc[i].append(common+lever)
            D=diags([np.ones(n-2),-2*np.ones(n-2),np.ones(n-2)],[0,1,2],shape=(n-2,n))
            segments.append(dict(index=m,knots=t,rate=np.rad2deg(omega),smooth=D,
                                 operators=[eye(n).tocsr()]*10,indexes=[np.arange(m*n,(m+1)*n)]*10))
        obs=[dict(accel=np.concatenate(a)) for a in all_acc]
        data=[dict(Q=R.identity()) for _ in range(10)]
        av=np.zeros((10,3));av[j.IDS]=np.vstack([np.eye(3),-np.eye(3)])
        st=[(None,av,None,None)]*10
        result=j.fit_lever_arms(data,st,obs,segments)
        np.testing.assert_allclose(result['relative_positions_m'],pos,atol=.008)
        self.assertLess(result['train_rms_g'],.0003)
        self.assertLess(np.mean([r['rigid_rms_g'] for r in result['held_motion_tests']]),.001)

if __name__=='__main__':unittest.main()

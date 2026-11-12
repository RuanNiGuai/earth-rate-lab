import copy
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
import continuous as c
import joint_rigid_v3 as j
from continuous_example import trajectory


def axis_data(axis=0, asymmetry=0, wrong_reverse=False):
    rng=np.random.default_rng(374)
    t=np.arange(.02,216.001,.02)
    speed=np.zeros(len(t))
    speed[(t>32)&(t<=92)]=120
    speed[(t>124)&(t<=184)]=120 if wrong_reverse else -120
    rows=[]
    for i in range(10):
        ppm=(i-4)*7;offset=.012*i;scale=1+(i-4)*.0008
        rate=1+ppm*1e-6;local=(t-t[0])/rate
        gyro=np.zeros((len(t),3));gyro[:,axis]=speed/scale
        gyro[speed<0,axis]*=1+asymmetry
        gyro+=rng.normal(0,.001,gyro.shape)+np.array([.012,-.008,.005])
        accel=np.zeros_like(gyro);accel[:,axis]=1
        accel[:,(axis+1)%3]=-(np.deg2rad(speed)**2)*i*.015/9.80665
        accel+=rng.normal(0,.0002,accel.shape)
        rows.append(dict(local_end=local,duration=np.full(len(t),.02/rate),
                         host=t+offset+rng.exponential(.003,len(t)),gyro=gyro,accel=accel,
                         temp=np.full(len(t),25.),seq=np.arange(len(t))))
    return rows


def pose_data(headings, tail=None):
    poses=[Rotation.from_euler('x',x,degrees=True) for x in headings]
    poses += tail or [Rotation.from_euler(a,x,degrees=True) for a,x in
                     [('z',180),('z',-90),('z',90),('y',90),('y',-90)]]
    t=np.arange(.01,408,.02);q=np.zeros((len(t),4));w=np.zeros((len(t),3))
    for k,pose in enumerate(poses):
        mask=(t>=k*42)&(t<k*42+30);q[mask]=pose.as_quat()
        if k==9:continue
        mask=(t>=k*42+30)&(t<(k+1)*42);u=(t[mask]-(k*42+30))/12
        delta=(pose.inv()*poses[k+1]).as_rotvec()
        q[mask]=(pose*Rotation.from_rotvec((10*u**3-15*u**4+6*u**5)[:,None]*delta)).as_quat()
        w[mask]=((30*u**2-60*u**3+30*u**4)/12)[:,None]*delta
    accel=Rotation.from_quat(q).inv().apply(np.tile([1.,0,0],(len(t),1)))
    return [dict(time=t,gyro=np.rad2deg(w),accel=accel.copy(),duration=np.full(len(t),.02)) for _ in range(10)]


class ContinuousTests(unittest.TestCase):
    def test_optional_final_return_keeps_all_eleven_rests(self):
        t,q,w,_=trajectory('poses',final_return=True)
        accel=q.inv().apply(np.tile([1.,0,0],(len(t),1)))
        rows=[dict(time=t,gyro=np.rad2deg(w),accel=accel.copy(),duration=np.full(len(t),.02)) for _ in range(10)]
        result=c.static_segments(rows,'poses')
        self.assertTrue(result['final_return_present'])
        self.assertEqual(len(result['static_segments_s']),11)
        self.assertEqual(result['observed_faces'][-1],'+X')
        for row in rows:row['accel'][t>420]=[0.,1.,0.]
        with self.assertRaisesRegex(ValueError,'第11段'):c.static_segments(rows,'poses')

    def test_long_dwell_clock_alignment_does_not_require_five_percent_motion(self):
        t,q,w,_=trajectory('poses',pose_rest_seconds=240)
        gyro=np.rad2deg(w);accel=q.inv().apply(np.tile([1.,0,0],(len(t),1)))
        self.assertLess(np.percentile(np.linalg.norm(gyro,axis=1),95),1)
        rows=[dict(local_end=t-t[0],duration=np.full(len(t),.02),host=t+.001*i,
                   gyro=gyro,accel=accel) for i in range(10)]
        model=c.align_clocks(rows)
        self.assertEqual(len(model),10)
        analysis=c.static_segments(rows,'poses')
        self.assertEqual(len(analysis['static_segments_s']),10)

    def test_pose_sequence_checks_quarter_turns_and_return_to_start(self):
        t,q,w,_=trajectory('poses')
        accel=q.inv().apply(np.tile([1.,0,0],(len(t),1)))
        rows=[dict(time=t,gyro=np.rad2deg(w),accel=accel,duration=np.full(len(t),.02)) for _ in range(10)]
        result=c.static_segments(rows,'poses')
        np.testing.assert_allclose(result['approximate_yaw_turns_deg'],90,atol=.1)
        altered=copy.deepcopy(rows)
        for d in altered:d['gyro'][(t>72)&(t<84)]*=-1
        with self.assertRaisesRegex(ValueError,'回到起始航向'):c.static_segments(altered,'poses')

    def test_reversed_heading_loop_and_reordered_faces_keep_measured_chronology(self):
        tail=[Rotation.from_euler(a,x,degrees=True) for a,x in
              [('z',180),('z',-90),('z',90),('y',-90),('y',90)]]
        rows=pose_data([0,90,180,90,0],tail);before=copy.deepcopy(rows)
        result=c.static_segments(rows,'poses')
        self.assertEqual(result['observed_faces'],['+X']*5+['-X','+Y','-Y','-Z','+Z'])
        np.testing.assert_allclose(result['approximate_headings_deg'],np.tile([0,90,180,90,0],(10,1)),atol=.1)
        self.assertFalse(result['reordered_samples'])
        self.assertTrue(result['nonstandard_sequence'])
        self.assertTrue(all(x['projected_singular_values'][1]>.2 for x in result['heading_information']))
        self.assertTrue(np.all(np.diff(result['static_segments_s'],axis=0)>0))
        for a,b in zip(rows,before):
            for key in a:np.testing.assert_array_equal(a[key],b[key])

    def test_two_heading_loop_and_missing_face_are_rejected(self):
        with self.assertRaisesRegex(ValueError,'地球航向覆盖不足'):
            c.static_segments(pose_data([0,90,0,90,0]),'poses')
        # Distinct yaw orientations on the same final face keep ten rest blocks.
        tail=[Rotation.from_euler(a,x,degrees=True) for a,x in
              [('z',180),('z',-90),('z',90),('y',90)]]
        tail.append(Rotation.from_euler('x',90,degrees=True)*tail[-1])
        with self.assertRaisesRegex(ValueError,'六面覆盖不足'):
            c.static_segments(pose_data([0,90,180,270,360],tail),'poses')

    def test_single_device_wrong_face_is_not_hidden_by_fleet_average(self):
        rows=pose_data([0,90,180,270,360])
        rows[7]['accel']=Rotation.from_euler('z',90,degrees=True).apply(rows[7]['accel'])
        with self.assertRaisesRegex(ValueError,'方向不一致'):c.static_segments(rows,'poses')

    def test_all_ten_clock_alignment_and_three_rest_detection(self):
        data=axis_data();models=c.align_clocks(data);analysis=c.static_segments(data,'x')
        self.assertEqual(len(analysis['static_segments_s']),3)
        np.testing.assert_allclose([r['clock_rate_correction_ppm'] for r in models],[(i-4)*7 for i in range(10)],atol=40)
        ends=np.array([d['time'][2000] for d in data])
        self.assertLess(np.ptp(ends),.015)
        self.assertTrue(np.all(np.asarray(analysis['approximate_turns'])[:,0]>19.8))

    def test_wrong_axis_same_direction_and_extra_rest_are_rejected(self):
        data=axis_data(wrong_reverse=True)
        # Ground-truth times isolate segmentation/admission from alignment.
        for d in data:d['time']=np.arange(.02,216.001,.02)
        with self.assertRaisesRegex(ValueError,'方向相同'):c.static_segments(data,'x')
        with self.assertRaisesRegex(ValueError,'未竖直'):c.static_segments(data,'y')

    def test_continuous_joint_fit_recovers_scales_and_retains_signed_candidates(self):
        data=[]
        for i in range(10):
            captures=[]
            for axis in range(3):
                d=axis_data(axis)[i]
                d['time']=np.arange(.02,216.001,.02)
                # Durations are nominal gyro integration intervals in this fixture.
                d['duration'][:]=.02
                d['rests']=[(4,28),(97,119),(189,212)]
                captures.append(d)
            data.append(dict(S=np.ones(3),M=np.zeros((3,3)),Qseed=Rotation.identity(),
                             lut=np.array([[0,15,0,0,0],[1,43,0,0,0]]),scale_captures=captures))
        result=j.joint_scales(data)
        expected=np.repeat((1+(np.arange(10)-4)*.0008)[:,None],3,axis=1)
        np.testing.assert_allclose(result['scales'],expected,atol=2e-6)
        self.assertTrue(all(r['passed'] for r in result['direction_checks']))
        self.assertEqual(np.asarray(result['direction_checks'][0]['positive_scale_candidate']).shape,(3,))
        self.assertEqual(len(result['direction_checks'][0]['capture_diagnostics']),6)
        for d in data:
            for cap in d['scale_captures']:
                cap['gyro'][cap['gyro'][:,np.argmax(np.max(abs(cap['gyro']),axis=0))]<-1]*=1.006
        asymmetric=j.joint_scales(data)
        self.assertFalse(any(r['passed'] for r in asymmetric['direction_checks']))
        self.assertIn('negative_scale_candidate',asymmetric['direction_checks'][0])


if __name__=='__main__':unittest.main()

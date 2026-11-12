"""No hardware: admission identity, stream continuity and motion gates."""
from dataclasses import replace
import unittest
import tempfile
from pathlib import Path

import batch_continuous as c
from f04_summary import Summary


def sample(seq, rate=(.34,-.1,.03), accel=(1000,0,0), **changes):
    s=Summary(7,23,4,seq,(seq+1)*20000,tuple(round(x*20000) for x in rate),accel,2700,8,8,20000)
    return replace(s,**changes)


class AdmissionTests(unittest.TestCase):
    def test_only_transport_admission(self):
        seq=c.admission('F05 ARC tid=7 flag=0x4f seq=42 status=6 (transport only)',7,0x4f)
        self.assertEqual(seq,42)
        self.assertFalse(c.b.Transaction(7,seq,0x4f,True).succeeded)

    def test_malformed_rejected_busy_wrong_target_or_flag(self):
        good='F05 ARC tid=7 flag=0x4f seq=42 status=6 (transport only)'
        for bad in ('',good+good,good.replace('tid=7','tid=8'),good.replace('0x4f','0x4e'),
                    good.replace('status=6','status=1'),good.replace('seq=42','seq=0')):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                c.admission(bad,7,0x4f)


class FixtureTests(unittest.TestCase):
    def fixture(self):
        f=c.Fixture(7)
        for i in range(201):
            f.add(sample(i),i*.02)
        stable=f.stable(4.)
        self.assertEqual(stable['face'],'+X')
        f.reset_motion(stable)
        return f

    def test_stationary_does_not_satisfy_move(self):
        f=self.fixture()
        self.assertFalse(f.moving)
        self.assertIsNone(f.stable(5.))

    def test_real_90_turn_and_still(self):
        f=self.fixture()
        for i in range(201,401):
            f.add(sample(i,rate=(-22.16,-.1,.03)),i*.02)
        self.assertTrue(f.moving)
        self.assertIsNone(f.stable(8.))
        for i in range(401,602):
            f.add(sample(i),i*.02)
        self.assertIsNotNone(f.stable(12.02))
        f.check_turn(-90,'X')
        with self.assertRaises(ValueError):
            f.check_turn(90,'X')

    def test_gap_reboot_operation_identity_and_stale_are_fatal(self):
        for s,now in ((sample(203),4.06),(sample(201,boot=24),4.02),
                      (sample(201,operation=5),4.02),(sample(201,tracker=8),4.02),
                      (sample(201),5.02),(sample(201,end_us=123),4.02),
                      (sample(201,accel_fresh=0),4.02)):
            f=self.fixture()
            with self.subTest(s=s), self.assertRaises(ValueError):
                f.add(s,now)

    def test_duplicate_does_not_invent_movement(self):
        f=self.fixture()
        f.add(sample(200),4.01)
        self.assertEqual(f.delta,[0,0,0])
        with self.assertRaises(ValueError):
            f.add(sample(200,rate=(3,0,0)),4.02)

    def test_noise_noncardinal_or_acceleration_blocks_stability(self):
        for accel in ((707,707,0),(1100,0,0)):
            f=c.Fixture(7)
            for i in range(201):
                f.add(sample(i,accel=accel),i*.02)
            self.assertIsNone(f.stable(4.))


class ControllerTests(unittest.TestCase):
    def make(self, path):
        obj=object.__new__(c.Controller)
        obj.targets={7:'010203040506',8:'010203040507'}
        obj.path=path/'session.json'
        obj.output=path
        obj.state={'next_sequence':1,'inflight':None,'devices':{
            str(k):{'on':True,'reference':True,'active':'earth-start','earth':0} for k in obj.targets}}
        obj.active_txs={}
        obj.capture_keys={}
        obj.cancelling_keys=set()
        obj.events=[]
        obj.armed=True
        obj.fatal=None
        return obj

    def status(self,slot,phase,seq=22,flag=0x4f,error=0):
        return c.b.Status(slot,seq,flag,phase,error,31,2,7,1,0,9)

    def test_parallel_dispatch_and_all_device_saved_barrier(self):
        with tempfile.TemporaryDirectory() as d:
            obj=self.make(Path(d))
            sent=[]
            def send(slot,action,cleanup):
                sent.append(slot)
                tx=c.b.Transaction(slot,22,c.b.FLAGS[action],True)
                tx.observe(self.status(slot,2))
                obj.active_txs[(slot,22,c.b.FLAGS[action])]=tx
                return tx
            def tick(cleanup):
                self.assertEqual(sent,[7,8])
                obj.observe_status(self.status(7,4))
                obj.observe_status(self.status(8,4))
            obj.send_target=send
            obj.tick=tick
            result=obj.action('earth-pose')
            self.assertTrue(all(r['success'] for r in result.values()))
            self.assertEqual(obj.state['devices']['7']['earth'],1)

    def test_unmatched_save_timeout_never_advances(self):
        with tempfile.TemporaryDirectory() as d:
            obj=self.make(Path(d))
            def send(slot,action,cleanup):
                tx=c.b.Transaction(slot,22,0x4f,True)
                tx.observe(self.status(slot,4,seq=21))
                return tx
            obj.send_target=send
            obj.tick=lambda cleanup:None
            with self.assertRaises(ValueError):
                obj.action('earth-pose',timeout=0)
            self.assertEqual(obj.state['devices']['7']['earth'],0)
            self.assertTrue(obj.state['devices']['8']['uncertain'])

    def test_expected_cancel_and_unexpected_async_failure_differ(self):
        with tempfile.TemporaryDirectory() as d:
            obj=self.make(Path(d))
            obj.cancelling_keys.add((7,22,0x4e))
            obj.observe_status(self.status(7,5,flag=0x4e,error=3))
            self.assertIsNone(obj.fatal)
            obj.observe_status(self.status(8,5,flag=0x4e,error=3))
            self.assertIsNotNone(obj.fatal)

    def test_partial_dispatch_preserves_inflight_uncertainty(self):
        with tempfile.TemporaryDirectory() as d:
            obj=self.make(Path(d))
            def send(slot,action,cleanup):
                if slot==8:
                    raise ValueError('receiver unplugged')
                return c.b.Transaction(slot,22,0x4f,True)
            obj.send_target=send
            with self.assertRaises(ValueError):
                obj.action('earth-pose')
            self.assertEqual(obj.state['inflight']['action'],'earth-pose')
            self.assertTrue(all(r['uncertain'] for r in obj.state['devices'].values()))


if __name__=='__main__':
    unittest.main()

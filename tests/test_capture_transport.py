"""Exercise strict slot boundaries, producer backpressure and fleet loss gates."""
from pathlib import Path
import json
import re
import struct
import sys
import tempfile
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from native_test import compile_run
from capture_quality import CaptureQuality, audit_recording, failure_message
from f04_summary import decode_frame, EndManifest, replay
from batch_record_only import readiness_error


def report(seq=0, tracker=0, boot=1, end=None):
    p = bytearray(48)
    p[:4] = bytes((0x15, tracker, 1, 2 if end else 1))
    if end:
        fields = dict(sequence_end=250, valid_windows=250, invalid_windows=0,
                      queue_drop=0, submitted=250, tail_drop=0, deferred=3, partial_samples=2)
        fields.update(end)
        struct.pack_into('<10I', p, 4, boot, 1, *fields.values())
    else:
        struct.pack_into('<4I', p, 4, boot, 1, seq, 20000*(seq+1))
        struct.pack_into('<3i3hhBBH', p, 20, 20000, 0, 0, 1000, 0, 0, 2700, 8, 4, 20000)
    struct.pack_into('<I', p, 44, zlib.crc32(p[:44]))
    return bytes(p) + b'\xc0' + struct.pack('>I', seq) + bytes(11)


class QualityTests(unittest.TestCase):
    def fleet(self, missing=None, ends=True):
        q = CaptureQuality('R', range(10))
        for seq in range(250):
            for i in range(10):
                if (i, seq) != missing:
                    q.add(report(seq, i), seq*20_000_000)
        if ends:
            for i in range(10):
                q.add(report(tracker=i, end={'deferred': 5}), 5_000_000_000)
        return q

    def test_all_members_ready_and_complete(self):
        q = self.fleet(ends=False)
        self.assertTrue(q.ready(5_000_000_000))
        self.assertFalse(q.ready(7_000_000_000))
        for i in range(10):
            q.add(report(tracker=i, end={'deferred': 5}), 5_000_000_000)
        self.assertTrue(q.finish(True, 0)['transport_pass'])
        self.assertEqual(failure_message(q.finish(True,0)),'')

    def test_rejection_names_device_and_distinguishes_device_drop_from_stop(self):
        q=self.fleet(missing=(3,100),ends=False)
        for i in range(10):
            end={'queue_drop':1,'submitted':249} if i==3 else {'queue_drop':0}
            q.add(report(tracker=i,end=end),5_000_000_000)
        result=q.finish(True,0);message=failure_message(result)
        self.assertIn('#3 缺失1个窗口',message)
        self.assertIn('追踪器发送队列溢出1窗',message)
        self.assertIn('结束已确认，接收器丢弃0个报告',message)
        self.assertNotIn('#2',message)
        self.assertFalse(result['transport_pass'])
        unknown=failure_message(self.fleet(ends=False).finish(False,None))
        self.assertIn('缺少结束清单',unknown)
        self.assertIn('结束未确认',unknown)

    def test_readiness_error_distinguishes_transport_orientation_and_motion(self):
        q=self.fleet(ends=False)
        rows={i:((0,0,0),20000,(1000,0,0)) for i in range(10)}
        self.assertEqual(q.readiness_failures(5_000_000_000),[])
        rows[3]=((0,0,0),20000,(0,1000,0))
        rows[6]=((40000,0,0),20000,(1000,0,0))
        error=readiness_error(q,rows,'x',5_000_000_000,Path('diagnostic'))
        self.assertIn('[3]号朝向',error)
        self.assertIn('[6]号仍检测到转动',error)
        self.assertNotIn('数据不连续',error)
        broken=self.fleet(missing=(9,100),ends=False)
        pending=dict(broken.streams[9].pending)
        error=readiness_error(broken,rows,'x',7_000_000_000,Path('diagnostic'))
        self.assertIn('#9 数据不连续',error)
        self.assertIn('超过1秒',error)
        self.assertIn('diagnostic',error)
        self.assertEqual(broken.streams[9].pending,pending)

    def test_internal_leading_and_trailing_single_loss_rejected(self):
        for seq in (0, 100, 249):
            with self.subTest(seq=seq):
                q = self.fleet(missing=(9, seq))
                result = q.finish(True, 0)
                self.assertFalse(result['transport_pass'])
                self.assertEqual(result['streams'][9]['missing_total'], 1)
                self.assertEqual(result['streams'][9]['missing_fraction'], 1/250)

    def test_duplicate_does_not_fill_missing_fleet_or_start_gate(self):
        q = CaptureQuality('R', range(10))
        for n in range(250):
            for i in range(9):
                q.add(report(0, i), n)
        self.assertFalse(q.ready(250))
        self.assertFalse(q.finish(True, 0)['transport_pass'])

    def test_reorder_and_duplicates_recovered_but_conflicts_rejected(self):
        q = CaptureQuality('R', [0])
        for n, seq in enumerate([0, 2, 1, 1]+list(range(3, 250))):
            q.add(report(seq), n)
        q.add(report(end={'deferred': 4}), 1000)
        self.assertTrue(q.finish(True, 0)['transport_pass'])
        self.assertEqual(q.streams[0].reordered, 1)
        self.assertEqual(q.streams[0].duplicates, 1)
        q.add(report(end={'deferred': 5}), 1000)
        self.assertFalse(q.finish(True, 0)['transport_pass'])

    def test_missing_manifest_receiver_drop_stop_and_reboot_fail(self):
        self.assertFalse(self.fleet(ends=False).finish(True, 0)['transport_pass'])
        self.assertFalse(self.fleet().finish(False, 0)['transport_pass'])
        self.assertFalse(self.fleet().finish(True, 1)['transport_pass'])
        self.assertFalse(self.fleet().finish(True, None)['transport_pass'])
        q = self.fleet()
        q.add(report(0, boot=2), 6_000_000_000)
        self.assertFalse(q.finish(True, 0)['transport_pass'])

    def test_manifest_conservation_crc_and_device_drop(self):
        frame = report(end={'deferred': 5})[:48]
        self.assertIsInstance(decode_frame(frame), EndManifest)
        for offset in range(48):
            bad = bytearray(frame); bad[offset] ^= 1
            with self.assertRaises(ValueError):
                decode_frame(bad)
        with self.assertRaisesRegex(ValueError, 'counters'):
            decode_frame(report(end={'queue_drop': 1})[:48])
        q = self.fleet(ends=False)
        for i in range(10):
            q.add(report(tracker=i, end={'queue_drop': 1, 'submitted': 249}), 5_000_000_000)
        self.assertFalse(q.finish(True, 0)['transport_pass'])

    def test_saved_raw_reports_reaudit_and_replay_agree(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'reports.jsonl'
            rows = [dict(format='F04HID1', receiver_serial='R', targets={'0': '010203040506'})]
            for seq in range(250):
                rows.append(dict(host_monotonic_ns=seq, report_hex=report(seq).hex()))
            rows.append(dict(host_monotonic_ns=250, report_hex=report(end={'deferred': 3}).hex()))
            rows.append(dict(event='capture-stop', confirmed=True, receiver_dropped=0))
            path.write_text('\n'.join(map(json.dumps, rows)), encoding='utf8')
            self.assertTrue(audit_recording(path)['transport_pass'])
            self.assertEqual(replay(path)[0]['trailing_unobserved'], 0)


class FirmwarePolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = ROOT/'.local/native-tests'
        cls.native.mkdir(parents=True, exist_ok=True)

    def test_radio_ack_failure_retains_ownership_and_disables_in_slot_retry(self):
        src = (ROOT/'integration/tracker/src/connection/esb.c').read_text(encoding='utf8')
        start = src.index('int esb_write_capture(')
        end = src.index('\nbool esb_ready(', start)
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include "f04_capture_ack.h"
struct esb_payload {unsigned pipe,length;bool noack;uint8_t data[64];};
#define ESB_CREATE_PAYLOAD(n) {0}
#define K_NO_WAIT 0
#define K_MSEC(n) (n)
static int esb_tx_lock,capture_tx_done;
static bool capture_tx_waiting,clock_status=true;
static int capture_tx_result;
static uint8_t capture_tx_frame[F04_SIZE];
static unsigned tracker_id=3,esb_write_queued;
static int64_t last_tx_time;
static struct {unsigned type,length;bool noack;int64_t timestamp;} last_tx;
static bool ready=true,idle=true,slot=true,ack=true;
static int fail_queue,fail_start;
static unsigned queued,started,retries=2,flushes;
static bool esb_ready(void){return ready;}
static bool esb_is_idle(void){return idle;}
static int clocks_start(void){return 0;}
static int k_mutex_lock(int *p,int t){(void)t;assert(!*p);*p=1;return 0;}
static void k_mutex_unlock(int *p){assert(*p);*p=0;}
static int esb_flush_tx(void){assert(idle);flushes++;return 0;}
static bool tdma_capture_wait(void){return slot;}
static bool tdma_capture_claim(void){return slot;}
static void k_sem_reset(int *p){*p=0;}
static unsigned irq_lock(void){return 0;}
static void irq_unlock(unsigned n){(void)n;}
static int64_t k_uptime_get(void){return 123;}
static int esb_set_retransmit_count(unsigned n){assert(idle);retries=n;return 0;}
static int esb_write_payload(const struct esb_payload *p){
assert(!p->noack && p->pipe==4 && p->length==48 && retries==0);
queued++;return fail_queue;}
static int esb_start_tx(void){started++;return fail_start;}
static int k_sem_take(int *p,int timeout){(void)p;assert(timeout==3);
/* Models a decoded, matching receiver receipt; bare radio success is tested
 * separately against the actual receipt handler. */
if(!ack)return 1;
capture_tx_waiting=false;capture_tx_result=0;return 0;}
'''+src[start:end]+r'''
int main(void){
    uint8_t frame[48]={0x15};
    idle=false;assert(esb_write_capture(frame,48)<0);assert(!queued && !flushes);
    idle=true;slot=false;assert(esb_write_capture(frame,48)<0);assert(!queued);
    slot=true;fail_queue=-ENOMEM;assert(esb_write_capture(frame,48)<0);
    assert(!started && retries==2);
    fail_queue=0;fail_start=-EBUSY;assert(esb_write_capture(frame,48)<0 && !capture_tx_waiting);
    fail_start=0;ack=false;assert(esb_write_capture(frame,48)<0 && retries==2);
    ack=true;assert(esb_write_capture(frame,48)==0 && retries==2);
    capture_tx_waiting=true;idle=false;unsigned old=queued;
    assert(esb_write_capture(frame,48)<0 && queued==old);
    idle=true;assert(esb_write_capture(frame,48)==0 && queued==old+1);
    return 0;
}
'''
        with tempfile.TemporaryDirectory(dir=self.native) as d:
            compile_run(code, d, [ROOT/'integration/tracker/src/sensor'])

    def test_every_phase_slot_and_repeated_call(self):
        code = r'''
#include <assert.h>
#include "tdma_capture_core.h"
int main(void) {
    assert(!tdma_capture_config_valid(10,10,16));
    assert(!tdma_capture_config_valid(0,0,16));
    assert(!tdma_capture_config_valid(0,10,31));
    /* Receiver reboot/32-bit rollover: old epoch must not block admission.
     * Small backwards corrections keep the guard until a genuinely new slot. */
    assert(tdma_capture_delay(3202,0,10,32,100000000)==0);
    assert(tdma_capture_delay(3202,0,10,32,3200)>0);
    assert(tdma_capture_delay(3202,0,10,32,3220)>0);
    assert(tdma_capture_delay(3522,0,10,32,3220)==0);
    assert(tdma_capture_delay(3226,0,10,32,100000000)>0); /* still too late */
    const unsigned counts[]={1,10,32}, widths[]={32,34,64,128,254};
    for (unsigned ni=0;ni<3;ni++) for (unsigned wi=0;wi<5;wi++) {
    unsigned n=counts[ni],w=widths[wi];
    for (unsigned s=0;s<n;s++) for (unsigned phase=0;phase<n*w;phase++) {
        uint64_t now=5*n*w+phase;
        unsigned delay=tdma_capture_delay(now,s,n,w,UINT64_MAX);
        if (!delay) {
            assert(phase>=s*w+2 && phase+TDMA_CAPTURE_BUDGET_TICKS<=s*w+w);
            assert(tdma_capture_delay(now,s,n,w,now-phase+s*w)>0);
        } else {
            assert(delay<=n*w);
            assert(tdma_capture_delay(now+delay,s,n,w,UINT64_MAX)==0);
        }
    }
    }
    return 0;
}
'''
        with tempfile.TemporaryDirectory(dir=self.native) as d:
            compile_run(code, d, [ROOT/'integration/tracker/src/connection'])

    def test_real_scheduler_rechecks_late_wakeup_and_stale_sync(self):
        source = (ROOT/'integration/tracker/src/connection/tdma.c').read_text(encoding='utf8')
        source = re.sub(r'^#include .*$', '', source, flags=re.M)
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include "tdma_capture_core.h"
#define CONFIG_CONNECTION_TDMA 1
#include "tdma.h"
#define PING_INTERVAL_MS 997
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define K_TICKS(n) (n)
#define ARG_UNUSED(x) ((void)(x))
static uint64_t ticks=100;
static int64_t age=-1;
static unsigned lateness;
static uint64_t esb_get_server_time_ticks_64(void){return ticks;}
static int64_t esb_get_sync_age_ms(void){return age;}
static int64_t k_uptime_get(void){return ticks/32;}
static void k_sleep(unsigned n){ticks+=n+lateness;}
static unsigned irq_lock(void){return 0;}
static void irq_unlock(unsigned n){(void)n;}
'''+source+r'''
int main(void){
    tdma_update_config(0,10,32,1);
    assert(!tdma_capture_ready());age=0;
    assert(tdma_capture_ready());assert(tdma_capture_wait());assert(tdma_capture_claim());
    assert(!tdma_capture_claim());
    lateness=10;assert(tdma_capture_wait());assert(!tdma_capture_claim());
    age=2*PING_INTERVAL_MS+1;assert(!tdma_capture_wait() && !tdma_capture_claim());
    age=0;tdma_update_config(0,10,16,2);assert(!tdma_capture_ready());
    tdma_update_config(0,10,32,3);tdma_set_enabled(false);assert(!tdma_capture_ready());
    return 0;
}
'''
        with tempfile.TemporaryDirectory(dir=self.native) as d:
            compile_run(code, d, [ROOT/'integration/tracker/src/connection'])

    def test_real_producer_retains_backpressure_and_closes_counts(self):
        source = (ROOT/'integration/tracker/src/sensor/f04_summary.c').read_text(encoding='utf8')
        source = re.sub(r'^#include .*$', '', source, flags=re.M)
        code = r'''
#include <assert.h>
#include <math.h>
#include <string.h>
#include "f04_summary_core.c"
#define CONFIG_ARC8_F04_SUMMARY 1
#define LOG_MODULE_REGISTER(...)
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define K_MUTEX_DEFINE(n) int n
#define K_FOREVER -1
#define K_NO_WAIT 0
#define K_MSGQ_DEFINE(n,sz,cap,align) struct queue n={0}
struct queue { unsigned r,w,n; uint8_t data[64][48]; };
static int k_mutex_lock(int *p,int timeout){(void)timeout;assert(!*p);*p=1;return 0;}
static void k_mutex_unlock(int *p){assert(*p);*p=0;}
static int k_msgq_put(struct queue *q,const void *p,int t){(void)t;if(q->n==64)return -1;
memcpy(q->data[q->w],p,48);q->w=(q->w+1)%64;q->n++;return 0;}
static int k_msgq_peek(struct queue *q,void *p){if(!q->n)return -1;memcpy(p,q->data[q->r],48);return 0;}
static int k_msgq_get(struct queue *q,void *p,int t){(void)t;int r=k_msgq_peek(q,p);if(!r){q->r=(q->r+1)%64;q->n--;}return r;}
static void k_msgq_purge(struct queue *q){memset(q,0,sizeof(*q));}
static unsigned k_msgq_num_used_get(struct queue *q){return q->n;}
typedef int atomic_t;
static int atomic_get(atomic_t *p){return *p;}
static void atomic_set(atomic_t *p,int v){*p=v;}
static void atomic_clear(atomic_t *p){*p=0;}
static uint32_t sys_rand32_get(void){return 123;}
static bool connection_get_data_collection(void){return false;}
static bool arc8_ota_active(void){return false;}
static bool calwire_active(void){return false;}
static bool esb_ready(void){return true;}
static bool tdma_capture_ready(void){return true;}
static int64_t now;
static int64_t k_uptime_get(void){return now;}
static bool reject;
static unsigned sent_count,end_copies;
static uint8_t sent[256][48],end_frame[48];
static int esb_write_capture(const uint8_t *p,size_t n){assert(n==48);
    if(reject)return -1;
    assert(f04_frame_valid(p,n));
    if(p[3]==F04_END_MANIFEST){memcpy(end_frame,p,48);end_copies++;}
    else {memcpy(sent[sent_count++],p,48);}
    return 0;
}
static void k_msleep(int ms);
'''+source+r'''
/* Inject producer calls while STOP drains to verify it cannot append. */
static void k_msleep(int ms){now+=ms;float g[3]={1,0,0},a[3]={1,0,0};
    f04_summary_feed(g,a,27,.0025f,true,true);}
static void produce(unsigned count){float g[3]={1,0,0},a[3]={1,0,0};
    for(unsigned i=0;i<count*8;i++)f04_summary_feed(g,a,27,.0025f,true,true);}
int main(void){
    assert(f04_summary_begin(0,50));produce(10);reject=true;
    f04_summary_drain();assert(sent_count==0 && k_msgq_num_used_get(&f04_queue)==10);
    reject=false;f04_summary_drain();assert(sent_count==2);
    f04_summary_end();assert(sent_count==10 && end_copies==3 && !f04_summary_active());
    assert(f04_u32(end_frame+12)==10 && f04_u32(end_frame+28)==10);
    for(unsigned i=0;i<10;i++)assert(f04_u32(sent[i]+12)==i);
    assert(f04_summary_begin(0,50));produce(70);f04_summary_end();
    assert(f04_u32(end_frame+12)==70 && f04_u32(end_frame+24)==6 && f04_u32(end_frame+28)==64);
    assert(f04_end_valid(end_frame,48));
    assert(f04_summary_begin(0,50));produce(10);reject=true;f04_summary_end();
    assert(!f04_summary_active());assert(now>=3000);
    return 0;
}
'''
        with tempfile.TemporaryDirectory(dir=self.native) as d:
            compile_run(code, d, [ROOT/'integration/tracker/src/sensor'])


if __name__ == '__main__':
    unittest.main()

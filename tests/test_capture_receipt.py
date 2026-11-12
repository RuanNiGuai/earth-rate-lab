"""Real C receipt/admission regressions for the observed silent ACK loss path."""
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from native_test import compile_run


class ReceiptTests(unittest.TestCase):
    def run_c(self, code):
        parent=ROOT/'.local/native-tests';parent.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(dir=parent) as d:
            compile_run(code,d,[ROOT/'integration/tracker/src/sensor',ROOT/'integration/receiver/src/connection'])

    def test_radio_crc_matches_portable_algorithm_for_every_length(self):
        self.run_c(r'''
#include <assert.h>
#include "f04_radio_crc.h"
#include "f04_summary_wire.h"
int main(void){
    assert(f04_radio_crc((const uint8_t*)"123456789",9)==0xcbf43926u);
    uint8_t data[256];uint32_t state=1;
    for(unsigned round=0;round<100;round++){
        for(unsigned i=0;i<256;i++){state=1664525u*state+1013904223u;data[i]=(uint8_t)(state>>24);}
        for(unsigned n=0;n<=256;n++)assert(f04_radio_crc(data,n)==f04_crc(data,n));
    }
    return 0;
}
''')

    def test_shared_headers_identical(self):
        self.assertEqual((ROOT/'integration/tracker/src/sensor/f04_capture_ack.h').read_bytes(),
                         (ROOT/'integration/receiver/src/connection/f04_capture_ack.h').read_bytes())

    def test_last_receipt_timeout_cannot_block_normal_stream_forever(self):
        src=(ROOT/'integration/tracker/src/connection/esb.c').read_text(encoding='utf8')
        start=src.index('\t/* A timed-out capture transaction')
        end=src.index('\tif (esb_is_idle()) (void)esb_set_retransmit_count(2);',start)
        self.run_c(r'''
#include <assert.h>
#include <stdbool.h>
static bool capture_tx_waiting,idle;
static unsigned normal_runs;
static bool esb_is_idle(void){return idle;}
static void normal_entry(void){
'''+src[start:end]+r'''
    normal_runs++;
}
int main(void){
    /* The capture call has timed out and released the TX mutex. With no
     * subsequent F04 call (e.g. end-manifest deadline), normal PING/pose is
     * the only caller that can retire the expired waiter. */
    capture_tx_waiting=true;idle=false;assert(!esb_is_idle());normal_entry();
    assert(!normal_runs && capture_tx_waiting);
    idle=true;normal_entry();
    assert(normal_runs==1 && !capture_tx_waiting);
    normal_entry();assert(normal_runs==2);
    return 0;
}
''')

    def test_wrong_tracker_session_sequence_payload_and_corruption_never_release(self):
        src=(ROOT/'integration/tracker/src/connection/esb.c').read_text(encoding='utf8')
        handler=src[src.index('static void capture_ack_received('):src.index('\nvoid event_handler(')]
        tx=src[src.index('    /* A bare radio ACK'):src.index('\n\tswitch (event->evt_id)')]
        self.run_c(r'''
#include <assert.h>
#include <string.h>
#include "f04_capture_ack.h"
static bool capture_tx_waiting;
static int capture_tx_result,capture_tx_done;
static uint8_t capture_tx_frame[F04_SIZE];
static void k_sem_give(int *p){++*p;}
'''+handler+r'''
enum {ESB_EVENT_TX_SUCCESS,ESB_EVENT_TX_FAILED};
#define EIO 5
struct esb_evt {int evt_id;};
static void observe_tx(int id){struct esb_evt e={id};const struct esb_evt *event=&e;
'''+tx+r'''
}
int main(void){
    uint8_t frame[48]={0x15,7,1,1},ack[24],bad[24];
    f04_put32(frame+4,123);f04_put32(frame+8,2);f04_put32(frame+12,696);
    f04_put32(frame+44,0x12345678);memcpy(capture_tx_frame,frame,48);
    f04_ack_make(ack,frame);capture_tx_waiting=true;capture_tx_result=-1;
    observe_tx(ESB_EVENT_TX_SUCCESS); /* actual production event branch */
    assert(capture_tx_waiting && !capture_tx_done && capture_tx_result==-1);
    capture_ack_received(ack,0);assert(capture_tx_waiting && !capture_tx_done);
    /* Semantically valid ACKs for other Tracker / boot / operation / sequence
     * or payload must fail even if their own CRC is correct. */
    const unsigned offsets[]={1,2,3,4,8,12,16};
    for(unsigned i=0;i<sizeof(offsets)/sizeof(*offsets);i++){
        memcpy(bad,ack,24);bad[offsets[i]]^=1;f04_put32(bad+20,f04_crc(bad,20));
        capture_ack_received(bad,24);assert(capture_tx_waiting && !capture_tx_done);
    }
    for(unsigned i=0;i<24;i++){
        memcpy(bad,ack,24);bad[i]^=1;capture_ack_received(bad,24);
        assert(capture_tx_waiting && !capture_tx_done);
    }
    capture_ack_received(ack,24);assert(!capture_tx_waiting && capture_tx_done==1 && capture_tx_result==0);
    capture_ack_received(ack,24);assert(capture_tx_done==1);
    return 0;
}
''')

    def test_radio_admission_precedes_receipt_and_full_queue_retries(self):
        src=(ROOT/'integration/receiver/src/data_collect_hid.c').read_text(encoding='utf8')
        queue=src[src.index('static bool dc_enqueue_report('):src.index('\nstatic void dc_reset_fifo_locked(')]
        rx=(ROOT/'integration/receiver/src/connection/esb.c').read_text(encoding='utf8')
        admit=rx[rx.index('static bool f04_receive_in_radio('):rx.index('\nstatic void esb_ack_handler_cb(')]
        self.run_c(r'''
#include <assert.h>
#include <string.h>
#include "f04_radio_crc.h"
#define F04_CRC_OVERRIDE f04_radio_crc
#include "f04_capture_ack.h"
#define CONFIG_DATA_COLLECT_HID 1
#define DC_REPORT_SIZE 64
#define DC_FIFO_SIZE 64
static unsigned stored_trackers=10;
static unsigned fresh_membership;
static void tdma_shadow_capture_seen(uint8_t id){assert(id<stored_trackers);fresh_membership++;}
static void *dc_hdev=(void*)1;
static bool dc_hid_ready=true,dc_active,dc_batch_active=true;
static unsigned dc_batch_mask=1023,dc_fifo_read,dc_fifo_write;
static uint8_t dc_fifo[64][64];
static unsigned dc_frames_received,dc_frames_dropped,dc_capture_backpressure;
static int64_t dc_last_rx_time;
static int dc_send_work;
static unsigned critical,scheduled;
static unsigned irq_lock(void){assert(!critical);critical=1;return 0;}
static void irq_unlock(unsigned key){(void)key;assert(critical);critical=0;}
static unsigned atomic_get(unsigned *p){return *p;}
static void atomic_set(unsigned *p,unsigned v){assert(critical);*p=v;}
typedef unsigned atomic_val_t;
static unsigned dc_fifo_next(unsigned n){return (n+1)%64;}
static bool dc_fifo_empty(void){return dc_fifo_read==dc_fifo_write;}
static int64_t k_uptime_get(void){return 123;}
static uint32_t k_uptime_ticks(void){return 999;}
static void sys_put_be32(uint32_t v,uint8_t *p){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(24-8*i));}
static void k_work_submit(int *p){(void)p;assert(!critical);scheduled++;}
static bool data_collect_batch_is_target(uint8_t id){return dc_batch_active && id<32 && (dc_batch_mask&(1u<<id));}
'''+queue+admit+r'''
static void checksum(uint8_t *frame){f04_put32(frame+44,f04_crc(frame,44));}
int main(void){
    uint8_t frame[48]={0x15,7,1,1},receipt[24];
    f04_put32(frame+4,123);f04_put32(frame+8,2);f04_put32(frame+12,696);
    frame[40]=8;frame[41]=2;f04_put16(frame+42,20000);checksum(frame);
    assert(f04_frame_valid(frame,48));
    assert(!f04_receive_in_radio(frame,48,2,19,receipt)); /* wrong pipe */
    assert(!dc_frames_received && !fresh_membership);
    assert(f04_receive_in_radio(frame,48,1,19,receipt));
    assert(dc_frames_received==1 && !memcmp(dc_fifo[0],frame,48));
    assert(!scheduled); /* no workqueue scheduler in RADIO turnaround */
    data_collect_capture_kick();assert(scheduled==1);
    assert(f04_ack_matches(receipt,24,frame));
    /* ESB hardware duplicate suppression may skip the event; RADIO callback
     * is still invoked. Retrying after lost ACK remains recoverable. */
    assert(f04_receive_in_radio(frame,48,1,19,receipt));
    frame[1]=0;checksum(frame); /* shared pipe, independent frame identity */
    assert(!f04_ack_matches(receipt,24,frame));
    for(unsigned i=2;i<63;i++)assert(f04_receive_in_radio(frame,48,1,19,receipt));
    uint8_t copy[64];memcpy(copy,dc_fifo[0],64);
    assert(!f04_receive_in_radio(frame,48,1,19,receipt));
    assert(dc_frames_received==63 && dc_capture_backpressure==1 && !dc_frames_dropped);
    assert(!memcmp(copy,dc_fifo[0],64)); /* in-flight slot never overwritten */
    dc_fifo_read=1;assert(f04_receive_in_radio(frame,48,1,19,receipt));
    assert(dc_frames_received==64 && fresh_membership==64);
    dc_fifo_read=2;frame[20]^=1;
    assert(!f04_receive_in_radio(frame,48,1,19,receipt)); /* bad CRC */
    assert(dc_frames_received==64 && fresh_membership==64);
    frame[20]^=1;dc_batch_active=false;
    assert(!f04_receive_in_radio(frame,48,1,19,receipt));
    dc_batch_active=true;dc_hid_ready=false;
    assert(!f04_receive_in_radio(frame,48,1,19,receipt));
    /* Legacy raw path keeps explicit drops; capture backpressure does not
     * masquerade as a received/forwarded window. */
    data_collect_write(frame,48,19);assert(dc_frames_dropped==1 && dc_frames_received==65);
    return 0;
}
''')


if __name__=='__main__':unittest.main()

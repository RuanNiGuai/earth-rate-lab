/* ARC5: all archive operations remain optional/lazy.  Sensor callback
 * never reads/writes NVS or runs the solver. No production auto-commit. */
#include "sensor/cal_archive_safe.h"
#include "sensor/calwire.h"
#include "sensor/cal_archive_core.h"
#include "sensor/cal_archive_wire.h"
#include "sensor/calibration.h"
#include "sensor/sensor.h"
#include "sensor/fusion/vqf/vqf.h"
#include "system/system.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>

#define ARC_META_ID0 100u
#define ARC_TCAL_ID0 102u
#define ARC_VERSION 4u
#define ARC_MAGIC 0x41524334u
#define ARC_SAMPLES_REQUIRED 1600u
#define ARC_BUFFER_POINT_LIMIT CAL_ARC_TMAX

typedef struct {
    uint32_t magic, version, generation, reference_id;
    float reference_accel[3];
    uint32_t checksum;
} arc_meta_t;

/* All large objects static: sensor task stack is only 2048 bytes. */
static K_THREAD_STACK_DEFINE(arc_stack, 6144);
static struct k_work_q arc_queue;
static struct k_work arc_control_work, arc_save_work;
static atomic_t arc_thread_started, arc_active, arc_collect_request, arc_capture_running;
static atomic_t arc_save_pending, arc_cmd_busy;
static volatile bool arc_reference_confirmed;
static cal_arc_tcal_t arc_points;
static arc_meta_t arc_meta;
static union { cal_arc_tcal_t t; arc_meta_t m; } arc_io_buffer;
static struct {
    uint32_t samples;
    double gyro[3], sq[3], accel[3], temp, time;
    float min_temp, max_temp;
} arc_accumulator;
static cal_arc_point_t arc_ready_point;
/* The work item is only a wakeup: never read a mutable global command.
 * A size-one queue owns a complete immutable request until consumed. */
typedef struct {
    int cmd;
    uint8_t flag, seq;
    uint32_t token;
} arc_request_t;
K_MSGQ_DEFINE(arc_cmd_msgq, sizeof(arc_request_t), 1, 4);
static atomic_t arc_next_token, arc_expected_token;
/* Transition queue: radio drain is done from ESB thread, never sensor ISR.
 * Oldest QUEUED can be sacrificed under overflow; final state remains in
 * arc_recent[] for explicit duplicate replay. */
typedef struct { uint8_t seq, flag, phase, error; } arc_event_t;
K_MSGQ_DEFINE(arc_event_msgq, sizeof(arc_event_t), 12, 4);
static void arc_enqueue_event(uint8_t seq, uint8_t flag, uint8_t phase, uint8_t error)
{
    if (!seq || flag < 0x40 || flag > 0x53) return; /* legacy USB */
    arc_event_t ev = { seq, flag, phase, error };
    if (k_msgq_put(&arc_event_msgq, &ev, K_NO_WAIT) != 0) {
        arc_event_t old;
        (void)k_msgq_get(&arc_event_msgq, &old, K_NO_WAIT);
        (void)k_msgq_put(&arc_event_msgq, &ev, K_NO_WAIT);
    }
}
static cal_arc_status_t arc_last_status;
static cal_arc_status_t arc_recent[2];
static uint8_t arc_recent_valid[2];
static uint8_t arc_capture_seq, arc_capture_flag;
static uint8_t arc_capture_valid;
static arc_request_t arc_executing_request; /* worker-owned until execute returns */

/* IRQ-safe small record update only; never run NVS or radio from here. */
static void arc_phase(uint8_t seq, uint8_t flag, uint8_t phase, uint8_t error)
{
    unsigned int key = irq_lock();
    bool terminal_before_execute =
        arc_last_status.seq==seq && arc_last_status.flag==flag &&
        (arc_last_status.phase==ARC_PHASE_SAVED || arc_last_status.phase==ARC_PHASE_FAILED) &&
        phase==ARC_PHASE_EXECUTED;
    if(!terminal_before_execute) {
        arc_last_status.seq=seq; arc_last_status.flag=flag;
        arc_last_status.phase=phase; arc_last_status.error=error;
    }
    for(unsigned i=0;i<2;i++) {
        if(arc_recent_valid[i] && arc_recent[i].seq==seq &&
           arc_recent[i].flag==flag) {
            if(!(phase==ARC_PHASE_EXECUTED &&
               (arc_recent[i].phase==ARC_PHASE_SAVED ||
                arc_recent[i].phase==ARC_PHASE_FAILED))) {
                arc_recent[i].phase=phase;
                arc_recent[i].error=error;
            }
        }
    }
    irq_unlock(key);
    if (!terminal_before_execute)
        arc_enqueue_event(seq, flag, phase, error);
}
static void arc_capture_phase(uint8_t phase, uint8_t error)
{
    unsigned int key=irq_lock();
    if(arc_capture_valid) {
        uint8_t seq=arc_capture_seq, flag=arc_capture_flag;
        irq_unlock(key);
        arc_phase(seq,flag,phase,error);
        if(phase==ARC_PHASE_SAVED || phase==ARC_PHASE_FAILED) {
            key=irq_lock();
            if(arc_capture_valid && arc_capture_seq==seq && arc_capture_flag==flag)
                arc_capture_valid=0;
            irq_unlock(key);
        }
    } else irq_unlock(key);
}
static void arc_capture_arm(uint8_t seq, uint8_t flag)
{
    unsigned int key=irq_lock();
    arc_capture_seq=seq; arc_capture_flag=flag;
    arc_capture_valid=1;
    irq_unlock(key);
}
static void arc_capture_arm_current(void)
{
    arc_capture_arm(arc_executing_request.seq,arc_executing_request.flag);
}
static uint8_t arc_error_code(const char *s)
{
    if(strstr(s,"compression") || strstr(s,"cross-axis"))return ARC_ERROR_COMPRESSION;
    if(strstr(s,"insufficient") || strstr(s,"20-turn"))return ARC_ERROR_ROTATION;
    if(strstr(s,"duplicate"))return ARC_ERROR_DUPLICATE_POSE;
    if(strstr(s,"cardinal") || strstr(s,"axis-up"))return ARC_ERROR_CARDINAL;
    if(strstr(s,"timed out"))return ARC_ERROR_TIMEOUT;
    if(strstr(s,"NVS") || strstr(s,"write/readback"))return ARC_ERROR_NVS;
    if(strstr(s,"queue"))return ARC_ERROR_QUEUE;
    if(strstr(s,"reference"))return ARC_ERROR_REFERENCE;
    if(strstr(s,"temperature") || strstr(s,"noise") || strstr(s,"unstable"))return ARC_ERROR_UNSTABLE;
    if(strstr(s,"range") || strstr(s,"saturation") || strstr(s,"invalid"))return ARC_ERROR_RANGE;
    return ARC_ERROR_OTHER;
}


/* ARC5 stage-2 functions are defined below; archive is worker-owned except
 * for the bounded, lock-free sensor sampler and save handoff. */
static void arc2_load(void);
static void arc2_reset(void);
static void arc2_off(void);
static void arc2_status(void);
static bool arc2_busy(void);
static bool arc2_control(int cmd);
static void arc2_feed(const float raw[3],const float a[3],float T,float dt,bool rest);
static bool arc2_save_pending(void);
static void arc2_flush(void);
static void arc2_solve_preview(void);
static void arc2_export_wire(void);
#if defined(CONFIG_ARC8P1_READONLY_EXPORT_DRAFT)
static bool arc2_partial_export(unsigned kind);
static bool arc_tcal_export(void);
#endif

enum {
    ARC_ON=1, ARC_OFF, ARC_STATUS, ARC_STORAGE, ARC_REF_NEW,
    ARC_REF_CONFIRM, ARC_TCAL_START, ARC_SOLVE,
    ARC_SX_START=20,ARC_SY_START,ARC_SZ_START,
    ARC_SX_STOP,ARC_SY_STOP,ARC_SZ_STOP,
    ARC_EARTH_START=30,ARC_EARTH_STOP,ARC_GSENS_START,ARC_GSENS_POSE,
    ARC_CAPTURE_CANCEL, ARC_COMBINED_START, ARC_EXPORT=60
#if defined(CONFIG_ARC8P1_READONLY_EXPORT_DRAFT)
    , ARC_P1_SCALE=70, ARC_P1_EARTH=71, ARC_P1_GSENS=72, ARC_TCAL_EXPORT=73
#endif
};

static bool arc_meta_valid(const arc_meta_t *m)
{
    return m->magic == ARC_MAGIC && m->version == ARC_VERSION &&
        m->reference_id != 0 &&
        m->checksum == cal_arc_checksum(m, offsetof(arc_meta_t, checksum));
}

static bool arc_points_valid(const cal_arc_tcal_t *t)
{
    return t->magic == CAL_ARC_MAGIC && t->version == CAL_ARC_VERSION &&
        t->count <= ARC_BUFFER_POINT_LIMIT &&
        t->checksum == cal_arc_checksum(t, offsetof(cal_arc_tcal_t, checksum));
}

/* Called exclusively from the on-demand worker; scratch never on stack. */
static bool arc_read_meta(void)
{
    bool found=false;
    arc_meta_t best={0};
    for (int slot=0;slot<2;slot++) {
        memset(&arc_io_buffer,0,sizeof(arc_io_buffer));
        int len=sys_arc_read(ARC_META_ID0+slot, &arc_io_buffer.m, sizeof(arc_meta_t));
        if (len==(int)sizeof(arc_meta_t) && arc_meta_valid(&arc_io_buffer.m) &&
            (!found || arc_io_buffer.m.generation>best.generation)) {
            best=arc_io_buffer.m; found=true;
        }
    }
    arc_meta=best;
    return found;
}
static void arc_read_points(void)
{
    bool found=false;
    uint32_t best_generation=0;
    for(int slot=0;slot<2;slot++) {
        memset(&arc_io_buffer,0,sizeof(arc_io_buffer));
        int len=sys_arc_read(ARC_TCAL_ID0+slot,&arc_io_buffer.t,sizeof(cal_arc_tcal_t));
        if(len==(int)sizeof(cal_arc_tcal_t) && arc_points_valid(&arc_io_buffer.t) &&
           arc_io_buffer.t.reference_id==arc_meta.reference_id &&
           (!found || arc_io_buffer.t.generation>best_generation)) {
            arc_points=arc_io_buffer.t;
            best_generation=arc_points.generation;
            found=true;
        }
    }
    if(!found) memset(&arc_points,0,sizeof(arc_points));
}
static bool arc_write_meta(void)
{
    arc_meta.magic=ARC_MAGIC; arc_meta.version=ARC_VERSION;
    arc_meta.checksum=cal_arc_checksum(&arc_meta,offsetof(arc_meta_t,checksum));
    uint16_t id=(uint16_t)(ARC_META_ID0+(arc_meta.generation&1));
    int result=sys_arc_write(id,&arc_meta,sizeof(arc_meta));
    if(result < 0) return false;
    memset(&arc_io_buffer,0,sizeof(arc_io_buffer));
    result=sys_arc_read(id,&arc_io_buffer.m,sizeof(arc_meta_t));
    return result==(int)sizeof(arc_meta_t) &&
           !memcmp(&arc_io_buffer.m,&arc_meta,sizeof(arc_meta));
}
static bool arc_write_points(void)
{
    arc_points.magic=CAL_ARC_MAGIC;
    arc_points.version=CAL_ARC_VERSION;
    arc_points.reference_id=arc_meta.reference_id;
    arc_points.generation++;
    arc_points.checksum=cal_arc_checksum(&arc_points,
                               offsetof(cal_arc_tcal_t,checksum));
    uint16_t id=(uint16_t)(ARC_TCAL_ID0+(arc_points.generation&1));
    int result=sys_arc_write(id,&arc_points,sizeof(arc_points));
    if(result<0)return false;
    memset(&arc_io_buffer,0,sizeof(arc_io_buffer));
    result=sys_arc_read(id,&arc_io_buffer.t,sizeof(cal_arc_tcal_t));
    return result==(int)sizeof(cal_arc_tcal_t) &&
           !memcmp(&arc_io_buffer.t,&arc_points,sizeof(arc_points));
}

static bool arc_reference_ready(float accel[3])
{
    if(!sensor_boot_zro_ready() || !vqf_get_rest_detected())return false;
    sensor_get_last_accel(accel);
    float a=sqrtf(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    return isfinite(a) && a>.97f && a<1.03f;
}
static bool arc_reference_accel_matches(const float a[3])
{
    float diff=0;
    for(int i=0;i<3;i++) {
        float d=a[i]-arc_meta.reference_accel[i];
        diff+=d*d;
    }
    return diff < 0.05f*0.05f; /* ~2.9 deg max tilt difference */
}

/* Worker-only command handling; normal boot never schedules this. */
static bool arc_control_execute(int cmd)
{
    if(cmd==ARC_ON) {
        if(!atomic_get(&arc_active)) {
            (void)arc_read_meta();
            if(arc_meta_valid(&arc_meta)) arc_read_points();
            else memset(&arc_points,0,sizeof(arc_points));
            arc2_load(); /* Lazy: only after explicit ON; never during boot. */
            arc_reference_confirmed=false; /* must reconfirm after EVERY reboot */
            atomic_set(&arc_active,1);
        }
        printk("ARC4 ON: reference confirmation required each boot\n");
        return true;
    }
    if(cmd==ARC_OFF) {
        atomic_clear(&arc_active);
        atomic_clear(&arc_collect_request);
        arc2_off();
        arc_reference_confirmed=false;
        printk("ARC4 OFF\n");return true;
    }
    if(cmd==ARC_STORAGE){sys_arc_storage_status();return true;}
    if(cmd==ARC_EXPORT) {
        /* Explicit, read-only export; no reference confirmation needed. */
        if(atomic_get(&arc_capture_running) || atomic_get(&arc_save_pending) ||
           arc2_busy() || arc2_save_pending() || atomic_get(&arc_collect_request)) {
            printk("ARC8X1 ERROR busy\n"); return false;
        }
        if(!arc_read_meta() || !arc_meta_valid(&arc_meta)) {
            printk("ARC8X1 ERROR missing_reference\n"); return false;
        }
        arc_read_points();
        arc2_load(); /* Re-read validated, persisted NVS records. */
        arc2_export_wire();
        return true;
    }
#if defined(CONFIG_ARC8P1_READONLY_EXPORT_DRAFT)
    if(cmd==ARC_TCAL_EXPORT) return arc_tcal_export();
    if(cmd>=ARC_P1_SCALE && cmd<=ARC_P1_GSENS) {
        /* No active capture may run. Only worker-owned, already-loaded RAM. */
        if(!atomic_get(&arc_active) || !arc_meta_valid(&arc_meta) ||
           atomic_get(&arc_capture_running) || atomic_get(&arc_save_pending) ||
           atomic_get(&arc_collect_request) || arc2_busy() ||
           arc2_save_pending()) {
            printk("ARC8P1 ERROR busy_or_not_loaded\n");
            return false;
        }
        return arc2_partial_export((unsigned)(cmd-ARC_P1_SCALE+1u));
    }
#endif
    if(cmd==ARC_STATUS) {
        printk("ARC5 enabled=%d ref_id=%u confirmed=%d tcal_points=%u collecting=%d save_pending=%d\n",
            atomic_get(&arc_active),arc_meta.reference_id,
            arc_reference_confirmed,arc_points.count,
            atomic_get(&arc_capture_running),atomic_get(&arc_save_pending));
        arc2_status();
        return true;
    }
    if(!atomic_get(&arc_active)) {printk("ARC4: first run cal archive on\n");return false;}
    if(cmd==ARC_REF_NEW || cmd==ARC_REF_CONFIRM) {
        if(arc2_busy()||arc2_save_pending()||
           atomic_get(&arc_capture_running)||atomic_get(&arc_save_pending)) {
            printk("ARC5 reference change refused: capture/save busy\n");return false;
        }
        float a[3];
        if(!arc_reference_ready(a)) {
            printk("ARC4 reference rejected: ZRO, VQF rest or accel not ready\n");return false;
        }
        if(cmd==ARC_REF_CONFIRM && arc_meta_valid(&arc_meta) &&
           !arc_reference_accel_matches(a)) {
            printk("ARC4 reference rejected: tilt differs from stored reference (Yaw must be checked manually)\n");
            return false;
        }
        if(cmd==ARC_REF_NEW || !arc_meta_valid(&arc_meta)) {
            memcpy(arc_meta.reference_accel,a,sizeof(a));
            arc_meta.reference_id++;
            if(!arc_meta.reference_id)arc_meta.reference_id=1;
            arc_meta.generation++;
            if(!arc_write_meta()) {
                printk("ARC4 reference NVS save failed\n");return false;
            }
            memset(&arc_points,0,sizeof(arc_points));
            arc2_reset();
        }
        arc_reference_confirmed=true;
        printk("ARC4 reference confirmed id=%u (manual physical pose check REQUIRED)\n",
               arc_meta.reference_id);
        return true;
    }
    if(cmd>=ARC_SX_START && cmd<=ARC_COMBINED_START) {
        if(!arc_reference_confirmed || !arc_meta_valid(&arc_meta)) {
            printk("ARC5 capture refused: confirm physical reference first\n");return false;
        }
        return arc2_control(cmd);
    }
    if(cmd==ARC_TCAL_START) {
        float a[3];
        if(!arc_reference_ready(a) || !arc_reference_accel_matches(a) ||
           !arc_reference_confirmed || arc2_busy() ||
           atomic_get(&arc_capture_running) ||
           atomic_get(&arc_collect_request) || atomic_get(&arc_save_pending)) {
            printk("ARC4 TCal rejected: reference not confirmed or busy\n");return false;
        }
        arc_capture_arm_current();
        atomic_set(&arc_collect_request,1);
        printk("ARC4 TCal raw capture queued (reference pose, stable rest)\n");return true;
    }
    if(cmd==ARC_SOLVE) { arc2_solve_preview(); return true; }
    return false;
}

static void arc_control_handler(struct k_work *unused)
{
    (void)unused;
    arc_request_t req;
    if(k_msgq_get(&arc_cmd_msgq,&req,K_NO_WAIT)!=0)return; /* spurious reschedule */
    if(req.token!=(uint32_t)atomic_get(&arc_expected_token)) {
        arc_phase(req.seq,req.flag,ARC_PHASE_REJECTED,ARC_ERROR_QUEUE);
        atomic_clear(&arc_cmd_busy);
        return;
    }
    arc_executing_request=req;
    bool ok=arc_control_execute(req.cmd);
    if(ok && (req.cmd==ARC_CAPTURE_CANCEL || req.cmd==ARC_OFF))
        arc_capture_phase(ARC_PHASE_FAILED,ARC_ERROR_WRONG_STATE);
    arc_phase(req.seq,req.flag,ok?ARC_PHASE_EXECUTED:ARC_PHASE_REJECTED,
              ok?ARC_ERROR_NONE:ARC_ERROR_WRONG_STATE);
    atomic_clear(&arc_cmd_busy);
}

static void arc_save_handler(struct k_work *unused)
{
    (void)unused;
    if(arc2_save_pending()) { arc2_flush(); return; }
    if(!atomic_get(&arc_save_pending))return;
    if(!atomic_get(&arc_active) || !arc_reference_confirmed ||
       !arc_meta_valid(&arc_meta) || arc_points.count>=ARC_BUFFER_POINT_LIMIT) {
        printk("ARC4 TCal discard: reference invalid, disabled or points full\n");
        atomic_clear(&arc_save_pending);
        arc_capture_phase(ARC_PHASE_FAILED,ARC_ERROR_REFERENCE);
        return;
    }
    arc_points.point[arc_points.count]=arc_ready_point;
    arc_points.count++;
    if(arc_write_points()) {
        printk("ARC4 TCal saved: %u/%u raw reference points\n",
               arc_points.count, ARC_BUFFER_POINT_LIMIT);
        arc_capture_phase(ARC_PHASE_SAVED,ARC_ERROR_NONE);
    } else {
        arc_points.count--;
        printk("ARC4 TCal NVS write/readback FAILED; production data unchanged\n");
        arc_capture_phase(ARC_PHASE_FAILED,ARC_ERROR_NVS);
    }
    atomic_clear(&arc_save_pending);
}

static bool arc_start_worker(void)
{
    /* 0=never started, 1=initializing, 2=ready. Do not publish ready
     * before the queue/work objects have been initialized. */
    if(atomic_get(&arc_thread_started)==2)return true;
    if(!atomic_cas(&arc_thread_started,0,1))return false;
    k_work_init(&arc_control_work,arc_control_handler);
    k_work_init(&arc_save_work,arc_save_handler);
    k_work_queue_start(&arc_queue,arc_stack,K_THREAD_STACK_SIZEOF(arc_stack),12,NULL);
    atomic_set(&arc_thread_started,2);
    return true;
}

cal_arc_submit_t cal_archive_command_txn(const char *command, uint8_t flag, uint8_t seq)
{
    if(calwire_active())return ARC_SUBMIT_BUSY;
    int c=0;
    if(!command)return ARC_SUBMIT_INVALID;
    if(!strcmp(command,"on")) c=ARC_ON;
    else if(!strcmp(command,"off")) c=ARC_OFF;
    else if(!strcmp(command,"status")) c=ARC_STATUS;
    else if(!strcmp(command,"storage")) c=ARC_STORAGE;
    else if(!strcmp(command,"ref new")) c=ARC_REF_NEW;
    else if(!strcmp(command,"ref confirm")) c=ARC_REF_CONFIRM;
    else if(!strcmp(command,"tcal start")) c=ARC_TCAL_START;
    else if(!strcmp(command,"solve")) c=ARC_SOLVE;
    else if(!strcmp(command,"scale x start")) c=ARC_SX_START;
    else if(!strcmp(command,"scale y start")) c=ARC_SY_START;
    else if(!strcmp(command,"scale z start")) c=ARC_SZ_START;
    else if(!strcmp(command,"scale x stop")) c=ARC_SX_STOP;
    else if(!strcmp(command,"scale y stop")) c=ARC_SY_STOP;
    else if(!strcmp(command,"scale z stop")) c=ARC_SZ_STOP;
    else if(!strcmp(command,"earth start")) c=ARC_EARTH_START;
    else if(!strcmp(command,"earth stop")) c=ARC_EARTH_STOP;
    else if(!strcmp(command,"gsens start")) c=ARC_GSENS_START;
    else if(!strcmp(command,"gsens pose")) c=ARC_GSENS_POSE;
    else if(!strcmp(command,"capture cancel")) c=ARC_CAPTURE_CANCEL;
    else if(!strcmp(command,"combined start")) c=ARC_COMBINED_START;
    else if(!strcmp(command,"export")) c=ARC_EXPORT;
#if defined(CONFIG_ARC8P1_READONLY_EXPORT_DRAFT)
    else if(!strcmp(command,"export partial scale")) c=ARC_P1_SCALE;
    else if(!strcmp(command,"export partial earth")) c=ARC_P1_EARTH;
    else if(!strcmp(command,"export partial gsens")) c=ARC_P1_GSENS;
    else if(!strcmp(command,"export tcal")) c=ARC_TCAL_EXPORT;
#endif
    if(!c) return ARC_SUBMIT_INVALID;
    if(c==ARC_OFF && !atomic_get(&arc_thread_started)) {
        printk("ARC4 OFF (never initialized)\n");
        /* No worker runs in this branch: publish an actual completion. */
        arc_enqueue_event(seq,flag,ARC_PHASE_EXECUTED,ARC_ERROR_NONE);
        return ARC_SUBMIT_QUEUED;
    }
    if(!arc_start_worker())return ARC_SUBMIT_BUSY;
    /* seq=0 is legacy (no dedup). Status may be queried independently. */
    if(seq && c!=ARC_STATUS) {
        unsigned int key=irq_lock();
        for(unsigned i=0;i<2;i++) {
            if(arc_recent_valid[i] && arc_recent[i].seq==seq &&
               arc_recent[i].flag==flag) {
                cal_arc_status_t replay = arc_recent[i];
                irq_unlock(key);
                arc_enqueue_event(replay.seq, replay.flag,
                                  replay.phase, replay.error);
                return ARC_SUBMIT_DUPLICATE;
            }
        }
        irq_unlock(key);
    }
    if(!atomic_cas(&arc_cmd_busy,0,1)) {
        printk("ARC4 busy; retry command\n");
        return ARC_SUBMIT_BUSY;
    }
    arc_request_t req={.cmd=c,.seq=seq,.flag=flag,
                       .token=(uint32_t)atomic_inc(&arc_next_token)+1u};
    atomic_set(&arc_expected_token,(atomic_val_t)req.token);
    /* Admission happens at message-queue insertion, not work submission.
     * Zephyr submit=0 and submit=2 are valid wakeup states: msgq owns the
     * one immutable request, and an empty handler is a no-op. */
    int put=k_msgq_put(&arc_cmd_msgq,&req,K_NO_WAIT);
    if(put) {
        atomic_clear(&arc_cmd_busy);
        return put==-ENOMSG ? ARC_SUBMIT_BUSY : ARC_SUBMIT_REJECTED;
    }
    if(seq && c!=ARC_STATUS) {
        unsigned int key=irq_lock();
        arc_recent[1]=arc_recent[0];arc_recent_valid[1]=arc_recent_valid[0];
        arc_recent[0]=(cal_arc_status_t){.seq=seq,.flag=flag,
                                      .phase=ARC_PHASE_QUEUED};
        arc_recent_valid[0]=1;
        irq_unlock(key);
    }
    arc_phase(seq,flag,ARC_PHASE_QUEUED,ARC_ERROR_NONE);
    int submit=k_work_submit_to_queue(&arc_queue,&arc_control_work);
    if(submit<0) {
        arc_request_t discarded;
        int removed=k_msgq_get(&arc_cmd_msgq,&discarded,K_NO_WAIT);
        if(removed==0) {
            arc_phase(seq,flag,ARC_PHASE_REJECTED,ARC_ERROR_QUEUE);
            if(seq) {
                unsigned int key=irq_lock();
                for(unsigned i=0;i<2;i++)
                    if(arc_recent_valid[i] && arc_recent[i].seq==seq &&
                       arc_recent[i].flag==flag)arc_recent_valid[i]=0;
                irq_unlock(key);
            }
            atomic_clear(&arc_cmd_busy);
            printk("ARC4 worker queue rejected request (%d)\n",submit);
            return ARC_SUBMIT_REJECTED;
        }
        /* Work may have consumed the request: don't report a false rejection
         * or clear the worker's admission guard while it's executing. */
        return ARC_SUBMIT_QUEUED;
    }
    return ARC_SUBMIT_QUEUED;
}

bool cal_archive_command(const char *command)
{
    return cal_archive_command_txn(command,0,0)==ARC_SUBMIT_QUEUED;
}


/* Called only from sensor_calibration_process_gyro BEFORE TCal/g-sens subtraction.
 * No NVS operations, no solver, no large local stack allocations. */
void cal_archive_feed_raw(const float g[3], float T, float dt, bool rest)
{
    if(!atomic_get(&arc_active) || !arc_reference_confirmed)return;
    float arc2_a[3];sensor_get_last_accel(arc2_a);
    arc2_feed(g,arc2_a,T,dt,rest);
    if(atomic_get(&arc_save_pending))return;
    if(atomic_cas(&arc_collect_request,1,0)) {
        memset(&arc_accumulator,0,sizeof(arc_accumulator));
        atomic_set(&arc_capture_running,1);
    }
    if(!atomic_get(&arc_capture_running))return;
    float a[3];sensor_get_last_accel(a);
    float am=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if(!rest || !isfinite(T) || !isfinite(dt) || dt<=0 || dt>.025f ||
       am<.97f || am>1.03f || !arc_reference_accel_matches(a)) {
        atomic_clear(&arc_capture_running);
        arc_capture_phase(ARC_PHASE_FAILED,ARC_ERROR_UNSTABLE);
        printk("ARC4 TCal cancelled: unstable sample\n");return;
    }
    if(!arc_accumulator.samples){arc_accumulator.min_temp=T;arc_accumulator.max_temp=T;}
    if(T<arc_accumulator.min_temp)arc_accumulator.min_temp=T;
    if(T>arc_accumulator.max_temp)arc_accumulator.max_temp=T;
    if(arc_accumulator.max_temp-arc_accumulator.min_temp>.15f){
        atomic_clear(&arc_capture_running);
        arc_capture_phase(ARC_PHASE_FAILED,ARC_ERROR_UNSTABLE);
        printk("ARC4 TCal cancelled: temperature unstable\n");return;
    }
    arc_accumulator.samples++;
    arc_accumulator.temp+=T;
    arc_accumulator.time+=dt;
    for(int i=0;i<3;i++) {
        if(!isfinite(g[i]) || fabsf(g[i])>5.f){
            atomic_clear(&arc_capture_running);
            arc_capture_phase(ARC_PHASE_FAILED,ARC_ERROR_RANGE);return;
        }
        arc_accumulator.gyro[i]+=g[i];
        arc_accumulator.sq[i]+=(double)g[i]*g[i];
        arc_accumulator.accel[i]+=a[i];
    }
    if(arc_accumulator.samples<ARC_SAMPLES_REQUIRED)return;
    cal_arc_point_t *p=&arc_ready_point;
    memset(p,0,sizeof(*p));
    double n=arc_accumulator.samples;
    p->samples=arc_accumulator.samples;
    p->seconds=(float)arc_accumulator.time;
    p->temp=(float)(arc_accumulator.temp/n);
    for(int i=0;i<3;i++) {
        p->gyro[i]=(float)(arc_accumulator.gyro[i]/n);
        p->accel[i]=(float)(arc_accumulator.accel[i]/n);
        double var=arc_accumulator.sq[i]/n-(arc_accumulator.gyro[i]/n)*(arc_accumulator.gyro[i]/n);
        p->gyro_sem[i]=(float)sqrt(var>0?var/n:0);
        if(p->gyro_sem[i]>.003f) {
            atomic_clear(&arc_capture_running);
            arc_capture_phase(ARC_PHASE_FAILED,ARC_ERROR_UNSTABLE);
            printk("ARC4 TCal cancelled: noisy raw gyro\n");return;
        }
    }
    /* Reference record remains in sensor axes, NEVER rotate intrinsic bias. */
    p->quat[0]=1.f;
    atomic_set(&arc_save_pending,1);
    atomic_clear(&arc_capture_running);
    if(k_work_submit_to_queue(&arc_queue,&arc_save_work)<0) {
        atomic_clear(&arc_save_pending);
        arc_capture_phase(ARC_PHASE_FAILED,ARC_ERROR_QUEUE);
    }
}

#include "cal_archive_stage2.inc"

void cal_archive_status_snapshot(cal_arc_status_t *out)
{
    if(!out)return;
    unsigned int key=irq_lock();
    *out=arc_last_status;
    irq_unlock(key);
    /* Progress is best-effort until the full CAL_STATUS_RSP worker snapshot
     * is implemented; do not interpret it as an NVS readback certificate. */
    out->tcal_points=(uint8_t)arc_points.count;
    out->scale_mask=(uint8_t)((arc2_valid[0]?1:0)|
                             (arc2_valid[1]?2:0)|(arc2_valid[2]?4:0));
    out->earth_done=(uint8_t)arc2_contiguous(ARC2_KIND_EARTH);
    out->gsens_done=(uint8_t)arc2_contiguous(ARC2_KIND_GSENS);
    out->saved_flags=(uint8_t)((out->tcal_points?1:0)|
                             (out->scale_mask<<1)|
                             (out->earth_done?0x10:0)|
                             (out->gsens_done?0x20:0));
}

/* F05 Stage2: phase events are immutable; progress is a current snapshot.
 * Both can be used independently by PC: phase relates to the transaction,
 * progress describes persisted dataset status at dispatch time. */
bool cal_archive_status_take_event(cal_arc_status_t *out)
{
    if (!out) return false;
    arc_event_t ev;
    if (k_msgq_get(&arc_event_msgq, &ev, K_NO_WAIT) != 0) return false;
    cal_archive_status_snapshot(out);
    out->seq = ev.seq;
    out->flag = ev.flag;
    out->phase = ev.phase;
    out->error = ev.error;
    return true;
}

void cal_archive_status_replay(uint8_t seq, uint8_t flag)
{
    if (!seq) return;
    unsigned int key = irq_lock();
    for (unsigned i=0;i<2;i++) {
        if (arc_recent_valid[i] && arc_recent[i].seq==seq &&
            arc_recent[i].flag==flag) {
            cal_arc_status_t s=arc_recent[i];
            irq_unlock(key);
            arc_enqueue_event(seq,flag,s.phase,s.error);
            return;
        }
    }
    irq_unlock(key);
    /* status queries are exempt from the two-item dedup cache */
    if (flag==0x42) {
        arc_enqueue_event(seq,flag,ARC_PHASE_EXECUTED,ARC_ERROR_NONE);
    }
}

void cal_archive_status_submit_failed(uint8_t seq, uint8_t flag,
                                      cal_arc_submit_t admission)
{
    if (!seq || admission==ARC_SUBMIT_QUEUED || admission==ARC_SUBMIT_DUPLICATE)
        return;
    uint8_t error = admission==ARC_SUBMIT_BUSY ? ARC_ERROR_BUSY :
                    admission==ARC_SUBMIT_INVALID ? ARC_ERROR_WRONG_STATE :
                    ARC_ERROR_QUEUE;
    /* Don't replace the running transaction's arc_last_status. */
    arc_enqueue_event(seq, flag, ARC_PHASE_REJECTED, error);
}

bool cal_archive_wire_idle(void)
{
    return !atomic_get(&arc_cmd_busy)&&!atomic_get(&arc_save_pending)&&
        !atomic_get(&arc_collect_request)&&!atomic_get(&arc_capture_running)&&
        !arc2_busy()&&!arc2_save_pending();
}

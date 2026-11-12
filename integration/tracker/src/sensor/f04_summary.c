#if defined(CONFIG_ARC8_F04_SUMMARY)
#include "f04_summary.h"
#include "calwire.h"
#include "f04_summary_core.h"
#include "connection/connection.h"
#include "connection/esb.h"
#include "connection/tdma.h"
#include "system/arc8_esb_ota.h"
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <math.h>
LOG_MODULE_REGISTER(f04_summary, LOG_LEVEL_INF);
K_MUTEX_DEFINE(f04_lock);
K_MUTEX_DEFINE(f04_tx_lock);
K_MSGQ_DEFINE(f04_queue, F04_SIZE, 64, 4); /* 1.28 s of windows; not a full recording. */
static f04_accumulator acc;
static atomic_t active;
static uint32_t boot_nonce, operation, queued, queue_drop, submitted;
static uint32_t deferred;
static uint32_t next_log;
static bool anchored, stopping;
bool f04_summary_active(void) { return atomic_get(&active)!=0; }
bool f04_summary_begin(uint8_t tracker, uint8_t rate) {
    if (rate!=50 || tracker>=32 || connection_get_data_collection() || arc8_ota_active() || calwire_active()) {
        LOG_WRN("F04 start rejected: rate/identity or raw/OTA busy"); return false;
    }
    k_mutex_lock(&f04_lock,K_FOREVER);
    if (f04_summary_active()) { k_mutex_unlock(&f04_lock); return false; }
    if (!boot_nonce) { do { boot_nonce=sys_rand32_get(); } while (!boot_nonce); }
    if (++operation==0) { do { boot_nonce=sys_rand32_get(); } while (!boot_nonce); operation=1; }
    (void)f04_init(&acc,tracker,boot_nonce,operation,0);
    queued=queue_drop=submitted=deferred=0; next_log=50; anchored=stopping=false;
    k_msgq_purge(&f04_queue);
    atomic_set(&active,1);
    k_mutex_unlock(&f04_lock);
    LOG_INF("F04 START tracker=%u boot=%u op=%u rate=50 time=estimated raw=1",tracker,boot_nonce,operation);
    return true;
}
/* Caller owns f04_tx_lock. Producer only appends; retain head until accepted. */
static bool drain_one(void) {
    uint8_t frame[F04_SIZE];
    if (k_msgq_peek(&f04_queue,frame)) return false;
    if (esb_write_capture(frame,F04_SIZE)) { deferred++; return false; }
    (void)k_msgq_get(&f04_queue,frame,K_NO_WAIT);
    submitted++;
    return true;
}
void f04_summary_end(void) {
    k_mutex_lock(&f04_tx_lock,K_FOREVER);
    k_mutex_lock(&f04_lock,K_FOREVER);
    if (!f04_summary_active()) { k_mutex_unlock(&f04_lock); k_mutex_unlock(&f04_tx_lock); return; }
    stopping=true; /* Stop producer BEFORE draining; keep radio awake. */
    k_mutex_unlock(&f04_lock);
    int64_t deadline=k_uptime_get()+2000;
    while (k_msgq_num_used_get(&f04_queue) && k_uptime_get()<deadline) {
        (void)drain_one(); k_msleep(1);
    }
    k_mutex_lock(&f04_lock,K_FOREVER);
    uint32_t tail=k_msgq_num_used_get(&f04_queue);
    k_msgq_purge(&f04_queue);
    uint8_t end[F04_SIZE]={F04_TYPE,acc.tracker,F04_VERSION,F04_END_MANIFEST};
    f04_put32(end+4,boot_nonce); f04_put32(end+8,operation);
    f04_put32(end+12,acc.sequence); f04_put32(end+16,acc.ok); f04_put32(end+20,acc.bad);
    f04_put32(end+24,queue_drop); f04_put32(end+28,submitted); f04_put32(end+32,tail);
    f04_put32(end+36,deferred); f04_put32(end+40,acc.count);
    f04_put32(end+44,f04_crc(end,44));
    k_mutex_unlock(&f04_lock);
    /* Three copies in different slots. Missing all copies fails host quality;
     * the OFF command acknowledgement alone is never proof of completeness. */
    deadline=k_uptime_get()+1000;
    unsigned copies=0;
    while (copies<3 && k_uptime_get()<deadline) {
        if (!esb_write_capture(end,sizeof(end))) copies++;
        k_msleep(1);
    }
    k_mutex_lock(&f04_lock,K_FOREVER);
    atomic_clear(&active);
    LOG_INF("F04 STOP ok=%u bad=%u queued=%u queue_drop=%u submitted=%u tail_drop=%u partial=%u",
            acc.ok,acc.bad,queued,queue_drop,submitted,tail,acc.count);
    k_mutex_unlock(&f04_lock);
    k_mutex_unlock(&f04_tx_lock);
}
void f04_summary_invalidate(void) {
    if (!f04_summary_active()) return;
    k_mutex_lock(&f04_lock,K_FOREVER);
    if (f04_summary_active() && !stopping) {
        f04_discontinuity(&acc,(uint32_t)((uint64_t)k_uptime_get()*1000));
        anchored=false;
    }
    k_mutex_unlock(&f04_lock);
}
void f04_summary_feed(const float gyro[3], const float accel[3], float temp,
                       float dt_s, bool available, bool fresh) {
    if (!f04_summary_active()) return;
    if (!isfinite(dt_s) || dt_s<0.000625f || dt_s>0.010001f) {
        f04_summary_invalidate(); return;
    }
    k_mutex_lock(&f04_lock,K_FOREVER); /* Never silently skip a sample on contention. */
    if (f04_summary_active() && !stopping) {
        if (!anchored) {
            /* Receiver widens slots after batch START. Do not overflow the
             * queue while waiting for NORMAL PONG to deliver that layout. */
            if (!tdma_capture_ready()) { k_mutex_unlock(&f04_lock); return; }
            acc.anchor_us=(uint32_t)((uint64_t)k_uptime_get()*1000); anchored=true;
        }
        uint8_t frame[F04_SIZE];
        if (f04_add(&acc,gyro,accel,temp,dt_s,available,fresh,frame)==1) {
            if (k_msgq_put(&f04_queue,frame,K_NO_WAIT)==0) queued++; else queue_drop++;
        }
    }
    k_mutex_unlock(&f04_lock);
}
void f04_summary_drain(void) {
    if (!f04_summary_active() || !esb_ready() || arc8_ota_active()) return;
    if (k_mutex_lock(&f04_tx_lock,K_NO_WAIT)) return;
    if (!f04_summary_active() || stopping) { k_mutex_unlock(&f04_tx_lock); return; }
    for (unsigned i=0;i<2;i++) {
        if (!drain_one()) break;
    }
    k_mutex_lock(&f04_lock,K_FOREVER);
    if (acc.sequence>=next_log) {
        LOG_INF("F04 windows ok=%u bad=%u queued=%u queue_drop=%u submitted=%u deferred=%u",
                acc.ok,acc.bad,queued,queue_drop,submitted,deferred);
        next_log=acc.sequence+50;
    }
    k_mutex_unlock(&f04_lock);
    k_mutex_unlock(&f04_tx_lock);
}
#endif

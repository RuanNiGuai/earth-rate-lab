/* Host-tested core adapter for H3's native FIFO and PONG command paths.
 * This is intentionally NOT a wireless Joint10 implementation.
 * No persistent writes; no calibration changes; no OTA changes.
 */
#if defined(CONFIG_ARC8_JOINT10_EXPERIMENTAL)
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>
#include "arc8_joint10_adapter.h"
#include "arc8_joint_core.h"

LOG_MODULE_REGISTER(arc8_joint10_exp, LOG_LEVEL_INF);
K_MUTEX_DEFINE(joint10_mutex);
static arc8_joint_device joint10;
static uint32_t joint10_op_id = 0;
static uint32_t joint10_boot_token = 0;
static uint32_t joint10_sample_seq;
static uint64_t joint10_anchor_us;
static uint32_t joint10_reported_windows;
static uint32_t joint10_invalid_input;
static uint32_t joint10_accel_fresh, joint10_accel_reused, joint10_accel_missing;
static uint32_t joint10_numeric_invalid;

bool arc8_joint10_begin(uint8_t tracker_id, uint8_t rate_hz)
{
    if (rate_hz != 50u) {
        LOG_WRN("Joint10 EXP rejected rate %u; only 50 Hz supported", rate_hz);
        return false;
    }
    k_mutex_lock(&joint10_mutex, K_FOREVER);
    if (joint10.state == ARC8_JOINT_RUNNING) {
        k_mutex_unlock(&joint10_mutex);
        LOG_WRN("Joint10 EXP already running; start ignored");
        return false;
    }
    /* Experimental token, NOT a persistent or authenticated boot identity. */
    if (!joint10_boot_token) {
        joint10_boot_token = k_cycle_get_32() ^ 0x8A10C00Du;
        if (!joint10_boot_token) joint10_boot_token = 1u;
    }
    if (++joint10_op_id == 0u) joint10_op_id = 1u;
    uint8_t pseudo_uid[8] = {0};
    pseudo_uid[0] = tracker_id; /* placeholder, not a hardware UID */
    arc8_joint_result st = arc8_joint_arm(&joint10, pseudo_uid, tracker_id,
                                         joint10_boot_token, joint10_op_id, 400u);
    if (st == ARC8_JOINT_OK)
        st = arc8_joint_start(&joint10, joint10_boot_token, joint10_op_id);
    joint10_sample_seq = 0u;
    joint10_reported_windows = 0u;
    joint10_invalid_input = 0u;
    joint10_accel_fresh = joint10_accel_reused = joint10_accel_missing = 0u;
    joint10_numeric_invalid = 0u;
    /* The timestamp is synthetic and has estimated quality, not synchronized. */
    joint10_anchor_us = (uint64_t)k_uptime_get() * 1000ULL;
    k_mutex_unlock(&joint10_mutex);
    if (st != ARC8_JOINT_OK) {
        LOG_ERR("Joint10 EXP start failed (%d)", (int)st);
        return false;
    }
    LOG_INF("Joint10 EXP START tracker=%u op=%u token=%u 400->50Hz ESTIMATED_TIME no_ESB",
            tracker_id, joint10_op_id, joint10_boot_token);
    return true;
}

void arc8_joint10_end(void)
{
    k_mutex_lock(&joint10_mutex, K_FOREVER);
    if (joint10.state == ARC8_JOINT_RUNNING || joint10.state == ARC8_JOINT_ARMED)
        (void)arc8_joint_stop(&joint10, joint10_boot_token, joint10_op_id);
    uint32_t ok = joint10.windows_ok, bad = joint10.windows_bad;
    uint32_t gaps = joint10.gaps_total, invalid = joint10_invalid_input;
    k_mutex_unlock(&joint10_mutex);
    LOG_INF("Joint10 EXP STOP windows_ok=%u bad=%u gaps=%u invalid=%u (no_ESB)",
            ok, bad, gaps, invalid);
    LOG_INF("ARC8 J10 B11 STOP accel fresh=%u reused=%u missing=%u numeric=%u timing=%u sensorflag=%u",
            joint10_accel_fresh, joint10_accel_reused, joint10_accel_missing,
            joint10_numeric_invalid, joint10.invalid_timing_total,
            joint10.invalid_sensor_flag_total);
    LOG_INF("ARC8 J10 B11 REASONS sample time_bad=%u saturation=%u fifo_loss=%u accel_missing=%u gap=%u",
            joint10.reason_time_bad, joint10.reason_saturation,
            joint10.reason_fifo_loss, joint10.reason_accel_missing, joint10.reason_gap);
    LOG_INF("ARC8 J10 B11 REJECT windows time_bad=%u saturation=%u fifo_loss=%u accel_missing=%u gap=%u partial=%u",
            joint10.bad_window_time, joint10.bad_window_saturation,
            joint10.bad_window_fifo_loss, joint10.bad_window_accel_missing,
            joint10.bad_window_gap, joint10.bad_window_partial);
}

static int32_t convert_fixed(float x, float factor, int32_t limit, bool *valid)
{
    if (!isfinite(x) || fabsf(x) > (float)limit / factor) {
        *valid = false;
        return 0;
    }
    return (int32_t)lroundf(x * factor);
}

void arc8_joint10_feed_raw_ex(const float gyro_dps[3], const float accel_g[3],
                               float temp_c, bool accel_available, bool accel_fresh)
{
    if (!gyro_dps || !accel_g) return;
    /* Called by the sensor owner thread, before the in-place gyro/accel
     * calibration. One FIFO gyro tag produces one sample.
     */
    if (k_mutex_lock(&joint10_mutex, K_NO_WAIT) != 0) return;
    if (joint10.state != ARC8_JOINT_RUNNING) {
        k_mutex_unlock(&joint10_mutex);
        return;
    }
    arc8_joint_sample sample = {0};
    bool valid = true;
    sample.seq = joint10_sample_seq;
    sample.sample_time_us = joint10_anchor_us + ((uint64_t)joint10_sample_seq * 2500ULL);
    sample.time_quality = ARC8_JOINT_TIME_ESTIMATED;
    for (int i=0; i<3; ++i) {
        sample.gyro_mdps[i] = convert_fixed(gyro_dps[i], 1000.0f, 2500000, &valid);
        sample.accel_mg[i] = convert_fixed(accel_g[i], 1000.0f, 200000, &valid);
    }
    sample.temperature_centi_c = (int16_t)convert_fixed(temp_c,100.0f, 30000,&valid);
    if (accel_available) {
        if (accel_fresh) joint10_accel_fresh++;
        else joint10_accel_reused++;
    } else {
        joint10_accel_missing++;
    }
    if (!valid) joint10_numeric_invalid++;
    sample.accel_quality = !accel_available ? ARC8_JOINT_ACCEL_MISSING :
                           accel_fresh ? ARC8_JOINT_ACCEL_FRESH : ARC8_JOINT_ACCEL_HELD;
    /* Age-expired accel is NOT evidence of a lost FIFO packet. */
    if (!accel_available) sample.flags |= ARC8_JOINT_FLAG_ACCEL_MISSING;
    if (!valid) sample.flags |= ARC8_JOINT_FLAG_FIFO_LOSS;
    if (!valid || !accel_available) joint10_invalid_input++;
    arc8_joint_summary sum = {0};
    arc8_joint_result res = arc8_joint_add(&joint10, joint10_boot_token,
                                            joint10_op_id, &sample, &sum);
    /* A window's accel_sum_mg is sum over ALL 8 gyro times (includes holds).
     * accel_fresh_sum_mg is sum over only actual FIFO accel tags;
     * fresh/held/missing counts must accompany downstream averages. */
    joint10_sample_seq++;
    if (res == ARC8_JOINT_WINDOW_READY || res == ARC8_JOINT_WINDOW_REJECTED) {
        joint10_reported_windows++;
        /* Once per second, not on each sample; no high-rate console output. */
        if (joint10_reported_windows % 50u == 0u) {
            LOG_INF("Joint10 EXP 1s windows ok=%u bad=%u gaps=%u last_seq=%u quality=estimated",
                    joint10.windows_ok, joint10.windows_bad,
                    joint10.gaps_total, sum.last_seq);
            LOG_INF("ARC8 J10 B11 DIAG accel fresh=%u reused=%u missing=%u numeric=%u timing=%u sensorflag=%u",
                    joint10_accel_fresh, joint10_accel_reused,
                    joint10_accel_missing, joint10_numeric_invalid,
                    joint10.invalid_timing_total,
                    joint10.invalid_sensor_flag_total);
            LOG_INF("ARC8 J10 B11 REASONS time_bad=%u sat=%u fifo_loss=%u accel_missing=%u gap=%u",
                    joint10.reason_time_bad, joint10.reason_saturation,
                    joint10.reason_fifo_loss, joint10.reason_accel_missing,
                    joint10.reason_gap);
            LOG_INF("ARC8 J10 B11 LAST_WINDOW accel_fresh=%u held=%u missing=%u (sum_mg includes holds)",
                    sum.accel_fresh_samples, sum.accel_held_samples,
                    sum.accel_missing_samples);
        }
    }
    k_mutex_unlock(&joint10_mutex);
}

void arc8_joint10_feed_raw(const float gyro_dps[3], const float accel_g[3], float temp_c)
{
    /* Compatibility with B08 callers: zero accel means no available tag.
     * Production FIFO path must use the explicit _ex() API. */
    bool available = accel_g &&
        (accel_g[0] != 0.0f || accel_g[1] != 0.0f || accel_g[2] != 0.0f);
    arc8_joint10_feed_raw_ex(gyro_dps, accel_g, temp_c, available, available);
}

bool arc8_joint10_snapshot(uint32_t *ok, uint32_t *bad,
                            uint32_t *gaps, uint32_t *invalid)
{
    if (!ok || !bad || !gaps || !invalid) return false;
    k_mutex_lock(&joint10_mutex, K_FOREVER);
    *ok = joint10.windows_ok;
    *bad = joint10.windows_bad;
    *gaps = joint10.gaps_total;
    *invalid = joint10_invalid_input;
    k_mutex_unlock(&joint10_mutex);
    return true;
}

#endif /* CONFIG_ARC8_JOINT10_EXPERIMENTAL */

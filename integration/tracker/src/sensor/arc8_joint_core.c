#include "arc8_joint_core.h"
#include <string.h>

static void new_window(arc8_joint_device *d) {
    memset(&d->partial, 0, sizeof(d->partial));
    d->partial.operation_id=d->operation_id;
    d->partial.boot_id=d->boot_id;
    d->partial.tracker_id=d->tracker_id;
    d->partial.expected_samples=ARC8_JOINT_WINDOW_SAMPLES;
}
arc8_joint_result arc8_joint_arm(arc8_joint_device *d, const uint8_t uid[8],
                                  uint8_t tracker_id, uint32_t boot_id,
                                  uint32_t operation_id, uint32_t odr_hz) {
    if (!d || !uid || !boot_id || !operation_id || tracker_id >= 64 ||
        odr_hz == 0 || odr_hz > 1600 || (1000000u % odr_hz)) return ARC8_JOINT_INVALID;
    if (d->state == ARC8_JOINT_RUNNING) return ARC8_JOINT_BAD_STATE;
    memset(d, 0, sizeof(*d));
    memcpy(d->uid, uid, 8);
    d->tracker_id=tracker_id; d->boot_id=boot_id;
    d->operation_id=operation_id; d->odr_hz=odr_hz;
    d->nominal_dt_us=1000000u/odr_hz;
    d->state=ARC8_JOINT_ARMED;
    new_window(d);
    return ARC8_JOINT_OK;
}
arc8_joint_result arc8_joint_start(arc8_joint_device *d, uint32_t boot_id,
                                    uint32_t operation_id) {
    if (!d) return ARC8_JOINT_INVALID;
    if (d->boot_id != boot_id || d->operation_id != operation_id)
        return ARC8_JOINT_WRONG_SESSION;
    if (d->state != ARC8_JOINT_ARMED) return ARC8_JOINT_BAD_STATE;
    d->state=ARC8_JOINT_RUNNING;
    return ARC8_JOINT_OK;
}
arc8_joint_result arc8_joint_add(arc8_joint_device *d, uint32_t boot_id,
                                  uint32_t operation_id, const arc8_joint_sample *s,
                                  arc8_joint_summary *out) {
    if (!d || !s || !out) return ARC8_JOINT_INVALID;
    if (d->boot_id != boot_id || d->operation_id != operation_id)
        return ARC8_JOINT_WRONG_SESSION;
    if (d->state != ARC8_JOINT_RUNNING) return ARC8_JOINT_BAD_STATE;
    uint32_t gap=0;
    uint64_t dt=d->nominal_dt_us;
    bool bad_timing=false, bad_flags=false;
    if (d->have_prev) {
        uint32_t diff=s->seq-d->prev_seq;
        if (diff==0 || diff>65536) return ARC8_JOINT_INVALID;
        gap=diff-1;
        if (s->sample_time_us <= d->prev_timestamp_us) bad_timing=true;
        else dt=s->sample_time_us-d->prev_timestamp_us;
        /* A batch read timestamp may not have per-sample accuracy. */
        if (dt < d->nominal_dt_us/2 || dt > d->nominal_dt_us*3/2) bad_timing=true;
    }
    if (!(s->time_quality & (ARC8_JOINT_TIME_ESTIMATED|ARC8_JOINT_TIME_HW_VALID))) bad_timing=true;
    if (s->flags & (ARC8_JOINT_FLAG_SATURATION|ARC8_JOINT_FLAG_FIFO_LOSS|
                    ARC8_JOINT_FLAG_ACCEL_MISSING)) bad_flags=true;
    bool bad = bad_timing || bad_flags;
    if (bad_timing) { d->invalid_timing_total++; d->reason_time_bad++; }
    if (s->flags & ARC8_JOINT_FLAG_SATURATION) d->reason_saturation++;
    if (s->flags & ARC8_JOINT_FLAG_FIFO_LOSS) d->reason_fifo_loss++;
    if (s->flags & ARC8_JOINT_FLAG_ACCEL_MISSING) d->reason_accel_missing++;
    if (gap) d->reason_gap+=gap;
    if (bad_flags) d->invalid_sensor_flag_total++;
    if (gap) d->gaps_total+=gap;
    if (bad) d->invalid_time_total++;
    d->prev_seq=s->seq;
    d->prev_timestamp_us=s->sample_time_us;
    d->have_prev=true;
    arc8_joint_summary *p=&d->partial;
    if (!p->received_samples) {p->first_seq=s->seq; p->first_time_us=s->sample_time_us;}
    p->last_seq=s->seq; p->last_time_us=s->sample_time_us;
    p->received_samples++;
    p->observed_gap_samples+=gap;
    p->time_quality |=s->time_quality;
    p->flags |=s->flags;
    if (bad_timing) p->timing_bad_samples++;
    switch (s->accel_quality) {
    case ARC8_JOINT_ACCEL_FRESH: p->accel_fresh_samples++; break;
    case ARC8_JOINT_ACCEL_HELD: p->accel_held_samples++; break;
    case ARC8_JOINT_ACCEL_MISSING: p->accel_missing_samples++; break;
    default: break; /* Existing clients with unspecified provenance. */
    }
    for (int a=0;a<3;a++) {
        p->gyro_delta_udeg[a]+=((int64_t)s->gyro_mdps[a] * (int64_t)dt)/1000;
        p->accel_sum_mg[a]+=s->accel_mg[a];
        if (s->accel_quality == ARC8_JOINT_ACCEL_FRESH)
            p->accel_fresh_sum_mg[a]+=s->accel_mg[a];
    }
    p->temp_sum_centi_c+=s->temperature_centi_c;
    if (p->received_samples != ARC8_JOINT_WINDOW_SAMPLES)
        return ARC8_JOINT_OK;
    *out=*p;
    arc8_joint_result result=(p->flags || p->observed_gap_samples || p->timing_bad_samples) ?
        ARC8_JOINT_WINDOW_REJECTED : ARC8_JOINT_WINDOW_READY;
    if (result==ARC8_JOINT_WINDOW_READY) d->windows_ok++;
    else {
        d->windows_bad++;
        if (p->timing_bad_samples) d->bad_window_time++;
        if (p->flags & ARC8_JOINT_FLAG_SATURATION) d->bad_window_saturation++;
        if (p->flags & ARC8_JOINT_FLAG_FIFO_LOSS) d->bad_window_fifo_loss++;
        if (p->flags & ARC8_JOINT_FLAG_ACCEL_MISSING) d->bad_window_accel_missing++;
        if (p->observed_gap_samples) d->bad_window_gap++;
    }
    new_window(d);
    return result;
}
arc8_joint_result arc8_joint_stop(arc8_joint_device *d, uint32_t boot_id,
                                   uint32_t operation_id) {
    if (!d) return ARC8_JOINT_INVALID;
    if (d->boot_id!=boot_id || d->operation_id!=operation_id)
        return ARC8_JOINT_WRONG_SESSION;
    if (d->state != ARC8_JOINT_RUNNING && d->state != ARC8_JOINT_ARMED)
        return ARC8_JOINT_BAD_STATE;
    if (d->partial.received_samples) {
        d->windows_bad++;
        d->bad_window_partial++;
    }
    d->state=ARC8_JOINT_STOPPED;
    return ARC8_JOINT_OK;
}

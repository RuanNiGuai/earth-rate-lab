/* ARC8-B11: Joint10 gyro/accel FIFO tag pairing (no Zephyr dependencies).
 * Preserves original raw USB/Data Collect pairing independently.
 */
#ifndef ARC8_JOINT10_ACCEL_PAIR_H
#define ARC8_JOINT10_ACCEL_PAIR_H
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* 400 Hz gyro / ~100 Hz accel: allow one fresh + seven reused gyro tags.
 * Without another accel tag, the 9th gyro sample is marked missing.
 * This is sample-age gating, NOT timestamp synchronization.
 */
#define ARC8_JOINT10_ACCEL_MAX_GYRO_AGE 8u

typedef struct {
    float last[3];
    uint8_t gyro_age;
    bool have_accel;
} arc8_joint10_accel_pair;

static inline bool arc8_joint10_accel_observe(arc8_joint10_accel_pair *p,
                                              const float accel[3]) {
    if (!p || !accel) return false;
    for (int i=0;i<3;i++) {
        if (!isfinite(accel[i])) { p->have_accel=false; return false; }
    }
    /* fifo_process emits all-zero for 'no accel tag', not an explicit
     * validity bit. Do not treat that sentinel as a fresh measurement. */
    if (accel[0]==0 && accel[1]==0 && accel[2]==0) return false;
    for (int i=0;i<3;i++) p->last[i]=accel[i];
    p->have_accel=true;
    p->gyro_age=0;
    return true;
}

static inline bool arc8_joint10_accel_for_gyro(arc8_joint10_accel_pair *p,
                                                float out[3], bool *fresh) {
    if (!p || !out || !fresh) return false;
    *fresh=false;
    if (!p->have_accel || p->gyro_age >= ARC8_JOINT10_ACCEL_MAX_GYRO_AGE) {
        out[0]=out[1]=out[2]=0;
        return false;
    }
    *fresh=(p->gyro_age == 0);
    for (int i=0;i<3;i++) out[i]=p->last[i];
    p->gyro_age++;
    return true;
}
#endif

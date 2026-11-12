/* SPDX-License-Identifier: MIT */
#ifndef SENSOR_TEMPERATURE_GUARD_H
#define SENSOR_TEMPERATURE_GUARD_H
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* Admission is per temperature acquisition, never per gyro sample in a FIFO.
 * A plausible step needs repeated evidence; hardware sentinels must already
 * have been converted to NAN by the driver. These are engineering limits. */
#define TEMP_GUARD_STEP_C 1.0f
#define TEMP_GUARD_AGREE_C 0.2f
#define TEMP_GUARD_CONFIRM_COUNT 3
#define TEMP_GUARD_CONFIRM_MS 100
#define TEMP_GUARD_MAX_AGE_MS 1000
#define TEMP_GUARD_FILTER_MS 500.0f

typedef struct {
    bool observed, initialized, current_valid;
    int64_t observation_ms, accepted_ms, pending_ms;
    float accepted, filtered, pending_min, pending_max;
    unsigned pending_count;
} temperature_guard_t;

static inline bool temperature_guard_update(temperature_guard_t *s, float x, int64_t now)
{
    s->current_valid = false;
    bool discontinuity = s->observed &&
        (now <= s->observation_ms || now - s->observation_ms > TEMP_GUARD_MAX_AGE_MS);
    bool ordered = !s->observed || now > s->observation_ms;
    s->observed = true;
    s->observation_ms = now;
    if (discontinuity) s->pending_count = 0;
    if (!ordered || now < 0 || !isfinite(x) || x <= -20.0f || x >= 60.0f) {
        s->pending_count = 0;
        return false;
    }
    bool fresh = s->initialized && now > s->accepted_ms &&
        now - s->accepted_ms <= TEMP_GUARD_MAX_AGE_MS;
    bool step = !fresh || fabsf(x - s->accepted) > TEMP_GUARD_STEP_C;
    if (step) {
        float lo = fminf(x, s->pending_min), hi = fmaxf(x, s->pending_max);
        if (!s->pending_count || hi - lo > TEMP_GUARD_AGREE_C) {
            s->pending_min = s->pending_max = x;
            s->pending_ms = now;
            s->pending_count = 1;
            return false;
        }
        s->pending_min = lo;
        s->pending_max = hi;
        if (s->pending_count < TEMP_GUARD_CONFIRM_COUNT) s->pending_count++;
        if (s->pending_count < TEMP_GUARD_CONFIRM_COUNT ||
            now - s->pending_ms < TEMP_GUARD_CONFIRM_MS) return false;
        /* Confirmed steps/restarts re-seed the LP: do not label a new bias
         * window with the old temperature while the LP catches up. */
        s->filtered = x;
    } else {
        float dt = (float)(now - s->accepted_ms);
        s->filtered += dt / (TEMP_GUARD_FILTER_MS + dt) * (x - s->filtered);
    }
    s->pending_count = 0;
    s->accepted = x;
    s->accepted_ms = now;
    s->initialized = s->current_valid = true;
    return true;
}

static inline float temperature_guard_compensation(const temperature_guard_t *s, int64_t now)
{
    return s->initialized && now >= s->accepted_ms &&
        now - s->accepted_ms <= TEMP_GUARD_MAX_AGE_MS ? s->filtered : NAN;
}

static inline float temperature_guard_collection(const temperature_guard_t *s, int64_t now)
{
    /* Only the accepted raw acquisition belongs to the current FIFO batch.
     * A held compensation value is never fresh calibration evidence. */
    return s->current_valid && isfinite(temperature_guard_compensation(s, now)) ? s->accepted : NAN;
}
#endif

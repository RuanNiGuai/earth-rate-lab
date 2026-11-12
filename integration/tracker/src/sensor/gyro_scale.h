#ifndef TRACKER_GYRO_SCALE_H
#define TRACKER_GYRO_SCALE_H
#include <stdbool.h>
#include <stdint.h>

/* Input is full-rate, temperature-corrected gyro BEFORE sensitivity scaling.
 * The operator supplies an independent mechanical whole-turn reference. */
enum gyro_scale_phase { GYRO_SCALE_IDLE, GYRO_SCALE_PREPARE, GYRO_SCALE_RUN,
                        GYRO_SCALE_DONE, GYRO_SCALE_FAILED };
typedef struct {
    enum gyro_scale_phase phase;
    const char *error;
    int axis, turns;
    uint32_t seed_samples;
    double seed_sum[3], seed_square[3], angle[3], absolute_angle, cross_angle;
    float bias[3], initial_scale[3], result_scale[3], start_accel[3];
    float elapsed, seed_time, end_rest_time, sample_dt, start_temp;
    int direction;
    double reverse_angle;
} gyro_scale_t;

bool gyro_scale_begin(gyro_scale_t *s, int turns, int axis, const float scales[3]);
void gyro_scale_sample(gyro_scale_t *s, const float gyro[3], const float accel[3],
                       float temp, float dt, bool rest, bool valid);
bool gyro_scale_finish(gyro_scale_t *s);
void gyro_scale_fail(gyro_scale_t *s, const char *reason);
#endif

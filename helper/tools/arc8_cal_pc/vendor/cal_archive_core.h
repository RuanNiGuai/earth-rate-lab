#ifndef SLIMEVR_CAL_ARCHIVE_CORE_H
#define SLIMEVR_CAL_ARCHIVE_CORE_H
#include <stdint.h>
#include <stdbool.h>
#define CAL_ARC_VERSION 2u
#define CAL_ARC_MAGIC 0x43414C31u
#define CAL_ARC_TMAX 12
#define CAL_ARC_SCALE 3
#define CAL_ARC_EARTH 5
#define CAL_ARC_GSENS 6
#define CAL_ARC_MSEG 12
/* No raw 400-Hz stream in NVS: chronologically ordered, lossy gyro/accel/time integrals. */
typedef struct {
    float raw_delta[3], accel_dt[3], temp_dt, seconds;
} cal_arc_motion_seg_t;
typedef struct {
    uint16_t samples;
    uint8_t count, merges;
    float lost_cross_deg; /* conservative second-order angle discarded by merging */
    cal_arc_motion_seg_t seg[CAL_ARC_MSEG];
} cal_arc_motion_t;
/* Call once per valid gyro FIFO sample while changing pose. */
void cal_arc_motion_add(cal_arc_motion_t *m, const float raw_dps[3],
                        const float accel_g[3], float temp, float dt);
/* Recomputes body-to-reference orientation from raw motion and final calibration. */
bool cal_arc_motion_replay(const cal_arc_motion_t *m, const float initial_q[4],
                           const float scale[3], const float gsens[9],
                           const float earth_ref[3], const float bias_dps[3],
                           float out_q[4]);
/* Raw pre-TCal/pre-gsens/pre-scale gyro in dps; acceleration in g. */
typedef struct {
    float temp, gyro[3], accel[3], quat[4], seconds, angle_deg;
    uint32_t samples;
    float gyro_sem[3];
} cal_arc_point_t;
typedef struct {
    uint32_t magic, version, generation, reference_id;
    uint8_t count, reserved[3];
    cal_arc_point_t point[CAL_ARC_TMAX];
    uint32_t checksum;
} cal_arc_tcal_t;
typedef struct {
    float initial_raw[3], initial_accel[3], initial_temp;
    float raw_angle[3], accel_time[3], temp_time, duration;
    float cross_angle, quality;
    float temp_min, temp_max, temp_moments[3]; /* raw temperature-time statistics */
    uint8_t axis, turns, reserved[2];
} cal_arc_scale_axis_t;
typedef struct {
    uint32_t magic, version, generation, reference_id;
    uint8_t count, reserved[3];
    cal_arc_scale_axis_t axis[CAL_ARC_SCALE];
    uint32_t checksum;
} cal_arc_scale_t;
typedef struct {
    uint32_t magic, version, generation, reference_id;
    uint8_t count, up_axis, reserved[2];
    float latitude_deg;
    cal_arc_point_t stop[CAL_ARC_EARTH];
    cal_arc_motion_t travel[CAL_ARC_EARTH-1];
    uint32_t checksum;
} cal_arc_earth_t;
typedef struct {
    uint32_t magic, version, generation, reference_id;
    uint8_t count, reserved[3];
    cal_arc_point_t pose[CAL_ARC_GSENS];
    cal_arc_motion_t travel[CAL_ARC_GSENS-1];
    uint32_t checksum;
} cal_arc_gsens_t;
typedef struct {
    float scale[3], earth_body[3], gsens[9], base_bias[3];
    float earth_rms, gsens_rms, scale_error[3];
    uint8_t iterations, ready;
} cal_arc_solution_t;
/* Correct inclination from stationary accelerometer without altering the
 * relative yaw supplied by gyro propagation. Returns false for gross errors. */
bool cal_arc_gravity_lock(float q[4], const float accel[3],
                          const float reference_up[3], float *mismatch_deg);
uint32_t cal_arc_checksum(const void *data, unsigned bytes);
/* The archive records can be solved solely on the tracker, with bounded stack/heap. */
bool cal_arc_solve(const cal_arc_tcal_t *t, const cal_arc_scale_t *s,
                   const cal_arc_earth_t *e, const cal_arc_gsens_t *g,
                   cal_arc_solution_t *out);
#endif

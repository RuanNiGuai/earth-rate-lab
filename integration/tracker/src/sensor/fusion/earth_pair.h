#ifndef EARTH_PAIR_H
#define EARTH_PAIR_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <vqf.h>

typedef enum {
    PAIR_NONE=0,
    PAIR_EARTH=1,
    PAIR_REST_BIAS=2,
} earth_pair_mode_t;

/* Independent full-rate candidate. Baseline state and persistent calibration
 * are never modified. Earth mode's world heading must belong to this boot. */
typedef struct {
    vqf_params_t params;
    vqf_state_t state;
    vqf_coeffs_t coeffs;
    float earth_world[3];
    earth_pair_mode_t mode;
    bool enabled;
} earth_pair_t;

bool earth_pair_start(earth_pair_t *pair, const vqf_params_t *params,
    vqf_state_t *baseline, const vqf_coeffs_t *coeffs,
    const float earth_world_rad[3]);
bool earth_pair_start_rest_bias(earth_pair_t *pair, const vqf_params_t *params,
    vqf_state_t *baseline, const vqf_coeffs_t *coeffs);
void earth_pair_gyro(earth_pair_t *pair, const float gyro_rad[3]);
void earth_pair_accel(earth_pair_t *pair, const float accel_ms2[3]);
void earth_pair_accel_gated(earth_pair_t *pair, const float accel_ms2[3], bool trusted);
void earth_pair_project(const float q[4], const float world[3], float body[3]);

/* RAM-only WoM snapshot, including the already-translated bias/filter histories. */
bool earth_pair_save(const earth_pair_t *pair, bool auto_pending, bool motion_gate,
    uint32_t samples, uint8_t accel_gate_mode, void *storage, size_t capacity);
bool earth_pair_load(earth_pair_t *pair, bool *auto_pending, bool *motion_gate,
    uint32_t *samples, uint8_t *accel_gate_mode, const void *storage, size_t capacity);
#define EARTH_PAIR_SNAPSHOT_CAPACITY 656
typedef struct {
    bool valid;
    uint8_t horizontal_axis;
    float quat[4];
    float up_body[3];
} north_yaw_ref_t;
/* Reference remains RAM-only and belongs to one Earth Pair initialization.
 * Store while in a reproducible physical heading, then revisit that heading.
 */
bool north_yaw_capture(north_yaw_ref_t *ref, const earth_pair_t *pair);
bool north_yaw_delta(const north_yaw_ref_t *ref, const earth_pair_t *pair,
                     float *delta_rad);

/* Soft heading realignment of the running Earth VQF candidate. This does
 * NOT overwrite the original fixed-pose Earth calibration. Owner thread only.
 * delta is world-Z yaw in radians; the implementation refuses >45 degrees.
 */
bool earth_pair_realign_yaw(earth_pair_t *pair, float delta_yaw_rad);
#endif


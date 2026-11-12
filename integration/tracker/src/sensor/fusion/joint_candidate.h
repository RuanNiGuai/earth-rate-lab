#ifndef JOINT_CANDIDATE_H
#define JOINT_CANDIDATE_H

#include <stdbool.h>
#include <stdint.h>
#include <vqf.h>

/* Portable experimental core only. No sensor, console, persistence or output
 * routing is installed by this module. Callers continue running the baseline
 * independently and use it whenever enabled is false. All calls belong to one
 * sensor thread; stage console requests outside this core.
 *
 * Supported domain: raw sensor-axis gyro in degrees/second, with identity
 * sensitivity scaling. The caller must verify the actual calibration/config
 * before filling sensitivity_scale and must invalidate config_id on changes.
 * Body b0/slope use those SAME raw sensor axes. world_vqf_dps is already mapped
 * from the independently referenced world into this boot's INTERNAL VQF world;
 * it is not ENU and must not use the device-output q3 rotation.
 */
#define JOINT_CANDIDATE_SCHEMA 1u
#define JOINT_CANDIDATE_DOMAIN_RAW_IMU_DPS 1u

typedef enum {
    JOINT_CANDIDATE_OFF = 0,
    JOINT_CANDIDATE_ACTIVE,
    JOINT_CANDIDATE_INVALID_MODEL,
    JOINT_CANDIDATE_INVALID_REFERENCE,
    JOINT_CANDIDATE_INVALID_TEMPERATURE,
    JOINT_CANDIDATE_STALE_TEMPERATURE,
    JOINT_CANDIDATE_TEMPERATURE_OUT_OF_RANGE,
    JOINT_CANDIDATE_CONTEXT_CHANGED,
    JOINT_CANDIDATE_INVALID_SAMPLE,
    JOINT_CANDIDATE_NUMERICAL_FAILURE
} joint_candidate_status_t;

typedef struct {
    uint32_t schema;
    uint32_t model_id;
    uint32_t config_id;
    uint32_t input_domain;
    float sensitivity_scale[3]; /* Must be 1 on every axis in this version. */
    float reference_temperature_c;
    float minimum_temperature_c;
    float maximum_temperature_c;
    float body_bias_dps[3];
    float body_slope_dps_per_c[3];
    float world_vqf_dps[3];
    uint64_t maximum_temperature_age_us;
} joint_candidate_model_t;

typedef struct {
    uint32_t boot_generation; /* Must change on boot AND fusion frame reset. */
    uint32_t config_id;
    uint64_t now_uptime_us; /* FIFO batch completion time is acceptable here. */
    uint64_t temperature_uptime_us;
    float temperature_c; /* Agreed filtering/definition from model validation. */
    bool temperature_valid; /* Explicit; default/startup 25 C is not valid. */
} joint_candidate_context_t;

typedef struct {
    vqf_params_t params;
    vqf_state_t state;
    vqf_coeffs_t coeffs;
    joint_candidate_model_t model;
    uint32_t boot_generation;
    uint64_t last_uptime_us;
    uint64_t gyro_samples;
    uint64_t accel_samples;
    float last_temperature_c;
    float last_body_correction_dps[3];
    float last_world_body_dps[3];
    float last_corrected_gyro_dps[3];
    joint_candidate_status_t status;
    bool enabled;
} joint_candidate_t;

/* Initializes fresh VQF filters and zero residual bias. Only baseline's 6D
 * attitude is copied (into gyrQuat; accQuat remains identity). No IIR state or
 * covariance is translated/copied, and baseline/params/model are not mutated.
 * Both motion AND rest bias learning are disabled, including apparent rest
 * during slow motion. restDetected is consequently NOT a candidate rest test.
 * The caller must enforce mechanical rest at start and allow a warm-up before
 * measuring a new baseline; start does not certify calibration accuracy.
 * Sampling periods are fixed-rate seconds, NOT host USB arrival intervals.
 * No retained-state loading/saving is supported for this experimental branch.
 */
bool joint_candidate_start(joint_candidate_t *candidate,
    const joint_candidate_model_t *model,
    const vqf_params_t *baseline_params, const vqf_state_t *baseline,
    float gyro_period_s, float accel_period_s, float mag_period_s,
    const joint_candidate_context_t *context);

/* Sensor-thread updates: raw gyro in deg/s, calibrated accel in m/s^2. Model
 * parameters never learn; current T and current candidate attitude are used
 * on each gyro sample. Invalid input/temperature/context disables the branch
 * immediately. The caller must then expose the independent baseline, not the
 * stale candidate quaternion. Re-enable only with an explicit new start.
 */
bool joint_candidate_update_gyro(joint_candidate_t *candidate,
    const float raw_gyro_dps[3], const joint_candidate_context_t *context);
bool joint_candidate_update_accel(joint_candidate_t *candidate,
    const float calibrated_accel_ms2[3], const joint_candidate_context_t *context);
void joint_candidate_stop(joint_candidate_t *candidate);

#endif

#include "joint_candidate.h"

#include <math.h>
#include <string.h>

#define JOINT_DEG_TO_RAD 0.01745329251994329577f

static bool finite_vector(const float v[3])
{
    return v && isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static bool stop_with(joint_candidate_t *candidate, joint_candidate_status_t status)
{
    candidate->enabled = false;
    candidate->status = status;
    return false;
}

static bool valid_model(const joint_candidate_model_t *model)
{
    if (!model || model->schema != JOINT_CANDIDATE_SCHEMA || !model->model_id ||
        !model->config_id || model->input_domain != JOINT_CANDIDATE_DOMAIN_RAW_IMU_DPS ||
        !model->maximum_temperature_age_us ||
        !isfinite(model->reference_temperature_c) ||
        !isfinite(model->minimum_temperature_c) || !isfinite(model->maximum_temperature_c) ||
        model->minimum_temperature_c >= model->maximum_temperature_c ||
        model->reference_temperature_c < model->minimum_temperature_c ||
        model->reference_temperature_c > model->maximum_temperature_c ||
        !finite_vector(model->body_bias_dps) || !finite_vector(model->body_slope_dps_per_c) ||
        !finite_vector(model->world_vqf_dps)) return false;
    for (int axis = 0; axis < 3; ++axis) {
        /* Identity means identity, not an unvalidated small scale correction. */
        if (model->sensitivity_scale[axis] != 1.0f) return false;
        float low = model->body_bias_dps[axis] + model->body_slope_dps_per_c[axis] *
            (model->minimum_temperature_c - model->reference_temperature_c);
        float high = model->body_bias_dps[axis] + model->body_slope_dps_per_c[axis] *
            (model->maximum_temperature_c - model->reference_temperature_c);
        if (!isfinite(low) || !isfinite(high)) return false;
    }
    return true;
}

static bool valid_context(joint_candidate_t *candidate, const joint_candidate_context_t *context)
{
    if (!context || !context->boot_generation ||
        context->boot_generation != candidate->boot_generation ||
        context->config_id != candidate->model.config_id ||
        context->now_uptime_us < candidate->last_uptime_us)
        return stop_with(candidate, JOINT_CANDIDATE_CONTEXT_CHANGED);
    if (!context->temperature_valid || !isfinite(context->temperature_c) ||
        context->temperature_uptime_us > context->now_uptime_us)
        return stop_with(candidate, JOINT_CANDIDATE_INVALID_TEMPERATURE);
    if (context->now_uptime_us - context->temperature_uptime_us >
        candidate->model.maximum_temperature_age_us)
        return stop_with(candidate, JOINT_CANDIDATE_STALE_TEMPERATURE);
    if (context->temperature_c < candidate->model.minimum_temperature_c ||
        context->temperature_c > candidate->model.maximum_temperature_c)
        return stop_with(candidate, JOINT_CANDIDATE_TEMPERATURE_OUT_OF_RANGE);
    return true;
}

static bool valid_quaternion(const float q[4])
{
    float norm2 = 0.0f;
    for (int i = 0; i < 4; ++i) {
        if (!isfinite(q[i])) return false;
        norm2 += q[i] * q[i];
    }
    return isfinite(norm2) && norm2 > .98f * .98f && norm2 < 1.02f * 1.02f;
}

static void world_to_body(const float q[4], const float world[3], float body[3])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    body[0] = (1-2*(y*y+z*z))*world[0] + 2*(x*y+w*z)*world[1] + 2*(x*z-w*y)*world[2];
    body[1] = 2*(x*y-w*z)*world[0] + (1-2*(x*x+z*z))*world[1] + 2*(y*z+w*x)*world[2];
    body[2] = 2*(x*z+w*y)*world[0] + 2*(y*z-w*x)*world[1] + (1-2*(x*x+y*y))*world[2];
}

bool joint_candidate_start(joint_candidate_t *candidate,
    const joint_candidate_model_t *model,
    const vqf_params_t *baseline_params, const vqf_state_t *baseline,
    float gyro_period_s, float accel_period_s, float mag_period_s,
    const joint_candidate_context_t *context)
{
    if (!candidate) return false;
    memset(candidate, 0, sizeof(*candidate));
    if (!valid_model(model)) return stop_with(candidate, JOINT_CANDIDATE_INVALID_MODEL);
    if (!baseline || !baseline_params || !context ||
        !isfinite(gyro_period_s) || gyro_period_s <= 0 ||
        !isfinite(accel_period_s) || accel_period_s <= 0 ||
        !isfinite(mag_period_s) || mag_period_s <= 0 ||
        !isfinite(baseline_params->tauAcc) || baseline_params->tauAcc <= 0 ||
        !isfinite(baseline_params->restFilterTau) || baseline_params->restFilterTau <= 0)
        return stop_with(candidate, JOINT_CANDIDATE_INVALID_REFERENCE);
    candidate->model = *model;
    candidate->boot_generation = context->boot_generation;
    candidate->last_uptime_us = context->now_uptime_us;
    if (!valid_context(candidate, context)) return false;
    /* The pinned getQuat6D API is non-const, so query a local copy; none of its
     * old bias/covariance/IIR state is installed in the candidate. */
    vqf_state_t reference = *baseline;
    float q[4];
    getQuat6D(&reference, q);
    if (!valid_quaternion(q)) return stop_with(candidate, JOINT_CANDIDATE_INVALID_REFERENCE);
    candidate->params = *baseline_params;
    candidate->params.motionBiasEstEnabled = false;
    candidate->params.restBiasEstEnabled = false;
    initVqf(&candidate->params, &candidate->state, &candidate->coeffs,
            gyro_period_s, accel_period_s, mag_period_s);
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (int i = 0; i < 4; ++i) candidate->state.gyrQuat[i] = q[i] / norm;
    candidate->last_temperature_c = context->temperature_c;
    candidate->enabled = true;
    candidate->status = JOINT_CANDIDATE_ACTIVE;
    return true;
}

bool joint_candidate_update_gyro(joint_candidate_t *candidate,
    const float raw_gyro_dps[3], const joint_candidate_context_t *context)
{
    if (!candidate || !candidate->enabled) return false;
    if (!valid_context(candidate, context)) return false;
    if (!finite_vector(raw_gyro_dps)) return stop_with(candidate, JOINT_CANDIDATE_INVALID_SAMPLE);
    float q[4], body[3], earth[3], corrected[3], rad[3];
    getQuat6D(&candidate->state, q);
    if (!valid_quaternion(q)) return stop_with(candidate, JOINT_CANDIDATE_NUMERICAL_FAILURE);
    world_to_body(q, candidate->model.world_vqf_dps, earth);
    for (int axis = 0; axis < 3; ++axis) {
        body[axis] = candidate->model.body_bias_dps[axis] + candidate->model.body_slope_dps_per_c[axis] *
            (context->temperature_c - candidate->model.reference_temperature_c);
        corrected[axis] = raw_gyro_dps[axis] - body[axis] - earth[axis];
        rad[axis] = corrected[axis] * JOINT_DEG_TO_RAD;
    }
    if (!finite_vector(body) || !finite_vector(earth) || !finite_vector(rad))
        return stop_with(candidate, JOINT_CANDIDATE_INVALID_SAMPLE);
    updateGyr(&candidate->params, &candidate->state, &candidate->coeffs, rad);
    getQuat6D(&candidate->state, q);
    if (!valid_quaternion(q)) return stop_with(candidate, JOINT_CANDIDATE_NUMERICAL_FAILURE);
    memcpy(candidate->last_body_correction_dps, body, sizeof(body));
    memcpy(candidate->last_world_body_dps, earth, sizeof(earth));
    memcpy(candidate->last_corrected_gyro_dps, corrected, sizeof(corrected));
    candidate->last_temperature_c = context->temperature_c;
    candidate->last_uptime_us = context->now_uptime_us;
    candidate->gyro_samples++;
    return true;
}

bool joint_candidate_update_accel(joint_candidate_t *candidate,
    const float calibrated_accel_ms2[3], const joint_candidate_context_t *context)
{
    if (!candidate || !candidate->enabled) return false;
    if (!valid_context(candidate, context)) return false;
    if (!finite_vector(calibrated_accel_ms2)) return stop_with(candidate, JOINT_CANDIDATE_INVALID_SAMPLE);
    updateAcc(&candidate->params, &candidate->state, &candidate->coeffs, calibrated_accel_ms2);
    float q[4];
    getQuat6D(&candidate->state, q);
    if (!valid_quaternion(q)) return stop_with(candidate, JOINT_CANDIDATE_NUMERICAL_FAILURE);
    candidate->last_uptime_us = context->now_uptime_us;
    candidate->accel_samples++;
    return true;
}

void joint_candidate_stop(joint_candidate_t *candidate)
{
    if (candidate) stop_with(candidate, JOINT_CANDIDATE_OFF);
}

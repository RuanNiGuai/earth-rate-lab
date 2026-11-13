/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "util.h"

#include <math.h>
#include <zephyr/kernel.h>

#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
#include <cmsis_core.h>
#endif

#include "../src/vqf.h" // conflicting with vqf.h in local path

#include "../vqf/vqf.h" // conflicting with vqf.h in vqf-c

#include "retained.h" // for BUILD_ASSERT on fusion_data size
#include "system/system.h"
#include "system/led.h"
#include "sensor/calibration.h"
#include "../earth_pair.h"
#include <zephyr/sys/atomic.h>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0f)
#endif

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
/* ---------- Adaptive tauAcc configuration ---------- */
/*
 * Three-regime adaptive tauAcc strategy:
 *
 * 1. REST (VQF rest detection active):
 *    tauAcc = TAU_ACC_REST.
 *    At true rest the accelerometer perfectly reflects gravity,
 *    so a small tau enables fast inclination convergence while
 *    the rest bias estimator handles gyro drift.
 *
 * 2. GENTLE MOTION (accel close to gravity, low linear acceleration):
 *    tauAcc = TAU_ACC_GENTLE (high).
 *    Micro-movements (breathing, subtle swaying) contaminate the
 *    accelerometer slightly.  A high tau avoids these perturbations
 *    being coupled into heading via the bias estimator R-matrix.
 *
 * 3. AGGRESSIVE MOTION (significant linear acceleration):
 *    tauAcc = TAU_ACC_AGGRESSIVE (lower).
 *    Active deliberate motion—fast inclination correction helps
 *    prevent pitch/roll drift from coupling into heading.
 *
 * The accel deviation |‖a‖ - g| drives a [0,1] motion_intensity
 * with fast-attack / slow-release dynamics.  During motion, tauAcc
 * is linearly interpolated from GENTLE (0) to AGGRESSIVE (1).
 * When rest is detected, TAU_ACC_REST overrides.
 */
#define ADAPTIVE_TAU_ACC_REST       7.6f   /* tauAcc when at rest (seconds) */
#define ADAPTIVE_TAU_ACC_GENTLE     1.0f   /* tauAcc during gentle motion (seconds) */
#define ADAPTIVE_TAU_ACC_AGGRESSIVE 8.5f   /* tauAcc under aggressive motion (seconds) */
#define ADAPTIVE_TAU_ACC_LEVELS     10     /* quantization levels */
#define ADAPTIVE_ACC_DEV_TH         2.0f   /* accel deviation threshold (m/s²) */
#define ADAPTIVE_ATTACK_ALPHA       0.27f  /* fast attack coefficient (per sample) */
#define ADAPTIVE_RELEASE_ALPHA      0.39f  /* slow release coefficient (per sample) */
#define TAU_SMOOTH_ALPHA_DOWN       0.21f   /* tauAcc decrease smoothing (per sample) */
#define TAU_SMOOTH_ALPHA_UP         0.21f  /* tauAcc increase smoothing (per sample) */
#endif /* CONFIG_VQF_ADAPTIVE_TAU_ACC */

#if IS_ENABLED(CONFIG_VQF_SOFT_MBE_GATE)
/* ---------- Soft motion-bias-estimation gate configuration ---------- */
#define SOFT_MBE_GYRO_TH_RAD_S       (6.0f * DEG_TO_RAD)
#define SOFT_MBE_ACC_DEV_TH          (0.009f * CONST_EARTH_GRAVITY)
#define SOFT_MBE_LIN_ACC_TH          (0.075f * CONST_EARTH_GRAVITY)
#define SOFT_MBE_SMOOTH_T            10.0f
#define SOFT_MBE_MIN_QUASI_T         45.0f
#define SOFT_MBE_MARGIN_T            4.0f
#define SOFT_MBE_BIAS_NOISE_SCALE    0.001733f
#endif /* CONFIG_VQF_SOFT_MBE_GATE */

static uint8_t imu_id;

static vqf_params_t params;
static vqf_state_t state;
static vqf_coeffs_t coeffs;
static earth_pair_t earth_pair;
static north_yaw_ref_t north_ref;
/* 0 = none, 1 = captured, 2 = aligned, 3 = refused */
static volatile uint8_t north_state;
static atomic_t north_command; /* 1 = 3-second delayed capture, 2 = 3-second delayed realign */
static atomic_t north_busy; /* Covers queue, countdown, and stable-observation interval. */
static uint8_t north_phase;
static int64_t north_deadline_ms, north_stable_since_ms;
static int64_t north_accel_ms;
static float north_accel_norm;
static float mbe_gyro_mag = 0.0f; /* calibrated gyro magnitude */
#define NORTH_SETTLE_MS 3000
#define NORTH_STABLE_MS 1000
#define NORTH_GYRO_MAX_RAD_S 0.035f
#define NORTH_ACCEL_MAX_DEVIATION 0.05f
#define NORTH_MAX_DELTA_RAD 0.436332313f /* 25 degrees */

static bool north_request(int command) {
    if (!atomic_cas(&north_busy, 0, 1)) return false;
    atomic_set(&north_command, command);
    return true;
}
uint8_t vqf_north_state(void) { return north_state; }

bool vqf_north_capture_reference_request(void) {
    return north_request(1);
}
bool vqf_north_realign_request(void) {
    return north_request(2);
}
/* Called only by the VQF gyro owner thread. A new Earth Pair invalidates this
 * reference; a full tracker reboot also clears it (not an NVS pose). */
static void north_reset_session(void) {
    north_ref.valid=false;
    north_phase=0;
    north_stable_since_ms=0;
    atomic_set(&north_command,0);
    atomic_set(&north_busy,0);
}
static void north_process_gyro(void) {
    const int64_t now = k_uptime_get();
    if (!earth_pair.enabled || earth_pair.mode != PAIR_EARTH || sensor_get_mag_enabled()) {
        if (north_ref.valid || north_phase)
            printk("NORTH reference invalidated: Earth Pair inactive\n");
        north_reset_session();
        return;
    }

    /* Requests are consumed once, on the gyro owner thread. Neither command
     * touches the persistent Earth calibration or Earth Pose Save/NVS. */
    const atomic_val_t cmd = atomic_set(&north_command, 0);
    if (cmd == 1 || cmd == 2) {
        if (cmd == 1) {
            /* Never reuse a previous wearing-session baseline after a failed
             * replacement request; the user may have moved the trackers. */
            north_ref.valid = false;
        } else if (!north_ref.valid) {
            printk("NORTH align refused: capture this wearing session first (send <id> north set)\n");
            atomic_set(&north_busy, 0);
            return;
        }
        north_phase = (uint8_t)cmd; /* 1=set, 2=align */
        north_deadline_ms = now + NORTH_SETTLE_MS;
        north_stable_since_ms = 0;
        printk("NORTH %s: wait 3s, face the SAME physical marker and hold the same upright pose\n",
               cmd == 1 ? "set" : "align");
    }

    if (!north_phase || now < north_deadline_ms) return;
    if (now > north_deadline_ms + 8000) {
        printk("NORTH %s refused: stable pose not found in 8s\n",
               north_phase == 1 ? "set" : "align");
        north_phase = 0;
        north_stable_since_ms = 0;
        atomic_set(&north_busy, 0);
        return;
    }

    const bool stable = earth_pair.state.restDetected &&
        mbe_gyro_mag < NORTH_GYRO_MAX_RAD_S &&
        (now - north_accel_ms) < 250 &&
        fabsf(north_accel_norm - CONST_EARTH_GRAVITY) <
            NORTH_ACCEL_MAX_DEVIATION * CONST_EARTH_GRAVITY;
    if (!stable) {
        north_stable_since_ms = 0;
        return;
    }
    if (!north_stable_since_ms) north_stable_since_ms = now;
    if (now - north_stable_since_ms < NORTH_STABLE_MS) return;

    const uint8_t completed_phase = north_phase;
    north_phase = 0;
    north_stable_since_ms = 0;
    atomic_set(&north_busy, 0);
    if (completed_phase == 1) {
        if (north_yaw_capture(&north_ref, &earth_pair))
            printk("NORTH reference captured for this boot/wearing session (RAM only)\n");
        else
            printk("NORTH reference capture refused: Earth Pair is not ready or still\n");
        return;
    }

    float delta = 0;
    if (!north_yaw_delta(&north_ref, &earth_pair, &delta)) {
        printk("NORTH align refused: reference tilt/heading axes inconsistent\n");
        return;
    }
    if (fabsf(delta) > NORTH_MAX_DELTA_RAD) {
        printk("NORTH align refused: |delta| exceeds 25 degrees (%.2f deg)\n",
               (double)(delta / DEG_TO_RAD));
        return;
    }
    if (fabsf(delta) < 0.5f * DEG_TO_RAD) {
        printk("NORTH unchanged: delta below 0.5 deg\n");
        return;
    }
    if (earth_pair_realign_yaw(&earth_pair, delta))
        printk("NORTH applied: internal Earth Pair yaw %+0.3f deg\n",
               (double)(delta / DEG_TO_RAD));
    else
        printk("NORTH align refused: Earth Pair state validation failed\n");
}

/* Console only queues requests. All filter state changes happen in sensor thread. */
static atomic_t earth_request;
static float requested_earth[3];
static uint32_t earth_gyro_samples;
#define VQF_MEM_SIZE (sizeof(vqf_state_t) + sizeof(vqf_coeffs_t))

/* At boot, rotate the saved body reference (or the shared firmware default)
 * into the live 6D world frame. The device must be in the matching physical
 * reference pose; gravity alone cannot establish geographic north. */
#define EARTH_AUTO_ENABLE 1
static bool earth_auto_pending = EARTH_AUTO_ENABLE;
static bool earth_pose_restored; /* a saved quaternion was restored: pose is known */

/* Gate the motion-bias estimator on the accelerometer matching gravity in BOTH
 * magnitude and direction. A horizontal linear acceleration can keep |a| at 1 g
 * while pointing away from vertical, so the norm alone is not sufficient: the
 * angle between the measured acceleration and the gravity direction implied by
 * the current attitude must also be small. Two thresholds with hysteresis. */
#define MBE_GATE_MAG_OPEN  0.03f   /* g */
#define MBE_GATE_MAG_CLOSE 0.06f   /* g */
#define MBE_GATE_ANG_OPEN  0.0175f /* rad, 1 deg */
#define MBE_GATE_ANG_CLOSE 0.0524f /* rad, 3 deg */
/* A flat yaw rotation keeps the accelerometer near 1 g and vertical, so the
 * accel-only gate stays open while the body is rotating. Require the angular
 * rate itself to be low before the motion-bias estimator may run. */
#define MBE_GATE_GYRO_OPEN  0.05f  /* rad/s, ~2.9 deg/s */
#define MBE_GATE_GYRO_CLOSE 0.10f  /* rad/s, ~5.7 deg/s */
/* A/B switch: 0 = motion-bias estimation follows only the accelerometer
 * consistency gate (original behaviour, updates during motion); 1 = also
 * require the body rate to be low. */
#define MBE_GYRO_GATE_ENABLED 0
/* Skip the accelerometer tilt correction when |a| deviates from 1 g by more
 * than this (linear acceleration corrupts the gravity direction). The gate
 * can be toggled at runtime with `gyro accelgate on|off`. */
#define ACCEL_UPDATE_GATE_DEV 0.05f
/* 0 = off, 1 = magnitude gate, 2 = angular-rate gate. The rate gate rejects
 * inclination observations during faster motion, where linear acceleration
 * and sensor timing mismatch can corrupt gravity. It does not detect all
 * linear acceleration during slow motion. */
static atomic_t accel_gate_mode = 2;
static atomic_t accel_gate_request; /* mode + 1, zero means no request */
static bool accel_recovery_pending;
static float accel_trusted_s;
#define ACCEL_GATE_RECOVERY_S 0.5f
static bool accel_rate_allowed = true;
#define ACCEL_RATE_GATE_OPEN 0.30f  /* rad/s, ~17 deg/s */
#define ACCEL_RATE_GATE_CLOSE 0.50f /* rad/s, ~29 deg/s */
static void accel_gate_reset(bool recovering)
{
    atomic_set(&accel_gate_mode,2);
    atomic_set(&accel_gate_request,0);
    accel_rate_allowed=true;
    accel_recovery_pending=recovering;
    accel_trusted_s=0;
}
static bool mbe_gate_enabled = true;
static void motion_bias_gate(float ax, float ay, float az)
{
	float q[4];
	getQuat6D(&state, q);
	const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
	/* third row of the body->world rotation: world up in body coordinates */
	float ux = 2.0f * (qx * qz - qw * qy);
	float uy = 2.0f * (qy * qz + qw * qx);
	float uz = 1.0f - 2.0f * (qx * qx + qy * qy);
	float an = sqrtf(ax * ax + ay * ay + az * az);
	float mag_dev = fabsf(an - 1.0f);
	float cos_ang = (an > 1e-6f)
		? (ax * ux + ay * uy + az * uz) / an : 1.0f;
	cos_ang = cos_ang > 1.0f ? 1.0f : (cos_ang < -1.0f ? -1.0f : cos_ang);
	float ang_dev = acosf(cos_ang);
	bool gyro_ok = !MBE_GYRO_GATE_ENABLED || (mbe_gyro_mag < MBE_GATE_GYRO_OPEN);
	bool gyro_bad = MBE_GYRO_GATE_ENABLED && (mbe_gyro_mag > MBE_GATE_GYRO_CLOSE);
	bool ok = (mag_dev < MBE_GATE_MAG_OPEN) && (ang_dev < MBE_GATE_ANG_OPEN) && gyro_ok;
	bool bad = (mag_dev > MBE_GATE_MAG_CLOSE) || (ang_dev > MBE_GATE_ANG_CLOSE) || gyro_bad;
	if (mbe_gate_enabled && bad) {
		mbe_gate_enabled = false;
	} else if (!mbe_gate_enabled && ok) {
		mbe_gate_enabled = true;
	}
	params.motionBiasEstEnabled = mbe_gate_enabled;
	earth_pair.params.motionBiasEstEnabled = mbe_gate_enabled;
}

bool vqf_earth_request(float azimuth_deg, float latitude_deg, bool enable)
{
	if (enable && (!isfinite(azimuth_deg) || !isfinite(latitude_deg) ||
		fabsf(latitude_deg)>90 || sensor_get_mag_enabled())) return false;
	if (!atomic_cas(&earth_request,0,-1)) return false;
	if (enable) {
		float lat=latitude_deg*DEG_TO_RAD, az=azimuth_deg*DEG_TO_RAD;
		const float rate=7.2921150e-5f;
		requested_earth[0]=rate*cosf(lat)*cosf(az);
		requested_earth[1]=rate*cosf(lat)*sinf(az);
		requested_earth[2]=rate*sinf(lat);
	}
	atomic_set(&earth_request,enable ? 1 : 2);
	return true;
}

bool vqf_bias_pair_request(bool enable)
{
	if (enable && sensor_get_mag_enabled()) return false;
	return atomic_cas(&earth_request,0,enable ? 3 : 4);
}

/* Boot-pose guard: the stored Earth reference is only valid for +X up. */
static float last_accel_g[3];
static bool boot_pose_ok = true;

static void earth_apply_request(void)
{
	atomic_val_t request=atomic_get(&earth_request);
	if (request==2) {
		earth_pair.enabled=false;
		earth_pair.mode=PAIR_NONE;
		printk("EARTH disabled: original VQF output active\n");
	} else if (request==1) {
		if (!earth_pair.enabled && !sensor_get_mag_enabled() && earth_pair_start(&earth_pair,&params,&state,&coeffs,requested_earth)) {
			earth_gyro_samples=0;
			earth_pose_restored = false;
			printk("EARTH enabled: coarse external north, full-rate pair, candidate output active, no calibration writes\n");
		} else {
			printk("EARTH rejected: requires settled rest, initialized filters and magnetometer disabled\n");
		}
	} else if (request==4) {
		earth_pair.enabled=false;
		earth_pair.mode=PAIR_NONE;
		printk("BIASPAIR disabled: original VQF output active\n");
	} else if (request==3) {
		if (!earth_pair.enabled && !sensor_get_mag_enabled() &&
			earth_pair_start_rest_bias(&earth_pair,&params,&state,&coeffs)) {
			earth_gyro_samples=0;
			printk("BIASPAIR enabled: rest-only bias candidate output active; Earth off; baseline retained\n");
		} else {
			printk("BIASPAIR rejected: requires pair off, settled rest, initialized filters and magnetometer disabled\n");
		}
	}
	if (request>0) {
        north_reset_session();
		earth_auto_pending=false; /* Explicit commands supersede boot auto-start. */
		atomic_set(&earth_request,0);
	}

	/* Boot-time automatic start. earth_pair_start() itself refuses until the
	 * filter and rest detector are settled, so this retries on later samples. */
	if (earth_auto_pending && request==0 && sensor_boot_zro_ready() && !earth_pair.enabled && !sensor_get_mag_enabled()) {
		/* The stored reference is only valid for the fixed +X-up pose. Reject a
		 * misaligned boot instead of applying a wrong Earth vector. */
		float an = sqrtf(last_accel_g[0]*last_accel_g[0] + last_accel_g[1]*last_accel_g[1] +
		                 last_accel_g[2]*last_accel_g[2]);
		if (an > 0.5f && !earth_pose_restored) {
			int uidx = retained->earthUpAxis / 2;
			float usg = (retained->earthUpAxis & 1) ? -1.0f : 1.0f;
			if (uidx < 0 || uidx > 2) { uidx = 0; usg = 1.0f; }
			float cos_up = last_accel_g[uidx] * usg / an;
			if (cos_up < 0.985f) { /* more than ~10 deg off */
				printk("EARTH boot pose rejected: +X up required (cos %.3f); reference NOT applied, reboot in the fixed pose\n",
				       (double)cos_up);
				set_led(SYS_LED_PATTERN_ERROR_B, SYS_LED_PRIORITY_BOOT);
				boot_pose_ok = false;
				earth_auto_pending = false;
				return;
			}
		}
		float q[4];
		getQuat6D(&state,q);
		const float qw=q[0], qx=q[1], qy=q[2], qz=q[3];
		float c[3];
		if (retained->earthCalValid) {
			c[0] = retained->earthCalBody[0] * DEG_TO_RAD;
			c[1] = retained->earthCalBody[1] * DEG_TO_RAD;
			c[2] = retained->earthCalBody[2] * DEG_TO_RAD;
		} else {
			sys_earth_reference_default(c);
			for (int i=0;i<3;i++) c[i] *= DEG_TO_RAD;
		}
		float e[3];
		e[0]=(1-2*(qy*qy+qz*qz))*c[0]+2*(qx*qy-qw*qz)*c[1]+2*(qx*qz+qw*qy)*c[2];
		e[1]=2*(qx*qy+qw*qz)*c[0]+(1-2*(qx*qx+qz*qz))*c[1]+2*(qy*qz-qw*qx)*c[2];
		e[2]=2*(qx*qz-qw*qy)*c[0]+2*(qy*qz+qw*qx)*c[1]+(1-2*(qx*qx+qy*qy))*c[2];
		if (earth_pair_start(&earth_pair,&params,&state,&coeffs,e)) {
			earth_gyro_samples=0;
			/* Boot-complete indication: two quick flashes once the Earth
			 * reference is applied; the device may be moved after this. */
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_BOOT);
			earth_auto_pending=false;
			printk("EARTH auto-enabled at boot: reference body vector rotated into the current world frame\n");
		}
	}
}

bool vqf_earth_debug(float baseline_q[4], float candidate_q[4], float candidate_bias[3], uint32_t *samples)
{
	getQuat9D(&state,baseline_q);
	if (earth_pair.enabled) getQuat6D(&earth_pair.state,candidate_q);
	else memcpy(candidate_q,baseline_q,4*sizeof(float));
	for (int i=0;i<3;i++) candidate_bias[i]=(earth_pair.enabled ? earth_pair.state.bias[i] : state.bias[i])/DEG_TO_RAD;
	*samples=earth_gyro_samples;
	return earth_pair.enabled && earth_pair.mode==PAIR_EARTH;
}

bool vqf_bias_pair_debug(float baseline_q[4], float candidate_q[4], float candidate_bias[3],
	uint32_t *samples, bool *candidate_rest)
{
	vqf_earth_debug(baseline_q,candidate_q,candidate_bias,samples);
	*candidate_rest=earth_pair.enabled ? earth_pair.state.restDetected : state.restDetected;
	return earth_pair.enabled && earth_pair.mode==PAIR_REST_BIAS;
}

static float last_a[3] = {0};
static volatile float vqf_bench_sink;
static vqf_params_t vqf_bench_params;
static vqf_params_t vqf_bench_warm_params;
static vqf_state_t vqf_bench_state;
static vqf_state_t vqf_bench_warm_state;
static vqf_coeffs_t vqf_bench_coeffs;
static vqf_coeffs_t vqf_bench_warm_coeffs;
static float vqf_bench_quat[4];

static const vqf_real_t vqf_bench_gyr_samples[][3] = {
	{0.12f, -0.34f, 0.56f},
	{-1.10f, 0.45f, 0.08f},
	{0.78f, 0.11f, -0.29f},
	{-0.05f, 0.92f, 0.37f},
};

static const vqf_real_t vqf_bench_acc_samples[][3] = {
	{0.30f, 0.10f, 9.70f},
	{-0.25f, 0.45f, 9.63f},
	{0.60f, -0.15f, 9.55f},
	{-0.40f, -0.20f, 9.81f},
};

static const vqf_real_t vqf_bench_mag_samples[][3] = {
	{0.32f, 0.05f, -0.41f},
	{0.28f, 0.10f, -0.39f},
	{0.35f, -0.02f, -0.43f},
	{0.30f, 0.08f, -0.40f},
};

static const size_t vqf_bench_sample_count = sizeof(vqf_bench_gyr_samples) / sizeof(vqf_bench_gyr_samples[0]);

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
/* Adaptive tauAcc state */
static float motion_intensity;       /* low-pass filtered motion intensity [0,1] */
static float current_tau_level = -1; /* current quantized level (-1 = unset) */
static float smoothed_tau;           /* smoothed tauAcc for gradual transitions */
#endif

#if IS_ENABLED(CONFIG_VQF_SOFT_MBE_GATE)
static float soft_mbe_latest_gyr_norm;
static float soft_mbe_gyr_norm_lp;
static float soft_mbe_acc_dev_lp;
static float soft_mbe_lin_acc_lp;
static float soft_mbe_quasi_t;
static float soft_mbe_margin_t;
static float soft_mbe_scale = 1.0f;
static float soft_mbe_base_bias_v;
#endif

/* Rest detection diagnostics */
static uint32_t rest_enter_count;
static uint32_t rest_exit_count;
static float rest_total_s;
static float rest_last_enter_time;   /* uptime when last rest started */
static float rest_last_duration_s;
static float uptime_s;
static bool prev_rest_detected;

/* Circular rest event log */
#define REST_EVENT_LOG_SIZE 5
static struct {
	float time_s;
	bool entered;
} rest_event_log[REST_EVENT_LOG_SIZE];
static uint8_t rest_event_idx;  /* next write position */
static uint8_t rest_event_total; /* total events (up to log size) */


#if IS_ENABLED(CONFIG_VQF_SOFT_MBE_GATE)
static inline float vqf_square_f(float v)
{
	return v * v;
}

static void soft_mbe_apply_bias_noise_scale(float scale)
{
	if (scale < 1e-6f)
		scale = 1e-6f;

	soft_mbe_scale = scale;
	coeffs.biasV = soft_mbe_base_bias_v * scale;

	float sigma_motion = params.biasSigmaMotion * 100.0f;
	float p_motion = sigma_motion * sigma_motion;
	coeffs.biasMotionW = vqf_square_f(p_motion) / coeffs.biasV + p_motion;
	coeffs.biasVerticalW = coeffs.biasMotionW /
		fmaxf(params.biasVerticalForgettingFactor, 1e-10f);

	float sigma_rest = params.biasSigmaRest * 100.0f;
	float p_rest = sigma_rest * sigma_rest;
	coeffs.biasRestW = vqf_square_f(p_rest) / coeffs.biasV + p_rest;
}

static void soft_mbe_reset(void)
{
	soft_mbe_latest_gyr_norm = 0.0f;
	soft_mbe_gyr_norm_lp = 0.0f;
	soft_mbe_acc_dev_lp = 0.0f;
	soft_mbe_lin_acc_lp = 0.0f;
	soft_mbe_quasi_t = 0.0f;
	soft_mbe_margin_t = 0.0f;
	soft_mbe_scale = 1.0f;
	soft_mbe_base_bias_v = coeffs.biasV;
}

static void soft_mbe_update_gyro_norm(const float g_rad[3])
{
	soft_mbe_latest_gyr_norm = sqrtf(g_rad[0] * g_rad[0] +
					       g_rad[1] * g_rad[1] +
					       g_rad[2] * g_rad[2]);
}

static void soft_mbe_pre_accel_update(const float a_m_s2[3])
{
	float dt = coeffs.accTs;
	float alpha = dt / (SOFT_MBE_SMOOTH_T + dt);

	float a_norm = sqrtf(a_m_s2[0] * a_m_s2[0] +
				   a_m_s2[1] * a_m_s2[1] +
				   a_m_s2[2] * a_m_s2[2]);
	float acc_dev = fabsf(a_norm - CONST_EARTH_GRAVITY);

	float q[4];
	getQuat6D(&state, q);
	float gravity_body[3];
	gravity_body[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]) * CONST_EARTH_GRAVITY;
	gravity_body[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]) * CONST_EARTH_GRAVITY;
	gravity_body[2] = 2.0f * (q[0] * q[0] - 0.5f + q[3] * q[3]) * CONST_EARTH_GRAVITY;
	float lin_x = a_m_s2[0] - gravity_body[0];
	float lin_y = a_m_s2[1] - gravity_body[1];
	float lin_z = a_m_s2[2] - gravity_body[2];
	float lin_acc = sqrtf(lin_x * lin_x + lin_y * lin_y + lin_z * lin_z);

	soft_mbe_gyr_norm_lp += alpha * (soft_mbe_latest_gyr_norm - soft_mbe_gyr_norm_lp);
	soft_mbe_acc_dev_lp += alpha * (acc_dev - soft_mbe_acc_dev_lp);
	soft_mbe_lin_acc_lp += alpha * (lin_acc - soft_mbe_lin_acc_lp);

	bool low_energy = !state.restDetected &&
		soft_mbe_gyr_norm_lp <= SOFT_MBE_GYRO_TH_RAD_S &&
		soft_mbe_acc_dev_lp <= SOFT_MBE_ACC_DEV_TH &&
		soft_mbe_lin_acc_lp <= SOFT_MBE_LIN_ACC_TH;

	if (low_energy) {
		soft_mbe_quasi_t += dt;
		if (soft_mbe_quasi_t >= SOFT_MBE_MIN_QUASI_T) {
			soft_mbe_margin_t = SOFT_MBE_MARGIN_T;
		}
	} else {
		soft_mbe_quasi_t = 0.0f;
		if (soft_mbe_margin_t > 0.0f) {
			soft_mbe_margin_t = fmaxf(soft_mbe_margin_t - dt, 0.0f);
		}
	}

	float scale = soft_mbe_margin_t > 0.0f ? SOFT_MBE_BIAS_NOISE_SCALE : 1.0f;
	soft_mbe_apply_bias_noise_scale(scale);
}
#endif /* CONFIG_VQF_SOFT_MBE_GATE */

void vqf_update_sensor_ids(int imu)
{
	imu_id = imu;
}

static void set_params()
{
	init_params(&params);
	params.tauAcc = 4.3f;
	params.biasClip = 2.0f;
	params.biasForgettingTime = 427.0f;
	params.biasSigmaInit = 1.10f;
	params.biasSigmaMotion = 0.048f;
	params.biasSigmaRest = 0.0153f;
	params.biasVerticalForgettingFactor = 0.0001f;
	params.motionBiasEstEnabled = true;
	params.restBiasEstEnabled = true;
	params.restFilterTau = 0.66f;
	params.restMinT = 0.63f;
	params.restThGyr = 0.68f;
	params.restThAcc = 0.21f;
	params.magDistRejectionEnabled = true;
	params.tauMag = 9.0f;
	params.magCurrentTau = 0.50f;
	params.magNormTh = 0.09f;
	params.magDipTh = 6.0f;
	params.magRefTau = 15.0f;
	params.magNewTime = 12.0f;
	params.magNewFirstTime = 5.5f;
	params.magNewMinGyr = 16.0f;
	params.magMinUndisturbedTime = 0.6f;
	params.magMaxRejectionTime = 3200.0f;
	params.magRejectionFactor = 1150.0f;
}

void vqf_init(float g_time, float a_time, float m_time)
{
    if(sys_is_wom_resume()) sys_startup_fusion_result(3);
    north_reset_session();
    accel_gate_reset(false);
	earth_pair.enabled=false;
	earth_auto_pending=EARTH_AUTO_ENABLE && !sys_is_wom_resume();
    if(sys_is_wom_resume()) printk("EARTH WoM baseline unavailable; reboot in fixed reference pose\n");
	atomic_set(&earth_request,0);
	set_params();
	initVqf(&params, &state, &coeffs, g_time, a_time, m_time);
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	motion_intensity = 0.0f;
	current_tau_level = -1.0f;
	smoothed_tau = params.tauAcc;
#endif
#if IS_ENABLED(CONFIG_VQF_SOFT_MBE_GATE)
	soft_mbe_reset();
#endif
	rest_enter_count = 0;
	rest_exit_count = 0;
	rest_total_s = 0;
	rest_last_enter_time = 0;
	rest_last_duration_s = 0;
	uptime_s = 0;
	prev_rest_detected = false;
	rest_event_idx = 0;
	rest_event_total = 0;
}

/* The baseline occupies all 512 bytes. Version its existing retained ID,
 * covered by the whole retained-data CRC, without changing the RAM layout. */
uint8_t vqf_retained_id(void)
{
    return 0x40u + VQF_STATE_VERSION;
}

void vqf_load(const void *data)
{
    north_reset_session();
    accel_gate_reset(true);
	earth_pair.enabled=false;
	earth_auto_pending=EARTH_AUTO_ENABLE;
	atomic_set(&earth_request,0);
	BUILD_ASSERT(VQF_MEM_SIZE <= sizeof(((struct retained_data *)0)->fusion_data),
		     "VQF state+coeffs exceeds fusion_data buffer in retained memory");
	set_params();
	memcpy(&state, data, sizeof(state));
	memcpy(&coeffs, (uint8_t *)data + sizeof(state), sizeof(coeffs));
    if(sys_is_wom_resume()) {
        bool pending=false;
        uint8_t saved_gate_mode=2;
        if(!sensor_get_mag_enabled() && earth_pair_load(&earth_pair,&pending,&mbe_gate_enabled,
            &earth_gyro_samples,&saved_gate_mode,retained->earth_fusion_data,sizeof(retained->earth_fusion_data))) {
            sys_startup_fusion_result(1);
            atomic_set(&accel_gate_mode,saved_gate_mode);
            params.motionBiasEstEnabled=mbe_gate_enabled;
            /* A pending first start had no confirmed heading before sleep. Do not
             * silently establish one from an arbitrary wake pose. */
            earth_auto_pending=false;
            if(earth_pair.enabled) printk("EARTH restored after WoM: saved world reference and output filter retained\n");
            else if(pending) printk("EARTH not initialized before sleep; reboot in fixed reference pose\n");
            else printk("EARTH remains disabled after WoM\n");
        } else {
            sys_startup_fusion_result(2);
            earth_pair.enabled=false;
            earth_auto_pending=false;
            printk("EARTH WoM snapshot unavailable; reboot in fixed reference pose\n");
        }
    }
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	motion_intensity = 0.0f;
	current_tau_level = -1.0f;
	smoothed_tau = params.tauAcc;
#endif
#if IS_ENABLED(CONFIG_VQF_SOFT_MBE_GATE)
	soft_mbe_reset();
#endif
	rest_enter_count = 0;
	rest_exit_count = 0;
	rest_total_s = 0;
	rest_last_enter_time = 0;
	rest_last_duration_s = 0;
	uptime_s = 0;
	prev_rest_detected = false;
	rest_event_idx = 0;
	rest_event_total = 0;
}

void vqf_save(void *data)
{
	BUILD_ASSERT(VQF_MEM_SIZE <= sizeof(((struct retained_data *)0)->fusion_data),
		     "VQF state+coeffs exceeds fusion_data buffer in retained memory");
	memcpy(data, &state, sizeof(state));
    BUILD_ASSERT(EARTH_PAIR_SNAPSHOT_CAPACITY <= sizeof(retained->earth_fusion_data),
        "Earth snapshot exceeds retained capacity");
    earth_pair_save(&earth_pair,earth_auto_pending,mbe_gate_enabled,earth_gyro_samples,
        (uint8_t)atomic_get(&accel_gate_mode),
        retained->earth_fusion_data,sizeof(retained->earth_fusion_data));
#if IS_ENABLED(CONFIG_VQF_SOFT_MBE_GATE)
	float saved_scale = soft_mbe_scale;
	soft_mbe_apply_bias_noise_scale(1.0f);
	memcpy((uint8_t *)data + sizeof(state), &coeffs, sizeof(coeffs));
	soft_mbe_apply_bias_noise_scale(saved_scale);
#else
	memcpy((uint8_t *)data + sizeof(state), &coeffs, sizeof(coeffs));
#endif
}

void vqf_update_gyro(float *g, float time)
{
	earth_apply_request();
	ARG_UNUSED(time);
	float g_rad[3] = {0};
	// g is in deg/s, convert to rad/s
	for (int i = 0; i < 3; i++)
		g_rad[i] = g[i] * DEG_TO_RAD;
	mbe_gyro_mag = sqrtf(g_rad[0]*g_rad[0] + g_rad[1]*g_rad[1] + g_rad[2]*g_rad[2]);
#if IS_ENABLED(CONFIG_VQF_SOFT_MBE_GATE)
	soft_mbe_update_gyro_norm(g_rad);
#endif
	updateGyr(&params, &state, &coeffs, g_rad);
	earth_pair_gyro(&earth_pair,g_rad);
	if (earth_pair.enabled) earth_gyro_samples++;
    north_process_gyro();
}

void vqf_update_gyro_ts(float *g, uint64_t timestamp_us)
{
    north_reset_session();
	/* This build's FIFO path uses fixed-rate update_gyro. Reject switching to
	 * an unvalidated timestamp path during a comparison. */
	earth_pair.enabled=false;
	float g_rad[3] = {0};
	for (int i = 0; i < 3; i++)
		g_rad[i] = g[i] * DEG_TO_RAD;
	mbe_gyro_mag = sqrtf(g_rad[0]*g_rad[0] + g_rad[1]*g_rad[1] + g_rad[2]*g_rad[2]);
#if IS_ENABLED(CONFIG_VQF_SOFT_MBE_GATE)
	soft_mbe_update_gyro_norm(g_rad);
#endif
	updateGyrTs(&params, &state, &coeffs, g_rad, timestamp_us);
}

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
/**
 * @brief Pre-accel-update processing: adaptive tauAcc (experimental).
 *
 * Called before each accelerometer update to adjust VQF parameters based on
 * current motion intensity and temperature dynamics.
 *
 * @param a_m_s2 accelerometer reading in m/s²
 */
static void vqf_pre_accel_update(const float a_m_s2[3])
{
	/* --- Adaptive tauAcc based on motion intensity --- */
	float a_norm = sqrtf(a_m_s2[0] * a_m_s2[0] +
			     a_m_s2[1] * a_m_s2[1] +
			     a_m_s2[2] * a_m_s2[2]);
	float a_dev = fabsf(a_norm - CONST_EARTH_GRAVITY);
	float alpha_inst = fminf(a_dev / ADAPTIVE_ACC_DEV_TH, 1.0f);

	/* Attack-release envelope: fast increase, slow decrease */
	if (alpha_inst > motion_intensity) {
		motion_intensity += ADAPTIVE_ATTACK_ALPHA * (alpha_inst - motion_intensity);
	} else {
		motion_intensity += ADAPTIVE_RELEASE_ALPHA * (alpha_inst - motion_intensity);
	}

	/*
	 * Compute target tauAcc:
	 * - Rest detected: use rest tauAcc
	 * - Motion: linearly interpolate from GENTLE (intensity=0) to
	 *   AGGRESSIVE (intensity=1).  This supports both increasing and
	 *   decreasing curves depending on how the defines are set.
	 */
	float target_tau;
	if (state.restDetected) {
		target_tau = ADAPTIVE_TAU_ACC_REST;
	} else {
		target_tau = ADAPTIVE_TAU_ACC_GENTLE +
			     (ADAPTIVE_TAU_ACC_AGGRESSIVE - ADAPTIVE_TAU_ACC_GENTLE) *
				     motion_intensity;
	}

	/*
	 * Smooth tauAcc transitions to avoid Butterworth LP filter transients.
	 * Decrease (rest→motion) is slow to prevent heading coupling from the
	 * sudden acc correction gain change.  Increase (motion→rest) is fast
	 * so the filter settles quickly at rest.
	 */
	float tau_alpha = (target_tau < smoothed_tau) ? TAU_SMOOTH_ALPHA_DOWN
						      : TAU_SMOOTH_ALPHA_UP;
	smoothed_tau += tau_alpha * (target_tau - smoothed_tau);

	/*
	 * Quantize target_tau directly to avoid unnecessary setTauAcc() calls.
	 * The step size is derived from the full possible range of tauAcc values.
	 */
	float tau_min = fminf(fminf(ADAPTIVE_TAU_ACC_REST, ADAPTIVE_TAU_ACC_GENTLE),
			      ADAPTIVE_TAU_ACC_AGGRESSIVE);
	float tau_max = fmaxf(fmaxf(ADAPTIVE_TAU_ACC_REST, ADAPTIVE_TAU_ACC_GENTLE),
			      ADAPTIVE_TAU_ACC_AGGRESSIVE);
	float tau_step = (tau_max - tau_min) / ADAPTIVE_TAU_ACC_LEVELS;
	float quantized_tau;
	if (tau_step > 0.001f) {
		quantized_tau = tau_min +
				roundf((smoothed_tau - tau_min) / tau_step) * tau_step;
		quantized_tau = fmaxf(tau_min, fminf(quantized_tau, tau_max));
	} else {
		quantized_tau = smoothed_tau;
	}

	if (quantized_tau != current_tau_level) {
		current_tau_level = quantized_tau;
		setTauAcc(&params, &state, &coeffs, quantized_tau);
	}
}
#endif /* CONFIG_VQF_ADAPTIVE_TAU_ACC */

/**
 * @brief Track rest detection transitions and accumulate diagnostics.
 * Called after each accelerometer update (which runs rest detection).
 */
static void vqf_track_rest_diag(void)
{
	float dt = coeffs.accTs;
	uptime_s += dt;

	bool cur = state.restDetected;
	if (cur && !prev_rest_detected) {
		/* entering rest */
		rest_enter_count++;
		rest_last_enter_time = uptime_s;
		rest_event_log[rest_event_idx].time_s = uptime_s;
		rest_event_log[rest_event_idx].entered = true;
		rest_event_idx = (rest_event_idx + 1) % REST_EVENT_LOG_SIZE;
		if (rest_event_total < REST_EVENT_LOG_SIZE)
			rest_event_total++;
	} else if (!cur && prev_rest_detected) {
		/* leaving rest */
		rest_exit_count++;
		rest_last_duration_s = uptime_s - rest_last_enter_time;
		rest_event_log[rest_event_idx].time_s = uptime_s;
		rest_event_log[rest_event_idx].entered = false;
		rest_event_idx = (rest_event_idx + 1) % REST_EVENT_LOG_SIZE;
		if (rest_event_total < REST_EVENT_LOG_SIZE)
			rest_event_total++;
	}
	if (cur) {
		rest_total_s += dt;
	}
	prev_rest_detected = cur;
}

void vqf_set_accel_gate(int mode)
{
    if (mode<0 || mode>2) { printk("ACCELGATE rejected: mode must be 0..2\n"); return; }
    atomic_set(&accel_gate_request,mode+1);
    printk("ACCELGATE request queued; await sensor acknowledgement\n");
}

int vqf_get_accel_gate(void)
{
    return (int)atomic_get(&accel_gate_mode);
}

/* Called only by the sensor thread. Low-rate callbacks continue throughout
 * rejection; only the inclination/bias observations are gated. */
static bool accel_observation_trusted(const float a[3], bool timing_valid)
{
    int request=(int)atomic_set(&accel_gate_request,0);
    if (request>0) {
        atomic_set(&accel_gate_mode,request-1);
        accel_rate_allowed=true;
        accel_recovery_pending=true;
        accel_trusted_s=0;
        printk("ACCELGATE applied: %d (0=off 1=mag 2=rate)\n",request-1);
    }
    int mode=(int)atomic_get(&accel_gate_mode);
    float an=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    bool allow=timing_valid && isfinite(an) && an>1e-6f;
    if (mode==1) allow=allow && fabsf(an-1.0f)<=ACCEL_UPDATE_GATE_DEV;
    else if (mode==2) {
        if (mbe_gyro_mag>ACCEL_RATE_GATE_CLOSE) accel_rate_allowed=false;
        else if (mbe_gyro_mag<ACCEL_RATE_GATE_OPEN) accel_rate_allowed=true;
        allow=allow && accel_rate_allowed && isfinite(mbe_gyro_mag);
    }
    if (!allow) {
        accel_recovery_pending=true;
        accel_trusted_s=0;
        return false;
    }
    if (accel_recovery_pending) {
        accel_trusted_s+=coeffs.accTs;
        if (accel_trusted_s<ACCEL_GATE_RECOVERY_S) return false;
        accel_recovery_pending=false;
    }
    return true;
}

static void vqf_accel_sample(float *a, bool timing_valid)
{
    float a_m_s2[3];
    for (int i=0;i<3;i++) a_m_s2[i]=a[i]*CONST_EARTH_GRAVITY;
    bool valid=isfinite(a[0]) && isfinite(a[1]) && isfinite(a[2]) &&
        (a[0]!=0 || a[1]!=0 || a[2]!=0);
    if (valid) {
        memcpy(last_a,a_m_s2,sizeof(a_m_s2));
        north_accel_norm=sqrtf(a_m_s2[0]*a_m_s2[0]+a_m_s2[1]*a_m_s2[1]+a_m_s2[2]*a_m_s2[2]);
        north_accel_ms=k_uptime_get();
    }
#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
    if (valid) vqf_pre_accel_update(a_m_s2);
#endif
#if IS_ENABLED(CONFIG_VQF_SOFT_MBE_GATE)
    if (valid) soft_mbe_pre_accel_update(a_m_s2);
#endif
    if (valid) motion_bias_gate(a[0],a[1],a[2]);
    bool trusted=accel_observation_trusted(a,timing_valid);
    updateAccGated(&params,&state,&coeffs,a_m_s2,trusted);
    earth_pair_accel_gated(&earth_pair,a_m_s2,trusted);
    vqf_track_rest_diag();
}

void vqf_update_accel(float *a, float time)
{
    ARG_UNUSED(time);
    vqf_accel_sample(a,true);
}

void vqf_update_accel_ts(float *a, uint64_t timestamp_us)
{
    /* This filter is configured for fixed-rate samples. Advance once for the
     * observed sample, not once with a giant dt after dropped/rejected data.
     * Discontinuities require fresh trusted observations before correction. */
    bool timing_valid=true;
    if (state.lastAccTsUs) {
        if (timestamp_us<=state.lastAccTsUs) {
            accel_recovery_pending=true;
            accel_trusted_s=0;
            return; /* Duplicate/nonmonotonic samples never advance the filter. */
        }
        float dt=(float)(timestamp_us-state.lastAccTsUs)*1e-6f;
        timing_valid=dt>=0.5f*coeffs.accTs && dt<=1.5f*coeffs.accTs;
    }
    state.lastAccTsUs=timestamp_us;
    vqf_accel_sample(a,timing_valid);
}

void vqf_update_mag(float *m, float time)
{
    north_reset_session();
	earth_pair.enabled=false;
	// Use the caller-supplied time step when valid so that VQF time accumulators
	// (magCandidateT, magRejectT, etc.) and gain k run at the correct real-time
	// rate even when the sensor loop runs faster or slower than the mag ODR.
	//
	// Build a synthetic cumulative microsecond timestamp from the elapsed time so
	// updateMagTs can derive the correct dt internally (avoids calling the static
	// updateMag_internal directly).
	if (time > 0.0f && time < 10.0f) {
		uint64_t synth_ts = state.lastMagTsUs + (uint64_t)(time * 1e6f);
		if (synth_ts == 0)
			synth_ts = 1; // avoid the "uninitialized" sentinel value
		updateMagTs(&params, &state, &coeffs, m, synth_ts);
	} else {
		updateMag(&params, &state, &coeffs, m);
	}
}

void vqf_update_mag_ts(float *m, uint64_t timestamp_us)
{
    north_reset_session();
	earth_pair.enabled=false;
	updateMagTs(&params, &state, &coeffs, m, timestamp_us);
}

void vqf_update(float *g, float *a, float *m, float time){
	// TODO: time unused?
	// TODO: gyro is a different rate to the others, should they be separated
	memcpy(last_accel_g, a, sizeof(last_accel_g));
	if (g[0] != 0 || g[1] != 0 || g[2] != 0) // ignore zeroed gyro
		vqf_update_gyro(g, time);
	vqf_update_accel(a, time);
	vqf_update_mag(m, time);
}

bool vqf_boot_pose_valid(void)
{
	return boot_pose_ok;
}

void vqf_get_gyro_bias(float *g_off)
{
	getBiasEstimate(&state, &coeffs, g_off);
	// VQF internal unit is rad/s, fusion interface expects deg/s
	for (int i = 0; i < 3; i++)
		g_off[i] *= 180.0f / M_PI;
}

/* Restore a previously saved fusion orientation (used after FUSION_RESET so
 * a full server reset does not re-define the tracker's frame). */
void vqf_set_quat(const float q[4])
{
    north_reset_session();
	float n = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
	if (!isfinite(n) || n < 0.9f || n > 1.1f) return;
	state.gyrQuat[0] = q[0];
	state.gyrQuat[1] = q[1];
	state.gyrQuat[2] = q[2];
	state.gyrQuat[3] = q[3];
	state.accQuat[0] = 1.0f;
	state.accQuat[1] = 0.0f;
	state.accQuat[2] = 0.0f;
	state.accQuat[3] = 0.0f;
	state.delta = 0.0f;
	earth_pair.enabled = false;
	earth_auto_pending = EARTH_AUTO_ENABLE;
	earth_pose_restored = true;
}

void vqf_set_gyro_bias(float *g_off)
{
    north_reset_session();
	earth_pair.enabled=false;
	/* A calibration rewrites the bias the pair translated, so re-arm the
	 * boot-time automatic start; it restarts on the next gyro sample once the
	 * filter is settled again with the new bias. */
	earth_auto_pending=EARTH_AUTO_ENABLE;
	float g_off_rad[3];
	// fusion interface receives values in deg/s, VQF requires rad/s
	for (int i = 0; i < 3; i++)
		g_off_rad[i] = g_off[i] * DEG_TO_RAD;
	setBiasEstimate(&state, g_off_rad, -1);
}

void vqf_update_gyro_sanity(float *g, float *m)
{
	// TODO: does vqf tell us a "recovery state"
	return;
}

int vqf_get_gyro_sanity(void)
{
	// TODO: does vqf tell us a "recovery state"
	return 0;
}

void vqf_get_lin_a(float *lin_a)
{
	float q[4] = {0};
	vqf_get_quat(q);

	float vec_gravity[3] = {0};
	vec_gravity[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
	vec_gravity[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
	vec_gravity[2] = 2.0f * (q[0] * q[0] - 0.5f + q[3] * q[3]);

//	float *a = state.lastAccLp; // not usable, rotated by inertial frame
	float *a = last_a;
	for (int i = 0; i < 3; i++)
		lin_a[i] = a[i] - vec_gravity[i] * CONST_EARTH_GRAVITY; // gravity vector to m/s^2 before subtracting
}

void vqf_get_quat(float *q)
{
	if (earth_pair.enabled) getQuat6D(&earth_pair.state,q);
	else getQuat9D(&state, q);
}

bool vqf_get_rest_detected(void)
{
	return getRestDetected(&state);
}

bool vqf_get_mag_dist_detected(void)
{
	return getMagDistDetected(&state);
}

void vqf_reset_mag_ref(void)
{
	setMagRef(&state, 0, 0);
}

void vqf_set_mag_ref(float norm, float dip)
{
	setMagRef(&state, norm, dip);
}

float vqf_get_mag_ref_norm(void)
{
	return getMagRefNorm(&state);
}

void vqf_get_mag_ref(float *norm, float *dip)
{
	*norm = getMagRefNorm(&state);
	*dip = getMagRefDip(&state);
}

float vqf_get_delta(void)
{
	return getDelta(&state);
}

void vqf_set_delta(float delta)
{
	state.delta = delta;
}

void vqf_get_relative_rest_deviations(float *out)
{
	getRelativeRestDeviations(&params, &state, out);
}

void vqf_get_debug_info(vqf_debug_info_t *info)
{
	if (!info) return;

	info->rest_detected = getRestDetected(&state);
	getRelativeRestDeviations(&params, &state, info->rest_deviations);
	info->bias_sigma = getBiasEstimate(&state, &coeffs, info->bias);

	// Heading correction state
	info->delta = getDelta(&state);

	// Magnetic disturbance / reference
	info->mag_dist_detected = getMagDistDetected(&state);
	info->mag_ref_norm = getMagRefNorm(&state);
	info->mag_ref_dip = getMagRefDip(&state);

	// Current magnetic field (after optional magCurrentTau LPF)
	info->mag_norm = state.magNormDip[0];
	info->mag_dip = state.magNormDip[1];

	// Heading correction diagnostics (from last magnetometer update)
	info->mag_dis_angle = state.lastMagDisAngle;
	info->mag_corr_rate = state.lastMagCorrAngularRate;

	// Disturbance rejection timers
	info->mag_undisturbed_t = state.magUndisturbedT;
	info->mag_reject_t = state.magRejectT;

	// Candidate field tracking
	info->mag_candidate_norm = state.magCandidateNorm;
	info->mag_candidate_dip = state.magCandidateDip;
	info->mag_candidate_t = state.magCandidateT;

	// Filter gains
	info->mag_k = coeffs.kMag;
	info->mag_k_init = state.kMagInit;

	// Convert bias from rad/s to °/s
	for (int i = 0; i < 3; i++) {
		info->bias[i] *= 180.0f / M_PI;
	}
	info->bias_sigma *= 180.0f / M_PI;

	// Convert rad-based angles to degrees
	info->delta *= 180.0f / M_PI;
	info->mag_ref_dip *= 180.0f / M_PI;
	info->mag_dip *= 180.0f / M_PI;
	info->mag_dis_angle *= 180.0f / M_PI;

	// Convert angular rates from rad/s to °/s
	info->mag_corr_rate *= 180.0f / M_PI;

	// Convert candidate dip from rad to degrees
	info->mag_candidate_dip *= 180.0f / M_PI;

#if IS_ENABLED(CONFIG_VQF_ADAPTIVE_TAU_ACC)
	// Adaptive tauAcc state
	info->tau_acc = params.tauAcc;
	info->motion_intensity = motion_intensity;
#endif
#if IS_ENABLED(CONFIG_VQF_SOFT_MBE_GATE)
	// Soft MBE gate state
	info->soft_mbe_scale = soft_mbe_scale;
	info->soft_mbe_quasi_t = soft_mbe_quasi_t;
	info->soft_mbe_margin_t = soft_mbe_margin_t;
	info->soft_mbe_gyr_norm = soft_mbe_gyr_norm_lp * 180.0f / M_PI;
	info->soft_mbe_acc_dev = soft_mbe_acc_dev_lp / CONST_EARTH_GRAVITY;
	info->soft_mbe_lin_acc = soft_mbe_lin_acc_lp / CONST_EARTH_GRAVITY;
#endif

	// Rest detection diagnostics
	info->rest_enter_count = rest_enter_count;
	info->rest_exit_count = rest_exit_count;
	info->rest_total_s = rest_total_s;
	info->rest_last_duration_s = rest_last_duration_s;
	info->uptime_s = uptime_s;

	// Copy rest event log (oldest first)
	info->rest_event_count = rest_event_total;
	for (uint8_t i = 0; i < REST_EVENT_LOG_SIZE; i++) {
		uint8_t src;
		if (rest_event_total >= REST_EVENT_LOG_SIZE)
			src = (rest_event_idx + i) % REST_EVENT_LOG_SIZE;
		else
			src = i;
		info->rest_events[i].time_s = rest_event_log[src].time_s;
		info->rest_events[i].entered = rest_event_log[src].entered;
	}

	// Kalman filter P diagonal
	info->biasP[0] = state.biasP[0];
	info->biasP[1] = state.biasP[4];
	info->biasP[2] = state.biasP[8];
}

static ALWAYS_INLINE void vqf_bench_timer_prepare(void)
{
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

static ALWAYS_INLINE uint32_t vqf_bench_timer_begin(unsigned int *irq_key)
{
	*irq_key = irq_lock();
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	DWT->CYCCNT = 0;
	return 0;
#else
	return k_cycle_get_32();
#endif
}

static ALWAYS_INLINE uint32_t vqf_bench_timer_end(unsigned int irq_key, uint32_t start_cycles)
{
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	uint32_t elapsed_cycles = DWT->CYCCNT;
	irq_unlock(irq_key);
	return elapsed_cycles;
#else
	uint32_t elapsed_cycles = k_cycle_get_32() - start_cycles;
	irq_unlock(irq_key);
	return elapsed_cycles;
#endif
}

static ALWAYS_INLINE uint32_t vqf_bench_timer_hz(void)
{
#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
	return SystemCoreClock ? SystemCoreClock : CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
#else
	return CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
#endif
}

static void vqf_bench_print_stats(const char *name, uint32_t iterations, uint32_t total_cycles,
				  uint32_t timer_hz)
{
	uint32_t avg_cycles_int = 0;
	uint32_t avg_cycles_frac = 0;
	uint32_t total_us_int = 0;
	uint32_t total_us_frac = 0;
	uint32_t avg_us_int = 0;
	uint32_t avg_us_frac = 0;

	if (iterations) {
		uint64_t avg_cycles_x1000 = (total_cycles * 1000ULL) / iterations;
		avg_cycles_int = (uint32_t)(avg_cycles_x1000 / 1000ULL);
		avg_cycles_frac = (uint32_t)(avg_cycles_x1000 % 1000ULL);
	}

	if (timer_hz) {
		uint64_t total_us_x1000 = (total_cycles * 1000000000ULL) / timer_hz;
		total_us_int = (uint32_t)(total_us_x1000 / 1000ULL);
		total_us_frac = (uint32_t)(total_us_x1000 % 1000ULL);

		if (iterations) {
			uint64_t avg_us_x1000 = total_us_x1000 / iterations;
			avg_us_int = (uint32_t)(avg_us_x1000 / 1000ULL);
			avg_us_frac = (uint32_t)(avg_us_x1000 % 1000ULL);
		}
	}

	printk(
		"  %-10s t_c=%10u avg_c=%4u.%03u t_us=%8u.%03u avg_us=%4u.%03u\n",
		name,
		total_cycles,
		avg_cycles_int,
		avg_cycles_frac,
		total_us_int,
		total_us_frac,
		avg_us_int,
		avg_us_frac
	);
}

#define VQF_BENCH_BATCH_SIZE 16U
#define VQF_BENCH_THREAD_PRIO 8

typedef enum {
	VQF_BENCH_UPDATE_GYR,
	VQF_BENCH_UPDATE_ACC,
	VQF_BENCH_UPDATE_MAG,
	VQF_BENCH_GET_QUAT9D,
} vqf_bench_op_t;

static uint32_t vqf_bench_measure(vqf_bench_op_t op, uint32_t iterations,
				  const vqf_real_t gyr_samples[][3],
				  const vqf_real_t acc_samples[][3],
				  const vqf_real_t mag_samples[][3], size_t sample_count,
				  vqf_params_t *bench_params, vqf_state_t *bench_state,
				  vqf_coeffs_t *bench_coeffs, float quat[4])
{
	uint32_t total_cycles = 0;
	uint32_t remaining = iterations;
	uint32_t sample_offset = 0;

	while (remaining > 0) {
		uint32_t batch_len = remaining > VQF_BENCH_BATCH_SIZE ? VQF_BENCH_BATCH_SIZE : remaining;
		unsigned int irq_key;
		uint32_t start_cycles = vqf_bench_timer_begin(&irq_key);

		for (uint32_t i = 0; i < batch_len; i++, sample_offset++) {
			size_t sample_idx = sample_offset % sample_count;

			switch (op) {
			case VQF_BENCH_UPDATE_GYR:
				updateGyr(bench_params, bench_state, bench_coeffs, gyr_samples[sample_idx]);
				break;
			case VQF_BENCH_UPDATE_ACC:
				updateAcc(bench_params, bench_state, bench_coeffs, acc_samples[sample_idx]);
				break;
			case VQF_BENCH_UPDATE_MAG:
				updateMag(bench_params, bench_state, bench_coeffs, mag_samples[sample_idx]);
				break;
			case VQF_BENCH_GET_QUAT9D:
				getQuat9D(bench_state, quat);
				vqf_bench_sink += quat[sample_offset & 3U];
				break;
			}
		}

		total_cycles += vqf_bench_timer_end(irq_key, start_cycles);
		remaining -= batch_len;
	}

	return total_cycles;
}

void vqf_run_benchmark(uint32_t iterations)
{
	vqf_bench_sink = 0.0f;
	uint32_t timer_hz;
	uint32_t elapsed_cycles;
	k_tid_t bench_thread = k_current_get();
	int bench_thread_prio = k_thread_priority_get(bench_thread);
	bool bench_prio_changed = false;

	if (iterations == 0) {
		iterations = 1000;
	}

	if (bench_thread_prio < VQF_BENCH_THREAD_PRIO) {
		k_thread_priority_set(bench_thread, VQF_BENCH_THREAD_PRIO);
		bench_prio_changed = true;
	}

	set_params();
	vqf_bench_params = params;
	initVqf(&vqf_bench_params, &vqf_bench_state, &vqf_bench_coeffs, 0.001f, 0.001f, 0.01f);

	for (size_t i = 0; i < vqf_bench_sample_count * 8; i++) {
		const size_t idx = i % vqf_bench_sample_count;
		updateGyr(&vqf_bench_params, &vqf_bench_state, &vqf_bench_coeffs, vqf_bench_gyr_samples[idx]);
		updateAcc(&vqf_bench_params, &vqf_bench_state, &vqf_bench_coeffs, vqf_bench_acc_samples[idx]);
		updateMag(&vqf_bench_params, &vqf_bench_state, &vqf_bench_coeffs, vqf_bench_mag_samples[idx]);
		getQuat9D(&vqf_bench_state, vqf_bench_quat);
		vqf_bench_sink += vqf_bench_quat[0];
	}

	vqf_bench_warm_params = vqf_bench_params;
	vqf_bench_warm_state = vqf_bench_state;
	vqf_bench_warm_coeffs = vqf_bench_coeffs;
	vqf_bench_timer_prepare();
	timer_hz = vqf_bench_timer_hz();

	printk("VQF benchmark (%u iterations, CMSIS-DSP=%s, timer=%s)\n", iterations,
	#ifdef CONFIG_CMSIS_DSP
			"on"
	#else
			"off"
	#endif
			,
	#if defined(CONFIG_CPU_CORTEX_M_HAS_DWT)
			"DWT CYCCNT"
	#else
			"system timer"
	#endif
		);

	vqf_bench_params = vqf_bench_warm_params;
	vqf_bench_state = vqf_bench_warm_state;
	vqf_bench_coeffs = vqf_bench_warm_coeffs;
	elapsed_cycles = vqf_bench_measure(VQF_BENCH_UPDATE_GYR, iterations, vqf_bench_gyr_samples,
				       vqf_bench_acc_samples, vqf_bench_mag_samples,
				       vqf_bench_sample_count, &vqf_bench_params,
				       &vqf_bench_state, &vqf_bench_coeffs, vqf_bench_quat);
	vqf_bench_print_stats("updateGyr", iterations, elapsed_cycles, timer_hz);

	vqf_bench_params = vqf_bench_warm_params;
	vqf_bench_state = vqf_bench_warm_state;
	vqf_bench_coeffs = vqf_bench_warm_coeffs;
	elapsed_cycles = vqf_bench_measure(VQF_BENCH_UPDATE_ACC, iterations, vqf_bench_gyr_samples,
				       vqf_bench_acc_samples, vqf_bench_mag_samples,
				       vqf_bench_sample_count, &vqf_bench_params,
				       &vqf_bench_state, &vqf_bench_coeffs, vqf_bench_quat);
	vqf_bench_print_stats("updateAcc", iterations, elapsed_cycles, timer_hz);

	vqf_bench_params = vqf_bench_warm_params;
	vqf_bench_state = vqf_bench_warm_state;
	vqf_bench_coeffs = vqf_bench_warm_coeffs;
	elapsed_cycles = vqf_bench_measure(VQF_BENCH_UPDATE_MAG, iterations, vqf_bench_gyr_samples,
				       vqf_bench_acc_samples, vqf_bench_mag_samples,
				       vqf_bench_sample_count, &vqf_bench_params,
				       &vqf_bench_state, &vqf_bench_coeffs, vqf_bench_quat);
	vqf_bench_print_stats("updateMag", iterations, elapsed_cycles, timer_hz);

	vqf_bench_params = vqf_bench_warm_params;
	vqf_bench_state = vqf_bench_warm_state;
	vqf_bench_coeffs = vqf_bench_warm_coeffs;
	elapsed_cycles = vqf_bench_measure(VQF_BENCH_GET_QUAT9D, iterations, vqf_bench_gyr_samples,
				       vqf_bench_acc_samples, vqf_bench_mag_samples,
				       vqf_bench_sample_count, &vqf_bench_params,
				       &vqf_bench_state, &vqf_bench_coeffs, vqf_bench_quat);
	vqf_bench_print_stats("getQuat9D", iterations, elapsed_cycles, timer_hz);

	printk("  checksum: %.6f\n", (double)vqf_bench_sink);

	if (bench_prio_changed) {
		k_thread_priority_set(bench_thread, bench_thread_prio);
	}
}

const sensor_fusion_t sensor_fusion_vqf = {
	vqf_init,
	vqf_load,
	vqf_save,

	vqf_update_gyro,
	vqf_update_accel,
	vqf_update_mag,
	vqf_update,

	vqf_get_gyro_bias,
	vqf_set_gyro_bias,

	vqf_update_gyro_sanity,
	vqf_get_gyro_sanity,

	vqf_get_lin_a,
	vqf_get_quat
};

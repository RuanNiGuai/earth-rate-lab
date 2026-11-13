#include "f04_summary.h"
#include "calwire.h"
#include "calwire_protocol.h"
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
#include "globals.h"
#include "sensor.h"
#include "connection/connection.h"
#include "tcal_defer.h"
#include "system/system.h"
#include "system/watchdog.h"
#include "system/led.h"
#include "util.h"

#include <math.h>
#include <stdlib.h>

#if CONFIG_CMSIS_DSP
#include <arm_math.h>
#endif

#include "sensors_enum.h"
#include "magneto/magneto1_4.h"
#include "imu/BMI270.h"
#if CONFIG_SENSOR_USE_VQF
#include "fusion/vqf/vqf.h"
#endif

#include "calibration.h"
#include "sensor/cal_archive_safe.h"
#include "field_calibration.h"
#include "gyro_scale.h"
#include "imu/ICM45686.h"
#include <zephyr/sys/atomic.h>

static uint8_t imu_id;
static float field_accel[3];
static int64_t field_accel_time = -1000;
static uint8_t sensor_data[128]; // any use sensor data

static float accelBias[3] = {0}, gyroBias[3] = {0}, magBias[3] = {0}; // offset biases

/* Boot zero-rate offset seeding for the fusion (see process_gyro). */
static bool boot_zro_pending = true;
static int boot_zro_count;
static double boot_zro_sum[3];

#if CONFIG_SENSOR_USE_TCAL
static atomic_t gyro_gsens_request;
static float requested_gyro_gsens[9];
static void sensor_gyro_gsens_apply(bool rest);
#endif

static float accBAinv[4][3];
static float magBAinv[4][3];

static uint8_t magneto_progress;
static uint8_t last_magneto_progress;
static int64_t magneto_progress_time;

// Only trust orientation updates when accel magnitude stays reasonably close to 1 g.
// This rejects samples collected during strong linear acceleration while keeping
// normal hand-rotation usable for calibration.
#define MAG_CAL_ACCEL_MAG_MIN_SQ 0.75f
#define MAG_CAL_ACCEL_MAG_MAX_SQ 1.3f

// Minimum samples before attempting trial calibration
#define MAG_CAL_MIN_SAMPLES 65
// Attempt trial calibration every this many new samples (manual cal)
#define MAG_CAL_TRIAL_INTERVAL 80

static int64_t mag_cal_last_status_log;

static double ata[100]; // manual calibration accumulator
static double norm_sum;
static double sample_count;
// Direction diversity tracking: sum of normalized mag directions
static float dir_sum[3];
// Direction range tracking for manual calibration: per-axis min/max of normalized direction
static float dir_min[3];
static float dir_max[3];
// Minimum direction range per axis for accepting manual calibration
// 0.5 ≈ 30° arc on each axis; requires meaningful rotation around at least 2 axes
#define MAG_CAL_MIN_DIR_RANGE 0.5f

// Per-quadrant ring buffer for online magnetometer calibration.
// Combines the directional coverage guarantee of quadrant-based sampling
// (8 octants based on sign of x, y, z) with the natural aging of FIFO
// per-quadrant sliding windows. Each octant independently wraps after
// QUADRANT_BUF_SIZE samples — staying in one orientation only updates
// that octant, leaving the other 7 with diverse data.
// 8 × 16 = 128 samples total (matching magcal's proven size),
// ~1.5KB vs 3.3KB for the old 4×80 segment design.
#define QUADRANT_BUF_SIZE 16
#define ONLINE_QUADRANT_COUNT 8

typedef struct {
	float x, y, z;
} quadrant_sample_t;

typedef struct {
	quadrant_sample_t samples[QUADRANT_BUF_SIZE];
	uint8_t head;   // next write position
	uint8_t count;  // valid samples (0..QUADRANT_BUF_SIZE)
	int64_t last_seq; // global accepted-sample sequence of the newest sample in this octant
} quadrant_buf_t;

// Incremental calibration blending (EMA on BAinv elements)
// Base alpha: weight given to new trial calibration. A value of 0.35 means
// 35% new + 65% existing → gradual convergence over ~3 updates.
// Higher when trial diverges significantly from existing (environment change).
#define ONLINE_BLEND_BASE_ALPHA 0.35f
#define ONLINE_BLEND_MIN_ALPHA 0.12f   // floor: very similar calibrations
#define ONLINE_BLEND_MAX_ALPHA 0.70f   // ceiling: significant divergence detected
#define ONLINE_BLEND_SIMILARITY_LOW 0.85f   // below this similarity, increase alpha
#define ONLINE_BLEND_SIMILARITY_HIGH 0.97f  // above this similarity, use min alpha

static quadrant_buf_t quad_buf[ONLINE_QUADRANT_COUNT];
static int64_t online_total_sample_count;
static int64_t online_last_checked_sample_count;
static int64_t online_last_check_time;
static int64_t online_last_sample_time; // rate limiting
// Drop octants that have not been refreshed for too long.
// This is kept separate from the check cadence: stale-history rejection should
// not depend on how often the background thread decides to run Magneto.
#define ONLINE_STALE_QUADRANT_MAX_AGE 80
// Minimum direction change to accept an online sample. The configured value is
// expressed in degrees and converted to the equivalent 1 - cos(theta) threshold.
static float online_last_dir[3];
static float online_last_accel_dir[3]; // accel direction for cross-validation

// Manual calibration direction tracking (same cross-validation logic as online path)
static float manual_last_dir[3];
static float manual_last_accel_dir[3];

#define ONLINE_MIN_DIR_CHANGE_DEG 10.0f
#define ONLINE_MIN_INTERVAL_MS 30  // minimum 30ms between online samples
// Background checks should not run on every calibration-thread pass.
// Tie the minimum check spacing to roughly one fresh fit's worth of accepted
// samples at the maximum online sampling rate.
#define ONLINE_MIN_CHECK_INTERVAL_MS (MAG_CAL_MIN_SAMPLES * ONLINE_MIN_INTERVAL_MS * 2)

// Runtime calibrated norm tracking (exponential moving average)
// Used to assess current calibration quality and decide if online update is needed
static float cal_norm_ema;        // EMA of calibrated mag norm
static float cal_norm_var_ema;    // EMA of squared deviation from mean
static uint32_t cal_norm_count;   // number of norm samples processed
#define CAL_NORM_EMA_ALPHA 0.01f  // smoothing factor (~100 sample window)
// Don't update calibration if current norm CV is below this threshold
#define CAL_NORM_GOOD_CV 0.07f    // 7% = good enough calibration

// Minimum time between online calibration updates (prevents frequent VQF mag ref resets)
#define ONLINE_MIN_UPDATE_INTERVAL_S 6  // 6 seconds cooldown
static int64_t online_last_update_time;

// Require at least N successful calibration updates before trusting the
// norm CV gate. Prevents a single early fit from being declared "good enough"
// when the buffer is still filling and directional coverage is incomplete.
#define ONLINE_MIN_UPDATES 3
static int online_update_count;

// Norm-change detection: when the average raw field strength in the buffer
// changes by >40% between updates, the magnetic environment has changed.
// We clear buffers to avoid mixed-data fits and let the next cycle fit
// on consistent data from the new environment.
static float online_last_buf_avg_norm = 0.0f;

static void magneto_online_runtime_reset(void);
static void magneto_online_runtime_load_retained(void);

// #define DEBUG true

#if DEBUG
LOG_MODULE_REGISTER(calibration, LOG_LEVEL_DBG);
#else
LOG_MODULE_REGISTER(calibration, LOG_LEVEL_INF);
#endif

static void sensor_sample_accel(const float a[3]);
static int sensor_wait_accel(float a[3], k_timeout_t timeout);


static void sensor_sample_mag(const float m[3]);
static int sensor_wait_mag(float m[3], k_timeout_t timeout);


#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static void sensor_calibrate_6_side(void);
#endif
static int sensor_calibrate_mag(void);

// =============================================================================
// Temperature-table configuration
// =============================================================================
#if CONFIG_SENSOR_USE_TCAL

#define TEMP_TO_IDX(temp) (int)((((float)temp) - CONFIG_SENSOR_POLY_TEMP_MIN) * CONFIG_SENSOR_POLY_STEPS_PER_DEGREE)
#define IDX_TO_TEMP(idx) (float)(((float)(idx) / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE) + CONFIG_SENSOR_POLY_TEMP_MIN)

#define TCAL_MIN_CURVE_POINTS 4

// Auto-calibration control
static bool tcal_auto_calibration_enabled = false;

// T-Cal compensation control (persisted via NVS)
static bool tcal_compensation_enabled = true;

// Temperature direction tracking for hysteresis-aware blending
// Tracks the direction of temperature change at the time of each calibration sample
typedef enum {
	TCAL_DIR_UNKNOWN = 0,
	TCAL_DIR_RISING,
	TCAL_DIR_FALLING,
} tcal_temp_direction_t;

static tcal_temp_direction_t tcal_current_direction = TCAL_DIR_UNKNOWN;
static float tcal_direction_ref_temp = NAN; // Reference temperature for direction detection

// Hysteresis blending EMA factors (new = alpha * measured + (1-alpha) * existing)
// Physically, IMU bias is more stable during warm-up (rising) than during cooling,
// so rising-direction measurements are given higher trust.
#define TCAL_HYSTERESIS_EMA_RISING  0.7f // Rising temp: favor new measurement
#define TCAL_HYSTERESIS_EMA_FALLING 0.3f // Falling temp: preserve warm-up data
#define TCAL_HYSTERESIS_EMA_UNKNOWN 0.5f // Unknown direction: equal blend

// Minimum bias change (dps) required to trigger a flash write.
// Skips NVS/LUT rebuild when successive flushes of the same bucket produce
// negligible change, preventing periodic flash wear during stable rest.
#define TCAL_SAVE_SIGNIFICANCE_THRESHOLD 0.002f

/* Disagreement (dps) above which an existing point is replaced, not blended. */
#define TCAL_POINT_REPLACE_THRESHOLD 0.02f
// =============================================================================
// Continuous Accumulator-Based T-Cal Sampling
// =============================================================================
// A single accumulator continuously collects gyro+temperature samples while
// the device is resting. Periodically (every ~20 seconds), the accumulator is
// flushed: the averaged bias is saved to the appropriate temperature bucket.
//
// If temperature drifts more than one bucket width during accumulation, an
// early flush is triggered to avoid cross-bucket contamination.
//
// Benefits over fixed-time one-shot sampling:
// - No temperature gap can be skipped regardless of change rate
// - No temperature change invalidation (each flush is short enough)
// - Measured mean temperature stored with its corresponding bias
// - Low overhead: just sum accumulation, no per-sample bucket lookup

// Flush interval: accumulator is committed every ~25 seconds
#define TCAL_ACCUM_FLUSH_INTERVAL_MS 25000

// Minimum samples required for a valid flush
#define TCAL_ACCUM_MIN_SAMPLES 1250

// Maximum temperature drift within one accumulation window before early flush.
// Slightly larger than bucket width (0.25°C) to allow full bucket coverage
// while preventing cross-bucket contamination.
#define TCAL_ACCUM_TEMP_DRIFT_MAX 0.11f

// A sample deviating more than this from the window mean is skipped (one glitch
// must not discard the whole window). Only this many consecutive outliers are
// treated as real motion and reset the accumulator.
#define TCAL_ACCUM_OUTLIER_DEV 2.0f
#define TCAL_ACCUM_OUTLIER_RESET 20

static struct {
	double gyro_sum[3];    // Accumulated raw gyro values
	double temp_sum;       // Accumulated temperature values
	int sample_count;      // Number of gyro samples
	int temp_count;        // Number of valid temperature samples
	float min_g[3];        // Gyro min per axis for diagnostics
	float max_g[3];        // Gyro max per axis for diagnostics
	float temp_min;        // Temperature min during accumulation
	float temp_max;        // Temperature max during accumulation
	int outlier_count;     // Consecutive outlier samples
	bool active;           // Whether accumulator is currently collecting
	int64_t start_time;    // Timestamp when accumulation started
} tcal_accum;

static int64_t tcal_accum_last_commit_time = 0;

// Forward declarations
static void tcal_accum_reset(void);
static void tcal_accum_flush(void);
static void tcal_save_point(int idx, float measured_temp, const float bias[3]);

static float last_gyro_tcal_offset[3] = {0.0f, 0.0f, 0.0f};

static void update_tcal_state(void); // Function to refresh T-Cal state

/* A large new-vs-old disagreement is only accepted after this many consistent
 * new windows, so noise or a bad window cannot overwrite a good stored point. */
#define TCAL_REPLACE_CONFIRM_COUNT 3
#define TCAL_REPLACE_AGREE_TOL 0.010f
static struct {
	int idx;
	uint8_t count;
	float bias[3];
} tcal_pending_replace = {.idx = -1};

/* Auto policy: collect while powered and resting, then verify at a low rate. */
#define TCAL_VERIFY_INTERVAL_MS (5 * 60 * 1000)
/* Observation writes are opt-in on every boot, including retained/NVS loads.
 * Applying an existing temperature table remains a separate persisted switch. */
static bool tcal_user_off = true; /* only an explicit "auto on" arms collection */
static bool tcal_verify_mode;   /* one-window verification runs */
static int64_t tcal_verify_next_ms;
/* A1: deferral policy state (logic in tcal_defer.c, unit tested on the host). */
static tcal_defer_state_t tcal_defer;
static tcal_action_t tcal_last_action;

// =============================================================================
// T-Cal Moving Least Squares (MLS) Implementation
// =============================================================================
// MLS provides smooth, continuous bias estimation without discontinuities
// caused by method switching. Uses inverse distance weighting with local
// linear fitting for optimal balance of smoothness and responsiveness.

// MLS Configuration
#define MLS_BANDWIDTH 2.5f       // Temperature bandwidth (°C) - controls locality
#define MLS_MIN_WEIGHT 0.03f     // Minimum weight threshold (~6.5°C distance cutoff)
#define MLS_MAX_POINTS 10        // Maximum points to consider for efficiency
#define MLS_MIN_POINTS_FOR_FIT 4 // Minimum points with significant weight for MLS to be valid
#define MLS_EXTRAP_POINTS 4      // Number of edge points for linear extrapolation (matches LUT)

/**
 * Moving Least Squares (MLS) lookup function
 * Computes weighted local linear fit at the query temperature
 *
 * For 1D linear MLS, we solve the weighted least squares problem:
 *   minimize Σ w_i * (b_i - (a + c*(t_i - t_query)))²
 * where w_i is the weight for point i, inversely related to distance
 *
 * The solution is:
 *   c = Σ w_i*(t_i-t_q)*(b_i-b_weighted_avg) / Σ w_i*(t_i-t_q)²
 *   a = b_weighted_avg
 *
 * @param temp Query temperature
 * @param bias_out Output: computed 3-axis bias
 * @return 0 on success, -1 if insufficient data
 */
static int sensor_tcal_mls_lookup(float temp, float bias_out[3]);

// =============================================================================
// MLS Cache - Performance Optimization
// =============================================================================
// Since temperature changes slowly (over seconds), we can cache the MLS result
// and only recompute when temperature changes significantly.
// Temperature sensor noise can be ~0.1°C, so we use a larger threshold
// to avoid unnecessary recomputation.
//
// We use multiple cache slots to cover a larger temperature range, which helps
// when temperature oscillates slightly within a small range.

// =============================================================================
// LUT (Look-Up Table) + Linear Interpolation - O(1) Runtime Lookup
// =============================================================================
// When calibration points change, pre-compute MLS output at fixed temperature
// grid points. Runtime lookup simply does linear interpolation between two
// adjacent grid points, achieving O(1) complexity without expensive MLS
// computation per query.
//
// LUT Configuration:
// - Step size: 0.5°C (2 steps per degree) - good balance of precision vs RAM
// - Temperature range: CONFIG_SENSOR_POLY_TEMP_MIN to CONFIG_SENSOR_POLY_TEMP_MAX
// - RAM usage: ~0.9KB for standard 10-45°C range
//
// Incremental Build Strategy:
// - At boot, first build entries within ±2°C of current temperature (priority zone)
// - Return quickly to allow other threads to run
// - Continue building remaining entries in small batches during idle time
// - LUT lookup falls back to MLS for entries not yet computed

#define MLS_LUT_STEP_PER_DEGREE 4   // Steps per degree (0.25°C per step)
#define MLS_LUT_STEP_SIZE (1.0f / MLS_LUT_STEP_PER_DEGREE)  // 0.25°C
#define MLS_LUT_TEMP_MIN ((float)CONFIG_SENSOR_POLY_TEMP_MIN)
#define MLS_LUT_TEMP_MAX ((float)CONFIG_SENSOR_POLY_TEMP_MAX)
#define MLS_LUT_SIZE ((int)((CONFIG_SENSOR_POLY_TEMP_MAX - CONFIG_SENSOR_POLY_TEMP_MIN) * MLS_LUT_STEP_PER_DEGREE) + 1)

// Incremental build configuration
#define MLS_LUT_PRIORITY_RANGE 2.0f  // ±2°C around current temp is priority zone
#define MLS_LUT_BATCH_SIZE 8        // Entries to compute per incremental batch
#define MLS_LUT_BATCH_YIELD_MS 10     // Sleep between batches to yield CPU

// Convert temperature to LUT index (continuous, for interpolation)
#define MLS_LUT_TEMP_TO_IDX(temp) (((temp) - MLS_LUT_TEMP_MIN) * MLS_LUT_STEP_PER_DEGREE)
// Convert LUT index to temperature
#define MLS_LUT_IDX_TO_TEMP(idx) (MLS_LUT_TEMP_MIN + (float)(idx) * MLS_LUT_STEP_SIZE)

typedef struct {
	float bias[3];  // Pre-computed MLS bias at this temperature
	bool computed;  // Whether this entry has been computed
} MlsLutEntry;

// LUT build state
typedef enum {
	MLS_LUT_BUILD_IDLE,         // No build in progress, LUT may be invalid or complete
	MLS_LUT_BUILD_PRIORITY,     // Building priority zone (±2°C around current temp)
	MLS_LUT_BUILD_BACKGROUND,   // Building remaining entries in background
	MLS_LUT_BUILD_COMPLETE      // All entries computed
} MlsLutBuildState;

static struct {
	MlsLutEntry entries[MLS_LUT_SIZE]; // Pre-computed bias values
	uint32_t version;                   // Point count when LUT was built (for invalidation)
	bool valid;                         // LUT has at least priority zone computed
	MlsLutBuildState build_state;       // Current build state
	int build_next_idx;                 // Next index to compute in background build
	int priority_idx_min;               // Priority zone minimum index
	int priority_idx_max;               // Priority zone maximum index
	int computed_count;                 // Number of entries computed so far
} mls_lut = {
	.entries = {{{0}}},
	.version = 0,
	.valid = false,
	.build_state = MLS_LUT_BUILD_IDLE,
	.build_next_idx = 0,
	.priority_idx_min = 0,
	.priority_idx_max = 0,
	.computed_count = 0
};

// Forward declarations for LUT functions
static void sensor_tcal_build_lut_priority(float current_temp);
static bool sensor_tcal_build_lut_continue(void);
static int sensor_tcal_lut_lookup(float temp, float bias_out[3]);

// =============================================================================
// Legacy MLS Cache (kept for fallback and LUT building)
// =============================================================================

#define MLS_CACHE_SLOTS 5            // Number of cache slots
#define MLS_CACHE_TEMP_THRESHOLD 0.5f // Match within this threshold of cached temp

typedef struct {
	float temp;        // Temperature at which cache was computed
	float bias[3];     // Cached bias values at temp
	float slope[3];    // Local d(bias)/d(temp) slope used for smooth cached interpolation
	bool valid;        // Cache validity flag
} MlsCacheSlot;

static struct {
	MlsCacheSlot slots[MLS_CACHE_SLOTS]; // Cache slots covering temperature range
	uint32_t count;                       // Point count when cached (invalidate all if points change)
} mls_cache = {
	.slots = {{0}},
	.count = 0
};

/**
 * Select the best cache slot to use for a new entry at the given temperature.
 * Strategy:
 * 1. If an invalid slot exists, use it
 * 2. Find the slot with the largest distance from the query temperature
 *    (this preserves nearby cached values for interpolation)
 * @param temp Query temperature for the new cache entry
 * @return Best slot index to use
 */
static int sensor_tcal_cache_select_slot(float temp)
{
	int best_slot = 0;
	float best_distance = -1.0f;

	for (int i = 0; i < MLS_CACHE_SLOTS; i++) {
		// Prefer invalid slots first
		if (!mls_cache.slots[i].valid) {
			return i;
		}
		// Find slot with largest distance from query temperature
		float distance = fabsf(mls_cache.slots[i].temp - temp);
		if (distance > best_distance) {
			best_distance = distance;
			best_slot = i;
		}
	}
	return best_slot;
}

// =============================================================================
// T-Cal Cache/LUT Invalidation (called when calibration points change)
// =============================================================================
static void sensor_tcal_cache_invalidate(void)
{
	// Invalidate legacy cache slots
	for (int i = 0; i < MLS_CACHE_SLOTS; i++) {
		mls_cache.slots[i].valid = false;
	}
	// Invalidate LUT and stop any incremental build in progress
	mls_lut.valid = false;
	mls_lut.build_state = MLS_LUT_BUILD_IDLE;
	mls_lut.computed_count = 0;
	// Mark all entries as not computed
	for (int i = 0; i < MLS_LUT_SIZE; i++) {
		mls_lut.entries[i].computed = false;
	}
	LOG_DBG("T-Cal cache/LUT invalidated, incremental build stopped");
}

#endif

// helpers
static void magneto_reset(void);
static void magneto_online_clear_history(void);
static void magneto_online_reset(void);
static double magneto_online_collect_recent(double ata_out[100], double *norm_sum_out, float dir_sum_out[3]);
static int magneto_online_recent_sample_count(void);
static float magneto_online_recent_dir_bias(void);
static float magneto_online_min_dir_change_threshold(void);
static float magneto_directional_bias(const float ds[3], double count);
static void magneto_accumulate_direction(float ds[3], float mx, float my, float mz);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static int isAccRest(float *, float *, float, int *, int);
#endif

// calibration logic
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static int sensor_6_sideBias(float a_inv[][3], int *captured_count_out);
#endif
static void sensor_sample_mag_magneto_sample(const float m[3]);

static int sensor_calibration_request(int id);

static void calibration_thread(void);
// Keep background calibration below the sensor loop so trial Magneto solves do
// not preempt FIFO servicing. This makes online/manual calibration a little less
// eager, but avoids sensor-loop timing regressions from background work.
K_THREAD_DEFINE(calibration_thread_id, 4096, calibration_thread, NULL, NULL, NULL, 8, 0, 0);

bool sensor_calibration_wire_idle(void)
{
    return sensor_is_initialized() && sensor_calibration_request(0)==0;
}

bool sensor_calibration_accel_snapshot(float out[4][3])
{
    unsigned key=irq_lock();
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
    bool ok=!memcmp(accBAinv,retained->accBAinv,sizeof(accBAinv));
    memcpy(out,accBAinv,sizeof(accBAinv));
#else
    bool ok=true;
    memset(out,0,sizeof(accBAinv));
    for(int i=0;i<3;i++) out[i+1][i]=1;
#endif
    irq_unlock(key);
    return ok;
}

void sensor_calibration_process_accel(float a[3])
{
	sensor_sample_accel(a);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	apply_BAinv(a, accBAinv);
#else
	// In single-side calibration mode, accelBias should be zero.
	// Single-side bias is orientation-dependent and should not be applied.
	// for (int i = 0; i < 3; i++) {
	// 	a[i] -= accelBias[i];
	// }
#endif
    memcpy(field_accel, a, sizeof(field_accel));
    field_accel_time = k_uptime_get();
}

bool sensor_boot_zro_ready(void)
{
#if CONFIG_SENSOR_USE_TCAL
    return !boot_zro_pending;
#else
    return true;
#endif
}

/* Seed in the same scaled deg/s units consumed by the fusion. */
static void sensor_boot_zro_feed(const float g[3], bool rest)
{
	if (boot_zro_pending) {
		if (sys_is_wom_resume()) {
			boot_zro_pending = false;
		} else if (rest && sqrtf(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]) < 0.5f) {
			for (int i = 0; i < 3; i++) {
				boot_zro_sum[i] += (double)g[i] * (double)retained->gyroSensScale[i];
			}
			boot_zro_count++;
			if (boot_zro_count >= 800) {
				float bias[3];
				for (int i = 0; i < 3; i++) {
					bias[i] = (float)(boot_zro_sum[i] / (double)boot_zro_count);
				}
				sensor_fusion_update_bias(bias);
				boot_zro_pending = false;
				printk("Boot ZRO: bias %.3f %.3f %.3f dps\n",
				       (double)bias[0], (double)bias[1], (double)bias[2]);
			}
		} else {
			boot_zro_count = 0;
			boot_zro_sum[0] = boot_zro_sum[1] = boot_zro_sum[2] = 0.0;
		}
	}

}

void sensor_earth_pose_save_poll(void);
void sensor_calibration_process_gyro(float g[3], float dt, bool rest)
{
    calwire_sensor_poll(rest);
    sensor_earth_pose_save_poll();
    /* Archive only raw input before all bias, g-sens, or scale corrections. */
    if (IS_ENABLED(CONFIG_SENSOR_USE_VQF)) {
        cal_archive_feed_raw(g, sensor_get_calibration_temperature(), dt, rest);
    }
#if CONFIG_SENSOR_USE_TCAL
    sensor_gyro_gsens_apply(rest);
	float calculated_offset[3] = {0.0f, 0.0f, 0.0f};
	float temp = sensor_get_current_imu_temperature();
	bool offset_calculated = false;

	/* The LUT lives in RAM and must be rebuilt after every boot. The startup
	 * build can be skipped when the IMU initializes late; retry lazily here. */
	if (!mls_lut.valid && mls_lut.build_state == MLS_LUT_BUILD_IDLE &&
	    retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT && isfinite(temp)) {
		sensor_tcal_build_lut_priority(temp);
	}

	// Feed raw gyro data into continuous bucket accumulator for auto T-Cal
	// This must happen before bias subtraction so we capture the true raw bias
	// (auto-cal collection continues regardless of compensation enable state)
	if (tcal_auto_calibration_enabled) {
		sensor_tcal_feed_continuous_sample(g, sensor_get_calibration_temperature(), rest);
	}

	// ==========================================================================
	// Unified T-Cal Strategy: LUT -> MLS -> Static Bias
	// Skipped when tcal_compensation_enabled is false (uses static bias instead)
	// ==========================================================================

	if (tcal_compensation_enabled && isfinite(temp) && retained->tempCalState.count >= 1) {
		// Strategy 1: Try LUT lookup (preferred - O(1) linear interpolation)
		if (retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT) {
			if (sensor_tcal_lut_lookup(temp, calculated_offset) == 0) {
				offset_calculated = true;
			}
			// Strategy 2: Fallback to MLS if LUT not available
			else if (sensor_tcal_mls_lookup(temp, calculated_offset) == 0) {
				offset_calculated = true;
			}
		}
	}

	// Strategy 2: Final fallback to static bias (when no T-Cal data or compensation disabled)
	if (!offset_calculated) {
		for (int i = 0; i < 3; i++) {
			calculated_offset[i] = gyroBias[i];
		}
	}

	// Apply the calculated offset to gyro data
#if CONFIG_CMSIS_DSP
	arm_sub_f32(g, calculated_offset, g, 3);
#else
	for (int i = 0; i < 3; i++) {
		g[i] -= calculated_offset[i];
	}
#endif

	/* g-sensitivity: b = M * a with the acceleration in g. A zero matrix
	 * (unset) disables the correction. Measured once from static tilts. */
	const float *gsm = retained->gyroGsens;
	if (gsm[0] || gsm[1] || gsm[2] || gsm[3] || gsm[4] ||
	    gsm[5] || gsm[6] || gsm[7] || gsm[8]) {
        if (k_uptime_get()-field_accel_time<=100 &&
            isfinite(field_accel[0]) && isfinite(field_accel[1]) && isfinite(field_accel[2])) {
            float ax = field_accel[0], ay = field_accel[1], az = field_accel[2];
            for (int i = 0; i < 3; i++) {
                g[i] -= gsm[3*i]*ax + gsm[3*i+1]*ay + gsm[3*i+2]*az;
            }
        }
	}

	memcpy(last_gyro_tcal_offset, calculated_offset, sizeof(last_gyro_tcal_offset));

	sensor_boot_zro_feed(g, rest);

	sensor_earth_cal_feed(g, dt, rest);
	sensor_gyro_scale_feed(g, dt, rest);
#else
#if CONFIG_CMSIS_DSP
	arm_sub_f32(g, gyroBias, g, 3);
#else
	for (int i = 0; i < 3; i++) {
		g[i] -= gyroBias[i];
	}
#endif
#endif
}

void sensor_calibration_process_mag(float m[3])
{
	//	for (int i = 0; i < 3; i++)
	//		m[i] -= magBias[i];
	sensor_sample_mag(m);
	apply_BAinv(m, magBAinv);
}

void sensor_calibration_update_sensor_ids(int imu)
{
	imu_id = imu;
}

uint8_t *sensor_calibration_get_sensor_data()
{
	return sensor_data;
}

static void magneto_online_runtime_load_retained(void)
{
	magneto_online_reset();
	online_update_count = retained->onlineMagState.update_count;
	online_last_buf_avg_norm = retained->onlineMagState.last_buf_avg_norm;
	cal_norm_ema = retained->onlineMagState.norm_ema;
	cal_norm_var_ema = retained->onlineMagState.norm_var_ema;
	cal_norm_count = retained->onlineMagState.norm_count;
}

void sensor_calibration_online_mag_retained_save(void)
{
	retained->onlineMagState.update_count = (uint8_t)CLAMP(online_update_count, 0, 255);
	retained->onlineMagState.last_buf_avg_norm = online_last_buf_avg_norm;
	retained->onlineMagState.norm_ema = cal_norm_ema;
	retained->onlineMagState.norm_var_ema = cal_norm_var_ema;
	retained->onlineMagState.norm_count = cal_norm_count;
}

void sensor_calibration_online_mag_retained_clear(void)
{
	memset(&retained->onlineMagState, 0, sizeof(retained->onlineMagState));
}

void sensor_calibration_online_mag_cold_start(void)
{
	magneto_online_runtime_reset();
	sensor_calibration_online_mag_retained_clear();
}

void sensor_calibration_read(void)
{
    sensor_boot_cal_reset();
	memcpy(sensor_data, retained->sensor_data, sizeof(sensor_data));
	memcpy(accelBias, retained->accelBias, sizeof(accelBias));
	memcpy(gyroBias, retained->gyroBias, sizeof(gyroBias));
	memcpy(magBias, retained->magBias, sizeof(magBias));
	memcpy(magBAinv, retained->magBAinv, sizeof(magBAinv));
	memcpy(accBAinv, retained->accBAinv, sizeof(accBAinv));
	{
		float zero[3] = {0};
		if (v_diff_mag(magBAinv[0], zero) != 0) {
			magneto_online_runtime_load_retained();
			if (online_update_count > 0 || cal_norm_count > 0) {
				LOG_INF("Online mag runtime restored (%d updates, %u norm samples)",
				        online_update_count, cal_norm_count);
			}
		} else {
			magneto_online_runtime_reset();
		}
	}
#if CONFIG_SENSOR_USE_TCAL
	tcal_compensation_enabled = retained->tcal_enabled;
	LOG_INF("T-Cal compensation: %s", tcal_compensation_enabled ? "enabled" : "disabled");
#endif
}

int sensor_calibration_validate(float *a_bias, float *g_bias, bool write)
{
	if (a_bias == NULL) {
		a_bias = accelBias;
	}
	if (g_bias == NULL) {
		g_bias = gyroBias;
	}
	float zero[3] = {0};
	if (!v_epsilon(a_bias, zero, 0.5) || !v_epsilon(g_bias, zero, 50.0)) // check accel is <0.5G and gyro <50dps
	{
		sensor_calibration_clear(a_bias, g_bias, write);
		// Validation failure: do NOT call any fusion function
		// Let fusion keep its current bias estimate to avoid residual drift
		LOG_WRN("Invalidated calibration");
		LOG_WRN("The IMU may be damaged or calibration was not completed properly");
		return -1;
	}
	return 0;
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
int sensor_calibration_validate_6_side(float a_inv[][3], bool write)
{
	if (a_inv == NULL) {
		a_inv = accBAinv;
	}
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++) {
		diagonal[i] = a_inv[i + 1][i];
	}
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	if (!v_epsilon(a_inv[0], zero, 0.5)
		|| !v_epsilon(diagonal, average, magnitude * 0.1f)) // check accel is <0.5G and diagonals are within 10%
	{
		sensor_calibration_clear_6_side(a_inv, write);
		LOG_WRN("Invalidated calibration");
		LOG_WRN("The IMU may be damaged or calibration was not completed properly");
		return -1;
	}
	return 0;
}
#endif

int sensor_calibration_validate_mag(float m_inv[][3], bool write)
{
	if (m_inv == NULL) {
		m_inv = magBAinv;
	}
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++) {
		diagonal[i] = m_inv[i + 1][i];
	}
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	if (!v_epsilon(m_inv[0], zero, 1)
		|| !v_epsilon(
			diagonal,
			average,
			MAX(magnitude * 0.2f, 0.1f)
		)) // check offset is <1 unit and diagonals are within 20%
	{
		sensor_calibration_clear_mag(m_inv, write);
		LOG_WRN("Invalidated calibration");
		LOG_WRN("The magnetometer may be damaged or calibration was not completed properly");
		return -1;
	}
	return 0;
}

void sensor_calibration_clear(float *a_bias, float *g_bias, bool write)
{
	if (a_bias == NULL) {
		a_bias = accelBias;
	}
	if (g_bias == NULL) {
		g_bias = gyroBias;
	}
	memset(a_bias, 0, sizeof(accelBias));
	memset(g_bias, 0, sizeof(gyroBias));
	if (write) {
		LOG_INF("Clearing stored calibration data");
		sys_write(MAIN_ACCEL_BIAS_ID, &retained->accelBias, a_bias, sizeof(accelBias));
		sys_write(MAIN_GYRO_BIAS_ID, &retained->gyroBias, g_bias, sizeof(gyroBias));
#if CONFIG_SENSOR_USE_TCAL
		// Also clear boot/runtime calibration D_offset since ZRO is being reset
		retained->bootCalState.doffset_valid = false;
		retained->bootCalState.doffset[0] = 0.0f;
		retained->bootCalState.doffset[1] = 0.0f;
		retained->bootCalState.doffset[2] = 0.0f;
		LOG_INF("Clearing D_offset along with ZRO calibration");
#endif
		// Note: Caller is responsible for calling sensor_fusion_update_bias() or
		// sensor_fusion_invalidate() as appropriate:
		// - sensor_fusion_update_bias(): for internal/automatic calibration (preserves quaternion)
		// - sensor_fusion_invalidate(): for manual reset commands (resets quaternion)
	}
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
void sensor_calibration_clear_6_side(float a_inv[][3], bool write)
{
    if(write && calwire_active()) return;
	if (a_inv == NULL) {
		a_inv = accBAinv;
	}
	memset(a_inv, 0, sizeof(accBAinv));
	for (int i = 0; i < 3; i++) { // set identity matrix
		a_inv[i + 1][i] = 1;
	}
	if (write) {
		LOG_INF("Clearing stored calibration data");
		sys_write(MAIN_ACC_6_BIAS_ID, &retained->accBAinv, a_inv, sizeof(accBAinv));
	}
}
#endif

void sensor_calibration_clear_mag(float m_inv[][3], bool write)
{
	bool clearing_live_state = (m_inv == NULL || m_inv == magBAinv);
	if (m_inv == NULL) {
		m_inv = magBAinv;
	}
	memset(m_inv, 0, sizeof(magBAinv)); // zeroed matrix will disable magnetometer in fusion
	if (clearing_live_state) {
		magneto_online_runtime_reset();
	}
	if (write) {
		LOG_INF("Clearing stored calibration data");
		sensor_calibration_online_mag_retained_clear();
		sys_write(MAIN_MAG_BIAS_ID, &retained->magBAinv, m_inv, sizeof(magBAinv));
		sensor_refresh_sensor_ids(); // Refresh reported mag status after clear
	}
}

void sensor_request_calibration(void)
{ printk("Legacy static calibration removed; use temperature calibration.\n"); }

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
void sensor_request_calibration_6_side(void)
{
	sensor_calibration_request(2);
}
#endif

void sensor_request_calibration_mag(void)
{
	// If already collecting, just check if ready
	if (magneto_progress & 0x80) {
		if (!get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
			set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
		}
		return;
	}

	if (!get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
		set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
	}

	// LED sequence before calibration:
	// 1. Flash LED to let user identify tracker
	LOG_INF("Magnetometer calibration: identify tracker");
	set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR);
	k_msleep(2000);  // Wait for pattern to complete

	// 2. Flash twice to indicate calibration is starting
	//    SYS_LED_PATTERN_ONESHOT_PROGRESS: 200ms on + 200ms off, 2 times = 800ms
	set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
	k_msleep(800);

	// Start fresh calibration
	magneto_progress = 0;
	last_magneto_progress = 0;
	magneto_progress_time = 0;
	mag_cal_last_status_log = 0;
	magneto_reset();  // Clear ata buffer and sample count
	magneto_online_reset();  // Clear online accumulator
	magneto_progress |= 1 << 7;  // Set collection active flag
	LOG_INF("Magnetometer calibration started (rotate tracker in all orientations)");
}

static float aBuf[3] = {0};
uint64_t accel_sample = 0;
uint64_t accel_wait_sample = 0;

static void sensor_sample_accel(const float a[3])
{
	memcpy(aBuf, a, sizeof(aBuf));
	accel_sample++;
	if (accel_wait_sample) {
		k_usleep(1); // yield to waiting thread
	}
}

static int sensor_wait_accel(float a[3], k_timeout_t timeout)
{
	int64_t sample_end_time = MAX(k_uptime_ticks() + timeout.ticks, timeout.ticks);
	accel_wait_sample = accel_sample;
	while (accel_sample <= accel_wait_sample && k_uptime_ticks() < sample_end_time) {
		k_usleep(1);
	}
	accel_wait_sample = 0;
	if (k_uptime_ticks() >= sample_end_time) {
		LOG_ERR("Accelerometer wait timed out");
		return -1;
	}
	memcpy(a, aBuf, sizeof(aBuf));
	return 0;
}

static float mBuf[3] = {0};
uint64_t mag_sample = 0;
uint64_t mag_wait_sample = 0;

static void sensor_sample_mag(const float m[3])
{
	memcpy(mBuf, m, sizeof(mBuf));
	mag_sample++;
	if (mag_wait_sample) {
		k_usleep(1); // yield to waiting thread
	}
}

static int sensor_wait_mag(float m[3], k_timeout_t timeout)
{
	int64_t sample_end_time = MAX(k_uptime_ticks() + timeout.ticks, timeout.ticks);
	mag_wait_sample = mag_sample;
	while (mag_sample <= mag_wait_sample && k_uptime_ticks() < sample_end_time) {
		k_usleep(1);
	}
	mag_wait_sample = 0;
	if (k_uptime_ticks() >= sample_end_time) {
		LOG_ERR("Magnetometer wait timed out");
		return -1;
	}
	memcpy(m, mBuf, sizeof(mBuf));
	return 0;
}



#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
// Minimum poses required for partial calibration save (must be before sensor_calibrate_6_side)
#define CALIB_MIN_POSES_FOR_PARTIAL 6

static void sensor_calibrate_6_side(void)
{
	float a_inv[4][3];
	int captured_count = 0;
	LOG_INF("Calibrating main accelerometer 6-side offset");
	LOG_INF("Rest the device on a stable surface");

	sensor_calibration_clear_6_side(a_inv, false);
	int err = sensor_6_sideBias(a_inv, &captured_count);
	if (err) {
		if (err == -3) {
			// Timeout occurred - check if we have enough samples for partial calibration
			LOG_WRN("Calibration timeout after %d poses (minimum: %d)", captured_count, CALIB_MIN_POSES_FOR_PARTIAL);
			if (captured_count >= CALIB_MIN_POSES_FOR_PARTIAL) {
				// We have enough samples, try to calculate calibration from partial data
				LOG_INF("Attempting partial calibration with %d poses...", captured_count);
				wait_for_threads();
				magneto_current_calibration(a_inv, ata, norm_sum, sample_count);
				magneto_reset();
				// Continue to validation below - err will be handled by validate function
				err = 0; // Clear error to allow validation
			} else {
				// Not enough samples - discard and restore previous calibration
				LOG_ERR("Insufficient poses for calibration, discarding data");
				magneto_reset();
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
				return; // Existing calibration is preserved in accBAinv
			}
		} else {
			magneto_reset();
			if (err == -1) {
				LOG_INF("Motion detected");
			}
			a_inv[0][0] = NAN; // invalidate calibration
		}
	}

	if (!err) {
		LOG_INF("Accelerometer matrix:");
		for (int i = 0; i < 3; i++) {
			LOG_INF(
				"%.5f %.5f %.5f %.5f",
				(double)a_inv[0][i],
				(double)a_inv[1][i],
				(double)a_inv[2][i],
				(double)a_inv[3][i]
			);
		}
	}
	if (sensor_calibration_validate_6_side(a_inv, false)) {
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Restoring previous calibration");
		LOG_INF("Accelerometer matrix:");
		for (int i = 0; i < 3; i++) {
			LOG_INF(
				"%.5f %.5f %.5f %.5f",
				(double)accBAinv[0][i],
				(double)accBAinv[1][i],
				(double)accBAinv[2][i],
				(double)accBAinv[3][i]
			);
		}
		sensor_calibration_validate_6_side(NULL, true); // additionally verify old calibration
		return;
	} else {
		LOG_INF("Applying calibration");
		memcpy(accBAinv, a_inv, sizeof(accBAinv));
		sensor_fusion_invalidate(); // only invalidate fusion if calibration was successful
	}
	sys_write(MAIN_ACC_6_BIAS_ID, &retained->accBAinv, accBAinv, sizeof(accBAinv));

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
}
#endif

static int sensor_calibrate_mag(void)
{
	float zero[3] = {0};
	if (v_diff_mag(magBAinv[0], zero) != 0) {
		magneto_reset();
		if (get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
			set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
		}
		return -1; // magnetometer calibration already exists
	}

	if (!get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
		set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
	}

	float m[3];
	if (sensor_wait_mag(m, K_MSEC(1000))) {
		return -1; // Timeout
	}
	sensor_sample_mag_magneto_sample(m); // 400us

	// Periodic status log every 1 second
	int64_t now = k_uptime_get();
	if (now - mag_cal_last_status_log >= 1000) {
		mag_cal_last_status_log = now;
		LOG_INF("Mag cal: %d samples collected", (int)sample_count);
	}

	if (magneto_progress != 0b11111111) {
		return 1;  // Still collecting - signal caller to use short sleep
	}

	float m_inv[4][3];
	LOG_INF("Calibrating magnetometer hard/soft iron offset");

	// max allocated 1072 bytes
#if DEBUG
	printk("ata:\n");
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			printk("%7.2f, ", (double)ata[i * 10 + j]);
		}
		printk("\n");
		k_msleep(3);
	}
	printk("norm_sum: %.2f, sample_count: %.0f\n", norm_sum, sample_count);
#endif
	wait_for_threads();
	magneto_current_calibration(m_inv, ata, norm_sum, sample_count); // 25ms
	magneto_reset();

	LOG_INF("Magnetometer matrix:");
	for (int i = 0; i < 3; i++) {
		LOG_INF(
			"%.5f %.5f %.5f %.5f",
			(double)m_inv[0][i],
			(double)m_inv[1][i],
			(double)m_inv[2][i],
			(double)m_inv[3][i]
		);
	}
	if (sensor_calibration_validate_mag(m_inv, false)) {
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Restoring previous calibration");
		LOG_INF("Magnetometer matrix:");
		for (int i = 0; i < 3; i++) {
			LOG_INF(
				"%.5f %.5f %.5f %.5f",
				(double)magBAinv[0][i],
				(double)magBAinv[1][i],
				(double)magBAinv[2][i],
				(double)magBAinv[3][i]
			);
		}
		sensor_calibration_validate_mag(NULL, true); // additionally verify old calibration
		if (get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
			set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
		}
		return -1;
	} else {
		LOG_INF("Applying calibration");
		memcpy(magBAinv, m_inv, sizeof(magBAinv));
		magneto_online_runtime_reset();  // Restart online calibration from a clean baseline
#if CONFIG_SENSOR_USE_VQF
		vqf_reset_mag_ref();
		sensor_mag_ref_reset(); // Recompute magRef from new calibration
#endif
		// fusion invalidation not necessary
	}
	sys_write(MAIN_MAG_BIAS_ID, &retained->magBAinv, magBAinv, sizeof(magBAinv));

	LOG_INF("Finished calibration");
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
	sensor_refresh_sensor_ids(); // Refresh reported mag status after calibration
	if (get_status(SYS_STATUS_CALIBRATION_RUNNING)) {
		set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
	}
	return 0;
}

// TODO: isAccRest


static void magneto_reset(void)
{
	magneto_progress = 0;
	last_magneto_progress = 0;
	magneto_progress_time = 0;
	memset(ata, 0, sizeof(ata));
	norm_sum = 0;
	sample_count = 0;
	memset(dir_sum, 0, sizeof(dir_sum));
	for (int i = 0; i < 3; i++) {
		dir_min[i] = 2.0f;   // start high
		dir_max[i] = -2.0f;  // start low
	}
	memset(manual_last_dir, 0, sizeof(manual_last_dir));
	memset(manual_last_accel_dir, 0, sizeof(manual_last_accel_dir));
}

static void magneto_online_reset(void)
{
	magneto_online_clear_history();
	online_last_sample_time = 0;
	online_last_update_time = 0;
}

static void magneto_online_runtime_reset(void)
{
	magneto_online_reset();
	online_update_count = 0;
	online_last_buf_avg_norm = 0.0f;
	cal_norm_count = 0;
	cal_norm_ema = 0.0f;
	cal_norm_var_ema = 0.0f;
}

static void magneto_online_clear_history(void)
{
	memset(quad_buf, 0, sizeof(quad_buf));
	online_total_sample_count = 0;
	online_last_checked_sample_count = 0;
	online_last_check_time = 0;
	memset(online_last_dir, 0, sizeof(online_last_dir));
	memset(online_last_accel_dir, 0, sizeof(online_last_accel_dir));
}

static bool magneto_online_quadrant_is_recent(const quadrant_buf_t *qbuf)
{
	if (qbuf->count == 0) {
		return false;
	}
	return (online_total_sample_count - qbuf->last_seq) <= ONLINE_STALE_QUADRANT_MAX_AGE;
}

// Collect all valid samples from all 8 quadrant ring buffers.
// Recomputes ATA, norm_sum, and dir_sum on the fly from raw samples.
static double magneto_online_collect_recent(double ata_out[100], double *norm_sum_out, float dir_sum_out[3])
{
	memset(ata_out, 0, sizeof(double) * 100);
	*norm_sum_out = 0;
	memset(dir_sum_out, 0, sizeof(float) * 3);

	double recent_sample_count = 0;
	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent(&quad_buf[q])) {
			continue;
		}
		for (int i = 0; i < quad_buf[q].count; i++) {
			quadrant_sample_t *s = &quad_buf[q].samples[i];
			magneto_sample((double)s->x, (double)s->y, (double)s->z, ata_out, norm_sum_out, &recent_sample_count);
			magneto_accumulate_direction(dir_sum_out, s->x, s->y, s->z);
		}
	}
	return recent_sample_count;
}

static int magneto_online_recent_sample_count(void)
{
	int count = 0;
	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent(&quad_buf[q])) {
			continue;
		}
		count += quad_buf[q].count;
	}
	return count;
}

static float magneto_online_recent_dir_bias(void)
{
	float dir_sum_recent[3] = {0};
	double recent_sample_count = 0;

	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent(&quad_buf[q])) {
			continue;
		}
		for (int i = 0; i < quad_buf[q].count; i++) {
			quadrant_sample_t *s = &quad_buf[q].samples[i];
			magneto_accumulate_direction(dir_sum_recent, s->x, s->y, s->z);
			recent_sample_count++;
		}
	}

	return magneto_directional_bias(dir_sum_recent, recent_sample_count);
}

static float magneto_online_min_dir_change_threshold(void)
{
	static bool initialized = false;
	static float threshold = 0.0f;

	if (!initialized) {
		const float deg_to_rad = 0.01745329251994329577f;
		threshold = 1.0f - cosf(ONLINE_MIN_DIR_CHANGE_DEG * deg_to_rad);
		initialized = true;
	}

	return threshold;
}

/**
 * Compute directional bias of accumulated mag samples.
 * Returns |sum(m/|m|)| / N, where 0=perfect sphere coverage, 1=all same direction.
 */
static float magneto_directional_bias(const float ds[3], double count)
{
	if (count < 2) {
		return 1.0f;
	}
	float inv_n = 1.0f / (float)count;
	float cx = ds[0] * inv_n;
	float cy = ds[1] * inv_n;
	float cz = ds[2] * inv_n;
	return sqrtf(cx * cx + cy * cy + cz * cz);
}

/**
 * Accumulate a normalized direction for diversity tracking.
 */
static void magneto_accumulate_direction(float ds[3], float mx, float my, float mz)
{
	float norm_sq = mx * mx + my * my + mz * mz;
	if (norm_sq < 1e-8f) {
		return;
	}
#if CONFIG_CMSIS_DSP
	float norm;
	arm_sqrt_f32(norm_sq, &norm);
	float inv_norm = 1.0f / norm;
#else
	float inv_norm = 1.0f / sqrtf(norm_sq);
#endif
	ds[0] += mx * inv_norm;
	ds[1] += my * inv_norm;
	ds[2] += mz * inv_norm;
}

/**
 * Update direction range tracking (min/max per axis of normalized direction).
 * Used only by manual calibration to ensure sufficient multi-axis rotation.
 */
static void magneto_update_dir_range(float mx, float my, float mz)
{
	float norm_sq = mx * mx + my * my + mz * mz;
	if (norm_sq < 1e-8f) {
		return;
	}
#if CONFIG_CMSIS_DSP
	float norm;
	arm_sqrt_f32(norm_sq, &norm);
	float inv_norm = 1.0f / norm;
#else
	float inv_norm = 1.0f / sqrtf(norm_sq);
#endif
	float d[3] = {mx * inv_norm, my * inv_norm, mz * inv_norm};
	for (int i = 0; i < 3; i++) {
		if (d[i] < dir_min[i]) { dir_min[i] = d[i]; }
		if (d[i] > dir_max[i]) { dir_max[i] = d[i]; }
	}
}

/**
 * Get minimum direction range across all three axes.
 * Range = max - min of normalized direction per axis.
 * Higher = more diverse coverage.
 */
static float magneto_min_dir_range(void)
{
	float min_range = 2.0f;
	for (int i = 0; i < 3; i++) {
		float range = dir_max[i] - dir_min[i];
		if (range < min_range) {
			min_range = range;
		}
	}
	return min_range;
}

/**
 * Check if a Magneto calibration result passes quality checks.
 * Performs silent validation (no warning logs) since this is called speculatively.
 * If m_inv_out is non-NULL and check passes, the computed calibration is stored there.
 * Returns true if quality is acceptable.
 */
static bool magneto_quality_check(double *ata_buf, double norm_sum_val, double sample_count_val,
                                  float m_inv_out[][3])
{
	if (sample_count_val < MAG_CAL_MIN_SAMPLES) {
		return false;
	}

	// Run trial calibration
	float m_inv[4][3];
	magneto_current_calibration(m_inv, ata_buf, norm_sum_val, sample_count_val);

	// Silent validation: check bias < 1 and diagonals within 20%
	// (same as sensor_calibration_validate_mag but without LOG_WRN or clearing)
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++) {
		diagonal[i] = m_inv[i + 1][i];
	}
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	if (!v_epsilon(m_inv[0], zero, 1)
	    || !v_epsilon(diagonal, average, MAX(magnitude * 0.2f, 0.1f))) {
		return false;
	}

	if (m_inv_out) {
		memcpy(m_inv_out, m_inv, sizeof(m_inv));
	}
	return true;
}

/**
 * Compute similarity between two BAinv calibration matrices.
 * Uses normalized Frobenius norm: similarity = 1.0 - ||candidate - existing|| / ||existing||.
 * Returns 1.0 for identical calibrations, approaching 0 for very different ones.
 * The offset row (row 0) and soft-iron rows (1-3) contribute equally to the norm.
 */
static float magneto_BAinv_similarity(float existing[4][3], float candidate[4][3])
{
	float diff_norm_sq = 0;
	float existing_norm_sq = 0;

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 3; c++) {
			float d = candidate[r][c] - existing[r][c];
			diff_norm_sq += d * d;
			existing_norm_sq += existing[r][c] * existing[r][c];
		}
	}

	if (existing_norm_sq < 1e-12f) {
		// Existing calibration is near-zero (identity): treat as low similarity
		return 0.0f;
	}

	float similarity = 1.0f - sqrtf(diff_norm_sq / existing_norm_sq);
	if (similarity < 0.0f) { similarity = 0.0f; }
	if (similarity > 1.0f) { similarity = 1.0f; }
	return similarity;
}

/**
 * Blend two BAinv calibrations using exponential moving average (EMA).
 * blended = (1 - alpha) * existing + alpha * candidate
 * Alpha is computed from similarity: more similar → lower alpha (conservative),
 * more different → higher alpha (adaptive to environmental change).
 *
 * The blended result is validated with the same structural checks as
 * magneto_quality_check. If blending produces an invalid result, the
 * candidate is used directly (fallback to full replacement).
 *
 * Returns true if the blend (or fallback) is valid, false if both are invalid.
 */
static bool magneto_blend_BAinv(float out[4][3], float existing[4][3],
                                float candidate[4][3])
{
	float similarity = magneto_BAinv_similarity(existing, candidate);

	// Compute adaptive blending weight
	float alpha;
	if (similarity >= ONLINE_BLEND_SIMILARITY_HIGH) {
		// Very similar: candidate is just a minor refinement → low alpha
		alpha = ONLINE_BLEND_MIN_ALPHA;
	} else if (similarity <= ONLINE_BLEND_SIMILARITY_LOW) {
		// Significant divergence (possible environment change) → high alpha
		alpha = ONLINE_BLEND_MAX_ALPHA;
	} else {
		// Linear interpolation between low and high thresholds
		float t = (similarity - ONLINE_BLEND_SIMILARITY_LOW)
		        / (ONLINE_BLEND_SIMILARITY_HIGH - ONLINE_BLEND_SIMILARITY_LOW);
		alpha = ONLINE_BLEND_MAX_ALPHA
		      + t * (ONLINE_BLEND_MIN_ALPHA - ONLINE_BLEND_MAX_ALPHA);
	}

	// Blend
	float blended[4][3];
	float one_minus_alpha = 1.0f - alpha;
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 3; c++) {
			blended[r][c] = one_minus_alpha * existing[r][c]
			              + alpha * candidate[r][c];
		}
	}

	// Validate blended result
	float zero[3] = {0};
	float diagonal[3];
	for (int i = 0; i < 3; i++) {
		diagonal[i] = blended[i + 1][i];
	}
	float magnitude = v_avg(diagonal);
	float average[3] = {magnitude, magnitude, magnitude};
	if (v_epsilon(blended[0], zero, 1)
	    && v_epsilon(diagonal, average, MAX(magnitude * 0.2f, 0.1f))) {
		// Blended result is valid
		memcpy(out, blended, sizeof(blended));
		return true;
	}

	// Fallback: use candidate directly if blend is invalid.
	// Candidate was already validated by magneto_quality_check before this call,
	// so validate with its own diagonal here for consistency.
	{
		float c_diag[3];
		for (int i = 0; i < 3; i++) {
			c_diag[i] = candidate[i + 1][i];
		}
		float c_avg = v_avg(c_diag);
		float c_avg_arr[3] = {c_avg, c_avg, c_avg};
		if (v_epsilon(candidate[0], zero, 1)
		    && v_epsilon(c_diag, c_avg_arr, MAX(c_avg * 0.2f, 0.1f))) {
			memcpy(out, candidate, sizeof(float) * 4 * 3);
			return true;
		}
	}

	// Both blend and fallback invalid — should not happen since candidate was
	// already validated by magneto_quality_check before calling this function
	return false;
}

#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
static int isAccRest(float *acc, float *pre_acc, float threshold, int *t, int restdelta)
{
	float delta[3];
	delta[0] = acc[0] - pre_acc[0];
	delta[1] = acc[1] - pre_acc[1];
	delta[2] = acc[2] - pre_acc[2];

#if CONFIG_CMSIS_DSP
	float norm_sq;
	arm_dot_prod_f32(delta, delta, 3, &norm_sq);
	float norm_diff;
	arm_sqrt_f32(norm_sq, &norm_diff);
#else
	float norm_diff = sqrtf(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
#endif

	if (norm_diff <= threshold) {
		*t += restdelta;
	} else {
		*t = 0;
	}

	if (*t > 2000) {
		return 1;
	}
	return 0;
}
#endif





#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
// Target number of samples, 6 faces + 12 edges
#define CALIB_TARGET_SAMPLES 18
// Orientation difference threshold (cosine value).
// cos(25 degrees) ≈ 0.90. If dot product > 0.90, the angle between two directions is less than 25 degrees, considered
// duplicate.
#define MIN_ORIENTATION_DIFF_COS 0.90f
// Acceleration threshold for determining stationary state
#define THRESHOLD_ACC 0.02f
// Number of samples to collect for each orientation
#define SAMPLES_PER_ORIENTATION 500
// Timeout for waiting for new pose (30 seconds in milliseconds)
#define CALIB_POSE_TIMEOUT_MS 30000
typedef struct {
	float x, y, z;
} Vector3;

int sensor_6_sideBias(float a_inv[][3], int *captured_count_out)
{
	float rawData[3];
	float pre_acc[3] = {0};
	int resttime = 0;

	Vector3 captured_dirs[CALIB_TARGET_SAMPLES];
	int captured_count = 0;
	int64_t last_new_pose_time = k_uptime_get(); // Track time of last new pose

	// Initialize output parameter
	if (captured_count_out) {
		*captured_count_out = 0;
	}

	magneto_reset();

	LOG_INF("Starting Multi-Position Calibration (Target: %d poses)", CALIB_TARGET_SAMPLES);
	LOG_INF("Please rotate device to random orientations and hold still.");

	// Main loop: until target number of samples collected
	while (captured_count < CALIB_TARGET_SAMPLES) {

		// 1. Wait for device to be stationary
		set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR); // Indicate searching for stationary state
		bool pose_timeout = false;
		while (1) {
			/* Feed watchdog during user interaction wait */
			watchdog_feed(WDT_CHANNEL_CALIBRATION);

			/* Check for timeout - no new pose in CALIB_POSE_TIMEOUT_MS */
			if ((k_uptime_get() - last_new_pose_time) > CALIB_POSE_TIMEOUT_MS) {
				LOG_WRN("Timeout: No new pose detected for %d seconds", CALIB_POSE_TIMEOUT_MS / 1000);
				pose_timeout = true;
				break;
			}

			if (sensor_wait_accel(rawData, K_MSEC(1000))) {
				return -2; // Timeout, magneto state not handled here
			}

			int rest = isAccRest(rawData, pre_acc, THRESHOLD_ACC, &resttime, 100);
			memcpy(pre_acc, rawData, sizeof(rawData)); // Update previous data

			if (rest == 1) {
				// Device is stationary, now check if this pose is new
				// Calculate current vector magnitude
#if CONFIG_CMSIS_DSP
				float norm_sq;
				arm_dot_prod_f32(rawData, rawData, 3, &norm_sq);
				float norm;
				arm_sqrt_f32(norm_sq, &norm);
#else
				float norm = sqrtf(rawData[0] * rawData[0] + rawData[1] * rawData[1] + rawData[2] * rawData[2]);
#endif
				if (norm < 0.1f) {
					continue; // Prevent division by zero (unlikely under gravity)
				}

				// Normalize current vector
				float curr_dir_x = rawData[0] / norm;
				float curr_dir_y = rawData[1] / norm;
				float curr_dir_z = rawData[2] / norm;

				bool is_duplicate = false;
				// Compare with historical data
				for (int i = 0; i < captured_count; i++) {
					// Calculate cosine of angle using dot product
					float dot = curr_dir_x * captured_dirs[i].x + curr_dir_y * captured_dirs[i].y
							  + curr_dir_z * captured_dirs[i].z;

					// If dot product is close to 1, the directions are almost the same,
					// or a slight wobble of a previous direction
					if (dot > MIN_ORIENTATION_DIFF_COS) {
						is_duplicate = true;
						break;
					}
				}

				if (!is_duplicate) {
					// This is a new valid pose! Break the wait loop and start capturing data
					// Save this direction vector
					captured_dirs[captured_count].x = curr_dir_x;
					captured_dirs[captured_count].y = curr_dir_y;
					captured_dirs[captured_count].z = curr_dir_z;
					break;
				} else {
					// Duplicate pose detected, briefly flash LED to prompt user to change orientation, but do not error
					set_led(SYS_LED_PATTERN_FLASH, SYS_LED_PRIORITY_SENSOR);
					k_msleep(100); // Debounce slightly
				}
			}
			k_msleep(20);
		}

		// Check if we timed out waiting for a new pose
		if (pose_timeout) {
			if (captured_count_out) {
				*captured_count_out = captured_count;
			}
			// Return -3 for timeout with captured_count available for partial save decision
			return -3;
		}

		// Reset timeout counter when a new valid pose is found
		last_new_pose_time = k_uptime_get();

		LOG_INF("Capturing pose %d/%d...", captured_count + 1, CALIB_TARGET_SAMPLES);
		set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_SENSOR);

		int sample_idx = 0;
		while (sample_idx < SAMPLES_PER_ORIENTATION) {
			if (sensor_wait_accel(rawData, K_MSEC(1000))) {
				return -2;
			}

			if (!v_epsilon(rawData, pre_acc, 0.03f)) {
				LOG_INF("Motion detected during capture, retrying...");
				sample_idx = -1;
				break;
			}
			memcpy(pre_acc, rawData, sizeof(rawData));

			magneto_sample(rawData[0], rawData[1], rawData[2], ata, &norm_sum, &sample_count);

			sample_idx++;

			if (sample_idx % 50 == 0) {
				printk(".");
				/* Feed watchdog periodically during sampling */
				watchdog_feed(WDT_CHANNEL_CALIBRATION);
			}
		}

		if (sample_idx == -1) {
			continue;
		}

		captured_count++;
		set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
		LOG_INF("Pose %d saved!", captured_count);

		k_msleep(500);
	}

	if (captured_count_out) {
		*captured_count_out = captured_count;
	}

	LOG_INF("Calculating calibration matrix...");

	wait_for_threads();
	magneto_current_calibration(a_inv, ata, norm_sum, sample_count);

	magneto_reset();

	LOG_INF("Calibration calculation complete.");
	return 0;
}
#endif

// Collect magnetometer sample for manual calibration.
// Accumulates into ATA and periodically runs trial calibration for quality check.
static void sensor_sample_mag_magneto_sample(const float m[3])
{
	// Direction diversity gate: accept sample if either mag direction OR
	// accelerometer (gravity) direction has changed since last accepted.
	// Identical logic to the online path — breaks direction lock-in during
	// magnetic interference by cross-validating with physical orientation.
	//
	// Gate accelerometer by magnitude as well: skip the direction check
	// entirely when the device is under strong linear acceleration, falling
	// back to mag-only for that sample.
	float accel_mag_sq = aBuf[0] * aBuf[0] + aBuf[1] * aBuf[1] + aBuf[2] * aBuf[2];
	bool accel_trustworthy = (accel_mag_sq >= MAG_CAL_ACCEL_MAG_MIN_SQ
	                       && accel_mag_sq <= MAG_CAL_ACCEL_MAG_MAX_SQ);

	// Normalize magnetometer
	float mag_norm_sq = m[0] * m[0] + m[1] * m[1] + m[2] * m[2];
	if (mag_norm_sq < 1e-8f) {
		return;
	}
	float mag_inv = 1.0f / sqrtf(mag_norm_sq);
	float cur_mag_dir[3] = {m[0] * mag_inv, m[1] * mag_inv, m[2] * mag_inv};

	// Normalize accelerometer (if trustworthy)
	float cur_accel_dir[3] = {0};
	if (accel_trustworthy) {
		float accel_inv = 1.0f / sqrtf(accel_mag_sq);
		cur_accel_dir[0] = aBuf[0] * accel_inv;
		cur_accel_dir[1] = aBuf[1] * accel_inv;
		cur_accel_dir[2] = aBuf[2] * accel_inv;
	}

	if (sample_count > 0) {
		float min_change = magneto_online_min_dir_change_threshold();

		// Mag direction change
		float mag_dot = cur_mag_dir[0] * manual_last_dir[0]
		              + cur_mag_dir[1] * manual_last_dir[1]
		              + cur_mag_dir[2] * manual_last_dir[2];
		bool mag_changed = (1.0f - mag_dot >= min_change);

		// Accel direction change (only if accel is trustworthy)
		bool accel_changed = false;
		if (accel_trustworthy) {
			float accel_dot = cur_accel_dir[0] * manual_last_accel_dir[0]
			                + cur_accel_dir[1] * manual_last_accel_dir[1]
			                + cur_accel_dir[2] * manual_last_accel_dir[2];
			accel_changed = (1.0f - accel_dot >= min_change);
		}

		// Accept only if at least one direction source shows movement
		if (!mag_changed && !accel_changed) {
			return; // redundant sample
		}
	}

	// Update last accepted directions
	manual_last_dir[0] = cur_mag_dir[0];
	manual_last_dir[1] = cur_mag_dir[1];
	manual_last_dir[2] = cur_mag_dir[2];
	if (accel_trustworthy) {
		manual_last_accel_dir[0] = cur_accel_dir[0];
		manual_last_accel_dir[1] = cur_accel_dir[1];
		manual_last_accel_dir[2] = cur_accel_dir[2];
	}

	// Accept sample - add to Magneto accumulator
	magneto_sample(m[0], m[1], m[2], ata, &norm_sum, &sample_count); // 400us
	magneto_accumulate_direction(dir_sum, m[0], m[1], m[2]);
	magneto_update_dir_range(m[0], m[1], m[2]);

	// Attempt trial calibration every MAG_CAL_TRIAL_INTERVAL samples
	if (sample_count >= MAG_CAL_MIN_SAMPLES &&
	    (int)sample_count % MAG_CAL_TRIAL_INTERVAL < 1) {
		float min_range = magneto_min_dir_range();
		LOG_INF("Mag cal check: %d samples, min_range=%.2f (need %.2f)",
		        (int)sample_count, (double)min_range, (double)MAG_CAL_MIN_DIR_RANGE);

		// Require minimum directional coverage before attempting calibration
		if (min_range < MAG_CAL_MIN_DIR_RANGE) {
			LOG_INF("Mag cal: need more rotation, keep turning");
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
			return;
		}

		if (magneto_quality_check(ata, norm_sum, sample_count, NULL)) {
			magneto_progress |= 0b01111111;
			LOG_INF("Mag cal ready: %d samples, min_range=%.2f",
			        (int)sample_count, (double)min_range);
			set_led(SYS_LED_PATTERN_FLASH, SYS_LED_PRIORITY_SENSOR);
		} else {
			LOG_INF("Mag cal: not ready yet, keep rotating (%d samples)",
			        (int)sample_count);
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_SENSOR);
		}
	}
}

// Phase 2: Background online magnetometer calibration
// Called from sensor loop for each new raw mag sample during normal operation.
// Gated by: VQF disturbance detection, accel magnitude, time interval, and direction change.
void sensor_calibration_online_mag_sample(const float m[3])
{
	// Don't accumulate during manual calibration
	if (magneto_progress & 0x80) {
		return;
	}

	// Reject if VQF detects magnetic disturbance (only when we have an existing
	// calibration — VQF only receives mag data when calibrated, so mag_dist_detected
	// is meaningless without calibration).
	// Exception: if current calibration quality is bad (norm CV > 10%), the disturbance
	// detection itself may be unreliable due to the bad calibration, so skip the gate.
#if CONFIG_SENSOR_USE_VQF
	{
		float zero[3] = {0};
		bool has_cal = (v_diff_mag(magBAinv[0], zero) != 0);
		float current_cv = sensor_calibration_get_mag_quality();
		if (has_cal && current_cv < 0.10f && vqf_get_mag_dist_detected()) {
			return;
		}
	}
#endif

	// Rate limit: minimum interval between samples
	int64_t now = k_uptime_get();
	if (now - online_last_sample_time < ONLINE_MIN_INTERVAL_MS) {
		return;
	}

	// Gate by accel magnitude: reject samples under strong linear acceleration
	float accel_mag_sq = aBuf[0] * aBuf[0] + aBuf[1] * aBuf[1] + aBuf[2] * aBuf[2];
	if (accel_mag_sq < MAG_CAL_ACCEL_MAG_MIN_SQ || accel_mag_sq > MAG_CAL_ACCEL_MAG_MAX_SQ) {
		return;
	}

	// Direction diversity gate: accept sample if either mag direction OR
	// accelerometer (gravity) direction has changed since last accepted sample.
	// Pure magnetometer-based direction check can suffer from "direction lock-in"
	// during strong magnetic interference: the mag reading points to a distorted
	// but stable direction while the tracker physically rotates.  The accelerometer
	// cross-check breaks this deadlock — if the device has physically moved
	// (accel direction changed), accept the sample regardless of mag direction.
	// Normalize magnetometer reading
	float norm_sq = m[0] * m[0] + m[1] * m[1] + m[2] * m[2];
	if (norm_sq < 1e-8f) {
		return;
	}
#if CONFIG_CMSIS_DSP
	float norm;
	arm_sqrt_f32(norm_sq, &norm);
	float inv_norm = 1.0f / norm;
#else
	float inv_norm = 1.0f / sqrtf(norm_sq);
#endif
	float cur_dir[3] = {m[0] * inv_norm, m[1] * inv_norm, m[2] * inv_norm};

	// Normalize accelerometer to get gravity direction
	// aBuf magnitude already validated (~1g) by the accel gate above
	float accel_norm = sqrtf(accel_mag_sq);
	float accel_inv = 1.0f / accel_norm;
	float cur_accel_dir[3] = {aBuf[0] * accel_inv, aBuf[1] * accel_inv, aBuf[2] * accel_inv};

	if (online_total_sample_count > 0) {
		float mag_dot = cur_dir[0] * online_last_dir[0]
		              + cur_dir[1] * online_last_dir[1]
		              + cur_dir[2] * online_last_dir[2];
		float accel_dot = cur_accel_dir[0] * online_last_accel_dir[0]
		                + cur_accel_dir[1] * online_last_accel_dir[1]
		                + cur_accel_dir[2] * online_last_accel_dir[2];

		float min_change = magneto_online_min_dir_change_threshold();
		bool mag_changed = (1.0f - mag_dot >= min_change);
		bool accel_changed = (1.0f - accel_dot >= min_change);

		if (!mag_changed && !accel_changed) {
			return; // neither mag nor accel direction changed enough
		}
	}

	online_last_sample_time = now;
	online_last_dir[0] = cur_dir[0];
	online_last_dir[1] = cur_dir[1];
	online_last_dir[2] = cur_dir[2];
	online_last_accel_dir[0] = cur_accel_dir[0];
	online_last_accel_dir[1] = cur_accel_dir[1];
	online_last_accel_dir[2] = cur_accel_dir[2];

	// Route sample to its octant based on sign of (x, y, z).
	// This guarantees each octant independently rolls its ring buffer,
	// preventing a single orientation from evicting diverse data in other octants.
	int octant = 0;
	if (m[0] < 0) octant |= 1;
	if (m[1] < 0) octant |= 2;
	if (m[2] < 0) octant |= 4;

	quadrant_buf_t *qbuf = &quad_buf[octant];
	online_total_sample_count++;
	qbuf->last_seq = online_total_sample_count;
	qbuf->samples[qbuf->head].x = m[0];
	qbuf->samples[qbuf->head].y = m[1];
	qbuf->samples[qbuf->head].z = m[2];
	qbuf->head = (qbuf->head + 1) % QUADRANT_BUF_SIZE;
	if (qbuf->count < QUADRANT_BUF_SIZE) {
		qbuf->count++;
	}
}

static bool sensor_calibration_online_mag_check(void)
{
	int recent_sample_count_now = magneto_online_recent_sample_count();
	int64_t now = k_uptime_get();

	if (recent_sample_count_now < MAG_CAL_MIN_SAMPLES) {
		return false;
	}
	if (online_total_sample_count == online_last_checked_sample_count) {
		return false;
	}
	if (online_last_check_time > 0 &&
	    (now - online_last_check_time) < ONLINE_MIN_CHECK_INTERVAL_MS) {
		return false;
	}

	online_last_checked_sample_count = online_total_sample_count;
	online_last_check_time = now;

	float zero[3] = {0};
	bool has_existing = (v_diff_mag(magBAinv[0], zero) != 0);
	float current_cv = has_existing ? sensor_calibration_get_mag_quality() : 1.0f;

	// If the current calibration is already good enough AND we've had enough
	// updates to trust that assessment, skip the heavy Magneto fit.
	//
	// Two-tier convergence:
	//   Tier 1 (strict): CV is good AND directional coverage is adequate
	//     → require low dir_bias (sphere sampled evenly)
	//   Tier 2 (relaxed): CV is excellent AND we've done many updates
	//     → trust the fit regardless of dir_bias (directional fluctuations
	//       during normal rotation are just sampling noise, not real problems)
	if (has_existing && current_cv < CAL_NORM_GOOD_CV && online_update_count >= ONLINE_MIN_UPDATES) {
		float dir_bias_check = magneto_online_recent_dir_bias();

		// Tier 2: Excellent fit + sufficient history — lock it in.
		// CV < 0.035 and 3+ updates mean the calibration has reliably
		// converged.  Further updates would only add noise.
		if (current_cv < 0.035f && online_update_count >= 3) {
			LOG_INF("Online mag cal: converged (cv=%.3f, dir_bias=%.3f, %d updates)",
			        (double)current_cv, (double)dir_bias_check, online_update_count);
			return false;
		}

		// Tier 1: Good fit with adequate directional coverage
		if (dir_bias_check < 0.10f) {
			LOG_INF("Online mag cal: skipping (cv=%.3f < %.3f, dir_bias=%.3f, %d updates)",
			        (double)current_cv, (double)CAL_NORM_GOOD_CV,
			        (double)dir_bias_check, online_update_count);
			return false;
		}
		// Directional bias still too high: buffer samples are clustered.
		// Fall through to run calibration even though CV looks good.
		LOG_INF("Online mag cal: CV ok but dir_bias=%.3f >= 0.10, continuing",
		        (double)dir_bias_check);
	}

	double ata_recent[100];
	double recent_norm_sum;
	float recent_dir_sum[3];
	double recent_sample_count = magneto_online_collect_recent(ata_recent, &recent_norm_sum, recent_dir_sum);
	if (recent_sample_count < MAG_CAL_MIN_SAMPLES) {
		return false;
	}

	// Detect magnetic environment changes by comparing the buffer's
	// average raw field strength against the last update's reference.
	// When the norm changes by >40% (e.g., moving between a desk and
	// a high-interference area), clear buffers to prevent mixed-data
	// fits.  Direction is preserved — only scale changes.
	float buf_avg_norm = (float)(recent_norm_sum / recent_sample_count);

	if (has_existing && online_update_count > 0 && online_last_buf_avg_norm > 0.0f) {
		float norm_ratio = buf_avg_norm / online_last_buf_avg_norm;
		if (norm_ratio > 1.4f || norm_ratio < 0.71f) {
			LOG_WRN("Online mag cal: env change detected (buf norm %.3f -> %.3f, ratio %.2f), "
			        "resetting buffers",
			        (double)online_last_buf_avg_norm, (double)buf_avg_norm, (double)norm_ratio);
			magneto_online_clear_history();
			online_update_count = 0;
			online_last_buf_avg_norm = 0.0f;
			return false;
		}
	}

	float dbias = magneto_directional_bias(recent_dir_sum, recent_sample_count);

	// Compute per-axis spread to diagnose directional coverage.
	// If Z-axis std is much lower than X/Y, the fit is under-constrained
	// in the vertical direction, causing poor calibration when tilted.
	double axis_sum[3] = {0};
	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent(&quad_buf[q])) {
			continue;
		}
		for (int i = 0; i < quad_buf[q].count; i++) {
			quadrant_sample_t *s = &quad_buf[q].samples[i];
			axis_sum[0] += (double)s->x;
			axis_sum[1] += (double)s->y;
			axis_sum[2] += (double)s->z;
		}
	}
	double axis_mean[3];
	double axis_var[3] = {0};
	for (int a = 0; a < 3; a++) {
		axis_mean[a] = axis_sum[a] / recent_sample_count;
	}
	for (int q = 0; q < ONLINE_QUADRANT_COUNT; q++) {
		if (!magneto_online_quadrant_is_recent(&quad_buf[q])) {
			continue;
		}
		for (int i = 0; i < quad_buf[q].count; i++) {
			quadrant_sample_t *s = &quad_buf[q].samples[i];
			double dx = (double)s->x - axis_mean[0];
			double dy = (double)s->y - axis_mean[1];
			double dz = (double)s->z - axis_mean[2];
			axis_var[0] += dx * dx;
			axis_var[1] += dy * dy;
			axis_var[2] += dz * dz;
		}
	}
	float axis_std[3];
	for (int a = 0; a < 3; a++) {
		axis_std[a] = sqrtf((float)(axis_var[a] / recent_sample_count));
	}
	// Z/X ratio: < 0.3 suggests strong horizontal bias
	float zx_ratio = axis_std[0] > 0.001f ? axis_std[2] / axis_std[0] : 0.0f;
	LOG_INF("Online mag cal: axis_std x=%.4f y=%.4f z=%.4f (zx=%.2f, n=%d)",
	        (double)axis_std[0], (double)axis_std[1], (double)axis_std[2],
	        (double)zx_ratio, (int)recent_sample_count);

	// Quality check: directional diversity + validation + compute calibration
	float m_inv[4][3];
	if (!magneto_quality_check(ata_recent, recent_norm_sum, recent_sample_count, m_inv)) {
		LOG_INF("Online mag cal: check failed (%d recent samples, dir_bias=%.3f)",
		        (int)recent_sample_count, (double)dbias);
		return false;
	}

	if (has_existing) {
		// Enforce minimum cooldown between updates to avoid frequent VQF mag ref resets.
		// Each update resets VQF's heading reference, causing ~6s of re-establishment.
		if (online_last_update_time > 0 &&
		    (now - online_last_update_time) < (ONLINE_MIN_UPDATE_INTERVAL_S * 1000LL)) {
			return false;
		}

		// Blend trial calibration with existing using EMA.
		// Blending weight is similarity-adaptive: more similar → conservative,
		// more divergent → faster adaptation (possible environment change).
		float blended[4][3];
		if (!magneto_blend_BAinv(blended, magBAinv, m_inv)) {
			LOG_WRN("Online mag cal: blend validation failed, skipping update");
			return false;
		}

		float similarity = magneto_BAinv_similarity(magBAinv, m_inv);

		// Reject candidate if similarity is below threshold — the data
		// is too inconsistent for a meaningful fit.  This guards against
		// mixed-data fits (e.g. when the tracker moves between magnetic
		// environments and the quadrant buffer holds samples from both old
		// and new locations).
		// Rather than blindly trusting a poor fit, let the buffer age out
		// stale samples; the next cycle will fit a consistent dataset with
		// much higher similarity.
		//
		// Exception: during the first ONLINE_MIN_UPDATES cycles we accept
		// even low-sim fits to establish an initial baseline (especially
		// important when booting with a stale NVS calibration).
		if (similarity < 0.85f) {
			if (online_update_count < ONLINE_MIN_UPDATES) {
				LOG_INF("Online mag cal: low sim=%.3f accepted (early bootstrap #%d)",
				        (double)similarity, online_update_count + 1);
			} else if (current_cv < CAL_NORM_GOOD_CV) {
				LOG_WRN("Online mag cal: rejecting candidate (sim=%.3f < 0.85, "
				        "current cv=%.3f is good — possible mixed data)",
				        (double)similarity, (double)current_cv);
				return false;
			} else {
				LOG_WRN("Online mag cal: rejecting candidate (sim=%.3f < 0.85, "
				        "current cv=%.3f — incomplete/dirty buffer?)",
				        (double)similarity, (double)current_cv);
				return false;
			}
		}

		LOG_INF("Online mag cal: blended (#%d, %d samples, dir_bias=%.3f, cur_cv=%.3f, sim=%.3f)",
		        online_update_count + 1,
		        (int)recent_sample_count, (double)dbias, (double)current_cv,
		        (double)similarity);
		memcpy(magBAinv, blended, sizeof(magBAinv));
		memcpy(m_inv, blended, sizeof(m_inv)); // for logging below

		// Start the next cycle from a clean buffer. Keeping pre-update samples
		// around can immediately re-mix old field conditions into the next fit.
		magneto_online_clear_history();
		online_last_update_time = now;
		online_update_count++;
		online_last_buf_avg_norm = buf_avg_norm;

		// Reset VQF mag reference so it re-establishes with the refined calibration
#if CONFIG_SENSOR_USE_VQF
		vqf_reset_mag_ref();
		sensor_mag_ref_reset();
#endif
	} else {
		LOG_INF("Online mag cal: first calibration (%d recent samples, dir_bias=%.3f)",
		        (int)recent_sample_count, (double)dbias);

		// First calibration: use candidate directly
		memcpy(magBAinv, m_inv, sizeof(magBAinv));
		magneto_online_clear_history();
		online_last_update_time = now;
		online_update_count = 1;
		online_last_buf_avg_norm = buf_avg_norm;

		// Reset VQF mag reference so it re-establishes with the new calibration
#if CONFIG_SENSOR_USE_VQF
		vqf_reset_mag_ref();
		sensor_mag_ref_reset();
#endif
	}

	// Reset norm tracking after calibration change
	cal_norm_count = 0;
	cal_norm_ema = 0;
	cal_norm_var_ema = 0;

	sys_write(MAIN_MAG_BIAS_ID, &retained->magBAinv, magBAinv, sizeof(magBAinv));
	sensor_refresh_sensor_ids();

	LOG_INF("Online mag cal applied:");
	for (int i = 0; i < 3; i++) {
		LOG_INF("%.5f %.5f %.5f %.5f",
			(double)m_inv[0][i], (double)m_inv[1][i],
			(double)m_inv[2][i], (double)m_inv[3][i]);
	}

	return true;
}

int sensor_calibration_online_mag_status(float *dir_bias)
{
	if (dir_bias) {
		*dir_bias = magneto_online_recent_dir_bias();
	}
	return magneto_online_recent_sample_count();
}

// Feed calibrated mag norm for runtime quality tracking.
// Called from sensor.c after applying BAinv calibration.
void sensor_calibration_track_mag_norm(float cal_norm)
{
	if (cal_norm < 1e-6f) {
		return;
	}
	if (cal_norm_count == 0) {
		cal_norm_ema = cal_norm;
		cal_norm_var_ema = 0;
	} else {
		float diff = cal_norm - cal_norm_ema;
		cal_norm_ema += CAL_NORM_EMA_ALPHA * diff;
		cal_norm_var_ema += CAL_NORM_EMA_ALPHA * (diff * diff - cal_norm_var_ema);
	}
	cal_norm_count++;
}

// Get current calibration quality: returns norm CV (std/mean).
// Lower is better. Returns 1.0 if insufficient data.
float sensor_calibration_get_mag_quality(void)
{
	if (cal_norm_count < 100 || cal_norm_ema < 1e-6f) {
		return 1.0f;
	}
	float std = sqrtf(cal_norm_var_ema);
	return std / cal_norm_ema;
}

static int sensor_calibration_request(int id)
{
    if(id>0 && calwire_active()) return -1;
	static int requested = 0;
	switch (id) {
	case -1:
		requested = 0;
		return 0;
	case 0:
		return requested;
	default:
		if (requested != 0) {
			LOG_ERR("Sensor calibration is already running");
			return -1;
		}
		requested = id;
		return 0;
	}
}

static void calibration_thread(void)
{
	/* Register calibration thread with watchdog - use long timeout for lengthy operations */
	watchdog_register_thread(WDT_CHANNEL_CALIBRATION, 0);

	sensor_calibration_read();

#if CONFIG_SENSOR_USE_TCAL
	// Wait for sensor to be initialized before building LUT
	// This prevents issues when sensor fails to initialize
	int init_wait_count = 0;
	while (!sensor_is_initialized()) {
		watchdog_feed(WDT_CHANNEL_CALIBRATION);
		k_msleep(100);
		init_wait_count++;
		// Timeout after 10 seconds to avoid infinite loop if sensor never initializes
		if (init_wait_count >= 100) {
			LOG_WRN("T-Cal: Timeout waiting for sensor initialization, skipping LUT build");
			break;
		}
	}

	// Build LUT at startup if T-Cal data is available and sensor is initialized
	// LUT is only in RAM and needs to be rebuilt after every boot
	// Use incremental build: priority zone first, then background completion
	if (sensor_is_initialized() && retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT) {
		// Validate tempCalState.count to prevent issues with corrupted retained data
		if (retained->tempCalState.count > TCAL_BUFFER_SIZE) {
			LOG_ERR("T-Cal: Invalid point count %u (max %d), resetting",
			        retained->tempCalState.count, TCAL_BUFFER_SIZE);
			retained->tempCalState.count = 0;
			retained->tempCalState.valid = false;
		} else {
			float current_temp = sensor_get_current_imu_temperature();
			if (!isnan(current_temp)) {
				LOG_INF("T-Cal: Starting incremental LUT build at startup (current temp: %.1f°C)", (double)current_temp);
				sensor_tcal_build_lut_priority(current_temp);
			} else {
				LOG_WRN("T-Cal: Cannot build LUT - temperature not available");
			}
		}
	}
#endif

	// TODO: be able to block the sensor while doing certain operations
	// TODO: reset fusion on calibration finished
	// TODO: start and run thread from request?
	// TODO: replace wait_for_motion with isAccRest

	// Verify calibrations
	sensor_calibration_validate(NULL, NULL, true);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
	sensor_calibration_validate_6_side(NULL, true);
#endif
	sensor_calibration_validate_mag(NULL, true);

	// requested calibrations run here
	while (1) {
		int requested = sensor_calibration_request(0);
		switch (requested) {
		case 1:
            sensor_request_calibration();
			sensor_calibration_request(-1); // clear request
			set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
			break;
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
		case 2:
			set_status(SYS_STATUS_CALIBRATION_RUNNING, true);
			sensor_calibrate_6_side();
			sensor_calibration_request(-1); // clear request
			set_status(SYS_STATUS_CALIBRATION_RUNNING, false);
			break;
#endif

		default:
			if (magneto_progress & 0b10000000) {
				requested = sensor_calibrate_mag();
			}
			break;
		}

#if CONFIG_SENSOR_USE_TCAL
		// Continue LUT background build if in progress
		if (mls_lut.build_state == MLS_LUT_BUILD_BACKGROUND) {
			if (sensor_tcal_build_lut_continue()) {
				LOG_INF("T-Cal LUT: Background build complete (%d/%d entries)",
				        mls_lut.computed_count, MLS_LUT_SIZE);
			}
		}
#endif

		// Phase 2: Background online magnetometer calibration check
		if (requested == 0) {
			sensor_calibration_online_mag_check();
		}

		/* Feed watchdog at end of each loop iteration */
		watchdog_feed(WDT_CHANNEL_CALIBRATION);

		if (requested < 0) {
			k_msleep(5);
		} else if (requested > 0) {
			k_msleep(20);  // Mag cal in progress - short sleep for fast sampling
		} else {
			k_msleep(100);
		}
	}
}

#if CONFIG_SENSOR_USE_TCAL

void sensor_tcal_get_summary(uint16_t *points, bool *lut_valid, uint16_t *lut_entries,
                             float *temp, bool *coverage, bool *collecting, bool *enabled)
{
	if (points) *points = retained->tempCalState.count;
	if (lut_valid) *lut_valid = mls_lut.valid;
	if (lut_entries) *lut_entries = (uint16_t)mls_lut.computed_count;
	if (temp) *temp = sensor_get_current_imu_temperature();
	if (coverage) *coverage = sensor_tcal_coverage_ready();
	if (collecting) *collecting = tcal_accum.active;
	if (enabled) *enabled = tcal_auto_calibration_enabled;
}

void sensor_tcal_status(void)
{
	printk("Temperature Calibration Status (MLS):\n");
	printk("  - MLS available: %s\n", retained->tempCalState.valid ? "Yes" : "No");
	printk("  - Points collected: %u / %d\n", retained->tempCalState.count, TCAL_BUFFER_SIZE);
	printk("  - Collection range: [%d, %d)C; bucket width: %.2fC\n",
	       CONFIG_SENSOR_POLY_TEMP_MIN, CONFIG_SENSOR_POLY_TEMP_MAX,
	       (double)(1.0f / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE));

	// Display LUT status
	const char *lut_state_str;
	switch (mls_lut.build_state) {
	case MLS_LUT_BUILD_IDLE:
		lut_state_str = "Idle";
		break;
	case MLS_LUT_BUILD_PRIORITY:
		lut_state_str = "Building priority zone";
		break;
	case MLS_LUT_BUILD_BACKGROUND:
		lut_state_str = "Background build";
		break;
	case MLS_LUT_BUILD_COMPLETE:
		lut_state_str = "Complete";
		break;
	default:
		lut_state_str = "Unknown";
		break;
	}
	printk("  - LUT valid: %s, state: %s, entries: %d/%d\n",
	       mls_lut.valid ? "Yes" : "No",
	       lut_state_str,
	       mls_lut.computed_count,
	       MLS_LUT_SIZE);

	// Use quality assessment to get calibrated temperature range and error
	float current_temp = sensor_get_current_imu_temperature();
	tcal_quality_t quality;
	bool quality_ok = sensor_tcal_assess_quality(current_temp, &quality);

	if (quality.point_count > 0 && quality.temp_min < quality.temp_max) {
		printk("  - Calibrated temp range: %.2fC to %.2fC\n", (double)quality.temp_min, (double)quality.temp_max);
	}

	// Display quality assessment
	if (quality.point_count > 0) {
		// Display quality status
		const char *quality_str;
        if (quality_ok && sensor_tcal_coverage_ready()) {
			quality_str = "GOOD - Temperature compensation available (MLS)";
		} else if (quality.point_count < TCAL_MIN_CURVE_POINTS) {
			quality_str = "INSUFFICIENT - Need more calibration points";
		} else {
			quality_str = "UNKNOWN";
		}
		printk("  - Calibration quality: %s\n", quality_str);
	}

    printk("  - Current temp: %.2fC\n", (double)current_temp);
    printk("  - Current operating temperature coverage: %s\n", sensor_tcal_coverage_ready() ? "ready" : "insufficient");
    printk("  - Bias tracking mode: T-Cal + Earth-aware VQF residual tracking\n");
    printk("  - Legacy static/boot/runtime calibration: removed\n");
}

static void update_tcal_state(void)
{
	// Invalidate lookup cache since calibration data changed
	sensor_tcal_cache_invalidate();

	// Polynomial coefficients are no longer used; keep persisted storage zeroed
	memset(retained->tempCalCoeffs, 0, sizeof(retained->tempCalCoeffs));
	retained->tempCalState.degree = 0;

	// Mark MLS availability based on point count
	retained->tempCalState.valid = (retained->tempCalState.count >= 1);
	retained->tempCalState.config_sig = TCAL_CONFIG_SIG;

	if (retained->tempCalState.valid) {
		LOG_INF("T-Cal: MLS state refreshed with %u points", retained->tempCalState.count);
		printk("T-Cal: MLS data refreshed successfully.\n");

		// Start incremental LUT build for O(1) runtime lookup
		// Priority zone (current temp ±3°C) is built immediately
		// Remaining entries are built in background by calibration_thread
		float current_temp = sensor_get_current_imu_temperature();
		if (!isnan(current_temp)) {
			sensor_tcal_build_lut_priority(current_temp);
		}
	} else {
		LOG_INF("T-Cal: No points available");
		printk("T-Cal: No points available.\n");
	}

	// Save updated state to NVS
	sys_write(
		MAIN_GYRO_TCAL_STATE_ID,
		&retained->tempCalState,
		&retained->tempCalState,
		sizeof(retained->tempCalState)
	);
	sys_write(
		MAIN_GYRO_TCAL_POINTS_ID,
		retained->tempCalPoints,
		retained->tempCalPoints,
		sizeof(retained->tempCalPoints)
	);
	sys_write(
		MAIN_GYRO_TCAL_COEFFS_ID,
		retained->tempCalCoeffs,
		retained->tempCalCoeffs,
		sizeof(retained->tempCalCoeffs)
	);

	// Update fusion bias while preserving orientation
	sensor_fusion_update_bias(NULL);
}

/**
 * Recover bins from their stored temperatures, including the fleet2 layout
 * (20..34 C / 0.1 C -> 15..43 C / 0.2 C, both 140 points). Adjacent measured
 * points are averaged; unmeasured temperatures remain empty. Keep the actual
 * mean temperature rather than inventing a new bucket-center observation.
 *
 * Startup only updates RAM. The original NVS observations remain intact until
 * normal collection/pause saves the table. Re-check indices even when the
 * signature matches: a reset between the separate state/points NVS writes can
 * leave the new signature beside the old array. Re-binning is idempotent.
 */
void sensor_tcal_validate_layout(void)
{
	const uint16_t previous_sig = retained->tempCalState.config_sig;
	const uint16_t legacy_sig = 20 * 1000 + 34 * 10 + 10;
	bool supported = previous_sig == TCAL_CONFIG_SIG ||
		(previous_sig == legacy_sig && TCAL_BUFFER_SIZE == 140 &&
		 CONFIG_SENSOR_POLY_TEMP_MIN == 15 && CONFIG_SENSOR_POLY_TEMP_MAX == 43 &&
		 CONFIG_SENSOR_POLY_STEPS_PER_DEGREE == 5);
	/* Static scratch avoids exhausting the startup thread's small stack. */
	static struct TempCalPoint rebinned[TCAL_BUFFER_SIZE];
	static uint16_t contributors[TCAL_BUFFER_SIZE];
	memset(rebinned, 0, sizeof(rebinned));
	memset(contributors, 0, sizeof(contributors));
	uint16_t kept = 0, rejected = 0;
	for (int i = 0; i < TCAL_BUFFER_SIZE; ++i) {
		const struct TempCalPoint *p = &retained->tempCalPoints[i];
		if (p->temp == 0.0f) continue;
		if (!supported || !isfinite(p->temp) ||
		    p->temp < CONFIG_SENSOR_POLY_TEMP_MIN || p->temp >= CONFIG_SENSOR_POLY_TEMP_MAX ||
		    !isfinite(p->bias[0]) || !isfinite(p->bias[1]) || !isfinite(p->bias[2])) {
			rejected++;
			continue;
		}
		int idx = TEMP_TO_IDX(p->temp);
		if (idx < 0 || idx >= TCAL_BUFFER_SIZE) {
			rejected++;
			continue;
		}
		unsigned n = ++contributors[idx];
		if (n == 1) {
			rebinned[idx] = *p;
			kept++;
		} else {
			/* Equal weights: the legacy table has no per-point sample counts. */
			rebinned[idx].temp += (p->temp - rebinned[idx].temp) / n;
			for (int axis = 0; axis < 3; ++axis) {
				rebinned[idx].bias[axis] = (float)((double)rebinned[idx].bias[axis] +
					((double)p->bias[axis] - rebinned[idx].bias[axis]) / n);
			}
		}
	}
	bool changed = previous_sig != TCAL_CONFIG_SIG ||
		retained->tempCalState.count != kept ||
		memcmp(retained->tempCalPoints, rebinned, sizeof(rebinned)) != 0;
	if (!changed) return;
	sensor_tcal_cache_invalidate();
	memcpy(retained->tempCalPoints, rebinned, sizeof(rebinned));
	memset(retained->tempCalCoeffs, 0, sizeof(retained->tempCalCoeffs));
	retained->tempCalState.count = kept;
	retained->tempCalState.valid = kept > 0;
	retained->tempCalState.degree = 0;
	retained->tempCalState.config_sig = TCAL_CONFIG_SIG;
	printk("T-Cal: layout %u -> %u, %u bins kept, %u invalid points ignored; NVS unchanged at boot\n",
	       previous_sig, TCAL_CONFIG_SIG, kept, rejected);
}

void sensor_tcal_clear(void)
{
	if (sensor_calibration_request(0) != 0) {
		LOG_ERR("Another calibration is running. Cannot clear T-Cal data.");
		printk("Error: Another calibration is running.\n");
		return;
	}

	// Invalidate lookup cache since calibration data will be cleared
	sensor_tcal_cache_invalidate();

	// Reset temperature direction tracking
	tcal_current_direction = TCAL_DIR_UNKNOWN;
	tcal_direction_ref_temp = NAN;

	LOG_INF("Clearing all manual T-Cal data.");
	memset(retained->tempCalPoints, 0, sizeof(retained->tempCalPoints));
	memset(retained->tempCalCoeffs, 0, sizeof(retained->tempCalCoeffs));
	memset(&retained->tempCalState, 0, sizeof(retained->tempCalState)); // Clear the whole state struct
	retained->tempCalState.config_sig = TCAL_CONFIG_SIG; // cleared table still belongs to this layout

	// Save cleared state to NVS directly (don't call update_tcal_state which refreshes runtime state)
	sys_write(
		MAIN_GYRO_TCAL_STATE_ID,
		&retained->tempCalState,
		&retained->tempCalState,
		sizeof(retained->tempCalState)
	);
	sys_write(
		MAIN_GYRO_TCAL_POINTS_ID,
		retained->tempCalPoints,
		retained->tempCalPoints,
		sizeof(retained->tempCalPoints)
	);
	sys_write(
		MAIN_GYRO_TCAL_COEFFS_ID,
		retained->tempCalCoeffs,
		retained->tempCalCoeffs,
		sizeof(retained->tempCalCoeffs)
	);

	// Also clear boot/runtime calibration D_offset since T-Cal is being reset
	retained->bootCalState.doffset_valid = false;
	retained->bootCalState.doffset[0] = 0.0f;
	retained->bootCalState.doffset[1] = 0.0f;
	retained->bootCalState.doffset[2] = 0.0f;
	LOG_INF("Clearing D_offset along with T-Cal data");

	// Reset continuous accumulator sampling state
	tcal_accum_reset();

	// Manual command: invalidate fusion to force quaternion recalculation
	sensor_fusion_invalidate();

	set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_CALIBRATION);
	printk("All temperature calibration data and legacy offset state have been cleared.\n");
}

// Public function for 'tcal remove <index>'
void sensor_tcal_remove_point(int index_to_remove)
{
	if (sensor_calibration_request(0) != 0) {
		LOG_ERR("Another calibration is running. Cannot remove T-Cal point.");
		printk("Error: Another calibration is running.\n");
		return;
	}

	if (index_to_remove < 0 || index_to_remove >= TCAL_BUFFER_SIZE) {
		printk("Error: Index %d is out of valid range (0 to %d).\n", index_to_remove, TCAL_BUFFER_SIZE - 1);
		return;
	}

	// Check if there was actually data in that slot
	if (retained->tempCalPoints[index_to_remove].temp != 0.0f) {
		// Invalidate lookup cache since a point is being removed
		sensor_tcal_cache_invalidate();

		LOG_INF("Removing T-Cal point at index %d.", index_to_remove);

		// Zero out the slot
		retained->tempCalPoints[index_to_remove].temp = 0.0f;
		memset(retained->tempCalPoints[index_to_remove].bias, 0, sizeof(retained->tempCalPoints[index_to_remove].bias));

		// Recalculate the count by scanning all points
		uint16_t new_count = 0;
		for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
			if (retained->tempCalPoints[i].temp != 0.0f) {
				new_count++;
			}
		}
		retained->tempCalState.count = new_count;
		retained->tempCalState.valid = false;

		printk("Point at index %d removed. Recalculating MLS state...\n", index_to_remove);
		update_tcal_state(); // Refresh and save
	} else {
		printk("No data found at index %d. Nothing to remove.\n", index_to_remove);
	}
}

// Check if current temperature needs calibration (missing nearby calibration point)
bool sensor_tcal_is_temp_outside_range(float temp, float *min_temp, float *max_temp)
{
	if (retained->tempCalState.count < 1) {
		if (min_temp) {
			*min_temp = NAN;
		}
		if (max_temp) {
			*max_temp = NAN;
		}
		return true; // No calibration data, need calibration
	}

	// Calculate the sampling interval from Kconfig
	// CONFIG_SENSOR_POLY_STEPS_PER_DEGREE: e.g., 2 means 0.5°C steps, 1 means 1.0°C steps
	float sampling_interval = 1.0f / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE;

	// Find the closest calibration point
	float closest_distance = INFINITY;
	float closest_temp = NAN;

	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (retained->tempCalPoints[i].temp != 0.0f) {
			float distance = fabsf(retained->tempCalPoints[i].temp - temp);
			if (distance < closest_distance) {
				closest_distance = distance;
				closest_temp = retained->tempCalPoints[i].temp;
			}
		}
	}

	// Return the closest point info if requested
	if (min_temp) {
		*min_temp = closest_temp;
	}
	if (max_temp) {
		*max_temp = closest_distance;
	}

	// Need calibration if closest point is farther than sampling interval
	// Add small tolerance (5%) to avoid triggering at boundary
	return (closest_distance > sampling_interval * 1.05f);
}

static field_temperature_t field_temperature = {.stable_since=-1, .cold_since=-1, .noise=0.05f};
static atomic_t field_tcal_request; /* 1=on, 2=manual off; consumed by sensor thread */

void sensor_tcal_set_auto_calibration(bool enabled)
{
    atomic_set(&field_tcal_request, enabled ? 1 : 2);
}

static void tcal_collection_apply(bool enabled)
{
    bool old = tcal_auto_calibration_enabled;
    tcal_auto_calibration_enabled = enabled;
    field_temperature_manual(&field_temperature, enabled);
    if (old != enabled) {
        tcal_accum_reset();
        set_led(enabled ? SYS_LED_PATTERN_CALIBRATION : SYS_LED_PATTERN_OFF,
                SYS_LED_PRIORITY_SENSOR);
        printk("T-Cal collection %s (rest-gated, temperature-bucketed); %u existing points kept\n",
               enabled ? "enabled" : "disabled", retained->tempCalState.count);
    }
}

static bool tcal_persist_for_pause(void)
{
    /* Small automatic EMA updates intentionally avoid routine flash writes.
     * A manual pause is the snapshot boundary: persist those existing points
     * before reporting OFF, so a calibration/reboot sees the same table. */
    if (tcal_defer_block_write(connection_get_data_collection() || f04_summary_active())) {
        printk("T-Cal pause rejected: recording active\n");
        return false;
    }
    update_tcal_state();
    static uint8_t points[sizeof(retained->tempCalPoints)];
    static uint8_t state[sizeof(retained->tempCalState)];
    sys_read(MAIN_GYRO_TCAL_POINTS_ID, points, sizeof(points));
    sys_read(MAIN_GYRO_TCAL_STATE_ID, state, sizeof(state));
    if (memcmp(points, retained->tempCalPoints, sizeof(points)) != 0 ||
        memcmp(state, &retained->tempCalState, sizeof(state)) != 0) {
        printk("T-Cal pause rejected: NVS readback mismatch\n");
        return false;
    }
    printk("T-Cal automatic observations saved and read back\n");
    return true;
}

void sensor_tcal_temperature_watch(float temp, bool rest)
{
    if(calwire_active())return; /* Defer automatic collection during frozen export. */
    int request=atomic_set(&field_tcal_request,0);
    if (request==1) {
        if (sensor_earth_cal_active() || sensor_gyro_scale_active()) {
            printk("T-Cal rejected: another calibration active.\n");
            return;
        }
        tcal_user_off = false;
        tcal_collection_apply(true);
    } else if (request==2) {
        if (!tcal_persist_for_pause()) return;
        tcal_user_off = true;
        tcal_verify_mode = false;
        tcal_collection_apply(false);
    }

    /* Never collect without USB power: stop as soon as the charger/host
     * goes away, even if the collection was started by a command. */
    if (tcal_auto_calibration_enabled && !get_status(SYS_STATUS_USB_CONNECTED) &&
        !sensor_earth_cal_active() && !sensor_gyro_scale_active()) {
        printk("T-Cal stopped: USB power lost\n");
        tcal_collection_apply(false);
        return;
    }

    if (tcal_auto_calibration_enabled || tcal_user_off) {
        return;
    }
    /* Policy check is rate limited: coverage_ready() scans buckets. */
    static int64_t last_policy_ms;
    int64_t policy_now = k_uptime_get();
    if (policy_now - last_policy_ms < 1000) {
        return;
    }
    last_policy_ms = policy_now;

    /* Never compete with the boot sequence: wait for the boot zero-rate
     * calibration to finish, then require 3 minutes of uninterrupted rest. */
    static int64_t policy_rest_since;
    static bool policy_rest_seen;
    if (!rest || !get_status(SYS_STATUS_USB_CONNECTED) || !vqf_boot_pose_valid() ||
        !sensor_boot_zro_ready() || sensor_earth_cal_active() || sensor_gyro_scale_active()) {
        policy_rest_seen = false;
        return;
    }
    if (!policy_rest_seen) {
        policy_rest_seen = true;
        policy_rest_since = policy_now;
        printk("T-Cal auto: rest timer started (3 min before first collection)\n");
        return;
    }
    if (policy_now - policy_rest_since < 180000) {
        return;
    }
    if (!isfinite(temp) || temp < (float)CONFIG_SENSOR_POLY_TEMP_MIN ||
        temp >= (float)CONFIG_SENSOR_POLY_TEMP_MAX) {
        return;
    }
    bool enough = (retained->tempCalState.count >= TCAL_BUFFER_SIZE) ||
                  sensor_tcal_coverage_ready();
    int64_t now = k_uptime_get();
    if (!enough) {
        printk("T-Cal auto: powered + rest + pose OK -> collection enabled\n");
        tcal_collection_apply(true);
    } else {
        /* A1: policy lives in tcal_defer.c so it can be unit tested on the host.
         * `recording` is the explicit collection-session state, never inferred. */
        bool recording = connection_get_data_collection() || f04_summary_active();
        tcal_action_t act = tcal_defer_step(&tcal_defer, now, recording,
                                           tcal_verify_mode);

        if (act != tcal_last_action) {
            if (act == TCAL_ACT_DEFER) {
                printk("T-Cal A1: verification deferred (recording active, count=%u)\n",
                       (unsigned)tcal_defer.defer_count);
            } else if (act == TCAL_ACT_SKIP) {
                printk("T-Cal A1: defer limit reached, run skipped "
                       "(deferrals=%u skips=%u)\n",
                       (unsigned)tcal_defer.defer_count,
                       (unsigned)tcal_defer.skip_count);
            } else if (act == TCAL_ACT_RUN && tcal_last_action != TCAL_ACT_NONE) {
                printk("T-Cal A1: running verification after deferral "
                       "(deferrals=%u skips=%u)\n",
                       (unsigned)tcal_defer.defer_count,
                       (unsigned)tcal_defer.skip_count);
            }
            tcal_last_action = act;
        }

        if (act != TCAL_ACT_RUN) {
            return;
        }

        tcal_verify_mode = true;
        tcal_verify_next_ms = tcal_defer.verify_next_ms;
        printk("T-Cal auto: verification window (next in %lld s, deferrals=%u skips=%u)\n",
               (long long)(TCAL_VERIFY_INTERVAL_MS / 1000),
               (unsigned)tcal_defer.defer_count,
               (unsigned)tcal_defer.skip_count);
        tcal_collection_apply(true);
    }
}

bool sensor_tcal_coverage_ready(void)
{
    float current=sensor_get_current_imu_temperature();
    if (!isfinite(current) || current<CONFIG_SENSOR_POLY_TEMP_MIN || current>=CONFIG_SENSOR_POLY_TEMP_MAX || !mls_lut.valid) return false;
    int center=TEMP_TO_IDX(current), nearby=0;
    bool center_valid=false;
    int lo=TEMP_TO_IDX(current-1.0f), hi=TEMP_TO_IDX(current+1.0f);
    for(int i=lo;i<=hi;i++) {
        if(i<0 || i>=TCAL_BUFFER_SIZE) continue;
        float t=retained->tempCalPoints[i].temp;
        if(!isfinite(t) || t==0 || fabsf(t-current)>1.0f || TEMP_TO_IDX(t)!=i) continue;
        bool valid=true;
        for(int k=0;k<3;k++) if(!isfinite(retained->tempCalPoints[i].bias[k])) valid=false;
        if (!valid) continue;
        nearby++;
        if(i==center) center_valid=true;
    }
    return center_valid && nearby>=TCAL_MIN_CURVE_POINTS;
}

// Get auto-calibration enabled status
bool sensor_tcal_get_auto_calibration(void)
{
	return tcal_auto_calibration_enabled;
}

/* Shift every stored temperature point by a constant vector. Used to
 * re-reference a table collected in a 180-degree rotated pose without
 * re-collecting it (the Earth projection in the table differs by 2*c). */
void sensor_tcal_shift(float dx, float dy, float dz)
{
	int count = 0;

	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (retained->tempCalPoints[i].temp != 0.0f) {
			retained->tempCalPoints[i].bias[0] += dx;
			retained->tempCalPoints[i].bias[1] += dy;
			retained->tempCalPoints[i].bias[2] += dz;
			count++;
		}
	}
	if (count == 0) {
		printk("T-Cal: no points to shift.\n");
		return;
	}

	sensor_tcal_cache_invalidate();
	sys_write(
		MAIN_GYRO_TCAL_POINTS_ID,
		retained->tempCalPoints,
		retained->tempCalPoints,
		sizeof(retained->tempCalPoints)
	);
	float t = sensor_get_current_imu_temperature();
	if (!isnan(t)) {
		sensor_tcal_build_lut_priority(t);
	}
	printk("T-Cal: shifted %d points by (%.5f, %.5f, %.5f).\n", count,
	       (double)dx, (double)dy, (double)dz);
}

/* Set one stored temperature point by bucket index. Used to restore a table
 * from a dump without re-collecting it. */
void sensor_tcal_set_point(int idx, float bx, float by, float bz)
{
	if (idx < 0 || idx >= TCAL_BUFFER_SIZE) {
		printk("T-Cal: index %d out of range.\n", idx);
		return;
	}
	float bucket_temp = IDX_TO_TEMP(idx) + (0.5f / CONFIG_SENSOR_POLY_STEPS_PER_DEGREE);
	if (retained->tempCalPoints[idx].temp == 0.0f) {
		retained->tempCalState.count++;
	}
	retained->tempCalPoints[idx].temp = bucket_temp;
	retained->tempCalPoints[idx].bias[0] = bx;
	retained->tempCalPoints[idx].bias[1] = by;
	retained->tempCalPoints[idx].bias[2] = bz;
	sensor_tcal_cache_invalidate();
	sys_write(
		MAIN_GYRO_TCAL_POINTS_ID,
		retained->tempCalPoints,
		retained->tempCalPoints,
		sizeof(retained->tempCalPoints)
	);
	sys_write(
		MAIN_GYRO_TCAL_STATE_ID,
		&retained->tempCalState,
		&retained->tempCalState,
		sizeof(retained->tempCalState)
	);
	float t = sensor_get_current_imu_temperature();
	if (!isnan(t)) {
		sensor_tcal_build_lut_priority(t);
	}
}

// =============================================================================
// Continuous Accumulator Implementation
// =============================================================================

static void tcal_accum_reset(void)
{
	memset(&tcal_accum, 0, sizeof(tcal_accum));
	tcal_accum.temp_min = INFINITY;
	tcal_accum.temp_max = -INFINITY;
}

/**
 * Save a calibration point with hysteresis-aware blending.
 * Keep the measured temperature paired with its measured bias. Bucket identity
 * is independent of the stored temperature within that bucket.
 */
static void tcal_save_point(int idx, float measured_temp, const float bias[3])
{
	if (idx < 0 || idx >= TCAL_BUFFER_SIZE || !isfinite(measured_temp) ||
	    TEMP_TO_IDX(measured_temp) != idx) {
		LOG_WRN("T-Cal: Index %d out of range, skipping", idx);
		return;
	}

	float point_temp = measured_temp;

	// Update temperature direction tracking
	if (!isnan(tcal_direction_ref_temp)) {
		float delta = measured_temp - tcal_direction_ref_temp;
		if (delta > 0.2f) {
			tcal_current_direction = TCAL_DIR_RISING;
		} else if (delta < -0.2f) {
			tcal_current_direction = TCAL_DIR_FALLING;
		}
	}
	tcal_direction_ref_temp = measured_temp;

	float final_bias[3];
	memcpy(final_bias, bias, sizeof(final_bias));

	// Hysteresis-aware blending when overwriting existing point.
	// Use tcal_current_direction directly — inferring direction from bucket center
	// temperatures is unreliable since same-bucket overwrites always have equal temps.
	bool is_new_point = (retained->tempCalPoints[idx].temp == 0.0f);
	if (!is_new_point) {
		// Select EMA alpha based on approach direction, preferring rising-phase data
		float ema_alpha;
		switch (tcal_current_direction) {
		case TCAL_DIR_RISING:  ema_alpha = TCAL_HYSTERESIS_EMA_RISING;  break;
		case TCAL_DIR_FALLING: ema_alpha = TCAL_HYSTERESIS_EMA_FALLING; break;
		default:               ema_alpha = TCAL_HYSTERESIS_EMA_UNKNOWN; break;
		}
		/* A large disagreement means the stored point is likely stale or was
		 * collected during motion: replace it instead of averaging it in. */
		float disagreement = 0.0f;
		for (int axis = 0; axis < 3; axis++) {
			float d = fabsf(final_bias[axis] - retained->tempCalPoints[idx].bias[axis]);
			if (d > disagreement) disagreement = d;
		}
		if (disagreement >= TCAL_POINT_REPLACE_THRESHOLD) {
			/* Confirm with repeated, mutually consistent windows before replacing. */
			bool consistent = false;
			if (tcal_pending_replace.idx == idx) {
				float spread = 0.0f;
				for (int axis = 0; axis < 3; axis++) {
					float d = fabsf(final_bias[axis] - tcal_pending_replace.bias[axis]);
					if (d > spread) spread = d;
				}
				consistent = (spread < TCAL_REPLACE_AGREE_TOL);
			}
			if (!consistent) {
				tcal_pending_replace.idx = idx;
				tcal_pending_replace.count = 1;
			} else {
				tcal_pending_replace.count++;
			}
			memcpy(tcal_pending_replace.bias, final_bias, sizeof(final_bias));

			if (tcal_pending_replace.count < TCAL_REPLACE_CONFIRM_COUNT) {
				LOG_INF("T-Cal: replace pending idx %d (diff %.4f, %u/%u)",
				        idx, (double)disagreement, tcal_pending_replace.count,
				        TCAL_REPLACE_CONFIRM_COUNT);
				return; /* keep the stored point until confirmed */
			}
			LOG_INF("T-Cal: Replacing idx %d after %u consistent windows (diff %.4f)",
			        idx, tcal_pending_replace.count, (double)disagreement);
			tcal_pending_replace.idx = -1;
		} else {
			tcal_pending_replace.idx = -1;
			LOG_INF("T-Cal: Blending at idx %d (dir: %s, alpha: %.2f, diff %.4f)",
			        idx,
			        tcal_current_direction == TCAL_DIR_RISING ? "rising" :
			        tcal_current_direction == TCAL_DIR_FALLING ? "falling" : "unknown",
			        (double)ema_alpha, (double)disagreement);
			for (int axis = 0; axis < 3; axis++) {
				final_bias[axis] = ema_alpha * final_bias[axis] +
				                   (1.0f - ema_alpha) * retained->tempCalPoints[idx].bias[axis];
			}
			point_temp = ema_alpha * measured_temp +
			             (1.0f - ema_alpha) * retained->tempCalPoints[idx].temp;
		}
		LOG_INF("T-Cal: Blended bias: %.5f %.5f %.5f",
		        (double)final_bias[0], (double)final_bias[1], (double)final_bias[2]);
	} else {
		retained->tempCalState.count++;
	}

	// Check if the change is significant enough to warrant a flash write.
	// New points always write; existing points only write if bias changed meaningfully.
	float max_delta = 0.0f;
	if (!is_new_point) {
		for (int axis = 0; axis < 3; axis++) {
			float d = fabsf(final_bias[axis] - retained->tempCalPoints[idx].bias[axis]);
			if (d > max_delta) {
				max_delta = d;
			}
		}
	}

	retained->tempCalPoints[idx].temp = point_temp;
	memcpy(retained->tempCalPoints[idx].bias, final_bias, sizeof(float) * 3);
	retained->tempCalState.valid = false;

	LOG_INF("T-Cal: Committed point at idx %d (%.2fC): [%.5f, %.5f, %.5f] (delta: %.4f dps)",
	        idx, (double)point_temp,
	        (double)final_bias[0], (double)final_bias[1], (double)final_bias[2],
	        (double)max_delta);

	if (is_new_point || max_delta >= TCAL_SAVE_SIGNIFICANCE_THRESHOLD) {
		// Significant change or first write: persist to NVS and rebuild LUT
		update_tcal_state();
	} else {
		// Minor update: refresh LUT cache so runtime lookup sees the new value
		// without paying the cost of a full NVS write cycle
		LOG_DBG("T-Cal: Change below threshold (%.4f < %.4f), skipping NVS write",
		        (double)max_delta, (double)TCAL_SAVE_SIGNIFICANCE_THRESHOLD);
		sensor_tcal_cache_invalidate();
	}
}

/**
 * Flush the accumulator: compute average bias/temperature, save to the
 * appropriate temperature bucket.
 */
static void tcal_accum_flush(void)
{
	if (!tcal_accum.active || tcal_accum.sample_count < TCAL_ACCUM_MIN_SAMPLES) {
		return;
	}

	/* A1: a flush writes flash (sys_write + tcal_save_point) and would stall raw
	 * TX. If a collection session is active - even one that started after the
	 * verification window was opened - postpone the whole flush and keep the
	 * accumulator. The time-based trigger stays expired, so this is retried and
	 * completes as soon as recording stops.
	 * Uses the same helper the host tests exercise; it is a best-effort gate,
	 * since an already-started flash write cannot be interrupted. */
	if (tcal_defer_block_write(connection_get_data_collection() || f04_summary_active())) {
		return;
	}

	// Compute averages
	float avg_bias[3];
	for (int axis = 0; axis < 3; axis++) {
		avg_bias[axis] = (float)(tcal_accum.gyro_sum[axis] / tcal_accum.sample_count);
	}

	float avg_temp = (tcal_accum.temp_count > 0)
	                 ? (float)(tcal_accum.temp_sum / tcal_accum.temp_count)
	                 : NAN;

	if (isnan(avg_temp)) {
		LOG_WRN("T-Cal: No valid temperature samples in accumulator, discarding");
		tcal_accum_reset();
		return;
	}

	// Map average temperature to bucket index
	int idx = TEMP_TO_IDX(avg_temp);
	if (idx < 0 || idx >= TCAL_BUFFER_SIZE) {
		LOG_WRN("T-Cal: Average temperature %.2fC outside calibration range, discarding",
		        (double)avg_temp);
		tcal_accum_reset();
		return;
	}

	LOG_INF("T-Cal: Flushing accumulator: %d samples, avg temp %.2fC (range %.2fC), bucket idx %d",
	        tcal_accum.sample_count, (double)avg_temp,
	        (double)(tcal_accum.temp_max - tcal_accum.temp_min), idx);

	// Update gyroTemp in retained
	bool tcal_in_verify = tcal_verify_mode;
	sys_write(MAIN_GYRO_TEMP_ID, &retained->gyroTemp, &avg_temp, sizeof(avg_temp));

	// Save to the bucket
	tcal_save_point(idx, avg_temp, avg_bias);
	tcal_accum_last_commit_time = k_uptime_get();
	if (tcal_in_verify) {
		tcal_verify_mode = false;
		/* A1.3: a postponed window may be older than multiple intervals.
		 * Rebase the schedule only after its real Flash flush completes. */
		tcal_defer_window_closed(&tcal_defer, k_uptime_get());
		tcal_verify_next_ms = tcal_defer.verify_next_ms;
		tcal_collection_apply(false);
	}

	// Reset accumulator for next collection period
	tcal_accum_reset();
}

/**
 * Feed a gyro sample into the continuous accumulator.
 * Called from sensor_calibration_process_gyro for every raw gyro sample.
 *
 * The accumulator collects data continuously. Periodically it is flushed
 * (by time or by temperature drift) to save the averaged result.
 *
 * @param g Raw gyro reading (before bias subtraction)
 * @param temp Current IMU temperature
 */
/* Gravity checks the reference face, not heading. The operator must also keep
 * the original heading throughout a thermal sweep. */
static bool tcal_at_reference_pose(void)
{
	int64_t age = k_uptime_get() - field_accel_time;
	if (field_accel_time <= 0 || age < 0 || age > 200) {
		return false;
	}
	float n = sqrtf(field_accel[0]*field_accel[0] + field_accel[1]*field_accel[1] +
	                field_accel[2]*field_accel[2]);
	if (!isfinite(n) || n < 0.7f || n > 1.3f) {
		return false;
	}
	return field_accel[0] / n > 0.985f; /* within ~10 deg of +X up */
}

void sensor_tcal_feed_continuous_sample(const float g[3], float temp, bool rest)
{
	if (!tcal_auto_calibration_enabled || !rest) {
		tcal_accum_reset();
		return;
	}
	if (!tcal_at_reference_pose()) {
		/* Never join windows across a tilt or a stale acceleration interval. */
		tcal_accum_reset();
		return;
	}

	// Validate temperature
	if (!isfinite(temp) || temp < (float)CONFIG_SENSOR_POLY_TEMP_MIN ||
	    temp >= (float)CONFIG_SENSOR_POLY_TEMP_MAX ||
	    !isfinite(g[0]) || !isfinite(g[1]) || !isfinite(g[2])) {
		tcal_accum_reset();
		return;
	}

	// Initialize accumulator on first sample
	if (!tcal_accum.active) {
		tcal_accum_reset();
		tcal_accum.active = true;
		tcal_accum.start_time = k_uptime_get();
		tcal_accum.min_g[0] = g[0]; tcal_accum.min_g[1] = g[1]; tcal_accum.min_g[2] = g[2];
		tcal_accum.max_g[0] = g[0]; tcal_accum.max_g[1] = g[1]; tcal_accum.max_g[2] = g[2];
		tcal_accum.temp_min = temp;
		tcal_accum.temp_max = temp;
	}

	// Outlier rejection: keep min/max for diagnostics, but never reset on a
	// single glitch sample (the old min/max check did, which discarded whole
	// windows during slow cooling). A sample far from the window mean is
	// skipped; only a run of consecutive outliers counts as motion.
	for (int j = 0; j < 3; j++) {
		if (g[j] < tcal_accum.min_g[j]) tcal_accum.min_g[j] = g[j];
		if (g[j] > tcal_accum.max_g[j]) tcal_accum.max_g[j] = g[j];
	}
	if (tcal_accum.sample_count > 0) {
		double n = (double)tcal_accum.sample_count;
		bool outlier = false;
		for (int j = 0; j < 3; j++) {
			double mean = tcal_accum.gyro_sum[j] / n;
			if (fabs((double)g[j] - mean) > (double)TCAL_ACCUM_OUTLIER_DEV) {
				outlier = true;
			}
		}
		if (outlier) {
			tcal_accum.outlier_count++;
			if (tcal_accum.outlier_count >= TCAL_ACCUM_OUTLIER_RESET) {
				LOG_DBG("T-Cal: %d consecutive outliers, resetting accumulator",
				        tcal_accum.outlier_count);
				tcal_accum_reset();
			}
			return;
		}
		tcal_accum.outlier_count = 0;
	}

	// Accumulate gyro
	tcal_accum.gyro_sum[0] += (double)g[0];
	tcal_accum.gyro_sum[1] += (double)g[1];
	tcal_accum.gyro_sum[2] += (double)g[2];
	tcal_accum.sample_count++;

	// Accumulate temperature
	if (temp < tcal_accum.temp_min) tcal_accum.temp_min = temp;
	if (temp > tcal_accum.temp_max) tcal_accum.temp_max = temp;
	tcal_accum.temp_sum += (double)temp;
	tcal_accum.temp_count++;

	// Check flush conditions
	int64_t elapsed = k_uptime_get() - tcal_accum.start_time;
	float temp_drift = tcal_accum.temp_max - tcal_accum.temp_min;

	// Condition 1: Temperature drifted beyond one bucket width — flush early
	// to avoid cross-bucket contamination, then start a new accumulation window
	if (temp_drift > TCAL_ACCUM_TEMP_DRIFT_MAX &&
	    tcal_accum.sample_count >= TCAL_ACCUM_MIN_SAMPLES) {
		LOG_INF("T-Cal: Temperature drift %.2fC exceeded threshold, early flush",
		        (double)temp_drift);
		tcal_accum_flush();
		return;
	}

	// Condition 2: Flush interval reached
	if (elapsed >= TCAL_ACCUM_FLUSH_INTERVAL_MS &&
	    tcal_accum.sample_count >= TCAL_ACCUM_MIN_SAMPLES) {
		tcal_accum_flush();
		return;
	}
}

/**
 * Called when motion is detected — flush if enough data, then reset.
 */
void sensor_tcal_continuous_motion_detected(void)
{
	if (tcal_accum.active) {
		if (tcal_accum.sample_count >= TCAL_ACCUM_MIN_SAMPLES) {
			tcal_accum_flush();
		} else {
			tcal_accum_reset();
		}
	}
}

// Compatibility entry point; initial stationary calibration was removed.
void sensor_tcal_check_auto_calibration(float current_temp) { (void)current_temp; }

void sensor_calibration_get_last_gyro_offset(float offset[3])
{
	memcpy(offset, last_gyro_tcal_offset, sizeof(last_gyro_tcal_offset));
}

// =============================================================================
// Temperature-table quality assessment and legacy compatibility stubs
// =============================================================================

/**
 * Assess temperature calibration quality at the current temperature
 * Checks if MLS has enough points with significant weight at the current temperature
 */
bool sensor_tcal_assess_quality(float current_temp, tcal_quality_t *quality)
{
	if (!quality) {
		return false;
	}

	// Initialize quality structure
	quality->curve_valid = retained->tempCalState.valid;
	quality->point_count = retained->tempCalState.count;
	quality->curve_error = 0.0f;
	quality->temp_min = INFINITY;
	quality->temp_max = -INFINITY;
	quality->temp_in_range = false;

	// Check minimum global point count
	if (quality->point_count < TCAL_MIN_CURVE_POINTS) {
		LOG_DBG("T-Cal: Insufficient points (%u < %d)", quality->point_count, TCAL_MIN_CURVE_POINTS);
		return false;
	}

	// Count points with significant weight at current temperature (MLS bandwidth check)
	// This ensures MLS will actually have usable data at this temperature
	float bandwidth_sq = MLS_BANDWIDTH * MLS_BANDWIDTH;
	int points_with_weight = 0;

	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (retained->tempCalPoints[i].temp != 0.0f) {
			float point_temp = retained->tempCalPoints[i].temp;

			// Track temperature range
			if (point_temp < quality->temp_min) {
				quality->temp_min = point_temp;
			}
			if (point_temp > quality->temp_max) {
				quality->temp_max = point_temp;
			}

			// Calculate weight at current temperature
			float d = point_temp - current_temp;
			float d_sq = d * d;
			float weight = 1.0f / (1.0f + d_sq / bandwidth_sq);

			// Count if weight is significant (>= MLS_MIN_WEIGHT)
			if (weight >= MLS_MIN_WEIGHT) {
				points_with_weight++;
			}
		}
	}

	// Check if current temperature is within or near the calibrated range
	if (current_temp >= quality->temp_min && current_temp <= quality->temp_max) {
		quality->temp_in_range = true;
	}

	// MLS needs at least 2 points with significant weight for linear fit
	if (points_with_weight < MLS_MIN_POINTS_FOR_FIT) {
		LOG_DBG(
			"T-Cal: Only %d point(s) with significant weight at %.2fC (need %d within %.1fC bandwidth)",
			points_with_weight,
			(double)current_temp,
			MLS_MIN_POINTS_FOR_FIT,
			(double)MLS_BANDWIDTH
		);
		return false;
	}

	// Log quality status (only once to avoid spam)
	static bool logged_quality = false;
	if (!logged_quality) {
		LOG_INF(
			"T-Cal: Quality check passed - %d points with weight at %.2fC (range: [%.2fC, %.2fC])",
			points_with_weight,
			(double)current_temp,
			(double)quality->temp_min,
			(double)quality->temp_max
		);
		logged_quality = true;
	}

	return true;
}

/* Legacy stationary calibration entry points intentionally do not collect bias. */
void sensor_tcal_boot_calibration_check(void) {}
void sensor_runtime_calibration_check(bool resting) { (void)resting; }
void sensor_runtime_cal_get_status(int64_t *last, int64_t *duration)
{ if (last) *last=0; if (duration) *duration=0; }
void sensor_boot_cal_set_enabled(bool enabled)
{ (void)enabled; printk("Legacy static calibration removed; use T-Cal and Earth-aware VQF tracking.\n"); }
bool sensor_boot_cal_is_completed(void) { return false; }
void sensor_boot_cal_get_doffset(float out[3]) { memset(out,0,3*sizeof(float)); }
void sensor_boot_cal_reset(void) { memset(&retained->bootCalState,0,sizeof(retained->bootCalState)); }

// T-Cal Test/Debug Functions
// =============================================================================

/**
 * Test and compare different calibration methods at a given temperature
 * Useful for debugging and understanding method differences
 */
void sensor_tcal_test_methods(float temp)
{
	if (retained->tempCalState.count < 1) {
		printk("No calibration data available.\n");
		return;
	}

	printk("\n=== T-Cal Method Comparison at %.2fC ===\n", (double)temp);
	printk("Total calibration points: %u\n\n", retained->tempCalState.count);

	// Show available calibration points
	float min_temp = INFINITY, max_temp = -INFINITY;
	int point_count = 0;
	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (retained->tempCalPoints[i].temp != 0.0f) {
			float t = retained->tempCalPoints[i].temp;
			if (t < min_temp) {
				min_temp = t;
			}
			if (t > max_temp) {
				max_temp = t;
			}
			point_count++;
		}
	}
	printk("Calibrated range: %.2fC to %.2fC\n", (double)min_temp, (double)max_temp);

	// Show temperature position
	if (temp < min_temp) {
		printk("Test temp is %.2fC BELOW calibrated range\n", (double)(min_temp - temp));
	} else if (temp > max_temp) {
		printk("Test temp is %.2fC ABOVE calibrated range\n", (double)(temp - max_temp));
	} else {
		printk("Test temp is WITHIN calibrated range\n");
	}
	printk("\n");

	// Method 1: MLS (Moving Least Squares - primary method)
	if (retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT) {
		float mls_bias[3];
		int result = sensor_tcal_mls_lookup(temp, mls_bias);

		if (result == 0) {
			printk("MLS Method (bandwidth=%.1fC):\n", (double)MLS_BANDWIDTH);
			printk("  Bias: [%.5f, %.5f, %.5f] dps\n", (double)mls_bias[0], (double)mls_bias[1], (double)mls_bias[2]);
		} else {
			printk("MLS Method: FAILED\n");
		}
		printk("\n");
	} else {
		printk("MLS Method: Not enough points (need >= %d)\n\n", MLS_MIN_POINTS_FOR_FIT);
	}

	// Show final effective bias that would be applied
	printk("Final Effective Bias (as applied to gyro data):\n");

	// Match the production LUT -> MLS -> stored static fallback strategy.
	float final_bias[3] = {0.0f, 0.0f, 0.0f};
	bool calculated = false;
	const char *method_used = "static";

	if (tcal_compensation_enabled && retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT
        && sensor_tcal_lut_lookup(temp, final_bias) == 0) {
        calculated = true;
        method_used = "LUT";
    } else if (tcal_compensation_enabled && retained->tempCalState.count >= MLS_MIN_POINTS_FOR_FIT
        && sensor_tcal_mls_lookup(temp, final_bias) == 0) {
		calculated = true;
		method_used = "MLS";
	}

	if (calculated) {

		printk(
			"  Total: [%.5f, %.5f, %.5f] dps\n",
			(double)final_bias[0],
			(double)final_bias[1],
			(double)final_bias[2]
		);
		printk("  Method: %s (Unified Strategy: LUT -> MLS -> Static)\n", method_used);
	} else {
		printk(
			"  Fallback to static bias: [%.5f, %.5f, %.5f] dps\n",
			(double)retained->gyroBias[0],
			(double)retained->gyroBias[1],
			(double)retained->gyroBias[2]
		);
		printk("  (No valid T-Cal method available)\n");
	}

	printk("\n=== End of T-Cal Method Comparison ===\n");
}

/**
 * Moving Least Squares (MLS) lookup function implementation
 *
 * This function provides a unified, smooth bias estimation that:
 * - Eliminates discontinuities from method switching
 * - Naturally handles boundaries and extrapolation
 * - Uses local linear fitting with distance-based weighting
 *
 * Weight function: w(d) = 1 / (1 + (d/bandwidth)²)
 * This is a Cauchy-like weight that provides smooth falloff without exp()
 *
 * @param temp Query temperature
 * @param bias_out Output: computed 3-axis bias
 * @return 0 on success, -1 if insufficient data
 */
static int sensor_tcal_mls_lookup_internal(float temp, float bias_out[3], bool use_cache)
{
	// Check if we have any calibration data
	if (retained->tempCalState.count < 1) {
		LOG_ERR("T-Cal MLS: No calibration data available");
		return -1;
	}

	// Check multi-slot cache:
	// First, check if point count changed (invalidates all cache slots)
	if (mls_cache.count != retained->tempCalState.count) {
		// Invalidate all slots
		for (int i = 0; i < MLS_CACHE_SLOTS; i++) {
			mls_cache.slots[i].valid = false;
		}
		mls_cache.count = retained->tempCalState.count;
	}

	// A persistent LUT entry must use its own exact local fit. Reusing a
	// nearby fallback cache makes the curve depend on boot temperature and
	// incremental build order, even when the stored observations are identical.
	for (int i = 0; use_cache && i < MLS_CACHE_SLOTS; i++) {
		if (mls_cache.slots[i].valid &&
		    fabsf(mls_cache.slots[i].temp - temp) < MLS_CACHE_TEMP_THRESHOLD) {
			// Cache hit - return smoothly interpolated value using cached local slope.
			// This prevents the output from becoming piecewise-constant within the
			// cache threshold window (e.g. 36.10C vs 36.20C).
			float dt = temp - mls_cache.slots[i].temp;
			for (int axis = 0; axis < 3; axis++) {
				bias_out[axis] = mls_cache.slots[i].bias[axis] + mls_cache.slots[i].slope[axis] * dt;
			}
			return 0;
		}
	}

	// If only one point, just return it (no fitting possible)
	if (retained->tempCalState.count == 1) {
		// Find the single point
		for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
			if (retained->tempCalPoints[i].temp != 0.0f) {
				memcpy(bias_out, retained->tempCalPoints[i].bias, sizeof(float) * 3);
				// Update cache (select best slot based on distance)
				int slot = sensor_tcal_cache_select_slot(temp);
				mls_cache.slots[slot].temp = temp;
				memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
				memset(mls_cache.slots[slot].slope, 0, sizeof(mls_cache.slots[slot].slope));
				mls_cache.slots[slot].valid = true;
				LOG_DBG("T-Cal MLS: Single point at %.2fC (cached in slot %d)", (double)retained->tempCalPoints[i].temp, slot);
				return 0;
			}
		}
		return -1; // Should not reach here
	}

	// Collect points and compute weights
	// Using simple structure to hold point data with weights
	typedef struct {
		float temp;
		float bias[3];
		float weight;
	} WeightedPoint;

	// Online top-k selection: maintain only the best MLS_MAX_POINTS by weight
	// This avoids large stack allocations while scanning all buffer entries
	WeightedPoint points[MLS_MAX_POINTS];
	int point_count = 0;
	int total_valid = 0;  // total points passing weight filter

	float bandwidth_sq = MLS_BANDWIDTH * MLS_BANDWIDTH;
	float min_selected_weight = 0.0f;  // track minimum weight in selected set
	int min_selected_idx = 0;

	for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
		if (retained->tempCalPoints[i].temp == 0.0f) {
			continue; // Skip empty slots
		}

		float point_temp = retained->tempCalPoints[i].temp;
		float d = point_temp - temp;
		float d_sq = d * d;

		// Cauchy-like weight: w = 1 / (1 + (d/h)²)
		float weight = 1.0f / (1.0f + d_sq / bandwidth_sq);

		// Skip points with negligible weight
		if (weight < MLS_MIN_WEIGHT) {
			continue;
		}

		total_valid++;

		if (point_count < MLS_MAX_POINTS) {
			// Still filling the selection buffer
			points[point_count].temp = point_temp;
			points[point_count].weight = weight;
			memcpy(points[point_count].bias, retained->tempCalPoints[i].bias, sizeof(float) * 3);
			point_count++;

			// Update minimum tracking when buffer is full
			if (point_count == MLS_MAX_POINTS) {
				min_selected_weight = points[0].weight;
				min_selected_idx = 0;
				for (int j = 1; j < MLS_MAX_POINTS; j++) {
					if (points[j].weight < min_selected_weight) {
						min_selected_weight = points[j].weight;
						min_selected_idx = j;
					}
				}
			}
		} else if (weight > min_selected_weight) {
			// Replace the weakest point in our selection
			points[min_selected_idx].temp = point_temp;
			points[min_selected_idx].weight = weight;
			memcpy(points[min_selected_idx].bias, retained->tempCalPoints[i].bias, sizeof(float) * 3);

			// Find new minimum
			min_selected_weight = points[0].weight;
			min_selected_idx = 0;
			for (int j = 1; j < MLS_MAX_POINTS; j++) {
				if (points[j].weight < min_selected_weight) {
					min_selected_weight = points[j].weight;
					min_selected_idx = j;
				}
			}
		}
	}

	if (point_count == 0) {
		// No points with sufficient weight - temperature is far outside calibrated range
		// Use linear extrapolation from edge points (similar to LUT extrapolation logic)

		// Collect all valid points with their temperatures
		typedef struct {
			float temp;
			float bias[3];
		} TempPoint;
		TempPoint edge_points[MLS_EXTRAP_POINTS];
		int edge_count = 0;

		// Determine if we're extrapolating low or high
		// First scan to find data range
		float data_min_temp = 1000.0f;
		float data_max_temp = -1000.0f;

		for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
			if (retained->tempCalPoints[i].temp == 0.0f) {
				continue;
			}
			float pt = retained->tempCalPoints[i].temp;
			if (pt < data_min_temp) data_min_temp = pt;
			if (pt > data_max_temp) data_max_temp = pt;
		}

		if (data_min_temp > 999.0f) {
			// No calibration data at all
			LOG_ERR("T-Cal MLS: No calibration data available");
			return -1;
		}

		bool extrapolate_low = (temp < data_min_temp);

		if (extrapolate_low) {
			// Collect lowest temperature points for low extrapolation
			// Sort by temperature ascending, take first MLS_EXTRAP_POINTS
			for (int k = 0; k < MLS_EXTRAP_POINTS && k < retained->tempCalState.count; k++) {
				float lowest_temp = 1000.0f;
				int lowest_idx = -1;

				for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
					if (retained->tempCalPoints[i].temp == 0.0f) continue;
					float pt = retained->tempCalPoints[i].temp;

					// Check if already selected
					bool already_selected = false;
					for (int j = 0; j < edge_count; j++) {
						if (fabsf(edge_points[j].temp - pt) < 0.01f) {
							already_selected = true;
							break;
						}
					}
					if (already_selected) continue;

					if (pt < lowest_temp) {
						lowest_temp = pt;
						lowest_idx = i;
					}
				}

				if (lowest_idx >= 0) {
					edge_points[edge_count].temp = retained->tempCalPoints[lowest_idx].temp;
					memcpy(edge_points[edge_count].bias, retained->tempCalPoints[lowest_idx].bias, sizeof(float) * 3);
					edge_count++;
				}
			}
		} else {
			// Collect highest temperature points for high extrapolation
			// Sort by temperature descending, take first MLS_EXTRAP_POINTS
			for (int k = 0; k < MLS_EXTRAP_POINTS && k < retained->tempCalState.count; k++) {
				float highest_temp = -1000.0f;
				int highest_idx = -1;

				for (int i = 0; i < TCAL_BUFFER_SIZE; i++) {
					if (retained->tempCalPoints[i].temp == 0.0f) continue;
					float pt = retained->tempCalPoints[i].temp;

					// Check if already selected
					bool already_selected = false;
					for (int j = 0; j < edge_count; j++) {
						if (fabsf(edge_points[j].temp - pt) < 0.01f) {
							already_selected = true;
							break;
						}
					}
					if (already_selected) continue;

					if (pt > highest_temp) {
						highest_temp = pt;
						highest_idx = i;
					}
				}

				if (highest_idx >= 0) {
					edge_points[edge_count].temp = retained->tempCalPoints[highest_idx].temp;
					memcpy(edge_points[edge_count].bias, retained->tempCalPoints[highest_idx].bias, sizeof(float) * 3);
					edge_count++;
				}
			}
		}

		if (edge_count == 0) {
			LOG_ERR("T-Cal MLS: No edge points found for extrapolation");
			return -1;
		}

		// Single point: constant extrapolation
		if (edge_count == 1) {
			memcpy(bias_out, edge_points[0].bias, sizeof(float) * 3);

			int slot = sensor_tcal_cache_select_slot(temp);
			mls_cache.slots[slot].temp = temp;
			memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
			memset(mls_cache.slots[slot].slope, 0, sizeof(mls_cache.slots[slot].slope));
			mls_cache.slots[slot].valid = true;

			LOG_DBG("T-Cal MLS: Single-point extrapolation at %.2fC", (double)temp);
			return 0;
		}

		// Multiple points: linear least squares fit (same as LUT extrapolation)
		float t_mean = 0.0f;
		for (int i = 0; i < edge_count; i++) {
			t_mean += edge_points[i].temp;
		}
		t_mean /= edge_count;

		float sum_dt_sq = 0.0f;
		for (int i = 0; i < edge_count; i++) {
			float dt = edge_points[i].temp - t_mean;
			sum_dt_sq += dt * dt;
		}

		float slope[3] = {0.0f, 0.0f, 0.0f};

		for (int axis = 0; axis < 3; axis++) {
			float b_mean = 0.0f;
			for (int i = 0; i < edge_count; i++) {
				b_mean += edge_points[i].bias[axis];
			}
			b_mean /= edge_count;

			float sum_dt_db = 0.0f;
			for (int i = 0; i < edge_count; i++) {
				float dt = edge_points[i].temp - t_mean;
				float db = edge_points[i].bias[axis] - b_mean;
				sum_dt_db += dt * db;
			}

			slope[axis] = (sum_dt_sq > 0.001f) ? sum_dt_db / sum_dt_sq : 0.0f;
			bias_out[axis] = b_mean + slope[axis] * (temp - t_mean);
		}

		// Cache the extrapolated result with slope for smooth interpolation
		int slot = sensor_tcal_cache_select_slot(temp);
		mls_cache.slots[slot].temp = temp;
		memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
		memcpy(mls_cache.slots[slot].slope, slope, sizeof(float) * 3);
		mls_cache.slots[slot].valid = true;

		LOG_DBG("T-Cal MLS: Linear extrapolation %s range (%.2fC, %d pts)",
			extrapolate_low ? "below" : "above", (double)temp, edge_count);
		return 0;
	}

	// If only one point has significant weight, just return it
	if (point_count == 1) {
		memcpy(bias_out, points[0].bias, sizeof(float) * 3);

		// Cache as a locally constant model (slope = 0)
		int slot = sensor_tcal_cache_select_slot(temp);
		mls_cache.slots[slot].temp = temp;
		memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
		memset(mls_cache.slots[slot].slope, 0, sizeof(mls_cache.slots[slot].slope));
		mls_cache.slots[slot].valid = true;

		LOG_DBG(
			"T-Cal MLS: Single significant point at %.2fC (w=%.3f)",
			(double)points[0].temp,
			(double)points[0].weight
		);
		return 0;
	}

	// Perform weighted linear least squares fit for each axis
	// We solve: minimize Σ w_i * (b_i - (a + c*(t_i - t_query)))²
	//
	// Normal equations:
	//   Σ w_i * b_i = a * Σ w_i + c * Σ w_i * (t_i - t_q)
	//   Σ w_i * (t_i - t_q) * b_i = a * Σ w_i * (t_i - t_q) + c * Σ w_i * (t_i - t_q)²
	//
	// Since t_q is our query point, let delta_t = t_i - t_q, then:
	//   sum_w = Σ w_i
	//   sum_wdt = Σ w_i * delta_t_i  (often ~0 if query is centered)
	//   sum_wdt2 = Σ w_i * delta_t_i²
	//   sum_wb[axis] = Σ w_i * b_i[axis]
	//   sum_wdtb[axis] = Σ w_i * delta_t_i * b_i[axis]
	//
	// Solution (Cramer's rule):
	//   det = sum_w * sum_wdt2 - sum_wdt * sum_wdt
	//   a = (sum_wb * sum_wdt2 - sum_wdtb * sum_wdt) / det
	//   c = (sum_w * sum_wdtb - sum_wdt * sum_wb) / det
	//
	// Result at query point (delta_t = 0): bias = a

	double sum_w = 0.0;
	double sum_wdt = 0.0;
	double sum_wdt2 = 0.0;
	double sum_wb[3] = {0.0, 0.0, 0.0};
	double sum_wdtb[3] = {0.0, 0.0, 0.0};

	for (int i = 0; i < point_count; i++) {
		double w = (double)points[i].weight;
		double dt = (double)(points[i].temp - temp);

		sum_w += w;
		sum_wdt += w * dt;
		sum_wdt2 += w * dt * dt;

		for (int axis = 0; axis < 3; axis++) {
			double b = (double)points[i].bias[axis];
			sum_wb[axis] += w * b;
			sum_wdtb[axis] += w * dt * b;
		}
	}

	// Calculate determinant
	double det = sum_w * sum_wdt2 - sum_wdt * sum_wdt;

	// Check for degenerate case (all points at same temperature)
	if (fabs(det) < 1e-10) {
		// Fallback to weighted average
		for (int axis = 0; axis < 3; axis++) {
			bias_out[axis] = (float)(sum_wb[axis] / sum_w);
		}
		// Update cache (select best slot based on distance)
		int slot = sensor_tcal_cache_select_slot(temp);
		mls_cache.slots[slot].temp = temp;
		memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
		memset(mls_cache.slots[slot].slope, 0, sizeof(mls_cache.slots[slot].slope));
		mls_cache.slots[slot].valid = true;
		LOG_DBG("T-Cal MLS: Degenerate case, using weighted average at %.2fC (cached in slot %d)", (double)temp, slot);
		return 0;
	}

	// Solve for 'a' (the bias at query temperature) and local slope 'c'
	// a = (sum_wb * sum_wdt2 - sum_wdtb * sum_wdt) / det
	// c = (sum_w * sum_wdtb - sum_wdt * sum_wb) / det
	float slope_out[3];
	for (int axis = 0; axis < 3; axis++) {
		double a = (sum_wb[axis] * sum_wdt2 - sum_wdtb[axis] * sum_wdt) / det;
		double c = (sum_w * sum_wdtb[axis] - sum_wdt * sum_wb[axis]) / det;
		bias_out[axis] = (float)a;
		slope_out[axis] = (float)c;
	}

	// Update cache with computed result (select best slot based on distance)
	int slot = sensor_tcal_cache_select_slot(temp);
	mls_cache.slots[slot].temp = temp;
	memcpy(mls_cache.slots[slot].bias, bias_out, sizeof(float) * 3);
	memcpy(mls_cache.slots[slot].slope, slope_out, sizeof(float) * 3);
	mls_cache.slots[slot].valid = true;

	LOG_DBG(
		"T-Cal MLS: Computed bias [%.4f, %.4f, %.4f] at %.2fC using %d points (cached in slot %d)",
		(double)bias_out[0],
		(double)bias_out[1],
		(double)bias_out[2],
		(double)temp,
		point_count,
		slot
	);

	return 0;
}

// =============================================================================
// LUT Incremental Build Functions
// =============================================================================

/**
 * Helper function to compute and store a single LUT entry
 * @param idx LUT index to compute
 * @return true if successfully computed, false on error
 */
static int sensor_tcal_mls_lookup(float temp, float bias_out[3])
{
    return sensor_tcal_mls_lookup_internal(temp, bias_out, true);
}

static bool sensor_tcal_lut_compute_entry(int idx)
{
	if (idx < 0 || idx >= MLS_LUT_SIZE) {
		return false;
	}

	// Skip if already computed
	if (mls_lut.entries[idx].computed) {
		return true;
	}

	float temp = MLS_LUT_IDX_TO_TEMP(idx);
	float bias[3];

	if (sensor_tcal_mls_lookup_internal(temp, bias, false) == 0) {
		memcpy(mls_lut.entries[idx].bias, bias, sizeof(float) * 3);
		mls_lut.entries[idx].computed = true;
		mls_lut.computed_count++;
		return true;
	}

	return false;
}

/**
 * Build priority zone of LUT (±3°C around current temperature)
 * This is called at startup and when calibration points change.
 * Returns quickly after building the priority zone.
 * Background build continues via sensor_tcal_build_lut_continue().
 *
 * @param current_temp Current device temperature
 */
static void sensor_tcal_build_lut_priority(float current_temp)
{
	// Check if we have enough points for MLS
	if (retained->tempCalState.count < MLS_MIN_POINTS_FOR_FIT) {
		LOG_INF("T-Cal LUT: Not enough points (%u < %d), LUT disabled",
		        retained->tempCalState.count, MLS_MIN_POINTS_FOR_FIT);
		mls_lut.valid = false;
		mls_lut.build_state = MLS_LUT_BUILD_IDLE;
		return;
	}

	int64_t start_time = k_uptime_get();

	// Reset LUT state for fresh build
	mls_lut.valid = false;
	mls_lut.version = retained->tempCalState.count;
	mls_lut.computed_count = 0;

	// Mark all entries as not computed
	for (int i = 0; i < MLS_LUT_SIZE; i++) {
		mls_lut.entries[i].computed = false;
	}

	// Clear legacy cache to force fresh MLS computation
	for (int i = 0; i < MLS_CACHE_SLOTS; i++) {
		mls_cache.slots[i].valid = false;
	}
	mls_cache.count = 0;

	// Calculate priority zone indices (±3°C around current temp)
	float priority_temp_min = current_temp - MLS_LUT_PRIORITY_RANGE;
	float priority_temp_max = current_temp + MLS_LUT_PRIORITY_RANGE;

	// Clamp to LUT range
	if (priority_temp_min < MLS_LUT_TEMP_MIN) {
		priority_temp_min = MLS_LUT_TEMP_MIN;
	}
	if (priority_temp_max > MLS_LUT_TEMP_MAX) {
		priority_temp_max = MLS_LUT_TEMP_MAX;
	}

	mls_lut.priority_idx_min = (int)MLS_LUT_TEMP_TO_IDX(priority_temp_min);
	mls_lut.priority_idx_max = (int)MLS_LUT_TEMP_TO_IDX(priority_temp_max) + 1;

	// Clamp indices
	if (mls_lut.priority_idx_min < 0) {
		mls_lut.priority_idx_min = 0;
	}
	if (mls_lut.priority_idx_max >= MLS_LUT_SIZE) {
		mls_lut.priority_idx_max = MLS_LUT_SIZE - 1;
	}

	LOG_INF("T-Cal LUT: Building priority zone [%d-%d] (%.1f°C to %.1f°C)",
	        mls_lut.priority_idx_min, mls_lut.priority_idx_max,
	        (double)priority_temp_min, (double)priority_temp_max);

	mls_lut.build_state = MLS_LUT_BUILD_PRIORITY;

	// Build priority zone entries
	int priority_count = 0;
	for (int idx = mls_lut.priority_idx_min; idx <= mls_lut.priority_idx_max; idx++) {
		if (sensor_tcal_lut_compute_entry(idx)) {
			priority_count++;
		}

		// Feed watchdog periodically
		if (priority_count % 20 == 0) {
			watchdog_feed(WDT_CHANNEL_CALIBRATION);
		}
	}

	// Mark LUT as valid once priority zone is complete
	mls_lut.valid = true;

	// Set up for background build of remaining entries
	mls_lut.build_next_idx = 0;
	mls_lut.build_state = MLS_LUT_BUILD_BACKGROUND;

	int64_t elapsed = k_uptime_get() - start_time;
	LOG_INF("T-Cal LUT: Priority zone built (%d entries) in %lld ms, background build started",
	        priority_count, elapsed);
}

/**
 * Continue building LUT entries in background.
 * Called from calibration_thread main loop.
 * Processes a small batch of entries per call to avoid blocking.
 *
 * @return true if build is complete, false if more work remains
 */
static bool sensor_tcal_build_lut_continue(void)
{
	// Check if build is needed
	if (mls_lut.build_state != MLS_LUT_BUILD_BACKGROUND) {
		return true;  // Not in background build state
	}

	// Check version match
	if (mls_lut.version != retained->tempCalState.count) {
		// Points changed, invalidate and stop
		mls_lut.build_state = MLS_LUT_BUILD_IDLE;
		mls_lut.valid = false;
		return true;
	}

	// Process a batch of entries
	int computed_this_batch = 0;
	while (computed_this_batch < MLS_LUT_BATCH_SIZE && mls_lut.build_next_idx < MLS_LUT_SIZE) {
		int idx = mls_lut.build_next_idx;
		mls_lut.build_next_idx++;

		// Skip already computed entries (priority zone)
		if (mls_lut.entries[idx].computed) {
			continue;
		}

		sensor_tcal_lut_compute_entry(idx);
		computed_this_batch++;
	}

	// Check if complete
	if (mls_lut.build_next_idx >= MLS_LUT_SIZE) {
		mls_lut.build_state = MLS_LUT_BUILD_COMPLETE;
		return true;
	}

	// Yield CPU time
	k_msleep(MLS_LUT_BATCH_YIELD_MS);
	return false;
}

// =============================================================================
// LUT Lookup Function - O(1) Linear Interpolation
// =============================================================================
/**
 * Fast O(1) lookup using pre-computed LUT with linear interpolation.
 * For entries not yet computed (during incremental build), falls back to -1.
 *
 * @param temp Query temperature
 * @param bias_out Output: interpolated 3-axis bias
 * @return 0 on success, -1 if required entries not computed
 */
/* Read-only export of the actual runtime curve, including cache effects.
 * Host reconstruction requires a complete LUT and disabled collection. */
void sensor_tcal_dump_lut(void)
{
    printk("TCAL_LUT valid=%d count=%u version=%u points=%u\n",
           mls_lut.valid, mls_lut.computed_count, mls_lut.version,
           retained->tempCalState.count);
    for (int i = 0; i < MLS_LUT_SIZE; i++) {
        if (!mls_lut.entries[i].computed) continue;
        printk("LUT %d %.9f %.9f %.9f %.9f\n", i,
               (double)MLS_LUT_IDX_TO_TEMP(i),
               (double)mls_lut.entries[i].bias[0],
               (double)mls_lut.entries[i].bias[1],
               (double)mls_lut.entries[i].bias[2]);
        if (i % 8 == 7) k_msleep(20);
    }
    printk("TCAL_LUT END\n");
}

size_t sensor_tcal_wire_snapshot(uint8_t *out,size_t capacity)
{
    size_t bytes=12+sizeof(retained->tempCalPoints)+MLS_LUT_SIZE*16;
    if(capacity<bytes)return 0;
    unsigned key=irq_lock();
    if(tcal_auto_calibration_enabled||!mls_lut.valid||mls_lut.version!=retained->tempCalState.count||
       mls_lut.computed_count!=MLS_LUT_SIZE){irq_unlock(key);return 0;}
    memset(out,0,12);
    cw_put16(out,retained->tempCalState.count);cw_put16(out+2,TCAL_BUFFER_SIZE);
    cw_put16(out+4,MLS_LUT_SIZE);cw_put16(out+6,mls_lut.version);
    cw_put16(out+8,retained->tempCalState.config_sig);out[10]=retained->tcal_enabled;
    memcpy(out+12,retained->tempCalPoints,sizeof(retained->tempCalPoints));
    uint8_t *row=out+12+sizeof(retained->tempCalPoints);
    for(unsigned i=0;i<MLS_LUT_SIZE;i++,row+=16){
        float temp=MLS_LUT_IDX_TO_TEMP(i);memcpy(row,&temp,4);
        memcpy(row+4,mls_lut.entries[i].bias,12);
    }
    irq_unlock(key);return bytes;
}

static int sensor_tcal_lut_lookup(float temp, float bias_out[3])
{
	// Check LUT validity and version
	if (!mls_lut.valid || mls_lut.version != retained->tempCalState.count) {
		return -1;  // LUT not available
	}

	// Track if we need to extrapolate (temp outside LUT range)
	bool extrapolate_low = (temp < MLS_LUT_TEMP_MIN);
	bool extrapolate_high = (temp > MLS_LUT_TEMP_MAX);
	float original_temp = temp;

	// For extrapolation, use 4 points with 1°C spacing for robust slope estimation
	// With MLS_LUT_STEP_PER_DEGREE = 2, 1°C = 2 LUT indices
	#define EXTRAP_POINT_SPACING MLS_LUT_STEP_PER_DEGREE  // 1°C spacing
	#define EXTRAP_NUM_POINTS 4

	if (extrapolate_low) {
		// Use first 4 points at 1°C intervals: indices 0, 2, 4, 6
		int indices[EXTRAP_NUM_POINTS];
		for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
			indices[i] = i * EXTRAP_POINT_SPACING;
			if (indices[i] >= MLS_LUT_SIZE) {
				return -1;  // Not enough range for extrapolation
			}
			if (!mls_lut.entries[indices[i]].computed) {
				return -1;  // Required points not computed
			}
		}

		// Linear least squares fit using 4 points
		// slope = Σ(t_i - t_mean)(b_i - b_mean) / Σ(t_i - t_mean)²
		float temps[EXTRAP_NUM_POINTS];
		float t_mean = 0.0f;
		for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
			temps[i] = MLS_LUT_IDX_TO_TEMP(indices[i]);
			t_mean += temps[i];
		}
		t_mean /= EXTRAP_NUM_POINTS;

		// Pre-compute Σ(t_i - t_mean)² (same for all axes)
		float sum_dt_sq = 0.0f;
		for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
			float dt = temps[i] - t_mean;
			sum_dt_sq += dt * dt;
		}

		for (int axis = 0; axis < 3; axis++) {
			float b_mean = 0.0f;
			for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
				b_mean += mls_lut.entries[indices[i]].bias[axis];
			}
			b_mean /= EXTRAP_NUM_POINTS;

			float sum_dt_db = 0.0f;
			for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
				float dt = temps[i] - t_mean;
				float db = mls_lut.entries[indices[i]].bias[axis] - b_mean;
				sum_dt_db += dt * db;
			}

			float slope = (sum_dt_sq > 0.001f) ? sum_dt_db / sum_dt_sq : 0.0f;
			bias_out[axis] = b_mean + slope * (original_temp - t_mean);
		}
		return 0;

	} else if (extrapolate_high) {
		// Use last 4 points at 1°C intervals: indices n-1, n-3, n-5, n-7
		int indices[EXTRAP_NUM_POINTS];
		for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
			indices[i] = (MLS_LUT_SIZE - 1) - i * EXTRAP_POINT_SPACING;
			if (indices[i] < 0) {
				return -1;  // Not enough range for extrapolation
			}
			if (!mls_lut.entries[indices[i]].computed) {
				return -1;  // Required points not computed
			}
		}

		// Linear least squares fit using 4 points
		float temps[EXTRAP_NUM_POINTS];
		float t_mean = 0.0f;
		for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
			temps[i] = MLS_LUT_IDX_TO_TEMP(indices[i]);
			t_mean += temps[i];
		}
		t_mean /= EXTRAP_NUM_POINTS;

		float sum_dt_sq = 0.0f;
		for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
			float dt = temps[i] - t_mean;
			sum_dt_sq += dt * dt;
		}

		for (int axis = 0; axis < 3; axis++) {
			float b_mean = 0.0f;
			for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
				b_mean += mls_lut.entries[indices[i]].bias[axis];
			}
			b_mean /= EXTRAP_NUM_POINTS;

			float sum_dt_db = 0.0f;
			for (int i = 0; i < EXTRAP_NUM_POINTS; i++) {
				float dt = temps[i] - t_mean;
				float db = mls_lut.entries[indices[i]].bias[axis] - b_mean;
				sum_dt_db += dt * db;
			}

			float slope = (sum_dt_sq > 0.001f) ? sum_dt_db / sum_dt_sq : 0.0f;
			bias_out[axis] = b_mean + slope * (original_temp - t_mean);
		}
		return 0;
	}

	// Normal interpolation within range
	float fidx = MLS_LUT_TEMP_TO_IDX(temp);

	// Get integer indices for interpolation
	int idx_lo = (int)fidx;
	int idx_hi = idx_lo + 1;

	// Clamp indices to valid range (handle edge cases)
	if (idx_lo < 0) {
		idx_lo = 0;
		idx_hi = 1;
	}
	if (idx_hi >= MLS_LUT_SIZE) {
		idx_hi = MLS_LUT_SIZE - 1;
		idx_lo = MLS_LUT_SIZE - 2;
	}
	if (idx_lo < 0) {
		idx_lo = 0;
	}

	// Check if required entries are computed
	if (!mls_lut.entries[idx_lo].computed || !mls_lut.entries[idx_hi].computed) {
		return -1;  // Required entries not yet computed, caller should use MLS fallback
	}

	// Linear interpolation
	float frac = fidx - (float)idx_lo;
	if (frac < 0.0f) {
		frac = 0.0f;
	} else if (frac > 1.0f) {
		frac = 1.0f;
	}

	const float *bias_lo = mls_lut.entries[idx_lo].bias;
	const float *bias_hi = mls_lut.entries[idx_hi].bias;

#if CONFIG_CMSIS_DSP
	// bias_out = bias_lo + frac * (bias_hi - bias_lo)
	float diff[3];
	arm_sub_f32(bias_hi, bias_lo, diff, 3);
	arm_scale_f32(diff, frac, diff, 3);
	arm_add_f32(bias_lo, diff, bias_out, 3);
#else
	for (int axis = 0; axis < 3; axis++) {
		bias_out[axis] = bias_lo[axis] + frac * (bias_hi[axis] - bias_lo[axis]);
	}
#endif

	return 0;
}

#endif

// Sensor-thread-owned field calibration. Console/buttons only queue requests.
static field_earth_t field_earth;
static atomic_t field_earth_request;
static atomic_t field_earth_active;

static void earth_cal_led_off_handler(struct k_work *work)
{ (void)work; if (!sensor_earth_cal_active() && !sensor_tcal_get_auto_calibration()) { set_led(SYS_LED_PATTERN_OFF,SYS_LED_PRIORITY_HIGHEST); set_led(SYS_LED_PATTERN_OFF,SYS_LED_PRIORITY_SENSOR); } }
static K_WORK_DELAYABLE_DEFINE(earth_cal_led_off_work,earth_cal_led_off_handler);

bool sensor_earth_cal_active(void) { return atomic_get(&field_earth_active)!=0; }
void sensor_earth_cal_start(void)
{
    if (sensor_earth_cal_active() || sensor_gyro_scale_active()) { printk("Earth cal rejected: calibration busy.\n"); return; }
    if (!atomic_cas(&field_earth_active,0,1)) return;
    atomic_set(&field_earth_request,1);
}
void sensor_earth_cal_cancel(void) { atomic_set(&field_earth_request,2); }

static void earth_reference_get(float body[3]);

/* Rotate v by q (sign=+1) or by its conjugate (sign=-1). */
static void earth_rot_quat(const float q[4], const float v[3], int sign, float out[3])
{
    float w = q[0], x = q[1] * (float)sign, y = q[2] * (float)sign, z = q[3] * (float)sign;
    float tx = 2.0f * (y * v[2] - z * v[1]);
    float ty = 2.0f * (z * v[0] - x * v[2]);
    float tz = 2.0f * (x * v[1] - y * v[0]);
    out[0] = v[0] + w * tx + (y * tz - z * ty);
    out[1] = v[1] + w * ty + (z * tx - x * tz);
    out[2] = v[2] + w * tz + (x * ty - y * tx);
}

/* Make the current resting pose the new reference pose: the stored Earth
 * vector is rotated into the current body frame and the up axis is taken from
 * gravity. Reboot afterwards in this pose to apply it. */
static int64_t pose_save_due_ms;
static bool pose_save_pending;

/* Save the current orientation after a short settle so the user can stand
 * still after issuing the command. */
void sensor_earth_pose_save_after(uint32_t delay_ms)
{
    pose_save_pending = true;
    pose_save_due_ms = k_uptime_get() + (int64_t)delay_ms;
    printk("Pose save in %u s: stand still in the target pose\n", delay_ms / 1000);
}

/* Called from the sensor loop; commits the pending save when due. */
void sensor_earth_pose_save_poll(void)
{
    if (!pose_save_pending) {
        return;
    }
    if (k_uptime_get() < pose_save_due_ms) {
        return;
    }
    pose_save_pending = false;
    sensor_earth_pose_save();
}

bool sensor_earth_pose_save(void)
{
    if (k_uptime_get() - field_accel_time > 200) {
        printk("Earth pose save rejected: accelerometer data is stale\n");
        return false;
    }
    float n = sqrtf(field_accel[0]*field_accel[0] + field_accel[1]*field_accel[1] +
                    field_accel[2]*field_accel[2]);
    if (!(n > 0.7f && n < 1.3f)) {
        printk("Earth pose save rejected: hold the device still (|a|=%.2f)\n", (double)n);
        return false;
    }
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    sensor_get_fusion_quat(q);
    if (!sys_pose_quat_save(q)) {
        printk("Earth pose save failed: NVS write\n");
        return false;
    }
    printk("Pose saved: q=%.4f %.4f %.4f %.4f (restored on FUSION_RESET)\n",
           (double)q[0], (double)q[1], (double)q[2], (double)q[3]);
    return true;
}

bool sensor_earth_reference_set(const float body[3])
{
    if(sensor_earth_cal_active()) { printk("Cancel Earth calibration before importing a reference.\n"); return false; }
    if (!sys_earth_reference_save(body)) { printk("Earth reference rejected or NVS write failed.\n"); return false; }
    printk("Earth reference saved to NVS. Reboot in the matching fixed pose to apply.\n");
    return true;
}

static void earth_reference_get(float body[3])
{
    if(retained->earthCalValid) memcpy(body,retained->earthCalBody,3*sizeof(float));
    else sys_earth_reference_default(body);
}

float sensor_earth_reference_latitude(void)
{
    float body[3]; earth_reference_get(body);
    return field_reference_latitude(body);
}

bool sensor_earth_set_latitude(float latitude_deg)
{
    float body[3]; earth_reference_get(body);
    if(!field_reference_set_latitude(body,latitude_deg)) {
        printk("Latitude rejected: use -90..90 degrees (north positive); reference must retain a horizontal heading.\n");
        return false;
    }
    return sensor_earth_reference_set(body);
}

void sensor_earth_cal_status(void)
{
    float body[3];
    earth_reference_get(body);
    printk("Earth cal: active=%d; reference source=%s\n",sensor_earth_cal_active(),retained->earthCalValid ? "NVS" : "firmware default");
    printk("Reference body dps: %.9f %.9f %.9f\n",(double)body[0],(double)body[1],(double)body[2]);
    printk("Site latitude: %.6f deg (north positive, reference +X up)\n",(double)field_reference_latitude(body));
    printk("Sync command: earth ref set %.9f %.9f %.9f\n",(double)body[0],(double)body[1],(double)body[2]);
}

void sensor_earth_cal_feed(const float g[3], float dt, bool rest)
{

    int request=atomic_set(&field_earth_request,0);
    if(request==2) {
        field_earth_cancel(&field_earth); atomic_set(&field_earth_active,0);
        k_work_cancel_delayable(&earth_cal_led_off_work);
        set_led(SYS_LED_PATTERN_OFF,SYS_LED_PRIORITY_HIGHEST);
        if(!sensor_tcal_get_auto_calibration()) set_led(SYS_LED_PATTERN_OFF,SYS_LED_PRIORITY_SENSOR);
        printk("Earth calibration cancelled; previous reference preserved.\n");
        return;
    }
    if(request==1) {
        if(sensor_gyro_scale_active() || boot_zro_pending || sensor_tcal_get_auto_calibration() || !sensor_tcal_get_enabled()) {
            atomic_set(&field_earth_active,0);
            printk("Earth cal rejected: finish startup/other calibration and use a calibrated temperature.\n");
            return;
        }
        k_work_cancel_delayable(&earth_cal_led_off_work);
        /* Pick the up axis from gravity at the 0-degree pose so the reference
         * frame does not depend on the IMU mounting. */
        if (k_uptime_get() - field_accel_time <= 200) {
            float n = sqrtf(field_accel[0]*field_accel[0] + field_accel[1]*field_accel[1] +
                            field_accel[2]*field_accel[2]);
            if (n > 0.7f && n < 1.3f) {
                int best = 0;
                float bv = fabsf(field_accel[0]);
                for (int i = 1; i < 3; i++) {
                    if (fabsf(field_accel[i]) > bv) { best = i; bv = fabsf(field_accel[i]); }
                }
                if (bv / n > 0.85f) {
                    uint8_t axis = (uint8_t)(best * 2 + (field_accel[best] < 0 ? 1 : 0));
                    if (axis != retained->earthUpAxis) {
                        retained->earthUpAxis = axis;
                        sys_earth_up_axis_save(axis);
                    }
                    printk("Earth cal: up axis = %c%s (accel %.2f)\n",
                           "XYZ"[best], field_accel[best] < 0 ? "-" : "+",
                           (double)(field_accel[best] / n));
                }
            }
        }
        float body[3]; earth_reference_get(body);
        field_earth_start(&field_earth,field_reference_latitude(body),retained->earthUpAxis);
        set_led(SYS_LED_PATTERN_CALIBRATION,SYS_LED_PRIORITY_HIGHEST);
        printk("Earth cal: +X up, fixed 0-degree pose; 3s settle + 5-60s adaptive sample per stop. Four clicks or earth cal stop cancels.\n");
    }
    if(!sensor_earth_cal_active()) return;
    int previous_stop=field_earth.stop;
    enum field_earth_phase previous=field_earth.phase;
    if(k_uptime_get()-field_accel_time>100) {
        field_earth.phase=FIELD_FAILED; field_earth.error="accelerometer unavailable";
    } else {
        field_earth_step(&field_earth,g,field_accel,sensor_get_current_imu_temperature(),dt,rest);
    }
    if(field_earth.phase==FIELD_DONE) {
        atomic_set(&field_earth_active,0);
        if(sys_earth_reference_save(field_earth.result)) {
            printk("Earth cal saved to NVS: %.9f %.9f %.9f dps; rms=%.6f. Reboot at 0 degrees.\n",
                (double)field_earth.result[0],(double)field_earth.result[1],(double)field_earth.result[2],(double)field_earth.rms);
            set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE,SYS_LED_PRIORITY_HIGHEST);
        } else {
            printk("Earth cal NVS write failed; reference not applied.\n");
            set_led(SYS_LED_PATTERN_ERROR_A,SYS_LED_PRIORITY_HIGHEST);
            k_work_schedule(&earth_cal_led_off_work,K_SECONDS(5));
        }
    } else if(field_earth.phase==FIELD_FAILED) {
        atomic_set(&field_earth_active,0);
        printk("Earth cal rejected: %s. Previous reference preserved.\n",field_earth.error);
        set_led(SYS_LED_PATTERN_ERROR_A,SYS_LED_PRIORITY_HIGHEST);
        k_work_schedule(&earth_cal_led_off_work,K_SECONDS(5));
    } else if(field_earth.stop!=previous_stop || field_earth.phase!=previous) {
        set_led(field_earth.phase==FIELD_TURN ? SYS_LED_PATTERN_ON : SYS_LED_PATTERN_CALIBRATION,SYS_LED_PRIORITY_HIGHEST);
        if(field_earth.phase==FIELD_TURN) printk("Earth cal: move to stop %d/5 in sequence 0/90/180/90/0.\n",field_earth.stop+1);
    }
}


void sensor_tcal_set_enabled(bool enabled)
{
	if (tcal_compensation_enabled == enabled) {
		LOG_INF("T-Cal compensation already %s", enabled ? "enabled" : "disabled");
		return;
	}
	tcal_compensation_enabled = enabled;
	bool val = enabled;
	sys_write(TCAL_ENABLED_ID, &retained->tcal_enabled, &val, sizeof(val));
	LOG_INF("T-Cal compensation %s (persisted)", enabled ? "enabled" : "disabled");
}

bool sensor_tcal_get_enabled(void)
{
	return tcal_compensation_enabled;
}

/* ---- Per-axis gyro scale calibration ------------------------------------ */

static gyro_scale_t gyro_scale_state;
static atomic_t gyro_scale_request;
static atomic_t gyro_scale_live;
static float gyro_scale_requested[3];
static int64_t gyro_scale_last_ms;
static icm45_fifo_diag_t gyro_scale_diag;
/* Button flow: start via 5 presses and stop automatically after the declared
 * turns when the device has been still for 3 s. */
static atomic_t gyro_scale_auto_pending;
static bool gyro_scale_auto_still;
static int64_t gyro_scale_auto_since;
static int64_t gyro_scale_auto_last_why;
static bool gyro_scale_tcal_enabled;

bool sensor_gyro_scale_active(void)
{
    return atomic_get(&gyro_scale_live) || atomic_get(&gyro_scale_auto_pending) || atomic_get(&gyro_scale_request)!=0;
}

static bool gyro_scale_queue(int request)
{
    if(!atomic_cas(&gyro_scale_request,0,request)) {
        printk("Gyro scale: another request is pending\n"); return false;
    }
    return true;
}

void sensor_gyro_scale_start(int turns, int axis)
{
    if(turns<1 || turns>20 || axis<0 || axis>2) {
        printk("Gyro scale: turns 1..20; axis x, y or z\n"); return;
    }
    if(sensor_gyro_scale_active()) {
        printk("Gyro scale: busy; stop or cancel the current measurement\n"); return;
    }
    if(gyro_scale_queue(10+20*axis+turns-1))
        printk("Gyro scale request queued; hold selected axis vertical and still, await READY\n");
}

void sensor_gyro_scale_stop(void)
{
    if(gyro_scale_queue(1)) printk("Gyro scale stop queued; await sensor acknowledgement\n");
}

void sensor_gyro_scale_auto_start(void)
{
    if(sensor_gyro_scale_active()) {
        printk("Gyro scale auto: busy; cancel the current measurement first\n");
        return;
    }
    if (gyro_scale_queue(5)) printk("Gyro scale auto request queued; await sensor acknowledgement\n");
}

void sensor_gyro_scale_cancel(void)
{
    if(gyro_scale_queue(2)) printk("Gyro scale cancellation queued\n");
}

void sensor_gyro_scale_status(void)
{
    if(gyro_scale_queue(3)) printk("Gyro scale status queued\n");
}

bool sensor_gyro_scale_set(const float scales[3])
{
    for(int i=0;i<3;i++) if(!isfinite(scales[i]) || scales[i]<=.5f || scales[i]>=1.5f) {
        printk("Gyro scale: finite scales strictly between 0.5 and 1.5 required\n"); return false;
    }
    if(atomic_get(&gyro_scale_live) || atomic_get(&gyro_scale_auto_pending) || !atomic_cas(&gyro_scale_request,0,-1)) {
        printk("Gyro scale: busy; cancel active measurement first\n"); return false;
    }
    memcpy(gyro_scale_requested,scales,sizeof(gyro_scale_requested));
    atomic_set(&gyro_scale_request,4);
    printk("Gyro scale values queued; await sensor save acknowledgement\n");
    return true;
}

void sensor_gyro_scale_invalidate(void)
{
    /* Sensor thread only: invalid FIFO data may not silently shorten turns. */
    if(atomic_get(&gyro_scale_live)) gyro_scale_fail(&gyro_scale_state,"invalid FIFO packet or sensor reset");
}

static bool gyro_scale_fifo_healthy(void)
{
    if(imu_id!=IMU_ICM45686) return true;
    icm45_fifo_diag_t d; icm45_fifo_get_diag(&d);
    /* With gyro/accel ODR 4:1, three of every four gyro packets normally
     * carry the no-new-accel sentinel counted by invalid_accel_packets.
     * Sparse accel availability is checked using field_accel_time below. */
    return d.count_read_errors==gyro_scale_diag.count_read_errors &&
        d.data_read_errors==gyro_scale_diag.data_read_errors &&
        d.buffer_limited_reads==gyro_scale_diag.buffer_limited_reads &&
        d.invalid_headers==gyro_scale_diag.invalid_headers &&
        d.invalid_gyro_packets==gyro_scale_diag.invalid_gyro_packets &&
        d.accel_near_rail_packets==gyro_scale_diag.accel_near_rail_packets &&
        d.gyro_near_rail_packets==gyro_scale_diag.gyro_near_rail_packets;
}

void sensor_gyro_scale_feed(const float g[3], float dt, bool rest)
{
    /* Button flow: arm as soon as every gate the real start checks is clear. */
    if (atomic_get(&gyro_scale_auto_pending) && !atomic_get(&gyro_scale_live) && !atomic_get(&gyro_scale_request)) {
        const char *why = NULL;
        if (boot_zro_pending) why = "boot zero-rate settling";
        else if (sensor_earth_cal_active()) why = "Earth calibration active";
        else if (sensor_tcal_get_auto_calibration()) why = "temperature collection active";
        else if (CONFIG_SENSOR_GYRO_OVERSAMPLING != 1) why = "oversampling disabled";
        else if (!isfinite(sensor_get_current_imu_temperature())) why = "temperature not ready";
        if (!why) {
            gyro_scale_queue(29); /* 20 turns, body X; already owns the armed state. */
        } else if (k_uptime_get() - gyro_scale_auto_last_why > 1000) {
            gyro_scale_auto_last_why = k_uptime_get();
            printk("Gyro scale auto waiting: %s\n", why);
        }
    }
    /* This is called only by the sensor thread after temperature/g-sens
     * correction and before gyroSensScale. All measurement/state changes and
     * persistent commits occur here, never in the console thread. */
    int request=atomic_get(&gyro_scale_request);
    float requested_scale[3];
    /* Capture the payload before releasing the slot to the console writer. */
    if(request==4) memcpy(requested_scale,gyro_scale_requested,sizeof(requested_scale));
    if(request>0) atomic_set(&gyro_scale_request,0);
    int64_t now=k_uptime_get();
    if(request>=10) {
        if(atomic_get(&gyro_scale_live)) {
            printk("Gyro scale rejected: measurement already active\n");
        } else if(sensor_tcal_get_auto_calibration() || sensor_earth_cal_active() || boot_zro_pending ||
                  CONFIG_SENSOR_GYRO_OVERSAMPLING!=1 || !isfinite(sensor_get_current_imu_temperature())) {
            printk("Gyro scale rejected: finish startup/other calibration; full-rate samples and temperature required\n");
        } else {
            int value=request-10;
            if(gyro_scale_begin(&gyro_scale_state,value%20+1,value/20,retained->gyroSensScale)) {
                atomic_set(&gyro_scale_live,1); gyro_scale_last_ms=now;
                set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_CALIBRATION);
                gyro_scale_tcal_enabled=sensor_tcal_get_enabled();
                if(imu_id==IMU_ICM45686) icm45_fifo_get_diag(&gyro_scale_diag);
                printk("Gyro scale: hold body %c vertical and still for 2s; wait for READY before rotating\n", "XYZ"[gyro_scale_state.axis]);
            }
        }
    } else if(request==2) {
        atomic_set(&gyro_scale_live,0); gyro_scale_state.phase=GYRO_SCALE_IDLE;
        atomic_set(&gyro_scale_auto_pending,0); gyro_scale_auto_still=false;
        set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_CALIBRATION);
        printk("Gyro scale cancelled; coefficients unchanged\n");
    } else if(request==5) {
        atomic_set(&gyro_scale_auto_pending,1); gyro_scale_auto_still=false;
        set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_CALIBRATION);
        printk("Gyro scale auto: armed; waiting for startup/other calibration\n");
    } else if(request==3) {
        printk("Gyro scale: X %.9f Y %.9f Z %.9f; active=%d phase=%d signed angle=%.3f deg; armed=%d\n",
            (double)retained->gyroSensScale[0],(double)retained->gyroSensScale[1],(double)retained->gyroSensScale[2],
            (int)atomic_get(&gyro_scale_live),(int)gyro_scale_state.phase,
            (double)gyro_scale_state.angle[gyro_scale_state.axis], (int)atomic_get(&gyro_scale_auto_pending));
    } else if(request==4) {
        if(atomic_get(&gyro_scale_live)) printk("Gyro scale save rejected: measurement active\n");
        else if(sensor_tcal_get_auto_calibration() || sensor_earth_cal_active() || boot_zro_pending)
            printk("Gyro scale save rejected: finish startup/other calibration first\n");
        else if(!rest) printk("Gyro scale save rejected: hold still and retry\n");
        else if(sys_gyro_scale_save(requested_scale)) {
            printk("Gyro scale saved and verified: X %.6f Y %.6f Z %.6f\n",
                (double)retained->gyroSensScale[0],(double)retained->gyroSensScale[1],(double)retained->gyroSensScale[2]);
        } else printk("Gyro scale save failed; runtime coefficients unchanged\n");
    }
    if(atomic_get(&gyro_scale_live)) {
        bool valid=now-gyro_scale_last_ms<=100 && now-field_accel_time<=50 && gyro_scale_fifo_healthy() &&
            !sensor_tcal_get_auto_calibration() && !sensor_earth_cal_active() &&
            gyro_scale_tcal_enabled==sensor_tcal_get_enabled() &&
            memcmp(gyro_scale_state.initial_scale,retained->gyroSensScale,sizeof(gyro_scale_state.initial_scale))==0;
        gyro_scale_last_ms=now;
        enum gyro_scale_phase before=gyro_scale_state.phase;
        gyro_scale_sample(&gyro_scale_state,g,field_accel,sensor_get_current_imu_temperature(),dt,rest,valid);
        if(before==GYRO_SCALE_PREPARE && gyro_scale_state.phase==GYRO_SCALE_RUN) {
            set_led(SYS_LED_PATTERN_CALIBRATION, SYS_LED_PRIORITY_CALIBRATION);
            printk("Gyro scale READY: rotate %d complete turns in ONE direction about body %c, return to the mark, hold still >=3s, then gyro scale stop\n",
                gyro_scale_state.turns,"XYZ"[gyro_scale_state.axis]);
        }
        if(request==1 && gyro_scale_state.phase!=GYRO_SCALE_FAILED) {
            if(gyro_scale_finish(&gyro_scale_state)) {
                if(sys_gyro_scale_save(gyro_scale_state.result_scale)) {
                    set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_CALIBRATION);
                    printk("Gyro scale: %c integrated %.3f deg over %d turns; X %.6f Y %.6f Z %.6f saved and verified\n",
                        "XYZ"[gyro_scale_state.axis],(double)gyro_scale_state.angle[gyro_scale_state.axis],gyro_scale_state.turns,
                        (double)retained->gyroSensScale[0],(double)retained->gyroSensScale[1],(double)retained->gyroSensScale[2]);
                } else {
                    gyro_scale_fail(&gyro_scale_state,"NVS save/readback failed");
                }
                atomic_set(&gyro_scale_live,0);
                atomic_set(&gyro_scale_auto_pending,0); gyro_scale_auto_still=false;
            }
        }
        if(gyro_scale_state.phase==GYRO_SCALE_FAILED) {
            atomic_set(&gyro_scale_live,0);
            atomic_set(&gyro_scale_auto_pending,0); gyro_scale_auto_still=false;
            set_led(SYS_LED_PATTERN_ONESHOT_ERROR, SYS_LED_PRIORITY_CALIBRATION);
            printk("Gyro scale rejected: %s; coefficients unchanged\n",gyro_scale_state.error);
        }
        /* Button flow: finish automatically once the declared turns are done
         * and the device has been still for 3 s. */
        if(atomic_get(&gyro_scale_auto_pending) && gyro_scale_state.phase==GYRO_SCALE_RUN) {
            double need=(double)gyro_scale_state.turns*360.0*0.90;
            if(fabs(gyro_scale_state.angle[gyro_scale_state.axis])>need) {
                if(rest && gyro_scale_state.end_rest_time>0) {
                    if(!gyro_scale_auto_still) { gyro_scale_auto_still=true; gyro_scale_auto_since=now; }
                    else if(now-gyro_scale_auto_since>=3000) {
                        printk("Gyro scale auto: rotation complete, saving\n");
                        if(gyro_scale_queue(1)) {
                            atomic_set(&gyro_scale_auto_pending,0);
                            gyro_scale_auto_still=false;
                        }
                    }
                } else {
                    gyro_scale_auto_still=false;
                }
            }
        }
    } else if(request==1) printk("Gyro scale: not started\n");
}

/* ---- g-sensitivity matrix (b = M * a, dps per g) ------------------------ */

/* ===================== g-sensitivity pose calibration =====================
 * Collect one still pose per command, then fit bias = b0 + M*a across poses and
 * apply a Newton step to the stored matrix: a pose captured while M is active
 * yields exactly the residual slope, so M_new = M_old + fit.  Poses that are
 * too close in direction are rejected so a repeated placement cannot skew the
 * fit (the direction is derived from gravity, not from user input).
 */
#define GSENS_MAX_POSES 8
#define GSENS_CAPTURE_MS 60000
#define GSENS_SETTLE_MS 10000
#define GSENS_MIN_DIR_COS 0.95f   /* ~18 deg: reject near-duplicate poses */
#define GSENS_MAX_COEFF 0.05f
#define GSENS_MAX_RESIDUAL 0.003f

static struct {
	bool active;
	int64_t start_ms;
	uint32_t samples;
	float acc_sum[3];
	float bias_sum[3];
} gsens_capture;
static struct {
	float acc[3];
	float bias[3];
} gsens_poses[GSENS_MAX_POSES];
static int gsens_pose_count;

void sensor_gsens_clear_poses(void)
{
	gsens_capture.active = false;
	gsens_pose_count = 0;
	printk("GSENS: captured poses cleared\n");
}

static bool gsens_guide_active;
static int64_t gsens_led_resume_ms;
static bool gsens_led_resume_pending;
static bool gsens_reject_blink;      /* slow blink until a new pose is seen */
static int64_t gsens_blink_toggle_ms;
static int gsens_pending_axis = -1;
static int64_t gsens_pending_since;
static bool gsens_blink_on;
static int gsens_next_axis;
static int gsens_pose_axis[GSENS_MAX_POSES];

void sensor_gsens_set_auto(bool enabled)
{
	(void)enabled;
}

/* Six quick presses: guided pose calibration by flipping the device axis-up. */
bool sensor_gsens_guide_active(void)
{
	return gsens_guide_active;
}

void sensor_gsens_guide_start(void)
{
	gsens_capture.active = false;
	gsens_pose_count = 0;
	memset(gsens_pose_axis, 0xFF, sizeof(gsens_pose_axis));
	gsens_guide_active = true;
	set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_HIGHEST);
	printk("GSENS guide: place the device with a body axis up and keep still (6 poses)\n");
}

static void gsens_reject_led(void)
{
	/* Solid ON means "waiting for a new orientation", whether after a success
	 * or a rejection: one simple state the user has to remember. */
	gsens_reject_blink = false;
	set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_HIGHEST);
}

void sensor_gsens_collect_start(void)
{
	memset(&gsens_capture, 0, sizeof(gsens_capture));
	gsens_capture.active = true;
	gsens_capture.start_ms = k_uptime_get();
	printk("GSENS: capture %d/%d started; keep the device still for %d s\n",
	       gsens_pose_count + 1, GSENS_MAX_POSES, GSENS_CAPTURE_MS / 1000);
}

void sensor_gsens_collect_feed(const float accel[3], const float bias[3], bool rest)
{
	if (!gsens_capture.active) {
		if (!gsens_guide_active || gsens_pose_count >= GSENS_MAX_POSES) {
			return;
		}
		if (gsens_led_resume_pending && k_uptime_get() >= gsens_led_resume_ms) {
			gsens_led_resume_pending = false;
			set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_HIGHEST);
		}
		if (gsens_reject_blink) {
			int64_t now_blink = k_uptime_get();
			if (now_blink >= gsens_blink_toggle_ms) {
				gsens_blink_on = !gsens_blink_on;
				set_led(gsens_blink_on ? SYS_LED_PATTERN_ON : SYS_LED_PATTERN_OFF,
				        SYS_LED_PRIORITY_CALIBRATION);
				gsens_blink_toggle_ms = now_blink + 400;
			}
		}
		if (!rest) {
			gsens_pending_axis = -1;
			return;
		}
		if (gsens_pending_axis >= 0) {
			if (k_uptime_get() - gsens_pending_since < GSENS_SETTLE_MS) {
				return;
			}
			gsens_pending_axis = -1;
			printk("GSENS: settled, capturing %d s\n", GSENS_CAPTURE_MS / 1000);
			sensor_gsens_collect_start();
			return;
		}
		/* Identify which body axis points up (cardinal classification). */
		float n = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
		if (n < 0.6f || n > 1.4f) {
			return;
		}
		float gx = accel[0]/n, gy = accel[1]/n, gz = accel[2]/n;
		int axis = 0, sign = 1;
		float best = fabsf(gx);
		if (fabsf(gy) > best) { axis = 1; best = fabsf(gy); }
		if (fabsf(gz) > best) { axis = 2; best = fabsf(gz); }
		if (best < 0.85f) {
			return; /* tilted: not a cardinal pose */
		}
		sign = ((axis == 0 ? gx : axis == 1 ? gy : gz) < 0.0f) ? -1 : 1;
		int key = axis * 2 + (sign < 0 ? 1 : 0);
		for (int k = 0; k < gsens_pose_count; k++) {
			if (gsens_pose_axis[k] == key) {
				return; /* this axis-up pose is already captured */
			}
		}
		gsens_next_axis = key;
		gsens_reject_blink = false;
		gsens_pending_axis = key;
		gsens_pending_since = k_uptime_get();
		printk("GSENS: axis-up pose (%d) detected; settling %d s before capture\n",
		       key, GSENS_SETTLE_MS / 1000);
		set_led(SYS_LED_PATTERN_CALIBRATION, SYS_LED_PRIORITY_HIGHEST);
		return;
	}
	if (!rest) {
		/* Discard a contaminated window and wait for a new still period. */
		gsens_capture.samples = 0;
		gsens_capture.start_ms = k_uptime_get();
		return;
	}
	for (int i = 0; i < 3; i++) {
		gsens_capture.acc_sum[i] += accel[i];
		gsens_capture.bias_sum[i] += bias[i];
	}
	gsens_capture.samples++;
	if (k_uptime_get() - gsens_capture.start_ms < GSENS_CAPTURE_MS) {
		return;
	}
	gsens_capture.active = false;
	if (gsens_capture.samples < 200) {
		printk("GSENS: too few samples, capture discarded\n");
		gsens_reject_led();
		return;
	}
	float acc[3], pose_bias[3];
	for (int i = 0; i < 3; i++) {
		acc[i] = gsens_capture.acc_sum[i] / gsens_capture.samples;
		pose_bias[i] = gsens_capture.bias_sum[i] / gsens_capture.samples;
	}
	float norm = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
	if (norm < 0.6f || norm > 1.4f) {
		printk("GSENS: accel norm %.2f g is not a rest pose, discarded\n", (double)norm);
		gsens_reject_led();
		return;
	}
	for (int i = 0; i < 3; i++) {
		acc[i] /= norm;
	}
	for (int k = 0; k < gsens_pose_count; k++) {
		float dot = acc[0]*gsens_poses[k].acc[0] + acc[1]*gsens_poses[k].acc[1] +
		            acc[2]*gsens_poses[k].acc[2];
		if (dot > GSENS_MIN_DIR_COS) {
			printk("GSENS: pose too similar to pose %d (cos %.3f), move the device\n",
			       k + 1, (double)dot);
			gsens_reject_led();
			return;
		}
	}
	if (gsens_pose_count >= GSENS_MAX_POSES) {
		printk("GSENS: pose list full\n");
		return;
	}
	memcpy(gsens_poses[gsens_pose_count].acc, acc, sizeof(acc));
	memcpy(gsens_poses[gsens_pose_count].bias, pose_bias, sizeof(pose_bias));
	gsens_pose_axis[gsens_pose_count] = gsens_next_axis;
	gsens_pose_count++;
	printk("GSENS: pose %d stored: acc=(%.3f,%.3f,%.3f) bias=(%.5f,%.5f,%.5f)\n",
	       gsens_pose_count, (double)acc[0], (double)acc[1], (double)acc[2],
	       (double)pose_bias[0], (double)pose_bias[1], (double)pose_bias[2]);
	if (gsens_pose_count >= 4) {
		printk("GSENS: %d poses ready; place another axis up or wait for the fit\n",
		       gsens_pose_count);
		set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_HIGHEST);
		if (gsens_pose_count >= 6) {
			gsens_guide_active = false;
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_CALIBRATION);
			sensor_gsens_calc();
		}
	} else {
		printk("GSENS: %d/6 poses; flip the device to another axis-up pose\n",
		       gsens_pose_count);
		set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_HIGHEST);
	}
}

void sensor_gsens_calc(void)
{
	if (gsens_pose_count < 4) {
		printk("GSENS: need at least 4 poses, have %d\n", gsens_pose_count);
		return;
	}
	float M[3][3];
	float b0[3];
	float worst_res = 0.0f;
	for (int ax = 0; ax < 3; ax++) {
		/* Solve the 4-parameter model b = b0 + M.a via normal equations. */
		double A[4][4] = {{0}}, y[4] = {0};
		for (int k = 0; k < gsens_pose_count; k++) {
			double row[4] = {gsens_poses[k].acc[0], gsens_poses[k].acc[1],
			                 gsens_poses[k].acc[2], 1.0};
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					A[i][j] += row[i] * row[j];
				}
				y[i] += row[i] * gsens_poses[k].bias[ax];
			}
		}
		/* Gaussian elimination with partial pivot. */
		double aug[4][5];
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) aug[i][j] = A[i][j];
			aug[i][4] = y[i];
		}
		for (int col = 0; col < 4; col++) {
			int piv = col;
			for (int r = col + 1; r < 4; r++) {
				if (fabs(aug[r][col]) > fabs(aug[piv][col])) piv = r;
			}
			if (fabs(aug[piv][col]) < 1e-9) {
				printk("GSENS: degenerate pose set, add poses\n");
				return;
			}
			if (piv != col) {
				for (int j = 0; j < 5; j++) {
					double tmp = aug[col][j]; aug[col][j] = aug[piv][j]; aug[piv][j] = tmp;
				}
			}
			for (int r = col + 1; r < 4; r++) {
				double f = aug[r][col] / aug[col][col];
				for (int j = col; j < 5; j++) aug[r][j] -= f * aug[col][j];
			}
		}
		double x[4];
		for (int i = 3; i >= 0; i--) {
			double s = aug[i][4];
			for (int j = i + 1; j < 4; j++) s -= aug[i][j] * x[j];
			x[i] = s / aug[i][i];
		}
		for (int j = 0; j < 3; j++) M[ax][j] = (float)x[j];
		b0[ax] = (float)x[3];
		for (int k = 0; k < gsens_pose_count; k++) {
			float pred = b0[ax] + M[ax][0] * gsens_poses[k].acc[0] +
			             M[ax][1] * gsens_poses[k].acc[1] + M[ax][2] * gsens_poses[k].acc[2];
			float res = fabsf(pred - gsens_poses[k].bias[ax]);
			if (res > worst_res) worst_res = res;
		}
	}
	if (worst_res > GSENS_MAX_RESIDUAL) {
		printk("GSENS: residual %.4f exceeds %.4f, matrix NOT applied\n",
		       (double)worst_res, (double)GSENS_MAX_RESIDUAL);
		set_led(SYS_LED_PATTERN_ONESHOT_ERROR, SYS_LED_PRIORITY_CALIBRATION);
		return;
	}
	float M_new[9];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			/* Newton step: captured poses ran with the stored matrix active,
			 * so the fit is the residual, not the absolute matrix. */
			float v = retained->gyroGsens[3*i + j] + M[i][j];
			if (!isfinite(v) || fabsf(v) > GSENS_MAX_COEFF) {
				printk("GSENS: coefficient %.4f out of range, matrix NOT applied\n",
				       (double)v);
				return;
			}
			M_new[3*i + j] = v;
		}
	}
	printk("GSENS: fit ok (%d poses, residual %.4f), applying Newton update\n",
	       gsens_pose_count, (double)worst_res);
	sensor_gyro_gsens_set(M_new);
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_CALIBRATION);
	gsens_guide_active = false;
}

void sensor_get_last_accel(float out[3])
{
	memcpy(out, field_accel, sizeof(field_accel));
}

void sensor_gsens_progress(uint8_t *count, bool *guide, bool *capturing)
{
	if (count) *count = (uint8_t)gsens_pose_count;
	if (guide) *guide = gsens_guide_active;
	if (capturing) *capturing = gsens_capture.active;
}

void sensor_gsens_status(void)
{
	printk("GSENS: %d/%d poses, capture %s\n", gsens_pose_count, GSENS_MAX_POSES,
	       gsens_capture.active ? "running" : "idle");
	for (int k = 0; k < gsens_pose_count; k++) {
		printk("  pose %d: acc=(%.3f,%.3f,%.3f) bias=(%.5f,%.5f,%.5f)\n", k + 1,
		       (double)gsens_poses[k].acc[0], (double)gsens_poses[k].acc[1],
		       (double)gsens_poses[k].acc[2], (double)gsens_poses[k].bias[0],
		       (double)gsens_poses[k].bias[1], (double)gsens_poses[k].bias[2]);
	}
}

void sensor_gyro_gsens_set(const float m[9])
{
    for (int i=0;i<9;i++) {
        if (!isfinite(m[i]) || fabsf(m[i])>10.0f) {
            printk("g-sens rejected: coefficients must be finite and within +/-10 dps/g\n");
            return;
        }
    }
    if (sensor_gyro_scale_active() || !atomic_cas(&gyro_gsens_request,0,-1)) {
        printk("g-sens rejected: calibration request busy\n");
        return;
    }
    memcpy(requested_gyro_gsens,m,sizeof(requested_gyro_gsens));
    atomic_set(&gyro_gsens_request,1);
    printk("g-sens request queued; await sensor acknowledgement\n");
}

static void sensor_gyro_gsens_apply(bool rest)
{
    if (atomic_get(&gyro_gsens_request)!=1) return;
    if (!rest || boot_zro_pending || sensor_gyro_scale_active() || sensor_earth_cal_active() || sensor_tcal_get_auto_calibration()) {
        printk("g-sens rejected: requires settled rest and other calibrations stopped\n");
    } else if (sys_gyro_gsens_save(requested_gyro_gsens)) {
        printk("g-sens saved and read back; keep still for residual bias convergence\n");
    } else {
        printk("g-sens save/readback failed; runtime coefficients preserved\n");
    }
    atomic_set(&gyro_gsens_request,0);
}

void sensor_gyro_gsens_status(void)
{
	const float *m = retained->gyroGsens;
	printk("g-sens: %.9f %.9f %.9f / %.9f %.9f %.9f / %.9f %.9f %.9f (0 = none)\n",
	       (double)m[0], (double)m[1], (double)m[2],
	       (double)m[3], (double)m[4], (double)m[5],
	       (double)m[6], (double)m[7], (double)m[8]);
}

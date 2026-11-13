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
#ifndef SLIMENRF_SENSOR_CALIBRATION
#define SLIMENRF_SENSOR_CALIBRATION

#include <stdbool.h>
#include <stdint.h>

/* Sensor feeds data to calibration */
void sensor_calibration_process_accel(float a[3]);
void sensor_calibration_process_gyro(float g[3], float dt, bool rest);
void sensor_calibration_process_mag(float m[3]);

void sensor_calibration_update_sensor_ids(int imu);
uint8_t *sensor_calibration_get_sensor_data();

void sensor_calibration_read(void);

int sensor_calibration_validate(float *a_bias, float *g_bias, bool write);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
int sensor_calibration_validate_6_side(float a_inv[][3], bool write);
#endif
int sensor_calibration_validate_mag(float m_inv[][3], bool write);

void sensor_calibration_clear(float *a_bias, float *g_bias, bool write);
#if CONFIG_SENSOR_USE_6_SIDE_CALIBRATION
void sensor_calibration_clear_6_side(float a_inv[][3], bool write);
#endif
void sensor_calibration_clear_mag(float m_inv[][3], bool write); // "request" mag cal

void sensor_request_calibration(void);
void sensor_request_calibration_6_side(void);
void sensor_request_calibration_mag(void);
void sensor_gyro_scale_start(int turns, int axis);
void sensor_gyro_scale_auto_start(void);
bool sensor_boot_zro_ready(void);
bool sensor_gyro_scale_set(const float scales[3]);
bool sensor_gyro_scale_active(void);
void sensor_gyro_scale_cancel(void);
void sensor_gyro_scale_feed(const float g[3], float dt, bool rest);
void sensor_gyro_scale_invalidate(void);
void sensor_gyro_gsens_set(const float m[9]);
void sensor_gyro_gsens_status(void);
void sensor_gsens_collect_start(void);
void sensor_gsens_collect_feed(const float accel[3], const float bias[3], bool rest);
void sensor_gsens_calc(void);
void sensor_gsens_clear_poses(void);
void sensor_gsens_status(void);
void sensor_gsens_progress(uint8_t *count, bool *guide, bool *capturing);
void sensor_gsens_set_auto(bool enabled);
void sensor_gsens_guide_start(void);
bool sensor_gsens_guide_active(void);
void sensor_get_last_accel(float out[3]);
void sensor_gyro_scale_stop(void);
void sensor_gyro_scale_status(void);
void sensor_calibration_online_mag_sample(const float m[3]);
int sensor_calibration_online_mag_status(float *dir_bias);
void sensor_calibration_track_mag_norm(float cal_norm);
float sensor_calibration_get_mag_quality(void);
void sensor_calibration_online_mag_retained_save(void);
void sensor_calibration_online_mag_retained_clear(void);
void sensor_calibration_online_mag_cold_start(void);

#if CONFIG_SENSOR_USE_TCAL

// Quality assessment structure for T-Cal
typedef struct {
	bool curve_valid;          // Curve is valid
	uint16_t point_count;      // Number of calibration points
	float curve_error;         // Fitting error
	float temp_min;            // Minimum calibrated temperature
	float temp_max;            // Maximum calibrated temperature
	bool temp_in_range;        // Current temperature is in range
} tcal_quality_t;

void sensor_calibration_get_last_gyro_offset(float offset[3]);

// T-Cal maintenance/status
void sensor_tcal_clear(void);
/* Layout signature: stored point indices depend on range and bucket width. */
#define TCAL_CONFIG_SIG \
	((uint16_t)(CONFIG_SENSOR_POLY_TEMP_MIN*1000 + CONFIG_SENSOR_POLY_TEMP_MAX*10 + \
	            CONFIG_SENSOR_POLY_STEPS_PER_DEGREE))
void sensor_tcal_validate_layout(void);
/* Compact status used by the wireless status packet. */
void sensor_tcal_get_summary(uint16_t *points, bool *lut_valid, uint16_t *lut_entries,
                             float *temp, bool *coverage, bool *collecting, bool *enabled);
void sensor_tcal_status(void);
void sensor_tcal_remove_point(int index_to_remove);
bool sensor_tcal_is_temp_outside_range(float temp, float *min_temp, float *max_temp);
void sensor_tcal_check_auto_calibration(float current_temp);
void sensor_tcal_temperature_watch(float temp, bool rest);
void sensor_tcal_shift(float dx, float dy, float dz);
void sensor_tcal_set_point(int idx, float bx, float by, float bz);
void sensor_earth_cal_start(void);
void sensor_earth_cal_status(void);
void sensor_earth_cal_feed(const float g[3], float dt, bool rest);
void sensor_earth_cal_cancel(void);
bool sensor_earth_cal_active(void);
bool sensor_earth_reference_set(const float body[3]);
bool sensor_earth_pose_save(void);
void sensor_earth_pose_save_after(uint32_t delay_ms);
void sensor_earth_pose_save_poll(void);
bool sensor_earth_set_latitude(float latitude_deg);
bool sensor_tcal_coverage_ready(void);
void sensor_tcal_set_auto_calibration(bool enabled);
bool sensor_tcal_get_auto_calibration(void);

// T-Cal compensation enable/disable (persisted)
void sensor_tcal_set_enabled(bool enabled);
bool sensor_tcal_get_enabled(void);

// Continuous bucket-based T-Cal sampling
void sensor_tcal_feed_continuous_sample(const float g[3], float temp, bool rest);
void sensor_tcal_continuous_motion_detected(void);

// Quality assessment function - returns true if quality is sufficient
bool sensor_tcal_assess_quality(float current_temp, tcal_quality_t *quality);

// Compatibility stubs: legacy stationary calibration has been removed.
void sensor_tcal_boot_calibration_check(void);
void sensor_boot_cal_set_enabled(bool enabled);
bool sensor_boot_cal_is_completed(void);
void sensor_boot_cal_get_doffset(float offset[3]);
void sensor_boot_cal_reset(void); // Reset boot calibration state (call before reboot/shutdown, not before WoM)

// Compatibility stubs: runtime D_offset collection has been removed.
void sensor_runtime_calibration_check(bool is_resting);
void sensor_runtime_cal_get_status(int64_t *last_cal_time, int64_t *rest_duration);

// Test function for comparing calibration methods
void sensor_tcal_test_methods(float temp);
void sensor_tcal_dump_lut(void);
#endif


/* Read-only site latitude for archived Earth observations. */
float sensor_earth_reference_latitude(void);

/* CALWIRE: active accelerometer transform, in raw driver axes and g. */
bool sensor_calibration_wire_idle(void);
bool sensor_calibration_accel_snapshot(float out[4][3]);
#endif

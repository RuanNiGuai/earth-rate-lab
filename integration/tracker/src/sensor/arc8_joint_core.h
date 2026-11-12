/* ARC8 Foundation v0 experimental host-compatible core.
 * This does NOT change the deployed radio protocol, enable radio collection,
 * or infer synchronized clocks where no hardware timestamp exists.
 * Units: gyro mdps; accel mg; time us; angular increment microdegrees.
 */
#ifndef ARC8_JOINT_CORE_H
#define ARC8_JOINT_CORE_H
#include <stdbool.h>
#include <stdint.h>
#define ARC8_JOINT_PROTOCOL_VERSION 1
#define ARC8_JOINT_MAX_TRACKERS 10
#define ARC8_JOINT_WINDOW_SAMPLES 8
#define ARC8_JOINT_TIME_ESTIMATED 0x01
#define ARC8_JOINT_TIME_HW_VALID 0x02
#define ARC8_JOINT_FLAG_SATURATION 0x04
#define ARC8_JOINT_FLAG_FIFO_LOSS 0x08
#define ARC8_JOINT_FLAG_ACCEL_MISSING 0x10
/* Provenance of accel paired to a gyro sample. 0 = unspecified (legacy). */
#define ARC8_JOINT_ACCEL_FRESH 1
#define ARC8_JOINT_ACCEL_HELD 2
#define ARC8_JOINT_ACCEL_MISSING 3

typedef enum {
  ARC8_JOINT_IDLE=0, ARC8_JOINT_ARMED=1, ARC8_JOINT_RUNNING=2,
  ARC8_JOINT_STOPPED=3, ARC8_JOINT_FAILED=4
} arc8_joint_state;
typedef enum {
  ARC8_JOINT_OK=0, ARC8_JOINT_WINDOW_READY=1, ARC8_JOINT_WINDOW_REJECTED=2,
  ARC8_JOINT_INVALID=-1, ARC8_JOINT_WRONG_SESSION=-2,
  ARC8_JOINT_BAD_STATE=-3
} arc8_joint_result;
typedef struct {
  uint32_t seq;
  uint64_t sample_time_us; /* May be reconstructed, indicated in time_quality. */
  int32_t gyro_mdps[3];
  int32_t accel_mg[3];
  int16_t temperature_centi_c;
  uint8_t time_quality;
  uint8_t flags;
  uint8_t accel_quality;
} arc8_joint_sample;
typedef struct {
  uint32_t operation_id, boot_id;
  uint8_t tracker_id;
  uint32_t first_seq, last_seq, expected_samples, received_samples;
  uint64_t first_time_us, last_time_us;
  int64_t gyro_delta_udeg[3];
  int64_t accel_sum_mg[3];
  int64_t temp_sum_centi_c;
  uint32_t observed_gap_samples;
  uint32_t timing_bad_samples;
  uint32_t accel_fresh_samples, accel_held_samples, accel_missing_samples;
  int64_t accel_fresh_sum_mg[3]; /* Only genuine accel tags, not hold-last. */
  uint8_t time_quality;
  uint8_t flags;
} arc8_joint_summary;
typedef struct {
  uint8_t uid[8], tracker_id;
  uint32_t operation_id, boot_id, odr_hz, nominal_dt_us;
  arc8_joint_state state;
  uint32_t prev_seq;
  uint64_t prev_timestamp_us;
  bool have_prev;
  uint32_t windows_ok, windows_bad, gaps_total, invalid_time_total;
  uint32_t invalid_timing_total, invalid_sensor_flag_total;
  /* Multi-label sample reason buckets; one sample can increment several. */
  uint32_t reason_time_bad, reason_saturation, reason_fifo_loss;
  uint32_t reason_accel_missing, reason_gap;
  /* Multi-label rejected-window reason buckets (not mutually exclusive). */
  uint32_t bad_window_time, bad_window_saturation, bad_window_fifo_loss;
  uint32_t bad_window_accel_missing, bad_window_gap, bad_window_partial;
  arc8_joint_summary partial;
} arc8_joint_device;
arc8_joint_result arc8_joint_arm(arc8_joint_device *d, const uint8_t uid[8],
                                  uint8_t tracker_id, uint32_t boot_id,
                                  uint32_t operation_id, uint32_t odr_hz);
arc8_joint_result arc8_joint_start(arc8_joint_device *d, uint32_t boot_id,
                                    uint32_t operation_id);
arc8_joint_result arc8_joint_add(arc8_joint_device *d, uint32_t boot_id,
                                  uint32_t operation_id,
                                  const arc8_joint_sample *s,
                                  arc8_joint_summary *output);
arc8_joint_result arc8_joint_stop(arc8_joint_device *d, uint32_t boot_id,
                                   uint32_t operation_id);
#endif

#ifndef SLIMEVR_CAL_ARCHIVE_SAFE_H
#define SLIMEVR_CAL_ARCHIVE_SAFE_H
#include <stdbool.h>
#include <stdint.h>

/* F-05: one worker command slot, one immutable request per accepted submit.
 * seq==0 designates legacy USB/flag-only commands (not deduplicated).
 * These types are local telemetry until CAL_STATUS_RSP transport is added. */
typedef enum {
    ARC_SUBMIT_QUEUED = 0, ARC_SUBMIT_BUSY = 1,
    ARC_SUBMIT_REJECTED = 2, ARC_SUBMIT_INVALID = 3,
    ARC_SUBMIT_DUPLICATE = 4
} cal_arc_submit_t;
typedef enum {
    ARC_PHASE_NONE = 0, ARC_PHASE_QUEUED = 1,
    ARC_PHASE_EXECUTED = 2, ARC_PHASE_REJECTED = 3,
    ARC_PHASE_SAVED = 4, ARC_PHASE_FAILED = 5
} cal_arc_phase_t;
typedef enum {
    ARC_ERROR_NONE = 0, ARC_ERROR_BUSY = 1,
    ARC_ERROR_REFERENCE = 2, ARC_ERROR_WRONG_STATE = 3,
    ARC_ERROR_ROTATION = 4, ARC_ERROR_COMPRESSION = 5,
    ARC_ERROR_CARDINAL = 6, ARC_ERROR_DUPLICATE_POSE = 7,
    ARC_ERROR_RANGE = 8, ARC_ERROR_TIMEOUT = 9,
    ARC_ERROR_UNSTABLE = 10, ARC_ERROR_NVS = 11,
    ARC_ERROR_QUEUE = 12, ARC_ERROR_OTHER = 255
} cal_arc_error_t;
typedef struct {
    uint8_t seq, flag, phase, error;
    uint8_t saved_flags; /* bit0 TCal, bits1..3 Scale XYZ, bit4 Earth, bit5 GSens */
    uint8_t tcal_points, scale_mask, earth_done, gsens_done;
} cal_arc_status_t;
cal_arc_submit_t cal_archive_command_txn(const char *command, uint8_t flag, uint8_t seq);
void cal_archive_status_snapshot(cal_arc_status_t *out);
/* F05 Stage2: pop one state transition, preserving its original seq/flag.
 * Only the ESB thread sends radio packets; worker/sensor paths never do. */
bool cal_archive_status_take_event(cal_arc_status_t *out);
/* Used only for a duplicate non-idempotent command (re-send cached result). */
void cal_archive_status_replay(uint8_t seq, uint8_t flag);
/* Communicate failed admission, which never reaches the worker. */
void cal_archive_status_submit_failed(uint8_t seq, uint8_t flag,
                                      cal_arc_submit_t admission);

/* User commands are enqueued to a lazy, lower priority Zephyr worker.
 * No NVS read/write or calibration solver in normal startup or IMU thread. */
bool cal_archive_command(const char *command);
void cal_archive_feed_raw(const float raw_dps[3], float temperature_c, float dt, bool rest);
#endif

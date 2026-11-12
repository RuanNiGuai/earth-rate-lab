/* ARC8 Joint10 experiment. No ESB summary transport or NVS writes.
 * Only enabled by CONFIG_ARC8_JOINT10_EXPERIMENTAL.
 */
#ifndef ARC8_JOINT10_ADAPTER_H
#define ARC8_JOINT10_ADAPTER_H
#include <stdint.h>
#include <stdbool.h>

/* rate_hz=50 only; gyro ODR=400 only in this experiment. */
bool arc8_joint10_begin(uint8_t tracker_id, uint8_t rate_hz);
void arc8_joint10_end(void);
bool arc8_joint10_snapshot(uint32_t *ok, uint32_t *bad, uint32_t *gaps,
                            uint32_t *invalid);
/* Legacy entry retained for host tests and other consumers. */
void arc8_joint10_feed_raw(const float gyro_dps[3],
                            const float accel_g[3], float temp_c);
/* Explicit pairing metadata is needed when accel/gyro FIFO tags differ. */
void arc8_joint10_feed_raw_ex(const float gyro_dps[3],
                               const float accel_g[3], float temp_c,
                               bool accel_available, bool accel_fresh);
#endif

#ifndef F04_SUMMARY_CORE_H
#define F04_SUMMARY_CORE_H
#include "f04_summary_wire.h"
typedef struct {
    uint32_t boot, operation, sequence, anchor_us;
    uint32_t phase_ns, duration_ns, dt_ns;
    uint64_t elapsed_ns;
    double delta[3], accel[3], temperature;
    uint32_t ok, bad;
    uint8_t tracker, count, fresh;
    bool invalid;
} f04_accumulator;
bool f04_init(f04_accumulator *s, uint8_t tracker, uint32_t boot, uint32_t op, uint32_t anchor);
/* Return 1: valid packet, -1: rejected window, 0: accumulating.
 * Whole samples are assigned once; 409.6 Hz produces 8/9-sample windows.
 */
int f04_add(f04_accumulator *s, const float gyro[3], const float accel[3],
            float temp, float dt_s, bool available, bool fresh, uint8_t out[F04_SIZE]);
/* Unknown FIFO loss cannot be timed: make a visible sequence gap and reanchor. */
void f04_discontinuity(f04_accumulator *s, uint32_t anchor_us);
#endif

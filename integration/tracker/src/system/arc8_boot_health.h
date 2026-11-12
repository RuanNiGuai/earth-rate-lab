/* Continuous healthy sampling gate for confirming a MCUboot test image. */
#ifndef ARC8_BOOT_HEALTH_H
#define ARC8_BOOT_HEALTH_H
#include <stdbool.h>
#include <stdint.h>
struct arc8_boot_health { uint32_t start, last; bool tracking; };
static inline bool arc8_boot_health_step(struct arc8_boot_health *h,
        uint32_t now, bool fresh_samples, bool idle)
{
    if (!fresh_samples || !idle) { h->tracking = false; return false; }
    if (!h->tracking || (uint32_t)(now - h->last) > 500U) {
        h->start = now;
        h->tracking = true;
    }
    h->last = now;
    return (uint32_t)(now - h->start) >= 60000U;
}
#endif

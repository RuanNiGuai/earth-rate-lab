/* Strict slot policy for 48-byte ACK calibration packets at 2 Mbit/s.
 * Unlike live pose traffic, late calibration packets must wait, never spill.
 * 24 RTC ticks (732 us) reserve ramp-up, data, ACK/timeout and margin.
 * Hardware retransmissions are disabled; failures retry in a later slot.
 * Receiver clock estimation error still requires hardware validation.
 */
#ifndef TDMA_CAPTURE_CORE_H
#define TDMA_CAPTURE_CORE_H
#include <stdbool.h>
#include <stdint.h>
#define TDMA_CAPTURE_BUDGET_TICKS 24u
#define TDMA_CAPTURE_OFFSET_TICKS 2u
static inline bool tdma_capture_config_valid(uint8_t slot, uint8_t total, uint8_t width) {
    return total > 0 && total <= 32 && slot < total && width >= 32;
}
/* Zero means safe now; otherwise wait this many ticks and recheck. last is
 * the absolute start of the most recently claimed slot, not a frame number
 * (frame length can change when receiver membership changes). */
static inline uint32_t tdma_capture_delay(uint64_t now, uint8_t slot, uint8_t total,
                                          uint8_t width, uint64_t last) {
    uint32_t frame = (uint32_t)total * width;
    uint32_t phase = (uint32_t)(now % frame);
    uint32_t start = (uint32_t)slot * width;
    uint32_t target = start + TDMA_CAPTURE_OFFSET_TICKS;
    uint64_t absolute = now - phase + start;
    /* Receiver reboot (or its 32-bit tick wrap) changes the clock epoch.
     * Keep the one-attempt guard across small clock corrections, but an old
     * slot more than a frame in the future must not block a new clock for
     * hours. A successful claim stores the new absolute slot immediately. */
    if (last != UINT64_MAX && last > now && last - now > frame) last = UINT64_MAX;
    if (phase >= target && phase <= start + width - TDMA_CAPTURE_BUDGET_TICKS &&
        (last == UINT64_MAX || absolute > last)) return 0;
    uint32_t wait = (target + frame - phase) % frame;
    return wait ? wait : frame;
}
#endif

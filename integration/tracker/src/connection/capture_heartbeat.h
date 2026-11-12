/* Capture recovery must not inherit the battery-saving offline backoff.
 * Per-attempt jitter breaks repeated collisions with other trackers/TDMA slots.
 * No extra traffic or scheduling change outside an explicit F04 session. */
#ifndef CAPTURE_HEARTBEAT_H
#define CAPTURE_HEARTBEAT_H
#include <stdbool.h>
#include <stdint.h>
static inline uint32_t capture_heartbeat_interval(uint8_t slot, uint32_t attempt,
                                                  int64_t sync_age_ms)
{
    uint32_t mixed = (attempt + 1u) * 1664525u + (slot + 1u) * 1013904223u;
    mixed ^= mixed >> 16;
    /* Renew before the strict 1994 ms sync expiry; retry quickly even after
     * the sync state has been cleared (age == -1). */
    return (sync_age_ms < 0 || sync_age_ms >= 1000 ? 160u : 600u) + mixed % 151u;
}
static inline bool heartbeat_reply_matches(uint8_t local_id, uint8_t reply_id,
                                          uint8_t sent_counter, uint8_t reply_counter,
                                          bool pending)
{
    return pending && local_id == reply_id && sent_counter == reply_counter;
}
#endif

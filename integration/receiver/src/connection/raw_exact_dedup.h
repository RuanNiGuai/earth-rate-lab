/*
 * Draft receiver-side raw packet integrity filter, standalone C11.
 * Copyright (c) 2026. MIT License, consistent with receiver source.
 *
 * Only byte-identical raw retransmissions are suppressed. A second packet
 * with the same type/slot/sequence but differing bytes is FORWARDED so the
 * host capture integrity gate can report the conflict. No sample reordering,
 * synthetic data, ISR work, heap allocation or protocol changes are made.
 *
 * Keep this header next to esb_receiver.c (or src/connection/esb.c).
 */
#ifndef ARC8_RAW_EXACT_DEDUP_H
#define ARC8_RAW_EXACT_DEDUP_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define ARC8_RAW_RECENT_CAPACITY 8U
/* 0x10 raw IMU = 48 B; 0x13 raw IMU+quat = 52 B in the host protocol.
 * A smaller cache silently disables protection for normal collection frames.
 * Keep the C tests and wire-format length assertions synchronized.
 */
#define ARC8_RAW_RECENT_MAX_BYTES 52U

struct arc8_raw_recent_packet {
    uint8_t length;
    uint8_t data[ARC8_RAW_RECENT_MAX_BYTES];
};

struct arc8_raw_exact_dedup {
    struct arc8_raw_recent_packet recent[ARC8_RAW_RECENT_CAPACITY];
    uint8_t used;
    uint8_t cursor;
    uint32_t identical_suppressed;
    uint32_t conflicts_forwarded;
    uint32_t unchecked_forwarded;
};

enum arc8_raw_guard_decision {
    ARC8_RAW_KEEP = 0,
    ARC8_RAW_DROP_BYTE_IDENTICAL = 1,
    ARC8_RAW_KEEP_SAME_SEQUENCE_CONFLICT = 2,
    ARC8_RAW_KEEP_UNCHECKED = 3
};

static inline void arc8_raw_dedup_reset(struct arc8_raw_exact_dedup *guard)
{
    memset(guard, 0, sizeof(*guard));
}

static inline enum arc8_raw_guard_decision
arc8_raw_dedup_observe(struct arc8_raw_exact_dedup *guard, const uint8_t *data, uint8_t length)
{
    bool conflict = false;
    if (data == NULL || length < 4U || length > ARC8_RAW_RECENT_MAX_BYTES) {
        guard->unchecked_forwarded++;
        return ARC8_RAW_KEEP_UNCHECKED;
    }
    /* The caller only invokes this on raw IMU 0x10 or raw QUAT IMU 0x13.
     * Bytewise comparisons include packet type, tracker id, and sequence.
     */
    for (uint8_t i = 0; i < guard->used; i++) {
        const struct arc8_raw_recent_packet *prior = &guard->recent[i];
        if (prior->length == length && memcmp(prior->data, data, length) == 0) {
            guard->identical_suppressed++;
            return ARC8_RAW_DROP_BYTE_IDENTICAL;
        }
        if (prior->length >= 4U &&
            memcmp(prior->data, data, 4U) == 0) {
            /* Same type, tracker and sequence; differing payload is NOT a duplicate. */
            conflict = true;
        }
    }
    struct arc8_raw_recent_packet *target = &guard->recent[guard->cursor];
    target->length = length;
    memcpy(target->data, data, length);
    guard->cursor = (uint8_t)((guard->cursor + 1U) % ARC8_RAW_RECENT_CAPACITY);
    if (guard->used < ARC8_RAW_RECENT_CAPACITY) {
        guard->used++;
    }
    if (conflict) {
        guard->conflicts_forwarded++;
        return ARC8_RAW_KEEP_SAME_SEQUENCE_CONFLICT;
    }
    return ARC8_RAW_KEEP;
}

#endif /* ARC8_RAW_EXACT_DEDUP_H */

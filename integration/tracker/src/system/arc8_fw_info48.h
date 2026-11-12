/* ARC8 48-byte OTA FW_INFO encoder: v2 receiver/PC binary ABI.
 * No Zephyr dependencies; built-in host test uses the SAME function.
 * The size field is the MCUboot slot0 capacity, not the exact ELF size.
 */
#ifndef ARC8_FW_INFO48_H
#define ARC8_FW_INFO48_H
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#define ARC8_FW_INFO48_LENGTH 48u

static inline void arc8_fw_info48_pack(uint8_t out[ARC8_FW_INFO48_LENGTH],
        uint8_t tracker_id, uint8_t major, uint8_t minor, uint8_t patch,
        unsigned year, unsigned month, unsigned day,
        unsigned hour, unsigned minute, unsigned second,
        uint32_t slot0_capacity, const char *board)
{
    memset(out, 0, ARC8_FW_INFO48_LENGTH);
    out[0] = 0x22; /* ESB_OTA_FW_INFO_TYPE */
    out[1] = tracker_id;
    out[2] = major; out[3] = minor; out[4] = patch;
    const uint32_t d = (((year - 2020u) & 0x7Fu) << 25) |
                       ((month & 0x0Fu) << 21) |
                       ((day & 0x1Fu) << 16) |
                       ((hour & 0x1Fu) << 11) |
                       ((minute & 0x3Fu) << 5) |
                       ((second / 2u) & 0x1Fu);
    out[5] = (uint8_t)(d >> 24); out[6] = (uint8_t)(d >> 16);
    out[7] = (uint8_t)(d >> 8); out[8] = (uint8_t)d;
    out[9]  = (uint8_t)slot0_capacity;
    out[10] = (uint8_t)(slot0_capacity >> 8);
    out[11] = (uint8_t)(slot0_capacity >> 16);
    out[12] = (uint8_t)(slot0_capacity >> 24);
    out[13] = 3u; /* MCUboot */
    out[14] = 2u; /* OTA protocol v2 */
    if (board) {
        size_t n = 0;
        while (n < 29u && board[n]) { out[15 + n] = (uint8_t)board[n]; n++; }
    }
    /* out[45..47] reserved, zero. No CRC present in this 48-byte profile. */
}
#endif

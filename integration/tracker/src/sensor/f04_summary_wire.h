/* F04 v1: explicit little-endian wire format; shared verbatim with Receiver.
 * CRC detects corruption, not authenticity. All timestamps are estimated.
 */
#ifndef F04_SUMMARY_WIRE_H
#define F04_SUMMARY_WIRE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define F04_SIZE 48u
#define F04_TYPE 0x15u
#define F04_VERSION 1u
#define F04_ESTIMATED_TIME 1u
#define F04_END_MANIFEST 2u
static inline uint16_t f04_u16(const uint8_t *p) {
    return (uint16_t)p[0] | (uint16_t)p[1] << 8;
}
static inline uint32_t f04_u32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}
static inline void f04_put16(uint8_t *p, uint16_t v) {
    p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8);
}
static inline void f04_put32(uint8_t *p, uint32_t v) {
    p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24);
}
static inline uint32_t f04_crc(const uint8_t *p, size_t n) {
#ifdef F04_CRC_OVERRIDE
    return F04_CRC_OVERRIDE(p,n);
#else
    uint32_t c=0xffffffffu;
    for (size_t i=0; i<n; i++) {
        c^=p[i];
        for (unsigned b=0; b<8; b++) c=(c>>1) ^ (0xedb88320u & (0u-(c&1u)));
    }
    return ~c;
#endif
}
static inline bool f04_valid(const uint8_t *p, size_t n) {
    if (!p || n!=F04_SIZE || p[0]!=F04_TYPE || p[1]>=32 || p[2]!=F04_VERSION ||
        p[3]!=F04_ESTIMATED_TIME || !f04_u32(p+4) || !f04_u32(p+8) ||
        p[40]<2 || p[40]>32 || p[41]>p[40] ||
        f04_u16(p+42)<10000 || f04_u16(p+42)>30000 ||
        f04_u32(p+44)!=f04_crc(p,44)) return false;
    /* Bounded physical representation. Reject even a CRC-correct impossible frame. */
    for (unsigned i=0;i<3;i++) {
        int32_t g=(int32_t)f04_u32(p+20+4*i);
        int16_t a=(int16_t)f04_u16(p+32+2*i);
        if ((int64_t)g > (int64_t)2500*f04_u16(p+42) ||
            (int64_t)g < -(int64_t)2500*f04_u16(p+42) || a>32000 || a< -32000) return false;
    }
    int16_t t=(int16_t)f04_u16(p+38);
    return t>=-10000 && t<=15000;
}
/* End manifest: boot/op plus exclusive sequence end and conservation counts.
 * All fields at 12..40 are u32. Never interpret this as an IMU window. */
static inline bool f04_end_valid(const uint8_t *p, size_t n) {
    if (!p || n != F04_SIZE || p[0] != F04_TYPE || p[1] >= 32 ||
        p[2] != F04_VERSION || p[3] != F04_END_MANIFEST ||
        !f04_u32(p+4) || !f04_u32(p+8) || f04_crc(p,44) != f04_u32(p+44)) return false;
    return (uint64_t)f04_u32(p+16) + f04_u32(p+20) == f04_u32(p+12) &&
        (uint64_t)f04_u32(p+24) + f04_u32(p+28) + f04_u32(p+32) == f04_u32(p+16) &&
        f04_u32(p+40) <= 32;
}
static inline bool f04_frame_valid(const uint8_t *p, size_t n) {
    return f04_valid(p,n) || f04_end_valid(p,n);
}
#endif

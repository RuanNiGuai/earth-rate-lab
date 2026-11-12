/* F04 receipt: receiver validated and copied this exact frame into its HID
 * queue. This is not a host/disk acknowledgement or authentication. */
#ifndef F04_CAPTURE_ACK_H
#define F04_CAPTURE_ACK_H
#include "f04_summary_wire.h"
#define F04_ACK_TYPE 0x28u
#define F04_ACK_SIZE 24u
static inline void f04_ack_make(uint8_t ack[F04_ACK_SIZE], const uint8_t frame[F04_SIZE]) {
    ack[0]=F04_ACK_TYPE;
    for (unsigned i=1;i<16;i++) ack[i]=frame[i];
    for (unsigned i=0;i<4;i++) ack[16+i]=frame[44+i];
    f04_put32(ack+20,f04_crc(ack,20));
}
static inline bool f04_ack_matches(const uint8_t *ack, size_t size, const uint8_t frame[F04_SIZE]) {
    if (!ack || size!=F04_ACK_SIZE || ack[0]!=F04_ACK_TYPE ||
        f04_u32(ack+20)!=f04_crc(ack,20)) return false;
    for (unsigned i=1;i<16;i++) if (ack[i]!=frame[i]) return false;
    return f04_u32(ack+16)==f04_u32(frame+44);
}
#endif

/* CALWIRE v1: bounded request/reply, independent of OTA and pose packets.
 * Address + CRC bind accidental routing/corruption, not cryptographic auth.
 * 0:type 1:slot 2:version 3:op 4:session(u32) 8:RF(6 LE)
 * 14:offset/field(u16) 16:length 17:status 18:payload(26) 44:CRC32.
 * BEGIN=0 READ=1 PUT=2 CHECK=3 APPLY=4 END=5. No arbitrary console/NVS writes.
 */
#ifndef CALWIRE_PROTOCOL_H
#define CALWIRE_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#define CW_SIZE 48u
#define CW_DATA 26u
#define CW_RESPONSE_DATA 22u /* First four reply payload bytes echo request CRC. */
static inline uint32_t cw_u32(const uint8_t *p) {
    return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}
static inline unsigned cw_u16(const uint8_t *p) { return p[0]|((unsigned)p[1]<<8); }
static inline void cw_put32(uint8_t *p,uint32_t v) { for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(i*8)); }
static inline void cw_put16(uint8_t *p,unsigned v) {p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static inline uint32_t cw_crc(const void *ptr,size_t n) {
    const uint8_t *p=ptr;uint32_t c=~0u;
    while(n--){c^=*p++;for(unsigned k=0;k<8;k++)c=(c>>1)^((0u-(c&1u))&0xedb88320u);}
    return ~c;
}
static inline bool cw_valid(const uint8_t *p,size_t n,uint8_t type) {
    if(n!=CW_SIZE||p[0]!=type||p[1]>=32||p[2]!=1||p[3]>5||!cw_u32(p+4)||p[16]>CW_DATA||
       (type==0x26&&p[17])||cw_u32(p+44)!=cw_crc(p,44))return false;
    for(unsigned k=18+p[16];k<44;k++)if(p[k])return false;
    return true;
}
static inline unsigned cw_field_size(unsigned field){return field==2?36:(field==1||field==3?12:0);}
static inline unsigned cw_field_offset(unsigned field){return field==1?0:(field==2?12:48);}
static inline bool cw_profile_valid(const uint8_t *p,unsigned field) {
    unsigned n=cw_field_size(field);if(!n)return false;
    float sum=0;
    for(unsigned j=0;j<n/4;j++){
        float v;memcpy(&v,p+4*j,4);if(!isfinite(v))return false;
        if(field==1&&(v<.9f||v>1.1f))return false;
        if(field==2&&fabsf(v)>.05f)return false;
        sum+=v*v;
    }
    return field!=3||fabsf(sqrtf(sum)-.0041780746f)<.0000001f;
}
#endif

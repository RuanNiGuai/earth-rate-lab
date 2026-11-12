#include "f04_summary_core.h"
#include <math.h>
#include <string.h>
static void clear_window(f04_accumulator *s) {
    s->duration_ns=0; s->count=0; s->fresh=0; s->invalid=false;
    memset(s->delta,0,sizeof(s->delta)); memset(s->accel,0,sizeof(s->accel));
    s->temperature=0;
}
bool f04_init(f04_accumulator *s, uint8_t tracker, uint32_t boot, uint32_t op, uint32_t anchor) {
    if (!s || tracker>=32 || !boot || !op) return false;
    memset(s,0,sizeof(*s)); s->tracker=tracker; s->boot=boot; s->operation=op; s->anchor_us=anchor;
    return true;
}
void f04_discontinuity(f04_accumulator *s, uint32_t anchor_us) {
    s->sequence++; s->bad++; s->phase_ns=0; s->elapsed_ns=0; s->dt_ns=0;
    s->anchor_us=anchor_us; clear_window(s);
}
int f04_add(f04_accumulator *s, const float gyro[3], const float accel[3],
            float temp, float dt_s, bool available, bool fresh, uint8_t out[F04_SIZE]) {
    if (!isfinite(dt_s) || dt_s<0.000625f || dt_s>0.010001f) {
        s->invalid=true;
        return 0; /* Caller must reanchor on invalid timing. */
    }
    uint32_t dt=(uint32_t)llround((double)dt_s*1e9);
    if (s->dt_ns && s->dt_ns!=dt) s->invalid=true;
    s->dt_ns=dt; s->phase_ns+=dt; s->duration_ns+=dt; s->elapsed_ns+=dt;
    s->count++; s->fresh+=available && fresh;
    bool valid=available && isfinite(temp) && temp>=-100 && temp<=150;
    for (unsigned i=0;i<3;i++)
        valid=valid && isfinite(gyro[i]) && fabsf(gyro[i])<1999.9f &&
               isfinite(accel[i]) && fabsf(accel[i])<31.999f;
    if (!valid) s->invalid=true;
    else {
        for (unsigned i=0;i<3;i++) {
            s->delta[i]+=(double)gyro[i]*(double)dt/1000.0; /* microdegrees */
            s->accel[i]+=(double)accel[i]*1000.0;
        }
        s->temperature+=(double)temp*100.0;
    }
    if (s->phase_ns<20000000u) return 0;
    s->phase_ns-=20000000u;
    memset(out,0,F04_SIZE);
    out[0]=F04_TYPE; out[1]=s->tracker; out[2]=F04_VERSION; out[3]=F04_ESTIMATED_TIME;
    f04_put32(out+4,s->boot); f04_put32(out+8,s->operation);
    f04_put32(out+12,s->sequence++);
    f04_put32(out+16,s->anchor_us+(uint32_t)(s->elapsed_ns/1000u));
    if (!s->invalid) {
        for (unsigned i=0;i<3;i++) {
            f04_put32(out+20+4*i,(uint32_t)(int32_t)llround(s->delta[i]));
            f04_put16(out+32+2*i,(uint16_t)(int16_t)lround(s->accel[i]/s->count));
        }
        f04_put16(out+38,(uint16_t)(int16_t)lround(s->temperature/s->count));
    }
    out[40]=s->count; out[41]=s->fresh;
    f04_put16(out+42,(uint16_t)((s->duration_ns+500)/1000));
    f04_put32(out+44,f04_crc(out,44));
    int result=(!s->invalid && f04_valid(out,F04_SIZE)) ? 1 : -1;
    if (result==1) s->ok++; else s->bad++;
    clear_window(s);
    return result;
}

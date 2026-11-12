#include "gyro_scale.h"
#include <math.h>
#include <string.h>

void gyro_scale_fail(gyro_scale_t *s, const char *reason)
{
    s->phase=GYRO_SCALE_FAILED; s->error=reason;
}

bool gyro_scale_begin(gyro_scale_t *s, int turns, int axis, const float scales[3])
{
    memset(s,0,sizeof(*s));
    if(turns<1 || turns>20 || axis<0 || axis>2) {
        gyro_scale_fail(s,"turns must be 1..20 and axis X/Y/Z"); return false;
    }
    for(int i=0;i<3;i++) if(!isfinite(scales[i]) || scales[i]<=0.5f || scales[i]>=1.5f) {
        gyro_scale_fail(s,"invalid existing scale"); return false;
    }
    s->axis=axis; s->turns=turns;
    memcpy(s->initial_scale,scales,sizeof(s->initial_scale));
    memcpy(s->result_scale,scales,sizeof(s->result_scale));
    s->phase=GYRO_SCALE_PREPARE;
    return true;
}

void gyro_scale_sample(gyro_scale_t *s, const float g[3], const float a[3],
                       float temp, float dt, bool rest, bool valid)
{
    if(s->phase!=GYRO_SCALE_PREPARE && s->phase!=GYRO_SCALE_RUN) return;
    if(!valid || !isfinite(temp) || !isfinite(dt) || dt<=0 || dt>.01f) {
        gyro_scale_fail(s,"invalid or missing samples"); return;
    }
    for(int i=0;i<3;i++) if(!isfinite(g[i]) || !isfinite(a[i])) {
        gyro_scale_fail(s,"nonfinite sensor input"); return;
    }
    if(s->sample_dt && fabsf(dt/s->sample_dt-1)>0.01f) {
        gyro_scale_fail(s,"sample rate changed"); return;
    }
    s->sample_dt=dt; s->elapsed+=dt;
    if(s->elapsed>600 || (s->phase==GYRO_SCALE_PREPARE && s->elapsed>30)) {
        gyro_scale_fail(s,"calibration timeout"); return;
    }
    float an=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    float gn=sqrtf(g[0]*g[0]+g[1]*g[1]+g[2]*g[2]);
    bool vertical=an>.90f && an<1.10f && fabsf(a[s->axis])/an>.98480775f;
    if(s->phase==GYRO_SCALE_PREPARE) {
        if(!rest || gn>2 || !vertical) {
            s->seed_time=0; s->seed_samples=0;
            memset(s->seed_sum,0,sizeof(s->seed_sum));
            memset(s->seed_square,0,sizeof(s->seed_square));
            return;
        }
        if(s->seed_samples==0) s->start_temp=temp;
        if(fabsf(temp-s->start_temp)>.3f) {
            gyro_scale_fail(s,"temperature unstable during bias measurement"); return;
        }
        for(int i=0;i<3;i++) {
            s->seed_sum[i]+=(double)g[i]; s->seed_square[i]+=(double)g[i]*(double)g[i];
        }
        s->seed_time+=dt; s->seed_samples++;
        if(s->seed_time<2) return;
        for(int i=0;i<3;i++) {
            double mean=s->seed_sum[i]/s->seed_samples;
            double var=s->seed_square[i]/s->seed_samples-mean*mean;
            if(var>.04) { gyro_scale_fail(s,"gyro not stable during bias measurement"); return; }
            s->bias[i]=(float)mean;
            s->start_accel[i]=a[i]/an;
        }
        s->start_temp=temp; s->phase=GYRO_SCALE_RUN;
        return;
    }
    if(fabsf(temp-s->start_temp)>.5f) {
        gyro_scale_fail(s,"temperature changed by more than 0.5 C"); return;
    }
    if(!vertical || a[s->axis]*s->start_accel[s->axis]<=0) {
        gyro_scale_fail(s,"keep the selected axis vertical and acceleration near gravity"); return;
    }
    float rate[3];
    float rate_norm2=0, cross_norm2=0;
    for(int i=0;i<3;i++) {
        rate[i]=g[i]-s->bias[i]; rate_norm2+=rate[i]*rate[i];
        s->angle[i]+=(double)rate[i]*(double)dt;
        if(i!=s->axis) cross_norm2+=rate[i]*rate[i];
    }
    float selected=rate[s->axis];
    if(rate_norm2>25) s->cross_angle+=(double)(sqrtf(cross_norm2)*dt);
    if(fabsf(selected)>5) {
        int dir=selected>0 ? 1 : -1;
        if(!s->direction) s->direction=dir;
        s->absolute_angle+=(double)(fabsf(selected)*dt);
        if(dir!=s->direction) s->reverse_angle+=(double)(fabsf(selected)*dt);
        if(s->reverse_angle>2) { gyro_scale_fail(s,"rotation reversed"); return; }
    }
    if(s->cross_angle>2 && s->cross_angle>fmax(s->absolute_angle*.05,2)) {
        gyro_scale_fail(s,"excessive cross-axis rotation"); return;
    }
    if(rest && rate_norm2<.25f) s->end_rest_time+=dt;
    else s->end_rest_time=0;
}

bool gyro_scale_finish(gyro_scale_t *s)
{
    if(s->phase!=GYRO_SCALE_RUN) {
        if(s->phase!=GYRO_SCALE_FAILED) gyro_scale_fail(s,"wait for READY before rotating");
        return false;
    }
    if(s->end_rest_time<1) {
        gyro_scale_fail(s,"need 1s detected rest; normally hold still at least 3s before stop"); return false;
    }
    double target=360.0*s->turns;
    double measured=s->angle[s->axis];
    double corrected=measured*(double)s->initial_scale[s->axis];
    /* Less than a quarter turn avoids accepting N +/- 1 turns at large N.
     * The relative bound still allows initial ~0.7% gain correction. */
    double tolerance=fmin(target*.05,90.0);
    if(!s->direction || s->direction*measured<=0 || fabs(fabs(corrected)-target)>tolerance) {
        gyro_scale_fail(s,"measured angle does not match declared turns (5 percent and 90 degree limits)"); return false;
    }
    float result=(float)(target/fabs(measured));
    if(!isfinite(result) || result<=.5f || result>=1.5f) {
        gyro_scale_fail(s,"result outside allowed sensitivity range"); return false;
    }
    s->result_scale[s->axis]=result; s->phase=GYRO_SCALE_DONE;
    return true;
}

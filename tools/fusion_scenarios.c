/* SPDX-License-Identifier: MIT
 * Synthetic identifiability/counterexample probes, NOT a wearable benchmark.
 * Execute actual vendored cores. No magnetometer, Earth correction, firmware
 * observation gates, radio, sensor noise, or missing-sample reconstruction.
 */
#include <stdio.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "vqf.c"
#include "FusionAhrs.c"
#include "FusionCompass.c"
#include "FusionOffset.c"

static float yaw(const float q[4]) {
    return atan2f(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))*180/M_PI;
}
static float tilt(const float q[4]) {
    return acosf(fmaxf(-1,fminf(1,1-2*(q[1]*q[1]+q[2]*q[2]))))*180/M_PI;
}
static const char *names[]={"vqf_fixed_bias_learning","vqf_fixed_bias_frozen",
    "fusion_ahrs_with_offset","fusion_ahrs_bias_frozen"};

static void run_case(int which, int mode) {
    const float dt=.01f;
    vqf_params_t p;vqf_state_t s;vqf_coeffs_t c;
    init_params(&p);
    p.tauAcc=4.3f;p.biasClip=2;p.biasForgettingTime=427;
    p.biasSigmaInit=1.10f;p.biasSigmaMotion=.048f;p.biasSigmaRest=.0153f;
    p.biasVerticalForgettingFactor=.0001f;p.restFilterTau=.66f;
    p.restMinT=.63f;p.restThGyr=.68f;p.restThAcc=.21f;
    if(mode==1)p.restBiasEstEnabled=p.motionBiasEstEnabled=false;
    initVqf(&p,&s,&c,dt,dt,dt);
    FusionAhrs f;FusionOffset off;
    FusionAhrsInitialise(&f);FusionOffsetInitialise(&off,100);
    // Explicit Fusion defaults, with rejection/recovery enabled.
    FusionAhrsSettings fs={FusionConventionNwu,.5f,2000,10,10,500};
    FusionAhrsSetSettings(&f,&fs);
    float q[4]={1,0,0,0},peak=0;
    for(int i=0;i<9000;i++) {
        float a[3]={0,0,9.80665f},g[3]={0,0,0};
        if(i>=3000) {
            if(which==0)g[2]=.2f*M_PI/180;
            else if(which==2 || i<3500) {
                // Total acceleration still has norm g: defeats norm-only gates.
                a[0]=9.80665f*sinf(20*M_PI/180);a[2]=9.80665f*cosf(20*M_PI/180);
            }
        }
        if(mode<2) {
            updateGyr(&p,&s,&c,g);updateAcc(&p,&s,&c,a);getQuat6D(&s,q);
        } else {
            FusionVector gv={{g[0]*180/M_PI,g[1]*180/M_PI,g[2]*180/M_PI}};
            FusionVector av={{a[0]/9.80665f,a[1]/9.80665f,a[2]/9.80665f}};
            if(mode==2)gv=FusionOffsetUpdate(&off,gv);
            FusionAhrsUpdateNoMagnetometer(&f,gv,av,dt);
            FusionQuaternion fq=FusionAhrsGetQuaternion(&f);memcpy(q,fq.array,sizeof(q));
            normalize(q,4);
        }
        if(i>=3000)peak=fmaxf(peak,tilt(q));
    }
    printf("{\"algorithm\":\"%s\",\"yaw_deg\":%.6f,\"peak_tilt_deg\":%.6f,\"final_tilt_deg\":%.6f}",
        names[mode],yaw(q),peak,tilt(q));
}
static float timing_case(int policy) {
    vqf_params_t p;vqf_state_t s;vqf_coeffs_t c;init_params(&p);
    p.restBiasEstEnabled=p.motionBiasEstEnabled=false;initVqf(&p,&s,&c,.01f,.01f,.01f);
    int last=0;
    for(int i=1;i<=200;i++) {
        if(policy && i%4==1)continue;
        float g[3]={0,0,30*M_PI/180};
        float dt=policy==2?(i-last)*.01f:.01f;
        updateGyr_internal(&p,&s,&c,dt,g);last=i;
    }
    float q[4];getQuat6D(&s,q);return yaw(q);
}
int main(void) {
    const char *cases[]={"same_input_static_bias_or_slow_yaw","five_second_acceleration","sustained_acceleration"};
    printf("{\"cases\":{");
    for(int j=0;j<3;j++) {
        printf("%s\"%s\":[",j?",":"",cases[j]);
        for(int k=0;k<4;k++){if(k)printf(",");run_case(j,k);}printf("]");
    }
    printf("},\"constant_rate_25_percent_missing\":{\"truth_deg\":60,\"complete_deg\":%.6f,\"fixed_step_deg\":%.6f,\"elapsed_step_deg\":%.6f}}\n",
        timing_case(0),timing_case(1),timing_case(2));
}

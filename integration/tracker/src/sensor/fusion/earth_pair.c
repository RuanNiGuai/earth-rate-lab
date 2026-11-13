#include "earth_pair.h"
#include <math.h>
#include <string.h>

typedef struct {
    uint32_t version;
    bool auto_pending, motion_gate;
    uint8_t accel_gate_mode, reserved;
    uint32_t samples;
    earth_pair_t pair;
    uint32_t checksum;
} earth_pair_snapshot_t;
_Static_assert(sizeof(earth_pair_snapshot_t)<=EARTH_PAIR_SNAPSHOT_CAPACITY,
    "Earth snapshot exceeds retained buffer");

static uint32_t snapshot_checksum(const earth_pair_snapshot_t *record)
{
    const unsigned char *p=(const unsigned char *)record;
    uint32_t hash=2166136261u;
    for(size_t i=0;i<offsetof(earth_pair_snapshot_t,checksum);i++) hash=(hash^p[i])*16777619u;
    return hash;
}

static bool snapshot_pair_valid(const earth_pair_t *p)
{
    if(!p->enabled) return true;
    if(p->mode!=PAIR_EARTH && p->mode!=PAIR_REST_BIAS) return false;
    if(!isfinite(p->coeffs.gyrTs) || p->coeffs.gyrTs<=0 ||
       !isfinite(p->coeffs.accTs) || p->coeffs.accTs<=0) return false;
    for(int i=0;i<3;i++)
        if(!isfinite(p->earth_world[i]) || !isfinite(p->state.bias[i])) return false;
    float norm_g=0, norm_a=0;
    for(int i=0;i<4;i++) {
        if(!isfinite(p->state.gyrQuat[i]) || !isfinite(p->state.accQuat[i])) return false;
        norm_g+=p->state.gyrQuat[i]*p->state.gyrQuat[i];
        norm_a+=p->state.accQuat[i]*p->state.accQuat[i];
    }
    return fabsf(norm_g-1)<0.01f && fabsf(norm_a-1)<0.01f;
}

bool earth_pair_save(const earth_pair_t *pair, bool auto_pending, bool motion_gate,
    uint32_t samples, uint8_t accel_gate_mode, void *storage, size_t capacity)
{
    if(capacity<sizeof(earth_pair_snapshot_t) || accel_gate_mode>2) return false;
    memset(storage,0,capacity);
    if(!snapshot_pair_valid(pair)) return false;
    earth_pair_snapshot_t record={0};
    record.version=VQF_STATE_VERSION; record.auto_pending=auto_pending; record.motion_gate=motion_gate;
    record.accel_gate_mode=accel_gate_mode;
    record.samples=samples; record.pair=*pair;
    record.checksum=snapshot_checksum(&record);
    memcpy(storage,&record,sizeof(record));
    return true;
}

bool earth_pair_load(earth_pair_t *pair, bool *auto_pending, bool *motion_gate,
    uint32_t *samples, uint8_t *accel_gate_mode, const void *storage, size_t capacity)
{
    if(capacity<sizeof(earth_pair_snapshot_t)) return false;
    earth_pair_snapshot_t record;
    memcpy(&record,storage,sizeof(record));
    if(record.version!=VQF_STATE_VERSION || record.accel_gate_mode>2 || record.checksum!=snapshot_checksum(&record) || !snapshot_pair_valid(&record.pair)) return false;
    *pair=record.pair; *auto_pending=record.auto_pending;
    *motion_gate=record.motion_gate; *samples=record.samples;
    *accel_gate_mode=record.accel_gate_mode;
    return true;
}


void earth_pair_project(const float q[4], const float e[3], float b[3])
{
    const float w=q[0], x=q[1], y=q[2], z=q[3];
    b[0]=(1-2*(y*y+z*z))*e[0]+2*(x*y+w*z)*e[1]+2*(x*z-w*y)*e[2];
    b[1]=2*(x*y-w*z)*e[0]+(1-2*(x*x+z*z))*e[1]+2*(y*z+w*x)*e[2];
    b[2]=2*(x*z+w*y)*e[0]+2*(y*z-w*x)*e[1]+(1-2*(x*x+y*y))*e[2];
}

static bool pair_state_settled(const vqf_state_t *s)
{
    if (!s->restDetected || !isfinite(s->delta) || fabsf(s->delta)>1e-6f) return false;
    for (int i=0;i<6;i++) if (!isfinite(s->restGyrLpState[i])) return false;
    for (int i=0;i<6;i++) if (!isfinite(s->restAccLpState[i])) return false;
    for (int i=0;i<6;i++) if (!isfinite(s->accLpState[i])) return false;
    for (int i=0;i<18;i++) if (!isfinite(s->motionBiasEstRLpState[i])) return false;
    for (int i=0;i<4;i++) if (!isfinite(s->motionBiasEstBiasLpState[i])) return false;
    return true;
}

bool earth_pair_start(earth_pair_t *p, const vqf_params_t *params,
    vqf_state_t *s, const vqf_coeffs_t *c, const float e[3])
{
    if (!pair_state_settled(s)) return false;
    for (int i=0;i<3;i++) if (!isfinite(e[i])) return false;
    float q[4], d[3];
    getQuat6D(s,q);
    earth_pair_project(q,e,d);
    p->params=*params; p->state=*s; p->coeffs=*c;
    memcpy(p->earth_world,e,sizeof(p->earth_world));
    /* The original rest bias already absorbed Earth. Translate bias AND its
     * filter histories once, so explicit subtraction does not subtract twice. */
    for (int j=0;j<3;j++) {
        p->state.bias[j]-=d[j];
        p->state.restLastGyrLp[j]-=d[j];
        #if VQF_STABLE_IIR
        p->state.restGyrLpState[2*j+1]-=d[j];
#else
        p->state.restGyrLpState[2*j]-=d[j]*(1-c->restGyrLpB[0]);
        p->state.restGyrLpState[2*j+1]-=d[j]*(c->restGyrLpB[2]-c->restGyrLpA[1]);
#endif
    }
    for (int row=0;row<2;row++) for (int z=0;z<2;z++)
        for (int k=0;k<3;k++)
            p->state.motionBiasEstBiasLpState[2*row+z]-=
                s->motionBiasEstRLpState[2*(3*row+k)+z]*d[k];
    /* A prescribed deterministic input translation keeps covariance unchanged.
     * Uncertain north is an experimental hypothesis, not a calibrated estimate. */
    p->mode=PAIR_EARTH;
    p->enabled=true;
    return true;
}

bool earth_pair_start_rest_bias(earth_pair_t *p, const vqf_params_t *params,
    vqf_state_t *s, const vqf_coeffs_t *c)
{
    if (!pair_state_settled(s) || !params->restBiasEstEnabled) return false;
    /* An exact state copy isolates motion bias updates. Rest detection, rest
     * bias estimation, covariance propagation and gyro integration are kept. */
    p->params=*params; p->state=*s; p->coeffs=*c;
    p->params.motionBiasEstEnabled=false;
    p->params.restBiasEstEnabled=true;
    memset(p->earth_world,0,sizeof(p->earth_world));
    p->mode=PAIR_REST_BIAS;
    p->enabled=true;
    return true;
}

void earth_pair_gyro(earth_pair_t *p, const float g[3])
{
    if (!p->enabled) return;
    if (p->mode==PAIR_REST_BIAS) {
        updateGyr(&p->params,&p->state,&p->coeffs,g);
        return;
    }
    float q[4], e[3], corrected[3];
    getQuat6D(&p->state,q);
    earth_pair_project(q,p->earth_world,e);
    for (int k=0;k<3;k++) corrected[k]=g[k]-e[k];
    updateGyr(&p->params,&p->state,&p->coeffs,corrected);
}

void earth_pair_accel(earth_pair_t *p, const float a[3])
{
    if (p->enabled) updateAcc(&p->params,&p->state,&p->coeffs,a);
}

void earth_pair_accel_gated(earth_pair_t *p, const float a[3], bool trusted)
{
    if (p->enabled) updateAccGated(&p->params,&p->state,&p->coeffs,a,trusted);
}

/* A dedicated state-preserving yaw correction path: unlike vqf_set_quat(),
 * it leaves the pair enabled and does not reinitialize the gyro estimator.
 * Earth/world Z is the VQF vertical axis. See README for preconditions.
 */
static bool pair_can_rotate_yaw(const earth_pair_t *p, float delta, bool relabel)
{
    if (!p || !p->enabled || p->mode != PAIR_EARTH ||
        !isfinite(delta) || fabsf(delta) > (relabel ? 3.141592654f : 0.7853981634f)) return false;
    float n = 0.0f;
    for (int i=0; i<4; i++) {
        if (!isfinite(p->state.accQuat[i]) || !isfinite(p->state.gyrQuat[i])) return false;
        n += p->state.accQuat[i]*p->state.accQuat[i];
    }
    return n >= 0.98f && n <= 1.02f;
}

static void rotate_xy_real(vqf_real_t *x, vqf_real_t *y, double cs, double sn)
{
    const double a = *x, b = *y;
    *x = cs*a - sn*b;
    *y = sn*a + cs*b;
}

/* In VQF getQuat6D = accQuat * gyrQuat. World-yaw changes must be left-
 * multiplied into accQuat, not gyrQuat. The two horizontal rows of the
 * motion-bias estimator's R IIR histories are world-frame values, and must
 * undergo the same world rotation. Horizontal R*b lowpass states also rotate.
 */
static void pair_rotate_world_state(earth_pair_t *p, float yaw)
{
    const float cs = cosf(yaw), sn = sinf(yaw);
    const float cw = cosf(0.5f*yaw), sz = sinf(0.5f*yaw);
    float *a = p->state.accQuat;
    const float w=a[0], x=a[1], y=a[2], z=a[3];
    a[0]=cw*w-sz*z;
    a[1]=cw*x-sz*y;
    a[2]=cw*y+sz*x;
    a[3]=cw*z+sz*w;
    for (int col=0; col<3; col++) for (int tap=0; tap<2; tap++) {
        const int ix = 2*(3*0+col)+tap, iy=2*(3*1+col)+tap;
        rotate_xy_real(&p->state.motionBiasEstRLpState[ix],
                         &p->state.motionBiasEstRLpState[iy],cs,sn);
    }
    for (int tap=0; tap<2; tap++) {
        rotate_xy_real(&p->state.motionBiasEstBiasLpState[tap],
                         &p->state.motionBiasEstBiasLpState[2+tap],cs,sn);
    }
}

static void pair_translate_earth_subtraction(earth_pair_t *p, const float diff[3])
{
#if !VQF_STABLE_IIR
    const vqf_coeffs_t *c=&p->coeffs;
#endif
    for (int j=0; j<3; j++) {
        p->state.bias[j]-=diff[j];
        p->state.restLastGyrLp[j]-=diff[j];
        #if VQF_STABLE_IIR
        p->state.restGyrLpState[2*j+1]-=diff[j];
#else
        p->state.restGyrLpState[2*j]-=diff[j]*(1-c->restGyrLpB[0]);
        p->state.restGyrLpState[2*j+1]-=diff[j]*(c->restGyrLpB[2]-c->restGyrLpA[1]);
#endif
    }
    for (int row=0; row<2; row++) for (int tap=0; tap<2; tap++) {
        double change=0;
        for (int col=0; col<3; col++) {
            change += p->state.motionBiasEstRLpState[2*(3*row+col)+tap]
                      * (double)diff[col];
        }
        p->state.motionBiasEstBiasLpState[2*row+tap]-=change;
    }
}

bool earth_pair_realign_yaw(earth_pair_t *p, float delta_yaw_rad)
{
    if (!pair_can_rotate_yaw(p,delta_yaw_rad,false)) return false;
    /* One atomic assignment to the live filter, from within its owner thread. */
    earth_pair_t t=*p;
    float before_q[4], after_q[4], before_body[3], after_body[3], diff[3];
    getQuat6D(&t.state,before_q);
    earth_pair_project(before_q,t.earth_world,before_body);
    pair_rotate_world_state(&t,delta_yaw_rad);
    getQuat6D(&t.state,after_q);
    earth_pair_project(after_q,t.earth_world,after_body);
    for (int k=0;k<3;k++) diff[k]=after_body[k]-before_body[k];
    /* Maintain continuity of the VQF gyroscope/bias filters under the
     * change of deterministic Earth-spin subtraction. This does not prove
     * the bias is physically correct; validation on recorded IMU data is
     * required before enabling in production. */
    pair_translate_earth_subtraction(&t,diff);
    if (!snapshot_pair_valid(&t)) return false;
    *p=t;
    return true;
}


#define NORTH_MAX_TILT_COS 0.996194698f /* <= 5 deg change in gravity axis */
static void body_to_world(const float q[4], const float v[3], float r[3]) {
    const float w=q[0],x=q[1],y=q[2],z=q[3];
    r[0]=(1-2*(y*y+z*z))*v[0]+2*(x*y-w*z)*v[1]+2*(x*z+w*y)*v[2];
    r[1]=2*(x*y+w*z)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z-w*x)*v[2];
    r[2]=2*(x*z-w*y)*v[0]+2*(y*z+w*x)*v[1]+(1-2*(x*x+y*y))*v[2];
}
static bool quat_valid(const float q[4]) {
    float n=0;
    for(int k=0;k<4;k++) { if(!isfinite(q[k])) return false; n+=q[k]*q[k]; }
    return n>0.98f && n<1.02f;
}
bool north_yaw_capture(north_yaw_ref_t *ref, const earth_pair_t *pair) {
    if(!ref || !pair || !pair->enabled || pair->mode!=PAIR_EARTH ||
       !pair->state.restDetected) return false;
    north_yaw_ref_t candidate={0};
    vqf_state_t s=pair->state;
    getQuat6D(&s,candidate.quat);
    if(!quat_valid(candidate.quat)) return false;
    const float up[3]={0,0,1};
    earth_pair_project(candidate.quat,up,candidate.up_body);
    float best=-1;
    for(uint8_t k=0;k<3;k++) {
        float axis[3]={0},world[3]; axis[k]=1;
        body_to_world(candidate.quat,axis,world);
        const float h=world[0]*world[0]+world[1]*world[1];
        if(h>best){best=h;candidate.horizontal_axis=k;}
    }
    if(best<0.50f) return false;
    candidate.valid=true;
    *ref=candidate;
    return true;
}
bool north_yaw_delta(const north_yaw_ref_t *ref, const earth_pair_t *pair,
                     float *delta_rad) {
    if(!ref || !ref->valid || !pair || !pair->enabled ||
       pair->mode!=PAIR_EARTH || !delta_rad || ref->horizontal_axis>2) return false;
    float q[4]; vqf_state_t s=pair->state; getQuat6D(&s,q);
    if(!quat_valid(q) || !quat_valid(ref->quat))return false;
    const float up[3]={0,0,1}; float up_now[3];
    earth_pair_project(q,up,up_now);
    float dot=0;
    for(int k=0;k<3;k++) dot+=ref->up_body[k]*up_now[k];
    if(dot<NORTH_MAX_TILT_COS) return false;
    float axis[3]={0},current[3],target[3];
    axis[ref->horizontal_axis]=1;
    body_to_world(ref->quat,axis,target);
    body_to_world(q,axis,current);
    if(hypotf(current[0],current[1])<0.50f ||
       hypotf(target[0],target[1])<0.50f) return false;
    *delta_rad=atan2f(current[0]*target[1]-current[1]*target[0],
                     current[0]*target[0]+current[1]*target[1]);
    return isfinite(*delta_rad);
}

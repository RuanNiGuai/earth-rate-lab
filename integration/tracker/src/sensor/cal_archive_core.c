#include "cal_archive_core.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#define EARTH_RATE_DPS 0.0041780746
#define RAD 0.017453292519943295
uint32_t cal_arc_checksum(const void *data, unsigned bytes) {
    const uint8_t *p=(const uint8_t *)data;
    uint32_t crc=0xFFFFFFFFu;
    for(unsigned i=0;i<bytes;i++){
        crc^=p[i];
        for(int bit=0;bit<8;bit++)
            crc=(crc>>1)^((crc&1u)?0xEDB88320u:0u);
    }
    return ~crc;
}
static bool fit(int n,double a[6][7],double *v) {
    for(int i=0;i<n;i++) {
        int p=i;
        for(int j=i+1;j<n;j++)if(fabs(a[j][i])>fabs(a[p][i]))p=j;
        if(fabs(a[p][i])<1.e-8)return false;
        for(int j=i;j<=n;j++){double x=a[i][j];a[i][j]=a[p][j];a[p][j]=x;}
        for(int j=i+1;j<n;j++){double f=a[j][i]/a[i][i];for(int k=i;k<=n;k++)a[j][k]-=f*a[i][k];}
    }
    for(int i=n-1;i>=0;i--){double z=a[i][n];for(int j=i+1;j<n;j++)z-=a[i][j]*v[j];v[i]=z/a[i][i];}
    return true;
}
static void accum(int n, double a[6][7], const double *row, double y){
    for(int i=0;i<n;i++){for(int j=0;j<n;j++)a[i][j]+=row[i]*row[j];a[i][n]+=row[i]*y;}
}
/* q describes a body-to-reference rotation, stored relative to the confirmed
 * startup reference. Reference Earth rotates into sensor axes by q^-1. */
static void earth_rotate(const float q[4],const float e[3],float out[3]){
    float x=q[1],y=q[2],z=q[3],w=q[0];
    float norm=w*w+x*x+y*y+z*z;
    if(norm<.1f||!isfinite(norm)) { memcpy(out,e,3*sizeof(float));return; }
    w/=sqrtf(norm);x/=sqrtf(norm);y/=sqrtf(norm);z/=sqrtf(norm);
    /* q^-1 * e * q */
    out[0]=(1-2*y*y-2*z*z)*e[0]+(2*x*y+2*w*z)*e[1]+(2*x*z-2*w*y)*e[2];
    out[1]=(2*x*y-2*w*z)*e[0]+(1-2*x*x-2*z*z)*e[1]+(2*y*z+2*w*x)*e[2];
    out[2]=(2*x*z+2*w*y)*e[0]+(2*y*z-2*w*x)*e[1]+(1-2*x*x-2*y*y)*e[2];
}
static bool quat_valid(const float q[4]);

/* ARC7: gravity is an observed 3-D direction, not the nearest nominal axis.
 * Quaternions map body to reference. Gyro integration preserves relative yaw;
 * gravity only corrects the observable two inclination degrees of freedom. */
static bool unit3(const float v[3],float n[3]) {
    float norm=sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    if(!isfinite(norm)||norm<.90f||norm>1.10f)return false;
    for(int i=0;i<3;i++)n[i]=v[i]/norm;
    return true;
}
static float dot3(const float a[3],const float b[3]) {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
static void cross3(const float a[3],const float b[3],float c[3]) {
    c[0]=a[1]*b[2]-a[2]*b[1];
    c[1]=a[2]*b[0]-a[0]*b[2];
    c[2]=a[0]*b[1]-a[1]*b[0];
}
static void basis_from_up(const float u[3],float h1[3],float h2[3]) {
    const float guide[3]={fabsf(u[0])<.8f?1.f:0.f, fabsf(u[0])<.8f?0.f:1.f,0.f};
    float component=dot3(guide,u),length=0;
    for(int i=0;i<3;i++){h1[i]=guide[i]-component*u[i];length+=h1[i]*h1[i];}
    length=sqrtf(length);
    for(int i=0;i<3;i++)h1[i]/=length;
    cross3(u,h1,h2);
}
static bool gravity_lock(float q[4],const float observed_accel[3],
                         const float reference_up[3],float *mismatch_deg) {
    float measured[3],predicted[3];
    if(!unit3(observed_accel,measured)||!quat_valid(q))return false;
    earth_rotate(q,reference_up,predicted);
    float cos_angle=dot3(predicted,measured);
    cos_angle=fmaxf(-1.f,fminf(1.f,cos_angle));
    float mismatch=acosf(cos_angle)/(float)RAD;
    if(mismatch_deg)*mismatch_deg=mismatch;
    /* A severely inconsistent gyro/accelerometer path is not a small
     * leveling error. Do not silently rotate away an incorrect yaw path. */
    if(!isfinite(mismatch)||mismatch>18.f)return false;
    float c[3];cross3(measured,predicted,c);
    float tilt[4]={1.f+cos_angle,c[0],c[1],c[2]};
    float len=sqrtf(dot3(c,c)+tilt[0]*tilt[0]);
    if(len<1.e-6f)return false;
    for(int i=0;i<4;i++)tilt[i]/=len;
    float result[4]={
      q[0]*tilt[0]-q[1]*tilt[1]-q[2]*tilt[2]-q[3]*tilt[3],
      q[0]*tilt[1]+q[1]*tilt[0]+q[2]*tilt[3]-q[3]*tilt[2],
      q[0]*tilt[2]-q[1]*tilt[3]+q[2]*tilt[0]+q[3]*tilt[1],
      q[0]*tilt[3]+q[1]*tilt[2]-q[2]*tilt[1]+q[3]*tilt[0]
    };
    memcpy(q,result,sizeof(result));
    return quat_valid(q);
}
/* Test hook: align measured gravity, without claiming to observe yaw. */
bool cal_arc_gravity_lock(float q[4],const float accel[3],
                          const float up[3],float *mismatch_deg) {
    return gravity_lock(q,accel,up,mismatch_deg);
}
/* Small independent 8x8 least-squares normal system. Fit is not
 * called in the real-time sensor thread. */
static bool fit8(double a[8][9],double v[8]) {
    for(int i=0;i<8;i++) {
        int pivot=i;
        for(int j=i+1;j<8;j++)if(fabs(a[j][i])>fabs(a[pivot][i]))pivot=j;
        if(fabs(a[pivot][i])<1.e-10)return false;
        for(int j=i;j<=8;j++){double tmp=a[i][j];a[i][j]=a[pivot][j];a[pivot][j]=tmp;}
        for(int j=i+1;j<8;j++){
            double ratio=a[j][i]/a[i][i];
            for(int k=i;k<=8;k++)a[j][k]-=ratio*a[i][k];
        }
    }
    for(int i=7;i>=0;i--){
        double x=a[i][8];for(int j=i+1;j<8;j++)x-=a[i][j]*v[j];
        v[i]=x/a[i][i];if(!isfinite(v[i]))return false;
    }
    return true;
}
static void accum8(double a[8][9],const double r[8],double y) {
    for(int i=0;i<8;i++){
        for(int j=0;j<8;j++)a[i][j]+=r[i]*r[j];
        a[i][8]+=r[i]*y;
    }
}

static bool bias_at(const cal_arc_tcal_t *,float,const float[3],const float[3],const float[9],float[3]);
/* Compress time-ordered FIFO angular increments into at most eight small
 * subintegrals per transition. When full, merge the least consequential
 * neighboring pair. This is explicitly lossy: not an exact coning replay. */
void cal_arc_motion_add(cal_arc_motion_t *m,const float raw[3],
                        const float a[3],float T,float dt){
    if(!m||!isfinite(T)||!isfinite(dt)||dt<=0||dt>.1f)return;
    for(int j=0;j<3;j++)if(!isfinite(raw[j])||!isfinite(a[j]))return;
    if(!m->count)m->count=1;
    int n=m->count;
    if(m->seg[n-1].seconds>=.25f){
        if(n==CAL_ARC_MSEG){
            int best=0;
            float score=INFINITY;
            for(int k=0;k<n-1;k++){
                const cal_arc_motion_seg_t *x=&m->seg[k],*y=&m->seg[k+1];
                float cross[3]={x->raw_delta[1]*y->raw_delta[2]-x->raw_delta[2]*y->raw_delta[1],
                    x->raw_delta[2]*y->raw_delta[0]-x->raw_delta[0]*y->raw_delta[2],
                    x->raw_delta[0]*y->raw_delta[1]-x->raw_delta[1]*y->raw_delta[0]};
                float kcost=x->seconds+y->seconds+.005f*sqrtf(cross[0]*cross[0]+cross[1]*cross[1]+cross[2]*cross[2]);
                if(kcost<score){score=kcost;best=k;}
            }
            cal_arc_motion_seg_t *x=&m->seg[best],*y=&m->seg[best+1];
            float cx=x->raw_delta[1]*y->raw_delta[2]-x->raw_delta[2]*y->raw_delta[1];
            float cy=x->raw_delta[2]*y->raw_delta[0]-x->raw_delta[0]*y->raw_delta[2];
            float cz=x->raw_delta[0]*y->raw_delta[1]-x->raw_delta[1]*y->raw_delta[0];
            m->lost_cross_deg+=.5f*(float)RAD*sqrtf(cx*cx+cy*cy+cz*cz);
            for(int j=0;j<3;j++){x->raw_delta[j]+=y->raw_delta[j];x->accel_dt[j]+=y->accel_dt[j];}
            x->temp_dt+=y->temp_dt;x->seconds+=y->seconds;
            for(int k=best+1;k<n-1;k++)m->seg[k]=m->seg[k+1];
            memset(&m->seg[n-1],0,sizeof(m->seg[n-1]));
            m->count--;if(m->merges<255)m->merges++;
        }
        m->count++;
    }
    cal_arc_motion_seg_t *b=&m->seg[m->count-1];
    for(int j=0;j<3;j++){b->raw_delta[j]+=raw[j]*dt;b->accel_dt[j]+=a[j]*dt;}
    b->temp_dt+=T*dt;b->seconds+=dt;
    if(m->samples<UINT16_MAX)m->samples++;
}
static bool quat_valid(const float q[4]){
    float n=0;for(int i=0;i<4;i++){if(!isfinite(q[i]))return false;n+=q[i]*q[i];}
    return n>.9f&&n<1.1f;
}
static void quat_apply(float q[4],const float d[3]){
    float n=sqrtf(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
    float h=.5f*n*(float)RAD;
    float f=n>1.e-7f?sinf(h)/n:.5f*(float)RAD;
    float p[4]={cosf(h),d[0]*f,d[1]*f,d[2]*f};
    float v[4]={q[0]*p[0]-q[1]*p[1]-q[2]*p[2]-q[3]*p[3],
      q[0]*p[1]+q[1]*p[0]+q[2]*p[3]-q[3]*p[2],
      q[0]*p[2]-q[1]*p[3]+q[2]*p[0]+q[3]*p[1],
      q[0]*p[3]+q[1]*p[2]-q[2]*p[1]+q[3]*p[0]};
    float t=0;for(int j=0;j<4;j++)t+=v[j]*v[j];
    if(t>.0001f){t=1/sqrtf(t);for(int j=0;j<4;j++)q[j]=v[j]*t;}
}
bool cal_arc_motion_replay(const cal_arc_motion_t *m,const float initial_q[4],
       const float S[3],const float M[9],const float E[3],const float b[3],float out[4]){
    if(!m||!quat_valid(initial_q)||m->count<1||m->count>CAL_ARC_MSEG||!m->samples||
       !isfinite(m->lost_cross_deg)||m->lost_cross_deg>.75f)return false;
    memcpy(out,initial_q,4*sizeof(float));
    for(int k=0;k<m->count;k++){
        const cal_arc_motion_seg_t *p=&m->seg[k];
        if(!isfinite(p->seconds)||p->seconds<=0)return false;
        float e[3];earth_rotate(out,E,e);
        float d[3];
        for(int i=0;i<3;i++){
            if(!isfinite(S[i])||S[i]<.8f||S[i]>1.2f)return false;
            d[i]=p->raw_delta[i]-p->seconds*b[i];
            for(int j=0;j<3;j++)d[i]-=M[i*3+j]*p->accel_dt[j];
            d[i]=S[i]*d[i]-p->seconds*e[i];
        }
        quat_apply(out,d);
    }
    return quat_valid(out);
}
/* Interpolate a temperature-only physical gyro bias from the archived
 * reference points, avoiding the dependency on their original VQF state. */
static bool replay_pose_chain(const cal_arc_tcal_t *t,
        const cal_arc_motion_t *travel,const cal_arc_point_t *poses,int n,
        const float anchor[4],const float S[3],const float M[9],
        const float E[3],float quat[][4]){
    float ref_up[3];
    if(!unit3(poses[0].accel,ref_up))return false;
    if(!quat_valid(anchor))return false;
    memcpy(quat[0],anchor,16);
    for(int k=1;k<n;k++){
        const cal_arc_motion_t *move=&travel[k-1];
        if(!move->count||!move->samples||!isfinite(move->lost_cross_deg)||
           move->lost_cross_deg>.75f)return false;
        float q[4];memcpy(q,quat[k-1],sizeof(q));
        for(int j=0;j<move->count;j++){
            /* Each saved time-bin has its own temperature. Do not use one
             * average bias for a possibly warming/cooling pose transition. */
            const cal_arc_motion_seg_t *seg=&move->seg[j];
            if(seg->seconds<=0||!isfinite(seg->seconds))return false;
            float bias[3];
            if(!bias_at(t,seg->temp_dt/seg->seconds,S,E,M,bias))return false;
            cal_arc_motion_t one={0};one.count=1;one.samples=1;one.seg[0]=*seg;
            float next[4];
            if(!cal_arc_motion_replay(&one,q,S,M,E,bias,next))return false;
            memcpy(q,next,sizeof(q));
        }
        /* Gravity observation removes inclination drift but leaves yaw from
         * gyro integration. It also supports a mildly tilted turntable. */
        if(!gravity_lock(q,poses[k].accel,ref_up,NULL))return false;
        memcpy(quat[k],q,sizeof(q));
    }
    return true;
}
static bool bias_at(const cal_arc_tcal_t *t,float T,const float scale[3],
                    const float earth[3],const float M[9],float b[3]) {
    if(t->count<1 || t->count>CAL_ARC_TMAX)return false;
    int lo=-1,hi=-1;
    for(unsigned k=0;k<t->count;k++){
        if(!isfinite(t->point[k].temp))return false;
        if(t->point[k].temp<=T && (lo<0 || t->point[k].temp>t->point[lo].temp))lo=k;
        if(t->point[k].temp>=T && (hi<0 || t->point[k].temp<t->point[hi].temp))hi=k;
    }
    if(lo<0)lo=hi;
    if(hi<0)hi=lo;
    if(lo<0||hi<0)return false;
    float w=(lo!=hi)?(T-t->point[lo].temp)/(t->point[hi].temp-t->point[lo].temp):0;
    float bl[3],bh[3];
    const cal_arc_point_t *p[2]={&t->point[lo],&t->point[hi]};
    for(int k=0;k<2;k++){
        float ep[3];earth_rotate(p[k]->quat,earth,ep);
        float *d=k?bh:bl;
        for(int i=0;i<3;i++){
            if(scale[i]<.8f)return false;
            d[i]=p[k]->gyro[i]-ep[i]/scale[i];
            for(int j=0;j<3;j++)d[i]-=M[3*i+j]*p[k]->accel[j];
        }
    }
    for(int j=0;j<3;j++)b[j]=bl[j]+w*(bh[j]-bl[j]);
    return true;
}
static bool earth_fit(const cal_arc_tcal_t *t,const cal_arc_earth_t *e,
                      const float scale[3],const float M[9],const float E_old[3],
                      float E_new[3],float *rms){
    /* Fit all three axes at all five stops. Earth vertical is the actual
     * measured reference gravity, not the nominal +X/+Y/+Z classification.
     * Unknowns: 3 constant offsets, 3 linear drifts, 2 horizontal Earth
     * components. Known: latitude and full three-dimensional pose chain. */
    float up[3],h1[3],h2[3];
    if(!unit3(e->stop[0].accel,up))return false;
    if(!isfinite(e->latitude_deg)||fabsf(e->latitude_deg)>85.f)return false;
    basis_from_up(up,h1,h2);
    float qchain[CAL_ARC_EARTH][4];
    const float identity[4]={1.f,0.f,0.f,0.f};
    if(!replay_pose_chain(t,e->travel,e->stop,CAL_ARC_EARTH,identity,
                          scale,M,E_old,qchain))return false;
    double A[8][9]={{0}},v[8]={0};
    float ycache[CAL_ARC_EARTH][3];
    float t0=e->stop[0].seconds;
    double vertical=EARTH_RATE_DPS*sin(e->latitude_deg*RAD);
    double horizontal=EARTH_RATE_DPS*cos(e->latitude_deg*RAD);
    for(int k=0;k<CAL_ARC_EARTH;k++){
        const cal_arc_point_t *p=&e->stop[k];
        if(p->samples<100||!isfinite(p->seconds)||!isfinite(p->temp))return false;
        float b[3],U[3],H1[3],H2[3];
        if(!bias_at(t,p->temp,scale,E_old,M,b))return false;
        earth_rotate(qchain[k],up,U);
        earth_rotate(qchain[k],h1,H1);
        earth_rotate(qchain[k],h2,H2);
        float time=(p->seconds-t0)/100.f;
        if(!isfinite(time))return false;
        for(int ax=0;ax<3;ax++){
            double y=(p->gyro[ax]-b[ax])*scale[ax]-vertical*U[ax];
            for(int j=0;j<3;j++)y-=(double)scale[ax]*M[3*ax+j]*p->accel[j];
            double row[8]={0};row[ax]=1;row[ax+3]=time;
            row[6]=H1[ax];row[7]=H2[ax];
            accum8(A,row,y);
            ycache[k][ax]=(float)y;
        }
    }
    if(!fit8(A,v))return false;
    double rss=0;
    for(int k=0;k<CAL_ARC_EARTH;k++){
        float H1[3],H2[3];
        earth_rotate(qchain[k],h1,H1);earth_rotate(qchain[k],h2,H2);
        double time=(e->stop[k].seconds-t0)/100.f;
        for(int ax=0;ax<3;ax++){
            double pred=v[ax]+v[ax+3]*time+v[6]*H1[ax]+v[7]*H2[ax];
            double r=ycache[k][ax]-pred;rss+=r*r;
        }
    }
    *rms=(float)sqrt(rss/15.);
    double magnitude=hypot(v[6],v[7]);
    if(!isfinite(magnitude)||magnitude<horizontal*.50||magnitude>horizontal*1.5||
       !isfinite(*rms)||*rms>.001f)return false;
    for(int i=0;i<3;i++){
        E_new[i]=(float)(vertical*up[i]+horizontal*(v[6]*h1[i]+v[7]*h2[i])/magnitude);
    }
    return true;
}
static bool gsens_fit(const cal_arc_tcal_t *t,const cal_arc_gsens_t *g,
                      const float scale[3],const float E[3],const float oldM[9],
                      float nextM[9],float *rms){
    /* Each pose must carry a valid quaternion relative to the same reference;
     * a gravity vector on its own cannot separate the horizontal Earth term. */
    double total=0;
    float qchain[CAL_ARC_GSENS][4];
    const float identity[4]={1,0,0,0};
    if(!replay_pose_chain(t,g->travel,g->pose,CAL_ARC_GSENS,identity,scale,oldM,E,qchain))return false;
    for(int ax=0;ax<3;ax++){
        double A[6][7]={{0}},v[6]={0};
        for(int k=0;k<6;k++){
            const cal_arc_point_t *p=&g->pose[k];
            if(p->samples<100)return false;
            if(!quat_valid(qchain[k]))return false;
            float b[3],Ebody[3];
            if(!bias_at(t,p->temp,scale,E,oldM,b))return false;
            earth_rotate(qchain[k],E,Ebody);
            /* Add old fitted M back into the residual before full fit. */
            double y=p->gyro[ax]-b[ax]-Ebody[ax]/scale[ax];
            double row[6]={p->accel[0],p->accel[1],p->accel[2],1,0,0};
            accum(4,A,row,y);
        }
        if(!fit(4,A,v))return false;
        for(int j=0;j<3;j++){
            if(!isfinite(v[j])||fabs(v[j])>.05)return false;
            nextM[3*ax+j]=(float)v[j];
        }
        for(int k=0;k<6;k++){
            const cal_arc_point_t *p=&g->pose[k];
            float b[3],Ebody[3];
            if(!bias_at(t,p->temp,scale,E,oldM,b))return false;
            earth_rotate(qchain[k],E,Ebody);
            double pred=v[3];for(int j=0;j<3;j++)pred+=v[j]*p->accel[j];
            double r=p->gyro[ax]-b[ax]-Ebody[ax]/scale[ax]-pred;
            total+=r*r;
        }
    }
    *rms=(float)sqrt(total/18);
    return *rms<=.006;
}
bool cal_arc_solve(const cal_arc_tcal_t *t,const cal_arc_scale_t *s,
                   const cal_arc_earth_t *e,const cal_arc_gsens_t *g,
                   cal_arc_solution_t *out){
    if(!t||!s||!e||!g||!out)return false;
    memset(out,0,sizeof(*out));
    if(t->count<1||t->count>CAL_ARC_TMAX||s->count!=3||e->count!=5||g->count!=6)return false;
    uint32_t ref=t->reference_id;
    if(!ref||ref!=s->reference_id||ref!=e->reference_id||ref!=g->reference_id)return false;
    if(e->up_axis>5||!isfinite(e->latitude_deg)||fabsf(e->latitude_deg)>85)return false;
    float M[9]={0},E[3]={0},S[3]={1,1,1};
    float up[3],h1[3],h2[3];
    if(!unit3(e->stop[0].accel,up))return false;
    basis_from_up(up,h1,h2);
    for(int k=0;k<3;k++)E[k]=(float)(EARTH_RATE_DPS*(sin(e->latitude_deg*RAD)*up[k]+
                                                     cos(e->latitude_deg*RAD)*h1[k]));
    for(int iter=0;iter<5;iter++){
        /* First solve scales against each axis' raw static seed. Temperature
         * correction is temperature-average approximation; linear M*a term
         * is integrated exactly from stored accel_time. */
        for(int j=0;j<3;j++){
            const cal_arc_scale_axis_t *r=&s->axis[j];
            if(r->axis!=j||r->turns!=20||r->duration<10||r->duration>600||r->cross_angle>1100.f||r->quality>.16f)return false;
            float start_b[3],mean_b[3];
            if(!bias_at(t,r->initial_temp,S,E,M,start_b) ||
               !bias_at(t,r->temp_time/r->duration,S,E,M,mean_b))return false;
            double delta[3];
            for(int ax=0;ax<3;ax++){
                delta[ax]=r->raw_angle[ax]-(double)r->initial_raw[ax]*r->duration;
                /* temperature drift difference from fixed start seed */
                delta[ax]-=(double)(mean_b[ax]-start_b[ax])*r->duration;
                for(int c=0;c<3;c++)
                    delta[ax]-=(double)M[3*ax+c]*
                      (r->accel_time[c]-r->initial_accel[c]*r->duration);
                if(!isfinite(delta[ax]))return false;
            }
            /* The turntable axis need not coincide exactly with the nominal
             * gyro axis. For a fixed physical axis, the calibrated angle
             * VECTOR has magnitude 7200 degrees, not its j component.
             * Solve S_j with the other two calibrated components present. */
            double other=0;
            for(int ax=0;ax<3;ax++)if(ax!=j)
                other+=(double)S[ax]*delta[ax]*S[ax]*delta[ax];
            if(other>7200.*7200.*.10 || fabs(delta[j])<6000 ||
               fabs(delta[j])>8000)return false;
            double required=sqrt(7200.*7200.-other);
            float candidate=(float)(required/fabs(delta[j]));
            if(candidate<.90f||candidate>1.10f)return false;
            S[j]=candidate;
            out->scale_error[j]=(float)(hypot(required,sqrt(other))-7200.);
        }
        float En[3];
        if(!earth_fit(t,e,S,M,E,En,&out->earth_rms))return false;
        memcpy(E,En,sizeof(E));
        float Mn[9];
        if(!gsens_fit(t,g,S,E,M,Mn,&out->gsens_rms))return false;
        memcpy(M,Mn,sizeof(M));
        out->iterations=(uint8_t)(iter+1);
    }
    float b[3];if(!bias_at(t,t->point[0].temp,S,E,M,b))return false;
    memcpy(out->scale,S,sizeof(S));memcpy(out->earth_body,E,sizeof(E));
    memcpy(out->gsens,M,sizeof(M));memcpy(out->base_bias,b,sizeof(b));
    out->ready=1;
    return true;
}

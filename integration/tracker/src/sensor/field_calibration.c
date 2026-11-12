#include "field_calibration.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/* Adaptive per-stop sampling: stop as soon as the standard error of each axis
 * mean is below the target, with hard lower/upper duration bounds. */
#define FIELD_EARTH_SAMPLE_MIN_S 5.0f
#define FIELD_EARTH_SAMPLE_MAX_S 60.0f
#define FIELD_EARTH_SEM_TARGET 1.0e-4f
#define FIELD_EARTH_MIN_SAMPLES 400

static uint32_t reference_checksum(const field_reference_t *r)
{
    const unsigned char *p = (const unsigned char *)r;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < offsetof(field_reference_t, checksum); ++i) h = (h ^ p[i]) * 16777619u;
    return h;
}

bool field_reference_make(field_reference_t *r, const float v[3])
{
    double n = 0;
    for (int i=0;i<3;i++) { if (!isfinite(v[i])) return false; n += (double)v[i]*(double)v[i]; }
    if (fabs(sqrt(n)-(double)FIELD_EARTH_RATE_DPS) > (double)FIELD_EARTH_RATE_DPS*0.02) return false;
    memset(r,0,sizeof(*r)); r->version=1;
    memcpy(r->body,v,sizeof(r->body)); r->checksum=reference_checksum(r);
    return true;
}

bool field_reference_valid(const field_reference_t *r)
{
    field_reference_t checked;
    return r->version==1 && r->checksum==reference_checksum(r) && field_reference_make(&checked,r->body);
}

float field_reference_latitude(const float body[3])
{
    field_reference_t checked;
    if (!field_reference_make(&checked,body)) return NAN;
    return (float)(atan2((double)body[0],hypot((double)body[1],(double)body[2]))*57.29577951308232);
}

bool field_reference_set_latitude(float body[3], float latitude_deg)
{
    field_reference_t checked;
    if (!isfinite(latitude_deg) || fabsf(latitude_deg)>90 || !field_reference_make(&checked,body)) return false;
    double h=hypot((double)body[1],(double)body[2]);
    /* A purely vertical imported reference has no horizontal heading to preserve. */
    if (h==0 && fabsf(latitude_deg)<90) return false;
    double lat=(double)latitude_deg*0.017453292519943295;
    double horizontal=(double)FIELD_EARTH_RATE_DPS*cos(lat);
    float updated[3]={(float)((double)FIELD_EARTH_RATE_DPS*sin(lat)),
        h>0 ? (float)((double)body[1]*horizontal/h) : 0,
        h>0 ? (float)((double)body[2]*horizontal/h) : 0};
    memcpy(body,updated,sizeof(updated));
    return true;
}

void field_temperature_manual(field_temperature_t *s, bool enabled)
{
    s->collecting=enabled; s->inhibited=!enabled; s->stable_since=-1;
    if (enabled) s->cold_since=-1;
}

#define FIELD_TEMP_STABLE_S 120000        /* dwell time below 34 C */
#define FIELD_TEMP_SPAN_MIN 0.10f          /* span floor, deg C (filtered noise ~0.02) */
#define FIELD_TEMP_SPAN_MAX 0.90f          /* span ceiling, deg C */
#define FIELD_TEMP_COLD_MAX 1800000        /* 30 min fallback below 34 C */

int field_temperature_step(field_temperature_t *s, float t, int64_t now)
{
    if (!isfinite(t)) { s->stable_since=-1; s->cold_since=-1; s->sampled=false; return 0; }
    bool gap = s->sampled && (now<s->last_sample || now-s->last_sample>5000);
    if (gap) { s->stable_since=-1; s->cold_since=-1; }
    if (s->sampled && !gap) {
        /* Track the instantaneous noise so the span test follows the sensor
         * rather than a fixed 0.1 C the raw signal can never meet. */
        /* Low-pass the temperature (~10 s) so the stability span is not
         * dominated by the sensor's sample noise; a raw span never settles. */
        float dt_s = (float)(now - s->last_sample) / 1000.0f;
        if (dt_s <= 0.0f || dt_s > 5.0f) dt_s = 0.1f;
        float a = dt_s / (10.0f + dt_s);
        float prev_lp = s->temp_lp;
        /* A large jump below 34 C means a new thermal regime (moved to a cold
         * room), not noise: re-seed instead of waiting for the filter. */
        if (t < 33.5f && fabsf(t - s->temp_lp) > 2.0f) s->temp_lp = t;
        else s->temp_lp += a * (t - s->temp_lp);
        /* Noise estimate follows the filtered signal, so the adaptive span
         * tracks real temperature wander instead of raw sensor jitter. */
        s->noise = 0.95f*s->noise + 0.05f*fabsf(s->temp_lp - prev_lp);
    } else {
        s->temp_lp = t;
    }
    s->last_temp=t;
    s->last_sample=now; s->sampled=true;
    /* Manual cancel is latched until a new heat cycle below 42 -> above 43 C. */
    if (!s->collecting) {
        if (t < 32.5f) s->inhibited=false;
        if (!s->inhibited && t>=33.5f) {
            s->collecting=true; s->stable_since=-1; s->cold_since=-1; return 1;
        }
        return 0;
    }
    /* The >34 C gate uses the raw reading so a fast cool-down is not delayed
     * by the low-pass; only the stability span uses the filtered value. */
    float tf = s->temp_lp;
    if (t>33.5f) { s->stable_since=-1; s->cold_since=-1; return 0; }
    if (s->cold_since<0) s->cold_since=now;
    float span = 3.0f*s->noise;
    if (span < FIELD_TEMP_SPAN_MIN) span = FIELD_TEMP_SPAN_MIN;
    if (span > FIELD_TEMP_SPAN_MAX) span = FIELD_TEMP_SPAN_MAX;
    if (s->stable_since<0) { s->stable_since=now; s->low=s->high=tf; }
    if (tf<s->low) s->low=tf;
    if (tf>s->high) s->high=tf;
    if (s->high-s->low>span) { s->stable_since=now; s->low=s->high=tf; }
    if (now-s->stable_since>=FIELD_TEMP_STABLE_S ||
        now-s->cold_since>=FIELD_TEMP_COLD_MAX) {
        s->collecting=false; s->inhibited=true; s->stable_since=-1; s->cold_since=-1; return -1;
    }
    return 0;
}

static void fail(field_earth_t *s, const char *why) { s->phase=FIELD_FAILED; s->error=why; }
void field_earth_start(field_earth_t *s, float latitude_deg, int up_axis)
{
    memset(s,0,sizeof(*s)); s->phase=FIELD_SETTLE;
    s->up_axis = (up_axis >= 0 && up_axis <= 5) ? up_axis : 0;
    if (!isfinite(latitude_deg) || fabsf(latitude_deg)>90) { fail(s,"invalid site latitude"); return; }
    double lat=(double)latitude_deg*0.017453292519943295;
    s->model_up=(float)((double)FIELD_EARTH_RATE_DPS*sin(lat));
    s->model_horizontal=(float)((double)FIELD_EARTH_RATE_DPS*cos(lat));
    if (s->model_horizontal<0.0002f) fail(s,"horizontal Earth signal too small near the poles; use a known reference");
}
void field_earth_cancel(field_earth_t *s) { memset(s,0,sizeof(*s)); }
static const float angles[5]={0,90,180,90,0};

static void reset_sample(field_earth_t *s)
{
    s->count=0; s->duration=0; s->settled=0;
    memset(s->sum,0,sizeof(s->sum)); memset(s->sumsq,0,sizeof(s->sumsq));
}

/* Fit separate Y/Z constant and linear time drift, plus two rotating components. */
static void finish(field_earth_t *s)
{
    double m[6][7]={{0}}, x[6]={0};
    int uidx=s->up_axis/2;
    int pidx=(uidx+1)%3, qidx=(uidx+2)%3;
    float usg=(s->up_axis&1)?-1.0f:1.0f;
    for (int k=0;k<5;k++) {
        double t=(double)(s->times[k]-s->times[0])/100.0;
        double a=(double)s->spin*(double)angles[k]*0.017453292519943295;
        double c=cos(a), sn=sin(a);
        /* Rotation about the up axis: body Earth = R_u(-angle) * E. */
        double yk=(double)s->means[k][pidx];
        double zk=(double)s->means[k][qidx]*usg;
        double rows[2][6]={{1,0,t,0,c,sn},{0,1,0,t,-sn,c}};
        double vals[2]={yk,zk};
        for (int r=0;r<2;r++) for (int i=0;i<6;i++) {
            for(int j=0;j<6;j++) m[i][j]+=rows[r][i]*rows[r][j];
            m[i][6]+=rows[r][i]*vals[r];
        }
    }
    for (int i=0;i<6;i++) {
        int p=i;
        for(int r=i+1;r<6;r++) if(fabs(m[r][i])>fabs(m[p][i])) p=r;
        if(fabs(m[p][i])<1e-8) { fail(s,"degenerate fit"); return; }
        for(int j=i;j<=6;j++) { double tmp=m[p][j]; m[p][j]=m[i][j]; m[i][j]=tmp; }
        for(int r=i+1;r<6;r++) {
            double f=m[r][i]/m[i][i];
            for(int j=i;j<=6;j++) m[r][j]-=f*m[i][j];
        }
    }
    for(int i=5;i>=0;i--) { double v=m[i][6]; for(int j=i+1;j<6;j++) v-=m[i][j]*x[j]; x[i]=v/m[i][i]; }
    double rss=0;
    for(int k=0;k<5;k++) {
        double t=(double)(s->times[k]-s->times[0])/100.0, a=(double)s->spin*(double)angles[k]*0.017453292519943295;
        double yk=(double)s->means[k][pidx];
        double zk=(double)s->means[k][qidx]*usg;
        double dy=yk-(x[0]+t*x[2]+cos(a)*x[4]+sin(a)*x[5]);
        double dz=zk-(x[1]+t*x[3]-sin(a)*x[4]+cos(a)*x[5]);
        rss+=dy*dy+dz*dz;
    }
    s->rms=(float)sqrt(rss/10);
    double horizontal=hypot(x[4],x[5]);
    double expected=(double)s->model_horizontal;
    if(!isfinite(horizontal) || horizontal<expected*0.75 || horizontal>expected*1.25) { fail(s,"Earth amplitude outside +/-25% of site model"); return; }
    if(!isfinite(s->rms) || s->rms>0.0006f) { printk("Earth cal: fit rms=%.6f limit=0.000600\n",(double)s->rms); fail(s,"fit residual too large"); return; }
    if(hypot(x[2],x[3])>0.002) { fail(s,"bias drift too large"); return; }
    /* Estimate direction, use known physical magnitude only AFTER acceptance. */
    for (int i=0;i<3;i++) s->result[i]=0.0f;
    s->result[uidx]=s->model_up*usg;
    s->result[pidx]=(float)(x[4]*expected/horizontal);
    s->result[qidx]=(float)(x[5]*expected/horizontal*usg);
    s->phase=FIELD_DONE;
}

void field_earth_step(field_earth_t *s, const float g[3], const float a[3], float temp, float dt, bool rest)
{
    if(s->phase==FIELD_IDLE || s->phase==FIELD_DONE || s->phase==FIELD_FAILED) return;
    {
        /* Periodic diagnostic so a stuck stop is visible in the log. */
        static float diag_acc;
        static enum field_earth_phase diag_phase = FIELD_IDLE;
        static int diag_stop = -1;
        diag_acc += dt;
        if (diag_acc >= 2.0f) {
            diag_acc = 0.0f;
            float norm = sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
            int uidx = s->up_axis/2;
            float usg = (s->up_axis&1) ? -1.0f : 1.0f;
            float tilt = (norm > 0.1f) ? acosf(fminf(1.0f, (a[uidx]*usg)/norm)) * 57.2958f : -1.0f;
            float spd = sqrtf(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
            float arange = 0.0f;
            for (int i = 0; i < 3; i++) {
                float r = s->accel_high[i] - s->accel_low[i];
                if (r > arange) arange = r;
            }
            printk("Earth cal diag: phase=%d stop=%d tilt10=%d rest=%d settled10=%d dur10=%d spd100=%d arange1000=%d\n",
                   (int)s->phase, s->stop, (int)(tilt * 10.0f), rest ? 1 : 0,
                   (int)(s->settled * 10.0f), (int)(s->duration * 10.0f),
                   (int)(spd * 100.0f), (int)(arange * 1000.0f));
            diag_phase = s->phase;
            diag_stop = s->stop;
        }
    }
    if(!isfinite(dt) || dt<=0 || dt>0.1f || !isfinite(temp)) { fail(s,"missing/invalid samples"); return; }
    for(int i=0;i<3;i++) if(!isfinite(g[i]) || !isfinite(a[i])) { fail(s,"nonfinite sensor input"); return; }
    s->elapsed+=dt;
    if(s->elapsed>900) { fail(s,"calibration timeout"); return; }
    if(s->elapsed==dt) s->first_temp=temp;
    if(fabsf(temp-s->first_temp)>0.3f) { fail(s,"temperature changed more than 0.3 C"); return; }
    int uidx=s->up_axis/2;
    float usg=(s->up_axis&1)?-1.0f:1.0f;
    float norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    bool flat=fabsf(norm-1)<0.03f && (a[uidx]*usg)/norm>0.99939f; /* 2 degrees, up axis */
    if(s->stop>0) s->angle+=(g[uidx]*usg-s->turn_bias)*dt;
    if(s->phase==FIELD_TURN) {
        s->turn_idle += dt;
        if(s->turn_idle > 120.0f) { fail(s,"turn timeout"); return; }
        if(s->spin==0 && fabsf(s->angle)>30) s->spin=s->angle>0 ? 1 : -1;
        if(s->spin && fabsf(s->angle-s->spin*angles[s->stop])<3 && rest && flat) {
            s->turn_idle = 0.0f;
            reset_sample(s); s->phase=FIELD_SETTLE;
        }
        return;
    }
    s->turn_idle = 0.0f;
    if(s->stop>0 && fabsf(s->angle-s->spin*angles[s->stop])>3) {
        reset_sample(s); s->phase=FIELD_TURN; return;
    }
    float speed=sqrtf(g[0]*g[0]+g[1]*g[1]+g[2]*g[2]);
    /* During a sample, tolerate short transients (cable tug, table bump):
     * skip the affected samples and only restart the stop if the disturbance
     * lasts. The mean is still built from clean, still samples. */
    if(!flat || !rest || speed>0.30f) {
        s->rejects += dt;
        if(s->rejects > 1.5f || s->phase==FIELD_SETTLE) {
            reset_sample(s);
            s->settled = 0.0f;
            s->phase = FIELD_SETTLE;
        }
        return;
    }
    s->rejects = 0.0f;
    if(s->phase==FIELD_SETTLE) {
        s->settled+=dt;
        if(s->settled<5) return;
        s->phase=FIELD_SAMPLE; s->temp_low=s->temp_high=temp;
        for(int i=0;i<3;i++) s->accel_low[i]=s->accel_high[i]=a[i];
    }
    if(temp<s->temp_low) s->temp_low=temp;
    if(temp>s->temp_high) s->temp_high=temp;
    if(s->temp_high-s->temp_low>0.1f) { fail(s,"temperature unstable at stop"); return; }
    for(int i=0;i<3;i++) {
        if(a[i]<s->accel_low[i]) s->accel_low[i]=a[i];
        if(a[i]>s->accel_high[i]) s->accel_high[i]=a[i];
    }
    s->duration+=dt; s->count++;
    for(int i=0;i<3;i++) { s->sum[i]+=(double)g[i]; s->sumsq[i]+=(double)g[i]*(double)g[i]; }
    if(s->duration<FIELD_EARTH_SAMPLE_MIN_S || s->count<FIELD_EARTH_MIN_SAMPLES) return;
    if(s->duration<FIELD_EARTH_SAMPLE_MAX_S) {
        bool precise=true;
        for(int i=0;i<3;i++) {
            double mean=s->sum[i]/s->count;
            double var=s->sumsq[i]/s->count-mean*mean;
            if(var>0 && sqrt(var/s->count)>FIELD_EARTH_SEM_TARGET) precise=false;
        }
        if(!precise) return;
    }
    for(int i=0;i<3;i++) {
        double mean=s->sum[i]/s->count;
        if(s->sumsq[i]/s->count-mean*mean>0.0064) { fail(s,"gyro variance too large"); return; }
        s->means[s->stop][i]=(float)mean;
    }
    s->times[s->stop]=s->elapsed-s->duration/2;
    if(s->stop==0) s->turn_bias=s->means[0][s->up_axis/2]*((s->up_axis&1)?-1.0f:1.0f);
    if(++s->stop==5) finish(s);
    else { reset_sample(s); s->phase=FIELD_TURN; }
}

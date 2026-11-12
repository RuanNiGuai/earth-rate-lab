/* ARC7 second-stage capture. This file is included ONLY from cal_archive_safe.c.
 * All NVS operations / candidate solving occur on the lazy ARC worker.
 * All high-rate accumulation and state transitions occur on the sensor thread.
 */
#define ARC2_MAGIC 0x41353252u
#define ARC2_VERSION 2u /* v7 motion segment count changed; never interpret v6 records as v7 */
#define ARC2_SCALE_BASE 104u
#define ARC2_EARTH_BASE 110u
#define ARC2_GSENS_BASE 120u
#define ARC2_NRECORDS 14u
#define ARC2_QUALITY_STATIC 6000u    /* ~19.5s at 409.6Hz */
#define ARC2_QUALITY_GSENS 8000u   /* ~29s, one pose */
#define ARC2_QUALITY_SEED 800u      /* ~2s, scale initial rest */
#define ARC2_STEP_IDLE 0
#define ARC2_STEP_WAIT_STOP 1
#define ARC2_STEP_STATIC 2
#define ARC2_STEP_MOVE 3
#define ARC2_STEP_SCALE_SEED 4
#define ARC2_STEP_SCALE_RUN 5
#define ARC2_STEP_SAVING 6
#define ARC2_KIND_SCALE 1
#define ARC2_KIND_EARTH 2
#define ARC2_KIND_GSENS 3

typedef struct {
    uint32_t magic, version, generation, reference_id, epoch;
    uint8_t kind, index, axis_key, up_axis;
    float latitude_deg;
    cal_arc_point_t point;
    cal_arc_motion_t motion; /* motion BEFORE this stop, empty for first */
    cal_arc_scale_axis_t scale;
    uint32_t checksum;
} arc2_record_t;

_Static_assert(sizeof(arc2_record_t) < 700u, "ARC5 NVS item must stay small");
static arc2_record_t arc2_saved[ARC2_NRECORDS], arc2_io, arc2_ready;
static uint8_t arc2_valid[ARC2_NRECORDS];
static uint32_t arc2_epoch[4];
static struct {
    uint32_t n;
    double g[3], g2[3], a[3], temperature, elapsed;
    float min_temp, max_temp;
} arc2_stat;
static cal_arc_motion_t arc2_motion;
static cal_arc_scale_axis_t arc2_scale;
static atomic_t arc2_step, arc2_flush_wait, arc2_stop_request;
static volatile uint8_t arc2_kind, arc2_index, arc2_scale_axis;
static uint32_t arc2_epoch_run;
static float arc2_series_elapsed;
static bool arc2_capture_error;
static bool arc2_combined; /* Earth first, then five GSENS flips; share reference stop */
static char arc2_error_message[96];
static cal_arc_tcal_t arc2_solver_tcal;
static cal_arc_scale_t arc2_solver_scale;
static cal_arc_earth_t arc2_solver_earth;
static cal_arc_gsens_t arc2_solver_gsens;
static cal_arc_solution_t arc2_candidate;

static uint16_t arc2_id(unsigned index,unsigned slot)
{
    unsigned base = index < 3 ? ARC2_SCALE_BASE + index * 2 :
         (index < 8 ? ARC2_EARTH_BASE + (index-3)*2 :
            ARC2_GSENS_BASE + (index-8)*2);
    return (uint16_t)(base+slot);
}
static unsigned arc2_offset(unsigned kind,unsigned i)
{
    return kind==ARC2_KIND_SCALE ? i :
        (kind==ARC2_KIND_EARTH ? 3u+i : 8u+i);
}
static unsigned arc2_limit(unsigned kind)
{
    return kind==ARC2_KIND_SCALE?3u:(kind==ARC2_KIND_EARTH?5u:6u);
}
static bool arc2_valid_record(const arc2_record_t *r,unsigned idx)
{
    uint8_t kind = idx<3? ARC2_KIND_SCALE:(idx<8?ARC2_KIND_EARTH:ARC2_KIND_GSENS);
    uint8_t pos  = idx<3?idx:(idx<8?idx-3:idx-8);
    return r->magic==ARC2_MAGIC && r->version==ARC2_VERSION &&
        r->reference_id!=0 && r->reference_id==arc_meta.reference_id &&
        r->kind==kind && r->index==pos &&
        r->checksum==cal_arc_checksum(r,offsetof(arc2_record_t,checksum));
}
static void arc2_clear_runtime(void)
{
    atomic_clear(&arc2_step);
    atomic_clear(&arc2_stop_request);
    atomic_clear(&arc2_flush_wait);
    arc2_kind=0;arc2_index=0;arc2_scale_axis=0;
    memset(&arc2_stat,0,sizeof(arc2_stat));
    memset(&arc2_motion,0,sizeof(arc2_motion));
    memset(&arc2_scale,0,sizeof(arc2_scale));
    arc2_series_elapsed=0;
    arc2_combined=false;
}
static void arc2_off(void) { /* Sensor stops instantly; no active archive load. */
    atomic_clear(&arc2_step);atomic_clear(&arc2_stop_request);
}
static void arc2_reset(void)
{
    arc2_clear_runtime();
    memset(arc2_valid,0,sizeof(arc2_valid));
    memset(arc2_saved,0,sizeof(arc2_saved));
    memset(arc2_epoch,0,sizeof(arc2_epoch));
}
static void arc2_load(void)
{
    arc2_reset();
    if(!arc_meta_valid(&arc_meta))return;
    for(unsigned i=0;i<ARC2_NRECORDS;i++) {
        bool found=false;
        for(unsigned slot=0;slot<2;slot++) {
            memset(&arc2_io,0,sizeof(arc2_io));
            int n=sys_arc_read(arc2_id(i,slot),&arc2_io,sizeof(arc2_io));
            if(n==(int)sizeof(arc2_io) && arc2_valid_record(&arc2_io,i) &&
               (!found || arc2_io.generation>arc2_saved[i].generation)) {
                arc2_saved[i]=arc2_io;found=true;
            }
        }
        arc2_valid[i]=found;
    }
    for(unsigned kind=ARC2_KIND_SCALE;kind<=ARC2_KIND_GSENS;kind++) {
        for(unsigned i=0;i<arc2_limit(kind);i++) {
            unsigned k=arc2_offset(kind,i);
            if(arc2_valid[k] && arc2_saved[k].epoch>arc2_epoch[kind])
                arc2_epoch[kind]=arc2_saved[k].epoch;
        }
    }
}
static unsigned arc2_contiguous(unsigned kind)
{
    if(kind==ARC2_KIND_SCALE){
        unsigned count=0;
        for(unsigned i=0;i<3;i++)if(arc2_valid[i])count++;
        return count;
    }
    unsigned count=0;uint32_t epoch=arc2_epoch[kind];
    for(unsigned i=0;i<arc2_limit(kind);i++) {
        unsigned k=arc2_offset(kind,i);
        if(!arc2_valid[k] || arc2_saved[k].epoch!=epoch)break;
        count++;
    }
    return count;
}
static bool arc2_busy(void) {return atomic_get(&arc2_step)!=ARC2_STEP_IDLE;}
static void arc2_status(void)
{
    const char *kind=arc2_kind==1?"scale":arc2_kind==2?"earth":arc2_kind==3?"gsens":"none";
    printk("ARC7 scale=%u/3 earth=%u/5 gsens=%u/6 capture=%s index=%u step=%d pending=%d combined=%d\n",
        arc2_contiguous(1),arc2_contiguous(2),arc2_contiguous(3),
        kind,arc2_index,(int)atomic_get(&arc2_step),
        (int)atomic_get(&arc2_flush_wait),arc2_combined);
}
static void arc2_fail(const char *why)
{
    arc2_combined=false;
    /* Never persist partial or questionable observations. */
    arc2_capture_error=true;
    strncpy(arc2_error_message,why,sizeof(arc2_error_message)-1);
    arc2_error_message[sizeof(arc2_error_message)-1]=0;
    atomic_clear(&arc2_stop_request);
    atomic_set(&arc2_step,ARC2_STEP_IDLE);
    arc_capture_phase(ARC_PHASE_FAILED,arc_error_code(why));
    printk("ARC5 capture rejected: %s\n",why);
}
static void arc2_reset_stat(void) {memset(&arc2_stat,0,sizeof(arc2_stat));}
static bool arc2_sample_stat(const float raw[3],const float a[3],float T,float dt)
{
    if(!arc2_stat.n){arc2_stat.min_temp=T;arc2_stat.max_temp=T;}
    if(T<arc2_stat.min_temp)arc2_stat.min_temp=T;
    if(T>arc2_stat.max_temp)arc2_stat.max_temp=T;
    if(arc2_stat.max_temp-arc2_stat.min_temp>.5f)return false;
    arc2_stat.n++;arc2_stat.temperature+=T;arc2_stat.elapsed+=dt;
    for(int i=0;i<3;i++) {
        arc2_stat.g[i]+=raw[i];arc2_stat.g2[i]+=(double)raw[i]*raw[i];
        arc2_stat.a[i]+=a[i];
    }
    return true;
}
static bool arc2_stat_point(cal_arc_point_t *p)
{
    memset(p,0,sizeof(*p));
    if(arc2_stat.n<100)return false;
    double n=arc2_stat.n;
    p->samples=arc2_stat.n;
    p->temp=(float)(arc2_stat.temperature/n);
    p->seconds=(float)arc2_stat.elapsed;
    p->quat[0]=1.f; /* orientation reconstructed from travel segments */
    for(int i=0;i<3;i++) {
        p->gyro[i]=(float)(arc2_stat.g[i]/n);
        p->accel[i]=(float)(arc2_stat.a[i]/n);
        double variance=arc2_stat.g2[i]/n-arc2_stat.g[i]*arc2_stat.g[i]/(n*n);
        p->gyro_sem[i]=(float)sqrt(variance>0?variance/n:0);
        if(!isfinite(p->gyro_sem[i])||p->gyro_sem[i]>.003f)return false;
    }
    return true;
}
static int arc2_axis_key(const float a[3])
{
    float norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if(norm<.96f||norm>1.04f)return -1;
    int k=0;for(int i=1;i<3;i++)if(fabsf(a[i])>fabsf(a[k]))k=i;
    if(fabsf(a[k])/norm<.90f)return -1; /* label only; solve uses actual 3-D vector */
    return 2*k+(a[k]<0?1:0);
}
static bool arc2_earth_motion_quality(void)
{
    if(arc2_index==0)return true;
    unsigned start=arc2_offset(ARC2_KIND_EARTH,0);
    if(!arc2_valid[start] || arc2_saved[start].epoch!=arc2_epoch_run)return false;
    const float *a=arc2_saved[start].point.accel;
    float an=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if(an<.90f||an>1.10f)return false;
    float d[3]={0};
    for(unsigned k=0;k<arc2_motion.count;k++)
        for(int j=0;j<3;j++)d[j]+=arc2_motion.seg[k].raw_delta[j];
    float along=(d[0]*a[0]+d[1]*a[1]+d[2]*a[2])/an;
    float off[3]={d[0]-along*a[0]/an,d[1]-along*a[1]/an,d[2]-along*a[2]/an};
    float off_mag=sqrtf(off[0]*off[0]+off[1]*off[1]+off[2]*off[2]);
    /* A mildly tilted turntable is acceptable: use actual 3-D attitude in
     * the solver. Only reject grossly wrong quarter-turn trajectories. */
    if(fabsf(along)<60.f||fabsf(along)>120.f||off_mag>40.f)return false;
    if(arc2_index>=2){
        unsigned prev=arc2_offset(ARC2_KIND_EARTH,1);
        if(!arc2_valid[prev]||arc2_saved[prev].epoch!=arc2_epoch_run)return false;
        float p=0;
        for(unsigned k=0;k<arc2_saved[prev].motion.count;k++)
            for(int j=0;j<3;j++)p+=arc2_saved[prev].motion.seg[k].raw_delta[j]*a[j]/an;
        if((arc2_index==2 && along*p<0) ||
           (arc2_index>=3 && along*p>0))return false;
    }
    return true;
}
static bool arc2_motion_good(void)
{
    if(!arc2_motion.count||arc2_motion.samples<20||arc2_motion.count>CAL_ARC_MSEG)return false;
    float theta[3]={0};
    for(unsigned s=0;s<arc2_motion.count;s++)
       for(int i=0;i<3;i++)theta[i]+=arc2_motion.seg[s].raw_delta[i];
    float magnitude=sqrtf(theta[0]*theta[0]+theta[1]*theta[1]+theta[2]*theta[2]);
    return isfinite(magnitude)&&magnitude>18.f&&magnitude<270.f&&
      isfinite(arc2_motion.lost_cross_deg)&&arc2_motion.lost_cross_deg<=1.5f; /* capture threshold; solver stricter */
}
static void arc2_request_flush(void)
{
    atomic_set(&arc2_step,ARC2_STEP_SAVING);
    atomic_set(&arc2_flush_wait,1);
    if(k_work_submit_to_queue(&arc_queue,&arc_save_work)<0) {
        atomic_clear(&arc2_flush_wait);arc2_fail("worker queue rejected save");
    }
}
static bool arc2_save_pending(void) {return atomic_get(&arc2_flush_wait)!=0;}
static void arc2_flush(void)
{
    unsigned kind=arc2_ready.kind,index=arc2_ready.index;
    unsigned k=arc2_offset(kind,index);
    if(k>=ARC2_NRECORDS || kind<1 || kind>3 || index>=arc2_limit(kind) ||
       !atomic_get(&arc_active) || arc2_ready.reference_id!=arc_meta.reference_id) {
        atomic_clear(&arc2_flush_wait);arc2_fail("reference changed / invalid record");return;
    }
    arc2_ready.magic=ARC2_MAGIC;arc2_ready.version=ARC2_VERSION;
    arc2_ready.generation=arc2_saved[k].generation+1;
    arc2_ready.checksum=cal_arc_checksum(&arc2_ready,offsetof(arc2_record_t,checksum));
    uint16_t id=arc2_id(k,arc2_ready.generation&1u);
    int written=sys_arc_write(id,&arc2_ready,sizeof(arc2_ready));
    memset(&arc2_io,0,sizeof(arc2_io));
    int n=written>=0 ? sys_arc_read(id,&arc2_io,sizeof(arc2_io)) : written;
    bool okay=n==(int)sizeof(arc2_ready) &&
        !memcmp(&arc2_io,&arc2_ready,sizeof(arc2_ready));
    if(!okay){atomic_clear(&arc2_flush_wait);arc2_fail("NVS write/readback failed; production untouched");return;}
    arc2_saved[k]=arc2_ready;arc2_valid[k]=1;
    if(arc2_ready.epoch>arc2_epoch[kind])arc2_epoch[kind]=arc2_ready.epoch;
    arc_capture_phase(ARC_PHASE_SAVED,ARC_ERROR_NONE);
    printk("ARC7 saved %s %u/%u (%u bytes, NVS %u)\n",
        kind==1?"scale":kind==2?"earth":"gsens",index+1,
        arc2_limit(kind),(unsigned)sizeof(arc2_ready),id);
    if(kind!=ARC2_KIND_SCALE&&index>0&&arc2_ready.motion.lost_cross_deg>.75f)
        /* F-06C3 DRAFT: saved, but cal_arc_motion_replay() will refuse this
         * transition (its own gate is >0.75f).  Informational only. */
        printk("ARC7 warn: motion loss %d mdeg > solver gate 750; capture-only record\n",
               (int)(arc2_ready.motion.lost_cross_deg*1000.0f+0.5f));
    atomic_clear(&arc2_flush_wait);
    if(kind==ARC2_KIND_EARTH && index+1==CAL_ARC_EARTH && arc2_combined) {
        /* One physical reference pose is sufficient for both datasets.
         * Save an independent GSENS stop-0 record; Earth is already durable.
         * This is worker-only NVS I/O and never runs in the sensor ISR/task. */
        const unsigned source=arc2_offset(ARC2_KIND_EARTH,0);
        const unsigned target=arc2_offset(ARC2_KIND_GSENS,0);
        if(!arc2_valid[source] || arc2_saved[source].epoch!=arc2_epoch_run){
            arc2_fail("combined reference Earth stop missing");return;
        }
        /* The fifth stop must return near the FIRST physical pose before
         * sharing its observation as GSENS pose 0. Check BOTH gravity tilt
         * and gyro-replayed 3-D closure; gravity alone cannot verify yaw. */
        const float *first_a=arc2_saved[source].point.accel;
        const float *last_a=arc2_saved[arc2_offset(ARC2_KIND_EARTH,4)].point.accel;
        float a_norm=sqrtf(first_a[0]*first_a[0]+first_a[1]*first_a[1]+first_a[2]*first_a[2]);
        float z_norm=sqrtf(last_a[0]*last_a[0]+last_a[1]*last_a[1]+last_a[2]*last_a[2]);
        if(a_norm<.90f||z_norm<.90f){arc2_fail("combined reference gravity invalid");return;}
        float match=(first_a[0]*last_a[0]+first_a[1]*last_a[1]+first_a[2]*last_a[2])/(a_norm*z_norm);
        if(match<.99619f){ /* cos(5 degrees) */
            arc2_fail("combined Earth final tilt differs from first pose >5 deg");return;
        }
        const float ones[3]={1,1,1},zero_m[9]={0},zero_e[3]={0};
        float closed_q[4]={1,0,0,0};
        for(unsigned k=1;k<5;k++){
            const arc2_record_t *step=&arc2_saved[arc2_offset(ARC2_KIND_EARTH,k)];
            float next[4];
            if(!cal_arc_motion_replay(&step->motion,closed_q,ones,zero_m,zero_e,
                                       arc2_saved[source].point.gyro,next)){
                arc2_fail("combined pose gyro closure replay failed");return;
            }
            memcpy(closed_q,next,sizeof(next));
        }
        float w=fminf(1.f,fabsf(closed_q[0]));
        if(2.f*acosf(w)/(float)0.017453292519943295f>5.f){
            arc2_fail("combined Earth final 3D closure >5 deg; restart GSENS separately");return;
        }
        arc2_ready=arc2_saved[source];
        arc2_ready.kind=ARC2_KIND_GSENS;
        arc2_ready.index=0;
        arc2_ready.epoch=arc2_epoch[ARC2_KIND_GSENS]+1;
        arc2_ready.generation=arc2_saved[target].generation+1;
        arc2_ready.axis_key=(uint8_t)arc2_axis_key(arc2_ready.point.accel);
        memset(&arc2_ready.motion,0,sizeof(arc2_ready.motion));
        if(arc2_ready.axis_key>5){arc2_fail("combined first pose not cardinal");return;}
        arc2_ready.checksum=cal_arc_checksum(&arc2_ready,offsetof(arc2_record_t,checksum));
        uint16_t shared_id=arc2_id(target,arc2_ready.generation&1u);
        int nw=sys_arc_write(shared_id,&arc2_ready,sizeof(arc2_ready));
        memset(&arc2_io,0,sizeof(arc2_io));
        int nr=nw>=0?sys_arc_read(shared_id,&arc2_io,sizeof(arc2_io)):nw;
        if(nr!=(int)sizeof(arc2_io)||memcmp(&arc2_io,&arc2_ready,sizeof(arc2_io))){
            arc2_fail("combined GSENS reference NVS write failed; Earth retained");return;
        }
        arc2_saved[target]=arc2_ready;
        arc2_valid[target]=1;
        arc2_epoch[ARC2_KIND_GSENS]=arc2_ready.epoch;
        arc2_epoch_run=arc2_ready.epoch;
        arc2_kind=ARC2_KIND_GSENS;
        arc2_index=1;
        memset(&arc2_motion,0,sizeof(arc2_motion));
        arc2_series_elapsed=0;
        atomic_set(&arc2_step,ARC2_STEP_MOVE);
        printk("ARC7 COMBINED: Earth 5/5 saved + GSENS 1/6 reused; flip to another face and run gsens pose (5 remain)\n");
        return;
    }
    if(kind==ARC2_KIND_SCALE || index+1==arc2_limit(kind)) {
        atomic_set(&arc2_step,ARC2_STEP_IDLE);
        arc2_combined=false;
        arc2_kind=0;
        printk("ARC7 %s capture %s; no production coefficients changed\n",
            kind==1?"scale":kind==2?"earth":"gsens",
            kind==1?"saved": "series complete");
    } else {
        arc2_index=index+1;
        memset(&arc2_motion,0,sizeof(arc2_motion));
        atomic_set(&arc2_step,ARC2_STEP_MOVE);
        printk("ARC5 move to next %s pose, then run cal archive %s %s\n",
            kind==ARC2_KIND_EARTH?"Earth":"g-sens",
            kind==ARC2_KIND_EARTH?"earth":"gsens",
            kind==ARC2_KIND_EARTH?"stop":"pose");
    }
}
static bool arc2_other_calibrations_busy(void)
{
    return atomic_get(&arc_capture_running) || atomic_get(&arc_save_pending) ||
        sensor_gyro_scale_active() || sensor_earth_cal_active() ||
        sensor_gsens_guide_active();
}
static bool arc2_control(int cmd)
{
    if(cmd==ARC_CAPTURE_CANCEL) {
        arc2_clear_runtime();printk("ARC5 capture cancelled; saved observations unchanged\n");return true;
    }
    if(cmd>=ARC_SX_START&&cmd<=ARC_SZ_START) {
        if(atomic_get(&arc2_step)||arc2_other_calibrations_busy()){
            printk("ARC5 scale refused: capture/calibration busy\n");return false;
        }
        float a[3];if(!arc_reference_ready(a)){
            printk("ARC5 scale refused: hold the selected axis upright and still\n");return false;
        }
        int axis=cmd-ARC_SX_START;
        int key=arc2_axis_key(a);
        if(key!=2*axis && key!=2*axis+1){
            printk("ARC5 scale refused: axis %c not vertical\n","XYZ"[axis]);return false;
        }
        arc_capture_arm_current();
        arc2_kind=ARC2_KIND_SCALE;arc2_index=axis;arc2_scale_axis=axis;
        arc2_epoch_run=arc2_epoch[1]+1;
        memset(&arc2_scale,0,sizeof(arc2_scale));
        arc2_reset_stat();atomic_set(&arc2_step,ARC2_STEP_SCALE_SEED);
        printk("ARC5 scale %c: hold still ~2s; then rotate exactly 20 turns; use scale %c stop at rest\n",
            "XYZ"[axis],"XYZ"[axis]);return true;
    }
    if(cmd>=ARC_SX_STOP&&cmd<=ARC_SZ_STOP) {
        if(arc2_kind!=ARC2_KIND_SCALE || arc2_scale_axis!=cmd-ARC_SX_STOP ||
           atomic_get(&arc2_step)!=ARC2_STEP_SCALE_RUN){
            printk("ARC5 scale stop refused: wrong axis or not READY\n");return false;
        }
        arc_capture_arm_current();
        atomic_set(&arc2_stop_request,1);
        printk("ARC5 scale stop queued (requires rest)\n");return true;
    }
    if(cmd==ARC_EARTH_START || cmd==ARC_GSENS_START || cmd==ARC_COMBINED_START) {
        if(atomic_get(&arc2_step)||arc2_other_calibrations_busy()){
            printk("ARC5 series refused: capture/calibration busy\n");return false;
        }
        float a[3];if(!arc_reference_ready(a)||!arc_reference_accel_matches(a)) {
            printk("ARC5 series refused: place tracker at confirmed reference pose first\n");return false;
        }
        arc_capture_arm_current();
        arc2_combined=(cmd==ARC_COMBINED_START);
        arc2_kind=(cmd==ARC_EARTH_START || arc2_combined)?ARC2_KIND_EARTH:ARC2_KIND_GSENS;
        arc2_epoch_run=arc2_epoch[arc2_kind]+1;
        arc2_index=0;
        arc2_reset_stat();memset(&arc2_motion,0,sizeof(arc2_motion));
        arc2_series_elapsed=0;
        atomic_set(&arc2_step,ARC2_STEP_WAIT_STOP);
        printk("ARC7 %s series started%s; first stop uses reference pose; run %s at first stable pose\n",
            arc2_kind==ARC2_KIND_EARTH?"Earth":"g-sens",
            arc2_combined?" [combined, Earth first + five GSENS flips]":"",
            arc2_kind==ARC2_KIND_EARTH?"earth stop":"gsens pose");return true;
    }
    if(cmd==ARC_EARTH_STOP || cmd==ARC_GSENS_POSE) {
        unsigned kind=cmd==ARC_EARTH_STOP?ARC2_KIND_EARTH:ARC2_KIND_GSENS;
        if(arc2_kind!=kind || (atomic_get(&arc2_step)!=ARC2_STEP_WAIT_STOP &&
            atomic_get(&arc2_step)!=ARC2_STEP_MOVE)) {
            printk("ARC5 stop refused: wrong series or pending save\n");return false;
        }
        arc_capture_arm_current();
        atomic_set(&arc2_stop_request,1);
        printk("ARC5 pose %u/%u requested; keep still until saved\n",
            arc2_index+1,arc2_limit(kind));return true;
    }
    return false;
}
static void arc2_feed(const float raw[3],const float a[3],float T,float dt,bool rest)
{
    int step=atomic_get(&arc2_step);
    if(step==ARC2_STEP_IDLE || step==ARC2_STEP_SAVING)return;
    float am=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if(!isfinite(am)||!isfinite(T)||!isfinite(dt)||dt<=0.f||dt>.025f) {
        arc2_fail("invalid sensor time/temperature/accel");return;
    }
    for(int i=0;i<3;i++)if(!isfinite(raw[i])||fabsf(raw[i])>3900.f||!isfinite(a[i])) {
        arc2_fail("gyro saturation/nonfinite sample");return;
    }
    if(arc2_kind==ARC2_KIND_EARTH || arc2_kind==ARC2_KIND_GSENS)
        arc2_series_elapsed+=dt;
    if(step==ARC2_STEP_WAIT_STOP || step==ARC2_STEP_MOVE) {
        if(atomic_cas(&arc2_stop_request,1,0)) {
            if(!rest || am<.95f || am>1.05f){
                printk("ARC5 pose rejected: wait for full rest then retry\n");return;
            }
            if(arc2_index==0 && !arc_reference_accel_matches(a)){
                printk("ARC5 first pose rejected: not reference tilt\n");return;
            }
            if(arc2_index>0 && (!arc2_motion_good() ||
               (arc2_kind==ARC2_KIND_EARTH && !arc2_earth_motion_quality()))){
                arc2_fail("transition rotation insufficient / compression error too large");return;
            }
            arc2_reset_stat();atomic_set(&arc2_step,ARC2_STEP_STATIC);
            step=ARC2_STEP_STATIC;
        } else {
            if(step==ARC2_STEP_MOVE) {
                cal_arc_motion_add(&arc2_motion,raw,a,T,dt);
                if(arc2_motion.lost_cross_deg>1.5f)arc2_fail("motion compression loss >1.5 deg; move more slowly");
            }
            return;
        }
    }
    if(step==ARC2_STEP_SCALE_SEED) {
        if(!rest||am<.95f||am>1.05f){arc2_reset_stat();return;}
        if(!arc2_sample_stat(raw,a,T,dt)){arc2_reset_stat();return;}
        if(arc2_stat.n<ARC2_QUALITY_SEED)return;
        for(int i=0;i<3;i++) {
            arc2_scale.initial_raw[i]=(float)(arc2_stat.g[i]/arc2_stat.n);
            arc2_scale.initial_accel[i]=(float)(arc2_stat.a[i]/arc2_stat.n);
        }
        arc2_scale.initial_temp=(float)(arc2_stat.temperature/arc2_stat.n);
        arc2_scale.axis=arc2_scale_axis;arc2_scale.turns=20;
        arc2_reset_stat();atomic_set(&arc2_step,ARC2_STEP_SCALE_RUN);
        printk("ARC5 scale %c READY: perform 20 rotations, then return to rest\n",
            "XYZ"[arc2_scale_axis]);return;
    }
    if(step==ARC2_STEP_SCALE_RUN) {
        if(arc2_scale.duration>600.f){arc2_fail("scale capture timed out");return;}
        arc2_scale.duration+=dt;arc2_scale.temp_time+=T*dt;
        if(arc2_scale.duration<=dt){arc2_scale.temp_min=T;arc2_scale.temp_max=T;}
        if(T<arc2_scale.temp_min)arc2_scale.temp_min=T;
        if(T>arc2_scale.temp_max)arc2_scale.temp_max=T;
        float dT=T-25.f;
        arc2_scale.temp_moments[0]+=dT*dt;
        arc2_scale.temp_moments[1]+=dT*dT*dt;
        arc2_scale.temp_moments[2]+=dT*dT*dT*dt;
        for(int i=0;i<3;i++){
            arc2_scale.raw_angle[i]+=raw[i]*dt;
            arc2_scale.accel_time[i]+=a[i]*dt;
        }
        float off_axis=0;
        for(int i=0;i<3;i++)if(i!=arc2_scale_axis) {
            float r=raw[i]-arc2_scale.initial_raw[i];off_axis+=r*r;
        }
        arc2_scale.cross_angle+=sqrtf(off_axis)*dt;
        if(atomic_cas(&arc2_stop_request,1,0)) {
            if(!rest){printk("ARC5 scale stop: not at rest; retry\n");return;}
            float corrected=arc2_scale.raw_angle[arc2_scale_axis]-
                arc2_scale.initial_raw[arc2_scale_axis]*arc2_scale.duration;
            if(fabsf(corrected)<6700.f||fabsf(corrected)>7800.f||
               arc2_scale.duration<10.f || arc2_scale.cross_angle>1100.f) {
                arc2_fail("20-turn raw integral/cross-axis quality out of range");return;
            }
            arc2_scale.quality=arc2_scale.cross_angle/7200.f;
            memset(&arc2_ready,0,sizeof(arc2_ready));
            arc2_ready.reference_id=arc_meta.reference_id;
            arc2_ready.kind=ARC2_KIND_SCALE;arc2_ready.index=arc2_scale_axis;
            arc2_ready.epoch=arc2_epoch_run;arc2_ready.scale=arc2_scale;
            arc2_request_flush();
        }
        return;
    }
    if(step==ARC2_STEP_STATIC) {
        if(!rest||am<.95f||am>1.05f){arc2_reset_stat();return;}
        if(!arc2_sample_stat(raw,a,T,dt)){arc2_reset_stat();return;}
        uint32_t required=arc2_kind==ARC2_KIND_EARTH?
            ARC2_QUALITY_STATIC:ARC2_QUALITY_GSENS;
        if(arc2_stat.n<required)return;
        memset(&arc2_ready,0,sizeof(arc2_ready));
        if(!arc2_stat_point(&arc2_ready.point)) {
            arc2_fail("pose noise / temperature instability");return;
        }
        arc2_ready.point.seconds=arc2_series_elapsed;
        arc2_ready.reference_id=arc_meta.reference_id;
        arc2_ready.kind=arc2_kind;arc2_ready.index=arc2_index;
        arc2_ready.epoch=arc2_epoch_run;
        if(arc2_kind==ARC2_KIND_EARTH) {
            float aavg[3];memcpy(aavg,arc2_ready.point.accel,sizeof(aavg));
            int up=arc2_axis_key(aavg);
            if(arc2_index==0 && up<0){arc2_fail("invalid first Earth up axis");return;}
            unsigned old0=arc2_offset(ARC2_KIND_EARTH,0);
            arc2_ready.up_axis=arc2_index==0?(uint8_t)up:arc2_saved[old0].up_axis;
            arc2_ready.latitude_deg=sensor_earth_reference_latitude();
        } else {
            int key=arc2_axis_key(arc2_ready.point.accel);
            if(key<0){arc2_fail("g-sens pose not cardinal");return;}
            for(unsigned p=0;p<arc2_index;p++) {
                unsigned n=arc2_offset(ARC2_KIND_GSENS,p);
                if(arc2_valid[n] && arc2_saved[n].epoch==arc2_epoch_run &&
                   arc2_saved[n].axis_key==key) {
                    arc2_fail("duplicate g-sens axis-up pose");return;
                }
            }
            arc2_ready.axis_key=(uint8_t)key;
        }
        if(arc2_index>0)arc2_ready.motion=arc2_motion;
        arc2_request_flush();
    }
}
static void arc2_solve_preview(void)
{
    if(atomic_get(&arc2_step)||arc2_contiguous(1)!=3||
        arc2_contiguous(2)!=5||arc2_contiguous(3)!=6||arc_points.count<1) {
        printk("ARC5 solve REFUSED: complete scale x/y/z, 5 Earth, 6 g-sens and TCal; no production writes\n");return;
    }
    memset(&arc2_solver_tcal,0,sizeof(arc2_solver_tcal));
    memset(&arc2_solver_scale,0,sizeof(arc2_solver_scale));
    memset(&arc2_solver_earth,0,sizeof(arc2_solver_earth));
    memset(&arc2_solver_gsens,0,sizeof(arc2_solver_gsens));
    arc2_solver_tcal=arc_points;
    arc2_solver_scale.count=3;arc2_solver_earth.count=5;arc2_solver_gsens.count=6;
    arc2_solver_scale.reference_id=arc2_solver_earth.reference_id=
        arc2_solver_gsens.reference_id=arc_meta.reference_id;
    arc2_solver_earth.up_axis=arc2_saved[3].up_axis;
    arc2_solver_earth.latitude_deg=arc2_saved[3].latitude_deg;
    for(int i=0;i<3;i++)arc2_solver_scale.axis[i]=arc2_saved[i].scale;
    for(int i=0;i<5;i++){
        arc2_solver_earth.stop[i]=arc2_saved[3+i].point;
        if(i)arc2_solver_earth.travel[i-1]=arc2_saved[3+i].motion;
    }
    for(int i=0;i<6;i++){
        arc2_solver_gsens.pose[i]=arc2_saved[8+i].point;
        if(i)arc2_solver_gsens.travel[i-1]=arc2_saved[8+i].motion;
    }
    if(!cal_arc_solve(&arc2_solver_tcal,&arc2_solver_scale,&arc2_solver_earth,
                      &arc2_solver_gsens,&arc2_candidate) || !arc2_candidate.ready) {
        printk("ARC5 solver rejected candidate (model/quality not identifiable); PRODUCTION unchanged\n");return;
    }
    printk("ARC5 CANDIDATE ONLY: scale=%.7f %.7f %.7f Earth=%.7f %.7f %.7f gRMS=%.6f eRMS=%.6f\n",
        (double)arc2_candidate.scale[0],(double)arc2_candidate.scale[1],
        (double)arc2_candidate.scale[2],(double)arc2_candidate.earth_body[0],
        (double)arc2_candidate.earth_body[1],(double)arc2_candidate.earth_body[2],
        (double)arc2_candidate.gsens_rms,(double)arc2_candidate.earth_rms);
    printk("ARC5 NO PRODUCTION NVS COMMIT -- candidate requires independent validation\n");
}

/* ARC8-H3: NVS snapshot -> console export. This path never calls cal_arc_solve,
 * any production setter, or sys_arc_write(). It runs only on the lazy worker. */
static int arc2_export_emit_line(const char *line, void *ctx)
{
    (void)ctx;
    printk("%s\n", line);
    k_msleep(5); /* avoid exhausting slow CDC/RTT host buffers */
    return 0;
}
static void arc2_export_wire(void)
{
    if(!arc_meta_valid(&arc_meta) || !arc_points_valid(&arc_points) ||
       arc_points.reference_id!=arc_meta.reference_id ||
       !arc_points.generation || !arc_points.count ||
       arc2_contiguous(ARC2_KIND_SCALE)!=3 ||
       arc2_contiguous(ARC2_KIND_EARTH)!=5 ||
       arc2_contiguous(ARC2_KIND_GSENS)!=6) {
        printk("ARC8X1 ERROR incomplete_archive\n");return;
    }
    /* Verify each copied NVS record, including its CRC, before exporting. */
    for(unsigned i=0;i<ARC2_NRECORDS;i++) {
        if(!arc2_valid[i] || !arc2_valid_record(&arc2_saved[i],i) ||
           !arc2_saved[i].generation ||
           (i>=3 && i<8 && arc2_saved[i].epoch!=arc2_epoch[ARC2_KIND_EARTH]) ||
           (i>=8 && arc2_saved[i].epoch!=arc2_epoch[ARC2_KIND_GSENS])) {
            printk("ARC8X1 ERROR invalid_record\n");return;
        }
    }
    memset(&arc2_solver_tcal,0,sizeof(arc2_solver_tcal));
    memset(&arc2_solver_scale,0,sizeof(arc2_solver_scale));
    memset(&arc2_solver_earth,0,sizeof(arc2_solver_earth));
    memset(&arc2_solver_gsens,0,sizeof(arc2_solver_gsens));
    arc2_solver_tcal=arc_points;
    cal_arc_scale_t *s=&arc2_solver_scale;
    cal_arc_earth_t *e=&arc2_solver_earth;
    cal_arc_gsens_t *g=&arc2_solver_gsens;
    s->magic=e->magic=g->magic=CAL_ARC_MAGIC;
    s->version=e->version=g->version=CAL_ARC_VERSION;
    s->reference_id=e->reference_id=g->reference_id=arc_meta.reference_id;
    s->count=3;e->count=5;g->count=6;
    e->up_axis=arc2_saved[3].up_axis;
    e->latitude_deg=arc2_saved[3].latitude_deg;
    for(unsigned i=0;i<3;i++) {
        s->axis[i]=arc2_saved[i].scale;
        if(arc2_saved[i].generation>s->generation)
            s->generation=arc2_saved[i].generation;
    }
    for(unsigned i=0;i<5;i++) {
        e->stop[i]=arc2_saved[i+3].point;
        if(i)e->travel[i-1]=arc2_saved[i+3].motion;
        if(arc2_saved[i+3].generation>e->generation)
            e->generation=arc2_saved[i+3].generation;
    }
    for(unsigned i=0;i<6;i++) {
        g->pose[i]=arc2_saved[i+8].point;
        if(i)g->travel[i-1]=arc2_saved[i+8].motion;
        if(arc2_saved[i+8].generation>g->generation)
            g->generation=arc2_saved[i+8].generation;
    }
    s->checksum=cal_arc_checksum(s,offsetof(cal_arc_scale_t,checksum));
    e->checksum=cal_arc_checksum(e,offsetof(cal_arc_earth_t,checksum));
    g->checksum=cal_arc_checksum(g,offsetof(cal_arc_gsens_t,checksum));
    /* Every packet is self-describing and CRC-protected; the PC refuses gaps. */
    if(arc8_wire_dump("tcal",&arc2_solver_tcal,sizeof(arc2_solver_tcal),
        arc_meta.reference_id,arc2_solver_tcal.generation,arc2_export_emit_line,NULL) ||
       arc8_wire_dump("scale",s,sizeof(*s),arc_meta.reference_id,
        s->generation,arc2_export_emit_line,NULL) ||
       arc8_wire_dump("earth",e,sizeof(*e),arc_meta.reference_id,
        e->generation,arc2_export_emit_line,NULL) ||
       arc8_wire_dump("gsens",g,sizeof(*g),arc_meta.reference_id,
        g->generation,arc2_export_emit_line,NULL)) {
        printk("ARC8X1 ERROR emit_failed\n");return;
    }
    printk("ARC8X1 DONE %u\n",arc_meta.reference_id);
}

#if defined(CONFIG_ARC8P1_READONLY_EXPORT_DRAFT)
/* F-06C3 DRAFT ONLY: self-contained SHA-256 for read-only ARC8P1 diagnostics.
 * Purposefully does not depend on target mbedTLS/Kconfig selection.
 * No security claim for host/device origin: only wire-payload integrity.
 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>

typedef struct {
    uint32_t h[8];
    uint8_t block[64];
    uint64_t nbytes;
    unsigned fill;
} arc8p1_sha256_ctx;

static uint32_t arc8p1_rotr(uint32_t x, unsigned n) { return (x >> n) | (x << (32u-n)); }
static void arc8p1_sha256_transform(arc8p1_sha256_ctx *s, const uint8_t *p)
{
    static const uint32_t k[64] = {
        0x428a2f98u,0x71374491u,0xb5c0fbcfu,0xe9b5dba5u,0x3956c25bu,0x59f111f1u,0x923f82a4u,0xab1c5ed5u,
        0xd807aa98u,0x12835b01u,0x243185beu,0x550c7dc3u,0x72be5d74u,0x80deb1feu,0x9bdc06a7u,0xc19bf174u,
        0xe49b69c1u,0xefbe4786u,0x0fc19dc6u,0x240ca1ccu,0x2de92c6fu,0x4a7484aau,0x5cb0a9dcu,0x76f988dau,
        0x983e5152u,0xa831c66du,0xb00327c8u,0xbf597fc7u,0xc6e00bf3u,0xd5a79147u,0x06ca6351u,0x14292967u,
        0x27b70a85u,0x2e1b2138u,0x4d2c6dfcu,0x53380d13u,0x650a7354u,0x766a0abbu,0x81c2c92eu,0x92722c85u,
        0xa2bfe8a1u,0xa81a664bu,0xc24b8b70u,0xc76c51a3u,0xd192e819u,0xd6990624u,0xf40e3585u,0x106aa070u,
        0x19a4c116u,0x1e376c08u,0x2748774cu,0x34b0bcb5u,0x391c0cb3u,0x4ed8aa4au,0x5b9cca4fu,0x682e6ff3u,
        0x748f82eeu,0x78a5636fu,0x84c87814u,0x8cc70208u,0x90befffau,0xa4506cebu,0xbef9a3f7u,0xc67178f2u
    };
    uint32_t w[64];
    for(unsigned i=0;i<16;i++)
        w[i]=((uint32_t)p[4*i]<<24)|((uint32_t)p[4*i+1]<<16)|((uint32_t)p[4*i+2]<<8)|p[4*i+3];
    for(unsigned i=16;i<64;i++) {
        uint32_t a=w[i-15], b=w[i-2];
        w[i]=w[i-16]+(arc8p1_rotr(a,7)^arc8p1_rotr(a,18)^(a>>3))+
             w[i-7]+(arc8p1_rotr(b,17)^arc8p1_rotr(b,19)^(b>>10));
    }
    uint32_t a=s->h[0],b=s->h[1],c=s->h[2],d=s->h[3];
    uint32_t e=s->h[4],f=s->h[5],g=s->h[6],h=s->h[7];
    for(unsigned i=0;i<64;i++) {
        uint32_t s1=arc8p1_rotr(e,6)^arc8p1_rotr(e,11)^arc8p1_rotr(e,25);
        uint32_t t1=h+s1+((e&f)^(~e&g))+k[i]+w[i];
        uint32_t s0=arc8p1_rotr(a,2)^arc8p1_rotr(a,13)^arc8p1_rotr(a,22);
        uint32_t t2=s0+((a&b)^(a&c)^(b&c));
        h=g;g=f;f=e;e=d+t1;d=c;c=b;b=a;a=t1+t2;
    }
    s->h[0]+=a;s->h[1]+=b;s->h[2]+=c;s->h[3]+=d;
    s->h[4]+=e;s->h[5]+=f;s->h[6]+=g;s->h[7]+=h;
}
static void arc8p1_sha256_init(arc8p1_sha256_ctx *s)
{
    static const uint32_t seed[8]={0x6a09e667u,0xbb67ae85u,0x3c6ef372u,0xa54ff53au,
                                   0x510e527fu,0x9b05688cu,0x1f83d9abu,0x5be0cd19u};
    memset(s,0,sizeof(*s));memcpy(s->h,seed,sizeof(seed));
}
static void arc8p1_sha256_update(arc8p1_sha256_ctx *s, const void *src, size_t len)
{
    const uint8_t *p=(const uint8_t *)src;
    s->nbytes+=(uint64_t)len;
    for(size_t i=0;i<len;i++) {
        s->block[s->fill++]=p[i];
        if(s->fill==64) {arc8p1_sha256_transform(s,s->block);s->fill=0;}
    }
}
static void arc8p1_sha256_final(arc8p1_sha256_ctx *s,uint8_t out[32])
{
    uint64_t bits=s->nbytes*8;
    s->block[s->fill++]=0x80u;
    if(s->fill>56) {
        memset(s->block+s->fill,0,64-s->fill);
        arc8p1_sha256_transform(s,s->block);s->fill=0;
    }
    memset(s->block+s->fill,0,56-s->fill);
    for(unsigned i=0;i<8;i++)s->block[56+i]=(uint8_t)(bits>>(56-8*i));
    arc8p1_sha256_transform(s,s->block);
    for(unsigned i=0;i<8;i++)for(unsigned j=0;j<4;j++)out[4*i+j]=(uint8_t)(s->h[i]>>(24-8*j));
}
static void arc8p1_digest_hex(const uint8_t digest[32],char out[65])
{
    static const char hex[]="0123456789abcdef";
    for(unsigned i=0;i<32;i++){out[2*i]=hex[digest[i]>>4];out[2*i+1]=hex[digest[i]&15u];}
    out[64]=0;
}
/* Compile-time archive layout contract; TCal remains a distinct raw record. */
_Static_assert(sizeof(arc2_record_t)==584u, "ARC8P1 requires 584-byte ARC2 records");
_Static_assert(CAL_ARC_SCALE==3, "ARC8P1 requires Scale capacity 3");
_Static_assert(CAL_ARC_EARTH==5, "ARC8P1 requires Earth capacity 5");
_Static_assert(CAL_ARC_GSENS==6, "ARC8P1 requires GSens capacity 6");
_Static_assert(ARC2_NRECORDS==(CAL_ARC_SCALE+CAL_ARC_EARTH+CAL_ARC_GSENS),
               "ARC8P1 slot layout must total 14 records");
_Static_assert(CAL_ARC_TMAX==12, "TCal point capacity is 12, not ARC2 slots");
#define ARC8P1_SNAP_LIMIT CAL_ARC_GSENS
static arc2_record_t arc8p1_snapshot[ARC8P1_SNAP_LIMIT];
static uint8_t arc8p1_slot[ARC8P1_SNAP_LIMIT];

static const char *arc8p1_kind_name(unsigned kind)
{
    return kind==ARC2_KIND_SCALE ? "scale" :
           kind==ARC2_KIND_EARTH ? "earth" : "gsens";
}

static bool arc8p1_snapshot_allowed(void)
{
    /* All arc2_saved[] and arc_meta writes are on arc_queue; this control
     * handler runs on that SAME queue. The sensor thread can only produce
     * pending capture/save flags and arc2_ready, never arc2_saved[]. */
    return atomic_get(&arc_active) && arc_meta_valid(&arc_meta) &&
        atomic_get(&arc2_step)==ARC2_STEP_IDLE && !arc2_save_pending() &&
        !atomic_get(&arc_capture_running) && !atomic_get(&arc_save_pending) &&
        !atomic_get(&arc_collect_request);
}

static bool arc2_partial_export(unsigned kind)
{
    if(kind<ARC2_KIND_SCALE || kind>ARC2_KIND_GSENS) {
        printk("ARC8P1 ERROR invalid_kind\n");return false;
    }
    if(!arc8p1_snapshot_allowed()) {
        printk("ARC8P1 ERROR NOT_SNAPSHOTTABLE\n");return false;
    }
    const uint32_t reference_id=arc_meta.reference_id;
    const unsigned limit=arc2_limit(kind);
    const char *name=arc8p1_kind_name(kind);
    unsigned count=0;
    if(limit>ARC8P1_SNAP_LIMIT) {
        printk("ARC8P1 ERROR NOT_SNAPSHOTTABLE\n");return false;
    }
    /* No DATA, BEGIN, or success END may escape until all marked-valid
     * records have passed CRC/identity validation in the frozen snapshot.
     * Preserve original sparse slot IDs, not dense output indices. */
    for(unsigned i=0;i<limit;i++) {
        const unsigned k=arc2_offset(kind,i);
        if(k>=ARC2_NRECORDS) {
            printk("ARC8P1 ERROR NOT_SNAPSHOTTABLE\n");return false;
        }
        if(!arc2_valid[k])continue;
        arc8p1_snapshot[count]=arc2_saved[k];
        arc8p1_slot[count]=(uint8_t)i;
        if(!arc2_valid_record(&arc8p1_snapshot[count],k) ||
           !arc8p1_snapshot[count].generation) {
            printk("ARC8P1 ERROR INVALID_RECORD %s %u\n",name,i);
            return false;
        }
        count++;
    }
    arc8p1_sha256_ctx sha;
    uint8_t raw_digest[32];
    char digest[65];
    arc8p1_sha256_init(&sha);
    for(unsigned j=0;j<count;j++)
        arc8p1_sha256_update(&sha,&arc8p1_snapshot[j],sizeof(arc8p1_snapshot[j]));
    arc8p1_sha256_final(&sha,raw_digest);
    arc8p1_digest_hex(raw_digest,digest);
    /* Recheck *after* building the complete snapshot and digest, but before
     * BEGIN. Worker serialization prevents archive mutation during emission.
     * The reference ID is captured once and used in the wire header. */
    if(!arc8p1_snapshot_allowed() || arc_meta.reference_id!=reference_id) {
        printk("ARC8P1 ERROR NOT_SNAPSHOTTABLE\n");return false;
    }
    printk("ARC8P1 BEGIN %s %u %u %u %s\n",name,count,limit,
           reference_id,digest);
    for(unsigned j=0;j<count;j++) {
        const uint8_t *p=(const uint8_t *)&arc8p1_snapshot[j];
        for(unsigned offset=0;offset<sizeof(arc2_record_t);offset+=32u) {
            const unsigned left=(unsigned)sizeof(arc2_record_t)-offset;
            const unsigned n=left<32u?left:32u;
            char datahex[65];
            static const char hex[]="0123456789abcdef";
            for(unsigned z=0;z<n;z++) {
                datahex[2*z]=hex[p[offset+z]>>4];
                datahex[2*z+1]=hex[p[offset+z]&15u];
            }
            datahex[n*2]=0;
            printk("ARC8P1 DATA %s %u %u %s\n",name,
                   (unsigned)arc8p1_slot[j],offset,datahex);
            k_msleep(5); /* same bounded CDC/RTT pacing as ARC8X1 */
        }
    }
    printk("ARC8P1 END %s %u %s\n",name,
           count*(unsigned)sizeof(arc2_record_t),digest);
    return true;
}

static bool arc_tcal_export(void)
{
    if(!arc8p1_snapshot_allowed()) {
        printk("ARC8T1 ERROR busy_or_not_loaded\n");return false;
    }
    /* Worker-owned source and snapshot; sensor only owns the ready point. */
    arc2_solver_tcal=arc_points;
    if(!arc_points_valid(&arc2_solver_tcal) || !arc2_solver_tcal.count ||
       !arc2_solver_tcal.generation ||
       arc2_solver_tcal.reference_id!=arc_meta.reference_id) {
        printk("ARC8T1 ERROR invalid_or_missing_tcal\n");return false;
    }
    if(arc8_wire_dump_tcal(&arc2_solver_tcal,arc_meta.reference_id,
            arc2_solver_tcal.generation,arc2_export_emit_line,NULL)) {
        printk("ARC8T1 ERROR transport\n");return false;
    }
    return true;
}
#endif /* CONFIG_ARC8P1_READONLY_EXPORT_DRAFT */

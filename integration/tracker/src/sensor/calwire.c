/* User-activated wireless calibration transfer. One frozen snapshot, one
 * session, finite lease. Reads never write calibration. Each APPLY commits one
 * NVS record from the sensor thread, with compare-before-write and readback.
 * A three-record profile is NOT an atomic transaction; host journals/rolls back.
 */
#include "calwire.h"
#include "calwire_protocol.h"
#include "calibration.h"
#include "connection/esb.h"
#include "connection/connection.h"
#include "system/system.h"
#include "system/arc8_esb_ota.h"
#include "sensor/f04_summary.h"
#include "retained.h"
#include "build_defines.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <hal/nrf_ficr.h>
#include <errno.h>

K_MSGQ_DEFINE(cw_queue,CW_SIZE,2,4);
static atomic_t active,apply_state;
static uint8_t snapshot[12288],scratch[4096],staged[60],present[60];
static uint8_t apply_bytes[36] __aligned(4),apply_before[36] __aligned(4);
static unsigned apply_field;
static uint32_t session,last_session,expires,thermal_crc;
static size_t snapshot_size;
static uint8_t begin_reply[8];
bool calwire_active(void){return atomic_get(&active)!=0;}
static bool idle(void){return !arc8_ota_active()&&!f04_summary_active()&&
    !connection_get_data_collection()&&!sensor_tcal_get_auto_calibration()&&
    !sensor_earth_cal_active()&&!sensor_gyro_scale_active()&&!sensor_gsens_guide_active()&&
    cal_archive_wire_idle();}
static const float *field_ptr(unsigned f){return f==1?retained->gyroSensScale:
    f==2?retained->gyroGsens:retained->earthCalBody;}
void calwire_sensor_poll(bool rest){
    if(!atomic_cas(&apply_state,1,2))return;
    bool ok=rest&&idle()&&!memcmp(field_ptr(apply_field),apply_before,cw_field_size(apply_field));
    if(ok)ok=apply_field==1?sys_gyro_scale_save((float*)apply_bytes):
        apply_field==2?sys_gyro_gsens_save((float*)apply_bytes):sys_earth_reference_save((float*)apply_bytes);
    if(ok)ok=!memcmp(field_ptr(apply_field),apply_bytes,cw_field_size(apply_field));
    atomic_set(&apply_state,ok?3:4);
}
void calwire_receive(const uint8_t *p,size_t n){
    if(!cw_valid(p,n,0x26))return;
    uint64_t addr=(*(uint64_t*)NRF_FICR->DEVICEADDR)&0xffffffffffffULL;
    if(memcmp(p+8,&addr,6))return;
    (void)k_msgq_put(&cw_queue,p,K_NO_WAIT);
}
static bool append(unsigned type,const void *p,size_t n){
    if(snapshot_size+4+n>sizeof(snapshot)||n>65535)return false;
    cw_put16(snapshot+snapshot_size,type);cw_put16(snapshot+snapshot_size+2,n);
    memcpy(snapshot+snapshot_size+4,p,n);snapshot_size+=4+n;return true;
}
static bool take_snapshot(void){
    snapshot_size=0;
    const char build[]=TOSTRING(APP_BUILD_VERSION);
    if(!append(1,build,sizeof(build)))return false;
    uint8_t profile[64];
    unsigned key=irq_lock();
    memcpy(profile,retained->gyroSensScale,12);memcpy(profile+12,retained->gyroGsens,36);
    memcpy(profile+48,retained->earthCalBody,12);profile[60]=retained->earthCalValid;
    profile[61]=retained->earthUpAxis;profile[62]=sensor_tcal_get_auto_calibration();
    profile[63]=retained->tcal_enabled;
    irq_unlock(key);
    if(!append(2,profile,sizeof(profile)))return false;
    size_t space=sizeof(snapshot)-snapshot_size-4;
    size_t n=sensor_tcal_wire_snapshot(snapshot+snapshot_size+4,space);
    if(!n)return false;
    thermal_crc=cw_crc(snapshot+snapshot_size+4,n);
    cw_put16(snapshot+snapshot_size,3);cw_put16(snapshot+snapshot_size+2,n);snapshot_size+=4+n;
    /* Both mirrors are exported verbatim; host verifies CRC/reference/generation.
     * Archive namespace is strictly read-only here, never flash/firmware slots. */
    for(unsigned id=100;id<=109;id++){
        int size=sys_arc_read(id,scratch,1900);
        if(size==-ENOENT)continue;
        if(size<0||size>sizeof(scratch)||!append(id,scratch,size))return false;
    }
    cw_put32(begin_reply,snapshot_size);cw_put32(begin_reply+4,cw_crc(snapshot,snapshot_size));
    memset(staged,0,sizeof(staged));memset(present,0,sizeof(present));
    return true;
}
static uint8_t handle(const uint8_t *p,uint8_t *out,uint8_t *length){
    unsigned op=p[3],offset=cw_u16(p+14),n=p[16];uint32_t token=cw_u32(p+4);
    *length=0;
    if(op==0){
        if(n||offset||token==last_session)return 1;
        if(calwire_active()&&token!=session)return 2;
        if(!calwire_active()){
            if(!idle())return 2;
            session=token;atomic_set(&active,1);
            if(!take_snapshot()){last_session=session;atomic_clear(&active);return 3;}
        }
        expires=k_uptime_get_32()+60000;
        memcpy(out,begin_reply,8);*length=8;return 0;
    }
    if(op==5&&token==last_session&&!n&&!offset)return 0;
    if(!calwire_active()||session!=token)return 4;
    expires=k_uptime_get_32()+60000;
    if(op==1){
        if(n||offset>=snapshot_size)return 1;
        *length=MIN(CW_RESPONSE_DATA,snapshot_size-offset);memcpy(out,snapshot+offset,*length);return 0;
    }
    if(op==2){
        if(!n||offset+n>sizeof(staged))return 1;
        /* Explicit replacement is allowed; APPLY always checks entire field CRC. */
        memcpy(staged+offset,p+18,n);memset(present+offset,1,n);return 0;
    }
    if(op==5){
        if(n||offset)return 1;
        last_session=session;atomic_clear(&active);return 0;
    }
    unsigned size=cw_field_size(offset),start=cw_field_offset(offset);
    if(!size||n!=12)return 1;
    for(unsigned j=start;j<start+size;j++)if(!present[j])return 5;
    if(!cw_profile_valid(staged+start,offset)||cw_u32(p+22)!=cw_crc(staged+start,size)||
       cw_u32(p+26)!=thermal_crc)return 5;
    if(!idle())return 2;
    size_t thermal_size=sensor_tcal_wire_snapshot(scratch,sizeof(scratch));
    if(!thermal_size||cw_crc(scratch,thermal_size)!=thermal_crc)return 6;
    unsigned key=irq_lock();memcpy(apply_before,field_ptr(offset),size);irq_unlock(key);
    /* A duplicate successful APPLY returns success without rewriting flash. */
    if(!memcmp(apply_before,staged+start,size)){memcpy(out,apply_before,MIN(size,CW_RESPONSE_DATA));*length=MIN(size,CW_RESPONSE_DATA);return 0;}
    if(cw_crc(apply_before,size)!=cw_u32(p+18))return 6;
    if(op==3)return 0;
    memcpy(apply_bytes,staged+start,size);apply_field=offset;
    atomic_set(&apply_state,1);
    uint32_t deadline=k_uptime_get_32()+4000;
    while(atomic_get(&apply_state)<3&&(int32_t)(k_uptime_get_32()-deadline)<0)k_msleep(5);
    if(atomic_cas(&apply_state,1,0))return 7; /* cancelled before execution */
    /* Never time out a commit already in progress; observe its actual result. */
    while(atomic_get(&apply_state)==2)k_msleep(5);
    bool ok=atomic_get(&apply_state)==3;atomic_set(&apply_state,0);
    if(ok){cw_put32(out,cw_crc(field_ptr(offset),size));*length=4;}
    return ok?0:8;
}
static void worker(void){
    uint8_t req[48],rsp[48];
    while(true){
        if(k_msgq_get(&cw_queue,req,K_MSEC(50))){
            if(calwire_active()&&(int32_t)(k_uptime_get_32()-expires)>=0){last_session=session;atomic_clear(&active);}
            /* Dedicated bounded poll while an explicit session is open.
             * Resting devices deliberately slow ordinary PINGs to seconds.
             * No polls survive END/lease expiry and no power policy changes. */
            if(calwire_active()&&esb_ready()&&!arc8_ota_active()){
                memset(rsp,0,sizeof(rsp));rsp[0]=0x27;rsp[1]=esb_get_tracker_id();rsp[2]=1;rsp[3]=1;
                cw_put32(rsp+4,session);uint64_t addr=(*(uint64_t*)NRF_FICR->DEVICEADDR)&0xffffffffffffULL;
                memcpy(rsp+8,&addr,6);cw_put16(rsp+14,65535);rsp[16]=4;
                cw_put32(rsp+44,cw_crc(rsp,44));esb_write(rsp,false,sizeof(rsp));
            }
            continue;
        }
        memcpy(rsp,req,18);memset(rsp+18,0,30);rsp[0]=0x27;
        rsp[17]=handle(req,rsp+22,rsp+16);memcpy(rsp+18,req+44,4);rsp[16]+=4;
        cw_put32(rsp+44,cw_crc(rsp,44));
        esb_write(rsp,false,sizeof(rsp));
    }
}
K_THREAD_DEFINE(calwire_thread,3072,worker,NULL,NULL,NULL,7,0,0);

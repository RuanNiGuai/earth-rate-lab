"""Execute production transfer/commit admission logic with fake storage/clock.
This does not substitute for radio, flash or reboot verification on hardware.
"""
import os
from pathlib import Path
import subprocess
import unittest

ROOT=Path(__file__).resolve().parents[2]
def function(src,name):
    start=src.index(name+'(');start=src.rfind('\n',0,start)+1
    pos=src.index('{',start);level=1;end=pos+1
    while level:
        if src[end]=='{':level+=1
        if src[end]=='}':level-=1
        end+=1
    return src[start:end]

class CalwireTests(unittest.TestCase):
    def test_production_admission_and_idempotent_commits(self):
        src=(ROOT/'integration/tracker/src/sensor/calwire.c').read_text()
        prefix=r'''
#include <assert.h>
#include <stdio.h>
#include "sensor/calwire_protocol.h"
#define MIN(a,b) ((a)<(b)?(a):(b))
typedef int atomic_t;
static atomic_t active,apply_state;
static uint8_t snapshot[64],scratch[4096],staged[60],present[60],apply_bytes[36],apply_before[36];
static unsigned apply_field,writes;
static uint32_t session,last_session,expires,thermal_crc;
static size_t snapshot_size=64;
static uint8_t begin_reply[8];
static float scales[3]={1,1,1},gsens[9],earth[3]={.0041780746f,0,0};
static int resting=1,is_idle=1;
static uint32_t clock_ms;
static unsigned irq_lock(void){return 0;}
static void irq_unlock(unsigned k){(void)k;}
static int atomic_get(int *p){return *p;}
static void atomic_set(int *p,int v){*p=v;}
static void atomic_clear(int *p){*p=0;}
static bool atomic_cas(int *p,int old,int value){if(*p!=old)return false;*p=value;return true;}
static bool calwire_active(void){return active;}
static bool idle(void){return is_idle;}
static const float *field_ptr(unsigned f){return f==1?scales:f==2?gsens:earth;}
static bool sys_gyro_scale_save(const float *p){memcpy(scales,p,12);writes++;return true;}
static bool sys_gyro_gsens_save(const float *p){memcpy(gsens,p,36);writes++;return true;}
static bool sys_earth_reference_save(const float *p){memcpy(earth,p,12);writes++;return true;}
static size_t sensor_tcal_wire_snapshot(uint8_t *p,size_t n){assert(n>=4);memcpy(p,"TEST",4);return 4;}
static bool take_snapshot(void){thermal_crc=cw_crc("TEST",4);return true;}
static uint32_t k_uptime_get_32(void){return clock_ms;}
void calwire_sensor_poll(bool rest);
static void k_msleep(int ms){clock_ms+=ms;calwire_sensor_poll(resting);}
'''
        suffix=r'''
static uint8_t req[48],out[26],len;
static int call(unsigned op,unsigned off,unsigned n){req[3]=op;cw_put16(req+14,off);req[16]=n;return handle(req,out,&len);}
int main(void){
    cw_put32(req+4,123);assert(call(0,0,0)==0);assert(active);
    assert(call(1,64,0)==1);assert(call(1,0,0)==0&&len==22);
    assert(call(2,59,2)==1);
    assert(call(4,1,12)==5); /* Missing staged bytes. */
    float next[3]={1.01f,1.02f,.99f};memcpy(req+18,next,12);assert(call(2,0,12)==0);
    cw_put32(req+18,1);cw_put32(req+22,cw_crc(next,12));cw_put32(req+26,thermal_crc);
    assert(call(4,1,12)==6&&writes==0); /* stale backup */
    cw_put32(req+18,cw_crc(scales,12));
    assert(call(3,1,12)==0&&writes==0); /* dry-run cannot commit */
    resting=0;assert(call(4,1,12)==8&&writes==0);
    resting=1;assert(call(4,1,12)==0&&writes==1);
    assert(!memcmp(scales,next,12));assert(call(4,1,12)==0&&writes==1);
    cw_put32(req+26,1);assert(call(4,1,12)==5&&writes==1);
    cw_put32(req+4,999);assert(call(4,1,12)==4);
    cw_put32(req+4,123);assert(call(5,0,0)==0&&!active);assert(call(5,0,0)==0);
    assert(call(0,0,0)==1); /* closed token cannot restart a snapshot */
    uint8_t p[48]={0x26,2,1,0};cw_put32(p+4,1);cw_put32(p+44,cw_crc(p,44));
    assert(cw_valid(p,48,0x26));for(unsigned i=0;i<48;i++){p[i]^=1;assert(!cw_valid(p,48,0x26));p[i]^=1;}
    puts("CALWIRE native bounds, stale CAS, rest gate, dry run, replay and session tests passed");
}
'''
        out=ROOT/'integration/native-build';out.mkdir(exist_ok=True)
        path=out/'calwire-test.c';path.write_text(prefix+function(src,'calwire_sensor_poll')+'\n'+function(src,'handle')+'\n'+suffix)
        import sys
        sys.path.insert(0, str(ROOT/'tools'))
        from native_test import compile_run
        compile_run(path.read_text(), out, [ROOT/'integration/tracker/src'])
    def test_shared_protocol_identical(self):
        self.assertEqual((ROOT/'integration/tracker/src/sensor/calwire_protocol.h').read_bytes(),(ROOT/'integration/receiver/src/calwire_protocol.h').read_bytes())

if __name__=='__main__':unittest.main()

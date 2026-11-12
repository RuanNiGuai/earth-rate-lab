"""Execute production C for temperature admission and VQF numeric invariants."""
from pathlib import Path
import re
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from native_test import compile_run


def function(source, name):
    match = re.search(r'^(?:static )?(?:inline )?\w+ ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    if not match:
        raise ValueError(name)
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end] + '\n'


class NumericTests(unittest.TestCase):
    def run_c(self, code, include=()):
        build = ROOT / '.local/native-tests'
        build.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=build) as tmp:
            compile_run(code, tmp, include)

    def test_temperature_confirmation_and_freshness(self):
        self.run_c(r'''
#include <assert.h>
#include "temperature_guard.h"
int main(void) {
    temperature_guard_t s={0};
    assert(!temperature_guard_update(&s,25,0));
    assert(isnan(temperature_guard_collection(&s,0)));
    assert(!temperature_guard_update(&s,25,50));
    assert(temperature_guard_update(&s,25,100));
    assert(temperature_guard_compensation(&s,100)==25);
    // Single plausible glitch never updates either the filter or timestamp.
    assert(!temperature_guard_update(&s,18.375f,110));
    assert(s.accepted_ms==100 && s.filtered==25);
    assert(isnan(temperature_guard_collection(&s,110)));
    assert(temperature_guard_compensation(&s,110)==25);
    assert(temperature_guard_update(&s,25.05f,120));
    // Genuine step: >=3 readings spanning >=100 ms, entire span <=0.2 C.
    assert(!temperature_guard_update(&s,35,130));
    assert(!temperature_guard_update(&s,35.05f,140));
    assert(!temperature_guard_update(&s,35.1f,150));
    assert(temperature_guard_update(&s,35,230));
    assert(s.filtered==35 && temperature_guard_collection(&s,230)==35);
    // Repeated hardware-invalid values are never promoted.
    float invalid[]={NAN,INFINITY,-231,-20,60};
    int64_t t=240;
    for(unsigned j=0;j<sizeof(invalid)/sizeof(invalid[0]);j++) {
        for(int i=0;i<4;i++,t+=50) assert(!temperature_guard_update(&s,invalid[j],t));
        assert(s.accepted==35);
    }
    assert(isnan(temperature_guard_compensation(&s,t+1000)));
    // Inconsistent candidates, duplicate timestamps and long gaps reset evidence.
    assert(!temperature_guard_update(&s,30,2000));
    assert(!temperature_guard_update(&s,30,2050));
    assert(!temperature_guard_update(&s,31,2100));
    assert(!temperature_guard_update(&s,31,2100));
    assert(!temperature_guard_update(&s,31,2150));
    assert(!temperature_guard_update(&s,31,2200));
    assert(temperature_guard_update(&s,31,2250));
    assert(!temperature_guard_update(&s,40,2300));
    assert(!temperature_guard_update(&s,40,2350));
    assert(!temperature_guard_update(&s,40,5000));
    assert(!temperature_guard_update(&s,40,5050));
    assert(temperature_guard_update(&s,40,5100));
    // Slowly ramping readings remain available, and collection uses raw accepted
    // temperature rather than the delayed compensation low-pass output.
    for(int i=1;i<=100;i++) assert(temperature_guard_update(&s,40+i*.01f,5100+i*10));
    assert(s.accepted==41 && s.filtered<41);
    assert(temperature_guard_collection(&s,6100)==41);
    assert(isnan(temperature_guard_compensation(&s,5000)));
    return 0;
}
''', [ROOT / 'integration/tracker/src/sensor'])

    def test_icm_temperature_and_header_contract(self):
        src = (ROOT / 'integration/tracker/src/sensor/imu/ICM45686.c').read_text(encoding='utf8')
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#define PACKET_SIZE 20
#define SENSOR_INTERFACE_DEV_IMU 0
#define ICM45686_TEMP_DATA1_UI 1
#define LOG_ERR(...) ((void)0)
static float fifo_temp;
static bool fifo_temp_valid;
static int bus_error, bus_reads;
static int16_t reg_temp;
static int ssi_burst_read(int dev,int reg,uint8_t *p,int n) {
    (void)dev;(void)reg;assert(n==2);bus_reads++;
    if(bus_error) return -1; // Output deliberately left uninitialized on failure.
    p[0]=(uint16_t)reg_temp>>8;p[1]=reg_temp&255;return 0;
}
'''
        code += ''.join(function(src, n) for n in ('icm45_hires_header', 'icm45_cache_fifo_temp', 'icm45_temp_read'))
        code += r'''
int main(void) {
    uint8_t p[320]={0};
    for(int i=0;i<16;i++){p[20*i]=0x78;p[20*i+13]=5;}
    p[13]=0x80; // one sentinel alongside 15 valid 35 C samples
    icm45_cache_fifo_temp(p,16);
    assert(fifo_temp_valid && icm45_temp_read()==35 && !bus_reads);
    for(int h=0;h<256;h++){
        p[0]=h;p[13]=5;icm45_cache_fifo_temp(p,1);
        assert(fifo_temp_valid==(h>=0x78 && h<=0x7b));
    }
    p[0]=0x78;p[13]=0x80;icm45_cache_fifo_temp(p,1);
    assert(!fifo_temp_valid);
    bus_error=1;assert(isnan(icm45_temp_read()));
    bus_error=0;reg_temp=INT16_MIN;assert(isnan(icm45_temp_read()));
    reg_temp=1280;assert(icm45_temp_read()==35);
    reg_temp=-3200;assert(icm45_temp_read()==0); // 0 C is valid, not a sentinel.
    icm45_cache_fifo_temp(p,0);assert(!fifo_temp_valid);
    return 0;
}
'''
        self.run_c(code)

    def test_vqf_continuity_inversion_covariance_and_gate(self):
        self.run_c(r'''
#include <assert.h>
#include "vqf.c"
int main(void) {
    vqf_double_t bo[3]={.1f,.2f,.1f},ao[2]={-.8f,.2f};
    vqf_double_t bn[3]={.2f,.4f,.2f},an[2]={-.4f,.2f},d[2];
    float one[1]={1};
    filterInitialState(1,bo,ao,d);
    filterAdaptStateForCoeffChange(one,1,bo,ao,bn,an,d);
    for(int i=0;i<100;i++) assert(fabsf(filterStep(1,bn,an,d)-1)<1e-5f);
    // Near-inverted gravity must be rotated toward +Z using the actual axis.
    for(int k=0;k<100;k++) {
        float phi=k*.063f,a[3]={1e-4f*cosf(phi),1e-4f*sinf(phi),-1},q[4],out[3];
        normalize(a,3);inclinationCorrection(a,q);normalize(q,4);quatRotate(q,a,out);
        assert(q[1]*sinf(phi)-q[2]*cosf(phi)>.999f);
        assert(isfinite(out[2]) && out[2]>.99999f);
        assert(fabsf(out[0])<.0002f && fabsf(out[1])<.0002f);
    }
    vqf_params_t p;vqf_state_t s;vqf_coeffs_t c;
    init_params(&p);initVqf(&p,&s,&c,.01f,.01f,.01f);
    for(int i=0;i<60000;i++) {
        float g[3]={.02f*sinf(i*.01f),.015f*cosf(i*.013f),.003f};
        float a[3]={.1f*sinf(i*.007f),.2f*cosf(i*.01f),9.80665f};
        updateGyr(&p,&s,&c,g);updateAccGated(&p,&s,&c,a,true);
        assert(s.biasP[1]==s.biasP[3] && s.biasP[2]==s.biasP[6] && s.biasP[5]==s.biasP[7]);
        for(int j=0;j<3;j++)assert(isfinite(s.biasP[j*4]) && s.biasP[j*4]>=0);
        assert(fabsf(norm(s.accQuat,4)-1)<1e-5f);
    }
    float before[4],after[4],bias[3],bad[3]={3,4,7};
    getQuat6D(&s,before);memcpy(bias,s.bias,sizeof(bias));
    for(int i=0;i<100;i++)updateAccGated(&p,&s,&c,bad,false);
    getQuat6D(&s,after);
    assert(!memcmp(before,after,sizeof(before)) && !memcmp(bias,s.bias,sizeof(bias)));
    assert(!s.restDetected);
    return 0;
}
''', [ROOT / 'integration/tracker/vqf-c/src'])

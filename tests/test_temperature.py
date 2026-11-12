"""Run production temperature-window and point storage code without hardware."""
from pathlib import Path
import re
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from native_test import compile_run


def function(source, name):
    match = re.search(r'^(?:static )?(?:void|bool) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end] + '\n'


class TemperatureTests(unittest.TestCase):
    def test_collection_requires_explicit_opt_in_each_boot(self):
        src = (ROOT / 'integration/tracker/src/sensor/calibration.c').read_text(encoding='utf8')
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#define printk(...) ((void)0)
#define CONFIG_SENSOR_POLY_TEMP_MIN 15
#define CONFIG_SENSOR_POLY_TEMP_MAX 43
#define TCAL_BUFFER_SIZE 140
#define TCAL_VERIFY_INTERVAL_MS 60000
#define SYS_STATUS_USB_CONNECTED 1
typedef int atomic_t;
typedef int tcal_action_t;
enum {TCAL_ACT_NONE,TCAL_ACT_DEFER,TCAL_ACT_SKIP,TCAL_ACT_RUN};
static atomic_t field_tcal_request;
static int tcal_last_action;
static bool tcal_verify_mode, powered=true, earth, scale, frozen, persist_ok=true;
static int64_t now, tcal_verify_next_ms;
static unsigned persists;
static struct {int64_t verify_next_ms; unsigned defer_count,skip_count;} tcal_defer;
static struct {struct {unsigned count;} tempCalState; bool tcal_enabled;} store, *retained=&store;
static int atomic_set(int *p,int v){int old=*p;*p=v;return old;}
static bool calwire_active(void){return frozen;}
static bool sensor_earth_cal_active(void){return earth;}
static bool sensor_gyro_scale_active(void){return scale;}
static bool get_status(int s){(void)s;return powered;}
static int64_t k_uptime_get(void){return now;}
static bool vqf_boot_pose_valid(void){return true;}
static bool sensor_boot_zro_ready(void){return true;}
static bool sensor_tcal_coverage_ready(void){return false;}
static bool connection_get_data_collection(void){return false;}
static bool f04_summary_active(void){return false;}
static int tcal_defer_step(void *p,int64_t n,bool r,bool v){(void)p;(void)n;(void)r;(void)v;return TCAL_ACT_RUN;}
static bool tcal_persist_for_pause(void){persists++;return persist_ok;}
'''
        for name in ('tcal_user_off', 'tcal_auto_calibration_enabled'):
            code += re.search(r'static bool ' + name + r'[^;]*;', src)[0] + '\n'
        code += 'static void tcal_collection_apply(bool b){tcal_auto_calibration_enabled=b;}\n'
        for name in ('sensor_tcal_set_auto_calibration', 'sensor_tcal_temperature_watch',
                     'sensor_tcal_get_auto_calibration'):
            code += function(src, name)
        code += r'''
int main(void){
    // Populated NVS table and enabled compensation do not arm observation writes.
    store.tempCalState.count=80;store.tcal_enabled=true;
    for(now=0;now<3600000;now+=1000)sensor_tcal_temperature_watch(28,true);
    assert(!sensor_tcal_get_auto_calibration() && store.tcal_enabled);
    assert(store.tempCalState.count==80 && persists==0);
    sensor_tcal_set_auto_calibration(true);
    sensor_tcal_temperature_watch(28,true);
    assert(sensor_tcal_get_auto_calibration());
    sensor_tcal_set_auto_calibration(false);
    sensor_tcal_temperature_watch(28,true);
    assert(!sensor_tcal_get_auto_calibration() && persists==1);
    for(int i=0;i<4000;i++){now+=1000;sensor_tcal_temperature_watch(29,true);}
    assert(!sensor_tcal_get_auto_calibration() && store.tcal_enabled);
    earth=true;sensor_tcal_set_auto_calibration(true);sensor_tcal_temperature_watch(28,true);
    assert(!sensor_tcal_get_auto_calibration());earth=false;
    sensor_tcal_set_auto_calibration(true);sensor_tcal_temperature_watch(28,true);
    powered=false;sensor_tcal_temperature_watch(28,true);
    assert(!sensor_tcal_get_auto_calibration());
    return 0;
}
'''
        build = ROOT / '.local/native-tests'
        build.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=build) as tmp:
            compile_run(code, tmp)

    def test_real_collection_gates_and_measured_temperature(self):
        src = (ROOT / 'integration/tracker/src/sensor/calibration.c').read_text(encoding='utf8')
        defines = '\n'.join(x for x in src.splitlines() if x.startswith('#define TCAL_'))
        structs = ''
        for end in ('} tcal_accum;', '} tcal_pending_replace = {.idx = -1};'):
            finish = src.index(end) + len(end)
            structs += src[src.rfind('static struct {', 0, finish):finish] + '\n'
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#define CONFIG_SENSOR_POLY_TEMP_MIN 15
#define CONFIG_SENSOR_POLY_TEMP_MAX 43
#define CONFIG_SENSOR_POLY_STEPS_PER_DEGREE 5
#define TCAL_BUFFER_SIZE 140
#define TEMP_TO_IDX(t) ((int)(((t)-15)*5))
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define MAIN_GYRO_TEMP_ID 1
static int64_t now=1000, field_accel_time=1000, tcal_accum_last_commit_time;
static int64_t tcal_verify_next_ms;
static float field_accel[3]={1,0,0}, tcal_direction_ref_temp=NAN;
enum {TCAL_DIR_UNKNOWN,TCAL_DIR_RISING,TCAL_DIR_FALLING};
static int tcal_current_direction;
static bool tcal_auto_calibration_enabled=true, tcal_verify_mode;
static struct {int64_t verify_next_ms;} tcal_defer;
static struct {
    struct {float temp,bias[3];} tempCalPoints[140];
    struct {unsigned count;bool valid;} tempCalState;
    float gyroTemp;
} store, *retained=&store;
static int64_t k_uptime_get(void){return now;}
static bool connection_get_data_collection(void){return false;}
static bool f04_summary_active(void){return false;}
static bool tcal_defer_block_write(bool active){return active;}
static void update_tcal_state(void){}
static void sensor_tcal_cache_invalidate(void){}
static void tcal_defer_window_closed(void *p,int64_t t){(void)p;(void)t;}
static void tcal_collection_apply(bool b){tcal_auto_calibration_enabled=b;}
static void sys_write(int id,void *dst,const void *p,size_t n){(void)id;memcpy(dst,p,n);}
'''
        code += defines + '\n' + structs
        for name in ('tcal_accum_reset', 'tcal_save_point', 'tcal_accum_flush',
                     'tcal_at_reference_pose', 'sensor_tcal_feed_continuous_sample'):
            code += function(src, name)
        code += r'''
int main(void){
    float g[3]={.01f,.02f,.03f};
    sensor_tcal_feed_continuous_sample(g,25.03f,true);
    assert(tcal_accum.sample_count==1);
    field_accel[0]=0;field_accel[1]=1;
    sensor_tcal_feed_continuous_sample(g,25.03f,true);
    assert(!tcal_accum.active && tcal_accum.sample_count==0);
    field_accel[0]=1;field_accel[1]=0;
    sensor_tcal_feed_continuous_sample(g,25.03f,true);
    now+=201;
    sensor_tcal_feed_continuous_sample(g,25.03f,true);
    assert(!tcal_accum.active);
    field_accel_time=now+1;
    sensor_tcal_feed_continuous_sample(g,25.03f,true);
    assert(!tcal_accum.active);
    field_accel_time=now;
    sensor_tcal_feed_continuous_sample(g,25.03f,true);
    sensor_tcal_feed_continuous_sample(g,25.03f,false);
    assert(!tcal_accum.active);
    float bads[]={NAN,INFINITY,14.99f,43.0f};
    for(unsigned i=0;i<sizeof(bads)/sizeof(bads[0]);i++){
        sensor_tcal_feed_continuous_sample(g,25.03f,true);
        sensor_tcal_feed_continuous_sample(g,bads[i],true);
        assert(!tcal_accum.active);
    }
    g[0]=NAN;sensor_tcal_feed_continuous_sample(g,25.03f,true);
    assert(!tcal_accum.active);g[0]=.01f;
    for(int i=0;i<TCAL_ACCUM_MIN_SAMPLES;i++)sensor_tcal_feed_continuous_sample(g,25.03f,true);
    tcal_accum_flush();
    int idx=TEMP_TO_IDX(25.03f);
    assert(store.tempCalState.count==1);
    assert(fabsf(store.tempCalPoints[idx].temp-25.03f)<1e-5f);
    assert(fabsf(store.tempCalPoints[idx].bias[0]-.01f)<1e-6f);
    tcal_current_direction=TCAL_DIR_UNKNOWN;
    g[0]=.012f;tcal_save_point(idx,25.15f,g);
    float a=TCAL_HYSTERESIS_EMA_UNKNOWN;
    assert(fabsf(store.tempCalPoints[idx].temp-(a*25.15f+(1-a)*25.03f))<1e-5f);
    assert(fabsf(store.tempCalPoints[idx].bias[0]-(a*.012f+(1-a)*.01f))<1e-6f);
    assert(store.tempCalState.count==1);
    float prior=store.tempCalPoints[idx].temp;
    tcal_save_point(idx,26.0f,g);assert(store.tempCalPoints[idx].temp==prior);
    // One bad bias window cannot overwrite a stored temperature point.
    float prior_bias=store.tempCalPoints[idx].bias[0];
    g[0]=prior_bias+2*TCAL_POINT_REPLACE_THRESHOLD;
    tcal_save_point(idx,25.1f,g);
    assert(store.tempCalPoints[idx].bias[0]==prior_bias);
    // A disagreeing candidate restarts the confirmation count.
    g[0]+=.1f;tcal_save_point(idx,25.1f,g);
    assert(store.tempCalPoints[idx].bias[0]==prior_bias);
    tcal_save_point(idx,25.1f,g);
    assert(store.tempCalPoints[idx].bias[0]==prior_bias);
    tcal_save_point(idx,25.1f,g);
    assert(store.tempCalPoints[idx].bias[0]==g[0]);
    return 0;
}
'''
        build = ROOT / '.local/native-tests'
        build.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=build) as tmp:
            compile_run(code, tmp)

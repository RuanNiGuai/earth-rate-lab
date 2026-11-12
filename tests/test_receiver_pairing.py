"""Pairing capacity/retry behavior using the actual receiver parser."""
import unittest
import test_upstream_numeric as numeric
ROOT, function = numeric.ROOT, numeric.function


class PairingTests(unittest.TestCase):
    run_c = numeric.NumericTests.run_c

    def test_full_roster_and_existing_tracker(self):
        source = (ROOT / 'integration/receiver/src/connection/esb.c').read_text(encoding='utf8')
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#define MAX_TRACKERS 16
#define K_FOREVER 0
#define SYS_LED_PATTERN_ONESHOT_PROGRESS 1
#define SYS_LED_PRIORITY_HIGHEST 1
static int tracker_store_lock, additions, warnings;
static uint8_t stored_trackers=MAX_TRACKERS;
static uint64_t stored_tracker_addr[MAX_TRACKERS];
static int64_t now;
static int64_t k_uptime_get(void){return now;}
static void k_mutex_lock(int *p,int t){(void)p;(void)t;}
static void k_mutex_unlock(int *p){(void)p;}
static void set_led(int a,int b){(void)a;(void)b;}
static void warning(const char *fmt,...){(void)fmt;warnings++;}
#define LOG_WRN(...) warning(__VA_ARGS__)
#define LOG_ERR(...) ((void)0)
static bool tracker_roster_address_valid(uint64_t a){return a>0 && a<0xffffffffffffULL;}
static int esb_add_pair(uint64_t addr,bool checksum) {
    (void)checksum;assert(stored_trackers<MAX_TRACKERS);
    stored_tracker_addr[stored_trackers]=addr;additions++;return stored_trackers++;
}
static uint8_t crc8_ccitt(uint8_t crc,const uint8_t *p,size_t n) {
    while(n--){crc^=*p++;for(int b=0;b<8;b++)crc=(crc<<1)^((crc&128)?7:0);}
    return crc;
}
static void packet(uint8_t p[8],uint64_t addr){
    uint64_t raw=addr<<16;memcpy(p,&raw,8);p[0]=crc8_ccitt(7,p+2,6);if(!p[0])p[0]=8;
}
'''
        code += function(source, 'warn_pairing_storage_full') + function(source, 'esb_parse_pair')
        code += r'''
int main(void){
    for(int i=0;i<MAX_TRACKERS;i++)stored_tracker_addr[i]=100+i;
    uint8_t p[8];packet(p,999);
    assert(!esb_parse_pair(p) && warnings==1 && additions==0);
    for(now=1;now<5000;now+=100)assert(!esb_parse_pair(p));
    assert(warnings==1);
    now=5000;assert(!esb_parse_pair(p) && warnings==2);
    packet(p,115);assert(esb_parse_pair(p) && additions==0 && warnings==2);
    p[0]^=128;assert(!esb_parse_pair(p) && additions==0 && warnings==2);
    stored_trackers=15;packet(p,999);assert(esb_parse_pair(p) && additions==1);
    assert(stored_trackers==16 && stored_tracker_addr[15]==999);
    return 0;
}
'''
        self.run_c(code)

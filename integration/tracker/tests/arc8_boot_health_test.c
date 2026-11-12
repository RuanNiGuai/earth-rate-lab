#include <assert.h>
#include "../src/system/arc8_boot_health.h"
int main(void)
{
    struct arc8_boot_health h = {0};
    for (uint32_t t=0; t<60000; t+=100) assert(!arc8_boot_health_step(&h,t,true,true));
    assert(arc8_boot_health_step(&h,60000,true,true));
    assert(!arc8_boot_health_step(&h,60100,false,true));
    assert(!arc8_boot_health_step(&h,60200,true,true));
    assert(!arc8_boot_health_step(&h,60300,true,false));
    assert(!arc8_boot_health_step(&h,60400,true,true));
    assert(!arc8_boot_health_step(&h,61000,true,true));
    assert(h.start==61000);
    h=(struct arc8_boot_health){0};
    uint32_t start=UINT32_MAX-30000U;
    for(uint32_t dt=0;dt<60000;dt+=100) assert(!arc8_boot_health_step(&h,start+dt,true,true));
    assert(arc8_boot_health_step(&h,start+60000U,true,true));
    return 0;
}

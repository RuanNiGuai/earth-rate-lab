/* ARC8-H3 synthetic fixture only: NEVER a genuine calibration observation. */
#include "cal_archive_core.h"
#include "cal_archive_wire.h"
#include <stdio.h>
#include <string.h>
#include <stddef.h>
static int emit(const char *line, void *arg) {
    (void)arg;
    return puts(line) == EOF ? -1 : 0;
}
#define INITIALIZE(v, num) do { \
    memset(&(v),0,sizeof(v)); (v).magic=CAL_ARC_MAGIC; \
    (v).version=CAL_ARC_VERSION; (v).reference_id=129; \
    (v).generation=(num); } while(0)
int main(void) {
    static cal_arc_tcal_t t;
    static cal_arc_scale_t s;
    static cal_arc_earth_t e;
    static cal_arc_gsens_t g;
    INITIALIZE(t,10); INITIALIZE(s,12); INITIALIZE(e,13); INITIALIZE(g,15);
    t.count=2; s.count=3; e.count=5; g.count=6;
    t.point[0].gyro[0]=0.0125f;
    t.point[1].temp=24.5f;
    s.axis[0].turns=20;
    e.stop[0].accel[2]=1.f;
    g.pose[0].quat[0]=1.f;
    t.checksum=cal_arc_checksum(&t,offsetof(cal_arc_tcal_t,checksum));
    s.checksum=cal_arc_checksum(&s,offsetof(cal_arc_scale_t,checksum));
    e.checksum=cal_arc_checksum(&e,offsetof(cal_arc_earth_t,checksum));
    g.checksum=cal_arc_checksum(&g,offsetof(cal_arc_gsens_t,checksum));
    if(arc8_wire_dump("tcal",&t,sizeof(t),129,10,emit,NULL) ||
       arc8_wire_dump("scale",&s,sizeof(s),129,12,emit,NULL) ||
       arc8_wire_dump("earth",&e,sizeof(e),129,13,emit,NULL) ||
       arc8_wire_dump("gsens",&g,sizeof(g),129,15,emit,NULL))return 2;
    puts("ARC8X1 DONE 129");
    return 0;
}

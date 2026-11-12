#include "cal_archive_core.h"
#include <stddef.h>
unsigned arc8_size_tcal(void) { return sizeof(cal_arc_tcal_t); }
unsigned arc8_size_scale(void) { return sizeof(cal_arc_scale_t); }
unsigned arc8_size_earth(void) { return sizeof(cal_arc_earth_t); }
unsigned arc8_size_gsens(void) { return sizeof(cal_arc_gsens_t); }
unsigned arc8_size_solution(void) { return sizeof(cal_arc_solution_t); }
unsigned arc8_magic(void) { return CAL_ARC_MAGIC; }
unsigned arc8_version(void) { return CAL_ARC_VERSION; }
unsigned arc8_checksum_offset_tcal(void) { return offsetof(cal_arc_tcal_t,checksum); }
unsigned arc8_checksum_offset_scale(void) { return offsetof(cal_arc_scale_t,checksum); }
unsigned arc8_checksum_offset_earth(void) { return offsetof(cal_arc_earth_t,checksum); }
unsigned arc8_checksum_offset_gsens(void) { return offsetof(cal_arc_gsens_t,checksum); }

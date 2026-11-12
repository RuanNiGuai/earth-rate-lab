#ifndef CALWIRE_H
#define CALWIRE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
bool calwire_active(void);
void calwire_receive(const uint8_t *data, size_t len);
void calwire_sensor_poll(bool rest);
/* Copy a complete immutable TCal observation/LUT snapshot; no setters. */
size_t sensor_tcal_wire_snapshot(uint8_t *out, size_t capacity);
bool cal_archive_wire_idle(void);
#endif

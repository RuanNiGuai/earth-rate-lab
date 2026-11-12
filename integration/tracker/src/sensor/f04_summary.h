#ifndef F04_SUMMARY_H
#define F04_SUMMARY_H
#include <stdbool.h>
#include <stdint.h>
#if defined(CONFIG_ARC8_F04_SUMMARY)
bool f04_summary_active(void);
bool f04_summary_begin(uint8_t tracker, uint8_t rate);
void f04_summary_end(void);
void f04_summary_feed(const float gyro[3], const float accel[3], float temp,
                       float dt_s, bool available, bool fresh);
void f04_summary_invalidate(void);
void f04_summary_drain(void);
#else
static inline bool f04_summary_active(void) { return false; }
#endif
#endif

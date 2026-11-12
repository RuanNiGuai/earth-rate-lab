/* ARC8-B: experimental MCUboot slot1-only ESB OTA, MIT license. */
#ifndef ARC8_ESB_OTA_H
#define ARC8_ESB_OTA_H
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#if defined(CONFIG_ARC8_ESB_OTA)
void arc8_ota_ack_received(const uint8_t *data, size_t len);
void arc8_ota_poll(void);
bool arc8_ota_active(void);
void arc8_ota_info(void);
void arc8_ota_abort(void);
void arc8_ota_console(const char *arg);
void arc8_ota_sensor_health(bool gyro, bool accel, bool valid);
#else
static inline void arc8_ota_ack_received(const uint8_t *d, size_t n) {(void)d;(void)n;}
static inline void arc8_ota_poll(void) {}
static inline bool arc8_ota_active(void) {return false;}
static inline void arc8_ota_info(void) {}
static inline void arc8_ota_abort(void) {}
static inline void arc8_ota_console(const char *a) {(void)a;}
static inline void arc8_ota_sensor_health(bool g, bool a, bool v) {(void)g;(void)a;(void)v;}
#endif
#endif

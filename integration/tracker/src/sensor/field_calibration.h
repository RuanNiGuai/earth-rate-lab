#ifndef FIELD_CALIBRATION_H
#define FIELD_CALIBRATION_H
#include <stdbool.h>
#include <stdint.h>

#define FIELD_EARTH_RATE_DPS 0.0041780746f

typedef struct {
    uint32_t version;
    float body[3]; /* deg/s, at the shared fixed boot pose */
    uint32_t checksum;
} field_reference_t;
bool field_reference_valid(const field_reference_t *record);
bool field_reference_make(field_reference_t *record, const float body[3]);
/* The shared reference pose is +X up. Latitude is encoded by the vector's elevation. */
float field_reference_latitude(const float body[3]);
bool field_reference_set_latitude(float body[3], float latitude_deg);

typedef struct {
    bool collecting, inhibited, sampled;
    int64_t stable_since, last_sample, cold_since;
    float low, high, last_temp;
    float noise;      /* EMA of the sample-to-sample change, deg C */
    float temp_lp;    /* low-passed temperature used for the stability span, deg C */
} field_temperature_t;
void field_temperature_manual(field_temperature_t *s, bool enabled);
/* Return 1=start, -1=stop, 0=no transition. */
int field_temperature_step(field_temperature_t *s, float temp, int64_t now);

enum field_earth_phase { FIELD_IDLE, FIELD_SETTLE, FIELD_SAMPLE, FIELD_TURN, FIELD_DONE, FIELD_FAILED };
typedef struct {
    enum field_earth_phase phase;
    int stop, spin, count, up_axis; /* up_axis: 0..5 = +X,-X,+Y,-Y,+Z,-Z */
    float angle, turn_bias, elapsed, settled, duration, first_temp;
    float rejects; /* time spent disturbed inside a sample window */
    float turn_idle; /* seconds spent waiting at a stop */
    float model_up, model_horizontal;
    float temp_low, temp_high, accel_low[3], accel_high[3];
    double sum[3], sumsq[3];
    float means[5][3], times[5], result[3], rms;
    const char *error;
} field_earth_t;
void field_earth_start(field_earth_t *s, float latitude_deg, int up_axis);
void field_earth_cancel(field_earth_t *s);
/* Gyro in deg/s, acceleration in g; dt is sensor sample interval, not task wall time. */
void field_earth_step(field_earth_t *s, const float g[3], const float a[3], float temp, float dt, bool rest);
#endif

#ifndef CALWIRE_RELAY_H
#define CALWIRE_RELAY_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
int calwire_relay_submit(const char *hex);
bool calwire_relay_ack(uint8_t slot,uint8_t out[48]);
void calwire_relay_receive(const uint8_t *p,size_t n);
#endif

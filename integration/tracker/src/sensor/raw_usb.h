#ifndef STYRIA_RAW_USB_H
#define STYRIA_RAW_USB_H

#include <stdbool.h>
#include <stdint.h>

/* Passive FIFO recording, before driver decoding or application calibration.
 * A session accepts a single sensor-thread producer. It has no fusion, sensor
 * register, power-management, or persistent-calibration side effects.
 *
 * RAWFIFO payload, little endian, exactly 40 bytes:
 *   0 u32 acquisition sequence (allocated BEFORE attempting queue insertion)
 *   4 u64 batch-read completion uptime in microseconds (NOT a sample timestamp)
 *  12 u16 index within the batch, 14 u16 packets read in the batch
 *  16 u32 cumulative queue drops before this record
 *  20 byte[20] unchanged ICM45686 high-resolution FIFO packet
 *
 * Output: RAWFIFO <80 lowercase hex digits> <8 lowercase CRC32-IEEE digits>\n
 * Sequence gaps also detect complete lines lost by the downstream console.
 */
bool raw_usb_start(uint32_t duration_seconds);
void raw_usb_stop(void);
void raw_usb_push(const uint8_t raw[20], uint64_t batch_us,
                  uint16_t index, uint16_t count);

/* True during start, acceptance, and drain/close. False only once the session
 * has emitted its final acknowledgement and can safely be started again. */
bool raw_usb_is_active(void);

#endif

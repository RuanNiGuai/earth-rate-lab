#include "raw_usb.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

/* Thread-context fifo_fill is intentionally restricted to this pinned CDC
 * driver: it uses a short lock around ring_buf_put and returns bytes accepted.
 * This is not a generic hardware UART transport. */
BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
             "RAWUSB requires the pinned USB CDC console driver");
static const struct device *const raw_usb_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#define RAW_USB_PAYLOAD_BYTES 40
#define RAW_USB_QUEUE_RECORDS 128
#define RAW_USB_MAX_SECONDS 900

K_MSGQ_DEFINE(raw_usb_queue, RAW_USB_PAYLOAD_BYTES, RAW_USB_QUEUE_RECORDS, 4);

enum raw_usb_state {
    RAW_USB_IDLE,
    RAW_USB_STARTING,
    RAW_USB_ACCEPTING,
    RAW_USB_DRAINING,
    RAW_USB_FINISHING,
};

static struct k_spinlock raw_usb_lock;
static enum raw_usb_state raw_usb_state;
static uint32_t raw_usb_next_seq;
static uint32_t raw_usb_dropped;
static uint32_t raw_usb_inflight;
static uint32_t raw_usb_tx_failed;
static int64_t raw_usb_deadline_ms;
static bool raw_usb_stop_requested;

bool raw_usb_start(uint32_t duration_seconds)
{
    if (duration_seconds == 0 || duration_seconds > RAW_USB_MAX_SECONDS)
        return false;
    if (!device_is_ready(raw_usb_uart)) return false;

    k_spinlock_key_t key = k_spin_lock(&raw_usb_lock);
    if (raw_usb_state != RAW_USB_IDLE) {
        k_spin_unlock(&raw_usb_lock, key);
        return false;
    }
    /* IDLE implies the previous producer and queue have both drained. */
    raw_usb_state = RAW_USB_STARTING;
    raw_usb_next_seq = 0;
    raw_usb_dropped = 0;
    raw_usb_inflight = 0;
    raw_usb_tx_failed = 0;
    raw_usb_stop_requested = false;
    k_spin_unlock(&raw_usb_lock, key);

    /* The first sensor record cannot be accepted until this is queued. */
    printk("RAWUSB enabled: Schema:1 Bytes:40 Duration:%u CRC32:IEEE\n",
           duration_seconds);

    key = k_spin_lock(&raw_usb_lock);
    raw_usb_deadline_ms = k_uptime_get() + (int64_t)duration_seconds * 1000;
    raw_usb_state = raw_usb_stop_requested ? RAW_USB_DRAINING : RAW_USB_ACCEPTING;
    k_spin_unlock(&raw_usb_lock, key);
    return true;
}

void raw_usb_stop(void)
{
    bool already_idle = false;
    uint32_t seq = 0, drops = 0, tx_failed = 0;
    k_spinlock_key_t key = k_spin_lock(&raw_usb_lock);
    if (raw_usb_state == RAW_USB_STARTING) {
        raw_usb_stop_requested = true;
    } else if (raw_usb_state == RAW_USB_ACCEPTING) {
        raw_usb_state = RAW_USB_DRAINING;
    } else if (raw_usb_state == RAW_USB_IDLE) {
        already_idle = true;
        seq = raw_usb_next_seq;
        drops = raw_usb_dropped;
        tx_failed = raw_usb_tx_failed;
    }
    k_spin_unlock(&raw_usb_lock, key);
    if (already_idle)
        printk("RAWUSB disabled: NextSeq:%u Dropped:%u TxFailed:%u\n", seq, drops, tx_failed);
}

bool raw_usb_is_active(void)
{
    k_spinlock_key_t key = k_spin_lock(&raw_usb_lock);
    bool active = raw_usb_state != RAW_USB_IDLE;
    k_spin_unlock(&raw_usb_lock, key);
    return active;
}

void raw_usb_push(const uint8_t raw[20], uint64_t batch_us,
                  uint16_t index, uint16_t count)
{
    uint8_t payload[RAW_USB_PAYLOAD_BYTES];
    uint32_t seq, drops;
    k_spinlock_key_t key = k_spin_lock(&raw_usb_lock);
    if (raw_usb_state != RAW_USB_ACCEPTING) {
        k_spin_unlock(&raw_usb_lock, key);
        return;
    }
    seq = raw_usb_next_seq++;
    drops = raw_usb_dropped;
    raw_usb_inflight++;
    k_spin_unlock(&raw_usb_lock, key);

    sys_put_le32(seq, payload);
    sys_put_le64(batch_us, payload + 4);
    sys_put_le16(index, payload + 12);
    sys_put_le16(count, payload + 14);
    sys_put_le32(drops, payload + 16);
    memcpy(payload + 20, raw, 20);

    /* Never print, wait for USB, or evict older records in the sensor thread.
     * Keep the inflight reference until the put completes so stop cannot ACK
     * an empty queue while this producer is still about to insert a record. */
    int err = k_msgq_put(&raw_usb_queue, payload, K_NO_WAIT);
    key = k_spin_lock(&raw_usb_lock);
    if (err)
        raw_usb_dropped++;
    raw_usb_inflight--;
    k_spin_unlock(&raw_usb_lock, key);
}

static void raw_usb_write_record(const uint8_t payload[RAW_USB_PAYLOAD_BYTES])
{
    static const char hex[] = "0123456789abcdef";
    char line[99];
    memcpy(line, "RAWFIFO ", 8);
    for (unsigned i = 0; i < RAW_USB_PAYLOAD_BYTES; i++) {
        line[8 + 2*i] = hex[payload[i] >> 4];
        line[9 + 2*i] = hex[payload[i] & 0x0f];
    }
    line[88] = ' ';
    uint32_t crc = crc32_ieee(payload, RAW_USB_PAYLOAD_BYTES);
    for (unsigned i = 0; i < 8; i++)
        line[89 + i] = hex[(crc >> (28 - 4*i)) & 0x0f];
    line[97] = '\n';
    line[98] = '\0';
    /* Bypass the lossy deferred logging queue. poll_out is also unsuitable:
     * this CDC configuration silently discards bytes when its TX ring is full.
     * fifo_fill reports accepted bytes; retry only in this low-priority worker.
     * Bound retries so a disconnected host cannot block this worker forever. */
    size_t sent=0;
    int64_t deadline=k_uptime_get()+100;
    while (sent<98) {
        int n=uart_fifo_fill(raw_usb_uart,(const uint8_t *)line+sent,98-sent);
        if (n>0) { sent+=(size_t)n; continue; }
        if (k_uptime_get()>=deadline) {
            k_spinlock_key_t key=k_spin_lock(&raw_usb_lock);
            raw_usb_tx_failed++;
            k_spin_unlock(&raw_usb_lock,key);
            break;
        }
        k_msleep(1);
    }
}

static void raw_usb_worker(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
    uint8_t payload[RAW_USB_PAYLOAD_BYTES];
    for (;;) {
        k_spinlock_key_t key = k_spin_lock(&raw_usb_lock);
        if (raw_usb_state == RAW_USB_ACCEPTING && k_uptime_get() >= raw_usb_deadline_ms)
            raw_usb_state = RAW_USB_DRAINING;
        k_spin_unlock(&raw_usb_lock, key);

        if (k_msgq_get(&raw_usb_queue, payload, K_MSEC(20)) == 0) {
            raw_usb_write_record(payload);
            continue;
        }

        /* Recheck queue occupancy: a producer could insert after the timed-out
         * get but before a concurrent stop. With acceptance disabled and no
         * inflight producer the count cannot increase. FINISHING excludes a
         * new start until the previous final acknowledgement is emitted. */
        bool finish = false;
        uint32_t seq = 0, drops = 0, tx_failed = 0;
        key = k_spin_lock(&raw_usb_lock);
        if (raw_usb_state == RAW_USB_DRAINING && raw_usb_inflight == 0 &&
            k_msgq_num_used_get(&raw_usb_queue) == 0) {
            raw_usb_state = RAW_USB_FINISHING;
            seq = raw_usb_next_seq;
            drops = raw_usb_dropped;
            tx_failed = raw_usb_tx_failed;
            finish = true;
        }
        k_spin_unlock(&raw_usb_lock, key);
        if (finish) {
            printk("RAWUSB disabled: NextSeq:%u Dropped:%u TxFailed:%u\n", seq, drops, tx_failed);
            key = k_spin_lock(&raw_usb_lock);
            raw_usb_state = RAW_USB_IDLE;
            k_spin_unlock(&raw_usb_lock, key);
        }
    }
}

K_THREAD_DEFINE(raw_usb_thread, 2048, raw_usb_worker, NULL, NULL, NULL, 11, 0, 0);

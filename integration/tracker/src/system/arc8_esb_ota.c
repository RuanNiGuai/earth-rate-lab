#include "sensor/f04_summary.h"
#include "sensor/calwire.h"
/*
 * ARC8-B staged ESB OTA for Styria / nRF52840, MIT License.
 * Design adapted from the upstream SlimeVR MCUboot slot1 approach.
 * ONLY the inactive MCUboot slot1 is ever erased or written.
 * Activation deliberately off by default: CONFIG_ARC8_ESB_OTA_ACTIVATE=n.
 * USB/UF2 recovery, signed-image boot chain and rollback require physical testing.
 */
#include <zephyr/kernel.h>
#include "arc8_esb_ota.h"
#if defined(CONFIG_ARC8_ESB_OTA)
#include <zephyr/storage/flash_map.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>
#include "globals.h"
#include "build_defines.h"
#include "arc8_fw_info48.h"
#include "connection/esb.h"
#include <zephyr/sys/atomic.h>
#include "arc8_boot_health.h"
#include "system.h"

#if !defined(CONFIG_BOOTLOADER_MCUBOOT) || !DT_NODE_EXISTS(DT_NODELABEL(slot1_partition))
#error "ARC8 OTA requires MCUboot and a real slot1_partition"
#endif
#if CONFIG_ESB_MAX_PAYLOAD_LENGTH < 48
#error "ARC8 OTA requires 48-byte ESB payloads"
#endif
#if FIXED_PARTITION_OFFSET(slot0_partition) != 0x0f000 || FIXED_PARTITION_SIZE(slot0_partition) != 0x6f000 || \
    FIXED_PARTITION_OFFSET(slot1_partition) != 0x7e000 || FIXED_PARTITION_SIZE(slot1_partition) != 0x70000 || \
    FIXED_PARTITION_OFFSET(storage_partition) != 0xee000 || FIXED_PARTITION_SIZE(storage_partition) != 0x6000 || \
    FIXED_PARTITION_OFFSET(uf2_partition) != 0xf4000
#error "ARC8 OTA flash map mismatch: refuse to build"
#endif

LOG_MODULE_REGISTER(arc8_ota, LOG_LEVEL_INF);

#define ARC8_PAYLOAD 44U
#define ARC8_BEGIN_LEN 32U
#define ARC8_BOARD "styria_mini_uf2/nrf52840/spi"
#define ARC8_PROTO 2U
#define OTA_DATA 0x20U
#define OTA_STATUS 0x21U
#define OTA_INFO 0x22U
#define OTA_BEGIN 0x23U
#define OTA_VERIFY 0x24U
#define OTA_ACTIVATE 0x25U
#define ST_IDLE 0U
#define ST_READY 1U
#define ST_RECEIVING 2U
#define ST_VERIFY_OK 3U
#define ST_VERIFY_FAIL 4U
#define ST_ACTIVATING 5U
#define ST_ERROR 0x10U
#define ST_BOARD 0x11U
#define ST_FLASH 0x12U
#define ST_SIZE 0x13U
#define ST_SEQ 0x14U

struct ota_message { uint8_t len; uint8_t data[48]; };
K_MSGQ_DEFINE(arc8_rxq, sizeof(struct ota_message), 12, 4);

struct ota_session {
    const struct flash_area *area;
    uint32_t image_start; /* relative to slot1 area: OFFSET SWAP second sector */
    uint32_t max_size;
    uint32_t size;
    uint32_t crc;
    uint32_t written;
    uint32_t started;
    uint32_t report_until;
    uint16_t packets;
    uint16_t next_seq;
    uint8_t status;
    bool active;
};
static struct ota_session ota;
static atomic_t sensor_health;
void arc8_ota_sensor_health(bool gyro, bool accel, bool valid)
{
    /* Only the sensor thread publishes; the OTA worker consumes atomically.
     * A bad packet dominates good packets within the same observation window. */
    atomic_or(&sensor_health, valid ? ((gyro ? 1 : 0) | (accel ? 2 : 0)) : 4);
}

static void close_area(void)
{
    if (ota.area) {
        flash_area_close(ota.area);
        ota.area = NULL;
    }
}

static void ota_set_status(uint8_t status)
{
    ota.status = status;
    ota.report_until = k_uptime_get_32() + 12000U;
}

static void fail(uint8_t why)
{
    ota_set_status(why);
    ota.active = false;
    close_area();
    LOG_ERR("ARC8 OTA stopped: status=0x%02x, received=%u", why, ota.written);
}

/* Never open arbitrary addresses. Only the named secondary flash area. */
static bool start_slot(void)
{
    if (!boot_is_img_confirmed()) {
        LOG_WRN("ARC8 OTA: current application not confirmed; use USB 'ota confirm' after health check");
        return false;
    }
    const uint8_t area_id = FIXED_PARTITION_ID(slot1_partition);
    if (flash_area_open(area_id, &ota.area)) return false;
    size_t start = boot_get_image_start_offset(area_id);
    ssize_t trailer = boot_get_area_trailer_status_offset(area_id);
    if (start != 0x1000U || trailer <= 0 ||
        (size_t)trailer > ota.area->fa_size ||
        (size_t)trailer <= start ||
        ota.area->fa_off != 0x7e000U || ota.area->fa_size != 0x70000U) {
        LOG_ERR("ARC8 OTA: slot1 layout mismatch: start=%u trailer=%d", (unsigned)start, (int)trailer);
        close_area();
        return false;
    }
    ota.image_start = (uint32_t)start;
    ota.max_size = (uint32_t)trailer - (uint32_t)start;
    return true;
}

static bool is_board(const uint8_t *buf)
{
    static const char board[] = ARC8_BOARD;
    /* 32-byte BEGIN: board prefix (17 chars) + NUL inside 17 bytes. */
    return memchr(buf + 13, 0, 17) != NULL &&
           strncmp((const char *)(buf + 13), board, 16) == 0;
}

static void begin(const struct ota_message *m)
{
    if (f04_summary_active() || calwire_active()) { fail(ST_ERROR); return; }
    const uint8_t *p = m->data;
    if (m->len != ARC8_BEGIN_LEN || p[12] != ARC8_PROTO || p[30] != 1 ||
        crc8_ccitt(0, p, 31) != p[31]) {
        fail(ST_ERROR); return;
    }
    if (!is_board(p)) { fail(ST_BOARD); return; }
    uint32_t size = sys_get_le32(&p[2]);
    uint32_t crc = sys_get_le32(&p[6]);
    uint16_t count = sys_get_be16(&p[10]);
    if (size < 0x240 || !count || count != ((size + ARC8_PAYLOAD - 1) / ARC8_PAYLOAD)) {
        fail(ST_SIZE); return;
    }
    if (ota.active) {
        /* BEGIN may be retransmitted; never erase a live transfer. */
        if (size == ota.size && crc == ota.crc && count == ota.packets) return;
        fail(ST_ERROR); return;
    }
    close_area();
    memset(&ota, 0, sizeof(ota));
    if (!start_slot()) { fail(ST_FLASH); return; }
    if (size > ota.max_size || size > 0x6b000U) { fail(ST_SIZE); return; }
    ota.size = size;
    ota.crc = crc;
    ota.packets = count;
    /* Erase ONLY the 448-KiB inactive secondary slot, including stale trailer.
     * Flash erasure is done in this dedicated worker, never in radio ISR. */
    if (flash_area_erase(ota.area, 0, ota.area->fa_size)) { fail(ST_FLASH); return; }
    ota.active = true;
    ota.started = k_uptime_get_32();
    ota_set_status(ST_READY);
    LOG_INF("ARC8 OTA ready: image=%u, capacity=%u, slot1 start=0x%x",
            size, ota.max_size, ota.image_start);
}

static bool header_ok(const uint8_t *p, size_t len)
{
    if (len < 32 || sys_get_le32(p) != 0x96f3b83dU) return false;
    uint16_t hdr = sys_get_le16(&p[8]);
    uint16_t prot = sys_get_le16(&p[10]);
    uint32_t img = sys_get_le32(&p[12]);
    return sys_get_le32(&p[4]) == 0U && hdr == 0x200U && img != 0 &&
           hdr + img + prot + 4 <= ota.size;
}

static void data_packet(const struct ota_message *m)
{
    if (!ota.active || (ota.status != ST_RECEIVING && ota.status != ST_READY)) return;
    if (m->len < 5) return;
    uint16_t seq = sys_get_be16(&m->data[2]);
    if (seq < ota.next_seq) return; /* ACK loss; duplicate was already committed. */
    if (seq != ota.next_seq) return; /* Receiver retransmits requested sequence. */
    uint32_t left = ota.size - ota.written;
    size_t n = left < ARC8_PAYLOAD ? left : ARC8_PAYLOAD;
    if (m->len != n + 4) { fail(ST_SEQ); return; }
    if (seq == 0 && !header_ok(&m->data[4], n)) { fail(ST_SIZE); return; }
    uint8_t aligned[ARC8_PAYLOAD];
    memset(aligned, 0xff, sizeof(aligned));
    memcpy(aligned, &m->data[4], n);
    /* nRF52840 write-block size is 4: all nonfinal packets use 44 bytes. */
    size_t write_len = (n + 3U) & ~3U;
    if (ota.written + write_len > ota.max_size ||
        flash_area_write(ota.area, ota.image_start + ota.written, aligned, write_len)) {
        fail(ST_FLASH); return;
    }
    ota.written += (uint32_t)n;
    ota.next_seq++;
    ota_set_status(ST_RECEIVING);
}

static void verify(void)
{
    if (!ota.active) return;
    if (ota.status == ST_VERIFY_OK) return;
    if (ota.written != ota.size || ota.next_seq != ota.packets) { fail(ST_SIZE); return; }
    uint8_t buffer[256];
    uint32_t crc = 0;
    for (uint32_t pos=0; pos < ota.size; pos += sizeof(buffer)) {
        size_t n = MIN(sizeof(buffer), ota.size-pos);
        if (flash_area_read(ota.area, ota.image_start + pos, buffer, n)) { fail(ST_FLASH); return; }
        crc = crc32_ieee_update(crc, buffer, n);
    }
    if (crc != ota.crc) { fail(ST_VERIFY_FAIL); return; }
    /* Sanity-check the *presence* of signed MCUboot TLVs. This is not a
     * cryptographic check; MCUboot performs that at TEST boot. */
    uint8_t h[32];
    if (flash_area_read(ota.area, ota.image_start, h, sizeof(h)) || !header_ok(h, sizeof(h))) {
        fail(ST_VERIFY_FAIL); return;
    }
    uint32_t tlv = sys_get_le16(&h[8]) + sys_get_le32(&h[12]) + sys_get_le16(&h[10]);
    uint8_t b[4];
    if (tlv + 4U > ota.size ||
        flash_area_read(ota.area, ota.image_start + tlv, b, 4) ||
        sys_get_le16(b) != 0x6907U) { fail(ST_VERIFY_FAIL); return; }
    uint16_t tlv_len = sys_get_le16(b + 2);
    if (tlv_len < 4 || tlv + tlv_len > ota.size) { fail(ST_VERIFY_FAIL); return; }
    bool hash_tlv = false, sig_tlv = false;
    uint32_t cursor = tlv + 4U;
    while (cursor + 4U <= tlv + tlv_len) {
        if (flash_area_read(ota.area, ota.image_start + cursor, b, 4)) {
            fail(ST_FLASH); return;
        }
        uint16_t len = sys_get_le16(b + 2);
        if (cursor + 4U + len > tlv + tlv_len) { fail(ST_VERIFY_FAIL); return; }
        hash_tlv |= b[0] == 0x10U;
        sig_tlv |= b[0] == 0x22U; /* ECDSA-P256 only */
        cursor += 4U + len;
    }
    if (cursor != tlv + tlv_len || !hash_tlv || !sig_tlv) {
        fail(ST_VERIFY_FAIL); return;
    }
    /* Signature authenticity is enforced by MCUboot when it boots, not CRC32. */
    ota_set_status(ST_VERIFY_OK);
    LOG_INF("ARC8 OTA CRC verified (signature remains MCUboot's responsibility)");
}

static void activate(void)
{
    if (!ota.active || ota.status != ST_VERIFY_OK) return;
#if defined(CONFIG_ARC8_ESB_OTA_ACTIVATE) && CONFIG_ARC8_ESB_OTA_ACTIVATE
    if (boot_request_upgrade(BOOT_UPGRADE_TEST) != 0) { fail(ST_FLASH); return; }
    ota_set_status(ST_ACTIVATING);
    close_area();
    /* Wait for the next explicit reboot from USB rather than auto-rebooting
     * inside this first experimental OTA integration. */
    ota.active = false;
    LOG_WRN("ARC8 OTA TEST swap requested; reboot manually after validation");
#else
    LOG_WRN("ARC8 OTA activation disabled at build time (verify-only stage)");
    ota_set_status(ST_VERIFY_OK);
#endif
}

static void handle(const struct ota_message *m)
{
    const uint8_t *p = m->data;
    if (m->len < 2 || p[1] != esb_get_tracker_id()) return;
    switch (p[0]) {
    case OTA_BEGIN: begin(m); break;
    case OTA_DATA: data_packet(m); break;
    case OTA_VERIFY: verify(); break;
    case OTA_ACTIVATE: activate(); break;
    case 0x26:
        ota.active = false;
        close_area();
        memset(&ota, 0, sizeof(ota));
        k_msgq_purge(&arc8_rxq);
        break;
    default: break;
    }
}

void arc8_ota_ack_received(const uint8_t *data, size_t len)
{
    if (len < 2 || len > 48 || data[1] != esb_get_tracker_id() ||
        (data[0] != OTA_BEGIN && data[0] != OTA_DATA &&
         data[0] != OTA_VERIFY && data[0] != OTA_ACTIVATE)) return;
    struct ota_message m = { .len = (uint8_t)len };
    memcpy(m.data, data, len);
    /* No blocking and no flash IO in the ESB radio IRQ. A dropped frame is
     * recovered by the next STATUS poll requesting the same sequence. */
    (void)k_msgq_put(&arc8_rxq, &m, K_NO_WAIT);
}

void arc8_ota_abort(void)
{
    /* Schedule cancellation on OTA worker: never close a flash_area from
     * ESB command thread while slot writes may still be in progress. */
    struct ota_message msg = {.len=2, .data={0x26, 0}};
    msg.data[1] = esb_get_tracker_id();
    (void)k_msgq_put(&arc8_rxq, &msg, K_NO_WAIT);
}

bool arc8_ota_active(void) { return ota.active; }

void arc8_ota_poll(void)
{
    if (!esb_ready()) return;
    uint32_t now = k_uptime_get_32();
    static uint32_t last;
    if ((uint32_t)(now-last) < (ota.active ? 12U : 100U)) return;
    if (!ota.active && (int32_t)(ota.report_until-now) <= 0) return;
    last = now;
    uint8_t pkt[9] = { OTA_STATUS, esb_get_tracker_id(), ota.status };
    sys_put_be16(ota.next_seq, &pkt[3]);
    sys_put_le32(ota.written, &pkt[5]);
    esb_write(pkt, false, sizeof(pkt));
}

void arc8_ota_info(void)
{
    /* 48B ESB packet, receiver splits into six 13-byte HID fragments.
     * Keep ABI 0x22/version/build/slot0-size/mcuboot/v2/board. */
    uint8_t info[ARC8_FW_INFO48_LENGTH];
    arc8_fw_info48_pack(info, esb_get_tracker_id(),
        FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH,
        BUILD_YEAR, BUILD_MONTH, BUILD_DAY, BUILD_HOUR, BUILD_MIN, BUILD_SEC,
        (uint32_t)FIXED_PARTITION_SIZE(slot0_partition), ARC8_BOARD);
    /* The 28-character board name ends at 42; 43 stays its NUL terminator.
     * Bit 0 marks this optional extension as present. Full RESETREAS is on USB. */
    BUILD_ASSERT(sizeof(ARC8_BOARD) <= 29, "Boot diagnostics must not overwrite board name");
    info[44] = sys_startup_diagnostic_flags();
    /* Optional confirmation extension in reserved bytes, old clients ignore it. */
    info[45] = 'C'; info[46] = boot_is_img_confirmed() ? 1 : 0;
    info[47] = (uint8_t)MIN(k_uptime_get() / 1000, 255);
    esb_write(info, false, sizeof(info));
}

void arc8_ota_console(const char *arg)
{
    if (!arg || strcmp(arg, "status") == 0) {
        printk("ARC8 OTA state=%u seq=%u bytes=%u active=%d enabled=%d\\n",
               ota.status, ota.next_seq, ota.written, ota.active,
               IS_ENABLED(CONFIG_ARC8_ESB_OTA_ACTIVATE));
        printk("app confirmed=%d; slot1-only, NVS/UF2 untouched\\n", boot_is_img_confirmed());
    } else if (!strcmp(arg, "confirm")) {
        if (ota.active) { printk("Cannot confirm during OTA session\\n"); return; }
        int err = boot_write_img_confirmed();
        printk("MCUboot confirm result=%d; inspect device health first\\n", err);
    } else if (!strcmp(arg, "abort")) {
        printk("Use receiver HID OTA abort for active sessions\\n");
    } else {
        printk("Usage: ota status|confirm\\n");
    }
}

static void ota_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    struct ota_message m;
    struct arc8_boot_health health = {0};
    bool needs_confirmation = !boot_is_img_confirmed();
    uint32_t health_check = 0;
    while (true) {
        if (k_msgq_get(&arc8_rxq, &m, K_MSEC(100)) == 0) {
            handle(&m);
        }
        if (ota.active && (uint32_t)(k_uptime_get_32()-ota.started) > 900000U) {
            fail(ST_ERROR);
        }
        uint32_t now = k_uptime_get_32();
        if (needs_confirmation && (uint32_t)(now-health_check) >= 100U) {
            health_check = now;
            atomic_val_t observations = atomic_set(&sensor_health, 0);
            if (arc8_boot_health_step(&health, now, observations == 3, !ota.active)) {
                int err = boot_write_img_confirmed();
                if (err == 0 && boot_is_img_confirmed()) {
                    needs_confirmation = false;
                    LOG_INF("MCUboot image confirmed after 60s continuous valid gyro/accel samples");
                } else {
                    health.tracking = false;
                    LOG_ERR("MCUboot health confirmation failed: %d", err);
                }
            }
        }
    }
}
K_THREAD_DEFINE(arc8_ota_thread, 3072, ota_thread, NULL, NULL, NULL, 8, 0, 0);
#endif /* CONFIG_ARC8_ESB_OTA */

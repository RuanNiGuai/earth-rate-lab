/* ARC8 S-05: 44-byte OTA DATA payload coverage (tracker ESB 48B frame: 4 header + 44 payload).
 *
 * The legacy cases in data_payload_cases.c model a 60-byte HID payload, which matches the
 * old 64-byte USB report path but NOT the current ESB relay: the tracker sends 48-byte ESB
 * packets whose DATA payload is 44 bytes. This case file exercises the same production
 * receiver_ota_process_hid() path with 44-byte payloads, including page crossings at 4096
 * and short tail packets.
 */
static void data44_report(uint16_t seq, size_t available, size_t needed)
{
    uint8_t report[64];
    memset(report, 0xa5, sizeof(report));
    report[0] = HID_OTA_DATA;
    report[1] = RECEIVER_OTA_ID;
    sys_put_be16(seq, report + 2);
    for (size_t i = 0; i < MIN(available, needed); i++)
        report[4 + i] = data_image_byte((uint32_t)seq * 44 + i);
    receiver_ota_process_hid(report, 4 + available);
}

/* Clean transfer of `size` bytes with 44-byte payloads, then abort. */
static void data44_transfer(uint32_t size)
{
    begin(size);
    assert(rcv_ota.state == RCV_OTA_READY);

    if (size == 0) {
        command(HID_OTA_ABORT);
        return;
    }

    for (uint16_t seq = 0; (uint32_t)seq * 44 < size; seq++) {
        size_t needed = MIN(44, size - (uint32_t)seq * 44);
        data44_report(seq, needed, needed);
        assert(rcv_ota.next_expected_seq == (uint16_t)(seq + 1));
        assert(rcv_ota.bytes_written == MIN(size, (uint32_t)(seq + 1) * 44));
        assert(rcv_ota.page_buf_offset < RCV_OTA_FLASH_PAGE_SIZE);
    }
    assert(rcv_ota.bytes_written == size);
    command(HID_OTA_ABORT);
    assert(rcv_ota.state == RCV_OTA_IDLE);
}

/* Duplicate and out-of-order handling, exercised while the transfer is still running. */
static void data44_ordering(void)
{
    begin(44 * 4);

    data44_report(0, 44, 44);
    assert(rcv_ota.bytes_written == 44);
    assert(rcv_ota.next_expected_seq == 1);

    /* Exact duplicate of seq 0 must not advance the writer. */
    data44_report(0, 44, 44);
    assert(rcv_ota.bytes_written == 44);
    assert(rcv_ota.next_expected_seq == 1);
    assert(rcv_ota.state == RCV_OTA_RECEIVING);

    /* seq 1 full payload is accepted. */
    data44_report(1, 44, 44);
    assert(rcv_ota.bytes_written == 88);
    assert(rcv_ota.next_expected_seq == 2);

    /* Gap (seq 3 while 2 is expected) must be rejected without advancing. */
    data44_report(3, 44, 44);
    assert(rcv_ota.state == RCV_OTA_ERROR);
    assert(rcv_ota.bytes_written == 88);
    assert(rcv_ota.next_expected_seq == 2);

    command(HID_OTA_ABORT);
    assert(rcv_ota.state == RCV_OTA_IDLE);
}

static void data44_payload_cases(void)
{
    /* 1: single short packet; 43/44/45: boundary around one payload;
     * 4096-44 / 4096 / 4096+44: page crossing; 8192: exactly two pages. */
    static const uint32_t sizes[] = {1, 43, 44, 45, 4096 - 44, 4096, 4096 + 44, 8192};
    for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++)
        data44_transfer(sizes[i]);
    data44_ordering();
}

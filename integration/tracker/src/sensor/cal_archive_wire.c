#include "cal_archive_wire.h"
#include "cal_archive_core.h"
#include <stdio.h>
#include <string.h>

/* ARC7 ABI is deliberately un-packed little-endian ARM float32.
 * Refuse silent layout changes until the PC decoder is updated together. */
_Static_assert(sizeof(cal_arc_tcal_t) == 840u, "ARC8X1 tcal ABI changed");
_Static_assert(sizeof(cal_arc_scale_t) == 300u, "ARC8X1 scale ABI changed");
_Static_assert(sizeof(cal_arc_earth_t) == 1936u, "ARC8X1 earth ABI changed");
_Static_assert(sizeof(cal_arc_gsens_t) == 2392u, "ARC8X1 gsens ABI changed");

/* 24 bytes / line bounds console bandwidth and avoids allocating large buffers. */
#define ARC8_WIRE_CHUNK 24u
static const char hex[] = "0123456789abcdef";

static int wire_dump(const char *prefix, const char *name, const void *blob, size_t size,
                   uint32_t reference_id, uint32_t generation,
                   arc8_wire_line_fn emit, void *ctx)
{
    if (!name || !blob || !emit || !reference_id || !generation ||
        size == 0 || size > 8192 || strlen(name) > 8) return -1;
    char line[128];
    uint32_t crc = cal_arc_checksum(blob, (unsigned)size);
    int n = snprintf(line, sizeof(line), "%s BEGIN %s %u %08x %u %u",
                     prefix, name, (unsigned)size, (unsigned)crc,
                     (unsigned)reference_id, (unsigned)generation);
    if (n < 0 || (size_t)n >= sizeof(line) || emit(line, ctx)) return -1;
    const uint8_t *p = blob;
    for (size_t at=0; at<size; at+=ARC8_WIRE_CHUNK) {
        char payload[ARC8_WIRE_CHUNK*2+1];
        size_t len=size-at < ARC8_WIRE_CHUNK ? size-at : ARC8_WIRE_CHUNK;
        for (size_t i=0; i<len; i++) {
            payload[i*2] = hex[p[at+i] >> 4];
            payload[i*2+1] = hex[p[at+i] & 15];
        }
        payload[len*2]=0;
        n=snprintf(line,sizeof(line),"%s DATA %s %u %s",
                   prefix,name,(unsigned)at,payload);
        if (n < 0 || (size_t)n >= sizeof(line) || emit(line,ctx)) return -1;
    }
    n=snprintf(line,sizeof(line),"%s END %s",prefix,name);
    if (n < 0 || (size_t)n >= sizeof(line) || emit(line,ctx)) return -1;
    return 0;
}

int arc8_wire_dump(const char *name, const void *blob, size_t size,
                   uint32_t reference_id, uint32_t generation,
                   arc8_wire_line_fn emit, void *ctx)
{
    return wire_dump("ARC8X1",name,blob,size,reference_id,generation,emit,ctx);
}

int arc8_wire_dump_tcal(const void *blob, uint32_t reference_id,
                        uint32_t generation, arc8_wire_line_fn emit, void *ctx)
{
    return wire_dump("ARC8T1","tcal",blob,sizeof(cal_arc_tcal_t),
                     reference_id,generation,emit,ctx);
}

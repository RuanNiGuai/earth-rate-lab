/* ARC8-H3 read-only console export framing; no Zephyr dependency. */
#ifndef ARC8_CAL_ARCHIVE_WIRE_H
#define ARC8_CAL_ARCHIVE_WIRE_H
#include <stddef.h>
#include <stdint.h>
/* Each emitted line is null-terminated, without a newline. */
typedef int (*arc8_wire_line_fn)(const char *line, void *ctx);
/* Returns 0 if every line emitted; negative if callback failed. */
int arc8_wire_dump(const char *name, const void *blob, size_t size,
                   uint32_t reference_id, uint32_t generation,
                   arc8_wire_line_fn emit, void *ctx);
/* Diagnostic raw TCal object (840 B), distinct from ARC2/ARC8P1 records.
 * ARC8T1 never certifies a complete archive and has no ARC8X1 DONE marker. */
int arc8_wire_dump_tcal(const void *blob, uint32_t reference_id,
                        uint32_t generation, arc8_wire_line_fn emit, void *ctx);
#endif

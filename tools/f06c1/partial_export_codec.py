#!/usr/bin/env python3
"""F-06C3 ARC8P1 v1 *partial diagnostic* codec (HOST ONLY).

The ARC8P1 prefix, field order, payload hash coverage and record bytes are
unchanged from the frozen v1 format. Decoding a frame does NOT authenticate
where its payload came from and must NEVER trigger a complete archive solver.

All real ARC2 Scale/Earth/GSens records are 584 B; TCal is a distinct raw
object and is NOT exported by the firmware draft. TCal frame parsing is retained
only for legacy test compatibility, without claiming a validated TCal object.
"""
from __future__ import annotations

import hashlib
import re
import struct
import sys
import zlib
from dataclasses import dataclass, field

KINDS = ('tcal', 'scale', 'earth', 'gsens')
LIMITS = {'tcal': 12, 'scale': 3, 'earth': 5, 'gsens': 6}
ARC2_KINDS = {'scale': 1, 'earth': 2, 'gsens': 3}
PREFIX = 'ARC8P1'
MAX_RECORD_BYTES = 2048
MAX_HEX_PER_LINE = 4096
MAX_LINES = 4096
MAX_FRAME_LINE_CHARS = 8192
RECORD_BYTES = 584
LAYERS = ('FRAME_PARSED', 'PAYLOAD_HASH_VERIFIED', 'RECORD_STRUCTURE_VERIFIED')
_SHA = re.compile(r'[0-9a-f]{64}\Z')


class PartialExportError(ValueError):
    """All invalid-wire and invalid-parameter cases use this exception."""


@dataclass
class PartialFrame:
    kind: str
    reference_id: int
    limit: int
    records: list[bytes] = field(default_factory=list)
    # None means legacy contiguous slots 0..len(records)-1; decoder always
    # populates explicit original slot identifiers (including sparse slots).
    indices: list[int] | None = None
    archive_level: str = field(default='PARTIAL', init=False)
    validation_layer: str = field(default='FRAME_PARSED', init=False)

    def payload(self) -> bytes:
        return b''.join(self.records)

    def payload_sha256(self) -> str:
        """SHA-256 of record bytes in slot order; NOT a signature over metadata."""
        return hashlib.sha256(self.payload()).hexdigest()

    def sha256(self) -> str:  # historical alias
        return self.payload_sha256()

    def slot_indices(self) -> list[int]:
        return list(range(len(self.records))) if self.indices is None else list(self.indices)


def _uint(token: str, field_name: str) -> int:
    if not token or not token.isascii() or not token.isdecimal():
        raise PartialExportError(f'{field_name}: expected unsigned decimal integer')
    try:
        return int(token, 10)
    except (ValueError, OverflowError) as exc:
        raise PartialExportError(f'{field_name}: invalid integer') from exc


def _sha(token: str) -> str:
    if _SHA.fullmatch(token) is None:
        raise PartialExportError('SHA-256 must be 64 lowercase hex digits')
    return token


def _check_meta(kind: str, count: int, limit: int, ref_id: int) -> None:
    if kind not in KINDS:
        raise PartialExportError(f'unknown kind {kind!r}')
    if not (1 <= limit <= LIMITS[kind]):
        raise PartialExportError('declared limit exceeds dataset capacity or is zero')
    if count < 0 or count > limit:
        raise PartialExportError('count exceeds declared limit')
    if not (0 < ref_id <= 0xFFFFFFFF):
        raise PartialExportError('reference_id must be nonzero uint32')


def _check_slots(slots: list[int], limit: int) -> None:
    if slots != sorted(set(slots)):
        raise PartialExportError('slot indexes must be unique and in ascending order')
    if any(not (0 <= index < limit) for index in slots):
        raise PartialExportError('index outside 0..limit-1')


def encode(frame: PartialFrame, chunk: int = 32) -> list[str]:
    """Encode frozen v1 wire lines, preserving original sparse slot IDs."""
    _check_meta(frame.kind, len(frame.records), frame.limit, frame.reference_id)
    if not isinstance(chunk, int) or not (1 <= chunk <= min(MAX_RECORD_BYTES, MAX_HEX_PER_LINE // 2)):
        raise PartialExportError('invalid chunk size')
    indices = frame.slot_indices()
    if len(indices) != len(frame.records):
        raise PartialExportError('slot index count mismatch')
    _check_slots(indices, frame.limit)
    for record in frame.records:
        if len(record) > MAX_RECORD_BYTES:
            raise PartialExportError('record exceeds MAX_RECORD_BYTES')
        # The wire encoder does not implicitly certify ARC2 record integrity;
        # decode and verify_arc2_record_integrity handle the separate layers.
    digest = frame.payload_sha256()
    lines = [f'{PREFIX} BEGIN {frame.kind} {len(frame.records)} '
             f'{frame.limit} {frame.reference_id} {digest}']
    total = 0
    for index, record in zip(indices, frame.records):
        for offset in range(0, len(record), chunk):
            piece = record[offset:offset + chunk]
            lines.append(f'{PREFIX} DATA {frame.kind} {index} {offset} {piece.hex()}')
            total += len(piece)
    lines.append(f'{PREFIX} END {frame.kind} {total} {digest}')
    if len(lines) > MAX_LINES:
        raise PartialExportError('frame too large')
    return lines


def decode(lines: list[str]) -> PartialFrame:
    """Strict decode: line sequence, sparse slots, exact offsets, size, digest.

    ARC2 records must be exactly 584 bytes; CRC/magic/version/reference checks
    are a *separate* layer (verify_arc2_record_integrity). This decoder does
    not call a solver or create any candidate.
    """
    if len(lines) > MAX_LINES:
        raise PartialExportError('frame too large')
    frame: PartialFrame | None = None
    expected_count: int | None = None
    expected_digest: str | None = None
    received_end: str | None = None
    pending: dict[int, dict[int, bytes]] = {}
    received_bytes = 0

    for raw in lines:
        if len(raw) > MAX_FRAME_LINE_CHARS:
            raise PartialExportError('frame line too large')
        line = raw.strip()
        if not line.startswith(PREFIX):
            continue  # unrelated printk noise remains permitted
        words = line.split()
        if len(words) < 2 or words[0] != PREFIX:
            raise PartialExportError('bad frame line')
        command = words[1]
        if received_end is not None:
            raise PartialExportError('ARC8P1 line after END / duplicated END')
        if command == 'BEGIN':
            if frame is not None or len(words) != 7:
                raise PartialExportError('bad or duplicated BEGIN')
            _, _, name, count_s, limit_s, ref_s, digest = words
            count, limit, ref_id = (_uint(count_s, 'count'),
                                    _uint(limit_s, 'limit'),
                                    _uint(ref_s, 'reference_id'))
            _check_meta(name, count, limit, ref_id)
            frame = PartialFrame(name, ref_id, limit)
            expected_count = count
            expected_digest = _sha(digest)
        elif command == 'DATA':
            if frame is None or len(words) != 6:
                raise PartialExportError('DATA without BEGIN or bad DATA line')
            _, _, name, index_s, offset_s, hex_data = words
            if name != frame.kind:
                raise PartialExportError('DATA dataset mismatch')
            index, offset = _uint(index_s, 'index'), _uint(offset_s, 'offset')
            if index >= frame.limit:
                raise PartialExportError('index outside 0..limit-1')
            if len(hex_data) > MAX_HEX_PER_LINE:
                raise PartialExportError('DATA line too long')
            if not hex_data or len(hex_data) % 2:
                raise PartialExportError('bad hex length')
            try:
                blob = bytes.fromhex(hex_data)
            except ValueError as exc:
                raise PartialExportError(f'bad hex: {exc}') from exc
            blocks = pending.setdefault(index, {})
            if offset in blocks:
                raise PartialExportError('duplicate / overlapping offset for an index')
            if len(blocks) >= MAX_LINES or len(pending) > frame.limit:
                raise PartialExportError('frame resource limit')
            blocks[offset] = blob
            received_bytes += len(blob)
            if received_bytes > frame.limit * MAX_RECORD_BYTES:
                raise PartialExportError('frame exceeds byte capacity')
        elif command == 'END':
            if frame is None or len(words) != 5:
                raise PartialExportError('bad END')
            _, _, name, size_s, digest = words
            if name != frame.kind:
                raise PartialExportError('END dataset mismatch')
            # Deliberately postpone size comparison until offsets are verified
            # so a gap + recomputed bytes/SHA fails for the *right reason*.
            end_size = _uint(size_s, 'END bytes')
            received_end = _sha(digest)
        else:
            raise PartialExportError(f'unknown record type {command}')

    if frame is None or received_end is None:
        raise PartialExportError('incomplete frame')
    slots = sorted(pending)
    _check_slots(slots, frame.limit)
    for index in slots:
        parts = sorted(pending[index].items())
        expected_offset = 0
        blocks = []
        for offset, blob in parts:
            if offset != expected_offset:
                raise PartialExportError(
                    f'record index {index}: offset discontinuity '
                    f'(expected {expected_offset}, got {offset})')
            expected_offset += len(blob)
            if expected_offset > MAX_RECORD_BYTES:
                raise PartialExportError('record exceeds MAX_RECORD_BYTES')
            blocks.append(blob)
        record = b''.join(blocks)
        if frame.kind in ARC2_KINDS and len(record) != RECORD_BYTES:
            raise PartialExportError(
                f'record index {index}: record length {len(record)} != {RECORD_BYTES}')
        frame.records.append(record)
    if len(slots) != expected_count:
        raise PartialExportError('record count mismatch')
    frame.indices = slots
    if end_size != received_bytes or end_size != sum(len(r) for r in frame.records):
        raise PartialExportError('END size mismatch')
    if not (frame.payload_sha256() == expected_digest == received_end):
        raise PartialExportError('SHA-256 mismatch')
    frame.validation_layer = 'PAYLOAD_HASH_VERIFIED'
    return frame


def verify_record_structure(frame: PartialFrame,
                            expected_bytes: int = RECORD_BYTES,
                            checksum_ok=None) -> None:
    """Layer 3, compatible with the original optional checksum callback."""
    for record in frame.records:
        if len(record) != expected_bytes:
            raise PartialExportError(f'record is {len(record)} B, expected {expected_bytes} B')
        if checksum_ok is not None and not checksum_ok(record):
            raise PartialExportError('record checksum failed')
    # Length-only checks are not sufficient to certify record integrity.
    if checksum_ok is not None:
        frame.validation_layer = 'RECORD_STRUCTURE_VERIFIED'


def verify_arc2_record_integrity(frame: PartialFrame) -> None:
    """Independent host verification mirroring ARC2 magic/version/ref/kind/slot,
    generation!=0 and cal_arc_checksum CRC32 on byte prefix [0:580].
    For Scale/Earth/GSens only; not TCal and not a device-origin guarantee.
    """
    if frame.kind not in ARC2_KINDS:
        raise PartialExportError('TCal uses a different storage layout; unsupported')
    for index, rec in zip(frame.slot_indices(), frame.records):
        if len(rec) != RECORD_BYTES:
            raise PartialExportError('ARC2 record length mismatch')
        magic, version, gen, ref, _epoch = struct.unpack_from('<IIIII', rec)
        actual_crc, = struct.unpack_from('<I', rec, 580)
        if (magic != 0x41353252 or version != 2 or gen == 0 or
            ref == 0 or ref != frame.reference_id or
            rec[20] != ARC2_KINDS[frame.kind] or rec[21] != index):
            raise PartialExportError('ARC2 record identity mismatch')
        if actual_crc != zlib.crc32(rec[:580]):
            raise PartialExportError('ARC2 record CRC32 mismatch')
    frame.validation_layer = 'RECORD_STRUCTURE_VERIFIED'


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2
    try:
        if argv[1] == 'encode':
            if len(argv) < 5:
                raise PartialExportError('encode requires kind ref_id limit and files')
            kind, ref_s, limit_s, *files = argv[2:]
            frame = PartialFrame(kind, _uint(ref_s, 'ref_id'), _uint(limit_s, 'limit'),
                                 [open(f, 'rb').read() for f in files])
            print('\n'.join(encode(frame)))
            return 0
        if argv[1] == 'decode':
            if len(argv) != 3:
                raise PartialExportError('decode requires one filename')
            with open(argv[2], encoding='utf-8') as handle:
                frame = decode(handle.read().splitlines())
            print(f'{frame.kind}: {len(frame.records)} records, sha256 ok, PARTIAL')
            return 0
    except (OSError, PartialExportError) as exc:
        print(f'ARC8P1 ERROR: {exc}', file=sys.stderr)
        return 2
    print(__doc__)
    return 2


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))

#!/usr/bin/env python3
"""ARC8-H3 read-only ARC7 archive export: UART/RTT text -> verified H2 JSON.

No device configuration writes, no OTA, no calibration-parameter install.
"""
from __future__ import annotations
import argparse
import ctypes as C
import json
import re
import sys
import time
import zlib
from pathlib import Path

try:
    from . import calibration as h2
except ImportError:
    import calibration as h2

DATASETS = ('tcal', 'scale', 'earth', 'gsens')
MAX_TRANSFER_BYTES = 16000
MAX_CAPTURE_LINES = 3000
HEX = re.compile(r'^[0-9a-f]+$')


def fail(reason):
    raise ValueError(reason)


def to_python(obj):
    if isinstance(obj, C.Array):
        return [to_python(x) for x in obj]
    if isinstance(obj, C.Structure):
        return {name: to_python(getattr(obj, name)) for name, _ in obj._fields_}
    return obj


def decode_lines(lines):
    """Decode exactly four ARC8X1 datasets; ignore unrelated printk lines.

    A partial transfer, missing frame, conflicting reference, bad record CRC,
    interrupted sequence or duplicate fails closed. No capture file is emitted.
    """
    if sys.byteorder != 'little':
        fail('ARC7 binary decoder requires a little-endian host')
    result = {}
    active = None
    ref = None
    seen_done = False
    total_bytes = 0
    seen_first = False
    line_count = 0
    for raw_line in lines:
        line_count += 1
        if line_count > MAX_CAPTURE_LINES:
            fail('too many lines / timeout / console corruption')
        line = raw_line.decode('ascii', 'strict') if isinstance(raw_line, bytes) else raw_line
        line = line.strip()
        if len(line) > 320:
            fail('oversized console line')
        if not line.startswith('ARC8X1'):
            continue
        seen_first = True
        if seen_done:
            fail('second export after DONE; select a single session')
        words = line.split()
        if len(words) < 2 or words[0] != 'ARC8X1':
            fail('invalid export prefix')
        kind = words[1]
        if kind == 'ERROR':
            fail('tracker export error: ' + ' '.join(words[2:]))
        if kind == 'BEGIN':
            if len(words) != 7 or active is not None or len(result) >= len(DATASETS):
                fail('bad or overlapping BEGIN')
            _, _, name, length, crc_hex, reference, generation = words
            if name != DATASETS[len(result)]:
                fail('unexpected or duplicated dataset')
            expected_size = C.sizeof(h2.STRUCTS[name])
            if not length.isdecimal() or int(length) != expected_size:
                fail('archive record binary ABI size mismatch')
            if len(crc_hex) != 8 or not HEX.fullmatch(crc_hex):
                fail('invalid transport CRC')
            if not reference.isdecimal() or not generation.isdecimal():
                fail('bad reference/generation')
            reference, generation = int(reference), int(generation)
            if not 0 < reference < 2**32 or not 0 < generation < 2**32:
                fail('zero/out-of-range archive identity')
            if ref is None:
                ref = reference
            elif ref != reference:
                fail('mixed archive reference_id')
            total_bytes += expected_size
            if total_bytes > MAX_TRANSFER_BYTES:
                fail('archive exceeds allowed total size')
            active = dict(name=name, size=expected_size, crc=int(crc_hex, 16),
                          generation=generation, data=bytearray())
        elif kind == 'DATA':
            if len(words) != 5 or active is None:
                fail('DATA without BEGIN')
            name, offset, hex_data = words[2:]
            if name != active['name'] or not offset.isdecimal():
                fail('wrong dataset or offset format')
            if int(offset) != len(active['data']):
                fail('missing / duplicate / reordered DATA frame')
            if not (2 <= len(hex_data) <= 48 and len(hex_data) % 2 == 0 and
                    HEX.fullmatch(hex_data)):
                fail('invalid DATA payload')
            chunk = bytes.fromhex(hex_data)
            if len(active['data']) + len(chunk) > active['size']:
                fail('DATA overflow')
            active['data'].extend(chunk)
        elif kind == 'END':
            if len(words) != 3 or active is None or words[2] != active['name']:
                fail('invalid END')
            binary = bytes(active['data'])
            if len(binary) != active['size']:
                fail('incomplete dataset')
            if zlib.crc32(binary) != active['crc']:
                fail('transport CRC32 mismatch')
            typ = h2.STRUCTS[active['name']]
            struct_obj = typ.from_buffer_copy(binary)
            if (struct_obj.magic != 0x43414C31 or struct_obj.version != 2 or
                struct_obj.reference_id != ref or
                struct_obj.generation != active['generation']):
                fail('binary dataset metadata does not match wire header')
            if zlib.crc32(binary[:typ.checksum.offset]) != struct_obj.checksum:
                fail('inner ARC7 record CRC32 mismatch')
            result[active['name']] = to_python(struct_obj)
            active = None
        elif kind == 'DONE':
            if len(words) != 3 or not words[2].isdecimal() or int(words[2]) != ref:
                fail('incorrect DONE reference')
            if active is not None or tuple(result) != DATASETS:
                fail('premature DONE; required records missing')
            seen_done = True
        else:
            fail('unknown ARC8X1 frame')
    if not seen_done:
        fail('export missing DONE' if seen_first else 'no ARC8X1 export data found')
    return ref, result


def make_capture(lines, tracker_uid, receiver_key, note):
    ref, archive = decode_lines(lines)
    target = dict(board=h2.BOARD, tracker_uid=tracker_uid,
                  receiver_key=receiver_key, reference_id=ref)
    doc = {'schema': h2.CAPTURE_SCHEMA, 'target': target, 'archive': archive,
           'capture_note': note}
    # H2 strict validation: counts, float32, reference, fields, dimensions.
    h2.assemble(doc)
    return doc


def capture_serial(port, baud, timeout_s):
    """Explicitly send one safe, read-only console command via Tracker USB CDC.
    Never opens the receiver HID, sets parameters, or writes NVS.
    """
    try:
        import serial
    except ImportError:
        fail('pyserial unavailable; install it explicitly or use parse-log')
    deadline = time.monotonic() + timeout_s
    lines = []
    try:
        with serial.Serial(port=port, baudrate=baud, timeout=0.5) as link:
            # USB UART may reset on open; give the console time to initialize.
            time.sleep(1.0)
            link.reset_input_buffer()
            link.write(b'cal archive export\r\n')
            link.flush()
            while time.monotonic() < deadline:
                raw = link.readline()
                if not raw:
                    continue
                lines.append(raw.decode('ascii', 'replace'))
                if len(lines) > MAX_CAPTURE_LINES:
                    fail('capture exceeded line limit')
                if lines[-1].strip().startswith('ARC8X1 DONE '):
                    return lines
                if lines[-1].strip().startswith('ARC8X1 ERROR '):
                    fail(lines[-1].strip())
    except OSError as ex:
        fail(f'serial I/O failed: {ex}')
    fail('serial export timed out / incomplete; no capture output saved')


def main(argv=None):
    parser = argparse.ArgumentParser(description='ARC8 H3: verify read-only USB/RTT archive export, no NVS writes')
    sub = parser.add_subparsers(dest='command', required=True)
    for cmd in ('parse-log', 'capture-serial'):
        op = sub.add_parser(cmd)
        if cmd == 'parse-log':
            op.add_argument('log', help='unmodified Tracker console transcript')
        else:
            op.add_argument('--port', required=True, help='Tracker USB CDC serial port, not receiver HID')
            op.add_argument('--baud', type=int, default=115200)
            op.add_argument('--timeout', type=float, default=120.0)
        op.add_argument('--tracker-uid', required=True, help='operator-verified physical device identity')
        op.add_argument('--receiver-key', required=True, help='operator-declared pairing context; not authenticated')
        op.add_argument('--note', required=True, help='real provenance of observation set')
        op.add_argument('--out', required=True, help='new H2 capture JSON destination')
    args = parser.parse_args(argv)
    try:
        if args.command == 'parse-log':
            lines = Path(args.log).read_text(encoding='utf-8').splitlines()
        else:
            if not 5 <= args.timeout <= 600:
                fail('serial timeout outside 5..600 seconds')
            lines = capture_serial(args.port, args.baud, args.timeout)
        doc = make_capture(lines, args.tracker_uid, args.receiver_key, args.note)
        out = Path(args.out)
        with out.open('x', encoding='utf8') as f:  # No partial file if parse fails
            json.dump(doc, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.write('\n')
        print(f'Verified four ARC7 records; saved {out}. Device identity is operator-declared; no device writes.')
    except (ValueError, UnicodeError, OSError) as ex:
        print(f'ARC8-H3 export REFUSED: {ex}', file=sys.stderr)
        return 2
    return 0

if __name__ == '__main__':
    raise SystemExit(main())

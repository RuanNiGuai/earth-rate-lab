#!/usr/bin/env python3
"""ARC8 cal-archive driver for tracker consoles (B11+).

Wraps `cal archive <sub>` with status polling so each capture is confirmed
before moving on. Saves the full console transcript to a log file.

Usage:
    python tools/cal_archive.py status
    python tools/cal_archive.py on
    python tools/cal_archive.py ref-confirm
    python tools/cal_archive.py scale x start   # then rotate exactly 20 turns
    python tools/cal_archive.py scale x stop    # waits for the saved confirmation
    python tools/cal_archive.py earth start
    python tools/cal_archive.py earth stop      # use a NEW pose each time
    python tools/cal_archive.py gsens start
    python tools/cal_archive.py gsens pose
    python tools/cal_archive.py tcal
    python tools/cal_archive.py solve
    python tools/cal_archive.py export --out logs/arc8x1-t13.txt
    python tools/cal_archive.py cancel
"""
from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

ROOT = Path(__file__).resolve().parent.parent
LOG = ROOT / 'logs' / 'cal-archive-session.log'


def parse_status(text: str) -> dict:
    """Require both status records; silence/partial replies are not zero counts."""
    out = {}
    for prefix, keys in (
        ('ARC5', ('enabled', 'ref_id', 'confirmed', 'tcal_points', 'collecting', 'save_pending')),
        ('ARC7', ('index', 'step', 'pending', 'combined')),
    ):
        lines = [line.strip() for line in text.splitlines() if line.strip().startswith(prefix + ' ')]
        if not lines:
            raise ValueError(f'missing {prefix} status response')
        line = lines[-1]
        for key in keys:
            match = re.search(rf'\b{key}=(\d+)\b', line)
            if not match:
                raise ValueError(f'missing {prefix} field {key}')
            out[key] = int(match.group(1))
        if prefix == 'ARC7':
            for key in ('scale', 'earth', 'gsens'):
                match = re.search(rf'\b{key}=(\d+)/(\d+)\b', line)
                if not match:
                    raise ValueError(f'missing ARC7 field {key}')
                out[key] = tuple(map(int, match.groups()))
            match = re.search(r'\bcapture=(\w+)', line)
            if not match:
                raise ValueError('missing capture state')
            out['capture'] = match.group(1)
    return out


def validate_export(text: str, partial_kind: str | None = None) -> None:
    if partial_kind == 'tcal':
        from f06c1.tcal_export_codec import decode
        decode(text.splitlines())
    elif partial_kind:
        from f06c1.partial_export_codec import decode, verify_arc2_record_integrity
        frame = decode(text.splitlines())
        if frame.kind != partial_kind:
            raise ValueError('export dataset does not match request')
        verify_arc2_record_integrity(frame)
    else:
        # Reuse the maintained full-archive decoder: BEGIN alone is not success.
        decoder_dir = ROOT / 'patches/v22-helper-h3-gui/tools/arc8_cal_pc'
        sys.path.insert(0, str(decoder_dir))
        try:
            from archive_transport import decode_lines
            decode_lines(text.splitlines())
        finally:
            sys.path.remove(str(decoder_dir))


def capture_saved(key: str, before: dict, after: dict, transcript: str) -> bool:
    """Count progress or a fresh save ACK, with all pending writes drained.

    Replacing a full scale axis cannot be inferred from an unchanged 3/3 count.
    """
    if (after['pending'] or after['save_pending'] or after['collecting']
            or before['ref_id'] != after['ref_id']):
        return False
    if key == 'tcal_points':
        return (after[key] > before[key] and
                bool(re.search(rf'ARC4 TCal saved: {after[key]}/\d+\b', transcript)))
    slot = before['index'] + 1
    ack = re.search(rf'ARC7 saved {key} {slot}/\d+\b', transcript)
    return bool(ack or after[key][0] > before[key][0])


def require_ack(text: str, pattern: str) -> None:
    """A quiet serial response or a rejected command is never success."""
    if re.search(r'(?:ARC[457]|ARCHIVE).*(?:failed|refused|rejected|discard|first run|busy;)', text, re.I):
        raise ValueError('firmware refused or failed the command; see console log')
    if not re.search(pattern, text):
        raise ValueError('firmware acknowledgement missing; see console log')


class Console:
    def __init__(self, port: str, baud: int = 115200, log: Path = LOG) -> None:
        self.port = port
        self.baud = baud
        device = next((p for p in list_ports.comports() if p.device.upper() == port.upper()), None)
        if not device or (device.vid, device.pid) != (0x1209, 0x7692):
            raise ValueError('expected tracker USB VID/PID 1209:7692')
        log.parent.mkdir(parents=True, exist_ok=True)
        self.fh = log.open('a', encoding='utf-8')
        self.link = None

    def send(self, command: str, wait: float = 2.5) -> str:
        if self.link is None:
            self.link = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=2)
            time.sleep(0.25)
            self.link.reset_input_buffer()
        self.link.write((command + '\r\n').encode('ascii'))
        end = time.monotonic() + wait
        buf = bytearray()
        while time.monotonic() < end:
            buf.extend(self.link.read(8192))
            if len(buf) > 1024 * 1024:
                raise ValueError('console response exceeds 1 MiB')
        text = buf.decode('utf-8', 'replace')
        stamp = time.strftime('%H:%M:%S')
        self.fh.write(f'[{stamp}] > {command}\n{text}\n')
        self.fh.flush()
        return text

    def status(self) -> dict:
        text = self.send('cal archive status', 1.8)
        return parse_status(text)

    def wait_saved(self, key: str, before: dict, transcript: str,
                   timeout: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            text = self.send('cal archive status', min(1.8, max(.1, deadline-time.monotonic())))
            transcript += text
            st = parse_status(text)
            if re.search(r'ARC[457].*(?:failed|refused|rejected|discard|cancelled)', transcript, re.I):
                return False
            if capture_saved(key, before, st, transcript):
                return True
        return False

    def close(self) -> None:
        if self.link is not None:
            self.link.close()
        self.fh.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM10')
    ap.add_argument('--wait', type=float, default=2.5)
    ap.add_argument('--out')
    ap.add_argument('--log', type=Path, default=LOG)
    ap.add_argument('action', nargs='+')
    a = ap.parse_args()

    if a.out and Path(a.out).exists():
        ap.error('output exists; preserve previous evidence')
    con = Console(a.port, log=a.log)
    try:
        parts = a.action
        head = parts[0]

        if head == 'status':
            st = con.status()
            print(st)
            return 0

        if head == 'export':
            partial = parts[2] if len(parts) == 3 and parts[1] == 'partial' else None
            if parts == ['export', 'tcal']:
                partial = 'tcal'
            if parts != ['export'] and partial not in ('scale', 'earth', 'gsens', 'tcal'):
                ap.error('export accepts: [tcal | partial scale|earth|gsens]')
            text = con.send('cal archive ' + ' '.join(parts), 30.0)
            print(text[:2000])
            validate_export(text, partial)
            if a.out:
                with Path(a.out).open('x', encoding='utf-8') as fh:
                    fh.write(text)
                print(f'written {a.out}')
            return 0

        if head in ('scale',) and len(parts) >= 3:
            axis, op = parts[1], parts[2]
            if axis not in ('x', 'y', 'z') or op not in ('start', 'stop') or len(parts) != 3:
                ap.error('scale requires x|y|z start|stop')
            before = con.status() if op == 'stop' else None
            text = con.send(f'cal archive scale {axis} {op}', 3.0)
            print(text.strip()[:400])
            if op == 'stop':
                before['index'] = 'xyz'.index(axis)
                ok = con.wait_saved('scale', before, text)
                st = con.status()
                print(f'after stop: scale={st.get("scale")} saved={ok}')
                return 0 if ok else 1
            require_ack(text, rf'ARC5 scale {axis.upper()}: hold still')
            return 0

        if head in ('earth', 'gsens'):
            op = parts[1] if len(parts) > 1 else 'start'
            allowed_ops = ('start', 'stop') if head == 'earth' else ('start', 'pose')
            if op not in allowed_ops or len(parts) > 2:
                ap.error(f'invalid {head} operation')
            before = con.status() if op in ('stop', 'pose') else None
            text = con.send(f'cal archive {head} {op}', 3.0)
            print(text.strip()[:400])
            if op in ('stop', 'pose'):
                ok = con.wait_saved(head, before, text)
                st = con.status()
                print(f'after {op}: {head}={st.get(head)} saved={ok}')
                return 0 if ok else 1
            require_ack(text, rf'ARC7 {head} series started')
            return 0

        if head == 'tcal':
            if parts not in (['tcal'], ['tcal', 'start']):
                ap.error('tcal accepts only start')
            before = con.status()
            text = con.send('cal archive tcal start', 3.0)
            print(text.strip()[:400])
            require_ack(text, r'ARC4 TCal raw capture queued')
            ok = con.wait_saved('tcal_points', before, text)
            st = con.status()
            print(f'tcal_points={st["tcal_points"]} saved={ok}')
            return 0 if ok else 1

        if head == 'ref-confirm':
            if len(parts) != 1:
                ap.error('ref-confirm takes no extra arguments')
            text = con.send('cal archive ref confirm', 2.5)
            print(text.strip()[:300])
            st = con.status()
            require_ack(text, rf'ARC4 reference confirmed id={st["ref_id"]}\b')
            if not st['confirmed']:
                raise ValueError('reference is not confirmed')
            print(st)
            return 0

        # generic pass-through: on / off / solve / cancel / ref new / storage
        patterns = {
            ('on',): r'ARC4 ON:', ('off',): r'ARC4 OFF',
            ('solve',): r'ARC5 CANDIDATE ONLY:',
            ('cancel',): r'ARC5 capture cancelled;',
            ('ref', 'new'): r'ARC4 reference confirmed id=\d+',
            ('storage',): r'ARCHIVE NVS partition=\d+ bytes sectors=\d+ x \d+ free=\d+ max_item=\d+',
            ('combined', 'start'): r'ARC7 earth series started',
        }
        if tuple(parts) not in patterns:
            ap.error('unsupported archive action')
        cmd = 'cal archive ' + ' '.join(parts)
        text = con.send(cmd, a.wait)
        print(text.strip()[:600])
        require_ack(text, patterns[tuple(parts)])
        print(con.status())
        return 0
    except (ValueError, serial.SerialException) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    finally:
        con.close()


if __name__ == '__main__':
    raise SystemExit(main())

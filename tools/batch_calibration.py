"""Identity-bound ARC6 fleet capture for a rigid fixture.

Shared physical motions, independent on-device records. Production coefficients
are never changed by this tool. USB export produces inputs for offline review,
not an installed calibration. See docs/BATCH_CALIBRATION_zh.md.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
import time

FLAGS = {
    'on': 0x40, 'off': 0x41, 'status': 0x42, 'storage': 0x43,
    'reference-new': 0x44, 'reference-confirm': 0x45, 'temperature': 0x46,
    'preview': 0x47, 'scale-x-start': 0x48, 'scale-y-start': 0x49,
    'scale-z-start': 0x4a, 'scale-x-stop': 0x4b, 'scale-y-stop': 0x4c,
    'scale-z-stop': 0x4d, 'earth-start': 0x4e, 'earth-pose': 0x4f,
    'gsens-start': 0x50, 'gsens-pose': 0x51, 'cancel': 0x52,
}
SAVES = {'temperature', 'scale-x-stop', 'scale-y-stop', 'scale-z-stop',
         'earth-pose', 'gsens-pose'}
ERRORS = {1: 'busy', 2: 'reference', 3: 'wrong state', 4: 'rotation',
          5: 'motion compression', 6: 'face not cardinal', 7: 'duplicate face',
          8: 'out of range', 9: 'timeout', 10: 'unstable', 11: 'NVS',
          12: 'queue', 255: 'other'}


def check(condition, message):
    if not condition:
        raise ValueError(message)


def crc8(data):
    value = 7
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (7 if value & 0x80 else 0)) & 255
    return value


@dataclass(frozen=True)
class Status:
    slot: int
    sequence: int
    flag: int
    phase: int
    error: int
    saved_flags: int
    temperature_points: int
    scale_mask: int
    earth_count: int
    gsens_count: int
    counter: int

    @classmethod
    def decode(cls, packet):
        if len(packet) != 16 or packet[0] != 0x16:
            return None
        check(packet[2] == 1 and crc8(packet[:15]) == packet[15],
              'Malformed CAL_STATUS version/CRC')
        check(packet[1] < 32 and packet[3] != 0 and 0x40 <= packet[4] <= 0x53
              and packet[5] <= 5 and packet[8] <= 12 and packet[9] <= 7
              and packet[10] <= 5 and packet[11] <= 6,
              'CAL_STATUS fields out of range')
        return cls(packet[1], packet[3], packet[4], packet[5], packet[6],
                   packet[7], packet[8], packet[9], packet[10], packet[11], packet[14])


class Transaction:
    """Transport ACK and old counts cannot satisfy this transaction."""
    def __init__(self, slot, sequence, flag, needs_save):
        self.slot, self.sequence, self.flag = slot, sequence, flag
        self.needs_save = needs_save
        self.status = None
        self.failure = None

    def observe(self, status):
        if (status.slot, status.sequence, status.flag) != (self.slot, self.sequence, self.flag):
            return False
        # Terminal outcomes cannot be undone by a delayed queued/execute event.
        if status.phase in (3, 5) or status.error:
            self.failure = ERRORS.get(status.error, f'phase {status.phase}')
        if self.status is None or status.phase >= self.status.phase:
            self.status = status
        return True

    @property
    def succeeded(self):
        return (not self.failure and self.status is not None
                and self.status.phase in ({4} if self.needs_save else {2, 4}))


def utc():
    return datetime.now(timezone.utc).isoformat()


def save_new(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())


def atomic_save(path, value, *, best_effort=False):
    """Atomically publish JSON; telemetry must never stall the receiver pump.

    Windows readers/AV may briefly deny replacement. Durable state retries with
    a bound and still raises on failure. Only explicit live telemetry may skip
    a conflicting update; the next update carries the full current state.
    """
    temporary=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=path.parent,
                prefix='.'+path.name+'.',suffix='.pending',delete=False) as stream:
            temporary=Path(stream.name)
            json.dump(value,stream,indent=2,ensure_ascii=False,allow_nan=False)
            stream.flush();os.fsync(stream.fileno())
        for attempt in range(7):
            try:
                os.replace(temporary,path)
                return True
            except OSError as error:
                if getattr(error,'winerror',None) not in (5,32,33):raise
                if best_effort:return False
                if attempt==6:raise
                time.sleep(min(.01*2**attempt,.08))
    finally:
        if temporary is not None and temporary.exists():
            try:temporary.unlink()
            except OSError:pass  # Never mask the original publication error.


@contextmanager
def lock(path):
    # An abandoned lock is intentionally not stolen: inspect the previous run.
    with path.open('x', encoding='ascii') as stream:
        stream.write(str(os.getpid()))
    try:
        yield
    finally:
        path.unlink()


class Journal:
    def __init__(self, path):
        self.path = path

    def add(self, **data):
        with self.path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps({'utc': utc(), **data}, ensure_ascii=False) + '\n')
            stream.flush()
            os.fsync(stream.fileno())


def validate_targets(targets):
    check(1 <= len(targets) <= 16, 'Choose 1..16 explicit targets')
    result = {}
    for slot, address in targets.items():
        check(str(int(slot)) == str(slot) and 0 <= int(slot) < 32, 'Invalid slot')
        check(isinstance(address, str) and re.fullmatch('[0-9a-fA-F]{12}', address)
              and address != '000000000000', 'Invalid RF address')
        result[int(slot)] = address.upper()
    check(len(set(result.values())) == len(result), 'Duplicate RF identity')
    return result


class Radio:
    def __init__(self, receiver_serial, targets, journal):
        import hid
        matches = [x for x in hid.enumerate(0x1209, 0x7690)
                   if x.get('serial_number') == receiver_serial
                   and x.get('interface_number') == 2 and x.get('usage_page') == 1]
        check(len(matches) == 1, 'Expected exact receiver serial and main interface')
        locks = Path(os.environ.get('EARTH_RATE_STATE_DIR', Path(__file__).resolve().parents[1] / '.local')) / 'hardware-locks'
        locks.mkdir(parents=True, exist_ok=True)
        self.lock_path = locks / (hashlib.sha256(receiver_serial.encode()).hexdigest() + '.lock')
        with self.lock_path.open('x', encoding='ascii') as stream:
            stream.write(str(os.getpid()))
        self.device = hid.device()
        try:
            self.device.open_path(matches[0]['path'])
            # Python hidapi treats read(..., 0) as hid_read(), not as a timed
            # poll. F04 suspends ordinary poses, so a blocking main handle can
            # stall the separate capture endpoint until its queues overflow.
            self.device.set_nonblocking(True)
        except BaseException:
            try:
                self.device.close()
            finally:
                self.lock_path.unlink()
            raise
        self.targets, self.journal = targets, journal
        self.registered, self.live = {}, {}
        self.conflicts = set()

    def close(self):
        try:
            self.device.close()
        finally:
            self.lock_path.unlink()

    def read(self, timeout_ms=100):
        report = bytes(self.device.read(64, timeout_ms))
        return self.observe(report)

    def observe(self, report):
        if not report:
            return []
        check(len(report) == 64, 'Unexpected main HID report length')
        statuses = []
        now = time.monotonic()
        for offset in range(0, 64, 16):
            packet = report[offset:offset + 16]
            if packet[0] == 251 and 0x40 <= packet[2] <= 0x53:
                self.journal.add(event='receiver-ack-not-save', packet=packet.hex())
                continue
            slot = packet[1]
            if slot not in self.targets:
                continue
            if packet[0] == 255 and packet[2:8] != bytes(6):
                address = packet[2:8][::-1].hex().upper()
                if address != self.targets[slot]:
                    self.conflicts.add(slot)
                else:
                    self.registered[slot] = now
            elif packet[0] in range(1, 8):
                self.live[slot] = now
            elif packet[0] == 0x16:
                status = Status.decode(packet)
                self.journal.add(event='status', packet=packet.hex(), status=asdict(status))
                statuses.append(status)
        check(not self.conflicts, f'RF identity changed at slots {sorted(self.conflicts)}')
        return statuses

    def bind(self, seconds=5):
        # No command is sent until the entire frozen target set is observed.
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.read()
        now = time.monotonic()
        missing = [slot for slot in self.targets if now-self.registered.get(slot, 0) > 3
                   or now-self.live.get(slot, 0) > 3]
        check(not missing, f'No fresh RF registration and sensor traffic: {missing}')

    def send(self, slot, sequence, flag):
        check(slot not in self.conflicts and slot in self.registered, 'Unbound target')
        frame = bytearray(65)
        frame[1:6] = bytes([254, sequence, flag, 0, slot])
        self.journal.add(event='about-to-send', slot=slot, sequence=sequence, flag=flag)
        transferred = self.device.write(frame)
        check(transferred == 65, 'Partial HID write; delivery uncertain, not retried')


def perform(radio, action, sequence, targets, timeout=45, progress=None):
    transactions = {slot: Transaction(slot, sequence, FLAGS[action], action in SAVES)
                    for slot in targets}
    last_checked = -1
    def report():
        nonlocal last_checked
        checked = sum(tx.succeeded for tx in transactions.values())
        if progress is not None and checked != last_checked:
            progress(dict(checked=checked, total=len(transactions)))
            last_checked = checked
    def ingest(timeout_ms=100):
        for status in radio.read(timeout_ms):
            if status.slot in transactions:
                transactions[status.slot].observe(status)
        report()

    report()
    for slot in targets:
        radio.send(slot, sequence, FLAGS[action])
        # This receiver has ONE global control mailbox, even though trackers
        # accumulate concurrently. Wait for execute, not just HID admission,
        # before dispatching the next target. Keep all earlier save events.
        end = time.monotonic() + min(timeout, 12)
        while time.monotonic() < end:
            ingest()
            tx = transactions[slot]
            if tx.status is not None and tx.status.phase >= 2:
                break
        else:
            # Do not silently send more commands into an unresolved mailbox.
            break
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if all(tx.succeeded or tx.failure for tx in transactions.values()):
            break
        ingest()
    return {str(slot): {'success': tx.succeeded, 'error': tx.failure or
                       (None if tx.succeeded else 'timeout: outcome uncertain; do not repeat blindly'),
                       'status': asdict(tx.status) if tx.status else None}
            for slot, tx in transactions.items()}


def stage_allowed(state, action, targets):
    rows = state['devices']
    for slot in targets:
        row = rows[str(slot)]
        if action in ('status', 'storage', 'cancel', 'off', 'on'):
            continue
        check(row.get('on'), f'#{slot}: archive must be enabled')
        if action.startswith('reference-'):
            check(not row.get('active'), f'#{slot}: cancel capture before changing reference')
            continue
        check(row.get('reference'), f'#{slot}: physical reference has not been established')
        if action.endswith('-start'):
            check(not row.get('active'), f'#{slot}: another capture is active')
        elif action.startswith('scale-') and action.endswith('-stop'):
            check(row.get('active') == action.replace('-stop', '-start'), f'#{slot}: wrong scale axis')
        elif action == 'earth-pose':
            check(row.get('active') in ('earth-start', 'combined-start'), f'#{slot}: no Earth series')
        elif action == 'gsens-pose':
            check(row.get('active') == 'gsens-start', f'#{slot}: no GSens series')
        elif action in ('temperature', 'preview'):
            check(not row.get('active'), f'#{slot}: another capture is active')


def advance(state, action, results):
    for slot, result in results.items():
        row = state['devices'][slot]
        if not result['success']:
            row['uncertain'] = True
            continue
        if action in ('status', 'storage', 'preview'):
            continue
        if action == 'on':
            row['on'] = True
        elif action in ('off', 'cancel'):
            row['active'] = None
            row['uncertain'] = False
            if action == 'off':
                row['on'] = False
                row['reference'] = False
        elif action.startswith('reference-'):
            row['reference'] = True
            if action == 'reference-new':
                row.update(earth=0, gsens=0, scale=[], temperature=False)
        elif action == 'temperature':
            row['temperature'] = True
        elif action.endswith('-start'):
            row['active'] = action
            if action in ('earth-start', 'combined-start'):
                row['earth'] = 0
            if action == 'gsens-start':
                row['gsens'] = 0
        elif action.startswith('scale-'):
            row['active'] = None
            axis = action.split('-')[1]
            row['scale'] = sorted(set(row.get('scale', [])) | {axis})
        elif action in ('earth-pose', 'gsens-pose'):
            kind = action.split('-')[0]
            row[kind] = row.get(kind, 0) + 1
            if row[kind] == (5 if kind == 'earth' else 6):
                if kind == 'earth' and row['active'] == 'combined-start':
                    # Firmware saves the shared GSens reference AFTER the Earth
                    # SAVED event. This v1 packet cannot certify that second save.
                    row['uncertain'] = True
                    row['active'] = 'combined-transition-needs-usb-verification'
                else:
                    row['active'] = None


def run_action(directory, action, selected=None, progress=None):
    with lock(directory / 'LOCK'):
        path = directory / 'session.json'
        state = json.loads(path.read_text(encoding='utf-8'))
        check(state.get('initialized'), 'Read-only initialization did not finish')
        summary_owner = directory / 'summary-owner.json'
        if summary_owner.exists() and action not in ('status', 'storage'):
            check(not json.loads(summary_owner.read_text())['active'],
                  'Summary capture cleanup is unverified; inspect receiver before calibration')
        targets = validate_targets(state['targets'])
        if selected is not None:
            check(set(selected) <= set(targets) and len(set(selected)) == len(selected), 'Invalid subset')
            targets = {slot: targets[slot] for slot in selected}
        check(targets, 'Empty target set')
        for slot in targets:
            check(not state['devices'][str(slot)].get('uncertain') or action in ('status', 'storage', 'cancel', 'off'),
                  f'#{slot}: previous operation uncertain; inspect/cancel before continuing')
        check(not state.get('inflight'), 'Interrupted command: inspect journal; use recover before further writes')
        stage_allowed(state, action, targets)
        sequence = state['next_sequence']
        check(1 <= sequence <= 250, 'Session sequence budget exhausted; export and start a new session')
        journal = Journal(directory / 'radio.jsonl')
        radio = Radio(state['receiver_serial'], targets, journal)
        try:
            if progress is not None:
                progress(dict(stage='binding'))
            radio.bind()
            state['inflight'] = dict(action=action, sequence=sequence, targets=list(targets), utc=utc())
            state['next_sequence'] += 1
            atomic_save(path, state)  # persist intent before the first command
            if progress is not None:
                progress(dict(stage='checking', round=1, rounds=1, checked=0, total=len(targets)))
            results = perform(radio, action, sequence, targets, progress=progress)
            record = dict(action=action, sequence=sequence, results=results, utc=utc())
            save_new(directory / f'{sequence:03d}-{action}.json', record)
            advance(state, action, results)
            state['inflight'] = None
            atomic_save(path, state)
            print(json.dumps(dict(action=action, sequence=sequence,
                devices={slot: dict(success=r['success'], error=r['error'],
                         phase=r['status']['phase'] if r['status'] else None)
                         for slot, r in results.items()}), ensure_ascii=False, indent=2), flush=True)
            return all(r['success'] for r in results.values())
        finally:
            radio.close()


def initialize(directory, config, progress=None):
    specification = json.loads(config.read_text(encoding='utf-8-sig'))
    targets = validate_targets(specification['targets'])
    check(isinstance(specification['receiver_serial'], str) and specification['receiver_serial'], 'Missing receiver serial')
    directory.mkdir(parents=True, exist_ok=False)
    state = dict(schema=1, created_at=utc(), receiver_serial=specification['receiver_serial'],
                 targets={str(k): v for k, v in targets.items()}, next_sequence=5,
                 devices={str(k): {} for k in targets}, inflight=None,
                 production_coefficients_changed=False, initialized=False)
    save_new(directory / 'session.json', state)
    journal = Journal(directory / 'radio.jsonl')
    radio = Radio(state['receiver_serial'], targets, journal)
    try:
        if progress is not None:
            progress(dict(stage='binding'))
        radio.bind()
        # Evict both tracker dedupe slots using read-only operations. Different
        # sequences prevent a previous session's final mutation being replayed.
        for seq in (1, 2, 3):
            if progress is not None:
                progress(dict(stage='checking', round=seq, rounds=3, checked=0, total=len(targets)))
            result = perform(radio, 'status', seq, targets, timeout=12, progress=progress)
            save_new(directory / f'initial-status-{seq}.json', result)
            check(all(x['success'] for x in result.values()), 'Initial status incomplete')
        save_new(directory / 'initial-status.json', result)
        state['initialized'] = True
        atomic_save(directory / 'session.json', state)
    finally:
        radio.close()
    print(f'Bound {len(targets)} trackers; session {directory}', flush=True)


def export_usb(directory, port_name):
    from serial.tools import list_ports
    from cal_archive import Console, parse_status, validate_export
    with lock(directory / 'LOCK'):
        state = json.loads((directory / 'session.json').read_text(encoding='utf-8'))
        targets = validate_targets(state['targets'])
        ports = [p for p in list_ports.comports() if p.device.upper() == port_name.upper()
                 and (p.vid, p.pid) == (0x1209, 0x7692) and p.serial_number]
        check(len(ports) == 1, 'Tracker USB identity unavailable')
        output = directory / ('usb-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        output.mkdir()
        console = Console(port_name, log=output / 'console.log')
        try:
            info = console.send('info')
            found = re.findall(r'^Device address: ([0-9A-Fa-f]{12})\s*$', info, re.M)
            check(len(found) == 1 and found[0].upper() in targets.values(), 'USB RF identity outside this batch')
            address = found[0].upper()
            slot = next(k for k, v in targets.items() if v == address)
            status = parse_status(console.send('cal archive status'))
            check(status['capture'] == 'none' and not any(status[k] for k in ('collecting', 'pending', 'save_pending')),
                  'Capture still running; finish or cancel before export')
            text = console.send('cal archive export', 12)
            validate_export(text)
            (output / 'archive.txt').write_text(text, encoding='utf-8')
            settings = {cmd: console.send(cmd, 3) for cmd in (
                'gyro scale status', 'gyro gsens status', 'earth cal status', 'tcal status', 'tcal dump', 'tcal lut')}
            recheck = console.send('info')
            check(re.findall(r'^Device address: ([0-9A-Fa-f]{12})\s*$', recheck, re.M) == found,
                  'Identity changed during export')
            save_new(output / 'backup.json', dict(slot=slot, rf_address=address,
                     usb_serial=ports[0].serial_number, info=info, responses=settings,
                     archive_sha256=hashlib.sha256(text.encode('utf-8')).hexdigest(),
                     installed=False, complete_nvs_backup=False))
        finally:
            console.close()
        print(str(output), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    sub = parser.add_subparsers(dest='action', required=True)
    create = sub.add_parser('init')
    create.add_argument('--config', type=Path, required=True)
    for action in FLAGS:
        command = sub.add_parser(action)
        command.add_argument('--ids', type=int, nargs='+')
    export = sub.add_parser('export-usb')
    export.add_argument('--port', required=True)
    sub.add_parser('recover', help='mark interrupted operations uncertain; never retry them')
    args = parser.parse_args()
    if args.action == 'init':
        initialize(args.session, args.config)
    elif args.action == 'export-usb':
        export_usb(args.session, args.port)
    elif args.action == 'recover':
        with lock(args.session / 'LOCK'):
            path = args.session / 'session.json'
            state = json.loads(path.read_text(encoding='utf-8'))
            request = state.get('inflight')
            check(request, 'No interrupted operation')
            Journal(args.session / 'radio.jsonl').add(event='operator-recover', request=request)
            for slot in request['targets']:
                state['devices'][str(slot)]['uncertain'] = True
            state['inflight'] = None
            atomic_save(path, state)
    else:
        return 0 if run_action(args.session, args.action, args.ids) else 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

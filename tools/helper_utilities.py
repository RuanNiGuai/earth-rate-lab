"""Helper device tools: frozen identities, confirmed commands, durable output.

Independent runs never modify the joint calibration session's artifact index.
Temperature START is explicit; inspection never starts temperature collection.
"""
import hashlib
import secrets
import time

import batch_calibration as b


class Radio(b.Radio):
    def __init__(self, *args):
        super().__init__(*args)
        self.receipts = {}
        self.temperatures = {}
        self.sequence = secrets.randbelow(255) + 1

    def observe(self, report):
        statuses = super().observe(report)  # includes RF identity conflict checks
        for offset in range(0, len(report), 16):
            p = report[offset:offset+16]
            if p[0] == 251:
                self.journal.add(event='command-receipt', packet=p.hex())
                self.receipts[(p[1], p[2])] = p[3]
            elif p[0] == 6 and p[1] in self.targets:
                self.temperatures[p[1]] = dict(slot=p[1], points=int.from_bytes(p[2:4], 'little'),
                    temperature_c=int.from_bytes(p[6:8], 'little', signed=True)/2,
                    collecting=bool(p[8] & 4), auto_collection_enabled=bool(p[8] & 8))
        return statuses

    def control(self, slot, flag):
        self.sequence = self.sequence % 255 + 1
        key = (self.sequence, flag)
        self.receipts.clear()
        self.send(slot, self.sequence, flag)
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            self.read(100)
            status = self.receipts.get(key)
            if status == 0:
                return
            b.check(status is None or status in (6, 7),
                    f'#{slot} command 0x{flag:02x} rejected ({status}); delivery is uncertain')
        raise TimeoutError(f'#{slot} command 0x{flag:02x} unconfirmed; inspect the saved journal')


def temperature(request, output, cancel, emit):
    from calwire import Radio as SnapshotRadio, unpack_snapshot
    from thermal_coverage import audit
    targets = b.validate_targets(request['targets'])
    action = request['action']
    output.mkdir()
    journal = b.Journal(output/'radio.jsonl')
    radio = Radio(request['receiver_serial'], targets, journal)
    touched = []
    confirmed = []
    try:
        emit(stage='binding')
        radio.bind()
        for slot in targets:
            b.check(not cancel.exists(), 'Cancelled; completed device commands are recorded in radio.jsonl')
            emit(stage='temperature', completed=len(confirmed), total=len(targets))
            if action != 'utility-temperature-status':
                touched.append(slot)
                radio.control(slot, 0x13 if action == 'utility-temperature-start' else 0x14)
                journal.add(event='temperature-command-confirmed', slot=slot, enabled=action.endswith('-start'))
            radio.temperatures.pop(slot, None)
            radio.control(slot, 0x11)  # fresh status report, after previous command completes
            deadline = time.monotonic()+3
            expected_auto = None if action == 'utility-temperature-status' else action.endswith('-start')
            def status_matches():
                return slot in radio.temperatures and (expected_auto is None or
                    radio.temperatures[slot]['auto_collection_enabled'] == expected_auto)
            while not status_matches() and time.monotonic() < deadline:
                radio.read(100)
            b.check(slot in radio.temperatures, f'#{slot} did not return fresh temperature status')
            b.check(status_matches(), f'#{slot} automatic temperature collection state did not match the command')
            confirmed.append(slot)
            emit(stage='temperature', completed=len(confirmed), total=len(targets),
                 temperatures=list(radio.temperatures.values()))
        rows = list(radio.temperatures.values())
    except BaseException:
        # Never leave a partially enabled fleet after a START failure/cancel.
        if action == 'utility-temperature-start':
            for slot in touched:
                try:
                    radio.control(slot, 0x14)
                    journal.add(event='start-rollback-confirmed', slot=slot)
                except Exception as error:
                    journal.add(event='start-rollback-unconfirmed', slot=slot, error=str(error))
        raise
    finally:
        radio.close()
    result = dict(temperatures=rows, confirmed_slots=confirmed)
    if action == 'utility-temperature-coverage':
        # Explicit STOP+inspect: freezing a coherent CALWIRE table requires auto OFF.
        snapshots = output/'snapshots'
        snapshots.mkdir()
        radio = SnapshotRadio(request['receiver_serial'], journal, cancel_file=cancel)
        try:
            for i, (slot, address) in enumerate(targets.items()):
                emit(stage='snapshot', completed=i, total=len(targets))
                dest = snapshots/f'slot-{slot}'
                dest.mkdir()
                with radio.session(slot, address) as (token, size, crc):
                    blob = radio.snapshot(slot, address, token, size, crc)
                (dest/'snapshot.bin').write_bytes(blob)
                data = unpack_snapshot(blob, require_scales=False, allow_empty_thermal=True)
                data.update(slot=slot, rf_address=address, snapshot_sha256=hashlib.sha256(blob).hexdigest())
                b.save_new(dest/'backup.json', data)
        finally:
            radio.close()
        result['coverage'] = audit(snapshots, sorted(targets))
        b.save_new(output/'coverage.json', result['coverage'])
    b.save_new(output/'temperature.json', result)
    return result


def execute(request, job, cancel, emit):
    action = request['action']
    output = job/'device-tool'
    if action == 'utility-record':
        from batch_record_only import record
        from capture_preflight import check_capture_conflicts
        from capture_quality import failure_message
        import json
        check_capture_conflicts()
        config = job/'fleet.json'
        b.save_new(config, dict(receiver_serial=request['receiver_serial'], targets=request['targets']))
        b.initialize(output, config, progress=lambda row: emit(**row))
        recording = record(output, cancel_file=cancel, progress=lambda row: emit(**row))
        quality = json.loads((recording/'quality.json').read_text(encoding='utf8'))
        return dict(output=str(recording), transport_pass=quality['transport_pass'],
                    failures=[] if quality['transport_pass'] else [failure_message(quality)])
    return dict(output=str(output), **temperature(request, output, cancel, emit))

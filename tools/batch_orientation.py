"""Brief, explicitly targeted F04 gravity observation for the batch fixture.

The 50 Hz summaries identify a face only. They are never used as full-rate
joint solver input. Collection is stopped and verified before returning.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import batch_calibration as batch
from f04_summary import decode_report


def classify(samples):
    batch.check(len(samples) >= 100, 'Need at least two seconds of fresh summary samples')
    batch.check(len({(s.tracker, s.boot, s.operation) for s in samples}) == 1, 'Mixed summary sessions')
    batch.check(len({s.sequence for s in samples}) == len(samples), 'Duplicate summary windows')
    batch.check(all(s.accel_fresh > 0 for s in samples), 'Missing fresh accelerometer observations')
    batch.check(all(max(abs(g / s.duration_us) for g in s.gyro_delta_udeg) < 3
                    for s in samples), 'Fixture is moving')
    count = sum(s.samples for s in samples)
    accel = [sum(s.accel_mean_mg[i] * s.samples for s in samples) / count / 1000 for i in range(3)]
    norm = math.sqrt(sum(x*x for x in accel))
    batch.check(.96 <= norm <= 1.04, 'Gravity magnitude is not stable near 1 g')
    axis = max(range(3), key=lambda i: abs(accel[i]))
    batch.check(abs(accel[axis])/norm >= .98, 'Place a cardinal face flat')
    return dict(axis='xyz'[axis], face=('+' if accel[axis] > 0 else '-') + 'XYZ'[axis],
                accel_g=accel, tracker=samples[0].tracker, samples=len(samples),
                boot=samples[0].boot, operation=samples[0].operation,
                purpose='fixture orientation only; not calibration solver input')


def observe(directory, slot):
    import hid
    import serial
    from serial.tools import list_ports
    with batch.lock(directory / 'LOCK'):
        state = json.loads((directory / 'session.json').read_text(encoding='utf-8'))
        targets = batch.validate_targets(state['targets'])
        batch.check(slot in targets and state.get('initialized') and not state.get('inflight'), 'Session not ready')
        batch.check(all(not row.get('active') and not row.get('uncertain') for row in state['devices'].values()),
                    'End all archive captures before observing fixture orientation')
        marker = directory / 'summary-owner.json'
        if marker.exists():
            batch.check(not json.loads(marker.read_text())['active'], 'Previous summary stop unverified')
        receiver = state['receiver_serial']
        ports = [p for p in list_ports.comports()
                 if (p.vid, p.pid, p.serial_number) == (0x1209, 0x7690, receiver)]
        devices = [d for d in hid.enumerate(0x1209, 0x7690)
                   if d.get('serial_number') == receiver and d.get('usage_page') == 0xff00]
        batch.check(len(ports) == len(devices) == 1, 'Receiver identity/interface mismatch')
        output = directory / ('orientation-' + str(time.time_ns()))
        output.mkdir()
        journal = batch.Journal(output / 'journal.jsonl')
        radio = batch.Radio(receiver, targets, journal)
        dev, console, owned = None, None, False
        samples = deque(maxlen=100)
        try:
            radio.bind()
            console = serial.Serial(ports[0].device, 115200, timeout=0, write_timeout=2)
            dev = hid.device()
            dev.open_path(devices[0]['path'])
            dev.set_nonblocking(True)

            def pump(seconds):
                deadline = time.monotonic()+seconds
                data = bytearray()
                while time.monotonic() < deadline:
                    data.extend(console.read(8192))
                    batch.check(len(data) <= 256*1024, 'Receiver output exceeded bound')
                    radio.read(0)
                    for _ in range(96):
                        report = bytes(dev.read(64))
                        if not report:
                            break
                        if owned:
                            sample, _, _ = decode_report(report)
                            batch.check(sample.tracker == slot, 'Unexpected F04 target')
                            samples.append(sample)
                            journal.add(event='summary', report_hex=report.hex())
                    time.sleep(.003)
                return data.decode('utf-8', 'replace')

            def command(text, seconds=1):
                journal.add(event='command', command=text)
                console.write((text+'\r\n').encode('ascii'))
                result = pump(seconds)
                journal.add(event='response', command=text, text=result)
                return result

            pump(.4)
            batch.check('Data collection inactive' in command('collect'), 'Another collection is active')
            batch.check('No active session' in command('ota'), 'OTA is active')
            owner = dict(active=True, receiver_serial=receiver, slot=slot, utc=batch.utc(), output=str(output))
            batch.atomic_save(marker, owner)
            owned = True  # delivery may be uncertain; cleanup still owns it
            try:
                batch.check('Batch collection STARTED' in command(f'collectall 50 {slot}', 2), 'Summary not admitted')
                pump(3)
            finally:
                stop = command('collectstop', 9)
                inactive = command('collect')
                batch.check('unconfirmed=' not in stop and 'Batch data collection STOPPED' in stop
                            and 'Data collection inactive' in inactive,
                            'Summary stop unconfirmed; do not start calibration')
                owned = False
                owner['active'] = False
                owner['stopped_at'] = batch.utc()
                batch.atomic_save(marker, owner)
            result = classify(list(samples))
            batch.save_new(output / 'orientation.json', result)
            print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
            return result
        finally:
            if dev:
                dev.close()
            if console:
                console.close()
            radio.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--slot', type=int, required=True)
    args = parser.parse_args()
    observe(args.session, args.slot)

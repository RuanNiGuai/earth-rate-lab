"""Compare two CALWIRE backup folders without opening or writing any device.

This proves byte equality of exported calibration, not that a physical reboot
occurred. Keep the observed reboot/build evidence with the resulting report.
"""
import argparse
import hashlib
import json
from pathlib import Path
from calwire import unpack_snapshot


def compare(before, after, *, allow_build_change=False):
    a, b = [unpack_snapshot(x, require_scales=False) for x in (before, after)]
    fields = ['profile_bytes', 'thermal_crc32', 'archive_objects_hex']
    changes = [k for k in fields if a[k] != b[k]]
    if a['build'] != b['build'] and not allow_build_change:
        changes.append('build')
    accel_known = a['accelerometer']['known'] and b['accelerometer']['known']
    if accel_known and a['accelerometer']['wire_hex'] != b['accelerometer']['wire_hex']:
        changes.append('accelerometer')
    return dict(parameters_match=not changes, changed_fields=changes,
                accelerometer_compared=accel_known,
                before_build=a['build'], after_build=b['build'],
                before_sha256=hashlib.sha256(before).hexdigest(),
                after_sha256=hashlib.sha256(after).hexdigest(),
                reboot_verified=False, device_writes=0)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--before', type=Path, required=True, help='Raw snapshot.bin before reboot')
    p.add_argument('--after', type=Path, required=True, help='Raw snapshot.bin after reboot')
    p.add_argument('--allow-build-change', action='store_true', help='Compare across an explicit firmware update')
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    # Bind files to the same physical device when generated backup metadata is present.
    meta = [x.with_name('backup.json') for x in (a.before, a.after)]
    if not all(x.is_file() for x in meta):
        raise ValueError('Both backup.json identity records are required')
    records = [json.loads(x.read_text(encoding='utf8')) for x in meta]
    for path, row in zip((a.before, a.after), records):
        if row['snapshot_sha256'] != hashlib.sha256(path.read_bytes()).hexdigest():
            raise ValueError('Backup metadata hash mismatch')
    if any(records[0].get(k) != records[1].get(k) or records[0].get(k) is None
           for k in ('receiver_serial', 'slot', 'rf_address')):
        raise ValueError('Physical identity mismatch or missing identity')
    report = compare(a.before.read_bytes(), a.after.read_bytes(), allow_build_change=a.allow_build_change)
    with a.out.open('x', encoding='utf8') as stream:
        json.dump(report, stream, indent=2)
    return 0 if report['parameters_match'] else 2


if __name__ == '__main__':
    raise SystemExit(main())

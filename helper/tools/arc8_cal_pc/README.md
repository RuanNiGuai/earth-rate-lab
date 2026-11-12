# ARC8-H2: PC-side calibration candidate, NOT a device writer

This proof-of-concept runs the **same** `cal_arc_solve()` as the ARC8-B ARC7
tracker, compiled as a temporary native shared library with GCC/Clang. It is
not a new VQF/calibration algorithm and does not touch existing IMU filters.

The source snapshots in `vendor/cal_archive_core.[ch]` were copied from the
uploaded `ARC8B_H1_只读HID管理_完整实验源码.zip` and SHA-256 pinned in
`calibration.py`. Do not substitute an upstream version silently. The input
schema is a **normalized, assembled ARC7 archive**; it is NOT raw HID samples,
NVS pages, or a `.hidlog`. ARC8-B does not currently implement a reliable
export of those assembled structures. The template is NOT measurement data.

## Usage (Python 3.10+, host GCC or Clang)

```shell
python tools/arc8_cal_pc/calibration.py template example-structure.json
python tools/arc8_cal_pc/calibration.py solve real-exported-archive.json candidate.cal.json
python tools/arc8_cal_pc/calibration.py verify candidate.cal.json
python -m unittest discover -s tools/arc8_cal_pc/tests -v
```

**Important**: `solve` correctly refuses incomplete, fictitious or low-quality
captures. A candidate can only be produced from complete, compatible real
observation data. The solver's C `ready` flag is mandatory. The output is a
candidate, not proof of NVS persistence. `verify` checks local JSON
integrity/limits only; it does not verify a signature or authenticate a
tracker. Neither command opens USB, sends HID/ESB or writes to NVS.

## Source and limitations

- The original ARC7 code requires 3 scale captures (20 turns per axis),
  5 Earth stops with their travel segments, 6 g-sens poses with motion
  segments, and at least one T-Cal reference point, all sharing the same
  `reference_id`. The PC must obtain the raw archive, not only a progress event.
- Captured `generation` and `reference_id` in the normalized JSON are checked;
  a later exporter must additionally verify actual NVS record checksums,
  acquisition epochs, identity, units and ordering before writing the JSON.
- `tracker_uid` and `receiver_key` in this JSON are *declarations* until the
  receiver/Tracker can cryptographically or physically bind an import session.
- A source-data SHA256 and solver SHA256 are retained in the candidate. SHA256
  detects accidental changes but does not authenticate who generated a file.
- No device import/commit/rollback implementation is included. It is unsafe to
  chain `earth ref set`, `gyro gsens set` and legacy one-by-one setters and
  call that an atomic calibration install.

See `docs/ARC8_H2_offline_calibration.zh-CN.md` for the proposed staged NVS
protocol and an explicit list of unverified items.

## ARC8-H3 read-only archive export

The paired tracker source now includes `cal archive export` (console only).
Run `python tools/arc8_cal_pc/archive_transport.py --help` for the
`parse-log` and `capture-serial` commands. Only four complete, verified ARC7
NVS archives can be converted to an H2 input JSON; raw IMU samples and
`.hidlog` are not a substitute. The tracker firmware must actually be rebuilt
and installed before its new export command exists on hardware. No receiver
ESB/HID export or calibration install is included.

For reproducible Windows MinGW use an ASCII-only `TMP`/`TEMP` directory.
The H2 ctypes temporary DLL fix uses `ignore_cleanup_errors=True` only for
Windows DLL cleanup; solver and quality failures remain fatal. Read
`docs/ARC8_H3_archive_export.zh-CN.md` for commands, safety limits and tests.

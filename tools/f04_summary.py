"""F04 v1 decoder and deterministic, bounded-reorder replay.

Input JSONL: a header {format: 'F04HID1', receiver_serial: ...}, then
{host_monotonic_ns: ..., report_hex: ...} records. Original 64-byte HID reports
are retained. Summaries contain uncalibrated data and estimated Tracker time.
"""
import argparse
from collections import OrderedDict
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import struct
import zlib

MASK = (1 << 32) - 1


@dataclass(frozen=True)
class EndManifest:
    tracker: int
    boot: int
    operation: int
    sequence_end: int
    valid_windows: int
    invalid_windows: int
    queue_drop: int
    submitted: int
    tail_drop: int
    deferred: int
    partial_samples: int


def decode_frame(payload):
    if len(payload) == 48 and payload[:1] == b'\x15' and payload[2:4] == b'\x01\x02':
        if payload[1] >= 32 or struct.unpack_from('<I', payload, 44)[0] != zlib.crc32(payload[:44]):
            raise ValueError('F04 end identity/CRC invalid')
        end = EndManifest(payload[1], *struct.unpack_from('<10I', payload, 4))
        if (not end.boot or not end.operation or end.sequence_end != end.valid_windows + end.invalid_windows
                or end.valid_windows != end.queue_drop + end.submitted + end.tail_drop
                or end.partial_samples > 32):
            raise ValueError('F04 end counters inconsistent')
        return end
    return decode(payload)


def decode_frame_report(report):
    if len(report) != 64 or any(report[53:]):
        raise ValueError('F04 HID report size/padding invalid')
    return decode_frame(report[:48]), struct.unpack_from('b', report, 48)[0], struct.unpack_from('>I', report, 49)[0]


@dataclass(frozen=True)
class Summary:
    tracker: int
    boot: int
    operation: int
    sequence: int
    end_us: int
    gyro_delta_udeg: tuple
    accel_mean_mg: tuple
    temperature_centi_c: int
    samples: int
    accel_fresh: int
    duration_us: int

    @property
    def accel_held(self):
        return self.samples - self.accel_fresh


def decode(payload):
    if len(payload) != 48:
        raise ValueError('F04 payload must be exactly 48 bytes')
    if payload[0] != 0x15 or payload[1] >= 32 or payload[2:4] != b'\x01\x01':
        raise ValueError('F04 type/tracker/version/estimated-time flags invalid')
    if struct.unpack_from('<I', payload, 44)[0] != zlib.crc32(payload[:44]):
        raise ValueError('F04 CRC mismatch')
    boot, op, seq, end = struct.unpack_from('<4I', payload, 4)
    gyro = struct.unpack_from('<3i', payload, 20)
    accel = struct.unpack_from('<3h', payload, 32)
    temp, count, fresh, duration = struct.unpack_from('<hBBH', payload, 38)
    if (not boot or not op or not 2 <= count <= 32 or fresh > count
            or not 10000 <= duration <= 30000 or not -10000 <= temp <= 15000
            or any(abs(g) > 2500 * duration for g in gyro)
            or any(abs(a) > 32000 for a in accel)):
        raise ValueError('F04 session/count/duration/physical bounds invalid')
    return Summary(payload[1], boot, op, seq, end, gyro, accel, temp, count, fresh, duration)


def decode_report(report):
    if len(report) != 64 or any(report[53:]):
        raise ValueError('F04 HID report size/padding invalid')
    sample = decode(report[:48])
    return sample, struct.unpack_from('b', report, 48)[0], struct.unpack_from('>I', report, 49)[0]


class Stream:
    """One receiver/tracker/boot/operation. Reorder up to 128 windows.

    Finalized old frames are counted as late, never used to silently revise a
    reported gap. Exact duplicates are recognized within 256 recent windows;
    older duplicates cannot be distinguished from other late frames.
    """
    def __init__(self, key, emit=lambda row: None, horizon=128):
        if not 1 <= horizon <= 4096:
            raise ValueError('reorder horizon must be 1..4096')
        self.key, self.emit, self.horizon = key, emit, horizon
        self.pending, self.recent = {}, OrderedDict()
        self.next = None
        self.unique = self.duplicates = self.late = self.missing = 0
        self.reordered = 0
        self.end = None
        self.gaps = []
        self.leading_unobserved = None
        self.first_end = self.last_end = None
        self.first_host_ns = self.last_host_ns = None
        self.elapsed_us = self.total_duration_us = self.samples = 0
        self.fresh = self.held = 0
        self.last_sequence = None
        self.interval_discontinuities = self.empty_accel_windows = 0

    def add(self, payload, host_ns=None, rssi=None, receiver_ticks=None):
        s = decode(payload)
        if self.key[1:] != (s.tracker, s.boot, s.operation):
            raise ValueError('mixed F04 stream identities')
        if self.next is None:
            self.next = s.sequence
            self.leading_unobserved = s.sequence
        for mapping in (self.pending, self.recent):
            if s.sequence in mapping:
                old = mapping[s.sequence][0] if mapping is self.pending else mapping[s.sequence]
                if old != payload:
                    raise ValueError('conflicting F04 duplicate payload')
                self.duplicates += 1
                return
        distance = (s.sequence - self.next) & MASK
        if distance >= (1 << 31):
            self.late += 1
            return
        if distance:
            self.reordered += 1
        self.pending[s.sequence] = (payload, s, host_ns, rssi, receiver_ticks)
        self._flush(False)

    def _flush(self, final):
        while self.pending:
            if self.next not in self.pending:
                distances = [((seq - self.next) & MASK, seq) for seq in self.pending]
                if not final and max(d for d, _ in distances) < self.horizon:
                    break
                gap, nearest = min(distances)
                self.gaps.append({'first': self.next, 'last': (nearest - 1) & MASK, 'count': gap})
                self.missing += gap
                self.next = nearest
            payload, s, host_ns, rssi, ticks = self.pending.pop(self.next)
            self.recent[self.next] = payload
            if len(self.recent) > 2 * self.horizon:
                self.recent.popitem(last=False)
            if self.last_end is not None:
                dt = (s.end_us - self.last_end) & MASK
                if not 0 < dt < (1 << 31):
                    raise ValueError('ambiguous or backward estimated timestamp')
                self.elapsed_us += dt
                if ((s.sequence-self.last_sequence) & MASK) == 1 and abs(dt-s.duration_us) > 3:
                    self.interval_discontinuities += 1
            else:
                self.first_end = s.end_us
            self.last_end = s.end_us
            self.last_sequence = s.sequence
            self.empty_accel_windows += s.accel_fresh == 0
            self.total_duration_us += s.duration_us
            self.samples += s.samples
            self.fresh += s.accel_fresh
            self.held += s.accel_held
            self.unique += 1
            if host_ns is not None:
                self.first_host_ns = host_ns if self.first_host_ns is None else min(self.first_host_ns, host_ns)
                self.last_host_ns = host_ns if self.last_host_ns is None else max(self.last_host_ns, host_ns)
            row = dict(receiver_serial=self.key[0], **asdict(s), accel_held=s.accel_held,
                       host_monotonic_ns=host_ns, rssi_dbm=rssi, receiver_ticks=ticks,
                       time_quality='estimated', calibrated=False)
            self.emit(row)
            self.next = (self.next + 1) & MASK

    def finish(self):
        self._flush(True)
        return dict(receiver_serial=self.key[0], tracker=self.key[1], boot=self.key[2],
                    operation=self.key[3], unique=self.unique, duplicates=self.duplicates,
                    late=self.late, missing=self.missing, gaps=self.gaps, reordered=self.reordered,
                    max_gap_windows=max((g['count'] for g in self.gaps), default=0),
                    end_manifest=asdict(self.end) if self.end else None,
                    trailing_unobserved=((self.end.sequence_end-self.next) & MASK
                                         if self.end and self.next is not None else None),
                    leading_unobserved=self.leading_unobserved, samples=self.samples,
                    accel_fresh=self.fresh, accel_held=self.held,
                    interval_discontinuities=self.interval_discontinuities,
                    empty_accel_windows=self.empty_accel_windows,
                    elapsed_us=self.elapsed_us, total_duration_us=self.total_duration_us,
                    rate_hz=((self.unique - 1) * 1e6 / self.elapsed_us if self.elapsed_us else None),
                    host_elapsed_ns=(self.last_host_ns-self.first_host_ns if self.first_host_ns is not None else None),
                    host_rate_hz=((self.unique-1)*1e9/(self.last_host_ns-self.first_host_ns)
                                  if self.first_host_ns is not None and self.last_host_ns>self.first_host_ns else None),
                    time_quality='estimated', calibrated=False)

    def set_end(self, end):
        if self.key[1:] != (end.tracker, end.boot, end.operation):
            raise ValueError('mixed F04 end identities')
        if self.end is not None and self.end != end:
            raise ValueError('conflicting F04 end manifest')
        self.end = end


def replay(path, emit=lambda row: None):
    streams = {}
    with Path(path).open(encoding='utf-8') as f:
        header = json.loads(next(f))
        receiver = header.get('receiver_serial')
        if header.get('format') != 'F04HID1' or not isinstance(receiver, str) or not receiver:
            raise ValueError('F04 recording identity/header missing')
        previous_host_ns = -1
        for lineno, line in enumerate(f, 2):
            try:
                entry = json.loads(line)
                if entry.get('event') == 'capture-stop':
                    continue
                host_ns = entry['host_monotonic_ns']
                if type(host_ns) is not int or host_ns < previous_host_ns:
                    raise ValueError('host monotonic timestamp regressed')
                previous_host_ns = host_ns
                report = bytes.fromhex(entry['report_hex'])
                s, rssi, ticks = decode_frame_report(report)
                key = receiver, s.tracker, s.boot, s.operation
                if key not in streams:
                    streams[key] = Stream(key, emit)
                if isinstance(s, EndManifest):
                    streams[key].set_end(s)
                else:
                    streams[key].add(report[:48], host_ns, rssi, ticks)
            except (ValueError, KeyError, TypeError) as e:
                raise ValueError(f'{path}:{lineno}: {e}') from e
    return [stream.finish() for stream in streams.values()]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('recording', type=Path)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=False)
    with (a.out / 'windows.jsonl').open('x', encoding='utf-8') as f:
        results = replay(a.recording, lambda row: f.write(json.dumps(row) + '\n'))
    (a.out / 'summary.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    print(json.dumps(results, indent=2))
    return 0 if results else 2


if __name__ == '__main__':
    raise SystemExit(main())

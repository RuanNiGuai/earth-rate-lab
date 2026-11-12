"""Transport gates only. Never claim sensor timing or calibration accuracy.

Recording continues after READY even when diagnostics fail. A zero-loss gate
is deliberate: a small percentage can hide all the missing data in one turn.
"""
from collections import Counter
import json
from pathlib import Path

from f04_summary import Stream, EndManifest, decode_frame_report


class CaptureQuality:
    def __init__(self, receiver, targets):
        self.receiver = receiver
        self.targets = set(map(int, targets))
        self.streams = {}
        self.identities = {}
        self.last_seen = {}
        self.errors = Counter()
        self.previous_host_ns = -1

    def add(self, raw, host_ns):
        try:
            if type(host_ns) is not int or host_ns < self.previous_host_ns:
                raise ValueError('host monotonic timestamp regressed')
            self.previous_host_ns = host_ns
            s, rssi, ticks = decode_frame_report(raw)
            if s.tracker not in self.targets:
                raise ValueError('unexpected tracker')
            key = self.receiver, s.tracker, s.boot, s.operation
            if s.tracker in self.identities and self.identities[s.tracker] != key:
                raise ValueError('tracker restarted or operation changed')
            self.identities[s.tracker] = key
            stream = self.streams.setdefault(s.tracker, Stream(key))
            if isinstance(s, EndManifest):
                stream.set_end(s)
            else:
                before = stream.unique + len(stream.pending)
                stream.add(raw[:48], host_ns, rssi, ticks)
                if stream.unique + len(stream.pending) > before:
                    self.last_seen[s.tracker] = host_ns
        except (ValueError, TypeError) as e:
            self.errors[str(e)] += 1

    def ready(self, now_ns, minimum=200):
        """Every member has >=4 seconds of contiguous, fresh unique windows."""
        return not self.errors and all(
            i in self.streams and self.streams[i].unique >= minimum
            and self.streams[i].leading_unobserved == 0 and not self.streams[i].missing
            and not self.streams[i].pending and not self.streams[i].late
            and not self.streams[i].interval_discontinuities and not self.streams[i].empty_accel_windows
            and self.streams[i].end is None
            and 0 <= now_ns-self.last_seen.get(i, 0) < 1_000_000_000
            for i in self.targets)

    def readiness_failures(self, now_ns, minimum=200):
        """Explain live admission without flushing pending reorder buffers."""
        failures=[]
        if self.errors:
            failures.append('解码／身份／时间异常：'+', '.join(self.errors))
        for i in sorted(self.targets):
            stream=self.streams.get(i)
            if stream is None:
                failures.append(f'#{i} 未收到数据')
                continue
            issues=[]
            if stream.unique<minimum:issues.append(f'连续数据不足（已确认{stream.unique}/{minimum}窗）')
            if stream.leading_unobserved or stream.missing or stream.pending or stream.late:
                issues.append(f'数据不连续（首部缺{stream.leading_unobserved}、中间缺{stream.missing}、待重排{len(stream.pending)}、迟到{stream.late}）')
            if stream.interval_discontinuities or stream.empty_accel_windows:
                issues.append('采样时间不连续或缺少加速度')
            if stream.end is not None:issues.append('采集已结束')
            if not 0<=now_ns-self.last_seen.get(i,0)<1_000_000_000:issues.append('超过1秒未收到新数据')
            if issues:failures.append(f'#{i} '+ '、'.join(issues))
        return failures

    def finish(self, stop_confirmed=None, receiver_dropped=None):
        rows, reasons = [], []
        if self.errors:
            reasons.append('decode, identity or timestamp errors')
        for i in sorted(self.targets):
            stream = self.streams.get(i)
            if stream is None:
                reasons.append(f'tracker {i}: no data or manifest')
                continue
            try:
                row = stream.finish()
            except ValueError as e:
                reasons.append(f'tracker {i}: {e}')
                continue
            end = stream.end
            failed = []
            if row['unique'] < 200:
                failed.append('too few windows')
            if row['leading_unobserved'] != 0 or row['missing'] or row['late']:
                failed.append('leading/internal missing or late windows')
            if row['interval_discontinuities'] or row['empty_accel_windows']:
                failed.append('sensor interval discontinuity or no fresh accelerometer')
            if end is None:
                failed.append('no end manifest; tail unknown')
            else:
                if end.invalid_windows or end.queue_drop or end.tail_drop:
                    failed.append('device invalid/queue/tail drops')
                if row['unique'] != end.sequence_end or row['trailing_unobserved'] != 0:
                    failed.append('end-to-end sequence/count mismatch')
                row['missing_total'] = max(0, end.sequence_end-row['unique'])
                row['missing_fraction'] = row['missing_total']/end.sequence_end if end.sequence_end else None
            row['failures'] = failed
            reasons.extend(f'tracker {i}: {x}' for x in failed)
            rows.append(row)
        if stop_confirmed is not True:
            reasons.append('receiver stop unconfirmed')
        if receiver_dropped is None:
            reasons.append('receiver drop counter unavailable')
        elif receiver_dropped != 0:
            reasons.append('receiver dropped reports')
        return dict(schema='F04QUALITY1', transport_pass=not reasons, failures=reasons,
                    receiver_serial=self.receiver, targets=sorted(self.targets),
                    stop_confirmed=stop_confirmed, receiver_dropped=receiver_dropped,
                    errors=dict(self.errors), streams=rows,
                    accuracy_validated=False, time_quality='estimated')


def failure_message(quality):
    """Explain transport rejection without inferring an RF or USB root cause."""
    if quality['transport_pass']:
        return ''
    details=[]
    rows={r['tracker']:r for r in quality['streams']}
    for slot in quality['targets']:
        row=rows.get(slot)
        if row is None:
            details.append(f'#{slot} 没有可核验的数据流')
            continue
        if not row['failures']:continue
        issues=[]
        end=row.get('end_manifest')
        missing=row.get('missing_total')
        if missing:
            fraction=row.get('missing_fraction')
            percent=f'，占{fraction:.2%}' if fraction is not None else ''
            issues.append(f'缺失{missing}个窗口{percent}')
            if row.get('max_gap_windows'):
                issues.append(f"最长连续缺失{row['max_gap_windows']}窗（50 Hz下约{row['max_gap_windows']/50:.2f}秒）")
        if end is None:issues.append('缺少结束清单，尾部完整性未知')
        else:
            for key,label in [('queue_drop','追踪器发送队列溢出'),('invalid_windows','传感器无效窗口'),('tail_drop','停止时未发送尾部')]:
                if end[key]:issues.append(f'{label}{end[key]}窗')
        if row.get('late'):issues.append(f"超出重排范围的迟到报告{row['late']}个")
        if row.get('interval_discontinuities'):issues.append('传感器时间不连续')
        if row.get('empty_accel_windows'):issues.append('窗口缺少新加速度采样')
        if row['unique']<200:issues.append(f"有效窗口不足（{row['unique']}/200）")
        if not issues:issues.append('生成、发送与保存计数不一致')
        details.append(f'#{slot} '+ '、'.join(issues))
    if quality['errors']:details.append('解码／身份／时间异常：'+', '.join(quality['errors']))
    stop='已确认' if quality['stop_confirmed'] is True else '未确认'
    drops=quality['receiver_dropped']
    details.append(f'结束{stop}，接收器丢弃'+('计数未知' if drops is None else f'{drops}个报告'))
    return '传输完整性检查未通过：'+'；'.join(details)+'。原录制和其他已通过步骤已保留；不能直接进入常规求解。详细记录见录制目录 quality.json。'


def audit_recording(recording, stop_confirmed=None, receiver_dropped=None):
    with Path(recording).open(encoding='utf8') as f:
        header = json.loads(next(f))
        if header.get('format') != 'F04HID1' or not header.get('receiver_serial') or not header.get('targets'):
            raise ValueError('Recording must declare receiver and frozen targets')
        quality = CaptureQuality(header['receiver_serial'], header['targets'])
        for line in f:
            entry = json.loads(line)
            if entry.get('event') == 'capture-stop':
                stop_confirmed = entry['confirmed']
                receiver_dropped = entry['receiver_dropped']
            else:
                quality.add(bytes.fromhex(entry['report_hex']), entry['host_monotonic_ns'])
    return quality.finish(stop_confirmed, receiver_dropped)

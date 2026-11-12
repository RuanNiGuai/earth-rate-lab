"""Fleet F04 recorder with a common READY barrier; no per-pose interruption.

Preserves complete receiver reports, including duplicates and damaged frames.
Gaps/reboots/identity conflicts are diagnostics, never automatic operator stops.
F04 is a timed 50-Hz integral summary, NOT a full-rate raw sample recording.
Stops only when STOP_RECORDING is created, or on unrecoverable I/O failure.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import re
import time

import batch_calibration as b
from f04_summary import decode_report
from capture_quality import CaptureQuality
from capture_preflight import check_capture_conflicts


def pose_ready(latest_motion, kind):
    axis=0 if kind=='poses' else 'xyz'.index(kind)
    return set(latest_motion)==set(range(10)) and all(
        abs(accel[axis])>920 and (kind!='poses' or accel[axis]>920)
        and .9e6<sum(a*a for a in accel)<1.1e6
        and sum((g/duration)**2 for g in gyro)<1
        for gyro,duration,accel in latest_motion.values())


def readiness_error(quality, latest_motion, kind, now_ns, output):
    reasons=quality.readiness_failures(now_ns)
    if kind:
        axis=0 if kind=='poses' else 'xyz'.index(kind)
        wrong=[];moving=[]
        for slot,(gyro,duration,accel) in sorted(latest_motion.items()):
            if not (abs(accel[axis])>920 and (kind!='poses' or accel[axis]>920)
                    and .9e6<sum(a*a for a in accel)<1.1e6):wrong.append(slot)
            if sum((g/duration)**2 for g in gyro)>=1:moving.append(slot)
        if wrong:
            face='+X' if kind=='poses' else kind.upper()
            reasons.append(f'{wrong}号朝向不符合要求，请将{face}轴竖直放稳')
        if moving:reasons.append(f'{moving}号仍检测到转动，请保持静止')
    if not reasons:reasons.append('尚未同时保持正确朝向并静止满2秒')
    return '录制准备未通过：'+'；'.join(reasons)+f'。本轮已停止，请先处理原因再重录；诊断保存在 {output}。'


def record(directory, passive=False, cancel_file=None, progress=None, kind=None):
    import hid
    import serial
    from serial.tools import list_ports
    state=json.loads((directory/'session.json').read_text(encoding='utf-8'))
    receiver=state['receiver_serial']
    targets=b.validate_targets(state['targets'])
    check_capture_conflicts()
    marker=directory/'summary-owner.json'
    if marker.exists():
        b.check(not json.loads(marker.read_text())['active'],'Previous summary stop unverified')
    ports=[p for p in list_ports.comports() if (p.vid,p.pid,p.serial_number)==(0x1209,0x7690,receiver)]
    devices=[d for d in hid.enumerate(0x1209,0x7690) if d.get('serial_number')==receiver and d.get('usage_page')==0xff00]
    b.check(len(ports)==len(devices)==1,'Receiver identity/interface mismatch')
    output=directory/('record-only-'+str(time.time_ns()))
    output.mkdir()
    journal=b.Journal(output/'events.jsonl')
    radio=b.Radio(receiver,targets,journal)
    console=dev=stream=None
    owned=False
    text=''
    counts=Counter()
    last_seen={}
    identities={}
    diagnostics=Counter()
    latest_motion={}
    quality=CaptureQuality(receiver, targets)
    confirmed=False
    receiver_dropped=None
    active=None
    started=time.monotonic()
    last_flush=started
    last_update=started
    stop_file=output/'STOP_RECORDING'
    def stopping():
        return stop_file.exists() or (cancel_file is not None and Path(cancel_file).exists())
    def pump(seconds):
        nonlocal text,last_flush,last_update
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            data=console.read(8192)
            if data:
                chunk=data.decode('utf-8','replace')
                text=(text+chunk)[-256000:]
                journal.add(event='console',text=chunk)
            try:
                radio.read(0)
            except ValueError as e:
                name=str(e)
                if diagnostics[name]==0:
                    journal.add(event='diagnostic',error=name)
                diagnostics[name]+=1
            for _ in range(256):
                raw=bytes(dev.read(64))
                if not raw:
                    break
                if stream:
                    host=time.monotonic_ns()
                    stream.write(json.dumps(dict(host_monotonic_ns=host,report_hex=raw.hex()))+'\n')
                    quality.add(raw,host)
                    try:
                        s,_,_=decode_report(raw)
                        key=(s.tracker,s.boot,s.operation)
                        identities[str(s.tracker)]=list(key)
                        counts[s.tracker]+=1
                        last_seen[s.tracker]=time.monotonic()
                        latest_motion[s.tracker]=(s.gyro_delta_udeg,s.duration_us,s.accel_mean_mg)
                        if s.tracker not in targets:
                            diagnostics['unexpected tracker']+=1
                    except ValueError as e:
                        # End manifests are control frames, not damaged IMU windows.
                        if not (len(raw)==64 and raw[2:4]==b'\x01\x02'):
                            diagnostics[str(e)]+=1
            now=time.monotonic()
            if stream and now-last_flush>=1:
                stream.flush()
                os.fsync(stream.fileno())
                last_flush=now
            if owned and now-last_update>=1:
                live=dict(utc=b.utc(),elapsed=now-started,output=str(output),counts=dict(counts),
                          last_seen_age={k:now-last_seen.get(k,started) for k in targets},
                          identities=identities,diagnostics=dict(diagnostics),
                          faces={k:('+' if row[2][max(range(3),key=lambda j:abs(row[2][j]))]>=0 else '-')+'XYZ'[max(range(3),key=lambda j:abs(row[2][j]))] for k,row in latest_motion.items()},
                          mode='record only; no calibration installed')
                if not b.atomic_save(output/'live.json',live,best_effort=True):
                    diagnostics['live updates deferred by Windows sharing conflict']+=1
                if progress:
                    progress(live)
                else:
                    print(json.dumps(live),flush=True)
                last_update=now
            time.sleep(.001)
    def command(value,seconds=1):
        nonlocal text
        text=''
        journal.add(event='command',command=value)
        payload=(value+'\r\n').encode('ascii')
        b.check(console.write(payload)==len(payload),'Partial console write')
        pump(seconds)
        return text
    try:
        b.check(not stopping(), 'Capture cancelled before opening devices')
        radio.bind()
        console=serial.Serial(ports[0].device,115200,timeout=0,write_timeout=2)
        dev=hid.device()
        dev.open_path(devices[0]['path'])
        dev.set_nonblocking(True)
        pump(.3)
        b.check('Data collection inactive' in command('collect'),'Another collection active')
        b.check('No active session' in command('ota'),'OTA active')
        pairs=re.findall(r'^([0-9A-F]{12})\s*$',command('list'),re.M)
        b.check({i:a for i,a in enumerate(pairs)}==targets,'All-paired selection differs from frozen fleet')
        stream=(output/'reports.jsonl').open('x',encoding='utf-8')
        stream.write(json.dumps(dict(format='F04HID1',receiver_serial=receiver,targets=targets,
                                    time_quality='estimated',calibrated=False,purpose='passive motion recording'))+'\n')
        b.atomic_save(marker,dict(active=True,receiver_serial=receiver,targets=targets,utc=b.utc(),output=str(output)))
        owned=True
        b.check('Batch collection STARTED' in command('collectall 50',2),'Summary start refused')
        active=dict(output=str(output),stop_file=str(stop_file),started_at=b.utc(),ready=False)
        b.atomic_save(directory/'active-recording.json',active)
        deadline=time.monotonic()+45
        ready=False
        aligned_since=None
        while time.monotonic()<deadline and not stopping():
            pump(.2)
            ready=quality.ready(time.monotonic_ns())
            if kind:
                aligned=pose_ready(latest_motion,kind)
                aligned_since=(aligned_since or time.monotonic()) if aligned else None
                ready=ready and aligned_since is not None and time.monotonic()-aligned_since>=2
            if ready or (passive and time.monotonic()-started>=10):
                break
        journal.add(event='preflight',all_ten_fresh=ready,counts=dict(counts))
        if not passive and not stopping():
            b.check(ready,readiness_error(quality,latest_motion,kind,time.monotonic_ns(),output))
        active.update(ready=ready,ready_at=b.utc() if ready else None)
        b.atomic_save(directory/'active-recording.json',active)
        if progress:
            progress(dict(stage='ready' if ready and not stopping() else 'stopping', **active))
        print(json.dumps(dict(recording_started=True,all_ten_fresh=ready,output=str(output))),flush=True)
        while not stopping():
            pump(.2)
        journal.add(event='operator-stop',utc_request=(stop_file.read_text(encoding='utf-8-sig')
                    if stop_file.exists() else 'Helper stop/cancel'))
    finally:
        try:
            if owned and console and dev:
                if progress:
                    progress(dict(stage='stopping', output=str(output), ready=False))
                stop=command('collectstop',10)
                inactive=command('collect')
                confirmed=('unconfirmed=' not in stop and 'Batch data collection STOPPED' in stop
                           and 'Data collection inactive' in inactive)
                totals=re.findall(r'Batch data collection STOPPED \(received: (\d+), sent: (\d+), dropped: (\d+)\)',stop)
                receiver_dropped=int(totals[-1][2]) if totals else None
                journal.add(event='stop-result',confirmed=confirmed,response=stop)
                stream.write(json.dumps(dict(event='capture-stop',confirmed=confirmed,
                                             receiver_dropped=receiver_dropped))+'\n')
                b.atomic_save(marker,dict(active=not confirmed,stopped_at=b.utc(),output=str(output)))
                b.check(confirmed,'Some summary stops unconfirmed; inspect before another capture')
        finally:
            try:
                if stream:
                    try:
                        stream.flush()
                        os.fsync(stream.fileno())
                    finally:
                        stream.close()
                    b.atomic_save(output/'quality.json',quality.finish(confirmed,receiver_dropped))
                    if active is not None:
                        active.update(ready=False,stopped=True,stop_confirmed=confirmed)
                        b.atomic_save(directory/'active-recording.json',active)
            finally:
                # Metadata persistence errors must not leak hardware ownership.
                try:
                    if dev:dev.close()
                finally:
                    try:
                        if console:console.close()
                    finally:
                        radio.close()
    print(json.dumps(dict(stopped=True,output=str(output),counts=dict(counts),diagnostics=dict(diagnostics))),flush=True)
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--passive',action='store_true',help='Record even if initial READY fails; quality still enforced separately')
    args=parser.parse_args()
    with b.lock(args.session/'LOCK'):
        record(args.session,args.passive)

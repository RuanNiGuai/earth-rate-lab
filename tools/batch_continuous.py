"""Continuous, identity-bound fixture acquisition (observations, not installation).

Receiver console admission uses per-tracker mailboxes. Only matching tracker
CAL_STATUS events prove completion. A single explicitly selected F04 stream is
used for fixture motion/face gating, never as full-rate calibration input.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict
import json
import math
from pathlib import Path
import re
import time

import batch_calibration as b
from f04_summary import decode_report

COMMANDS = {'status': 'status', 'cancel': 'capture cancel',
            'earth-start': 'earth start', 'earth-pose': 'earth stop',
            'gsens-start': 'gsens start', 'gsens-pose': 'gsens pose'}
ADMISSION = re.compile(r'F05 ARC tid=(\d+) flag=0x([0-9a-fA-F]+) seq=(\d+) status=(\d+) \(transport only\)')


def admission(text, slot, flag):
    rows = ADMISSION.findall(text)
    b.check(len(rows) == 1, 'Missing/ambiguous console admission; never retry blindly')
    tid, code, seq, status = rows[0]
    b.check((int(tid), int(code, 16)) == (slot, flag), 'Console admission target/command mismatch')
    b.check(1 <= int(seq) <= 255 and int(status) == 6, f'Console transport refused: {status}')
    return int(seq)


def length(v):
    return math.sqrt(sum(x*x for x in v))


class Fixture:
    """Fail closed on lost/reordered windows, reboot, operation change or gaps.

    Native capture remains on all trackers. F04 gates operator instructions
    only; bias-subtracted deltas here are approximate and are not saved as
    calibration observations.
    """
    def __init__(self, slot):
        self.slot = slot
        self.key = None
        self.last = None
        self.host_time = 0.
        self.windows = deque(maxlen=300)
        self.bias = [0., 0., 0.]
        self.delta = [0., 0., 0.]
        self.path = 0.
        self.moving = False

    def add(self, s, now):
        b.check(s.tracker == self.slot, 'Unexpected summary tracker')
        key = (s.boot, s.operation)
        b.check(self.key in (None, key), 'Tracker rebooted/summary operation changed')
        if self.last is not None:
            distance = (s.sequence-self.last.sequence) & 0xffffffff
            if distance == 0:
                b.check(s == self.last, 'Conflicting duplicate summary')
                return
            b.check(distance == 1, 'Summary loss/reorder: motion estimate is uncertain')
            elapsed = (s.end_us-self.last.end_us) & 0xffffffff
            b.check(abs(elapsed-s.duration_us) <= 30, 'Summary sensor timeline discontinuity')
            b.check(now-self.host_time < .5, 'Stale summary delivery')
        b.check(s.accel_fresh > 0, 'No fresh accelerometer in summary window')
        self.key, self.last, self.host_time = key, s, now
        rate = [s.gyro_delta_udeg[i]/s.duration_us for i in range(3)]
        speed = length([rate[i]-self.bias[i] for i in range(3)])
        self.windows.append((now, s, rate, speed))
        for i in range(3):
            self.delta[i] += s.gyro_delta_udeg[i]/1e6-self.bias[i]*s.duration_us/1e6
        self.path += speed*s.duration_us/1e6
        if speed > 4:
            self.moving = True

    def stable(self, now, seconds=3.):
        if not self.windows or now-self.host_time > .25:
            return None
        rows = [r for r in self.windows if now-r[0] <= seconds+.10]
        if len(rows) < seconds*45 or rows[-1][0]-rows[0][0] < seconds-.08:
            return None
        if any(r[3] > 1.0 for r in rows):
            return None
        means = [sum(r[2][i] for r in rows)/len(rows) for i in range(3)]
        if any(max(abs(r[2][i]-means[i]) for r in rows) > .35 for i in range(3)):
            return None
        accel = [sum(r[1].accel_mean_mg[i] for r in rows)/len(rows)/1000 for i in range(3)]
        norm = length(accel)
        if not .96 <= norm <= 1.04:
            return None
        if any(length([r[1].accel_mean_mg[i]/1000-accel[i] for i in range(3)]) > .03 for r in rows):
            return None
        axis = max(range(3), key=lambda i: abs(accel[i]))
        if abs(accel[axis])/norm < .98:
            return None
        return dict(face=('+' if accel[axis]>0 else '-')+'XYZ'[axis], accel=accel, bias=means)

    def reset_motion(self, stable):
        self.bias = stable['bias']
        self.delta = [0., 0., 0.]
        self.path = 0.
        self.moving = False

    def check_turn(self, degrees, axis):
        along = self.delta['XYZ'.index(axis)]
        off = length([v for i, v in enumerate(self.delta) if i != 'XYZ'.index(axis)])
        b.check(abs(along-degrees) <= 12 and off < 15 and self.path < abs(degrees)+40,
                f'Wrong fixture turn: delta={self.delta}, expected {degrees} about {axis}')


class Controller:
    def __init__(self, directory, slot, audio_dir=None):
        self.directory = directory
        self.path = directory/'session.json'
        self.state = json.loads(self.path.read_text(encoding='utf-8'))
        self.targets = b.validate_targets(self.state['targets'])
        b.check(slot in self.targets and self.state.get('initialized') and not self.state.get('inflight'), 'Session not ready')
        b.check(all(not r.get('active') and not r.get('uncertain') for r in self.state['devices'].values()), 'All captures must be idle and certain')
        self.marker = directory/'summary-owner.json'
        if self.marker.exists():
            b.check(not json.loads(self.marker.read_text())['active'], 'Previous summary stop unverified')
        self.output = directory/('continuous-'+str(time.time_ns()))
        self.output.mkdir()
        self.journal = b.Journal(self.output/'events.jsonl')
        self.radio = self.console = self.summary = None
        self.fixture = Fixture(slot)
        self.text = ''
        self.events = []
        self.active_txs = {}
        self.capture_keys = {}
        self.cancelling_keys = set()
        self.last_console_sequence = None
        self.used = set()
        self.owned = False
        self.fatal = None
        self.armed = False
        self.audio_dir = audio_dir
        for file in [directory/'radio.jsonl', *directory.glob('continuous-*/events.jsonl')]:
            if file.exists():
                for line in file.read_text(encoding='utf-8').splitlines():
                    row = json.loads(line)
                    if row.get('event') == 'status':
                        s = row['status']
                        self.used.add((s['slot'], s['sequence'], s['flag']))

    def open(self):
        import hid
        import serial
        from serial.tools import list_ports
        receiver = self.state['receiver_serial']
        ports = [p for p in list_ports.comports() if (p.vid,p.pid,p.serial_number)==(0x1209,0x7690,receiver)]
        devices = [d for d in hid.enumerate(0x1209,0x7690) if d.get('serial_number')==receiver and d.get('usage_page')==0xff00]
        b.check(len(ports)==len(devices)==1, 'Receiver identity/interface mismatch')
        self.radio = b.Radio(receiver,self.targets,self.journal)
        self.radio.bind()
        self.console = serial.Serial(ports[0].device,115200,timeout=0,write_timeout=2)
        self.summary = hid.device()
        self.summary.open_path(devices[0]['path'])
        self.summary.set_nonblocking(True)
        self.pump(.4, cleanup=True)
        b.check('Data collection inactive' in self.command('collect'), 'Another collection is active')
        b.check('No active session' in self.command('ota'), 'OTA is active')
        # Actual console sequence comes from the receiver. Status is read-only
        # and exempt from tracker dedupe, so it safely establishes the sequence.
        self.action('status')
        self.events.clear()
        self.fatal = None
        self.armed = True

    def tick(self, cleanup=False):
        if not cleanup:
            b.check(not (self.directory/'STOP_CONTINUOUS').exists(), 'Operator stop requested')
            b.check(not self.fatal, self.fatal)
        if self.console:
            chunk = self.console.read(8192)
            if chunk:
                decoded = chunk.decode('utf-8','replace')
                self.text += decoded
                b.check(len(self.text)<512*1024, 'Console response exceeded bound')
                self.journal.add(event='console',text=decoded)
        if self.radio:
            for s in self.radio.read(0):
                self.observe_status(s)
            if self.armed and not cleanup:
                now=time.monotonic()
                b.check(all(now-self.radio.live.get(k,0)<5 and now-self.radio.registered.get(k,0)<5 for k in self.targets), 'Tracker traffic/identity stale')
        if self.summary:
            for _ in range(64):
                raw=bytes(self.summary.read(64))
                if not raw:
                    break
                if self.owned:
                    self.journal.add(event='summary',report_hex=raw.hex())
                    if not cleanup:
                        sample,_,_=decode_report(raw)
                        self.fixture.add(sample,time.monotonic())
        if not cleanup:
            b.check(not self.fatal,self.fatal)
        time.sleep(.003)

    def observe_status(self, s):
        self.events.append(s)
        tx = self.active_txs.get((s.slot,s.sequence,s.flag))
        if tx:
            tx.observe(s)
        expected_cancel = ((s.slot,s.sequence,s.flag) in self.cancelling_keys
                           and s.phase==5 and s.error==3)
        if self.armed and not expected_cancel and (s.phase in (3,5) or s.error):
            self.fatal = f'Tracker {s.slot} failed: {b.ERRORS.get(s.error,s.phase)}'

    def pump(self, seconds, cleanup=False):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.tick(cleanup)

    def command(self, command, seconds=.7, cleanup=False):
        self.text=''
        self.journal.add(event='console-intent',command=command)
        payload=(command+'\r\n').encode('ascii')
        b.check(self.console.write(payload)==len(payload), 'Partial console write; outcome uncertain')
        self.pump(seconds,cleanup)
        return self.text

    def send_target(self, slot, action, cleanup=False):
        flag=b.FLAGS[action]
        # Do not let a console sequence collide with any prior use of the same
        # device/command, including earlier HID sessions. Consume status instead.
        while action!='status' and self.last_console_sequence is not None and (slot,self.last_console_sequence%255+1,flag) in self.used:
            skip=self.send_target(slot,'status',cleanup)
            end=time.monotonic()+8
            while not skip.succeeded and not skip.failure and time.monotonic()<end:
                self.tick(cleanup)
            b.check(skip.succeeded,'Could not advance console sequence safely')
        expected=None if self.last_console_sequence is None else self.last_console_sequence%255+1
        self.text=''
        self.journal.add(event='about-to-send-console',slot=slot,flag=flag,expected_sequence=expected)
        if action=='cancel' and slot in self.capture_keys:
            self.cancelling_keys.add(self.capture_keys[slot])
        payload=f'send {slot} archive {COMMANDS[action]}\r\n'.encode('ascii')
        b.check(self.console.write(payload)==len(payload),'Partial console write; outcome uncertain')
        end=time.monotonic()+2
        while not ADMISSION.search(self.text) and time.monotonic()<end:
            self.tick(cleanup)
        seq=admission(self.text,slot,flag)
        b.check(expected in (None,seq),'Console sequence changed: another owner or receiver reset')
        self.last_console_sequence=seq
        b.check(action=='status' or (slot,seq,flag) not in self.used,'Reused mutation sequence')
        self.used.add((slot,seq,flag))
        tx=b.Transaction(slot,seq,flag,action in b.SAVES)
        self.active_txs[(slot,seq,flag)]=tx
        if action in ('earth-start','earth-pose','gsens-start','gsens-pose'):
            self.capture_keys[slot]=(slot,seq,flag)
        for s in self.events:
            tx.observe(s)
        return tx

    def action(self, action, timeout=40, cleanup=False, hold_still=False):
        if not cleanup:
            b.stage_allowed(self.state,action,self.targets)
        number=self.state['next_sequence']
        self.state['next_sequence']+=1  # evidence index, console wire sequences are per tracker
        self.state['inflight']=dict(action=action,transport='console',record=number,utc=b.utc())
        b.atomic_save(self.path,self.state)
        self.events.clear()  # old same-sequence status after wire wrap is not fresh evidence
        transactions={}
        try:
            for slot in self.targets:
                transactions[slot]=self.send_target(slot,action,cleanup)
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                if all(t.succeeded or t.failure for t in transactions.values()):
                    break
                self.tick(cleanup)
                if hold_still:
                    b.check(self.fixture.windows and self.fixture.windows[-1][3]<3, 'Fixture moved during capture')
                    b.check(time.monotonic()-self.fixture.host_time<.5,'Summary lost during capture')
            results={str(k):dict(success=t.succeeded,error=t.failure or (None if t.succeeded else 'timeout; outcome uncertain'),
                                status=asdict(t.status) if t.status else None) for k,t in transactions.items()}
            for k in self.targets:
                results.setdefault(str(k),dict(success=False,error='not dispatched',status=None))
            b.save_new(self.output/f'{number:03d}-{action}.json',dict(action=action,results=results,utc=b.utc()))
            b.advance(self.state,action,results)
            if action in b.SAVES or action=='cancel':
                for slot in transactions:
                    if results[str(slot)]['success']:
                        self.capture_keys.pop(slot,None)
            self.state['inflight']=None
            b.atomic_save(self.path,self.state)
            print(json.dumps(dict(action=action,results=results),ensure_ascii=False),flush=True)
            b.check(all(r['success'] for r in results.values()),f'{action}: incomplete tracker completion')
            return results
        except BaseException:
            for row in self.state['devices'].values():
                row['uncertain']=True
            b.atomic_save(self.path,self.state)
            raise

    def start_summary(self):
        owner=dict(active=True,receiver_serial=self.state['receiver_serial'],slot=self.fixture.slot,utc=b.utc(),output=str(self.output))
        b.atomic_save(self.marker,owner)
        self.owned=True
        b.check('Batch collection STARTED' in self.command(f'collectall 50 {self.fixture.slot}',2),'Summary start refused')
        self.pump(4)
        stable=self.wait_stable(timeout=8)
        self.fixture.reset_motion(stable)
        return stable

    def wait_stable(self, timeout=55, moved=False, face=None, unseen=None, seconds=3):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.tick()
            b.check(time.monotonic()-self.fixture.host_time<.5,'No fresh fixture data')
            stable=self.fixture.stable(time.monotonic(),seconds)
            if stable and (not moved or self.fixture.moving):
                if face is not None:
                    b.check(stable['face']==face, f'Unexpected face {stable["face"]}, expected {face}')
                if unseen is not None:
                    b.check(stable['face'] not in unseen,'Already captured this face')
                self.journal.add(event='stable',value=stable,delta=self.fixture.delta,path=self.fixture.path)
                return stable
        raise ValueError('Fixture did not reach requested stable pose in time')

    def cue(self, name, text):
        self.journal.add(event='cue',name=name,text=text)
        b.atomic_save(self.output/'display.json',dict(utc=b.utc(),message=text))
        print(text,flush=True)
        if self.audio_dir:
            import winsound
            path=self.audio_dir/(name+'.wav')
            b.check(path.exists(),'Missing audio prompt')
            winsound.PlaySound(str(path),winsound.SND_FILENAME|winsound.SND_ASYNC)

    def capture(self, kind):
        self.cue('still','请保持静止，正在采集。听到下一条动作提示后再动。')
        self.action(kind+'-pose',hold_still=True)
        return self.wait_stable(timeout=5)

    def run(self, at_reference=False):
        # The preceding interactive attempt ended +90 degrees from the physical
        # reference. Restore it without changing the archive reference ID.
        stable=self.wait_stable(face='+X')
        self.fixture.reset_motion(stable)
        if not at_reference:
            self.cue('restore','开始准备：保持同一面朝上，从上往下看，逆时针转九十度，回到最初朝向，放稳。')
            stable=self.wait_stable(moved=True,face='+X')
            self.fixture.check_turn(90,'X')
            self.fixture.reset_motion(stable)
        self.action('earth-start')
        stable=self.capture('earth')
        for index,angle in enumerate((-90,-90,90,90)):
            self.fixture.reset_motion(stable)
            clockwise=angle<0
            self.cue('cw' if clockwise else 'ccw', '顺时针转九十度后放稳。' if clockwise else '逆时针转九十度后放稳。')
            stable=self.wait_stable(moved=True,face='+X')
            self.fixture.check_turn(angle,'X')
            stable=self.capture('earth')
        # Separate series: each GSens save has its own confirmed write. No
        # unconfirmed shared combined-start record is assumed durable.
        self.action('gsens-start')
        stable=self.capture('gsens')
        seen={stable['face']}
        for index in range(5):
            self.fixture.reset_motion(stable)
            self.cue('flip'+str(index+1),'请换到第'+str(index+2)+'个不同的面朝上，慢慢翻转，放稳。')
            opposite=('-' if stable['face'][0]=='+' else '+')+stable['face'][1]
            stable=self.wait_stable(moved=True,unseen=seen,face=opposite if index in (0,2,4) else None)
            b.check(45<length(self.fixture.delta)<220 and self.fixture.path<240,'Unexpected flip trajectory')
            stable=self.capture('gsens')
            seen.add(stable['face'])
        result=self.action('status')
        b.check(all(r['status']['scale_mask']==7 and r['status']['earth_count']==5 and r['status']['gsens_count']==6 for r in result.values()), 'Final archive counts incomplete')
        b.save_new(self.output/'complete.json',dict(utc=b.utc(),devices=result,production_coefficients_changed=False))
        self.cue('complete','整组动作采集完成，可以放下夹具。参数还需要后续计算和验证。')

    def close(self, failed=False):
        errors=[]
        if failed and self.radio and self.console:
            self.armed=False
            try:
                self.action('cancel',timeout=12,cleanup=True)
            except BaseException as e:
                errors.append('cancel: '+str(e))
        if self.owned and self.console:
            try:
                stop=self.command('collectstop',9,cleanup=True)
                inactive=self.command('collect',cleanup=True)
                b.check('unconfirmed=' not in stop and 'Batch data collection STOPPED' in stop and 'Data collection inactive' in inactive,'Summary stop unconfirmed')
                self.owned=False
                b.atomic_save(self.marker,dict(active=False,stopped_at=b.utc(),output=str(self.output)))
            except BaseException as e:
                errors.append('summary cleanup: '+str(e))
        for device in (self.summary,self.console,self.radio):
            if device:
                try:
                    device.close()
                except BaseException as e:
                    errors.append(str(e))
        if errors:
            self.journal.add(event='cleanup-failed',errors=errors)
            raise ValueError('; '.join(errors))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',type=Path,required=True)
    p.add_argument('--slot',type=int,required=True)
    p.add_argument('--audio',type=Path)
    p.add_argument('--start-delay',type=float,default=0)
    p.add_argument('--at-reference',action='store_true',help='Operator has explicitly confirmed original physical reference yaw')
    p.add_argument('--quiet',action='store_true',help='No audio; operator follows the agreed 30s hold/10s move schedule')
    p.add_argument('--await-go',action='store_true',help='Wait idle for a fresh GO_CONTINUOUS file after preflight')
    p.add_argument('mode',choices=('preflight','probe','run'))
    args=p.parse_args()
    with b.lock(args.session/'LOCK'):
        c=Controller(args.session,args.slot,None if args.quiet else args.audio)
        okay=False
        try:
            c.open()
            stable=c.start_summary()
            print(json.dumps(dict(preflight=stable,output=str(c.output)),ensure_ascii=False),flush=True)
            if args.mode=='probe':
                c.action('earth-start')  # no pose/save at the currently rotated yaw
                c.pump(3)
                c.action('cancel')
                c.action('status')
            elif args.mode=='run':
                b.check(args.audio is not None or args.quiet,'Continuous run requires audio or explicit quiet mode')
                b.check(not args.quiet or args.at_reference,'Quiet entry requires confirmed reference pose')
                c.cue('ready','设备已准备好，请保持不动，稍后语音会提示开始。')
                if args.await_go:
                    start_time=time.time()
                    go=args.session/'GO_CONTINUOUS'
                    while not (go.exists() and go.stat().st_mtime>=start_time):
                        c.tick()
                        b.check(time.time()-start_time<180,'Start signal timeout')
                    go.rename(c.output/'GO_CONSUMED')
                c.pump(args.start_delay)
                c.run(at_reference=args.at_reference)
            okay=True
        except BaseException as e:
            c.journal.add(event='fatal',error=str(e))
            if args.audio and not args.quiet:
                c.cue('error','采集已停止，请放稳夹具，等待检查。不要继续动作。')
            raise
        finally:
            c.close(failed=not okay)


if __name__=='__main__':
    main()

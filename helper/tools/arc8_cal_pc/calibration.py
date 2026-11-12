#!/usr/bin/env python3
"""ARC8-H2 computer-side native ARC7 calibration solver, with NO transport/write path.

Reuse ARC7's exact cal_archive_core.c. Input is an explicitly exported, assembled
archive; never fabricate missing captures or treat an OTA/image flash as calibration.
"""
from __future__ import annotations
import argparse
import ctypes as C
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parent
VENDOR = ROOT / 'vendor'
CORE_SHA256 = 'a1b3a3fe07f72704d35e602fe612c8e1dbdc8045ca3890beeb2f9db4b4fd6641'
HEADER_SHA256 = '1cf479961d6ff54017b51e4a59222f28d1ec052faf4dfa76bbabf461b3ec7963'
BOARD = 'styria_mini_uf2/nrf52840/spi'
FORMAT = 'arc8-pc-calibration-v1'
CAPTURE_SCHEMA = 'arc8-archive-assembled-v1'

F=C.c_float
U=C.c_uint32
B=C.c_uint8
H=C.c_uint16

def arr(tp, n): return tp * n
class MotionSeg(C.Structure):
    _fields_=[('raw_delta',arr(F,3)),('accel_dt',arr(F,3)),('temp_dt',F),('seconds',F)]
class Motion(C.Structure):
    _fields_=[('samples',H),('count',B),('merges',B),('lost_cross_deg',F),('seg',arr(MotionSeg,12))]
class Point(C.Structure):
    _fields_=[('temp',F),('gyro',arr(F,3)),('accel',arr(F,3)),('quat',arr(F,4)),('seconds',F),('angle_deg',F),('samples',U),('gyro_sem',arr(F,3))]
class Tcal(C.Structure):
    _fields_=[('magic',U),('version',U),('generation',U),('reference_id',U),('count',B),('reserved',arr(B,3)),('point',arr(Point,12)),('checksum',U)]
class ScaleAxis(C.Structure):
    _fields_=[('initial_raw',arr(F,3)),('initial_accel',arr(F,3)),('initial_temp',F),('raw_angle',arr(F,3)),('accel_time',arr(F,3)),('temp_time',F),('duration',F),('cross_angle',F),('quality',F),('temp_min',F),('temp_max',F),('temp_moments',arr(F,3)),('axis',B),('turns',B),('reserved',arr(B,2))]
class Scale(C.Structure):
    _fields_=[('magic',U),('version',U),('generation',U),('reference_id',U),('count',B),('reserved',arr(B,3)),('axis',arr(ScaleAxis,3)),('checksum',U)]
class Earth(C.Structure):
    _fields_=[('magic',U),('version',U),('generation',U),('reference_id',U),('count',B),('up_axis',B),('reserved',arr(B,2)),('latitude_deg',F),('stop',arr(Point,5)),('travel',arr(Motion,4)),('checksum',U)]
class Gsens(C.Structure):
    _fields_=[('magic',U),('version',U),('generation',U),('reference_id',U),('count',B),('reserved',arr(B,3)),('pose',arr(Point,6)),('travel',arr(Motion,5)),('checksum',U)]
class Solution(C.Structure):
    _fields_=[('scale',arr(F,3)),('earth_body',arr(F,3)),('gsens',arr(F,9)),('base_bias',arr(F,3)),('earth_rms',F),('gsens_rms',F),('scale_error',arr(F,3)),('iterations',B),('ready',B)]

STRUCTS={'tcal':Tcal,'scale':Scale,'earth':Earth,'gsens':Gsens}

def canonical(obj):
    return json.dumps(obj,sort_keys=True,ensure_ascii=False,separators=(',',':'),allow_nan=False).encode('utf8')

def digest(obj): return hashlib.sha256(canonical(obj)).hexdigest()

def fail(what): raise ValueError(what)

def default(tp):
    if issubclass(tp,C.Array): return [default(tp._type_) for _ in range(tp._length_)]
    if issubclass(tp,C.Structure): return {name:default(field) for name,field in tp._fields_}
    return 0

def build(tp,obj,path='archive'):
    if issubclass(tp,C.Array):
        if not isinstance(obj,list) or len(obj)!=tp._length_:
            fail(f'{path}: expected list of exactly {tp._length_} elements')
        return tp(*(build(tp._type_,v,f'{path}[{i}]') for i,v in enumerate(obj)))
    if issubclass(tp,C.Structure):
        if not isinstance(obj,dict): fail(f'{path}: expected object')
        names={name for name,_ in tp._fields_}
        if set(obj)!=names:
            fail(f'{path}: missing {sorted(names-set(obj))}, unexpected {sorted(set(obj)-names)}')
        out=tp()
        for name,field in tp._fields_: setattr(out,name,build(field,obj[name],f'{path}.{name}'))
        return out
    if tp in (F,):
        if isinstance(obj,bool) or not isinstance(obj,(int,float)) or not math.isfinite(obj):
            fail(f'{path}: must be finite float')
        # assignment rounds to float32; overflow is not allowed
        try: v=tp(obj).value
        except (OverflowError,TypeError): fail(f'{path}: not representable as float32')
        if not math.isfinite(v): fail(f'{path}: out of float32 range')
        return v
    if isinstance(obj,bool) or not isinstance(obj,int): fail(f'{path}: integer expected')
    upper={B:255,H:65535,U:4294967295}[tp]
    if obj<0 or obj>upper: fail(f'{path}: integer out of range')
    return obj

def check_target(target):
    if not isinstance(target,dict) or set(target)!={'board','tracker_uid','receiver_key','reference_id'}:
        fail('target: requires board, tracker_uid, receiver_key, reference_id')
    if target['board']!=BOARD: fail('wrong target board')
    for name in ('tracker_uid','receiver_key'):
        val=target[name]
        if not isinstance(val,str) or not (8<=len(val)<=128) or any(ord(c)<33 for c in val):
            fail(f'bad target {name}')
    ref=target['reference_id']
    if isinstance(ref,bool) or not isinstance(ref,int) or not (0<ref<=4294967295):
        fail('invalid reference_id')

def assemble(document):
    if not isinstance(document,dict) or set(document)!={'schema','target','archive','capture_note'}:
        fail('capture document must contain only schema,target,archive,capture_note')
    if document['schema']!=CAPTURE_SCHEMA: fail('wrong archive schema')
    check_target(document['target'])
    if not isinstance(document['capture_note'],str) or not document['capture_note'].strip():
        fail('capture_note/provenance required')
    data=document['archive']
    if not isinstance(data,dict) or set(data)!=set(STRUCTS): fail('four archive datasets required')
    parsed={name:build(tp,data[name],name) for name,tp in STRUCTS.items()}
    ref=document['target']['reference_id']
    for name,part in parsed.items():
        if part.reference_id!=ref: fail(f'{name}: reference_id mismatch')
        if part.magic!=0x43414c31 or part.version!=2: fail(f'{name}: magic/version mismatch')
        if part.generation==0: fail(f'{name}: empty generation')
    if not 1<=parsed['tcal'].count<=12 or parsed['scale'].count!=3 or parsed['earth'].count!=5 or parsed['gsens'].count!=6:
        fail('incomplete calibration collection: TCal≥1, scale 3/3, Earth 5/5, g-sens 6/6 required')
    return parsed

def validate_binary(path, expected_sha):
    if hashlib.sha256(path.read_bytes()).hexdigest()!=expected_sha:
        fail('vendored ARC7 solver source has changed; refuse to run unpinned algorithm')

def compile_solver(directory):
    validate_binary(VENDOR/'cal_archive_core.c',CORE_SHA256)
    validate_binary(VENDOR/'cal_archive_core.h',HEADER_SHA256)
    cc=os.environ.get('CC') or shutil.which('gcc') or shutil.which('clang')
    if not cc: fail('native compiler missing; install GCC or Clang; no calibration generated')
    libname='arc8_cal.dll' if os.name=='nt' else ('libarc8_cal.dylib' if sys.platform=='darwin' else 'libarc8_cal.so')
    output=Path(directory)/libname
    args=[cc,'-std=c11','-O2','-shared', '-I',str(VENDOR),str(VENDOR/'cal_archive_core.c'),str(VENDOR/'abi.c'),'-o',str(output)]
    if os.name!='nt': args.insert(4,'-fPIC')
    if sys.platform!='darwin': args.append('-lm')
    subprocess.run(args,check=True,capture_output=True,text=True)
    lib=C.CDLL(str(output))
    for name,typ in [('tcal',Tcal),('scale',Scale),('earth',Earth),('gsens',Gsens),('solution',Solution)]:
        get=getattr(lib,f'arc8_size_{name}')
        get.restype=C.c_uint
        if get()!=C.sizeof(typ): fail(f'ABI mismatch: {name} (C={get()}, ctypes={C.sizeof(typ)})')
    for name,typ in STRUCTS.items():
        get=getattr(lib,f'arc8_checksum_offset_{name}');get.restype=C.c_uint
        if get()!=getattr(typ,'checksum').offset: fail(f'checksum ABI offset mismatch: {name}')
    lib.cal_arc_solve.argtypes=[C.POINTER(Tcal),C.POINTER(Scale),C.POINTER(Earth),C.POINTER(Gsens),C.POINTER(Solution)]
    lib.cal_arc_solve.restype=C.c_bool
    return lib

def solve(document):
    parsed=assemble(document)
    # On Windows ctypes keeps the native DLL mapped beyond this scope.
    # Suppress only temporary-directory cleanup errors; never suppress compiler/solver errors.
    with tempfile.TemporaryDirectory(prefix='arc8-cal-', ignore_cleanup_errors=True) as tmp:
        solver=compile_solver(tmp)
        output=Solution()
        ready=solver.cal_arc_solve(C.byref(parsed['tcal']),C.byref(parsed['scale']),C.byref(parsed['earth']),C.byref(parsed['gsens']),C.byref(output))
    if not ready or output.ready!=1:
        fail('ARC7 native solver rejected observation quality or identifiability; nothing written')
    values={key:list(getattr(output,key)) for key in ('scale','earth_body','gsens','base_bias','scale_error')}
    values.update(earth_rms=float(output.earth_rms), gsens_rms=float(output.gsens_rms), iterations=int(output.iterations))
    for key,value in values.items():
        for n in value if isinstance(value,list) else [value]:
            if not math.isfinite(n): fail(f'{key}: nonfinite solution rejected')
    if any(x<.9 or x>1.1 for x in values['scale']): fail('scale outside approved 0.9..1.1 range')
    if any(abs(x)>.05 for x in values['gsens']): fail('gsens outside approved ±0.05 dps/g')
    base={
        'schema':FORMAT,'target':document['target'],
        'provenance':{'capture_sha256':digest(document),'solver_c_sha256':CORE_SHA256,
                      'reference_id':document['target']['reference_id'],
                      'capture_note':document['capture_note']},
        'solution':values,
        'deployment':{'state':'OFFLINE_CANDIDATE_UNAPPLIED','transport_enabled':False,
                      'needs_tracker_staged_import':True,'nvs_readback_verified':False,
                      'reboot_verified':False},
        'created_utc':datetime.now(timezone.utc).isoformat(),
    }
    base['payload_sha256']=digest(base)
    return base

def verify_bundle(bundle):
    if not isinstance(bundle,dict) or set(bundle)!={'schema','target','provenance','solution','deployment','created_utc','payload_sha256'}:
        fail('invalid bundle layout')
    if bundle['schema']!=FORMAT: fail('bundle schema mismatch')
    check_target(bundle['target'])
    claimed=bundle['payload_sha256']
    if not isinstance(claimed,str) or len(claimed)!=64 or digest({k:v for k,v in bundle.items() if k!='payload_sha256'})!=claimed:
        fail('bundle SHA256 mismatch')
    state=bundle['deployment']
    if state!={'state':'OFFLINE_CANDIDATE_UNAPPLIED','transport_enabled':False,
              'needs_tracker_staged_import':True,'nvs_readback_verified':False,'reboot_verified':False}:
        fail('unexpected deploy state; cannot assert device activation')
    values=bundle['solution']
    if set(values)!={'scale','earth_body','gsens','base_bias','scale_error','earth_rms','gsens_rms','iterations'}:
        fail('solution fields mismatch')
    for key,quantity in [('scale',3),('earth_body',3),('gsens',9),('base_bias',3),('scale_error',3)]:
        if not isinstance(values[key],list) or len(values[key])!=quantity or any(isinstance(n,bool) or not isinstance(n,(float,int)) or not math.isfinite(n) for n in values[key]):
            fail(f'invalid solution {key}')
    if any(not (.9<=x<=1.1) for x in values['scale']) or any(abs(x)>.05 for x in values['gsens']):
        fail('solution physical safety gates failed')
    return True

def make_template():
    doc={'schema':CAPTURE_SCHEMA,
         'target':{'board':BOARD,'tracker_uid':'UNVERIFIED-UID-PLACEHOLDER',
                   'receiver_key':'UNVERIFIED-RECEIVER','reference_id':1},
         'capture_note':'EXAMPLE ONLY: populate with actual exported records; placeholder is invalid for solving',
         'archive':{key:default(tp) for key,tp in STRUCTS.items()}}
    for name in STRUCTS:
        p=doc['archive'][name]
        p.update(magic=0x43414c31,version=2,generation=1,reference_id=1)
    doc['archive']['tcal']['count']=1
    doc['archive']['scale']['count']=3
    doc['archive']['earth']['count']=5
    doc['archive']['gsens']['count']=6
    return doc

def main(argv=None):
    ap=argparse.ArgumentParser(description='ARC8-H2 offline calibration; cannot write to a tracker')
    sub=ap.add_subparsers(dest='command',required=True)
    sp=sub.add_parser('template',help='write schema template, NOT real observations');sp.add_argument('output')
    sp=sub.add_parser('solve',help='solve actual assembled archive using original C algorithm');sp.add_argument('input');sp.add_argument('output')
    sp=sub.add_parser('verify',help='verify candidate bundle integrity (not device NVS)');sp.add_argument('bundle')
    args=ap.parse_args(argv)
    try:
        if args.command=='template':
            Path(args.output).write_text(json.dumps(make_template(),indent=2,ensure_ascii=False)+'\n',encoding='utf8')
            print('Wrote structural template; contents are NOT valid calibration observations.')
        elif args.command=='solve':
            source=json.loads(Path(args.input).read_text(encoding='utf8'))
            result=solve(source)
            verify_bundle(result)
            destination=Path(args.output)
            if destination.exists(): fail('refusing to overwrite existing calibration bundle')
            destination.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
            print(f'Candidate saved: {destination}; no device write performed')
        elif args.command=='verify':
            verify_bundle(json.loads(Path(args.bundle).read_text(encoding='utf8')))
            print('Candidate JSON hash and structure OK; tracker NVS and boot state NOT checked')
    except (OSError,ValueError,subprocess.CalledProcessError,json.JSONDecodeError) as exc:
        print(f'ARC8 calibration refused: {exc}',file=sys.stderr)
        return 2
    return 0

if __name__=='__main__': raise SystemExit(main())

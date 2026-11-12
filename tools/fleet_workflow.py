"""Durable guided calibration workflow; recordings are never overwritten."""
import json
from pathlib import Path
import sys
import time

import batch_calibration as b

sys.path.insert(0,str(Path(__file__).resolve().parent/'jointcal'))
import continuous
from inputs import digest

STAGES = ('x','y','z','poses')


def read(session):
    path=Path(session)/'wizard.json'
    if path.exists():
        state=json.loads(path.read_text(encoding='utf8'))
        b.check(state.get('schema')=='FLEET_WIZARD1','Unsupported calibration session')
        return state
    return dict(schema='FLEET_WIZARD1',captures={},history=[],artifacts={})


def save(session,state):
    b.atomic_save(Path(session)/'wizard.json',state)


def invalidate(session):
    state=read(session)
    state['artifacts']={}
    save(session,state)
    b.atomic_save(Path(session)/'helper-artifacts.json',{})


def presentation(session):
    state=read(session)
    installation=None
    artifacts=state['artifacts']
    if artifacts.get('installation') and artifacts.get('plan'):
        report=json.loads(Path(artifacts['installation']).read_text(encoding='utf8'))
        if report.get('plan_sha256')==digest(artifacts['plan']):
            installation=report
    # Full attempts and analyses remain on disk; keep GUI IPC bounded.
    return dict(schema=state['schema'],latitude=state.get('latitude'),
                preflight=state.get('preflight'),artifacts=state['artifacts'],installation=installation,
                history_count=len(state['history']),
                captures={k:dict(accepted=v['accepted'],recording=v['recording'])
                          for k,v in state['captures'].items()})


def admit(state,kind):
    b.check(kind in STAGES,'Unknown recording step')
    for previous in STAGES[:STAGES.index(kind)]:
        b.check(previous in state['captures'],f'请先完成 {previous.upper()} 录制。')


def accept_capture(session,kind,recording):
    state=read(session);admit(state,kind)
    entry=dict(kind=kind,recording=str(Path(recording).resolve()),sha256=digest(recording),utc=b.utc(),accepted=False)
    if kind=='poses':entry['analysis_version']=continuous.POSE_ANALYSIS_VERSION
    state['history'].append(entry)
    # Persist the attempt before analysis; old accepted files remain recoverable.
    state['captures'].pop(kind,None)
    state['artifacts']={}
    save(session,state)
    try:
        _,header,analysis=continuous.analyze(recording,kind)
        binding=json.loads((Path(session)/'session.json').read_text(encoding='utf8'))
        b.check(header['receiver_serial']==binding['receiver_serial'] and b.validate_targets(header['targets'])==b.validate_targets(binding['targets']),
                '录制设备与本会话不一致。')
        entry.update(accepted=True,analysis=analysis)
        state['captures'][kind]=entry.copy()
    except Exception as error:
        entry['error']=str(error)
        save(session,state)
        raise
    save(session,state)
    return state


def reanalyze_rejected_pose(session):
    """Revisit the latest old rejection once, retaining the immutable evidence."""
    state=read(session)
    if 'poses' in state['captures'] or not set(STAGES[:3])<=set(state['captures']):
        return False
    latest=next((v for v in reversed(state['history']) if v.get('kind')=='poses'),None)
    if not latest or latest.get('accepted') or latest.get('analysis_version')==continuous.POSE_ANALYSIS_VERSION:
        return False
    source=Path(latest['recording'])
    b.check(source.is_file(),'原十姿态录制文件不存在，无法重新分析。')
    b.check(digest(source)==latest['sha256'],'原十姿态录制文件已改变，不能重新分析。')
    invalidate(session)
    accept_capture(session,'poses',source)
    return True


def prepare(session, snapshots, latitude):
    invalidate(session)
    state=read(session)
    b.check(set(state['captures'])==set(STAGES),'请完成三轴和十姿态四次录制。')
    for entry in state['captures'].values():
        b.check(digest(entry['recording'])==entry['sha256'],'录制文件已改变，不能继续求解。')
    out=Path(session)/('continuous-input-'+str(time.time_ns()))
    path=continuous.prepare({k:v['recording'] for k,v in state['captures'].items()},snapshots,latitude,out)
    state['artifacts']=dict(input=str(path),snapshots=str(snapshots))
    state['latitude']=latitude
    save(session,state)
    return path

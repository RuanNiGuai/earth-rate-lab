"""Compare a joint calibration with its backup and freeze a partial update.

Only scales can currently be selected automatically: the wireless snapshot
does not carry the installed acceleration matrix required to convert raw M.
Earth and g-sens remain byte-identical. No hardware access or trial override.
"""
import copy
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import inputs
import joint_rigid_v3 as joint
from make_plan import require_convergence, require_final_return, assess_quality
from optimize_fusion import prepare, evaluate
from fusion_replay import Replay, POLICIES, ROOT

VERSION='joint-scale-replay-selection-1'


def source_hashes():
    return {name:inputs.digest(ROOT/name) for name in (
        'tools/jointcal/select_parameters.py','tools/jointcal/optimize_fusion.py',
        'tools/jointcal/inputs.py','tools/jointcal/joint_rigid_v3.py',
        'tools/jointcal/joint_fit_v2.py','tools/fusion_replay.py','tools/fusion_replay.c',
        'tools/fusion_stable_iir.py','integration/tracker/vqf-c/src/vqf.c',
        'integration/tracker/vqf-c/src/vqf.h','integration/tracker/src/sensor/fusion/earth_pair.c',
        'integration/tracker/src/sensor/fusion/earth_pair.h','integration/tracker/src/sensor/fusion/vqf/vqf.c')}


def save(path, value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')


def device_metrics(run,slot):
    rows=[r for r in run['devices'] if r['slot']==slot]
    if len(rows)!=4 or {r['kind'] for r in rows}!={'x','y','z','poses'}:
        raise ValueError('Incomplete replay fleet')
    closures=np.concatenate([r['closure_deg'] for r in rows])
    rests=np.concatenate([r['rest_drift_deg_per_min'] for r in rows])
    return dict(closure_rms_deg=float(np.sqrt(np.mean(closures**2))),
                rest_rms_deg_per_min=float(np.sqrt(np.mean(rests**2))))


def compare(config_path,result_path,out,emit=lambda **kw:None):
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    config=inputs.configure(config_path,joint.old);data=inputs.load()
    fit=json.loads(Path(result_path).read_text(encoding='utf8'))
    require_convergence(fit)
    inputs.require(fit['input_config']==config['public_description'],'Result/input mismatch')
    inputs.require(fit['snapshot_hashes']==[d['backup']['snapshot_sha256'] for d in data],'Result/backup mismatch')
    inputs.require(config['dataset_kind']=='measured' and config.get('transport_pass') is True
                   and not config.get('transport_diagnostic_override'),'Complete measured recordings required')
    inputs.require(bool(config.get('scale_captures')),'Continuous bidirectional recordings required')
    inputs.require(all(r['covered'] for r in fit['coverage']),'Temperature support missing')
    if config.get('final_return_present'):require_final_return(fit)
    # The installer stores float32. Evaluate those exact candidate values.
    for row in fit['static']['devices']:
        row['scale']=np.asarray(row['scale'],dtype=np.float32).tolist()
    # No acceleration recalibration is applied to the comparison or installation.
    dummy=dict(devices=[{} for _ in data])
    engines={k:Replay(out/'native',iir=k) for k in ('production','stable')}
    runs={}
    for label,calibration in [('old','old'),('new_scale','new_scale')]:
        emit(stage='replay',detail=label)
        runs[label]=evaluate(engines['production'],prepare(data,fit,dummy,calibration),{},substeps=8)
    # Recheck the exact new-S + old-M combination in the six 20-turn integrals.
    joint.mounting(data,[joint.old.means(d,'train') for d in data])
    scale_fit=fit['iterations'][-1]['scale']
    adjustments=np.vstack([np.zeros(3),scale_fit['mounting_adjustments_deg']])
    directions=np.asarray(scale_fit['shared_turn_vectors_deg'])
    expected=np.stack([directions,-directions],axis=1).reshape(6,3)
    checks=scale_fit['direction_checks']
    inputs.require([r['slot'] for r in checks]==list(range(10)),'Invalid direction checks')
    decisions=[];mixed=copy.deepcopy(fit)
    for slot,(d,new,dc) in enumerate(zip(data,fit['static']['devices'],checks)):
        old=d['backup']['production'];delta=joint.continuous_turns(d)
        q=Rotation.from_rotvec(np.deg2rad(adjustments[slot]))*d['Qseed']
        turn_rms={name:float(np.sqrt(np.mean((q.apply(delta*np.asarray(s))-expected)**2)))
                  for name,s in [('old',old['scale']),('new',new['scale'])]}
        before,after=[device_metrics(runs[name],slot) for name in ('old','new_scale')]
        accepted=bool(scale_fit['success'] and scale_fit['bidirectional'] and dc['passed']
                      and max(dc['relative_difference'])<=.002 and turn_rms['new']<=7.2
                      and turn_rms['new']<turn_rms['old']*.99
                      and after['closure_rms_deg']<=before['closure_rms_deg']+.05
                      and after['rest_rms_deg_per_min']<=before['rest_rms_deg_per_min']*1.1+.005)
        values=copy.deepcopy(old)
        if accepted:values['scale']=np.asarray(new['scale'],dtype=np.float32).tolist()
        mixed['static']['devices'][slot]['scale']=values['scale']
        decisions.append(dict(slot=slot,values=values,scale_updated=accepted,
            turn_rms_deg=turn_rms,replay_before=before,replay_after=after,
            reasons=[('比例：三轴正反转及实际混合参数回放通过，采用新值。' if accepted else
                      '比例：未证明新值在旋转与静置检查中更好，保留备份值。'),
                     '地球项：保留已有向量；新拟合总残差另列诊断，不阻止未改变的字段。',
                     'g-sens：保留已有矩阵；快照缺少加速度校准矩阵，暂不写入原始坐标的新拟合。']))
    selected=prepare(data,mixed,dummy,'new_scale')
    for label,engine,policy in [('selected',engines['production'],{}),
                               ('stable_numeric',engines['stable'],{}),
                               ('adaptive_rest',engines['stable'],POLICIES['adaptive_rest'])]:
        emit(stage='replay',detail=label)
        runs[label]=evaluate(engine,selected,policy,substeps=8)
    report=dict(schema='JOINT_PARAMETER_SELECTION1',version=VERSION,
        input_sha256=inputs.digest(config_path),result_sha256=inputs.digest(result_path),
        snapshot_hashes=fit['snapshot_hashes'],source_sha256=source_hashes(),
        replay_sources={k:e.hashes for k,e in engines.items()},all_ten_used=True,
        decisions=decisions,runs=runs,full_quality=assess_quality(fit,config),
        device_writes=0,firmware_change=False,
        limitations=[
            'F04 积分窗按 400/100 Hz 重复均值；未重建窗内高频运动。',
            '回放未包含已安装的加速度矩阵，不是固件逐位复现。仅更改比例，保留其余补偿。',
            '校准已使用这批数据；回位是人工返回约束，无外部角度真值。',
            '稳定低通、动态零偏只作回放比较，不通过参数写入启用固件算法。',
            '动态策略等待连续静置 30 秒后渐进加快学习；慢转仍可能误判，不作为默认。'])
    save(out/'selection.json',report)
    return report


def load_selection(path,config_path,result_path,data,result):
    report=json.loads(Path(path).read_text(encoding='utf8'))
    inputs.require(report.get('schema')=='JOINT_PARAMETER_SELECTION1' and report.get('version')==VERSION,
                   'Unknown parameter selection version; rerun comparison')
    inputs.require(report['input_sha256']==inputs.digest(config_path)
                   and report['result_sha256']==inputs.digest(result_path)
                   and report['source_sha256']==source_hashes(),'Comparison changed or is stale; rerun comparison')
    inputs.require(report['snapshot_hashes']==[d['backup']['snapshot_sha256'] for d in data],
                   'Comparison/backup mismatch')
    inputs.require([r['slot'] for r in report['decisions']]==list(range(10)),'Incomplete selected fleet')
    for row,d,new in zip(report['decisions'],data,result['static']['devices']):
        old=d['backup']['production'];values=row['values']
        inputs.require(type(row['scale_updated']) is bool,'Invalid selection decision')
        numeric=[*values['scale'],*values['earth_dps'],*values['gsens_dps_per_g'],
                 *row['turn_rms_deg'].values(),*row['replay_before'].values(),*row['replay_after'].values()]
        inputs.require(np.isfinite(numeric).all() and all(v>=0 for v in (
            *row['turn_rms_deg'].values(),*row['replay_before'].values(),*row['replay_after'].values())),
            'Invalid comparison metrics')
        inputs.require(values['earth_dps']==old['earth_dps'] and values['gsens_dps_per_g']==old['gsens_dps_per_g'],
                       'Partial update must preserve Earth and g-sens exactly')
        target=np.asarray(new['scale'],dtype=np.float32).tolist() if row['scale_updated'] else old['scale']
        inputs.require(values['scale']==target,'Selected scale differs from joint result or backup')
        if row['scale_updated']:
            scale=result['iterations'][-1]['scale'];direction=scale['direction_checks'][row['slot']]
            inputs.require(scale['success'] and scale['bidirectional'] and direction['passed']
                           and direction['slot']==row['slot'] and max(direction['relative_difference'])<=.002,
                           'Selected scale failed bidirectional gates')
            t=row['turn_rms_deg'];before=row['replay_before'];after=row['replay_after']
            inputs.require(t['new']<=7.2 and t['new']<t['old']*.99
                           and after['closure_rms_deg']<=before['closure_rms_deg']+.05
                           and after['rest_rms_deg_per_min']<=before['rest_rms_deg_per_min']*1.1+.005,
                           'Selected scale failed comparison gates')
    return report


def presentation(report):
    return dict(version=report['version'],decisions=report['decisions'],limitations=report['limitations'],
                changed_devices=sum(r['scale_updated'] for r in report['decisions']),
                metrics={k:v['summary'] for k,v in report['runs'].items()},firmware_change=False)

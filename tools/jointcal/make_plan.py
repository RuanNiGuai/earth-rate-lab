"""Prepare an explicitly reviewed ten-device plan; never opens hardware."""
import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
from inputs import configure, digest, require, load
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calwire_apply import prepare
from refinement import LIMITS


def require_convergence(result):
    c=result.get('convergence',{})
    require(result.get('solver_version')=='adaptive-window-rigid-1'
            and result.get('numerics_version')=='sensor-residual-window20ms-2'
            and c.get('converged') is True,
            'Solver did not establish convergence; rerun the current solver. Diagnostic results cannot be installed.')
    history=c.get('history',[])
    require(len(history)>=4 and len(history)==c.get('iterations'),'Incomplete convergence history')
    require(all(row.get('iteration')==i+1 and np.isfinite(row.get('objective',np.nan))
                for i,row in enumerate(history)), 'Invalid convergence objectives or iteration order')
    for row in history[-3:]:
        delta=row.get('changes') or {}
        require(row.get('inner_solvers_converged') is True and all(
            k in delta and np.isfinite(delta[k]) and 0<=delta[k]<=limit for k,limit in LIMITS.items()),
            'Final solver changes exceed convergence limits')


def information_failures(result):
    info=result['static'].get('earth_information',{})
    ratio=result['static'].get('horizontal_free_magnitude_ratio')
    phase=info.get('conditional_phase_std_deg')
    failures=[]
    if info.get('projected_rank')!=2 or ratio is None or not np.isfinite(ratio) or not .5<=ratio<=1.5:
        failures.append('Earth signal is not identifiable or free horizontal amplitude differs from physics by more than 50 percent')
    if phase is None or not np.isfinite(phase) or phase>30:
        failures.append('Conditional Earth phase uncertainty exceeds 30 degrees')
    for row in result['static']['devices']:
        condition=row.get('design_condition')
        if row.get('design_rank')!=5 or condition is None or not np.isfinite(condition) or condition>=1e6:
            failures.append(f"slot {row['slot']} calibration design is rank deficient or ill conditioned")
    return failures


def require_final_return(result):
    returned=result.get('final_return',{})
    rows=returned.get('devices',[])
    require(returned.get('present') is True and [r.get('slot') for r in rows]==list(range(10)),
            'Missing or invalid final-return diagnostic')
    closure=returned.get('orientation_closure_deg',np.nan)
    require(np.isfinite(closure) and 0<=closure<=3
            and all(np.isfinite(r.get('gravity_return_deg',np.nan)) and 0<=r['gravity_return_deg']<=3 for r in rows),
            'Final pose does not return to the initial face and heading within 3 degrees; inspect the return before installation')


def assess_quality(result, config):
    """Return the same installation gates as a reviewable, non-installable report."""
    failures=information_failures(result)
    checks=[]
    def check(name, value, limit, unit, slot=None, valid=True):
        finite=bool(np.isfinite(value))
        passed=bool(valid and finite and value<=limit)
        label=(f'#{slot} ' if slot is not None else '')+name
        actual=f'{value:.9g}' if finite else '无有效数值'
        message=f'{label}：实测 {actual} {unit}，门槛 ≤ {limit:g} {unit}'
        if not valid:message+='；拟合状态未通过'
        checks.append(dict(name=name,slot=slot,value=float(value) if finite else None,
                           limit=limit,unit=unit,passed=passed,message=message))
        if not passed:failures.append(message)
    if config.get('scale_captures'):
        scale=result['iterations'][-1]['scale']
        require(scale.get('bidirectional') is True and len(scale['direction_checks'])==10,
                'Missing bidirectional scale validation')
        for row in scale['direction_checks']:
            check('正反转比例差异',max(row['relative_difference'])*100,.2,'%',row['slot'],row['passed'])
        check('三轴联合角度 RMS',scale['joint_angle_rms_deg'],7.2,'°',valid=scale['success'])
        check('地球航向回环',result['earth_loop_closure_deg'],3,'°')
    for row in result['static']['devices']:
        for name,label,limit in [('earth','地球项',.001),('gsens','g-sens',.006)]:
            for part,part_label in [('train','训练'),('holdout','留出')]:
                check(f'{label}{part_label} RMS',row[f'{name}_{part}_rms_dps'],limit,'°/s',row['slot'])
    return dict(schema='JOINT_PLAN_REVIEW1',quality_gates_passed=not failures,
                unmet_quality_gates=failures,checks=checks)


def save_review(out, report):
    out.mkdir(parents=True,exist_ok=False)
    (out/'review.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    lines=['# 校准写入审阅','',
           '质量检查通过，可继续核对写入计划。' if report['quality_gates_passed'] else
           '质量检查未通过，未生成写入计划。录制和求解结果已保留。', '',
           '| 检查项 | 实测值 | 门槛 | 结果 |','|---|---:|---:|---|']
    for row in report['checks']:
        slot=f"#{row['slot']} " if row['slot'] is not None else ''
        value=f"{row['value']:.9g}" if row['value'] is not None else '无有效数值'
        status='诊断；未采用此候选字段' if row.get('blocking') is False else ('通过' if row['passed'] else '未通过')
        lines.append(f"| {slot}{row['name']} | {value} {row['unit']} | ≤ {row['limit']:g} {row['unit']} | {status} |")
    if report['unmet_quality_gates']:
        lines+=['','未通过的项目：','']+['- '+x for x in report['unmet_quality_gates']]
    if report.get('scope'):
        lines+=['','本计划只更新经比较验证的比例；地球项和 g-sens 保留备份值。',
                '下列是未采用的新参数诊断，不是已选字段的写入失败：','']
        lines+=['- '+x for x in report.get('retained_field_diagnostics',[])]
        for row in report.get('decisions',[]):
            lines+=['',f"设备 #{row['slot']}："]+['- '+x for x in row['reasons']]
    lines+=['','本报告不是写入计划，不会访问或修改追踪器。']
    (out/'review.md').write_text('\n'.join(lines)+'\n',encoding='utf8')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('config','result','session','out'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--reference-confirmed',action='store_true')
    p.add_argument('--authorization',required=True)
    p.add_argument('--trial',action='store_true',help='Explicitly accept disclosed failed gates for trial')
    p.add_argument('--review-on-failure',action='store_true',help='Save a rejected review without creating an installation plan')
    p.add_argument('--selection',type=Path,help='Bound replay comparison; update validated scales and retain Earth/g-sens')
    a=p.parse_args()
    config=configure(a.config,SimpleNamespace());data=load()
    require(config['dataset_kind']=='measured','Synthetic data cannot be installed')
    require(config.get('transport_pass') is True and not config.get('transport_diagnostic_override'),
            'Only a complete, audited recording can produce a new installation plan')
    require(a.reference_confirmed and a.authorization.strip(),'Reference pose and explicit authorization required')
    result=json.loads(a.result.read_text(encoding='utf8'))
    require_convergence(result)
    require(result.get('input_config')==config['public_description'],'Result/config mismatch')
    hashes=[d['backup']['snapshot_sha256'] for d in data]
    require(result['snapshot_hashes']==hashes,'Result/snapshot mismatch')
    state=json.loads((a.session/'session.json').read_text(encoding='utf8'))
    require(set(state['targets'])==set(map(str,range(10))),'Entire ten-tracker session required')
    rows=result['static']['devices']
    require([r['slot'] for r in rows]==list(range(10)),'Invalid result fleet')
    if config.get('final_return_present'):
        require_final_return(result)
    require(all(x['covered'] for x in result['coverage']),'Required temperature support missing')
    review=assess_quality(result,config)
    selection=None
    if a.selection:
        require(not a.trial,'Selected-field plans do not use trial overrides')
        from select_parameters import load_selection
        selection=load_selection(a.selection,a.config,a.result,data,result)
        original=review
        selected={r['slot'] for r in selection['decisions'] if r['scale_updated']}
        checks=[]
        for row in original['checks']:
            check=dict(row)
            check['blocking']=bool(row['name']=='正反转比例差异' and row['slot'] in selected)
            checks.append(check)
        for decision in selection['decisions']:
            if decision['scale_updated']:
                value=decision['turn_rms_deg']['new']
                checks.append(dict(name='保留原 g-sens 的三轴角度 RMS',slot=decision['slot'],value=value,
                    limit=7.2,unit='°',passed=value<=7.2,blocking=True,
                    message=f"#{decision['slot']} 实际组合三轴角度 RMS：{value:.6g}°，门槛 ≤ 7.2°"))
        failures=[r['message'] for r in checks if r['blocking'] and not r['passed']]
        if not selected:failures.append('比较未发现需要更新的比例；现有参数保持不变。')
        review=dict(schema='JOINT_PLAN_REVIEW2',scope='validated_scales_keep_earth_gsens',
                    quality_gates_passed=not failures,unmet_quality_gates=failures,checks=checks,
                    full_candidate_quality_gates_passed=original['quality_gates_passed'],
                    retained_field_diagnostics=original['unmet_quality_gates'],
                    selection_path=str(a.selection.resolve()),selection_sha256=digest(a.selection),
                    decisions=selection['decisions'],limitations=selection['limitations'])
    if selection is None:
        review['quality_gates_passed']=False
        review['unmet_quality_gates'].append('原始 M/E/B 坐标尚不支持安全写入；请使用包含已安装加速度矩阵的快照，比较并选择仅更新比例。')
    review.update(result_sha256=digest(a.result),input_config_sha256=digest(a.config),device_writes=0)
    save_review(a.out,review)
    unmet=review['unmet_quality_gates']
    # A trial cannot override a coordinate mismatch. Full raw M/E/B candidates
    # need an affine conversion and a versioned constant-bias transaction first.
    if unmet and not a.trial:
        if a.review_on_failure:
            print(json.dumps(dict(review=str(a.out/'review.json'),quality_gates_passed=False,device_writes=0)))
            return
        raise ValueError('质量检查未通过，未生成写入计划。\n'+'\n'.join(unmet)+f'\n审阅报告：{a.out / "review.md"}')
    require(selection is not None, 'Raw M/E/B coordinates cannot be installed, including trials')
    evidence=dict(mode='USER_AUTHORIZED_TRIAL' if unmet else 'QUALITY_ACCEPTED',quality_gates_passed=not unmet,
        unmet_quality_gates=unmet,user_authorization_quote=a.authorization,
        reference_pose_confirmed='original +X-up face and original heading',
        result_sha256=digest(a.result),input_config_sha256=digest(a.config),decisions={})
    builds={d['backup']['build'] for d in data};require(len(builds)==1,'Mixed firmware builds')
    plan=dict(schema='CALWIRE_APPLY2',receiver_serial=state['receiver_serial'],build=builds.pop(),devices=[])
    for row,spec,b in zip(rows,config['devices'],data):
        # CALWIRE export metadata binds this physical target to its raw snapshot.
        meta=json.loads(spec['snapshot'].with_name('backup.json').read_text(encoding='utf8'))
        slot=row['slot'];address=state['targets'][str(slot)]
        require(meta['slot']==slot and meta['rf_address']==address and meta['snapshot_sha256']==spec['snapshot_sha256'],
                'Physical target/snapshot metadata mismatch')
        values=dict(scale=np.asarray(row['scale'],dtype=np.float32).tolist(),
                    gsens_dps_per_g=np.asarray(row['gsens_dps_per_g'],dtype=np.float32).ravel().tolist(),
                    earth_dps=np.asarray(row['earth_dps'],dtype=np.float32).tolist())
        if selection:values=selection['decisions'][slot]['values']
        plan['devices'].append(dict(slot=slot,rf_address=address,snapshot_path=str(spec['snapshot']),
                                   snapshot_sha256=spec['snapshot_sha256'],values=values))
        evidence['decisions'][str(slot)]=dict(install_authorized=True,reason=a.authorization,
                                             snapshot_sha256=spec['snapshot_sha256'],values=values)
        if selection:
            actions=dict(scale='update' if selection['decisions'][slot]['scale_updated'] else 'keep',
                         gsens_dps_per_g='keep',earth_dps='keep')
            evidence['decisions'][str(slot)]['field_actions']=actions
            plan['devices'][-1]['field_actions']=actions
    if selection:
        evidence.update(scope=review['scope'],selection_path=review['selection_path'],
                        selection_sha256=review['selection_sha256'],
                        retained_field_diagnostics=review['retained_field_diagnostics'])
    ev=a.out.resolve()/'evidence.json';ev.write_text(json.dumps(evidence,indent=2),encoding='utf8')
    plan['evidence_path']=str(ev);plan['evidence_sha256']=digest(ev)
    prepare(plan,state)  # Same physical bounds and exact snapshot validation as the installer.
    (a.out/'plan.json').write_text(json.dumps(plan,indent=2),encoding='utf8')
    print(json.dumps(dict(plan=str(a.out/'plan.json'),unmet_quality_gates=unmet,device_writes=0)))


if __name__=='__main__':
    main()

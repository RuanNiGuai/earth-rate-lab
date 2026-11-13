"""Optional Helper bridge. Structured requests; no shell commands or implicit installs.

The GUI owns receiver/serial reservations until this process has exited. Each
request freezes receiver and RF identities; every device action rechecks them.
Offline actions also work without hardware. Outputs remain in the selected
session; a failed action never advances the accepted artifact pointers.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import runpy
import sys
import time

os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
import batch_calibration as b

ROOT = Path(sys._MEIPASS)/'toolkit' if getattr(sys,'frozen',False) else Path(__file__).resolve().parents[1]
for directory in (ROOT/'tools', ROOT/'tools/jointcal'):
    sys.path.insert(0,str(directory))
UTILITIES = {'utility-record', *('utility-temperature-'+k for k in ('start','stop','status','coverage'))}
HARDWARE = {*UTILITIES, 'init', 'record', 'snapshot', 'apply', 'verify', 'rollback', 'workflow-solve',
            *('workflow-record-'+k for k in ('x','y','z','poses')), *b.FLAGS}
ACTIONS = HARDWARE | {'prepare', 'solve', 'plan', 'review', 'audit', 'workflow-open', 'compare'}


def read_json(path):
    p = Path(path)
    b.check(p.stat().st_size <= 8*1024*1024, 'Oversized configuration')
    return json.loads(p.read_text(encoding='utf-8-sig'))


def validate_request(r):
    b.check(r.get('schema') == 'HELPER_FLEET1' and r.get('action') in ACTIONS, 'Unsupported Helper request')
    b.check(isinstance(r.get('session'), str) and r['session'], 'Choose a session directory')
    if r['action'] in HARDWARE:
        targets = b.validate_targets(r['targets'])
        b.check(set(targets) == set(range(10)), 'Joint fleet requires exactly slots 0..9')
        b.check(isinstance(r.get('receiver_serial'), str) and r['receiver_serial'], 'Missing receiver identity')
    if r['action'] in ('plan', 'apply', 'rollback', 'reference-new'):
        b.check(r.get('confirmed') is True, 'Explicit review confirmation required')
    if r['action'] in ('prepare', 'workflow-solve'):
        b.check(math.isfinite(r['latitude']) and abs(r['latitude']) < 89, 'Latitude must be between -89 and 89 degrees')


def validate_binding(request, state):
    b.check(request['receiver_serial'] == state['receiver_serial'], 'Selected receiver differs from saved session')
    b.check(b.validate_targets(request['targets']) == b.validate_targets(state['targets']),
            'Selected RF roster differs from saved session')


def review_plan(path, state):
    from calwire_apply import prepare
    plan = read_json(path)
    prepare(plan, state)
    evidence = read_json(plan['evidence_path'])
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                quality_gates_passed=evidence['quality_gates_passed'],
                unmet_quality_gates=evidence['unmet_quality_gates'], devices=plan['devices'],
                scope=evidence.get('scope','all_fields'),
                retained_field_diagnostics=evidence.get('retained_field_diagnostics',[]))


def script(relative, args):
    old_argv, old_path = sys.argv, list(sys.path)
    path = ROOT / relative
    try:
        sys.argv = [str(path), *map(str, args)]
        sys.path.insert(0, str(path.parent))
        runpy.run_path(str(path), run_name='__main__')
    finally:
        sys.argv, sys.path = old_argv, old_path


def snapshot(request, session, output, cancel, emit):
    from calwire import Radio, unpack_snapshot
    build = request.get('build', '').strip()
    guided=request.get('guided') is True
    b.check((build or guided) and len(build) <= 128, 'Enter the expected tracker firmware build from OTA info')
    state = read_json(session/'session.json')
    targets = b.validate_targets(state['targets'])
    output.mkdir()
    radio = Radio(state['receiver_serial'], b.Journal(output/'wire.jsonl'), cancel_file=cancel)
    try:
        for slot, address in sorted(targets.items()):
            emit(stage='snapshot', slot=slot, completed=slot, total=10)
            dest = output/f'slot-{slot}'
            dest.mkdir()
            with radio.session(slot, address) as (token, size, crc):
                blob = radio.snapshot(slot, address, token, size, crc)
            (dest/'snapshot.bin').write_bytes(blob)
            try:
                data = unpack_snapshot(blob, require_scales=not guided)
            except ValueError as error:
                raise ValueError(f'#{slot} 备份数据尚不完整：{error}。请先在命令页建立 +X 地球参考并采集温度表，再准备会话。') from error
            if guided:
                b.check(data['earth_valid'] and data['earth_up_axis']==0,
                        f'#{slot} 尚无有效 +X 地球参考。请将 +X 面朝上，在命令页建立参考后重新准备。')
            if guided and not build:
                b.check(data['build'].endswith(('_cw5_f04ack','_cw6_stable')),'该固件尚未验证连续校准协议，请更新配套版本。')
                build=data['build']
            b.check(data['build'] == build, 'Tracker firmware build mismatch')
            data.update(receiver_serial=state['receiver_serial'],slot=slot, rf_address=address, snapshot_sha256=hashlib.sha256(blob).hexdigest())
            b.save_new(dest/'backup.json', data)
            for i, raw in enumerate(data['scale_records_hex']):
                (dest/f'scale-{i}.bin').write_bytes(bytes.fromhex(raw))
    finally:
        radio.close()


def execute(request, job):
    validate_request(request)
    action = request['action']
    if action=='record' or action.startswith('workflow-record-'):
        from capture_preflight import check_capture_conflicts
        check_capture_conflicts()  # Reject before invalidating accepted captures/plans.
    session = Path(request['session']).resolve()
    cancel = job/'CANCEL'
    status = dict(stage='starting', ready=False, action=action)
    def emit(**values):
        status.update(values)
        if not b.atomic_save(job/'progress.json', status, best_effort=True):
            status['progress_updates_deferred']=status.get('progress_updates_deferred',0)+1
    emit()
    b.check(not cancel.exists(), 'Cancelled before starting')
    if action in UTILITIES:
        import helper_utilities
        result=dict(action=action, utility=True, state='COMPLETE',
                    **helper_utilities.execute(request,job,cancel,emit))
    elif action == 'init':
        reused = (session/'session.json').exists()
        if reused:
            state = read_json(session/'session.json')
            validate_binding(request, state)
            # Reuse the durable sequence and state machine; never overwrite
            # existing captures, artifact pointers or an unresolved command.
            b.check(b.run_action(session, 'status', progress=lambda row: emit(**row)),
                    'Session verification incomplete; inspect radio.jsonl')
        else:
            session = session/('fleet-'+str(time.time_ns()))
            config = job/'fleet.json'
            b.save_new(config, dict(receiver_serial=request['receiver_serial'], targets=request['targets']))
            b.initialize(session, config, progress=lambda row: emit(**row))
        b.check(not cancel.exists(), 'Cancelled after read-only verification; session data retained')
        result = dict(session=str(session), output=str(session), action=action,
                      state='SESSION_READY', reused=reused)
        import fleet_workflow as workflow
        if request.get('guided') is True:
            current=workflow.read(session)
            current.pop('preflight',None)
            workflow.save(session,current)
            preflight=session/('preflight-'+str(time.time_ns()))
            try:
                snapshot(request,session,preflight,cancel,emit)
                b.check(not cancel.exists(),'Cancelled after preflight; backups retained')
                current['preflight']=dict(snapshots=str(preflight),verified_at=b.utc())
                workflow.save(session,current)
            except (ValueError, RuntimeError) as error:
                result.update(state='PREFLIGHT_FAILED',failures=[str(error)])
        result['workflow']=workflow.presentation(session)
    elif action.startswith('workflow-'):
        import fleet_workflow as workflow
        state=read_json(session/'session.json')
        b.check(state.get('initialized'),'请先准备并核验十台会话。')
        if action in HARDWARE:
            validate_binding(request,state)
        result=dict(session=str(session),output=str(session),action=action,state='COMPLETE')
        with b.lock(session/'LOCK'):
            if action.startswith('workflow-record-'):
                kind=action.removeprefix('workflow-record-')
                current=workflow.read(session)
                b.check(current.get('preflight'),'请先完成十台会话预检。')
                workflow.admit(current,kind)
                workflow.invalidate(session)
                current=workflow.read(session)
                current['captures'].pop(kind,None)
                workflow.save(session,current)
                from batch_record_only import record
                emit(stage='starting',action='record',capture_kind=kind)
                output=record(session,cancel_file=cancel,progress=lambda row: emit(**row),kind=kind)
                result['output']=str(output)
                emit(stage='analyzing',ready=False)
                try:
                    workflow.accept_capture(session,kind,output/'reports.jsonl')
                except ValueError as error:
                    result.update(state='CAPTURE_REJECTED',failures=[str(error)])
                # Every rerecord invalidates solved/installable pointers.
                b.atomic_save(session/'helper-artifacts.json',{})
            elif action=='workflow-solve':
                current=workflow.read(session)
                b.check(set(current['captures'])==set(workflow.STAGES),'请先完成全部四次录制。')
                workflow.invalidate(session)
                output=session/('workflow-solve-'+str(time.time_ns()));output.mkdir()
                snap=output/'snapshots'
                snapshot(dict(request,guided=True),session,snap,cancel,emit)
                emit(stage='analyzing',ready=False)
                input_path=workflow.prepare(session,snap,request['latitude'])
                b.check(not cancel.exists(),'Cancelled before solving; source recordings retained')
                emit(stage='solving',ready=False)
                solved=output/'solution'
                script('tools/jointcal/joint_rigid_v3.py',['--config',input_path,'--out',solved])
                b.check(not cancel.exists(),'Cancelled after solving; diagnostic results retained')
                fit=read_json(solved/'result.json')
                b.check(fit.get('convergence',{}).get('converged') is True,
                        f"求解达到迭代上限但尚未收敛，已保留诊断报告：{solved/'result.json'}。本次不生成可写入结果。")
                current=workflow.read(session)
                current['artifacts'].update(result=str(solved/'result.json'))
                workflow.save(session,current)
                b.atomic_save(session/'helper-artifacts.json',current['artifacts'])
                result.update(output=str(solved),devices=fit['static']['devices'],final_return=fit.get('final_return',{'present':False}),
                              direction_checks=fit['iterations'][-1]['scale'].get('direction_checks',[]))
            elif action=='workflow-open':
                try:
                    result['recovered_pose_capture']=workflow.reanalyze_rejected_pose(session)
                    current=workflow.read(session)
                    artifacts=current['artifacts']
                    latest=next((row for row in reversed(current['history']) if row.get('kind') in workflow.STAGES),None)
                    if latest and not latest.get('accepted') and latest['kind'] not in current['captures']:
                        result.update(state='CAPTURE_REJECTED',output=str(Path(latest['recording']).parent),
                                      failures=[latest.get('error','最近一次录制检查未通过；原文件已保留。')])
                    if artifacts.get('result'):
                        fit=read_json(artifacts['result'])
                        from inputs import digest
                        b.check(fit['input_config']['config_sha256']==digest(artifacts['input']),
                                '已保存的结果与输入不一致，请重新求解。')
                        result.update(output=str(Path(artifacts['result']).parent),installable=False,
                                      devices=fit['static']['devices'],final_return=fit.get('final_return',{'present':False}),
                                      direction_checks=fit['iterations'][-1]['scale'].get('direction_checks',[]))
                        if artifacts.get('review'):
                            review=read_json(artifacts['review'])
                            b.check(review['result_sha256']==digest(artifacts['result'])
                                    and review['input_config_sha256']==digest(artifacts['input']),
                                    '审阅报告与当前输入／求解结果不一致，请重新生成审阅。')
                            if not review['quality_gates_passed']:
                                result.update(state='PLAN_REJECTED',quality_review=review,
                                              output=str(Path(artifacts['review']).parent),
                                              failures=['审阅未通过，未生成写入计划；现有录制和求解结果已保留。',
                                                        *review['unmet_quality_gates']])
                        if artifacts.get('selection'):
                            from select_parameters import presentation
                            selected=read_json(artifacts['selection'])
                            b.check(selected['input_sha256']==digest(artifacts['input'])
                                    and selected['result_sha256']==digest(artifacts['result']),
                                    '回放比较与输入不一致，请重新比较。')
                            result['comparison']=presentation(selected)
                        if artifacts.get('plan') and result.get('state')!='PLAN_REJECTED':
                            result['reviewed_plan']=review_plan(Path(artifacts['plan']),read_json(session/'session.json'))
                            result['installable']=result['reviewed_plan']['quality_gates_passed']
                            if artifacts.get('review'):result['quality_review']=read_json(artifacts['review'])
                except ValueError as error:
                    result.update(state='CAPTURE_REJECTED',failures=[str(error)])
            else:
                raise ValueError('Unknown workflow action')
        result['workflow']=workflow.presentation(session)
    else:
        state = read_json(session/'session.json')
        if action in HARDWARE:
            validate_binding(request, state)
        index_path = session/'helper-artifacts.json'
        index = read_json(index_path) if index_path.exists() else {}
        output = session/('helper-'+action+'-'+str(time.time_ns()))
        def artifact(name):
            value = request.get(name) or index.get(name)
            b.check(isinstance(value, str) and value, 'Select or produce '+name+' first')
            path = Path(value).resolve()
            b.check(path.exists(), 'Missing '+name+': '+str(path))
            return path
        updates = {}
        reviewed = None
        review_result = {}
        if action in b.FLAGS:
            # Own run_action lock/state machine verifies actual CAL_STATUS, not
            # just receiver admission. Its finite transaction must finish on cancel.
            b.check(b.run_action(session, action), 'Some trackers rejected or did not complete the operation; inspect radio.jsonl')
        else:
            with b.lock(session/'LOCK'):
                # Revoke old write admission before work starts, even if it fails.
                invalid = {'record':('input','result','selection','review','plan'),
                           'snapshot':('input','result','selection','review','plan'),
                           'prepare':('result','selection','review','plan'),
                           'solve':('selection','review','plan'),
                           'compare':('selection','review','plan'), 'plan':('review','plan')}.get(action,())
                if invalid:
                    for key in invalid:index.pop(key,None)
                    b.atomic_save(index_path,index)
                    if (session/'wizard.json').exists():
                        import fleet_workflow as workflow
                        current=workflow.read(session)
                        for key in invalid:current['artifacts'].pop(key,None)
                        workflow.save(session,current)
                if action == 'record':
                    from batch_record_only import record
                    output = record(session, cancel_file=cancel, progress=lambda row: emit(**row))
                    updates = dict(recording=str(output/'reports.jsonl'))
                    index = {k:v for k,v in index.items() if k not in ('input', 'result', 'plan')}
                elif action == 'snapshot':
                    snapshot(request, session, output, cancel, emit)
                    updates = dict(snapshots=str(output))
                    index = {k:v for k,v in index.items() if k not in ('input', 'result', 'plan')}
                elif action == 'audit':
                    from capture_quality import audit_recording
                    quality = audit_recording(artifact('recording'))
                    output.mkdir()
                    b.save_new(output/'quality.json', quality)
                    emit(transport_pass=quality['transport_pass'], failures=quality['failures'])
                    updates = dict(recording=str(artifact('recording')))
                elif action == 'prepare':
                    # Timing and pose boundaries remain explicit measured inputs.
                    # Do not substitute USB arrival times for sensor sample times.
                    script('tools/jointcal/prepare.py', ['--recording', artifact('recording'),
                        '--snapshots', artifact('snapshots'), '--clocks', artifact('clocks'),
                        '--segments', artifact('segments'), '--latitude', request['latitude'], '--out', output])
                    updates = dict(input=str(output/'input.json'), recording=str(artifact('recording')),
                                   snapshots=str(artifact('snapshots')), clocks=str(artifact('clocks')),
                                   segments=str(artifact('segments')))
                    index = {k:v for k,v in index.items() if k not in ('result', 'plan')}
                elif action == 'solve':
                    script('tools/jointcal/joint_rigid_v3.py', ['--config', artifact('input'), '--out', output])
                    updates = dict(result=str(output/'result.json'), input=str(artifact('input')))
                    index.pop('plan', None)
                elif action == 'compare':
                    from select_parameters import compare, presentation
                    selected=compare(artifact('input'),artifact('result'),output,emit)
                    updates=dict(selection=str(output/'selection.json'))
                    index.pop('plan',None);index.pop('review',None)
                    review_result=dict(comparison=presentation(selected),installable=False)
                    if (session/'wizard.json').exists():
                        import fleet_workflow as workflow
                        current=workflow.read(session)
                        current['artifacts'].pop('plan',None);current['artifacts'].pop('review',None)
                        current['artifacts'].update(updates);workflow.save(session,current)
                elif action == 'plan':
                    args=['--config', artifact('input'), '--result', artifact('result'),
                        '--session', session, '--out', output, '--reference-confirmed',
                        '--review-on-failure',
                        '--authorization', 'Operator reviewed the joint result and confirmed the original +X-up pose in Helper']
                    if request.get('selection_mode')=='validated-parts-v1':
                        args+=['--selection',artifact('selection')]
                    script('tools/jointcal/make_plan.py',args)
                    report=read_json(output/'review.json')
                    updates = dict(review=str(output/'review.json'), input=str(artifact('input')),
                                   result=str(artifact('result')))
                    if report['quality_gates_passed']:
                        updates['plan']=str(output/'plan.json')
                        reviewed = review_plan(output/'plan.json', state)
                        review_result=dict(quality_review=report,installable=True)
                    else:
                        index.pop('plan',None)
                        review_result=dict(state='PLAN_REJECTED',installable=False,quality_review=report,
                                           failures=['审阅未通过，未生成写入计划；现有录制和求解结果已保留。',
                                                     *report['unmet_quality_gates']])
                    if (session/'wizard.json').exists():
                        import fleet_workflow as workflow
                        current=workflow.read(session)
                        current['artifacts'].pop('plan',None)
                        current['artifacts'].update(updates)
                        workflow.save(session,current)
                elif action == 'review':
                    reviewed = review_plan(artifact('plan'), state)
                    updates = dict(plan=reviewed['path'])
                elif action in ('apply', 'verify', 'rollback'):
                    path = artifact('plan')
                    plan_bytes = path.read_bytes()
                    b.check(len(plan_bytes) <= 8*1024*1024, 'Oversized installation plan')
                    frozen_plan = job/'frozen-plan.json'
                    with frozen_plan.open('xb') as stream:
                        stream.write(plan_bytes)
                    if action in ('apply', 'rollback'):
                        reviewed = review_plan(frozen_plan, state)
                        b.check(reviewed['sha256'] == request.get('plan_sha256'), 'Plan changed since Helper review')
                        if action == 'apply':
                            b.check(reviewed['quality_gates_passed'] is True, 'This plan has failed quality gates; diagnostic only in Helper')
                    # Apply has its own all-fleet preflight and partial-write
                    # rollback. Never kill it or cancel a rollback midway.
                    emit(stage='transaction', cancellation='finish transaction and readback before releasing devices')
                    script('tools/calwire_apply.py', ['--plan', frozen_plan, '--session', session,
                        '--out', output, '--mode', action])
                    updates['installation']=str(output/'result.json')
                    if (session/'wizard.json').exists():
                        import fleet_workflow as workflow
                        current=workflow.read(session)
                        current['artifacts'].update(updates)
                        workflow.save(session,current)
                else:
                    raise ValueError('Unsupported action')
        index.update(updates)
        b.atomic_save(index_path, index)
        result = dict(session=str(session), action=action, output=str(output if output.exists() else session), artifacts=index,
                      state='COMPLETE', reboot_verified=False)
        result.update(review_result)
        if action in ('apply','verify','rollback'):
            result['installation']=read_json(output/'result.json')
        if reviewed is not None:
            result['reviewed_plan'] = reviewed
        if action == 'record':
            quality = read_json(output/'quality.json')
        if action in ('record', 'audit'):
            result.update(transport_pass=quality['transport_pass'], failures=quality['failures'])
        if action == 'solve':
            fit = read_json(output/'result.json')
            result.update(all_ten_used=fit['all_ten_used'], installable=False,
                          devices=fit['static']['devices'])
        import fleet_workflow as workflow
        if (session/'wizard.json').exists():
            result['workflow']=workflow.presentation(session)
    emit(stage='finished', ready=False, result=result)
    b.save_new(job/'result.json', result)
    if action not in UTILITIES and os.environ.get('EARTH_RATE_STATE_DIR'):
        b.atomic_save(Path(os.environ['EARTH_RATE_STATE_DIR'])/'last-session.json',dict(session=str(session)))
    return result


def main():
    # Frozen Python ignores PYTHONUTF8 on some Windows bootloaders. The GUI
    # reads logs as UTF-8, including traceback messages from failed preflights.
    for output in (sys.stdout, sys.stderr):
        if hasattr(output, 'reconfigure'):
            output.reconfigure(encoding='utf8', line_buffering=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', type=Path, required=True)
    args = parser.parse_args()
    job = args.request.resolve().parent
    try:
        request = read_json(args.request)
        validate_request(request)
        Path(request['session']).mkdir(parents=True,exist_ok=True)
        with b.lock(Path(request['session'])/'HELPER_LOCK'):
            result = execute(request, job)
        print(json.dumps(result, ensure_ascii=False), flush=True)
    except Exception as exc:
        b.atomic_save(job/'progress.json', dict(stage='failed', ready=False, error=str(exc)))
        raise


if __name__ == '__main__':
    main()

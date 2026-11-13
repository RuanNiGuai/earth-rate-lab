"""Offline all-ten calibration ablation and causal VQF parameter comparison.

Selection uses X/Y first turns and the first four pose dwells only. Z, reverse
turns and later pose dwells are reported separately. Parameters from the joint
calibration already used these recordings, so this is conditional validation.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fusion_replay import POLICIES, Replay, metrics, resample_windows
import inputs
from calibration_coordinates import installed_accel
import joint_rigid_v3 as joint


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf8')


def fit_accelerometers(data, fit):
    """Ten coupled sensors, shared gravity per static pose, per-sensor b/gain.

    Six static faces cannot identify arbitrary per-device 3x3 calibration and
    mounting without gauge/priors. Keep the original jointly estimated mounts
    fixed and fit just offset/diagonal gains, with shared unit gravity vectors.
    """
    st = [joint.old.means(d, 'train') for d in data]
    held = [joint.old.means(d, 'hold') for d in data]
    joint.mounting(data, st)
    adjust = np.vstack([np.zeros(3), fit['iterations'][-1]['scale']['mounting_adjustments_deg']])
    qs = [R.from_rotvec(np.deg2rad(v)) * d['Qseed'] for d, v in zip(data, adjust)]
    av = np.array([s[1][joint.IDS] for s in st])
    seed = np.mean([q.apply(v) for q, v in zip(qs, av)], axis=0)
    seed /= np.linalg.norm(seed, axis=1)[:, None]
    def residual(x):
        bias = x[:30].reshape(10, 3)
        gain = np.exp(x[30:60].reshape(10, 3))
        gravity = x[60:].reshape(6, 3)
        gravity = gravity / np.linalg.norm(gravity, axis=1)[:, None]
        # Normalize common gravity; small scale regularization prevents a free
        # radial parameter for each shared vector, without pinning directions.
        return np.r_[np.concatenate([(q.apply((v-b)*s)-gravity).ravel() for q,v,b,s in zip(qs,av,bias,gain)]),
                     (np.linalg.norm(x[60:].reshape(6,3),axis=1)-1)]
    start = np.r_[np.zeros(60), seed.ravel()]
    lo = np.r_[np.full(30,-.03),np.full(30,np.log(.97)),np.full(18,-1.1)]
    hi = np.r_[np.full(30,.03),np.full(30,np.log(1.03)),np.full(18,1.1)]
    result = least_squares(residual, start, bounds=(lo,hi), ftol=1e-12, xtol=1e-12, gtol=1e-12)
    if not result.success:
        raise ValueError('Accelerometer fit did not converge')
    biases, gains = result.x[:30].reshape(10,3), np.exp(result.x[30:60].reshape(10,3))
    rows=[]
    for slot,(s,h,b,g) in enumerate(zip(st,held,biases,gains)):
        def norm_error(v): return float(np.sqrt(np.mean((np.linalg.norm(v,axis=1)-1)**2)))
        rows.append(dict(slot=slot,offset_g=b.tolist(),diagonal_gain=g.tolist(),
                         raw_train_norm_rms_g=norm_error(s[1][joint.IDS]),
                         corrected_train_norm_rms_g=norm_error((s[1][joint.IDS]-b)*g),
                         raw_hold_norm_rms_g=norm_error(h[1]),corrected_hold_norm_rms_g=norm_error((h[1]-b)*g),
                         installed_train_norm_rms_g=(norm_error(installed_accel(s[1][joint.IDS],data[slot]['backup']))
                             if data[slot]['backup'].get('accelerometer',{}).get('known') else None),
                         installed_hold_norm_rms_g=(norm_error(installed_accel(h[1],data[slot]['backup']))
                             if data[slot]['backup'].get('accelerometer',{}).get('known') else None)))
    return dict(all_ten_joint=True,fixed_mounts=True,success=result.success,optimality=float(result.optimality),
                jacobian_rank=int(np.linalg.matrix_rank(result.jac)),parameters=len(result.x),devices=rows,
                write_permitted=False,
                limitation='Installed comparison requires TLV4; legacy snapshots report null installed metrics. Same-data fit is not independent validation, and no affine M/B/accel transaction is supported.')


def prepare(data, fit, accel_fit, calibration):
    recordings=[]
    for slot,(d,new,af) in enumerate(zip(data,fit['static']['devices'],accel_fit['devices'])):
        prior=d['backup']['production']
        S=np.asarray(prior['scale'] if calibration=='old' else new['scale'])
        M=np.asarray(prior['gsens_dps_per_g'] if calibration in ('old','new_scale') else new['gsens_dps_per_g']).reshape(3,3)
        E=np.asarray(new['earth_dps'] if calibration=='new_all' else prior['earth_dps'])
        for name,rec in [('poses',d)]+[(c,d['scale_captures'][k]) for k,c in enumerate('xyz')]:
            accel=installed_accel(rec['accel'],d['backup'],allow_unknown=True)
            if calibration=='new_accel':
                accel=(rec['accel']-af['offset_g'])*af['diagonal_gain']
            # Installed M acts on calibrated acceleration. Newly fitted M
            # acts on raw samples; converting it also changes the constant B.
            ma=installed_accel(rec['accel'],d['backup'],allow_unknown=True) if calibration in ('old','new_scale') else rec['accel']
            gyro=(rec['gyro']-joint.old.table(d,rec['temp'])-ma@M.T)*S
            duration=rec['duration']*rec['clock_rate']
            start=rec['interval_end'][0]-duration[0]
            t,g,a,dt=resample_windows(gyro,accel,duration,start)
            timing_error=float(np.ptp(rec['interval_end']-np.cumsum(duration)))
            if timing_error>.025:
                raise ValueError('Sensor timeline diverges from accumulated intervals')
            # Axis recordings start on different faces with unmeasured compass
            # heading; no invented geographic direction is used for those.
            earth=E if name=='poses' and calibration!='earth_off' else np.zeros(3)
            rests=joint.old.SEG if name=='poses' else rec['rests']
            recordings.append(dict(slot=slot,kind=name,time=t,gyro=g,accel=a,dt=dt,earth=earth,rests=rests,
                                   timing_residual_span_s=timing_error,
                                   installed_accel_known=d['backup'].get('accelerometer',{}).get('known',False)))
    return recordings


def evaluate(engine, recordings, policy, substeps=1):
    rows=[]
    for r in recordings:
        out=engine.run(r['gyro'],r['accel'],r['dt'],r['earth'],substeps=substeps,**policy)
        row=metrics(r['time'],out,r['rests'])
        row.update(slot=r['slot'],kind=r['kind'])
        rows.append(row)
    train_drift=[];train_closure=[];held_drift=[];held_closure=[]
    for r in rows:
        if r['kind']=='poses':
            train_drift+=r['rest_drift_deg_per_min'][:4]
            held_drift+=r['rest_drift_deg_per_min'][4:]
            held_closure+=r['closure_deg']
        elif r['kind'] in ('x','y'):
            train_drift+=r['rest_drift_deg_per_min'][:2]
            train_closure.append(r['closure_deg'][0])
            held_drift+=r['rest_drift_deg_per_min'][2:]
            held_closure+=r['closure_deg'][1:]
        else:
            held_drift+=r['rest_drift_deg_per_min']
            held_closure+=r['closure_deg']
    def stats(x):return dict(rms=float(np.sqrt(np.mean(np.array(x)**2))),median=float(np.median(x)),maximum=float(max(x)))
    summary=dict(train_rest_deg_per_min=stats(train_drift),held_rest_deg_per_min=stats(held_drift),
                 train_closure_deg=stats(train_closure),held_closure_deg=stats(held_closure))
    # Training-only selection, no validation endpoint in this score.
    summary['training_score']=summary['train_closure_deg']['rms']+5*summary['train_rest_deg_per_min']['rms']
    return dict(policy=policy,substeps=substeps,summary=summary,devices=rows,
                installed_accel_known=all(r.get('installed_accel_known',False) for r in recordings))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,required=True);p.add_argument('--result',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--iir',choices=('production','stable'),default='production')
    p.add_argument('--confirm-policy',choices=tuple(POLICIES),
                   help='Compare one fixed policy against baseline at 400/100 update cadence; no parameter search')
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    config=inputs.configure(args.input,joint.old);data=inputs.load()
    fit=json.loads(args.result.read_text(encoding='utf8'))
    if fit['input_config']['config_sha256']!=inputs.digest(args.input):
        raise ValueError('Result/input mismatch')
    accel_fit=fit_accelerometers(data,fit);write(args.out/'accel-candidate.json',accel_fit)
    engine=Replay(args.out/'native',iir=args.iir)
    if args.confirm_policy:
        recordings=prepare(data,fit,accel_fit,'new_keep_earth')
        policies={k:POLICIES[k] for k in ('baseline',args.confirm_policy)}
        report=dict(schema='FLEET_FUSION_CONFIRMATION1',status='OFFLINE_CANDIDATE_NOT_INSTALLED',
                    iir=args.iir,source_sha256=engine.hashes,evaluator_sha256=inputs.digest(__file__),
                    input_sha256=inputs.digest(args.input),result_sha256=inputs.digest(args.result),
                    selection='Fixed policies supplied by caller; no new search or installation approval',
                    limitations=[
                        'F04 window means repeated at 400/100 update cadence; raw high-frequency signals are unavailable.',
                        'Installed acceleration uses TLV4 when available; legacy unknowns are flagged and cannot authorize installation.',
                        'Joint calibration already used these recordings; held segments are conditional checks.',
                        'Manual return constraints have no external angle truth. Earth is off in axis recordings.'],
                    results={k:evaluate(engine,recordings,p,substeps=8) for k,p in policies.items()},
                    device_writes=0)
        write(args.out/'confirmation.json',report)
        return
    results={}
    for calibration in ('old','new_scale','new_keep_earth','new_all','earth_off','new_accel'):
        result=evaluate(engine,prepare(data,fit,accel_fit,calibration),{})
        results[calibration]=result
        write(args.out/('calibration-'+calibration+'.json'),result)
        print('calibration',calibration,json.dumps(result['summary']),flush=True)
    # Hold Earth unchanged unless a separately justified comparison selects it.
    recordings=prepare(data,fit,accel_fit,'new_keep_earth')
    candidates=[{}]
    for sigma in (.005,.008,.012,.025):
        candidates.append(dict(rest_sigma=sigma))
    for forgetting in (100,1500):candidates.append(dict(forgetting=forgetting))
    for mode in (1,2):candidates.append(dict(motion_mode=mode))
    candidates+=[dict(accel_gate=3),dict(tau=2),dict(tau=8)]
    runs=[]
    for index,policy in enumerate(candidates):
        run=results['new_keep_earth'] if not policy else evaluate(engine,recordings,policy)
        runs.append(run);write(args.out/f'policy-{index:02d}.json',run)
        print('policy',index,policy,'training',run['summary']['training_score'],flush=True)
    chosen=min(range(len(runs)),key=lambda i:runs[i]['summary']['training_score'])
    selected=runs[chosen]
    rate_check={}
    for name,policy in [('baseline',{}),('selected',selected['policy'])]:
        rate_check[name]=evaluate(engine,recordings,policy,substeps=8)
        write(args.out/('rate-'+name+'.json'),rate_check[name])
    report=dict(schema='FLEET_FUSION_REPLAY1',status='OFFLINE_CANDIDATE_NOT_INSTALLED',
                iir=args.iir,evaluator_sha256=inputs.digest(__file__),
                input_sha256=inputs.digest(args.input),result_sha256=inputs.digest(args.result),
                source_sha256=engine.hashes,selected_index=chosen,selected_policy=selected['policy'],
                selection='X/Y forward closures + first four pose dwells and first two X/Y rests; minimize closure RMS + 5*rest-rate RMS',
                baseline=results['new_keep_earth']['summary'],selected=selected['summary'],
                calibration={k:v['summary'] for k,v in results.items()},
                rate_sensitivity={k:v['summary'] for k,v in rate_check.items()},
                limitations=[
                    'F04 window means, not the original raw 400/100 Hz stream. Conservative overlap resampling; substeps do not add information.',
                    'Production VQF core, Earth pair and gate functions; no boot-ZRO handoff, temperature low-pass or MCU CMSIS simulation.',
                    'Installed acceleration uses TLV4 when available; legacy snapshots use a labelled raw diagnostic fallback, never an installation baseline.',
                    'Calibration used all four recordings already. Fusion selection has withheld segments, but needs independent recording confirmation.',
                    'Return constraints are user-performed physical returns, not external orientation metrology.',
                    'Earth comparison applies only to poses recording with its matching reference; axes use Earth off for every variant.',
                    'Parameters are global, never selected independently for each device. No calibration or firmware is written.'])
    write(args.out/'comparison.json',report)
    print('SELECTED',chosen,selected['policy'],flush=True)


if __name__=='__main__':main()

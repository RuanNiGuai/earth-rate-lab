"""Independent RAWFIFO-v1 and known-truth scenario checks; never accesses a device."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from fusion_replay import POLICIES, Replay, orientations


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_raw(capture):
    capture=Path(capture);summary_path=capture/'decoded/summary.json';csv_path=capture/'decoded/samples.csv'
    summary=json.loads(summary_path.read_text(encoding='utf8'))
    if summary.get('transport_complete') is not True:raise ValueError('Incomplete transport')
    if summary.get('header_invalid_records') or summary['gyro_valid_records']!=summary['valid_records']:
        raise ValueError('Invalid raw gyro packets require explicit handling')
    source=capture/'serial.txt'
    if not source.is_file() or digest(source)!=summary['source_sha256']:
        raise ValueError('RAWFIFO source hash mismatch')
    n=summary['valid_records']
    if not isinstance(n,int) or n<2:raise ValueError('RAWFIFO recording is empty or too short')
    g=np.empty((n,3));a=np.full((n,3),np.nan);t=np.empty(n);temp=np.empty(n)
    count=0
    with csv_path.open(encoding='utf8',newline='') as f:
        reader=csv.DictReader(f)
        for i,row in enumerate(reader):
            if i>=n or int(row['sequence'])!=summary['first_sequence']+i:raise ValueError('Raw sequence mismatch')
            t[i]=int(row['batch_uptime_us'])/1e6
            temp[i]=float(row['temperature_c_driver'])
            g[i]=[float(row['gyro_'+axis+'_dps']) for axis in 'xyz']
            if row['accel_valid']=='True':a[i]=[float(row['accel_'+axis+'_g']) for axis in 'xyz']
            elif row['accel_valid']!='False':raise ValueError('Invalid acceleration freshness tag')
            count+=1
    if count!=n or np.any(np.diff(t)<0):raise ValueError('Raw length/time mismatch')
    if not all(np.isfinite(v).all() for v in (g,t,temp)):raise ValueError('Nonfinite raw gyro/time/temperature')
    fresh=np.flatnonzero(np.isfinite(a).all(axis=1))
    if len(fresh)<2 or not np.all(np.diff(fresh)==4):raise ValueError('Unexpected fresh accel cadence')
    return dict(gyro=g,accel=a,batch_time=t,temp=temp,summary=summary,
                source_sha256=summary['source_sha256'],csv_sha256=digest(csv_path))


def static_metrics(output,dt):
    # Evaluate once per five seconds after 15-second initialization. Report
    # both total displacement and local increments; end cancellation is not
    # sufficient evidence for low drift.
    ix=np.arange(int(15/dt),len(output),max(1,int(5/dt)))
    q=orientations(output[ix]);angles=np.rad2deg((q*q[0].inv()).magnitude())
    increments=np.rad2deg((q[1:]*q[:-1].inv()).magnitude())/5*60
    return dict(duration_s=len(output)*dt,angle_from_15s_deg=float(angles[-1]),
                maximum_displacement_deg=float(max(angles)),
                five_second_drift_rms_deg_per_min=float(np.sqrt(np.mean(increments**2))),
                rest_fraction=float(np.mean(output[int(15/dt):,7])))


def scenarios(engine,policies):
    dt=1/400;t=np.arange(0,120,dt);out={}
    for name,rate,linear in [('stationary',0,False),('yaw_0.2dps',.2,False),('yaw_0.05dps',.05,False),
                             ('yaw_0.01dps',.01,False),('horizontal_accel',0,True)]:
        g=np.zeros((len(t),3));g[t>=30,2]=rate
        a=np.full_like(g,np.nan);a[::4]=[0,0,1]
        if linear:a[(t>=30)&(t<40)&(np.arange(len(t))%4==0),0]=.5
        out[name]={}
        for label,policy in policies.items():
            result=engine.run(g,a,np.full(len(t),dt),raw_accel_stride=4,**policy)
            rotations=orientations(result)
            delta=rotations[-1]*rotations[int(30/dt)].inv()
            yaw=float(np.rad2deg(delta.as_rotvec()[2]))
            up=rotations.apply(np.broadcast_to([0,0,1],(len(t),3)))
            tilt=np.rad2deg(np.arccos(np.clip(up[:,2],-1,1)))
            out[name][label]=dict(expected_yaw_deg=rate*90,output_yaw_deg=yaw,
                                  yaw_error_deg=abs(yaw-rate*90),maximum_tilt_deg=float(max(tilt)),
                                  final_tilt_deg=float(tilt[-1]),rest_fraction_after_30s=float(np.mean(result[t>=30,7])))
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--raw-session',type=Path)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--iir',choices=('production','stable'),default='production')
    p.add_argument('--policy',action='append',choices=tuple(POLICIES),
                   help='Repeat to compare a fixed subset; default compares all policies')
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    engine=Replay(args.out/'native',iir=args.iir)
    policies={k:POLICIES[k] for k in (args.policy or POLICIES)}
    report=dict(schema='FUSION_CONFIRMATION1',iir=args.iir,policies=policies,source_sha256=engine.hashes,
                validator_sha256=digest(__file__),scenarios=scenarios(engine,policies),raw=[],skipped=[],device_writes=0)
    if args.raw_session:
        session=json.loads(args.raw_session.read_text(encoding='utf8'))
        report['raw_session_sha256']=digest(args.raw_session)
        report['device_serial']=session['device_serial']
        for c in session['captures']:
            if c.get('mechanical_stationarity_confirmed') is not True:continue
            raw=load_raw(args.raw_session.parent/c['path'])
            mask=np.ones(len(raw['gyro']),dtype=bool)
            if 'start_device_s' in c:mask &= raw['batch_time']>=c['start_device_s']
            if 'end_device_s' in c:mask &= raw['batch_time']<=c['end_device_s']
            # Exclude excessive temperature at the start, without deleting
            # interior samples and compressing time.
            if 'maximum_temperature_c' in c:
                valid_temp=raw['temp']<=c['maximum_temperature_c']
                bad=np.flatnonzero(mask & ~valid_temp)
                if len(bad):mask[:bad[-1]+1]=False
            selected=np.flatnonzero(mask)
            if len(selected)<409.6*35:
                report['skipped'].append(dict(capture_id=c['capture_id'],reason='less than 35 s of eligible support'));continue
            if np.any(np.diff(selected)!=1):raise ValueError('Selection creates an interior gap')
            g,a=raw['gyro'][mask],raw['accel'][mask]
            dt=1/409.6
            row=dict(capture_id=c['capture_id'],source=str((args.raw_session.parent/c['path']).resolve()),
                     csv_sha256=raw['csv_sha256'],source_sha256=raw['source_sha256'],samples=len(g),
                     batch_time_range_s=[float(raw['batch_time'][mask][0]),float(raw['batch_time'][mask][-1])],
                     temperature_range_c=[float(raw['temp'][mask].min()),float(raw['temp'][mask].max())],results={})
            for label,policy in policies.items():
                result=engine.run(g,a,np.full(len(g),dt),raw_accel_stride=4,**policy)
                row['results'][label]=static_metrics(result,dt)
            report['raw'].append(row)
            print(c['capture_id'],len(g),{k:round(v['five_second_drift_rms_deg_per_min'],4) for k,v in row['results'].items()},flush=True)
    report['limitations']=[
        'Raw validation uses original gyro packets and only fresh acceleration tags; nominal 409.6/102.4 Hz, since FIFO timestamp codes are not a calibrated per-sample clock.',
        'Single-tracker data validates global filter policies only. It does not recalibrate or replace the ten-device joint solution.',
        'These old recordings contain raw bias/thermal variation. No current ten-device LUT, scales or Earth vector is applied to a different identity.',
        'Each eligible recording starts a fresh filter; inter-recording gaps are never silently concatenated.',
        'Mechanical static metadata supplies the zero-motion assumption, with no external angular metrology.',
        'Guard threshold is an exploratory bias-rate prior; a true bias jump larger than the threshold can be misclassified as motion.',
        'Static drift can always be reduced by mistaking slow motion for bias. Known-motion scenarios must also be examined.']
    (args.out/'confirmation.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf8')


if __name__=='__main__':main()

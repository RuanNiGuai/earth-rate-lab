"""Automatic preparation of four complete, immutable ten-device recordings.

Sensor intervals remain measurements. Host arrivals only initialize clock models;
all-ten motion alignment refines them. No resampled values enter saved inputs.
"""
import json
from pathlib import Path
import sys
import numpy as np
from scipy.ndimage import uniform_filter1d
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capture_quality import audit_recording, failure_message
from f04_summary import EndManifest, decode_frame_report
from inputs import digest, require

KINDS = ('x', 'y', 'z', 'poses')
POSE_ANALYSIS_VERSION = 'observed-face-heading-return-2'


def rest_bias(gyro):
    speed=np.linalg.norm(gyro,axis=1)
    return np.median(gyro[speed<=np.percentile(speed,20)],axis=0)


def read_capture(path):
    path = Path(path)
    quality = audit_recording(path)
    require(quality['transport_pass'], failure_message(quality))
    require(quality['targets'] == list(range(10)), '必须同时录制 0–9 号十台设备。')
    seen = [{} for _ in range(10)]
    with path.open(encoding='utf8') as stream:
        header = json.loads(next(stream))
        for line in stream:
            row = json.loads(line)
            if 'report_hex' not in row:
                continue
            sample, _, _ = decode_frame_report(bytes.fromhex(row['report_hex']))
            if not isinstance(sample, EndManifest):
                seen[sample.tracker].setdefault(sample.sequence, (sample, row['host_monotonic_ns']/1e9))
    data = []
    origin = min(v[0][1] for v in seen)
    for entries in seen:
        ordered = [entries[k] for k in sorted(entries)]
        rows, host = zip(*ordered)
        end = np.array([((s.end_us-rows[0].end_us) & 0xffffffff)/1e6 for s in rows])
        duration = np.array([s.duration_us/1e6 for s in rows])
        require(np.all(np.diff(end)>0) and end[-1] < 3600, '时间戳异常或录制超过一小时，请分步重录。')
        data.append(dict(local_end=end, duration=duration, host=np.asarray(host)-origin,
                         gyro=np.array([s.gyro_delta_udeg for s in rows])/duration[:,None]/1e6,
                         accel=np.array([s.accel_mean_mg for s in rows])/1000,
                         temp=np.array([s.temperature_centi_c for s in rows])/100,
                         seq=np.array([s.sequence for s in rows])))
    return data, header, quality


def align_clocks(data):
    """Lower arrival envelope seed, then leave-one-device-out motion alignment."""
    models = []
    for d in data:
        t = d['local_end']
        require(t[-1] >= 40, '录制太短：请完成全部静置和旋转段后再结束。')
        points = []
        for lo in np.arange(0, t[-1]-2, 2):
            ix = np.flatnonzero((t>=lo)&(t<lo+2))
            j = ix[np.argmin(d['host'][ix]-t[ix])]
            points.append((t[j], d['host'][j]))
        x, y = np.asarray(points).T
        fit = least_squares(lambda p: (p[0]+x*(1+p[1]*1e-6)-y)/.01,
                            [np.median(y-x),0], loss='soft_l1', x_scale=[.1,100])
        offset, ppm = fit.x
        require(abs(ppm)<900, '采样时钟速率异常，请保持接收器稳定并重录。')
        models.append([offset, ppm])
    models = np.asarray(models)
    # Normalize angular speeds to avoid treating per-axis scale/mounting as time.
    curves = []
    for d in data:
        speed = np.linalg.norm(d['gyro']-rest_bias(d['gyro']),axis=1)
        speed = uniform_filter1d(speed, size=11)
        moving_speed=speed[speed>5]
        require(len(moving_speed)>100, '没有检测到足够旋转，请按图示完成动作。')
        # Long dwells can make >95% of the recording stationary. Normalize
        # using actual motion, independently of the stationary/moving ratio.
        curves.append(speed/np.percentile(moving_speed,75))
    for _ in range(3):
        times = [(d['local_end']-d['duration']/2)*(1+p[1]*1e-6)+p[0] for d,p in zip(data,models)]
        grid = np.arange(max(t[0] for t in times)+.4,min(t[-1] for t in times)-.4,.04)
        values = np.array([np.interp(grid,t,v) for t,v in zip(times,curves)])
        corrections = []
        for i in range(10):
            common = np.median(np.delete(values,i,axis=0),axis=0)
            moving = (common>.02)|(np.abs(np.gradient(common))>.002)
            require(np.count_nonzero(moving)>100, '运动不足，无法对齐十台采样时钟。')
            g=grid[moving];target=common[moving]
            def residual(p):
                warped=times[i]+p[0]+(times[i]-grid.mean())*p[1]*1e-6
                return np.r_[(np.interp(g,warped,curves[i])-target),p[1]/10000]
            fit=least_squares(residual,[0.,0.],bounds=([-.25,-150],[.25,150]),x_scale=[.05,50])
            require(abs(fit.x[0])<.245 and abs(fit.x[1])<149, '十台时钟无法可靠对齐，请重录本步骤。')
            require(np.sqrt(np.mean(residual(fit.x)[:-1]**2))<.07,
                    '十台角运动不一致，请检查夹具是否松动或某台方向不同。')
            corrections.append(fit.x)
        corrections=np.asarray(corrections);corrections-=corrections.mean(axis=0)
        models[:,0]+=corrections[:,0]-grid.mean()*corrections[:,1]*1e-6
        models[:,1]+=corrections[:,1]
    for d,(offset,ppm) in zip(data,models):
        rate=1+ppm*1e-6
        require(abs(ppm)<1000,'时钟漂移超过可校准范围。')
        d.update(interval_end=d['local_end']*rate+offset,
                 time=(d['local_end']-d['duration']/2)*rate+offset,clock_rate=rate)
    return [dict(tracker=i,offset_seconds=float(p[0]),clock_rate_correction_ppm=float(p[1]),
                 method='arrival-envelope seed; all-ten motion refinement') for i,p in enumerate(models)]


def static_segments(data, kind):
    grid=np.arange(max(d['time'][0] for d in data),min(d['time'][-1] for d in data),.1)
    stable=np.ones(len(grid),dtype=bool)
    for d in data:
        gyro=d['gyro']-rest_bias(d['gyro'])
        speed=np.linalg.norm(gyro,axis=1)
        accel=d['accel']
        variation=np.sqrt(np.maximum(0,uniform_filter1d(accel**2,51,axis=0)-uniform_filter1d(accel,51,axis=0)**2).sum(axis=1))
        stable &= (np.interp(grid,d['time'],speed)<.6)&(np.interp(grid,d['time'],variation)<.025)
    edges=np.diff(np.r_[False,stable,False].astype(int))
    ranges=[(grid[a]+3,grid[b-1]-3) for a,b in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)) if grid[b-1]-grid[a]>=22]
    expected=(10,11) if kind=='poses' else (3,)
    require(len(ranges) in expected, f'检测到 {len(ranges)} 段足够长的静置，需要 '+('10段，或加上回到原姿态的第11段' if kind=='poses' else '3段')+'；不要在旋转中途长时间停顿。')
    means=np.array([[np.mean(d['accel'][(d['time']>=lo)&(d['time']<=hi)],axis=0) for lo,hi in ranges] for d in data])
    gravity=np.mean(means,axis=0);norm=np.linalg.norm(gravity,axis=1)
    require(np.all((norm>.8)&(norm<1.2)),'静置时加速度异常，请检查固定与振动。')
    unit=gravity/norm[:,None]
    if kind=='poses':
        # Keep every segment in acquisition order. The solver integrates the
        # intervening motion and uses each measured acceleration, not face labels
        # as synthetic orientations. Sorting the samples would break both.
        axes=np.argmax(abs(unit),axis=1)
        faces=[('+' if v[a]>0 else '-')+'XYZ'[a] for v,a in zip(unit,axes)]
        directions=np.eye(3)[axes]*np.sign(unit[np.arange(len(ranges)),axes])[:,None]
        per_device=means/np.linalg.norm(means,axis=2)[:,:,None]
        require(np.all(np.sum(per_device*directions[None],axis=2)>.92),
                '部分设备的静置面不明确或方向不一致，请检查夹具；识别顺序：'+', '.join(faces))
        require(faces[:5]==['+X']*5, '前五个地球姿态须保持 +X 朝上；识别顺序：'+', '.join(faces))
        required={'-X','+Y','-Y','+Z','-Z'}
        missing=sorted(required-set(faces[5:10]))
        require(not missing and len(set(faces[5:10]))==5,
                '六面覆盖不足，缺少 '+', '.join(missing)+'；识别顺序：'+', '.join(faces)+'。不能用重复面代替缺失面。')
        if len(ranges)==11:
            require(faces[-1]=='+X','第11段应回到最初的 +X 面及原始航向；原录制保留。')
        yaw_turns=[]
        for d in data:
            rests=[np.mean(d['gyro'][(d['time']>=a)&(d['time']<=b)],axis=0) for a,b in ranges[:5]]
            angles=[]
            for k in range(4):
                m=(d['time']>ranges[k][1])&(d['time']<ranges[k+1][0])
                delta=np.sum((d['gyro'][m]-(rests[k]+rests[k+1])/2)*d['duration'][m,None],axis=0)
                require(65<abs(delta[0])<115 and np.linalg.norm(delta[1:])<20,
                        '前五个姿态之间应保持 +X 朝上，每次转约 90°。')
                angles.append(float(delta[0]))
            yaw_turns.append(angles)
        headings=np.c_[np.zeros(len(data)),np.cumsum(yaw_turns,axis=1)]
        closure=abs((headings[:,-1]+180)%360-180)
        require(np.all(closure<15), '前五个姿态结束时没有回到起始航向；请完成回环，原数据已保留。')
        # Geometric observability after removing a constant and linear drift.
        # Reversed turns/repeated headings can still supply two independent
        # horizontal Earth components; repeating only two headings cannot.
        centers=np.mean(np.asarray(ranges[:5]),axis=1)
        nuisance=np.c_[np.ones(5),(centers-centers.mean())/np.ptp(centers)]
        information=[]
        for heading in headings:
            h=np.deg2rad(heading);basis=np.c_[np.cos(h),np.sin(h)]
            projected=basis-nuisance@np.linalg.lstsq(nuisance,basis,rcond=None)[0]
            singular=np.linalg.svd(projected,compute_uv=False)
            condition=float(singular[0]/singular[1]) if singular[1]>0 else float('inf')
            require(singular[1]>.2 and condition<20,
                    '地球航向覆盖不足：扣除常量和时间漂移后无法区分两个水平分量，请增加不同航向；原数据已保留。')
            information.append(dict(projected_singular_values=singular.tolist(),condition=condition))
        return dict(static_segments_s=ranges,approximate_yaw_turns_deg=yaw_turns,
                    analysis_version=POSE_ANALYSIS_VERSION,observed_faces=faces,
                    approximate_headings_deg=headings.tolist(),heading_information=information,
                    final_return_present=len(ranges)==11,
                    reordered_samples=False,nonstandard_sequence=bool(
                        faces[5:10]!=['-X','+Y','-Y','+Z','-Z'] or
                        any(len(set(np.sign(a)))>1 for a in yaw_turns)))
    else:
        axis='xyz'.index(kind)
        require(np.all(abs(unit[:,axis])>.92),'选择的轴未竖直朝上，或录制中改变了朝上面。')
        require(np.all(unit@unit[0]>.98),'三次静置的朝向面不同，请保持同一面朝上。')
        angles=[]
        for d in data:
            rests=[np.mean(d['gyro'][(d['time']>=a)&(d['time']<=b)],axis=0) for a,b in ranges]
            turn=[]
            for k in range(2):
                m=(d['time']>ranges[k][1])&(d['time']<ranges[k+1][0])
                delta=np.sum((d['gyro'][m]-(rests[k]+rests[k+1])/2)*d['duration'][m,None],axis=0)
                require(abs(delta[axis])>6000 and abs(delta[axis])<8400,'旋转总角度明显不符 20 圈，请检查圈数后重录。')
                require(np.linalg.norm(np.delete(delta,axis))<abs(delta[axis])*.15,'旋转轴晃动过大或选错轴，请平稳绕指定轴旋转。')
                turn.append(float(delta[axis]))
            require(turn[0]*turn[1]<0,'两段旋转方向相同：第二段必须反向转 20 圈。')
            angles.append(turn)
        return dict(static_segments_s=ranges,approximate_turns=(np.array(angles)/360).tolist())
    return dict(static_segments_s=ranges)


def analyze(path, kind):
    require(kind in KINDS,'Unknown capture kind')
    data,header,quality=read_capture(path)
    clocks=align_clocks(data)
    result=static_segments(data,kind)
    result.update(kind=kind,clock_models=clocks,transport_pass=True,recording_sha256=digest(path))
    return data,header,result


def prepare(recordings, snapshots, latitude, out):
    require(set(recordings)==set(KINDS),'请先完成 X、Y、Z 和十姿态四次录制。')
    out=Path(out);out.mkdir(exist_ok=False,parents=True)
    config=dict(schema='JOINT10_INPUT1',dataset_kind='measured',latitude_deg=latitude,
                transport_pass=True,transport_diagnostic_override=False,devices=[],scale_captures=[],recordings=[])
    binding=None
    for kind in KINDS:
        path=Path(recordings[kind]);data,header,analysis=analyze(path,kind)
        identity=(header['receiver_serial'],header['targets'])
        require(binding is None or identity==binding,'四次录制的接收器或 RF 身份不一致。')
        binding=identity
        dataset_kind=header.get('dataset_kind','measured')
        require(dataset_kind in ('synthetic','measured'),'Unknown recording origin')
        if kind=='x':config['dataset_kind']=dataset_kind
        require(config['dataset_kind']==dataset_kind,'Synthetic and measured recordings cannot be mixed')
        files=[]
        for i,d in enumerate(data):
            arrays=out/f'{kind}-{i}.npz'
            np.savez_compressed(arrays,**{k:d[k] for k in ('time','interval_end','duration','gyro','accel','temp','seq')})
            files.append(dict(arrays=arrays.name,arrays_sha256=digest(arrays),clock_rate_correction_ppm=analysis['clock_models'][i]['clock_rate_correction_ppm']))
        (out/f'{kind}-analysis.json').write_text(json.dumps(analysis,indent=2),encoding='utf8')
        config['recordings'].append(dict(kind=kind,path=str(path.resolve()),sha256=digest(path)))
        if kind=='poses':
            config.update(recording=str(path.resolve()),recording_sha256=digest(path),static_segments_s=analysis['static_segments_s'],
                          final_return_present=analysis['final_return_present'])
            for i,entry in enumerate(files):
                snapshot=Path(snapshots)/f'slot-{i}/snapshot.bin'
                config['devices'].append(dict(slot=i,**entry,snapshot=str(snapshot.resolve()),snapshot_sha256=digest(snapshot)))
        else:
            config['scale_captures'].append(dict(axis=kind,devices=files,static_segments_s=analysis['static_segments_s']))
    path=out/'input.json';path.write_text(json.dumps(config,indent=2),encoding='utf8')
    return path


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recordings',type=Path,required=True)
    parser.add_argument('--snapshots',type=Path,required=True)
    parser.add_argument('--latitude',type=float,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    print(prepare(json.loads(args.recordings.read_text(encoding='utf8')),args.snapshots,args.latitude,args.out))

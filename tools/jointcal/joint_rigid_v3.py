"""All-ten joint rigid-body diagnostic. Never writes device parameters.

Common angular motion, individual mounts/scales/thermal tables/g-sensitivity;
accelerations include unknown relative lever arms, not a shared point path.
"""
import argparse,hashlib,json,sys,os
from pathlib import Path
# Repeated small/skinny least-squares systems are slower with automatic BLAS
# oversubscription. This only sets the current process default.
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
import numpy as np
from scipy.optimize import least_squares,minimize_scalar
from scipy.sparse import vstack,hstack,block_diag,kron,eye,coo_matrix,diags
from scipy.sparse.linalg import lsmr
from scipy.spatial.transform import Rotation as R
from scipy.signal import savgol_filter
import joint_fit_v2 as old
import refinement

OUT=None
HELD_MOVES={3,7};IDS=[0,5,6,7,8,9];DT=.02

def dump(name,value):
    (OUT/name).write_text(json.dumps(value,indent=2),encoding='utf-8')

def coverage(data):
    report=[]
    for i,d in enumerate(data):
        if d.get('scale_captures'):
            temps=np.concatenate([d['temp']]+[r['temp'] for r in d['scale_captures']])
            lo,hi=temps.min(),temps.max()
        else:
            rec=np.asarray([r[:22] for r in d['records']])
            lo=min(d['temp'].min(),rec[:,17].min(),rec[:,6].min())
            hi=max(d['temp'].max(),rec[:,18].max(),rec[:,6].max())
        # Require contiguous real observations across the full input support,
        # plus one bucket on either side, not just an extrapolated runtime LUT.
        needed=np.arange(max(0,int(np.floor((lo-15)*5))-1),min(139,int(np.floor((hi-15)*5))+1)+1)
        missing=[int(k) for k in needed if d['backup']['thermal_points'][k][0]==0]
        report.append(dict(slot=i,input_temperature_c=[float(lo),float(hi)],missing_indices=missing,
                           missing_centers_c=[round(15.1+.2*k,1) for k in missing],covered=bool(not missing and lo>=15 and hi<43)))
    dump('coverage.json',report)
    if not all(x['covered'] for x in report):
        details=['#'+str(x['slot'])+': '+str(x['input_temperature_c'])+' °C; 缺少 '+str(x['missing_centers_c'])+' °C' for x in report if not x['covered']]
        raise ValueError('温度表未覆盖录制温度，请补齐温度观测后重新求解。'+'; '.join(details))
    return report

def mounting(data,st):
    # Body frame gauge is sensor 0. The common gravity directions are refined
    # from all ten, not taken as exact from any one accelerometer.
    centered=[s[1][IDS]-s[1][IDS].mean(axis=0) for s in st]
    common=centered[0].copy();qs=[R.identity()]*10
    for _ in range(30):
        previous=common.copy()
        qs=[R.align_vectors(common,a)[0] for a in centered]
        gauge=qs[0].inv();qs=[gauge*q for q in qs]
        common=np.mean([q.apply(a) for q,a in zip(qs,centered)],axis=0)
        if np.max(abs(common-previous))<1e-10:break
    for d,q in zip(data,qs):d['Qseed']=q;d['Q']=q
    return common

def continuous_turns(d):
    turns=[]
    diagnostics=[]
    for axis,capture in enumerate(d['scale_captures']):
        t=capture['time'];rests=capture['rests']
        corrected=capture['gyro']-old.table(d,capture['temp'])-capture['accel']@d['M'].T
        bias=np.array([np.mean(corrected[(t>=a)&(t<=b)],axis=0) for a,b in rests])
        centers=np.mean(rests,axis=1)
        base=np.column_stack([np.interp(t,centers,bias[:,j]) for j in range(3)])
        for k in range(2):
            m=(t>rests[k][1])&(t<rests[k+1][0])
            turns.append(np.sum((corrected[m]-base[m])*capture['duration'][m,None],axis=0))
            diagnostics.append(dict(axis='xyz'[axis],direction_in_recording=k+1,
                duration_s=float(capture['duration'][m].sum()),
                temperature_range_c=[float(capture['temp'][m].min()),float(capture['temp'][m].max())],
                mean_abs_rate_dps=float(np.mean(np.linalg.norm(corrected[m]-base[m],axis=1))),
                rest_bias_before_dps=bias[k].tolist(),rest_bias_after_dps=bias[k+1].tolist()))
    d['scale_capture_diagnostics']=diagnostics
    return np.asarray(turns)


def joint_scales(data,window_rows=None):
    delta=[]
    for d in data:
        if d.get('scale_captures'):
            delta.append(continuous_turns(d))
            continue
        rows=[]
        for r in d['records']:
            v=np.asarray(r[:22]);duration=v[14]
            meanT=25+v[19]/duration # stored moments are about 25 C
            rows.append(v[7:10]-v[:3]*duration-(old.table(d,meanT)-old.table(d,v[6]))*duration
                        -d['M']@(v[10:13]-v[3:6]*duration))
        delta.append(rows)
    delta=np.asarray(delta)
    bidirectional=delta.shape[1]==6
    seed=np.mean([d['Qseed'].apply(delta[i]*d['S']) for i,d in enumerate(data)],axis=0)
    if bidirectional:seed=(seed[::2]-seed[1::2])/2
    seed/=np.linalg.norm(seed,axis=1)[:,None]
    mounting_seed=np.array([(d.get('Q',d['Qseed'])*d['Qseed'].inv()).as_rotvec() for d in data])[1:]
    x0=np.r_[np.log(np.array([d['S'] for d in data])).ravel(),mounting_seed.ravel(),seed.ravel()]
    lower=np.r_[np.full(30,np.log(.9)),np.full(27,-.04),np.full(9,-1.5)]
    upper=np.r_[np.full(30,np.log(1.1)),np.full(27,.04),np.full(9,1.5)]
    if window_rows:
        # The latent curve is estimated from these same trackers. Let its
        # common linear frame/gain adjust here so it cannot lock absolute scale
        # to the previous iteration instead of the known 20-turn angles.
        x0=np.r_[x0,np.zeros(9)];lower=np.r_[lower,np.full(9,-.03)];upper=np.r_[upper,np.full(9,.03)]
    def unpack(x):
        scales=np.exp(x[:30].reshape(10,3));dq=np.vstack([np.zeros(3),x[30:57].reshape(9,3)])
        qs=[R.from_rotvec(q)*d['Qseed'] for q,d in zip(dq,data)]
        dirs=x[57:66].reshape(3,3);dirs=dirs/np.linalg.norm(dirs,axis=1)[:,None]
        return scales,qs,dirs
    def fun(x):
        scales,qs,dirs=unpack(x)
        expected=np.stack([dirs,-dirs],axis=1).reshape(6,3) if bidirectional else dirs
        difference=np.array([q.apply(delta[i]*scales[i])-7200*expected for i,q in enumerate(qs)])
        # A measured mounting prior resolves the frame/calibration ambiguity.
        # Its scale is deliberately reported; it is not an accuracy claim.
        residual=[difference.ravel()/2,x[30:57]/np.deg2rad(.2),
                  (np.linalg.norm(x[57:66].reshape(3,3),axis=1)-1)*100]
        if window_rows:
            transform=np.eye(3)+x[66:].reshape(3,3)
            # Compare in the measured sensor units. Scaling the observation
            # and its residual by the fitted gain lets unmodeled motion reward
            # smaller gains, despite a freely adjustable common latent curve.
            residual += [(rows[:,:3]-rows[:,3:6]@d['M'].T-q.inv().apply(rows[:,6:9]@transform.T)/s).ravel()
                         for d,s,q,rows in zip(data,scales,qs,window_rows)]
        return np.concatenate(residual)
    result=least_squares(fun,x0,bounds=(lower,upper),max_nfev=150,xtol=1e-10,ftol=1e-10,gtol=1e-8)
    scales,qs,dirs=unpack(result.x)
    expected=np.stack([dirs,-dirs],axis=1).reshape(6,3) if bidirectional else dirs
    residual=np.array([q.apply(delta[i]*scales[i])-7200*expected for i,q in enumerate(qs)])
    along=np.sum(residual*expected,axis=2)
    transverse=residual-along[:,:,None]*expected
    for d,s,q in zip(data,scales,qs):d['S']=s;d['Q']=q
    directional=[]
    if bidirectional:
        for i,d in enumerate(data):
            # Project each independently onto the all-ten shared axis; publish
            # both candidates, but firmware still installs one scale per axis.
            projection=np.sum(d['Q'].apply(delta[i])*expected,axis=1).reshape(3,2)
            candidates=7200/projection
            mismatch=2*abs(candidates[:,0]-candidates[:,1])/(candidates[:,0]+candidates[:,1])
            signs=np.sign(delta[i,2*np.arange(3),np.arange(3)])
            positive=np.where(signs>0,candidates[:,0],candidates[:,1])
            negative=np.where(signs>0,candidates[:,1],candidates[:,0])
            directional.append(dict(slot=i,forward_scale=candidates[:,0].tolist(),reverse_scale=candidates[:,1].tolist(),
                                    positive_scale_candidate=positive.tolist(),negative_scale_candidate=negative.tolist(),
                                    corrected_turn_integrals_deg=delta[i].tolist(),relative_difference=mismatch.tolist(),
                                    capture_diagnostics=d['scale_capture_diagnostics'],
                                    passed=bool(np.all(mismatch<=.002)),
                                    firmware_candidate_only=True,
                                    interpretation='Retain all windows; check temperature, rest bias, speed and turn-count repeatability before adding signed firmware coefficients.'))
    return dict(success=bool(result.success),joint_angle_rms_deg=float(np.sqrt(np.mean(residual**2))),
        turn_residual_definition='RMS of 10 devices x turns x 3 integrated-vector components; not endpoint orientation error',
        turn_axis_rms_deg=float(np.sqrt(np.mean(along**2))),
        turn_transverse_norm_rms_deg=float(np.sqrt(np.mean(np.sum(transverse**2,axis=2)))),
        turn_axis_residuals_deg=along.tolist(),
        window_residual_frame='measured sensor coordinates; fitted scale does not multiply residual noise',
        bidirectional=bidirectional,direction_checks=directional,direction_difference_limit=.002,
        scales=scales.tolist(),shared_turn_vectors_deg=(7200*dirs).tolist(),window_constraints=bool(window_rows),
        window_curve_transform=(np.eye(3)+result.x[66:].reshape(3,3)).tolist() if window_rows else None,
        mounting_adjustments_deg=np.rad2deg(result.x[30:57].reshape(9,3)).tolist(),
        per_device_rms_deg=np.sqrt(np.mean(residual**2,axis=(1,2))).tolist(),
        limitation='Continuous 50-Hz window integrals retain both rotations; sub-window coning is not resolved.' if bidirectional else
                   '20-turn vector constraint assumes nearly fixed rotation axis; no full-rate coning history exists.')

def mean_curve(knots,rate,start,end):
    h=knots[1]-knots[0];integral=np.vstack([np.zeros(3),np.cumsum((rate[:-1]+rate[1:])*.5*h,axis=0)])
    def primitive(t):
        k=np.clip(np.floor((t-knots[0])/h).astype(int),0,len(knots)-2);u=t-knots[k]
        return integral[k]+rate[k]*u[:,None]+(rate[k+1]-rate[k])*(u*u/(2*h))[:,None]
    return (primitive(end)-primitive(start))/(end-start)[:,None]

def clock_residual(x,pairs,jacobian=False):
    """Analytic interval derivatives avoid sub-ULP ppm finite differences."""
    result=[];offset=x[0]*.001;ppm=x[1];factor=1+ppm*1e-6
    for seg,a,b,y in pairs:
        aa=a+offset+(a-300)*ppm*1e-6;bb=b+offset+(b-300)*ppm*1e-6
        mean=mean_curve(seg['knots'],seg['rate'],aa,bb)
        if not jacobian:
            result.append((mean-y/factor).ravel());continue
        knots=seg['knots'];rate=seg['rate'];h=knots[1]-knots[0]
        def endpoint(t):
            k=np.clip(np.floor((t-knots[0])/h).astype(int),0,len(knots)-2)
            return rate[k]+(rate[k+1]-rate[k])*((t-knots[k])/h)[:,None]
        va=endpoint(aa);vb=endpoint(bb);span=(bb-aa)[:,None]
        offset_derivative=(vb-va)/span*.001
        rate_derivative=((vb-mean)*(b-300)[:,None]-(va-mean)*(a-300)[:,None])/span*1e-6+y*1e-6/factor**2
        result.append(np.c_[offset_derivative.ravel(),rate_derivative.ravel()])
    if jacobian:return np.vstack([*result,[[0.,.005]]])
    return np.r_[np.concatenate(result),ppm*.005]

def dynamic_observations(data,st,clocks):
    obs=[]
    for i,d in enumerate(data):
        y,a,t,_=st[i]
        base=np.column_stack([np.interp(d['time'],t,y[:,j]) for j in range(3)])
        abase=np.column_stack([np.interp(d['time'],t,a[:,j]) for j in range(3)])
        gy=d['gyro']-old.table(d,d['temp'])-base
        rate=d['Q'].apply((gy-(d['accel']-abase)@d['M'].T)*d['S'])/d['clock_rate']
        offset,ppm=clocks[i];factor=1+ppm*1e-6
        end=d['interval_end']+offset+(d['interval_end']-300)*ppm*1e-6
        begin=end-d['interval_duration']*factor
        obs.append(dict(start=begin,end=end,rate=rate/factor,accel=d['Q'].apply(d['accel']),
                        gyro_delta=gy,accel_base=abase,clock_factor=np.full(len(end),d['clock_rate']*factor)))
    return obs

def motion(data,st,clocks,ranges=None,step=DT):
    obs=dynamic_observations(data,st,clocks);segments=[]
    up=np.mean([d['Q'].apply(s[1][0]) for d,s in zip(data,st)],axis=0);up/=np.linalg.norm(up)
    poses=[R.identity()]
    for m,(lo,hi) in enumerate(old.MOV if ranges is None else ranges):
        knots=np.arange(lo,hi+step*.5,step);operators=[];values=[];indexes=[]
        for i,o in enumerate(obs):
            ix=np.flatnonzero((o['start']>=knots[0]+.025)&(o['end']<=knots[-1]-.025))
            A=old.interval_operator(o['start'][ix],o['end'][ix],knots)
            operators.append(A);values.append(o['rate'][ix]);indexes.append(ix)
        A=vstack(operators).tocsr();Y=np.concatenate(values)
        n=len(knots);D=diags([np.ones(n-2),-2*np.ones(n-2),np.ones(n-2)],[0,1,2],shape=(n-2,n))
        matrix=vstack([A,.15*D]);target=np.vstack([Y,np.zeros((n-2,3))])
        rate,curve_check=refinement.solve_curve(matrix,target)
        q=poses[-1]
        for w in (rate[:-1]+rate[1:])*.5:q=q*R.from_rotvec(np.deg2rad(w*step))
        av=np.mean([d['Q'].apply(s[1][m+1]) for d,s in zip(data,st)],axis=0);av/=np.linalg.norm(av)
        predicted=q.inv().apply(up);angle=np.rad2deg(np.arccos(np.clip(predicted@av,-1,1)))
        tilt,_=R.align_vectors(av[None],predicted[None]);poses.append(q*tilt.inv())
        segments.append(dict(index=m,knots=knots,rate=rate,operators=operators,indexes=indexes,values=values,
            gravity_error_deg=float(angle),fit_rms_dps=float(np.sqrt(np.mean((A@rate-Y)**2))),smooth=D,curve_solve=curve_check))
    return obs,segments,poses,up

def refine_clocks(data,st,clocks,segments,return_status=False):
    base=dynamic_observations(data,st,np.zeros((10,2)));new=clocks.copy();success=True
    for i in range(10):
        pairs=[]
        for seg in segments:
            if seg['index'] in HELD_MOVES:continue
            ix=seg['indexes'][i];o=base[i]
            pairs.append((seg,o['start'][ix],o['end'][ix],o['rate'][ix]))
        guess=np.clip([clocks[i,0]*1000,clocks[i,1]],[-14.999,-49.999],[14.999,49.999])
        r=least_squares(lambda x:clock_residual(x,pairs),guess,
                        jac=lambda x:clock_residual(x,pairs,jacobian=True),
                        bounds=([-15,-50],[15,50]),max_nfev=25)
        success=bool(success and r.success)
        new[i]=[r.x[0]*.001,r.x[1]]
    # Fit all devices against the current latent curve, then restore the time
    # gauge. Leaving device 0 out of the fit makes common-offset convergence
    # unnecessarily slow because the curve itself follows the other nine.
    reference=1+new[0,1]*1e-6
    new[:,0]=(new[:,0]-new[0,0])/reference
    new[:,1]=((1+new[:,1]*1e-6)/reference-1)*1e6
    return (new,success) if return_status else new

def earth_gsens(data,st,held,poses,up,omit=None,window_rows=None):
    guide=np.eye(3)[np.argmin(abs(up))];h1=guide-up*(guide@up);h1/=np.linalg.norm(h1);h2=np.cross(up,h1)
    vertical=.0041780746*np.sin(np.deg2rad(old.LATITUDE_DEG));horizontal=.0041780746*np.cos(np.deg2rad(old.LATITUDE_DEG))
    count=len(poses)
    keep=np.arange(count)!=omit if omit is not None else np.ones(count,dtype=bool)
    specs=[];weights=np.ones((10,count));previous=None;robust_converged=False
    for i,d in enumerate(data):
        y,a,t,_=st[i];D=np.c_[a,np.ones(count),(t-t[0])/100]
        V=np.array([d['Q'].inv().apply(q.inv().apply(up))*vertical/d['S'] for q in poses])
        B=np.array([[d['Q'].inv().apply(q.inv().apply(h))/d['S'] for h in (h1,h2)] for q in poses]).transpose(0,2,1)
        sigma=np.maximum(np.asarray(d.get('static_sigma_dps',np.full(count,.0002))),.0002)
        specs.append((D,V,B,sigma))
    for robust_iteration in range(100):
        parts=[];targets=[];systems=[]
        for i,d in enumerate(data):
            y,_,_,_=st[i];D,V,B,sigma=specs[i]
            w=np.sqrt(weights[i,keep])/sigma[keep]
            design=D[keep]*w[:,None];target=(y-V)[keep]*w[:,None];basis=B[keep]*w[:,None,None]
            if window_rows is not None and omit is None:
                r=window_rows[i]
                design=np.vstack([design,np.c_[r[:,3:6],np.zeros((len(r),2))]])
                target=np.vstack([target,r[:,:3]-d['Q'].inv().apply(r[:,6:9])/d['S']])
                basis=np.concatenate([basis,np.zeros((len(r),3,2))])
            flat=basis.reshape(len(design),6)
            projection=flat-design@np.linalg.lstsq(design,flat,rcond=None)[0]
            residual=target-design@np.linalg.lstsq(design,target,rcond=None)[0]
            parts.append(projection.reshape(-1,2));targets.append(residual.ravel())
            systems.append((design,target,basis))
        projected=np.concatenate(parts);observed=np.concatenate(targets)
        free=np.linalg.lstsq(projected,observed,rcond=None)[0]
        def cost(angle):
            error=projected@(horizontal*np.array([np.cos(angle),np.sin(angle)]))-observed
            return float(error@error)
        grid=np.linspace(-np.pi,np.pi,361);best=min(grid,key=cost)
        phase=minimize_scalar(cost,bounds=(best-.1,best+.1),method='bounded',options={'xatol':1e-12}).x
        eh=horizontal*np.array([np.cos(phase),np.sin(phase)]);coefficients=[]
        updated=weights.copy()
        for i,((design,target,basis),(D,V,B,sigma)) in enumerate(zip(systems,specs)):
            C=np.linalg.lstsq(design,target-basis@eh,rcond=None)[0];coefficients.append(C)
            error=st[i][0]-V-B@eh-D@C
            updated[i]=np.minimum(1,3*sigma/np.maximum(np.linalg.norm(error,axis=1)/np.sqrt(3),1e-12))
        state=np.r_[np.asarray(coefficients).ravel(),np.cos(phase),np.sin(phase)]
        if previous is not None and np.max(abs(state-previous))<1e-7 and np.max(abs(updated-weights))<1e-3:
            robust_converged=True;break
        previous=state;weights=updated
    E=vertical*up+eh[0]*h1+eh[1]*h2;rows=[];matrices=[]
    for i,d in enumerate(data):
        y,a,t,_=st[i];hy,ha,ht,_=held[i];D,V,B,sigma=specs[i];C=coefficients[i]
        train=y-V-B@eh-D@C;test=hy-V-B@eh-np.c_[ha,np.ones(count),(ht-t[0])/100]@C
        matrices.append(C[:3].T)
        design=systems[i][0];condition=float(np.linalg.cond(design));rank=int(np.linalg.matrix_rank(design))
        covariance=np.linalg.pinv(design.T@design,rcond=1e-12)
        stderr=np.sqrt(np.maximum(np.diag(covariance)[:3],0))
        rows.append(dict(slot=i,scale=d['S'].tolist(),gsens_dps_per_g=C[:3].T.tolist(),earth_dps=d['Q'].inv().apply(E).tolist(),
            earth_train_rms_dps=float(np.sqrt(np.mean(train[:5]**2))),earth_holdout_rms_dps=float(np.sqrt(np.mean(test[:5]**2))),
            gsens_train_rms_dps=float(np.sqrt(np.mean(train[IDS]**2))),gsens_holdout_rms_dps=float(np.sqrt(np.mean(test[IDS]**2))),
            omitted_pose_residual_dps=train[omit].tolist() if omit is not None else None,
            design_condition=condition if np.isfinite(condition) else None,design_rank=rank,
            conditional_gsens_stderr_dps_per_g=stderr.tolist(),static_robust_weights=weights[i].tolist(),
            bias_dps=C[3].tolist(),drift_per_100s_dps=C[4].tolist(),installable=False))
        if count==11:
            rows[-1]['return_train_residual_dps']=train[-1].tolist()
            rows[-1]['return_holdout_residual_dps']=test[-1].tolist()
    singular=np.linalg.svd(projected,compute_uv=False);rank=int(np.linalg.matrix_rank(projected))
    derivative=projected@(horizontal*np.array([-np.sin(phase),np.cos(phase)]))
    information=float(derivative@derivative)
    phase_std=float(np.rad2deg(1/np.sqrt(information))) if information>1e-16 else None
    ratio=float(np.linalg.norm(free)/horizontal)
    identifiable=bool(rank==2 and phase_std is not None and phase_std<=30 and .5<=ratio<=1.5
                      and all(r['design_rank']==5 and r['design_condition']<1e6 for r in rows))
    return dict(horizontal_free_magnitude_ratio=ratio,phase_rad=float(phase),devices=rows,
        earth_information=dict(identifiable=identifiable,projected_rank=rank,singular_values=singular.tolist(),
            conditional_phase_std_deg=phase_std,free_magnitude_ratio_bounds=[.5,1.5],phase_std_limit_deg=30,
            limitation='Conditional linearized diagnostic with fixed clocks/mounts/scales, not external accuracy.'),
        robust_iterations=robust_iteration+1,robust_converged=robust_converged,
        dynamic_window_constraints=window_rows is not None),matrices


def final_return_check(st,held,poses,static):
    if len(poses)!=11:return dict(present=False)
    rows=[]
    for i,(s,h,r) in enumerate(zip(st,held,static['devices'])):
        gravity=s[1][[0,-1]];gravity=gravity/np.linalg.norm(gravity,axis=1)[:,None]
        difference=h[0][-1]-h[0][0]-(h[1][-1]-h[1][0])@np.asarray(r['gsens_dps_per_g']).T
        drift=np.asarray(r['drift_per_100s_dps'])*(h[2][-1]-h[2][0])/100
        rows.append(dict(slot=i,gravity_return_deg=float(np.rad2deg(np.arccos(np.clip(gravity[0]@gravity[1],-1,1)))),
            corrected_first_last_difference_dps=difference.tolist(),fitted_drift_difference_dps=drift.tolist(),
            return_holdout_rms_dps=float(np.sqrt(np.mean(np.asarray(r['return_holdout_residual_dps'])**2)))))
    closure=float(np.rad2deg(poses[-1].magnitude()))
    return dict(present=True,orientation_closure_deg=closure,limit_deg=3.,
        passed=bool(closure<=3 and all(r['gravity_return_deg']<=3 for r in rows)),devices=rows,
        interpretation='Final rest extends the fitted bias/drift time span; its second half is held out. Orientation is integrated without forcing return to identity; gravity alone cannot verify heading. Not an independent calibration.')


def angular_holdout(segments):
    reports=[]
    for seg in segments:
        if seg['index'] not in HELD_MOVES:continue
        for excluded in range(10):
            A=vstack([a for i,a in enumerate(seg['operators']) if i!=excluded]);Y=np.concatenate([v for i,v in enumerate(seg['values']) if i!=excluded])
            matrix=vstack([A,.15*seg['smooth']]);target=np.vstack([Y,np.zeros((seg['smooth'].shape[0],3))])
            w=np.column_stack([lsmr(matrix,target[:,j],atol=1e-8,btol=1e-8,maxiter=800)[0] for j in range(3)])
            error=seg['operators'][excluded]@w-seg['values'][excluded]
            reports.append(dict(motion=seg['index'],excluded_tracker=excluded,rms_dps=float(np.sqrt(np.mean(error**2)))))
    return reports

def cross_matrix(v):
    a=np.zeros((len(v),3,3));a[:,0,1]=-v[:,2];a[:,0,2]=v[:,1]
    a[:,1,0]=v[:,2];a[:,1,2]=-v[:,0];a[:,2,0]=-v[:,1];a[:,2,1]=v[:,0]
    return a

def kinematics(seg):
    step=float(seg['knots'][1]-seg['knots'][0]);window=max(5,int(round(.2/step))|1)
    w=np.deg2rad(savgol_filter(seg['rate'],window,3,axis=0))
    alpha=np.deg2rad(savgol_filter(seg['rate'],window,3,deriv=1,delta=step,axis=0))
    W=cross_matrix(w)
    B=(cross_matrix(alpha)+W@W)/9.80665
    return [(A@B.reshape(len(B),9)).reshape(-1,3,3) for A in seg['operators']]

def scaled_lsmr(A,y):
    size=np.sqrt(np.asarray(A.power(2).sum(axis=0)).ravel());size=np.maximum(size,1e-10)
    ans=lsmr(A@diags(1/size),y,atol=1e-8,btol=1e-8,maxiter=1500)
    return ans[0]/size,dict(stop=int(ans[1]),iterations=int(ans[2]),estimated_condition=float(ans[6]))

def fit_lever_arms(data,st,obs,segments,validate=True):
    acc=[o['accel']-d['Q'].apply(s[1][IDS].mean(axis=0)) for o,d,s in zip(obs,data,st)]
    train=[s for s in segments if s['index'] not in HELD_MOVES]
    common=[];smooth=[];ys=[];ri=[];ci=[];vals=[];rows=[];crows=[];cvals=[];nrow=0
    for seg in train:
        common.append(vstack(seg['operators']));smooth.append(seg['smooth'])
        for slot,B in enumerate(kinematics(seg)):
            y=acc[slot][seg['indexes'][slot]];ys.append(y.ravel());n=len(y)
            if slot:
                for j in range(3):
                    rr=nrow+np.arange(n)*3+j
                    crows.extend(rr);ci.extend(np.full(n,(slot-1)*3+j));cvals.extend(np.ones(n))
                    for k in range(3):
                        rows.extend(rr);ri.extend(np.full(n,(slot-1)*3+k));vals.extend(B[:,j,k])
            nrow+=n*3
    F=kron(block_diag(common),eye(3)).tocsr();D=kron(block_diag(smooth),eye(3)).tocsr()
    P=coo_matrix((vals,(rows,ri)),shape=(nrow,27)).tocsr()
    C=coo_matrix((cvals,(crows,ci)),shape=(nrow,27)).tocsr();Y=np.concatenate(ys)
    A=hstack([F,P,C]).tocsr()
    penalty=vstack([hstack([.15*D,coo_matrix((D.shape[0],54))]),
        hstack([coo_matrix((54,F.shape[1])),diags(np.r_[np.full(27,.002),np.full(27,.2)])])]).tocsr()
    solution,solveinfo=scaled_lsmr(vstack([A,penalty]).tocsr(),np.r_[Y,np.zeros(penalty.shape[0])])
    position=np.vstack([np.zeros(3),solution[F.shape[1]:F.shape[1]+27].reshape(9,3)])
    bias=np.vstack([np.zeros(3),solution[-27:].reshape(9,3)])
    # Co-located baseline is refitted to the same training observations.
    A0=hstack([F,C]).tocsr();P0=vstack([hstack([.15*D,coo_matrix((D.shape[0],27))]),
         hstack([coo_matrix((27,F.shape[1])),.2*eye(27)])]).tocsr()
    z,baseinfo=scaled_lsmr(vstack([A0,P0]).tocsr(),np.r_[Y,np.zeros(P0.shape[0])]) if validate else (np.zeros(A0.shape[1]),{})
    bias0=np.vstack([np.zeros(3),z[-27:].reshape(9,3)])
    tests=[]
    for seg in segments if validate else []:
        if seg['index'] not in HELD_MOVES:continue
        terms=kinematics(seg)
        readings=[acc[i][ix] for i,ix in enumerate(seg['indexes'])]
        for excluded in range(10):
            AA=vstack([a for i,a in enumerate(seg['operators']) if i!=excluded]).tocsr()
            mat=vstack([AA,.15*seg['smooth']]).tocsr();errors=[]
            for use_arm,biases in [(True,bias),(False,bias0)]:
                corrected=[y-(B@position[i] if use_arm else 0)-biases[i] for i,(y,B) in enumerate(zip(readings,terms))]
                target=np.concatenate([y for i,y in enumerate(corrected) if i!=excluded])
                target=np.vstack([target,np.zeros((seg['smooth'].shape[0],3))])
                force=np.column_stack([lsmr(mat,target[:,j],atol=1e-8,btol=1e-8,maxiter=800)[0] for j in range(3)])
                err=seg['operators'][excluded]@force-corrected[excluded]
                errors.append(float(np.sqrt(np.mean(err**2))))
            tests.append(dict(motion=seg['index'],excluded_tracker=excluded,rigid_rms_g=errors[0],colocated_rms_g=errors[1]))
    return dict(relative_positions_m=position.tolist(),accelerometer_residual_bias_g=bias.tolist(),
        train_rms_g=float(np.sqrt(np.mean((A@solution-Y)**2))),colocated_train_rms_g=float(np.sqrt(np.mean((A0@z-Y)**2))) if validate else None,
        held_motion_tests=tests,solve=solveinfo,baseline_solve=baseinfo,
        constraints='r0=0; bias0=0; weak 1 m lever-arm and 0.01 g bias priors; common specific force in body frame',
        limitation='Relative positions are inferred, not surveyed; no claim of absolute position trajectory.')

def run():
    global OUT
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--max-iterations',type=int,default=30,help='Safety ceiling; an unconverged result cannot produce an installation plan')
    args=p.parse_args();OUT=args.out
    controller=refinement.Convergence(maximum=args.max_iterations,minimum=min(4,args.max_iterations))
    config=old.configure(args.config,old)
    OUT.mkdir(parents=True,exist_ok=False)
    data=old.load();cov=coverage(data)
    st=[old.means(d,'train') for d in data];held=[old.means(d,'hold') for d in data]
    for d in data:d['static_sigma_dps']=refinement.static_uncertainty(sys.modules[__name__],d)
    mounting(data,st);clocks=np.zeros((10,2));axis_clocks=[np.zeros((10,2)) for _ in range(3)]
    iterations=[];window_rows=None
    for iteration in range(controller.maximum):
        print(json.dumps(dict(iteration=iteration+1,stage='scale_and_pose_motion')),flush=True)
        scales=joint_scales(data,window_rows)
        obs,segments,poses,up,clocks,clock_check=refinement.clock_motion(sys.modules[__name__],data,st,clocks)
        print(json.dumps(dict(iteration=iteration+1,stage='three_axis_motion')),flush=True)
        all_obs,all_segments,axis_clocks,axis_checks=refinement.all_recordings(
            sys.modules[__name__],data,st,obs,segments,axis_clocks)
        print(json.dumps(dict(iteration=iteration+1,stage='rigid_acceleration')),flush=True)
        rigid=fit_lever_arms(data,st,all_obs,all_segments,validate=False)
        print(json.dumps(dict(iteration=iteration+1,stage='window_and_static_refinement')),flush=True)
        window_rows,window_diagnostics=refinement.window_statistics(sys.modules[__name__],data,st,all_obs,all_segments,rigid)
        static,matrices=earth_gsens(data,st,held,poses,up,window_rows=window_rows)
        if not np.isfinite(matrices).all() or max(np.max(abs(m)) for m in matrices)>.05:
            dump('failed-static.json',static)
            raise ValueError('Joint g-sens fit exceeds physical bounds; diagnostic preserved')
        for d,m in zip(data,matrices):d['M']=m
        # Raw-unit scores have fixed scaling across iterations; changing robust
        # weights cannot by itself manufacture an objective improvement.
        static_error=np.mean([r['earth_train_rms_dps']**2+r['gsens_train_rms_dps']**2 for r in static['devices']])
        angular_error=np.mean([s['fit_rms_dps']**2 for s in all_segments])
        objective=float((scales['joint_angle_rms_deg']/2)**2+static_error/.001**2+
                        angular_error/.02**2+(rigid['train_rms_g']/.003)**2)
        state=refinement.snapshot(data,clocks,axis_clocks,static['phase_rad'],rigid['relative_positions_m'])
        inner_ok=scales['success'] and static['robust_converged'] and clock_check['converged'] and all(c['converged'] for c in axis_checks)
        inner_ok=inner_ok and rigid['solve']['stop'] in (0,1,2,4,5) and window_diagnostics['inner_converged']
        done=controller.update(state,objective,inner_ok=inner_ok)
        iterations.append(dict(iteration=iteration,scale=scales,static=static,clocks=clocks.tolist(),
            axis_clocks=[c.tolist() for c in axis_clocks],clock_checks=[clock_check,*axis_checks],
            rigid_train_rms_g=rigid['train_rms_g'],rigid_solve=rigid['solve'],convergence=controller.history[-1]))
        dump('iterations.json',iterations);dump('convergence.json',controller.report())
        dump('window-refinement.json',window_diagnostics)
        print(json.dumps(dict(iteration=iteration+1,converged=done,objective=objective,
            changes=controller.history[-1]['changes'],turn_rms_deg=scales['joint_angle_rms_deg'],
            earth_ratio=static['horizontal_free_magnitude_ratio'],inner_converged=bool(inner_ok))),flush=True)
        if done:break
    angular=angular_holdout(segments);poses_out=[]
    # Omit the entire pose across all ten from this static refit. Dynamic QR
    # terms are also omitted because their baselines used those static samples.
    for omitted in IDS:
        result,_=earth_gsens(data,st,held,poses,up,omitted)
        poses_out.append(dict(omitted_pose=omitted,earth_phase_rad=result['phase_rad'],
            residuals_dps=[x['omitted_pose_residual_dps'] for x in result['devices']]))
    print('Refinement complete; validating individual rigid accelerations on held motions',flush=True)
    rigid=fit_lever_arms(data,st,all_obs,all_segments);dump('rigid-acceleration.json',rigid)
    report=dict(status='JOINT_DIAGNOSTIC_UNAPPLIED',solver_version='adaptive-window-rigid-1',
        numerics_version='sensor-residual-window20ms-2',all_ten_used=True,
        convergence=controller.report(),coverage=cov,iterations=iterations,static=static,
        angular_held_motion_tests=angular,static_leave_pose_out=poses_out,
        joint_pose_quaternions_xyzw=[q.as_quat().tolist() for q in poses],
        final_return=final_return_check(st,held,poses,static),
        earth_loop_closure_deg=float(np.rad2deg(poses[4].magnitude())),
        motion=[dict(index=s['index'],source=s.get('source','poses'),gravity_error_deg=s['gravity_error_deg'],fit_rms_dps=s['fit_rms_dps']) for s in all_segments],
        rigid_acceleration=rigid,window_refinement=window_diagnostics,
        snapshot_hashes=[d['backup']['snapshot_sha256'] for d in data],
        source_recording_sha256=config['recording_sha256'],source_recordings=config.get('recordings',[]),input_config=config['public_description'],
        installable=False,limitations=[
          'Original F04 windows remain measurements; QR statistics are objective compression, not extra observations.',
          'Latent angular grids: 20 ms for both pose transitions and long turns; sub-window coning remains unresolved.',
          'Common gyro bias and g-sens components still require the static/known-turn anchors; ten devices are not an external truth.',
          'Motion Earth contributions are approximated through adjacent static baselines and shared angular curves.',
          'Whole-pose validation refits the static model only; scales/mounts/clocks were already estimated from this session.',
          'Conditional uncertainties hold other calibration blocks fixed and do not prove physical accuracy.',
          'Block-alternating convergence is numerical stability, not a guarantee of global optimality; motion uses the preceding g-sens update within the reported stopping tolerances.',
          'Temperature LUTs stay fixed; separate thermal hysteresis/independent thermal cycles are not inferred here.',
          'Separate direction coefficients remain diagnostic; no new firmware storage format or sign-switching is installed.'])
    dump('result.json',report)
    print(json.dumps(dict(result=str(OUT/'result.json'),converged=controller.converged,iterations=len(iterations),
        closure_deg=report['earth_loop_closure_deg'],rigid_train=rigid['train_rms_g'],
        colocated_train=rigid['colocated_train_rms_g'],installable=False)),flush=True)

if __name__=='__main__':run()

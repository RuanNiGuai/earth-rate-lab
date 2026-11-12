"""Window-preserving joint refinement and explicit stopping diagnostics.

QR rows are sufficient statistics for a weighted least-squares objective, not
new sensor measurements. All original windows stay in the input archives.
"""
import numpy as np
from scipy.sparse import vstack
from scipy.sparse.linalg import lsmr

LIMITS=dict(scale_relative=2e-6,gsens_dps_per_g=2e-6,mount_deg=.001,
            clock_offset_s=1e-5,clock_ppm=.05,earth_phase_rad=.001,
            lever_arm_m=.0005,objective_relative=.002)


def solve_curve(matrix,target):
    solved=[lsmr(matrix,target[:,k],atol=1e-8,btol=1e-8,maxiter=800) for k in range(3)]
    return np.column_stack([r[0] for r in solved]),dict(
        converged=all(r[1] in (0,1,2,4,5) and np.isfinite(r[0]).all() for r in solved),
        stop=[int(r[1]) for r in solved],iterations=[int(r[2]) for r in solved])


def static_uncertainty(j,d):
    result=[]
    for lo,hi in j.old.SEG:
        end=(lo+hi)/2-1;means=[]
        for a in np.arange(lo+2,end-.5,.5):
            mask=(d['time']>=a)&(d['time']<a+.5)
            if mask.any():means.append(np.mean(d['gyro'][mask]-j.old.table(d,d['temp'][mask]),axis=0))
        if len(means)<4:result.append(.0002);continue
        delta=np.diff(means,axis=0)
        scatter=1.4826*np.median(abs(delta-np.median(delta,axis=0)),axis=0)/np.sqrt(2)
        result.append(max(.0002,float(scatter.max()/np.sqrt(len(means)))))
    return result


def compress_rows(features, targets, weights):
    values=np.column_stack((features,targets))*np.sqrt(weights)[:,None]
    if not np.isfinite(values).all():raise ValueError('Non-finite dynamic observations')
    return np.linalg.qr(values,mode='r')


def snapshot(data,clocks,axis_clocks,phase,positions):
    return dict(scale=np.array([d['S'] for d in data]),gsens=np.array([d['M'] for d in data]),
                mount=np.array([d['Q'].as_matrix() for d in data]),
                clocks=np.concatenate([clocks,*axis_clocks]),phase=phase,positions=np.array(positions))


def changes(previous,current,old_objective,objective):
    from scipy.spatial.transform import Rotation
    rotations=current['mount']@previous['mount'].transpose(0,2,1)
    return dict(scale_relative=float(np.max(abs(current['scale']/previous['scale']-1))),
        gsens_dps_per_g=float(np.max(abs(current['gsens']-previous['gsens']))),
        mount_deg=float(np.rad2deg(Rotation.from_matrix(rotations).magnitude()).max()),
        clock_offset_s=float(np.max(abs(current['clocks'][:,0]-previous['clocks'][:,0]))),
        clock_ppm=float(np.max(abs(current['clocks'][:,1]-previous['clocks'][:,1]))),
        earth_phase_rad=float(abs(np.angle(np.exp(1j*(current['phase']-previous['phase']))))),
        lever_arm_m=float(np.max(np.linalg.norm(current['positions']-previous['positions'],axis=1))),
        objective_relative=float(abs(objective-old_objective)/max(abs(old_objective),abs(objective),1e-6)))


class Convergence:
    def __init__(self,maximum=30,minimum=4,stable_required=3):
        if not 1<=minimum<=maximum<=100:raise ValueError('Invalid iteration bounds')
        self.maximum=maximum;self.minimum=minimum;self.stable_required=stable_required
        self.previous=None;self.objective=None;self.stable=0;self.history=[];self.converged=False

    def update(self,state,objective,inner_ok=True):
        if not np.isfinite(objective) or any(not np.isfinite(v).all() for v in state.values()):
            raise ValueError('Non-finite solver state; diagnostic only')
        delta=changes(self.previous,state,self.objective,objective) if self.previous is not None else None
        stable=bool(inner_ok and delta is not None and all(delta[k]<=v for k,v in LIMITS.items()))
        self.stable=self.stable+1 if stable else 0
        self.history.append(dict(iteration=len(self.history)+1,objective=objective,changes=delta,
                                 inner_solvers_converged=bool(inner_ok),stable=stable))
        self.converged=len(self.history)>=self.minimum and self.stable>=self.stable_required
        self.previous=state;self.objective=objective
        return self.converged

    def report(self):
        return dict(converged=self.converged,reason='stable_parameters_and_objective' if self.converged else 'iteration_limit',
                    iterations=len(self.history),maximum_iterations=self.maximum,minimum_iterations=self.minimum,
                    consecutive_stable_required=self.stable_required,thresholds=LIMITS,history=self.history)


def clock_motion(j,data,st,clocks,ranges=None,step=.02):
    obs,segments,poses,up=j.motion(data,st,clocks,ranges=ranges,step=step)
    history=[]
    for _ in range(8):
        updated,fit_ok=j.refine_clocks(data,st,clocks,segments,return_status=True)
        delta=abs(updated-clocks)
        history.append(dict(offset_s=float(delta[:,0].max()),ppm=float(delta[:,1].max()),fit_converged=fit_ok))
        clocks=updated
        obs,segments,poses,up=j.motion(data,st,clocks,ranges=ranges,step=step)
        if history[-1]['offset_s']<1e-5 and history[-1]['ppm']<.05:break
    converged=bool(fit_ok and all(s['curve_solve']['converged'] for s in segments)
                   and history[-1]['offset_s']<1e-5 and history[-1]['ppm']<.05
                   and np.max(abs(clocks[:,0]))<.0149 and np.max(abs(clocks[:,1]))<49.9)
    return obs,segments,poses,up,clocks,dict(converged=converged,history=history)


def all_recordings(j,data,st,obs,segments,axis_clocks):
    combined=[{k:np.array(v,copy=True) for k,v in o.items()} for o in obs]
    all_segments=list(segments);checks=[]
    if not data[0].get('scale_captures'):return combined,all_segments,axis_clocks,checks
    for axis in range(3):
        views=[];rests=[]
        ranges=data[0]['scale_captures'][axis]['rests']
        for d in data:
            cap=d['scale_captures'][axis];t=cap['time']
            view=dict(d,**cap)
            view['interval_duration']=cap['duration']*cap['clock_rate']
            views.append(view)
            gy=cap['gyro']-j.old.table(d,cap['temp'])
            means=lambda values:np.array([np.mean(values[(t>=a)&(t<=b)],axis=0) for a,b in ranges])
            rests.append((means(gy),means(cap['accel']),np.mean(ranges,axis=1),None))
        moves=list(zip(np.array(ranges)[:-1,1],np.array(ranges)[1:,0]))
        # Long turns contain rapid speed changes too. A 100 ms latent grid
        # cannot explain the 20 ms observations and feeds its bandwidth error
        # back into calibration. Use the same window-scale grid as pose moves.
        extra,segs,_,_,axis_clocks[axis],check=clock_motion(j,views,rests,axis_clocks[axis],ranges=moves,step=j.DT)
        check['axis']='xyz'[axis];checks.append(check)
        offsets=[len(o['start']) for o in combined]
        for s in segs:
            s['index']=len(all_segments)
            s['indexes']=[ix+offset for ix,offset in zip(s['indexes'],offsets)]
            s['source']='xyz'[axis]
            all_segments.append(s)
        for old,new in zip(combined,extra):
            for key in old:old[key]=np.concatenate((old[key],new[key]))
    return combined,all_segments,axis_clocks,checks


def window_statistics(j,data,st,obs,segments,rigid):
    """Feed individual rigid accelerations and all angular windows into S/M fits."""
    position=np.asarray(rigid['relative_positions_m']);bias=np.asarray(rigid['accelerometer_residual_bias_g'])
    centers=[d['Q'].apply(s[1][j.IDS].mean(axis=0)) for d,s in zip(data,st)]
    pieces=[[] for _ in data];diagnostics=[];curve_checks=[]
    for seg in segments:
        if seg['index'] in j.HELD_MOVES:continue
        terms=j.kinematics(seg)
        acc=[o['accel'][ix]-center for o,ix,center in zip(obs,seg['indexes'],centers)]
        corrected=[a-B@position[i]-bias[i] for i,(a,B) in enumerate(zip(acc,terms))]
        matrix=vstack([*seg['operators'],.15*seg['smooth']]).tocsr()
        target=np.vstack([*corrected,np.zeros((seg['smooth'].shape[0],3))])
        force,check=solve_curve(matrix,target);curve_checks.append(dict(motion=seg['index'],**check))
        for i,(d,o,A,ix,B) in enumerate(zip(data,obs,seg['operators'],seg['indexes'],terms)):
            modeled=A@force+B@position[i]+bias[i]+centers[i]
            error=np.linalg.norm(modeled-o['accel'][ix],axis=1)
            # Rigid denoising is trusted only where it agrees with measured
            # acceleration. A bad lever model cannot overwrite observations.
            trust=1/(1+(error/.003)**2)
            acc_body=trust[:,None]*modeled+(1-trust[:,None])*o['accel'][ix]
            ad=d['Q'].inv().apply(acc_body)-o['accel_base'][ix]
            factor=o['clock_factor'][ix]
            gy=o['gyro_delta'][ix]/factor[:,None];ad=ad/factor[:,None]
            angular=A@seg['rate']
            residual=gy-ad@d['M'].T-d['Q'].inv().apply(angular)/d['S']
            spread=1.4826*np.median(abs(residual-np.median(residual,axis=0)))
            sigma=max(.02,float(spread))
            huber=np.minimum(1,3*sigma/np.maximum(np.linalg.norm(residual,axis=1),1e-12))
            weights=(o['end'][ix]-o['start'][ix])/.2/sigma**2*huber
            pieces[i].append(compress_rows(np.c_[gy,ad],angular,weights))
            diagnostics.append(dict(motion=seg['index'],source=seg.get('source','poses'),slot=i,
                windows=len(ix),gyro_noise_scale_dps=sigma,rigid_blend_mean=float(trust.mean()),
                downweighted_fraction=float(np.mean(huber<1)),accel_residual_rms_g=float(np.sqrt(np.mean(error**2)))))
    rows=[np.linalg.qr(np.vstack(p),mode='r') for p in pieces]
    return rows,dict(blocks=diagnostics,method='weighted QR in measured sensor units; 0.2 s effective independence',
                     curve_solvers=curve_checks,inner_converged=all(c['converged'] for c in curve_checks),
                     training_motions=[s['index'] for s in segments if s['index'] not in j.HELD_MOVES])

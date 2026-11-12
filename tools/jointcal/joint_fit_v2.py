"""Joint F04 diagnostic with true interval operators and independent accelerations.
Never installs parameters. Shared angular motion only; no common-point accel.
"""
import json,struct,hashlib,sys
from pathlib import Path
import numpy as np
from scipy.sparse import coo_matrix,vstack,diags
from scipy.sparse.linalg import lsmr
from scipy.spatial.transform import Rotation as R

from inputs import configure, load
SEG=[]
MOV=[]
LATITUDE_DEG=45.0

def interval_operator(start,end,knots):
    """Exact integral of piecewise linear angular rate divided by observed dt."""
    h=knots[1]-knots[0];left=knots[0];rows=[];cols=[];values=[]
    start=np.asarray(start);end=np.asarray(end)
    assert np.all(end>start) and np.all(start>=left-1e-7) and np.all(end<=knots[-1]+1e-7)
    k0=np.maximum(0,np.floor((start-left)/h).astype(int))
    k1=np.minimum(len(knots)-2,np.floor((end-left)/h).astype(int))
    for offset in range(int(np.max(k1-k0))+1):
        row=np.flatnonzero(k0+offset<=k1);k=k0[row]+offset
        l=np.maximum(start[row],knots[k]);r=np.minimum(end[row],knots[k+1]);keep=r>l
        row=row[keep];k=k[keep];l=l[keep];r=r[keep]
        right=((r-knots[k])**2-(l-knots[k])**2)/(2*h);duration=end[row]-start[row]
        rows.extend((row,row));cols.extend((k,k+1));values.extend(((r-l-right)/duration,right/duration))
    A=coo_matrix((np.concatenate(values),(np.concatenate(rows),np.concatenate(cols))),shape=(len(start),len(knots))).tocsr()
    assert np.max(abs(np.asarray(A.sum(axis=1)).ravel()-1))<1e-7
    return A

def table(d,temp):
    a=d['lut'];return np.stack([np.interp(temp,a[:,1],a[:,j]) for j in (2,3,4)],axis=-1)

def means(d,part):
    y=[];a=[];t=[];weights=[]
    for lo,hi in SEG:
        middle=(lo+hi)/2
        m=(d['time']>=lo+2)&(d['time']<middle-1) if part=='train' else (d['time']>=middle+1)&(d['time']<hi-2)
        assert m.sum()>100
        w=d['duration'][m]
        y.append(np.average(d['gyro'][m]-table(d,d['temp'][m]),axis=0,weights=w))
        a.append(np.average(d['accel'][m],axis=0,weights=w));t.append(np.average(d['time'][m],weights=w));weights.append(w.sum())
    return np.array(y),np.array(a),np.array(t),weights


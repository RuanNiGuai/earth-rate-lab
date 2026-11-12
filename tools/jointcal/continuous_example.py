"""Four synthetic F04 captures with known ten-position rigid-body physics.

Explicitly marked synthetic; installation is forbidden by the plan builder.
"""
import argparse
import json
from pathlib import Path
import struct
import zlib
import numpy as np
from scipy.spatial.transform import Rotation as R
from example import snapshot
from inputs import digest


def trajectory(kind,final_return=False,pose_rest_seconds=30):
    count=11 if final_return else 10
    rests=[(i*(pose_rest_seconds+12.),i*(pose_rest_seconds+12.)+pose_rest_seconds) for i in range(count)] if kind=='poses' else [(0,30),(90,120),(180,210)]
    t=np.arange(.01,rests[-1][1],.02);q=np.zeros((len(t),4));w=np.zeros((len(t),3));alpha=w.copy()
    if kind=='poses':
        poses=[R.from_rotvec([x,0,0]) for x in np.deg2rad([0,90,180,270,360])]
        poses += [R.from_euler('z',180,degrees=True),R.from_euler('z',-90,degrees=True),R.from_euler('z',90,degrees=True),R.from_euler('y',90,degrees=True),R.from_euler('y',-90,degrees=True)]
        if final_return:poses.append(R.identity())
    else:
        axis='xyz'.index(kind);up=np.eye(3)[axis]
        initial=R.align_vectors(np.array([[1.,0,0]]),up[None])[0]
        poses=[initial]*3
    for k,(a,b) in enumerate(rests):
        mask=(t>=a)&(t<b);q[mask]=poses[k].as_quat()
        if k==len(rests)-1:continue
        mask=(t>=b)&(t<rests[k+1][0]);span=rests[k+1][0]-b;u=(t[mask]-b)/span
        if kind=='poses':
            delta=(poses[k].inv()*poses[k+1]).as_rotvec()
            s=10*u**3-15*u**4+6*u**5
            ds=(30*u**2-60*u**3+30*u**4)/span
            dds=(60*u-180*u**2+120*u**3)/span**2
        else:
            delta=np.eye(3)[axis]*np.deg2rad(7200)*(1 if k==0 else -1)
            s=u-np.sin(2*np.pi*u)/(2*np.pi);ds=(1-np.cos(2*np.pi*u))/span;dds=2*np.pi*np.sin(2*np.pi*u)/span**2
        q[mask]=(poses[k]*R.from_rotvec(s[:,None]*delta)).as_quat()
        w[mask]=ds[:,None]*delta;alpha[mask]=dds[:,None]*delta
    return t,R.from_quat(q),w,alpha


def write_reports(path,gyro,accel):
    n=gyro.shape[1]
    header=dict(format='F04HID1',dataset_kind='synthetic',receiver_serial='SYNTHETIC',targets={str(i):f'{i+1:012X}' for i in range(10)})
    with path.open('x',encoding='utf8') as out:
        out.write(json.dumps(header)+'\n')
        for seq in range(n):
            for slot in range(10):
                raw=bytearray(64);raw[:4]=bytes([0x15,slot,1,1])
                struct.pack_into('<4I',raw,4,1,1,seq,(seq+1)*20000)
                struct.pack_into('<3i3hhBBH',raw,20,*np.rint(gyro[slot,seq]*20000).astype(int),*np.rint(accel[slot,seq]*1000).astype(int),2510,8,4,20000)
                struct.pack_into('<I',raw,44,zlib.crc32(raw[:44]))
                struct.pack_into('>I',raw,49,seq*655)
                out.write(json.dumps(dict(report_hex=raw.hex(),host_monotonic_ns=10**9+(seq+1)*20_000_000+slot*1_000_000))+'\n')
        for slot in range(10):
            raw=bytearray(64);raw[:4]=bytes([0x15,slot,1,2])
            struct.pack_into('<10I',raw,4,1,1,n,n,0,0,n,0,0,0)
            struct.pack_into('<I',raw,44,zlib.crc32(raw[:44]))
            out.write(json.dumps(dict(report_hex=raw.hex(),host_monotonic_ns=10**9+(n+1)*20_000_000))+'\n')
        out.write(json.dumps(dict(event='capture-stop',confirmed=True,receiver_dropped=0))+'\n')


def generate(out,final_return=False):
    out=Path(out);out.mkdir(exist_ok=False,parents=True)
    rng=np.random.default_rng(20261112)
    scales=rng.uniform(.995,1.008,(10,3));matrices=rng.normal(0,.001,(10,3,3))
    positions=rng.uniform(-.08,.08,(10,3));positions[0]=0
    bias=rng.normal(0,.01,(10,3));earth=.0041780746*np.array([np.sin(np.pi/4),np.cos(np.pi/4)*np.cos(.7),np.cos(np.pi/4)*np.sin(.7)])
    recordings={}
    for kind in ('x','y','z','poses'):
        t,q,w,alpha=trajectory(kind,final_return=final_return);gyros=[];accels=[]
        gravity=q.inv().apply(np.tile([1.,0,0],(len(t),1)))
        for i in range(10):
            accel=gravity+(np.cross(alpha,positions[i])+np.cross(w,np.cross(w,positions[i])))/9.80665
            gyro=(np.rad2deg(w)+q.inv().apply(np.tile(earth,(len(t),1))))/scales[i]+accel@matrices[i].T+bias[i]
            gyros.append(gyro);accels.append(accel)
        path=out/(kind+'-reports.jsonl');write_reports(path,np.asarray(gyros),np.asarray(accels));recordings[kind]=str(path)
    for i in range(10):
        dest=out/'snapshots'/f'slot-{i}';dest.mkdir(parents=True)
        blob=snapshot(scales[i]);p=dest/'snapshot.bin';p.write_bytes(blob)
        (dest/'backup.json').write_text(json.dumps(dict(slot=i,rf_address=f'{i+1:012X}',snapshot_sha256=digest(p))),encoding='utf8')
    (out/'truth.json').write_text(json.dumps(dict(scales=scales.tolist(),matrices=matrices.tolist(),positions=positions.tolist()),indent=2),encoding='utf8')
    (out/'recordings.json').write_text(json.dumps(recordings,indent=2),encoding='utf8')
    return recordings


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--final-return',action='store_true')
    args=p.parse_args()
    print(json.dumps(generate(args.out,final_return=args.final_return)))

"""Generate a reproducible synthetic ten-position rigid fixture; never device data."""
import argparse
import json
from pathlib import Path
import struct
import zlib
import numpy as np
from scipy.spatial.transform import Rotation as R
from inputs import digest


def snapshot(scale):
    def tlv(k, v):
        return struct.pack('<HH', k, len(v)) + v
    def crc(raw):
        return bytes(raw[:-4]) + struct.pack('<I', zlib.crc32(raw[:-4]))
    profile = struct.pack('<15f4B', 1, 1, 1, *([0]*9), .0041780746, 0, 0, 1, 0, 0, 1)
    thermal = struct.pack('<5H2B', 140, 140, 113, 140, 1, 1, 0)
    thermal += b''.join(struct.pack('<4f', 15.1+.2*k, 0, 0, 0) for k in range(140))
    thermal += b''.join(struct.pack('<4f', 15+.25*k, 0, 0, 0) for k in range(113))
    meta = bytearray(32)
    struct.pack_into('<4I', meta, 0, 0x41524334, 4, 1, 1)
    result = tlv(1, b'SYNTHETIC_NOT_FOR_INSTALL\0') + tlv(2, profile) + tlv(3, thermal) + tlv(100, crc(meta))
    for axis in range(3):
        raw = bytearray(584)
        struct.pack_into('<5I2B', raw, 0, 0x41353252, 2, 1, 1, 1, 1, axis)
        v = np.zeros(22)
        v[:3] = .01
        v[3+axis] = 1
        v[6] = 25.1
        v[7:10] = .6
        v[7+axis] += 7200/scale[axis]
        v[10+axis] = 60
        v[13] = 25.1*60
        v[14] = 60
        v[17:19] = 25.1
        v[19] = .1*60
        struct.pack_into('<22f4B', raw, 488, *v, axis, 20, 0, 0)
        result += tlv(104+2*axis, crc(raw))
    return result


def generate(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    rng = np.random.default_rng(20261111)
    static = [[i*16., i*16.+12.] for i in range(10)]
    poses = [R.from_rotvec([a, 0, 0]) for a in np.deg2rad([0, 90, 180, 90, 0])]
    poses += [R.from_euler('z', 180, degrees=True), R.from_euler('z', -90, degrees=True),
              R.from_euler('z', 90, degrees=True), R.from_euler('y', 90, degrees=True), R.from_euler('y', -90, degrees=True)]
    times = np.arange(.01, static[-1][1], .02)
    q = np.zeros((len(times), 4));omega = np.zeros((len(times), 3));alpha = omega.copy()
    for i, (a, b) in enumerate(static):
        m = (times >= a) & (times < b)
        q[m] = poses[i].as_quat()
        if i == 9:
            continue
        m = (times >= b) & (times < static[i+1][0])
        u = (times[m]-b)/4
        s = 10*u**3-15*u**4+6*u**5
        delta = (poses[i].inv()*poses[i+1]).as_rotvec()
        q[m] = (poses[i]*R.from_rotvec(s[:,None]*delta)).as_quat()
        omega[m] = ((30*u**2-60*u**3+30*u**4)/4)[:,None]*delta
        alpha[m] = ((60*u-180*u**2+120*u**3)/16)[:,None]*delta
    rotations = R.from_quat(q)
    up = np.array([1.,0,0])
    earth = .0041780746*np.array([np.sin(np.pi/4),np.cos(np.pi/4)*np.cos(.7),np.cos(np.pi/4)*np.sin(.7)])
    source = out/'synthetic-source.json'
    source.write_text(json.dumps(dict(kind='synthetic', seed=20261111,
        note='Noise-free midpoint windows; mathematical regression only, not physical accuracy evidence.')), encoding='utf8')
    config = dict(schema='JOINT10_INPUT1',dataset_kind='synthetic',latitude_deg=45,
                  static_segments_s=static,recording=source.name,recording_sha256=digest(source),devices=[])
    truth = []
    for i in range(10):
        # Aligned mounts simplify the example; the physics tests cover differing mounts.
        scale = rng.uniform(.995, 1.008, 3)
        pos = np.zeros(3) if i==0 else rng.uniform(-.08,.08,3)
        matrix = rng.normal(0,.001,(3,3))
        acc = rotations.inv().apply(np.tile(up,(len(times),1)))
        acc += (np.cross(alpha,pos)+np.cross(omega,np.cross(omega,pos)))/9.80665
        gyro = (np.rad2deg(omega)+rotations.inv().apply(np.tile(earth,(len(times),1))))/scale
        gyro += acc@matrix.T + rng.normal(0,.01,3) + times[:,None]/100*rng.normal(0,.0001,3)
        arrays = out/f'tracker-{i}.npz'
        np.savez_compressed(arrays,time=times,interval_end=times+.01,duration=np.full(len(times),.02),
                            gyro=gyro,accel=acc,temp=np.full(len(times),25.1),seq=np.arange(len(times)))
        snap = out/f'snapshot-{i}.bin';snap.write_bytes(snapshot(scale))
        config['devices'].append(dict(slot=i,arrays=arrays.name,arrays_sha256=digest(arrays),
            snapshot=snap.name,snapshot_sha256=digest(snap),clock_rate_correction_ppm=0))
        truth.append(dict(slot=i,scale=scale.tolist(),position_m=pos.tolist(),gsens=matrix.tolist()))
    (out/'input.json').write_text(json.dumps(config,indent=2),encoding='utf8')
    (out/'truth.json').write_text(json.dumps(truth,indent=2),encoding='utf8')
    return out/'input.json'


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    print(generate(p.parse_args().out))

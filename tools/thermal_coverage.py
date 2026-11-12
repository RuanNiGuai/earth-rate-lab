"""Audit real 0.2 C buckets from CALWIRE snapshots, including interior gaps."""
import argparse,hashlib,json,math
from pathlib import Path
from calwire import unpack_snapshot,require

def runs(indices):
    out=[]
    for i in indices:
        if out and i==out[-1][1]+1:out[-1][1]=i
        else:out.append([i,i])
    return [[round(15+.2*a,1),round(15+.2*(b+1),1)] for a,b in out]

def audit(root, slots=range(10)):
    devices=[];sets=[]
    for i in slots:
        path=root/f'slot-{i}/snapshot.bin';raw=path.read_bytes();saved=json.loads(path.with_name('backup.json').read_text())
        require(hashlib.sha256(raw).hexdigest()==saved['snapshot_sha256'],'Snapshot hash')
        d=unpack_snapshot(raw,require_scales=False,allow_empty_thermal=True);require(len(d['thermal_points'])==140,'Expected 15..43 C / 0.2 C configuration')
        present=[]
        for index,row in enumerate(d['thermal_points']):
            t=row[0]
            if t==0:continue
            require(all(math.isfinite(x) for x in row),'Nonfinite point')
            require(15+.2*index-1e-4<=t<15+.2*(index+1)+1e-4,'Temperature in wrong bucket')
            present.append(index)
        missing=sorted(set(range(140))-set(present));sets.append(set(missing))
        devices.append(dict(slot=i,rf_address=saved['rf_address'],points=len(present),missing_points=len(missing),
            collected_temperature_centers_c=[round(min(d['thermal_points'][k][0] for k in present),2),round(max(d['thermal_points'][k][0] for k in present),2)] if present else None,
            missing_bucket_intervals_c=runs(missing),missing_bucket_indices=missing,
            snapshot_sha256=saved['snapshot_sha256']))
    return dict(target=dict(min_c=15,max_exclusive_c=43,step_c=.2,buckets=140),devices=devices,
        any_device_missing_intervals_c=runs(sorted(set.union(*sets))),
        all_devices_missing_intervals_c=runs(sorted(set.intersection(*sets))),
        complete=all(not x for x in sets),
        note='Range presence is necessary, not a noise/hysteresis/accuracy certificate. Intervals use inclusive lower, exclusive upper edges.')

def main():
    p=argparse.ArgumentParser();p.add_argument('--snapshots',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    r=audit(a.snapshots);a.out.parent.mkdir(parents=True,exist_ok=True)
    require(not a.out.exists(),'Existing audit preserved');a.out.write_text(json.dumps(r,indent=2),encoding='utf-8')
    for d in r['devices']:print(json.dumps({k:v for k,v in d.items() if k not in ('rf_address','snapshot_sha256','missing_bucket_indices')}))
    print(json.dumps({k:v for k,v in r.items() if k!='devices'}))

if __name__=='__main__':main()

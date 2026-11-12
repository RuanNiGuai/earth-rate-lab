"""Validate read-only device exports for fleet fitting; no parameter setters.

Strict ARC8P1 SHA/CRC/layout parsing and actual variable-size runtime LUT.
The snapshot format is separate from a full NVS backup or installation proof.
"""
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import time
import zlib


def require(ok, text):
    if not ok:
        raise ValueError(text)


def scale_records(text):
    begin=re.findall(r'ARC8P1 BEGIN scale (\d+) (\d+) (\d+) ([0-9a-f]{64})',text)
    end=re.findall(r'ARC8P1 END scale (\d+) ([0-9a-f]{64})',text)
    require(len(begin)==len(end)==1,'Incomplete scale snapshot')
    count,capacity,reference,digest=begin[0]
    require((int(count),int(capacity))==(3,3) and int(reference)>0,'Need all three scale axes')
    require(end[0]==('1752',digest),'Scale end/digest mismatch')
    buffers={i:bytearray() for i in range(3)}
    for axis,offset,data in re.findall(r'ARC8P1 DATA scale (\d+) (\d+) ([0-9a-f]+)',text):
        axis,offset=int(axis),int(offset)
        require(axis in buffers and len(buffers[axis])==offset,'Scale gaps/duplicates/order')
        buffers[axis].extend(bytes.fromhex(data))
    raw=[bytes(buffers[i]) for i in range(3)]
    require(all(len(x)==584 for x in raw),'Scale size')
    require(hashlib.sha256(b''.join(raw)).hexdigest()==digest,'Scale SHA mismatch')
    for axis,record in enumerate(raw):
        magic,version,generation,ref,epoch=struct.unpack_from('<5I',record)
        require(magic==0x41353252 and version==2 and generation>0 and ref==int(reference) and epoch>0,'Scale record identity')
        require(record[20]==1 and record[21]==axis,'Scale kind/index')
        require(zlib.crc32(record[:580])==struct.unpack_from('<I',record,580)[0],'Scale CRC')
        values=struct.unpack_from('<22f4B',record,488)
        require(all(math.isfinite(v) for v in values[:22]),'Nonfinite scale record')
        require(values[22]==axis and values[23]==20,'Scale axis/turn count')
        require(10<=values[14]<=600 and values[15]<=1100 and values[16]<=.16,'Scale quality')
    return raw


def runtime_lut(text):
    headers=re.findall(r'TCAL_LUT valid=(\d+) count=(\d+) version=(\d+) points=(\d+)',text)
    require(len(headers)==1 and 'TCAL_LUT END' in text,'Incomplete LUT header/end')
    valid,count,version,points=map(int,headers[0])
    require(valid==1 and count>1 and points>0 and version==points,'Invalid/stale runtime LUT')
    rows=[tuple(map(float,row)) for row in re.findall(r'^LUT (\d+) ([\d.eE+-]+) ([\d.eE+-]+) ([\d.eE+-]+) ([\d.eE+-]+)',text,re.M)]
    require(len(rows)==count and [x[0] for x in rows]==list(range(count)),'LUT rows/order')
    require(all(math.isfinite(v) for row in rows for v in row),'Nonfinite LUT')
    require(all(a[1]<b[1] for a,b in zip(rows,rows[1:])),'LUT temperature order')
    return rows


def calibration_values(data):
    scale=re.search(r'Gyro scale: X ([\d.-]+) Y ([\d.-]+) Z ([\d.-]+);',data['gyro scale status'])
    earth=re.search(r'Reference body dps: ([\d.-]+) ([\d.-]+) ([\d.-]+)',data['earth cal status'])
    gsens=re.search(r'g-sens: (.+?) \(0 = none\)',data['gyro gsens status'])
    require(scale and earth and gsens,'Missing production calibration readback')
    matrix=list(map(float,re.findall(r'-?\d+\.\d+',gsens[1])))
    require(len(matrix)==9,'Incomplete g-sens matrix')
    return dict(scale=list(map(float,scale.groups())),earth_dps=list(map(float,earth.groups())),gsens_dps_per_g=matrix)


def validate_snapshot(data):
    scales=scale_records(data['cal archive export partial scale'])
    lut=runtime_lut(data['tcal lut'])
    rows=re.findall(r'^\s*(\d+)\s+\|\s+([\d.-]+)\s+\|\s+([\d.-]+)\s+\|\s+([\d.-]+)\s+\|\s+([\d.-]+)',data['tcal dump'],re.M)
    count=re.findall(r'Total points printed: (\d+)',data['tcal dump'])
    require(len(count)==1 and len(rows)==int(count[0]),'Incomplete temperature backup')
    return dict(production=calibration_values(data),lut=lut,temperature_rows=rows,
                scale_sha256=[hashlib.sha256(x).hexdigest() for x in scales])


def restore_auto(console, original, data):
    """Restore and verify state before releasing USB or reporting success."""
    if original is None:
        return
    if original=='enabled':
        data['restore auto reply']=console.send('tcal auto on',1)
    reply=console.send('tcal status',1.8)
    data['tcal status after']=reply
    auto=re.findall(r'^Auto-calibration: (enabled|disabled)',reply,re.M)
    require(auto==[original],'Auto-calibration restore/readback not verified')
    data['auto_restore_verified']=True


def publish_snapshot(temp, destination, data):
    """Caller must close console handles first (Windows directory rename)."""
    import batch_calibration as b
    require(data.get('auto_restore_verified') is True,'State restoration unverified')
    require(not destination.exists(),'Existing destination not overwritten')
    data['checked']=validate_snapshot(data)
    b.save_new(temp/'backup.json',data)
    for i,raw in enumerate(scale_records(data['cal archive export partial scale'])):
        (temp/f'scale-{i}.bin').write_bytes(raw)
    temp.rename(destination)


def watch(session,output):
    import serial
    from serial.tools import list_ports
    from cal_archive import Console,parse_status
    import batch_calibration as b
    state=json.loads((session/'session.json').read_text(encoding='utf-8'))
    targets=b.validate_targets(state['targets'])
    output.mkdir(parents=True,exist_ok=True)
    done=set()
    for p in output.glob('slot-*'):
        if not (p/'backup.json').exists():continue
        saved=json.loads((p/'backup.json').read_text(encoding='utf-8'))
        slot=int(p.name.split('-')[1])
        require(saved.get('rf_address')==targets[slot],'Saved identity mismatch')
        validate_snapshot(saved)
        done.add(slot)
    b.atomic_save(output/'progress.json',dict(done=sorted(done),remaining=sorted(set(targets)-done),utc=b.utc()))
    failed=set()
    print(json.dumps(dict(waiting_usb=True,already_saved=sorted(done))),flush=True)
    while len(done)<len(targets) and not (output/'STOP').exists():
        ports=[p for p in list_ports.comports() if (p.vid,p.pid)==(0x1209,0x7692) and p.serial_number]
        present={p.serial_number for p in ports}
        failed.intersection_update(present)
        for port in ports:
            if port.serial_number in failed:continue
            temp=output/('read-'+str(time.time_ns()))
            temp.mkdir()
            c=None
            data={};original_auto=None
            try:
                c=Console(port.device,log=temp/'console.log')
                info=c.send('info',1.6);data['info']=info
                address=re.findall(r'^Device address: ([A-F0-9]{12})\s*$',info,re.M)
                require(len(address)==1 and address[0] in targets.values(),'USB identity outside frozen fleet')
                slot=next(k for k,v in targets.items() if v==address[0])
                if slot in done:
                    failed.add(port.serial_number)
                    continue
                require('Commit 06ce018fa6d1_wom3_t02_on,' in info,'Unsupported installed build')
                data['tcal status before']=c.send('tcal status',1.8)
                auto=re.findall(r'^Auto-calibration: (enabled|disabled)',data['tcal status before'],re.M)
                require(len(auto)==1,'Auto-calibration state unknown')
                original_auto=auto[0]
                # OFF is also the explicit persistence boundary for automatic
                # EMA changes left in RAM, even when collection already stopped.
                data['pause']=c.send('tcal auto off',1.8)
                require('T-Cal automatic observations saved and read back' in data['pause'],'TCal persistence not verified')
                require('Auto-calibration: disabled' in c.send('tcal status',1.8),'TCal pause not verified')
                st=c.status();data['archive before']=st
                if st['capture']!='none' or st['step']!=0 or st['pending'] or st['collecting']:
                    data['cancel']=c.send('cal archive capture cancel',2)
                    st=c.status()
                    require(st['step']==0 and st['capture']=='none' and not st['pending'] and not st['collecting'],'Archive not idle')
                if not st['enabled']:
                    data['on']=c.send('cal archive on',2)
                for cmd in ('gyro scale status','gyro gsens status','earth cal status','tcal status','tcal lut','tcal dump',
                            'cal archive export partial scale','cal archive export tcal'):
                    data[cmd]=c.send(cmd,2.2)
                validate_snapshot(data)
                end_info=c.send('info',1.6)
                require(f'Device address: {address[0]}' in end_info,'USB changed during snapshot')
                data['serial']=port.serial_number;data['rf_address']=address[0];data['slot']=slot
                data['original_auto']=original_auto
                destination=output/f'slot-{slot}'
                restore_auto(c,original_auto,data)
                c.close();c=None
                publish_snapshot(temp,destination,data)
                done.add(slot)
                b.atomic_save(output/'progress.json',dict(done=sorted(done),remaining=sorted(set(targets)-done),utc=b.utc()))
                print(json.dumps(dict(saved=slot,done=sorted(done),remaining=sorted(set(targets)-done))),flush=True)
            except Exception as e:
                failed.add(port.serial_number)
                (temp/'failure.json').write_text(json.dumps(dict(error=str(e),data=data),ensure_ascii=False,indent=2),encoding='utf-8')
                print(json.dumps(dict(failed_port=port.device,error=str(e))),flush=True)
            finally:
                if c is not None:
                    if original_auto is not None and not data.get('auto_restore_verified'):
                        try:
                            restore_auto(c,original_auto,data)
                        except Exception as e:
                            (temp/'restore-failure.json').write_text(json.dumps(dict(error=str(e),original_auto=original_auto),indent=2),encoding='utf-8')
                            print(json.dumps(dict(state_restore_failed=port.device,error=str(e))),flush=True)
                    c.close()
        time.sleep(.8)
    print(json.dumps(dict(snapshot_complete=len(done)==len(targets),done=sorted(done))),flush=True)


if __name__=='__main__':
    import argparse
    import batch_calibration as b
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    with b.lock(args.session/'LOCK'):
        watch(args.session,args.out)

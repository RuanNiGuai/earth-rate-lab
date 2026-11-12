"""CALWIRE v1: exact-address wireless snapshot and bounded calibration writes.

Snapshots are binary evidence, not USB transcripts or a complete flash image.
No solve or implicit write occurs on export. Applying coefficients requires a
separate validated solution and caller-owned journal/rollback workflow.
"""
import argparse
from contextlib import contextmanager
import hashlib
import json
import math
from pathlib import Path
import re
import secrets
import struct
import time
import zlib


def require(ok, message):
    if not ok: raise ValueError(message)


def packet(slot,address,token,op,offset=0,data=b''):
    require(0<=slot<32 and 0<token<2**32 and 0<=op<=5 and 0<=offset<65536 and len(data)<=26,'Request bounds')
    require(re.fullmatch('[0-9A-Fa-f]{12}',address) is not None,'RF address')
    p=bytearray(48);p[:4]=bytes([0x26,slot,1,op])
    struct.pack_into('<I',p,4,token);p[8:14]=bytes.fromhex(address)[::-1]
    struct.pack_into('<H',p,14,offset);p[16]=len(data);p[18:18+len(data)]=data
    struct.pack_into('<I',p,44,zlib.crc32(p[:44]));return bytes(p)


def response(raw,request):
    require(len(raw)==48 and raw[0]==0x27 and raw[1:16]==request[1:16],'Response identity/transaction')
    require(raw[16]<=26 and not any(raw[18+raw[16]:44]),'Response length/padding')
    require(struct.unpack_from('<I',raw,44)[0]==zlib.crc32(raw[:44]),'Response CRC')
    require(raw[16]>=4 and raw[18:22]==request[44:48],'Response request digest')
    require(raw[17]==0,f'Tracker rejected CALWIRE operation: status {raw[17]}')
    return raw[22:18+raw[16]]


def unpack_snapshot(blob, require_scales=True, allow_empty_thermal=False):
    objects={};position=0
    while position<len(blob):
        require(position+4<=len(blob),'Truncated object header')
        kind,size=struct.unpack_from('<HH',blob,position);position+=4
        require(kind not in objects and size>0 and position+size<=len(blob),'Duplicate/truncated object')
        objects[kind]=blob[position:position+size];position+=size
    require(all(k in objects for k in (1,2,3)),'Missing snapshot sections')
    require(objects[1].endswith(b'\0'),'Build termination')
    build=objects[1][:-1].decode('ascii')
    p=objects[2];require(len(p)==64 and p[60]<=1 and p[61]<=5 and p[62]==0 and p[63]<=1,'Profile flags')
    values=struct.unpack_from('<15f',p)
    require(all(math.isfinite(v) for v in values),'Nonfinite profile')
    thermal=objects[3];require(len(thermal)>=12,'Thermal header')
    count,capacity,entries,version,sig,enabled,_=struct.unpack_from('<5H2B',thermal)
    require(0<=count<=capacity<=256 and 0<=entries<=256 and version==count and enabled==p[63], 'Thermal counts/flags')
    require(allow_empty_thermal or (count>1 and entries>1), 'Thermal table not ready')
    require(len(thermal)==12+capacity*16+entries*16,'Thermal byte count')
    points=[list(struct.unpack_from('<4f',thermal,12+16*i)) for i in range(capacity)]
    rows=[list(struct.unpack_from('<4f',thermal,12+capacity*16+16*i)) for i in range(entries)]
    require(all(math.isfinite(v) for row in points+rows for v in row),'Nonfinite thermal data')
    require(sum(x[0]!=0 for x in points)==count,'Thermal point count')
    require(all(a[0]<b[0] for a,b in zip(rows,rows[1:])),'LUT ordering')
    def valid_record(raw,magic,version,length):
        return len(raw)==length and struct.unpack_from('<2I',raw)==(magic,version) and zlib.crc32(raw[:-4])==struct.unpack_from('<I',raw,len(raw)-4)[0]
    metas=[objects[k] for k in (100,101) if k in objects and valid_record(objects[k],0x41524334,4,32)]
    require(metas or not require_scales,'No valid archive metadata mirror')
    ref=struct.unpack_from('<I',max(metas,key=lambda x:struct.unpack_from('<I',x,8)[0]),12)[0] if metas else 0
    require(ref>0 or not require_scales,'Zero archive reference')
    scales=[]
    for axis in range(3):
        candidates=[]
        for key in (104+2*axis,105+2*axis):
            if key not in objects:continue
            raw=objects[key]
            if not valid_record(raw,0x41353252,2,584):continue
            if struct.unpack_from('<I',raw,12)[0]!=ref or raw[20:22]!=bytes([1,axis]):continue
            candidates.append(raw)
        if not candidates and not require_scales:
            scales=[]
            break
        require(candidates,f'No current scale observation for axis {axis}')
        scales.append(max(candidates,key=lambda x:struct.unpack_from('<I',x,8)[0]))
    # Apply the same physical/CRC gates as USB, using the maintained wire codec.
    from f06c1.partial_export_codec import PartialFrame,encode
    from batch_snapshot import scale_records
    if scales:
        frame='\n'.join(encode(PartialFrame('scale',ref,3,scales)))
        require(scale_records(frame)==scales,'Scale validation')
    return dict(schema='CALWIRE1',build=build,reference_id=ref,profile_bytes=p.hex(),
        production=dict(scale=list(values[:3]),gsens_dps_per_g=list(values[3:12]),earth_dps=list(values[12:])),
        earth_valid=bool(p[60]),earth_up_axis=p[61],original_auto='disabled',
        thermal_crc32=zlib.crc32(thermal),thermal_config_sig=sig,thermal_points=points,
        lut=[[i]+row for i,row in enumerate(rows)],
        scale_sha256=[hashlib.sha256(x).hexdigest() for x in scales],
        scale_records_hex=[x.hex() for x in scales],
        archive_objects_hex={str(k):v.hex() for k,v in objects.items() if k>=100})


class Radio:
    def __init__(self, receiver_serial, journal, cancel_file=None):
        import serial
        from serial.tools import list_ports
        ports=[p for p in list_ports.comports() if (p.vid,p.pid,p.serial_number)==(0x1209,0x7690,receiver_serial)]
        require(len(ports)==1,'Receiver USB identity absent/ambiguous')
        import os
        lockdir=Path(os.environ.get('EARTH_RATE_STATE_DIR',Path(__file__).resolve().parents[1]/'.local'))/'hardware-locks';lockdir.mkdir(parents=True,exist_ok=True)
        self.lock=lockdir/(hashlib.sha256(receiver_serial.encode()).hexdigest()+'.lock')
        with self.lock.open('x',encoding='ascii') as out:out.write(str(os.getpid()))
        self.journal=journal
        self.cancel_file=Path(cancel_file) if cancel_file else None
        try:
            self.link=serial.Serial(ports[0].device,115200,timeout=.05,write_timeout=2)
            time.sleep(.5);self.link.reset_input_buffer()
        except BaseException:
            self.lock.unlink();raise

    def close(self):
        try:self.link.close()
        finally:self.lock.unlink()

    def request(self,slot,address,token,op,offset=0,data=b''):
        req=packet(slot,address,token,op,offset,data)
        self.journal.add(event='request',packet=req.hex())
        deadline=time.monotonic()+20;retry=0;buffer=bytearray()
        while time.monotonic()<deadline:
            # END remains available during cancellation so frozen device state
            # is released before the GUI returns its hardware reservations.
            if op != 5 and self.cancel_file and self.cancel_file.exists():
                raise InterruptedError('Wireless snapshot cancelled')
            if time.monotonic()>=retry:
                self.link.write(('calwire '+req.hex()+'\r\n').encode('ascii'))
                self.link.flush();retry=time.monotonic()+3.2
            # A fixed 4096-byte read waits out the serial timeout even after a
            # complete reply arrived. Drain available bytes and otherwise wait
            # for one byte; keep the same bounded deadline and line framing.
            buffer.extend(self.link.read(min(4096,self.link.in_waiting or 1)))
            while b'\n' in buffer:
                line,_,buffer=buffer.partition(b'\n')
                match=re.search(rb'CALWIRE ([0-9a-f]{96})',line)
                if match:
                    raw=bytes.fromhex(match[1].decode())
                    self.journal.add(event='reply',packet=raw.hex())
                    if raw[1:16]!=req[1:16] or raw[18:22]!=req[44:48]:continue
                    return response(raw,req)
                if b'CALWIRE_ERROR' in line:
                    self.journal.add(event='relay-error',line=line.decode('ascii','replace'))
            require(len(buffer)<8192,'Unbounded receiver console line')
        raise TimeoutError(f'CALWIRE tracker {slot}, op {op}, offset {offset}')

    @contextmanager
    def session(self,slot,address):
        token=secrets.randbelow(2**32-1)+1
        opened=False
        try:
            # Even a lost BEGIN response may have opened a session: always END.
            opened=True
            meta=self.request(slot,address,token,0)
            require(len(meta)==8,'Snapshot metadata')
            size,crc=struct.unpack('<2I',meta);require(64<=size<=12288,'Snapshot capacity')
            yield token,size,crc
        finally:
            if opened:
                try:self.request(slot,address,token,5)
                except Exception as e:
                    self.journal.add(event='end-unconfirmed',error=str(e),lease_seconds=60)
                    raise

    def snapshot(self,slot,address,token,size,crc):
        blob=bytearray()
        while len(blob)<size:
            chunk=self.request(slot,address,token,1,len(blob))
            require(len(chunk)==min(22,size-len(blob)),'Snapshot chunk length')
            blob.extend(chunk)
        require(zlib.crc32(blob)==crc,'Whole snapshot CRC')
        return bytes(blob)

    def stage_field(self,slot,address,token,before,field,values,apply=False):
        require(field in (1,2,3),'Unknown field')
        size=36 if field==2 else 12;start={1:0,2:12,3:48}[field]
        raw=struct.pack('<'+'f'*(size//4),*values)
        for pos in range(0,size,26):self.request(slot,address,token,2,start+pos,raw[pos:pos+26])
        old=bytes.fromhex(before['profile_bytes'])[start:start+size]
        checks=struct.pack('<3I',zlib.crc32(old),zlib.crc32(raw),before['thermal_crc32'])
        self.request(slot,address,token,3,field,checks)
        if apply:self.request(slot,address,token,4,field,checks)


def main():
    from batch_calibration import Journal,save_new,validate_targets
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--slots',type=int,nargs='+',required=True);p.add_argument('--build',required=True)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    state=json.loads((a.session/'session.json').read_text(encoding='utf-8'))
    targets=validate_targets(state['targets']);radio=Radio(state['receiver_serial'],Journal(a.out/'wire.jsonl'))
    try:
        for slot in a.slots:
            require(slot in targets,'Slot outside fleet')
            dest=a.out/f'slot-{slot}';require(not dest.exists(),'Existing snapshot preserved')
            dest.mkdir()
            with radio.session(slot,targets[slot]) as (token,size,crc):
                blob=radio.snapshot(slot,targets[slot],token,size,crc)
            (dest/'snapshot.bin').write_bytes(blob)
            data=unpack_snapshot(blob);require(data['build']==a.build,'Firmware identity mismatch')
            data.update(slot=slot,rf_address=targets[slot],snapshot_sha256=hashlib.sha256(blob).hexdigest())
            save_new(dest/'backup.json',data)
            for i,raw in enumerate(data['scale_records_hex']):(dest/f'scale-{i}.bin').write_bytes(bytes.fromhex(raw))
            print(json.dumps(dict(saved=slot,bytes=len(blob),points=sum(x[0]!=0 for x in data['thermal_points']))),flush=True)
    finally:radio.close()

if __name__=='__main__':main()

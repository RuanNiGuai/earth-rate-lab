"""Install reviewed CALWIRE S/M/E plans; journal each non-atomic NVS write.

Trial authorization is recorded separately from quality-gate acceptance.
Every prefix of the three-field transaction is identifiable for rollback.
An unrecognized state is never overwritten. Reboot verification is separate.
"""
import argparse,hashlib,json,math,os,struct,zlib
from pathlib import Path
from calwire import Radio,unpack_snapshot,require
from batch_calibration import Journal,save_new,validate_targets

FIELDS={'scale':(1,0,12),'gsens_dps_per_g':(2,12,36),'earth_dps':(3,48,12)}

def digest(raw):return hashlib.sha256(raw).hexdigest()

def profile_location(blob):
    p=0
    while p+4<=len(blob):
        kind,n=struct.unpack_from('<HH',blob,p)
        require(p+4+n<=len(blob),'Truncated TLV')
        if kind==2:
            require(n==64,'Profile size');return p+4
        p+=4+n
    raise ValueError('Missing profile')

def prepare(plan,state):
    require(plan['schema']=='CALWIRE_APPLY2','Plan schema')
    require(plan['receiver_serial']==state['receiver_serial'],'Receiver identity')
    evpath=Path(plan['evidence_path']);evidence=json.loads(evpath.read_text(encoding='utf-8'))
    require(digest(evpath.read_bytes())==plan['evidence_sha256'],'Evidence changed')
    if evidence['mode']=='USER_AUTHORIZED_TRIAL':
        require(bool(evidence['user_authorization_quote']),'Missing trial authorization')
        require(evidence['quality_gates_passed'] is False and evidence['unmet_quality_gates'],
                'Trial must disclose unmet quality gates')
    else:
        require(evidence['mode']=='QUALITY_ACCEPTED' and evidence['quality_gates_passed'] is True,'Acceptance mode')
    if evidence.get('selection_path'):
        require(digest(Path(evidence['selection_path']).read_bytes())==evidence['selection_sha256'],
                'Parameter comparison changed')
    targets=validate_targets(state['targets']);entries=[];seen=set()
    for item in plan['devices']:
        slot=item['slot']
        require(slot in targets and slot not in seen and item['rf_address']==targets[slot],'Target identity/duplicate')
        seen.add(slot);decision=evidence['decisions'][str(slot)]
        require(decision['install_authorized'] is True and decision['reason'],'Missing install authorization')
        before=Path(item['snapshot_path']).read_bytes()
        require(digest(before)==item['snapshot_sha256']==decision['snapshot_sha256'],'Backup evidence binding')
        parsed=unpack_snapshot(before, require_scales=False);require(parsed['build']==plan['build'],'Firmware build')
        require(item['values']==decision['values'] and set(item['values'])==set(FIELDS),'Candidate/evidence fields')
        actions=item.get('field_actions')
        if actions is not None:
            require(actions==decision.get('field_actions') and set(actions)==set(FIELDS)
                    and all(v in ('keep','update') for v in actions.values()),'Invalid field selection')
        require(parsed['earth_valid'] and parsed['earth_up_axis']==0,'Earth reference must already be valid +X up')
        require(evidence.get('reference_pose_confirmed')=='original +X-up face and original heading',
                'Earth installation requires matching fixed reference pose')
        states=[before];steps=[]
        for field,(number,start,size) in FIELDS.items():
            values=item['values'][field]
            require(len(values)==size//4 and all(math.isfinite(v) for v in values),'Candidate values')
            if number==1:require(all(.9<=v<=1.1 for v in values),'Scale bounds')
            elif number==2:require(all(abs(v)<=.05 for v in values),'G-sens bounds')
            else:require(abs(math.sqrt(sum(v*v for v in values))-.0041780746)<1e-7,'Earth magnitude')
            offset=profile_location(before)+start;value=struct.pack('<'+'f'*(size//4),*values)
            if actions and actions[field]=='keep':
                require(before[offset:offset+size]==value,'Retained parameter changed')
            after=bytearray(states[-1]);after[offset:offset+size]=value;after=bytes(after)
            if after==states[-1]:continue
            steps.append(dict(field=field,number=number,values=values))
            states.append(after)
        require(steps or actions is not None,'No-op candidate')
        entries.append(dict(slot=slot,address=targets[slot],before=before,after=states[-1],states=states,steps=steps))
    require(seen==set(targets),'Joint plan must include the entire bound fleet')
    require(any(e['steps'] for e in entries),'No parameter changes are needed')
    return entries

def check_session(radio,e,token,size,crc,expected):
    require(size==len(expected) and crc==zlib.crc32(expected),'Snapshot changed')
    offset=profile_location(expected);got=bytearray()
    while len(got)<64:
        chunk=radio.request(e['slot'],e['address'],token,1,offset+len(got))
        require(len(chunk)==min(22,size-offset-len(got)),'Profile chunk length')
        got.extend(chunk)
    require(bytes(got[:64])==expected[offset:offset+64],'Profile readback differs')

def verify_session(radio,e,expected):
    """Check whole frozen snapshot CRC and independently read all profile bytes."""
    with radio.session(e['slot'],e['address']) as (token,size,crc):
        check_session(radio,e,token,size,crc,expected)

def identify(radio,e):
    with radio.session(e['slot'],e['address']) as (token,size,crc):
        matches=[i for i,blob in enumerate(e['states']) if len(blob)==size and zlib.crc32(blob)==crc]
        require(len(matches)==1,'Recovery stopped: unrecognized concurrent change')
        i=matches[0];check_session(radio,e,token,size,crc,e['states'][i]);return i

def restore(radio,e,journal):
    """Recover any known partial commit, including a committed write with lost ACK."""
    count=identify(radio,e)
    while count:
        step=e['steps'][count-1];current=e['states'][count];previous=e['states'][count-1]
        with radio.session(e['slot'],e['address']) as (token,size,crc):
            check_session(radio,e,token,size,crc,current)
            old=unpack_snapshot(previous, require_scales=False)['production'][step['field']]
            journal.add(event='rollback-requested',slot=e['slot'],field=step['field'])
            radio.stage_field(e['slot'],e['address'],token,unpack_snapshot(current, require_scales=False),step['number'],old,apply=True)
        verify_session(radio,e,previous);count-=1
    verify_session(radio,e,e['before'])
    journal.add(event='rollback-readback-verified',slot=e['slot'],requires_reboot_verification=True)

def apply_entries(radio,entries,journal):
    # All-fleet preflight happens before the first NVS mutation.
    installed=set()
    for e in entries:
        try:
            state=identify(radio,e)
            require(state in (0,len(e['steps'])),
                    'Known partial write; restore the reviewed backup before retrying')
        except ValueError as error:
            raise ValueError(f"Tracker #{e['slot']}: {error}") from error
        if state==len(e['steps']):installed.add(e['slot'])
    touched=[]
    try:
        for e in entries:
            if e['slot'] in installed:
                verify_session(radio,e,e['after'])
                journal.add(event='already-applied-readback-verified',slot=e['slot'])
                print(json.dumps(dict(applied=e['slot'],fields=[],already_applied=True,reboot_verified=False)),flush=True)
                continue
            touched.append(e)
            for i,step in enumerate(e['steps']):
                before=e['states'][i];after=e['states'][i+1]
                with radio.session(e['slot'],e['address']) as (token,size,crc):
                    check_session(radio,e,token,size,crc,before)
                    journal.add(event='apply-requested',slot=e['slot'],field=step['field'],values=step['values'])
                    radio.stage_field(e['slot'],e['address'],token,unpack_snapshot(before, require_scales=False),step['number'],step['values'],apply=True)
                verify_session(radio,e,after)
                journal.add(event='field-readback-verified',slot=e['slot'],field=step['field'])
            journal.add(event='apply-readback-verified',slot=e['slot'],requires_reboot_verification=True)
            print(json.dumps(dict(applied=e['slot'],fields=[s['field'] for s in e['steps']],reboot_verified=False)),flush=True)
    except BaseException as exc:
        journal.add(event='apply-failed',error=str(exc))
        for e in reversed(touched):
            try:restore(radio,e,journal)
            except Exception as recovery:journal.add(event='rollback-unconfirmed',slot=e['slot'],error=str(recovery))
        raise
    return dict(already_applied_slots=sorted(installed),written_slots=[e['slot'] for e in touched])

def rollback_entries(radio,entries,journal):
    # A foreign peer must stop the entire explicit rollback before any writes.
    for e in entries:identify(radio,e)
    for e in reversed(entries):restore(radio,e,journal)


def durable_bytes(path,raw):
    with path.open('xb') as stream:
        stream.write(raw);stream.flush();os.fsync(stream.fileno())


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True);p.add_argument('--session',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--mode',choices=['apply','rollback','verify'],default='apply');a=p.parse_args()
    plan=json.loads(a.plan.read_text(encoding='utf-8'));state=json.loads((a.session/'session.json').read_text(encoding='utf-8'))
    entries=prepare(plan,state);a.out.mkdir(parents=True,exist_ok=False)
    save_new(a.out/'plan.json',plan);journal=Journal(a.out/'journal.jsonl')
    for e in entries:
        folder=a.out/f"slot-{e['slot']}";folder.mkdir()
        durable_bytes(folder/'before.bin',e['before']);durable_bytes(folder/'expected-after.bin',e['after'])
    radio=Radio(plan['receiver_serial'],journal)
    details={}
    try:
        if a.mode=='apply':details=apply_entries(radio,entries,journal)
        elif a.mode=='rollback':rollback_entries(radio,entries,journal)
        else:
            for e in entries:
                verify_session(radio,e,e['after']);print(json.dumps(dict(verified=e['slot'])),flush=True)
    finally:radio.close()
    save_new(a.out/'result.json',dict(state=a.mode.upper()+'_READBACK_VERIFIED',reboot_verified=False,
        plan_sha256=digest(a.plan.read_bytes()),slots=[e['slot'] for e in entries],**details))

if __name__=='__main__':main()

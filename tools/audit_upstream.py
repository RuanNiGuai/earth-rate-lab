"""Compare published source trees with pinned upstream Git revisions, offline.

Clone/fetch the repositories separately. This tool neither updates repositories
nor reads credentials. Line ending differences are ignored; missing Git submodule
contents are audited separately, never counted as local feature additions.
"""
import argparse
import difflib
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile

from release_export import select

ROOT = Path(__file__).resolve().parents[1]


def git(repo, *args):
    return subprocess.check_output(['git', '-C', str(repo), *args])


def archived(repo, revision, exclude):
    raw = git(repo, 'archive', revision)
    with tarfile.open(fileobj=io.BytesIO(raw)) as archive:
        return {m.name: archive.extractfile(m).read().replace(b'\r\n', b'\n')
                for m in archive.getmembers() if m.isfile()
                and not any(m.name == p or m.name.startswith(p + '/') for p in exclude)}


def compare(before, after):
    rows=[]
    for name in sorted(before.keys() | after.keys()):
        a,b=before.get(name),after.get(name)
        if a==b:continue
        row=dict(path=name,status='added' if a is None else 'removed' if b is None else 'modified')
        for key,data in [('upstream',a),('local',b)]:
            if data is not None:row[key+'_sha256']=hashlib.sha256(data).hexdigest()
        try:
            aa=(a or b'').decode('utf8').splitlines();bb=(b or b'').decode('utf8').splitlines()
            plus=minus=0
            for tag,i,j,k,l in difflib.SequenceMatcher(None,aa,bb,autojunk=False).get_opcodes():
                if tag in ('replace','delete'):minus+=j-i
                if tag in ('replace','insert'):plus+=l-k
            row.update(added_lines=plus,removed_lines=minus)
        except UnicodeDecodeError:
            row['binary']=True
        rows.append(row)
    return dict(counts={k:sum(r['status']==k for r in rows) for k in ['added','modified','removed']},files=rows)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['tracker','receiver','vqf','helper']:p.add_argument('--'+key+'-repo',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    pins=json.loads((ROOT/'config/upstream-audit.json').read_text(encoding='utf8'))
    files=select(ROOT);output=dict(checked_at=pins['checked_at'],method='LF-normalized published files versus git archive; commits pinned',repositories=[])
    for row in pins['repositories']:
        folder=ROOT/row['local_path'];exclude=row.get('exclude',[])
        local={f.relative_to(folder).as_posix():f.read_bytes().replace(b'\r\n',b'\n') for f in files
               if f.is_relative_to(folder) and not any(f.relative_to(folder).as_posix()==x or f.relative_to(folder).as_posix().startswith(x+'/') for x in exclude)}
        repo=getattr(a,row['key']+'_repo')
        report={k:row[k] for k in ['key','url','branch','base','head','local_path']}
        report['base_kind']=row.get('base_kind','commit')
        report['excluded']=exclude
        for kind in ['base','head']:report['versus_'+kind]=compare(archived(repo,row[kind],exclude),local)
        if report['base_kind']=='commit':
            log=git(repo,'log','--reverse','--format=%H%x09%cs%x09%s',row['base']+'..'+row['head']).decode('utf8')
            report['upstream_commits_since_base']=[dict(zip(['sha','date','subject'],line.split('\t',2))) for line in log.splitlines()]
        output['repositories'].append(report)
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    for row in output['repositories']:print(row['key'],row['versus_base']['counts'],row['versus_head']['counts'])


if __name__=='__main__':main()

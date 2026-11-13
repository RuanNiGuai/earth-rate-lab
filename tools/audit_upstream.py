"""Compare a pinned published project commit with upstream Git commits, offline.

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

ROOT = Path(__file__).resolve().parents[1]


def git(repo, *args):
    repo = repo.resolve()
    return subprocess.check_output(['git', '-c', 'safe.directory=' + repo.as_posix(),
                                    '-C', str(repo), *args])


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


def audit(pins, project_repo, upstream_repos):
    # Both sides come from committed Git objects. Dirty checkouts, earlier local
    # experiments and later documentation edits cannot change this comparison.
    project = pins['project']
    published = archived(project_repo, project['commit'], [])
    output = dict(schema_version=2, checked_at=pins['checked_at'], project=project,
                  method='LF-normalized git archives of pinned project and upstream commits; path comparison, not ancestry',
                  repositories=[])
    for row in pins['repositories']:
        prefix = row['local_path'].rstrip('/') + '/'
        exclude = row.get('exclude', [])
        local = {name[len(prefix):]: data for name, data in published.items()
                 if name.startswith(prefix) and not any(
                     name[len(prefix):] == x or name[len(prefix):].startswith(x + '/')
                     for x in exclude)}
        if not local:
            raise ValueError('No published files at ' + row['local_path'])
        upstream = archived(upstream_repos[row['key']], row['head'], exclude)
        report = {key: row[key] for key in ['key', 'url', 'branch', 'head', 'local_path']}
        report['excluded'] = exclude
        report['upstream_files'] = len(upstream)
        report['project_files'] = len(local)
        report['unchanged_files'] = sum(name in local and data == local[name]
                                        for name, data in upstream.items())
        report['versus_upstream'] = compare(upstream, local)
        output['repositories'].append(report)
    return output


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['tracker','receiver','vqf','helper']:p.add_argument('--'+key+'-repo',type=Path,required=True)
    p.add_argument('--project-repo', type=Path, required=True,
                   help='Git repository containing config/upstream-audit.json project.commit')
    p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    pins=json.loads((ROOT/'config/upstream-audit.json').read_text(encoding='utf8'))
    output = audit(pins, a.project_repo,
                   {row['key']: getattr(a, row['key'] + '_repo') for row in pins['repositories']})
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    for row in output['repositories']:
        print(row['key'], row['versus_upstream']['counts'], 'unchanged:', row['unchanged_files'])


if __name__=='__main__':main()

"""Build the optional standalone calibration component; never accesses hardware."""
import argparse
import hashlib
import importlib.metadata as metadata
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def copy_compiler_notices(destination):
    from native_test import compiler
    licenses=Path(compiler()).resolve().parents[1]/'share/licenses'
    for name in ('libgcc','crt'):
        source=licenses/name
        if not source.is_dir():raise ValueError('Missing native compiler runtime notices: '+str(source))
        shutil.copytree(source,destination/name)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--work',type=Path,required=True)
    p.add_argument('--python-license',type=Path,help='License for this exact Python build when its installation omits LICENSE.txt')
    a=p.parse_args();a.out=a.out.resolve();a.work=a.work.resolve()
    python_license=a.python_license or Path(sys.base_prefix)/'LICENSE.txt'
    if not python_license.exists():raise ValueError('Python license unavailable; do not distribute the component')
    if (a.out/'calibration').exists():raise ValueError('Existing runtime preserved; choose a new output directory')
    a.work.mkdir(parents=True,exist_ok=True);a.out.mkdir(parents=True,exist_ok=True)
    scripts=['helper_utilities.py','thermal_coverage.py','helper_fleet.py','fleet_workflow.py','batch_calibration.py','batch_record_only.py','capture_quality.py','capture_preflight.py',
             'fusion_replay.py','fusion_replay.c','fusion_stable_iir.py','native_test.py',
             'f04_summary.py','calibration_coordinates.py','hardware_lock.py','calwire.py','calwire_apply.py','cal_archive.py','batch_snapshot.py','f06c1/partial_export_codec.py']
    scripts += [str(p.relative_to(ROOT/'tools')) for p in (ROOT/'tools/jointcal').glob('*.py') if not p.name.startswith('test_')]
    args=[sys.executable,'-m','PyInstaller','--noconfirm','--onedir','--name','earth-rate-calibration',
          '--distpath',str(a.work/'dist'),'--workpath',str(a.work/'build'),'--specpath',str(a.work),
          '--paths',str(ROOT/'tools'),'--paths',str(ROOT/'tools/jointcal'),
          '--hidden-import','joint_rigid_v3','--hidden-import','joint_fit_v2','--hidden-import','calwire_apply',
          '--hidden-import','serial.tools.list_ports','--hidden-import','hid']
    source={}
    for rel in scripts:
        path=ROOT/'tools'/rel
        args += ['--add-data',str(path)+';'+str(Path('toolkit/tools')/Path(rel).parent)]
        source['tools/'+Path(rel).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    # Ship source-bound native replays; users do not need a C compiler.
    from fusion_replay import Replay
    native=a.work/'native';native.mkdir(exist_ok=True)
    engines={key:Replay(a.work/'native-build',iir=key) for key in ('production','stable','legacy')}
    engines['production-cmsis']=Replay(a.work/'native-build',math_backend='cmsis')
    replay_manifest={}
    for key,engine in engines.items():
        name=key+engine.library_path.suffix
        shutil.copy2(engine.library_path,native/name)
        replay_manifest[key]=dict(library=name,sources=engine.hashes,
                                  sha256=hashlib.sha256((native/name).read_bytes()).hexdigest())
        for rel in engine.hashes:
            if rel.startswith(('integration/','helper/')) and rel not in source:
                path=ROOT/rel
                args+=['--add-data',str(path)+';'+str(Path('toolkit')/Path(rel).parent)]
                source[rel]=hashlib.sha256(path.read_bytes()).hexdigest()
    (native/'replay-manifest.json').write_text(json.dumps(replay_manifest,indent=2),encoding='utf8')
    args+=['--add-data',str(native)+';toolkit/native','--hidden-import','select_parameters',
           '--hidden-import','scipy.spatial.transform._rotation']
    args.append(str(ROOT/'tools/helper_fleet.py'))
    with (a.work/'pyinstaller.log').open('w',encoding='utf8') as log:
        subprocess.run(args,stdout=log,stderr=subprocess.STDOUT,check=True)
    dest=a.out/'calibration'
    shutil.copytree(a.work/'dist/earth-rate-calibration',dest)
    notices=dest/'licenses';notices.mkdir()
    copy_compiler_notices(notices)
    shutil.copy2(ROOT/'integration/tracker/vqf-c/LICENSE',notices/'VQF-LICENSE.txt')
    for name,source_path in [('Project-LICENSE.txt',ROOT/'LICENSE'),
                             ('Tracker-LICENSE-MIT.txt',ROOT/'integration/tracker/LICENSE-MIT'),
                             ('Tracker-LICENSE-APACHE.txt',ROOT/'integration/tracker/LICENSE-APACHE')]:
        shutil.copy2(source_path,notices/name)
    packages={}
    for name in ('numpy','scipy','pyserial','hidapi','PyInstaller','pyinstaller-hooks-contrib','altgraph','pefile','pywin32-ctypes','packaging','setuptools'):
        dist=metadata.distribution(name);packages[name]=dist.version
        for entry in dist.files or []:
            if '..' not in entry.parts and any(word in entry.name.lower() for word in ('license','copying','notice')) and entry.suffix.lower() not in ('.py','.pyc'):
                target=notices/name/str(entry);target.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(dist.locate_file(entry),target)
        (notices/(name+'-metadata.txt')).write_text(dist.read_text('METADATA') or '',encoding='utf8')
    python_license=a.python_license or Path(sys.base_prefix)/'LICENSE.txt'
    if not python_license.exists():raise ValueError('Python license unavailable; do not distribute the component')
    shutil.copy2(python_license,notices/'Python-LICENSE.txt')
    native_notices=ROOT/'config/helper-runtime-notices'
    frozen=json.loads((native_notices/'sources.json').read_text(encoding='utf8'))
    for name,row in frozen.items():
        if hashlib.sha256((native_notices/name).read_bytes()).hexdigest()!=row['sha256']:
            raise ValueError('Native runtime notice changed: '+name)
    shutil.copytree(native_notices,notices/'native-runtime')
    manifest=dict(python=sys.version,packages=packages,source_sha256=source,files={},
                  build_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  requirements_sha256=hashlib.sha256((ROOT/'requirements-helper-runtime.txt').read_bytes()).hexdigest())
    for path in sorted(dest.rglob('*')):
        if path.is_file():manifest['files'][path.relative_to(dest).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    (dest/'RUNTIME_MANIFEST.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
    print('Standalone calibration component built:',dest)


if __name__=='__main__':main()

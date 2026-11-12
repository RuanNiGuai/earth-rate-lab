"""Run reproducible synthetic fusion counterexamples; never accesses devices.

This compares the fixed-parameter VQF core and x-io Fusion AHRS, each with and
without runtime bias learning. It does not simulate the full firmware or the
upstream experimental bundle and cannot rank wearable accuracy.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
from native_test import compiler

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    cc = compiler()
    env = dict(os.environ)
    env['PATH'] = str(Path(cc).resolve().parent) + os.pathsep + env.get('PATH', '')
    build = ROOT / '.local/native-tests'
    build.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=build) as tmp:
        env.update(TEMP=tmp, TMP=tmp)
        exe = Path(tmp) / ('scenarios.exe' if os.name == 'nt' else 'scenarios')
        subprocess.run([cc, '-std=c11', '-O2', '-Wall', '-Wextra',
                        '-I'+str(ROOT/'integration/tracker/vqf-c/src'),
                        '-I'+str(ROOT/'integration/tracker/Fusion/Fusion'),
                        str(ROOT/'tools/fusion_scenarios.c'), '-o', str(exe), '-lm'],
                       check=True, env=env)
        results = json.loads(subprocess.check_output([str(exe)], env=env, text=True))
    results['scope'] = 'Synthetic, noiseless core counterexamples; not measured wearable accuracy or a full upstream strategy replay.'
    paths = [ROOT/'tools/fusion_scenarios.c', *sorted((ROOT/'integration/tracker/vqf-c/src').glob('*.[ch]')),
             *sorted((ROOT/'integration/tracker/Fusion/Fusion').glob('*.[ch]'))]
    results['source_sha256'] = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(results, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k:v for k,v in results.items() if k!='source_sha256'}, indent=2))


if __name__ == '__main__':
    main()

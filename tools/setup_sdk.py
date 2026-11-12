"""Initialize pinned SDK workspaces; no firmware deployment or host-wide configuration."""
import argparse
import json
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('target', choices=['tracker', 'receiver'])
    p.add_argument('--workspace', type=Path, required=True)
    p.add_argument('--init-only', action='store_true')
    a = p.parse_args()
    ws = a.workspace.resolve()
    config = ROOT / 'config/sdk' / a.target
    if (ws/'.west').exists():
        raise ValueError('Use a new SDK workspace; existing workspaces are not reset')
    (ws/'manifest').mkdir(parents=True, exist_ok=False)
    shutil.copy2(config/'west.yml', ws/'manifest/west.yml')
    subprocess.run([sys.executable, '-m', 'west', 'init', '-l', 'manifest'], cwd=ws, check=True)
    if a.init_only:
        return
    subprocess.run([sys.executable, '-m', 'west', 'update', '--narrow'], cwd=ws, check=True)
    if a.target == 'receiver':
        subprocess.run(['git', '-C', str(ws/'zephyr'), 'apply', '--check', str(config/'zephyr-compat.patch')], check=True)
        subprocess.run(['git', '-C', str(ws/'zephyr'), 'apply', str(config/'zephyr-compat.patch')], check=True)
    print('SDK ready. Install its Python requirements:')
    print(f'python -m pip install -r "{ROOT / "requirements-build-lock.txt"}"')


if __name__ == '__main__':
    main()

"""Export an allowlisted source tree, refusing keys, caches and oversized files."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
SKIP = {'.git', '.github', '.vscode', '.local', '__pycache__', '.pytest_cache', 'target',
        'node_modules', 'build', 'native-build'}
PRIVATE = re.compile(rb'-----BEGIN (?:[A-Z0-9 ]+ )?PRIVATE KEY-----')
TOKEN = re.compile(rb'\b(?:gh[pousr]_[A-Za-z0-9]{36,255}|github_pat_[A-Za-z0-9_]{40,255})\b')


def select(root):
    spec = json.loads((root/'config/publish.json').read_text(encoding='utf8'))
    paths = set()
    for name in spec['files']:
        paths.add(root/name)
    for manifest in spec.get('manifests', []):
        names = json.loads((root/manifest['files']).read_text(encoding='utf8'))
        for name in names:
            paths.add(root/manifest['root']/name)
    for name in spec['trees']:
        if not (root/name).is_dir():
            raise ValueError('Missing source tree: '+name)
        for base, dirs, names in os.walk(root/name):
            dirs[:] = [d for d in dirs if d not in SKIP and not d.startswith('build-')]
            for f in names:
                path = Path(base)/f
                if path == root/'integration/tracker/VERSION':
                    continue  # Generated from VERSION_SRC and Git metadata by CMake.
                if f not in ('.git', '.gitmodules') and not f.endswith(('.pyc', '.log')):
                    paths.add(path)
    return sorted(paths)


def inspect(path, root):
    if path.is_symlink() or not path.is_file() or not path.resolve().is_relative_to(root.resolve()):
        raise ValueError('Missing file, symlink or path escape: '+str(path))
    if path.stat().st_size > 10*1024*1024:
        raise ValueError('Oversized source file; use separate release/data artifact: '+str(path))
    data = path.read_bytes()
    if PRIVATE.search(data) or TOKEN.search(data):
        raise ValueError('Secret marker found; content withheld: '+str(path))
    if path.suffix.lower() in ('.pfx', '.p12', '.key', '.exe', '.dll', '.elf', '.zip'):
        raise ValueError('Non-source/private artifact: '+str(path))
    if path.suffix.lower() == '.pem' and b'PUBLIC KEY-----' not in data:
        raise ValueError('Not a public-only PEM: '+str(path))
    if re.search(rb'[A-Za-z]:[\\/](?:Users|MyGame)[\\/]', data):
        raise ValueError('Machine-specific path: '+str(path))
    return data


def check_links(root, files):
    # Maintained documentation only. Third-party documents retain upstream links.
    selected = {p.resolve() for p in files}
    for path in files:
        rel = path.relative_to(root)
        if path.suffix != '.md' or (rel.parts[0] == 'integration' and len(rel.parts) > 2) or rel.parts[0] == 'helper':
            continue
        for match in re.finditer(r'\]\(([^)]+)\)', path.read_text(encoding='utf8')):
            target = match[1].split('#')[0]
            if not target or re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:', target):
                continue
            from urllib.parse import unquote
            dest = (path.parent/unquote(target.strip('<>'))).resolve()
            if dest not in selected and not (dest.is_dir() and any(p.is_relative_to(dest) for p in selected)):
                raise ValueError(f'Unpublished document link: {rel} -> {target}')


def export(root, out=None):
    paths = select(root)
    manifest = {}
    # Validate everything before creating the destination.
    for p in paths:
        data = inspect(p, root)
        manifest[p.relative_to(root).as_posix()] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
    check_links(root, paths)
    if out is not None:
        out = out.resolve()
        out.mkdir(parents=True, exist_ok=False)
        for p in paths:
            dest = out/p.relative_to(root)
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(p, dest)
        (out/'SOURCE_MANIFEST.json').write_text(json.dumps(manifest, indent=2), encoding='utf8')
    return dict(files=len(manifest), bytes=sum(x['bytes'] for x in manifest.values()))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument('--out', type=Path)
    group.add_argument('--check-only', action='store_true')
    a = p.parse_args()
    print(json.dumps(export(ROOT, a.out)))


if __name__ == '__main__':
    main()

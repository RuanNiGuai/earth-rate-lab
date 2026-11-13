"""Build the current Styria candidate with explicit SDK/toolchain/key paths."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]


def fingerprint(source):
    h = hashlib.sha256()
    for directory in (source, ROOT/'config'):
        for p in sorted(directory.rglob('*')):
            if not p.is_file() or any(x in ('.git', '.github', '__pycache__') for x in p.parts):
                continue
            if p == ROOT/'integration/tracker/VERSION':
                continue  # Generated metadata must not change the source identity.
            h.update(p.relative_to(ROOT).as_posix().encode())
            h.update(b'\0')
            h.update(p.read_bytes())
    return h.hexdigest()


def verify_sdk(target, ws, git='git'):
    config = ROOT/'config/sdk'/target
    manifest = yaml.safe_load((config/'west.yml').read_text())['manifest']
    for p in manifest['projects']:
        repo = ws/p.get('path', p['name'])
        cmd = [git, '-c', 'safe.directory='+repo.as_posix(), '-C', str(repo)]
        actual = subprocess.check_output(cmd+['rev-parse', 'HEAD'], text=True).strip()
        if actual != p['revision']:
            raise ValueError('SDK commit mismatch: '+p['name'])
        diff = subprocess.check_output(cmd+['diff', 'HEAD', '--binary'])
        expected = (config/'zephyr-compat.patch').read_bytes() if target == 'receiver' and p['name'] == 'zephyr' else b''
        if diff.replace(b'\r\n', b'\n') != expected.replace(b'\r\n', b'\n'):
            raise ValueError('Unrecorded SDK changes: '+p['name'])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('target', choices=['tracker', 'receiver'])
    p.add_argument('--workspace', type=Path, required=True)
    p.add_argument('--toolchain', type=Path, required=True)
    p.add_argument('--build-dir', type=Path, required=True)
    p.add_argument('--signing-key', type=Path)
    p.add_argument('--jobs', type=int, default=2)
    p.add_argument('--git', default='git', help='Git executable (optional)')
    a = p.parse_args()
    ws = a.workspace.resolve()
    source = ROOT/'integration'/a.target
    build = a.build_dir.resolve()
    if build.exists():
        raise ValueError('Use a new build directory; this command never reuses a CMake cache')
    git = str(Path(a.git).resolve()) if Path(a.git).is_file() else a.git
    verify_sdk(a.target, ws, git)
    env = dict(os.environ, ZEPHYR_BASE=str(ws/'zephyr'), ZEPHYR_TOOLCHAIN_VARIANT='zephyr',
               ZEPHYR_SDK_INSTALL_DIR=str(a.toolchain.resolve()), CMAKE_BUILD_PARALLEL_LEVEL=str(a.jobs))
    env['PATH'] = str(Path(sys.executable).parent) + os.pathsep + env.get('PATH', '')
    if Path(git).is_file():
        env['PATH'] = str(Path(git).parent) + os.pathsep + env['PATH']
    # Trust this owned checkout only in the build process, not global Git config.
    idx = int(env.get('GIT_CONFIG_COUNT', '0'))
    env.update({f'GIT_CONFIG_KEY_{idx}':'safe.directory', f'GIT_CONFIG_VALUE_{idx}':source.as_posix(),
                'GIT_CONFIG_COUNT':str(idx+1)})
    sha = fingerprint(source)
    identity = sha[:12] + '_cw6_stable' if a.target == 'tracker' else sha[:12] + '_cw4_f04ack'
    configs = []
    overlays = []
    extra = ['-DBOARD_ROOT='+source.as_posix(), '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON']
    if a.target == 'tracker':
        if a.signing_key is None or not a.signing_key.is_file():
            raise ValueError('Tracker requires --signing-key; use the key matching its MCUboot public key')
        configs = [source/'arc8/app_mcuboot.conf', source/'boards/arc8_joint10_ota32_test.conf',
                   ROOT/'config/tracker_trace.conf', source/'boards/f04_summary.conf', source/'boards/wom_retained.conf']
        overlays = [source/'arc8/app_mcuboot.overlay', source/'boards/f04_entropy.overlay', source/'arc8/wom_retained.overlay']
        extra += ['-DPM_STATIC_YML_FILE='+str(source/'arc8/pm_static_wom.yml'),
                  '-DCONFIG_MCUBOOT_SIGNATURE_KEY_FILE="'+a.signing_key.resolve().as_posix()+'"',
                  '-DCONFIG_ARC8P1_READONLY_EXPORT_DRAFT=y', '-DARC8_BUILD_ID='+identity]
        board = 'styria_mini_uf2/nrf52840/spi'
    else:
        board = 'styria_r1_uf2/nrf52840'
        configs = [ROOT/'config/receiver_trace.conf']
        overlays = [ROOT/'config/receiver_hid.overlay']
        extra += ['-Dreceiver_ARC8_BUILD_ID='+identity]
    extra += ['-DEXTRA_CONF_FILE='+';'.join(p.as_posix() for p in configs),
              '-DEXTRA_DTC_OVERLAY_FILE='+';'.join(p.as_posix() for p in overlays)]
    cmd = [sys.executable, '-m', 'west', 'build', '--no-sysbuild' if a.target == 'tracker' else '--sysbuild',
           '-s', str(source), '-b', board, '-d', str(build), '--'] + extra
    # Some Windows toolchains cannot use non-ASCII profile temporary paths.
    tmp = build/'compiler-tmp'
    tmp.mkdir(parents=True)
    env.update(TMP=str(tmp), TEMP=str(tmp), TMPDIR=str(tmp))
    subprocess.run(cmd, cwd=ws, env=env, check=True)
    (build/'source-identity.json').write_text(json.dumps(dict(build_id=identity, source_sha256=sha,
        target=a.target, deployed=False, sdk_manifest_sha256=hashlib.sha256((ROOT/'config/sdk'/a.target/'west.yml').read_bytes()).hexdigest()), indent=2), encoding='utf8')
    print('Built candidate; not deployed: '+identity)


if __name__ == '__main__':
    main()

"""Compile production C in a host harness; require an available compiler."""
import os
from pathlib import Path
import shutil
import subprocess


def compiler():
    explicit = os.environ.get('CC')
    if explicit:
        return explicit
    for name in ('gcc', 'clang'):
        if shutil.which(name):
            return shutil.which(name)
    local = Path(__file__).resolve().parents[1] / 'toolchain/msys64/msys64/mingw64/bin/gcc.exe'
    if local.exists():
        return str(local)
    raise RuntimeError('Install GCC/Clang or set CC to the compiler executable')


def compile_run(code, directory, include=()):
    directory = Path(directory)
    source = directory / 'test.c'
    source.write_text(code, encoding='utf8')
    exe = directory / ('test.exe' if os.name == 'nt' else 'test')
    cc = compiler()
    env = dict(os.environ)
    env['PATH'] = str(Path(cc).resolve().parent) + os.pathsep + env.get('PATH', '')
    env['TEMP'] = env['TMP'] = str(directory)
    cmd = [cc, '-std=c11', '-Wall', '-Wextra', '-Werror']
    cmd += ['-I' + str(x) for x in include]
    cmd += [str(source), '-o', str(exe), '-lm']
    result = subprocess.run(cmd, capture_output=True, text=True, errors='replace', env=env)
    if result.returncode:
        raise RuntimeError(result.stderr)
    result = subprocess.run([str(exe)], capture_output=True, text=True, errors='replace', env=env)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)

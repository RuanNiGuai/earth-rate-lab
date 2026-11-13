"""Process-lifetime lock with conservative recovery of an abandoned PID marker.

The persistent .lease inode serializes new clients; the exclusive PID marker
also blocks older clients. Never remove a marker belonging to a live/unknown
process. A reused PID therefore causes a safe refusal, not a stolen lease.
"""
import os
from pathlib import Path


def process_alive(pid):
    if pid <= 0 or pid > 0xffffffff:
        return True
    if os.name != 'nt':
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True
    import ctypes
    from ctypes import wintypes as w
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.OpenProcess.argtypes=[w.DWORD,w.BOOL,w.DWORD];kernel.OpenProcess.restype=w.HANDLE
    kernel.GetExitCodeProcess.argtypes=[w.HANDLE,ctypes.POINTER(w.DWORD)];kernel.GetExitCodeProcess.restype=w.BOOL
    kernel.CloseHandle.argtypes=[w.HANDLE];kernel.CloseHandle.restype=w.BOOL
    handle=kernel.OpenProcess(0x1000,False,pid)
    if not handle:
        return ctypes.get_last_error()!=87  # Only invalid PID proves absence.
    try:
        code=w.DWORD()
        return not kernel.GetExitCodeProcess(handle,ctypes.byref(code)) or code.value==259
    finally:
        kernel.CloseHandle(handle)


class ProcessLock:
    def __init__(self,path):
        self.path=Path(path);self.owner=str(os.getpid());self.owned=False
        self.stream=self.path.with_name(self.path.name+'.lease').open('a+b')
        try:
            self.stream.seek(0)
            if os.name=='nt':
                import msvcrt
                msvcrt.locking(self.stream.fileno(),msvcrt.LK_NBLCK,1)
            else:
                import fcntl
                fcntl.flock(self.stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
            if self.path.exists():
                previous=self.path.read_text(encoding='ascii').strip()
                if not previous.isdigit() or process_alive(int(previous)):
                    raise FileExistsError('Live or unknown previous owner: '+str(self.path))
                self.path.unlink()
            with self.path.open('x',encoding='ascii') as out:
                out.write(self.owner);out.flush();os.fsync(out.fileno())
            self.owned=True
        except (OSError, UnicodeError, ValueError) as error:
            self.stream.close()
            raise FileExistsError('Resource busy or lock cannot be verified: '+str(self.path)) from error

    def close(self):
        try:
            if self.owned:
                if self.path.read_text(encoding='ascii').strip()!=self.owner:
                    raise RuntimeError('Lock ownership changed')
                self.path.unlink();self.owned=False
        finally:
            self.stream.close()  # Kernel releases lease even on hard exit.

    def __enter__(self):
        return self

    def __exit__(self,*_):
        self.close()

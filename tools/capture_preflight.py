"""Read-only process check for observed Windows F04/SlimeVR contention."""
from pathlib import PureWindowsPath
import sys


def windows_processes():
    if sys.platform!='win32':return []
    import ctypes
    from ctypes import wintypes as w

    class ProcessEntry(ctypes.Structure):
        _fields_=[('dwSize',w.DWORD),('cntUsage',w.DWORD),('th32ProcessID',w.DWORD),
                  ('th32DefaultHeapID',ctypes.c_size_t),('th32ModuleID',w.DWORD),
                  ('cntThreads',w.DWORD),('th32ParentProcessID',w.DWORD),
                  ('pcPriClassBase',w.LONG),('dwFlags',w.DWORD),('szExeFile',w.WCHAR*260)]
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.CreateToolhelp32Snapshot.argtypes=[w.DWORD,w.DWORD];kernel.CreateToolhelp32Snapshot.restype=w.HANDLE
    for name in ('Process32FirstW','Process32NextW'):
        fn=getattr(kernel,name);fn.argtypes=[w.HANDLE,ctypes.POINTER(ProcessEntry)];fn.restype=w.BOOL
    kernel.CloseHandle.argtypes=[w.HANDLE];kernel.CloseHandle.restype=w.BOOL
    kernel.OpenProcess.argtypes=[w.DWORD,w.BOOL,w.DWORD];kernel.OpenProcess.restype=w.HANDLE
    kernel.QueryFullProcessImageNameW.argtypes=[w.HANDLE,w.DWORD,w.LPWSTR,ctypes.POINTER(w.DWORD)]
    kernel.QueryFullProcessImageNameW.restype=w.BOOL
    snapshot=kernel.CreateToolhelp32Snapshot(2,0)
    if snapshot==ctypes.c_void_p(-1).value:raise ctypes.WinError(ctypes.get_last_error())
    rows=[]
    try:
        entry=ProcessEntry();entry.dwSize=ctypes.sizeof(entry)
        found=kernel.Process32FirstW(snapshot,ctypes.byref(entry))
        while found:
            name=entry.szExeFile;path=''
            if name.lower() in ('java.exe','javaw.exe'):
                handle=kernel.OpenProcess(0x1000,False,entry.th32ProcessID)
                if handle:
                    try:
                        size=w.DWORD(32768);buffer=ctypes.create_unicode_buffer(size.value)
                        if kernel.QueryFullProcessImageNameW(handle,0,buffer,ctypes.byref(size)):path=buffer.value
                    finally:kernel.CloseHandle(handle)
            rows.append((name,path))
            found=kernel.Process32NextW(snapshot,ctypes.byref(entry))
        if ctypes.get_last_error()!=18:raise ctypes.WinError(ctypes.get_last_error())  # ERROR_NO_MORE_FILES
    finally:kernel.CloseHandle(snapshot)
    return rows


def check_capture_conflicts():
    for name,path in windows_processes():
        executable=name.lower()
        bundled_server=(executable in ('java.exe','javaw.exe')
                        and any(part.lower()=='slimevr' for part in PureWindowsPath(path).parts))
        if executable in ('slimevr.exe','slimevr-server.exe') or bundled_server:
            raise ValueError('检测到 SlimeVR 仍在运行。当前接收器在 SlimeVR 同时运行时可能出现校准数据反压和缺口；'
                             '请先完全退出 SlimeVR（包括后台服务器），再点击录制。普通追踪时可照常使用。'
                             '本次尚未开始录制，也未向设备发送命令。')

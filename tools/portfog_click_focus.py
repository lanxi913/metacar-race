# -*- coding: utf-8 -*-
"""启动场景后，把窗口切前台并在画面中央真点一下鼠标，然后看时钟走不走。"""
import ctypes
import ctypes.wintypes as wt
import json
import socket
import time

user32 = ctypes.windll.user32


class RECT(ctypes.Structure):
    _fields_ = [("Left", ctypes.c_long), ("Top", ctypes.c_long),
                ("Right", ctypes.c_long), ("Bottom", ctypes.c_long)]


def find_scene_window():
    found = []

    @ctypes.WINFUNCTYPE(ctypes.c_bool, wt.HWND, wt.LPARAM)
    def cb(hwnd, lp):
        pid = wt.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
        buf = ctypes.create_unicode_buffer(256)
        user32.GetWindowTextW(hwnd, buf, 256)
        if "港口迷雾" in buf.value and user32.IsWindowVisible(hwnd):
            found.append((hwnd, buf.value))
        return True

    user32.EnumWindows(cb, 0)
    return found


def ask(p, timeout=4):
    s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
    s.settimeout(timeout)
    try:
        s.sendall((json.dumps(p) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            c = s.recv(65536)
            if not c:
                break
            buf += c
        return json.loads(buf.split(b"\n")[0].decode("utf-8", "replace"))
    finally:
        s.close()


wins = find_scene_window()
print("windows:", wins, flush=True)
if wins:
    hwnd = wins[0][0]
    user32.ShowWindow(hwnd, 3)          # SW_MAXIMIZE
    user32.SetForegroundWindow(hwnd)
    time.sleep(0.8)
    r = RECT()
    user32.GetWindowRect(hwnd, ctypes.byref(r))
    cx = (r.Left + r.Right) // 2
    cy = (r.Top + r.Bottom) // 2
    print(f"clicking at ({cx},{cy})", flush=True)
    user32.SetCursorPos(cx, cy)
    time.sleep(0.2)
    user32.mouse_event(0x0002, 0, 0, 0, 0)   # LEFTDOWN
    time.sleep(0.05)
    user32.mouse_event(0x0004, 0, 0, 0, 0)   # LEFTUP
    time.sleep(0.5)

for i in range(6):
    ev = ask({"command": "get_events"}).get("events") or []
    print(f"  t+{i*2}s events={len(ev)} last_t={round(ev[-1].get('elapsed_seconds', 0), 1) if ev else 0}",
          flush=True)
    time.sleep(2.0)

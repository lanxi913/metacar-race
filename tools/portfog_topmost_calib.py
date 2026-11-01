# -*- coding: utf-8 -*-
"""把场景窗口置顶，再测油门-速度映射（验证是不是帧率问题）。"""
from __future__ import annotations

import ctypes
import ctypes.wintypes as wt
import json
import socket
import time

user32 = ctypes.windll.user32
HWND_TOPMOST = -1
SWP_NOMOVE = 0x0002
SWP_NOSIZE = 0x0001
SWP_SHOWWINDOW = 0x0040


def find_scene():
    found = []

    @ctypes.WINFUNCTYPE(ctypes.c_bool, wt.HWND, wt.LPARAM)
    def cb(hwnd, lp):
        buf = ctypes.create_unicode_buffer(256)
        user32.GetWindowTextW(hwnd, buf, 256)
        if "港口迷雾" in buf.value:
            found.append(hwnd)
        return True

    user32.EnumWindows(cb, 0)
    return found


for h in find_scene():
    user32.SetWindowPos(h, HWND_TOPMOST, 0, 0, 0, 0,
                        SWP_NOMOVE | SWP_NOSIZE | SWP_SHOWWINDOW)
    user32.ShowWindow(h, 3)
    user32.SetForegroundWindow(h)
print("scene window pinned topmost", flush=True)
time.sleep(1.5)

s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
s.settimeout(6)
buf = b""


def call(p):
    global buf
    s.sendall((json.dumps(p) + "\n").encode())
    while b"\n" not in buf:
        c = s.recv(65536)
        if not c:
            raise ConnectionError
        buf += c
    line, rest = buf.split(b"\n", 1)
    buf = rest
    return json.loads(line.decode("utf-8", "replace"))


def ctrl(vid, throttle=0.0, brake=0.0, steering=0.0, gear=1):
    return call({"command": "set_vehicle_control", "vehicle_id": vid,
                 "vehicle_control": {"gear": gear, "throttle": throttle,
                                     "brake": brake, "steering": steering}})


def pos(vid):
    o = call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
    p = o.get("position") or {}
    return p.get("x", 0.0), p.get("y", 0.0)


for thr in (0.0, 0.3, 0.6, 1.0):
    a = pos("Car_D")
    t0 = time.time()
    while time.time() - t0 < 2.0:
        ctrl("Car_D", throttle=thr)
        time.sleep(0.1)
    b = pos("Car_D")
    d = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
    print(f"thr={thr:4.1f} moved {d:6.2f} m in 2.0 s => {d/2.0:5.2f} m/s", flush=True)
for _ in range(15):
    ctrl("Car_D", brake=1.0)
    time.sleep(0.1)
s.close()

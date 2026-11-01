# -*- coding: utf-8 -*-
"""找港口迷雾的相机通道：监听端口 + demo_screenshot + 各种帧命令。"""
from __future__ import annotations

import json
import os
import socket
import subprocess
import time


def ports_of_scene() -> list[int]:
    """SmartCar 进程监听的所有端口。"""
    out = []
    try:
        ps = subprocess.run(
            ["powershell.exe", "-NoProfile", "-Command",
             "(Get-Process -Name SmartCar -ErrorAction SilentlyContinue).Id"],
            capture_output=True, text=True, timeout=20)
        pids = [int(x) for x in ps.stdout.split() if x.strip().isdigit()]
        for pid in pids:
            ps2 = subprocess.run(
                ["powershell.exe", "-NoProfile", "-Command",
                 f"Get-NetTCPConnection -OwningProcess {pid} -State Listen -ErrorAction SilentlyContinue | "
                 f"Select-Object -ExpandProperty LocalPort"],
                capture_output=True, text=True, timeout=20)
            out += [int(x) for x in ps2.stdout.split() if x.strip().isdigit()]
    except Exception as e:
        print("port enum err:", e, flush=True)
    return sorted(set(out))


print("scene listening ports:", ports_of_scene(), flush=True)

s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
s.settimeout(10)
buf = b""


def call(p):
    global buf
    s.sendall((json.dumps(p) + "\n").encode())
    while b"\n" not in buf:
        c = s.recv(1 << 20)
        if not c:
            raise ConnectionError
        buf += c
    line, rest = buf.split(b"\n", 1)
    buf = rest
    return json.loads(line.decode("utf-8", "replace"))


def show(tag, r):
    keys = [k for k in r.keys() if r.get(k) not in (None, "", [], {}, 0, False)]
    print(f"-- {tag}: ok={r.get('ok')} msg={str(r.get('message'))[:70]}", flush=True)
    print(f"   keys: {keys[:18]}", flush=True)
    for k in ("screenshot_path", "map_path", "image", "frame", "data", "base64", "objects"):
        v = r.get(k)
        if v:
            sv = str(v)
            print(f"   {k} = {sv[:120]}", flush=True)


show("demo_screenshot", call({"command": "demo_screenshot"}))
time.sleep(2.0)
r = call({"command": "get_observation", "vehicle_id": "Car_C"})
show("obs after shot", r)
sp = r.get("screenshot_path")
if sp:
    print("   file exists:", os.path.exists(sp), "->", sp, flush=True)

for name in ("get_camera_frame", "get_frames", "get_camera", "camera_frame",
             "get_image", "capture", "get_cameras", "get_video_frame"):
    try:
        r = call({"command": name, "vehicle_id": "Car_C"})
        ok = r.get("ok")
        msg = str(r.get("message"))[:60]
        print(f"{name:18s} ok={ok} msg={msg}", flush=True)
    except Exception as e:
        print(f"{name:18s} ERR {e}", flush=True)
    time.sleep(0.2)
s.close()

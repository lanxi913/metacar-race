# -*- coding: utf-8 -*-
"""强力脱困：多次「长倒车 + 短暂前进」把车从边界拉回路面。"""
from __future__ import annotations

import json
import math
import socket
import time

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


def ctrl(vid, throttle=0.0, brake=0.0, steering=0.0, gear=2):
    call({"command": "set_vehicle_control", "vehicle_id": vid,
          "vehicle_control": {"gear": gear, "throttle": throttle,
                              "brake": brake, "steering": steering}})


def pos(vid):
    o = call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
    p = o.get("position") or {}
    return float(p.get("x", 0.0)), float(p.get("y", 0.0))


def run(vid, secs, thr, steer, gear):
    t0 = time.time()
    while time.time() - t0 < secs:
        ctrl(vid, throttle=thr, steering=steer, gear=gear)
        time.sleep(0.08)
    ctrl(vid, brake=1.0, gear=3)
    time.sleep(0.2)


for vid in ("Car_C", "Car_D"):
    a = pos(vid)
    print(f"{vid} start ({a[0]:.1f},{a[1]:.1f})", flush=True)
    for attempt in range(4):
        run(vid, 3.0, 0.55, 1.0 if attempt % 2 == 0 else -1.0, 4)   # 长倒车
        mid = pos(vid)
        run(vid, 1.5, 0.40, 0.0, 2)                                  # 短暂前进
        b = pos(vid)
        moved = math.hypot(b[0] - a[0], b[1] - a[1])
        print(f"   尝试{attempt+1}: 倒车->({mid[0]:.1f},{mid[1]:.1f}) 前进->({b[0]:.1f},{b[1]:.1f}) "
              f"总位移 {moved:.1f} m", flush=True)
        a = b
        if b[1] < 130:
            print(f"   {vid} 已回到路面范围 (y={b[1]:.1f})", flush=True)
            break
s.close()

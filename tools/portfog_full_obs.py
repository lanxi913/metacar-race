# -*- coding: utf-8 -*-
"""把 get_observation 的完整回包存盘，重点看 run_report / scene_static_data / planning_problem。"""
from __future__ import annotations

import json
import socket

OUT = r"C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race\docs\portfog_full_obs.json"

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


r = call({"command": "get_observation", "vehicle_id": "Car_C"})
json.dump(r, open(OUT, "w", encoding="utf-8"), ensure_ascii=False, indent=1)

for k in ("run_report", "scene_static_data", "planning_problem", "observation"):
    v = r.get(k)
    print(f"== {k} ==", flush=True)
    if isinstance(v, dict):
        print("   keys:", list(v.keys()), flush=True)
        for kk, vv in v.items():
            if vv not in (None, "", [], {}, 0):
                print(f"   {kk}: {str(vv)[:200]}", flush=True)
    else:
        print("  ", str(v)[:300], flush=True)
    print(flush=True)
s.close()
print("saved ->", OUT)

# -*- coding: utf-8 -*-
"""打印四辆车当前位置与任务进度。"""
import json
import socket

s = socket.create_connection(("127.0.0.1", 9100), timeout=3)
s.settimeout(8)
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


for v in ("Car_A", "Car_B", "Car_C", "Car_D"):
    o = call({"command": "get_observation", "vehicle_id": v}).get("observation") or {}
    p = o.get("position") or {}
    print(f"{v}: ({p.get('x', 0):7.1f},{p.get('y', 0):7.1f}) phase={o.get('phase')} "
          f"sig={o.get('verified_signal_count')}/{o.get('total_signal_count')} "
          f"dest={o.get('unlocked_destination_count')}/{o.get('total_destination_count')}")
sc = call({"command": "get_task"}).get("score") or {}
print("score:", {k: sc.get(k) for k in ("total_score", "signals_verified", "signals_total",
                                        "destinations_completed", "destinations_total",
                                        "penalties", "elapsed_seconds")})
s.close()

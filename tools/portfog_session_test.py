# -*- coding: utf-8 -*-
"""港口迷雾：长连接心跳驱动测试（判断"车不动"是不是连接被重置导致）。"""
from __future__ import annotations

import json
import socket
import time


class Session:
    def __init__(self, host="127.0.0.1", port=9100):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.settimeout(5)
        self.buf = b""

    def call(self, payload: dict) -> dict:
        self.sock.sendall((json.dumps(payload) + "\n").encode())
        while b"\n" not in self.buf:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("closed")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8", "replace"))

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


def main() -> int:
    s = None
    for _ in range(180):
        try:
            s = Session()
            break
        except Exception:
            time.sleep(1.0)
    if s is None:
        print("!! 9100 没起来", flush=True)
        return 2
    print("connected (persistent)", flush=True)
    st = s.call({"command": "get_task_state"}).get("task_state") or {}
    print("phase:", st.get("phase"), flush=True)

    def pos(vid="Car_C"):
        o = s.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        return p.get("x", 0.0), p.get("y", 0.0), o.get("phase")

    x0, y0, ph = pos()
    print(f"start ({x0:.2f},{y0:.2f}) phase={ph}", flush=True)

    for i in range(40):
        s.call({"command": "set_vehicle_control", "vehicle_id": "Car_C", "gear": 1,
                "throttle": 0.35, "brake": 0.0, "steering": 0.0})
        time.sleep(0.1)
        if i % 10 == 9:
            x, y, ph = pos()
            sc = s.call({"command": "get_task"}).get("score") or {}
            print(f"  t={i*0.1:4.1f}s ({x:7.2f},{y:7.2f}) d={((x-x0)**2+(y-y0)**2)**0.5:5.2f} "
                  f"elapsed={sc.get('elapsed_seconds')} max_spd={sc.get('maximum_speed_mps')}", flush=True)
    s.call({"command": "set_vehicle_control", "vehicle_id": "Car_C", "gear": 1,
            "throttle": 0.0, "brake": 1.0, "steering": 0.0})
    s.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

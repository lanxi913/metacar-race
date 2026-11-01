# -*- coding: utf-8 -*-
"""港口迷雾：先等 9100，再持续驱动 Car_C 60 秒（配合外部把窗口切到前台）。"""
from __future__ import annotations

import json
import socket
import time

VIDS = ["Car_A", "Car_B", "Car_C", "Car_D"]


class S:
    def __init__(self):
        self.s = None
        for _ in range(180):
            try:
                self.s = socket.create_connection(("127.0.0.1", 9100), timeout=5)
                self.s.settimeout(5)
                break
            except Exception:
                time.sleep(1.0)
        if self.s is None:
            raise SystemExit("9100 not up")
        self.buf = b""

    def call(self, p):
        self.s.sendall((json.dumps(p) + "\n").encode())
        while b"\n" not in self.buf:
            c = self.s.recv(65536)
            if not c:
                raise ConnectionError
            self.buf += c
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8", "replace"))


def main():
    s = S()
    print("connected", flush=True)
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < 60:
        s.call({"command": "set_vehicle_controls",
                "vehicle_controls": [{"vehicle_id": v, "gear": 1, "throttle": 0.30,
                                      "brake": 0.0, "steering": 0.0} for v in VIDS]})
        time.sleep(0.1)
        now = time.time()
        if now - last > 2.0:
            last = now
            sc = s.call({"command": "get_task"}).get("score") or {}
            o = s.call({"command": "get_observation", "vehicle_id": "Car_C"}).get("observation") or {}
            p = o.get("position") or {}
            print(f"[{now-t0:5.1f}s] Car_C=({p.get('x'):7.1f},{p.get('y'):7.1f}) "
                  f"elapsed={sc.get('elapsed_seconds')} max_spd={sc.get('maximum_speed_mps')} "
                  f"phase={o.get('phase')} score={sc.get('total_score')}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

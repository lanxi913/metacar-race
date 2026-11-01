# -*- coding: utf-8 -*-
"""试：批量接管四辆车 -> 看任务时钟是否开始走。"""
from __future__ import annotations

import json
import socket
import time


def conn():
    s = None
    for _ in range(180):
        try:
            s = socket.create_connection(("127.0.0.1", 9100), timeout=5)
            s.settimeout(5)
            return s
        except Exception:
            time.sleep(1.0)
    raise SystemExit("9100 not up")


class S:
    def __init__(self):
        self.s = conn()
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
    vids = ["Car_A", "Car_B", "Car_C", "Car_D"]

    # 1) 批量接管：四辆车都下发一次控制
    ctrls = [{"vehicle_id": v, "gear": 1, "throttle": 0.0, "brake": 1.0, "steering": 0.0}
             for v in vids]
    r = s.call({"command": "set_vehicle_controls", "vehicle_controls": ctrls})
    print("batch ->", r.get("ok"), r.get("message"), flush=True)

    for i in range(30):
        s.call({"command": "set_vehicle_controls",
                "vehicle_controls": [{"vehicle_id": v, "gear": 1, "throttle": 0.30,
                                      "brake": 0.0, "steering": 0.0} for v in vids]})
        time.sleep(0.1)
        if i % 10 == 9:
            sc = s.call({"command": "get_task"}).get("score") or {}
            o = s.call({"command": "get_observation", "vehicle_id": "Car_C"}).get("observation") or {}
            p = o.get("position") or {}
            print(f"  t={i*0.1:4.1f}s Car_C=({p.get('x'):.1f},{p.get('y'):.1f}) "
                  f"elapsed={sc.get('elapsed_seconds')} max_spd={sc.get('maximum_speed_mps')} "
                  f"phase={o.get('phase')}", flush=True)
    for v in vids:
        s.call({"command": "set_vehicle_control", "vehicle_id": v, "gear": 1,
                "throttle": 0.0, "brake": 1.0, "steering": 0.0})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

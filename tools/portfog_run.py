# -*- coding: utf-8 -*-
"""港口迷雾（PORT-FOG-01）驾驶入口。

协议：SmartCar 在本机 9100 端口开一个「换行分隔 JSON」服务，我们是客户端。
  {"command": "get_task"}                                   -> 任务定义（车辆/目标/动作）
  {"command": "get_task_state"}                             -> 任务状态（含物体模糊区域）
  {"command": "get_observation", "vehicle_id": "Car_C"}     -> 该车的局部观测
  {"command": "set_vehicle_control", "vehicle_id": ..., "gear": 1,
   "throttle": .., "brake": .., "steering": ..}             -> 单车上控制
  {"command": "set_vehicle_controls", "vehicle_controls": [...]} -> 批量控制
  {"command": "interact", "action": "scan|submit_key|report_observation|request_entry|release|deliver",
   "object_id": ..., "vehicle_id": ...}                     -> 任务交互
  {"command": "get_events"}                                 -> 事件日志

控制是「心跳」模式：要持续下发，否则会记 control_timeout。
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import time

HOST = "127.0.0.1"
PORT = 9100


class Link:
    """9100 上的 JSON 行客户端（每请求一条连接，简单可靠）。"""

    def __init__(self, host: str = HOST, port: int = PORT, timeout: float = 5.0) -> None:
        self.host, self.port, self.timeout = host, port, timeout

    def wait_ready(self, max_wait: float = 120.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < max_wait:
            try:
                r = self.ask({"command": "get_task"}, timeout=2.0)
                if r.get("ok"):
                    return True
            except Exception:
                pass
            time.sleep(1.0)
        return False

    def ask(self, payload: dict, timeout: float | None = None) -> dict:
        s = socket.create_connection((self.host, self.port), timeout=3)
        s.settimeout(timeout or self.timeout)
        try:
            s.sendall((json.dumps(payload) + "\n").encode())
            buf = b""
            while b"\n" not in buf:
                chunk = s.recv(65536)
                if not chunk:
                    break
                buf += chunk
            line = buf.split(b"\n")[0].decode("utf-8", "replace")
            return json.loads(line) if line.strip().startswith("{") else {"raw": line}
        finally:
            s.close()

    # 便捷封装
    def task(self) -> dict:
        return self.ask({"command": "get_task"}).get("task") or {}

    def state(self) -> dict:
        return self.ask({"command": "get_task_state"}).get("task_state") or {}

    def obs(self, vid: str) -> dict:
        return self.ask({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}

    def score(self) -> dict:
        return self.ask({"command": "get_task"}).get("score") or {}

    def ctrl(self, vid: str, throttle: float = 0.0, brake: float = 0.0,
             steering: float = 0.0, gear: int = 1) -> dict:
        return self.ask({"command": "set_vehicle_control", "vehicle_id": vid, "gear": gear,
                         "throttle": throttle, "brake": brake, "steering": steering})

    def interact(self, action: str, object_id: str, vid: str, **extra) -> dict:
        payload = {"command": "interact", "action": action, "object_id": object_id,
                   "vehicle_id": vid}
        payload.update(extra)
        return self.ask(payload)


def wrap180(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def main() -> int:
    ap = argparse.ArgumentParser(description="port fog driver")
    ap.add_argument("--seconds", type=float, default=90.0, help="跑多久（秒）")
    ap.add_argument("--vehicle", default="Car_C")
    ap.add_argument("--throttle", type=float, default=0.35)
    args = ap.parse_args()

    link = Link()
    print("waiting for scenario server on 9100 ...", flush=True)
    if not link.wait_ready(180.0):
        print("!! 场景服务没起来", flush=True)
        return 2
    print("connected", flush=True)

    t = link.task()
    print("task:", t.get("scenario_id"), t.get("control_mode"), flush=True)
    for o in t.get("objectives") or []:
        print(f"  obj {o.get('objective_id')}: {o.get('state')}", flush=True)

    vid = args.vehicle
    o0 = link.obs(vid)
    p0 = o0.get("position") or {}
    print(f"start {vid} at ({p0.get('x'):.1f},{p0.get('y'):.1f}) phase={o0.get('phase')}", flush=True)

    t0 = time.time()
    last_report = 0.0
    moved = 0.0
    prev = (p0.get("x", 0.0), p0.get("y", 0.0))
    try:
        while time.time() - t0 < args.seconds:
            link.ctrl(vid, throttle=args.throttle, brake=0.0, steering=0.0, gear=1)
            o = link.obs(vid)
            p = o.get("position") or {}
            x, y = p.get("x", 0.0), p.get("y", 0.0)
            moved += math.hypot(x - prev[0], y - prev[1])
            prev = (x, y)
            now = time.time()
            if now - last_report > 2.0:
                last_report = now
                sc = link.score()
                print(f"[{now - t0:5.1f}s] ({x:7.1f},{y:7.1f}) moved={moved:6.1f} "
                      f"phase={o.get('phase')} signals={o.get('verified_signal_count')}/"
                      f"{o.get('total_signal_count')} score={sc.get('total_score')}", flush=True)
    finally:
        for v in ("Car_A", "Car_B", "Car_C", "Car_D"):
            try:
                link.ctrl(v, throttle=0.0, brake=1.0, steering=0.0, gear=1)
            except Exception:
                pass
    print("done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

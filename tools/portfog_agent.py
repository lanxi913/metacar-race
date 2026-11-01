# -*- coding: utf-8 -*-
"""港口迷雾 agent v4：稳定控制（2 m 基线航向 / 增益 60 / 20 Hz / 脱困）+ 按坐标导航 +
记录场景自带视觉（visible_objects）。

标定（实测）：
  gear 0=Waiting 1=蠕动 2=行驶 3=驻车 4=倒车
  steering +1 左 / -1 右
  控制必须嵌套在 vehicle_control 里（平铺会被丢弃）
  rotation_y 不是车身航向，航向用位移 atan2 算
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import time


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class Agent:
    def __init__(self) -> None:
        self.sock = None
        self.buf = b""
        self.pts: dict[str, list] = {}
        self.head: dict[str, float] = {}
        for _ in range(120):
            try:
                self.sock = socket.create_connection(("127.0.0.1", 9100), timeout=5)
                self.sock.settimeout(10)
                break
            except Exception:
                time.sleep(1.0)
        if self.sock is None:
            raise SystemExit("9100 unreachable")

    def call(self, p: dict) -> dict:
        self.sock.sendall((json.dumps(p) + "\n").encode())
        while b"\n" not in self.buf:
            chunk = self.sock.recv(1 << 20)
            if not chunk:
                raise ConnectionError("closed")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8", "replace"))

    def ctrl(self, vid: str, throttle: float = 0.0, brake: float = 0.0,
             steering: float = 0.0, gear: int = 2) -> None:
        self.call({"command": "set_vehicle_control", "vehicle_id": vid,
                   "vehicle_control": {"gear": gear, "throttle": throttle,
                                       "brake": brake, "steering": steering}})

    def obs(self, vid: str) -> tuple[float, float, dict]:
        o = self.call({"command": "get_observation", "vehicle_id": vid}).get("observation") or {}
        p = o.get("position") or {}
        x, y = float(p.get("x", 0.0)), float(p.get("y", 0.0))
        pl = self.pts.setdefault(vid, [])
        pl.append((x, y))
        if len(pl) > 40:
            pl.pop(0)
        if len(pl) >= 2:
            for bx, by in reversed(pl[:-1]):
                if math.hypot(x - bx, y - by) > 2.0:
                    h = math.degrees(math.atan2(y - by, x - bx))
                    prev = self.head.get(vid)
                    self.head[vid] = h if prev is None else prev + 0.7 * wrap180(h - prev)
                    break
        return x, y, o

    def heading(self, vid: str) -> float | None:
        return self.head.get(vid)

    def score(self) -> dict:
        return self.call({"command": "get_task"}).get("score") or {}

    def interact(self, action: str, object_id: str, vid: str) -> dict:
        return self.call({"command": "interact", "action": action,
                          "object_id": object_id, "vehicle_id": vid})

    def unstick(self, vid: str, secs: float = 1.6) -> None:
        t0 = time.time()
        while time.time() - t0 < secs:
            self.ctrl(vid, throttle=0.5, steering=-0.8, gear=4)
            time.sleep(0.08)
        self.ctrl(vid, brake=1.0, gear=3)
        time.sleep(0.2)

    def drive_to(self, vid: str, tx: float, ty: float, tol: float = 5.0,
                 thr: float = 0.32, deadline: float = 90.0, on_step=None) -> bool:
        t0 = time.time()
        best_d = 1e9
        stall_t = time.time()
        while time.time() - t0 < deadline:
            x, y, o = self.obs(vid)
            d = math.hypot(tx - x, ty - y)
            if on_step:
                on_step(x, y, o)
            if d < best_d - 0.5:
                best_d, stall_t = d, time.time()
            if d <= tol:
                self.ctrl(vid, brake=1.0, gear=3)
                return True
            if time.time() - stall_t > 3.0:
                print(f"    !! 卡住 ({x:.1f},{y:.1f}) d={d:.1f}，倒车脱困", flush=True)
                self.unstick(vid)
                stall_t = time.time()
                best_d = 1e9
                continue
            h = self.heading(vid)
            if h is None:
                self.ctrl(vid, throttle=0.25, steering=0.0, gear=2)
                time.sleep(0.05)
                continue
            want = math.degrees(math.atan2(ty - y, tx - x))
            err = wrap180(want - h)
            steer = max(-0.7, min(0.7, err / 60.0))
            thr_now = thr if abs(err) < 45 else 0.20
            if d < 12:
                thr_now = min(thr_now, 0.26)
            self.ctrl(vid, throttle=thr_now, steering=steer, gear=2)
            time.sleep(0.05)
        self.ctrl(vid, brake=1.0, gear=3)
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="Car_C")
    ap.add_argument("--tx", type=float, required=True)
    ap.add_argument("--ty", type=float, required=True)
    ap.add_argument("--deadline", type=float, default=150.0)
    args = ap.parse_args()

    a = Agent()
    vid = args.vehicle
    x, y, o = a.obs(vid)
    print(f"{vid} 从 ({x:.1f},{y:.1f}) 开往 ({args.tx},{args.ty})", flush=True)

    seen: dict[str, dict] = {}
    last_log = [0.0]

    def on_step(x, y, o):
        objs = o.get("visible_objects") or []
        for ob in objs:
            oid = ob.get("object_id") or str(ob)[:20]
            if oid not in seen:
                seen[oid] = ob
                print(f"  [发现] {json.dumps(ob, ensure_ascii=False)[:240]}", flush=True)
        now = time.time()
        if now - last_log[0] > 3.0:
            last_log[0] = now
            print(f"    ({x:7.1f},{y:7.1f}) 可见={len(objs)} "
                  f"sig={o.get('verified_signal_count')}/{o.get('total_signal_count')}", flush=True)

    ok = a.drive_to(vid, args.tx, args.ty, deadline=args.deadline, on_step=on_step)
    x, y, o = a.obs(vid)
    print(f"\n到达 ({x:.1f},{y:.1f}) ok={ok}", flush=True)
    print("score:", json.dumps(a.score(), ensure_ascii=False)[:300], flush=True)
    print("累计发现:", list(seen.keys()), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

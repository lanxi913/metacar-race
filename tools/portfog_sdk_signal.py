# -*- coding: utf-8 -*-
"""用官方 SDK 接近 Signal_A，7 m 内打印全字段并尝试 submit_key。"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar.models import GearMode, VehicleControl  # noqa: E402
from metacar_multiagent import ScenarioTaskAPI  # noqa: E402


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def ctrl(api, vid, thr=0.0, brk=0.0, st=0.0, gear=2):
    gear_map = {2: GearMode.DRIVE, 3: GearMode.PARKING, 4: GearMode.REVERSE}
    api.set_vehicle_control(vid, VehicleControl(
        throttle=thr, brake=brk, steering=st, gear=gear_map.get(gear, GearMode.DRIVE)))


def heading(api, vid):
    return (float(api.get_vehicle_state(vid).pose.ori_z) + 180.0) % 360.0


def pos(api, vid):
    p = api.get_vehicle_state(vid).pose
    return float(p.pos_x), float(p.pos_y)


def orient(api, vid, target, tol=12.0, rounds=8):
    for _ in range(rounds):
        h = heading(api, vid)
        err = wrap180(target - h)
        print(f"    对准 航向={h:6.1f} 误差={err:+6.1f}", flush=True)
        if abs(err) <= tol:
            return True
        secs = min(1.5, max(0.3, abs(err) / 50.0))
        st = 1.0 if err > 0 else -1.0
        t0 = time.time()
        while time.time() - t0 < secs:
            ctrl(api, vid, thr=0.45, st=st, gear=4)
            time.sleep(0.07)
        ctrl(api, vid, gear=3)
        time.sleep(0.15)
    return False


def main():
    with ScenarioTaskAPI() as api:
        vid = "Car_C"
        task = api.get_task()
        sig = next((o for o in task.shared_task_objects if o.object_type == "signal"), None)
        print("当前信号:", sig.object_id, f"({sig.position.x:.1f},{sig.position.y:.1f})",
              "精度:", sig.information_level, flush=True)

        tx, ty = float(sig.position.x), float(sig.position.y)
        for step in range(18):
            x, y = pos(api, vid)
            d = math.hypot(tx - x, ty - y)
            h = heading(api, vid)
            obs = api.get_observation(vid)
            vis = [o.object_id for o in obs.visible_objects]
            print(f"[{step}] ({x:.1f},{y:.1f}) h={h:6.1f} d={d:5.1f} 可见={vis}", flush=True)

            tob = next((o for o in obs.visible_objects if o.object_id == sig.object_id), None)
            if tob is not None and d <= 8.0:
                print("=== 近距离对象全字段 ===", flush=True)
                print(json.dumps(tob.model_dump(), ensure_ascii=False, indent=1), flush=True)
                for payload in ({"key": tob.observation_token}, None):
                    try:
                        r = api.interact(vid, sig.object_id, "submit_key", payload)
                        print(f"submit_key({payload}) -> granted={r.granted} msg={r.message}",
                              flush=True)
                    except Exception as e:
                        print("submit err:", e, flush=True)
                break

            want = math.degrees(math.atan2(ty - y, tx - x))
            err = wrap180(want - h)
            if abs(err) > 25:
                orient(api, vid, want)
                continue
            t0 = time.time()
            while time.time() - t0 < 0.8:
                ctrl(api, vid, thr=0.20, st=max(-0.5, min(0.5, err / 60.0)), gear=2)
                time.sleep(0.06)
            ctrl(api, vid, brk=1.0, gear=3)

        x, y = pos(api, vid)
        obs = api.get_observation(vid)
        print(f"结束 ({x:.1f},{y:.1f}) 偏差={math.hypot(tx-x, ty-y):.1f} "
              f"sig={obs.verified_signal_count}/{obs.total_signal_count}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

# -*- coding: utf-8 -*-
"""VLA · 停车区违规检测：纯几何求解（不依赖视觉）。

判分：成功率 = 2×报对 /（实际违规数 + 提交数），即 F1；最终 = 成功率×1000。
思路：静态数据里有 regions（类型 0 正常停车区 / 1 禁停区 / 2 功能区）和 buildings；
      每帧 obstacles 里带贴纸（sticker）的障碍物。落在禁停区（RESTRICTED_PARKING）
      内的贴纸就是违规，把它们的 id 报上去。

用法：先跑脚本，再点「停车区违规检测任务」。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402
from metacar.models import (  # noqa: E402
    ParkingResult,
    VLAExtensionOutput,
)

RESTRICTED = 1
NORMAL = 0


def point_in_region(px, py, r) -> bool:
    """区域按中心 + 朝向 + 长宽定义（长方形），把点转到区域坐标系再判范围。"""
    th = math.radians(-float(r.ori_z))
    dx, dy = px - float(r.pos_x), py - float(r.pos_y)
    u = dx * math.cos(th) - dy * math.sin(th)
    v = dx * math.sin(th) + dy * math.cos(th)
    return abs(u) <= float(r.length) / 2.0 and abs(v) <= float(r.width) / 2.0


def main() -> int:
    ap = argparse.ArgumentParser(description="VLA 停车区违规检测")
    ap.add_argument("--max-seconds", type=float, default=95.0)
    ap.add_argument("--dry-run", action="store_true", help="只打印不提交")
    ap.add_argument("--drive", action="store_true",
                    help="边巡检边看：直行（0.3 油门）并每秒打印障碍物统计，"
                         "用来确认贴纸/车辆是否只有在靠近时才出现在 obstacles 里")
    args = ap.parse_args()

    print("waiting for VLA scene ...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    ext = static.vla_extension
    print("sub_scenes:", [s.name for s in static.sub_scenes], flush=True)
    if ext is None:
        print("!! 不是 VLA 场景", flush=True)
        return 2

    print(f"buildings={len(ext.buildings)} regions={len(ext.regions)}", flush=True)
    for r in ext.regions:
        print(f"  region id={r.id} type={r.type} pos=({r.pos_x:.2f},{r.pos_y:.2f}) "
              f"ori_z={r.ori_z:.1f} size={r.length:.2f}x{r.width:.2f}", flush=True)

    reported = False
    frame = 0
    t0 = None
    for msg, frames in api.main_loop():
        frame += 1
        st = msg.scene_status
        if t0 is None:
            t0 = st.used_time
        pose = msg.pose_gnss
        if args.drive:
            # 巡检模式：直行看 obstacles 会不会随靠近而出现
            api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=0.30,
                                                  brake=0.0, steering=0.0))
            if frame % 30 == 1:
                from collections import Counter
                cnt = Counter(getattr(getattr(o, "type", None), "name", str(o.type))
                              for o in msg.obstacles)
                print(f"[{st.used_time:5.1f}s] pos=({pose.pos_x:.2f},{pose.pos_y:.2f}) "
                      f"v={float(msg.main_vehicle.speed):.2f} obstacles={len(msg.obstacles)} {dict(cnt)}",
                      flush=True)
                for o in list(msg.obstacles)[:12]:
                    print(f"      id={o.id} {getattr(o.type, 'name', o.type)} "
                          f"({o.pos_x:.1f},{o.pos_y:.1f}) extra={getattr(o, 'extra_info', '')!r}",
                          flush=True)
        else:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))

        if frame == 1:
            obs = list(msg.obstacles)
            print(f"\nobstacles: {len(obs)}", flush=True)
            for i, o in enumerate(obs[:40]):
                tn = getattr(getattr(o, "type", None), "name", str(getattr(o, "type", "?")))
                print(f"  #{i} id={o.id} type={tn} pos=({o.pos_x:.2f},{o.pos_y:.2f}) "
                      f"extra={getattr(o, 'extra_info', '') or ''}", flush=True)

        # 计算违规贴纸
        violating = []
        for o in msg.obstacles:
            for r in ext.regions:
                if int(getattr(r.type, "value", r.type)) != RESTRICTED:
                    continue
                if point_in_region(float(o.pos_x), float(o.pos_y), r):
                    violating.append(str(o.id))
                    break
        violating = sorted(set(violating))

        if frame % 30 == 1 or not reported:
            print(f"[{st.used_time:5.1f}s] pos=({pose.pos_x:.2f},{pose.pos_y:.2f}) "
                  f"violating={violating}", flush=True)

        if violating and not args.dry_run:
            out = VLAExtensionOutput(parking_result=ParkingResult(violating_sticker_ids=violating))
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0), out)
            reported = True

        if st.used_time > args.max_seconds and reported:
            break

    print("SUMMARY " + json.dumps({"frames": frame, "violating": violating}, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

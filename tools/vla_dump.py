# -*- coding: utf-8 -*-
"""VLA 探针：把 get_scene_static_data() 和第一帧消息整份 dump 成 JSON。

用途：确认"停车位/停车区域/任务文本"这些系统提供的信息到底在哪个字段里，
避免靠截图猜坐标。
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import cv2

from metacar import GearMode, SceneAPI, VehicleControl

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))


def dump(obj, depth=0):
    try:
        return obj.model_dump()
    except Exception:
        return repr(obj)


def main():
    out_dir = _REPO / "tmp"
    out_dir.mkdir(parents=True, exist_ok=True)

    print("等待场景连接...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print("已连接", flush=True)

    payload = {
        "sub_scenes": [s.model_dump() for s in static.sub_scenes],
        "route_len": len(static.route),
        "roads_len": len(static.roads),
        "vla_extension": dump(static.vla_extension) if static.vla_extension else None,
    }
    (out_dir / "vla_static.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, default=str), encoding="utf-8")

    vla = static.vla_extension
    if vla is not None:
        print(f"buildings={len(vla.buildings)} regions={len(vla.regions)}", flush=True)
        for r in vla.regions:
            print("  region:", r.model_dump(), flush=True)

    t0 = time.time()
    last = -1.0
    for msg, frames in api.main_loop():
        now = time.time() - t0
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        if frames and len(frames) >= 4:
            for i, fr in enumerate(frames):
                cv2.imwrite(str(out_dir / f"vla_cam{i}.png"), fr.frame)
        if now - last >= 3.0:
            last = now
            scene = msg.scene_status.model_dump()
            obs = [o.model_dump() for o in msg.obstacles]
            (out_dir / "vla_msg.json").write_text(
                json.dumps({"scene_status": scene,
                            "pose": msg.pose_gnss.model_dump(),
                            "n_obstacles": len(obs),
                            "obstacles": obs[:40]},
                           ensure_ascii=False, indent=2, default=str), encoding="utf-8")
            print(f"[{now:5.1f}s] scene_status={scene} "
                  f"障碍物={len(obs)} 类型={sorted({str(o['type']) for o in obs})}", flush=True)
            print(f"    位姿 ({msg.pose_gnss.pos_x:.1f},{msg.pose_gnss.pos_y:.1f}) "
                  f"yaw={msg.pose_gnss.ori_z:.1f}", flush=True)
        if now > 6.0:
            break

    print("dump 完成：tmp/vla_static.json / tmp/vla_msg.json", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

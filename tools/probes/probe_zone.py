"""VLA 功能区合规审查 —— 采数探针 v2（车基本不动，主要抓帧与障碍物时间线）。

与 v1 的差别（v1 的教训都在这）：
  * v1 用 `cv2.imwrite` 存图，路径含中文（算法邀請賽）→ OpenCV 在 Windows 上**静默失败**，
    一张图都没落盘。这里改成 `cv2.imencode` + `Path.write_bytes`。
  * v1 只在首帧 dump 一次障碍物（那时场景还在加载，列表是空的）→ 改成每秒比对，
    有变化就整表打印，并保留完整时间线。
  * v1 在运行中逐帧跑 Windows OCR（每次要起一次 PowerShell，2~3 s），既慢又会把场景
    加载期的黑帧读成垃圾 → 改成**只抓帧**，OCR 事后离线跑（tools\\ocr_frames.py）。
  * 场景加载期约 10~15 s（wall 5.4 s 时 used_time 才 0.078，wall 15 s 时跳到 5.0），
    所以默认观察 45 s，并在第 22 s 起以极小油门直行 12 s，把入口规则牌与场内车辆拉近。

全程只在 `--explore` 窗口内给 throttle=0.03（约 1 m/s 爬行），其余时间 gear=PARKING、
brake=1.0。不提交任何结果。
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
from metacar import GearMode, RegionType, SceneAPI, VehicleControl

OUT_DEFAULT = Path(r"D:\download3\算法邀請賽\accel_window_probe\_vla_ref\zone_probe2")


def save_png(path: Path, img) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        print(f"!! imencode 失败: {path}", flush=True)
        return False
    path.write_bytes(buf.tobytes())
    return True


def _region_name(t) -> str:
    try:
        return RegionType(t).name
    except Exception:
        return f"?{t}"


def _type_name(t) -> str:
    name = getattr(t, "name", None)
    return name if name else f"?{getattr(t, 'value', t)}"


def _obstacle_rows(obstacles) -> list[dict]:
    rows = []
    for o in obstacles or []:
        rows.append({
            "id": str(o.id),
            "type": _type_name(getattr(o, "type", "?")),
            "x": round(float(o.pos_x), 2),
            "y": round(float(o.pos_y), 2),
            "ori": round(float(getattr(o, "ori_z", 0.0)), 1),
            "L": round(float(getattr(o, "length", 0.0)), 2),
            "W": round(float(getattr(o, "width", 0.0)), 2),
            "extra": str(getattr(o, "extra_info", None)),
        })
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--explore-at", type=float, default=22.0, help="从第几秒开始爬行")
    ap.add_argument("--explore", type=float, default=12.0, help="爬行持续多少秒（0=不动）")
    ap.add_argument("--explore-throttle", type=float, default=0.03)
    ap.add_argument("--shot-every", type=float, default=2.0)
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    ap.add_argument("--self-check", action="store_true")
    args = ap.parse_args()

    out_dir = Path(args.out)
    print(f"输出目录: {out_dir}", flush=True)
    if args.self_check:
        print("self-check OK", flush=True)
        return 0
    out_dir.mkdir(parents=True, exist_ok=True)

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()

    report: dict = {"static": {}, "regions": [], "obstacle_timeline": [], "frames": [],
                    "poses": []}
    print("=== SceneStaticData 全字段 ===", flush=True)
    print(f"  类型: {type(static).__module__}.{type(static).__name__}", flush=True)
    for k, v in static:
        if k == "vla_extension":
            nb = len(v.buildings) if v is not None else None
            nr = len(v.regions) if v is not None else None
            print(f"  {k}: {type(v).__name__} buildings={nb} regions={nr}", flush=True)
            report["static"][k] = f"buildings={nb} regions={nr}"
            continue
        if k == "route":
            print(f"  {k}: len={len(v) if v is not None else 0}", flush=True)
            report["static"][k] = f"len={len(v) if v is not None else 0}"
            continue
        try:
            n = len(v)
        except TypeError:
            n = None
        if n is not None:
            print(f"  {k}: len={n} {str(v)[:120]}", flush=True)
        else:
            print(f"  {k}: {v!r}"[:200], flush=True)
        report["static"][k] = str(v)[:400]

    vla = getattr(static, "vla_extension", None)
    if vla is None:
        print("!! vla_extension 为 None", flush=True)
    else:
        for r in vla.regions:
            row = {k: str(getattr(r, k)) for k in ["id", "type", "pos_x", "pos_y", "ori_z",
                                                   "length", "width"]}
            row["type_name"] = _region_name(getattr(r, "type", None))
            report["regions"].append(row)
            print(f"  region {row['type_name']} id={row['id']} "
                  f"({row['pos_x']},{row['pos_y']}) {row['length']}x{row['width']} "
                  f"ori={row['ori_z']}", flush=True)

    t0 = time.time()
    prev_log = -1e9
    prev_shot = -1e9
    last_rows: list[dict] = []
    n_shot = 0

    for msg, frames in api.main_loop():
        now = time.time() - t0
        pose = msg.pose_gnss
        speed = abs(msg.main_vehicle.speed)
        st = msg.scene_status
        rows = _obstacle_rows(msg.obstacles)

        if rows != last_rows:
            last_rows = rows
            print(f"[{now:5.1f}s] 障碍物 {len(rows)} 个", flush=True)
            for r in rows:
                print(f"    {r}", flush=True)
            report["obstacle_timeline"].append({"t": round(now, 1),
                                               "used_time": getattr(st, "used_time", None),
                                               "obstacles": rows})

        if frames and now - prev_shot >= args.shot_every:
            prev_shot = now
            for cam, fr in zip(msg.sensor.ego_rgb_cams, frames):
                if fr.frame is None:
                    continue
                if save_png(out_dir / "f" / f"t{now:05.1f}_cam{cam.id}.png", fr.frame):
                    n_shot += 1

        if now - prev_log >= 1.0:
            prev_log = now
            print(f"[{now:5.1f}s] used={getattr(st, 'used_time', 0):7.3f} "
                  f"limit={getattr(st, 'time_limit', 0)} "
                  f"({pose.pos_x:7.2f},{pose.pos_y:7.2f}) yaw={pose.ori_z:6.1f} "
                  f"v={speed:5.2f} obs={len(rows)}", flush=True)
            report["poses"].append({"t": round(now, 1), "x": round(float(pose.pos_x), 2),
                                    "y": round(float(pose.pos_y), 2),
                                    "yaw": round(float(pose.ori_z), 1),
                                    "v": round(float(speed), 2),
                                    "used_time": getattr(st, "used_time", None),
                                    "n_obs": len(rows)})

        exploring = args.explore > 0 and args.explore_at <= now < args.explore_at + args.explore
        if exploring:
            api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE,
                                                   throttle=args.explore_throttle,
                                                   brake=0.0, steering=0.0))
        else:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))

        if now >= args.seconds:
            print(f"=== 观察结束（{now:.1f}s，存图 {n_shot} 张）===", flush=True)
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except OSError:
        pass

    js = out_dir / "report.json"
    js.write_text(json.dumps(report, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    print(f"报告: {js}  帧: {out_dir / 'f'}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

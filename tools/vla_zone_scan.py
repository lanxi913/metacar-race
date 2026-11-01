# -*- coding: utf-8 -*-
"""VLA · 停车区违规检测（邀请赛版）：巡合法区 → OCR 读车身贴纸号 → 补集提交。

思路（用户 2026-11-01 10:17 定）：
  1. 场景共 10 辆车，其中 4 辆停在**合法区**（NORMAL_PARKING region）；
  2. 开车沿合法区走一遍，用 4 路相机 OCR 读车身贴纸编号（实测形如 `B-07`，白字，2~6 m 内清晰）；
  3. 提交 = 全集（同前缀 01~10）减去读到的合法车号 → 剩下的就是违停的 6 辆。

关键点：
  - 出生点变了，所以路线不写死：从 `vla_extension.regions` 里取 NORMAL_PARKING 的中心/长宽，
    自己算一条"贴着合法区前沿、从东到西"的扫描线；
  - 每次 OCR 只认一个相机（rapidocr 一次 0.3~0.8 s，4 路全认会拖死控制循环），轮流认；
  - 读到的号做归一化（`B 07` / `B—7` / `B-7` → `B-07`），多轮投票。
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402
from metacar.models import (FunctionZoneResult, FunctionZoneViolation,
                            ParkingResult, VLAExtensionOutput)  # noqa: E402
from metacar_race.perception.ocr_engine import read_text  # noqa: E402

NORMAL = 0
RESTRICTED = 1
FUNCTION = 2
STICKER_RE = re.compile(r"([A-Za-z])\s*[-–—~_ ]?\s*(\d{1,2})")


def make_out(args, ids):
    if args.region == "function":
        return VLAExtensionOutput(function_zone_result=FunctionZoneResult(
            violations=[FunctionZoneViolation(rule_code=str(args.rule), sticker_ids=list(ids))]))
    return VLAExtensionOutput(parking_result=ParkingResult(violating_sticker_ids=list(ids)))


def region_center(r):
    return float(r.pos_x), float(r.pos_y)


def point_in_region(px, py, r) -> bool:
    th = math.radians(-float(r.ori_z))
    dx, dy = px - float(r.pos_x), py - float(r.pos_y)
    u = dx * math.cos(th) - dy * math.sin(th)
    v = dx * math.sin(th) + dy * math.cos(th)
    return abs(u) <= float(r.length) / 2.0 and abs(v) <= float(r.width) / 2.0


def norm_id(letter: str, num: str) -> str:
    return f"{letter.upper()}-{int(num):02d}"


def main() -> int:
    ap = argparse.ArgumentParser(description="VLA 停车区违规检测（补集法）")
    ap.add_argument("--max-seconds", type=float, default=95.0)
    ap.add_argument("--ocr-every", type=int, default=4, help="每几帧认一次（轮流 4 路相机）")
    ap.add_argument("--passes", type=int, default=2, help="沿合法区往返几趟")
    ap.add_argument("--dry-run", action="store_true", help="只打印不提交")
    ap.add_argument("--shot", default="", help="前缀，存几帧看效果")
    ap.add_argument("--region", default="normal", choices=("normal", "function"),
                    help="巡哪个区：normal=停车区合法区，function=功能区")
    ap.add_argument("--rule", default="R01", help="功能区提交用的规则码")
    args = ap.parse_args()

    print("连接仿真场景...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    ext = static.vla_extension
    if ext is None:
        print("!! 不是 VLA 场景", flush=True)
        return 2
    regs = list(ext.regions)
    want = FUNCTION if args.region == "function" else NORMAL
    normal = [r for r in regs if int(getattr(r.type, "value", r.type)) == want]
    print(f"regions={len(regs)} normal={len(normal)}", flush=True)
    for r in regs:
        print(f"   region {r.id} type={r.type} ({r.pos_x:.1f},{r.pos_y:.1f}) "
              f"{r.length:.1f}x{r.width:.1f} ori={r.ori_z:.0f}", flush=True)
    if not normal:
        print("!! 没有 NORMAL_PARKING 区域", flush=True)
        return 3
    r = normal[0]
    cx, cy = region_center(r)
    th = math.radians(-float(r.ori_z))
    ax, ay = math.cos(th), math.sin(th)          # 区域长轴
    bx, by = -math.sin(th), math.cos(th)         # 区域短轴
    half_len = float(r.length) / 2.0
    half_wid = float(r.width) / 2.0
    # 扫描线：沿长轴走，短轴方向让出半个地区宽度 + 6 m（车走在车位前面看侧脸）
    off = half_wid + 4.5
    x0, y0 = cx - ax * (half_len + 6.0) - bx * off, cy - ay * (half_len + 6.0) - by * off
    x1, y1 = cx + ax * (half_len + 6.0) - bx * off, cy + ay * (half_len + 6.0) - by * off
    print(f"合法区中心 ({cx:.1f},{cy:.1f})，扫描线 ({x0:.1f},{y0:.1f}) → ({x1:.1f},{y1:.1f})",
          flush=True)

    # 路线规划（用户 11:00：**别进违规区和合法区**，进去就撞车；在外侧绕一圈读合法区的车）。
    # 两个矩形 x 都是 11.2~41.2（合法 y 40.8~46.8、违规 y 6.9~36.9），所以外侧矩形环线是：
    #   南边 y = 违规区南界 - 6  →  东边 x = 合法区东界 + 7  →  北边 y = 合法区北界 + 4  →  西边
    # 全程离两个区都有 ≥4 m，不进门也不压线；北边那条腿正对合法区那一排，是读牌的主扫描段。
    restr = [q for q in regs if int(getattr(q.type, "value", q.type)) == RESTRICTED]
    if restr:
        rr = restr[0]
        south_y = float(rr.pos_y) - float(rr.width) / 2.0 - 6.0
        east_x = cx + half_len + 7.0
        north_y = cy + half_wid + 4.0
        west_x = cx - half_len - 7.0
        waypoints = [(east_x, south_y),      # 南侧走廊往东（y≈0.9，在违规区南边）
                     (east_x, north_y),      # 东侧立柱往北（x≈48.2，在两区东边）
                     (west_x, north_y)]      # 北侧走廊往西（y≈50.8，在合法区北边）★ 读牌段
        if args.passes >= 2:
            waypoints += [(east_x, north_y), (west_x, north_y)]   # 再来回一趟
    else:
        waypoints = [(x0, y0), (x1, y1)]
    print("waypoints = " + str([(round(a,1), round(b,1)) for a, b in waypoints]), flush=True)
    wp_i = 0
    votes: dict[str, int] = {}
    last_ocr = -10
    ocr_i = 0
    stuck_t = 0.0
    back_until = 0.0
    frame = 0
    t0 = time.time()
    submitted: list[str] | None = None
    shot_done = set()

    for msg, frames in api.main_loop():
        frame += 1
        pose = msg.pose_gnss
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        now = time.time() - t0
        used = float(msg.scene_status.used_time or 0.0)

        # ---- 轮巡合法区 ----
        if wp_i < len(waypoints):
            gx, gy = waypoints[wp_i]
            dx, dy = gx - x, gy - y
            dist = math.hypot(dx, dy)
            rad = math.radians(-yaw)
            lx = dx * math.cos(rad) + dy * math.sin(rad)
            ly = -dx * math.sin(rad) + dy * math.cos(rad)
            if dist < 3.5:
                wp_i += 1
                continue
            if lx < 0:
                steer = 1.0
            else:
                curv = 2.0 * ly / max(dist * dist, 1e-3)
                steer = max(-1.0, min(1.0, -math.atan(curv * 2.6) / 0.6))
            # 扫描段（从第 3 个航点起是北侧读牌走廊）放慢到 ~1.2 m/s，
            # 其它走廊段正常速度，省时间给读牌。
            scanning = wp_i >= 2
            if scanning:
                thr = 0.010 if abs(steer) < 0.75 else 0.008
            else:
                thr = 0.050 if abs(steer) < 0.75 else 0.030
            sp = abs(float(msg.main_vehicle.speed))
            # 顶住就倒车（用户 10:21：撞车了要及时倒车）
            if now < back_until:
                api.set_vehicle_control(VehicleControl(gear=GearMode.REVERSE,
                                                      throttle=0.02, brake=0.0, steering=0.0))
            elif sp < 0.10 and thr > 0:
                stuck_t += 0.1
                if stuck_t > 1.5:
                    stuck_t = 0.0
                    back_until = now + 1.0
                    print(f"[{now:5.1f}s] 顶住 1.5s，倒车脱困", flush=True)
                    api.set_vehicle_control(VehicleControl(gear=GearMode.REVERSE,
                                                          throttle=0.02, brake=0.0, steering=0.0))
                else:
                    api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=thr,
                                                          brake=0.0, steering=steer))
            else:
                stuck_t = 0.0
                api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=thr,
                                                      brake=0.0, steering=steer))
        else:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))

        # ---- 视觉识别：轮流认 4 路相机 ----
        if frames and now - last_ocr > 0.25:
            last_ocr = now
            cam = ocr_i % len(frames)
            ocr_i += 1
            try:
                txt = read_text(frames[cam].frame, scale=1.5)
            except Exception as exc:  # noqa: BLE001
                txt = ""
                print(f"   ocr err {exc!r}", flush=True)
            if txt:
                hits = {norm_id(a, b) for a, b in STICKER_RE.findall(txt)}
                if hits:
                    for h in hits:
                        votes[h] = votes.get(h, 0) + 1
                    print(f"[{now:5.1f}s] cam{cam} OCR {txt!r} → {sorted(hits)} "
                          f"（累计 {len(votes)}）", flush=True)

        if args.shot and frame in (120, 300, 480, 660) and frame not in shot_done:
            shot_done.add(frame)
            import cv2
            for k, cf in enumerate(frames or []):
                try:
                    cv2.imwrite(f"{args.shot}_f{frame}_cam{k}.png", cf.frame)
                except Exception:  # noqa: BLE001
                    pass

        # ---- 一读到号就先提交一次（用户 11:01：先拿点分），后面继续读继续更新 ----
        if votes and submitted is None:
            letters = [k.split("-")[0] for k in votes] or ["B"]
            prefix = max(set(letters), key=letters.count)
            legal_now = sorted(k for k in votes if k.startswith(prefix + "-"))
            all_ids = [f"{prefix}-{i:02d}" for i in range(1, 11)]
            submitted = [i for i in all_ids if i not in legal_now]
            print(f"读到 {legal_now} → 先提交补集 {submitted}", flush=True)
            if not args.dry_run:
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0),
                                        make_out(args, submitted))

        # ---- 走完扫描线：补集提交 ----
        if wp_i >= len(waypoints) and submitted is None:
            letters = [k.split("-")[0] for k in votes] or ["B"]
            prefix = max(set(letters), key=letters.count)
            legal = sorted(k for k in votes if k.startswith(prefix + "-"))
            all_ids = [f"{prefix}-{i:02d}" for i in range(1, 11)]
            illegal = [i for i in all_ids if i not in legal]
            submitted = illegal
            if args.region == "function":
                ids = sorted(k for k in votes)
                submitted = ids
                print(f"功能区读到车牌 {ids} → 按 {args.rule} 全部上报", flush=True)
            else:
                print(f"合法区读到 {legal} → 全集 10 辆，提交补集 {illegal}", flush=True)
            if not args.dry_run:
                out = make_out(args, submitted)
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0), out)

        if submitted is not None and not args.dry_run and frame % 15 == 0:
            out = make_out(args, submitted)
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0), out)

        if used > args.max_seconds:
            print(f"到时间 {used:.1f}s，结束。投票 {json.dumps(votes, ensure_ascii=False)}",
                  flush=True)
            break

    print("SUMMARY " + json.dumps({"votes": votes, "submitted": submitted}, ensure_ascii=False),
          flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

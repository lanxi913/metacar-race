# -*- coding: utf-8 -*-
"""贴涂装边界巡检（第一趟：只沿边界跑、只数车，不读号、不交卷）。

思路（用户定下的决策）：**涂装边界就是道路指向标**。
* 车都停在涂装内侧 ⇒ 沿边界外侧跑，撞车概率天然最小；
* 一旦离开涂装进入无标线沙地就没有参照物 ⇒ 硬规矩：找不到边界就减速/停，不乱开。

控制：把画面里最近的涂装边界拟合成直线（先换到车体坐标：前向 x、右向 y），
得到"边界相对我的侧向距离 e"和"边界走向角 β"，然后
`steer = clamp(-K_E·(e − E_REF) - K_B·β, ±1)`，并保证涂装在**同侧**（外侧行驶）。
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

FOCAL = 420.0
K_GROUND = 1560.0
HUE_LO, HUE_HI = 20, 105
E_REF = 2.6             # 目标：边界在我侧向 2.6 m
K_E = 0.55              # 侧向误差 → 舵
K_B = 0.90              # 边界走向 → 舵
V_CRUISE = 2.0          # m/s
V_SLOW = 0.9
A_ACC, A_DEC = 1.6, 1.8
THR_CAP = 0.30
BITE = 0.15


def imread_uni(p):
    try:
        return cv2.imdecode(np.fromfile(str(p), dtype=np.uint8), cv2.IMREAD_COLOR)
    except Exception:
        return None


def imwrite_uni(p, img):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    if ok:
        buf.tofile(str(p))


def horizon_row(img) -> int:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1].astype(np.float32).mean(axis=1)
    for y in range(2, img.shape[0] - 2):
        if sat[y] < 75.0:
            return y
    return int(img.shape[0] * 0.5)


def hatch_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    m = ((h >= HUE_LO) & (h <= HUE_HI) & (s >= 60) & (v >= 60)).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return m


def car_boxes(img, horizon):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    s, v = hsv[:, :, 1], hsv[:, :, 2]
    paint = ((s > 90) & (v > 30) & (v < 235)).astype(np.uint8) * 255
    paint[: horizon + 4, :] = 0
    paint = cv2.morphologyEx(paint, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    paint = cv2.morphologyEx(paint, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _l, stats, _c = cv2.connectedComponentsWithStats(paint, 8)
    out = []
    H = img.shape[0]
    for i in range(1, n):
        x, y, w, hh, area = stats[i]
        if area < 380 or w < 20 or hh < 10:
            continue
        if area / float(w * hh) < 0.40 or (y + hh) > 0.94 * H:
            continue
        if not (0.25 <= hh / float(w) <= 4.0):
            continue
        out.append(dict(x=int(x), y=int(y), w=int(w), h=int(hh)))
    out.sort(key=lambda d: -d["w"])
    return out


def fit_edge(mask, horizon, u_center=320):
    """把最近的涂装边界拟合成车体坐标里的一条直线 y = m·x + b（x 前向、y 右向）。

    返回 dict(m, b, side, e, cnt, pts_ego, ui) 或 None。
    side=+1 表示"涂装在我的右侧"。
    """
    H, W = mask.shape[:2]
    y0 = min(H - 2, horizon + 35)
    y1 = min(H - 2, horizon + 215)
    pts_ego, pts_img, sides = [], [], []
    for y in range(y0, y1, 5):
        idx = np.where(mask[y] > 0)[0]
        if idx.size < 30:
            continue
        # 取离画面中心最近的那一段涂装边界
        runs = np.split(idx, np.where(np.diff(idx) > 8)[0] + 1)
        best = None
        for r in runs:
            if r.size < 12:
                continue
            for u in (int(r[0]), int(r[-1])):
                if best is None or abs(u - u_center) < abs(best[0] - u_center):
                    best = (u, "L" if u == int(r[0]) else "R")
        if best is None:
            continue
        u, tag = best
        d = K_GROUND / max(6.0, float(y - horizon))
        yr = (u - u_center) * d / FOCAL          # 右为正
        pts_ego.append((d, yr))
        pts_img.append((u, y))
        sides.append(-1.0 if tag == "R" else 1.0)   # 取右边界 ⇒ 涂装在左(-1)
    if len(pts_ego) < 5:
        return None
    xs = np.array([p[0] for p in pts_ego], float)
    ys = np.array([p[1] for p in pts_ego], float)
    A = np.vstack([xs, np.ones_like(xs)]).T
    sol, *_ = np.linalg.lstsq(A, ys, rcond=None)
    m, b = float(sol[0]), float(sol[1])
    side = 1.0 if np.mean(sides) > 0 else -1.0
    denom = math.sqrt(1.0 + m * m)
    e = side * b / denom          # 边界在我这一侧的（正）距离
    return dict(m=m, b=b, side=side, e=abs(e), cnt=len(pts_ego),
                pts_img=pts_img, ui=None)


def longitudinal(v, v_t, dt):
    err = v_t - v
    a = max(-A_DEC, min(A_ACC, 1.6 * err))
    thr = max(0.0, min(THR_CAP, a / 18.0))
    brk = max(0.0, min(0.06, -a / 25.0))
    if v < 0.5 and v_t > 0.6:
        thr = max(thr, BITE)
    return thr, brk


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None)
    ap.add_argument("--max-seconds", type=float, default=95.0)
    ap.add_argument("--shots", action="store_true")
    args = ap.parse_args()

    out = Path(args.out) if args.out else Path("logs") / "vla2" / ("edge_" + time.strftime("%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)
    shots = out / "shots"
    shots.mkdir(exist_ok=True)

    api = SceneAPI()
    print("连接仿真场景…", flush=True)
    api.connect()
    print("已连接", flush=True)

    csv_f = open(out / "run.csv", "w", newline="", encoding="utf-8")
    wr = csv.writer(csv_f)
    wr.writerow(["frame", "t", "x", "y", "yaw", "v", "thr", "brk", "steer",
                 "v_tgt", "e_edge", "side", "slope", "ncars", "hrow", "note"])

    frame, t0, last_t = 0, None, 0.0
    prev_v, last_log = 0.0, -1.0
    lost = 0
    stuck_t, escape_t = 0.0, 0.0
    stat = dict(edge_frames=0, lost_frames=0, max_abs_e=0.0, min_car_gap=99.0)

    try:
        for msg, cams in api.main_loop():
            frame += 1
            pose, car = msg.pose_gnss, msg.main_vehicle
            t = float(msg.scene_status.used_time) if msg.scene_status else 0.0
            if t0 is None:
                t0 = t
            dt = max(1e-3, min(0.2, t - last_t)) if last_t else 1 / 30.0
            last_t = t
            x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
            v = float(car.speed)

            img = None
            if cams:
                for cam in cams:
                    arr = getattr(cam, "frame", cam)
                    if arr is None or getattr(arr, "size", 0) == 0:
                        continue
                    a = np.asarray(arr, dtype=np.uint8)
                    img = (cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if a.ndim == 3
                           else cv2.cvtColor(a, cv2.COLOR_GRAY2BGR))
                    break

            hrow, edge, cars = 240, None, []
            note = ""
            if img is not None:
                hrow = horizon_row(img)
                if frame % 2 == 0:
                    edge = fit_edge(hatch_mask(img), hrow)
                    cars = car_boxes(img, hrow)
                    if edge:
                        stat["edge_frames"] += 1
                        stat["max_abs_e"] = max(stat["max_abs_e"], edge["e"])
                    else:
                        stat["lost_frames"] += 1
                    if args.shots and frame % 90 == 0:
                        vis = img.copy()
                        for (u, yy) in (edge["pts_img"] if edge else []):
                            cv2.circle(vis, (u, yy), 2, (0, 0, 255), -1)
                        for c in cars:
                            cv2.rectangle(vis, (c["x"], c["y"]),
                                          (c["x"] + c["w"], c["y"] + c["h"]),
                                          (255, 128, 0), 2)
                        imwrite_uni(shots / f"f{frame:05d}_e{edge['e']:.2f}.jpg" if edge
                                   else shots / f"f{frame:05d}_noedge.jpg", vis)

            # ---------------- 控制
            if edge is not None:
                lost = 0
                # 边界应在我涂装那一侧（side=+1 涂装在右），距离 E_REF
                err_e = edge["e"] - E_REF                      # >0 = 离边界太远
                beta = math.atan(edge["m"])                    # >0 = 边界向右偏
                # 想靠近边界：如果边界在右(side=+1)且太远(err_e>0) → 右转 → steer<0
                steer = -edge["side"] * (K_E * err_e) - K_B * beta
                steer = max(-1.0, min(1.0, steer))
                v_t = V_CRUISE if abs(err_e) < 1.2 else V_SLOW
            else:
                lost += 1
                # 丢线：保持上一次舵角、降速直行；连续丢 20 帧（≈0.7 s）就停
                steer = 0.0
                v_t = V_SLOW if lost < 20 else 0.0
                note = f"lost={lost}"

            # 近车：只降速，不改航向（不乱打舵）
            nearest = min([c["w"] for c in cars], default=999)
            if nearest >= 150:
                v_t = min(v_t, 0.6)
            elif nearest >= 110:
                v_t = min(v_t, 1.0)

            thr, brk = longitudinal(v, v_t, dt)
            gear = GearMode.DRIVE

            if v < 0.08 and thr > 0.10:
                stuck_t += dt
            else:
                stuck_t = max(0.0, stuck_t - dt)
            if stuck_t > 1.2 or escape_t > 0.0:
                escape_t += dt
                gear = GearMode.REVERSE
                thr, brk = 0.30, 0.0
                steer = -steer * 0.6
                if escape_t > 1.0:
                    escape_t, stuck_t = 0.0, 0.0
                    note = "escape"

            api.set_vehicle_control(VehicleControl(gear=gear, throttle=float(thr),
                                                   brake=float(brk),
                                                   steering=float(steer)))
            elapsed = t - t0
            wr.writerow([frame, round(elapsed, 3), round(x, 3), round(y, 3),
                         round(yaw, 2), round(v, 3), round(thr, 3), round(brk, 3),
                         round(steer, 3), round(v_t, 2),
                         round(edge["e"], 2) if edge else "",
                         edge["side"] if edge else "",
                         round(math.atan(edge["m"]), 3) if edge else "",
                         len(cars), hrow, note])
            if int(elapsed) != int(last_log):
                last_log = elapsed
                print(f"[{elapsed:5.1f}s] x={x:6.2f} y={y:6.2f} yaw={yaw:6.1f} "
                      f"v={v:4.2f} steer={steer:+.2f} "
                      f"e={edge['e']:5.2f} side={edge['side']:+.0f} "
                      f"b={math.degrees(math.atan(edge['m'])):+5.1f}° "
                      f"cars={len(cars)} thr={thr:.2f}" if edge else
                      f"[{elapsed:5.1f}s] x={x:6.2f} y={y:6.2f} lost edge v={v:4.2f}",
                      flush=True)
            prev_v = v
            if elapsed > args.max_seconds:
                break
    except KeyboardInterrupt:
        print("interrupted", flush=True)
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, throttle=0.0,
                                                   brake=1.0, steering=0.0))
        except Exception:
            pass
        csv_f.close()
        print(f"[stat] edge_frames={stat['edge_frames']} lost={stat['lost_frames']} "
              f"max_e={stat['max_abs_e']:.2f} out={out}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

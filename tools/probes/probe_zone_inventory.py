"""VLA 功能区 —— 巡场「清点」探针（v5）。

目的：一趟走完蓝区，用**方位角三角化**把所有车顶牌的世界坐标解出来，同时记录每块牌
在不同距离下的像素尺寸——这两件事决定正式实现要不要逐车停靠、停多远、走几条车道。

原理（为什么可行）：车顶牌有高度，单帧反投影到地面会偏；但**方位角与高度无关**——
同一块牌在两帧里的水平视线相交，交点是它真实的 (x, y)。相机位姿已知（pose_gnss），
cam0 内参 fx=184.8 / cx=320 已知，所以：
    方位角 = yaw + atan2(u - 320, 184.8)   （u 为像素列，yaw 顺时针为正，相机朝 +x）
    视线方向 = (cos, sin)(方位角)
两帧视线求交即得坐标。

用法：
    python probe_zone_inventory.py --drive          # 起场景、跑三趟蛇形、落 CSV
    python probe_zone_inventory.py --analyze        # 离线三角化 + 聚类 + 汇总
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import cv2
import numpy as np

OUT = Path(r"D:\download3\算法邀請賽\accel_window_probe\_vla_ref\zone_inventory")
FX, CX = 184.8, 320.0
WHEELBASE_M = 2.6
MAX_STEER_RAD = 0.6
LANES = [(18.0, 11.0), (-18.0, 11.0), (-18.0, 0.0), (18.0, 0.0),
         (18.0, -11.0), (-18.0, -11.0)]


def save_png(path: Path, img) -> bool:
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(buf.tobytes())
    return True


def plate_candidates(img):
    """返回 [(x, y, w, h, area, fill)]，阈值放得比较松，分析时再收紧。"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    mask = ((h >= 10) & (h <= 45) & (s >= 60) & (v >= 100)).astype(np.uint8) * 255
    mask[:200, :] = 0                     # 天空区不算
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    out = []
    for i in range(1, n):
        x, y, w, hh, area = stats[i]
        if area < 12 or hh < 3 or w < 5:
            continue
        ar = w / float(hh)
        if not (0.7 <= ar <= 8.0):
            continue
        out.append((int(x), int(y), int(w), int(hh), int(area), area / float(w * hh)))
    out.sort(key=lambda r: -r[4])
    return out[:12]


class Follower:
    def __init__(self, wps, lookahead=6.0):
        self.wps = [tuple(map(float, w)) for w in wps]
        self.lookahead = lookahead
        self.seg = 0
        self.seg_off = 0.0
        self.uturn = 0.0

    def _len(self, i):
        (ax, ay), (bx, by) = self.wps[i], self.wps[i + 1]
        return math.hypot(bx - ax, by - ay)

    def update(self, x, y):
        best = None
        for i in range(len(self.wps) - 1):
            (ax, ay), (bx, by) = self.wps[i], self.wps[i + 1]
            vx, vy = bx - ax, by - ay
            L2 = vx * vx + vy * vy
            if L2 < 1e-9:
                continue
            t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L2))
            d = math.hypot(x - (ax + vx * t), y - (ay + vy * t))
            if best is None or d < best[1]:
                best = (t * math.sqrt(L2), d, i)
        if best:
            self.seg_off, _, self.seg = best
        return best[1] if best else 0.0

    def target(self):
        want = self.seg_off + self.lookahead
        acc = 0.0
        for i in range(self.seg, len(self.wps) - 1):
            L = self._len(i)
            if acc + L >= want:
                (ax, ay), (bx, by) = self.wps[i], self.wps[i + 1]
                f = (want - acc) / L if L > 1e-9 else 0.0
                return (ax + (bx - ax) * f, ay + (by - ay) * f)
            acc += L
        return self.wps[-1]

    def remaining(self):
        return max(0.0, sum(self._len(i) for i in range(self.seg, len(self.wps) - 1))
                   - self.seg_off)

    def steering(self, x, y, yaw_deg):
        tx, ty = self.target()
        dx, dy = tx - x, ty - y
        dist = max(math.hypot(dx, dy), 1e-3)
        rad = math.radians(-yaw_deg)
        lx = dx * math.cos(rad) + dy * math.sin(rad)
        ly = -dx * math.sin(rad) + dy * math.cos(rad)
        if lx < 0.0:
            if self.uturn == 0.0:
                self.uturn = 1.0 if ly > 0 else -1.0
            return self.uturn
        self.uturn = 0.0
        curvature = 2.0 * ly / (dist * dist)
        return max(-1.0, min(1.0, -math.atan(curvature * WHEELBASE_M) / MAX_STEER_RAD))


def drive(seconds: float, shot_every: float) -> int:
    from metacar import GearMode, SceneAPI, VehicleControl

    OUT.mkdir(parents=True, exist_ok=True)
    csv_path = OUT / "dets.csv"
    fh = csv_path.open("w", newline="", encoding="utf-8")
    wr = csv.writer(fh)
    wr.writerow(["t", "ego_x", "ego_y", "yaw", "v", "u", "vp", "w", "h", "area", "fill", "tag"])

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    if getattr(static, "vla_extension", None) is None:
        print("!! 不是 VLA 场景")
        return 2

    follower = Follower(LANES)
    t0 = time.time()
    tag = 0
    prev_shot = -1e9
    prev_log = -1e9
    n_det = 0

    for msg, frames in api.main_loop():
        now = time.time() - t0
        pose = msg.pose_gnss
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        v = abs(float(msg.main_vehicle.speed))

        if frames:
            img = frames[0].frame
            cands = plate_candidates(img)
            if cands:
                n_det += len(cands)
                for (u, vp, w, hh, area, fill) in cands:
                    wr.writerow([f"{now:.2f}", f"{x:.2f}", f"{y:.2f}", f"{yaw:.1f}",
                                 f"{v:.2f}", u, vp, w, hh, area, f"{fill:.2f}", f"t{tag:04d}"])
            if now - prev_shot >= shot_every:
                prev_shot = now
                tag += 1
                save_png(OUT / "f" / f"t{tag:04d}_cam0.png", img)

        steer = follower.steering(x, y, yaw)
        vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.02, brake=0.0, steering=steer)
        api.set_vehicle_control(vc)

        if now - prev_log >= 1.0:
            prev_log = now
            print(f"[{now:5.1f}s] ({x:7.2f},{y:7.2f}) yaw={yaw:6.1f} v={v:5.2f} "
                  f"剩余={follower.remaining():5.1f} 帧={tag} 候选累计={n_det}", flush=True)
        if now > seconds:
            print("到时退出", flush=True)
            break

    fh.close()
    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except OSError:
        pass
    print(f"检测记录 {csv_path}（{n_det} 条），帧 {OUT / 'f'}", flush=True)
    return 0


def _intersect(p1, a1, p2, a2):
    """两条水平视线求交；平行返回 None。"""
    d1 = (math.cos(a1), math.sin(a1))
    d2 = (math.cos(a2), math.sin(a2))
    det = d1[0] * (-d2[1]) - d1[1] * (-d2[0])
    if abs(det) < 1e-6:
        return None
    rx, ry = p2[0] - p1[0], p2[1] - p1[1]
    t = (rx * (-d2[1]) - ry * (-d2[0])) / det
    if t <= 0:
        return None
    return (p1[0] + d1[0] * t, p1[1] + d1[1] * t)


def analyze(min_area: int, min_w: int) -> int:
    rows = list(csv.DictReader((OUT / "dets.csv").open(encoding="utf-8")))
    print(f"检测记录 {len(rows)} 条")
    obs = []
    for r in rows:
        if int(r["area"]) < min_area or int(r["w"]) < min_w:
            continue
        u = float(r["u"])
        yaw = math.radians(float(r["yaw"]))
        az = yaw + math.atan2(u - CX, FX)
        obs.append((float(r["ego_x"]), float(r["ego_y"]), az,
                    int(r["w"]), int(r["h"]), int(r["area"]), r["tag"], float(r["t"])))
    print(f"参与三角化 {len(obs)} 条（area≥{min_area}, w≥{min_w}）")

    pts = []
    for i in range(len(obs)):
        for j in range(i + 1, min(i + 40, len(obs))):
            x1, y1, a1 = obs[i][0], obs[i][1], obs[i][2]
            x2, y2, a2 = obs[j][0], obs[j][1], obs[j][2]
            if math.hypot(x2 - x1, y2 - y1) < 1.5:
                continue                      # 基线太短，误差大
            p = _intersect((x1, y1), a1, (x2, y2), a2)
            if p is None:
                continue
            d1 = math.hypot(p[0] - x1, p[1] - y1)
            d2 = math.hypot(p[0] - x2, p[1] - y2)
            if not (3.0 <= d1 <= 60.0 and 3.0 <= d2 <= 60.0):
                continue
            pts.append((p[0], p[1], max(obs[i][3], obs[j][3]), obs[i][7]))

    clusters = []
    for (px, py, w, t) in pts:
        for c in clusters:
            if math.hypot(c["x"] - px, c["y"] - py) < 2.0:
                c["n"] += 1
                c["sx"] += px
                c["sy"] += py
                c["w"] = max(c["w"], w)
                break
        else:
            clusters.append({"x": px, "y": py, "sx": px, "sy": py, "n": 1, "w": w})
    clusters = [c for c in clusters if c["n"] >= 3]
    for c in clusters:
        c["mx"] = c["sx"] / c["n"]
        c["my"] = c["sy"] / c["n"]
    # 合并相距 2.5 m 内的簇
    merged = []
    for c in sorted(clusters, key=lambda q: -q["n"]):
        for m in merged:
            if math.hypot(m["mx"] - c["mx"], m["my"] - c["my"]) < 2.5:
                break
        else:
            merged.append(c)
    print(f"\n三角化得到 {len(pts)} 个交点，聚类出 {len(merged)} 个目标：")
    for c in sorted(merged, key=lambda q: -q["n"]):
        print(f"  ({c['mx']:7.2f},{c['my']:7.2f}) 顶点数={c['n']:4d} 最大牌宽={c['w']:3d} px")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--drive", action="store_true")
    ap.add_argument("--analyze", action="store_true")
    ap.add_argument("--seconds", type=float, default=50.0)
    ap.add_argument("--shot-every", type=float, default=0.5)
    ap.add_argument("--min-area", type=int, default=25)
    ap.add_argument("--min-w", type=int, default=6)
    args = ap.parse_args()
    if args.drive:
        return drive(args.seconds, args.shot_every)
    return analyze(args.min_area, args.min_w)


if __name__ == "__main__":
    raise SystemExit(main())

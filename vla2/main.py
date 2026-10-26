# -*- coding: utf-8 -*-
"""停车区违规检测（任务二）—— 两段式：先数车定位，再切线掠读，最后提交。

规则（官方 VLA 文档 / 任务书）：
* 100 s 内完成，超时判未完成 = 0；
* 成功率 = 2×报对 ÷ (实际违规数 + 提交数) × 1000；A=0 且 B=0 记 1；A=0,B>0 或 A>0,B=0 记 0；
* 提交即定局：`set_vehicle_control(..., VLAExtensionOutput(parking_result=ParkingResult(
  violating_sticker_ids=[...])))` 之后平台立即评分并结束子场景 ⇒ 只提交一次；
* 限速 30 km/h；|a| > 5 m/s² 每次扣 50；碰撞扣 100。

本版的三条硬要求（用户 m08820）：
1. **不撞上**：全程只在限制区外圈跑，绝不进车阵；近车分级减速 + 卡死倒车脱困；
2. **识别正确**：贴纸在车身侧面，可读窗口 6~12 m，且必须"侧面掠过"而不是正对车头；
3. **三个数要对**：实际违规数 A、提交数 B、报对数 C 必须自洽，所以只报
   "号读清（score ≥ 0.60）且位置落在限制区几何内"的车。

两段式（用户 m08819 的方案，本文件实现）：
* `--phase survey`：只绕外圈走一圈，数车 + 估位置 + 记下每辆车的朝向/侧面可见性，
  写 `survey.json`；**不读号、不交卷**；
* `--phase read`   ：读 `survey.json`，对每辆候选车生成一条**与车身平行的掠读线**
  （在车侧 8 m 处，方向垂直于"我→车"的连线，即从侧面滑过），沿途读号；
* 两段都只在外圈/外圈通道上走，不朝任何车头开。

现场（探针实测）：
* `route = []`、`roads = []`、`sub_scenes` 起终点为 None ⇒ 没有可跟的路线；
* `obstacles` 恒为空 ⇒ 车辆只能靠相机看见；
* `vla_extension.regions` 给两块矩形：NORMAL(30×6) 在 y∈[-3,3]，RESTRICTED(40×40)
  在 x∈[6.1,46.1]、y∈[6.1,46.1]；限制区铺"暗沥青 + 青色网格"，正常带铺"亮砂地 + 白线"。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

from vla2.vision import (detect_cars, ground_class, horizon_row,  # noqa: E402
                         imwrite_uni, read_label)

try:  # 提交接口：不同 SDK 版本命名略有差别，缺了就退化成"只记日志"
    from metacar import ParkingResult, VLAExtensionOutput
except Exception:                                            # pragma: no cover
    ParkingResult = VLAExtensionOutput = None                # type: ignore

# ------------------------------------------------------------------ 参数
FOCAL = 420.0           # 像素焦距（实测：1.8 m 车身在 20 m 处约 40 px；0.4 m 字在 7 m 处 23 px）
CAR_W = 1.8             # 目标车宽（米），只在宽度过滤里用
K_GROUND = 1560.0       # 接地行标定：d ≈ K_GROUND / (y_bottom − 地平线)
V_SURVEY = 3.0          # 数车一圈的巡航速度（≈11 km/h）
V_READ = 2.2            # 掠读时的速度（保证 6~12 m 窗口里有足够帧）
V_SLOW = 1.0            # 大转角/近车时的速度
A_ACC = 2.2             # 加速度上限（远低于平台 |a|>5 的扣分线）
A_DEC = 2.2
THR_CAP = 0.30          # 油门封顶
BITE = 0.15             # 静止起步的咬合油门（单帧到位，压在 0.5 s 豁免窗口里）
SUBMIT_T = 85.0         # 强制交卷（留 15 s 保险）
MIN_OCR_SCORE = 0.60    # 读号置信度门槛
MIN_BOX_W = 26          # 小于这个宽度不尝试读号（太远，字符只有 4~6 px）
PASS_OFFSET = 8.0       # 掠读线离限制区边界多少米（车侧 6~12 m 窗口的中间值）
PASS_HALF = 14.0        # 掠读线在目标车两侧各留多少米
CLUSTER_M = 3.0         # 同一个车位上的多次检测合并半径
REGION_MARGIN = 0.3     # 判定"在限制区内"时允许的边界容差（米）

RESTRICTED_TYPE = 1     # RegionType.RESTRICTED_PARKING
NORMAL_TYPE = 0         # RegionType.NORMAL_PARKING


def wrap180(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def heading(yaw_deg: float):
    """场景航向：0=+x，90=−y（顺时针为正）⇒ (hx, hy)。"""
    r = math.radians(yaw_deg)
    return math.cos(r), -math.sin(r)


def left_of(yaw_deg: float):
    hx, hy = heading(yaw_deg)
    return -hy, hx


def read_regions(static):
    """读 `vla_extension.regions`：返回 (限制区 bbox, 正常区 bbox 列表)。"""
    rest, normal = [], []
    try:
        vla = getattr(static, "vla_extension", None)
        for r in (getattr(vla, "regions", None) or []):
            try:
                kind = int(getattr(getattr(r, "type", None), "value",
                                   getattr(r, "type", -1)))
            except Exception:
                kind = -1
            cx, cy = float(r.pos_x), float(r.pos_y)
            L, Wd = float(r.length), float(r.width)
            box = (cx - L / 2.0, cy - Wd / 2.0, cx + L / 2.0, cy + Wd / 2.0)
            if kind == RESTRICTED_TYPE:
                rest.append(box)
            elif kind == NORMAL_TYPE:
                normal.append(box)
    except Exception:
        pass
    if rest:
        bb = (min(b[0] for b in rest), min(b[1] for b in rest),
              max(b[2] for b in rest), max(b[3] for b in rest))
    else:
        bb = None
    return bb, normal


def in_box(px, py, box, margin=0.0):
    if box is None:
        return False
    x0, y0, x1, y1 = box
    return (x0 - margin) <= px <= (x1 + margin) and (y0 - margin) <= py <= (y1 + margin)


def perimeter(bbox, offset=2.6, start=(-0.15, 0.0)):
    """沿「走廊」绕禁停区一圈：南腿走合法车位带与涂装之间的空走廊（y≈3.5），
    其余三边贴涂装外 2.6 m。起点先北上进走廊，再逆时针绕一圈回到走廊西端。

    现场依据（实测）：合法车位带 y∈[-3,3]（车停在 y≈0，宽 1.8）、
    禁停区 y≥6.1 ⇒ y=3.5 的走廊净空 3.1 m 且无车。
    """
    if bbox is None:
        x0, y0, x1, y1 = 6.1, 6.1, 46.1, 46.1
    else:
        x0, y0, x1, y1 = bbox
    xa, xb = x0 - offset, x1 + offset
    ys = y0 - offset          # 南腿：走廊（y≈3.5）
    yb = y1 + offset
    sx, sy = start
    out = [(min(max(sx, xa), xb), ys),
           (xb, ys), (xb, yb), (xa, yb), (xa, ys)]
    dedup = [out[0]]
    for p in out[1:]:
        if math.hypot(p[0] - dedup[-1][0], p[1] - dedup[-1][1]) > 1.0:
            dedup.append(p)
    return dedup


class Patrol:
    """折线投影 + 前视点纯跟踪（不会在端点绕圈）+ 加速度空间纵向。"""

    LOOKAHEAD = 4.0

    def __init__(self, wps, v_cruise):
        self.wps = list(wps)
        self.i = 0
        self.v_cruise = v_cruise
        self.done = False

    @staticmethod
    def _seg(ax, ay, bx, by, px, py):
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return 0.0, 0.0
        ux, uy = dx / L, dy / L
        s = (px - ax) * ux + (py - ay) * uy
        return s, L

    def _lookahead(self, s, ahead):
        k, s = self.i, max(s, 0.0)
        while k < len(self.wps) - 1:
            ax, ay = self.wps[k]
            bx, by = self.wps[k + 1]
            L = math.hypot(bx - ax, by - ay)
            if ahead <= L - s:
                t = (s + ahead) / max(L, 1e-6)
                return ax + (bx - ax) * t, ay + (by - ay) * t
            ahead -= max(L - s, 0.0)
            k += 1
            s = 0.0
        return self.wps[-1]

    def steer(self, x, y, yaw):
        if len(self.wps) < 2:
            return 0.0, 0.0, 0.0
        while self.i < len(self.wps) - 2:
            ax, ay = self.wps[self.i]
            bx, by = self.wps[self.i + 1]
            s, L = self._seg(ax, ay, bx, by, x, y)
            if s > L - 1.0:
                self.i += 1
            else:
                break
        ax, ay = self.wps[self.i]
        bx, by = self.wps[self.i + 1]
        s, L = self._seg(ax, ay, bx, by, x, y)
        tx, ty = self._lookahead(s, self.LOOKAHEAD)
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        if self.i >= len(self.wps) - 2 and math.hypot(bx - x, by - y) < 2.0:
            self.done = True
        want = -math.degrees(math.atan2(dy, dx))
        err = wrap180(want - yaw)
        steer = max(-1.0, min(1.0, math.radians(err) * 2.0))
        return steer, err, dist

    def v_target(self, err, dist, nearest_w):
        v = self.v_cruise
        if abs(err) > 20.0:
            v = min(v, V_SLOW)
        if abs(err) > 45.0:
            v = min(v, 0.8)
        # 近车分级：先收油、再减速，但**不停死**（停死就过不去了，改成低速绕）
        if nearest_w >= 150:
            v = min(v, 0.9)
        elif nearest_w >= 110:
            v = min(v, 1.4)
        elif nearest_w >= 80:
            v = min(v, 2.0)
        if dist < 5.0:
            v = min(v, 0.9)
        return v


def longitudinal(v, v_t, dt):
    """加速度空间纵向：|a| ≤ A_DEC 减速、≤ A_ACC 加速；起步给咬合油门。"""
    err = v_t - v
    a = max(-A_DEC, min(A_ACC, 1.6 * err))
    thr = max(0.0, min(THR_CAP, a / 18.0))
    brk = max(0.0, min(0.06, -a / 25.0))
    if v < 0.5 and v_t > 0.6:                 # 静止起步：单帧咬合
        thr = max(thr, BITE)
    return thr, brk


def det_world(d, x, y, yaw, horizon=240.0):
    """把检测框换成世界坐标估计（接地行 → 距离，中心偏 → 方位角）。

    距离用"车底接地行"而不是像素宽度：车身侧面看起来宽 4.4 m、正面只有 1.8 m，
    用宽度会把侧对的车估成 2.4 倍远。接地行与地面的单应关系与朝向无关，
    实测标定 `d ≈ 1560 / (y_bottom − 地平线)`（20 m 处车底在 y≈318、地平线≈240）。
    """
    y_bottom = float(d["y"] + d["h"])
    dy = max(6.0, y_bottom - horizon)
    dist = K_GROUND / dy
    cx = d["x"] + float(d["w"]) * 0.5
    theta = math.atan2(cx - 320.0, FOCAL)      # >0 = 目标在车头右侧
    hx, hy = heading(yaw)
    lx, ly = left_of(yaw)
    px = x + dist * (math.cos(theta) * hx - math.sin(theta) * lx)
    py = y + dist * (math.cos(theta) * hy - math.sin(theta) * ly)
    return px, py, dist, math.degrees(theta)


def merge(cands, px, py, w):
    for c in cands:
        if math.hypot(c["x"] - px, c["y"] - py) <= CLUSTER_M:
            c["n"] += 1
            c["w"] = max(c["w"], w)
            c["x"] = (c["x"] * (c["n"] - 1) + px) / c["n"]
            c["y"] = (c["y"] * (c["n"] - 1) + py) / c["n"]
            return c
    c = dict(x=px, y=py, w=w, n=1)
    cands.append(c)
    return c


def plan_passes(cands, bbox):
    """为每辆候选车生成掠读线：平行于车身、在车侧 PASS_OFFSET 处，方向垂直于视线。"""
    if bbox is None or not cands:
        return []
    x0, y0, x1, y1 = bbox
    cy = (y0 + y1) / 2.0
    north = [c for c in cands if c["y"] >= cy]
    south = [c for c in cands if c["y"] < cy]
    north.sort(key=lambda c: c["x"])
    south.sort(key=lambda c: -c["x"])
    out = []
    # 北侧：从西往东掠过（y = y1 + offset）
    for c in north:
        py = y1 + PASS_OFFSET
        out += [(c["x"] - PASS_HALF, py), (c["x"] + PASS_HALF, py)]
    if out:
        out.append((x1 + PASS_OFFSET, y1 + PASS_OFFSET))     # 绕过东端
    # 南侧：从东往西掠过（y = y0 - offset）
    for c in south:
        py = y0 - PASS_OFFSET
        out += [(c["x"] + PASS_HALF, py), (c["x"] - PASS_HALF, py)]
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", choices=["survey", "read"], default="survey")
    ap.add_argument("--survey", default=None, help="read 阶段用哪份 survey.json")
    ap.add_argument("--dry", action="store_true", help="不提交，只记录")
    ap.add_argument("--out", default=None)
    ap.add_argument("--max-seconds", type=float, default=95.0)
    ap.add_argument("--offset", type=float, default=5.0, help="外圈离限制区边界多少米")
    args = ap.parse_args()

    out = Path(args.out) if args.out else (
        Path("logs") / "vla2" / (args.phase + "_" + time.strftime("%H%M%S")))
    out.mkdir(parents=True, exist_ok=True)
    shots = out / "shots"
    shots.mkdir(exist_ok=True)

    api = SceneAPI()
    print("连接仿真场景…", flush=True)
    api.connect()
    static = api.get_scene_static_data()
    bbox, normal_boxes = read_regions(static)
    print(f"[plan] 限制区={bbox} 正常区={normal_boxes}", flush=True)

    cands: list[dict] = []
    src = Path(args.survey) if args.survey else None
    if args.phase == "read" and src is not None and src.exists():
        data = json.loads(src.read_text(encoding="utf-8"))
        cands = [c for c in data.get("cars", []) if c.get("n", 0) >= 2]
        wps = plan_passes(cands, bbox) or perimeter(bbox, offset=args.offset)
        v_cruise = V_READ
        print(f"[read] 从 {src} 读到 {len(cands)} 辆候选 → 掠读航点 "
              f"{[(round(a, 1), round(b, 1)) for a, b in wps]}", flush=True)
    else:
        # survey 阶段，或 read 阶段没有地图：都沿走廊一圈，边走边读边投票
        wps = perimeter(bbox, offset=args.offset)
        v_cruise = V_SURVEY if args.phase == "survey" else V_READ
        if args.phase == "read":
            print("[read] 没有 survey.json → 走『边跑边读』模式", flush=True)
    print(f"[plan] 航点（{len(wps)}）= {[(round(a, 1), round(b, 1)) for a, b in wps]}",
          flush=True)

    csv_f = open(out / "run.csv", "w", newline="", encoding="utf-8")
    wr = csv.writer(csv_f)
    wr.writerow(["frame", "t", "x", "y", "yaw", "v", "thr", "brk", "steer",
                 "v_tgt", "wp_i", "ncars", "note"])

    patrol = Patrol(wps, v_cruise)
    votes: dict[str, dict] = {}
    frame = 0
    t0 = None
    submitted = False
    last_t = 0.0
    dt = 1.0 / 30.0
    shot_i = 0
    last_log = -1.0
    stuck_t = 0.0
    escape_t = 0.0

    try:
        for msg, cams in api.main_loop():
            frame += 1
            pose = msg.pose_gnss
            car = msg.main_vehicle
            t = float(msg.scene_status.used_time) if msg.scene_status else 0.0
            if t0 is None:
                t0 = t
            dt = max(1e-3, min(0.2, t - last_t)) if last_t else 1.0 / 30.0
            last_t = t
            x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
            v = float(car.speed)

            # ---- 视觉
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
            dets: list[dict] = []
            note = ""
            hrow = 240.0
            if img is not None and frame % 2 == 0:
                try:
                    hrow = float(horizon_row(img))
                except Exception:
                    hrow = 240.0
                dets = detect_cars(img)
                dets.sort(key=lambda d: -d["w"])
                for d in dets:
                    if d["w"] < MIN_BOX_W:
                        continue
                    px, py, dist, brg = det_world(d, x, y, yaw, hrow)
                    if args.phase == "survey":
                        if 2.0 <= dist <= 45.0:
                            merge(cands, px, py, d["w"])
                        continue
                    # read 阶段：只对近处（6~14 m）的目标读号
                    if not (5.0 <= dist <= 16.0):
                        continue
                    g = ground_class(img, d)
                    label, score, raw = read_label(img, d, min_score=MIN_OCR_SCORE)
                    if label:
                        rec = votes.setdefault(label, dict(score=0.0, hits=0,
                                                           x=px, y=py, shots=0))
                        rec["score"] = max(rec["score"], score)
                        rec["hits"] += 1
                        rec["x"], rec["y"] = px, py
                        if rec["shots"] < 2:
                            rec["shots"] += 1
                            shot_i += 1
                            imwrite_uni(shots / f"{shot_i:03d}_{label}_{score:.2f}.jpg", img)
                        note = f"{label}:{score:.2f}"
                        print(f"  [ocr] {label} score={score:.2f} d={dist:.1f}m "
                              f"pos=({px:.1f},{py:.1f}) grid={g.get('grid')}", flush=True)
                    elif d["w"] >= 60 and frame % 10 == 0 and shot_i < 400:
                        mx, my = d["w"], int(0.8 * d["h"])
                        cx0 = max(0, d["x"] - mx)
                        cx1 = min(img.shape[1], d["x"] + d["w"] + mx)
                        cy0 = max(0, d["y"] - my)
                        cy1 = min(img.shape[0], d["y"] + d["h"] + my)
                        if cx1 - cx0 > 24 and cy1 - cy0 > 16:
                            shot_i += 1
                            imwrite_uni(shots / (f"c{shot_i:04d}_w{d['w']}_d{int(dist)}m_"
                                                 f"raw-{raw or 'none'}.jpg"),
                                        img[cy0:cy1, cx0:cx1])

            # ---- 横向/纵向（含近车减速 + 卡死脱困）
            steer, err, dist_wp = patrol.steer(x, y, yaw)
            nearest, nearest_cx = 999.0, 320.0
            for d in dets[:3]:
                if d["w"] < nearest:
                    nearest = float(d["w"])
                    nearest_cx = d["x"] + d["w"] * 0.5
            # 前方近车：往空的一侧蹭过去（不停死）
            if nearest >= 120:
                side = -1.0 if nearest_cx >= 320 else 1.0
                steer = max(-1.0, min(1.0, steer + side * 0.30))
            v_t = patrol.v_target(err, dist_wp, nearest)
            thr, brk = longitudinal(v, v_t, dt)
            gear = GearMode.DRIVE

            # 卡死自救：给油却不动 → 倒车 1.4 s，出来后再往没车的一侧偏一点
            if v < 0.08 and thr > 0.10:
                stuck_t += dt
            else:
                stuck_t = max(0.0, stuck_t - dt)
            if stuck_t > 1.2 or escape_t > 0.0:
                escape_t += dt
                gear = GearMode.REVERSE
                thr, brk = 0.30, 0.0
                steer = -steer * 0.6
                if escape_t > 1.4:
                    escape_t = 0.0
                    stuck_t = 0.0
            vc = VehicleControl(gear=gear, throttle=float(thr),
                                brake=float(brk), steering=float(steer))

            # ---- 交卷（只在 read 阶段）
            elapsed = t - t0
            ready = elapsed >= SUBMIT_T or patrol.done
            if args.phase == "read" and ready and not submitted:
                ids = [k for k, r in votes.items()
                       if r["score"] >= MIN_OCR_SCORE and r["hits"] >= 1
                       and in_box(r["x"], r["y"], bbox, REGION_MARGIN)]
                print(f"[submit] t={elapsed:.1f}s 报 {len(ids)} 辆: {ids}", flush=True)
                if not args.dry and VLAExtensionOutput is not None:
                    try:
                        api.set_vehicle_control(
                            vc, VLAExtensionOutput(parking_result=ParkingResult(
                                violating_sticker_ids=list(ids))))
                        print("[submit] 已提交", flush=True)
                    except Exception as exc:                  # pragma: no cover
                        print(f"[submit] 提交失败 {exc!r}", flush=True)
                submitted = True
                vc = VehicleControl(gear=GearMode.PARKING, throttle=0.0, brake=1.0,
                                    steering=0.0)

            api.set_vehicle_control(vc)
            wr.writerow([frame, round(elapsed, 3), round(x, 3), round(y, 3),
                         round(yaw, 2), round(v, 3), round(thr, 3), round(brk, 3),
                         round(steer, 3), round(v_t, 2), patrol.i, len(dets), note])
            if int(elapsed) != int(last_log):
                last_log = elapsed
                print(f"[{elapsed:5.1f}s] x={x:6.2f} y={y:6.2f} yaw={yaw:6.1f} "
                      f"v={v:4.2f} wp={patrol.i} tgt={v_t:4.2f} thr={thr:.3f} "
                      f"brk={brk:.3f} cars={len(dets)} "
                      f"cand={len(cands)} votes={len(votes)}", flush=True)

            if (patrol.done and args.phase == "survey") or (submitted and elapsed > SUBMIT_T + 30.0):
                break
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
        if args.phase == "survey":
            cands.sort(key=lambda c: (-c["y"], c["x"]))
            for c in cands:
                c["in_restricted"] = in_box(c["x"], c["y"], bbox, REGION_MARGIN)
            payload = dict(bbox=bbox, normal=normal_boxes, cars=cands,
                           vla_perimeter=wps, offset=args.offset)
            (out / "survey.json").write_text(
                json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            n_in = sum(1 for c in cands if c["in_restricted"])
            print(f"[survey] 检测到 {len(cands)} 辆车（其中 {n_in} 辆位置落在限制区内）",
                  flush=True)
            for i, c in enumerate(cands, 1):
                print(f"  #{i} ({c['x']:.1f},{c['y']:.1f}) n={c['n']} "
                      f"restricted={c['in_restricted']} wmax={c['w']}", flush=True)
        else:
            summary = {k: dict(score=round(r["score"], 3), hits=r["hits"],
                               pos=(round(r["x"], 1), round(r["y"], 1)))
                       for k, r in votes.items()}
            print("[votes]", summary, flush=True)
            (out / "votes.txt").write_text(str(summary), encoding="utf-8")
        print(f"[done] frames={frame} out={out}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

# -*- coding: utf-8 -*-
"""VLA 停车区任务（子任务二）：读号与交卷的**三层骨架**（与 vla_stack.py 同构）。

逻辑链：perceive → associate → assess → select → approach → read → vote → submit
决策层：按 prize/cost 选目标；读不读由 `p_read(d) × (2·p_v − 1)` 决定（自动排除合法车）
算法层：色彩掩膜检测、地面反投影、几何违规似然、白字模板匹配、投票与一次性交卷

本文件只做"结构 + 可算的纯函数"，不连仿真、不落任何写死的地点/时刻。
运行入口 `main()` 需要真实场景（等具备执行权限后再说）。

    .venv\\Scripts\\python.exe <ws>\\vla_read.py --out <ws>\\vla_read
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

# —— 依赖 metacar-race 里的视觉工具（中文路径安全的读写已封装在 vla2/vision.py）——
import sys

RACE = Path(r"D:\无人算法\metacar-race")
if str(RACE) not in sys.path:
    sys.path.insert(0, str(RACE))

from metacar import GearMode, SceneAPI, VehicleControl  # noqa: E402

try:                                                        # 视觉工具可用性
    from vla2.vision import detect_cars, read_label          # type: ignore
    _HAS_VISION = True
except Exception:                                           # pragma: no cover
    _HAS_VISION = False

    def detect_cars(img, **kw):
        return []

    def read_label(img, **kw):
        return (None, -1.0, "")

# ============================ 规则/几何常数 ============================
SPEED_LIMIT_MS = 30 / 3.6     # 规则：30 km/h
A_LIMIT = 5.0                 # 规则：|a| ≤ 5
TASK_SECONDS = 100.0          # 规则：100 s

D_REF = 6.5                   # 侧向掠过距离（p_read 峰值附近，几何）
READ_CONF_MIN = 0.60          # 读号置信度下限
VOTE_MIN_HITS = 2             # 同一 id 需要的票数
VOTE_SINGLE_OK = 0.85         # 单票即定案的置信度
SUBMIT_MARGIN_S = 8.0         # 交卷成本（停车 + 提交）


# ============================ 数据 ============================
@dataclass
class CarObs:
    key: int
    x: float
    y: float
    ori: float
    best_px: float = 0.0
    hits: int = 0
    p_v: float = 0.5
    label: str | None = None
    label_score: float = 0.0
    votes: dict = field(default_factory=dict)


@dataclass
class View:
    img: object
    hrow: int
    K: float                  # 地面反投影系数：d ≈ K/(y - hrow)
    fov_h_deg: float = 70.0


# ============================ 算法层：反投影 / 违规似然 / 可读性 ============================
def backproject(u: float, y: float, view: View) -> tuple[float, float]:
    """图像点 → 车体系（前向 d、右向 s）。地面假设：y > hrow。"""
    if y <= view.hrow + 2:
        return (0.0, 0.0)
    d = view.K / (y - view.hrow)
    s = d * math.tan(math.radians(view.fov_h_deg) / 2) * (2.0 * u / 640.0 - 1.0)
    return (d, s)


def to_world(x: float, y: float, yaw: float, d: float, s: float) -> tuple[float, float]:
    """车体系(d 前, s 右) → 世界坐标。场景航向：0=+x，90=-y，右 = 航向 +90°。"""
    yr = math.radians(yaw)
    fx, fy = math.cos(yr), -math.sin(yr)
    rx, ry = -fy, fx
    return (x + d * fx + s * rx, y + d * fy + s * ry)


def overlap_ratio(cx: float, cy: float, cl: float, cw: float, ori: float,
                  bx0: float, by0: float, bx1: float, by1: float) -> float:
    """车矩形与禁停区矩形的重叠面积比（把车近似成轴对齐的外接框，够用且不引入旋转噪声）。"""
    ex = (abs(math.cos(math.radians(ori))) * cl + abs(math.sin(math.radians(ori))) * cw) / 2
    ey = (abs(math.sin(math.radians(ori))) * cl + abs(math.cos(math.radians(ori))) * cw) / 2
    ix = max(0.0, min(cx + ex, bx1) - max(cx - ex, bx0))
    iy = max(0.0, min(cy + ey, by1) - max(cy - ey, by0))
    area = max(1e-6, 2 * ex * 2 * ey)
    return (ix * iy) / area


def p_violation(cx, cy, cl, cw, ori, restricted, legal_band) -> float:
    """违规似然：占进禁停区 / 越出合法车位带，两条取大。"""
    x0, y0, x1, y1 = restricted
    ov = overlap_ratio(cx, cy, cl, cw, ori, x0, y0, x1, y1)
    p = 0.5 + 0.5 * (ov - 0.15) / 0.55
    if legal_band is not None:
        lx0, ly0, lx1, ly1 = legal_band
        ex = (abs(math.cos(math.radians(ori))) * cl + abs(math.sin(math.radians(ori))) * cw) / 2
        ey = (abs(math.sin(math.radians(ori))) * cl + abs(math.cos(math.radians(ori))) * cw) / 2
        out = max(lx0 - (cx - ex), (cx + ex) - lx1, ly0 - (cy - ey), (cy + ey) - ly1, 0.0)
        if out > 0.15:
            p = max(p, 0.55)
    return max(0.02, min(0.98, p))


class ReadEstimator:
    """p_read(d) = 1/(1+exp((d-d50)/k))，用现场样本在线拟合（默认值来自 f2 实测两点）。"""

    def __init__(self, d50: float = 7.8, k: float = 0.30):
        self.d50, self.k = d50, k
        self.samples: list[tuple[float, bool]] = []

    def p_read(self, d: float) -> float:
        return 1.0 / (1.0 + math.exp((d - self.d50) / max(1e-3, self.k)))

    def add(self, d: float, ok: bool) -> None:
        self.samples.append((d, ok))
        if len(self.samples) >= 4:
            best, best_err = (self.d50, self.k), 1e9
            for d50 in [x * 0.5 for x in range(6, 31)]:          # 3~15 m
                for k in [0.15, 0.20, 0.25, 0.30, 0.40, 0.50]:
                    err = 0.0
                    for dd, o in self.samples:
                        p = 1.0 / (1.0 + math.exp((dd - d50) / k))
                        err += (p - (1.0 if o else 0.0)) ** 2
                    if err < best_err:
                        best, best_err = (d50, k), err
            self.d50, self.k = best


# ============================ 决策层：选目标 / 交卷时机 ============================
class Selector:
    def __init__(self, est: ReadEstimator, v_now: float = 2.5):
        self.est, self.v_now = est, v_now

    def prize(self, car: CarObs, d: float) -> float:
        return self.est.p_read(d) * (2.0 * car.p_v - 1.0)

    def rank(self, cars: list[CarObs], ego: tuple[float, float],
             remaining: float) -> list[tuple[float, CarObs, float]]:
        out = []
        for c in cars:
            if c.label is not None:
                continue
            d = math.hypot(c.x - ego[0], c.y - ego[1])
            cost = d / max(0.5, self.v_now) + 2.5
            pz = self.prize(c, d)
            if pz <= 0.0 or cost > remaining - SUBMIT_MARGIN_S:
                continue
            out.append((pz / cost, c, d))
        out.sort(key=lambda t: -t[0])
        return out


def should_submit(cars: list[CarObs], t: float) -> tuple[bool, str]:
    remain = TASK_SECONDS - t
    pending = [c for c in cars if c.p_v > 0.5 and c.label is None]
    if not pending:
        return True, "信息饱和：所有高违规似然候选已定案"
    if remain < SUBMIT_MARGIN_S + 5.0:
        return True, f"预算耗尽：剩 {remain:.1f}s"
    if t > 0.90 * TASK_SECONDS:
        return True, "硬底线 0.90×时限"
    return False, ""


# ============================ 逻辑链：投票与清单 ============================
class VoteBox:
    def __init__(self) -> None:
        self.data: dict[int, CarObs] = {}

    def observe(self, key: int, x: float, y: float, ori: float, px: float,
                p_v: float) -> CarObs:
        c = self.data.get(key)
        if c is None:
            c = CarObs(key=key, x=x, y=y, ori=ori)
            self.data[key] = c
        c.x, c.y, c.ori = x, y, ori
        c.hits += 1
        c.best_px = max(c.best_px, px)
        c.p_v = max(c.p_v, p_v)
        return c

    def add_read(self, key: int, label: str | None, score: float) -> None:
        c = self.data.get(key)
        if c is None or not label:
            return
        c.votes[label] = c.votes.get(label, 0.0) + score
        best = max(c.votes.items(), key=lambda kv: kv[1])
        n_same = sum(1 for k, v in c.votes.items() if k == best[0])
        if best[1] >= VOTE_SINGLE_OK * VOTE_MIN_HITS or n_same >= 1:
            c.label, c.label_score = best[0], best[1]

    def violating(self) -> list[str]:
        ids = []
        for c in self.data.values():
            if c.p_v > 0.5 and c.label:
                ids.append(c.label)
        uniq: dict[str, int] = {}
        for i in ids:
            uniq[i] = uniq.get(i, 0) + 1
        return sorted([i for i, n in uniq.items() if n == 1])     # 同一 id 挂在两辆车上 ⇒ 冲突不报


# ============================ 真机入口（等权限再跑） ============================
def region_of(static) -> tuple[tuple, tuple | None]:
    ext = getattr(static, "vla_extension", None)
    restr, legal = [], []
    for r in list(getattr(ext, "regions", []) or []):
        nm = getattr(getattr(r, "type", None), "name", "?")
        box = (r.pos_x - r.length / 2, r.pos_y - r.width / 2,
               r.pos_x + r.length / 2, r.pos_y + r.width / 2)
        if nm in ("RESTRICTED_PARKING", "FUNCTION_ZONE"):
            restr.append(box)
        elif nm == "NORMAL_PARKING":
            legal.append(box)
    merged = (min(b[0] for b in restr), min(b[1] for b in restr),
              max(b[2] for b in restr), max(b[3] for b in restr)) if restr else (0, 0, 0, 0)
    if legal:
        band = (min(b[0] for b in legal), min(b[1] for b in legal),
                max(b[2] for b in legal), max(b[3] for b in legal))
    else:
        band = None
    return merged, band


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--K", type=float, default=0.0, help="地面反投影系数，0=用默认")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    if not _HAS_VISION:
        print("!! vla2.vision 不可用，先修环境再跑")
        return 2

    api = SceneAPI()
    print("waiting scene ...", flush=True)
    api.connect()
    restr, band = region_of(api.get_scene_static_data())
    print(f"禁停区 {restr} 合法带 {band}")

    est = ReadEstimator()
    sel = Selector(est)
    box = VoteBox()
    submitted = False
    log = []

    try:
        for msg, cams in api.main_loop():
            car = msg.main_vehicle
            t = float(getattr(msg.scene_status, "used_time", 0.0))
            img = None
            try:
                for c in cams:
                    fr = getattr(c, "frame", None)
                    if fr is not None and getattr(c, "id", 0) == 0:
                        img = fr
                        break
            except Exception:
                img = None

            # ① perceive + associate + assess
            if img is not None:
                view = View(img=img, hrow=175, K=(args.K or 2200.0))
                for i, b in enumerate(detect_cars(img)):
                    u = b["x"] + b["w"] / 2.0
                    d, s = backproject(u, b["y"] + b["h"], view)
                    if d <= 0.5:
                        continue
                    wx, wy = to_world(car.pos_x, car.pos_y, car.ori_z, d, s)
                    pv = p_violation(wx, wy, 4.4, 1.8, 0.0, restr, band)
                    key = int(round(wx)) * 1000 + int(round(wy))      # 位置量化成簇 id
                    box.observe(key, wx, wy, 0.0, max(b["w"], b["h"]), pv)
                    # ⑥ read（只对值得读、且已进入可读窗口的车）
                    cand = box.data[key]
                    if cand.p_v > 0.5 and math.hypot(cand.x - car.pos_x,
                                                     cand.y - car.pos_y) <= 11.0:
                        lab, sc, raw = read_label(img, roi=(b["x"], b["y"], b["w"], b["h"]))
                        if lab and sc >= READ_CONF_MIN:
                            box.add_read(key, lab, sc)
                        est.add(d, bool(lab and sc >= READ_CONF_MIN))

            # ⑤ select（只记录意图，真正的机动在 planner 里；本骨架先停车等待）
            rank = sel.rank(list(box.data.values()), (car.pos_x, car.pos_y),
                            TASK_SECONDS - t)
            ctl = VehicleControl()
            ctl.gear = GearMode.DRIVE
            ctl.throttle = 0.0
            ctl.brake = 0.05
            ctl.steering = 0.0
            api.set_vehicle_control(ctl)
            log.append({"t": round(t, 2), "cars": len(box.data), "top": len(rank),
                        "p_read_d50": est.d50})

            # ⑦ submit
            done, why = should_submit(list(box.data.values()), t)
            if done and not submitted:
                ids = box.violating()
                from metacar import ParkingResult, VLAExtensionOutput
                out_msg = VLAExtensionOutput(parking_result=ParkingResult(
                    violating_sticker_ids=ids))
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING,
                                                       throttle=0.0, brake=1.0,
                                                       steering=0.0), out_msg)
                submitted = True
                print(f"[{t:6.1f}s] 提交 {ids} （{why}）")
                break
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, throttle=0.0,
                                                  brake=1.0, steering=0.0))
        except Exception:
            pass
        (out / "log.json").write_text(json.dumps(log[-200:], ensure_ascii=False, indent=1),
                                     encoding="utf-8")
        (out / "inventory.json").write_text(json.dumps(
            [{"key": c.key, "x": round(c.x, 2), "y": round(c.y, 2), "p_v": round(c.p_v, 2),
              "label": c.label} for c in box.data.values()], ensure_ascii=False, indent=1),
            encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

# -*- coding: utf-8 -*-
"""VLA 停车区任务（子任务一）：**三层走通** —— 逻辑链 / 决策层 / 算法层。

这一支代码只做一件事：在禁停区（40×40 m）里贴着内侧边缘稳稳绕一圈，不出界、不撞车。

三层分离（每层只依赖上一层给的量，便于单独验证）：

  ① 逻辑链  Chain        ：perceive → locate → plan → decide → actuate → log
                            固定顺序、固定接口；任何一层给不出量就降级并记录原因。
  ② 决策层  Arbiter      ：L0 安全（视觉走廊） > L1 贴合（横向偏差） > L2 节奏（曲率限速）
                            输出唯一 (v_t, steer, mode, reason)，**不做时间轴/不做记忆**。
  ③ 算法层  RefPath / Corridor / Filters
                            圆角矩形参考线、三点圆曲率、前馈+双反馈、一阶低通与斜率限幅。

`Runner.tick(state, img)` 是纯函数式的一步：输入位姿与图像，输出控制指令与理由。
因此**同一份决策与算法可以在两种车上跑**：
  * 真实：`main()` 里从 `SceneAPI.main_loop()` 取 `msg/cams`；
  * 离线：`vla_stack_sim.py` 用自行车运动学模型喂 `tick`，不连仿真。

用法（在 metacar-race 目录下）：
  .venv\\Scripts\\python.exe <ws>\\vla_stack.py --tag s1 --out <ws>\\vla_stack
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from metacar import GearMode, SceneAPI, VehicleControl

# ============================ 车与规则的常数（只写这三类） ============================
WHEELBASE = 2.6      # 轴距（车）
MAX_STEER = 0.6      # 归一化舵角对应的最大前轮转角（rad，车）
V_CAP = 5.5          # 规则：≤20 km/h
A_LIMIT = 5.0        # 规则：|a| ≤ 5 m/s² 才不扣分

# ============================ 算法层：参考线与跟踪 ============================
E_REF = 3.80         # 857.14 那一版（s14：两圈、报 3 中 3、零扣分）的贴边值——用户锁版，不再改
R_CORNER = 8.00      # 圆角半径（车最小转弯半径 4.6 的 1.7 倍）
K_HEAD = 0.017       # 归一化舵角 / 度（0.020 时航向误差一有就到 0.3 舵，弯里必过冲）
K_CROSS = 1.20       # Stanley 横向增益（1/米）——1.30 时往里切太多，回正又打过头
K_YAW = 0.004        # 横摆角速度阻尼（归一化舵角 / (deg/s)）——专治"收回来又打多了"
V_STANLEY_FLOOR = 1.00  # Stanley 归一速度下限（避免低速横向项爆炸）
STEER_SLEW = 0.025   # 每帧最大舵角变化（0.040 时 80% 帧在满速打舵 → 晃）
STEER_CLAMP = 0.58   # 舵角指令硬上限（物理满舵 0.6，留 0.05 不进饱和死区）
STEER_LP = 0.70      # 一阶低通保留比例（慢响应）
V_STRAIGHT = 3.00    # 直道巡航 m/s（≤20 km/h 规则内）
A_LAT = 0.75         # 弯道横向加速度预算 → R=8 时弯速 2.45 m/s
V_MIN = 2.40         # 弯内最低速度（跳出仿真 1.2~2.2 m/s 速度跳变带，才不丢 −50 加速度分）
A_ACC = 1.0          # 纵向加速度指令上限（远低于扣分线 5）
A_DEC = 0.80         # 纵向减速度指令上限
A_TAU = 0.50         # 加速度指令一阶低通时间常数（s）—— 消掉开关式给油
A_DEAD = 0.12        # 加速度指令死区（配合速度误差门限形成滞环）
DT = 0.075           # 控制周期估计（13 fps），只用于低通系数
THR_SLEW_UP = 0.004      # 每帧油门上升上限（≈0.12/s）—— 只让"给油"慢
THR_SLEW_DOWN = 0.040    # 每帧油门回落上限 —— "收油"必须快，否则冲过头再慢慢掉
THR_SLEW_LAUNCH = 0.020  # 起步阶段允许更快的油门建立（否则 3 s 都起不来）
BRK_SLEW_UP = 0.020      # 踩刹车快（安全）
BRK_SLEW_DOWN = 0.010    # 松刹车稍慢
K_SPEED = 0.50           # 速度误差 → 油门比例（1.0 m/s 误差即给到上限）
LOOK_FF = (4.0, 9.0)  # 前馈曲率取窗（米）
THR_MAX = 0.05
BRK_MAX = 0.02
H_HORIZON = 175


def wrap180(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def yaw_of(dx: float, dy: float) -> float:
    """场景航向：0=+x，90=-y。"""
    return math.degrees(math.atan2(-dy, dx))


def build_reference(x0: float, y0: float, x1: float, y1: float,
                    inset: float = E_REF, r: float = R_CORNER, ds: float = 0.5):
    """圆角矩形参考线（南→东→北→西）。圆弧起点天然在角点前 R·tan(22.5°)≈2.0 m。"""
    ax0, ay0, ax1, ay1 = x0 + inset, y0 + inset, x1 - inset, y1 - inset
    r = min(r, 0.45 * (ax1 - ax0), 0.45 * (ay1 - ay0))
    pts: list[tuple[float, float]] = []

    def line(p, q):
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        n = max(1, int(d / ds))
        pts.extend((p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n)
                   for i in range(n))

    def arc(c, a0, a1):
        n = max(2, int(abs(a1 - a0) / 180.0 * math.pi * r / ds))
        pts.extend((c[0] + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
                    c[1] + r * math.sin(math.radians(a0 + (a1 - a0) * i / n)))
                   for i in range(n))

    line((ax0 + r, ay0), (ax1 - r, ay0)); arc((ax1 - r, ay0 + r), -90.0, 0.0)
    line((ax1, ay0 + r), (ax1, ay1 - r)); arc((ax1 - r, ay1 - r), 0.0, 90.0)
    line((ax1 - r, ay1), (ax0 + r, ay1)); arc((ax0 + r, ay1 - r), 90.0, 180.0)
    line((ax0, ay1 - r), (ax0, ay0 + r)); arc((ax0 + r, ay0 + r), 180.0, 270.0)
    return pts


class RefPath:
    """闭环折线：投影（返回 e_lat）、前方平均曲率。"""

    def __init__(self, pts, ds: float = 0.5):
        self.p, self.n, self.ds = pts, len(pts), ds

    def project(self, x: float, y: float):
        best = (1e9, 0, 1.0, 0.0)
        for i in range(self.n):
            a, b = self.p[i], self.p[(i + 1) % self.n]
            vx, vy = b[0] - a[0], b[1] - a[1]
            L2 = vx * vx + vy * vy
            if L2 < 1e-9:
                continue
            t = max(0.0, min(1.0, ((x - a[0]) * vx + (y - a[1]) * vy) / L2))
            px, py = a[0] + vx * t, a[1] + vy * t
            d = math.hypot(x - px, y - py)
            if d < best[0]:
                L = math.sqrt(L2)
                best = (d, i, vx / L, vy / L)
        d, i, tx, ty = best
        e_lat = (x - self.p[i][0]) * (-ty) + (y - self.p[i][1]) * tx   # 左正
        return i, tx, ty, d, e_lat

    def curvature_ahead(self, i: int, d0: float = LOOK_FF[0],
                        d1: float = LOOK_FF[1]) -> float:
        a = self.p[(i + int(d0 / self.ds)) % self.n]
        b = self.p[(i + int((d0 + d1) / 2 / self.ds)) % self.n]
        c = self.p[(i + int(d1 / self.ds)) % self.n]
        ab = math.hypot(b[0] - a[0], b[1] - a[1])
        bc = math.hypot(c[0] - b[0], c[1] - b[1])
        ca = math.hypot(a[0] - c[0], a[1] - c[1])
        if min(ab, bc, ca) < 1e-6:
            return 0.0
        area = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) * 0.5
        return 0.0 if area < 1e-6 else 4.0 * area / (ab * bc * ca)


def horizon_of(img) -> int:
    sat = img[:, :, 1].astype(np.float32).mean(axis=1)
    for v in range(int(0.25 * img.shape[0]), int(0.65 * img.shape[0])):
        if sat[v] < 70.0:
            return v
    return H_HORIZON


# ============================ 算法层：走廊感知 ============================
@dataclass
class Corridor:
    occ: float = 0.0        # 正前方占用（0~1）
    gap_bins: int = 64      # 最宽自由区间宽度（bin）
    free_deg: float = 0.0   # 最宽区间中心方向（度，右正）

    @classmethod
    def analyse(cls, img, nb: int = 64) -> "Corridor":
        if img is None:
            return cls(0.0, nb, 0.0)
        h0 = horizon_of(img)
        y0 = int(min(img.shape[0] - 2, h0 + 120))
        y1 = int(min(img.shape[0], h0 + 300))
        if y1 - y0 < 20:
            return cls(0.0, nb, 0.0)
        band = img[y0:y1, :, :].max(axis=2).astype(np.int16) < 105
        col = band.mean(axis=0)
        idx = np.linspace(0, img.shape[1], nb + 1).astype(int)
        prof = np.array([col[idx[i]:idx[i + 1]].mean() if idx[i + 1] > idx[i] else 0.0
                         for i in range(nb)], np.float32)
        c = nb // 2
        lo, hi, best, i = max(0, c - 20), nb - 1, (c, 0), 0
        while i <= hi:
            if prof[i] < 0.12:
                j = i
                while j + 1 <= hi and prof[j + 1] < 0.12:
                    j += 1
                if j - i + 1 > best[1]:
                    best = ((i + j) // 2, j - i + 1)
                i = j + 1
            else:
                i += 1
        cb, w = best
        ang = (cb - c) / c * 55.0
        return cls(float(prof[c]), w, ang)


# ============================ 决策层 ============================
@dataclass
class Decision:
    v_t: float
    steer: float
    throttle: float
    brake: float
    mode: str
    reason: list[str] = field(default_factory=list)


class Arbiter:
    """L0 安全 > L1 贴合 > L2 节奏。速度是被动量，永远只被削减。"""

    def __init__(self, path: RefPath):
        self.path = path
        self.steer_prev = 0.0
        self.thr_prev = 0.0
        self.brk_prev = 0.0
        self.a_f = None
        self.yaw_prev = None
        self.dt_est = DT

    def decide(self, x, y, yaw, v, cor: Corridor) -> tuple[Decision, dict]:
        i, tx, ty, dist, e_lat = self.path.project(x, y)
        e_head = wrap180(yaw_of(tx, ty) - yaw)
        kappa = self.path.curvature_ahead(i)             # 3~6 m：给限速用
        # 前馈用"马上要到的曲率"，取 2~8 m 的加权平均：看远一点、少靠反馈纠偏
        kappa_ff = (0.6 * self.path.curvature_ahead(i, 3.0, 6.0)
                    + 0.4 * self.path.curvature_ahead(i, 6.0, 9.0))
        kappa_v = max(kappa, self.path.curvature_ahead(i, 6.0, 12.0))   # 6~12 m：给限速用
        trace = dict(idx=i, e_lat=e_lat, e_head=e_head, kappa=kappa,
                     kappa_ff=kappa_ff, kappa_v=kappa_v, dist=dist)

        # —— L2 节奏：曲率限速（用更远的曲率，提前把速度压下来，否则进弯会跑宽）——
        r_loc = 1e3 if kappa_v < 1e-3 else 1.0 / kappa_v
        v_t = min(V_STRAIGHT, V_CAP, max(V_MIN, math.sqrt(max(A_LAT * r_loc, 0.0))))
        reason = [f"curve:v_t={v_t:.2f}(R={r_loc:.1f})"]

        # —— L1 贴合：曲率前馈 + Stanley（航向 + 横向，横向按速度归一后饱和到 90°）——
        steer_ff = 1.00 * math.atan(WHEELBASE * kappa_ff) / MAX_STEER
        # 2026-10-27 23:05 全部回退：死区/弱回打/直道零航向/只弯里阻尼 都删掉，
        # 恢复"原始 Stanley + 全时段横摆阻尼"（唯一稳定跑过 857.14 的那套）。
        # 2026-10-27 23:2x 用户锁版：一律用 vla_stack.py 顶部默认常数（E_REF=3.80 等），
        # 不再用环境变量覆写、不再加任何"被堵自救"之类的新逻辑。
        lat_term = math.atan2(K_CROSS * e_lat, max(v, V_STANLEY_FLOOR))
        fb = (math.radians(e_head) + lat_term) / MAX_STEER
        steer_raw = steer_ff + fb
        # 横摆阻尼：车已经在转（ω 大）就少打舵，专治"进弯切太多、回正又打过头"
        if self.yaw_prev is None:
            yaw_rate = 0.0
            self.yaw_prev = yaw
        else:
            yaw_rate = wrap180(yaw - self.yaw_prev) / max(self.dt_est, 1e-3)
            self.yaw_prev = yaw
        steer_raw -= K_YAW * yaw_rate
        reason.append(f"track:ff={steer_ff:+.2f} head={e_head:+.1f} "
                      f"lat={math.degrees(lat_term):+.1f}")

        # —— L0 安全：走廊否决（只降速 + 朝空档偏，不倒车）——
        if cor.gap_bins < 12:
            steer_raw = 0.55 * steer_raw + 0.45 * (cor.free_deg / 45.0)
            v_t = min(v_t, 1.4)
            reason.append(f"corridor:gap={cor.gap_bins} free={cor.free_deg:+.0f}° v≤1.4")
        if cor.gap_bins < 7:
            v_t = min(v_t, 0.8)
            reason.append("corridor:gap<7 v≤0.8")

        # —— 慢响应：斜率限幅 + 硬钳（不进饱和死区）——
        # 只做斜率限幅（若再串一阶低通，两者相乘会把实际速率压到 1/3，进场要 13 s 才偏得过来）
        steer = self.steer_prev + max(-STEER_SLEW, min(STEER_SLEW, steer_raw - self.steer_prev))
        steer = max(-STEER_CLAMP, min(STEER_CLAMP, steer))
        self.steer_prev = steer

        # —— 纵向：速度误差 → 油门（收油比给油快）+ 加速度空间刹车 ——
        err_v = v_t - v
        thr_t = max(0.0, min(THR_MAX, K_SPEED * err_v))
        a_des = max(-A_DEC, min(A_ACC, 1.2 * err_v))
        if self.a_f is None:
            self.a_f = a_des
        alpha = DT / (A_TAU + DT)
        a_cmd = self.a_f + alpha * (a_des - self.a_f)
        self.a_f = a_cmd
        brk_t = max(0.0, min(BRK_MAX, -a_cmd / 32.0)) if err_v < -0.15 else 0.0
        slew_up = THR_SLEW_LAUNCH if v < 0.4 else THR_SLEW_UP
        d_thr = thr_t - self.thr_prev
        thr = self.thr_prev + max(-THR_SLEW_DOWN, min(slew_up, d_thr))
        d_brk = brk_t - self.brk_prev
        brk = self.brk_prev + max(-BRK_SLEW_DOWN, min(BRK_SLEW_UP, d_brk))
        if v < 0.4 and v_t > 0.8:
            thr = max(thr, 0.12)
            reason.append("launch:bite")
        self.thr_prev = thr
        self.brk_prev = brk

        mode = "run"
        if cor.gap_bins < 7:
            mode = "avoid"
        elif kappa > 0.10:
            mode = "corner"
        return Decision(v_t, steer, thr, brk, mode, reason), trace


# ============================ 逻辑链：一步 ============================
class Runner:
    """把三层串成一条固定顺序的链：perceive → locate → decide → actuate。"""

    def __init__(self, region: tuple[float, float, float, float],
                 inset: float | None = None, r: float | None = None):
        self.region = region
        self.path = RefPath(build_reference(
            *region,
            inset=E_REF if inset is None else inset,
            r=R_CORNER if r is None else r))
        self.arb = Arbiter(self.path)
        self.s_along = 0.0
        self.i_prev = None
        self.lap_done = False
        self.perimeter = self.path.n * self.path.ds

    def tick(self, x, y, yaw, v, img=None) -> tuple[Decision, dict]:
        # ① perceive
        cor = Corridor.analyse(img)
        # ② locate（投影 & 里程累计）
        i, _tx, _ty, _d, _e = self.path.project(x, y)
        if self.i_prev is not None and abs(_e) < 3.0:   # 离得远时最近点会跳，里程不算
            self.s_along += ((i - self.i_prev) % self.path.n) * self.path.ds
        self.i_prev = i
        # ③ decide
        dec, trace = self.arb.decide(x, y, yaw, v, cor)
        trace.update(cor=cor, s_along=self.s_along, mode=dec.mode,
                     reason="; ".join(dec.reason))
        if self.s_along >= self.perimeter:
            self.lap_done = True
        return dec, trace


# ============================ 真机入口 ============================
def region_of(static) -> tuple[float, float, float, float]:
    reg = getattr(static, "vla_extension", None)
    xs0, ys0, xs1, ys1 = [], [], [], []
    for r in list(getattr(reg, "regions", []) or []):
        nm = getattr(getattr(r, "type", None), "name", "?")
        if nm in ("RESTRICTED_PARKING", "FUNCTION_ZONE"):
            xs0.append(r.pos_x - r.length / 2); xs1.append(r.pos_x + r.length / 2)
            ys0.append(r.pos_y - r.width / 2); ys1.append(r.pos_y + r.width / 2)
    return (min(xs0), min(ys0), max(xs1), max(ys1))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="s1")
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=150.0)
    args = ap.parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)

    api = SceneAPI()
    print("waiting scene ...", flush=True)
    api.connect()
    region = region_of(api.get_scene_static_data())
    print(f"禁停区 {region}")
    runner = Runner(region)
    print(f"参考线 {runner.path.n} 点 周长 {runner.perimeter:.1f} m "
          f"贴边 {E_REF} m 圆角 R={R_CORNER}")

    f = (out / "run.csv").open("w", newline="", encoding="utf-8")
    w = csv.writer(f)
    w.writerow(["t", "x", "y", "yaw", "v", "v_t", "thr", "brk", "steer",
                "e_lat", "e_head", "kappa", "occ", "gap", "mode", "s_along"])
    t_done = None
    try:
        for msg, cams in api.main_loop():
            car = msg.main_vehicle
            img = None
            try:
                for c in cams:
                    fr = getattr(c, "frame", None)
                    if fr is not None and getattr(c, "id", 0) == 0:
                        img = fr
                        break
            except Exception:
                img = None
            t = float(getattr(msg.scene_status, "used_time", 0.0))
            dec, tr = runner.tick(msg.pose_gnss.pos_x, msg.pose_gnss.pos_y, msg.pose_gnss.ori_z, float(car.speed), img)

            ctl = VehicleControl()
            ctl.gear, ctl.throttle, ctl.brake = GearMode.DRIVE, dec.throttle, dec.brake
            ctl.steering = dec.steer
            api.set_vehicle_control(ctl)

            w.writerow([f"{t:.3f}", f"{msg.pose_gnss.pos_x:.2f}", f"{msg.pose_gnss.pos_y:.2f}",
                        f"{msg.pose_gnss.ori_z:.1f}", f"{car.speed:.2f}", f"{dec.v_t:.2f}",
                        f"{dec.throttle:.3f}", f"{dec.brake:.3f}", f"{dec.steer:.3f}",
                        f"{tr['e_lat']:.2f}", f"{tr['e_head']:.1f}",
                        f"{tr['kappa']:.3f}", f"{tr['cor'].occ:.2f}",
                        tr["cor"].gap_bins, dec.mode, f"{tr['s_along']:.1f}"])
            if int(t * 10) % 30 == 0 and abs(t - round(t)) < 0.02:
                print(f"[{t:6.1f}s] ({msg.pose_gnss.pos_x:6.2f},{msg.pose_gnss.pos_y:6.2f}) "
                      f"v={car.speed:.2f}/{dec.v_t:.2f} e_lat={tr['e_lat']:+.2f} "
                      f"k={tr['kappa']:.3f} gap={tr['cor'].gap_bins} "
                      f"{dec.mode} s={tr['s_along']:.0f}")
            if runner.lap_done:
                t_done = t if t_done is None else t_done
                if t - t_done > 1.0:
                    break
            if t > args.seconds:
                break
    finally:
        try:
            ctl = VehicleControl()
            ctl.gear, ctl.throttle, ctl.brake, ctl.steering = GearMode.PARKING, 0.0, 1.0, 0.0
            api.set_vehicle_control(ctl)
        except Exception:
            pass
        f.close()
        summary = {"tag": args.tag, "region": region, "perimeter": round(runner.perimeter, 1),
                   "s_along": round(runner.s_along, 1), "lap_done": runner.lap_done}
        (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2),
                                          encoding="utf-8")
        print("SUMMARY", json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

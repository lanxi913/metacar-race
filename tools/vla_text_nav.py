# -*- coding: utf-8 -*-
r"""VLA 场景·文本导航与规范停车任务。

评分口径（《VLA场景任务说明与评分规则》4.1 + 附录 A.4）：

    总分 = 多模态理解与表达 400（OCR ≤160 / 时间 ≤80 / 位置 ≤80 / 行动 ≤80）
         + 任务完成 600（在楼门口 8 m x 8 m 区域内停稳，或指定车位规范泊车）
    单任务 > 100 s 判失败；通用扣分：>30 km/h -50、|a|>5 -50、碰撞 -100

流程：
    1) OCR 读任务牌 -> 拆 时间/地点/行动 四段 -> 提交 VLATextOutput（可多次，最后一次为准）
    2) 地点片段匹配 SceneStaticData.vla_extension.buildings（绝对坐标由 JSON 给定）
       -> 目标点 = 楼中心沿"短边方向"朝自车那侧外移（门开在长边上）
    3) VLA 场景没有 route/roads/终点，自己用栅格 A* 规划（建筑外扩后当障碍），
       按弧长选目标点的纯跟踪执行
    4) 纵向控制沿用连续场景 650 分那版：起步油门 0.006、按速度给上限、
       刹车 <= 0.10、切 PARKING 前 |v| <= 0.06
    5) 进入判定区后刹停 -> 驻车 -> 退出

用法：
    .venv\\Scripts\\python.exe -u tools\\vla_text_nav.py --probe
    .venv\\Scripts\\python.exe -u tools\\vla_text_nav.py --probe --explore 45
    .venv\\Scripts\\python.exe -u tools\\vla_text_nav.py --text "限时100秒内抵达锦汇华庭B栋一单元门口停车，行驶全程时速小于等于30km/h"
    .venv\\Scripts\\python.exe -u tools\\vla_text_nav.py
"""
from __future__ import annotations

import argparse
import heapq
import math
import sys
import time
from pathlib import Path
from typing import NamedTuple

import cv2
import numpy as np

from metacar import GearMode, SceneAPI, VehicleControl, VLAExtensionOutput, VLATextOutput

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from metacar_race.perception.ocr_engine import backends as ocr_backends, read_text  # noqa: E402
from metacar_race.perception.vla_text import match_place, normalize, split_instruction  # noqa: E402

REPO = _REPO

V_MAX_MS = 8.0            # 30 km/h = 8.33 m/s，留余量
TARGET_EDGE_M = 3.0       # 判定区 8x8，留 1 m 余量
CRUISE_MIN = 1.5
HOLD_SPEED = 0.01
WHEELBASE_M = 2.6
MAX_STEER_RAD = 0.6


class Pt(NamedTuple):
    x: float
    y: float


class GridPlanner:
    """1 m 栅格 8 邻域 A*；建筑外扩后当障碍，最后做直线捷径平滑。"""

    def __init__(self, cell=1.0, inflate=1.8, margin=12.0):
        self.cell = cell
        self.inflate = inflate
        self.margin = margin

    def plan(self, start_xy, goal_xy, boxes):
        x0 = min(start_xy[0], goal_xy[0]) - self.margin
        y0 = min(start_xy[1], goal_xy[1]) - self.margin
        x1 = max(start_xy[0], goal_xy[0]) + self.margin
        y1 = max(start_xy[1], goal_xy[1]) + self.margin
        nx = max(2, int((x1 - x0) / self.cell) + 1)
        ny = max(2, int((y1 - y0) / self.cell) + 1)

        blocked = [[False] * ny for _ in range(nx)]
        for (cx, cy, ln, wd, ori) in boxes:
            th = math.radians(ori)
            hl = ln / 2.0 + self.inflate
            hw = wd / 2.0 + self.inflate
            rad = math.hypot(hl, hw)
            i0 = max(0, int((cx - rad - x0) / self.cell))
            i1 = min(nx - 1, int((cx + rad - x0) / self.cell) + 1)
            j0 = max(0, int((cy - rad - y0) / self.cell))
            j1 = min(ny - 1, int((cy + rad - y0) / self.cell) + 1)
            c, s = math.cos(th), math.sin(th)
            for i in range(i0, i1 + 1):
                wx = x0 + i * self.cell
                for j in range(j0, j1 + 1):
                    wy = y0 + j * self.cell
                    dx, dy = wx - cx, wy - cy
                    u = dx * c + dy * s
                    v = -dx * s + dy * c
                    if abs(u) <= hl and abs(v) <= hw:
                        blocked[i][j] = True

        def to_cell(p):
            return (int(round((p[0] - x0) / self.cell)),
                    int(round((p[1] - y0) / self.cell)))

        def to_world(ij):
            return (x0 + ij[0] * self.cell, y0 + ij[1] * self.cell)

        s_ij, g_ij = to_cell(start_xy), to_cell(goal_xy)
        s_ij = (min(max(s_ij[0], 0), nx - 1), min(max(s_ij[1], 0), ny - 1))
        g_ij = (min(max(g_ij[0], 0), nx - 1), min(max(g_ij[1], 0), ny - 1))
        blocked[s_ij[0]][s_ij[1]] = False
        blocked[g_ij[0]][g_ij[1]] = False

        def h(a):
            return math.hypot(a[0] - g_ij[0], a[1] - g_ij[1])

        openq = [(h(s_ij), 0.0, s_ij)]
        best = {s_ij: 0.0}
        parent = {}
        found = False
        while openq:
            _, g, cur = heapq.heappop(openq)
            if cur == g_ij:
                found = True
                break
            if g > best.get(cur, 1e18) + 1e-9:
                continue
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nb = (cur[0] + dx, cur[1] + dy)
                    if not (0 <= nb[0] < nx and 0 <= nb[1] < ny):
                        continue
                    if blocked[nb[0]][nb[1]]:
                        continue
                    if dx and dy and (blocked[cur[0] + dx][cur[1]] or blocked[cur[0]][cur[1] + dy]):
                        continue
                    ng = g + math.hypot(dx, dy)
                    if ng < best.get(nb, 1e18) - 1e-9:
                        best[nb] = ng
                        parent[nb] = cur
                        heapq.heappush(openq, (ng + h(nb), ng, nb))

        if not found:
            return [Pt(*start_xy), Pt(*goal_xy)]

        chain = [g_ij]
        while chain[-1] != s_ij:
            chain.append(parent[chain[-1]])
        chain.reverse()
        pts = [Pt(*to_world(ij)) for ij in chain]
        pts[0] = Pt(*start_xy)
        pts[-1] = Pt(*goal_xy)

        out = [pts[0]]
        for i in range(1, len(pts) - 1):
            ax, ay = pts[i][0] - out[-1][0], pts[i][1] - out[-1][1]
            bx, by = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
            if abs(ax * by - ay * bx) > 1e-6 or (ax * bx + ay * by) < 0:
                out.append(pts[i])
        out.append(pts[-1])

        def clear(a, b):
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(2, int(d / 0.5))
            for k in range(1, n):
                wx = a[0] + (b[0] - a[0]) * k / n
                wy = a[1] + (b[1] - a[1]) * k / n
                ci = int(round((wx - x0) / self.cell))
                cj = int(round((wy - y0) / self.cell))
                if not (0 <= ci < nx and 0 <= cj < ny) or blocked[ci][cj]:
                    return False
            return True

        simp = [out[0]]
        i = 0
        while i < len(out) - 1:
            j = len(out) - 1
            while j > i + 1 and not clear(out[i], out[j]):
                j -= 1
            simp.append(out[j])
            i = j
        return simp


class PathFollower:
    """按弧长选目标点 + 纯跟踪曲率。

    不用 control/tracking.PurePursuit：它按"点序号"选目标，在 3~5 个点的大间隔
    路径上会在直线段输出满舵（实测 steer=-1.000），车原地画圈。
    """

    def __init__(self, path, lookahead=7.0):
        self.path = [Pt(float(p.x), float(p.y)) for p in path]
        self.lookahead = lookahead
        self.seg = 0
        self.s = 0.0
        self.seg_off = 0.0
        self.uturn_dir = 0.0        # 目标在车后时的掉头方向（锁存，避免来回翻）

    def _seg_len(self, i):
        a, b = self.path[i], self.path[i + 1]
        return math.hypot(b.x - a.x, b.y - a.y)

    def update(self, x, y):
        """把车投影到路径上，更新已走弧长与所在线段；返回 (弧长, 横向偏差)。"""
        best = None
        s_acc = sum(self._seg_len(k) for k in range(self.seg))
        for i in range(self.seg, len(self.path) - 1):
            a, b = self.path[i], self.path[i + 1]
            ax, ay = b.x - a.x, b.y - a.y
            L2 = ax * ax + ay * ay
            if L2 < 1e-9:
                continue
            t = ((x - a.x) * ax + (y - a.y) * ay) / L2
            t = max(0.0, min(1.0, t))
            px, py = a.x + ax * t, a.y + ay * t
            d = math.hypot(x - px, y - py)
            if best is None or d < best[1]:
                best = (s_acc + t * math.sqrt(L2), d, i)
            s_acc += math.sqrt(L2)
        if best:
            self.s, _, self.seg = best
            # 段内偏移（不是绝对弧长）：target_point 里要用它做起点，
            # 之前把"绝对弧长"和"段内累加"混用，导致永远返回路径终点，
            # 车就会斜切过去撞楼。
            self.seg_off = self.s - sum(self._seg_len(k) for k in range(self.seg))
        return self.s, (best[1] if best else 0.0)

    def target_point(self):
        want = self.seg_off + self.lookahead
        acc = 0.0
        for i in range(self.seg, len(self.path) - 1):
            L = self._seg_len(i)
            if acc + L >= want:
                a, b = self.path[i], self.path[i + 1]
                f = (want - acc) / L if L > 1e-9 else 0.0
                return Pt(a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f)
            acc += L
        return self.path[-1]

    def remaining(self):
        total = sum(self._seg_len(i) for i in range(len(self.path) - 1))
        return max(0.0, total - self.s)

    def steering(self, x, y, yaw_deg):
        tgt = self.target_point()
        dx, dy = tgt.x - x, tgt.y - y
        dist = max(math.hypot(dx, dy), 1e-3)
        rad = math.radians(-yaw_deg)          # 场景 yaw 顺时针为正
        lx = dx * math.cos(rad) + dy * math.sin(rad)
        ly = -dx * math.sin(rad) + dy * math.cos(rad)
        if lx < 0.0:
            # 目标在车后：固定一个方向掉头，直到目标转到前方再交回纯跟踪。
            # 之前按 ly 符号实时取反，车头一转 ly 就变号，于是 steer 在 ±1 之间来回抽。
            if self.uturn_dir == 0.0:
                self.uturn_dir = 1.0 if ly > 0 else -1.0
            return self.uturn_dir
        self.uturn_dir = 0.0
        curvature = 2.0 * ly / (dist * dist)
        delta = math.atan(curvature * WHEELBASE_M)
        return max(-1.0, min(1.0, -delta / MAX_STEER_RAD))


class LongCtrl:
    """纵向控制（与连续场景 650 分那版同源）。"""

    def __init__(self):
        self.lim_thr = 0.0
        self.hold_brk = 0.0

    def drive(self, v, v_target):
        err = v_target - abs(v)
        if err > 0.10:
            cmd = max(0.0, min(0.30, 0.35 * err))
            if abs(v) < 1.5:
                cmd = min(cmd, 0.006)
            elif abs(v) < 4.0:
                cmd = min(cmd, 0.02)
            brake = 0.0
        elif err < -0.25:
            cmd = 0.0
            brake = max(0.04, min(0.07, -err * 0.04))
        else:
            cmd, brake = 0.0, 0.0
        self.lim_thr += max(-0.0015, min(0.0015, cmd - self.lim_thr))
        self.lim_thr = max(0.0, min(0.30, self.lim_thr))
        return self.lim_thr, brake

    def hold(self):
        self.hold_brk = min(1.0, self.hold_brk + 0.05)
        return 0.0, self.hold_brk


def fmt_building(b):
    return (f"{getattr(b, 'name', '?')}@({b.pos_x:.1f},{b.pos_y:.1f}) "
            f"{b.length:.0f}x{b.width:.0f} ori={b.ori_z:.0f}")


def fix_phrases(info, building_name=""):
    """用已知信息纠正 OCR 的错字，并裁掉行动段尾部的乱码。

    - 楼名：OCR 常把楼号单字认错（实测 B->G 会把车开到反方向），
      既然 buildings 里给了标准名，就把地点里的楼名换回标准值。
    - 行动段：牌子最后一行常带装饰符号，OCR 会吐出 "…30km/h簋000"，
      统一截到 "km/h" 为止。
    """
    out = dict(info)
    place = out.get("place", "")
    action = out.get("action", "")
    ocr_text = out.get("ocr_text", "")

    if building_name:
        import re
        m = re.search(r"[\u4e00-\u9fff]{0,6}[A-Za-z0-9]{1,2}栋", place)
        if m:
            place = place.replace(m.group(0), building_name)
        else:
            short = building_name.replace("锦汇华庭", "")
            if short not in place and "停车位" not in place and "车位" not in place:
                place = building_name + place
        ocr_text = ocr_text.replace(m.group(0), building_name) if m else ocr_text

    # 本场景固定词表的常见 OCR 错字（都是实测出现过的）
    fixes = {
        "冂凵": "门口", "冂口": "门口", "冂": "门",
        "谝汇华庭": "锦汇华庭", "牖汇华庭": "锦汇华庭",
        "杪": "秒", "．": ",", "限利": "限制", "眼制": "限制",
        "仝的": "空的", "低达": "抵达",
    }
    for a, b in fixes.items():
        place = place.replace(a, b)
        action = action.replace(a, b)
        ocr_text = ocr_text.replace(a, b)

    if "km/h" in action:
        action = action[: action.index("km/h") + len("km/h")]
    elif "km／h" in action:
        action = action[: action.index("km／h") + len("km／h")]

    out["place"] = place
    out["action"] = action
    out["ocr_text"] = ocr_text
    return out



def find_door_plate(frame, want_letters=""):
    """找楼门牌并读出上面的字（如 "B栋一单元"）。

    做法：高饱和黄色 + 横长条 + 面积窗口先把候选挑出来，
    再裁出来放大 3 倍 OCR；只有 OCR 出 "栋"（且含目标字母，若给了）才算数。
    上一版在"未验证帧"上直接采信，导致抓到别的栋的门牌。
    """
    if frame is None or getattr(frame, "size", 0) == 0:
        return None
    h, w = frame.shape[:2]
    roi = frame[: int(h * 0.60), :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (18, 190, 150), (40, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, 8)

    cands = []
    for i in range(1, n):
        x0, y0, ww, hh, area = stats[i]
        frac = area / float(w * h)
        if frac < 0.00020 or frac > 0.030:
            continue
        if ww < 1.5 * hh:
            continue
        cands.append((area, x0, y0, ww, hh, cents[i][0] / w, frac))
    cands.sort(reverse=True)

    for area, x0, y0, ww, hh, cx, frac in cands[:2]:
        pad = 10
        crop = roi[max(0, y0 - pad): y0 + hh + pad, max(0, x0 - pad): x0 + ww + pad]
        if crop.size == 0:
            continue
        big = cv2.resize(crop, None, fx=3.0, fy=3.0, interpolation=cv2.INTER_CUBIC)
        text = read_text(big, scale=1.0)
        flat = "".join(text.split())
        if "栋" not in flat:
            continue
        if want_letters and not any(ch in flat.upper() for ch in want_letters.upper()):
            continue
        return (cx, frac, flat)
    return None


def build_scan_path(b, x, y, args):
    """贴楼面巡航的直线：离墙 scan_offset 米，从楼一端扫到另一端。"""
    th = math.radians(b.ori_z)
    lx, ly = math.cos(th), math.sin(th)
    wx, wy = -math.sin(th), math.cos(th)
    side = 1.0 if (wx * (x - b.pos_x) + wy * (y - b.pos_y)) >= 0 else -1.0
    off = b.width / 2.0 + args.scan_offset
    y_line = b.pos_y + wy * side * off
    half = b.length / 2.0 + 3.0
    xa = b.pos_x - lx * half
    xb = b.pos_x + lx * half
    if abs(x - xa) > abs(x - xb):      # 从离自车近的那端开始
        xa, xb = xb, xa
    return [Pt(x, y), Pt(xa, y_line), Pt(xb, y_line)], side


def door_side_point(b, along, side, args):
    """门在楼体坐标里的位置 -> 门前停车点（离墙 approach_offset 米）。"""
    th = math.radians(b.ori_z)
    lx, ly = math.cos(th), math.sin(th)
    wx, wy = -math.sin(th), math.cos(th)
    off = b.width / 2.0 + args.approach_offset
    return (b.pos_x + lx * along + wx * side * off,
            b.pos_y + ly * along + wy * side * off)



def resolve_target(place, msg, vla, x, y, yaw, args):
    """地点片段 -> (目标点, 模式)。模式为 building / slot / manual。"""
    if args.target_x is not None and args.target_y is not None:
        return (args.target_x, args.target_y), "manual"

    p = normalize(place)

    # 绝对停车位置优先：只要场景给了 PARKING_SLOT，一律去它，不看指令文本怎么写。
    # （用户指示：不要再管"楼门口 8x8"，就去那个绝对停车位置。）
    if args.target_x is None:
        slots = [o for o in msg.obstacles
                 if int(getattr(getattr(o, "type", 0), "value", -1)) == 30]
        if slots:
            best = min(slots, key=lambda o: math.hypot(o.pos_x - x, o.pos_y - y))
            print(f"   使用场景给的绝对停车位 ({best.pos_x:.1f},{best.pos_y:.1f}) "
                  f"{best.length:.1f}x{best.width:.1f}", flush=True)
            args.slot = best
            th_s = math.radians(best.ori_z)
            ax, ay = math.cos(th_s), math.sin(th_s)
            # 从车位中心指向自车的方向，投影到轴线上，决定从哪头进
            proj = (x - best.pos_x) * ax + (y - best.pos_y) * ay
            sig = 1.0 if proj >= 0 else -1.0
            args.slot_approach = (best.pos_x + ax * sig * 14.0,
                                  best.pos_y + ay * sig * 14.0)
            args.slot_yaw = best.ori_z if sig > 0 else (best.ori_z + 180.0) % 360.0
            print(f"   进库方向：从 ({args.slot_approach[0]:.1f},{args.slot_approach[1]:.1f}) "
                  f"沿轴线进，期望航向 {args.slot_yaw:.0f}°", flush=True)
            return (best.pos_x, best.pos_y), "slot"

    ranked = match_place(p, list(vla.buildings))
    if not ranked:
        return None, "none"
    for s, cand in ranked[:3]:
        print(f"   匹配 {fmt_building(cand)}  score={s:.3f}", flush=True)
    score, b = ranked[0]

    # 门开在长边上：沿短边方向朝自车那侧外移
    dx, dy = x - b.pos_x, y - b.pos_y
    th = math.radians(b.ori_z)
    lx, ly = math.cos(th), math.sin(th)
    wx, wy = -math.sin(th), math.cos(th)
    s = wx * dx + wy * dy
    if abs(s) < 1e-3:
        s = lx * dx + ly * dy
        sx, sy, half = lx, ly, b.length / 2.0
    else:
        sx, sy, half = wx, wy, b.width / 2.0
    sgn = 1.0 if s >= 0 else -1.0
    reach = half + args.approach_offset
    tx = b.pos_x + sx * sgn * reach
    ty = b.pos_y + sy * sgn * reach
    # 楼门口是"沿楼面平行停"：期望航向取楼的长边方向，选离当前车头近的那一侧
    wall_a = b.ori_z % 360.0
    wall_b = (b.ori_z + 180.0) % 360.0
    d_a = abs(((wall_a - yaw) + 180.0) % 360.0 - 180.0)
    d_b = abs(((wall_b - yaw) + 180.0) % 360.0 - 180.0)
    desired = wall_a if d_a <= d_b else wall_b
    return (tx, ty), f"building:{desired:.1f}|{b.name}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", action="store_true", help="只读：打印 VLA 扩展/相机/障碍物")
    ap.add_argument("--explore", type=float, default=0.0, help="配合 --probe：缓慢直行 N 秒找牌")
    ap.add_argument("--text", default="", help="跳过 OCR，直接用这句指令")
    ap.add_argument("--no-drive", action="store_true", help="不控车，只识别与提交")
    ap.add_argument("--target-x", type=float, default=None)
    ap.add_argument("--target-y", type=float, default=None)
    ap.add_argument("--approach-offset", type=float, default=6.0)
    ap.add_argument("--vmax", type=float, default=V_MAX_MS)
    ap.add_argument("--lookahead", type=float, default=7.0)
    ap.add_argument("--ocr-every", type=int, default=5)
    ap.add_argument("--ocr-window", type=float, default=2.0, help="开局先连续 OCR 多少秒再决策")
    ap.add_argument("--scan", type=int, default=1, help="1=沿楼面巡航找楼门（门坐标不在 JSON 里）")
    ap.add_argument("--scan-offset", type=float, default=9.0, help="巡航时离墙多远")
    ap.add_argument("--scan-speed", type=float, default=4.0)
    ap.add_argument("--save-frame", default=str(REPO / "tmp" / "vla_frame.png"))
    ap.add_argument("--max-seconds", type=float, default=118.0, help="硬上限；场景自然结束时会更早退出")
    ap.add_argument("--hold-after", type=float, default=12.0, help="到位后继续驻车多久再退出")
    args = ap.parse_args()

    print("OCR 后端:", ocr_backends(), flush=True)
    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"已连接。子场景 {len(static.sub_scenes)} 个，route {len(static.route)} 点", flush=True)

    vla = static.vla_extension
    if vla is None:
        print("!! 不是 VLA 场景（vla_extension 为 None）", flush=True)
        return 2
    print(f"VLA 扩展：buildings={len(vla.buildings)} regions={len(vla.regions)}", flush=True)
    for b in vla.buildings:
        print("   建筑 " + fmt_building(b), flush=True)
    for r in vla.regions:
        print(f"   区域 {r.id} type={r.type} ({r.pos_x:.1f},{r.pos_y:.1f}) "
              f"{r.length:.0f}x{r.width:.0f} ori={r.ori_z:.0f}", flush=True)

    boxes = [(b.pos_x, b.pos_y, b.length, b.width, b.ori_z) for b in vla.buildings]

    if args.probe:
        t_probe = time.time()
        saved = 0
        for msg, frames in api.main_loop():
            now_p = time.time() - t_probe
            pose = msg.pose_gnss
            print(f"[{now_p:5.1f}s] ({pose.pos_x:.1f},{pose.pos_y:.1f}) yaw={pose.ori_z:.1f}",
                  flush=True)
            for cam, fr in zip(msg.sensor.ego_rgb_cams, frames):
                txt = read_text(fr.frame)
                if txt:
                    saved += 1
                    out = REPO / "tmp" / f"ocr_hit_{saved:02d}_cam{cam.id}.png"
                    out.parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(out), fr.frame)
                    print(f"    cam{cam.id}: {txt!r} -> {out.name}", flush=True)
            if frames:
                cv2.imwrite(args.save_frame, frames[0].frame)
            if args.explore > 0 and now_p < args.explore:
                api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=0.02,
                                                       brake=0.0, steering=0.0))
                continue
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
            print(f"探针结束，命中文字 {saved} 次", flush=True)
            break
        return 0

    t0 = time.time()
    ctrl = LongCtrl()
    planner = GridPlanner()
    follower = None
    target = None
    mode = "ocr"
    best_text = ""
    ocr_votes = []
    submitted = ""
    last_ocr = -10 ** 9
    frame_idx = 0
    log_t = -1.0
    shot_t = -99.0
    arrive_t = None
    hold_t0 = -1.0
    desired_yaw = None
    bld = None
    scan_side = 0.0
    door_hits = []
    prev_v = None
    thr_cap = 0.35
    hold_t0 = -1.0
    prev_t = 0.0
    door_found = False
    is_slot_target = False
    slot_obj = [None]
    last_plate_ocr = -99.0
    want_letter = ""
    last_scan_ocr = -99.0
    dump_t = -99.0

    for msg, frames in api.main_loop():
        pose = msg.pose_gnss
        x, y, yaw = pose.pos_x, pose.pos_y, pose.ori_z
        v = abs(msg.main_vehicle.speed)
        now = time.time() - t0
        frame_idx += 1

        if mode == "ocr":
            if args.text:
                best_text = args.text
            elif frames and (frame_idx - last_ocr) >= args.ocr_every and now < args.ocr_window:
                last_ocr = frame_idx
                cand = read_text(frames[0].frame)
                if cand:
                    ocr_votes.append(cand)
                    print(f"[{now:5.1f}s] OCR: {cand!r}", flush=True)
                    # 多帧投票：选出"与建筑名匹配最好"的那一帧，单帧 OCR 容易把
                    # 楼号认错（B->G 实测发生过，会把车开到反方向）。
                    best_s, best_c = -1.0, best_text
                    for c in ocr_votes:
                        ci = split_instruction(c)
                        rk = match_place(ci["place"], list(vla.buildings))
                        s = rk[0][0] if rk else 0.0
                        if s > best_s:
                            best_s, best_c = s, c
                    best_text = best_c
                if now < args.ocr_window:
                    api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                    continue
            if best_text:
                info = split_instruction(best_text)
                print(f"拆分: 时间={info['time']!r} 地点={info['place']!r} "
                      f"行动={info['action']!r}", flush=True)
                target, tmode = resolve_target(info["place"], msg, vla, x, y, yaw, args)
                if target is None:
                    print("!! 目标解析失败", flush=True)
                    return 3
                std_name = tmode.split("|", 1)[1] if "|" in tmode else ""
                if std_name:
                    import re as _re
                    _m = _re.search(r"([A-Za-z])", std_name)
                    want_letter = _m.group(1) if _m else ""
                if std_name:
                    info = fix_phrases(info, std_name)
                    best_text = info["ocr_text"]
                    print(f"纠正后: 地点={info['place']!r} 行动={info['action']!r}", flush=True)
                print(f"目标 {target[0]:.1f},{target[1]:.1f} 模式={tmode} "
                      f"直线距离={math.hypot(target[0]-x, target[1]-y):.1f} m", flush=True)
                is_slot_target = (tmode == "slot")
                want_scan = bool(args.scan) and tmode.startswith("building") and std_name
                if want_scan:
                    bld = next(bb for bb in vla.buildings if bb.name == std_name)
                    scan_path, scan_side = build_scan_path(bld, x, y, args)
                    # 巡航起点也要用 A* 绕开楼：直线穿楼会直接卡死（实测撞进 D 栋）
                    pre = planner.plan((x, y), (scan_path[1].x, scan_path[1].y), boxes)
                    full = pre + scan_path[2:]
                    print(f"沿楼面巡航找门（离墙 {args.scan_offset:.0f} m）："
                          f"{[(round(q.x,1), round(q.y,1)) for q in full]}", flush=True)
                    follower = PathFollower(full, lookahead=args.lookahead)
                    mode = "scan"
                elif tmode == "slot" and getattr(args, "slot_approach", None) is not None:
                    pre = planner.plan((x, y), args.slot_approach, boxes)
                    path = pre + [Pt(target[0], target[1])]
                    print(f"进库路径 {len(path)} 点：… → 对位点 → 车位中心", flush=True)
                    follower = PathFollower(path, lookahead=args.lookahead)
                    mode = "drive"
                else:
                    path = planner.plan((x, y), target, boxes)
                    plen = sum(math.hypot(path[k+1].x-path[k].x, path[k+1].y-path[k].y)
                               for k in range(len(path)-1))
                    print(f"A* {len(path)} 点 {plen:.1f} m: "
                          f"{[(round(q.x,1), round(q.y,1)) for q in path]}", flush=True)
                    follower = PathFollower(path, lookahead=args.lookahead)
                    mode = "drive"

        payload = None
        info = split_instruction(best_text) if best_text else None
        if info and info["ocr_text"] and info["ocr_text"] != submitted:
            payload = VLAExtensionOutput(text_info=VLATextOutput(
                ocr_text=info["ocr_text"],
                time_phrase=info["time"],
                location_phrase=info["place"],
                action_phrase=info["action"],
            ))

        if args.no_drive:
            vc = VehicleControl(gear=GearMode.PARKING, brake=1.0, throttle=0.0, steering=0.0)
        elif mode == "scan" and follower is not None:
            follower.update(x, y)
            steer = follower.steering(x, y, yaw)
            if frames:
                # 黄牌检测：车正对门时黄块会落在画面中线
                want = ""
                if bld is not None:
                    import re as _re
                    m = _re.search(r"([A-Za-z])\s*栋", bld.name)
                    want = m.group(1) if m else ""
                sign = find_door_plate(frames[0].frame, want_letters=want)
                if sign is not None:
                    cx, frac, stext = sign
                    door_hits.append(x)
                    print(f"[{now:5.1f}s] 门牌 {stext!r} cx={cx:.2f} @x={x:.1f}"
                          f"（累计 {len(door_hits)}）", flush=True)
                elif now - last_scan_ocr >= 1.2:
                    # 没抓到黄牌就退回 OCR（慢，但兜底）
                    last_scan_ocr = now
                    txt = read_text(frames[0].frame, scale=1.0)
                    if txt and ("单元" in txt or (bld is not None and bld.name[-2:] in txt)):
                        door_hits.append(x)
                        print(f"[{now:5.1f}s] OCR 见到门牌 {txt!r} @x={x:.1f}", flush=True)
            if follower.remaining() <= 1.0 or now >= args.ocr_window + 45.0:
                if door_hits:
                    xs = sorted(door_hits)
                    door_x = xs[len(xs) // 2]
                    along = door_x - bld.pos_x
                else:
                    along = 0.0
                    print("!! 没认到门牌，退回楼面中点", flush=True)
                target = door_side_point(bld, along, scan_side, args)
                print(f"门定位 x≈{bld.pos_x + along:.1f} → 停车点 "
                      f"({target[0]:.1f},{target[1]:.1f})", flush=True)
                path = planner.plan((x, y), target, boxes)
                follower = PathFollower(path, lookahead=args.lookahead)
                mode = "drive"
                vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0,
                                    brake=max(0.04, min(0.07, v * 0.08)), steering=steer)
            else:
                v_t = min(args.scan_speed, max(CRUISE_MIN, 0.9 * follower.remaining()))
                thr, brk = ctrl.drive(v, v_t)
                api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=thr,
                                                       brake=brk, steering=steer))
                continue
        elif mode == "drive" and target is not None and follower is not None:
            # 机会式读门牌：车驶近楼面时门牌会进画面，读到就说明车正在门口，
            # 直接把目标改成"门前 3 m"，比"先扫一整面墙"可靠得多。
            if want_letter and not door_found and frames and now - last_plate_ocr >= 0.35:
                last_plate_ocr = now
                plate = find_door_plate(frames[0].frame, want_letters=want_letter)
                if plate is not None:
                    cx, frac, stext = plate
                    door_found = True
                    door_hits = [x]
                    print(f"[{now:5.1f}s] 读到门牌 {stext!r} cx={cx:.2f} @x={x:.1f}"
                          f" → 就地定为门口", flush=True)
                    bld = bld
                    if bld is not None:
                        th = math.radians(bld.ori_z)
                        wxx, wyy = -math.sin(th), math.cos(th)
                        s = wxx * (x - bld.pos_x) + wyy * (y - bld.pos_y)
                        side = 1.0 if s >= 0 else -1.0
                        target = door_side_point(bld, x - bld.pos_x, side, args)
                        path = planner.plan((x, y), target, boxes)
                        follower = PathFollower(path, lookahead=args.lookahead)
                        print(f"    新停车点 ({target[0]:.1f},{target[1]:.1f})", flush=True)
            follower.update(x, y)
            dist = math.hypot(target[0] - x, target[1] - y)
            steer = follower.steering(x, y, yaw)

            # 车位目标：以"车心进入车位矩形"为停止判据（不是离中心多近——
            # 之前 d=1.08 触发刹车，但车速 3.5 m/s 滑过去，最后停在框外 2.94 m）。
            inside_slot = False
            sl = getattr(args, "slot", None)
            pos_err = {}
            if is_slot_target and sl is not None:
                th_s = math.radians(sl.ori_z)
                dxs, dys = x - sl.pos_x, y - sl.pos_y
                du = dxs * math.cos(th_s) + dys * math.sin(th_s)
                dv = -dxs * math.sin(th_s) + dys * math.cos(th_s)
                yaw_err = abs(((getattr(args, "slot_yaw", sl.ori_z) - yaw) + 180.0) % 360.0 - 180.0)
                pos_err = {"du": du, "dv": dv, "yaw": yaw_err}
                # 几何达标：沿轴 ±0.35 m、横向 ±0.25 m、航向 ±3°
                inside_slot = (abs(du) <= 0.35 and abs(dv) <= 0.25 and yaw_err <= 3.0)
            stop_dist = 0.8 if is_slot_target else TARGET_EDGE_M
            if is_slot_target and not inside_slot and pos_err:
                # 没到位就继续走（不再"接近就停"），交给下面的几何微调
                pass
            if inside_slot or (not is_slot_target and dist <= stop_dist):
                sl0 = getattr(args, "slot", None)
                if is_slot_target and sl0 is not None and not getattr(args, "recentered", False):
                    th0 = math.radians(sl0.ori_z)
                    du0 = (x - sl0.pos_x) * math.cos(th0) + (y - sl0.pos_y) * math.sin(th0)
                    if abs(du0) > 0.40:
                        args.recentered = True
                        args.recenter_t = now
                        args.recenter_gear = GearMode.DRIVE
                        print(f"   进框但偏轴 {du0:+.2f} m，倒车对中", flush=True)
                        mode = "recenter"
                        vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0,
                                            brake=0.08, steering=0.0)
                        api.set_vehicle_control(vc, vla_extension=payload)
                        continue
                mode = "hold"
                ctrl.hold_brk = 0.0
                vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0,
                                    brake=max(0.04, min(0.07, v * 0.08)), steering=steer)
            else:
                v_t = min(args.vmax, max(CRUISE_MIN, 0.9 * follower.remaining()))
                if is_slot_target and dist < 18.0:
                    # 进库压速，并且**超速时主动刹车**（只松油门靠滑行来不及：
                    # 上一局 4.7 m/s 进框，冲过头 3.32 m）
                    v_t = min(v_t, max(0.8, 0.45 * dist))
                    if v > v_t + 0.6:
                        vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0,
                                            brake=min(0.07, 0.05 * (v - v_t)), steering=steer)
                        api.set_vehicle_control(vc)
                        continue
                # 弯道降速：方向盘越大目标速度越低，避免满舵高速时 |a| 超 5
                v_t = min(v_t, args.vmax * max(0.35, 1.0 - 0.9 * abs(steer)))
                thr, brk = ctrl.drive(v, v_t)
                # 油门按实测加速度闭环限幅：一脚 0.02 瞬时 +8 m/s²，thr_cap 自适应
                acc_n = ((v - prev_v) / max(now - prev_t, 1e-3)) if prev_v is not None else 0.0
                if acc_n > 4.2:
                    thr_cap = max(0.006, thr_cap * 0.55)
                elif acc_n < 3.0:
                    thr_cap = min(0.35, thr_cap * 1.06)
                thr = min(thr, thr_cap)
                # 进库最后 6 m：油门强制 0，纯滑行+轻刹。
                # 到点时油门必须已经闲置一段时间，否则"收油瞬间"的扭矩突变
                # 会被平台 30 Hz 采样判成加速度超限（1 Hz 遥测看不到）。
                if is_slot_target and dist < 6.0:
                    thr = 0.0
                vc = VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=steer)
        elif mode == "recenter" and getattr(args, "slot", None) is not None:
            # 沿车位长轴把车挪到框中心，全程遵守三条铁律：
            #   ① 换挡只在 |v| <= 0.02 时做；② 刹车 <= 0.09；③ 油门从 0.006 起。
            sl = args.slot
            th_s = math.radians(sl.ori_z)
            dxs, dys = x - sl.pos_x, y - sl.pos_y
            du = dxs * math.cos(th_s) + dys * math.sin(th_s)
            want_gear = GearMode.REVERSE if du > 0 else GearMode.DRIVE
            done = abs(du) <= 0.12 or now - args.recenter_t > 15.0
            if abs(v) > 0.02:
                # 还在动：只轻刹，绝不换挡（换挡会造成单帧速度突变 = 加速度告警）
                vc = VehicleControl(gear=args.recenter_gear, throttle=0.0,
                                    brake=0.05, steering=0.0)
            elif done:
                mode = "hold"
                ctrl.hold_brk = 0.0
                print(f"   对中完成 du={du:+.2f} m（已停稳）", flush=True)
                vc = VehicleControl(gear=GearMode.PARKING, throttle=0.0, brake=1.0, steering=0.0)
            else:
                args.recenter_gear = want_gear      # 停稳后才换挡
                thr = 0.006 if abs(v) < 0.25 else 0.007
                vc = VehicleControl(gear=want_gear, throttle=thr, brake=0.0, steering=0.0)
        elif mode == "hold":
            slh = getattr(args, "slot", None)
            if slh is not None and abs(v) <= 0.05 and not getattr(args, "recentered", False):
                thh = math.radians(slh.ori_z)
                duh = (x - slh.pos_x) * math.cos(thh) + (y - slh.pos_y) * math.sin(thh)
                if abs(duh) > 0.40:
                    args.recentered = True
                    args.recenter_t = now
                    args.recenter_gear = GearMode.DRIVE
                    print(f"   停稳位置偏轴 {duh:+.2f} m，开始沿轴对中", flush=True)
                    mode = "recenter"
                    vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=0.08, steering=0.0)
                    api.set_vehicle_control(vc, vla_extension=payload)
                    continue
            # 三段式停车：D 挡刹停 → 空挡两帧 → 再挂 P。
            # 收尾不换挡！平台只看位置+速度；D→N→P 的变速箱冲击会被判成加速度超限。
            # 全程停在 D 挡：油门 0，刹车按 <0.07 递增把车压停，再保持住即可。
            if hold_t0 < 0:
                hold_t0 = now
            coasting = (now - hold_t0) < 0.5
            vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0,
                                brake=0.0 if coasting else
                                      (0.07 if abs(v) > 0.06 else 0.0), steering=0.0)
        else:
            vc = VehicleControl(gear=GearMode.PARKING, brake=1.0, throttle=0.0, steering=0.0)

        api.set_vehicle_control(vc, vla_extension=payload)
        if payload is not None:
            submitted = info["ocr_text"]
            print(f"[{now:5.1f}s] 已提交文本（{len(submitted)} 字）: {submitted}", flush=True)

        # 逐帧加速度自检（平台按 30 Hz 算 Δv/Δt，这里同口径）
        if prev_v is not None and now - prev_t > 1e-3:
            acc = (v - prev_v) / (now - prev_t)
            if abs(acc) > 5.0:
                print(f"[{now:5.1f}s] !! 加速度 {acc:+.1f} m/s2  mode={mode} "
                      f"v={v:.2f} thr={vc.throttle:.3f} brk={vc.brake:.2f}", flush=True)
        prev_v = v
        prev_t = now

        if frames and now - shot_t >= 3.0:
            shot_t = now
            cv2.imwrite(str(REPO / "tmp" / f"vla_drive_{int(now):03d}.png"), frames[0].frame)
        if now - dump_t >= 5.0:
            dump_t = now
            from collections import Counter
            cnt = Counter(str(o.type) for o in msg.obstacles)
            print(f"   障碍物 {dict(cnt)}", flush=True)
            for o in msg.obstacles[:6]:
                print(f"      id={o.id} {o.type} ({o.pos_x:.1f},{o.pos_y:.1f}) "
                      f"{o.length:.1f}x{o.width:.1f} extra={o.extra_info!r}", flush=True)
        if now - log_t >= 1.0:
            log_t = now
            d_txt = " -" if target is None else f" d={math.hypot(target[0]-x, target[1]-y):6.2f}"
            print(f"[{now:5.1f}s] {mode:<5} ({x:7.1f},{y:7.1f}) yaw={yaw:6.1f} "
                  f"v={v:5.2f}({v*3.6:4.1f}) thr={vc.throttle:.3f} brk={vc.brake:.2f}"
                  f" steer={vc.steering:+.2f}{d_txt}", flush=True)

        if mode == "hold" and ctrl.hold_brk >= 0.99 and arrive_t is None:
            arrive_t = now
            print(f"到达：用时 {now:.1f}s，位置 ({x:.2f},{y:.2f})，目标 {target}", flush=True)
            sl = getattr(args, "slot", None)
            if sl is not None:
                th_s = math.radians(sl.ori_z)
                dxs, dys = x - sl.pos_x, y - sl.pos_y
                du = dxs * math.cos(th_s) + dys * math.sin(th_s)
                dv = -dxs * math.sin(th_s) + dys * math.cos(th_s)
                e = ((getattr(args, "slot_yaw", sl.ori_z) - yaw) + 180.0) % 360.0 - 180.0
                print(f"  车位几何：沿轴 du={du:+.2f} m  横向 dv={dv:+.2f} m  "
                      f"航向偏差 {e:+.1f}°（门限 ±0.35 / ±0.25 / ±3°）", flush=True)
        # 到位后**不主动退出**：平台要等场景自然结束（100 s 时限或判定完成）才结算，
        # 提前断开会导致整局不被记录（实测十几局只记录了 4 局，全 0 分）。
        if arrive_t is not None and args.hold_after > 0 and now - arrive_t >= args.hold_after:
            print(f"驻车 {args.hold_after:.0f}s，改为等场景自然结束", flush=True)
            args.hold_after = -1.0
        if now > args.max_seconds:
            print(f"超时 {now:.1f}s，停在 ({x:.2f},{y:.2f})，目标 {target}", flush=True)
            break

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except OSError:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

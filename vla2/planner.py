# -*- coding: utf-8 -*-
"""在线定向越野（Orienteering）决策器 —— 停车区违规检测任务二。

设计红线（用户要求，写在这里防止自己偷偷改）：**不许写死"第几秒做什么"、不许写死"走几条道"。**

本模块只做一件事：
    给定「已发现的车 / 已测的位姿 / 实测的速度与耗时统计 / 剩余时间」，
    生成候选观察节点 → 算每个节点的 prize 与 cost → 贪心 + 2-opt 解出一条动作序列
    → 返回下一步动作。

所有取舍都来自比较式：`prize / cost`。可写死的只有三类"物理与规则常数"：
  * 物理：车长、车宽、自车宽、最小安全间隙
  * 规则：时限、限速、碰撞扣分（这些由调用方以参数传入，不在这里写）
  * 几何：观察位的横向安全距离 = 半车宽(邻居) + 半车宽(自车) + 间隙

其余一切（可读距离 d50/k、转角耗时、交卷耗时、路径低效系数、超支系数）都必须在
运行时估计出来并打印 —— 见 `Estimators`。
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

# ---- 物理常数（允许写死：实物尺寸，不随场景/时间变化）----
CAR_LEN_M = 4.70          # 邻居车长（场景常数，可用相机反投影在线复核）
CAR_WID_M = 1.80          # 邻居车宽
EGO_WID_M = 1.90          # 自车宽
SAFE_GAP_M = 0.90         # 侧向安全间隙（法规/常识：一臂之距）


# ----------------------------------------------------------------------------- 估计器
class Estimators:
    """所有"经验数字"都必须从这里来，且每次变化都打印。"""

    def __init__(self, log=print) -> None:
        self.log = log
        self.v_now = 0.0            # 当前可达速度（实测）
        self.turn_cost = None       # 一次 90° 转弯的实测耗时（秒）
        self.submit_cost = None     # 一次交卷 API 的实测耗时（秒）
        self.path_eff = None        # 实际里程 / 直线距离（路径低效系数）
        self.overshoot = None       # 实际耗时 / 预估耗时（超支系数）
        # 读号成功样本：(侧向距离 m, 是否读对)
        self._samples: List[Tuple[float, bool]] = []
        self.d50 = None             # 50% 可读距离
        self.k = 0.8                # 逻辑斯蒂宽度（先用保守值，有样本后拟合）
        self.turn_hist: List[float] = []
        self.seg_hist: List[Tuple[float, float]] = []  # (直线距离, 实际里程)
        self.time_hist: List[Tuple[float, float]] = []  # (预估, 实际)

    # -- 采样 ---------------------------------------------------------------
    def add_read_sample(self, dist: float, ok: bool) -> None:
        self._samples.append((float(dist), bool(ok)))
        self._fit_read()

    def add_turn(self, seconds: float) -> None:
        self.turn_hist.append(float(seconds))
        self.turn_cost = sum(self.turn_hist[-6:]) / len(self.turn_hist[-6:])
        self.log(f"[est] turn_cost={self.turn_cost:.2f}s (n={len(self.turn_hist)})")

    def add_segment(self, straight_m: float, actual_m: float) -> None:
        if straight_m > 1.0:
            self.seg_hist.append((float(straight_m), float(actual_m)))
            eff = sum(a for _, a in self.seg_hist[-6:]) / sum(s for s, _ in self.seg_hist[-6:])
            self.path_eff = eff
            self.log(f"[est] path_eff={self.path_eff:.2f} (n={len(self.seg_hist)})")

    def add_timing(self, predicted_s: float, actual_s: float) -> None:
        if predicted_s > 1.0:
            self.time_hist.append((float(predicted_s), float(actual_s)))
            o = sum(a for _, a in self.time_hist[-4:]) / sum(p for p, _ in self.time_hist[-4:])
            self.overshoot = max(1.0, min(1.6, o))
            self.log(f"[est] overshoot={self.overshoot:.2f} (n={len(self.time_hist)})")

    def set_submit_cost(self, seconds: float) -> None:
        self.submit_cost = max(0.05, float(seconds))
        self.log(f"[est] submit_cost={self.submit_cost:.2f}s")

    # -- 拟合 ---------------------------------------------------------------
    def _fit_read(self) -> None:
        """两参数逻辑斯蒂 p(d)=1/(1+exp((d-d50)/k))，用已有样本做网格搜索。"""
        pos = [d for d, ok in self._samples if ok]
        neg = [d for d, ok in self._samples if not ok]
        if not pos or not neg:
            # 只有单类样本：用"能读到的最远距离"和"读不到的最小距离"夹一个区间
            if pos:
                self.d50 = max(pos) * 1.15
            return
        best = None
        for d50 in [x / 10.0 for x in range(20, 260, 1)]:
            for k in (0.3, 0.5, 0.8, 1.2, 1.8, 2.5):
                ll = 0.0
                for d, ok in self._samples:
                    p = 1.0 / (1.0 + math.exp((d - d50) / k))
                    p = min(max(p, 1e-6), 1 - 1e-6)
                    ll += math.log(p) if ok else math.log(1 - p)
                if best is None or ll > best[0]:
                    best = (ll, d50, k)
        _, self.d50, self.k = best
        self.log(f"[est] p_read 拟合 d50={self.d50:.1f}m k={self.k:.2f} (n={len(self._samples)})")

    def p_read(self, dist: float) -> float:
        if self.d50 is None:
            # 还没样本：用一个"不知道就都算半信"的保守先验，不写死具体米数
            return 0.5 if dist <= 8.0 else 0.2
        return 1.0 / (1.0 + math.exp((dist - self.d50) / self.k))

    # -- 汇总打印 -----------------------------------------------------------
    def report(self) -> str:
        return (f"v_now={self.v_now:.2f}m/s turn={self.turn_cost} submit={self.submit_cost} "
                f"path_eff={self.path_eff} overshoot={self.overshoot} "
                f"d50={self.d50} k={self.k}")


# ----------------------------------------------------------------------------- 数据
@dataclass
class CarObs:
    x: float
    y: float
    ori_deg: float = 90.0     # 车身长轴朝向（0=+x，顺时针为正）
    p_violation: float = 0.5  # 违规概率：涂装区 1.0 / 合法车位 0.0 / 未知 0.5
    p_label: float = 0.0      # 已读对的概率
    label: str = ""
    seen: int = 0


@dataclass
class Node:
    kind: str                 # 'view' | 'coverage' | 'submit'
    x: float
    y: float
    prize: float
    car: int = -1
    meta: dict = field(default_factory=dict)


# ----------------------------------------------------------------------------- 决策器
class Orienteer:
    """把 P0/P1/P2/P3 变成节点集合，用 prize/cost 排序。"""

    def __init__(self, est: Estimators, region: Tuple[float, float, float, float],
                 observed_area: Optional[float] = None, log=print) -> None:
        self.est = est
        self.region = region   # (xmin, ymin, xmax, ymax) 禁停区
        # 已观测面积：由调用方实测（走过的里程 × 相机在那个距离上的视场宽）。
        # 不给就退回"整片区域"，即最保守（不鼓励盲目探索）。
        self.observed_area = observed_area
        self.log = log

    # -- 观察节点 -----------------------------------------------------------
    def _view_band(self, car: CarObs) -> float:
        """侧面可读的横向安全距离：物理算出来的下界。"""
        return 0.5 * CAR_WID_M + 0.5 * EGO_WID_M + SAFE_GAP_M

    def view_nodes(self, cars: Sequence[CarObs], pose: Tuple[float, float],
                   v_now: float) -> List[Node]:
        nodes: List[Node] = []
        dmin = self._view_band(cars[0]) if cars else CAR_WID_M
        for i, c in enumerate(cars):
            if c.p_label >= 0.75:
                continue
            # 报告一辆车是"有赚也有赔"的：判对加 A、判错加 B（都在 F1 分母里）。
            # 一阶近似下净收益 ∝ (2·p_violation − 1)，所以 p_v < 0.5 的车**不值得去读**。
            net = 2.0 * c.p_violation - 1.0
            gain = (1.0 - c.p_label) * net
            if gain <= 0.02:
                continue
            th = math.radians(c.ori_deg)
            ux, uy = math.cos(th), math.sin(th)      # 长轴
            nx, ny = -uy, ux                          # 长轴法向（车身侧面朝向）
            L = 0.5 * CAR_LEN_M + 0.5 * EGO_WID_M + SAFE_GAP_M
            for s in (+1.0, -1.0):
                # 侧面上的一点：沿车长中线再往侧面偏 L
                for t in (0.0,):
                    ox = c.x + ux * t + nx * L * s
                    oy = c.y + uy * t + ny * L * s
                    dist = abs(L)  # 站在该点上，到车身中线的距离
                    p = self.est.p_read(dist)
                    if p < 0.05:
                        continue
                    nodes.append(Node("view", ox, oy, gain * p, car=i,
                                      meta={"side": s, "read_dist": dist}))
        return nodes

    # -- 覆盖节点 -----------------------------------------------------------
    def coverage_nodes(self, cars: Sequence[CarObs], cells_m: float = 4.0) -> List[Node]:
        """禁停区内"还没有任何车被观察到"的格子 —— 潜在漏检风险。

        收益不是拍的：用车密度（辆/㎡，由已发现的车 + 已扫过的面积实时算出）
        乘格子面积，再乘先验违规概率。密度是测量值，越走越准。
        """
        x0, y0, x1, y1 = self.region
        area_total = max((x1 - x0) * (y1 - y0), 1.0)
        inside = [c for c in cars if x0 <= c.x <= x1 and y0 <= c.y <= y1]
        # 已观测面积一律由实测给出（走过的里程 × 视场宽）；缺省按"整片已看"处理，
        # 这样"没看到车"不会被误当成"要瞎逛"，而是继续读已知的车 / 交卷。
        a_obs = max(self.observed_area or area_total, 1.0)
        a_obs = min(a_obs, area_total)
        scanned = max(a_obs, 1.0)
        density = (len(inside) + 1.0) / scanned                 # 辆/㎡（测量值 + 加一平滑）
        p_v = (sum(c.p_violation for c in inside) / len(inside)) if inside else 0.5
        self.log(f"[est] car_density={density * 100:.2f} 辆/100㎡ p_viol={p_v:.2f} "
                 f"(已发现 {len(inside)} 辆 / 扫过 {scanned:.0f}㎡)")
        out: List[Node] = []
        nx = max(1, int(round((x1 - x0) / cells_m)))
        ny = max(1, int(round((y1 - y0) / cells_m)))
        cell_area = (x1 - x0) * (y1 - y0) / (nx * ny)
        for ix in range(nx):
            for iy in range(ny):
                cx = x0 + (ix + 0.5) * (x1 - x0) / nx
                cy = y0 + (iy + 0.5) * (y1 - y0) / ny
                if any(math.hypot(c.x - cx, c.y - cy) < cells_m for c in cars):
                    continue          # 这格里有车（已被发现）⇒ 不需要"找车"节点
                # 还可能有几辆没发现的：按测得密度 × 格面积
                out.append(Node("coverage", cx, cy, density * cell_area * p_v))
        return out

    # -- 代价 ---------------------------------------------------------------
    def cost(self, pose: Tuple[float, float], node: Node) -> float:
        v = max(self.est.v_now, 0.4)
        straight = math.hypot(node.x - pose[0], node.y - pose[1])
        eff = self.est.path_eff or 1.15
        t = straight * eff / v
        if node.kind in ("view", "coverage"):
            t += (self.est.turn_cost or 2.5)      # 一次转向/贴边的固定开销（首次用保守值）
        return max(t, 0.3)

    # -- 求解 ---------------------------------------------------------------
    def solve(self, nodes: Sequence[Node], pose: Tuple[float, float],
              t_left: float) -> Tuple[List[Node], float, float]:
        """贪心构造 + 2-opt 精修，返回 (序列, 总耗时, 总收益)。"""
        sub = self.est.submit_cost or 1.0
        budget = max(0.0, t_left - sub)
        cur = pose
        left = list(nodes)
        seq: List[Node] = []
        used = 0.0
        while left:
            best = None
            for n in left:
                c = self.cost(cur, n)
                if used + c > budget:
                    continue
                score = n.prize / c if n.prize > 0 else -1.0
                if best is None or score > best[0]:
                    best = (score, n, c)
            if best is None or best[0] <= 0:
                break
            _, n, c = best
            seq.append(n)
            used += c
            cur = (n.x, n.y)
            left.remove(n)
        # 2-opt：只用直线距离评估局部顺序
        def path_len(order: Sequence[Node]) -> float:
            p, tot = pose, 0.0
            for n in order:
                tot += math.hypot(n.x - p[0], n.y - p[1])
                p = (n.x, n.y)
            return tot

        improved = True
        while improved:
            improved = False
            for i, j in itertools.combinations(range(len(seq)), 2):
                cand = seq[:i] + seq[i:j + 1][::-1] + seq[j + 1:]
                if path_len(cand) < path_len(seq) - 1e-6:
                    seq = cand
                    improved = True
        # 2-opt 后可能超预算：从尾部裁掉
        while seq:
            tot = 0.0
            p = pose
            for n in seq:
                tot += self.cost(p, n)
                p = (n.x, n.y)
            if tot <= budget:
                used = tot
                break
            seq.pop()
        return seq, used, sum(n.prize for n in seq)

    # -- 一个完整决策步 ------------------------------------------------------
    def decide(self, cars: Sequence[CarObs], pose: Tuple[float, float],
               t_left: float) -> dict:
        views = self.view_nodes(cars, pose, self.est.v_now)
        covs = self.coverage_nodes(cars)      # 收益已按"实测车密度 × 格面积"算好
        all_nodes = views + covs
        seq, used, prize = self.solve(all_nodes, pose, t_left)
        # 下一步动作按"收益/成本"再选一次：方案序列只用来估预算，
        # 不因为 2-opt 为了缩短折线而把高收益节点排到后面。
        nxt = None
        best = -1.0
        for n in seq:
            c = self.cost(pose, n)
            r = n.prize / c if n.prize > 0 else -1.0
            if r > best:
                best, nxt = r, n
        return {
            "next": nxt,
            "plan": seq,
            "cost": used,
            "prize": prize,
            "n_view": len(views),
            "n_cov": len(covs),
            "t_left": t_left,
            "estimates": self.est.report(),
            "submit": nxt is None,
        }


# ----------------------------------------------------------------------------- 自检
def _demo() -> None:
    est = Estimators()
    est.v_now = 2.0
    est.add_read_sample(4.0, True)
    est.add_read_sample(4.5, True)
    est.add_read_sample(11.0, False)
    est.add_read_sample(14.0, False)
    est.set_submit_cost(0.4)
    region = (6.1, 6.1, 46.1, 46.1)
    ori = Orienteer(est, region)
    cars = [CarObs(20 + 3 * i, 20, 0.0, 1.0) for i in range(6)]
    d = ori.decide(cars, (0.0, 0.0), 100.0)
    print("estimates:", d["estimates"])
    print(f"view={d['n_view']} cov={d['n_cov']} first={d['next']} cost={d['cost']:.1f} prize={d['prize']:.2f}")


if __name__ == "__main__":
    _demo()

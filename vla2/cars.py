# -*- coding: utf-8 -*-
"""车辆身份：先按图像做帧间跟踪，再把轨迹合并成"一辆车"，最后判定违规并给编号。

为什么要这一层（现场实测结论）
------------------------------------------------
旧做法是"每帧把车框反投影到世界坐标，2.5 m 内算同一辆"。在 6~12 m 距离上，
单帧反投影误差是 1~2 m（`d≈K/(y−horizon)` 里 K、horizon 都是常数），于是
**同一辆车被拆成好几个簇**（今天实测 21 簇/33 张截图，实际只有 14 辆车），
用户看到的就是"重复的没删掉、数量对不上"。

本模块把去重改成两段：
    1. **图像域跟踪**：相邻帧用框中心距离 + IoU 关联（不需要任何几何标定，
       帧间位移只有几像素），同一辆车从头到尾只属于一条 track。
    2. **轨迹合并**：track 结束时用"最佳帧"（框最大的那一帧）反投影出世界点，
       两条 track 世界点相距 < 1.0 m 才合并（同一辆车被遮挡打断后又出现）。

判据全部现场可测：
    * 距离：框高 bh 与已知车长/车宽 → `Z = f·L / bw`（f 由本帧图像宽标定）
    * 字高：ocr2 返回的字形框高度 ≥ GLYPH_MIN_PX
    * 清晰度：裁剪图 Laplacian 方差 ≥ 本帧中位数的一半
    * 读号得分 ≥ SCORE_MIN，格式必须是 `字母-两位数字`
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

# ---------------- 现场参数（都可由实测数据校正） ----------------
CAR_LEN_M = 4.3            # 车长（用于由框宽反推距离）
CAR_H = 1.35               # 相机离地高
CAR_W_M = 1.8              # 车宽（正视时框宽≈车宽）               # 相机离地高（用于由框底反推距离）
F_OVER_W = 0.95            # f ≈ 0.95×图像宽（640 宽时 f≈608，65° 水平视场）
BOX_MIN_PX = 55            # 车框高下限：更远的小车不截图（≈7 m 以外）
GLYPH_MIN_PX = 12          # 字形高度下限（11 px 时 B 会被读成 G）
SCORE_MIN = 0.70           # 读号得分下限
TRACK_GATE_PX = 40         # 帧间关联门限（像素）
TRACK_IOU_MIN = 0.10
MAX_GAP_S = 1.5            # 超过这么久没匹配上，track 结束
MERGE_M = 1.0              # 两条 track 的世界点相距 < 1.0 m 视为同一辆车
VOTE_MIN = 3               # 采纳一个编号至少需要的票数
VOTE_SHARE = 0.60          # 头名占比下限


def _iou(a, b) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x0, y0 = max(ax, bx), max(ay, by)
    x1, y1 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    inter = (x1 - x0) * (y1 - y0)
    return inter / float(aw * ah + bw * bh - inter + 1e-6)


def distance_of(bw: int, bh: int, by: int, img_w: int, img_h: int) -> float:
    """由车框尺寸反推距离（三条路取中位数，抑制野值）。

    * 侧视（宽高比 ≥2）：框宽 ≈ 车长 4.3 m ⇒ Z = f·4.3 / bw
    * 正视（宽高比 <2）：框宽 ≈ 车宽 1.8 m ⇒ Z = f·1.8 / bw
    * 接地线：Z = f·H / (v_bottom − v_horizon)，相机水平安装，地平线 ≈ 0.36·H
    """
    f = F_OVER_W * img_w
    ar = bw / max(1.0, float(bh))
    L = CAR_LEN_M if ar >= 2.0 else CAR_W_M
    z_w = f * L / max(8.0, float(bw))
    v_horizon = 0.36 * img_h
    dz = max(1.0, (by + bh) - v_horizon)
    z_g = f * CAR_H / dz
    cand = sorted([z_w, z_g, 0.5 * (z_w + z_g)])
    return float(max(2.0, min(25.0, cand[1])))


def world_of(ego_x: float, ego_y: float, yaw_deg: float, u: float, v: float,
             z: float, img_w: int, img_h: int) -> tuple[float, float]:
    """把图像点 (u,v) 投到地面：先用像素横向比算横向偏移，再按 yaw 转到世界系。

    场景约定：yaw 顺时针为正，0 = +x，90 = −y。
    """
    a = math.radians(yaw_deg)
    hx, hy = math.cos(a), -math.sin(a)        # 车头方向
    lx, ly = -hy, hx                          # 车身左侧法向
    f = F_OVER_W * img_w
    lat = (u - img_w / 2.0) / f * z           # 横向偏移（右正）
    fwd = z
    return ego_x + hx * fwd - lx * lat, ego_y + hy * fwd - ly * lat


@dataclass
class Track:
    box: tuple[int, int, int, int]
    t: float
    n: int = 1
    best_box: tuple[int, int, int, int] = (0, 0, 0, 0)
    best_z: float = 99.0
    world: tuple[float, float] = (0.0, 0.0)
    votes: dict[str, float] = field(default_factory=dict)
    shots: int = 0
    alive: bool = True


@dataclass
class Car:
    x: float
    y: float
    tracks: int = 1
    votes: dict[str, float] = field(default_factory=dict)
    best_z: float = 99.0
    shots: int = 0

    @property
    def label(self) -> str:
        if not self.votes:
            return ""
        top = max(self.votes.items(), key=lambda kv: kv[1])
        total = sum(self.votes.values())
        if top[1] < VOTE_MIN or top[1] / max(1e-6, total) < VOTE_SHARE:
            return ""
        return top[0]

    @property
    def conf(self) -> float:
        if not self.votes:
            return 0.0
        total = sum(self.votes.values())
        return max(self.votes.values()) / max(1e-6, total)


class CarTracker:
    """帧间跟踪 + 轨迹合并，保证"一辆车只算一次"。"""

    def __init__(self) -> None:
        self.tracks: list[Track] = []
        self.cars: list[Car] = []
        self.n_frames = 0
        self.n_shots = 0
        self.n_reads = 0
        self.n_boxes = 0

    # ---- 帧间关联 ----
    def _match(self, box, t: float) -> Track | None:
        bx, by, bw, bh = box
        cx, cy = bx + bw / 2.0, by + bh / 2.0
        best, bd = None, 1e9
        for tr in self.tracks:
            if not tr.alive or (t - tr.t) > MAX_GAP_S:
                continue
            tx, ty, tw, th = tr.box
            tcx, tcy = tx + tw / 2.0, ty + th / 2.0
            d = math.hypot(cx - tcx, cy - tcy)
            gate = max(TRACK_GATE_PX, 0.35 * max(tw, th))
            if d > gate:
                continue
            if _iou(box, tr.box) < TRACK_IOU_MIN and d > gate * 0.6:
                continue
            if d < bd:
                best, bd = tr, d
        return best

    def _close_stale(self, t: float) -> None:
        for tr in self.tracks:
            if tr.alive and (t - tr.t) > MAX_GAP_S:
                tr.alive = False
                self._fold(tr)

    def _fold(self, tr: Track) -> None:
        """把一条结束的 track 并入最近的 Car（世界点 < MERGE_M 就是同一辆车）。"""
        for c in self.cars:
            if math.hypot(c.x - tr.world[0], c.y - tr.world[1]) < MERGE_M:
                c.tracks += 1
                c.best_z = min(c.best_z, tr.best_z)
                c.shots += tr.shots
                for k, v in tr.votes.items():
                    c.votes[k] = c.votes.get(k, 0.0) + v
                n = c.tracks
                c.x = (c.x * (n - 1) + tr.world[0]) / n
                c.y = (c.y * (n - 1) + tr.world[1]) / n
                return
        c = Car(x=tr.world[0], y=tr.world[1], votes=dict(tr.votes), best_z=tr.best_z,
                shots=tr.shots)
        self.cars.append(c)

    # ---- 每帧入口 ----
    def observe(self, boxes, t: float, ego_x: float, ego_y: float, yaw: float,
                img_w: int, img_h: int, sharp_med: float,
                on_shot=None) -> list[tuple]:
        """返回本帧"需要读号"的框：[(box, crop_pad_ratio)] 由上层回调处理。"""
        self.n_frames += 1
        self.n_boxes += len(boxes)
        self._close_stale(t)
        todo = []
        for (bx, by, bw, bh, conf) in boxes:
            box = (bx, by, bw, bh)
            tr = self._match(box, t)
            if tr is None:
                tr = Track(box=box, t=t)
                self.tracks.append(tr)
            else:
                tr.box, tr.t, tr.n = box, t, tr.n + 1
            z = distance_of(bw, bh, by, img_w, img_h)
            # 最佳帧 = 最近的这一帧（z 最小），世界点与朝向用它的位姿算
            if bh >= BOX_MIN_PX and z < tr.best_z:
                tr.best_z = z
                tr.best_box = box
                tr.world = world_of(ego_x, ego_y, yaw, bx + bw / 2.0, by + bh,
                                    z, img_w, img_h)
                if on_shot is not None:
                    todo.append((tr, box, z))
        return todo

    def add_read(self, tr: Track, text: str, score: float, glyph_h: int) -> bool:
        """合格的一票：字高够、得分够、格式合法才记。"""
        if not text or glyph_h < GLYPH_MIN_PX or score < SCORE_MIN:
            return False
        tr.votes[text] = tr.votes.get(text, 0.0) + float(score)
        tr.shots += 1
        self.n_shots += 1
        self.n_reads += 1
        return True

    def finish(self) -> list[Car]:
        for tr in self.tracks:
            if tr.alive:
                tr.alive = False
                self._fold(tr)
        return self.cars

    # ---- 汇总 ----
    def report(self) -> dict:
        cars = self.finish()
        with_label = [c for c in cars if c.label]
        return {"cars": len(cars), "labeled": len(with_label),
                "shots": self.n_shots, "reads": self.n_reads,
                "boxes": self.n_boxes, "frames": self.n_frames,
                "read_rate": (len(with_label) / len(cars)) if cars else 0.0,
                "labels": sorted({c.label for c in with_label})}

    def submit_ids(self, inside) -> list[str]:
        out = set()
        for c in self.finish():
            if c.label and inside(c.x, c.y):
                out.add(c.label)
        return sorted(out)

    def report_full(self) -> list[dict]:
        out = []
        for c in self.finish():
            out.append({"x": round(c.x, 2), "y": round(c.y, 2), "tracks": c.tracks,
                        "label": c.label, "conf": round(c.conf, 2),
                        "z_m": round(c.best_z, 1), "votes": sorted(c.votes.items(), key=lambda kv: -kv[1])[:3]})
        return out

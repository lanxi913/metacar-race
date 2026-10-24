# -*- coding: utf-8 -*-
r"""连续场景（当前主用场景4）通用跑分骨架。

与 `park_cs_single_arc.py` 的关系：轨迹跟踪、曲率剖面、速度环直接复用后者验证过的
那套（`PurePursuit` + `SpeedProfile` + `SpeedController`），区别在于**不再绑死泊车**，
而是补上跑完 12 个元素必须有的三件事：

1. **看得见障碍**：每帧把障碍投影到路线上，算出"前方多远、偏多少、什么类型"，
   落 `obs.jsonl`；据此限速 / 刹停。没有这一层，车顶着前车会一直给 0.85 油门干蹭
   （2026-10-24 16:43 的实测：速度 0.06 m/s 蹭了 55 s，场景被平台收掉）。
2. **起步破挡**：本车升挡门槛在油门 0.7 上下，闭环追 8 m/s 时长时间给 0.45~0.5
   推不动（场景4 采集时 91 s 才升挡）。这里在"油门 >= 0.5 且速度 < 0.25"持续
   1.5 s 后阶梯补油，升挡后自动退回。
3. **卡死检测**：位置 4 s 纹丝不动就记 `stuck` 事件并落一份障碍快照，事后一眼看出
   是谁挡的；不再出现"跑完才知道车在原地"。

元素按 `static.sub_scenes` 顺序切段，`--skip-to-index N` 直达第 N 个元素（0=全程）。

产物 `logs/park_cs4/<时间戳>/`：

    run.csv       逐帧：位姿/命令/v_tgt/route_s/lat_err/前方障碍
    obs.jsonl     障碍快照（集合变化、或卡死时强制写）
    events.jsonl  事件：skip / element / breakaway / stuck / end
    summary.json  收尾摘要

用法（先跑脚本，再在客户端点「启动场景」）：

    .venv\Scripts\python.exe -u tools\park_cs4.py --skip-to-index 2
    .venv\Scripts\python.exe -u tools\park_cs4.py --skip-to-index 0 --vmax 8.0
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from metacar import GearMode, VehicleControl, SceneAPI  # noqa: E402

from metacar_race.config import LOOKAHEAD_M  # noqa: E402
from metacar_race.control.profile import SpeedProfile  # noqa: E402
from metacar_race.control.tracking import PurePursuit, SpeedController  # noqa: E402

PARKING_SLOT_TYPE = 30
SPEED_SIGN_TYPE = 26
IGNORE_TYPES = {PARKING_SLOT_TYPE, SPEED_SIGN_TYPE}

LANE_HALF_M = 1.90          # 判定"在同一条车道里"的横向半宽（车道 3.76 m + 车身余量）
BLOCK_HORIZON_M = 45.0      # 只关心前方这么远的障碍
BLOCK_MARGIN_M = 4.80       # 车长一半 + 安全余量，从障碍距离里扣掉
BLOCK_SLOW_M = 30.0         # 进入这个距离开始限速
BLOCK_STOP_M = 7.0          # 这个距离内直接刹停
FOLLOW_DECEL = 0.45         # 限速曲线 v = FOLLOW_DECEL * (d - 6)

OVERTAKE_OFFSET_M = 3.50    # 绕行时往相邻车道偏多少（车道宽 3.5~3.8）
OVERTAKE_TRIGGER_M = 30.0   # 前方这个距离内出现静止车就开始准备绕
OVERTAKE_WAIT_S = 0.60      # 连续判定这么久才真正开始
OVERTAKE_CLEAR_M = 60.0     # 目标车道要空出这么远才敢进
OVERTAKE_BAND_M = 2.20      # 目标车道占用判定的横向半宽
OVERTAKE_SPEED = 6.0        # 绕行时的目标速度上限
OVERTAKE_RAMP_RATE = 0.25   # 每行驶 1 m 允许横移多少米（3.5 m 需跑 14 m）
OVERTAKE_TIMEOUT_S = 30.0

STALL_SPEED_MS = 0.25       # "推不动"的速度阈值
STALL_THROTTLE = 0.50       # 油门给到这个值还推不动就开始补油
STALL_TRIGGER_S = 1.50
STALL_STEP = 0.05           # 每 0.5 s 补这么多油门
STALL_STEP_EVERY = 0.50

STUCK_WINDOW_S = 4.0
STUCK_MOVE_M = 0.30

HOLD_END_S = 8.0


def save_shot(camera_frames, out_dir: Path, tag: str) -> None:
    """存一张前视相机图（cv2 直接吃 ndarray）。"""
    try:
        import cv2  # noqa: PLC0415
    except Exception:  # noqa: BLE001
        return
    for cam in camera_frames or []:
        img = getattr(cam, "frame", None)
        if img is None:
            continue
        try:
            name = "%s_%s.jpg" % (tag, getattr(cam, "id", "cam"))
            path = out_dir / "shots" / name
            path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(path), img)
        except Exception:  # noqa: BLE001
            pass


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


class RouteTrack:
    """路线几何：累计里程 + 局部索引跟踪 + 障碍投影。"""

    def __init__(self, route) -> None:
        self.pts = [(float(p.x), float(p.y)) for p in route]
        self.n = len(self.pts)
        self.s = [0.0] * self.n
        for i in range(1, self.n):
            self.s[i] = self.s[i - 1] + math.hypot(self.pts[i][0] - self.pts[i - 1][0],
                                                   self.pts[i][1] - self.pts[i - 1][1])
        self.idx = 0

    @property
    def length(self) -> float:
        return self.s[-1] if self.n else 0.0

    def sync(self, x: float, y: float) -> int:
        """就近定位；跳关后位移很大时退回全局搜索。"""
        lo = max(0, self.idx - 30)
        hi = min(self.n, self.idx + 400)
        best, bd = self.idx, float("inf")
        for i in range(lo, hi):
            px, py = self.pts[i]
            d = (px - x) ** 2 + (py - y) ** 2
            if d < bd:
                bd, best = d, i
        if bd > 400.0:  # 20 m 以外：认为被瞬移了，全局重找
            best, bd = 0, float("inf")
            for i in range(self.n):
                px, py = self.pts[i]
                d = (px - x) ** 2 + (py - y) ** 2
                if d < bd:
                    bd, best = d, i
        self.idx = best
        return best

    def tangent(self, i: int) -> tuple[float, float]:
        j = min(self.n - 1, i + 3)
        tx = self.pts[j][0] - self.pts[i][0]
        ty = self.pts[j][1] - self.pts[i][1]
        n = math.hypot(tx, ty) or 1.0
        return tx / n, ty / n

    def project(self, x: float, y: float, lo: int, hi: int) -> tuple[int, float]:
        """在 [lo,hi) 里找最近点，返回 (idx, 横向距离)。"""
        best, bd = lo, float("inf")
        for i in range(lo, hi):
            px, py = self.pts[i]
            d = (px - x) ** 2 + (py - y) ** 2
            if d < bd:
                bd, best = d, i
        return best, math.sqrt(bd)


class ObstacleScan:
    """把障碍投影到"自车前方这一段路线"上。"""

    @staticmethod
    def scan(track: RouteTrack, i_now: int, obstacles, horizon_m: float = BLOCK_HORIZON_M):
        """返回 (block, items)。

        block = {"dist": 沿路线距离, "lat": 横向偏差, "id":..., "type":...} 或 None
        items = 每个障碍的投影结果，供日志用
        """
        hi = min(track.n, i_now + int(horizon_m) + 30)
        s_now = track.s[i_now]
        items = []
        block = None
        for obs in obstacles:
            try:
                otype = int(obs.type.value)
            except Exception:  # noqa: BLE001
                otype = -1
            ox, oy = float(obs.pos_x), float(obs.pos_y)
            j, lat = track.project(ox, oy, i_now, hi)
            dist = track.s[j] - s_now
            vmag = math.hypot(float(getattr(obs, "vel_x", 0.0) or 0.0),
                              float(getattr(obs, "vel_y", 0.0) or 0.0))
            item = {
                "id": int(getattr(obs, "id", -1)),
                "type": otype,
                "x": round(ox, 2),
                "y": round(oy, 2),
                "v": round(vmag, 2),
                "len": round(float(getattr(obs, "length", 0.0) or 0.0), 2),
                "wid": round(float(getattr(obs, "width", 0.0) or 0.0), 2),
                "ori": round(float(getattr(obs, "ori_z", 0.0) or 0.0), 1),
                "dist": round(dist, 2),
                "lat": round(lat, 2),
            }
            items.append(item)
            if otype in IGNORE_TYPES:
                continue
            if dist < -1.0 or dist > horizon_m or lat > LANE_HALF_M:
                continue
            if block is None or dist < block["dist"]:
                block = item
        items.sort(key=lambda o: o["dist"] if o["dist"] > -1.0 else 1e9)
        return block, items


class StallGuard:
    """起步破挡：油门给足但速度不涨时阶梯补油，直到平台升挡。"""

    def __init__(self) -> None:
        self.stall_t = 0.0
        self.extra = 0.0
        self.step_t = 0.0
        self.active = False

    def update(self, throttle: float, speed: float, dt: float) -> float:
        if throttle >= STALL_THROTTLE and abs(speed) < STALL_SPEED_MS:
            self.stall_t += dt
        else:
            if self.stall_t > 0.0:
                self.stall_t = 0.0
                self.extra = 0.0
                self.step_t = 0.0
            self.active = False
            return throttle
        if self.stall_t < STALL_TRIGGER_S:
            return throttle
        self.active = True
        self.step_t += dt
        while self.step_t >= STALL_STEP_EVERY:
            self.step_t -= STALL_STEP_EVERY
            self.extra = min(1.0, self.extra + STALL_STEP)
        return min(1.0, throttle + self.extra)


class StuckWatch:
    """位置 4 s 不动就报卡死。"""

    def __init__(self) -> None:
        self.hist: deque[tuple[float, float, float]] = deque()

    def update(self, t: float, x: float, y: float) -> bool:
        self.hist.append((t, x, y))
        while self.hist and t - self.hist[0][0] > STUCK_WINDOW_S:
            self.hist.popleft()
        if len(self.hist) < 6:
            return False
        x0, y0 = self.hist[0][1], self.hist[0][2]
        return all(math.hypot(px - x0, py - y0) < STUCK_MOVE_M for _, px, py in self.hist)


class Pt:
    """只带 x/y 的轻量路点，给 PurePursuit 用。"""

    __slots__ = ("x", "y")

    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y


class ShiftRoute:
    """可临时横移的路线：只改车辆附近一段，不每帧重建 4700 个点。"""

    def __init__(self, route) -> None:
        self.base = [(float(p.x), float(p.y)) for p in route]
        self.n = len(self.base)
        self.items = [Pt(bx, by) for bx, by in self.base]
        self.nx = [0.0] * self.n
        self.ny = [0.0] * self.n
        for k in range(self.n):
            a = self.base[max(0, k - 1)]
            b = self.base[min(self.n - 1, k + 1)]
            tx, ty = b[0] - a[0], b[1] - a[1]
            m = math.hypot(tx, ty)
            if m < 1e-6:
                continue
            self.nx[k] = -ty / m      # 左法向
            self.ny[k] = tx / m

    def apply(self, off: float, i_now: int, span: int = 90) -> list[Pt]:
        lo = max(0, i_now - 12)
        hi = min(self.n, i_now + span)
        for k in range(lo, hi):
            bx, by = self.base[k]
            self.items[k].x = bx + self.nx[k] * off
            self.items[k].y = by + self.ny[k] * off
        return self.items


class Overtaker:
    """前方车道被静止车堵死时，平移到相邻车道绕过去，过了再回本车道。

    只在"前车几乎不动（v<0.6）且在本车道走廊里"时触发；目标车道要空出
    OVERTAKE_CLEAR_M 才敢并进去。横移按行驶距离限速（OVERTAKE_RAMP_RATE），
    不会原地打方向。
    """

    def __init__(self, offset: float = OVERTAKE_OFFSET_M, enabled: bool = True) -> None:
        self.offset = float(offset)
        self.enabled = enabled
        self.active = False
        self.side = -1.0          # -1 往右（lat 减小），+1 往左
        self.lat_off = 0.0
        self.target_id = None
        self.wait_t = 0.0
        self.t_enter = 0.0
        self.done: set[int] = set()

    def _free(self, items, off: float) -> bool:
        """目标车道在 0~OVERTAKE_CLEAR_M 内是否空着。"""
        for o in items:
            if o["type"] in IGNORE_TYPES:
                continue
            if o["dist"] < -3.0 or o["dist"] > OVERTAKE_CLEAR_M:
                continue
            if abs(o["lat"] - off) < OVERTAKE_BAND_M:
                return False
        return True

    def update(self, t: float, block, items, dt: float, ds: float) -> dict:
        out = {"lat_off": self.lat_off, "active": self.active, "started": False,
               "finished": False, "side": self.side, "target_id": self.target_id}
        step = max(ds, 0.02) * OVERTAKE_RAMP_RATE

        if self.active:
            tgt = self.side * self.offset
            if abs(tgt - self.lat_off) <= step:
                self.lat_off = tgt
            else:
                self.lat_off += step if tgt > self.lat_off else -step
            blk = None
            for o in items:
                if o["id"] == self.target_id:
                    blk = o
                    break
            if blk is None or blk["dist"] < -2.5 or t - self.t_enter > OVERTAKE_TIMEOUT_S:
                self.active = False
                self.done.add(self.target_id)
                out["finished"] = True
        else:
            if abs(self.lat_off) <= step:
                self.lat_off = 0.0
            else:
                self.lat_off += -step if self.lat_off > 0 else step
            blocked = (self.enabled and self.lat_off == 0.0 and block is not None
                       and block["id"] not in self.done and block["v"] < 0.60
                       and 4.0 < block["dist"] < OVERTAKE_TRIGGER_M)
            self.wait_t = self.wait_t + dt if blocked else 0.0
            if self.wait_t >= OVERTAKE_WAIT_S:
                # 先试左侧（国内超车侧；本场景左侧是可并行的同向车道，边界为虚线），
                # 左侧被占才退到右侧。
                for side in (1.0, -1.0):
                    if self._free(items, side * self.offset):
                        self.active = True
                        self.side = side
                        self.target_id = block["id"]
                        self.t_enter = t
                        self.wait_t = 0.0
                        out["started"] = True
                        break
                else:
                    self.wait_t = 0.0

        out["lat_off"] = self.lat_off
        out["active"] = self.active
        out["side"] = self.side
        out["target_id"] = self.target_id
        return out


def main() -> int:
    parser = argparse.ArgumentParser(description="连续场景通用跑分骨架")
    parser.add_argument("--skip-to-index", type=int, default=0,
                        help="只跑第 N 个元素（1 起，0=全程）")
    parser.add_argument("--vmax", type=float, default=8.0)
    parser.add_argument("--max-seconds", type=float, default=700.0)
    parser.add_argument("--end-hold", type=float, default=HOLD_END_S)
    parser.add_argument("--shots", type=float, default=0.0,
                        help="每隔多少秒存一张前视相机图到 shots/（0=不存）")
    parser.add_argument("--no-overtake", action="store_true",
                        help="关掉自动绕行（默认开启）")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    track = RouteTrack(static.route)
    print(f"已连接。元素 {len(static.sub_scenes)} 个，赛道 {track.n} 点 / "
          f"{track.length:.1f} m", flush=True)

    target = ""
    elem_start = None
    if 1 <= args.skip_to_index <= len(static.sub_scenes):
        elem = static.sub_scenes[args.skip_to_index - 1]
        target = elem.name
        if elem.start_point is not None:
            elem_start = (float(elem.start_point.x), float(elem.start_point.y))
        print(f"目标元素：{target}（其余自动跳关）", flush=True)

    pursuit = PurePursuit(static.route, lookahead=LOOKAHEAD_M) if track.n >= 2 else None
    shift_route = ShiftRoute(static.route) if pursuit is not None else None
    if pursuit is not None and shift_route is not None:
        pursuit.route = shift_route.items
    overtaker = Overtaker(enabled=not args.no_overtake)
    profile = SpeedProfile(static.route, v_max=args.vmax) if track.n >= 3 else None
    speed_ctl = SpeedController(v_target=args.vmax, accel_limit=1.5, throttle_max=0.85)
    speed_ctl.reset(0.0)
    stall = StallGuard()
    stuck_watch = StuckWatch()

    out_dir = args.out or (REPO / "logs" / "park_cs4" / time.strftime("%Y%m%d-%H%M%S"))
    out_dir.mkdir(parents=True, exist_ok=True)
    run_fh = (out_dir / "run.csv").open("w", newline="", encoding="utf-8-sig")
    run_w = csv.writer(run_fh)
    run_w.writerow([
        "frame", "sim_t", "sub_scene", "elem_i", "phase", "x", "y", "yaw", "speed",
        "gear", "throttle", "brake", "steering", "v_tgt", "route_s", "lat_err",
        "blk_dist", "blk_type", "blk_id", "obs_n", "stuck", "lat_off",
    ])
    obs_fh = (out_dir / "obs.jsonl").open("w", encoding="utf-8")
    ev_fh = (out_dir / "events.jsonl").open("w", encoding="utf-8")

    def emit(kind: str, **kw) -> None:
        rec = {"sim_t": round(sim_t, 3), "frame": frame, "event": kind}
        rec.update(kw)
        ev_fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
        ev_fh.flush()

    phase = "skip" if target else "cruise"
    skipped: set[str] = set()
    elem_i = 0
    cur_sub = ""
    hold_t = 0.0
    last_obs_key = None
    last_stuck_log = -99.0
    last_shot = -99.0
    last_break_log = -99.0
    frame = 0
    sim_t = 0.0
    dt = 1.0 / 30.0
    t_end = time.time() + args.max_seconds
    ending = "ended"
    final = (0.0, 0.0, 0.0)

    try:
        for frame, (msg, _frames) in enumerate(api.main_loop(), start=1):
            pose = msg.pose_gnss
            car = msg.main_vehicle
            status = msg.scene_status
            sim_t += dt
            sub = status.sub_scene_name or ""
            x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
            speed = float(car.speed)
            if args.shots > 0 and _frames and sim_t - last_shot >= args.shots:
                last_shot = sim_t
                save_shot(_frames, out_dir, "t%06.1f" % sim_t)
            final = (x, y, yaw)

            if sub and sub != cur_sub:
                for k, sc in enumerate(static.sub_scenes, start=1):
                    if sc.name == sub:
                        cur_sub = sub
                        elem_i = k
                        emit("element", name=sub, index=k,
                             limit=round(float(status.time_limit), 1))
                        break

            # ---- 跳关 ----
            if phase == "skip":
                if target and sub != target:
                    if sub not in skipped and len(skipped) < 12:
                        skipped.add(sub)
                        api.skip_level()
                        print(f"跳过元素 {sub}", flush=True)
                        emit("skip", name=sub)
                    api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                    continue
                arrived = True
                if elem_start is not None:
                    arrived = math.hypot(elem_start[0] - x, elem_start[1] - y) <= 20.0
                if not arrived:
                    api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                    continue
                phase = "cruise"
                i_now = track.sync(x, y)
                if pursuit is not None:
                    pursuit.sync(x, y, max_jump_m=1.0)
                print(f"进入 {sub}，位置 ({x:.2f},{y:.2f}) route_s={track.s[i_now]:.1f}",
                      flush=True)

            # ---- 定位 & 障碍 ----
            i_now = track.sync(x, y)
            block, obs_items = ObstacleScan.scan(track, i_now, msg.obstacles or [])
            obs_n = len(obs_items)

            obs_key = tuple(sorted((o["id"], o["type"]) for o in obs_items))
            stuck_now = stuck_watch.update(sim_t, x, y)
            if obs_key != last_obs_key or stuck_now:
                last_obs_key = obs_key
                obs_fh.write(json.dumps({
                    "sim_t": round(sim_t, 3), "frame": frame,
                    "ego": [round(x, 2), round(y, 2)],
                    "route_s": round(track.s[i_now], 1),
                    "block": block, "obstacles": obs_items,
                }, ensure_ascii=False) + "\n")
                obs_fh.flush()

            if stuck_now and sim_t - last_stuck_log > 3.0:
                last_stuck_log = sim_t
                emit("stuck", pos=[round(x, 3), round(y, 3)], speed=round(speed, 3),
                     block=block, n=obs_n)
                save_shot(_frames, out_dir, "stuck%06.1f" % sim_t)
                print(f"[{sim_t:6.1f}s] 卡死 pos=({x:8.2f},{y:8.2f}) v={speed:.2f} "
                      f"block={block}", flush=True)

            # ---- 绕行（超车）----
            ds = max(abs(speed) * dt, 0.0)
            ov = overtaker.update(sim_t, block, obs_items, dt, ds)
            lat_off = ov["lat_off"]
            if shift_route is not None:
                pursuit.route = shift_route.apply(lat_off, i_now)
            if ov["started"]:
                emit("overtake_start", side=ov["side"], target_id=ov["target_id"],
                     s=round(float(track.s[i_now]), 1))
                print(f"[{sim_t:6.1f}s] 开始绕行 side={ov['side']:+.0f} "
                      f"id={ov['target_id']} v={speed:.2f}", flush=True)
            if ov["finished"]:
                emit("overtake_end", s=round(float(track.s[i_now]), 1))
                print(f"[{sim_t:6.1f}s] 绕行完成，回本车道", flush=True)

            blk_ctrl = block
            if ov["active"] and block is not None and block["id"] == ov["target_id"]:
                blk_ctrl = None   # 正在绕的车不再当跟车目标

            # ---- 速度指令 ----
            v_tgt = profile.target(i_now, speed) if profile is not None else args.vmax
            if blk_ctrl is not None:
                d_eff = blk_ctrl["dist"] - BLOCK_MARGIN_M
                if d_eff < BLOCK_SLOW_M:
                    v_cap = max(0.0, FOLLOW_DECEL * (d_eff - 1.5))
                    if v_cap < v_tgt:
                        v_tgt = v_cap
            if ov["active"]:
                v_tgt = min(v_tgt, OVERTAKE_SPEED)

            steer = pursuit.steering(x, y, yaw) if pursuit is not None else 0.0
            speed_ctl.v_target = v_tgt
            thr, brk = speed_ctl.update(speed, dt, v_ceiling=args.vmax)
            thr = stall.update(thr, speed, dt)
            if stall.active and sim_t - last_break_log > 2.0 and thr > 0.55:
                last_break_log = sim_t
                emit("breakaway", throttle=round(thr, 2), speed=round(speed, 2))
            if blk_ctrl is not None and blk_ctrl["dist"] < BLOCK_STOP_M and speed > 0.30:
                thr, brk = 0.0, 0.35
            thr = min(1.0, max(0.0, thr))
            brk = min(1.0, max(0.0, brk))

            # ---- 到终点就驻车 ----
            if track.length - track.s[i_now] < 3.0:
                phase = "hold"
            if phase == "hold":
                hold_t += dt
                api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
                run_w.writerow([
                    frame, round(sim_t, 3), sub, elem_i, "hold", round(x, 3), round(y, 3),
                    round(yaw, 2), round(speed, 3), car.gear.name, 0.0, 1.0, 0.0,
                    round(v_tgt, 2), round(track.s[i_now], 1), 0.0,
                    "" if block is None else round(block["dist"], 2),
                    "" if block is None else block["type"],
                    "" if block is None else block["id"], obs_n, int(stuck_now),
                    round(lat_off, 2),
                ])
                if frame % 60 == 0:
                    run_fh.flush()
                    print(f"[{sim_t:6.1f}s] hold 到终点，驻车 {hold_t:.1f}s", flush=True)
                if hold_t >= args.end_hold:
                    ending = "hold_end"
                    break
                if time.time() > t_end:
                    ending = "timeout"
                    break
                continue

            vc = VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=steer)
            api.set_vehicle_control(vc)

            tx, ty = track.tangent(i_now)
            lat_err = (x - track.pts[i_now][0]) * (-ty) + (y - track.pts[i_now][1]) * tx
            run_w.writerow([
                frame, round(sim_t, 3), sub, elem_i, phase, round(x, 3), round(y, 3),
                round(yaw, 2), round(speed, 3), car.gear.name,
                round(thr, 3), round(brk, 2), round(steer, 3), round(v_tgt, 2),
                round(track.s[i_now], 1), round(lat_err, 3),
                "" if block is None else round(block["dist"], 2),
                "" if block is None else block["type"],
                "" if block is None else block["id"], obs_n, int(stuck_now),
                round(lat_off, 2),
            ])
            if frame % 30 == 0:
                run_fh.flush()
                print(f"[{sim_t:6.1f}s] {sub:8s} s={track.s[i_now]:7.1f} "
                      f"pos=({x:8.1f},{y:8.1f}) v={speed:5.2f} tgt={v_tgt:5.2f} "
                      f"thr={thr:4.2f} blk={'' if block is None else block['dist']:>6} "
                      f"obs={obs_n}", flush=True)

            if time.time() > t_end:
                ending = "timeout"
                break
    except KeyboardInterrupt:
        ending = "interrupt"
    finally:
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass
        run_fh.close()
        obs_fh.close()
        emit("end", ending=ending, phase=phase)

    summary = {
        "ending": ending,
        "phase": phase,
        "frames": frame,
        "route_s": round(track.s[track.idx], 1) if track.n else 0.0,
        "final": [round(final[0], 3), round(final[1], 3), round(final[2], 2)],
        "dir": str(out_dir),
    }
    (out_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    ev_fh.close()
    print("SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

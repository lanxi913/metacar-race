"""障碍投影、起步破挡、卡死检测。

三个都是"看着状态做判断"的护栏，不直接产出控制量：
`ObstacleScan` 把障碍投到路线上，`StallGuard` 只在车该走没走时补油门，
`StuckWatch` 只报告卡死。
"""

from __future__ import annotations

import math
from collections import deque

from . import config as C
from .track import RouteTrack


class ObstacleScan:
    """把障碍投影到"自车前方这一段路线"上。"""

    @staticmethod
    def scan(track: RouteTrack, i_now: int, obstacles,
             horizon_m: float = C.BLOCK_HORIZON_M, lat_ref: float = 0.0):
        """返回 (block, items)。

        block：本道正前方最近的障碍（dict，含沿路线距离 dist 与横向 lat）或 None
        items：所有障碍的投影结果（按 dist 升序），供日志/超车状态机用

        `lat_ref` 是自车当前的横向位置（相对路线中心，左正）：换道/切弯时把它传进来，
        跟车与硬刹就只看**自车所在的那条带**，不会把隔壁道的前车当成本道堵点。
        elem6-r1 那次在 +3.50 m 的左道上被右道静止车压到 0.03 m/s 就是这个 bug 的表现。
        """
        hi = min(track.n, i_now + int(horizon_m) + 30)
        lo = max(0, i_now - 40)      # 往后看一点，后方障碍才会落到负距离
        s_now = track.s[i_now]
        lat_ref = float(lat_ref)
        items = []
        block = None
        for obs in obstacles:
            try:
                otype = int(obs.type.value)
            except Exception:  # noqa: BLE001
                otype = -1
            ox, oy = float(obs.pos_x), float(obs.pos_y)
            j, lat = track.project(ox, oy, lo, hi)
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
            if otype in C.IGNORE_TYPES:
                continue
            if dist < -1.0 or dist > horizon_m or abs(lat - lat_ref) > C.LANE_HALF_M:
                continue
            if block is None or dist < block["dist"]:
                block = item
        items.sort(key=lambda o: o["dist"] if o["dist"] > -1.0 else 1e9)
        return block, items


class StallGuard:
    """起步破挡：油门给足但速度不涨时阶梯补油，直到平台升挡。

    一挡是平台给的"墙"（`UPSHIFT_SPEED_THRESHOLD`），油门不加深就永远贴着 4.78
    m/s 走——元素 2 因为这条墙超时过（full-r2）。
    """

    def __init__(self) -> None:
        self.stall_t = 0.0
        self.extra = 0.0
        self.step_t = 0.0
        self.active = False

    def reset(self) -> None:
        self.stall_t = 0.0
        self.extra = 0.0
        self.step_t = 0.0
        self.active = False

    def update(self, throttle: float, speed: float, dt: float) -> float:
        if throttle >= C.STALL_THROTTLE and abs(speed) < C.STALL_SPEED_MS:
            self.stall_t += dt
        else:
            if self.stall_t > 0.0:
                self.reset()
            self.active = False
            return throttle
        if self.stall_t < C.STALL_TRIGGER_S:
            return throttle
        self.active = True
        self.step_t += dt
        while self.step_t >= C.STALL_STEP_EVERY:
            self.step_t -= C.STALL_STEP_EVERY
            self.extra = min(1.0, self.extra + C.STALL_STEP)
        return min(1.0, throttle + self.extra)


class StuckWatch:
    """位置 4 s 不动就报卡死（只报，不处置——处置交给 strategy/driver）。"""

    def __init__(self) -> None:
        self.hist: deque[tuple[float, float, float]] = deque()

    def reset(self) -> None:
        self.hist.clear()

    def update(self, t: float, x: float, y: float) -> bool:
        self.hist.append((t, x, y))
        while self.hist and t - self.hist[0][0] > C.STUCK_WINDOW_S:
            self.hist.popleft()
        if len(self.hist) < 6:
            return False
        x0, y0 = self.hist[0][1], self.hist[0][2]
        return all(math.hypot(px - x0, py - y0) < C.STUCK_MOVE_M
                   for _, px, py in self.hist)


class TeleportWatch:
    """平台瞬移检测：一帧位移超过 `TELEPORT_MIN_M` 且超过 2·v·dt+1.5 就认为被拽走。

    元素超时重开、跳关、复位都表现为"一帧几十米"，必须和正常行驶区分开：
    跟踪器只增不减，被拽回去后会一直朝几百米外的前视点开（full-r1 开出路面）。
    """

    def __init__(self) -> None:
        self.last: tuple[float, float] | None = None
        self.count = 0

    def reset(self) -> None:
        self.last = None
        self.count = 0

    def update(self, x: float, y: float, speed: float, dt: float) -> tuple[bool, float]:
        jump = 0.0
        teleport = False
        if self.last is not None:
            jump = math.hypot(x - self.last[0], y - self.last[1])
            if jump > max(C.TELEPORT_MIN_M, 2.0 * abs(speed) * dt + 1.5):
                teleport = True
        self.last = (x, y)
        if teleport:
            self.count += 1
        return teleport, jump

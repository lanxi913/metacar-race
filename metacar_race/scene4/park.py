"""元素 12「平行车道泊车」（侧方停车）：复用 main 已验证的几何推导控制器。

为什么是「复用 + 旋转」而不是重写：
    main 的 `metacar_race/control/parallel_parking.py`（PR#19，`f2a76a6`）已在平行泊车
    单场景连续 5 次拿 210 分、停位离散 < 3 cm，`docs/parallel-parking-calibration.md`
    把它的四个坑写得很清楚（staging 对齐远端那辆车、舵角斜坡、收尾轻刹、换向前停稳）。
    本机连续场景 4 的平行泊车与它几何同构，只差一个 180° 旋转：

        单场景：ego 朝 −y，车位在右侧 −x，远端车在车位 −y 方向
        连续 4：ego 朝 +y，车位在右侧 +x，远端车在车位 +y 方向

    绕车位中心旋 180° 正好把前者映到后者，且旋转保持手性 ⇒ 舵角符号不用改。
    于是这里只做三件事：接近段自己把侧向拉到 `PARK_LATERAL_M`、把位姿/障碍物
    映射进规范坐标系、把控制量映射回来。

加速度分（元素内 |a| ≤ 5 m/s² 每元素 +10）：
    · 接近段只用松油 + 0.06 轻刹（|a|≈1.6），不出现基线那种 blk<7 m 的 0.35 硬刹；
    · 倒车段沿用控制器的 PWM 油门（占空比 ≤0.5、slew 0.0006）；
    · 入库完成后挂 P、刹车只到 0.55（元素 8 实测满刹 1.00 在静止车上会抖 −5.16）。

`Command.park=True` 是给 driver 的信号：本元素自己管泊车，别在 `remain<3 m` 时
抢去 `EndPark`（本站车位中心就是路线末点，倒车入库必须先开过它 6 m）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from metacar import GearMode

from metacar_race import config as _G

from . import config as C
from .strategies import Command, ElementContext, Strategy

CAR_TYPE = 6                                #: `CAR(6)`：平台里车辆的障碍物类型号

try:                                        # main 上已验证的平行泊车控制器
    from metacar_race.control.parallel_parking import ParallelParkingController
    _PARK_OK = True
    _PARK_ERR = ""
except Exception as exc:  # noqa: BLE001
    ParallelParkingController = None        # type: ignore[assignment]
    _PARK_OK = False
    _PARK_ERR = repr(exc)


# ---------------------------------------------------------------------------
# 障碍物/位姿适配：ctx.obstacles 是 ObstacleScan 的 dict（id/type/x/y/len/wid/ori…），
# 而控制器要的是带 `.type.value` / `.type.name` / `.pos_x` 的对象。
# ---------------------------------------------------------------------------
def _field(obj, key: str, default: float = 0.0) -> float:
    val = obj.get(key) if isinstance(obj, dict) else getattr(obj, key, None)
    try:
        return float(val)
    except (TypeError, ValueError):
        return float(default)


def _otype(obj) -> int:
    val = obj.get("type") if isinstance(obj, dict) else getattr(obj, "type", None)
    if isinstance(val, (int, float)):
        return int(val)
    try:
        return int(getattr(val, "value", -1))
    except (TypeError, ValueError):
        return -1


@dataclass
class _SlotType:
    value: int
    name: str


@dataclass
class _SlotObs:
    """控制器眼里的障碍物（只用到这六个字段）。"""

    type: _SlotType
    pos_x: float
    pos_y: float
    ori_z: float
    length: float
    width: float


@dataclass
class _Pose:
    pos_x: float
    pos_y: float
    ori_z: float


def wrap180(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


class ParkStrategy(Strategy):
    """元素 12：接近段（真实坐标）→ staging → 交给旋转后的规范坐标系控制器。"""

    key = "park"
    label = "平行车道泊车"

    def __init__(self, ctx: ElementContext, route, log=None) -> None:
        super().__init__(ctx, route, log)
        self.controller = None
        self.slot = None                  #: (cx, cy, sori, slen, swid)，真实坐标
        self.handed = False
        self.thr = 0.0
        self.phase_log = ""
        self.warned = ""

    # ------------------------------------------------------------------ 工具
    def _say(self, msg: str) -> None:
        print(f"[park] {msg}", flush=True)

    def _parse(self, ctx: ElementContext) -> None:
        if self.slot is not None:
            return
        for obj in ctx.obstacles or []:
            if _otype(obj) == int(C.PARKING_SLOT_TYPE):
                self.slot = (_field(obj, "x"), _field(obj, "y"), _field(obj, "ori"),
                             _field(obj, "len"), _field(obj, "wid"))
                self._say(f"识别车位：中心=({self.slot[0]:.2f}, {self.slot[1]:.2f}) "
                          f"朝向={self.slot[2]:.1f}° 尺寸={self.slot[3]:.1f}×{self.slot[4]:.1f}")
                return

    def _canonical(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        """绕车位中心旋 180°：位姿映射到 main 单场景的坐标系。"""
        cx, cy = self.slot[0], self.slot[1]
        return (2.0 * cx - x, 2.0 * cy - y, wrap180(yaw + 180.0))

    def _obs_shims(self, ctx: ElementContext) -> list[_SlotObs]:
        out: list[_SlotObs] = []
        cx, cy = self.slot[0], self.slot[1]
        for obj in ctx.obstacles or []:
            ot = _otype(obj)
            if ot == int(C.PARKING_SLOT_TYPE):
                kind = _SlotType(ot, "SLOT")
            elif ot == CAR_TYPE:                   # 平台里车辆的类型号，控制器认 name=="CAR"
                kind = _SlotType(ot, "CAR")
            else:
                continue
            out.append(_SlotObs(kind,
                                2.0 * cx - _field(obj, "x"),
                                2.0 * cy - _field(obj, "y"),
                                _field(obj, "ori") + 180.0,
                                _field(obj, "len"),
                                _field(obj, "wid")))
        return out

    def _far_car_y(self, ctx: ElementContext) -> float:
        """规范坐标系里「远端那辆车」的 y（车位中心另一侧、y 更小的那辆）。"""
        cx, cy = self.slot[0], self.slot[1]
        best = None
        for obj in ctx.obstacles or []:
            if _otype(obj) != CAR_TYPE:
                continue
            ox, oy = _field(obj, "x"), _field(obj, "y")
            if abs(ox - cx) > 1.5:
                continue
            yc = 2.0 * cy - oy                 # 旋 180° 后的 y
            if yc >= cy:
                continue                       # 只要车位 y 更小那一侧（拓扑上的「远端」）
            if best is None or yc > best:
                best = yc                      # 取离车位最近的那辆
        if best is None:
            return cy - self.slot[3] / 2.0 - _G.PARALLEL_CAR_LEN / 2.0 - 1.0
        return best

    def _staging(self, ctx: ElementContext) -> tuple[float, float]:
        """规范坐标系里的 staging 点 (x, y)。

        main 的单场景是「开到远端车旁再倒」，但它的车位两侧车距、车道宽与本机连续 4
        略有差别：直接照抄 `远端车 + 0.15` 会让两段圆弧多走 1.5 m，落点越过车位中心
        （离线标定：L=3.00 时过 1.56 m），全靠 FIT 慢慢蹭回来。
        这里按几何把 staging 反推出来——两段等半径圆弧沿车位轴的位移是
        `Δ = 2R·sinθ`（`cosθ = 1 − L/2R`），所以 staging 应停在车位中心沿行进方向
        前方 Δ 处，圆弧走完正好落在中心。
        """
        cx, cy = self.slot[0], self.slot[1]
        radius = _G.PARALLEL_WHEELBASE / math.tan(
            _G.PARALLEL_MAX_STEER_RAD * _G.PARALLEL_STEER)
        cos_t = max(-0.99, min(0.99, 1.0 - C.PARK_LATERAL_M / (2.0 * radius)))
        theta = math.acos(cos_t)
        advance = 2.0 * radius * math.sin(theta)
        return cx + C.PARK_LATERAL_M, cy - advance

    # ------------------------------------------------------------ 阶段一：接近
    def _approach(self, ctx: ElementContext, x: float, y: float, yaw: float) -> Command:
        cx, cy = self.slot[0], self.slot[1]
        xp, yp, yawp = self._canonical(x, y, yaw)
        target_x, target_y = self._staging(ctx)
        far_y = self._far_car_y(ctx)
        ex = xp - target_x
        rem = yp - target_y                    # 规范系里行进方向是 −y，所以 >0 表示还没到

        bias = max(-C.PARK_APPROACH_YAW_MAX_DEG,
                   min(C.PARK_APPROACH_YAW_MAX_DEG,
                       ex * C.PARK_APPROACH_GAIN_DEG_PER_M))
        desired = 90.0 + bias
        steer = max(-C.PARK_APPROACH_STEER_MAX,
                    min(C.PARK_APPROACH_STEER_MAX,
                        C.PARK_APPROACH_STEER_GAIN * wrap180(desired - yawp)))

        v_des = min(C.PARK_APPROACH_MS,
                    math.sqrt(max(2.0 * C.PARK_APPROACH_DECEL_MS2 * (rem - 0.8), 0.0)))
        if abs(ex) > 1.0:
            v_des = min(v_des, 3.0)

        brk = 0.0
        if rem <= C.PARK_HANDOVER_REM_M and abs(ctx.speed) <= C.PARK_HANDOVER_MS:
            self.handed = True
            self.controller = ParallelParkingController()
            shims = self._obs_shims(ctx)
            # 先把它的 geom 建出来，再把 stage_y 改成我们几何反推的 staging——
            # 否则它的 APPROACH 会自己开回「远端车 + 0.15」，白跑一段。
            self.controller._parse_scene(shims, xp)          # noqa: SLF001
            if self.controller.geom:
                self.controller.geom["stage_y"] = target_y   # noqa: SLF001
            self._say(f"到达 staging：x={xp:.2f}（目标 {target_x:.2f}）、"
                      f"y={yp:.2f}（目标 {target_y:.2f}，远端车 {far_y:.2f}）、"
                      f"侧向误差 {ex:+.2f} m、v={ctx.speed:.2f} → 交给 ParallelParkingController")
            return self._step_controller(ctx, x, y, yaw)

        if ctx.speed > v_des + 0.25:
            self.thr = max(0.0, self.thr - 2.0 * ctx.dt)
            brk = C.PARK_APPROACH_BRAKE
        elif abs(ctx.speed) < 0.12 and rem > C.PARK_HANDOVER_REM_M:
            # 死区：速度掉到 0、但离 staging 还差几十厘米时，速度环的 ±0.25 容差
            # 会让油门一路衰减到 0（elem89 前的基线就是这么停早的）。这里显式给
            # 0.10 的蠕行油门，把它蹭到位。
            self.thr = min(0.20, max(0.10, self.thr + C.PARK_APPROACH_THR_RAMP_PER_S * ctx.dt))
        elif ctx.speed < v_des - 0.25:
            self.thr = min(C.PARK_APPROACH_THR_MAX,
                           self.thr + C.PARK_APPROACH_THR_RAMP_PER_S * ctx.dt)
        else:
            self.thr = max(0.0, self.thr - 0.5 * ctx.dt)

        self._log_phase("APPROACH",
                        f"rem={rem:6.1f} ex={ex:+.2f} v={ctx.speed:4.2f} "
                        f"v_des={v_des:4.2f} thr={self.thr:.2f} brk={brk:.2f}")
        return Command(steer=float(steer), throttle=float(self.thr), brake=float(brk),
                       gear=GearMode.DRIVE, park=True,
                       note=f"park approach rem={rem:.1f} ex={ex:+.2f}")

    # ------------------------------------------------------- 阶段二：控制器
    def _step_controller(self, ctx: ElementContext, x: float, y: float, yaw: float) -> Command:
        xp, yp, yawp = self._canonical(x, y, yaw)
        vc, info = self.controller.update(_Pose(xp, yp, yawp), float(ctx.speed),
                                          self._obs_shims(ctx), float(ctx.dt))
        self._log_phase(str(info.phase),
                        f"dx={info.dx:+.2f} dy={info.dy:+.2f} "
                        f"yaw_err={info.yaw_error:5.1f}° θ={info.theta:4.1f}° "
                        f"v={ctx.speed:5.2f} steer={float(vc.steering):+.2f}")
        if self.controller.done():
            self._say(f"入库完成：dx={info.dx:+.2f} dy={info.dy:+.2f} "
                      f"航向误差={info.yaw_error:.1f}° → 挂 P 保持")
            return Command(steer=0.0, throttle=0.0, brake=C.PARK_HOLD_BRAKE,
                           gear=GearMode.PARKING, park=True, note="park done")
        return Command(steer=float(vc.steering), throttle=float(vc.throttle),
                       brake=float(vc.brake), gear=vc.gear, park=True,
                       note=f"park {info.phase}")

    def _log_phase(self, phase: str, detail: str) -> None:
        if phase != self.phase_log:
            self.phase_log = phase
            self._say(f"相位 {phase}：{detail}")

    # --------------------------------------------------------------- 主入口
    def step(self, ctx: ElementContext) -> Command | None:
        if not _PARK_OK:
            if self.warned != "import":
                self.warned = "import"
                self._say(f"parallel_parking 不可用（{_PARK_ERR}），退回基线")
            return None
        self._parse(ctx)
        if self.slot is None:
            if self.warned != "slot":
                self.warned = "slot"
                self._say("没找到车位（type=30），本元素退回基线")
            return None
        if self.handed and self.controller is not None:
            return self._step_controller(ctx, float(ctx.x), float(ctx.y), float(ctx.yaw))
        return self._approach(ctx, float(ctx.x), float(ctx.y), float(ctx.yaw))

    def reset(self) -> None:
        self.controller = None
        self.handed = False
        self.thr = 0.0
        self.phase_log = ""


__all__ = ["ParkStrategy", "wrap180"]

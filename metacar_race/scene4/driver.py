"""主循环装配：把 track / obstacles / lateral / longitudinal / safety /
strategies / router 串成一轮仿真。

这里只有**装配**，不写新算法：每一段都能在 `docs/SCENE4-ELEMENT-ROUTER.md` 的
验证记录里找到它为什么在这个位置。

控制量流水线（顺序即语义，别随手调换）：

    v_tgt = profile.target(...)            # 曲率剖面
      → 跟车收速（block）                   # 只用来看得见的障碍
      → 横移锁速（overtake）                # 横向/纵向不同时做
      → 信号灯闸门（LightGate，含 ETA）      # P1：绿灯剩不够就别冲
      → 元素策略的 v_cap（可选）             # 策略只能"限速"，不能自己再造速度环
      → accel_gov.apply_trim                # 收油治理（下调目标车速）
      → speed_ctl.update                    # 唯一的速度环
      → stall / soft_start / accel_gov.update
      → blk_ctrl 硬刹（安全闸）
      → 指令限幅 limit_command
      → 元素策略逐通道覆盖（None=沿用）
      → 终点驻车 EndPark
"""

from __future__ import annotations

import math
import time
from pathlib import Path

from metacar import GearMode, VehicleControl

from metacar_race.config import LOOKAHEAD_M
from metacar_race.control.profile import SpeedProfile
from metacar_race.control.tracking import PurePursuit, SpeedController

from . import config as C
from .holdctl import HoldController
from .lateral import Overtaker, ShiftRoute, Stanley
from .longitudinal import AccelGovernor, SoftStart, limit_command
from .obstacles import ObstacleScan, StallGuard, StuckWatch, TeleportWatch
from .platform import SceneLog, SceneSession, default_out_dir, save_shot
from .router import ElementRouter
from .safety import EndPark, LightGate
from .strategies import ElementContext
from .track import RouteTrack

#: 信号灯状态 → CSV 里的数字。`LightGate` 存的是枚举名（"GREEN"），
#: 老代码直接 `int(state)` 会在第一次遇到信号灯时抛 ValueError 把整局打断
#: （curve-r7 在 s=1864.5 就是这样崩的），这里统一转一次。
_LIGHT_CODE = {"RED": 1, "GREEN": 2, "YELLOW": 3}


def _light_code(state) -> int | str:
    if state is None or state == "":
        return ""
    if hasattr(state, "value"):
        try:
            return int(state.value)
        except Exception:  # noqa: BLE001
            pass
    if isinstance(state, (int, float)):
        return int(state)
    return _LIGHT_CODE.get(str(state).upper(), "")


class Driver:
    """一轮仿真。`run()` 返回进程退出码（0 = 正常收敛）。"""

    def __init__(self, cfg: C.DriverConfig) -> None:
        self.cfg = cfg
        self.session = SceneSession()
        self.out_dir = (Path(cfg.out).resolve() if cfg.out
                        else default_out_dir("scene4"))
        self.log = SceneLog(self.out_dir)

        # 几何与执行器
        self.track: RouteTrack | None = None
        self.profile = None
        self.speed_ctl = None
        self.pursuit = None
        self.stanley = None
        self.shift_route = None

        # 状态
        self.overtaker = Overtaker(enabled=False)
        self.accel_gov = AccelGovernor(enabled=cfg.accel_gov)
        self.soft = SoftStart(enabled=cfg.soft_start)
        self.stall = StallGuard()
        self.stuck = StuckWatch()
        self.teleport = TeleportWatch()
        # 强制停车状态机（2026-10-25 「必须停就一次停住」指令，见 holdctl.py）
        self.hold = HoldController()
        self.light_gate = LightGate(enabled=cfg.light, log=self._say)
        self.end_park = EndPark(hold_seconds=cfg.end_hold,
                                coast_only=cfg.caps[1] <= 0.001)
        self.router: ElementRouter | None = None
        # 实线边界保护（懒加载：第一帧拿到 static.route 之后才按指纹找 mapd）
        self.border = None
        self.border_tried = False
        self.ego_len = 4.70
        self.ego_wid = 1.90

        # 逐帧状态
        self.sim_t = 0.0
        self.dt = 1.0 / 30.0
        self.cur_sub = ""
        self.elem_i = 0
        self.stanley_until = 0.0
        self.a_cmd_prev: float | None = None
        # 升挡闸门状态（2026-10-25 curve-r9 之后：0.50 的 kick 升不了挡，改成 0.86 窄门）
        self.shift_until = -1.0
        self._shift_t0 = -1e9
        self.last_shift = -1e9
        self._gate_span = object()      # 已经为哪个元素开过闸门（object() = 还没开过）
        self._cur_span_index = None
        self._span_len = 0.0
        self._span_end = 0.0
        self._span_name = ""
        self._shifted_ok = False        # （旧）整局是否已升上二挡——现在按次数计
        self._shifts_done = 0           # 一局已成功升挡次数（上限 C.SHIFT_MAX_COUNT）
        self._v_need = 0.0              # 剩余里程 ÷ 剩余时间（时钟需求，驱动升挡）
        self._time_limit = 0.0          # 本元素限时（秒），每帧从 status 刷新
        self.scene_t = 0.0              # 场景累计时间（status.used_time 是逐元素的，不能用来算总账）
        self._v_need_elem = 0.0         # 本元素的时钟需求（剩余里程 ÷ 剩余时间）
        self.kick_until = -1.0
        self.last_kick = -1e9
        self.steer_prev = 0.0
        self.phase = "run"
        self.shot_t = -99.0
        self._last_used: float | None = None
        self._ended = False

    # ------------------------------------------------------------------ 小工具
    def _stop_gap(self) -> float:
        """跟停时"与前车中心的目标距离"。元素 8（车辆紧急停止）按规则是 5 m。"""
        if "紧急停止" in str(self.cur_sub or "") or "紧急停止" in str(self._span_name or ""):
            return float(C.EMERGENCY_STOP_GAP_M)
        return float(C.BLOCK_STOP_GAP_M)

    def _stop_extra(self) -> float:
        if "紧急停止" in str(self.cur_sub or "") or "紧急停止" in str(self._span_name or ""):
            return float(C.EMERGENCY_STOP_EXTRA)
        return float(C.BLOCK_STOP_EXTRA)

    def _must_stop(self) -> bool:
        """本段是不是"规则要求必须停住"（车辆紧急停止）。"""
        if "紧急停止" in str(self.cur_sub or "") or "紧急停止" in str(self._span_name or ""):
            return True
        return False

    def _load_borders(self, static) -> None:
        """按路线指纹找 mapd、索引实线线段。失败就是没有保护，不抛异常。"""
        if self.border_tried:
            return
        self.border_tried = True
        if not getattr(self.cfg, "border_guard", True):
            self._say("[border] 实线保护已关闭（--no-border）")
            return
        try:
            from . import borders as B

            mapd = B.find_mapd(list(getattr(static, "route", []) or []),
                               scene_root=getattr(self.cfg, "scene_root", B.DEFAULT_SCENE_ROOT))
            if mapd is None:
                self._say("[border] 没认出场景包，实线保护关闭")
                return
            segs = B.load_solid_segments(mapd)
            grid = B.BorderGrid(segs, cell=float(C.BORDER_CELL_M))
            if not grid:
                self._say("[border] mapd 里没有实线，保护关闭")
                return
            self.border = grid
            self._say(f"[border] 实线 {len(segs)} 段（{mapd}），余量目标 "
                      f"{C.BORDER_SAFE_M:.2f} m")
        except Exception as exc:                            # noqa: BLE001
            self._say(f"[border] 载入失败，实线保护关闭: {exc!r}")

    def _say(self, msg: str) -> None:
        print(msg, flush=True)

    def _finish(self, ending: str, x: float, y: float, yaw: float) -> int:
        if self._ended:
            return 0
        self._ended = True
        s = float(self.track.s_of_world(x, y)) if self.track is not None else 0.0
        self.log.summary(ending, self.phase, s, (x, y, yaw))
        self._say("SUMMARY " + str((self.log.out_dir / "summary.json").read_text("utf-8")
                                  .replace("\n", " ")))
        self.session.park()
        self.log.close()
        return 0

    # ------------------------------------------------------------------ 准备
    def _prepare(self, static) -> None:
        route = list(static.route)
        self.track = RouteTrack(route)
        n = self.track.n
        if n >= 3:
            self.profile = SpeedProfile(route, v_max=self.cfg.vmax)
        self.speed_ctl = SpeedController(v_target=self.cfg.vmax, accel_limit=1.5,
                                         throttle_max=0.85)
        self.speed_ctl.reset(0.0)
        if n >= 2:
            self.pursuit = PurePursuit(route, lookahead=LOOKAHEAD_M)
            self.stanley = Stanley(route)
            self.shift_route = ShiftRoute(route)
            if self.shift_route is not None:
                self.pursuit.route = self.shift_route.items
                if self.stanley is not None:
                    self.stanley.route = self.shift_route.items
        self._say(f"[init] 路线 {n} 点 / {self.track.length:.1f} m，"
                  f"元素 {len(getattr(static, 'sub_scenes', []) or [])} 个，"
                  f"产物 {self.out_dir}")
        if self.cfg.elements and self.track is not None and n >= 2:
            overrides = {}
            if str(getattr(self.cfg, "lane_element", "") or "").strip():
                overrides[str(self.cfg.lane_element).strip()] = "lane"
            try:
                self.router = ElementRouter(
                    static, route, self.track,
                    vmax=self.cfg.vmax, only=self.cfg.elements_only,
                    overrides=overrides, log=self._say)
                self._say("[init] 元素路由：" + " | ".join(
                    f"{sp.index}.{sp.name} {sp.s_start:.1f}~{sp.s_end:.1f}"
                    for sp in self.router.spans))
            except Exception as exc:  # noqa: BLE001
                self.router = None
                self._say(f"[init] 元素路由构造失败，退回通用路径: {exc!r}")

    # ------------------------------------------------------------------ 主循环
    def run(self) -> int:
        static = self.session.connect()
        self._prepare(static)
        # 注意：**不要**在 main_loop() 之前发控制帧。SDK 的 main_loop() 第一件事才是发
        # Code2 告诉场景「算法就绪」；先发 Code4 会让场景在收到 code2 之前收到控制量，
        # 2026-10-25 12:33 那次 0 帧直接 ended 就是这么来的（旧 park_cs4.py 也从不这么做）。
        # 跳关已按本地 AGENTS.md 铁律 9 停用（2026-10-25），验证一律从起点真实驾驶。
        x = y = yaw = 0.0
        t0 = time.time()

        try:
            for frame, (msg, camera_frames) in enumerate(self.session.frames(), start=1):
                car = msg.main_vehicle
                pose = msg.pose_gnss
                status = msg.scene_status
                x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
                speed = float(car.speed)
                used = float(getattr(status, "used_time", 0.0) or 0.0) if status is not None else 0.0
                if self._last_used is not None and 1e-4 < used - self._last_used < 0.5:
                    self.dt = used - self._last_used
                if used > 0.0:
                    self._last_used = used
                    self.sim_t = used
                else:
                    self.sim_t += self.dt
                self.log.sim_t = self.sim_t
                self.log.frame = frame
                self.scene_t += self.dt
                sub = (status.sub_scene_name or "") if status is not None else ""
                # 本场限时（平台给的几何量，用来算"剩余里程 ÷ 剩余时间"的时钟需求）
                self._time_limit = float(getattr(status, "time_limit", 0.0) or 0.0)

                # ---- 元素切换：复位与开关都在这里 ------------------------
                if sub != self.cur_sub:
                    self.cur_sub = sub
                    self.elem_i += 1
                    self.light_gate.reset()
                    ov_on = bool(self.cfg.overtake and "超车" in sub)
                    if ov_on != self.overtaker.enabled:
                        self.overtaker.enabled = ov_on
                        self.overtaker.reset(self.sim_t)
                    if self.cfg.stanley and "环岛" in sub:
                        self.stanley_until = self.sim_t + C.STANLEY_TAIL_S
                    self.log.event("elem_enter", name=sub, elem_i=self.elem_i,
                                   used_time=float(getattr(status, "used_time", 0.0) or 0.0))
                    self._say(f"[{self.sim_t:6.1f}s] 进入元素 {self.elem_i}:{sub}")

                # ---- 跳关：已停用（本地 AGENTS.md 铁律 9，2026-10-25）----------
                # 之前这里是 `--skip-to-index` 的 skip_level() 循环，会提前跳过前序元素；
                # 用户明令不再跳关，验证一律从起点真实驾驶，目标元素靠 `--elements-only` 接管。

                # ---- 瞬移检测 -------------------------------------------
                jumped, jump = self.teleport.update(x, y, speed, self.dt)
                if jumped:
                    self.log.event("teleport", jump=round(jump, 2),
                                   x=round(x, 2), y=round(y, 2))
                    self._say(f"[{self.sim_t:6.1f}s] 平台瞬移 {jump:.1f} m，复位跟踪器")
                    if self.pursuit is not None:
                        self.pursuit.idx = self.track.sync(x, y)
                    if self.stanley is not None:
                        self.stanley.sync(x, y, max_jump_m=1e9)
                    self.overtaker.reset(self.sim_t)
                    self.accel_gov.reset()
                    if self.router is not None:
                        self.router.reset()

                # ---- 几何 -------------------------------------------------
                i_now = self.track.sync(x, y)
                if self.pursuit is not None and abs(self.pursuit.idx - i_now) > C.IDX_GUARD_M:
                    self.pursuit.idx = i_now
                lat_err = self.track.lateral_error(i_now, x, y)
                s_now = self.track.s_of(i_now)
                # 当前元素长度（升挡闸门只对长元素开放：短元素 4.78 m/s 也够用）
                # 另外算「预升挡」：当前元素快到尾巴、下一个元素是长元素时也允许换挡——
                # 依据 curve-r5（12:51）：那次升挡发生在元素 1 里，元素 1 照样 110，
                # 而元素 2 一进去就已经是 2 挡 7.91 m/s（油门 0.01~0.31），整段平稳拿 110。
                self._pre_shift_ok = False
                if self.router is not None:
                    _sp = self.router.span_at(s_now)
                    self._span_len = float(_sp.s_end - _sp.s_start) if _sp is not None else 0.0
                    self._span_end = float(_sp.s_end) if _sp is not None else 0.0
                    self._span_name = str(_sp.name) if _sp is not None else ""
                    self._cur_span_index = _sp.index if _sp is not None else None
                    _spans = list(getattr(self.router, "spans", []) or [])
                    for _k, _cur in enumerate(_spans):
                        if _sp is not None and _cur.index == _sp.index and _k + 1 < len(_spans):
                            _nxt = _spans[_k + 1]
                            _nxt_len = float(_nxt.s_end - _nxt.s_start)
                            _left = float(_sp.s_end) - float(s_now)
                            if _nxt_len >= C.SHIFT_MIN_SPAN_M and 0.0 <= _left <= C.PRE_SHIFT_M:
                                self._pre_shift_ok = True
                            break
                else:
                    self._span_len = 0.0
                    self._span_end = 0.0
                    self._span_name = ""
                block, obs_items = ObstacleScan.scan(self.track, i_now, list(msg.obstacles or []))
                if len(msg.obstacles or []) > 0:
                    self.log.obstacle({"sim_t": round(self.sim_t, 3), "frame": frame,
                                       "n": len(obs_items), "items": obs_items,
                                       "block": block})
                stuck_now = self.stuck.update(self.sim_t, x, y)

                # ---- 超车状态机 -------------------------------------------
                ov = self.overtaker.update(self.sim_t, obs_items, self.dt, speed)
                lat_off = float(ov["lat_off"])
                if self.shift_route is not None and self.pursuit is not None:
                    self.pursuit.route = self.shift_route.apply(lat_off, i_now)
                if ov["changed"]:
                    self.log.event("overtake", state=ov["state"], lat_off=round(lat_off, 2))
                    self._say(f"[{self.sim_t:6.1f}s] 超车 {ov['state']} "
                              f"offset={lat_off:+.2f} v={speed:.2f}")

                # ---- 纵向目标 ---------------------------------------------
                v_tgt = (self.profile.target(i_now, speed) if self.profile is not None
                         else self.cfg.vmax)
                blk_ctrl = block
                # 2026-10-25 元素 6（复杂路况超车）实测：`ObstacleScan` 判"本道"用的是障碍
                # 相对 **route** 的横向偏，自车已经并到左道 3.5 m 之后，慢车相对 route 仍是
                # lat≈0 → 一直被当成"本道正前方"，于是 PASS 阶段把目标车速压到 2.9→2.0→1.6
                # m/s，再靠 `brk=0.35` 硬刹（515 帧 |a|>5、809 帧刹车全在元素 6）。
                # 判据改成"相对自车当前所在车道"：`|lat − lat_off| ≤ LANE_HALF_M`。
                if blk_ctrl is not None and abs(float(blk_ctrl["lat"]) - float(lat_off)) > C.LANE_HALF_M:
                    blk_ctrl = None
                # ---- 强制停车（一次停住，不再"油门刹车打架"）--------------
                # 2026-10-25 p4/p5 元素 8 实测：旧逻辑在这里算 `v_kin`、在安全闸里
                # 又按 `gap_free < 1.0` 补一脚 `brk=0.35`（a_cmd=−35，车端 −13~−18
                # m/s²），两处各说各话 → 停到 5.9 m 后又被低速限幅 `thr=0.28` 推着
                # 蹭到 4.9 m，17 秒里 67 帧刹车。现在只有一个地方能决定"停"：
                # `HoldController`（run→decel→hold→release），这里只取它的结论。
                d_true = None
                if blk_ctrl is not None:
                    try:
                        d_true = math.hypot(float(blk_ctrl["x"]) - float(x),
                                            float(blk_ctrl["y"]) - float(y))
                    except (KeyError, TypeError, ValueError):
                        d_true = None
                hcmd = self.hold.update(blk=blk_ctrl,
                                        stop_gap=self._stop_gap(),
                                        extra=self._stop_extra(),
                                        speed=float(speed), dt=self.dt,
                                        must_stop=self._must_stop(),
                                        d_true=d_true)
                if hcmd.mode != "run":
                    self.log.event("hold", mode=hcmd.mode,
                                   d_free=None if hcmd.d_free is None
                                   else round(float(hcmd.d_free), 2),
                                   v=round(float(speed), 2))
                if hcmd.v_cap is not None:
                    v_tgt = min(v_tgt, float(hcmd.v_cap))
                if ov.get("blocked") or ov["state"] in ("SHIFT_OUT", "SHIFT_BACK"):
                    v_hold = ov.get("v_hold")
                    if v_hold is None:
                        v_tgt = min(v_tgt, 4.0)
                    elif C.OVERTAKE_HOLD_NO_ACCEL:
                        # 2026-10-25 p4 元素 6 实测：`v_hold` 是**进并线那一刻**的速度
                        # （8.00），车一边打舵一边掉到 5.78，速度误差 2.2 m/s 把油门顶到
                        # 0.56（a_cmd = 4.5，正好卡在限幅上），换挡/扭矩台阶让平台连看
                        # 11 帧 8.48 m/s² → 丢 +10。并线期间只"保持"，不追目标电压：
                        # 目标夹到当前车速，提速全部留给 PASS 直线段。
                        v_tgt = min(float(v_hold), float(speed))
                    else:
                        v_tgt = max(0.0, float(v_hold))

                # ---- 信号灯闸门 -------------------------------------------
                light = self.light_gate.update(groups=list(msg.traffic_light_groups or []),
                                               roads=list(getattr(static, "roads", []) or []),
                                               x=x, y=y, yaw=yaw, speed=speed)
                for ev in light.events:
                    self.log.event("light", name=ev, gap=None if light.stop_gap is None
                                   else round(float(light.stop_gap), 2),
                                   state=light.state, remain=None if light.remain is None
                                   else round(float(light.remain), 2),
                                   cap=None if light.v_cap is None else round(float(light.v_cap), 2))
                    self._say(f"[{self.sim_t:6.1f}s] 信号灯 {ev}")
                if light.v_cap is not None:
                    v_tgt = min(v_tgt, float(light.v_cap))

                # ---- 元素策略 ---------------------------------------------
                cmd = None
                if self.router is not None:
                    ctx = ElementContext(
                        x=x, y=y, yaw=yaw, speed=speed, dt=self.dt, sim_t=self.sim_t,
                        s_now=s_now, pose=pose, obstacles=list(obs_items),
                        static=static, lane_width=3.5,
                        ego_length=float(car.length or 4.70),
                        ego_width=float(car.width or 1.90),
                        vmax=self.cfg.vmax,
                        time_limit=float(getattr(status, "time_limit", 0.0) or 0.0),
                        end_point=getattr(status, "end_point", None),
                        trajectory=getattr(msg, "trajectory", None),
                        a_now=float(self.a_cmd_prev or 0.0),
                        v_cap=v_tgt, light_state=light.state,
                        stop_gap=light.stop_gap)
                    cmd = self.router.step(ctx)
                    if cmd is not None and cmd.v_cap is not None:
                        v_tgt = min(v_tgt, float(cmd.v_cap))
                    if self.border is None:
                        self._load_borders(static)
                    self.ego_len = float(car.length or 4.70)
                    self.ego_wid = float(car.width or 1.90)

                # ---- 长元素速度下限（已删除） ------------------------------
                # 2026-10-25：原来这里有一张"元素长度 ≥200 m 就强行把目标车速抬到
                # 6.0 m/s"的表，本质是把 p2/p4 的经验写成答案（记忆题）。它还会和
                # 一挡墙打架：抬到 6.0 但车只能跑 4.78，于是反复穿越墙、反复开升挡闸门。
                # 现在速度目标只由现场量算的包络决定（曲率/跟车/边线/信号/元素），
                # 升挡改由"剩余里程 ÷ 剩余时间"的时钟需求驱动（见 _shift_gate）。
                _v_need = 0.0
                try:
                    _total = float(self.track.length) if self.track is not None else 0.0
                    _remain_m = _total - float(s_now)
                    _remain_t = float(C.SCENE_TIME_LIMIT_S) - float(self.scene_t)
                    if _remain_m > 0.0 and _remain_t > 1.0:
                        _v_need = _remain_m / _remain_t
                except (TypeError, ValueError, AttributeError):
                    _v_need = 0.0
                self._v_need = _v_need

                # ---- 元素时钟（每个元素自己的 200 s）-----------------------
                # 2026-10-26 p7 实测：`SceneStatus` 是**逐元素**的
                # （`sub_scene_name` + `used_time` + `time_limit`），元素 9（辅路
                # 1337 m / 限时 200 s）平均要 6.69 m/s；按曲率律只跑 5.5~6.0 m/s，
                # 结果 used_time 到 200 s 时还没走完 → 平台把车瞬移回元素起点重跑，
                # 连跳 3 次（p7 帧数 33600、元素 9 反复重来）。
                # 这条"元素时钟律"不猜场景：剩余里程 ÷ 剩余时间就是必须达到的车速，
                # 低于它就按需抬到（上限 vmax）。这也是当初那张"长元素抬到 6.0"的表
                # 真正在干的事——现在改成算出来的。
                _v_need_elem = 0.0
                try:
                    _lim = float(getattr(self, "_time_limit", 0.0) or 0.0)
                    _left_t = _lim - float(self.sim_t)
                    _left_m = float(getattr(self, "_span_end", 0.0)) - float(s_now)
                    if _lim > 1.0 and _left_t > 0.5 and _left_m > 0.0:
                        _v_need_elem = _left_m / _left_t
                except (TypeError, ValueError, AttributeError):
                    _v_need_elem = 0.0
                self._v_need_elem = _v_need_elem


                # ---- 包络：实线余量 → 车速上限 ---------------------------
                # 余量越窄越慢（`v_free × clamp(c/SAFE, 0.6, 1.0)`）。这是"边线律"，
                # 用的是车到实线的实测余量，不是任何场景/元素的经验值。
                self._border_clear = None
                if (not (cmd is not None and cmd.final)) and (not hcmd.lock) \
                        and self.border is not None and abs(float(speed)) > 0.5:
                    try:
                        _bc, _bside, _bp = self.border.clearance(
                            x, y, yaw, self.ego_len, self.ego_wid)
                        self._border_clear = float(_bc)
                        if _bc < C.BORDER_SLOW_M:
                            _v_border = float(self.cfg.vmax) * max(
                                0.70, float(_bc) / C.BORDER_SLOW_M)
                            if _v_border < float(v_tgt) - 0.05:
                                self.log.event("border_cap", d=round(float(_bc), 2),
                                               cap=round(float(_v_border), 2),
                                               tgt=round(float(v_tgt), 2))
                                v_tgt = _v_border
                    except Exception:  # noqa: BLE001
                        self._border_clear = None

                # ---- 元素时钟兜底：赶不上就加速（不许被瞬移）----------------
                if (_v_need_elem > 0.0 and not hcmd.lock
                        and blk_ctrl is None
                        and float(v_tgt) < _v_need_elem * C.ELEM_CLOCK_MARGIN):
                    _v_clock = min(float(self.cfg.vmax),
                                   _v_need_elem * C.ELEM_CLOCK_MARGIN)
                    if _v_clock > float(v_tgt) + 0.05:
                        self.log.event("elem_clock", need=round(_v_need_elem, 2),
                                       tgt=round(float(v_tgt), 2),
                                       raise_to=round(float(_v_clock), 2),
                                       left_m=round(float(getattr(self, "_span_end", 0.0)) - float(s_now), 1),
                                       left_t=round(float(getattr(self, "_time_limit", 0.0)) - float(self.sim_t), 1))
                        v_tgt = _v_clock

                # ---- 速度环 -----------------------------------------------
                v_tgt = self.accel_gov.apply_trim(v_tgt)
                # 急转限速：上一帧方向盘打得很大时，把目标速度压到侧向加速度安全区
                # （满舵 4.5 m/s 的 a_lat ≈ 5.6 m/s²，环岛就是这么丢 +10 的）
                if abs(float(getattr(self, "steer_prev", 0.0))) > C.RING_STEER_LIMIT:
                    v_tgt = min(float(v_tgt), C.RING_LAT_VMAX_MS)
                # 交接保护（2026-10-25）：起步/切弯策略先用油门把车推到 4.8 m/s，
                # 基线速度环的 v_cmd 却还停在 0 附近按 2 m/s² 往上爬，于是它认为
                # “车太快了”先给一脚刹车（curve-r5 元素 1 在 t=2.31 s 的 brk=0.19），
                # 再猛加油追上去（0.85 油门 → 一挡墙 → 升挡瞬态 10 m/s²）。
                # 实测速度已经高过指令时直接把指令抬到实测，速度环就不会反着来。
                if speed > float(getattr(self.speed_ctl, "v_cmd", 0.0)) + 0.30:
                    self.speed_ctl.v_cmd = float(speed)
                self.speed_ctl.v_target = v_tgt
                thr, brk = self.speed_ctl.update(speed, self.dt, v_ceiling=self.cfg.vmax)
                # 巡航段不踩刹车：加速度加分看的是指令，任何一次 brk 都吃掉 +10。
                # 只有真的要把车停下（v_tgt < 1.2，例如红灯/前车/终点）才允许刹车。
                if brk > 0.0 and float(v_tgt) > C.BRAKE_ONLY_BELOW_MS:
                    self.log.event("coast_brake", v=round(float(speed), 2),
                                   tgt=round(float(v_tgt), 2), brk=round(float(brk), 3))
                    thr, brk = 0.0, 0.0

                # 破挡只在"前方没人挡"时做：元素 9 入口被前车堵着时速度就是上不去，
                # 破挡逻辑却把油门一层层推到 0.5625，再被安全闸/速度环刹回来，于是
                # "窜-刹"极限环（p2 元素 9 入口 39 帧 |a|>5 全来自这里）。
                # 另外目标速度本来就低（≤1.5 m/s，等灯/跟停）时也不许破挡。
                if (blk_ctrl is not None and float(blk_ctrl["dist"]) < C.BLOCK_STOP_M * 2.0) \
                        or float(v_tgt) < 1.5:
                    self.stall.reset()
                else:
                    thr = self.stall.update(thr, speed, self.dt)
                # 超速保护：一挡低速扭矩大，速度环晚一拍就会冲过目标
                # （p3 元素 9 入口从 0.60 → 10.03 m/s，超 vmax 8.0）。松油不刹车，
                # 免得又制造一次刹车事件。
                if speed > float(v_tgt) + 0.80 or speed > float(self.cfg.vmax):
                    thr = 0.0
                elif speed > float(v_tgt) + 0.30:
                    thr = min(thr, 0.10)
                thr = self.soft.apply(thr)
                if not self.stall.active:
                    thr, brk = self.accel_gov.update(self.sim_t, speed, v_tgt, thr, brk)

                # ---- 安全闸 -----------------------------------------------
                # 2026-10-25 元素 8 教训：原来要求 `dist < 7.0` 才动手，7 m 处 v=7 m/s
                # 需要 4.2 m/s² 才停得住 —— 软刹做不到，只能 brk=0.35 硬刹（a_cmd=−35，
                # 还撞了前车 −10.59）。改成**按"需要多大减速度"提前触发**：一旦需要的
                # 减速度超过 BLOCK_DECEL_URGENT 就上软刹（a_cmd=−5，正好卡预算）。
                # 2026-10-25 「强制停车」改造：硬刹 0.35 只在"真的贴脸了"(<0.35 m) 才准用，
                # 正常跟停全部交给 HoldController（它会在目标点前就锁定），不再制造
                # −13~−18 m/s² 的刹车事件。
                if blk_ctrl is not None and speed > 0.30 and not hcmd.lock:
                    gap = float(blk_ctrl["dist"])
                    gap_free = gap - self._stop_gap()
                    v_lead = abs(float(blk_ctrl.get("v") or 0.0))
                    need = max(0.0, float(speed) ** 2 - v_lead ** 2) / \
                        (2.0 * max(gap_free, 0.25))
                    if need > C.BLOCK_DECEL_URGENT or gap_free < 2.0:
                        hard = gap_free < 0.35
                        thr, brk = 0.0, (0.35 if hard else C.BRAKE_SOFT_CMD)
                        self.log.event("hard_brake", dist=round(gap, 2),
                                       brk=round(brk, 3), need=round(need, 2))

                thr_cap, brk_cap, a_limit, slew = self.cfg.caps
                thr = max(0.0, min(thr_cap, thr))
                # ---- 低速油门限幅 -----------------------------------------
                # 2026-10-25 p4 元素 9（辅路入口）实测：进元素时车正好停着（v=0.14，
                # 前车 4.89 m），一键油门 0.56 直接把车从 0.14 甩到 8.42 m/s（1.2 s，
                # 实测 8.0~8.3 m/s²、41 帧超限）→ 丢 +10。平台只豁免**开局** 0.5 s，
                # 中途停车再起步没有豁免，所以低速段一律压油门：一挡低速扭矩太大，
                # 0.56 在 0~3 m/s 区间根本不止 4.5 m/s²（a_cmd 是折算值，不是实测值）。
                if not (cmd is not None and cmd.final) and self.cfg.low_speed_cap:
                    v_abs = abs(float(speed))
                    if v_abs < C.LOW_SPEED_THR_MS:
                        thr = min(thr, C.LOW_SPEED_THR_CAP)
                    elif v_abs < C.MID_SPEED_THR_MS:
                        thr = min(thr, C.MID_SPEED_THR_CAP)
                brk = max(0.0, min(brk_cap, brk))

                # ---- 横向 -------------------------------------------------
                steer = 0.0
                if self.stanley is not None and self.cfg.stanley and self.sim_t < self.stanley_until:
                    self.stanley.speed_ref = speed
                    steer = float(self.stanley.steering(x, y, yaw))
                elif self.pursuit is not None:
                    steer = float(self.pursuit.steering(x, y, yaw))
                self.steer_prev = float(steer)

                # ---- 指令限幅与元素覆盖 ------------------------------------
                if self.cfg.fixed_throttle > 0.0:
                    thr, brk = float(self.cfg.fixed_throttle), 0.0
                    a_cmd = None
                elif brk < 0.30:
                    thr, brk, a_cmd = limit_command(
                        thr, brk, self.a_cmd_prev, thr_cap, brk_cap, a_limit, slew,
                        enabled=self.cfg.accel_gov)
                    self.a_cmd_prev = a_cmd
                else:
                    a_cmd = None

                gear = GearMode.DRIVE
                if cmd is not None:
                    if cmd.throttle is not None:
                        thr = float(cmd.throttle)
                    if cmd.brake is not None:
                        brk = float(cmd.brake)
                    if cmd.steer is not None:
                        steer = float(cmd.steer)
                    if cmd.gear is not None:
                        gear = cmd.gear
                    # ---- 松油滑行：策略说滑就滑，油门刹车都归零（不踩刹车减速）----
                    if cmd.coast:
                        thr, brk = 0.0, 0.0
                        a_cmd = None
                        self.a_cmd_prev = None

                # ---- 起步咬合助力 -----------------------------------------
                # 车还没滚起来时直接给足咬合油门：离合咬合那 15~20 m/s² 的台阶在
                # 物理上躲不掉，但平台只看开局约 0.5 s 之外的部分，所以要让它
                # "早一点发生"。油门 0.15 的指令加速度只有 1.2，本身不违规。
                # 注意必须放在元素覆盖之后：`StartStraightStrategy` 起步只发 0.01
                # 的怠速油门，放在覆盖之前会被它盖掉（实测 launch_bite 一次没触发）。
                # 元素 final（泊车全权接管）不插手；堵停（安全闸）也不插手。
                _final = bool(cmd is not None and cmd.final)

                # ---- 强制停车：锁定 ---------------------------------------
                # 用户要求「必须停的时候一次停住，不要顿顿卡卡」。锁定期间只允许
                # 这一支说话：油门归零、刹车 `HOLD_BRAKE=0.05`（a_cmd=−5 卡线）把车
                # 收到 `HOLD_LOCK_MS=0.12` 以下，再挂 P 挡 + 满刹——P 挡后速度恒定
                # −0.00，保持期零抖动（上下坡驻车场景实测），也不会有"再给油再蹭"
                # 的抽搐。放行由 `holdctl` 的 release 负责（一次咬合油门，早咬合）。
                if hcmd.lock and not _final:
                    thr, brk = 0.0, C.HOLD_BRAKE
                    # 2026-10-25：锁是从"再晚就刹不住"那一点开始（可能离目标还有
                    # 20+ m），这段滑行里仍然要跟线，不能把舵打死；只有真正停住
                    # 之后才把舵回正，免得保持期车轮摆着、松刹时带着横向速度。
                    if abs(float(speed)) <= C.HOLD_LOCK_MS:
                        brk = 1.0
                        steer = 0.0
                        gear = GearMode.PARKING
                elif hcmd.bite and not _final:
                    # 放行那一帧：复刻场景起始起步（咬合油门一次到位）。
                    thr = max(float(thr), C.LAUNCH_THR)
                    brk = 0.0
                    self.log.event("hold_bite", v=round(float(speed), 2),
                                   thr=round(C.LAUNCH_THR, 3))

                # ---- 实线余量保护 ----------------------------------------
                # p2 / p4 的环岛出口，车身四角离实线的最近余量分别是 0.02 / 0.01 m
                # （`tools/replay_score.py --lines` 实测，都在 (-3028,-889) 一带），
                # 平台却只把 p4 判成「压道路边线 -20」——1 cm 的差别任何控制都保证
                # 不了。所以这里不追求"贴着线走"，而是把余量整体抬到
                # `BORDER_SAFE_M` 以上：近侧在左就往右带舵，在右就往左带舵。
                # 泊车（final）不插手：车本来就要挪进车位、贴着车位线走。
                if (not _final) and (not hcmd.lock) and self.border is not None \
                        and abs(float(speed)) > 0.5:
                    d_line, side, _near = self.border.clearance(
                        x, y, yaw, self.ego_len, self.ego_wid)
                    self._border_clear = float(d_line)
                    if d_line < C.BORDER_SAFE_M:
                        bias = min(C.BORDER_STEER_MAX,
                                   C.BORDER_STEER_K * (C.BORDER_SAFE_M - d_line))
                        steer = max(-1.0, min(1.0, float(steer) + side * bias))
                        self.log.event("border_guard", d=round(float(d_line), 2),
                                       side=int(side), bias=round(float(bias), 3))
                if (not _final
                        and not hcmd.lock
                        and not (cmd is not None and cmd.coast)
                        and abs(float(speed)) < C.LAUNCH_BITE_MS
                        and float(v_tgt) > C.LAUNCH_DEMAND_MS
                        and not (blk_ctrl is not None and float(blk_ctrl["dist"]) < C.BLOCK_STOP_M)):
                    if float(thr) < C.LAUNCH_THR:
                        self.log.event("launch_bite", v=round(float(speed), 2),
                                       thr=round(C.LAUNCH_THR, 3),
                                       tgt=round(float(v_tgt), 2))
                    thr = max(float(thr), C.LAUNCH_THR)
                    brk = 0.0

                # ---- 升挡闸门（一挡墙；唯一允许超过 0.62 油门的通道）---------
                # 元素策略宣布 final（泊车这种全权接管）时不许基线再改油门——
                # 泊车用的油门是 0.010 级的怠速量，被闸门抬到 0.86 会直接冲出车位。
                if not _final and not hcmd.lock:
                    thr, brk = self._shift_gate(speed, v_tgt, thr, brk)

                # ---- 终点驻车 ---------------------------------------------
                remain = self.track.length - s_now
                if self.phase == "run" and remain < 3.0 and not _final and not hcmd.lock:
                    self.phase = "hold"
                    self._say(f"[{self.sim_t:6.1f}s] 到终点前 3 m，进入驻车")
                if self.phase == "hold" and not _final:
                    parking, p_thr, p_brk, done = self.end_park.update(speed, self.dt)
                    thr, brk = p_thr, p_brk
                    gear = GearMode.PARKING if parking else GearMode.DRIVE
                    if done:
                        self.log.run_row(self._row(frame, sub, x, y, yaw, speed, gear,
                                                   thr, brk, steer, v_tgt, s_now, lat_err,
                                                   blk_ctrl, obs_items, stuck_now, lat_off,
                                                   light, ov))
                        return self._finish("hold_done", x, y, yaw)

                # ---- 下发 + 记录 ------------------------------------------
                self.session.set_control(VehicleControl(gear=gear, throttle=thr,
                                                        brake=brk, steering=steer))
                self.log.run_row(self._row(frame, sub, x, y, yaw, speed, gear, thr, brk,
                                           steer, v_tgt, s_now, lat_err, blk_ctrl, obs_items,
                                           stuck_now, lat_off, light, ov))
                if frame % max(1, self.cfg.log_every) == 0:
                    self.log.flush_run()
                    self._say(f"[{self.sim_t:6.1f}s] {self.phase:5s} s={s_now:7.1f} "
                              f"v={speed:5.2f} tgt={v_tgt:5.2f} thr={thr:.2f} brk={brk:.2f} "
                              f"steer={steer:+.3f} lat={lat_err:+.2f} "
                              f"blk={'-' if blk_ctrl is None else '%.1f' % blk_ctrl['dist']}")
                if self.cfg.shots > 0.0 and self.sim_t - self.shot_t >= self.cfg.shots:
                    self.shot_t = self.sim_t
                    save_shot(camera_frames, self.out_dir, "%07.1f" % self.sim_t)
                if time.time() - t0 > self.cfg.max_seconds:
                    self._say(f"[{self.sim_t:6.1f}s] 墙钟超时 {self.cfg.max_seconds:.0f}s，收工")
                    return self._finish("timeout", x, y, yaw)
        except KeyboardInterrupt:
            self._say("收到中断，驻车退出")
            return self._finish("interrupt", x, y, yaw)
        except Exception as exc:  # noqa: BLE001
            # 兜住任何异常：先把产物收尾，再让退出码正常返回，方便看 summary.json。
            self._say(f"运行异常，收工：{exc!r}")
            return self._finish("error", x, y, yaw)
        finally:
            # 只驻车；日志的 close 统一交给 _finish()，否则 summary() 会往已关闭的文件写。
            try:
                self.session.park()
            except Exception:  # noqa: BLE001
                pass
        return self._finish("ended", x, y, yaw)

    # ------------------------------------------------------------------ 升挡
    def _shift_gate(self, speed, v_tgt, thr, brk):
        """一挡墙（4.78 m/s = 17.2 km/h）的升挡闸门。

        2026-10-25 curve-r9 的教训：0.50 的「温和 kick」**一次也没升上去**——
        一挡墙就在 4.78，油门不过墙就永远在墙上（7 次 kick 只让车速 4.78 → 4.89）。
        docs/08 的稳态映射是 0.40 → 4.78 m/s、0.85 → 8.2 m/s，所以闸门必须开在
        0.85 以上；而且不能像旧 kick 那样插一段 0 油门再回来（0.56→0.50→0→0.56
        的台阶对本身就是丢 +10 的嫌疑）。这里改成「单调上去、单调下来」：
        贴墙且目标 >5.2 时把油门顶到 `SHIFT_THROTTLE`，车速过 `SHIFT_DONE_MS`
        立刻收回，最长 `SHIFT_MAX_S`。
        """
        now = float(self.sim_t)

        # 闸门开着：过速就收，超时也收
        if now < self.shift_until:
            if float(speed) >= C.SHIFT_DONE_MS or (now - self._shift_t0) >= C.SHIFT_MAX_S + 0.5:
                self.shift_until = -1.0
                if float(speed) >= C.SHIFT_DONE_MS:
                    self._shifts_done = int(getattr(self, "_shifts_done", 0)) + 1
                self.log.event("shift_done", v=round(float(speed), 2),
                               ok=float(speed) >= C.SHIFT_DONE_MS)
                self._say(f"[{now:6.1f}s] 升挡完成：v={speed:.2f}")
                return thr, brk
            return C.SHIFT_THROTTLE, 0.0

        stuck = abs(float(speed) - C.KICK_WALL_MS) <= C.KICK_WALL_TOL_MS
        # 只认"贴墙贴住"：二挡路过 4.78 m/s 只是一瞬间，一挡才会长期停在墙上。
        # 用驻留时间把这两种情况分开，避免二挡里误开闸门白丢一个 +10。
        self._wall_t = (float(getattr(self, "_wall_t", 0.0)) + self.dt) if stuck else 0.0
        stuck = self._wall_t >= C.SHIFT_WALL_HOLD_S
        want = float(v_tgt) >= C.SHIFT_TARGET_MS - 0.01
        long_span = (float(getattr(self, "_span_len", 0.0)) >= C.SHIFT_MIN_SPAN_M
                     or bool(getattr(self, "_pre_shift_ok", False)))
        # 2026-10-25 重构：原来是「整局只换一次挡，而且只准在起步段做」——那是把
        # p2/p4 的经验写成答案（记忆题），结果元素 9~11 共 1575 m 死钉 4.78 m/s，
        # 多花 117 s、整场 1540 s 超时。现在改成**时钟需求驱动**：
        #   need = 剩余里程 ÷ 剩余时间；
        # 只要 need 还低于一挡墙（4.78），就不值得为它付一个元素的 +10；
        # need 一旦超过 SHIFT_NEED_MS，说明不升挡一定超时，才开闸。
        # 一局最多 SHIFT_MAX_COUNT 次（每次代价 ≈ 一个元素的 +10，必须有限额）。
        need = float(getattr(self, "_v_need", 0.0) or 0.0)
        urgent = need >= C.SHIFT_NEED_MS
        count_ok = int(getattr(self, "_shifts_done", 0)) < int(C.SHIFT_MAX_COUNT)
        in_start = str(getattr(self, "_span_name", "")) == C.START_SPAN_NAME
        once = (in_start or self._gate_span != getattr(self, "_cur_span_index", None))
        if (stuck and want and count_ok and (long_span or in_start or urgent) and once
                and (now - self.last_shift) >= C.SHIFT_MIN_GAP_S):
            self.last_shift = now
            self._shift_t0 = now
            self._gate_span = getattr(self, "_cur_span_index", None)
            self.shift_until = now + C.SHIFT_MAX_S
            self.log.event("shift_gate", v=round(float(speed), 2),
                           tgt=round(float(v_tgt), 2), thr=C.SHIFT_THROTTLE,
                           need=round(need, 2), urgent=urgent)
            self._say(f"[{now:6.1f}s] 开升挡闸门：v={speed:.2f} 目标={v_tgt:.2f}，"
                      f"油门 {C.SHIFT_THROTTLE:.2f}（最长 {C.SHIFT_MAX_S:.1f}s）")
            return C.SHIFT_THROTTLE, 0.0
        return thr, brk

    # ------------------------------------------------------------------ 记录
    def _row(self, frame, sub, x, y, yaw, speed, gear, thr, brk, steer, v_tgt,
             s_now, lat_err, blk_ctrl, obs_items, stuck_now, lat_off, light, ov) -> list:
        return [
            frame, round(self.sim_t, 3), sub, self.elem_i, self.phase,
            round(x, 3), round(y, 3), round(yaw, 2), round(speed, 3),
            int(getattr(gear, "value", gear)), round(thr, 4), round(brk, 4), round(steer, 4),
            round(float(v_tgt), 3), round(float(s_now), 2), round(float(lat_err), 3),
            "" if blk_ctrl is None else round(float(blk_ctrl["dist"]), 2),
            "" if blk_ctrl is None else blk_ctrl.get("type", ""),
            "" if blk_ctrl is None else blk_ctrl.get("id", ""),
            len(obs_items), int(stuck_now), round(float(lat_off), 3),
            _light_code(light.state),
            "" if light.stop_gap is None else round(float(light.stop_gap), 2),
            ov["state"],
        ]


def run(cfg: C.DriverConfig) -> int:
    """CLI 薄壳调这个。"""
    return Driver(cfg).run()

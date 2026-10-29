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

import time
from pathlib import Path

from metacar import GearMode, VehicleControl

from metacar_race.config import LOOKAHEAD_M
from metacar_race.control.profile import SpeedProfile
from metacar_race.control.tracking import PurePursuit, SpeedController

from . import config as C
from .lateral import Overtaker, ShiftRoute, Stanley
from .longitudinal import AccelGovernor, SoftStart, limit_command
from .obstacles import ObstacleScan, StallGuard, StuckWatch, TeleportWatch
from .platform import SceneLog, SceneSession, default_out_dir, save_shot
from .router import ElementRouter
from .safety import EndPark, LightGate
from .strategies import ElementContext, kind_of
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
        self.light_gate = LightGate(enabled=cfg.light, log=self._say, track=self.track,
                                    vmax=cfg.vmax)
        self.end_park = EndPark(hold_seconds=cfg.end_hold,
                                coast_only=cfg.caps[1] <= 0.001)
        self.router: ElementRouter | None = None

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
        self._shifted_ok = False        # 整局是否已成功升上二挡（m03278：只换一次）
        self.kick_until = -1.0
        self.last_kick = -1e9
        self.steer_prev = 0.0
        self._last_hold_say = -10.0
        self.phase = "run"
        self.shot_t = -99.0
        self._last_used: float | None = None
        self._ended = False

    # ------------------------------------------------------------------ 小工具
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
        # 信号灯闸门要按里程盯「前方最近的停止线」，track 在 __init__ 时还没有
        self.light_gate.track = self.track
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
                sub = (status.sub_scene_name or "") if status is not None else ""

                # ---- 元素切换：复位与开关都在这里 ------------------------
                if sub != self.cur_sub:
                    self.cur_sub = sub
                    self.elem_i += 1
                    self.light_gate.reset()
                    # 元素 6 由 `overtake` 策略接管时关掉基线超车机（两套并线逻辑会打架）
                    # 判据改成"按元素名"——router 在这一帧还没 step，用
                    # router.kind() 会拿到上一个元素，elem6-r2 就是这样让基线
                    # 超车机抢在策略前面把 v_hold 锁成 3.2 m/s 达 45 s。
                    ov_on = bool(self.cfg.overtake and "超车" in sub
                                 and kind_of(sub) != "overtake")
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
                block, obs_items = ObstacleScan.scan(self.track, i_now,
                                                     list(msg.obstacles or []),
                                                     lat_ref=lat_err)
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
                if blk_ctrl is not None:
                    d_eff = float(blk_ctrl["dist"]) - C.BLOCK_MARGIN_M
                    if d_eff < C.BLOCK_SLOW_M:
                        v_cap = max(0.0, C.FOLLOW_DECEL * (d_eff - 1.5))
                        v_tgt = min(v_tgt, v_cap)
                if ov.get("blocked") or ov["state"] in ("SHIFT_OUT", "SHIFT_BACK"):
                    v_hold = ov.get("v_hold")
                    v_tgt = max(0.0, float(v_hold)) if v_hold is not None else min(v_tgt, 4.0)

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
                        stop_gap=light.stop_gap,
                        platform_element=sub)
                    cmd = self.router.step(ctx)
                    if cmd is not None and cmd.v_cap is not None:
                        v_tgt = min(v_tgt, float(cmd.v_cap))

                # ---- 长元素速度下限（m03278） ------------------------------
                # 元素 2（连续转弯 1023 m / 限时 200 s）必须 ≥5.12 m/s 才不超时；
                # 取 6.0 是因为它落在二挡稳态区，低于 6.0 车就会在 4.3~4.8 的一挡墙
                # 上反复穿越，每穿一次都要重新升挡，而每次升挡都是 6~9 m/s² 的尖峰
                # （curve-r11 元素 2 因此被误开 4 次闸门、丢了 +10）。
                _tail_left = float(getattr(self, "_span_end", 0.0)) - float(s_now)
                _floor_ok = (
                    float(getattr(self, "_span_len", 0.0)) >= C.SHIFT_MIN_SPAN_M
                    and _tail_left > C.LONG_SPAN_FLOOR_TAIL_M
                    and (blk_ctrl is None or float(blk_ctrl["dist"]) > C.BLOCK_SLOW_M)
                    and not bool(ov.get("blocked"))
                    and (light.v_cap is None or float(light.v_cap) >= C.LONG_SPAN_MIN_MS)
                    # 环岛不吃这个下限（2026-10-25 r14）：它有曲率限速，硬抬到 6.0
                    # 会像 r14 那样 9.1 m/s 冲进入口 S 弯，侧向直接过 5。
                    and "环岛" not in str(getattr(self, "_span_name", ""))
                )
                if _floor_ok and float(v_tgt) < C.LONG_SPAN_MIN_MS:
                    if float(v_tgt) < C.LONG_SPAN_MIN_MS - 0.05:
                        self.log.event("span_floor", tgt=round(float(v_tgt), 2),
                                       floor=C.LONG_SPAN_MIN_MS,
                                       span=getattr(self, "_span_name", ""))
                    v_tgt = C.LONG_SPAN_MIN_MS

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
                # 例外：`light.waiting` 时是「决定在路口停住等下一次绿灯」，必须允许刹。
                if brk > 0.0 and float(v_tgt) > C.BRAKE_ONLY_BELOW_MS and not light.waiting:
                    self.log.event("coast_brake", v=round(float(speed), 2),
                                   tgt=round(float(v_tgt), 2), brk=round(float(brk), 3))
                    thr, brk = 0.0, 0.0

                # 等灯期间不补油：stall 的「卡死补油门」会和路口刹车互相打架
                # （curve-r19 线前 `thr=0.56` 与 `brk=0.35` 交替，速度在 ±0.7 m/s
                #   抖了 20 s，车还往前蹭了 6 m —— 用户 m04149 说的「刹车位置不对」）
                if not light.waiting:
                    thr = self.stall.update(thr, speed, self.dt)
                thr = self.soft.apply(thr)
                if not self.stall.active:
                    thr, brk = self.accel_gov.update(self.sim_t, speed, v_tgt, thr, brk)

                # ---- 安全闸 -----------------------------------------------
                if blk_ctrl is not None and float(blk_ctrl["dist"]) < C.BLOCK_STOP_M and speed > 0.30:
                    thr, brk = 0.0, 0.35
                    self.log.event("hard_brake", dist=round(float(blk_ctrl["dist"]), 2))

                thr_cap, brk_cap, a_limit, slew = self.cfg.caps
                thr = max(0.0, min(thr_cap, thr))
                brk = max(0.0, min(brk_cap, brk))

                # ---- 路口停住等灯（m04041 方案 + m04362 停车位置修正） --------
                # 已决定在停止线前等「红转绿」：按「还差多远」反算需要的减速度，
                # 只补「滑行（COAST_DECEL）给不了」的那部分。
                # r21 的教训：固定给 0.35，1 s 内 3.24→0.47，停在线前 10 m，
                # 平台判「红灯时停车位置不正确」−20。
                if light.waiting and light.stop_gap is not None:
                    _to_stop = float(light.stop_gap) - C.LIGHT_STOP_AT_M
                    if _to_stop <= 0.20:
                        # 已经到停车点：踩住，别往线上顶
                        thr = 0.0
                        brk = max(float(brk), C.LIGHT_WAIT_BRAKE)
                        self.a_cmd_prev = None
                    elif speed > 0.20:
                        _need = (float(speed) ** 2) / (2.0 * max(_to_stop, 0.20))
                        _extra = _need - C.LIGHT_COAST_DECEL_MS2
                        if _extra > 0.10:
                            # 需要的减速度超过滑行能力 → 补一点刹车（随差额渐增）
                            thr = 0.0
                            brk = max(float(brk),
                                      min(C.LIGHT_WAIT_BRAKE,
                                          C.LIGHT_WAIT_BRAKE * min(1.0, _extra / 2.0)))
                            self.a_cmd_prev = None
                    elif speed < C.LIGHT_CREEP_MS and _to_stop > C.LIGHT_CREEP_M:
                        # 停早了：小油门爬到停车点（不踩刹车、不补 stall、不超 0.05）
                        thr = min(max(float(thr), 0.02), C.LIGHT_CREEP_THROTTLE)
                        brk = 0.0
                if light.holding and speed < 1.20:
                    thr = 0.0
                    brk = max(float(brk), C.LIGHT_WAIT_BRAKE)
                    self.a_cmd_prev = None
                if (light.waiting or light.released) and \
                        self.sim_t - getattr(self, "_last_hold_say", -10.0) > 2.0:
                    self._last_hold_say = self.sim_t
                    self.log.event("light_hold", v=round(float(speed), 2),
                                   gap=None if light.stop_gap is None
                                   else round(float(light.stop_gap), 2),
                                   state=light.state, brk=round(float(brk), 3))
                    self._say("[%.1fs] 路口等灯 gap=%.1f %s v=%.2f"
                              % (self.sim_t, float(light.stop_gap or 0.0),
                                 light.state, float(speed)))
                if light.released:
                    # 放行这一帧：把「卡死补油门」和「软启动」都复位，重新平稳起步
                    self.stall.reset()
                    try:
                        self.soft.soft_start(self.sim_t)
                    except Exception:  # noqa: BLE001
                        pass

                # ---- 横向 -------------------------------------------------
                steer = 0.0
                if self.stanley is not None and self.cfg.stanley and self.sim_t < self.stanley_until:
                    self.stanley.speed_ref = speed
                    steer = float(self.stanley.steering(x, y, yaw))
                elif self.pursuit is not None:
                    steer = float(self.pursuit.steering(x, y, yaw))

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

                # 急转限速看的是**实际下发**的方向盘：之前这行在元素覆盖之前，
                # 元素 6 并到 +3.50 m 左道后，基线纯跟踪仍在追没偏移的中心线，
                # 每帧都算出 0.53 rad 的"回中"修正 → 急转限速永久把 v_tgt 压到
                # 3.20 m/s（elem6-r3 实测 2369/2489 帧，整段 168 m 用了 81 s）。
                self.steer_prev = float(steer)

                # 策略全权接管（`Command.final`，见 parkslot.py）：升挡闸门与终点驻车
                # 都不许再插手——元素 12 要开过路线末点 6 m 去 staging、再倒车入库，
                # 基线在这两处抢一次控制就会把车钉死在车位前（elem89-r1 的 0 分）。
                _final = bool(cmd is not None and cmd.final)

                # ---- 低速段油门上限 ---------------------------------------
                # 平台的加速度-油门比在低速段比模型（ACCEL_CMD_A=8）大得多：5.5 m/s
                # 时 0.36 的油门实测就是 +5.1 m/s²（环岛进出各一次，2026-10-29
                # 15:28/15:40 两局都在这丢过 +10）。这里把 1~8 m/s 的油门压到 0.30，
                # 实测对应 4.2 m/s²，留出余量。
                # **必须在升挡闸门之前**：闸门要顶 0.86 才能过一挡墙，压在它后面会把
                # 升挡也压掉（16:03 那局就卡死在 4.78 m/s）。
                if 1.0 < float(speed) < 8.0 and float(thr) > float(C.LOW_SPEED_THR_CAP):
                    thr = float(C.LOW_SPEED_THR_CAP)

                # ---- 升挡闸门（一挡墙；唯一允许超过 0.62 油门的通道）---------
                if not _final:
                    thr, brk = self._shift_gate(speed, v_tgt, thr, brk, s_now)

                # ---- 终点驻车 ---------------------------------------------
                # 元素 12 自己管泊车（`Command.park`）：本站车位中心就是路线末点，
                # 倒车入库必须先把车开过它 6 m 到 staging，所以不能让 EndPark 在
                # `remain<3 m` 时抢走控制（elem89-r1 就是这样停在 s=4716.4 的）。
                remain = self.track.length - s_now
                self_park = bool(_final or (cmd is not None and cmd.park))
                if self.phase == "run" and remain < 3.0 and not self_park:
                    self.phase = "hold"
                    self._say(f"[{self.sim_t:6.1f}s] 到终点前 3 m，进入驻车")
                if self.phase == "hold" and not self_park:
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
    def _shift_gate(self, speed, v_tgt, thr, brk, s_now=0.0):
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
                    self._shifted_ok = True      # 全局只换这一次（m03278）
                self.log.event("shift_done", v=round(float(speed), 2),
                               ok=float(speed) >= C.SHIFT_DONE_MS)
                self._say(f"[{now:6.1f}s] 升挡完成：v={speed:.2f}")
                return thr, brk
            return C.SHIFT_THROTTLE, 0.0

        stuck = abs(float(speed) - C.KICK_WALL_MS) <= C.KICK_WALL_TOL_MS
        want = float(v_tgt) >= C.SHIFT_TARGET_MS - 0.01
        long_span = (float(getattr(self, "_span_len", 0.0)) >= C.SHIFT_MIN_SPAN_M
                     or bool(getattr(self, "_pre_shift_ok", False)))
        # m03278：换挡整局只做一次，而且放在不评分的「起步直行」段（0~8.2 m）里做完。
        # 一个 8.2 m 的段里车 1 s 就到一挡墙，所以这一段的闸门随便开；成功升上去之后
        # 全局不再开闸（curve-r11 是「每个元素一次」，元素 2 里仍被触发 4 次）。
        in_start = str(getattr(self, "_span_name", "")) == C.START_SPAN_NAME
        once = (not bool(getattr(self, "_shifted_ok", False))
                and (in_start or self._gate_span != getattr(self, "_cur_span_index", None)))
        # 平台换挡模型在换挡帧的固定速度阶跃（4.88 → 5.53 → 6.20 m/s）与控制无关，
        # 这一脚只能挑一个元素去承担。`SHIFT_MIN_S_M=60` 把它从元素 1（开头）挪到
        # 元素 1 结束之后：元素 1 只跑 0→4.78 的起步（平台对 0→20 km/h 不判，
        # 21:20 实测元素 1 = 110），升挡的代价改由元素 2 承担。详见 config.py。
        early = float(s_now) < float(C.SHIFT_MIN_S_M)
        if (stuck and want and (long_span or in_start) and once and not early
                and (now - self.last_shift) >= C.SHIFT_MIN_GAP_S):
            self.last_shift = now
            self._shift_t0 = now
            self._gate_span = getattr(self, "_cur_span_index", None)
            self.shift_until = now + C.SHIFT_MAX_S
            self.log.event("shift_gate", v=round(float(speed), 2),
                           tgt=round(float(v_tgt), 2), thr=C.SHIFT_THROTTLE)
            self._say(f"[{now:6.1f}s] 开升挡闸门：v={speed:.2f} 目标={v_tgt:.2f}，"
                      f"油门 {C.SHIFT_THROTTLE:.2f}"
                      f"（最长 {C.SHIFT_MAX_S:.1f}s）")
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

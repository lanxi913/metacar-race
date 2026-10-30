# -*- coding: utf-8 -*-
r"""港口迷雾全流程执行器：信号链 → 路障扫描 → 路口通行 → 泊位交付。

阶段（对应 1000 分）：
    1. 三信号（240）：巡检车按顺序开到 7 m 内，读 hint 里的 KEY-XXX，submit_key。
    2. 路障扫描（160）：动态路障 Roadblock_* 在 30 m 内可见后，巡检车开到 9 m 内 scan。
    3. 路口通行（200）：运输车到 Intersection_Center 12 m 内、对齐进口车道，request_entry，
       穿到对侧 13 m 后 release。两辆运输车各做一次。
    4. 泊位交付（400）：Berth_A/B 解锁后，把运输车停到 3 m 内 deliver。

硬性约定：
    - 每 0.2 s 发四车控制，其余车 PARKING + 满刹车。
    - 出现碰撞/违规事件立即终止本轮（用户要求）。
    - 卡住只倒车脱困一次，第二次终止。
    - 路障只有 WorkZoneDivider 参与避障；检查站 PassageBarrier 中间的通道是允许走的。

用法：
    .venv\Scripts\python.exe -u tools/portfog_full.py --seconds 560
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import portfog_run3 as base  # noqa: E402

from metacar_multiagent import GearMode, ScenarioTaskAPI, VehicleControl  # noqa: E402

CONTROL_PERIOD = base.CONTROL_PERIOD


class Full(base.PortFog):
    """在 run3 的基础上补齐扫描、路口、泊位三个阶段。"""

    def __init__(self, api):
        super().__init__(api)
        # 通道用的图：不避障。检查站中间 7 m 就是给人走的，避障会把它整段封死。
        static = api.get_scene_static_data()
        self.roads = static.roads
        self.static_objects = static.static_objects
        self.nodes_plain, self.edges_plain = base.build_graph(static.roads, [])

    # ---------- 工具 ----------
    def point_to(self, vid, tx, ty, stop_dist=1.5, timeout_s=25, throttle=0.2, max_speed=4.0):
        """朝一个明确坐标点开（不走路网），用于对齐进口车道、穿路口、泊位微调。"""
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            if self.check_events():
                print(f"  违规，终止：{self.violation_msg}", flush=True)
                return False
            vs = self.api.get_vehicle_state(vid)
            p = vs.pose
            d = math.hypot(tx - p.pos_x, ty - p.pos_y)
            if d <= stop_dist:
                self.send(self.idle())
                return True
            err = base.wrap_pi(math.atan2(ty - p.pos_y, tx - p.pos_x) - base.yaw_of(p))
            steer = max(-0.6, min(0.6, -1.5 * err))
            if vs.speed_mps > max_speed:
                ctrl = VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=0.20, steering=steer)
            else:
                ctrl = VehicleControl(gear=GearMode.DRIVE, throttle=throttle, brake=0.0, steering=steer)
            controls = self.idle()
            controls[vid] = ctrl
            self.send(controls)
            time.sleep(CONTROL_PERIOD)
        self.send(self.idle())
        return False

    def reverse_along_x(self, vid, target_x, timeout_s=45):
        """沿当前车道倒车到 x >= target_x（只用于从停车位退到路口，避免原地掉头撞邻车）。"""
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            if self.check_events():
                print(f"  违规，终止：{self.violation_msg}", flush=True)
                return False
            vs = self.api.get_vehicle_state(vid)
            p = vs.pose
            if p.pos_x >= target_x:
                self.send(self.idle())
                print(f"  倒车到位 x={p.pos_x:.1f}", flush=True)
                return True
            brake = max(0.0, min(0.4, (vs.speed_mps - 1.2) * 0.25))
            ctrl = VehicleControl(gear=GearMode.REVERSE, throttle=0.20, brake=brake, steering=0.0)
            controls = self.idle()
            controls[vid] = ctrl
            self.send(controls)
            time.sleep(CONTROL_PERIOD)
        self.send(self.idle())
        return False

    def straight_south(self, vid, x, y_to, timeout_s=45, speed=1.2):
        """沿 x 常量直线向南慢速通过（检查站通道窄，必须锁横向偏差）。"""
        start = time.monotonic()
        last_log = -1.0
        while time.monotonic() - start < timeout_s:
            if self.check_events():
                print(f"  违规，终止：{self.violation_msg}", flush=True)
                return False
            vs = self.api.get_vehicle_state(vid)
            p = vs.pose
            if p.pos_y <= y_to:
                self.send(self.idle())
                print(f"  穿通道完成 y={p.pos_y:.2f} x={p.pos_x:.2f}", flush=True)
                return True
            yaw = base.yaw_of(p)
            heading_err = base.wrap_pi(-math.pi / 2.0 - yaw)      # 目标朝南
            lateral = p.pos_x - x                                  # 正 = 偏东
            # 朝南时 steer 正 = 顺时针 = 向西，所以"偏东"要用正转向往西修。
            steer = max(-0.5, min(0.5, -1.2 * heading_err + 0.35 * lateral))
            if vs.speed_mps > speed:
                ctrl = VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=0.10, steering=steer)
            else:
                ctrl = VehicleControl(gear=GearMode.DRIVE, throttle=0.12, brake=0.0, steering=steer)
            controls = self.idle()
            controls[vid] = ctrl
            self.send(controls)
            now = time.monotonic() - start
            if now - last_log >= 1.0:
                last_log = now
                print(f"    穿通道 x={p.pos_x:.2f} y={p.pos_y:.2f} yaw={math.degrees(yaw):.0f} "
                      f"lat={lateral:+.2f} steer={steer:+.2f} v={vs.speed_mps:.2f}", flush=True)
            time.sleep(CONTROL_PERIOD)
        self.send(self.idle())
        return False

    @staticmethod
    def _seg_dist(px, py, ax, ay, bx, by):
        """点到线段距离。"""
        vx, vy = bx - ax, by - ay
        wx, wy = px - ax, py - ay
        l2 = vx * vx + vy * vy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, (wx * vx + wy * vy) / l2))
        return math.hypot(px - (ax + vx * t), py - (ay + vy * t))

    def lane_through(self, jx, jy, max_dist=4.0):
        """在静态路网里找穿过路口的那条车道，返回 (lane_id, [(x,y), ...])。"""
        best = None
        for road in self.roads:
            for lane in road.lanes:
                pts = [(p.x, p.y) for p in lane.path_points]
                if len(pts) < 2:
                    continue
                d = min(self._seg_dist(jx, jy, pts[i][0], pts[i][1],
                                       pts[i + 1][0], pts[i + 1][1])
                        for i in range(len(pts) - 1))
                if d <= max_dist and (best is None or d < best[0]):
                    best = (d, lane.id, pts)
        if best is None:
            return None
        return best[1], best[2]

    def drive_along(self, vid, p0, p1, speed=1.2, timeout_s=60):
        """沿 p0→p1 每 1 m 取一个路径点，逐点追；最后 2.5 m 降速，保证停在点上。

        之前用 2 m 步长 + 每个点 1.2 m 容差 + 整体超时，最后一个点经常冲过 3 m，
        车头跟着斜 30°，request 就判不在入口车道内。现在：
          - 1 m 步长、0.6 m 容差；
          - 距终点 2.5 m 内目标速度降到 0.6 m/s；
          - 每个点单独 12 s 超时，不会整段拖死。
        """
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        length = math.hypot(dx, dy)
        if length < 0.5:
            return True
        n = max(1, int(length / 1.0))
        start_all = time.monotonic()
        last_log = -1.0
        for i in range(1, n + 1):
            tx = p0[0] + dx * i / n
            ty = p0[1] + dy * i / n
            remain = length * (n - i) / n
            target_v = 0.6 if remain < 2.5 else speed
            pt_start = time.monotonic()
            while time.monotonic() - pt_start < 12.0 and time.monotonic() - start_all < timeout_s:
                if self.check_events():
                    print(f"  违规，终止：{self.violation_msg}", flush=True)
                    return False
                vs = self.api.get_vehicle_state(vid)
                p = vs.pose
                if math.hypot(tx - p.pos_x, ty - p.pos_y) < 0.6:
                    break
                err = base.wrap_pi(math.atan2(ty - p.pos_y, tx - p.pos_x) - base.yaw_of(p))
                steer = max(-0.5, min(0.5, -1.5 * err))
                dv = target_v - vs.speed_mps
                if dv > 0.3:
                    ctrl = VehicleControl(gear=GearMode.DRIVE,
                                          throttle=min(0.20, 0.08 + 0.06 * dv),
                                          brake=0.0, steering=steer)
                else:
                    ctrl = VehicleControl(gear=GearMode.DRIVE, throttle=0.0,
                                          brake=min(0.25, 0.06 + 0.06 * (-dv)), steering=steer)
                controls = self.idle()
                controls[vid] = ctrl
                self.send(controls)
                now = time.monotonic() - start_all
                if now - last_log >= 1.0:
                    last_log = now
                    print(f"    沿车道 pt={i}/{n} pos=({p.pos_x:.2f},{p.pos_y:.2f}) "
                          f"yaw={math.degrees(base.yaw_of(p)):.0f} v={vs.speed_mps:.2f} "
                          f"目标={target_v:.1f}", flush=True)
                time.sleep(CONTROL_PERIOD)
            else:
                self.send(self.idle())
                print(f"    沿车道卡在 pt={i}/{n}，返回 False", flush=True)
                return False
        # 终点处彻底停稳
        for _ in range(5):
            ctrl = self.idle()
            ctrl[vid] = VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=0.3, steering=0.0)
            self.send(ctrl)
            time.sleep(CONTROL_PERIOD)
        self.send(self.idle())
        return True

    def visible(self, vid, object_type=None, prefix=None):
        """返回指定类型的可见任务对象。"""
        obs = self.api.get_observation(vid)
        out = []
        for o in obs.visible_objects:
            if object_type and o.object_type != object_type:
                continue
            if prefix and not o.object_id.startswith(prefix):
                continue
            out.append(o)
        return out

    # ---------- 阶段 2：路障扫描 ----------
    def scan_phase(self, patrol_vids, deadline):
        """找到动态路障并 scan，每辆车都试一遍。"""
        for vid in patrol_vids:
            if time.monotonic() > deadline:
                return False
            rbs = self.visible(vid, object_type="roadblock")
            if not rbs:
                # 路障可能在别的区域，先沿已知路网往码头方向挪一段
                for tx, ty in ((752.5, -94.0), (720.0, -90.0)):
                    if time.monotonic() > deadline:
                        return False
                    self.drive_to(vid, tx, ty, stop_dist=12.0, timeout_s=60)
                    rbs = self.visible(vid, object_type="roadblock")
                    if rbs:
                        break
            if not rbs:
                continue
            rb = rbs[0]
            print(f"  发现 {rb.object_id} @ ({rb.position.x:.1f},{rb.position.y:.1f}) "
                  f"dist={rb.distance_m:.1f}", flush=True)
            if self.creep_to(vid, rb.position.x, rb.position.y, stop_dist=9.0, timeout_s=35):
                res = self.api.interact(vid, rb.object_id, "scan")
                print(f"  scan {rb.object_id} -> granted={res.granted} {res.message!r}", flush=True)
                if res.granted:
                    return True
        return False

    # ---------- 阶段 3：路口通行 ----------
    def intersection_phase(self, cargo_vids, deadline):
        """两辆运输车先并行开到路口附近，再串行申请/通行（路口本身必须单车互斥）。"""
        done = 0
        # 并行前置：各车只发自己的控制，互不覆盖
        def _advance(v):
            try:
                vs0 = self.api.get_vehicle_state(v)
                if vs0.pose.pos_x < 630 and abs(vs0.pose.pos_y - 120.8) < 4.0:
                    self.reverse_along_x(v, 633.0)
                self.drive_to(v, 726.7, 4.7, stop_dist=6.0, timeout_s=150)
            except Exception as exc:  # noqa: BLE001
                print(f"  {v} 前置移动异常: {exc}", flush=True)

        threads = [threading.Thread(target=_advance, args=(v,)) for v in cargo_vids]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        for vid in cargo_vids:
            if time.monotonic() > deadline or done >= 2 or self.violation:
                break
            # 运输车停机位朝西、出口在东：先倒车到路口，避免原地掉头扫到邻车
            vs0 = self.api.get_vehicle_state(vid)
            if vs0.pose.pos_x < 630 and abs(vs0.pose.pos_y - 120.8) < 4.0:
                self.reverse_along_x(vid, 633.0)

            # 找路口对象（对象只在 30 m 内可见，先沿路网朝主路南侧靠）
            ic = self.visible(vid, object_type="intersection")
            if not ic:
                self.drive_to(vid, 726.7, 4.7, stop_dist=6.0, timeout_s=110)
                ic = self.visible(vid, object_type="intersection")
            if not ic:
                print(f"  {vid} 看不到路口对象", flush=True)
                continue
            jx, jy = ic[0].position.x, ic[0].position.y
            print(f"  路口 @ ({jx:.1f},{jy:.1f})", flush=True)

            # 通用几何：从静态路网里取"穿过路口"的那条车道中心线，
            # 沿它的方向外推出进口点/出口点，不再写死 728.8、-13 这类经验值。
            lane = self.lane_through(jx, jy)
            if lane is None:
                print("  找不到穿过路口的车道", flush=True)
                return done
            lane_id, pts = lane
            ax, ay = pts[0]
            bx, by = pts[-1]
            length = math.hypot(bx - ax, by - ay)
            ux, uy = (bx - ax) / length, (by - ay) / length
            vs = self.api.get_vehicle_state(vid)
            p = vs.pose
            t_veh = (p.pos_x - ax) * ux + (p.pos_y - ay) * uy
            t_j = (jx - ax) * ux + (jy - ay) * uy
            if t_veh > t_j:          # 车在出口侧 → 反过来走
                ux, uy = -ux, -uy
            entry = (jx - ux * 10.0, jy - uy * 10.0)
            exit_pt = (jx + ux * 13.0, jy + uy * 13.0)
            print(f"  车道 {lane_id} 方向=({ux:+.2f},{uy:+.2f}) "
                  f"进口=({entry[0]:.1f},{entry[1]:.1f}) 出口=({exit_pt[0]:.1f},{exit_pt[1]:.1f})",
                  flush=True)

            # 回到昨天 600 分那轮的做法：drive_to 已经停在路口 10~11 m 的入口车道内，
            # 不要再 point_to/straight_south 额外挪车（那会把车推到 732、贴东侧围栏）。
            self.point_to(vid, jx, entry[1] + uy * 2.3, stop_dist=0.3, timeout_s=35, throttle=0.10, max_speed=0.8)
            _p = self.api.get_vehicle_state(vid).pose
            print(f"  request 前 pos=({_p.pos_x:.2f},{_p.pos_y:.2f}) "
                  f"yaw={math.degrees(base.yaw_of(_p)):.0f} "
                  f"dist={math.hypot(_p.pos_x - jx, _p.pos_y - jy):.1f}", flush=True)
            # 文档 4.2：入口车道内 + 距中心 ≤12 m + 未进 6 m 互斥区，停稳即可申请。
            res = None
            for payload in (None, None, None, None, None, "north", "south", "west", "east"):
                try:
                    res = self.api.interact(vid, ic[0].object_id, "request_entry", payload)
                except Exception as exc:  # noqa: BLE001
                    print(f"    request payload={payload!r} 异常: {exc}", flush=True)
                    continue
                print(f"    request payload={payload!r} -> granted={res.granted} "
                      f"{res.message!r}", flush=True)
                if res.granted:
                    break
            if not res.granted:
                # 换思路：北侧判定不通过，绕到通道另一侧，从南向北申请。
                # 文档只写"另一侧出口"，但本局的实际入口可能就是南侧。
                print("  北侧申请失败，绕到南侧入口再试", flush=True)
                far = (jx + ux * 13.0, jy + uy * 13.0)
                self.drive_to(vid, far[0], far[1], stop_dist=8.0, timeout_s=140)
                self.point_to(vid, far[0], far[1], stop_dist=1.0, timeout_s=35,
                              throttle=0.15, max_speed=1.5)
                self.drive_along(vid, (far[0] + ux * 3.0, far[1] + uy * 3.0), far,
                                 speed=0.8, timeout_s=25)
                _p = self.api.get_vehicle_state(vid).pose
                print(f"  south request 前 pos=({_p.pos_x:.2f},{_p.pos_y:.2f}) "
                      f"yaw={math.degrees(base.yaw_of(_p)):.0f} "
                      f"dist={math.hypot(_p.pos_x - jx, _p.pos_y - jy):.1f}", flush=True)
                res = self.api.interact(vid, ic[0].object_id, "request_entry")
                print(f"  south request_entry -> granted={res.granted} {res.message!r}", flush=True)
                if not res.granted:
                    return done
                # 从南向北穿
                self.drive_along(vid, far, (jx - ux * 13.0, jy - uy * 13.0),
                                 speed=1.2, timeout_s=70)
                res = self.api.interact(vid, ic[0].object_id, "release")
                print(f"  release {vid} (south->north) -> granted={res.granted} {res.message!r}",
                      flush=True)
                if res.granted:
                    done += 1
                return done
            # granted 时车头可能是斜的（这一轮 −148°、x 贴西围栏），直接前进会撞栏。
            # 只倒 4 帧（约 1.5 m）把车头摆向南，再沿车道穿到出口；倒太多会退出入口。
            for _ in range(4):
                _c = self.idle()
                _c[vid] = VehicleControl(gear=GearMode.REVERSE, throttle=0.20, brake=0.0,
                                         steering=-0.4)
                self.send(_c)
                time.sleep(CONTROL_PERIOD)
            _now = self.api.get_vehicle_state(vid).pose
            print(f"  摆正后 pos=({_now.pos_x:.2f},{_now.pos_y:.2f}) "
                  f"yaw={math.degrees(base.yaw_of(_now)):.0f}", flush=True)
            if not self.drive_along(vid, (_now.pos_x, _now.pos_y), exit_pt,
                                    speed=1.5, timeout_s=70):
                print(f"  {vid} 穿路口失败，先停下不再派第二辆", flush=True)
                return done
            res = self.api.interact(vid, ic[0].object_id, "release")
            print(f"  release {vid} -> granted={res.granted} {res.message!r}", flush=True)
            if res.granted:
                done += 1
                # 驶离出口 12 m，别把通道口堵住（上一轮 Car_B 挤了 259 s）
                self.drive_along(vid, exit_pt,
                                 (exit_pt[0] + ux * 12.0, exit_pt[1] + uy * 12.0),
                                 speed=2.0, timeout_s=30)
        return done

    # ---------- 阶段 4：泊位交付 ----------
    def _unlocked_berths(self):
        state = self.api.get_task_state()
        return [o for o in state.shared_task_objects
                if o.object_type == "destination" and o.position
                and math.hypot(o.position.x, o.position.y) > 1.0]

    def berth_phase(self, cargo_vids, deadline):
        """两辆运输车分别去堆场东/西观测点，产生有效观测把泊位抬到 lv3 再交付。"""
        done = 0
        spots = ((704.0, 24.0), (743.0, 24.0))
        for i, vid in enumerate(cargo_vids):
            if time.monotonic() > deadline:
                break
            tx, ty = spots[i % len(spots)]
            print(f"  {vid} 去堆场观测点 ({tx:.0f},{ty:.0f})", flush=True)
            self.drive_to(vid, tx, ty, stop_dist=10.0, timeout_s=110)
            # 观测 20 s：对可见的 destination 提交 report_observation（带 token）
            for _ in range(10):
                obs = self.api.get_observation(vid)
                for vo in obs.visible_objects:
                    print(f"    {vid} 可见 {vo.object_id} {vo.object_type} "
                          f"d={vo.distance_m:.1f} token={'Y' if vo.observation_token else 'N'}",
                          flush=True)
                    if vo.object_type == "destination" and vo.observation_token:
                        try:
                            r = self.api.interact(vid, vo.object_id, "report_observation",
                                                  {"observation_token": vo.observation_token})
                            print(f"    report {vo.object_id} granted={r.granted} {r.message!r}",
                                  flush=True)
                        except Exception as exc:  # noqa: BLE001
                            print(f"    report {vo.object_id} 异常 {exc}", flush=True)
                time.sleep(2.0)
            berths = self._unlocked_berths()
            if not berths:
                print("  泊位仍未解锁", flush=True)
                return done
            b = berths[i % len(berths)]
            print(f"  泊位 {b.object_id} @ ({b.position.x:.1f},{b.position.y:.1f})", flush=True)
            if self.drive_to(vid, b.position.x, b.position.y, stop_dist=3.0, timeout_s=110):
                res = self.api.interact(vid, b.object_id, "deliver")
                print(f"  deliver {b.object_id} -> granted={res.granted} {res.message!r}", flush=True)
                if res.granted:
                    done += 1
        return done

    # ---------- 主流程 ----------
    def run_full(self, deadline):
        patrol = ["Car_C", "Car_D"]
        cargo = ["Car_A", "Car_B"]

        print("\n########## 阶段 1：信号链 ##########", flush=True)
        if not self.run(patrol, deadline):
            return False

        print("\n########## 阶段 2：路障扫描 ##########", flush=True)
        scanned = self.scan_phase(patrol, deadline)
        print(f"扫描结果: {scanned}", flush=True)

        print("\n########## 阶段 3：路口通行 ##########", flush=True)
        passed = self.intersection_phase(cargo, deadline)
        print(f"路口完成: {passed}", flush=True)

        print("\n########## 阶段 4：泊位交付 ##########", flush=True)
        delivered = self.berth_phase(cargo, deadline)
        print(f"泊位完成: {delivered}", flush=True)

        s = self.api.get_task_state()
        print(f"\n最终 signals={s.verified_signal_count}/{s.total_signal_count} "
              f"dest={s.unlocked_destination_count}/{s.total_destination_count}", flush=True)
        return True


def main() -> int:
    parser = argparse.ArgumentParser(description="港口迷雾全流程")
    parser.add_argument("--seconds", type=float, default=560.0)
    args = parser.parse_args()

    with ScenarioTaskAPI(port=9100) as api:
        fog = Full(api)
        deadline = time.monotonic() + args.seconds
        fog.run_full(deadline)
        api.set_vehicle_controls(fog.idle())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

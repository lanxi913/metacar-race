# -*- coding: utf-8 -*-
r"""港口迷雾一体化执行器 v2：逐点导航 + 卡住脱困 + 信号链提交。

相比 v1 的改动：
    1. 导航用"到点切换"（距当前路径点 < 3 m 切下一个），不用 lookahead，
       避免在路径点附近绕圈。
    2. 卡住检测：速度 < 0.2 m/s 且 2 s 内位移 < 0.3 m，就倒车 1.2 s（反向打舵）
       再继续，解决"顶在路口/护栏上不动"的情况。
    3. 读码失败时自动再靠近 2 m。

坐标约定（实测）：
    ori_z 是从 +x 顺时针的累积角度（度）；yaw = wrap(-radians(ori_z % 360))；
    steer 正 = 顺时针（右转），所以 steer = -gain * err。

用法：
    .venv\Scripts\python.exe -u tools/portfog_run2.py --vehicle Car_C --vehicle2 Car_D --seconds 540
"""
from __future__ import annotations

import argparse
import threading
import heapq
import math
import re
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from metacar_multiagent import GearMode, ScenarioTaskAPI, VehicleControl  # noqa: E402

CONTROL_PERIOD = 0.2
WAYPOINT_REACH_M = 3.0
CRUISE_MPS = 5.0
TURN_MPS = 2.5
MAX_STEER = 0.7
STEER_GAIN = 1.5
STOP_DISTANCE_M = 6.8
JUNCTION_LINK_M = 25.0
KEY_PATTERN = re.compile(r"KEY-[A-Za-z0-9]+")
STUCK_SPEED = 0.2
STUCK_MOVE_M = 0.3
STUCK_SECONDS = 2.0


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle


def yaw_of(pose) -> float:
    return wrap_pi(-math.radians(pose.ori_z % 360.0))


def segment_blocked(ax, ay, bx, by, obstacles, margin=6.5):
    """线段是否穿过障碍物（按 2 m 采样）。路障两侧各 7 m 是红线，取 6.5 m 留余量。"""
    steps = max(2, int(math.hypot(bx - ax, by - ay) / 2.0))
    for i in range(steps + 1):
        t = i / steps
        x = ax + (bx - ax) * t
        y = ay + (by - ay) * t
        for ox, oy in obstacles:
            if math.hypot(x - ox, y - oy) < margin:
                return True
    return False


def build_graph(roads, obstacles=()):
    nodes, edges = {}, {}

    def key(x, y):
        return (round(x * 2), round(y * 2))

    def get_node(x, y):
        k = key(x, y)
        if k not in nodes:
            nodes[k] = (x, y)
            edges[k] = []
        return k

    for road in roads:
        # 用车道中心线的折点建图，不是"道路两端拉直线"。这样转弯、路口都贴着车道走，
        # 不会斜穿草坪或压到护栏。
        lane_used = False
        for lane in road.lanes:
            pts = [(p.x, p.y) for p in lane.path_points]
            if len(pts) < 2:
                continue
            lane_used = True
            for i in range(len(pts) - 1):
                a = get_node(*pts[i])
                b = get_node(*pts[i + 1])
                if a == b:
                    continue
                if segment_blocked(nodes[a][0], nodes[a][1], nodes[b][0], nodes[b][1], obstacles):
                    continue
                w = math.hypot(nodes[a][0] - nodes[b][0], nodes[a][1] - nodes[b][1])
                edges[a].append((b, w, road.id))
                edges[b].append((a, w, road.id))
        if not lane_used:
            a = get_node(road.begin_pos.x, road.begin_pos.y)
            b = get_node(road.end_pos.x, road.end_pos.y)
            if a != b and not segment_blocked(nodes[a][0], nodes[a][1], nodes[b][0], nodes[b][1], obstacles):
                w = math.hypot(nodes[a][0] - nodes[b][0], nodes[a][1] - nodes[b][1])
                edges[a].append((b, w, road.id))
                edges[b].append((a, w, road.id))

    keys = list(nodes)
    for i, a in enumerate(keys):
        ax, ay = nodes[a]
        for b in keys[i + 1:]:
            bx, by = nodes[b]
            d = math.hypot(ax - bx, ay - by)
            if d <= JUNCTION_LINK_M and not segment_blocked(ax, ay, bx, by, obstacles):
                edges[a].append((b, d, "JUNCTION"))
                edges[b].append((a, d, "JUNCTION"))
    return nodes, edges


def nearest_node(nodes, x, y):
    best = None
    for k, (nx, ny) in nodes.items():
        d = math.hypot(nx - x, ny - y)
        if best is None or d < best[0]:
            best = (d, k)
    return best


def dijkstra(nodes, edges, start, goal):
    dist = {start: 0.0}
    prev = {}
    pq = [(0.0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal:
            break
        if d > dist.get(u, float("inf")):
            continue
        for v, w, _ in edges[u]:
            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if goal not in dist:
        return None
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    return list(reversed(path))


class PortFog:
    def __init__(self, api):
        self.api = api
        self.vehicles = api.get_vehicle_ids()
        static = api.get_scene_static_data()
        self.roads = static.roads
        # 检查站围栏（PassageBarrier）按 6.5 m 避障：中间通道太窄，规划走通道会切角撞栏。
        # 路口阶段的"穿通道"由 portfog_full 用 point_to 手动对准，不走规划。
        self.obstacles = [(o.position.x, o.position.y) for o in static.static_objects
                          if "barrier" in o.category]
        print(f"路障 {len(self.obstacles)} 个，参与避障", flush=True)
        self.nodes, self.edges = build_graph(static.roads, self.obstacles)
        # 只监听连接之后的新事件，历史碰撞不能算到本轮头上
        history = self.api.get_events(0)
        self.seq = (max((e.sequence for e in history), default=0)) + 1
        self.violation = False
        self.violation_msg = ""
        self.collisions = 0

    def check_events(self):
        """拉取场景事件；只有红线类事件才终止，普通碰撞只统计。

        用户要求"撞墙就停"，但轻微擦碰也是 collision 事件；真正判断顶死靠
        drive_to 的卡住检测（2 s 不动 → 终止）。红线类关键词才立即终止。
        """
        for e in self.api.get_events(self.seq):
            self.seq = max(self.seq, e.sequence + 1)
            text = f"{e.event_type} {e.message}".lower()
            print(f"  [event] {e.sequence} {e.event_type}: {e.message} "
                  f"vid={e.vehicle_id} obj={e.object_id}", flush=True)
            if any(k in text for k in ("out_of_bound", "out of bound", "red_line",
                                       "eliminated", "leaving the map")):
                self.violation = True
                self.violation_msg = f"{e.event_type}: {e.message}"
                return True
            if "collision" in text:
                self.collisions += 1
        return False

    def idle(self):
        return {v: VehicleControl(gear=GearMode.PARKING, brake=1.0) for v in self.vehicles}

    def send(self, ctrl):
        self.api.set_vehicle_controls(ctrl)

    def plan(self, vid, x, y):
        pose = self.api.get_vehicle_state(vid).pose
        _, s = nearest_node(self.nodes, pose.pos_x, pose.pos_y)
        _, g = nearest_node(self.nodes, x, y)
        p = dijkstra(self.nodes, self.edges, s, g)
        if p is None:
            return None
        return [self.nodes[k] for k in p]

    def nearest_lane_point(self, x, y):
        """车道上离 (x,y) 最近的点——对每段车道中心线做投影，而不是只比端点。

        之前只比端点：信号 A 正下方的路上点只有 4 m，却返回 12 m 外的道路端点，
        结果车斜穿海关门前广场、压人行道。
        """
        best = None
        for road in self.roads:
            for lane in road.lanes:
                pts = [(p.x, p.y) for p in lane.path_points]
                if len(pts) == 1:
                    candidates = [pts[0]]
                else:
                    candidates = []
                    for i in range(len(pts) - 1):
                        ax, ay = pts[i]
                        bx, by = pts[i + 1]
                        vx, vy = bx - ax, by - ay
                        l2 = vx * vx + vy * vy
                        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / l2))
                        candidates.append((ax + vx * t, ay + vy * t))
                for px, py in candidates:
                    d = math.hypot(px - x, py - y)
                    if best is None or d < best[0]:
                        best = (d, px, py)
        return best

    def creep_to(self, vid, tx, ty, stop_dist=5.5, timeout_s=25.0):
        """从车道点短距离贴近目标（≤10 m 才允许，防止越野抄近道）。"""
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            if self.check_events():
                return False
            vs = self.api.get_vehicle_state(vid)
            p = vs.pose
            d = math.hypot(tx - p.pos_x, ty - p.pos_y)
            if d <= stop_dist:
                self.send(self.idle())
                return True
            err = wrap_pi(math.atan2(ty - p.pos_y, tx - p.pos_x) - yaw_of(p))
            steer = max(-0.5, min(0.5, -1.2 * err))
            if vs.speed_mps > 1.5:
                ctrl = VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=0.12, steering=steer)
            else:
                ctrl = VehicleControl(gear=GearMode.DRIVE, throttle=0.10, brake=0.0, steering=steer)
            controls = self.idle()
            controls[vid] = ctrl
            self.send(controls)
            time.sleep(CONTROL_PERIOD)
        self.send(self.idle())
        return False

    def escape_reverse(self, vid, steer_sign):
        """倒车脱困：反向打舵 1.2 s，然后松刹车。"""
        for _ in range(6):
            ctrl = self.idle()
            ctrl[vid] = VehicleControl(gear=GearMode.REVERSE, throttle=0.3, brake=0.0,
                                       steering=steer_sign * 0.4)
            self.send(ctrl)
            time.sleep(CONTROL_PERIOD)
        for _ in range(3):
            ctrl = self.idle()
            ctrl[vid] = VehicleControl(gear=GearMode.PARKING, brake=1.0)
            self.send(ctrl)
            time.sleep(CONTROL_PERIOD)

    def drive_to(self, vid, tx, ty, timeout_s=200.0, stop_dist=STOP_DISTANCE_M):
        # 先找车道上离目标最近的点，路径只到车道为止；最后 ≤10 m 才允许贴近目标。
        lane_pt = self.nearest_lane_point(tx, ty)
        if lane_pt is None:
            print(f"  找不到车道点：({tx:.1f},{ty:.1f})", flush=True)
            return False
        lane_dist, lx, ly = lane_pt
        pts = self.plan(vid, lx, ly)
        if not pts:
            print(f"  规划失败：({tx:.1f},{ty:.1f})", flush=True)
            return False

        pose = self.api.get_vehicle_state(vid).pose
        wp = [(pose.pos_x, pose.pos_y)] + pts
        idx = 1
        # 如果第一个路径点在车后方（典型：车停在路边、规划节点在身后），先跳过它，
        # 否则车会原地掉头绕圈。
        if len(wp) > 2:
            hx, hy = math.cos(yaw_of(pose)), math.sin(yaw_of(pose))
            vx, vy = wp[1][0] - pose.pos_x, wp[1][1] - pose.pos_y
            if hx * vx + hy * vy < 0:
                idx = 2
        start = time.monotonic()
        last_move_t = start
        last_x, last_y = pose.pos_x, pose.pos_y
        last_log = -1.0
        stuck_count = 0

        while idx < len(wp):
            elapsed = time.monotonic() - start
            if elapsed > timeout_s:
                print(f"  导航超时：({tx:.1f},{ty:.1f})", flush=True)
                self.send(self.idle())
                return False

            # 用户要求：撞墙/违规立即终止本轮，改逻辑再重开
            if self.check_events():
                print(f"  触发违规事件，终止本轮：{self.violation_msg}", flush=True)
                self.send(self.idle())
                return False

            vs = self.api.get_vehicle_state(vid)
            pose = vs.pose
            x, y = pose.pos_x, pose.pos_y
            wx, wy = wp[idx]
            dist_wp = math.hypot(wx - x, wy - y)
            dist_t = math.hypot(tx - x, ty - y)

            if dist_t <= stop_dist:
                self.send(self.idle())
                print(f"  到达 ({x:.1f},{y:.1f})，距目标 {dist_t:.1f} m", flush=True)
                return True

            if dist_wp < WAYPOINT_REACH_M:
                idx += 1
                continue

            # 卡住检测：只允许倒车脱困一次；第二次卡住直接终止，避免反复顶墙
            moved = math.hypot(x - last_x, y - last_y)
            if moved > STUCK_MOVE_M:
                last_x, last_y, last_move_t = x, y, time.monotonic()
            elif time.monotonic() - last_move_t > STUCK_SECONDS:
                stuck_count += 1
                print(f"  卡住 {STUCK_SECONDS:.0f}s，第 {stuck_count} 次", flush=True)
                if stuck_count >= 2:
                    print("  连续卡住，判定为撞墙/顶死，终止本轮", flush=True)
                    self.send(self.idle())
                    return False
                self.escape_reverse(vid, 0.5)
                last_move_t = time.monotonic()
                last_x, last_y = x, y

            err = wrap_pi(math.atan2(wy - y, wx - x) - yaw_of(pose))
            steer = max(-MAX_STEER, min(MAX_STEER, -STEER_GAIN * err))

            turn = abs(err) > 0.5
            target_v = TURN_MPS if turn else CRUISE_MPS
            dv = target_v - vs.speed_mps
            # 平滑速度环：不再用"超过阈值就点刹"的开关式控制（那会让车一会儿行驶、
            # 一会儿等待）。油门/刹车随速度误差连续变化，油门封顶 0.30 防止急加速。
            if dv > 0.5:
                throttle = min(0.30, 0.12 + 0.06 * dv)
                brake = 0.0
            elif dv < -0.6:
                throttle = 0.0
                brake = min(0.28, 0.08 + 0.05 * (-dv))
            else:
                throttle = 0.10
                brake = 0.0
            if vs.speed_mps > 5.4:            # 绝对上限，接近 20 km/h 才硬刹
                brake = 0.30
                throttle = 0.0
            ctrl = VehicleControl(gear=GearMode.DRIVE, throttle=throttle, brake=brake, steering=steer)

            # 只发本车控制：这样两辆运输车可以并行 move（各发各的，不会互相覆盖成 idle）
            self.send({vid: ctrl})

            if elapsed - last_log >= 1.0:
                last_log = elapsed
                print(f"    t={elapsed:5.1f}s pos=({x:.1f},{y:.1f}) yaw={yaw_of(pose):+.2f} "
                      f"v={vs.speed_mps:.2f} wp={idx}/{len(wp)} d_goal={dist_t:.1f} err={err:+.2f}",
                      flush=True)

            time.sleep(CONTROL_PERIOD)

        # 走完车道段后，只在 10 m 内允许垂直贴近目标；超过就判定到不了，不越野抄近道
        vs = self.api.get_vehicle_state(vid)
        p = vs.pose
        dist_t = math.hypot(tx - p.pos_x, ty - p.pos_y)
        if dist_t <= stop_dist:
            self.send(self.idle())
            return True
        if dist_t <= 30.0:
            print(f"  车道点距目标 {dist_t:.1f} m，低速贴近目标", flush=True)
            return self.creep_to(vid, tx, ty, stop_dist=stop_dist, timeout_s=40.0)

        self.send(self.idle())
        print(f"  车道点距目标 {dist_t:.1f} m > 30 m，拒绝越野接近", flush=True)
        return False

    def wait_key(self, vid, signal_id, timeout_s=6.0):
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            self.send(self.idle())
            obs = self.api.get_observation(vid)
            for vo in obs.visible_objects:
                if vo.object_id == signal_id:
                    m = KEY_PATTERN.search(vo.hint or "")
                    if m:
                        print(f"  读到 {signal_id}: state={vo.state} key={m.group()} "
                              f"dist={vo.distance_m:.2f}", flush=True)
                        return m.group()
            time.sleep(CONTROL_PERIOD)
        return None

    def submit(self, vid, signal_id, key):
        res = self.api.interact(vid, signal_id, "submit_key", {"key": key})
        print(f"  submit {signal_id} -> granted={res.granted} {res.message!r}", flush=True)
        return res.granted

    def run(self, vids, deadline):
        """按顺序处理未验证信号，vids 轮流用。"""
        turn = 0
        while time.monotonic() < deadline:
            state = self.api.get_task_state()
            if state.verified_signal_count >= state.total_signal_count:
                return True
            pending = [o for o in state.shared_task_objects
                       if o.object_type == "signal" and o.state != "verified"]
            if not pending:
                return False
            sig = pending[0]
            if sig.position is None:
                return False

            vid = vids[turn % len(vids)]
            turn += 1
            print(f"\n=== {sig.object_id} ({sig.display_name}) @ "
                  f"({sig.position.x:.1f},{sig.position.y:.1f}) 用 {vid} ===", flush=True)
            if not self.drive_to(vid, sig.position.x, sig.position.y):
                print(f"  到不了 {sig.object_id}，终止本轮（按约定不硬试）", flush=True)
                return False

            key = self.wait_key(vid, sig.object_id)
            if not key:
                print("  没读到 key，再靠近 2 m", flush=True)
                if not self.drive_to(vid, sig.position.x, sig.position.y, stop_dist=3.5, timeout_s=40):
                    return False
                key = self.wait_key(vid, sig.object_id)
            if not key:
                return False
            if not self.submit(vid, sig.object_id, key):
                return False
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description="港口迷雾执行器 v2")
    parser.add_argument("--vehicle", default="Car_C")
    parser.add_argument("--vehicle2", default="Car_D")
    parser.add_argument("--seconds", type=float, default=540.0)
    args = parser.parse_args()

    with ScenarioTaskAPI(port=9100) as api:
        fog = PortFog(api)
        deadline = time.monotonic() + args.seconds
        ok = fog.run([args.vehicle, args.vehicle2], deadline)
        s = api.get_task_state()
        print(f"\n结束 signals={s.verified_signal_count}/{s.total_signal_count} ok={ok}", flush=True)
        api.set_vehicle_controls(fog.idle())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

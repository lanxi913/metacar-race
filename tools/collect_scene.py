# -*- coding: utf-8 -*-
r"""连续场景数据采集器：连上场景后逐帧记录几何、位姿与控制量。

为什么要有这个脚本
    连续场景（5 关，基准 180 / 210 / 285 / 480 / 810 s）是若干元素拼起来的整条赛道。
    开跑之前必须先摸清：赛道中心线怎么走、每个元素从哪个里程到哪个里程、车道边界线
    （实线 / 虚线 / 停止线 / 斑马线）在哪、场景自己给的建议轨迹长什么样。
    这些只能从 API 里抠出来，客户端界面上看不到。

用法（顺序不能反）
    1) 先在终端跑本脚本，它会阻塞在 connect() 等场景：
           .venv\Scripts\python.exe -u tools\collect_scene.py --mode hold
       （或双击 tools\collect_scene.cmd，自带 UTF-8 控制台）
    2) 再去客户端点「启动场景」
    3) 场景结束 / Ctrl-C / 到 --duration 都会正常收尾，数据落在 logs\collect\<时间戳>\

模式
    --mode hold   驻车不动（PARKING + 刹车 1.0）。只采静态几何与逐帧状态，最安全，
                  用来先把赛道画清楚。
    --mode traj   沿场景每帧下发的建议轨迹（SimCarMsg.Trajectory）低速循迹，
                  能把整关跑一遍，采到真实的动态数据。
    --mode route  沿静态 route 低速循迹（建议轨迹为空时的退路）。

    hold 模式加 --skip-after N：每个元素停留 N 仿真秒后调 skip_level() 跳关，
    几分钟内把全部元素的动态数据扫一遍（跳关丢时间附加分，只用于采集）。

产出（logs\collect\<时间戳>\）
    meta.json            参数、git 版本、统计汇总
    static.json          场景静态数据原样（route / roads / sub_scenes / VLA 扩展）
    route.csv            赛道中心线：序号、坐标、累计里程
    subscenes.csv        每个元素的起终点与里程区间
    roads.csv            道路：走向、行驶类型、交通标志、车道数、停止线
    lanes.csv            车道：宽度、左右相邻车道、中心线点数与长度
    borders.csv          车道边界线逐点（左/右，线型）
    frames.csv           逐帧标量：位姿、速度、加速度、控制量、坡道投影、最近障碍物
    events.jsonl         事件流：元素切换、障碍物增减、红绿灯变化、超速/急加速、异常
    frames_detail.jsonl  逐帧完整消息（仅 --detail，体积大）

坐标系约定
    位置是米制右手系；yaw(ori_z) 单位是度、顺时针为正（yaw=90° 时车头朝 -y），
    所以车头在数学坐标系里的朝向是 -yaw。
    lat_err 符号：车在赛道中心线左侧为正、右侧为负。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import platform
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from metacar import GearMode, SceneAPI, VehicleControl

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from metacar_race.config import (  # noqa: E402
    ACCEL_BONUS_LIMIT,
    LIMIT_SCENE_ELEMENT_KMH,
    LOOKAHEAD_M,
)
from metacar_race.control.tracking import OutputLimiter, PurePursuit, SpeedController  # noqa: E402
from metacar_race.control.profile import SpeedProfile  # noqa: E402
from metacar_race.control.reverse_park import ReversePark, find_slot  # noqa: E402

FRAME_FIELDS = [
    "frame", "wall_t", "sim_t", "dt_sim", "elem_t", "elem_limit", "elem_remain",
    "sub_scene", "sub_idx", "sub_progress", "restart",
    "x", "y", "z", "yaw_deg", "vx", "vy", "vz", "speed", "speed_kmh", "v_tgt",
    "accel", "accel_raw",
    "gear", "throttle", "brake", "steering",
    "cmd_gear", "cmd_throttle", "cmd_brake", "cmd_steer",
    "route_idx", "route_s", "route_len", "lat_err", "head_err",
    "dist_route_end", "dist_sub_end",
    "obs_count", "obs_dyn_count", "obs_min_dist", "obs_min_id", "obs_min_type",
    "tl_count",
]


# --------------------------------------------------------------------------- #
# 小工具
# --------------------------------------------------------------------------- #
def f3(value: float, digits: int = 3) -> str:
    return f"{value:.{digits}f}"


def wrap_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def git_info(repo: Path) -> dict:
    def run(*args: str) -> str:
        try:
            done = subprocess.run(
                ["git", "-C", str(repo), *args],
                capture_output=True, text=True, timeout=5,
            )
            return done.stdout.strip()
        except Exception:  # noqa: BLE001
            return ""

    return {
        "head": run("rev-parse", "--short", "HEAD"),
        "branch": run("rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": bool(run("status", "--porcelain")),
    }


# --------------------------------------------------------------------------- #
# 赛道中心线投影
# --------------------------------------------------------------------------- #
class RouteProjector:
    """把车的位置投影到赛道中心线上，给出最近点、里程、横向偏差和航向差。"""

    def __init__(self, route):
        self.n = len(route)
        if self.n:
            self.pts = np.array([[float(p.x), float(p.y)] for p in route], dtype=float)
        else:
            self.pts = np.zeros((0, 2), dtype=float)

        if self.n >= 2:
            seg = np.diff(self.pts, axis=0)
            self.seg_len = np.hypot(seg[:, 0], seg[:, 1])
            self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
            self.total = float(self.cum[-1])
        else:
            self.seg_len = np.zeros(0, dtype=float)
            self.cum = np.zeros(self.n, dtype=float)
            self.total = 0.0

    def project(self, x: float, y: float) -> dict:
        """返回 dict(idx, s, lat, dist, heading)，无路线时全 0。"""
        empty = {"idx": -1, "s": 0.0, "lat": 0.0, "dist": 0.0, "heading": 0.0}
        if self.n < 2:
            return empty

        d2 = (self.pts[:, 0] - x) ** 2 + (self.pts[:, 1] - y) ** 2
        near = int(np.argmin(d2))

        best = None
        for k in (near - 1, near):
            if k < 0 or k + 1 >= self.n:
                continue
            length = float(self.seg_len[k])
            if length < 1e-9:
                continue
            ax, ay = float(self.pts[k, 0]), float(self.pts[k, 1])
            bx, by = float(self.pts[k + 1, 0]), float(self.pts[k + 1, 1])
            ux, uy = bx - ax, by - ay
            t = ((x - ax) * ux + (y - ay) * uy) / (length * length)
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            px, py = ax + t * ux, ay + t * uy
            dist = math.hypot(x - px, y - py)
            if best is not None and dist >= best["dist"]:
                continue
            # 二维叉积：>0 表示车在前进方向的左侧
            cross = ux * (y - ay) - uy * (x - ax)
            best = {
                "idx": k,
                "s": float(self.cum[k]) + t * length,
                "lat": cross / length,
                "dist": dist,
                "heading": math.degrees(math.atan2(uy, ux)),
            }
        return best if best is not None else empty

    def s_of(self, x: float, y: float) -> float:
        return self.project(x, y)["s"]


# --------------------------------------------------------------------------- #
# 静态数据导出
# --------------------------------------------------------------------------- #
def write_rows(path: Path, header: list[str], rows) -> int:
    count = 0
    with path.open("w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        for row in rows:
            writer.writerow(row)
            count += 1
    return count


def export_static(out_dir: Path, static, projector: RouteProjector) -> dict:
    """把静态数据摊成几个 csv，返回各文件行数。"""
    raw = static.model_dump(mode="json", by_alias=True)
    (out_dir / "static.json").write_text(
        json.dumps(raw, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    counts: dict[str, int] = {}

    counts["route.csv"] = write_rows(
        out_dir / "route.csv",
        ["idx", "x", "y", "z", "s"],
        [
            [i, f3(p.x, 4), f3(p.y, 4), f3(p.z, 4), f3(float(projector.cum[i]), 4)]
            for i, p in enumerate(static.route)
        ],
    )

    sub_rows = []
    sub_ranges = {}
    for i, sub in enumerate(static.sub_scenes):
        if sub.start_point is not None and projector.n >= 2:
            s0 = projector.s_of(sub.start_point.x, sub.start_point.y)
        else:
            s0 = 0.0
        if sub.end_point is not None and projector.n >= 2:
            s1 = projector.s_of(sub.end_point.x, sub.end_point.y)
        else:
            s1 = projector.total
        sub_ranges[sub.name] = (i, s0, s1)

        def coord(point, attr):
            return f3(getattr(point, attr), 4) if point is not None else ""

        sub_rows.append([
            i, sub.name,
            coord(sub.start_point, "x"), coord(sub.start_point, "y"), coord(sub.start_point, "z"),
            coord(sub.end_point, "x"), coord(sub.end_point, "y"), coord(sub.end_point, "z"),
            f3(s0, 3), f3(s1, 3), f3(max(0.0, s1 - s0), 3),
        ])
    counts["subscenes.csv"] = write_rows(
        out_dir / "subscenes.csv",
        ["idx", "name", "start_x", "start_y", "start_z",
         "end_x", "end_y", "end_z", "s_start", "s_end", "s_len"],
        sub_rows,
    )

    road_rows, lane_rows, border_rows = [], [], []
    for road in static.roads:
        stop_line = getattr(road, "stop_line", []) or []
        road_rows.append([
            road.id, road.driving_type.name, road.traffic_sign_type.name,
            f3(road.begin_pos.x, 3), f3(road.begin_pos.y, 3),
            f3(road.end_pos.x, 3), f3(road.end_pos.y, 3),
            len(road.lanes), "|".join(lane.id for lane in road.lanes),
            "|".join(road.predecessor_ids), "|".join(road.successor_ids),
            len(stop_line),
        ])
        for lane in road.lanes:
            lane_rows.append([
                road.id, lane.id, f3(lane.width, 3),
                lane.left_lane_id, lane.right_lane_id,
                len(lane.path_points),
            ])
            for side, border in (("left", lane.left_border), ("right", lane.right_border)):
                for seq, pt in enumerate(border.path_points):
                    border_rows.append([
                        road.id, lane.id, side,
                        border.type.value, border.type.name, seq,
                        f3(pt.x, 4), f3(pt.y, 4),
                    ])

    counts["roads.csv"] = write_rows(
        out_dir / "roads.csv",
        ["road_id", "driving_type", "traffic_sign", "begin_x", "begin_y",
         "end_x", "end_y", "n_lanes", "lane_ids", "predecessor", "successor",
         "n_stop_line"],
        road_rows,
    )
    counts["lanes.csv"] = write_rows(
        out_dir / "lanes.csv",
        ["road_id", "lane_id", "width", "left_lane", "right_lane", "n_center_pts"],
        lane_rows,
    )
    counts["borders.csv"] = write_rows(
        out_dir / "borders.csv",
        ["road_id", "lane_id", "side", "line_type", "line_type_name", "seq", "x", "y"],
        border_rows,
    )
    return {"counts": counts, "sub_ranges": sub_ranges}


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #
PARK_GEARS = {"drive": GearMode.DRIVE,
              "reverse": GearMode.REVERSE,
              "parking": GearMode.PARKING}

CAR_HALF_WIDTH = 0.90      # 车宽 1.8 m 的一半
LINE_MARGIN = 0.25         # 贴线时额外留的余量


def midline_points(static) -> np.ndarray:
    """所有中心线（borderType=1，双黄线）的采样点。

    park 模式要"靠近双黄线但不要压线"，得先知道线在哪。
    """
    pts: list[tuple[float, float]] = []
    for road in static.roads:
        for lane in road.lanes:
            for border in (lane.left_border, lane.right_border):
                kind = getattr(border.type, "value", border.type)
                try:
                    is_middle = int(kind) == 1
                except (TypeError, ValueError):
                    is_middle = False
                if is_middle:
                    pts.extend((float(p.x), float(p.y)) for p in border.path_points)
    return np.array(pts, dtype=float) if pts else np.zeros((0, 2), dtype=float)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="连续场景数据采集")
    parser.add_argument("--mode", choices=["hold", "traj", "route", "park"], default="hold",
                        help="hold=驻车只采几何（默认）；traj=沿建议轨迹低速跑；route=沿静态路线跑；"
                             "park=跟车+泊车元素内自动变道并倒车入库")
    parser.add_argument("--speed", type=float, default=3.0, help="traj/route 模式的目标速度 m/s")
    parser.add_argument("--vmax", type=float, default=0.0,
                        help="按曲率剖面跑时的直道速度上限 m/s（>0 时接管 --speed）")
    parser.add_argument("--alat", type=float, default=2.5, help="剖面用横向加速度上限 m/s²")
    parser.add_argument("--atan", type=float, default=1.5, help="剖面用纵向加减速上限 m/s²")
    parser.add_argument("--throttle-cap", type=float, default=0.40,
                        help="油门上限（0.40 约 17 km/h，跑高速要放到 0.85 以上）")
    parser.add_argument("--lookahead", type=float, default=LOOKAHEAD_M, help="纯跟踪前视距离 m")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="采集多少仿真秒后停（0=一直采到场景结束）")
    parser.add_argument("--skip-after", type=float, default=0.0,
                        help="hold 模式下每个元素停留多少秒后跳关（0=不跳）")
    parser.add_argument("--kick", action="store_true",
                        help="允许升挡 kick：低挡位下油门再大也卡在 17.2 km/h，开这个才会升挡")
    parser.add_argument("--kick-throttle", type=float, default=0.45,
                        help="升挡 kick 的油门（1.0 会把车甩到 14 m/s，实测）")
    parser.add_argument("--kick-time", type=float, default=0.4, help="升挡 kick 持续秒数")
    parser.add_argument("--park-probe", action="store_true",
                        help="进入泊车元素后降到 --park-speed，并在终点前 --park-stop-m 驻车，"
                             "把车位附近障碍物完整写进 obstacles.jsonl")
    parser.add_argument("--park-speed", type=float, default=2.0, help="泊车元素内的限速 m/s")
    parser.add_argument("--park-stop-m", type=float, default=8.0,
                        help="在泊车元素终点前多少米驻车（正值=提前停）")
    parser.add_argument("--park-shift", type=float, default=1.0,
                        help="park 模式进车位前先往路的另一侧借位多少米（会被双黄线余量卡住）")
    parser.add_argument("--park-engage", type=float, default=30.0,
                        help="看到车位且对位点距离小于该值时接管倒车入库（0=一看到就接管）")
    parser.add_argument("--park-hold", type=float, default=8.0,
                        help="入库后保持驻车秒数（平台要 5 s，多留 3 s 余量）")
    parser.add_argument("--park-slow-radius", type=float, default=20.0,
                        help="离车位/元素终点多近才开始把目标速度压到 --park-speed")
    parser.add_argument("--detail", action="store_true", help="额外写逐帧完整消息 frames_detail.jsonl")
    parser.add_argument("--out", type=Path, default=None, help="输出目录（默认 logs\\collect\\<时间戳>）")
    parser.add_argument("--max-wall", type=float, default=3600.0, help="墙钟保护上限 s")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stamp = time.strftime("%Y%m%d-%H%M%S")
    out_dir = Path(args.out) if args.out else REPO / "logs" / "collect" / stamp
    out_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 72, flush=True)
    print(f"模式 {args.mode}"
          + (f"  目标速度 {args.speed:.1f} m/s" if args.mode != "hold" else "  车辆保持驻车")
          + (f"  每元素 {args.skip_after:.0f}s 后跳关" if args.skip_after > 0 else ""), flush=True)
    print(f"输出 {out_dir}", flush=True)
    print("等待仿真场景连接（现在去客户端点「启动场景」，这里会一直等）...", flush=True)
    print("=" * 72, flush=True)

    api = SceneAPI()
    api.connect()

    static = api.get_scene_static_data()
    projector = RouteProjector(static.route)
    exported = export_static(out_dir, static, projector)
    sub_ranges = exported["sub_ranges"]

    print(f"已连接。赛道 {projector.n} 点 / {projector.total:.1f} m，"
          f"道路 {len(static.roads)} 条，元素 {len(static.sub_scenes)} 个，"
          f"VLA={'是' if static.vla_extension is not None else '否'}", flush=True)
    for i, sub in enumerate(static.sub_scenes):
        _, s0, s1 = sub_ranges[sub.name]
        print(f"  {i + 1}. {sub.name:<12s} s={s0:7.1f} -> {s1:7.1f}  ({max(0.0, s1 - s0):6.1f} m)",
              flush=True)
    print("  静态数据: " + ", ".join(f"{k}({v})" for k, v in exported["counts"].items()), flush=True)

    profile = None
    if args.vmax > 0.0 and args.mode != "hold" and projector.n >= 3:
        profile = SpeedProfile(static.route, v_max=args.vmax,
                               a_lat=args.alat, a_tan=args.atan)
        vmin = min(profile.v)
        est = sum((profile.s[i] - profile.s[i - 1]) / max(profile.v[i], 0.3)
                  for i in range(1, len(profile.v)))
        print(f"  曲率剖面: v_max={args.vmax:.1f} a_lat={args.alat:.1f} a_tan={args.atan:.1f}"
              f" -> 最低限速 {vmin:.1f} m/s，按剖面理论全程 {est:.1f} s", flush=True)

    frames_path = out_dir / "frames.csv"
    frames_fh = frames_path.open("w", newline="", encoding="utf-8-sig")
    frames_csv = csv.DictWriter(frames_fh, fieldnames=FRAME_FIELDS, extrasaction="ignore")
    frames_csv.writeheader()

    events_fh = (out_dir / "events.jsonl").open("w", encoding="utf-8")
    detail_fh = (out_dir / "frames_detail.jsonl").open("w", encoding="utf-8") if args.detail else None
    obs_fh = ((out_dir / "obstacles.jsonl").open("w", encoding="utf-8")
              if args.park_probe else None)

    def emit(kind: str, **payload) -> None:
        events_fh.write(json.dumps(
            {"wall_t": round(time.time() - t0, 3), "kind": kind, **payload},
            ensure_ascii=False) + "\n")
        events_fh.flush()

    t0 = time.time()
    emit("connect", route_points=projector.n, route_len=round(projector.total, 2),
         sub_scenes=[sub.name for sub in static.sub_scenes], mode=args.mode)

    speed_ctl = SpeedController(v_target=args.speed,
                                accel_limit=args.atan if profile else 1.5,
                                throttle_max=args.throttle_cap,
                                kick_throttle=args.kick_throttle,
                                kick_time=args.kick_time)
    speed_ctl.reset(0.0)
    pursuit = PurePursuit(static.route, lookahead=args.lookahead) if projector.n >= 2 else None

    frame_no = 0
    prev_sim: float | None = None
    prev_wall = t0
    prev_v = 0.0
    accel_f = 0.0
    last_xy: tuple[float, float] | None = None
    travelled = 0.0
    max_speed = 0.0
    max_accel = 0.0
    lat_abs_sum = 0.0
    lat_abs_max = 0.0
    sim_t = 0.0          # 全局仿真时钟（累加，不随元素重开归零）
    elem_start = 0.0     # 当前元素开始的 sim_t
    clean_since = 0.0    # 元素重开后到这一刻之前的帧不计入极值统计

    visited: list[dict] = []
    current_sub: str | None = None
    sub_first_t: dict[str, float] = {}
    skipped: set[str] = set()
    prev_obstacles: set[tuple[int, int]] = set()
    prev_lights: dict[str, str] = {}
    last_print = 0.0
    last_speed_warn = -99.0
    last_accel_warn = -99.0
    stop_reason = "场景结束"
    detail_route_warned = False

    limiter = OutputLimiter()
    limiter.reset(t0)
    park: ReversePark | None = None
    last_park_phase: str | None = None
    mid_pts = midline_points(static) if args.mode == "park" else np.zeros((0, 2), dtype=float)

    def mid_clear(x: float, y: float) -> float:
        """到最近双黄线采样点的距离；没有中心线时给个大数。"""
        if mid_pts.size == 0:
            return 99.0
        return float(np.hypot(mid_pts[:, 0] - x, mid_pts[:, 1] - y).min())


    try:
        for msg, camera_frames in api.main_loop():
            frame_no += 1
            now = time.time()
            status = msg.scene_status
            car = msg.main_vehicle
            pose = msg.pose_gnss

            # scene_status.used_time 是「当前元素内已用时间」：元素切换、元素重开（超时没跑完
            # 会被平台拉回起点重来）都会归零，所以它不能当全局时钟。
            # sim_t 由逐帧增量累加；elem_t 才是 used_time 的原始值。
            elem_t = float(status.used_time)
            dropped = prev_sim is not None and elem_t < prev_sim - 1.0
            if prev_sim is None:
                dt_sim = 0.0
            elif dropped:
                dt_sim = elem_t               # 归零那一下，把新的 used_time 整段算进来
            else:
                dt_sim = max(0.0, elem_t - prev_sim)
            sim_t += dt_sim
            dt_wall = max(0.0, now - prev_wall)
            prev_sim, prev_wall = elem_t, now
            limit = float(status.time_limit)
            if frame_no == 1:
                emit("first_frame", sim_t=round(sim_t, 3), sub_scene=status.sub_scene_name,
                     x=round(pose.pos_x, 3), y=round(pose.pos_y, 3),
                     yaw=round(pose.ori_z, 3), cameras=[frame.id for frame in camera_frames])

            # ---- 元素切换 / 元素重开 ----
            step = 0.0
            if last_xy is not None:
                step = math.hypot(pose.pos_x - last_xy[0], pose.pos_y - last_xy[1])
            restart = status.sub_scene_name == current_sub and (dropped or step > 20.0)
            if status.sub_scene_name != current_sub:
                if visited:
                    visited[-1]["t1"] = round(sim_t, 2)
                visited.append({"name": status.sub_scene_name, "t0": round(sim_t, 2),
                                "t1": None, "limit": round(limit, 1), "restarts": 0})
                current_sub = status.sub_scene_name
                elem_start = sim_t
                sub_first_t.setdefault(current_sub, sim_t)
                emit("sub_scene", name=current_sub, sim_t=round(sim_t, 3),
                     s=round(projector.project(pose.pos_x, pose.pos_y)["s"], 2),
                     time_limit=round(limit, 2))
            elif restart:
                if visited:
                    visited[-1]["t1"] = round(sim_t, 2)
                    visited[-1]["restarts"] = visited[-1].get("restarts", 0) + 1
                visited.append({"name": current_sub, "t0": round(sim_t, 2), "t1": None,
                                "limit": round(limit, 1), "restarts": 0})
                elem_start = sim_t
                clean_since = sim_t + 2.0
                emit("restart", sub_scene=current_sub, sim_t=round(sim_t, 3),
                     elem_t=round(elem_t, 2), limit=round(limit, 2),
                     jump_m=round(step, 1),
                     s=round(projector.project(pose.pos_x, pose.pos_y)["s"], 2))

            # ---- 里程与横向偏差 ----
            proj = projector.project(pose.pos_x, pose.pos_y)
            heading_err = wrap_deg(proj["heading"] - (-pose.ori_z))
            lat_abs_sum += abs(proj["lat"])
            lat_abs_max = max(lat_abs_max, abs(proj["lat"]))

            # ---- 加速度 ----
            speed = float(car.speed)
            if restart:
                # 平台把车拉回起点重开，速度是它塞进来的，不能当成我们的加减速
                speed_ctl.reset(speed)
                accel_f = 0.0
                prev_v = speed
                a_raw = 0.0
            else:
                a_raw = (speed - prev_v) / dt_sim if dt_sim > 1e-4 else 0.0
                a_raw = max(-20.0, min(20.0, a_raw))
                accel_f += 0.25 * (a_raw - accel_f)
                prev_v = speed
            if sim_t >= clean_since:
                max_speed = max(max_speed, abs(speed))
                max_accel = max(max_accel, abs(accel_f))

            # 位移跳变（元素重开、跳关）不计入里程
            if last_xy is not None and step <= 20.0:
                travelled += step
            last_xy = (pose.pos_x, pose.pos_y)

            # ---- 控制 ----
            cmd_throttle = cmd_brake = cmd_steer = 0.0
            cmd_gear = GearMode.PARKING
            v_tgt = args.speed
            park_hold = False
            shift_used = 0.0
            in_park_elem = bool(current_sub and "泊车" in current_sub)
            if args.mode == "hold":
                vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
                cmd_brake = 1.0
            elif park is not None:
                # 倒车入库接管：开到位、摆正、倒进去、驻车保持
                gear_name, thr, brk, steer = park.update(
                    pose.pos_x, pose.pos_y, pose.ori_z, speed, now)
                cmd_gear = PARK_GEARS.get(gear_name, GearMode.PARKING)
                cmd_steer = steer
                if cmd_gear == GearMode.PARKING:
                    cmd_throttle, cmd_brake = 0.0, max(brk, 0.5)
                else:
                    cmd_throttle, cmd_brake = limiter.apply(thr, brk, now)
                if park.phase != last_park_phase:
                    last_park_phase = park.phase
                    emit("park_phase", phase=park.phase, sim_t=round(sim_t, 3),
                         d_stage=round(park.d_stage, 2), a_long=round(park.a_long, 2),
                         a_lat=round(park.a_lat, 2),
                         goal_dist=(round(park.goal_dist, 2)
                                    if park.goal_dist < 1e9 else None))
                vc = VehicleControl(gear=cmd_gear, throttle=cmd_throttle,
                                    brake=cmd_brake, steering=cmd_steer)
            else:
                points = None
                if args.mode in ("traj", "park") and msg.trajectory:
                    points = msg.trajectory
                # park 模式进车位前先往路的另一侧借位：把「虚拟本车」摆在真车右侧
                # shift 米处，纯跟踪就会把真车带到偏左 shift 米的位置。能借多少由
                # 双黄线的实际距离决定，不会压线。
                off_x = off_y = 0.0
                slot_now = find_slot(msg.obstacles) if (args.mode == "park" and in_park_elem) else None
                # 平台的「停车检查点」：静态元素终点。泊车元素不把车开到这个点上
                # 就不得分（实测停在车位中心、离它 0.40 m，200 分直接没了）。
                park_goal = None
                if in_park_elem:
                    for sub in static.sub_scenes:
                        if sub.name == current_sub and sub.end_point is not None:
                            park_goal = (float(sub.end_point.x), float(sub.end_point.y))
                            break
                if slot_now is not None and args.park_shift > 0 and proj["idx"] >= 1:
                    i = proj["idx"]
                    j = min(i + 1, projector.n - 1)
                    tx = projector.pts[j, 0] - projector.pts[i, 0]
                    ty = projector.pts[j, 1] - projector.pts[i, 1]
                    norm = math.hypot(tx, ty) or 1.0
                    lx, ly = -ty / norm, tx / norm        # 路线左法向
                    # 对位点在车的哪一侧，就往哪一侧借位；借多少看线/余量
                    probe = ReversePark(hold_seconds=0.0)
                    pinfo = probe.begin(pose.pos_x, pose.pos_y, pose.ori_z, slot_now, now)
                    toward = ((pinfo["stage_x"] - pose.pos_x) * lx
                              + (pinfo["stage_y"] - pose.pos_y) * ly)
                    if toward >= 0.0:
                        room = mid_clear(pose.pos_x, pose.pos_y) - CAR_HALF_WIDTH - LINE_MARGIN
                    else:
                        room = 1.5      # 往泊车侧没有线可卡，只给保守上限
                    shift_used = max(0.0, min(args.park_shift, room))
                    if toward < 0.0:
                        shift_used = -shift_used
                    off_x, off_y = -shift_used * lx, -shift_used * ly
                if points is not None:
                    tracker = PurePursuit(points, lookahead=args.lookahead)
                    cmd_steer = tracker.steering(
                        pose.pos_x + off_x, pose.pos_y + off_y, pose.ori_z)
                elif pursuit is not None:
                    cmd_steer = pursuit.steering(
                        pose.pos_x + off_x, pose.pos_y + off_y, pose.ori_z)
                elif args.mode == "traj" and not detail_route_warned:
                    detail_route_warned = True
                    emit("warning", what="建议轨迹为空，退回静态 route")
                dt_ctl = dt_sim if dt_sim > 1e-4 else (dt_wall if dt_wall > 1e-4 else 0.2)
                if profile is not None:
                    v_tgt = profile.target(proj["idx"], speed)
                # 泊车探针：先降速，到元素终点前 park_stop_m 直接驻车，
                # 让我们能在不撞车的前提下把车位附近的障碍物看全。
                if args.park_probe and in_park_elem:
                    park_end_s = sub_ranges.get(current_sub, (-1, 0.0, 0.0))[2]
                    if proj["s"] < park_end_s - args.park_stop_m:
                        v_tgt = min(v_tgt, args.park_speed)
                    else:
                        park_hold = True
                if args.mode == "park" and in_park_elem:
                    # 只在「已经贴近车位/终点」时压速。以前是从元素起点就压到 2 m/s，
                    # 180 m 的进近段全程爬行（实测垂直泊车元素 121 s / 200 s）：车既没有
                    # 加速段，也就谈不上「先加速再减速」。进近段的正常减速交给曲率剖面和
                    # 倒车入库的 approach（d_stage<=park_engage 时接管，自带距离减速）去做。
                    park_end_s = sub_ranges.get(current_sub, (-1, 0.0, 0.0))[2]
                    slot_dist = (float("inf") if slot_now is None else
                                 math.hypot(slot_now.pos_x - pose.pos_x,
                                            slot_now.pos_y - pose.pos_y))
                    if slot_dist < args.park_slow_radius or proj["s"] > park_end_s - args.park_slow_radius:
                        v_tgt = min(v_tgt, args.park_speed)
                speed_ctl.v_target = v_tgt
                cmd_throttle, cmd_brake = speed_ctl.update(
                    speed, dt_ctl, a_now=accel_f, allow_kick=args.kick)
                cmd_throttle, cmd_brake = limiter.apply(cmd_throttle, cmd_brake, now)
                cmd_gear = GearMode.DRIVE
                vc = VehicleControl(
                    gear=cmd_gear, throttle=cmd_throttle,
                    brake=cmd_brake, steering=cmd_steer,
                )
                # 看到车位、且到了对位点附近，就把控制权交给倒车入库
                if slot_now is not None and park is None and not park_hold:
                    slot = slot_now
                    if slot is not None:
                        candidate = ReversePark(hold_seconds=args.park_hold,
                                                goal=park_goal)
                        info0 = candidate.begin(
                            pose.pos_x, pose.pos_y, pose.ori_z, slot, now)
                        near = args.park_engage <= 0 or info0["d_stage"] <= args.park_engage
                        if near:
                            park = candidate
                            last_park_phase = park.phase
                            emit("park_begin", sim_t=round(sim_t, 3),
                                 slot_x=round(float(slot.pos_x), 2),
                                 slot_y=round(float(slot.pos_y), 2),
                                 slot_ori=round(float(getattr(slot, "ori_z", 0.0)), 1),
                                 stage_x=round(info0["stage_x"], 2),
                                 stage_y=round(info0["stage_y"], 2),
                                 d_stage=round(info0["d_stage"], 2),
                                 goal_x=round(park_goal[0], 2) if park_goal else None,
                                 goal_y=round(park_goal[1], 2) if park_goal else None,
                                 shift=round(shift_used, 2))
            if park_hold:
                cmd_throttle = 0.0
                cmd_brake = 1.0
                cmd_steer = 0.0
                cmd_gear = GearMode.PARKING
                vc = VehicleControl(gear=GearMode.PARKING, brake=1.0)
            api.set_vehicle_control(vc)

            # ---- 跳关 ----
            if args.skip_after > 0 and args.mode == "hold" and current_sub not in skipped:
                if sim_t - sub_first_t.get(current_sub, sim_t) >= args.skip_after:
                    skipped.add(current_sub)
                    api.skip_level()
                    emit("skip_level", sub_scene=current_sub, sim_t=round(sim_t, 3))

            # ---- 障碍物 / 红绿灯 ----
            nearest_dist, nearest_id, nearest_type = float("inf"), -1, ""
            dyn_count = 0
            obstacle_keys: set[tuple[int, int]] = set()
            for obs in msg.obstacles:
                obstacle_keys.add((int(obs.id), int(obs.type.value)))
                if abs(obs.vel_x) + abs(obs.vel_y) > 0.3:
                    dyn_count += 1
                dist = math.hypot(obs.pos_x - pose.pos_x, obs.pos_y - pose.pos_y)
                if dist < nearest_dist:
                    nearest_dist, nearest_id, nearest_type = dist, int(obs.id), obs.type.name
            if obstacle_keys != prev_obstacles:
                added = sorted(obstacle_keys - prev_obstacles)
                removed = sorted(prev_obstacles - obstacle_keys)
                if added or removed:
                    emit("obstacles", sim_t=round(sim_t, 3),
                         added=[[i, t] for i, t in added][:40],
                         removed=[[i, t] for i, t in removed][:40],
                         total=len(obstacle_keys))
                prev_obstacles = obstacle_keys

            if obs_fh is not None and current_sub and "泊车" in current_sub:
                obs_fh.write(json.dumps({
                    "sim_t": round(sim_t, 3),
                    "ego": [round(pose.pos_x, 3), round(pose.pos_y, 3), round(pose.ori_z, 3),
                            round(speed, 3)],
                    "obstacles": [{
                        "id": int(o.id), "type": o.type.name, "type_id": int(o.type.value),
                        "x": round(o.pos_x, 3), "y": round(o.pos_y, 3), "z": round(o.pos_z, 3),
                        "ori_z": round(o.ori_z, 3),
                        "vx": round(o.vel_x, 3), "vy": round(o.vel_y, 3),
                        "len": round(o.length, 3), "wid": round(o.width, 3),
                        "hgt": round(o.height, 3), "extra": o.extra_info,
                    } for o in msg.obstacles],
                }, ensure_ascii=False) + "\n")
                obs_fh.flush()

            light_state: dict[str, str] = {}
            for group in msg.traffic_light_groups:
                for light in group.traffic_lights:
                    light_state[f"{group.id}/{light.id}"] = (
                        f"{light.straight_state.name[:1]}"
                        f"{light.left_state.name[:1]}"
                        f"{light.right_state.name[:1]}"
                        f"@{light.straight_remaining_time:.0f}"
                    )
            if light_state != prev_lights:
                changed = {
                    key: value for key, value in light_state.items()
                    if prev_lights.get(key) != value
                }
                if changed:
                    emit("traffic_light", sim_t=round(sim_t, 3), changed=changed)
                prev_lights = light_state

            # ---- 违规预警（只记事件，不干预）----
            if speed * 3.6 > LIMIT_SCENE_ELEMENT_KMH and sim_t - last_speed_warn > 2.0:
                last_speed_warn = sim_t
                emit("overspeed", sim_t=round(sim_t, 3), speed_kmh=round(speed * 3.6, 1))
            if abs(accel_f) > ACCEL_BONUS_LIMIT and sim_t - last_accel_warn > 2.0:
                last_accel_warn = sim_t
                emit("harsh_accel", sim_t=round(sim_t, 3), accel=round(accel_f, 2))

            # ---- 落盘 ----
            sub_idx, s0, s1 = sub_ranges.get(current_sub, (-1, 0.0, 0.0))
            progress = (proj["s"] - s0) / (s1 - s0) if s1 > s0 else -1.0
            sub_end = None
            for sub in static.sub_scenes:
                if sub.name == current_sub:
                    sub_end = sub.end_point
                    break

            frames_csv.writerow({
                "frame": frame_no,
                "wall_t": f3(now - t0, 2),
                "sim_t": f3(sim_t, 3),
                "dt_sim": f3(dt_sim, 4),
                "elem_t": f3(elem_t, 3),
                "elem_limit": f3(limit, 1),
                "elem_remain": f3(max(0.0, limit - elem_t), 2),
                "sub_scene": current_sub,
                "sub_idx": sub_idx,
                "sub_progress": f3(progress, 4) if progress >= 0 else "",
                "restart": 1 if restart else 0,
                "x": f3(pose.pos_x, 3), "y": f3(pose.pos_y, 3), "z": f3(pose.pos_z, 3),
                "yaw_deg": f3(pose.ori_z, 2),
                "vx": f3(pose.vel_x, 3), "vy": f3(pose.vel_y, 3), "vz": f3(pose.vel_z, 3),
                "speed": f3(speed, 3), "speed_kmh": f3(speed * 3.6, 2),
                "v_tgt": f3(v_tgt, 2),
                "accel": f3(accel_f, 3), "accel_raw": f3(a_raw, 3),
                "gear": car.gear.name, "throttle": f3(car.throttle, 2),
                "brake": f3(car.brake, 2), "steering": f3(car.steering, 3),
                "cmd_gear": cmd_gear.name, "cmd_throttle": f3(cmd_throttle, 2),
                "cmd_brake": f3(cmd_brake, 2), "cmd_steer": f3(cmd_steer, 3),
                "route_idx": proj["idx"], "route_s": f3(proj["s"], 2),
                "route_len": f3(projector.total, 2),
                "lat_err": f3(proj["lat"], 3), "head_err": f3(heading_err, 1),
                "dist_route_end": f3(max(0.0, projector.total - proj["s"]), 2),
                "dist_sub_end": (
                    f3(math.hypot(sub_end.x - pose.pos_x, sub_end.y - pose.pos_y), 2)
                    if sub_end is not None else ""
                ),
                "obs_count": len(msg.obstacles), "obs_dyn_count": dyn_count,
                "obs_min_dist": f3(nearest_dist, 2) if nearest_dist < float("inf") else "",
                "obs_min_id": nearest_id if nearest_id >= 0 else "",
                "obs_min_type": nearest_type,
                "tl_count": len(msg.traffic_light_groups),
            })
            if frame_no % 10 == 0:
                frames_fh.flush()

            if detail_fh is not None:
                detail_fh.write(json.dumps(
                    {"frame": frame_no, "wall_t": round(now - t0, 3),
                     "msg": msg.model_dump(mode="json", by_alias=True)},
                    ensure_ascii=False) + "\n")

            # ---- 进度条 ----
            if sim_t - last_print >= 1.0:
                last_print = sim_t
                live = out_dir / "live.txt"
                line = (f"[{sim_t:6.1f}s] {current_sub or '-':<10s} "
                        f"v={speed:5.2f}({speed * 3.6:5.1f}) a={accel_f:+5.2f} "
                        f"s={proj['s']:6.1f}/{projector.total:6.1f} "
                        f"横={proj['lat']:+5.2f} 障碍={len(msg.obstacles)} "
                        f"最近={nearest_dist:5.1f}m "
                        f"元素{elem_t:5.1f}/{limit:5.0f}s"
                        + (f" 进度={progress * 100:4.0f}%" if progress >= 0 else ""))
                print(line, flush=True)
                try:
                    live.write_text(line + "\n", encoding="utf-8")
                except OSError:
                    pass

            if args.duration > 0 and sim_t - visited[0]["t0"] >= args.duration:
                stop_reason = f"到达 --duration {args.duration:.0f}s"
                break
            if now - t0 > args.max_wall:
                stop_reason = f"到达 --max-wall {args.max_wall:.0f}s"
                break
    except KeyboardInterrupt:
        stop_reason = "手动中断"
        print("\n收到 Ctrl-C。", flush=True)
    except Exception as exc:  # noqa: BLE001
        stop_reason = f"异常: {type(exc).__name__}: {exc}"
        emit("error", message=str(exc))
        print(f"\n{stop_reason}", flush=True)
    finally:
        if visited and visited[-1]["t1"] is None:
            visited[-1]["t1"] = round(sim_t, 2)
        try:
            api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
        except Exception:  # noqa: BLE001
            pass
        frames_fh.close()
        if detail_fh is not None:
            detail_fh.close()
        if obs_fh is not None:
            obs_fh.close()

    summary = {
        "stop_reason": stop_reason,
        "frames": frame_no,
        "sim_seconds": round(sim_t, 2),
        "wall_seconds": round(time.time() - t0, 2),
        "travelled_m": round(travelled, 2),
        "max_speed_ms": round(max_speed, 3),
        "max_accel_ms2": round(max_accel, 3),
        "lat_abs_mean": round(lat_abs_sum / frame_no, 4) if frame_no else 0.0,
        "lat_abs_max": round(lat_abs_max, 3),
        "restarts": sum(item.get("restarts", 0) for item in visited),
        "visited": visited,
        "skipped": sorted(skipped),
        "output": str(out_dir),
    }
    emit("end", **summary)

    meta = {
        "args": {key: (str(value) if isinstance(value, Path) else value)
                 for key, value in vars(args).items()},
        "git": git_info(REPO),
        "python": platform.python_version(),
        "platform": platform.platform(),
        "started": stamp,
        "summary": summary,
        "static": {
            "route_points": projector.n,
            "route_len": round(projector.total, 2),
            "roads": len(static.roads),
            "lanes": sum(len(road.lanes) for road in static.roads),
            "sub_scenes": [sub.name for sub in static.sub_scenes],
            "vla": static.vla_extension is not None,
            "exported": exported["counts"],
        },
    }
    (out_dir / "meta.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")

    print("\n" + "=" * 72, flush=True)
    print(f"采集结束：{stop_reason}", flush=True)
    print(f"帧数 {frame_no}  仿真 {sim_t:.1f}s  墙钟 {time.time() - t0:.1f}s", flush=True)
    print(f"里程 {travelled:.1f} m  最高速度 {max_speed:.2f} m/s  "
          f"最大 |a| {max_accel:.2f} m/s²", flush=True)
    print("元素：" + " -> ".join(
        f"{item['name']}({item['t0']}~{item['t1']}s"
        + (f",限{item.get('limit', 0):.0f}s" if item.get("limit") else "")
        + (f",重开{item['restarts']}次" if item.get("restarts") else "")
        + ")"
        for item in visited), flush=True)
    print(f"数据在 {out_dir}", flush=True)
    print("=" * 72, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

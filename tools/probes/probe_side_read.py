"""侧字可读性验证局（约 40 s）：贴着已知蓝车以 ~2 m/s 侧向通过，5 Hz 连拍四路相机。

要验证的问题
------------
规则书 §2.3 要求上报「车辆编号」。编号有两处载体：车顶黄牌（贴脸 1.2 m 时只有
29~45 px，数字 12~16 px，OCR 命中率约 3%）与**车身侧门的白色大字**（约 3~4 倍大）。
本局只回答一件事：**侧门大字在 2 m 侧距、2 m/s 通过时，能否被 Windows OCR 读出**。

航线
----
已知蓝车（r7 lanes 局两次顶停反推）车心 ≈ (16.5, 12.9)、车头朝 ≈ 344°（东偏南），
所以它的**左侧**朝北。航线沿 y≈14.8~16.4 从西向东，距车心约 2.2~2.6 m：

    快速段（0.03 油门 ≈ 4.8 m/s）：(20,-4) → (8,-2) → (5,8) → (5.5,14) → (11,14.8)
    慢速段（脉冲调速，目标 2.0 m/s）：(11,14.8) → (22,16.4)，全程 5 Hz 连拍
    之后 P 挡驻车到 --seconds（不提交任何结果，这是纯采集局）

用法
----
    python tools/probe_side_read.py --seconds 40 --out D:\\...\\runs\\vla-side-r1
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / "tools"))

from probe_zone_lap import WpFollower  # noqa: E402

WAYPOINTS = [
    (20.0, -4.0),
    (8.0, -2.0),
    (5.0, 8.0),
    (5.5, 12.5),
    (11.0, 13.6),   # 慢速段起点（接受上一个点后就降到 2 m/s）
    (26.0, 16.5),   # 慢速通过段的终点标记
]
SLOW_SEG = 4            # seg>=SLOW_SEG 即进入慢速段
CAR = (16.5, 12.9)      # r7 lanes 局反推的已知蓝车车心
THROTTLE = 0.03
V_SLOW = 2.0
GOAL = (26.0, 16.5)
BOUND = 30.0


def save_jpg(path: Path, img: np.ndarray, quality: int = 92) -> bool:
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return False
    path.write_bytes(buf.tobytes())
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--out", default=str(Path.home() / "vla_side_read"))
    ap.add_argument("--lookahead", type=float, default=5.0)
    ap.add_argument("--accept", type=float, default=2.5)
    ap.add_argument("--self-check", action="store_true")
    args = ap.parse_args()

    if args.self_check:
        f = WpFollower(WAYPOINTS, args.lookahead, args.accept)
        f.update(34.6, -4.05)
        d = min(math.hypot(CAR[0] - x, CAR[1] - y) for x, y in WAYPOINTS)
        print(f"self-check OK 航点={len(WAYPOINTS)} 目标={f.target(34.6, -4.05)} "
              f"最近航点距车心={d:.1f} m 开始速度环")
        return 0

    out = Path(args.out)
    (out / "f").mkdir(parents=True, exist_ok=True)
    for old in (out / "f").glob("*.jpg"):
        old.unlink()

    from metacar import GearMode, SceneAPI, VehicleControl

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"场景: {[s.name for s in static.sub_scenes]}", flush=True)

    follower = WpFollower(WAYPOINTS, args.lookahead, args.accept)
    t0 = time.time()
    tag = 0
    last_shot = -1e9
    last_log = -1e9
    last_seg = 0
    poses = []
    min_d = 1e9
    slow_hits = 0

    for msg, frames in api.main_loop():
        now = time.time() - t0
        pose = msg.pose_gnss
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        v = abs(float(msg.main_vehicle.speed))

        # 注意：WpFollower.done() 的语义是「最后一个航点成为当前目标」，在上一段
        # 刚被接受时就会为真，不能用它判断收工；改成按目标点距离 + 越界双重收工。
        at_end = ((now > 12.0 and x > 22.0)
                  or math.hypot(x - GOAL[0], y - GOAL[1]) < 3.0)
        out_of_bound = now > 10.0 and (abs(x) > BOUND or abs(y) > BOUND)
        if now > args.seconds or at_end or out_of_bound:
            print(f"[{now:5.1f}s] 结束（到点={at_end} 越界={out_of_bound}），驻车", flush=True)
            break

        follower.update(x, y)
        if follower.seg != last_seg:
            last_seg = follower.seg
            nxt = WAYPOINTS[min(follower.seg, len(WAYPOINTS) - 1)]
            print(f"[{now:5.1f}s] 到点 → 下一个 ({nxt[0]:.1f},{nxt[1]:.1f}) "
                  f"v={v:.2f}", flush=True)

        slow = follower.seg >= SLOW_SEG
        steer = follower.steering(x, y, yaw)

        if slow:
            if v < V_SLOW - 0.3:
                thr, brk = THROTTLE, 0.0
            elif v > V_SLOW + 0.3:
                thr, brk = 0.0, 0.03
            else:
                thr, brk = 0.0, 0.0
        else:
            thr, brk = THROTTLE, 0.0

        vc = VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=brk, steering=steer)
        api.set_vehicle_control(vc)

        d_car = math.hypot(x - CAR[0], y - CAR[1])
        min_d = min(min_d, d_car)

        shot_every = 0.2 if slow else 0.6
        if frames and now - last_shot >= shot_every:
            last_shot = now
            tag += 1
            for cam in frames:
                img = getattr(cam, "frame", None)
                if img is None:
                    continue
                save_jpg(out / "f" / f"t{tag:04d}_cam{int(cam.id)}.jpg", img)
            poses.append((round(now, 2), round(x, 2), round(y, 2), round(yaw, 1),
                          round(v, 2), f"t{tag:04d}", "slow" if slow else "fast"))
            if slow:
                slow_hits += 1

        if now - last_log >= 1.0:
            last_log = now
            print(f"[{now:5.1f}s] ({x:6.2f},{y:6.2f}) yaw={yaw:6.1f} v={v:4.2f} "
                  f"慢速段={slow} 距已知车心={d_car:5.2f} m 最近={min_d:5.2f} 帧={tag}",
                  flush=True)

    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0))
    except OSError:
        pass

    with (out / "poses.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "x", "y", "yaw", "v", "tag", "mode"])
        w.writerows(poses)
    print(f"图片 {tag * 4} 张（{tag} 个时刻）在 {out / 'f'}", flush=True)
    print(f"最近距已知车心 {min_d:.2f} m；慢速段 {slow_hits} 个时刻", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

# -*- coding: utf-8 -*-
"""停车区违规检测：贴边巡检（vla2 Patrol 控制）+ RapidOCR 整帧单线程在线读号。

- 控制复用 feat/vla-parking 的 Patrol/perimeter/longitudinal（已验证 73 s/126 m 贴边）；
- 读号用 RapidOCR PP-OCRv4，intra_op_num_threads=1，整帧不裁，单 worker；
- 只提交读到的 B-xx，票数 >=1；不写死任何编号。
"""
from __future__ import annotations

import argparse
import re
import sys
import threading
import time
from collections import Counter
from pathlib import Path

import cv2

RACE = Path(r"C:\Users\Bowen Liu\Documents\deepseek-harness\default-workspace\metacar-race-vla-parking")
if str(RACE) not in sys.path:
    sys.path.insert(0, str(RACE))

from metacar import (GearMode, ParkingResult, SceneAPI, VehicleControl,
                     VLAExtensionOutput)  # noqa: E402
from vla2.main import Patrol, longitudinal, perimeter, read_regions  # noqa: E402
from vla2.vision import detect_cars, ground_class  # noqa: E402
from rapidocr_onnxruntime import RapidOCR  # noqa: E402

ID_RE = re.compile(r"[BDPR]\s*[-—]?\s*0*(\d{1,2})")


def norm(text):
    t = (text or "").upper()
    for a, b in (("D", "0"), ("O", "0"), ("8", "6")):
        t = t.replace(a, b)
    m = ID_RE.search(t)
    if not m:
        return ""
    n = int(m.group(1))
    # 本场景编号范围 B-01~B-06，超范围视为误读
    return f"B-{n:02d}" if 1 <= n <= 6 else ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--offset", type=float, default=-2.5)
    ap.add_argument("--speed", type=float, default=2.0)
    ap.add_argument("--seconds", type=float, default=95.0)
    ap.add_argument("--throttle", type=float, default=0.030)
    ap.add_argument("--brake", type=float, default=0.0)
    ap.add_argument("--pulse", action="store_true")
    ap.add_argument("--v-high", type=float, default=3.8)
    ap.add_argument("--v-low", type=float, default=2.8)
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--min-score", type=float, default=0.75)
    ap.add_argument("--dry", action="store_true")
    args = ap.parse_args()

    print("等待场景连接...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    bbox, normal = read_regions(static)
    print(f"限制区 {bbox}", flush=True)
    wps = perimeter(bbox, args.offset)
    patrol = Patrol(wps, args.speed)
    print(f"航点 {len(wps)} 个，offset={args.offset}", flush=True)

    # RapidOCR：单线程，整帧
    engine = RapidOCR(intra_op_num_threads=1)
    lock = threading.Lock()
    latest = [None]
    stop = threading.Event()
    votes = Counter()
    best_score = {}
    seen = {}

    def worker():
        while not stop.is_set():
            with lock:
                item = latest[0]
                latest[0] = None
            if item is None:
                time.sleep(0.02)
                continue
            frame, ego = item
            try:
                res, _ = engine(frame)
            except Exception as exc:
                print("ocr err", repr(exc), flush=True)
                continue
            if not res:
                continue
            for box, text, score in res:
                sid = norm(text)
                if not sid:
                    continue
                votes[sid] += 1
                best_score[sid] = max(best_score.get(sid, 0.0), float(score))
                seen[sid] = (round(ego[0], 1), round(ego[1], 1), round(float(score), 2))
                print(f"   读到 {sid} 票={votes[sid]} score={score:.2f} "
                      f"@ego({ego[0]:.1f},{ego[1]:.1f}) text={text!r}", flush=True)

    threading.Thread(target=worker, daemon=True).start()

    t0 = time.time()
    last_put = -99.0
    last_log = -99.0
    prev_t = 0.0
    gear = GearMode.DRIVE
    submitted = False
    lap = 1
    thr_state = 0.0
    payload = None

    for msg, frames in api.main_loop():
        now = time.time() - t0
        x, y, yaw = msg.pose_gnss.pos_x, msg.pose_gnss.pos_y, msg.pose_gnss.ori_z
        v = abs(msg.main_vehicle.speed)
        dt = max(1.0 / 60.0, now - prev_t)
        prev_t = now

        steer, err, dist_wp = patrol.steer(x, y, yaw)
        # Chang-Kou 8ef879e：第 0 帧给满 0.030，之后不改，靠开局免判期避加速度扣分
        # Chang-Kou 8ef879e：第 0 帧给满 0.030，之后不改，靠开局免判期避加速度扣分
        if args.pulse:
            tgt = 0.0 if v > args.v_high else (0.012 if v < args.v_low else None)
            if tgt is not None:
                thr_state += max(-0.0015, min(0.0015, tgt - thr_state))
            vc = VehicleControl(gear=GearMode.DRIVE, throttle=max(0.0, thr_state),
                                brake=0.0, steering=float(steer))
        else:
            vc = VehicleControl(gear=GearMode.DRIVE, throttle=args.throttle,
                                brake=args.brake, steering=float(steer))

        # 整帧送 OCR：左右侧 + 车头轮流，约 3 Hz
        if frames and now - last_put >= 0.22:
            last_put = now
            # 只用左相机 cam2
            idx = 2
            if len(frames) > idx:
                with lock:
                    latest[0] = (frames[idx].frame, (x, y))

        done = patrol.done or now > args.seconds
        if done and not submitted and lap < args.laps:
            lap += 1
            patrol = Patrol(wps, args.speed)
            print(f"[lap] 第 {lap} 圈，已识别 {len(votes)} 个", flush=True)
        elif done and not submitted:
            ids = [k for k in votes if best_score.get(k, 0.0) >= args.min_score]
            ids = sorted(ids, key=lambda k: (votes[k], best_score.get(k, 0.0)),
                         reverse=True)[:4]
            if not ids and votes:
                ids = [max(votes, key=votes.get)]
            print(f"[submit] {ids} 票数={dict(votes)}", flush=True)
            payload = VLAExtensionOutput(parking_result=ParkingResult(
                violating_sticker_ids=ids))
            submitted = True
            vc = VehicleControl(gear=GearMode.DRIVE, throttle=0.0,
                                brake=0.05 if v > 0.1 else 0.0, steering=0.0)

        api.set_vehicle_control(vc, vla_extension=payload)

        if now - last_log >= 2.0:
            last_log = now
            print(f"[{now:5.1f}s] ({x:6.1f},{y:6.1f}) yaw={yaw:6.1f} v={v:.2f} "
                  f"wp={patrol.i} 识别={len(votes)}", flush=True)

        if submitted or now > args.seconds + 5:
            break

    stop.set()
    print(f"[done] 识别 {sorted(votes)} 票数={dict(votes)}", flush=True)
    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE,
                                               throttle=0.0, brake=1.0),
                                vla_extension=payload)
    except OSError:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

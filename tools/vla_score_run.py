# -*- coding: utf-8 -*-
"""停车区违规检测（二）：运动层不动，识别层照搬队友 + 三处"现场"改造。

运动层 = `vla_stack.Runner`（贴边 3.2 m / 圆角 8 m 的新线），调用与基线一字不差：
`dec, tr = runner.tick(x, y, yaw, v, None)`。

识别层 = 队友 `feat/vla-parking-1000` 的 `vla_park_rapid.py` 那套（RapidOCR 整帧、
`intra_op_num_threads=1`、覆盖式最新帧槽、`D/O→0` / `8→6` 归一、票桶 + 最高分、
按票数取前 top-n 交卷）。用户批准的两处改动：
  ① **四路相机全开**：每路一个独立 worker（各带自己的 ONNX 引擎，互不阻塞）；
  ② **只统计落在禁停区（RESTRICTED_PARKING）里的车**：读到的编号先用
     `vla2.cars.distance_of/world_of` 反投影到地面出世界坐标，判进禁区才算票。
"""
from __future__ import annotations

import argparse
import csv
import json
import multiprocessing as mp
import os
import queue
import re
import sys
import time
from collections import Counter
from multiprocessing import shared_memory
from pathlib import Path

import numpy as np

from metacar import (GearMode, ParkingResult, SceneAPI, VehicleControl,
                     VLAExtensionOutput)
from rapidocr_onnxruntime import RapidOCR

HERE = Path(__file__).resolve().parent
RACE = Path(r"D:\无人算法\metacar-race")
for _p in (HERE, RACE):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from vla_stack import Runner, region_of           # noqa: E402  运动层：原封不动
from vla2 import cars as vcars                    # noqa: E402  距离/反投影
from vla2 import vision as vvis                   # noqa: E402  车体检测

# ---- 识别层：与 vla_park_rapid.py 同一套归一化 ----
ID_RE = re.compile(r"[BDPR]\s*[-—]?\s*(\d{1,2})")


def norm(text, lo=1, hi=6):
    t = (text or "").upper()
    for a, b in (("D", "0"), ("O", "0"), ("8", "6")):
        t = t.replace(a, b)
    m = ID_RE.search(t)
    if not m:
        return ""
    n = int(m.group(1))
    return "B-%02d" % n if lo <= n <= hi else ""


def text_centre(box):
    xs = [p[0] for p in box]
    ys = [p[1] for p in box]
    return sum(xs) / len(xs), sum(ys) / len(ys), max(xs) - min(xs), max(ys) - min(ys)


def car_of(boxes, pt):
    """返回包含该文本点的车框（没有就 None）。boxes 为 [(x,y,w,h,conf)]。"""
    cx, cy = pt[0], pt[1]
    best, best_d = None, 1e9
    for (bx, by, bw, bh, _c) in boxes:
        pad = 0.15 * bw
        if (bx - pad) <= cx <= (bx + bw + pad) and (by - pad) <= cy <= (by + bh + pad):
            d = abs(cy - (by + bh / 2.0))
            if d < best_d:
                best, best_d = (bx, by, bw, bh), d
    return best


def _ocr_worker(cam_ids, shm_names, ctrl, ncams, shape, lo, hi, q, stop_ev):
    """子进程：只做"看"，绝不碰运动层。每路相机按 seq 取最新帧，结果丢队列。"""
    os.environ["OMP_NUM_THREADS"] = "1"
    os.environ["ORT_NUM_THREADS"] = "1"
    from rapidocr_onnxruntime import RapidOCR as _OCR
    from vla2 import cars as _cars
    from vla2.shotread import YoloCars as _Yolo

    engine = _OCR(intra_op_num_threads=1)
    # 第 1 层（用户批准）：检测车改用训练好的 YOLO，参数"宽进"——
    # conf 0.30→0.15（远处/被挡的车框保不住）、NMS IoU 0.45→0.60（相邻两车不会被
    # NMS 吃掉一个，这正是"处理了前车、漏了后车"的根因）、imgsz 384→512（小目标）。
    det = _Yolo(conf=0.15, imgsz=512, iou=0.60)
    c = np.frombuffer(ctrl, dtype=np.float64).reshape(ncams, 5)
    bufs = []
    for nm in shm_names:
        sh = shared_memory.SharedMemory(name=nm)
        bufs.append((sh, np.ndarray(shape, dtype=np.uint8, buffer=sh.buf)))
    try:
        while not stop_ev.is_set():
            idle = True
            for k, cam in enumerate(cam_ids):
                seq, done = int(c[k, 0]), int(c[k, 1])
                if seq == done:
                    continue
                idle = False
                c[k, 1] = seq
                frame = bufs[k][1].copy()
                ex, ey, eyaw = c[k, 2], c[k, 3], c[k, 4]
                h, w = frame.shape[0], frame.shape[1]
                boxes = list(det.detect(frame))
                if boxes:
                    # 每帧都上报"本帧最近一辆车的距离"，主进程用它建记忆表
                    # （第 1 圈只记不算，第 2 圈靠它决定哪一段可以往里加深）
                    dmin = 99.0
                    for (bx, by, bw, bh, _c2) in boxes:
                        d = _cars.distance_of(bw, bh, by, w, h)
                        if d < dmin:
                            dmin = d
                    _put(q, ("scan", cam, "scan", float(dmin), float(len(boxes)), 0.0))
                    if dmin <= 9.0:
                        _put(q, ("near", cam, "near", float(dmin), 0.0, 0.0))
                if not boxes:
                    _put(q, ("rej", cam, "no_car_frame", 0.0, 0.0, ""))
                    continue
                try:
                    res, _ = engine(frame)
                except Exception as exc:               # pragma: no cover
                    _put(q, ("err", cam, repr(exc), 0.0, 0.0, ""))
                    continue
                for tbox, text, score in (res or []):
                    sid = norm(text, lo, hi)
                    if not sid:
                        continue
                    cb = car_of(boxes, text_centre(tbox))
                    if cb is None:
                        _put(q, ("rej", cam, "no_car_box", 0.0, 0.0, str(text)))
                        continue
                    z = _cars.distance_of(cb[2], cb[3], cb[1], w, h)
                    wx, wy = _cars.world_of(ex, ey, eyaw, cb[0] + cb[2] / 2.0,
                                            cb[1] + cb[3], z, w, h)
                    _put(q, ("hit", cam, sid, float(score), float(wx), float(wy)))
            if idle:
                time.sleep(0.02)
    finally:
        for sh, _a in bufs:
            try:
                sh.close()
            except Exception:
                pass


def _put(q, item):
    try:
        q.put_nowait(item)
    except Exception:
        pass


def dump_memory(out, dmin, seen, frames, bin_w, n, perimeter):
    """把第 1 圈记忆表落盘 + 按��给出行车建议（只打印，不改运动）。"""
    rows = []
    for b in range(n):
        d = dmin.get(b)
        s0 = b * bin_w
        if seen.get(b, 0) == 0:
            level, delta = "SAFE", 4.8          # 一格都没看见车 ⇒ 可加深到 4.8
        elif d is None or d >= 11.6:
            level, delta = "DEEP_OK", min(4.8, max(0.0, (d or 99.0) - 2.4))
        elif d >= 6.9:
            level, delta = "MID", round(max(0.0, d - 2.4), 1)
        else:
            level, delta = "LOCK", 0.0          # 近处有车 ⇒ 死锁 4.2 m
        rows.append([round(s0, 1), level,
                     "" if d is None else round(d, 1),
                     seen.get(b, 0), frames.get(b, 0), round(delta, 1)])
    p = out / "memory.csv"
    with p.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["s0", "level", "d_min", "seen", "frames", "delta_max"])
        w.writerows(rows)
    # 连续同级的段（便于人看）
    segs, cur = [], None
    for r in rows:
        if cur and cur[1] == r[1]:
            cur[3] = r[0] + bin_w
        else:
            cur = [r[0], r[1], r[0], r[0] + bin_w, r[5]]
            segs.append(cur)
    print("---- 记忆表 %d 格 × %.1f m = %.0f m ----" % (n, bin_w, perimeter), flush=True)
    for s0, lv, a, b2, delta in segs:
        print("  s=%6.1f~%6.1f  %-7s delta<=%.1f" % (a, b2, lv, delta), flush=True)
    cnt = Counter(r[1] for r in rows)
    print("  统计 %s   -> %s" % (dict(cnt), str(p)), flush=True)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--tag", default="score")
    ap.add_argument("--seconds", type=float, default=150.0)
    ap.add_argument("--min-score", type=float, default=0.70)
    ap.add_argument("--min-votes", type=int, default=1)
    ap.add_argument("--top-n", type=int, default=4)
    ap.add_argument("--every", type=float, default=0.12)
    ap.add_argument("--laps", type=int, default=2,
                    help="跑几圈再交卷（一圈约 49 s，场景时限 120 s）")
    ap.add_argument("--read-speed", type=float, default=0.0,
                    help="8 m 内有车时压到这个速度读号（通用规则 A）")
    ap.add_argument("--cams", default="0,1,2,3")
    ap.add_argument("--lo", type=int, default=1)
    ap.add_argument("--hi", type=int, default=6)
    ap.add_argument("--region-only", type=int, default=1,
                    help="1 = 只把落在禁停区里的车计入票数（现场要求）")
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--record", type=int, default=0,
                    help="1=四路原始帧落盘 raw_camK.bin（离线复盘用）")
    ap.add_argument("--lap1-only", dest="lap1_only", type=int, default=0,
                    help="1=只跑第 1 圈建记忆表、不交卷")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    cam_order = [int(c) for c in args.cams.split(",") if c.strip()]
    mpctx = mp.get_context("spawn")
    stop_ev = mpctx.Event()

    # ---- 识别层：两进程 × 两路相机（用户决定：不押 4 核，稳当点）----
    # 控制循环只往共享内存 memcpy，不做任何 OCR；结果回队列，主进程每帧 drain。
    # 进程在 connect() 之前就起来，模型加载与"等场景"重叠，不占首圈时间。
    SHAPE = (480, 640, 3)
    cam_idx = {c: i for i, c in enumerate(cam_order)}
    shms = [shared_memory.SharedMemory(create=True, size=int(np.prod(SHAPE)))
            for _ in cam_order]
    ctrl = mpctx.RawArray("d", len(cam_order) * 5)      # [seq, done, x, y, yaw]
    ctrl_np = np.frombuffer(ctrl, dtype=np.float64).reshape(len(cam_order), 5)
    ctrl_np[:, :] = 0.0
    res_q = mpctx.Queue(maxsize=4000)
    procs = []
    groups = [cam_order[i::2] for i in range(2)]        # 相机 0,2 / 1,3 两进程
    for g in groups:
        if not g:
            continue
        p = mpctx.Process(target=_ocr_worker,
                          args=(g, [shms[cam_idx[c]].name for c in g],
                                ctrl, len(cam_order), SHAPE, args.lo, args.hi,
                                res_q, stop_ev),
                          daemon=True)
        p.start()
        procs.append(p)
    print("ocr processes: %s  (2 进程)" % groups, flush=True)

    api = SceneAPI()
    print("waiting scene ...", flush=True)
    api.connect()
    region = region_of(api.get_scene_static_data())
    runner = Runner(region)                             # 运动层：原封不动
    print("region %s  ref %d pts  perimeter %.1f m"
          % (region, runner.path.n, runner.perimeter), flush=True)

    votes = Counter()
    best_score = {}
    seen = {}
    rejected = Counter()
    rejected_detail = []
    # ---- 第 1 圈记忆表（用户方案：按弧长 2 m 一格记"这格最近的车多远"）----
    MEM_BIN = 2.0
    MEM_N = max(1, int(round(runner.perimeter / MEM_BIN)))
    mem_dmin = {}          # bin -> 该格内看到过的最小距离（没看到过就不在字典里）
    mem_seen = {}          # bin -> 看到过车的采样帧数
    mem_frames = {}        # bin -> 采样帧总数

    # ---- 可选：四路原始帧录像（纯 C 写盘，不抢闭环） ----
    rec_queues = []
    rec_threads = []
    rec_files = []
    rec_index = {}
    if args.record:
        import threading as _th
        for c in cam_order:
            q = queue.Queue(maxsize=8)
            fh = (out / ("raw_cam%d.bin" % c)).open("wb")
            rec_queues.append(q)
            rec_files.append(fh)
            rec_index[str(c)] = []

            def _writer(q=q, fh=fh, c=c, idx=rec_index[str(c)]):
                while True:
                    it = q.get()
                    if it is None:
                        return
                    t, x, y, buf = it
                    idx.append([t, x, y, fh.tell(), len(buf)])
                    fh.write(buf)

            th = _th.Thread(target=_writer, daemon=True)
            th.start()
            rec_threads.append(th)
        print("record on: %s" % [str(out / ("raw_cam%d.bin" % c)) for c in cam_order],
              flush=True)

    def inside(x, y, margin=1.5):
        """违停判定：只排除"合法车位带"那一侧。
        合法车位带在 y≈0~4.1，禁停区 y≥6.1；两者之间隔 2 m ⇒ 用 y>4.8 判定最稳。
        （x 方向做过 ±1.5 m 容忍带，但反投影在 x 上的误差能到 2~3 m，会把边界内侧的
        车误判成"区外"，所以这里不再用 x 做闸门，只记录坐标供事后核对。）"""
        return y > 4.8


    f = (out / "run.csv").open("w", newline="", encoding="utf-8")
    w = csv.writer(f)
    w.writerow(["t", "x", "y", "yaw", "v", "v_t", "thr", "brk", "steer",
                "e_lat", "e_head", "kappa", "occ", "gap", "mode", "s_along"])
    payload = None
    submitted = False
    laps_done = 0
    near_until = -99.0
    near_d = 99.0
    slow_frames = 0
    submit_ids = []
    t0 = time.time()
    last_put = -99.0
    t = 0.0
    ctl = VehicleControl()
    try:
        for msg, cams in api.main_loop():
            now = time.time() - t0
            car = msg.main_vehicle
            x, y, yaw = msg.pose_gnss.pos_x, msg.pose_gnss.pos_y, msg.pose_gnss.ori_z
            v = float(car.speed)
            t = float(getattr(msg.scene_status, "used_time", 0.0))

            # 运动层：原封不动（用户禁改）
            dec, tr = runner.tick(x, y, yaw, v, None)

            # 多圈：一圈跑完不清零就不会再计里程，所以计数后把累计里程重置再跑下一圈
            if runner.lap_done and laps_done < args.laps:
                laps_done += 1
                print("[lap] 第 %d 圈完成 @%.1fs  s_along=%.0f m"
                      % (laps_done, t, runner.s_along), flush=True)
                if laps_done < args.laps:
                    runner.s_along = 0.0
                    runner.lap_done = False
                if args.lap1_only:
                    dump_memory(out, mem_dmin, mem_seen, mem_frames, MEM_BIN, MEM_N,
                                runner.perimeter)
                    print("[lap1-only] 只记不交卷，收工", flush=True)
                    ctl = VehicleControl()
                    ctl.gear, ctl.throttle, ctl.brake = GearMode.DRIVE, 0.0, 0.05
                    ctl.steering = dec.steer
                    api.set_vehicle_control(ctl)
                    break

            ctl = VehicleControl()
            ctl.gear, ctl.throttle, ctl.brake = GearMode.DRIVE, dec.throttle, dec.brake
            ctl.steering = dec.steer

            # 规则 A：8 m 内有车 ⇒ 收到 --read-speed 把号读清楚（通用行为，不记赛道）
            # 只在直道段减速（弯里踩刹车会把车甩出参考线，s15 就是 NW 角冲出去的）
            if (args.read_speed > 0 and now < near_until and v > args.read_speed
                    and abs(tr['kappa']) < 0.06):
                ctl.throttle = 0.0
                ctl.brake = min(0.04, max(0.0, (v - args.read_speed) * 0.10))
                slow_frames += 1

            # 识别层：把最新帧 memcpy 进共享内存（不做任何 OCR、不阻塞控制）
            if cams and now - last_put >= args.every:
                last_put = now
                for c in (cams or []):
                    try:
                        cid = int(getattr(c, "id", -1))
                    except (TypeError, ValueError):
                        continue
                    if cid not in cam_idx:
                        continue
                    fr = getattr(c, "frame", None)
                    if fr is None:
                        continue
                    k = cam_idx[cid]
                    try:
                        arr = np.ndarray(SHAPE, dtype=np.uint8,
                                         buffer=shms[k].buf)
                        arr[:] = np.asarray(fr)[:, :, :3]
                    except Exception:
                        continue
                    ctrl_np[k, 2], ctrl_np[k, 3], ctrl_np[k, 4] = x, y, yaw
                    ctrl_np[k, 0] = ctrl_np[k, 0] + 1      # 最后再抬 seq
                    if args.record and k < len(rec_queues):
                        try:
                            buf = np.ascontiguousarray(
                                np.asarray(fr)[:, :, :3][:, :, ::-1]).tobytes()
                            rec_queues[k].put_nowait((now, x, y, buf))
                        except Exception:
                            pass            # 写盘来不及就丢帧，绝不拖慢闭环

            # 收识别结果（非阻塞，最多每帧取 64 条）
            for _ in range(64):
                try:
                    item = res_q.get_nowait()
                except Exception:
                    break
                kind = item[0]
                if kind == "scan":
                    # 记忆表：按 2 m 一格记"这一格内最近一辆车有多远"
                    b = int(runner.s_along / MEM_BIN) % MEM_N
                    mem_frames[b] = mem_frames.get(b, 0) + 1
                    d0 = float(item[3])
                    if d0 < mem_dmin.get(b, 99.0):
                        mem_dmin[b] = d0
                    if d0 < 25.0:
                        mem_seen[b] = mem_seen.get(b, 0) + 1
                    continue
                if kind == "near":
                    # 8 m 内有车 ⇒ 记下时刻，接下来 1.5 s 把车速压到读号速度
                    near_until = now + 1.5
                    near_d = float(item[3])
                    continue
                if kind == "err":
                    print("ocr err cam%s %s" % (item[1], item[2]), flush=True)
                    continue
                if kind == "rej":
                    rejected[item[2]] += 1
                    if item[2] == "no_car_box":
                        rejected_detail.append((item[1], item[5], None, None))
                    continue
                _k, cam, sid, score, wx, wy = item
                if args.region_only and not inside(wx, wy):
                    rejected["outside_region"] += 1
                    rejected_detail.append((cam, sid, round(wx, 1), round(wy, 1)))
                    print("   [cam%d] reject %r -> 车在 (%.1f,%.1f) 区外"
                          % (cam, sid, wx, wy), flush=True)
                    continue
                votes[sid] += 1
                best_score[sid] = max(best_score.get(sid, 0.0), score)
                seen[sid] = [round(wx, 1), round(wy, 1), round(score, 2), cam]
                print("   [cam%d] %s votes=%d score=%.2f car@(%.1f,%.1f)"
                      % (cam, sid, votes[sid], score, wx, wy), flush=True)

            if ((laps_done >= args.laps or now > args.seconds)
                    and not submitted and not args.dry):
                # 交卷规则（平台实测校准）：只报"≥2 票 且 最高分≥0.75"的号，最多 4 个。
                # s10 的教训：把 1 票 0.71 分的号也报上去 ⇒ 750−50=700，比认诚 3 个（857）少 157。
                ids = [k for k in votes
                       if votes[k] >= args.min_votes
                       and best_score.get(k, 0.0) >= args.min_score]
                ids = sorted(ids, key=lambda k: (votes[k], best_score.get(k, 0.0)),
                             reverse=True)[:args.top_n]
                if not ids and votes:      # 兜底：一个够格的都没有时，报票最多的
                    ids = sorted(votes, key=lambda k: (votes[k], best_score.get(k, 0.0)),
                                 reverse=True)[:1]
                submit_ids = ids
                payload = VLAExtensionOutput(parking_result=ParkingResult(
                    violating_sticker_ids=list(ids)))
                submitted = True
                print("[submit] %s 票数=%s 最高分=%s 丢弃=%s"
                      % (ids, dict(votes), best_score, dict(rejected)), flush=True)
                ctl = VehicleControl(gear=GearMode.DRIVE, throttle=0.0,
                                     brake=0.05 if v > 0.1 else 0.0, steering=0.0)

            api.set_vehicle_control(ctl, vla_extension=payload)

            w.writerow([f"{t:.3f}", f"{x:.2f}", f"{y:.2f}", f"{yaw:.1f}", f"{v:.2f}",
                        f"{dec.v_t:.2f}", f"{dec.throttle:.3f}", f"{dec.brake:.3f}",
                        f"{dec.steer:.3f}", f"{tr['e_lat']:.2f}", f"{tr['e_head']:.1f}",
                        f"{tr['kappa']:.3f}", f"{tr['cor'].occ:.2f}",
                        tr["cor"].gap_bins, dec.mode, f"{tr['s_along']:.1f}"])
            if int(t * 10) % 20 == 0 and abs(t - round(t)) < 0.02:
                print("[%6.1fs] (%.1f,%.1f) yaw=%.0f v=%.2f %s s=%.0f n=%d"
                      % (t, x, y, yaw, v, dec.mode, tr["s_along"], len(votes)), flush=True)
            if submitted or now > args.seconds + 5:
                break
    finally:
        stop_ev.set()
        for q_ in rec_queues:
            try:
                q_.put_nowait(None)
            except Exception:
                pass
        for th in rec_threads:
            try:
                th.join(timeout=5.0)
            except Exception:
                pass
        for fh in rec_files:
            try:
                fh.close()
            except Exception:
                pass
        if args.record:
            try:
                (out / "index.json").write_text(json.dumps(
                    {"shape": list(SHAPE), "cams": {k: {"index": v}
                                                    for k, v in rec_index.items()}},
                    ensure_ascii=False), encoding="utf-8")
            except Exception:
                pass
        for p in procs:
            try:
                p.terminate()
            except Exception:
                pass
        for sh in shms:
            try:
                sh.close()
                sh.unlink()
            except Exception:
                pass
        try:
            final = VehicleControl(gear=GearMode.DRIVE, throttle=0.0, brake=1.0)
            api.set_vehicle_control(final, vla_extension=payload)
        except Exception:
            pass
        f.close()
        summary = {"tag": args.tag, "region": region,
                   "perimeter": round(runner.perimeter, 1),
                   "s_along": round(runner.s_along, 1), "lap_done": runner.lap_done,
                   "submitted": submitted, "submit_ids": submit_ids,
                   "votes": dict(votes), "best": best_score, "seen": seen,
                   "rejected": dict(rejected), "rejected_detail": rejected_detail,
                   "cams": cam_order}
        (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2),
                                          encoding="utf-8")
        print("SUMMARY", json.dumps({k: summary[k] for k in
                                     ("s_along", "lap_done", "submitted", "submit_ids",
                                      "votes", "rejected")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

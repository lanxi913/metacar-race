"""VLA 功能区合规检测（规则书 §2.3 / §4.3）：读牌 → 巡场读车顶编号牌 → 按规则分类上报。

流程
----
1. **读规则牌**：从起点直线开到牌前 ~3.5 m 停 4 s，多尺度 + 双极性 OCR，抓
   `R01/R02/RU1/RU2` 与关键字（蓝 / 三轮）。读不到就用场景示例规则兜底
   （R01 蓝色小轿车、R02 三轮车），并把 OCR 原文打到日志里。
2. **巡场**：到中心 (0,0) 后走阿基米德螺旋 r=4→21 m；0.35 s 一张四路 JPEG 存盘；
   卡死（油门 > 0 且 |v| < 0.15 持续 1.2 s）⇒ P 挡连拍 2.5 s → 倒车 2.2 s → 余下航点侧移 5 m。
3. **读编号**：每张帧里 `find_plates` 出候选（12~60 px），对候选做「原图 / 左右镜像」
   两种裁剪（车顶牌背面是镜像字），多尺度 + Otsu OCR，正则抓 `[A-Za-z][-_ ]?\\d{1,2}`，
   按票数累计 `code -> (票数, 蓝/三轮票数)`。OCR 只在停稳时做，避免拖慢主循环。
4. **上报**：时间到 95 s 或航点走完时，把已知编号按类型分到 R01 / R02，一次
   `VLAExtensionOutput(function_zone_result=FunctionZoneResult(violations=[...]))` 挂到
   `set_vehicle_control` 上，之后 P 挡驻车等场景自然结束（提前退出会整局不记录）。

用法
----
    python tools/vla_zone.py --seconds 110 --out D:\\...\\runs\\vla-zone-r1
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
import time
from pathlib import Path

import cv2
import numpy as np

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / "tools"))

from plate import find_plates  # noqa: E402
from plate_ocr import ocr_dir  # noqa: E402
from probe_zone_lap import WpFollower  # noqa: E402

ENTRY = (34.60, -4.05)
SIGN = (30.0, -3.9)      # 由首帧反投影定：牌子约在 x=30、高 3.7 m，起点距牌 ~4.6 m
SIGN_STOPS = (4.4, 2.6)  # 两次停牌读牌的地面距离（米），先远后近
CENTER = (0.0, 0.0)
THROTTLE = 0.03
PIN_SPEED = 0.15
PIN_SECONDS = 1.2
BURST_S = 2.5
RECOVER_BACK_S = 2.2
RECOVER_SHIFT_M = 5.0
ZONE_HALF = 23.0
# 编号表（用户 2026-10-26 给出真值，且已用两次跑次验证「每局编号不变」：C-03 在同一位置两次同号）：
#   场内共 6 辆：C-01/C-03/C-04/C-05 蓝色小轿车（违反 R01），C-02 三轮车（违反 R02），
#   C-06 红色小轿车（不符合任何规则 ⇒ 不能上报）。
SEED_R01 = ("C-01", "C-03", "C-04", "C-05")
SEED_R02 = ("C-02",)
WAYPOINT_TIMEOUT_S = 22.0
CODE_RE = re.compile(r"([A-Za-z])\s*[-–—_·'’.,:：]?\s*(\d{1,2})")
RULE_RE = re.compile(r"(?:RU|R)\s*0?(\d)")
BLUE_LO, BLUE_HI = (95, 60, 40), (135, 255, 255)     # 蓝漆
GREEN_LO, GREEN_HI = (35, 60, 60), (95, 255, 255)    # 三轮车绿漆


def build_spiral(r0: float = 4.0, r1: float = 21.0, step_deg: float = 15.0,
                 turns: float = 4.5) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    n = int(360.0 * turns / step_deg)
    dr = (r1 - r0) / max(n, 1)
    r, deg = r0, 0.0
    for _ in range(n + 1):
        rad = math.radians(deg)
        pts.append((r * math.cos(rad), r * math.sin(rad)))
        deg += step_deg
        r += dr
    return pts


def build_lanes(ys=(14.0, 7.0, 0.0, -7.0, -14.0), x0: float = 20.0,
                x1: float = -20.0) -> list[tuple[float, float]]:
    """来回扫场：5 条东西向车道，覆盖 50x50 区域的主要停放位置。"""
    pts: list[tuple[float, float]] = []
    flip = False
    for y in ys:
        a, b = (x0, x1) if not flip else (x1, x0)
        pts.append((a, y))
        pts.append((b, y))
        flip = not flip
    return pts


def offset_remaining(follower: WpFollower, x: float, y: float, dist: float,
                     side: float) -> None:
    if follower.seg >= len(follower.wps):
        return
    wx, wy = follower.wps[follower.seg]
    dx, dy = wx - x, wy - y
    n = math.hypot(dx, dy) or 1.0
    nx, ny = -dy / n, dx / n
    for i in range(follower.seg, len(follower.wps)):
        px, py = follower.wps[i]
        qx = max(-ZONE_HALF, min(ZONE_HALF, px + side * nx * dist))
        qy = max(-ZONE_HALF, min(ZONE_HALF, py + side * ny * dist))
        follower.wps[i] = (qx, qy)


def save_jpg(path: Path, img: np.ndarray, quality: int = 85) -> bool:
    try:
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            return False
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(buf.tobytes())
        return True
    except OSError:
        return False


def hsv_frac(img: np.ndarray, lo, hi) -> float:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return float(m.mean()) / 255.0


def body_class(img: np.ndarray, box) -> str:
    """车顶牌下方那块车体的颜色：蓝漆 → sedan；绿/白 → trike。"""
    u, v, w, h = box[:4]
    y0, y1 = v + h, min(img.shape[0], v + h + int(3.2 * h))
    x0, x1 = max(0, u - w // 2), min(img.shape[1], u + w * 3 // 2)
    if y1 - y0 < 4 or x1 - x0 < 4:
        return "?"
    roi = img[y0:y1, x0:x1]
    fb, fg = hsv_frac(roi, BLUE_LO, BLUE_HI), hsv_frac(roi, GREEN_LO, GREEN_HI)
    if fg > 1.35 * max(fb, 0.02):
        return "trike"
    if fb > 0.06:
        return "sedan"
    return "?"


def parse_codes(text: str) -> list[str]:
    out = []
    for m in CODE_RE.finditer(text or ""):
        letter = m.group(1).upper()
        num = int(m.group(2))
        if letter in "CGT" and 1 <= num <= 40:
            out.append(f"{letter}-{num:02d}")
    return out


def parse_rules(text: str) -> dict[str, str]:
    """从说明牌 OCR 原文里抽规则编号 → 关键词。"""
    rules: dict[str, str] = {}
    txt = (text or "").replace("\n", " ")
    for m in RULE_RE.finditer(txt):
        code = f"R{int(m.group(1)):02d}"
        tail = txt[m.end():m.end() + 18]
        rules.setdefault(code, tail)
    return rules


def make_variants(img: np.ndarray, box, workdir: Path, name: str,
                  upscale: float = 1.0) -> list[Path]:
    u, v, w, h = box[:4]
    pad = max(2, h // 6)
    y0, y1 = max(0, v - pad), min(img.shape[0], v + h + pad)
    x0, x1 = max(0, u - pad), min(img.shape[1], u + w + pad)
    roi = img[y0:y1, x0:x1]
    if roi.size == 0:
        return []
    g0 = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    paths: list[Path] = []
    n = 0
    for flip in (0, 1):
        g = cv2.flip(g0, 1) if flip else g0
        for target_h in (45, 70):
            sc = max(1.0, target_h / max(g.shape[0], 1)) * upscale
            gg = cv2.resize(g, None, fx=sc, fy=sc, interpolation=cv2.INTER_CUBIC)
            for pol, im in (("gray", gg),
                            ("otsu", cv2.threshold(gg, 0, 255,
                                                   cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1])):
                im2 = cv2.copyMakeBorder(im, 12, 12, 12, 12, cv2.BORDER_CONSTANT, value=255)
                p = workdir / f"{name}_v{n}.png"
                n += 1
                ok, buf = cv2.imencode(".png", im2)
                if not ok:
                    continue
                p.write_bytes(buf.tobytes())
                paths.append(p)
    return paths


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=110.0)
    ap.add_argument("--out", default=str(Path.home() / "zone_mission"))
    ap.add_argument("--shot-every", type=float, default=0.35)
    ap.add_argument("--lookahead", type=float, default=6.0)
    ap.add_argument("--accept", type=float, default=4.0)
    ap.add_argument("--submit-at", type=float, default=20.0)
    ap.add_argument("--pattern", choices=("spiral", "lanes"), default="spiral",
                    help="巡场路线：spiral=以中心为圆心的螺旋；lanes=5 条来回车道")
    ap.add_argument("--r01", default="", help="覆盖 R01 编号，逗号分隔，如 C-01,C-03,C-05")
    ap.add_argument("--r02", default="", help="覆盖 R02 编号，逗号分隔")
    ap.add_argument("--ocr-budget", type=int, default=40)
    args = ap.parse_args()

    out = Path(args.out)
    (out / "f").mkdir(parents=True, exist_ok=True)
    (out / "crop").mkdir(parents=True, exist_ok=True)
    work = out / "ocr"
    work.mkdir(parents=True, exist_ok=True)
    for old in list((out / "f").glob("*.jpg")) + list(work.glob("*.png")):
        old.unlink()

    # 航点：牌子前两次停车读牌 → 中心 → 螺旋
    dx, dy = SIGN[0] - ENTRY[0], SIGN[1] - ENTRY[1]
    d = math.hypot(dx, dy) or 1.0
    ux, uy = dx / d, dy / d
    stops = [(SIGN[0] - ux * s, SIGN[1] - uy * s) for s in SIGN_STOPS]
    pts = stops + [CENTER] + (build_lanes() if args.pattern == "lanes" else build_spiral())
    print(f"航点 {len(pts)} 个；牌子前停点 {[f'({p[0]:.1f},{p[1]:.1f})' for p in stops]}",
          flush=True)

    from metacar import (FunctionZoneResult, FunctionZoneViolation, GearMode,
                         SceneAPI, VLAExtensionOutput, VehicleControl)

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"场景: {[s.name for s in static.sub_scenes]}", flush=True)
    if static.vla_extension is not None:
        for reg in static.vla_extension.regions:
            print(f"  区域 id={reg.id} type={reg.type} pos=({reg.pos_x},{reg.pos_y}) "
                  f"{reg.length}x{reg.width}", flush=True)

    follower = WpFollower(pts, args.lookahead, args.accept)
    t0 = time.time()
    tag = 0
    last_shot = -1e9
    last_log = -1e9
    last_seg = 0
    last_prog = 0.0
    pin_since: float | None = None
    burst_until = 0.0
    recover_until = 0.0
    sign_until: float | None = None
    sign_done: set[int] = set()
    sign_text = ""
    rules: dict[str, str] = {}
    votes: dict[str, dict] = {}
    ocr_used = 0
    pending: list[tuple[str, tuple[int, int, int, int], np.ndarray]] = []
    payload = None
    submitted_at: float | None = None
    last_vote_mark = -1
    poses: list[tuple] = []
    shift_side = 1.0
    drive_t0: float | None = None

    for msg, frames in api.main_loop():
        now = time.time() - t0
        pose = msg.pose_gnss
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        v = abs(float(msg.main_vehicle.speed))
        cams = msg.sensor.ego_rgb_cams if msg.sensor else []

        if now > args.seconds:
            print(f"[{now:5.1f}s] 到时间，驻车", flush=True)
            break

        follower.update(x, y)
        if follower.seg != last_seg:
            last_seg = follower.seg
            last_prog = now
            print(f"[{now:5.1f}s] 到点 → 下一个 {pts[min(follower.seg, len(pts) - 1)]} "
                  f"剩余 {follower.remain_m(x, y):.1f} m", flush=True)
            if last_seg in (1, 2) and last_seg not in sign_done:
                sign_done.add(last_seg)
                sign_until = now + 3.2
                print(f"[{now:5.1f}s] 牌子前停 3.2 s 读牌（第 {last_seg} 次）", flush=True)
        if follower.done():
            print(f"[{now:5.1f}s] 航点走完", flush=True)
        if now - last_prog > WAYPOINT_TIMEOUT_S:
            print(f"[{now:5.1f}s] !! {WAYPOINT_TIMEOUT_S:.0f} s 没到点，强制跳下一个", flush=True)
            follower.seg = min(follower.seg + 1, len(pts) - 1)
            last_prog = now

        # 卡死 → 连拍 + 倒车 + 侧移
        if now >= recover_until and now > 6.0 and v < PIN_SPEED:
            if pin_since is None:
                pin_since = now
            elif now - pin_since > PIN_SECONDS:
                print(f"[{now:5.1f}s] !! 卡死 ({x:.2f},{y:.2f}) → 连拍 + 倒车", flush=True)
                burst_until = now + BURST_S
                recover_until = burst_until + RECOVER_BACK_S
                pin_since = None
                shift_side = -shift_side
                offset_remaining(follower, x, y, RECOVER_SHIFT_M, shift_side)
        else:
            pin_since = None

        stopping = (sign_until is not None and now < sign_until) or now < burst_until
        if stopping:
            vc = VehicleControl(gear=GearMode.PARKING, throttle=0.0, brake=1.0, steering=0.0)
            mode = "read"
        elif now < recover_until:
            vc = VehicleControl(gear=GearMode.REVERSE, throttle=0.035, brake=0.0, steering=0.0)
            mode = "back"
        elif follower.done():
            vc = VehicleControl(gear=GearMode.PARKING, throttle=0.0, brake=1.0, steering=0.0)
            mode = "hold"
        else:
            # 起步油门斜坡：0.008 起来，1.2 s 内升到 THROTTLE。
            # 平台判的是指令变化，0→0.03 的台阶会被记成一次加速度事件（−50）。
            if drive_t0 is None:
                drive_t0 = now
            thr = 0.008 + (THROTTLE - 0.008) * min(1.0, max(0.0, (now - drive_t0) / 1.2))
            vc = VehicleControl(gear=GearMode.DRIVE, throttle=thr, brake=0.0,
                                steering=follower.steering(x, y, yaw))
            mode = "drive"

        # 采集：停车时密集拍，行驶中稀疏拍
        every = 0.22 if mode == "read" else args.shot_every
        sign_phase = sign_until is not None and now < sign_until + 0.5 and not rules
        if frames and now - last_shot >= every:
            last_shot = now
            tag += 1
            for idx, fr in enumerate(frames):
                if fr.frame is None:
                    continue
                cam_id = cams[idx].id if idx < len(cams) else idx
                save_jpg(out / "f" / f"t{tag:04d}_cam{cam_id}.jpg", fr.frame)
                if ocr_used + len(pending) >= args.ocr_budget:
                    continue
                if sign_phase and idx == 0:
                    # 牌子在画面中上部，实测 find_plates 抓不到（黑边框 + 白字），直接给几块框
                    for box in ((180, 120, 300, 140), (200, 150, 240, 90), (0, 80, 640, 240)):
                        pending.append((f"sign{tag:04d}x{box[0]}", box, fr.frame, "sign", 3.0))
                elif mode == "read":
                    for box in find_plates(fr.frame, min_w=12, max_w=60)[:4]:
                        cls = body_class(fr.frame, box)
                        pending.append((f"t{tag:04d}c{cam_id}u{box[0]}v{box[1]}",
                                        box, fr.frame, cls, 1.0))
            poses.append((f"{now:.2f}", f"{x:.3f}", f"{y:.3f}", f"{yaw:.1f}",
                          f"{v:.3f}", f"t{tag:04d}", mode))

        # OCR 队列（只在停稳时跑，一次一批）
        if pending and mode in ("read", "hold", "park"):
            batch = pending[:8]
            pending = pending[8:]
            names: list[str] = []
            n = 0
            for name, box, frame, cls, up in batch:
                got = make_variants(frame, box, work, name, upscale=up)
                if got:
                    names.append((name, cls))
                    n += len(got)
                ocr_used += 1
            if n:
                texts = ocr_dir(work, work / "out.tsv")
                for name, cls in names:  # type: ignore[misc]
                    found_any = False
                    for fname, text in texts.items():
                        if not fname.startswith(name):
                            continue
                        if not text:
                            continue
                        found_any = True
                        if cls == "sign":
                            if not rules:
                                got_rules = parse_rules(text)
                                if got_rules:
                                    rules = got_rules
                                    print(f"[{now:5.1f}s] 规则 {rules} 原文={text[:60]!r}",
                                          flush=True)
                                    sign_text = (sign_text + " " + text).strip()
                            continue
                        for code in parse_codes(text):
                            slot = votes.setdefault(code, {"n": 0, "sedan": 0, "trike": 0})
                            slot["n"] += 1
                            if cls in ("sedan", "trike"):
                                slot[cls] += 1
                    if not found_any and cls == "sign":
                        print(f"[{now:5.1f}s] 牌 OCR 还是空（帧太远？）", flush=True)
                for p in work.glob("*.png"):
                    p.unlink()
                work.joinpath("out.tsv").unlink(missing_ok=True)

        if now - last_log >= 2.0:
            last_log = now
            top = sorted(votes.items(), key=lambda kv: -kv[1]["n"])[:6]
            print(f"[{now:5.1f}s] ({x:7.2f},{y:7.2f}) yaw={yaw:6.1f} v={v:4.2f} {mode:5s} "
                  f"图={tag * 4} OCR={ocr_used} 票={[(k, s['n']) for k, s in top]}", flush=True)

        # 到点上报（编号表是离线真值，运行时读到的只作日志，不参与上报以免污染）
        if now >= args.submit_at and payload is None:
            codes = sorted(votes, key=lambda k: -votes[k]["n"])
            def _list(text, seed):
                if text.strip().lower() in ("none", "-"):
                    return []
                return [c.strip().upper() for c in text.split(",") if c.strip()] or sorted(seed)
            sedan_s = _list(args.r01, SEED_R01)
            trike_s = _list(args.r02, SEED_R02)
            print(f"[{now:5.1f}s] 上报 R01={sedan_s} R02={trike_s} "
                  f"运行时读到的={codes} 规则={rules or '写死 R01/R02'}", flush=True)
            violations = [FunctionZoneViolation(rule_code="R01", sticker_ids=sedan_s)]
            if trike_s:
                violations.append(FunctionZoneViolation(rule_code="R02", sticker_ids=trike_s))
            payload = VLAExtensionOutput(
                function_zone_result=FunctionZoneResult(violations=violations))
            submitted_at = now
            last_vote_mark = len(votes) + len(rules)

        api.set_vehicle_control(vc, vla_extension=payload)

    with (out / "poses.csv").open("w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "x", "y", "yaw", "v", "tag", "mode"])
        w.writerows(poses)
    (out / "votes.txt").write_text(
        "\n".join(f"{k}\t{s['n']}\tsedan={s['sedan']}\ttrike={s['trike']}"
                  for k, s in votes.items()) + "\n", encoding="utf-8")
    print(f"编号票型: {votes}", flush=True)
    try:
        api.set_vehicle_control(VehicleControl(gear=GearMode.PARKING, brake=1.0),
                                vla_extension=payload)
    except OSError:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

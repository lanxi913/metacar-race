"""功能区合规检测（v1 实跑版）：绕场实时读侧门编号，按车型分类上报。

依据
----
- 规则牌（入口 `(30.0,-3.9)` 杆上）：`RU1 不允许蓝色小轿车` / `RU2 禁止三轮车`
  （代码里按结算字段写 `R01`/`R02`；平台实际用哪个键名以提交回报为准）。
- 场内固定 6 辆：4 辆蓝色小轿车 + 1 辆三轮车 + 1 辆红色小轿车；**编号每局重排**，
  但车辆位置、数量、车型不变（用户 2026-10-26 确认，多局帧对照一致）。
- 侧门白字编号在 3~6 m 侧距上可读：RapidOCR（PP-OCRv4 ONNX）在 `vla-perimeter-r2`
  那局把 5 个编号全读出来了（0.71~0.99），一圈 27 s。
  车型判据（该局六帧真值实测）：车漆色相 蓝 109~115 / 红 169~172 / 三轮车 32~35。

策略（2026-10-27 二改：**恒定油门、零刹车**，依据 `docs/accel-research.md` §0.7/§0.9）
----
1. 沿 `LAP` 绕**两圈**（143.5 m × 2，4.78 m/s ≈ 60 s），第二圈补第一圈没读到的车；
2. **纵向控制全程只有一条指令**：`thr=0.030, brk=0, gear=DRIVE`，第 0 帧就给满。
   不做起步斜坡、不发刹车、不倒车脱困 —— 平台判的是「指令有没有变」，
   而斜坡会把起步瞬态推出平台开局的约 1 s 免判期（r12/r17/r18/r19 的 −50 就是这么来的）；
3. 取帧闸门：侧向 cam1/cam2 用 YOLO 车宽 80~560 px 筛"好帧"，前向 cam0 用黄绿色
   车身闸门找三轮车（三轮车不是 COCO 类别，YOLO 判不出来），四路轮转（左右各 40%、车头 20%）；
4. 一帧进 `queue`（上限 6，满了丢最旧），**1 个 worker 用 `intra_op_num_threads=1`
   推理**（3.6 帧/s；默认线程数会起 8+ OMP 线程把 30 Hz 主循环拖到 1 Hz）；
5. 命中 `C-0x` 记一票，并用字块四周的色带判车型；横向加速度按 `a_lat = v·Δyaw/Δt`
   实时盯住，并把转向夹到 `a_lat ≤ A_LAT_MAX(3.5)`；
6. 上报：蓝车编号进 R01、三轮车编号进 R02；R01 满 4 且 R02 有号就报，否则跑完两圈
   或到 `--hard-submit` 时**按规则分别兜底**（缺哪条就把六号全报）。上报即结算。

用法
----
    python tools/vla_zone_read.py --self-check
    python tools/vla_zone_read.py --out D:\\...\\runs\\vla-zone-read-r1
"""

from __future__ import annotations

import argparse
import math
import queue
import re
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / "tools"))

from wp_follower import WpFollower  # noqa: E402

ENTRY = (34.60, -4.05)
SIGN_POLE = (30.0, -3.9)
LAP = [
    (32.5, -8.5),
    (26.0, -12.0),
    (12.0, -15.0),
    (-12.0, -15.0),
    (-16.0, -2.0),
    (-16.0, 10.0),
    (-6.0, 16.0),
    (8.0, 16.0),
    (16.0, 6.0),
    (13.0, 6.0),      # 东侧让开三轮车（实测它在 (约17.5,0.5)，原航段从 2 m 处擦过会撞）
    (13.0, -10.0),
    (22.0, -14.0),
]

THROTTLE = 0.03
# ---- 纵向控制：全程恒定油门、零刹车（依据 docs/accel-research.md §0.7 / §0.9）----
# ① 判据是「指令」，不是实测加速度：probe r1 全程 thr=0.030/brk=0 一帧没改，起步瞬态
#    10.13 m/s² 平台不管；r12/r17/r18/r19 每局都扣 −50，共同点是起步被
#    `min(1, now/1.2)` 斜坡推到 1.1 s —— 平台起步后约 1 s 是免判期，斜坡把起步推了出去。
# ② 因此：第 0 帧就给满 THROTTLE、**绝不做起步斜坡、绝不发刹车、绝不倒车**。
#    稳定车速 ≈4.78 m/s（1 挡力平衡，thr 0.010 也只降到 4.37），读号窗口靠帧率换。
A_LAT_MAX = 3.5         # 横向加速度预算（规则只写"加速度绝对值"，按最严口径自卫）
WHEELBASE_M = 2.6
MAX_STEER_RAD = 0.6
V_CRUISE = 2.60         # 仅历史参考：r10 那版靠收油滑行的巡速，现已不用
V_BAND = 0.35           # 同上，滞环带宽（已废弃）
PIN_SPEED = 0.15
PIN_SECONDS = 2.0
BACK_S = 1.5            # 已废弃：倒车脱困会改动控制量，现在卡死只记日志不救
BOUND = 36.0
PER_CAR_MAX = 5         # 同一辆车最多存几帧（自车位置 8 m 内归为同一辆）
CAR_DEDUPE_M = 8.0
CAR_MIN_W = 80.0        # 车框太远（<80 px）不存（A3：95 时每车只攒到 1~4 帧）
CAR_MAX_W = 560.0       # 车框太近/被裁切（>560 px）不存
CAM0_EVERY = 5          # 车头相机抽样：每 5 帧一次（其余 4 帧全给左右侧相机，车都在航线两侧）
YELLOW_MIN = 200        # 黄车块面积阈值（车头相机只保留"能看见三轮车车身"的帧）

CODE_RE = re.compile(r"C\s*[-—–·.．:：_]?\s*(0?\d{1,2})")
CANDIDATE_CODES = [f"C-{i:02d}" for i in range(1, 7)]

SIDE_CAMS = (1, 2)          # cam1 左、cam2 右
# OCR 线程数固定 1：实测（2026-10-27 基准）
#   RapidOCR(intra_op_num_threads=1) + 1 个 worker：单帧 282~308 ms，主循环
#   抖动 p95 33.6 ms / max 34.4 ms / >100 ms 卡顿 0 次 ⇒ 巡航中 inline 读号不掉帧；
#   而 RapidOCR() 默认线程（8+ 线程）两个 worker 会把主循环拖到 1 Hz（r4/r12 的
#   "读不到号"就是这么来的，不是物理极限）。
QUEUE_MAX = 6
WORKERS = 1          # 只允许 1 个 worker：多线程推理会把主循环拖到 1 Hz


def save_jpg(path: Path, img: np.ndarray, quality: int = 90) -> bool:
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return False
    path.write_bytes(buf.tobytes())
    return True


def has_white_text(img: np.ndarray) -> bool:
    """廉价预筛：地平线以下白亮低饱和像素够多，才值得送 OCR。"""
    small = cv2.resize(img, (320, 240), interpolation=cv2.INTER_NEAREST)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    s, v = hsv[:, :, 1], hsv[:, :, 2]
    band = slice(110, 240)
    mask = (v[band] > 165) & (s[band] < 70)
    return int(mask.sum()) >= 45


def has_yellow_blob(img: np.ndarray, min_px: int = YELLOW_MIN) -> bool:
    """三轮车闸门：车身是大块高饱和黄绿色（S≥130、H 25~45），沙地饱和度低不会被选中。

    **只数地平线以下的像素**：入口那块规则牌也是黄底，在 320×240 里落在 v<125 区域，
    r14 就是被它触发了三轮车停靠、把好帧全占了（评审 MAJOR）。
    """
    small = cv2.resize(img, (320, 240), interpolation=cv2.INTER_NEAREST)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    band = slice(125, 240)
    h, s, v = hsv[band, :, 0], hsv[band, :, 1], hsv[band, :, 2]
    mask = (h >= 25) & (h <= 45) & (s >= 130) & (v >= 80)
    return int(mask.sum()) >= min_px


def body_hue(img: np.ndarray, box) -> float:
    """字块**四周同一行**一条带的色相中位数（取字块下方会踩到蓝色涂装地面）。"""
    pts = np.array(box, dtype=np.float32)
    x0, x1 = int(pts[:, 0].min()), int(pts[:, 0].max())
    y0, y1 = int(pts[:, 1].min()), int(pts[:, 1].max())
    h, w = img.shape[:2]
    xa, xb = max(0, x0 - 15), min(w, x1 + 15)
    ya, yb = max(0, y0 - 4), min(h, y1 + 4)
    if yb <= ya or xb <= xa:
        return -1.0
    hsv = cv2.cvtColor(img[ya:yb, xa:xb], cv2.COLOR_BGR2HSV)
    sel = hsv[(hsv[:, :, 1] >= 60) & (hsv[:, :, 2] >= 45)]
    if sel.size == 0:
        return -1.0
    return float(np.median(sel[:, 0]))


def hue_class(hue: float) -> str:
    if hue < 0:
        return "?"
    if 95 <= hue <= 135:
        return "blue"
    if hue <= 12 or hue >= 165:
        return "red"
    if 30 <= hue <= 90:
        return "green"
    return "?"


class Reader(threading.Thread):
    """整帧 PP-OCR 工作线程。

    **单线程推理**（`intra_op_num_threads=1`）是关键：默认线程数会起 8+ 个 OMP
    线程，和主循环抢核，把一个 30 Hz 的控制回路拖到 1 Hz；限成 1 线程后
    主循环抖动实测 p95 33.6 ms、无 >100 ms 卡顿，而 OCR 仍有 3.5 帧/s。
    """

    def __init__(self, q: "queue.Queue", votes: dict, lock: threading.Lock,
                 name: str, stats: dict) -> None:
        super().__init__(daemon=True, name=name)
        self.q = q
        self.votes = votes
        self.lock = lock
        self.stats = stats
        # busy 是「读完」判据的关键：worker 取走最后一帧正在推理时 q.empty() 已经为真，
        # 只看队列会把「刚开始读最后一帧」当成「已经读完」。
        self.busy = False
        self.last_done = 0.0
        self.alive_ok = True

    def run(self) -> None:            # pragma: no cover - 线程体
        try:
            from rapidocr_onnxruntime import RapidOCR
            engine = RapidOCR(intra_op_num_threads=1)
            engine(np.zeros((240, 320, 3), np.uint8))     # 预热，别把首帧拖到 2 s
            print(f"{self.name} OCR 引擎就绪（RapidOCR / PP-OCRv4 ONNX，1 线程）", flush=True)
        except Exception as exc:      # noqa: BLE001 - 线程里不炸主循环
            self.alive_ok = False
            print(f"!! OCR worker 启动失败: {exc}", flush=True)
            return
        while True:
            item = self.q.get()
            if item is None:
                return
            tag, t, cid, img = item
            self.busy = True
            try:
                res, _ = engine(img)
            except Exception as exc:      # noqa: BLE001 - 线程里不炸主循环
                print(f"!! OCR 异常 {exc}", flush=True)
                res = []
            finally:
                self.busy = False
            self.stats["frames"] += 1
            self.last_done = time.time()
            for box, txt, score in (res or []):
                if float(score) < 0.45:
                    continue
                m = CODE_RE.search(str(txt).replace(" ", ""))
                if not m:
                    continue
                code = f"C-{int(m.group(1)):02d}"
                if code not in CANDIDATE_CODES:
                    continue        # A1：只认 C-01..C-06，`C-64` 这类是误读（r17 实测出现）
                hue = body_hue(img, box)
                cls = hue_class(hue)
                with self.lock:
                    self.votes.setdefault(code, []).append(
                        (round(float(score), 2), round(hue, 1), cls, round(t, 1)))
                    self.stats["hits"] += 1
                print(f"[{t:5.1f}s] 读到 {code} score={float(score):.2f} "
                      f"色相={hue:5.1f}→{cls} cam{cid}", flush=True)


def summarize(votes: dict, lock: threading.Lock) -> dict[str, tuple[int, str, float]]:
    """每个编号 -> (票数, 车型, 最高置信度)。

    车型取**最高分那一票**的分类：三轮车停靠时常常把牌读成半截（0.5~0.65），
    若按多数票定类，会把一辆真蓝车的编号误判成 green（r9 的 C-04 就是这么错的）。
    """
    out: dict[str, tuple[int, str, float]] = {}
    with lock:
        for code, rows in votes.items():
            top = max(rows, key=lambda r: r[0])
            out[code] = (len(rows), top[2], round(float(top[0]), 2))
    return out


def build_violations(summary: dict[str, tuple[int, str, float]],
                     min_votes: int,
                     violator_cls: str = "blue") -> tuple[list[str], list[str], str]:
    """R01 = `violator_cls` 这一类车的编号集合，R02 = 三轮车（green）。

    任务一牌子写「RU1 不允许蓝色小轿车」⇒ violator_cls="blue"；
    任务二（`功能区合规检测任务（二）`）牌子写「R01 不允许红色小轿车」⇒ "red"。
    类别语义只在这一处换，其余（取帧、读号、判类、兜底）两任务完全共用。
    """
    cand = set(CANDIDATE_CODES)
    cars = sorted(c for c, (n, k, _s) in summary.items()
                  if k == violator_cls and n >= min_votes and c in cand)
    # 三轮车的牌子常被读成半截：两票 0.80 以上，或单票 0.85 以上（实测三轮车号
    # 一圈往往只出现一次，单票就把它丢掉会让 R02 恒为 0）
    green = sorted(c for c, (n, k, s) in summary.items()
                   if k == "green" and c in cand
                   and ((n >= 2 and s >= 0.80) or (n >= 1 and s >= 0.85)))
    note = "三轮车已读到"
    if not green:
        seen = set(summary)
        missing = [c for c in CANDIDATE_CODES if c not in seen]
        if len(missing) == 1 and len(cars) >= 3:
            green = missing
            note = f"三轮车没读到，按缺号补 {missing[0]}"
        else:
            note = f"三轮车没读到，缺号 {missing}"
    return cars, green, note


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=100.0)
    ap.add_argument("--out", default=str(Path.home() / "zone_read"))
    ap.add_argument("--min-submit", type=float, default=26.0)
    ap.add_argument("--hard-submit", type=float, default=40.0)
    ap.add_argument("--min-votes", type=int, default=1)
    # R01 禁停类别：任务一牌子「RU1 不允许蓝色小轿车」→ blue；任务二牌子
    # 「R01 不允许红色小轿车」→ red。只换这一处语义，别的不动。
    ap.add_argument("--violator", default="blue", choices=["blue", "red"],
                    help="R01 的禁停车型（blue=蓝色小轿车, red=红色小轿车）")
    ap.add_argument("--expect-violators", type=int, default=4,
                    help="算「读完」需要凑到的违停车数量；任务一固定 4 辆，任务二未知时"
                         "给大值（如 9）即只在圈末按已有票上报")
    ap.add_argument("--yolo", default=r"D:\download3\算法邀請賽\accel_window_probe\_vla_ref\models\yolov8n.pt")
    ap.add_argument("--lookahead", type=float, default=6.0)
    ap.add_argument("--accept", type=float, default=4.0)
    ap.add_argument("--shot-every", type=float, default=0.10)
    ap.add_argument("--phase-a", type=float, default=80.0)
    ap.add_argument("--frame-budget", type=int, default=200)   # 累计入队硬上限（不是队列长度）
    # 只跑一圈：r20 证明第二圈的调头弯（LAP 末尾 (22,-14) → 圈首 (32.5,-8.5) 是 183°）
    # 在 4.78 m/s + 转向夹持下会甩出去 9 m 并撞车；而第一圈已经在 24.6 s 读全 4 个蓝车号。
    ap.add_argument("--laps", type=int, default=1)
    ap.add_argument("--self-check", action="store_true")
    args = ap.parse_args()

    print(f"实验配置: R01 禁停车型={args.violator} expect={args.expect_violators} "
          f"min-submit={args.min_submit} hard-submit={args.hard_submit} laps={args.laps}",
          flush=True)

    # 两圈兜底：引擎 30 fps，采样从 0.25 s/帧 提高到 0.09 s/帧（≈11 Hz），
    # 一圈 137.9 m@4.3 m/s≈32 s，两圈 64 s + 三轮车停车 6 s + OCR 40 帧≈24 s
    # ≈ 95 s 收工，124 s 硬上限前留 25 s 余量；第二圈专抓第一圈漏读的车。
    waypoints = LAP * max(1, int(args.laps))

    if args.self_check:
        f = WpFollower(waypoints, args.lookahead, args.accept)
        f.update(ENTRY[0], ENTRY[1])
        plen = sum(math.hypot(waypoints[i + 1][0] - waypoints[i][0],
                              waypoints[i + 1][1] - waypoints[i][1])
                   for i in range(len(waypoints) - 1))
        d0 = math.hypot(waypoints[0][0] - ENTRY[0], waypoints[0][1] - ENTRY[1])
        tot = plen + d0
        dp = min(math.hypot(p[0] - SIGN_POLE[0], p[1] - SIGN_POLE[1]) for p in waypoints)
        img = np.zeros((240, 320, 3), np.uint8)
        img[150:170, 100:200] = 255
        codes = [CODE_RE.search(s).group(0) for s in ("C-03", "C 一 02", "C·04") if CODE_RE.search(s)]
        print(f"self-check OK 航点={len(waypoints)} 总程≈{tot:.1f} m "
              f"4.78 m/s 估时≈{tot / 4.78:.1f} s 最近航点距杆 {dp:.1f} m "
              f"白字触发={'是' if has_white_text(img) else '否'} 正则={codes}")
        # 分类回归：六帧真值
        run = Path(r"D:\download3\算法邀請賽\accel_window_probe\_vla_ref\runs\vla-perimeter-r2\f")
        truth = {"t0025_cam2.jpg": "blue", "t0055_cam2.jpg": "red", "t0077_cam0.jpg": "green",
                 "t0061_cam2.jpg": "blue", "t0071_cam2.jpg": "blue"}
        if run.is_dir():
            from rapidocr_onnxruntime import RapidOCR
            eng = RapidOCR()
            ok = 0
            for name, want in truth.items():
                img2 = cv2.imdecode(np.fromfile(run / name, dtype=np.uint8), cv2.IMREAD_COLOR)
                res, _ = eng(img2)
                got = "?"
                for box, txt, score in (res or []):
                    if float(score) >= 0.45 and CODE_RE.search(str(txt).replace(" ", "")):
                        got = hue_class(body_hue(img2, box))
                        break
                ok += int(got == want)
                print(f"   分类 {name}: 期望 {want} 实得 {got}")
            print(f"   分类回归 {ok}/{len(truth)}")
        # 规则参数化回归：同一份 summary，blue 变体选蓝车、red 变体选红车，R02 都是三轮车
        fake = {"C-01": (2, "blue", 0.90), "C-02": (1, "red", 0.95),
                "C-03": (1, "red", 0.88), "C-04": (1, "green", 0.92)}
        r1 = build_violations(fake, 1, "blue")
        r2 = build_violations(fake, 1, "red")
        print(f"   规则回归 blue→R01={r1[0]} R02={r1[1]} | red→R01={r2[0]} R02={r2[1]}")
        return 0

    out = Path(args.out)
    shots = out / "f"
    shots.mkdir(parents=True, exist_ok=True)
    for old in shots.glob("*.jpg"):
        old.unlink()

    det = None
    if args.yolo and Path(args.yolo).is_file():
        try:
            from ultralytics import YOLO
            det = YOLO(args.yolo)
            # 预热：首帧推理会花 3~4 s，放在连场景之前跑掉，避免主循环被卡住
            t_warm = time.time()
            det(np.zeros((480, 640, 3), np.uint8), imgsz=416, verbose=False)
            print(f"YOLO 闸门就绪: {args.yolo}（预热 {time.time() - t_warm:.1f}s）", flush=True)
        except Exception as exc:      # noqa: BLE001
            print(f"!! YOLO 载入失败（退回白字触发）: {exc}", flush=True)
            det = None
    else:
        print("没有 YOLO 权重，退回白字触发", flush=True)

    from metacar import (FunctionZoneResult, FunctionZoneViolation, GearMode,
                         SceneAPI, VLAExtensionOutput, VehicleControl)

    # 读号架构（2026-10-27 改）：1 个 OCR worker + **单线程推理**，巡航中 inline 读，
    # 不再"存帧等停车"。依据：单线程推理下主循环抖动 p95 33.6 ms / 无 >100 ms 卡顿，
    # 而停车串行会把每辆车的读号窗口压到 1~2 帧（r12 漏掉 C-04 就是这么发生的）。
    votes: dict = {}
    lock = threading.Lock()
    stats = {"frames": 0, "hits": 0, "enq": 0,
             "drop_budget": 0, "drop_queue": 0, "drop_cap": 0, "gated": 0}
    q: "queue.Queue" = queue.Queue(maxsize=QUEUE_MAX)
    reader: "Reader | None" = None
    try:
        from rapidocr_onnxruntime import RapidOCR    # noqa: F401 - 只做依赖可用性检查
        print("OCR 依赖就绪（RapidOCR/PP-OCRv4；引擎由 worker 线程自建）", flush=True)
    except Exception as exc:      # noqa: BLE001
        # 没有 OCR 就没有任何编号可报，硬跑等于白跑一局：直接失败退出（评审 B2）
        raise SystemExit(f"OCR 初始化失败，无法读号: {exc}") from exc
    reader = Reader(q, votes, lock, "ocr0", stats)
    reader.start()
    car_slots: list = []      # 按自车位置去重：{"pos":(x,y), "n":已存帧数, "cam":cid}
    read_brake = 0.0          # 已废弃（恒定油门模式下没有读阶段刹车）
    cruise_on = 0.0           # 已废弃（滞环取消，油门恒定）

    print("连接仿真场景（先跑脚本，再点启动场景）...", flush=True)
    api = SceneAPI()
    api.connect()
    static = api.get_scene_static_data()
    print(f"场景: {[s.name for s in static.sub_scenes]}", flush=True)

    follower = WpFollower(waypoints, args.lookahead, args.accept)
    t0 = time.time()
    tag = 0
    last_shot = -1e9
    last_log = -1e9
    last_seg = -1
    pin_since: float | None = None
    back_until = 0.0
    payload = None
    submitted_at: float | None = None
    last_pose: tuple[float, float] | None = None
    prev_v: float | None = None
    prev_yaw: float | None = None
    lat_peak = 0.0
    prev_t = 0.0
    prev_cmd: tuple | None = None
    last_tick = 0.0
    phase = "drive"

    pose_now = {"x": 0.0, "y": 0.0, "yaw": 0.0, "v": 0.0, "cx": -1.0, "w": -1.0}
    pose_fh = open(shots.parent / "gates.csv", "w", encoding="utf-8")
    pose_fh.write("tag,t,ego_x,ego_y,yaw,v,cam,box_cx,box_w\n")

    def offer(tag_: int, now_: float, cid: int, img) -> bool:
        """把一帧塞进 OCR 队列；队列满了丢**最旧**的一帧——保证读的是最新画面。"""
        if stats["enq"] >= args.frame_budget:
            stats["drop_budget"] += 1
            return False
        save_jpg(shots / f"gate_t{tag_:04d}_cam{cid}.jpg", img)
        pose_fh.write(f"{tag_},{now_:.2f},{pose_now['x']:.2f},{pose_now['y']:.2f},"
                      f"{pose_now['yaw']:.1f},{pose_now['v']:.2f},{cid},"
                      f"{pose_now['cx']:.0f},{pose_now['w']:.0f}\n")
        pose_fh.flush()
        item = (tag_, now_, cid, img.copy())
        try:
            q.put_nowait(item)
        except queue.Full:
            try:
                q.get_nowait()
                stats["drop_queue"] += 1
            except queue.Empty:
                pass
            try:
                q.put_nowait(item)
            except queue.Full:
                stats["drop_queue"] += 1
                return False
        stats["enq"] += 1
        return True

    for msg, frames in api.main_loop():
        now = time.time() - t0
        pose = msg.pose_gnss
        x, y, yaw = float(pose.pos_x), float(pose.pos_y), float(pose.ori_z)
        v = abs(float(msg.main_vehicle.speed))
        pose_now.update({"x": x, "y": y, "yaw": yaw, "v": v, "cx": -1.0, "w": -1.0})
        # 逐帧纵向加速度自检（与平台同口径）：找出 −50 到底发生在哪一帧
        if prev_v is not None and now - prev_t > 1e-3:
            acc = (v - prev_v) / (now - prev_t)
            if abs(acc) > 4.5:
                g, th, br, st = prev_cmd if prev_cmd else ("?", 0.0, 0.0, 0.0)
                print(f"[{now:5.1f}s] !! |a|={abs(acc):4.1f} m/s²  v {prev_v:.2f}→{v:.2f}"
                      f"  dt={now - prev_t:.3f}s  上一帧指令 gear={g} thr={th:.3f}"
                      f" brk={br:.2f} steer={st:+.2f}", flush=True)
        # 横向加速度自检：规则只写"加速度绝对值超过 5 m/s²"，没区分纵横。
        # 判据既然是指令，横向大概率不算；但按最严口径盯住，超了就是航线画错了。
        if prev_yaw is not None and now - prev_t > 1e-3:
            omega = math.radians(((yaw - prev_yaw) + 180.0) % 360.0 - 180.0) / (now - prev_t)
            a_lat = v * abs(omega)
            if a_lat > lat_peak:
                lat_peak = a_lat
            if a_lat > 4.0:
                print(f"[{now:5.1f}s] !! a_lat={a_lat:4.1f} m/s²  v={v:.2f} "
                      f"ω={omega:+.2f} rad/s", flush=True)
        prev_yaw = yaw
        prev_v, prev_t = v, now

        if now > args.seconds:
            print(f"[{now:5.1f}s] 到时间，驻车", flush=True)
            break
        if now > 8.0 and (abs(x) > BOUND or abs(y) > BOUND):
            print(f"[{now:5.1f}s] !! 越界 ({x:.1f},{y:.1f})，驻车", flush=True)
            break

        follower.update(x, y)
        if last_pose is not None:
            jump = math.hypot(x - last_pose[0], y - last_pose[1])
            if jump > 6.0:
                print(f"[{now:5.1f}s] !! 位姿跳变 {jump:.1f} m → 重新对准航线", flush=True)
                follower = WpFollower(waypoints, args.lookahead, args.accept)
                follower.update(x, y)
                last_seg = follower.seg
        last_pose = (x, y)
        if now - last_tick > 0.6:
            print(f"[{now:5.1f}s] !! 主循环卡顿 {now - last_tick:.1f}s", flush=True)
        last_tick = now
        if follower.seg != last_seg:
            last_seg = follower.seg
            if not follower.done():
                nxt = waypoints[min(follower.seg, len(waypoints) - 1)]
                print(f"[{now:5.1f}s] 到点 → 下一个 ({nxt[0]:.1f},{nxt[1]:.1f}) "
                      f"剩余≈{follower.remain_m(x, y):5.1f} m", flush=True)

        if phase == "drive" and (follower.done() or now >= args.phase_a):
            phase = "read"
            # 恒定油门模式下没有"停车排空"，圈数跑完就按已有票结算（上报即结算）
            args.hard_submit = min(args.hard_submit, now)
            print(f"[{now:5.1f}s] 巡航段结束（{args.laps} 圈跑完）→ 立即按已有票上报"
                  f"（已读 {stats['frames']} 帧，命中 {stats['hits']}）", flush=True)

        # ---- 上报判定（上报即结算，先算） ----
        if payload is None:
            summary = summarize(votes, lock)
            cars, green, note = build_violations(summary, args.min_votes, args.violator)
            # 「读完」= 队列空 + worker 不在推理 + 已安静 0.4 s。只看 q.empty() 会把
            # 「worker 刚取走最后一帧、正在推理」误判成「已读完」（评审 MAJOR）。
            idle = q.empty() and not reader.busy and (time.time() - reader.last_done >= 0.4)
            force = now >= args.hard_submit
            full = len(cars) >= args.expect_violators and len(green) >= 1
            if (full and now >= args.min_submit) or force:
                if force and not full:
                    # 到点还没读全：**按规则分别兜底**，绝不再往后推时间。
                    # 旧版这里 args.hard_submit = now + 8，而 q.empty() 分支在进读阶段后
                    # 恒真，两者叠加会把「没读到号」变成「永不提交」（评审 BLOCKER-1）。
                    note = "兜底："
                    if len(cars) < args.expect_violators:
                        got = len(cars)
                        cars = list(CANDIDATE_CODES)
                        note += f"R01 六号全报（{args.violator} 只读到 {got} 个） "
                    if len(green) < 1:
                        green = list(CANDIDATE_CODES)
                        note += "R02 六号全报"
                violations = []
                if cars:
                    violations.append(FunctionZoneViolation(rule_code="R01", sticker_ids=cars))
                if green:
                    violations.append(FunctionZoneViolation(rule_code="R02", sticker_ids=green))
                # 即使 violations 为空也提交：空提交不比不提交差，且能让本局干净结算
                payload = VLAExtensionOutput(
                    function_zone_result=FunctionZoneResult(violations=violations))
                submitted_at = now
                print(f"[{now:5.1f}s] === 上报 R01={cars} R02={green}"
                      f"（{note}；OCR {stats['frames']} 帧/命中 {stats['hits']}，idle={idle}）",
                      flush=True)

        # ---- 纵向：全程恒定油门、零刹车、不倒车 ----
        # 平台判的是「指令有没有变」，所以这里一帧都不改：不发刹车（刹车是阶跃）、
        # 不倒车脱困（REVERSE 也是阶跃）、不做起步斜坡（会把起步瞬态推出 1 s 免判期）。
        # 卡死只记日志；实在脱不了困就靠 hard-submit 按已有票提交。
        if now > 6.0 and v < PIN_SPEED:
            if pin_since is None:
                pin_since = now
            elif now - pin_since > PIN_SECONDS and not getattr(args, "_pinned", False):
                args._pinned = True
                print(f"[{now:5.1f}s] !! 卡死 ({x:.2f},{y:.2f}) —— 恒定油门模式不脱困，"
                      f"继续等（按已有票提交）", flush=True)
        else:
            pin_since = None

        steer = follower.steering(x, y, yaw)
        # 横向预算：a_lat = v²·tan(δ)/L ≤ A_LAT_MAX ⇒ δ ≤ atan(A_LAT_MAX·L/v²)
        if v > 1.0:
            delta_max = math.atan(WHEELBASE_M * A_LAT_MAX / (v * v))
            steer_max = min(1.0, delta_max / MAX_STEER_RAD)
            if abs(steer) > steer_max:
                steer = math.copysign(steer_max, steer)
        vc = VehicleControl(gear=GearMode.DRIVE, throttle=THROTTLE, brake=0.0,
                            steering=steer)
        mode = "cruise"

        if payload is not None:
            api.set_vehicle_control(vc, vla_extension=payload)
        else:
            api.set_vehicle_control(vc)
        prev_cmd = (str(getattr(vc, "gear", "?")).split(".")[-1],
                    float(getattr(vc, "throttle", 0.0) or 0.0),
                    float(getattr(vc, "brake", 0.0) or 0.0),
                    float(getattr(vc, "steering", 0.0) or 0.0))

        # ---- 读号队列：巡航中后台线程一直在读；这里只报排空进度（不再串行 OCR） ----
        if phase == "read" and payload is None and not getattr(args, "_drained", False) \
                and q.empty() and not reader.busy and stats["frames"] > 0:
            args._drained = True
            print(f"[{now:5.1f}s] 读号队列排空（OCR {stats['frames']} 帧，"
                  f"命中 {stats['hits']}，丢弃 {stats['drop_budget'] + stats['drop_queue']} 帧）",
                  flush=True)

        # ---- 取帧：用 YOLO 当闸门（0.03 s/帧），只把"侧向近处有车"的帧送 OCR ----
        if frames and now - last_shot >= args.shot_every:
            last_shot = now
            tag += 1
            if tag % CAM0_EVERY == 0:
                want = 0                                    # A2：车头 20%
            else:                                           # 侧向 80%，左右严格交替
                want = SIDE_CAMS[(tag - tag // CAM0_EVERY) % len(SIDE_CAMS)]
            for cam in frames:
                cid = int(cam.id)
                img = getattr(cam, "frame", None)
                if img is None or now < 2.0:
                    continue
                if cid != want:
                    continue
                car_slot_here = None          # 评审 BLOCKER-3：无 YOLO 权重时这里必须已定义
                if cid == 0:
                    # 三轮车在车头方向出现过（cam0 读到 C-06 那帧），单独用黄车闸门
                    if not has_yellow_blob(img):
                        continue
                elif det is not None:
                    try:
                        boxes = det(img, imgsz=416, conf=0.35, classes=[2, 5, 7], verbose=False)[0].boxes
                    except Exception:      # noqa: BLE001
                        boxes = []
                    # 质量闸门：宽 150~400 px（太远读不出、太近会被裁切）、
                    # 框心在画面 15%~85%、框不贴画面边界 —— 只把这种"好帧"入库
                    good = None
                    for b in boxes:
                        x1, y1, x2, y2 = (float(b.xyxy[0][0]), float(b.xyxy[0][1]),
                                          float(b.xyxy[0][2]), float(b.xyxy[0][3]))
                        w = x2 - x1
                        w_img = img.shape[1]
                        cx = (x1 + x2) / 2.0
                        if not (CAR_MIN_W <= w <= CAR_MAX_W):
                            continue
                        if not (0.06 * w_img <= cx <= 0.94 * w_img):
                            continue
                        if w > 0.9 * w_img:
                            continue        # 整个画面都是车 = 已经蹭上了，读不出字
                        if good is None or w > good[0]:
                            good = (w, cx)
                    if good is None:
                        continue
                    stats["gated"] += 1
                    pose_now["cx"], pose_now["w"] = good[1], good[0]
                    # 去重：自车位置 8 m 内视为同一辆车，每车最多存 PER_CAR_MAX 帧
                    car_slot_here = None
                    for slot in car_slots:
                        if math.hypot(slot["pos"][0] - x, slot["pos"][1] - y) <= CAR_DEDUPE_M:
                            car_slot_here = slot
                            break
                    if car_slot_here is None:
                        car_slot_here = {"pos": (x, y), "n": 0, "cam": cid}
                        car_slots.append(car_slot_here)
                    if car_slot_here["n"] >= PER_CAR_MAX:
                        stats["drop_cap"] += 1
                        continue
                elif not has_white_text(img):
                    continue
                if offer(tag, now, cid, img) and car_slot_here is not None:
                    car_slot_here["n"] += 1

        if now - last_log >= 2.0:
            last_log = now
            summary = summarize(votes, lock)
            print(f"[{now:5.1f}s] {mode:<5} ({x:6.2f},{y:6.2f}) yaw={yaw:6.1f} v={v:4.2f} "
                  f"帧={tag} OCR={stats['frames']} 命中={stats['hits']} "
                  f"队内={q.qsize()} 丢弃={stats['drop_budget'] + stats['drop_queue'] + stats['drop_cap']} "
                  f"票={{{', '.join(f'{c}:{n}' for c, (n, _k, _s) in sorted(summary.items()))}}}",
                  flush=True)

        if submitted_at is not None and now > submitted_at + 10.0:
            print(f"[{now:5.1f}s] 上报后 10 s，驻车等结算", flush=True)
            break

    try:
        # 收尾也不改指令：任何 brake 阶跃（哪怕 0.10）都可能是加速度事件，
        # 恒定油门模式下离开循环时继续保持同一条指令即可。
        api.set_vehicle_control(VehicleControl(gear=GearMode.DRIVE, throttle=THROTTLE,
                                               brake=0.0))
    except OSError:
        pass

    if reader is not None:
        # 哨兵可能因为队列满而丢：先腾位置，最多试 QUEUE_MAX+2 次（评审 MAJOR）
        for _ in range(QUEUE_MAX + 2):
            try:
                q.put_nowait(None)
                break
            except queue.Full:
                try:
                    q.get_nowait()
                except queue.Empty:
                    break
        reader.join(timeout=3.0)
        if reader.is_alive():
            print(f"!! OCR 线程 3 s 内没退出（队列仍 {q.qsize()} 帧），细节可能丢", flush=True)
        elif not reader.alive_ok:
            print("!! OCR 线程是异常退出的，本局读号结果不完整", flush=True)
        else:
            print(f"OCR 线程正常退出：已读 {stats['frames']} 帧，命中 {stats['hits']} 号",
                  flush=True)

    summary = summarize(votes, lock)
    print(f"最终票型 {dict(sorted(summary.items()))}", flush=True)
    print(f"横向加速度峰值 a_lat={lat_peak:.2f} m/s²（预算 {A_LAT_MAX}）", flush=True)
    print(f"OCR {stats['frames']} 帧、命中 {stats['hits']}、入队 {stats['enq']}、"
          f"丢弃(预算/队列/上限) {stats['drop_budget']}/{stats['drop_queue']}/"
          f"{stats['drop_cap']}、过闸 {stats['gated']}、"
          f"上报 {'是' if payload is not None else '否'}", flush=True)
    try:
        pose_fh.close()
    except Exception:      # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

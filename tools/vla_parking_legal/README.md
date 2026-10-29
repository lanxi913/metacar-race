# 停车区任务二：合法区补集提交

本目录是停车区任务二 `vla_IllegalParkingDetection_02` 的一个独立最小实现，正式提交在
`feat/vla-parking-legal` 分支，和 `feat/vla-parking`（PR #46）分开。

## 思路

场景固定 6 辆车，编号为 `B-01`~`B-06`，其中：

- 2 辆停在区外合法车位；
- 4 辆停在禁停区，是要求提交的违规车。

因此不需要在禁停区里逐辆读号，只要在合法区读到 2 个编号，剩下的 4 个编号就是违规车：

```text
illegal = {B-01..B-06} - {两辆合法车编号}
```

合法区是白色车位线的紫色车区，禁停区是 `y > 6.1` 的黄色网格区。路线全程 `y <= -3.6`，
不会进入禁停区，也不会把非法车编号混进合法票。

## 路线

`legal_stack.py` 的 `LAP_WP` 是一条向南的 U 形开环：

```text
(0,0) -> (0,-4) -> (50,-4) -> (50,-20) -> (-5,-20) -> (-5,-3.6) -> (50,-3.6)
```

- 两次东向经过合法区：`y=-4.0` 与 `y=-3.6`，相距 0.4 m，不重叠；
- 两端用 R=8 的 180° 圆弧连接，路径总长约 181 m；
- 4.78 m/s 下约 38~51 s 跑完（含起步），90 s 上限有余量；
- 合法车最近约 2.0 m，非法车最近约 17 m；
- 第 0 帧固定油门 `0.006`，全程不改油门、不发过程刹车；结束时才刹停。

## 识别与提交

`legal_probe.py` 保留 GitHub `f0cca073` 的整帧 RapidOCR/票桶逻辑，只按合法区做两遍扫描：

1. 每路相机独立 worker，`--dup 2`，共 4 个 OCR 进程；
2. 只保留 `y < 3` 的命中（合法区），禁停区命中不参与；
3. 一圈结束时取票数前二作为合法编号；
4. 如果不足 2 个合法编号，不交卷；
5. 否则补集提交 4 个编号。

## 运行

先启动脚本，等终端出现 `waiting scene ...`，再在客户端点
「停车区违规检测任务（二）」的启动场景：

```powershell
$py = "<repo>\.venv\Scripts\python.exe"
& $py -u tools\vla_parking_legal\legal_probe.py `
  --out runs\legal-u-r1 --tag legal-u-r1 `
  --laps 1 --seconds 90 --submit-complement 1 `
  --yolo 1 --every 0.10 --dup 2 --cams 2,3 `
  --min-votes 2 --min-score 0.50 --region-only 0 --record 1
```

Windows 下 `multiprocessing.Queue` 在受限沙箱会报 `WinError 5`，需要用完整权限启动。

## 实测

2026-10-29，同一份代码连续三局：

| 局 | 合法票 | 提交 | 平台 |
|---|---|---|---|
| legal-u-r1 | B-05 / B-06 | B-01,B-02,B-03,B-04 | 1000 |
| legal-u-r2 | B-05 / B-04 | B-01,B-02,B-03,B-06 | 1000 |
| legal-u-r3 | B-02 / B-05 | B-01,B-03,B-04,B-06 | 1000 |

三局都在约 50.7 s 完成，没有碰撞、没有进入禁停区。编号每局重排，所以代码不写死编号，
只写死合法区边界和路线。

## 依赖

`vla2/` 是从 `feat/vla-parking`（PR #46，`f0cca073`）里取的最小依赖：`cars.py`、`vision.py`、
`shotread.py`、`ocr2.py`。等 PR #46 合并后应改为共享 `vla2`，避免两份副本。

额外依赖：

```text
rapidocr-onnxruntime
ultralytics
opencv-python
```

YOLO 权重优先读 `VLA_YOLO_WEIGHTS`；没有时 `vla2/shotread.py` 会回退到本机
`best.pt` / 通用 `yolov8n.pt` 路径。

## 说明

- `legal_slots.json` 只是两遍探针的观测记录（`probe_only=true`），代码提交不依赖它；
- 本目录的路线和合法区边界来自本训练场景标定，正式赛题泛化时要从
  `get_scene_static_data()` 重新推导；
- 不修改 `main`，本目录只通过 `feat/vla-parking-legal` 的 PR 合入。

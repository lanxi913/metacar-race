# VLA 停车区违规检测（任务二）—— 视觉与巡检落地记录

> 场景：`vla_IllegalParkingDetection_02`（客户端卡片「停车区违规检测任务（二）」）
> 分支：`feat/vla-parking`　（从 `feat/continuous4` 拉出，不改 main）
> 记录日期：2026-10-26 深夜 ~ 2026-10-27 凌晨

---

## 1. 本阶段做成了什么（一句话）

1. **贴边巡检跑通了**：真车在禁停区**内侧**沿边线跑完整整一圈，73.4 s / 126 m / 进场后**零越界**、零碰撞、零倒车。
2. **视觉换成 YOLOv8n 并训练成功**：自建 496 帧自动标注数据集，CPU 12 分钟训完，**车 mAP50 = 0.893**，推理 15 ms/帧。
3. **确认编号有两条读法**：车身侧面白字 `B-05`，以及**车头/车顶的小蓝牌**（同样印着编号）。

![这一圈的俯视轨迹与速度曲线](lap_topview.jpg)

*图 1：真车跑次 `stack5` 的俯视回放。红框=禁停区（`vla_extension.regions` 给的 `RESTRICTED_PARKING`），黄线=参考线，彩色折线=实车轨迹（绿/橙/蓝按速度分段），右下=速度曲线。原始数据见同目录 `stack5_lap.csv`（1164 帧）。*

---

## 2. 现场运行逻辑（这一圈是怎么跑出来的）

### 2.1 三层结构

| 层 | 职责 | 实现 | 关键常数（来源） |
|---|---|---|---|
| ① 路径层 | 定"我该在哪条线上" | 禁停区矩形**向内缩 `E_REF`**，四角用 `R_CORNER` 圆弧倒角 → 一条 **252 点 / 126.0 m** 的圆角矩形参考线 | `E_REF=2.6 m`（= 车半宽 0.95 + 余量）、`R_CORNER=6.5 m`（= 最小转弯半径 4.6 × 1.4） |
| ② 决策层 | 定"现在该多快" | 三档节奏：直道 `V_STRAIGHT`；弯道 `v=√(A_LAT/R)`；进场先对准参考线 | `V_STRAIGHT=2.5 m/s`、`A_LAT=0.30`、`A_ACC=1.6 m/s²` |
| ③ 执行层 | 定"舵打多少" | 曲率前馈 + Stanley 横向 + 航向项，再限斜率 | `K_HEAD=0.020/°`、`K_CROSS=1.3`、`STEER_SLEW=0.03/帧`、`V_STANLEY_FLOOR=1.0` |

**控制律**（`vla2/stack.py::Arbiter.decide`）：

```
kappa      = 参考线前方 3~6 m 的三点圆曲率          # 给前馈
kappa_v    = max(kappa, 前方 6~12 m 的曲率)          # 给限速（提前压速度，否则进弯跑宽）
v_t        = min(V_STRAIGHT, sqrt(A_LAT / kappa_v))
steer_ff   = atan(WHEELBASE * kappa) / MAX_STEER
lat_term   = atan2(K_CROSS * e_lat, max(v, V_STANLEY_FLOOR))
steer_raw  = steer_ff + (radians(e_head) + lat_term) / MAX_STEER
steer      = prev + clamp(steer_raw - prev, ±STEER_SLEW)   # 只做斜率限幅
```

> 踩过的两个坑（都写进代码注释了）：
> 1. 横向项若写成 `K_CROSS * e_lat`（线性），在 13 m 大偏差时只有 ±1.6 却与航向项反向抵消 → 必须用 **Stanley 的 atan2 形式**；
> 2. 斜率限幅**再串**一阶低通，两者相乘会把实际起舵速率压到 1/3（进场要 13 s 才偏得过来）→ 只保留斜率限幅。

### 2.2 离线验收（四起点，全 PASS）

| 起点 | 一圈 | 耗时 | 出界帧 | \|e_lat\| 平均 / 最差 |
|---|---|---|---|---|
| (0,0) 朝东 | 126.0 m | 80.1 s | **0** | 0.44 / 1.08 m |
| (2,2) 朝北 | 126.0 m | 81.7 s | **0** | 0.45 / 1.09 m |
| 区内 (30,40) 朝西 | 126.0 m | 74.2 s | **0** | 0.46 / 1.14 m |
| 区内 (40,10) 朝南 | 126.0 m | 72.5 s | **0** | 0.51 / 2.64 m |

验收口径按**物理要求**定（不是"贴合到某个理想线"）：①不越界；②平均偏差 ≤0.6 m；③最差偏差 ≤ `E_REF+0.5`（任何时刻都不跨过涂装线）；④不超速；⑤`|a| ≤ 1.6 m/s²`。

### 2.3 真车跑次（`stack5`）

- 起止：t=0 起点 (−0.15, 0.00) → 73.4 s 收工 (15.32, 8.39)
- `s_along = 128.5 m`（周长 126.0 m）、`lap_done = true`
- 进场：t=0~5.99 s 在场外（起点本来就在禁停区外），**t=6.06 s 进入后 0 帧越界**（x≤44.08、y≤43.99）
- 速度：1.28~**3.40 m/s**（4.6~12.2 km/h，任务限速 20 km/h）
- 横向误差实测：+0.00 / −0.25 / −0.28 / −0.60 / −0.64 m
- 事件：0 次碰撞、0 次卡死、0 次倒车

**时间账**：任务总限时 100 s，这一圈用掉 73 s —— 所以"绕一圈摸清车在哪"这件事，必须在**第二件事（读号）**之前就把时间留出来（计划：巡检 55~70 s + 读号在巡检过程中顺带完成）。

---

## 3. 视觉：为什么要换 YOLO，怎么训的

### 3.1 起点：传统颜色连通域检测（能跑，但脆弱）

`vla2/vision.py::detect_cars` 用 HSV 车漆掩膜 + 形状过滤（`min_area=380`、填充率 ≥0.40、宽高比 0.25~4.0、拒绝 `y+h>0.94H` 的车前地面）。效果：近处 3320/2134/980 面积的三辆车全中，误检从 21 降到 3 —— **但它依赖"车是深色高饱和"这一条现场假设**，光照/远距离/侧倾一变就掉点。

### 3.2 数据集（`tools/vla2_make_dataset.py`）

| 项 | 值 |
|---|---|
| 来源 | `vla_imgss5/6/7` + `vla_imgs` 四个既有巡检目录抽样 |
| 图像 | **496 帧**（每目录 150 帧，按等间隔抽样） |
| 标注方式 | **自动打标**：车框来自上面的颜色检测器；蓝牌框来自 HSV 蓝掩膜 + 形状过滤（宽 6~60 px、高 4~28 px、宽高比 1.1~5、填充率 ≥0.45） |
| 框数 | 车 **874**、车牌 **2991** |
| 划分 | train 85% / val 15% |

### 3.3 训练（`YOLOv8n`，CPU 12 分钟）

```
model = yolov8n.pt        imgsz = 384      epochs = 20
batch = 16                workers = 8      device = cpu
```

| 类别 | P | R | mAP50 | mAP50-95 |
|---|---|---|---|---|
| **car** | 0.858 | 0.806 | **0.893** | 0.722 |
| plate | 0.828 | 0.455 | 0.486 | 0.245 |
| 全部 | 0.843 | 0.630 | 0.690 | 0.484 |

- 训练总耗时 **0.203 h（约 12 分钟）**，推理 **15 ms/帧**（30 Hz 实时够用）
- 权重：`D:\vla_runs\vla2_car\weights\best.pt`（6.5 MB，**不入库**，见 `.gitignore` 里的 `vla2/models/*`）
- plate 类偏弱是可预期的：它的标签是"颜色掩膜自动标的"，本身噪声大。**下一轮**要用"YOLO 车框 + 蓝牌候选 → 人工抽检 50 张修正"的半自动标注重训

![YOLO 检测实拍](yolo_detect_sample.jpg)

*图 2：训好的 `best.pt` 在实拍帧上的效果 —— `B-05` 那辆车框得准（conf 0.45），远处那辆也抓到了（0.49）。*

### 3.4 关键现场发现：编号有两条读法

![车身上的两处编号](car_label_closeup.jpg)

*图 3：把最大的几个车框裁出来放大看 —— ①车身侧面白字 `B-05`（字高约 0.4 m）；②**车头/车顶上方的小蓝牌**，牌子上同样印着编号（`B-05`、`B-01`…）。*

所以读号策略定为**双通道**：

1. **主通道**：YOLO 框车 → 在车框上部找**蓝色小牌** → 切图 → 模板匹配读字（牌子对比度高、位置基本固定）
2. **备份通道**：蓝牌读不出时，退回读**车身侧面白字**（已实测 `B-03`、`B-05` 能读对，相似度 0.87）

实测可读窗口：7 m 处字高 23 px、20 m 处 4~6 px ⇒ **6~12 m 是甜区**，且必须是**侧面视角**（贴脸/正前方读不到）。

---

## 4. 复现方法

```powershell
# 0) 环境
cd D:\无人算法\metacar-race
$env:PYTHONPATH = 'D:\无人算法\metacar-race'
$env:PYTHONIOENCODING = 'utf-8'

# 1) 离线验收这一圈的控制律（四个起点，秒级，不跑仿真）
.\.venv\Scripts\python.exe vla2\stack_sim.py

# 2) 真车跑一圈（先起脚本，再让客户端点“启动场景”）
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force
.\tools\auto_scene.ps1 -Script 'vla2\stack.py' `
  -SceneCp '505C,8F66,533A,8FDD,89C4,68C0,6D4B,4EFB,52A1,FF08,4E8C,FF09' `
  -TimeoutSeconds 300

# 3) 把跑次画成俯视回放
.\.venv\Scripts\python.exe tools\vla2_lap_view.py --csv docs\vla\stack5_lap.csv --out lap.jpg

# 4) 重建数据集 + 重训（约 12 分钟 CPU）
.\.venv\Scripts\python.exe tools\vla2_make_dataset.py --out D:\vla_ds `
  --dirs "D:\无人算法\工作区\vla_imgss5;D:\无人算法\工作区\vla_imgss6;D:\无人算法\工作区\vla_imgss7;D:\无人算法\工作区\vla_imgs" --per-dir 150
.\.venv\Scripts\python.exe -c "from ultralytics import YOLO; YOLO('vla2/models/yolov8n.pt').train(data=r'D:\vla_ds\data.yaml', imgsz=384, epochs=20, batch=16, workers=8, device='cpu', project=r'D:\vla_runs', name='vla2_car')"
```

> 注意：客户端场景卡片名必须传**完整标题**的码点（`停车区违规检测任务（二）`），只传子串会找不到卡片。

---

## 5. 文件清单

| 文件 | 作用 |
|---|---|
| `vla2/stack.py` | 三层控制器 + SceneAPI 入口（本次跑通的这一圈） |
| `vla2/stack_sim.py` | 离线验收台（自行车模型、四起点、出界/误差统计） |
| `vla2/vision.py` | 传统检测（颜色连通域）、地面分类、编号 OCR（模板匹配） |
| `vla2/read.py` / `vla2/mission.py` | 读号子任务、合并状态机（离线件，尚未接真车） |
| `tools/vla2_make_dataset.py` | 从既有帧自动打标生成 YOLO 数据集 |
| `tools/vla2_crop_mosaic.py` | 裁剪放大车框（看编号在哪） |
| `tools/vla2_lap_view.py` | 把 run.csv 画成俯视回放图 |
| `docs/vla/stack5_lap.csv` | 真车 1164 帧原始轨迹 |
| `docs/vla/{lap_topview,yolo_detect_sample,car_label_closeup}.jpg` | 本文三张图 |

---

## 6. 还没做完（明天继续）

1. **plate 类重训**：半自动标注（YOLO 车框 + 蓝牌候选 + 人工抽检 50 张）→ 目标 mAP50 ≥ 0.8
2. **接读号**：车框 → 蓝牌/白字 → 切字 → 模板匹配 → 多帧投票（`vla2/read.py` + `mission.py` 已有离线件）
3. **合并一次跑**：巡检 73 s 内顺带读号 → 85 s 强制交卷（提交一次定局）
4. **平台的 `vla_extension` 提交**：`api.set_vehicle_control(vc, VLAExtensionOutput(parking_result=ParkingResult(violating_sticker_ids=[...])))`
5. 速度锯齿（±0.2 m/s）可换成连续油门斜坡，进一步平顺

---

## 7. 关键常数速查

```
WHEELBASE=2.6  MAX_STEER=0.6  V_CAP=5.5
E_REF=2.6      R_CORNER=6.5   V_STRAIGHT=2.5   A_LAT=0.30   A_ACC=1.6
K_HEAD=0.020/deg   K_CROSS=1.3   V_STANLEY_FLOOR=1.0   STEER_SLEW=0.03
THR_MAX=0.35   BRK_MAX=0.06
参考线：252 点 / 126.0 m（禁停区 40×40，内缩 2.6 m，四角 R=6.5 圆角）
```

禁停区（现场实测）：`x∈[6.2, 46.2]`、`y∈[6.1, 46.1]`，面积 40 m × 40 m；
合法车位带：`x∈[11.2, 41.2]`、`y∈[-3, 3]`（停满车，是"外侧绕行"走不通的原因）。

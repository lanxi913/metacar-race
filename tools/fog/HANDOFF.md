# 港口迷雾 —— 交接说明（2026-10-30 17:20，给新窗口的 AI）

> 比赛 2026-11-01。这份是"我脑子里的全部状态"，读完就能接着干。

## 0. 一句话现状

**8 项任务成果已经稳了**：白天连跑 6 局全成（17:17~18:21），晚上把 6 个"偶发但致命"的坑填完
之后又连跑 2 局全成（21:27 A 382.4 / B 521.6、21:37 A 382.3 / B 521.4，违规 11~13 条、**0 碰撞**）。
代码在 **`feat/fog-port-8of8`**（HEAD `5bcacf2`，已推 GitHub），快照 `fog_mission.py.gold9`。
注意：远端的 `feat/fog-port` 是另一条线的实现（`fa884ff`，600/1000），和这版**不是一套代码**，别混。

### 三个"接口脾气"（比调参重要，晚上摸清的）

1. **`request_entry` 只在"车正开在进车道上、且离路口中心 ≤12 m"时判 eligible**；
   站着申请一定回 `is not on an eligible intersection approach`。所以必须**边开边申请**，
   到申请线 `(728.79,−30.5)` 停死，没批就放弃。
2. **`release` 的出口是一个很小的箱子**：`x ≥ 728.75`、`y ∈ [−12, −9.5]`。
   穿闸分两段：`−31→−14` 走 1.6 m/s，`−15→−8` 走 0.5 m/s（采样 0.05 m）。
3. **没有通行权进 6 m 互斥区 = 整局判结束**（比撞墙狠，撞墙只是锁车）。
   所以 `cross_one` 里"没 granted 绝不往北开"，A 没过闸时 B 也不许申请。

## 1. 怎么跑（一条命令）

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog\fog_run.ps1" -Seconds 620 -Deliver 1
```

它会：清场 → 起客户端 → UIA 点「港口迷雾」→ 等 9100 → 跑 `fog_mission.py`
→ 看门狗 260 s 无输出判卡死。每局约 11 分钟，日志在 `.train\fog800\logs\m_MMDD_HHMMSS\`。

一屏看结论（**必须带 UTF-8，否则中文全乱码**）：

```powershell
$env:PYTHONIOENCODING="utf-8"; [Console]::OutputEncoding=[System.Text.Encoding]::UTF8
& "C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race\.venv\Scripts\python.exe" `
  "C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog\fog_report.py" `
  "<日志目录>"
```

## 2. 现在的流程（`main()` 里四段）

```
阶段 0  stage_signals      Car_C 跑 A(6.6s) → B(138s) → C(164s) 三个信号
阶段 1  stage_scan         Car_C 停在 10.5 m 内报 scan；扫完就地刹住，之后不再动
阶段 2  stage_cross        ← 今天重排的核心，见下
阶段 3  stage_deliver_v2   兜底：只补阶段 2 没交付的泊位（正常一条都不跑）
```

`stage_cross` 的顺序（**别改回原来的"A、B 都过闸之后才交付"**）：

| 步 | 内容 | 实测时刻 |
|---|---|---|
| 3.0 | 把扫到的路障记成 8 m 禁区（`ctx.dyn_avoid`），Car_C 就地刹住 | 174 s |
| 3.1 | `_pre()` 并行：A→闸口待命、B→后方 19 m 备用待命（错峰 8 s）、D→泊位 A 见证位 | 174→252 |
| 3.2 | `cross_one("Car_A")` 过闸 | 288.6 s |
| 3.3 | A→泊位 A 南侧 28 m 助跑位，然后**立刻交付**（不等 B） | 366.2 s |
| 3.4 | `cross_one("Car_B")` 过闸（B 早在待命位） | 402.6 s |
| 3.5 | B→泊位 B 南侧 28 m、**A**（交付完闲着的）→泊位 B 北侧见证位 | 402→455 |
| 3.6 | B 交付 | 498.3 s |

## 3. 代码结构（都在 `.train/fog800/tools/fog/`，**未提交**）

| 文件 | 作用 |
|---|---|
| `fog_mission.py` | 主流程。**文件末尾（1359 行起）有一大段"覆盖块"**，重新定义了 drive_to / drive_polyline / stage_signals / stage_cross / deliver_berth / main 等，靠"后定义覆盖先定义"生效。**改逻辑改后面那段**；前面 259 行的旧 `stage_signals` 是死代码，别往里写东西（今天就踩过一次） |
| `fog_mission.py.gold6` | **当前最好版本**（17:17 那局 8/8、6 条违规） |
| `fog_mission.py.gold3/4/5` | 今天中间态，回滚用 |
| `fog_mission.py.gold2` | 今天 14:39 的版本（B 泊位还超时的那版，别用） |
| `fog_mission.py.gold` | 昨天 8/8 那版（B 泊位只剩 20~40 s 余量） |
| `fog_run5.py` | `LaneDriver`：弧长投影 + 纯跟踪 + 曲率前馈 + 油门/刹车斜坡 + 卡死自救。`--selftest` 离线自测 |
| `fog_safety.py` | 防撞层：`guard()` 按 0.6 s 预测位置分级；`path_clearance()` 量路线净空 |
| `fog_run3.py` | `pose_of`（**位移法航向**）、`reset_pose`、`heading_ready`、`KEY_RE`、`WHEELBASE_RATIO=0.55`、`MAX_STEER=0.62` |
| `fog_route.py` / `fog_nav.py` | 路网路由 / 栅格 A*（`nav_path(ctx, frm, to, avoid=[...])`，会自动叠加 `ctx.dyn_avoid`） |
| `fog_report.py` | 一局日志压成一屏结论 |
| `fog_run.ps1` | 起场景 + 跑任务 + 看门狗（**必须 UTF-8 with BOM**） |

## 4. 硬事实（都是实测，别再踩）

- **撞一下就整车锁死**（油门 0.62、倒挡都没反应），所以"别撞墙"是第一优先级。
- **车走不动立刻倒车**，倒车后必须 `reset_pose` + `drv.reset_heading()`；但**倒车冲得极快**
  （0.32 油门 3 s 就是 27 m），脱困只能用 1.2~1.4 s，别拿来当转向道具。
- **正向满舵 2~3°/s，倒车满舵约 50°/s**（差 20 倍）。要用倒车摆头就看看会不会冲出去。
- **位移航向（`pose_of`）可信，`ori_z` 不可信**：同一辆车两者能差 100° 以上，
  而巡检车（C/D）的观测是 360°、运输车（A/B）**必须车头对着泊位（±60°）才给观测**。
  所以 `_stage_at` 把"车头差 ≤40°"当硬条件 —— 这是今天把 A 的交付从 426 s 提前到 366 s 的关键。
- **`get_observation` 会返回空快照**（同一位置同一朝向，隔一两秒就正常），要轮询几次再下结论。
- **观测/扫描/读码阈值**：观测 30 m + 前方 120°（运输车实测只有 ±60°）；读码 ≤7 m；
  扫描 ≤10.5 m；路口申请 12 m；互斥区 6 m；泊位观测两车都要 ≤13 m、30 s 内、间距 ≥5.3 m。
- **出口 `release` 是很窄的一个点**：`(729.0,−10)` 附近，沿 y=−18~−2 每 0.5 m 试。
- **泊位必须沿轴线从南侧直着开进去**（28 m 助跑位 → 12 m 观测点），中途别用 A*。
- **倒车/贴墙时 A\* 的起点会落在膨胀禁区里**，`_nearest_free` 会把起点挪到几十米外，
  规划出 40 m 的绕圈路线（Car_A 就这样绕仓库撞了 9 次）。最后一段只用直连折线。
- **学生 API 读不到分数**：只能按"任务成果 + `[violation]` 条数"迭代。
- **ps1 必须 UTF-8 with BOM**；python 输出要设 `PYTHONIOENCODING=utf-8`。

## 5. 下一步（按价值）

1. **连跑 3 局确认稳定**（17:17 那局是第 1 局；比赛要求稳定满分口径）。
   每局看 `fog_report.py`：8 项是否全成、违规条数、末行时间。
2. 违规还剩 6 条：2 条起步瞬态（平台给的，压不下去）、2 条 `excessive_acceleration`
   （看是不是 `unstick` 的倒车阶跃）、2 条 `road_boundary_violation`（D/A 直穿场院，
   想压掉得让 `_job` 走 `fog_route.route` 的正式路网，代价是多跑 60~100 m）。
3. 时间上还有 100 s 余量，可以拿来换违规：把 v_cap 从 4.0~4.2 降到 3.6 少踩 `speeding`。
4. 成绩与经验写回 `projects/metacar-race/docs/PROGRESS.md` 第七之二节（17:17 那局已经写进去了）。

## 6. 踩坑速查

| 现象 | 原因 | 解决 |
|---|---|---|
| 启动即退出、stdout 空 | ps1 丢了 BOM / 中文乱码 | 重存 UTF-8 with BOM |
| `9100 没起来` | 上一局 SmartCar 没退干净 | 脚本会重试一次；仍失败就手动杀 `SmartCar`/`metapower_desktop` 再跑 |
| 进程活着但没输出 | 客户端卡死 | 看门狗 260 s 会杀 |
| `ModuleNotFoundError: fog_xxx` | 没用项目 venv | 用 `projects\metacar-race\.venv\Scripts\python.exe` |
| `[deliver] ... 仍然看不见 Berth_x` | 车头没对着泊位（运输车 ±60°）/ 空快照 | 看 `[aim]` 那行的"位移航向 vs 泊位方位"；差 >40° 就重跑助跑 |
| `[deliver] ... Destination is still fuzzy` | 泊位没到 lv3 | 两车都必须 `report_observation` 成功（看 `[obs] ... tok=` 是否为空） |
| 车绕着一个建筑转圈 | A* 起点落在膨胀禁区 | 最后一段别用 `nav_path`，用直连折线 |

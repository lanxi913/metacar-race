# 港口迷雾协同运输 —— 当前进度与下一步（2026-10-29 03:55）

## 一、已经稳拿的分数：400 / 1000

| 项 | 分值 | 状态 | 做法 |
| --- | --- | --- | --- |
| 现场信号 A | 80 | ✅ 每趟都成（≈7 s） | `Signal_A`=(587.47,124.78)，距离 ≤7 m 时 `hint` 出现 `KEY-A7`，**滑行中**提交 |
| 现场信号 B | 80 | ✅（≈142 s） | `Signal_B`=(766.5,-34.5) `KEY-B4` |
| 现场信号 C | 80 | ✅（≈173 s） | `Signal_C`=(720.5,-96.0) `KEY-C9` |
| 路障扫描 | 160 | ✅（≈184 s） | 灯塔旁 `Roadblock_East` 进 10 m，**停稳**后 `interact(Car_C, oid, "scan")`（不能带 dict） |

三条铁律（都是踩坑换来的）：
1. `submit_key` 必须在**车还在动**的时候提交 —— 停稳再交一律 `not_available`。
2. `scan` 必须**先停稳**，且 payload 不能是 dict。
3. 未获批**绝不能**进路口 6 m 区，否则 `Mission has ended; vehicle controls are locked by the judge.`（m18 整趟作废）。

## 二、还差的 600 分

- 两辆运输车通行中央路口：2×100（`request_entry` → 穿 6 m 互斥区 → 到对侧连接出口 → `release`）
- 两个泊位交付：2×200（泊位需 `report_observation` 抬到 lv3 → 进 3 m 停稳 → `deliver`）

### 路口：已拿到的全部事实
- `Intersection_Center` = **(728.79, -20.22)**，`object_type='intersection'`。
- 可申请区：距中心 ≤12 m **且**在进车道。实测获批点：**(727.0,-9.5) yaw≈74（朝南）**、**(729.3,-31.2) yaw≈298（朝北）**。
- `release` 三种报错的语义（已逐条试出）：
  - `not on an eligible intersection approach` → 不在进车道 / 超 12 m
  - `has not reached a connected intersection exit` → 这个点不是该入口对应的出口
  - `has not physically cleared the intersection` → 出口对了，但还没完全驶出互斥区（要继续往外走）
- 未解：**朝南穿过路口时车在 (727.7,-12.9) 原地锁死**（约 6.9 m，正是互斥区边界），油门踩到 0.62 也不动；
  朝北穿过（m17）能走通，但 release 点还没找到。

### 泊位：已拿到的全部事实
- `Berth_A` x[624.47,637.47] y[-72.22,-59.22]；`Berth_B` x[827.47,841.47] y[-72.22,-59.22]，开局 `lv0/fuzzy`。
- 只“看见”不会升级：必须 `get_observation` 取 `observation_token` → 同一辆车 `interact(vid,oid,'report_observation',{'observation_token':token})`；
  同一次升级要**两辆不同车**、两观测点间距 ≥ 两车公开长度较大值、时间跨度 ≤30 s，且 3 个信号才能到 lv3。
- `Obstacle_1` x[755,812] y[34,105]、`Obstacle_2` x[807,868] y[-76,-4] 区内有真实物理障碍，路由必须绕开。

## 三、下一步（按顺序做，每步一趟真车 ≈10 min）

1. **先把"卡死"这件事彻底解决**：`drive_polyline` 已补上卡死自救（`|v|<0.15` 持续 3.5 s → 0.62 油门 1.6 s，日志打 `[stall]`）。先跑一趟验证 `[stall]` 能在 (727.7,-12.9) 那种点把车踢出来。
2. **路口改朝北穿**（m17 已验证能过）：南侧进车道 (728.8,-46)→(728.8,-30) 边开边 `request_entry`，获批后沿 x=728.8 一路向北到 y=+10，
   再沿候选点逐点 `release`：`(728.8,2.0) (728.8,8.0) (730.9,2.0) (726.7,2.0) (730.9,-4.0)`，每点 `hold_still(0.6)` 后试 2 次。
3. **泊位**：Car_A 到 Berth_A 南侧、Car_B 到 Berth_B 南侧，两车各自 `report_observation`（间隔 ≤30 s），确认 `unlocked_destination_count` 变 2/2 后再进泊位 `deliver`。
4. 全程守铁律：≤5.69 m/s、`|a|≤5` 持续不得超过 0.5 s、不撞、不压黄实线、不穿封闭道路（路障中心线两侧 7 m）、不出边界。

## 四、跑一趟的标准命令

```powershell
Get-Process SmartCar,metapower_desktop -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Seconds 12
cd 'D:\无人算法\metacar-race'
& .\tools\auto_scene.ps1 -Script 'C:\Users\Administrator\Documents\deepseek-harness\default-workspace\fog_mission.py' `
  -SceneCp '6E2F,53E3,8FF7,96FE' `
  -PythonArgs '--out D:\无人算法\metacar-race\logs\fog_m25 --seconds 620 --cross 1' `
  -TimeoutSeconds 900
```

不杀进程就起场景，会 `auto_scene.ps1:102 throw "client window not ready"`。

## 五、代码位置

- `fog_mission.py`：阶段 0 信号 / 1 路障 / 2 侦察 / 3 路口（v11：南进北出 + 候选出口逐点 release）/ 4 交付
- `fog_route.py`：车道航点（每条 road 每 2 m 一点）
- `fog_nav.py`：栅格 A*（物体按自身坐标系膨胀，避开 QuayEdge 这类长构件）
- 日志：`D:\无人算法\metacar-race\logs\fog_m1..m24`

## 六、05:10 新发现：路口是一条 5 m 宽的闸口（这是 m19~m33 全部卡死的根因）

static_objects 里有一对 construction_barrier，它们把路口围成了一条窄通道：

| id | position | z 角 | length |
| --- | --- | --- | --- |
| Port_PassageBarrier_-1_2 | (726.26, -20.22) | 90° | 8.80 |
| Port_PassageBarrier_1_2 | (731.32, -20.22) | 90° | 8.80 |
| Port_PassageBarrier_-1_1/-1_3 | (725.26,-14.27)/(725.26,-26.17) | 57°/123° | 3.69 |
| Port_PassageBarrier_1_1/1_3 | (732.32,-14.27)/(732.32,-26.17) | 123°/57° | 3.69 |

两墙中心距 **5.06 m**，中心线 x=**728.79**（就是 Intersection_Center 的 x）。货车宽 2.45 m，可余量左右各 1.3 m。

所有卡死点都能对上这个闸口：(730.0,-27.8)、(727.7,-12.9)、(731.4,-30.0)、(733.2,-33.9) —— 偏离中心线 1.3~4.4 m，全是贴着墙碰住。

### 下一步（确定性做法）
现有 LaneDriver（纯跟随，前视 1.4 m）在 1.2 m/s 下仍然会漂 1.5~4 m，不够。闸口段必须换成**专用走廊控制器**：
1. 在 y=-42 先将车摆正（航向与车道平行、横向误差 <0.3 m），用“横向 P + 航向阻尼”两项控制，不用前视点；
2. 横向误差 ex = 728.79 - x，steer = clamp(0.55*ex - 0.35*yaw_err_rad)，每帧 10 Hz；
3. 入口前就把速度压到 1.0 m/s，进闸口后不停地直行到 y=+8；
4. 边进边 
equest_entry（已知可批点 (728.9,-29.5) 和 (727.0,-9.5)），获批后直行；
5. 出闸口后在 (728.8,6)/(728.8,10)/(726.7,2) 逐点 
elease。



## 七、泊位交付（当前已稳 800/1000）

### 成绩账
| 项 | 分 | 时点（典型跑次） |
|---|---|---|
| 现场信号 A/B/C | 3×80=240 | 7 / 142 / 173 s |
| 动态路障扫描 | 160 | ~185 s |
| 中央路口通行（Car_A/Car_B） | 2×100 | 260 / 340 s |
| 泊位 A 交付（Car_A） | 200 | 429~440 s |
| **合计** | **800** | |

可复现性：m63 / m64 / m65 / m66 / m68 / m69 六条跑次均在 429~440 s 拿到 `Delivery completed.`

### 观测（observation）四个实测条件
1. 目标必须进 `get_observation(vid).visible_objects`：标称 30 m，**泊位实测要 ~13 m 内**才出现（m68 在 21.4 m 处 `visible=[]`）。
2. 车头朝向目标（120° 前向视场）；开进车位后车头背向泊位就不会被接受。
3. 必须拿到 `observation_token`；`hint` 写着“目标已精确定位或本车当前等级已有有效观测”时不发凭证。
4. 同一等级提升需**两辆不同车**、两观测点间距 ≥ 5.30 m（Cargo 车长）、时间跳度 ≤ 30 s。两车必须**在同一个 30 s 窗口内**各报一次（m69 失败就是 Car_B 在 510 s 报、Car_D 到 603 s 才靠近）。

### 交付（deliver）四个前置
1. 目标 `lv3/precise`；只有 lv3 后 `shared_task_objects[].position` 才是精确点（Berth_A = `(629.47,-66.72)`，矩形中心 `(630.97,-65.72)` 是错的）。
2. 本车已完成中央路口通行（transport 车）。
3. 距精确点 ≤ 3 m（实测到 1.15 m 才稳）。
4. `GearMode.PARKING` + `brake=1.0` 静置 ≥ 2 s，否则报 `must hold a stable position before delivery.`

### 报错字典（实测）
- `Destination is still fuzzy.` → 等级不到 lv3
- `Car_X must hold a stable position before delivery.` → 没挂 P 档静置
- `Car_X is 3.0 m from the precise berth.` → 用了矩形中心而不是精确点
- `Car_X has not entered the intersection after receiving access` / `has not physically cleared` / `has not reached a connected intersection exit` → 路口三段式报错

### 关键待解
- Berth_B 的第二观测车（Car_D）赶到时已超 30 s 窗口 ⇒ 需让 Car_B 与 Car_D 同时就位后再一起报（m70 正在验证）。


## 八、规划书与解决方案（按用户要求重写）

- `港口迷雾-规划书.md`：计分结构、八项完成条件、依赖 DAG、现场几何、时间预算、风险与验收。
- `港口迷雾-解决方案.md`：四车并行架构、各得分项算法、观测成对协议、参数表、失败恢复。

### 本轮代码变更（已 `ast.parse`，尚未真跑）

1. `stage_prepos()`：信号阶段并行预就位——Car_A→(731,-31)、Car_B→(731,-56)（排在南口后方）；实测 45 s 完成。
2. `--prepos`（默认 1）；跳过赶路的快路径：`stage_cross` 里若距入口 ≤30 m 就不再 `nav_path`。
3. `mark_done()` / `mission_done()`：八项完成计数（sigA/B/C、scan、crossA/B、berthA/B），全齐打 `[mission] 八项全完成`。
4. 速度：B 段两车并行 4.2/4.6 → **5.2 m/s**；`west_cc` 4.2 → 5.0。
5. B 交付：每次试交前强制 6×`PARKING+brake=1.0`（1.2 s），解决 `must hold a stable position`。
6. B 观测：`tok_of()` 双门控（两车同时有凭证才成对提交）+ Car_A 站位 (826.2,-65.2) 朝东（与 Car_B 间距 7.6 m ≥ 5.30）。


## 九、满分里程碑（phase=Complete，两次连续）

| 跑次 | 信号 | 路障 scan | 路口 A/B | 泊位 A | 泊位 B | phase | 耗时 |
|---|---|---|---|---|---|---|---|
| m93 | 171.5 s 3/3 | 183.0 s | 251.5 / 320.7 s | 413.0 s ✅ | 503.6 s ✅ | **Complete** | **503.7 s** |
| m94 | 172.5 s 3/3 | 183.8 s | 249.0 / 316.9 s | 412.7 s ✅ | 503.2 s ✅ | **Complete** | **503.2 s** |

两次均：`signals=3/3`、`unlocked=2/2`、`Berth_A/Berth_B` 均 `lv3/precise`、无超速/无碰撞/无未授权进区。㜮制 600 s，余量 ~96 s。
按计分规则：240 + 160 + 200 + 400 = **1000**。

### 让 B 泊位成功的两行修复（关键）
1. 泊位 A 观测点改为固定值：`Car_A (634.4,-60.8)` + `Car_D (641.0,-57.6)`（间距 7.3 m ≥ 5.30 m），不再用“沿射线自动摆位”（那版两点只隔 4.5 m、Car_D 还站进车位口袋）。
2. B 交付前强制 6×`PARKING + brake=1.0`（1.2 s）静置，写死 `must hold a stable position` / `hold position to confirm` 两连拒。

### 复现命令
```powershell
Stop-Process SmartCar,metapower_desktop -Force -ErrorAction SilentlyContinue; Start-Sleep 12
cd 'D:\无人算法\metacar-race'
$env:FOG_DELIVER_B='1'
& .\tools\auto_scene.ps1 -Script 'C:\Users\Administrator\Documents\deepseek-harness\default-workspace\fog_mission.py' `\
  -SceneCp '6E2F,53E3,8FF7,96FE' `\
  -PythonArgs '--out D:\无人算法\metacar-race\logs\fog_run --seconds 620 --cross 1 --deliver 1' `\
  -TimeoutSeconds 900
```

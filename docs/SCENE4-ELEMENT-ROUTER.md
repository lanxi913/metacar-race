# 连续场景 4：元素级控制器路由

日期：2026-10-25　分支：`feat/scene4-runner`　状态：代码已接好、离线冒烟通过，**未跑仿真**

## 1. 为什么要重构

`tools/park_cs4.py` 原来用同一套「曲率剖面 + 纯跟踪 + 跟车限速」跑完整条
4718.5 m 赛道。10-25 的 cont-r2 跑次（15100 帧 / 502 s）结算表：

| 元素 | 得分 | 备注 |
|---|---|---|
| 车辆直行 | 100.00 | 无加速度 +10 |
| 连续转弯 | 110.00 | 100+10 |
| 环岛驶入、驶出 | 90.00 | 100+10−20 压道路边线 |
| 车辆跟随 | 110.00 | 100+10 |
| 十字路口通行 | 170.34 | 200+10−39.66 闯红灯 |
| 复杂路况超车 / 匝道行驶 / 车辆紧急停止 | 0（整行消失） | 元素 8 处卡死 13 s 直到场景超时 |
| 车辆变道 / 车辆切出 / 辅路驶入驶出 / 平行车道泊车 | 未进入 | — |

可见合计 580.34。三类问题：**该拿分的元素拿不到（超车/匝道/紧急停止）、
该停的没停（闯红灯）、该过的过不去（紧急停止元素卡死）**。

同时 `main` 分支上每个元素都有单独实测过分的入口脚本，所以这次重构的原则是
**控制律一行不重写，只做「按里程切元素 → 交给已验证的控制器」**。

## 2. 元素 → 控制器映射

| 元素 | 处理器 | 依据 |
|---|---|---|
| 车辆直行 | 定油门 0.55 + 纯跟踪（`StraightElement`） | 本仓库 10-24 局 17/18 实测 110 |
| 连续转弯 / 匝道行驶 / 辅路驶入驶出 / 复杂路况超车 / 车辆变道 / 车辆切出 | `geo_follower.GeoFollower`（`GeoElement`） | `lianxu1_run.py`（连续场景1 三局 630）、`fudao_run.py` 辅路 110、`fuza_run.py` 复杂超车 113.81 |
| 环岛驶入、驶出 | `ring_turn.RingTurnController`（`RingElement`），带 `dynamic_path` + `a_now` | `tools/ring_turn_run.py` |
| 车辆跟随 | `follow.select_leader` + `compute_target_speed` + `LongitudinalController`（`FollowElement`） | `tools/follow_run.py` |
| 车辆紧急停止 | `emergency_stop` 剖面 + `LongitudinalController` + 停稳保持（`StopAtEndElement(stop_at_end=True)`） | `tools/emergency_stop_run.py` |
| 十字路口通行 / 丁字路口通行 / 无保护左转 | 同上的纵向栈 + 绿灯 ETA 闸门，不在元素终点停车（`StopAtEndElement(stop_at_end=False, use_light=True)`） | 纵向/横向照抄 `emergency_stop_run.py`；`shizi_run.py` 用 `RouteFollower` 拿 200，但 main 上**没有任何控制器读信号灯**，闸门是本仓库补的 |
| 平行车道泊车 | `parallel_parking.ParallelParkingController`（`ParallelParkElement`） | `parallel_run.py` 连续 5 次 210.00 |

**没有接的**：`RouteFollower`（它的目标点是整条 route 末点，放进连续场景会一路开到 4718 m）、
`LaneFollower`（同样吃整个 `static`，目标是 route 末点）、`VerticalPark`（本场景是平行车位）。

## 3. 关键实现点

* 新文件 `tools/scene4_elements.py`（481 行）：`ElementContext` / `ElementControl` /
  七个处理器 / `ElementRouter` / `slice_route` / `lane_width_at`。
* 路由按 `elem_spans` 的里程选元素，**不依赖平台报的元素名**——cont-r2 里
  `SceneStatus.sub_scene_name` 到 s=3042.9 还停在「复杂路况超车」。
* 两个硬约束：
  1. **单个元素出错不许杀整局**：处理器构造或执行抛异常 → 该元素加入
     `router.disabled` 永久降级到通用路径，日志里留一条 `[router] ... 降级`。
  2. **紧急停止元素不许无限等**：停稳保持 `EMERGENCY_STOP_HOLD_SECONDS(3 s)`，
     再多等 `25 s` 仍未结束就交回通用路径，避免重演「卡死 13 s 到超时」。
* 信号灯闸门沿用 `park_cs4.py` 里十字路口那套标定（那里实测过 200 分），
  只补一条：绿灯剩余时间不够过线时，按「过不去」处理
  （`t_line + 0.5 > remain` → 按红灯收速）。
* 接线在 `tools/park_cs4.py`：第一帧用 `msg.main_vehicle.length/width` 构造路由；
  每帧在 `limit_command()` 之后、终点驻车之前接管控制量；`--no-elements` 可整体关闭，
  `--elements-only 紧急停止,十字路口` 可只开指定元素。

## 4. 怎么验证（按元素单独跑，一局一类问题）

```powershell
# 只测紧急停止元素（元素 8）
.venv\Scripts\python.exe -u tools\park_cs4.py --skip-to-index 8 --max-seconds 300

# 只测十字路口（元素 5），看还闯不闯红灯
.venv\Scripts\python.exe -u tools\park_cs4.py --skip-to-index 5 --max-seconds 200

# 只测环岛（元素 3）
.venv\Scripts\python.exe -u tools\park_cs4.py --skip-to-index 3 --max-seconds 200

# 全程（12 个元素）
.venv\Scripts\python.exe -u tools\park_cs4.py --max-seconds 700
```

注意：`--skip-to-index` 只是把车瞬移到该元素起点，**元素自身的关卡判定不一定跟着跳**，
所以单元素跑次的分只作参考，最终以全程跑次为准。

## 5. 离线冒烟

`tools/_smoke_scene4_elements.py` 用假 static（12 个元素各 100 m 的直线）把
12 条分支各走 6 帧：全部分支构造成功、控制量在合法范围、无降级。

## 6. 风险

| 风险 | 说明 | 兜底 |
|---|---|---|
| `GeoFollower` 的 `speed_limit` 给的是 `--vmax`（8 m/s） | 元素自己的限速没读（场景里 `LIMIT_SCENE_ELEMENT_KMH`=60 km/h 只是元素限时口径） | 超速扣分会在结算表体现，下一轮按元素加限速 |
| 十字路口的横向用的是切片上的纯跟踪 | main 上 `shizi_run.py` 用的是 `RouteFollower`（带更细的 v 剖面），横向差别未知 | 若这轮十字路口仍掉分，改成 `RouteFollower` 只喂该元素切片 |
| 紧急停止的 `select_leader` 用的是平台原始障碍 | 与 `emergency_stop_run.py` 一致；但元素 8 处那辆车是 `type=6 dist=4.89 v=0`，若它不在 `VEHICLE_TYPES` 里就选不到 | 已加 `safe_stop_margin` 的 endpoint 剖面（不依赖 leader 也能停在终点前） |
| 平行泊车的 `get_parking_slot` 用的是 `static.obstacles` | 与 `parallel_run.py` 一致 | 找车位失败会 SEEK 直行，日志可见 |

## 7. 验证记录（2026-10-25）

| 跑次 | 参数 | 现象 | 结论 |
|---|---|---|---|
| full-r1 | `--skip-to-index 0` | 起步 0.57 s 出现 `brake=0.17`；元素 2 钉在 4.78 m/s，211.9 s 平台瞬移；瞬移后 `steer=0.000`、横向误差 +13.2 m、开出路面 | 三个问题：起步没人管（丢元素 1 的 +10）、瞬移不复位、一挡墙 |
| full-r2 | 同上 + 起步接管 + 瞬移复位 | 起步 `thr=0.55 brk=0.00` 全程零刹车；元素 1 走完；瞬移后能正常接管；元素 2 仍 4.78 m/s、211.9 s 瞬移 | **元素 1 结算 110（+10 拿回）**；瞬移复位修好；一挡墙仍在 |
| full-r3 | 同上 + `GeoElement` 改用 `SpeedController`（`allow_kick`、`kick_throttle=0.75` / `kick_time=0.30 s`） | 元素 2 在 199.0 s 出现 `thr=0.75`、车速升到 7.46 m/s、188 s 内跑完 1023 m；元素 3 环岛在 s=1088.9 趴窝十余秒 | **元素 2 通过**（一挡墙解决）；环岛纵向仍不动 |
| full-r4 | `--elements-only 起步直行,环岛,十字路口,复杂路况超车`（其余元素回到通用栈） | 元素 1 起步 110；元素 2 走通用栈 100（丢 +10）；元素 3 环岛在 0.6~1.2 m/s 抽搐，175 s 只走 165 m，用户退出 | 环岛**纵向不能用本车速度环**：油门增益 12~15 m/s²，速度环按 1.5 m/s² 闭环会反复「加油→切 0→掉速」。已改为**环岛只接管方向盘、纵向沿用通用栈** |

### 8. 环岛纵向的三次尝试（教训）

1. 直接用 `RingTurnController` 的油门：本车永不供油（`RING_TURN_START_THROTTLE_MAX=0.004`）→ 16 s 原地不动。
2. 换成 `SpeedController`（`throttle_max=0.85`）：进场时把车从 7 m/s 刹到 0（`v_cmd` 从 0 爬坡），之后在 0.6~1.2 m/s 抽搐 → full-r4。
3. 结论：**只接管方向盘**。本车油门偏开关型（0.45 就有 12~15 m/s²），任何按加速度闭环的纵向控制器都会超调；纵向交给已经调好的通用栈（cont-r2 里它能把环岛跑到 8 m/s）。

### 9. 临时结档（2026-10-25）

- 接管范围默认收窄为 4 处：起步段、环岛（**仅横向**）、十字路口、复杂路况超车；其余 8 个元素一律走通用栈（cont-r2 能跑到元素 8 的那套）。
- `ElementControl` 的三个通道都可为 `None`，`None` = 该通道沿用通用栈。
- 待办：十字路口与复杂路况超车尚未在全程跑次里验证；元素 8 紧急停止仍是既有停滞行为；后续按用户要求重构整个流程。

## 10. 框架版（2026-10-25 下午，`metacar_race/scene4/`）

旧的整体式驱动（1519 行）已经拆成包，`tools/park_cs4.py` 退化成 CLI 薄壳
（`argv → DriverConfig → Driver.run()`）。旧实现留在 git 历史
`9bdb2e8:tools/park_cs4.py`，需要对照行为时 `git show` 它；仓库里不再放第二份拷贝。
`tools/scene4_elements.py` 同时标为 **deprecated**（框架版不再引用它）。

| 模块 | 干什么 | 不干什么 |
|---|---|---|
| `config.py` | 全部参数 + `DriverConfig`（`caps` 在 `__post_init__` 里由 `--bonus` 推导） | 不含逻辑 |
| `track.py` | 累计里程、局部投影、停止线/信号灯几何、车道宽 | 不产生控制量 |
| `obstacles.py` | 障碍投影（`ObstacleScan`）、起步破挡（`StallGuard`）、卡死（`StuckWatch`）、瞬移（`TeleportWatch`） | 不决定速度 |
| `lateral.py` | `Stanley`、并线状态机 `Overtaker`、横移路线 `ShiftRoute` | 不碰油门 |
| `longitudinal.py` | 收油治理 `AccelGovernor`、指令限幅 `limit_command`、上升限幅 `SoftStart` | 不含第二个速度环 |
| `safety.py` | 信号灯闸门 `LightGate`（每帧按 id 刷新 + 绿灯 ETA）、终点驻车 `EndPark` | 不选路线 |
| `platform.py` | `SceneSession`（connect / frames / set_control / park）、`SceneLog`（四件产物） | 不含策略 |
| `strategies.py` | `Command`（`steer/v_cap/throttle/brake/gear`，`None`=沿用基线）、`StartStraightStrategy`、`RingStrategy` | 不实现纵向闭环 |
| `router.py` | 里程选元素、名称路由、瞬移复位、失败降级 | 不发控制量 |
| `driver.py` | 唯一的主循环与唯一的速度环 | 不写元素特例 |

### 10.1 通道契约（这次重构唯一的新约束）

策略只声明自己接管哪条通道，没声明的（`None`）一律沿用基线：

```python
Command(steer=..., v_cap=..., throttle=..., brake=..., gear=...)
```

* 已占用的映射：`起步直行`/`车辆直行` → `straight`（定油门 0.55 + PurePursuit，实测 110）；
  `环岛` → `ring`（**只给 steer**，纵向留 None）。
* 其余元素（连续转弯/跟随/十字路口/超车/匝道/紧急停止/辅路/变道/切出/平行泊车）
  目前一律 `None`，跑 cont-r2 那套能到元素 8 的通用栈；要接一个元素，先加策略、再加单元素跑次验证。

### 10.2 控制量流水线（顺序即语义）

```
profile.target → 跟车收速 → 横移锁速 → 信号灯闸门 → 策略 v_cap
  → accel_gov.apply_trim → speed_ctl.update → stall → soft_start → accel_gov.update
  → blk_ctrl 硬刹 → 指令限幅 limit_command → 策略逐通道覆盖 → 终点驻车 → 下发
```

`limit_command` 之后才允许策略覆盖：起步段的定油门 0.55 是**故意**绕过限幅的
（限幅在 `--bonus` 下把油门压到 `4.5/8=0.5625`，0.55 仍在预算内，两者不冲突）。

### 10.3 离线验证

```powershell
.venv\Scripts\python.exe tools\_smoke_scene4_framework.py
```

18 项检查（2026-10-25 全绿）：起步段合成、13 段切分、直行接管且油门 0.55、
连续转弯/跟随不接管、环岛只给 steer、`reset()` 后重建、指令限幅 0.5625、
`EndPark` 滑行驻车、`LightGate` 绿灯 ETA 两例。

### 10.4 已知取舍

* 环岛在平坦直线合成路线上的 `steering=1.0`（控制器按真实几何算，合成数据不足为凭）；
  真实环岛几何要在跑次里看 `[elem] 接管 ring` 那行。
* `LightGate` 依赖 `static.roads[*].stop_line`；没有停止线就**不做**绿灯 ETA，
  只在红灯时按老逻辑收速——cont-r2 的闯红灯正是没有停止线刷新导致的。
* 元素 6/7/8/9–12 仍走基线，元素 8 的 4.89 m 静止车停滞是既有行为（改造前就停在那里）。


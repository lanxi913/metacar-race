# 加速度违规到底怎么判的（2026-10-31 实测）

用 `fog_probe_accel.py` + `fog_run_probe.ps1` 在真实场景里做的三组实验，全程 10 Hz 采样
`get_vehicle_state`，同时记录 `get_events` 里的 violation。

## 一、平台到底给了我们什么

`get_vehicle_state(vid)` 实测回的字段（原样）：

```json
{"vehicle_id": "Car_D", "state": "Idle",
 "pose": {"pos_x": 626.46, "pos_y": 120.78, "pos_z": 0.021,
          "vel_x": -0.000002, "vel_y": 0.0001, "vel_z": 0.000017,
          "ori_x": -0.003, "ori_y": 0.00003, "ori_z": 179.99986},
 "speed_mps": 2.2e-06,
 "control": {"throttle": 0.0, "brake": 1.0, "steering": 0.0, "gear": 3,
             "left_blinker_on": false, "right_blinker_on": false,
             "hazard_lights_on": false, "headlights_on": false}}
```

- **位姿带速度矢量** `vel_x/vel_y/vel_z`，还有标量 `speed_mps` —— 加速度可以自己精确算，不用猜。
- `gear` 是 SDK 的 `GearMode`：NEUTRAL=0、DRIVE=1、REVERSE=2、PARKING=3。**没有"蠕动档"。**

`get_task_state()`：`phase / instance_seed / intersection_owner / roadblock_active /
shared_task_objects[] / verified_signal_count / unlocked_destination_count / total_*`。
每个任务对象带 `information_level / uncertainty_region(minimum,maximum) /
valid_observation_count / required_observation_count / position / road_ids / lane_ids / length_m / width_m`。

`get_events(seq)`：`{sequence, elapsed_seconds, event_type, message, vehicle_id, object_id}`，
`event_type` 有 `system` 和 `violation`；violation 的 message 里写明类型与对象。
**碰撞是按接触帧重复计数的**（实测一次贴墙 1 s 记 10 条），所以"撞一下就废一局"是真的。

`get_scene_static_data()`：`coordinate_system / roads / static_objects / vehicles`。
车辆规格实测：`role='Cargo'`、长 5.30、宽 2.45、高 2.20、**maximum_speed_mps = 6.944（25 km/h）**。

**没有暴露的**：加速度、jerk、score、图像/激光（`get_image/get_lidar` 等在 student API 里一律不可用）。

## 二、油门 / 刹车 → 加速度（实测曲线）

从静止、DRIVE 档、单帧切换控制量，10 Hz 采样：

| 油门 | 0 → 终速 | 峰值加速度（逐帧 dv/dt） |
|---|---|---|
| 0.06 | 0 → 5.58 m/s，约 1.0 s | **+10.07** |
| 0.10 | 0 → 6.05 m/s，约 0.8 s | **+12.75** |
| 0.15 | 0 → 4.84 m/s，约 0.5 s | +11.32 |
| 0.20 | 0 → 4.86 m/s，约 0.5 s | +11.16 |
| 0.30 | 0 → 4.84 m/s，约 0.4 s | +11~12 |
| 0.55 | 0 → 4.83 m/s，约 0.4 s | +12.05 |
| 刹车 1.0 | 5.2 → 0，约 0.2 s | **−15.24** |

**结论 1：油门不是比例控制。** 0.06 和 0.55 的差别只是"用 1.0 s 还是 0.4 s 冲到 4.8 m/s"，
峰值加速度都在 10~13 m/s²。想在起步时把加速度压到 5 以下，靠"给油小一点"是无效的。

**结论 2：静止时速度信号有 ±0.24 m/s 抖动**（对应 ±1.7 m/s²），但远低于门限，不会误报。

## 三、判定不是"逐帧超 5 就记"

三组实验的对照（同样 10 Hz 采样、同一个判定接口）：

| 实验 | 逐帧超 5 m/s² 的帧数 | 峰值 | **平台记的 accel 事件** |
|---|---|---|---|
| 连续油门阶跃（0.55/0.30）+ 硬刹 | 4 | +12.05 / −14.93 | **1** |
| 连续小油门（0.06/0.10/0.15/0.20/0.30/0.55）各 2 s | 多 | +10~+12.8 | **2** |
| **脉冲油门（1 帧给油 / 1~3 帧断开）** | 1~3 | +10.7 | **0** |

第三组是最关键的：**13 帧逐帧加速度超过 5 m/s²（最高 +24），平台一条都没记。**
说明它判的是**一段时间窗内的持续加速度**（message 原文就是"绝对加速度**持续**超过 5.0 m/s²"），
不是看单帧。瞬时冲一下不算，持续 0.5~1 s 才算。

**结论 3：真正要控的是"1 秒内的速度变化量"**，把它压到 5 m/s 以内。
- 起步：从 0 到 4.8 m/s 一口气冲完（0.4 s）→ 会踩线；用**脉冲给油**把爬升拉到 ~2 s（≈2.4 m/s²）→ 实测 0 事件。
- 停车：刹车 1.0 是 4.8→0 只用 0.2 s → 必然踩线；用**低刹车或脉冲刹车**把停车拉到 1 s 以上。

## 四、我们能控制的东西

| 层 | 可控量 |
|---|---|
| 单车执行 | `throttle` / `brake` / `steering` / `gear`（NEUTRAL·DRIVE·REVERSE·PARKING）/ 转向灯·大灯 |
| 任务交互 | `submit_key` / `report_observation` / `scan` / `request_entry` / `release` / `deliver` |
| 路线 | 完全自定义（我们有路网图 `fog_route` + 栅格 A* `fog_nav`） |
| 时序 | 每辆车什么时候动、什么时候停、并行还是串行 |

## 五、由此得到的改法（待实施）

1. **起步用 PWM**：加速阶段按"1 帧给油 / 1 帧断开"输出（10 Hz 下等效 50% 占空比），
   实测能把 0→4.8 m/s 拉到约 2 s，峰值窗口加速度 ≈2.4 m/s²，不触发。
2. **停车用 PWM 刹车或低刹车**：目标是把 4.8→0 拉到 ≥1 s（等效 −4.8 m/s² 以内）。
   注意刹车太软会冲过停车点，需要提前减速（把 `ACCEL` 规划值也一起调）。
3. **不要用"减小油门"来控加速度**——无效（见结论 1）。
4. 巡航段完全不用管：匀速时逐帧加速度只有 ±0.1 m/s²。

## 六、复现

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog\fog_run_probe.ps1" -Seconds 60 -Deliver 0
```

日志在 `.train\fog800\logs\m_MMDD_HHMMSS\probe.log`（字段 dump + 每 0.1 s 的
`t / tag / thr / brk / v / a_speed` + 所有 violation 事件）。

## 七、第二轮实测（2026-10-31 15:30~15:50，往返式扫描）

### 油门（从静止，DRIVE 档，每档 1.5~2.0 s）

| 油门 | 稳态速度 | 峰值 | 到 90% 用时 | 逐帧 a 峰值 |
|---|---|---|---|---|
| 0.01 | 1.21 m/s | 1.49 | 1.3 s | 4.8 |
| 0.02 | 3.64 | 4.36 | 1.3 s | 6.4 |
| 0.03 | 4.89 | 5.23 | 1.4 s | 6.3 |
| 0.04 | 5.21 | 5.37 | 1.1 s | 6.4 |
| 0.05 | 5.27 | 5.55 | 0.9 s | 7.4 |
| 0.06 | 5.36 | 5.50 | 0.9 s | 8.9 |
| 0.08 | 5.52 | 5.71 | 0.7 s | 11.4 |
| 0.10 | 5.73 | 6.07 | 0.7 s | 13.0 |
| 0.15 | 4.80 | 4.86 | 0.5 s | 11.2 |
| 0.20 | 4.80 | 4.84 | 0.5 s | 13.4 |
| 0.30 | 4.79 | 4.83 | 0.5 s | 12.1 |
| 0.40 | 4.79 | 4.82 | 0.3 s | 11.9 |
| 0.50 | 4.78 | 4.81 | 0.3 s | 12.8 |
| 0.75 | 4.79 | 4.81 | 0.3 s | 12.6 |

**0.15~0.75 全部停在 4.78~4.80 m/s**——和 `AGENTS.md` 第 5 节 10-25 那批 22 局的结论完全一致
（1 挡力平衡点）。真正能调速度的只有 **0.01~0.10** 这一段（1.2 → 5.7 m/s）。

### 刹车（从 5.23 m/s 起刹，2 s）—— 这一组直接锁定了判定口径

| 刹车 | 末速 | 刹停用时 | 逐帧 a 峰值 | **1 秒窗 a 峰值** | 刹距 | 平台记账 |
|---|---|---|---|---|---|---|
| 0.05 | 2.71 | >2 s | 1.5 | **1.4** | 7.27 m | 无 |
| 0.10 | 0.06 | 1.7 s | 4.4 | **3.3** | 4.80 m | 无 |
| 0.15 | -0.03 | 1.2 s | 6.1 | **4.5** | 3.23 m | 无 |
| 0.20 | -0.05 | 0.9 s | 7.1 | **5.8** | 2.47 m | **记** |
| 0.30 | -0.19 | 0.7 s | 8.7 | **6.1** | 1.71 m | **记** |
| 0.50 | -0.22 | 0.5 s | 15.2 | 6.0 | 1.07 m | **记** |
| 0.75 | +0.29 | 0.5 s | 15.3 | 6.3 | 0.94 m | **记** |
| 1.00 | -0.06 | 0.3 s | 15.3 | 6.2 | 0.89 m | **记** |

**分界线正好在 1 秒窗 5.0 m/s² 上**：4.5 → 不记，5.8 → 记。
据此可判定：**平台的 `excessive_acceleration` ≈「1 秒内速度变化量 > 5 m/s」**（提示语"持续超过 5.0" = 窗口口径）。

### 由此得到的硬约束

1. **刹车有效上限 ≈ 0.15**（从 5.2 m/s 刹到停 1.2 s、3.2 m）。0.20 以上就开始记违规。
2. 刹车 0.15 相比 1.0，**刹距从 0.89 m 变成 3.23 m（3.6 倍）**，所以必须**提前减速**，不能靠最后一脚。
3. 油门 0.01~0.08 的 1 秒窗都在 5 以内，起步本身不是主因；**主因是停车那一下**（我们控制器现在常用 0.5~0.9 的刹车）。

### 复现命令

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "...\tools\fog\fog_run_probe_curve2.ps1" -Seconds 200 -Deliver 0   # 油门
powershell -NoProfile -ExecutionPolicy Bypass -File "...\tools\fog\fog_run_probe_brake.ps1"  -Seconds 120 -Deliver 0   # 刹车
```


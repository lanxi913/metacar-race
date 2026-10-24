# 垂直车道泊车 · chang-kou 基线

目标分数：200。当前状态：从安全基线 `5f72206` 优化倒车纵向控制，尚未完成回归验证。

稳定回退版本是提交 `5f72206`。本分支只改变倒车油门和停车锁存，
暂不加入横向修正。

## 三条复现命令

```powershell
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe -u templates\parking\chang-kou\run.py
# 然后到客户端启动“垂直车道泊车”
```

## 策略

1. 沿车道低速直行并开过车位。
2. 在车位中心前方约一个车道横向距离处停稳。
3. 挂 `GearMode.REVERSE`，根据车位轴向误差持续转向倒车。
4. 挂入倒挡后先制动啮合 `0.5 s`，再按斜坡缓慢增加油门。
5. 使用低速目标、速度迟滞和 10 Hz 状态更新，避免油门与刹车逐帧切换。
6. 距车位中心 `1.5 m` 后按剩余距离逐步降低目标速度，使用小油门和小制动。
7. 进入车位且速度低于 `0.06 m/s` 后驻车保持 5 秒。

## 参数

| 参数 | 默认 | 含义 |
|---|---:|---|
| `APPROACH_THROTTLE` | `0.01` | 前进油门，沿用车辆停止标定值 |
| `STAGE_BRAKE` | `0.10` | 到达倒车准备点的制动值 |
| `REVERSE_THROTTLE` | `0.16` | 正常倒车油门上限；低于它时按斜坡增加 |
| `REVERSE_ENGAGE_SECONDS` | `0.50` | 挂入倒挡后先制动的啮合时间 |
| `REVERSE_TARGET_SPEED_MS` | `0.80` | 达到该速度后松油滑行 |
| `REVERSE_RESUME_SPEED_MS` | `0.55` | 低于该速度后恢复加速，形成速度迟滞 |
| `REVERSE_THROTTLE_RAMP_PER_SEC` | `0.08` | 倒车油门每秒增加量 |
| `REVERSE_CONTROL_INTERVAL_SECONDS` | `0.10` | 倒车速度模式更新周期 |
| `REVERSE_SOFT_LIMIT_SPEED_MS` | `1.15` | 超过该速度时施加轻微安全制动 |
| `REVERSE_SOFT_LIMIT_BRAKE` | `0.05` | 超速安全制动强度 |
| `REVERSE_STOP_START_M` | `1.50` | 距车位中心多远进入最终减速段 |
| `REVERSE_FINAL_SPEED_GAIN` | `0.30` | 剩余距离换算为目标速度的增益 |
| `REVERSE_FINAL_MIN_SPEED_MS` | `0.08` | 最终段最低目标速度 |
| `REVERSE_FINAL_MAX_SPEED_MS` | `0.40` | 最终段最高目标速度 |
| `REVERSE_FINAL_BRAKE` | `0.03` | 最终段轻微制动强度 |
| `REVERSE_CREEP_THROTTLE` | `0.02` | 最终段速度偏低时的补油门 |
| `REVERSE_STEER_SIGN` | `-1` | 倒车转向符号；实测负打舵会让航向反向增加 |
| `CENTER_TOL_M` | `0.35` | 车位中心误差要求 |
| `HEADING_TOL_DEG` | `8.0` | 车身与车位轴向误差要求 |

## 实测参数

- 车辆：长 `4.814 m`、宽 `2.180 m`。
- 车位：中心 `(-7.80, -0.12)`、朝向 `0°`、尺寸 `5.50 × 2.50 m`。
- 两侧车辆：`(-8.37, 3.03)` 和 `(-8.37, -3.05)`。
- 车辆最小转弯半径实测约 `9 m`，因此不采用前进单圆弧入位。

## 踩坑

- 先脚本、后场景。
- 场景启动后车辆可能需要约 7 秒才开始移动，不要提前杀脚本。
- 倒车方向与前进相反，不能直接复用正向纯跟踪的符号。

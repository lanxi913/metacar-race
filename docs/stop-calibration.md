# 车辆停止标定记录

日期：2026-10-23
场景：`车辆停止`
目标：到达终点后停稳，并获得 `|a| <= 5 m/s²` 的 10 分加速度加分。

## 实测参数

| 参数 | 值 | 说明 |
|---|---:|---|
| `STOP_THROTTLE_COMMAND` | `0.01` | 稳态约 4.78 m/s |
| `STOP_BRAKE_COMMAND` | `0.10` | 从 4.78 m/s 刹停约 4.22 m |
| `STOP_BRAKE_DECEL_MS2` | `2.70` | 用于计算何时开始刹车 |
| `STOP_BRAKE_LEAD_M` | `0.10` | 补偿 30 Hz 控制周期，消除位置误差 |
| `STOP_STILL_SPEED_MS` | `0.30` | 平台静止时速度信号仍会小幅抖动 |
| `STOP_SETTLE_SECONDS` | `0.80` | 低速持续该时间后锁存驻车 |

## 标定结论

- 油门 `0.02` 在起步时出现约 11.2 m/s² 的尖峰，不能用。
- 刹车 `0.50` 从约 5.1 m/s 制动时出现约 19.3 m/s² 的尖峰，不能用。
- 油门 `0.01` 的 0.5 秒窗口最大加速度约 4.19 m/s²，可以作为停车题巡航值。
- 刹车 `0.10` 从 4.78 m/s 刹停的实测距离约 4.22 m。
- 不再逐帧根据抖动的 `car.speed` 切换油门和刹车，避免控制脉冲。

## 复现

先启动脚本，再在客户端启动“车辆停止”场景：

```powershell
.venv\Scripts\python.exe -u tools\calibrate_stop.py
```

单独标定油门或刹车时：

```powershell
# 从静止保持油门 2 秒，再用刹车 0.10 停车
.venv\Scripts\python.exe -u tools\calibrate_longitudinal.py `
  --mode throttle --throttle 0.01 --hold-seconds 2 --brake 0.10

# 用油门 0.01 加速到 3 m/s，再用刹车 0.10 停车
.venv\Scripts\python.exe -u tools\calibrate_longitudinal.py `
  --mode brake --start-throttle 0.01 --target-speed 3 --brake 0.10
```

## 三次验收

| 次数 | 得分 | 用时 | 最终位置误差 |
|---:|---:|---:|---:|
| 1 | 110.00 | 17 s | 0.180 m |
| 2 | 110.00 | 17 s | 0.180 m |
| 3 | 110.00 | 17 s | 0.179 m |

三次均获得任务分 100 和加速度加分 10，没有停车位置扣分或碰撞扣分。

> 日志中的速度差分加速度受仿真信号抖动影响很大，最终是否获得加速度加分以平台得分为准。

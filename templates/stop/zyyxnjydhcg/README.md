# 车辆停止 · 基线（目标 110）

## 怎么跑

```bat
.venv\Scripts\python.exe -u templates\stop\zyyxnjydhcg\run.py
:: 然后客户端点「启动场景」（车辆停止）
```

跑分：`powershell -File tools\bench.ps1 -Runs 5 -PythonArgs "templates\stop\zyyxnjydhcg\run.py"`，
Edge 先开 `https://www.i-mylab.com/icv/history.html?id=stop`。

## 参数

| 参数 | 默认 | 含义 |
|---|---:|---|
| `--throttle` | 0.01 | 巡航油门。稳态约 4.78 m/s，0.5 s 窗口最大加速度 ≈4.19 m/s² |
| `--brake` | 0.10 | 制动指令。从 4.78 m/s 刹停约 4.22 m，等效减速度 2.70 m/s² |
| `--hold` | 5.0 | 停稳后保持制动的秒数（题目要求停稳 5 s） |

## 为什么不用「速度闭环 + 到点踩死」

- 油门 `0.02` 起步会出现 **11.2 m/s²** 尖峰，刹车 `0.50` 会出现 **19.3 m/s²** 尖峰 —— 都会丢掉 10 分加速度分。
- 逐帧按抖动的 `car.speed` 在油门/刹车之间切换会产生控制脉冲，所以改成**按刹车距离算好时机，一次连续制动**：
  `刹车距离 = v²/(2×2.70) + 0.10 m`，进入这个距离就开始刹。
- 锁存条件：速度 <0.30 m/s 持续 0.80 s **且** 距终点 <0.40 m。不满足就继续蹭，别提前锁死。

## 踩过的坑

1. 场景 yaw 顺时针为正：`along_track = dx·cos(-yaw) + dy·sin(-yaw)`，符号错了会一直往前冲。
2. 终点附近（<2.5 m）要回正方向盘，否则车会歪着停在终点，位置误差变大。
3. 静止时速度信号有 ±0.23 抖动，判静止的阈值别设太小。

## 成绩

`logs/bench.csv`；标定过程见 `docs/stop-calibration.md`。

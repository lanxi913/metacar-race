# 直角转弯 · chang-kou 基线

目标：110 分。

```powershell
.venv\Scripts\python.exe -u templates\rightturn\chang-kou\run.py
```

先运行脚本，再在客户端启动“直角转弯”。

参数：

- `--throttle 0.01`：保持加速度加分。
- `--lookahead 3.0`：匹配 3 m 曲率的转弯前视距离。

到终点前不要主动停车；平台需要检测车辆实际到达。当前已完成 3 次 110 分，仍需统一跑满 5 次。

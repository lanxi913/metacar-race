# 车辆直行 · chang-kou 基线

目标：110 分。

```powershell
.venv\Scripts\python.exe -u templates\straight\chang-kou\run.py
```

先运行脚本，再在客户端启动“车辆直行”。

参数：

- `--throttle 0.5`：定油门巡航，实测 5 次 110 分。
- `--lookahead 6.0`：直行前视距离。

不要用速度闭环；闭环实测会因加速度尖峰丢掉 10 分。

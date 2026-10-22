# 直线赛道最小闭环

本目录保存已在「车辆直行」场景实测跑通的单车主入口。控制逻辑与仓库根目录
`main.py` 的已验证版本一致，仅调整了脚本内提示的运行路径。

## 运行

所有命令都在仓库根目录执行，先启动脚本，再在客户端点击「启动场景」。

```powershell
cd D:\metacar_work\metacar-race
.venv\Scripts\python.exe -u straight_track\main.py
```

入口默认安全刹停，用于确认连接、静态数据和相机帧正常。需要匀速前进时：

```powershell
.venv\Scripts\python.exe -u straight_track\main.py --drive --throttle 0.35
```

## 实机验证

2026-10-22 在「车辆直行」场景验证：

- 场景连接成功，路线点 50 个，子场景 `["车辆直行"]`，相机 1 路。
- 默认刹停模式连续刷新到 591 帧，20 秒上限到达后正常退出。
- 连接顺序为「先脚本、后场景」，终端持续输出 `pos`、`yaw`、`v` 和相机帧数。

本入口只负责最小闭环和状态观察，不包含循迹、速度环或 VLA 提交逻辑。

# RUNBOOK：把泊车方案跑出 210 分

> 照着做，20 分钟内能复现。遇到问题先看最后的排错表。

## 0. 前置条件

| 项目 | 要求 |
|---|---|
| 系统 | Windows 10/11 |
| Python | 3.12（3.10 以上都行） |
| 磁盘 | D 盘留 20 GB 以上 |
| 网络 | 能访问 github.com 和 api.i-mylab.com |
| 客户端 | 元动力客户端 1.1.1，已登录 |

## 1. 拉代码

```bat
cd D:\
git clone https://github.com/lanxi913/metacar-race.git
cd metacar-race
```

> 仓库已公开，直接 clone 即可，不需要邀请。只想看代码不跑仿真的话到这里就够了。

## 2. 建虚拟环境

```bat
python -m venv .venv
.venv\Scripts\python.exe -m pip install -U pip
.venv\Scripts\python.exe -m pip install metacar==0.4.0 metacar_multiagent==0.1.0
```

**必须用 `.venv\Scripts\python.exe`**，不要用系统 python，否则会 `ModuleNotFoundError: metacar`。

## 3. 确认场景已装

打开客户端 → 自动驾驶 → 第 2 页 → 找到 **垂直车道泊车**，显示「启动场景」就说明装好了。
显示「下载场景」就先点下载，等解压完。

如果点启动报「场景文件不存在」，执行：

```bat
.venv\Scripts\python.exe -u tools\fix_scene_dirs.py
```

## 4. 自检

```bat
.venv\Scripts\python.exe check_env.py
```

应该看到 metacar 0.4.0、metacar_multiagent 0.1.0、9100 端口正常。

## 5. 跑泊车（关键顺序）

**第一步：先起脚本**（它会阻塞等待场景连上来，这是正常的）

```bat
.venv\Scripts\python.exe -u tools\park_v8.py
```

看到 `等待场景连接...` 后再做第二步。

**第二步：点客户端「启动场景」**

顺序反了会一直连不上：脚本是服务端，场景启动后主动连 `127.0.0.1:5061`。

**第三步：等约 30 秒**，脚本会打印：

```
到位并停稳(d=..., v=...)，切倒挡满舵
车头已摆正(dyaw=...)，回正倒直线
入库：dx=+0.05 dy=+0.00 航向 1.5°
加速度峰值：逐帧 ...，0.1s 2.76，1s 2.76 m/s2
保持完成，用时 29.5s，中心误差 0.13 m，航向误差 1.5°
```

## 6. 看分数

客户端 → 垂直车道泊车 → **训练记录** → 第一条应该显示 **210.00**。

## 7. 应该看到的结果

| 指标 | 正常值 |
|---|---|
| 得分 | **210.00**（满分） |
| 中心误差 | 0.10 ~ 0.15 m |
| 横向偏差 dy | ≤ 0.15 m |
| 航向误差 | ≤ 2° |
| 加速度（0.1s / 1s 窗口） | ≤ 3 m/s² |
| 用时 | 28 ~ 31 s |

## 8. 参数在哪

全部在 `tools/park_v8.py` 顶部（v8 是几何推导版，**没有任何场景常数**，推荐用这个）：

```python
ARC_RADIUS = 5.26        # 倒车满舵半径（车辆属性，实测）
STAGE_SPEED = 0.14       # 接近段油门
V_APPROACH = 3.6         # 接近段目标速度
HOLD_SECONDS = 3.0       # 停稳保持
```

平滑三件套（决定加速度分，改之前先读 `docs/06-results.md`）：

| 项 | 值 | 位置 |
|---|---|---|
| 油门斜率 | 0.0015/帧 | `send()` 里 `lim["thr"]` |
| 刹车斜率 | 0.004/帧 | `send()` 里 `lim["brk"]` |
| 提前减速距离 | 20 m | `if d_stage < 20.0` |

## 9. 排错表

| 现象 | 原因 | 解决 |
|---|---|---|
| `ModuleNotFoundError: metacar` | 用了系统 python | 换 `.venv\Scripts\python.exe` |
| 一直 `等待场景连接...` | 顺序反了 | 先跑脚本，再点「启动场景」 |
| 报「场景文件不存在」 | 场景包多一层目录 | 跑 `tools\fix_scene_dirs.py` |
| 分数 0.00 | 没停进车位 / 平台偶发 | 重跑一次；连续 0 再查日志 |
| 分数 200 | 加速度没达标 | 检查油门/刹车斜率有没有被改大 |
| 车倒过头（x 到 −14） | 对位点或判据被改坏 | 对照本文件第 8 节恢复参数 |
| 端口 5061 被占用 | 上次脚本没退干净 | 任务管理器结束 `python.exe` |

## 10. 想改进哪里

- **横向偏差**：调 `ARC_END_Y_FIX`（大一点往 +y，小一点往 −y）
- **停得深浅**：调 `abs(dx) < 0.06` 这个判据
- **更慢更稳**：把油门斜率再降到 0.0010，代价是总时长 +5 s

改完记得连跑 3 次，三次都 210 才算稳定。

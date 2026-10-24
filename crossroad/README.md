# crossroad — 十字路口通行（场景元素 18）

> 状态：✅ **210.00 分**（基础 200 + `|a| ≤ 5` 加分 10），连续 3 次复现
> 入口：`main.py`

---

## 1. 规则（表 18）

- 200 s 内从起点到终点（**200 分**基础）
- 压线 −20
- 碰撞对手车 −5×v
- **违反交通信号 −5×v**
- **红灯时车头距停止线 10 m 进入检测区，必须在距停止线 0~3 m 内停下，否则 −20**

---

## 2. 现场数据

| 项 | 值 |
|---|---|
| 路线 | 70 点 / **69.0 m**，纯直行穿过路口 |
| 起终点 | `(-5.59, 45.0)` → `(-5.59, -24.0)` |
| 道路 | 8 条（十字路口，四条引道各 2 车道） |
| **停止线** | `RoadInfo.stop_line` 直接给（实测 `(-4.0, 20.0)`） |
| **信号灯** | `SimMsg.traffic_light_groups[].traffic_lights[].straight_state` |
| 对手车 | 1 辆 CAR（终点外侧，不影响） |
| 静态障碍 | 4 个，在路口四角，不在路径上 |

---

## 3. 做法

```
读 route -> 求曲率 -> 车身坐标系纯跟踪
停止线 <- 道路数据里的 stopLine（在车正前方、最近的那条）
信号灯 <- traffic_light_groups 里本车方向最近的一组
红灯   <- v_cmd 按"到停止线剩余距离"收缩：v = sqrt(2*1.2*gap)，停在 0~3 m 内
纵向   <- PI 连续量，油门封顶 0.40（永不升挡）
```

### 3.1 关键：SDK 自带停止线

`RoadInfo` 里有 `stop_line: list[Vector2]`（别名 `stopLine`），**不用自己从道路几何推**。
挑"在车正前方（前向投影 > 0.8）、横向偏移 < 10 m、距离最近"的那条即可。

### 3.2 红灯停车

规则要求停在距停止线 **0~3 m**，所以目标不是"停在线前"，而是"停在线前 2 m 左右"：

```python
gap = max(dist_to_stopline - STOP_GAP_MAX, 0.0)     # STOP_GAP_MAX = 2.6
v_allow = sqrt(2 * 1.2 * gap)                        # 1.2 m/s² 匀减速
v_cmd = min(v_cmd, v_allow)
if dist <= 2.9:  v_cmd = 0                           # 稳住
```

绿灯再放行。`TrafficLightState` 枚举：`RED=1 / GREEN=2 / YELLOW=3`。

### 3.3 纵向沿用丁字路口验证过的 PI

```python
err = v_cmd - v
integ = clamp(integ + err*dt, -3, 24)
u = 0.10*err + 0.05*integ
throttle = min(0.40, u);  brake = 0 if u>=0 else min(1, -u*1.6)
```

0.40 封顶的意义：车稳定在 1 挡最高速 4.78 m/s，**永不升挡**，
避免换挡瞬间的速度跳变让平台判 `|a|` 超限（详见 `../AGENT_PLAYBOOK.md` §3.2）。

---

## 4. 实测

| 时间 | 得分 |
|---|---|
| 2026-10-24 14:44:35 | **210.00** |
| 2026-10-24 14:47:25 | **210.00** |
| 2026-10-24 14:48:25 | **210.00** |

- 69 m / 14.5 s（限时 200 s）
- 峰值 17.4 km/h，速度稳定 4.78 m/s，油门恒定 0.40
- 三次都是绿灯直行通过，无压线、无碰撞

> ⚠️ 三次实测都是**绿灯**开局，红灯停车逻辑尚未在实跑中被触发验证。
> 代码里已实现（按剩余距离收缩目标速度），但正式比赛若遇到红灯开局需要复测。

---

## 5. 用法

```powershell
.venv\Scripts\python.exe -u crossroad\main.py
```

```powershell
powershell -ExecutionPolicy Bypass -File tools\auto_scene.ps1 `
  -Script "crossroad\main.py" -SceneCp "5341,5B57,8DEF,53E3,901A,884C" -SkipAuto
```

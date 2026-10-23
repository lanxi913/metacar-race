# straight_full — 「车辆直行」全分跑分

面向「场景元素 / 车辆直行」这一题的独立跑分入口，目标是 **110 分**
（到达终点 100 + 全程 `|a| ≤ 5 m/s²` 加 10）。

与根目录 `main.py` 完全解耦：不改 `main.py`，也不依赖 `metacar_race/control/`。

---

## 1. 用法

```powershell
# 终端 A：先跑脚本，它会阻塞在 connect
.venv\Scripts\python.exe -u straight_full\main.py --cruise 0.5

# 终端 B / 客户端：点「启动场景」
# 终端开始刷 pos/v，跑完打一行 SUMMARY
```

一键跑分（自动点「自动驾驶」→「启动场景」，跑完打印日志尾部）：

```powershell
powershell -ExecutionPolicy Bypass -File tools\auto_straight.ps1 -PythonArgs "--cruise 0.5"
```

---

## 2. 为什么默认用「定油门巡航」

这是本题最关键的一条经验，写在这里免得后人再踩：

**不要在这个场景上用速度闭环去追油门。**

仿真的油门响应有明显滞后，30 Hz 下用 PI 闭环会在 `0.45 → 0 → 0.17` 之间反复抖，
实测瞬时加速度被顶到 **19 m/s²**，直接丢掉 `|a| ≤ 5` 的 10 分（总分 100 而不是 110）。

定油门巡航的做法是：油门只朝目标值做**单调斜坡**（`--cruise-rate`，默认 0.6/s），
全程不踩刹车，靠油门开度自然收敛到低挡位平台速度 17.2 km/h。这样加速度几乎为 0，
`|a|` 那 10 分稳拿，且 49 m 的路 10 秒左右就跑完，离 100 s 时限很远。

实测：`--cruise 0.5` 连续多次 **110.00 分**。

---

## 3. 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--cruise` | 无 | 定油门值，建议 `0.5`；给了就走巡航模式 |
| `--cruise-rate` | `0.6` | 巡航油门每秒最大变化量，调小更平滑 |
| `--speed` | `10.0` | 闭环模式的目标速度 m/s（巡航模式忽略） |
| `--accel` | `2.0` | 闭环模式的加速度上限 m/s² |
| `--no-kick` | 关 | 禁用低挡位升挡 kick |
| `--steer-sign` | `-1` | 转向符号；车往反方向偏就改 `+1` |
| `--seconds` | `120` | 墙钟运行上限 |

---

## 4. 输出

- 终端每 10 帧一行：`pos / yaw / v / thr / brk / str / lat / left`
- 结束一行 `SUMMARY {json}`：`reached_end`、`s_done`、`v_peak_kmh` 等
- `logs/frames_<时间戳>.csv`：逐帧数据（`t, v, a, throttle, brake, steer, s_left`）
- `logs/straight_<时间戳>.json`：本次 SUMMARY

> `SUMMARY` 里的 `a_max_abs` 是拿 `car.speed` 差分算的，受仿真速度信号抖动影响，
> 会明显大于真实加速度（实测 19~21，而平台判定通过）。**以平台「得分」为准**，
> 这个字段只作参考。

---

## 5. 实现要点

- `RoutePath`：把 route 点列做成可按里程查询的路径，带线段投影，得到连续的剩余里程
- `SpeedPlanner`：梯形速度规划，受加速度上限约束（闭环模式用）
- `LongController`：加速度误差积分出油门/刹车，带低挡位 kick（闭环模式用）
- `StraightDriver`：纯跟踪转向（前视距离随速度自适应）+ 上述纵向策略
- 转向符号：场景 `yaw` 顺时针为正，`steering > 0` 对应往 `-x` 侧偏，需要取反

---

## 6. 排错

| 现象 | 原因 | 动作 |
|---|---|---|
| 一直「加载中」 | 先开了场景 | 关场景 → 跑脚本 → 再开场景 |
| 车往一侧偏出路面 | 转向符号反了 | `--steer-sign 1` |
| 拿到 100 而不是 110 | 某处 `|a| > 5` | 用 `--cruise` 走定油门，不要用闭环 |
| 端口 5061/5063 占用 | 上次脚本没退 | 结束残留 `python.exe` |
| `ModuleNotFoundError: metacar` | 用了系统 python | 换 `.venv\Scripts\python.exe` |

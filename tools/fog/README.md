# 港口迷雾协同运输（PORT-FOG-01）运行说明

> 本目录是「港口迷雾协同运输」场景的完整实现，**已实测 800/1000**（信号 240 + 路障扫描 160 + 中央路口 2×100 + 泊位 A 交付 200），复现十余次。

## 1. 环境

- Windows + PowerShell（`tools/auto_scene.ps1` 用 UI Automation 打开元动力客户端并点「启动场景」）
- Python：仓库自带 `.venv`（`metacar`、`metacar-multiagent` 已装好）
- 场景包：`D:\无人算法\场景包\scenes\PORT-FOG-01`（客户端已注册）
- **本目录所有 `.py` / `.md` 都是 UTF-8 无 BOM**（早期提交曾误用 UTF-16LE，导致 `SyntaxError`，已修正）

## 2. 一键跑

```powershell
cd 'D:\无人算法\metacar-race'
# 先把上一局残留的进程杀干净，否则会 throw "client window not ready"
Get-Process SmartCar,metapower_desktop -ErrorAction SilentlyContinue |
  ForEach-Object { Stop-Process -Id $_.Id -Force }
Start-Sleep -Seconds 12

$env:FOG_DELIVER_B = '1'          # 打开「泊位 B」阶段
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force
& .\tools\auto_scene.ps1 `
  -Script '<仓库绝对路径>\tools\fog\fog_mission.py' `
  -SceneCp '6E2F,53E3,8FF7,96FE' `          # 「港口迷雾」四个字的码点
  -PythonArgs '--out <日志目录> --seconds 620 --cross 1 --deliver 1' `
  -TimeoutSeconds 900
```

场景名码点（`-SceneCp`）：`6E2F`港 `53E3`口 `8FF7`迷 `96FE`雾。

## 3. 关键参数（`fog_mission.py` 顶部常量）

| 常量 | 值 | 说明 |
| --- | --- | --- |
| `SIGNALS` | A (587.47,124.78) / B (766.5,-34.5) / C (720.5,-96.0) | 三个现场信号，**必须 A→B→C 顺序提交** |
| `CENTER` | (728.79,-20.22) | 中央路口中心；南北闸口净宽 5.06 m，中心线 x=728.79 |
| `STOP_FAR` | 6.0 m | 信号读码/交付的停车距离 |
| `E_REF` / `V_CAP` | — | 行车速度上限 5.5 m/s（超速判定 20.5 km/h，持续 0.5 s 才罚） |

## 4. 文件说明

| 文件 | 作用 |
| --- | --- |
| `fog_mission.py` | 主流程：预就位 → 三信号 → 路障扫描 → 两车路口通行 → 泊位 A/B 交付 |
| `fog_drive.py` | 纯跟踪执行器 `LaneDriver`（前视、限速、末端收速） |
| `fog_nav.py` | 栅格代价图 + A*（`_rect` 按构件自身坐标系膨胀，避免长条构件变黑圆） |
| `fog_map.py` | 路网图（官方 successor 几何不闭合，仅作投影/最近点搜索） |
| `fog_route.py` | 车道中心线取点（每 2 m 一点，密集航点才能切弯不脱路） |
| `fog_probe.py` / `fog_probe_b.py` | 现场探针：四车状态、可见对象、泊位有效观测点标定 |

## 5. 现场硬结论（踩过的坑）

- 验证码在 `get_observation(vid).visible_objects[].hint`，≤7 m 时 `state=key_visible`，格式 `KEY-<字母><数字>`；**滑行中提交**才行，停稳后提交返回 `not_available`。
- `scan` **不接受 dict payload**；必须先停稳。
- 路口：南口 `request_entry` 获批（约 (731,-31)），沿 **x=728.79 中心线**北上，到 **(728.8,-9.7)** `release` 成功；走 x=730.9 会贴东墙卡死；**未获批不得进入 6 m 区**（否则整局 `Mission has ended`）。
- 泊位观测：同一等级提升需**两辆不同车**、观测点间距 ≥5.30 m、**30 s 内**；B 泊位唯一有效观测点是**正西 8.6 m (826,-65.7) 且车头朝东**。
- 交付：必须先 `GearMode.PARKING + brake=1.0` 静置 ≥2 s，否则报 `must hold a stable position`；目标必须用 lv3 后的**精确点**（A = (629.47,-66.72)），不是车位矩形中心。
- `drive_polyline` **不会掉头**：目标在车后 90°~180° 时必然 stall，需要先 `turn_around()`。

# 比赛日运行手册（2026-11-01，正式赛题 6 个场景）

> 来源：《竞赛事宜》PDF —— 正式赛题只有 **连续场景1、连续场景4、VLA 3 个、港口迷雾**。
> 客户端「比赛列表 → 2026智能无人系统算法挑战赛（邀请赛）」里能看到这 6 个（名字带「（邀请赛2026）」）。
> 所有命令都在仓库根目录 `C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race` 下执行，
> 解释器一律 `.venv\Scripts\python.exe`。

## 0. 赛前 10 分钟（只做一次）

> **赛题场景赛前起不来**：2026-11-01 05:51 实测，在「比赛列表」里点赛题场景的「启动场景」，
> 平台回 `HTTP/1.1 400 Bad Request`，控制器连不上 5061。09:30 之前只能跑训练场景。

```powershell
# 1) 确认环境
.venv\Scripts\python.exe check_env.py

# 2) 确认代码是最新的（本次定版）
git log --oneline -5          # 应看到 "docs(progress): CS4 回到 1600…" 或之后

# 3) 开始录屏（OBS 全屏录制，mkv，30 fps，4000 Kbps）——赛题要求提交录屏
# 4) 清掉残留场景进程
Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force
```

> **判分只认平台服务器**。每局跑完 20~25 s 后用下面这条查分（不用截结算画面）：
> ```powershell
> .venv\Scripts\python.exe tools\last_run.py <slug> 1 --det
> ```
> slug：`lianxu` / `lianxu4` / `vla_TextNavParking_02` / `vla_IllegalParkingDetection` / `vla_ZoneComplianceCheck` / `PORT-FOG-01`

## 1. 六个场景怎么跑

### 1.1 连续场景1（满分 650，约 6 分钟）

```powershell
& .\tools\run_scene.ps1 -Slug lianxu `
    -PythonArgs "tools/park_cs_single_arc.py --skip-to-index 0 --hold 15 --max-seconds 420" `
    -MaxSeconds 460 -SettleSeconds 25
.venv\Scripts\python.exe tools\last_run.py lianxu 1 --det      # 期望 650.00
```

### 1.2 连续场景4（满分 1600，约 12 分钟）

```powershell
& .\tools\run_scene.ps1 -Slug lianxu4 `
    -PythonArgs "tools/park_cs4.py --elements-only 起步直行,车辆直行,连续转弯,环岛,复杂路况超车,匝道行驶,车辆紧急停止,辅路,平行车道泊车 --vmax 9.0 --max-seconds 810" `
    -MaxSeconds 900 -SettleSeconds 25
.venv\Scripts\python.exe tools\last_run.py lianxu4 1 --det     # 期望 1600.00
```

### 1.3 VLA 文本导航与规范停车（满分 1000，约 3 分钟）

**训练场景**（锦汇华庭，验证过 1000）：

```powershell
& .\tools\run_scene.ps1 -Slug vla_TextNavParking_02 `
    -PythonArgs "tools/vla_text_nav.py --vmax 3.8 --ocr-window 8 --max-seconds 155" `
    -MaxSeconds 200 -SettleSeconds 20
.venv\Scripts\python.exe tools\last_run.py vla_TextNavParking_02 1 --det   # 期望 1000.00
```

**赛题场景（09:30 之后才能起）**：

```powershell
& .\tools\run_scene.ps1 -Slug ius_vla_text `
    -PythonArgs "tools/vla_text_nav.py --vmax 3.8 --ocr-window 8 --max-seconds 155" `
    -MaxSeconds 200 -SettleSeconds 20
.venv\Scripts\python.exe tools\last_run.py vla_TextNavParking_ius2026 1 --det
```

- 赛题题面是「147培训基地西入口 / 办公楼B西入口 / 147培训基地停车区01号停车位」，
  指令措辞有 6 种（见 `docs/PROGRESS.md` 第 0.4 节），代码已按这些措辞适配并离线单测；
  **只有 9:30 后才能真机验证**。
- 指令是「找到并停入…停车位」变体时 → 走绝对车位；是「…XX西入口」变体时 → 直接去那栋楼
  西面外 6 m（跳过巡航找门牌）。

### 1.4 VLA 停车区违规检测 / 功能区合规检测（各 1000，目前没有可用的实现）

- 停车区违规检测：`tools/vla_parking.py` 是几何版（报禁停区里的贴纸 id），但 11-01 探针实测
  该场景 `obstacles` 恒为 0、车也开不动 → 拿不到任何违规对象，跑也是 0 分。
- 功能区合规检测：仓库里没有实现。
- **建议**：这两个场景开赛后各跑一局把流程走完（提交空结果不扣分，只是 0），
  把时间留给上面四个能拿分的场景。

### 1.5 港口迷雾（满分 1000，约 11 分钟）

**顺序和单车场景相反：先起场景，再跑脚本**（场景是 9100 服务端，算法是客户端）。

```powershell
& .\tools\run_scene.ps1 -Slug PORT-FOG-01 -SceneFirst `
    -PythonArgs "fog_mission.py --out `"$env:TEMP\fog_run`" --seconds 620 --cross 1 --deliver 1" `
    -WorkDir "C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog" `
    -MaxSeconds 700 -SettleSeconds 25
.venv\Scripts\python.exe tools\last_run.py PORT-FOG-01 1 --det        # 期望 900~950
```

跑完用 `fog_report.py` 一屏看 8 项成果（**必须设 UTF-8**）：

```powershell
$env:PYTHONIOENCODING="utf-8"; [Console]::OutputEncoding=[System.Text.Encoding]::UTF8
& .venv\Scripts\python.exe "C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog\fog_report.py" "<日志目录>"
```

期望：信号 3/3、路障扫描 granted、两车过闸成功、两个泊位交付成功、碰撞 0、490 s 左右结束。
计分口径：8 项成果 1000 分，每条约 3.8 分违规扣分；起步急加速那条压不掉（平台瞬态）。

## 2. 三条硬规矩（踩过坑的）

1. **顺序**：单车场景先跑脚本再点「启动场景」（脚本是 5061 服务端）；
   港口迷雾反过来，先起点场景再跑脚本（9100）。
   `run_scene.ps1` 已经按 `-SceneFirst` 帮你处理，别自己反着来。
2. **别急着杀场景**：脚本退出后 `-SettleSeconds 25` 才关 SmartCar。杀太快平台不记账
   （CS4 第一次跑到 1570 就被吞了，VLA 也一样）。
3. **一局失败先看日志再重跑**，不要连着重跑：
   - 一直「加载中」= 启动顺序反了；
   - `ModuleNotFoundError: metacar` = 用错 python；
   - 端口占用 = 上次脚本没退干净（`Get-NetTCPConnection -LocalPort 5061`）。

## 3. 时间预算（9:30 开赛 → 11:30 结束，共 120 分钟）

| 场景 | 一局耗时 | 建议局数 | 小计 |
|---|---|---|---|
| 连续场景4 | 12 min | 2（留一次重跑） | 24 min |
| 港口迷雾 | 11 min | 2 | 22 min |
| 连续场景1 | 6 min | 2 | 12 min |
| VLA 文本导航 | 3 min | 6~8（变体随机，抢到「停车位」变体就是 1000） | 24 min |
| VLA 违规/功能区检测 | 2 min | 各 1（走流程） | 4 min |
| 余量 | | | 34 min |

> VLA 那 6~8 局是有意义的：指令变体随机，跑得越多越可能撞上能满分的「停车位」变体，
> 平台按**最好一次**记录（run-records 里同场景多条，取最高）。

## 4. 本次定版的代码位置

| 场景 | 入口 | 分支 |
|---|---|---|
| 连续场景1 | `tools/park_cs_single_arc.py` | `feat/competition-ready` |
| 连续场景4 | `tools/park_cs4.py` + `metacar_race/scene4/` | `feat/competition-ready` |
| VLA 文本导航 | `tools/vla_text_nav.py` + `metacar_race/perception/` | `feat/competition-ready` |
| VLA 违规检测 | `tools/vla_parking.py`（未验证） | `feat/competition-ready` |
| 港口迷雾 | `.train/fog800/tools/fog/fog_mission.py` | `feat/fog-port-competition`（远端）|

提交记录：`git log --oneline -12` 里 11-01 的那几个 merge/fix/perf 提交。

# 用户背景 + 操作手册（接手必读）

> 换模型后先读这份，再读 `docs/12-MODEL-HANDOFF.md`（项目状态）和 `docs/00-PROJECT-OVERVIEW.md`（规则与技术）。

---

# 第一部分：我是谁

## 基本背景

| 项 | 内容 |
|---|---|
| 姓名 | 王烨 |
| 学校 | 哈尔滨工业大学（威海）· 汽车工程学院 |
| 专业 | 交通工程（2025 级，2026-07 转入） |
| 学号 | 2025211447 |
| GitHub | `lanxi913` |
| 邮箱 | 2121193604@qq.com |

## 这个项目是什么

2026 智能无人系统算法挑战赛（智能车 / 自动驾驶赛项），**5 人队**，全员用 AI 协作开发。
我做的是 MetaCar 仿真里的元素场景（直行、转弯、泊车、路口），目标每个场景拿满分 + 奖励分。

- 报名截止 **2026-10-25**
- 正式比赛 **2026-11-01**
- 仓库：https://github.com/lanxi913/metacar-race

## 我的机器环境

| 项 | 值 |
|---|---|
| 系统 | Windows，用户目录 `C:\Users\lanxi` |
| 项目路径 | `C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race` |
| Python | **三个**：Anaconda 3.12.4（`C:\Users\lanxi\anaconda3\python.exe`，包最全）、3.9.13（PATH 上的 `python`）、3.14.2（几乎没包）|
| 本项目用 | 项目自带 `.venv`（`.venv\Scripts\python.exe`）——**必须用这个** |
| 代理 | `http://127.0.0.1:7897`，git push 必须走它 |
| C 盘空间 | 只剩约 32 GB，大文件放 D 盘 |
| 没有的软件 | ffmpeg、ImageMagick、LibreOffice、pnpm、Java、Go、Rust、Docker |

## 我的工作习惯（照做，不然我会骂人）

1. **一轮对话只跑一次仿真**——改一处、跑一次、把日志给我看，我告诉你下一步
2. **要具体数字和下一步动作**，不要长篇分析、不要"我觉得"
3. **说"完成"之前必须真的验证过**——成绩以仿真结算画面「总分」为准
4. **卡住就停下来讲清楚卡在哪**，别继续瞎试
5. 中文回答，术语保留英文
6. 交付给完整的东西：能直接粘贴的命令、能直接跑的整份代码

---

# 第二部分：怎么操作（一步一步）

## 1. 开仿真（每次跑分前）

```
打开「元动力」客户端（桌面快捷方式）
→ 停在场景列表页
→ 确认列表滚到第 1 页（泊车类场景在第 2 页，要先点 "Go to page 2"）
```

## 2. 清残留（必做，否则连不上）

上次的 Python 脚本可能还占着 5061 端口，导致场景一直显示"等待算法连接"。

```powershell
Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
  Where-Object { $_.CommandLine -like '*track_*' -or $_.CommandLine -like '*park_*' -or $_.CommandLine -like '*stop_run*' } |
  ForEach-Object { Stop-Process -Id $_.ProcessId -Force }
Get-Process -Name SmartCar,UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force
```

## 3. 起脚本（顺序不能反）

```powershell
cd C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race
$env:PYTHONIOENCODING='utf-8'
Start-Process -FilePath '.\.venv\Scripts\python.exe' -ArgumentList @('-u','tools\跑分脚本.py') `
  -RedirectStandardOutput 'logs\run.log' -RedirectStandardError 'logs\run.err' -NoNewWindow
Start-Sleep -Seconds 6
```

脚本会阻塞着打印 `等待场景连接...`，**这是正常的**，别以为卡死。

## 4. 点「启动场景」

```powershell
& .\tools\launch_scene.ps1 -Title "场景名"
```

成功会打印 `launched: 场景名 (dist=xx)`。
**失败两种情况**：
- `title xxx not found` → 场景不在当前页，先切页或滚回顶部
- `client window not found` → 客户端没开

## 5. 等结果 + 看分数

脚本跑完（日志出现 `到达终点` / `驻车完成` / `SUMMARY`）后，仿真画面中央会出现：

```
测试结束
总分：110.00
```

**右上角**同时列出本次所有违规，例如：

```
16:22:12 - [加速度限制] 加速度高于允许上限
16:22:15 - [压线检测] 压路边沿线扣5分
```

有违规 = 奖励分丢了（110→100）。

## 6. 截图存档

```powershell
& .\tools\capture_window.ps1 -Frames 1 -IntervalMs 200 -CropLeft 0 -CropTop 0.2 -CropRight 1 -CropBottom 1
Copy-Item "$env:TEMP\hudseq2\c01.png" "logs\results\场景名_$(Get-Date -Format HHmmss).png"
```

这个脚本按**窗口句柄**抓图，不受前台焦点影响。

## 7. 记成绩

```powershell
Add-Content -Path docs\results.csv -Encoding UTF8 -Value "2026-10-24 17:03:06,场景名,slug,脚本,110.00,备注"
```

同时把成绩写回 `docs/PROGRESS.md` 的成绩总表。

## 8. 提交代码

```powershell
git add -A
git commit -m "feat(场景): 做了什么"
$env:HTTPS_PROXY='http://127.0.0.1:7897'
git -c http.proxy=http://127.0.0.1:7897 push origin main
```

## 9. 关窗口（每跑几局清一次）

```powershell
Get-Process -Name SmartCar,msedge -ErrorAction SilentlyContinue | Stop-Process -Force
```

**别杀 `msedgewebview2`**——那是客户端内嵌的网页运行时，杀了客户端白屏。

---

# 第三部分：常见故障对照

| 现象 | 原因 | 处理 |
|---|---|---|
| 场景显示"等待算法连接"不动 | 旧脚本占着 5061 | 执行第 2 步清残留 |
| `ModuleNotFoundError: metacar` | 用了系统 python | 改用 `.venv\Scripts\python.exe` |
| "场景文件不存在" | 场景包多解压了一层目录 | `python -u tools\fix_scene_dirs.py` |
| 点「启动场景」启错场景 | 列表滚动位置变了 | 滚回顶部再点 |
| 脚本一启动就崩，车不动 | 看 `logs\*.err` | 通常是字段名错（如 `center` 应为 `pos_x`） |
| PowerShell 脚本中文乱码 | 文件丢了 BOM | 用 UTF-8 with BOM 保存 |
| 车原地打转 | 转向符号反了 | `steer = -atan2(2·ly,d)/0.6`，见总览文档 |
| 倒车飙到 6 m/s | 恒定油门 | 改速度闭环，油门只给 0.10 / 0 |

---

# 第四部分：当前进度（详见 docs/PROGRESS.md）

**17 个场景已满分**：直行、停止、直角、u 型、连续转弯、环形、环岛、限速、变道、超车、辅路、紧急停止（各 110）；
垂直泊车、平行泊车、十字、丁字、左转（各 210）；连续场景 1 = 630。

**还剩 3 项**：

| 场景 | 现状 | 下一步 |
|---|---|---|
| 异形车道泊车 | 未跑通 | 读 `docs/11-angled-parking-handoff.md`，**先做倒车转向标定** |
| 斜坡泊车 | 未开工 | 先跑通平地平行泊车（已 210），再加 15° 坡补偿 |
| 车辆跟随 | 100 | 找出那 1 次加速度违规的时刻，调减速曲线 |

## 交接第一件事

```powershell
cd C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race
git pull origin main
```

然后先做**倒车转向标定**：倒挡固定 `steer=+0.3` 跑 2 秒看 yaw 变化，再 `-0.3` 跑 2 秒。
10 秒出结果，能省掉后面一串符号错误。

# 环境搭建（可复现）

这份文档记录的是**一台机器从零到跑通**的完整过程，照着做就能得到一样的环境。
最后更新：2026-10-22（实际验证通过）。

---

## 0. 前置条件

| 项目 | 要求 | 本机实际 |
|---|---|---|
| 系统 | Windows 10 及以上 | Windows 11 |
| CPU / 内存 | i5-10210 起、16 GB | 满足 |
| 显卡 | 建议独立显卡 | 满足（核显会黑屏退出） |
| 磁盘 | 512 GB，**场景包很大，放 D 盘** | C 120 GB 可用 / D 202 GB 可用 |
| Python | 3.9+（SDK 要求 **≥3.10**） | 3.12.4 |

---

## 1. 安装客户端（启动器）

1. 打开 <https://www.i-mylab.com/icv/>，右上角注册登录。
2. 页面右侧「**下载启动器**」→ 得到 `元动力客户端_1.1.1_x64-setup.exe`。

> **Edge 拦截**：下载会提示「通常不会下载…」，这是文件没有数字签名导致的信誉提醒，
> 不是病毒。点「保留」即可；被拦下的文件会以 `未确认 xxx.crdownload` 躺在
> `%USERPROFILE%\Downloads`，可以直接改名成 `.exe` 使用。

3. 右键「以管理员身份运行」安装。**装的时候把目录改到 D 盘**：

```
D:\Program Files\元动力客户端
```

安装包是 NSIS，支持静默安装并指定目录（命令行末尾的 `/D=` 不能加引号）：

```bat
"元动力客户端_1.1.1_x64-setup.exe" /S /D=D:\Program Files\元动力客户端
```

装完验证：

```powershell
# 注册表里的安装位置应该是 D 盘
Get-ItemProperty 'HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*' |
  Where-Object { $_.DisplayName -match '元动力' } |
  Select-Object DisplayName, InstallLocation
```

### 已经装到 C 盘了怎么办

标准做法是卸载重装：运行 `%LOCALAPPDATA%\元动力客户端\uninstall.exe /S`，再用上面的
`/D=` 参数重装。用户数据在 `%LOCALAPPDATA%\com.i-mylab.metapower-desktop`，卸载不会删。

---

## 2. 下载场景包

客户端左侧「自动驾驶」→ 点场景卡片的「下载场景」。

- 首次会先下 **公共资源包**（约 436 MB），所有场景共用，只下一次；
- 然后下场景本身（「车辆直行」解压后约 371 MB）；
- 合计 **约 811 MB / 327 个文件**。

### 场景默认落在 C 盘，建议挪到 D

默认目录：

```
%APPDATA%\com.i-mylab.metapower-desktop\car
```

把它做成**目录联接**指向 D 盘，程序无感，C 盘不再增长（不需要管理员权限）：

```powershell
# 先退出客户端，确保没有进程占用
$src = "$env:APPDATA\com.i-mylab.metapower-desktop\car"
$dst = "D:\元动力场景"

New-Item -ItemType Directory -Force -Path $dst | Out-Null
Copy-Item "$src\*" $dst -Recurse -Force
Remove-Item $src -Recurse -Force

cmd /c mklink /J "$src" "$dst"
```

验证：`(Get-Item $src).LinkType` 应为 `Junction`，`Target` 为 `D:\元动力场景`。

> 39 个场景全下大约 12–15 GB（压缩包 6.4 GB，解压后更大），所以务必放 D 盘。

---

## 3. Python 环境

**不要用系统裸 Python**，每个项目一个虚拟环境，避免包冲突。

```bat
cd C:\Users\<你>\Documents\deepseek-harness\projects\metacar-race

:: 用 Anaconda 的 3.12 建 venv（也可以直接用任意 3.10+ 的 python -m venv）
C:\Users\<你>\anaconda3\python.exe -m venv .venv

.venv\Scripts\activate
python -m pip install -U pip
python -m pip install -r requirements.txt
```

`requirements.txt`：

```
metacar==0.4.0
metacar-multiagent==0.1.0
```

> 两个包都在 PyPI 上，版本正好是比赛要求的 0.4.0 / 0.1.0。
> 要求 `python_requires >= 3.10`，3.9 装不上。

装完的依赖（2026-10-22 实测）：

```
metacar 0.4.0, metacar-multiagent 0.1.0
numpy 2.5.4, opencv-python 5.0.0.93, pydantic 2.14.1
```

### 自检

```bat
.venv\Scripts\python.exe check_env.py
```

期望输出：

```
Python : 3.12.4 (...\.venv\Scripts\python.exe)
metacar: 0.4.0
metacar_multiagent: 0.1.0
port 9100: 未监听（先跑脚本、再开场景）
结果: 通过
```

端口未监听是正常的——场景没开的时候本来没人连。

---

## 4. 连接原理（决定了启动顺序）

`metacar.SceneAPI` **不是客户端，是服务端**：

```python
# metacar/sceneapi.py
self._model_socket = ModelSocket("127.0.0.1", 5061)     # JSON 数据
self._streaming_socket = StreamingSocket("127.0.0.1", 5063)  # 视频流
def connect(self):
    self._model_socket.accept()      # 阻塞，等场景来连
    self._streaming_socket.accept()
```

所以顺序必须是：

1. **先跑 Python 脚本** → 它监听 5061/5063 并阻塞在 `accept()`；
2. **再启动场景** → Unity 场景（SmartCar.exe）主动连上来；
3. 连上后 `main_loop()` 开始出帧。

顺序反了就是「一直加载中」：场景连不上，脚本也没在等。

多车场景走 `metacar_multiagent.ScenarioTaskAPI`，默认端口 **9100**。

---

## 5. 实测记录（车辆直行，2026-10-22）

| 测试 | 控制 | 结果 |
|---|---|---|
| 只刹停 | `gear=PARKING, brake=1.0` | 20 s 收 601 帧，v=0，位置不动 |
| 油门 0.5 | `gear=DRIVE, throttle=0.5` | **卡在 17.2 km/h**，一路匀速到终点 |
| 油门 1.0 | `gear=DRIVE, throttle=1.0` | 0→109 km/h，2.7 s 冲过终点 |

**17.2 km/h 是挡位问题，不是动力不足**：低挡位下油门再大也上不去，必须把油门顶到 1.0
触发升挡，之后才会继续加速。写速度环时，只要目标速度超过 17 km/h，油门输出就必须能到 1.0。

另外：车辆冲过终点后 `main_loop()` 会自然结束，脚本正常退出。

---

## 6. 常见报错对照

| 现象 | 原因 | 处理 |
|---|---|---|
| 一直「加载中」 | 先开场景后跑脚本 / 脚本已退出 | 关场景 → 跑脚本 → 看到阻塞 → 再开场景 |
| 有声音但黑屏/黑白屏后退出 | 显卡性能不足 | 换独显机器，关掉其它占 GPU 的程序 |
| `ModuleNotFoundError: metacar` | 解释器不是 `.venv` 里的 | 用 `.venv\Scripts\python.exe`，VSCode 里 Select Interpreter |
| 脚本在跑，车不动 | 程序已异常退出 / 控制值全 0 | 看终端最后 20 行；确认每帧都在 `set_vehicle_control` |
| 速度上不去（≈17 km/h） | 卡低挡 | 油门给 1.0 等升挡 |
| 端口 5061/5063 被占用 | 上一次的脚本没退干净 | 任务管理器结束对应 python.exe |

---

## 7. 换一台机器 checklist

- [ ] 客户端装在 D 盘，能打开、能登录
- [ ] 至少下载一个场景（车辆直行）
- [ ] `car` 目录已改为指向 D 盘的 junction
- [ ] `git clone` 本仓库，`pip install -r requirements.txt`
- [ ] `check_env.py` 三项通过
- [ ] 先跑 `main.py`、再开场景，能看到 30 fps 刷状态

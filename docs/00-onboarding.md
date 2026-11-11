# 新人接入（复制即用）

仓库：<https://github.com/lanxi913/metacar-race>（公开，直接 clone，无需邀请）

---

## 第一步 · 环境

```bat
git clone https://github.com/lanxi913/metacar-race.git
cd metacar-race
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
.venv\Scripts\python.exe check_env.py
```

`check_env.py` 三项都得过：Python 版本、两个包能导入、端口状态。
端口显示「未监听」是正常的，场景没开的时候本来没人连。

---

## 第二步 · 跑通一次（顺序不能反）

```bat
:: 1) 先跑脚本，它会卡在 connect，这是正常的
.venv\Scripts\python.exe -u main.py
```

2）再去客户端点「启动场景」。
3）终端开始刷 `[ 0.3s] pos=(...) v=...` 就成功了。

---

## 第三步 · 领任务

1. 打开 <https://github.com/lanxi913/metacar-race/issues>
2. 找 `P0` 标签里你能做的，**assign 给自己**
3. 去 Issue #1 贴一张终端截图，表示环境已就绪

---

## 用 AI 的队友

把仓库地址丢给你的 AI，让它先读根目录的 **`AGENTS.md`**。

里面有：环境绝对路径、启动顺序的原理、代码归属、评分规则的硬数字、排错对照表、
以及「不要做的事」（别重建环境、别写死坐标、别提交大文件）。

Claude Code 会自动读 `CLAUDE.md`，Codex / Cursor 会读 `AGENTS.md`，不用你解释。

---

## 常见卡点

| 现象 | 处理 |
|---|---|
| `ModuleNotFoundError: metacar` | 用 `.venv\Scripts\python.exe`，别用系统 python |
| 一直「加载中」 | 顺序反了：先脚本，再开场景 |
| 速度卡 17.2 km/h | 油门给到 1.0 等升挡（实测结论，见 docs/01） |
| 客户端装到 C 盘 | 卸载后用 `/D=D:\...` 重装，见 docs/01 |

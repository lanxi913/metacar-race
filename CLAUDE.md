# CLAUDE.md

本项目的全部上下文、环境、约定、评分数字和排错表都写在 **[AGENTS.md](AGENTS.md)**，请先读它。

要点速记：

- 用 `.venv\Scripts\python.exe`，不要用系统 python
- 先跑 `main.py`（阻塞在 connect），再开场景
- 不写死坐标 / 验证码 / object_id
- 只改自己负责的子包；入口 `main.py` 归 A
- 提交前跑 `check_env.py`

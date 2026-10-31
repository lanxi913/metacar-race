# -*- coding: utf-8 -*-
"""在**生效的** main()（第二个）里挂上 10 Hz 轨迹落盘。

fog_mission.py 里有两份 main（前半部分是历史残留），这里按"第二次出现"定位。
"""
import io
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
p = os.path.join(FOG, "fog_mission.py")
s = io.open(p, encoding="utf-8").read()

anchor = '    t0 = time.time()\n'
idx1 = s.find(anchor)
idx2 = s.find(anchor, idx1 + 1)
assert idx1 != -1 and idx2 != -1, (idx1, idx2)

insert = (
    "    # 10 Hz 轨迹记录（量化画龙：转向抖动 / 横向摆动），退出时落盘\n"
    "    import atexit\n"
    "    import fog_run5 as _fr5\n"
    "    _fr5.TRACE = []\n"
    "\n"
    "    def _dump_trace():\n"
    "        rows = getattr(_fr5, \"TRACE\", None) or []\n"
    "        if not rows:\n"
    "            return\n"
    "        try:\n"
    "            with open(os.path.join(args.out, \"trace.csv\"), \"w\", encoding=\"utf-8\") as fh:\n"
    "                fh.write(\"t,vid,x,y,steer,thr,brk,speed\\n\")\n"
    "                for r in rows:\n"
    "                    fh.write(\"%.2f,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\\n\" % r)\n"
    "        except Exception:                                  # noqa: BLE001\n"
    "            pass\n"
    "\n"
    "    atexit.register(_dump_trace)\n"
)

pos = idx2 + len(anchor)
s = s[:pos] + insert + s[pos:]
io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("ok: trace 落盘已挂在生效的 main()")

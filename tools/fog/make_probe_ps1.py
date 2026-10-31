# -*- coding: utf-8 -*-
"""生成探针用的 ps1（复制 fog_run.ps1，只把跑 fog_mission 换成跑 fog_probe_accel）。"""
import os

FOG = r"C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800\tools\fog"
src = open(os.path.join(FOG, "fog_run.ps1"), encoding="utf-8-sig").read()

old_run = (
    "$proc = Start-Process -FilePath $Py `\n"
    "    -ArgumentList @((Join-Path $Fog 'fog_mission.py'), '--out', $Out,\n"
    "                    '--seconds', $Seconds, '--cross', '1', '--deliver', $Deliver) `\n"
)
new_run = (
    "$proc = Start-Process -FilePath $Py `\n"
    "    -ArgumentList @((Join-Path $Fog 'fog_probe_accel.py'), '--out', $Out) `\n"
)
assert src.count(old_run) == 1, ("run block", src.count(old_run))
src = src.replace(old_run, new_run)

old_say = 'Say "跑 fog_mission --seconds $Seconds"'
assert src.count(old_say) == 1, ("say", src.count(old_say))
src = src.replace(old_say, 'Say "跑 fog_probe_accel（加速度响应探针）"')
src = src.replace('Say "=== fog mission run $stamp  out=$Out ==="',
                  'Say "=== fog ACCEL PROBE $stamp  out=$Out ==="')

out = os.path.join(FOG, "fog_run_probe.ps1")
with open(out, "w", encoding="utf-8-sig", newline="\r\n") as fh:
    fh.write(src)
b = open(out, "rb").read(3)
print("written", out, "BOM:", " ".join("%02X" % x for x in b))

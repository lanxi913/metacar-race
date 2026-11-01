# -*- coding: utf-8 -*-
"""自己点启动场景，然后确认任务是否真的在跑（事件/位置/时钟）。"""
from __future__ import annotations

import json
import socket
import subprocess
import time

PS = r"""
Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes
$p = Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue | Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
if (-not $p) { Write-Host 'no-client'; exit 1 }
$win = [System.Windows.Automation.AutomationElement]::FromHandle($p.MainWindowHandle)
$els = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, [System.Windows.Automation.Condition]::TrueCondition)
$label = $els | Where-Object { $_.Current.Name -eq '港口迷雾' } | Select-Object -First 1
$btns = @($els | Where-Object { $_.Current.ControlType.ProgrammaticName -eq 'ControlType.Button' -and $_.Current.Name -eq '启动场景' })
if (-not $label -or $btns.Count -eq 0) { Write-Host 'not-found'; exit 2 }
$lx = $label.Current.BoundingRectangle.X; $ly = $label.Current.BoundingRectangle.Y
$best = $null; $bestD = 1e9
foreach ($b in $btns) {
  $dx = $b.Current.BoundingRectangle.X - $lx; $dy = $b.Current.BoundingRectangle.Y - $ly
  if ($dx -lt 20 -or $dx -gt 340 -or $dy -lt 60 -or $dy -gt 440) { continue }
  $d = [Math]::Sqrt($dx*$dx + $dy*$dy)
  if ($d -lt $bestD) { $bestD = $d; $best = $b }
}
if (-not $best) { Write-Host 'no-btn'; exit 3 }
$best.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()
Write-Host ("clicked d={0:N0}" -f $bestD)
"""


def call(p, sock, buf):
    sock.sendall((json.dumps(p) + "\n").encode())
    while b"\n" not in buf[0]:
        c = sock.recv(1 << 20)
        if not c:
            raise ConnectionError
        buf[0] += c
    line, rest = buf[0].split(b"\n", 1)
    buf[0] = rest
    return json.loads(line.decode("utf-8", "replace"))


print("[1] 点启动", flush=True)
r = subprocess.run(["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
                    "-Command", PS], capture_output=True, text=True)
print("   ", r.stdout.strip()[:120], r.stderr.strip()[:120], flush=True)

print("[2] 等场景起来（9100 可连 + 事件出现）", flush=True)
sock = None
for _ in range(90):
    try:
        sock = socket.create_connection(("127.0.0.1", 9100), timeout=3)
        sock.settimeout(10)
        break
    except Exception:
        time.sleep(1.0)
if sock is None:
    print("!! 9100 起不来", flush=True)
    raise SystemExit(2)
buf = [b""]

t0 = time.time()
for i in range(8):
    ev = call({"command": "get_events"}, sock, buf).get("events") or []
    o = call({"command": "get_observation", "vehicle_id": "Car_D"}, sock, buf).get("observation") or {}
    p = o.get("position") or {}
    print(f"   t+{i*2}s events={len(ev)} last={round(ev[-1].get('elapsed_seconds',0),1) if ev else 0} "
          f"Car_D=({p.get('x',0):.1f},{p.get('y',0):.1f}) phase={o.get('phase')}", flush=True)
    time.sleep(2.0)

# 试着推一下车，确认控制生效
a = call({"command": "get_observation", "vehicle_id": "Car_D"}, sock, buf).get("observation") or {}
ap = a.get("position") or {}
for _ in range(15):
    call({"command": "set_vehicle_control", "vehicle_id": "Car_D",
          "vehicle_control": {"gear": 2, "throttle": 0.4, "brake": 0.0, "steering": 0.0}},
         sock, buf)
    time.sleep(0.08)
call({"command": "set_vehicle_control", "vehicle_id": "Car_D",
      "vehicle_control": {"gear": 3, "throttle": 0.0, "brake": 1.0, "steering": 0.0}},
     sock, buf)
b = call({"command": "get_observation", "vehicle_id": "Car_D"}, sock, buf).get("observation") or {}
bp = b.get("position") or {}
import math
d = math.hypot(bp.get("x", 0) - ap.get("x", 0), bp.get("y", 0) - ap.get("y", 0))
print(f"[3] 控制验证：Car_D 动了 {d:.2f} m", flush=True)
sc = call({"command": "get_task"}, sock, buf).get("score") or {}
print("score:", {k: sc.get(k) for k in ("total_score", "signals_verified", "penalties")}, flush=True)
sock.close()

# -*- coding: utf-8 -*-
"""只启动、不打扰：点「启动场景」，然后纯读 9100，看任务时钟会不会自己走。"""
import subprocess
import sys
import time

PS = r"""
Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes
$p = Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue | Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
if (-not $p) { Write-Host 'no client'; exit 1 }
$win = [System.Windows.Automation.AutomationElement]::FromHandle($p.MainWindowHandle)
$els = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, [System.Windows.Automation.Condition]::TrueCondition)
$label = $els | Where-Object { $_.Current.Name -eq '港口迷雾' } | Select-Object -First 1
$btns = @($els | Where-Object { $_.Current.ControlType.ProgrammaticName -eq 'ControlType.Button' -and $_.Current.Name -eq '启动场景' })
if (-not $label) { Write-Host 'label not found'; exit 2 }
if ($btns.Count -eq 0) { Write-Host 'no start button'; exit 3 }
$lx = $label.Current.BoundingRectangle.X; $ly = $label.Current.BoundingRectangle.Y
$best = $null; $bestD = 1e9
foreach ($b in $btns) {
  $dx = $b.Current.BoundingRectangle.X - $lx; $dy = $b.Current.BoundingRectangle.Y - $ly
  if ($dx -lt 20 -or $dx -gt 340 -or $dy -lt 60 -or $dy -gt 440) { continue }
  $d = [Math]::Sqrt($dx*$dx + $dy*$dy)
  if ($d -lt $bestD) { $bestD = $d; $best = $b }
}
if (-not $best) { Write-Host 'no nearby start button'; exit 4 }
$best.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()
Write-Host ("clicked start, d={0:N0}" -f $bestD)
"""

print("[1] 点启动场景（不跑任何驱动脚本）", flush=True)
r = subprocess.run(["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
                    "-Command", PS], capture_output=True, text=True)
print("   ", r.stdout.strip(), r.stderr.strip()[:200], flush=True)

print("[2] 等 25 秒让场景初始化", flush=True)
time.sleep(25)

sys.path.insert(0, r"C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race")
from tools.portfog_agent import Agent  # noqa: E402

a = Agent()
print("[3] 纯读，看时钟", flush=True)
prev = None
for i in range(6):
    ev = a.events()
    st = a.task_state()
    o = a.obs("Car_C")
    p = o.get("position") or {}
    last_t = round(ev[-1].get("elapsed_seconds", 0), 1) if ev else 0
    print(f"   t+{i*2}s events={len(ev)} last_ev_t={last_t} phase={st.get('phase')} "
          f"Car_C=({p.get('x', 0):.1f},{p.get('y', 0):.1f})", flush=True)
    prev = last_t
    time.sleep(2.0)
print("done", flush=True)

# 港口迷雾：起客户端 → 点「启动场景」→ 跑任意脚本（诊断/正式都走这条）
# 用法: powershell -NoProfile -ExecutionPolicy Bypass -File fog_run_any.ps1 -Script fog_probe_path.py -PyArgs "--out ... --seconds 90"
param(
    [Parameter(Mandatory = $true)][string]$Script,
    [string]$PyArgs = "",
    [string]$Out = "",
    [int]$StartWait = 0
)

$ErrorActionPreference = 'Continue'
Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes

$Root = 'C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800'
$Fog = Join-Path $Root 'tools\fog'
$Py = 'C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race\.venv\Scripts\python.exe'
$ClientExe = 'D:\Program Files\元动力客户端\metapower_desktop.exe'

$stamp = Get-Date -Format 'MMdd_HHmmss'
if (-not $Out) { $Out = Join-Path $Root ('logs\p_' + $stamp) }
New-Item -ItemType Directory -Force -Path $Out | Out-Null
$console = Join-Path $Out 'console.log'

function Say($m) {
    $line = "[{0}] {1}" -f (Get-Date -Format 'HH:mm:ss'), $m
    Write-Output $line
    Add-Content -Path $console -Value $line -Encoding UTF8
}

function Invoke-El($el) {
    try { $el.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke(); return $true }
    catch { return $false }
}

function Launch-Scene([string]$sceneName) {
    $cw = Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue |
          Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
    if (-not $cw) { return $false }
    $win = [System.Windows.Automation.AutomationElement]::FromHandle($cw.MainWindowHandle)
    for ($page = 0; $page -le 5; $page++) {
        for ($try = 1; $try -le 4; $try++) {
            $els = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
                                [System.Windows.Automation.Condition]::TrueCondition)
            $btns = @($els | Where-Object {
                $_.Current.ControlType.ProgrammaticName -eq 'ControlType.Button' -and
                $_.Current.Name -eq '启动场景' })
            $label = $els | Where-Object { $_.Current.Name -eq $sceneName } | Select-Object -First 1
            if ($label -and $btns.Count -gt 0) {
                $lx = $label.Current.BoundingRectangle.X
                $ly = $label.Current.BoundingRectangle.Y
                $best = $null; $bestD = 1e9
                foreach ($b in $btns) {
                    $dx = $b.Current.BoundingRectangle.X - $lx
                    $dy = $b.Current.BoundingRectangle.Y - $ly
                    if ($dx -lt 20 -or $dx -gt 340 -or $dy -lt 60 -or $dy -gt 440) { continue }
                    $d = [Math]::Sqrt($dx * $dx + $dy * $dy)
                    if ($d -lt $bestD) { $bestD = $d; $best = $b }
                }
                if ($best) { [void](Invoke-El $best); return $true }
            }
            Start-Sleep -Milliseconds 500
        }
        $nex = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
                            [System.Windows.Automation.Condition]::TrueCondition) |
               Where-Object {
                   $_.Current.ControlType.ProgrammaticName -eq 'ControlType.Button' -and
                   $_.Current.Name -eq 'Next page' } | Select-Object -First 1
        if ($nex) { [void](Invoke-El $nex); Start-Sleep -Milliseconds 1200 } else { break }
    }
    return $false
}

Say "=== fog run $Script  out=$Out ==="
Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Seconds 2

$cw = Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue |
      Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
if (-not $cw) {
    Say "启动客户端"
    Start-Process -FilePath $ClientExe
}
for ($i = 0; $i -lt 90; $i++) {
    Start-Sleep -Seconds 1
    $cw = Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue |
          Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
    if ($cw) { break }
}
if (-not $cw) { Say "!! 客户端没起来"; exit 1 }
Say "客户端就绪 pid=$($cw.Id)"

$win = [System.Windows.Automation.AutomationElement]::FromHandle($cw.MainWindowHandle)
$els = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
                    [System.Windows.Automation.Condition]::TrueCondition)
for ($k = 1; $k -le 18; $k++) {
    if ($els.Count -gt 5) { break }
    Start-Sleep -Seconds 5
    $els = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
                        [System.Windows.Automation.Condition]::TrueCondition)
}
Say "UI 元素数=$($els.Count)"

$ok = Launch-Scene '港口迷雾'
Say "launched=$ok"
if (-not $ok) { exit 2 }

$up = $false
for ($i = 0; $i -lt 120; $i++) {
    Start-Sleep -Seconds 1
    if (Get-NetTCPConnection -State Listen -LocalPort 9100 -ErrorAction SilentlyContinue) { $up = $true; break }
}
Say "9100_up=$up ($i s)"
if (-not $up) { exit 3 }
if ($StartWait -gt 0) { Start-Sleep -Seconds $StartWait }

$env:PYTHONIOENCODING = 'utf-8'
$stdout = Join-Path $Out 'stdout.log'
$argList = @((Join-Path $Fog $Script)) + ($PyArgs -split ' ' | Where-Object { $_ })
Say ("跑 " + ($argList -join ' '))
& $Py @argList 2>&1 | ForEach-Object { $_ | Out-File -FilePath $stdout -Encoding UTF8 -Append }
Say "=== 退出 code=$LASTEXITCODE ==="
Get-Process -Name SmartCar -ErrorAction SilentlyContinue | Stop-Process -Force
Say "DONE out=$Out"

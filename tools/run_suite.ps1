# -*- coding: utf-8 -*-
<#
.SYNOPSIS
    等场景下载完 → 重启客户端 → 依次跑泊车和直行 → 记录日志。

.EXAMPLE
    & .\tools\run_suite.ps1
#>
param(
    [int]$MaxWaitMinutes = 90
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Set-Location $root
$logDir = Join-Path $root "logs"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$py = Join-Path $root ".venv\Scripts\python.exe"
$env:PYTHONIOENCODING = "utf-8"

Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes

function Get-ClientWindow {
    $clientPid = (Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue | Select-Object -First 1).Id
    if (-not $clientPid) { return $null }
    $cond = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::ProcessIdProperty, $clientPid)
    $rootEl = [System.Windows.Automation.AutomationElement]::RootElement
    return $rootEl.FindAll([System.Windows.Automation.TreeScope]::Children, $cond) |
        Where-Object { $_.Current.Name -like "*客户端*" } | Select-Object -First 1
}

function Invoke-ByName($win, $type, $name) {
    $cond = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::ControlTypeProperty, $type)
    $els = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $cond)
    $t = $els | Where-Object { $_.Current.Name -eq $name } | Select-Object -First 1
    if (-not $t) { return $false }
    $p = $null
    if ($t.TryGetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern, [ref]$p)) { $p.Invoke(); return $true }
    return $false
}

function Clean-Scene {
    Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force
    Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -like "*main.py*" -or $_.CommandLine -like "*park_v4*" } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
    Start-Sleep -Seconds 2
}

Write-Host "等待 download_scenes.py 结束（最多 $MaxWaitMinutes 分钟）..."
$deadline = (Get-Date).AddMinutes($MaxWaitMinutes)
while ((Get-Date) -lt $deadline) {
    $dl = Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -like "*download_scenes.py*" }
    if (-not $dl) { break }
    Start-Sleep -Seconds 20
}
Write-Host "下载阶段结束。"

# 重启客户端，让它加载新场景清单
Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Seconds 2
Start-Process "D:\Program Files\元动力客户端\metapower_desktop.exe"
Start-Sleep -Seconds 12
Write-Host "客户端已重启。"

$stamp = Get-Date -Format "MMdd_HHmm"
$report = Join-Path $logDir "suite_$stamp.txt"
"=== 跑分记录 $stamp ===" | Set-Content $report -Encoding UTF8

# ---------- 泊车 ----------
Write-Host "`n[1/2] 泊车"
Clean-Scene
$parkLog = Join-Path $logDir "suite_park_$stamp.log"
$p1 = Start-Process -FilePath $py -ArgumentList @("-u", "tools\park_v4.py") `
    -RedirectStandardOutput $parkLog -RedirectStandardError "$parkLog.err" -PassThru -WindowStyle Hidden
Start-Sleep -Seconds 5

$win = Get-ClientWindow
Invoke-ByName $win ([System.Windows.Automation.ControlType]::Button) "Go to page 2" | Out-Null
Start-Sleep -Seconds 2
$ok = Invoke-ByName $win ([System.Windows.Automation.ControlType]::Button) "启动场景"
Write-Host "  启动泊车场景: $ok"
Start-Sleep -Seconds 45
if (-not $p1.HasExited) { Stop-Process -Id $p1.Id -Force -ErrorAction SilentlyContinue }
Clean-Scene
"--- 泊车日志尾 ---" | Add-Content $report -Encoding UTF8
Get-Content $parkLog -ErrorAction SilentlyContinue | Select-Object -Last 12 | Add-Content $report -Encoding UTF8

# ---------- 直行 ----------
Write-Host "`n[2/2] 直行"
Clean-Scene
$straightLog = Join-Path $logDir "suite_straight_$stamp.log"
$p2 = Start-Process -FilePath $py -ArgumentList @("-u", "main.py", "--track", "--cruise", "0.5", "--seconds", "40") `
    -RedirectStandardOutput $straightLog -RedirectStandardError "$straightLog.err" -PassThru -WindowStyle Hidden
Start-Sleep -Seconds 5

$win = Get-ClientWindow
Invoke-ByName $win ([System.Windows.Automation.ControlType]::Button) "Go to page 1" | Out-Null
Start-Sleep -Seconds 2
$ok2 = Invoke-ByName $win ([System.Windows.Automation.ControlType]::Button) "启动场景"
Write-Host "  启动直行场景: $ok2"
Start-Sleep -Seconds 30
if (-not $p2.HasExited) { Stop-Process -Id $p2.Id -Force -ErrorAction SilentlyContinue }
Clean-Scene

"--- 直行日志尾 ---" | Add-Content $report -Encoding UTF8
Get-Content $straightLog -ErrorAction SilentlyContinue | Select-Object -Last 8 | Add-Content $report -Encoding UTF8

Write-Host "`n完成，报告：$report"
Get-Content $report | Select-Object -Last 30

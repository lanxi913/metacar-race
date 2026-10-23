# -*- coding: utf-8 -*-
<#
.SYNOPSIS
    跑一次「车辆直行」，返回耗时和最后的日志尾巴。

.EXAMPLE
    & .\tools\one_run.ps1 -PythonArgs "main.py --track --cruise 0.5"
#>
param(
    [string]$PythonArgs = "main.py --track --cruise 0.5",
    [int]$BootSeconds = 4,
    [int]$RunSeconds = 16
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Set-Location $root
$py = Join-Path $root ".venv\Scripts\python.exe"
$logDir = Join-Path $root "logs"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$stamp = Get-Date -Format "HHmmss"
$log = Join-Path $logDir "one_$stamp.log"

Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes

# 1) 清场
Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force
Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
    Where-Object { $_.CommandLine -like "*main.py*" } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
Start-Sleep -Seconds 2

# 2) 起脚本
$env:PYTHONIOENCODING = "utf-8"
$proc = Start-Process -FilePath $py -ArgumentList (@("-u") + ($PythonArgs -split ' ')) `
    -RedirectStandardOutput $log -RedirectStandardError "$log.err" -PassThru -WindowStyle Hidden
Write-Host "python pid=$($proc.Id) log=$log"

# 3) 起场景
Start-Sleep -Seconds $BootSeconds
$clientPid = (Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue | Select-Object -First 1).Id
if (-not $clientPid) { throw "客户端没运行" }
$cond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ProcessIdProperty, $clientPid)
$rootEl = [System.Windows.Automation.AutomationElement]::RootElement
$win = $rootEl.FindAll([System.Windows.Automation.TreeScope]::Children, $cond) |
    Where-Object { $_.Current.Name -like "*客户端*" } | Select-Object -First 1

$hc = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty, [System.Windows.Automation.ControlType]::Hyperlink)
($win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $hc) |
    Where-Object { $_.Current.Name -eq "自动驾驶" } | Select-Object -First 1) | ForEach-Object {
    $p = $null; if ($_.TryGetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern, [ref]$p)) { $p.Invoke() }
}
Start-Sleep -Milliseconds 1500

$bc = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty, [System.Windows.Automation.ControlType]::Button)
$start = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $bc) |
    Where-Object { $_.Current.Name -eq "启动场景" } | Select-Object -First 1
if ($start) {
    $p = $null
    if ($start.TryGetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern, [ref]$p)) { $p.Invoke(); Write-Host "scene launched" }
} else {
    Write-Host "!! 没找到启动场景按钮"
}

# 4) 等这一次跑完
Start-Sleep -Seconds $RunSeconds
if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue }
Start-Sleep -Seconds 1
Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force

Write-Host "--- 日志尾 ---"
Get-Content $log -ErrorAction SilentlyContinue | Select-Object -Last 4
Get-Content "$log.err" -ErrorAction SilentlyContinue | Select-Object -Last 3

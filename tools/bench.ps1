# -*- coding: utf-8 -*-
<#
.SYNOPSIS
    批量跑「车辆直行」，自动从平台训练记录里读分数。

.DESCRIPTION
    每次循环：清场 -> 起 Python 脚本 -> UIA 点启动场景 -> 等收货 -> 读记录页 -> 记一行。
    跑完把结果打到屏幕，同时写 logs/bench.csv。

.EXAMPLE
    pwsh -File tools\bench.ps1 -Runs 5
#>
param(
    [int]$Runs = 5,
    [string]$PythonArgs = "main.py --track --cruise 0.5",
    [int]$BootSeconds = 4,
    [int]$RunSeconds = 15
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Set-Location $root

$py = Join-Path $root ".venv\Scripts\python.exe"
if (-not (Test-Path $py)) { throw "找不到虚拟环境解释器: $py" }
Write-Host "python : $py"
Write-Host "args   : $PythonArgs"
$logDir = Join-Path $root "logs"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$csv = Join-Path $logDir "bench.csv"
if (-not (Test-Path $csv)) { "run,started,ended,score,log" | Set-Content $csv -Encoding UTF8 }

Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes

function Get-ClientWindow {
    $clientPid = (Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue | Select-Object -First 1).Id
    if (-not $clientPid) { throw "客户端没运行：先打开元动力客户端" }
    $cond = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::ProcessIdProperty, $clientPid)
    $rootEl = [System.Windows.Automation.AutomationElement]::RootElement
    $wins = $rootEl.FindAll([System.Windows.Automation.TreeScope]::Children, $cond)
    return $wins | Where-Object { $_.Current.Name -like "*客户端*" } | Select-Object -First 1
}

function Invoke-ByName($win, $controlType, $name) {
    $cond = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::ControlTypeProperty, $controlType)
    $els = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $cond)
    $target = $els | Where-Object { $_.Current.Name -eq $name } | Select-Object -First 1
    if (-not $target) { return $false }
    $pat = $null
    if ($target.TryGetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern, [ref]$pat)) {
        $pat.Invoke()
        return $true
    }
    return $false
}

function Stop-Scene {
    Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force
    # 清掉上一次残留的 main.py，否则 5061/5063 被占用，新脚本起不来
    Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -like "*main.py*" } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
    Start-Sleep -Seconds 1
}

function Read-Scores {
    # 从 Edge 的训练记录页读表格
    $rootEl = [System.Windows.Automation.AutomationElement]::RootElement
    $wins = $rootEl.FindAll([System.Windows.Automation.TreeScope]::Children,
        [System.Windows.Automation.Condition]::TrueCondition)
    $win = $wins | Where-Object { $_.Current.Name -like "元动力训练营*" } | Select-Object -First 1
    if (-not $win) { throw "没找到训练记录页面：先在 Edge 打开 https://www.i-mylab.com/icv/history.html?id=straight" }

    Invoke-ByName $win ([System.Windows.Automation.ControlType]::Button) "刷新" | Out-Null
    Start-Sleep -Seconds 3

    $all = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.Condition]::TrueCondition)
    $texts = @()
    foreach ($e in $all) {
        $n = $e.Current.Name
        if ($n -and $n.Trim()) { $texts += $n.Trim() }
    }
    $idx = [array]::IndexOf($texts, "得分")
    if ($idx -lt 0) { throw "记录表没加载出来" }

    $rows = @()
    $i = $idx + 1
    while ($i + 3 -lt $texts.Count) {
        if ($texts[$i] -ne "车辆直行") { break }
        $rows += [pscustomobject]@{
            name  = $texts[$i]
            start = $texts[$i + 1]
            end   = $texts[$i + 2]
            score = [double]$texts[$i + 3]
        }
        $i += 4
    }
    return $rows
}

$before = Read-Scores
$known = ($before | ForEach-Object { $_.start }) -join "|"

for ($i = 1; $i -le $Runs; $i++) {
    Write-Host "=== run $i/$Runs ==="
    Stop-Scene

    $log = Join-Path $logDir ("run_{0:00}.log" -f $i)
    $env:PYTHONIOENCODING = "utf-8"
    $proc = Start-Process -FilePath $py -ArgumentList ($PythonArgs -split ' ') `
        -RedirectStandardOutput $log -RedirectStandardError "$log.err" -PassThru -WindowStyle Hidden

    Start-Sleep -Seconds $BootSeconds

    $win = Get-ClientWindow
    Invoke-ByName $win ([System.Windows.Automation.ControlType]::Hyperlink) "自动驾驶" | Out-Null
    Start-Sleep -Milliseconds 1500
    $ok = Invoke-ByName $win ([System.Windows.Automation.ControlType]::Button) "启动场景"
    if (-not $ok) { Write-Host "  启动场景按钮没点到，跳过这次"; continue }

    Start-Sleep -Seconds $RunSeconds
    if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -Force }
    Stop-Scene

    Start-Sleep -Seconds 2
    $rows = Read-Scores
    $new = $rows | Where-Object { $known -notlike "*$($_.start)*" } | Select-Object -First 1
    if ($new) {
        Write-Host ("  {0} -> {1} 分" -f $new.end, $new.score)
        "{0},{1},{2},{3},{4}" -f $i, $new.start, $new.end, $new.score, $log | Add-Content $csv -Encoding UTF8
        $known = $known + "|" + $new.start
    } else {
        Write-Host "  这次没读到新记录"
        "{0},,,-1,{1}" -f $i, $log | Add-Content $csv -Encoding UTF8
    }
}

Write-Host "`n=== 汇总 ==="
$rows = Read-Scores
$rows | Select-Object -First 10 | Format-Table name, start, score -AutoSize
Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force




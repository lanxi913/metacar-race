# -*- coding: utf-8 -*-
<#
.SYNOPSIS
    Start a python control script, then click the "start scene" button of a specific
    scenario row in the client UI, then wait for the python process to exit.

.DESCRIPTION
    ASCII-only on purpose: Windows PowerShell 5.1 parses .ps1 as ANSI without a BOM,
    so literal CJK here would be mangled. UI strings are built from code points.

    -SceneCp receives the scenario name as comma separated hex code points,
    e.g. 平行车道泊车 = 5E73,884C,8F66,9053,6CCA,8F66

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File tools\auto_scene.ps1 -Script probe.py -SceneCp "5E73,884C,8F66,9053,6CCA,8F66"
#>
param(
    [string]$PythonArgs = "",
    [string]$Script = "straight_full\main.py",
    [string]$SceneCp = "",
    [int]$BootSeconds = 3,
    [int]$TimeoutSeconds = 140,
    [switch]$NoLaunch,
    [switch]$SkipAuto
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Set-Location $root
$py = Join-Path $root ".venv\Scripts\python.exe"
$logDir = Join-Path $root "logs"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$stamp = Get-Date -Format "HHmmss"
$log = Join-Path $logDir "auto_$stamp.log"

$U_CLIENT = -join ([char]0x5BA2, [char]0x6237, [char]0x7AEF)
$U_AUTO   = -join ([char]0x81EA, [char]0x52A8, [char]0x9A7E, [char]0x9A76)
$U_START  = -join ([char]0x542F, [char]0x52A8, [char]0x573A, [char]0x666F)

Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes
Add-Type @"
using System;
using System.Runtime.InteropServices;
public class Win {
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
}
"@

$sceneName = ""
if ($SceneCp) {
    $sceneName = -join ($SceneCp.Split(',') | ForEach-Object { [char][Convert]::ToInt32($_.Trim(), 16) })
}

function Stop-Leftovers {
    Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
    Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -like "*straight_full*" -or $_.CommandLine -like "*main.py*" -or $_.CommandLine -like "*probe*" -or $_.CommandLine -like "*parking*" } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
    Start-Sleep -Seconds 1
}

function Get-ClientWindow {
    $cp = (Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue | Select-Object -First 1).Id
    if (-not $cp) { throw "client not running" }
    $cond = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::ProcessIdProperty, $cp)
    $rootEl = [System.Windows.Automation.AutomationElement]::RootElement
    return $rootEl.FindAll([System.Windows.Automation.TreeScope]::Children, $cond) |
        Where-Object { $_.Current.Name -like "*$U_CLIENT*" } | Select-Object -First 1
}

function Invoke-El($el) {
    if (-not $el) { return $false }
    $p = $null
    if ($el.TryGetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern, [ref]$p)) {
        $p.Invoke(); return $true
    }
    return $false
}

Stop-Leftovers

$proc = $null
if (-not $NoLaunch) {
    $pyArgs = @("-u", $Script)
    if ($PythonArgs.Trim()) { $pyArgs += ($PythonArgs.Trim() -split '\s+') }
    $env:PYTHONIOENCODING = "utf-8"
    $env:PYTHONUTF8 = "1"
    $proc = Start-Process -FilePath $py -ArgumentList $pyArgs `
        -RedirectStandardOutput $log -RedirectStandardError "$log.err" -PassThru -WindowStyle Hidden
    Write-Host "python pid=$($proc.Id) log=$log"
    Start-Sleep -Seconds $BootSeconds
}

$win = Get-ClientWindow
[Win]::ShowWindow([IntPtr]$win.Current.NativeWindowHandle, 9) | Out-Null
[Win]::SetForegroundWindow([IntPtr]$win.Current.NativeWindowHandle) | Out-Null
Start-Sleep -Milliseconds 800
Write-Host "client: $($win.Current.Name)"

$all = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, [System.Windows.Automation.Condition]::TrueCondition)

if (-not $SkipAuto) {
    $auto = $all | Where-Object { $_.Current.Name -eq $U_AUTO } | Select-Object -First 1
    Invoke-El $auto | Out-Null
    Start-Sleep -Milliseconds 1500
}

# 页面是 virtual list，切页后元素会重建，这里重试等它稳定
$target = $null
for ($page = 0; $page -le 4 -and -not $target; $page++) {
    for ($try = 1; $try -le 6; $try++) {
        $all = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, [System.Windows.Automation.Condition]::TrueCondition)
        $btns = @($all | Where-Object { $_.Current.ControlType.ProgrammaticName -eq 'ControlType.Button' -and $_.Current.Name -eq $U_START })

        if (-not $sceneName) {
            if ($btns.Count -gt 0) { $target = $btns[0]; break }
        } else {
            $label = $all | Where-Object { $_.Current.Name -eq $sceneName } | Select-Object -First 1
            if ($label) {
            # 卡片里标题在上、按钮在下，不能按行高匹配；取欧氏距离最近的启动按钮
            $lx = $label.Current.BoundingRectangle.X
            $ly = $label.Current.BoundingRectangle.Y
            $best = $null; $bestD = 1e9
            foreach ($b in $btns) {
                $bx = $b.Current.BoundingRectangle.X
                $by = $b.Current.BoundingRectangle.Y
                $d = [Math]::Sqrt([Math]::Pow($bx - $lx, 2) + [Math]::Pow($by - $ly, 2))
                if ($d -lt $bestD) { $bestD = $d; $best = $b }
            }
            if ($best -and $bestD -lt 400) {
                $target = $best
                Write-Host ("found row: label({0},{1}) -> button({2},{3}) d={4:N0} on try {5}" -f `
                    [int]$lx, [int]$ly, [int]$best.Current.BoundingRectangle.X, [int]$best.Current.BoundingRectangle.Y, $bestD, $try)
                break
            }
        }
    }
    Start-Sleep -Milliseconds 700
    }

    # not on this page -> go to next page and retry
    if (-not $target -and $sceneName) {
        $next = $all | Where-Object {
            $_.Current.ControlType.ProgrammaticName -eq 'ControlType.Button' -and $_.Current.Name -eq 'Next page'
        } | Select-Object -First 1
        if ($next) {
            Write-Host "page $page : not found, going next"
            Invoke-El $next | Out-Null
            Start-Sleep -Milliseconds 1200
        } else {
            Write-Host "page $page : not found, no next page"
            break
        }
    }
}

if (-not $target -and $sceneName) {
    $names = @($all | ForEach-Object { $_.Current.Name } | Where-Object { $_ })
    Write-Host "!! scene label not found: $sceneName (page has $($names.Count) named elements)"
}

$ok = Invoke-El $target
Write-Host "scene launch: $ok"

if ($proc) {
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while (-not $proc.HasExited -and $sw.Elapsed.TotalSeconds -lt $TimeoutSeconds) {
        Start-Sleep -Milliseconds 500
    }
    if (-not $proc.HasExited) {
        Write-Host "!! timeout, killing"
        Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
    }
    Start-Sleep -Milliseconds 500
    Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
    Write-Host "--- log tail ---"
    Get-Content $log -ErrorAction SilentlyContinue | Select-Object -Last 15
    if (Test-Path "$log.err") {
        $e = Get-Content "$log.err" -ErrorAction SilentlyContinue | Select-Object -Last 5
        if ($e) { Write-Host "--- stderr ---"; $e }
    }
}

# -*- coding: utf-8 -*-
<#
.SYNOPSIS
    Run one "straight" scenario automatically and print the log tail + SUMMARY.

.DESCRIPTION
    1) kill leftover python / SmartCar
    2) start the python script (stdout redirected to logs\)
    3) UI Automation: click "autopilot" hyperlink then "start scene" button
    4) wait until the script exits or timeout
    5) print the tail and the SUMMARY line

    NOTE: keep this file ASCII-only. Windows PowerShell 5.1 reads .ps1 as ANSI
    unless there is a BOM, and the edit tool writes UTF-8 without BOM, so any
    literal CJK here gets mangled and breaks parsing. CJK UI names are built
    from code points below instead.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File tools\auto_straight.ps1
    powershell -ExecutionPolicy Bypass -File tools\auto_straight.ps1 -PythonArgs "--speed 8 --no-kick"
#>
param(
    [string]$PythonArgs = "",
    [string]$Script = "straight_full\main.py",
    [int]$BootSeconds = 3,
    [int]$TimeoutSeconds = 140
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Set-Location $root
$py = Join-Path $root ".venv\Scripts\python.exe"
$logDir = Join-Path $root "logs"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$stamp = Get-Date -Format "HHmmss"
$log = Join-Path $logDir "auto_$stamp.log"

# UI strings, built from code points to stay ASCII-safe in this file
$U_CLIENT = -join ([char]0x5BA2, [char]0x6237, [char]0x7AEF)          # ke hu duan
$U_AUTO   = -join ([char]0x81EA, [char]0x52A8, [char]0x9A7E, [char]0x9A76)  # zi dong jia shi
$U_START  = -join ([char]0x542F, [char]0x52A8, [char]0x573A, [char]0x666F)  # qi dong chang jing

Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes

function Stop-Leftovers {
    Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
    Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -like "*straight_full*" -or $_.CommandLine -like "*main.py*" } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
    Start-Sleep -Seconds 1
}

function Get-ClientWindow {
    $cp = (Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue | Select-Object -First 1).Id
    if (-not $cp) { throw "client not running (metapower_desktop)" }
    $cond = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::ProcessIdProperty, $cp)
    $rootEl = [System.Windows.Automation.AutomationElement]::RootElement
    return $rootEl.FindAll([System.Windows.Automation.TreeScope]::Children, $cond) |
        Where-Object { $_.Current.Name -like "*$U_CLIENT*" } | Select-Object -First 1
}

function Invoke-Element($win, $controlType, $name) {
    if (-not $win) { return $false }
    $cond = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::ControlTypeProperty, $controlType)
    $els = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $cond)
    $target = $els | Where-Object { $_.Current.Name -eq $name } | Select-Object -First 1
    if (-not $target) { Write-Host "    (not found: $name)"; return $false }
    $pat = $null
    if ($target.TryGetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern, [ref]$pat)) {
        $pat.Invoke()
        return $true
    }
    Write-Host "    (no InvokePattern: $name)"
    return $false
}

Stop-Leftovers

$pyArgs = @("-u", $Script)
if ($PythonArgs.Trim()) { $pyArgs += ($PythonArgs.Trim() -split '\s+') }
$env:PYTHONIOENCODING = "utf-8"
$env:PYTHONUTF8 = "1"

$proc = Start-Process -FilePath $py -ArgumentList $pyArgs `
    -RedirectStandardOutput $log -RedirectStandardError "$log.err" -PassThru -WindowStyle Hidden
Write-Host "python pid=$($proc.Id)  log=$log"

Start-Sleep -Seconds $BootSeconds

$win = Get-ClientWindow
Write-Host "client window: $($win.Current.Name)"
Invoke-Element $win ([System.Windows.Automation.ControlType]::Hyperlink) $U_AUTO | Out-Null
Start-Sleep -Milliseconds 1200
$ok = Invoke-Element $win ([System.Windows.Automation.ControlType]::Button) $U_START
Write-Host "scene launch: $ok"

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
Get-Content $log -ErrorAction SilentlyContinue | Select-Object -Last 12
if (Test-Path "$log.err") {
    $errTail = Get-Content "$log.err" -ErrorAction SilentlyContinue | Select-Object -Last 5
    if ($errTail) { Write-Host "--- stderr ---"; $errTail }
}

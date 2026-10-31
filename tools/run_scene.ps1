# -*- coding: utf-8 -*-
<#
.SYNOPSIS
    One-shot scene runner: kill stale scene -> start controller -> click "启动场景" -> wait -> report.

.DESCRIPTION
    Hard order for single scenes: the controller script is the TCP *server* (127.0.0.1:5061),
    so it must be started BEFORE the Unity scene, otherwise the client hangs on "loading".

    Scene titles are Chinese; this file stays ASCII-only and reads them from tools/scenes.json
    (UTF-8) so the ps1 itself never depends on console code page.

.EXAMPLE
    .\tools\run_scene.ps1 -Slug straight -PythonArgs "main.py --track --cruise 0.5 --seconds 60"
#>
param(
    [Parameter(Mandatory = $true)][string]$Slug,
    [string]$PythonArgs = "",
    [int]$MaxSeconds = 240,
    [int]$BootSeconds = 6,
    [int]$ConnectTimeoutSeconds = 90,
    [switch]$KeepScene
)

$ErrorActionPreference = 'Continue'
Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes

$root = Split-Path -Parent $PSScriptRoot
$py = Join-Path $root '.venv\Scripts\python.exe'
$sceneMap = Get-Content -Path (Join-Path $PSScriptRoot 'scenes.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$launchButton = $sceneMap._meta.launch_button     # Chinese label kept out of this ASCII file
$currentPageGlob = $sceneMap._meta.current_page
if (-not $sceneMap.PSObject.Properties.Name.Contains($Slug)) { Write-Host "!! unknown slug $Slug"; exit 2 }
$title = $sceneMap.$Slug.title
$page = [int]$sceneMap.$Slug.page

$logDir = Join-Path $root 'logs'
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$stamp = Get-Date -Format 'MMdd_HHmmss'
$log = Join-Path $logDir ("run_{0}_{1}.log" -f $Slug, $stamp)
$errlog = "$log.err"

function Kill-Stale {
    Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
    foreach ($port in 5061, 5063) {
        Get-NetTCPConnection -State Listen -LocalPort $port -ErrorAction SilentlyContinue |
            ForEach-Object { Stop-Process -Id $_.OwningProcess -Force -ErrorAction SilentlyContinue }
    }
    Start-Sleep -Seconds 2
}

function Get-ClientWindow {
    $cw = Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue |
          Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
    if (-not $cw) { return $null }
    return [System.Windows.Automation.AutomationElement]::FromHandle($cw.MainWindowHandle)
}

function Invoke-El($el) {
    try {
        $p = $el.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern)
        $p.Invoke()
        return $true
    } catch {
        Write-Host ("   invoke exception: " + $_.Exception.GetType().Name + " " + $_.Exception.Message)
        return $false
    }
}

function Goto-Page($win, [int]$n) {
    $btns = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.Condition]::TrueCondition) |
        Where-Object { $_.Current.ControlType.ProgrammaticName -eq 'ControlType.Button' }
    $cur = $btns | Where-Object { $_.Current.Name -like $currentPageGlob } | Select-Object -First 1
    if ($cur) {
        $curNum = ($cur.Current.Name -replace '[^0-9]', '')
        if ($curNum -eq "$n") { return $true }
    }
    $b = $btns | Where-Object { $_.Current.Name -eq "Go to page $n" } | Select-Object -First 1
    if (-not $b) { return $false }
    [void](Invoke-El $b)
    Start-Sleep -Milliseconds 2500
    return $true
}

function Launch-Scene($win, [string]$sceneTitle, [int]$pageNo) {
    $okPage = Goto-Page $win $pageNo
    Write-Host ("   goto page {0} -> {1}" -f $pageNo, $okPage)
    for ($try = 1; $try -le 8; $try++) {
        $els = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
            [System.Windows.Automation.Condition]::TrueCondition)
        $btns = @($els | Where-Object {
            $_.Current.ControlType.ProgrammaticName -eq 'ControlType.Button' -and $_.Current.Name -eq $launchButton })
        $label = $els | Where-Object { $_.Current.Name -eq $sceneTitle -and $_.Current.BoundingRectangle.Width -gt 0 } |
                 Select-Object -First 1
        Write-Host ("   try {0}: els={1} btns={2} label={3}" -f $try, $els.Count, $btns.Count, [bool]$label)
        if ($label -and $btns.Count -gt 0) {
            $lx = $label.Current.BoundingRectangle.X
            $ly = $label.Current.BoundingRectangle.Y
            $best = $null; $bestD = 1e9
            foreach ($b in $btns) {
                $dx = $b.Current.BoundingRectangle.X - $lx
                $dy = $b.Current.BoundingRectangle.Y - $ly
                if ($dx -lt -20 -or $dx -gt 360 -or $dy -lt 40 -or $dy -gt 470) { continue }
                $d = [Math]::Sqrt($dx * $dx + $dy * $dy)
                if ($d -lt $bestD) { $bestD = $d; $best = $b }
            }
            Write-Host ("     dx/dy best d={0}" -f $bestD)
            if ($best) { if (Invoke-El $best) { return $true } }
        }
        Start-Sleep -Milliseconds 700
    }
    return $false
}

Write-Host "=== run $Slug ($title) page $page ==="
Write-Host "args: $PythonArgs"
Write-Host "log:  $log"

Kill-Stale

$argList = @('-u') + ($PythonArgs -split '\s+' | Where-Object { $_ -ne '' })
$env:PYTHONIOENCODING = 'utf-8'
$proc = Start-Process -FilePath $py -ArgumentList $argList -WorkingDirectory $root `
    -RedirectStandardOutput $log -RedirectStandardError $errlog -PassThru -WindowStyle Hidden
Write-Host "controller pid=$($proc.Id)"

Start-Sleep -Seconds $BootSeconds

$win = Get-ClientWindow
if (-not $win) { Write-Host "!! client window not found"; if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -Force }; exit 3 }

if (-not (Launch-Scene $win $title $page)) {
    Write-Host "!! could not click launch for $title"
    if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -Force }
    exit 4
}
Write-Host "clicked launch"

# wait for the controller to report a connection
$connected = $false
for ($i = 0; $i -lt $ConnectTimeoutSeconds; $i++) {
    Start-Sleep -Seconds 1
    if ($proc.HasExited) { break }
    $est = Get-NetTCPConnection -State Established -LocalPort 5061 -ErrorAction SilentlyContinue
    if ($est) { $connected = $true; break }
}
if ($connected) { Write-Host "controller connected" } else { Write-Host "!! no connection detected" }

$t0 = Get-Date
while (-not $proc.HasExited) {
    Start-Sleep -Seconds 2
    if (((Get-Date) - $t0).TotalSeconds -gt $MaxSeconds) {
        Write-Host "!! controller still running after $MaxSeconds s, killing"
        Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
        break
    }
}
$code = if ($proc.HasExited) { $proc.ExitCode } else { -1 }
Write-Host "controller exit code=$code"

if (-not $KeepScene) {
    Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
}

Write-Host "--- log tail ---"
Get-Content -Path $log -Encoding UTF8 -ErrorAction SilentlyContinue | Select-Object -Last 12
$e = Get-Content -Path $errlog -Encoding UTF8 -ErrorAction SilentlyContinue | Select-Object -Last 6
if ($e) { Write-Host "--- stderr ---"; $e }

Write-Host ("RESULT slug={0} connected={1} exit={2} log={3}" -f $Slug, $connected, $code, $log)
exit 0

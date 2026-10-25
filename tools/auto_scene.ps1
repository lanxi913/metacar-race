# -*- coding: utf-8 -*-
<#
.SYNOPSIS
    Launch a python script, then click the "start scene" button of one specific
    scenario card in the meta-power client, then wait for the python process.

.DESCRIPTION
    ASCII-only on purpose: Windows PowerShell 5.1 parses .ps1 as ANSI when there is
    no BOM, and the editor writes UTF-8 without BOM. Literal CJK here gets mangled
    and can even swallow the next line of code, so all UI strings are built from
    code points.

    -SceneCp takes the scenario name as comma separated hex code points:
        ?????? = 5E73,884C,8F66,9053,6CCA,8F66
        ????     = 9650,901F,884C,9A76
        ????     = 76F4,89D2,8F6C,5F2F
        ????     = 8F66,8F86,76F4,884C
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
    # A killed run leaves TWO python processes behind (the .venv shim plus the real
    # interpreter it spawns), and the survivor keeps listening on 5061/5063 - the
    # next run's script then blocks in connect() forever while the scene talks to
    # the zombie. So kill: the scene process, every python whose command line runs
    # this repo's scripts, and whoever owns the SDK ports.
    Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
    $self = $PID
    Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.ProcessId -ne $self -and $_.CommandLine -and
                       ($_.CommandLine -match 'main\.py|probe|park_cs4|\.py"|\.py -u| -u [\w\\]+\.py') } |
        ForEach-Object {
            Write-Host "stale python $($_.ProcessId) killed"
            Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
        }
    foreach ($port in 5061, 5063) {
        Get-NetTCPConnection -LocalPort $port -State Listen -ErrorAction SilentlyContinue | ForEach-Object {
            $owner = $_.OwningProcess
            if ($owner -and $owner -ne $self) {
                Write-Host "port $port held by $owner, killed"
                Stop-Process -Id $owner -Force -ErrorAction SilentlyContinue
            }
        }
    }
    Start-Sleep -Seconds 1
}

function Get-ClientWindow {
    # ??????? UIA ?????????/????????? MainWindowHandle ????
    # ????? restore + ????? UIA ??? 0 ??????
    for ($i = 1; $i -le 20; $i++) {
        $p = Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue |
             Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
        if ($p) {
            $h = [IntPtr]$p.MainWindowHandle
            [void][Win]::ShowWindow($h, 9)
            [void][Win]::SetForegroundWindow($h)
            Start-Sleep -Milliseconds 1200
            $el = [System.Windows.Automation.AutomationElement]::FromHandle($h)
            $kids = $el.FindAll([System.Windows.Automation.TreeScope]::Descendants,
                                [System.Windows.Automation.Condition]::TrueCondition)
            if ($kids.Count -gt 5) { return $el }
        }
        Start-Sleep -Seconds 1
    }
    throw "client window not ready"
}

if (-not ('AutoSceneMouse' -as [type])) {
    Add-Type -Namespace AutoScene -Name SceneMouse -MemberDefinition @'
[DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
[DllImport("user32.dll")] public static extern void mouse_event(uint dwFlags, uint dx, uint dy, uint dwData, System.IntPtr dwExtraInfo);
'@
}

function Click-Point([double]$x, [double]$y) {
    [AutoScene.SceneMouse]::SetCursorPos([int]$x, [int]$y) | Out-Null
    Start-Sleep -Milliseconds 80
    [AutoScene.SceneMouse]::mouse_event(0x0002, 0, 0, 0, [System.IntPtr]::Zero)
    Start-Sleep -Milliseconds 50
    [AutoScene.SceneMouse]::mouse_event(0x0004, 0, 0, 0, [System.IntPtr]::Zero)
    Start-Sleep -Milliseconds 120
}

function Invoke-El($el) {
    if (-not $el) { return $false }
    $p = $null
    try {
        if ($el.TryGetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern, [ref]$p)) {
            $p.Invoke(); return $true
        }
    } catch {
        # ElementNotEnabled / "unknown error" happens while the list is repainting;
        # fall back to a real mouse click on the element centre.
        Write-Host ("invoke failed ({0}), clicking by mouse" -f $_.Exception.GetType().Name)
    }
    try {
        $r = $el.Current.BoundingRectangle
        if ($r.Width -gt 1 -and $r.Height -gt 1) {
            Click-Point ($r.X + $r.Width / 2.0) ($r.Y + $r.Height / 2.0)
            return $true
        }
    } catch { }
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

# The client sometimes ends up with no visible main window (tray / crashed UI).
# Start it when we cannot find one, and give it time to come up.
$clientExe = 'D:\' + (-join ([char]0x65E0,[char]0x4EBA,[char]0x7B97,[char]0x6CD5)) + '\' + (-join ([char]0x5143,[char]0x52A8,[char]0x529B,[char]0x5BA2,[char]0x6237,[char]0x7AEF)) + '\metapower_desktop.exe'

function Start-Client {
    if (Test-Path $clientExe) {
        Start-Process -FilePath $clientExe -WorkingDirectory (Split-Path $clientExe)
        Start-Sleep -Seconds 18
    } else {
        Write-Host "!! client exe not found: $clientExe"
    }
}

function Test-ClientVisible {
    # Only decide whether the client PROCESS is running. Whether its window is
    # currently minimised / behind Edge does not matter: Get-ClientWindow already
    # restores the window by MainWindowHandle. Checking the UIA rectangle here used
    # to return false for a perfectly healthy client, which made this script spawn a
    # second instance and then fail with "client window not ready".
    $cpids = @(Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id)
    return ($cpids.Count -gt 0)
}

if (-not (Test-ClientVisible)) {
    Write-Host "client window missing, starting the client..."
    Start-Client
}

# The client sometimes collapses into its 14x14 tray window after a scene run: the
# process is alive with a MainWindowTitle, but the window has no UIA descendants.
# In that state every click is impossible, so kill and relaunch it.
$win = $null
try {
    $win = Get-ClientWindow
} catch {
    Write-Host "client window unusable, restarting the client..."
    Get-Process -Name metapower_desktop -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
    Start-Sleep -Seconds 3
    Start-Client
    $win = Get-ClientWindow
}
[Win]::ShowWindow([IntPtr]$win.Current.NativeWindowHandle, 9) | Out-Null
[Win]::SetForegroundWindow([IntPtr]$win.Current.NativeWindowHandle) | Out-Null
Start-Sleep -Milliseconds 800
Write-Host "client: $($win.Current.Name)"

if (-not $SkipAuto) {
    $all0 = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, [System.Windows.Automation.Condition]::TrueCondition)
    $auto = $all0 | Where-Object { $_.Current.Name -eq $U_AUTO } | Select-Object -First 1
    Invoke-El $auto | Out-Null
    Start-Sleep -Milliseconds 1500
}

# Walk pages until the scenario card is found. Card layout: the start button sits
# to the lower-right of the title (measured dx ~ +175, dy ~ +258), so candidates
# are filtered by that offset before picking the closest one.
#
# IMPORTANT: the list remembers the last visited page, so a later run can start on
# page 4 and "walk forward" only sees the tail pages ("not found" everywhere).
# Always jump back to page 1 first.
$target = $null
try {
    $allp = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, [System.Windows.Automation.Condition]::TrueCondition)
    $first = $allp | Where-Object {
        $_.Current.ControlType.ProgrammaticName -eq 'ControlType.Button' -and $_.Current.Name -eq 'Go to page 1'
    } | Select-Object -First 1
    if (-not $first) {
        $first = $allp | Where-Object {
            $_.Current.ControlType.ProgrammaticName -eq 'ControlType.ListItem' -and $_.Current.Name -eq '1'
        } | Select-Object -First 1
    }
    if ($first) {
        Invoke-El $first | Out-Null
        Start-Sleep -Milliseconds 1200
        Write-Host "pagination reset to page 1"
    }
} catch {
    Write-Host "pagination reset skipped: $($_.Exception.Message)"
}

for ($page = 0; $page -le 8 -and -not $target; $page++) {
    for ($try = 1; $try -le 6; $try++) {
        $all = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, [System.Windows.Automation.Condition]::TrueCondition)
        $btns = @($all | Where-Object { $_.Current.ControlType.ProgrammaticName -eq 'ControlType.Button' -and $_.Current.Name -eq $U_START })

        if (-not $sceneName) {
            if ($btns.Count -gt 0) { $target = $btns[0]; break }
        }
        else {
            $label = $all | Where-Object { $_.Current.Name -eq $sceneName } | Select-Object -First 1
            if ($label) {
                $lx = $label.Current.BoundingRectangle.X
                $ly = $label.Current.BoundingRectangle.Y
                $best = $null
                $bestD = 1e9
                foreach ($b in $btns) {
                    $dx = $b.Current.BoundingRectangle.X - $lx
                    $dy = $b.Current.BoundingRectangle.Y - $ly
                    if ($dx -lt 20) { continue }
                    if ($dx -gt 320) { continue }
                    if ($dy -lt 80) { continue }
                    if ($dy -gt 420) { continue }
                    $d = [Math]::Sqrt($dx * $dx + $dy * $dy)
                    if ($d -lt $bestD) { $bestD = $d; $best = $b }
                }
                if ($best) {
                    $target = $best
                    Write-Host ("found card: label({0},{1}) -> button({2},{3}) d={4:N0}" -f `
                        [int]$lx, [int]$ly, [int]$best.Current.BoundingRectangle.X, [int]$best.Current.BoundingRectangle.Y, $bestD)
                    break
                }
            }
        }
        Start-Sleep -Milliseconds 700
    }

    if (-not $target -and $sceneName) {
        # Prefer the explicit "Go to page N" button: it is a plain enabled button,
        # whereas "Next page" is briefly disabled while the list repaints (that is
        # what threw ElementNotEnabledException and aborted the whole launch).
        $wantNum = $page + 2
        $goto = $all | Where-Object {
            $_.Current.ControlType.ProgrammaticName -eq 'ControlType.Button' -and
            $_.Current.Name -eq ("Go to page {0}" -f $wantNum)
        } | Select-Object -First 1
        if ($goto) {
            Write-Host ("page {0} : not found, goto page {1}" -f $page, $wantNum)
            Invoke-El $goto | Out-Null
            Start-Sleep -Milliseconds 1200
        }
        else {
            $next = $all | Where-Object {
                $_.Current.ControlType.ProgrammaticName -eq 'ControlType.Button' -and $_.Current.Name -eq 'Next page'
            } | Select-Object -First 1
            if ($next) {
                Write-Host "page $page : not found, going next"
                Invoke-El $next | Out-Null
                Start-Sleep -Milliseconds 1200
            }
            else {
                Write-Host "page $page : not found, no next page"
                break
            }
        }
    }
}

if (-not $target) {
    Write-Host "!! target scene not found; nothing launched"
    if ($proc -and -not $proc.HasExited) { Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue }
    exit 2
}

$ok = Invoke-El $target
Write-Host "scene launch: $ok"

# The client sometimes swallows the first click (still busy closing the previous
# scene): SmartCar never spawns and the python script sits in connect() forever.
# Poll for the scene process and re-click a few times before giving up.
if ($proc) {
    for ($i = 1; $i -le 3; $i++) {
        Start-Sleep -Seconds 5
        if (Get-Process -Name SmartCar -ErrorAction SilentlyContinue) {
            Write-Host "scene process is up"
            break
        }
        Write-Host "scene process missing, re-clicking start ($i)"
        Invoke-El $target | Out-Null
    }
}

if ($proc) {
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while (-not $proc.HasExited -and $sw.Elapsed.TotalSeconds -lt $TimeoutSeconds) {
        Start-Sleep -Milliseconds 500
    }
    if (-not $proc.HasExited) {
        Write-Host "!! timeout, killing"
        Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
    }
    # Give the client time to upload the result before the scene process is killed.
    Write-Host "waiting 12s for the client to upload the result..."
    Start-Sleep -Seconds 12
    Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
    Write-Host "--- log tail ---"
    Get-Content $log -ErrorAction SilentlyContinue | Select-Object -Last 15
    if (Test-Path "$log.err") {
        $e = Get-Content "$log.err" -ErrorAction SilentlyContinue | Select-Object -Last 5
        if ($e) { Write-Host "--- stderr ---"; $e }
    }
}

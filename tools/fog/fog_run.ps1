param(
    [int]$Seconds = 620,
    [int]$Deliver = 1,
    [string]$Out = ""
)

$ErrorActionPreference = 'Continue'
Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes

$Root = 'C:\Users\lanxi\Documents\deepseek-harness\default-workspace\.train\fog800'
$Fog = Join-Path $Root 'tools\fog'
$Py = 'C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race\.venv\Scripts\python.exe'
$ClientExe = 'D:\Program Files\元动力客户端\metapower_desktop.exe'

$stamp = Get-Date -Format 'MMdd_HHmmss'
if (-not $Out) { $Out = Join-Path $Root ('logs\m_' + $stamp) }
New-Item -ItemType Directory -Force -Path $Out | Out-Null
$console = Join-Path $Out 'console.log'

function Say($m) {
    $line = "[{0}] {1}" -f (Get-Date -Format 'HH:mm:ss'), $m
    Write-Host $line          # 不能用 Write-Output：会把 Say 的返回值变成数组，
                              # 让 'if (Start-SceneOnce)' 恒为真（场景没起来也硬跑）
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

function Start-SceneOnce {
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
    if (-not $cw) { Say "!! 客户端没起来"; return $false }
    $win = [System.Windows.Automation.AutomationElement]::FromHandle($cw.MainWindowHandle)
    $els = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
                        [System.Windows.Automation.Condition]::TrueCondition)
    for ($k = 1; $k -le 18; $k++) {
        if ($els.Count -gt 5) { break }
        Start-Sleep -Seconds 5
        $els = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
                            [System.Windows.Automation.Condition]::TrueCondition)
    }
    if (-not (Launch-Scene '港口迷雾')) { Say "!! 点不到启动场景"; return $false }
    for ($i = 0; $i -lt 120; $i++) {
        Start-Sleep -Seconds 1
        if (Get-NetTCPConnection -State Listen -LocalPort 9100 -ErrorAction SilentlyContinue) {
            Say "9100_up ($i s)"
            return $true
        }
    }
    Say "!! 9100 没起来"
    return $false
}

Say "=== fog mission run $stamp  out=$Out ==="

$started = $false
for ($attempt = 1; $attempt -le 2; $attempt++) {
    Say ("起场景 第 {0} 次" -f $attempt)
    if (Start-SceneOnce) { $started = $true; break }
    Get-Process -Name metapower_desktop, SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue |
        Stop-Process -Force
    Start-Sleep -Seconds 8
}
if (-not $started) { Say "!! 两次都没起来，放弃这局"; exit 3 }

$env:PYTHONIOENCODING = 'utf-8'
$stdout = Join-Path $Out 'stdout.log'
Say "跑 fog_mission --seconds $Seconds"

$proc = Start-Process -FilePath $Py `
    -ArgumentList @((Join-Path $Fog 'fog_mission.py'), '--out', $Out,
                    '--seconds', $Seconds, '--cross', '1', '--deliver', $Deliver) `
    -WorkingDirectory $Fog -PassThru -NoNewWindow `
    -RedirectStandardOutput $stdout -RedirectStandardError (Join-Path $Out 'stderr.log')
$lastSize = -1
$lastChange = Get-Date
while (-not $proc.HasExited) {
    Start-Sleep -Seconds 10
    $size = (Get-Item $stdout -ErrorAction SilentlyContinue).Length
    if ($null -eq $size) { $size = 0 }
    if ($size -ne $lastSize) {
        $lastSize = $size
        $lastChange = Get-Date
    } elseif (((Get-Date) - $lastChange).TotalSeconds -gt 260) {
        Say "!! 260 s 无新输出，判定卡死，杀掉重来"
        try { $proc.Kill() } catch { }
        break
    }
}
$code = if ($proc.HasExited) { $proc.ExitCode } else { -1 }
Say "=== fog_mission 退出 code=$code size=$lastSize ==="
Get-Process -Name SmartCar -ErrorAction SilentlyContinue | Stop-Process -Force
Say "DONE out=$Out"

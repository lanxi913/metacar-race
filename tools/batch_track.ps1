# 批量跑弯道类场景：逐个启动 tracker + 点启动场景 + 收日志
param(
    [string[]]$Scenes = @("u型弯", "环形转弯", "连续转弯", "匝道行驶", "限速行驶", "环岛驶入、驶出", "车辆变道"),
    [int]$MaxSeconds = 150,
    [string]$Script = "tools\track_run.py"
)

$ErrorActionPreference = "Continue"
$root = "C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race"
Set-Location $root
$py = Join-Path $root ".venv\Scripts\python.exe"
$env:PYTHONIOENCODING = "utf-8"
New-Item -ItemType Directory -Force -Path (Join-Path $root "logs") | Out-Null

Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes
Add-Type -AssemblyName System.Windows.Forms

function Go-PageOne {
    $rootEl = [System.Windows.Automation.AutomationElement]::RootElement
    $win = $rootEl.FindAll([System.Windows.Automation.TreeScope]::Children,
        [System.Windows.Automation.Condition]::TrueCondition) |
        Where-Object { $_.Current.Name -like "*元动力客户端*" } | Select-Object -First 1
    if (-not $win) { return }
    $bc = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
        [System.Windows.Automation.ControlType]::Button)
    $p1 = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $bc) |
        Where-Object { $_.Current.Name -eq "Go to page 1" } | Select-Object -First 1
    if ($p1) {
        $p = $null
        if ($p1.TryGetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern, [ref]$p)) {
            $p.Invoke()
            Start-Sleep -Seconds 2
        }
    }
}

$results = @()

foreach ($scene in $Scenes) {
    Write-Host "`n===== $scene ====="
    Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force
    Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -like "*track_*.py*" } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
    Start-Sleep -Seconds 2

    Go-PageOne

    $safe = $scene -replace '[^\w\u4e00-\u9fa5]', '_'
    $log = Join-Path $root "logs\track_$safe.log"
    $proc = Start-Process -FilePath $py -ArgumentList @("-u", $Script) `
        -RedirectStandardOutput $log -RedirectStandardError "$log.err" -PassThru -WindowStyle Hidden
    Start-Sleep -Seconds 5

    $out = & (Join-Path $root "tools\launch_scene.ps1") -Title $scene
    Write-Host "  $out"

    $deadline = (Get-Date).AddSeconds($MaxSeconds)
    while ((Get-Date) -lt $deadline -and -not $proc.HasExited) {
        Start-Sleep -Seconds 2
    }
    if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue }
    Start-Sleep -Seconds 2

    $tail = Get-Content $log -ErrorAction SilentlyContinue | Select-Object -Last 3
    $results += [pscustomobject]@{ Scene = $scene; Log = $log; Tail = ($tail -join " / ") }
    Write-Host "  最后: $($tail -join ' / ')"
}

Write-Host "`n===== 汇总 ====="
$results | Format-Table -AutoSize
Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force

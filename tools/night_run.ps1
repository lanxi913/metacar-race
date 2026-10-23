# 通宵自动跑分（精简版）：逐场景启动控制器 -> 等待 -> 抓结算截图 -> 循环
param(
    [int]$RunTimeoutSec = 200,
    [double]$CooldownMinutes = 90,
    [int]$CooldownSleepMin = 15
)

$root = "C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race"
Set-Location $root
$py = Join-Path $root ".venv\Scripts\python.exe"
$env:PYTHONIOENCODING = "utf-8"
$resDir = Join-Path $root "logs\results"
New-Item -ItemType Directory -Force -Path $resDir | Out-Null

Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes
Add-Type -AssemblyName System.Windows.Forms

$Scenes = @(
    @{ T = "车辆直行";       S = "straight";       A = @("main.py", "--track", "--cruise", "0.5", "--seconds", "60") },
    @{ T = "车辆停止";       S = "stop";           A = @("tools\stop_run.py") },
    @{ T = "直角转弯";       S = "rightangleturn"; A = @("tools\track_bonus.py") },
    @{ T = "u型弯";          S = "uxingwan";       A = @("tools\track_bonus.py", "--offset", "0.8") },
    @{ T = "连续转弯";       S = "lianxuzhuanwan"; A = @("tools\track_bonus.py", "--offset", "1.3", "--cruise", "3.4", "--nobrake") },
    @{ T = "环形转弯";       S = "huanxingzhuanwan"; A = @("tools\track_bonus.py", "--cruise", "5.5", "--cscale", "2.0", "--nobrake", "--offset", "0.8") },
    @{ T = "匝道行驶";       S = "zadaoxingshi";   A = @("tools\track_bonus.py", "--offset", "0.8") },
    @{ T = "限速行驶";       S = "xiansuxingshi";  A = @("tools\track_bonus.py", "--offset", "0.8") },
    @{ T = "车辆变道";       S = "cheliangbiandao"; A = @("tools\track_bonus.py", "--offset", "0.8") },
    @{ T = "环岛驶入、驶出"; S = "huandaoshirushichu"; A = @("tools\track_bonus.py", "--offset", "1.0") }
)

function Close-Windows {
    Get-Process -Name msedge -ErrorAction SilentlyContinue | Stop-Process -Force
    Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force
    Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -like "*track_*" -or $_.CommandLine -like "*stop_run*" -or $_.CommandLine -like "*main.py*" } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
    Start-Sleep -Seconds 3
}

function Go-PageOne {
    $rootEl = [System.Windows.Automation.AutomationElement]::RootElement
    $win = $rootEl.FindAll([System.Windows.Automation.TreeScope]::Children,
        [System.Windows.Automation.Condition]::TrueCondition) |
        Where-Object { $_.Current.Name -like "*客户端*" } | Select-Object -First 1
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
    # 滚回列表顶部，保证按标题定位卡片时位置一致
    try {
        $win.SetFocus()
        [System.Windows.Forms.SendKeys]::SendWait("{PGUP 12}")
        Start-Sleep -Milliseconds 900
    } catch { }
}

$lastCooldown = Get-Date
$round = 0

while ($true) {
    $round++
    Write-Host "`n##### round $round #####"

    foreach ($s in $Scenes) {
        $title = $s.T
        $slug = $s.S
        $arg = $s.A
        Write-Host "`n=== $title ==="

        Close-Windows
        Go-PageOne

        $stamp = Get-Date -Format "HHmmss"
        $base = Join-Path $resDir ("{0}_{1}" -f $slug, $stamp)

        $proc = Start-Process -FilePath $py -ArgumentList $arg `
            -RedirectStandardOutput "$base.log" -RedirectStandardError "$base.err" `
            -PassThru -WindowStyle Hidden -WorkingDirectory $root
        Start-Sleep -Seconds 5

        & (Join-Path $root "tools\launch_scene.ps1") -Title $title | Out-Null

        $deadline = (Get-Date).AddSeconds($RunTimeoutSec)
        while ((Get-Date) -lt $deadline -and -not $proc.HasExited) { Start-Sleep -Seconds 2 }
        if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue }
        Start-Sleep -Seconds 3

        try {
            & (Join-Path $root "tools\capture_window.ps1") -Frames 1 -IntervalMs 200 `
                -CropLeft 0 -CropTop 0.2 -CropRight 1 -CropBottom 1 | Out-Null
            $shot = Join-Path $env:TEMP "hudseq2\c01.png"
            if (Test-Path $shot) { Copy-Item $shot "$base.png" -Force }
        } catch { }

        Close-Windows

        if (((Get-Date) - $lastCooldown).TotalMinutes -ge $CooldownMinutes) {
            Write-Host "`n=== cooldown $CooldownSleepMin min ==="
            Start-Sleep -Seconds ($CooldownSleepMin * 60)
            $lastCooldown = Get-Date
        }
    }
}

# 通宵自动跑分：逐场景跑 -> 读分 -> 记录 -> 定时降温 -> 清理窗口
param(
    [int]$RunTimeoutSec = 240,
    [double]$CooldownMinutes = 90,
    [int]$CooldownSleepMin = 15,
    [string]$Csv = "docs\results.csv"
)

$root = "C:\Users\lanxi\Documents\deepseek-harness\projects\metacar-race"
Set-Location $root
$py = Join-Path $root ".venv\Scripts\python.exe"
$env:PYTHONIOENCODING = "utf-8"
New-Item -ItemType Directory -Force -Path (Join-Path $root "logs") | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $root "logs\results") | Out-Null

Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes

# 场景清单：标题 / slug / 控制器参数
$Scenes = @(
    @{ Title = "车辆直行";        Slug = "straight";          Arg = @("main.py", "--track", "--cruise", "0.5", "--seconds", "60") },
    @{ Title = "车辆停止";        Slug = "stop";              Arg = @("tools\stop_run.py") },
    @{ Title = "直角转弯";        Slug = "rightangleturn";    Arg = @("tools\track_bonus.py") },
    @{ Title = "u型弯";           Slug = "uxingwan";          Arg = @("tools\track_bonus.py", "--offset", "0.8") },
    @{ Title = "连续转弯";        Slug = "lianxuzhuanwan";    Arg = @("tools\track_bonus.py", "--offset", "1.3", "--cruise", "3.4", "--nobrake") },
    @{ Title = "环形转弯";        Slug = "huanxingzhuanwan";  Arg = @("tools\track_bonus.py", "--cruise", "5.5", "--cscale", "2.0", "--nobrake", "--offset", "0.8") },
    @{ Title = "匝道行驶";        Slug = "zadaoxingshi";      Arg = @("tools\track_bonus.py", "--offset", "0.8") },
    @{ Title = "限速行驶";        Slug = "xiansuxingshi";     Arg = @("tools\track_bonus.py", "--offset", "0.8") },
    @{ Title = "车辆变道";        Slug = "cheliangbiandao";   Arg = @("tools\track_bonus.py", "--offset", "0.8") },
    @{ Title = "环岛驶入、驶出";  Slug = "huandaoshirushichu"; Arg = @("tools\track_bonus.py", "--offset", "1.0") },
)

function Close-Windows {
    Get-Process -Name msedge -ErrorAction SilentlyContinue | Stop-Process -Force
    Get-Process -Name SmartCar, UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep -Seconds 2
}

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
            $p.Invoke(); Start-Sleep -Seconds 2
        }
    }
}

function Read-Score([string]$Slug) {
    Close-Windows
    Start-Process "msedge.exe" -ArgumentList "--new-window", "https://www.i-mylab.com/icv/history.html?id=$Slug"
    Start-Sleep -Seconds 25
    $pids = @((Get-Process -Name msedge -ErrorAction SilentlyContinue).Id)
    $rootEl = [System.Windows.Automation.AutomationElement]::RootElement
    $win = $rootEl.FindAll([System.Windows.Automation.TreeScope]::Children,
        [System.Windows.Automation.Condition]::TrueCondition) |
        Where-Object { $pids -contains $_.Current.ProcessId -and $_.Current.Name -like "*元动力*" } |
        Select-Object -First 1
    if (-not $win) { return @("", "") }
    $all = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.Condition]::TrueCondition)
    $t = @()
    foreach ($e in $all) { $n = $e.Current.Name; if ($n -and $n.Trim()) { $t += $n.Trim() } }
    $i = [array]::IndexOf($t, "得分")
    if ($i -lt 0) { return @("", "") }
    $body = @($t[($i + 1)..($t.Count - 1)] | Where-Object { $_ -ne "" -and $_ -ne "详情" })
    for ($k = 0; $k -lt $body.Count; $k++) {
        if ($body[$k] -match '^\d{4}/\d{2}/\d{2} \d{2}:\d{2}:\d{2}$') {
            for ($m = $k + 1; $m -lt [Math]::Min($k + 4, $body.Count); $m++) {
                if ($body[$m] -match '^\d{1,3}\.\d{2}$') { return @($body[$k], $body[$m]) }
            }
        }
    }
    return @("", "")
}

if (-not (Test-Path $Csv)) {
    "时间,场景,slug,控制器,得分,记录时间" | Set-Content -Path $Csv -Encoding UTF8
}

$startTime = Get-Date
$lastCooldown = Get-Date
$round = 0

while ($true) {
    $round++
    Write-Host "`n########## 第 $round 轮 ##########"

    foreach ($s in $Scenes) {
        $title = $s.Title; $slug = $s.Slug; $arg = $s.Arg
        $ctrl = ($arg -join " ")
        Write-Host "`n=== $title  ($ctrl) ==="

        Close-Windows
        Go-PageOne

        $logBase = Join-Path $root ("logs\results\{0}_{1:HHmmss}" -f $slug, (Get-Date))
        $proc = Start-Process -FilePath $py -ArgumentList $arg `
            -RedirectStandardOutput "$logBase.log" -RedirectStandardError "$logBase.err" `
            -PassThru -WindowStyle Hidden -WorkingDirectory $root
        Start-Sleep -Seconds 5

        $out = & (Join-Path $root "tools\launch_scene.ps1") -Title $title
        Write-Host "   $out"

        $deadline = (Get-Date).AddSeconds($RunTimeoutSec)
        while ((Get-Date) -lt $deadline -and -not $proc.HasExited) { Start-Sleep -Seconds 2 }
        if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue }
        Start-Sleep -Seconds 3

        # 抓一张结算画面存档
        try {
            & (Join-Path $root "tools\capture_window.ps1") -Frames 1 -IntervalMs 200 `
                -CropLeft 0 -CropTop 0.25 -CropRight 1 -CropBottom 1 | Out-Null
            $shot = Join-Path $env:TEMP "hudseq2\c01.png"
            if (Test-Path $shot) { Copy-Item $shot "$logBase.png" -Force }
        } catch { }

        $score = ""; $ts = ""
        try {
            $r = Read-Score $slug
            $ts = $r[0]; $score = $r[1]
        } catch { }

        $line = "{0},{1},{2},{3},{4},{5}" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss"), $title, $slug, $ctrl, $score, $ts
        if ($score) { Write-Host "   >>> 得分 $score ($ts)"; $line | Add-Content -Path $Csv -Encoding UTF8 }
        else { Write-Host "   >>> 未读到分数"; }

        Close-Windows

        # 降温
        if (((Get-Date) - $lastCooldown).TotalMinutes -ge $CooldownMinutes) {
            Write-Host "`n=== 降温休息 $CooldownSleepMin 分钟 ==="
            Close-Windows
            Start-Sleep -Seconds ($CooldownSleepMin * 60)
            $lastCooldown = Get-Date
        }
    }

    Write-Host "`n一轮完成，用时 $([int]((Get-Date) - $startTime).TotalMinutes) 分钟，累计记录："
    if (Test-Path $Csv) { Get-Content $Csv | Select-Object -Last 12 }
}

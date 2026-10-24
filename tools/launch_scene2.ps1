# 场景启动器 v2：按"标题-按钮最近邻"配对，窗口尺寸变化也不会点错
# 用法： powershell -File tools\launch_scene2.ps1 -Title "异形车道泊车"
param(
    [Parameter(Mandatory = $true)][string]$Title
)

Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes

$rootEl = [System.Windows.Automation.AutomationElement]::RootElement
$win = $rootEl.FindAll([System.Windows.Automation.TreeScope]::Children,
    [System.Windows.Automation.Condition]::TrueCondition) |
    Where-Object { $_.Current.Name -like "*元动力客户端*" } | Select-Object -First 1
if (-not $win) { Write-Output "client window not found"; exit 1 }

$all = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
    [System.Windows.Automation.Condition]::TrueCondition)

$titles = @()
$btns = @()
foreach ($e in $all) {
    $c = $e.Current
    if ($c.Name -eq "启动场景") { $btns += $e; continue }
    if ($c.ControlType.ProgrammaticName -eq "ControlType.Text" -and $c.Name -and
        $c.Name -notmatch "lanxi913|场景训练|比赛列表|下载管理|查看详情|启动场景") {
        if ($c.BoundingRectangle.Width -gt 0) { $titles += $e }
    }
}

# 场景不在视口内时先翻页/滚动一次
if (-not ($titles | Where-Object { $_.Current.Name -eq $Title })) {
    $p2 = $all | Where-Object { $_.Current.Name -eq "Go to page 2" } | Select-Object -First 1
    if ($p2) {
        $pp = $null
        if ($p2.TryGetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern, [ref]$pp)) {
            $pp.Invoke(); Start-Sleep -Seconds 2
        }
        $all = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
            [System.Windows.Automation.Condition]::TrueCondition)
        $titles = @(); $btns = @()
        foreach ($e in $all) {
            $c = $e.Current
            if ($c.Name -eq "启动场景") { $btns += $e; continue }
            if ($c.ControlType.ProgrammaticName -eq "ControlType.Text" -and $c.Name -and
                $c.Name -notmatch "lanxi913|场景训练|比赛列表|下载管理|查看详情|启动场景") {
                if ($c.BoundingRectangle.Width -gt 0) { $titles += $e }
            }
        }
    }
}

# 每个按钮配对它正上方最近的标题，取标题等于目标的那一个
$target = $null
foreach ($b in $btns) {
    $br = $b.Current.BoundingRectangle
    $bestT = $null; $bd = 1e9
    foreach ($t in $titles) {
        $tr = $t.Current.BoundingRectangle
        $dy = [double]$br.Y - [double]$tr.Y
        $dx = [math]::Abs([double]$br.X - [double]$tr.X)
        if ($dy -gt 100 -and $dy -lt 600 -and $dx -lt 700) {
            $d = $dx + [math]::Abs($dy - 310)
            if ($d -lt $bd) { $bd = $d; $bestT = $t }
        }
    }
    if ($bestT -and $bestT.Current.Name -eq $Title) { $target = $b; break }
}

if (-not $target) { Write-Output "title '$Title' not found"; exit 1 }

$p = $null
if ($target.TryGetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern, [ref]$p)) {
    $p.Invoke()
    Write-Output "launched: $Title"
} else {
    Write-Output "invoke failed"
    exit 1
}

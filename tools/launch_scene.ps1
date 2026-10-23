# 场景启动器：按场景名找到对应的「启动场景」按钮并点击
# 用法： .\tools\launch_scene.ps1 -Title "异形车道泊车"
param(
    [Parameter(Mandatory = $true)][string]$Title
)

Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes

$rootEl = [System.Windows.Automation.AutomationElement]::RootElement
$win = $rootEl.FindAll([System.Windows.Automation.TreeScope]::Children,
    [System.Windows.Automation.Condition]::TrueCondition) |
    Where-Object { $_.Current.Name -like "*元动力客户端*" } | Select-Object -First 1
if (-not $win) { Write-Output "client window not found"; exit 1 }

function Get-StartButtons {
    $bc = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
        [System.Windows.Automation.ControlType]::Button)
    return @($win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $bc) |
        Where-Object { $_.Current.Name -eq "启动场景" })
}

function Find-TitleElement([string]$name) {
    $all = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.Condition]::TrueCondition)
    foreach ($e in $all) {
        if ($e.Current.Name -eq $name) {
            $r = $e.Current.BoundingRectangle
            if ($r.Width -gt 0) { return $r }
        }
    }
    return $null
}

# 直接在所有元素里找标题（UIA 对未渲染区域也可能给出坐标）
$rect = Find-TitleElement $Title

if (-not $rect) { Write-Output "title '$Title' not found"; exit 1 }

# 卡片布局：按钮在标题下方约 310px、横向 +210px
$targetX = $rect.X + 210
$targetY = $rect.Y + 310

$best = $null
$bestDist = 1e9
foreach ($b in (Get-StartButtons)) {
    $r = $b.Current.BoundingRectangle
    if ($r.Width -le 0) { continue }
    $cx = $r.X + $r.Width / 2
    $cy = $r.Y + $r.Height / 2
    $d = [math]::Sqrt(($cx - $targetX) * ($cx - $targetX) + ($cy - $targetY) * ($cy - $targetY))
    if ($d -lt $bestDist) { $bestDist = $d; $best = $b }
}

if (-not $best -or $bestDist -gt 260) { Write-Output "no nearby start button (dist=$([int]$bestDist))"; exit 1 }

$p = $null
if ($best.TryGetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern, [ref]$p)) {
    $p.Invoke()
    Write-Output "launched: $Title (dist=$([int]$bestDist))"
} else {
    Write-Output "invoke failed"
    exit 1
}

# 读某个场景最新一条训练记录：开干净 Edge 窗口 -> 训练记录 -> 正则抓「时间戳 + 得分」
param(
    [Parameter(Mandatory = $true)][string]$Slug
)

Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes

Get-Process -Name msedge -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Seconds 2
Start-Process "msedge.exe" -ArgumentList "--new-window", "https://www.i-mylab.com/icv/detail.html?id=$Slug"
Start-Sleep -Seconds 15

# 只认 Edge 进程的窗口（DSH 自己的窗口也是 Chrome_WidgetWin_1，必须按进程过滤）
$edgePids = @((Get-Process -Name msedge -ErrorAction SilentlyContinue).Id)
$rootEl = [System.Windows.Automation.AutomationElement]::RootElement
$win = $rootEl.FindAll([System.Windows.Automation.TreeScope]::Children,
    [System.Windows.Automation.Condition]::TrueCondition) |
    Where-Object { $edgePids -contains $_.Current.ProcessId } | Select-Object -First 1
if (-not $win) { Write-Output "$Slug`tNO_WINDOW"; exit 1 }

$bc = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
    [System.Windows.Automation.ControlType]::Button)
$rec = @($win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $bc) |
    Where-Object { $_.Current.Name -eq "训练记录" })[0]
if ($rec) {
    $p = $null
    if ($rec.TryGetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern, [ref]$p)) {
        $p.Invoke()
        Start-Sleep -Seconds 9
    }
}

$dc = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
    [System.Windows.Automation.ControlType]::DataItem)
$rows = @($win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $dc) |
    ForEach-Object { $_.Current.Name.Trim() } | Where-Object { $_ -ne "" })

$ts = $null
$score = $null
for ($i = 0; $i -lt $rows.Count; $i++) {
    if ($rows[$i] -match '^\d{4}/\d{2}/\d{2} \d{2}:\d{2}:\d{2}$') {
        $ts = $rows[$i]
        for ($j = $i + 1; $j -lt [Math]::Min($i + 6, $rows.Count); $j++) {
            if ($rows[$j] -match '^\d{1,3}\.\d{2}$') { $score = $rows[$j]; break }
        }
        if ($score) { break }
    }
}

if ($ts -and $score) {
    Write-Output ("{0}`t{1}`t{2}" -f $Slug, $ts, $score)
} else {
    Write-Output ("{0}`tNO_RECORD`trows={1}" -f $Slug, $rows.Count)
}

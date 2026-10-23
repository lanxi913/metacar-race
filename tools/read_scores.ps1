# 批量读成绩：逐个 slug 打开 history 页面，解析最新一条记录（场景/时间/得分）
param(
    [string[]]$Slugs = @("uxingwan", "lianxuzhuanwan", "zadaoxingshi", "xiansuxingshi", "cheliangbiandao", "huandaoshirushichu", "rightangleturn")
)

Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes

foreach ($slug in $Slugs) {
    Get-Process -Name msedge -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep -Seconds 2
    Start-Process "msedge.exe" -ArgumentList "--new-window", "https://www.i-mylab.com/icv/history.html?id=$slug"
    Start-Sleep -Seconds 20

    $pids = @((Get-Process -Name msedge -ErrorAction SilentlyContinue).Id)
    $rootEl = [System.Windows.Automation.AutomationElement]::RootElement
    $win = $rootEl.FindAll([System.Windows.Automation.TreeScope]::Children,
        [System.Windows.Automation.Condition]::TrueCondition) |
        Where-Object { $pids -contains $_.Current.ProcessId -and $_.Current.Name -like "*元动力*" } |
        Select-Object -First 1
    if (-not $win) { Write-Output "$slug`tNO_WINDOW"; continue }

    $all = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
        [System.Windows.Automation.Condition]::TrueCondition)
    $t = @()
    foreach ($e in $all) { $n = $e.Current.Name; if ($n -and $n.Trim()) { $t += $n.Trim() } }

    # 找到「得分」表头后，取第一条记录：场景名 / 开始 / 结束 / 得分
    $i = [array]::IndexOf($t, "得分")
    if ($i -lt 0) { Write-Output "$slug`tNO_TABLE`ttexts=$($t.Count)"; continue }
    $body = $t[($i + 2)..($t.Count - 1)]
    if ($body.Count -ge 4) {
        Write-Output ("{0}`t{1}`t{2}`t{3}" -f $slug, $body[0], $body[1].Substring(0, [Math]::Min(16, $body[1].Length)), $body[3])
    } else {
        Write-Output "$slug`tNO_RECORD"
    }
}

Get-Process -Name msedge -ErrorAction SilentlyContinue | Stop-Process -Force

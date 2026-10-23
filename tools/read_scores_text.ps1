# 读多个场景的最新成绩：history.html 页面的表格是 Text 元素（不是 DataItem）
param(
    [string[]]$Slugs = @("uxingwan", "lianxuzhuanwan", "zadaoxingshi", "xiansuxingshi", "cheliangbiandao", "huandaoshirushichu")
)

Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes

foreach ($slug in $Slugs) {
    Get-Process -Name msedge -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep -Seconds 2
    Start-Process "msedge.exe" -ArgumentList "--new-window", "https://www.i-mylab.com/icv/history.html?id=$slug"
    Start-Sleep -Seconds 22

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

    $i = [array]::IndexOf($t, "得分")
    if ($i -lt 0) { Write-Output ("{0}`tNO_TABLE`ttexts={1}" -f $slug, $t.Count); continue }

    # 得分表头后面：场景名 / 开始 / 结束 / 分数 / 查看
    $body = @($t[($i + 1)..($t.Count - 1)] | Where-Object { $_ -ne "" -and $_ -ne "详情" })
    $ts = $null
    $score = $null
    for ($k = 0; $k -lt $body.Count; $k++) {
        if ($body[$k] -match '^\d{4}/\d{2}/\d{2} \d{2}:\d{2}:\d{2}$') {
            for ($m = $k + 1; $m -lt [Math]::Min($k + 4, $body.Count); $m++) {
                if ($body[$m] -match '^\d{1,3}\.\d{2}$') { $ts = $body[$k]; $score = $body[$m]; break }
            }
            if ($score) { break }
        }
    }
    if ($score) { Write-Output ("{0}`t{1}`t{2}" -f $slug, $ts, $score) }
    else { Write-Output ("{0}`tNO_RECORD" -f $slug) }
}

Get-Process -Name msedge -ErrorAction SilentlyContinue | Stop-Process -Force

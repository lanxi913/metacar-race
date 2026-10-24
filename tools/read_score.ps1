param(
    [string]$Url = "https://www.i-mylab.com/icv/history.html?id=93C092E5A4A9E8B9AFE8B5B0E58F917465",
    [int]$Wait = 9,
    [int]$MaxLines = 120
)

# ASCII-only helper: open the platform history page in Edge and dump its text via UIA.
Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type @"
using System;
using System.Runtime.InteropServices;
public class W32s {
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
    [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr h);
}
"@
$AE = [System.Windows.Automation.AutomationElement]
$TS = [System.Windows.Automation.TreeScope]

function Get-Text([System.Windows.Automation.AutomationElement]$root) {
    $out = New-Object System.Collections.ArrayList
    $walk = {
        param($el, $depth)
        if ($depth -gt 40) { return }
        $kids = $null
        try { $kids = $el.FindAll($TS::Children, [System.Windows.Automation.Condition]::TrueCondition) } catch { return }
        foreach ($k in $kids) {
            $nm = ''
            try { $nm = $k.Current.Name } catch { }
            $ct = ''
            try { $ct = $k.Current.ControlType.ProgrammaticName } catch { }
            if ($nm -and $nm.Trim().Length -gt 0 -and $ct -match 'Text|DataItem|Button|Header|Hyperlink|ListItem') {
                [void]$out.Add(("$ct|" + $nm.Trim()))
            }
            & $walk $k ($depth + 1)
        }
    }
    & $walk $root 0
    return $out
}

Start-Process "msedge.exe" -ArgumentList "--new-window", $Url | Out-Null
Start-Sleep -Seconds $Wait

# 顶层窗口里挑标题含 "元动力训练营" 的那个（Edge 多窗口时 MainWindowHandle 不一定是它）
$h = [IntPtr]::Zero
$best = $null
$top = $AE::RootElement.FindAll($TS::Children, [System.Windows.Automation.Condition]::TrueCondition)
foreach ($w in $top) {
    $nm = ''
    try { $nm = $w.Current.Name } catch { }
    if ($nm -and $nm -match [char]0x5143 -and $nm -match [char]0x8BAD) { $best = $w; break }
}
if (-not $best) {
    $proc = Get-Process msedge -ErrorAction SilentlyContinue |
        Where-Object { $_.MainWindowHandle -ne 0 } |
        Sort-Object StartTime -Descending | Select-Object -First 1
    if (-not $proc) { Write-Output "NO_EDGE_WINDOW"; exit 1 }
    $best = $AE::FromHandle($proc.MainWindowHandle)
}
$h = [IntPtr]$best.Current.NativeWindowHandle
[void][W32s]::ShowWindow($h, 9)
Start-Sleep -Milliseconds 400
[void][W32s]::SetForegroundWindow($h)
Start-Sleep -Milliseconds 1500

$root = $AE::FromHandle($h)
$texts = Get-Text $root
if (($texts -join "`n") -notmatch 'DataItem') {
    # page body did not render: retype the URL in the address bar and hit Enter
    Add-Type -AssemblyName System.Windows.Forms
    [void][W32s]::SetForegroundWindow($h)
    Start-Sleep -Milliseconds 500
    [System.Windows.Forms.SendKeys]::SendWait("^l")
    Start-Sleep -Milliseconds 300
    [System.Windows.Forms.SendKeys]::SendWait($Url)
    Start-Sleep -Milliseconds 300
    [System.Windows.Forms.SendKeys]::SendWait("{ENTER}")
    Start-Sleep -Seconds 8
    $texts = Get-Text $root
}
# poll: the table is blank for a few seconds after refresh, wait for data rows
for ($k = 0; $k -lt 6; $k++) {
    $n = 0
    foreach ($t in $texts) { if ($t -like 'ControlType.DataItem|*') { $n++ } }
    if ($n -gt 6) { break }
    Start-Sleep -Seconds 4
    $texts = Get-Text $root
}
$i = 0
foreach ($t in $texts) {
    if ($i -ge $MaxLines) { break }
    Write-Output $t
    $i++
}
Write-Output ("TOTAL=" + $texts.Count)

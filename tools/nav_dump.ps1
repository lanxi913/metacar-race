param(
    [string]$Url,
    [int]$Wait = 12,
    [int]$MaxLines = 60
)
# ASCII-only: navigate the existing "训练营" Edge window to $Url via UIA ValuePattern
# (SendKeys ^l is unreliable with several tabs), then dump page text.
Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -AssemblyName System.Windows.Forms
Add-Type @"
using System;
using System.Runtime.InteropServices;
public class W32n {
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
}
"@
$AE = [System.Windows.Automation.AutomationElement]
$TS = [System.Windows.Automation.TreeScope]
$CT = [System.Windows.Automation.ControlType]

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
            if ($nm -and $nm.Trim().Length -gt 0 -and $ct -match 'Text|DataItem|Button|Header|Hyperlink|ListItem|Edit') {
                [void]$out.Add(("$ct|" + $nm.Trim()))
            }
            & $walk $k ($depth + 1)
        }
    }
    & $walk $root 0
    return $out
}

$top = $AE::RootElement.FindAll($TS::Children, [System.Windows.Automation.Condition]::TrueCondition)
$best = $null
foreach ($w in $top) {
    $nm = ''
    try { $nm = $w.Current.Name } catch { }
    if ($nm -and $nm -match [char]0x8BAD -and $nm -match [char]0x8425) { $best = $w; break }
}
if (-not $best) {
    $proc = Get-Process msedge -ErrorAction SilentlyContinue |
        Where-Object { $_.MainWindowHandle -ne 0 } |
        Sort-Object StartTime -Descending | Select-Object -First 1
    if (-not $proc) { Write-Output "NO_EDGE_WINDOW"; exit 1 }
    $best = $AE::FromHandle($proc.MainWindowHandle)
}
$h = [IntPtr]$best.Current.NativeWindowHandle
[void][W32n]::ShowWindow($h, 9)
Start-Sleep -Milliseconds 400
[void][W32n]::SetForegroundWindow($h)
Start-Sleep -Milliseconds 1200
$root = $AE::FromHandle($h)

$cond = New-Object System.Windows.Automation.PropertyCondition($AE::ControlTypeProperty, $CT::Edit)
$edits = $root.FindAll($TS::Descendants, $cond)
$addr = $null
foreach ($e in $edits) {
    $aid = ''
    try { $aid = $e.Current.AutomationId } catch { }
    $nm = ''
    try { $nm = $e.Current.Name } catch { }
    Write-Output ("EDIT|" + $aid + "|" + $nm)
    if ($aid -eq 'view_1017' -or $nm -match [char]0x5730 -or $nm -match 'Address') { $addr = $e }
}
if (-not $addr -and $edits.Count -gt 0) { $addr = $edits[0] }
if ($addr) {
    try {
        $vp = $addr.GetCurrentPattern([System.Windows.Automation.ValuePattern]::Pattern)
        $vp.SetValue($Url)
        Start-Sleep -Milliseconds 500
        [System.Windows.Forms.SendKeys]::SendWait("{ENTER}")
        Write-Output "NAV_OK"
    } catch {
        Write-Output ("NAV_FAIL|" + $_.Exception.Message)
    }
} else {
    Write-Output "NO_ADDRESS_BAR"
}
Start-Sleep -Seconds $Wait
$title = ''
try { $title = $root.Current.Name } catch { }
Write-Output ("TITLE|" + $title)
$texts = Get-Text $root
for ($k = 0; $k -lt 5; $k++) {
    $n = 0
    foreach ($t in $texts) { if ($t -like 'ControlType.DataItem|*') { $n++ } }
    if ($n -gt 6) { break }
    Start-Sleep -Seconds 4
    $texts = Get-Text $root
    try { $title = $root.Current.Name } catch { }
}
Write-Output ("TITLE2|" + $title)
$i = 0
foreach ($t in $texts) {
    if ($t -like 'EDIT|*') { continue }
    if ($i -ge $MaxLines) { break }
    Write-Output $t
    $i++
}
Write-Output ("TOTAL=" + $texts.Count)

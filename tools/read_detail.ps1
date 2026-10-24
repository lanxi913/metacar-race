param(
    [string]$Url = "https://www.i-mylab.com/icv/history.html?id=shangxiapotingche",
    [int]$Wait = 9,
    [int]$Rows = 0,
    [int]$MaxLines = 120
)

# ASCII-only helper: open a history page, click the "view detail" button of the
# N-th row (or the "TOTAL" row when -Rows >= 999), then dump the detail text.
Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type @"
using System;
using System.Runtime.InteropServices;
public class W32d {
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
}
"@
$AE = [System.Windows.Automation.AutomationElement]
$TS = [System.Windows.Automation.TreeScope]
$U_VIEW = -join ([char]0x67E5, [char]0x770B)

function Get-Texts([System.Windows.Automation.AutomationElement]$root) {
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
            if ($nm -and $nm.Trim().Length -gt 0 -and $ct -match 'Text|DataItem|Button|Header') {
                [void]$out.Add("$ct|" + $nm.Trim())
            }
            & $walk $k ($depth + 1)
        }
    }
    & $walk $root 0
    return $out
}

Start-Process "msedge.exe" -ArgumentList "--new-window", $Url | Out-Null
Start-Sleep -Seconds $Wait

$best = $null
$top = $AE::RootElement.FindAll($TS::Children, [System.Windows.Automation.Condition]::TrueCondition)
foreach ($w in $top) {
    $nm = ''
    try { $nm = $w.Current.Name } catch { }
    if ($nm -and $nm -match [char]0x8BAD -and $nm -match [char]0x8425) { $best = $w; break }
}
if (-not $best) { Write-Output "NO_EDGE_WINDOW"; exit 1 }
$h = [IntPtr]$best.Current.NativeWindowHandle
[void][W32d]::ShowWindow($h, 9)
Start-Sleep -Milliseconds 400
[void][W32d]::SetForegroundWindow($h)
Start-Sleep -Milliseconds 1200

$root = $AE::FromHandle($h)
$texts = Get-Texts $root
for ($k = 0; $k -lt 6; $k++) {
    $n = 0
    foreach ($t in $texts) { if ($t -like 'ControlType.DataItem|*') { $n++ } }
    if ($n -gt 6) { break }
    Start-Sleep -Seconds 4
    $texts = Get-Texts $root
}

$views = @($texts | Where-Object { $_ -eq ("ControlType.Button|" + $U_VIEW) })
Write-Output ("view buttons: " + $views.Count)
if ($views.Count -eq 0) {
    foreach ($t in $texts) { Write-Output $t }
    exit 1
}

$idx = 0
if ($Rows -gt 0 -and $Rows -lt $views.Count) { $idx = $Rows - 1 }
$all = $root.FindAll($TS::Descendants, [System.Windows.Automation.Condition]::TrueCondition)
$btns = @($all | Where-Object {
    $_.Current.ControlType.ProgrammaticName -eq 'ControlType.Button' -and $_.Current.Name -eq $U_VIEW
})
$target = $btns[$idx]
$pat = $target.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern)
$pat.Invoke()
Start-Sleep -Seconds 5

$root2 = $AE::FromHandle($h)
$t2 = @(Get-Texts $root2)
$i = 0
foreach ($t in $t2) {
    if ($i -ge $MaxLines) { break }
    Write-Output $t
    $i++
}
Write-Output ("TOTAL=" + $t2.Count)


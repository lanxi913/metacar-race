param(
    [string]$Url = "https://www.i-mylab.com/icv/history.html?id=youxingrenyouzhuan",
    [int]$Wait = 10,
    [int]$Rows = 1,
    [int]$MaxLines = 140
)

# ASCII-only helper. Same idea as read_detail.ps1 but tolerant about how the
# "view detail" control is exposed: newer pages render it as a hyperlink /
# data item instead of a Button, so try InvokePattern then LegacyIAccessible.
Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -AssemblyName System.Windows.Forms
Add-Type @"
using System;
using System.Runtime.InteropServices;
public class W32e {
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
}
"@
$AE = [System.Windows.Automation.AutomationElement]
$TS = [System.Windows.Automation.TreeScope]
$U_VIEW = -join ([char]0x67E5, [char]0x770B)

function Get-Window {
    $top = $AE::RootElement.FindAll($TS::Children, [System.Windows.Automation.Condition]::TrueCondition)
    foreach ($w in $top) {
        $nm = ''
        try { $nm = $w.Current.Name } catch { }
        if ($nm -and $nm -match [char]0x8BAD -and $nm -match [char]0x8425) { return $w }
    }
    return $null
}

function Nav-To($url) {
    $w = Get-Window
    if (-not $w) { return $null }
    $h = [IntPtr]$w.Current.NativeWindowHandle
    [void][W32e]::ShowWindow($h, 9)
    [void][W32e]::SetForegroundWindow($h)
    Start-Sleep -Milliseconds 800
    $root = $AE::FromHandle($h)
    $all = $root.FindAll($TS::Descendants, [System.Windows.Automation.Condition]::TrueCondition)
    foreach ($e in $all) {
        $aid = ''
        try { $aid = $e.Current.AutomationId } catch { }
        if ($aid -eq 'view_1017') {
            try {
                $vp = $e.GetCurrentPattern([System.Windows.Automation.ValuePattern]::Pattern)
                $vp.SetValue($url)
                Start-Sleep -Milliseconds 300
                [System.Windows.Forms.SendKeys]::SendWait("{ENTER}")
                return $h
            } catch { }
        }
    }
    return $null
}

function Get-Texts($root) {
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
            if ($nm -and $nm.Trim().Length -gt 0) {
                [void]$out.Add("$ct|" + $nm.Trim())
            }
            & $walk $k ($depth + 1)
        }
    }
    & $walk $root 0
    return $out
}

$w = Get-Window
if (-not $w) {
    Start-Process "msedge.exe" -ArgumentList "--new-window", $Url | Out-Null
    Start-Sleep -Seconds $Wait
    $w = Get-Window
}
if (-not $w) { Write-Output "NO_EDGE_WINDOW"; exit 1 }
$h = Nav-To $Url
if (-not $h) { $h = [IntPtr]$w.Current.NativeWindowHandle }
Start-Sleep -Seconds 6

$root = $AE::FromHandle($h)
$all = $root.FindAll($TS::Descendants, [System.Windows.Automation.Condition]::TrueCondition)
$cands = @($all | Where-Object {
    $n = ''
    try { $n = $_.Current.Name } catch { }
    $n -eq $U_VIEW
})
Write-Output ("view candidates: " + $cands.Count)
foreach ($c in $cands) {
    $ct = ''
    try { $ct = $c.Current.ControlType.ProgrammaticName } catch { }
    $pats = ''
    try { $pats = ($c.GetSupportedPatterns() | ForEach-Object { $_.ProgrammaticName }) -join ',' } catch { }
    Write-Output ("  -> " + $ct + "  [" + $pats + "]")
}
if ($cands.Count -eq 0) {
    $t0 = @(Get-Texts $root)
    foreach ($t in $t0) { Write-Output $t }
    exit 1
}

$idx = 0
if ($Rows -gt 0 -and $Rows -le $cands.Count) { $idx = $Rows - 1 }
$target = $cands[$idx]
$ok = $false
try {
    $p = $target.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern)
    $p.Invoke()
    $ok = $true
    Write-Output "clicked via InvokePattern"
} catch {
    try {
        $p = $target.GetCurrentPattern([System.Windows.Automation.LegacyIAccessiblePattern]::Pattern)
        $p.DoDefaultAction()
        $ok = $true
        Write-Output "clicked via LegacyIAccessible"
    } catch { }
}
if (-not $ok) {
    $r = $target.Current.BoundingRectangle
    [System.Windows.Forms.Cursor]::Position = New-Object System.Drawing.Point([int]($r.X + $r.Width / 2), [int]($r.Y + $r.Height / 2))
    Start-Sleep -Milliseconds 200
    [System.Windows.Forms.SendKeys]::SendWait("{ENTER}")
    Write-Output "clicked via focus+ENTER"
}
Start-Sleep -Seconds 6

$root2 = $AE::FromHandle($h)
$t2 = @(Get-Texts $root2)
$i = 0
foreach ($t in $t2) {
    if ($i -ge $MaxLines) { break }
    Write-Output $t
    $i++
}
Write-Output ("TOTAL=" + $t2.Count)

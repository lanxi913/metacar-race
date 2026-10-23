# 按窗口句柄抓取指定进程主窗口（不受前台焦点影响），并裁剪左下角 HUD 区域
param(
    [string]$ProcessName = "SmartCar",
    [string]$OutDir = "$env:TEMP\hudseq2",
    [int]$Frames = 24,
    [int]$IntervalMs = 1000,
    [double]$CropLeft = 0.0,
    [double]$CropTop = 0.62,
    [double]$CropRight = 0.30,
    [double]$CropBottom = 1.0
)

Add-Type -AssemblyName System.Drawing

$code = @"
using System;
using System.Runtime.InteropServices;
using System.Drawing;
using System.Drawing.Imaging;

public class WinShot
{
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr hWnd, IntPtr hdcBlt, uint nFlags);
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hWnd, out RECT lpRect);

    [StructLayout(LayoutKind.Sequential)]
    public struct RECT { public int Left, Top, Right, Bottom; }

    public static Bitmap Capture(IntPtr hwnd)
    {
        RECT r;
        if (!GetWindowRect(hwnd, out r)) return null;
        int w = r.Right - r.Left, h = r.Bottom - r.Top;
        if (w <= 0 || h <= 0) return null;
        Bitmap bmp = new Bitmap(w, h, PixelFormat.Format32bppArgb);
        using (Graphics g = Graphics.FromImage(bmp))
        {
            IntPtr hdc = g.GetHdc();
            PrintWindow(hwnd, hdc, 2);   // PW_RENDERFULLCONTENT
            g.ReleaseHdc(hdc);
        }
        return bmp;
    }
}
"@

if (-not ("WinShot" -as [type])) { Add-Type -TypeDefinition $code -ReferencedAssemblies System.Drawing, System.Drawing.Primitives }

$proc = Get-Process -Name $ProcessName -ErrorAction SilentlyContinue | Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
if (-not $proc) { Write-Output "NO_WINDOW"; exit 1 }

if (Test-Path $OutDir) { Remove-Item $OutDir -Recurse -Force }
New-Item -ItemType Directory -Path $OutDir | Out-Null

for ($i = 1; $i -le $Frames; $i++) {
    $p = Get-Process -Name $ProcessName -ErrorAction SilentlyContinue | Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
    if (-not $p) { break }
    $bmp = [WinShot]::Capture($p.MainWindowHandle)
    if ($bmp) {
        $w = $bmp.Width; $h = $bmp.Height
        $rect = New-Object System.Drawing.Rectangle(
            [int]($w * $CropLeft), [int]($h * $CropTop),
            [int]($w * ($CropRight - $CropLeft)), [int]($h * ($CropBottom - $CropTop)))
        $crop = $bmp.Clone($rect, $bmp.PixelFormat)
        $crop.Save((Join-Path $OutDir ("c{0:d2}.png" -f $i)), [System.Drawing.Imaging.ImageFormat]::Png)
        $crop.Dispose()
        $bmp.Dispose()
    }
    Start-Sleep -Milliseconds $IntervalMs
}

Write-Output $OutDir

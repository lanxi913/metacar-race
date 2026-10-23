# 采集仿真画面若干帧，并裁剪左下角 HUD 的加速度读数区域拼图
param(
    [int]$Frames = 24,
    [int]$IntervalMs = 1200
)

$dir = Join-Path $env:TEMP "hudseq"
if (Test-Path $dir) { Remove-Item $dir -Recurse -Force }
New-Item -ItemType Directory -Path $dir | Out-Null

$w = 'C:\Users\lanxi\.agents\skills\computer-control\scripts\window.ps1'
& $w -Action minimize -Title "DeepSeek Harness" | Out-Null
Start-Sleep -Milliseconds 400
& $w -Action maximize -Title "SmartCar" | Out-Null
Start-Sleep -Seconds 1

for ($i = 1; $i -le $Frames; $i++) {
    & 'C:\Users\lanxi\.agents\skills\computer-control\scripts\screen.ps1' -Output (Join-Path $dir ("h{0:d2}.png" -f $i)) -Quiet | Out-Null
    Start-Sleep -Milliseconds $IntervalMs
}

& $w -Action restore -Title "DeepSeek Harness" | Out-Null
Write-Output $dir

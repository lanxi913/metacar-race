param([int]$Keep = 120)
$exe = 'D:\Program Files\元动力客户端\metapower_desktop.exe'
Start-Process -FilePath $exe -WorkingDirectory (Split-Path $exe)
Start-Sleep -Seconds $Keep

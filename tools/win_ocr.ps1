# Windows 自带 OCR（Windows.Media.Ocr）命令行桥
# 用法: powershell.exe -NoProfile -ExecutionPolicy Bypass -File win_ocr.ps1 -Path in.png -Out out.txt [-Lang zh-Hans-CN]
# 说明:
#   - 不依赖任何 pip 包，结果按行写入 UTF-8 文本，Python 侧读文件
#   - 刻意不走 StorageFile.GetFileFromPathAsync（UWP 文件访问白名单会拒掉普通目录），
#     改成 .NET 读字节 + InMemoryRandomAccessStream，任何路径都能用
param(
    [Parameter(Mandatory = $true)][string]$Path,
    [Parameter(Mandatory = $true)][string]$Out,
    [string]$Lang = ""
)

$ErrorActionPreference = "Stop"

Add-Type -AssemblyName System.Runtime.WindowsRuntime | Out-Null
$null = [Windows.Media.Ocr.OcrEngine, Windows.Foundation, ContentType = WindowsRuntime]
$null = [Windows.Graphics.Imaging.BitmapDecoder, Windows.Foundation, ContentType = WindowsRuntime]
$null = [Windows.Storage.Streams.InMemoryRandomAccessStream, Windows.Foundation, ContentType = WindowsRuntime]
$null = [Windows.Storage.Streams.DataWriter, Windows.Foundation, ContentType = WindowsRuntime]
$null = [Windows.Globalization.Language, Windows.Foundation, ContentType = WindowsRuntime]

$asTaskGeneric = ([System.WindowsRuntimeSystemExtensions].GetMethods() | Where-Object {
        $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and
        $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1'
    })[0]

function Await($op, $resultType) {
    $asTask = $asTaskGeneric.MakeGenericMethod($resultType)
    $task = $asTask.Invoke($null, @($op))
    try {
        $task.Wait(-1) | Out-Null
    } catch {
        $inner = $_.Exception
        while ($inner.InnerException) { $inner = $inner.InnerException }
        Write-Error ("Await failed: " + $inner.Message)
        throw
    }
    $task.Result
}

# 选语言：显式 > 用户配置 > 第一可用
$engine = $null
if ($Lang -ne "") {
    try {
        $language = New-Object Windows.Globalization.Language $Lang
        $engine = [Windows.Media.Ocr.OcrEngine]::TryCreateFromLanguage($language)
    } catch { $engine = $null }
}
if ($null -eq $engine) {
    $engine = [Windows.Media.Ocr.OcrEngine]::TryCreateFromUserProfileLanguages()
}
if ($null -eq $engine) {
    Set-Content -Path $Out -Value "" -Encoding UTF8
    Write-Error "No OCR engine available (language pack missing?)"
    exit 2
}

# 图片 → 内存流 → SoftwareBitmap
$bytes = [System.IO.File]::ReadAllBytes($Path)
$stream = New-Object Windows.Storage.Streams.InMemoryRandomAccessStream
$writer = New-Object Windows.Storage.Streams.DataWriter $stream
$writer.WriteBytes($bytes)
Await ($writer.StoreAsync()) ([uint32]) | Out-Null
$writer.DetachStream() | Out-Null
$stream.Seek(0) | Out-Null

$decoder = Await ([Windows.Graphics.Imaging.BitmapDecoder]::CreateAsync($stream)) ([Windows.Graphics.Imaging.BitmapDecoder])
$bitmap = Await ($decoder.GetSoftwareBitmapAsync()) ([Windows.Graphics.Imaging.SoftwareBitmap])
$result = Await ($engine.RecognizeAsync($bitmap)) ([Windows.Media.Ocr.OcrResult])

$lines = @()
foreach ($line in $result.Lines) { $lines += $line.Text }
Set-Content -Path $Out -Value ($lines -join "`n") -Encoding UTF8
Write-Output ("lines=" + $lines.Count)

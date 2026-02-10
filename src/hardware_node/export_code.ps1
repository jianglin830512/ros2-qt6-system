# ===== 工程根目录 =====
$root = "E:\WGS\SRC\HARDWARE_NODE"
$output = "$root\ALL_CODE_UPLOAD.txt"

# 删除旧文件
if (Test-Path $output) { Remove-Item $output }

# 收集全部文本
$allText = New-Object System.Text.StringBuilder

$extensions = "*.cpp","*.hpp","*.h","*.yaml","*.yml","*.txt"

Get-ChildItem -Path $root -Recurse -Include $extensions | ForEach-Object {

    $file = $_.FullName

    $allText.AppendLine("============================================================") | Out-Null
    $allText.AppendLine("FILE: $file") | Out-Null
    $allText.AppendLine("============================================================") | Out-Null
    $allText.AppendLine("") | Out-Null

    Get-Content $file | ForEach-Object {
        $allText.AppendLine($_) | Out-Null
    }

    $allText.AppendLine("`n") | Out-Null
}

# ⭐关键：一次性UTF8写入
[System.IO.File]::WriteAllText($output, $allText.ToString(), [System.Text.Encoding]::UTF8)

Write-Host ""
Write-Host "✅ 已生成 UTF8 文件:"
Write-Host $output
Pause

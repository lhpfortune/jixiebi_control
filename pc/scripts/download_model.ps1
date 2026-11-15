$ErrorActionPreference = "Stop"
$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$ModelDir = Join-Path $ProjectRoot "models"
$ModelPath = Join-Path $ModelDir "hand_landmarker.task"
$Url = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
New-Item -ItemType Directory -Path $ModelDir -Force | Out-Null
if ((Test-Path -LiteralPath $ModelPath) -and ((Get-Item -LiteralPath $ModelPath).Length -gt 1000000)) {
    Write-Host "[model] Already present: $ModelPath"
    exit 0
}
Write-Host "[model] Downloading official MediaPipe HandLandmarker model"
Invoke-WebRequest -Uri $Url -UseBasicParsing -OutFile $ModelPath
$size = (Get-Item -LiteralPath $ModelPath).Length
if ($size -lt 1000000) {
    throw "Model download is incomplete ($size bytes)"
}
Write-Host "[model] Saved $ModelPath ($size bytes)"

param(
    [string]$Message = "MATLAB: guncelleme"
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

git add -A
$changes = git status --porcelain

if (-not $changes) {
    Write-Host "Commit edilecek degisiklik yok."
    exit 0
}

git status
git commit -m $Message
git push

Write-Host "`nTamam: https://github.com/Alp2246/matlab-fmcw-isac-examples"

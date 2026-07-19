# Backs build.bat. Drop one or more .cpp files onto build.bat to compile just
# those; run with no arguments to build everything (make build).

param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Files
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

if (-not $Files -or $Files.Count -eq 0) {
    Write-Host "No file dropped - building everything (make build)..."
    make build
    exit $LASTEXITCODE
}

$exitCode = 0
foreach ($f in $Files) {
    $name = [System.IO.Path]::GetFileNameWithoutExtension($f)
    $ext = [System.IO.Path]::GetExtension($f)

    if ($ext -ne '.cpp') {
        Write-Host "Skipping '$f' - expected a .cpp file"
        continue
    }
    if (-not (Test-Path (Join-Path $repoRoot "$name.cpp"))) {
        Write-Host "'$name.cpp' not found in the repo root - copy/move it next to the Makefile first."
        $exitCode = 1
        continue
    }

    Write-Host "Building $name.cpp ..."
    make "plugins/$name.o"
    if ($LASTEXITCODE -ne 0) {
        Write-Host "  FAILED"
        $exitCode = $LASTEXITCODE
    } else {
        Write-Host "  OK -> plugins/$name.o"
    }
}
exit $exitCode

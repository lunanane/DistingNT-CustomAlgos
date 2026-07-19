# Backs push.bat. Drop one or more .cpp/.o files onto push.bat to build
# (if needed) and push them to a connected disting NT over MIDI SysEx.
# With no arguments, prompts for a filename. The MIDI port is auto-detected
# by nt_push.py (looks for a port name containing "disting").

param(
    [Parameter(Position = 0, ValueFromRemainingArguments = $true)]
    [string[]]$Files
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

if (-not $Files -or $Files.Count -eq 0) {
    $answer = Read-Host "Enter the .cpp or .o file to push (relative to repo root)"
    if ([string]::IsNullOrWhiteSpace($answer)) {
        Write-Host "No file given."
        exit 1
    }
    $Files = @($answer)
}

$toPush = @()
foreach ($f in $Files) {
    $name = [System.IO.Path]::GetFileNameWithoutExtension($f)
    $ext = [System.IO.Path]::GetExtension($f)

    if ($ext -eq '.cpp') {
        if (-not (Test-Path (Join-Path $repoRoot "$name.cpp"))) {
            Write-Host "'$name.cpp' not found in the repo root - copy/move it next to the Makefile first."
            continue
        }
        Write-Host "Building $name.cpp ..."
        make "plugins/$name.o"
        if ($LASTEXITCODE -ne 0) {
            Write-Host "  build FAILED for $name.cpp - skipping"
            continue
        }
        $toPush += "plugins/$name.o"
    } elseif ($ext -eq '.o') {
        $toPush += $f
    } else {
        Write-Host "Skipping '$f' - expected a .cpp or .o file"
    }
}

if ($toPush.Count -eq 0) {
    Write-Host "Nothing to push."
    exit 1
}

$pythonCmd = @('python3', 'python') |
    ForEach-Object { Get-Command $_ -ErrorAction SilentlyContinue } |
    Where-Object { $_.Source -notlike '*WindowsApps*' } |
    Select-Object -First 1 -ExpandProperty Source
if (-not $pythonCmd) {
    throw "No Python 3 found on PATH - run install.bat first."
}

$pushArgs = @('tools/nt_push.py') + $toPush

& $pythonCmd @pushArgs
exit $LASTEXITCODE

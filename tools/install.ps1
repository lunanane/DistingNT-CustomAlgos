# Sets up the full build environment for disting NT custom algorithms:
# ARM GNU Toolchain, GNU Make, and the Python deps for tools/nt_push.py.
# Safe to re-run - already-installed pieces are detected and skipped.

$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'

$repoRoot = Split-Path -Parent $PSScriptRoot
$toolsDir = Join-Path $env:LOCALAPPDATA 'Programs'

$armVersion = '10.3-2021.10'
$armRoot    = Join-Path $toolsDir 'gcc-arm-none-eabi'
$armBin     = Join-Path $armRoot "gcc-arm-none-eabi-$armVersion\bin"
$armUrl     = 'https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-win32.zip'
$armMd5     = '2bc8f0c4c4659f8259c8176223eeafc1'

$makeRoot = Join-Path $toolsDir 'make'
$makeBin  = Join-Path $makeRoot 'bin'
$makeUrl  = 'https://sourceforge.net/projects/ezwinports/files/make-4.4.1-without-guile-w32-bin.zip/download'

function Write-Step($msg) {
    Write-Host ""
    Write-Host "==> $msg" -ForegroundColor Cyan
}

function Test-CommandExists($name) {
    return [bool](Get-Command $name -ErrorAction SilentlyContinue)
}

function Install-ZipTool {
    param($Name, $Url, $DestRoot, $MarkerRelPath, $ExpectedMd5)

    $marker = Join-Path $DestRoot $MarkerRelPath
    if (Test-Path $marker) {
        Write-Host "  $Name already installed at $DestRoot"
        return
    }

    Write-Host "  downloading $Name..."
    $zipPath = Join-Path $env:TEMP "$Name-install.zip"
    Invoke-WebRequest -Uri $Url -OutFile $zipPath

    if ($ExpectedMd5) {
        $actual = (Get-FileHash -Path $zipPath -Algorithm MD5).Hash.ToLower()
        if ($actual -ne $ExpectedMd5.ToLower()) {
            Remove-Item $zipPath -Force
            throw "$Name download checksum mismatch (got $actual, expected $ExpectedMd5)"
        }
    }

    Write-Host "  extracting $Name..."
    New-Item -ItemType Directory -Force -Path $DestRoot | Out-Null
    Expand-Archive -Path $zipPath -DestinationPath $DestRoot -Force
    Remove-Item $zipPath -Force
}

Write-Step "Initializing git submodules"
if (-not (Test-CommandExists 'git')) {
    throw "git not found on PATH - install Git for Windows first."
}
Push-Location $repoRoot
try {
    # Non-recursive: distingNT_API/include is all the build needs. Its own
    # nested airwindows submodule is unrelated to building custom algorithms
    # and its long file paths fail to clone on Windows without core.longpaths.
    git submodule update --init
} finally {
    Pop-Location
}
$apiHeader = Join-Path $repoRoot 'distingNT_API\include\distingnt\api.h'
if (Test-Path $apiHeader) {
    Write-Host "  distingNT_API present at $repoRoot\distingNT_API"
} else {
    throw "distingNT_API/include/distingnt/api.h still missing after submodule update"
}

Write-Step "ARM GNU Toolchain ($armVersion)"
if (Test-CommandExists 'arm-none-eabi-c++') {
    Write-Host "  already on PATH"
} else {
    Install-ZipTool -Name 'gcc-arm-none-eabi' -Url $armUrl -DestRoot $armRoot `
        -MarkerRelPath "gcc-arm-none-eabi-$armVersion\bin\arm-none-eabi-c++.exe" -ExpectedMd5 $armMd5
}

Write-Step "GNU Make"
if (Test-CommandExists 'make') {
    Write-Host "  already on PATH"
} else {
    Install-ZipTool -Name 'make' -Url $makeUrl -DestRoot $makeRoot -MarkerRelPath 'bin\make.exe'
}

Write-Step "Git's usr\bin (sh.exe + coreutils needed by Make's recipes)"
# GNU Make's Windows port only runs recipes with real POSIX shell syntax
# (mkdir -p, rm -f, [ -f ... ]) if it can find sh.exe on PATH at startup;
# otherwise it silently falls back to cmd.exe and recipes fail to parse.
# The Makefile's recipes also shell out to mkdir/rm directly, which only
# exist (as real .exe coreutils) under usr\bin, not the slimmer bin\
# compat folder - so that's the one to add. Git for Windows already
# bundles all of this, so reuse it instead of adding yet another dependency.
$gitShBin = $null
$gitCmd = Get-Command git -ErrorAction SilentlyContinue
if ($gitCmd) {
    # git.exe can resolve to <root>\cmd\git.exe or <root>\mingw64\bin\git.exe
    # depending on PATH order, which sit at different depths under the Git
    # root - so walk upward looking for <ancestor>\usr\bin\sh.exe instead of
    # assuming a fixed number of parent levels.
    $dir = Split-Path -Parent $gitCmd.Source
    for ($i = 0; $i -lt 5 -and $dir; $i++) {
        $candidate = Join-Path $dir 'usr\bin'
        if (Test-Path (Join-Path $candidate 'sh.exe')) {
            $gitShBin = $candidate
            break
        }
        $dir = Split-Path -Parent $dir
    }
}
if (-not $gitShBin) {
    throw "Could not find sh.exe under the Git for Windows install - Make's recipes need it. Reinstall Git for Windows and re-run this script."
}
if (Get-Command sh -ErrorAction SilentlyContinue) {
    Write-Host "  sh.exe already on PATH"
} else {
    Write-Host "  found at $gitShBin"
}

Write-Step "Updating user PATH"
$userPath = [Environment]::GetEnvironmentVariable('PATH', 'User')
$parts = $userPath -split ';' | Where-Object { $_ -ne '' }
$candidates = @($armBin, $makeBin, $gitShBin) | Where-Object { $_ -and (Test-Path $_) -and ($parts -notcontains $_) }
if ($candidates.Count -gt 0) {
    $newPath = ($parts + $candidates) -join ';'
    [Environment]::SetEnvironmentVariable('PATH', $newPath, 'User')
    Write-Host "  added: $($candidates -join ', ')"
} else {
    Write-Host "  already up to date"
}
foreach ($p in @($armBin, $makeBin, $gitShBin)) {
    if ($p -and (Test-Path $p) -and ($env:PATH -notlike "*$p*")) {
        $env:PATH += ";$p"
    }
}

Write-Step "Python dependencies for the push tool"
# Windows' "App Execution Alias" stubs (WindowsApps\python.exe etc.) resolve
# as a command but just print a Store-install nag instead of running Python,
# so real installs under WindowsApps must be excluded here.
$pythonCmd = @('python3', 'python') |
    ForEach-Object { Get-Command $_ -ErrorAction SilentlyContinue } |
    Where-Object { $_.Source -notlike '*WindowsApps*' } |
    Select-Object -First 1 -ExpandProperty Source
if (-not $pythonCmd) {
    Write-Host "  WARNING: no Python 3 found on PATH." -ForegroundColor Yellow
    Write-Host "  Install it from https://www.python.org/downloads/ then run:"
    Write-Host "    pip install -r `"$repoRoot\tools\requirements.txt`""
} else {
    & $pythonCmd -m pip install --quiet -r (Join-Path $repoRoot 'tools\requirements.txt')
}

Write-Step "Verifying"
& "$armBin\arm-none-eabi-c++.exe" --version | Select-Object -First 1
& "$makeBin\make.exe" --version | Select-Object -First 1
if ($pythonCmd) { & $pythonCmd --version }

Write-Host ""
Write-Host "Setup complete." -ForegroundColor Green
Write-Host "Open a NEW terminal window (so the PATH change is picked up), then:"
Write-Host "  cd `"$repoRoot`""
Write-Host "  make build"

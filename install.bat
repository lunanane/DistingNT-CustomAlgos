@echo off
rem Sets up the full build environment for disting NT custom algorithms
rem (ARM toolchain, GNU Make, Python deps). Safe to re-run.

setlocal
set "SCRIPT_DIR=%~dp0"

powershell -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%tools\install.ps1"
set "EXIT_CODE=%ERRORLEVEL%"

echo.
if not "%EXIT_CODE%"=="0" (
    echo Install script exited with an error ^(code %EXIT_CODE%^).
)

pause
exit /b %EXIT_CODE%

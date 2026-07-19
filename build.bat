@echo off
rem Drag .cpp file(s) onto this .bat to build just those.
rem Double-click with nothing dropped to build everything (make build).

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\build.ps1" %*
echo.
pause

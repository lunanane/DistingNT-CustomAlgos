@echo off
rem Drag .cpp/.o file(s) onto this .bat to build (if needed) and push them
rem to a connected disting NT over MIDI SysEx - the module is auto-detected.
rem Double-click with nothing dropped to be prompted for a filename.

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\push.ps1" %*
echo.
pause

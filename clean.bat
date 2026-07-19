@echo off
rem Removes all built plugin .o files.

cd /d "%~dp0"
make clean
echo.
pause

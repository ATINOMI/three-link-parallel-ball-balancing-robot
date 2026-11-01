@echo off
cd /d "%~dp0"
if not exist "build-attitude\dualforge_attitude_web.exe" (
  powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0build_attitude.ps1"
  if errorlevel 1 (pause & exit /b 1)
)
"%~dp0build-attitude\dualforge_attitude_web.exe"
if errorlevel 1 pause

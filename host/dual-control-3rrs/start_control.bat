@echo off
cd /d "%~dp0"
if not exist "build-input-v2\dual_control_3rrs.exe" (
  echo Please run build.ps1 first.
  pause
  exit /b 1
)
"build-input-v2\dual_control_3rrs.exe" %*
if errorlevel 1 pause

@echo off
setlocal

set "PROJECT_ROOT=%~dp0"
set "PYTHON=%LOCALAPPDATA%\Python\bin\python.exe"

if not exist "%PYTHON%" (
  echo Python runtime not found: %PYTHON%
  exit /b 1
)

cd /d "%PROJECT_ROOT%"
"%PYTHON%" serve_web.py --open %*

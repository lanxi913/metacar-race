@echo off
rem ASCII only: cmd.exe reads this file with the OEM codepage.
rem Double-click friendly launcher for tools\collect_scene.py
chcp 65001 >nul
set PYTHONIOENCODING=utf-8
set PYTHONUTF8=1
"%~dp0..\.venv\Scripts\python.exe" -u "%~dp0collect_scene.py" %*
set RC=%ERRORLEVEL%
if "%~1"=="" pause
exit /b %RC%

@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"
if exist "python_env\python.exe" (
    "python_env\python.exe" -s "main.py" %*
) else if exist "python_env\Scripts\python.exe" (
    "python_env\Scripts\python.exe" "main.py" %*
) else (
    python "main.py" %*
)
if not "%ERRORLEVEL%"=="0" pause
exit /b %ERRORLEVEL%


@echo off
title Bo Dieu Khien Drone AirSim (WASD)
chcp 65001 >nul
set PYTHONIOENCODING=utf-8
set PYTHONUTF8=1
cd /d "%~dp0"
python -c "import keyboard" 2>nul || pip install keyboard >nul 2>&1
python "%~dp0dieu_khien_drone.py"
if errorlevel 1 pause

@echo off
setlocal enabledelayedexpansion
title KHOI DONG MO PHONG AFRICA AIRSIM (UE 4.27)
chcp 65001 >nul
cd /d "%~dp0"

echo ========================================================
echo   KHOI DONG MOI TRUONG AFRICA AIRSIM (UNREAL ENGINE 4.27)
echo ========================================================

set "SETTINGS_FILE=%~dp0settings.json"
set "AFRICA_DIR=%~dp0..\Africa\Africa\WindowsNoEditor"
set "AFRICA_EXE=%AFRICA_DIR%\Africa_001.exe"

if not exist "%AFRICA_EXE%" (
    if exist "D:\Simulator\Env\Africa\Africa\WindowsNoEditor\Africa_001.exe" (
        set "AFRICA_DIR=D:\Simulator\Env\Africa\Africa\WindowsNoEditor"
        set "AFRICA_EXE=D:\Simulator\Env\Africa\Africa\WindowsNoEditor\Africa_001.exe"
    )
)

if not exist "%AFRICA_EXE%" (
    echo [LOI] Khong tim thay file mo phong tai: "%AFRICA_EXE%"
    pause
    exit /b 1
)

echo [1/3] Dong bo cau hinh settings.json (FOV 90, FLIR Thermal IR) vao he thong...
if not exist "C:\AirSim" mkdir "C:\AirSim"
copy /y "%SETTINGS_FILE%" "C:\AirSim\settings.json" >nul
if not exist "%USERPROFILE%\Documents\AirSim" mkdir "%USERPROFILE%\Documents\AirSim"
copy /y "%SETTINGS_FILE%" "%USERPROFILE%\Documents\AirSim\settings.json" >nul
copy /y "%SETTINGS_FILE%" "%AFRICA_DIR%\settings.json" >nul

echo [2/3] Khoi chay cua so 3D Unreal Africa UE 4.27...
taskkill /F /IM Africa_001.exe 2>nul
timeout /t 1 /nobreak >nul

echo [THONG BAO] Dang mo cua so 3D Africa UE 4.27 (thu nho xuong taskbar de khong de GCS)...
cd /d "%AFRICA_DIR%"
start "" /min Africa_001.exe -settings="C:\AirSim\settings.json" -windowed -WinX=0 -WinY=0 -ResX=640 -ResY=360 -USEALLAVAILABLECORES -FPS=30
cd /d "%~dp0"

:CALIBRATE_TARGETS
echo [3/3] Dang kich hoat tien trinh tao nguoi va hieu chuan Camera Nhiet FLIR...
start "" /b python "%~dp0setup_africa_targets.py"

echo.
echo ========================================================
echo   [HOAN TAT] Ban do Africa da mo va he thong san sang!
echo   + Cua so Unreal chay thu nho (minimized), ViewMode: NoDisplay
echo   + Tat toan bo Subwindows vien xanh la de tranh de len GCS
echo   + LiDAR Ouster OS1-64 (64 kenh, tam 120m, 1.31M diem/s)
echo   + Camera Nhiet FLIR & RGB dong tam (FOV 50 do, 640x512)
echo   + 4 Muc tieu nguoi (Nan nhan cuu ho) da duoc thiet lap
echo ========================================================

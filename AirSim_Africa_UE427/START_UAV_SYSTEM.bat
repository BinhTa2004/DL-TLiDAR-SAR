@echo off
setlocal enabledelayedexpansion
title KHOI DONG TOAN BO HE THONG UAV (UE 4.27 AFRICA)
chcp 65001 >nul
set PYTHONIOENCODING=utf-8
set PYTHONUTF8=1
cd /d "%~dp0"

echo ========================================================
echo   KHOI DONG HE THONG UAV VA TRAM GCS DA CAM BIEN
echo   Ban do Africa (Unreal Engine 4.27) + AirSim 1.8.1
echo ========================================================
echo.

:: 1. Dong bo settings.json vao tat ca cac vi tri can thiet
echo [1/3] Dong bo cau hinh settings.json vao he thong...
if not exist "C:\AirSim" mkdir "C:\AirSim"
copy /y "%~dp0settings.json" "C:\AirSim\settings.json" >nul
if not exist "%USERPROFILE%\Documents\AirSim" mkdir "%USERPROFILE%\Documents\AirSim"
copy /y "%~dp0settings.json" "%USERPROFILE%\Documents\AirSim\settings.json" >nul
set "AFRICA_DIR=%~dp0..\Africa\Africa\WindowsNoEditor"
if not exist "%AFRICA_DIR%\Africa_001.exe" (
    if exist "D:\Simulator\Env\Africa\Africa\WindowsNoEditor\Africa_001.exe" (
        set "AFRICA_DIR=D:\Simulator\Env\Africa\Africa\WindowsNoEditor"
    )
)
copy /y "%~dp0settings.json" "%AFRICA_DIR%\settings.json" >nul

:: 2. Kiem tra va khoi chay ban do 3D Africa
echo [2/3] Khoi chay cua so 3D Unreal Africa UE 4.27...
taskkill /F /IM Africa_001.exe 2>nul
timeout /t 1 /nobreak >nul

cd /d "%AFRICA_DIR%"
:: Unreal chay thu nho (minimized) + ViewMode NoDisplay trong settings.json -> chi con 1 cua so GCS Dashboard
start "" /min Africa_001.exe -settings="C:\AirSim\settings.json" -windowed -WinX=0 -WinY=0 -ResX=640 -ResY=360 -USEALLAVAILABLECORES -FPS=30
cd /d "%~dp0"

:WAIT_CONNECTION
echo.
echo [3/3] Dang cho ban do 3D Africa khoi dong va mo cong 41451...
set wait_sec=0

:WAIT_LOOP
ping 127.0.0.1 -n 2 >nul
set /a wait_sec+=1
netstat -ano | find "41451" | find "LISTENING" >nul
if not errorlevel 1 goto LAUNCH_GCS
if !wait_sec! geq 35 goto TIMEOUT_GCS
goto WAIT_LOOP

:TIMEOUT_GCS
echo.
echo [CANH BAO] Chua thay cong 41451 mo sau !wait_sec! giay. Van thu khoi dong GCS Station...
goto LAUNCH_GCS

:LAUNCH_GCS
echo.
echo [THIET LAP] Dang tao muc tieu nguoi va hieu chuan Camera Nhiet FLIR...
python "%~dp0setup_africa_targets.py"
echo.
echo [KHOI CHAY] Dang mo Tram dieu khien GCS Station Tang Duoi...
python "%~dp0uav_gcs_station.py"
echo.
echo ========================================================
echo   He thong da ket thuc phien lam viec.
echo ========================================================
pause

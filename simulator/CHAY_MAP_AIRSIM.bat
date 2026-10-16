@echo off
title Trinh Khoi Dong Map AirSim
chcp 65001 >nul
set PYTHONIOENCODING=utf-8
set PYTHONUTF8=1
cd /d "%~dp0"

echo ========================================================
echo        TRÌNH KHỞI ĐỘNG MAP AIRSIM ^& BỘ ĐIỀU KHIỂN
echo ========================================================
echo.
echo [1] Dong bo cau hinh settings.json vao he thong...
if not exist "C:\AirSim" mkdir "C:\AirSim"
copy /y "%~dp0settings.json" "C:\AirSim\settings.json" >nul
if not exist "%USERPROFILE%\Documents\AirSim" mkdir "%USERPROFILE%\Documents\AirSim"
copy /y "%~dp0settings.json" "%USERPROFILE%\Documents\AirSim\settings.json" >nul

echo.
echo Danh sach ban do co san:
echo   [1] Africa_001
echo   [2] LandscapeMountains
echo   [3] ZhangJiajie
echo.
set /p map_choice="Chon map muon mo [1-3] (Nhan Enter mac dinh chon 1): "
if "%map_choice%"=="" set map_choice=1

if "%map_choice%"=="1" (
    if exist "%~dp0map\Africa\Africa\WindowsNoEditor\Africa_001.exe" (
        set "EXE_PATH=%~dp0map\Africa\Africa\WindowsNoEditor\Africa_001.exe"
        set "MAP_DIR=%~dp0map\Africa\Africa\WindowsNoEditor"
    ) else (
        set "EXE_PATH=%~dp0Africa\Africa\WindowsNoEditor\Africa_001.exe"
        set "MAP_DIR=%~dp0Africa\Africa\WindowsNoEditor"
    )
)
if "%map_choice%"=="2" (
    if exist "%~dp0map\LandscapeMountains\LandscapeMountains\WindowsNoEditor\LandscapeMountains.exe" (
        set "EXE_PATH=%~dp0map\LandscapeMountains\LandscapeMountains\WindowsNoEditor\LandscapeMountains.exe"
        set "MAP_DIR=%~dp0map\LandscapeMountains\LandscapeMountains\WindowsNoEditor"
    ) else (
        set "EXE_PATH=%~dp0LandscapeMountains\LandscapeMountains\WindowsNoEditor\LandscapeMountains.exe"
        set "MAP_DIR=%~dp0LandscapeMountains\LandscapeMountains\WindowsNoEditor"
    )
)
if "%map_choice%"=="3" (
    if exist "%~dp0map\ZhangJiajie\ZhangJiajie\WindowsNoEditor\ZhangJiajie.exe" (
        set "EXE_PATH=%~dp0map\ZhangJiajie\ZhangJiajie\WindowsNoEditor\ZhangJiajie.exe"
        set "MAP_DIR=%~dp0map\ZhangJiajie\ZhangJiajie\WindowsNoEditor"
    ) else (
        set "EXE_PATH=%~dp0ZhangJiajie\ZhangJiajie\WindowsNoEditor\ZhangJiajie.exe"
        set "MAP_DIR=%~dp0ZhangJiajie\ZhangJiajie\WindowsNoEditor"
    )
)

copy /y "%~dp0settings.json" "%MAP_DIR%\settings.json" >nul

echo [2] Dang mo ban do 3D o che do Cua so (1280x720)...
cd /d "%MAP_DIR%"
start "" "%EXE_PATH%" -settings="C:\AirSim\settings.json" -windowed -ResX=1280 -ResY=720 -USEALLAVAILABLECORES
cd /d "%~dp0"

echo [3] Dang cho ban do 3D khoi dong (5 giay)...
timeout /t 5 /nobreak >nul
python -c "import keyboard" 2>nul || pip install keyboard >nul 2>&1
python "%~dp0dieu_khien_drone.py"
pause

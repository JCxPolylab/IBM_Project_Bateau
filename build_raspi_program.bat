@echo off
setlocal EnableExtensions EnableDelayedExpansion

rem =======================
rem  CONFIG A ADAPTER
rem =======================
set "CONFIG_FILE=%PROJECT_ROOT%devConfig.txt"

rem Options de build
set "BUILD_TYPE=Release"
set "USE_ORT=OFF"
set "USE_RPLIDAR=ON"

rem =======================
rem  CHECK OUTILS
rem =======================
where ssh >nul 2>&1
if errorlevel 1 (
  echo [ERROR] ssh introuvable. Active "OpenSSH Client" sur Windows.
  exit /b 1
)

REM ======= CHECKS =======
if not exist "%CONFIG_FILE%" (
  echo [ERROR] Fichier config introuvable: "%CONFIG_FILE%"
  pause
  exit /b 1
)

set /p "CONFIG_MODE="<"%CONFIG_FILE%"

if /i "%CONFIG_MODE%"=="EPF" (
    echo Configuration EPF
    set "REMOTE_DIR=/home/ibm_bateau/code/jerryCamera"
    set "PI_USER=ibm_bateau"
    set "PI_HOST=10.150.74.211"
    set "PI_PORT=22"
) else (
    echo Configuration inconnue ou autre valeur
    set "REMOTE_DIR=~/camera_project/venv/code/IBM_Bateau"
    set "PI_USER=jerrycrozet"
    set "PI_HOST=raspberrypi"
    set "PI_PORT=22"
)

echo.
echo ===== Remote build on %PI_USER%@%PI_HOST%:%REMOTE_DIR% =====
echo.

rem =======================
rem  BUILD REMOTE
rem =======================
::ssh -p %PI_PORT% %PI_USER%@%PI_HOST% ^
  ::"chmod -R u+rwx %REMOTE_DIR% && cd %REMOTE_DIR% && rm -rf build && cmake -S . -B build -DCMAKE_BUILD_TYPE=%BUILD_TYPE% -DUSE_RPLIDAR_SDK=%USE_RPLIDAR% -DUSE_ORT=%USE_ORT% && cmake --build build --parallel"

ssh -p %PI_PORT% %PI_USER%@%PI_HOST% ^
  "cd %REMOTE_DIR% && rm -rf build && cmake -S . -B build -DCMAKE_BUILD_TYPE=%BUILD_TYPE% -DUSE_RPLIDAR_SDK=%USE_RPLIDAR% -DUSE_ORT=%USE_ORT% && cmake --build build -j$(nproc)"

if errorlevel 1 (
  echo.
  echo [ERROR] Build failed.
  pause
  exit /b 2
)

echo.
echo [OK] Build done.
pause
exit /b 0
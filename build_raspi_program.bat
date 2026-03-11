@echo off
setlocal EnableExtensions EnableDelayedExpansion

rem =======================
rem  CONFIG A ADAPTER
rem =======================
set "PI_USER=jerrycrozet"
set "PI_HOST=raspberrypi"
set "PI_PORT=22"

rem Chemin du projet SUR la Raspberry Pi (dossier qui contient le CMakeLists.txt)
set "REMOTE_DIR=~/camera_project/venv/code/IBM_Bateau"

rem Options de build
set "BUILD_TYPE=Release"
set "USE_ORT=ON"

rem =======================
rem  CHECK OUTILS
rem =======================
where ssh >nul 2>&1
if errorlevel 1 (
  echo [ERROR] ssh introuvable. Active "OpenSSH Client" sur Windows.
  exit /b 1
)

echo.
echo ===== Remote build on %PI_USER%@%PI_HOST%:%REMOTE_DIR% =====
echo.

rem =======================
rem  BUILD REMOTE
rem =======================
ssh -p %PI_PORT% %PI_USER%@%PI_HOST% ^
  "cd %REMOTE_DIR% && rm -rf build && cmake -S . -B build -DCMAKE_BUILD_TYPE=%BUILD_TYPE% -DUSE_ORT=%USE_ORT% && cmake --build build --parallel"

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
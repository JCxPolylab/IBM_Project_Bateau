@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "BAT_DIR=%~dp0"
call "%BAT_DIR%_load_raspi_config.bat"
if errorlevel 1 (
    pause
    exit /b 1
)

where ssh >nul 2>&1
if errorlevel 1 (
    echo [ERROR] ssh introuvable. Active "OpenSSH Client" sur Windows.
    pause
    exit /b 1
)

echo.
echo ===== Remote build on %PI_USER%@%PI_HOST%:%REMOTE_DIR% =====
echo BUILD_TYPE  : %BUILD_TYPE%
echo USE_ORT     : %USE_ORT%
echo USE_RPLIDAR : %USE_RPLIDAR%
echo PROGRAM     : %PROGRAM_NAME%
echo.

ssh -p "%PI_PORT%" "%PI_USER%@%PI_HOST%" ^
  "cd '%REMOTE_DIR%' && rm -rf build && cmake -S . -B build -DCMAKE_BUILD_TYPE=%BUILD_TYPE% -DUSE_RPLIDAR_SDK=%USE_RPLIDAR% -DUSE_ORT=%USE_ORT% && cmake --build build -j$(nproc)"

if errorlevel 1 (
    echo.
    echo [ERROR] Build failed.
    pause
    exit /b 2
)

echo.
echo [OK] Build done.
pause
endlocal
exit /b 0

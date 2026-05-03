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
echo ===== Execution remote =====
echo Cible     : %PI_USER%@%PI_HOST%
echo Projet    : %REMOTE_DIR%
echo Programme : %REMOTE_DIR%/build/%PROGRAM_NAME%
echo.

ssh -t -p "%PI_PORT%" "%PI_USER%@%PI_HOST%" "cd '%REMOTE_DIR%' && './build/%PROGRAM_NAME%'"

if errorlevel 1 (
    echo.
    echo [ERROR] Le programme distant s'est termine avec une erreur.
    pause
    exit /b 1
)

pause
endlocal
exit /b 0

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
echo ======= CONNEXION SSH =======
echo Mode          : %CONFIG_MODE%
echo User          : %PI_USER%
echo Host          : %PI_HOST%
echo Port          : %PI_PORT%
echo Racine locale : %PROJECT_ROOT%
echo Remote dir    : %REMOTE_DIR%
echo =============================
echo.

ssh -t -p "%PI_PORT%" "%PI_USER%@%PI_HOST%" "mkdir -p '%REMOTE_DIR%' && cd '%REMOTE_DIR%' && exec bash -l"

pause
endlocal
exit /b 0

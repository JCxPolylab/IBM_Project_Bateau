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

echo Ouverture du fichier config.ini sur la Raspberry Pi...
echo Cible : %PI_USER%@%PI_HOST%:%REMOTE_DIR%/parametres/config.ini

ssh -t -p "%PI_PORT%" "%PI_USER%@%PI_HOST%" "sudo nano '%REMOTE_DIR%/parametres/config.ini'"

pause
endlocal
exit /b 0

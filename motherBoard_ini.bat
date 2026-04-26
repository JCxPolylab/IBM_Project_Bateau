@echo off
setlocal

REM Dossier du script .bat
set "PROJECT_ROOT=%~dp0"
set "CONFIG_FILE=%PROJECT_ROOT%devConfig.txt"

REM ======= CHECKS =======
if not exist "%CONFIG_FILE%" (
    echo [ERROR] Fichier config introuvable : "%CONFIG_FILE%"
    pause
    exit /b 1
)

set /p "CONFIG_MODE="<"%CONFIG_FILE%"

if /i "%CONFIG_MODE%"=="EPF" (
    echo Configuration EPF
    set "PI_USER=ibm_bateau"
    set "PI_HOST=10.150.74.211"
    set "PI_PORT=22"
    set "REMOTE_DIR=/home/ibm_bateau/code/jerryCamera"
) else (
    echo Configuration locale
    set "PI_USER=jerrycrozet"
    set "PI_HOST=raspberrypi"
    set "PI_PORT=22"
    set "REMOTE_DIR=/home/jerrycrozet/camera_project/venv/code/IBM_Bateau"
)

echo Ouverture du fichier config.ini sur la Raspberry Pi...
ssh -t -p %PI_PORT% %PI_USER%@%PI_HOST% "sudo nano %REMOTE_DIR%/parametres/motherboard_trame.ini"

pause
endlocal
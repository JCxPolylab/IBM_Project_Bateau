echo off
set "CONFIG_FILE=%PROJECT_ROOT%devConfig.txt"

REM ======= CHECKS =======
if not exist "%CONFIG_FILE%" (
  echo [ERROR] Fichier config introuvable: "%CONFIG_FILE%"
  pause
  exit /b 1
)

set /p "CONFIG_MODE="<"%CONFIG_FILE%"

if /i "%CONFIG_MODE%"=="EPF" (
    echo Configuration EPF
    set "PI_USER=ibm_bateau"
    set "PI_HOST=10.150.74.211"
    set "PI_PORT=22"
) else (
    echo Configuration inconnue ou autre valeur
    set "PI_USER=jerrycrozet"
    set "PI_HOST=raspberrypi"
    set "PI_PORT=22"
)

ssh %PI_USER%@%PI_HOST% ~/camera_project/venv/code/IBM_Bateau/build/CATJ_ibmRobotProject

PAUSE




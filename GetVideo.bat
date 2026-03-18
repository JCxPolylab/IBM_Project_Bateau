@echo off
setlocal EnableExtensions

set "CONFIG_FILE=%PROJECT_ROOT%devConfig.txt"
set "defaultLocalDest=%~dp0"

REM ======= CHECKS =======
if not exist "%CONFIG_FILE%" (
  echo [ERROR] Fichier config introuvable: "%CONFIG_FILE%"
  pause
  exit /b 1
)

set /p "CONFIG_MODE="<"%CONFIG_FILE%"

if /i "%CONFIG_MODE%"=="EPF" (
    echo Configuration EPF
    set "ipAddress=raspiEPF"
    set "remoteUser=ibm_bateau"
    set "raspberryHome=/home/ibm_bateau/"
    set "remotePath=code/jerryCamera/parametres/video"
) else (
    echo Configuration inconnue ou autre valeur
    set "remoteUser=jerrycrozet"
    set "ipAddress=raspberrypi"
    set "raspberryHome=/home/jerrycrozet/"
    set "remotePath=camera_project/venv/code/IBM_Bateau/parametres/video"
)

where scp >nul 2>&1
if errorlevel 1 (
    echo [ERREUR] scp est introuvable.
    echo Verifie que le client OpenSSH est installe sur Windows.
    pause
    exit /b 1
)


set "sourcePath=%raspberryHome%%remotePath%"

set "type=D"
set "destLocal=%defaultLocalDest%parametres"

if not exist "%destLocal%" (
    echo Le dossier "%destLocal%" n'existe pas. Creation...
    mkdir "%destLocal%"
    if errorlevel 1 (
        echo [ERREUR] Impossible de creer le dossier local.
        pause
        exit /b 1
    )
)

echo.
echo Source distante : %remoteUser%@%ipAddress%:%sourcePath%
echo Destination locale : %destLocal%
echo.

    scp -r "%remoteUser%@%ipAddress%:%sourcePath%" "%destLocal%"

if errorlevel 1 (
    echo.
    echo [ERREUR] La copie a echoue.
    pause
    exit /b 1
)

echo.
echo Copie reussie !
pause
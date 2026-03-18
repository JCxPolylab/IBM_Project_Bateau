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
    set "ipAddress=10.150.74.211"
    set "remoteUser=ibm_bateau"
    set "raspberryHome=/home/ibm_bateau/"
) else (
    echo Configuration inconnue ou autre valeur
    set "remoteUser=jerrycrozet"
    set "ipAddress=raspberrypi"
    set "raspberryHome=/home/jerrycrozet/"
    REM set "raspberryHome=/home/jerrycrozet/camera_project/venv/code/IBM_Bateau"
)

where scp >nul 2>&1
if errorlevel 1 (
    echo [ERREUR] scp est introuvable.
    echo Verifie que le client OpenSSH est installe sur Windows.
    pause
    exit /b 1
)

echo.
set /p "remotePath=Entrez le chemin source sur la Raspberry Pi (ex: Documents/test.txt ou Code/projet) : "

if "%remotePath%"=="" (
    echo [ERREUR] Aucun chemin source renseigne.
    pause
    exit /b 1
)

:: Si le chemin commence deja par / ou ~, on le garde tel quel
if "%remotePath:~0,1%"=="/" (
    set "sourcePath=%remotePath%"
) else if "%remotePath:~0,1%"=="~" (
    set "sourcePath=%remotePath%"
) else (
    set "sourcePath=%raspberryHome%%remotePath%"
)

echo.
set /p "type=Est-ce un FICHIER ou un DOSSIER ? (F/D) : "
if /I not "%type%"=="F" if /I not "%type%"=="D" (
    echo [ERREUR] Reponse invalide. Tape F ou D.
    pause
    exit /b 1
)

echo.
set /p "destLocal=Entrez le dossier de destination sur le PC (laisser vide pour le dossier du script) : "

if "%destLocal%"=="" (
    set "destLocal=%defaultLocalDest%"
)

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

set /p "rep=Voulez-vous lancer la copie ? (O/N) : "
if /I not "%rep%"=="O" exit /b

echo.
if /I "%type%"=="D" (
    scp -r "%remoteUser%@%ipAddress%:%sourcePath%" "%destLocal%\"
) else (
    scp "%remoteUser%@%ipAddress%:%sourcePath%" "%destLocal%\"
)

if errorlevel 1 (
    echo.
    echo [ERREUR] La copie a echoue.
    pause
    exit /b 1
)

echo.
echo Copie reussie !
pause
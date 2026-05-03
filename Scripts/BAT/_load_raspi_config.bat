@echo off
REM ============================================================
REM  Charge devConfig.txt et expose les variables suivantes :
REM  CONFIG_MODE, PI_USER, PI_HOST, PI_PORT, REMOTE_HOME, REMOTE_DIR
REM  BUILD_TYPE, USE_ORT, USE_RPLIDAR, PROGRAM_NAME
REM
REM  Nouvelle arborescence supportee :
REM    IBM_Project_Bateau\
REM      CMakeLists.txt
REM      sources\
REM      parametres\
REM      Scripts\
REM        devConfig.txt
REM        folders_to_copy.txt
REM        BAT\
REM          *.bat
REM ============================================================

if not defined BAT_DIR set "BAT_DIR=%~dp0"
for %%I in ("%BAT_DIR%.") do set "BAT_DIR=%%~fI\"

REM Dossier Scripts : un niveau au-dessus de Scripts\BAT.
if not defined SCRIPTS_DIR (
    if exist "%BAT_DIR%..\devConfig.txt" (
        for %%I in ("%BAT_DIR%..") do set "SCRIPTS_DIR=%%~fI\"
    ) else (
        set "SCRIPTS_DIR=%BAT_DIR%"
    )
)

REM Racine projet : un niveau au-dessus de Scripts.
if not defined PROJECT_ROOT (
    if exist "%SCRIPTS_DIR%..\CMakeLists.txt" (
        for %%I in ("%SCRIPTS_DIR%..") do set "PROJECT_ROOT=%%~fI\"
    ) else if exist "%BAT_DIR%..\..\CMakeLists.txt" (
        for %%I in ("%BAT_DIR%..\..") do set "PROJECT_ROOT=%%~fI\"
    ) else if exist "%BAT_DIR%CMakeLists.txt" (
        set "PROJECT_ROOT=%BAT_DIR%"
    ) else (
        echo [ERROR] Impossible de trouver la racine du projet.
        echo [INFO] Emplacement du script : "%BAT_DIR%"
        echo [INFO] Le fichier CMakeLists.txt doit etre a la racine du projet.
        exit /b 1
    )
)

if not defined CONFIG_FILE set "CONFIG_FILE=%SCRIPTS_DIR%devConfig.txt"
if not defined LIST_FILE set "LIST_FILE=%SCRIPTS_DIR%folders_to_copy.txt"

if not exist "%CONFIG_FILE%" (
    echo [ERROR] Fichier config introuvable : "%CONFIG_FILE%"
    echo [INFO] Verifie que devConfig.txt est dans le dossier Scripts.
    exit /b 1
)

REM Nettoyage des variables selectionnees pour eviter les restes d'un ancien appel.
set "PI_USER="
set "PI_HOST="
set "REMOTE_HOME="
set "REMOTE_DIR="
set "RASPI_USER="
set "RASPI_HOST="
set "RASPI_DEST="
set "remoteUser="
set "ipAddress="
set "raspberryHome="

REM Format attendu : CLE=VALEUR
REM Les lignes vides sont ignorees et les lignes commencant par # sont des commentaires.
for /f "usebackq eol=# tokens=1,* delims==" %%A in ("%CONFIG_FILE%") do (
    set "CFG_KEY=%%~A"
    set "CFG_VALUE=%%~B"

    REM Suppression des espaces en debut de cle et de valeur.
    for /f "tokens=* delims= " %%K in ("!CFG_KEY!") do set "CFG_KEY=%%K"
    for /f "tokens=* delims= " %%V in ("!CFG_VALUE!") do set "CFG_VALUE=%%V"

    if defined CFG_KEY set "!CFG_KEY!=!CFG_VALUE!"
)

if not defined PI_PORT set "PI_PORT=22"
if not defined BUILD_TYPE set "BUILD_TYPE=Release"
if not defined USE_ORT set "USE_ORT=ON"
if not defined USE_RPLIDAR set "USE_RPLIDAR=ON"
if not defined PROGRAM_NAME set "PROGRAM_NAME=CATJ_ibmRobotProject"

if not defined CONFIG_MODE (
    echo [ERROR] CONFIG_MODE n'est pas defini dans "%CONFIG_FILE%".
    exit /b 1
)

if /i "%CONFIG_MODE%"=="EPF" (
    set "PI_USER=!EPF_USER!"
    set "PI_HOST=!EPF_HOST!"
    set "REMOTE_HOME=!EPF_HOME!"
    set "REMOTE_DIR=!EPF_PROJECT_DIR!"
) else if /i "%CONFIG_MODE%"=="PERSO" (
    set "PI_USER=!PERSO_USER!"
    set "PI_HOST=!PERSO_HOST!"
    set "REMOTE_HOME=!PERSO_HOME!"
    set "REMOTE_DIR=!PERSO_PROJECT_DIR!"
) else (
    echo [ERROR] CONFIG_MODE inconnu : "%CONFIG_MODE%"
    echo Valeurs acceptees : EPF ou PERSO
    exit /b 1
)

if not defined PI_USER (
    echo [ERROR] Utilisateur SSH non defini pour le mode "%CONFIG_MODE%".
    exit /b 1
)

if not defined PI_HOST (
    echo [ERROR] Adresse IP / hostname non defini pour le mode "%CONFIG_MODE%".
    exit /b 1
)

if not defined REMOTE_HOME set "REMOTE_HOME=/home/%PI_USER%"

if not defined REMOTE_DIR (
    echo [ERROR] Chemin projet distant non defini pour le mode "%CONFIG_MODE%".
    exit /b 1
)

REM Alias de compatibilite pour les anciens scripts.
set "RASPI_USER=%PI_USER%"
set "RASPI_HOST=%PI_HOST%"
set "RASPI_DEST=%REMOTE_DIR%"
set "remoteUser=%PI_USER%"
set "ipAddress=%PI_HOST%"
set "raspberryHome=%REMOTE_HOME%"

exit /b 0

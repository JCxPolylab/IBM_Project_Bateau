@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "BAT_DIR=%~dp0"
call "%BAT_DIR%_load_raspi_config.bat"
if errorlevel 1 (
    pause
    exit /b 1
)

set "defaultLocalDest=%PROJECT_ROOT%"

where scp >nul 2>&1
if errorlevel 1 (
    echo [ERROR] scp introuvable. Active "OpenSSH Client" sur Windows.
    pause
    exit /b 1
)

echo.
echo Mode          : %CONFIG_MODE%
echo Cible         : %PI_USER%@%PI_HOST%
echo Projet distant: %REMOTE_DIR%
echo Racine locale : %PROJECT_ROOT%
echo.
set /p "remotePath=Entrez le chemin source distant, relatif au projet ou absolu : "

if "%remotePath%"=="" (
    echo [ERROR] Aucun chemin source renseigne.
    pause
    exit /b 1
)

if "%remotePath:~0,1%"=="/" (
    set "sourcePath=%remotePath%"
) else if "%remotePath:~0,1%"=="~" (
    set "sourcePath=%remotePath%"
) else (
    set "sourcePath=%REMOTE_DIR%/%remotePath%"
)

echo.
set /p "type=Est-ce un FICHIER ou un DOSSIER ? (F/D) : "
if /I not "%type%"=="F" if /I not "%type%"=="D" (
    echo [ERROR] Reponse invalide. Tape F ou D.
    pause
    exit /b 1
)

echo.
set /p "destLocal=Entrez le dossier de destination sur le PC, ou laissez vide pour la racine projet : "

if "%destLocal%"=="" set "destLocal=%defaultLocalDest%"

if not exist "%destLocal%" (
    echo Le dossier "%destLocal%" n'existe pas. Creation...
    mkdir "%destLocal%"
    if errorlevel 1 (
        echo [ERROR] Impossible de creer le dossier local.
        pause
        exit /b 1
    )
)

echo.
echo Source distante    : %PI_USER%@%PI_HOST%:%sourcePath%
echo Destination locale : %destLocal%
echo.

set /p "rep=Voulez-vous lancer la copie ? (O/N) : "
if /I not "%rep%"=="O" exit /b 0

echo.
if /I "%type%"=="D" (
    scp -P "%PI_PORT%" -r "%PI_USER%@%PI_HOST%:%sourcePath%" "%destLocal%\"
) else (
    scp -P "%PI_PORT%" "%PI_USER%@%PI_HOST%:%sourcePath%" "%destLocal%\"
)

if errorlevel 1 (
    echo.
    echo [ERROR] La copie a echoue.
    pause
    exit /b 1
)

echo.
echo Copie reussie.
pause
endlocal
exit /b 0

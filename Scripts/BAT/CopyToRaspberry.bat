@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "BAT_DIR=%~dp0"
call "%BAT_DIR%_load_raspi_config.bat"
if errorlevel 1 (
    pause
    exit /b 1
)

where scp >nul 2>&1
if errorlevel 1 (
    echo [ERROR] scp introuvable. Active "OpenSSH Client" sur Windows.
    pause
    exit /b 1
)

if "%~1"=="" (
    echo Veuillez glisser-deposer un fichier ou un dossier sur ce script.
    pause
    exit /b 1
)

set "pathItem=%~1"
set "fileName=%~nx1"
set "extension=%~x1"

echo.
echo Mode      : %CONFIG_MODE%
echo Cible     : %PI_USER%@%PI_HOST%
echo Projet    : %REMOTE_DIR%
echo Chemin    : %pathItem%
echo Nom       : %fileName%
echo Extension : %extension%

if exist "%pathItem%\*" (
    set "type=DOSSIER"
) else if exist "%pathItem%" (
    set "type=FICHIER"
) else (
    echo [ERROR] Fichier ou dossier introuvable : "%pathItem%"
    pause
    exit /b 1
)

echo Type detecte : %type%
echo.
set /p "rep=Voulez-vous copier ce %type% vers la Raspberry Pi ? (O/N) : "
if /I not "%rep%"=="O" exit /b 0

echo.
echo Destination par defaut : %REMOTE_DIR%
echo Exemples : parametres, sources/fonctions, /home/%PI_USER%/Documents
echo Si le chemin est relatif, il sera interprete depuis le dossier projet distant.
set /p "saisir=Entrez le dossier de destination distant, ou laissez vide pour le projet : "

if "%saisir%"=="" (
    set "destination=%REMOTE_DIR%"
) else if "%saisir:~0,1%"=="/" (
    set "destination=%saisir%"
) else if "%saisir:~0,1%"=="~" (
    set "destination=%saisir%"
) else (
    set "destination=%REMOTE_DIR%/%saisir%"
)

echo.
echo Destination distante : %PI_USER%@%PI_HOST%:%destination%
echo.

if "%type%"=="DOSSIER" (
    scp -P "%PI_PORT%" -r "%pathItem%" "%PI_USER%@%PI_HOST%:%destination%/"
) else (
    scp -P "%PI_PORT%" "%pathItem%" "%PI_USER%@%PI_HOST%:%destination%/"
)

if errorlevel 1 (
    echo.
    echo [ERROR] Erreur lors de la copie.
    pause
    exit /b 1
)

echo.
echo Copie reussie.
pause
endlocal
exit /b 0

@echo off
setlocal
set "CONFIG_FILE=%PROJECT_ROOT%devConfig.txt"

:: Vérifie si un fichier/dossier est passé
if "%~1"=="" (
    echo Veuillez glisser-deposer un fichier ou un dossier sur ce script.
    pause
    exit /b
)

REM ======= CHECKS =======
if not exist "%CONFIG_FILE%" (
  echo [ERROR] Fichier config introuvable: "%CONFIG_FILE%"
  pause
  exit /b 1
)

set /p "CONFIG_MODE="<"%CONFIG_FILE%"

if /i "%CONFIG_MODE%"=="EPF" (
    echo Configuration EPF
    set "pathItem=%~1"
    set "fileName=%~nx1"
    set "extension=%~x1"
    set "raspberryHome=/"
    set ipAddress=10.150.74.211
    set raspiUsr=ibm_bateau
) else (
    echo Configuration inconnue ou autre valeur
    set "pathItem=%~1"
    set "fileName=%~nx1"
    set "extension=%~x1"
    set "raspberryHome=~/"
    set ipAddress=raspberrypi
    set raspiUsr=jerrycrozet
)

echo Chemin complet : %pathItem%
echo Nom : %fileName%
echo Extension : %extension%

:: Détection fichier ou dossier
if exist "%pathItem%\*" (
    set "type=DOSSIER"
) else (
    set "type=FICHIER"
)

echo Type detecte : %type%

:: Exemple : traitement selon extension
if /I "%extension%"==".txt" (
    echo Fichier texte detecte.
)

echo.
set /p rep=Voulez-vous copier ce %type% vers le Raspberry Pi ? (O/N) :
if /I not "%rep%"=="O" exit /b

echo.
set /p saisir=Entrez le chemin de destination (ex: Documents ou laisser vide pour le HOME ex de Chemin connus : Code/[C  CPP  PythonV3  server  web]):

if "%saisir%"=="" (
    set "destination=%raspberryHome%"
) else (
    set "destination=%raspberryHome%%saisir%/"
)

echo Destination distante : %destination%

:: Copie (fichier ou dossier)
if "%type%"=="DOSSIER" (
    scp -r "%pathItem%" %raspiUsr%@%ipAddress%:%destination%
) else (
    scp "%pathItem%" %raspiUsr%@%ipAddress%:%destination%%fileName%
)

if errorlevel 1 (
    echo Erreur lors de la copie.
    pause
    exit /b
)

echo Copie reussie !
pause

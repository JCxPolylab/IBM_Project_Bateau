@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "BAT_DIR=%~dp0"
call "%BAT_DIR%_load_raspi_config.bat"
if errorlevel 1 (
    pause
    exit /b 1
)

set "sourcePath=%REMOTE_DIR%/parametres/video"
set "destLocal=%PROJECT_ROOT%parametres"

where scp >nul 2>&1
if errorlevel 1 (
    echo [ERROR] scp introuvable. Active "OpenSSH Client" sur Windows.
    pause
    exit /b 1
)

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

scp -P "%PI_PORT%" -r "%PI_USER%@%PI_HOST%:%sourcePath%" "%destLocal%"

if errorlevel 1 (
    echo.
    echo [ERROR] La copie a echoue.
    pause
    exit /b 1
)

echo.
echo Copie video reussie.
pause
endlocal
exit /b 0

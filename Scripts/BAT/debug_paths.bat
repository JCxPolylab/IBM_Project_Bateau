@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "BAT_DIR=%~dp0"
call "%BAT_DIR%_load_raspi_config.bat"
if errorlevel 1 (
    pause
    exit /b 1
)

echo.
echo ===== DEBUG CHEMINS =====
echo BAT_DIR      = %BAT_DIR%
echo SCRIPTS_DIR  = %SCRIPTS_DIR%
echo PROJECT_ROOT = %PROJECT_ROOT%
echo CONFIG_FILE  = %CONFIG_FILE%
echo LIST_FILE    = %LIST_FILE%
echo.
echo CONFIG_MODE  = %CONFIG_MODE%
echo PI_USER      = %PI_USER%
echo PI_HOST      = %PI_HOST%
echo PI_PORT      = %PI_PORT%
echo REMOTE_HOME  = %REMOTE_HOME%
echo REMOTE_DIR   = %REMOTE_DIR%
echo PROGRAM_NAME = %PROGRAM_NAME%
echo.
echo ===== VERIFICATION LOCALE =====
if exist "%PROJECT_ROOT%CMakeLists.txt" (
    echo [OK] CMakeLists.txt trouve.
) else (
    echo [ERROR] CMakeLists.txt introuvable.
)
if exist "%PROJECT_ROOT%sources\*" (
    echo [OK] Dossier sources trouve.
) else (
    echo [ERROR] Dossier sources introuvable.
)
if exist "%PROJECT_ROOT%parametres\*" (
    echo [OK] Dossier parametres trouve.
) else (
    echo [ERROR] Dossier parametres introuvable.
)
if exist "%LIST_FILE%" (
    echo [OK] folders_to_copy.txt trouve.
) else (
    echo [ERROR] folders_to_copy.txt introuvable.
)
echo.
pause
endlocal
exit /b 0

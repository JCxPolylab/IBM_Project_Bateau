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

where ssh >nul 2>&1
if errorlevel 1 (
    echo [ERROR] ssh introuvable. Active "OpenSSH Client" sur Windows.
    pause
    exit /b 1
)

if not exist "%LIST_FILE%" (
    echo [ERROR] Fichier liste introuvable : "%LIST_FILE%"
    echo [INFO] Il doit etre dans le dossier Scripts.
    pause
    exit /b 1
)

echo.
echo ===== Synchronisation projet =====
echo Racine locale : %PROJECT_ROOT%
echo Liste         : %LIST_FILE%
echo Cible         : %PI_USER%@%PI_HOST%:%REMOTE_DIR%
echo ================================
echo.

ssh -p "%PI_PORT%" "%PI_USER%@%PI_HOST%" "mkdir -p '%REMOTE_DIR%'"
if errorlevel 1 (
    echo [ERROR] Impossible de creer/verifier le dossier distant : %REMOTE_DIR%
    pause
    exit /b 1
)

set /a COUNT=0
for /f "usebackq delims=" %%L in ("%LIST_FILE%") do (
    set "line=%%L"
    for /f "tokens=* delims= " %%A in ("!line!") do set "line=%%A"
    if defined line if not "!line:~0,1!"=="#" set /a COUNT+=1
)

echo Nombre d'elements a copier = !COUNT!
if !COUNT! LEQ 0 (
    echo [INFO] Rien a copier.
    pause
    exit /b 0
)

set /a IDX=1
for /f "usebackq delims=" %%L in ("%LIST_FILE%") do (
    set "rel=%%L"
    for /f "tokens=* delims= " %%A in ("!rel!") do set "rel=%%A"

    if defined rel if not "!rel:~0,1!"=="#" (
        if "!rel:~1,1!"==":" (
            set "src=!rel!"
        ) else (
            set "src=%PROJECT_ROOT%!rel!"
        )

        echo.
        echo [!IDX!/!COUNT!] Copie de "!rel!"
        echo Source : "!src!"

        if exist "!src!\*" (
            scp -P "%PI_PORT%" -r "!src!" "%PI_USER%@%PI_HOST%:%REMOTE_DIR%/"
        ) else if exist "!src!" (
            scp -P "%PI_PORT%" "!src!" "%PI_USER%@%PI_HOST%:%REMOTE_DIR%/"
        ) else (
            echo [ERROR] Source introuvable : "!src!"
            echo [INFO] Les chemins de folders_to_copy.txt doivent etre relatifs a : "%PROJECT_ROOT%"
            pause
            exit /b 2
        )

        if errorlevel 1 (
            echo [ERROR] Echec copie : "!rel!"
            pause
            exit /b 3
        )

        set /a IDX+=1
    )
)

echo.
echo [OK] Synchronisation terminee.

echo.
echo ===== Remote build on %PI_USER%@%PI_HOST%:%REMOTE_DIR% =====
echo BUILD_TYPE  : %BUILD_TYPE%
echo USE_ORT     : %USE_ORT%
echo USE_RPLIDAR : %USE_RPLIDAR%
echo PROGRAM     : %PROGRAM_NAME%
echo.

ssh -p "%PI_PORT%" "%PI_USER%@%PI_HOST%" ^
  "cd '%REMOTE_DIR%' && rm -rf build && cmake -S . -B build -DCMAKE_BUILD_TYPE=%BUILD_TYPE% -DUSE_RPLIDAR_SDK=%USE_RPLIDAR% -DUSE_ORT=%USE_ORT% && cmake --build build -j$(nproc)"

if errorlevel 1 (
    echo.
    echo [ERROR] Build failed.
    pause
    exit /b 4
)

echo.
echo [OK] Copy + build done.
pause
endlocal
exit /b 0

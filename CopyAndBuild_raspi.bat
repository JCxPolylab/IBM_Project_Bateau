@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM ======= CONFIG =======
set "PROJECT_ROOT=%~dp0"
set "LIST_FILE=%PROJECT_ROOT%folders_to_copy.txt"

set "RASPI_USER=jerrycrozet"
set "RASPI_HOST=raspberrypi"

REM Je te conseille un chemin absolu plutot que ~
set "RASPI_DEST=/home/jerrycrozet/camera_project/venv/code/IBM_Bateau"

REM ======= CHECKS =======
if not exist "%LIST_FILE%" (
  echo [ERROR] Fichier liste introuvable: "%LIST_FILE%"
  pause
  exit /b 1
)

echo.
echo === Lecture de la liste: "%LIST_FILE%" ===

REM ======= PASS 1 : COUNT =======
set /a COUNT=0
for /f "usebackq delims=" %%L in ("%LIST_FILE%") do (
  set "line=%%L"
  for /f "tokens=* delims= " %%A in ("!line!") do set "line=%%A"
  if defined line if not "!line:~0,1!"=="#" (
    set /a COUNT+=1
  )
)

echo Nombre de dossiers a copier = !COUNT!
if !COUNT! LEQ 0 (
  echo [INFO] Rien a copier.
  pause
  exit /b 0
)

echo.
echo === Copie vers %RASPI_USER%@%RASPI_HOST%:%RASPI_DEST% ===

REM ======= PASS 2 : COPY =======
set /a IDX=0
for /f "usebackq delims=" %%L in ("%LIST_FILE%") do (
  set "rel=%%L"
  for /f "tokens=* delims= " %%A in ("!rel!") do set "rel=%%A"
  if not defined rel goto :continue
  if "!rel:~0,1!"=="#" goto :continue

  REM construit src: si chemin absolu Windows (C:\...) on le garde, sinon relatif au projet
  set "src="
  if "!rel:~1,1!"==":" (
    set "src=!rel!"
  ) else (
    set "src=%PROJECT_ROOT%!rel!"
  )

  ::if exist "!src!\NUL" (
    echo.
    echo [!IDX!/!COUNT!] Copy "!rel!" ...
    scp -r "!src!" "%RASPI_USER%@%RASPI_HOST%:%RASPI_DEST%/"
    if errorlevel 1 (
      echo [ERROR] Echec copie: "!rel!"
      pause
      exit /b 3
    ::)
  ::) else (
	::echo "dossier introuvable : !src!"
	::pause
  )

  set /a IDX+=1
)

echo "copie terminée"

setlocal EnableExtensions EnableDelayedExpansion

rem =======================
rem  CONFIG A ADAPTER
rem =======================
set "PI_USER=jerrycrozet"
set "PI_HOST=raspberrypi"
set "PI_PORT=22"

rem Chemin du projet SUR la Raspberry Pi (dossier qui contient le CMakeLists.txt)
set "REMOTE_DIR=~/camera_project/venv/code/IBM_Bateau"

rem Options de build
set "BUILD_TYPE=Release"
set "USE_ORT=ON"

rem =======================
rem  CHECK OUTILS
rem =======================
where ssh >nul 2>&1
if errorlevel 1 (
  echo [ERROR] ssh introuvable. Active "OpenSSH Client" sur Windows.
  exit /b 1
)

echo.
echo ===== Remote build on %PI_USER%@%PI_HOST%:%REMOTE_DIR% =====
echo.

rem =======================
rem  BUILD REMOTE
rem =======================
ssh -p %PI_PORT% %PI_USER%@%PI_HOST% ^
  "cd %REMOTE_DIR% && rm -rf build && cmake -S . -B build -DCMAKE_BUILD_TYPE=%BUILD_TYPE% -DUSE_ORT=%USE_ORT% && cmake --build build --parallel"

if errorlevel 1 (
  echo.
  echo [ERROR] Build failed.
  pause
  exit /b 2
)

echo.
echo [OK] Build done.
pause
exit /b 0
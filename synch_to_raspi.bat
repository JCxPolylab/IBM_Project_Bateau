@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM ======= CONFIG =======
set "CONFIG_FILE=%PROJECT_ROOT%devConfig.txt"
set "PROJECT_ROOT=%~dp0"
set "LIST_FILE=%PROJECT_ROOT%folders_to_copy.txt"
DZ

REM ======= CHECKS =======
if not exist "%LIST_FILE%" (
  echo [ERROR] Fichier liste introuvable: "%LIST_FILE%"
  pause
  exit /b 1
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
    set "RASPI_USER=ibm_bateau"
    set "RASPI_HOST=10.224.133.212"
    set "RASPI_DEST=/home/ibm_bateau/code/jerryCamera"
) else (
    echo Configuration inconnue ou autre valeur
    set "RASPI_USER=jerrycrozet"
    set "RASPI_HOST=raspberrypi"
    set "RASPI_DEST=/home/jerrycrozet/camera_project/venv/code/IBM_Bateau"
)

REM config RASPI Jerry

REM set "RASPI_DEST=/home/jerrycrozet/camera_project/venv/code/IBM_Bateau"

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

echo.
echo === OK: copie terminee ===
pause
endlocal
exit /b 0
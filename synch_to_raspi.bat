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

echo.
echo === OK: copie terminee ===
pause
endlocal
exit /b 0
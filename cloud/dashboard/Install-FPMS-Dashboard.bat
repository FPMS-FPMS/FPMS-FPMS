@echo off
REM ============================================================
REM  Install / update FPMS Dashboard as a real Windows app.
REM  Double-click this file.
REM
REM  This is a thin wrapper. All the work - and all the checks
REM  that stop you installing over a running copy or launching
REM  into a stale backend - is in scripts\Install-App.ps1.
REM ============================================================

cd /d "%~dp0"

REM  -ExecutionPolicy Bypass is required: the default policy on a home Windows
REM  install refuses unsigned .ps1 files, and the error it prints looks like the
REM  script is broken rather than blocked.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\Install-App.ps1" %*
set "RC=%ERRORLEVEL%"

echo.
if not "%RC%"=="0" (
    echo [!] Install failed with code %RC%. The reason is printed above -
    echo     read it before retrying; retrying rarely helps on its own.
) else (
    echo [+] Done. Launch FPMS Dashboard from the Desktop or Start Menu.
)
echo.
pause

@echo off
REM ============================================================
REM  FPMS  ·  Persistent public dashboard
REM
REM  Double-click this to publish the dashboard on your permanent
REM  URL. If the app or its tunnel crashes, it restarts here.
REM
REM  The APP owns the tunnel (FPMS_AUTO_TUNNEL below) rather than
REM  this script launching cloudflared itself. That matters: only
REM  the app parses the fresh *.trycloudflare.com hostname and
REM  registers it with the permanent gateway URL. A cloudflared
REM  started out here would publish nothing, and the permanent
REM  link would keep pointing at a tunnel that no longer exists.
REM
REM  Closing this window stops everything.
REM ============================================================

setlocal EnableDelayedExpansion
title FPMS - Persistent Public Dashboard

cd /d "%~dp0"

REM ---- 0. Resolve the password without ever blocking on input -------------
REM  The logon task runs with no console attached, so a set /p prompt here
REM  would hang forever and the public URL would silently never come up.
if not defined FPMS_PASSWORD (
    for /f "usebackq tokens=*" %%p in (`powershell -NoProfile -Command "[Environment]::GetEnvironmentVariable('FPMS_PASSWORD','User')"`) do set "FPMS_PASSWORD=%%p"
)

if not defined FPMS_PASSWORD (
    if defined FPMS_UNATTENDED (
        if not exist "%LOCALAPPDATA%\FPMS" mkdir "%LOCALAPPDATA%\FPMS" 2>nul
        echo [%DATE% %TIME%] FPMS_PASSWORD not set - cannot publish unattended.>> "%LOCALAPPDATA%\FPMS\autostart.log"
        exit /b 1
    )
    REM  No parentheses in this prompt text - cmd parses the whole if-block up
    REM  front and a bare ")" here would close it early, failing the entire
    REM  script with ": was unexpected at this time."
    set /p FPMS_PASSWORD=Shared password - anyone with this can view the dashboard:
)
if "!FPMS_PASSWORD!"=="" (
    echo [!] Password is required. Aborting.
    if defined FPMS_UNATTENDED exit /b 1
    pause & exit /b 1
)

REM ---- 1. cloudflared must exist; the app spawns it from bin\ -------------
if not exist "%~dp0bin\cloudflared.exe" (
    echo [~] downloading cloudflared once...
    mkdir "%~dp0bin" 2>nul
    powershell -NoProfile -Command "Invoke-WebRequest -UseBasicParsing -Uri 'https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-windows-amd64.exe' -OutFile '%~dp0bin\cloudflared.exe'"
)

REM ---- 2. Tell the app to raise the tunnel and register it ----------------
REM  Headless on purpose: this is the always-on background service. Double-
REM  clicking FPMS-Dashboard.exe still opens your normal desktop window - it
REM  detects this backend on :8000 and attaches to it instead of starting a
REM  second one. Without headless, closing your window would trip the restart
REM  loop below and the window would keep reappearing.
set FPMS_AUTO_TUNNEL=1
set FPMS_HEADLESS=1

echo.
echo ============================================================
echo   Publishing the dashboard globally.
echo.
echo   Permanent URL (bookmark this - it never changes):
echo     https://fpms.aryan0419wadhawan.workers.dev
echo.
echo   Leave this window open. Closing it takes the app offline.
echo ============================================================
echo.

REM ---- 3. App with restart-on-crash --------------------------------------
REM  Redirect stdout/stderr to a file. Under Task Scheduler there is no console
REM  at all, so a windowed PyInstaller build gets sys.stderr = None and uvicorn's
REM  logging setup has nowhere to attach - the app then stalls right after
REM  "starting embedded uvicorn" and never binds the port.
if not exist "%LOCALAPPDATA%\FPMS" mkdir "%LOCALAPPDATA%\FPMS" 2>nul

REM  Rotate at ~20 MB. This appends on every restart and the machine is meant
REM  to run for months, so without this the log grows without limit.
for %%F in ("%LOCALAPPDATA%\FPMS\service.log") do if %%~zF GTR 20000000 (
    move /y "%LOCALAPPDATA%\FPMS\service.log" "%LOCALAPPDATA%\FPMS\service.log.old" >nul 2>&1
)

REM  Prefer the installed copy over the build output. Running the build folder
REM  meant the service held dist\FPMS-Dashboard.exe open, so installing a new
REM  version could not replace it and the backend silently stayed on the old
REM  build while the window ran the new one.
set "FPMS_EXE=%~dp0dist\FPMS-Dashboard.exe"
if exist "%ProgramFiles(x86)%\FPMS Dashboard\FPMS-Dashboard.exe" set "FPMS_EXE=%ProgramFiles(x86)%\FPMS Dashboard\FPMS-Dashboard.exe"
if exist "%ProgramFiles%\FPMS Dashboard\FPMS-Dashboard.exe" set "FPMS_EXE=%ProgramFiles%\FPMS Dashboard\FPMS-Dashboard.exe"

REM  Explicit opt-in override, checked last so it beats both of the above.
REM
REM  Needed because the backend is COMPILED INTO the exe: rebuilding the React
REM  frontend (or pointing FPMS_FRONTEND_DIST at a newer one) updates the UI,
REM  but every API route still comes from whichever exe is running. A new tab
REM  whose buttons hit routes the installed build has never heard of looks
REM  broken, so a fresh backend needs a fresh exe.
REM
REM  Updating the installed copy needs elevation, so this lets a freshly built
REM  exe be used without reinstalling. It is opt-in precisely because it
REM  reintroduces the lock the preference order above exists to avoid: while
REM  this runs, dist\FPMS-Dashboard.exe is held open and an installer cannot
REM  replace it. Clear the variable before installing a new version.
if defined FPMS_EXE_OVERRIDE if exist "%FPMS_EXE_OVERRIDE%" set "FPMS_EXE=%FPMS_EXE_OVERRIDE%"
echo [i] running: %FPMS_EXE%

:appLOOP
"%FPMS_EXE%" >> "%LOCALAPPDATA%\FPMS\service.log" 2>&1
echo.
echo [~] dashboard exited, restarting in 3s...
timeout /t 3 /nobreak >nul
goto appLOOP

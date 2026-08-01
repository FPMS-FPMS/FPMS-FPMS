@echo off
REM ============================================================
REM  FPMS Robotics Operations Console  -  native desktop app
REM  Double-click to launch.
REM ============================================================

setlocal EnableDelayedExpansion
title FPMS Dashboard

echo.
echo ============================================================
echo   FPMS - Fire Prevention ^& Management System
echo   Robotics Operations Console  -  desktop edition
echo ============================================================
echo.

cd /d "%~dp0"

REM ---- 1. Docker Desktop (only needed for AWS LOCAL mode) -----
if /I "%FPMS_AWS_MODE%"=="cloud" (
    echo [i] AWS mode: CLOUD  ^(using real AWS in your account^)
    goto skip_docker
)
echo [i] AWS mode: LOCAL  ^(using LocalStack^)

set "DOCKER_EXE=C:\Program Files\Docker\Docker\resources\bin\docker.exe"
if not exist "%DOCKER_EXE%" (
    echo [!] Docker Desktop is required for AWS LOCAL mode. Install from
    echo     https://www.docker.com/products/docker-desktop
    echo     To use real AWS instead, set FPMS_AWS_MODE=cloud and re-run.
    pause
    exit /b 1
)
"%DOCKER_EXE%" info >nul 2>&1
if errorlevel 1 (
    echo [~] Starting Docker Desktop, please wait...
    start "" "C:\Program Files\Docker\Docker\Docker Desktop.exe"
    REM  Bounded wait. This used to be an unconditional `goto` loop, so a Docker
    REM  Desktop that never came up left the launcher spinning forever with no
    REM  message and no way to tell it apart from a slow start.
    set /a DOCKER_WAIT=0
    :waitdocker
    timeout /t 3 /nobreak >nul
    set /a DOCKER_WAIT+=3
    "%DOCKER_EXE%" info >nul 2>&1
    if not errorlevel 1 goto dockerup
    if !DOCKER_WAIT! GEQ 180 (
        echo [!] Docker Desktop did not come up within 3 minutes.
        echo     Start it by hand, or set FPMS_AWS_MODE=cloud to skip LocalStack.
        pause
        exit /b 1
    )
    echo     ... still waiting for Docker ^(!DOCKER_WAIT!s^)
    goto waitdocker
    :dockerup
    echo [+] Docker Desktop is up.
) else (
    echo [+] Docker Desktop is already running.
)

REM ---- 2. LocalStack + Mosquitto stack ------------------------
pushd ..\localstack
"%DOCKER_EXE%" compose ps --format "{{.Name}} {{.Status}}" | findstr /C:"fpms-localstack" | findstr /C:"Up" >nul
if errorlevel 1 (
    echo [~] Bringing up LocalStack + Mosquitto + rule-bridge...
    "%DOCKER_EXE%" compose up -d
) else (
    echo [+] AWS local stack already running.
)
popd
:skip_docker

REM ---- 3. Python venv + backend deps --------------------------
if not exist ".venv\Scripts\python.exe" (
    echo [~] Creating Python virtual environment...
    where python >nul 2>&1 || (
        echo [!] Python 3.11+ is required. Install from https://www.python.org/downloads/
        pause
        exit /b 1
    )
    python -m venv .venv
    .venv\Scripts\python.exe -m pip install --upgrade pip
    .venv\Scripts\python.exe -m pip install -r backend\requirements.txt
) else (
    .venv\Scripts\python.exe -m pip install -q -r backend\requirements.txt
)

REM ---- 3b. MQTT credentials ----------------------------------
REM  A backend started without broker credentials against a broker with
REM  allow_anonymous false connects, is told "Not authorized", and retries in a
REM  loop. Nothing in the UI says so - every rover panel just waits forever.
REM  Check it here, before anything else has a chance to look healthy.
if not defined FPMS_MQTT_USERNAME (
    for /f "usebackq tokens=*" %%p in (`powershell -NoProfile -Command "[Environment]::GetEnvironmentVariable('FPMS_MQTT_USERNAME','User')"`) do set "FPMS_MQTT_USERNAME=%%p"
)
if not defined FPMS_MQTT_PASSWORD (
    for /f "usebackq tokens=*" %%p in (`powershell -NoProfile -Command "[Environment]::GetEnvironmentVariable('FPMS_MQTT_PASSWORD','User')"`) do set "FPMS_MQTT_PASSWORD=%%p"
)
if not defined FPMS_MQTT_USERNAME goto nocreds
if not defined FPMS_MQTT_PASSWORD goto nocreds
echo [+] MQTT credentials found for user "%FPMS_MQTT_USERNAME%".
goto creds_done
:nocreds
echo.
echo [!] ============================================================
echo [!]  NO MQTT BROKER CREDENTIALS ARE SET.
echo [!]
echo [!]  If the broker runs with allow_anonymous false, this backend
echo [!]  will be refused with "Not authorized", retry forever, and
echo [!]  EVERY rover panel will sit at "waiting" with no error shown.
echo [!]
echo [!]  Set them once (User scope), then re-run this file:
echo [!]    [Environment]::SetEnvironmentVariable('FPMS_MQTT_USERNAME','fpms','User')
echo [!]    [Environment]::SetEnvironmentVariable('FPMS_MQTT_PASSWORD','^<password^>','User')
echo [!]
echo [!]  Or create the account: scripts\Setup-Mosquitto.ps1 -Password '^<password^>'
echo [!] ============================================================
echo.
echo     Continuing anyway in 10s - correct only if your broker is anonymous.
timeout /t 10 >nul
:creds_done

REM ---- 4. Frontend build (if missing) -------------------------
if not exist "frontend\dist\index.html" (
    echo [~] Building frontend (one-time)...
    set "NPM=C:\Program Files\nodejs\npm.cmd"
    if not exist "!NPM!" (
        echo [!] Node.js is required to build the frontend.
        echo     Install from https://nodejs.org  ^(LTS^) then re-run this file.
        pause
        exit /b 1
    )
    pushd frontend
    call "!NPM!" install --loglevel=error
    call "!NPM!" run build
    popd
)

REM ---- 5. Launch the native desktop window --------------------
if not defined FPMS_AWS_MODE set FPMS_AWS_MODE=local
if not defined FPMS_BIND_PORT set FPMS_BIND_PORT=8000
if not defined AWS_DEFAULT_REGION set AWS_DEFAULT_REGION=us-east-1
if /I "%FPMS_AWS_MODE%"=="local" (
    REM  Defaults only. These used to be unconditional, so an operator who had
    REM  pointed the app at a broker on another machine had it silently reset to
    REM  localhost every launch.
    if not defined FPMS_AWS_ENDPOINT set FPMS_AWS_ENDPOINT=http://localhost:4566
    if not defined AWS_ACCESS_KEY_ID set AWS_ACCESS_KEY_ID=test
    if not defined AWS_SECRET_ACCESS_KEY set AWS_SECRET_ACCESS_KEY=test
    if not defined FPMS_MQTT_HOST set FPMS_MQTT_HOST=localhost
    if not defined FPMS_MQTT_PORT set FPMS_MQTT_PORT=1883
) else (
    if not defined FPMS_MQTT_HOST (
        echo [!] For CLOUD mode set FPMS_MQTT_HOST to your AWS IoT Core endpoint,
        echo     e.g.  set FPMS_MQTT_HOST=xxxxxxxx-ats.iot.us-east-1.amazonaws.com
        echo     and set FPMS_MQTT_PORT=8883, FPMS_MQTT_TLS=1 with your device certs.
        pause
        exit /b 1
    )
)

REM  Which UI will be served. main.py honours FPMS_FRONTEND_DIST only when it is
REM  a real directory, so say out loud which one wins - a stale override is
REM  otherwise indistinguishable from "the rebuild didn't work".
if defined FPMS_FRONTEND_DIST (
    if exist "%FPMS_FRONTEND_DIST%\index.html" (
        echo [i] UI override: %FPMS_FRONTEND_DIST%
    ) else (
        echo [!] FPMS_FRONTEND_DIST=%FPMS_FRONTEND_DIST% has no index.html - it will be
        echo     IGNORED and frontend\dist served instead.
    )
) else (
    echo [i] UI: %CD%\frontend\dist
)

echo.
echo ============================================================
echo   Launching native window...
echo   AWS mode: %FPMS_AWS_MODE%
echo   MQTT    : %FPMS_MQTT_HOST%:%FPMS_MQTT_PORT%  user=%FPMS_MQTT_USERNAME%
echo   Health  : http://localhost:%FPMS_BIND_PORT%/api/health
echo ============================================================
echo.

.venv\Scripts\python.exe -m backend.desktop
set "RC=%ERRORLEVEL%"

REM  A non-zero exit here is the "app flashed and vanished" failure. Show it
REM  and hold the window open so the reason is readable.
if not "%RC%"=="0" (
    echo.
    echo [!] FPMS exited with code %RC%.
    echo     Full log: %LOCALAPPDATA%\FPMS\launch.log
    pause
)

endlocal

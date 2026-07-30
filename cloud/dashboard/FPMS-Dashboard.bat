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
    :waitdocker
    timeout /t 3 /nobreak >nul
    "%DOCKER_EXE%" info >nul 2>&1
    if errorlevel 1 goto waitdocker
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
    set FPMS_AWS_ENDPOINT=http://localhost:4566
    set AWS_ACCESS_KEY_ID=test
    set AWS_SECRET_ACCESS_KEY=test
    set FPMS_MQTT_HOST=localhost
    set FPMS_MQTT_PORT=1883
) else (
    if not defined FPMS_MQTT_HOST (
        echo [!] For CLOUD mode set FPMS_MQTT_HOST to your AWS IoT Core endpoint,
        echo     e.g.  set FPMS_MQTT_HOST=xxxxxxxx-ats.iot.us-east-1.amazonaws.com
        echo     and set FPMS_MQTT_PORT=8883, FPMS_MQTT_TLS=1 with your device certs.
        pause
        exit /b 1
    )
)

echo.
echo ============================================================
echo   Launching native window...
echo   AWS mode: %FPMS_AWS_MODE%
echo ============================================================
echo.

.venv\Scripts\python.exe -m backend.desktop

endlocal

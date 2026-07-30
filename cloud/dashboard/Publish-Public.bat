@echo off
REM ============================================================
REM  FPMS  ·  Publish the dashboard to the public internet
REM
REM  Uses Cloudflare Tunnel (free) to give you a public HTTPS URL
REM  that anyone in the world can open. Nothing runs on a server —
REM  Cloudflare just relays traffic to the app on this laptop.
REM
REM  A password is REQUIRED before the tunnel starts. Set:
REM      set FPMS_PASSWORD=your-strong-password
REM  in the same terminal, then double-click / run this file.
REM ============================================================

setlocal EnableDelayedExpansion
title FPMS · Publish

cd /d "%~dp0"

REM ---- 1. Require a password ----------------------------------
if not defined FPMS_PASSWORD (
    echo.
    echo   [!] FPMS_PASSWORD is not set. Refusing to expose the dashboard
    echo       to the public internet without a password.
    echo.
    echo       In this terminal, run:
    echo           set FPMS_PASSWORD=your-strong-password
    echo       then double-click / run this file again.
    echo.
    pause
    exit /b 1
)

REM ---- 2. Ensure the dashboard is running ---------------------
echo [~] Checking dashboard on http://localhost:8000 ...
powershell -NoProfile -Command "try { (Invoke-WebRequest -UseBasicParsing -Uri http://localhost:8000/api/health -TimeoutSec 2).StatusCode } catch { 0 }" > "%TEMP%\fpms-hc.txt"
set /p HC=<"%TEMP%\fpms-hc.txt"
if not "%HC%"=="200" (
    echo [~] Dashboard is not up. Starting FPMS-Dashboard.exe ...
    start "" "%~dp0dist\FPMS-Dashboard.exe"
    timeout /t 6 /nobreak >nul
)

REM ---- 3. Locate cloudflared ----------------------------------
set "CFD=cloudflared.exe"
where cloudflared >nul 2>&1
if errorlevel 1 (
    if exist "%~dp0bin\cloudflared.exe" (
        set "CFD=%~dp0bin\cloudflared.exe"
    ) else (
        echo.
        echo [~] cloudflared not found. Downloading (one-time)...
        mkdir "%~dp0bin" 2>nul
        powershell -NoProfile -Command "Invoke-WebRequest -UseBasicParsing -Uri 'https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-windows-amd64.exe' -OutFile '%~dp0bin\cloudflared.exe'"
        if errorlevel 1 (
            echo [!] Download failed. Install manually from https://github.com/cloudflare/cloudflared/releases
            pause
            exit /b 1
        )
        set "CFD=%~dp0bin\cloudflared.exe"
    )
)

REM ---- 4. Start the tunnel ------------------------------------
echo.
echo ============================================================
echo   Publishing http://localhost:8000 to the public internet.
echo   Password protection is ACTIVE (FPMS_PASSWORD is set).
echo   Ctrl+C to stop and take it offline.
echo.
echo   Watch the log below for the public URL — it looks like:
echo       https://xxxxx-xxxxx-xxxxx.trycloudflare.com
echo ============================================================
echo.

"%CFD%" tunnel --url http://localhost:8000 --no-autoupdate

endlocal

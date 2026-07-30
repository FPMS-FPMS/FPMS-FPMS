@echo off
REM ============================================================
REM  FPMS  ·  Server-Only (no native window)
REM
REM  Use this if FPMS-Dashboard.exe won't open a window on your
REM  machine (WebView2 issue, antivirus quarantine, etc). The
REM  backend still starts and the tunnel URL + LAN URL keep
REM  working — you just view the app in a normal browser.
REM
REM  Ctrl+C in this window to stop.
REM ============================================================

setlocal EnableDelayedExpansion
title FPMS · Server Only

cd /d "%~dp0"

if not defined FPMS_PASSWORD (
    set /p FPMS_PASSWORD=Shared password (leave blank for open access):
)

set FPMS_HEADLESS=1
set FPMS_BIND_HOST=0.0.0.0
if not defined FPMS_BIND_PORT set FPMS_BIND_PORT=8000

echo.
echo ============================================================
echo   Starting FPMS backend on 0.0.0.0:%FPMS_BIND_PORT% (no window).
echo   Local:  http://localhost:%FPMS_BIND_PORT%
echo   LAN:    check ipconfig or the app's Share panel
echo.
echo   Ctrl+C to stop.
echo ============================================================
echo.

"%~dp0dist\FPMS-Dashboard.exe" --headless

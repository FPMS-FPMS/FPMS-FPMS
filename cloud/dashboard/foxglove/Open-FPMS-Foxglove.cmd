@echo off
REM ============================================================
REM  FPMS - one-click Foxglove operator console.
REM
REM  Double-click this. It finds the rover (by the NAME
REM  fpms-pi.local, never a written-down IP), checks the bridge
REM  is actually answering on port 8765, and opens Foxglove
REM  Studio already connected.
REM
REM  If mDNS is broken and you know the address, you can pass it:
REM      Open-FPMS-Foxglove.cmd 192.168.137.42
REM
REM  Nothing here needs administrator rights, and that is on
REM  purpose: this laptop has no admin account, and the whole
REM  reason Foxglove replaced the MQTT bridge is that the Pi
REM  LISTENS and the laptop dials OUT, so no inbound firewall
REM  rule is ever needed.
REM ============================================================
setlocal
title FPMS Foxglove Console
cd /d "%~dp0"

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0Open-FPMS-Foxglove.ps1" %*

if errorlevel 1 (
    echo.
    echo Something above failed. Read the numbered steps, then close this window.
    pause
)
endlocal

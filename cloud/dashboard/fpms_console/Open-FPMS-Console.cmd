@echo off
REM ===========================================================================
REM  FPMS CONSOLE - one click. Double-click this file.
REM
REM  It opens the operator console, which is SERVED BY THE ROVER ITSELF.
REM  Nothing is installed on this laptop and nothing needs to be running here.
REM
REM  ALWAYS BY NAME. fpms-pi.local, never an IP address: the rover's DHCP
REM  address has moved more than seven times in this project and every note
REM  that wrote one down went stale within a day.
REM
REM  The laptop DIALS OUT to the Pi. That direction is not a preference - the
REM  Windows Firewall here has no inbound rule for 8090 or 9090 and there is no
REM  admin account to add one, so outbound is the only direction that works.
REM ===========================================================================
setlocal
set HOST=fpms-pi.local
set URL=http://%HOST%:8090/

echo.
echo   FPMS CONSOLE
echo   ------------
echo   Opening %URL%
echo.

REM A quick reachability check, so a dead link says so here instead of showing
REM up as a blank browser tab that the operator has to interpret.
ping -n 1 -w 1500 %HOST% >nul 2>&1
if errorlevel 1 (
  echo   WARNING: %HOST% did not answer a ping.
  echo.
  echo   Check, in this order:
  echo     1. Is the rover powered on and finished booting? Give it 60 seconds.
  echo     2. Is this laptop on the SAME wifi / hotspot as the rover?
  echo     3. Try again - mDNS name resolution is sometimes slow to warm up.
  echo.
  echo   Opening the browser anyway in case the ping is just blocked.
  echo.
)

start "" "%URL%"

echo   If the page loads but the header says LINK DOWN, the console is being
echo   served but rosbridge is not answering. On the rover:
echo       sudo systemctl status fpms-rosbridge
echo.
timeout /t 6 >nul
endlocal

@echo off
REM ===========================================================================
REM  FPMS Dashboard launcher.
REM
REM  The Startup shortcut points HERE, not at the exe. The packaged app does not
REM  reliably inherit User-scope environment variables when Explorer starts it
REM  from Startup: it then connects to the broker anonymously, mosquitto refuses
REM  it, and every rover panel sits at "waiting" with no error anywhere.
REM
REM  ---------------------------------------------------------------------------
REM  THE BROKER ADDRESS IS RESOLVED, NOT TYPED.
REM  ---------------------------------------------------------------------------
REM  This file used to carry a hard-coded Pi address. The Pi moves constantly -
REM  it has been .213, then .94, and it will be something else tomorrow - so the
REM  launcher spent most of its life pointing at a machine that was not there,
REM  producing the single worst symptom this dashboard has: every panel waiting,
REM  no error, looking exactly like a broken app.
REM
REM  Resolve-Broker.ps1 works it out instead, preferring the mDNS name
REM  fpms-pi.local and falling back through the last host that worked, the Pi's
REM  MAC in the neighbour table, and the hosts this laptop has recently talked
REM  to. It does not just check that something is listening on 1883 - it speaks
REM  MQTT and waits for actual fpms telemetry, because there is a broker on THIS
REM  laptop that answers instantly and has no rover on it.
REM
REM  TO OVERRIDE, set either of these before launching:
REM      FPMS_MQTT_HOST        used as-is, no resolution at all
REM      FPMS_MQTT_HOST_FORCE  same, but survives into the resolver's own logic
REM
REM  Resolver commentary lands in %LOCALAPPDATA%\FPMS\broker-resolve.log.
REM ===========================================================================
setlocal

set FPMS_MQTT_USERNAME=fpms
set FPMS_MQTT_PASSWORD=__FPMS_MQTT_PASSWORD__
if not defined FPMS_MQTT_PORT set FPMS_MQTT_PORT=1883

if not exist "%LOCALAPPDATA%\FPMS" mkdir "%LOCALAPPDATA%\FPMS" >nul 2>&1

REM An address supplied by the caller is never second-guessed.
if defined FPMS_MQTT_HOST goto :launch

if not exist "%~dp0Resolve-Broker.ps1" (
    REM Fail towards the name the operator uses rather than towards a number
    REM that was true once. If it is wrong the dashboard says so on its own.
    set FPMS_MQTT_HOST=fpms-pi.local
    echo [!] Resolve-Broker.ps1 is missing - falling back to fpms-pi.local >> "%LOCALAPPDATA%\FPMS\broker-resolve.log"
    goto :launch
)

for /f "usebackq delims=" %%H in (`powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0Resolve-Broker.ps1" 2^>^> "%LOCALAPPDATA%\FPMS\broker-resolve.log"`) do set FPMS_MQTT_HOST=%%H
if not defined FPMS_MQTT_HOST set FPMS_MQTT_HOST=fpms-pi.local

:launch
echo [i] %DATE% %TIME% FPMS_MQTT_HOST=%FPMS_MQTT_HOST%:%FPMS_MQTT_PORT% >> "%LOCALAPPDATA%\FPMS\broker-resolve.log"
start "" "__FPMS_EXE__"
endlocal

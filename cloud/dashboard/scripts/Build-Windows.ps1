<#
.SYNOPSIS
  Build the FPMS Dashboard Windows app and its installer, from a clean slate.

.DESCRIPTION
  There used to be no single command for this, and the steps were run by hand in
  varying order. That produced the exact mess this script exists to prevent:
  `dist\FPMS-Dashboard.exe` was locked by the running background service, so a
  fresh build was sent to `dist-new\` instead - and installer.iss still defaults
  to `dist`, so the installer kept shipping the OLD exe while the operator
  believed they had installed the new one. Nothing anywhere reported the
  mismatch.

  So: stop whatever is holding the exe first, then always build into `dist`, then
  always package from `dist`. One output folder, no ambiguity.

  Steps
    1. Stop the 'FPMS HQ' scheduled task and any running FPMS-Dashboard.exe.
    2. Build the React frontend  -> frontend\dist
    3. PyInstaller               -> dist\FPMS-Dashboard.exe
    4. Inno Setup (if present)   -> build\installer-dist\FPMS-Dashboard-Setup.exe

  Requires: Node.js, the dashboard venv (or pyinstaller on PATH), and - for
  step 4 only - Inno Setup 6 (ISCC.exe). Step 4 is skipped with a warning if
  ISCC is not installed; the exe from step 3 can still be run directly.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\scripts\Build-Windows.ps1
  powershell -ExecutionPolicy Bypass -File .\scripts\Build-Windows.ps1 -SkipFrontend
#>
[CmdletBinding()]
param(
    [switch]$SkipFrontend,
    [switch]$SkipInstaller
)

$ErrorActionPreference = 'Stop'

$dashboard = Split-Path -Parent $PSScriptRoot
Set-Location $dashboard
Write-Host "[i] dashboard root: $dashboard"

# ---- 1. Release the lock on dist\FPMS-Dashboard.exe -----------------------
# PyInstaller cannot overwrite a running exe, and its error ("permission
# denied") does not mention the scheduled task that is actually holding it.
$task = Get-ScheduledTask -TaskName 'FPMS HQ' -ErrorAction Ignore
if ($task) {
    if ($task.State -eq 'Running') {
        Write-Host "[~] Stopping scheduled task 'FPMS HQ'..."
        try {
            Stop-ScheduledTask -TaskName 'FPMS HQ' -ErrorAction Stop
            Start-Sleep -Seconds 2
        } catch {
            Write-Warning "Could not stop 'FPMS HQ': $($_.Exception.Message)"
            Write-Warning "  It runs under an S4U logon. Re-run this from an ADMINISTRATOR PowerShell if the build fails to overwrite dist\."
        }
    } else {
        Write-Host "[+] Scheduled task 'FPMS HQ' is not running."
    }
} else {
    Write-Host "[+] No 'FPMS HQ' scheduled task registered."
}

# Stop-Process on an instance started by that task fails with Access Denied
# from an unelevated shell, and -ErrorAction Ignore made that silent - the
# build then died later on a "permission denied" from PyInstaller that named a
# file rather than the process holding it. Report which ones survived.
$running = @(Get-Process -Name 'FPMS-Dashboard' -ErrorAction Ignore)
if ($running.Count -gt 0) {
    Write-Host ("[~] Stopping {0} running FPMS-Dashboard process(es)..." -f $running.Count)
    foreach ($p in $running) {
        try { Stop-Process -Id $p.Id -Force -ErrorAction Stop }
        catch { Write-Warning ("  PID {0} could not be stopped: {1}" -f $p.Id, $_.Exception.Message) }
    }
    Start-Sleep -Seconds 2
    $left = @(Get-Process -Name 'FPMS-Dashboard' -ErrorAction Ignore)
    if ($left.Count -gt 0) {
        Write-Warning ("{0} FPMS-Dashboard process(es) are still running: PID {1}" -f
                       $left.Count, (($left | ForEach-Object { $_.Id }) -join ', '))
        Write-Warning "  These need an ADMINISTRATOR PowerShell. If PyInstaller now fails with a permission error on dist\, this is why."
    }
}

# ---- 2. Frontend ----------------------------------------------------------
if ($SkipFrontend) {
    Write-Host "[--] Skipping frontend build (-SkipFrontend)."
} else {
    $npm = 'C:\Program Files\nodejs\npm.cmd'
    if (-not (Test-Path $npm)) {
        $cmd = Get-Command npm -ErrorAction Ignore
        if ($null -eq $cmd) { throw 'Node.js / npm not found. Install the LTS build from https://nodejs.org' }
        $npm = $cmd.Source
    }
    Push-Location (Join-Path $dashboard 'frontend')
    try {
        if (-not (Test-Path 'node_modules')) {
            Write-Host '[~] npm install...'
            & $npm install --loglevel=error
            if ($LASTEXITCODE -ne 0) { throw "npm install failed ($LASTEXITCODE)" }
        }
        Write-Host '[~] npm run build...'
        & $npm run build
        if ($LASTEXITCODE -ne 0) { throw "npm run build failed ($LASTEXITCODE)" }
    } finally { Pop-Location }

    $index = Join-Path $dashboard 'frontend\dist\index.html'
    if (-not (Test-Path $index)) { throw "Frontend build produced no $index" }
    Write-Host ("[ok] frontend\dist built {0}" -f (Get-Item $index).LastWriteTime)
}

# ---- 3. PyInstaller -> dist\ ---------------------------------------------
# Run from the dashboard root: fpms.spec resolves everything from Path.cwd().
$py = Join-Path $dashboard '.venv\Scripts\python.exe'
if (-not (Test-Path $py)) {
    Write-Warning 'No .venv found; falling back to the python on PATH.'
    $cmd = Get-Command python -ErrorAction Ignore
    if ($null -eq $cmd) { throw 'Python 3.11+ not found.' }
    $py = $cmd.Source
}

Write-Host '[~] PyInstaller -> dist\FPMS-Dashboard.exe (this takes a few minutes)...'
& $py -m PyInstaller 'build\fpms.spec' --distpath 'dist' --workpath 'build' --noconfirm
if ($LASTEXITCODE -ne 0) { throw "PyInstaller failed ($LASTEXITCODE)" }

# The spec is a ONE-FOLDER build now, so the output moved:
#     was  dist\FPMS-Dashboard.exe            (one file)
#     now  dist\FPMS-Dashboard\FPMS-Dashboard.exe  + _internal\
#
# This check used to look at the old path. That is not a harmless stale check:
# dist\FPMS-Dashboard.exe STILL EXISTS as a leftover from the last one-file
# build, so the check passed, and the script cheerfully reported the size and
# timestamp of a build it had not just produced. The Desktop shortcut on this
# machine was pointing at that exact leftover.
$appDir = Join-Path $dashboard 'dist\FPMS-Dashboard'
$exe    = Join-Path $appDir 'FPMS-Dashboard.exe'
if (-not (Test-Path $exe)) {
    throw "PyInstaller reported success but $exe does not exist. Expected a one-folder build; check build\fpms.spec still ends in a COLLECT()."
}
if (-not (Test-Path (Join-Path $appDir '_internal'))) {
    throw "$exe exists but there is no _internal\ beside it. That is a one-FILE build and it cannot start once installed. build\fpms.spec must use exclude_binaries=True + COLLECT()."
}
$appSize = (Get-ChildItem $appDir -Recurse -File | Measure-Object -Property Length -Sum).Sum / 1MB
Write-Host ("[ok] {0}  ({1:N1} MB total, {2})" -f $appDir, $appSize,
    (Get-Item $exe).LastWriteTime) -ForegroundColor Green

# Leftovers from the one-file era. Both are exes that look installable and are
# not - installing either gives an app that exits before it can log why.
foreach ($stale in @((Join-Path $dashboard 'dist\FPMS-Dashboard.exe'),
                     (Join-Path $dashboard 'dist-new\FPMS-Dashboard.exe'))) {
    if (Test-Path $stale) {
        Write-Warning "Stale one-file build still present: $stale"
        Write-Warning "  Nothing uses it any more. Delete it - a shortcut pointing at it launches a build that cannot start."
    }
}

# ---- 4. Installer ---------------------------------------------------------
if ($SkipInstaller) {
    Write-Host '[--] Skipping installer (-SkipInstaller).'
    return
}

$iscc = @(
    'C:\Program Files (x86)\Inno Setup 6\ISCC.exe',
    'C:\Program Files\Inno Setup 6\ISCC.exe'
) | Where-Object { Test-Path $_ } | Select-Object -First 1

if (-not $iscc) {
    # Not an error, and not a dead end. Install-App.ps1 does the same job -
    # copies the one-folder build to %LOCALAPPDATA%\Programs, makes both
    # shortcuts, and registers an uninstaller in Add/Remove Programs - with no
    # Inno Setup required. It is also the only one of the two that has been
    # tested on this machine, because ISCC is not installed here.
    Write-Host ''
    Write-Host '[--] ISCC.exe (Inno Setup 6) not found - no Setup.exe was built.' -ForegroundColor Yellow
    Write-Host '     That is fine. Install the app with:' -ForegroundColor Yellow
    Write-Host ''
    Write-Host '         powershell -ExecutionPolicy Bypass -File .\scripts\Install-App.ps1' -ForegroundColor Cyan
    Write-Host '     or just double-click  Install-FPMS-Dashboard.bat' -ForegroundColor Cyan
    Write-Host ''
    Write-Host '     A Setup.exe is only needed to hand the app to someone else;'
    Write-Host '     for that, install Inno Setup 6 from https://jrsoftware.org/isdl.php'
    Write-Host '     and re-run this script.'
    return
}

Write-Host '[~] Inno Setup -> build\installer-dist\FPMS-Dashboard-Setup.exe...'
& $iscc 'build\installer.iss'
if ($LASTEXITCODE -ne 0) { throw "ISCC failed ($LASTEXITCODE)" }

$setup = Join-Path $dashboard 'build\installer-dist\FPMS-Dashboard-Setup.exe'
Write-Host ("[ok] {0}  ({1:N1} MB)" -f $setup, ((Get-Item $setup).Length / 1MB)) -ForegroundColor Green
Write-Host ''
Write-Host 'Next: run the setup exe, then launch FPMS Dashboard from the Start Menu.'

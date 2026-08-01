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
        Stop-ScheduledTask -TaskName 'FPMS HQ'
        Start-Sleep -Seconds 2
    } else {
        Write-Host "[+] Scheduled task 'FPMS HQ' is not running."
    }
} else {
    Write-Host "[+] No 'FPMS HQ' scheduled task registered."
}

$running = @(Get-Process -Name 'FPMS-Dashboard' -ErrorAction Ignore)
if ($running.Count -gt 0) {
    Write-Host ("[~] Stopping {0} running FPMS-Dashboard process(es)..." -f $running.Count)
    $running | Stop-Process -Force -ErrorAction Ignore
    Start-Sleep -Seconds 2
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

$exe = Join-Path $dashboard 'dist\FPMS-Dashboard.exe'
if (-not (Test-Path $exe)) { throw "PyInstaller reported success but $exe does not exist" }
Write-Host ("[ok] {0}  ({1:N1} MB, {2})" -f $exe,
    ((Get-Item $exe).Length / 1MB), (Get-Item $exe).LastWriteTime) -ForegroundColor Green

# dist-new\ only ever existed to dodge the lock handled in step 1. Leaving a
# stale copy there invites someone to install the wrong one again.
$distNew = Join-Path $dashboard 'dist-new\FPMS-Dashboard.exe'
if (Test-Path $distNew) {
    Write-Warning "dist-new\FPMS-Dashboard.exe is stale now - dist\ is the one that gets packaged. Delete dist-new\ when convenient."
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
    Write-Warning 'ISCC.exe (Inno Setup 6) not found - skipping the installer.'
    Write-Warning "Run dist\FPMS-Dashboard.exe directly, or install Inno Setup from https://jrsoftware.org/isdl.php and re-run."
    return
}

Write-Host '[~] Inno Setup -> build\installer-dist\FPMS-Dashboard-Setup.exe...'
& $iscc 'build\installer.iss'
if ($LASTEXITCODE -ne 0) { throw "ISCC failed ($LASTEXITCODE)" }

$setup = Join-Path $dashboard 'build\installer-dist\FPMS-Dashboard-Setup.exe'
Write-Host ("[ok] {0}  ({1:N1} MB)" -f $setup, ((Get-Item $setup).Length / 1MB)) -ForegroundColor Green
Write-Host ''
Write-Host 'Next: run the setup exe, then launch FPMS Dashboard from the Start Menu.'

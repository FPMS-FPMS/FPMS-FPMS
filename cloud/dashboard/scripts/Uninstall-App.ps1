<#
.SYNOPSIS
  Remove the per-user FPMS Dashboard install created by Install-App.ps1.

.DESCRIPTION
  Reverses everything Install-App.ps1 did: stops the app, deletes
  %LOCALAPPDATA%\Programs\FPMS Dashboard, removes both shortcuts, and removes
  the Add/Remove Programs entry.

  This file is DELIBERATELY SELF-CONTAINED and is copied into the install
  folder by Install-App.ps1, because the UninstallString in the registry points
  at that copy. Add/Remove Programs has to keep working after the repo is moved
  or deleted - which is exactly the situation someone is usually in when they
  reach for it. Do not factor its helpers out into a shared file.

  Operator data under %LOCALAPPDATA%\FPMS (settings, downloads, logs) is kept
  unless -RemoveData is given. Uninstalling an app should not be how you
  discover your settings are gone.

.PARAMETER Silent
  No prompts. Used by QuietUninstallString.

.PARAMETER RemoveData
  Also delete %LOCALAPPDATA%\FPMS.

.PARAMETER Destination
  Install folder to remove. Normally omitted: the folder is read from the
  InstallLocation value the installer wrote, so an install made with
  Install-App.ps1 -Destination still uninstalls cleanly instead of deleting the
  default path it never used.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\scripts\Uninstall-App.ps1
#>
[CmdletBinding()]
param(
    [switch]$Silent,
    [switch]$RemoveData,
    [string]$Destination
)

$ErrorActionPreference = 'Stop'

$AppName  = 'FPMS Dashboard'
$ProcName = 'FPMS-Dashboard'
$TaskName = 'FPMS HQ'
$RegKey   = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\FPMS Dashboard'
$DataDir  = Join-Path $env:LOCALAPPDATA 'FPMS'

# Trust what the installer recorded over what this script assumes. Hardcoding
# the default path would delete the wrong folder for anyone who installed
# elsewhere, and leave the real one behind with no Add/Remove entry to reach it.
$Dest = $null
if ($Destination) {
    $Dest = [System.IO.Path]::GetFullPath($Destination)
} else {
    try {
        $loc = (Get-ItemProperty -Path $RegKey -Name InstallLocation -ErrorAction Stop).InstallLocation
        if (-not [string]::IsNullOrWhiteSpace($loc)) { $Dest = $loc }
    } catch { }
}
if (-not $Dest) { $Dest = Join-Path $env:LOCALAPPDATA "Programs\$AppName" }

function Say  ($m) { Write-Host "    $m" }
function Ok   ($m) { Write-Host "[ok] $m" -ForegroundColor Green }
function Info ($m) { Write-Host "[i]  $m" }
function Warn ($m) { Write-Host "[!]  $m" -ForegroundColor Yellow }
function Loud ($m) { Write-Host "[!!] $m" -ForegroundColor Red }

Write-Host ''
Write-Host "  Uninstalling $AppName" -ForegroundColor Cyan
Write-Host ('-' * 74) -ForegroundColor DarkGray

if (-not (Test-Path -LiteralPath $Dest)) {
    Warn "Not installed at $Dest - cleaning up any leftover shortcuts and registry entry."
}

if (-not $Silent) {
    $ans = Read-Host "Remove $AppName from this computer? [y/N]"
    if ($ans -notmatch '^(y|yes)$') { Write-Host 'Cancelled.'; return }
}

# ---- 1. Stop the instances that belong to THIS install ---------------------
# ONLY the ones running out of $Dest. An earlier version of this script stopped
# every FPMS-Dashboard process it could see, which killed a window the operator
# was actively using while uninstalling a copy in a different folder. An
# uninstaller has no business terminating an application it is not removing.
#
# A process whose ExecutablePath cannot be read is also one this account cannot
# terminate - same permission. Those are reported rather than retried, because
# Stop-Process on them fails with "Access is denied" and never says which
# process or why.
$rows = @()
foreach ($p in @(Get-Process -Name $ProcName -ErrorAction Ignore)) {
    $path = $null
    try { $path = $p.Path } catch { $path = $null }
    $rows += [PSCustomObject]@{
        Id = $p.Id; Path = $path; Protected = [string]::IsNullOrWhiteSpace($path)
    }
}
$protected = @($rows | Where-Object { $_.Protected })
$mine      = @($rows | Where-Object { -not $_.Protected -and $_.Path -like "$Dest\*" })
$others    = @($rows | Where-Object { -not $_.Protected -and $_.Path -notlike "$Dest\*" })

foreach ($i in $others) {
    Info ("leaving PID {0} alone - it runs from {1}, not from the folder being removed." -f $i.Id, $i.Path)
}
foreach ($i in $mine) {
    try {
        Stop-Process -Id $i.Id -Force -ErrorAction Stop
        Ok ("stopped PID {0}" -f $i.Id)
    } catch {
        Warn ("could not stop PID {0}: {1}" -f $i.Id, $_.Exception.Message)
    }
}
if ($mine.Count -gt 0) { Start-Sleep -Seconds 2 }

# Whether anything still holds the folder is a question for the filesystem, not
# the process list - Windows will not open a running exe for writing, and that
# answers it even for processes this account may not inspect. Guessing from the
# process list instead refused to uninstall whenever the 'FPMS HQ' task was
# alive, even though that task usually runs from the repo and never touches
# this folder.
function Test-FolderInUse([string]$Folder) {
    $probe = Join-Path $Folder 'FPMS-Dashboard.exe'
    if (-not (Test-Path -LiteralPath $probe)) { return $false }
    try {
        $fs = [System.IO.File]::Open($probe, 'Open', 'ReadWrite', 'None')
        $fs.Close(); $fs.Dispose()
        return $false
    } catch { return $true }
}

if (Test-FolderInUse $Dest) {
    Loud "The files in $Dest are still locked by a running process."
    Say ''
    if ($protected.Count -gt 0) {
        Say ("Processes this shell cannot inspect or stop: PID {0}" -f (($protected | ForEach-Object { $_.Id }) -join ', '))
        Say 'That means the holder is elevated or on another logon.'
        Say ''
    }
    $hq = $null
    try { $hq = (Get-ScheduledTask -TaskName $TaskName -ErrorAction Stop).State.ToString() } catch { }
    if ($hq) {
        Say ("The scheduled task '{0}' is registered (state: {1}) and runs under an" -f $TaskName, $hq)
        Say 'S4U logon outside your desktop session. From an ADMINISTRATOR PowerShell:'
        Say ''
        Say ("    Stop-ScheduledTask    -TaskName '{0}'" -f $TaskName)
        Say ("    Disable-ScheduledTask -TaskName '{0}'" -f $TaskName)
        Say ("    Get-Process -Name {0} | Stop-Process -Force" -f $ProcName)
    } else {
        Say 'Close every FPMS window, or reboot.'
    }
    Say ''
    Say 'Then re-run this uninstaller. Deleting the folder now would leave files'
    Say 'behind and an Add/Remove entry pointing at a half-removed install.'
    Write-Host ('-' * 74) -ForegroundColor DarkGray
    throw "Cannot uninstall: $Dest is locked by a process this shell cannot stop. See above. Nothing was removed."
}

if ($protected.Count -gt 0) {
    Warn ("PID {0} could not be inspected, but {1} is not locked, so removal can proceed." -f
          (($protected | ForEach-Object { $_.Id }) -join ', '), $Dest)
}

# ---- 2. Shortcuts ----------------------------------------------------------
$links = @(
    (Join-Path ([Environment]::GetFolderPath('Desktop'))  "$AppName.lnk"),
    (Join-Path ([Environment]::GetFolderPath('Programs')) "FPMS\$AppName.lnk"),
    (Join-Path ([Environment]::GetFolderPath('Programs')) "$AppName.lnk")
)
foreach ($l in $links) {
    if (Test-Path -LiteralPath $l) {
        Remove-Item -LiteralPath $l -Force -ErrorAction Ignore
        Ok "removed shortcut $l"
    }
}
$startDir = Join-Path ([Environment]::GetFolderPath('Programs')) 'FPMS'
if (Test-Path -LiteralPath $startDir) {
    if (@(Get-ChildItem -LiteralPath $startDir -Force).Count -eq 0) {
        Remove-Item -LiteralPath $startDir -Force -ErrorAction Ignore
    }
}

# ---- 3. Files --------------------------------------------------------------
# The registry entry runs this script FROM the folder being deleted, so the
# .ps1 is open and Windows will not remove it while it is executing. Copy to
# %TEMP% and relaunch from there, once.
if (Test-Path -LiteralPath $Dest) {
    $here = $MyInvocation.MyCommand.Path
    if ($here -and $here.StartsWith($Dest, [StringComparison]::OrdinalIgnoreCase)) {
        $relay = Join-Path $env:TEMP ("FPMS-Uninstall-{0}.ps1" -f $PID)
        Copy-Item -LiteralPath $here -Destination $relay -Force
        Info 'Relaunching from %TEMP% so the install folder can be deleted...'
        # NOT $args - that is an automatic variable and assigning to it here
        # would shadow the script's own argument array.
        $psArgs = @('-NoProfile','-ExecutionPolicy','Bypass','-File',$relay,'-Silent')
        if ($RemoveData) { $psArgs += '-RemoveData' }
        Start-Process -FilePath 'powershell.exe' -ArgumentList $psArgs -Wait -NoNewWindow
        Remove-Item -LiteralPath $relay -Force -ErrorAction Ignore
        return
    }

    try {
        Remove-Item -LiteralPath $Dest -Recurse -Force -ErrorAction Stop
        Ok "removed $Dest"
    } catch {
        Loud "Could not fully remove $Dest"
        Say  $_.Exception.Message
        Say  'Something still holds files in that folder. Close any FPMS window,'
        Say  'or reboot, then delete the folder by hand.'
    }
}

# Staging / retired folders left by an interrupted install.
$parent = Split-Path -Parent $Dest
if (Test-Path -LiteralPath $parent) {
    foreach ($leftover in @(Get-ChildItem -LiteralPath $parent -Directory -Filter "$AppName.*" -ErrorAction Ignore)) {
        Remove-Item -LiteralPath $leftover.FullName -Recurse -Force -ErrorAction Ignore
        Info ("removed leftover {0}" -f $leftover.Name)
    }
}

# ---- 4. Registry -----------------------------------------------------------
if (Test-Path -LiteralPath $RegKey) {
    Remove-Item -LiteralPath $RegKey -Recurse -Force -ErrorAction Ignore
    Ok 'removed the Add/Remove Programs entry'
}
$appsKey = 'HKCU:\Software\Classes\Applications\FPMS-Dashboard.exe'
if (Test-Path -LiteralPath $appsKey) {
    Remove-Item -LiteralPath $appsKey -Recurse -Force -ErrorAction Ignore
}

# ---- 5. Data ---------------------------------------------------------------
if ($RemoveData) {
    if (Test-Path -LiteralPath $DataDir) {
        Remove-Item -LiteralPath $DataDir -Recurse -Force -ErrorAction Ignore
        Ok "removed $DataDir"
    }
} elseif (Test-Path -LiteralPath $DataDir) {
    Info "Kept your settings and logs in $DataDir"
    Say  'Re-run with -RemoveData to delete those too.'
}

Write-Host ('-' * 74) -ForegroundColor DarkGray
Ok "$AppName removed."

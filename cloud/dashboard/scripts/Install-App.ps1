<#
.SYNOPSIS
  Install (or update) FPMS Dashboard as a real per-user Windows application.

.DESCRIPTION
  This is the no-Inno install path. Inno Setup's ISCC.exe is not installed on
  this machine, so build\installer.iss cannot be compiled here - this script
  does the same job with nothing but PowerShell, and unlike the .iss it has
  actually been run.

  It:
    1. Verifies the source really is the ONE-FOLDER build (exe + _internal\).
       dist\FPMS-Dashboard.exe still exists as a leftover from the old one-file
       build; installing that produces an app that cannot start.
    2. Refuses to start if a previous instance is holding its files, and tells
       you which kind of instance it is - because one kind cannot be stopped
       from an ordinary shell at all.
    3. Stages the copy, then swaps it in. It never half-copies over a working
       install: if anything is locked, the old install is left exactly as it was.
    4. Creates Desktop + Start Menu shortcuts with the FPMS icon.
    5. Registers under HKCU\...\Uninstall so it appears in Add/Remove Programs
       (Settings > Apps) with a working Uninstall button.
    6. Checks the backend port and complains LOUDLY if something already owns
       it - see the PORT section below.

.PARAMETER Source
  Folder containing FPMS-Dashboard.exe and _internal\.
  Default: <dashboard>\dist\FPMS-Dashboard

.PARAMETER Force
  Stop running instances without asking. (Only affects instances this account
  is actually allowed to stop.)

.PARAMETER Launch
  Start the app when the install finishes.

.PARAMETER Port
  Backend port to check for conflicts. Default 8000 (FPMS_BIND_PORT's default).

.PARAMETER Destination
  Install folder. Default %LOCALAPPDATA%\Programs\FPMS Dashboard, which is the
  same path build\installer.iss uses so the two install paths cannot produce two
  rival copies. Override only for testing a build side by side - the shortcuts
  and the Add/Remove Programs entry are NOT renamed, so a non-default
  destination repoints the operator's existing ones at it.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\scripts\Install-App.ps1
  powershell -ExecutionPolicy Bypass -File .\scripts\Install-App.ps1 -Force -Launch
#>
[CmdletBinding()]
param(
    [string]$Source,
    [switch]$Force,
    [switch]$Launch,
    [int]$Port = 8000,
    [string]$Destination,
    [switch]$NoIntegration
)

$ErrorActionPreference = 'Stop'

$AppName    = 'FPMS Dashboard'
$ExeName    = 'FPMS-Dashboard.exe'
$ProcName   = 'FPMS-Dashboard'
$IcoName    = 'fpms.ico'
$Version    = '1.0.0'
$Publisher  = 'FPMS Robotics'
$RegKey     = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\FPMS Dashboard'
$InnoRegKey = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\{9F1CBA07-2C6B-4A1E-B14A-3F2B9C4E5D01}_is1'
$TaskName   = 'FPMS HQ'

$dashboard = Split-Path -Parent $PSScriptRoot
if ($Destination) {
    $Dest = [System.IO.Path]::GetFullPath($Destination)
} else {
    $Dest = Join-Path $env:LOCALAPPDATA "Programs\$AppName"
}

function Say  ($m) { Write-Host "    $m" }
function Ok   ($m) { Write-Host "[ok] $m" -ForegroundColor Green }
function Info ($m) { Write-Host "[i]  $m" }
function Warn ($m) { Write-Host "[!]  $m" -ForegroundColor Yellow }
function Loud ($m) { Write-Host "[!!] $m" -ForegroundColor Red }
function Rule     { Write-Host ('-' * 74) -ForegroundColor DarkGray }

Write-Host ''
Write-Host "  Installing $AppName" -ForegroundColor Cyan
Rule

# ===========================================================================
# 1. SOURCE - and prove it is the one-folder build
# ===========================================================================
if (-not $Source) { $Source = Join-Path $dashboard 'dist\FPMS-Dashboard' }
$Source = [System.IO.Path]::GetFullPath($Source)

if (-not (Test-Path -LiteralPath $Source -PathType Container)) {
    $legacy = Join-Path $dashboard 'dist\FPMS-Dashboard.exe'
    Loud "No build found at: $Source"
    if (Test-Path -LiteralPath $legacy) {
        Say ''
        Say "There IS a $legacy, but it is the OLD one-file build."
        Say 'The current build\fpms.spec produces a one-FOLDER build, and a'
        Say 'one-folder exe installed without its _internal\ folder cannot start'
        Say 'at all - it exits before it can log why.'
    }
    Say ''
    Say 'Build it first:'
    Say '    powershell -ExecutionPolicy Bypass -File .\scripts\Build-Windows.ps1'
    throw "Source folder not found: $Source"
}

$srcExe = Join-Path $Source $ExeName
$srcInt = Join-Path $Source '_internal'
if (-not (Test-Path -LiteralPath $srcExe)) { throw "Missing $ExeName in $Source" }
if (-not (Test-Path -LiteralPath $srcInt -PathType Container)) {
    Loud 'This is a one-FILE build, not the one-folder build.'
    Say "Found $ExeName but no _internal\ beside it. Installing this gives you"
    Say 'an app that silently fails to launch. Rebuild with the current spec:'
    Say '    powershell -ExecutionPolicy Bypass -File .\scripts\Build-Windows.ps1'
    throw "No _internal\ folder in $Source"
}

$srcIndex = Join-Path $Source '_internal\frontend\dist\index.html'
if (-not (Test-Path -LiteralPath $srcIndex)) {
    Warn 'The build contains no _internal\frontend\dist\index.html.'
    Say  'The API will work but every page will be blank. Continuing anyway.'
}

$srcSize = [math]::Round(((Get-ChildItem -LiteralPath $Source -Recurse -File |
                           Measure-Object -Property Length -Sum).Sum / 1MB), 1)
Info "Source : $Source"
Info ("Build  : {0}, {1} MB" -f (Get-Item -LiteralPath $srcExe).LastWriteTime, $srcSize)
Info "Target : $Dest"
Write-Host ''

# ===========================================================================
# 2. RUNNING INSTANCES
#
# Two categories, and the difference decides whether this install can proceed:
#
#   normal    - started from this desktop by this user. Stoppable. Only the
#               ones running out of the target folder actually block the copy;
#               an instance running from the repo or another folder is a port
#               problem, not a file-lock problem, and stopping it uninvited
#               would close a window the operator is using.
#   protected - we cannot even read its ExecutablePath, which means we do not
#               hold PROCESS_QUERY_LIMITED_INFORMATION on it, which means
#               Stop-Process will get Access Denied too. On this machine that
#               is the instance launched by the 'FPMS HQ' scheduled task under
#               an S4U logon. No amount of retrying from an unelevated shell
#               will touch it.
#
# Classifying BEFORE copying is the whole point: a copy that dies partway
# through leaves an install directory that is neither the old build nor the new
# one, and nothing on screen says which files made it.
# ===========================================================================
function Get-FpmsInstances {
    $rows = @()
    foreach ($p in @(Get-Process -Name $ProcName -ErrorAction Ignore)) {
        $path = $null
        try { $path = $p.Path } catch { $path = $null }
        $rows += [PSCustomObject]@{
            Id        = $p.Id
            Path      = $path
            Protected = [string]::IsNullOrWhiteSpace($path)
        }
    }
    return @($rows)
}

# Ask the filesystem, not the process list, whether the target is actually
# locked. Windows will not open a running exe for writing, so a sharing
# violation here means "something is running out of this exact folder" - and it
# answers that even for the protected instances whose paths we are not allowed
# to read. Guessing from the process list instead used to refuse every update
# while the 'FPMS HQ' task was alive, including the common case where that task
# runs from the repo and does not touch the install folder at all.
function Test-FolderInUse([string]$Folder) {
    $probe = Join-Path $Folder $ExeName
    if (-not (Test-Path -LiteralPath $probe)) { return $false }
    try {
        $fs = [System.IO.File]::Open($probe, 'Open', 'ReadWrite', 'None')
        $fs.Close()
        $fs.Dispose()
        return $false
    } catch {
        return $true
    }
}

function Get-HqTaskState {
    try {
        $t = Get-ScheduledTask -TaskName $TaskName -ErrorAction Stop
        return $t.State.ToString()
    } catch { return $null }
}

$instances = Get-FpmsInstances
$protected = @($instances | Where-Object { $_.Protected })
$normal    = @($instances | Where-Object { -not $_.Protected })
$hqState   = Get-HqTaskState

if ($instances.Count -gt 0) {
    Warn ("{0} FPMS-Dashboard process(es) are running:" -f $instances.Count)
    foreach ($i in $instances) {
        if ($i.Protected) {
            Say ("  PID {0}  <access denied - elevated or another logon>" -f $i.Id)
        } else {
            Say ("  PID {0}  {1}" -f $i.Id, $i.Path)
        }
    }
    Write-Host ''
}

if ($protected.Count -gt 0) {
    Rule
    Loud 'AN FPMS INSTANCE IS RUNNING THAT THIS SHELL CANNOT STOP.'
    Say ''
    Say ("PID(s): {0}" -f (($protected | ForEach-Object { $_.Id }) -join ', '))
    Say 'Windows will not even tell this shell where their exe lives, which is'
    Say 'the same permission that Stop-Process needs. Retrying, or clicking End'
    Say 'Task in an unelevated Task Manager, will keep failing with Access'
    Say 'Denied and no explanation.'
    Say ''
    if ($hqState) {
        Say ("This is the scheduled task '{0}' (state: {1})." -f $TaskName, $hqState)
        Say 'It runs with an S4U logon, so it lives outside your desktop session.'
    } else {
        Say ("No scheduled task named '{0}' is registered, so this instance was" -f $TaskName)
        Say 'started some other way - most likely from an elevated shell.'
    }
    Say ''
    Say 'Fix it from an ADMINISTRATOR PowerShell (Win+X, "Terminal (Admin)"):'
    Say ''
    Say ("    Stop-ScheduledTask -TaskName '{0}'" -f $TaskName)
    Say ("    Get-Process -Name {0} | Stop-Process -Force" -f $ProcName)
    Say ''
    Say 'To stop it coming back on every logon, also:'
    Say ("    Disable-ScheduledTask -TaskName '{0}'" -f $TaskName)
    Say ''
    Say 'Then re-run this installer from your normal shell.'
    Rule

    # Whether this actually blocks the install is a separate question from
    # whether it is stoppable, and it is answered below by the lock probe rather
    # than assumed here. The 'FPMS HQ' task runs Publish-Public-Persistent.bat
    # out of the repo, so it usually does NOT hold the install folder - refusing
    # every update on its account would be wrong. It always holds the PORT
    # though, which is the warning at the end of this run.
    Warn 'Whether that blocks this install is checked next; it always affects the port.'
    Write-Host ''
}

if (Test-FolderInUse $Dest) {
    $stillMine = @($normal | Where-Object { $_.Path -like "$Dest\*" })
    if ($stillMine.Count -eq 0) {
        Rule
        Loud "THE FILES AT $Dest ARE LOCKED BY A PROCESS THIS SHELL CANNOT SEE."
        Say ''
        Say 'Windows refuses to open the installed exe for writing, but no process'
        Say 'this account can inspect is running from there. That combination means'
        Say 'the holder is elevated or on another logon.'
        Say ''
        if ($protected.Count -gt 0) {
            Say ("Almost certainly PID {0} above." -f (($protected | ForEach-Object { $_.Id }) -join ' / '))
        }
        if ($hqState) {
            Say ("Stop the scheduled task '{0}' (state: {1}) from an ADMINISTRATOR" -f $TaskName, $hqState)
            Say 'PowerShell, then re-run this installer:'
            Say ''
            Say ("    Stop-ScheduledTask -TaskName '{0}'" -f $TaskName)
            Say ("    Get-Process -Name {0} | Stop-Process -Force" -f $ProcName)
        } else {
            Say 'Close every FPMS window, or reboot, then re-run this installer.'
        }
        Say ''
        Say 'Nothing has been changed. Your current install is intact.'
        Rule
        throw ("$Dest is locked and cannot be replaced. Stop the process holding it " +
               "(see above) and re-run. No files were modified.")
    }
    # Locked by an instance we CAN stop - handled below.
}

$inTarget = @($normal | Where-Object { $_.Path -like "$Dest\*" })
$elsewhere = @($normal | Where-Object { $_.Path -notlike "$Dest\*" })

foreach ($i in $elsewhere) {
    Warn ("PID {0} is running from {1} - not from the folder being replaced, so it" -f $i.Id, $i.Path)
    Say  ("is left alone. It will still hold port {0}; see the port check below." -f $Port)
}

if ($inTarget.Count -gt 0) {
    $stop = $Force
    if (-not $stop) {
        Warn ("{0} instance(s) are running out of {1} and hold its files open." -f $inTarget.Count, $Dest)
        $ans = Read-Host 'Stop them and continue? [y/N]'
        $stop = ($ans -match '^(y|yes)$')
    }
    if (-not $stop) {
        throw 'Aborted: FPMS Dashboard is running and holds the files being replaced.'
    }
    foreach ($i in $inTarget) {
        try {
            Stop-Process -Id $i.Id -Force -ErrorAction Stop
            Ok ("stopped PID {0}" -f $i.Id)
        } catch {
            throw ("Could not stop PID {0}: {1}" -f $i.Id, $_.Exception.Message)
        }
    }
    # Handles are released asynchronously; copying immediately still hits
    # sharing violations on _internal\*.pyd.
    Start-Sleep -Seconds 2
    $still = @(Get-FpmsInstances | Where-Object { $_.Path -like "$Dest\*" })
    if ($still.Count -gt 0) {
        throw ("{0} instance(s) survived Stop-Process. Not copying." -f $still.Count)
    }
}

# ===========================================================================
# 3. STAGE, THEN SWAP
#
# Copy to a sibling folder first and only then displace the old install. If
# _internal\ is locked, the rename fails, the staging folder is thrown away,
# and the previous install is untouched and still launchable. The alternative -
# deleting the target and copying into it - turns any lock into a machine with
# no working FPMS at all.
# ===========================================================================
$stamp   = Get-Date -Format 'yyyyMMdd-HHmmss'
$staging = "$Dest.staging-$PID"
$retired = "$Dest.old-$stamp"

if (Test-Path -LiteralPath $staging) { Remove-Item -LiteralPath $staging -Recurse -Force }

Info 'Copying files...'
$parent = Split-Path -Parent $Dest
if (-not (Test-Path -LiteralPath $parent)) {
    New-Item -ItemType Directory -Path $parent -Force | Out-Null
}
try {
    Copy-Item -LiteralPath $Source -Destination $staging -Recurse -Force -ErrorAction Stop
} catch {
    if (Test-Path -LiteralPath $staging) {
        Remove-Item -LiteralPath $staging -Recurse -Force -ErrorAction Ignore
    }
    throw ("Copy failed, nothing was changed: {0}" -f $_.Exception.Message)
}

$stagedExe = Join-Path $staging $ExeName
if (-not (Test-Path -LiteralPath $stagedExe)) {
    Remove-Item -LiteralPath $staging -Recurse -Force -ErrorAction Ignore
    throw 'Copy completed but the staged folder has no exe. Nothing was changed.'
}

# Ship the icon and the uninstaller inside the install folder. The uninstaller
# has to live here, not in the repo: Add/Remove Programs keeps working after
# the repo is moved or deleted, which is exactly when someone reaches for it.
$repoIco = Join-Path $dashboard 'build\fpms.ico'
if (Test-Path -LiteralPath $repoIco) {
    Copy-Item -LiteralPath $repoIco -Destination (Join-Path $staging $IcoName) -Force
}
$repoUninst = Join-Path $PSScriptRoot 'Uninstall-App.ps1'
if (Test-Path -LiteralPath $repoUninst) {
    Copy-Item -LiteralPath $repoUninst -Destination (Join-Path $staging 'Uninstall-App.ps1') -Force
} else {
    Warn 'scripts\Uninstall-App.ps1 not found; Add/Remove Programs will have no working uninstaller.'
}

# ---------------------------------------------------------------------------
# CARRY THE LAUNCHER .cmd FORWARD. THIS IS NOT OPTIONAL.
#
# The Startup shortcut points at Start-FPMS-Dashboard.cmd INSIDE the install
# folder, not at the exe - the app does not reliably inherit User-scope
# environment variables when Explorer starts it from Startup, so it would come
# up with no MQTT credentials, be refused by the broker, and show every rover
# panel waiting forever.
#
# That file lives only here. It holds the broker password, so it cannot be kept
# in the repo and re-copied like the icon and the uninstaller. And the install
# below is a whole-folder swap: without this, every reinstall silently deleted
# it and left the Startup shortcut pointing at nothing.
# ---------------------------------------------------------------------------
$LauncherName = 'Start-FPMS-Dashboard.cmd'
$existingLauncher = Join-Path $Dest $LauncherName

# ---------------------------------------------------------------------------
# THE CREDENTIALS ARE CARRIED FORWARD. THE ADDRESS IS NOT.
#
# The previous version of this block copied the old launcher wholesale, which
# preserved the password (right) and ALSO preserved a hard-coded Pi IP (wrong).
# The Pi's address changes constantly, so every reinstall faithfully restored a
# stale one, and the dashboard came up pointing at a machine that had moved -
# every panel waiting, no error, indistinguishable from a broken app.
#
# So the launcher is now REGENERATED from scripts\Start-FPMS-Dashboard.template.cmd
# with only the password lifted out of the old file. The address is worked out
# at every launch by Resolve-Broker.ps1, which prefers the mDNS name the
# operator actually uses. A reinstall can no longer resurrect an old IP.
#
# The password is still never in the repo: it is read out of the file already on
# this machine, and when there is none a CHANGE_ME placeholder is written and
# reported loudly.
# ---------------------------------------------------------------------------
$mqttPassword = $null
if (Test-Path -LiteralPath $existingLauncher) {
    foreach ($line in (Get-Content -LiteralPath $existingLauncher -ErrorAction SilentlyContinue)) {
        if ($line -match '^\s*set\s+FPMS_MQTT_PASSWORD=(.*)$') {
            $mqttPassword = $Matches[1].Trim()
            break
        }
    }
}

$repoResolver = Join-Path $PSScriptRoot 'Resolve-Broker.ps1'
if (Test-Path -LiteralPath $repoResolver) {
    Copy-Item -LiteralPath $repoResolver -Destination (Join-Path $staging 'Resolve-Broker.ps1') -Force
} else {
    Warn 'scripts\Resolve-Broker.ps1 not found; the launcher will fall back to fpms-pi.local.'
}

$repoLauncherTemplate = Join-Path $PSScriptRoot 'Start-FPMS-Dashboard.template.cmd'
if (Test-Path -LiteralPath $repoLauncherTemplate) {
    if ([string]::IsNullOrWhiteSpace($mqttPassword) -or $mqttPassword -eq 'CHANGE_ME') {
        $mqttPassword = 'CHANGE_ME'
        Warn "No broker password could be read from an existing $LauncherName."
        Say  "Edit $(Join-Path $Dest $LauncherName) and set FPMS_MQTT_PASSWORD,"
        Say  'or the dashboard will start with no broker credentials.'
    } else {
        Ok "carried the existing broker password forward into $LauncherName"
    }
    (Get-Content -LiteralPath $repoLauncherTemplate -Raw).
        Replace('__FPMS_MQTT_PASSWORD__', $mqttPassword).
        Replace('__FPMS_EXE__', (Join-Path $Dest $ExeName)) |
        Set-Content -LiteralPath (Join-Path $staging $LauncherName) -Encoding ascii
    Ok "wrote $LauncherName (broker address resolved at launch, not hard-coded)"
} elseif (Test-Path -LiteralPath $existingLauncher) {
    # No template in the repo: preserving the old file beats deleting it, even
    # with its stale address. A missing launcher makes the Startup shortcut fail
    # with a dialog and no dashboard at all.
    Copy-Item -LiteralPath $existingLauncher -Destination (Join-Path $staging $LauncherName) -Force
    Warn "scripts\Start-FPMS-Dashboard.template.cmd is missing - preserved the OLD $LauncherName verbatim, stale broker address and all."
} else {
    Warn "Neither a template nor an existing $LauncherName was found. The Startup shortcut will have nothing to run."
}

if (Test-Path -LiteralPath $Dest) {
    Info 'Replacing the previous install...'
    try {
        Rename-Item -LiteralPath $Dest -NewName (Split-Path -Leaf $retired) -ErrorAction Stop
    } catch {
        Remove-Item -LiteralPath $staging -Recurse -Force -ErrorAction Ignore
        Loud 'The existing install could not be moved aside - its files are locked.'
        Say  'Your current install is INTACT and still works; nothing was replaced.'
        Say  'Something is still running out of it. Close the FPMS window, or:'
        Say  ("    Get-Process -Name {0} | Stop-Process -Force" -f $ProcName)
        Say  ("and if that says Access Denied, stop '{0}' from an admin shell." -f $TaskName)
        throw ("Could not replace $Dest : {0}" -f $_.Exception.Message)
    }
}

try {
    Rename-Item -LiteralPath $staging -NewName (Split-Path -Leaf $Dest) -ErrorAction Stop
} catch {
    # Vanishingly unlikely, but if it happens the machine has no install at all
    # and needs to be told precisely where the pieces are.
    Loud 'Failed to move the new build into place.'
    Say  "New build is at : $staging"
    Say  "Old build is at : $retired"
    Say  "Rename one of them to: $Dest"
    throw
}

$exe = Join-Path $Dest $ExeName
Ok "installed to $Dest"

if (Test-Path -LiteralPath $retired) {
    try {
        Remove-Item -LiteralPath $retired -Recurse -Force -ErrorAction Stop
    } catch {
        Warn "Could not delete the previous build at $retired - delete it when convenient."
        Say  'The new install is live and working; this is only wasted disk.'
    }
}

# ===========================================================================
# 4. SHORTCUTS
# ===========================================================================
$icoPath = Join-Path $Dest $IcoName
if (Test-Path -LiteralPath $icoPath) { $iconRef = $icoPath } else { $iconRef = "$exe,0" }

function New-Shortcut([string]$LinkPath, [string]$Target, [string]$Icon, [string]$WorkDir) {
    $dir = Split-Path -Parent $LinkPath
    if (-not (Test-Path -LiteralPath $dir)) {
        New-Item -ItemType Directory -Path $dir -Force | Out-Null
    }
    $sh = New-Object -ComObject WScript.Shell
    $sc = $sh.CreateShortcut($LinkPath)
    $sc.TargetPath       = $Target
    $sc.WorkingDirectory = $WorkDir
    $sc.IconLocation     = $Icon
    $sc.Description      = 'FPMS Robotics Operations Console'
    $sc.WindowStyle      = 1
    $sc.Save()
    [System.Runtime.InteropServices.Marshal]::ReleaseComObject($sh) | Out-Null
}

$desktopLnk = Join-Path ([Environment]::GetFolderPath('Desktop')) "$AppName.lnk"
$startDir   = Join-Path ([Environment]::GetFolderPath('Programs')) 'FPMS'
$startLnk   = Join-Path $startDir "$AppName.lnk"

# Taskbar grouping/pinning comes from the AppUserModelID the app sets on itself
# at startup (build\launcher.py _set_aumid), so it does not need to be stamped
# onto the .lnk here - WScript.Shell cannot set it anyway.
if ($NoIntegration) {
    Info 'Skipping shortcuts (-NoIntegration).'
} else {
    New-Shortcut $desktopLnk $exe $iconRef $Dest
    Ok "Desktop shortcut   : $desktopLnk"
    New-Shortcut $startLnk   $exe $iconRef $Dest
    Ok "Start Menu shortcut: $startLnk"

    # A shortcut left at the top level of the Start Menu by an older install
    # sits alongside the FPMS\ folder as a duplicate. One of this pair is
    # already on this machine.
    $strayLnk = Join-Path ([Environment]::GetFolderPath('Programs')) "$AppName.lnk"
    if (Test-Path -LiteralPath $strayLnk) {
        Remove-Item -LiteralPath $strayLnk -Force -ErrorAction Ignore
        Info 'removed a duplicate Start Menu shortcut from an earlier install'
    }
}

# ===========================================================================
# 5. ADD / REMOVE PROGRAMS
# ===========================================================================
if ($NoIntegration) {
    Info 'Skipping the Add/Remove Programs entry (-NoIntegration).'
} else {
$uninstScript = Join-Path $Dest 'Uninstall-App.ps1'
$uninstCmd    = ('powershell.exe -NoProfile -ExecutionPolicy Bypass -File "{0}"' -f $uninstScript)
$sizeKb       = [int](((Get-ChildItem -LiteralPath $Dest -Recurse -File |
                        Measure-Object -Property Length -Sum).Sum) / 1KB)

if (-not (Test-Path -LiteralPath $RegKey)) { New-Item -Path $RegKey -Force | Out-Null }
$vals = @{
    DisplayName          = $AppName
    DisplayVersion       = $Version
    DisplayIcon          = $icoPath
    Publisher            = $Publisher
    InstallLocation      = $Dest
    UninstallString      = $uninstCmd
    QuietUninstallString = "$uninstCmd -Silent"
    InstallDate          = (Get-Date -Format 'yyyyMMdd')
    URLInfoAbout         = 'https://github.com/FPMS-FPMS'
    Comments             = 'FPMS Robotics Operations Console'
}
foreach ($k in $vals.Keys) {
    New-ItemProperty -Path $RegKey -Name $k -Value $vals[$k] -PropertyType String -Force | Out-Null
}
New-ItemProperty -Path $RegKey -Name 'EstimatedSize' -Value $sizeKb -PropertyType DWord -Force | Out-Null
New-ItemProperty -Path $RegKey -Name 'NoModify'      -Value 1       -PropertyType DWord -Force | Out-Null
New-ItemProperty -Path $RegKey -Name 'NoRepair'      -Value 1       -PropertyType DWord -Force | Out-Null
Ok 'registered in Add/Remove Programs (Settings > Apps > Installed apps)'

# An Inno-built setup registers its own entry. Two entries for one folder means
# uninstalling via the wrong one deletes files the other still claims to own.
if (Test-Path -LiteralPath $InnoRegKey) {
    Warn 'An Inno Setup install of FPMS Dashboard is ALSO registered.'
    Say  'Add/Remove Programs will list FPMS Dashboard twice, and both entries'
    Say  "point at the same folder. Uninstall the Inno one first, then re-run this."
}
}

# Explorer caches shortcut icons per target path, so an updated icon at the same
# path often keeps rendering the old glyph. Nudge it.
try {
    $sig = '[DllImport("shell32.dll")] public static extern void SHChangeNotify(int e, uint f, IntPtr a, IntPtr b);'
    $sc = Add-Type -MemberDefinition $sig -Name 'FpmsShell' -Namespace 'Fpms' -PassThru -ErrorAction Stop
    $sc::SHChangeNotify(0x08000000, 0, [IntPtr]::Zero, [IntPtr]::Zero)  # SHCNE_ASSOCCHANGED
} catch { }

# ===========================================================================
# 6. PORT CONFLICT - the failure that looks exactly like success
#
# backend/desktop.py:_server_alive() probes /api/auth-status and treats ANY
# HTTP answer under 500 as "a backend is already running, attach to it". It
# does not check that the backend is the same build, or the same exe, or even
# that it is FPMS. So if anything is listening on the port when you launch the
# freshly installed app, the app opens a window onto THAT server and you are
# looking at the old UI with no indication anywhere that this happened.
# ===========================================================================
Write-Host ''
Rule
Info "Checking port $Port..."

function Get-PortHolders([int]$P) {
    $ids = @()
    try {
        $ids = @(Get-NetTCPConnection -LocalPort $P -State Listen -ErrorAction Stop |
                 Select-Object -ExpandProperty OwningProcess -Unique)
    } catch {
        # Get-NetTCPConnection is absent on some SKUs; netstat always exists.
        foreach ($line in (& netstat -ano -p TCP)) {
            if ($line -match "\s\S*:$P\s+\S+\s+LISTENING\s+(\d+)") { $ids += [int]$Matches[1] }
        }
        $ids = @($ids | Select-Object -Unique)
    }
    return @($ids)
}

function Get-Json([string]$Url) {
    try {
        $r = Invoke-WebRequest -Uri $Url -UseBasicParsing -TimeoutSec 4
        return @{ Answered = $true; Status = [int]$r.StatusCode; Body = $r.Content }
    } catch [System.Net.WebException] {
        $resp = $_.Exception.Response
        if ($null -ne $resp) {
            $body = ''
            try {
                $sr = New-Object System.IO.StreamReader($resp.GetResponseStream())
                $body = $sr.ReadToEnd()
                $sr.Close()
            } catch { }
            return @{ Answered = $true; Status = [int]$resp.StatusCode; Body = $body }
        }
        return @{ Answered = $false }
    } catch {
        return @{ Answered = $false }
    }
}

$holders = Get-PortHolders $Port
$probe   = Get-Json ("http://127.0.0.1:{0}/api/auth-status" -f $Port)

if ($holders.Count -eq 0 -and -not $probe.Answered) {
    Ok "port $Port is free - the app will start its own backend"
} else {
    Rule
    Loud "PORT $Port IS ALREADY IN USE. READ THIS BEFORE YOU LAUNCH THE APP."
    Say ''
    foreach ($id in $holders) {
        $desc = "PID $id"
        try {
            $pp = Get-Process -Id $id -ErrorAction Stop
            $ppath = $null
            try { $ppath = $pp.Path } catch { $ppath = $null }
            if ([string]::IsNullOrWhiteSpace($ppath)) {
                $desc = "PID $id  ($($pp.ProcessName))  <access denied - elevated or another logon>"
            } else {
                $desc = "PID $id  $ppath"
            }
        } catch { }
        Say "  listening: $desc"
    }
    if ($probe.Answered) {
        Say ("  responds to /api/auth-status with HTTP {0}" -f $probe.Status)
    }
    Say ''
    Say 'What this means, concretely:'
    Say ''
    Say '  The app you just installed will NOT start its own backend. It probes'
    Say '  this port first and attaches to whatever answers. The window opens,'
    Say '  the app looks healthy, and you are looking at the OLD build - the new'
    Say '  UI you just installed is never loaded. Nothing in the UI says so.'
    Say ''

    # /api/health names the exact frontend directory it is serving, which is the
    # only way to tell the two builds apart from outside. It may be password
    # protected, in which case say that rather than staying quiet.
    $health = Get-Json ("http://127.0.0.1:{0}/api/health" -f $Port)
    if ($health.Answered -and $health.Status -eq 200) {
        try {
            $h = $health.Body | ConvertFrom-Json
            Say ("  That backend is serving its UI from:")
            Say ("      {0}" -f $h.frontend.dist)
            if ($h.frontend.built_at) {
                $built = [DateTimeOffset]::FromUnixTimeSeconds([int64]$h.frontend.built_at).LocalDateTime
                Say ("      packaged {0}" -f $built)
                $mine = (Get-Item -LiteralPath $srcIndex -ErrorAction Ignore)
                if ($mine -and $built -lt $mine.LastWriteTime) {
                    Say ''
                    Loud ("  CONFIRMED OLDER: it serves a UI from {0}; you just installed {1}." -f $built, $mine.LastWriteTime)
                }
            }
            if ($h.frontend.dist -and ($h.frontend.dist -notlike "$Dest*")) {
                Say ("  It is NOT running from $Dest, so it is a different install.")
            }
        } catch { }
    } elseif ($health.Answered -and $health.Status -eq 401) {
        Say '  (/api/health is password-protected, so which UI it serves cannot be'
        Say '   read from here. Treat it as the old build until proven otherwise.)'
    }

    Say ''
    Say 'Do one of these BEFORE launching:'
    Say ''
    Say '  A) Stop whatever owns the port.'
    if ($hqState) {
        Say ("     The scheduled task '{0}' is registered (state: {1}) and is the" -f $TaskName, $hqState)
        Say '     usual owner. From an ADMINISTRATOR PowerShell:'
        Say ("         Stop-ScheduledTask   -TaskName '{0}'" -f $TaskName)
        Say ("         Disable-ScheduledTask -TaskName '{0}'   # stop it returning" -f $TaskName)
        Say ("         Get-Process -Name {0} | Stop-Process -Force" -f $ProcName)
    } else {
        Say ("         Get-Process -Id {0} | Stop-Process -Force" -f (($holders | Select-Object -First 1)))
    }
    Say ''
    Say '  B) Or put the new app on a different port, leaving the old one alone:'
    # Backtick-escaped on purpose: this is text for the operator to TYPE, not a
    # value to expand. Unescaped, PowerShell interpolated the (unset) variable
    # and printed a line beginning "= '8010'", which reads as a typo.
    Say ("         `$env:FPMS_BIND_PORT = '8010'; & '{0}'" -f $exe)
    Say ''
    Say ("  Verify afterwards - this should FAIL before you launch:")
    Say ("         Invoke-WebRequest http://127.0.0.1:{0}/api/auth-status -UseBasicParsing" -f $Port)
    Rule
}

# ===========================================================================
Write-Host ''
Rule
Ok "$AppName $Version installed."
Say  "Location : $Dest"
Say  "Launch   : Start Menu > FPMS > $AppName, or the Desktop shortcut"
Say  "Update   : re-run this script after scripts\Build-Windows.ps1"
Say  "Remove   : Settings > Apps > Installed apps > $AppName"
Rule

if ($Launch) {
    if ($holders.Count -gt 0 -or $probe.Answered) {
        Loud "NOT launching: port $Port is occupied and you would get the old UI."
        Say  'Clear the port (above), then start it from the Desktop shortcut.'
    } else {
        Info 'Launching...'
        Start-Process -FilePath $exe -WorkingDirectory $Dest
    }
}

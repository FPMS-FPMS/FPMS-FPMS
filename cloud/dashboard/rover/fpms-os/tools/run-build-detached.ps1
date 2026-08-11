<#
.SYNOPSIS
    Start the FPMS-OS image build inside WSL2, detached from everything that
    has killed it so far, and return in about two seconds.

.DESCRIPTION
    THE PROBLEM THIS EXISTS TO SOLVE
    ================================
    A full build is 15-20 hours (stage 10 alone was measured at 14h20m under
    qemu-user emulation - see docs/BUILDING.md, "MEASURED, 2026-08-11").
    Nothing caches the micro-ROS colcon build, so every death costs the whole
    14 hours again. Three builds have died. Two of them died because "the
    controlling process went away" DESPITE having been launched with
    `nohup setsid` from inside WSL.

    WHY `nohup setsid` WAS NOT ENOUGH - the most likely mechanism
    ------------------------------------------------------------
    `setsid` detaches from the controlling terminal and `nohup` ignores
    SIGHUP, so neither of those was the killer. But NEITHER OF THEM CHANGES
    THE INHERITED FILE DESCRIPTORS.

    When you run

        wsl -d Ubuntu-22.04 -u root -- bash -lc 'nohup setsid ./build.sh &'

    from an agent or a script, wsl.exe's stdout is a PIPE owned by the
    caller. `nohup` only redirects stdout when stdout is a TTY - here it is a
    pipe, so nohup leaves it exactly as it found it, and the detached build
    inherits that pipe as fd 1. build.sh then runs it through
    `exec > >(stdbuf -oL tee -a "$LOGFILE")`, so tee's own stdout is the pipe
    too.

    The moment the caller exits, the read end of that pipe closes. The next
    write from tee gets EPIPE / SIGPIPE and the build dies - hours later,
    with no error in the log, looking exactly like "something killed it".
    That is a death that `setsid` and `nohup` cannot prevent and that only
    shows up when the caller goes away, which matches the reported history.

    WHAT THIS SCRIPT DOES DIFFERENTLY
    ---------------------------------
      1. NO INHERITED PIPES OR TTYS. The detached process gets
         `< /dev/null` and `>> <logfile> 2>&1` explicitly, so every one of
         fds 0/1/2 is a file or /dev/null. There is no descriptor left in it
         that any Windows-side process owns, so nothing the caller does can
         break a write. The script VERIFIES this after launch by printing
         /proc/<pid>/fd/{0,1,2} - if fd1 is not the log file, say so loudly.
      2. NO LONG-LIVED WINDOWS PROCESS. `setsid --fork` forks and its parent
         exits immediately, so the launching wsl.exe returns in about a
         second. There is no wsl.exe left running that a job object, a
         closed terminal, or a dying agent could kill. This is why
         `Start-Process -WindowStyle Hidden` is NOT used: a hidden window is
         still a process in the caller's tree, and a Windows Job Object with
         JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE (which terminals and agent
         harnesses do use) kills it when the caller dies. The best defence
         against having your process killed is not owning one.
      3. REPARENTED TO PID 1. `setsid --fork` makes the runner a session
         leader with no controlling terminal, and its immediate parent exits,
         so /init (PID 1) adopts it. It is a distro-level daemon at that
         point, in the same category as anything started by systemd.
      4. A HEARTBEAT, so that when a build does die you can tell WHICH death
         it was: heartbeat stale + lock present = the whole distro went away
         (wsl --shutdown, sign-out, sleep, host reboot). Heartbeat fresh but
         no build = build.sh exited on its own.

    HONESTY ABOUT WHAT IS AND IS NOT PROVEN
    ---------------------------------------
    Proven by construction and checkable in seconds (this script prints the
    evidence at the end of a launch):
      - the build's fds point at /dev/null and a file, not at a pipe;
      - its PPID is 1 and it has no controlling tty;
      - the launcher's own wsl.exe has already exited.
    NOT proven without a real 15-hour run:
      - that WSL2 never reaps a distro that has only PID-1-parented
        processes left in it. Everything observable says it does not (this is
        how people run sshd/docker in WSL), but the only proof is a build
        that survives an overnight with every terminal closed.
    The definitive test, which takes 10 minutes and not 15 hours, is in
    tools/README.md under "Proving it is really detached": launch, close
    every terminal, `taskkill /IM wsl.exe /F`, wait, and check that the log
    is still growing.

    WHAT WILL ALWAYS KILL IT
    ------------------------
    `wsl --shutdown` (and `wsl --terminate <distro>`) stop the utility VM
    regardless of what is running inside it. So does signing out of Windows,
    a reboot, and - usually - sleep/hibernate. Nothing in user space can
    defend against those. See the table in tools/README.md.

.PARAMETER Distro
    WSL distro name. Default Ubuntu-22.04.

.PARAMETER RepoPath
    Linux path of the fpms-os checkout INSIDE the distro (the directory
    holding build.sh). Must be on the distro's own ext4 - never /mnt/c: 9p
    makes the emulated apt stages effectively never finish, and loop-mounting
    a file across 9p is unreliable (docs/BUILDING.md section 1).
    Left empty, the script probes the usual locations and tells you what it
    found.

.PARAMETER From
    Resume at this stage and run everything after it, e.g. -From 20.
    Maps to build.sh --from. Omit for a full build.

.PARAMETER Fresh
    Maps to build.sh --fresh: with -From, start from the vendor base image
    again instead of resuming the existing one. This THROWS AWAY every hour
    already in the image, so it is refused when an .img exists unless you
    also pass -Force.

.PARAMETER NoDownload
    Maps to build.sh --no-download (reuse the cached base image).

.PARAMETER MinFreeGB
    Refuse to start below this much free space on the Windows volume backing
    the distro's VHDX. Default 20. build.sh's own header asks for ~14 GB for
    the image plus the .xz; docs/BUILDING.md records that under about 20 GB
    the build "will probably fail", and names where. The VHDX grows and never
    shrinks, so this number only goes down during a build.

.PARAMETER Force
    Override the -Fresh-would-destroy-an-image refusal. Does NOT override the
    already-running refusal, which is never safe to override.

.PARAMETER DryRun
    Print the pre-flight that can be done from Windows alone, print the exact
    runner script that would be installed, and exit. Touches nothing and does
    not invoke wsl.exe at all.

.EXAMPLE
    .\tools\run-build-detached.ps1
    Full build, detached. Returns in ~2 s. Walk away.

.EXAMPLE
    .\tools\run-build-detached.ps1 -From 20
    Resume after a stage-10 success, against the existing image.

.NOTES
    GIT BASH / MSYS PATH MANGLING - not a problem here, but it will be for you
    ------------------------------------------------------------------------
    You are reading a PowerShell script, and PowerShell does not rewrite
    arguments, so every /mnt/... and /root/... path below reaches wsl.exe
    intact. If you run the equivalent commands from Git Bash or any other
    MSYS2 shell, MSYS rewrites anything that looks like a Unix absolute path
    into a Windows path BEFORE wsl.exe sees it:

        $ wsl -d Ubuntu-22.04 -u root -- ls /mnt/c/Users
        ls: cannot access 'C:/Users': No such file or directory

    The fix is to set MSYS_NO_PATHCONV=1 for the call:

        MSYS_NO_PATHCONV=1 wsl -d Ubuntu-22.04 -u root -- ls /mnt/c/Users

    A single leading slash is enough to trigger it, so `-c '/root/...'` and
    `--from 20` are both at risk (the second only if it ever grows a slash).
    Prefer PowerShell for anything that drives WSL.
#>

[CmdletBinding()]
param(
    [string]$Distro = 'Ubuntu-22.04',
    [string]$RepoPath = '',
    [ValidatePattern('^([0-9]{1,2})?$')]
    [string]$From = '',
    [switch]$Fresh,
    [switch]$NoDownload,
    [int]$MinFreeGB = 20,
    [switch]$Force,
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'

# WSL_UTF8 makes wsl.exe emit UTF-8 instead of UTF-16LE for its own messages.
# Without it, `wsl --list` output arrives with a NUL between every character
# and every -match/-eq against it silently fails. Harmless on builds that do
# not support it.
$env:WSL_UTF8 = '1'

# State lives in /var/tmp inside the distro, NOT in the repo's .build/.
# .build/ is the directory an operator deletes to reclaim disk, and it is the
# directory that holds live bind mounts; the lock and the heartbeat have to
# outlive both of those.
$StateDir   = '/var/tmp/fpms-os-build'
$LockPath   = "$StateDir/build.lock"
$RunnerPath = "$StateDir/runner.sh"
$HeartPath  = "$StateDir/heartbeat"
$LastPath   = "$StateDir/last-run"

# --------------------------------------------------------------------------
# output helpers
# --------------------------------------------------------------------------
function Write-Head { param([string]$m) Write-Host ''; Write-Host "==> $m" -ForegroundColor Cyan }
function Write-Note { param([string]$m) Write-Host "    $m" }
function Write-Ok   { param([string]$m) Write-Host "    OK   $m" -ForegroundColor Green }
function Write-Warn2{ param([string]$m) Write-Host "    WARN $m" -ForegroundColor Yellow }
function Fail       { param([string]$m) Write-Host ''; Write-Host "REFUSING TO START: $m" -ForegroundColor Red; Write-Host ''; exit 1 }

# --------------------------------------------------------------------------
# Invoke-WslBash - run a bash script inside the distro and capture it.
#
# The script is fed on STDIN, not as an argument. That is deliberate: it means
# the wsl.exe argument vector is only simple tokens (-d, name, -u, root, -e,
# /bin/bash, -s), so there is no quoting layer between PowerShell, wsl.exe's
# CommandLineToArgvW parsing, and bash. Embedding a script as a `-c '...'`
# argument is where this kind of tooling usually breaks.
#
# The temp file is written with LF endings and UTF-8 without BOM. A BOM at the
# top of a bash script is a syntax error, and CRLF gives you the classic
# `$'\r': command not found`.
# --------------------------------------------------------------------------
function Invoke-WslBash {
    param(
        [Parameter(Mandatory=$true)][string]$Script,
        [int]$TimeoutSec = 120,
        [string]$DistroName = $Distro
    )

    $inFile  = [IO.Path]::GetTempFileName()
    $outFile = [IO.Path]::GetTempFileName()
    $errFile = [IO.Path]::GetTempFileName()
    try {
        $body = $Script -replace "`r`n", "`n"
        [IO.File]::WriteAllText($inFile, $body, (New-Object Text.UTF8Encoding($false)))

        $p = Start-Process -FilePath 'wsl.exe' `
                           -ArgumentList @('-d', $DistroName, '-u', 'root', '-e', '/bin/bash', '-s') `
                           -NoNewWindow -PassThru `
                           -RedirectStandardInput  $inFile `
                           -RedirectStandardOutput $outFile `
                           -RedirectStandardError  $errFile

        if (-not $p.WaitForExit($TimeoutSec * 1000)) {
            try { $p.Kill() } catch { }
            return [pscustomobject]@{ TimedOut = $true; ExitCode = -1; Out = ''; Err = "timed out after ${TimeoutSec}s" }
        }

        $o = ''
        $e = ''
        if ((Get-Item $outFile).Length -gt 0) { $o = [IO.File]::ReadAllText($outFile) }
        if ((Get-Item $errFile).Length -gt 0) { $e = [IO.File]::ReadAllText($errFile) }
        return [pscustomobject]@{ TimedOut = $false; ExitCode = $p.ExitCode; Out = $o; Err = $e }
    }
    finally {
        foreach ($f in @($inFile, $outFile, $errFile)) {
            Remove-Item $f -Force -ErrorAction SilentlyContinue
        }
    }
}

# Pull KEY=VALUE lines out of a probe's output.
function Get-Kv {
    param([string]$Text, [string]$Key)
    foreach ($line in ($Text -split "`r?`n")) {
        if ($line -like "$Key=*") { return $line.Substring($Key.Length + 1) }
    }
    return $null
}

# --------------------------------------------------------------------------
# Windows-side facts. None of this invokes wsl.exe, so it is also what
# -DryRun can report and what build-status.ps1 can report when the distro is
# not running.
# --------------------------------------------------------------------------
function Get-WslDistroKey {
    param([string]$Name)
    $root = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Lxss'
    if (-not (Test-Path $root)) { return $null }
    foreach ($k in Get-ChildItem $root -ErrorAction SilentlyContinue) {
        $p = Get-ItemProperty $k.PSPath -ErrorAction SilentlyContinue
        if ($p -and $p.DistributionName -eq $Name) { return $p }
    }
    return $null
}

function Get-VhdxInfo {
    param([string]$Name)
    $key = Get-WslDistroKey -Name $Name
    $base = $null
    if ($key -and $key.BasePath) { $base = ([string]$key.BasePath).Replace('\\?\', '') }
    if (-not $base) { $base = $env:LOCALAPPDATA }   # last resort, right volume at least

    $vhdx = Join-Path $base 'ext4.vhdx'
    $vhdxGB = $null
    if (Test-Path $vhdx) { $vhdxGB = [math]::Round((Get-Item $vhdx).Length / 1GB, 1) }

    $root = [IO.Path]::GetPathRoot($base)
    $freeGB = $null
    try {
        $d = Get-PSDrive -Name $root.Substring(0,1) -ErrorAction Stop
        $freeGB = [math]::Round($d.Free / 1GB, 1)
    } catch {
        try {
            $ld = Get-CimInstance Win32_LogicalDisk -Filter "DeviceID='$($root.Substring(0,2))'" -ErrorAction Stop
            $freeGB = [math]::Round($ld.FreeSpace / 1GB, 1)
        } catch { }
    }

    return [pscustomobject]@{
        Exists  = [bool]$key
        BasePath = $base
        Vhdx    = $vhdx
        VhdxGB  = $vhdxGB
        Volume  = $root
        FreeGB  = $freeGB
    }
}

# Sleep/hibernate suspends (and in practice usually destroys) the WSL2 VM.
# A 15-hour build on a laptop that sleeps at 30 minutes is a build that dies
# at minute 30, and that failure looks exactly like "something killed it".
function Test-SleepSettings {
    try {
        $q = & powercfg.exe /query SCHEME_CURRENT SUB_SLEEP STANDBYIDLE 2>$null
        $txt = ($q -join "`n")
        $m = [regex]::Match($txt, 'Current AC Power Setting Index:\s*(0x[0-9a-fA-F]+)')
        if ($m.Success) {
            $secs = [Convert]::ToInt64($m.Groups[1].Value, 16)
            return $secs
        }
    } catch { }
    return $null
}

# --------------------------------------------------------------------------
# The runner script that actually lives inside the distro.
#
# It is generated here rather than committed so that it always matches the
# launcher, and it is written to /var/tmp so that deleting the repo's .build/
# (the standard disk-reclaim move) cannot remove the thing that is running.
#
# Placeholders are @@NAME@@ and are substituted below with .Replace(), not
# with -replace, so nothing in a path is treated as a regex.
# --------------------------------------------------------------------------
$RunnerTemplate = @'
#!/bin/bash
# fpms-os detached build runner. GENERATED by tools/run-build-detached.ps1.
# Edit the launcher, not this file - this file is overwritten on every launch.
set -u

# An explicit environment. This process is parented to PID 1 and has no login
# shell behind it, so nothing else is going to set these. /usr/sbin and /sbin
# matter: losetup, parted, e2fsck, blkid and friends live there and build.sh
# calls them by name.
export HOME=/root
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export DEBIAN_FRONTEND=noninteractive
export TERM=dumb
export LC_ALL=C.UTF-8

STATE='@@STATE@@'
REPO='@@REPO@@'
LOCK="$STATE/build.lock"
HB="$STATE/heartbeat"
LAST="$STATE/last-run"
WRAPLOG='@@WRAPLOG@@'

mkdir -p "$STATE"

# The lock is written by the runner itself, with the runner's own PID, AFTER
# it is already detached. A lock written by the launcher would name a PID that
# belongs to a process that has already exited.
{
  echo "pid=$$"
  echo "pgid=$$"
  echo "started=$(date -Is)"
  echo "started_epoch=$(date +%s)"
  echo "repo=$REPO"
  echo "args=@@ARGSDISPLAY@@"
  echo "wrapper_log=$WRAPLOG"
} > "$LOCK"

# Heartbeat. Its only job is to make the difference between "the build died"
# and "the whole distro went away" visible after the fact: if the lock is
# present but the heartbeat is minutes stale, the VM was stopped underneath
# it (wsl --shutdown, sign-out, sleep, host reboot) and no log will say so.
# It also guarantees at least one live process in the distro for the moment
# between build.sh exiting and this runner finishing its bookkeeping, which is
# what keeps WSL2 from tearing the instance down mid-write.
( while kill -0 $$ 2>/dev/null; do date +%s > "$HB"; sleep 30; done ) &
HBPID=$!

cd "$REPO" || { echo "runner: cannot cd to $REPO"; exit 1; }

echo "======================================================================"
echo " fpms-os detached build"
echo "   runner pid : $$"
echo "   started    : $(date -Is)"
echo "   repo       : $REPO"
echo "   command    : bash ./build.sh @@ARGSDISPLAY@@"
echo "   uname      : $(uname -srm)"
echo "   free (repo): $(df -Pm "$REPO" | awk 'NR==2 {print $4}') MB"
echo "======================================================================"

# build.sh runs in the BACKGROUND and is waited on, rather than exec'd, so
# that a TERM sent to this runner can be forwarded to it. build.sh traps TERM
# and runs cleanup(): unmount the binds in reverse order, then detach the loop
# device. Killing build.sh without giving that trap a chance is how you end up
# with a leaked loop device and an .build/mnt/dev that is still a bind mount
# of the host's /dev - which is how an rm -rf goes catastrophically wrong.
#
# `bash ./build.sh` rather than `./build.sh`: it survives a lost exec bit, and
# the tree is authored on Windows.
bash ./build.sh @@BUILDARGS@@ &
BPID=$!
echo "build_pid=$BPID" >> "$LOCK"

forward() { echo "runner: forwarding SIG$1 to build.sh pid $BPID"; kill -"$1" "$BPID" 2>/dev/null; }
trap 'forward TERM' TERM
trap 'forward INT'  INT

# `wait` returns early when a trapped signal arrives, so loop until the child
# is genuinely gone or the exit status here is meaningless.
rc=0
wait "$BPID"; rc=$?
while kill -0 "$BPID" 2>/dev/null; do
    wait "$BPID"; rc=$?
done

fin_epoch=$(date +%s)
start_epoch=$(awk -F= '/^started_epoch=/{print $2; exit}' "$LOCK" 2>/dev/null)
[ -n "${start_epoch:-}" ] || start_epoch=$fin_epoch
el=$(( fin_epoch - start_epoch ))

{
  echo "rc=$rc"
  echo "finished=$(date -Is)"
  echo "finished_epoch=$fin_epoch"
  echo "elapsed_seconds=$el"
  echo "elapsed=$(( el / 3600 ))h$(( (el % 3600) / 60 ))m"
  echo "repo=$REPO"
  echo "args=@@ARGSDISPLAY@@"
  echo "wrapper_log=$WRAPLOG"
} > "$LAST"

echo "======================================================================"
echo " build.sh exited rc=$rc after $(( el / 3600 ))h$(( (el % 3600) / 60 ))m"
echo " finished: $(date -Is)"
echo "======================================================================"

# Drop the lock LAST, and only after build.sh is really gone, so that the
# "is one already running?" check can never see a window in which a live
# build looks finished.
rm -f "$LOCK"
kill "$HBPID" 2>/dev/null
exit "$rc"
'@

# --------------------------------------------------------------------------
# Pre-flight: Windows side
# --------------------------------------------------------------------------
Write-Head "pre-flight (Windows side)"

$vhdx = Get-VhdxInfo -Name $Distro
if (-not $vhdx.Exists) {
    Fail @"
no WSL distro named '$Distro' is registered for this user.
  Registered distros:
$( (Get-ChildItem 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Lxss' -ErrorAction SilentlyContinue |
      ForEach-Object { (Get-ItemProperty $_.PSPath).DistributionName } |
      ForEach-Object { "    - $_" }) -join "`n" )
  Pass the right one with -Distro <name>.
"@
}
Write-Ok "distro '$Distro' is registered"
Write-Note "VHDX:   $($vhdx.Vhdx)"
if ($null -ne $vhdx.VhdxGB) { Write-Note "        currently $($vhdx.VhdxGB) GB (it grows; it never shrinks by itself)" }

if ($null -eq $vhdx.FreeGB) {
    Write-Warn2 "could not read free space on $($vhdx.Volume) - check it by hand before you walk away"
} else {
    Write-Note "free on $($vhdx.Volume) $($vhdx.FreeGB) GB   (required: $MinFreeGB GB)"
    if ($vhdx.FreeGB -lt $MinFreeGB) {
        Fail @"
only $($vhdx.FreeGB) GB free on $($vhdx.Volume), and the build needs about $MinFreeGB GB.

This is the most likely non-code way the build dies, and it dies badly: the
guest sees I/O errors that look like filesystem corruption, hours in.
Free space only ever goes DOWN during a build - the ext4.vhdx grows to cover
every byte written inside the distro and does not shrink again when files are
deleted.

Where it dies, by free space (docs/BUILDING.md section 2):
    < ~6 GB    decompressing the base image - early and obvious
    ~6-12 GB   stage 10, mid-apt, as 'No space left on device'
    ~12-17 GB  stage 90's zero-fill or the final xz

Reclaim space, then (with no build running):
    wsl --shutdown
    Optimize-VHD -Path '$($vhdx.Vhdx)' -Mode Full      # Hyper-V module, or:
    diskpart  ->  select vdisk file="$($vhdx.Vhdx)"  ->  compact vdisk

Or lower the bar deliberately with -MinFreeGB <n>, knowing the table above.
"@
    }
    Write-Ok "free space"
}

$standby = Test-SleepSettings
if ($null -ne $standby -and $standby -gt 0) {
    Write-Warn2 "this machine sleeps after $([math]::Round($standby/60)) minutes on AC power."
    Write-Warn2 "Sleep suspends the WSL2 VM and in practice usually kills it. A 15-hour"
    Write-Warn2 "build will not survive it. Disable it before walking away:"
    Write-Warn2 "    powercfg /change standby-timeout-ac 0"
    Write-Warn2 "    powercfg /change hibernate-timeout-ac 0"
} elseif ($null -ne $standby) {
    Write-Ok "AC sleep timeout is 0 (never) - the VM will not be suspended out from under the build"
}

$wslconfig = Join-Path $env:USERPROFILE '.wslconfig'
if (Test-Path $wslconfig) {
    $wc = Get-Content $wslconfig -Raw
    if ($wc -match '(?im)^\s*vmIdleTimeout\s*=\s*(\d+)') {
        Write-Warn2 "$wslconfig sets vmIdleTimeout=$($Matches[1]) ms. That governs how long the VM"
        Write-Warn2 "lingers with nothing running in it; a running build keeps it alive, but if you"
        Write-Warn2 "want no doubt at all set vmIdleTimeout=-1 while building."
    }
}

if ($DryRun) {
    Write-Head "-DryRun: stopping before anything touches wsl.exe"
    $argsList = @()
    if ($From)       { $argsList += @('--from', $From) }
    if ($Fresh)      { $argsList += '--fresh' }
    if ($NoDownload) { $argsList += '--no-download' }
    $argsDisplay = ($argsList -join ' ')
    $repoShown = $RepoPath
    if (-not $repoShown) { $repoShown = '<auto-detected inside the distro>' }
    Write-Note "distro     : $Distro"
    Write-Note "repo       : $repoShown"
    Write-Note "build args : $(if ($argsDisplay) { $argsDisplay } else { '<none - full build>' })"
    Write-Note "runner     : $RunnerPath"
    Write-Note "lock       : $LockPath"
    Write-Host ''
    Write-Host '--- runner.sh that would be installed -------------------------------'
    $RunnerTemplate.Replace('@@STATE@@', $StateDir).
                    Replace('@@REPO@@', $repoShown).
                    Replace('@@WRAPLOG@@', "$repoShown/.build/detached-<timestamp>.log").
                    Replace('@@ARGSDISPLAY@@', $argsDisplay).
                    Replace('@@BUILDARGS@@', $argsDisplay) | Write-Host
    Write-Host '---------------------------------------------------------------------'
    exit 0
}

# --------------------------------------------------------------------------
# Pre-flight: inside the distro. One round trip.
# --------------------------------------------------------------------------
Write-Head "pre-flight (inside $Distro)"

# Candidate repo locations, most specific first. docs/BUILDING.md uses
# /root/work/rover/fpms-os in its worked example and refers to /root/fpms-build
# as the tree that held the 14-hour image.
$candidates = @(
    '/root/fpms-build/rover/fpms-os',
    '/root/fpms-build/fpms-os',
    '/root/work/rover/fpms-os',
    '/root/rover/fpms-os',
    '/root/fpms-os'
)
if ($RepoPath) { $candidates = @($RepoPath) + $candidates }

$probe = @"
set -u
echo "PROBE_OK=1"
echo "WHOAMI=`$(id -un)"
echo "UIDNUM=`$(id -u)"
echo "ARCH=`$(uname -m)"
echo "KERNEL=`$(uname -r)"

REG=/proc/sys/fs/binfmt_misc/qemu-aarch64
if [ -e "`$REG" ]; then
    echo "BINFMT=yes"
    echo "BINFMT_INTERP=`$(awk '/^interpreter /{print `$2; exit}' "`$REG" 2>/dev/null)"
    echo "BINFMT_FLAGS=`$(awk -F':[[:space:]]*' '/^flags:/{print `$2; exit}' "`$REG" 2>/dev/null)"
    echo "BINFMT_ENABLED=`$(head -1 "`$REG" 2>/dev/null)"
else
    echo "BINFMT=no"
fi
if [ -x /usr/bin/qemu-aarch64-static ]; then echo "QEMU_STATIC=yes"; else echo "QEMU_STATIC=no"; fi

MISSING=
for t in sgdisk parted e2fsck resize2fs blkid partx blockdev losetup mountpoint xz wget sha256sum setsid; do
    command -v "`$t" >/dev/null 2>&1 || MISSING="`$MISSING `$t"
done
echo "MISSING_TOOLS=`$MISSING"

REPO=
for c in $($candidates -join ' '); do
    if [ -f "`$c/build.sh" ]; then REPO="`$c"; break; fi
done
if [ -z "`$REPO" ]; then
    # Bounded, and -xdev so it can never walk into a mounted image or a /dev
    # bind mount left behind by a dead build.
    for c in `$(find /root /home /opt -xdev -maxdepth 6 -type f -path '*/fpms-os/build.sh' 2>/dev/null | head -3); do
        echo "REPO_CANDIDATE=`$(dirname "`$c")"
    done
fi
echo "REPO=`$REPO"
if [ -n "`$REPO" ]; then
    echo "REPO_FS=`$(df -PT "`$REPO" 2>/dev/null | awk 'NR==2 {print `$2}')"
    echo "REPO_FREE_MB=`$(df -Pm "`$REPO" 2>/dev/null | awk 'NR==2 {print `$4}')"
    echo "REPO_PARENT_OK=`$([ -d "`$REPO/../stack" ] && echo yes || echo no)"
    echo "REPO_CRLF=`$(head -1 "`$REPO/build.sh" | grep -c `$'\r')"
    echo "IMAGES=`$(ls -1 "`$REPO"/fpms-os-*.img "`$REPO"/fpms-os-*.img.xz 2>/dev/null | tr '\n' ',')"
fi

# Is a build already running? Three independent questions, because the answer
# has to be right even for a build somebody started by hand.
if [ -f '$LockPath' ]; then
    LP=`$(awk -F= '/^pid=/{print `$2; exit}' '$LockPath' 2>/dev/null)
    echo "LOCK=yes"
    echo "LOCK_PID=`${LP:-}"
    if [ -n "`${LP:-}" ] && kill -0 "`$LP" 2>/dev/null; then
        echo "LOCK_PID_ALIVE=yes"
        echo "LOCK_PID_CMD=`$(tr '\0' ' ' < /proc/`$LP/cmdline 2>/dev/null)"
    else
        echo "LOCK_PID_ALIVE=no"
    fi
    echo "LOCK_BODY_BEGIN=1"
    cat '$LockPath'
    echo "LOCK_BODY_END=1"
else
    echo "LOCK=no"
fi
echo "PGREP_BUILD=`$(pgrep -f '[b]uild\.sh' 2>/dev/null | tr '\n' ' ')"
echo "MOUNTED_MNT=`$(mount | grep -c '/\.build/mnt' 2>/dev/null)"
echo "LOOP_ATTACHED=`$(losetup -a 2>/dev/null | grep -c 'fpms-os-.*\.img')"
echo "HEARTBEAT=`$(cat '$HeartPath' 2>/dev/null)"
echo "NOW=`$(date +%s)"
"@

$r = Invoke-WslBash -Script $probe -TimeoutSec 90
if ($r.TimedOut) {
    Fail @"
the pre-flight probe did not come back within 90 seconds.

That is not a slow disk - a probe that only reads /proc and runs df should
answer instantly. Either the distro is starting from cold and something in
root's profile is blocking, or the WSL VM is wedged. Check with:
    wsl.exe --list --running
    wsl.exe -d $Distro -u root -e /bin/true
"@
}
if ($r.ExitCode -ne 0 -and -not (Get-Kv $r.Out 'PROBE_OK')) {
    Fail @"
could not run bash as root in '$Distro' (wsl.exe exited $($r.ExitCode)).
$($r.Err)
Check by hand:
    wsl.exe -d $Distro -u root -e id
"@
}

$out = $r.Out
if ((Get-Kv $out 'UIDNUM') -ne '0') {
    Fail "wsl -u root did not give uid 0 (got '$(Get-Kv $out 'WHOAMI')'). build.sh needs real root for loop devices and chroot."
}
Write-Ok "root shell in '$Distro' (uid 0, $(Get-Kv $out 'ARCH'), kernel $(Get-Kv $out 'KERNEL'))"

# --- binfmt / qemu ------------------------------------------------------
$arch = Get-Kv $out 'ARCH'
if ($arch -ne 'aarch64') {
    if ((Get-Kv $out 'BINFMT') -ne 'yes') {
        Fail @"
qemu-aarch64 is not registered with binfmt_misc, so every command inside the
chroot would fail with 'Exec format error'.

    wsl -d $Distro -u root -e bash -lc 'apt-get install -y qemu-user-static binfmt-support'
  or, if that distro has ever run Docker Desktop with multi-arch enabled:
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

build.sh checks this too, but it checks it after the base image download.
"@
    }
    $interp = Get-Kv $out 'BINFMT_INTERP'
    $flags  = Get-Kv $out 'BINFMT_FLAGS'
    Write-Ok "binfmt_misc has qemu-aarch64  (interpreter: $interp, flags: $flags)"

    # The trap docs/BUILDING.md records: a distro that has run Docker Desktop
    # with multi-arch enabled has the binfmt REGISTRATION but not the package,
    # and the two look alike until stage 00 dies.
    if ((Get-Kv $out 'QEMU_STATIC') -ne 'yes' -and ($flags -notlike '*F*')) {
        Fail @"
binfmt_misc has a qemu-aarch64 registration but /usr/bin/qemu-aarch64-static
does not exist, and the registration does not carry the F (fix-binary) flag -
so the kernel is NOT holding the interpreter open and build.sh has nothing to
copy into the image.

This is the Docker-Desktop-multiarch case: the registration is real, the
package is not.
    wsl -d $Distro -u root -e bash -lc 'apt-get install -y qemu-user-static'
"@
    }
    if ($flags -like '*F*') { Write-Note "the F flag is set - the kernel holds the interpreter open, nothing needs copying into the image" }
} else {
    Write-Ok "native aarch64 host - no emulation needed (and no 14-hour stage 10)"
}

$missing = Get-Kv $out 'MISSING_TOOLS'
if ($missing -and $missing.Trim()) {
    Fail @"
these tools are missing inside the distro and build.sh needs every one of
them:$missing

    wsl -d $Distro -u root -e bash -lc 'apt-get install -y qemu-user-static binfmt-support xz-utils parted e2fsprogs dosfstools kpartx gdisk util-linux udev wget'
"@
}
Write-Ok "build tool-chain present (losetup, parted, sgdisk, resize2fs, xz, setsid, ...)"

# --- the repo -----------------------------------------------------------
$repo = Get-Kv $out 'REPO'
if (-not $repo) {
    $cands = @()
    foreach ($line in ($out -split "`r?`n")) {
        if ($line -like 'REPO_CANDIDATE=*') { $cands += $line.Substring(15) }
    }
    $hint = ''
    if ($cands.Count) { $hint = "`n  A search did find build.sh here:`n" + (($cands | ForEach-Object { "    -RepoPath $_" }) -join "`n") }
    Fail @"
no fpms-os checkout with a build.sh was found inside '$Distro'.
  Looked at:
$(($candidates | ForEach-Object { "    $_" }) -join "`n")$hint

The tree must live on the distro's OWN ext4, never on /mnt/c: 9p turns the
emulated apt stages into something that does not finish, and loop-mounting an
image file across 9p is unreliable. Copy the whole rover/ tree, not just
fpms-os/ - stage 30 stages source from two levels up:

    wsl -d $Distro -u root
    mkdir -p /root/fpms-build
    cp -a /mnt/c/Users/$env:USERNAME/fpms-os-wt/cloud/dashboard/rover /root/fpms-build/
"@
}
Write-Ok "repo: $repo  ($(Get-Kv $out 'REPO_FS'), $(Get-Kv $out 'REPO_FREE_MB') MB free)"

if ($repo -like '/mnt/*') {
    Fail @"
$repo is on a Windows drive mounted into WSL (9p/drvfs).

Do not build there. Two independent reasons, both recorded in
docs/BUILDING.md section 1:
  - every one of the tens of thousands of small file creates an emulated apt
    does becomes a 9p round trip; stages that take tens of minutes on ext4 do
    not finish;
  - loop-mounting a file across 9p is unreliable, and it fails as e2fsck or
    resize2fs reporting damage that is not in the file.
Copy the tree onto the distro's ext4 and point -RepoPath at it.
"@
}

if ((Get-Kv $out 'REPO_PARENT_OK') -eq 'no') {
    Write-Warn2 "$repo/../stack does not exist. build.sh stages the rover source from TWO"
    Write-Warn2 "levels up; if that tree is not there the build runs for hours and then dies"
    Write-Warn2 "in stage 30 with 'MISSING: fpms_missions.py'. Copy the whole rover/ tree."
}
if ((Get-Kv $out 'REPO_CRLF') -eq '1') {
    Write-Warn2 "build.sh's first line has a CR in it (CRLF line endings). Expect"
    Write-Warn2 ('$' + "'\r': command not found. Fix with:  sed -i 's/\r`$//' $repo/build.sh")
}

$repoFreeMb = 0
[int]::TryParse((Get-Kv $out 'REPO_FREE_MB'), [ref]$repoFreeMb) | Out-Null
if ($repoFreeMb -gt 0 -and $repoFreeMb -lt ($MinFreeGB * 1024)) {
    Write-Warn2 "the distro's own filesystem reports only $([math]::Round($repoFreeMb/1024,1)) GB free."
    Write-Warn2 "df inside WSL and free space on $($vhdx.Volume) are different questions and you need both."
}

# --------------------------------------------------------------------------
# Refuse to start a second build.
#
# Two builds against one image and one loop device is a corrupted image and a
# wedged host, and it is not recoverable by retrying - so this check is not
# overridable by -Force.
# --------------------------------------------------------------------------
Write-Head "checking that nothing is already building"

$lockAlive = ((Get-Kv $out 'LOCK') -eq 'yes' -and (Get-Kv $out 'LOCK_PID_ALIVE') -eq 'yes')
$pgrep     = (Get-Kv $out 'PGREP_BUILD')
$mounted   = 0; [int]::TryParse((Get-Kv $out 'MOUNTED_MNT'), [ref]$mounted) | Out-Null
$loops     = 0; [int]::TryParse((Get-Kv $out 'LOOP_ATTACHED'), [ref]$loops) | Out-Null

if ($lockAlive -or ($pgrep -and $pgrep.Trim())) {
    $body = ''
    $inBody = $false
    foreach ($line in ($out -split "`r?`n")) {
        if ($line -eq 'LOCK_BODY_END=1') { $inBody = $false }
        if ($inBody) { $body += "    $line`n" }
        if ($line -eq 'LOCK_BODY_BEGIN=1') { $inBody = $true }
    }
    Fail @"
a build is ALREADY RUNNING in '$Distro'. Not starting a second one.

  lock pid  : $(Get-Kv $out 'LOCK_PID')  (alive: $(Get-Kv $out 'LOCK_PID_ALIVE'))
  build.sh  : pid(s) $pgrep
$body
Two builds sharing one image file and one loop device corrupt the image and
wedge the host, and neither symptom points back at the cause. There is no
-Force for this.

  See what it is doing:   .\tools\build-status.ps1
  Stop it cleanly:        see "Stopping a build" in tools\README.md
"@
}

if ((Get-Kv $out 'LOCK') -eq 'yes') {
    Write-Warn2 "a stale lock is present (pid $(Get-Kv $out 'LOCK_PID') is gone). A previous build died"
    Write-Warn2 "without cleaning up - most likely the distro was stopped underneath it."
}
if ($mounted -gt 0 -or $loops -gt 0) {
    Write-Warn2 "leftovers from a dead build: $mounted mount(s) under .build/mnt, $loops loop device(s) on an image."
    Write-Warn2 "build.sh's release_stale() unmounts and detaches these at startup, so this is"
    Write-Warn2 "survivable - but do NOT rm -rf .build while they exist. .build/mnt/dev is a"
    Write-Warn2 "bind mount of the host's /dev and rm -rf will walk straight into it."
}
if (-not $lockAlive -and -not ($pgrep -and $pgrep.Trim())) { Write-Ok "nothing is building" }

# --------------------------------------------------------------------------
# -Fresh guard
# --------------------------------------------------------------------------
$images = Get-Kv $out 'IMAGES'
if ($Fresh -and $images -and $images.Trim(',').Trim() -and -not $Force) {
    Fail @"
-Fresh throws away the existing image and starts again from the vendor base,
and there IS an existing image:

$((($images.Trim(',') -split ',') | ForEach-Object { "    $_" }) -join "`n")

Stage 10 alone is 14 hours and nothing caches it. If you meant to resume
against that image, drop -Fresh:

    .\tools\run-build-detached.ps1 -From $(if ($From) { $From } else { '<stage>' })

If you really do mean to discard it, add -Force.
"@
}

# --------------------------------------------------------------------------
# Launch
# --------------------------------------------------------------------------
$argsList = @()
if ($From)       { $argsList += @('--from', $From) }
if ($Fresh)      { $argsList += '--fresh' }
if ($NoDownload) { $argsList += '--no-download' }
$argsDisplay = ($argsList -join ' ')

$stamp    = Get-Date -Format 'yyyyMMdd-HHmmss'
$wrapLog  = "$repo/.build/detached-$stamp.log"

$runner = $RunnerTemplate.
            Replace('@@STATE@@',       $StateDir).
            Replace('@@REPO@@',        $repo).
            Replace('@@WRAPLOG@@',     $wrapLog).
            Replace('@@ARGSDISPLAY@@', $argsDisplay).
            Replace('@@BUILDARGS@@',   $argsDisplay)

Write-Head "launching"
Write-Note "command : bash ./build.sh $argsDisplay"
Write-Note "cwd     : $repo"
Write-Note "log     : $wrapLog"

# The launch script itself. Everything here finishes in well under a second;
# the point is that this wsl.exe call RETURNS, leaving no Windows process
# behind for anything to kill.
#
# The one line that matters:
#
#     setsid --fork /usr/bin/nohup /bin/bash RUNNER </dev/null >>LOG 2>&1
#
#   setsid --fork  new session, new process group, no controlling terminal,
#                  and - because setsid's parent exits immediately rather
#                  than waiting - the runner is orphaned and reparented to
#                  PID 1 (/init) within milliseconds. It is a daemon.
#   nohup          belt and braces: SIGHUP ignored, and the disposition is
#                  inherited across every exec, so build.sh and all nine
#                  stages inherit it too.
#   </dev/null     stdin is a device, not the pipe wsl.exe handed us. A build
#                  that reads stdin gets EOF instead of blocking forever on a
#                  descriptor whose writer has gone.
#   >>LOG 2>&1     THE IMPORTANT ONE. stdout and stderr are a FILE on the
#                  distro's ext4. This is what the previous attempts did not
#                  do: `nohup` only redirects stdout when stdout is a tty, so
#                  a build launched from a script inherited the caller's PIPE
#                  and died of SIGPIPE the moment the caller exited.
$launch = @"
set -e
mkdir -p '$StateDir'
mkdir -p '$repo/.build'
cat > '$RunnerPath' <<'FPMS_RUNNER_EOF'
$runner
FPMS_RUNNER_EOF
chmod +x '$RunnerPath'

# Fail loudly here rather than produce a runner that dies on its first line.
bash -n '$RunnerPath' || { echo "LAUNCH_ERR=runner failed bash -n"; exit 1; }

command -v setsid >/dev/null 2>&1 || { echo "LAUNCH_ERR=setsid not found (apt install util-linux)"; exit 1; }
NOHUP=`$(command -v nohup || true)
[ -n "`$NOHUP" ] || { echo "LAUNCH_ERR=nohup not found (apt install coreutils)"; exit 1; }

setsid --fork "`$NOHUP" /bin/bash '$RunnerPath' </dev/null >>'$wrapLog' 2>&1

# Wait for the runner to write its own lock, then report what it actually
# became. This is the evidence that it is detached, gathered while we are
# still here to print it.
for i in `$(seq 1 40); do
    [ -f '$LockPath' ] && break
    sleep 0.5
done
if [ ! -f '$LockPath' ]; then
    echo "LAUNCH_ERR=the runner never wrote $LockPath"
    echo "LOGTAIL_BEGIN=1"; tail -40 '$wrapLog' 2>/dev/null; echo "LOGTAIL_END=1"
    exit 1
fi
P=`$(awk -F= '/^pid=/{print `$2; exit}' '$LockPath')
B=`$(awk -F= '/^build_pid=/{print `$2; exit}' '$LockPath')
echo "LAUNCH_OK=1"
echo "RUNNER_PID=`$P"
echo "BUILD_PID=`${B:-}"
echo "PPID=`$(ps -o ppid= -p `$P 2>/dev/null | tr -d ' ')"
echo "SID=`$(ps -o sid= -p `$P 2>/dev/null | tr -d ' ')"
echo "PGID=`$(ps -o pgid= -p `$P 2>/dev/null | tr -d ' ')"
echo "TTY=`$(ps -o tty= -p `$P 2>/dev/null | tr -d ' ')"
echo "FD0=`$(readlink /proc/`$P/fd/0 2>/dev/null)"
echo "FD1=`$(readlink /proc/`$P/fd/1 2>/dev/null)"
echo "FD2=`$(readlink /proc/`$P/fd/2 2>/dev/null)"
echo "LOGTAIL_BEGIN=1"; tail -25 '$wrapLog' 2>/dev/null; echo "LOGTAIL_END=1"
"@

$lr = Invoke-WslBash -Script $launch -TimeoutSec 120
if ($lr.TimedOut) {
    Fail "the launch call did not return within 120 s. Check with build-status.ps1 - it is possible the build DID start."
}

$lout = $lr.Out
if (-not (Get-Kv $lout 'LAUNCH_OK')) {
    Write-Host $lout
    Write-Host $lr.Err -ForegroundColor Red
    Fail "the build did not start. $(Get-Kv $lout 'LAUNCH_ERR')"
}

$pid_    = Get-Kv $lout 'RUNNER_PID'
$bpid    = Get-Kv $lout 'BUILD_PID'
$ppid_   = Get-Kv $lout 'PPID'
$tty     = Get-Kv $lout 'TTY'
$fd0     = Get-Kv $lout 'FD0'
$fd1     = Get-Kv $lout 'FD1'
$fd2     = Get-Kv $lout 'FD2'

Write-Head "started - and here is the proof it is detached"
Write-Note "runner pid   : $pid_   (build.sh pid: $bpid)"
Write-Note "parent pid   : $ppid_"
Write-Note "session/pgid : $(Get-Kv $lout 'SID') / $(Get-Kv $lout 'PGID')"
Write-Note "tty          : $(if ($tty -and $tty -ne '?') { $tty } else { '<none>' })"
Write-Note "fd 0 (stdin) : $fd0"
Write-Note "fd 1 (stdout): $fd1"
Write-Note "fd 2 (stderr): $fd2"

$clean = $true
if ($ppid_ -ne '1') {
    Write-Warn2 "PPID is $ppid_, not 1. The runner has not been reparented to init - it is still"
    Write-Warn2 "a child of something. That is exactly the state that dies when the caller exits."
    $clean = $false
}
if ($pid_ -ne (Get-Kv $lout 'SID')) {
    Write-Warn2 "the runner is not a session leader (pid $pid_, sid $(Get-Kv $lout 'SID')). setsid did not take."
    $clean = $false
}
if ($tty -and $tty -ne '?' -and $tty -ne '-') {
    Write-Warn2 "the runner still has a controlling terminal ($tty). It will get SIGHUP when that closes."
    $clean = $false
}
foreach ($fd in @(@('stdout', $fd1), @('stderr', $fd2))) {
    if ($fd[1] -like 'pipe:*') {
        Write-Warn2 "$($fd[0]) is $($fd[1]) - A PIPE. This is the failure mode that killed the previous"
        Write-Warn2 "builds: when whatever holds the other end exits, the next write raises SIGPIPE"
        Write-Warn2 "and the build dies silently, hours in. Stop the build and fix the launcher."
        $clean = $false
    }
}
if ($fd0 -ne '/dev/null') {
    Write-Warn2 "stdin is $fd0, not /dev/null."
    $clean = $false
}
if ($clean) {
    Write-Ok "session leader, parented to init, no tty, all three fds are files or /dev/null."
    Write-Ok "Nothing on the Windows side owns a descriptor in this process."
}

$tail = @()
$inTail = $false
foreach ($line in ($lout -split "`r?`n")) {
    if ($line -eq 'LOGTAIL_END=1') { $inTail = $false }
    if ($inTail) { $tail += $line }
    if ($line -eq 'LOGTAIL_BEGIN=1') { $inTail = $true }
}
if ($tail.Count) {
    Write-Head "first lines of the log"
    $tail | ForEach-Object { Write-Host "    $_" -ForegroundColor DarkGray }
}

Write-Head "you can close this window now"
Write-Note "The launcher's wsl.exe has already exited. There is no Windows process left"
Write-Note "holding this build; closing the terminal, ending this session, or killing the"
Write-Note "agent that ran this cannot reach it."
Write-Host ''
Write-Note "check on it     :  .\tools\build-status.ps1"
Write-Note "watch it live   :  wsl -d $Distro -u root -e tail -f '$wrapLog'"
Write-Note "stop it cleanly :  see 'Stopping a build' in tools\README.md"
Write-Host ''
Write-Warn2 'wsl --shutdown, signing out of Windows, sleep and a reboot all kill the VM'
Write-Warn2 "and therefore the build, whatever it is parented to. Expect 15-20 hours."
Write-Host ''
exit 0

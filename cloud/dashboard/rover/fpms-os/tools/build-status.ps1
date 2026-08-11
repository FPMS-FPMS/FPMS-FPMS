<#
.SYNOPSIS
    Report on a detached FPMS-OS build: running or not, which stage, how long
    each finished stage took, the end of the log, whether an image exists, and
    free space. Read-only, and cheap enough to run every minute.

.DESCRIPTION
    Safe at any time, by construction:
      - It never writes anything, never mounts anything, and never signals
        anything.
      - It asks Windows first whether the distro is even running, and if it is
        not, it does NOT invoke into it. `wsl -d X -e ...` BOOTS a stopped
        distro, which on a build host is both slow and misleading; a stopped
        distro is already the answer to "is my build running?" (it is not).
      - Everything inside the distro is gathered in ONE round trip with a
        bounded script that only reads /proc, runs df/ls/stat, and tails a
        log. There is no `find` over the image and nothing that touches
        .build/mnt beyond a single stat().
      - Every WSL call has a timeout. A probe that hangs means a wedged VM,
        which is itself a finding, and this script says so rather than
        hanging with it.

    WHAT TO LOOK AT WHEN IT LOOKS FROZEN
    ------------------------------------
    Stage 10 compiles the micro-ROS agent from source under emulation and
    redirects it to /dev/null, so a healthy build prints NOTHING for many
    hours (docs/BUILDING.md section 3). Do not conclude anything from a quiet
    log. This script reports three independent liveness signals instead:
      - heartbeat age (the runner touches it every 30 s),
      - qemu-aarch64-static processes and their CPU time,
      - the mtime of the uros_ws build directory inside the mounted image.
    If those move, it is working. Stage 10 alone was measured at 14h20m.

.PARAMETER Distro
    WSL distro name. Default Ubuntu-22.04.

.PARAMETER RepoPath
    Linux path of the fpms-os checkout inside the distro. Usually unnecessary:
    the running build records it in its lock file and this script reads it
    from there.

.PARAMETER Tail
    How many log lines to show. Default 25.

.NOTES
    GIT BASH / MSYS PATH MANGLING
    -----------------------------
    This is PowerShell, so the /root/... and /var/tmp/... paths below reach
    wsl.exe unmodified. If you run the same commands from Git Bash, MSYS
    rewrites Unix-looking absolute paths into Windows paths first and you get
    'cannot access C:/var/tmp/...'. Prefix the call:

        MSYS_NO_PATHCONV=1 wsl -d Ubuntu-22.04 -u root -e tail -5 /var/tmp/fpms-os-build/build.lock
#>

[CmdletBinding()]
param(
    [string]$Distro = 'Ubuntu-22.04',
    [string]$RepoPath = '',
    [int]$Tail = 25
)

$ErrorActionPreference = 'Stop'
$env:WSL_UTF8 = '1'

$StateDir  = '/var/tmp/fpms-os-build'
$LockPath  = "$StateDir/build.lock"
$HeartPath = "$StateDir/heartbeat"
$LastPath  = "$StateDir/last-run"

function Write-Head { param([string]$m) Write-Host ''; Write-Host "==> $m" -ForegroundColor Cyan }
function Write-Note { param([string]$m) Write-Host "    $m" }
function Write-Ok   { param([string]$m) Write-Host "    $m" -ForegroundColor Green }
function Write-Bad  { param([string]$m) Write-Host "    $m" -ForegroundColor Red }
function Write-Warn2{ param([string]$m) Write-Host "    $m" -ForegroundColor Yellow }

# Identical to the one in run-build-detached.ps1. Duplicated rather than dot-
# sourced on purpose: either script has to work when copied somewhere on its
# own, and a status tool that fails because a sibling file moved is worse than
# forty duplicated lines.
function Invoke-WslBash {
    param(
        [Parameter(Mandatory=$true)][string]$Script,
        [int]$TimeoutSec = 60,
        [string]$DistroName = $Distro
    )
    $inFile  = [IO.Path]::GetTempFileName()
    $outFile = [IO.Path]::GetTempFileName()
    $errFile = [IO.Path]::GetTempFileName()
    try {
        [IO.File]::WriteAllText($inFile, ($Script -replace "`r`n", "`n"), (New-Object Text.UTF8Encoding($false)))
        $p = Start-Process -FilePath 'wsl.exe' `
                           -ArgumentList @('-d', $DistroName, '-u', 'root', '-e', '/bin/bash', '-s') `
                           -NoNewWindow -PassThru `
                           -RedirectStandardInput  $inFile `
                           -RedirectStandardOutput $outFile `
                           -RedirectStandardError  $errFile
        if (-not $p.WaitForExit($TimeoutSec * 1000)) {
            try { $p.Kill() } catch { }
            return [pscustomobject]@{ TimedOut = $true; ExitCode = -1; Out = ''; Err = '' }
        }
        $o = ''; $e = ''
        if ((Get-Item $outFile).Length -gt 0) { $o = [IO.File]::ReadAllText($outFile) }
        if ((Get-Item $errFile).Length -gt 0) { $e = [IO.File]::ReadAllText($errFile) }
        return [pscustomobject]@{ TimedOut = $false; ExitCode = $p.ExitCode; Out = $o; Err = $e }
    }
    finally {
        foreach ($f in @($inFile, $outFile, $errFile)) { Remove-Item $f -Force -ErrorAction SilentlyContinue }
    }
}

function Get-Kv {
    param([string]$Text, [string]$Key)
    foreach ($line in ($Text -split "`r?`n")) {
        if ($line -like "$Key=*") { return $line.Substring($Key.Length + 1) }
    }
    return $null
}

function Get-Block {
    param([string]$Text, [string]$Name)
    $acc = @(); $inside = $false
    foreach ($line in ($Text -split "`r?`n")) {
        if ($line -eq "${Name}_END") { $inside = $false }
        if ($inside) { $acc += $line }
        if ($line -eq "${Name}_BEGIN") { $inside = $true }
    }
    return $acc
}

function Format-Duration {
    param([int]$Seconds)
    if ($Seconds -lt 0) { return '?' }
    $h = [math]::Floor($Seconds / 3600)
    $m = [math]::Floor(($Seconds % 3600) / 60)
    $s = $Seconds % 60
    if ($h -gt 0) { return "${h}h${m}m" }
    if ($m -gt 0) { return "${m}m${s}s" }
    return "${s}s"
}

# --------------------------------------------------------------------------
# Windows side. Never invokes wsl.exe, so this part always answers.
# --------------------------------------------------------------------------
function Get-VhdxInfo {
    param([string]$Name)
    $root = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Lxss'
    $key = $null
    if (Test-Path $root) {
        foreach ($k in Get-ChildItem $root -ErrorAction SilentlyContinue) {
            $p = Get-ItemProperty $k.PSPath -ErrorAction SilentlyContinue
            if ($p -and $p.DistributionName -eq $Name) { $key = $p; break }
        }
    }
    $base = $env:LOCALAPPDATA
    if ($key -and $key.BasePath) { $base = ([string]$key.BasePath).Replace('\\?\', '') }
    $vhdx = Join-Path $base 'ext4.vhdx'
    $vhdxGB = $null
    if (Test-Path $vhdx) { $vhdxGB = [math]::Round((Get-Item $vhdx).Length / 1GB, 1) }
    $vol = [IO.Path]::GetPathRoot($base)
    $freeGB = $null
    try { $freeGB = [math]::Round((Get-PSDrive -Name $vol.Substring(0,1) -ErrorAction Stop).Free / 1GB, 1) } catch { }
    return [pscustomobject]@{ Exists=[bool]$key; Vhdx=$vhdx; VhdxGB=$vhdxGB; Volume=$vol; FreeGB=$freeGB }
}

Write-Host ''
Write-Host "FPMS-OS build status   $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" -ForegroundColor White

$vhdx = Get-VhdxInfo -Name $Distro
Write-Head 'host'
if (-not $vhdx.Exists) {
    Write-Bad "no distro named '$Distro' is registered for this user."
    exit 2
}
if ($null -ne $vhdx.FreeGB) {
    $spaceLine = "free on $($vhdx.Volume) $($vhdx.FreeGB) GB"
    if ($null -ne $vhdx.VhdxGB) { $spaceLine += "   (ext4.vhdx is $($vhdx.VhdxGB) GB and only grows)" }
    if ($vhdx.FreeGB -lt 8)      { Write-Bad  $spaceLine }
    elseif ($vhdx.FreeGB -lt 20) { Write-Warn2 ($spaceLine + '  <- tight; see docs/BUILDING.md section 2') }
    else                         { Write-Note $spaceLine }
} else {
    Write-Warn2 "could not read free space on $($vhdx.Volume)"
}

# Is the distro even up? Asking this first is what keeps the script from
# BOOTING a stopped distro just to be told nothing is running in it.
$running = @()
try {
    $raw = (& wsl.exe --list --running --quiet 2>$null)
    foreach ($l in (($raw -join "`n") -replace "`0", '') -split "`r?`n") {
        $t = $l.Trim()
        if ($t) { $running += $t }
    }
} catch { }

if ($running -notcontains $Distro) {
    Write-Bad "distro '$Distro' is NOT running."
    Write-Host ''
    Write-Note 'A stopped distro means no build is running - WSL2 stops the VM when the last'
    Write-Note 'process in it exits, and `wsl --shutdown` / signing out / sleep / a reboot all'
    Write-Note 'stop it regardless of what was running.'
    Write-Note ''
    Write-Note 'Nothing was started to answer this, and nothing was lost by asking: the build'
    Write-Note 'log survives on the distro ext4. To read it, start the distro deliberately:'
    Write-Note "    wsl -d $Distro -u root -e ls -lt /var/tmp/fpms-os-build/"
    Write-Host ''
    exit 3
}
Write-Ok "distro '$Distro' is running"

# --------------------------------------------------------------------------
# One round trip into the distro.
# --------------------------------------------------------------------------
$probe = @"
set -u
NOW=`$(date +%s)
echo "NOW=`$NOW"

# The repo path comes from the lock (a running build), then the last-run
# record (a finished one), then -RepoPath, then the usual places. Reading it
# from the lock is the only one that is guaranteed to be the tree actually
# being built.
REPO=''
for f in '$LockPath' '$LastPath'; do
    if [ -z "`$REPO" ] && [ -f "`$f" ]; then
        REPO=`$(awk -F= '/^repo=/{print `$2; exit}' "`$f" 2>/dev/null)
    fi
done
if [ -z "`$REPO" ]; then
    for c in '$RepoPath' /root/fpms-build/rover/fpms-os /root/fpms-build/fpms-os \
             /root/work/rover/fpms-os /root/rover/fpms-os /root/fpms-os; do
        [ -n "`$c" ] && [ -f "`$c/build.sh" ] && { REPO="`$c"; break; }
    done
fi
echo "REPO=`$REPO"

# --- is it running? -------------------------------------------------------
RUNNING=no
if [ -f '$LockPath' ]; then
    LP=`$(awk -F= '/^pid=/{print `$2; exit}' '$LockPath')
    BP=`$(awk -F= '/^build_pid=/{print `$2; exit}' '$LockPath')
    ST=`$(awk -F= '/^started_epoch=/{print `$2; exit}' '$LockPath')
    echo "LOCK=yes"
    echo "LOCK_PID=`${LP:-}"
    echo "BUILD_PID=`${BP:-}"
    echo "STARTED_EPOCH=`${ST:-}"
    echo "STARTED=`$(awk -F= '/^started=/{print `$2; exit}' '$LockPath')"
    echo "ARGS=`$(sed -n 's/^args=//p' '$LockPath' | head -1)"
    if [ -n "`${LP:-}" ] && kill -0 "`$LP" 2>/dev/null; then RUNNING=yes; fi
else
    echo "LOCK=no"
fi
echo "RUNNING=`$RUNNING"
echo "PGREP_BUILD=`$(pgrep -f '[b]uild\.sh' 2>/dev/null | tr '\n' ' ')"

if [ -n "`${BP:-}" ] && kill -0 "`${BP:-0}" 2>/dev/null; then
    echo "BUILD_STATE=`$(ps -o stat= -p "`$BP" | tr -d ' ')"
    echo "BUILD_ETIME=`$(ps -o etime= -p "`$BP" | tr -d ' ')"
fi

HB=`$(cat '$HeartPath' 2>/dev/null)
if [ -n "`${HB:-}" ]; then echo "HEARTBEAT_AGE=`$(( NOW - HB ))"; else echo "HEARTBEAT_AGE="; fi

# --- last finished run ----------------------------------------------------
if [ -f '$LastPath' ]; then
    echo "LAST_BEGIN"
    cat '$LastPath'
    echo "LAST_END"
fi

# --- liveness: is qemu actually burning a core? ---------------------------
echo "QEMU_COUNT=`$(pgrep -c -f 'qemu-aarch64' 2>/dev/null)"
echo "QEMU_TOP_BEGIN"
ps -eo pcpu,time,comm --sort=-pcpu 2>/dev/null | grep -i -E 'qemu|cc1plus|colcon|dpkg|apt' | head -4
echo "QEMU_TOP_END"

if [ -n "`$REPO" ]; then
    UW="`$REPO/.build/mnt/home/ubuntu/uros_ws/build"
    if [ -d "`$UW" ]; then
        echo "UROS_MTIME_AGE=`$(( NOW - `$(stat -c %Y "`$UW") ))"
    fi

    echo "REPO_FREE_MB=`$(df -Pm "`$REPO" 2>/dev/null | awk 'NR==2 {print `$4}')"

    # --- log ---------------------------------------------------------------
    LOG=`$(ls -1t "`$REPO"/.build/build-*.log 2>/dev/null | head -1)
    WRAP=`$(ls -1t "`$REPO"/.build/detached-*.log 2>/dev/null | head -1)
    echo "LOG=`${LOG:-}"
    echo "WRAP_LOG=`${WRAP:-}"
    SHOW="`${LOG:-`${WRAP:-}}"
    if [ -n "`$SHOW" ] && [ -f "`$SHOW" ]; then
        echo "LOG_AGE=`$(( NOW - `$(stat -c %Y "`$SHOW") ))"
        echo "LOG_SIZE=`$(stat -c %s "`$SHOW")"
        # Strip the colour escapes build.sh writes, or every banner arrives
        # wrapped in ESC[1;36m and the stage names do not match.
        CLEAN=`$(sed 's/\x1b\[[0-9;]*[a-zA-Z]//g' "`$SHOW")

        echo "STAGES_BEGIN"
        printf '%s\n' "`$CLEAN" | grep -oE 'stage [0-9]+-[A-Za-z0-9._-]+ OK in [0-9]+m[0-9]+s' || true
        echo "STAGES_END"

        echo "CURRENT_STAGE=`$(printf '%s\n' "`$CLEAN" | awk '
            /==> stage: /   { s=`$0; sub(/.*stage: /,"",s); sub(/ .*/,"",s); cur=s }
            / OK in [0-9]+m/{ cur="" }
            /FATAL/         { cur="" }
            END             { print cur }')"
        echo "FATAL=`$(printf '%s\n' "`$CLEAN" | grep -c '^FATAL:' || true)"

        echo "TAIL_BEGIN"
        printf '%s\n' "`$CLEAN" | tail -$Tail
        echo "TAIL_END"
    fi

    # --- output ------------------------------------------------------------
    echo "IMAGES_BEGIN"
    ls -lh "`$REPO"/fpms-os-*.img "`$REPO"/fpms-os-*.img.xz "`$REPO"/fpms-os-*.sha256 2>/dev/null \
        | awk '{print `$5, `$9}'
    echo "IMAGES_END"

    # --- leftovers ---------------------------------------------------------
    echo "MOUNTS=`$(mount | grep -c '/\.build/mnt' || true)"
    echo "LOOPS_BEGIN"
    losetup -a 2>/dev/null | grep 'fpms-os-.*\.img' || true
    echo "LOOPS_END"
fi
"@

$r = Invoke-WslBash -Script $probe -TimeoutSec 60
if ($r.TimedOut) {
    Write-Head 'build'
    Write-Bad 'the probe did not answer within 60 seconds.'
    Write-Note 'Reading /proc and tailing a file cannot take that long. Either the VM is'
    Write-Note 'wedged or the disk backing it is full. Check free space above first; do NOT'
    Write-Note 'reach for `wsl --shutdown` while a build is running - that kills it.'
    exit 4
}
$o = $r.Out

$repo = Get-Kv $o 'REPO'
$now  = 0; [int]::TryParse((Get-Kv $o 'NOW'), [ref]$now) | Out-Null

# --------------------------------------------------------------------------
Write-Head 'build'
$isRunning = ((Get-Kv $o 'RUNNING') -eq 'yes')
$pgrep     = Get-Kv $o 'PGREP_BUILD'

if ($isRunning) {
    $started = Get-Kv $o 'STARTED'
    $se = 0; [int]::TryParse((Get-Kv $o 'STARTED_EPOCH'), [ref]$se) | Out-Null
    $elapsed = 0
    if ($se -gt 0 -and $now -gt 0) { $elapsed = $now - $se }
    Write-Ok "RUNNING   runner pid $(Get-Kv $o 'LOCK_PID'), build.sh pid $(Get-Kv $o 'BUILD_PID')"
    Write-Note "started   $started   (elapsed $(Format-Duration $elapsed))"
    $bargs = Get-Kv $o 'ARGS'
    if ($bargs) { Write-Note "args      build.sh $bargs" } else { Write-Note 'args      build.sh (full build)' }
    $st = Get-Kv $o 'BUILD_STATE'
    if ($st) { Write-Note "state     $st   (D = uninterruptible I/O, R = running, S = sleeping)" }
} elseif ($pgrep -and $pgrep.Trim()) {
    Write-Warn2 "build.sh is running (pid(s) $($pgrep.Trim())) but there is no live lock."
    Write-Warn2 'Somebody started it by hand rather than through run-build-detached.ps1, so it'
    Write-Warn2 'is probably attached to a terminal and will die when that terminal does.'
} else {
    Write-Bad 'NOT RUNNING'
    $last = @(Get-Block $o 'LAST')
    if ($last.Count) {
        Write-Note 'last finished run:'
        foreach ($l in $last) { if ($l.Trim()) { Write-Note "    $l" } }
    } elseif ((Get-Kv $o 'LOCK') -eq 'yes') {
        Write-Warn2 "a lock file is present for pid $(Get-Kv $o 'LOCK_PID') but that process is gone,"
        Write-Warn2 'and no completion record was written. The build did not exit on its own -'
        Write-Warn2 'the distro was stopped underneath it (wsl --shutdown, sign-out, sleep, reboot)'
        Write-Warn2 'or something SIGKILLed it. Check the tail of the log below: if it stops'
        Write-Warn2 'mid-stage with no FATAL, that is what happened.'
    }
}

$hb = Get-Kv $o 'HEARTBEAT_AGE'
if ($hb) {
    $hbi = 0; [int]::TryParse($hb, [ref]$hbi) | Out-Null
    if ($isRunning -and $hbi -gt 180) {
        Write-Bad "heartbeat is $(Format-Duration $hbi) stale while the lock says running - the runner is wedged or the clock moved."
    } elseif ($isRunning) {
        Write-Note "heartbeat $(Format-Duration $hbi) ago"
    }
}

# --------------------------------------------------------------------------
Write-Head 'stages'
$stages = @(Get-Block $o 'STAGES' | Where-Object { $_.Trim() })
if ($stages.Count) {
    $total = 0
    foreach ($s in $stages) {
        $m = [regex]::Match($s, '^stage ([A-Za-z0-9._-]+) OK in ([0-9]+)m([0-9]+)s$')
        if ($m.Success) {
            $secs = ([int]$m.Groups[2].Value * 60) + [int]$m.Groups[3].Value
            $total += $secs
            Write-Ok ("  done  {0,-22} {1}" -f $m.Groups[1].Value, (Format-Duration $secs))
        } else {
            Write-Note "  $s"
        }
    }
    Write-Note ("        {0,-22} {1}" -f 'total in stages', (Format-Duration $total))
} else {
    Write-Note '  no stage has completed yet'
}

$cur = Get-Kv $o 'CURRENT_STAGE'
if ($cur) {
    Write-Warn2 "  now   $cur"
    if ($cur -like '10-*') {
        Write-Note '        stage 10 builds micro-ROS from source under emulation and prints'
        Write-Note '        nothing while it does. It was measured at 14h20m. It is not hung.'
    }
}
$fatal = Get-Kv $o 'FATAL'
if ($fatal -and $fatal -ne '0') {
    Write-Bad '  the log contains a FATAL - the build stopped on an error. See the tail below.'
    Write-Note '  Resume without losing the finished stages:  .\tools\run-build-detached.ps1 -From <stage>'
}

# --------------------------------------------------------------------------
Write-Head 'liveness'
$qc = Get-Kv $o 'QEMU_COUNT'
if ($qc) { Write-Note "qemu-aarch64 processes: $qc" }
$top = @(Get-Block $o 'QEMU_TOP')
foreach ($l in $top) { if ($l.Trim()) { Write-Note "  $l" } }
$uw = Get-Kv $o 'UROS_MTIME_AGE'
if ($uw) {
    $uwi = 0; [int]::TryParse($uw, [ref]$uwi) | Out-Null
    Write-Note "uros_ws/build last written $(Format-Duration $uwi) ago"
}
$la = Get-Kv $o 'LOG_AGE'
if ($la) {
    $lai = 0; [int]::TryParse($la, [ref]$lai) | Out-Null
    Write-Note "log last written $(Format-Duration $lai) ago ($(Get-Kv $o 'LOG_SIZE') bytes)"
    if ($isRunning -and $lai -gt 3600) {
        Write-Note 'A quiet log is normal inside stage 10. Judge by qemu CPU and uros_ws above.'
    }
}

# --------------------------------------------------------------------------
Write-Head 'disk'
$rf = Get-Kv $o 'REPO_FREE_MB'
if ($rf) {
    $rfi = 0; [int]::TryParse($rf, [ref]$rfi) | Out-Null
    $line = "inside the distro: $([math]::Round($rfi/1024,1)) GB free on $repo"
    if ($rfi -lt 6144)       { Write-Bad  ($line + '  <- the base-image step alone needs more than this') }
    elseif ($rfi -lt 17408)  { Write-Warn2 ($line + '  <- stage 10 or stage 90 will run out') }
    else                     { Write-Note $line }
}
Write-Note "on Windows:        $($vhdx.FreeGB) GB free on $($vhdx.Volume)"
Write-Note 'Both numbers matter: the ext4 can have room while C: cannot grow the VHDX to match.'

# --------------------------------------------------------------------------
Write-Head 'output'
$imgs = @(Get-Block $o 'IMAGES' | Where-Object { $_.Trim() })
if ($imgs.Count) {
    foreach ($i in $imgs) { Write-Note $i }
    if (($imgs -join ' ') -match '\.img\.xz') { Write-Ok 'a compressed image exists - this build produced its deliverable' }
} else {
    Write-Note 'no .img or .img.xz yet'
}

$mounts = Get-Kv $o 'MOUNTS'
$loops  = @(Get-Block $o 'LOOPS' | Where-Object { $_.Trim() })
if (($mounts -and $mounts -ne '0') -or $loops.Count) {
    if ($isRunning) {
        Write-Note "attached: $mounts mount(s) under .build/mnt, $($loops.Count) loop device(s) - expected while building"
    } else {
        Write-Head 'leftovers from a build that is no longer running'
        Write-Warn2 "$mounts mount(s) under .build/mnt, $($loops.Count) loop device(s) still attached:"
        foreach ($l in $loops) { Write-Warn2 "  $l" }
        Write-Warn2 'Release these BEFORE deleting anything. .build/mnt/dev is a bind mount of the'
        Write-Warn2 "host's /dev, and an rm -rf that crosses it does not stop at the image."
        Write-Warn2 'See "Cleaning up after a dead build" in tools\README.md.'
    }
}

# --------------------------------------------------------------------------
$tailLines = @(Get-Block $o 'TAIL')
if ($tailLines.Count) {
    Write-Head "log, last $Tail lines  ($(Get-Kv $o 'LOG'))"
    foreach ($l in $tailLines) { Write-Host "    $l" -ForegroundColor DarkGray }
}
Write-Host ''
if ($isRunning) { exit 0 } else { exit 1 }

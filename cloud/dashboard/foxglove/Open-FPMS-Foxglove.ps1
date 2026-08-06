<#
=============================================================================
 FPMS — open the Foxglove operator console, pointed at the rover.

   Double-click Open-FPMS-Foxglove.cmd, or:
       powershell -ExecutionPolicy Bypass -File Open-FPMS-Foxglove.ps1
       powershell -ExecutionPolicy Bypass -File Open-FPMS-Foxglove.ps1 192.168.137.42

 WHY THIS SCRIPT EXISTS AT ALL
 ----------------------------
 The rover is addressed as `fpms-pi.local`, never as an IP, because its DHCP
 address has moved more than seven times. But mDNS is not reliably available to
 every resolver on this laptop: within one session `fpms-pi.local` resolved
 correctly from .NET and FAILED from Python's getaddrinfo. So this script does
 not ask "does the name resolve?" — it asks the only question that matters,
 "can I open a TCP connection to port 8765?", and it asks it with .NET, which
 is the resolver Foxglove Studio (an Electron/Chromium app) also uses.

 DIRECTION OF THE CONNECTION — do not change this
 ------------------------------------------------
 foxglove_bridge LISTENS on the Pi. This laptop dials OUT. Windows Firewall on
 this machine has no inbound rule for anything and there is no admin account to
 add one, so any design where the rover connects INTO the laptop is dead — that
 is precisely what broke the MQTT bridge. Outbound TCP is allowed by default
 and needs no privilege.

 DESKTOP APP, NOT THE WEB APP
 ----------------------------
 app.foxglove.dev is served over HTTPS. A page in a secure context may not open
 an insecure `ws://` socket — the browser blocks it as mixed content, and the
 localhost carve-out does not apply because `fpms-pi.local` is not localhost.
 The web app therefore CANNOT reach this rover. Use the desktop app. (Making
 the web app work would mean running the bridge with TLS and a certificate the
 browser trusts for `fpms-pi.local`, which is not something to attempt the week
 of a competition.)
=============================================================================
#>

param(
    [string]$HostOverride = "",
    [int]$Port = 8765
)

$ErrorActionPreference = "Continue"

$PRIMARY_HOST = "fpms-pi.local"
$LAYOUT = Join-Path $PSScriptRoot "layouts\FPMS-Operator.json"
$MARKER_DIR = Join-Path $env:LOCALAPPDATA "FPMS"
$MARKER = Join-Path $MARKER_DIR "foxglove-layout-imported.txt"

function Say($m)  { Write-Host $m }
function Ok($m)   { Write-Host "[ok]   $m"   -ForegroundColor Green }
function Warn($m) { Write-Host "[warn] $m"   -ForegroundColor Yellow }
function Bad($m)  { Write-Host "[FAIL] $m"   -ForegroundColor Red }

# ---------------------------------------------------------------------------
# The only reachability test worth trusting: an actual TCP connect, with .NET's
# resolver, on the actual port. A ping proves nothing (the bridge could be
# down) and a DNS lookup proves nothing (Python's resolver already disagreed
# with .NET's once, in the same session).
# ---------------------------------------------------------------------------
function Test-Port {
    param([string]$TargetHost, [int]$TargetPort, [int]$TimeoutMs = 2500)
    $client = New-Object System.Net.Sockets.TcpClient
    try {
        $ar = $client.BeginConnect($TargetHost, $TargetPort, $null, $null)
        if (-not $ar.AsyncWaitHandle.WaitOne($TimeoutMs, $false)) {
            $client.Close(); return $false
        }
        $client.EndConnect($ar)
        $client.Close()
        return $true
    } catch {
        try { $client.Close() } catch { }
        return $false
    }
}

Say ""
Say "=============================================================="
Say "  FPMS  ->  Foxglove operator console"
Say "=============================================================="
Say ""

# ------------------------------------------------------------ 1. find the Pi
$target = $null

if ($HostOverride -ne "") {
    Say "Trying the address you gave me: $HostOverride ..."
    if (Test-Port $HostOverride $Port) {
        $target = $HostOverride
        Ok "$HostOverride`:$Port answered"
    } else {
        Bad "$HostOverride`:$Port did not answer"
    }
}

if (-not $target) {
    Say "Trying $PRIMARY_HOST`:$Port (mDNS) ..."
    if (Test-Port $PRIMARY_HOST $Port) {
        $target = $PRIMARY_HOST
        Ok "$PRIMARY_HOST`:$Port answered - using the NAME, so the address can move again"
    } else {
        Warn "$PRIMARY_HOST`:$Port did not answer."
        Say  "       Either the rover is off, foxglove_bridge is not running, or"
        Say  "       mDNS is not resolving on this adapter. Falling back to a scan."
    }
}

# --------------------------------------------------- 2. fallback: address scan
if (-not $target) {
    Say ""
    Say "Scanning addresses this laptop has recently talked to ..."
    $candidates = @()
    try {
        # State matters. HANDOFF.md records a `Permanent` ARP entry for an
        # address that was NOT actually present, and it was read as "the Pi is
        # there". Permanent entries are static configuration, not evidence.
        $candidates += (Get-NetNeighbor -AddressFamily IPv4 -ErrorAction SilentlyContinue |
                        Where-Object { $_.State -in @('Reachable','Stale','Delay','Probe') } |
                        Select-Object -ExpandProperty IPAddress)
    } catch { }
    try {
        # The Mobile Hotspot subnet is where the rover lives (192.168.137.x).
        # .1 is this laptop; do not probe it.
        $candidates += (arp -a | Select-String -Pattern '(\d+\.\d+\.\d+\.\d+)' -AllMatches |
                        ForEach-Object { $_.Matches } | ForEach-Object { $_.Value })
    } catch { }

    $candidates = $candidates |
        Where-Object { $_ -match '^\d+\.\d+\.\d+\.\d+$' } |
        Where-Object { -not ($_ -match '^(127\.|224\.|239\.|255\.)') } |
        Where-Object { -not ($_ -like '*.255') } |
        Where-Object { $_ -ne '192.168.137.1' } |
        Select-Object -Unique

    Say ("  $($candidates.Count) candidate addresses")
    foreach ($ip in $candidates) {
        if (Test-Port $ip $Port 400) {
            $target = $ip
            Ok "found the bridge at $ip`:$Port"
            Warn "This is an IP, not the name. It WILL change on the next DHCP lease."
            Warn "Fix mDNS when you have five minutes; do not write this address down."
            break
        }
    }
}

# ------------------------------------------------------ 3. still nothing found
if (-not $target) {
    Say ""
    Bad "Could not reach foxglove_bridge on port $Port."
    Say ""
    Say "WORK THROUGH THIS IN ORDER:"
    Say ""
    Say "  1. Is the rover powered on, and is its light on?"
    Say ""
    Say "  2. Is it on the laptop's Mobile Hotspot?  Settings -> Mobile hotspot."
    Say "     If 'Devices connected' is 0, the Pi has not joined. Nothing on this"
    Say "     laptop can fix that - somebody has to go to the rover."
    Say ""
    Say "  3. From a terminal:      ping fpms-pi.local"
    Say "     If the name fails but you know the address, run me with it:"
    Say "         Open-FPMS-Foxglove.cmd 192.168.137.42"
    Say ""
    Say "  4. SSH in and check the bridge:"
    Say "         ssh ubuntu@fpms-pi.local"
    Say "         systemctl status fpms-foxglove-bridge"
    Say "         sudo journalctl -u fpms-foxglove-bridge -n 40"
    Say ""
    Say "  5. If mDNS is the problem and only the name is broken, add a line to"
    Say "     C:\Windows\System32\drivers\etc\hosts  ->  needs admin, which you"
    Say "     do not have on this laptop. Use option 3 instead."
    Say ""
    exit 1
}

# ------------------------------------------------------------- 4. the layout
$wsUrl = "ws://$target`:$Port"
Say ""
Ok "connecting to  $wsUrl"

if (-not (Test-Path $MARKER)) {
    Say ""
    Say "--------------------------------------------------------------"
    Say " ONE-TIME STEP: load the saved layout."
    Say ""
    Say " Foxglove's deep link can choose the CONNECTION but not a layout"
    Say " file, so the layout is imported once and then remembered."
    Say ""
    Say "   In Foxglove:  Layout menu (top left)  ->  Import from file..."
    Say "   Choose:       $LAYOUT"
    Say "--------------------------------------------------------------"
    Say ""
    if (Test-Path $LAYOUT) {
        try { Start-Process explorer.exe "/select,`"$LAYOUT`"" } catch { }
    } else {
        Warn "layout file not found at $LAYOUT"
    }
    try {
        if (-not (Test-Path $MARKER_DIR)) {
            New-Item -ItemType Directory -Path $MARKER_DIR -Force | Out-Null
        }
        "Layout import was offered on $(Get-Date -Format s). Delete this file to be reminded again." |
            Out-File -FilePath $MARKER -Encoding utf8
    } catch { }
}

# ------------------------------------------------------------ 5. launch Studio
# The `foxglove://` protocol handler is registered by the desktop app's
# installer. Using it rather than a hard-coded .exe path means this keeps
# working across Foxglove versions and across the rename from
# "Foxglove Studio" to "Foxglove".
$deepLink = "foxglove://open?ds=foxglove-websocket&ds.url=$wsUrl"

$handlerOk = $false
try {
    $handlerOk = (Test-Path "Registry::HKEY_CLASSES_ROOT\foxglove") -or
                 (Test-Path "HKCU:\Software\Classes\foxglove")
} catch { }

if (-not $handlerOk) {
    Say ""
    Bad "Foxglove Studio (desktop) does not appear to be installed."
    Say ""
    Say "  Install the DESKTOP app from https://foxglove.dev/download"
    Say "  The WEB app at app.foxglove.dev CANNOT connect to this rover:"
    Say "  it is served over HTTPS and a secure page is not allowed to open"
    Say "  an insecure ws:// socket to a LAN address."
    Say ""
    Say "  Once installed, open it and use:  Open connection ->"
    Say "  Foxglove WebSocket ->  $wsUrl"
    Say ""
    exit 1
}

Say ""
Say "Opening Foxglove ..."
try {
    Start-Process $deepLink
    Ok "launched"
} catch {
    Bad "could not open the foxglove:// link ($($_.Exception.Message))"
    Say ""
    Say "Open Foxglove yourself and use:  Open connection -> Foxglove WebSocket"
    Say "URL:  $wsUrl"
    exit 1
}

Say ""
Say "If Foxglove says 'no data', check the topic list in the left sidebar:"
Say "  * an EMPTY list means the bridge is up but sees no ROS topics. That is"
Say "    almost always ROS_DOMAIN_ID: the drive board publishes ONLY on"
Say "    domain 20, and the unit must set it."
Say "  * a FULL list but frozen panels means the source stopped, not the link."
Say "    Look at the HEALTH tab - the Pi reports each source's own age there."
Say ""

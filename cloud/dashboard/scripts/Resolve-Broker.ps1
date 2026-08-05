<#
.SYNOPSIS
  Work out which host the dashboard should point FPMS_MQTT_HOST at, and print it.

.DESCRIPTION
  THE PROBLEM THIS EXISTS FOR.

  Start-FPMS-Dashboard.cmd used to carry a hard-coded Pi address. The Pi moves
  constantly - it has been .213, then .94, and it will be something else by the
  time anyone reads this - so the launcher spent most of its life pointing at a
  machine that was not there. The symptom is the worst one this dashboard has:
  every panel sits at "waiting", no error appears anywhere, and it looks exactly
  like a broken app rather than a wrong address.

  THE PREFERRED ANSWER IS THE mDNS NAME, fpms-pi.local. It is the name the
  operator uses and it follows the Pi across DHCP leases. But it cannot be the
  ONLY answer, and this was measured on the operator's laptop rather than
  assumed: `ping fpms-pi.local` succeeds and resolves to an IPv6 LINK-LOCAL
  address (fe80::...), while getaddrinfo - which is what Python's paho and .NET
  both use - fails outright with "no such host". A link-local address is not
  usable without a scope id, and paho does not attach one. So handing the name
  straight to the backend would have swapped a stale address for one that never
  connects at all.

  Hence: TRY, THEN PROVE. Every candidate is TCP-probed on the broker port and
  only a candidate that actually accepts a connection is printed. The order is
  the order of how much a human would trust the answer:

    1. FPMS_MQTT_HOST_FORCE       an explicit override, used as-is, no probe.
                                  The escape hatch for a case this never saw.
    2. the mDNS name(s)           preferred, because it survives DHCP.
    3. the last host that worked  remembered across runs.
    4. the Pi's MAC in the ARP    this is what ACTUALLY survives a DHCP change
       / neighbour table          on this network, and it is why the MAC of
                                  every successful host is remembered below.
    5. 127.0.0.1                  a broker on this laptop, the old default.

  It prints ONE line - the chosen host - and nothing else on stdout, because
  the launcher reads stdout. Everything else goes to stderr so it still lands
  in launch.log when something needs explaining.

  It never fails. If nothing answers it prints the first preferred name anyway
  and says so loudly on stderr: a launcher that refused to start because the
  Pi was off would be a worse outcome than a dashboard that comes up and
  reports its own disconnection, which is a thing this app already does well.
#>
[CmdletBinding()]
param(
    # Preferred names, most-preferred first. The mDNS name leads deliberately.
    [string[]]$Names = @('fpms-pi.local'),
    [int]$Port = 1883,
    # Per-candidate probe budget. Generous enough for a sleepy WiFi link,
    # small enough that five dead candidates cost under four seconds total.
    [int]$TimeoutMs = 700
)

$ErrorActionPreference = 'Continue'
$stateDir  = Join-Path $env:LOCALAPPDATA 'FPMS'
$hostFile  = Join-Path $stateDir 'broker-host.txt'
$macFile   = Join-Path $stateDir 'broker-mac.txt'

function Note([string]$m) { [Console]::Error.WriteLine("[broker] $m") }

# ---------------------------------------------------------------------------
# 1. Explicit override wins and is NOT probed.
#
# Not probing is the point: this is the switch someone flips when they know
# something this script does not - a tunnel, a port-forward, a broker that is
# about to come up. A probe here would second-guess a human who is looking
# straight at the machine.
# ---------------------------------------------------------------------------
if ($env:FPMS_MQTT_HOST_FORCE) {
    Note "FPMS_MQTT_HOST_FORCE is set - using $($env:FPMS_MQTT_HOST_FORCE) without probing."
    Write-Output $env:FPMS_MQTT_HOST_FORCE
    exit 0
}

# ---------------------------------------------------------------------------
# Does anything answer on the broker port at this address?
#
# A DNS lookup is NOT enough and never has been. The whole reason this file
# exists is a name that resolves to something unreachable, so "it resolved" is
# not evidence and only a completed TCP handshake counts.
# ---------------------------------------------------------------------------
function Test-Broker([string]$candidate) {
    if ([string]::IsNullOrWhiteSpace($candidate)) { return $false }

    # ----------------------------------------------------------------------
    # PROBE THE WAY THE CONSUMER RESOLVES, NOT THE WAY POWERSHELL CAN.
    #
    # This check is not belt-and-braces, it is the actual bug. Measured on the
    # operator's laptop: TcpClient.BeginConnect('fpms-pi.local', 1883) CONNECTS
    # - .NET's async path reaches the Pi over the mDNS IPv6 link-local address,
    # scope id and all. Python's socket.getaddrinfo('fpms-pi.local') fails with
    # "no such host", and Python is what the backend uses.
    #
    # So a probe that only asked "can PowerShell reach this?" would cheerfully
    # certify a name the dashboard cannot use, and the launcher would write it
    # into FPMS_MQTT_HOST - producing the identical every-panel-waiting symptom
    # this whole file exists to end, with a green tick next to it.
    #
    # A name therefore has to yield an IPv4 address before it is allowed to
    # win. Literal IPv4 addresses skip this and go straight to the handshake.
    # ----------------------------------------------------------------------
    if ($candidate -notmatch '^\d{1,3}(\.\d{1,3}){3}$') {
        $v4 = $null
        try {
            $v4 = [Net.Dns]::GetHostAddresses($candidate) |
                  Where-Object { $_.AddressFamily -eq 'InterNetwork' } |
                  Select-Object -First 1
        } catch { $v4 = $null }
        if (-not $v4) {
            Note "$candidate does not resolve to an IPv4 address - the backend's resolver cannot use it, skipping."
            return $false
        }
    }

    $client = New-Object Net.Sockets.TcpClient
    try {
        $iar = $client.BeginConnect($candidate, $Port, $null, $null)
        if (-not $iar.AsyncWaitHandle.WaitOne($TimeoutMs, $false)) { return $false }
        $client.EndConnect($iar)
        if (-not $client.Connected) { return $false }
        return (Test-RoverTraffic $client)
    } catch {
        return $false
    } finally {
        try { $client.Close() } catch {}
    }
}

# ---------------------------------------------------------------------------
# "SOMETHING IS LISTENING ON 1883" IS NOT THE QUESTION.
#
# Measured, again on the operator's own laptop: 127.0.0.1:1883 accepts a
# connection. There IS a mosquitto here - it is the one the Pi's bridge is
# supposed to feed, and that bridge does not work (no inbound firewall rule for
# 1883, which is exactly why the launcher connects OUTBOUND to the Pi instead).
# So the nearest, fastest, most confident answer to a plain TCP probe is a
# broker with NO ROVER ON IT, and a plain TCP probe would pick it every single
# time.
#
# The real question is "does this broker carry rover telemetry". So: speak
# MQTT 3.1.1 at it, subscribe to fpms/+/telemetry/#, and wait for a publish.
# Roughly a second is plenty - the rover pushes LiDAR and camera frames several
# times a second - and silence is a NO. That is the difference between the
# dashboard coming up live and coming up connected-to-nothing, which are states
# that look identical in the UI and completely different to the operator.
#
# Hand-rolled because there is no MQTT client in Windows PowerShell 5.1 and
# adding a dependency to a launcher is how launchers stop working.
# ---------------------------------------------------------------------------
function Write-VarInt([System.Collections.Generic.List[byte]]$buf, [int]$len) {
    do {
        $b = $len % 128
        $len = [math]::Floor($len / 128)
        if ($len -gt 0) { $b = $b -bor 128 }
        $buf.Add([byte]$b)
    } while ($len -gt 0)
}

function Add-MqttString([System.Collections.Generic.List[byte]]$buf, [string]$s) {
    $b = [Text.Encoding]::UTF8.GetBytes($s)
    $buf.Add([byte](($b.Length -shr 8) -band 0xFF))
    $buf.Add([byte]($b.Length -band 0xFF))
    $buf.AddRange($b)
}

function Test-RoverTraffic([Net.Sockets.TcpClient]$client) {
    try {
        $user = $env:FPMS_MQTT_USERNAME
        $pass = $env:FPMS_MQTT_PASSWORD
        $stream = $client.GetStream()
        $stream.ReadTimeout = 1500

        # ---- CONNECT ----
        $vh = New-Object System.Collections.Generic.List[byte]
        Add-MqttString $vh 'MQTT'
        $vh.Add([byte]4)                        # protocol level 3.1.1
        $flags = 2                              # clean session
        if ($user) { $flags = $flags -bor 128 }
        if ($pass) { $flags = $flags -bor 64 }
        $vh.Add([byte]$flags)
        $vh.Add([byte]0); $vh.Add([byte]60)     # keepalive 60s
        Add-MqttString $vh ("fpms-probe-" + (Get-Random -Maximum 99999))
        if ($user) { Add-MqttString $vh $user }
        if ($pass) { Add-MqttString $vh $pass }

        $pkt = New-Object System.Collections.Generic.List[byte]
        $pkt.Add([byte]0x10)
        Write-VarInt $pkt $vh.Count
        $pkt.AddRange($vh)
        $bytes = $pkt.ToArray()
        $stream.Write($bytes, 0, $bytes.Length)
        $stream.Flush()

        # ---- CONNACK ----
        $hdr = New-Object byte[] 4
        if ($stream.Read($hdr, 0, 4) -lt 4) { return $false }
        # 0x20 = CONNACK; byte 3 is the return code, 0 = accepted. A refusal
        # here (bad credentials) is reported rather than swallowed: it is a
        # completely different fix from "the Pi is off".
        if ($hdr[0] -ne 0x20) { return $false }
        if ($hdr[3] -ne 0) {
            Note "broker refused our credentials (CONNACK rc=$($hdr[3])) - it is reachable, but FPMS_MQTT_USERNAME/PASSWORD are wrong."
            return $false
        }

        # ---- SUBSCRIBE fpms/+/telemetry/# ----
        $vh2 = New-Object System.Collections.Generic.List[byte]
        $vh2.Add([byte]0); $vh2.Add([byte]1)    # packet id 1
        Add-MqttString $vh2 'fpms/+/telemetry/#'
        $vh2.Add([byte]0)                       # qos 0
        $pkt2 = New-Object System.Collections.Generic.List[byte]
        $pkt2.Add([byte]0x82)
        Write-VarInt $pkt2 $vh2.Count
        $pkt2.AddRange($vh2)
        $b2 = $pkt2.ToArray()
        $stream.Write($b2, 0, $b2.Length)
        $stream.Flush()

        # ---- wait for a PUBLISH (0x3x) ----
        # SUBACK arrives first and is skipped by type. Anything that is not a
        # publish within the window means this broker is quiet, and a quiet
        # broker is not the rover's broker.
        $deadline = (Get-Date).AddMilliseconds(1500)
        $one = New-Object byte[] 1
        while ((Get-Date) -lt $deadline) {
            try {
                if ($stream.Read($one, 0, 1) -lt 1) { return $false }
            } catch { return $false }
            $type = ($one[0] -shr 4) -band 0x0F
            # Remaining length, then skip the body.
            $len = 0; $mult = 1
            do {
                if ($stream.Read($one, 0, 1) -lt 1) { return $false }
                $len += ($one[0] -band 127) * $mult
                $mult *= 128
            } while (($one[0] -band 128) -ne 0)
            if ($type -eq 3) { return $true }   # PUBLISH - real rover traffic
            if ($len -gt 0) {
                $skip = New-Object byte[] $len
                $read = 0
                while ($read -lt $len) {
                    $n = $stream.Read($skip, $read, $len - $read)
                    if ($n -le 0) { return $false }
                    $read += $n
                }
            }
        }
        Note 'connected and authenticated, but no fpms telemetry arrived - this is not the rover''s broker.'
        return $false
    } catch {
        return $false
    }
}

# The IPv4 currently leased to a known MAC. This is the piece that genuinely
# survives a DHCP change: the address moves, the network card does not.
function Get-IpForMac([string]$mac) {
    if ([string]::IsNullOrWhiteSpace($mac)) { return $null }
    $want = $mac.ToUpper().Replace(':', '-')
    try {
        $n = Get-NetNeighbor -AddressFamily IPv4 -ErrorAction Stop |
             Where-Object { $_.LinkLayerAddress -and
                            $_.LinkLayerAddress.ToUpper() -eq $want -and
                            $_.State -ne 'Unreachable' } |
             Select-Object -First 1
        if ($n) { return $n.IPAddress }
    } catch {}
    return $null
}

function Get-MacForIp([string]$ip) {
    try {
        $n = Get-NetNeighbor -AddressFamily IPv4 -IPAddress $ip -ErrorAction Stop |
             Select-Object -First 1
        if ($n -and $n.LinkLayerAddress) { return $n.LinkLayerAddress }
    } catch {}
    return $null
}

# ---------------------------------------------------------------------------
# 2..5 - build the candidate list, in trust order, without duplicates.
# ---------------------------------------------------------------------------
$candidates = New-Object System.Collections.Generic.List[string]
function Add-Candidate([string]$c, [string]$why) {
    if ([string]::IsNullOrWhiteSpace($c)) { return }
    if ($candidates -contains $c) { return }
    $candidates.Add($c) | Out-Null
    Note "candidate: $c  ($why)"
}

foreach ($n in $Names) { Add-Candidate $n 'preferred mDNS name' }

$lastHost = $null
if (Test-Path -LiteralPath $hostFile) {
    $lastHost = (Get-Content -LiteralPath $hostFile -TotalCount 1 -ErrorAction SilentlyContinue)
    if ($lastHost) { $lastHost = $lastHost.Trim() }
    Add-Candidate $lastHost 'last host that worked'
}

if (Test-Path -LiteralPath $macFile) {
    $lastMac = (Get-Content -LiteralPath $macFile -TotalCount 1 -ErrorAction SilentlyContinue)
    if ($lastMac) {
        $byMac = Get-IpForMac $lastMac.Trim()
        Add-Candidate $byMac "current lease for known MAC $($lastMac.Trim())"
    }
}

# ---------------------------------------------------------------------------
# 4b. Every IPv4 neighbour this laptop has
#     recently talked to.
#
# This is what covers the genuinely new case - a fresh laptop, a re-imaged Pi,
# a MAC nobody has recorded yet - and it is why the operator never has to type
# an address. It is bounded by the ARP/neighbour table, which on the hotspot
# network this rover runs on is a handful of entries, so the worst case is a
# couple of seconds of probing and not a subnet sweep.
#
# Deliberately LAST. It picks whatever answers on 1883 first, and "some machine
# on this network runs a broker" is a much weaker claim than any candidate
# above it.
# ---------------------------------------------------------------------------
try {
    $neighbours = Get-NetNeighbor -AddressFamily IPv4 -ErrorAction Stop |
        Where-Object {
            $_.State -notin @('Unreachable', 'Incomplete') -and
            $_.LinkLayerAddress -and
            $_.LinkLayerAddress -ne '00-00-00-00-00-00' -and
            $_.LinkLayerAddress -ne 'FF-FF-FF-FF-FF-FF' -and
            $_.IPAddress -notmatch '\.255$' -and
            $_.IPAddress -notmatch '^(127\.|169\.254\.|22[4-9]\.|23[0-9]\.)'
        } |
        Select-Object -ExpandProperty IPAddress -Unique
    foreach ($ip in $neighbours) { Add-Candidate $ip 'host seen on this network' }
} catch {
    Note "could not read the neighbour table: $($_.Exception.Message)"
}

# ---------------------------------------------------------------------------
# 5. The laptop's own broker, DEAD LAST and for a specific reason.
#
# It is the fastest thing to answer and the most tempting to pick, and it is
# almost always the wrong one. There is a mosquitto on this machine; the Pi is
# supposed to bridge into it and cannot (no inbound rule for 1883), so it sits
# there empty. Ranked above the network sweep it would win every race and the
# dashboard would come up connected to a broker with no rover on it - which is
# why the telemetry check above exists as well as this ordering. Belt and
# braces, because this exact confusion has cost hours.
# ---------------------------------------------------------------------------
Add-Candidate '127.0.0.1' 'a broker on this laptop (usually empty)'

# ---------------------------------------------------------------------------
# Probe, in order. First one that answers wins.
# ---------------------------------------------------------------------------
$chosen = $null
foreach ($c in $candidates) {
    if (Test-Broker $c) { $chosen = $c; Note "OK: $c answers on $Port - using it."; break }
    Note "no answer from $c on $Port"
}

if (-not $chosen) {
    $fallback = if ($Names.Count -gt 0) { $Names[0] } else { '127.0.0.1' }
    Note "NOTHING answered on port $Port. Falling back to $fallback."
    Note 'The dashboard will start and report itself disconnected, which is the'
    Note 'honest outcome - it is not evidence that the app is broken. Check the'
    Note 'Pi is powered, on the same network, and running mosquitto.'
    Write-Output $fallback
    exit 0
}

# ---------------------------------------------------------------------------
# Remember what worked, and remember the MAC behind it.
#
# The MAC is the durable half. Next time the Pi takes a different lease, step 4
# above finds it again without anyone editing a file.
# ---------------------------------------------------------------------------
try {
    if (-not (Test-Path -LiteralPath $stateDir)) {
        New-Item -ItemType Directory -Path $stateDir -Force | Out-Null
    }
    Set-Content -LiteralPath $hostFile -Value $chosen -Encoding ascii
    # Resolve the winner to an IPv4 so the neighbour lookup has something to
    # match. A name that only yields IPv6 simply leaves the MAC unrecorded.
    $ip = $chosen
    if ($chosen -notmatch '^\d{1,3}(\.\d{1,3}){3}$') {
        try {
            $ip = ([Net.Dns]::GetHostAddresses($chosen) |
                   Where-Object { $_.AddressFamily -eq 'InterNetwork' } |
                   Select-Object -First 1).IPAddressToString
        } catch { $ip = $null }
    }
    if ($ip) {
        $mac = Get-MacForIp $ip
        if ($mac) { Set-Content -LiteralPath $macFile -Value $mac -Encoding ascii }
    }
} catch {
    Note "could not persist the choice: $($_.Exception.Message)"
}

Write-Output $chosen
exit 0

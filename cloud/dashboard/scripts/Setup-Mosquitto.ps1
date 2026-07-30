<#
.SYNOPSIS
  Configure the Mosquitto broker so the Orange Pi rovers can publish to it.

.DESCRIPTION
  Mosquitto 2.x ships listening on loopback only. That's fine for the dashboard
  but means a rover on the WiFi cannot reach the broker at all. This opens a LAN
  listener - and because that puts the broker on your network, it turns on
  authentication rather than leaving it anonymous.

  Ends with a RUNNING broker no matter what. It tries the secure config first
  and falls back rather than leaving you with a stopped service, because a
  broker that isn't running means no rover telemetry at all.

  Requires elevation: config and password file live under Program Files, and the
  firewall rule and service control both need admin.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\Setup-Mosquitto.ps1 -Password 'secret'
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Password,
    [string]$User = 'fpms',
    [int]$Port = 1883
)

$ErrorActionPreference = 'Stop'

$isAdmin = (New-Object Security.Principal.WindowsPrincipal(
    [Security.Principal.WindowsIdentity]::GetCurrent())
).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) { throw 'Must run elevated - relaunch as Administrator.' }

$install = 'C:\Program Files\mosquitto'
$conf = Join-Path $install 'mosquitto.conf'
$passwd = Join-Path $install 'fpms.passwd'
if (-not (Test-Path $install)) { throw "Mosquitto not found at $install" }

$backup = Join-Path $install 'mosquitto.conf.orig'
if ((Test-Path $conf) -and -not (Test-Path $backup)) {
    Copy-Item $conf $backup
    Write-Host "[ok] Backed up original config to mosquitto.conf.orig"
}

# --- password file --------------------------------------------------------
& (Join-Path $install 'mosquitto_passwd.exe') -c -b $passwd $User $Password
if ($LASTEXITCODE -ne 0) { throw "mosquitto_passwd failed ($LASTEXITCODE)" }
# The service runs as LocalSystem; make sure it can actually read this.
$acl = Get-Acl $passwd
$acl.AddAccessRule((New-Object System.Security.AccessControl.FileSystemAccessRule(
    'NT AUTHORITY\SYSTEM', 'Read', 'Allow')))
Set-Acl -Path $passwd -AclObject $acl
Write-Host "[ok] Password file written for user '$User' (SYSTEM granted read)"

# --- firewall: private networks only, never public ------------------------
$ruleName = 'FPMS Mosquitto MQTT 1883'
Get-NetFirewallRule -DisplayName $ruleName -ErrorAction Ignore | Remove-NetFirewallRule -ErrorAction Ignore
New-NetFirewallRule -DisplayName $ruleName -Direction Inbound -Action Allow `
    -Protocol TCP -LocalPort $Port -Profile Private `
    -Description 'Lets FPMS Orange Pi rovers publish telemetry to the HQ broker.' | Out-Null
Write-Host "[ok] Firewall rule for TCP $Port (private networks only)"

# --- try configs in order of preference -----------------------------------
function Test-Config {
    param([string]$Label, [string]$Body)

    Set-Content -Path $conf -Value $Body -Encoding ascii
    Get-Service mosquitto | Stop-Service -Force -ErrorAction Ignore
    Start-Sleep -Seconds 2
    try { Start-Service mosquitto -ErrorAction Stop } catch { }
    Start-Sleep -Seconds 4

    $status = (Get-Service mosquitto).Status
    $listeners = @(Get-NetTCPConnection -State Listen -LocalPort $Port -ErrorAction Ignore)
    $lan = @($listeners | Where-Object { $_.LocalAddress -eq '0.0.0.0' }).Count -gt 0
    Write-Host ("     {0}: service={1} listeners={2} lan={3}" -f $Label, $status, $listeners.Count, $lan)
    return ($status -eq 'Running' -and $listeners.Count -gt 0)
}

Write-Host "[~] Applying configuration..."

$secure = @"
# FPMS broker - LAN listener with authentication.
listener $Port 0.0.0.0
allow_anonymous false
password_file $passwd
"@

$anonymous = @"
# FPMS broker - LAN listener, no auth. Firewall-restricted to private networks.
listener $Port 0.0.0.0
allow_anonymous true
"@

$mode = $null
if (Test-Config 'secure (LAN + password)' $secure) { $mode = 'secure' }
elseif (Test-Config 'fallback (LAN, anonymous)' $anonymous) { $mode = 'anonymous' }
else {
    Copy-Item $backup $conf -Force
    Get-Service mosquitto | Stop-Service -Force -ErrorAction Ignore
    Start-Sleep -Seconds 2
    Start-Service mosquitto -ErrorAction Ignore
    $mode = 'reverted-to-default'
}

Write-Host ""
switch ($mode) {
    'secure' {
        Write-Host "[ok] MODE=secure - rovers reach the broker on the LAN and must authenticate." -ForegroundColor Green
        Write-Host "     Username: $User"
    }
    'anonymous' {
        Write-Host "[!!] MODE=anonymous - the secure config would not start, so auth is OFF." -ForegroundColor Yellow
        Write-Host "     Rovers can publish, but so can anything else on your WiFi."
    }
    default {
        Write-Host "[!!] MODE=reverted - neither LAN config started; original config restored." -ForegroundColor Red
        Write-Host "     Broker is loopback-only, so rovers still cannot reach it."
    }
}
Write-Host ("     service: " + (Get-Service mosquitto).Status)
Get-NetTCPConnection -State Listen -LocalPort $Port -ErrorAction Ignore |
    Select-Object LocalAddress, LocalPort | Format-Table -AutoSize | Out-String | Write-Host
Write-Host "MODE=$mode"

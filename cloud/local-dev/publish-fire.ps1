# Publish a fake fire-detected event as if rover1 saw one. Uses the
# mosquitto_pub.exe bundled with the Mosquitto Windows installer.
#
# Usage:  .\publish-fire.ps1 [thing]  (default: rover1)

param(
    [string]$Thing = "rover1"
)

$ErrorActionPreference = "Stop"

$mp = "C:\Program Files\mosquitto\mosquitto_pub.exe"
if (-not (Test-Path $mp)) {
    Write-Error "mosquitto_pub not found at $mp — install Eclipse Mosquitto for Windows."
}

$ts      = (Get-Date).ToUniversalTime().ToString("yyyy-MM-ddTHH:mm:ssZ")
$eventId = "demo-" + (Get-Date).ToUniversalTime().ToString("yyyyMMddTHHmmssZ")
$topic   = "fpms/$Thing/events/fire-detected"

$payload = @{
    event_id  = $eventId
    timestamp = $ts
    thing     = $Thing
    severity  = "high"
    location  = @{ lat = 43.6532; lon = -79.3832; frame = "gps" }
    cameras   = @{ rgb_confidence = 0.94; thermal_max_c = 312.4 }
    photos    = @()
} | ConvertTo-Json -Compress

Write-Host "publishing to $topic (event_id=$eventId)"
& $mp -h localhost -p 1883 -i $Thing -t $topic -m $payload
Write-Host "done. check archive with: .\check-archive.ps1 $Thing fire-detected"

# Publish a fake heritage-documented event.

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
$topic   = "fpms/$Thing/events/heritage-documented"

$payload = @{
    event_id    = $eventId
    timestamp   = $ts
    thing       = $Thing
    severity    = "info"
    marker_id   = "hm-0042"
    description = "Rock formation with visible pigment marks"
    location    = @{ lat = 43.6533; lon = -79.3841; frame = "gps" }
    photos      = @("s3://fpms-heritage/markers/hm-0042/photos/$ts.jpg")
} | ConvertTo-Json -Compress

Write-Host "publishing to $topic (event_id=$eventId)"
& $mp -h localhost -p 1883 -i $Thing -t $topic -m $payload
Write-Host "done."

#!/usr/bin/env bash
# Simulate a fire-detected event from rover1. Uses mosquitto_pub inside the
# Mosquitto container so you don't need mosquitto-clients installed locally.

set -euo pipefail

thing="${1:-rover1}"
event_id="demo-$(date -u +%Y%m%dT%H%M%SZ)"
ts="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

payload=$(cat <<EOF
{
  "event_id": "$event_id",
  "timestamp": "$ts",
  "thing": "$thing",
  "severity": "high",
  "location": { "lat": 43.6532, "lon": -79.3832, "frame": "gps" },
  "cameras": { "rgb_confidence": 0.94, "thermal_max_c": 312.4 },
  "photos": []
}
EOF
)

topic="fpms/$thing/events/fire-detected"

echo "==> publishing to $topic"
docker exec -i fpms-mosquitto mosquitto_pub \
  -h localhost -p 1883 \
  -i "$thing" \
  -t "$topic" \
  -m "$payload"

echo "==> published event_id=$event_id"
echo "    check archive:  cloud/scripts/check-s3.sh"
echo "    watch alerts:   cloud/scripts/tail-lambda-logs.sh fpms-alert-dispatcher"

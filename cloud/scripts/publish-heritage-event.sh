#!/usr/bin/env bash
# Simulate a heritage-documented event from rover1.

set -euo pipefail

thing="${1:-rover1}"
event_id="demo-$(date -u +%Y%m%dT%H%M%SZ)"
ts="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

payload=$(cat <<EOF
{
  "event_id": "$event_id",
  "timestamp": "$ts",
  "thing": "$thing",
  "severity": "info",
  "marker_id": "hm-0042",
  "description": "Rock formation with visible pigment marks",
  "location": { "lat": 43.6533, "lon": -79.3841, "frame": "gps" },
  "photos": [ "s3://fpms-heritage/markers/hm-0042/photos/$ts.jpg" ]
}
EOF
)

topic="fpms/$thing/events/heritage-documented"

echo "==> publishing to $topic"
docker exec -i fpms-mosquitto mosquitto_pub \
  -h localhost -p 1883 \
  -i "$thing" \
  -t "$topic" \
  -m "$payload"

echo "==> published event_id=$event_id"

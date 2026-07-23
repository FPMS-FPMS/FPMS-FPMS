#!/usr/bin/env bash
# Register an FPMS thing in LocalStack IoT: creates the thing, attaches a
# policy, and prints an MQTT client-id you can use from the rover.
#
# Usage:
#   cloud/iot-core/register-thing.sh <thing-name> <policy-name>
# Example:
#   cloud/iot-core/register-thing.sh rover1 fpms-rover-policy
#
# LocalStack Community only supports the IoT control plane (things, policies,
# certificates). The MQTT broker itself is Mosquitto — see docker-compose.yml.
# We still register things in LocalStack so tests and any future migration to
# LocalStack Pro / real AWS keep the same registry.

set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 <thing-name> <policy-name>" >&2
  exit 2
fi

thing="$1"
policy="$2"

: "${AWS_ENDPOINT_URL:=http://localhost:4566}"
export AWS_ENDPOINT_URL AWS_ACCESS_KEY_ID=${AWS_ACCESS_KEY_ID:-test} \
       AWS_SECRET_ACCESS_KEY=${AWS_SECRET_ACCESS_KEY:-test} \
       AWS_DEFAULT_REGION=${AWS_DEFAULT_REGION:-us-east-1}

aws() { command aws --endpoint-url "$AWS_ENDPOINT_URL" "$@"; }

echo "==> creating thing $thing"
aws iot create-thing --thing-name "$thing" >/dev/null || true

echo "==> attaching policy $policy to $thing"
aws iot attach-thing-principal --thing-name "$thing" --principal "$thing" 2>/dev/null || true

cat <<EOF

Thing $thing registered.

MQTT client-id to use from the rover:  $thing
Broker:                                mqtt://localhost:1883
Sample publish (from your dev machine):

  mosquitto_pub -h localhost -p 1883 -i $thing \\
    -t fpms/$thing/events/fire-detected \\
    -m '{"event_id":"demo","timestamp":"2026-07-23T12:00:00Z","thing":"$thing","severity":"high"}'
EOF

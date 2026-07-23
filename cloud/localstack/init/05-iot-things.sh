#!/usr/bin/env bash
# Register FPMS things and policies in LocalStack IoT (control plane only —
# the MQTT broker itself is Mosquitto, see docker-compose.yml).
#
# Kept idempotent so restarts don't fail.

set -euo pipefail

export AWS_ACCESS_KEY_ID=test
export AWS_SECRET_ACCESS_KEY=test
export AWS_DEFAULT_REGION=us-east-1

policies_dir=/opt/fpms/iot-core/policies

# The policies dir is mounted read-only via docker-compose (adjust if you
# don't mount it — the policy JSON is inlined here as a fallback).
if [[ ! -d "$policies_dir" ]]; then
  echo "[init] iot policies dir not mounted; skipping IoT registration"
  exit 0
fi

create_policy() {
  local name="$1" doc_file="$2"
  if ! awslocal iot get-policy --policy-name "$name" >/dev/null 2>&1; then
    awslocal iot create-policy \
      --policy-name "$name" \
      --policy-document "file://$doc_file" >/dev/null
    echo "[init]   policy $name created"
  fi
}

create_thing() {
  local name="$1"
  if ! awslocal iot describe-thing --thing-name "$name" >/dev/null 2>&1; then
    awslocal iot create-thing --thing-name "$name" >/dev/null
    echo "[init]   thing $name created"
  fi
}

echo "[init] registering IoT policies and things"
create_policy fpms-rover-policy "$policies_dir/rover-policy.json"
create_policy fpms-station-policy "$policies_dir/station-policy.json"

create_thing rover1
create_thing rover2
create_thing station

echo "[init] IoT ready"

#!/usr/bin/env bash
# FPMS local cloud — one command bring-up.
#
#   cloud/scripts/setup.sh
#
# Starts LocalStack + Mosquitto + rule-bridge via docker compose, then
# streams status until every service is healthy. Init hooks inside
# LocalStack create buckets, roles, Lambdas, and SNS subs automatically.

set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
cd "$here/../localstack"

echo "==> starting fpms-cloud (LocalStack + Mosquitto + rule-bridge)"
docker compose up -d --build

echo "==> waiting for LocalStack to be healthy"
until docker inspect -f '{{.State.Health.Status}}' fpms-localstack 2>/dev/null | grep -q healthy; do
  sleep 2
done

echo "==> waiting for Mosquitto to be healthy"
until docker inspect -f '{{.State.Health.Status}}' fpms-mosquitto 2>/dev/null | grep -q healthy; do
  sleep 2
done

# LocalStack's /health goes green as soon as the HTTP server binds; init hooks
# under ready.d still run afterwards. Poll the Lambda we deployed in
# 04-lambda-deploy.sh so we don't return until it actually exists.
echo "==> waiting for init hooks to finish deploying Lambdas"
export AWS_ACCESS_KEY_ID=test AWS_SECRET_ACCESS_KEY=test AWS_DEFAULT_REGION=us-east-1
tries=0
until docker exec fpms-localstack awslocal lambda get-function \
        --function-name fpms-event-router >/dev/null 2>&1; do
  tries=$((tries + 1))
  [[ $tries -gt 60 ]] && { echo "timed out waiting for fpms-event-router" >&2; exit 1; }
  sleep 2
done

# The rule bridge logs 'subscribed to fpms/+/events/#' once it is actually
# listening. Wait for that so the very first publish doesn't get dropped.
echo "==> waiting for iot-rule-bridge to subscribe"
tries=0
until docker logs fpms-rule-bridge 2>&1 | grep -q "subscribed to fpms/+/events/#"; do
  tries=$((tries + 1))
  [[ $tries -gt 30 ]] && { echo "timed out waiting for rule-bridge subscription" >&2; exit 1; }
  sleep 2
done

echo
echo "fpms-cloud is up."
echo "  LocalStack   http://localhost:4566"
echo "  Mosquitto    mqtt://localhost:1883 (ws://localhost:9001)"
echo
echo "Try:  cloud/scripts/publish-fire-event.sh"
echo "Then: cloud/scripts/check-s3.sh"

#!/usr/bin/env bash
# Create SNS topics for FPMS alerts. Idempotent.
#
# Topics:
#   fpms-fire-alerts   — event_router publishes here on fire-detected.
#                        alert_dispatcher subscribes and fans out to real
#                        channels (email/SMS in production).

set -euo pipefail

export AWS_ACCESS_KEY_ID=test
export AWS_SECRET_ACCESS_KEY=test
export AWS_DEFAULT_REGION=us-east-1

echo "[init] creating SNS topics"

arn=$(awslocal sns create-topic --name fpms-fire-alerts --query TopicArn --output text)
echo "[init]   fpms-fire-alerts -> $arn"

# Persist the ARN so later init steps can read it.
mkdir -p /tmp/fpms
echo "$arn" > /tmp/fpms/fire-alerts-topic-arn
echo "[init] SNS ready"

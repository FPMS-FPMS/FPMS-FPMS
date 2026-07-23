#!/usr/bin/env bash
# Package and deploy the FPMS Lambdas into LocalStack.
#
# Runs inside the LocalStack container, so cloud/lambda/ is mounted at
# /opt/fpms/lambda (see docker-compose.yml). We copy to a writable staging
# directory before packaging.

set -euo pipefail

export AWS_ACCESS_KEY_ID=test
export AWS_SECRET_ACCESS_KEY=test
export AWS_DEFAULT_REGION=us-east-1

src=/opt/fpms/lambda
stage=/tmp/fpms/lambda
role_arn="arn:aws:iam::000000000000:role/fpms-lambda-role"
alert_topic_arn="$(cat /tmp/fpms/fire-alerts-topic-arn)"

mkdir -p "$stage"

package() {
  local fn="$1"
  local out="$stage/$fn.zip"
  rm -f "$out"
  ( cd "$src/$fn" && zip -qr "$out" handler.py *.json 2>/dev/null || zip -qr "$out" handler.py )
  echo "$out"
}

deploy() {
  local fn="$1"; shift
  local zip="$1"; shift
  local env_json="$1"

  if awslocal lambda get-function --function-name "$fn" >/dev/null 2>&1; then
    awslocal lambda update-function-code \
      --function-name "$fn" --zip-file "fileb://$zip" >/dev/null
    awslocal lambda update-function-configuration \
      --function-name "$fn" --environment "$env_json" >/dev/null
    echo "[init]   updated $fn"
  else
    awslocal lambda create-function \
      --function-name "$fn" \
      --runtime python3.11 \
      --handler handler.handler \
      --role "$role_arn" \
      --zip-file "fileb://$zip" \
      --environment "$env_json" \
      --timeout 30 \
      --memory-size 256 >/dev/null
    echo "[init]   created $fn"
  fi
}

echo "[init] packaging Lambdas"
router_zip=$(package event_router)
dispatch_zip=$(package alert_dispatcher)

echo "[init] deploying Lambdas"
deploy fpms-event-router "$router_zip" \
  "{\"Variables\":{\"ARCHIVE_BUCKET\":\"fpms-archive\",\"ALERT_TOPIC_ARN\":\"$alert_topic_arn\",\"AWS_ENDPOINT_URL\":\"http://localstack:4566\"}}"

deploy fpms-alert-dispatcher "$dispatch_zip" \
  '{"Variables":{"LOG_LEVEL":"INFO"}}'

# Wire the dispatcher to the alerts topic.
dispatcher_arn=$(awslocal lambda get-function \
  --function-name fpms-alert-dispatcher \
  --query 'Configuration.FunctionArn' --output text)

awslocal sns subscribe \
  --topic-arn "$alert_topic_arn" \
  --protocol lambda \
  --notification-endpoint "$dispatcher_arn" >/dev/null

awslocal lambda add-permission \
  --function-name fpms-alert-dispatcher \
  --statement-id sns-invoke \
  --action lambda:InvokeFunction \
  --principal sns.amazonaws.com \
  --source-arn "$alert_topic_arn" >/dev/null 2>&1 || true

echo "[init] Lambdas ready"

#!/usr/bin/env bash
# Tail CloudWatch logs for a Lambda in LocalStack. Ctrl-C to exit.
#
# Usage:
#   cloud/scripts/tail-lambda-logs.sh fpms-event-router
#   cloud/scripts/tail-lambda-logs.sh fpms-alert-dispatcher

set -euo pipefail

fn="${1:?usage: $0 <function-name>}"
group="/aws/lambda/$fn"

export AWS_ACCESS_KEY_ID=test
export AWS_SECRET_ACCESS_KEY=test
export AWS_DEFAULT_REGION=us-east-1
endpoint="${AWS_ENDPOINT_URL:-http://localhost:4566}"

# LocalStack has cwlogs tail via the AWS CLI v2 if you have it; otherwise
# fall back to polling filter-log-events.
if aws --endpoint-url "$endpoint" logs tail "$group" --follow 2>/dev/null; then
  exit 0
fi

echo "==> polling $group (aws logs tail not available)"
seen=""
while true; do
  events=$(aws --endpoint-url "$endpoint" logs filter-log-events \
    --log-group-name "$group" \
    --query 'events[].[timestamp,message]' \
    --output text 2>/dev/null || true)
  if [[ -n "$events" && "$events" != "$seen" ]]; then
    diff <(echo "$seen") <(echo "$events") | sed -n 's/^> //p'
    seen="$events"
  fi
  sleep 2
done

#!/usr/bin/env bash
# List the archive bucket. Shows the freshest event first.
#
# Usage:
#   cloud/scripts/check-s3.sh              # list everything
#   cloud/scripts/check-s3.sh rover1       # filter to a thing
#   cloud/scripts/check-s3.sh rover1 fire-detected

set -euo pipefail

export AWS_ACCESS_KEY_ID=test
export AWS_SECRET_ACCESS_KEY=test
export AWS_DEFAULT_REGION=us-east-1
endpoint="${AWS_ENDPOINT_URL:-http://localhost:4566}"

prefix="events/"
[[ $# -ge 1 ]] && prefix="events/thing=$1/"
[[ $# -ge 2 ]] && prefix="events/thing=$1/type=$2/"

echo "==> s3://fpms-archive/$prefix"
aws --endpoint-url "$endpoint" s3 ls "s3://fpms-archive/$prefix" --recursive \
  | sort -r

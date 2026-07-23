#!/usr/bin/env bash
# Create FPMS S3 buckets on LocalStack startup. Idempotent — safe to re-run.
#
# Buckets:
#   fpms-archive   — every rover event, keyed by thing/type/date. Written by
#                    the event_router Lambda.
#   fpms-heritage  — heritage marker photos + descriptions. Versioned so a
#                    site record cannot be silently overwritten.
#   fpms-media     — public dashboard static assets (thumbnails, maps).

set -euo pipefail

export AWS_ACCESS_KEY_ID=test
export AWS_SECRET_ACCESS_KEY=test
export AWS_DEFAULT_REGION=us-east-1
alias aws='awslocal'

echo "[init] creating S3 buckets"

for b in fpms-archive fpms-heritage fpms-media; do
  if ! awslocal s3api head-bucket --bucket "$b" 2>/dev/null; then
    awslocal s3api create-bucket --bucket "$b" >/dev/null
    echo "[init]   created $b"
  else
    echo "[init]   $b already exists"
  fi
done

# Heritage bucket is versioned — a marker record is a permanent record.
awslocal s3api put-bucket-versioning \
  --bucket fpms-heritage \
  --versioning-configuration Status=Enabled >/dev/null

echo "[init] S3 ready"

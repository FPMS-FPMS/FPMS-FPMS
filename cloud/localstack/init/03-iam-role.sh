#!/usr/bin/env bash
# Create the shared Lambda execution role used by every FPMS Lambda.
#
# LocalStack does not enforce IAM by default (IAM_SOFT_MODE=1 is the default),
# but we still create the role so ARNs exist and the setup mirrors real AWS.

set -euo pipefail

export AWS_ACCESS_KEY_ID=test
export AWS_SECRET_ACCESS_KEY=test
export AWS_DEFAULT_REGION=us-east-1

role_name=fpms-lambda-role

trust='{
  "Version": "2012-10-17",
  "Statement": [{
    "Effect": "Allow",
    "Principal": {"Service": "lambda.amazonaws.com"},
    "Action": "sts:AssumeRole"
  }]
}'

echo "[init] creating IAM role $role_name"

if ! awslocal iam get-role --role-name "$role_name" >/dev/null 2>&1; then
  awslocal iam create-role \
    --role-name "$role_name" \
    --assume-role-policy-document "$trust" >/dev/null
fi

# Broad permissions for LocalStack dev — real AWS would use least-privilege.
awslocal iam attach-role-policy \
  --role-name "$role_name" \
  --policy-arn arn:aws:iam::aws:policy/AWSLambdaExecute >/dev/null 2>&1 || true

echo "[init] IAM ready"

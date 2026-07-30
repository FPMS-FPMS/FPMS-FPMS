#!/usr/bin/env bash
# Deploy FPMS Dashboard to AWS App Runner.
# One command → permanent https://xxxxx.awsapprunner.com URL.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
cd "$ROOT"

command -v aws    >/dev/null || { echo "AWS CLI required: https://aws.amazon.com/cli/" ; exit 1 ; }
command -v docker >/dev/null || { echo "Docker required." ; exit 1 ; }

: "${FPMS_PASSWORD:?Set FPMS_PASSWORD (shared password gating the dashboard)}"

AWS_REGION="${AWS_REGION:-us-east-1}"
REPO="${FPMS_ECR_REPO:-fpms-dashboard}"
SVC="${FPMS_SERVICE_NAME:-fpms-dashboard}"
ROLE_NAME="FPMSAppRunnerECRRole"

ACCOUNT=$(aws sts get-caller-identity --query Account --output text)
IMG="${ACCOUNT}.dkr.ecr.${AWS_REGION}.amazonaws.com/${REPO}:latest"

echo "→ ensuring ECR repo ${REPO}"
aws ecr describe-repositories --repository-names "$REPO" --region "$AWS_REGION" >/dev/null 2>&1 \
  || aws ecr create-repository --repository-name "$REPO" --region "$AWS_REGION" >/dev/null

echo "→ docker login"
aws ecr get-login-password --region "$AWS_REGION" \
  | docker login --username AWS --password-stdin "${ACCOUNT}.dkr.ecr.${AWS_REGION}.amazonaws.com"

echo "→ docker build (3-5 min first time)"
docker build -t fpms-dashboard:latest .

echo "→ push"
docker tag fpms-dashboard:latest "$IMG"
docker push "$IMG"

if ! aws iam get-role --role-name "$ROLE_NAME" >/dev/null 2>&1; then
  echo "→ creating IAM role ${ROLE_NAME}"
  aws iam create-role --role-name "$ROLE_NAME" \
    --assume-role-policy-document '{"Version":"2012-10-17","Statement":[{"Effect":"Allow","Principal":{"Service":"build.apprunner.amazonaws.com"},"Action":"sts:AssumeRole"}]}' >/dev/null
  aws iam attach-role-policy --role-name "$ROLE_NAME" \
    --policy-arn arn:aws:iam::aws:policy/service-role/AWSAppRunnerServicePolicyForECRAccess >/dev/null
  sleep 8
fi
ROLE_ARN=$(aws iam get-role --role-name "$ROLE_NAME" --query Role.Arn --output text)

EXISTING=$(aws apprunner list-services --region "$AWS_REGION" \
  --query "ServiceSummaryList[?ServiceName=='${SVC}'].ServiceArn" --output text)

if [ -n "$EXISTING" ] && [ "$EXISTING" != "None" ]; then
  echo "→ updating existing service"
  aws apprunner start-deployment --region "$AWS_REGION" --service-arn "$EXISTING" >/dev/null
  URL=$(aws apprunner describe-service --region "$AWS_REGION" --service-arn "$EXISTING" \
    --query 'Service.ServiceUrl' --output text)
else
  echo "→ creating new App Runner service"
  SRC=$(cat <<JSON
{
  "ImageRepository": {
    "ImageIdentifier": "${IMG}",
    "ImageRepositoryType": "ECR",
    "ImageConfiguration": {
      "Port": "8000",
      "RuntimeEnvironmentVariables": {
        "FPMS_PASSWORD": "${FPMS_PASSWORD}",
        "FPMS_BIND_HOST": "0.0.0.0",
        "FPMS_BIND_PORT": "8000"
      }
    }
  },
  "AutoDeploymentsEnabled": false,
  "AuthenticationConfiguration": { "AccessRoleArn": "${ROLE_ARN}" }
}
JSON
)
  URL=$(aws apprunner create-service \
    --service-name "$SVC" --region "$AWS_REGION" \
    --source-configuration "$SRC" \
    --instance-configuration '{"Cpu":"1024","Memory":"2048"}' \
    --query 'Service.ServiceUrl' --output text)
fi

echo ""
echo "======================================================"
echo "  Deployment kicked off."
echo "  App Runner takes 3-5 minutes to become RUNNING."
echo "  URL: https://${URL}"
echo "  Password: (whatever you set FPMS_PASSWORD to)"
echo "  Console: https://console.aws.amazon.com/apprunner"
echo "======================================================"

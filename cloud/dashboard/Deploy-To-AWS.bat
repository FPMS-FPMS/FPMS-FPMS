@echo off
REM ============================================================
REM  FPMS  ·  Deploy the dashboard to AWS App Runner
REM  Result: a permanent https://xxxxx.awsapprunner.com URL
REM  reachable from any device on any network in the world.
REM
REM  Prerequisites (one-time):
REM    - AWS CLI installed and configured (`aws configure`)
REM    - Docker Desktop running
REM    - An IAM user/role with: ecr:*, apprunner:*, iam:PassRole
REM    - A shared password picked
REM ============================================================

setlocal EnableDelayedExpansion
title FPMS · Deploy to AWS

cd /d "%~dp0"

REM ---- 0. Prereq: AWS CLI --------------------------------------
where aws >nul 2>&1
if errorlevel 1 (
    echo [~] AWS CLI not found. Installing via winget (one-time)...
    winget install --id Amazon.AWSCLI --silent --accept-package-agreements --accept-source-agreements
    REM refresh PATH for this shell
    set "PATH=%ProgramFiles%\Amazon\AWSCLIV2;%PATH%"
    where aws >nul 2>&1 || (
        echo [!] AWS CLI install failed. Install manually: https://aws.amazon.com/cli/
        pause & exit /b 1
    )
)

REM Ensure AWS CLI is configured
aws sts get-caller-identity >nul 2>&1
if errorlevel 1 (
    echo.
    echo [!] AWS CLI is not configured. Run `aws configure` first ^(access key id + secret + region^).
    echo     Get keys from: IAM Console -^> Users -^> Your user -^> Security credentials
    echo.
    pause & exit /b 1
)

REM Docker check
where docker >nul 2>&1 || set "PATH=%ProgramFiles%\Docker\Docker\resources\bin;%PATH%"
docker version >nul 2>&1 || (
    echo [!] Docker not reachable. Start Docker Desktop and re-run.
    pause & exit /b 1
)

if not defined FPMS_PASSWORD (
    set /p FPMS_PASSWORD=Pick a shared password (anyone with this can view the dashboard):
)
if "!FPMS_PASSWORD!"=="" (
    echo [!] Password is required for public deployment. Aborting.
    pause & exit /b 1
)

if not defined AWS_REGION set AWS_REGION=us-east-1
if not defined FPMS_ECR_REPO set FPMS_ECR_REPO=fpms-dashboard
if not defined FPMS_SERVICE_NAME set FPMS_SERVICE_NAME=fpms-dashboard

echo.
echo ============================================================
echo   Region: %AWS_REGION%
echo   ECR repo: %FPMS_ECR_REPO%
echo   Service: %FPMS_SERVICE_NAME%
echo ============================================================
echo.

REM ---- 1. Who am I? -------------------------------------------
for /f "usebackq tokens=*" %%A in (`aws sts get-caller-identity --query Account --output text 2^>nul`) do set ACCOUNT=%%A
if not defined ACCOUNT (
    echo [!] Failed to run 'aws sts get-caller-identity'. Configure AWS CLI first: `aws configure`
    pause & exit /b 1
)
echo [i] AWS account: %ACCOUNT%

REM ---- 2. ECR repo (idempotent) -------------------------------
echo [~] Ensuring ECR repository %FPMS_ECR_REPO% exists...
aws ecr describe-repositories --repository-names %FPMS_ECR_REPO% --region %AWS_REGION% >nul 2>&1
if errorlevel 1 (
    aws ecr create-repository --repository-name %FPMS_ECR_REPO% --region %AWS_REGION% >nul || (
        echo [!] Failed to create ECR repo. & pause & exit /b 1
    )
)

REM ---- 3. Docker login + build + push -------------------------
echo [~] Docker login to ECR...
aws ecr get-login-password --region %AWS_REGION% | docker login --username AWS --password-stdin %ACCOUNT%.dkr.ecr.%AWS_REGION%.amazonaws.com || (
    echo [!] ECR login failed. & pause & exit /b 1
)

echo [~] Building Docker image (this can take 3-5 minutes on first build)...
docker build -t fpms-dashboard:latest . || (echo [!] Build failed. & pause & exit /b 1)

echo [~] Tagging + pushing to ECR...
docker tag fpms-dashboard:latest %ACCOUNT%.dkr.ecr.%AWS_REGION%.amazonaws.com/%FPMS_ECR_REPO%:latest
docker push %ACCOUNT%.dkr.ecr.%AWS_REGION%.amazonaws.com/%FPMS_ECR_REPO%:latest || (
    echo [!] Push failed. & pause & exit /b 1
)

REM ---- 4. IAM role for App Runner to pull from ECR ------------
set "ROLE_NAME=FPMSAppRunnerECRRole"
aws iam get-role --role-name %ROLE_NAME% >nul 2>&1
if errorlevel 1 (
    echo [~] Creating IAM role %ROLE_NAME%...
    aws iam create-role --role-name %ROLE_NAME% --assume-role-policy-document "{\"Version\":\"2012-10-17\",\"Statement\":[{\"Effect\":\"Allow\",\"Principal\":{\"Service\":\"build.apprunner.amazonaws.com\"},\"Action\":\"sts:AssumeRole\"}]}" >nul
    aws iam attach-role-policy --role-name %ROLE_NAME% --policy-arn arn:aws:iam::aws:policy/service-role/AWSAppRunnerServicePolicyForECRAccess >nul
    timeout /t 8 /nobreak >nul
)
for /f "usebackq tokens=*" %%A in (`aws iam get-role --role-name %ROLE_NAME% --query Role.Arn --output text`) do set ROLE_ARN=%%A
echo [i] Access role: %ROLE_ARN%

REM ---- 5. Create-or-update App Runner service -----------------
set "IMG_ARN=%ACCOUNT%.dkr.ecr.%AWS_REGION%.amazonaws.com/%FPMS_ECR_REPO%:latest"

powershell -NoProfile -Command "$env:IMG='%IMG_ARN%'; $env:ROLE='%ROLE_ARN%'; $env:PW='%FPMS_PASSWORD%'; $env:SVC='%FPMS_SERVICE_NAME%'; $env:REG='%AWS_REGION%'; $existing = (aws apprunner list-services --region $env:REG --query \"ServiceSummaryList[?ServiceName=='$($env:SVC)'].ServiceArn\" --output text 2>$null); if ($existing) { Write-Host \"[~] Updating existing service $($env:SVC)...\"; aws apprunner start-deployment --region $env:REG --service-arn $existing | Out-Null; Write-Host \"[i] Redeployment started. URL:\"; aws apprunner describe-service --region $env:REG --service-arn $existing --query 'Service.ServiceUrl' --output text } else { Write-Host \"[~] Creating new App Runner service $($env:SVC)...\"; $cfg = @{ ImageRepository = @{ ImageIdentifier = $env:IMG; ImageRepositoryType = 'ECR'; ImageConfiguration = @{ Port = '8000'; RuntimeEnvironmentVariables = @{ FPMS_PASSWORD = $env:PW; FPMS_BIND_HOST = '0.0.0.0'; FPMS_BIND_PORT = '8000' } } }; AutoDeploymentsEnabled = $false; AuthenticationConfiguration = @{ AccessRoleArn = $env:ROLE } } | ConvertTo-Json -Depth 10 -Compress; $inst = '{\"Cpu\":\"1024\",\"Memory\":\"2048\"}'; aws apprunner create-service --service-name $env:SVC --region $env:REG --source-configuration $cfg --instance-configuration $inst --query 'Service.ServiceUrl' --output text }"

echo.
echo ============================================================
echo   Deployment kicked off. App Runner takes 3-5 minutes to
echo   pull the image and start serving. Watch progress at:
echo     https://console.aws.amazon.com/apprunner
echo.
echo   Once status is RUNNING, open the URL from any device.
echo   Login with password: (whatever you entered)
echo ============================================================
echo.
pause
endlocal

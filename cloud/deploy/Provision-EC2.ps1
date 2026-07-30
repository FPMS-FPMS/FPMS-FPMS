<#
.SYNOPSIS
  Provision the EC2 host for the FPMS cloud app, then deploy onto it.

.DESCRIPTION
  Creates everything the cloud app needs on AWS free tier:
    * SSH key pair (saved locally, chmod-equivalent applied)
    * Security group with least-privilege ingress
    * t3.micro Ubuntu 24.04 instance (750 hrs/month free for 12 months)
    * IAM instance role for S3 / SNS / CloudWatch  (-WithIamRole)

  Ingress is deliberately narrow:
    22    your current public IP only
    8883  0.0.0.0/0   - field rovers on cellular have no fixed IP
    8000  Cloudflare ranges only - the origin must not be reachable directly
    1883  never opened - plaintext MQTT stays inside the Docker network

.PARAMETER Region
  AWS region. ca-central-1 is closest to Toronto; us-east-1 is cheapest.

.EXAMPLE
  .\Provision-EC2.ps1 -Region ca-central-1 -WithIamRole
#>
[CmdletBinding()]
param(
    [string]$Region = 'ca-central-1',
    [string]$Name = 'fpms-cloud',
    [string]$InstanceType = 't3.micro',
    [switch]$WithIamRole,
    [string]$KeyPath = "$env:USERPROFILE\.ssh"
)

$ErrorActionPreference = 'Stop'

function Need($cmd) {
    if (-not (Get-Command $cmd -ErrorAction Ignore)) { throw "$cmd not found on PATH." }
}
Need aws

Write-Host "==> checking credentials"
$who = aws sts get-caller-identity --output json 2>&1
if ($LASTEXITCODE -ne 0) {
    throw "AWS credentials are not configured. Run:  aws configure --region $Region"
}
$acct = ($who | ConvertFrom-Json).Account
Write-Host "    account $acct, region $Region"

# --- SSH key pair ----------------------------------------------------------
if (-not (Test-Path $KeyPath)) { New-Item -ItemType Directory -Path $KeyPath -Force | Out-Null }
$pem = Join-Path $KeyPath "$Name.pem"
$existing = aws ec2 describe-key-pairs --key-names $Name --region $Region 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "==> creating key pair $Name"
    aws ec2 create-key-pair --key-name $Name --region $Region `
        --query 'KeyMaterial' --output text | Out-File -FilePath $pem -Encoding ascii
    # SSH refuses keys that other users can read.
    icacls $pem /inheritance:r /grant:r "$($env:USERNAME):(R)" | Out-Null
    Write-Host "    saved $pem"
} else {
    Write-Host "==> key pair $Name already exists"
    if (-not (Test-Path $pem)) {
        throw "AWS has key pair '$Name' but $pem is missing. Delete the AWS key pair or restore the .pem - you cannot re-download it."
    }
}

# --- security group --------------------------------------------------------
$myIp = (Invoke-RestMethod -Uri 'https://api.ipify.org?format=json' -TimeoutSec 10).ip
Write-Host "==> security group (your IP: $myIp)"

$sgId = aws ec2 describe-security-groups --group-names $Name --region $Region `
    --query 'SecurityGroups[0].GroupId' --output text 2>$null
if ($LASTEXITCODE -ne 0 -or -not $sgId -or $sgId -eq 'None') {
    $sgId = aws ec2 create-security-group --group-name $Name --region $Region `
        --description 'FPMS cloud app: dashboard via Cloudflare, MQTT/TLS for rovers' `
        --query 'GroupId' --output text
    Write-Host "    created $sgId"
} else {
    Write-Host "    reusing $sgId"
}

function Allow($port, $cidr, $why) {
    aws ec2 authorize-security-group-ingress --group-id $sgId --region $Region `
        --ip-permissions "IpProtocol=tcp,FromPort=$port,ToPort=$port,IpRanges=[{CidrIp=$cidr,Description='$why'}]" 2>$null | Out-Null
}

Allow 22 "$myIp/32" 'admin SSH'
Allow 8883 '0.0.0.0/0' 'field rovers MQTT over TLS'

# Origin lock: only Cloudflare may reach the dashboard port, so nobody can
# bypass the proxy and hit the origin IP directly.
Write-Host "==> allowing port 8000 from Cloudflare ranges only"
$cf = (Invoke-RestMethod -Uri 'https://api.cloudflare.com/client/v4/ips').result
$ranges = @($cf.ipv4_cidrs)
foreach ($c in $ranges) { Allow 8000 $c 'Cloudflare proxy' }
Write-Host "    $($ranges.Count) Cloudflare IPv4 ranges allowed"

# --- IAM instance role (optional) -----------------------------------------
$iamArg = @()
if ($WithIamRole) {
    $roleName = "$Name-role"
    Write-Host "==> IAM instance role $roleName"
    $trust = '{"Version":"2012-10-17","Statement":[{"Effect":"Allow","Principal":{"Service":"ec2.amazonaws.com"},"Action":"sts:AssumeRole"}]}'
    $trustFile = New-TemporaryFile
    Set-Content -Path $trustFile -Value $trust -Encoding ascii
    aws iam create-role --role-name $roleName --assume-role-policy-document "file://$trustFile" 2>$null | Out-Null

    $policy = @'
{"Version":"2012-10-17","Statement":[
 {"Effect":"Allow","Action":["s3:PutObject","s3:GetObject","s3:ListBucket"],"Resource":["arn:aws:s3:::fpms-archive","arn:aws:s3:::fpms-archive/*"]},
 {"Effect":"Allow","Action":["sns:Publish"],"Resource":"*"},
 {"Effect":"Allow","Action":["logs:CreateLogGroup","logs:CreateLogStream","logs:PutLogEvents"],"Resource":"*"}]}
'@
    $polFile = New-TemporaryFile
    Set-Content -Path $polFile -Value $policy -Encoding ascii
    aws iam put-role-policy --role-name $roleName --policy-name fpms-services `
        --policy-document "file://$polFile" | Out-Null
    aws iam create-instance-profile --instance-profile-name $roleName 2>$null | Out-Null
    aws iam add-role-to-instance-profile --instance-profile-name $roleName --role-name $roleName 2>$null | Out-Null
    Start-Sleep -Seconds 10   # IAM is eventually consistent; RunInstances can 404 otherwise
    $iamArg = @('--iam-instance-profile', "Name=$roleName")
    Write-Host "    role attached (no static keys needed on the host)"
}

# --- launch ----------------------------------------------------------------
Write-Host "==> resolving latest Ubuntu 24.04 AMI"
$ami = aws ssm get-parameters --region $Region `
    --names /aws/service/canonical/ubuntu/server/24.04/stable/current/amd64/hvm/ebs-gp3/ami-id `
    --query 'Parameters[0].Value' --output text
if (-not $ami -or $ami -eq 'None') { throw "Could not resolve an Ubuntu 24.04 AMI in $Region" }
Write-Host "    $ami"

$running = aws ec2 describe-instances --region $Region `
    --filters "Name=tag:Name,Values=$Name" 'Name=instance-state-name,Values=pending,running' `
    --query 'Reservations[0].Instances[0].InstanceId' --output text 2>$null
if ($running -and $running -ne 'None') {
    Write-Host "==> instance already running: $running (not launching another)"
    $id = $running
} else {
    Write-Host "==> launching $InstanceType"
    $id = (aws ec2 run-instances --region $Region --image-id $ami --instance-type $InstanceType `
        --key-name $Name --security-group-ids $sgId `
        --block-device-mappings 'DeviceName=/dev/sda1,Ebs={VolumeSize=30,VolumeType=gp3}' `
        --tag-specifications "ResourceType=instance,Tags=[{Key=Name,Value=$Name}]" `
        @iamArg --query 'Instances[0].InstanceId' --output text)
    Write-Host "    $id - waiting for running state"
    aws ec2 wait instance-running --instance-ids $id --region $Region
}

$ip = aws ec2 describe-instances --instance-ids $id --region $Region `
    --query 'Reservations[0].Instances[0].PublicIpAddress' --output text

Write-Host ""
Write-Host "============================================================"
Write-Host "  EC2 ready"
Write-Host "    instance : $id"
Write-Host "    public IP: $ip"
Write-Host "    key      : $pem"
Write-Host ""
Write-Host "  Deploy (wait ~60s for first boot to finish):"
Write-Host "    scp -i `"$pem`" -r . ubuntu@${ip}:~/fpms-deploy"
Write-Host "    ssh -i `"$pem`" ubuntu@$ip"
Write-Host "    cd ~/fpms-deploy && chmod +x Deploy-Cloud.sh"
Write-Host "    ./Deploy-Cloud.sh '<dashboard-pw>' '<broker-pw>' mqtt.yourdomain.com"
Write-Host ""
Write-Host "  Then Cloudflare DNS:"
Write-Host "    A  fpms  ->  $ip   (Proxied)"
Write-Host "    A  mqtt  ->  $ip   (DNS only - MQTT is not HTTP)"
Write-Host "============================================================"

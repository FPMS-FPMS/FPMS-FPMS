# List the archive bucket. Talks to the local moto S3 mock started by
# pipeline.py.
#
# Usage:
#   .\check-archive.ps1                       # list everything
#   .\check-archive.ps1 rover1                # filter to a thing
#   .\check-archive.ps1 rover1 fire-detected  # filter to thing + type

param(
    [string]$Thing = "",
    [string]$Type  = ""
)

$env:AWS_ACCESS_KEY_ID     = "test"
$env:AWS_SECRET_ACCESS_KEY = "test"
$env:AWS_DEFAULT_REGION    = "us-east-1"

$prefix = "events/"
if ($Thing) { $prefix = "events/thing=$Thing/" }
if ($Thing -and $Type) { $prefix = "events/thing=$Thing/type=$Type/" }

Write-Host "s3://fpms-archive/$prefix"
aws --endpoint-url http://127.0.0.1:4566 s3 ls "s3://fpms-archive/$prefix" --recursive

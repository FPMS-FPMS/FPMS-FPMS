# Switch FPMS Dashboard to real AWS IoT Core

By default the dashboard uses **LocalStack** (AWS running on your laptop).
Follow this to point it at **real AWS** in your account instead.

## What you need

- An AWS account (the AWS IoT Core free tier is 500k messages / 250k minutes
  connect time / month — more than enough for FPMS demos)
- AWS CLI installed and configured (`aws configure`) with a user or role that
  has `iot:*`, `s3:*` (on `fpms-archive`), `lambda:*`, and `sns:*`
- Your AWS IoT Core endpoint. Get it with:
  ```
  aws iot describe-endpoint --endpoint-type iot:Data-ATS
  ```
  It looks like `abc1234-ats.iot.us-east-1.amazonaws.com`.

## One-time AWS setup

Run once in your AWS account (same commands the `cloud/localstack/init/`
scripts run against LocalStack):

```bash
# S3 archive bucket
aws s3 mb s3://fpms-archive

# SNS topic for alerts
aws sns create-topic --name fpms-alerts

# IoT policy so registered Things can connect + publish
aws iot create-policy --policy-name fpms-rover-policy \
  --policy-document file://cloud/iot-core/policies/rover.json
```

The Lambda functions (`event_router`, `alert_dispatcher`) live in
`cloud/lambda/` — deploy them with your usual pipeline (SAM, CDK, Terraform)
or with the `awslocal` scripts pointed at real AWS.

## Launch the dashboard in cloud mode

Set two environment variables before running `FPMS-Dashboard.bat`:

```powershell
setx FPMS_AWS_MODE cloud
setx FPMS_MQTT_HOST abc1234-ats.iot.us-east-1.amazonaws.com
setx FPMS_MQTT_PORT 8883
setx FPMS_MQTT_TLS 1
# and your device X.509 cert files so the dashboard can subscribe:
setx FPMS_MQTT_CA_CERT  "C:\path\to\AmazonRootCA1.pem"
setx FPMS_MQTT_CLIENT_CERT "C:\path\to\dashboard-cert.pem"
setx FPMS_MQTT_CLIENT_KEY  "C:\path\to\dashboard-key.pem"
```

Open a fresh terminal (so `setx` takes effect), then double-click
`FPMS-Dashboard.bat`.

The AWS chip in the top nav flips from **AWS · Local** (orange) to
**AWS · Cloud** (green with an AWS logo). Every service card on the AWS tab
now shows live counts from your real account.

## Rover-side

Provisioning from the Devices tab creates the Thing + X.509 certs in your
real AWS account and SCPs the systemd publisher unit onto the Pi. The
publisher connects to `mqtts://<your-endpoint>:8883` and publishes to the
same `fpms/<thing>/telemetry/*` topics — no code change.

## Cost note

At FPMS's telemetry rate (5 Hz LiDAR + 5 Hz camera + 4 Hz thermal ≈ 14
msgs/sec per rover, ~1 MB/s of camera JPEGs) you'll hit ~1.2M messages/day
per rover. That's well under free-tier for messages but the JPEG payload
volume adds real bandwidth cost. Consider throttling the camera stream or
downsampling frames for cloud mode; the LocalStack mode has no such limits.

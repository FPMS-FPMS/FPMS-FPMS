# Deploy FPMS Dashboard to AWS

Puts the whole backend on AWS with a real HTTPS URL that works from any
device, any network, anywhere — no tunnel, no LAN weirdness. Recommended
target: **AWS App Runner** (managed, ~5 min setup, free tier covers demos).

Alternative targets that work with the same image: ECS Fargate, Lightsail
Container, EC2 with Docker, or non-AWS: Cloud Run / Fly.io / Railway.

## Prerequisites

- AWS account (free tier is fine)
- AWS CLI installed and configured (`aws configure`)
- Docker Desktop running locally (only for the build)
- A domain is optional — App Runner gives you `<random>.awsapprunner.com`

## Fast path: App Runner from source (no Docker push needed)

App Runner can build directly from your GitHub repo — no Dockerfile push.
Fork this repo, then in the AWS Console:

1. **App Runner → Create service**
2. **Source: source code repository → GitHub** → authorize, pick your fork
3. **Deployment settings → Automatic** (redeploys on push)
4. **Build settings → Configure all settings here:**
   - Runtime: Python 3.11
   - Build command: `pip install -r cloud/dashboard/backend/requirements.txt && (cd cloud/dashboard/frontend && npm install && npm run build)`
   - Start command: `python -m uvicorn cloud.dashboard.backend.main:app --host 0.0.0.0 --port 8080`
   - Port: `8080`
5. **Environment variables:**
   - `FPMS_AWS_MODE=cloud`
   - `FPMS_PASSWORD=your-strong-shared-password`  ← required for public access
   - `FPMS_MQTT_HOST=<your-iot-endpoint>-ats.iot.us-east-1.amazonaws.com`
   - `FPMS_MQTT_PORT=8883`
   - `FPMS_MQTT_TLS=1`
   - (cert files are read from S3 — see step 7)
6. Configure service (name, memory 1 GB, CPU 1 vCPU is plenty)
7. Create — App Runner builds and gives you `https://xxxxx.us-east-1.awsapprunner.com`

Open the URL from any phone → login screen → dashboard.

## Container path (any host)

```bash
cd cloud/dashboard

# 1. Build the image
docker build -t fpms-dashboard:latest .

# 2. Test it locally
docker run --rm -p 8000:8000 \
  -e FPMS_PASSWORD=test \
  fpms-dashboard:latest
# Open http://localhost:8000 — should show login page.

# 3. Push to Amazon ECR
aws ecr create-repository --repository-name fpms-dashboard
ACCOUNT=$(aws sts get-caller-identity --query Account --output text)
REGION=us-east-1
aws ecr get-login-password --region $REGION | docker login --username AWS --password-stdin $ACCOUNT.dkr.ecr.$REGION.amazonaws.com
docker tag fpms-dashboard:latest $ACCOUNT.dkr.ecr.$REGION.amazonaws.com/fpms-dashboard:latest
docker push $ACCOUNT.dkr.ecr.$REGION.amazonaws.com/fpms-dashboard:latest

# 4. Deploy to App Runner (points at the ECR image)
aws apprunner create-service \
  --service-name fpms-dashboard \
  --source-configuration '{
    "ImageRepository": {
      "ImageIdentifier": "'$ACCOUNT'.dkr.ecr.'$REGION'.amazonaws.com/fpms-dashboard:latest",
      "ImageRepositoryType": "ECR",
      "ImageConfiguration": {
        "Port": "8000",
        "RuntimeEnvironmentVariables": {
          "FPMS_PASSWORD": "your-strong-password",
          "FPMS_AWS_MODE": "cloud",
          "FPMS_MQTT_HOST": "xxxxx-ats.iot.us-east-1.amazonaws.com",
          "FPMS_MQTT_PORT": "8883",
          "FPMS_MQTT_TLS": "1"
        }
      }
    },
    "AutoDeploymentsEnabled": false
  }' \
  --instance-configuration '{"Cpu":"1024","Memory":"2048","InstanceRoleArn":"<arn-of-iam-role-with-iot-perms>"}'
```

App Runner returns the service URL — that's what you share.

## Rover-side: publish to real IoT Core

Every Orange Pi 5B needs a Thing + certificate in your AWS account.

Use the app's own onboarding: run FPMS-Dashboard.exe on your laptop with
`FPMS_AWS_MODE=cloud` and normal AWS credentials, open the Devices tab,
scan → SSH → Register + install. The app creates the Thing in real AWS
IoT Core, generates the X.509 cert, SCPs it to the Pi, installs a systemd
publisher unit that connects to `mqtts://<endpoint>:8883` and streams to
`fpms/<thing>/telemetry/#`.

The App-Runner-hosted backend is subscribed to the same topic tree, so
any device opening the hosted URL sees the streams immediately.

## What runs where after this

```
Orange Pi 5B rover   ──MQTT/TLS 8883──▶   AWS IoT Core
                                              │
                                              │  IoT Rule: fpms/+/telemetry/#
                                              ▼
                                       AWS App Runner (this image)
                                              │
                                              │  WebSocket
                                              ▼
                        📱 phone / 💻 laptop / 🖥 desktop — anywhere in the world
```

Nothing on your laptop needs to be running. `FPMS-Dashboard.exe` becomes
a **desktop viewer** for the same hosted backend (or a local-only variant
for LAN-only demos, either).

## Cost sanity check (free tier / us-east-1)

- **App Runner**: 1 vCPU / 2 GB / active hours only. Free tier: 200 build
  minutes + limited runtime. For an always-on demo, ~$25–50/month at full
  utilization. Pause the service when not demoing to pay $0.
- **IoT Core**: 500k messages/month free forever. FPMS at 15 msg/s per
  rover = ~40M msg/month per rover → beyond free tier at $1/million.
  Sample every N frames (already in the config) if cost matters.
- **S3 (event archive)**: pennies.

Total for a demo weekend: about $2. For a competition day: less than a coffee.

## Rollback / take it down

```bash
aws apprunner delete-service --service-arn <the-arn-from-create>
```

The URL stops resolving. No orphaned resources. IoT Things + certs stay
until you delete them from the IoT Console (safe to leave for reuse).

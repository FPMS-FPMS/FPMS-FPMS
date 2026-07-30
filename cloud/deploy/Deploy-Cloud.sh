#!/usr/bin/env bash
# ============================================================================
#  FPMS Cloud app — one-shot bootstrap for a fresh Ubuntu host.
#
#  Target: AWS EC2 t3.micro (free tier, 12 months) or any 1 GB VPS.
#  Brings up the MQTT broker (TLS, for field rovers) and the dashboard in
#  cloud role, then prints exactly what to put in Cloudflare DNS.
#
#  Usage, as a normal sudo-capable user:
#     ./Deploy-Cloud.sh <dashboard-password> <broker-password> [mqtt-hostname]
#
#  mqtt-hostname is the DNS name rovers will use for MQTT, e.g.
#  mqtt.yourdomain.com. It goes in the TLS certificate, so pass it if you have
#  it; otherwise the public IP is used and rovers must disable hostname checks.
# ============================================================================
set -euo pipefail

DASH_PW="${1:?usage: ./Deploy-Cloud.sh <dashboard-password> <broker-password> [mqtt-hostname]}"
MQTT_PW="${2:?broker password required}"
MQTT_HOST="${3:-}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

echo "==> 1/6 installing docker"
if ! command -v docker >/dev/null 2>&1; then
    sudo apt-get update -y
    sudo apt-get install -y ca-certificates curl gnupg openssl
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
        | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
        | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
    sudo apt-get update -y
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
    sudo usermod -aG docker "$USER" || true
else
    echo "    docker already present"
fi

PUBLIC_IP="$(curl -fsS --max-time 5 https://api.ipify.org || echo '')"
CERT_CN="${MQTT_HOST:-${PUBLIC_IP:-localhost}}"

echo "==> 2/6 broker credentials for user 'fpms'"
mkdir -p mosquitto/certs
# Generate the password file using the broker image itself, so we don't need
# mosquitto-clients installed on the host.
sudo docker run --rm -v "$HERE/mosquitto:/work" eclipse-mosquitto:2 \
    mosquitto_passwd -c -b /work/passwd fpms "$MQTT_PW"

echo "==> 3/6 TLS certificate for $CERT_CN"
if [ ! -f mosquitto/certs/server.crt ]; then
    # Self-signed CA + server cert, valid 10 years. Rovers pin this CA, so a
    # self-signed chain is genuinely fine here - no public trust needed, and it
    # avoids a renewal that would silently break the fleet mid-season.
    openssl req -new -x509 -days 3650 -extensions v3_ca -nodes \
        -keyout mosquitto/certs/ca.key -out mosquitto/certs/ca.crt \
        -subj "/CN=FPMS-CA" 2>/dev/null
    openssl genrsa -out mosquitto/certs/server.key 2048 2>/dev/null
    openssl req -new -key mosquitto/certs/server.key \
        -out mosquitto/certs/server.csr -subj "/CN=$CERT_CN" 2>/dev/null
    openssl x509 -req -in mosquitto/certs/server.csr -days 3650 \
        -CA mosquitto/certs/ca.crt -CAkey mosquitto/certs/ca.key -CAcreateserial \
        -out mosquitto/certs/server.crt 2>/dev/null
    rm -f mosquitto/certs/server.csr
    echo "    generated CA + server cert (CN=$CERT_CN)"
else
    echo "    certs already exist, keeping them"
fi
# The broker runs as uid 1883 inside the image and must read these.
sudo chown -R 1883:1883 mosquitto/certs mosquitto/passwd 2>/dev/null || true
sudo chmod 600 mosquitto/certs/*.key mosquitto/passwd 2>/dev/null || true

echo "==> 4/6 writing .env"
if [ -f .env ]; then
    cp .env ".env.bak.$(date +%s)"
    echo "    existing .env backed up"
fi
cat > .env <<ENV
FPMS_PASSWORD=$DASH_PW
FPMS_MQTT_USERNAME=fpms
FPMS_MQTT_PASSWORD=$MQTT_PW

# --- AWS services ----------------------------------------------------------
# Flip to "cloud" to use your real AWS account for S3 archiving, SNS alerts,
# IoT Core and CloudWatch. On EC2, attach an IAM instance role and leave the
# keys blank - boto3 reads the role from instance metadata, so no long-lived
# credentials ever sit on disk.
FPMS_AWS_MODE=local
AWS_DEFAULT_REGION=${AWS_DEFAULT_REGION:-us-east-1}
AWS_ACCESS_KEY_ID=
AWS_SECRET_ACCESS_KEY=
FPMS_ARCHIVE_BUCKET=fpms-archive
ENV
chmod 600 .env

echo "==> 5/6 building and starting"
sudo docker compose -f docker-compose.cloud.yml up -d --build

echo "==> 6/6 waiting for health"
ok=0
for _ in $(seq 1 30); do
    sleep 3
    if curl -fsS --max-time 3 http://127.0.0.1:8000/api/health >/dev/null 2>&1; then ok=1; break; fi
done

echo
echo "============================================================"
if [ "$ok" = "1" ]; then
    echo "  Dashboard is UP on port 8000"
else
    echo "  Dashboard did NOT answer. Check: sudo docker compose -f docker-compose.cloud.yml logs"
fi
sudo docker compose -f docker-compose.cloud.yml ps
echo
echo "  Cloudflare DNS to add (both PROXIED unless noted):"
echo "    A    fpms    ->  ${PUBLIC_IP:-<this-host-public-ip>}     (proxied: yes)"
echo "    A    mqtt    ->  ${PUBLIC_IP:-<this-host-public-ip>}     (proxied: NO - MQTT is not HTTP)"
echo
echo "  MQTT for rovers:  ${MQTT_HOST:-mqtt.yourdomain.com}:8883  (TLS, user 'fpms')"
echo "  Copy mosquitto/certs/ca.crt to each rover so it can verify the broker."
echo
echo "  Open ports: 8000 (Cloudflare only) and 8883 (rovers). Keep 1883 closed."
echo "============================================================"

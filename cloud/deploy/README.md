# FPMS — two apps, one system

FPMS ships as **two separate applications**. They are not the same build with a
flag flipped by accident; the split is enforced in code by `FPMS_ROLE`.

| | **Edge app** | **Cloud app** |
|---|---|---|
| Runs on | your own machine | a cloud host (EC2 / any VPS) |
| Install | `FPMS-Dashboard-Setup.exe` | `docker compose` |
| Purpose | **control** the rovers | **stream** data to the world |
| Terminal / SSH / LAN scan / provisioning | **yes, unrestricted** | **refused, always** |
| Rover data | yes | yes |
| Reachable from | this machine + your LAN | anywhere on earth |
| `FPMS_ROLE` | `edge` (default) | `cloud` |

The cloud app refuses machine-control routes for **every** request, not just
ones that look remote. On a custom domain there's no tunnel hostname to detect,
and a `Host` header can be forged — the role comes from the process's own
environment, so it can't be spoofed by a visitor.

---

## Cloud app

### What runs

```
field rovers (cellular) ──MQTT/TLS 8883──▶ mosquitto ──1883 internal──▶ dashboard
                                                                            │
                                                              Cloudflare (free)
                                                                            │
                                                                fpms.yourdomain.com
```

Two containers, one host. Cloudflare provides SSL, CDN and DDoS protection for
free, so the host only serves plain HTTP on 8000 to Cloudflare.

### Deploy

On a fresh Ubuntu host — AWS EC2 `t3.micro` is free for 12 months, and any 1 GB
VPS works:

```bash
git clone <your-repo> && cd cloud/deploy
chmod +x Deploy-Cloud.sh
./Deploy-Cloud.sh '<dashboard-password>' '<broker-password>' mqtt.yourdomain.com
```

It installs Docker, generates the broker password file and a 10-year self-signed
CA + server certificate, builds the dashboard image, starts both containers,
waits for health, and prints the exact DNS records to add.

### Cloudflare DNS

| Type | Name | Value | Proxy |
|---|---|---|---|
| A | `fpms` | host public IP | **Proxied** |
| A | `mqtt` | host public IP | **DNS only** |

`mqtt` **must not** be proxied. Cloudflare's free plan proxies HTTP/HTTPS only —
MQTT is raw TCP and would be dropped. Proxying `fpms` is what gives you free
SSL, caching and DDoS protection.

### Firewall / security group

| Port | Open to | Why |
|---|---|---|
| 8000 | Cloudflare IP ranges only | dashboard; never expose directly |
| 8883 | anywhere | field rovers over TLS |
| 1883 | **closed** | plaintext, internal to the compose network |
| 22 | your IP | admin |

Restricting 8000 to [Cloudflare's published ranges](https://www.cloudflare.com/ips/)
stops anyone bypassing the proxy and hitting the origin directly.

### Connecting field rovers

From the **edge app** (it holds the SSH capability the cloud app deliberately
lacks), with the CA from `deploy/mosquitto/certs/ca.crt`:

```bash
curl -X POST http://localhost:8000/api/discovery/provision \
  -H 'Content-Type: application/json' \
  -d '{"ip":"<rover-ip>","username":"orangepi","password":"<pw>",
       "thing_name":"rover1",
       "broker_tls":true,
       "broker_host":"mqtt.yourdomain.com",
       "broker_ca_cert":"-----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----"}'
```

Response confirms the transport:

```json
{"thing":"rover1","transport":"cloud-mqtt-tls","iot_endpoint":"mqtt.yourdomain.com","mqtt_port":8883}
```

The rover then publishes from anywhere with a signal — no VPN, no port
forwarding, no dependence on your laptop being awake.

### AWS services (EC2, S3, SNS, IoT, CloudWatch)

The cloud app runs **on** AWS and can also talk **to** AWS. Two independent
things, so don't conflate them:

| Layer | Component | Free tier |
|---|---|---|
| Compute | **EC2** `t3.micro` running both containers | 750 hrs/mo for 12 months |
| Archive | **S3** — `FPMS_ARCHIVE_BUCKET` | 5 GB for 12 months |
| Alerts | **SNS** — email/SMS on rover events | 1M publishes/mo, always free |
| Logs | **CloudWatch Logs** | 5 GB/mo, always free |
| Devices *(optional)* | **IoT Core** instead of Mosquitto | 500k messages/mo for 12 months |
| Edge | **Cloudflare** — SSL, CDN, DDoS, DNS | always free |

By default `FPMS_AWS_MODE=local`, which points boto3 at LocalStack and touches
nothing real. To use your actual account, set `FPMS_AWS_MODE=cloud` in `.env`.

**Use an IAM instance role, not access keys.** Attach a role to the EC2 instance
and leave `AWS_ACCESS_KEY_ID` / `AWS_SECRET_ACCESS_KEY` blank — boto3 reads
temporary credentials from instance metadata and rotates them automatically. A
static key pair in a `.env` on an internet-facing host is the single most common
way AWS accounts get compromised. A least-privilege policy is enough:

```json
{
  "Version": "2012-10-17",
  "Statement": [
    { "Effect": "Allow",
      "Action": ["s3:PutObject", "s3:GetObject", "s3:ListBucket"],
      "Resource": ["arn:aws:s3:::fpms-archive", "arn:aws:s3:::fpms-archive/*"] },
    { "Effect": "Allow", "Action": ["sns:Publish"], "Resource": "*" },
    { "Effect": "Allow",
      "Action": ["logs:CreateLogStream", "logs:PutLogEvents", "logs:CreateLogGroup"],
      "Resource": "*" }
  ]
}
```

**Why Cloudflare in front of EC2 rather than CloudFront:** Cloudflare's free plan
covers SSL, CDN, DDoS protection and DNS at no cost, and — the part that
actually saves money — cached static assets are served from Cloudflare's edge,
so they never reach EC2 and never incur AWS egress charges. AWS bills egress at
~$0.09/GB after the free allowance; Cloudflare doesn't bill egress at all.

**Where MQTT fits:** you have two options and the code supports both.
Self-hosted Mosquitto in this compose stack is free and unmetered. AWS IoT Core
is metered per message but adds per-device X.509 certificates, revocation and
fleet management — worth it at dozens of rovers, overkill at two. Switch by
passing `"use_aws": true` when provisioning.

### Message volume — read this before scaling

Telemetry rate drives cost on any metered broker, and it's tuned by
`FPMS_PUBLISH_INTERVAL` in `/etc/fpms/config.env` on each rover (default 5s).

| Interval | Messages/month (2 rovers) |
|---|---|
| 0.5s (the old default) | ~10.4M |
| 5s (current default) | ~1M |

Detections and alerts publish to `fpms/<thing>/events/…` at **QoS 1** so they
cannot be silently dropped. Pose telemetry stays QoS 0 — losing one sample
between 5-second updates costs nothing, and QoS 1 on a firehose is expensive.

### Operations

```bash
docker compose -f docker-compose.cloud.yml ps
docker compose -f docker-compose.cloud.yml logs -f dashboard
docker compose -f docker-compose.cloud.yml restart
docker compose -f docker-compose.cloud.yml up -d --build   # after a code change
```

---

## Edge app

Install `FPMS-Dashboard-Setup.exe`. Start Menu and desktop shortcuts are
created; it runs in a native WebView2 window on `127.0.0.1:8000`.

Full control, no restrictions: terminal, SSH, LAN scan, rover provisioning.
Nothing about it depends on Cloudflare or the cloud app — it can drive rovers on
your own WiFi with the local broker, and it's the tool you use to provision
rovers for the cloud broker too.

To also expose the edge app publicly (optional, and separate from the cloud
app), `Publish-Public-Persistent.bat` raises a Cloudflare tunnel and registers
it with the gateway Worker. Safe mode then hides the control tabs from public
visitors while keeping them for you locally.

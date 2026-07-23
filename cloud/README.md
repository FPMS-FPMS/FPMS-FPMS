# FPMS cloud — local AWS via LocalStack

This directory is the cloud half of FPMS. It runs the whole stack —
**S3 archive, Lambda event routing, and IoT Core MQTT** — **entirely on your
laptop**, no AWS account required. The rover code and topic taxonomy are
identical to what would run against real AWS; only the endpoints change.

```
┌──────────────┐   MQTT   ┌──────────────┐   invoke  ┌──────────────┐
│  Rover 1/2   │──────────▶│  Mosquitto   │──────────▶│  Rule bridge │
│  Station     │   1883    │  (IoT Core)  │           │  (Python)    │
└──────────────┘           └──────────────┘           └───────┬──────┘
                                                              │  invoke
                                                              ▼
                                                     ┌─────────────────┐
                                                     │  event_router   │──▶ S3
                                                     │  (Lambda)       │──▶ SNS
                                                     └─────────────────┘
                                                              │
                                                              ▼
                                                     ┌─────────────────┐
                                                     │ alert_dispatcher│──▶ logs
                                                     │  (Lambda)       │   (→ email/SMS in prod)
                                                     └─────────────────┘
```

## What runs where

| Piece                     | Container        | Local port     | Real AWS analog          |
|---------------------------|------------------|----------------|--------------------------|
| S3 buckets                | `localstack`     | 4566           | Amazon S3                |
| Lambda functions          | `localstack`     | 4566           | AWS Lambda               |
| SNS topic                 | `localstack`     | 4566           | Amazon SNS               |
| IoT registry (things, policies) | `localstack` | 4566        | AWS IoT Core control plane |
| MQTT broker               | `mosquitto`      | 1883 / 9001    | AWS IoT Core MQTT broker |
| MQTT → Lambda routing     | `iot-rule-bridge`| —              | AWS IoT Rules            |

**Why Mosquitto?** LocalStack Community edition doesn't ship the full IoT
Core MQTT broker or IoT Rules engine (those are Pro features). Mosquitto is
a real, standards-compliant MQTT broker — your rover code sees the same
protocol as production AWS IoT Core. The small `iot-rule-bridge` container
does what an IoT Rule would do in production: subscribe to a topic filter
and invoke a Lambda.

## Requirements

- Docker + `docker compose` (v2 syntax)
- `awslocal` on your host, optional but convenient:
  ```bash
  pip install awscli-local
  ```
  (You can always call `aws --endpoint-url http://localhost:4566 ...`
  instead.)

**No Docker?** See `local-dev/README.md` for a Docker-free variant that
runs the same pipeline as native processes (Python + Mosquitto). Same topic
taxonomy and S3 layout — rover code doesn't change.

## Bring it up

```bash
cd cloud
scripts/setup.sh
```

That builds the rule-bridge image, starts all three containers, and waits
for LocalStack's init hooks to finish. When you see `fpms-cloud is up.`,
everything is ready.

## Try it end-to-end

```bash
# Publish a fake fire-detected event as if rover1 had spotted a hotspot:
scripts/publish-fire-event.sh

# Confirm the router archived it:
scripts/check-s3.sh rover1 fire-detected

# Watch the alert dispatcher fire:
scripts/tail-lambda-logs.sh fpms-alert-dispatcher
```

You should see a JSON file in `s3://fpms-archive/events/thing=rover1/…`
and an `ALERT [rover1] …` line in the dispatcher logs.

## Tear it down

```bash
scripts/teardown.sh          # stop containers, keep volumes
scripts/teardown.sh --wipe   # stop and delete all local data
```

## Layout

```
cloud/
├── README.md                 ← this file
├── .env.example
├── localstack/
│   ├── docker-compose.yml    ← LocalStack + Mosquitto + bridge
│   ├── mosquitto/mosquitto.conf
│   └── init/                 ← runs inside LocalStack on ready
│       ├── 01-s3-buckets.sh
│       ├── 02-sns-topics.sh
│       ├── 03-iam-role.sh
│       ├── 04-lambda-deploy.sh
│       └── 05-iot-things.sh
├── iot-core/
│   ├── topics.md             ← MQTT topic taxonomy — read this before adding events
│   ├── policies/             ← IoT policies (rover, station)
│   ├── register-thing.sh
│   └── rule-bridge/          ← Mosquitto → Lambda dispatcher
├── lambda/
│   ├── event_router/         ← IoT event → S3 archive + SNS fanout
│   ├── alert_dispatcher/     ← SNS → human-visible alert
│   └── package.sh            ← manual re-package (init also does this)
├── s3/bucket-layout.md       ← bucket + key conventions
└── scripts/                  ← operator scripts
    ├── setup.sh
    ├── teardown.sh
    ├── publish-fire-event.sh
    ├── publish-heritage-event.sh
    ├── check-s3.sh
    └── tail-lambda-logs.sh
```

## Rover-side code (sketch)

The rover connects to Mosquitto with a normal MQTT client and publishes to
`fpms/<thing>/events/<subtype>`:

```python
import json, uuid, time, paho.mqtt.client as mqtt

client = mqtt.Client(client_id="rover1")
client.connect("localhost", 1883)     # your Mosquitto host on the field WiFi

client.publish(
    "fpms/rover1/events/fire-detected",
    json.dumps({
        "event_id": str(uuid.uuid4()),
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "thing": "rover1",
        "severity": "high",
        "location": {"lat": 43.6532, "lon": -79.3832, "frame": "gps"},
    }),
    qos=1,
)
```

Nothing else changes when this moves to real AWS IoT Core — you just swap
in the AWS endpoint and X.509 certs.

## Honest limits

- **IoT Rules SQL** — real AWS IoT Rules can filter and transform in SQL.
  The bridge here does a plain topic-filter match. If you need SQL-like
  filtering, either extend `bridge.py` or upgrade to LocalStack Pro.
- **Device Shadows** — not implemented. Add if/when we actually need
  desired-state sync.
- **X.509 certs** — Mosquitto is anonymous locally. Real AWS enforces per-
  thing certs; the policies under `iot-core/policies/` are written to be
  compatible with that model when we get there.
- **SNS delivery** — SMS/email don't fire from LocalStack. The
  `alert_dispatcher` Lambda just logs. Wire real channels there when the
  dashboard goes live.

## Going to real AWS

The migration is mostly `s/localhost:4566/<real endpoint>/`:

1. Create the same buckets, role, SNS topic, and IoT things in your AWS
   account (Terraform, CDK, or the same `awslocal` scripts pointed at
   real AWS).
2. Deploy the Lambda zips from `cloud/lambda/build/`.
3. Replace `iot-rule-bridge` with an AWS IoT Rule: SQL
   `SELECT * FROM 'fpms/+/events/#'` → Lambda `fpms-event-router`.
4. On the rover, swap the broker from Mosquitto to AWS IoT Core endpoint
   and add the per-thing X.509 cert. Topics and payloads stay identical.

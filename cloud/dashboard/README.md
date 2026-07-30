# FPMS Robotics Dashboard — local LAN app

A multi-page dashboard that anyone on your WiFi can open in a browser to see
what the two Orange Pi 5B rovers are doing right now — **without anything
running on a remote server**. All AWS services run locally via LocalStack;
device MQTT lands on the same Mosquitto broker that stands in for the AWS
IoT Core MQTT endpoint in this repo.

```
Orange Pi 5B rover ──MQTT──▶ AWS IoT Core (LocalStack + Mosquitto)
                                     │
                                     ├──▶ IoT Rule → Lambda → S3 archive
                                     │        (existing cloud/ pipeline)
                                     │
                                     └──▶ FastAPI backend ──WebSocket──▶ React SPA
                                                                       (this dashboard)
```

**Nothing runs on a remote server.** LocalStack + Mosquitto + this backend
all live on your laptop. Any device on the same WiFi opens
`http://<your-laptop-ip>:5173` and joins the live view.

## Pages

| Page                | What it shows                                                                 |
|---------------------|--------------------------------------------------------------------------------|
| **Introduction**    | Mission, team, hardware, architecture, recent events pulled from S3 (LocalStack) |
| **LiDAR**           | Two live 360° polar grids — one for `rover1`, one for `rover2` — with per-rover Connect / Disconnect controls that publish through AWS IoT Data plane |
| **YOLO Camera**     | Live YOLO camera stream from `rover1` with detection overlays and a snapshot button |
| **Thermal + Analyst** | Live 32×24 LWIR heatmap from `rover1` plus a mini on-laptop analyst (stats, hotspot connected-components, 30-second trend, plain-English report, severity chip) |

## Prerequisites

- Docker + `docker compose`
- Python 3.11+
- Node.js 18+ (for the Vite frontend)
- macOS / Linux shell, or Git Bash / WSL on Windows

## One-time: bring up the local AWS stack

From the repo root:

```bash
cd cloud
scripts/setup.sh
```

That starts LocalStack (S3, Lambda, SNS, IAM, IoT registry, `iot-data`) on
:4566 and Mosquitto (the IoT Core MQTT broker) on :1883. See
`cloud/README.md` for what's inside.

## Start the dashboard

Two terminals from `cloud/dashboard/`:

```bash
# terminal 1 — FastAPI backend on :8000
scripts/start-backend.sh

# terminal 2 — Vite dev server on :5173 (bound to 0.0.0.0)
scripts/start-frontend.sh
```

Open on any device on your WiFi:

```
http://<your-laptop-ip>:5173
```

Find your laptop's IP with `ipconfig` (Windows), `ifconfig` / `ip a` (Linux),
or `System Settings → Wi-Fi → Details` (macOS).

## What the rovers need to publish

Real Orange Pi 5B code publishes to the same MQTT topics the dashboard
already subscribes to. When you swap this from LocalStack to real AWS, the
only change is the endpoint URL — topics and payloads stay identical.

### Topics

| Topic                                    | Rate       | Consumed by            |
|------------------------------------------|------------|-------------------------|
| `fpms/rover1/telemetry/lidar`            | up to 5 Hz | LiDAR page (rover 1)    |
| `fpms/rover2/telemetry/lidar`            | up to 5 Hz | LiDAR page (rover 2)    |
| `fpms/rover1/telemetry/camera`           | up to 5 Hz | YOLO Camera page        |
| `fpms/rover1/telemetry/thermal`          | up to 4 Hz | Thermal page + analyst  |
| `fpms/<thing>/telemetry/pose`            | 2 Hz       | LiDAR page (heading, battery) |
| `fpms/<thing>/events/#`                  | on event   | Intro "recent events" (via S3) |
| `fpms/<thing>/commands/connect` \| `disconnect` | on button click | rover-side subscriber |

The dashboard publishes `commands/*` through AWS IoT Core using the
`iot-data` API (boto3 → LocalStack) — the exact same call an AWS-hosted
app would make against production IoT Core. Real rover code should
subscribe to `fpms/+/commands/#` and start / stop its streams accordingly.

### Payload shapes

**LiDAR** — `fpms/<thing>/telemetry/lidar`
```json
{
  "ts": 1721777777.123,
  "thing": "rover1",
  "angle_min_deg": 0,
  "angle_max_deg": 360,
  "angle_step_deg": 1,
  "range_min_m": 0.15,
  "range_max_m": 6.0,
  "ranges_m": [ 1.23, 1.24, ... 360 values ... ],
  "heading_deg": 87.3
}
```

**YOLO camera** — `fpms/rover1/telemetry/camera`
```json
{
  "ts": 1721777777.123,
  "thing": "rover1",
  "format": "jpeg",
  "encoding": "base64",
  "width": 640,
  "height": 480,
  "frame": "<base64-jpeg>",
  "detections": [
    { "cls": "fire", "conf": 0.87, "box": [x1, y1, x2, y2] }
  ]
}
```

**Thermal** — `fpms/rover1/telemetry/thermal`
```json
{
  "ts": 1721777777.123,
  "thing": "rover1",
  "unit": "celsius",
  "rows": 24,
  "cols": 32,
  "grid": [[22.1, 22.4, ...], ...]
}
```

**Pose** — `fpms/<thing>/telemetry/pose`
```json
{ "ts": 1721777777.123, "thing": "rover1",
  "x_m": 0.42, "y_m": -0.13, "heading_deg": 87.3, "battery_pct": 78.5 }
```

### Minimal rover-side publisher (Python)

```python
import base64, io, json, time
import paho.mqtt.client as mqtt

client = mqtt.Client()
client.connect("<laptop-ip>", 1883)   # AWS IoT Core (LocalStack + Mosquitto)

# LiDAR at 5 Hz
while True:
    ranges = read_ldrobot_d500()      # list of 360 floats
    client.publish(
        "fpms/rover1/telemetry/lidar",
        json.dumps({
            "ts": time.time(),
            "thing": "rover1",
            "angle_min_deg": 0, "angle_max_deg": 360, "angle_step_deg": 1,
            "range_min_m": 0.15, "range_max_m": 6.0,
            "ranges_m": ranges,
            "heading_deg": current_heading_deg(),
        }),
        qos=0,
    )
    time.sleep(0.2)
```

Same shape, same broker, same topics — regardless of whether the broker is
LocalStack-hosted Mosquitto or production AWS IoT Core.

## Verification checklist

1. `curl http://localhost:8000/api/health` — expect
   `{"ok": true, "mqtt": {"connected": true}, "localstack": {"reachable": true}}`
2. Open `http://<laptop-ip>:5173` from another device on the same WiFi.
3. Intro page loads with team + architecture + hardware BOM.
4. Point a real rover's publisher at your laptop's IP on port 1883, or use
   the existing `cloud/scripts/publish-fire-event.sh` to push a canned event
   through the AWS IoT Core → Lambda → S3 pipeline (the event will appear on
   the Intro "Recent events" panel within a few seconds).
5. Once a rover starts publishing on `fpms/rover1/telemetry/lidar`, the LiDAR
   page's rover 1 panel flips to `live` and the point cloud appears.
6. Same for `camera` (YOLO page) and `thermal` (Thermal page). The Analyst
   panel starts writing plain-English reports within ~4 frames of thermal
   data arriving.

## Going to real AWS

Only the endpoint URLs change:

- Set `FPMS_MQTT_HOST` to your AWS IoT Core MQTT endpoint (and add X.509 certs)
- Unset `FPMS_AWS_ENDPOINT` so boto3 talks to real AWS
- Deploy the existing Lambda / S3 / SNS infra to your AWS account
- Point the frontend build at your production backend URL

Topics, payload shapes, and dashboard code are unchanged.

## Layout

```
cloud/dashboard/
├── README.md                ← this file
├── backend/                 ← FastAPI + MQTT bridge + thermal analyst
│   ├── main.py              ← REST + WebSocket routes; boto3 iot-data publish
│   ├── mqtt_bridge.py       ← subscribes to fpms/+/telemetry/# and fanout
│   ├── hub.py               ← per-channel WebSocket manager
│   ├── thermal_analysis.py  ← numpy stats + hotspot CC + trend + report
│   ├── config.py            ← env-driven settings
│   └── requirements.txt
├── frontend/                ← React + Vite + TypeScript + Tailwind SPA
│   ├── src/
│   │   ├── App.tsx, main.tsx, index.css
│   │   ├── lib/            (api.ts, ws.ts)
│   │   ├── components/     (Layout, NavBar, Card, StatusPill,
│   │   │                    LidarView, CameraView, ThermalView, AnalystPanel)
│   │   └── pages/          (Intro, Lidar, Camera, Thermal)
│   ├── package.json, vite.config.ts, tsconfig*.json,
│   ├── tailwind.config.js, postcss.config.js, index.html
│   └── public/favicon.svg
└── scripts/
    ├── start-backend.sh     ← venv + uvicorn on :8000
    └── start-frontend.sh    ← npm install + vite --host 0.0.0.0
```

## Honest limits

- **No simulator / no mock data.** The dashboard shows "waiting…" on each
  panel until a real rover publishes to the corresponding topic. This is
  intentional — the dashboard is the real thing, not a demo.
- **No auth.** Anyone on the LAN who has the URL sees everything. Matches
  "anyone can join the app". Add a reverse proxy with basic auth if you
  need to gate it.
- **LocalStack Community IoT.** The IoT registry, `iot-data` publish, and
  event archival all work; the actual MQTT broker duty is delegated to
  Mosquitto (LocalStack Pro is required for the built-in IoT Core MQTT
  broker). This is documented in `cloud/README.md` and matches the existing
  rule-bridge setup — dashboard code is unchanged when you move to real AWS.

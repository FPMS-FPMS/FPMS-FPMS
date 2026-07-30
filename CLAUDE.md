# FPMS — orientation

FPMS (Fire Prevention & Management System) is a wildfire-watch project: autonomous
rovers with RGB + thermal cameras patrol land (including cultural heritage sites),
detect fire and hazards on-device, and report to a cloud dashboard. Built by two
Grade 8 students for WRO Canada Nationals / International Finals.

This file is the durable layer — architecture and constraints that don't change
session to session. For **current state** (what's broken, what's pending right
now), read `cloud/HANDOFF.md` — don't re-derive it from here.

## Two-app + gateway architecture

- **Edge app** — `cloud/dashboard/` (Python/FastAPI backend + React/Vite frontend).
  Runs on the operator's Windows laptop. Full **control**: terminal, SSH, LAN scan,
  talks to the rover over MQTT. URL `http://127.0.0.1:8000`.
- **Cloud app** — `cloud/cloudapp/` (Cloudflare Worker, JS). **Streams + analyses**
  only — deliberately refuses hardware control (`FPMS_ROLE=cloud`). Archives
  telemetry to D1, runs a cron agent, calls a VLM for scene description, sends
  alert emails via Resend.
- **Gateway Worker** — `cloud/gateway/` — a stable public URL that proxies to
  whatever Cloudflare quick-tunnel the edge app currently has open (tunnel
  hostnames rotate every restart; the gateway hides that churn). Per HANDOFF.md;
  verify it exists before assuming its code is checked in.

## Directory map

```
cloud/
  dashboard/            edge app
    backend/            FastAPI: hub.py (MQTT hub, session state), thermal_analysis.py
    frontend/            React/Vite/Tailwind: CameraView, ThermalView, LidarView, AnalystPanel
    rover/               code that runs ON the Orange Pi (not the laptop)
      fpms_rover_agent.py     the agent (systemd fpms-rover-agent.service on the Pi)
      fpms_yolo26_npu.py      YOLO26 RKNN decode (NPU output -> boxes)
      test_yolo26_decode.py   22 offline checks for the decode, no Pi needed
      profile_steps.py, bench_npu.py   perf profiling scripts
    models/              yolo26n.pt/.onnx -> convert_yolo26.sh -> yolo26n-rk3588.rknn
  cloudapp/              cloud Worker: src/worker.js, agents.js, public.js; wrangler.jsonc, schema.sql (D1)
  gateway/               tunnel-proxy Worker
  lambda/, iot-core/, s3/, localstack/, deploy/   older AWS-path pieces, partly superseded by cloudapp
  FPMS-FPMS/             NESTED GIT CLONE — see Security below
  HANDOFF.md             volatile current-state doc, read every session
```

## Key commands

- Deploy cloud app: `cd cloud/cloudapp && npx wrangler deploy`
- Run cloud app locally: `cd cloud/cloudapp && npx wrangler dev --local`
- Run the rover YOLO26 decode test (no Pi/hardware needed):
  `python cloud/dashboard/rover/test_yolo26_decode.py`
- Build the dashboard frontend: `cd cloud/dashboard/frontend && npm run build`

## Hard-won constraints — do not re-litigate

- **Camera caps at 30 fps in hardware.** `v4l2-ctl --list-formats-ext` shows
  30.000 fps as the fastest interval at every resolution (1920x1080, 1280x720,
  640x480) — it's the sensor's USB descriptor. 60 fps needs different hardware
  (a 60 fps USB module or MIPI CSI camera), not a software fix.
- **Timestamps are unix seconds everywhere** (readings.ts, hub `last_message_at`,
  etc.). Conversion to ms happens explicitly and only at the browser boundary.
- **Rover preprocessing is `cv2.resize` to 640x640, a stretch, not a letterbox.**
  Box rescaling is therefore an independent per-axis (x, y) ratio, not a single
  uniform scale + padding offset. If preprocessing ever switches to letterbox,
  the decoder needs padding compensation added.
- **YOLO26 `output0` is `[1, 84, 8400]` with DFL already folded into the graph
  and classes already sigmoided — do not sigmoid again.** Despite being called
  "NMS-free" (a training-time property), the exported graph has no
  NonMaxSuppression/TopK op and still emits all 8400 anchors, so thresholding
  and a light NMS are still required downstream.
- **Alert emails must stay edge-triggered with a cooldown.** A past bug emailed
  330 obstacle events and exhausted the Resend daily quota, so a real fire alert
  could not send. Keep the rover-side trigger + cooldown gate in the Worker.
- **Fire detection uses asymmetric hysteresis** (fast-ish to alarm, much slower
  to clear) because a naive threshold flapped 173 times on one borderline frame.
- **VLM severity must come from a structured field, never keyword-matched from
  prose.** Prose matching once scored "there is no visible smoke" as critical.

## Security

This repo has real secrets sitting in plaintext files (untracked so far, NOT yet
committed — keep it that way until they are redacted and rotated):
- A Cloudflare ingest bearer token and the account ID are in `cloud/HANDOFF.md`.
- The shared MQTT/SSH password is hardcoded rather than read from the
  environment in `cloud/dashboard/rover/profile_steps.py` and
  `cloud/dashboard/rover/bench_npu.py`.
- A second demo password is in `cloud/.claude/settings.local.json`.

Refer to these by location, never by value — do not copy the literal secrets
into new files, commit messages, or docs (including this one).

Redact these before any commit, push, or sharing of file contents outside this
environment. Also: `cloud/FPMS-FPMS/` is a **nested git clone** (has its own
`.git`) — a plain `git add -A` from the repo root would stage it as an
accidental nested repo/submodule. Check `git status` for it before broad adds.

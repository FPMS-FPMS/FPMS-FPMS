# FPMS — session handoff

Read this first in a new session. It is the current state of both apps, the
rover, what is verified, and what is pending. Written 2026-07-29 (updated 2026-07-29: YOLO26 decode landed, rover files rescued into rover/).

---

## The system: two separate apps

| | **Edge app** | **Cloud app** |
|---|---|---|
| Runs on | this laptop | Cloudflare (Workers) |
| Source | `cloud/dashboard/` | `cloud/cloudapp/` |
| Install | `FPMS-Dashboard-Setup.exe` | `npx wrangler deploy` |
| Purpose | **control** rovers | **stream + analyse** |
| Terminal / SSH / LAN scan | yes, unrestricted | refused always (`FPMS_ROLE=cloud`) |
| URL | `http://127.0.0.1:8000` | `https://fpms-cloud.aryan0419wadhawan.workers.dev` |

Password for both: the User env var `FPMS_PASSWORD` (value not recorded here).

There is also a **gateway Worker** (`cloud/gateway/`) at
`https://fpms.aryan0419wadhawan.workers.dev` — a permanent URL that proxies to
whatever Cloudflare quick tunnel the edge app currently has. Quick-tunnel
hostnames rotate on every restart; the gateway hides that.

---

## Rover 2 (Orange Pi 5B) — CURRENTLY OFFLINE

**Last known:** `192.168.137.73`, hostname `fpms-pi`, user `ubuntu` (password not
recorded here — see the credentials table at the end). It sits on the **laptop's
Mobile Hotspot** subnet (192.168.137.x), NOT the Wi-Fi LAN (192.168.0.x).

It dropped off mid-session: 25/25 ping loss, SSH timeouts, zero MQTT messages.
Hotspot adapter `Local Area Connection* 2` shows LinkSpeed 0 bps.

**Re-checked 2026-07-29 — still offline, and the laptop side is not at fault:**

| check | result |
|---|---|
| `Test-NetConnection .73 -Port 22` | ping fail, TCP fail |
| Hotspot IP `192.168.137.1` | present on `Local Area Connection* 2` |
| Hotspot LinkSpeed | **0 bps — no client associated** |
| mosquitto (`:1883`) | listening, pid 18884 |
| Edge app (`:8000`) | listening, pid 24212 + cloudflared up |
| ARP for `.73` | entry exists but is state **Permanent** (a static entry, not a live neighbour) — do not read it as "the Pi is there" |

So the hotspot, broker and edge app are all healthy; the Pi simply is not
joining. That points at the board itself — power, or it failed to boot.
**This needs someone physically at the rover**; it cannot be fixed from here.

**To bring back:** the hotspot no longer needs re-enabling (confirmed up above) —
what is left is to **check the Pi has power and actually boots**. The agent is
`Restart=always` + enabled at boot, so once the board is up it rejoins by itself.

**Worth checking when it returns:** `journalctl -b -1 -n 50` — did it reboot?
The camera has already thrown USB `error -71` (a power/protocol fault), so a
marginal supply could explain both the camera dropping off the bus and the whole
board vanishing.

### What is installed on the Pi

- `/usr/local/bin/fpms-rover-agent` — the agent (systemd: `fpms-rover-agent.service`)
- `/etc/fpms/config.env` — chmod 600, holds broker credentials
- Reuses `/home/ubuntu/yolo/fpms_yolo_npu.py` for the YOLOv8 decode path
- `/home/ubuntu/yolo/yolov8n.rknn` — current model
- `/home/ubuntu/yolo/custom_labels.json` — `{"0": "Aryan"}` (class 0 = COCO person)

Config values that matter:
```
FPMS_CAMERA_FPS=30      # camera hardware max; see below
FPMS_INFER_EVERY=2      # YOLO on every 2nd frame, boxes carried over
FPMS_FIRE_EVERY=2       # fire screen cadence
FPMS_FIRE_MIN_RATIO=0.025
FPMS_FIRE_MIN_AREA=2500
FPMS_MQTT_HOST=192.168.137.1
```

### Measured performance (real numbers, not estimates)

Per-frame on the RK3588, measured:
```
capture      1.4 ms
preprocess   1.6 ms
NPU infer   20.1 ms
decode+NMS  10.8 ms
jpeg enc     1.6 ms
TOTAL       35.5 ms  -> 28.1 fps ceiling
```
Delivered: **2 fps → 23.6 fps** over the session.

**60 fps is impossible with this camera.** `v4l2-ctl --list-formats-ext` shows
30.000 fps as the fastest interval at *every* resolution (1920x1080, 1280x720,
640x480). It is the HBV sensor's USB descriptor. 60 fps needs different
hardware — a 60 fps USB module or a MIPI CSI camera on the ribbon connector.

---

## PENDING WORK (staged, blocked on the Pi returning)

### 1. Deploy YOLO26 — converted, decode written+tested, not yet on the Pi
`cloud/dashboard/models/yolo26n_rknn_model/yolo26n-rk3588.rknn` (7,445,558 bytes).

Conversion succeeded in Docker after pinning **`onnx==1.14.1`** and **`numpy<2`**
— `rknn-toolkit2` calls `onnx.mapping`, removed in onnx ≥1.16. The converter is
x86-only; the Pi has `rknn-toolkit-lite2`, which can only *run* models.
Script: `cloud/dashboard/models/convert_yolo26.sh`.

**The decode is done: `cloud/dashboard/rover/fpms_yolo26_npu.py`**, with
`rover/test_yolo26_decode.py` (22 checks, all passing offline — no Pi needed).
The agent selects it with `FPMS_YOLO_VARIANT=v26`; default stays `v8`.

#### Correction to the previous handoff
The earlier note said "YOLO26 is NMS-free, so a new decode path is required"
and implied the 10.8 ms decode step disappears. Reading `models/yolo26n.onnx`
directly (opset 19) shows what the export really emits:

```
input   images  : [1, 3, 640, 640]
output  output0 : [1, 84, 8400]        # 4 box + 80 class, one tensor
```

- The **DFL reconstruction is already folded into the graph** (two `Softmax`
  ops in `model.23`). Channels 0..3 arrive as xywh *centre* form already
  multiplied by the per-anchor stride constant (`[8]*6400 + [16]*1600 +
  [32]*400`), i.e. in 640-pixel units. Channels 4..83 are already through
  `Sigmoid` — **do not sigmoid them again**.
- The graph contains **no `NonMaxSuppression` and no `TopK`** (verified by op
  histogram). "NMS-free" describes how YOLO26 was *trained*, not the tensor you
  get back: with `end2end=false` the head still emits all 8400 anchors, so you
  must still threshold and still want a light NMS. The decoder keeps one.
- Layout is therefore **the same as YOLOv8's `output0`**. The reason
  `fpms_yolo_npu.decode()` can't be reused is not the shape — it is that v8's
  decode does its own DFL+sigmoid, which would double-apply here.

Because the agent preprocesses with a plain `cv2.resize(frame, (640,640))`
**stretch, not a letterbox**, rescaling is an independent per-axis ratio. If the
preprocess ever changes to letterbox, the decoder needs padding compensation.

Revised expectation: CPU decode shrinks but does **not** vanish — argmax over
8400x80 still runs on the CPU each inferred frame — and folding DFL into the
graph likely costs a little more NPU time. Net gain is probably well under the
10.8 ms previously assumed, and delivered fps stays capped by the 30 fps camera.
Benchmark before believing any number.

Next on the Pi: `scp` the .rknn to `/home/ubuntu/yolo/`, drop
`fpms_yolo26_npu.py` beside it, set `FPMS_YOLO_VARIANT=v26`, and compare
inference time against v8.

### 2. Deploy the optimised agent
**Now at `cloud/dashboard/rover/fpms_rover_agent.py`** — it previously existed
only in a session Temp scratchpad and on the Pi, i.e. one temp cleanup away from
being lost, with no copy in the repo at all. Moved 2026-07-29 along with
`profile_steps.py` and `bench_npu.py`.

It has an unshipped change: `detect_fire()` now runs on a **quarter-scale copy**
and only every `FIRE_EVERY` frames. A subagent found it was the one heavy CV
step that `INFER_EVERY=2` did not halve.

~22 ms/frame is still unaccounted for (20.1 ms of measured steps vs 42.4 ms
actual). The profiler is at `rover/profile_steps.py` — it measures
`detect_fire`, base64, `json.dumps`, MQTT publish and overlay draw, publishing
to `profiling/scratch` so it will not pollute `fpms/#`. Not yet run.

Other suspects the subagent identified by reading the source:
- `json.dumps()` serialises the ~20 KB base64 frame inside `Bus.publish()`
- overlay draws up to **6** boxes (3 YOLO + 3 fire), re-drawn on carried-over frames

### 2b. Direct rover → cloud uplink — BUILT AND TESTED, needs deploying to the Pi

The laptop used to be a **mandatory relay**: rover → MQTT → laptop → HTTPS →
cloud. With the laptop off, the cloud app did not go blank, it went *stale and
silently lied* — `latest:<channel>` persists, so a frozen frame stayed on screen
behind a green "connected" pill. Both halves of that are now fixed.

`cloud/dashboard/rover/fpms_cloud_uplink.py` sends telemetry straight to the
Worker, alongside MQTT rather than instead of it. It is hooked into
`Bus.publish` **above** the `if not self.connected: return` guard — that
ordering is load-bearing, since hooking below it would mean the cloud goes dark
exactly when the local broker does.

Verified end to end on a laptop with no rover (`test_uplink_hook.py`,
`test_uplink_chain.py`): with the broker marked down, a frame and a fire event
still reached a real Worker over WebSocket.

**To turn it on, in `/etc/fpms/config.env`:**
```
FPMS_CLOUD_UPLINK=1
FPMS_INGEST_URL=https://fpms-cloud.aryan0419wadhawan.workers.dev/ingest
FPMS_INGEST_WS_URL=wss://fpms-cloud.aryan0419wadhawan.workers.dev/ingest/ws
FPMS_INGEST_TOKEN=<the same token the Worker holds as FPMS_INGEST_TOKEN>
FPMS_CLOUD_MODE=auto      # auto | ws | http | off
FPMS_CLOUD_FPS=5
```
Also `scp fpms_cloud_uplink.py` next to `fpms_rover_agent.py`, and
`pip install websocket-client` for the WebSocket path (without it, it falls back
to HTTPS keep-alive automatically — no failure, just slower).

**Default is OFF.** With `FPMS_CLOUD_UPLINK` unset the agent behaves exactly as
before, so merging this changed nothing until an operator opts in.

Do **not** raise `FPMS_CLOUD_FPS` much: at ~27 KB/frame, 10 fps is ~23 GB/day.
The data plan binds long before Cloudflare does.

### 3. Thermal sensor is not attached
`thermal: 0 readings, never`. This is the most important gap: the fire screen is
an HSV colour heuristic with nothing to cross-validate against, which is why it
produced false positives and why thresholds keep needing tuning.

---

## Verified working

- **Camera** — real JPEG (`ffd8ff`), 640x480, NPU active, YOLO boxes drawn on-device
- **LiDAR** — 2,400+ points/scan at 230400 baud, 360-bin `ranges_m`
- **Cloud app** — telemetry archived to D1, agents on a `*/15 * * * *` cron
- **VLM** — `@cf/llava-hf/llava-1.5-7b-hf` describing frames, ~4-5 s
- **Email** — Resend, verified delivery id `9a781dba-...`, to `aryan0419wadhawan@gmail.com`
- **Autostart** — scheduled task "FPMS HQ", S4U, boot + logon + 30 min
- **Session cookies** survive restarts (secret persisted in `~/.fpms/settings.json`)

---

## Bugs fixed this session (do not reintroduce)

1. **Session secret regenerated per process** — 30-day cookie died on every restart
2. **Overview blocked 8.5 s** on a boto3 `list_buckets()` while polled every 2.5 s
3. **AWS tab took 42 s** — five service probes in series against a dead endpoint
4. **Installed app could never start its tunnel** — `bin/` path off by one directory
5. **Service ran a different build than the window** — held the build-folder exe open
6. **Pages hardcoded to `rover1`** — Camera, Thermal, Analyst; fleet runs rover2
7. **LiDAR crashed the whole app** — `pose.data.data.x_m.toFixed()` on a rover with
   no odometry, no error boundary → white screen
8. **LiDAR payload contract** — canvas needs `ranges_m` (360 entries, index = degree)
   and `range_max_m`; agent was sending `sectors_mm`
9. **Camera never recovered from USB re-enumeration** — held `/dev/video0` forever
   after the device came back as `/dev/video1`
10. **Alert email flood** — 330 obstacle events each emailed, exhausting the Resend
    daily quota so a real fire alert could not send. Now: edge-triggered on the
    rover + 10-min cooldown gate in the Worker
11. **Fire detector flapping** — 173 `fire`/`fire_cleared` pairs from one borderline
    frame at 22 fps. Fixed with asymmetric hysteresis (5 frames on, 45 off)
12. **VLM false positives** — severity was keyword-matched from prose, so
    "there is no visible smoke" scored critical. Now the model must return
    `HAZARD: YES|NO` and severity comes from that
13. **SPA shell cached** — app kept showing an old build after updates; `index.html`
    now sent `no-store`

---

## Things I deliberately did NOT do

- **Did not accept Meta's licence** for `@cf/meta/llama-3.2-11b-vision-instruct`.
  Workers AI gates it behind submitting "agree". To enable the stronger model,
  the operator runs:
  `npx wrangler ai run @cf/meta/llama-3.2-11b-vision-instruct --prompt agree`
- **Did not run a VLM on the Pi.** 3.9 GB RAM, and it is busy doing 23 fps
  detection. A VLM there would take seconds per frame and destroy the loop.
- **Did not delete anything outside an explicit, listed scope.** Only 16
  `fpms_phase5.py.bak.*` files were removed, after listing them. All ROS
  workspaces and `yolo/` untouched.

---

## Credentials / endpoints

**No secret values live in this file.** They used to, and this file is now
committed, so they must not come back. Record *where* a credential is kept, never
what it is.

```
Edge app        http://127.0.0.1:8000            password: $env:FPMS_PASSWORD
Cloud app       https://fpms-cloud.aryan0419wadhawan.workers.dev
Gateway URL     https://fpms.aryan0419wadhawan.workers.dev
Pi              ssh ubuntu@192.168.137.73        password: same as FPMS_PASSWORD
MQTT broker     192.168.137.1:1883               user fpms / $FPMS_MQTT_PASS
Cloud ingest    <cloud>/ingest                   Bearer $FPMS_INGEST_TOKEN
Cloudflare acct see `npx wrangler whoami`
AWS             account exists, but NO CLI credentials — `aws login` never completed
```

Where each one actually lives:

| Secret | Stored in |
|---|---|
| `FPMS_PASSWORD` (dashboard login) | Windows user env var; Worker: `npx wrangler secret put FPMS_PASSWORD` |
| `FPMS_INGEST_TOKEN` (rover → cloud) | Worker secret + `/etc/fpms/config.env` on the Pi (chmod 600) |
| `FPMS_MQTT_PASS` | `/etc/fpms/config.env` on the Pi |
| Pi SSH password | not stored in this repo |
| Cloudflare account | wrangler OAuth token in `~/.wrangler/config/default.toml` |

**These values were in plaintext on disk before this commit and should be treated
as exposed.** Rotate the ingest token and the shared password:
```
npx wrangler secret put FPMS_INGEST_TOKEN   # then update /etc/fpms/config.env
npx wrangler secret put FPMS_PASSWORD
```

---

## First moves in a new session

1. Is the Pi back? `Test-NetConnection 192.168.137.73 -Port 22`
   (as of 2026-07-29 it is not — see the table above; the blocker is physical)
2. Is telemetry flowing? subscribe to `fpms/rover2/#` on 192.168.137.1:1883
3. If yes: deploy `rover/fpms_rover_agent.py`, then upload and benchmark
   `models/yolo26n_rknn_model/yolo26n-rk3588.rknn` with
   `rover/fpms_yolo26_npu.py` and `FPMS_YOLO_VARIANT=v26`
4. Run `rover/profile_steps.py` to find the missing ~22 ms/frame

Everything above needs the Pi. What does **not**: `rover/test_yolo26_decode.py`
runs anywhere with numpy and re-checks the decode contract in isolation.

### Still worth doing with the rover down
- `fpms_yolo_npu.py` (the v8 decode) exists **only on the Pi**. If that SD card
  dies, it is gone — there is no copy in this repo. Pull it the moment the board
  is reachable.
- The whole `cloud/` tree is still **untracked** in git (`git status` shows
  `?? cloud/`). Only 4 commits exist, none containing this work.

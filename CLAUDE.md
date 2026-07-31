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
    rover/               code that runs ON the Orange Pi / rover boards (not the laptop)
      fpms_rover_agent.py     the agent (systemd fpms-rover-agent.service on the Pi)
      fpms_yolo26_npu.py      YOLO26 RKNN decode (NPU output -> boxes)
      test_yolo26_decode.py   22 offline checks for the decode, no Pi needed
      profile_steps.py, bench_npu.py   perf profiling scripts
      fpms_teleop.py           MQTT -> /cmd_vel bridge to the drive board, 0.6s jog
                                deadman (systemd fpms-teleop.service; needs
                                micro-ros-agent.service running alongside it)
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

## Rover drive board — hard constraints (verified on real hardware)

The rover's drive/motor subsystem is a separate board from the Orange Pi's
camera/LiDAR stack. These facts cost real time to establish — do not
re-derive or re-litigate them.

- **It is a Yahboom MicroROS Board V2.0 (ESP32-S3)**, speaking micro-ROS /
  Micro XRCE-DDS — **not** the Rosmaster STM32 board the old GOLDEN code
  targets. `Rosmaster_Lib` over direct serial cannot talk to this board: it
  returns version=-1, battery 0.0V, encoders all zero. Seeing those zeros
  means you're on the wrong protocol, not looking at dead hardware.
- **Motor transport is micro-ROS over serial at 921600 baud**, on the stable
  path `/dev/serial/by-path/platform-fc880000.usb-usb-0:1.3:1.0-port0`.
- **Both onboard CP2102 USB-serial adapters report an identical
  `ID_SERIAL`.** Only the USB topology path tells them apart — the other one
  (`...usb-0:1.2:1.0-port0`, `ttyUSB0`) is the LiDAR, owned by
  fpms-rover-agent. **Never bind by `/dev/ttyUSBn`**: enumeration order is
  not guaranteed, and getting it wrong sends motor commands to the LiDAR
  port or vice versa.
- **The firmware stores its own `ROS_DOMAIN_ID = 20`**, independent of
  whatever the host defaults to. A session can establish and DDS entities
  can be created correctly while every `ros2` tool running on domain 0 sees
  an empty topic list — that looks exactly like a broken link and isn't
  one. Read the domain back over Yahboom's separate config protocol at
  115200 baud (frames `0xFF`/`0xF8` host->board, `0xF7` board->host,
  checksum = `sum % 256`; address `0x06` = domain id, `0x51` = firmware
  version). That link is independent of the 921600 micro-ROS transport.
- **The board self-reconnects unaided in 90-225s** after any agent
  restart. No reset-button press is needed or helpful for this — if the
  link looks dead right after a restart, wait before touching hardware.
- **systemd must use `After=` + `Restart=always` on the device unit, not
  `BindsTo=`.** `BindsTo=` plus udev churn kills the agent.
- **Topics on `ROS_DOMAIN_ID=20`, node `/YB_Car_Node`, all QoS
  RELIABLE/VOLATILE:**
  - `/cmd_vel` `geometry_msgs/msg/Twist` (sub)
  - `/battery` `std_msgs/msg/UInt16` @1Hz — **decivolts, divide by 10**
  - `/odom_raw` `nav_msgs/msg/Odometry` @11.2Hz
  - `/imu` `sensor_msgs/msg/Imu` @25Hz — **orientation is not fused
    (identity quaternion); integrate heading from `angular_velocity.z`**
  - `/scan` `sensor_msgs/msg/LaserScan` — **dead, all ranges 0.0, not a
    usable LiDAR source** (LiDAR data comes from the other CP2102, above)
  - `/beep`, `/servo_s1`, `/servo_s2`
  - Differential drive; `linear.y` is ignored. Firmware clamps `vx` to
    +/-1.0 m/s and `wz` to +/-5.0 rad/s.
- **`/odom_raw` reports `twist.linear.x` with an INVERTED SIGN relative to
  its own `pose.position`.** Measured directly, wheels off, with nothing
  else publishing `/cmd_vel`:

  | commanded | reported twist | pose displacement | actual |
  |---|---|---|---|
  | +0.012 | −0.842 | **+1.395** | forward |
  | +0.100 | −1.225 | **+3.505** | forward |
  | −0.012 | +0.574 | **−1.506** | backward |
  | 0 | 0 | 0 | agree |

  **Trust `pose`, not `twist`.** Any guard that compares a commanded sign
  against reported twist will abort every *correct* move — that is exactly
  what killed the first deadband sweep.

  This one inversion produced a whole cascade of wrong conclusions on
  2026-07-31, all of them now retracted: that the chassis "cannot creep",
  that a motor deadband made it stall-then-lurch, that a 290mm *backward*
  lurch had occurred (it was a 290mm *forward* move), and a
  do-not-re-litigate rule saying "raise the floor, never creep slower".
  **The drive is correct and proportional at every magnitude tested.**
  A real deadband may still exist at some low value, but it has never been
  measured — the sweep aborted on the inverted sign, not on a deadband.

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

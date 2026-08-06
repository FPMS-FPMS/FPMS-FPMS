# Dashboard → ROS-native port

Moving the dashboard's live data off MQTT and onto the ROS graph. Increment 1
landed 2026-08-06; it is additive, feature-flagged and **off by default**.

Read `TAB_AUDIT.md` first for what each tab renders and where it gets it.

---

## The architecture decision, and why it is not literally `rclpy`

The instruction was "a direct rclpy client". That is not buildable in this
backend, and the reason should not need rediscovering:

- The backend runs on the operator's **Windows laptop**. There is no ROS 2 on
  it — `import rclpy` fails.
- Its interpreter is **Python 3.14**. ROS 2 Humble (what the Pi runs) targets
  3.10, and there is no supported Humble build for 3.14 on Windows.

Installing ROS 2 on Windows would put a large, fragile dependency on the one
machine used on race day. So the rclpy process stays where ROS already is — on
the Pi — and the backend speaks to it over `rosbridge_server`, which is
**already running** as `fpms-rosbridge.service` on :9090.

What was actually wanted is satisfied: every value this path produces arrives as
a ROS message and MQTT is nowhere in it. It is the same architecture
`fpms_console` already uses ("nothing in the live path from Pi to laptop is
MQTT"), just terminating in Python instead of a browser.

**Cost: zero new dependencies.** `websockets` ships with `uvicorn[standard]`.

Files: `backend/ros_bridge.py` (new), `backend/mqtt_bridge.py` (one guard),
`backend/main.py` (startup + `/api/health.ros`),
`scripts/verify_ros_bridge.py` (new).

## Turning it on

```powershell
$env:FPMS_ROS_ENABLED = "1"      # default "0"
$env:FPMS_ROS_HOST    = "fpms-pi.local"
$env:FPMS_ROS_PORT    = "9090"
$env:FPMS_ROS_THING   = "rover2" # which dashboard thing the Pi's graph is
```

`GET /api/health` gains a `ros` block: `connected`, `subscribed`,
`messages_seen`, `claimed_channels` and a plain-English `problem`. Top-level
`ok` still keys on MQTT alone, so a green dashboard means what it always did.

Verify without the app:

```
python scripts/verify_ros_bridge.py           # offline mapping checks, no Pi
python scripts/verify_ros_bridge.py --live    # + real WebSocket to the Pi
```

18/18 passing as of 2026-08-06.

## What increment 1 actually gains — a real pose

Not just a different transport. `frontend/src/lib/arena.ts` says it outright:

> **THIS FLEET DOES NOT PUBLISH POSE.**

The rover agent's `telemetry/pose` is a heartbeat — `uptime_s`, `camera_ok`,
`lidar_ok`, `frames`, `scans`, `streaming` — with no coordinate in it. So over
MQTT the arena glyph is **always** simulated and the HUD always reads
POSE ASSUMED. The ROS graph does carry a real anchored position.

Proven end to end on 2026-08-06 against the live Pi: injected
`/fpms/mission/{x_mm,y_mm,heading_deg}` = 972 / 228 / 90 arrived as

```
channel pose:rover2
  x_m 0.972   y_m 0.228   heading_deg 90.0
  frame arena   source ros:/fpms/mission
```

which `readPose` draws at arena (972, 228) facing up the arena — the start box.
**No frontend change was needed**; the envelope shape already matches.

### Why the pose comes from `/fpms/mission/*` and NOT `/odom`

The most important decision in this port, and the easy one to get backwards.

`arena.ts` draws in ARENA coordinates (origin bottom-left, mm, 1200x1200, start
box at 972/228). `/odom` is **not in that frame** — its origin is wherever the
encoders were last zeroed. Feeding `/odom` to `readPose` yields plausible
coordinates that are wrong by however far the odom origin sits from the arena
origin: a stable, silent offset that looks exactly like a working map. That
confusion has already cost this project a session.

`fpms_missions.py` publishes `/fpms/mission/x_mm`, `/y_mm`, `/heading_deg`
already anchored in arena millimetres — it applies the teleop `set_coordinate`
origin itself. Those are the only values on the graph that mean what the map
means.

Consequence, and it is the honest one: they are published while a mission runs,
not on an idle heartbeat. **Parked, this bridge publishes no pose**, the glyph
falls back to the start box, and the HUD says ASSUMED. An assumed pose that says
so beats a confident pose in the wrong frame.

## Channel ownership — how the two bridges coexist

Both fan into the same `hub` channels. If both published `pose:<thing>`, the
glyph would alternate between the ROS pose (~5 Hz) and the coordinate-free
heartbeat (~0.2 Hz) and blink MEASURED/ASSUMED every five seconds.

So a channel has exactly one owner:

- `ros_bridge.claimed_channels()` reports what it is *currently* publishing.
- `mqtt_bridge._dispatch_telemetry` skips a claimed channel — and hands the
  payload over via `note_suppressed()` rather than dropping it, because the
  heartbeat is the only source of `lidar_health` / `camera_ok` / `uptime_s`.
  Those are merged back *underneath* the ROS values on every emit, so a takeover
  never costs a panel fields it had.
- The claim is **live-gated** (`_CLAIM_TTL_S = 6 s`). If the Pi link drops, the
  claim lapses and MQTT takes the panel back on its own. A bridge that is
  connected but silent must never hold a panel dark.
- Both interop helpers in `mqtt_bridge` swallow everything including
  `ImportError`, so a fault in the new path degrades to the old behaviour rather
  than a dark dashboard.

The cloud uplink is deliberately left running for suppressed messages — it
archives telemetry and has nothing to do with which source draws a panel.

## Next increments — decisions, not rediscovery

| Channel | ROS source | State |
|---|---|---|
| `pose:` | `/fpms/mission/{x_mm,y_mm,heading_deg}` | **DONE** |
| `events` | `/fpms/events` (`std_msgs/String` of JSON) | Nearly direct. Held back only so increment 1 changed one behaviour, not two. Do this next — it is the cheapest. |
| `drive:` | `/fpms_health` + `/battery` | `/fpms_health` is an `Int32MultiArray` whose meaning is the 12-field table in `fpms_main.cpp` (keep them in sync). Note the panel also reads teleop-only fields — measured topic rates, micro-ROS link state — that are **not on the ROS graph at all**, so this is a partial port, not a swap. |
| `lidar:` | `/scan_lidar` (`LaserScan`) | Needs care. The panel consumes the agent's shape (`ranges_m`, `health`, `hz`, `scan_age_s`, `seq`); `LaserScan` carries none of the health/staleness fields, which the agent derives from scanner timing. Doing this badly replaces a feed that is currently **honest about going stale** with one that is not. Its own increment. |
| `mission:` | `/fpms/mission/{state,phase,leg_i,segment_i,distance_*}` | Straightforward; several topics to assemble into one envelope, same collect-then-emit pattern as pose. |
| `mission_plan:` | `/fpms/plan/route` | Already mirrored into ROS by `fpms_console.PlanMirror`. |
| `camera:` / `thermal:` | — | **NOT PORTABLE.** No ROS publisher exists; the agent sends frames straight to MQTT. These need new rover-side publishers — a rover change, not a dashboard one. Do not plan the port as if all 11 tabs can move. |

## Operational note found while building this — rosbridge can wedge silently

On 2026-08-06 `fpms-rosbridge.service` was **accepting WebSocket connections,
logging "Subscribed to /odom", and forwarding nothing at all** — zero messages
on five different topics including a trivial `Int32MultiArray`. `systemctl
restart fpms-rosbridge` fixed it instantly (0 → 280 msgs in 18 s).

While wedged, the operator console on :8090 has no live data either, since every
value it renders comes through this service.

Root cause is **not established**. The obvious theory — that the mass service
restart in `deploy_stack.sh` orphaned rosbridge's DDS readers, since
`fpms-rosbridge`, `fpms-console` and the two foxglove units are absent from that
script's `UNITS` list — was tested and **disproved**: restarting `fpms-odom-tf`
mid-stream caused only a brief dip and full recovery.

What to remember is the symptom, because it is the worst kind: **connected,
subscribed, silent — no error anywhere.** If the console or this bridge shows a
healthy link and no data, restart `fpms-rosbridge` before believing anything
else.

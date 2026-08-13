# FPMS operator dashboard — ROS-native

A single-page operator view of the rover, fed **entirely by ROS topics over
`rosbridge_server`**. No MQTT in the live path, no build step, no npm, no CDN,
no framework. Four static files and a stdlib-only Python server.

It is **read-only except for STOP**. It cannot drive the rover, and that is
enforced twice over — see [What it deliberately cannot do](#what-it-deliberately-cannot-do).

Built alongside the existing `frontend/`, `backend/`, `fpms_console/`,
`foxglove/` and `rover/` trees. **Nothing outside this directory was touched.**

```
fpms-dashboard/
  index.html          structure
  assets/app.css      styling, including the four freshness states
  assets/rosbridge.js the connection layer — reconnect, heartbeat, silence
  assets/feeds.js     the freshness registry
  assets/arena.js     the arena map, zone geometry, pose rendering
  assets/app.js       subscriptions, STOP, panels, the 5 Hz ticker
  serve.py            a static server. Not in the data path.
  verify_link.js      optional: proves the reconnect logic (28 assertions)
  verify_render.js    optional: proves the staleness + silence contract (48)
```

---

## Identity — this image is ROVER 1

| | value | where it comes from |
|---|---|---|
| host | `fpms-rover1.local` | `fpms-os/config/fpms-os.conf` → `FPMS_HOSTNAME=fpms-rover1` |
| port | `9090` | `fpms-rosbridge.service` |
| thing name | `rover1` | `/etc/fpms/config.env` → `FPMS_THING_NAME=rover1` |

These are the built-in defaults. Opening the page with no configuration at all
dials `ws://fpms-rover1.local:9090` and stamps **`rover1`** in the header.

**The thing name is shown in the header at all times**, never behind a settings
panel, because a two-rover fleet is coming and "which robot am I looking at" is
not a detail.

### Why the thing name is here at all, given this is a ROS dashboard

Be precise about this, because getting it wrong wastes a session.

The ROS topic names this page subscribes to **contain no thing name**.
`/scan_lidar` and `/fpms/mission/x_mm` are the same strings on every rover. The
thing name is the **MQTT topic root**, and it reaches this dashboard in exactly
two ways:

1. **It is what the Pi-side bridge subscribes.** `fpms_foxglove_cmd` listens on
   `fpms/<thing>/telemetry/#` and mirrors what it hears onto `/fpms/*`. Point
   that bridge at the wrong root and the mirrors go silent — while the socket,
   the broker and every systemd unit stay healthy. `config.env` says it
   outright: a consumer on the wrong root *"connects, authenticates, stays
   connected and receives nothing forever."*
2. **It is checkable.** Every `DiagnosticStatus` the bridge publishes carries
   `hardware_id = FPMS_THING_NAME`. So `/diagnostics` is the rover's own
   statement of which rover it is, and the dashboard compares it against what
   the header says. Disagreement raises a full-width banner naming **both**
   sides. Agreement shows nothing.

### mDNS: the IP override is not optional

`change` → the picker has a separate **IP override** field that wins over the
hostname. Use it without embarrassment.

> mDNS resolution is **per-resolver, not per-machine**. The image's own
> `FLASHING.md` records a session where `fpms-rover1.local` resolved from .NET
> and failed from Python's `getaddrinfo` on the same box at the same time.

So "ping works" does not prove the browser will resolve it. Get the address
from your router or hotspot client list and paste it in.

Overrides, most explicit first:

```
?rover=192.168.1.42:9090&thing=rover1     one visit, beats everything
localStorage                              what you last chose in the picker
config.json                               what serve.py was started with
built-in                                  fpms-rover1.local:9090 / rover1
```

---

## Running it

### Ubuntu

```bash
cd cloud/dashboard/fpms-dashboard
python3 serve.py                       # http://localhost:8099/
```

Point it somewhere else without touching the page:

```bash
python3 serve.py --rover fpms-rover1.local --thing rover1
python3 serve.py --rover 192.168.1.42:9090          # when mDNS will not resolve
FPMS_ROVER_HOST=fpms-rover1.local python3 serve.py --port 8099
```

No `pip install`. No venv. Python 3 standard library only, deliberately: a
dashboard whose job is to be available must not need a network at start-up.

### WSL, reached from Windows

```bash
# inside WSL
cd /mnt/c/Users/<you>/fpms-os-wt/cloud/dashboard/fpms-dashboard
python3 serve.py
```

Then in a **Windows** browser open `http://localhost:8099/`. WSL2 forwards
localhost, and `serve.py` binds `0.0.0.0` by default so it works even on builds
that do not. If localhost does not work, the start-up banner prints the WSL
interface address — use that instead:

```
  http://localhost:8099/
  http://172.24.112.3:8099/            <- use this from Windows if localhost does not work
```

**The rosbridge connection does not go through WSL.** The browser runs on
Windows and dials the Pi directly, so WSL's NAT, port forwarding and DNS are
all irrelevant to whether live data flows. The only thing crossing the WSL
boundary is the HTML. That also means `serve.py` can be restarted, or killed
outright, without an already-open tab noticing.

### Firewall

Nothing to configure. The **Pi listens and the laptop dials out**, which is the
only direction that works: the operator's Windows machine has no inbound
firewall rule and no admin account to add one. Nothing on the rover ever opens
a connection towards the laptop.

### Without any server at all

`index.html` can be opened straight off disk (`file://`). `config.json` and
`zones.json` will 404, which is handled — the page falls back to the built-in
rover1 defaults and the built-in arena derivation. WebSockets are unaffected by
`file://`.

---

## The headline requirement: it must never lose the connection

Six mechanisms, each covering something the other five do not. All of them live
in `assets/rosbridge.js` and all of them are exercised by `verify_link.js`.

### 1. Reconnect forever, with exponential backoff and jitter

Base 300 ms → ×1.7 → capped at 10 s, with a 250 ms floor. **Half-jitter**
(`d/2 + random()·d/2`), not full jitter: full jitter can draw near-zero
repeatedly and turn a backoff into a hammer. Jitter matters here because a
single tab runs **two** sockets, and without it they would retry in lockstep
forever.

### 2. Backoff resets only after sustained health

The obvious version — reset the backoff in `onopen` — is wrong, and it is worth
naming because the existing console has it. A rosbridge that accepts a socket
and drops it 200 ms later then produces open/close/open/close at the base delay
for as long as you let it. Here the backoff resets only after the link has been
**open for 5 continuous seconds**. Health is seconds survived, not one callback.

### 3. A connect timeout

A WebSocket to a host that has gone away — Pi rebooted, wifi roamed, route
black-holed — sits in `readyState CONNECTING` with **no event at all** until the
OS TCP stack gives up, which can be 75 s or more. Nothing would be retried for
that whole time. Attempts are abandoned after 12 s and redialed.

### 4. A heartbeat that detects a *silently dead* socket

`readyState === OPEN` proves nothing. A NAT box that dropped the flow, a
suspended laptop, a wedged server — each leaves a socket that is OPEN, will
never deliver another byte, and may never fire `close`.

So: whenever the socket has been **quiet for 4 s**, a probe goes out and an
answer is required within 3 s. No answer → close it ourselves and reconnect.

**The probe is deliberately an unknown op**, `{op:"__fpms_ping__", id:…}`.
rosbridge's `Protocol.incoming()` pulls `id` off every inbound message before
dispatch and answers an unrecognised op with `{op:"status", level:"error",
id:…}`. That makes the probe:

* answered by a stock, unmodified `rosbridge_server`;
* dependent on **no topic and no service**, so neither `topics_glob` nor
  `services_glob` can swallow it;
* free of any effect on the ROS graph.

A `/rosapi/get_time` call would have looked tidier and would have been wrong:
`services_glob` is `"[/fpms/*]"`, and a heartbeat that a safety whitelist can
silently eat is a heartbeat that will report a healthy link as dead at a
competition.

Probes only fire when the link is **already quiet**, so under normal load
(10 Hz `/scan_lidar`) none is ever sent and the Pi's journal stays clean.

### 5. Automatic re-advertise and re-subscribe on every open

rosbridge keeps **no state** across a dropped socket. Anything not replayed is
gone silently — the subscription simply never delivers again and nothing errors.
Every `advertise` and every `subscribe` is recorded and replayed on every open,
reconnects included.

### 6. Per-topic silence detection — "connected, subscribed, silent"

The most important mechanism here, because **two independent faults produce a
dashboard that looks completely healthy and shows nothing**:

**(a) The rosbridge discovery bug.**

> rosbridge only delivers topics whose **publisher already existed when
> rosbridge started**. A publisher created afterwards is never discovered, and
> the client subscription asking for it is accepted and then silent forever.

The image now orders `fpms-rosbridge` `After=fpms-ros-publishers.target`, so
this should be rare — but a dashboard that connects *during* boot can still
attach before a publisher exists.

**(b) A wrong topic root**, as described under Identity above.

Neither raises an error anywhere. So each subscription may declare `expectS` —
the seconds of silence that are definitely abnormal *for that topic* — and the
supervisor reports **exactly which topics** have gone quiet. The banner then
names the specific check to run:

| observed | verdict shown |
|---|---|
| `/diagnostics` silent too | link or rosbridge itself. Check `ROS_DOMAIN_ID=20`, then `sudo systemctl restart fpms-rosbridge` |
| `/diagnostics` **arriving**, `/fpms/*` mirrors silent | **"CHECK THE THING NAME"** — the bridge is alive and its MQTT side is empty. Names both the dashboard's thing and the rover's reported one |
| only `/scan_lidar` silent | the LiDAR publisher, not the link. Check `fpms-lidar-ros` |

Remediation is bounded: resubscribe the silent topics → force one fresh socket
(this is the reconnect trigger the boot-ordering case asks for) → **stop**, and
hold the banner. Retrying past that point would be worse than useless: neither
cause is fixable from a browser, and each reconnect drops the topics that *are*
working. The ladder is **not** reset by the reconnect it triggers — otherwise a
wrong thing name produces an endless 25-second reconnect cycle that blanks the
map every time round.

Only **four** topics declare an `expectS`, and the narrowness is the point:

| topic | why it qualifies |
|---|---|
| `/scan_lidar` | the LiDAR node runs whether or not a mission does |
| `/diagnostics` | `fpms_foxglove_cmd`'s own 0.5 s timer — the strongest canary on the graph, and the same node that mirrors `/fpms/*` |
| `/fpms/mission/state` | `_mirror_mission` publishes it unconditionally on every inbound `telemetry/mission` |
| `/fpms/mission/phase` | likewise, and it defaults to `"idle"` rather than being omitted |

Everything else is left unflagged on purpose. `x_mm`/`y_mm`/`heading_deg` are
omitted by the bridge when the executor has no pose (the normal parked state);
`/fpms_health` and `/wheel_ticks` are **firmware v3 only** and legitimately
absent on the factory firmware this rover runs today; residuals, the plan and
events are event-driven. *A silence alarm that fires while the rover is parked
and healthy is an alarm the operator learns to ignore, and then it is worth
nothing on the day it is right.*

### Also

* **Browser lifecycle.** `online`, `focus`, `pageshow` and `visibilitychange`
  each force an immediate retry. Timers in a hidden tab are throttled to as
  little as once a minute, so a backgrounded dashboard would otherwise sit on a
  10 s backoff for far longer than 10 s.
* **A supervisor at 1 Hz** re-checks every invariant, including "we believe we
  are waiting but no retry is pending". Belt to every other brace: a dashboard
  that must never lose its link cannot depend on one timer.
* **`dispose()`** retires a link when the operator re-points at another rover.
  Without it the old pair keeps its supervisor and goes on dialing the old host
  forever, invisibly.

---

## Honest about staleness

**Every value's age is measured from the moment this browser received it. No
timestamp inside a payload is ever used for freshness.** Two reasons, both of
which have bitten this project:

* a stuck publisher keeps re-sending — sometimes re-stamping — a payload whose
  sensor is dead, so the payload's opinion of its own age is not evidence;
* the Pi's clock and the laptop's do not agree, are not synchronised at a venue
  with no NTP route, and the sign of the disagreement is unknown.

`performance.now()` is used rather than `Date.now()` because it is monotonic: an
NTP step on the operator's laptop cannot make a stale value look fresh.

**The 5 Hz ticker is the only renderer.** Message handlers call `Feeds.mark()`
and touch no DOM. Every visible value is re-derived from the registry on every
tick, so there is no code path that leaves a number on screen without
re-checking its age.

Four states, styled in exactly one place so a new panel cannot forget them:

| state | rendering |
|---|---|
| `ok` | normal |
| `warn` | value and age turn amber |
| `stale` | greyed, **struck through**, age tagged `STALE` |
| `never` | greyed, "never seen" |

Colour alone fails for a colour-blind operator and fails again on a sunlit
screen, hence the strike-through as well.

Thresholds are **per feed**, taken from the Pi's own `WATCH_ROS`/`WATCH_MQTT`
table so the dashboard and the rover's `/diagnostics` never disagree about the
same feed. Event-driven feeds are aged and greyed but not accused: a 40 s gap in
the residual stream is a rover standing still.

Consequences worth knowing:

* **A stale LiDAR frame is dropped, not held.** The scan array is emptied and
  the map draws nothing. "No dots" is the honest picture of a dead scanner —
  and much safer than a held frame, because all-zero ranges convert to `inf`
  ("no return"), which reads as a completely clear 360°.
* **The pose card is judged by its worst component.** A fresh `x` with a stale
  heading is not a fresh pose.
* **Battery falls back** from `/battery` to `/fpms/mission/batt_v` only while
  `/battery` is absent *or stale* — not merely never-seen, because "we saw a
  voltage once, an hour ago" is exactly the frozen-but-plausible reading this
  dashboard exists to stop showing. The panel always names which source it is
  displaying.

---

## The arena map

1.2 m × 1.2 m, **origin bottom-left**, arena millimetres. **Map metres = arena
mm ÷ 1000**, same origin corner — so nav2 `(0.228, 0.972)` *is* arena
`(228, 972)` and no conversion exists anywhere.

### Pose comes from `/fpms/mission/*`, never `/odom`

`/odom`'s origin is wherever the encoders were last zeroed. Drawing it here
produces coordinates that are plausible, stable, smooth, and wrong by however
far the odom origin sits from the arena origin — a silent constant offset that
looks exactly like a working map. That has already cost this project a session.
`/odom_raw` is not fed to the map at all.

The honest consequence: `/fpms/mission/*` is published while a mission runs, not
on an idle heartbeat. **Parked, there is no live pose.** The map then draws a
hollow dashed ghost at the start box labelled **ASSUMED**, versus a solid cyan
body labelled **MEASURED** with the coordinates. There is no third rendering and
the difference is visible across a room.

LiDAR returns are only drawn when the pose is MEASURED — placing real obstacle
geometry using an assumed pose would put it in a made-up place.

### Zone geometry is derived, then checked — never copied

`/etc/fpms/zones.json` ships the three zone centres and states its own contract:

> A consumer that reads `cx_mm`/`cy_mm` and uses them directly has created the
> fifth independent copy of these numbers […] A consumer must recompute the
> centres from `arena_mm`, `zone_side_frac` and `zone_margin_frac` and **REFUSE**
> this whole file if any published centre disagrees by more than 0.05 mm.

So `arena.js` derives everything from three numbers (`1200`, `0.30`, `0.04`) and
`ARENA.validateZones()` implements the refusal. `serve.py` serves a copy of
`zones.json` if it finds one (`/etc/fpms/zones.json`, or the overlay in this
repo, or `FPMS_ZONES_FILE`); the result of the cross-check appears in the LINK
DIAGNOSTICS panel. A **missing file does nothing at all** — built-in
derivation, no note, no fault — which is what `zones.json` itself specifies.

Verified against the image: all four centres agree to within 0.05 mm.

| id | corner | mission | centre (mm) | map (m) |
|---|---|---|---|---|
| `zone-a` | TOP-LEFT | `m1` | 228, 972 | 0.228, 0.972 |
| `zone-b` | TOP-RIGHT | `m2` | 972, 972 | 0.972, 0.972 |
| `water-station` | BOTTOM-LEFT | `water` | 228, 228 | 0.228, 0.228 |
| start box | BOTTOM-RIGHT | `home` | 972, 228 @ 90° | 0.972, 0.228 |

**Every label on the map names the CORNER**, with the mission id in brackets.
That is deliberate, and `zones.json` explains why: standing behind the rover at
the start box, the zone *straight ahead* (top-right) is mission `m2`, and the
*far* one (top-left) is `m1` — so an operator's spoken "Zone 1" is the code's
`m2`. Nothing is renumbered, because renaming ids would silently change what
every stored command and log line means. A corner is the one description that
cannot be read two ways.

---

## STOP

* **Its own WebSocket, which subscribes to nothing.** A browser `send()` queues
  behind whatever is already in that socket's send buffer, and the main socket
  carries a 10 Hz `LaserScan` plus an event stream measured at ~330 Hz during a
  plan. A STOP on that socket can sit behind a scan frame. This one's buffer is
  empty by construction, and it is dialed **first** so it wins the race to the
  server on a cold start.
* **A topic publish, never a service call.** No request id, no pending state, no
  server needed to answer, no timeout to expire.
* **Both verbs.** `/estop` (`std_msgs/Bool` true), honoured by the firmware-v3
  control task and by `fpms-cored`; and `/fpms/cmd/stop` (`std_msgs/Empty`),
  which the Pi-side bridge fans out to the MQTT `stop` **and** `estop` verbs at
  QoS 1.
* **Falls back to the main socket** if the dedicated one is down, and says which
  path carried it. If both fail it says `PUBLISH FAILED ON BOTH SOCKETS — GET TO
  THE ROVER` and turns the STOP chip red.
* **Spacebar fires it** from anywhere except a text field. No modifier, no
  confirmation — a confirmation dialog is a second thing that can be busy.
* **Never disabled**, not even when the link is down. A greyed-out panic button
  tells the operator nothing; the fallback path and the failure message tell
  them everything.

---

## Topics — and whether each is on the whitelist

The whitelist is `topics_glob` in
`rover/fpms-os/overlay/etc/fpms/rosbridge_params.yaml`:

```
[/estop, /fpms/*, /clicked_point, /scan_lidar, /odom_raw, /odom, /imu,
 /battery, /wheel_ticks, /wheel_duty, /fpms_health, /diagnostics, /rosout]
```

`fnmatch` semantics: `*` matches `/` too, so `/fpms/*` covers `/fpms/cmd/stop`
and `/fpms/mission/x_mm` alike. **It was not widened.** `verify_render.js`
asserts every requested topic against this list, so the check runs rather than
being claimed.

### Subscribed (28)

| topic | type | on whitelist | used for |
|---|---|---|---|
| `/scan_lidar` | `sensor_msgs/LaserScan` | **yes** (by name) | map returns, LiDAR link tile. `expectS 8` |
| `/battery` | *(untyped — resolved by rosbridge)* | **yes** (by name) | battery panel |
| `/wheel_ticks` | `std_msgs/Int32MultiArray` | **yes** (by name) | ESP32 tile corroboration |
| `/fpms_health` | `std_msgs/Int32MultiArray` | **yes** (by name) | ESP32 tile: fw build, agent-connected bit, loop Hz, uptime, heap, estop latch, IMU |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | **yes** (by name) | Pi-side freshness second opinion, **and `hardware_id` = the identity check**. `expectS 8` |
| `/fpms/mission/x_mm` | `std_msgs/Float32` | **yes** (`/fpms/*`) | **pose x** |
| `/fpms/mission/y_mm` | `std_msgs/Float32` | **yes** (`/fpms/*`) | **pose y** |
| `/fpms/mission/heading_deg` | `std_msgs/Float32` | **yes** (`/fpms/*`) | **pose heading** |
| `/fpms/mission/state` | `std_msgs/String` | **yes** (`/fpms/*`) | mission state (whole JSON payload). `expectS 25` |
| `/fpms/mission/phase` | `std_msgs/String` | **yes** (`/fpms/*`) | phase. `expectS 25` |
| `/fpms/mission/armed` | `std_msgs/Bool` | **yes** (`/fpms/*`) | armed indicator |
| `/fpms/mission/link_ok` | `std_msgs/Bool` | **yes** (`/fpms/*`) | ESP32 tile |
| `/fpms/mission/lidar_ok` | `std_msgs/Bool` | **yes** (`/fpms/*`) | LiDAR tile |
| `/fpms/mission/leg_i` | `std_msgs/Int32` | **yes** (`/fpms/*`) | leg counter |
| `/fpms/mission/segment_i` | `std_msgs/Int32` | **yes** (`/fpms/*`) | segment counter |
| `/fpms/mission/distance_remaining_mm` | `std_msgs/Float32` | **yes** (`/fpms/*`) | mission panel |
| `/fpms/mission/distance_travelled_mm` | `std_msgs/Float32` | **yes** (`/fpms/*`) | mission panel |
| `/fpms/mission/front_mm` | `std_msgs/Float32` | **yes** (`/fpms/*`) | front-clearance arc |
| `/fpms/mission/batt_v` | `std_msgs/Float32` | **yes** (`/fpms/*`) | battery fallback |
| `/fpms/residual/drive_mm` | `std_msgs/Float32` | **yes** (`/fpms/*`) | residual panel |
| `/fpms/residual/turn_deg` | `std_msgs/Float32` | **yes** (`/fpms/*`) | residual panel |
| `/fpms/residual/lateral_mm` | `std_msgs/Float32` | **yes** (`/fpms/*`) | residual panel |
| `/fpms/residual/cumulative_drive_mm` | `std_msgs/Float32` | **yes** (`/fpms/*`) | residual panel |
| `/fpms/residual/cumulative_turn_deg` | `std_msgs/Float32` | **yes** (`/fpms/*`) | residual panel |
| `/fpms/residual/raw` | `std_msgs/String` | **yes** (`/fpms/*`) | the per-segment table and the diagnostic verdict |
| `/fpms/plan/route` | `std_msgs/String` | **yes** (`/fpms/*`) | planned route on the map |
| `/fpms/events` | `std_msgs/String` | **yes** (`/fpms/*`) | event log (throttled to 10 Hz) |

### Advertised and published (2)

| topic | type | on whitelist | used for |
|---|---|---|---|
| `/estop` | `std_msgs/Bool` | **yes** (by name) | STOP |
| `/fpms/cmd/stop` | `std_msgs/Empty` | **yes** (`/fpms/*`) | STOP |

### Services called: none

`services_glob` is `"[/fpms/*]"` and this dashboard **calls nothing**, not even
`/rosapi/*`. That is why the heartbeat is an unknown-op echo: it keeps the
liveness proof independent of any whitelist.

### Not requested, and cannot be

`/cmd_vel`, `/cmd_duty`, `/cmd_enable` are **absent from `topics_glob` by
construction**, so `rosbridge_server` refuses both `advertise` and `publish` for
them in three separate places (`subscribe.py`, `advertise.py`, `publish.py`).
This page also never asks. Two independent layers, neither relying on the other
being careful.

### Nothing was needed that is not on the list

One consequence to flag rather than fix: **camera and NPU health have no ROS
publisher at all.** The rover agent sends camera frames straight to MQTT and
`FPMS_NPU_REQUIRED=1` raises its fault on MQTT too — neither has a mirror inside
`topics_glob`. Those two tiles therefore read **NO ROS SOURCE**, hatched grey,
with the reason attached. They are not coloured good or bad, because the
dashboard has no evidence either way and guessing green would be the worst
failure it could have: *a blind fire-detection rover reported healthy.*

Reaching them needs a **rover-side publisher**, not a dashboard change and not a
wider whitelist. Flagging it here as requested; nothing has been widened.

---

## What each panel shows

* **Arena map** — 1.2 m × 1.2 m, bottom-left origin, 100 mm grid, arena
  boundary, an explicit `0,0` origin marker, the three zones labelled by corner,
  the start box, the planned route, a trail of measured poses, LiDAR returns,
  the 400 mm front-stop arc and the live front clearance, and the rover as
  MEASURED or ASSUMED.
* **Pose / LiDAR / front clearance** — value, age, and the frame warning.
* **Mission** — state, phase, armed, leg/segment, travelled, remaining, route.
* **Battery** — volts, source, age, and a bar (never a percentage: that would
  imply a state-of-charge model nobody has calibrated).
* **Link health** — LIDAR, ESP32, CAMERA, NPU. See above for why two of them
  are permanently "no ROS source".
* **Residual** — `drive_mm`, `turn_deg`, both cumulatives, a table of the last
  12 settled segments with target/measured/ratio from `/fpms/residual/raw`, and
  a **verdict** applying `STACK.md`'s table:

  | pattern | cause | knob |
  |---|---|---|
  | constant **ratio** | scale error, counts/mm | `odom_scale` (2.467 = 74/30 is the known candidate, and it is named when seen) |
  | constant **offset** | coast after the burst is cut | `coast_mm` |
  | grows only on **turns** | gyro / heading | `gyro_scale` |

  It refuses to guess from fewer than three drive segments and says so.
* **Events** — `/fpms/events`, throttled to 10 Hz *deliberately*. rosbridge
  serialises every subscription onto one thread and one socket; unthrottled, an
  event storm starves `/scan_lidar` on the same socket, and a frozen map beside
  a scrolling log is the exact failure this dashboard exists to prevent.
* **Link diagnostics** — phases, uptime, attempts, frames, topic messages,
  opens, closes, probes and failures, wedges; the dashboard's thing name against
  the rover's reported one; and the `zones.json` cross-check result.

---

## What it deliberately cannot do

* **It cannot drive the rover.** No `/cmd_vel`, no `/cmd_duty`, no
  `/cmd_enable` — not merely unused, but absent from the whitelist, so the
  server refuses them regardless of what this page asks.
* **It cannot plan, arm, run or reset pose.** It calls no ROS service at all.
  Those buttons live in `fpms_console`; this is a monitoring dashboard with a
  panic button.
* **It cannot change a ROS parameter.** `params_glob` is `"[]"`.
* **It cannot show camera or NPU health**, because nothing publishes them onto
  the ROS graph. It says so rather than implying otherwise.
* **It cannot show a pose while the rover is parked**, because the mission
  mirror does not publish one. It draws ASSUMED rather than guessing.
* **It cannot fix a wedged rosbridge.** It detects it, names it, and tells you
  to restart the service on the Pi.

---

## Verifying it

Optional, and **not** a build step — nothing here is loaded by the page. Node is
not required to run the dashboard.

```bash
node verify_link.js     # 28 assertions: reconnect, heartbeat, silence, backoff
node verify_render.js   # 48 assertions: identity, staleness, zones, STOP, panels
```

`verify_link.js` loads `rosbridge.js` unmodified against a fake WebSocket and
drives the failure modes that actually happen: a socket that closes; one that
opens and immediately flaps; one that hangs in CONNECTING; one that is open and
silently dead; total silence; partial silence (the wrong-topic-root shape); and
re-pointing at another rover without leaving a zombie.

`verify_render.js` loads `index.html` plus all four scripts into a minimal DOM
shim, moves a **fake monotonic clock** forward, and asserts what is actually
rendered — including that a value goes stale with no new message arriving, that
`zones.json` is refused when a centre is 100 mm out, that a `hardware_id`
mismatch raises the identity banner, and that a silent mirror with a live
`/diagnostics` produces **CHECK THE THING NAME**.

Both were run against the current tree: **28/28 and 48/48**.

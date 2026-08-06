# FPMS on Foxglove — the operator console

This replaces the custom MQTT telemetry pipeline between the rover and the
laptop with a native ROS 2 one. The old Electron/FastAPI dashboard is **not**
deleted and still works — see [What happens to the old dashboard](#what-happens-to-the-old-dashboard).

---

## TL;DR — how you connect

1. Turn the rover on. Wait ~60 seconds.
2. On the laptop, double-click **`Open-FPMS-Foxglove.cmd`** (in this folder).
3. First time only: it opens Explorer with `layouts/FPMS-Operator.json` selected.
   In Foxglove, click the **layout menu (top-left) → Import from file…** and pick
   that file. You never have to do this again.

That's it. If it can't find the rover it prints five numbered things to check.

The address is **`ws://fpms-pi.local:8765`**. Write that down; never write down
an IP address — the rover's DHCP address has moved more than seven times.

---

## Why we moved off MQTT

Four measured problems, and what Foxglove does about each.

### 1. The bridge dropped ~99% of the LiDAR

Measured: the Pi published LiDAR at **9.52 Hz**; the laptop received **1 message
in 15 seconds** under WiFi saturation. The mosquitto Pi→laptop bridge ran at
**QoS 0**, which by definition has no delivery guarantee and no backpressure, so
under load it simply threw scans away and told nobody.

Foxglove's transport is a **WebSocket over TCP**. TCP retransmits lost segments
and applies backpressure, so a saturated link makes messages *late*, not *gone*.
When a client genuinely cannot keep up, `foxglove_bridge` hits its
`send_buffer_limit` (10 MB), drops that client's queue and **logs it**. Visible
loss beats silent loss. This is the core reason for the change.

*If the link is still not keeping up:* set `use_compression:=true` in
`units/fpms-foxglove-bridge.service` and restart it. That's the first knob.

### 2. Windows Firewall has no inbound rule, and there is no admin account

So nothing on the network can connect *into* this laptop. The old workaround was
to make the laptop connect **outbound** to the broker.

Foxglove keeps that direction for free: **`foxglove_bridge` LISTENS on the Pi**
and **Studio on the laptop dials out.** Outbound TCP is allowed by default and
needs no privilege at all. **Do not invert this.** If anyone ever suggests
running the bridge on the laptop and having the Pi connect to it, the answer is
no, for this reason.

### 3. The Pi's address keeps moving

Everything here addresses it as **`fpms-pi.local`**. The bridge binds
`0.0.0.0` so it does not care what address it gets; the launcher resolves the
name; the systemd unit contains no address.

The launcher has an IP fallback, but it *discovers* the address at runtime from
the neighbour table — there is no hard-coded IP anywhere in this folder. If it
does fall back, it says so loudly and tells you not to write the number down.

### 4. Telemetry went "stale but plausible"

This has bitten twice: a frozen 11 Hz rate that read as healthy while the link
had been dead for minutes, and a LiDAR panel that held its last frame forever.

That failure has **two different causes** and the old dashboard could not tell
them apart:

- **(a) the source stopped producing** on the Pi, or
- **(b) the link stopped delivering** to the laptop.

Foxglove answers **(b)** structurally, and this is the important part: the
WebSocket is a TCP connection with a state. When it dies, Studio's connection
indicator goes to *disconnected* and says so on screen. There is no code path in
Studio that invents a frame — every panel renders messages that actually
arrived, stamped with their real receive time. The Plot panel's x-axis is wall
time, so **a source that stops produces a flat line while the axis keeps
scrolling.** That is the picture you could never get before.

This project answers **(a)** with the command bridge, which publishes
`/diagnostics` at 2 Hz carrying the **age and measured rate of every source as
seen on the Pi**. Look at the **HEALTH** tab: if `/scan_lidar` is 40 seconds
old, the Pi itself says so — in a message whose own arrival proves the link is
fine.

Age measured at the source, plus arrival measured at the sink. Together they
make "stale but plausible" impossible. Neither one alone would.

---

## What got built

| File | Where it runs | What it is |
|---|---|---|
| `units/fpms-foxglove-bridge.service` | Pi | systemd unit for `foxglove_bridge`, port **8765**, enabled for cold boot |
| `units/fpms-foxglove-cmd.service` | Pi | systemd unit for the command bridge |
| `fpms_foxglove_cmd.py` | Pi | ROS topics/services ⇄ the rover's existing MQTT command verbs, plus the freshness diagnostics |
| `install_foxglove.sh` | Pi | one-shot installer + verifier |
| `layouts/FPMS-Operator.json` | laptop | the saved console layout |
| `Open-FPMS-Foxglove.cmd` / `.ps1` | laptop | one-click launcher |

Nothing in `fpms_missions.py`, `fpms_teleop.py`, `fpms-rover-agent` or
`fpms_lidar_ros.py` was changed. Nothing needed to be.

---

## Installing on the Pi

From the laptop, with the rover on:

```bash
scp -r cloud/dashboard/foxglove ubuntu@fpms-pi.local:~/foxglove
ssh ubuntu@fpms-pi.local "bash ~/foxglove/install_foxglove.sh"
```

The installer is idempotent — run it as often as you like. It:

1. checks arm64 / Ubuntu 22.04 / ROS 2 Humble / `python3-paho-mqtt`
2. `apt install ros-humble-foxglove-bridge`, **and if apt does not have it,
   builds from source into `~/fox_ws`** (10–25 min; the unit already sources
   that overlay if it exists)
3. installs `fpms_foxglove_cmd.py` to `/home/ubuntu`
4. installs and **`systemctl enable --now`** both units — *enabled*, so a cold
   boot at the venue comes up with the console working and nobody at a keyboard
5. verifies: both units active **and enabled**, something listening on `:8765`,
   `ROS_DOMAIN_ID=20` present in the unit, topic count on domain 20, and the
   `/fpms/*` services advertised

### `ROS_DOMAIN_ID=20` — the trap that has cost this project hours

The ESP32 drive board stores its own domain id of **20** and publishes on that
domain **and nowhere else**. Every existing `fpms-*.service` sets
`ROS_DOMAIN_ID=20` **in the unit**; `/etc/fpms/config.env` does **not** set it
and never has. Every ad-hoc `ros2 topic list` that forgot it saw an empty list,
which looks exactly like a dead link and is not one.

So:

- both new units set `Environment=ROS_DOMAIN_ID=20`;
- both also set `FPMS_ROS_DOMAIN_ID=20` and re-export it *inside* `ExecStart`
  **after** the login shell has run, because `bash -lc` sources `~/.bashrc` and
  a stray `export ROS_DOMAIN_ID=0` there would otherwise silently beat systemd;
- the command bridge logs its effective domain at startup and prints a
  four-line warning if it isn't 20;
- the installer checks it and fails the run if it's missing.

**If Foxglove connects but the topic list is empty, this is the first thing to
check, before touching any hardware.**

```bash
ssh ubuntu@fpms-pi.local
systemctl show fpms-foxglove-bridge -p Environment
source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=20 ros2 topic list
```

---

## The console

The layout has four tabs, and a **column down the right that never changes**,
so **STOP is on screen no matter which tab you are on.** That is deliberate: a
panic button that can be hidden by a tab is not a panic button.

**Always visible (right column):**

- **■ STOP** — big and red
- *clear e-stop latch* — small and grey
- **ARMED / SAFE** indicator — red when the rover can move
- odometry-link indicator
- **command receipts** — every ack, nack and receipt from the rover, live. This
  is how you know the button you pressed was *honoured*, instead of guessing
  from whether the rover moved.

**ARENA** — 3D view of the arena: the `map` frame with a 1200 × 1200 mm grid,
the TF tree, `/scan_lidar`, and `/odom`. Plus pose (x, y, heading) over time and
the mission phase as a state-transition strip.

**MISSION** — the six buttons (below), the full mission state as raw JSON, and
plots of distance remaining/travelled, front clearance, and **per-leg residual**
in mm and degrees, both instantaneous and cumulative.

**HEALTH** — the `/diagnostics` summary (per-source age and rate, from the Pi),
a battery gauge, the freshness plot, the connection info panel, and `/rosout`.

**SENSORS** — `/odom_raw` pose *and* twist (twist is labelled
`SIGN IS INVERTED — DO NOT TRUST`, because it is), `/imu`, `/wheel_ticks`,
raw `/battery`.

### About `/scan` vs `/scan_lidar` — read this

The task asked for `/scan`. **`/scan` is not the LiDAR.** On the current Yahboom
stock firmware `/scan` exists and every range in it is `0.0`; under firmware v3
it does not exist at all. The live scanner is owned by `fpms-rover-agent` on the
*other* CP2102 and reaches ROS as **`/scan_lidar`** (`fpms_lidar_ros.py`), in
frame `laser_frame`.

So the layout shows **`/scan_lidar` visible and `/scan` present but hidden**.
Turn `/scan` on if you want to see the dead topic for yourself; leave it off
otherwise, because a ring of zero-range returns at the origin looks like an
obstacle right on top of the robot.

---

## Mission control — how the buttons work

The rover's command surface is MQTT (`fpms/rover2/commands/…`) and Foxglove
speaks ROS. `fpms_foxglove_cmd.py` translates, so **nothing on the rover had to
change**. It is a translator and nothing else: it has **zero publishers on any
actuation topic** (`/cmd_vel`, `/cmd_duty`, `/cmd_enable`, `/estop`), and it
*aborts at startup* if anybody ever adds one.

| Button | Mechanism | Sends |
|---|---|---|
| **■ STOP** | **topic publish** `/estop` `{data:true}` | `commands/estop` **and** `commands/stop`, QoS 1 |
| 1. PLAN M2 | service `/fpms/plan_m2` | `commands/mission` `{"name":"m2","preview":true}` |
| 2. ARM | service `/fpms/arm` `{data:true}` | `commands/mission` `{"arm":true}` |
| 3. RUN M2 | service `/fpms/run_m2` | `commands/mission` `{"name":"m2"}` |
| DISARM | service `/fpms/arm` `{data:false}` | `commands/mission` `{"arm":false}` |
| RESET POSE | service `/fpms/reset_pose` | `commands/set_coordinate` `{"x_mm":972,"y_mm":228}` |
| READ ENCODERS | service `/fpms/read_encoders` | `commands/read_encoders` `{}` |

### STOP — why it is a topic and not a service, and why it cannot be blocked

**It is a topic publish, not a service call.** That is the whole design:

- A service call is request/response. Foxglove greys the button out while a call
  is in flight, the call needs a live server, and it can time out. **A panic
  button that can be "busy" is not a panic button.**
- A topic publish is fire-and-forget down an already-open TCP WebSocket. It
  cannot be refused, cannot block, and needs no reply to have worked.

There *is* a `/fpms/stop` service as well, for scripts that want a receipt. It
is not the button.

When you press STOP, `/estop true` fans out to **three consumers that do not
depend on each other**:

1. **The ESP32 firmware v3** subscribes `/estop` directly and latches all four
   motors to zero, overriding every path.
   ⚠️ **This is a no-op today** — the board is on Yahboom stock firmware, which
   has no `/estop`. See [What is not verified](#what-is-verified-and-what-is-not).
2. **The command bridge** mirrors it to MQTT `commands/estop` **and**
   `commands/stop` at **QoS 1**, on a *dedicated MQTT client that subscribes to
   nothing* — so its outbound queue can never be sitting behind telemetry.
   `fpms_missions` aborts on either verb; `fpms_teleop` acks and zero-Twists.
3. **`fpms-cored`** — the stop authority — sees those verbs on *its* dedicated
   stop client, re-publishes them at QoS 1, asserts ROS `/estop` itself, and
   **SIGTERMs `fpms-missions` if the executor still reports a driving phase 3
   seconds later.** That last resort exists for a wedged executor under a
   moving rover.

Inside the bridge, the stop path is hardened the same way `fpms_cored`'s is:

- its own `MutuallyExclusiveCallbackGroup` under a **MultiThreadedExecutor**, so
  a slow service handler cannot serialise ahead of it. (With rclpy's *default*
  single-threaded executor it would have — a stop arriving while a service
  handler was blocked in an MQTT publish would have waited. That is a panic
  button behind a queue, and it is why the executor choice is not incidental.)
- no JSON parsing, no locks, no state read before the publish;
- **no gate on arm, link, battery, or mission state. Ever.**
- QoS on the `/estop` subscription is `RELIABLE` + **`VOLATILE`**, and the
  VOLATILE is load-bearing: `fpms_cored` publishes `/estop` as
  TRANSIENT_LOCAL while Foxglove's client publisher is VOLATILE, and a
  TRANSIENT_LOCAL *reader* is **incompatible** with a VOLATILE *writer* — it
  would silently receive nothing from the panic button. Do not "improve" it.

The one thing that *can* stop STOP working is the Pi's local mosquitto being
down. That is reported explicitly on the HEALTH tab as
`fpms_foxglove_cmd/mqtt → STOP PATH DOWN`, in red, at 2 Hz. If you see that,
the buttons are decorations — go to the rover.

### RUN — why a stray click cannot drive the rover

**Three independent interlocks**, and only one of them is code being careful:

1. **Transport.** `foxglove_bridge` runs with
   `client_topic_whitelist:=[/estop, /fpms/cmd/.*, /clicked_point]`. The
   WebSocket is **physically incapable of publishing `/cmd_vel` or
   `/cmd_duty`.** Not "no panel does it" — there is no wire at all.
2. **Rover-side (the real one).** `fpms_missions` `REQUIRE_ARM` refuses any
   mission until `{"arm":true}` has been sent, and auto-disarms on any stop or
   abort, when a mission ends, and after 120 s unused.
3. **Bridge-side.** `/fpms/run_m2` refuses unless `/fpms/arm` was called within
   60 s, **and one ARM buys exactly one RUN** (the window is consumed). This
   still holds on a bench where somebody set `FPMS_MISSION_REQUIRE_ARM=0`.

So RUN is genuinely two deliberate actions, in order, within a minute. The
buttons are numbered 1-2-3 in the layout for that reason. If you press RUN
without ARM you get a refusal in the panel telling you exactly that.

**PLAN M2 cannot move the rover** — `fpms_missions._cmd_mission` returns from
the preview branch *before* any of the motion path. It plans, publishes the
route, and touches nothing that turns a wheel.

**RESET POSE does not move the rover either.** It *declares* that the rover is
at (972, 228) mm — the arena start pose from `arena_zones.json`, matching the
`map`→`odom` anchor in `fpms-tf.service`. **Physically put the rover on the
start square first**, then press it.

---

## What is verified, and what is not

**The Pi was powered off for this entire build.** Nothing below marked
UNVERIFIED has been run against hardware.

### Verified

- `fpms_foxglove_cmd.py` compiles (`py_compile`).
- `layouts/FPMS-Operator.json` is valid JSON, and every one of its 28 panels is
  both declared and placed exactly once (checked programmatically).
- `Open-FPMS-Foxglove.ps1` parses under Windows PowerShell 5.1 and **was run
  end to end on this laptop**: it correctly failed to reach `fpms-pi.local:8765`,
  fell back to scanning 10 neighbour addresses, found nothing, and printed the
  troubleshooting steps. The happy path (rover present) was **not** exercised.
- `fpms-pi.local` failed to resolve from Python's `getaddrinfo` in this session,
  exactly as previously recorded. This is why the launcher tests with .NET and a
  real TCP connect instead of trusting a name lookup.
- The MQTT verbs, payload shapes and reply topics were read directly out of
  `fpms_missions.py`, `fpms_teleop.py` and `stack/fpms_cored.py`, not assumed.

### UNVERIFIED — the honest list

1. **`ros-humble-foxglove-bridge` exists in apt for arm64/jammy.** Humble is
   Tier 1 on jammy arm64 and foxglove_bridge is a released Humble package
   (3.4.3), so this *should* be a one-line install — but it was not run against
   this Pi's apt. The installer falls back to a source build if it isn't there.
   **Budget 25 minutes for that on the day you first install.**
2. **The `foxglove_bridge` parameter names.** `client_topic_whitelist`,
   `service_whitelist`, `capabilities`, `send_buffer_limit`, `use_compression`,
   `num_threads`. `ros2` **rejects a `-p` for a parameter a node does not
   declare**, so a version skew shows up as a unit that will not start, naming
   the parameter in the journal. A minimal fallback `ExecStart` is in the unit,
   commented out, one line. **Using it loses the client-publish whitelist** —
   read the warning above it.
3. **Whether the layout loads cleanly in your Foxglove version.** It is
   hand-written to the standard export shape (`configById` / `layout` /
   `globalVariables` / `userNodes` / `playbackConfig`). Panel *settings* are
   permissive — Foxglove ignores keys it doesn't know and defaults ones it
   doesn't find — so the realistic failure is a panel that opens with wrong
   colours, not a layout that refuses to import. Not proven.
4. **Firmware `/estop` does nothing right now.** `PI_FILE_INVENTORY.md` §6
   establishes the board is on Yahboom stock (`ros2 param list /YB_Car_Node` is
   empty and `/scan` is present). Stock has no `/estop`. **Today, STOP works
   entirely through the MQTT path** — which is the path that has always worked
   and which `fpms-cored` backs with escalation. It gets a second, independent
   hardware-level path for free the moment firmware v3 is flashed.
5. **`/wheel_ticks` does not exist yet** — it is a firmware v3 topic. The
   SENSORS plot for it will be empty and `/diagnostics` will report it as
   `NEVER SEEN`. That is correct reporting, not a bug.
6. **The systemd units have never been loaded.** `systemd-analyze verify` was
   not run. They are modelled line-for-line on the existing units
   (`/bin/bash -lc 'source /opt/ros/humble/setup.bash && exec …'`,
   `Restart=always`, `After=network-online.target`, `User=ubuntu`).
   One deliberate deviation: mine use `;` instead of `&&` between the sources,
   so the optional `~/fox_ws` overlay is allowed to be absent.
7. **Bandwidth under real WiFi saturation.** The *reason* to expect an
   improvement is structural (TCP vs QoS 0 UDP-ish best-effort), not measured
   on this link. Measure it: watch the LiDAR rate in the topic sidebar during a
   run, and if it is still bad, `use_compression:=true` is the first knob.

---

## What happens to the old dashboard

**Nothing. It is not deleted and it still works.**

- **Foxglove becomes the primary operator console** for anything live:
  telemetry, LiDAR, TF, pose, plots, mission state, and the STOP button.
- **`cloud/dashboard/` (Electron + FastAPI, `http://127.0.0.1:8000`) stays as
  the fallback.** It is the only thing that has ever run at a competition, and
  the week before Nationals is not when you delete your fallback. Launch it the
  same way as always: `FPMS-Dashboard.bat`.

Run both at once if you like — they use different transports and different
ports and do not conflict.

### Capability genuinely lost, stated honestly

**The arena bird's-eye map with click-to-place markers and right-click keep-out
obstacles is NOT reproduced.** That is what the operator actually valued in a
UI, and it lives in the rescued Flask dashboard at
`cloud/dashboard/rover/rescued/fpms_phase6_LATEST.py` (rover-hosted, port 8085).

What Foxglove gives you instead is a real 3D/TF view of `/scan_lidar` in the
`map` frame with a 1.2 m arena grid — which is *better* for seeing where the
rover and the obstacles actually are, and **worse** for putting things into the
world with a mouse. The two are not the same tool.

Partially bridged, and only partially: Foxglove's 3D panel has a *Publish point*
tool that emits `geometry_msgs/PointStamped` on `/clicked_point`. That topic is
in the client whitelist, and the command bridge logs each click and echoes it to
MQTT `control/operator_point`. **Nothing on the rover consumes it.** Clicking
gets you a marker in the 3D view and a line in the journal — it does **not**
place a keep-out obstacle. Making that real means a consumer in the planner,
which is not this work and not this week.

Two smaller gaps, for completeness:

- **Camera and thermal views.** `fpms-rover-agent` publishes those to MQTT as
  JPEG payloads, not as ROS `sensor_msgs/Image` or `CompressedImage`, so
  Foxglove's Image panel has nothing to show. The old dashboard keeps this.
  Fixable later with a small ROS republisher; not done.
- **Terminal / SSH / LAN-scan.** The edge app has them; Foxglove is a
  visualisation tool and never will. Use the old dashboard, or a terminal.

---

## Troubleshooting, in the order things actually break

| Symptom | Almost always |
|---|---|
| Launcher can't reach `:8765` | Rover off, or not joined to the hotspot. Check *Settings → Mobile hotspot → Devices connected*. Nothing on the laptop fixes this. |
| Connects, **topic list empty** | `ROS_DOMAIN_ID`. See above. Not a dead link. |
| Connects, topics listed, **panels frozen** | The *source* stopped, not the link. HEALTH tab tells you which one and how old it is. |
| STOP button greyed out | You are not connected, or `client_topic_whitelist` doesn't include `/estop`. Reconnect first. |
| Buttons do nothing, no receipts | HEALTH tab → `fpms_foxglove_cmd/mqtt`. If it's red, the Pi's mosquitto is down. |
| RUN refused | You didn't ARM, or the 60 s window expired. The refusal message says which. |
| LiDAR ring of zeros at the origin | You turned `/scan` on. Turn it off; `/scan_lidar` is the real one. |
| Web app won't connect | It can't, ever. Use the desktop app — see below. |

### Desktop app vs web app — get this right

**You must use the Foxglove *desktop* app.** The web app at `app.foxglove.dev`
is served over HTTPS, and a page in a secure context is not allowed to open an
insecure `ws://` socket. The browser blocks it as mixed content. The
"localhost is trustworthy" carve-out does not help, because `fpms-pi.local` is
not localhost.

Making the web app work would mean running `foxglove_bridge` with TLS *and* a
certificate the browser trusts for `fpms-pi.local`. Do not attempt that the week
of a competition.

### If mDNS fails but the rover is up

`fpms-pi.local` resolving is per-resolver, not per-machine — in one session it
worked from .NET and failed from Python's `getaddrinfo` **at the same time**. So
"ping works" does not prove Foxglove will resolve it, and vice versa.

Find the address and pass it in:

```
Open-FPMS-Foxglove.cmd 192.168.137.42
```

Or in Foxglove: *Open connection → Foxglove WebSocket →* `ws://<address>:8765`.

The proper fix is a `hosts` file entry, which needs administrator rights that
this laptop does not have. Use the argument.

---

## Ports

| Port | What |
|---|---|
| **8765** | `foxglove_bridge` WebSocket **(this)** |
| 1883 | mosquitto, on the Pi |
| 8000 | old edge dashboard, on the laptop |
| 8085 | rescued phase6 Flask dashboard, on the rover |
| 8089 | teach-in server, on the rover |

No conflict.

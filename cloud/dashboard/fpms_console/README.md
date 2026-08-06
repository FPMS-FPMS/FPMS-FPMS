# FPMS Console — our own dashboard, on ROS

This is the FPMS rover's own operator console. It is **not** Foxglove. It runs
in a browser, it is **served by the rover itself**, and every number on it
arrives as a native ROS message over a `rosbridge` WebSocket.

Foxglove is still installed on the Pi and still works — keep it, it is a good
debugging tool. It is just not the thing you fly the rover with any more.

---

## 1. How you open it (this is the whole thing)

**Double-click `Open-FPMS-Console.cmd`.**

Or type this into any browser, on the laptop, a phone or a tablet:

```
http://fpms-pi.local:8090/
```

That is it. Nothing to install, nothing to start on the laptop.

> **Always type the NAME, `fpms-pi.local`. Never an IP address.** The rover's
> address has changed more than seven times in this project. Every note that
> wrote one down was wrong the next day.

If the page does not load:

| What you see | What it means | What to do |
|---|---|---|
| Browser says "can't reach this site" | The Pi is off, still booting, or on different wifi | Wait 60 s after power-on. Check the laptop and the Pi are on the same hotspot. |
| Page loads, header says **LINK DOWN** in red | The console is served but rosbridge is not answering | On the Pi: `sudo systemctl status fpms-rosbridge` |
| Page loads, everything says **never seen** | ROS is running on the wrong domain | Check `ROS_DOMAIN_ID=20` is in the unit files. This is the #1 trap in this project. |

---

## 2. What is on the screen

**The big red STOP button, top right.** It is the largest control on the page
and it never scrolls away. **The spacebar does the same thing** from anywhere.

**The map, on the left.** The arena is drawn **fixed** — the square never
moves, never rotates. The rover moves on it. (The operator calls this "Tesla
configuration".) On it you see:

- the arena, the two zones and the water station, drawn from the same numbers
  `fpms_missions.py` uses;
- the rover, as a rectangle with a nose line, at its real pose;
- the **LiDAR dots**, in the arena frame;
- the **planned route** as a green line with the waypoints marked;
- **front clearance**: an amber arc at the 400 mm stop threshold and a live arc
  at whatever the rover can actually see. When it goes red, that is *why* the
  rover stopped;
- **left-click** puts a marker down. **Right-click** drops a keep-out disc.
  **Shift + right-click** clears the discs.

**The right column**, top to bottom: commands, arm state, **residuals**,
telemetry, the planned route detail, your markers, feed ages, and the event log.

---

## 3. Running a mission

1. **PLAN M2 (preview)** — safe. It asks the rover to work out a route and draw
   it. It *cannot* move the rover; the mission code returns before any motion
   when `preview` is set.
2. Look at the green line. Is it going where you expect?
3. **ARM** — this unlocks RUN for 60 seconds.
4. **RUN M2** — the rover drives.

RUN is deliberately two actions. It is also **greyed out** whenever the link is
down, or the LiDAR or odometry has gone stale, and the yellow line underneath
tells you exactly which one. That is not a suggestion — the button is disabled.

**STOP works at every point in that sequence, including while RUN is in
flight.**

---

## 4. Reading the residual panel — the most useful numbers on the rover

After every measured segment the rover reports **what it was told to do** next
to **what it actually did**. Nothing has ever displayed this before.

| What you see | What is wrong | What to change |
|---|---|---|
| Every drive short (or long) by the **same ratio** | counts-per-mm scale | `odom_scale` in `/etc/fpms/calibration.json` — 2.467 is the known suspect |
| Same **fixed extra mm** every time, long or short | it coasts after the burst is cut | `coast_mm` |
| Only the **turns** drift | gyro scale | `gyro_scale` |

---

## 5. Staleness — why you can trust what you see

This project has twice been fooled by a dashboard showing plausible numbers
while the rover was actually unreachable. So this console does the following,
by mechanism, not by good intentions:

- **Every value is aged in your browser** from the moment its message arrived.
  No timestamp inside any message is used for freshness — a dead publisher can
  keep echoing an old stamp, and the Pi's clock and the laptop's clock disagree.
- After **3 seconds** a value is **struck through**, greyed, and labelled
  `STALE 7s`.
- A stale LiDAR is drawn as **ABSENT** — the dots vanish and a red box says so.
  The map never holds a dead frame.
- A stale LiDAR or odometry **disables RUN**.
- If the WebSocket drops, the header goes red **immediately** and it reconnects
  by itself.
- The **FEED AGES** panel shows the raw ages of every stream, all the time.

---

## 6. The two things Foxglove could not do, and one it still cannot

Recovered from the old :8085 phase6 console:

- ✅ the **fixed-arena bird's-eye map with the rover moving on it**;
- ✅ **click-to-place markers**;
- ✅ **right-click keep-out discs**;
- ✅ the dark tactical look, the mission buttons and the event log.

Not recovered, and **do not pretend otherwise**:

> ⚠️ **The keep-out discs are NOT used by the planner.** There is no keep-out
> command anywhere in `fpms_missions.py`; its occupancy grid is built from
> LiDAR returns only. The discs are drawn, published on
> `/fpms/console/keepout` and logged on the Pi so everyone can see what the
> operator believed was blocked — but the rover will happily plan straight
> through one. The console says this on screen, in those words.

> ⚠️ **The dots on the map are LiDAR returns, not the planner's grid.** The
> planner only publishes occupancy *counts*, never the cells. The counts are
> shown as numbers in the PLANNED ROUTE panel.

---

## 7. Installing / reinstalling it on the Pi

From the laptop, copy this whole folder to the Pi, then:

```sh
ssh ubuntu@fpms-pi.local
cd ~/fpms_console
sudo ./install_console.sh
```

Useful variants:

```sh
sudo ./install_console.sh --dry-run   # show every action, change nothing
sudo ./install_console.sh --status    # is it up? what whitelist is in force?
python3 verify_console.py             # PROVE the safety + freshness claims
```

The installer touches **only** the two new services and one config file. It
does not restart `fpms-missions`, `fpms-teleop`, `fpms-cored`, the rover agent,
micro-ROS, or either Foxglove unit.

Both services are **enabled**, so they come back on their own after a power
cut. Verified: the Pi rebooted mid-session and both came back without help.

### Editing the UI

`console.html` is re-read from disk on every page load. Edit it on the Pi (or
re-copy it), press reload — no service restart.

---

## 8. What talks to what

```
  BROWSER  (laptop / phone — dials OUT, always)
     |  http :8090   the page itself
     |  ws   :9090   every live value, and every command
     v
  ORANGE PI 5B
     +-- fpms-console.service    serves console.html
     |                           mirrors telemetry/mission_plan -> /fpms/plan/route
     +-- fpms-rosbridge.service  ROS 2 <-> WebSocket, WHITELISTED
     +-- fpms-foxglove-cmd       owns /fpms/* services + the mission mirrors
     +-- fpms-cored              STOP authority, receipts, escalation
     +-- fpms-missions           the planner and the measured-segment executor
     +-- micro-ros-agent         serial link to the ESP32
```

**The Pi listens; the laptop dials out.** Do not invert this. Windows Firewall
on the operator laptop has no inbound rule for 8090, 9090 or 1883 and there is
no admin account to add one.

### Ports

| Port | Service | Purpose |
|---|---|---|
| **8090** | `fpms-console` | the console web page |
| **9090** | `fpms-rosbridge` | ROS ↔ WebSocket (rosbridge v2 protocol) |
| 8765 | `fpms-foxglove-bridge` | Foxglove, kept as a debug tool |
| 1883 | `mosquitto` | MQTT, internal to the Pi |

### Topics the console reads

`/scan_lidar` (**not** `/scan` — that one is empty), `/odom_raw`, `/imu`,
`/battery`, `/wheel_ticks`, `/wheel_duty`, `/fpms_health`, `/diagnostics`,
`/fpms/mission/*`, `/fpms/residual/*`, `/fpms/events`, `/fpms/plan/route`,
`/fpms/console/keepout_state`.

### The only things the console can send

| What | How | Why it is safe |
|---|---|---|
| STOP | **publish** `/estop` (Bool true) + `/fpms/cmd/stop` (Empty) | stopping is never gated |
| marker | publish `/clicked_point` | nothing on the rover acts on it |
| keep-out | publish `/fpms/console/keepout` | recorded only, see §6 |
| plan / run / arm / reset pose / read encoders | ROS **services** `/fpms/*` | each applies the rover's own refusals |

---

## 9. Why STOP cannot be blocked

Four separate reasons, and none of them relies on another being true:

1. **It is a topic publish, not a service call.** A service call has a request
   id, needs a live server, and can time out. A panic button that can be
   *busy* is not a panic button. A publish is fire-and-forget down a socket
   that is already open.
2. **It has its own WebSocket**, which subscribes to nothing. The main socket
   carries a 10 Hz LiDAR scan; a browser `send()` queues behind whatever is
   already in that socket's buffer. The stop socket's buffer is empty by
   construction.
3. **It sends both verbs.** `/estop` reaches the firmware's control task and
   `fpms-cored` directly; `/fpms/cmd/stop` reaches the Pi-side bridge, which
   fans out to MQTT `commands/stop` and `commands/estop` at QoS 1.
4. **`fpms-cored` latches it.** It re-publishes at QoS 1, asserts `/estop`
   itself, and if the executor is still reporting a driving phase 3 seconds
   later it SIGTERMs `fpms-missions`.

If the dedicated socket happens to be down, STOP falls back to the main socket
**and the event log says which path carried it**. It never silently does
nothing.

---

## 10. Why a stray click cannot drive the rover

`rosbridge` whitelists nothing by default — a stock server lets any browser tab
publish anything, `/cmd_vel` and `/cmd_duty` included. So:

- `/etc/fpms/rosbridge_params.yaml` sets `topics_glob`, and the drive topics
  are **absent** from it. rosbridge enforces the glob in three places
  (`subscribe`, `advertise`, `publish`), so the browser cannot even create a
  publisher on a drive topic.
- `services_glob` is limited to `/fpms/*`.
- `params_glob` is `[]` — no parameter access at all.
- On top of that, RUN needs ARM first, in the console *and* on the rover.

**Verify it, don't trust this file:**

```sh
python3 ~/fpms_console/verify_console.py
```

It attempts to advertise `/cmd_vel`, `/cmd_duty` and `/cmd_enable` and reports
whether rosbridge refused. It also subscribes to `/tf` — a busy topic that is
deliberately *not* whitelisted — because silence there is positive proof the
whitelist is switched on, rather than proof that nothing happened to be
publishing.

---

## 11. Troubleshooting

| Symptom | First thing to check |
|---|---|
| every feed "never seen" | `ROS_DOMAIN_ID=20` in the unit. It is **not** in `config.env` and never has been. |
| LiDAR box says ABSENT | `systemctl status fpms-rover-agent fpms-lidar-ros`. Remember `/scan` is *not* the LiDAR. |
| no green route line | `systemctl status fpms-console` — the plan mirror needs mosquitto |
| RUN stays greyed | read the yellow line under the buttons; it names the reason |
| buttons say "no response" | `systemctl status fpms-foxglove-cmd` — it owns the `/fpms/*` services |
| page is stale after an edit | it is `no-store`; hard-reload once (Ctrl-F5) |

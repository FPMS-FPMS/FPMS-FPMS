# The FPMS Pi stack — services, topics, and the three guarantees

Orange Pi 5B, ROS 2 Humble, `ROS_DOMAIN_ID=20`. Built 2026-08-06 on top of the
B8B/phase6 method, not against it.

Deploy the whole thing with one command:

```sh
sudo ./deploy_stack.sh          # --dry-run first if you want to see it
```

Check it without deploying:

```sh
./deploy_stack.sh --status
python3 test_stack.py           # 40 offline checks, no Pi needed
```

---

## 0. The one-paragraph version

Nine services, one dependency order, everything enabled for cold boot. A new
process, **`fpms-cored`**, owns the STOP latch and answers every command with a
receipt. The **LiDAR publishes on a timer, not on data**, so a dead scanner is a
declared fault instead of a frozen picture. The mission executor is unchanged
where it matters: it still plans autonomously and still drives
burst → stop → settle → **measure at rest** → correct. What changed around it is
that odometry now passes through **one** documented sign-and-scale boundary, and
every segment publishes what it was **asked** to do next to what it **actually**
did.

---

## 1. Service tree, in dependency order

```
network-online.target
└── mosquitto.service                     the broker; EVERY unit now waits on it
    │
    ├── micro-ros-agent.service           serial link to the ESP32  ── UNTOUCHABLE
    │   └── fpms-uros-supervisor.service  keeps the XRCE session alive
    │
    ├── fpms-cored.service          [1]   STOP authority · receipts · health
    │   │                                 Before= everything that can move
    │   ├── fpms-teleop.service     [2]   MQTT → /cmd_vel (joystick, manual)
    │   └── fpms-missions.service   [3]   planner + B8B measured-segment executor
    │
    ├── fpms-rover-agent.service          camera + LiDAR → MQTT   ── the scan source
    ├── fpms-lidar-ros.service            MQTT LiDAR → /scan_lidar
    ├── fpms-tf.service                   static TF + the map→odom arena anchor
    ├── fpms-odom-tf.service              /odom_raw + /imu → /odom, odom→base_footprint
    └── fpms-wifi-powersave-hold.service  bcmdhd re-enables power save every ~30s
```

**Masked, permanently:** `fpms-ros-tunnel`, `fpms-rtos-follower`. Both can write
`/cmd_vel`. Masked rather than merely disabled, because `disable` only drops the
`WantedBy` symlink and anything pulling them in by name still starts them. Two
writers on `/cmd_vel` is not a race the executor can win or reliably detect.

**Retired:** `fpms-map-odom.service`, merged into `fpms-tf.service`.

### Why the ordering is mostly *absent*, and where it isn't

Almost every unit here declares only `After=network-online.target
mosquitto.service`. That is deliberate and inherited from the previous
maintainer's reasoning, which is sound: **the micro-ROS serial link costs
90–225 s to re-establish**, so a dependency edge onto `micro-ros-agent` would
let a trivial restart of a bridge become a multi-minute outage of every topic on
the robot. Each node instead detects its own missing input and says so.

There are exactly two real edges:

| Edge | Why |
|---|---|
| everything → `mosquitto.service` | **NEW, and it was a genuine cold-boot race.** No unit declared it. The broker takes a moment to bind :1883, and clients that lost the race sat in their own retry loops while the dashboard showed nothing. |
| `fpms-cored` → **before** `fpms-missions`/`fpms-teleop` | A safety property. There must be no window at boot where something that can move is accepting commands while the stop authority is not yet listening. `Wants=`, not `Requires=`: if cored fails, missions must still start — a rover you cannot drive is not safer than one you can stop. |

---

## 2. What I consolidated, and what I deliberately split

| Change | Why |
|---|---|
| **+ `fpms-cored`** (new) | Three gaps had no owner: STOP was never acknowledged, unknown verbs vanished silently, and no process could act when the executor itself was wedged. |
| **`fpms-map-odom` merged into `fpms-tf`** | Two units for two static transforms bought nothing and left a boot-order question with no answer. One unit, one TF owner. |
| **`fpms-ros-tunnel` + `fpms-rtos-follower` masked** | Removes a whole class of failure (second `/cmd_vel` writer) rather than documenting it. Both were already disabled and neither is needed. |
| **LiDAR and camera SPLIT onto separate MQTT clients** | The one place consolidation was *wrong*. See §3. |
| **`fpms_missions.py` NOT rewritten** | It is gate-verified (m2 = 744.0 mm). Four surgical additions, proven planner-neutral by `test_stack.py`. |

Net: 7+ ad-hoc units → 9 units with one documented order, all enabled, all
`Restart=always`.

---

## 3. Guarantee 1 — the LiDAR never goes stale

This has failed twice, for two different reasons. Both are now closed by
mechanism, not by intent.

### The publisher (`fpms-rover-agent.py`)

1. **The emit is on a TIMER, not on data.** The old code read
   `if time.time() - last_emit >= interval and points_seen:` — so a stopped
   scanner published *nothing*, and the dashboard held its last frame forever,
   indistinguishable from live. There is now no `and points_seen`, no early
   `continue` that can skip the emit, and no path where silence is the output.
   The only way this loop stops publishing is if the process dies — which
   systemd and the MQTT last-will both make visible.
2. **`hz` is recomputed from scratch every emit and FORCED to 0.0 unless the
   newest point is fresh right now.** Never an EMA, never a ring-buffer average.
   Both keep returning a plausible number long after the source dies, and a
   frozen plausible value has fooled this project repeatedly.
3. **The port reopens on BOTH failure shapes.** Byte silence (a wedged
   descriptor that `read()`s empty forever without raising) *and* frame
   starvation (bytes arriving that are not LD frames — wrong baud, another
   reader on the tty). Neither raises, so neither could be caught by the thread
   supervisor above; each now has its own deadline.
4. **The scanner is resolved by USB topology path**, and any device resolving to
   the micro-ROS board is refused. Both CP2102 adapters report ID_SERIAL `0001`,
   so `by-id` is unusable, and opening the board's tty raises the handshake
   lines wired to its EN/IO0 pins — resetting the ESP32 and costing a 90–225 s
   re-link.
5. **LiDAR gets its own MQTT client, camera gets another with a bounded queue.**
   paho serialises all publishes from one client onto one network thread, in
   order. A 40 kB base64 camera frame queued ahead of a 2.5 kB scan delays the
   scan by however long the frame takes to drain. Nothing was *dropped* — scans
   simply arrived **late**, which is the hardest version of this failure to see.
   Camera frames now drop instead of queueing (`max_queued=2`); QoS-1 fire and
   fault events are routed to the unbounded client so they are never discarded.

### The stale contract (the consumer side — this is the sharp edge)

Publishing on cadence means **arrival no longer implies freshness**, and a dead
payload carries all-zero ranges. Zero converts to `inf` — "no return" — which
reads as a **completely clear 360°**. Left unhandled, the never-stale fix would
have turned "the sensor is broken" into "full speed ahead": strictly worse than
the old fail-open, because the guard would be *confidently* wrong.

So every consumer drops `stale: true` rather than acting on it:

| Consumer | Behaviour |
|---|---|
| `fpms_lidar_ros.py` | Does not publish a `LaserScan`. Nav2 sees the scan **stop** — a failure mode ROS already handles correctly. |
| `fpms_missions.py` `on_lidar` | Does not store the scan, does not refresh `lidar_last`, does not fold into the occupancy grid. Within `LIDAR_STALE_S` the existing obstacle guard ages out and behaves exactly as it always has when the LiDAR is missing. **This adds a refusal; it removes no guard.** |

`stale` is additive, so an un-upgraded agent still works unchanged.

### Payload — additive only, nothing renamed

Existing keys are untouched: `ranges_m`, `range_max_m`, `heading_deg`, `points`,
`sectors_mm`, `sector_width_deg`, `min_mm`, `min_bearing_deg`, `baud`, `ts`,
`thing`. Added: `scan_age_s`, `last_scan_ts`, `health` (`ok`/`stale`/`dead`),
`stale`, `hz`, `port`, `seq`, `port_reopens`.

`seq` advances on every emit including dead ones — so `seq` climbing while
`stale` is true means *the scanner* is the fault, and `seq` frozen means *the
process* is. Those need different fixes and were previously indistinguishable.

---

## 4. Guarantee 2 — STOP is honoured unconditionally

**Mechanism: STOP has its own MQTT client, on its own socket, with its own
network thread, subscribed to nothing but `stop`/`estop`/`auto_off`.**

That is the whole trick. paho delivers all messages for a client on one thread,
in order. Inside `fpms_missions.py` that same thread also runs `on_lidar` (a
360-bin occupancy-grid integration) and `_cmd_mission` (which holds a lock
across an A\* search). Neither is time-bounded, so a stop arriving behind either
of them waits. In `fpms-cored` nothing can ever be in front of a stop, because
nothing else is subscribed on that client.

The callback does **no work**: no JSON parse before the latch, no lock, no I/O —
it sets a `threading.Event` and appends to a deque. Measured at **3.9 µs/call
over 10 000 calls**, and a deliberately malformed payload still latches
(`test_stack.py`).

Fan-out then runs on a **pre-started worker thread**, over four independent
paths, because the point is not to depend on any one surviving:

| # | Path | Covers |
|---|---|---|
| a | Re-publish `commands/stop` at **QoS 1** | The executor missed the original (dashboard published QoS 0, or a reconnect race). |
| b | ROS `/estop` (`Bool`, RELIABLE + **TRANSIENT_LOCAL**) | Firmware v3 cuts motors in its control task, independent of Pi logic. Transient-local so a node subscribing *after* the stop still gets it. **Harmless no-op on factory firmware** — nobody subscribes. |
| c | `events/stop_asserted` + a receipt | The dashboard shows STOP LATCHED without waiting for the executor to agree. |
| d | **Escalation:** SIGTERM `fpms-missions` | The one case a–c cannot cover: they all assume the executor is still running its loop. |

**Escalation is the only destructive action in the stack**, and it is fenced:

- fires only if `telemetry/mission` still reports a **driving** phase 3 s after the stop;
- fires only on **fresh** evidence (≤ 2.5 s old) — killing a service because a
  *stale* reading said "driving" is its own hazard;
- an **absent** phase counts as *not* driving, an **unknown** phase counts as driving;
- once per stop, logged loudly, and disableable via `FPMS_CORE_STOP_ESCALATE=0`.

SIGTERM is the right signal, not a blunt one: `fpms-missions` installs a handler
that aborts the mission and publishes ten zero Twists before exiting, and
`Restart=always` brings it back.

> Two real bugs in this logic were found by `test_stack.py` before it ever ran on
> hardware: a missing `phase` stringified to `"none"` and counted as *driving*
> (would have caused a spurious SIGTERM of a healthy executor), and the evidence
> freshness bound was tied to the escalation delay, making escalation
> unreachable. Both fixed; both now have tests.

### Command receipts — no click is ever silent

`fpms-cored` publishes to **`events/command_receipt`** (QoS 1) for **every**
command:

```jsonc
{"cmd_id":"a1b2c3", "verb":"mission", "state":"received",
 "owner":"fpms-missions", "known_verb":true, "payload_echo":{...}}
```

States: `received` → then one of `acked`, `nacked`, `honoured` (stop),
`acted_silently`, `no_owner`, `timeout`, `escalated`, `escalation_skipped`.

This closes three holes:
- `stop` had **no receipt at all** (the executor's "ACT, DO NOT ANSWER" policy is
  correct for the executor, but left the most safety-critical button unconfirmed);
- an **unknown verb** produced no reply of any kind — indistinguishable from success;
- ownership is learned from each service's `events/online`, so the receipt names
  *who* should have answered.

Correlation to the executor's own `ack`/`nack` is by verb + recency (it does not
echo `cmd_id`), and the receipt says so explicitly: `"correlated":
"by_verb_recency"`. Honest about its own limits rather than implying an exact match.

---

## 5. Guarantee 3 — clean cold boot

- Every unit is **enabled** (`WantedBy=multi-user.target`) — `deploy_stack.sh`
  enables all nine, including `micro-ros-agent`. `fpms-missions` was previously
  found **disabled and silently absent after a reboot**.
- Every unit is `Restart=always`. `RestartSec` is 2 s for `fpms-cored` (it is the
  stop path), 3 s for the rest, 5 s for TF.
- Every MQTT unit now declares `mosquitto.service`.
- `EnvironmentFile=-/etc/fpms/config.env` on every unit, so one file configures
  the stack. The leading `-` means a missing file is not fatal, but
  `deploy_stack.sh` refuses to deploy without it — services would otherwise come
  up on defaults pointing at the wrong broker.
- After any board power-cycle, `fpms-teleop`/`fpms-odom-tf`/`fpms-missions`
  historically failed to re-match XRCE subscriptions. `fpms-uros-supervisor`
  handles the link; **this remains the least-covered failure mode — see §9.**

---

## 6. Topic map

### Consumed by the dashboard (backward compatible — additive keys only)

| Topic | Publisher | Notes |
|---|---|---|
| `telemetry/lidar` | rover-agent | + `scan_age_s`, `health`, `stale`, `hz`, `seq` |
| `telemetry/camera` | rover-agent | unchanged; now on a bounded client |
| `telemetry/pose` | rover-agent | + `lidar_health`, `lidar_age_s`, `lidar_hz`, `lidar_seq` |
| `telemetry/mission` | missions | unchanged |
| `telemetry/mission_plan` | missions | unchanged |
| `telemetry/drive` | teleop | unchanged |
| `events/*` | all | unchanged |

### New topics (purely additive — nothing renders them yet)

| Topic | Publisher | Purpose |
|---|---|---|
| `events/command_receipt` | cored | Every command, every disposition |
| `events/stop_asserted` | cored | STOP latched, immediately |
| `telemetry/health` | cored | Unit states, feed freshness, stop counters |
| **`telemetry/residual`** | missions | **Per-segment commanded vs MEASURED** |

### `telemetry/residual` — the most diagnostic number this rover produces

Published the instant each segment settles, with the chassis **stationary**, so
both numbers are trustworthy. Nothing displayed this before. It separates three
failures that look identical on the arena map:

| Pattern | Cause | Fix |
|---|---|---|
| Constant **ratio** (every drive short/long by the same factor) | scale error — counts/mm | set `odom_scale` (§7). **2.467 = 74/30** is the known candidate |
| Constant **offset** regardless of length | coast after the burst is cut | `coast_mm` |
| Grows only on **turns** | gyro/heading | `gyro_scale` |

Per segment, not per leg — a leg is several segments and averaging hides exactly
that pattern.

---

## 7. Pi-side change table for the new ESP32 firmware

Firmware v3 (`firmware_v3/fpms_config.h`) makes `/cmd_duty` primary, boots
`/cmd_vel` **disarmed**, publishes **raw per-wheel ticks**, `/odom_raw` with the
**correct sign**, a real gyro, and `BatteryState`.

| Constant | Factory (today) | Firmware v3 | Where | Status |
|---|---|---|---|---|
| **`FPMS_MISSION_ODOM_POSE_SIGN`** | **`-1`** | **`+1`** | `fpms_missions.py`, applied at `_take_odom` **only** | **NEW — was not implemented at all** |
| `odom_scale` | `1.0` | re-measure | `/etc/fpms/calibration.json` | Derivable without reflash |
| `FPMS_MISSION_TURN_WIRE_SIGN` | `-1` | `+1` | config.env | Unchanged, still unmeasured |
| `CMD_SCALE` | `6.1` | must collapse toward `1.0` | `fpms_missions.py:704` | **Not changed — see §9** |
| `ODOM_TWIST_SIGN` | `-1` | `+1` | teleop, odom-tf | **Not changed — see §9** |
| `MIN_PULSE_S` / min duty | `0.35 s` (dead-zone artefact) | `min_moving_duty` | calibration profile | v3 has no dead zone |
| `/estop` | absent (no-op) | honoured in the control task | `fpms-cored` | Publishes either way |

### `FPMS_MISSION_ODOM_POSE_SIGN` — read this before flashing

`firmware_v3/fpms_config.h:28–31` asserted *"the Pi compensates with
`FPMS_MISSION_ODOM_POSE_SIGN=-1`"*. **It did not.** The constant did not exist
anywhere in `fpms_missions.py`. A header comment nobody executes and the running
code disagreed about a sign, and that disagreement is invisible until the rover
drives the wrong way.

It now exists, is applied at **exactly one place**, and is enforced by a test.
Getting it wrong is silent and total: every displacement comes from differencing
pose, so an inverted sign makes a forward burst measure as reverse travel, the
residual correction push the wrong way, and the retrace replay the error again.

**Check it without driving** — 30 seconds, no battery:

```sh
python3 ~/fpms_charact.py --push-check     # push the rover along a tape measure
```

Pushed forward, `x` must **increase**.

> **Changing the sign invalidates the saved anchor.** `.fpms_teleop_origin.json`
> records a pair — "odom was here, the arena was there" — captured under
> whatever sign was active. Flipping it reflects the anchor through the origin,
> and both conventions produce perfectly plausible numbers. **Re-zero with
> `set_coordinate` after changing it.** The stale-anchor refusal catches the
> reboot case but not this one.

---

## 8. Phase 1 / Phase 2 — the joystick teaches the SYSTEM, not the ROUTE

**Phase 1 — characterisation (`stack/fpms_charact.py`).** The operator drives by
joystick; the tool only **watches**. There is no publisher of `/cmd_vel`,
`/cmd_duty` or any actuation topic in that file. It derives and persists to
`/etc/fpms/calibration.json`:

`odom_scale` · `counts_per_mm` (from raw ticks) · `gyro_scale` · `coast_mm` ·
`coast_deg` · `min_moving_duty` · `lr_asymmetry`

```sh
python3 fpms_charact.py --push-check      # sign + odom_scale   (do this FIRST)
python3 fpms_charact.py --spin-check 90   # gyro_scale
python3 fpms_charact.py --drive           # coast, min duty, asymmetry
python3 fpms_charact.py --report          # what is set, and where it came from
python3 fpms_charact.py --joystick-help   # joystick setup + the no-hardware fallback
```

A profile is adopted only when it carries `measured: true` **and** the value is
inside a sanity range; refusals are logged to `CFG_NOTES` at executor startup
(`calibration ... ADOPTED` / `... REFUSED`). A missing profile changes nothing.

**`odom_scale` is why this needs no reflash.** `/odom_raw` arrives in metres,
already integrated on the board using its `COUNTS_PER_REV`. Multiplying position
by a measured ratio at one boundary corrects the whole chain — and counts/mm is
the longest-running open number in this project: 6.00 derived, 14.8
tape-measured, 0.743× hand-push, plus a possible ×2 from `attachHalfQuad`.
14.8 / 6.00 = **2.467 = exactly 74/30**, a 74:1 gearbox where 30:1 was assumed.

> **No joystick node is configured on this Pi today.** `joy` and
> `teleop_twist_joy` are not installed and nothing launches them. Check with
> `ros2 pkg list | grep -E 'joy|teleop'` and `ls /dev/input/js*`.
> `--joystick-help` prints both the install path and the fallback: the
> **dashboard's existing drive controls work fine for characterisation**, because
> what is measured is what the *rover* did, not what the operator pressed.

**Phase 2 — the mission is fully autonomous.** No recorded path is involved.
Given a target, `fpms_missions.py` plans its own route (Theta\*/A\* over the live
LiDAR occupancy grid, turn cost, clearance gradient, hysteresis, fallback ladder,
`ObstacleDetour` live reroute), then converts it into slow measured segments:
**burst → STOP → settle → measure at rest → residual → correct**, distance
closing on encoders and heading on the gyro. That is B8B, already implemented and
gate-verified, and **it was not rewritten** — only fed better constants and made
to report its residuals.

---

## 9. Risks — blunt

**Nothing below was verified on hardware. The Pi was powered off for this entire
session (`fpms-pi.local` did not resolve) and nothing was driven.**

1. **`fpms-cored` has never run on the Pi.** The stop path is tested offline
   (7 assertions) but has never touched a broker, a real `telemetry/mission`
   stream, or systemd. **Verify it before trusting it**, per the deploy script's
   step 2. Until then, treat it as *additional* protection over the executor's
   existing stop, never as a replacement.
2. **Stop escalation can SIGTERM a healthy executor if `telemetry/mission`
   phases do not match my idle set.** I inferred that set from the code, not
   from a live capture. If you see unexplained `fpms-missions` restarts, set
   `FPMS_CORE_STOP_ESCALATE=0` and tell me the real phase strings.
3. **The repo was ~1100 lines behind the Pi on `fpms_missions.py`.** The repo
   copy (4915 lines, Aug 4) predates the Pi's live copy (6013 lines, Aug 5)
   *and* a further burst-cap/`ABORT_ODOM_FRAME` patch (6059). I installed the
   **newest** (6059) and proved it planner-identical. **But I could not check
   what is actually on the Pi**, because it is off. `deploy_stack.sh` refuses to
   overwrite with a shorter file unless forced — do not force it without a diff.
4. **`CMD_SCALE = 6.1` is unchanged and is a known defect artefact**, not a
   calibration. `fpms_firmware/README.md` says plainly "do not carry 6.1
   across". It must be re-measured after the v3 flash. I did not touch it
   because changing it, `TURN_WIRE_SIGN` and `ODOM_TWIST_SIGN` piecemeal makes
   the rover drive wrong in a *new* way, and I could not measure any of them.
5. **`ODOM_TWIST_SIGN` (teleop, odom-tf) is untouched at `-1`** and must become
   `+1` with v3. Same reason.
6. **The LiDAR mount transform is all zeros.** Every offset in
   `nav2/fpms_tf.launch.py` is a `MEASURE ME` placeholder and the unit launches
   it with no arguments. By the file's own arithmetic **1° of mount yaw ≈ 10.5 mm
   of position error**, and both the cone guard and the occupancy grid inherit it
   in the same direction. `LIDAR_ROTATION_SIGN = -1` is still marked UNVERIFIED.
7. **The obstacle and battery guards still fail OPEN on missing data**
   (`clear is None`, `v is None`). I strengthened the *stale* case; I did not
   change fail-open-on-absent, because that is a behavioural change to a proven
   file that I cannot test.
8. **`fpms-tf.service`'s merged `map→odom` publisher uses a PID file and
   `setsid`.** It is the least elegant thing here and the least tested. If TF
   misbehaves after deploy, split it back out first.
9. **The escalation sudoers rule is validated with `visudo -cf` but never
   exercised.** If it fails, escalation logs an error and paths a–c still work.
10. **Judge every one of these by the operator's eyes, not by odometry.** This
    rover has reported clean travel while spinning in place and has lied about
    direction. Nothing in this stack changes that; the residual stream is meant
    to make it *visible*, not to make odometry trustworthy.

### Not done

- No D\* Lite. An earlier measurement showed it optimises the wrong half here
  (worst-case A\* is 1.2 ms; the costmap rebuild it cannot avoid is also 1.2 ms).
- `fpms_teleop.py` untouched. Its two documented conflicts with the executor
  (answering `mission`, and the 2 Hz idle zero-Twist) are **already fixed in the
  code** — the warnings in `fpms-missions.service` were stale and have been
  removed. The app-level foreign-writer NACK remains the real guard.
- The duty path (`/cmd_duty`) is **not** wired up. The executor still publishes
  `Twist`. That is the correct next step once v3 is flashed and characterised,
  and it is a real piece of work — not a config change.

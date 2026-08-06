# Next session — plan (written 2026-08-06, end of session)

Rover was **shut down cleanly** at the end of this session (`sudo shutdown -h
now`). It is powered off, not crashed.

**Do this before anything else: CHARGE THE BATTERY.** It fell from 11.46 V to
10.94 V across the session (`/fpms_health data[8]`, millivolts). Do not start a
drive test on that.

**Then check all four wheel hubs by hand.** A front-left hub came off mid-session
on 2026-08-05 after three per-wheel anomalies were each explained away
separately. It was refitted; verify before driving.

---

## State at shutdown

Working and verified this session:

- Firmware v3.00.01, all four `MOTORn_INV` flipped, `COUNTS_PER_REV` 1120,
  counts/mm 5.5. Board uptime was climbing steadily, no resets after 21:35.
- `/estop` latch **cleared** (`/fpms_health data[1]` = 60, bit0 = 0).
- Gyro is **fine**. It reads exactly 0.0 parked only because of a ±0.01 rad/s
  deadband in `lib/imu/imu_interface.h:93-100`. `data[1]` bit3 (IMU init OK) was
  set. Do not re-open this from a parked reading — see the memory note.
- Full Pi stack deployed, all nine units **enabled at boot** for the first time
  (`fpms-cored` was previously running but not enabled). Stop-escalation sudoers
  rule installed; the two rogue `/cmd_vel` writers masked.
- LiDAR honest: `health ok`, `hz 9.45`, `stale false`. The `points_seen` bug is
  gone from the installed agent (emit is on a timer, not gated on data).
- STOP answered by `fpms-cored` at 0.0 ms, exactly one receipt per press.

---

## 1. Re-zero — BLOCKED ON A HUMAN, DO IT FIRST

Nothing else on the rover is trustworthy until this is done. The encoders were
zeroed by an unplanned ESP32 reboot, so `/odom_raw` is at ~0 — which actually
makes for a clean anchor.

Place the rover **physically in the start box, nose up the arena**, then:

```bash
eval $(sudo grep -E "^FPMS_MQTT_(USER|PASS)" /etc/fpms/config.env | sed "s/^/export /")
mosquitto_pub -h localhost -u "$FPMS_MQTT_USER" -P "$FPMS_MQTT_PASS" \
  -t 'fpms/rover2/commands/set_coordinate' -m '{"x_mm":972,"y_mm":228}'
```

972/228 is `ROVER_START`, verified independently from `fpms_missions.py`
(`ARENA_MM 1200`, `MARGIN_MM 48`, `ZONE_MM 360`). Then preview — **this commands
no motion**:

```bash
mosquitto_pub -h localhost -u "$FPMS_MQTT_USER" -P "$FPMS_MQTT_PASS" \
  -t 'fpms/rover2/commands/mission' -m '{"name":"m2","preview":true}'
```

**It must read 744.0 mm.** Zone-b centre is (972, 972); start is (972, 228);
744 mm straight up (+y), heading 90°. Anything else means the anchor is wrong —
stop and fix it rather than driving.

## 2. Straight-line test — needs your eyes and a tape measure

`ROS_DOMAIN_ID=20 python3 ~/fpms_drive.py straight 800`. It has still never been
executed. Judge the result by tape measure and by eye, never by odometry.

## 3. Fix the deploy properly — the Pi is running an OLD `fpms_cored.py`

This is the highest-value software task and it is not what it looks like.

`cloud/dashboard/rover/stack/fpms_cored.py` in this repo is **1027 lines** and
contains five layered loop guards plus a header documenting the exact STOP
self-feed incident ("NEVER FEED ITSELF... 3394 commands/estop ... took the
micro-ROS link down"). The Pi was running an **803-line copy with none of them**,
which is why one STOP press produced a ~53/s storm this session that rebooted
the ESP32.

`~/deploy_v3/` on the Pi is a **stale snapshot** of `cloud/dashboard/rover/`, so
re-running `deploy_stack.sh` from it cannot fix this — its `stack/fpms_cored.py`
is the old unguarded file too. **Sync `cloud/dashboard/rover/` to the Pi first,
then deploy.** Confirm with:

```bash
grep -c "NEVER FEED ITSELF" ~/fpms_cored.py     # must be non-zero
```

Then re-verify: one STOP, count receipts for 10 s, expect exactly 1.

An interim 4-line guard is live on the Pi (marked "ADDED 2026-08-06") and does
stop the loop, but it is narrower than the repo's five guards. Prefer the repo's.

**Two more deploy landmines while you are in there:**

- `deploy_stack.sh` version-guards **only** `fpms_missions.py`. It silently
  overwrites `fpms_teleop.py` and `stack/fpms_cored.py`, so any live fix is
  reverted by the next deploy. That nearly cost the `/battery` fix this session.
- Its `UNITS` list omits `fpms-rosbridge`, `fpms-console` and both foxglove
  units, so they are never restarted or enabled by a deploy.

## 4. ROS-native dashboard port — increment 2

Increment 1 is landed, verified 18/18, and **off by default**. Read
`cloud/dashboard/ROS_PORT.md` before touching it — it has the architecture
decision, the channel table and the reason for each remaining choice.

Do them in this order, cheapest first:

1. **`events`** ← `/fpms/events` (`std_msgs/String` of JSON). Nearly direct;
   held back only so increment 1 changed one behaviour, not two.
2. **`mission:`** ← `/fpms/mission/{state,phase,leg_i,segment_i,distance_*}`.
   Same collect-then-emit pattern as pose.
3. **`drive:`** ← `/fpms_health` + `/battery`. Partial port only — the panel also
   reads teleop-only fields (measured topic rates, micro-ROS link state) that do
   not exist on the ROS graph.
4. **`lidar:`** ← `/scan_lidar`. Needs care: `LaserScan` carries none of the
   health/staleness fields the panel consumes, and a careless mapping would
   replace a feed that is currently honest about going stale with one that is
   not.

**`camera:` and `thermal:` cannot be ported** — no ROS publisher exists, the
agent sends frames straight to MQTT. That needs a rover-side change.

Enable and verify:

```powershell
$env:FPMS_ROS_ENABLED="1"; $env:FPMS_ROS_HOST="fpms-pi.local"
python scripts/verify_ros_bridge.py --live      # 18/18 expected
```

`GET /api/health` now has a `ros` block reporting connection, subscriptions and
which channels ROS currently owns.

**If it connects and delivers nothing, restart `fpms-rosbridge` first.** It was
found wedged this session — accepting connections, logging "Subscribed", and
forwarding zero messages on five topics with no error anywhere. A restart fixed
it (0 → 280 msgs in 18 s). Root cause is NOT established; the theory that the
deploy's mass restart orphaned its DDS readers was tested and disproved. While
it is wedged the operator console on :8090 has no live data either.

## 5. Still unmeasured — unchanged from last session

- **Track width** — three live values, none measured. Turns are untrustworthy.
- **LiDAR mount transform** — all zeros, every offset a MEASURE ME placeholder.
- **Turn calibration** — no turn has been performed deliberately.
- `CMD_SCALE 6.1`, `TURN_WIRE_SIGN -1`, `ODOM_TWIST_SIGN -1` are entangled — do
  not change them piecemeal.

---

## Standing rules that keep earning their keep

- Judge motion by the operator's eyes, never by odometry.
- Never turn-test for liveness — it destroys heading.
- Per-wheel disagreement on a hand push is mechanical. Nothing "slips".
- A correct register readback does not prove correct initialisation.
- A zeroed tick count is not evidence of motion — check `/fpms_health data[11]`
  (uptime) for a reboot first.
- `ROS_DOMAIN_ID=20` lives in the unit files, not `config.env`. Forgetting it
  shows an empty topic list and has produced false conclusions repeatedly.

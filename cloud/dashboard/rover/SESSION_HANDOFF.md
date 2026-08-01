# FPMS rover — takeover brief

**You are picking up mid-project. Read this, then `NAV2_BRIEF.md`, then start.**
Between them they carry everything the previous session learned, including the
things it got wrong. Where either file disagrees with your assumptions, the file
wins — every claim in them was verified against live hardware.

Written 2026-07-31 at the end of a long session.

---

## 2026-08-01: the drivetrain fault, and what slow actually means

**A single bad motor cable caused everything.** The rear-left lead was dead, so
one side dragged, the rover pivoted instead of driving, and at the firmware's
50% feed-forward duty that pivot threw three wheels off. It was never the
mission code, never `CMD_SCALE`, never Nav2, never the firmware. Hours went into
those before anyone swapped a cable.

Diagnosis order that worked, cheapest first — use it again:
1. Command straight, measure **odometry yaw drift**. Symmetric drivetrain holds
   heading; a weak side arcs. Reference points measured on this rover:
   `-62deg` = one side dead, `-3deg` = all four healthy, `+217deg` = other side
   dead. This is a real automated symmetry test and it needs no human watching.
2. Move a KNOWN-GOOD motor into the suspect port (tests the board channel).
3. Move the suspect motor into a KNOWN-GOOD port (tests motor + cable).
4. Swap only the cable (separates cable from motor).

**But that yaw test is only valid while the wiring is still.** Motor and encoder
share one connector, so moving a plug moves its encoder, and the firmware's
4-wheel kinematics then computes garbage heading — single pulses read
`-325deg`, `+260deg`. Two wrong conclusions were published from that noise
during this session, including "the M2 output stage is dead", which was false.
**When the operator's eyes and this metric disagree mid-rewire, the eyes win.**

### MOTOR LAYOUT CHANGED — turn directions are MIRRORED
Rewired 2026-08-01 while chasing the fault:

    M1 = FRONT-RIGHT   (was front-left)
    M2 = REAR-RIGHT    (was rear-left)
    M3 = FRONT-LEFT    (was front-right)
    M4 = REAR-LEFT     (was rear-right)

Firmware treats M1/M2 as one side and M3/M4 as the other, so FORWARD is
unaffected — but every TURN is mirrored versus what the mission code assumes.
Verify turn sign before trusting any heading control.

### There is no slow speed on /cmd_vel. Slow is bought with TIME.
Measured on this rover, wheels off:

    wire 0.00295 (10% of cruise) -> no motion at all
    wire 0.0295  (cruise)        -> violent
    pulse 0.25s                  -> no motion
    pulse 0.35s                  -> MOVES, ~0.18 per pulse

The firmware slams ~50% duty on ANY non-zero setpoint, so amplitude buys
nothing: 0.0010 span fast while 0.00295 did not move. The only lever is pulse
width. **Minimum pulse that moves anything is ~0.35 s** — longer than the
0.075-0.17 s the golden code used, because that commanded raw duty (instant
torque) while this commands a velocity setpoint a PID must converge to.

**EVERY number above is FREE-SPINNING, wheels off. All of it must be
re-measured under load before any of it is trusted for distance.**

### Other things that cost time this session
- **Re-zero the origin after EVERY boot.** The origin file survives reboots but
  the board's odometry restarts at 0, so a stale reference put the rover at
  (-9182, -11926) mm in a 1200 mm arena. It plans confidently from that.
- **The board went silent ~15 min after a reset, twice** — then ran over an hour
  once the drivetrain was fixed. Suspect current draw from the dragging motor,
  not a timer. If topics die, check board liveness FIRST; every downstream
  symptom looks like a software fault (blind executor, frozen poses, probes
  returning zero).
- **WiFi swung -47 to -78 dBm.** Below about -70 the LiDAR telemetry stops
  crossing entirely (small drive/mission messages still get through), and the
  mission then refuses to start because its obstacle guard is blind. That
  refusal is correct. Move the rover nearer the host.
- The LiDAR takes a round trip Pi -> laptop broker -> Pi purely because the
  broker is off-board. Running mosquitto ON the Pi removes WiFi from a
  Nav2-critical path and needs no code change, only a broker address.

---

## The single most useful thing to know

This project has repeatedly been derailed by **confident conclusions drawn from
untrustworthy measurements**. Twice, hours went into "fixing" a fault that did
not exist:

1. `/odom_raw` reports `twist.linear.x` **sign-inverted** relative to its own
   `pose.position`. This made forward motion look backward, produced a phantom
   "the chassis lurches" theory, a phantom deadband floor, and a safety guard
   wired exactly backwards.
2. The firmware's velocity loop regulates **integer encoder counts per 10 ms**,
   so it physically cannot hold a setpoint below ~0.0145 m/s on the wire. This
   made "just go slower" look like a safety measure when it was the opposite.

Neither was found by tuning. Both were found by **a test designed to
discriminate between hypotheses** — comparing twist against pose for the same
event, and reading the firmware source. When something looks impossible, suspect
the measurement before the mechanism.

Corollary that also earned its keep: **two agents refused to guess** when they
lacked hardware access, and said so. That refusal is why the right answer was
found instead of a plausible wrong one. Preserve that norm.

---

## Where things actually stand

### Working, deployed, verified
- Camera + LiDAR → MQTT → dashboard (`fpms-rover-agent`, no ROS dependency by
  design, so it survives ROS being broken)
- micro-ROS link to the ESP32 drive board, domain 20, auto-reconnecting
- `fpms-teleop`: joystick with 0.6 s deadman, nudge, turn, `test_motors`,
  `read_encoders`, beep, servo, `set_speed`, stop. Guards differentiate **pose**,
  never twist.
- Dashboard: Control tab (16 actions), Drive tab (joystick + health), LiDAR
  arena map that starts facing forward and never rotates
- Public cloud dashboard at `/live`, alerting fixed to state-change
- WiFi power-save hold (the `bcmdhd` driver silently re-enables it every
  30–60 s, which was flapping MQTT on a 47 s cycle)
- **Nav2 1.1.20 already installed** — do not reinstall

### Written, with the physical claims measured (commit `01135d2`)
- **`fpms_lidar_ros.py` + `fpms-lidar-ros.service`** — MQTT scan → `LaserScan`
  on `/scan_lidar`. Link 1 of the Nav2 chain is no longer blocking.
  Two numbers in its docstring are **real measurements taken on this rover**,
  not estimates, and are worth trusting: the publish rate (1.90 Hz at
  `FPMS_LIDAR_HZ=2`, 9.83 Hz at 10, ~5060 raw points/s either way) and the
  MQTT round-trip latency (median 100 ms, p90 179 ms). `FPMS_LIDAR_HZ=10` was
  written to `/etc/fpms/config.env` on the Pi.
  It goes through MQTT rather than the serial port on purpose — taking the port
  would kill the dashboard LiDAR view, and adding `rclpy` to `fpms-rover-agent`
  would destroy the property that makes that agent reliable. If the feed goes
  quiet it **stops publishing** instead of repeating the last scan; a frozen
  scan is worse than an absent one, because Nav2 keeps planning against a world
  nobody is observing any more.
- `nav2/` params, bringup, arena map, TF launch — verified locally only
- `fpms_missions.py` + service — defaults to `deadreckon`, see below
- `fpms_odom_tf.py` + `fpms-odom-tf.service` — `/odom` and `odom`→`base_footprint`
- `deadband_sweep.py` — fixed to use pose, never re-run since

### Not verified — the rover was powered off before any of it ran end to end
Nothing below is known-broken. It is **unproven**, which is not the same thing,
and the fastest way to waste the next session is to treat it as either.
- Whether `fpms-lidar-ros` is actually installed and enabled. The docstring and
  `deploy_rover.py` both say installed; **that was never confirmed with
  `systemctl status` while the rover was up.** Check it first, assume nothing.
- The Nav2 launch as a whole, and any mission end to end.
- The deadband has **never been measured** on corrected data.
- LiDAR mount offsets are **placeholders**. Measure them before trusting any map.

---

## The decision I would defend to whoever takes over

**Missions should default to dead reckoning, not Nav2.**

The rover's own prior code (`/home/ubuntu/fpms_phase6_LATEST.py`) drove this
arena with **0.6 % distance error** and **±1–4° turns**, with no SLAM, no AMCL
and no Nav2. Its return strategy was a *retrace*: replay the measured outbound
motions in reverse, so dead-reckoning drift cancels by construction rather than
accumulating. That is why "come back perfectly" worked.

Nav2 is worth having and is being configured. But it needs a live scan, a
complete TF tree, measured laser offsets and a map — and a Nav2 that plans
against unmeasured transforms fails in ways that are genuinely hard to debug.
Ship the proven path first, keep Nav2 selectable, and label it honestly.

**And do not try to make the rover slow by lowering the setpoint.** Below the
firmware floor the controller is blind. "Slow and controlled" on this hardware
means short bounded segments with full stops between them, run at a speed the
loop can actually hold.

---

## Operating rules that exist for a reason

1. **Never restart `micro-ros-agent`.** Costs a 90–225 s board reconnect. Hours
   were lost to this.
2. **Only one agent touches the Pi at a time.** Concurrent SSH caused a
   protocol-banner failure and corrupted a measurement run.
3. **Never publish `/cmd_vel`** unless motion is explicitly authorised, and then
   bounded, short, with zero Twist after.
4. **Do not fabricate results.** If you cannot reach hardware, say so and stop.
5. **No secrets in repo files.** This repo is public. The first draft of
   `NAV2_BRIEF.md` leaked the SSH password and had to be scrubbed — it is an
   easy mistake to make while writing helpful docs.
6. Verify with real output and paste it. `ast.parse` for Python,
   `npx tsc --noEmit` + `npm run build` for frontend.

---

## Physical state at handoff

Rover is **on blocks with the wheels removed**. Battery was ~12.2 V and drops
under load; it fell to 11.4 V during a long session. The rover is safe to put
back on the floor — the reversed-motion fault was a reporting artefact, not a
drive fault.

Windows Mobile Hotspot turns itself off when no device is connected, which cost
a 12-minute boot delay. Disable that setting or the rover does it every power
cycle.

---

## First moves for the next session

1. `git log --oneline -20` — see what landed after this file was written.
2. Read `NAV2_BRIEF.md` §3a and §3b. Do not skip them.
3. Power the rover on. Windows Mobile Hotspot must be on *first* or the Pi
   boots with no network and takes ~12 minutes to appear.
4. **Confirm what is actually running before building on it:**
   `systemctl is-active fpms-rover-agent fpms-teleop fpms-lidar-ros` and
   `ros2 topic hz /scan_lidar` (expect ~9.8 Hz). If `/scan_lidar` is silent,
   that is the whole job — nothing downstream can be tested without it.
5. Verify the TF tree with the commands in `nav2/TF_TREE.md` before launching
   Nav2. It will fail confusingly if any link is missing.
6. Re-run `deadband_sweep.py --on-blocks` on the corrected code to get a real
   floor, and only then decide whether a `MIN_CMD` is needed at all.

Steps 4 and 5 are the whole reason this file exists. The two multi-hour losses
described at the top both began by skipping a cheap check and trusting a
plausible assumption instead.

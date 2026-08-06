# Next session: convert the output stage to B8B raw duty

Written 2026-08-04. Everything below was measured on live hardware today.

## The one thing that matters

**`/cmd_vel` cannot crawl on this firmware.** Yahboom factory firmware v2.0.0
applies `PWM_MOTOR_DEAD_ZONE` feed-forward on the velocity path, so any non-zero
velocity command becomes ~50% duty ~= 0.7 m/s regardless of the number sent.

Measured today: a **correct** 744 mm plan, commanded at 0.18 m/s, drove **~2 m
and drifted left** before the 400 mm obstacle guard aborted it ~3 s in. The plan
was right, the guard fired on time, and the rover still overshot by 2.7x --
because every burst is full speed.

So `CRUISE_MPS`, `MIN_PULSE_S` and `MAX_LEG_MM` are amplitude knobs on a path
that discards amplitude. Do not tune them to go slower. This is the "firmware
problem, not a tuning problem" from the original brief.

## What to build

Replace the mission executor's **output stage only**:

- Stop publishing `Twist` on `/cmd_vel`.
- Send **raw duty over the Rosmaster framed protocol**: `0xFF/0xF8` out, `0xF7`
  back, checksum = `sum % 256`, duty `-100..100`. At 26/100 the rover really
  does 26% -- this is how B8B crawled and reached 0.6% distance accuracy.
- Measure distance from odometry **between bursts**, at rest, exactly as B8B did.
- Reuse B8B's proven structure: open-loop constant duty (26 drive / 50 turn),
  20 Hz Pi-side loop, gyro P-heading trim (gain 10, clamp +-6), turn cut at 93%
  then coast then MEASURE, measured retrace home.

Reference: `golden_backup/` and `GOLDEN_B8B_STUDY.md` in this directory.

## The hard constraint that shapes the design

**Raw duty and micro-ROS cannot share the serial port.** Proven in an earlier
session: baud conflict (921600 vs 115200), two readers on one tty, `TIOCEXCL`,
and the board only answers the framed protocol ~5 s after reset.

So moving to raw duty also means sourcing odometry/gyro/battery through the
framed protocol, as B8B did. This is an architecture change, not a config edit.
Decide deliberately: either the framed protocol owns the port (B8B model), or
micro-ROS does (today's model) -- not both.

## What is already done and should NOT be rebuilt

- **Planner layer is finished and gate-verified.** Theta* any-angle with turn
  cost (one 90 deg turn = 391 mm of driving, measured), clearance gradient,
  hysteresis, fallback ladder, occupancy grid from live LiDAR with centroid
  tracking, and `ObstacleDetour` live rerouting. 168 checks pass; the clear-arena
  m2 gate holds at exactly **744.0 mm / 10 segments / 11 waypoints**.
  `FPMS_MISSION_PLANNER=theta|astar|straight`.
- **D\* Lite was evaluated and rejected with a measurement**, not an opinion:
  worst-case A* on this Pi is 1.2 ms and the costmap rebuild it cannot avoid is
  also 1.2 ms. It optimises the half that does not dominate. Revisit only if the
  grid grows a lot.
- **micro-ROS link fix**: the ESP32-S3 boots on the **EN rising edge**; the
  agent's `open()` leaves handshake lines at a static level with no edge, so the
  app never restarts. "A level is not a reset." Fixed by a sequenced app-mode
  reset (IO0 high, EN low, hold 150 ms, release) in
  `/usr/local/bin/fpms-uros-release-reset`.
- **LiDAR staleness fix**: publish was gated on `and points_seen`, so a stopped
  scanner published nothing and the dashboard held its last frame forever. Now
  always publishes with `scan_age_s` / `health` / `stale` / `hz`, and `hz` is
  forced to 0.0 when stale rather than a cached number.
- `fpms_ros_tunnel.py` and `fpms_rtos_follower.py` exist, installed and
  **disabled**. NOTE: the follower emits Twist and assumes a linear duty-speed
  curve -- that mapping is meaningless on this firmware and needs the same
  rework as the executor.

## Open issues

- **micro-ROS link is intermittent.** Recovers reliably when `micro-ros-agent` is
  restarted (which fires the EN pulse), but drops again after some minutes with
  `NRestarts: 0` and no dmesg disconnect. Board goes quiet under a healthy
  descriptor. Suspect marginal USB connection or board-side brownout.
  **After any board power cycle, restart `fpms-teleop` / `fpms-odom-tf` /
  `fpms-missions`** -- the XRCE participant is replaced and their subscriptions
  do not re-match.
- **Pi DHCP address moved 5x today** (.248 -> .184 -> .213 -> .86). Set a static
  lease; it repeatedly broke SSH and the dashboard's broker connection.
- **Wi-Fi RTT to the laptop is ~108 ms** on a direct hotspot -- should be 2-5 ms.
  With the QoS 0 bridge (drops rather than queues) this loses ~70% of telemetry.
  Move the rover closer / check the 2.4 GHz channel.
- **Windows Firewall has no inbound rule for 1883**, so the Pi's bridge cannot
  reach the laptop. Worked around by pointing the dashboard OUTBOUND at the Pi
  (`FPMS_MQTT_HOST` in `Start-FPMS-Dashboard.cmd`) -- update that when the Pi's
  IP changes, or add the rule as admin:
  `New-NetFirewallRule -DisplayName 'FPMS MQTT' -Direction Inbound -Action Allow -Protocol TCP -LocalPort 1883 -RemoteAddress 192.168.137.0/24`
- **LiDAR mount yaw/rotation sign is still UNVERIFIED.** Both the cone guard and
  the occupancy grid inherit that error by the same angle in the same direction.
  Measure it before trusting any detour.
- Rear-right wheel was slipping earlier (950 mm vs 740-755 mm on others).

## Safety notes

At 12+ V the low-voltage interlock (11.1 V) no longer refuses missions, and with
a live link the missing-pose refusal is satisfied too. The **arm latch is the
only remaining gate**. Keep the operator watching, hand ready, for every run.

Judge motion by the operator's eyes, never by odometry -- today's run reported
`moved 1533.6 mm` at pose `(4453, -1969)` for a rover that had gone ~2 m
forward, because `/odom_raw` switched from IDENTITY to a real yaw mid-mission.

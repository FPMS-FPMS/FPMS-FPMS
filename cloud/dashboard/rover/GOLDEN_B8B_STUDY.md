# What the golden code (B8B / phase6) actually does — and why it works

Studied 2026-08-04 from `fpms_phase6_LATEST.py` after a long session of failures
with a closed-loop velocity-PID firmware. **The firmware must be made to match
THIS.** Live path is `_p5_navdrive` (1307-1516) + `odom_loop` (841-883);
everything after `app.run` at 2395 is dead code.

---

## THE ROOT MISMATCH

**B8B commands RAW DUTY. It never commands a velocity.**

    _DRV = 26      # 26% duty out of +/-100
    _TRN = 50      # 50% duty for turns
    bot.set_motor(pwr-c, pwr-c, pwr+c, pwr+c)

A square wave: 0 -> 26 -> 0. No velocity setpoint, no speed PID, **no ramping**.

The linorobot firmware we flashed takes `cmd_vel` and runs a float-RPM PID. That
PID is what ran away on inverted encoders, produced phantom distances, and drove
the rover into a wall. Open-loop duty tests on the same hardware gave **0.3%**
distance error, repeatably. The architecture, not the tuning, was wrong.

## THE FOUR PROPERTIES TO REPRODUCE

### 1. The control loop is on the PI, at 20 Hz
`_fwd` re-issues duty every 50 ms with a P-only gyro heading trim
(`_KPH = 10`, hard-clamped to +/-6 counts, i.e. +/-23% differential). The board
is a dumb duty amplifier. **This matters because the obstacle guard needs the
LiDAR, and the LiDAR is on the Pi.**

### 2. It plans fully before moving, and stops the motors first

    gates (robot_enabled, marker locked, home locked)
      -> plan_route()          A* + Douglas-Peucker, BEFORE opening the board
      -> open_bot()
      -> stop_bot()            <-- first hardware command is a STOP
      -> plan_route() again
      -> build segments, DROP any leg < 30 mm
      -> LOG THE ENTIRE PLAN
      -> only now: _spin()

### 3. Five guards, evaluated EVERY tick, in this exact order (1355-1362)

    1. stop_requested            -> stop, "STOP"
    2. arrival (marker dist)     -> stop, "ARRIVED"   (400mm zones, 5mm home)
    3. front LiDAR < 120 mm      -> stop, "BLOCKED"   (forward only; +/-25 deg cone)
    4. distance reached          -> stop, "DONE"
    5. 12 s per-move timeout     -> stop, "TIMEOUT"

The obstacle guard runs **20 times a second, before every duty command**. A
firmware-side control loop cannot do this — it cannot see the LiDAR.

Note `_fr()` returns 9999 when front_raw is missing: **fail-open**. And `_spin`
has NO front guard and NO timeout — a dead gyro spins forever.

### 4. Slowness comes from STRUCTURE, not just low duty
- low constant duty (26/100)
- **short bounded bursts with mandatory full stops between them**
  (final approach chunks at `min(500, dist - stop_at + 80)` mm, max 15 attempts)
- deliberate dwells: 250 ms coast-settle after every spin, 600 ms (+1000 ms if
  the marker is lost) after a >45 deg spin, 300 ms between segments, 400 ms in
  retrace
- turns cut at 93% of target (`_COAST`) and coast the rest, then integrate for a
  further 250 ms to measure and return the **actual** degrees turned

## LIVE CONSTANTS (1309-1311)

    _TKMM = pi*70/1320   mm per tick
    _DRV  = 26           drive duty  (of 100)
    _TRN  = 50           turn duty
    _COAST= 0.93         cut turns at 93%, momentum finishes
    _HZ   = 20           control rate
    _FSTOP= 120          front stop, mm, LiDAR-referenced
    _KPH  = 10           heading P-gain, clamped +/-6
    TURN_SIGN = -1       (line 178)
    B6_TARGET_STOP_RAW = 400   B6_HOME_STOP_RAW = 5

## FAILURE BEHAVIOUR

| condition | action |
|---|---|
| `stop_requested` | stop, unwind, `stop_bot` twice + pump off |
| `NO_ENC` | return 0 mm, sleep 0.3 s, **continue to next segment** |
| `BLOCKED` | stop, log, **skip the segment and continue** (no reroute) |
| `BLOCKED` in final approach | stop, mission fails |
| 12 s `TIMEOUT` | stop, reports the REQUESTED mm as driven (odometry optimism — a known bug) |
| any exception | `stop_bot`, state ERROR, 18 s cooldown |

## THE FIRMWARE CHANGE THIS IMPLIES

Expose **raw per-side duty** (matching `set_motor`) and nothing else: duty in,
encoder counts out. No PID, no velocity, no odometry integration on the board.
Then port B8B's 20 Hz loop to the Pi unchanged, where it can see the LiDAR.

That deletes the entire class of bug from this session: no velocity loop to run
away, no encoder-sign positive feedback, no odometry that can lie while the
chassis spins.

## MEASURED HARDWARE FACTS TO CARRY OVER (verified this session)

- RIGHT-side motors are wired opposite the left -> right pair needs inverting
- Only the LEFT encoders are inverted
- ~14.8 counts/mm (operator tape-measured over 800 mm)
- open-loop crawl that works: base duty 70 with per-wheel trims
  `{0.820, 0.839, 0.914, 1.000}`, kick-start duty 140 for 220 ms (without the
  kick the motors only buzz and nothing moves)
- coast compensation: 1270 counts forward, 1448 reverse
- serial must be 230400; `FPMS_LIDAR_HZ=2`; the board dies when a ROS node tears
  down

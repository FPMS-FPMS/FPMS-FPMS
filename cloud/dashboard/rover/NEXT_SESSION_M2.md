# NEXT SESSION: MISSION 2

**Goal: M2 — plan the route, crawl straight to Zone B, dwell, come back.**
No turning is needed for M2: zone-b is straight ahead of the start box.

Read `SESSION_2026-08-03.md` for the full history. This file is the short list.

---

## DO THESE TWO THINGS FIRST — both are physical, neither is software

1. **CHARGE THE BATTERY.** It ended the last session at **11.58 V**, down from
   13.75 V. Below ~12 V the numbers stop being trustworthy, and worse, every
   calibrated constant below was measured at a specific voltage — as the pack
   sags the same duty makes less torque, the trims drift and the coast shortens.
   Some of the turn scatter was probably this.

2. **TIGHTEN THE M2 REAR-RIGHT WHEEL.** It slipped through the last runs:
   `M1 746.9, M2 950.7, M3 755.4, M4 739.0 mm` — three wheels agree within
   16 mm and M2 is 200 mm beyond them, creeping up run over run
   (847 → 817 → 835 → 950). Same failure as the wheel already fixed once.
   **Do not tune turns until this is fixed** — the left turn scatters because a
   left turn is pushed by the right-side wheels.

---

## CALIBRATION — measured, operator-verified, ready to use

All from open-loop tests the operator watched. NO velocity PID: that loop ran
away and drove the rover into a wall. Straight legs terminate on encoder counts,
turns on the gyro — the same shape as the golden phase6 driver.

    COUNTS_PER_MM   14.8          (operator tape-measured over 800mm)
    BASE_DUTY       70
    TRIM (fwd)      {0.820, 0.839, 0.914, 1.000}   M1,M2,M3,M4
    TRIM_REV        {0.820, 0.737, 0.894, 0.960}   reverse imbalance differs
    KICK            duty 140 for 220ms   (without it the motors only buzz)
    COAST_FWD       1270 counts          (kick + coast overshoot)
    COAST_REV       1448 counts
    TURN_DUTY       95
    TURN_COAST_L    38 deg   <- still scattering, blocked on the M2 wheel
    TURN_COAST_R    27 deg   <- repeatable at -85 to -87

    Motor polarity: RIGHT side wired opposite -> MOTOR2_INV, MOTOR4_INV = true
    Encoder signs:  LEFT side inverted -> MOTOR1_, MOTOR3_ENCODER_INV = true
    Wheelbase:      105 mm (operator-measured; fpms_config.h still says 170 --
                    fix before trusting any turn geometry)

### Achieved on the last good run

    FORWARD 800mm -> 797.8 mm   (0.3% error)   spread 10%
    BACK    800mm -> 797.4 mm   (0.3% error)   spread  8%
    TURN RIGHT -90 -> -86.9 deg (repeatable)
    TURN LEFT  +90 -> 71.7-95.8 deg (SCATTERING - M2 wheel slip)

Straight-line beats the golden driver's 0.6%.

---

## THE RULE THAT MATTERS MOST

**The odometry has lied convincingly, more than once.** Two M2 runs were reported
as successes — 0.35% distance error, 18.9mm homecoming — while the rover was
spinning on the spot. Inverted encoder feedback made the sensors agree with each
other and with nothing real.

- Verify motion by **asking the operator to watch**, never by a sensor reading.
- Any new diagnostic must carry a **control value whose answer is already known**
  (e.g. WHO_AM_I). Two gyro diagnostics returned all-zeros for reasons that were
  entirely artefacts of the instrument; the control channel is what caught both.
- **Never use a turning test as a liveness check** — heading has no absolute
  reference here and the static map->odom is pinned to the start pose.

---

## OPERATIONAL FACTS

- **The board dies when a ROS node tears down.** Reset → agent → mission with NO
  ROS node in between. `tools/m2_tight.sh` is that sequence.
- Cold boot needs a **DTR/RTS pulse**; the board does not start its session on
  its own.
- **Serial is 230400**, not 921600 — 921600 was unstable and died in 1–3 min.
  Both ends must match: `BAUDRATE` in `fpms_config.h` and `-b` on the agent unit.
- `FPMS_LIDAR_HZ=2`. At 10 Hz the LaserScan traffic starved the agent and killed
  the link.
- **In micro-ROS the CLIENT declares its DDS domain.** Getting it wrong gives a
  flawless-looking agent session with `Publisher count: 0` on every topic.
- `fpms-missions` is **disabled at boot on purpose** — it is dashboard-triggerable
  and must not launch a mission before the drivetrain is trusted. Enable it only
  when ready.
- `fpms-odom-tf` holds stale subscriptions across a board session change and must
  be restarted after any board reset, or it sits frozen.

## ALREADY WORKING, LEAVE ALONE

- 8 services enabled at boot: micro-ros-agent, fpms-teleop, fpms-rover-agent,
  fpms-lidar-ros, fpms-odom-tf, fpms-tf, fpms-map-odom, mosquitto
- Mosquitto runs ON the Pi with a bridge to the laptop at `192.168.137.1:1883`,
  `topic # both` — telemetry streams both ways on every boot with no action
- Battery reaches the dashboard (`battery_v`). The fix was `fpms_teleop.py`
  subscribing `std_msgs/UInt16` when the firmware now publishes
  `sensor_msgs/BatteryState` — a type mismatch matches nothing SILENTLY: the
  topic looks alive and the value stays None forever
- Dashboard is a native installed app that auto-starts at login
- TF tree complete: map → odom → base_footprint → base_link → laser_frame

## M2 GEOMETRY

Arena 1200x1200, origin bottom-left. Start (972, 228) heading 90deg.
Zone B centre **(972, 972)** — straight ahead, **turn 0deg, drive 744 mm**.
Zone centres sit 228 mm from two walls, so use a standoff (~180 mm) or the
400 mm front guard aborts at the goal. FRONT_STOP is measured FROM THE LIDAR,
which sits ~160 mm behind the nose.

## SUGGESTED ORDER

1. Charge battery, tighten M2 wheel.
2. Re-run `tools/run_moves.sh` at full voltage: forward, back, both turns.
   Confirm the left turn stops scattering.
3. If straight-line still lands ~0.3% and turns are repeatable, build M2 from
   these primitives — it is forward + dwell + reverse, nothing more.
4. Only then re-enable `fpms-missions` and drive it from the dashboard button.

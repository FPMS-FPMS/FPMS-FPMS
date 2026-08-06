# FPMS v2 architecture — built on B8B, not against it

Decided 2026-08-04 after a session where a closed-loop velocity-PID firmware
drove the rover into a wall twice. See `GOLDEN_B8B_STUDY.md` for the evidence.

## THE ONE RULE

**Intelligence lives on the Pi. The ESP32 is a real-time duty amplifier.**

Everything that failed this session failed because policy lived on the board:
a velocity PID that could wind up, board-side odometry that could lie, and a
control loop that could not see the LiDAR. B8B put all of that on the host and
achieved 0.6% distance error and +/-1-4 deg turns.

    ESP32 (FreeRTOS)          |  Orange Pi (ROS 2 Humble)
    --------------------------|--------------------------------------------
    duty in -> motors         |  planner (Nav2 global + B8B executor)
    encoders + gyro out       |  20 Hz control loop, gyro heading trim
    deadman: 300 ms -> STOP   |  LiDAR obstacle guard (checked every tick)
    NO pid, NO odometry,      |  odometry, TF, map, mission state
    NO planning, NO velocity  |  MQTT telemetry -> dashboard

**The obstacle guard MUST be on the Pi.** The LiDAR reaches ROS via MQTT; a
board-side loop is blind. B8B checked front distance 20 times a second before
every duty command — that is the check that stops a wall hit.

## LAYERS, IN BUILD ORDER

Each layer is testable on its own. Do not start the next until the current one
is verified **by the operator watching**, never by a sensor reading — the
odometry on this rover has reported clean travel while the chassis span in place.

### L0 — ESP32 duty amplifier  (`firmware_v2/fpms_drive_rtos.ino`)  [WRITTEN]
FreeRTOS. `motorTask` at 100 Hz is highest priority and the only writer of the
motor pins. `sensorTask` at 50 Hz. Deadman: no command in 300 ms -> all stop.
Motor commands never in `loop()`.
**Test:** duty command from the Pi moves the rover; cutting the command stops it
within 300 ms.

### L1 — duty transport (Pi <-> ESP32)
micro-ROS at 230400. Pi publishes per-motor duty already sign-corrected; board
publishes encoder counts, gyro Z, battery.
**Test:** commanded duty appears on the wheels; counts come back with the right
signs (right +, left -, on forward).

### L2 — B8B motion primitives, on the Pi
Port `_fwd` and `_spin` unchanged in shape: 20 Hz loop, constant duty, P-only
gyro heading trim clamped +/-6, turn cut at 93% then coast and MEASURE.
Add two guards B8B lacked, both of which caught real faults this session:
- **slip detector**: wheels disagreeing by >55% after the start transient is a
  loose wheel, not travel
- **spin timeout**: B8B's `_spin` had none; a dead gyro spun forever
**Test:** forward 800, back 800, turn +90, turn -90. Target 0.3% and +/-5 deg.

### L3 — planner (upgraded from B8B's A*)
B8B: A* on a 100 mm grid, Douglas-Peucker simplification, obstacle inflation
`ROUTE_RADIUS = 205`, legs under 30 mm dropped, whole plan LOGGED BEFORE MOVING.
Upgrade: use Nav2's global planner over the prebuilt arena map, keep B8B's
segment executor underneath. Nav2 plans; B8B drives.
**Test:** plan renders on the dashboard before the wheels move.

### L4 — dashboard
Arena map is already world-fixed and never rotates (`ArenaMap.tsx`). It needs a
real pose: publish `x_m`, `y_m`, `heading_deg` to
`fpms/<thing>/telemetry/pose` and the "POSE ASSUMED" badge clears with NO
frontend change (`lib/arena.ts:431-508`).
Then move the mission buttons onto the LiDAR/arena view: PLAN and FOLLOW for
m1 / m2 / water / home, plus re-zero. They already exist in `Drive.tsx:1300`.

### L5 — missions
M2 first: zone-b is straight ahead of the start box, so turn 0, drive 744 mm
less a standoff. Then M1 (+45 deg turn), then the water dock.

## MEASURED CONSTANTS — carry these forward, do not re-derive

    counts/mm            14.8        (tape-measured over 800 mm)
    drive duty           26/100      B8B, or 70/255 open-loop measured here
    turn duty            50/100
    turn coast           93% then measure the actual
    control rate         20 Hz
    front stop           120 mm B8B / 400 mm current config (LiDAR-referenced;
                         the LiDAR sits ~160 mm behind the nose)
    heading gain         P=10, clamped +/-6
    per-move timeout     12 s
    max burst            500 mm, full stop between segments
    right motors         wired opposite -> invert
    left encoders        inverted
    serial               230400 (921600 died in 1-3 min)
    FPMS_LIDAR_HZ        2 (10 Hz starved the agent and killed the link)

## OPERATING RULES EARNED THE HARD WAY

- Verify motion by **asking the operator what they saw**. The odometry has lied
  convincingly and self-consistently.
- Any diagnostic must carry a control value whose answer is already known.
- Never use a turning test as a liveness check — heading has no absolute
  reference here.
- The board dies when a ROS node tears down: reset -> agent -> run, with no ROS
  node in between.
- Never put motor commands in `loop()`.

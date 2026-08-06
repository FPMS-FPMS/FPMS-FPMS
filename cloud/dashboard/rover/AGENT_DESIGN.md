# The Pi-side agent — planner + executor design

Designed 2026-08-04. Implements `ARCHITECTURE_V2.md`. Nav2 plans, B8B drives.

## Five SEPARATE processes (not one executor)

The 20 Hz control loop must not share a process with a planner callback or an
MQTT publish, or it can be starved.

| process (`rover/agent/`) | rate | owns |
|---|---|---|
| `fpms_board_io.py` | pass-through | sign-corrects duty; republishes `/enc_counts`, `/gyro_z`, `/battery`. **Never repeats a duty command** — the 300 ms deadman must only ever be fed by a live control tick |
| `fpms_obstacle_guard.py` | 20 Hz | min range in the +/-25 deg nose cone -> `/guard/front_mm` |
| `fpms_segment_executor.py` | **20 Hz, own process, nice -10** | the ONLY publisher of `/motor_duty`; action server `/fpms/execute_segments` |
| `fpms_route_planner.py` | on demand | Nav2 `ComputePathToPose` client; path -> segments |
| `fpms_mission.py` | 5 Hz | state machine, retrace stack, MQTT telemetry |

Nav2 lifecycle: `map_server`, `amcl`, `planner_server` ONLY.
**`controller_server` and `bt_navigator` are NOT launched** — nothing but the
executor may reach the motors.

## Executor: tick state machine, not a while-loop

One mandatory structural change from B8B: `_fwd`/`_spin` become tick state
machines (`IDLE / TURNING / TURN_COAST / DRIVING / DWELL`), because a blocking
loop inside a ROS callback starves everything else. Shape and constants are
otherwise ported verbatim:

    TKMM  DRV=26  TRN=50  COAST=0.93  HZ=20  FSTOP=120
    KPH=10 clamped +/-6   MOVE_TIMEOUT=12s   BURST_MAX=500mm

### Guard order, EVERY tick
    1. stop_requested            -> STOP
    2. arrival (dist <= stop_at) -> ARRIVED
    3. front < FSTOP  OR reading older than 0.5 s -> BLOCKED
    4. slip detector             -> SLIP
    5. distance done             -> DONE
    6. t > 12 s                  -> TIMEOUT (reports MEASURED mm)

Then, and only then, compute the heading trim and publish duty.

**Two corrections to B8B, both from faults seen this session:**
- Guard 3 **fails CLOSED**. B8B's `_fr()` returned 9999 when the LiDAR was
  missing — it failed OPEN, and would drive blind.
- TIMEOUT reports **measured** distance. B8B reported the *requested* distance,
  which silently banked travel that never happened.

## The two guards B8B never had

- **Slip detector** — after a 300 ms start transient, if
  `(max_wheel - min_wheel) / mean > 0.55` for 3 consecutive ticks, abort SLIP.
  This is exactly the M2 rear-right signature: 950 mm against three wheels at
  740-755 mm.
- **Spin timeout** — `deadline = max(4.0, 2.5 * expected_s)`, plus abort if
  `|gz| < 0.02 rad/s` for 1 s while turn duty is commanded (dead gyro). B8B's
  `_spin` had NO timeout: a dead gyro spun forever.

## Genuine upgrades over B8B

1. **NavFn over the prebuilt `arena_map.pgm`** instead of A* over a costmap
   synthesised from live LiDAR clusters. The walls are known exactly; B8B
   rediscovered them each plan and could plan into a wall it had not scanned.
2. **Real-footprint inflation** (`robot_radius 0.15`, `inflation 0.20`) instead
   of B8B's flat 205 mm disc.
3. **Replan on BLOCKED** — B8B skipped the blocked segment and drove on, which
   is how a blocked rover ends up somewhere unplanned. Now: stop, stamp the
   obstacle, replan up to 3x, then HOLD.
4. **Plan logged and rendered before motion**, gated by `plan_id`.
5. **Retrace-home preserved verbatim** — a reversed stack of MEASURED
   (turn, mm) pairs beats a fresh plan from a drifted pose.

## Deliberately REFUSED

No `controller_server`/RPP/DWB driving the wheels — a 26%-duty skid-steer cannot
track a carrot. No online SLAM (its pose comes from the same drifting gyro the
map exists to correct). No continuous replanning mid-burst. No velocity
smoother. **Reintroducing a velocity concept anywhere is the failure mode.**

## Failure behaviour

| abort | rover does |
|---|---|
| STOP | zero duty twice, pump off, drop plan, IDLE |
| NO_ENC / counts stale >0.5 s | stop, **fail the segment** (B8B continued — do not) |
| BLOCKED | stop, stamp obstacle, replan <=3x, else HOLD |
| BLOCKED in final approach | stop, mission FAILED |
| SLIP | stop, FAILED, log all four per-wheel mm — mechanical fault |
| TIMEOUT | stop, report measured mm, fail the segment |
| SPIN_TIMEOUT | stop, FAILED, flag "gyro suspect" |
| guard topic stale | treated as BLOCKED |
| executor process dies | deadman stops the rover within 300 ms |

## Test plan — every step verified by the OPERATOR WATCHING

- L0 duty + deadman: wheels turn; kill the publisher -> stop within 300 ms
- L1 signs: operator pushes the rover by hand, counts must move the agreed way
- L2a `_fwd`: chalk mark + tape measure, 800 mm out and back
- L2b `_spin`: protractor on the floor, +/-90 deg
- L2c guard: operator holds a board in the nose cone mid-burst -> BLOCKED
- L2d slip: operator grips one wheel -> abort SLIP, not "completed"
- L2e spin timeout: unplug the gyro -> abort within 4 s
- L3 planner: **rover on a stand, wheels off the ground**; plan renders on the
  dashboard and the operator confirms it matches the arena
- L4 M2 (straight, one leg), operator walking alongside with a hand on stop
- L5 M1 (+45 deg), water, home retrace

## Build order

    b8b_constants.py -> fpms_board_io.py -> fpms_obstacle_guard.py
      -> fpms_segment_executor.py   [L2 GATE: all five L2 tests signed off]
      -> fpms_route_planner.py -> fpms_mission.py -> dashboard pose/buttons

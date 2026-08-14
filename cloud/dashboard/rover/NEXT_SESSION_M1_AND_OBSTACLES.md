# Next session — M1, and obstacles

**M2 completed out-and-back on 2026-08-14 at 01:32.** This is the first
end-to-end mission on the STM32 board. Start here, do not re-derive it.

```
01:31:48 mission 'm2' accepted: (1322,178)mm h=90deg -> (1365,1265)mm, 2 planned segments, retrace home
01:31:54 crawl FWD asked +340mm at duty +18 -> ticks +252mm, odom +222mm, drift +0.5deg, 1.84s (arrived (lidar))
01:31:54 DOCKED on the LiDAR at 350mm scanner-to-target; the encoders still had 88mm to run.
         This is an ARRIVAL: the mission continues into the hold and the return.
01:32:01 crawl BWD asked -1001mm at duty -24 -> ticks -1026mm, odom -1004mm, drift +0.5deg, 4.68s (done)
01:32:01 mission 'm2' completed: done; 2028mm travelled, 3 segments, 13.1s
```

## What this run MEASURED (these were guesses before — use them)

| Quantity | Measured | Was |
|---|---|---|
| Crawl speed @ duty 18 | **~137 mm/s** (252 mm in 1.84 s) | `CRAWL_CAP_MPS=0.020`, `CRAWL_MIN_MPS=0.012` — **both ~10x too low** |
| Crawl speed @ duty 24 | **~219 mm/s** (1026 mm in 4.68 s) | " |
| Heading hold over 1 m | **+0.5 deg**, both directions | unknown |
| Ticks vs odom, forward | 252 vs 222 mm (13% apart) | — |
| Ticks vs odom, reverse | 1026 vs 1004 mm (2% apart) | — |
| Duty forward sign | **+1**, measured (+18 -> +129.7 mm, all four encoders +) | golden's guess |
| Tick scale | **0.16657 mm/tick** validated: 798.5 ticks x 0.16657 = 133.0 mm vs odom 129.7 mm | golden's value |

**First job next session:** set `FPMS_MISSION_CRAWL_CAP_MPS` / `CRAWL_MIN_MPS`
from the numbers above. They currently make the crawl guards ~10x more generous
than they should be — safe, but they are not doing their job.

The 13% forward tick-vs-odom gap is worth one look. Reverse agreed to 2%, so
suspect the forward burst's ramp (odom integrates the board's reported velocity,
which lags during acceleration) rather than the encoders.

## M1 — what is actually different

M2 is **straight ahead**: (1322,178) -> (1365,1265), bearing ~87 deg, no turn.
M1 is (135, 1265): a **~50 degree turn then ~1.6 m**. M2 exercised none of the
turn path, so M1 is the first real test of it.

**The one thing most likely to bite:**
`TURN_WIRE_SIGN = -1` is **DERIVED from the 2026-08-01 rewiring, NOT measured.**
The startup banner says so. A wrong value cost a whole session already — the
rover reached M2, turned the wrong way, and the guard aborted:
`asked -12.6deg, the rover rotated +8.1deg — the OPPOSITE way`.

Verify it cheaply before trusting a long M1 run:
1. Arm, command a small mission that needs a turn, and watch the FIRST rotation.
2. `_turn`'s wrong-way guard aborts past `TURN_WRONG_WAY_DEG` (8 deg), so a
   wrong sign costs one aborted segment, not a spin. Let it abort; do not
   "help" it.

**Turns still run on `/cmd_vel`, not duty.** The crawl (`_drive_crawl`, raw duty,
encoder-closed, gyro-held) covers DRIVES only. Turns go through the velocity
path with its dead zone and ramp — the same path whose short bursts kept
aborting before the ramp allowance was added. Expect turn accuracy to be the
weakest link on M1. Porting the turn to duty is the obvious next build:
golden's `_spin()` (`fpms_phase6_M1_M2_WORKING.py:1289-1310`) is the reference —
`set_motor(-p,-p,+p,+p)`, integrate the gyro until `rad*_COAST`, then measure
the coast while it settles.

Also unmeasured and only exercised by turning: **track width** (three live
values, 0.170 / 0.105 / "equal to wheelbase", none measured) and `gyro_scale`.

## Obstacles — the mechanism exists, it has never met a real obstacle

Golden's idea is now implemented (`planning_obstacles` -> our grid):

- `TARGET_EXCLUDE_MM = 220` — returns within 220 mm of the leg's own target are
  not obstacles. **This is what made docking work.** Without it the rover aborts
  on the thing it is docking against.
- `SELF_FILTER_MM = 140` — ignore returns from the chassis itself.
- Arena clip — discard returns outside the 1500x1400 box as room clutter.
- Route: straight when nothing blocks, Theta*/A* only when something does.
- `DOCK_FRONT_LIMIT_MM = min(FRONT_STOP_MM, DOCK_MIN_CLEAR_MM)` — one expression
  owns the dock-vs-guard decision; `DOCK_MIN_CLEAR_MM = 200` never lifts.

**What to test, in order:**
1. **Obstacle NOT on the path** — off to one side. The rover should ignore it
   and drive straight. Confirms the self-filter and arena clip are not
   manufacturing phantoms.
2. **Obstacle ON the path, well before the target.** It is NEARER than the
   target, so it fails the exclusion test, keeps the full 400 mm guard, and
   should trigger a reroute. Watch for `ObstacleDetour` in the journal, and
   check the grid actually believes it: `occupancy.cells` in
   `telemetry/mission` must be non-zero. A reroute that fires with `cells: 0`
   means the cone saw something the grid never confirmed — that combination
   produced the "no clear route at any padding" aborts.
3. **Obstacle just short of the target**, inside 220 mm. It will be EXCLUDED
   and the rover will dock against it. That is by design, and it is the
   limitation to know: the system cannot distinguish "my docking target" from
   "an obstacle parked on my docking target".

## Known-open, carried forward

- **No localisation.** `try_arena_fix()` wants a live `map->odom`.
  `slam_toolbox 2.6.10` is installed and was proven to publish one on this
  hardware, but no room map exists and `fpms-tf.service` holds that edge with a
  STATIC transform (static transforms carry `stamp = 0`, so the executor
  rejects them as ~1.79e9 s stale). Fix: rename the static edge to
  `map->slam_map` so slam owns `map->odom`, per the TF note in the session doc.
- **`base_footprint -> laser_frame` is a placeholder** `[0,0,0.1]`, x/y/yaw all
  zero, still MEASURE ME. 1 deg of mount yaw ~ 10.5 mm of error.
- **`set_coordinate` has a read/write race**: the first RESET POSE re-anchors
  from a STALE read of the origin file and leaves the pose ~500 mm off; the
  second press is correct. **Press it twice**, or fix the race.
- **The board resets its own odometry** occasionally (`/odom_raw RESET to the
  origin`), seen three times on 2026-08-13. Re-anchor if pose looks wrong.
- **`telemetry/residual` is documented and subscribed but never published.**
  Read residuals from `events/mission_done` -> `measured[]`.
- **The Windows dashboard needs a rebuild** to see `telemetry/board`:
  `backend.mqtt_bridge` is frozen into the exe by PyInstaller.
  `Build-Windows.ps1` then `Install-App.ps1 -Force -Launch`. No React page
  consumes the `board:rover2` channel yet; the Pi console at `:8090` shows it.
- **The Pi's IP moves** across reboots (was .176, now **192.168.137.150**).

## Do not undo these

- `ARRIVE_TOL_MM` is floored at `MIN_MOVE_MM`. A tolerance tighter than the
  smallest motion the chassis can express is unsatisfiable and produces an
  arrival oscillation that looks like a random 45 deg turn.
- `_cfg_float` returns the DEFAULT on out-of-range, it does not clamp. A config
  value outside a constant's range is silently ignored — this bit
  `MAX_LEG_MM=2000` (ceiling was 600, so it fell back to 300) and
  `DOCK_STEP_MM=250` (range [15,150], fell back to 70). **Check the ceiling
  whenever a config change appears to do nothing.**
- `burst_cap_s` must keep its additive ramp term. Do NOT "fix" a burst-cap abort
  by lowering `FULL_DUTY_MPS`: that constant must OVER-estimate chassis speed or
  the bound means nothing.
- The dock terminator runs at the TOP of the crawl loop, before `check_abort`.
  Putting it after lets the guard consume the reading first — that was the bug.

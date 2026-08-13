# Planning — how the rover gets to a zone, and why it is not D\* Lite

FPMS-OS ships a rover that plans its own route to any of three arena zones on
first boot. This document says what the planner does, where the zones come from,
what the telemetry tells you, and — at length, with arithmetic — why D\* Lite is
still not in it.

Everything here is about `fpms_missions.py`, which was **not rewritten** when the
stack was reorganised. It is gate-verified (m2 = 744.0 mm) and four surgical
additions were made to it instead, proven planner-neutral by `test_stack.py`.
Nothing in this document proposes changing it.

---

## 1. What the planner already does

Given a target, the executor plans its own route. There is no recorded path, no
teach-and-repeat, and no joystick anywhere in the loop.

**Theta\* over a live LiDAR occupancy grid.** `FPMS_MISSION_PLANNER` selects one
of three, in increasing order of how much they are allowed to do
(`fpms_missions.py:1730-1750`):

| setting | what it does |
|---|---|
| `straight` | the pre-planner behaviour exactly: no grid consulted, no detour possible, obstacle means abort. The bail-out switch. |
| `astar` | 8-connected grid A\* with the string-puller, as first shipped. Kept as a first-class setting, not dead code — it is the field fallback if `theta` misbehaves, and both are tested. |
| `theta` | **the default.** Any-angle search: a leg may point anywhere, not only at 45° steps. |

The reason `theta` is the default is on this chassis, not in the literature. One
90° turn costs this rover about as much time as **391 mm of driving**, so a
planner that can only turn in 45° increments is paying the most expensive thing
it owns for the privilege of staying on the grid.

**The straight line is still the plan.** `plan_detour` asks one question first:
is the straight line clear in the grid as it stands? Only when the answer is no
does it search (`fpms_missions.py:3094-3097`). On a clean arena the output is
`plan_route`'s output, segment for segment — which is exactly what the m2
regression checks.

**Turn cost is derived, never typed** (`fpms_missions.py:1804-1867`):

```
LEG_EFFECTIVE_MM_S   = MAX_LEG_MM / (MAX_LEG_MM/1000/CRUISE_MPS + STOP_SETTLE_S)
                     = 70 / (0.070/0.18 + 0.45)          = 83.4 mm/s
one 90° turn         = (pi/2)/TURN_RADPS + TURN_SETTLE_S = 3.49 + 1.20 = 4.69 s
turn_cost_mm(pi/2)   = 4.69 s x 83.4 mm/s (weighted)     = 391 mm
```

The speed that converts turn time into millimetres is **not** `CRUISE_MPS`.
This rover never cruises: it drives one 70 mm leg, stops, and settles for
`STOP_SETTLE_S` before the next. It makes good 83.4 mm/s, not the 180 mm/s
`CRUISE_MPS` claims, and pricing a turn against the wrong one costs a factor of
two.

**A clearance gradient**, Nav2's inflation-layer idea and only that idea: a cost
that decays with distance from the nearest obstacle, expressed directly in
millimetres of equivalent detour. It is deliberately too weak to buy a turn —
buying one 90° turn at `CLEARANCE_COST_W = 0.25` would take 1.5 m of continuous
wall-hugging, and this arena is 1.2 m across (`fpms_missions.py:1887-1898`).

**Hysteresis.** The committed path is sticky: kept unless it has become unsafe,
or unless the new plan is better by `PLAN_HYSTERESIS_FRAC` (15 %). Without it a
planner re-deciding from scratch takes the left detour, then the right, then the
left, spending a 391 mm turn each time it changes its mind
(`fpms_missions.py:1984-1998`, enforced at `3118-3147`).

**A live `ObstacleDetour` reroute.** When the obstacle guard trips, the running
segment stops, settles and is measured as usual, then the route is re-planned
around what the LiDAR has just seen — up to `REPLAN_MAX` (5) times per leg.

**The occupancy grid keeps centroids, not cell indices.** A wall at x = 1200 is
remembered at 1200, not at the 1175 cell centre. That single change deleted three
stacked pessimisms that had turned the one leg needing no turn at all into a
detour when the wall was measured 60 mm too near (`fpms_missions.py:1908-1965`).

**Forgetting is a feature.** `OCC_TTL_S = 6.0 s`: a cell nobody has seen for six
seconds stops blocking routes, so a path that has been cleared re-opens on its
own, and a person who steps out of the way is gone before the next leg.
`OCC_MIN_HITS = 2` stops one bad return walling off the only route.

---

## 2. The three zones, and how a mission targets one

### The zones

Verified 2026-08-13 against **four** independent sources, which agree exactly.

| id | corner | mission | centre (mm) | map (m) | rect x,y,w,h (mm) | status |
|---|---|---|---|---|---|---|
| `zone-a` | TOP-LEFT | `m1` | (228, 972) | (0.228, 0.972) | 48, 792, 360, 360 | DERIVED |
| `zone-b` | TOP-RIGHT | `m2` | (972, 972) | (0.972, 0.972) | 792, 792, 360, 360 | DERIVED |
| `water-station` | BOTTOM-LEFT | `water` | (228, 228) | (0.228, 0.228) | 48, 48, 360, 360 | DERIVED |
| start box | BOTTOM-RIGHT | `home` | (972, 228) @ 90° | (0.972, 0.228) | 792, 48, 360, 360 | DERIVED coords, **ASSUMED placement** |

All of it follows from three numbers — `ARENA_MM = 1200`, `Z = 0.30 · ARENA_MM`,
`M = 0.04 · ARENA_MM` — so a rescale is still a one-number change:

```
side   = 0.30 x 1200 = 360        margin = 0.04 x 1200 = 48
zone-a = (48+180, 1200-48-360+180)        = (228, 972)
zone-b = (1200-48-360+180, same)          = (972, 972)
water  = (48+180, 48+180)                 = (228, 228)
start  = (1200-48-180, 48+180) @ 90°      = (972, 228), 1.5707963267948966 rad
```

**Sources checked, and the result.** No disagreement was found on any zone
centre, on the start pose, or on the arena size:

- `frontend/src/lib/arena.ts:201-204, 228-263, 280, 294-352` — the single source of truth
- `nav2/make_arena_map.py:103-162` — independent mirror of the same derivation
- `nav2/arena_map.yaml:36-46` — states the frame identity and quotes zone-a as arena (228, 972) = nav2 (0.228, 0.972)
- `fpms_missions.py:520-548` — the executor's own constants
- `fpms-os/overlay/etc/fpms/config.env` — `FPMS_ARENA_ANCHOR_X/Y/YAW` (cited by name, not line: that file is edited often)
- `rover/rescued/arena_zones.json` — a previously generated sidecar; byte-for-byte agreement

**Three things that look like disagreements and are not.** Recorded so nobody
rediscovers them as findings:

1. `FPMS_ARENA_ANCHOR_YAW = 1.5707963` is π/2 truncated at seven decimals. The
   difference is 2.679 × 10⁻⁸ rad — **3.2 × 10⁻⁵ mm** over the arena diagonal.
2. `make_arena_map.py`'s map origin is (−0.10, −0.10). That is the *image's*
   bottom-left corner including the 10-pixel wall ring; the arena interior still
   starts at exactly (0, 0).
3. `arena.ts`'s `worldToCanvasY` flip is a canvas concern. It must never be
   applied to a coordinate sent to a planner.

**One real disagreement, and it is not about the zones.**
`nav2/nav2_params.yaml` sets `robot_radius: 0.15` in both costmap blocks (lines
558 and 668); `fpms_missions.py` uses `ROBOT_RADIUS_MM = 170.0`. That is a 20 mm
difference about the *chassis*, already flagged as `NAV2_PLAN.md` item 2, and
still unreconciled because Nav2 is planning-only and off the critical path. Both
numbers are **ASSUMED** — NAV2_PLAN.md derives 0.166 from
`sqrt(0.12² + 0.115²)` and says plainly *"Measure the chassis before setting
this."* Nobody has.

### The naming trap

The mission ids are historical and do **not** line up with what an operator says
out loud. Standing behind the rover at the start box:

- the zone **straight ahead** (top-right, `zone-b`) is mission **`m2`**
- the **far** zone (top-left, `zone-a`) is mission **`m1`**

So an operator's spoken "Zone 1" is the code's `m2`. Nothing is renumbered —
renaming ids to match one operator's vocabulary would silently change what every
stored command, log line and dashboard button means. Every operator-facing string
names the **corner** instead, because a corner is the one description that cannot
be read two ways.

### How a mission targets a zone

`mission_target(name)` returns `(x_mm, y_mm, final_heading_deg or None)`
(`fpms_missions.py:562-575`). All three zones return `None` for the final
heading; only `home` requires one (90°), because coming home means coming home
facing the way you started, not merely standing in the right square.

A **route** is an ordered list of single-target missions driven back to back,
finished by one retrace of the whole thing. `patrol` is `m2 → m1 → water`. Two
orders tie for shortest at 2232 mm of outbound travel (`m2→m1→water` and
`water→m1→m2`); the tie is broken by the first leg, because the rover starts at
(972, 228) facing 90° — pointing exactly at `m2` — so leg 1 needs **no turn at
all**, and every turn is ±1–4° of heading error that every later leg inherits.

A single-target mission is executed as a one-leg route, on purpose: the executor
then has exactly one shape to run, and there is no second code path to drift.

### `/etc/fpms/zones.json`

FPMS-OS now ships `overlay/etc/fpms/zones.json`, which carries all of the above
as data: the derivation inputs, the derived centres, per-zone approach headings,
keep-out slots, wall clearances, the mission↔zone map, the route mirror, and a
`status` on every number.

It is installed automatically — `scripts/50-overlay.sh:72-75` copies the whole
overlay and lines 337-341 chmod every `/etc/fpms/*` file to 0644 root:root. The
CRLF sweep at line 132 already globs `/etc/fpms/*`, and `.gitattributes` forces
LF. **No build-script change was needed and none should be added.**

Two properties of that file are deliberate and easy to undo by accident:

- **It is strict JSON, with no `//` comments.** `calibration.json.example` has
  comments because it is an example and is never parsed. This file is meant to
  be parsed, so every piece of provenance is a *data field* — which can be read,
  published to telemetry and asserted on by a test. A comment cannot.
- **`keep_out_radius_mm` is `null` on all three zones, not `0.0`.** `null` means
  "no keep-out is applied", which is behaviour-identical to today. `0.0` would be
  a number, and no number has been measured: nothing has ever been placed in a
  zone and its footprint recorded. `calibration.json.example` makes the same call
  about `gyro_scale` — *omit the key rather than writing 1.0, so a refusal is
  logged instead of a wrong number adopted.*

**Nothing reads it yet.** As of 2026-08-13 `fpms_missions.py` still computes
these values from its own constants and is byte-identical in behaviour with or
without the file present. Shipping the data first and wiring it second means the
wiring can be *proved* to change nothing. Section 6 says exactly what the wiring
would be.

---

## 3. The fallback ladder

Aborting the instant the search fails at full padding throws away the difference
between *"there is no way through"* and *"there is no way through **at full
margin**"*. Those are different answers and the operator deserves the second one.

```
RELAX_LADDER = (("none",        1.0, 1.0),
                ("wall",        0.5, 1.0),
                ("wall+radius", 0.5, RELAX_RADIUS_FRAC = 0.85))
```

The wall pad is relaxed **first** because it is a policy about a boundary that is
known and static. The obstacle radius is relaxed **last and least** — it is the
rover's actual half-width, and cutting it is the only rung that can put the
chassis into something. It is floored at 0.85 so it can never go far.

**The ladder cannot reach zero.** There is no rung that removes padding, because
"no route at any margin" and "a route only if the chassis is allowed to collide"
are the same answer as far as the rover is concerned
(`fpms_missions.py:3020-3027`).

Every rung is **reported, never silent**. A path that only exists because a
margin was cut is not the same answer as one that exists at full margin, and both
the note and `telemetry/mission_plan` say which rung drew the line. Even a rung
that returns a *direct* route emits a note, because drawing that silently would
tell the operator the route is ordinary when it is the one case they most need to
be told about (`fpms_missions.py:3221-3229`).

Below the ladder, the safety net ends and the original behaviour resumes.
An empty grid, a stale grid, a search that finds nothing, or `REPLAN_MAX`
exhausted all end in `ABORT_OBSTACLE` — the same abort, with the same
operator-facing string, as before any of this existed. **Everything in the
planner is stretched in front of that abort, never in place of it.**

One more guard worth knowing: a route is only ever returned after `segment_ok`
has confirmed every one of its legs on the costmap that produced it. A search
that returns a route its own map calls undrivable is a bug, and it is caught
there rather than at the wheels (`fpms_missions.py:3103-3105`).

---

## 4. What the telemetry tells you

### `telemetry/mission_plan` — plan, then follow

Published by one producer, from one code path, whether it was asked for as a
preview or is about to be driven. `committed` is the only difference, and it says
which. A route drawn on the dashboard that the rover does not then follow is
worse than drawing nothing.

On acceptance it is published **before the worker starts**, so the drawn line
always exists before the first wheel turns rather than being reconstructed
afterwards (`fpms_missions.py:5351-5456`).

| field | what it answers |
|---|---|
| `waypoints` | the polyline the map draws — cumulative pose after each segment, each tagged with its leg |
| `segments`, `distance_mm`, `eta_s` | what will be driven, how far, how long |
| `legs`, `legs_n` | per-leg target, label, segment count, distance |
| `planner` | which of `theta` / `astar` / `straight` drew it |
| `planner_note` | **why the line bends** — the detour, the waypoint and obstacle-cell counts, how many cells were the arena wall and handled by the wall pad rather than routed around, and which relaxation rung produced the line |
| `planner_cost` | the cost model that chose it: `turn_cost_mm_per_90deg`, turn weight, clearance weight and decay, hysteresis fraction |
| `planner_margins` | the margins it was allowed to use: robot radius, wall pad, wall band, point pad, whether relaxation is enabled |
| `occupancy` | what the grid currently believes |
| `pose_assumed`, `pose_source`, `arena_fix` | **whether the start of this plan is a measurement or a belief** |

`planner_cost` and `planner_margins` exist so a route that looks wrong on the map
can be explained without an ssh session.

### `telemetry/residual` — the single most diagnostic number this rover produces

Emitted per segment, the instant it settles. Every segment ends with a full stop,
a settle and a measurement at rest — that is the whole B8B method — so at that
exact point both the commanded value and what actually happened are known, with
the chassis stationary and the measurement therefore trustworthy.

Per segment rather than per leg, because a leg is several segments and averaging
them hides exactly the pattern below (`fpms_missions.py:5784-5850`).

Watching `residual` accumulate live is how an operator distinguishes three
failures that look identical on the arena map:

| pattern | cause | fix |
|---|---|---|
| a constant **ratio** between measured and target | scale error — counts/mm | `odom_scale` in the calibration profile (2.467 = 74/30 is the known candidate) |
| a constant **offset**, whatever the leg length | coast: the rover keeps moving after the burst is cut | `coast_mm` / `coast_deg` |
| growth only on **turns** | heading / gyro | `gyro_scale` |
| growth only **after** a turn | heading leaking into the next drive | — |

`lateral_mm` is carried too: drift *across* the leg, which the executor reports
and deliberately does not correct. Each message also carries the `odom_scale` in
force, so a reader can tell which calibration produced the numbers without
cross-referencing a log.

---

## 5. The D\* Lite question — the arithmetic

**Verdict: do not implement D\* Lite. Not on this arena, not at this scale, and
not on this planner.** The repo's existing one-line rejection is correct; what
follows is the working, plus two arguments the original note does not make.

### What is actually being asked

D\* Lite's advantage is cheap *replanning* after a local costmap change: it
repairs the previous search instead of re-deriving it. That is worth something
only if (a) the search is a large share of the replan cost, and (b) the replan
cost is a large share of the mission.

### Measured inputs

Two numbers, both MEASURED on this Pi, recorded in
`fpms_missions.py:1699-1710`, `STACK.md` line 433 and
`NEXT_SESSION_B8B_DUTY.md` lines 53-56:

```
worst-case A* over this grid ............ 1.2 ms
costmap rebuild (unavoidable) ........... 1.2 ms
```

A third, also measured: the m2 detour took **69 ms** before line-of-sight
memoisation, and ~1.2 ms after (`fpms_missions.py:2724-2731`). The cheap win in
this planner was already taken.

### Derived: how big the problem actually is

```
grid          n = ceil(ARENA_MM / FPMS_MISSION_GRID_MM) = ceil(1200/50) = 24
cells         24 x 24                                   =   576
directed edges (8-connected)                             = 4 608
relaxation rungs per plan_detour                         =     3   (RELAX_LADDER)
REPLAN_MAX per leg                                       =     5
```

### The clean-arena case: the search does not run at all

On a clear arena `plan_detour` builds **one** costmap, asks `segment_ok` whether
the straight line is clear, and returns (`fpms_missions.py:3094-3097`). Cost:
**1.2 ms, zero searches.** D\* Lite would be optimising a code path the rover does
not execute.

### The blocked case, worst case, in full

```
per replan   3 rungs x (1.2 ms rebuild + 1.2 ms search)  =  7.2 ms
per leg      5 replans x 7.2 ms                          = 36.0 ms
```

Now the denominator. From `config.env` (`CRUISE_MPS 0.18`, `MAX_LEG_MM 70`) and
`STOP_SETTLE_S 0.45`:

```
per segment   0.070/0.18 + 0.45                          =  0.839 s
m2 leg        10 segments (the gate) x 0.839 s           =  8.39 s
one 90° turn  (pi/2)/0.45 + 1.2                          =  4.69 s
```

So the **entire worst-case planning budget for a whole leg is 36 ms out of
8 390 ms — 0.43 %.** It is 0.77 % of a single 90° turn.

### The Amdahl ceiling

D\* Lite replaces the *search* half only. The costmap rebuild is unavoidable
precisely because the obstacle set changed — that change is what triggered the
replan. Even if the search were **free**:

```
per replan   7.2 ms -> 3.6 ms                    (2.0x on the step)
per leg      36 ms  ->  18 ms                    saving 18 ms = 0.21 % of the leg
```

Convert to the unit that matters on this rover — 18 ms × 83.4 mm/s:

> **D\* Lite's best conceivable saving over an entire leg is 1.5 mm of driving.**
> `ARRIVE_TOL_MM` is 25 mm. It is 6 % of the tolerance the rover already treats
> as "arrived", and 0.38 % of one turn.

And 18 ms is the *indefensibly generous* bound. Realistically D\* Lite state is
tied to one cost function, so it cannot carry across the three relaxation rungs
(each is a different `CostMap` with different pads), and the first plan of each
leg is a full computation anyway. That leaves ≤ 4 replans × 1 rung × 1.2 ms =
**4.8 ms per leg, before subtracting the repair work it still has to do** —
0.4 mm of driving.

### Argument the original note does not make (1): the cost function is not edge-local

This is the blocker, and it is not a matter of effort. D\* Lite's `g`/`rhs`
invariants require an edge cost `c(u,v)` that depends only on the pair `(u,v)`.
This planner's does not (`fpms_missions.py:2760-2768`):

```python
def step_cost(frm, to):
    ...
    return seg_cost(frm, to) + turn_cost_mm(wrap_pi(b - hdg[frm])), b
```

`hdg[frm]` is the heading the search *arrived at* `frm` with — a function of the
whole path, not of the edge. Making the cost edge-local means lifting the state
to `(cell, arriving heading)`:

```
8-connected:  576 cells x 8 arriving directions =  4 608 states
              8 successors each                 = 36 864 directed edges   (8x)
any-angle:    arriving heading is continuous    -> unbounded
```

So for `astar` mode this is not "≈200 lines of incremental bookkeeping" bolted
onto the existing search — it is a different search over an **8× larger state
space by exact edge count**, which makes every *from-scratch* computation
correspondingly more expensive. Every leg begins with one from-scratch
computation. **The first plan of each leg would plausibly cost more than all four
possible replan savings combined.**

For `theta` mode — the default — the arriving heading is continuous, so D\* Lite
cannot be applied at all without discretising heading, which reintroduces exactly
the turn quantisation that Theta\* exists to remove. And that has a price tag:
one avoided 45° staircase step is worth 391 mm. Trading 1.5 mm of search saving
for one extra turn is a **260:1 loss**.

### Argument the original note does not make (2): the change set is not small

D\* Lite wins when *few* cells change between replans. Here:

- `OCC_TTL_S = 6.0 s` — any cell not re-seen within six seconds is forgotten
  outright, by design.
- The LiDAR folds 360 bins per scan at ~9.83 Hz, and each fold updates a running
  **centroid**, so believed surfaces move continuously rather than flipping
  boolean.
- Every replan happens after the rover has **moved**, which changes `free=(here,)`
  and re-runs the pose-dependent wall absorption — so the costmap differs even
  where the world did not.

D\* Lite's repair is O(changed states × log heap). As the change set approaches
the whole grid it degenerates to a full search **plus** bookkeeping — strictly
slower than the A\* it replaced. Six seconds is longer than one segment (0.84 s)
and shorter than a leg (8.4 s), which puts consecutive replans squarely in the
region where the belief has substantially turned over.

The original note makes a related and correct point from the other direction: a
search re-derived from scratch every time **cannot carry a stale belief across a
replan**, which on a rover whose pose is dead reckoning is worth more than the
millisecond.

### Under what conditions D\* Lite *would* be right

Two independent thresholds, both quantified, neither met here.

**(a) Scale.** The rebuild is roughly O(cells); the search is roughly
O(cells · log cells). Their ratio grows only as `log n`, so the search overtakes
the rebuild very slowly. At 576 cells they are exactly equal (1.2 ms / 1.2 ms).
The code's own estimate — "a few hundred times this many cells" — puts the
crossover near 10⁵ cells:

```
576 x 200 = 115 200 cells  ->  339 cells a side  ->  17 m x 17 m at a 50 mm grid
                            or the same 1.2 m arena at a ~3.5 mm grid
```

A 17 m arena, or a grid 14× finer than the rover's own 25 mm arrival tolerance
can justify. Neither is this arena.

**(b) Replan rate.** D\* Lite pays when replanning is a *loop*. Here it is
event-driven — raised by the obstacle guard as `ObstacleDetour` — and capped at
5 per leg. For planning to reach even 5 % of a leg's wall time:

```
8 390 ms x 0.05 = 419 ms  /  7.2 ms per replan  =  58 replans per 8.4 s leg
                                                =  ~7 Hz, sustained
```

The rover has no such loop, and adding one would not help it: it is **standing
still for 54 % of every segment** (0.45 s of settle per 0.839 s) by design,
because that stop is the speed control on this firmware.

### If planner time ever did need to come down

It does not. But if it did, the cheapest remaining win is **not** the search.
`plan_detour` builds a fresh `CostMap` for every relaxation rung
(`fpms_missions.py:3089-3093`), and the rungs differ only in `wall_pad_mm` and
`radius_mm`. The obstacle point list, the wall absorption and the spatial index
(indexed at the largest `reach`) could be built once and shared, saving up to
**2 of the 3 rebuilds = 2.4 ms per blocked replan — twice D\* Lite's realistic
saving**, with no incremental invariants, no float-comparison traps, and no
change to the search at all.

Design sketch, not a measurement; `blocked` and the distance field genuinely
change per rung and would still have to be recomputed. And rungs 2 and 3 only run
when rung 1 fails, so on a clean arena this saves precisely zero.

**Which is the real conclusion: every available planner optimisation on this
arena is noise. The right answer is not to optimise the planner.**

### What would actually help, in order

Ranked by how much floor each one moves, all of them already documented in this
repo as open:

1. **`odom_scale` is 1.0 and the profile is unmeasured.** counts/mm is the
   longest-running open number in this project — 5.5 hand-pushed, 14.8
   tape-measured, 6.00 derived — and a 2.7× error in `COUNTS_PER_REV` already
   explains the 2.3 m overshoot, **the 744 mm plan that drove ~2 m**, and the
   300 mm move that went ~1 m. *One constant, not three bugs.* A planner emitting
   an exact 744.0 mm route into a chassis with a 2.7× scale error is not
   search-limited. Cost: `fpms_charact.py --push-check` then `--drive`. Payoff:
   metres.
2. **`FPMS_TF_OFFSETS_MEASURED = 0`, and all six LiDAR mount offsets are zero.**
   1° of mount yaw ≈ 10.5 mm of position error, and **both** the obstacle cone
   guard and the occupancy grid inherit it in the same direction, so it does not
   average out. Every obstacle cell the planner routes around is placed by that
   transform. Cost: a ruler. See `docs/CALIBRATION.md`.
3. **`LIDAR_ROTATION_SIGN = -1` is UNVERIFIED.** If the scan is mirrored, no
   value of `laser_yaw` can undo it — a mirror is not a rigid transform. The
   planner would then route confidently around obstacles reflected to the wrong
   side of the rover. Cost: one asymmetric object and one scan.
4. **`gyro_scale` is unmeasured.** Heading closes on the gyro, so turn accuracy
   rests entirely on it — and the ±1–4° per turn is exactly what `turn_cost_mm`
   is pricing. Halving it is worth more than any search change could be.
5. **Measure `robot_radius`.** It is 170 mm in the executor and 150 mm in
   `nav2_params.yaml`, and both are assumptions. At the zone centres there are
   only 58 mm of margin, so a 20 mm error is a third of the entire budget.

Every one of those is a measurement, not code. That is the honest ranking: this
rover's planner is better than its pose, and has been for some time.

---

## 6. What `fpms_missions.py` would need to consume `zones.json`

Not done here — `fpms_missions.py` is owned elsewhere and was deliberately not
edited. This is the specification.

**Where.** `ZONES` and `ROVER_START` are module-level constants at lines 531-548,
and `Anchor`'s field defaults reference `ROVER_START` at class-definition time
(line 3263-3265). So the loader must run **after** `CFG_NOTES = []` (line 457)
and **before** the `ARENA_MM` block (line 520). One insertion point, around line
519.

1. **A loader shaped exactly like `load_calibration()`** (lines 804-826).
   `ZONES_FILE = os.environ.get("FPMS_ZONES_FILE", "/etc/fpms/zones.json")` —
   env for the *path only*, mirroring `CALIB_FILE`, as a test hook. Never raises.
   `FileNotFoundError` returns `{}` **silently**: a missing file must change
   nothing, and a file that must exist for the rover to work is a new way for the
   rover to stop working.

2. **A verification gate, not a plausibility gate.** This is the one place the
   calibration pattern must *not* be copied. Calibration values can be
   sanity-ranged; zone coordinates cannot — a zone 100 mm from where it belongs
   is a perfectly plausible number that drives the rover to the wrong place,
   silently and forever. The loader must **recompute** each centre from the
   file's own `arena_mm`, `zone_side_frac` and `zone_margin_frac` and **refuse
   the whole file** if any published `cx_mm`/`cy_mm` disagrees by more than
   0.05 mm — the same tolerance `test_stack.py:96` uses on the 744.0 mm gate.
   Refusal keeps the built-in constants and logs to `CFG_NOTES`, exactly like
   `calibration ... REFUSED`.

3. **Exactly three constants become file-overridable:** `ARENA_MM`, `ZONE_FRAC`,
   `MARGIN_FRAC`. `ZONES` and `ROVER_START` stay **derived** from them. The
   file's `cx_mm`/`cy_mm` are read *only to be checked*, never to be used —
   reading them back would create the fifth independent copy of these numbers,
   which is the exact failure the "mirror the derivation, not the output" rule
   exists to prevent.

4. **`final_heading_deg` must stay `null` for the three zones.**
   `mission_target()` returns `None` for `m1`/`m2`/`water` today. Adopting a
   non-null value adds an arrival turn worth 391 mm of equivalent driving plus
   ±1–4° of inherited heading error. The loader should adopt a non-null zone
   heading only with a loud `CFG_NOTES` line naming the behaviour change.
   `approach_heading_deg` is informational and must not be consumed at all.

5. **One import-time reachability assertion.** With `ARENA_MM` file-settable, a
   rescale could put a zone centre inside the wall pad. Require
   `WALL_PAD_MM <= c <= ARENA_MM - WALL_PAD_MM` for every zone centre and the
   start pose. Today that margin is exactly 58.0 mm on all four. A file that
   makes it negative must be REFUSED, not driven. (Note the ordering: `ZONES` is
   defined at line 531 and `WALL_PAD_MM` at 1768, so this check belongs after the
   planner constants, not in the loader.)

6. **Report it.** Add the ADOPTED/REFUSED line to `CFG_NOTES`, which is already
   published on `telemetry/config_notes` (line 3548) and logged at startup
   (line 3719). Add a `zones` block to `telemetry/mission_plan` beside
   `planner_margins` (line 5449) carrying the source path, `schema`, and adopted
   /refused. **Additive only** — every existing key keeps its meaning and shape.

7. **The regression is the proof.** `test_stack.py:96` asserts m2 = 744.0 mm to
   within 0.05 mm. With `zones.json` shipped and matching, that gate must pass
   **unchanged** — that is the evidence the file changed nothing. Add a second
   test that a deliberately corrupted `zones.json` (one centre moved 1 mm) is
   REFUSED and the built-ins are kept.

Note the standing trap while wiring this: `fpms_missions.py` reads
`/etc/fpms/config.env` and **not** the process environment, so an
`FPMS_MISSION_*` value set as a systemd `Environment=` is ignored by the mission
executor. `FPMS_ZONES_FILE` should be the path override only, for tests — never
a way to point a running rover at different zones.

---

## 7. Honest limits

**The planner is only as good as the pose, and the pose is not measured.**

- The anchor is **assumed**. Nothing localises this rover. `Anchor` defaults to
  `assumed=True`, `source="assumed start pose"`, and the dashboard already
  renders a **POSE: SIMULATED** badge for exactly this case. A plan is right in
  arena coordinates while its *start* drifts.
- The **LiDAR mount transform is all zeros** and `FPMS_TF_OFFSETS_MEASURED=0`.
  Zero is a legal value: the TF tree publishes a complete, well-formed, entirely
  fictional transform and nothing errors. Every occupancy cell the planner routes
  around is placed by it, at ~10.5 mm of error per degree of mount yaw, in the
  same direction for both the grid and the cone guard.
- **`odom_scale` is unmeasured** and every displacement the executor computes
  passes through it.
- `LIDAR_ROTATION_SIGN = -1` is **unverified**, and a mirror is not a rigid
  transform.
- **58 mm.** That is the entire pose-error budget at a zone centre under the full
  wall pad. Sixty millimetres of LiDAR range noise was once enough to make the
  planner refuse the m2 corridor it can plainly drive; that specific bug was
  fixed by keeping centroids, but the 58 mm did not get any bigger.

Two structural limits, not calibration:

- **The obstacle abort is still the floor.** Everything in section 3 is a net in
  front of `ABORT_OBSTACLE`, never a replacement. If the planner is switched off,
  the grid is empty or stale, no route exists, or `REPLAN_MAX` is exhausted, the
  rover stops and says so — the same way it did before any planner existed.
- **The grid says no more than the sensor does.** It records "something was seen
  in this 50 mm square", nothing else. Fitting polygons to returns whose mount
  yaw and rotation sign are still unverified would dress a calibration error up
  as geometry.

And the standing rule that outranks all of it:

> **Judge every one of these by the operator's eyes, not by odometry.** This
> rover has reported clean travel while spinning in place, and has lied about
> direction. Nothing in the planner changes that. The residual stream exists to
> make it *visible*, not to make odometry trustworthy.

---

## See also

| file | what it holds |
|---|---|
| `/etc/fpms/zones.json` | the three zones and the start pose, with provenance |
| `/etc/fpms/config.env` | `FPMS_MISSION_*` planner tuning, the arena anchor, the LiDAR mount |
| `/etc/fpms/calibration.json` | the measured chassis profile (`odom_scale`, `gyro_scale`, coast) |
| `docs/CALIBRATION.md` | how to measure the LiDAR mount, and why zero is the dangerous value |
| `docs/FAILURE_MODES.md` | what is fixed and what is not |
| `../STACK.md` §8 | Phase 1 / Phase 2 — the joystick teaches the system, not the route |
| `../NAV2_PLAN.md` | Nav2 as a planning-only backend, and the three things to fix first |
| `../nav2/make_arena_map.py` | the arena raster and the zone sidecar, derived from the same fractions |

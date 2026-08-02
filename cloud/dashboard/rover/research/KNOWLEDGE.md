# FPMS rover — synthesised knowledge base

Digest of three commissioned research reports (`R1_AUTONOMY.md`, `R2_COORDINATES.md`,
`R3_UI.md`) reconciled against the project's own measured facts in
`rover/SESSION_HANDOFF.md`, `rover/NAV2_BRIEF.md` and `rover/nav2/TF_TREE.md`.

Machine-readable companion: **`knowledge.json`** — 112 records, one per actionable
finding, schema `{id, source_file, source_url, topic, tags[], claim,
applies_to_our_hardware, why, confidence}`.

**Written 2026-08-02. No code was changed, nothing was ssh'd, nothing committed.**

**The standing rule in this file: where an external source disagrees with a fact measured
on this rover, the measured fact wins, and the disagreement is named with both citations.**

---

## 0. On the "vector database" — what was built and why

The operator asked for the research to be streamed into a vector DB. **It was not, and here
is the honest reasoning.**

A real vector store needs three things this project does not have: an embedding model, a
running index service, and enough documents that lexical search stops working. The corpus
here is **three reports and two project docs — about 145 KB of prose on one rover**. At
that size a `grep` over `knowledge.json` beats a nearest-neighbour lookup on every axis
that matters: it is exact, it is auditable, it has no recall cliff, it needs no service to
be up, and it cannot silently return a semantically-adjacent record that is factually the
opposite of what was asked. Embedding 112 records so an agent can find the six about
`inflation_radius` is infrastructure without payoff.

**One correction to the premise of the request.** The brief stated Cloudflare Vectorize is
already a binding in this project's Worker. **It is not.** Both wrangler files were read:
`cloud/cloudapp/wrangler.jsonc` binds `AI`, `HUB` (Durable Object), `DB` (D1
`fpms-history`) and `ASSETS`; `cloud/dashboard/gateway/wrangler.toml` binds `FPMS_KV`
only. A repo-wide search for `vectorize` returns nothing. Standing one up would be **new
infrastructure** — index creation, an embedding pipeline, an ingest path, and a second
source of truth to keep in sync with the repo — not the flip of an existing switch.

**What was built instead:** a structured, chunked, tagged JSON index. Every record carries
its source file, a real URL where the source had one, a `topic`, searchable `tags`, and
the load-bearing `applies_to_our_hardware` field. It is greppable
(`rg '"tags".*OVERRIDE' knowledge.json`), loadable in three lines of Python, diffable in
review, and it lives in the repo next to the code it describes.

**When a real vector DB would be warranted — the concrete trigger.** If this corpus grows
past roughly a few hundred documents, or if telemetry archives in D1 start being queried
semantically ("show me sessions that looked like the drivetrain fault"), then Vectorize
becomes the right tool. The setup would be: `npx wrangler vectorize create fpms-knowledge
--dimensions=768 --metric=cosine`, add a `vectorize` binding to `wrangler.jsonc`, embed
with the **already-bound `AI`** binding via `@cf/baai/bge-base-en-v1.5`, and store the
record `id` in vector metadata so retrieval returns a pointer into `knowledge.json` rather
than duplicating the text. The `AI` binding already existing is what would make that cheap.
Until the corpus justifies it, this file and its JSON are the retrievable artefact.

---

## 1. What all three reports and the project agree on

Convergence from independent directions is the strongest signal in this knowledge base.

**The firmware floor is real and it is the design constraint.** R1 opens with it as its
hardware contract; NAV2_BRIEF §3b derives it from firmware source (integer encoder counts
per 10 ms PID period, one count = 0.0145 m/s on the wire, a 200/400 dead zone that makes
duty either 0 or 50–100%). Nobody disputes it. Its consequence — *slow is bought with
short bounded segments and full stops, never with a lower setpoint* — is the single
sentence to hand any future agent.

**Heading must come from the gyro, not the wheels.** R1 §3.6, R2 §2.2 and
`fpms_odom_tf.py` all reach this independently. A 4WD skid-steer produces every degree of
yaw by dragging four wheels sideways, so the *effective* track is 1.5–2.5× the geometric
one and varies with friction, load and turn rate. `TRACK_M = 0.170` would be badly wrong as
an effective track — and the project never uses it for heading. R2 states plainly this
"should be preserved, not fixed by switching to encoder-derived yaw".

**Stop-and-fix against the known arena walls is the top recommendation of both R1 and R2.**
R1 ranks it #1 of five; R2 ranks it #1 of nine. Two perpendicular walls fully determine
planar pose in a known rectangle — no SLAM, no AMCL, no particle filter. And the project
has **already written and unit-tested the mathematics** (`manhattan_offset`, `wall_fit`,
`arena_fix_from_scan`, `map_to_odom_from_fix` in `fpms_odom_tf.py`, exercised by
`--selftest`). Three sources agreeing, with the code already in the tree, is as strong as
this base gets.

**Segment-and-settle is not a workaround — it is the correct architecture.** R1's key
find: Nav2's own repo ships `odometry_calibration.xml`, a behaviour tree that drives a
square from `DriveOnHeading` (a *displacement*, terminated on measured pose delta) plus
`Spin` (a yaw delta). Upstream Nav2 already endorses this when a controller cannot be
trusted. It is exactly what the project's `deadreckon` backend does and what the prior
autonomy did at 0.6% distance error.

**AMCL is not the route to centimetres.** R1 says skip it; R2 cites published AMCL RMSE of
~8.5 cm empty and ~33.7 cm cluttered — 10–30× short of ±1 cm. Keep the existing config as a
Nav2-compatible fallback; do not rely on it for precision.

---

## 2. Conflicts, and what to do about each

### 2.1 "Just flip `LIDAR_FIX_ENABLED`" — **the project wins on method, R2 wins on priority**

> **R2_COORDINATES.md §7 item 1:** "Flip `LIDAR_FIX_ENABLED` (`rover/fpms_odom_tf.py:583`)
> … **Highest value by a wide margin — the code is already written and tested.**"

> **`rover/fpms_odom_tf.py:576-583`:** "`LIDAR_FIX_ENABLED` is not a convenience switch.
> Turning it on without M4, M5 and M6 from the docstring produces a pose correction that is
> **CONFIDENT and WRONG**, which is strictly worse than the dead reckoning it would replace,
> because dead-reckoning error is smooth and recognisable while a bad absolute fix teleports
> the robot."

**The measured fact wins.** R2 is right that the wall fix is the only thing that *bounds*
drift, and right that it is the highest-value item. It is wrong that the flag is the first
step — it is the **last**. The gates are `M4` (LiDAR mount transform, above all
`LASER_YAW_RAD` in `nav2/fpms_tf.launch.py`), `M5` (`LIDAR_ZERO_OFFSET_DEG` and
`LIDAR_ROTATION_SIGN`), and `M6`.

**R2 missed M6 entirely, and M6 gates everything else:** *does the arena have physical walls
the scanner can see, at the scanner's mounting height?* If the 1200 × 1200 boundary is tape
on the floor, or the wall is shorter than the LiDAR is tall, there is nothing to fit and the
entire correction design is void. Nobody has recorded this either way. It is a five-second
observation that decides whether items 2 and 3 of the action list are worth taking at all.

**Do:** answer M6 first, then calibrate M4/M5, then flip the flag. Never in another order.

### 2.2 Run UMBmark — **R1 is wrong for this rover, R2 is right**

> **R1_AUTONOMY.md, Top 5 item 5:** "Run UMBmark (bidirectional square, CW and CCW) to
> calibrate systematic odometry error."

> **R2_COORDINATES.md §2.3:** "`Ed` is not applicable to us… `Eb` calibrates
> encoder-derived yaw, which **we do not use**. **Not applicable.**"

**R2 wins, and its reasoning was verified against the code.** UMBmark's two corrections are
`Ed` (wheel-diameter ratio) and `Eb` (effective wheelbase). Applying `Ed` needs per-wheel
control, and this rover exposes **no per-wheel encoder topic and no `/joint_states`** —
confirmed, the ESP32 publishes only `/cmd_vel`, `/odom_raw`, `/imu`, `/battery`. `Eb`
calibrates the encoder-derived yaw that `fpms_odom_tf.py` deliberately does not use.

**Do:** run only the **straight-line scale calibration** — the scalar mapping `/odom_raw`
pose-delta to true ground distance. A 20-minute bench job that takes a typical 1–3% error
down to 0.2–0.5%.

**Keep one nuance from the project's own docstring:** `M2` asks for the skid-steer
effective-track factor via four 90° turns ×5, read as `scale` from the 30 s report line.
That is Eb-shaped, but it is explicitly a **witness** — it "turns nothing on", it converts
the heading-source argument from prose into a number. Measure it; do not apply it.

### 2.3 Goal tolerance versus the motion quantum — **unresolved, and the cheapest check on this list**

> **R1_AUTONOMY.md §2.3:** "any `xy_goal_tolerance` tighter than the motion quantum is a
> guaranteed infinite oscillation… **This is the single most important number in this
> document and it cannot be sourced from any repo — only from your bench.**"

> **`rover/nav2/nav2_params.yaml:384`:** `xy_goal_tolerance: 0.10`

The quantum is `v_full × 0.35 s`. `SESSION_HANDOFF.md` records "pulse 0.35 s → MOVES, ~0.18
per pulse" **free-spinning, wheels off**, and NAV2_BRIEF §3b records a ~6× wire-to-real gain
(commanding 0.10 yields ~0.61 m/s actual). If `v_full` under load lands anywhere near
0.5 m/s, the quantum is ~0.18 m — **larger than the configured 0.10 m tolerance, which would
make every Nav2 goal oscillate forever.**

**Neither source can settle this**, because the one number it turns on has never been
measured under load. SESSION_HANDOFF is explicit: "EVERY number above is FREE-SPINNING,
wheels off. All of it must be re-measured under load before any of it is trusted for
distance."

**Do:** measure `v_full` under load, compute the quantum, then set
`xy_goal_tolerance ≥ 1.5 × quantum` in `nav2_params.yaml`. Until then, treat the Nav2
backend's goal tolerance as unproven. This is a high-consequence bug that costs one bench
run to rule in or out.

### 2.4 Regulated Pure Pursuit — **agreement on the diagnosis, a real gap in the cure**

R1 §1.2 calls RPP "the single biggest NOT APPLICABLE in the report": all five of its
velocity-regulation knobs are no-ops on a 50%-duty plant. The project **already reached this
conclusion independently** — `nav2_params.yaml` disables or floors exactly those five knobs,
with a comment block calling them "THE MOST IMPORTANT NUMBERS IN THIS FILE". No conflict on
the diagnosis.

**But R1's deeper point survives the fix.** With every taper disabled, RPP degenerates to a
constant-velocity geometric follower that still emits **continuous** `/cmd_vel`, and the
firmware turns any continuous non-zero setpoint into continuous 50% duty. That is *cruise*,
not the *segment-and-stop* motion `NAV2_BRIEF` §3b prescribes. The `deadreckon` backend
produces segment-and-stop; the Nav2 backend as configured does not.

**Do:** adopt R1 §4.3 — **use Nav2's top and bottom, skip the middle.** Keep
`nav2_simple_commander` / the BT navigator for sequencing, `Spin` + `DriveOnHeading` for
primitives, `nav2_waypoint_follower` with a long pause, and costmaps for obstacle checking.
Skip `controller_server` `FollowPath`. This keeps the Nav2 requirement satisfied and the
migration path open, while making the Nav2 backend match the hardware's actual motion model.

### 2.5 Drop the static layer and run in the `odom` frame — **both right, about different phases**

> **R1_AUTONOMY.md §3.1:** set `global_frame: odom` on both costmaps, drop the `map` frame,
> and "**also drop the `static_layer`** … with no localisation, a static map layer will
> progressively disagree with reality as drift accumulates, producing ghost walls the rover
> refuses to drive through."

> **`rover/nav2/nav2_params.yaml`:** `global_costmap` includes a `StaticLayer` against
> `nav2/arena_map.yaml`, with AMCL configured.

**Not a contradiction — a phase difference.** AMCL is configured but has **never been
launched** (NAV2_BRIEF §7; the LiDAR→ROS→TF→map→AMCL chain is unbuilt). So *today* R1 is
right: a static layer referenced to a drifting `odom` frame accumulates ghost walls that
raytrace-clearing will not remove, because the drift is in the *pose*, not the *scan*. Once
AMCL runs, the project's configuration becomes the correct one.

**Do:** keep two profiles rather than editing one back and forth — an interim
no-localisation profile (`global_frame: odom`, no static layer) and the existing
AMCL profile. Without the frame change, every Nav2 lifecycle node stalls waiting for
`map → odom`. Related one-line fix: `waitUntilNav2Active(localizer=None)`, or it hangs
forever waiting for a lifecycle node that does not exist.

### 2.6 Costmap geometry — **`robot_radius` settled, `inflation_radius` open**

R1 §2.3 derives `robot_radius: 0.11` (marked "MEASURE IT") and `inflation_radius: 0.05`.
The project uses `0.15` and `0.20`.

- **`robot_radius`: the project wins.** `0.15` is the circumscribed radius of the measured
  240 × 180 mm chassis (`√(0.12² + 0.09²) = 0.15`). R1's `0.11` is a placeholder its own
  text tells you to replace.
- **`inflation_radius`: genuinely open.** R1's arithmetic — that inflation swamps a 1.2 m
  arena and leaves the planner no gradient to descend — was computed against the upstream
  `0.55`, not against the project's `0.20`. Whether a usable gradient survives at `0.20`
  with `cost_scaling_factor: 10.0` is a bench question, not a documentation question.

**Do:** keep `robot_radius: 0.15`. Test `inflation_radius` by inspecting a published
costmap in the real arena before changing it. Also note R1's finding that **no published
Nav2 param set exists for a sub-2 m arena** — neither TurtleBot3 Burger nor linorobot2 goes
below `resolution: 0.05` or `inflation_radius: 0.5`. These numbers are being *derived*, not
copied, so bench validation is the only available check.

### 2.7 R3's palette Defect 2 — **stale; the code already fixed it**

> **R3_UI.md §7 Defect 2:** the measured trail `#38bdf8` and the water station `#22d3ee`
> measure ΔE 6.7, "a hard fail for full-colour vision".

> **`ArenaMap.tsx:298`:** `trailMeasured: "#34d399"`, with an in-code comment: "This was sky
> (#38bdf8 dark / #0369a1 light) … Emerald is `theme.measured` … Validated against the
> planned-route ink on both surfaces (dark dE 15.7 normal / 12.7 deutan…)".

**The code wins — R3 audited a revision one step behind the working tree.** The three-way
blue collision is already resolved. What remains is a smaller two-way question between the
planned-route sky `#7dd3fc` and the water-station cyan `#22d3ee`.

**Do:** do not "fix" this and regress a validated colour choice. **Defects 1 and 3 were
checked against current code and both stand** — see below.

### 2.8 The two R3 defects that are real — **verified in code**

- **Defect 1.** `THEME_DARK.assumed` and `CLASS_COLORS_DARK[3]` are the **same hex**
  `#fbbf24`; in light mode `assumed === "#b45309" === CLASS_COLORS_LIGHT[3]`. The amber
  ghost rover, the amber assumed trail and every `OBSTACLE` LiDAR point are one colour — the
  channel that must never be misread (*is this pose real?*) shares an ink with ordinary
  sensor data. **Do:** reserve amber for provenance; re-hue `OBSTACLE` in
  `lib/lidarCluster.ts`.
- **Defect 3.** `measured` `#34d399` against `danger` `#fb7185` measures ΔE 4.6 deutan —
  `POSE MEASURED` and `LINK DOWN` are the same colour to a deuteranope, and no red/green
  pair will fix it. The HUD badge carries text so it is mitigated there; the **`< 150 mm`
  danger ring is hue-only and is not**. **Do:** add a glyph or `NEAR 120mm` label to the ring.
- **ABORT.** `MissionStrip.tsx:189` renders `ABORT MISSION` only when `(running || f.blind)`
  — verified. A control that moves or vanishes with state cannot be hit from muscle memory.
  **Do:** always render it in a fixed position, disabled rather than absent, and keep the
  existing "ungated by link checks" comment verbatim.

### 2.9 An internal contradiction the reports could not see

`SESSION_HANDOFF.md` (2026-08-01) records a rewire: `M1 = FRONT-RIGHT, M2 = REAR-RIGHT,
M3 = FRONT-LEFT, M4 = REAR-LEFT`, and warns **every turn is mirrored** versus what the
mission code assumes. `NAV2_BRIEF.md` §2 still documents the **old** layout (`M1 front-left,
M2 back-left…`). **The newer file wins.** Verify turn sign before trusting any heading
control, and reconcile the two documents.

### 2.10 "±5 cm" versus "0.6% distance error" — **not a conflict**

R2 §6a concludes dead reckoning gives ~±40–75 mm per leg and 50–150 mm absolute. NAV2_BRIEF
records the prior autonomy at **0.6% distance error and ±1–4° turns**. These measure
different things: 0.6% is **relative** error along a leg; R2's figure is **absolute arena
position**, and it is dominated by the assumed start pose and accumulated heading — terms no
proprioceptive sensor can observe. Both are true. The retrace strategy is precisely what
makes the relative number good without ever needing the absolute one: it replays *measured*
outbound motions reversed, so errors cancel by construction rather than accumulating.

---

## 3. The most valuable single idea in each report

- **R1** — Nav2 already ships the primitives this chassis needs
  (`odometry_calibration.xml`: `DriveOnHeading` + `Spin`, both terminated on measurement).
  Use Nav2's top and bottom, skip the middle.
- **R2** — the wall-fit maths is already written, unit-tested and switched off, and the
  thing standing between it and ±1 cm is a **mount-yaw calibration**: 1° of uncalibrated
  mount yaw is ~10.5 mm across the arena half-width, which alone blows the entire budget.
- **R3** — **drag-to-re-zero.** Freedom Robotics makes relocalisation direct manipulation:
  drag the glyph against a live LiDAR overlay until the walls line up, then commit. We have
  every precondition already — a live scan, a wall-bounded arena, and an origin file. It
  turns the project's worst property, an unfixable accumulating drift, into a *recoverable*
  one. Runner-up: uncertainty must have a *size* you can compare to the arena (RViz's
  covariance ellipse at a stated sigma, plus a yaw cone), not a decorative pulse — and a
  `prefers-reduced-motion` operator cannot be served by a CSS rule that never reaches a
  canvas rAF loop.

---

## 4. THE 10 HIGHEST-VALUE ACTIONS

Ranked by consequence × confidence ÷ effort. Every one is blocked on hardware access, which
nobody has had since the rover was powered off — so the ordering is also a session plan.

| # | Action | File that changes | Expected effect |
|---|---|---|---|
| 1 | **Answer M6: do physical walls exist at the scanner's mounting height?** Look at the arena and record it. | `rover/fpms_odom_tf.py` (docstring `MEASUREMENTS REQUIRED`) | Decides whether actions 2, 3 and the whole ±1 cm programme are possible at all. If the boundary is tape on the floor, the correction design is **void** and R2's entire recommendation stack collapses. Five seconds; currently unrecorded. |
| 2 | **Confirm the LiDAR→ROS link is actually up** — `systemctl is-active fpms-lidar-ros` and `ros2 topic hz /scan_lidar` (expect ~9.8 Hz, `ROS_DOMAIN_ID=20`). | none (verification) | Link 1 of the Nav2 chain. Docstring and `deploy_rover.py` both *say* installed but this was **never confirmed with the rover up**. If `/scan_lidar` is silent, that is the whole job — nothing downstream is testable. |
| 3 | **Move the MQTT broker onto the Pi.** | `/etc/fpms/config.env` (broker address only — **no code change**) | Deletes WiFi from a Nav2-critical path and is what keeps action 2 *staying* true. Today the scan does a round trip Pi → laptop broker → Pi, and below about −70 dBm the LiDAR telemetry stops crossing entirely while drive commands still get through — so perception dies silently while control looks healthy. Also cuts the measured 100 ms median / 179 ms p90 round trip. Highest reliability-per-unit-work in the project. |
| 4 | **Calibrate LiDAR mount yaw to < 0.5° and verify `LIDAR_ROTATION_SIGN`** (M4/M5) using the wall test already written out in the file header. | `rover/nav2/fpms_tf.launch.py` (`LASER_*_OFFSET_M`, `LASER_YAW_RAD`); `rover/fpms_lidar_ros.py` (`LIDAR_ZERO_OFFSET_DEG`, `LIDAR_ROTATION_SIGN`) | Removes the **dominant** term in the fix budget. A wrong sign mirrors the scan and a mirrored scan of a *square* arena fits perfectly — within 150 mm of the centre line it is **accepted with up to 300 mm of error and no complaint**. The one failure here that is silent. |
| 5 | **Then enable the arena wall fix.** | `rover/fpms_odom_tf.py:583` (`LIDAR_FIX_ENABLED`) | The only change that **bounds** drift instead of slowing it: ~±5–9 mm at fix points versus 50–150 mm absolute today. Do not do this before 1 and 4 — the file's own comment explains why a confident wrong fix is worse than honest dead reckoning. |
| 6 | **Measure `v_full` under load, compute the motion quantum, set `xy_goal_tolerance ≥ 1.5 ×` it.** | `rover/deadband_sweep.py` (re-run, fixed but never re-run); `rover/nav2/nav2_params.yaml` | Rules in or out a **guaranteed infinite goal oscillation** (§2.3). Also converts every free-spinning number in `SESSION_HANDOFF` into a load-verified one, which the file explicitly demands before any of them is trusted for distance. |
| 7 | **Resolve M1 with one command:** `ros2 topic echo /odom_raw --once` after a turn — is `pose.pose.orientation` a real yaw or the identity quaternion? | `rover/fpms_teleop.py` (reconcile the contradictory comments) | Settles a contradiction the repo has carried unexamined, and unblocks M2/M3 and the complementary heading filter, whose time constant is deliberately `None` until this is known. One command. |
| 8 | **Restructure the Nav2 backend to primitives:** `Spin` + `DriveOnHeading` instead of `FollowPath`/RPP; raise `waypoint_pause_duration` to 1500–3000 ms; set `stop_on_failure: true`; switch to `StoppedGoalChecker` with `trans/rot_stopped_velocity ≈ 0.02/0.05`. Add the interim no-localisation profile alongside (`global_frame: odom`, no `map`, no `static_layer`, `waitUntilNav2Active(localizer=None)`). | `rover/nav2/nav2_params.yaml`; `rover/nav2/fpms_nav2.launch.py`; `rover/fpms_missions.py` (nav2 backend) | Makes the Nav2 path match the hardware's motion model instead of emitting continuous cruise (§2.4), and lets the stack activate at all — without the frame change every lifecycle node stalls waiting for `map → odom`. The long pause is *free accuracy*: the chassis stops, the LiDAR gets 15–30 stationary scans, gyro bias re-estimates. Watch `StoppedGoalChecker` reads **twist** — verify against the sign inversion first. |
| 9 | **Fix the three verified UI safety defects:** always-render `ABORT`; re-hue `OBSTACLE` off amber; add a glyph/label to the `< 150 mm` danger ring. | `frontend/src/components/MissionStrip.tsx`; `frontend/src/lib/lidarCluster.ts`; `frontend/src/components/ArenaMap.tsx` | Three defects confirmed against current code. Restores abort muscle memory, separates the *is this pose real?* channel from sensor data, and removes a colour-only encoding of the one state the obstacle guard acts on. Ignore R3's Defect 2 — already fixed (§2.7). Small, and none of it needs the rover. |
| 10 | **Add drag-to-re-zero:** let the operator drag the ghost rover (inner circle translates, outer ring rotates) against the live scan until the walls line up with the arena border, then commit the new origin. | `frontend/src/components/ArenaMap.tsx`; `rover/fpms_teleop_origin.json` (the origin it commits to) | R3's highest-leverage addition. Converts an unfixable accumulating drift into a **recoverable** one, and directly retires the re-zero-after-every-boot ritual that once left the rover planning from (−9182, −11926) mm inside a 1200 mm arena. Complements action 5 rather than duplicating it: the human eye does the alignment, so it does not need the mount yaw calibrated to <0.5° first. Shares the M6 precondition, and an operator aligning a *mirrored* scan could still commit a wrong pose. |

**Honourable mention, highest leverage of all but out of ROS scope:** implement a
**minimum-PWM map plus an encoder PI loop on the ESP32**. R1 §1.7 identifies the measured
symptom — `0.0010` fast, `0.00295` dead, load-dependent — as the textbook signature of no
PWM mapping and no closed-loop speed control, and `NAV2_BRIEF` §3b independently concludes
the real fix is the encoder/wheel constants, i.e. a firmware rebuild. Doing it **deletes the
"cannot creep" constraint entirely** and unlocks RPP, DWB and MPPI without rewriting the
mission layer. If the ESP32 is ever in scope, this comes before everything above. If it is
not, then the burst-primitive architecture is not a workaround — it is the correct
architecture, permanently.

---

## 5. Standing cautions carried forward

- **Suspect the measurement before the mechanism.** Both multi-hour losses in this project
  came from confident conclusions drawn from untrustworthy measurements, and neither was
  found by tuning — both were found by a test designed to *discriminate between hypotheses*.
- **A refusal beats a plausible guess.** Two agents declined to guess without hardware
  access, and that is why the right answer was found. `backend: "nav2"` refusing with a
  named missing link is the same norm expressed in code.
- **Trust pose, never twist — including the sign.** `along = dx·cos(yaw) + dy·sin(yaw)`,
  both fields from the same message. R1 §1.3 adds the Nav2-specific consequence: the
  behavior server projects twist forward for collision checking, so an inverted twist can
  abort a move with a **phantom collision**.
- **Never restart `micro-ros-agent`** (90–225 s reconnect). **One agent on the Pi at a time.**
  **Never publish `/cmd_vel`** unless motion is explicitly authorised — there is no gentle
  duty, so an accidental publish is never a gentle mistake.
- **This repo is public.** No secrets in committed files; the first draft of `NAV2_BRIEF.md`
  leaked the SSH password and had to be scrubbed.

---

## 6. Provenance

| Source | Status |
|---|---|
| `R1_AUTONOMY.md` | Read in full (47.4 KB). Cites fetched-and-verified upstream sources; marks unverified repos explicitly. |
| `R2_COORDINATES.md` | Read in full (37.3 KB). Repo line-citations spot-checked and **accurate** (`LIDAR_FIX_ENABLED:583`, the four wall-fit functions, `TRACK_M`/`MM_PER_TICK`). |
| `R3_UI.md` | Read in full, **twice** — it was revised mid-synthesis (34.6 KB → 38.6 KB) and the revision was re-read and incorporated. The revision added drag-to-re-zero, the yaw cone, and RViz's graduated `Frame Timeout` staleness idiom. Palette and `ABORT` claims verified in code; **Defect 2 found stale**, Defects 1 and 3 confirmed. |
| `SESSION_HANDOFF.md`, `NAV2_BRIEF.md`, `nav2/TF_TREE.md` | Read in full. Treated as **overriding** throughout. |
| `nav2_params.yaml`, `fpms_odom_tf.py`, `fpms_missions.py`, `fpms_lidar_ros.py`, `arena.ts`, `mission.ts`, `ArenaMap.tsx`, `MissionStrip.tsx`, `lidarCluster.ts`, both `wrangler` files | Read directly to verify report claims. |

All three reports arrived during synthesis after a ~20 minute poll; none was missing.
No claim in this file or in `knowledge.json` was invented — where a number has never been
measured on this rover, it is labelled as such.

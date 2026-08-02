# R3 — UI research: birdseye arena map + mission console

**Scope.** Best-in-class patterns for a robot operations console whose centrepiece is a
**fixed birdseye coordinate map with a moving robot**. Researched against RViz/RViz2,
Foxglove Studio, Webviz/Lichtblick, QGroundControl, ArduPilot Mission Planner, PX4/MAVLink,
Formant and Freedom Robotics, then reduced to changes implementable in our React +
TypeScript + Tailwind + canvas/rAF stack.

**Method.** Primary sources (cited inline, mostly plugin source and vendor docs rather than
blog posts) → design decisions → compatibility check against the project `dataviz` skill
(form heuristic, colour-by-job, the six palette checks, mark specs, accessibility pass).
**Every colour claim in §7 was computed**, not eyeballed: `dataviz/scripts/validate_palette.js`
was run against our *actual* arena floor (`#0b0f16`), not the skill's default dark surface.
Those runs found three defects in the shipping palette.

**Constraints honoured.** No rover access, no code edits (`ArenaMap.tsx` is owned by another
agent), no commits. This file is the only artefact.

**Our frame, restated so every recommendation below is unambiguous:** 1200 × 1200 mm arena,
origin **bottom-left**, **+x right, +y up**, heading **CCW from +x**, **90° = up**. Three
target regions + a start box. **The map never moves or rotates; only the rover glyph does.**

---

## 0. What `ArenaMap.tsx` already gets right

Recorded first, because several of these are things the reference tools get *wrong* by
default, and none of them should be regressed by a change below.

| Property | Reference-tool equivalent |
|---|---|
| Exactly one `ctx.rotate()` in the module, inside `drawRover`'s save/restore | Foxglove follow mode **"Off"**; RViz `Fixed Frame: map` + `TopDownOrtho` — literally the Nav2 shipped default |
| `FORWARD 90°` marker on the **static** layer, derived from `FORWARD_HEADING_DEG` | QGC compass rose — but ours cannot drift from the constant, which theirs can |
| Assumed vs measured expressed in the **glyph** (dash, hollow nose, halo), not only a badge | Stronger than RViz, which says nothing without a covariance topic |
| Trail lifts the pen on `TRAIL_BREAK` rather than drawing a leg never driven | Better than Mission Planner's purple track, which joins across resets |
| Absent values render `--`, never `0` ("THE ZERO RULE") | Matches MAVLink's explicit unknown sentinels (`eph = UINT16_MAX`, `satellites_visible = UINT8_MAX`) |
| Stale scan fades and is captioned `SCAN STALE n.ns` rather than freezing | RViz **Decay Time** / **Frame Timeout**; Foxglove's muted `(0.4s ago)` |
| Region identity stated four ways (wash, brackets, plate, glyph) | Satisfies the skill's "never colour alone" rule |
| Scale bar + cm axis ticks | **Nobody else ships these.** Not RViz, not Foxglove, not Webviz. Keep them; they are a genuine advantage |
| No per-frame allocation; static/dynamic layer split; 2 Hz message-edge HUD rebuild | — |

The gaps below are refinements to a good map, not a rewrite.

---

## 1. Reference implementations — the fixed world map

### RViz / RViz2 (ROS)
**The `Fixed Frame` property is the whole mechanism**, documented in source as *"Frame into
which all data is transformed before being displayed."*
([`visualization_manager.cpp`](https://raw.githubusercontent.com/ros2/rviz/rolling/rviz_common/src/rviz_common/visualization_manager.cpp)).
`map` → world static, robot moves through it. `base_link` → robot pinned, world sweeps
around it. Critically, **the frame choice is separate from the camera controller**
(`Orbit`, `XYOrbit`, `FPS`, `TopDownOrtho`, `ThirdPersonFollower`) — and Nav2's shipped
config is `TopDownOrtho` + `Fixed Frame: map`, i.e. exactly our north-up static arena
([`nav2_default_view.rviz`](https://raw.githubusercontent.com/ros-navigation/navigation2/main/nav2_bringup/rviz/nav2_default_view.rviz)).
*At a glance:* where the robot is in the world; a `Grid` whose **`Reference Frame` is a TF
frame, not the screen** (default `<Fixed Frame>`, `Plane Cell Count` 10, `Cell Size` 1.0 m),
and a `TF` display drawing labelled RGB axis triads (**x=red, y=green, z=blue**, an
industry-wide constant).
*Failure honesty:* the status line goes to `"No tf data. Actual error: …"` /
`"No transform from [frame] to [fixed_frame]"` and the object is simply **not drawn**.
Nothing is ever rendered at a guessed pose.

### Foxglove Studio
The most directly transferable control in any of these tools. Foxglove separates
**Fixed frame** ("the stationary world reference relative to which all other objects in the
scene are located") from **Display frame** ("the coordinate frame the camera follows"), and
exposes a four-way **Follow mode** ([3D panel](https://docs.foxglove.dev/docs/visualization/panels/3d)):

- **Pose** — "follows the display frame's position, roll, pitch, and yaw"
- **Heading** — "follows the display frame's position and yaw (heading), while pitch and roll are removed so the horizon stays level"
- **Position** — "follows the display frame's position only, **with orientation aligned to the fixed frame**"
- **Off** — "does not update viewport"

**Our map is permanently Follow = Off, fixed frame = arena.** Naming it in those terms in
the code makes the guarantee legible to anyone with robotics background.
Foxglove's own rationale ([discussion #465](https://github.com/orgs/foxglove/discussions/465))
is worth quoting to future maintainers: rendering everything through one root frame makes it
impossible to tell *which* transform is bad, and accumulated LiDAR "should blur in the moving
frame but appear sharp in the static frame" — **accumulation and decay are only meaningful
relative to a fixed frame.**
Two more copyable patterns: the [Indicator panel](https://docs.foxglove.dev/docs/visualization/panels/indicator)
uses ordered rules ("first matching rule wins") plus an explicit **`Fallback label` /
`Fallback color`** for the no-match case — the right shape for "no fix"; and the
[Plot panel](https://docs.foxglove.dev/docs/visualization/panels/plot) falls back to "most
recent sample at or before the cursor" rendered **muted with the elapsed age inline, e.g.
`1.234 (0.4s ago)`**. That is the cleanest staleness idiom found anywhere in this survey.

### Webviz (Cruise) / Lichtblick
Webviz's `FollowTFControl` compresses Foxglove's menu into **one tri-state button** —
tooltips cycle `"Follow [frame]"` → `"Follow Orientation"` → `"Unfollow"`, with the off state
rendering the frame name muted
([source](https://github.com/cruise-automation/webviz/blob/master/packages/webviz-core/src/panels/ThreeDimensionalViz/FollowTFControl.js)).
Cheaper UI than a four-item menu, and the muted-when-off treatment is a good model for any
toggle whose "off" is the safe default. Rendering is `regl-worldview`, published standalone.
[Lichtblick](https://github.com/lichtblick-suite/lichtblick) (a Foxglove fork) keeps the same
reference-frame + follow-mode model.

### ArduPilot Mission Planner
*At a glance:* HUD for attitude/mode; Flight Data map for where you are and where you were.
**The actually-flown track is a purple polyline, visually distinct from the planned mission
polyline, with an explicit `Clear Track` action**
([Flight Data](https://docs.ardupilot.org/planner/docs/mission-planner-flight-data.html)).
The **EKF chip** is the best compact uncertainty affordance in the survey: coloured by the
*highest normalised variance* in `EKF_STATUS_REPORT` — **green < 0.5, amber < 0.8, red > 0.8**,
where "0 means the estimate is very trustworthy and 1.0 is very untrustworthy" — and clickable
for per-axis detail. Prearm failures are one red HUD line in a fixed machine-generated format:
`PreArm: GPS x: Bad fix`, `PreArm: Gyros not calibrated`, `PreArm: Compass X not healthy`
([prearm checks](https://ardupilot.org/copter/docs/common-prearm-safety-checks.html)).
The waypoint grid computes per-leg **Dist**; **Home is row 0** with its own map marker.

### QGroundControl
*At a glance:* the toolbar answers "can I command this vehicle right now?" without opening
anything — Flight Status (Ready To Fly / Armed / Flying / **Communication Lost**), Flight Mode,
Vehicle Messages (turns red on important messages), GPS (**satellite count and HDOP**),
Battery, Telemetry RSSI, RC RSSI
([Fly View Toolbar](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view_toolbar.html)).
Crucially, **the reason you cannot arm lives inside the arm control**: selecting the
flight-readiness text opens arm / disarm / emergency-stop *and* the list of blocking issues.
**Plan View and Fly View are hard-separated** — missions are authored in one and read-only in
the other, and Save/Upload highlight on unsent changes (dirty state)
([Plan View](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/plan_view.html)).
Planned routes carry "numbered indicators", a distinct "Planned Home", and "flight path lines
and direction arrows".

### Formant
Teleop is a grid of draggable modules (video, map, charts, readout, joystick, buttons,
slider, terminal), and — the pattern worth stealing — **a lock button in the lower-right must
be disabled before any control command can be sent**, with the header carrying "ping to your
device" and boolean stream status
([docs](https://docs.formant.io/docs/getting-started-build-a-teleoperation-interface.md)).
That is an arm/disarm gate expressed as web UI. Formant also publishes **latency budgets you
can design against** ([FAQ](https://docs.formant.io/docs/faq-latency-of-formant-features.md)):
realtime video ~20 ms Formant-incurred, telemetry < 1.5 s, standard commands < 6 s, realtime
commands ~75 ms — and "realtime command latency can be identified as half of the Formant
displayed ping." **Latency is a first-class displayed number**, because an operator driving
through a lagged picture needs its size, not merely its existence.

### Freedom Robotics
Layers are `nav_msgs/OccupancyGrid` with per-layer transparency; paths are `nav_msgs/Path`
at 1–10 Hz; the robot glyph is a **circle with an orientation indicator**
([maps and navigation](https://docs.freedomrobotics.ai/docs/configure-maps-and-navigation.md)).
The standout: **relocalisation is direct manipulation** — drag the inner circle to translate,
the semi-transparent outer ring to rotate, **with the live LiDAR overlaid as the alignment
cue**, publishing `PoseWithCovarianceStamped` to `/initialpose`. Goals require an explicit
**CONFIRM** click, and a red **MANUAL CONTROL** button takes over by sending zero velocity
plus a cancel. Fleet status language is uniform: **active = green, inactive = yellow,
offline = red**, with a legend bottom-left on the fleet map.

---

## 2. Coordinate / grid presentation

1. **Two grid densities, one labelled.** RViz's `Grid` and Foxglove's grid layers use a coarse
   cell with optional subdivision; both anchor the grid **to a frame, not the screen**, which
   is why it stays put when the robot moves. We already do this (`GRID_MINOR_MM` under
   `GRID_MAJOR_MM`), labelled in **centimetres** — correct, because the arena is measured with
   a tape. Keep it.
2. **Label the origin explicitly.** Every tool draws a frame origin (RViz's axis triad;
   QGC/Mission Planner's Home marker at row 0). Our map labels `0…120` on both axes but draws
   **nothing at (0,0)**. A small `0,0` corner glyph with two short axis arrows (**→ +X**,
   **↑ +Y**) at the bottom-left removes the last ambiguity about which corner is the origin —
   static-layer work, free per frame. RViz's `Map` display also surfaces origin metadata as
   **read-only fields** ("Position of the bottom left corner of the map, in meters"); the same
   idea gives us a permanent `ORIGIN 0,0 BOTTOM-LEFT · ARENA 1200 × 1200 mm` caption.
3. **Scale bar.** Ours is `GRID_MAJOR_MM` long, bottom-centre, in cm. **No tool in this survey
   ships one** — keep it, and add the arena extent beside it so the map reads as a measured
   object rather than a grid.
4. **Cursor coordinate readout.** Standard in mapping GCS, absent from RViz/Foxglove/Webviz —
   and absent from ours: the dynamic canvas is `pointer-events-none` and the static canvas has
   no handler. A pointer-move readout (`X 742 mm · Y 318 mm`) is the cheapest possible
   "cm-level readout" and satisfies the `dataviz` skill's rule that a canvas chart ships a
   hover layer by default. Write to a ref on pointer events, render in the existing HUD scrim,
   zero per-frame cost.
5. **Snapping.** Snap only where a snap is *true*: the three region centroids, the start-box
   centre, and major grid intersections (100 mm). Free-form 1 mm placement on a 1200 mm arena
   at ~300–600 px is sub-pixel precision the operator cannot see and the rover cannot achieve.
6. **Zoom without losing the world frame.** The arena is fixed and fits on screen, so **do not
   add pan/zoom.** If a magnifier is ever wanted, use the GCS convention: an inset overview
   rectangle showing the zoomed extent within the full arena. Never allow view rotation.
7. **Why the map must not appear to move when the rover turns.** This is a solved question:
   Foxglove Follow **Off**/**Position** vs **Pose**; RViz `Fixed Frame: map` vs `base_link`;
   RViz's `ThirdPersonFollower` is explicitly the mode where "the camera also turns with the
   target frame" — the one you usually don't want. Rover-centric views are right for *sensor
   debugging* (is the scan aligned to the chassis?) and wrong for *spatial reasoning* (where is
   that obstacle in the arena?). We are permanently doing the latter. The corollary the tools
   also teach: because the view never rotates, **"forward" must be stated on the map** — which
   our static `FORWARD 90°` marker already does.

---

## 3. Uncertainty — making an estimated pose never look like a fix

The highest-stakes section for us: the rover is **dead-reckoned from an assumed start with
nothing correcting it**, and normally publishes no pose at all.

| Tool | Mechanism | What it buys |
|---|---|---|
| RViz2 | Position covariance as a **2σ ellipsoid** — eigendecomposition of the 3×3 block, "eigenvalues are the variances, so we take the sqrt to draw the standard deviation" × 2 ([`covariance_visual.cpp`](https://raw.githubusercontent.com/ros2/rviz/rolling/rviz_rendering/src/rviz_rendering/objects/covariance_visual.cpp)) | Uncertainty has a *size* comparable to the arena, at a stated sigma |
| RViz2 | **2D poses draw a yaw cone** instead of three orientation ellipses | Heading uncertainty is a wedge, not a line — exactly our case |
| RViz2 | `TF` **`Frame Timeout` 15 s: stale frames fade to grey for the last 1/3 of the timeout, then disappear** | Graduated decay, then honest absence |
| RViz2 | Missing TF → status error, object not drawn | Absence is visible; nothing drawn at a guessed pose |
| Mission Planner | `EKF` chip, tri-colour on max normalised variance (**green <0.5 / amber <0.8 / red >0.8**), click for per-axis | One glanceable scalar; detail on demand |
| QGC / MAVLink | `GPS_RAW_INT.fix_type` rendered **as words** — "No GPS receiver" / "no position fix" / 2D Fix / 3D Fix / RTK Float / RTK Fixed — plus `eph`, `satellites_visible`, and a **position uncertainty circle scaled to EPH** | The *class* of fix is text; the *magnitude* is a circle |
| Foxglove | `Fallback label` / `Fallback color` when no rule matches; `(0.4s ago)` inline on stale samples | "Unknown" is a designed state, not an empty one |
| Freedom | Drag-to-relocalise with the **live LiDAR as the alignment cue** | The operator can *fix* the drift, not merely observe it |

**What we should change.**

Today the assumed pose gets an amber dashed glyph, a hollow nose, and a **pulsing halo whose
radius grows and resets on a 2.6 s cycle** — deliberately "declining to name a radius nobody
has measured". That instinct is right about *honesty* and wrong about *usefulness*:

- An animation-driven radius is a **decorative** magnitude. It reads as a radar ping and
  invites the eye to skip it.
- It is **unreachable by `prefers-reduced-motion`.** `index.css` kills CSS animation globally,
  but the halo is drawn by `drawRover` inside the rAF loop and no CSS rule can touch it. The
  state must survive with motion off, so it needs a **static** representation.
- We *can* bound the drift honestly. Dead-reckoning error grows with **path length and
  accumulated turn**, both already tracked (`PoseTrail` gives cumulative distance; heading
  integration gives accumulated yaw). A ring of radius `r = k_d·Σ|Δs| + k_θ·Σ|Δψ|·L`, with the
  constants stated in the tooltip and the caption reading `DRIFT ≥ ±NN cm (modelled,
  unmeasured)`, is a **declared model** — far more honest than a fixed circle *or* a pulsing
  one, because it visibly grows the longer the rover drives without a fix. This is RViz's
  covariance ellipse adapted to what we actually know.
- **Add the yaw cone.** RViz draws a cone for 2-D pose orientation uncertainty precisely
  because a single heading ray overstates confidence. Our heading ray is dashed when assumed —
  good, but a shallow wedge either side of it (widening with accumulated turn) says the same
  thing quantitatively.

**Pose age is missing entirely.** Pose arrives on a 0.2 Hz heartbeat, so a displayed pose can
be 5 s old while the scan is 0.5 s old and nothing says so. Copy Foxglove's idiom verbatim:
render the pose readout muted with the age inline — `X 620 mm (4.2s ago)` — and copy RViz's
`Frame Timeout`: **grey the glyph progressively over the last third of a timeout window, then
stop drawing it and leave only the caption.**

**Keep the wording.** The HUD's `POSE ASSUMED` / `NOTHING REPORTED A POSITION` /
`DEAD-RECKONED · NOT LOCALISED` is a *class stated in words*, which is exactly QGC's
`fix_type` pattern and better than most commercial consoles. Add age and the modelled radius;
change nothing about the phrasing.

**The highest-leverage addition: drag-to-re-zero.** Freedom Robotics lets the operator drag
the robot glyph (inner circle = translate, outer ring = rotate) against a live LiDAR overlay
and publishes the result as the new pose estimate. We have the same preconditions — a live
2 Hz scan, a wall-bounded 1200 mm arena, and an origin file (`fpms_teleop_origin.json`). Let
the operator drag the ghost until the scan's walls line up with the arena border, then commit.
That converts our worst property (an unfixable accumulating drift) into a recoverable one, and
it makes every other pose-dependent feature — cross-track error, distance-to-next, occupancy —
worth building.

---

## 4. Route / plan visualisation — intended vs achieved

**How the tools separate them:**

- **Nav2/RViz (concrete, copyable):** `global plan` is **red `255,0,0`** with magenta pose
  arrows; `local plan` is **blue `0,12,255`** with no pose markers; footprint polygon green;
  global costmap α 0.3 under local costmap α 0.7. **Two different hues**, never one line
  ([`nav2_default_view.rviz`](https://raw.githubusercontent.com/ros-navigation/navigation2/main/nav2_bringup/rviz/nav2_default_view.rviz)).
- **Mission Planner:** planned polyline vs **purple actual track** + `Clear Track`.
- **QGC Plan View:** **numbered** waypoints, direction **arrows**, distinct **Planned Home**,
  and Save/Upload highlighting when the on-screen plan differs from the vehicle's (dirty state).
- **RViz `Odometry`:** breadcrumbs are **spatially decimated, not time-decimated** —
  `Position Tolerance` 0.1 m and `Angle Tolerance` 0.1 rad ("Distance… from the last arrow
  dropped, that will cause a new arrow to drop"), with `Keep` 100.

**What we should change:**

1. **Break the three-way blue collision (§7).** Planned route `rgba(125,211,252,…)`, measured
   trail `#38bdf8`, water station `#22d3ee` — three near-identical blues carrying three
   different meanings, with intent-vs-achievement resting entirely on a dash pattern. Give
   **intent** and **achievement** different **hues** (Nav2 does), and keep the dash as
   reinforcement.
2. **Decimate the trail spatially, not on the message edge.** `PoseTrail.push` currently
   appends on every pose message, so a stationary rover piles hundreds of samples on one point
   and the ring buffer's usable history shrinks to nothing while parked. Adopt RViz's rule:
   drop a sample only past a position **or** angle tolerance (say 15 mm / 0.1 rad).
3. **Number the waypoints** `1 2 3 …`, matching the mission strip's leg list, so "leg 2 failed"
   points at a specific dot (QGC).
4. **Direction chevrons on the plan spine**, one per leg. On a 1200 mm arena an operator
   otherwise cannot tell an out-and-back from a back-and-out.
5. **Highlight the active leg** — current leg 2 px full opacity, pending legs 1.5 px/60 %,
   completed legs stepped down further. Progress along the route is currently *unrepresented*:
   the plan renders identically before, during and after execution.
6. **Show deviation.** A hairline perpendicular from the rover to the active leg plus the
   **cross-track error in mm** beside the badge — the number that reveals dead-reckoning coming
   apart. One point-to-segment projection per frame.
7. **Distance-to-next** (`NEXT WP · 340 mm`). Mission Planner computes per-leg `Dist` for this.
8. **Keep plan under measurement.** The existing comment — *"a plan is intent; everything else
   on this canvas is measurement, and measurement should never be obscured by intent"* — is
   right and matches Nav2's layering. Do not change draw order.

---

## 5. LiDAR presentation

**What the tools do.** RViz's `PointCloud2` defaults to **"Flat Squares"** style (options
listed "in order of computational complexity"), exposes **two size modes** — `Size (m)` 0.01
world-scaled vs `Size (Pixels)` 3 screen-scaled — and a `Decay Time` of 0 meaning "only show
the latest points", with a `Color Transformer` of **Intensity / AxisColor / FlatColor / RGB8**
([`point_cloud_common.cpp`](https://raw.githubusercontent.com/ros2/rviz/rolling/rviz_default_plugins/src/rviz_default_plugins/displays/pointcloud/point_cloud_common.cpp)).
Foxglove mirrors this (Circle/Square/Cube, decay time, colour modes, chosen colour field).
Nav2 renders the **costmap as an occupancy grid** underneath the raw scan, because a grid
answers "can I drive there?" while points answer "what did the sensor see?".
Note that colour-by-class is *nobody's default* — flat or intensity is.

**What is readable on a small card** (worst case: a phone-width card, ~50 px per region):

1. **Points are not a series channel.** We colour four classes by hue on ~2 px squares with
   **no shape difference** — identity by colour alone at the size where hue perception is
   weakest, and the one place the map breaks a `dataviz` non-negotiable. The **track layer
   already carries shape** (circle for `TREE`, filled rect for `OBSTACLE`, unfilled for wall);
   that encoding is load-bearing and must never be removed. For raw points, either (a) vary
   point size by class (wall 1.6 px, tree/obstacle 2.4 px), or (b) **drop class colouring on
   raw points entirely, render them one recessive grey, and let the boxes carry class.**
   (b) is stronger — it matches how Nav2/RViz layer raw returns underneath and semantics on
   top, and it fixes the CVD warning in §7 without touching the track palette.
2. **Occupancy is the better small-card form — but not for us yet.** A 1200 mm arena at 50 mm
   cells is a 24 × 24 grid, legible at 50 px, and it directly answers "where can I drive?".
   But our scan is 2 Hz single-plane with a drifting pose, so accumulation would smear.
   **Keep points + tracks; revisit occupancy only after drag-to-re-zero or a real fix exists.**
   (Foxglove's own argument applies: accumulation is only meaningful relative to a fixed frame,
   and our rover→arena transform is the thing that is wrong.)
3. **Stale scans: fade, then stop.** Ours fades to α 0.3 and captions the age — better than
   most. Add RViz's second stage: past ~8 s **stop drawing the cloud entirely** and leave only
   the caption, mirroring `Frame Timeout`'s grey-then-disappear. A faint picture is still a
   picture, and operators plan against it.
4. **The `< 150 mm` danger ring is colour-alone.** The nearest return correctly gets a ring —
   that is the number the obstacle guard acts on — but in the danger case *only the hue
   changes*, and rose-vs-neutral fails CVD (§7, Defect 3). Add a second concentric ring, a `!`
   glyph, or a `NEAR 120mm` label.

---

## 6. Mission-control affordances

Drone GCS has solved this; port rather than reinvent.

1. **Two modes, hard-separated.** QGC's **Plan View** (author, edit, Upload with dirty-state
   highlight) vs **Fly View** (command and monitor; mission read-only). Never allow editing a
   route while it executes.
2. **A three-state arming ladder, not a boolean.** PX4 ships **Disarmed → Prearmed → Armed**,
   where prearmed brings up non-dangerous actuators while motors stay locked
   ([PX4](https://docs.px4.io/main/en/advanced_config/prearm_arm_disarm.html)). For us:
   `SAFE` (no motion possible) → `ARMED` (mission may start) → `RUNNING`. Formant's web analogue
   is a **lock button that must be disabled before any control command can be sent** — a single
   persistent widget, not a per-command state.
3. **Prearm reasons live inside the arm control.** Copy ArduPilot's format literally: one line,
   prefixed, first blocking reason in red — `PreArm: LiDAR not publishing`, `PreArm: pose never
   reported`, `PreArm: link down 12s`, `PreArm: rover not in start box`. QGC additionally offers
   **"Use preflight checklist"** and **"Enforce checklist"**, i.e. the gate can be advisory or
   blocking. Ship advisory first; make it enforcing once the checks are trustworthy.
4. **Confirm motion with a gesture, not a modal.** QGC: **"Slide to confirm operation."** with
   **hold-spacebar to confirm / X to cancel**, and the Fly Tools variant "click and hold the
   button until the action is accepted"
   ([HUD](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/fly_view/hud.html)).
   This beats `Are you sure?` because it is *effortful and continuous* — habituated
   click-through cannot complete it. NN/g is explicit that a bare confirmation is worse than
   nothing: *"the only sensible reaction is 'of course I want to do the thing I just told you to
   do'"*, and *"if you cry wolf too many times, people will stop paying attention"*
   ([NN/g](https://www.nngroup.com/articles/confirmation-dialog/)). Motion is not undoable,
   which is exactly why the effort belongs in the gesture rather than a dismissible prompt.
5. **Abort must never be conditional, and never confirmed.** ISO 13850 requires the actuator to
   be *"readily accessible"*, red on yellow, latching, with a deliberate reset that does not
   itself restart motion. `MissionStrip` renders `ABORT MISSION` **only when `running || blind`**
   — so the button's position changes with state, destroying the muscle memory that makes an
   abort fast. **Always render it** in a fixed position; disable it only when there is provably
   nothing to stop, and keep the footprint even then. Add a global keyboard shortcut (QGC
   precedent for keyboard equivalents), and preserve the existing "ungated by link checks"
   rule — that code comment is correct.
6. **Distinguish recoverable stop from latching stop.** PX4's Kill is **revertible within 5 s**,
   then auto-disarms; Flight Termination is not revertible without reboot
   ([PX4](https://docs.px4.io/main/en/advanced_config/flight_termination.html)). State which one
   our `ABORT` is, and what the recovery path is.
7. **Hold-to-drive with panic-release semantics.** Modern enabling devices are **three-position**
   (OFF–ON–OFF): released *or* squeezed hard both stop, because operators clench under stress
   (ISO 10218 / IEC 60947-5-8). Web analogue: `Joystick.tsx` should cancel drive on `pointerup`,
   `pointercancel`, **`blur`**, **`visibilitychange`**, and on a heartbeat gap — not only on
   pointer release. Freedom's red **MANUAL CONTROL** button models the takeover: send zero
   velocity *and* a cancel, not just a mode flag.
8. **Command acknowledgement.** QGC surfaces *"Vehicle did not respond to command: `<CMD>`"* on
   ACK timeout and treats unacknowledged as **failed**, not optimistically successful. Lock the
   control while a command is in flight; name the specific command in the failure.
9. **A persistent status strip that never scrolls away.** QGC's toolbar is the model: link, pose
   quality, battery, mode, arm state — each icon + tap-to-expand. Formant adds **ping**, and
   publishes budgets to design against (realtime commands ~75 ms, telemetry < 1.5 s). Our
   `MissionStrip` is close; make it sticky and add pose-quality and link-age chips.

---

## 7. Palette audit — computed, not eyeballed

The colour work is computable, so it was computed. All runs used our real arena floor as the
surface and `--pairs all` (map marks are scattered, not adjacent stacked bars, so all-pairs is
the correct gate):

```
node scripts/validate_palette.js "<hexes>" --mode dark --surface "#0b0f16" --pairs all
```

### Defect 1 — `assumed` and `OBSTACLE` are the *same hex*
`THEME_DARK.assumed === "#fbbf24"` and `CLASS_COLORS_DARK[3] === "#fbbf24"`. Identically in
light mode: `assumed === "#b45309" === CLASS_COLORS_LIGHT[3]`. So the amber ghost rover, the
amber assumed trail, and every OBSTACLE point are one colour. **The provenance channel and a
LiDAR class channel are literally indistinguishable.** Amber is a *status*, not a series —
reserve it for provenance and re-hue OBSTACLE.

### Defect 2 — the measured trail and the water station are the same blue
```
"#c084fc,#a3e635,#22d3ee,#fbbf24,#38bdf8"   zone-a, zone-b, water, assumed, trailMeasured
  [FAIL] Normal-vision floor  worst all-pairs #38bdf8 ↔ #22d3ee  ΔE 6.7  — below the 15 floor
  [FAIL] CVD separation       worst all-pairs #fbbf24 ↔ #a3e635  ΔE 1.4 (deutan)
```
ΔE 6.7 is a **hard fail for full-colour vision**: the measured trail crossing the bottom-left
water station is very nearly invisible. And the amber assumed-pose ghost sitting in the
top-right lime zone is ΔE 1.4 for a deuteranope — gone. The route ink `#7dd3fc` is in the same
blue family, making it three-way.

### Defect 3 — measured-green vs danger-rose fails CVD
```
"#fbbf24,#34d399,#38bdf8,#fb7185"   assumed, measured, trailMeasured, danger
  [FAIL] CVD separation  #fb7185 ↔ #34d399  ΔE 4.6 (deutan) · 3.0 (tritan)
```
`POSE MEASURED` (emerald) and `LINK DOWN` / near-obstacle `danger` (rose) are the same colour
to a deuteranope. Swapping to the reserved status steps does **not** fix it
(`#0ca30c ↔ #d03b3b` measures ΔE 4.1 deutan) — red/green never will. The skill's own rule is
the answer: **status colours ship with an icon + label, never colour alone.** The HUD badge
already carries text, so it is mitigated *there*; the `< 150 mm` danger **ring** is not.

### Passing checks worth keeping
- Zone hues `#c084fc / #a3e635 / #22d3ee` **pass** CVD (worst 9.9), normal-vision (23.8) and
  contrast (all ≥ 3:1) among themselves. They fail only the lightness *band* (L 0.72–0.85,
  brighter than the skill's dark band) — an accepted deviation, because our floor is `#0b0f16`,
  far darker than the skill's `#1a1a19` reference surface.
- LiDAR classes `#94a3b8 / #4ade80 / #fbbf24 / #f97316`: worst pair `#fbbf24 ↔ #4ade80`
  **ΔE 7.3 protan — the 6–8 WARN band**, legal *only* with secondary encoding. The track-shape
  encoding supplies it; the raw points do not (§5.1).
- `WALL #94a3b8` has chroma 0.035 — it "reads gray". That is *correct*: it is the de-emphasis
  ink, not a series slot. Keep it and don't count it as a categorical colour.

### A validated replacement for the dynamic layer
Reserve the categorical hues for the things that *move and change* (plan, driven track,
provenance, classes) and let static regions carry identity by glyph + bracket + label, which
they already do. Validated on our floor:

```
"#3987e5,#f97316,#fbbf24,#199e70"    plan-blue, driven-ember, assumed-amber, tree-aqua
  [PASS] Chroma floor         all 4 >= 0.1
  [PASS] CVD separation       worst all-pairs #199e70 ↔ #f97316  ΔE 8.5 (protan)   [target >= 8]
  [PASS] Normal-vision floor  worst all-pairs #fbbf24 ↔ #f97316  ΔE 17.4           [floor >= 15]
  [PASS] Contrast vs surface  all 4 >= 3:1
  (only the lightness-band check deviates — same accepted reason as the zones)
```
Reading: **planned = blue, driven = ember (the brand accent, and the thing that actually
happened deserves the accent), assumed = amber**, and every hue clears both the CVD and
normal-vision floors simultaneously. This also matches Nav2's precedent of two distinct hues
for plan vs execution. Re-run the validator for `--mode light` against `#ffffff` before
shipping the light table.

### Accessibility items still open
- **No text alternative for the canvas.** Add an `aria-live="polite"` visually-hidden summary
  (`"Rover at X 620 mm, Y 180 mm, heading 090°, pose assumed, scan 0.4 s old"`), rebuilt on the
  same 2 Hz message edge as the HUD strings. This is the skill's "a table view exists"
  requirement adapted to a map.
- **`prefers-reduced-motion` does not reach the canvas.** `index.css` disables CSS animation
  globally but cannot touch `drawRover`'s rAF halo pulse. Read the media query once in the
  effect, store it in a ref, and render a **static** ring when reduced motion is requested — the
  state must not depend on the animation.
- **No legend.** The map has ≥ 2 series (plan, trail, three point classes, three zones). A
  collapsed one-row key beneath the card — swatch + shape + word — is the dependable identity
  channel the skill requires; the on-map plates are direct labels that *supplement* it. Freedom
  Robotics puts exactly this bottom-left on its fleet map.

---

## 8. Recommendations ranked by operator value

| # | Change | Value | Effort |
|---|---|---|---|
| 1 | Always-present, fixed-position ABORT + global key; never conditional on state | Safety | S |
| 2 | Break the three-way blue collision: plan ≠ driven ≠ water station (validated set §7) | Safety | S |
| 3 | Free amber for provenance only; re-hue the OBSTACLE class | Safety | S |
| 4 | Modelled drift ring + yaw cone replacing the pulsing halo; static under reduced-motion | Safety | M |
| 5 | Pose age inline `(4.2s ago)` + graduated grey-out then drop (RViz `Frame Timeout`) | Safety | S |
| 6 | Drag-to-re-zero the assumed pose against the live scan (Freedom Robotics `/initialpose`) | Safety | L |
| 7 | Icon/label on the `<150 mm` danger ring (CVD failure today) | Safety | S |
| 8 | Active-leg highlight + waypoint numbers + direction chevrons | Task | M |
| 9 | Cross-track deviation hairline + mm readout | Task | M |
| 10 | Cursor coordinate readout on pointer-move | Task | S |
| 11 | Distance-to-next-waypoint in the HUD | Task | S |
| 12 | Origin `0,0` glyph + `→ +X` / `↑ +Y` axis arrows + arena-extent caption (static layer) | Orientation | S |
| 13 | Raw points to one recessive grey; class carried by the track boxes | Legibility | S |
| 14 | Spatially-decimated trail (15 mm / 0.1 rad tolerance) instead of message-edge push | Legibility | S |
| 15 | Hard cutoff (not just fade) for scans older than ~8 s | Legibility | S |
| 16 | Collapsed legend row beneath the card | Legibility | M |
| 17 | `aria-live` pose/scan summary for the canvas | A11y | S |
| 18 | `SAFE → ARMED → RUNNING` ladder with `PreArm: <reason>` lines | Safety | L |
| 19 | Slide/hold-to-confirm for mission start; no modal | Safety | M |
| 20 | Hold-to-drive cancelling on blur/visibilitychange/heartbeat gap | Safety | M |
| 21 | Command ACK tracking; "Rover did not respond to command: `<NAME>`" | Trust | M |
| 22 | Link ping / latency chip in the status strip | Trust | S |
| 23 | Plan/Drive mode separation (route read-only while running) | Trust | L |

---

## TOP 8 CHANGES — ArenaMap + mission UI

1. **Render `ABORT` always, in a fixed position, disabled rather than absent** — a control that
   moves or vanishes with state cannot be hit from muscle memory, and ISO 13850's "readily
   accessible" is a layout requirement, not a wording one.
2. **Re-hue the dynamic layer to the validated set (plan `#3987e5`, driven `#f97316`, assumed
   `#fbbf24`, tree `#199e70`)** — today the planned route, the measured trail and the
   water-station region are three near-identical blues, and `#38bdf8 ↔ #22d3ee` measures ΔE 6.7,
   a hard fail even for full-colour vision.
3. **Stop using amber for both "pose assumed" and the `OBSTACLE` LiDAR class** — they are
   literally the same hex in both themes, so the one channel that must never be misread (is this
   pose real?) shares a colour with ordinary sensor data.
4. **Replace the pulsing halo with a drift ring modelled from driven distance and accumulated
   turn, plus a yaw cone on the heading ray, drawn statically under `prefers-reduced-motion`** —
   RViz draws a 2σ ellipse and a yaw cone because an uncertainty with a *size* can be compared to
   the arena, and a canvas animation is unreachable by our reduced-motion CSS.
5. **Show pose age inline (`X 620 mm (4.2s ago)`) and grey the glyph out progressively before
   dropping it** — the pose heartbeat is 0.2 Hz against a 2 Hz scan, so the glyph can be five
   seconds stale while the cloud looks live; this is Foxglove's and RViz's staleness idiom exactly.
6. **Add drag-to-re-zero: let the operator drag the ghost rover until the live scan's walls line
   up with the arena border, then commit the new origin** — Freedom Robotics' relocalisation
   pattern turns our worst property, an unfixable accumulating drift, into a recoverable one and
   makes every pose-dependent feature below worth building.
7. **Highlight the active leg and add waypoint numbers plus direction chevrons to the route** —
   the plan renders identically before, during and after execution, so progress along the route is
   currently invisible; QGC numbers waypoints and arrows the path for precisely this reason.
8. **Add a pointer-move coordinate readout and an origin `0,0` marker with `→ +X` / `↑ +Y`
   arrows** — the map claims cm-level precision but offers no way to interrogate a location, and
   with the origin unmarked the bottom-left convention lives only in a source comment.

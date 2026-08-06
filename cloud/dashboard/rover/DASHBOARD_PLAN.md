# Dashboard: mission buttons on the LiDAR page, and live pose

Investigated 2026-08-04. **Most of this already exists** — far less to build than
expected. Two files to touch, one new component, one thing for the rover to
publish.

## The LiDAR page is already ~90% of "show everything live"

`frontend/src/pages/Lidar.tsx` (913 lines, route `/lidar`). Each `RoverLidar`
card (`:538`) ALREADY composes:

- `ArenaMap` **with the planned route** (`:785-790`) — route rendering needs NO
  work; `readPlan(mission_plan:<thing>)` feeds it at `:623`
- `MissionStrip` with a working abort (`:713`)
- clearance readout, broken-pose and stale-scan banners
- pose / plan / leg footer, polar `LidarView` inset
- six metric tiles: position, battery, travelled, LiDAR guard, nearest, points
- already subscribes `lidar:`, `pose:`, `drive:`, `mission_plan:`, `mission:`

**The only thing missing is mission COMMANDS.**

## `ArenaMap` needs no changes at all

    type Props = { thing, accent?, poseEnvelope?, route? }
    RoutePoint = { x_mm, y_mm, kind?, dock? }

`drawDynamic` already paints route -> trail -> deviation -> cloud -> rover -> HUD.
Static layer paints grid, zones, start box, FORWARD marker.

## Take the buttons from Mission.tsx, NOT Drive.tsx

`Mission.tsx` is the safer console. It has what Drive lacks:

- **an ARM gate** (`:437-451`), reset by abort
- **real two-step PLAN -> FOLLOW**: `planTarget` records `asked[name]`;
  `followTarget` re-checks `gateFor()` at fire time and REFUSES a stale or
  unrequested plan (`PLAN_MAX_AGE_MS` 180 s). Drive's second row is an
  unconditional RUN with no gate.
- per-target 2x2 cards laid out in physical corner positions
- `TEST ENCODERS` -> `read_encoders`

Drive.tsx has one thing Mission lacks: **`set_coordinate`** (`:1439`).

Payloads are identical in both and must not change:

    PLAN    mission  {name, backend, preview: true}
    FOLLOW  mission  {name, backend}
    ABORT   stop     {}
    ENCODERS read_encoders {}
    SET COORD set_coordinate {x_mm, y_mm}

All go through `apiPostJson('/api/control/<thing>/<action>', {params})` ->
backend -> MQTT `fpms/{thing}/commands/{action}`.

## The pose gap — one message clears "POSE ASSUMED"

`lib/arena.ts:431-508` reads `data.x_m`, `data.y_m` (**METRES**, multiplied by
1000) and `data.heading_deg`. The ASSUMED badge clears ONLY when both `x_m` and
`y_m` are finite; heading is ignored unless position is real.

Convention: origin bottom-left, +x right, +y up, heading degrees **CCW from +x**,
so **90 deg = up the arena**. Publish at >= 1 Hz to
`fpms/<thing>/telemetry/pose`:

    {"x_m": 0.972, "y_m": 0.228, "heading_deg": 90.0}

Values must sit within [0, 1.2] m (+/-240 mm slack) or the page rejects the pose.
**Never send `heading_deg: 0` as a placeholder** — that reads as "facing right".

Useful shortcut: `Lidar.tsx:610-613` already synthesises this envelope from
`mission:`/`drive:` mm poses via `poseEnvelopeFromMm`, so the badge clears as
soon as the executor publishes `x_mm`/`y_mm` on either channel.

## Implementation plan

1. **NEW `src/components/MissionConsole.tsx`** — lift `Mission.tsx` ~330-700 and
   ~1340-1540 verbatim: `armed` state, `planTarget`/`followTarget`/`abort`/
   `testEncoders`, `statusFor`/`gateFor`, `TargetCard`, `ConfirmButton`,
   `PlanLine`. Props `{thing, backend, caps, plan, feed}`. Payloads unchanged.
2. **`pages/Lidar.tsx`** — render `<MissionConsole/>` in `RoverLidar` after
   `MissionStrip` (`:713`); add `useRoverEvents(thing)` so a PLAN refusal is
   visible; port Drive's `SetCoordinate`; add a backend selector.
3. **`Mission.tsx` / `Drive.tsx`** — re-point at the shared component, keep both
   routes working.
4. **Rover** — publish `{x_m, y_m, heading_deg}` at >= 1 Hz.

**Change nothing in:** ArenaMap rendering, `mqtt_bridge` topic filters,
MissionStrip abort, clearance, battery, plan chips.

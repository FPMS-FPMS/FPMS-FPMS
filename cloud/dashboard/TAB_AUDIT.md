# FPMS Dashboard — Tab Audit

Read-only audit. No source file was modified by this audit.

- Repo: `C:/Users/ruchi/FPMS-FPMS`, branch `claude/fpms-aws-localstack-setup-s3pp8k`
- Live instance audited: `http://127.0.0.1:8010` (GET only; no control endpoint was ever POSTed)
- Live state at audit time: MQTT **not connected** (broker refused credentials), `messages_seen: 0`,
  `things_seen: []`, `aws.reachable: false`, `controls_disabled: false`, `role: edge`.
  So this audit observed the **rover-offline** case directly, which is the normal case.

## Which build is actually running

`/api/health` reports the live server serving
`C:\Users\ruchi\AppData\Local\Programs\FPMS Dashboard\_internal\frontend\dist`
via the `FPMS_FRONTEND_DIST` override (`override_honoured: true`).

That directory is byte-identical to the repo's `cloud/dashboard/frontend/dist`
(same asset hashes `index-sU1sq4v4.js` / `index-DKlN1Ws0.css`), so the live app **is** the
current build. The built bundle contains all eleven tab labels and the strings
`ZONE A`, `ZONE B`, `WATER`, `START`.

The operator's original "tabs missing" report was a stale server on `:8000`. That is consistent
with the code: `main.py` resolves `FPMS_FRONTEND_DIST` and falls back to the bundled dist, and
`/api/health.frontend` names the exact directory. **If tabs ever look wrong again, read
`/api/health` → `frontend.dist` and `frontend.built_at` first.** That field exists precisely for
this failure.

Caveat: the bundle was built at 14:51. `lib/arena.ts`, `components/ArenaMap.tsx` and a new
`pages/Mission.tsx` were being edited by concurrent agents *during* this audit (15:09–15:54), so
the running bundle predates those edits. Everything below about the *source* is current; the
running UI is one build behind on the arena labelling work.

---

## HEADLINE FINDINGS

### 1. The arena map is CORRECT and DOES render offline — on LiDAR and Drive

`src/lib/arena.ts` matches the operator's description exactly:

| region | id | corner | source |
|---|---|---|---|
| Zone A | `zone-a` | **top-LEFT** | `arena.ts` ZONES[0] |
| Zone B | `zone-b` | **top-RIGHT** | `arena.ts` ZONES[1] |
| Water station | `water-station` | **bottom-LEFT** (hatched, `REFILL`) | `arena.ts` ZONES[2] |
| Start box | `START_BOX` | **bottom-RIGHT** (dashed, deliberately not a zone) | `arena.ts` |

`ARENA_MM = 1200` (1200x1200 mm), origin bottom-left, single Y flip in `worldToCanvasY`.

They are drawn on the **static** canvas layer (`ArenaMap.tsx` `drawStatic`, zones loop and start-box
block), which is painted from `resize()` on mount — **not** from telemetry. Therefore the two zones,
the water station, the start box, the grid and the axis ticks all render with the rover offline.

- **LiDAR tab**: renders `ArenaMap` once per bay, and `bays` always unions in `rover1`/`rover2`,
  so two maps always render (`pages/Lidar.tsx`).
- **Drive tab**: renders `ArenaMap` in its own "Arena" card. The only condition is `thing ? … : null`,
  and `thing` falls back through `things[0] ?? bays[0]`, where `bays` always contains `rover1`.
  **`thing` is never null, so the Arena card always renders.** It sits mid-page — below STOP, link
  banner, rover selector, Health, Speed envelope, Speed floor and Mission — so it requires scrolling.
- **Control tab has NO arena map at all.** `pages/Control.tsx` does not import `ArenaMap`. If the
  operator expects the map on Control, it has never been there.

Offline the map is honest: `readPose(null)` puts the glyph at the start-box centre facing
`FORWARD_HEADING_DEG` (90°, up the arena) with `simulated: true`, the HUD reads
`POSE ASSUMED / NOTHING REPORTED A POSITION`, and Drive's header chip goes `chip-warn`
`pose · assumed`. This is the correct design.

### 2. There is no Mission tab — and one is half-built right now

The operator asked about "Mission". There is **no `/mission` route and no Mission nav entry** in
`App.tsx` / `NavBar.tsx`. Mission state appears only as `FleetMissionBar` in `Layout.tsx`, which is
passed `health.mqtt.things_seen` and **renders hidden when that list is empty** — i.e. the mission
bar is invisible whenever no rover has reported this session. Offline, mission is nowhere on screen.

During this audit a concurrent agent created `src/pages/Mission.tsx` (42 KB, untracked, renders
`ArenaMap`). As of audit end it is **not imported by `App.tsx` and not listed in `NavBar.tsx`**, so
it is dead code that produces no tab. Presumably mid-change — needs a route + nav entry to appear.

### 3. `App.tsx` has no catch-all route → unknown paths render a BLANK page

The backend SPA catch-all returns 200 for any non-`/api`, non-`/ws` path (verified live:
`/mission` and `/nonsense` both 200). `App.tsx`'s `<Routes>` has no `path="*"` fallback, so React
renders the nav, the footer and an **empty `<main>`**. A stale bookmark or a typo produces a page
that looks exactly like "the tab is broken/missing". This is very likely part of what the operator saw.

### 4. `StatusPill` renders green **"live · 0 pkts"** for a rover that has never sent a byte

`components/StatusPill.tsx:19`:

```ts
const stale = !connected || (lastAt !== null && Date.now() - lastAt > 3000);
```

With `lastAt === null` the freshness clause is skipped entirely → `stale === false` → emerald
pulsing dot and the word **"live"**. And `connected` is the **browser→dashboard** WebSocket, not the
rover: `main.py`'s `/ws/{channel}` accepts *any* channel name unconditionally (verified live — a
raw WebSocket to `/ws/lidar:rover1` reached state `Open` with MQTT down), so `connected` is true
whenever the backend is up.

Result offline: Camera, Thermal (via LiDAR/Drive/Control paths), LiDAR, Drive and Control card
headers can show a green **live** badge for a dead feed. On Camera the header literally reads
`not reporting` on the left and `live` on the right in the same row.

Second defect in the same component: staleness is only recomputed on re-render, and `useChannel`
only re-renders on message/open/close. A feed that simply **stops** keeps rendering "live" forever.
The correct 1 s tick pattern already exists in this repo in `lib/mission.ts`.

**This one fix removes the false "live" from five tabs.**

### 5. Drive: every motion control is enabled when the rover has never been heard from

`pages/Drive.tsx:373-383`:

```ts
const teleAgeMs = drive.lastAt === null ? null : now - drive.lastAt;
const teleStale  = teleAgeMs !== null && teleAgeMs > TELEMETRY_STALE_MS;
const rosDown    = tele?.ros_ok === false;   // undefined === false → false
const motionLocked = rosDown || teleStale;   // → false, offline
```

Never-seen is deliberately excluded from the lock set (documented in the comment). The consequence
is that offline, `motionLocked` is false, the red "motion controls are disabled" banner does not
render, and the joystick, Turn 15/45/90, Nudge F/B, all five mission buttons and Set-coordinate are
enabled and clickable. The joystick will POST `jog` at 10 Hz into the void.

Self-contradiction on the same screen: `LinkBanner` renders **"OFFLINE — never seen this session"**
directly above a page of fully live controls.

`Control.tsx` is the same by design (documented: "Every button on this page therefore stays
enabled") but has **no freshness signal at all** — no link banner, no tick, no lockout. The only
"something is wrong" signal is a log row appearing 3 s after a press.

---

## Tab-by-tab

| Tab | Route | Renders? | Gated by | Data sources | Offline behaviour | BUGS |
|---|---|---|---|---|---|---|
| **Overview** | `/` → `Intro.tsx` | yes | never hidden | `GET /api/info`, `GET /api/events?limit=8` (5 s); `SharePanel` → `GET /api/network` (4 s), `/qr`; `AlertsCard` → `GET /api/alerts/status` (5 s) | Renders **completely and identically to a healthy fleet**. No rover panel, no telemetry age, no "0 rovers" line anywhere. | Whole page is the string **"Loading FPMS…"** forever if `/api/info` fails (`.catch(()=>{})`, no retry). Hardcoded architecture inventory (`Rover 1 / Rover 2 / Sensors / Refill dock`, `IoT Core / Lambda / S3 / SNS`) reads as a live inventory but is a literal array. `AlertsCard` shows the definite claim "Not signed in" when its fetch fails. A personal email address is hardcoded as a default in `AlertsCard.tsx`. |
| **Control** | `/control` | yes | **`hqOnly`** — nav-hidden AND route-swapped for `NotAvailableRemotely` when `controls_disabled`; server 403s `/api/control/` | `POST /api/control/{thing}/{action}` (22 actions); WS `events` (opened twice — page + `capabilities.ts`), WS `mission:{thing}`; `GET /api/health` 5 s | Renders complete and operational-looking. "Command set" and "Acknowledgements" cards effectively empty. | **No arena map** (never had one). **No stale/offline indicator of any kind.** All motion buttons enabled. False green "live · 0 pkts" pills. `"stream up"` chip driven by the browser's own socket. `things_seen` never expires → "1 rover reporting" forever. Numeric formatting is otherwise rigorous (`—` for non-finite, explicit "not a measured zero"). |
| **Drive** | `/drive` | yes | **`hqOnly`** — same treatment as Control | `POST /api/control/{thing}/{jog,stop,nudge,turn,mission,set_coordinate}`; WS `events`, `drive:`, `mission:`, `pose:`, `mission_plan:`, plus `lidar:` opened inside `ArenaMap` | Arena card **renders with zones + water station + start box**; pose correctly `assumed`. Health card all `—`. `LinkBanner` says "OFFLINE — never seen this session". | **Motion lockout never engages** (finding 5). Green "live · 0 pkts" in 3 places incl. a Health card whose every field is `—`. `sent · vx 0.00 · wz 0.00` shown before anything was ever sent. Set-coordinate prefilled `0/0` and enabled — one click publishes "you are at the origin". Speed-envelope shows compiled-in `CAP_DEFAULTS` at full precision (mitigated by a `· default` suffix). |
| **Devices** | `/devices` | yes | **`hqOnly`** — nav-hidden + route-swapped; server 403s `/api/discovery/*` | `GET /api/discovery/interfaces`, `GET /api/aws/things`, `GET /api/health` 5 s, `POST /api/discovery/{scan,ssh,provision}` | **Best-behaved page in the audit.** Separates "registered" from "reporting"; every registered Thing gets an amber **silent** chip with a real explanation. | Minor: a failed `/api/aws/things` is swallowed, so "No Things yet" is asserted from a failed request. |
| **AWS** | `/aws` | yes | **INCONSISTENT** — `hqOnly` in NavBar (tab hidden) but **`App.tsx` renders it unguarded** and no `/api/aws/*` path is in `auth._PRIVILEGED_PREFIXES`. Typing the URL over the public link gives a fully working AWS page. | `GET /api/aws/services` (5 s), `POST /api/aws/verify` | Unaffected by the rover, but shows the worst zero-defaulting. | **Five gauges render a big mono `0`** when the payload is absent or the fetch failed — Things, Objects, Functions, Topics, Log Groups (`?? 0` / `.length` on `?? []`). Indistinguishable from "genuinely zero". They also read `0` on first paint. Page claims **"Live inventory"** with no freshness check; a failed poll leaves the last good `inv` on screen. `sum()` over `undefined` object counts renders `NaN`. |
| **LiDAR** | `/lidar` | yes | **not gated** — always visible | WS `lidar:`, `pose:`, `drive:`, `mission_plan:` per bay; `useMission` → WS `mission:`; `GET /api/health` 5 s; `POST /api/control/{thing}/stop` for abort | **Arena renders, both bays, zones + water station + start box.** Scan-staleness handled well: a loud red "map is not current and the rover is driving" banner and a quieter amber note when idle. | False green "live · 0 pkts" via `StatusPill`. `ArenaMap`'s HUD status word (`AWAITING SCAN` / `LINK DOWN`) is only rebuilt when the scan seq or pose prop changes, so offline it latches on the first frame and never re-evaluates. |
| **Camera** | `/camera` | yes | **not gated** | WS `camera:rover1` and `camera:rover2` (both always opened); `GET /api/health` 5 s; `POST /api/rover/{thing}/{connect,disconnect}` | Two bays render, both empty; `CameraView` shows "waiting for camera feed…". | **Claims a clear frame that never existed**: `Detections 0` + **"Nothing detected in this frame."** with zero frames received — the wrong answer for a fire-detection dashboard. Green "live · 0 pkts" beside "not reporting" in the same header row. A hardcoded benchmark ("Measured on this hardware: 20 ms inference … 28 fps") is printed unconditionally next to a dead feed. `CameraView` renders `<img src="data:image/undefined;base64,undefined">` and sets canvas size to `NaN` if an envelope arrives with detections but no `frame`. `cmd()` has no `catch` → a 502 from `/api/rover/...` is an unhandled rejection with no user feedback. |
| **Thermal** | `/thermal` | yes | **not gated** | WS `thermal:{thing}` and `thermal-analysis:{thing}`, both **null-guarded on `thing`**; `GET /api/health` 5 s; `POST /api/rover/{thing}/{connect,disconnect}` | **Correct.** Title reads "no rover reporting"; the null guard means no socket opens, so `StatusPill` correctly shows amber **NO SIGNAL**; explicit "waiting for thermal stream…" and "Waiting for thermal frames to analyze…"; severity defaults to `unknown` so no false CRITICAL. | `ThermalView` and `AnalystPanel` are **not** wrapped in `ErrorBoundary` and index deeply (`d.stats.max_c.toFixed(1)`, `grid[y][x]`) — a partial envelope crashes the fire-warning page. Card titled "Live thermal grid" regardless. Thresholds text hardcoded rather than read from the payload. Connect button enabled with `thing === null` and bails silently. |
| **Analyst** | `/analyst` | yes | **not gated** | `GET /api/analyst/status`, `GET /api/analyst/report?thing=` (8 s), `POST /api/analyst/chat`, `GET /api/health` 5 s | Backend is honest — emits "ROVER1 is silent — no sensor stream in the last 5 seconds" with a `no telemetry` concern. Frontend's freshness grid correctly shows `never`, not `0s ago`. | Headings say **"Live rover analysis"** / "Live analysis" while the tiles below say `silent / never`. Rover selector falls back to hardcoded `["rover1","rover2"]` — directly contradicting the comment above it — and `thing` defaults to `"rover1"`. `loadReport` has no `catch`: an error leaves "Generating first report…" forever plus an unhandled rejection every 8 s. |
| **Terminal** | `/terminal` | yes | **`hqOnly`** — nav-hidden + route-swapped; server closes `/ws/term/` with 1008; local shell additionally LAN-only | `GET /api/aws/things`; WS `/ws/term/ssh?host=…`, WS `/ws/term/local` | xterm always mounts; prints "FPMS terminal ready." | Header chip goes green **"connected"** on WebSocket open — *before* SSH is attempted (backend accepts the socket, then dials). Pointing at a dead rover shows "connected". Quick-connect chips come from registry `ip` attributes with no cross-check against who is actually reporting. `onData`/`onResize` handlers are never disposed and accumulate per connect. **The SSH password is passed in the WebSocket URL query string**, where it lands in access logs and browser history. |
| **Install** | `/install` | yes | **INCONSISTENT** — `hqOnly` in NavBar (tab hidden) but `App.tsx` renders it unguarded and no server rule covers `/api/downloads` | `GET /api/downloads`; `SharePanel` → `GET /api/network` (4 s), `/qr`, tunnel/firewall POSTs | Unaffected by rover state. | If `/api/downloads` fails, the platform grid renders **nothing** — heading, then a silent gap, then the next section. No loading state, no error, no retry. `humanBytes` lacks the `n === 0` guard its twin in `Aws.tsx` has. |

---

## Exactly what disappears, and when

`App.tsx` computes `const locked = !!status.controls_disabled;` from `GET /api/auth-status`.
`controls_disabled` is true when **safe mode is on AND the visitor arrived over the public link**
(host/`x-forwarded-host` matches a tunnel suffix), or when the process runs with `FPMS_ROLE=cloud`.

When `locked` is true, `NavBar` filters out every tab flagged `hqOnly`:

**Hidden: Control, Drive, Devices, AWS, Terminal, Install (6 of 11).**
**Still visible: Overview, LiDAR, Camera, Thermal, Analyst (5 of 11).**

A blue "Live view" banner appears under the nav explaining it.

Gating inconsistencies worth fixing:

1. **AWS and Install are hidden but not blocked.** `NavBar` marks them `hqOnly`, but `App.tsx` has no
   `locked ?` guard on `/aws` or `/install`, and neither `/api/aws/*` nor `/api/downloads` is in
   `auth._PRIVILEGED_PREFIXES`. A public visitor who types the URL gets the full page, including
   `POST /api/aws/verify` and `POST /api/aws/register-thing`.
2. **`POST /api/rover/{thing}/{action}` is not privileged.** It is reachable over the public link and
   publishes `fpms/{thing}/commands/{connect|disconnect}`. Unlike `/api/control/`, it does **not**
   validate `thing` against `THING_RE`, so `thing` is interpolated raw into an MQTT topic.
3. `/api/terminal` is listed in `_PRIVILEGED_PREFIXES` but **no such route exists** — dead prefix.
4. `POST /api/alerts/signin` (accepts mail credentials) is likewise not privileged.

At the time of audit `controls_disabled` was **false** on the live instance, so **no tab was hidden**.
If the operator sees tabs missing again, `GET /api/auth-status` answers it in one line.

## Backend cross-check — every channel the frontend opens exists

`mqtt_bridge.TOPIC_FILTERS`:
`fpms/+/telemetry/{lidar,camera,thermal,pose,drive,mission,mission_plan}` and `fpms/+/events/#`.
Topics map to WS channels as `{subtype}:{thing}`, plus a locally derived `thermal-analysis:{thing}`
and the singleton `events`.

Frontend opens: `lidar:`, `camera:`, `thermal:`, `thermal-analysis:`, `pose:`, `drive:`, `mission:`,
`mission_plan:`, `events`, `/ws/term/ssh`, `/ws/term/local`. **All exist. No orphan channel.**

Every HTTP endpoint the frontend calls exists (confirmed against the live `/openapi.json`).
There are no routers — every route is declared directly on `app` in `main.py`.

Two backend behaviours matter for offline honesty:

- **No fabrication.** No `random`, no synthetic telemetry, no `or 0` numeric defaults anywhere in the
  backend. On MQTT disconnect the bridge broadcasts nothing and notifies no WS client; sockets just
  go quiet. `/api/health.ok` is genuinely `bool(mqtt.connected)`. `publish_command` returns an
  explicit `"MQTT bridge is not connected"` rather than a false success.
- **`hub._latest` replays forever with no TTL.** Every new `/ws/{channel}` subscriber immediately
  receives the last frame ever broadcast on that channel, with no expiry. Combined with
  `lib/ws.ts` setting `lastAt: Date.now()` on receipt (it never reads the envelope's own `ts`), a
  rover that died an hour ago produces a green "live · 1 pkts" pill on a fresh page load. Not
  triggered at `messages_seen: 0`, but triggered by any rover that dies mid-session.
  `analyst.py` reads the same cache, though it does compute real ages and guards with a 5 s cutoff.

Also: `mqtt_bridge.things_seen` is a set that only ever grows for the life of the process. Nothing
ever leaves the roster, so "N rovers reporting" and the `reporting` tooltips on Control/Drive stay
true forever after a rover's last packet, and the `· silent` marker never returns.

---

## Ranked fix list

1. `components/StatusPill.tsx:19` — treat `lastAt === null && messages === 0` as stale, and add a 1 s
   tick (copy the pattern in `lib/mission.ts`). Stop passing the browser's own WS state as
   `connected`. **One fix, five tabs.**
2. `App.tsx` — add a `path="*"` route. An unknown URL currently renders a blank page, which is
   indistinguishable from "the tab is broken".
3. `pages/Drive.tsx:373-383` — either engage the lockout on never-seen after a grace period, or
   render the controls disabled-with-reason until first telemetry. Today: `LinkBanner` says OFFLINE
   directly above a live joystick.
4. `pages/Camera.tsx` — never print `Detections 0` / "Nothing detected in this frame." when
   `stream.data` is null. Pass the already-computed safe `frame` into `CameraView` and guard the
   `data:image/undefined` path.
5. `pages/Aws.tsx` — render `—` or a skeleton instead of `0` in all five gauges when `inv` is null.
6. Wire `pages/Mission.tsx` into `App.tsx` and `NavBar.tsx`, or delete it. Right now it is a 42 KB
   file that produces no tab.
7. Close the gating gap: guard `/aws` and `/install` in `App.tsx` to match their `hqOnly` flags, and
   add `/api/aws/`, `/api/downloads`, `/api/rover/` and `/api/alerts/signin` to
   `auth._PRIVILEGED_PREFIXES`. Apply `THING_RE` in `rover_command`.
8. Add a `catch` + visible error state to `Intro.tsx` (`/api/info`), `Analyst.tsx`
   (`/api/analyst/report`) and `Install.tsx` (`/api/downloads`). All three currently fail into a
   permanent loading string or a silent gap.
9. Wrap `ThermalView` / `AnalystPanel` in `ErrorBoundary` and guard their payload indexing.
10. `pages/Terminal.tsx` — set `connected` only after the backend confirms the SSH session; move the
    SSH password out of the WebSocket query string.
11. Expire `mqtt_bridge.things_seen` (or track last-seen per thing) so "reporting" can go false.

## Not verified

- Behaviour with `controls_disabled: true` was **not** observed live — the running instance reports
  `false`. The gating above is read from source, not exercised.
- No control endpoint was POSTed, so ack paths, jog behaviour and mission execution are unexercised.
- The concurrent edits to `arena.ts`, `ArenaMap.tsx` and the new `Mission.tsx` were in flight during
  this audit; findings about those three files describe the working copy as of ~15:55 and may have
  moved since.

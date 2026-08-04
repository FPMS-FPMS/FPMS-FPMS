import { useEffect, useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { StatusPill } from "../components/StatusPill";
import { LidarView } from "../components/LidarView";
import { ArenaMap } from "../components/ArenaMap";
import { ErrorBoundary } from "../components/ErrorBoundary";
import { MissionStrip } from "../components/MissionStrip";
import { MissionConsole, useMissionConsole } from "../components/MissionConsole";
import { useChannel } from "../lib/ws";
import { apiPost } from "../lib/api";
import { useThings } from "../lib/things";
import {
  poseEnvelopeFromMm,
  readPlan,
  readReplan,
  useMission,
  useRoverEvents,
  type MissionPlan,
  type ReplanDetail,
} from "../lib/mission";
import {
  emptyCapabilities,
  useRoverCapabilities,
  type RoverCapabilities,
} from "../lib/capabilities";
// Arena geometry is READ, never restated. ARENA_MM is the single source of
// truth for the pose sanity check below and ZONES/START_BOX are the single
// source of truth for the region key — a copy of either here is a copy that
// goes stale the first time the arena is rescaled.
import {
  ARENA_MM,
  RANGE_SATURATION_FRAC,
  START_BOX,
  ZONES,
  ZONE_AHEAD_OF_START,
  shortCorner,
  type Zone,
} from "../lib/arena";

/**
 * THIS IS THE TAB PEOPLE WATCH WHILE THE ROVER MOVES.
 *
 * That is the whole reason the mission belongs here and not only on Drive. An
 * operator standing next to the arena has the map open, not the joystick page —
 * so the map has to answer, on its own, whether the machine in front of them is
 * driving itself, where it thinks it is going, and whether what they are
 * looking at is current.
 *
 * ---------------------------------------------------------------------------
 * WHAT THE LAST BAD RUN LOOKED LIKE, AND WHAT THIS PAGE NOW DOES ABOUT IT
 * ---------------------------------------------------------------------------
 * A mission ran at full speed, outran its own 11 Hz odometry and hit something.
 * Two numbers said so while it was happening and neither was in the operator's
 * eye-line:
 *
 *   FRONT CLEARANCE fell 989 mm -> 117 mm. It was a 12 px cell in a grid of six
 *                   equally-weighted metrics. It is now the largest thing on the
 *                   card, it is coloured by state, and it shouts when it closes.
 *
 *   REPORTED POSE   read (625, -2720) — 2.9 m outside a 1.2 m arena — while the
 *                   rover had physically moved under a metre. The map drew it
 *                   anyway, off-canvas, which looks like a rendering bug rather
 *                   than what it was: a broken estimate. A pose outside
 *                   [0, ARENA_MM] is now REFUSED as a position. It is not
 *                   plotted, it is printed in full with the bound it violated,
 *                   and the map falls back to the assumed start so its own
 *                   POSE ASSUMED badge stays honest.
 *
 * Three things were added earlier for the same reason, each replacing an
 * assumption the page used to make silently:
 *
 *   MISSION STATE   a strip per rover. Unknown phases count as RUNNING.
 *   PLANNED ROUTE   the same preview the Drive tab shows, on the same map, so
 *                   "where is it going" does not require switching tabs.
 *   MISSION CONTROL the console itself — ARM, PLAN, FOLLOW, SET COORDINATE —
 *                   is rendered here from `components/MissionConsole`, the same
 *                   component the Mission page is built from. It is NOT a
 *                   second copy: the arm gate, the plan-before-follow rule and
 *                   the confirm are stated once and inherited. Watching the map
 *                   and starting the run were on different tabs, so the
 *                   operator planned on one and watched on the other, which is
 *                   how a route gets followed without being read.
 *   REAL POSE       the map used to draw every rover at the assumed start
 *                   corner, because `readPose` reads the rover-agent heartbeat
 *                   and that heartbeat carries no position. The mission
 *                   executor and the teleop bridge both publish a real
 *                   dead-reckoned pose in millimetres; when one exists AND it
 *                   is inside the arena it is used, and otherwise the old
 *                   fallback still raises the SIMULATED badge exactly as before.
 */

/** Per-bay accents, cycled so a third rover is not drawn in rover1's colour. */
const ACCENTS = ["#f97316", "#38bdf8", "#a3e635", "#c084fc"];

export default function Lidar() {
  // The bays were hardcoded to rover1/rover2, so a fleet whose live unit was
  // named anything else got two permanently empty maps while its scans streamed
  // past. Both defaults are still shown, so a bay that has not reported is
  // visibly absent rather than silently missing (same reasoning as Camera).
  const things = useThings();
  const bays = Array.from(new Set([...things, "rover1", "rover2"])).sort().slice(0, 4);

  // Subscribed ONCE for the page and handed down. Each card needs the mission
  // executor's announced `front_stop_mm` to colour its clearance readout, and a
  // hook per card would open one accumulator per bay over the same shared
  // `events` stream for a value that is identical across all of them.
  const capsByThing = useRoverCapabilities();

  return (
    <div className="space-y-6">
      <div className="flex items-end justify-between">
        <div>
          <div className="lbl">Page 2</div>
          <h1 className="h-page mt-1">LiDAR — arena maps, one per rover</h1>
        </div>
        <div className="text-xs text-slate-500">
          {things.length
            ? `${things.length} rover${things.length > 1 ? "s" : ""} reporting: ${things.join(", ")}`
            : "no rovers reporting"}
        </div>
      </div>

      {/* The regions the maps below are painted with, named in text.
          Deliberately above the maps: an operator who cannot yet tell the two
          fire zones apart on a 400 px canvas can read which corner each one is
          in here, and the corner is the one description the person standing at
          the arena and the code running the mission cannot disagree about. */}
      <ErrorBoundary label="Arena regions">
        <ZoneKey />
      </ErrorBoundary>

      <div className="grid gap-6 lg:grid-cols-2">
        {bays.map((t, i) => (
          <RoverLidar
            key={t}
            thing={t}
            accent={ACCENTS[i % ACCENTS.length]}
            caps={capsByThing[t] ?? emptyCapabilities()}
          />
        ))}
      </div>

      <Card>
        <CardHeader title="How this stream works" subtitle="Behind the scenes" />
        <p className="text-sm leading-relaxed text-slate-300">
          Each rover publishes 360-range LiDAR frames on
          {" "}<code className="rounded bg-black/50 px-1 py-0.5 text-xs">fpms/&lt;rover&gt;/telemetry/lidar</code>{" "}
          to the local Mosquitto broker (standing in for AWS IoT Core). The publish rate
          is set on the rover (<span className="font-mono">FPMS_LIDAR_HZ</span>) rather
          than fixed here, so each card shows the rate it is actually receiving instead
          of a number this page would go on printing after the rover was retuned. This
          dashboard subscribes over WebSocket — no cloud round-trip, no per-frame cost.
          Data never leaves your LAN.
        </p>
        <p className="mt-3 text-sm leading-relaxed text-slate-400">
          The arena map is world-fixed: the {ARENA_MM / 10} cm x {ARENA_MM / 10} cm
          grid, the two fire zones and the water station stay put, and only the
          rover moves within them. Returns are segmented into walls, trees and
          unclassified obstacles, then voted across five frames so the labels hold
          still.
        </p>
        <p className="mt-3 text-sm leading-relaxed text-slate-400">
          The rover glyph is drawn from the best pose available, and the card says
          which one: the mission executor's while a mission is running, the teleop
          bridge's otherwise. Both are <b>dead-reckoned from an assumed start</b> —
          nothing on this rover localises against the map, so neither is a
          measurement of where the rover is, only an integration of where it was
          told it began. They drift, and a{" "}
          <span className="font-mono">pose assumed</span> chip appears when the run
          started from a position that was assumed rather than set. A reported
          position outside the arena is refused outright rather than plotted.
          When nothing publishes a position at all, the rover falls back to a known
          start corner and the map raises its own SIMULATED badge. The dashed line
          is the route from the last <b>PLAN</b> preview — pressed on this card, on
          the Mission tab or on Drive, since all three ask the same executor; it
          is nominal intent, and the executor re-measures its bearing after every
          leg and inserts corrections, so the driven path will not match it exactly.
        </p>
      </Card>
    </div>
  );
}

/* -------------------------------------------------------------- arena key --- */

/**
 * The named regions of the arena, in text, straight out of `lib/arena`.
 *
 * NOTHING HERE IS TYPED IN. The labels, the corners, the roles, the colours and
 * the coordinates are all read from `ZONES` and `START_BOX`, which are the same
 * constants `ArenaMap.drawStatic` paints from — so this key and the picture
 * below it cannot disagree, and moving a zone moves both.
 *
 * Bare numbers are avoided on purpose. The code calls the top-left zone `m1` and
 * the top-right one `m2`, while an operator standing at the start box calls the
 * one in front of them "Zone 1" — which is `m2`. Every row therefore carries the
 * physical CORNER, and the row that is straight ahead of the start box says so
 * (derived by `ZONE_AHEAD_OF_START` from the geometry, not asserted here).
 */
function ZoneKey() {
  const regions: Array<{
    id: string;
    label: string;
    corner: string;
    role: string;
    stroke: string;
    x: number;
    y: number;
    ahead: boolean;
    note: string;
  }> = [
    ...ZONES.map((z: Zone) => ({
      id: z.id,
      label: z.label,
      corner: z.corner,
      role: z.role,
      stroke: z.stroke,
      x: z.x_mm + z.w_mm / 2,
      y: z.y_mm + z.h_mm / 2,
      ahead: ZONE_AHEAD_OF_START?.id === z.id,
      note:
        z.kind === "fire"
          ? "Mission destination — the rover drives here and discharges."
          : "Refill point — the rover drives here to take water ON BOARD. Not a target.",
    })),
    {
      id: "start-box",
      label: START_BOX.label,
      corner: START_BOX.corner,
      role: "ASSUMED START",
      stroke: "#94a3b8",
      x: START_BOX.x_mm + START_BOX.w_mm / 2,
      y: START_BOX.y_mm + START_BOX.h_mm / 2,
      ahead: false,
      note:
        "Not a destination. This is where the operator is asked to PUT the rover, and where the map draws it when nothing publishes a position.",
    },
  ];

  const fireCount = ZONES.filter((z) => z.kind === "fire").length;

  return (
    <Card>
      <CardHeader
        title="Arena regions"
        subtitle={`${fireCount} fire zones, the water station and the start box — as drawn on every map below`}
        right={
          <span className="chip font-mono" title="Read from ARENA_MM in lib/arena">
            {ARENA_MM} × {ARENA_MM} mm
          </span>
        }
      />
      <div className="grid gap-3 sm:grid-cols-2 lg:grid-cols-4">
        {regions.map((r) => (
          <div
            key={r.id}
            className="rounded-xl border border-white/5 bg-black/30 p-3"
            style={{ borderLeft: `3px solid ${r.stroke}` }}
          >
            <div className="flex items-center gap-2">
              <span
                className="inline-block h-3 w-3 shrink-0 rounded-sm"
                style={{ background: r.stroke }}
                aria-hidden
              />
              <span className="font-mono text-sm font-semibold text-slate-100">
                {r.label}
              </span>
            </div>
            <div className="mt-1 font-mono text-[11px] uppercase tracking-wider text-slate-400">
              {r.corner}
            </div>
            <div className="mt-0.5 font-mono text-[11px] text-slate-500">
              {r.role} · centre {Math.round(r.x)}, {Math.round(r.y)} mm
            </div>
            {r.ahead && (
              <div className="mt-2 rounded border border-ember-500/40 bg-ember-500/10 px-2 py-1 text-[11px] text-ember-200">
                Straight ahead of the start box — this is the one an operator
                calls “Zone 1”.
              </div>
            )}
            <div className="mt-2 text-[11px] leading-relaxed text-slate-500">{r.note}</div>
          </div>
        ))}
      </div>
      <p className="mt-3 text-xs text-slate-500">
        No label on these maps is ever a bare number. The mission code calls the
        top-LEFT zone <span className="font-mono">m1</span> and the top-RIGHT one{" "}
        <span className="font-mono">m2</span>, but the rover starts in the
        bottom-RIGHT box facing up the arena, so the zone an operator would call
        “the first one” is <span className="font-mono">m2</span>. Every region is
        therefore identified by the corner it physically occupies, which is the
        one description that cannot be read two ways. The water station is drawn
        hatched with a droplet rather than merely in a different hue — mistaking
        the refill point for a target sends the rover to spray it.
      </p>
    </Card>
  );
}

/* ------------------------------------------------------------- clearance --- */

/**
 * The executor's front cone, in degrees — `FRONT_CONE_DEG` in fpms_missions.py.
 *
 * Restated here only because the value is needed to derive the same number from
 * a raw scan when the executor is not publishing one. When the executor IS
 * publishing, its own `front_mm` wins and this constant is not used.
 */
const FRONT_CONE_DEG = 30;

/**
 * Fallback stop threshold, millimetres — `FRONT_STOP_MM` in fpms_missions.py.
 *
 * Used only until the executor announces `limits.front_stop_mm` on
 * events/online. That message is published once and not retained, so a
 * dashboard opened after the rover booted has never seen it, and colouring the
 * clearance readout "unknown" for the whole session would be worse than
 * colouring it against the value the rover ships with.
 */
const FRONT_STOP_FALLBACK_MM = 120;

/**
 * How much headroom above the stop threshold still counts as CLOSING.
 *
 * A presentation constant, not a rover setting, and deliberately generous: the
 * run this page was rebuilt for went 989 mm -> 117 mm, and the operator needed
 * the readout to change character on the way down rather than at the bottom.
 */
const CLOSING_FACTOR = 3;

type ClearanceState = "clear" | "closing" | "blocked" | "unknown";

/**
 * Front clearance derived from a raw scan, in millimetres.
 *
 * WHY THIS EXISTS. `front_mm` comes off the mission executor's snapshot, and the
 * executor only publishes while it is running. An operator driving by hand, or
 * watching a rover whose executor is down, had no clearance number at all — the
 * page went blank on the one reading that predicts a collision.
 *
 * The semantics are the rover's, not this file's invention. The LiDAR payload's
 * `ranges_m` is metres, one entry per bin, where `0.0` means NO RETURN (not a
 * surface at zero range) and anything at or fractionally under `range_max_m` is
 * the driver's saturation value. `fpms_missions.front_clearance_mm` discards
 * both, and so does this. Bin 0 points out the nose, so a cone CENTRED on the
 * nose is symmetric about bin 0 and the scanner's angle sign never enters —
 * which is why this cannot be mirrored the way a one-sided sector would be.
 *
 * Returns null when nothing in the cone gave a usable return. Null renders as
 * "--", never as 0: a zero here would read as "something is touching the bumper".
 */
function frontClearanceFromScan(ranges: unknown, rangeMaxM: unknown): number | null {
  if (!Array.isArray(ranges) || ranges.length < 4) return null;
  const n = ranges.length;
  const maxM =
    typeof rangeMaxM === "number" && Number.isFinite(rangeMaxM) && rangeMaxM > 0
      ? rangeMaxM
      : 6.0;
  const sat = maxM * RANGE_SATURATION_FRAC;
  const halfBins = (FRONT_CONE_DEG / 2) * (n / 360);

  let best = Infinity;
  for (let i = 0; i < n; i++) {
    // Degrees off the nose the short way round, in bins.
    const off = Math.min(i, n - i);
    if (off > halfBins) continue;
    const v = ranges[i];
    if (typeof v !== "number" || !Number.isFinite(v)) continue;
    if (v <= 0) continue; // no return
    if (v >= sat) continue; // saturation — nothing out there
    if (v < best) best = v;
  }
  return best === Infinity ? null : best * 1000;
}

function clearanceState(mm: number | null, stopMm: number): ClearanceState {
  if (mm === null) return "unknown";
  if (mm <= stopMm) return "blocked";
  if (mm <= stopMm * CLOSING_FACTOR) return "closing";
  return "clear";
}

const CLEAR_BOX: Record<ClearanceState, string> = {
  clear: "border-emerald-500/40 bg-emerald-500/5",
  closing: "border-amber-500/50 bg-amber-500/10",
  blocked: "border-rose-500/60 bg-rose-950/40",
  unknown: "border-slate-500/40 bg-black/30",
};
const CLEAR_INK: Record<ClearanceState, string> = {
  clear: "text-emerald-300",
  closing: "text-amber-300",
  blocked: "text-rose-300",
  unknown: "text-slate-500",
};
const CLEAR_WORD: Record<ClearanceState, string> = {
  clear: "CLEAR",
  closing: "CLOSING",
  blocked: "BLOCKED",
  unknown: "NO READING",
};

/**
 * Front clearance, at the size the number deserves.
 *
 * `source` is never hidden. The executor's own guard reading and a number this
 * dashboard derived from the raw scan are both useful and they are NOT the same
 * claim: the first is what the rover is steering by, the second is what the
 * scan says regardless of whether anything on the rover is looking at it.
 */
function ClearanceReadout({
  mm,
  source,
  stopMm,
  stopFromRover,
  running,
  scanStale,
}: {
  mm: number | null;
  source: "executor" | "scan" | null;
  stopMm: number;
  stopFromRover: boolean;
  running: boolean;
  scanStale: boolean;
}) {
  const state = clearanceState(mm, stopMm);
  const loud = state === "blocked" || (state === "unknown" && running);

  return (
    <div className={`rounded-xl border-2 p-3 ${CLEAR_BOX[state]}`}>
      <div className="flex items-center justify-between gap-2">
        <span className="lbl">Front clearance</span>
        <span className="flex items-center gap-1.5">
          {loud && (
            <span className="inline-block h-2.5 w-2.5 rounded-full bg-rose-400 pulse-dot text-rose-400" />
          )}
          <span
            className={
              state === "blocked"
                ? "chip-hot"
                : state === "closing"
                  ? "chip-warn"
                  : state === "clear"
                    ? "chip-ok"
                    : "chip"
            }
          >
            {CLEAR_WORD[state]}
          </span>
        </span>
      </div>
      {/* A MISSING NUMBER IS "--". Rendering 0 here would say the bumper is
          against something, which is the most dangerous possible mistranslation
          of "the rover did not tell us". */}
      <div className={`mt-1 font-mono text-4xl font-semibold tabular-nums ${CLEAR_INK[state]}`}>
        {mm === null ? "--" : Math.round(mm)}
        <span className="ml-1 text-lg text-slate-500">mm</span>
      </div>
      <div className="mt-1 font-mono text-[11px] text-slate-500">
        stops at {Math.round(stopMm)} mm
        {stopFromRover ? " · rover's own limit" : " · executor default, not confirmed"}
        {source === "executor"
          ? " · executor's guard"
          : source === "scan"
            ? ` · derived from this scan, ${FRONT_CONE_DEG}° cone`
            : ""}
      </div>
      {state === "blocked" && (
        <div className="mt-2 text-xs text-rose-100">
          <b>Inside the executor's stop distance.</b> If the rover is still
          driving it is either about to stop itself or it is not seeing this —
          abort.
        </div>
      )}
      {state === "unknown" && (
        <div className="mt-2 text-xs text-slate-400">
          {running
            ? "The rover is driving and NOTHING is reporting a front distance. The executor omits front_mm when the LiDAR is blind — treat the path ahead as unknown, not as open."
            : "No front distance is being reported. The mission executor publishes one while it runs; otherwise it is derived from the live scan."}
        </div>
      )}
      {scanStale && source === "scan" && (
        <div className="mt-2 text-xs text-amber-200/90">
          Derived from a scan that is no longer current — this is how far away
          things <i>were</i>.
        </div>
      )}
    </div>
  );
}

/* ------------------------------------------------------------ pose sanity --- */

/**
 * Is a reported position a POSITION, or a broken estimate?
 *
 * THE INCIDENT. A run reported (625, -2720) mm while the rover had physically
 * moved under a metre inside a 1200 mm arena. Every consumer plotted it:
 * the glyph went off-canvas and the page looked like it had a rendering fault,
 * when what had actually happened was that the odometry integration had come
 * apart. A number 2.9 m outside a 1.2 m box is not a position that happens to be
 * unusual — it is not a position.
 *
 * The bound is `ARENA_MM` read from `lib/arena`, plus one rover length of slack
 * so a machine legitimately sitting half over the line at the start box is not
 * called broken. Nothing is clamped, corrected or hidden: the number is printed
 * in full, and the map is handed the heartbeat instead so it draws the assumed
 * start and raises its own badge.
 *
 * NOTE: an identical guard exists in Drive.tsx. It is duplicated rather than
 * shared because both pages own their own file and `lib/arena` is not this
 * change's to edit; if a third consumer needs it, it belongs in `lib/arena`
 * beside `ARENA_MM`.
 */
const POSE_SLACK_MM = 240; // ROVER_LEN_MM — a rover straddling the line is fine.

function poseOutOfArena(x: number | null, y: number | null): boolean {
  if (x === null || y === null) return false;
  return (
    x < -POSE_SLACK_MM ||
    y < -POSE_SLACK_MM ||
    x > ARENA_MM + POSE_SLACK_MM ||
    y > ARENA_MM + POSE_SLACK_MM
  );
}

/** Which named region contains a point, or null. Geometry only — no guessing. */
function regionAt(x: number | null, y: number | null): string | null {
  if (x === null || y === null) return null;
  for (const z of ZONES) {
    if (x >= z.x_mm && x <= z.x_mm + z.w_mm && y >= z.y_mm && y <= z.y_mm + z.h_mm) {
      return `${z.label} · ${shortCorner(z.corner)}`;
    }
  }
  if (
    x >= START_BOX.x_mm &&
    x <= START_BOX.x_mm + START_BOX.w_mm &&
    y >= START_BOX.y_mm &&
    y <= START_BOX.y_mm + START_BOX.h_mm
  ) {
    return `${START_BOX.label} · ${shortCorner(START_BOX.corner)}`;
  }
  return null;
}

/* ------------------------------------------------------------- rover card --- */

/** A scan older than this is history, not the world in front of the rover. */
const SCAN_STALE_MS = 4000;

function RoverLidar({
  thing,
  accent,
  caps,
}: {
  thing: string;
  accent: string;
  caps: RoverCapabilities;
}) {
  const state = useChannel<any>(`lidar:${thing}`);
  const pose = useChannel<any>(`pose:${thing}`);
  const drive = useChannel<any>(`drive:${thing}`);
  const planCh = useChannel<any>(`mission_plan:${thing}`);
  const feed = useMission(thing);
  // The rover's own replies. Without this a PLAN the executor REFUSED would
  // look identical to one that simply had not come back yet, and the operator
  // would sit waiting for a route that is never going to arrive.
  const events = useRoverEvents(thing);
  const [busy, setBusy] = useState(false);

  // Staleness has to be a function of wall time, not of arriving data: without
  // a tick, a feed that simply STOPS keeps rendering its last frame as current
  // forever — which on a map is the most convincing lie available.
  const [now, setNow] = useState<number>(() => Date.now());
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, []);

  const cmd = async (action: "connect" | "disconnect") => {
    setBusy(true);
    try { await apiPost(`/api/rover/${thing}/${action}`); }
    finally { setBusy(false); }
  };

  /**
   * The mission console for this bay — the arm gate, the four runs, the
   * backend and SET COORDINATE, all of it shared with the Mission page.
   *
   * Held here rather than inside the component because ABORT lives in the
   * mission strip above it, and ABORT is what SHUTS the arm gate. A card whose
   * stop button could not reach the gate would leave the console armed after
   * the operator had hit the panic control.
   */
  const mc = useMissionConsole({ thing, caps, plan: planCh, feed, events, drive });

  const plan = readPlan(planCh.data);
  const m = feed.mission;

  /**
   * Pose, best source first, NEVER invented, and now never plotted when it is
   * outside the arena.
   *
   * The mission executor wins because it is the process doing the driving; the
   * teleop bridge is the fallback when no mission is running. If neither has a
   * position we hand the map the rover-agent heartbeat exactly as before, so
   * `readPose` falls back to the start corner and raises its own SIMULATED
   * badge. Synthesising a coordinate here would silence that badge while the
   * position was still an assumption.
   *
   * The out-of-arena case takes the SAME path as "no position at all", because
   * operationally it is the same thing: nobody knows where this rover is. What
   * differs is that we say so loudly instead of quietly, since a broken estimate
   * is a fault and a missing one is merely a gap.
   */
  const driveData = drive.data?.data ?? null;
  const execX = m?.poseX ?? null;
  const execY = m?.poseY ?? null;
  const bridgeX = numOrNull(driveData?.x_mm);
  const bridgeY = numOrNull(driveData?.y_mm);

  const rawX = execX ?? bridgeX;
  const rawY = execY ?? bridgeY;
  const poseBroken = poseOutOfArena(rawX, rawY);

  const posed = poseBroken
    ? null
    : poseEnvelopeFromMm(execX, execY, m?.poseHeadingDeg ?? null) ??
      poseEnvelopeFromMm(bridgeX, bridgeY, numOrNull(driveData?.heading_deg));
  const poseSource = posed
    ? execX !== null
      ? "mission executor"
      : "teleop bridge"
    : null;
  const brokenSource = execX !== null ? "mission executor" : "teleop bridge";

  const region = posed ? regionAt(rawX, rawY) : null;

  // The console's SHOW button picks which of the four previews is drawn; until
  // one is picked that is simply the newest one, which is what this card drew
  // before the console existed. The chip and the footer read the same value, so
  // the line on the map and the words under it can never describe two different
  // routes.
  const shown = mc.focusPlan ?? plan;
  const route = shown?.waypoints ?? null;

  /**
   * SCAN FRESHNESS, AND WHY IT IS SHOUTED ABOUT DURING A RUN.
   *
   * The mission executor refuses to start without a fresh scan and aborts if it
   * loses one, because its obstacle guard is the only thing between the rover
   * and whatever is in front of it. WiFi on this rover has been measured
   * swinging to -78 dBm, at which point the LiDAR telemetry stops crossing
   * entirely while the small drive and mission messages still get through — so
   * "the map is frozen but everything else looks fine" is a real, observed
   * state and not a hypothetical.
   *
   * A stale map during a mission therefore gets a red banner, not a grey chip.
   */
  const scanAgeMs = state.lastAt === null ? null : Math.max(0, now - state.lastAt);
  const scanStale = scanAgeMs !== null && scanAgeMs > SCAN_STALE_MS;
  // Rule: an UNKNOWN phase counts as RUNNING. `feed.blind` is running-and-stale,
  // and it is folded in so a mission that went quiet mid-drive still counts as
  // in motion rather than dropping to idle the moment its telemetry stops.
  const missionRunning = !!feed.mission?.running || feed.blind;
  const scanBlind = missionRunning && (scanStale || state.lastAt === null);
  // Measured, not assumed: the interval between the last two frames is the rate
  // this dashboard is actually receiving, whatever FPMS_LIDAR_HZ says.
  const scanHz =
    scanAgeMs !== null && scanAgeMs > 0 && scanAgeMs < 30000 && state.messages > 1
      ? 1000 / Math.max(scanAgeMs, 1)
      : null;

  /**
   * Front clearance. The executor's own guard reading wins — that is the number
   * the rover is actually steering by — and the scan-derived one fills the gap
   * when no mission is running, so the readout is never blank while a scan is
   * arriving.
   */
  const scanData = state.data?.data ?? null;
  const execFront = m?.frontMm ?? null;
  const scanFront = frontClearanceFromScan(scanData?.ranges_m, scanData?.range_max_m);
  const frontMm = execFront ?? scanFront;
  const frontSource: "executor" | "scan" | null =
    execFront !== null ? "executor" : scanFront !== null ? "scan" : null;

  const announcedStop = numOrNull(caps.limits?.front_stop_mm);
  const stopMm = announcedStop ?? FRONT_STOP_FALLBACK_MM;

  return (
    <Card>
      <CardHeader
        title={thing.toUpperCase()}
        subtitle={`Arena map · world-fixed · 360 ranges`}
        right={
          <div className="flex flex-wrap items-center gap-2">
            <span
              className={
                scanBlind ? "chip-hot font-mono" : scanStale ? "chip-warn font-mono" : "chip font-mono"
              }
              title={
                scanAgeMs === null
                  ? "No scan has arrived on this channel"
                  : `Newest scan is ${(scanAgeMs / 1000).toFixed(1)}s old`
              }
            >
              {scanAgeMs === null
                ? "no scan"
                : scanStale
                  ? `scan ${Math.round(scanAgeMs / 1000)}s old`
                  : `scan · ${scanHz === null ? "--" : `${scanHz.toFixed(1)} Hz`}`}
            </span>
            <span
              className={route ? "chip font-mono" : "chip font-mono text-slate-500"}
              title={
                route
                  ? `Nominal planned route from the last preview of ${shown?.mission ?? "a mission"}`
                  : "No planned route — press PLAN on one of the four runs below"
              }
            >
              {route ? `route · ${route.length} pts` : "no plan"}
            </span>
            <StatusPill
              connected={state.connected}
              lastAt={state.lastAt}
              messages={state.messages}
            />
          </div>
        }
      />

      {/* Mission first, above the map. Someone glancing at this card while
          standing next to the arena needs "is it driving itself" answered
          before anything else on it.

          The strip's ABORT is the console's: it publishes `stop`, which
          fpms-missions subscribes and aborts on — it used to publish
          `mission {name:"abort"}`, a name neither publisher accepts, so the
          button answered with a nack and stopped nothing — AND it shuts the arm
          gate below. Gated on nothing: an abort has to work precisely when
          everything else looks broken. */}
      <MissionStrip thing={thing} feed={feed} onAbort={mc.abort} className="mb-4" />

      {/* The controls, between "is it driving" and "what is in front of it".
          Deliberately above the map rather than under it: the operator arms,
          plans, reads the dashed line and only then follows, and a FOLLOW
          button below the fold is one the map never got looked at for. */}
      <MissionConsole
        mc={mc}
        className="mb-4 rounded-xl border border-white/10 bg-black/20 p-4"
      />

      {/* WHY THE LINE ON THE MAP LOOKS LIKE THAT. The route was already drawn
          but never explained: which planner produced it, what it was avoiding,
          and whether the executor has since had to reroute were all published
          by the rover and read by nothing. */}
      <PlannerBand
        plan={shown}
        replan={readReplan(events.replan)}
        replanCount={events.replanCount}
        now={now}
        className="mb-4"
      />

      {/* CLEARANCE, IN THE EYE-LINE. This sits above the map on purpose: it is
          the number that predicts a collision, and it used to be a cell in a
          six-up grid of metrics under the fold. */}
      <div className="mb-4">
        <ClearanceReadout
          mm={frontMm}
          source={frontSource}
          stopMm={stopMm}
          stopFromRover={announcedStop !== null}
          running={missionRunning}
          scanStale={scanStale}
        />
      </div>

      {/* A POSITION THAT IS NOT A POSITION. Loud, and above the map, because
          the map is not going to show it — refusing to plot it silently would
          leave the operator looking at a rover apparently parked in the start
          box while the executor drove by a number 2.9 m outside the arena. */}
      {poseBroken && (
        <div className="mb-4 flex items-start gap-3 rounded-xl border-2 border-rose-500/60 bg-rose-950/50 p-3">
          <span className="mt-1 inline-block h-3 w-3 shrink-0 rounded-full bg-rose-400 pulse-dot text-rose-400" />
          <div className="text-sm text-rose-100">
            <b>
              POSE ESTIMATE BROKEN — the {brokenSource} reports (
              {rawX === null ? "--" : Math.round(rawX)},{" "}
              {rawY === null ? "--" : Math.round(rawY)}) mm.
            </b>{" "}
            That is outside the {ARENA_MM} × {ARENA_MM} mm arena, so it is not a
            position — it is a dead-reckoning integration that has come apart.
            It is <b>not plotted</b>; the map below has fallen back to the
            assumed start corner and says so. Anything computed from this number
            — the planned route, the distance remaining, the arrival test — is
            computed from the same broken value. Abort, stop the rover, and use{" "}
            <b>Set coordinate</b> in the console above before running anything
            else.
          </div>
        </div>
      )}

      {/* A frozen map under a driving rover. Loud, because the executor's
          obstacle guard reads the same feed and the operator is looking at a
          picture of where things WERE. */}
      {scanBlind && (
        <div className="mb-4 flex items-start gap-3 rounded-xl border-2 border-rose-500/50 bg-rose-950/40 p-3">
          <span className="mt-1 inline-block h-3 w-3 shrink-0 rounded-full bg-rose-400 pulse-dot text-rose-400" />
          <div className="text-sm text-rose-100">
            <b>The map below is not current, and the rover is driving.</b>{" "}
            {scanAgeMs === null
              ? "No scan has arrived at all on this channel."
              : `The newest scan is ${Math.round(scanAgeMs / 1000)}s old.`}{" "}
            Everything drawn is where obstacles <i>were</i>. On this rover the
            LiDAR telemetry is the first thing WiFi drops — below about −70 dBm
            it stops crossing while the smaller drive and mission messages still
            get through, so a live-looking page with a frozen map is exactly what
            that looks like. Abort above before trusting any clearance here.
          </div>
        </div>
      )}
      {!scanBlind && scanStale && state.lastAt !== null && (
        <div className="mb-4 rounded-lg border border-amber-500/30 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90">
          Newest scan is {Math.round(scanAgeMs! / 1000)}s old — the map is
          history, not the world now. No mission is running, so this is a
          quiet note rather than an alarm.
        </div>
      )}

      {/* The map is the view that can lie to you: it depends on the arena
          constants, the pose and the body->world transform all being right.
          Wrapped so a fault in any of that reports itself instead of taking
          the page down. */}
      <ErrorBoundary label={`${thing} arena map`}>
        <ArenaMap
          thing={thing}
          accent={accent}
          poseEnvelope={posed ?? pose.data}
          route={route}
        />
      </ErrorBoundary>

      <div className="mt-2 flex flex-wrap items-center gap-2 text-[11px] text-slate-500">
        <span className="font-mono">
          pose ·{" "}
          {poseBroken ? (
            <span className="text-rose-300">REFUSED — outside the arena</span>
          ) : poseSource ? (
            <span className="text-slate-300">{poseSource} · dead-reckoned</span>
          ) : (
            <span className="text-amber-300">assumed start corner</span>
          )}
        </span>
        {region ? (
          <span className="chip font-mono" title="Which named region the reported position falls in">
            in {region}
          </span>
        ) : null}
        {shown ? (
          <span className="font-mono" title="Nominal — the executor re-measures its bearing after every leg and inserts corrections">
            plan · {shown.mission ?? "?"} ·{" "}
            {shown.distanceMm === null ? "--" : `${Math.round(shown.distanceMm)} mm`} · ETA{" "}
            {shown.etaS === null ? "--" : `~${Math.round(shown.etaS)} s`}
          </span>
        ) : null}
        {m?.segmentKind ? <span className="font-mono">leg · {m.segmentKind}</span> : null}
      </div>

      {/* Raw polar inset. When the arena map looks wrong this answers the only
          question that matters first — is it the data or the transform? If the
          polar view is also empty or ragged, the sensor is the problem; if it
          looks clean, the fault is in the world projection. */}
      <div className="mt-4 flex items-start gap-4">
        <div className="shrink-0">
          <div className="lbl mb-1 text-[10px]">Sensor / polar</div>
          <LidarView envelope={state.data} accent={accent} size={140} />
        </div>
        <div className="grid flex-1 grid-cols-2 gap-3 text-center">
          <Metric label="Nearest, any bearing" value={fmt(scanData?.min_mm, 0, " mm")} />
          <Metric label="Points" value={fmt(scanData?.points, 0)} />
          <Metric
            label="Position"
            // Whatever the map is drawing, in arena millimetres. A rover with no
            // odometry anywhere still shows "--" rather than a coordinate:
            // calling .toFixed() on the missing field used to throw and, with no
            // error boundary, blanked the whole dashboard. A REFUSED pose shows
            // the refusal, not the number, so it can never be copied out of here
            // into Set coordinate by accident.
            value={
              poseBroken
                ? "REFUSED"
                : posed
                  ? `${Math.round(posed.data.x_m * 1000)}, ${Math.round(posed.data.y_m * 1000)} mm`
                  : "--"
            }
          />
          <Metric
            label="Battery"
            value={
              m?.battV !== null && m?.battV !== undefined
                ? `${m.battV.toFixed(1)} V`
                : fmt(numOrNull(driveData?.battery_v), 2, " V")
            }
          />
          <Metric
            label="Travelled"
            value={
              m?.travelledMm === null || m?.travelledMm === undefined
                ? "--"
                : `${Math.round(m.travelledMm)} mm`
            }
          />
          <Metric
            label="LiDAR guard"
            // The executor says whether its own obstacle guard has a fresh
            // scan. "--" when the executor is not publishing at all, which is a
            // different statement from "the guard is blind".
            value={
              m?.lidarOk === null || m?.lidarOk === undefined
                ? "--"
                : m.lidarOk
                  ? "fresh"
                  : "BLIND"
            }
          />
        </div>
      </div>

      <div className="mt-4 flex justify-end gap-2">
        <button className="btn-primary" disabled={busy} onClick={() => cmd("connect")}>
          Connect {thing}
        </button>
        <button className="btn-danger" disabled={busy} onClick={() => cmd("disconnect")}>
          Disconnect
        </button>
      </div>
    </Card>
  );
}

/**
 * An occupancy grid older than this is describing a world that has moved on.
 *
 * The grid ages independently of the LiDAR channel this page draws: the rover
 * folds scans into it on the mission node, so the map on screen can be updating
 * at 5 Hz while the thing the PLANNER is routing against has not been refreshed
 * in a minute. Those two staleness states look identical from the map alone,
 * which is half of why "the plan isn't working" has been so hard to pin down.
 */
const OCC_STALE_S = 20;

/** Compact "5s" / "2m 10s". Kept module-private, as the page's other helpers are. */
function ageText(s: number | null): string {
  if (s === null) return "--";
  if (s < 60) return `${s < 10 ? s.toFixed(1) : Math.round(s)}s`;
  const m = Math.floor(s / 60);
  return `${m}m ${Math.round(s % 60)}s`;
}

/**
 * Planner state for the route currently drawn on the map.
 *
 * Renders nothing when there is no plan — this band must not add a row of
 * dashes to a card that is simply idle.
 */
function PlannerBand({
  plan,
  replan,
  replanCount,
  now,
  className = "",
}: {
  plan: MissionPlan | null;
  replan: ReplanDetail | null;
  replanCount: number;
  now: number;
  className?: string;
}) {
  if (!plan) return null;

  const occ = plan.occupancy;
  // The grid's age is reported by the rover as of the moment it published the
  // plan, so the time since we RECEIVED that plan has to be added back on.
  // Showing the rover's figure alone would freeze at whatever it was when the
  // plan landed and read as fresh forever — the exact failure this band exists
  // to make visible.
  const sincePlanS = plan.ts === null ? null : Math.max(0, (now - plan.ts * 1000) / 1000);
  const occAgeS =
    occ === null || occ.ageS === null ? null : occ.ageS + (sincePlanS ?? 0);
  const occStale = occAgeS !== null && occAgeS > OCC_STALE_S;
  const astar = (plan.planner ?? "").toLowerCase() === "astar";

  return (
    <div className={`flex flex-wrap items-center gap-2 text-[11px] ${className}`}>
      {plan.planner && (
        <span
          className={astar ? "chip-ok font-mono" : "chip font-mono"}
          title={
            astar
              ? "A* routed this line around cells the rover has actually observed"
              : `Planner reported by the rover: ${plan.planner}`
          }
        >
          planner · {plan.planner}
        </span>
      )}

      {occ && (
        <span
          className={occStale ? "chip-warn font-mono" : "chip font-mono"}
          title={
            occStale
              ? `The occupancy grid this route was planned against last saw a scan ` +
                `${ageText(occAgeS)} ago. Obstacles in it may no longer exist, and ` +
                `new ones will not be in it.`
              : `Occupancy grid: ${occ.cells ?? "?"} occupied of ${
                  occ.cellsTracked ?? "?"
                } tracked, ${occ.scans ?? "?"} scans folded in`
          }
        >
          grid · {occ.cells ?? "?"} cells · {ageText(occAgeS)}
          {occStale ? " STALE" : ""}
        </span>
      )}

      {replanCount > 0 && (
        <span
          className="chip-warn font-mono"
          title={
            replan
              ? `Last reroute on leg ${replan.leg ?? "?"} (${replan.replanI ?? "?"}/${
                  replan.replanMax ?? "?"
                }): ${replan.reason ?? "no reason given"}${
                  replan.viaN ? ` — ${replan.viaN} detour point(s)` : ""
                }`
              : "The executor has rerouted mid-leg"
          }
        >
          replans · {replanCount}
          {replan?.replanMax ? ` / ${replan.replanMax} per leg` : ""}
        </span>
      )}

      {/* The planner's own words. Truncated on screen, full text on hover — it
          can be several clauses long when more than one thing forced the
          detour, and this band must stay on one line. */}
      {plan.plannerNote && (
        <span className="max-w-full truncate text-slate-400" title={plan.plannerNote}>
          {plan.plannerNote}
        </span>
      )}

      {replan?.reason && (
        <span className="text-amber-300/80" title="Reason for the newest reroute">
          rerouted: {replan.reason}
        </span>
      )}
    </div>
  );
}

/** Telemetry fields are optional by nature — never assume one is a number. */
function isNum(v: unknown): v is number {
  return typeof v === "number" && Number.isFinite(v);
}

function numOrNull(v: unknown): number | null {
  return isNum(v) ? v : null;
}

/** A missing number is "--". It is NEVER 0 — see the note on ClearanceReadout. */
function fmt(v: unknown, digits: number, suffix = ""): string {
  return isNum(v) ? `${v.toFixed(digits)}${suffix}` : "--";
}

function Metric({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded-lg border border-white/5 bg-black/30 px-3 py-2">
      <div className="lbl text-[10px]">{label}</div>
      <div className="mt-1 font-mono text-sm text-slate-100">{value}</div>
    </div>
  );
}

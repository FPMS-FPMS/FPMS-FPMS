import { useEffect, useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { StatusPill } from "../components/StatusPill";
import { LidarView } from "../components/LidarView";
import { ArenaMap } from "../components/ArenaMap";
import { ErrorBoundary } from "../components/ErrorBoundary";
import { MissionStrip } from "../components/MissionStrip";
import { useChannel } from "../lib/ws";
import { apiPost } from "../lib/api";
import { useThings } from "../lib/things";
import {
  poseEnvelopeFromMm,
  postMissionAbort,
  readPlan,
  useMission,
} from "../lib/mission";

/**
 * THIS IS THE TAB PEOPLE WATCH WHILE THE ROVER MOVES.
 *
 * That is the whole reason the mission belongs here and not only on Drive. An
 * operator standing next to the arena has the map open, not the joystick page —
 * so the map has to answer, on its own, whether the machine in front of them is
 * driving itself, where it thinks it is going, and whether what they are
 * looking at is current.
 *
 * Three things were added for that, each replacing an assumption the page used
 * to make silently:
 *
 *   MISSION STATE   a strip per rover. Unknown phases count as RUNNING.
 *   PLANNED ROUTE   the same preview the Drive tab shows, on the same map, so
 *                   "where is it going" does not require switching tabs.
 *   REAL POSE       the map used to draw every rover at the assumed start
 *                   corner, because `readPose` reads the rover-agent heartbeat
 *                   and that heartbeat carries no position. The mission
 *                   executor and the teleop bridge both publish a real
 *                   dead-reckoned pose in millimetres; when one exists it is
 *                   used, and when none does the old fallback still raises the
 *                   SIMULATED badge exactly as before.
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

      <div className="grid gap-6 lg:grid-cols-2">
        {bays.map((t, i) => (
          <RoverLidar key={t} thing={t} accent={ACCENTS[i % ACCENTS.length]} />
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
          The arena map is world-fixed: the 120 cm x 120 cm grid, the zones and the water
          station stay put, and only the rover moves within them. Returns are segmented
          into walls, trees and unclassified obstacles, then voted across five frames so
          the labels hold still.
        </p>
        <p className="mt-3 text-sm leading-relaxed text-slate-400">
          The rover glyph is drawn from the best pose available, and the card says
          which one: the mission executor's while a mission is running, the teleop
          bridge's otherwise. Both are <b>dead-reckoned</b> — nothing on this rover
          localises against the map — so they drift, and a{" "}
          <span className="font-mono">pose assumed</span> chip appears when the run
          started from a position that was assumed rather than set. When neither
          publishes a position at all, the rover falls back to a known start corner
          and the map raises its own SIMULATED badge. The dashed line is the last
          planned route from a <b>PLAN</b> preview on the Drive tab; it is nominal
          intent, and the executor re-measures its bearing after every leg and
          inserts corrections, so the driven path will not match it exactly.
        </p>
      </Card>
    </div>
  );
}

/** A scan older than this is history, not the world in front of the rover. */
const SCAN_STALE_MS = 4000;

function RoverLidar({ thing, accent }: { thing: string; accent: string }) {
  const state = useChannel<any>(`lidar:${thing}`);
  const pose = useChannel<any>(`pose:${thing}`);
  const drive = useChannel<any>(`drive:${thing}`);
  const planCh = useChannel<any>(`mission_plan:${thing}`);
  const feed = useMission(thing);
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
   * Abort, from the map.
   *
   * Watching a rover drive somewhere wrong and having to change tabs to stop it
   * is the exact gap this closes. It publishes `stop`, which fpms-missions
   * subscribes and aborts on — it used to publish `mission {name:"abort"}`,
   * a name neither publisher accepts, so this button answered with a nack and
   * stopped nothing. Gated on nothing: an abort has to work precisely when
   * everything else looks broken.
   */
  const abort = () => { void postMissionAbort(thing); };

  const plan = readPlan(planCh.data);
  const m = feed.mission;

  /**
   * Pose, best source first, and NEVER invented.
   *
   * The mission executor wins because it is the process doing the driving; the
   * teleop bridge is the fallback when no mission is running. If neither has a
   * position we hand the map the rover-agent heartbeat exactly as before, so
   * `readPose` falls back to the start corner and raises its own SIMULATED
   * badge. Synthesising a coordinate here would silence that badge while the
   * position was still an assumption.
   */
  const driveData = drive.data?.data ?? null;
  const posed =
    poseEnvelopeFromMm(m?.poseX ?? null, m?.poseY ?? null, m?.poseHeadingDeg ?? null) ??
    poseEnvelopeFromMm(
      numOrNull(driveData?.x_mm),
      numOrNull(driveData?.y_mm),
      numOrNull(driveData?.heading_deg),
    );
  const poseSource = posed
    ? m?.poseX !== null && m?.poseX !== undefined
      ? "mission executor"
      : "teleop bridge"
    : null;

  const route = plan?.waypoints ?? null;

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
  const missionRunning = !!feed.mission?.running || feed.blind;
  const scanBlind = missionRunning && (scanStale || state.lastAt === null);
  // Measured, not assumed: the interval between the last two frames is the rate
  // this dashboard is actually receiving, whatever FPMS_LIDAR_HZ says.
  const scanHz =
    scanAgeMs !== null && scanAgeMs > 0 && scanAgeMs < 30000 && state.messages > 1
      ? 1000 / Math.max(scanAgeMs, 1)
      : null;

  return (
    <Card>
      <CardHeader
        title={thing.toUpperCase()}
        subtitle="Arena map · world-fixed · 360 ranges"
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
                  ? `Nominal planned route from the last preview of ${plan?.mission ?? "a mission"}`
                  : "No planned route — run a PLAN on the Drive tab to see one here"
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
          before anything else on it. */}
      <MissionStrip thing={thing} feed={feed} onAbort={abort} className="mb-4" />

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
          {poseSource ? (
            <span className="text-slate-300">{poseSource}</span>
          ) : (
            <span className="text-amber-300">assumed start corner</span>
          )}
        </span>
        {plan ? (
          <span className="font-mono" title="Nominal — the executor re-measures its bearing after every leg and inserts corrections">
            plan · {plan.mission ?? "?"} ·{" "}
            {plan.distanceMm === null ? "--" : `${Math.round(plan.distanceMm)} mm`} · ETA{" "}
            {plan.etaS === null ? "--" : `~${Math.round(plan.etaS)} s`}
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
          <Metric label="Nearest" value={fmt(state.data?.data?.min_mm, 0, " mm")} />
          <Metric label="Points" value={fmt(state.data?.data?.points, 0)} />
          <Metric
            label="Position"
            // Whatever the map is drawing, in arena millimetres. A rover with no
            // odometry anywhere still shows "—" rather than a coordinate:
            // calling .toFixed() on the missing field used to throw and, with no
            // error boundary, blanked the whole dashboard.
            value={
              posed
                ? `${Math.round(posed.data.x_m * 1000)}, ${Math.round(posed.data.y_m * 1000)} mm`
                : "—"
            }
          />
          <Metric
            label="Front clear"
            // The mission executor's obstacle guard reading. Absent is "—",
            // never 0 — 0 mm would read as "something is touching the bumper".
            value={m?.frontMm === null || m?.frontMm === undefined ? "—" : `${Math.round(m.frontMm)} mm`}
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
                ? "—"
                : `${Math.round(m.travelledMm)} mm`
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

/** Telemetry fields are optional by nature — never assume one is a number. */
function isNum(v: unknown): v is number {
  return typeof v === "number" && Number.isFinite(v);
}

function numOrNull(v: unknown): number | null {
  return isNum(v) ? v : null;
}

function fmt(v: unknown, digits: number, suffix = ""): string {
  return isNum(v) ? `${v.toFixed(digits)}${suffix}` : "—";
}

function Metric({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded-lg border border-white/5 bg-black/30 px-3 py-2">
      <div className="lbl text-[10px]">{label}</div>
      <div className="mt-1 font-mono text-sm text-slate-100">{value}</div>
    </div>
  );
}

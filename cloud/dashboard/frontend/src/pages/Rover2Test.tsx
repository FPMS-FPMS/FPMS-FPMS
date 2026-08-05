import { useEffect, useRef, useState } from "react";

import { ArenaMap } from "../components/ArenaMap";
import { ErrorBoundary } from "../components/ErrorBoundary";
import { PlannerBand } from "../components/PlannerBand";
import { apiPostJson } from "../lib/api";
import { useChannel } from "../lib/ws";
import { useThings } from "../lib/things";
import {
  MISSION_ABORT_ACTION,
  num,
  poseEnvelopeFromMm,
  readEnvelope,
  readMission,
  readPlan,
  readReplan,
  useRoverEvents,
  type MissionPlan,
} from "../lib/mission";

/**
 * ROVER 2 MISSION TEST — the panel you use STANDING NEXT TO A MOVING ROBOT.
 *
 * This is deliberately NOT the Mission page. Mission is a console: arena map,
 * four corner cards, backend picker, plan expiry rules, an explanation of why
 * the ids do not match what people say out loud. All of that is correct and
 * none of it is readable at arm's length from a machine that is driving.
 *
 * This page is the opposite trade. Big targets, one job each, no charts, and
 * the numbers that decide whether it is safe to press anything rendered at a
 * size you can read while looking up. Everything it sends goes down the SAME
 * path as the rest of the dashboard — `POST /api/control/<thing>/<action>`,
 * which the MQTT bridge turns into `fpms/<thing>/commands/<action>` — so there
 * is no second transport here to drift out of step with the first.
 *
 * ---------------------------------------------------------------------------
 * EXACTLY WHAT EACH BUTTON PUTS ON THE WIRE
 * ---------------------------------------------------------------------------
 *   FULL STOP      commands/stop            {}
 *   PLAN M1/M2     commands/mission         {name, backend:"deadreckon", preview:true}
 *   RUN M1/M2      commands/mission         {name, backend:"deadreckon"}
 *   ARM / DISARM   commands/mission         {arm:true} / {arm:false}
 *   RESET POSE     commands/set_coordinate  {x_mm:972, y_mm:228}
 *   READ ENCODERS  commands/read_encoders   {}
 *
 * (The bridge adds `source` and `ts` to every payload; it has always done that
 * and the executor ignores them.)
 *
 * ---------------------------------------------------------------------------
 * THE RULES, EACH FROM A REAL INCIDENT ON THIS ROVER
 * ---------------------------------------------------------------------------
 *   FRESHNESS IS AGED FORWARD, NOT LATCHED. A frozen-but-plausible reading has
 *   twice been mistaken for a live link here. So the age of the newest packet
 *   is recomputed against a wall clock four times a second, from the moment
 *   THIS BROWSER received it — not from a timestamp inside the payload, which
 *   a stuck publisher will happily keep re-sending. A feed that stops therefore
 *   cannot read "fresh" for even a second longer than it is; the number climbs
 *   in front of the operator and the panel goes STALE on its own.
 *
 *   STALE DISABLES THE RUN BUTTONS, and says so. It never disables FULL STOP.
 *
 *   A MISSING NUMBER IS `--`, NEVER 0. The executor omits fields it does not
 *   have. "0 mm remaining" reads as ARRIVED when it means NO IDEA.
 *
 *   A RUN IS NEVER ONE CLICK. Every motion button is arm-then-fire: the first
 *   click poises it for five seconds and the second commits. A control that
 *   goes dead while poised un-poises itself, so it cannot come back live and
 *   fire on what the operator thinks is the first click.
 *
 *   ABORT IS `stop`. Not `mission {name:"abort"}` — that name is not
 *   commandable and the rover answered "unknown mission" while aborting
 *   nothing. See MISSION_ABORT_ACTION in lib/mission.
 */

/** This page is for one rover and says so in its name. */
const THING = "rover2";

/** The only backend that has ever driven this machine. */
const BACKEND = "deadreckon";

/** The start box, in arena millimetres. RESET POSE re-zeroes to exactly this. */
const START_X_MM = 972;
const START_Y_MM = 228;

/**
 * Older than this and the panel is history, not state.
 *
 * Short on purpose. The executor publishes mission telemetry continuously, so
 * three seconds of silence beside a rover that may be moving is already a
 * situation — this page would rather cry stale than let someone read a frozen
 * pose as a live one.
 */
const STALE_MS = 3000;

/**
 * The same rule, for the picture.
 *
 * The LiDAR publishes far faster than the mission executor, so a scan that is
 * three seconds old is already several missed frames. This is deliberately the
 * SAME number as `ArenaMap`'s own internal fade threshold — the map dims itself
 * at three seconds, and a banner that disagreed with the thing it sits above
 * would be worse than no banner.
 */
const SCAN_STALE_MS = 3000;

/** How long a poised motion button stays poised before it forgets. */
const CONFIRM_MS = 5000;

/** Redraw interval for the age readouts. Fast enough that they visibly tick. */
const TICK_MS = 250;

type Cmd = {
  key: string;
  label: string;
  action: string;
  params: Record<string, unknown>;
};

const PLAN_M1: Cmd = { key: "plan-m1", label: "PLAN M1", action: "mission", params: { name: "m1", backend: BACKEND, preview: true } };
const PLAN_M2: Cmd = { key: "plan-m2", label: "PLAN M2", action: "mission", params: { name: "m2", backend: BACKEND, preview: true } };
const PLAN_WATER: Cmd = { key: "plan-water", label: "PLAN DOCK / WATER", action: "mission", params: { name: "water", backend: BACKEND, preview: true } };

const RUN_M1: Cmd = { key: "run-m1", label: "RUN M1", action: "mission", params: { name: "m1", backend: BACKEND } };
const RUN_M2: Cmd = { key: "run-m2", label: "RUN M2", action: "mission", params: { name: "m2", backend: BACKEND } };
const RUN_WATER: Cmd = { key: "run-water", label: "RUN DOCK / WATER", action: "mission", params: { name: "water", backend: BACKEND } };

/* --------------------------------------------------------------- formatting */

/** No default parameter, at any call site. A blank reading stays blank. */
function mm(v: number | null | undefined): string {
  return v === null || v === undefined ? "--" : `${Math.round(v)} mm`;
}

function volts(v: number | null | undefined): string {
  return v === null || v === undefined ? "--" : `${v.toFixed(1)} V`;
}

function deg(v: number | null | undefined): string {
  return v === null || v === undefined ? "--" : `${v.toFixed(0)}°`;
}

/** Age, as the operator would say it. Only ever called with a real number. */
function age(ms: number): string {
  if (ms < 1000) return `${ms}ms`;
  const s = ms / 1000;
  if (s < 60) return `${s.toFixed(1)}s`;
  return `${Math.floor(s / 60)}m ${String(Math.round(s % 60)).padStart(2, "0")}s`;
}

/* ------------------------------------------------------------------- page */

export default function Rover2Test() {
  const things = useThings();
  const reporting = things.includes(THING);

  const feed = useChannel<any>(`mission:${THING}`);
  const planCh = useChannel<any>(`mission_plan:${THING}`);
  const events = useRoverEvents(THING);

  /**
   * The LiDAR and pose channels, subscribed HERE only for their arrival times.
   *
   * `ArenaMap` opens its own `lidar:<thing>` socket and draws from it at frame
   * rate, which is why it is not handed the scan as a prop. But the map cannot
   * be the thing that TELLS you it has stopped updating: a canvas holding its
   * last frame looks exactly like a canvas holding the world. So the scan's age
   * is measured out here, from receipt in this browser, and stated in words
   * above the picture.
   *
   * Both channels connect on their own the moment this tab mounts. There is no
   * connect button on this page and there is not going to be one.
   */
  const scan = useChannel<any>(`lidar:${THING}`);
  const poseCh = useChannel<any>(`pose:${THING}`);

  /**
   * The wall clock. EVERYTHING about freshness on this page is a function of
   * this, not of arriving data — that is the whole point. Without it the last
   * packet renders as current forever, which is the exact failure that has
   * twice sent someone towards a rover they believed was idle.
   */
  const [now, setNow] = useState<number>(() => Date.now());
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), TICK_MS);
    return () => window.clearInterval(id);
  }, []);

  const m = readMission(readEnvelope(feed.data));
  const raw = readEnvelope(feed.data) ?? {};

  /**
   * The arm latch, as the ROVER reports it — not as this page remembers it.
   *
   * `armed` here is read straight off telemetry. This page keeps no private
   * arm flag: the latch lives on the rover now, and a dashboard that drew its
   * own idea of armed beside a machine holding a different one would be the
   * worst possible version of this control. `null` means the rover has not
   * said, and that is rendered as UNKNOWN rather than as safe.
   */
  const armed: boolean | null = typeof raw.armed === "boolean" ? (raw.armed as boolean) : null;
  const armRequired: boolean | null =
    typeof raw.arm_required === "boolean" ? (raw.arm_required as boolean) : null;

  // Age of the newest packet, measured from RECEIPT IN THIS BROWSER.
  const ageMs = feed.lastAt === null ? null : Math.max(0, now - feed.lastAt);
  const never = ageMs === null;
  const stale = never || ageMs > STALE_MS;

  const plan: MissionPlan | null = readPlan(planCh.data);
  const planAgeMs = planCh.lastAt === null ? null : Math.max(0, now - planCh.lastAt);

  // Same rule as the mission feed, applied to the picture: aged forward from
  // receipt, never trusted because a frame is on screen.
  const scanAgeMs = scan.lastAt === null ? null : Math.max(0, now - scan.lastAt);
  const scanStale = scanAgeMs === null || scanAgeMs > SCAN_STALE_MS;

  /**
   * The pose the map draws at.
   *
   * The mission executor's own belief wins when it has one, because that is the
   * pose the rover is STEERING by — a map that drew a different one while the
   * vitals above showed the executor's would leave the operator with two
   * positions and no way to tell which the machine believes. When the executor
   * has no pose this falls through to the raw pose channel, and `ArenaMap`
   * raises its own ASSUMED badge from there. It must never be given a made-up
   * coordinate, or that badge stops appearing while the position is still a
   * guess.
   */
  const mapPose =
    poseEnvelopeFromMm(m?.poseX ?? null, m?.poseY ?? null, m?.poseHeadingDeg ?? null) ??
    poseCh.data;

  /** The planned line, exactly as the LiDAR tab draws it. */
  const route = plan?.waypoints ?? null;

  /* --------------------------------------------------------------- sending */

  type LogLine = { at: number; kind: "ok" | "bad"; text: string };
  const [log, setLog] = useState<LogLine[]>([]);
  const say = (kind: "ok" | "bad", text: string) =>
    setLog((prev) => [{ at: Date.now(), kind, text }, ...prev].slice(0, 10));

  const send = (label: string, action: string, params: Record<string, unknown>) => {
    say("ok", `${label} → commands/${action} ${JSON.stringify(params)}`);
    apiPostJson<unknown>(`/api/control/${THING}/${action}`, { params }).catch((e: unknown) => {
      say("bad", `${label} — NOT SENT: ${e instanceof Error ? e.message : String(e)}`);
    });
  };

  const fire = (c: Cmd) => send(c.label, c.action, c.params);

  /**
   * FULL STOP. Gated on NOTHING — not on staleness, not on the arm latch, not
   * on whether a mission is believed to be running, not on whether this page
   * has ever heard from the rover. It has to work precisely when every other
   * check has decided things are wrong.
   */
  const fullStop = () => send("FULL STOP", MISSION_ABORT_ACTION, {});

  /* ---------------------------------------------------------------- gates */

  /** Why RUN is unavailable, in the words that say what to do about it. */
  const runBlock: string | null = !reporting
    ? `${THING} is not reporting to this dashboard, so nothing would receive the command.`
    : never
      ? "No mission telemetry has EVER arrived. The executor may not be running — nothing here can confirm the rover would even hear this."
      : stale
        ? `Mission telemetry is ${age(ageMs!)} old. The rover's state is unknown; STOP and get the feed back before driving.`
        : armRequired && armed === false
          ? "The rover's arm latch is CLOSED. Press ARM first — the rover itself will refuse this command."
          : null;

  return (
    // Bottom padding clears the fixed FULL STOP bar. Without it the last row
    // of controls sits underneath the one control that must never be covered.
    <div className="space-y-5 pb-40">
      <header>
        <h1 className="text-2xl font-semibold tracking-wide text-slate-50">
          Rover 2 Mission Test
        </h1>
        <p className="mt-1 text-sm text-slate-400">
          Buttons only. Every command goes to{" "}
          <span className="font-mono text-slate-300">fpms/{THING}/commands/…</span> on{" "}
          <span className="font-mono text-slate-300">{BACKEND}</span>. PLAN moves
          nothing; RUN moves the rover and needs two clicks.
        </p>
      </header>

      {/* ----------------------------------------------- 1 · IS THIS LIVE? */}
      <Liveness
        never={never}
        stale={stale}
        ageMs={ageMs}
        linkOk={m?.linkOk ?? null}
        connected={feed.connected}
        reporting={reporting}
        messages={feed.messages}
      />

      {/* ---------------------------------------------- 2 · WHERE IT IS NOW */}
      <MapPanel
        thing={THING}
        poseEnvelope={mapPose}
        route={route}
        plan={plan}
        events={events}
        now={now}
        scanStale={scanStale}
        scanAgeMs={scanAgeMs}
        scanConnected={scan.connected}
        running={!!m?.running}
        frontMm={m?.frontMm ?? null}
      />

      {/* ------------------------------------------------- 3 · THE ARM STATE */}
      <ArmPanel
        armed={armed}
        armRequired={armRequired}
        stale={stale}
        onArm={() => send("ARM", "mission", { arm: true })}
        onDisarm={() => send("DISARM", "mission", { arm: false })}
      />

      {/* ------------------------------------------------------ 4 · VITALS */}
      <div className="grid grid-cols-2 gap-3 sm:grid-cols-3 lg:grid-cols-6">
        <Vital
          label="battery"
          value={volts(m?.battV)}
          stale={stale}
          bad={m?.battV !== null && m?.battV !== undefined && m.battV < 11.0}
        />
        <Vital
          label="pose x, y"
          value={
            m?.poseX === null || m?.poseX === undefined || m?.poseY === null || m?.poseY === undefined
              ? "--"
              : `${Math.round(m.poseX)}, ${Math.round(m.poseY)}`
          }
          note="mm · dead-reckoned"
          stale={stale}
        />
        <Vital label="heading" value={deg(m?.poseHeadingDeg)} note="deg" stale={stale} />
        <Vital
          label="front clearance"
          value={mm(m?.frontMm)}
          note={m?.lidarOk === false ? "LIDAR DOWN" : "lidar"}
          stale={stale}
          bad={m?.lidarOk === false || (m?.frontMm !== null && m?.frontMm !== undefined && m.frontMm < 400)}
        />
        <Vital
          label="phase"
          value={m?.phase ?? "--"}
          note={m?.running ? "RUNNING" : "not driving"}
          stale={stale}
          hot={!!m?.running}
        />
        <Vital
          label="planned distance"
          value={mm(plan?.distanceMm)}
          note={
            planAgeMs === null
              ? "no plan this session"
              : `${plan?.planner ?? "planner ?"} · ${age(planAgeMs)} ago`
          }
        />
      </div>

      {/* Progress through the run, when there is one. Two numbers, no chart. */}
      {m?.running || m?.travelledMm !== null || m?.remainingMm !== null ? (
        <div className="grid grid-cols-2 gap-3 sm:grid-cols-4">
          <Vital label="travelled" value={mm(m?.travelledMm)} stale={stale} />
          <Vital label="remaining" value={mm(m?.remainingMm)} stale={stale} />
          <Vital
            label="mission"
            value={m?.mission ?? "--"}
            note={m?.backend ?? undefined}
            stale={stale}
          />
          <Vital
            label="replans"
            value={events.replanCount === 0 ? "0" : String(events.replanCount)}
            note={events.replan ? "rerouted mid-leg" : "none this session"}
          />
        </div>
      ) : null}

      {/* The planner's own words for why the route is not a straight line, and
          the executor's own words for why it rerouted. Both are prose the rover
          wrote about its own decision, so neither is paraphrased here. */}
      {plan?.plannerNote ? (
        <Note tone="sky" title="Planner">{plan.plannerNote}</Note>
      ) : null}
      {events.replan ? (
        <Note tone="amber" title="Replanned">
          {(events.replan.data as any)?.reason ?? "The executor rerouted mid-leg and gave no reason."}
        </Note>
      ) : null}
      {events.nack ? (
        <Note tone="rose" title="Rover REFUSED the last command">
          {events.nack.error ?? "no reason given"}
        </Note>
      ) : null}

      {/* -------------------------------------------------------- 5 · PLAN */}
      <Section
        title="PLAN — preview only, nothing moves"
        subtitle="Asks the executor for the route it would drive. Safe to press at any time, armed or not."
      >
        <div className="grid gap-3 sm:grid-cols-3">
          <BigButton tone="plan" label={PLAN_M1.label} sub="preview · far zone" disabled={!reporting} onClick={() => fire(PLAN_M1)} />
          <BigButton tone="plan" label={PLAN_M2.label} sub="preview · straight ahead" disabled={!reporting} onClick={() => fire(PLAN_M2)} />
          <BigButton tone="plan" label={PLAN_WATER.label} sub="preview · docking approach" disabled={!reporting} onClick={() => fire(PLAN_WATER)} />
        </div>
      </Section>

      {/* --------------------------------------------------------- 6 · RUN */}
      <Section
        title="RUN — THE ROVER MOVES"
        subtitle="Two clicks. The first poises the button for five seconds; the second commits."
        danger
      >
        {runBlock ? (
          <Note tone="amber" title="RUN is blocked">{runBlock}</Note>
        ) : null}
        <div className="mt-3 grid gap-3 sm:grid-cols-3">
          <ConfirmButton label={RUN_M1.label} sub="drives to the FAR zone" disabled={!!runBlock} onFire={() => fire(RUN_M1)} />
          <ConfirmButton label={RUN_M2.label} sub="drives STRAIGHT AHEAD up the arena" disabled={!!runBlock} onFire={() => fire(RUN_M2)} />
          <ConfirmButton label={RUN_WATER.label} sub="drives and DOCKS at the water station" disabled={!!runBlock} onFire={() => fire(RUN_WATER)} />
        </div>
      </Section>

      {/* ------------------------------------------------------- 7 · UTILITY */}
      <Section
        title="Setup and checks — none of these move the rover"
        subtitle="Available whatever the arm latch says, and whatever the link is doing."
      >
        <div className="grid gap-3 sm:grid-cols-2">
          <BigButton
            tone="util"
            label="RESET POSE"
            sub={`re-zero to the start box · ${START_X_MM}, ${START_Y_MM} mm`}
            disabled={!reporting}
            onClick={() =>
              send("RESET POSE", "set_coordinate", { x_mm: START_X_MM, y_mm: START_Y_MM })
            }
          />
          <BigButton
            tone="util"
            label="READ ENCODERS"
            sub="reads odometry back · commands no motion"
            disabled={!reporting}
            onClick={() => send("READ ENCODERS", "read_encoders", {})}
          />
        </div>
        <EncoderReadout events={events} now={now} />
      </Section>

      {/* ------------------------------------------------------------- log */}
      {log.length ? (
        <Section title="What this page has sent" subtitle="Newest first. The exact payload, verbatim.">
          <div className="space-y-1">
            {log.map((e) => (
              <div
                key={`${e.at}-${e.text}`}
                className={`overflow-x-auto rounded px-2 py-1 font-mono text-[11px] ${
                  e.kind === "bad"
                    ? "border border-rose-500/40 bg-rose-500/10 text-rose-100"
                    : "border border-white/10 bg-white/[0.04] text-slate-300"
                }`}
              >
                <span className="opacity-60">{new Date(e.at).toLocaleTimeString()}</span> {e.text}
              </div>
            ))}
          </div>
        </Section>
      ) : null}

      <p className="text-[11px] leading-snug text-slate-500">
        The arm latch is enforced ON THE ROVER — this page only shows and toggles
        it. Nothing here is an interlock on the machine: a second browser tab, the
        Drive page or a <span className="font-mono">mosquitto_pub</span> can still
        command motion. FULL STOP publishes{" "}
        <span className="font-mono">commands/stop</span>, which the executor acts on
        immediately and silently — it sends no reply, so an unanswered stop is
        normal and is not evidence that it was missed.
      </p>

      {/* --------------------------------------------------- THE PANIC BAR */}
      <StopBar onStop={fullStop} />
    </div>
  );
}

/* -------------------------------------------------------------- components */

/**
 * FULL STOP, fixed to the bottom of the viewport.
 *
 * FIXED, not sticky, and not part of the scrolling column: the operator must
 * reach it in one click from anywhere on this page without first finding out
 * where they are in it. It is the largest control on screen by a wide margin,
 * it is the only red thing, and it is never disabled by any state this page can
 * be in — including "we have never heard from the rover", which is exactly when
 * someone wants to hit it hardest.
 *
 * It is deliberately ONE CLICK. Every other motion control here is two, and
 * that asymmetry is the design: stopping must always be cheaper than starting.
 */
function StopBar({ onStop }: { onStop: () => void }) {
  const [flash, setFlash] = useState(false);
  return (
    <div className="pointer-events-none fixed inset-x-0 bottom-0 z-40 px-3 pb-3">
      <div className="app-container pointer-events-auto">
        <button
          onClick={() => {
            onStop();
            setFlash(true);
            window.setTimeout(() => setFlash(false), 900);
          }}
          className={`w-full rounded-2xl border-4 py-6 text-2xl font-black tracking-[0.2em] shadow-2xl transition sm:text-3xl ${
            flash
              ? "border-white bg-white text-rose-700"
              : "border-rose-300/60 bg-rose-600 text-white hover:bg-rose-500 active:bg-rose-700"
          }`}
          title="Publishes commands/stop. Gated on nothing — it works stale, disarmed, mid-run and with no telemetry at all."
        >
          {flash ? "STOP SENT" : "FULL STOP"}
        </button>
      </div>
    </div>
  );
}

/**
 * THE ARENA, WITH THE ROVER ON IT — on the same screen as the buttons.
 *
 * The renderer is `components/ArenaMap`, unchanged and unwrapped: the same
 * component the LiDAR and Drive tabs draw. That is the whole point of putting
 * it here rather than writing a second one. It already does the things this
 * page would otherwise have had to reinvent and get subtly different:
 *
 *   THE ARENA IS FIXED AND THE ROVER MOVES ON IT. Bird's-eye, origin at the
 *   bottom-left, +x right, +y up, north-up always. The map does NOT rotate
 *   under the robot — a heading change turns the glyph, not the room, which is
 *   the only way a person standing at one end of the arena can match what they
 *   see on screen to what they see in front of them.
 *
 *   IT DRAWS WHAT THE PLANNER IS ACTUALLY USING. The observed LiDAR clusters
 *   (walls, obstacles), the pose trail, the planned waypoints handed in as
 *   `route`, the heading ray and the front clearance — so "why did it stop
 *   there" is answerable from the picture.
 *
 *   IT FADES ITSELF WHEN THE SCAN GOES QUIET, at the same three seconds this
 *   page calls stale.
 *
 *   EVERY SAMPLE IS DRAWN. Nothing here decimates the pose trail or smooths it
 *   towards a target. At a 70 mm leg the operator has to be able to see each
 *   step land, and an eased glyph sliding between two poses is a picture of a
 *   motion the rover did not make.
 *
 * It subscribes its own LiDAR channel and starts on mount, so there is no
 * connect step on this tab and no button that could be left unpressed.
 *
 * What this wrapper adds is the part a canvas cannot say about itself: whether
 * the frame on screen is the world or a photograph of it.
 */
function MapPanel({
  thing,
  poseEnvelope,
  route,
  plan,
  events,
  now,
  scanStale,
  scanAgeMs,
  scanConnected,
  running,
  frontMm,
}: {
  thing: string;
  poseEnvelope: unknown;
  route: MissionPlan["waypoints"];
  plan: MissionPlan | null;
  events: ReturnType<typeof useRoverEvents>;
  now: number;
  scanStale: boolean;
  scanAgeMs: number | null;
  scanConnected: boolean;
  running: boolean;
  frontMm: number | null;
}) {
  return (
    <section className="rounded-2xl border border-white/10 bg-black/20 p-4">
      <div className="mb-3 flex flex-wrap items-center justify-between gap-3">
        <div>
          <h2 className="text-lg font-semibold text-slate-100">
            Where it is — live arena map
          </h2>
          <p className="mt-0.5 text-xs text-slate-400">
            The room is fixed; the rover moves on it. Planned route, observed
            obstacles and the pose trail are drawn from the same feeds the
            planner uses.
          </p>
        </div>
        <div className="flex flex-wrap items-center gap-2">
          <span className={scanStale ? "chip-bad font-mono" : "chip-ok font-mono"}>
            {scanAgeMs === null
              ? "NO SCAN EVER"
              : scanStale
                ? `SCAN ${age(scanAgeMs)} OLD`
                : `scan ${age(scanAgeMs)}`}
          </span>
          <span className={scanConnected ? "chip font-mono" : "chip-bad font-mono"}>
            {scanConnected ? "auto-connected" : "socket down"}
          </span>
          <span className="chip font-mono">front {mm(frontMm)}</span>
        </div>
      </div>

      {/*
        THE BANNER IS NOT DECORATION. A canvas holding its last frame is
        indistinguishable from a canvas holding the world, and this rover has
        already been believed stationary while it was driving. When a run is in
        progress AND the scan has stopped, that is the worst case on this page
        and it gets the loudest words available.
      */}
      {scanStale && running ? (
        <div className="mb-3 rounded-lg border-2 border-rose-500/60 bg-rose-500/15 px-3 py-2 text-sm font-semibold text-rose-100">
          THE PICTURE BELOW IS FROZEN AND THE ROVER WAS DRIVING.{" "}
          {scanAgeMs === null
            ? "No scan has ever arrived."
            : `Newest scan is ${age(scanAgeMs)} old.`}{" "}
          Treat it as still moving — FULL STOP first, diagnose after.
        </div>
      ) : scanStale ? (
        <div className="mb-3 rounded-lg border border-amber-500/40 bg-amber-500/[0.07] px-3 py-2 text-xs text-amber-100/90">
          {scanAgeMs === null
            ? "No LiDAR scan has arrived on this tab yet — the map below is the empty arena, not an empty room."
            : `Newest scan is ${age(scanAgeMs)} old, so the obstacles below are history, not the world now. No mission is running, so this is a note rather than an alarm.`}
        </div>
      ) : null}

      {/* The map depends on the arena constants, the pose and the body→world
          transform all being right. Wrapped so a fault in any of that reports
          itself here instead of taking the buttons down with it — on THIS page
          losing the tab would mean losing FULL STOP. */}
      <ErrorBoundary label={`${thing} arena map`}>
        <ArenaMap thing={thing} poseEnvelope={poseEnvelope} route={route} />
      </ErrorBoundary>

      {/* The same planner band the LiDAR tab draws, from the same component —
          occupancy-grid age included, aged forward from when the plan landed. */}
      <PlannerBand
        plan={plan}
        replan={readReplan(events.replan)}
        replanCount={events.replanCount}
        now={now}
        className="mt-3"
      />
    </section>
  );
}

/**
 * IS THIS FEED ALIVE — the honest version.
 *
 * Three separate facts, never merged into one green dot, because they fail
 * independently and the remedies differ:
 *
 *   the WebSocket to this backend  (connected)
 *   the AGE of the newest mission packet  (the one that has lied twice)
 *   the rover's OWN claim about its micro-ROS link  (link_ok)
 *
 * `link_ok: true` inside a packet that arrived four minutes ago is not a live
 * link, it is a four-minute-old opinion, and it is drawn as such.
 */
function Liveness({
  never,
  stale,
  ageMs,
  linkOk,
  connected,
  reporting,
  messages,
}: {
  never: boolean;
  stale: boolean;
  ageMs: number | null;
  linkOk: boolean | null;
  connected: boolean;
  reporting: boolean;
  messages: number;
}) {
  const tone = never
    ? "border-rose-500/60 bg-rose-500/10"
    : stale
      ? "border-rose-500/60 bg-rose-500/10"
      : "border-emerald-500/50 bg-emerald-500/[0.08]";

  return (
    <div className={`rounded-2xl border-2 p-4 ${tone}`}>
      <div className="flex flex-wrap items-center justify-between gap-4">
        <div className="flex items-center gap-3">
          <span
            className={`inline-block h-4 w-4 shrink-0 rounded-full ${
              never || stale ? "bg-rose-400" : "bg-emerald-300 pulse-dot text-emerald-300"
            }`}
          />
          <div>
            <div
              className={`text-2xl font-black tracking-wide ${
                never || stale ? "text-rose-100" : "text-emerald-100"
              }`}
            >
              {never ? "NO TELEMETRY — EVER" : stale ? "STALE — DO NOT TRUST THESE NUMBERS" : "LIVE"}
            </div>
            <div className="mt-0.5 font-mono text-sm text-slate-300">
              last packet{" "}
              <span className={never || stale ? "text-rose-200" : "text-emerald-200"}>
                {never ? "never" : age(ageMs!)} ago
              </span>{" "}
              · {messages} received · stale above {STALE_MS / 1000}s
            </div>
          </div>
        </div>

        <div className="flex flex-wrap items-center gap-2">
          <Flag ok={reporting} label={reporting ? `${THING} reporting` : `${THING} NOT in the roster`} />
          <Flag ok={connected} label={connected ? "backend socket up" : "backend socket DOWN"} />
          <Flag
            ok={linkOk === true && !stale}
            unknown={linkOk === null}
            label={
              linkOk === null
                ? "link_ok not reported"
                : linkOk
                  ? stale
                    ? "link_ok true — but STALE"
                    : "link_ok true"
                  : "link_ok FALSE"
            }
          />
        </div>
      </div>

      {never || stale ? (
        <p className="mt-3 text-sm font-semibold text-rose-100">
          Every number below is a snapshot from {never ? "no packet at all" : age(ageMs!) + " ago"}.
          The rover may have moved since. RUN is disabled; FULL STOP is not.
        </p>
      ) : null}
    </div>
  );
}

/**
 * THE ARM LATCH, at the size of the thing it gates.
 *
 * ARMED is deliberately loud and slightly unpleasant to look at. It is the only
 * gate left between a click and a moving machine, and the state that costs
 * something when misread is "I thought it was disarmed" — so armed shouts and
 * disarmed is quiet.
 *
 * UNKNOWN is its own state and is NOT drawn as disarmed. The rover not having
 * told us the latch position is not the same as the latch being shut, and
 * treating the two alike is how a page reassures an operator about a machine it
 * has heard nothing from.
 */
function ArmPanel({
  armed,
  armRequired,
  stale,
  onArm,
  onDisarm,
}: {
  armed: boolean | null;
  armRequired: boolean | null;
  stale: boolean;
  onArm: () => void;
  onDisarm: () => void;
}) {
  const unknown = armed === null;
  return (
    <div
      className={`rounded-2xl border-4 p-5 ${
        armed
          ? "border-amber-400 bg-amber-500/20"
          : unknown
            ? "border-slate-500/50 bg-white/[0.03]"
            : "border-emerald-600/40 bg-emerald-500/[0.06]"
      }`}
    >
      <div className="flex flex-wrap items-center justify-between gap-4">
        <div className="min-w-0">
          <div
            className={`text-3xl font-black tracking-[0.15em] ${
              armed ? "text-amber-100" : unknown ? "text-slate-300" : "text-emerald-100"
            }`}
          >
            {armed ? "⚠ ARMED" : unknown ? "ARM STATE UNKNOWN" : "DISARMED"}
          </div>
          <p className="mt-1 max-w-2xl text-sm text-slate-300">
            {armed ? (
              <>
                <b>The rover will act on a mission command.</b> Clear the arena.
                {stale ? " This reading is STALE — the latch may have changed since." : ""}
              </>
            ) : unknown ? (
              <>
                The rover has not reported <span className="font-mono">armed</span> in
                the telemetry this page has seen. Do not read this as safe.
              </>
            ) : (
              <>
                The rover's own latch is shut and it will refuse a mission.
                {armRequired === false
                  ? " NOTE: it also reports arm_required=false, so the latch may not be enforced."
                  : ""}
              </>
            )}
          </p>
        </div>
        <div className="flex shrink-0 gap-3">
          <button
            onClick={onArm}
            className="rounded-xl border-2 border-amber-400/60 bg-amber-500/15 px-6 py-4 text-lg font-bold tracking-widest text-amber-100 transition hover:bg-amber-500/25"
            title='Publishes commands/mission {"arm": true}. Enforced by the rover, not by this page.'
          >
            ARM
          </button>
          <button
            onClick={onDisarm}
            className="rounded-xl border-2 border-emerald-500/50 bg-emerald-500/10 px-6 py-4 text-lg font-bold tracking-widest text-emerald-100 transition hover:bg-emerald-500/20"
            title='Publishes commands/mission {"arm": false}. Does NOT stop a run already in progress — use FULL STOP for that.'
          >
            DISARM
          </button>
        </div>
      </div>
    </div>
  );
}

/** One big readout. `value` arrives pre-formatted, so a blank stays blank. */
function Vital({
  label,
  value,
  note,
  stale = false,
  bad = false,
  hot = false,
}: {
  label: string;
  value: string;
  note?: string;
  stale?: boolean;
  bad?: boolean;
  hot?: boolean;
}) {
  return (
    <div
      className={`rounded-xl border p-3 ${
        bad ? "border-rose-500/50 bg-rose-500/[0.08]" : hot ? "border-ember-500/50 bg-ember-500/[0.08]" : "border-white/10 bg-white/[0.03]"
      }`}
    >
      <div className="lbl text-[10px]">{label}</div>
      <div
        className={`mt-0.5 font-mono text-2xl leading-tight ${
          bad ? "text-rose-200" : hot ? "text-ember-100" : "text-slate-100"
        } ${value === "--" ? "opacity-40" : ""} ${stale ? "opacity-50 line-through decoration-rose-400/60 decoration-2" : ""}`}
        title={stale ? "This value is from a stale packet." : undefined}
      >
        {value}
      </div>
      {note ? <div className="mt-0.5 text-[10px] text-slate-500">{note}</div> : null}
    </div>
  );
}

function Flag({ ok, label, unknown = false }: { ok: boolean; label: string; unknown?: boolean }) {
  return (
    <span className={unknown ? "chip-warn" : ok ? "chip-ok" : "chip-bad"}>{label}</span>
  );
}

function Section({
  title,
  subtitle,
  danger = false,
  children,
}: {
  title: string;
  subtitle?: string;
  danger?: boolean;
  children: React.ReactNode;
}) {
  return (
    <section
      className={`rounded-2xl border p-4 ${
        danger ? "border-rose-500/30 bg-rose-500/[0.04]" : "border-white/10 bg-black/20"
      }`}
    >
      <h2 className={`text-lg font-semibold ${danger ? "text-rose-100" : "text-slate-100"}`}>
        {title}
      </h2>
      {subtitle ? <p className="mb-3 mt-0.5 text-xs text-slate-400">{subtitle}</p> : null}
      {children}
    </section>
  );
}

function Note({
  tone,
  title,
  children,
}: {
  tone: "sky" | "amber" | "rose";
  title: string;
  children: React.ReactNode;
}) {
  const cls =
    tone === "rose"
      ? "border-rose-500/40 bg-rose-500/10 text-rose-100"
      : tone === "amber"
        ? "border-amber-500/40 bg-amber-500/[0.07] text-amber-100"
        : "border-sky-500/40 bg-sky-500/[0.07] text-sky-100";
  return (
    <div className={`rounded-lg border px-3 py-2 text-sm ${cls}`}>
      <b>{title}:</b> {children}
    </div>
  );
}

/** A one-click control that commands no motion. */
function BigButton({
  tone,
  label,
  sub,
  disabled,
  onClick,
}: {
  tone: "plan" | "util";
  label: string;
  sub: string;
  disabled?: boolean;
  onClick: () => void;
}) {
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      className={`rounded-xl border-2 px-4 py-5 text-left transition disabled:cursor-not-allowed disabled:opacity-40 ${
        tone === "plan"
          ? "border-sky-500/40 bg-sky-500/10 hover:bg-sky-500/20"
          : "border-white/15 bg-white/[0.05] hover:bg-white/[0.09]"
      }`}
      title={disabled ? `${THING} is not reporting.` : sub}
    >
      <div className="text-xl font-bold tracking-wide text-slate-50">{label}</div>
      <div className="mt-0.5 text-xs text-slate-400">{sub}</div>
    </button>
  );
}

/**
 * ARM-THEN-FIRE, for the controls that move the machine.
 *
 * The first click poises; the second commits; five seconds of not deciding
 * un-poises it. A stray click therefore costs an amber button and nothing else.
 *
 * The disabled-while-poised reset is load-bearing rather than tidy: without it
 * a button that went dead mid-decision — telemetry going stale, the latch
 * closing — would come back still poised and fire the rover on what the
 * operator experienced as a first click.
 */
function ConfirmButton({
  label,
  sub,
  disabled,
  onFire,
}: {
  label: string;
  sub: string;
  disabled: boolean;
  onFire: () => void;
}) {
  const [poisedUntil, setPoisedUntil] = useState<number | null>(null);
  const [now, setNow] = useState(() => Date.now());
  const timer = useRef<number | null>(null);

  useEffect(() => {
    if (poisedUntil === null) return;
    const id = window.setInterval(() => setNow(Date.now()), 100);
    return () => window.clearInterval(id);
  }, [poisedUntil]);

  useEffect(() => () => { if (timer.current) window.clearTimeout(timer.current); }, []);

  useEffect(() => {
    if (disabled) setPoisedUntil(null);
  }, [disabled]);

  const poised = poisedUntil !== null && poisedUntil > now;
  useEffect(() => {
    if (poisedUntil !== null && poisedUntil <= now) setPoisedUntil(null);
  }, [poisedUntil, now]);

  const click = () => {
    if (poised) {
      setPoisedUntil(null);
      onFire();
      return;
    }
    setNow(Date.now());
    setPoisedUntil(Date.now() + CONFIRM_MS);
  };

  const left = poised ? Math.ceil((poisedUntil! - now) / 1000) : 0;

  return (
    <button
      onClick={click}
      disabled={disabled}
      className={`rounded-xl border-4 px-4 py-6 text-left transition disabled:cursor-not-allowed disabled:opacity-35 ${
        poised
          ? "border-amber-300 bg-amber-500/30 text-amber-50"
          : "border-ember-500/50 bg-ember-500/10 text-slate-50 hover:bg-ember-500/20"
      }`}
      title={disabled ? "Blocked — see the reason above." : "Two clicks: the first poises this button, the second drives the rover."}
    >
      <div className="text-xl font-black tracking-wide">
        {poised ? `CLICK AGAIN TO DRIVE · ${left}s` : label}
      </div>
      <div className="mt-0.5 text-xs opacity-80">
        {poised ? "or wait — this cancels itself" : sub}
      </div>
    </button>
  );
}

/**
 * The answer to READ ENCODERS.
 *
 * Only the handful of numbers that matter beside the arena. The full reading —
 * twist, gyro, integrated yaw, rate — is on the Mission tab, which is where
 * someone sitting down to diagnose odometry should be.
 */
function EncoderReadout({ events, now }: { events: ReturnType<typeof useRoverEvents>; now: number }) {
  const r = events.encoders;
  if (!r) return null;
  const reading = r.reading;
  const ageS = Math.max(0, (now - r.at) / 1000);
  return (
    <div className="mt-3 grid grid-cols-2 gap-3 sm:grid-cols-4">
      <Vital
        label="odom x"
        value={reading?.xM === null || reading?.xM === undefined ? "--" : `${reading.xM.toFixed(3)} m`}
      />
      <Vital
        label="odom y"
        value={reading?.yM === null || reading?.yM === undefined ? "--" : `${reading.yM.toFixed(3)} m`}
      />
      <Vital label="odom heading" value={deg(reading?.headingDeg)} />
      <Vital
        label="read"
        value={`${ageS.toFixed(0)}s ago`}
        note={reading?.stale === true ? "ODOMETRY STALE" : reading?.sourceTopic ?? undefined}
        bad={reading?.stale === true}
      />
    </div>
  );
}

/* `num` is imported for the shared null-safe numeric parse; referenced here so
   a future edit that needs it does not re-import a second copy. */
void num;

import { useEffect, useMemo, useRef, useState } from "react";

import { Card, CardHeader } from "../components/Card";
import { ArenaMap } from "../components/ArenaMap";
import { ErrorBoundary } from "../components/ErrorBoundary";
import { StatusPill } from "../components/StatusPill";
import { apiPostJson } from "../lib/api";
import { useChannel } from "../lib/ws";
import { useThings } from "../lib/things";
import {
  ROVER_START,
  START_BOX,
  ZONES,
  ZONE_AHEAD_OF_START,
  type ArenaCorner,
  type Zone,
} from "../lib/arena";
import {
  emptyCapabilities,
  useRoverCapabilities,
  type MissionInfo,
} from "../lib/capabilities";
import {
  MISSION_ABORT_ACTION,
  MISSION_ABORT_NOTE,
  num,
  poseEnvelopeFromMm,
  postMissionAbort,
  readEnvelope,
  readPlan,
  useMission,
  type MissionPlan,
} from "../lib/mission";

/**
 * THE MISSION TAB — running a mission is the whole page, not a card on it.
 *
 * Before this existed, "send the rover to a corner" was assembled by the
 * operator out of three places: a PLAN row and a mission row buried under the
 * joystick on Drive, a progress card next to it, and the fleet bar in the
 * Layout that could abort but could not start. The one task this machine
 * exists to do had no home. This page is that home, and it is laid out as the
 * sequence the operator actually performs:
 *
 *      1  PICK      a corner, named by where it physically is
 *      2  PLAN      preview the route, drawn on the arena map, drives nothing
 *      3  RUN       the real thing, confirm-gated
 *      4  WATCH     phase, leg, segment, distance, ETA, battery, clearance
 *      -  ABORT     always on screen, always enabled, at every step
 *
 * ---------------------------------------------------------------------------
 * WHY THE BUTTONS ARE IN A 2x2 GRID AND NOT A LIST
 * ---------------------------------------------------------------------------
 * The mission ids do not line up with what anyone says out loud. Standing at
 * the start box, the zone the operator calls "Zone 1" — the one directly in
 * front of them — is `zone-b`, commanded as `m2`. `m1` is the FAR zone,
 * diagonally across the arena. A vertical list of "MISSION 1 / MISSION 2"
 * invites exactly the substitution that sends a rover to the opposite corner.
 *
 * So the four targets are laid out in the same arrangement as the arena floor,
 * seen from the same side the arena map draws it: origin bottom-left, +y up,
 * start box bottom-right. The button in the top-right of the grid is the zone
 * in the top-right of the room. Every caption states the PHYSICAL CORNER
 * first and the mission id second, and nothing here renumbers anything — the
 * ids are printed verbatim, and the "Zone 1" ambiguity is called out in text
 * rather than papered over.
 *
 * Coordinates are DERIVED from lib/arena (ZONES, START_BOX) rather than typed
 * in again, so the map and these buttons cannot drift apart. When the rover
 * announces its own targets we show its label and coordinates too, and if they
 * disagree with ours we say so instead of picking a winner.
 *
 * ---------------------------------------------------------------------------
 * THE RULES THIS PAGE INHERITS, EACH FROM A REAL INCIDENT
 * ---------------------------------------------------------------------------
 *   A MISSING NUMBER IS `--`, NEVER 0. The executor omits fields it does not
 *   have; "0 mm remaining" reads as ARRIVED when it means NO IDEA. Every
 *   readout here goes through `mm`/`secs`/`volts` below, which take
 *   `number | null` and cannot be handed a default.
 *
 *   AN UNKNOWN PHASE COUNTS AS RUNNING. `useMission` decides that (see
 *   MISSION_IDLE_PHASES) and this page never second-guesses it. Showing
 *   "idle" while the rover drives is how someone walks up to a moving machine.
 *
 *   STALE TELEMETRY DURING A RUN IS LOUD. `feed.blind` — it was driving and
 *   the feed stopped — gets a full-width rose banner with the instruction
 *   attached, not a chip.
 *
 *   POSE IS DEAD-RECKONED FROM AN ASSUMED START. It is never presented as
 *   measured. The map raises its own amber ASSUMED styling and this page does
 *   not override it; it only forwards the best pose that exists.
 *
 *   ABORT IS `stop`, NOT `mission {name:"abort"}`. See MISSION_ABORT_NOTE:
 *   "abort" is not a commandable mission and the rover answered "unknown
 *   mission" while aborting nothing.
 */

/* ------------------------------------------------------------------ targets */

/**
 * Fallback backends, used only until the executor announces its own list.
 * `deadreckon` is the one that has ever worked on this rover; nav2 needs a
 * live scan topic, a TF tree and a map, none of which are up.
 */
const FALLBACK_BACKENDS = ["deadreckon", "nav2"] as const;
const FALLBACK_DEFAULT_BACKEND = "deadreckon";

const BACKEND_WARN: Record<string, string> = {
  nav2:
    "Nav2 needs a live LaserScan in ROS, a complete TF tree and a map before it can localise. None of those are up on this rover — the board's /scan publishes zeros and the working LiDAR goes to MQTT, not ROS. A mission sent on this backend will not plan.",
};

/**
 * Zone id -> the mission name that drives there.
 *
 * THIS IS THE ONLY MAPPING IN THE FILE, and it is the one thing that genuinely
 * cannot be derived: the rover's mission ids are its own, and nothing in the
 * arena geometry knows that the top-left square is reached by sending "m1".
 * Everything else on a button — the corner, the label, the coordinates — comes
 * out of lib/arena, so a zone that moves takes its button with it.
 */
const ZONE_MISSION: Record<string, string> = {
  "zone-a": "m1",
  "zone-b": "m2",
  "water-station": "water",
};

/** The mission that returns the rover to the start box. Not a ZONES member. */
const HOME_MISSION = "home";

/** Grid slot per corner, so the buttons sit the same way up as the map. */
const SLOT: Record<ArenaCorner, number> = {
  "TOP-LEFT": 0,
  "TOP-RIGHT": 1,
  "BOTTOM-LEFT": 2,
  "BOTTOM-RIGHT": 3,
};

/**
 * The sentence that separates each corner from the one next to it.
 *
 * Keyed by mission id because that is what the operator is actually choosing
 * between, and written from the operator's own standpoint — behind the start
 * box, looking up the arena.
 */
const DETAIL: Record<string, string> = {
  m1: "The FAR zone — diagonally across the arena from the start box. This is NOT the one in front of you.",
  m2: "Straight ahead of the start box, up the arena at heading 90°.",
  water:
    "Along the near wall, to the LEFT of the start box. Same end of the arena as you.",
  home:
    "The start box itself — where the rover is placed before a run and where its assumed pose sits. Driving here is a real mission like any other.",
};

type Target = {
  /** The id sent on the wire. Printed verbatim; never renamed, never renumbered. */
  mission: string;
  /** Where it IS, in the room. From the arena geometry, and it leads every caption. */
  corner: ArenaCorner;
  /** The arena's own name for the place. */
  place: string;
  /** What the place is for, when the arena says. */
  role: string | null;
  detail: string;
  x_mm: number | null;
  y_mm: number | null;
  slot: number;
};

function centreOf(z: { x_mm: number; y_mm: number; w_mm: number; h_mm: number }) {
  return { x_mm: z.x_mm + z.w_mm / 2, y_mm: z.y_mm + z.h_mm / 2 };
}

/**
 * The places this rover can be sent, in arena order.
 *
 * Built by walking ZONES rather than by listing four squares here: a zone
 * added, moved or recoloured in lib/arena shows up on this page with the right
 * corner and the right coordinates, and one removed loses its button instead of
 * leaving a control that points at nothing.
 *
 * `home` is appended by hand because the start box is deliberately NOT a member
 * of ZONES — it is the one region whose position is an ASSUMPTION rather than a
 * destination. Its centre is ROVER_START, the same constant the map draws the
 * assumed pose at, so the button and the glyph cannot disagree.
 */
const TARGETS: Target[] = (() => {
  const out: Target[] = ZONES.filter((z: Zone) => ZONE_MISSION[z.id]).map((z: Zone) => {
    const c = centreOf(z);
    const name = ZONE_MISSION[z.id];
    return {
      mission: name,
      corner: z.corner,
      place: z.label,
      role: z.role,
      detail: DETAIL[name] ?? "This corner is not described by the dashboard.",
      x_mm: c.x_mm,
      y_mm: c.y_mm,
      slot: SLOT[z.corner] ?? 9,
    };
  });
  out.push({
    mission: HOME_MISSION,
    corner: START_BOX.corner,
    place: START_BOX.label,
    role: "START BOX",
    detail: DETAIL[HOME_MISSION],
    x_mm: ROVER_START.x_mm,
    y_mm: ROVER_START.y_mm,
    slot: SLOT[START_BOX.corner] ?? 9,
  });
  return out.sort((p, q) => p.slot - q.slot);
})();

/**
 * The zone the rover faces from the start box — the operator's "Zone 1".
 *
 * Taken from lib/arena's ZONE_AHEAD_OF_START, which works it out from the
 * coordinates (same vertical lane as the start box, further up the arena)
 * rather than being told. A hardcoded "Zone 1 is m2" here would keep asserting
 * a relationship after someone moved a zone, which is the exact class of stale
 * claim that produced the naming trap in the first place. Null is a legitimate
 * answer and the callout below says so instead of inventing one.
 */
const AHEAD = ZONE_AHEAD_OF_START;
const AHEAD_MISSION = AHEAD ? ZONE_MISSION[AHEAD.id] ?? null : null;

/* --------------------------------------------------------------- formatting */

/**
 * Every readout on this page goes through one of these three.
 *
 * They take `number | null` and have NO default parameter, which is the point:
 * there is no call site at which a missing reading can quietly become a zero.
 */
function mm(v: number | null | undefined): string {
  return v === null || v === undefined ? "--" : `${Math.round(v)} mm`;
}

function secs(v: number | null | undefined): string {
  if (v === null || v === undefined) return "--";
  const s = Math.max(0, Math.round(v));
  if (s < 60) return `${s}s`;
  return `${Math.floor(s / 60)}m ${String(s % 60).padStart(2, "0")}s`;
}

function volts(v: number | null | undefined): string {
  return v === null || v === undefined ? "--" : `${v.toFixed(1)} V`;
}

/** "3/7", or "--" when either half is absent. Never "0/0". */
function ratio(i: number | null | undefined, n: number | null | undefined): string {
  return i === null || i === undefined || n === null || n === undefined
    ? "--"
    : `${i}/${n}`;
}

/* --------------------------------------------------------------------- page */

/** Drive telemetry older than this means we no longer know the rover's state. */
const DRIVE_STALE_MS = 5000;

export default function Mission() {
  return (
    <ErrorBoundary label="Mission">
      <MissionPage />
    </ErrorBoundary>
  );
}

function MissionPage() {
  const things = useThings();
  const [picked, setPicked] = useState<string | null>(null);
  const thing = picked && things.includes(picked) ? picked : things[0] ?? null;

  // ---------------------------------------------------------------- feeds ---
  const feed = useMission(thing);
  const m = feed.mission;

  const driveCh = useChannel<any>(thing ? `drive:${thing}` : null);
  const tele = readEnvelope(driveCh.data);
  const poseCh = useChannel<any>(thing ? `pose:${thing}` : null);
  const planCh = useChannel<any>(thing ? `mission_plan:${thing}` : null);

  // useChannel has no clear(), so a dismissed plan is remembered by its own
  // timestamp rather than by a boolean — the NEXT preview then reappears by
  // itself instead of staying hidden behind a flag nobody remembers to reset.
  const [dismissedPlanTs, setDismissedPlanTs] = useState<number | null>(null);
  const planRaw = readPlan(planCh.data);
  const planTs = planRaw?.ts ?? null;
  const plan: MissionPlan | null =
    planTs !== null && planTs === dismissedPlanTs ? null : planRaw;
  const route = plan?.waypoints ?? null;

  // Wall-clock tick. Staleness has to be a function of time, not of arriving
  // data: without this a feed that simply STOPS renders its last value as
  // current forever, which is the exact failure the page is here to catch.
  const [now, setNow] = useState<number>(() => Date.now());
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, []);

  // --------------------------------------------------------- capabilities ---
  const capsByThing = useRoverCapabilities();
  const caps = thing ? capsByThing[thing] ?? emptyCapabilities() : emptyCapabilities();

  const backends: readonly string[] =
    caps.backends && caps.backends.length ? caps.backends : FALLBACK_BACKENDS;
  const [pickedBackend, setPickedBackend] = useState<string | null>(null);
  // The rover's own default outranks ours until the operator chooses one:
  // FPMS_MISSION_BACKEND is configurable on the rover, so hardcoding
  // "deadreckon" would send missions on a backend it was configured away from.
  const backend =
    pickedBackend && backends.includes(pickedBackend)
      ? pickedBackend
      : caps.defaultBackend && backends.includes(caps.defaultBackend)
        ? caps.defaultBackend
        : backends[0] ?? FALLBACK_DEFAULT_BACKEND;

  /**
   * Which of the four targets the executor says it will actually accept.
   *
   * UNKNOWN IS NOT UNSUPPORTED. `events/online` is published once and not
   * retained, so a dashboard opened after the rover booted has heard nothing,
   * and greying out every button then would remove the controls on the
   * strength of a message that was never sent. Silence therefore leaves all
   * four live and says so; a rover that HAS spoken and does not list a
   * mission gets that button marked, not hidden.
   */
  const announced: ReadonlySet<string> | null = useMemo(
    () => (caps.missions && caps.missions.length ? new Set(caps.missions) : null),
    [caps.missions],
  );

  // ------------------------------------------------------------ selection ---
  // Defaults to the zone the rover is FACING, which is the one an operator
  // standing behind the start box means when they point. Derived, not typed in
  // — and if the geometry says nothing is ahead, the first target is used and
  // nothing is claimed about it.
  const [selected, setSelected] = useState<string>(() => AHEAD_MISSION ?? TARGETS[0].mission);
  const target = TARGETS.find((t) => t.mission === selected) ?? TARGETS[0];

  // ----------------------------------------------------------- motion lock ---
  const driveAgeMs = driveCh.lastAt === null ? null : now - driveCh.lastAt;
  const driveStale = driveAgeMs !== null && driveAgeMs > DRIVE_STALE_MS;
  const rosDown = tele?.ros_ok === false;
  /**
   * RUN is blocked when we positively know the link is bad. "Nothing has ever
   * arrived" is deliberately NOT in that set — if drive telemetry is not
   * plumbed through on a deployment, locking the page would leave an operator
   * with controls that cannot work and no explanation.
   *
   * PLAN and ABORT are outside this entirely. See below.
   */
  const motionLocked = rosDown || driveStale;
  const lockReason = rosDown
    ? "The rover reports ROS is down. It cannot act on a motion command."
    : driveStale
      ? `No drive telemetry for ${Math.round((driveAgeMs ?? 0) / 1000)}s — the rover's state is unknown.`
      : null;

  // ------------------------------------------------------------- commands ---
  const [note, setNote] = useState<{ kind: "ok" | "bad"; text: string } | null>(null);
  const send = (action: string, params: Record<string, unknown>, ok: string) => {
    if (!thing) return;
    setNote({ kind: "ok", text: `${ok} — sent` });
    apiPostJson<unknown>(`/api/control/${thing}/${action}`, { params }).catch(
      (e: unknown) => {
        setNote({
          kind: "bad",
          text: `${ok} — NOT SENT: ${e instanceof Error ? e.message : String(e)}`,
        });
      },
    );
  };

  /**
   * PLAN. Asks the executor what route it WOULD drive and commands no motion.
   *
   * Deliberately not gated on `motionLocked`, and that is not an oversight:
   * a preview moves nothing, and the moment an operator most wants to see the
   * intended line is precisely when the rover is locked out and they are
   * deciding whether it is safe to release it. Withholding the map then would
   * be backwards.
   */
  const planTarget = (name: string) =>
    send("mission", { name, backend, preview: true }, `PLAN ${name}`);

  /** RUN. The real thing — confirm-gated by the button, lock-gated here. */
  const runTarget = (name: string) => {
    if (motionLocked) return;
    send("mission", { name, backend }, `RUN ${name}`);
  };

  /**
   * ABORT. Publishes `stop`.
   *
   * NOT `mission {name:"abort"}` — that name is not in fpms_missions.
   * COMMANDABLE nor in teleop's stub list, so the one control that exists for
   * a machine driving somewhere wrong answered "unknown mission" and aborted
   * nothing. `stop` is the verb the executor actually subscribes and acts on,
   * and it halts the motors in the same publish.
   *
   * Gated on nothing at all, including the absence of a mission: an abort has
   * to work exactly when every other check has decided things are wrong.
   */
  const abort = () => {
    if (!thing) return;
    setNote({ kind: "ok", text: `ABORT (${MISSION_ABORT_ACTION}) — sent` });
    postMissionAbort(thing);
  };

  const abortRef = useRef(abort);
  abortRef.current = abort;
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") abortRef.current();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  // ------------------------------------------------------------- map pose ---
  /**
   * Best pose that EXISTS, never one we made up.
   *
   * The mission executor wins over the teleop bridge when both have one: they
   * read the same odometry, but the executor is the process actually driving,
   * and two positions on screen with no way to tell which the rover steers by
   * is worse than one. When nothing has a position we pass the heartbeat
   * envelope straight through so `readPose` raises its own amber ASSUMED
   * styling — that fallback is load-bearing and must never be replaced with a
   * manufactured coordinate.
   */
  const mapPose =
    poseEnvelopeFromMm(m?.poseX ?? null, m?.poseY ?? null, m?.poseHeadingDeg ?? null) ??
    poseEnvelopeFromMm(num(tele?.x_mm), num(tele?.y_mm), num(tele?.heading_deg));
  const poseSource =
    mapPose === null
      ? null
      : m?.poseX !== null && m?.poseX !== undefined
        ? "mission executor"
        : "teleop bridge";

  const running = !!m?.running;
  const noRover = !thing;

  return (
    <div className="space-y-5">
      <div className="flex flex-wrap items-end justify-between gap-4">
        <div>
          <div className="lbl">Mission · pick, plan, run</div>
          <h1 className="h-page mt-1">Send the rover to a corner</h1>
        </div>
        <div className="flex flex-wrap items-center gap-2">
          {things.length === 0 ? (
            <span className="chip-warn">no rovers reporting</span>
          ) : (
            things.map((t) => (
              <button
                key={t}
                onClick={() => setPicked(t)}
                className={`chip ${t === thing ? "border-ember-500/40 bg-ember-500/10 text-ember-200" : ""}`}
                title={`Run missions on ${t}`}
              >
                <span className="font-mono">{t}</span>
              </button>
            ))
          )}
        </div>
      </div>

      {/* ABORT. Sticky at the same offset the Drive tab pins its stop bar, so
          the two consoles have it in the same place; never disabled, because
          greying it out removes the control at exactly the moment the previous
          press might have been the one that failed. It is above everything
          else on the page on purpose. */}
      <div className="sticky top-[86px] z-20 md:top-[58px]">
        <button
          onClick={abort}
          className="flex w-full items-center justify-center gap-3 rounded-xl border border-rose-500/50 bg-rose-600/25 px-4 py-5 text-lg font-semibold tracking-wide text-rose-50 shadow-lg shadow-black/50 backdrop-blur transition hover:bg-rose-600/40 active:scale-[0.995]"
          title={`Abort the run and halt the motors on ${thing ?? "the selected rover"}. Always enabled. Shortcut: Esc. ${MISSION_ABORT_NOTE}`}
        >
          <span className="inline-block h-3 w-3 rounded-full bg-rose-400 pulse-dot text-rose-400" />
          ABORT — STOP {thing ? thing.toUpperCase() : "THE ROVER"}
          <span className="rounded border border-rose-300/30 bg-black/30 px-1.5 py-0.5 font-mono text-[11px] font-normal text-rose-200">
            Esc
          </span>
        </button>
      </div>

      {/* BLIND: it was driving, and the telemetry stopped. The single loudest
          state on the page, and the only one that carries an instruction. */}
      {feed.blind && (
        <div className="flex items-start gap-3 rounded-xl border-2 border-rose-500/50 bg-rose-950/50 p-4">
          <span className="mt-0.5 inline-block h-3 w-3 shrink-0 rounded-full bg-rose-400 pulse-dot text-rose-400" />
          <div>
            <div className="text-base font-semibold text-rose-100">
              TELEMETRY LOST WHILE DRIVING — {Math.round((feed.ageMs ?? 0) / 1000)}s
              of silence
            </div>
            <p className="mt-1 text-sm text-rose-200/85">
              The last thing this rover reported was a mission in progress, and
              nothing has arrived since. <b>Treat it as still moving.</b> Every
              number below is history, not state. Press ABORT before approaching
              the arena — a machine you cannot see is not a machine you can
              assume has stopped.
            </p>
          </div>
        </div>
      )}

      {lockReason && (
        <div className="flex items-start gap-3 rounded-xl border-2 border-rose-500/50 bg-rose-950/40 p-4">
          <span className="mt-0.5 inline-block h-3 w-3 shrink-0 rounded-full bg-rose-400 pulse-dot text-rose-400" />
          <div>
            <div className="text-base font-semibold text-rose-100">
              {rosDown ? "ROS DOWN — the rover cannot move" : "LINK STALE — rover state unknown"}
            </div>
            <p className="mt-1 text-sm text-rose-200/85">
              {lockReason} RUN is disabled until it recovers. <b>PLAN and ABORT
              still work</b> — a preview moves nothing, and an abort has to work
              precisely when the link check says things are wrong.
            </p>
          </div>
        </div>
      )}

      <div className="grid gap-5 lg:grid-cols-2">
        {/* ------------------------------------------------- 1. pick + 2/3 */}
        <Card>
          <CardHeader
            title="1 · Pick the corner"
            subtitle="Laid out like the arena, seen from the start box"
            right={
              <span
                className={announced ? "chip-ok" : "chip-warn"}
                title={
                  announced
                    ? "This rover announced its mission list; targets it does not list are marked."
                    : "The executor has not announced its mission list — events/online is published once and is not retained. All four stay available; silence is not a refusal."
                }
              >
                {announced ? "list from rover" : "not yet announced"}
              </span>
            }
          />

          {/*
            THE THING THAT SENDS ROVERS TO THE WRONG CORNER, said out loud.
            Which zone is "in front" comes from the geometry (AHEAD), not from
            a sentence typed here, and NOTHING is renumbered to make the ids
            agree with the spoken names — the mismatch is real, it lives on the
            rover, and hiding it would only move the surprise somewhere worse.
          */}
          <div className="mb-4 rounded-lg border border-amber-500/40 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90">
            <b>The ids do not match what people say.</b>{" "}
            {AHEAD && AHEAD_MISSION ? (
              <>
                Standing at the start box, the zone <i>directly in front of you</i>{" "}
                — the one usually called “Zone 1” — is{" "}
                <span className="font-mono">{AHEAD.id}</span> ({AHEAD.label},{" "}
                {AHEAD.corner}) and it is commanded as{" "}
                <span className="font-mono">{AHEAD_MISSION}</span>, not{" "}
                <span className="font-mono">m1</span>.{" "}
                <span className="font-mono">m1</span> is the far zone, diagonally
                across.
              </>
            ) : (
              <>
                The arena geometry says no zone sits directly ahead of the start
                box, so there is no “the one in front” to lean on. Read the corner
                on each button rather than its number.
              </>
            )}{" "}
            The buttons below are arranged like the room, so pick by corner and
            let the id follow.
          </div>

          <div className="grid grid-cols-2 gap-3">
            {TARGETS.map((t) => (
              <TargetButton
                key={t.mission}
                t={t}
                selected={t.mission === selected}
                onSelect={() => setSelected(t.mission)}
                info={caps.missionInfo[t.mission]}
                unlisted={!!announced && !announced.has(t.mission)}
              />
            ))}
          </div>

          <p className="mt-2 text-[11px] text-slate-500">
            Origin is the arena's bottom-left corner, +x right, +y up, so these
            four squares sit where the map draws them. The rover starts in the
            bottom-right box facing 90° — straight up the arena, towards the two
            zones.
          </p>

          {/* ------------------------------------------------------ backend */}
          <div className="mt-5 border-t border-white/[0.06] pt-4">
            <div className="lbl mb-2">Driven by</div>
            <div className="flex flex-wrap gap-2">
              {backends.map((id) => (
                <button
                  key={id}
                  onClick={() => setPickedBackend(id)}
                  className={`chip ${backend === id ? "border-ember-500/40 bg-ember-500/10 text-ember-200" : ""}`}
                  title={BACKEND_WARN[id] ?? "Backend the mission will be planned and driven with"}
                >
                  <span className="font-mono">{id}</span>
                  {caps.defaultBackend === id && (
                    <span className="ml-1.5 text-[10px] text-slate-500">· rover default</span>
                  )}
                </button>
              ))}
            </div>
            {BACKEND_WARN[backend] ? (
              <div className="mt-2 rounded-lg border border-amber-500/40 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90">
                {BACKEND_WARN[backend]}
              </div>
            ) : null}
          </div>

          {/* --------------------------------------------------- 2. plan */}
          <div className="mt-5 border-t border-white/[0.06] pt-4">
            <div className="lbl mb-2">2 · Plan it — draws the route, drives nothing</div>
            <div className="flex flex-wrap items-center gap-2">
              <button
                className="btn"
                onClick={() => planTarget(target.mission)}
                disabled={noRover}
                title={
                  noRover
                    ? "No rover is reporting."
                    : `Ask the executor for the route to ${target.mission} (${target.corner}) on ${backend}. Commands no motion — deliberately available even while RUN is locked out.`
                }
              >
                PLAN → {target.corner}
                <span className="font-mono text-[11px] opacity-70">{target.mission}</span>
              </button>
              {route ? (
                <button
                  className="chip"
                  onClick={() => setDismissedPlanTs(planTs)}
                  title="Remove the planned route from the map"
                >
                  CLEAR ROUTE
                </button>
              ) : null}
              <span className="text-[11px] text-slate-500">
                Not gated by the link check — a preview moves nothing.
              </span>
            </div>

            {plan ? (
              <div className="mt-2 rounded-lg border border-sky-500/30 bg-sky-500/5 px-3 py-2 text-xs text-sky-100/90">
                <span className="font-mono">{plan.mission ?? "--"}</span> ·{" "}
                {mm(plan.distanceMm)} ·{" "}
                {plan.segments === null ? "--" : plan.segments} segments · ETA{" "}
                {plan.etaS === null ? "--" : `~${secs(plan.etaS)}`}
                {plan.returnsHome ? " · returns home" : ""}
                {plan.backend ? ` · via ${plan.backend}` : ""}
                <div className="mt-1 text-[11px] text-sky-200/70">
                  Nominal route. The executor re-measures its bearing after every
                  leg and inserts correction turns, so the driven path will
                  differ from the dashed line.
                  {plan.poseAssumed
                    ? " The start pose is ASSUMED — nothing localises this rover, so the whole route is only as right as that assumption."
                    : ""}
                </div>
              </div>
            ) : (
              <p className="mt-2 text-[11px] text-slate-500">
                No route has been previewed yet. Plan before you run: the map is
                the only place the rover's intention is visible before it moves.
              </p>
            )}
          </div>

          {/* ---------------------------------------------------- 3. run */}
          <div className="mt-5 border-t border-white/[0.06] pt-4">
            <div className="lbl mb-2">3 · Run it — the rover drives itself</div>
            <ConfirmButton
              className="btn-primary w-full justify-center py-4 text-base"
              label={`RUN → ${target.corner} · ${target.place}`}
              disabled={noRover || motionLocked}
              onFire={() => runTarget(target.mission)}
              hint={
                noRover
                  ? "No rover is reporting."
                  : motionLocked
                    ? (lockReason ?? "Motion is locked out.")
                    : `Send mission ${target.mission} on ${backend}. The rover drives to the ${target.corner} corner (${target.place}) on its own.`
              }
            />
            <p className="mt-2 text-xs text-slate-500">
              Confirm-gated: from this page a rover on blocks and a rover on the
              floor look identical. Sent as{" "}
              <span className="font-mono text-slate-400">
                mission {"{"}name: "{target.mission}", backend: "{backend}"{"}"}
              </span>
              . This rover has ONE speed — the firmware slams roughly 50% duty on
              any non-zero setpoint on /cmd_vel, so there is no crawl to fall
              back on and nothing on this page can ask for one. Clear the arena
              before confirming.
            </p>
            {note ? (
              <div
                className={`mt-2 rounded-lg px-3 py-2 font-mono text-[11px] ${
                  note.kind === "bad"
                    ? "border border-rose-500/40 bg-rose-500/10 text-rose-100"
                    : "border border-white/10 bg-white/[0.04] text-slate-300"
                }`}
              >
                {note.text}
              </div>
            ) : null}
          </div>
        </Card>

        {/* --------------------------------------------------- 4. the map */}
        <Card>
          <CardHeader
            title="Where it is going"
            subtitle={thing ? `${thing} · arena map` : "no rover selected"}
            right={
              <div className="flex flex-wrap items-center gap-2">
                <span
                  className={route ? "chip font-mono" : "chip font-mono text-slate-500"}
                  title={route ? "A previewed route is drawn on the map" : "Nothing has been previewed"}
                >
                  {route ? `route · ${route.length} pts` : "no route"}
                </span>
                <StatusPill
                  connected={feed.connected}
                  lastAt={feed.lastAt}
                  messages={feed.messages}
                />
              </div>
            }
          />
          {thing ? (
            <ErrorBoundary label={`${thing} arena map`}>
              <ArenaMap thing={thing} poseEnvelope={mapPose ?? poseCh.data} route={route} />
            </ErrorBoundary>
          ) : (
            <div className="rounded-xl border border-white/5 bg-black/30 p-8 text-center text-sm text-slate-400">
              No rover is reporting, so there is nothing to draw. This is not the
              same as an idle rover.
            </div>
          )}
          <div className="mt-2 flex flex-wrap items-center gap-2 text-[11px] text-slate-500">
            <span className="font-mono">
              pose ·{" "}
              {poseSource ? (
                <span className="text-slate-300">{poseSource}</span>
              ) : (
                <span className="text-amber-300">assumed start corner</span>
              )}
            </span>
            <span>
              Dead-reckoned from an assumed start, and it drifts. The map draws
              an assumed pose in amber for that reason — nothing localises this
              rover, so no number on it is a measurement of where it truly is.
            </span>
          </div>
        </Card>
      </div>

      {/* ------------------------------------------------------ 4. progress */}
      <Card glow={running || feed.blind}>
        <CardHeader
          title="4 · What it is doing now"
          subtitle="Live from the mission executor"
          right={
            <span
              className={
                feed.blind ? "chip-bad" : running ? "chip-hot" : feed.messages ? "chip" : "chip-warn"
              }
              title={
                feed.blind
                  ? "The rover was driving when telemetry stopped."
                  : running
                    ? "A phase the executor reports as in progress. Phases this dashboard does not recognise count as RUNNING."
                    : feed.messages
                      ? "The executor reports a phase that positively means not driving."
                      : "Nothing has ever arrived on this channel — the executor may not be running. That is not the same as idle."
              }
            >
              {(running || feed.blind) && (
                <span
                  className={`inline-block h-2 w-2 shrink-0 rounded-full pulse-dot ${
                    feed.blind ? "bg-rose-300 text-rose-300" : "bg-ember-300 text-ember-300"
                  }`}
                />
              )}
              {feed.blind
                ? "DRIVING — TELEMETRY LOST"
                : running
                  ? "RUNNING"
                  : feed.messages
                    ? "IDLE"
                    : "NO TELEMETRY"}
            </span>
          }
        />

        {!feed.messages ? (
          <p className="text-sm text-slate-400">
            Nothing has arrived on{" "}
            <span className="font-mono">
              {thing ? `mission:${thing}` : "the mission channel"}
            </span>
            . The executor may not be running at all. <b>This is not “idle”</b> —
            an idle rover reports that it is idle, and this one is reporting
            nothing.
          </p>
        ) : (
          <>
            <div className="grid grid-cols-2 gap-x-4 gap-y-4 sm:grid-cols-3 lg:grid-cols-4">
              <Stat
                label="mission"
                value={m?.mission ?? "--"}
                note={m?.backend ? `via ${m.backend}` : undefined}
              />
              <Stat
                label="phase"
                value={m?.phase ?? "--"}
                note={
                  running && m?.phase
                    ? "in progress"
                    : m?.phase
                      ? "not driving"
                      : undefined
                }
              />
              <Stat
                label="leg"
                value={m?.legLabel ?? m?.leg ?? "--"}
                note={`leg ${ratio(m?.legI, m?.legsN)}`}
                accent
              />
              <Stat
                label="segment"
                value={ratio(m?.segmentI, m?.segmentsN)}
                note={m?.segmentKind ?? undefined}
              />
              <Stat label="distance remaining" value={mm(m?.remainingMm)} />
              <Stat label="travelled" value={mm(m?.travelledMm)} />
              <Stat
                label="ETA"
                value={secs(m?.etaS)}
                note={m?.elapsedS !== null && m?.elapsedS !== undefined ? `elapsed ${secs(m.elapsedS)}` : undefined}
              />
              <Stat
                label="front clearance"
                value={mm(m?.frontMm)}
                note="obstacle guard"
              />
              <Stat label="battery" value={volts(m?.battV)} />
              <Stat
                label="target"
                value={
                  m?.targetX !== null && m?.targetX !== undefined &&
                  m?.targetY !== null && m?.targetY !== undefined
                    ? `${Math.round(m.targetX)}, ${Math.round(m.targetY)}`
                    : "--"
                }
                note="arena mm"
              />
              <Stat
                label="pose (assumed)"
                value={
                  m?.poseX !== null && m?.poseX !== undefined &&
                  m?.poseY !== null && m?.poseY !== undefined
                    ? `${Math.round(m.poseX)}, ${Math.round(m.poseY)}`
                    : "--"
                }
                note={
                  m?.poseHeadingDeg !== null && m?.poseHeadingDeg !== undefined
                    ? `${Math.round(m.poseHeadingDeg)}° · dead-reckoned`
                    : "dead-reckoned"
                }
              />
              <Stat
                label="telemetry age"
                value={feed.ageMs === null ? "--" : `${Math.round(feed.ageMs / 1000)}s`}
                note={m?.odomSource ? `odom ${m.odomSource}` : undefined}
                bad={feed.blind}
              />
            </div>

            <div className="mt-4 flex flex-wrap items-center gap-2">
              {/* Staleness is only shouted about while running: the executor
                  throttles idle telemetry on purpose, so flagging a quiet idle
                  feed would train the operator to ignore the chip that matters. */}
              {feed.stale && running ? (
                <span className="chip-hot">
                  stale {Math.round((feed.ageMs ?? 0) / 1000)}s — treat as moving
                </span>
              ) : null}
              {m?.linkOk === false ? <span className="chip-bad">micro-ROS link down</span> : null}
              {m?.lidarOk === false ? (
                <span className="chip-bad">LiDAR stale — the obstacle guard is blind</span>
              ) : null}
              {m?.poseAssumed ? (
                <span
                  className="chip-warn"
                  title="Nothing localises this rover; the pose is dead-reckoned from an assumed start and drifts."
                >
                  pose assumed
                </span>
              ) : null}
            </div>
          </>
        )}
      </Card>
    </div>
  );
}

/* --------------------------------------------------------------- components */

/**
 * One target, drawn in its physical position in a 2x2 grid.
 *
 * The corner leads, the place name follows, and the mission id is printed as
 * the small monospace token it is. That ordering is the whole point: an
 * operator picks a corner of a room, and the id is what the dashboard happens
 * to have to send.
 *
 * When the rover has announced its own label and coordinates for the target we
 * show them, and if they disagree with the arena constants by more than a
 * millimetre we say so rather than choosing a winner — a button that quietly
 * renders one set of coordinates while the rover drives to another is how the
 * next wrong-corner incident happens.
 */
function TargetButton({
  t,
  selected,
  onSelect,
  info,
  unlisted,
}: {
  t: Target;
  selected: boolean;
  onSelect: () => void;
  info?: MissionInfo;
  unlisted: boolean;
}) {
  const roverX = info?.x_mm ?? null;
  const roverY = info?.y_mm ?? null;
  const disagrees =
    roverX !== null && roverY !== null && t.x_mm !== null && t.y_mm !== null &&
    (Math.abs(roverX - t.x_mm) > 1 || Math.abs(roverY - t.y_mm) > 1);

  return (
    <button
      onClick={onSelect}
      className={`flex h-full flex-col items-start gap-1 rounded-xl border p-3 text-left transition ${
        selected
          ? "border-ember-500/50 bg-ember-500/[0.12] shadow-glow"
          : "border-white/10 bg-white/[0.03] hover:border-white/20 hover:bg-white/[0.06]"
      }`}
      title={`${t.corner} — ${t.place}. ${t.detail} Commanded as mission "${t.mission}".`}
      aria-pressed={selected}
    >
      <span
        className={`text-sm font-semibold tracking-wide ${
          selected ? "text-ember-100" : "text-slate-100"
        }`}
      >
        {t.corner}
        {/* Geometry-derived, so this badge follows the zones if they move. It
            is the operator's "Zone 1" and it is deliberately attached to the
            corner rather than to the id. */}
        {AHEAD_MISSION === t.mission ? (
          <span
            className="ml-2 rounded border border-sky-400/40 bg-sky-500/10 px-1 py-px text-[9px] font-medium tracking-wide text-sky-200"
            title="From the start box the rover faces this zone at heading 90°. This is the one people call Zone 1."
          >
            STRAIGHT AHEAD
          </span>
        ) : null}
      </span>
      <span className="text-xs text-slate-300">
        {t.place}
        {t.role ? <span className="ml-1 text-[10px] text-slate-500">· {t.role}</span> : null}
        {info?.label && info.label !== t.place ? (
          <span className="ml-1 text-[10px] text-slate-500">
            · rover calls it {info.label}
          </span>
        ) : null}
      </span>
      <span className="font-mono text-[11px] text-slate-400">
        mission {t.mission} ·{" "}
        {t.x_mm === null || t.y_mm === null
          ? "-- , --"
          : `${Math.round(t.x_mm)}, ${Math.round(t.y_mm)}`}
      </span>
      <span className="mt-0.5 text-[11px] leading-snug text-slate-500">{t.detail}</span>
      {disagrees ? (
        <span
          className="chip-warn mt-1"
          title="The rover announced different coordinates for this mission than the arena constants in this dashboard. Neither is overridden here."
        >
          rover says {Math.round(roverX!)}, {Math.round(roverY!)}
        </span>
      ) : null}
      {unlisted ? (
        <span
          className="chip-warn mt-1"
          title="This rover announced its mission list and this name was not on it. The button still sends — the announcement is a snapshot, not a contract — but expect a refusal."
        >
          not in the rover's list
        </span>
      ) : null}
    </button>
  );
}

/** One readout. `value` is pre-formatted, so a missing number arrives as `--`. */
function Stat({
  label,
  value,
  note,
  accent = false,
  bad = false,
}: {
  label: string;
  value: string;
  note?: string;
  accent?: boolean;
  bad?: boolean;
}) {
  return (
    <div>
      <div className="lbl text-[10px]">{label}</div>
      <div
        className={`mt-0.5 font-mono text-lg leading-tight ${
          bad ? "text-rose-200" : accent ? "text-sky-200" : "text-slate-100"
        } ${value === "--" ? "opacity-50" : ""}`}
      >
        {value}
      </div>
      {note ? <div className="text-[10px] text-slate-500">{note}</div> : null}
    </div>
  );
}

/**
 * Two-click commit. Copied in behaviour from the Drive tab's gate so the two
 * pages cannot develop different muscle memory for the same act.
 */
function ConfirmButton({
  label,
  onFire,
  className = "btn",
  disabled = false,
  hint,
}: {
  label: string;
  onFire: () => void;
  className?: string;
  disabled?: boolean;
  hint?: string;
}) {
  const [armed, setArmed] = useState(false);
  const timer = useRef<number | null>(null);

  useEffect(() => () => { if (timer.current) window.clearTimeout(timer.current); }, []);

  // A control that goes dead while armed must not stay armed — when it comes
  // back it would fire on a single click.
  useEffect(() => {
    if (disabled) setArmed(false);
  }, [disabled]);

  const click = () => {
    if (armed) {
      if (timer.current) window.clearTimeout(timer.current);
      setArmed(false);
      onFire();
      return;
    }
    setArmed(true);
    timer.current = window.setTimeout(() => setArmed(false), 4000);
  };

  return (
    <button
      onClick={click}
      disabled={disabled}
      className={`${className} ${armed ? "border-amber-500/50 bg-amber-500/15 text-amber-100" : ""}`}
      title={hint ?? "Requires a second click to confirm"}
    >
      {armed ? "CLICK AGAIN TO SEND THE ROVER" : label}
    </button>
  );
}

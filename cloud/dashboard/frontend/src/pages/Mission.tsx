import { useEffect, useMemo, useRef, useState } from "react";

import { Card, CardHeader } from "../components/Card";
import { ArenaMap } from "../components/ArenaMap";
import { ErrorBoundary } from "../components/ErrorBoundary";
import { StatusPill } from "../components/StatusPill";
import { apiPostJson } from "../lib/api";
import { useChannel } from "../lib/ws";
import { useThings } from "../lib/things";
import {
  FORWARD_HEADING_DEG,
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
  MISSION_ARM_NOTE,
  PLAN_MAX_AGE_MS,
  PLAN_WAIT_MS,
  num,
  poseEnvelopeFromMm,
  postMissionAbort,
  readEnvelope,
  readPlan,
  useMission,
  useRoverEvents,
  type EncoderReading,
  type MissionPlan,
} from "../lib/mission";

/**
 * THE MISSION CONSOLE — arm it, plan a route, follow the route, abort.
 *
 * Before this existed, "send the rover to a corner" was assembled by the
 * operator out of three places: a PLAN row and a mission row buried under the
 * joystick on Drive, a progress card next to it, and the fleet bar in the
 * Layout that could abort but could not start. The one task this machine
 * exists to do had no home. This page is that home, and it is laid out as the
 * sequence the operator actually performs:
 *
 *      0  ARM       a deliberate, separate act. Nothing here moves until it is
 *                   done, and it un-does itself on every abort
 *      1  PLAN      per mission. Asks for the route, commands no motion, draws
 *                   the line on the arena map
 *      2  FOLLOW    the same route, for real. Confirm-gated, ARMED-gated, and
 *                   IMPOSSIBLE before that mission's own plan has been shown
 *      3  WATCH     phase, leg, segment, distance, ETA, battery, clearance
 *      -  ABORT     always on screen, always enabled, Esc, at every step
 *
 * ---------------------------------------------------------------------------
 * WHY EVERY MISSION IS TWO BUTTONS AND NOT ONE
 * ---------------------------------------------------------------------------
 * A mission on this rover is a ROUTE — for `patrol`, several legs; for the rest
 * a turn, a drive and a dock — computed from wherever the executor believes the
 * rover currently is. That belief is dead-reckoned and can be wrong. The only
 * moment the intended path is inspectable is BEFORE it is driven, so PLAN is
 * not a convenience here: FOLLOW is disabled until the preview for that exact
 * mission has come back and been drawn, on the current backend, recently. The
 * operator therefore cannot run a route they have not seen.
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
 * in the top-right of the room. Every card states the PHYSICAL CORNER first,
 * the operator's own name for the test second, and the mission id verbatim —
 * nothing here renumbers anything, and the "Zone 1" ambiguity is called out in
 * text rather than papered over.
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
 *
 *   THERE IS NO SLOW SPEED. The firmware slams roughly 50% duty on any
 *   non-zero setpoint, so nothing on this page offers a gentle version of a
 *   move. Missions are made survivable by being SHORT and by ABORT being one
 *   key away, not by being slow.
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
 * Everything else on a card — the corner, the label, the coordinates — comes
 * out of lib/arena, so a zone that moves takes its card with it.
 */
const ZONE_MISSION: Record<string, string> = {
  "zone-a": "m1",
  "zone-b": "m2",
  "water-station": "water",
};

/** The mission that returns the rover to the start box. Not a ZONES member. */
const HOME_MISSION = "home";

/** The teleop verb behind TEST ENCODERS. Commands no motion. */
const ENCODER_ACTION = "read_encoders";

/**
 * What the operator calls each run, in their words.
 *
 * These are the button names asked for out loud — "test full mission 1",
 * "water station docking" — kept SEPARATE from the mission id and from the
 * corner. All three appear on every card because all three are in use around
 * the arena and translating between them in someone's head is what goes wrong.
 */
const TITLE: Record<string, string> = {
  m1: "TEST FULL MISSION 1",
  m2: "TEST FULL MISSION 2",
  water: "WATER STATION DOCKING",
  home: "RETURN HOME",
};

/** Grid slot per corner, so the cards sit the same way up as the map. */
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
    "Along the near wall, to the LEFT of the start box. Same end of the arena as you. The last segment is a DOCKING approach, not a drive-by.",
  home:
    "The start box itself — where the rover is placed before a run and where its assumed pose sits. Driving here is a real mission like any other.",
};

type Target = {
  /** The id sent on the wire. Printed verbatim; never renamed, never renumbered. */
  mission: string;
  /** The operator's name for this run. */
  title: string;
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
 * corner and the right coordinates, and one removed loses its card instead of
 * leaving a control that points at nothing.
 *
 * `home` is appended by hand because the start box is deliberately NOT a member
 * of ZONES — it is the one region whose position is an ASSUMPTION rather than a
 * destination. Its centre is ROVER_START, the same constant the map draws the
 * assumed pose at, so the card and the glyph cannot disagree.
 */
const TARGETS: Target[] = (() => {
  const out: Target[] = ZONES.filter((z: Zone) => ZONE_MISSION[z.id]).map((z: Zone) => {
    const c = centreOf(z);
    const name = ZONE_MISSION[z.id];
    return {
      mission: name,
      title: TITLE[name] ?? `MISSION ${name.toUpperCase()}`,
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
    title: TITLE[HOME_MISSION] ?? "RETURN HOME",
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
 * Every readout on this page goes through one of these.
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

function metres(v: number | null | undefined): string {
  return v === null || v === undefined ? "--" : `${v.toFixed(3)} m`;
}

function mps(v: number | null | undefined): string {
  return v === null || v === undefined ? "--" : `${v.toFixed(3)} m/s`;
}

function radps(v: number | null | undefined): string {
  return v === null || v === undefined ? "--" : `${v.toFixed(3)} rad/s`;
}

function degrees(v: number | null | undefined): string {
  return v === null || v === undefined ? "--" : `${v.toFixed(1)}°`;
}

function hertz(v: number | null | undefined): string {
  return v === null || v === undefined ? "--" : `${v.toFixed(1)} Hz`;
}

/** "3/7", or "--" when either half is absent. Never "0/0". */
function ratio(i: number | null | undefined, n: number | null | undefined): string {
  return i === null || i === undefined || n === null || n === undefined
    ? "--"
    : `${i}/${n}`;
}

/** "12s" / "3m 20s" of wall-clock age. Only ever called with a real number. */
function ageText(ms: number): string {
  return secs(ms / 1000);
}

/* --------------------------------------------------------------------- page */

/** Drive telemetry older than this means we no longer know the rover's state. */
const DRIVE_STALE_MS = 5000;

/** A previewed route, remembered per mission with the time it landed. */
type PlanRec = { plan: MissionPlan; at: number };

/**
 * Why FOLLOW is or is not available for one mission. `why` is rendered
 * verbatim under the button — a disabled control that does not say what would
 * enable it is just a dead control.
 */
type Gate = { ok: boolean; why: string };

/** What has happened to this mission's preview, as far as the console knows. */
type PlanStatus =
  | { kind: "none" }
  | { kind: "waiting"; sinceMs: number }
  | { kind: "silent"; sinceMs: number }
  | { kind: "refused"; error: string }
  | { kind: "ready"; rec: PlanRec; ageMs: number; expired: boolean };

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
  const events = useRoverEvents(thing);

  // Wall-clock tick. Staleness and plan age have to be functions of time, not
  // of arriving data: without this a feed that simply STOPS renders its last
  // value as current forever, which is the exact failure the page is here to
  // catch, and a three-minute-old route would never age out of the gate.
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
   * and greying out every card then would remove the controls on the strength
   * of a message that was never sent. Silence therefore leaves all four live
   * and says so; a rover that HAS spoken and does not list a mission gets that
   * card marked, not hidden.
   */
  const announced: ReadonlySet<string> | null = useMemo(
    () => (caps.missions && caps.missions.length ? new Set(caps.missions) : null),
    [caps.missions],
  );

  /* ------------------------------------------------------------ the log --- */
  const [log, setLog] = useState<{ at: number; kind: "ok" | "bad" | "gate"; text: string }[]>([]);
  const say = (kind: "ok" | "bad" | "gate", text: string) =>
    setLog((prev) => [{ at: Date.now(), kind, text }, ...prev].slice(0, 8));

  /* ------------------------------------------------------------- ARM/SAFE --
   *
   * THE GATE IS THIS PAGE'S, AND THE PAGE SAYS SO.
   *
   * There is no arm concept anywhere on the rover: fpms-missions accepts a
   * `mission` from anything that can reach the broker. This state variable
   * gates the buttons in this console and nothing else, which is worth having
   * — every incident on this project started with one unconsidered click —
   * but must never be dressed up as an interlock on the machine.
   *
   * DISARMED IS THE DEFAULT, and it is restored by ABORT. A gate that stayed
   * open after the operator hit the panic control would be worse than no gate,
   * because it would have taught them it was closed.
   */
  const [armed, setArmed] = useState(false);
  const [armedAt, setArmedAt] = useState<number | null>(null);

  const arm = () => {
    setArmed(true);
    setArmedAt(Date.now());
    say("gate", "ARMED — this console will now send motion commands");
  };
  const disarm = (reason: string) => {
    setArmed((was) => {
      if (was) say("gate", `DISARMED — ${reason}`);
      return false;
    });
    setArmedAt(null);
  };

  /* --------------------------------------------------------------- plans --- */
  // Previews, kept per mission with the moment each landed, plus the moment the
  // operator ASKED for each. FOLLOW needs both: a plan that arrived before the
  // request is a leftover from a previous run or another tab, and following a
  // route nobody in this console asked for is the thing this gate exists to
  // prevent.
  const [plans, setPlans] = useState<Record<string, PlanRec>>({});
  const [asked, setAsked] = useState<Record<string, number>>({});
  const [focus, setFocus] = useState<string>(AHEAD_MISSION ?? TARGETS[0].mission);

  const planSeen = useRef(0);
  useEffect(() => {
    if (planCh.messages === planSeen.current) return;
    planSeen.current = planCh.messages;
    const p = readPlan(planCh.data);
    if (!p || !p.mission) return;
    const name = p.mission;
    setPlans((prev) => ({ ...prev, [name]: { plan: p, at: Date.now() } }));
    // The map follows the newest route, so what is drawn is always the thing
    // that was most recently planned rather than whatever was clicked last.
    setFocus(name);
  }, [planCh.messages, planCh.data]);

  // Declared AFTER the capture effect on purpose: on mount both run, and this
  // one wins. mission_plan is published qos=1 and can be replayed to a fresh
  // subscriber, so without this a page opened long after someone else planned a
  // route would come up with FOLLOW already unlocked against a stale line.
  useEffect(() => {
    setPlans({});
    setAsked({});
    disarm("rover selection changed");
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [thing]);

  // A route is only valid for the backend it was computed on; switching
  // backends changes both the planner and the driving. Nothing is silently
  // carried across.
  useEffect(() => {
    setPlans({});
    setAsked({});
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [backend]);

  const focusPlan = plans[focus]?.plan ?? null;
  const route = focusPlan?.waypoints ?? null;

  // ----------------------------------------------------------- motion lock ---
  const driveAgeMs = driveCh.lastAt === null ? null : now - driveCh.lastAt;
  const driveStale = driveAgeMs !== null && driveAgeMs > DRIVE_STALE_MS;
  const rosDown = tele?.ros_ok === false;
  /**
   * FOLLOW is blocked when we positively know the link is bad. "Nothing has
   * ever arrived" is deliberately NOT in that set — if drive telemetry is not
   * plumbed through on a deployment, locking the page would leave an operator
   * with controls that cannot work and no explanation.
   *
   * PLAN, TEST ENCODERS and ABORT are outside this entirely. See below.
   */
  const motionLocked = rosDown || driveStale;
  const lockReason = rosDown
    ? "The rover reports ROS is down. It cannot act on a motion command."
    : driveStale
      ? `No drive telemetry for ${Math.round((driveAgeMs ?? 0) / 1000)}s — the rover's state is unknown.`
      : null;

  const running = !!m?.running;
  const noRover = !thing;

  // ------------------------------------------------------------- commands ---
  const send = (action: string, params: Record<string, unknown>, label: string) => {
    if (!thing) return;
    say("ok", `${label} — sent`);
    apiPostJson<unknown>(`/api/control/${thing}/${action}`, { params }).catch(
      (e: unknown) => {
        say("bad", `${label} — NOT SENT: ${e instanceof Error ? e.message : String(e)}`);
      },
    );
  };

  /**
   * PLAN. Asks the executor what route it WOULD drive and commands no motion.
   *
   * Deliberately not gated on `armed` or on `motionLocked`, and neither is an
   * oversight: a preview moves nothing, and the moment an operator most wants
   * to see the intended line is precisely when the rover is locked out or the
   * console is safe and they are deciding whether to open either. Withholding
   * the map then would be backwards.
   */
  const planTarget = (name: string) => {
    setAsked((prev) => ({ ...prev, [name]: Date.now() }));
    setFocus(name);
    send("mission", { name, backend, preview: true }, `PLAN ${name}`);
  };

  /**
   * FOLLOW. The real thing.
   *
   * Re-checks the gate at fire time rather than trusting the button's disabled
   * state: a plan can expire, a link can drop and a mission can start between
   * the first click of the confirm and the second.
   */
  const followTarget = (name: string) => {
    const g = gateFor(name);
    if (!g.ok) {
      say("bad", `FOLLOW ${name} — BLOCKED: ${g.why}`);
      return;
    }
    send("mission", { name, backend }, `FOLLOW ${name} (route as planned)`);
    // Every route on the page was computed from a pose the rover is about to
    // leave, so all of them stop being followable the moment one is run. The
    // next FOLLOW needs its own fresh PLAN, which is the whole point.
    setPlans({});
    setAsked({});
  };

  /** TEST ENCODERS. Reads odometry back; commands no motion, so no arm gate. */
  const [encAskedAt, setEncAskedAt] = useState<number | null>(null);
  const testEncoders = () => {
    setEncAskedAt(Date.now());
    send(ENCODER_ACTION, {}, "TEST ENCODERS (read_encoders)");
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
   * Gated on nothing at all, including the absence of a mission and the arm
   * state: an abort has to work exactly when every other check has decided
   * things are wrong. It also SHUTS the arm gate, so the next motion command
   * needs a fresh, deliberate arm.
   */
  const abort = () => {
    if (!thing) return;
    say("ok", `ABORT (${MISSION_ABORT_ACTION}) — sent`);
    postMissionAbort(thing);
    disarm("aborted");
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

  /* ------------------------------------------------------- the plan gate --- */

  /** What the console knows about one mission's preview, right now. */
  const statusFor = (name: string): PlanStatus => {
    const askedAt = asked[name] ?? null;
    const rec = plans[name] ?? null;
    const fresh = rec !== null && askedAt !== null && rec.at >= askedAt;
    if (fresh && rec) {
      const ageMs = Math.max(0, now - rec.at);
      return { kind: "ready", rec, ageMs, expired: ageMs > PLAN_MAX_AGE_MS };
    }
    if (askedAt === null) return { kind: "none" };
    // A refusal only counts if it landed after we asked and either names this
    // mission or names none — the executor omits the name on "unknown backend".
    const nack = events.nack;
    if (nack && nack.at >= askedAt && (nack.name === name || nack.name === null)) {
      return { kind: "refused", error: nack.error ?? "refused, with no reason given" };
    }
    const sinceMs = Math.max(0, now - askedAt);
    return sinceMs > PLAN_WAIT_MS ? { kind: "silent", sinceMs } : { kind: "waiting", sinceMs };
  };

  /**
   * Whether FOLLOW may fire for one mission, and if not, exactly why.
   *
   * The order is the order an operator should fix things in: no rover, then
   * the console's own gate, then the missing plan, then the rover's state.
   */
  const gateFor = (name: string): Gate => {
    if (!thing) {
      return { ok: false, why: "No rover is reporting, so there is nothing to send to." };
    }
    if (!armed) {
      return {
        ok: false,
        why: "DISARMED. This console will not put a motion command on the wire — arm it above first.",
      };
    }
    const st = statusFor(name);
    if (st.kind === "none") {
      return {
        ok: false,
        why: "No route has been planned here. Press PLAN, read the route on the map, then follow it.",
      };
    }
    if (st.kind === "waiting") {
      return { ok: false, why: "Waiting for the executor's route — it answers a preview in well under a second." };
    }
    if (st.kind === "silent") {
      return {
        ok: false,
        why: `No route came back in ${ageText(st.sinceMs)}. The mission executor may not be running at all; nothing will follow a route that was never planned.`,
      };
    }
    if (st.kind === "refused") {
      return { ok: false, why: `The executor REFUSED the preview: ${st.error}` };
    }
    if (st.expired) {
      return {
        ok: false,
        why: `That route is ${ageText(st.ageMs)} old. It was computed from where the rover was then, and this pose is dead-reckoned — re-plan before following it.`,
      };
    }
    const planBackend = st.rec.plan.backend;
    if (planBackend && planBackend !== backend) {
      return {
        ok: false,
        why: `The route was planned on ${planBackend}; the console is now set to ${backend}. Re-plan on the backend you intend to drive.`,
      };
    }
    if (motionLocked) return { ok: false, why: lockReason ?? "Motion is locked out." };
    if (feed.blind) {
      return {
        ok: false,
        why: "The rover was driving when its telemetry stopped. Treat it as still moving and ABORT — do not stack a second mission on top.",
      };
    }
    if (running) {
      return {
        ok: false,
        why: `A mission is already running (phase ${m?.phase ?? "unknown"}). The executor refuses a second one — ABORT first.`,
      };
    }
    return {
      ok: true,
      why: `Sends mission ${name} on ${backend}. The rover drives the route drawn on the map, on its own.`,
    };
  };

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

  /**
   * Fraction of the planned distance covered, or null.
   *
   * Both halves must be present. A bar drawn from travelled alone would sit at
   * 100 % the instant `distance_remaining_mm` went missing, which reads as
   * ARRIVED — the same lie as rendering a missing number as zero.
   */
  const progress =
    m?.travelledMm !== null && m?.travelledMm !== undefined &&
    m?.remainingMm !== null && m?.remainingMm !== undefined &&
    m.travelledMm + m.remainingMm > 0
      ? Math.min(1, Math.max(0, m.travelledMm / (m.travelledMm + m.remainingMm)))
      : null;

  return (
    <div className="space-y-5">
      <div className="flex flex-wrap items-end justify-between gap-4">
        <div>
          <div className="lbl">Mission console · arm, plan, follow</div>
          <h1 className="h-page mt-1">Run the rover</h1>
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
                title={`Run missions on ${t}. Switching rovers clears every plan and disarms the console.`}
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
          else on the page on purpose, including the arm gate. */}
      <div className="sticky top-[86px] z-20 md:top-[58px]">
        <button
          onClick={abort}
          className="flex w-full items-center justify-center gap-3 rounded-xl border border-rose-500/50 bg-rose-600/25 px-4 py-5 text-lg font-semibold tracking-wide text-rose-50 shadow-lg shadow-black/50 backdrop-blur transition hover:bg-rose-600/40 active:scale-[0.995]"
          title={`Abort the run and halt the motors on ${thing ?? "the selected rover"}. Always enabled, armed or not. Shortcut: Esc. Also disarms this console. ${MISSION_ABORT_NOTE}`}
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
              {lockReason} FOLLOW is disabled until it recovers. <b>PLAN, TEST
              ENCODERS and ABORT still work</b> — a preview and a readback move
              nothing, and an abort has to work precisely when the link check
              says things are wrong.
            </p>
          </div>
        </div>
      )}

      {/* ------------------------------------------------------- 0 · ARM */}
      <ArmBar
        armed={armed}
        armedAt={armedAt}
        now={now}
        thing={thing}
        onArm={arm}
        onDisarm={() => disarm("disarmed by the operator")}
      />

      {/* ------------------------------------------- 1/2 · the four missions */}
      <Card>
        <CardHeader
          title="The four runs — PLAN first, then FOLLOW"
          subtitle="Laid out like the arena, seen from the start box"
          right={
            <div className="flex flex-wrap items-center gap-2">
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
              <span className={armed ? "chip-hot" : "chip"} title={MISSION_ARM_NOTE}>
                {armed ? "ARMED" : "DISARMED"}
              </span>
            </div>
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
              across. So “{TITLE[AHEAD_MISSION] ?? AHEAD_MISSION}” is the run
              that goes to the zone you are looking at.
            </>
          ) : (
            <>
              The arena geometry says no zone sits directly ahead of the start
              box, so there is no “the one in front” to lean on. Read the corner
              on each card rather than its number.
            </>
          )}{" "}
          The cards below are arranged like the room, so pick by corner and let
          the id follow.
        </div>

        <div className="grid gap-3 md:grid-cols-2">
          {TARGETS.map((t) => (
            <MissionCard
              key={t.mission}
              t={t}
              info={caps.missionInfo[t.mission]}
              unlisted={!!announced && !announced.has(t.mission)}
              focused={focus === t.mission}
              backend={backend}
              status={statusFor(t.mission)}
              gate={gateFor(t.mission)}
              armed={armed}
              noRover={noRover}
              activeNow={running && m?.mission === t.mission}
              onPlan={() => planTarget(t.mission)}
              onFollow={() => followTarget(t.mission)}
              onShow={() => setFocus(t.mission)}
              onClear={() => {
                setPlans((prev) => {
                  const next = { ...prev };
                  delete next[t.mission];
                  return next;
                });
                setAsked((prev) => {
                  const next = { ...prev };
                  delete next[t.mission];
                  return next;
                });
              }}
            />
          ))}
        </div>

        <p className="mt-3 text-[11px] text-slate-500">
          Origin is the arena's bottom-left corner, +x right, +y up, so these
          four squares sit where the map draws them. The rover starts in the
          bottom-right box facing {FORWARD_HEADING_DEG}° — straight up the
          arena, towards the two zones.
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
                title={BACKEND_WARN[id] ?? "Backend the mission will be planned and driven with. Changing it clears every planned route — a route is only valid for the planner that produced it."}
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

        {/* ------------------------------------------------ the speed truth */}
        <p className="mt-4 border-t border-white/[0.06] pt-4 text-xs text-slate-500">
          <b className="text-slate-400">This rover has one speed.</b> The
          firmware slams roughly 50 % duty on any non-zero setpoint on
          /cmd_vel, so there is no crawl to fall back on and nothing on this
          page can ask for one — a mission is made survivable by being short
          and by ABORT being one key away, not by being gentle. Manual jogging
          lives on the Drive tab, which pulses and then fully stops for the same
          reason.
        </p>

        {log.length ? (
          <div className="mt-4 space-y-1 border-t border-white/[0.06] pt-3">
            <div className="lbl mb-1">Console log</div>
            {log.map((e) => (
              <div
                key={`${e.at}-${e.text}`}
                className={`rounded px-2 py-1 font-mono text-[11px] ${
                  e.kind === "bad"
                    ? "border border-rose-500/40 bg-rose-500/10 text-rose-100"
                    : e.kind === "gate"
                      ? "border border-amber-500/30 bg-amber-500/[0.06] text-amber-100/90"
                      : "border border-white/10 bg-white/[0.04] text-slate-300"
                }`}
              >
                <span className="opacity-60">
                  {new Date(e.at).toLocaleTimeString()}
                </span>{" "}
                {e.text}
              </div>
            ))}
          </div>
        ) : null}
      </Card>

      <div className="grid gap-5 lg:grid-cols-2">
        {/* --------------------------------------------------- 3. the map */}
        <Card>
          <CardHeader
            title="The route it will drive"
            subtitle={thing ? `${thing} · arena map` : "no rover selected"}
            right={
              <div className="flex flex-wrap items-center gap-2">
                <span
                  className={route ? "chip font-mono" : "chip font-mono text-slate-500"}
                  title={route ? "The previewed route, drawn as a dashed line" : "Nothing has been previewed"}
                >
                  {route ? `${focus} · ${route.length} pts` : "no route"}
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

          {focusPlan ? <PlanDetail plan={focusPlan} live={m?.legI ?? null} /> : (
            <p className="mt-3 text-xs text-slate-500">
              No route is drawn. Press PLAN on one of the four runs — the map is
              the only place the rover's intention is visible before it moves.
            </p>
          )}

          <div className="mt-3 flex flex-wrap items-center gap-2 text-[11px] text-slate-500">
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

        {/* ------------------------------------------------- 4. progress */}
        <Card glow={running || feed.blind}>
          <CardHeader
            title="What it is doing now"
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
              . The executor may not be running at all. <b>This is not “idle”</b>{" "}
              — an idle rover reports that it is idle, and this one is reporting
              nothing.
            </p>
          ) : (
            <>
              {/* Route progress. Only drawn when BOTH halves exist — see the
                  `progress` comment; a bar fed by travelled alone would read
                  ARRIVED the moment the remaining distance went missing. */}
              <div className="mb-4">
                <div className="flex items-baseline justify-between">
                  <span className="lbl text-[10px]">route progress</span>
                  <span className="font-mono text-[11px] text-slate-400">
                    {progress === null ? "--" : `${Math.round(progress * 100)}%`}
                  </span>
                </div>
                <div className="mt-1 h-2 w-full overflow-hidden rounded-full bg-white/[0.06]">
                  {progress === null ? null : (
                    <div
                      className={`h-full rounded-full ${feed.blind ? "bg-rose-400/70" : "bg-ember-400/80"}`}
                      style={{ width: `${progress * 100}%` }}
                    />
                  )}
                </div>
                {progress === null ? (
                  <div className="mt-1 text-[10px] text-slate-500">
                    The executor is not reporting both travelled and remaining
                    distance, so there is no fraction to draw. An empty bar here
                    means UNKNOWN, not zero.
                  </div>
                ) : null}
              </div>

              <div className="grid grid-cols-2 gap-x-4 gap-y-4 sm:grid-cols-3">
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
                  note={
                    m?.elapsedS !== null && m?.elapsedS !== undefined
                      ? `elapsed ${secs(m.elapsedS)}`
                      : undefined
                  }
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

              {events.done ? (
                <div className="mt-4 rounded-lg border border-white/10 bg-white/[0.04] px-3 py-2 text-[11px] text-slate-300">
                  <span className="lbl text-[10px]">last outcome</span>{" "}
                  <span className="font-mono">
                    {String(events.done.data.mission ?? events.done.name ?? "--")} ·{" "}
                    {String(events.done.data.outcome ?? events.done.data.state ?? "--")}
                  </span>
                  <div className="mt-0.5 text-[10px] text-slate-500">
                    From events/mission_done, which reports the MEASURED outcome
                    rather than the requested one.
                  </div>
                </div>
              ) : null}
            </>
          )}
        </Card>
      </div>

      {/* ------------------------------------------------- 5. encoders */}
      <EncoderCard
        thing={thing}
        askedAt={encAskedAt}
        now={now}
        reply={events.encoders}
        supported={caps.announced ? caps.actions.has(ENCODER_ACTION) : null}
        onTest={testEncoders}
      />
    </div>
  );
}

/* --------------------------------------------------------------- components */

/**
 * THE ARM GATE.
 *
 * Two states, one deliberate act between them, and a paragraph that refuses to
 * let the operator believe the rover is enforcing it. The DISARMED copy is the
 * important one: it says what is blocked (this page's motion buttons) and what
 * is NOT (the machine, the Drive tab, anything else holding the broker).
 */
function ArmBar({
  armed,
  armedAt,
  now,
  thing,
  onArm,
  onDisarm,
}: {
  armed: boolean;
  armedAt: number | null;
  now: number;
  thing: string | null;
  onArm: () => void;
  onDisarm: () => void;
}) {
  return (
    <div
      className={`rounded-xl border-2 p-4 ${
        armed
          ? "border-ember-500/50 bg-ember-500/[0.08]"
          : "border-white/10 bg-white/[0.03]"
      }`}
    >
      <div className="flex flex-wrap items-center justify-between gap-4">
        <div className="min-w-0">
          <div className="flex items-center gap-2">
            {armed ? (
              <span className="inline-block h-3 w-3 shrink-0 rounded-full bg-ember-300 pulse-dot text-ember-300" />
            ) : (
              <span className="inline-block h-3 w-3 shrink-0 rounded-full border border-slate-500" />
            )}
            <span
              className={`text-base font-semibold tracking-wide ${
                armed ? "text-ember-100" : "text-slate-200"
              }`}
            >
              {armed ? "ARMED — motion buttons live" : "DISARMED — nothing on this page will move the rover"}
            </span>
            {armed && armedAt !== null ? (
              <span className="font-mono text-[11px] text-slate-400">
                for {ageText(Math.max(0, now - armedAt))}
              </span>
            ) : null}
          </div>
          <p className="mt-1 max-w-3xl text-xs text-slate-400">
            {armed ? (
              <>
                FOLLOW is enabled for any run whose route has been planned and
                drawn. <b>Clear the arena.</b> {MISSION_ARM_NOTE}
              </>
            ) : (
              <>
                PLAN, TEST ENCODERS and ABORT still work — none of them command
                motion. {MISSION_ARM_NOTE}
              </>
            )}
          </p>
        </div>
        <div className="flex shrink-0 items-center gap-2">
          {armed ? (
            <button
              className="btn"
              onClick={onDisarm}
              title="Close the gate. Motion buttons on this page stop working immediately. It does NOT stop a mission that is already running — use ABORT for that."
            >
              DISARM
            </button>
          ) : (
            <button
              className="btn-hot"
              onClick={onArm}
              disabled={!thing}
              title={
                thing
                  ? `Open this console's gate for ${thing}. Enforced by the dashboard only — the rover has no arm concept. Every FOLLOW still needs its own plan and its own confirmation.`
                  : "No rover is reporting."
              }
            >
              ARM THE CONSOLE
            </button>
          )}
        </div>
      </div>
    </div>
  );
}

/**
 * One run, drawn in its physical position in a 2x2 grid, with its own PLAN and
 * its own FOLLOW.
 *
 * The corner leads, the operator's name for the run follows, and the mission id
 * is printed as the small monospace token it is. That ordering is the whole
 * point: an operator picks a corner of a room, and the id is what the dashboard
 * happens to have to send.
 *
 * When the rover has announced its own label and coordinates for the target we
 * show them, and if they disagree with the arena constants by more than a
 * millimetre we say so rather than choosing a winner — a card that quietly
 * renders one set of coordinates while the rover drives to another is how the
 * next wrong-corner incident happens.
 */
function MissionCard({
  t,
  info,
  unlisted,
  focused,
  backend,
  status,
  gate,
  armed,
  noRover,
  activeNow,
  onPlan,
  onFollow,
  onShow,
  onClear,
}: {
  t: Target;
  info?: MissionInfo;
  unlisted: boolean;
  focused: boolean;
  backend: string;
  status: PlanStatus;
  gate: Gate;
  armed: boolean;
  noRover: boolean;
  activeNow: boolean;
  onPlan: () => void;
  onFollow: () => void;
  onShow: () => void;
  onClear: () => void;
}) {
  const roverX = info?.x_mm ?? null;
  const roverY = info?.y_mm ?? null;
  const disagrees =
    roverX !== null && roverY !== null && t.x_mm !== null && t.y_mm !== null &&
    (Math.abs(roverX - t.x_mm) > 1 || Math.abs(roverY - t.y_mm) > 1);

  const ready = status.kind === "ready" && !status.expired;

  return (
    <div
      className={`flex h-full flex-col gap-2 rounded-xl border p-3 transition ${
        activeNow
          ? "border-ember-500/60 bg-ember-500/[0.10] shadow-glow"
          : focused
            ? "border-sky-500/40 bg-sky-500/[0.05]"
            : "border-white/10 bg-white/[0.03]"
      }`}
    >
      <div className="flex items-start justify-between gap-2">
        <div className="min-w-0">
          <div className="flex flex-wrap items-center gap-1.5">
            <span className="text-sm font-semibold tracking-wide text-slate-100">
              {t.corner}
            </span>
            {/* Geometry-derived, so this badge follows the zones if they move.
                It is the operator's "Zone 1" and it is deliberately attached to
                the corner rather than to the id. */}
            {AHEAD_MISSION === t.mission ? (
              <span
                className="rounded border border-sky-400/40 bg-sky-500/10 px-1 py-px text-[9px] font-medium tracking-wide text-sky-200"
                title="From the start box the rover faces this zone at heading 90°. This is the one people call Zone 1."
              >
                STRAIGHT AHEAD
              </span>
            ) : null}
            {activeNow ? (
              <span className="chip-hot text-[9px]">RUNNING NOW</span>
            ) : null}
          </div>
          <div className="mt-0.5 text-base font-semibold leading-tight text-slate-50">
            {t.title}
          </div>
        </div>
        <button
          className="chip shrink-0 text-[10px]"
          onClick={onShow}
          title="Draw this run's planned route on the map (if one has been planned)."
        >
          {focused ? "ON MAP" : "SHOW"}
        </button>
      </div>

      <div className="text-xs text-slate-300">
        {t.place}
        {t.role ? <span className="ml-1 text-[10px] text-slate-500">· {t.role}</span> : null}
        {info?.label && info.label !== t.place ? (
          <span className="ml-1 text-[10px] text-slate-500">
            · rover calls it {info.label}
          </span>
        ) : null}
      </div>
      <div className="font-mono text-[11px] text-slate-400">
        mission {t.mission} ·{" "}
        {t.x_mm === null || t.y_mm === null
          ? "-- , --"
          : `${Math.round(t.x_mm)}, ${Math.round(t.y_mm)}`}
      </div>
      <div className="text-[11px] leading-snug text-slate-500">{t.detail}</div>

      {disagrees ? (
        <span
          className="chip-warn"
          title="The rover announced different coordinates for this mission than the arena constants in this dashboard. Neither is overridden here."
        >
          rover says {Math.round(roverX!)}, {Math.round(roverY!)}
        </span>
      ) : null}
      {unlisted ? (
        <span
          className="chip-warn"
          title="This rover announced its mission list and this name was not on it. The button still sends — the announcement is a snapshot, not a contract — but expect a refusal."
        >
          not in the rover's list
        </span>
      ) : null}

      {/* -------------------------------------------------------- 1 · PLAN */}
      <div className="mt-auto border-t border-white/[0.06] pt-2">
        <div className="flex flex-wrap items-center gap-2">
          <button
            className="btn text-xs"
            onClick={onPlan}
            disabled={noRover}
            title={
              noRover
                ? "No rover is reporting."
                : `Ask the executor for the route to ${t.mission} (${t.corner}) on ${backend}. Commands no motion — available disarmed and while FOLLOW is locked out.`
            }
          >
            1 · PLAN
          </button>
          {status.kind === "ready" ? (
            <button
              className="chip text-[10px]"
              onClick={onClear}
              title="Forget this route. FOLLOW locks again until it is re-planned."
            >
              CLEAR
            </button>
          ) : null}
        </div>
        <PlanLine status={status} />
      </div>

      {/* ------------------------------------------------------ 2 · FOLLOW */}
      <div>
        <ConfirmButton
          className={`w-full justify-center py-3 text-sm ${ready && armed ? "btn-primary" : "btn"}`}
          label={`2 · FOLLOW → ${t.corner}`}
          disabled={!gate.ok}
          onFire={onFollow}
          hint={gate.why}
        />
        <p
          className={`mt-1 text-[10px] leading-snug ${
            gate.ok ? "text-slate-500" : "text-amber-200/70"
          }`}
        >
          {gate.why}
        </p>
      </div>
    </div>
  );
}

/** The one line under PLAN that says what the preview did. */
function PlanLine({ status }: { status: PlanStatus }) {
  if (status.kind === "none") {
    return (
      <div className="mt-1 text-[10px] text-slate-500">
        No route planned. FOLLOW stays locked until one has been drawn.
      </div>
    );
  }
  if (status.kind === "waiting") {
    return (
      <div className="mt-1 text-[10px] text-sky-200/80">
        planning… asked {ageText(status.sinceMs)} ago
      </div>
    );
  }
  if (status.kind === "silent") {
    return (
      <div className="mt-1 text-[10px] text-amber-200/85">
        NO ROUTE CAME BACK in {ageText(status.sinceMs)} — the mission executor
        may not be running.
      </div>
    );
  }
  if (status.kind === "refused") {
    return (
      <div className="mt-1 text-[10px] text-rose-200/90">
        PLAN REFUSED — {status.error}
      </div>
    );
  }
  const p = status.rec.plan;
  return (
    <div
      className={`mt-1 text-[10px] ${status.expired ? "text-amber-200/85" : "text-sky-200/85"}`}
    >
      route · {mm(p.distanceMm)} · {p.segments === null ? "--" : p.segments} segments
      {p.legsN !== null ? ` · ${p.legsN} leg${p.legsN === 1 ? "" : "s"}` : ""} · ETA{" "}
      {p.etaS === null ? "--" : `~${secs(p.etaS)}`}
      {p.returnsHome ? " · returns home" : ""}
      <span className="ml-1 opacity-70">
        (planned {ageText(status.ageMs)} ago{status.expired ? " — EXPIRED, re-plan" : ""})
      </span>
    </div>
  );
}

/**
 * The planned route in full, under the map.
 *
 * NOMINAL is the operative word and it is stated rather than implied: the
 * executor re-measures its bearing after every leg and inserts correction
 * turns, so the driven path will not match the dashed line exactly. The leg
 * list is what "follows the route" actually means, in order, with the live leg
 * marked when the rover is on it.
 */
function PlanDetail({ plan, live }: { plan: MissionPlan; live: number | null }) {
  return (
    <div className="mt-3 rounded-lg border border-sky-500/30 bg-sky-500/5 px-3 py-2 text-xs text-sky-100/90">
      <div className="font-mono">
        {plan.mission ?? "--"} · {mm(plan.distanceMm)} ·{" "}
        {plan.segments === null ? "--" : plan.segments} segments · ETA{" "}
        {plan.etaS === null ? "--" : `~${secs(plan.etaS)}`}
        {plan.backend ? ` · via ${plan.backend}` : ""}
      </div>

      {plan.routeText ? (
        <div className="mt-1 text-[11px] text-sky-200/80">{plan.routeText}</div>
      ) : null}

      {plan.legs ? (
        <ol className="mt-2 space-y-1">
          {plan.legs.map((l, i) => {
            const isLive = live !== null && l.i !== null && l.i === live;
            return (
              <li
                key={`${l.name ?? "leg"}-${i}`}
                className={`flex flex-wrap items-baseline gap-x-2 rounded px-1.5 py-0.5 font-mono text-[11px] ${
                  isLive ? "bg-ember-500/15 text-ember-100" : "text-sky-100/85"
                }`}
              >
                <span className="opacity-70">{l.i ?? i + 1}.</span>
                <span>{l.label ?? l.name ?? "--"}</span>
                <span className="opacity-70">{l.name ?? ""}</span>
                <span className="opacity-70">
                  {l.x_mm === null || l.y_mm === null
                    ? "--, --"
                    : `${Math.round(l.x_mm)}, ${Math.round(l.y_mm)}`}
                </span>
                <span className="opacity-70">{mm(l.distanceMm)}</span>
                <span className="opacity-70">
                  {l.segments === null ? "--" : `${l.segments} seg`}
                </span>
                {isLive ? <span className="chip-hot text-[9px]">ON THIS LEG</span> : null}
              </li>
            );
          })}
        </ol>
      ) : null}

      <div className="mt-2 text-[11px] text-sky-200/70">
        From{" "}
        {plan.fromX === null || plan.fromY === null
          ? "-- , --"
          : `${Math.round(plan.fromX)}, ${Math.round(plan.fromY)}`}
        {plan.fromHeadingDeg === null ? "" : ` at ${Math.round(plan.fromHeadingDeg)}°`}
        {plan.returnStrategy ? ` · return: ${plan.returnStrategy}` : ""}. Nominal
        route: the executor re-measures its bearing after every leg and inserts
        correction turns, so the driven path will differ from the dashed line.
        {plan.poseAssumed
          ? " The start pose is ASSUMED — nothing localises this rover, so the whole route is only as right as that assumption."
          : ""}
      </div>
    </div>
  );
}

/**
 * TEST ENCODERS.
 *
 * DELIBERATELY NOT CALLED "read the encoders", because this board has none to
 * read. The Yahboom MicroROS Board V2.0 publishes an integrated pose and a
 * twist on /odom_raw and exposes no per-wheel counters at all, so `ticks` is
 * shown as unavailable rather than being synthesised from a guessed wheel
 * radius — a number that looked like a measurement would be worse than a blank.
 *
 * Commands no motion, so it is outside the arm gate and outside the link lock:
 * a stale reading, labelled stale, is exactly what is wanted while diagnosing
 * why the link is stale.
 */
function EncoderCard({
  thing,
  askedAt,
  now,
  reply,
  supported,
  onTest,
}: {
  thing: string | null;
  askedAt: number | null;
  now: number;
  reply: { at: number; reading: EncoderReading | null } | null;
  /** null = the rover has not announced its action list. Unknown ≠ unsupported. */
  supported: boolean | null;
  onTest: () => void;
}) {
  const waiting =
    askedAt !== null && (reply === null || reply.at < askedAt) && now - askedAt < PLAN_WAIT_MS;
  const silent =
    askedAt !== null && (reply === null || reply.at < askedAt) && now - askedAt >= PLAN_WAIT_MS;
  const r = reply?.reading ?? null;

  return (
    <Card>
      <CardHeader
        title="Test the encoders"
        subtitle="read_encoders · commands no motion"
        right={
          <div className="flex flex-wrap items-center gap-2">
            {supported === false ? (
              <span
                className="chip-warn"
                title="This rover announced its action list and read_encoders was not on it. The button still sends — an announcement is a snapshot, not a contract."
              >
                not in the rover's action list
              </span>
            ) : null}
            <button
              className="btn"
              onClick={onTest}
              disabled={!thing}
              title={
                thing
                  ? "Ask the drive bridge for its odometry. No motion is commanded, so this works disarmed, with a mission running, and with the link down — a stale reading labelled stale is what you want when diagnosing the link."
                  : "No rover is reporting."
              }
            >
              TEST ENCODERS
            </button>
          </div>
        }
      />

      <div className="mb-3 rounded-lg border border-amber-500/40 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90">
        <b>There are no encoder ticks on this rover.</b> The Yahboom MicroROS
        Board V2.0 publishes an integrated pose and a twist on{" "}
        <span className="font-mono">/odom_raw</span> and nothing else — no{" "}
        <span className="font-mono">/wheel_ticks</span>, no joint states, no
        per-wheel counters. What this test proves is that <i>odometry is moving
        and plausible</i>, which is the thing every mission's dead reckoning
        depends on. It does not prove a wheel turned.
      </div>

      {waiting ? (
        <p className="text-sm text-sky-200/85">Asked. Waiting for the reply…</p>
      ) : null}
      {silent ? (
        <p className="text-sm text-amber-200/90">
          NO REPLY in {ageText(now - (askedAt ?? now))}. fpms-teleop answers this
          verb even with the micro-ROS link down, so silence points at the
          bridge process or the broker rather than at the drive board.
        </p>
      ) : null}

      {r === null ? (
        !waiting && !silent ? (
          <p className="text-sm text-slate-400">
            Nothing read back yet. Press TEST ENCODERS.
          </p>
        ) : null
      ) : (
        <>
          <div className="grid grid-cols-2 gap-x-4 gap-y-4 sm:grid-cols-3 lg:grid-cols-6">
            <Stat
              label="raw ticks"
              value={
                r.ticksAvailable
                  ? `${r.ticksLeft ?? "--"} / ${r.ticksRight ?? "--"}`
                  : "not published"
              }
              note={r.ticksAvailable ? "left / right" : "this board has no counters"}
            />
            <Stat label="odom x" value={metres(r.xM)} note={r.sourceTopic ?? undefined} />
            <Stat label="odom y" value={metres(r.yM)} />
            <Stat label="heading" value={degrees(r.headingDeg)} note="from /odom_raw" />
            <Stat
              label="arena pose"
              value={
                r.arenaXmm === null || r.arenaYmm === null
                  ? "--"
                  : `${Math.round(r.arenaXmm)}, ${Math.round(r.arenaYmm)}`
              }
              note="mm · anchored, assumed"
            />
            <Stat
              label="ground speed"
              value={mps(r.groundSpeedMps)}
              note="differentiated from pose"
              accent
            />
            <Stat
              label="twist linear"
              value={mps(r.twistLinMps)}
              note="board's own, sign-corrected"
            />
            <Stat label="twist angular" value={radps(r.twistAngRadps)} note="NOT sign-corrected" />
            <Stat label="gyro z" value={radps(r.gyroZRadps)} note="from /imu" />
            <Stat label="yaw integrated" value={degrees(r.yawIntegratedDeg)} note="from gyro" />
            <Stat label="odom rate" value={hertz(r.hz)} note={r.frames === null ? undefined : `${r.frames} frames`} />
            <Stat
              label="odom age"
              value={r.ageS === null ? "--" : secs(r.ageS)}
              bad={r.stale === true}
              note={r.stale === true ? "STALE" : r.rosOk === true ? "link ok" : undefined}
            />
          </div>

          <div className="mt-3 flex flex-wrap items-center gap-2">
            {r.stale === true ? (
              <span className="chip-bad">odometry stale — this is history, not state</span>
            ) : null}
            {r.rosOk === false ? <span className="chip-bad">micro-ROS link down</span> : null}
            <span className="font-mono text-[11px] text-slate-500">
              read {reply ? new Date(reply.at).toLocaleTimeString() : "--"}
            </span>
          </div>

          {r.note ? (
            <p className="mt-2 text-[11px] leading-snug text-slate-500">{r.note}</p>
          ) : null}
          <p className="mt-2 text-[11px] leading-snug text-slate-500">
            <b className="text-slate-400">How to read it:</b> ground speed is
            differentiated from consecutive /odom_raw POSITIONS by the bridge and
            is the trustworthy figure — pose was correct in every bench trial
            while the board's own twist was not. To test the encoders under
            motion you need the rover to move, which means arming and following
            a short run: RETURN HOME from the start box is the smallest one.
          </p>
        </>
      )}
    </Card>
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
 *
 * `poised`, not "armed": ARM on this page means the console's safety gate, and
 * two things called armed in one file is how the wrong one gets checked.
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
  const [poised, setPoised] = useState(false);
  const timer = useRef<number | null>(null);

  useEffect(() => () => { if (timer.current) window.clearTimeout(timer.current); }, []);

  // A control that goes dead while poised must not stay poised — when it comes
  // back it would fire on a single click.
  useEffect(() => {
    if (disabled) setPoised(false);
  }, [disabled]);

  const click = () => {
    if (poised) {
      if (timer.current) window.clearTimeout(timer.current);
      setPoised(false);
      onFire();
      return;
    }
    setPoised(true);
    timer.current = window.setTimeout(() => setPoised(false), 4000);
  };

  return (
    <button
      onClick={click}
      disabled={disabled}
      className={`${className} ${poised ? "border-amber-500/50 bg-amber-500/15 text-amber-100" : ""}`}
      title={hint ?? "Requires a second click to confirm"}
    >
      {poised ? "CLICK AGAIN TO SEND THE ROVER" : label}
    </button>
  );
}

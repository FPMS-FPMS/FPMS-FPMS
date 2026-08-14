import { useEffect, useMemo, useRef, useState } from "react";

import { Card, CardHeader } from "./Card";
import { apiPostJson } from "../lib/api";
import {
  FORWARD_HEADING_DEG,
  ROVER_START,
  START_BOX,
  ZONES,
  ZONE_AHEAD_OF_START,
  type ArenaCorner,
  type Zone,
} from "../lib/arena";
import { type MissionInfo, type RoverCapabilities } from "../lib/capabilities";
import {
  MISSION_ABORT_ACTION,
  MISSION_ARM_NOTE,
  PLAN_MAX_AGE_MS,
  PLAN_WAIT_MS,
  postMissionAbort,
  readEnvelope,
  readPlan,
  type MissionFeed,
  type MissionPlan,
  type RoverEventFeed,
} from "../lib/mission";

/**
 * THE MISSION CONSOLE — arm it, plan a route, follow the route, abort.
 *
 * This was the body of the Mission page, and it is here because the page it
 * lived on is not the page the operator is looking at. Someone standing beside
 * the arena watches the LiDAR tab: the map, the planned line and the front
 * clearance are there, and the four buttons that start the thing they are
 * watching were one tab away. Two copies of these controls was never an option
 * — the gates below are the only reason a click is survivable — so the controls
 * moved into a component and BOTH pages render this one.
 *
 * The sequence is the one the operator actually performs:
 *
 *      0  ARM       a deliberate, separate act. Nothing here moves until it is
 *                   done, and it un-does itself on every abort
 *      1  PLAN      per mission. Asks for the route, commands no motion, draws
 *                   the line on the arena map
 *      2  FOLLOW    the same route, for real. Confirm-gated, ARMED-gated, and
 *                   IMPOSSIBLE before that mission's own plan has been shown
 *      -  ABORT     always on screen, always enabled, at every step
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
 * HOW A PAGE USES THIS
 * ---------------------------------------------------------------------------
 * `useMissionConsole` holds the state and owns every command; the components
 * below only draw it. That split is not decoration: ABORT lives in a different
 * place on each page — a sticky bar on Mission, the MissionStrip on LiDAR — and
 * ABORT is what SHUTS the arm gate. A page therefore holds the console object
 * and wires its own stop button to `mc.abort`, rather than each page owning a
 * private arm flag that the other page's abort could not reach.
 *
 * The feeds are passed IN rather than subscribed here. Every `useChannel` opens
 * its own socket, and the LiDAR page draws one console per bay — a hook that
 * subscribed for itself would quadruple the sockets for telemetry the card has
 * already got in its hand.
 *
 * ---------------------------------------------------------------------------
 * THE RULES THIS CONSOLE INHERITS, EACH FROM A REAL INCIDENT
 * ---------------------------------------------------------------------------
 *   A MISSING NUMBER IS `--`, NEVER 0. The executor omits fields it does not
 *   have; "0 mm remaining" reads as ARRIVED when it means NO IDEA. Every
 *   readout here goes through `mm`/`secs`/`metres` below, which take
 *   `number | null` and cannot be handed a default.
 *
 *   AN UNKNOWN PHASE COUNTS AS RUNNING. `useMission` decides that and this
 *   console never second-guesses it. Showing "idle" while the rover drives is
 *   how someone walks up to a moving machine.
 *
 *   ABORT IS `stop`, NOT `mission {name:"abort"}`. See MISSION_ABORT_NOTE:
 *   "abort" is not a commandable mission and the rover answered "unknown
 *   mission" while aborting nothing.
 *
 *   THERE IS NO SLOW SPEED. The firmware slams roughly 50% duty on any
 *   non-zero setpoint, so nothing here offers a gentle version of a move.
 *   Missions are made survivable by being SHORT and by ABORT being one key
 *   away, not by being slow.
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

/** The teleop verb behind SET COORDINATE. Writes a belief, moves nothing. */
const SET_COORDINATE_ACTION = "set_coordinate";

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
 * added, moved or recoloured in lib/arena shows up with the right corner and
 * the right coordinates, and one removed loses its card instead of leaving a
 * control that points at nothing.
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
 * Every readout here goes through one of these.
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

/** "12s" / "3m 20s" of wall-clock age. Only ever called with a real number. */
function ageText(ms: number): string {
  return secs(ms / 1000);
}

/* --------------------------------------------------------------- the console */

/** Drive telemetry older than this means we no longer know the rover's state. */
const DRIVE_STALE_MS = 5000;

/** A previewed route, remembered per mission with the time it landed. */
type PlanRec = { plan: MissionPlan; at: number };

/**
 * Why FOLLOW is or is not available for one mission. `why` is rendered
 * verbatim under the button — a disabled control that does not say what would
 * enable it is just a dead control.
 */
export type Gate = { ok: boolean; why: string };

/** What has happened to this mission's preview, as far as the console knows. */
export type PlanStatus =
  | { kind: "none" }
  | { kind: "waiting"; sinceMs: number }
  | { kind: "silent"; sinceMs: number }
  | { kind: "refused"; error: string }
  | { kind: "ready"; rec: PlanRec; ageMs: number; expired: boolean };

/**
 * What a console log line MEANS, which is what decides how it is painted.
 *
 *   ok    a command left the browser and was accepted.
 *   info  something is IN FLIGHT and has no outcome yet. Never a success.
 *   gate  the arm gate opened or shut — the operator's own safety state.
 *   bad   a refusal, or a precondition that failed before anything was sent.
 *   err   a command that HAD to work and did not. The loudest kind there is.
 *
 * This alias exists because the union used to be written out twice — once on
 * LogEntry and once on say() — and both copies drifted from the calls. abort()
 * was already saying "info" and "err" against a union containing neither,
 * which is the three TS2345 errors that made `tsc -b` fail and therefore
 * blocked every Windows rebuild. One name, one place to add a kind.
 */
type LogKind = "ok" | "info" | "gate" | "bad" | "err";

type LogEntry = { at: number; kind: LogKind; text: string };

/**
 * How each kind is painted.
 *
 * A RECORD, not the if/else chain this used to be. Adding a kind to LogKind
 * without giving it a tone is now a compile error, where before it silently
 * fell through to the neutral default — which is how an "err" on the ABORT
 * path would have been painted identically to an "ok".
 *
 * `err` is deliberately louder than `bad`: a heavier border and a filled
 * background, matching the banner style the Telemetry page already uses for
 * its two unmissable states. The one control that has to work when everything
 * else has failed does not get to look like ordinary bad news.
 */
const LOG_TONE: Record<LogKind, string> = {
  ok: "border border-white/10 bg-white/[0.04] text-slate-300",
  info: "border border-sky-500/30 bg-sky-500/[0.06] text-sky-100/90",
  gate: "border border-amber-500/30 bg-amber-500/[0.06] text-amber-100/90",
  bad: "border border-rose-500/40 bg-rose-500/10 text-rose-100",
  err: "border-2 border-rose-500/60 bg-rose-950/40 text-rose-100",
};

/** The bare shape of a `useChannel` result — only the parts the console reads. */
type Feed = { data: unknown; messages: number; lastAt: number | null };

export type MissionConsoleInput = {
  thing: string | null;
  caps: RoverCapabilities;
  /** `mission_plan:<thing>`, subscribed by the page. */
  plan: Feed;
  /** `useMission(thing)`. */
  feed: MissionFeed;
  /** `useRoverEvents(thing)` — where a PLAN refusal arrives. */
  events: RoverEventFeed;
  /** `drive:<thing>`, read only for the motion lock and the origin warning. */
  drive: Feed;
};

/** Everything the console knows and everything it can do. */
export type MissionConsoleState = MissionConsoleInput & {
  now: number;
  armed: boolean;
  armedAt: number | null;
  arm: () => void;
  disarm: (reason: string) => void;
  abort: () => void;
  backend: string;
  backends: readonly string[];
  pickBackend: (id: string) => void;
  /** Missions the rover has announced, or null when it has said nothing. */
  announced: ReadonlySet<string> | null;
  focus: string;
  setFocus: (name: string) => void;
  /** The previewed route the map should be drawing, or null. */
  focusPlan: MissionPlan | null;
  statusFor: (name: string) => PlanStatus;
  gateFor: (name: string) => Gate;
  planTarget: (name: string) => void;
  followTarget: (name: string) => void;
  clearPlan: (name: string) => void;
  testEncoders: () => void;
  encAskedAt: number | null;
  setCoordinate: (x_mm: number, y_mm: number) => void;
  /** True when the rover says it has no arena origin. null = it has not said. */
  originSet: boolean | null;
  running: boolean;
  noRover: boolean;
  motionLocked: boolean;
  rosDown: boolean;
  lockReason: string | null;
  log: LogEntry[];
};

/**
 * The console's state and its commands.
 *
 * Held by the page rather than by the drawing components below, because ABORT
 * is rendered somewhere different on each page and ABORT is what closes the arm
 * gate.
 */
export function useMissionConsole(input: MissionConsoleInput): MissionConsoleState {
  const { thing, caps, plan: planCh, feed, events, drive } = input;
  const m = feed.mission;
  const tele = readEnvelope(drive.data);

  // Wall-clock tick. Staleness and plan age have to be functions of time, not
  // of arriving data: without this a feed that simply STOPS renders its last
  // value as current forever, which is the exact failure the console is here to
  // catch, and a three-minute-old route would never age out of the gate.
  const [now, setNow] = useState<number>(() => Date.now());
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, []);

  // --------------------------------------------------------- capabilities ---
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
  const [log, setLog] = useState<LogEntry[]>([]);
  const say = (kind: LogKind, text: string) =>
    setLog((prev) => [{ at: Date.now(), kind, text }, ...prev].slice(0, 8));

  /* ------------------------------------------------------------- ARM/SAFE --
   *
   * THE GATE IS THIS CONSOLE'S, AND THE CONSOLE SAYS SO.
   *
   * There is no arm concept anywhere on the rover: fpms-missions accepts a
   * `mission` from anything that can reach the broker. This state variable
   * gates the buttons here and nothing else, which is worth having — every
   * incident on this project started with one unconsidered click — but must
   * never be dressed up as an interlock on the machine.
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

  // ----------------------------------------------------------- motion lock ---
  const driveAgeMs = drive.lastAt === null ? null : now - drive.lastAt;
  const driveStale = driveAgeMs !== null && driveAgeMs > DRIVE_STALE_MS;
  const rosDown = tele?.ros_ok === false;
  /**
   * FOLLOW is blocked when we positively know the link is bad. "Nothing has
   * ever arrived" is deliberately NOT in that set — if drive telemetry is not
   * plumbed through on a deployment, locking the console would leave an
   * operator with controls that cannot work and no explanation.
   *
   * PLAN, TEST ENCODERS, SET COORDINATE and ABORT are outside this entirely.
   */
  const motionLocked = rosDown || driveStale;
  const lockReason = rosDown
    ? "The rover reports ROS is down. It cannot act on a motion command."
    : driveStale
      ? `No drive telemetry for ${Math.round((driveAgeMs ?? 0) / 1000)}s — the rover's state is unknown.`
      : null;

  const running = !!m?.running;
  const noRover = !thing;
  const originSet =
    tele?.origin_set === undefined ? null : tele.origin_set === true;

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

  const clearPlan = (name: string) => {
    setPlans((prev) => {
      const next = { ...prev };
      delete next[name];
      return next;
    });
    setAsked((prev) => {
      const next = { ...prev };
      delete next[name];
      return next;
    });
  };

  /** TEST ENCODERS. Reads odometry back; commands no motion, so no arm gate. */
  const [encAskedAt, setEncAskedAt] = useState<number | null>(null);
  const testEncoders = () => {
    setEncAskedAt(Date.now());
    send(ENCODER_ACTION, {}, "TEST ENCODERS (read_encoders)");
  };

  /**
   * SET COORDINATE. Writes what the rover BELIEVES about where it is.
   *
   * Outside the arm gate and outside the motion lock because it commands no
   * motion — and because the state it fixes, a dead-reckoning integration that
   * has come apart, is exactly the state in which the motion lock is closed.
   */
  const setCoordinate = (x_mm: number, y_mm: number) => {
    send(SET_COORDINATE_ACTION, { x_mm, y_mm }, `SET COORDINATE ${x_mm}, ${y_mm}`);
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
    // FIXED 2026-08-07. This used to say("ok", "… — sent") on the line BEFORE
    // the post, and postMissionAbort swallows its own failure
    // (lib/mission.ts: `.catch(() => undefined)`). So a 500, a dead backend or
    // a dropped request painted a green "sent" for a command that never left
    // the browser — on the one control that has to work exactly when
    // everything else has gone wrong. Report the attempt, then correct it from
    // the outcome, which is what the sibling send() at :553 already does.
    say("info", `ABORT (${MISSION_ABORT_ACTION}) — sending…`);
    postMissionAbort(thing)
      .then((ok) =>
        ok
          ? say("ok", `ABORT (${MISSION_ABORT_ACTION}) — sent`)
          : say(
              "err",
              `ABORT (${MISSION_ABORT_ACTION}) — NOT SENT. The rover did not ` +
                `receive it. Use the physical stop.`,
            ),
      )
      .catch(() =>
        say(
          "err",
          `ABORT (${MISSION_ABORT_ACTION}) — NOT SENT. Use the physical stop.`,
        ),
      );
    // Disarm regardless: the console's own gate must shut even if the wire
    // failed, so nothing else can be launched on the strength of it.
    disarm("aborted");
  };

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

  return {
    ...input,
    now,
    armed,
    armedAt,
    arm,
    disarm,
    abort,
    backend,
    backends,
    pickBackend: setPickedBackend,
    announced,
    focus,
    setFocus,
    focusPlan,
    statusFor,
    gateFor,
    planTarget,
    followTarget,
    clearPlan,
    testEncoders,
    encAskedAt,
    setCoordinate,
    originSet,
    running,
    noRover,
    motionLocked,
    rosDown,
    lockReason,
    log,
  };
}

/**
 * ARM, the four runs, the backend they are driven on, SET COORDINATE and the
 * log — the whole console except the stop button, which each page places where
 * that page's operator already looks for it.
 *
 * Drawn as a bare section rather than a Card so it can sit inside one: on the
 * LiDAR tab this goes under the mission strip of a rover card that already has
 * its own frame, and a card inside a card reads as a bug.
 */
export function MissionConsole({
  mc,
  className = "",
}: {
  mc: MissionConsoleState;
  className?: string;
}) {
  const { caps, thing } = mc;

  return (
    <section className={className}>
      <CardHeader
        title="The four runs — PLAN first, then FOLLOW"
        subtitle="Laid out like the arena, seen from the start box"
        right={
          <div className="flex flex-wrap items-center gap-2">
            <span
              className={mc.announced ? "chip-ok" : "chip-warn"}
              title={
                mc.announced
                  ? "This rover announced its mission list; targets it does not list are marked."
                  : "The executor has not announced its mission list — events/online is published once and is not retained. All four stay available; silence is not a refusal."
              }
            >
              {mc.announced ? "list from rover" : "not yet announced"}
            </span>
            <span className={mc.armed ? "chip-hot" : "chip"} title={MISSION_ARM_NOTE}>
              {mc.armed ? "ARMED" : "DISARMED"}
            </span>
          </div>
        }
      />

      {/* ------------------------------------------------------- 0 · ARM */}
      <ArmBar
        armed={mc.armed}
        armedAt={mc.armedAt}
        now={mc.now}
        thing={thing}
        onArm={mc.arm}
        onDisarm={() => mc.disarm("disarmed by the operator")}
      />

      {/*
        THE THING THAT SENDS ROVERS TO THE WRONG CORNER, said out loud.
        Which zone is "in front" comes from the geometry (AHEAD), not from
        a sentence typed here, and NOTHING is renumbered to make the ids
        agree with the spoken names — the mismatch is real, it lives on the
        rover, and hiding it would only move the surprise somewhere worse.
      */}
      <div className="my-4 rounded-lg border border-amber-500/40 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90">
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

      {/* ------------------------------------------- 1/2 · the four missions */}
      <div className="grid gap-3 md:grid-cols-2">
        {TARGETS.map((t) => (
          <MissionCard
            key={t.mission}
            t={t}
            info={caps.missionInfo[t.mission]}
            unlisted={!!mc.announced && !mc.announced.has(t.mission)}
            focused={mc.focus === t.mission}
            backend={mc.backend}
            status={mc.statusFor(t.mission)}
            gate={mc.gateFor(t.mission)}
            armed={mc.armed}
            noRover={mc.noRover}
            activeNow={mc.running && mc.feed.mission?.mission === t.mission}
            onPlan={() => mc.planTarget(t.mission)}
            onFollow={() => mc.followTarget(t.mission)}
            onShow={() => mc.setFocus(t.mission)}
            onClear={() => mc.clearPlan(t.mission)}
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
          {mc.backends.map((id) => (
            <button
              key={id}
              onClick={() => mc.pickBackend(id)}
              className={`chip ${mc.backend === id ? "border-ember-500/40 bg-ember-500/10 text-ember-200" : ""}`}
              title={BACKEND_WARN[id] ?? "Backend the mission will be planned and driven with. Changing it clears every planned route — a route is only valid for the planner that produced it."}
            >
              <span className="font-mono">{id}</span>
              {caps.defaultBackend === id && (
                <span className="ml-1.5 text-[10px] text-slate-500">· rover default</span>
              )}
            </button>
          ))}
        </div>
        {BACKEND_WARN[mc.backend] ? (
          <div className="mt-2 rounded-lg border border-amber-500/40 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90">
            {BACKEND_WARN[mc.backend]}
          </div>
        ) : null}
      </div>

      {/* --------------------------------------------- where it thinks it is */}
      <div className="mt-5 border-t border-white/[0.06] pt-4">
        <div className="lbl mb-2">Set coordinate — tells the rover where it is</div>
        {/* The rover says whether it has an origin at all, and the answer
            changes what this control is for: with no origin the pose is
            measured from wherever the bridge happened to start, so this is
            not a correction, it is the thing that makes the arena frame
            exist. */}
        {mc.originSet === false && (
          <div className="mb-3 rounded-lg border border-amber-500/40 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90">
            <b>No origin is set on this rover.</b> x/y are being measured from
            wherever the teleop bridge started, not in the arena frame the map
            draws in, so every coordinate here and every mission planned from it
            is offset by an unknown amount. Set it before running anything.
          </div>
        )}
        <SetCoordinate disabled={mc.noRover} onFire={mc.setCoordinate} />
        <p className="mt-3 text-xs text-slate-500">
          Writes the rover's believed position in arena millimetres. It moves
          nothing, so there is nothing to confirm and it stays available when the
          motion controls are locked — but the number has to be right, or every
          mission afterwards is wrong by the same amount.{" "}
          <b>Re-set it after every boot.</b> The origin file survives a reboot
          while the board's odometry restarts at zero, so a stale reference is
          not a small error — it has already placed a rover nine metres outside a
          1.2 m arena, and the planner believed it.
        </p>
      </div>

      {/* ------------------------------------------------ the speed truth */}
      <p className="mt-4 border-t border-white/[0.06] pt-4 text-xs text-slate-500">
        <b className="text-slate-400">This rover has one speed.</b> The
        firmware slams roughly 50 % duty on any non-zero setpoint on
        /cmd_vel, so there is no crawl to fall back on and nothing here can
        ask for one — a mission is made survivable by being short and by
        ABORT being one key away, not by being gentle. Manual jogging lives
        on the Drive tab, which pulses and then fully stops for the same
        reason.
      </p>

      {mc.log.length ? (
        <div className="mt-4 space-y-1 border-t border-white/[0.06] pt-3">
          <div className="lbl mb-1">Console log</div>
          {mc.log.map((e) => (
            <div
              key={`${e.at}-${e.text}`}
              className={`rounded px-2 py-1 font-mono text-[11px] ${LOG_TONE[e.kind]}`}
            >
              <span className="opacity-60">
                {new Date(e.at).toLocaleTimeString()}
              </span>{" "}
              {e.text}
            </div>
          ))}
        </div>
      ) : null}
    </section>
  );
}

/* --------------------------------------------------------------- components */

/**
 * THE ARM GATE.
 *
 * Two states, one deliberate act between them, and a paragraph that refuses to
 * let the operator believe the rover is enforcing it. The DISARMED copy is the
 * important one: it says what is blocked (this console's motion buttons) and
 * what is NOT (the machine, the Drive tab, anything else holding the broker).
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
              {armed ? "ARMED — motion buttons live" : "DISARMED — nothing here will move the rover"}
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
              title="Close the gate. Motion buttons here stop working immediately. It does NOT stop a mission that is already running — use ABORT for that."
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
 * SET COORDINATE.
 *
 * Exported because the Drive tab has its own card for this and used to hold a
 * second copy of the control. One implementation, so the rule about the empty
 * boxes below cannot be relaxed on one page and not the other.
 */
export function SetCoordinate({
  onFire,
  disabled,
}: {
  onFire: (x: number, y: number) => void;
  disabled: boolean;
}) {
  // Empty, not "0". These boxes used to arrive pre-filled with 0/0, so the
  // control was one click away from publishing "you are at the arena origin"
  // — a coordinate nobody typed and nothing measured, on a page that may never
  // have heard from the rover at all. An unfilled box disables the button.
  const [x, setX] = useState("");
  const [y, setY] = useState("");
  const nx = Number(x);
  const ny = Number(y);
  const valid =
    x.trim() !== "" && y.trim() !== "" && Number.isFinite(nx) && Number.isFinite(ny);

  return (
    <div className="flex flex-wrap items-end gap-3">
      <label className="flex flex-col gap-1">
        <span className="lbl">x (mm)</span>
        <input
          type="number"
          value={x}
          onChange={(e) => setX(e.target.value)}
          placeholder="--"
          className="w-28 rounded-lg border border-white/10 bg-black/40 px-2 py-1.5 font-mono text-sm text-slate-200 outline-none focus:border-ember-500/40"
        />
      </label>
      <label className="flex flex-col gap-1">
        <span className="lbl">y (mm)</span>
        <input
          type="number"
          value={y}
          onChange={(e) => setY(e.target.value)}
          placeholder="--"
          className="w-28 rounded-lg border border-white/10 bg-black/40 px-2 py-1.5 font-mono text-sm text-slate-200 outline-none focus:border-ember-500/40"
        />
      </label>
      <button
        className="btn-primary"
        disabled={disabled || !valid}
        onClick={() => valid && onFire(nx, ny)}
        title={
          valid
            ? "Set the rover's believed position"
            : "Type both coordinates — there is no default, because a default here is a made-up position"
        }
      >
        Set coordinate
      </button>
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
 *
 * Its own Card, and separate from `MissionConsole`, because each page puts it
 * somewhere different — at the bottom on Mission, and nowhere at all on a LiDAR
 * card that is already the length of a page.
 */
export function MissionEncoders({ mc }: { mc: MissionConsoleState }) {
  const { thing, caps, now, encAskedAt: askedAt } = mc;
  const reply = mc.events.encoders;
  /** null = the rover has not announced its action list. Unknown ≠ unsupported. */
  const supported = caps.announced ? caps.actions.has(ENCODER_ACTION) : null;

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
              onClick={mc.testEncoders}
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
        <b>This test reads odometry, not ticks.</b> <span className="font-mono">
        read_encoders</span> is answered by fpms-teleop from the ROS side, which
        publishes an integrated pose and a twist on{" "}
        <span className="font-mono">/odom_raw</span> and nothing else — no{" "}
        <span className="font-mono">/wheel_ticks</span>, no joint states. What
        this test proves is that <i>odometry is moving and plausible</i>, which
        is the thing every mission's dead reckoning depends on. It does not
        prove a wheel turned.
        <br />
        <br />
        <b className="text-amber-100">Raw per-wheel counts do now exist</b> —
        just not in this reply.{" "}
        <span className="font-mono">fpms_stm32_bridge</span> reads all four
        counters off the board and publishes them at 5 Hz on{" "}
        <span className="font-mono">board:&lt;thing&gt;</span>, with the
        millimetres each wheel has travelled at 6.00 counts/mm. Read them on the{" "}
        <b>Telemetry</b> tab, BOARD card. That feed is what proves a wheel
        turned.
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
              note={r.ticksAvailable ? "left / right" : "not in this reply — see board:"}
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
export function Stat({
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
 * `poised`, not "armed": ARM here means the console's safety gate, and two
 * things called armed in one file is how the wrong one gets checked.
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

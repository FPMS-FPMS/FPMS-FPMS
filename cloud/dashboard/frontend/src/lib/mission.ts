import { useEffect, useRef, useState } from "react";
import { useChannel } from "./ws";
import { apiPostJson } from "./api";

/**
 * Mission state, in ONE place.
 *
 * This used to live inside Drive.tsx, which meant the mission — the one thing
 * on this dashboard that makes a machine drive itself across a room — was
 * visible on exactly one tab. An operator watching the LiDAR map while the
 * rover moved had no way to know a mission was running at all, and the Control
 * tab would happily offer `test_motors` mid-run.
 *
 * The rules encoded here are the ones that were learned the hard way, and every
 * consumer inherits them by construction rather than by remembering to
 * reimplement them:
 *
 *   1. A MISSING NUMBER IS NOT ZERO. Every field is `number | null`. The rover
 *      OMITS fields it does not have (see fpms_missions.snapshot(), which
 *      deliberately drops x_mm/y_mm rather than sending 0,0), so a consumer
 *      that coerces gets a confident lie about where the rover is.
 *
 *   2. AN UNKNOWN PHASE COUNTS AS RUNNING. `running` is false only for phases
 *      we positively recognise as not-driving. A phase this file has never
 *      heard of — a new one added on the rover, a typo, a truncated payload —
 *      reads as RUNNING. Showing "idle" while the rover drives is the failure
 *      that gets someone to walk up to a moving machine; the reverse is untidy.
 *
 *   3. STALE TELEMETRY DURING A RUN IS LOUD. `blind` is the case that matters:
 *      the last thing we heard was "driving", and we have not heard since.
 *
 * Field names here are read off the real publisher — fpms_missions.py
 * `snapshot()` — not guessed.
 */

/**
 * Mission telemetry older than this stops being the rover's state and starts
 * being history.
 *
 * Deliberately shorter than the Drive page's TELEMETRY_STALE_MS: a mission is a
 * machine driving itself, and "the progress bar is six seconds out of date" is a
 * materially different situation from a health panel being six seconds old.
 *
 * The executor throttles idle telemetry (IDLE_TELEM_S in fpms_missions.py), so
 * an IDLE mission legitimately goes quiet for a while. That is why staleness is
 * only ever escalated in combination with `running` — see `blind`.
 */
export const MISSION_STALE_MS = 6000;

/**
 * HOW A MISSION IS ACTUALLY ABORTED — and why it is not `mission {name:"abort"}`.
 *
 * Every ABORT button on this dashboard used to publish
 * `mission {name: "abort"}`. Neither publisher accepts that:
 *
 *   fpms_missions._cmd_mission  refuses any name outside COMMANDABLE
 *                               ("m1","m2","water","home","patrol") with
 *                               `unknown mission 'abort'`.
 *   fpms_teleop._cmd_mission    (the stub that answers only while the executor
 *                               is NOT publishing) refuses anything outside
 *                               ("m1","m2","water","home") the same way.
 *
 * So the button that exists for the moment a machine is driving somewhere wrong
 * produced a nack and no abort. What the executor DOES act on is
 * `stop`/`estop`/`auto_off`: `fpms_missions.handle_command` maps all three to
 * `request_abort(ABORT_STOP)` and deliberately publishes no reply, because
 * teleop owns the ack for those verbs. The rover announces this itself — it is
 * the `acts_silently_on` list in the missions service's `events/online`, which
 * lib/capabilities already records.
 *
 * `stop` is therefore the abort, and it is the right one for a second reason:
 * it halts the motors AND ends the executor in one publish. A `mission` verb
 * that only ended the executor would leave whatever teleop had on the wire.
 */
export const MISSION_ABORT_ACTION = "stop";

/** Tooltip/explanation shared by every abort control, so they cannot drift. */
export const MISSION_ABORT_NOTE =
  "Publishes `stop`. fpms-missions subscribes stop/estop/auto_off and aborts " +
  "the run instantly (it answers nothing — teleop owns the ack for those " +
  "verbs). The mission verb does not accept an `abort` name and would be " +
  "refused.";

/**
 * Send the abort. Never gated on anything: an abort has to work precisely when
 * every other check has decided things are wrong.
 *
 * Errors are swallowed by design for the callers that have no log of their own
 * — the acknowledgement lands in the Control and Drive command logs either way,
 * and a toast here would be one more thing between the operator and a second
 * press.
 */
export function postMissionAbort(thing: string): Promise<unknown> {
  return apiPostJson<unknown>(`/api/control/${thing}/${MISSION_ABORT_ACTION}`, {
    params: {},
  }).catch(() => undefined);
}

/**
 * Phases that mean "not driving".
 *
 * Sourced from MissionState.phase in fpms_missions.py
 * (idle|planning|outbound|docking|hold|return|reface|aborting) plus the
 * terminal words a future executor is likely to use. `planning` is NOT here on
 * purpose: the executor sets it after accepting the mission and before the
 * first segment, so it is a mission in progress, not an idle rover. `hold` is
 * not here either — the rover is parked at a target mid-mission and WILL move
 * again without anyone touching the dashboard.
 */
export const MISSION_IDLE_PHASES: ReadonlySet<string> = new Set([
  "idle", "ready", "none", "done", "complete", "completed", "finished",
  "aborted", "cancelled", "canceled", "failed", "error", "stopped",
]);

/**
 * Normalised mission state. Every field is optional because every field can be
 * genuinely absent: the executor may not be running, may have no pose, and may
 * have no LiDAR.
 */
export type MissionState = {
  mission: string | null;
  backend: string | null;
  phase: string | null;
  segmentI: number | null;
  segmentsN: number | null;
  segmentKind: string | null;
  /**
   * Progress through a multi-target route, as opposed to through the turn/drive
   * SEGMENTS of one leg. A patrol visits several corners, so "segment 3/7" and
   * "leg 2/4" answer different questions and both matter.
   *
   * `legLabel` is the rover's own plain-English name for the corner
   * (fpms_missions.TARGET_LABEL). It exists because the mission ids do not
   * match what an operator says out loud, and a card that reads "m1" where the
   * operator means the other zone is how a rover gets sent to the wrong corner.
   */
  legI: number | null;
  legsN: number | null;
  leg: string | null;
  legLabel: string | null;
  remainingMm: number | null;
  travelledMm: number | null;
  etaS: number | null;
  elapsedS: number | null;
  targetX: number | null;
  targetY: number | null;
  /**
   * The executor's OWN pose belief, in arena mm. Omitted by the rover when
   * there is no odometry — never zero-filled — so null here means "the mission
   * executor does not know where it is", which is a different and much more
   * alarming statement than "it is at the origin".
   */
  poseX: number | null;
  poseY: number | null;
  poseHeadingDeg: number | null;
  poseAssumed: boolean;
  odomSource: string | null;
  linkOk: boolean | null;
  battV: number | null;
  frontMm: number | null;
  lidarOk: boolean | null;
  /** Phase is not in MISSION_IDLE_PHASES. Unknown phases land here. */
  running: boolean;
};

export function num(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

function text(v: unknown): string | null {
  return typeof v === "string" && v.trim() !== "" ? v : null;
}

/** Pull the payload out of the bridge envelope, tolerating either shape. */
export function readEnvelope(env: unknown): Record<string, unknown> | null {
  if (!env || typeof env !== "object") return null;
  const data = (env as Record<string, unknown>).data;
  if (data && typeof data === "object" && !Array.isArray(data)) {
    return data as Record<string, unknown>;
  }
  return null;
}

export function readMission(raw: unknown): MissionState | null {
  if (!raw || typeof raw !== "object") return null;
  const d = raw as Record<string, any>;
  const phase = text(d.phase);
  const tgt = d.target && typeof d.target === "object" ? d.target : {};
  return {
    mission: text(d.mission),
    backend: text(d.backend),
    phase,
    segmentI: num(d.segment_i),
    segmentsN: num(d.segments_n),
    segmentKind: text(d.segment_kind),
    legI: num(d.leg_i),
    legsN: num(d.legs_n),
    leg: text(d.leg),
    legLabel: text(d.leg_label),
    remainingMm: num(d.distance_remaining_mm),
    travelledMm: num(d.distance_travelled_mm),
    etaS: num(d.eta_s),
    elapsedS: num(d.elapsed_s),
    targetX: num(tgt.x_mm),
    targetY: num(tgt.y_mm),
    poseX: num(d.x_mm),
    poseY: num(d.y_mm),
    poseHeadingDeg: num(d.heading_deg),
    poseAssumed: d.pose_assumed === true,
    odomSource: text(d.odom_source),
    linkOk: typeof d.link_ok === "boolean" ? d.link_ok : null,
    battV: num(d.batt_v),
    frontMm: num(d.front_mm),
    lidarOk: typeof d.lidar_ok === "boolean" ? d.lidar_ok : null,
    // Rule 2. Note the ordering: a phase we have never seen is NOT idle.
    running: phase !== null && !MISSION_IDLE_PHASES.has(phase),
  };
}

export type MissionFeed = {
  mission: MissionState | null;
  /** Age of the newest mission packet, ms. null = nothing has ever arrived. */
  ageMs: number | null;
  stale: boolean;
  /** Running AND stale — the rover was driving when the feed went quiet. */
  blind: boolean;
  connected: boolean;
  lastAt: number | null;
  messages: number;
};

/**
 * Subscribe to a rover's mission telemetry and its planned route.
 *
 * Staleness is computed against a wall-clock tick, not against arriving data:
 * without a tick, a feed that simply STOPS would keep rendering its last value
 * as current forever — which is precisely the failure this is here to catch.
 */
export function useMission(thing: string | null): MissionFeed {
  const ch = useChannel<any>(thing ? `mission:${thing}` : null);
  const [now, setNow] = useState<number>(() => Date.now());
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, []);

  const mission = readMission(readEnvelope(ch.data));
  const ageMs = ch.lastAt === null ? null : Math.max(0, now - ch.lastAt);
  const stale = ageMs !== null && ageMs > MISSION_STALE_MS;
  return {
    mission,
    ageMs,
    stale,
    blind: !!mission?.running && stale,
    connected: ch.connected,
    lastAt: ch.lastAt,
    messages: ch.messages,
  };
}

/** One waypoint of a planned route, as published on telemetry/mission_plan. */
export type PlanWaypoint = {
  x_mm: number;
  y_mm: number;
  kind?: string;
  dock?: boolean;
};

/**
 * One leg of a planned route — a target the rover drives to and holds at,
 * as opposed to one turn/drive SEGMENT within a leg.
 *
 * Read off `leg_meta` in fpms_missions._preview. `label` is the rover's own
 * plain-English name for the corner; it is shown next to the mission id rather
 * than instead of it, because the two disagree by design.
 */
export type PlanLeg = {
  i: number | null;
  name: string | null;
  label: string | null;
  x_mm: number | null;
  y_mm: number | null;
  finalHeadingDeg: number | null;
  segments: number | null;
  distanceMm: number | null;
};

export type MissionPlan = {
  mission: string | null;
  backend: string | null;
  distanceMm: number | null;
  etaS: number | null;
  segments: number | null;
  returnsHome: boolean;
  poseAssumed: boolean;
  /** Publish time, epoch seconds. Used as the plan's identity for dismissal. */
  ts: number | null;
  waypoints: PlanWaypoint[] | null;
  /** Ordered targets. A single-target mission has one; `patrol` has three. */
  legs: PlanLeg[] | null;
  legsN: number | null;
  /** The executor's own prose description of the route, when it sends one. */
  routeText: string | null;
  /** "retrace" | "none" — how the executor intends to get back. */
  returnStrategy: string | null;
  /**
   * The pose the route was planned FROM. Load-bearing for the follow gate: a
   * plan is only a description of what will happen if the rover is still where
   * it was when the plan was made.
   */
  fromX: number | null;
  fromY: number | null;
  fromHeadingDeg: number | null;
  /** Final target of the route, arena mm. */
  targetX: number | null;
  targetY: number | null;
};

/**
 * The planned route, from a preview. Published once per plan rather than on a
 * heartbeat (hence qos=1 on the bridge subscription), so it is held until a new
 * plan replaces it.
 *
 * Waypoints are filtered, not defaulted: a point missing a coordinate is
 * dropped rather than drawn at the origin, because a route line that detours
 * through the arena corner is worse than a route line with a gap in it.
 */
export function readPlan(env: unknown): MissionPlan | null {
  const d = readEnvelope(env);
  if (!d) return null;
  const raw = d as Record<string, any>;
  const wps = Array.isArray(raw.waypoints)
    ? raw.waypoints
        .filter((w: any) => num(w?.x_mm) !== null && num(w?.y_mm) !== null)
        .map((w: any) => ({
          x_mm: w.x_mm as number,
          y_mm: w.y_mm as number,
          kind: typeof w.kind === "string" ? w.kind : undefined,
          dock: w.dock === true,
        }))
    : null;
  const legs: PlanLeg[] | null = Array.isArray(raw.legs)
    ? raw.legs.map((l: any) => ({
        i: num(l?.i),
        name: text(l?.name),
        label: text(l?.label),
        x_mm: num(l?.x_mm),
        y_mm: num(l?.y_mm),
        finalHeadingDeg: num(l?.final_heading_deg),
        segments: num(l?.segments),
        distanceMm: num(l?.distance_mm),
      }))
    : null;
  const from = raw.from && typeof raw.from === "object" ? raw.from : {};
  const tgt = raw.target && typeof raw.target === "object" ? raw.target : {};
  return {
    mission: text(raw.mission),
    backend: text(raw.backend),
    distanceMm: num(raw.distance_mm),
    etaS: num(raw.eta_s),
    segments: Array.isArray(raw.segments) ? raw.segments.length : null,
    returnsHome: raw.returns_home === true,
    poseAssumed: raw.pose_assumed === true,
    ts: num((env as any)?.ts),
    waypoints: wps && wps.length ? wps : null,
    legs: legs && legs.length ? legs : null,
    legsN: num(raw.legs_n) ?? (legs && legs.length ? legs.length : null),
    routeText: text(raw.route),
    returnStrategy: text(raw.return_strategy),
    fromX: num(from.x_mm),
    fromY: num(from.y_mm),
    fromHeadingDeg: num(from.heading_deg),
    targetX: num(tgt.x_mm),
    targetY: num(tgt.y_mm),
  };
}

/* -------------------------------------------------------------------- arming
 *
 * THE ARM GATE IS THIS DASHBOARD'S, NOT THE ROVER'S.
 *
 * Nothing on the rover has an arm concept: fpms_missions accepts a `mission`
 * from any client that can reach the broker, and fpms_teleop accepts `jog`,
 * `nudge`, `turn` and `test_motors` the same way. There is no interlock in the
 * firmware, none in micro-ROS, and none in the MQTT bridge.
 *
 * So this is a gate on the SENDER. It stops the mission console from putting a
 * motion command on the wire until the operator has said, in a separate and
 * deliberate act, that the arena is clear. That is worth having — every
 * wrong-corner and rover-off-the-bench incident on this project began with a
 * single click — but it must never be described as a safety system on the
 * machine, because a second browser tab, the Drive page or a `mosquitto_pub`
 * will drive this rover while the console says DISARMED.
 *
 * The one control that ignores the gate entirely is ABORT: see
 * MISSION_ABORT_ACTION.
 */
export const MISSION_ARM_NOTE =
  "ARM is enforced by this dashboard, not by the rover. The firmware has no " +
  "arm concept — fpms-missions and fpms-teleop accept commands from any " +
  "client whether or not this console says ARMED. Disarming blocks THIS " +
  "page's motion buttons; it does not immobilise the machine, and it does not " +
  "stop a mission already running. Use ABORT for that.";

/**
 * How long a previewed route stays good enough to follow.
 *
 * A plan is computed from the pose the rover had at preview time. Pose here is
 * dead-reckoned and drifts, and the rover may have been nudged, re-zeroed or
 * driven from another tab in the meantime, so an old plan describes a route
 * from somewhere the rover no longer is. Three minutes is long enough to read
 * the route and walk the arena, short enough that a plan from before a coffee
 * break cannot be followed by accident.
 */
export const PLAN_MAX_AGE_MS = 180_000;

/**
 * How long to wait for a preview before saying it never came back.
 *
 * The executor answers a preview in well under a second when it is running;
 * this timeout exists to distinguish "planning" from "nothing is listening",
 * which are states an operator must never have to guess between.
 */
export const PLAN_WAIT_MS = 8000;

/* ------------------------------------------------------------ encoder read */

/**
 * The answer to `read_encoders`.
 *
 * NAMED FOR WHAT THE BOARD ACTUALLY HAS. The Yahboom MicroROS Board V2.0
 * publishes an integrated pose and a twist on /odom_raw and nothing else — no
 * /wheel_ticks, no joint_states, no per-wheel counters. `ticksAvailable` is
 * therefore false on this rover and `ticksLeft`/`ticksRight` stay null; they
 * are kept in the shape so that a board which DOES publish counts is rendered
 * rather than ignored, and so that a consumer cannot mistake a pose for a tick
 * count. Every numeric field is `number | null` for the usual reason.
 */
export type EncoderReading = {
  ok: boolean | null;
  ticksAvailable: boolean;
  ticksLeft: number | null;
  ticksRight: number | null;
  sourceTopic: string | null;
  note: string | null;
  xM: number | null;
  yM: number | null;
  headingDeg: number | null;
  arenaXmm: number | null;
  arenaYmm: number | null;
  twistLinMps: number | null;
  twistAngRadps: number | null;
  groundSpeedMps: number | null;
  gyroZRadps: number | null;
  yawIntegratedDeg: number | null;
  frames: number | null;
  hz: number | null;
  ageS: number | null;
  stale: boolean | null;
  rosOk: boolean | null;
};

export function readEncoders(data: unknown): EncoderReading | null {
  if (!data || typeof data !== "object") return null;
  const d = data as Record<string, any>;
  const pose = d.pose && typeof d.pose === "object" ? d.pose : {};
  const arena = d.arena_mm && typeof d.arena_mm === "object" ? d.arena_mm : {};
  const twist = d.twist && typeof d.twist === "object" ? d.twist : {};
  const derived = d.derived && typeof d.derived === "object" ? d.derived : {};
  return {
    ok: typeof d.ok === "boolean" ? d.ok : null,
    // Absent means "this reply does not claim ticks exist", which is the same
    // operational fact as false. It is never defaulted to true.
    ticksAvailable: d.ticks_available === true,
    ticksLeft: num(d.ticks_left),
    ticksRight: num(d.ticks_right),
    sourceTopic: text(d.source_topic),
    note: text(d.note),
    xM: num(pose.x_m),
    yM: num(pose.y_m),
    headingDeg: num(pose.heading_deg),
    arenaXmm: num(arena.x_mm),
    arenaYmm: num(arena.y_mm),
    twistLinMps: num(twist.linear_x_mps),
    twistAngRadps: num(twist.angular_z_radps),
    groundSpeedMps: num(derived.ground_speed_mps),
    gyroZRadps: num(derived.gyro_z_radps),
    yawIntegratedDeg: num(derived.yaw_integrated_deg),
    frames: num(d.odom_frames_since_boot),
    hz: num(d.odom_hz),
    ageS: num(d.odom_age_s),
    stale: typeof d.stale === "boolean" ? d.stale : null,
    rosOk: typeof d.ros_ok === "boolean" ? d.ros_ok : null,
  };
}

/* ------------------------------------------------------------ event replies */

/** One event off `fpms/<thing>/events/<subtype>`, normalised. */
export type RoverEvent = {
  subtype: string;
  action: string | null;
  name: string | null;
  preview: boolean;
  accepted: boolean | null;
  error: string | null;
  at: number;
  data: Record<string, unknown>;
};

export type RoverEventFeed = {
  /** Last accepted reply. For a preview, `preview` is true and nothing moved. */
  ack: RoverEvent | null;
  /** Last refusal, carrying the executor's own reason string. */
  nack: RoverEvent | null;
  /** Last events/mission_done — the MEASURED outcome, not the requested one. */
  done: RoverEvent | null;
  /** Last read_encoders reply, already parsed. */
  encoders: { at: number; reading: EncoderReading | null } | null;
};

const EMPTY_EVENTS: RoverEventFeed = {
  ack: null, nack: null, done: null, encoders: null,
};

/**
 * The rover's replies, for one thing.
 *
 * All events share ONE bridge channel ("events", from `fpms/+/events/#`), so a
 * consumer that just reads the channel's latest envelope sees whichever event
 * happened last and loses the one it was waiting for. This keeps the newest of
 * each KIND instead, filtered to the selected rover: a preview refusal stays on
 * screen while telemetry keeps flowing behind it.
 *
 * Messages are consumed by counter rather than by value so a repeated identical
 * reply — press PLAN twice, get the same ack twice — still registers as new.
 */
export function useRoverEvents(thing: string | null): RoverEventFeed {
  const ch = useChannel<any>("events");
  const [feed, setFeed] = useState<RoverEventFeed>(EMPTY_EVENTS);
  const seenRef = useRef(0);

  // A different rover's replies are not this rover's. Clearing on switch stops
  // a stale refusal from being read as the new selection's answer.
  useEffect(() => {
    seenRef.current = 0;
    setFeed(EMPTY_EVENTS);
  }, [thing]);

  useEffect(() => {
    if (!thing) return;
    if (ch.messages === seenRef.current) return;
    seenRef.current = ch.messages;
    const env = ch.data as Record<string, any> | null;
    if (!env || env.thing !== thing) return;
    const subtype = typeof env.subtype === "string" ? env.subtype : "";
    const data = readEnvelope(env) ?? {};
    const d = data as Record<string, any>;
    const ev: RoverEvent = {
      subtype,
      action: text(d.action),
      name: text(d.name),
      preview: d.preview === true,
      accepted: typeof d.accepted === "boolean" ? d.accepted : null,
      error: text(d.error) ?? text(d.reason),
      at: Date.now(),
      data,
    };
    if (subtype === "ack") setFeed((f) => ({ ...f, ack: ev }));
    else if (subtype === "nack") setFeed((f) => ({ ...f, nack: ev }));
    else if (subtype === "mission_done") setFeed((f) => ({ ...f, done: ev }));
    else if (subtype === "encoders") {
      setFeed((f) => ({ ...f, encoders: { at: ev.at, reading: readEncoders(data) } }));
    }
  }, [ch.messages, ch.data, thing]);

  return feed;
}

/**
 * Best available pose for the arena map, as a pose envelope `readPose` accepts.
 *
 * WHY THIS EXISTS. `lib/arena.readPose` reads `x_m`/`y_m` off the rover-agent's
 * heartbeat, and that heartbeat carries no position at all — so every map on
 * this dashboard drew the rover at the assumed start corner even while the
 * mission executor and the teleop bridge were both publishing a real,
 * dead-reckoned pose in millimetres. The map was ignoring the only pose the
 * rover actually has.
 *
 * Returns null when NOTHING has a position, and the caller then falls back to
 * the heartbeat envelope so `readPose` can raise its own SIMULATED badge. That
 * fallback is load-bearing: this function must never invent a coordinate, or
 * the badge that says "this position is assumed" would stop appearing while the
 * position was still assumed.
 *
 * The mission executor wins over teleop when both have one. They read the same
 * odometry, but the executor is the process actually driving, and a map that
 * disagreed with the mission card would leave the operator with two positions
 * and no way to tell which one the rover was steering by.
 */
export function poseEnvelopeFromMm(
  x_mm: number | null,
  y_mm: number | null,
  heading_deg: number | null,
): { data: { x_m: number; y_m: number; heading_deg?: number } } | null {
  if (x_mm === null || y_mm === null) return null;
  return {
    data: {
      x_m: x_mm / 1000,
      y_m: y_mm / 1000,
      ...(heading_deg === null ? {} : { heading_deg }),
    },
  };
}

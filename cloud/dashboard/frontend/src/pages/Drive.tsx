import { useEffect, useMemo, useRef, useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { StatusPill } from "../components/StatusPill";
import ErrorBoundary from "../components/ErrorBoundary";
import Joystick, { type StickValue } from "../components/Joystick";
import { ArenaMap } from "../components/ArenaMap";
import { useChannel } from "../lib/ws";
import { apiPostJson } from "../lib/api";
import { useThings } from "../lib/things";
// Mission normalisation lives in lib/mission so that Control and LiDAR inherit
// the same three rules — missing is not zero, an unknown phase is RUNNING, and
// stale-while-running is loud — rather than each reimplementing them.
import {
  MISSION_ABORT_ACTION,
  MISSION_ABORT_NOTE,
  MISSION_STALE_MS,
  poseEnvelopeFromMm,
  readMission,
  readPlan,
  type MissionPlan,
  type MissionState,
} from "../lib/mission";
// The mission names, their corner labels and the backends all come from what
// the executor announced, not from a copy kept here that can drift away from it.
import {
  emptyCapabilities,
  missionCatalog,
  useRoverCapabilities,
  type MissionInfo,
} from "../lib/capabilities";
// Arena geometry is READ, never restated: ARENA_MM is the single source of
// truth for the pose sanity check below.
import { ARENA_MM } from "../lib/arena";

/**
 * Manual driving. Control.tsx is the command console — discrete actions and
 * their acks. This page is the other half: a stick, closed-loop turns, the
 * missions, and a health panel that says whether any of it can work.
 *
 * Three constraints shape everything here.
 *
 * The rover runs a 0.6 s deadman: if jog messages stop arriving it halts by
 * itself. That is a safety net, not a control scheme — so while the stick is
 * held we send continuously at 10 Hz, including when it is held perfectly
 * still, and on release we send an explicit zero and a stop rather than letting
 * the timeout do it.
 *
 * Everything is slow on purpose. The first test drive was far too fast. The
 * rover's limits are printed on the page rather than left to be guessed at.
 *
 * And when we cannot say what the rover is doing — ROS reported down, telemetry
 * gone stale, or nothing ever heard from this bay at all — the motion controls
 * say so, naming which of the three it is, instead of accepting clicks that go
 * nowhere. STOP ALL is the exception: it
 * stays live no matter what, because the moment everything else is broken is
 * exactly when a stop has to still be reachable.
 */

/** Jog send rate. Comfortably inside the rover's deadman window. */
const JOG_HZ = 10;
/** The rover's own failsafe, quoted in the UI copy. */
const DEADMAN_MS = 600;

/** Nudge step. Small and bounded, which is why nudges are not confirm-gated. */
const NUDGE_MM = 100;
/** Closed-loop turn magnitudes offered, smallest first. */
const TURN_DEGS = [15, 45, 90] as const;

/**
 * Speed limits, printed so the operator is not guessing.
 *
 * These are DEFAULTS, and they are labelled as such when that is all we have.
 * `set_speed` on the Control tab retunes jog and nudge at runtime, and the
 * rover then reports the values it is actually using on every telemetry tick
 * (`jog_max_mps`, `nudge_mps`) — so a card that printed these constants
 * unconditionally would go on claiming 0.05 m/s after an operator had changed
 * it, which is precisely the drift this dashboard is supposed to catch rather
 * than commit.
 */
const CAP_DEFAULTS = {
  jogMaxMps: 0.05,
  nudgeMps: 0.04,
  dockMps: 0.025,
  turnRadps: 0.4,
} as const;

/**
 * Fallback mission ids, used ONLY until the executor announces its own.
 *
 * fpms_missions publishes `missions`, `single_targets`, `targets` (each with
 * the rover's own corner label) and `routes` on events/online. Those win
 * outright once heard. This list exists because that message is published once
 * and not retained, so a dashboard opened after the rover booted has heard
 * nothing — and offering no missions at all then would be reading silence as
 * "this rover cannot drive anywhere".
 */
const FALLBACK_MISSIONS = ["home", "m1", "m2", "water", "patrol"] as const;

/**
 * Labels of last resort, for the same window before the rover has spoken.
 *
 * `where` names the physical CORNER, because the ids do not line up with what
 * an operator says out loud: standing at the start box, the zone straight
 * ahead is `m2` and the far one is `m1`. When the rover's own label arrives it
 * replaces this — fpms_missions.TARGET_LABEL is the single authority and this
 * is a stale copy of it by construction.
 */
const FALLBACK_LABELS: Record<string, { label: string; where: string; primary?: boolean }> = {
  home: { label: "RETURN HOME", where: "start box, bottom-RIGHT", primary: true },
  m1: { label: "MISSION 1", where: "top-LEFT zone (zone-a, the far one)" },
  m2: { label: "MISSION 2", where: "top-RIGHT zone (zone-b, straight ahead of the start box)" },
  water: { label: "WATER REFILL", where: "bottom-LEFT water station" },
  patrol: {
    label: "ALL TARGETS",
    where:
      "top-RIGHT zone, then top-LEFT zone, then the water station, then retraces the whole path back to the start box",
  },
};

/** One mission button, however we came to know about it. */
type MissionOption = {
  name: string;
  label: string;
  where: string;
  primary: boolean;
  /** True when the label came from the rover rather than from FALLBACK_LABELS. */
  fromRover: boolean;
};

function buildMissionOptions(
  names: readonly string[],
  info: Readonly<Record<string, MissionInfo>>,
): MissionOption[] {
  return names.map((name) => {
    const ann = info[name];
    const fb = FALLBACK_LABELS[name];
    // A route's `describe` names every corner in order and is the better
    // tooltip; a single target has only its corner label.
    const where = ann?.describe ?? ann?.label ?? fb?.where ?? "target not described by the rover";
    return {
      name,
      // The rover's route label ("ALL TARGETS") is operator-facing already.
      // Single targets announce a corner description rather than a button
      // caption, so the local caption is kept for those and the corner goes in
      // the tooltip where it belongs.
      label: fb?.label ?? (ann?.legs ? ann.label ?? name.toUpperCase() : name.toUpperCase()),
      where,
      primary: fb?.primary === true || name === "home",
      fromRover: !!ann,
    };
  });
}

/** Telemetry older than this means we no longer know what the rover is doing. */
const TELEMETRY_STALE_MS = 5000;
/** Battery reading older than this is reported as stale rather than current. */
const BATTERY_STALE_S = 5;

/**
 * Link banner thresholds. Deliberately separate from TELEMETRY_STALE_MS, which
 * governs the motion lockout — the lockout is a safety decision and the banner
 * is a status readout, and pinning them together would mean a copy change to one
 * silently altering the other.
 *
 * Under LINK_LIVE_MS the feed is current. Between the two the feed has gone
 * quiet but a single dropped packet at 1 Hz still looks like this, so it reads
 * STALE. Past LINK_LOST_MS it is not a hiccup any more: the rover has stopped
 * publishing and we are waiting for it to come back, which is RECONNECTING.
 */
const LINK_LIVE_MS = 3000;
const LINK_LOST_MS = 10000;

/**
 * uptime_s must fall by more than this to count as a restart. Telemetry is
 * sampled, not synchronised, so a packet can carry an uptime a shade below its
 * predecessor without anything having happened.
 */
const REBOOT_SLACK_S = 2;

const ACK_TIMEOUT_MS = 3000;
const LOG_LIMIT = 40;

/* ---- front clearance ------------------------------------------------------
 *
 * THE NUMBER THAT PREDICTED THE LAST COLLISION.
 *
 * A mission ran at full speed, outran its own 11 Hz odometry and struck an
 * obstacle. Front clearance fell 989 mm -> 117 mm on the way in. It was visible
 * the whole time — as one cell in a nine-up grid of mission stats, below the
 * fold, in the same weight as "elapsed". It is now a full-width readout above
 * the controls, coloured by state, that shouts when it closes.
 *
 * The value is `front_mm` from fpms_missions.snapshot(). It is `null` when the
 * LiDAR is blind, and null renders "--" — never 0, which would read as "the
 * bumper is against something".
 */

/**
 * Fallback stop threshold, millimetres — `FRONT_STOP_MM` in fpms_missions.py.
 *
 * Used only until the executor announces `limits.front_stop_mm` on
 * events/online. That message is published once and not retained, so a
 * dashboard opened after the rover booted has never seen it, and refusing to
 * colour the readout for the rest of the session would be worse than colouring
 * it against the value the rover ships with. Which of the two is in play is
 * always printed.
 */
const FRONT_STOP_FALLBACK_MM = 120;

/**
 * How much headroom above the stop threshold still counts as CLOSING.
 *
 * A presentation constant, not a rover setting, and deliberately generous — the
 * point is that the readout changes character on the way down rather than at
 * the bottom.
 */
const CLOSING_FACTOR = 3;

/* ---- pose sanity ----------------------------------------------------------
 *
 * A reported position outside the arena is a BROKEN ESTIMATE, not a position.
 *
 * A run reported (625, -2720) mm while the rover had physically moved under a
 * metre inside a 1200 mm arena, and an earlier one reported (-9182, -11926).
 * Both were plotted. A glyph off the edge of the canvas reads as a rendering
 * fault; what it actually meant was that the dead-reckoning integration had come
 * apart and every distance, ETA and arrival test computed from it was wrong.
 *
 * The bound is ARENA_MM from lib/arena plus one rover length of slack, so a
 * machine legitimately straddling the line at the start box is not called
 * broken. Nothing is clamped or corrected — the number is printed in full and
 * the map is handed the heartbeat instead, so it draws the assumed start and
 * raises its own badge.
 *
 * NOTE: an identical guard exists in Lidar.tsx. It is duplicated rather than
 * shared because both pages own their own file and lib/arena is not this
 * change's to edit; if a third consumer needs it, it belongs in lib/arena
 * beside ARENA_MM.
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

/**
 * Which executor drives a mission.
 *
 * deadreckon is the default because it is the only one that has ever worked:
 * turn-then-drive segments with dead-reckoned localisation, measured at 0.6%
 * distance error and ±1–4° on turns. nav2 is offered but not defaulted — the
 * navigation stack needs a live scan topic in ROS, a complete TF tree and a
 * map, and on this rover the board's /scan is dead and the real LiDAR
 * publishes to MQTT only. Selecting it before that chain is up sends a
 * mission to a planner that cannot localise.
 */
const BACKEND_NOTES: Record<string, { label: string; note: string; warn: string | null }> = {
  deadreckon: {
    label: "dead reckoning",
    note: "proven — 0.6% distance error, ±1–4° turns",
    warn: null,
  },
  nav2: {
    label: "Nav2",
    note: "requires the navigation stack to be up",
    warn:
      "Nav2 needs a live LaserScan in ROS, a complete TF tree and a map before it can localise. Those are not up on this rover yet — the board's /scan publishes all zeros and the working LiDAR goes to MQTT, not ROS. A mission sent on this backend will not plan.",
  },
};

/** Used until the executor announces `backends` / `default_backend`. */
const FALLBACK_BACKENDS = ["deadreckon", "nav2"] as const;
const FALLBACK_DEFAULT_BACKEND = "deadreckon";

type Action = "stop" | "jog" | "nudge" | "turn" | "mission" | "set_coordinate";

type LogKind = "sent" | "ack" | "nack" | "stale" | "nohw" | "timeout" | "error" | "event";

type LogItem = {
  id: string;
  at: number;
  thing: string;
  action?: string;
  kind: LogKind;
  text: string;
  hint?: string;
};

export default function Drive() {
  const things = useThings();
  // Both bays are always offered so a rover that has not reported yet is
  // visibly present rather than missing from the selector (as in Control).
  const bays = useMemo(
    () => Array.from(new Set([...things, "rover1", "rover2"])).sort().slice(0, 4),
    [things.join(",")], // eslint-disable-line react-hooks/exhaustive-deps
  );

  // Driving targets exactly one rover — a stick that moves two machines at once
  // is not a control, it is an accident waiting for an audience.
  const [picked, setPicked] = useState<string | null>(null);
  const thing = picked && bays.includes(picked) ? picked : things[0] ?? bays[0] ?? null;

  const [log, setLog] = useState<LogItem[]>([]);
  const [seen, setSeen] = useState<Record<string, { lastAt: number; count: number }>>({});
  const [stickActive, setStickActive] = useState(false);
  const [jogFails, setJogFails] = useState(0);
  // null until the stick has actually sent something. It used to start at
  // {0,0}, so the readout said "sent · vx 0.00 · wz 0.00" on a page that had
  // never sent a command in its life — a zero standing in for "nothing".
  const [lastCmd, setLastCmd] = useState<StickValue | null>(null);

  const ev = useChannel<any>("events");
  const drive = useChannel<any>(thing ? `drive:${thing}` : null);
  const tele = readTelemetry(drive.data);

  // Mission progress, published by the mission executor on
  // fpms/<thing>/telemetry/mission. The executor may not be running at all,
  // so every consumer below treats "no data" as a first-class state rather
  // than as zeroes.
  const missionCh = useChannel<any>(thing ? `mission:${thing}` : null);

  // The planned route, from a preview. Published once per plan rather than on a
  // heartbeat, so it is held until a new plan replaces it.
  // Same channel the LiDAR page's map reads, so both views place the rover in
  // exactly the same spot. Two pose sources would eventually disagree, and the
  // operator would have no way to tell which map was lying.
  const poseCh = useChannel<any>(thing ? `pose:${thing}` : null);

  const planCh = useChannel<any>(thing ? `mission_plan:${thing}` : null);
  const planRaw = readPlan(planCh.data);

  // useChannel has no clear(), so dismissal is local: remember which plan was
  // dismissed by its timestamp. Keyed on the plan itself rather than a boolean
  // so the NEXT preview reappears automatically instead of staying hidden
  // behind a flag the operator has forgotten they set.
  const [dismissedPlanTs, setDismissedPlanTs] = useState<number | null>(null);
  const planTs = planRaw?.ts ?? null;
  const plan: MissionPlan | null =
    planTs !== null && planTs === dismissedPlanTs ? null : planRaw;
  const planRoute = plan?.waypoints ?? null;

  const missionRaw = readTelemetry(missionCh.data);
  const mission = readMission(missionRaw);

  /**
   * ARM STATE, READ OFF THE RAW SNAPSHOT.
   *
   * fpms_missions.snapshot() publishes `armed` (bool, or null when the runner is
   * not up) and `arm_required` (bool, the executor's REQUIRE_ARM). lib/mission's
   * MissionState does not carry them, so they are read here directly rather than
   * inferred — inferring an interlock is exactly the class of thing that must
   * not be guessed.
   *
   * THE ASYMMETRY IS DELIBERATE. Motion is locked out only on POSITIVE evidence
   * that arming is required and has not happened. "Nothing has published an arm
   * state" is NOT treated as unarmed, because the arm concept lives entirely in
   * the mission executor: a rover with fpms-missions stopped would otherwise
   * have its joystick locked out forever by a service that is not running and
   * was never needed for manual teleop. That case gets a named chip instead of a
   * lockout, so the operator can see the difference.
   */
  const armed = typeof missionRaw?.armed === "boolean" ? missionRaw.armed : null;
  const armRequired =
    typeof missionRaw?.arm_required === "boolean" ? missionRaw.arm_required : null;
  const notArmed = armRequired === true && armed !== true;

  /**
   * What THIS rover says it can drive.
   *
   * The mission buttons, their corner labels and the backend chips are all
   * built from this. Before it arrives the fallbacks above are used whole and
   * the card says so — an unheard announcement must never be rendered as "this
   * rover has no missions", because `events/online` is published once and is
   * not retained, so a dashboard opened after the executor started has heard
   * nothing at all.
   */
  const capsByThing = useRoverCapabilities();
  const caps = thing ? capsByThing[thing] ?? emptyCapabilities() : emptyCapabilities();
  const catalog = useMemo(
    () => missionCatalog(caps, FALLBACK_MISSIONS),
    [caps],
  );
  const missionOptions = useMemo(
    () => buildMissionOptions(catalog.names, catalog.info),
    [catalog],
  );

  const backends: readonly string[] =
    caps.backends && caps.backends.length ? caps.backends : FALLBACK_BACKENDS;
  const [pickedBackend, setPickedBackend] = useState<string | null>(null);
  // The rover's own default wins over ours until the operator chooses. It is
  // configurable on the rover (FPMS_MISSION_BACKEND), so hardcoding
  // "deadreckon" here would silently send missions on a backend the rover was
  // deliberately configured away from.
  const backend =
    pickedBackend && backends.includes(pickedBackend)
      ? pickedBackend
      : caps.defaultBackend && backends.includes(caps.defaultBackend)
        ? caps.defaultBackend
        : backends[0] ?? FALLBACK_DEFAULT_BACKEND;
  const setBackend = setPickedBackend;

  /**
   * Where the arena map draws the rover.
   *
   * This page was passing the rover-agent heartbeat straight through, and that
   * heartbeat carries no position at all — so the map on the DRIVE tab, the one
   * an operator uses to decide whether a mission is going the right way, drew
   * the rover parked in the start corner no matter where it actually was. Both
   * the mission executor and the teleop bridge publish a real dead-reckoned
   * pose in millimetres; the executor wins when it has one, because it is the
   * process doing the driving and a map that disagreed with the mission card
   * would leave two positions on screen and no way to tell which the rover was
   * steering by.
   *
   * When nothing has a position we fall back to the heartbeat envelope, exactly
   * as before, so `readPose` raises its own SIMULATED badge. That fallback is
   * the point: this must never manufacture a coordinate, or the badge saying
   * the position is assumed would disappear while it still was.
   */
  const execX = mission?.poseX ?? null;
  const execY = mission?.poseY ?? null;
  const bridgeX = num(tele?.x_mm);
  const bridgeY = num(tele?.y_mm);
  const rawPoseX = execX ?? bridgeX;
  const rawPoseY = execY ?? bridgeY;

  /**
   * OUT-OF-ARENA IS REFUSED, not plotted. See poseOutOfArena above. The refused
   * case takes the same path as "no position at all" — operationally they are
   * the same statement, nobody knows where this rover is — and differs only in
   * that this one is a fault and gets said loudly.
   */
  const poseBroken = poseOutOfArena(rawPoseX, rawPoseY);
  const brokenPoseSource = execX !== null ? "mission executor" : "teleop bridge";

  const mapPose = poseBroken
    ? null
    : poseEnvelopeFromMm(execX, execY, mission?.poseHeadingDeg ?? null) ??
      poseEnvelopeFromMm(bridgeX, bridgeY, num(tele?.heading_deg));
  const mapPoseSource = mapPose === null ? null : execX !== null ? "mission executor" : "teleop bridge";

  // Staleness is a function of wall time, not of arriving data — without a tick
  // a feed that simply stops would keep rendering its last value as current
  // forever, which is the failure mode this whole panel exists to catch.
  const [now, setNow] = useState<number>(() => Date.now());
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, []);

  /**
   * Link history for the selected rover. drive.lastAt alone cannot answer the
   * question the operator actually has when the rover goes quiet — "has this
   * thing ever been here, or did it drop?" — so we keep the session ourselves:
   * when the first packet landed, when the last one did, how many gaps it has
   * come back from, and whether uptime_s ever went backwards.
   *
   * Reset per rover on purpose. rover1 having been live all afternoon says
   * nothing whatsoever about rover2, and carrying the history across the
   * selector would make an empty bay look like a healthy one.
   */
  const [link, setLink] = useState<LinkSession>(newSession);
  useEffect(() => {
    setLink(newSession());
  }, [thing]);
  useEffect(() => {
    if (drive.lastAt === null) return;
    const uptime = num(tele?.uptime_s);
    setLink((s) => advanceSession(s, uptime, Date.now()));
  }, [drive.messages]); // eslint-disable-line react-hooks/exhaustive-deps

  const linkState = linkPhase(link, now);

  const teleAgeMs = drive.lastAt === null ? null : now - drive.lastAt;
  const teleStale = teleAgeMs !== null && teleAgeMs > TELEMETRY_STALE_MS;
  const rosDown = tele?.ros_ok === false;
  /** Nothing has EVER arrived on this rover's drive channel this session. */
  const neverSeen = drive.lastAt === null;
  /**
   * Motion is blocked whenever we cannot say what the rover is doing, and that
   * includes never having heard from it at all.
   *
   * "Never received anything" used to be deliberately excluded, on the argument
   * that a deployment without the drive telemetry topic plumbed through would
   * be left with a page that cannot drive. That trade was the wrong way round.
   * The banner directly above these controls already reads "OFFLINE - never
   * seen this session" while the joystick, the turns, the nudges, all five
   * mission buttons and Set coordinate sat enabled underneath it: a page that
   * contradicts itself, and a joystick that streams jog at 10 Hz into nothing.
   * A control that looks armed and does nothing teaches an operator that the
   * rover is broken - or worse, banks a command that lands the moment the link
   * comes back. An unheard rover is exactly the case where motion must be
   * refused, with the reason printed.
   *
   * All three lockout causes are named separately below. "Locked" alone sends
   * the operator hunting; "never seen", "ROS down" and "telemetry stale" each
   * point at a different thing to go and check.
   */
  const motionLocked = rosDown || teleStale || neverSeen;

  type LockKind = "ros" | "stale" | "never";
  // Ordered by which fact is most load-bearing when several are true at once: a
  // rover that has never been seen is not usefully described as "stale".
  const lockKind: LockKind | null = neverSeen
    ? "never"
    : rosDown
      ? "ros"
      : teleStale
        ? "stale"
        : null;

  const LOCK_TITLE: Record<LockKind, string> = {
    ros: "ROS DOWN — the rover cannot move",
    stale: "LINK STALE — rover state unknown",
    never: "NEVER SEEN — this rover has not reported at all",
  };
  const lockReason =
    lockKind === "ros"
      ? "The rover reports ROS is down. It cannot act on a motion command."
      : lockKind === "stale"
        ? `No drive telemetry for ${Math.round((teleAgeMs ?? 0) / 1000)}s — the rover's state is unknown.`
        : lockKind === "never"
          ? `Nothing has ever arrived on ${thing ? `drive:${thing}` : "this channel"} since the page loaded, so this dashboard has no idea whether ${thing ?? "the rover"} is powered, where it is, or whether anything it is sent would be acted on.`
          : null;

  /**
   * THE ARM LOCKOUT IS SCOPED TO WHAT ARMING ACTUALLY GATES.
   *
   * fpms_missions checks `armed()` in exactly one place — the run path of the
   * `mission` verb — and refuses with "rover is not armed". fpms_teleop has no
   * arm concept at all: jog, nudge, turn and test_motors are accepted whether or
   * not the executor is armed.
   *
   * So an unarmed rover locks out the MISSION buttons, with the rover's own
   * refusal as the reason, and does NOT lock out the stick. Locking the stick on
   * it would be this dashboard inventing an interlock the machine does not have
   * and attributing it to the rover — and, since REQUIRE_ARM ships enabled and
   * an arm expires after 120 s unused, it would leave manual driving dead
   * whenever fpms-missions happened to be running. Both facts are named
   * separately on screen rather than merged into one "locked".
   */
  const missionLocked = motionLocked || notArmed;

  // The hub replays its last broadcast to each new subscriber, so without this
  // an ack from minutes ago would appear on mount as if it had just landed.
  const mountedAt = useRef(Date.now() / 1000);
  const pending = useRef(new Map<string, { id: string; timer: number }>());

  const pushLog = (item: LogItem) => setLog((l) => [item, ...l].slice(0, LOG_LIMIT));
  const patchLog = (id: string, patch: Partial<LogItem>) =>
    setLog((l) => l.map((e) => (e.id === id ? { ...e, ...patch } : e)));

  /** Logged, ack-tracked command. Everything except the jog stream. */
  const fire = (action: Action, to: string[], params: Record<string, unknown> = {}) => {
    for (const target of to) {
      const key = `${target}:${action}`;
      const id = `${key}:${Date.now()}:${Math.random().toString(36).slice(2, 7)}`;

      const prev = pending.current.get(key);
      if (prev) window.clearTimeout(prev.timer);

      const timer = window.setTimeout(() => {
        if (pending.current.get(key)?.id !== id) return;
        pending.current.delete(key);
        patchLog(id, { kind: "timeout", text: "no reply" });
      }, ACK_TIMEOUT_MS);
      pending.current.set(key, { id, timer });

      pushLog({
        id,
        at: Date.now(),
        thing: target,
        action,
        kind: "sent",
        text: describeParams(params),
      });

      apiPostJson<unknown>(`/api/control/${target}/${action}`, { params }).catch((e: unknown) => {
        const p = pending.current.get(key);
        if (p?.id === id) {
          window.clearTimeout(p.timer);
          pending.current.delete(key);
        }
        patchLog(id, { kind: "error", text: `not sent — ${errText(e)}` });
      });
    }
  };

  /** Fire a motion command at the selected rover, if there is one and it can move. */
  const move = (action: Action, params: Record<string, unknown> = {}) => {
    if (!thing || motionLocked) return;
    fire(action, [thing], params);
  };

  /**
   * Ask the executor what route it WOULD drive. Commands no motion.
   *
   * Deliberately not gated on `motionLocked`. Every other control on this page
   * is, and correctly so — but a preview moves nothing, and the moment an
   * operator most wants to see the intended route is exactly when the rover is
   * locked out and they are working out whether it is safe to release it.
   * Withholding the map then would be backwards.
   */
  const planMission = (name: string) => {
    if (!thing) return;
    fire("mission", [thing], { name, backend, preview: true });
  };

  /**
   * The jog stream. Deliberately not logged and not ack-tracked: at 10 Hz it
   * would bury every real acknowledgement within a second of the first nudge of
   * the stick. Failures surface as a counter instead, which is the thing that
   * matters — one dropped jog is nothing, a run of them means the link is gone
   * and the rover is about to deadman.
   */
  const jogRef = useRef<string | null>(thing);
  jogRef.current = thing;
  const sendJog = (v: StickValue) => {
    const target = jogRef.current;
    if (!target) return;
    const vx = unit(v.vx);
    const wz = unit(v.wz);
    setLastCmd({ vx, wz });
    apiPostJson<unknown>(`/api/control/${target}/jog`, { params: { vx, wz } })
      .then(() => setJogFails(0))
      .catch(() => setJogFails((n) => Math.min(n + 1, 99)));
  };

  /**
   * Release. The stick emits its own {0,0} the moment it is let go; this adds a
   * second one and an explicit stop, because a single dropped packet at exactly
   * this moment is the one that leaves a machine rolling for 0.6 s. Redundant
   * on purpose — a duplicate stop costs nothing.
   */
  const onStickActive = (a: boolean) => {
    setStickActive(a);
    if (a) return;
    const target = jogRef.current;
    if (!target) return;
    sendJog({ vx: 0, wz: 0 });
    fire("stop", [target]);
  };

  /**
   * Abort whatever mission is running on `targets`, without touching the
   * motion lockout — an abort has to work precisely when the link check has
   * decided things are wrong, so it goes through `fire` and never `move`.
   *
   * This used to publish `mission {name:"abort"}`, which BOTH publishers
   * refuse: `abort` is not in fpms_missions.COMMANDABLE and not in teleop's
   * stub list either, so the one control that exists for a machine driving
   * somewhere wrong answered with a nack and aborted nothing. `stop` is the
   * verb the executor actually subscribes and acts on (see MISSION_ABORT_NOTE
   * and the rover's own `acts_silently_on` announcement), and it halts the
   * motors in the same publish.
   */
  const abortMission = (targets: string[]) => fire(MISSION_ABORT_ACTION, targets);

  /**
   * Fleet emergency stop. Ignores the rover selector on purpose: an e-stop that
   * only halts the rover you happen to have selected is a trap. Never disabled
   * and never confirm-gated — a dialog on an e-stop costs a click during an
   * emergency, and an accidental stop is the safe outcome.
   *
   * One publish per rover does both jobs. `stop` halts the motors in teleop
   * AND aborts the run in fpms-missions, which subscribes the same verb and
   * calls request_abort on it. The earlier version sent a second
   * `mission {name:"abort"}` alongside — that name is not commandable, so all
   * it ever added was a refusal in the log at the worst possible moment.
   */
  const stopAll = () => fire(MISSION_ABORT_ACTION, bays);
  const stopRef = useRef(stopAll);
  stopRef.current = stopAll;
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") stopRef.current();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  // Inbound rover traffic — acks for our commands, and anything else worth
  // seeing. useChannel keeps only the newest envelope, so the message counter
  // is what tells us a fresh one arrived.
  useEffect(() => {
    const env = ev.data;
    if (!env || typeof env !== "object") return;
    const ts = normalizeTs(env.ts);
    if (ts !== null && ts < mountedAt.current) return; // replayed, not new

    const from = String(env.thing ?? "?");
    const subtype = String(env.subtype ?? "");
    const data = (env.data ?? {}) as Record<string, any>;
    const action = typeof data.action === "string" ? data.action : undefined;

    setSeen((s) => ({
      ...s,
      [from]: { lastAt: Date.now(), count: (s[from]?.count ?? 0) + 1 },
    }));

    // Jog acks are left out for the same reason jog sends are.
    if (action === "jog") return;

    const key = action ? `${from}:${action}` : null;
    const p = key ? pending.current.get(key) : undefined;

    if ((subtype === "ack" || subtype === "nack") && key && p) {
      window.clearTimeout(p.timer);
      pending.current.delete(key);
      patchLog(p.id, subtype === "ack" ? describeAck(data) : describeNack(data));
      return;
    }

    pushLog({
      id: `${from}:${subtype}:${Date.now()}:${Math.random().toString(36).slice(2, 7)}`,
      at: Date.now(),
      thing: from,
      action,
      ...(subtype === "nack"
        ? describeNack(data)
        : { kind: "event" as LogKind, text: subtype || "event" }),
    });
  }, [ev.messages]); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    const map = pending.current;
    return () => {
      map.forEach((p) => window.clearTimeout(p.timer));
      map.clear();
    };
  }, []);

  const noRover = !thing;
  const motionDisabled = noRover || motionLocked;
  // Every disabled control says WHICH of the three states put it there. A
  // greyed-out button with no reason is indistinguishable from a broken page.
  const disabledHint = noRover
    ? "no rover selected"
    : lockKind === "ros"
      ? "ROS is down on the rover"
      : lockKind === "stale"
        ? "telemetry stale — link unknown"
        : lockKind === "never"
          ? `${thing} has never reported this session — nothing to drive`
          : "";

  // Mission buttons carry the arm gate on top of the link gate, and name which
  // of the two stopped them.
  const missionDisabled = noRover || missionLocked;
  const missionDisabledHint = noRover
    ? "no rover selected"
    : motionLocked
      ? disabledHint
      : notArmed
        ? `${thing} is not armed — fpms-missions refuses every mission with "rover is not armed" until it is`
        : "";

  return (
    <ErrorBoundary label="Drive">
      <div className="space-y-5">
        <div className="flex items-end justify-between gap-4">
          <div>
            <div className="lbl">Drive · manual piloting</div>
            <h1 className="h-page mt-1">Stick, turns, missions and health</h1>
          </div>
          <div className="text-xs text-slate-500">
            {things.length
              ? `${things.length} rover${things.length > 1 ? "s" : ""} reporting: ${things.join(", ")}`
              : "no rovers reporting"}
          </div>
        </div>

        {/* Sticky so it never scrolls out of reach; never disabled, because a
            second stop costs nothing and greying it out removes the control at
            exactly the moment the first one might have been the one that failed.
            It stays live even when the link check has locked everything else. */}
        <div className="sticky top-[86px] z-20 md:top-[58px]">
          <button
            onClick={stopAll}
            className="flex w-full items-center justify-center gap-3 rounded-xl border border-rose-500/50 bg-rose-600/25 px-4 py-5 text-lg font-semibold tracking-wide text-rose-50 shadow-lg shadow-black/50 backdrop-blur transition hover:bg-rose-600/40 active:scale-[0.995]"
            title="Emergency stop — halts every rover AND aborts any running mission (fpms-missions subscribes `stop` and aborts on it). Always enabled. Shortcut: Esc"
          >
            <span className="inline-block h-3 w-3 rounded-full bg-rose-400 pulse-dot text-rose-400" />
            STOP ALL — EVERY ROVER
            <span className="rounded border border-rose-300/30 bg-black/30 px-1.5 py-0.5 font-mono text-[11px] font-normal text-rose-200">
              Esc
            </span>
          </button>
        </div>

        {/* Link state, directly under the stop bar. The operator is usually
            standing next to the rover with the laptop in the other hand; this
            has to answer "is it back yet?" from across the room. */}
        <ErrorBoundary label="Drive link">
          <LinkBanner
            thing={thing}
            phase={linkState.phase}
            ageMs={linkState.ageMs}
            session={link}
            socketConnected={drive.connected}
            uptime={tele?.uptime_s}
            now={now}
          />
        </ErrorBoundary>

        {/* Loud, unmissable, and above the controls it explains. A greyed-out
            button with no reason next to it sends the operator hunting the
            dashboard for a bug that is on the far end of the link. */}
        {lockReason && lockKind && (
          <div className="flex items-start gap-3 rounded-xl border-2 border-rose-500/50 bg-rose-950/40 p-4">
            <span className="mt-0.5 inline-block h-3 w-3 shrink-0 rounded-full bg-rose-400 pulse-dot text-rose-400" />
            <div>
              <div className="text-base font-semibold text-rose-100">
                {LOCK_TITLE[lockKind]}
              </div>
              <p className="mt-1 text-sm text-rose-200/85">
                {lockReason}{" "}
                {lockKind === "never"
                  ? "Motion controls are disabled until it publishes drive telemetry — they will enable themselves the moment it does, with no reload. Check that the teleop bridge is running on the rover, that the rover is powered, and that the right bay is selected above."
                  : "Motion controls are disabled until it recovers."}{" "}
                <b>STOP ALL still works</b> and is still worth pressing.
              </p>
            </div>
          </div>
        )}

        {/* A REPORTED POSITION THAT IS NOT A POSITION. Above everything it
            affects, because it invalidates all of it: the map, the mission
            card's coordinates, the planned route and the distance remaining are
            all computed from this number. */}
        {poseBroken && (
          <div className="flex items-start gap-3 rounded-xl border-2 border-rose-500/60 bg-rose-950/50 p-4">
            <span className="mt-1 inline-block h-3 w-3 shrink-0 rounded-full bg-rose-400 pulse-dot text-rose-400" />
            <div>
              <div className="text-base font-semibold text-rose-100">
                POSE ESTIMATE BROKEN — reported position is outside the arena
              </div>
              <p className="mt-1 max-w-3xl text-sm text-rose-200/90">
                The {brokenPoseSource} reports{" "}
                <span className="font-mono">
                  ({rawPoseX === null ? "--" : Math.round(rawPoseX)},{" "}
                  {rawPoseY === null ? "--" : Math.round(rawPoseY)}) mm
                </span>
                , which is outside the {ARENA_MM} × {ARENA_MM} mm arena. That is
                not a position — it is a dead-reckoning integration that has come
                apart. It is <b>not plotted</b>: the map below has fallen back to
                the assumed start corner and says so. Every distance, ETA and
                arrival test on this page is computed from the same broken value.
                Stop the rover and use <b>Set coordinate</b> below before running
                anything.
              </p>
            </div>
          </div>
        )}

        {/* FRONT CLEARANCE, IN THE EYE-LINE. This is the number that fell
            989 mm -> 117 mm during the run that ended in a collision, while
            being rendered as one cell of a nine-up stat grid below the fold. */}
        <ErrorBoundary label="Drive clearance">
          <ClearanceBar
            mm={mission?.frontMm ?? null}
            stopMm={num(caps.limits.front_stop_mm) ?? FRONT_STOP_FALLBACK_MM}
            stopFromRover={num(caps.limits.front_stop_mm) !== null}
            running={!!mission?.running}
            lidarOk={mission?.lidarOk ?? null}
            hasExecutor={missionCh.messages > 0}
          />
        </ErrorBoundary>

        {/* NOT ARMED — a named lockout in its own right, kept apart from the
            link lockout because it stops a different set of controls. It is the
            rover's own refusal, quoted, not a gate this dashboard invented. */}
        {notArmed && !motionLocked && thing && (
          <div className="flex items-start gap-3 rounded-xl border-2 border-amber-500/50 bg-amber-500/10 p-4">
            <span className="mt-0.5 inline-block h-3 w-3 shrink-0 rounded-full bg-amber-400 pulse-dot text-amber-400" />
            <div>
              <div className="text-base font-semibold text-amber-100">
                NOT ARMED — missions are locked out
              </div>
              <p className="mt-1 max-w-3xl text-sm text-amber-100/85">
                fpms-missions reports{" "}
                <span className="font-mono">arm_required = true</span> and{" "}
                <span className="font-mono">
                  armed = {armed === null ? "not reported" : "false"}
                </span>
                . It refuses every run with <i>“rover is not armed”</i>, so the
                mission buttons below are disabled rather than left to produce
                refusals. An arm is the operator's consent that the arena is
                clear; it expires on its own, and any stop or abort clears it.{" "}
                <b>
                  The stick, the turns and the nudges are NOT affected — teleop
                  has no arm concept
                </b>
                , and pretending otherwise would attribute an interlock to a
                machine that does not have one.
              </p>
            </div>
          </div>
        )}

        {/* Arm state, said out loud in all three cases. armed / not armed / not
            reported are three different situations and only one is a lockout.
            Collapsing "the executor is not running" into "not armed" would
            blame an interlock for a service that was never up. */}
        {thing && (
          <div className="flex flex-wrap items-center gap-2 text-xs text-slate-500">
            <span className="lbl">Arming</span>
            {armRequired === null ? (
              <span
                className="chip"
                title="fpms-missions has not published arm_required. Arming lives entirely in the mission executor, so nothing here is asserted about it — and nothing is locked out on the strength of a message that was never sent."
              >
                not reported — executor silent
              </span>
            ) : armRequired === false ? (
              <span className="chip" title="The executor reports arm_required = false">
                not required by the executor
              </span>
            ) : armed === true ? (
              <span className="chip-ok" title="The executor reports armed = true">
                ARMED
              </span>
            ) : (
              <span
                className="chip-hot"
                title="The executor requires arming and reports it is not armed. It refuses every mission with “rover is not armed”."
              >
                NOT ARMED — missions refused by the rover
              </span>
            )}
            <span>
              Arming gates the <span className="font-mono">mission</span> verb and
              nothing else. It is the executor's, it times out on its own, and
              this dashboard reads it rather than setting it.
            </span>
          </div>
        )}

        <Card>
          <CardHeader
            title="Rover under the stick"
            subtitle="Everything below except STOP ALL"
            right={
              <div className="flex flex-wrap items-center gap-2">
                {bays.map((b) => (
                  <StatusPill
                    key={b}
                    connected={ev.connected && things.includes(b)}
                    lastAt={seen[b]?.lastAt ?? null}
                    messages={seen[b]?.count ?? 0}
                  />
                ))}
              </div>
            }
          />
          <div className="flex flex-wrap gap-2">
            {bays.map((b) => (
              <button
                key={b}
                onClick={() => setPicked(b)}
                className={`chip font-mono ${
                  thing === b ? "border-ember-500/40 bg-ember-500/10 text-ember-200" : ""
                }`}
                title={things.includes(b) ? "reporting" : "not reporting"}
              >
                {b}
                {!things.includes(b) && <span className="text-[10px] text-slate-500">· silent</span>}
              </button>
            ))}
          </div>
          <p className="mt-3 text-xs text-slate-500">
            One rover at a time — a stick that moves the whole fleet is not a
            control. STOP ALL deliberately ignores this selector.
          </p>
        </Card>

        {/* Health — the operator asked for battery volts and link state front
            and centre, so it sits above the controls rather than beside them. */}
        <ErrorBoundary label="Drive health">
          <HealthCard
            thing={thing}
            tele={tele}
            channel={{ connected: drive.connected, lastAt: drive.lastAt, messages: drive.messages }}
            ageMs={teleAgeMs}
            stale={teleStale}
            reboots={link.reboots}
            lastRebootAt={link.lastRebootAt}
          />
        </ErrorBoundary>

        {/* The envelope, read off the rover where the rover reports it. jog and
            nudge are retunable at runtime from the Control tab, so printing the
            compiled-in defaults would go on claiming a cap the operator had
            already changed. */}
        <ErrorBoundary label="Drive limits">
          <SpeedEnvelope tele={tele} limits={caps.limits} />
        </ErrorBoundary>

        {/* The caps above say how fast the rover may go. This says what happens
            at the bottom of the range — which on this firmware is not what
            anybody assumed, and the assumption cost two multi-hour debugging
            sessions. */}
        <ErrorBoundary label="Drive speed floor">
          <SpeedFloorCard tele={tele} />
        </ErrorBoundary>

        {/* What the rover is doing to itself, as opposed to what the stick is
            doing to it. Sits above the mission buttons so the answer to "did
            that go anywhere?" is on screen before the next click. */}
        <ErrorBoundary label="Drive mission">
          <MissionCard
            thing={thing}
            mission={mission}
            channel={{
              connected: missionCh.connected,
              lastAt: missionCh.lastAt,
              messages: missionCh.messages,
            }}
            ageMs={missionCh.lastAt === null ? null : now - missionCh.lastAt}
            fallbackPose={{
              x_mm: num(tele?.x_mm),
              y_mm: num(tele?.y_mm),
              heading_deg: num(tele?.heading_deg),
            }}
            onAbort={thing ? () => abortMission([thing]) : undefined}
          />
        </ErrorBoundary>

        {/* Birdseye. The arena is fixed — grid, zones and orientation never
            rotate — and the rover moves across it. That is the whole contract
            of this view: if the map moved with the rover, "where is it" would
            have no answer you could point at. */}
        {thing ? (
          <ErrorBoundary label="Drive arena map">
            <Card>
              <CardHeader
                title="Arena"
                subtitle="Fixed birdseye · the rover moves, the map does not"
                right={
                  <div className="flex flex-wrap items-center gap-2">
                    <span
                      className={
                        poseBroken
                          ? "chip-hot font-mono"
                          : mapPoseSource
                            ? "chip font-mono"
                            : "chip-warn font-mono"
                      }
                      title={
                        poseBroken
                          ? "The reported position is outside the arena and has been refused — the glyph is at the assumed start, not where anything claims the rover is"
                          : mapPoseSource
                            ? `Rover drawn from the ${mapPoseSource}'s dead-reckoned pose — integrated from an assumed start, not measured`
                            : "Nothing is publishing a position — the rover is drawn at the assumed start corner"
                      }
                    >
                      pose · {poseBroken ? "REFUSED" : (mapPoseSource ?? "assumed")}
                    </span>
                    {planRoute ? (
                      <span className="chip font-mono" title="A planned route is shown">
                        route · {planRoute.length} pts
                      </span>
                    ) : (
                      <span className="chip font-mono text-slate-500">no plan</span>
                    )}
                  </div>
                }
              />
              <div className="mx-auto w-full max-w-[560px]">
                <ArenaMap
                  thing={thing}
                  poseEnvelope={mapPose ?? poseCh.data}
                  route={planRoute}
                />
              </div>
              <p className="mt-3 text-xs text-slate-500">
                {poseBroken
                  ? `The ${brokenPoseSource} reported a position outside the arena, so it has been refused rather than drawn. The glyph below is the assumed start corner and the map says SIMULATED — that is this dashboard declining to plot a broken estimate, not the rover parking itself.`
                  : mapPoseSource
                    ? `The rover glyph is the ${mapPoseSource}'s own position — dead-reckoned in arena millimetres from an ASSUMED start, never measured. Nothing on this rover localises against the map, so it drifts and only a set coordinate resets it.`
                    : "Nothing is publishing a position, so the rover is drawn at the assumed start corner and the map says SIMULATED. Neither the mission executor nor the teleop bridge has reported x/y."}
              </p>
            </Card>
          </ErrorBoundary>
        ) : null}

        <div className="grid gap-5 lg:grid-cols-2">
          <Card>
            <CardHeader
              title="Joystick"
              subtitle={`Streams jog at ${JOG_HZ} Hz`}
              right={
                <div className="flex items-center gap-2">
                  {jogFails > 1 && <span className="chip-hot">{jogFails} jogs failed</span>}
                  <span className={stickActive ? "chip-hot" : "chip"}>
                    {stickActive ? "held" : "idle"}
                  </span>
                </div>
              }
            />
            <div className="flex flex-col items-center gap-4">
              <Joystick
                onChange={sendJog}
                onActiveChange={onStickActive}
                hz={JOG_HZ}
                deadZone={0.08}
                size={240}
                disabled={motionDisabled}
                disabledHint={disabledHint}
              />
              <div className="w-full rounded-lg border border-white/5 bg-black/30 px-3 py-2 text-center font-mono text-xs text-slate-400">
                {lastCmd === null
                  ? "nothing sent from this stick yet"
                  : `sent · vx ${lastCmd.vx.toFixed(2)} · wz ${lastCmd.wz.toFixed(2)}`}
              </div>
            </div>
            <p className="mt-3 text-xs text-slate-500">
              Push up to go forward, right to turn right; output is normalised
              −1…1 and the rover maps full deflection to its jog cap —{" "}
              {num(tele?.jog_max_mps) !== null
                ? `${num(tele?.jog_max_mps)!.toFixed(3)} m/s, as the rover reported it on its last packet`
                : `${CAP_DEFAULTS.jogMaxMps.toFixed(3)} m/s by default; the rover has not reported the value in force`}
              . The centre <span className="font-mono">8%</span> is a dead
              zone so a resting thumb cannot creep the rover. While the stick is
              held, jog is sent continuously — even when it is held still —
              because the rover halts on its own if nothing arrives for{" "}
              <span className="font-mono">{DEADMAN_MS} ms</span>. Letting go
              sends a zero and a stop immediately.
            </p>
          </Card>

          <Card>
            <CardHeader title="Turn in place" subtitle="Closed loop to the angle" />
            <div className="flex items-center justify-between gap-4">
              {/* Left buttons on the left, right on the right. Reading the
                  control has to match doing it — a row of identical buttons
                  labelled by text alone is a mis-click waiting to happen. */}
              <div className="flex flex-col items-start gap-2">
                <span className="lbl">left ↺</span>
                <div className="flex gap-2">
                  {[...TURN_DEGS].reverse().map((d) => (
                    <button
                      key={`l${d}`}
                      className="btn font-mono"
                      disabled={motionDisabled}
                      title={motionDisabled ? disabledHint : `Turn left ${d}°`}
                      onClick={() => move("turn", { dir: "left", deg: d })}
                    >
                      ← {d}°
                    </button>
                  ))}
                </div>
              </div>
              <div className="flex flex-col items-end gap-2">
                <span className="lbl">right ↻</span>
                <div className="flex gap-2">
                  {TURN_DEGS.map((d) => (
                    <button
                      key={`r${d}`}
                      className="btn font-mono"
                      disabled={motionDisabled}
                      title={motionDisabled ? disabledHint : `Turn right ${d}°`}
                      onClick={() => move("turn", { dir: "right", deg: d })}
                    >
                      {d}° →
                    </button>
                  ))}
                </div>
              </div>
            </div>

            <div className="mt-5 border-t border-white/5 pt-4">
              <div className="lbl mb-2">Nudge · {NUDGE_MM} mm steps</div>
              <div className="flex flex-wrap gap-2">
                <button
                  className="btn"
                  disabled={motionDisabled}
                  title={motionDisabled ? disabledHint : "Forward 100 mm"}
                  onClick={() => move("nudge", { dir: "fwd", mm: NUDGE_MM })}
                >
                  ↑ F 10cm
                </button>
                <button
                  className="btn"
                  disabled={motionDisabled}
                  title={motionDisabled ? disabledHint : "Back 100 mm"}
                  onClick={() => move("nudge", { dir: "back", mm: NUDGE_MM })}
                >
                  ↓ B 10cm
                </button>
              </div>
            </div>
            <p className="mt-3 text-xs text-slate-500">
              None of these are confirm-gated: each is bounded — a fixed angle or
              a {NUDGE_MM} mm step — and asking twice for something that small
              trains the reflex to click through the confirmations that do
              matter.
            </p>
          </Card>

          <Card>
            <CardHeader
              title="Missions"
              subtitle="Hands the route to the rover"
              right={
                <div className="flex flex-wrap items-center gap-2">
                  <span
                    className={catalog.fromRover ? "chip-ok" : "chip-warn"}
                    title={
                      catalog.fromRover
                        ? "Buttons built from the mission list this rover announced"
                        : "The executor has not announced its mission list — events/online is published once and not retained. These are the dashboard's fallbacks, which may be out of date."
                    }
                  >
                    {catalog.fromRover ? "from rover" : "not yet announced"}
                  </span>
                  <span className="chip font-mono" title="Backend the next mission will be sent with">
                    via {backend}
                  </span>
                </div>
              }
            />

            {/* The selector sits above the buttons it changes. A backend
                picked in another card is a setting nobody reads before
                clicking. The list itself is the rover's — fpms_missions
                announces `backends` and `default_backend`, and the default is
                configurable on the rover, so hardcoding it here would send
                missions on a backend the rover was deliberately configured
                away from. */}
            <div className="mb-4">
              <div className="lbl mb-2">Driven by</div>
              <div className="flex flex-wrap gap-2">
                {backends.map((id) => {
                  const b = BACKEND_NOTES[id];
                  return (
                    <button
                      key={id}
                      onClick={() => setBackend(id)}
                      className={`chip ${
                        backend === id
                          ? "border-ember-500/40 bg-ember-500/10 text-ember-200"
                          : ""
                      }`}
                      title={b?.warn ?? b?.note ?? "Backend advertised by the rover"}
                    >
                      <span className="font-mono">{id}</span>
                      {b ? (
                        <span className="ml-1.5 text-[10px] text-slate-500">· {b.note}</span>
                      ) : (
                        <span className="ml-1.5 text-[10px] text-amber-300">
                          · advertised by the rover, unknown to this dashboard
                        </span>
                      )}
                      {caps.defaultBackend === id && (
                        <span className="ml-1.5 text-[10px] text-slate-500">· rover default</span>
                      )}
                    </button>
                  );
                })}
              </div>
              {BACKEND_NOTES[backend]?.warn ? (
                <div className="mt-2 rounded-lg border border-amber-500/40 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90">
                  <b>{BACKEND_NOTES[backend].label} is not ready on this rover.</b>{" "}
                  {BACKEND_NOTES[backend].warn} Switch back to{" "}
                  <span className="font-mono">
                    {caps.defaultBackend ?? FALLBACK_DEFAULT_BACKEND}
                  </span>{" "}
                  unless you are deliberately testing the stack.
                </div>
              ) : null}
            </div>

            {/* PLAN FIRST. Separate row, above the buttons that actually drive,
                because the intended order of operations is plan -> look at the
                map -> commit. These need no confirmation and no motion lock:
                they move nothing. */}
            <div className="mb-4">
              <div className="lbl mb-2">
                Plan first · shows the route, drives nothing
              </div>
              <div className="flex flex-wrap gap-2">
                {/* RETURN HOME is previewable too, and used to be the one route
                    you could not look at before committing to it. The executor
                    plans it through the identical code path as the others —
                    fpms_missions._preview accepts any commandable name — and
                    "get the rover back" is exactly the moment an operator wants
                    to see the line before pressing the button. */}
                {missionOptions.map((m) => (
                  <button
                    key={m.name}
                    className="chip"
                    onClick={() => planMission(m.name)}
                    disabled={!thing}
                    title={`Preview the route to ${m.name} — ${m.where} — without moving${
                      m.fromRover ? "" : " (corner description is the dashboard's, not the rover's)"
                    }`}
                  >
                    PLAN {m.label.replace(/^MISSION /, "").replace(/^RETURN /, "")}
                  </button>
                ))}
                {planRoute ? (
                  <button
                    className="chip"
                    onClick={() => setDismissedPlanTs(planTs)}
                    title="Remove the planned route from the map"
                  >
                    CLEAR PLAN
                  </button>
                ) : null}
              </div>
              {plan ? (
                <div className="mt-2 rounded-lg border border-sky-500/30 bg-sky-500/5 px-3 py-2 text-xs text-sky-100/90">
                  {/* Every number here can be genuinely absent, and an absent
                      distance rendered as "0 mm" would read as "already
                      there" — which is the most misleading thing a route
                      summary could possibly say. */}
                  <span className="font-mono">{plan.mission ?? "--"}</span> ·{" "}
                  {plan.distanceMm === null ? "--" : `${Math.round(plan.distanceMm)} mm`} ·{" "}
                  {plan.segments === null ? "--" : plan.segments} segments · ETA{" "}
                  {plan.etaS === null ? "--" : `~${Math.round(plan.etaS)} s`}
                  {plan.returnsHome ? " · returns home" : ""}
                  {plan.backend ? ` · via ${plan.backend}` : ""}
                  <div className="mt-1 text-[11px] text-sky-200/70">
                    Nominal route. The executor re-measures its bearing after
                    every leg and inserts corrections, so the driven path will
                    differ from this line.
                    {plan.poseAssumed
                      ? " Start pose is ASSUMED — nothing localises this rover, so the whole route is only as right as that assumption."
                      : ""}
                  </div>
                </div>
              ) : null}
            </div>

            <div className="flex flex-wrap gap-2">
              {missionOptions.map((m) => (
                <ConfirmButton
                  key={m.name}
                  className={m.primary ? "btn-primary" : "btn"}
                  label={m.label}
                  onFire={() => {
                    // Gated on the mission lock, not the motion lock: arming is
                    // the executor's and applies here and nowhere else.
                    if (!thing || missionLocked) return;
                    fire("mission", [thing], { name: m.name, backend });
                  }}
                  disabled={missionDisabled}
                  hint={
                    missionDisabled
                      ? missionDisabledHint
                      : `Run ${m.name} (${m.where}) using the ${backend} backend`
                  }
                />
              ))}
            </div>
            <p className="mt-3 text-xs text-slate-500">
              {notArmed && !motionLocked ? (
                <>
                  <b className="text-amber-200">
                    Disabled because {thing} is not armed.
                  </b>{" "}
                  The executor would refuse each of these with “rover is not
                  armed”. The stick and the bounded turns and nudges above are
                  unaffected — teleop does not have an arm gate.{" "}
                </>
              ) : null}
              All {missionOptions.length} are confirm-gated: each one drives the
              rover somewhere on its own, and from here a rover on blocks and a
              rover on the floor look identical. The PLAN row above is not gated
              at all — a preview commands no motion, and the moment you most want
              to see the intended route is while you are deciding whether it is
              safe to run. Each is sent as{" "}
              <span className="font-mono">{`{name, backend}`}</span> — progress
              comes back in the Mission card above. Take the stick or hit STOP ALL
              to cut a mission short; STOP ALL is also the abort — fpms-missions
              subscribes <span className="font-mono">stop</span> and ends the run
              on it, which is why there is no separate abort verb.
            </p>
            <p className="mt-2 text-xs text-slate-500">
              {catalog.fromRover ? (
                <>
                  This list is <b>the rover's</b>: fpms-missions announced{" "}
                  {catalog.names.length} commandable name
                  {catalog.names.length === 1 ? "" : "s"}, and a mission it drops
                  loses its button here on the next announcement.
                </>
              ) : (
                <>
                  <b>The executor has not announced its mission list.</b>{" "}
                  <span className="font-mono">events/online</span> is published
                  once and not retained, so a dashboard opened after the rover
                  booted never saw it — these are the dashboard's fallback names
                  and they may be out of date. Sending an unknown one is refused
                  by the rover with the valid list attached, which lands in the
                  log below.
                </>
              )}
            </p>
          </Card>

          <Card>
            <CardHeader title="Set coordinate" subtitle="Tells the rover where it is" />
            {/* The rover says whether it has an origin at all, and the answer
                changes what this control is for: with no origin the pose is
                measured from wherever the bridge happened to start, so this is
                not a correction, it is the thing that makes the arena frame
                exist. */}
            {tele?.origin_set === false && (
              <div className="mb-3 rounded-lg border border-amber-500/40 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90">
                <b>No origin is set on this rover.</b> x/y are being measured
                from wherever the teleop bridge started, not in the arena frame
                the map draws in, so every coordinate on this page and every
                mission planned from it is offset by an unknown amount. Set it
                before running anything.
              </div>
            )}
            {/* This control is left usable while motion is locked — it moves
                nothing — but an operator has to know it is being sent into a
                silence. */}
            {neverSeen && !noRover && (
              <div className="mb-3 rounded-lg border border-amber-500/40 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90">
                <b>{thing} has never reported this session.</b> A coordinate set
                now goes out over MQTT and will not be acknowledged by anything
                this page can see. The boxes start empty on purpose: there is no
                default position, and a pre-filled 0/0 would publish "you are at
                the arena origin" on a single click.
              </div>
            )}
            <SetCoordinate
              disabled={noRover}
              onFire={(x, y) => thing && fire("set_coordinate", [thing], { x_mm: x, y_mm: y })}
            />
            <p className="mt-3 text-xs text-slate-500">
              Writes the rover's believed position in arena millimetres. It moves
              nothing, so there is nothing to confirm and it stays available when
              the motion controls are locked — but the number has to be right, or
              every mission afterwards is wrong by the same amount.{" "}
              <b>Re-set it after every boot.</b> The origin file survives a
              reboot while the board's odometry restarts at zero, so a stale
              reference is not a small error — it has already placed a rover
              nine metres outside a 1.2 m arena, and the planner believed it.
            </p>
          </Card>
        </div>

        <Card>
          <CardHeader
            title="Acknowledgements"
            subtitle="Live from fpms/+/events/#"
            right={
              <div className="flex items-center gap-2">
                <span className={ev.connected ? "chip-ok" : "chip-warn"}>
                  {ev.connected ? "stream up" : "stream down"}
                </span>
                <button className="btn" onClick={() => setLog([])} disabled={log.length === 0}>
                  Clear
                </button>
              </div>
            }
          />
          {log.length === 0 ? (
            <div className="text-sm text-slate-500">
              Nothing yet. Jog traffic is left out on purpose — at {JOG_HZ} Hz it
              would bury every reply that matters.
            </div>
          ) : (
            <ul className="space-y-1">
              {log.map((e) => (
                <li
                  key={e.id}
                  className="flex items-center gap-3 rounded-md border border-white/5 bg-black/30 px-3 py-1.5"
                >
                  <span className="font-mono text-[11px] text-slate-500">
                    {new Date(e.at).toLocaleTimeString()}
                  </span>
                  <span className="font-mono text-xs text-slate-300">{e.thing}</span>
                  <span className="font-mono text-xs text-ember-300/90">{e.action ?? "—"}</span>
                  <span className={`${KIND_CHIP[e.kind]} ml-auto`}>{KIND_LABEL[e.kind]}</span>
                  <span
                    className="max-w-[46%] truncate font-mono text-[11px] text-slate-400"
                    title={e.hint ?? e.text}
                  >
                    {e.text}
                  </span>
                </li>
              ))}
            </ul>
          )}
          {log.some((e) => e.kind === "stale") && (
            <div className="mt-3 rounded-lg border border-amber-500/25 bg-amber-500/5 p-3 text-xs text-amber-200/90">
              <b>Agent too old</b> — the rover answered “unknown command”. The
              drive actions (jog, turn, nudge, mission, set_coordinate) need an
              on-rover agent that implements them; the dashboard side worked.
            </div>
          )}
        </Card>
      </div>
    </ErrorBoundary>
  );
}

/* ---- link state ---------------------------------------------------------- */

type LinkPhase = "live" | "stale" | "reconnecting" | "never";

type LinkSession = {
  /** When the first packet of this session landed. null = none, ever. */
  firstSeenAt: number | null;
  lastSeenAt: number | null;
  packets: number;
  /** Gaps longer than LINK_LOST_MS that telemetry came back from. */
  reconnects: number;
  /** Times uptime_s went backwards — the rover power-cycled or its bridge did. */
  reboots: number;
  lastRebootAt: number | null;
  /** uptime_s from the previous packet — the baseline the next one is judged against. */
  lastUptimeS: number | null;
  /** uptime_s immediately before the most recent restart, for the copy. */
  uptimeBeforeReboot: number | null;
};

function newSession(): LinkSession {
  return {
    firstSeenAt: null,
    lastSeenAt: null,
    packets: 0,
    reconnects: 0,
    reboots: 0,
    lastRebootAt: null,
    lastUptimeS: null,
    uptimeBeforeReboot: null,
  };
}

/**
 * Fold one arriving packet into the session.
 *
 * The restart test is uptime_s going backwards. That is the only signal the
 * rover gives that survives a power cut: the link dropping tells us nothing
 * about why, and a rover that reboots quickly enough can come back inside a gap
 * short enough that nothing else on the page would ever mention it. A silent
 * reboot mid-session invalidates the odometry, the mode and anything in flight,
 * so it is worth a counter that persists for the rest of the session.
 */
function advanceSession(s: LinkSession, uptimeS: number | null, at: number): LinkSession {
  const gapMs = s.lastSeenAt === null ? null : at - s.lastSeenAt;
  const reconnected = gapMs !== null && gapMs > LINK_LOST_MS;

  const prevUptime = s.lastUptimeS;
  const rebooted =
    prevUptime !== null && uptimeS !== null && uptimeS + REBOOT_SLACK_S < prevUptime;

  return {
    firstSeenAt: s.firstSeenAt ?? at,
    lastSeenAt: at,
    packets: s.packets + 1,
    reconnects: s.reconnects + (reconnected ? 1 : 0),
    reboots: s.reboots + (rebooted ? 1 : 0),
    lastRebootAt: rebooted ? at : s.lastRebootAt,
    // A packet without uptime_s must not erase the baseline, or the field
    // flickering would hide the very restart it is there to catch.
    lastUptimeS: uptimeS ?? prevUptime,
    uptimeBeforeReboot: rebooted ? prevUptime : s.uptimeBeforeReboot,
  };
}

function linkPhase(s: LinkSession, now: number): { phase: LinkPhase; ageMs: number | null } {
  if (s.lastSeenAt === null || !Number.isFinite(s.lastSeenAt)) {
    return { phase: "never", ageMs: null };
  }
  const ageMs = Math.max(0, now - s.lastSeenAt);
  if (ageMs < LINK_LIVE_MS) return { phase: "live", ageMs };
  if (ageMs < LINK_LOST_MS) return { phase: "stale", ageMs };
  return { phase: "reconnecting", ageMs };
}

const LINK_STYLE: Record<LinkPhase, { box: string; dot: string; title: string; chip: string }> = {
  live: {
    box: "border-emerald-500/40 bg-emerald-500/5",
    dot: "bg-emerald-400 text-emerald-400",
    title: "text-emerald-200",
    chip: "chip-ok",
  },
  stale: {
    box: "border-amber-500/40 bg-amber-500/5",
    dot: "bg-amber-400 text-amber-400 pulse-dot",
    title: "text-amber-100",
    chip: "chip-warn",
  },
  reconnecting: {
    box: "border-ember-500/40 bg-ember-500/10",
    dot: "bg-ember-400 text-ember-400 pulse-dot",
    title: "text-ember-200",
    chip: "chip-hot",
  },
  never: {
    box: "border-slate-500/30 bg-black/30",
    dot: "bg-slate-500 text-slate-500",
    title: "text-slate-300",
    chip: "chip",
  },
};

/**
 * The four states an operator actually cares about, kept apart on purpose.
 *
 * "Never seen" and "was here and dropped" get the same silence on the wire and
 * mean opposite things standing next to the machine: one is a bay that was never
 * publishing — bridge not running, wrong rover selected — and the other is a
 * rover that is rebooting and will come back on its own. Collapsing them into a
 * single "offline" is what sends someone to power-cycle a rover that was about
 * to reconnect.
 */
function LinkBanner({
  thing,
  phase,
  ageMs,
  session,
  socketConnected,
  uptime,
  now,
}: {
  thing: string | null;
  phase: LinkPhase;
  ageMs: number | null;
  session: LinkSession;
  socketConnected: boolean;
  uptime: unknown;
  now: number;
}) {
  const st = LINK_STYLE[phase];
  const channel = thing ? `drive:${thing}` : "no rover selected";
  const ago = agoText(ageMs);

  const headline =
    phase === "live"
      ? "LIVE — rover is talking to the dashboard"
      : phase === "stale"
        ? `STALE — no packet for ${ago}`
        : phase === "reconnecting"
          ? `RECONNECTING — silent for ${ago}`
          : session.firstSeenAt === null && thing
            ? "OFFLINE — never seen this session"
            : "OFFLINE — no rover selected";

  const body =
    phase === "live" ? (
      <>
        Telemetry is current on <span className="font-mono">{channel}</span> — last
        packet {ago} ago, {session.packets} this session.
        {session.reconnects > 0 && (
          <>
            {" "}
            It has come back from {session.reconnects} dropout
            {session.reconnects > 1 ? "s" : ""} since the page loaded.
          </>
        )}
      </>
    ) : phase === "stale" ? (
      <>
        The last packet on <span className="font-mono">{channel}</span> was {ago} ago.
        The rover was live at {clock(session.lastSeenAt)}; one dropped packet looks
        exactly like this, so nothing is being called yet. Readings below are that
        old — treat them as history, not as the rover's state now.
      </>
    ) : phase === "reconnecting" ? (
      <>
        <b>This rover was connected and dropped.</b> First packet at{" "}
        {clock(session.firstSeenAt)}, last at {clock(session.lastSeenAt)},{" "}
        {session.packets} in total — then nothing for {ago}. That is a rover
        powering off, rebooting, or losing its link, not a rover that was never
        there. The page keeps listening and flips itself back to{" "}
        <b className="text-emerald-300">LIVE</b> the moment telemetry returns; no
        reload needed.
      </>
    ) : thing ? (
      <>
        <b>Nothing has ever arrived</b> on <span className="font-mono">{channel}</span>{" "}
        since this page loaded — this is not a dropped link, it is a bay that has
        not published at all. Either the teleop bridge is not running on{" "}
        <span className="font-mono">{thing}</span>, the rover is off, or the wrong
        bay is selected above. A rover that had connected and then dropped would
        say <b className="text-ember-300">RECONNECTING</b> instead.
      </>
    ) : (
      <>Pick a rover above to subscribe to its drive telemetry.</>
    );

  return (
    <div className={`rounded-xl border-2 p-4 ${st.box}`}>
      <div className="flex flex-wrap items-start justify-between gap-3">
        <div className="flex items-start gap-3">
          <span className={`mt-1.5 inline-block h-3 w-3 shrink-0 rounded-full ${st.dot}`} />
          <div>
            <div className={`text-base font-semibold tracking-wide ${st.title}`}>{headline}</div>
            <p className="mt-1 max-w-3xl text-sm text-slate-300/90">{body}</p>
          </div>
        </div>
        <div className="flex flex-wrap items-center gap-2">
          <span className={st.chip}>{PHASE_LABEL[phase]}</span>
          <span className="chip font-mono" title="Age of the newest drive telemetry packet">
            last {ago}
          </span>
          <span className="chip font-mono" title="Drive telemetry packets received since page load">
            {session.packets} pkt
          </span>
          {session.reconnects > 0 && (
            <span className="chip-warn font-mono" title="Times telemetry returned after a gap">
              {session.reconnects} reconnect{session.reconnects > 1 ? "s" : ""}
            </span>
          )}
          <span
            className={socketConnected ? "chip-ok" : "chip-hot"}
            title={
              socketConnected
                ? "The browser's websocket to the dashboard is open — silence here is the rover's"
                : "The browser cannot reach the dashboard backend. This is a dashboard-side fault, not the rover."
            }
          >
            {socketConnected ? "ws up" : "ws down"}
          </span>
          <span className="chip font-mono" title="uptime_s reported by the rover">
            up {dur(uptime)}
          </span>
        </div>
      </div>

      {/* A restart mid-session is the thing that quietly costs an hour: the
          odometry resets, the mode resets, and nothing else on the page says so. */}
      {session.reboots > 0 && (
        <div className="mt-3 flex items-start gap-3 rounded-lg border border-ember-500/40 bg-ember-500/10 px-3 py-2">
          <span className="mt-1 inline-block h-2.5 w-2.5 shrink-0 rounded-full bg-ember-400 text-ember-400 pulse-dot" />
          <div className="text-sm text-ember-100">
            <b>
              ROVER RESTARTED
              {session.reboots > 1 ? ` ×${session.reboots}` : ""}
            </b>{" "}
            — uptime went backwards at {clock(session.lastRebootAt)}
            {session.uptimeBeforeReboot !== null && (
              <> (was {dur(session.uptimeBeforeReboot)} before the drop)</>
            )}
            {session.lastRebootAt !== null && (
              <> · {agoText(Math.max(0, now - session.lastRebootAt))} ago</>
            )}
            .
            <div className="mt-0.5 text-xs text-ember-200/80">
              Odometry, mode and anything in flight before that point are gone —
              x/y/heading below are measured from wherever the rover happened to be
              at power-on. Re-set the coordinate before trusting a mission.
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

const PHASE_LABEL: Record<LinkPhase, string> = {
  live: "LIVE",
  stale: "STALE",
  reconnecting: "RECONNECTING",
  never: "NEVER SEEN",
};

/* ---- front clearance ----------------------------------------------------- */

type ClearanceState = "clear" | "closing" | "blocked" | "unknown";

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
const CLEAR_CHIP: Record<ClearanceState, string> = {
  clear: "chip-ok",
  closing: "chip-warn",
  blocked: "chip-hot",
  unknown: "chip",
};
const CLEAR_WORD: Record<ClearanceState, string> = {
  clear: "CLEAR",
  closing: "CLOSING",
  blocked: "BLOCKED",
  unknown: "NO READING",
};

/**
 * How far it is to whatever is in front of the rover, at the size that number
 * deserves.
 *
 * Reads `front_mm` off fpms_missions.snapshot(). The executor OMITS that field
 * when its LiDAR is blind rather than sending a zero, so `null` here means "no
 * one is measuring the path ahead" and renders `--`. Rendering it as 0 would
 * say the bumper is against something — the exact opposite of the truth on a
 * rover whose scanner has dropped out, and the most dangerous single
 * mistranslation available on this page.
 *
 * The three loud states are deliberately not symmetric with the quiet one:
 * CLOSING opens at three times the stop distance so the readout changes
 * character on the way in rather than at the moment the executor is already
 * braking.
 */
function ClearanceBar({
  mm,
  stopMm,
  stopFromRover,
  running,
  lidarOk,
  hasExecutor,
}: {
  mm: number | null;
  stopMm: number;
  stopFromRover: boolean;
  running: boolean;
  lidarOk: boolean | null;
  hasExecutor: boolean;
}) {
  const state = clearanceState(mm, stopMm);
  const loud = state === "blocked" || (state === "unknown" && running);

  return (
    <div className={`rounded-xl border-2 p-4 ${CLEAR_BOX[state]}`}>
      <div className="flex flex-wrap items-center justify-between gap-4">
        <div className="flex items-center gap-4">
          {loud && (
            <span className="inline-block h-3 w-3 shrink-0 rounded-full bg-rose-400 pulse-dot text-rose-400" />
          )}
          <div>
            <div className="lbl">Front clearance</div>
            <div
              className={`font-mono text-5xl font-semibold tabular-nums ${CLEAR_INK[state]}`}
            >
              {mm === null ? "--" : Math.round(mm)}
              <span className="ml-1 text-xl text-slate-500">mm</span>
            </div>
          </div>
        </div>
        <div className="flex flex-wrap items-center gap-2">
          <span className={CLEAR_CHIP[state]}>{CLEAR_WORD[state]}</span>
          <span
            className="chip font-mono"
            title={
              stopFromRover
                ? "front_stop_mm, as the mission executor announced it"
                : "The executor has not announced its limits — this is the value fpms_missions ships with"
            }
          >
            stops at {Math.round(stopMm)} mm
            {stopFromRover ? "" : " · default"}
          </span>
          {lidarOk === false && (
            <span className="chip-hot" title="The executor reports its obstacle guard has no fresh scan">
              guard blind
            </span>
          )}
        </div>
      </div>

      {state === "blocked" && (
        <p className="mt-3 text-sm text-rose-100">
          <b>Inside the executor's stop distance.</b> A mission would stop here.
          If the rover is still moving, either it is braking now or it is not
          seeing this — hit STOP ALL rather than waiting to find out which.
        </p>
      )}
      {state === "closing" && (
        <p className="mt-3 text-sm text-amber-100/90">
          <b>Closing on something.</b> The last collision on this rover went{" "}
          <span className="font-mono">989 mm → 117 mm</span> while nobody was
          watching this number. Slow is bought with short bursts and full stops,
          not with a smaller setpoint — see Speed floor below.
        </p>
      )}
      {state === "unknown" && (
        <p className="mt-3 text-sm text-slate-400">
          {!hasExecutor ? (
            <>
              <b>No mission telemetry.</b> Front clearance comes from
              fpms-missions, which is not publishing on this bay, so nothing on
              this page is watching the path ahead. The LiDAR tab derives the
              same distance straight from the live scan when a scan is arriving.
            </>
          ) : running ? (
            <>
              <b>The rover is driving and no front distance is being reported.</b>{" "}
              The executor omits <span className="font-mono">front_mm</span> when
              its LiDAR is blind rather than sending a zero. Treat the path ahead
              as <b>unknown</b>, not as open.
            </>
          ) : (
            <>
              The executor is not reporting a front distance. It omits{" "}
              <span className="font-mono">front_mm</span> when the LiDAR is blind
              — an absent reading is shown as <span className="font-mono">--</span>,
              never as 0.
            </>
          )}
        </p>
      )}
    </div>
  );
}

/* ---- speed envelope ------------------------------------------------------ */

/**
 * What the rover is CURRENTLY enforcing, not what it was compiled with.
 *
 * `set_speed` on the Control tab retunes jog and nudge on the running service,
 * and every telemetry tick reports the values in force (`jog_max_mps`,
 * `nudge_mps`). A panel that printed the defaults would keep claiming 0.05 m/s
 * after an operator had halved it — a small lie of exactly the kind that is
 * only discovered by driving into something.
 *
 * Where the rover has not told us, the default is shown and labelled as a
 * default rather than passed off as a reading.
 */
function SpeedEnvelope({
  tele,
  limits,
}: {
  tele: Record<string, unknown> | null;
  limits: Readonly<Record<string, unknown>>;
}) {
  const jog = num(tele?.jog_max_mps) ?? num(limits.jog_max_mps);
  const jogLive = num(tele?.jog_max_mps) !== null;
  const nudge = num(tele?.nudge_mps) ?? num(limits.nudge_mps);
  const nudgeLive = num(tele?.nudge_mps) !== null;
  const dock = num(limits.dock_mps);
  const turn = num(limits.turn_max_radps) ?? num(limits.turn_radps);
  const hardLin = num(limits.hard_max_lin_mps);

  const rows: { what: string; value: number | null; unit: string; live: boolean; fallback: number }[] = [
    { what: "jog (stick)", value: jog, unit: "m/s", live: jogLive, fallback: CAP_DEFAULTS.jogMaxMps },
    { what: "nudge", value: nudge, unit: "m/s", live: nudgeLive, fallback: CAP_DEFAULTS.nudgeMps },
    { what: "dock", value: dock, unit: "m/s", live: false, fallback: CAP_DEFAULTS.dockMps },
    { what: "turn", value: turn, unit: "rad/s", live: false, fallback: CAP_DEFAULTS.turnRadps },
  ];

  return (
    <div className="rounded-xl border border-amber-500/25 bg-amber-500/5 p-4 text-sm text-amber-100/90">
      <b>Everything here is capped slow, on purpose.</b> The rover enforces these
      limits — full stick deflection asks for the jog cap, not for whatever the
      motors can do:
      <div className="mt-2 flex flex-wrap gap-2">
        {rows.map((r) => (
          <span
            key={r.what}
            className={r.live ? "chip-ok font-mono" : "chip font-mono"}
            title={
              r.live
                ? "Reported by the rover on this telemetry tick — this is the value in force"
                : r.value !== null
                  ? "From the rover's announced limits"
                  : "The rover has not reported this; showing the compiled-in default"
            }
          >
            {r.what} ≤ {(r.value ?? r.fallback).toFixed(3)} {r.unit}
            {r.value === null && <span className="ml-1 text-[10px] text-slate-400">· default</span>}
          </span>
        ))}
        {hardLin !== null && (
          <span className="chip font-mono" title="Not tunable from anywhere at runtime">
            hard cap ≤ {hardLin.toFixed(3)} m/s
          </span>
        )}
      </div>
      <div className="mt-2 text-xs text-amber-200/70">
        Nudges are fixed {NUDGE_MM} mm steps and turns are closed-loop to the
        requested angle. Green chips are values the rover reported on its last
        packet; plain chips are announced or compiled-in defaults it has not
        confirmed — jog and nudge are retunable at runtime from the Control tab,
        so those two are the ones worth reading off the rover rather than off
        this page.
      </div>
    </div>
  );
}

/* ---- speed floor --------------------------------------------------------- */

/**
 * WHAT HAPPENS AT THE BOTTOM OF THE RANGE — and the retraction that belongs
 * with it.
 *
 * This card used to be headed "Motor deadband floor · why very slow is not
 * available" and asserted, with a measurement, that a 100 mm forward command
 * came out as a 290 mm BACKWARD lurch with 24° of unrequested rotation.
 *
 * That measurement was an artefact. `/odom_raw`'s `twist.linear.x` is
 * sign-inverted relative to its own `pose.position` on this firmware; the rover
 * had moved forward, as commanded, the whole time. fpms_teleop.py carries the
 * retraction in full ("Every claim of the form 'this chassis cannot creep' is
 * withdrawn. It creeps.") and set both floors back to 0.0 — OFF. Repeating the
 * withdrawn claim on the operator's screen is how a corrected fault gets
 * re-learned by the next person.
 *
 * TWO THINGS ARE TRUE INSTEAD, AND THEY ARE NOT THE SAME THING:
 *
 *   1. The configurable floor is OFF and UNMEASURED. The rover publishes
 *      min_cmd_lin/min_cmd_ang as 0.0 by default. 0.0 means "no floor", NOT
 *      "measured at zero", and rendering it as a measured minimum was the
 *      remaining piece of the old story.
 *   2. Amplitude does not buy slowness on this firmware anyway. The velocity
 *      loop regulates integer encoder counts per 10 ms and the board applies a
 *      large fixed duty to any non-zero setpoint, so a smaller number is not a
 *      slower rover — it is a rover that either moves at the speed it always
 *      moves at, or does not move. Slow is bought with TIME: short bounded
 *      segments with full stops between them.
 */
function SpeedFloorCard({ tele }: { tele: Record<string, unknown> | null }) {
  const lin = num(tele?.min_cmd_lin);
  const ang = num(tele?.min_cmd_ang);
  const reported = lin !== null || ang !== null;
  // 0.0 is the rover's way of saying the mechanism is switched off. It is a
  // real reading and it is not a floor, so it is never rendered as one.
  const floorOn = (lin ?? 0) > 0 || (ang ?? 0) > 0;
  const snapped = tele?.deadband_snapped === true;

  return (
    <Card>
      <CardHeader
        title="There is no slow speed on this chassis"
        subtitle="What the bottom of the range actually does"
        right={
          <div className="flex flex-wrap items-center gap-2">
            <span
              className={floorOn ? "chip-warn" : reported ? "chip" : "chip-warn"}
              title={
                !reported
                  ? "The rover has not reported min_cmd_lin/min_cmd_ang yet"
                  : floorOn
                    ? "A floor is configured on the rover; commands below it are raised to it"
                    : "Both floors are 0.0 — the mechanism is switched off, not measured at zero"
              }
            >
              {!reported ? "not reported" : floorOn ? "FLOOR ON" : "floor off"}
            </span>
            {snapped && (
              <span className="chip-hot" title="The last command was below the configured floor and was raised to it">
                FLOOR APPLIED
              </span>
            )}
          </div>
        }
      />

      {/* The plain statement, first and unqualified. Everything below it is
          evidence for it or caveats to it, and an operator who reads nothing
          else on this card must still leave with this. */}
      <div className="mb-4 rounded-lg border-2 border-amber-500/50 bg-amber-500/10 p-3 text-sm text-amber-50">
        <b>The firmware applies roughly 50% duty to ANY non-zero setpoint.</b> A
        smaller number in a speed box does not produce a slower rover — it
        produces a rover that either moves at the speed it always moves at, or
        does not move at all. There is no gentle duty and there is no slow
        setpoint. <b>Slowness on this chassis comes from short bursts with full
        stops between them</b>, which is what the mission executor's
        dead-reckoning backend does and what the {NUDGE_MM} mm nudge buttons do.
        No control on this page is offered that would imply otherwise.
      </div>

      <div className="grid gap-3 sm:grid-cols-2">
        <FloorReadout
          label="min linear · min_cmd_lin"
          value={lin}
          unit="m/s"
          digits={4}
        />
        <FloorReadout
          label="min angular · min_cmd_ang"
          value={ang}
          unit="rad/s"
          digits={4}
        />
      </div>

      {snapped && (
        <div className="mt-3 rounded-lg border border-ember-500/40 bg-ember-500/10 px-3 py-2 text-sm text-ember-100">
          <b>Floor applied to the last command.</b> Someone has configured a
          non-zero <span className="font-mono">FPMS_MIN_CMD_*</span> on this
          rover, and the last command was below it and was raised. The rover is
          moving <b>faster</b> than asked — not slower, and not ignoring you.
        </div>
      )}

      <div className="mt-3 rounded-lg border border-amber-500/25 bg-amber-500/5 p-3 text-sm text-amber-100/90">
        <b>Do not try to make this rover slow by lowering the setpoint.</b> The
        board's velocity loop regulates integer encoder counts per 10 ms and
        applies a large fixed duty to any non-zero setpoint, so amplitude buys
        almost nothing: measured on this chassis, one small setpoint moved it at
        cruise while another, larger, produced no motion at all. The only lever
        that works is <b>time</b> — short bounded segments with a full stop
        between them, run at a speed the loop can actually hold. That is what
        the mission executor's dead-reckoning backend does.
      </div>

      <div className="mt-3 rounded-lg border border-white/5 bg-black/30 p-3 text-xs text-slate-400">
        <b className="text-slate-300">Retraction, kept on purpose.</b> This card
        previously reported a measured “100 mm forward became a 290 mm backward
        lurch”. That reading came from{" "}
        <span className="font-mono">/odom_raw twist.linear.x</span>, which is
        sign-inverted relative to its own{" "}
        <span className="font-mono">pose.position</span> on this firmware — the
        rover had gone forward the whole time. Both floors were switched back off
        and have never been measured on corrected data. A floor of{" "}
        <span className="font-mono">0.0000</span> above means the mechanism is{" "}
        <b>off</b>, not that the minimum was measured at zero.
      </div>
    </Card>
  );
}

function FloorReadout({
  label,
  value,
  unit,
  digits,
}: {
  label: string;
  value: number | null;
  unit: string;
  digits: number;
}) {
  const off = value !== null && value <= 0;
  return (
    <div
      className={`rounded-lg border px-3 py-2 ${
        value === null ? "border-amber-500/25 bg-amber-500/5" : "border-white/5 bg-black/20"
      }`}
    >
      <div className="lbl">{label}</div>
      <div
        className={`mt-0.5 font-mono text-2xl tabular-nums ${
          value === null ? "text-amber-300/80" : off ? "text-slate-400" : "text-slate-200"
        }`}
      >
        {value === null ? (
          <span className="text-base">not reported</span>
        ) : (
          <>
            {value.toFixed(digits)}
            <span className="ml-1 text-sm text-slate-500">{unit}</span>
          </>
        )}
      </div>
      <div className="mt-1 text-[11px] text-slate-500">
        {value === null
          ? "The rover has not published this field."
          : off
            ? "Zero means the floor is OFF — no command is raised. It is not a measured minimum."
            : "Commands below this are raised to it before they reach the wire."}
      </div>
    </div>
  );
}

/* ---- health -------------------------------------------------------------- */

function HealthCard({
  thing,
  tele,
  channel,
  ageMs,
  stale,
  reboots = 0,
  lastRebootAt = null,
}: {
  thing: string | null;
  tele: Record<string, unknown> | null;
  channel: { connected: boolean; lastAt: number | null; messages: number };
  ageMs: number | null;
  stale: boolean;
  /** Restarts counted this session — annotates the uptime readout. */
  reboots?: number;
  lastRebootAt?: number | null;
}) {
  const battV = num(tele?.battery_v);
  const battAge = num(tele?.battery_age_s);
  const battStale = stale || (battAge !== null && battAge > BATTERY_STALE_S);
  const battLow = tele?.battery_low === true;
  const battState: "OK" | "LOW" | "STALE" | "—" =
    battV === null ? "—" : battLow ? "LOW" : battStale ? "STALE" : "OK";

  const lastError = str(tele?.last_error);

  return (
    <Card glow>
      <CardHeader
        title="Health"
        subtitle={thing ? `drive:${thing}` : "no rover selected"}
        right={
          <div className="flex flex-wrap items-center gap-2">
            <span className={rosClass(tele)}>{rosLabel(tele)}</span>
            <span className={tele?.deadman_ok === false ? "chip-warn" : "chip"}>
              deadman {bool(tele?.deadman_ok)}
            </span>
            <StatusPill
              connected={channel.connected}
              lastAt={channel.lastAt}
              messages={channel.messages}
            />
          </div>
        }
      />

      {/* The rover reporting ros_ok:false is the single most useful thing this
          panel can say, so it gets its own row rather than a chip in a corner. */}
      {tele?.ros_ok === false && (
        <div className="mb-4 rounded-lg border border-rose-500/40 bg-rose-950/30 px-3 py-2 text-sm text-rose-100">
          <b>ros_ok = false</b> — the rover's ROS stack is not running. Motion
          commands will not be acted on.
        </div>
      )}

      {lastError && (
        <div className="mb-4 rounded-lg border border-rose-500/40 bg-rose-950/30 px-3 py-2">
          <div className="lbl text-rose-300">last error</div>
          <div className="mt-0.5 break-words font-mono text-sm text-rose-200">{lastError}</div>
        </div>
      )}

      <div className="grid gap-4 md:grid-cols-3">
        {/* Battery, big. This is the number the operator asked to see. */}
        <div
          className={`rounded-xl border p-4 md:col-span-1 ${
            battState === "LOW"
              ? "border-rose-500/50 bg-rose-950/30"
              : battState === "STALE"
                ? "border-amber-500/40 bg-amber-500/5"
                : "border-white/5 bg-black/30"
          }`}
        >
          <div className="flex items-center justify-between">
            <span className="lbl">Battery</span>
            <span className={BATT_CHIP[battState]}>{battState}</span>
          </div>
          <div className={`font-mono text-5xl font-semibold tabular-nums ${battTone(battState)}`}>
            {fmt(tele?.battery_v, 2)}
            <span className="ml-1 text-xl text-slate-500">V</span>
          </div>
          <div className="mt-2 space-y-0.5 font-mono text-[11px] text-slate-500">
            <div>raw {fmt(tele?.battery_raw, 2)}</div>
            <div>
              reading age {battAge === null ? "—" : `${battAge.toFixed(1)}s`}
              {battStale && <span className="text-amber-300"> · stale</span>}
            </div>
          </div>
        </div>

        <div className="grid grid-cols-2 gap-3 md:col-span-2">
          <Rate label="odom" hz={tele?.odom_hz} />
          <Rate label="imu" hz={tele?.imu_hz} />
          <Rate label="batt" hz={tele?.batt_hz} />
          <Readout label="µROS session" value={flag(tele?.uros_session)} />
          <Readout label="Mode" value={str(tele?.mode) ?? "—"} />
          <Readout
            label="Moving"
            value={bool(tele?.moving)}
            tone={tele?.moving === true ? "text-ember-300" : undefined}
          />
          <Readout label="X" value={`${fmt(tele?.x_mm, 0)} mm`} />
          <Readout label="Y" value={`${fmt(tele?.y_mm, 0)} mm`} />
          <Readout label="Heading" value={`${fmt(tele?.heading_deg, 1)}°`} />
          {/*
            ORIGIN. The rover publishes this and nothing on this page used to
            show it. The origin FILE survives a reboot while the board's
            odometry restarts at zero, so a stale reference has already put a
            rover at (-9182, -11926) mm inside a 1200 mm arena — and it planned
            from there, confidently. Whether an origin exists at all is the
            cheapest possible check against that.
          */}
          <Readout
            label="Origin"
            value={
              tele?.origin_set === true
                ? "set"
                : tele?.origin_set === false
                  ? "NOT SET"
                  : "—"
            }
            tone={tele?.origin_set === false ? "text-amber-300" : undefined}
            hint={
              tele?.origin_set === false
                ? "No origin — x/y are measured from wherever the bridge started, not in the arena frame the map draws in. Use Set coordinate."
                : "Whether the teleop bridge has an arena origin for its pose"
            }
          />
          <Readout
            label="cmd_vel vx / wz"
            value={`${fmt(cmdVel(tele, "vx"), 2)} / ${fmt(cmdVel(tele, "wz"), 2)}`}
          />
          <Readout label="Last cmd" value={str(tele?.last_cmd) ?? "—"} hint={lastCmdAge(tele)} />
          <Readout label="Since jog" value={ms(tele?.ms_since_jog)} />
          <Readout
            label="cmd_scale"
            value={fmt(tele?.cmd_scale, 3)}
            hint="speed calibration factor — provisional"
          />
          {/* Uptime is only interesting relative to itself: a small number here
              after a large one earlier is a power cycle. */}
          <Readout
            label={reboots > 0 ? `Uptime · restarted ×${reboots}` : "Uptime"}
            value={dur(tele?.uptime_s)}
            tone={reboots > 0 ? "text-ember-300" : undefined}
            hint={
              reboots > 0
                ? `uptime_s went backwards ${reboots} time${reboots > 1 ? "s" : ""} this session — last at ${clock(lastRebootAt)}`
                : "seconds since the rover's teleop bridge started"
            }
          />
        </div>
      </div>

      <p className="mt-4 text-xs text-slate-500">
        {tele === null
          ? `No drive telemetry received on ${thing ? `drive:${thing}` : "this channel"} yet — every field reads “—” until the rover publishes fpms/<rover>/telemetry/drive.`
          : `Last packet ${ageMs === null ? "—" : `${(ageMs / 1000).toFixed(1)}s ago`}. Every field is rendered only if it arrives as a finite number; anything missing shows as “—” rather than taking the page down with it.`}
      </p>
    </Card>
  );
}

function Rate({ label, hz }: { label: string; hz: unknown }) {
  const n = num(hz);
  const dead = n === null || n <= 0;
  return (
    <div className="rounded-lg border border-white/5 bg-black/20 px-3 py-2">
      <dt className="lbl">{label}</dt>
      <dd
        className={`mt-0.5 font-mono text-sm tabular-nums ${
          dead ? "text-slate-600" : "text-emerald-300"
        }`}
      >
        {n === null ? "—" : `${n.toFixed(1)} Hz`}
      </dd>
    </div>
  );
}

function Readout({
  label,
  value,
  tone,
  hint,
}: {
  label: string;
  value: string;
  tone?: string;
  hint?: string;
}) {
  return (
    <div className="rounded-lg border border-white/5 bg-black/20 px-3 py-2" title={hint}>
      <dt className="lbl">{label}</dt>
      <dd className={`mt-0.5 truncate font-mono text-sm tabular-nums ${tone ?? "text-slate-200"}`}>
        {value}
      </dd>
    </div>
  );
}

/* ---- small pieces -------------------------------------------------------- */

function SetCoordinate({
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
 * Arms on the first click and fires on the second, disarming after a few
 * seconds. Inline rather than a modal: the confirmation stays on the control
 * being confirmed, and an unconfirmed click simply decays.
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

  // A control that goes dead while armed must not stay armed — the next time it
  // comes back it would fire on a single click.
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
      {armed ? "click again to confirm" : label}
    </button>
  );
}

/* ---- telemetry formatting ------------------------------------------------ */
/*
 * Every one of these takes `unknown` and checks it. A previous version of this
 * dashboard called .toFixed() on a telemetry field that had not arrived yet and
 * took the whole page down with it — a missing reading must degrade to a dash,
 * never to a white screen. The ErrorBoundary above is the seatbelt; these are
 * the reason it should never be needed.
 */

/** Pull the payload out of the bridge envelope, tolerating either shape. */
function readTelemetry(env: unknown): Record<string, unknown> | null {
  if (!env || typeof env !== "object") return null;
  const data = (env as Record<string, unknown>).data;
  if (data && typeof data === "object" && !Array.isArray(data)) {
    return data as Record<string, unknown>;
  }
  return null;
}

function num(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

function fmt(v: unknown, digits: number): string {
  const n = num(v);
  return n === null ? "—" : n.toFixed(digits);
}

function str(v: unknown): string | null {
  return typeof v === "string" && v.trim() !== "" ? v : null;
}

function bool(v: unknown): string {
  return typeof v === "boolean" ? (v ? "yes" : "no") : "—";
}

/** uros_session may arrive as a flag or as a session name; both are useful. */
function flag(v: unknown): string {
  if (typeof v === "boolean") return v ? "up" : "down";
  const s = str(v);
  if (s !== null) return s;
  const n = num(v);
  return n === null ? "—" : String(n);
}

function ms(v: unknown): string {
  const n = num(v);
  return n === null ? "—" : `${Math.round(n)} ms`;
}

function dur(v: unknown): string {
  const n = num(v);
  if (n === null || n < 0) return "—";
  const s = Math.floor(n);
  if (s < 60) return `${s}s`;
  if (s < 3600) return `${Math.floor(s / 60)}m ${s % 60}s`;
  return `${Math.floor(s / 3600)}h ${Math.floor((s % 3600) / 60)}m`;
}

/** Elapsed milliseconds as something readable across six orders of magnitude. */
function agoText(v: unknown): string {
  const n = num(v);
  if (n === null) return "—";
  const s = Math.max(0, n) / 1000;
  if (s < 10) return `${s.toFixed(1)}s`;
  if (s < 60) return `${Math.round(s)}s`;
  return dur(Math.round(s));
}

/** Wall clock for an epoch-ms instant, or a dash if we never had one. */
function clock(at: unknown): string {
  const n = num(at);
  if (n === null) return "—";
  try {
    return new Date(n).toLocaleTimeString();
  } catch {
    return "—";
  }
}

function cmdVel(tele: Record<string, unknown> | null, key: "vx" | "wz"): unknown {
  const cv = tele?.cmd_vel;
  return cv && typeof cv === "object" ? (cv as Record<string, unknown>)[key] : undefined;
}

function lastCmdAt(tele: Record<string, unknown> | null): number | null {
  return num(tele?.last_cmd_at);
}

function lastCmdAge(tele: Record<string, unknown> | null): string | undefined {
  const at = lastCmdAt(tele);
  if (at === null) return undefined;
  const secs = at > 1e11 ? at / 1000 : at;
  return `at ${new Date(secs * 1000).toLocaleTimeString()}`;
}

const BATT_CHIP: Record<"OK" | "LOW" | "STALE" | "—", string> = {
  OK: "chip-ok",
  LOW: "chip-hot",
  STALE: "chip-warn",
  "—": "chip",
};

function battTone(state: "OK" | "LOW" | "STALE" | "—"): string {
  if (state === "LOW") return "text-rose-300";
  if (state === "STALE") return "text-amber-300";
  if (state === "OK") return "text-emerald-300";
  return "text-slate-500";
}

function rosClass(tele: Record<string, unknown> | null): string {
  if (!tele) return "chip";
  if (tele.ros_ok === false) return "chip-hot";
  if (tele.ros_ok === true) {
    const hz = num(tele.odom_hz);
    return hz !== null && hz > 0 ? "chip-ok" : "chip-warn";
  }
  return "chip-warn";
}

function rosLabel(tele: Record<string, unknown> | null): string {
  if (!tele) return "no telemetry";
  if (tele.ros_ok === false) return "ROS down";
  if (tele.ros_ok === true) {
    const hz = num(tele.odom_hz);
    return hz !== null && hz > 0 ? "ROS ok" : "ROS up · no odom";
  }
  return "ROS unknown";
}

/* ---- ack plumbing -------------------------------------------------------- */

const KIND_CHIP: Record<LogKind, string> = {
  sent: "chip",
  ack: "chip-ok",
  nack: "chip-hot",
  stale: "chip-warn",
  nohw: "chip-warn",
  timeout: "chip-warn",
  error: "chip-hot",
  event: "chip",
};

const KIND_LABEL: Record<LogKind, string> = {
  sent: "waiting",
  ack: "ack",
  nack: "refused",
  stale: "agent too old",
  nohw: "no hardware",
  timeout: "no reply",
  error: "not sent",
  event: "event",
};

type Outcome = { kind: LogKind; text: string; hint?: string };

function describeAck(data: Record<string, any>): Outcome {
  const detail = data.result ?? data.detail ?? data.message;
  return { kind: "ack", text: detail == null ? "acknowledged" : compact(detail) };
}

/**
 * Two rover replies are not dashboard faults and must not read like one: an
 * un-updated agent answers "unknown command" — likely for the actions this page
 * introduces — and a rover with no motor driver answers "no motor interface".
 */
function describeNack(data: Record<string, any>): Outcome {
  const raw = String(data.error ?? data.reason ?? data.message ?? "");
  const err = raw.toLowerCase();
  if (err.includes("unknown command")) {
    return { kind: "stale", text: "rover agent too old — command not supported", hint: raw };
  }
  if (err.includes("no motor interface")) {
    return { kind: "nohw", text: "no motor hardware", hint: raw };
  }
  return { kind: "nack", text: raw ? `refused — ${raw}` : "refused", hint: raw };
}

function describeParams(params: Record<string, unknown>): string {
  const keys = Object.keys(params);
  if (keys.length === 0) return "sent";
  return keys.map((k) => `${k}=${compact(params[k])}`).join(" ");
}

/** MQTT timestamps are epoch seconds; tolerate a millisecond one anyway. */
function normalizeTs(ts: unknown): number | null {
  if (typeof ts !== "number" || !Number.isFinite(ts)) return null;
  return ts > 1e11 ? ts / 1000 : ts;
}

/** Clamp a stick axis into −1…1, dropping anything non-finite. */
function unit(v: number): number {
  if (!Number.isFinite(v)) return 0;
  const clamped = v > 1 ? 1 : v < -1 ? -1 : v;
  return Math.round(clamped * 1000) / 1000;
}

function compact(v: unknown): string {
  if (typeof v === "string") return v;
  try {
    return JSON.stringify(v) ?? String(v);
  } catch {
    return String(v);
  }
}

function errText(e: unknown): string {
  return e instanceof Error ? e.message : String(e);
}

/**
 * What the rover is doing to itself, right now.
 *
 * The hard rule here is that this card must never imply motion it cannot
 * substantiate, and never imply stillness it cannot substantiate either. Both
 * directions have a cost, but they are not symmetric: an operator who believes
 * a moving rover is parked will walk up to it. So staleness is surfaced loudly,
 * unknown phases count as running, and absent numbers render as "--" rather
 * than as zero.
 */
function MissionCard({
  thing,
  mission,
  channel,
  ageMs,
  fallbackPose,
  onAbort,
}: {
  thing: string | null;
  mission: MissionState | null;
  channel: { connected: boolean; lastAt: number | null; messages: number };
  ageMs: number | null;
  fallbackPose: { x_mm: number | null; y_mm: number | null; heading_deg: number | null };
  onAbort?: () => void;
}) {
  const stale = ageMs !== null && ageMs > MISSION_STALE_MS;
  const running = !!mission?.running;

  // Stale telemetry from a RUNNING mission is the dangerous case: the last
  // thing we heard was "driving", and we have not heard since.
  const blind = running && stale;

  const seg =
    mission?.segmentI !== null && mission?.segmentI !== undefined && mission?.segmentsN
      ? `${mission.segmentI}/${mission.segmentsN}`
      : "--";

  return (
    <Card>
      <CardHeader
        title="Mission"
        subtitle={
          mission?.mission
            ? `${mission.mission}${mission.backend ? ` · via ${mission.backend}` : ""}`
            : "Nothing running"
        }
        right={
          <div className="flex items-center gap-2">
            {blind ? (
              <span className="chip-hot" title="Last known state was DRIVING, and telemetry has since stopped">
                telemetry lost while driving
              </span>
            ) : stale ? (
              <span className="chip" title="Mission telemetry is old">
                stale {Math.round((ageMs ?? 0) / 1000)}s
              </span>
            ) : null}
            {/*
              The phase chip is coloured by what it means. An idle-looking chip
              on a driving rover is the failure this whole card exists to
              prevent, and an unrecognised phase counts as RUNNING (see
              MISSION_IDLE_PHASES) so a new or truncated phase name errs towards
              the safe direction rather than reading as parked.
            */}
            <span
              className={running ? "chip-hot font-mono" : "chip font-mono"}
              title={
                running
                  ? "The executor is driving, or reported a phase this dashboard does not recognise — either way, treat it as moving"
                  : "The executor reported a phase that positively means not-driving"
              }
            >
              {mission?.phase ?? (mission ? "phase not reported" : "no mission state")}
            </span>
            <StatusPill
              connected={channel.connected}
              lastAt={channel.lastAt}
              messages={channel.messages}
            />
          </div>
        }
      />

      {!channel.messages ? (
        <p className="text-sm text-slate-400">
          No mission telemetry has arrived on this channel. Either the executor
          is not running on {thing ?? "this rover"} (
          <span className="font-mono">systemctl status fpms-missions</span>), or
          it cannot reach the broker.
        </p>
      ) : (
        <>
          <div className="grid grid-cols-2 gap-x-4 gap-y-2 text-sm sm:grid-cols-4">
            <Stat label="Phase" value={mission?.phase ?? "--"} />
            {/*
              LEG vs SEGMENT are different questions and the card answers both.
              A leg is one target on a multi-corner route; a segment is one
              turn-or-drive within it. `leg_label` is the rover's own
              plain-English corner name — the mission ids do not match what an
              operator says out loud, so the label is what prevents a run being
              read as going to the opposite zone.
            */}
            <Stat
              label="Leg"
              value={
                mission?.legLabel || mission?.leg
                  ? `${mission.legLabel ?? mission.leg}${
                      mission?.legI !== null && mission?.legI !== undefined && mission?.legsN
                        ? ` · ${mission.legI}/${mission.legsN}`
                        : ""
                    }`
                  : "--"
              }
            />
            <Stat label="Segment" value={seg} />
            <Stat
              label="Remaining"
              value={mission?.remainingMm !== null && mission?.remainingMm !== undefined
                ? `${Math.round(mission.remainingMm)} mm`
                : "--"}
            />
            <Stat
              label="Travelled"
              value={mission?.travelledMm !== null && mission?.travelledMm !== undefined
                ? `${Math.round(mission.travelledMm)} mm`
                : "--"}
            />
            {/* Null-checked, not truthiness-checked: `0 s elapsed` is a real
                reading a second into a run, and `?` would have hidden it. */}
            <Stat
              label="Elapsed"
              value={mission?.elapsedS !== null && mission?.elapsedS !== undefined
                ? `${Math.round(mission.elapsedS)} s`
                : "--"}
            />
            <Stat
              label="ETA"
              value={mission?.etaS !== null && mission?.etaS !== undefined
                ? `~${Math.round(mission.etaS)} s`
                : "--"}
            />
            <Stat
              label="Battery"
              value={mission?.battV !== null && mission?.battV !== undefined
                ? `${mission.battV.toFixed(1)} V`
                : "--"}
            />
            <Stat
              label="Front clear"
              value={mission?.frontMm !== null && mission?.frontMm !== undefined
                ? `${Math.round(mission.frontMm)} mm`
                : "--"}
            />
          </div>

          <div className="mt-3 flex flex-wrap items-center gap-2 text-[11px] text-slate-400">
            {/* BOTH halves are required. `targetY ?? 0` used to fill a missing
                y with the origin, which drew the destination on an arena edge
                the rover was never sent to — a fabricated coordinate is worse
                than an absent one. */}
            <span className="font-mono">
              target{" "}
              {mission?.targetX !== null && mission?.targetX !== undefined &&
              mission?.targetY !== null && mission?.targetY !== undefined
                ? `(${Math.round(mission.targetX)}, ${Math.round(mission.targetY)}) mm`
                : "--"}
            </span>
            {/*
              The executor's OWN pose, with the teleop bridge as the fallback.
              Both are omitted rather than zero-filled when there is no
              odometry, so the pair is checked together — an x without a y is
              not a position, and rendering the missing half as 0 would place
              the rover on an arena edge it has never been near.
            */}
            <span className="font-mono">
              pose{" "}
              {mission?.poseX !== null && mission?.poseX !== undefined &&
              mission?.poseY !== null && mission?.poseY !== undefined
                ? `(${Math.round(mission.poseX)}, ${Math.round(mission.poseY)}) mm · executor`
                : fallbackPose.x_mm !== null && fallbackPose.y_mm !== null
                  ? `(${Math.round(fallbackPose.x_mm)}, ${Math.round(fallbackPose.y_mm)}) mm · bridge`
                  : "--"}
            </span>
            {mission?.poseHeadingDeg !== null && mission?.poseHeadingDeg !== undefined ? (
              <span className="font-mono">
                heading {Math.round(mission.poseHeadingDeg)}°
              </span>
            ) : null}
            {mission?.odomSource ? (
              <span className="chip font-mono">odom · {mission.odomSource}</span>
            ) : null}
            {mission?.linkOk === false ? (
              <span className="chip-hot">micro-ROS link down</span>
            ) : null}
            {mission?.lidarOk === false ? (
              <span className="chip-hot">LiDAR stale — obstacle guard blind</span>
            ) : null}
          </div>

          {mission?.poseAssumed ? (
            <div className="mt-3 rounded-lg border border-amber-500/40 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90">
              <b>Pose is assumed, not measured.</b> Nothing localises this rover,
              so its position is dead-reckoned from an assumed start. Every
              coordinate on this card and every route on the map inherits that
              assumption — if the rover did not start where the dashboard thinks,
              all of it is offset by the same amount.
            </div>
          ) : null}

          {blind ? (
            <div className="mt-3 rounded-lg border border-rose-500/40 bg-rose-500/5 px-3 py-2 text-xs text-rose-100/90">
              <b>The rover was driving when telemetry stopped.</b> Treat it as
              still moving until you can see otherwise. Stop it before
              approaching.
            </div>
          ) : null}
        </>
      )}

      {onAbort ? (
        <div className="mt-4 flex flex-wrap items-center gap-3">
          <button
            className="btn-hot"
            onClick={onAbort}
            disabled={!thing}
            title={MISSION_ABORT_NOTE}
          >
            ABORT MISSION
          </button>
          <span className="text-[11px] text-slate-500">
            Ungated by the link check — an abort has to work exactly when
            everything else has decided things are wrong.
          </span>
        </div>
      ) : null}
    </Card>
  );
}

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div>
      <div className="lbl">{label}</div>
      <div className="font-mono text-slate-200">{value}</div>
    </div>
  );
}

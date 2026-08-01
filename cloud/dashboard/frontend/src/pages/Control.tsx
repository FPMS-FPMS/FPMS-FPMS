import { useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import { Card, CardHeader } from "../components/Card";
import { StatusPill } from "../components/StatusPill";
import { ErrorBoundary } from "../components/ErrorBoundary";
import { MissionStrip } from "../components/MissionStrip";
import { useChannel } from "../lib/ws";
import { apiPostJson } from "../lib/api";
import { useThings } from "../lib/things";
import { useMission } from "../lib/mission";
import {
  emptyCapabilities,
  mergeCapabilities,
  useRoverCapabilities,
} from "../lib/capabilities";

/**
 * Direct rover control. Every button here publishes a command to a physical
 * machine, so the page is built around one rule: the operator must always be
 * able to see what the rover said back. A command that silently succeeds in the
 * UI while the rover never heard it is worse than an error.
 *
 * The command set is split across three on-rover services — fpms-teleop owns
 * motion, beeper, servos and the micro-ROS link on ROS_DOMAIN_ID=20;
 * fpms-missions owns autonomous runs; fpms-rover-agent owns the sensors and the
 * service lifecycle — but that split is not the operator's problem and is
 * deliberately invisible in the grouping below. Commands are grouped by what
 * they DO, not by which daemon happens to answer them. The one place the split
 * IS shown is the Command set card, because that card's entire job is to say
 * which process owes you a reply.
 *
 * THE BUTTON SET IS CROSS-CHECKED AGAINST THE ROVER, NOT ASSUMED.
 * fpms-teleop publishes its real `TELEOP_ACTIONS` on events/online and in every
 * drive_status reply, with a comment saying the dashboard should build its
 * buttons from that rather than from a hardcoded list that can drift. This page
 * still owns the parameter UI for each verb — a generic "send this verb"
 * control could not offer a clamped millimetre box — but the CATALOG below is
 * diffed against what the rover advertises, both ways, and any disagreement is
 * reported on screen instead of being discovered one timeout at a time.
 */

/**
 * Every verb the dashboard knows about, with which tab owns its control.
 *
 * This is the dashboard's half of the drift check. The rover's half arrives
 * over MQTT. The diff between them is rendered in the Command set card.
 *
 * `blindSafe` means: is it safe to fire this verb with NO parameters at all?
 * It gates the auto-generated buttons offered for verbs the rover advertises
 * that this page has never heard of. The default for anything unknown is false
 * — an unrecognised verb gets named on screen but not wired to a button,
 * because "the rover advertised it" is not evidence that firing it blind is
 * harmless. `set_coordinate` is the worked example: an empty payload could be
 * read as "you are at the origin", silently invalidating every later mission.
 */
type Owner = "control" | "drive" | "none";

const CATALOG: Record<
  string,
  { owner: Owner; label: string; blindSafe: boolean; note?: string }
> = {
  // -- fpms-teleop, motion
  stop: { owner: "control", label: "Stop", blindSafe: true },
  estop: {
    owner: "control",
    label: "E-stop",
    blindSafe: true,
    note: "fpms-teleop routes stop, estop and auto_off through one ungated handler — this is a second name for the same halt, not a stronger one.",
  },
  jog: {
    owner: "drive",
    label: "Jog (stick)",
    blindSafe: false,
    note: "A streamed analog command; it needs the joystick on the Drive tab, not a button.",
  },
  nudge: { owner: "control", label: "Nudge", blindSafe: false },
  turn: { owner: "control", label: "Turn", blindSafe: false },
  test_motors: { owner: "control", label: "Test motors", blindSafe: false },
  auto_off: { owner: "control", label: "Autonomy OFF", blindSafe: true },
  // -- fpms-teleop, actuators and reads
  beep: { owner: "control", label: "Beep", blindSafe: false },
  servo: { owner: "control", label: "Servo", blindSafe: false },
  read_encoders: { owner: "control", label: "Pose / odometry", blindSafe: true },
  set_speed: { owner: "control", label: "Apply speeds", blindSafe: false },
  drive_status: { owner: "control", label: "Bridge snapshot", blindSafe: true },
  drive_connect: { owner: "control", label: "Bridge stream on", blindSafe: true },
  drive_disconnect: { owner: "control", label: "Bridge stream off", blindSafe: true },
  // -- fpms-missions / fpms-teleop, autonomy
  mission: {
    owner: "drive",
    label: "Missions",
    blindSafe: false,
    note: "Named routes with a plan-first preview; lives on the Drive tab beside the map that shows the route.",
  },
  set_coordinate: {
    owner: "drive",
    label: "Set coordinate",
    blindSafe: false,
    note: "Needs an x/y in arena mm. Firing it empty could be read as 'you are at the origin', which would silently offset every mission afterwards.",
  },
  // -- fpms-rover-agent
  ping: { owner: "control", label: "Ping", blindSafe: true },
  status: { owner: "control", label: "Status", blindSafe: true },
  connect: { owner: "control", label: "Connect", blindSafe: true },
  disconnect: { owner: "control", label: "Disconnect", blindSafe: true },
  restart: { owner: "control", label: "Restart agent", blindSafe: true },
  auto_on: { owner: "control", label: "Autonomy ON", blindSafe: true },
};

/**
 * Verbs no rover process subscribes, and what to do instead.
 *
 * fpms-teleop states this itself in its drive_status reply (`not_owned_here`)
 * and fpms-rover-agent answers such a verb with a nack that names the owner.
 * Repeating it here means the operator is told BEFORE pressing rather than
 * three seconds afterwards; when the rover's own list arrives it takes
 * precedence, because the rover is the authority on its own commands.
 */
const UNOWNED_FALLBACK: Record<string, string> = {
  auto_on:
    "no autonomy loop exists on this rover — fpms-rover-agent nacks it and names the owner. Autonomous runs are the `mission` command on the Drive tab.",
};

/**
 * Verbs that move the rover. These are the ones that stay confirm-gated while a
 * mission is running: the executor is driving, a manual motion command fights
 * it for the same wire, and from this page a rover on blocks and a rover
 * crossing a room look identical.
 */
const MOTION_VERBS = new Set(["nudge", "turn", "test_motors", "jog", "mission"]);

type Action =
  // Motion — these move the rover.
  | "stop"
  | "estop" // the fleet e-stop below deliberately publishes `stop`; see stopAll
  | "test_motors"
  | "nudge"
  | "turn"
  // Diagnostics, actuators and state — no motion.
  | "read_encoders"
  | "ping"
  | "status"
  | "beep"
  | "servo"
  | "set_speed"
  // fpms-teleop's own status and stream verbs, distinctly named so they do not
  // shadow fpms-rover-agent's ping/status/connect/disconnect.
  | "drive_status"
  | "drive_connect"
  | "drive_disconnect"
  // Mode, stream and lifecycle.
  | "auto_on"
  | "auto_off"
  | "connect"
  | "disconnect"
  | "restart"
  // Anything the rover advertises that this file predates. Widening the type
  // rather than casting at the call site keeps the union above useful for
  // autocomplete while making the drift path a first-class, typed one.
  | (string & {});

/** How long we wait for an events/ack before calling it unanswered. */
const ACK_TIMEOUT_MS = 3000;
const LOG_LIMIT = 50;

/**
 * CLIENT-SIDE CLAMPS.
 *
 * These mirror the caps the rover enforces; they are NOT the authority and are
 * not trusted as one — the rover clamps or refuses independently, and its ack
 * reports what it actually used. What they are is a typo guard: a stray zero in
 * a millimetre box must not become a metre of unplanned travel across a
 * heritage site before the far end gets a chance to argue.
 *
 * Every limit here is stated in the UI next to the input it governs, because a
 * silent clamp teaches the operator that the dial does nothing.
 */
const LIM = {
  /** fpms-teleop refuses anything over a metre outright. */
  nudgeMm: { min: 1, max: 1000, def: 100, step: 10, digits: 0 },
  /** Closed-loop on the integrated gyro; a full turn is the cap. */
  turnDeg: { min: 1, max: 360, def: 15, step: 5, digits: 0 },
  /**
   * Real m/s, both ends of the envelope load-bearing. Below the 0.035 m/s
   * deadband the firmware stalls, winds up its integrator and then lurches in a
   * direction that is not reliably the commanded one; 0.12 m/s is the hard cap
   * the drive code was reviewed against and is not tunable from anywhere.
   */
  speedMps: { min: 0.035, max: 0.12, def: 0.05, step: 0.005, digits: 3 },
  /** Per-leg self-test duration. */
  testS: { min: 0.3, max: 3.0, def: 1.0, step: 0.1, digits: 1 },
  /** 0 means silence; see clampBeepMs for the 1..9 firmware quirk. */
  beepMs: { min: 0, max: 2000, def: 200, step: 50, digits: 0 },
  /** Inset from the firmware's 0..180 so the horn never parks on a stop. */
  servoDeg: { min: 10, max: 170, def: 90, step: 5, digits: 0 },
} as const;

type Limit = { min: number; max: number; def: number; step: number; digits: number };

type LogKind =
  | "sent"      // in flight, waiting on the rover
  | "ack"       // rover confirmed
  | "nack"      // rover refused, generic
  | "stale"     // rover agent predates this command
  | "nohw"      // rover has no motor interface wired up yet
  | "timeout"   // nothing came back within ACK_TIMEOUT_MS
  | "error"     // the POST itself failed — never reached MQTT
  | "event";    // unsolicited rover event

type Field = { k: string; v: string };

type LogItem = {
  id: string;
  at: number;
  thing: string;
  action?: string;
  kind: LogKind;
  text: string;
  hint?: string;
  /** Parsed reply fields, rendered as chips rather than dumped as JSON. */
  fields?: Field[];
};

/**
 * Replies that arrive on their own topic instead of events/ack. Without this
 * map a `ping` would sit in the log as "no reply" for three seconds while its
 * pong was already on screen as an unrelated event — the reply names itself by
 * topic, not by an `action` field.
 */
const SUBTYPE_ACTION: Record<string, string> = {
  pong: "ping",
  encoders: "read_encoders",
  status: "status",
  motor_test: "test_motors",
  // fpms-teleop answers drive_status on its own topic, exactly as it does for
  // encoders. Without this entry the request would sit at "no reply" for three
  // seconds with its answer already on screen as an unrelated event.
  drive_status: "drive_status",
};

export default function Control() {
  const things = useThings();
  // Always offer both bays so a rover that has not reported yet is visibly
  // present rather than missing from the selector (same reasoning as Camera).
  const bays = useMemo(
    () => Array.from(new Set([...things, "rover1", "rover2"])).sort().slice(0, 4),
    [things.join(",")], // eslint-disable-line react-hooks/exhaustive-deps
  );

  const [target, setTarget] = useState<string>("BOTH");
  const targets = target === "BOTH" ? bays : [target];

  // What the rovers say they support. Accumulated off the same events stream
  // the log below reads; see lib/capabilities for why it is additive and why an
  // empty list must never be rendered as "unsupported".
  const capsByThing = useRoverCapabilities();
  const caps = useMemo(
    () => (targets.length ? mergeCapabilities(capsByThing, targets) : emptyCapabilities()),
    [capsByThing, targets.join(",")], // eslint-disable-line react-hooks/exhaustive-deps
  );

  // Mission state for the SELECTED rover. In BOTH mode there is no single
  // mission to show, so the strip subscribes to the first bay and says which —
  // an aggregate "something somewhere is running" would be worse than useless.
  const missionThing = target === "BOTH" ? bays[0] ?? null : target;
  const missionFeed = useMission(missionThing);
  // Unknown phases count as running (lib/mission), so this errs towards
  // treating the rover as in motion — which is the direction that is safe to be
  // wrong in.
  const missionActive = !!missionFeed.mission?.running || missionFeed.blind;
  const motionHint = missionActive
    ? "A mission is driving this rover — this command contends with the executor and will be overwritten within a tick. Abort the mission to take over."
    : undefined;

  // The rover's own word on why a verb lands nowhere beats ours, when we have
  // it. Ours is only a fallback for the case where nothing has announced yet.
  const autoOnNote = caps.notOwned["auto_on"] ?? UNOWNED_FALLBACK["auto_on"];

  /**
   * The drift check, both directions.
   *
   * `advertisedNoControl` is the one that would otherwise be invisible: a verb
   * the rover gained that no button reaches. `controlNotAdvertised` is the
   * softer signal — it is expected while `caps.announced` is false, and only
   * means something once a rover HAS told us its list.
   */
  const drift = useMemo(() => {
    const advertised = Array.from(caps.actions).sort();
    const known = new Set(Object.keys(CATALOG));
    const advertisedNoControl = advertised.filter(
      (a) => !known.has(a) || CATALOG[a].owner === "none",
    );
    // Verbs this page has a button for, checked against the announcement. Drive
    // tab verbs are excluded: they are reachable, just not from here.
    const controlNotAdvertised = Object.entries(CATALOG)
      .filter(([a, c]) => c.owner === "control" && !caps.actions.has(a))
      .map(([a]) => a)
      .sort();
    return { advertised, advertisedNoControl, controlNotAdvertised };
  }, [caps]);

  const [log, setLog] = useState<LogItem[]>([]);
  const [seen, setSeen] = useState<Record<string, { lastAt: number; count: number }>>({});

  // Parameter inputs. Held as strings so a half-typed number is not fought with
  // mid-keystroke; they are clamped to LIM on the way out, and the clamped
  // value is written back so the box always shows what was actually sent.
  const [nudgeMm, setNudgeMm] = useState(String(LIM.nudgeMm.def));
  const [turnDeg, setTurnDeg] = useState(String(LIM.turnDeg.def));
  const [testSpeed, setTestSpeed] = useState(String(LIM.speedMps.def));
  const [testDur, setTestDur] = useState(String(LIM.testS.def));
  const [beepMs, setBeepMs] = useState(String(LIM.beepMs.def));
  const [servoWhich, setServoWhich] = useState<1 | 2>(1);
  const [servoAngle, setServoAngle] = useState(String(LIM.servoDeg.def));
  const [jogMax, setJogMax] = useState(String(LIM.speedMps.def));
  const [nudgeSpeed, setNudgeSpeed] = useState("0.04");

  const ev = useChannel<any>("events");
  // The hub replays its last broadcast to every new subscriber, so on mount we
  // would otherwise show a stale ack from minutes ago as if it just arrived.
  const mountedAt = useRef(Date.now() / 1000);
  // key: `${thing}:${action}` → the log row waiting to be resolved by an ack.
  // sentAt is kept for `ping`: it is the fallback round-trip measurement when
  // the echoed timestamp comes back unusable.
  const pending = useRef(new Map<string, { id: string; timer: number; sentAt: number }>());

  const pushLog = (item: LogItem) =>
    setLog((l) => [item, ...l].slice(0, LOG_LIMIT));
  const patchLog = (id: string, patch: Partial<LogItem>) =>
    setLog((l) => l.map((e) => (e.id === id ? { ...e, ...patch } : e)));

  const fire = (action: Action, to: string[], params: Record<string, unknown> = {}) => {
    for (const thing of to) {
      const key = `${thing}:${action}`;
      const id = `${key}:${Date.now()}:${Math.random().toString(36).slice(2, 7)}`;

      // A repeat of the same command supersedes the one still in flight —
      // otherwise the older row would resolve against the newer rover reply.
      const prev = pending.current.get(key);
      if (prev) window.clearTimeout(prev.timer);

      const timer = window.setTimeout(() => {
        if (pending.current.get(key)?.id !== id) return;
        pending.current.delete(key);
        patchLog(id, { kind: "timeout", text: "no reply" });
      }, ACK_TIMEOUT_MS);
      pending.current.set(key, { id, timer, sentAt: Date.now() });

      pushLog({
        id, at: Date.now(), thing, action,
        kind: "sent",
        // Showing the parameters as sent is how a clamp becomes visible: the
        // row says mm=1000 when 10000 was typed.
        text: paramSummary(params),
      });

      apiPostJson<unknown>(`/api/control/${thing}/${action}`, { params }).catch((e: unknown) => {
        const p = pending.current.get(key);
        if (p?.id === id) {
          window.clearTimeout(p.timer);
          pending.current.delete(key);
        }
        patchLog(id, { kind: "error", text: `not sent — ${errText(e)}` });
      });
    }
  };

  /**
   * Fleet emergency stop. This deliberately ignores the rover selector: an
   * e-stop that only halts the rover you happen to have selected is a trap —
   * the moment you need it is the moment you have not checked which chip is
   * highlighted.
   */
  const stopAll = () => fire("stop", bays);

  // Escape is bound to the e-stop, so the reflex that closes a dialog also
  // halts the fleet. Kept in a ref so the listener never goes stale as `bays`
  // fills in from the roster poll.
  const stopRef = useRef(stopAll);
  stopRef.current = stopAll;
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") stopRef.current();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  // --- parameterised senders. Each clamps, echoes the clamped value back into
  // its input, and only then fires.
  const sendNudge = (dir: "fwd" | "back") => {
    const mm = commit(nudgeMm, LIM.nudgeMm, setNudgeMm);
    fire("nudge", targets, { dir, mm });
  };
  const sendTurn = (dir: "left" | "right") => {
    const deg = commit(turnDeg, LIM.turnDeg, setTurnDeg);
    fire("turn", targets, { dir, deg });
  };
  const sendTestMotors = () => {
    const speed = commit(testSpeed, LIM.speedMps, setTestSpeed);
    const duration_s = commit(testDur, LIM.testS, setTestDur);
    fire("test_motors", targets, { speed, duration_s });
  };
  const sendBeep = () => {
    const ms = clampBeepMs(beepMs);
    setBeepMs(String(ms));
    fire("beep", targets, { ms });
  };
  const sendServo = () => {
    const angle = commit(servoAngle, LIM.servoDeg, setServoAngle);
    fire("servo", targets, { which: servoWhich, angle });
  };
  const sendSetSpeed = () => {
    const jog = commit(jogMax, LIM.speedMps, setJogMax);
    // The rover refuses a nudge speed above jog_max (jog_max is the hard clamp,
    // so the nudge would be clipped to it anyway). Holding the pair consistent
    // here turns that refusal into something the operator never has to see.
    const nudge = round(
      Math.min(clampTo(nudgeSpeed, LIM.speedMps), jog),
      LIM.speedMps.digits,
    );
    setJogMax(String(jog));
    setNudgeSpeed(String(nudge));
    fire("set_speed", targets, { jog_max: jog, nudge });
  };
  // The echoed timestamp is what the round-trip is measured from.
  const sendPing = () => fire("ping", targets, { t: Date.now() });

  /**
   * Abort the running mission on the rover the strip is showing.
   *
   * Goes out as `mission {name: "abort"}` — the same shape the Drive tab uses —
   * and is never gated on anything. A stop alone is not enough against the
   * executor: a mission driving a segment republishes cmd_vel continuously, so
   * a single halt is overwritten by the next tick.
   */
  const abortMission = (to: string[]) => fire("mission", to, { name: "abort" });

  // Inbound rover traffic. useChannel only keeps the latest envelope, so the
  // message counter is what tells us a new one landed.
  useEffect(() => {
    const env = ev.data;
    if (!env || typeof env !== "object") return;
    const ts = normalizeTs(env.ts);
    if (ts !== null && ts < mountedAt.current) return; // replayed, not new

    const thing = String(env.thing ?? "?");
    const subtype = String(env.subtype ?? "");
    const data = (env.data ?? {}) as Record<string, any>;
    // An ack names its own action; a pong/encoders/status/motor_test reply is
    // identified by the topic it arrived on instead.
    const action =
      typeof data.action === "string" ? data.action : SUBTYPE_ACTION[subtype];

    setSeen((s) => ({
      ...s,
      [thing]: { lastAt: Date.now(), count: (s[thing]?.count ?? 0) + 1 },
    }));

    const key = action ? `${thing}:${action}` : null;
    const p = key ? pending.current.get(key) : undefined;
    const outcome = describeReply(subtype, data, p?.sentAt);

    if (outcome && key && p) {
      window.clearTimeout(p.timer);
      pending.current.delete(key);
      patchLog(p.id, outcome);
      return;
    }

    // Anything we did not send — another operator's ack, an online notice, a
    // fire alert, or the second half of a two-stage motion ack — still belongs
    // in the log. Silence is the thing to avoid.
    pushLog({
      id: `${thing}:${subtype}:${Date.now()}:${Math.random().toString(36).slice(2, 7)}`,
      at: Date.now(),
      thing,
      action,
      ...(outcome ?? { kind: "event" as LogKind, text: subtype || "event" }),
    });
  }, [ev.messages]); // eslint-disable-line react-hooks/exhaustive-deps

  // Clear every pending timer if the operator navigates away mid-command.
  useEffect(() => {
    const map = pending.current;
    return () => {
      map.forEach((p) => window.clearTimeout(p.timer));
      map.clear();
    };
  }, []);

  return (
    <div className="space-y-5">
      <div className="flex items-end justify-between">
        <div>
          <div className="lbl">Control · direct rover commands</div>
          <h1 className="h-page mt-1">Drive, diagnose and halt the fleet</h1>
        </div>
        <div className="text-xs text-slate-500">
          {things.length
            ? `${things.length} rover${things.length > 1 ? "s" : ""} reporting: ${things.join(", ")}`
            : "no rovers reporting"}
        </div>
      </div>

      {/*
        Emergency stop. Sticky so it never scrolls out of reach, never disabled
        (a stop is idempotent — a second one costs nothing, and greying it out
        while the first is in flight removes the control exactly when the first
        one might be the one that failed), and deliberately un-gated by any
        confirmation modal: a dialog on an e-stop adds a click during an
        emergency and trains the dismiss reflex, and an accidental stop is the
        safe outcome. Offset clears the sticky NavBar above it.
      */}
      <div className="sticky top-[86px] z-20 md:top-[58px]">
        <button
          onClick={stopAll}
          className="flex w-full items-center justify-center gap-3 rounded-xl border border-rose-500/50 bg-rose-600/25 px-4 py-5 text-lg font-semibold tracking-wide text-rose-50 shadow-lg shadow-black/50 backdrop-blur transition hover:bg-rose-600/40 active:scale-[0.995]"
          title="Emergency stop — halts every rover. Shortcut: Esc"
        >
          <span className="inline-block h-3 w-3 rounded-full bg-rose-400 pulse-dot text-rose-400" />
          EMERGENCY STOP — ALL ROVERS
          <span className="rounded border border-rose-300/30 bg-black/30 px-1.5 py-0.5 font-mono text-[11px] font-normal text-rose-200">
            Esc
          </span>
        </button>
      </div>

      {/*
        MISSION STATE, ON THIS TAB TOO.
        This page can fire test_motors and a nudge, both of which fight the
        mission executor for the same /cmd_vel wire. Before this strip existed,
        the only way to know a mission was running was to be on the Drive tab —
        so an operator could arrive here, see a page full of enabled buttons,
        and have no indication that the rover was already crossing the room.
      */}
      <MissionStrip
        thing={missionThing}
        feed={missionFeed}
        onAbort={missionThing ? () => abortMission([missionThing]) : undefined}
      />
      {target === "BOTH" && missionThing && bays.length > 1 && (
        <div className="-mt-3 text-xs text-slate-500">
          Mission state above is <span className="font-mono">{missionThing}</span> only — with
          BOTH selected there is no single mission to report. Pick a bay to watch the other.
        </div>
      )}

      {/* A running mission is not a reason to disable the controls — an
          operator interrupting a mission by hand is a legitimate and sometimes
          urgent thing to do. It IS a reason to say so and to make every motion
          verb ask twice, which is what missionActive does below. */}
      {missionActive && (
        <div className="flex items-start gap-3 rounded-xl border-2 border-ember-500/40 bg-ember-500/10 p-4">
          <span className="mt-1 inline-block h-3 w-3 shrink-0 rounded-full bg-ember-400 pulse-dot text-ember-400" />
          <div className="text-sm text-ember-100">
            <b>A mission is driving this rover right now.</b> Manual motion
            commands from this page contend with the executor for the same{" "}
            <span className="font-mono">/cmd_vel</span> publisher, and the
            executor republishes at its control rate — so a nudge will be
            overwritten within a tick rather than taking effect. Every motion
            control below is confirm-gated while this is true. To take over,
            abort the mission first.
          </div>
        </div>
      )}

      {/* Target selector */}
      <Card>
        <CardHeader
          title="Command target"
          subtitle="Applies to everything below except the e-stop"
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
          {[...bays, "BOTH"].map((b) => (
            <button
              key={b}
              onClick={() => setTarget(b)}
              className={`chip font-mono ${
                target === b ? "border-ember-500/40 bg-ember-500/10 text-ember-200" : ""
              }`}
              title={b === "BOTH" ? "Send to every bay" : things.includes(b) ? "reporting" : "not reporting"}
            >
              {b === "BOTH" ? "BOTH" : b}
              {b !== "BOTH" && !things.includes(b) && (
                <span className="text-[10px] text-slate-500">· silent</span>
              )}
            </button>
          ))}
        </div>
        <p className="mt-3 text-xs text-slate-500">
          Commands publish to{" "}
          <code className="rounded bg-black/50 px-1 py-0.5 font-mono text-[11px]">
            /api/control/&lt;rover&gt;/&lt;action&gt;
          </code>{" "}
          and are answered on{" "}
          <code className="rounded bg-black/50 px-1 py-0.5 font-mono text-[11px]">
            fpms/&lt;rover&gt;/events/#
          </code>. Unanswered after {ACK_TIMEOUT_MS / 1000}s is reported as no reply.
          Numeric inputs are clamped here before sending, and the rover clamps
          again on arrival — the ack reports the value it actually used.
        </p>
      </Card>

      <div className="grid gap-5 lg:grid-cols-2">
        {/* ---------------------------------------------------------- Motion */}
        <Card>
          <CardHeader title="Motion" subtitle="These move the rover" />
          <div className="space-y-3">
            <Row label="Halt">
              <CommandButton
                className="btn-danger"
                label="Stop selected"
                onFire={() => fire("stop", targets)}
              />
              {/*
                `estop` is a separate verb in fpms-teleop's TELEOP_ACTIONS, and
                a verb the dashboard advertises no way to send is a verb that
                may as well not exist. It is offered here rather than promoted
                to the big red bar because teleop routes stop, estop and
                auto_off through ONE ungated handler: this is a second name for
                the same halt, not a stronger one, and dressing it up as a
                bigger stop would be a safety claim the rover does not make.
              */}
              <CommandButton
                className="btn-danger"
                label="E-stop verb"
                title="Publishes `estop`. fpms-teleop handles stop/estop/auto_off through one ungated path — same halt, different name."
                onFire={() => fire("estop", targets)}
              />
              <CommandButton
                label="Abort mission"
                title="mission {name: abort} — stops the executor. A plain stop is not enough: a driving mission republishes cmd_vel every tick."
                onFire={() => abortMission(targets)}
              />
            </Row>

            <Row label="Nudge">
              <CommandButton
                label="Forward"
                confirm={missionActive}
                title={motionHint}
                onFire={() => sendNudge("fwd")}
              />
              <CommandButton
                label="Back"
                confirm={missionActive}
                title={motionHint}
                onFire={() => sendNudge("back")}
              />
              <NumField
                label="step"
                unit="mm"
                value={nudgeMm}
                onChange={setNudgeMm}
                lim={LIM.nudgeMm}
              />
            </Row>

            <Row label="Turn">
              <CommandButton
                label="Left"
                confirm={missionActive}
                title={motionHint}
                onFire={() => sendTurn("left")}
              />
              <CommandButton
                label="Right"
                confirm={missionActive}
                title={motionHint}
                onFire={() => sendTurn("right")}
              />
              <NumField
                label="angle"
                unit="°"
                value={turnDeg}
                onChange={setTurnDeg}
                lim={LIM.turnDeg}
              />
            </Row>

            <Row label="Self-test">
              <CommandButton
                className="btn-danger"
                label="Test motors"
                confirm
                onFire={sendTestMotors}
              />
              <NumField
                label="speed"
                unit="m/s"
                value={testSpeed}
                onChange={setTestSpeed}
                lim={LIM.speedMps}
              />
              <NumField
                label="per leg"
                unit="s"
                value={testDur}
                onChange={setTestDur}
                lim={LIM.testS}
              />
            </Row>
          </div>
          <p className="mt-3 text-xs text-slate-500">
            Nudge is closed-loop on odometry ({LIM.nudgeMm.min}–{LIM.nudgeMm.max} mm),
            turn is closed-loop on the integrated gyro ({LIM.turnDeg.min}–{LIM.turnDeg.max}°).
            Speeds are clamped to {LIM.speedMps.min}–{LIM.speedMps.max} m/s: under
            the low end the firmware stalls and then lurches instead of creeping,
            and the high end is the reviewed envelope. Test motors is the only
            one here that is confirm-gated — a rover on blocks and a rover on the
            ground look identical from this page.
          </p>
        </Card>

        {/* ----------------------------------------------------- Diagnostics */}
        <Card>
          <CardHeader title="Diagnostics" subtitle="Read-back and actuators — no drive motion" />
          <div className="space-y-3">
            <Row label="Read">
              {/*
                Deliberately NOT "read encoders" in the label. This drive board
                publishes /odom_raw and nothing resembling a tick count, so a
                button promising ticks would send an operator looking for a
                number that does not exist on this hardware.
              */}
              <CommandButton
                label="Pose / odometry"
                title="read_encoders — returns /odom_raw pose and twist. This board exposes no raw encoder ticks."
                onFire={() => fire("read_encoders", targets)}
              />
              <CommandButton label="Status" onFire={() => fire("status", targets)} />
              <CommandButton
                label="Ping"
                title="Echoes a timestamp; the log shows the round trip in ms"
                onFire={sendPing}
              />
            </Row>

            <Row label="Beeper">
              <CommandButton label="Beep" onFire={sendBeep} />
              <NumField
                label="length"
                unit="ms"
                value={beepMs}
                onChange={setBeepMs}
                lim={LIM.beepMs}
              />
            </Row>

            <Row label="Servo">
              <div className="flex overflow-hidden rounded-lg border border-white/10">
                {([1, 2] as const).map((w) => (
                  <button
                    key={w}
                    onClick={() => setServoWhich(w)}
                    className={`px-2.5 py-1.5 font-mono text-xs transition ${
                      servoWhich === w
                        ? "bg-ember-500/20 text-ember-200"
                        : "bg-white/5 text-slate-400 hover:bg-white/10"
                    }`}
                    title={`/servo_s${w}`}
                  >
                    S{w}
                  </button>
                ))}
              </div>
              <NumField
                label="angle"
                unit="°"
                value={servoAngle}
                onChange={setServoAngle}
                lim={LIM.servoDeg}
              />
              <CommandButton label="Move servo" onFire={sendServo} />
            </Row>
          </div>
          <p className="mt-3 text-xs text-slate-500">
            <b>Pose / odometry</b> is the <code className="font-mono">read_encoders</code>{" "}
            command: this drive board publishes cumulative pose and twist on
            /odom_raw and no raw tick counts at all, so that is what comes back.
            Beep accepts 0 (silence) or {LIM.beepMs.min + 10}–{LIM.beepMs.max} ms —
            1–9 are mode flags to this firmware rather than durations, and 1
            latches the beeper on, so anything in that range is sent as 10 ms.
            Servo travel is inset to {LIM.servoDeg.min}–{LIM.servoDeg.max}° so the
            horn never parks against a mechanical end stop and stalls.
          </p>
        </Card>

        {/* -------------------------------------------------- Mode & stream */}
        <Card>
          <CardHeader title="Mode & stream" subtitle="Autonomy, telemetry, service" />
          <div className="space-y-3">
            <Row label="Autonomy">
              {/*
                Kept, and now labelled honestly. NOTHING on this rover
                subscribes auto_on: fpms-teleop's drive_status lists it under
                `not_owned_here` as "nobody — no autonomy loop exists", and
                fpms-rover-agent answers it with a nack naming the real owner.
                The button stays because that nack is genuinely useful, but it
                is no longer styled as the primary action on the card — a
                prominent button whose only possible outcome is a refusal reads
                as a broken rover rather than as a verb that was never wired.
              */}
              <CommandButton
                label="Autonomy ON"
                confirm
                title={autoOnNote}
                onFire={() => fire("auto_on", targets)}
              />
              <CommandButton label="Autonomy OFF" onFire={() => fire("auto_off", targets)} />
            </Row>

            <Row label="Telemetry">
              <CommandButton
                className="btn-primary"
                label="Connect"
                title="fpms-rover-agent — camera and LiDAR publishing"
                onFire={() => fire("connect", targets)}
              />
              <CommandButton
                className="btn-danger"
                label="Disconnect"
                title="fpms-rover-agent — camera and LiDAR publishing"
                onFire={() => fire("disconnect", targets)}
              />
            </Row>

            {/*
              fpms-teleop's OWN stream and status verbs. They were subscribed
              and advertised on the rover from the start and were unreachable
              from here: the backend allowlist did not carry them, so every
              attempt 400'd before it was ever published — which on this page
              looks identical to a dashboard bug.

              drive_status is also the capability-refresh path for the Command
              set card below: its reply carries the live TELEOP_ACTIONS list.
              It reads state and commands no motion, so it is never gated.
            */}
            <Row label="Bridge">
              <CommandButton
                label="Bridge snapshot"
                title="drive_status — fpms-teleop's motion envelope, deadband floors, micro-ROS link and its live verb list. Reads only."
                onFire={() => fire("drive_status", targets)}
              />
              <CommandButton
                label="Stream on"
                title="drive_connect — resumes fpms-teleop's drive telemetry. The Drive tab's health panel and its motion lockout both key off that feed."
                onFire={() => fire("drive_connect", targets)}
              />
              <CommandButton
                className="btn-danger"
                label="Stream off"
                confirm
                title="drive_disconnect — pauses drive telemetry. The Drive tab locks out motion when that feed goes stale, so this disables driving there."
                onFire={() => fire("drive_disconnect", targets)}
              />
            </Row>

            <Row label="Service">
              <CommandButton
                className="btn-danger"
                label="Restart agent"
                confirm
                onFire={() => fire("restart", targets)}
              />
            </Row>
          </div>
          <p className="mt-3 text-xs text-slate-500">
            <b>Autonomy ON is not wired to anything on this rover</b> — it exists
            so the refusal can name where autonomy actually lives, which is the{" "}
            <code className="font-mono">mission</code> command on the Drive tab.
            Autonomy OFF is real: fpms-teleop treats it as a stop and routes it
            through the same ungated path. Connect/disconnect control{" "}
            <b>fpms-rover-agent's</b> camera and LiDAR publishing; the Bridge row
            controls <b>fpms-teleop's</b> separate drive telemetry — two
            different streams, which is exactly why the rover gave them different
            verbs. Turning the bridge stream off will lock out the Drive tab,
            which treats a silent drive feed as an unknown rover state. Restart
            bounces the systemd unit.
          </p>
        </Card>

        {/* -------------------------------------------------------- Tuning */}
        <Card>
          <CardHeader title="Tuning" subtitle="Runtime speed envelope" />
          <div className="space-y-3">
            <Row label="Speeds">
              <NumField
                label="jog max"
                unit="m/s"
                value={jogMax}
                onChange={setJogMax}
                lim={LIM.speedMps}
              />
              <NumField
                label="nudge"
                unit="m/s"
                value={nudgeSpeed}
                onChange={setNudgeSpeed}
                lim={LIM.speedMps}
              />
              <CommandButton
                className="btn-primary"
                label="Apply speeds"
                onFire={sendSetSpeed}
              />
            </Row>
          </div>
          <p className="mt-3 text-xs text-slate-500">
            Real m/s — the same units the acks and telemetry report, so a number
            can be copied straight out of one into here. Both are clamped to{" "}
            {LIM.speedMps.min}–{LIM.speedMps.max} m/s and the nudge speed is held
            at or below jog max (jog max is the hard clamp, so a higher nudge
            would only be clipped back to it). Values below {LIM.speedMps.min} m/s
            are refused by the rover rather than quietly raised: that gap between
            what was set and what ran is where the lurch came from. This changes
            the running service only — it does not survive a restart.
          </p>
        </Card>
      </div>

      {/* ------------------------------------------------- Command set / drift */}
      <ErrorBoundary label="Control command set">
        <Card>
          <CardHeader
            title="Command set"
            subtitle="What the rover says it supports, against what this page offers"
            right={
              <div className="flex flex-wrap items-center gap-2">
                <span className={caps.announced ? "chip-ok" : "chip-warn"}>
                  {caps.announced
                    ? `${caps.actions.size} verbs advertised`
                    : "no capability list received"}
                </span>
                <CommandButton
                  label="Ask the rover"
                  title="Publishes drive_status, which replies with fpms-teleop's live TELEOP_ACTIONS. Reads only — commands no motion."
                  onFire={() => fire("drive_status", targets)}
                />
              </div>
            }
          />

          {!caps.announced ? (
            <div className="rounded-lg border border-amber-500/25 bg-amber-500/5 p-3 text-sm text-amber-100/90">
              <b>Nothing has announced a command set yet, and that is not the
              same as “this rover supports nothing”.</b>{" "}
              <span className="font-mono">events/online</span> is published once,
              when a service connects to the broker, and it is not retained — so a
              dashboard opened after the rover booted will never have seen it. Every
              button on this page therefore stays enabled. Press{" "}
              <b>Ask the rover</b> above to request{" "}
              <span className="font-mono">drive_status</span>, whose reply carries
              the live list.
            </div>
          ) : (
            <>
              <div className="flex flex-wrap gap-1.5">
                {drift.advertised.map((a) => {
                  const c = CATALOG[a];
                  const reachable = !!c && c.owner !== "none";
                  return (
                    <span
                      key={a}
                      className={reachable ? "chip" : "chip-warn"}
                      title={
                        `${caps.owners[a] ? `answered by fpms-${caps.owners[a]}` : "owner not stated"}` +
                        `${caps.replyTopics[a] ? ` · replies on ${caps.replyTopics[a]}` : ""}` +
                        `${c ? ` · control on the ${c.owner} tab` : " · NO control on this dashboard"}`
                      }
                    >
                      <span className="font-mono">{a}</span>
                      {c?.owner === "drive" && (
                        <span className="ml-1 text-[10px] text-slate-500">· Drive tab</span>
                      )}
                      {!reachable && (
                        <span className="ml-1 text-[10px] text-amber-300">· no control</span>
                      )}
                    </span>
                  );
                })}
              </div>

              {/* Verbs the rover has and the dashboard does not. A bare button
                  is offered only where firing with no parameters is knowably
                  harmless; anything else is named but left unwired, because
                  "the rover advertised it" is not evidence that firing it blind
                  is safe. */}
              {drift.advertisedNoControl.length > 0 && (
                <div className="mt-4 rounded-lg border border-amber-500/40 bg-amber-500/5 p-3">
                  <div className="text-sm text-amber-100">
                    <b>
                      {drift.advertisedNoControl.length} verb
                      {drift.advertisedNoControl.length > 1 ? "s" : ""} the rover
                      supports and this dashboard has no control for.
                    </b>{" "}
                    The rover gained a command that no button reaches — this list
                    is how that becomes visible instead of being found by
                    accident.
                  </div>
                  <div className="mt-2 flex flex-wrap items-center gap-2">
                    {drift.advertisedNoControl.map((a) =>
                      CATALOG[a]?.blindSafe ? (
                        <CommandButton
                          key={a}
                          label={a}
                          confirm={MOTION_VERBS.has(a)}
                          title={`Sends ${a} with no parameters. ${caps.replyTopics[a] ? `Replies on ${caps.replyTopics[a]}.` : ""}`}
                          onFire={() => fire(a, targets)}
                        />
                      ) : (
                        <span
                          key={a}
                          className="chip-warn"
                          title="Not offered as a button: this verb needs parameters, and firing it empty could mean something the operator did not intend."
                        >
                          <span className="font-mono">{a}</span>
                          <span className="ml-1 text-[10px]">· needs parameters</span>
                        </span>
                      ),
                    )}
                  </div>
                </div>
              )}

              {drift.controlNotAdvertised.length > 0 && (
                <div className="mt-3 rounded-lg border border-white/5 bg-black/30 p-3 text-xs text-slate-400">
                  <b className="text-slate-300">
                    Offered here but not in the announcement:
                  </b>{" "}
                  {drift.controlNotAdvertised.map((a) => (
                    <span key={a} className="mr-1.5 font-mono text-slate-300">
                      {a}
                    </span>
                  ))}
                  <div className="mt-1">
                    Expected for most of these: only fpms-teleop and fpms-missions
                    publish verb lists at all, so fpms-rover-agent's own
                    ping/status/connect/disconnect/restart never appear even
                    though they work.{" "}
                    {caps.notOwned["auto_on"] ? (
                      <>
                        The rover has explicitly told us{" "}
                        <span className="font-mono">auto_on</span> is owned by
                        nobody: “{caps.notOwned["auto_on"]}”.
                      </>
                    ) : (
                      <>
                        A motion verb in this list, though, is a button that will
                        time out.
                      </>
                    )}
                  </div>
                </div>
              )}
            </>
          )}

          {caps.services.length > 0 && (
            <div className="mt-4 grid gap-2 sm:grid-cols-2">
              {caps.services.map((s) => (
                <div key={s.svc} className="rounded-lg border border-white/5 bg-black/20 px-3 py-2">
                  <div className="flex items-center justify-between gap-2">
                    <span className="font-mono text-sm text-slate-200">fpms-{s.svc}</span>
                    <span className="chip font-mono text-[10px]">via {s.via}</span>
                  </div>
                  <div className="mt-1 font-mono text-[11px] text-slate-500">
                    {s.actions.length
                      ? `${s.actions.length} verbs`
                      : "no verbs — sensor service"}
                    {s.capabilities.length ? ` · ${s.capabilities.join(", ")}` : ""}
                    {s.actsSilentlyOn.length
                      ? ` · acts silently on ${s.actsSilentlyOn.join(", ")}`
                      : ""}
                  </div>
                </div>
              ))}
            </div>
          )}

          <p className="mt-3 text-xs text-slate-500">
            fpms-teleop publishes its real{" "}
            <code className="font-mono">TELEOP_ACTIONS</code> on{" "}
            <code className="font-mono">events/online</code> and again in every{" "}
            <code className="font-mono">drive_status</code> reply, with a comment
            in its source saying the dashboard should build its button set from
            that rather than from a hardcoded list that can drift. This page still
            owns each verb's parameter UI — a generic sender could not offer a
            clamped millimetre box — so the honest arrangement is this one: the
            controls are written here, the list is read from the rover, and the
            difference is printed above rather than discovered one three-second
            timeout at a time.
          </p>
        </Card>
      </ErrorBoundary>

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
            Nothing yet. Send a command — every reply, refusal and silence lands here.
          </div>
        ) : (
          <ul className="space-y-1">
            {log.map((e) => (
              <li
                key={e.id}
                className="rounded-md border border-white/5 bg-black/30 px-3 py-1.5"
              >
                <div className="flex items-center gap-3">
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
                </div>
                {e.fields && e.fields.length > 0 && (
                  <div className="mt-1 flex flex-wrap gap-1">
                    {e.fields.map((f) => (
                      <span
                        key={f.k}
                        className="rounded border border-white/5 bg-white/[0.03] px-1.5 py-0.5 font-mono text-[10px] text-slate-300"
                      >
                        <span className="text-slate-500">{f.k}</span> {f.v}
                      </span>
                    ))}
                  </div>
                )}
              </li>
            ))}
          </ul>
        )}
        {log.some((e) => e.kind === "stale" || e.kind === "nohw") && (
          <div className="mt-3 space-y-1 rounded-lg border border-amber-500/25 bg-amber-500/5 p-3 text-xs text-amber-200/90">
            {log.some((e) => e.kind === "stale") && (
              <div>
                <b>Agent too old</b> — the rover answered “unknown command”. Its
                agent build predates this action; update the on-rover service.
                Nothing is broken on the dashboard side.
              </div>
            )}
            {log.some((e) => e.kind === "nohw") && (
              <div>
                <b>No motor interface</b> — the rover answered “no motor
                interface”. On this chassis that means the command reached a
                service that does not own the motors (they live in fpms-teleop,
                on the micro-ROS side), not that the hardware is missing. The
                command path itself worked end to end.
              </div>
            )}
          </div>
        )}
      </Card>
    </div>
  );
}

/** One labelled row of controls inside a card, so the cards stay scannable. */
function Row({ label, children }: { label: string; children: ReactNode }) {
  return (
    <div className="flex flex-wrap items-center gap-2">
      <span className="w-20 shrink-0 text-xs uppercase tracking-wider text-slate-500">
        {label}
      </span>
      {children}
    </div>
  );
}

/**
 * A small inline numeric input. Deliberately not validating on every keystroke:
 * clamping mid-typing fights the operator (a "1" on the way to "150" would
 * become the minimum). The value is clamped when the command fires and written
 * back, so what the box shows afterwards is what the rover was asked for.
 */
function NumField({
  label,
  value,
  onChange,
  lim,
  unit,
}: {
  label: string;
  value: string;
  onChange: (v: string) => void;
  lim: Limit;
  unit?: string;
}) {
  return (
    <label
      className="inline-flex items-center gap-1.5 text-xs text-slate-400"
      title={`${label}: ${lim.min}–${lim.max}${unit ? ` ${unit}` : ""} (clamped before sending)`}
    >
      <span>{label}</span>
      <input
        type="number"
        inputMode="decimal"
        value={value}
        min={lim.min}
        max={lim.max}
        step={lim.step}
        onChange={(e) => onChange(e.target.value)}
        className="w-20 rounded-md border border-white/10 bg-black/40 px-2 py-1 font-mono text-xs text-slate-100 outline-none transition focus:border-ember-500/40"
      />
      <span className="text-[10px] text-slate-500">
        {unit} <span className="text-slate-600">{lim.min}–{lim.max}</span>
      </span>
    </label>
  );
}

/**
 * A command button that optionally arms on first click and fires on the second,
 * disarming itself after a few seconds. Inline rather than a modal: it keeps the
 * confirmation on the control being confirmed, and an unconfirmed click simply
 * decays instead of leaving a dialog to dismiss.
 */
function CommandButton({
  label,
  onFire,
  className = "btn",
  confirm = false,
  title,
}: {
  label: string;
  onFire: () => void;
  className?: string;
  confirm?: boolean;
  title?: string;
}) {
  const [armed, setArmed] = useState(false);
  const timer = useRef<number | null>(null);

  useEffect(() => () => { if (timer.current) window.clearTimeout(timer.current); }, []);

  const click = () => {
    if (!confirm) { onFire(); return; }
    if (armed) {
      if (timer.current) window.clearTimeout(timer.current);
      setArmed(false);
      onFire();
      return;
    }
    setArmed(true);
    timer.current = window.setTimeout(() => setArmed(false), 4000);
  };

  // The reason a control is confirm-gated is often the single most useful thing
  // on the page (see `motionHint`), so it is appended to the caller's tooltip
  // rather than replacing it — the old behaviour discarded it entirely.
  const tip = confirm
    ? `${title ? `${title} · ` : ""}Requires a second click to confirm`
    : title;

  return (
    <button
      onClick={click}
      className={`${className} ${armed ? "border-amber-500/50 bg-amber-500/15 text-amber-100" : ""}`}
      title={tip}
    >
      {armed ? "click again to confirm" : label}
    </button>
  );
}

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
  nohw: "no motor interface",
  timeout: "no reply",
  error: "not sent",
  event: "event",
};

type Outcome = { kind: LogKind; text: string; hint?: string; fields?: Field[] };

/**
 * Turn a reply into a log row. Returns null for subtypes that are not replies
 * to a command (online/offline notices, alerts) — those are logged as events
 * and never resolve a pending row.
 */
function describeReply(
  subtype: string,
  data: Record<string, any>,
  sentAt?: number,
): Outcome | null {
  switch (subtype) {
    case "ack": return describeAck(data);
    case "nack": return describeNack(data);
    case "pong": return describePong(data, sentAt);
    case "encoders": return describeEncoders(data);
    case "status": return describeStatus(data);
    case "drive_status": return describeDriveStatus(data);
    case "motor_test": return describeMotorTest(data);
    default: return null;
  }
}

/**
 * fpms-teleop's own snapshot. Distinct from `status`, which is
 * fpms-rover-agent's — the two answer different questions and the rover
 * deliberately gave them different verbs so neither shadows the other.
 *
 * This is the richest read on the rover, and the one that answers "why will it
 * not move": `motion_allowed` plus `motion_block_reason` is the rover's own
 * verdict, which beats inferring it from a stale telemetry age on the Drive
 * tab. The verb list it carries is consumed separately by lib/capabilities.
 */
function describeDriveStatus(data: Record<string, any>): Outcome {
  const fields: Field[] = [];

  const blocked = data.motion_allowed === false;
  const reason = str(data.motion_block_reason);
  fields.push({
    k: "motion",
    v: data.motion_allowed === true ? "allowed" : blocked ? `BLOCKED — ${reason ?? "reason not given"}` : "—",
  });
  fields.push({ k: "micro-ROS", v: linkWord(data.ros_ok) });
  fields.push({ k: "mode", v: str(data.mode) ?? "—" });
  fields.push({ k: "moving", v: fmtBool(data.moving) });

  const batt = num(data.battery_v);
  fields.push({
    k: "battery",
    v: batt === null
      ? "—"
      : `${batt.toFixed(2)} V${data.battery_low === true ? " LOW" : ""}${data.battery_stale === true ? " (stale)" : ""}`,
  });

  const pose = obj(data.pose);
  if (pose) {
    fields.push({
      k: "pose",
      v: `${fmt(pose.x_mm, 0, "")}, ${fmt(pose.y_mm, 0, "")} mm @ ${fmt(pose.heading_deg, 1, "°")}`,
    });
    // The rover reports whether it has an origin at all. Without it the pose
    // above is measured from wherever the bridge happened to start, which is
    // not the arena frame the map draws in.
    fields.push({ k: "origin set", v: fmtBool(data.origin_set) });
  }

  const rates = obj(data.rates_hz);
  if (rates) {
    fields.push({ k: "odom", v: fmt(rates.odom, 1, " Hz") });
    fields.push({ k: "imu", v: fmt(rates.imu, 1, " Hz") });
  }

  const limits = obj(data.limits);
  if (limits) {
    fields.push({ k: "jog max", v: fmt(limits.jog_max_mps, 3, " m/s") });
    fields.push({ k: "nudge", v: fmt(limits.nudge_mps, 3, " m/s") });
  }

  // The deadband floors are the reason "just go slower" is not available, and
  // the rover states whether they are even switched on. Reporting a floor of
  // 0.000 as though it were measured would be the same lie in the other
  // direction, so `enabled` is shown alongside it.
  const floors = obj(data.floors);
  if (floors) {
    fields.push({
      k: "floors",
      v: floors.enabled === true
        ? `${fmt(floors.min_cmd_lin_mps, 4, " m/s")} / ${fmt(floors.min_cmd_ang_radps, 4, " rad/s")}`
        : "off (unmeasured)",
    });
  }

  if ("deadman_ok" in data) fields.push({ k: "deadman", v: fmtBool(data.deadman_ok) });
  if ("lurch_trips" in data) fields.push({ k: "lurch trips", v: fmt(data.lurch_trips, 0, "") });
  if ("telemetry_enabled" in data) {
    fields.push({ k: "drive stream", v: data.telemetry_enabled === true ? "on" : "off" });
  }
  if ("uptime_s" in data) fields.push({ k: "uptime", v: durText(data.uptime_s) });

  const actions = strList(data.actions);
  if (actions.length) fields.push({ k: "verbs", v: String(actions.length) });

  const err = str(data.last_error);
  return {
    kind: "ack",
    text: blocked
      ? `bridge snapshot — MOTION BLOCKED: ${reason ?? "reason not given"}`
      : err
        ? `bridge snapshot — last error: ${err}`
        : "bridge snapshot",
    hint: reason ?? err ?? undefined,
    fields,
  };
}

/** Numeric ack fields worth surfacing, and how to render each. */
const ACK_NUMS: Array<[key: string, label: string, digits: number, unit: string]> = [
  ["mm", "mm", 0, " mm"],
  ["measured_mm", "measured", 0, " mm"],
  ["lateral_mm", "lateral", 0, " mm"],
  ["deg", "deg", 1, "°"],
  ["measured_deg", "measured", 1, "°"],
  ["error_deg", "error", 1, "°"],
  ["speed_mps", "speed", 3, " m/s"],
  ["rate_radps", "rate", 3, " rad/s"],
  ["elapsed_s", "elapsed", 2, " s"],
  ["ms", "beep", 0, " ms"],
  ["which", "servo", 0, ""],
  ["angle", "angle", 0, "°"],
  ["jog_max", "jog max", 3, " m/s"],
  ["nudge", "nudge", 3, " m/s"],
];

function describeAck(data: Record<string, any>): Outcome {
  const detail = data.result ?? data.detail ?? data.message;
  const state = str(data.state);
  const reason = str(data.reason);
  // The rover reports every clamp it applied; those notes are the whole point
  // of asking, so they are never dropped on the floor.
  const notes = strList(data.notes);

  const base =
    typeof detail === "string" && detail
      ? detail
      : detail != null
        ? compact(detail)
        : state && reason
          ? `${state} — ${reason}`
          : state || reason || "acknowledged";

  const fields: Field[] = [];
  for (const [key, label, digits, unit] of ACK_NUMS) {
    if (key in data) fields.push({ k: label, v: fmt(data[key], digits, unit) });
  }
  if (typeof data.deadband_floored === "boolean" && data.deadband_floored) {
    fields.push({ k: "note", v: "raised to clear the motor deadband" });
  }
  if (notes.length) fields.push({ k: "clamped", v: notes.join(" · ") });

  return {
    kind: "ack",
    text: base,
    hint: notes.length ? notes.join(" · ") : undefined,
    fields: fields.length ? fields : undefined,
  };
}

/**
 * Two rover replies are not failures of this dashboard and must not read like
 * one. "unknown command" is exactly what an un-updated rover agent returns, and
 * "no motor interface on this rover" means the command landed on the service
 * that does not own the motors — on this chassis they live in fpms-teleop.
 * Rendering either as a generic error sends the operator hunting a bug that
 * isn't there.
 */
function describeNack(data: Record<string, any>): Outcome {
  const raw = String(data.error ?? data.reason ?? data.message ?? "");
  const err = raw.toLowerCase();
  if (err.includes("unknown command")) {
    return {
      kind: "stale",
      text: "rover agent too old — command not supported",
      hint: raw,
    };
  }
  if (err.includes("no motor interface")) {
    return {
      kind: "nohw",
      text: "answered by a service that does not own the motors",
      hint: raw,
    };
  }
  return { kind: "nack", text: raw ? `refused — ${raw}` : "refused", hint: raw };
}

/**
 * Round trip for `ping`. Measured from the timestamp we echoed through the
 * rover, which is the number that includes the whole path — browser, backend,
 * broker, rover and back. A client clock far enough off to make that nonsense
 * falls back to the local send time rather than reporting a three-week latency.
 */
function describePong(data: Record<string, any>, sentAt?: number): Outcome {
  const echoed = echoedMs(data);
  const now = Date.now();

  let rtt: number | null = null;
  let source = "echoed timestamp";
  if (echoed !== null) {
    const r = now - echoed;
    // Anything outside this window is a clock disagreement, not a latency.
    if (r >= -1000 && r <= 5 * 60_000) rtt = Math.max(0, r);
  }
  const localSentAt = num(sentAt);
  if (rtt === null && localSentAt !== null) {
    rtt = Math.max(0, now - localSentAt);
    source = echoed === null ? "local send time" : "local send time (clock skew)";
  }

  const fields: Field[] = [
    { k: "round trip", v: rtt === null ? "—" : `${rtt.toFixed(0)} ms` },
    { k: "measured from", v: rtt === null ? "—" : source },
  ];
  // The rover measures the uplink leg on its own side when the clocks agree.
  if ("uplink_ms" in data) fields.push({ k: "uplink", v: fmt(data.uplink_ms, 0, " ms") });

  return {
    kind: "ack",
    text: rtt === null ? "pong — round trip not measurable" : `round trip ${rtt.toFixed(0)} ms`,
    fields,
  };
}

/**
 * `read_encoders` on this chassis. The Yahboom MicroROS board publishes
 * cumulative pose and twist on /odom_raw and does not publish tick counts at
 * all, so this renders pose/odometry and says so — labelling it "ticks" would
 * describe a number that does not exist on this hardware. A legacy agent that
 * does return counts still has them shown.
 */
function describeEncoders(data: Record<string, any>): Outcome {
  const pose = obj(data.pose) ?? data;
  const twist = obj(data.twist) ?? data;

  const fields: Field[] = [
    { k: "x", v: fmtNum(poseMm(pose, "x"), 0, " mm") },
    { k: "y", v: fmtNum(poseMm(pose, "y"), 0, " mm") },
    { k: "heading", v: fmtNum(headingDeg(pose), 1, "°") },
    { k: "vx", v: fmt(twist.odom_vx ?? twist.vx ?? obj(twist.linear)?.x, 4, " m/s") },
    { k: "yaw rate", v: fmt(twist.gyro_z_dps ?? twist.wz_dps, 2, " °/s") },
  ];
  if ("odom_hz" in data) fields.push({ k: "odom", v: fmt(data.odom_hz, 1, " Hz") });
  if (Array.isArray(data.counts)) {
    fields.push({ k: "counts", v: compact(data.counts) });
  }

  return {
    kind: "ack",
    text: "pose + twist — this board publishes no encoder ticks",
    fields,
  };
}

/** Full snapshot. Curated into readable chips rather than dumped as JSON. */
function describeStatus(data: Record<string, any>): Outcome {
  const fields: Field[] = [];

  const batt = num(data.battery_v);
  fields.push({
    k: "battery",
    v: batt === null ? "—" : `${batt.toFixed(2)} V${data.battery_low === true ? " LOW" : ""}`,
  });
  fields.push({ k: "link", v: linkWord(data.ros_ok ?? data.uros_session) });
  fields.push({ k: "mode", v: str(data.mode) ?? "—" });
  fields.push({ k: "moving", v: fmtBool(data.moving) });

  const x = poseMm(data, "x");
  const y = poseMm(data, "y");
  const hdg = headingDeg(data);
  if (x !== null || y !== null || hdg !== null) {
    fields.push({
      k: "pose",
      v: `${fmtNum(x, 0)}, ${fmtNum(y, 0)} mm @ ${fmtNum(hdg, 1, "°")}`,
    });
  }
  if ("odom_hz" in data) fields.push({ k: "odom", v: fmt(data.odom_hz, 1, " Hz") });
  if ("imu_hz" in data) fields.push({ k: "imu", v: fmt(data.imu_hz, 1, " Hz") });
  if ("uptime_s" in data) fields.push({ k: "uptime", v: durText(data.uptime_s) });
  const last = str(data.last_cmd);
  if (last) fields.push({ k: "last cmd", v: last });
  if ("lurch_trips" in data) fields.push({ k: "lurch trips", v: fmt(data.lurch_trips, 0, "") });

  // The other service reports a different snapshot; both are legitimate answers
  // to `status` and neither should render as a blank card.
  if ("camera_ok" in data) fields.push({ k: "camera", v: fmtBool(data.camera_ok) });
  if ("lidar_ok" in data) fields.push({ k: "lidar", v: fmtBool(data.lidar_ok) });
  if ("frames" in data) fields.push({ k: "frames", v: fmt(data.frames, 0, "") });
  if ("scans" in data) fields.push({ k: "scans", v: fmt(data.scans, 0, "") });

  const err = str(data.last_error);
  return {
    kind: "ack",
    text: err ? `snapshot — last error: ${err}` : "snapshot",
    hint: err ?? undefined,
    fields,
  };
}

/** Bounded motor self-test result. */
function describeMotorTest(data: Record<string, any>): Outcome {
  const err = str(data.error);
  const aborted = data.aborted === true;
  const started = data.started === true;

  const fields: Field[] = [];
  if ("speed_mps" in data || "speed" in data) {
    fields.push({ k: "speed", v: fmt(data.speed_mps ?? data.speed, 3, " m/s") });
  }
  if ("ang_radps" in data) fields.push({ k: "spin", v: fmt(data.ang_radps, 3, " rad/s") });
  if ("duration_s" in data) fields.push({ k: "per leg", v: fmt(data.duration_s, 1, " s") });
  if ("motors_stopped" in data) fields.push({ k: "stopped", v: fmtBool(data.motors_stopped) });

  // Each leg reports what the chassis actually did, which is the entire reason
  // to run the test — a leg that measured nothing is the interesting result.
  const legs = Array.isArray(data.legs) ? data.legs.slice(0, 6) : [];
  legs.forEach((raw: unknown, i: number) => {
    const leg = obj(raw);
    if (!leg) return;
    const name = str(leg.name) ?? str(leg.dir) ?? `leg ${i + 1}`;
    const mm = num(leg.measured_mm) ?? num(leg.mm);
    const deg = num(leg.measured_deg) ?? num(leg.deg);
    const v = mm !== null ? `${mm.toFixed(0)} mm` : deg !== null ? `${deg.toFixed(1)}°` : "—";
    fields.push({ k: name, v });
  });
  const notes = strList(data.notes);
  if (notes.length) fields.push({ k: "clamped", v: notes.join(" · ") });

  const text = err
    ? `self-test failed — ${err}`
    : aborted
      ? "self-test aborted (stop received)"
      : started
        ? "self-test started"
        : data.completed === true
          ? "self-test complete"
          : "self-test finished";

  return { kind: err ? "nack" : "ack", text, hint: err ?? undefined, fields };
}

/** MQTT timestamps are epoch seconds; tolerate a millisecond one anyway. */
function normalizeTs(ts: unknown): number | null {
  if (typeof ts !== "number" || !Number.isFinite(ts)) return null;
  return ts > 1e11 ? ts / 1000 : ts;
}

/**
 * The timestamp we sent, as it came back. The rover may echo it at the top
 * level, under a different name, or wrapped in the whole original payload, and
 * it may be in seconds or milliseconds — which one is decided by magnitude, not
 * by trusting a units field.
 */
function echoedMs(data: Record<string, any>): number | null {
  const echo = obj(data.echo);
  const raw =
    num(data.t) ??
    num(data.t_echo) ??
    (echo ? num(echo.t) ?? num(echo.t_ms) : null);
  if (raw === null) return null;
  if (raw > 1e11) return raw;        // already milliseconds
  if (raw > 1e8) return raw * 1000;  // epoch seconds
  return null;                       // not a wall clock at all
}

/**
 * Every number out of telemetry passes through here before it is formatted. A
 * `.toFixed()` on a field the rover happened to omit once took the whole
 * dashboard white, so an absent or non-finite value becomes "—" and never an
 * exception.
 */
function num(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

function fmt(v: unknown, digits: number, unit: string): string {
  return fmtNum(num(v), digits, unit);
}

function fmtNum(n: number | null, digits: number, unit = ""): string {
  return n === null ? "—" : `${n.toFixed(digits)}${unit}`;
}

function fmtBool(v: unknown): string {
  return typeof v === "boolean" ? (v ? "yes" : "no") : "—";
}

function linkWord(v: unknown): string {
  return typeof v === "boolean" ? (v ? "up" : "down") : "—";
}

function durText(v: unknown): string {
  const s = num(v);
  if (s === null) return "—";
  if (s < 90) return `${s.toFixed(0)}s`;
  if (s < 5400) return `${(s / 60).toFixed(1)}m`;
  return `${(s / 3600).toFixed(1)}h`;
}

function str(v: unknown): string | null {
  return typeof v === "string" && v.trim() !== "" ? v : null;
}

function strList(v: unknown): string[] {
  return Array.isArray(v) ? v.filter((x): x is string => typeof x === "string") : [];
}

function obj(v: unknown): Record<string, any> | null {
  return v !== null && typeof v === "object" && !Array.isArray(v)
    ? (v as Record<string, any>)
    : null;
}

/** Pose in millimetres, whichever unit the rover chose to report it in. */
function poseMm(src: Record<string, any>, axis: "x" | "y"): number | null {
  const mm = num(src[`${axis}_mm`]);
  if (mm !== null) return mm;
  const m = num(src[`${axis}_m`]);
  if (m !== null) return m * 1000;
  const pos = obj(src.position);
  const p = pos ? num(pos[axis]) : null;
  return p === null ? null : p * 1000;
}

function headingDeg(src: Record<string, any>): number | null {
  const deg = num(src.heading_deg) ?? num(src.yaw_deg);
  if (deg !== null) return deg;
  const rad = num(src.yaw);
  return rad === null ? null : (rad * 180) / Math.PI;
}

function clampTo(raw: string, lim: Limit): number {
  const n = Number(raw);
  if (!Number.isFinite(n)) return lim.def;
  return Math.min(lim.max, Math.max(lim.min, n));
}

/** Clamp, round to the field's precision, and write the result back. */
function commit(raw: string, lim: Limit, set: (v: string) => void): number {
  const v = round(clampTo(raw, lim), lim.digits);
  set(String(v));
  return v;
}

/**
 * Beep is the one input where 0 is meaningful (silence now) and where the
 * range just above it is not a duration at all: this firmware reads 1..9 as
 * mode flags, and 1 latches the beeper on with no way to silence it if the link
 * then drops. Anything in that range is therefore sent as the shortest real
 * duration instead.
 */
function clampBeepMs(raw: string): number {
  const n = Number(raw);
  if (!Number.isFinite(n)) return LIM.beepMs.def;
  const ms = Math.round(Math.min(LIM.beepMs.max, Math.max(0, n)));
  return ms === 0 ? 0 : Math.max(10, ms);
}

function round(n: number, digits: number): number {
  const f = 10 ** digits;
  return Math.round(n * f) / f;
}

function paramSummary(params: Record<string, unknown>): string {
  const parts = Object.entries(params)
    // The ping nonce is an epoch millisecond count; it says nothing useful in a
    // log line and its round trip is reported by the pong row anyway.
    .filter(([k]) => k !== "t")
    .map(([k, v]) => `${k}=${compact(v)}`);
  return parts.length ? parts.join(" ") : "sent";
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

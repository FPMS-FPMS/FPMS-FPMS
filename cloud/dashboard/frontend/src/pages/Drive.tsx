import { useEffect, useMemo, useRef, useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { StatusPill } from "../components/StatusPill";
import ErrorBoundary from "../components/ErrorBoundary";
import Joystick, { type StickValue } from "../components/Joystick";
import { useChannel } from "../lib/ws";
import { apiPostJson } from "../lib/api";
import { useThings } from "../lib/things";

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
 * And when the link is down the rover cannot move, so the motion controls say
 * so instead of accepting clicks that go nowhere. STOP ALL is the exception: it
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

/** Speed limits enforced on the rover. Printed so the operator is not guessing. */
const CAPS: { what: string; limit: string }[] = [
  { what: "jog (stick)", limit: "0.05 m/s" },
  { what: "nudge", limit: "0.04 m/s" },
  { what: "dock", limit: "0.025 m/s" },
  { what: "turn", limit: "0.4 rad/s" },
];

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

/**
 * Mission telemetry older than this stops being the rover's state and starts
 * being history. Shorter than TELEMETRY_STALE_MS on purpose: a mission is a
 * machine driving itself across a room, and "the progress bar is six seconds
 * out of date" is a materially different situation from a health panel being
 * six seconds out of date.
 */
const MISSION_STALE_MS = 6000;

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
const MISSION_BACKENDS = [
  {
    id: "deadreckon",
    label: "dead reckoning",
    note: "proven — 0.6% distance error, ±1–4° turns",
    warn: null as string | null,
  },
  {
    id: "nav2",
    label: "Nav2",
    note: "requires the navigation stack to be up",
    warn:
      "Nav2 needs a live LaserScan in ROS, a complete TF tree and a map before it can localise. Those are not up on this rover yet — the board's /scan publishes all zeros and the working LiDAR goes to MQTT, not ROS. A mission sent on this backend will not plan.",
  },
] as const;

type MissionBackend = (typeof MISSION_BACKENDS)[number]["id"];

const DEFAULT_MISSION_BACKEND: MissionBackend = "deadreckon";

/** The four routes offered. Kept as data so the buttons and the wiring agree. */
const MISSIONS: { name: string; label: string; primary?: boolean }[] = [
  { name: "home", label: "RETURN HOME", primary: true },
  { name: "m1", label: "MISSION 1" },
  { name: "m2", label: "MISSION 2" },
  { name: "water", label: "WATER REFILL" },
];

/**
 * Phases that mean "not driving". A phase outside this set — including one we
 * have never seen — counts as running, because the failure that matters is
 * showing "idle" while the rover is moving, not the reverse.
 */
const MISSION_IDLE_PHASES = new Set([
  "idle", "ready", "none", "done", "complete", "completed", "finished",
  "aborted", "cancelled", "canceled", "failed", "error", "stopped",
]);

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
  const [lastCmd, setLastCmd] = useState<StickValue>({ vx: 0, wz: 0 });

  const ev = useChannel<any>("events");
  const drive = useChannel<any>(thing ? `drive:${thing}` : null);
  const tele = readTelemetry(drive.data);

  // Mission progress, published by the mission executor on
  // fpms/<thing>/telemetry/mission. The executor may not be running at all,
  // so every consumer below treats "no data" as a first-class state rather
  // than as zeroes.
  const missionCh = useChannel<any>(thing ? `mission:${thing}` : null);
  const mission = readMission(readTelemetry(missionCh.data));
  const [backend, setBackend] = useState<MissionBackend>(DEFAULT_MISSION_BACKEND);

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
  /**
   * Motion is blocked when we know the link is bad. "Never received anything"
   * is deliberately not in that set: if the drive telemetry topic is not
   * plumbed through on this deployment, locking the controls would leave the
   * operator with a page that cannot drive and no way to tell why. A missing
   * feed is called out in the health panel instead.
   */
  const motionLocked = rosDown || teleStale;
  const lockReason = rosDown
    ? "The rover reports ROS is down. It cannot act on a motion command."
    : teleStale
      ? `No drive telemetry for ${Math.round((teleAgeMs ?? 0) / 1000)}s — the rover's state is unknown.`
      : null;

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
   */
  const abortMission = (targets: string[]) =>
    fire("mission", targets, { name: "abort" });

  /**
   * Fleet emergency stop. Ignores the rover selector on purpose: an e-stop that
   * only halts the rover you happen to have selected is a trap. Never disabled
   * and never confirm-gated — a dialog on an e-stop costs a click during an
   * emergency, and an accidental stop is the safe outcome.
   *
   * The stop goes first because it is the command that has always existed and
   * halts the motors. The mission abort goes with it because a stop alone is
   * not enough against an executor: a mission driving segments republishes
   * cmd_vel continuously, so a single halt would be overwritten by the next
   * tick and the rover would carry on as if nothing had been pressed.
   */
  const stopAll = () => {
    fire("stop", bays);
    abortMission(bays);
  };
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
  const disabledHint = noRover
    ? "no rover selected"
    : rosDown
      ? "ROS is down on the rover"
      : "telemetry stale — link unknown";

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
            title="Emergency stop — halts every rover. Always enabled. Shortcut: Esc"
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
        {lockReason && (
          <div className="flex items-start gap-3 rounded-xl border-2 border-rose-500/50 bg-rose-950/40 p-4">
            <span className="mt-0.5 inline-block h-3 w-3 shrink-0 rounded-full bg-rose-400 pulse-dot text-rose-400" />
            <div>
              <div className="text-base font-semibold text-rose-100">
                {rosDown ? "ROS DOWN — the rover cannot move" : "LINK STALE — rover state unknown"}
              </div>
              <p className="mt-1 text-sm text-rose-200/85">
                {lockReason} Motion controls are disabled until it recovers.{" "}
                <b>STOP ALL still works</b> and is still worth pressing.
              </p>
            </div>
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

        <div className="rounded-xl border border-amber-500/25 bg-amber-500/5 p-4 text-sm text-amber-100/90">
          <b>Everything here is capped slow, on purpose.</b> The rover enforces
          these limits — full stick deflection asks for the jog cap, not for
          whatever the motors can do:
          <div className="mt-2 flex flex-wrap gap-2">
            {CAPS.map((c) => (
              <span key={c.what} className="chip font-mono">
                {c.what} ≤ {c.limit}
              </span>
            ))}
          </div>
          <div className="mt-2 text-xs text-amber-200/70">
            Nudges are fixed {NUDGE_MM} mm steps and turns are closed-loop to the
            requested angle.
          </div>
        </div>

        {/* The caps above say how fast the rover may go. This says how slow it
            can go — the other end of the same argument, and the one that looks
            like the dashboard ignoring the request unless it is spelled out. */}
        <ErrorBoundary label="Drive deadband">
          <DeadbandCard tele={tele} />
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
                sent · vx {lastCmd.vx.toFixed(2)} · wz {lastCmd.wz.toFixed(2)}
              </div>
            </div>
            <p className="mt-3 text-xs text-slate-500">
              Push up to go forward, right to turn right; output is normalised
              −1…1 and the rover maps full deflection to its {CAPS[0].limit} jog
              cap. The centre <span className="font-mono">8%</span> is a dead
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
                <span className="chip font-mono" title="Backend the next mission will be sent with">
                  via {backend}
                </span>
              }
            />

            {/* The selector sits above the buttons it changes. A backend
                picked in another card is a setting nobody reads before
                clicking. */}
            <div className="mb-4">
              <div className="lbl mb-2">Driven by</div>
              <div className="flex flex-wrap gap-2">
                {MISSION_BACKENDS.map((b) => (
                  <button
                    key={b.id}
                    onClick={() => setBackend(b.id)}
                    className={`chip ${
                      backend === b.id
                        ? "border-ember-500/40 bg-ember-500/10 text-ember-200"
                        : ""
                    }`}
                    title={b.warn ?? b.note}
                  >
                    <span className="font-mono">{b.id}</span>
                    <span className="ml-1.5 text-[10px] text-slate-500">· {b.note}</span>
                  </button>
                ))}
              </div>
              {MISSION_BACKENDS.map((b) =>
                b.id === backend && b.warn ? (
                  <div
                    key={b.id}
                    className="mt-2 rounded-lg border border-amber-500/40 bg-amber-500/5 px-3 py-2 text-xs text-amber-100/90"
                  >
                    <b>{b.label} is not ready on this rover.</b> {b.warn} Switch back
                    to <span className="font-mono">deadreckon</span> unless you are
                    deliberately testing the stack.
                  </div>
                ) : null,
              )}
            </div>

            <div className="flex flex-wrap gap-2">
              {MISSIONS.map((m) => (
                <ConfirmButton
                  key={m.name}
                  className={m.primary ? "btn-primary" : "btn"}
                  label={m.label}
                  onFire={() => move("mission", { name: m.name, backend })}
                  disabled={motionDisabled}
                  hint={
                    motionDisabled
                      ? disabledHint
                      : `Run ${m.name} using the ${backend} backend`
                  }
                />
              ))}
            </div>
            <p className="mt-3 text-xs text-slate-500">
              All four are confirm-gated: each one drives the rover somewhere on
              its own, and from here a rover on blocks and a rover on the floor
              look identical. Each is sent as{" "}
              <span className="font-mono">{`{name, backend}`}</span> — progress
              comes back in the Mission card above. Take the stick or hit STOP ALL
              to cut a mission short; STOP ALL aborts the mission as well as
              halting the motors.
            </p>
          </Card>

          <Card>
            <CardHeader title="Set coordinate" subtitle="Tells the rover where it is" />
            <SetCoordinate
              disabled={noRover}
              onFire={(x, y) => thing && fire("set_coordinate", [thing], { x_mm: x, y_mm: y })}
            />
            <p className="mt-3 text-xs text-slate-500">
              Writes the rover's believed position in arena millimetres. It moves
              nothing, so there is nothing to confirm and it stays available when
              the motion controls are locked — but the number has to be right, or
              every mission afterwards is wrong by the same amount.
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

/* ---- deadband ------------------------------------------------------------ */

/**
 * Why the floor exists, next to the numbers that make it up. The operator asked
 * for very slow and got a minimum instead; without this block that reads as the
 * request having been ignored rather than answered.
 */
function DeadbandCard({ tele }: { tele: Record<string, unknown> | null }) {
  const lin = num(tele?.min_cmd_lin);
  const ang = num(tele?.min_cmd_ang);
  const measured = lin !== null || ang !== null;
  const snapped = tele?.deadband_snapped;
  const isSnapped = snapped === true;

  return (
    <Card>
      <CardHeader
        title="Motor deadband floor"
        subtitle="Why very slow is not available"
        right={
          <div className="flex flex-wrap items-center gap-2">
            <span className={measured ? "chip-ok" : "chip-warn"}>
              {measured ? "measured" : "not yet measured"}
            </span>
            <span
              className={isSnapped ? "chip-hot" : snapped === false ? "chip" : "chip-warn"}
              title={
                snapped === undefined
                  ? "The rover is not reporting deadband_snapped yet"
                  : isSnapped
                    ? "The last command was below the floor and was raised to it"
                    : "The last command was already at or above the floor"
              }
            >
              {snapped === undefined
                ? "snap · not reported"
                : isSnapped
                  ? "FLOOR APPLIED"
                  : "no floor applied"}
            </span>
          </div>
        }
      />

      <div className="grid gap-3 sm:grid-cols-2">
        <div
          className={`rounded-lg border px-3 py-2 ${
            lin === null ? "border-amber-500/25 bg-amber-500/5" : "border-white/5 bg-black/20"
          }`}
        >
          <div className="lbl">min linear · min_cmd_lin</div>
          <div
            className={`mt-0.5 font-mono text-2xl tabular-nums ${
              lin === null ? "text-amber-300/80" : "text-slate-200"
            }`}
          >
            {lin === null ? (
              <span className="text-base">not yet measured</span>
            ) : (
              <>
                {lin.toFixed(3)}
                <span className="ml-1 text-sm text-slate-500">m/s</span>
              </>
            )}
          </div>
          <div className="mt-1 text-[11px] text-slate-500">
            Slowest forward/back command the chassis will actually execute.
          </div>
        </div>

        <div
          className={`rounded-lg border px-3 py-2 ${
            ang === null ? "border-amber-500/25 bg-amber-500/5" : "border-white/5 bg-black/20"
          }`}
        >
          <div className="lbl">min angular · min_cmd_ang</div>
          <div
            className={`mt-0.5 font-mono text-2xl tabular-nums ${
              ang === null ? "text-amber-300/80" : "text-slate-200"
            }`}
          >
            {ang === null ? (
              <span className="text-base">not yet measured</span>
            ) : (
              <>
                {ang.toFixed(3)}
                <span className="ml-1 text-sm text-slate-500">rad/s</span>
              </>
            )}
          </div>
          <div className="mt-1 text-[11px] text-slate-500">
            Slowest turn command the chassis will actually execute.
          </div>
        </div>
      </div>

      {isSnapped && (
        <div className="mt-3 rounded-lg border border-ember-500/40 bg-ember-500/10 px-3 py-2 text-sm text-ember-100">
          <b>Floor applied to the last command.</b> You asked for less than the
          deadband, so the rover raised it to the minimum above rather than sending
          a value it would stall on. The rover is moving faster than requested — it
          is not moving slower, and it is not ignoring you.
        </div>
      )}

      <div className="mt-3 rounded-lg border border-amber-500/25 bg-amber-500/5 p-3 text-sm text-amber-100/90">
        <b>Why speeds are floored.</b> Below the motor deadband the chassis does
        not creep — it stalls, the drivers wind up against a load that will not
        move, and then it lurches when something finally breaks free. Measured on
        this chassis: a <span className="font-mono">100 mm</span> forward command
        came out as a <span className="font-mono">290 mm</span>{" "}
        <b>backward</b> lurch with <span className="font-mono">24°</span> of
        unrequested rotation. A command that is floored to a slow-but-real speed is
        both slower and far more predictable than one that is honoured literally
        and then discharged all at once. The caps above still apply on top: the
        floor is a minimum, not a licence to go fast.
      </div>
    </Card>
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
  const [x, setX] = useState("0");
  const [y, setY] = useState("0");
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
          className="w-28 rounded-lg border border-white/10 bg-black/40 px-2 py-1.5 font-mono text-sm text-slate-200 outline-none focus:border-ember-500/40"
        />
      </label>
      <label className="flex flex-col gap-1">
        <span className="lbl">y (mm)</span>
        <input
          type="number"
          value={y}
          onChange={(e) => setY(e.target.value)}
          className="w-28 rounded-lg border border-white/10 bg-black/40 px-2 py-1.5 font-mono text-sm text-slate-200 outline-none focus:border-ember-500/40"
        />
      </label>
      <button
        className="btn-primary"
        disabled={disabled || !valid}
        onClick={() => valid && onFire(nx, ny)}
        title={valid ? "Set the rover's believed position" : "Both values must be numbers"}
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

import { useEffect, useMemo, useRef, useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { StatusPill } from "../components/StatusPill";
import { useChannel } from "../lib/ws";
import { apiPostJson } from "../lib/api";
import { useThings } from "../lib/things";

/**
 * Direct rover control. Every button here publishes a command to a physical
 * machine, so the page is built around one rule: the operator must always be
 * able to see what the rover said back. A command that silently succeeds in the
 * UI while the rover never heard it is worse than an error.
 */

type Action =
  | "stop"
  | "auto_on"
  | "auto_off"
  | "test_motors"
  | "read_encoders"
  | "ping"
  | "status"
  | "connect"
  | "disconnect"
  | "restart";

/** How long we wait for an events/ack before calling it unanswered. */
const ACK_TIMEOUT_MS = 3000;
const LOG_LIMIT = 50;

type LogKind =
  | "sent"      // in flight, waiting on the rover
  | "ack"       // rover confirmed
  | "nack"      // rover refused, generic
  | "stale"     // rover agent predates this command
  | "nohw"      // rover has no motor interface wired up yet
  | "timeout"   // nothing came back within ACK_TIMEOUT_MS
  | "error"     // the POST itself failed — never reached MQTT
  | "event";    // unsolicited rover event

type LogItem = {
  id: string;
  at: number;
  thing: string;
  action?: string;
  kind: LogKind;
  text: string;
  hint?: string;
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

  const [log, setLog] = useState<LogItem[]>([]);
  const [seen, setSeen] = useState<Record<string, { lastAt: number; count: number }>>({});

  const ev = useChannel<any>("events");
  // The hub replays its last broadcast to every new subscriber, so on mount we
  // would otherwise show a stale ack from minutes ago as if it just arrived.
  const mountedAt = useRef(Date.now() / 1000);
  // key: `${thing}:${action}` → the log row waiting to be resolved by an ack.
  const pending = useRef(new Map<string, { id: string; timer: number }>());

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
      pending.current.set(key, { id, timer });

      pushLog({ id, at: Date.now(), thing, action, kind: "sent", text: "sent" });

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
    const action = typeof data.action === "string" ? data.action : undefined;

    setSeen((s) => ({
      ...s,
      [thing]: { lastAt: Date.now(), count: (s[thing]?.count ?? 0) + 1 },
    }));

    const key = action ? `${thing}:${action}` : null;
    const p = key ? pending.current.get(key) : undefined;

    if ((subtype === "ack" || subtype === "nack") && key && p) {
      window.clearTimeout(p.timer);
      pending.current.delete(key);
      patchLog(p.id, subtype === "ack" ? describeAck(data) : describeNack(data));
      return;
    }

    // Anything we did not send — another operator's ack, an online notice, a
    // fire alert — still belongs in the log. Silence is the thing to avoid.
    pushLog({
      id: `${thing}:${subtype}:${Date.now()}:${Math.random().toString(36).slice(2, 7)}`,
      at: Date.now(),
      thing,
      action,
      ...(subtype === "nack"
        ? describeNack(data)
        : { kind: "event", text: subtype || "event" }),
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
            fpms/&lt;rover&gt;/events/ack
          </code>. Unanswered after {ACK_TIMEOUT_MS / 1000}s is reported as no reply.
        </p>
      </Card>

      <div className="grid gap-5 lg:grid-cols-2">
        <Card>
          <CardHeader title="Motion" subtitle="Drive state" />
          <div className="flex flex-wrap gap-2">
            <CommandButton
              className="btn-danger"
              label="Stop selected"
              onFire={() => fire("stop", targets)}
            />
            <CommandButton
              className="btn-primary"
              label="Autonomy ON"
              confirm
              onFire={() => fire("auto_on", targets)}
            />
            <CommandButton label="Autonomy OFF" onFire={() => fire("auto_off", targets)} />
          </div>
          <p className="mt-3 text-xs text-slate-500">
            Autonomy ON hands the drive train to the rover's own planner — it is
            confirm-gated. Turning it off, like stopping, is not.
          </p>
        </Card>

        <Card>
          <CardHeader title="Diagnostics" subtitle="Read-back and self-test" />
          <div className="flex flex-wrap gap-2">
            <CommandButton
              className="btn-danger"
              label="Test motors"
              confirm
              onFire={() => fire("test_motors", targets)}
            />
            <CommandButton label="Read encoders" onFire={() => fire("read_encoders", targets)} />
            <CommandButton label="Ping" onFire={() => fire("ping", targets)} />
            <CommandButton label="Status" onFire={() => fire("status", targets)} />
          </div>
          <p className="mt-3 text-xs text-slate-500">
            Test motors spins the wheels. Confirm-gated because a rover on blocks
            and a rover on the ground look identical from here.
          </p>
        </Card>

        <Card>
          <CardHeader title="Stream" subtitle="Sensor publishing" />
          <div className="flex flex-wrap gap-2">
            <CommandButton
              className="btn-primary"
              label="Connect"
              onFire={() => fire("connect", targets)}
            />
            <CommandButton
              className="btn-danger"
              label="Disconnect"
              onFire={() => fire("disconnect", targets)}
            />
          </div>
          <p className="mt-3 text-xs text-slate-500">
            Starts or stops telemetry publishing. Safe either way — the rover
            keeps running, the dashboard just stops receiving.
          </p>
        </Card>

        <Card>
          <CardHeader title="Agent" subtitle="On-rover service" />
          <div className="flex flex-wrap gap-2">
            <CommandButton
              className="btn-danger"
              label="Restart agent"
              confirm
              onFire={() => fire("restart", targets)}
            />
          </div>
          <p className="mt-3 text-xs text-slate-500">
            Restarts the systemd unit. Telemetry drops for a few seconds and any
            autonomy state is lost, so this one asks twice.
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
            Nothing yet. Send a command — every reply, refusal and silence lands here.
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
                <span className="max-w-[46%] truncate font-mono text-[11px] text-slate-400" title={e.hint ?? e.text}>
                  {e.text}
                </span>
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
                <b>No motor hardware</b> — the rover answered “no motor interface”.
                Expected today: no motor driver is wired up yet. The command path
                itself worked end to end.
              </div>
            )}
          </div>
        )}
      </Card>
    </div>
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
}: {
  label: string;
  onFire: () => void;
  className?: string;
  confirm?: boolean;
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

  return (
    <button
      onClick={click}
      className={`${className} ${armed ? "border-amber-500/50 bg-amber-500/15 text-amber-100" : ""}`}
      title={confirm ? "Requires a second click to confirm" : undefined}
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
  nohw: "no hardware",
  timeout: "no reply",
  error: "not sent",
  event: "event",
};

type Outcome = { kind: LogKind; text: string; hint?: string };

function describeAck(data: Record<string, any>): Outcome {
  const detail = data.result ?? data.detail ?? data.message;
  return {
    kind: "ack",
    text: detail == null ? "acknowledged" : compact(detail),
  };
}

/**
 * Two rover replies are not failures of this dashboard and must not read like
 * one. "unknown command" is exactly what an un-updated rover agent returns, and
 * "no motor interface on this rover" is the expected answer today — no motor
 * driver exists yet. Rendering either as a generic error sends the operator
 * hunting a bug that isn't there.
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
      text: "no motor hardware",
      hint: raw,
    };
  }
  return { kind: "nack", text: raw ? `refused — ${raw}` : "refused", hint: raw };
}

/** MQTT timestamps are epoch seconds; tolerate a millisecond one anyway. */
function normalizeTs(ts: unknown): number | null {
  if (typeof ts !== "number" || !Number.isFinite(ts)) return null;
  return ts > 1e11 ? ts / 1000 : ts;
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

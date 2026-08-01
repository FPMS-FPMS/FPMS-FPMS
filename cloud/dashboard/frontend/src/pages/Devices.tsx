import { useEffect, useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { apiGet } from "../lib/api";
import { useChannel } from "../lib/ws";

type Iface = { name: string; ip: string; netmask: string; cidr: string; is_up: boolean; is_loopback: boolean };
type Host = { ip: string; hostname: string | null; open_ports: number[]; ssh_banner: string | null; guess: string };
type Thing = { name: string; arn: string; attributes: Record<string, string> };

type SshResult = {
  ok: boolean; ip: string; username: string; error: string | null;
  hostname: string | null; os_release: string | null; kernel: string | null;
  is_orange_pi: boolean; uptime: string | null;
};

/** backend/main.py::discovery_provision — every field below is in its return. */
type ProvisionResult = {
  thing: string;
  transport?: string;
  iot_endpoint: string;
  mqtt_port?: number | null;
  install: { ok: boolean; exit_code?: number; stdout?: string; stderr?: string; error?: string };
};

/** Subset of /api/health this page reads. Nothing here is invented — see
 *  backend/mqtt_bridge.py::Bridge.status and backend/main.py::health. */
type Health = {
  ok?: boolean;
  mqtt?: {
    connected?: boolean;
    host?: string;
    port?: number;
    tls?: boolean;
    mode?: string;
    messages_seen?: number;
    things_seen?: string[];
    last_message_at?: number | null;
    connect_attempts?: number;
    subscribed?: string[];
    credentials?: { username?: string | null; username_set?: boolean; password_set?: boolean };
    auth_failed?: boolean;
    last_error?: string | null;
    last_error_at?: number | null;
    problem?: string | null;
  };
};

/**
 * The telemetry streams a rover can publish, and the MQTT topic behind each.
 *
 * This list is a copy of TOPIC_FILTERS in backend/mqtt_bridge.py, minus the
 * derived thermal-analysis channel and the shared `events` stream. Channel name
 * is always `<subtype>:<thing>` — that is what /ws/{channel} expects.
 *
 * It is a module constant, and every RoverCard maps over the whole of it in
 * order, so the useChannel calls inside that map are a fixed-length, fixed-order
 * set of hooks. Do not make this list conditional.
 */
const FEEDS = [
  { key: "drive", what: "battery, micro-ROS link, cmd_vel, arena pose", from: "fpms-teleop" },
  { key: "pose", what: "position + heading", from: "rover agent" },
  { key: "lidar", what: "scan ranges", from: "fpms-lidar-ros" },
  { key: "camera", what: "JPEG frames", from: "rover agent" },
  { key: "thermal", what: "thermal grid", from: "rover agent" },
  { key: "mission", what: "mission phase + progress", from: "fpms-missions" },
  { key: "mission_plan", what: "planned route", from: "fpms-missions" },
] as const;

/** Older than this and we stop calling a feed live. Rovers publish at >= 1 Hz. */
const LIVE_WINDOW_S = 20;

/** Bound on how many rovers get their own socket set. 7 feeds each adds up. */
const MAX_CARDS = 6;

function useHealth(): { health: Health | null; err: string | null } {
  const [health, setHealth] = useState<Health | null>(null);
  const [err, setErr] = useState<string | null>(null);
  useEffect(() => {
    let alive = true;
    const tick = () =>
      apiGet<Health>("/api/health")
        .then((h) => { if (alive) { setHealth(h); setErr(null); } })
        .catch((e) => { if (alive) setErr(String(e)); });
    tick();
    const id = window.setInterval(tick, 5000);
    return () => { alive = false; window.clearInterval(id); };
  }, []);
  return { health, err };
}

/** Re-render once a second so every "last seen" age counts up honestly. */
function useSecondTick(): number {
  const [n, setN] = useState(0);
  useEffect(() => {
    const id = window.setInterval(() => setN((v) => v + 1), 1000);
    return () => window.clearInterval(id);
  }, []);
  return n;
}

export default function Devices() {
  const [ifaces, setIfaces] = useState<Iface[]>([]);
  const [cidr, setCidr] = useState("");
  const [scanning, setScanning] = useState(false);
  const [hosts, setHosts] = useState<Host[]>([]);
  const [things, setThings] = useState<Thing[]>([]);
  const [activeHost, setActiveHost] = useState<Host | null>(null);
  const [banner, setBanner] = useState<string | null>(null);
  const { health, err: healthErr } = useHealth();
  useSecondTick();

  const mqtt = health?.mqtt;
  // Registration and reporting are different facts, and this page only ever
  // showed the first. A Thing that exists in the registry but has never
  // published looks identical here to a healthy rover — which is the state
  // after a provision that half-worked, and the one worth seeing.
  //
  // `things_seen` is CUMULATIVE since the backend process started and is never
  // pruned, so it answers "has this ever spoken" and not "is it up". Liveness
  // comes from the telemetry channels themselves, below.
  const everSeen = mqtt?.things_seen ?? [];

  const fleet = Array.from(new Set([...things.map((t) => t.name), ...everSeen])).sort();

  const loadInterfaces = () => apiGet<{ interfaces: Iface[] }>("/api/discovery/interfaces")
    .then((r) => {
      setIfaces(r.interfaces);
      const preferred = r.interfaces.find((i) => i.is_up && !i.is_loopback && !i.ip.startsWith("169.254"));
      if (preferred && !cidr) setCidr(preferred.cidr);
    })
    .catch(() => {});
  const loadThings = () => apiGet<{ things: Thing[] }>("/api/aws/things")
    .then((r) => setThings(r.things)).catch(() => {});

  useEffect(() => { loadInterfaces(); loadThings(); }, []);

  const startScan = async () => {
    if (!cidr) return;
    setScanning(true); setBanner(null); setHosts([]);
    try {
      const r = await fetch("/api/discovery/scan", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ cidr }),
      });
      if (!r.ok) throw new Error(await r.text());
      const data = await r.json();
      setHosts(data.hosts);
      setBanner(`Scan complete — ${data.count} responder${data.count === 1 ? "" : "s"} on ${cidr}.`);
    } catch (e: any) {
      setBanner(`Scan failed: ${String(e).slice(0, 200)}`);
    } finally {
      setScanning(false);
    }
  };

  return (
    <div className="space-y-6">
      <div>
        <div className="lbl">Devices · fleet + onboarding</div>
        <h1 className="h-page mt-1">Which rovers exist, and what each is saying</h1>
        <p className="mt-2 max-w-3xl text-sm text-slate-400">
          Every rover the dashboard knows about — from the AWS IoT registry, from the
          broker, or both — with the last time each one published and on which streams.
          A rover that is powered down is <b>offline, not broken</b>; this page is
          designed to make that state readable rather than alarming.
        </p>
      </div>

      {/* The single most common reason this whole page looks empty. */}
      <BrokerBanner health={health} healthErr={healthErr} />

      {/* Fleet */}
      <Card>
        <CardHeader
          title="Fleet"
          subtitle="Who is out there"
          right={
            <div className="flex flex-wrap items-center gap-2">
              <button className="btn" onClick={loadThings}>Refresh registry</button>
              <span className="chip" title="Things in the AWS IoT registry (LocalStack)">
                {things.length} registered
              </span>
              <span
                className={everSeen.length ? "chip-ok" : "chip"}
                title="Distinct rovers the broker has delivered at least one message from since the backend started. Cumulative — it is not a liveness signal."
              >
                {everSeen.length} heard since boot
              </span>
            </div>
          }
        />

        {fleet.length === 0 ? (
          <div className="rounded-lg border border-white/5 bg-black/30 p-4 text-sm text-slate-400">
            <div className="font-semibold text-slate-200">No rovers known yet.</div>
            <p className="mt-1 leading-relaxed">
              Nothing is registered in IoT Core and the broker has delivered nothing since
              this backend started
              {mqtt?.last_message_at
                ? <> (last message of any kind: {clock(mqtt.last_message_at * 1000)}).</>
                : <> (no message has ever arrived on this run).</>}
              {" "}That is the expected reading with the rover powered off.
            </p>
            <p className="mt-2 leading-relaxed">
              To change it: scan the subnet below and provision a Pi, or follow the
              copy-paste steps on the <b>Install</b> tab. If the rover IS running, check the
              broker banner above — an unauthenticated broker connection makes every panel
              in this app sit at "waiting" with no other symptom.
            </p>
          </div>
        ) : (
          <div className="grid gap-3 lg:grid-cols-2">
            {fleet.slice(0, MAX_CARDS).map((name) => (
              <RoverCard
                key={name}
                name={name}
                thing={things.find((t) => t.name === name) ?? null}
                everSeen={everSeen.includes(name)}
                brokerUp={!!mqtt?.connected}
              />
            ))}
          </div>
        )}
        {fleet.length > MAX_CARDS && (
          <div className="mt-3 text-xs text-slate-500">
            Showing {MAX_CARDS} of {fleet.length}. Each card opens {FEEDS.length} WebSockets,
            so the list is capped rather than opening {fleet.length * FEEDS.length} of them.
          </div>
        )}

        {/* Publishing without being registered is the other half of the same
            check, and it is how a hand-installed rover stays invisible to every
            page that iterates the registry. */}
        {everSeen.filter((r) => !things.some((t) => t.name === r)).length > 0 && (
          <div className="mt-3 rounded-lg border border-amber-500/30 bg-amber-500/5 p-3 text-xs text-amber-100/90">
            <b>Publishing but not registered:</b>{" "}
            {everSeen
              .filter((r) => !things.some((t) => t.name === r))
              .map((r) => <span key={r} className="mr-1.5 font-mono">{r}</span>)}
            <div className="mt-1 text-amber-200/70">
              The broker is receiving telemetry from these, but they have no
              Thing in the registry — provisioned by hand, or registered against
              a different endpoint.
            </div>
          </div>
        )}
      </Card>

      {/* Scanner */}
      <Card>
        <CardHeader
          title="Network scanner"
          subtitle="Onboarding · step 1 · discover"
          right={
            <button className="btn-primary" onClick={startScan} disabled={scanning || !cidr}>
              {scanning ? "Scanning…" : "Scan subnet"}
            </button>
          }
        />
        <div className="grid gap-4 md:grid-cols-[1.4fr_1fr]">
          <div>
            <label className="lbl">Subnet (CIDR)</label>
            <input
              value={cidr}
              onChange={(e) => setCidr(e.target.value)}
              placeholder="e.g. 192.168.0.0/24"
              className="mt-1 w-full rounded-lg border border-white/10 bg-black/40 px-3 py-2 font-mono text-sm text-slate-100 outline-none focus:border-ember-500/50"
            />
            <div className="mt-3 flex flex-wrap gap-2">
              {ifaces.filter((i) => i.is_up && !i.is_loopback).map((i) => (
                <button
                  key={i.cidr}
                  onClick={() => setCidr(i.cidr)}
                  className={`chip ${cidr === i.cidr ? "border-ember-500/40 bg-ember-500/10 text-ember-200" : ""}`}
                  title={`${i.name} · ${i.ip}`}
                >
                  {i.cidr}
                </button>
              ))}
            </div>
          </div>
          <div className="rounded-lg border border-white/5 bg-black/30 p-3 text-xs text-slate-400">
            <div className="lbl mb-1">What this does</div>
            TCP connect scan of ports 22 (SSH), 1883 (MQTT), 80/8080/443. No
            payloads sent. Only responding hosts appear. Scans are bounded to
            /23 or smaller. A powered-off rover will not appear — that is the
            scan working, not failing.
          </div>
        </div>
      </Card>

      {banner && (
        <div className={`rounded-lg border px-4 py-2 text-sm ${
          banner.startsWith("Scan failed") ? "border-rose-500/30 bg-rose-500/10 text-rose-200"
                                            : "border-emerald-500/30 bg-emerald-500/10 text-emerald-200"
        }`}>
          {banner}
        </div>
      )}

      {/* Results */}
      {hosts.length > 0 && (
        <Card>
          <CardHeader title="Discovered hosts" subtitle="Onboarding · step 2 · verify & provision" right={<span className="chip">{hosts.length}</span>} />
          <div className="overflow-x-auto">
            <table className="w-full text-sm">
              <thead className="text-left text-xs uppercase tracking-widest text-slate-500">
                <tr className="border-b border-white/5">
                  <th className="px-3 py-2">Host</th>
                  <th className="px-3 py-2">Guess</th>
                  <th className="px-3 py-2">Open ports</th>
                  <th className="px-3 py-2">SSH banner</th>
                  <th className="px-3 py-2"></th>
                </tr>
              </thead>
              <tbody className="divide-y divide-white/5">
                {hosts.map((h) => (
                  <tr key={h.ip} className="hover:bg-white/5">
                    <td className="px-3 py-2">
                      <div className="font-mono text-slate-100">{h.ip}</div>
                      {h.hostname && <div className="text-xs text-slate-500">{h.hostname}</div>}
                    </td>
                    <td className="px-3 py-2">
                      <span className={h.guess.includes("Pi") ? "chip-hot" : "chip"}>{h.guess}</span>
                    </td>
                    <td className="px-3 py-2 font-mono text-xs text-slate-300">
                      {h.open_ports.length ? h.open_ports.join(", ") : "--"}
                    </td>
                    <td className="px-3 py-2 max-w-[280px] truncate font-mono text-[10px] text-slate-500">
                      {h.ssh_banner ?? "--"}
                    </td>
                    <td className="px-3 py-2 text-right">
                      <button
                        className="btn"
                        disabled={!h.open_ports.includes(22)}
                        title={h.open_ports.includes(22) ? "" : "No SSH on this host — nothing to provision over"}
                        onClick={() => setActiveHost(h)}
                      >
                        SSH + Provision
                      </button>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </Card>
      )}

      {activeHost && (
        <ProvisionDialog
          host={activeHost}
          health={health}
          onClose={() => setActiveHost(null)}
          onDone={() => { loadThings(); }}
        />
      )}
    </div>
  );
}

// ---- fleet ----------------------------------------------------------------

/**
 * One rover, its liveness, and what it is publishing.
 *
 * The last-seen time is REAL and predates this page: hub.connect() replays the
 * last envelope on every channel to each new subscriber (backend/hub.py), and
 * that envelope carries `ts` — the server clock at the moment the message
 * arrived from the broker. So a rover that went quiet an hour ago reports an
 * hour, not "since you opened this tab".
 *
 * The clock is the dashboard host's, compared against this browser's. On the
 * laptop itself they are the same clock; from another device a few seconds of
 * skew is possible, which is why ages are floored at zero and never used to
 * claim something is fresher than the live window.
 */
function RoverCard({
  name, thing, everSeen, brokerUp,
}: {
  name: string;
  thing: Thing | null;
  everSeen: boolean;
  brokerUp: boolean;
}) {
  // Fixed-length map over a module constant — see the note on FEEDS.
  const states = FEEDS.map((f) => useChannel<any>(`${f.key}:${name}`));

  const nowS = Date.now() / 1000;
  const rows = FEEDS.map((f, i) => {
    const st = states[i];
    const ts = typeof st.data?.ts === "number" && Number.isFinite(st.data.ts) ? st.data.ts : null;
    return {
      ...f,
      ts,
      ageS: ts === null ? null : Math.max(0, nowS - ts),
      messages: st.messages,
      socketOpen: st.connected,
      data: st.data?.data ?? null,
    };
  });

  const heard = rows.filter((r) => r.ageS !== null);
  const freshest = heard.length ? Math.min(...heard.map((r) => r.ageS as number)) : null;
  const live = freshest !== null && freshest <= LIVE_WINDOW_S;
  const liveFeeds = rows.filter((r) => r.ageS !== null && r.ageS <= LIVE_WINDOW_S);

  // The one drive field an operator wants at a glance. Rendered only when the
  // feed is live — a battery voltage from two hours ago is a lie in a status row.
  const drive = rows.find((r) => r.key === "drive");
  const driveLive = drive && drive.ageS !== null && drive.ageS <= LIVE_WINDOW_S ? drive.data : null;

  return (
    <div className={`rounded-lg border p-3 ${live ? "border-emerald-500/25 bg-emerald-500/[0.04]" : "border-white/5 bg-black/30"}`}>
      <div className="flex flex-wrap items-center justify-between gap-2">
        <div className="min-w-0">
          <div className="truncate font-semibold text-slate-100">{name}</div>
          <div className="mt-0.5 text-[10px] uppercase tracking-widest text-slate-500">
            {thing ? "in IoT registry" : "not in IoT registry"}
            {everSeen ? " · heard since boot" : " · never heard this run"}
          </div>
        </div>
        <span className={live ? "chip-ok" : "chip"}>
          <span className={`inline-block h-2 w-2 rounded-full ${live ? "bg-emerald-400" : "bg-slate-500"}`} />
          {live ? "ONLINE" : "OFFLINE"}
        </span>
      </div>

      <div className="mt-2 grid grid-cols-2 gap-2 text-xs sm:grid-cols-3">
        <Stat label="Last published" value={freshest === null ? "--" : `${agoText(freshest)} ago`} />
        <Stat
          label="At"
          value={heard.length ? clock(Math.max(...heard.map((r) => (r.ts as number))) * 1000) : "--"}
        />
        <Stat label="Live streams" value={heard.length ? `${liveFeeds.length} of ${heard.length} seen` : "--"} />
      </div>

      {!live && (
        <p className="mt-2 rounded-md border border-white/5 bg-black/40 p-2 text-[11px] leading-relaxed text-slate-400">
          {freshest === null ? (
            brokerUp ? (
              <>
                Offline. Nothing has arrived on any of this rover's topics since the backend
                started — normal when the rover is powered down or its services are stopped.
                Neither the dashboard nor the broker is at fault.
              </>
            ) : (
              <>
                Offline, and the dashboard is <b>not connected to the broker</b> — so it could
                not see this rover even if it were running. Fix the broker first (banner above);
                only then does this state mean anything about the rover.
              </>
            )
          ) : (
            <>
              Offline. Last packet {agoText(freshest)} ago on{" "}
              <span className="font-mono">{heard.sort((a, b) => (a.ageS as number) - (b.ageS as number))[0].key}</span>.
              Values below are that old and are not being refreshed.
            </>
          )}
        </p>
      )}

      {driveLive && (
        <div className="mt-2 grid grid-cols-2 gap-2 text-xs sm:grid-cols-4">
          <Stat label="Battery" value={fmtNum(driveLive.battery_v, 2, " V")} />
          <Stat label="micro-ROS" value={typeof driveLive.ros_ok === "boolean" ? (driveLive.ros_ok ? "ok" : "down") : "--"} />
          <Stat label="Mode" value={typeof driveLive.mode === "string" && driveLive.mode ? driveLive.mode : "--"} />
          <Stat label="Uptime" value={typeof driveLive.uptime_s === "number" ? agoText(driveLive.uptime_s) : "--"} />
        </div>
      )}

      <div className="lbl mt-3 mb-1">Publishing</div>
      <div className="space-y-1">
        {rows.map((r) => (
          <div key={r.key} className="flex items-baseline justify-between gap-2 text-[11px]">
            <span className="font-mono text-slate-300">
              {r.key}
              <span className="ml-1.5 font-sans text-slate-600">fpms/{name}/telemetry/{r.key}</span>
            </span>
            <span className={r.ageS !== null && r.ageS <= LIVE_WINDOW_S ? "text-emerald-300" : "text-slate-500"}>
              {r.ageS === null
                ? (r.socketOpen ? "never" : "never · socket down")
                : `${agoText(r.ageS)} ago`}
            </span>
          </div>
        ))}
      </div>
      <div className="mt-1.5 text-[10px] leading-relaxed text-slate-600">
        "never" means no message has reached this dashboard on that topic since the backend
        started — the sensor may be absent, its service may be stopped, or the rover may be off.
        Sources: {FEEDS.map((f) => `${f.key} (${f.from})`).join(", ")}.
      </div>

      {thing && Object.entries(thing.attributes).length > 0 && (
        <div className="mt-2 flex flex-wrap gap-1">
          {Object.entries(thing.attributes).map(([k, v]) => (
            <span key={k} className="chip text-[10px]">{k}: {v || "--"}</span>
          ))}
        </div>
      )}
      {thing && <div className="mt-1 truncate font-mono text-[10px] text-slate-600" title={thing.arn}>{thing.arn}</div>}
    </div>
  );
}

/**
 * Broker health, stated in terms of what it does to this page.
 *
 * MQTT credentials are unset on this machine more often than not, and the only
 * symptom anywhere else in the app is silence. /api/health already computes the
 * exact sentence (mqtt.problem) and the exact discriminator (mqtt.auth_failed);
 * this renders both rather than re-deriving them.
 */
function BrokerBanner({ health, healthErr }: { health: Health | null; healthErr: string | null }) {
  if (healthErr) {
    return (
      <div className="rounded-lg border border-rose-500/30 bg-rose-500/5 p-4 text-sm text-rose-100">
        <div className="font-semibold">Cannot read /api/health ({healthErr}).</div>
        <p className="mt-1 text-xs opacity-90">
          Everything on this page that depends on rover data will be blank, and that is this
          request failing — not the fleet being down.
        </p>
      </div>
    );
  }
  if (!health?.mqtt) return null;

  const m = health.mqtt;
  const creds = m.credentials;
  if (m.connected) {
    return (
      <div className="rounded-lg border border-emerald-500/25 bg-emerald-500/5 p-3 text-xs text-emerald-100">
        <b>Broker connected</b> — {m.host ?? "--"}:{m.port ?? "--"}
        {m.tls ? " (TLS)" : ""} · {m.messages_seen ?? 0} messages this run ·
        last message {m.last_message_at ? `${agoText(Math.max(0, Date.now() / 1000 - m.last_message_at))} ago` : "--"}.
        {" "}Subscribed to {m.subscribed?.length ?? 0} topic filters. A rover that is running
        will appear below within a second or two.
      </div>
    );
  }

  return (
    <div className="rounded-lg border border-rose-500/30 bg-rose-500/5 p-4 text-sm text-rose-100">
      <div className="font-semibold">
        {m.auth_failed
          ? "The broker REFUSED this dashboard's credentials."
          : `Not connected to the MQTT broker at ${m.host ?? "--"}:${m.port ?? "--"}.`}
      </div>
      <p className="mt-1 text-xs leading-relaxed opacity-90">
        No rover can appear on this page, and every telemetry panel in the app will sit at
        "waiting", <b>whether or not a rover is actually running</b>. This is the cause; the
        empty panels are the symptom.
      </p>
      {m.problem && (
        <p className="mt-2 rounded bg-black/40 p-2 font-mono text-[11px] leading-relaxed">{m.problem}</p>
      )}
      <div className="mt-2 flex flex-wrap gap-x-4 gap-y-1 text-[11px] opacity-80">
        <span>username {creds?.username_set ? <b>set</b> : <b>NOT set</b>}</span>
        <span>password {creds?.password_set ? <b>set</b> : <b>NOT set</b>}</span>
        <span>connect attempts {m.connect_attempts ?? "--"}</span>
        {m.last_error && <span className="font-mono">last error: {m.last_error}</span>}
      </div>
      {!creds?.password_set && (
        <p className="mt-2 text-[11px] leading-relaxed opacity-90">
          Note for provisioning: the installer copies <i>this dashboard's</i> MQTT username and
          password onto the rover (backend/main.py passes <span className="font-mono">settings.mqtt_username
          </span>/<span className="font-mono">mqtt_password</span> into the provisioner script). With
          them unset, a rover provisioned right now gets blank credentials and will be refused by the
          same broker that is refusing this dashboard. Set them first.
        </p>
      )}
    </div>
  );
}

// ---- provisioning ---------------------------------------------------------

function ProvisionDialog({
  host, health, onClose, onDone,
}: {
  host: Host;
  health: Health | null;
  onClose: () => void;
  onDone: () => void;
}) {
  const [username, setUsername] = useState("orangepi");
  const [password, setPassword] = useState("");
  const [thingName, setThingName] = useState(defaultThingName(host));
  const [phase, setPhase] = useState<"idle" | "sshing" | "provisioning" | "done">("idle");
  const [ssh, setSsh] = useState<SshResult | null>(null);
  const [prov, setProv] = useState<ProvisionResult | null>(null);
  const [err, setErr] = useState<string | null>(null);

  const canProvision = ssh?.ok;
  const credsUnset = health?.mqtt?.credentials?.password_set === false;

  const runSsh = async () => {
    setPhase("sshing"); setErr(null); setSsh(null); setProv(null);
    try {
      const r = await fetch("/api/discovery/ssh", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ ip: host.ip, username, password }),
      });
      const data: SshResult = await r.json();
      setSsh(data);
      if (!data.ok) setErr(data.error ?? "SSH failed");
    } catch (e: any) {
      setErr(String(e));
    } finally {
      setPhase("idle");
    }
  };

  const runProvision = async () => {
    setPhase("provisioning"); setErr(null); setProv(null);
    try {
      const r = await fetch("/api/discovery/provision", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ ip: host.ip, username, password, thing_name: thingName }),
      });
      const data: ProvisionResult = await r.json();
      setProv(data);
      if (data.install.ok) { setPhase("done"); onDone(); }
      else { setErr(data.install.error ?? data.install.stderr ?? "install failed"); setPhase("idle"); }
    } catch (e: any) {
      setErr(String(e)); setPhase("idle");
    }
  };

  return (
    <div className="fixed inset-0 z-40 flex items-center justify-center bg-black/70 p-4">
      <div className="w-full max-w-2xl overflow-hidden rounded-2xl border border-white/10 bg-ink-900 shadow-2xl">
        <div className="flex items-start justify-between border-b border-white/5 px-5 py-4">
          <div>
            <div className="lbl">Onboard device</div>
            <div className="mt-0.5 font-mono text-sm text-slate-100">{host.ip}</div>
          </div>
          <button onClick={onClose} className="btn text-slate-400">Close</button>
        </div>

        <div className="max-h-[70vh] space-y-5 overflow-y-auto px-5 py-4">
          {credsUnset && (
            <div className="rounded-lg border border-amber-500/30 bg-amber-500/5 p-3 text-xs leading-relaxed text-amber-100">
              <b>This dashboard has no MQTT password set.</b> The provisioner writes the
              dashboard's own broker credentials into <span className="font-mono">/etc/fpms/config.env</span>
              {" "}on the Pi, so the rover would be installed with blank credentials. If the broker
              requires auth it will refuse the rover, and the only symptom will be panels that never
              fill. Set the credentials, restart the app, then provision.
            </div>
          )}

          <section>
            <div className="lbl mb-2">Step 1 · SSH credentials</div>
            <div className="grid gap-3 sm:grid-cols-2">
              <Field label="Username" value={username} onChange={setUsername} />
              <Field label="Password" value={password} onChange={setPassword} type="password" />
            </div>
            <div className="mt-3">
              <button className="btn-primary" disabled={phase === "sshing"} onClick={runSsh}>
                {phase === "sshing" ? "Connecting…" : "SSH probe"}
              </button>
            </div>
            {ssh && (
              <div className={`mt-3 rounded-lg border p-3 text-xs ${
                ssh.ok ? "border-emerald-500/30 bg-emerald-500/10" : "border-rose-500/30 bg-rose-500/10"
              }`}>
                {ssh.ok ? (
                  <div className="space-y-1">
                    <div className="text-emerald-300">✓ SSH OK · {ssh.is_orange_pi ? "identified as Orange Pi" : "generic Linux host"}</div>
                    <div className="text-slate-300"><b>hostname</b>: {ssh.hostname ?? "--"}</div>
                    <div className="text-slate-300"><b>kernel</b>: {ssh.kernel ?? "--"}</div>
                    <div className="whitespace-pre-wrap text-slate-400">{ssh.os_release ?? "--"}</div>
                    <div className="text-slate-400"><b>uptime</b>: {ssh.uptime ?? "--"}</div>
                  </div>
                ) : (
                  <div className="text-rose-300">✗ {ssh.error ?? "SSH failed with no error text"}</div>
                )}
              </div>
            )}
          </section>

          <section>
            <div className="lbl mb-2">Step 2 · Register as AWS IoT Thing + install publisher</div>
            <Field label="Thing name" value={thingName} onChange={setThingName} mono />
            <div className="mt-3">
              <button className="btn-primary" disabled={!canProvision || phase === "provisioning"} onClick={runProvision}>
                {phase === "provisioning" ? "Provisioning…" : "Register + install"}
              </button>
              <span className="ml-3 text-xs text-slate-500">
                Writes /etc/fpms/config.env + /usr/local/bin/fpms-publisher and enables the
                fpms-publisher systemd unit on the device.
              </span>
            </div>
            {prov && (
              <div className={`mt-3 rounded-lg border p-3 ${
                prov.install.ok ? "border-emerald-500/30 bg-emerald-500/10" : "border-rose-500/30 bg-rose-500/10"
              }`}>
                <div className="text-sm">
                  {prov.install.ok ? "✓ Provisioned. Publisher is running as a systemd service." : "✗ Install failed."}
                </div>
                <div className="mt-1 font-mono text-[10px] text-slate-400">
                  thing={prov.thing} · transport={prov.transport ?? "--"} · endpoint={prov.iot_endpoint || "--"} · port={prov.mqtt_port ?? "--"}
                </div>
                {prov.install.ok && (
                  <div className="mt-1 text-[11px] text-slate-400">
                    It will appear in the Fleet list above once its first message reaches the broker.
                    If it does not, the rover reached SSH but not MQTT — check the broker banner.
                  </div>
                )}
                {(prov.install.stdout || prov.install.stderr) && (
                  <pre className="mt-2 max-h-48 overflow-y-auto rounded bg-black/60 p-2 text-[10px] leading-relaxed text-slate-300">
{(prov.install.stdout ?? "") + (prov.install.stderr ? "\n---stderr---\n" + prov.install.stderr : "")}
                  </pre>
                )}
              </div>
            )}
          </section>

          {err && (
            <div className="rounded-lg border border-rose-500/30 bg-rose-500/10 p-3 text-xs text-rose-200">{err}</div>
          )}
        </div>
      </div>
    </div>
  );
}

// ---- helpers --------------------------------------------------------------

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded-md border border-white/5 bg-black/40 px-2 py-1.5">
      <div className="text-[9px] uppercase tracking-widest text-slate-500">{label}</div>
      <div className="mt-0.5 font-mono text-[11px] text-slate-200">{value}</div>
    </div>
  );
}

/** A number, or "--". Never a zero standing in for a value that never arrived. */
function fmtNum(v: unknown, digits: number, suffix = ""): string {
  return typeof v === "number" && Number.isFinite(v) ? `${v.toFixed(digits)}${suffix}` : "--";
}

/** Elapsed seconds, readable across six orders of magnitude. */
function agoText(s: number): string {
  if (!Number.isFinite(s)) return "--";
  const v = Math.max(0, s);
  if (v < 10) return `${v.toFixed(1)}s`;
  if (v < 60) return `${Math.round(v)}s`;
  if (v < 3600) return `${Math.floor(v / 60)}m ${Math.round(v % 60)}s`;
  if (v < 86400) return `${Math.floor(v / 3600)}h ${Math.floor((v % 3600) / 60)}m`;
  return `${Math.floor(v / 86400)}d ${Math.floor((v % 86400) / 3600)}h`;
}

/** Wall clock for an epoch-ms instant, or a dash if we never had one. */
function clock(atMs: number | null | undefined): string {
  if (typeof atMs !== "number" || !Number.isFinite(atMs)) return "--";
  try { return new Date(atMs).toLocaleTimeString(); } catch { return "--"; }
}

function Field({
  label, value, onChange, type = "text", mono,
}: { label: string; value: string; onChange: (v: string) => void; type?: string; mono?: boolean }) {
  return (
    <label className="block">
      <div className="lbl">{label}</div>
      <input
        type={type}
        value={value}
        onChange={(e) => onChange(e.target.value)}
        className={`mt-1 w-full rounded-lg border border-white/10 bg-black/40 px-3 py-2 text-sm text-slate-100 outline-none focus:border-ember-500/50 ${mono ? "font-mono" : ""}`}
      />
    </label>
  );
}

function defaultThingName(h: Host): string {
  const seed = h.hostname ?? `rover-${h.ip.replaceAll(".", "-")}`;
  return seed.replace(/[^A-Za-z0-9_-]/g, "-").slice(0, 60);
}

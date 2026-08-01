import { useEffect, useState } from "react";
import type { ReactNode } from "react";
import { Card, CardHeader } from "../components/Card";
import SharePanel from "../components/SharePanel";
import { apiGet } from "../lib/api";

type Platform = {
  id: "windows" | "macos" | "linux" | "ios" | "android";
  name: string;
  type: string;
  size_bytes: number | null;
  available: boolean;
  download_url: string | null;
  download_filename?: string | null;
  install_notes: string[];
};

type Catalog = { platforms: Platform[]; server_host: string; public_note: string };

/** /api/network — backend/main.py::network_info. */
type NetUrl = {
  kind: string; url: string; description: string;
  interface?: string; ip?: string; recommended?: boolean;
};
type Network = { urls: NetUrl[] };

/** /api/auth-status. The same fields App.tsx uses to decide which tabs exist. */
type AuthStatus = {
  auth_required: boolean;
  authenticated: boolean;
  is_lan: boolean;
  client_host: string | null;
  is_remote?: boolean;
  safe_mode?: boolean;
  role?: string;
  controls_disabled?: boolean;
};

/** /api/health, only the broker part. */
type Health = {
  mqtt?: {
    connected?: boolean;
    host?: string;
    port?: number;
    things_seen?: string[];
    auth_failed?: boolean;
    problem?: string | null;
    credentials?: { username?: string | null; username_set?: boolean; password_set?: boolean };
  };
};

const ICONS: Record<Platform["id"], string> = {
  windows: "🪟",
  macos: "🍎",
  linux: "🐧",
  ios: "📱",
  android: "🤖",
};

export default function Install() {
  const [cat, setCat] = useState<Catalog | null>(null);
  const [net, setNet] = useState<Network | null>(null);
  const [auth, setAuth] = useState<AuthStatus | null>(null);
  const [health, setHealth] = useState<Health | null>(null);

  useEffect(() => {
    apiGet<Catalog>("/api/downloads").then(setCat).catch(() => {});
    apiGet<AuthStatus>("/api/auth-status").then(setAuth).catch(() => {});
    const loadHealth = () => apiGet<Health>("/api/health").then(setHealth).catch(() => {});
    loadHealth();
    // /api/network enumerates this machine's interfaces and is one of the
    // privileged paths — a public visitor gets 403 and simply sees the
    // placeholder address in the commands below.
    apiGet<Network>("/api/network").then(setNet).catch(() => setNet(null));
    const id = window.setInterval(loadHealth, 10000);
    return () => window.clearInterval(id);
  }, []);

  const lan = net?.urls.find((u) => u.kind === "lan");
  const lanIp = lan?.ip ?? null;
  const hostForCurl = cat?.server_host ?? window.location.host;
  const lanHostForCurl = lanIp ? `${lanIp}:${portOf(hostForCurl)}` : "<laptop-lan-ip>:8000";

  return (
    <div className="space-y-6">
      <div>
        <div className="lbl">Install &amp; connect</div>
        <h1 className="h-page mt-1">Get the app, then get a rover talking to it</h1>
        <p className="mt-2 max-w-3xl text-sm text-slate-400">
          FPMS runs on this laptop. Windows gets a native{" "}
          <code className="rounded bg-black/50 px-1 py-0.5 text-xs">.exe</code>; everything else uses
          the Progressive Web App. Below that: the exact, copy-paste sequence that makes an Orange Pi
          publish into this dashboard, and what to check when it does not.
        </p>
      </div>

      <SharePanel />

      {/* ---- Rover connection ------------------------------------------- */}
      <Card>
        <CardHeader
          title="Connect a rover to this dashboard"
          subtitle="Six steps, in order"
          right={<BrokerChip health={health} />}
        />

        <p className="mb-4 max-w-3xl text-sm leading-relaxed text-slate-400">
          The rover talks to a <b>Mosquitto broker on this laptop over plain LAN MQTT</b>. That
          traffic never goes through Cloudflare — the tunnel only carries the dashboard UI. So the
          rover and the laptop must be on the same network, and the broker must accept connections
          from it.
        </p>

        <Step n={1} title="Put the broker on the LAN and give it an account">
          <p>
            Mosquitto ships loopback-only, so a rover cannot reach it out of the box. Run once in an{" "}
            <b>Administrator</b> PowerShell, from{" "}
            <code className="rounded bg-black/50 px-1 py-0.5 text-xs">cloud\dashboard</code>:
          </p>
          <Snippet text={`powershell -ExecutionPolicy Bypass -File scripts\\Setup-Mosquitto.ps1 -Password '<broker-password>'`} />
          <p>
            It prints <code>MODE=secure</code> (LAN + auth), <code>MODE=anonymous</code> (LAN, no
            auth) or <code>MODE=reverted</code>. Anything but <code>secure</code> deserves a second
            look; <code>reverted</code> means rovers still cannot connect.
          </p>
        </Step>

        <Step n={2} title="Give this dashboard the same broker account">
          <p>
            The dashboard is an MQTT client too. If the broker requires auth and these are unset,
            the backend is refused and <b>every rover panel in the app waits forever with no other
            error</b> — the single most common failure on this machine.
          </p>
          <Snippet
            text={[
              `[Environment]::SetEnvironmentVariable('FPMS_MQTT_USERNAME','fpms','User')`,
              `[Environment]::SetEnvironmentVariable('FPMS_MQTT_PASSWORD','<broker-password>','User')`,
            ].join("\n")}
          />
          <p>Restart the app afterwards — the values are read at startup.</p>
          <CredStatus health={health} />
        </Step>

        <Step n={3} title="Put the rover on the same Wi-Fi, and note this laptop's address">
          <div className="flex flex-wrap items-center gap-2">
            <span>Laptop LAN address:</span>
            <code className="rounded bg-black/50 px-2 py-1 font-mono text-xs text-ember-200">
              {lanIp ?? "--"}
            </code>
            {lan?.interface && <span className="text-slate-500">via {lan.interface}</span>}
          </div>
          <p className="mt-1">
            {lanIp
              ? "This address is baked into the rover's /etc/fpms/config.env at provisioning time, so a DHCP change breaks the link. A router reservation avoids that."
              : "No LAN address is visible right now — either this laptop is off Wi-Fi, or /api/network is not readable from where you are viewing this page. Provisioning needs one: the rover cannot publish to \"localhost\"."}
          </p>
        </Step>

        <Step n={4} title="Provision the Pi">
          <p>
            Easiest: the <b>Devices</b> tab — scan the subnet, pick the Pi, enter its SSH login,
            press Register + install. The same thing over the API:
          </p>
          <Snippet
            text={`curl -X POST http://${lanHostForCurl}/api/discovery/provision \\\n  -H 'Content-Type: application/json' \\\n  -d '{"ip":"192.168.0.42","username":"orangepi","password":"<pi-password>","thing_name":"rover1"}'`}
          />
          <p>
            It replies with the transport it chose, e.g.{" "}
            <code>{`{"thing":"rover1","transport":"lan-mqtt","iot_endpoint":"${lanIp ?? "<laptop-lan-ip>"}","mqtt_port":1883}`}</code>.
            Pass <code>"use_aws": true</code> to route through AWS IoT Core over TLS instead, or{" "}
            <code>"broker_host"</code> to aim at a different broker.
          </p>
          <p className="rounded-md border border-white/5 bg-black/40 p-2 text-xs">
            What lands on the Pi: <code>/etc/fpms/config.env</code> (chmod 600),{" "}
            <code>/usr/local/bin/fpms-publisher</code>, and{" "}
            <code>/etc/systemd/system/fpms-publisher.service</code> with{" "}
            <code>Restart=always</code>. The provisioner copies <i>this dashboard's</i> MQTT username
            and password into that config file — which is why step 2 comes first.
          </p>
        </Step>

        <Step n={5} title="Check it from the Pi">
          <Snippet text={`systemctl status fpms-publisher\njournalctl -u fpms-publisher -f`} />
          <p>
            On connect it publishes <code>fpms/&lt;thing&gt;/events/online</code> at QoS 1, then a
            pose message every 5 seconds. <code>connect failed, retrying in 5s</code> in a loop means
            it cannot reach the broker: wrong address, no firewall rule for TCP 1883, or the broker
            still loopback-only. Connect-then-immediate-drop means wrong credentials.
          </p>
        </Step>

        <Step n={6} title="Check it from here">
          <Snippet text={`curl -s http://${hostForCurl}/api/health`} />
          <p>
            Look at <code>mqtt.connected</code> and <code>mqtt.things_seen</code>. The rover's name
            appearing in <code>things_seen</code> is proof the broker delivered its message to this
            backend. The <b>Devices</b> tab renders exactly that, plus a per-stream last-seen age.
          </p>
          <ThingsStatus health={health} />
        </Step>

        <div className="mt-5 rounded-lg border border-amber-500/25 bg-amber-500/5 p-3 text-xs leading-relaxed text-amber-100">
          <div className="font-semibold">Reality check: what "provisioned" actually gets you</div>
          <p className="mt-1">
            The publisher installed by the steps above is a <b>placeholder</b>. Read it on the Pi and
            you will find a 5-second loop publishing{" "}
            <code>{`{"ts":…, "thing":…, "note":"hook up your ldlidar_ros2 topic here"}`}</code> to{" "}
            <code>telemetry/pose</code> — no LiDAR, no camera, no thermal, no battery. A rover
            provisioned this way shows up as online with one stream, and the LiDAR / Camera / Thermal
            / Drive pages stay empty. That is correct behaviour, not a bug in those pages.
          </p>
          <p className="mt-2">
            The real telemetry comes from the services in{" "}
            <code>cloud/dashboard/rover/</code> — <code>fpms_teleop.py</code> (drive, battery,
            micro-ROS link, arena pose), <code>fpms_missions.py</code> (mission + mission_plan),{" "}
            <code>fpms_lidar_ros.py</code> (lidar). Those are deployed to an already-set-up Pi with:
          </p>
          <Snippet
            tone="amber"
            text={[
              `# set FPMS_PI_PASSWORD in the environment first — never on the command line`,
              `python cloud/dashboard/rover/deploy_rover.py            # dry run: the DEFAULT`,
              `python cloud/dashboard/rover/deploy_rover.py --live`,
              `python cloud/dashboard/rover/deploy_rover.py --live --only fpms_teleop.py`,
            ].join("\n")}
          />
          <p className="mt-1">
            That script backs up every file it replaces on the Pi, syntax-checks each one there
            before restarting anything, stops <code>fpms-teleop</code> cleanly before overwriting it
            so no motion command is cut off mid-flight, and refuses by name to restart{" "}
            <code>micro-ros-agent</code> (a bounce costs a 90-225s board reconnect on this hardware).
          </p>
        </div>

        <div className="mt-4 overflow-x-auto">
          <div className="lbl mb-1">When it does not work</div>
          <table className="w-full text-xs">
            <thead className="text-left uppercase tracking-widest text-slate-500">
              <tr className="border-b border-white/5">
                <th className="py-1.5 pr-3">Symptom</th>
                <th className="py-1.5">Cause</th>
              </tr>
            </thead>
            <tbody className="divide-y divide-white/5 text-slate-300">
              {[
                ["Publisher logs `connect failed`, retries forever", "Broker not reachable on the LAN — check MODE=secure and the firewall rule for TCP 1883."],
                ["Connects, then drops immediately", "Wrong credentials. The Pi's /etc/fpms/config.env must match the broker's password file."],
                ["Every panel in the app says \"waiting\", no error anywhere", "This dashboard's own broker connection is refused. Check the Devices tab banner and /api/health → mqtt.problem."],
                ["Pi looks healthy, dashboard shows nothing", "Thing-name mismatch — every channel is keyed by <thing>."],
                ["Rover online but LiDAR/Camera/Thermal empty", "The placeholder publisher only sends pose. Deploy the real services (above)."],
                ["Worked yesterday, dead today", "The laptop's DHCP lease changed its IP. Re-provision, or set a reservation."],
              ].map(([sym, cause]) => (
                <tr key={sym}>
                  <td className="py-1.5 pr-3 align-top">{sym}</td>
                  <td className="py-1.5 align-top text-slate-400">{cause}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>

        <p className="mt-3 text-[11px] text-slate-500">
          The broker is firewalled to private networks and is never exposed through the tunnel — only
          the UI is. Anyone with the broker password and LAN access can publish telemetry, and that
          password is stored on every provisioned rover. Treat it as sensitive.
        </p>
      </Card>

      {/* ---- Why the Console tab may be missing --------------------------- */}
      <ConsoleAvailability auth={auth} />

      {/* ---- App downloads ------------------------------------------------ */}
      <div>
        <div className="lbl mb-2">Get the app · {cat ? `${cat.platforms.length} platforms` : "loading…"}</div>
        {!cat && (
          <div className="rounded-lg border border-white/5 bg-black/30 p-4 text-sm text-slate-400">
            Could not read <span className="font-mono">/api/downloads</span> yet. The list below fills
            in once the backend answers; nothing is missing from your install.
          </div>
        )}
        <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
          {cat?.platforms.map((p) => (
            <Card key={p.id}>
              <CardHeader
                title={p.name}
                subtitle={p.type}
                right={<span className="text-2xl" aria-hidden>{ICONS[p.id]}</span>}
              />
              {p.download_url && p.available && (
                <a
                  href={p.download_url}
                  className="btn-primary mb-4 w-full justify-center py-2 text-sm font-semibold"
                  download
                >
                  ⬇ Download {p.size_bytes ? `(${humanBytes(p.size_bytes)})` : ""}
                </a>
              )}
              {!p.download_url && (
                <div className="mb-4 rounded-md border border-white/5 bg-black/30 p-3 text-xs text-slate-400">
                  No native installer for {p.name}. Follow the PWA steps below — same features.
                </div>
              )}
              <div className="lbl mb-1.5">Steps</div>
              <ol className="space-y-1.5 text-xs leading-relaxed text-slate-300">
                {p.install_notes.map((step, i) => (
                  <li key={i} className="flex gap-2">
                    <span className="font-mono text-slate-500">{i + 1}.</span>
                    <span>{step}</span>
                  </li>
                ))}
              </ol>
            </Card>
          ))}
        </div>
      </div>

      <Card>
        <CardHeader
          title="What runs where"
          subtitle="Everything is on this laptop"
          right={<span className="chip font-mono text-[10px]">{cat?.server_host ?? window.location.host}</span>}
        />
        <div className="grid gap-4 text-sm md:grid-cols-3">
          <div>
            <div className="font-semibold text-slate-100">The app</div>
            <p className="mt-1 text-slate-400">Native window on Windows via WebView2. Elsewhere it's a PWA — Chrome/Safari's install-as-app.</p>
          </div>
          <div>
            <div className="font-semibold text-slate-100">The backend</div>
            <p className="mt-1 text-slate-400">
              FastAPI on this laptop. Serves the UI, the API, every WebSocket stream, and the MQTT
              bridge that subscribes the rovers' topics
              {health?.mqtt?.host ? <> at <span className="font-mono">{health.mqtt.host}:{health.mqtt.port ?? "--"}</span></> : null}.
            </p>
          </div>
          <div>
            <div className="font-semibold text-slate-100">Public access</div>
            <p className="mt-1 text-slate-400">
              {cat?.public_note ??
                "Optional. Run Publish-Public.bat to expose this laptop's app on a public HTTPS URL (Cloudflare Tunnel). Password required."}
            </p>
          </div>
        </div>
      </Card>
    </div>
  );
}

// ---- console availability -------------------------------------------------

/**
 * WHY THE CONSOLE TAB IS OR IS NOT THERE.
 *
 * App.tsx hides Control, Drive, Devices and Terminal whenever
 * /api/auth-status returns controls_disabled — and a hidden tab explains
 * nothing. This page is never hidden, so the explanation lives here, derived
 * from the same field the router uses.
 */
function ConsoleAvailability({ auth }: { auth: AuthStatus | null }) {
  if (!auth) {
    return (
      <Card>
        <CardHeader title="The Console tab" subtitle="Terminal · SSH + local shell" />
        <div className="text-sm text-slate-400">
          Reading <span className="font-mono">/api/auth-status</span>… if this never resolves, the
          backend is not answering and every tab that needs it will be missing.
        </div>
      </Card>
    );
  }

  const disabled = !!auth.controls_disabled;
  const cloud = auth.role === "cloud";

  return (
    <Card>
      <CardHeader
        title="The Console tab"
        subtitle="Terminal · SSH + local shell"
        right={<span className={disabled ? "chip-warn" : "chip-ok"}>{disabled ? "hidden here" : "available"}</span>}
      />
      {disabled ? (
        <>
          <p className="text-sm leading-relaxed text-slate-300">
            <b>The Console is not in the tab bar right now, and that is deliberate.</b>{" "}
            {cloud
              ? "This is the cloud deployment. There is no laptop behind it to open a shell on, so the backend refuses every /ws/term/* connection outright — the tab would only ever show a dead terminal."
              : "Safe mode is on and you reached this page over the public link. The backend refuses /ws/term/* for public visitors (close code 1008), so the tab is hidden rather than left to fail."}
          </p>
          <p className="mt-2 text-sm leading-relaxed text-slate-400">
            Control, Drive and Devices are hidden by the same switch — everything that can act on the
            HQ laptop or its network. Live rover data (LiDAR, Camera, Thermal, Analyst, AWS) stays
            available here.
          </p>
          <p className="mt-2 text-sm leading-relaxed text-slate-400">
            To use it: open the dashboard <b>on the HQ laptop</b> — its own window, or{" "}
            <code className="rounded bg-black/50 px-1 py-0.5 text-xs">http://127.0.0.1:8010</code>, or
            its LAN address from another device on the same network.
          </p>
        </>
      ) : (
        <p className="text-sm leading-relaxed text-slate-300">
          <b>The Console is available</b> from this browser. It gives you an SSH session to any host
          this laptop can reach{auth.is_lan ? ", plus a real PowerShell on the laptop itself" : ""}.
          {!auth.is_lan && (
            <> The local-PowerShell half is LAN-only and will be refused from your address
            ({auth.client_host ?? "unknown"}); SSH still works.</>
          )}
        </p>
      )}
      <div className="mt-3 grid gap-2 text-xs sm:grid-cols-2 lg:grid-cols-4">
        <Fact label="Your address" value={auth.client_host ?? "--"} />
        <Fact label="On the LAN" value={auth.is_lan ? "yes" : "no"} />
        <Fact label="Arrived over tunnel" value={auth.is_remote === undefined ? "--" : auth.is_remote ? "yes" : "no"} />
        <Fact label="Backend role" value={auth.role ?? "--"} />
      </div>
      <p className="mt-2 text-[11px] leading-relaxed text-slate-500">
        Hiding the tab is cosmetic. The refusal is enforced server-side in{" "}
        <span className="font-mono">backend/auth.py</span> for the WebSocket routes themselves, so
        typing the URL gets you the same answer.
      </p>
    </Card>
  );
}

// ---- small pieces ---------------------------------------------------------

function BrokerChip({ health }: { health: Health | null }) {
  const m = health?.mqtt;
  if (!m) return <span className="chip">broker --</span>;
  if (m.connected) return <span className="chip-ok">broker connected</span>;
  if (m.auth_failed) return <span className="chip-bad">broker refused our credentials</span>;
  return <span className="chip-warn">broker not connected</span>;
}

function CredStatus({ health }: { health: Health | null }) {
  const m = health?.mqtt;
  if (!m) return null;
  const c = m.credentials;
  return (
    <div
      className={`mt-2 rounded-md border p-2 text-xs leading-relaxed ${
        m.connected
          ? "border-emerald-500/25 bg-emerald-500/5 text-emerald-100"
          : "border-rose-500/25 bg-rose-500/5 text-rose-100"
      }`}
    >
      <b>Right now:</b> username {c?.username_set ? <>set ({c.username})</> : <b>not set</b>}, password{" "}
      {c?.password_set ? "set" : <b>not set</b>}, connection{" "}
      {m.connected ? "established" : m.auth_failed ? "REFUSED by the broker" : "not established"}.
      {m.problem && <div className="mt-1 font-mono text-[11px] opacity-90">{m.problem}</div>}
    </div>
  );
}

function ThingsStatus({ health }: { health: Health | null }) {
  const seen = health?.mqtt?.things_seen;
  if (!seen) return null;
  return (
    <div className="mt-2 rounded-md border border-white/5 bg-black/40 p-2 text-xs text-slate-300">
      <b>Right now:</b>{" "}
      {seen.length === 0
        ? "no rover has published to this backend since it started. With the rover powered off that is the expected reading."
        : <>heard from {seen.map((s) => <span key={s} className="mr-1.5 font-mono text-ember-200">{s}</span>)}</>}
    </div>
  );
}

function Step({ n, title, children }: { n: number; title: string; children: ReactNode }) {
  return (
    <div className="mb-4 border-l-2 border-white/10 pl-4">
      <div className="flex items-baseline gap-2">
        <span className="font-mono text-xs text-ember-300">{n}</span>
        <span className="font-semibold text-slate-100">{title}</span>
      </div>
      <div className="mt-1 space-y-1.5 text-xs leading-relaxed text-slate-400">{children}</div>
    </div>
  );
}

function Snippet({ text, tone = "slate" }: { text: string; tone?: "slate" | "amber" }) {
  const [copied, setCopied] = useState(false);
  const copy = () => {
    navigator.clipboard?.writeText(text).then(
      () => { setCopied(true); window.setTimeout(() => setCopied(false), 1500); },
      () => { /* clipboard blocked (insecure origin) — the text is selectable anyway */ },
    );
  };
  return (
    <div className="group relative my-1.5">
      <pre
        className={`overflow-x-auto rounded-md border p-2.5 pr-16 font-mono text-[11px] leading-relaxed ${
          tone === "amber"
            ? "border-amber-500/20 bg-black/50 text-amber-100"
            : "border-white/5 bg-black/60 text-slate-200"
        }`}
      >{text}</pre>
      <button
        onClick={copy}
        className="absolute right-2 top-2 rounded border border-white/10 bg-black/70 px-2 py-0.5 text-[10px] text-slate-300 hover:border-ember-500/40 hover:text-ember-200"
      >
        {copied ? "copied" : "copy"}
      </button>
    </div>
  );
}

function Fact({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded-lg border border-white/5 bg-black/30 px-3 py-2">
      <div className="lbl">{label}</div>
      <div className="mt-0.5 font-mono text-xs text-slate-200">{value}</div>
    </div>
  );
}

/** Port from a "host:port" string, defaulting the way the backend does. */
function portOf(hostPort: string): string {
  const i = hostPort.lastIndexOf(":");
  return i > -1 ? hostPort.slice(i + 1) : "8000";
}

function humanBytes(n: number): string {
  const units = ["B", "KB", "MB", "GB"];
  let i = 0, v = n;
  while (v >= 1024 && i < units.length - 1) { v /= 1024; i++; }
  return `${v.toFixed(v >= 100 ? 0 : 1)} ${units[i]}`;
}

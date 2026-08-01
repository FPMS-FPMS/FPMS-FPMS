import { useEffect, useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { apiGet } from "../lib/api";
import { useThings } from "../lib/things";

type Iface = { name: string; ip: string; netmask: string; cidr: string; is_up: boolean; is_loopback: boolean };
type Host = { ip: string; hostname: string | null; open_ports: number[]; ssh_banner: string | null; guess: string };
type Thing = { name: string; arn: string; attributes: Record<string, string> };

type SshResult = {
  ok: boolean; ip: string; username: string; error: string | null;
  hostname: string | null; os_release: string | null; kernel: string | null;
  is_orange_pi: boolean; uptime: string | null;
};

type ProvisionResult = {
  thing: string; iot_endpoint: string;
  install: { ok: boolean; exit_code?: number; stdout?: string; stderr?: string; error?: string };
};

export default function Devices() {
  const [ifaces, setIfaces] = useState<Iface[]>([]);
  const [cidr, setCidr] = useState("");
  const [scanning, setScanning] = useState(false);
  const [hosts, setHosts] = useState<Host[]>([]);
  const [things, setThings] = useState<Thing[]>([]);
  const [activeHost, setActiveHost] = useState<Host | null>(null);
  const [banner, setBanner] = useState<string | null>(null);
  // Registration and reporting are different facts, and this page only ever
  // showed the first. A Thing that exists in the registry but has never
  // published looks identical here to a healthy rover — which is the state
  // after a provision that half-worked, and the one worth seeing.
  const reporting = useThings();

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
        <div className="lbl">Devices · onboarding + inventory</div>
        <h1 className="h-page mt-1">Find and provision rovers on your network</h1>
        <p className="mt-2 max-w-3xl text-sm text-slate-400">
          Scan a local subnet for candidate Orange Pi 5B devices, verify with SSH,
          and register them as <span className="text-ember-300">AWS IoT Things</span>
          {" "}(via LocalStack). Once provisioned, the device streams telemetry to
          the LiDAR / Camera / Thermal pages automatically.
        </p>
      </div>

      {/* Scanner */}
      <Card>
        <CardHeader
          title="Network scanner"
          subtitle="Step 1 · discover"
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
            /23 or smaller.
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
          <CardHeader title="Discovered hosts" subtitle="Step 2 · verify & provision" right={<span className="chip">{hosts.length}</span>} />
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
                      {h.open_ports.join(", ")}
                    </td>
                    <td className="px-3 py-2 max-w-[280px] truncate font-mono text-[10px] text-slate-500">
                      {h.ssh_banner ?? "—"}
                    </td>
                    <td className="px-3 py-2 text-right">
                      <button
                        className="btn"
                        disabled={!h.open_ports.includes(22)}
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

      {/* AWS IoT Things */}
      <Card>
        <CardHeader
          title="Registered AWS IoT Things"
          subtitle="Live from IoT Core (LocalStack)"
          right={
            <div className="flex items-center gap-2">
              <button className="btn" onClick={loadThings}>Refresh</button>
              <span className="chip" title="Things in the registry">{things.length} registered</span>
              <span
                className={reporting.length ? "chip-ok" : "chip-warn"}
                title="Things the broker has actually heard from"
              >
                {reporting.length} reporting
              </span>
            </div>
          }
        />
        {things.length === 0 ? (
          <div className="text-sm text-slate-500">
            No Things yet. Scan a subnet and provision one — it lands here.
          </div>
        ) : (
          <ul className="grid gap-2 sm:grid-cols-2 lg:grid-cols-3">
            {things.map((t) => (
              <li key={t.arn} className="rounded-lg border border-white/5 bg-black/30 p-3">
                <div className="flex items-center justify-between gap-2">
                  <span className="font-semibold text-slate-100">{t.name}</span>
                  <span
                    className={reporting.includes(t.name) ? "chip-ok text-[10px]" : "chip-warn text-[10px]"}
                    title={
                      reporting.includes(t.name)
                        ? "Publishing telemetry the broker has seen"
                        : "Registered, but nothing has been heard from it — the publisher may not be running"
                    }
                  >
                    {reporting.includes(t.name) ? "reporting" : "silent"}
                  </span>
                </div>
                <div className="mt-1 truncate font-mono text-[10px] text-slate-500" title={t.arn}>{t.arn}</div>
                {Object.entries(t.attributes).length > 0 && (
                  <div className="mt-2 flex flex-wrap gap-1">
                    {Object.entries(t.attributes).map(([k, v]) => (
                      <span key={k} className="chip text-[10px]">{k}: {v}</span>
                    ))}
                  </div>
                )}
              </li>
            ))}
          </ul>
        )}
        {/* Publishing without being registered is the other half of the same
            check, and it is how a hand-installed rover stays invisible to every
            page that iterates the registry. */}
        {reporting.filter((r) => !things.some((t) => t.name === r)).length > 0 && (
          <div className="mt-3 rounded-lg border border-amber-500/30 bg-amber-500/5 p-3 text-xs text-amber-100/90">
            <b>Publishing but not registered:</b>{" "}
            {reporting
              .filter((r) => !things.some((t) => t.name === r))
              .map((r) => (
                <span key={r} className="mr-1.5 font-mono">{r}</span>
              ))}
            <div className="mt-1 text-amber-200/70">
              The broker is receiving telemetry from these, but they have no
              Thing in the registry — provisioned by hand, or registered against
              a different endpoint.
            </div>
          </div>
        )}
      </Card>

      {activeHost && (
        <ProvisionDialog
          host={activeHost}
          onClose={() => setActiveHost(null)}
          onDone={() => { loadThings(); }}
        />
      )}
    </div>
  );
}

function ProvisionDialog({
  host, onClose, onDone,
}: {
  host: Host;
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
                    <div className="text-slate-300"><b>hostname</b>: {ssh.hostname}</div>
                    <div className="text-slate-300"><b>kernel</b>: {ssh.kernel}</div>
                    <div className="whitespace-pre-wrap text-slate-400">{ssh.os_release}</div>
                    {ssh.uptime && <div className="text-slate-400"><b>uptime</b>: {ssh.uptime}</div>}
                  </div>
                ) : (
                  <div className="text-rose-300">✗ {ssh.error}</div>
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
                Creates the Thing + certs in IoT Core, SCPs a systemd publisher to the device, enables it.
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
                  thing={prov.thing} · endpoint={prov.iot_endpoint}
                </div>
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

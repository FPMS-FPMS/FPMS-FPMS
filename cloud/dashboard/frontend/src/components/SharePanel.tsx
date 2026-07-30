import { useEffect, useState } from "react";
import { Card, CardHeader } from "./Card";
import { apiGet } from "../lib/api";

type Url = { kind: string; url: string; description: string; ip?: string; interface?: string; recommended?: boolean };
type Firewall = { supported: boolean; allowed: boolean | null; rule_present: boolean; platform: string; port?: number; rule_name?: string };
type Tunnel = {
  running: boolean; url: string | null; started_at: number | null;
  password_set: boolean; cloudflared_available: boolean; recent_output: string[];
  // The permanent workers.dev front door that redirects to `url`. Stays valid
  // across restarts, unlike `url`, which is a fresh random hostname each time.
  gateway_url: string | null; gateway_configured: boolean;
  gateway_published: boolean; gateway_error: string | null;
};
type Network = { urls: Url[]; firewall: Firewall; tunnel: Tunnel };

export default function SharePanel({ compact = false }: { compact?: boolean }) {
  const [net, setNet] = useState<Network | null>(null);
  const [busy, setBusy] = useState<"" | "firewall" | "tunnel-start" | "tunnel-stop">("");
  const [msg, setMsg] = useState<string | null>(null);
  const [copied, setCopied] = useState<string | null>(null);
  const [showDiag, setShowDiag] = useState(false);

  const load = () => apiGet<Network>("/api/network").then(setNet).catch(() => {});
  useEffect(() => { load(); const id = window.setInterval(load, 4000); return () => window.clearInterval(id); }, []);

  const copy = (url: string) => {
    navigator.clipboard.writeText(url).then(() => {
      setCopied(url);
      window.setTimeout(() => setCopied(null), 1500);
    });
  };

  const allowFirewall = async () => {
    setBusy("firewall"); setMsg(null);
    try {
      const r = await fetch("/api/network/firewall/allow", { method: "POST", credentials: "include" });
      const d = await r.json();
      if (d.ok) { setMsg("Firewall rule added — LAN devices can now reach the dashboard."); load(); }
      else setMsg("Firewall change was cancelled or failed. You can add it manually in an Admin PowerShell:\n" + (d.manual_command ?? ""));
    } finally { setBusy(""); }
  };

  const startTunnel = async () => {
    setBusy("tunnel-start"); setMsg(null);
    try {
      const r = await fetch("/api/network/tunnel/start", { method: "POST", credentials: "include" });
      const d = await r.json();
      if (d.ok) load(); else setMsg(d.error ?? "Failed to start tunnel.");
    } finally { setBusy(""); }
  };

  const stopTunnel = async () => {
    setBusy("tunnel-stop"); setMsg(null);
    try { await fetch("/api/network/tunnel/stop", { method: "POST", credentials: "include" }); load(); }
    finally { setBusy(""); }
  };

  if (!net) return <Card><div className="text-sm text-slate-500">Discovering network…</div></Card>;

  const lanUrls = net.urls.filter((u) => u.kind === "lan");
  const primary = lanUrls.find((u) => u.recommended) ?? lanUrls[0] ?? net.urls[0];
  // The relay hostname rotates on every restart; the gateway URL doesn't. Always
  // put the durable one in front of the user so that's what gets shared.
  const relayUrl = net.tunnel.url;
  const permanentUrl = net.tunnel.gateway_url;
  const publicUrl = permanentUrl ?? relayUrl;

  return (
    <div className="space-y-6">
      {/* Firewall + tunnel status banner */}
      {net.firewall.supported && !net.firewall.allowed && (
        <div className="flex flex-wrap items-center justify-between gap-3 rounded-lg border border-amber-500/30 bg-amber-500/10 p-4">
          <div className="text-sm text-amber-100">
            <div className="font-semibold">LAN access is currently blocked.</div>
            <div className="mt-0.5 text-xs text-amber-200/80">
              Windows Firewall is dropping inbound to port {net.firewall.port}.
              Devices on your WiFi (phones, laptops) can't reach the dashboard until this is allowed.
            </div>
          </div>
          <button className="btn-primary" onClick={allowFirewall} disabled={busy === "firewall"}>
            {busy === "firewall" ? "Waiting for UAC…" : "Allow LAN access (UAC)"}
          </button>
        </div>
      )}

      {/* Primary "give this to your phone" card */}
      <Card glow>
        <CardHeader
          title={permanentUrl ? "Permanent URL" : publicUrl ? "Public URL" : "Your LAN URL"}
          subtitle={
            permanentUrl
              ? "Bookmark this once — it survives restarts and follows the tunnel by itself"
              : publicUrl
                ? "Anyone in the world with the password can open this — but it changes on every restart"
                : "Anyone on the same WiFi can open this"
          }
          right={
            <div className="flex gap-2">
              {publicUrl
                ? <button className="btn-danger" onClick={stopTunnel} disabled={busy === "tunnel-stop"}>
                    {busy === "tunnel-stop" ? "…" : "Stop public URL"}
                  </button>
                : <button
                    className="btn-primary"
                    onClick={startTunnel}
                    disabled={busy === "tunnel-start" || !net.tunnel.password_set || !net.tunnel.cloudflared_available}
                    title={
                      !net.tunnel.password_set
                        ? "Set FPMS_PASSWORD first (safer)"
                        : !net.tunnel.cloudflared_available
                          ? "cloudflared not found — install from cloudflare/cloudflared releases"
                          : ""
                    }
                  >
                    {busy === "tunnel-start" ? "Starting tunnel…" : "Publish to public URL"}
                  </button>}
            </div>
          }
        />

        <PrimaryLinkRow url={publicUrl ?? primary?.url ?? ""} copied={copied} onCopy={copy} />

        {!publicUrl && !net.tunnel.password_set && (
          <div className="mt-4 rounded-md border border-amber-500/20 bg-amber-500/5 p-3 text-xs text-amber-200">
            <b>Before going public:</b> stop the app, set{" "}
            <code className="rounded bg-black/40 px-1">FPMS_PASSWORD=your-strong-password</code>{" "}
            in the same terminal (or via System → Environment Variables), then relaunch.
            Without a password, publishing is disabled.
          </div>
        )}

        {publicUrl && (
          <div className="mt-4 rounded-md border border-emerald-500/20 bg-emerald-500/5 p-3 text-xs text-emerald-200">
            ✓ Live at <code className="font-mono">{publicUrl}</code>. Share this URL. Stopping the tunnel here revokes access instantly.
            {permanentUrl && relayUrl && (
              <div className="mt-1.5 text-emerald-300/70">
                Currently relaying via <code className="font-mono">{relayUrl}</code> — that part rotates, the link above doesn't.
              </div>
            )}
          </div>
        )}

        {/* Permanent URL exists but this tunnel never reached the Worker — the
            bookmarked link is pointing somewhere stale. Loud, because it looks
            fine from this laptop and only fails for everyone else. */}
        {permanentUrl && relayUrl && !net.tunnel.gateway_published && (
          <div className="mt-4 rounded-md border border-amber-500/30 bg-amber-500/10 p-3 text-xs text-amber-200">
            <b>Permanent URL is out of date.</b> The tunnel is up, but registering it with
            the gateway failed{net.tunnel.gateway_error ? <> — <code className="font-mono">{net.tunnel.gateway_error}</code></> : null}.
            Visitors using the permanent link will see the offline page until this succeeds.
          </div>
        )}
      </Card>

      {/* All LAN URLs */}
      {!compact && (
        <Card>
          <CardHeader
            title="All URLs this app is reachable at"
            subtitle="From this laptop and the LAN"
            right={net.firewall.supported && net.firewall.allowed ? <span className="chip-ok">firewall OK</span> : null}
          />
          <ul className="space-y-2">
            {net.urls.map((u) => (
              <li key={u.url} className="flex items-center justify-between gap-3 rounded-lg border border-white/5 bg-black/30 px-3 py-2">
                <div>
                  <div className="font-mono text-sm text-slate-100">{u.url}</div>
                  <div className="text-xs text-slate-500">{u.description}</div>
                </div>
                <div className="flex items-center gap-1.5">
                  <span className={u.kind === "lan" ? "chip-ok" : "chip"}>{u.kind}</span>
                  <button className="btn text-xs" onClick={() => copy(u.url)}>
                    {copied === u.url ? "copied" : "copy"}
                  </button>
                </div>
              </li>
            ))}
          </ul>
        </Card>
      )}

      {msg && (
        <div className="rounded-md border border-white/10 bg-white/5 p-3 text-xs text-slate-200 whitespace-pre-wrap">{msg}</div>
      )}

      {/* Diagnostic */}
      <div>
        <button
          className="text-xs text-slate-400 hover:text-slate-200 underline underline-offset-2"
          onClick={() => setShowDiag((v) => !v)}
        >
          {showDiag ? "Hide" : "Phone can't connect? Show diagnostics"}
        </button>
        {showDiag && <ConnectivityDiag net={net} />}
      </div>
    </div>
  );
}

function ConnectivityDiag({ net }: { net: Network }) {
  const publicUrl = net.tunnel.url;
  const permanentUrl = net.tunnel.gateway_url;
  const lanUrl = net.urls.find((u) => u.kind === "lan")?.url;

  const checks: { title: string; ok: boolean; body: React.ReactNode }[] = [
    {
      title: "1. Windows Firewall allows inbound TCP 8000",
      ok: !!net.firewall.allowed,
      body: net.firewall.allowed
        ? <span>Rule <code className="font-mono">{net.firewall.rule_name}</code> is active.</span>
        : <span>Click <b>Allow LAN access (UAC)</b> above. If UAC was denied, run in an Admin PowerShell:
            <pre className="mt-1 rounded bg-black/60 p-2 font-mono text-[10px] text-slate-200">
{`netsh advfirewall firewall add rule name="FPMS Dashboard Inbound 8000" dir=in action=allow protocol=TCP localport=8000 profile=any`}
            </pre>
          </span>,
    },
    {
      title: "2. Phone is on the SAME WiFi as this laptop",
      ok: false,
      body: <span>
        Cellular data and different WiFi networks (guest network!) cannot reach{" "}
        {lanUrl ? <code className="font-mono">{lanUrl}</code> : "a LAN URL"}.
        Check your phone's WiFi settings and connect to the same network as this laptop.
      </span>,
    },
    {
      title: "3. Router isn't isolating devices from each other",
      ok: false,
      body: <span>
        Some routers (guest networks, hotel WiFi, corporate WiFi) block device-to-device
        traffic even when both devices are on the network. Symptom: pinging from phone
        to laptop fails. Fix: use your home WiFi's primary network, or use the tunnel.
      </span>,
    },
    {
      title: "4. Public URL (works everywhere, including cellular)",
      ok: !!publicUrl,
      body: publicUrl
        ? <span>Public URL is live: <code className="font-mono">{publicUrl}</code>. Open that from any device on any network.</span>
        : <span>
            The only 100%-works path from a phone that isn't on your WiFi.
            Click <b>Publish to public URL</b> above (needs <code>FPMS_PASSWORD</code> set).
            Or deploy to AWS App Runner — see <code>DEPLOY-TO-AWS.md</code>.
          </span>,
    },
    {
      title: "5. Permanent URL is registered and pointing at the live tunnel",
      ok: !!permanentUrl && net.tunnel.gateway_published,
      body: !permanentUrl
        ? <span>
            No permanent URL configured, so every restart mints a new
            <code className="font-mono"> *.trycloudflare.com </code> address and any link you
            already shared stops resolving — the browser says <i>"server cannot be found"</i>.
            Deploy the gateway Worker (<code>gateway/</code>) to get one address that never changes.
          </span>
        : net.tunnel.gateway_published
          ? <span>
              <code className="font-mono">{permanentUrl}</code> is forwarding to the current
              tunnel. This is the link to bookmark and hand out.
            </span>
          : <span>
              <code className="font-mono">{permanentUrl}</code> exists but isn't pointing at
              this tunnel{net.tunnel.gateway_error ? <> — <code className="font-mono">{net.tunnel.gateway_error}</code></> : null}.
              Visitors get the offline page. Check the gateway secret matches the one set with
              <code> wrangler secret put</code>.
            </span>,
    },
  ];

  return (
    <div className="mt-3 rounded-lg border border-white/5 bg-black/40 p-4">
      <div className="lbl mb-2">Connectivity diagnostic</div>
      <ol className="space-y-3">
        {checks.map((c, i) => (
          <li key={i} className="flex items-start gap-3">
            <span className={`mt-0.5 inline-flex h-5 w-5 flex-none items-center justify-center rounded-full text-[11px] font-bold ${
              c.ok ? "bg-emerald-500/20 text-emerald-300" : "bg-amber-500/20 text-amber-300"
            }`}>
              {c.ok ? "✓" : "?"}
            </span>
            <div className="text-xs text-slate-300">
              <div className="font-semibold text-slate-100">{c.title}</div>
              <div className="mt-0.5 leading-relaxed text-slate-400">{c.body}</div>
            </div>
          </li>
        ))}
      </ol>
      <div className="mt-4 border-t border-white/5 pt-3 text-[11px] text-slate-500">
        Reading order: if #1 and #2 are both ✓ and it still doesn't work, #3 (router isolation)
        is almost certainly the cause. Fall back to #4 (public URL) — that path is immune to all three.
      </div>
    </div>
  );
}

function PrimaryLinkRow({ url, copied, onCopy }: { url: string; copied: string | null; onCopy: (u: string) => void }) {
  if (!url) return <div className="text-sm text-slate-500">No URL available yet.</div>;
  const qr = `/qr?data=${encodeURIComponent(url)}`;
  return (
    <div className="grid gap-6 md:grid-cols-[auto_1fr] items-center">
      <div className="rounded-xl bg-white p-3 shadow-inner shadow-black/40">
        <img src={qr} alt="QR code" width={220} height={220} className="block" />
      </div>
      <div>
        <div className="lbl">Open this URL from any device</div>
        <div className="mt-1 flex items-center gap-2">
          <a href={url} className="truncate font-mono text-lg font-semibold text-ember-300 hover:underline" target="_blank" rel="noreferrer">{url}</a>
          <button className="btn text-xs" onClick={() => onCopy(url)}>
            {copied === url ? "✓ copied" : "copy"}
          </button>
        </div>
        <ol className="mt-4 space-y-1.5 text-sm text-slate-300">
          <li><b className="text-slate-100">Phone / tablet:</b> point the camera at the QR — Safari or Chrome opens the app.</li>
          <li><b className="text-slate-100">Then:</b> Share → Add to Home Screen (iOS) or install icon (Android) for a fullscreen app.</li>
          <li><b className="text-slate-100">Password:</b> if you set one, the dashboard prompts on first open.</li>
        </ol>
      </div>
    </div>
  );
}

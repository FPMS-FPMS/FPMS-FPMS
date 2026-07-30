import { useEffect, useState } from "react";
import { Card, CardHeader } from "./Card";
import { apiGet } from "../lib/api";

type AlertsStatus = {
  configured: boolean;
  provider: "smtp" | "resend";
  providers: {
    smtp:   { ready: boolean; user: string };
    resend: { ready: boolean; from: string };
  };
  smtp_host: string;
  smtp_port: number;
  smtp_user: string;
  alert_to: string;
  cooldown_s: number;
  settings_path: string;
  recent: {
    ts: number; ok: boolean; error?: string; subject?: string; to?: string;
    provider?: string; skipped?: boolean; reason?: string;
  }[];
};

type Mode = null | "gmail" | "resend";

export default function AlertsCard() {
  const [st, setSt] = useState<AlertsStatus | null>(null);
  const [busy, setBusy] = useState<"" | "test" | "signin" | "signout">("");
  const [msg, setMsg] = useState<string | null>(null);
  const [mode, setMode] = useState<Mode>(null);

  // Gmail form
  const [gmail, setGmail] = useState("aryan0419wadhawan@gmail.com");
  const [gpw, setGpw] = useState("");

  // Resend form
  const [rkey, setRkey] = useState("");
  const [rto, setRto] = useState("aryan0419wadhawan@gmail.com");

  const load = () => apiGet<AlertsStatus>("/api/alerts/status").then(setSt).catch(() => {});
  useEffect(() => { load(); const id = window.setInterval(load, 5000); return () => window.clearInterval(id); }, []);

  const test = async () => {
    setBusy("test"); setMsg(null);
    try {
      const r = await fetch("/api/alerts/test", { method: "POST", credentials: "include",
        headers: { "Content-Type": "application/json" }, body: JSON.stringify({}) });
      const d = await r.json();
      if (d.ok) setMsg(`✓ Test email sent to ${d.to} via ${d.provider}. Check the inbox (and spam).`);
      else if (d.skipped) setMsg(`⚠ Skipped: ${d.reason}`);
      else setMsg(`✗ ${d.error ?? "failed"}`);
      load();
    } finally { setBusy(""); }
  };

  const signinGmail = async (e: React.FormEvent) => {
    e.preventDefault(); setBusy("signin"); setMsg(null);
    try {
      const r = await fetch("/api/alerts/signin", { method: "POST", credentials: "include",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ user: gmail, pw: gpw, to: gmail }) });
      const d = await r.json();
      if (d.ok) { setMsg(`✓ Signed in as ${d.user} via Gmail SMTP.`); setMode(null); setGpw(""); load(); }
      else setMsg(`✗ ${d.error ?? "sign-in failed"}`);
    } finally { setBusy(""); }
  };

  const signinResend = async (e: React.FormEvent) => {
    e.preventDefault(); setBusy("signin"); setMsg(null);
    try {
      const r = await fetch("/api/alerts/signin-resend", { method: "POST", credentials: "include",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ api_key: rkey, to: rto }) });
      const d = await r.json();
      if (d.ok) { setMsg(`✓ Signed in via Resend. Emails will be sent to ${rto}.`); setMode(null); setRkey(""); load(); }
      else setMsg(`✗ ${d.error ?? "sign-in failed"}`);
    } finally { setBusy(""); }
  };

  const signout = async () => {
    setBusy("signout"); setMsg(null);
    try { await fetch("/api/alerts/signout", { method: "POST", credentials: "include" });
      setMsg("Signed out. Saved credentials removed."); load(); }
    finally { setBusy(""); }
  };

  return (
    <Card>
      <CardHeader
        title="HQ email alerts"
        subtitle="Fires when a rover publishes fire-detected or alert events"
        right={
          <div className="flex items-center gap-2">
            <span className={st?.configured ? "chip-ok" : "chip-warn"}>
              {st?.configured ? `Signed in · ${st.provider}` : "Not signed in"}
            </span>
            {st?.configured && (
              <>
                <button className="btn-primary" onClick={test} disabled={busy === "test"}>
                  {busy === "test" ? "Sending…" : "Send test email"}
                </button>
                <button className="btn" onClick={signout} disabled={busy === "signout"}>Sign out</button>
              </>
            )}
          </div>
        }
      />

      {!st?.configured && (
        <div className="mb-4 grid gap-3 md:grid-cols-2">
          <ProviderCard
            title="Resend"
            tag="Recommended · easiest"
            body="Free tier · 3000 emails/month. 60-second signup with your Gmail address. No 2FA, no App Password."
            cta="Sign in with Resend"
            selected={mode === "resend"}
            onClick={() => setMode(mode === "resend" ? null : "resend")}
          />
          <ProviderCard
            title="Gmail SMTP"
            tag="Direct · needs App Password"
            body="Uses your own Gmail. Requires a 16-char Google App Password (blocked on some Family Link / Workspace accounts)."
            cta="Sign in with Gmail"
            selected={mode === "gmail"}
            onClick={() => setMode(mode === "gmail" ? null : "gmail")}
          />
        </div>
      )}

      {mode === "resend" && !st?.configured && (
        <form onSubmit={signinResend} className="mb-4 rounded-lg border border-white/10 bg-black/40 p-4">
          <div className="lbl mb-2">Resend — 60-second sign-up</div>
          <ol className="mb-3 space-y-1 text-xs text-slate-300">
            <li>1. Open <a className="text-ember-300 underline" href="https://resend.com/signup" target="_blank" rel="noreferrer">resend.com/signup</a>, sign up with your Gmail (no phone verify needed).</li>
            <li>2. Verify your email (click the link Resend sends you).</li>
            <li>3. Go to <a className="text-ember-300 underline" href="https://resend.com/api-keys" target="_blank" rel="noreferrer">resend.com/api-keys</a> → <b>Create API Key</b> → Permission "Sending access" → Create.</li>
            <li>4. Copy the key (starts with <code>re_</code>) and paste below.</li>
          </ol>
          <div className="grid gap-3 md:grid-cols-2">
            <Field label="Resend API key" value={rkey} onChange={setRkey} placeholder="re_xxxxxxxxxxxxxxxx" mono type="password" />
            <Field label="Send alerts to" value={rto} onChange={setRto} placeholder="you@example.com" type="email" />
          </div>
          <div className="mt-3 flex items-center justify-between">
            <div className="text-[10px] text-slate-500">
              Free tier sends from <code>onboarding@resend.dev</code>. Add your own domain later on the Resend dashboard.
            </div>
            <button className="btn-primary" disabled={busy === "signin" || !rkey}>
              {busy === "signin" ? "Saving…" : "Sign in"}
            </button>
          </div>
        </form>
      )}

      {mode === "gmail" && !st?.configured && (
        <form onSubmit={signinGmail} className="mb-4 rounded-lg border border-white/10 bg-black/40 p-4">
          <div className="lbl mb-2">Gmail — via App Password</div>
          <p className="mb-3 text-xs leading-relaxed text-slate-400">
            Google requires a 16-character App Password.{" "}
            <a className="text-ember-300 underline" href="https://myaccount.google.com/apppasswords" target="_blank" rel="noreferrer">Create one here</a>
            {" "}(needs 2-Step Verification). If you get "error generating your app password", switch to Resend above — it usually means Family Link or Workspace policy is blocking it.
          </p>
          <div className="grid gap-3 md:grid-cols-2">
            <Field label="Your Gmail address" value={gmail} onChange={setGmail} type="email" placeholder="you@gmail.com" autoComplete="username" />
            <Field label="16-character App Password" value={gpw} onChange={setGpw} type="password" placeholder="xxxx xxxx xxxx xxxx" mono autoComplete="current-password" />
          </div>
          <div className="mt-3 flex items-center justify-between">
            <div className="text-[10px] text-slate-500">
              Stored locally at <code>{st?.settings_path ?? "~/.fpms/settings.json"}</code>.
            </div>
            <button className="btn-primary" disabled={busy === "signin" || !gmail || !gpw}>
              {busy === "signin" ? "Saving…" : "Sign in"}
            </button>
          </div>
        </form>
      )}

      <div className="grid gap-3 text-sm md:grid-cols-3">
        <Info label="Provider" value={st?.provider ?? "—"} />
        <Info label="Recipient" value={st?.alert_to ?? "—"} />
        <Info label="Cooldown" value={st ? `${st.cooldown_s}s per event` : "—"} />
      </div>

      {msg && (
        <div className="mt-3 rounded-md border border-white/10 bg-white/5 px-3 py-2 text-xs text-slate-200">{msg}</div>
      )}

      {st && st.recent.length > 0 && (
        <div className="mt-4">
          <div className="lbl mb-1.5">Recent alerts ({st.recent.length})</div>
          <ul className="space-y-1">
            {st.recent.slice().reverse().slice(0, 6).map((r, i) => (
              <li key={i} className="flex items-center justify-between rounded-md border border-white/5 bg-black/30 px-2 py-1.5 text-xs">
                <span className={r.ok ? "text-emerald-300" : r.skipped ? "text-slate-400" : "text-rose-300"}>
                  {r.ok ? "✓" : r.skipped ? "⋯" : "✗"} {r.subject ?? r.reason ?? r.error ?? "(no subject)"}
                </span>
                <span className="font-mono text-[10px] text-slate-500">
                  {new Date(r.ts * 1000).toLocaleTimeString()}
                </span>
              </li>
            ))}
          </ul>
        </div>
      )}
    </Card>
  );
}

function ProviderCard({ title, tag, body, cta, selected, onClick }: {
  title: string; tag: string; body: string; cta: string; selected: boolean; onClick: () => void;
}) {
  return (
    <button
      type="button"
      onClick={onClick}
      className={`text-left rounded-lg border p-4 transition ${
        selected ? "border-ember-500/50 bg-ember-500/10" : "border-white/10 bg-black/30 hover:bg-white/5"
      }`}
    >
      <div className="flex items-center justify-between">
        <div className="text-sm font-semibold text-slate-100">{title}</div>
        <span className="text-[10px] text-ember-300">{tag}</span>
      </div>
      <p className="mt-2 text-xs leading-relaxed text-slate-400">{body}</p>
      <div className={`mt-3 text-xs font-semibold ${selected ? "text-ember-200" : "text-slate-300"}`}>
        {selected ? "▾ Selected — fill in below" : `→ ${cta}`}
      </div>
    </button>
  );
}

function Info({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded-md border border-white/5 bg-black/30 px-3 py-2">
      <div className="lbl text-[10px]">{label}</div>
      <div className="mt-0.5 font-mono text-xs text-slate-100 truncate">{value}</div>
    </div>
  );
}

function Field({
  label, value, onChange, type = "text", placeholder, mono, autoComplete,
}: {
  label: string; value: string; onChange: (v: string) => void;
  type?: string; placeholder?: string; mono?: boolean; autoComplete?: string;
}) {
  return (
    <label className="block">
      <div className="lbl">{label}</div>
      <input
        type={type}
        value={value}
        placeholder={placeholder}
        onChange={(e) => onChange(e.target.value)}
        autoComplete={autoComplete}
        className={`mt-1 w-full rounded-lg border border-white/10 bg-black/40 px-3 py-2 text-sm text-slate-100 outline-none focus:border-ember-500/50 ${mono ? "font-mono" : ""}`}
      />
    </label>
  );
}

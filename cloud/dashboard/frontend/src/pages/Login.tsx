import { useState } from "react";

export default function Login({ onAuthed }: { onAuthed: () => void }) {
  const [pw, setPw] = useState("");
  const [err, setErr] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  const submit = async (e: React.FormEvent) => {
    e.preventDefault();
    setBusy(true); setErr(null);
    try {
      const r = await fetch("/api/login", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        credentials: "include",
        body: JSON.stringify({ password: pw }),
      });
      if (!r.ok) {
        const j = await r.json().catch(() => ({}));
        throw new Error(j.detail || "Invalid password");
      }
      onAuthed();
    } catch (e: any) {
      setErr(String(e.message || e));
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="min-h-screen flex items-center justify-center px-4">
      <form onSubmit={submit} className="card-glow w-full max-w-sm space-y-5 p-7">
        <div className="text-center">
          <div className="mx-auto mb-3 flex h-12 w-12 items-center justify-center">
            <FireMark />
          </div>
          <div className="lbl">FPMS</div>
          <h1 className="mt-1 text-2xl font-bold tracking-tight text-slate-100">
            Robotics Operations Console
          </h1>
          <p className="mt-2 text-xs text-slate-500">Enter the shared access password.</p>
        </div>

        <div>
          <label className="lbl">Password</label>
          <input
            type="password"
            autoFocus
            autoComplete="current-password"
            value={pw}
            onChange={(e) => setPw(e.target.value)}
            className="mt-1 w-full rounded-lg border border-white/10 bg-black/40 px-3 py-2.5 text-sm text-slate-100 outline-none focus:border-ember-500/60"
            placeholder="••••••••"
          />
        </div>

        {err && (
          <div className="rounded-md border border-rose-500/30 bg-rose-500/10 px-3 py-2 text-sm text-rose-200">
            {err}
          </div>
        )}

        <button
          className="btn-primary w-full justify-center py-2.5 text-sm font-semibold"
          disabled={busy || !pw}
        >
          {busy ? "Signing in…" : "Sign in"}
        </button>

        <p className="text-center text-[10px] text-slate-600">
          Session cookie · valid 30 days · survives app restarts.
        </p>
      </form>
    </div>
  );
}

function FireMark() {
  return (
    <svg width="48" height="48" viewBox="0 0 32 32">
      <defs>
        <linearGradient id="lg-fmg" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0" stopColor="#fbbf24" />
          <stop offset="1" stopColor="#c2410c" />
        </linearGradient>
      </defs>
      <rect width="32" height="32" rx="9" fill="#0b0f16" stroke="rgba(255,255,255,0.08)" />
      <path
        d="M16 4 C 10 12, 22 14, 16 20 C 22 24, 8 26, 16 28 C 8 24, 12 18, 10 14 C 12 16, 14 12, 16 4 Z"
        fill="url(#lg-fmg)"
      />
    </svg>
  );
}

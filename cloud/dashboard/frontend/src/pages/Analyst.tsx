import { useEffect, useRef, useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { apiGet } from "../lib/api";
import { useActiveThing } from "../lib/things";

type Report = {
  thing: string;
  verdict: "NOMINAL" | "ELEVATED" | "CRITICAL";
  concerns: string[];
  report: string;
  provider: string;
  snapshot: {
    streams_live: Record<string, boolean>;
    ages_s: Record<string, number | null>;
    lidar: any;
    detections: any[];
    thermal: any;
    pose: any;
  };
};

type Status = {
  providers: { "local-rules": boolean; "github-copilot": boolean; bedrock: boolean };
  default_provider: string;
};

type ChatTurn = { who: "you" | "analyst"; text: string; provider?: string };

export default function Analyst() {
  // Default to a rover that is actually reporting. Defaulting to "rover1"
  // made the analyst confidently announce "ROVER1 is silent" on a fleet whose
  // only live unit was rover2 — a correct statement about the wrong subject.
  const [picked, setPicked] = useState<string | null>(null);
  const { thing: active, things } = useActiveThing(picked);
  const thing = active ?? "rover1";
  const setThing = setPicked;
  const [rep, setRep] = useState<Report | null>(null);
  const [busy, setBusy] = useState(false);
  const [status, setStatus] = useState<Status | null>(null);
  const [chat, setChat] = useState<ChatTurn[]>([]);
  const [q, setQ] = useState("");
  const [sending, setSending] = useState(false);
  const scrollerRef = useRef<HTMLDivElement>(null);

  const loadReport = async () => {
    setBusy(true);
    try {
      const r = await apiGet<Report>(`/api/analyst/report?thing=${thing}`);
      setRep(r);
    } finally { setBusy(false); }
  };

  useEffect(() => {
    apiGet<Status>("/api/analyst/status").then(setStatus).catch(() => {});
    loadReport();
    const id = window.setInterval(loadReport, 8000);
    return () => window.clearInterval(id);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [thing]);

  useEffect(() => {
    scrollerRef.current?.scrollTo({ top: scrollerRef.current.scrollHeight, behavior: "smooth" });
  }, [chat]);

  const ask = async (e: React.FormEvent) => {
    e.preventDefault();
    const question = q.trim();
    if (!question) return;
    setQ(""); setSending(true);
    setChat((c) => [...c, { who: "you", text: question }]);
    try {
      const body = await fetch("/api/analyst/chat", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        credentials: "include",
        body: JSON.stringify({ question, thing }),
      }).then((r) => r.json());
      setChat((c) => [...c, { who: "analyst", text: body.answer, provider: body.provider }]);
    } catch (e: any) {
      setChat((c) => [...c, { who: "analyst", text: `Error: ${e}`, provider: "error" }]);
    } finally { setSending(false); }
  };

  const verdictClass =
    rep?.verdict === "CRITICAL" ? "chip-hot"
    : rep?.verdict === "ELEVATED" ? "chip-warn"
    : "chip-ok";

  return (
    <div className="space-y-6">
      <div className="flex items-end justify-between gap-4">
        <div>
          <div className="lbl">AI Analyst · small, local, honest</div>
          <h1 className="h-page mt-1">Live rover analysis + chat</h1>
          <p className="mt-2 max-w-3xl text-sm text-slate-400">
            Watches every sensor stream + recent S3 events and writes plain-English reports.
            Default provider is a local rule-based analyst (no API cost, no network).
            Set <code className="rounded bg-black/50 px-1 py-0.5 text-xs">GITHUB_COPILOT_TOKEN</code>
            {" "}or <code className="rounded bg-black/50 px-1 py-0.5 text-xs">FPMS_BEDROCK_MODEL_ID</code>
            {" "}to upgrade to Copilot or AWS Bedrock — same UI, better answers.
          </p>
        </div>
        <div className="flex flex-col items-end gap-2">
          <select
            value={thing}
            onChange={(e) => setThing(e.target.value)}
            className="rounded-lg border border-white/10 bg-black/40 px-3 py-1.5 text-sm text-slate-100 outline-none"
          >
            {/* Only offer rovers that have actually reported, so the selector
                can't point at a unit that was never provisioned. */}
            {(things.length ? things : ["rover1", "rover2"]).map((t) => (
              <option key={t} value={t}>{t}</option>
            ))}
          </select>
          {status && (
            <div className="flex gap-1">
              <span className={status.providers["local-rules"] ? "chip-ok" : "chip"}>local</span>
              <span className={status.providers["github-copilot"] ? "chip-ok" : "chip"}>copilot</span>
              <span className={status.providers.bedrock ? "chip-ok" : "chip"}>bedrock</span>
            </div>
          )}
        </div>
      </div>

      <div className="grid gap-6 lg:grid-cols-[1.3fr_1fr]">
        {/* Report */}
        <Card glow>
          <CardHeader
            title="Live analysis"
            subtitle={rep?.provider ?? "…"}
            right={
              <div className="flex items-center gap-2">
                {rep && <span className={verdictClass}>{rep.verdict}</span>}
                <button className="btn" disabled={busy} onClick={loadReport}>
                  {busy ? "Analyzing…" : "Refresh"}
                </button>
              </div>
            }
          />
          {!rep ? (
            <div className="text-sm text-slate-500">Generating first report…</div>
          ) : (
            <>
              <p className="text-sm leading-relaxed text-slate-200">{rep.report}</p>
              {rep.concerns.length > 0 && (
                <div className="mt-4">
                  <div className="lbl mb-1.5">Concerns</div>
                  <ul className="space-y-1">
                    {rep.concerns.map((c) => (
                      <li key={c} className="flex items-start gap-2 text-sm text-amber-200">
                        <span className="mt-1 h-1.5 w-1.5 flex-none rounded-full bg-amber-400" />
                        {c}
                      </li>
                    ))}
                  </ul>
                </div>
              )}
              {/* "live" and "silent" hide how live: a stream last heard 4s ago
                  and one last heard 40s ago both rendered as "live", and the
                  age is the number that says whether the report above is about
                  the rover now or the rover earlier. */}
              <div className="mt-5 grid grid-cols-4 gap-2">
                {Object.entries(rep.snapshot.streams_live).map(([k, v]) => {
                  const age = rep.snapshot.ages_s?.[k];
                  const ageText =
                    typeof age === "number" && Number.isFinite(age)
                      ? age < 90 ? `${age.toFixed(0)}s ago` : `${(age / 60).toFixed(0)}m ago`
                      : "never";
                  return (
                    <div
                      key={k}
                      className="rounded-md border border-white/5 bg-black/30 px-2 py-1.5 text-center"
                      title={`${k}: last packet ${ageText}`}
                    >
                      <div className="lbl text-[9px]">{k}</div>
                      <div className={`mt-0.5 text-xs font-semibold ${v ? "text-emerald-300" : "text-slate-500"}`}>
                        {v ? "live" : "silent"}
                      </div>
                      <div className="font-mono text-[9px] text-slate-500">{ageText}</div>
                    </div>
                  );
                })}
              </div>
            </>
          )}
        </Card>

        {/* Chat */}
        <Card>
          <CardHeader
            title="Ask the analyst"
            subtitle="Small, focused answers about the current live state"
          />
          <div
            ref={scrollerRef}
            className="mb-3 h-72 overflow-y-auto rounded-lg border border-white/5 bg-black/30 p-3"
          >
            {chat.length === 0 ? (
              <div className="space-y-2 text-xs text-slate-500">
                Try:
                <ul className="ml-3 mt-1 list-disc space-y-1">
                  <li>"How is {thing} doing?"</li>
                  <li>"Any hotspots?"</li>
                  <li>"What's the battery?"</li>
                  <li>"What can you see on camera?"</li>
                </ul>
              </div>
            ) : (
              <div className="space-y-3">
                {chat.map((m, i) => (
                  <div key={i} className={m.who === "you" ? "text-right" : ""}>
                    <div className={`inline-block max-w-[85%] rounded-lg px-3 py-2 text-sm ${
                      m.who === "you"
                        ? "bg-ember-500/15 text-ember-100 border border-ember-500/25"
                        : "bg-white/[0.03] text-slate-100 border border-white/10"
                    }`}>
                      {m.text}
                      {m.provider && m.who === "analyst" && (
                        <div className="mt-1 text-[9px] uppercase tracking-widest text-slate-500">
                          via {m.provider}
                        </div>
                      )}
                    </div>
                  </div>
                ))}
              </div>
            )}
          </div>
          <form onSubmit={ask} className="flex gap-2">
            <input
              value={q}
              onChange={(e) => setQ(e.target.value)}
              placeholder="Ask about the rover…"
              className="flex-1 rounded-lg border border-white/10 bg-black/40 px-3 py-2 text-sm text-slate-100 outline-none focus:border-ember-500/50"
            />
            <button className="btn-primary" disabled={sending || !q.trim()}>
              {sending ? "…" : "Send"}
            </button>
          </form>
        </Card>
      </div>
    </div>
  );
}

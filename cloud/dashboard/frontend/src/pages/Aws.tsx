import { useEffect, useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { apiGet } from "../lib/api";

type Inventory = {
  endpoint: string;
  region: string;
  mode?: "real-aws" | "local-emulation";
  emulated?: boolean;
  credentials_found?: boolean;
  message?: string;
  services: {
    iot: any;
    s3: any;
    lambda: any;
    sns: any;
    logs: any;
  };
};

type VerifyCheck = { name: string; ok: boolean; evidence: any; error: string | null };
type VerifyResult = { probe_id: string; probe_thing: string; passed: number; failed: number; checks: VerifyCheck[] };

export default function Aws() {
  const [inv, setInv] = useState<Inventory | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [refreshing, setRefreshing] = useState(false);
  const [verifying, setVerifying] = useState(false);
  const [verify, setVerify] = useState<VerifyResult | null>(null);

  const load = async () => {
    setRefreshing(true);
    try {
      const r = await apiGet<Inventory>("/api/aws/services");
      setInv(r);
      setErr(null);
    } catch (e: any) {
      setErr(String(e));
    } finally {
      setRefreshing(false);
    }
  };
  useEffect(() => {
    load();
    const id = window.setInterval(load, 5000);
    return () => window.clearInterval(id);
  }, []);

  return (
    <div className="space-y-6">
      {/* Never let emulated numbers pass for a real account — they look
          entirely plausible and would be wrong. */}
      {inv?.emulated && (
        <div className="rounded-lg border border-amber-500/30 bg-amber-500/10 p-4">
          <div className="text-sm font-semibold text-amber-100">
            Not real AWS — local emulation
          </div>
          <div className="mt-1 text-xs text-amber-200/80">
            {inv.message ??
              "Showing LocalStack and a local Thing registry, not your AWS account."}
          </div>
          <div className="mt-2 text-xs text-amber-200/70">
            Run <code className="rounded bg-black/40 px-1">aws login</code> (or{" "}
            <code className="rounded bg-black/40 px-1">aws configure</code>) in a
            terminal. This page switches to your real account on its own — no restart.
          </div>
        </div>
      )}
      {inv?.mode === "real-aws" && (
        <div className="rounded-lg border border-emerald-500/25 bg-emerald-500/10 p-3 text-xs text-emerald-200">
          ✓ Live AWS account · region <code className="font-mono">{inv.region}</code> ·
          credentials resolved from the standard chain.
        </div>
      )}

      <div className="flex items-end justify-between gap-4">
        <div>
          <div className="lbl">AWS Cloud Console</div>
          <h1 className="h-page mt-1">Every service, one pane of glass</h1>
          <p className="mt-2 text-sm text-slate-400">
            Live inventory of AWS services powering FPMS · endpoint{" "}
            <code className="rounded bg-black/50 px-1.5 py-0.5 text-xs">{inv?.endpoint ?? "…"}</code>
            {" "}· region <code className="rounded bg-black/50 px-1.5 py-0.5 text-xs">{inv?.region ?? "…"}</code>.
          </p>
        </div>
        <div className="flex gap-2">
          <button
            className="btn"
            disabled={verifying}
            onClick={async () => {
              setVerifying(true); setVerify(null);
              try {
                const r = await fetch("/api/aws/verify", { method: "POST" });
                setVerify(await r.json());
                await load();
              } finally { setVerifying(false); }
            }}
          >
            {verifying ? "Verifying…" : "Verify all services"}
          </button>
          <button className="btn-primary" onClick={load} disabled={refreshing}>
            {refreshing ? "Refreshing…" : "Refresh"}
          </button>
        </div>
      </div>

      {verify && (
        <Card>
          <CardHeader
            title={`End-to-end verification — ${verify.passed}/${verify.passed + verify.failed} passed`}
            subtitle={`Probe ${verify.probe_thing}`}
            right={
              <span className={verify.failed === 0 ? "chip-ok" : "chip-hot"}>
                {verify.failed === 0 ? "ALL OK" : `${verify.failed} FAILED`}
              </span>
            }
          />
          <ul className="space-y-2">
            {verify.checks.map((c) => (
              <li key={c.name} className="rounded-lg border border-white/5 bg-black/30 p-3">
                <div className="flex items-center justify-between gap-4">
                  <div className="font-medium text-slate-100">
                    <span className={c.ok ? "text-emerald-300" : "text-rose-300"}>
                      {c.ok ? "✓" : "✗"}
                    </span>{" "}
                    {c.name}
                  </div>
                  {c.error && <span className="text-xs text-rose-300 truncate max-w-[50%]">{c.error}</span>}
                </div>
                {c.evidence && (
                  <pre className="mt-1 overflow-x-auto rounded bg-black/40 p-2 text-[10px] text-slate-400">
{JSON.stringify(c.evidence, null, 2)}
                  </pre>
                )}
              </li>
            ))}
          </ul>
        </Card>
      )}

      {err && (
        <Card>
          <div className="text-sm text-rose-300">Failed to reach LocalStack: {err}</div>
        </Card>
      )}

      <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
        <ServiceCard
          name="AWS IoT Core"
          desc="Device registry + MQTT broker"
          accent="orange"
          count={inv?.services.iot?.thing_count ?? 0}
          countLabel="Things"
        >
          {inv?.services.iot?.error ? (
            <ErrorLine msg={inv.services.iot.message} />
          ) : (
            <div className="space-y-2">
              <Row label="Endpoint" value={inv?.services.iot?.endpoint ?? "—"} mono />
              <Row label="Policies" value={(inv?.services.iot?.policies ?? []).join(", ") || "—"} />
              <div className="mt-3">
                <div className="lbl mb-1.5">Registered Things</div>
                {(inv?.services.iot?.things ?? []).length === 0 ? (
                  <div className="text-xs text-slate-500">None yet — go to the Devices tab to onboard one.</div>
                ) : (
                  <ul className="space-y-1">
                    {inv!.services.iot.things.map((t: any) => (
                      <li key={t.name} className="rounded border border-white/5 bg-black/30 px-2 py-1 font-mono text-xs">
                        {t.name}
                      </li>
                    ))}
                  </ul>
                )}
              </div>
            </div>
          )}
        </ServiceCard>

        <ServiceCard
          name="Amazon S3"
          desc="Event archive + heritage record store"
          accent="green"
          count={sum((inv?.services.s3?.buckets ?? []).map((b: any) => b.object_count))}
          countLabel="Objects"
        >
          {inv?.services.s3?.error ? (
            <ErrorLine msg={inv.services.s3.message} />
          ) : (
            <ul className="space-y-1.5">
              {(inv?.services.s3?.buckets ?? []).map((b: any) => (
                <li key={b.name} className="flex items-center justify-between rounded border border-white/5 bg-black/30 px-3 py-2 text-xs">
                  <span className="font-mono text-slate-200">{b.name}</span>
                  <span className="text-slate-400">
                    {b.object_count} objs · {humanBytes(b.bytes)}
                  </span>
                </li>
              ))}
              {(inv?.services.s3?.buckets ?? []).length === 0 && (
                <li className="text-xs text-slate-500">No buckets yet.</li>
              )}
            </ul>
          )}
        </ServiceCard>

        <ServiceCard
          name="AWS Lambda"
          desc="Event routing + alert dispatch"
          accent="purple"
          count={(inv?.services.lambda?.functions ?? []).length}
          countLabel="Functions"
        >
          {inv?.services.lambda?.error ? (
            <ErrorLine msg={inv.services.lambda.message} />
          ) : (
            <ul className="space-y-1.5">
              {(inv?.services.lambda?.functions ?? []).map((f: any) => (
                <li key={f.name} className="rounded border border-white/5 bg-black/30 px-3 py-2 text-xs">
                  <div className="flex items-center justify-between">
                    <span className="font-mono text-slate-200">{f.name}</span>
                    <span className="text-slate-500">{f.runtime}</span>
                  </div>
                  <div className="mt-0.5 text-[10px] text-slate-500">
                    {f.memory_mb} MB · updated {f.last_modified}
                  </div>
                </li>
              ))}
              {(inv?.services.lambda?.functions ?? []).length === 0 && (
                <li className="text-xs text-slate-500">No functions deployed.</li>
              )}
            </ul>
          )}
        </ServiceCard>

        <ServiceCard
          name="Amazon SNS"
          desc="Alert fan-out"
          accent="blue"
          count={(inv?.services.sns?.topics ?? []).length}
          countLabel="Topics"
        >
          {inv?.services.sns?.error ? (
            <ErrorLine msg={inv.services.sns.message} />
          ) : (
            <ul className="space-y-1.5">
              {(inv?.services.sns?.topics ?? []).map((t: any) => (
                <li key={t.arn} className="flex items-center justify-between rounded border border-white/5 bg-black/30 px-3 py-2 text-xs">
                  <span className="font-mono text-slate-200">{t.name}</span>
                  <span className="text-slate-400">{t.subscription_count} subs</span>
                </li>
              ))}
              {(inv?.services.sns?.topics ?? []).length === 0 && (
                <li className="text-xs text-slate-500">No topics yet.</li>
              )}
            </ul>
          )}
        </ServiceCard>

        <ServiceCard
          name="CloudWatch Logs"
          desc="Function + service telemetry"
          accent="grey"
          count={(inv?.services.logs?.groups ?? []).length}
          countLabel="Log Groups"
        >
          {inv?.services.logs?.error ? (
            <ErrorLine msg={inv.services.logs.message} />
          ) : (
            <ul className="space-y-1.5">
              {(inv?.services.logs?.groups ?? []).slice(0, 8).map((g: any) => (
                <li key={g.name} className="flex items-center justify-between rounded border border-white/5 bg-black/30 px-3 py-2 text-xs">
                  <span className="font-mono text-slate-200 truncate">{g.name}</span>
                  <span className="text-slate-400">{humanBytes(g.bytes)}</span>
                </li>
              ))}
              {(inv?.services.logs?.groups ?? []).length === 0 && (
                <li className="text-xs text-slate-500">No log groups.</li>
              )}
            </ul>
          )}
        </ServiceCard>

        <Card className="lg:col-span-1">
          <CardHeader title="Endpoint" subtitle="Local AWS surface" />
          <div className="space-y-2 text-sm">
            <Row label="Region" value={inv?.region ?? "—"} mono />
            <Row label="Endpoint" value={inv?.endpoint ?? "—"} mono />
            <div className="pt-3 text-xs text-slate-500">
              To point at real AWS, unset <code className="rounded bg-black/40 px-1">FPMS_AWS_ENDPOINT</code>
              {" "}and configure standard AWS credentials. No code changes.
            </div>
          </div>
        </Card>
      </div>
    </div>
  );
}

function ServiceCard({
  name, desc, accent, count, countLabel, children,
}: {
  name: string;
  desc: string;
  accent: "orange" | "green" | "purple" | "blue" | "grey";
  count: number;
  countLabel: string;
  children: React.ReactNode;
}) {
  const accentClass = {
    orange: "text-ember-300 border-ember-500/30 bg-ember-500/10",
    green: "text-emerald-300 border-emerald-500/30 bg-emerald-500/10",
    purple: "text-violet-300 border-violet-500/30 bg-violet-500/10",
    blue: "text-sky-300 border-sky-500/30 bg-sky-500/10",
    grey: "text-slate-300 border-slate-500/30 bg-slate-500/10",
  }[accent];
  return (
    <section className="card p-5">
      <div className="mb-4 flex items-start justify-between gap-4">
        <div>
          <div className="text-xs uppercase tracking-widest text-slate-500">{desc}</div>
          <div className="mt-0.5 text-lg font-semibold tracking-tight text-slate-100">{name}</div>
        </div>
        <div className={`rounded-lg border px-3 py-1.5 text-right ${accentClass}`}>
          <div className="text-[10px] uppercase tracking-widest opacity-80">{countLabel}</div>
          <div className="font-mono text-lg font-semibold leading-none">{count}</div>
        </div>
      </div>
      {children}
    </section>
  );
}

function Row({ label, value, mono }: { label: string; value: string; mono?: boolean }) {
  return (
    <div className="flex items-baseline gap-2">
      <span className="lbl min-w-[70px] text-[10px]">{label}</span>
      <span className={`text-xs text-slate-200 truncate ${mono ? "font-mono" : ""}`}>{value}</span>
    </div>
  );
}

function ErrorLine({ msg }: { msg: string }) {
  return <div className="text-xs text-rose-400">Unreachable: {msg}</div>;
}

function humanBytes(n: number): string {
  if (!n) return "0 B";
  const units = ["B", "KB", "MB", "GB"];
  let i = 0, v = n;
  while (v >= 1024 && i < units.length - 1) { v /= 1024; i++; }
  return `${v.toFixed(v >= 100 ? 0 : 1)} ${units[i]}`;
}

function sum(xs: number[]): number { return xs.reduce((a, b) => a + b, 0); }

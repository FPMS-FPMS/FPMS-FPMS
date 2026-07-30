import { useEffect, useState } from "react";
import { Card, CardHeader } from "../components/Card";
import SharePanel from "../components/SharePanel";
import AlertsCard from "../components/AlertsCard";
import { apiGet } from "../lib/api";

type Info = {
  name: string;
  tagline: string;
  summary: string;
  team: { name: string; role: string }[];
  school: string;
  achievements: string[];
  mission: {
    reactive: { title: string; body: string };
    proactive: { title: string; body: string };
  };
  how_it_sees: { camera: string; sees: string; good_at: string }[];
  hardware_rover1: { component: string; purpose: string }[];
  stack: string[];
  principles: string[];
  repo: string;
};

type EventItem = {
  key: string;
  modified: string;
  event: { event_id?: string; thing?: string; severity?: string; timestamp?: string };
};

export default function Intro() {
  const [info, setInfo] = useState<Info | null>(null);
  const [events, setEvents] = useState<EventItem[]>([]);

  useEffect(() => {
    apiGet<Info>("/api/info").then(setInfo).catch(() => {});
    const load = () =>
      apiGet<{ events: EventItem[] }>("/api/events?limit=8")
        .then((r) => setEvents(r.events))
        .catch(() => {});
    load();
    const id = window.setInterval(load, 5000);
    return () => window.clearInterval(id);
  }, []);

  if (!info) {
    return <div className="text-slate-400">Loading FPMS…</div>;
  }

  return (
    <div className="space-y-6">
      {/* Share panel first — makes reaching the app from a phone one click */}
      <SharePanel compact />

      {/* Hero */}
      <section className="card-glow overflow-hidden">
        <div className="grid gap-6 p-8 md:grid-cols-[1.4fr_1fr]">
          <div>
            <div className="lbl">Fire Prevention & Management System</div>
            <h1 className="mt-2 text-4xl font-extrabold tracking-tight text-slate-50 sm:text-5xl">
              Protecting the past with the <span className="text-ember-400">power of the present.</span>
            </h1>
            <p className="mt-4 max-w-2xl text-base leading-relaxed text-slate-300">
              {info.summary}
            </p>
            <div className="mt-6 flex flex-wrap gap-2">
              {info.achievements.map((a) => (
                <span key={a} className="chip-hot">{a}</span>
              ))}
              <a href={info.repo} target="_blank" rel="noreferrer" className="chip">
                ↗ github.com/FPMS-FPMS
              </a>
            </div>
          </div>
          <div className="rounded-xl border border-white/5 bg-black/40 p-5">
            <div className="lbl">Team</div>
            <ul className="mt-3 space-y-3">
              {info.team.map((p) => (
                <li key={p.name}>
                  <div className="text-sm font-semibold text-slate-100">{p.name}</div>
                  <div className="text-xs text-slate-400">{p.role}</div>
                </li>
              ))}
            </ul>
            <div className="mt-4 border-t border-white/5 pt-3 text-xs text-slate-500">
              {info.school}
            </div>
          </div>
        </div>
      </section>

      {/* Mission halves */}
      <div className="grid gap-6 md:grid-cols-2">
        <Card>
          <CardHeader title={info.mission.reactive.title} subtitle="🔥 Reactive" />
          <p className="text-sm leading-relaxed text-slate-300">{info.mission.reactive.body}</p>
        </Card>
        <Card>
          <CardHeader title={info.mission.proactive.title} subtitle="🌱 Proactive" />
          <p className="text-sm leading-relaxed text-slate-300">{info.mission.proactive.body}</p>
        </Card>
      </div>

      {/* How it sees + Principles */}
      <div className="grid gap-6 md:grid-cols-2">
        <Card>
          <CardHeader title="How the rover sees" subtitle="Perception" />
          <div className="space-y-3">
            {info.how_it_sees.map((row) => (
              <div key={row.camera} className="rounded-lg border border-white/5 bg-black/30 p-3">
                <div className="flex items-center gap-2">
                  <span className="chip-hot">{row.camera}</span>
                  <span className="text-xs text-slate-400">{row.sees}</span>
                </div>
                <div className="mt-2 text-sm text-slate-200">{row.good_at}</div>
              </div>
            ))}
            <p className="pt-1 text-xs text-slate-500">
              Only when both cameras agree does the rover act. Cross-validated
              perception keeps false positives (shadows, sun-warmed rocks) from
              wasting water and eroding trust.
            </p>
          </div>
        </Card>
        <Card>
          <CardHeader title="Design principles" subtitle="Why it's built this way" />
          <ul className="space-y-2 text-sm text-slate-300">
            {info.principles.map((p) => (
              <li key={p} className="flex gap-2">
                <span className="mt-1 h-1.5 w-1.5 flex-none rounded-full bg-ember-400" />
                {p}
              </li>
            ))}
          </ul>
        </Card>
      </div>

      {/* Architecture */}
      <Card>
        <CardHeader title="System architecture" subtitle="Four tiers" />
        <div className="grid gap-4 md:grid-cols-4">
          {[
            { t: "I · Field", items: ["Rover 1", "Rover 2", "Sensors", "Refill dock"] },
            { t: "II · Transport", items: ["WiFi 6", "MQTT / TLS", "Batched", "State only"] },
            { t: "III · Cloud", items: ["AWS IoT Core", "Lambda", "S3 archive", "SNS alerts"] },
            { t: "IV · Public", items: ["Live map", "Event log", "Heritage db", "This dashboard"] },
          ].map((col) => (
            <div key={col.t} className="rounded-lg border border-white/5 bg-black/30 p-3">
              <div className="text-xs font-semibold uppercase tracking-widest text-ember-300">
                {col.t}
              </div>
              <ul className="mt-2 space-y-1 text-sm text-slate-200">
                {col.items.map((i) => <li key={i}>{i}</li>)}
              </ul>
            </div>
          ))}
        </div>
        <p className="mt-4 text-xs text-slate-500">
          Event-based, not streaming. Video does not go to the cloud — only state
          changes do. Bandwidth stays minimal, storage predictable, dashboard readable.
        </p>
      </Card>

      {/* Hardware + stack + recent events */}
      <div className="grid gap-6 lg:grid-cols-[1.4fr_1fr]">
        <Card>
          <CardHeader title="Rover 1 — Reactive suppression" subtitle="Hardware" />
          <div className="overflow-hidden rounded-lg border border-white/5">
            <table className="w-full text-sm">
              <thead className="bg-white/[0.03] text-left text-xs uppercase tracking-widest text-slate-400">
                <tr>
                  <th className="px-3 py-2">Component</th>
                  <th className="px-3 py-2">Purpose</th>
                </tr>
              </thead>
              <tbody className="divide-y divide-white/5">
                {info.hardware_rover1.map((r) => (
                  <tr key={r.component}>
                    <td className="px-3 py-2 font-medium text-slate-200">{r.component}</td>
                    <td className="px-3 py-2 text-slate-400">{r.purpose}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </Card>

        <div className="space-y-6">
          <Card>
            <CardHeader title="Software stack" subtitle="What powers the rover" />
            <div className="flex flex-wrap gap-2">
              {info.stack.map((s) => (
                <span key={s} className="chip">{s}</span>
              ))}
            </div>
          </Card>

          <Card>
            <CardHeader
              title="Recent events"
              subtitle="From S3 (LocalStack)"
              right={<span className="chip">{events.length}</span>}
            />
            {events.length === 0 ? (
              <div className="text-sm text-slate-500">
                No events yet. Events land here when a rover publishes to{" "}
                <code className="rounded bg-black/50 px-1 py-0.5 text-xs">
                  fpms/&lt;rover&gt;/events/#
                </code>{" "}
                via AWS IoT Core (LocalStack).
              </div>
            ) : (
              <ul className="space-y-2">
                {events.map((e) => (
                  <li key={e.key} className="rounded-md border border-white/5 bg-black/30 px-3 py-2 text-xs">
                    <div className="flex items-center gap-2">
                      <span className="chip-hot text-[10px]">{e.event.severity ?? "—"}</span>
                      <span className="font-semibold text-slate-200">{e.event.thing ?? "?"}</span>
                      <span className="text-slate-500">{e.event.timestamp ?? e.modified}</span>
                    </div>
                    <div className="mt-1 truncate font-mono text-[10px] text-slate-500">{e.key}</div>
                  </li>
                ))}
              </ul>
            )}
          </Card>
        </div>
      </div>

      <AlertsCard />

      <Card>
        <CardHeader title="Honest limits" subtitle="What FPMS is not" />
        <p className="text-sm leading-relaxed text-slate-300">
          FPMS is a competition prototype built by two Grade 8 students. A small
          robot cannot replace a fire crew, community fire knowledge, or the
          professional systems that already protect land and life. What we're
          trying to show is a smaller thing — a working example of a bigger idea.
          That technology, when built respectfully and kept transparent to the
          people it affects, can contribute in modest ways to protecting places
          that matter.
        </p>
      </Card>
    </div>
  );
}

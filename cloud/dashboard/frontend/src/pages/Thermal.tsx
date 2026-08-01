import { useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { StatusPill } from "../components/StatusPill";
import { ThermalView } from "../components/ThermalView";
import { AnalystPanel } from "../components/AnalystPanel";
import { useChannel } from "../lib/ws";
import { useActiveThing } from "../lib/things";
import { apiPost } from "../lib/api";

export default function Thermal() {
  // The page used to be titled "rover 1" while the feed underneath it was
  // whichever rover happened to be reporting — so a critical hotspot could be
  // labelled with the wrong machine's name. The rover is now picked explicitly
  // and its name is printed wherever the reading is.
  const [picked, setPicked] = useState<string | null>(null);
  const { thing, things } = useActiveThing(picked);
  const stream = useChannel<any>(thing ? `thermal:${thing}` : null);
  const analysis = useChannel<any>(thing ? `thermal-analysis:${thing}` : null);
  const [busy, setBusy] = useState(false);

  const cmd = async (action: "connect" | "disconnect") => {
    setBusy(true);
    try { if (!thing) return;
      await apiPost(`/api/rover/${thing}/${action}`); }
    finally { setBusy(false); }
  };

  // `?.data.severity` threw the moment an envelope arrived without a payload,
  // and a thrown render on this page takes the fire warning down with it.
  const severity: string =
    (analysis.data?.data?.severity as string | undefined) ?? "unknown";

  return (
    <div className="space-y-6">
      <div className="flex items-end justify-between gap-4">
        <div>
          <div className="lbl">Page 4</div>
          <h1 className="h-page mt-1">
            Thermal — {thing ?? "no rover reporting"} · with mini analyst
          </h1>
        </div>
        <div className="flex items-center gap-2">
          {severity === "critical" && (
            <span className="chip-hot animate-pulse">
              ⚠ CRITICAL — potential fire on {thing ?? "this rover"}
            </span>
          )}
          {things.length > 1 && (
            <select
              value={thing ?? ""}
              onChange={(e) => setPicked(e.target.value)}
              className="rounded-lg border border-white/10 bg-black/40 px-3 py-1.5 text-sm text-slate-100 outline-none"
              title="Which rover's thermal feed to show"
            >
              {things.map((t) => (
                <option key={t} value={t}>{t}</option>
              ))}
            </select>
          )}
        </div>
      </div>

      <div className="grid gap-6 lg:grid-cols-[1.5fr_1fr]">
        <Card>
          <CardHeader
            title="Live thermal grid"
            subtitle={thing ? `LWIR · 32 × 24 · ironbow · ${thing}` : "LWIR · 32 × 24 · ironbow"}
            right={
              <StatusPill
                connected={stream.connected}
                lastAt={stream.lastAt}
                messages={stream.messages}
              />
            }
          />
          <ThermalView envelope={stream.data} analysis={analysis.data} height={420} />

          <Legend />

          <div className="mt-4 flex justify-end gap-2">
            <button className="btn-primary" disabled={busy} onClick={() => cmd("connect")}>
              Connect {thing ?? "rover"}
            </button>
            <button className="btn-danger" disabled={busy} onClick={() => cmd("disconnect")}>
              Disconnect
            </button>
          </div>
        </Card>

        <AnalystPanel analysis={analysis.data} />
      </div>

      <Card>
        <CardHeader title="About the analyst" subtitle="On-laptop model" />
        <p className="text-sm leading-relaxed text-slate-300">
          Every incoming thermal frame runs through a small numpy-based analysis
          on this laptop — no cloud, no GPU required. It computes scene
          statistics, finds contiguous hotspot regions with a 4-connected
          component labeler, tracks a 30-second trend of peak temperature, and
          writes a plain-English report you can hand to a judge or a community
          member without needing to explain a colormap.
        </p>
        <p className="mt-3 text-xs text-slate-500">
          Severity thresholds:
          {" "}<span className="chip-ok">nominal &lt; 55°C</span>
          {" "}<span className="chip-warn">elevated ≥ 55°C</span>
          {" "}<span className="chip-hot">critical ≥ 80°C</span>
        </p>
      </Card>
    </div>
  );
}

function Legend() {
  return (
    <div className="mt-3 flex items-center gap-3">
      <span className="lbl">Cold</span>
      <div
        className="h-2 flex-1 rounded-full"
        style={{
          background:
            "linear-gradient(to right, rgb(0,0,15), rgb(60,0,90), rgb(180,30,60), rgb(235,90,30), rgb(250,190,60), rgb(255,255,220))",
        }}
      />
      <span className="lbl">Hot</span>
    </div>
  );
}

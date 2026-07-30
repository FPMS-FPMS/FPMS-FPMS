import { useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { StatusPill } from "../components/StatusPill";
import { ThermalView } from "../components/ThermalView";
import { AnalystPanel } from "../components/AnalystPanel";
import { useChannel } from "../lib/ws";
import { useActiveThing } from "../lib/things";
import { apiPost } from "../lib/api";

export default function Thermal() {
  const { thing } = useActiveThing(null);
  const stream = useChannel<any>(thing ? `thermal:${thing}` : null);
  const analysis = useChannel<any>(thing ? `thermal-analysis:${thing}` : null);
  const [busy, setBusy] = useState(false);

  const cmd = async (action: "connect" | "disconnect") => {
    setBusy(true);
    try { if (!thing) return;
      await apiPost(`/api/rover/${thing}/${action}`); }
    finally { setBusy(false); }
  };

  const severity = analysis.data?.data.severity ?? "nominal";

  return (
    <div className="space-y-6">
      <div className="flex items-end justify-between">
        <div>
          <div className="lbl">Page 4</div>
          <h1 className="h-page mt-1">Thermal — rover 1 · with mini analyst</h1>
        </div>
        <div className="flex items-center gap-2">
          {severity === "critical" && (
            <span className="chip-hot animate-pulse">
              ⚠ CRITICAL — potential fire
            </span>
          )}
        </div>
      </div>

      <div className="grid gap-6 lg:grid-cols-[1.5fr_1fr]">
        <Card>
          <CardHeader
            title="Live thermal grid"
            subtitle="LWIR · 32 × 24 · ironbow"
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

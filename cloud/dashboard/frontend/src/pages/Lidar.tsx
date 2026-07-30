import { useState } from "react";
import { Card, CardHeader } from "../components/Card";
import { StatusPill } from "../components/StatusPill";
import { LidarView } from "../components/LidarView";
import { useChannel } from "../lib/ws";
import { apiPost } from "../lib/api";

export default function Lidar() {
  return (
    <div className="space-y-6">
      <div className="flex items-end justify-between">
        <div>
          <div className="lbl">Page 2</div>
          <h1 className="h-page mt-1">Dual LiDAR — both Orange Pi 5B rovers</h1>
        </div>
      </div>

      <div className="grid gap-6 lg:grid-cols-2">
        <RoverLidar thing="rover1" accent="#f97316" />
        <RoverLidar thing="rover2" accent="#38bdf8" />
      </div>

      <Card>
        <CardHeader title="How this stream works" subtitle="Behind the scenes" />
        <p className="text-sm leading-relaxed text-slate-300">
          Each rover publishes 360-range LiDAR frames at 5 Hz on
          {" "}<code className="rounded bg-black/50 px-1 py-0.5 text-xs">fpms/&lt;rover&gt;/telemetry/lidar</code>{" "}
          to the local Mosquitto broker (standing in for AWS IoT Core). This dashboard
          subscribes over WebSocket — no cloud round-trip, no per-frame cost. Data never
          leaves your LAN.
        </p>
      </Card>
    </div>
  );
}

function RoverLidar({ thing, accent }: { thing: string; accent: string }) {
  const state = useChannel<any>(`lidar:${thing}`);
  const pose = useChannel<any>(`pose:${thing}`);
  const [busy, setBusy] = useState(false);

  const cmd = async (action: "connect" | "disconnect") => {
    setBusy(true);
    try { await apiPost(`/api/rover/${thing}/${action}`); }
    finally { setBusy(false); }
  };

  return (
    <Card>
      <CardHeader
        title={thing.toUpperCase()}
        subtitle="LiDAR grid · 360 ranges · 5 Hz"
        right={
          <div className="flex items-center gap-2">
            <StatusPill
              connected={state.connected}
              lastAt={state.lastAt}
              messages={state.messages}
            />
          </div>
        }
      />
      <div className="flex items-center justify-center">
        <LidarView envelope={state.data} accent={accent} />
      </div>

      <div className="mt-4 grid grid-cols-3 gap-3 text-center">
        <Metric label="Nearest" value={fmt(state.data?.data?.min_mm, 0, " mm")} />
        <Metric
          label="Position"
          value={
            // Rovers without odometry publish no x/y. Showing "—" is correct;
            // calling .toFixed() on the missing field used to throw and, with no
            // error boundary, blanked the whole dashboard.
            isNum(pose.data?.data?.x_m) && isNum(pose.data?.data?.y_m)
              ? `${pose.data.data.x_m.toFixed(2)}, ${pose.data.data.y_m.toFixed(2)}`
              : "—"
          }
        />
        <Metric label="Battery" value={fmt(pose.data?.data?.battery_pct, 1, "%")} />
      </div>

      <div className="mt-4 flex justify-end gap-2">
        <button className="btn-primary" disabled={busy} onClick={() => cmd("connect")}>
          Connect {thing}
        </button>
        <button className="btn-danger" disabled={busy} onClick={() => cmd("disconnect")}>
          Disconnect
        </button>
      </div>
    </Card>
  );
}

/** Telemetry fields are optional by nature — never assume one is a number. */
function isNum(v: unknown): v is number {
  return typeof v === "number" && Number.isFinite(v);
}

function fmt(v: unknown, digits: number, suffix = ""): string {
  return isNum(v) ? `${v.toFixed(digits)}${suffix}` : "—";
}

function Metric({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded-lg border border-white/5 bg-black/30 px-3 py-2">
      <div className="lbl text-[10px]">{label}</div>
      <div className="mt-1 font-mono text-sm text-slate-100">{value}</div>
    </div>
  );
}

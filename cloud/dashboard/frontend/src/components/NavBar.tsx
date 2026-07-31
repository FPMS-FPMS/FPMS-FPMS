import { NavLink } from "react-router-dom";

type Props = {
  health: {
    ok: boolean;
    mqtt: { connected: boolean; messages_seen: number };
    aws: { mode: "local" | "cloud"; reachable: boolean; region: string };
  } | null;
  /** Safe mode is on and this visitor came in over the public link. */
  controlsDisabled?: boolean;
};

// `hqOnly` tabs can act on the HQ laptop itself rather than just showing rover
// data, so they're hidden from public visitors. The server enforces this too —
// hiding a tab is convenience, not the security boundary.
const tabs = [
  { to: "/", label: "Overview", end: true },
  { to: "/control", label: "Control", hqOnly: true },
  { to: "/devices", label: "Devices", hqOnly: true },
  // AWS and Install describe the operator's own machine — an AWS console view
  // and a "download the desktop app" page. Neither means anything in the cloud
  // deployment, where there is no local machine and no installer.
  { to: "/aws", label: "AWS", hqOnly: true },
  { to: "/lidar", label: "LiDAR" },
  { to: "/camera", label: "Camera" },
  { to: "/thermal", label: "Thermal" },
  { to: "/analyst", label: "Analyst" },
  { to: "/terminal", label: "Terminal", hqOnly: true },
  { to: "/install", label: "Install", hqOnly: true },
];

export default function NavBar({ health, controlsDisabled = false }: Props) {
  const visibleTabs = controlsDisabled ? tabs.filter((t) => !t.hqOnly) : tabs;
  const mqttOn = !!health?.mqtt.connected;
  const awsOn = !!health?.aws.reachable;
  const awsMode = health?.aws.mode ?? "local";
  const awsRegion = health?.aws.region ?? "";

  return (
    <header className="sticky top-0 z-30 border-b border-white/5 bg-ink-950/85 backdrop-blur">
      <div className="mx-auto flex max-w-7xl items-center gap-6 px-4 py-3 sm:px-6 lg:px-8">
        <div className="flex items-center gap-3">
          <FireMark />
          <div className="leading-tight">
            <div className="text-sm font-semibold tracking-wide text-slate-100">FPMS</div>
            <div className="text-[10px] uppercase tracking-[0.18em] text-slate-500">
              Robotics Operations Console
            </div>
          </div>
        </div>
        <nav className="hidden gap-1 md:flex">
          {visibleTabs.map((t) => (
            <NavLink
              key={t.to}
              to={t.to}
              end={t.end as any}
              className={({ isActive }) =>
                `rounded-lg px-3 py-1.5 text-sm font-medium transition ${
                  isActive
                    ? "bg-white/10 text-white"
                    : "text-slate-400 hover:bg-white/5 hover:text-slate-200"
                }`
              }
            >
              {t.label}
            </NavLink>
          ))}
        </nav>
        <div className="ml-auto flex items-center gap-2">
          <AwsChip on={awsOn} mode={awsMode} region={awsRegion} />
          <Dot label="IoT Core" ok={mqttOn} />
          <span className="hidden font-mono text-[11px] text-slate-500 sm:inline">
            {health ? `${health.mqtt.messages_seen} pkts` : "…"}
          </span>
        </div>
      </div>
      <nav className="flex gap-1 border-t border-white/5 px-3 py-2 md:hidden">
        {visibleTabs.map((t) => (
          <NavLink
            key={t.to}
            to={t.to}
            end={t.end as any}
            className={({ isActive }) =>
              `flex-1 rounded-md px-2 py-1 text-center text-xs font-medium transition ${
                isActive ? "bg-white/10 text-white" : "text-slate-400"
              }`
            }
          >
            {t.label}
          </NavLink>
        ))}
      </nav>
    </header>
  );
}

function AwsChip({ on, mode, region }: { on: boolean; mode: "local" | "cloud"; region: string }) {
  const label = mode === "cloud" ? `AWS · Cloud · ${region}` : "AWS · Local";
  const cls = mode === "cloud"
    ? (on ? "chip-ok" : "chip-warn")
    : (on ? "chip" : "chip-warn");
  const title = mode === "cloud"
    ? (on ? `Real AWS in ${region}` : "AWS unreachable — check credentials")
    : (on ? "LocalStack running on this laptop" : "LocalStack unreachable");
  return (
    <span className={cls} title={title}>
      <svg width="14" height="14" viewBox="0 0 32 32" aria-hidden>
        <path
          d="M8.7 12.4 16 8.4l7.3 4-7.3 4-7.3-4Zm0 3.3 6.7 3.7v7.4l-6.7-3.7v-7.4Zm14.6 0v7.4l-6.7 3.7v-7.4l6.7-3.7Z"
          fill="#ff9900"
        />
      </svg>
      {label}
    </span>
  );
}

function Dot({ label, ok }: { label: string; ok: boolean }) {
  return (
    <span
      className={`chip ${ok ? "chip-ok" : "chip-warn"}`}
      title={`${label}: ${ok ? "connected" : "waiting"}`}
    >
      <span
        className={`inline-block h-2 w-2 rounded-full pulse-dot ${
          ok ? "bg-emerald-400 text-emerald-400" : "bg-amber-400 text-amber-400"
        }`}
      />
      {label}
    </span>
  );
}

function FireMark() {
  return (
    <svg width="30" height="30" viewBox="0 0 32 32" aria-hidden>
      <defs>
        <linearGradient id="fmg" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0" stopColor="#fbbf24" />
          <stop offset="1" stopColor="#c2410c" />
        </linearGradient>
      </defs>
      <rect width="32" height="32" rx="9" fill="#0b0f16" stroke="rgba(255,255,255,0.08)" />
      <path
        d="M16 4 C 10 12, 22 14, 16 20 C 22 24, 8 26, 16 28 C 8 24, 12 18, 10 14 C 12 16, 14 12, 16 4 Z"
        fill="url(#fmg)"
      />
    </svg>
  );
}

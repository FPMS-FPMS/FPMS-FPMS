import { useEffect, useState } from "react";
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

type Tab = {
  to: string;
  label: string;
  end?: boolean;
  hqOnly?: boolean;
  icon: (p: { className?: string }) => JSX.Element;
};

/*
  Tabs are grouped by what the operator is trying to DO, not by when they were
  built. Eleven undifferentiated tabs in one row is a dev tool; three named
  groups is a console. The membership and the `hqOnly` flags are unchanged —
  only the presentation groups them.

  `hqOnly` tabs can act on the HQ laptop itself rather than just showing rover
  data, so they're hidden from public visitors. The server enforces this too —
  hiding a tab is convenience, not the security boundary.
*/
const groups: { name: string; tabs: Tab[] }[] = [
  {
    name: "Operations",
    tabs: [
      { to: "/", label: "Overview", end: true, icon: IconGrid },
      { to: "/control", label: "Control", hqOnly: true, icon: IconSliders },
      { to: "/drive", label: "Drive", hqOnly: true, icon: IconSteering },
    ],
  },
  {
    name: "Sensors",
    tabs: [
      { to: "/lidar", label: "LiDAR", icon: IconRadar },
      { to: "/camera", label: "Camera", icon: IconCamera },
      { to: "/thermal", label: "Thermal", icon: IconFlame },
      { to: "/analyst", label: "Analyst", icon: IconSpark },
    ],
  },
  {
    name: "System",
    tabs: [
      { to: "/devices", label: "Devices", hqOnly: true, icon: IconChip },
      // AWS and Install describe the operator's own machine — an AWS console
      // view and a "download the desktop app" page. Neither means anything in
      // the cloud deployment, where there is no local machine and no installer.
      { to: "/aws", label: "AWS", hqOnly: true, icon: IconCloud },
      { to: "/terminal", label: "Terminal", hqOnly: true, icon: IconTerminal },
      { to: "/install", label: "Install", hqOnly: true, icon: IconDownload },
    ],
  },
];

export default function NavBar({ health, controlsDisabled = false }: Props) {
  const visibleGroups = groups
    .map((g) => ({
      ...g,
      tabs: controlsDisabled ? g.tabs.filter((t) => !t.hqOnly) : g.tabs,
    }))
    .filter((g) => g.tabs.length > 0);

  const mqttOn = !!health?.mqtt.connected;
  const awsOn = !!health?.aws.reachable;
  const awsMode = health?.aws.mode ?? "local";
  const awsRegion = health?.aws.region ?? "";

  // `health === null` means the health poll itself failed — the browser cannot
  // reach the HQ backend. That is a different and worse failure than "IoT Core
  // has not connected yet", and it used to be rendered identically to it (a
  // single amber dot), which reads as "warming up" when it actually means every
  // number on screen is frozen. It gets its own loud state.
  const hqDown = health === null;

  return (
    <header className="sticky top-0 z-30 border-b border-white/[0.07] bg-ink-850/90 shadow-chrome backdrop-blur-xl">
      {/* ---------------------------------------------- identity + status */}
      <div className="app-container flex h-14 items-center gap-4">
        <div className="flex items-center gap-3">
          <FireMark />
          <div className="leading-tight">
            <div className="flex items-center gap-2">
              <span className="text-[0.9375rem] font-semibold tracking-wide text-slate-50">
                FPMS
              </span>
              <span className="hidden rounded border border-white/10 bg-white/[0.06] px-1.5 py-px text-[0.625rem] font-semibold uppercase tracking-wider text-slate-400 sm:inline">
                {awsMode === "cloud" ? "Cloud" : "Local"}
              </span>
            </div>
            <div className="hidden text-[0.625rem] uppercase tracking-[0.18em] text-slate-400 sm:block">
              Robotics Operations Console
            </div>
          </div>
        </div>

        {/*
          The status cluster. Sticky, so it is on screen from every tab, and
          boxed so it reads as one instrument rather than three loose chips.
          "Is anything wrong?" should be answerable without reading a word.
        */}
        <div
          className="ml-auto flex items-center gap-1.5 rounded-xl border border-white/[0.07] bg-black/25 p-1.5"
          role="status"
          aria-label="System status"
        >
          {hqDown ? (
            <span
              className="chip-bad"
              title="The dashboard cannot reach the HQ backend. Every value on screen is stale."
            >
              <span className="inline-block h-2 w-2 shrink-0 rounded-full bg-rose-300 text-rose-300 pulse-dot" />
              HQ LINK DOWN
            </span>
          ) : (
            <>
              <AwsChip on={awsOn} mode={awsMode} region={awsRegion} />
              <Dot label="IoT Core" ok={mqttOn} />
            </>
          )}
          <span
            className="hidden px-1 font-mono text-2xs text-slate-400 sm:inline"
            title="MQTT messages seen since the backend started"
          >
            {health ? `${health.mqtt.messages_seen} pkts` : "-- pkts"}
          </span>
          <Clock />
        </div>
      </div>

      {/* ------------------------------------------------------ navigation */}
      <nav
        aria-label="Primary"
        className="app-container no-scrollbar flex items-center gap-1 overflow-x-auto border-t border-white/[0.05] py-2"
      >
        {visibleGroups.map((g, gi) => (
          <div key={g.name} className="flex items-center gap-1">
            {gi > 0 && (
              <span aria-hidden className="mx-1 h-5 w-px shrink-0 bg-white/[0.08]" />
            )}
            <span className="nav-group-label">{g.name}</span>
            {g.tabs.map((t) => (
              <NavLink
                key={t.to}
                to={t.to}
                end={t.end}
                className={({ isActive }) =>
                  `nav-link ${isActive ? "nav-link-active" : ""}`
                }
              >
                <t.icon className="h-4 w-4 shrink-0 opacity-80" />
                <span>{t.label}</span>
              </NavLink>
            ))}
          </div>
        ))}
      </nav>
    </header>
  );
}

/** Wall clock. An ops console without one feels like a web page. */
function Clock() {
  const [now, setNow] = useState(() => new Date());
  useEffect(() => {
    const id = window.setInterval(() => setNow(new Date()), 1000);
    return () => window.clearInterval(id);
  }, []);
  return (
    <span
      className="hidden rounded-lg bg-white/[0.05] px-2 py-1 font-mono text-2xs text-slate-300 md:inline"
      title={now.toString()}
    >
      {now.toLocaleTimeString([], { hour12: false })}
    </span>
  );
}

function AwsChip({ on, mode, region }: { on: boolean; mode: "local" | "cloud"; region: string }) {
  const label = mode === "cloud" ? `AWS · ${region}` : "AWS · Local";
  const cls = mode === "cloud"
    ? (on ? "chip-ok" : "chip-warn")
    : (on ? "chip" : "chip-warn");
  const title = mode === "cloud"
    ? (on ? `Real AWS in ${region}` : "AWS unreachable — check credentials")
    : (on ? "LocalStack running on this laptop" : "LocalStack unreachable");
  return (
    <span className={cls} title={title}>
      <svg width="14" height="14" viewBox="0 0 32 32" aria-hidden className="shrink-0">
        <path
          d="M8.7 12.4 16 8.4l7.3 4-7.3 4-7.3-4Zm0 3.3 6.7 3.7v7.4l-6.7-3.7v-7.4Zm14.6 0v7.4l-6.7 3.7v-7.4l6.7-3.7Z"
          fill="#ff9900"
        />
      </svg>
      <span className="hidden sm:inline">{label}</span>
      <span className="sm:hidden">AWS</span>
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
        className={`inline-block h-2 w-2 shrink-0 rounded-full pulse-dot ${
          ok ? "bg-emerald-300 text-emerald-300" : "bg-amber-300 text-amber-300"
        }`}
      />
      <span className="hidden sm:inline">{label}</span>
    </span>
  );
}

function FireMark() {
  return (
    <svg width="32" height="32" viewBox="0 0 32 32" aria-hidden className="shrink-0">
      <defs>
        <linearGradient id="fmg" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0" stopColor="#fbbf24" />
          <stop offset="1" stopColor="#c2410c" />
        </linearGradient>
      </defs>
      <rect width="32" height="32" rx="9" fill="#0b0f16" stroke="rgba(255,255,255,0.10)" />
      <path
        d="M16 4 C 10 12, 22 14, 16 20 C 22 24, 8 26, 16 28 C 8 24, 12 18, 10 14 C 12 16, 14 12, 16 4 Z"
        fill="url(#fmg)"
      />
    </svg>
  );
}

/* -------------------------------------------------------------- icons */
/* 16px stroke icons, currentColor so they inherit the nav link's state. */

const S = {
  fill: "none",
  stroke: "currentColor",
  strokeWidth: 1.6,
  strokeLinecap: "round" as const,
  strokeLinejoin: "round" as const,
};

function Svg({ className, children }: { className?: string; children: JSX.Element }) {
  return (
    <svg viewBox="0 0 24 24" className={className} aria-hidden {...S}>
      {children}
    </svg>
  );
}

function IconGrid({ className }: { className?: string }) {
  return (
    <Svg className={className}>
      <g>
        <rect x="3" y="3" width="7" height="7" rx="1.5" />
        <rect x="14" y="3" width="7" height="7" rx="1.5" />
        <rect x="3" y="14" width="7" height="7" rx="1.5" />
        <rect x="14" y="14" width="7" height="7" rx="1.5" />
      </g>
    </Svg>
  );
}

function IconSliders({ className }: { className?: string }) {
  return (
    <Svg className={className}>
      <g>
        <path d="M4 6h16M4 12h16M4 18h16" />
        <circle cx="9" cy="6" r="2" />
        <circle cx="15" cy="12" r="2" />
        <circle cx="8" cy="18" r="2" />
      </g>
    </Svg>
  );
}

function IconSteering({ className }: { className?: string }) {
  return (
    <Svg className={className}>
      <g>
        <circle cx="12" cy="12" r="9" />
        <circle cx="12" cy="12" r="3" />
        <path d="M12 3v6M4.2 16.5 9.4 13.5M19.8 16.5 14.6 13.5" />
      </g>
    </Svg>
  );
}

function IconRadar({ className }: { className?: string }) {
  return (
    <Svg className={className}>
      <g>
        <circle cx="12" cy="12" r="9" />
        <circle cx="12" cy="12" r="4.5" />
        <path d="M12 12 19 7" />
        <circle cx="16" cy="9" r="1" fill="currentColor" stroke="none" />
      </g>
    </Svg>
  );
}

function IconCamera({ className }: { className?: string }) {
  return (
    <Svg className={className}>
      <g>
        <path d="M3 8.5A2.5 2.5 0 0 1 5.5 6h1.8l1.2-2h6.6l1.2 2h1.2A2.5 2.5 0 0 1 21 8.5v8A2.5 2.5 0 0 1 18.5 19h-13A2.5 2.5 0 0 1 3 16.5Z" />
        <circle cx="12" cy="12.5" r="3.4" />
      </g>
    </Svg>
  );
}

function IconFlame({ className }: { className?: string }) {
  return (
    <Svg className={className}>
      <path d="M12 3c2.5 4 5.5 5.2 5.5 9a5.5 5.5 0 0 1-11 0c0-1.8.9-3 1.8-4.2.5 1 1.2 1.6 2 1.9C10 7.6 10.8 5.4 12 3Z" />
    </Svg>
  );
}

function IconSpark({ className }: { className?: string }) {
  return (
    <Svg className={className}>
      <g>
        <path d="M12 3v3.5M12 17.5V21M3 12h3.5M17.5 12H21M6 6l2.4 2.4M15.6 15.6 18 18M18 6l-2.4 2.4M8.4 15.6 6 18" />
        <circle cx="12" cy="12" r="2.6" />
      </g>
    </Svg>
  );
}

function IconChip({ className }: { className?: string }) {
  return (
    <Svg className={className}>
      <g>
        <rect x="7" y="7" width="10" height="10" rx="2" />
        <path d="M10 3v4M14 3v4M10 17v4M14 17v4M3 10h4M3 14h4M17 10h4M17 14h4" />
      </g>
    </Svg>
  );
}

function IconCloud({ className }: { className?: string }) {
  return (
    <Svg className={className}>
      <path d="M7 18a4 4 0 0 1-.6-7.95A5.5 5.5 0 0 1 17 9.5a3.5 3.5 0 0 1 .5 6.96Z" />
    </Svg>
  );
}

function IconTerminal({ className }: { className?: string }) {
  return (
    <Svg className={className}>
      <g>
        <rect x="3" y="4" width="18" height="16" rx="2" />
        <path d="m7 9 3 3-3 3M13 15h4" />
      </g>
    </Svg>
  );
}

function IconDownload({ className }: { className?: string }) {
  return (
    <Svg className={className}>
      <path d="M12 3v11m0 0 4-4m-4 4-4-4M4 17v2a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2v-2" />
    </Svg>
  );
}

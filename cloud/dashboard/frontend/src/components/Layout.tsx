import { ReactNode, useEffect, useState } from "react";
import NavBar from "./NavBar";
import { FleetMissionBar } from "./MissionStrip";
import { apiGet } from "../lib/api";

type Health = {
  ok: boolean;
  mqtt: { connected: boolean; messages_seen: number; things_seen: string[] };
  channels: string[];
  aws: { mode: "local" | "cloud"; endpoint: string; region: string; reachable: boolean };
};

export default function Layout(
  { children, controlsDisabled = false }:
  { children: ReactNode; controlsDisabled?: boolean },
) {
  const [health, setHealth] = useState<Health | null>(null);

  useEffect(() => {
    let alive = true;
    const tick = async () => {
      try {
        const h = await apiGet<Health>("/api/health");
        if (alive) setHealth(h);
      } catch {
        if (alive) setHealth(null);
      }
    };
    tick();
    const id = window.setInterval(tick, 2500);
    return () => {
      alive = false;
      window.clearInterval(id);
    };
  }, []);

  return (
    <div className="flex min-h-full flex-col">
      {/* First tab stop on every page — the nav is long, and a keyboard user
          should not have to walk eleven links to reach the content. */}
      <a href="#main" className="skip-link">
        Skip to content
      </a>

      <NavBar health={health} controlsDisabled={controlsDisabled} />

      {controlsDisabled && (
        <div className="border-b border-sky-500/20 bg-sky-500/[0.07]">
          <div className="app-container flex items-start gap-2.5 py-2.5 text-xs leading-relaxed text-sky-100/90">
            <svg
              viewBox="0 0 24 24"
              className="mt-px h-4 w-4 shrink-0 text-sky-300"
              fill="none"
              stroke="currentColor"
              strokeWidth={1.7}
              aria-hidden
            >
              <circle cx="12" cy="12" r="9" />
              <path d="M12 11v5M12 8h.01" strokeLinecap="round" />
            </svg>
            <span>
              <b className="font-semibold text-sky-50">Live view.</b> This is the
              streaming deployment — controls that act on hardware (terminal, SSH,
              network scanning, provisioning) are switched off here. Use the FPMS
              edge app on the operator machine for those. Rover data is live.
            </span>
          </div>
        </div>
      )}

      {/*
        MISSION STATE, ON EVERY TAB.
        A mission is the one thing on this dashboard that makes a machine drive
        itself across a room, and it used to be visible only on Drive. The
        roster comes from the health poll already running above rather than
        from a second one; the bar renders nothing until a rover has actually
        published mission telemetry, and is quiet until one is moving.

        Deliberately rendered outside <main> and left non-sticky — Control and
        Drive pin their own EMERGENCY STOP bar just below the header, and two
        sticky elements at the same offset ends with one covering the other.
      */}
      <FleetMissionBar things={health?.mqtt.things_seen ?? []} />

      <main
        id="main"
        tabIndex={-1}
        className="app-container w-full flex-1 pb-16 pt-6 focus:outline-none sm:pt-8"
      >
        {children}
      </main>

      <footer className="mt-auto border-t border-white/[0.06] bg-ink-950/60">
        <div className="app-container flex flex-wrap items-center gap-x-4 gap-y-2 py-5 text-xs text-slate-400">
          <span className="font-medium text-slate-400">
            FPMS Robotics Operations Console
          </span>
          <span aria-hidden className="hidden h-3 w-px bg-white/10 sm:block" />
          <span>Running locally on your laptop · anyone on this network can view</span>
          <a
            className="ml-auto inline-flex items-center gap-1.5 rounded text-slate-400 transition hover:text-ember-300"
            href="https://github.com/FPMS-FPMS"
            target="_blank"
            rel="noreferrer"
          >
            <svg viewBox="0 0 16 16" className="h-3.5 w-3.5" fill="currentColor" aria-hidden>
              <path d="M8 0a8 8 0 0 0-2.53 15.59c.4.07.55-.17.55-.38l-.01-1.34c-2.23.48-2.7-1.07-2.7-1.07-.36-.93-.89-1.18-.89-1.18-.73-.5.05-.49.05-.49.8.06 1.23.83 1.23.83.72 1.23 1.88.88 2.34.67.07-.52.28-.88.51-1.08-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82a7.6 7.6 0 0 1 4 0c1.53-1.03 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.28.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48l-.01 2.2c0 .21.15.46.55.38A8 8 0 0 0 8 0Z" />
            </svg>
            github.com/FPMS-FPMS
          </a>
        </div>
      </footer>
    </div>
  );
}

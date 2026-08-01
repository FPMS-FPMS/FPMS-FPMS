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
    <div className="min-h-full">
      <NavBar health={health} controlsDisabled={controlsDisabled} />
      {controlsDisabled && (
        <div className="border-b border-emerald-500/20 bg-emerald-500/5">
          <div className="mx-auto max-w-7xl px-4 py-2 text-xs text-emerald-200 sm:px-6 lg:px-8">
            <b>Live view.</b> This is the streaming deployment — controls that act on
            hardware (terminal, SSH, network scanning, provisioning) are switched off here.
            Use the FPMS edge app on the operator machine for those. Rover data is live.
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
      */}
      <FleetMissionBar things={health?.mqtt.things_seen ?? []} />
      <main className="mx-auto max-w-7xl px-4 pb-16 pt-6 sm:px-6 lg:px-8">
        {children}
      </main>
      <footer className="mx-auto max-w-7xl px-4 pb-10 text-xs text-slate-500 sm:px-6 lg:px-8">
        FPMS · running locally on your laptop · anyone on this network can view ·
        {" "}
        <a
          className="text-slate-400 hover:text-ember-300"
          href="https://github.com/FPMS-FPMS"
          target="_blank"
          rel="noreferrer"
        >
          github.com/FPMS-FPMS
        </a>
      </footer>
    </div>
  );
}

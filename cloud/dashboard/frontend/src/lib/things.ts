import { useEffect, useState } from "react";

import { apiGet } from "./api";

type Health = { mqtt?: { things_seen?: string[] } };

/**
 * Rovers the backend has actually heard from, newest roster every few seconds.
 *
 * Pages used to hard-code "rover1". When the fleet's live unit was rover2 the
 * panels sat on "waiting for feed" forever while telemetry streamed past them —
 * the failure looked exactly like a dead sensor.
 */
export function useThings(): string[] {
  const [things, setThings] = useState<string[]>([]);

  useEffect(() => {
    let alive = true;
    const tick = () =>
      apiGet<Health>("/api/health")
        .then((h) => {
          if (alive) setThings([...(h.mqtt?.things_seen ?? [])].sort());
        })
        .catch(() => {});
    tick();
    const id = window.setInterval(tick, 5000);
    return () => {
      alive = false;
      window.clearInterval(id);
    };
  }, []);

  return things;
}

/**
 * The rover a single-rover page should display: the caller's choice when it is
 * still connected, otherwise the first one reporting.
 */
export function useActiveThing(preferred?: string | null): {
  thing: string | null;
  things: string[];
} {
  const things = useThings();
  const thing =
    preferred && things.includes(preferred) ? preferred : things[0] ?? null;
  return { thing, things };
}

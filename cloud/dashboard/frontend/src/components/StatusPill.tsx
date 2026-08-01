import { useEffect, useState } from "react";

/**
 * Per-stream connection state, used in card headers across Camera, Thermal,
 * LiDAR, Drive and Control.
 *
 * THREE states, not two, because "no signal" and "nothing has ever arrived"
 * are different facts and only one of them can be inferred from silence.
 *
 *   live    — a message arrived recently. An assertion, and it is only made
 *             when there is a timestamp to back it.
 *   stale   — messages HAD been arriving and stopped, or the browser's socket
 *             to the dashboard dropped. A warning: amber, pulsing, uppercase.
 *   no data — nothing has EVER arrived on this channel. Not live, not stale,
 *             and never green.
 *
 * The bug this replaces: `stale` skipped the freshness test entirely when
 * `lastAt === null`, so a feed with zero packets rendered as a pulsing green
 * "live · 0 pkts". Compounding it, `connected` is the BROWSER->DASHBOARD
 * websocket, not the rover — `/ws/{channel}` accepts any channel name
 * unconditionally, so it is true whenever the backend process is up and says
 * nothing whatsoever about whether a rover exists. An open socket is not
 * evidence that data exists, so it is no longer allowed to produce "live" on
 * its own; only a real `lastAt` can.
 *
 * `connected` is still read, but only in the direction it is trustworthy: it
 * going FALSE is a genuine dashboard-side fault, and that is surfaced rather
 * than dropped.
 *
 * Staleness is also a function of wall time, not of arriving data. Without the
 * tick below, a feed that simply stopped would keep rendering its last state
 * forever — the component only re-rendered when a message arrived, which is
 * exactly what has stopped happening. Same 1 s tick pattern as lib/mission.ts.
 */

/** No frame for this long and the reading on screen is history, not state. */
const STALE_MS = 3000;

export function StatusPill({
  connected,
  lastAt,
  messages,
}: {
  connected: boolean;
  lastAt: number | null;
  messages: number;
}) {
  const [now, setNow] = useState<number>(() => Date.now());
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, []);

  // Nothing has ever arrived. The socket being open proves only that the
  // dashboard is running, so there is nothing here to call live or stale.
  const noData = messages === 0 && lastAt === null;
  const stale = !noData && (!connected || lastAt === null || now - lastAt > STALE_MS);

  const state: "live" | "stale" | "nodata" = noData ? "nodata" : stale ? "stale" : "live";

  // No-data is deliberately its own colour. Amber already means "this WAS
  // working and stopped", and collapsing the two would tell an operator to go
  // looking for a dropout that never happened. When the dashboard socket is
  // also down that is a real fault of ours, so it keeps the amber it had.
  const cls =
    state === "live" ? "chip-ok" : state === "stale" ? "chip-warn" : connected ? "chip" : "chip-warn";

  const label = state === "live" ? "live" : state === "stale" ? "no signal" : "no data";

  const dot =
    state === "live"
      ? "bg-emerald-300 text-emerald-300 pulse-dot"
      : state === "stale"
        ? "bg-amber-300 text-amber-300 pulse-dot"
        : connected
          ? // Hollow and still: nothing is arriving, and nothing is pretending to.
            "border border-slate-500 bg-transparent text-slate-500"
          : "bg-amber-300 text-amber-300 pulse-dot";

  const title =
    state === "nodata"
      ? connected
        ? "Nothing has ever arrived on this channel. The dashboard websocket is open, but that only means the dashboard is running — it accepts any channel name and is not evidence that a rover is publishing."
        : "Nothing has ever arrived on this channel, AND the browser cannot reach the dashboard backend. Fix the dashboard link before reading anything into the rover's silence."
      : state === "stale"
        ? !connected
          ? `The browser's websocket to the dashboard is down — this is a dashboard-side fault, not the rover.${
              lastAt ? ` Last frame ${new Date(lastAt).toLocaleTimeString()}.` : ""
            }`
          : lastAt === null
            ? "Messages were counted but none carried a timestamp - treat this feed as stale."
            : `No frames for over ${STALE_MS / 1000}s - last at ${new Date(lastAt).toLocaleTimeString()}`
        : `Live · last frame ${new Date(lastAt!).toLocaleTimeString()}`;

  return (
    <span className={cls} title={title} role="status">
      <span className={`inline-block h-2 w-2 shrink-0 rounded-full ${dot}`} />
      <span className={state === "live" ? "" : "font-semibold uppercase tracking-wide"}>{label}</span>
      <span className="font-mono text-2xs opacity-70">· {messages} pkts</span>
    </span>
  );
}

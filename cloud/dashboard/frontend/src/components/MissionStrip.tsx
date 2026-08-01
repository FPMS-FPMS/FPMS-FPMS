import { ErrorBoundary } from "./ErrorBoundary";
import { useMission, type MissionFeed } from "../lib/mission";

/**
 * "IS THE ROVER DRIVING ITSELF RIGHT NOW?" — answered on every tab.
 *
 * A one-line banner, deliberately not a card, so it can sit at the top of any
 * page without displacing that page's own content. Drive keeps its full mission
 * panel; this is the version for everywhere else — Control, where an operator
 * could otherwise fire `test_motors` into a running mission, and LiDAR, which
 * is the tab people actually watch while the rover moves.
 *
 * THE THREE STATES ARE NOT EQUALLY SIZED, ON PURPOSE:
 *
 *   BLIND   running, and the telemetry has stopped. Rose, pulsing, with the
 *           instruction attached: treat it as still moving. This is the only
 *           state that gets a second line, because it is the only one where
 *           what to do about it is not obvious.
 *   RUNNING amber-to-ember and pulsing. Includes every phase this dashboard
 *           does not recognise — see MISSION_IDLE_PHASES.
 *   IDLE    quiet slate. Never pulsing, never coloured. An idle banner that
 *           looks like a warning trains people to ignore the warning.
 *
 * Nothing here renders a number it does not have. `--` is the answer to a
 * missing field, never 0: the executor omits pose and distance fields rather
 * than zero-filling them (fpms_missions.snapshot()), and "0 mm remaining" reads
 * as "arrived" when it means "no idea".
 */

export function MissionStrip({
  thing,
  feed,
  onAbort,
  className = "",
}: {
  thing: string | null;
  /** Pass a feed to share one subscription; omit it to open one here. */
  feed?: MissionFeed;
  /**
   * Abort handler. Ungated by anything: an abort must work precisely when the
   * link checks have decided things are wrong, which is when it is needed.
   */
  onAbort?: () => void;
  className?: string;
}) {
  return (
    <ErrorBoundary label={`${thing ?? "rover"} mission`}>
      <MissionStripInner thing={thing} feed={feed} onAbort={onAbort} className={className} />
    </ErrorBoundary>
  );
}

function MissionStripInner({
  thing,
  feed,
  onAbort,
  className,
}: {
  thing: string | null;
  feed?: MissionFeed;
  onAbort?: () => void;
  className?: string;
}) {
  // Hooks cannot be conditional, so the subscription is always opened; passing
  // `null` when a feed was supplied makes it a no-op rather than a second
  // socket to the same channel.
  const own = useMission(feed ? null : thing);
  const f = feed ?? own;
  const m = f.mission;

  // Nothing has ever arrived. Silence is NOT "idle" — the executor may not be
  // running at all — so this says which it is instead of guessing.
  if (!f.messages) {
    return (
      <div
        className={`flex flex-wrap items-center gap-3 rounded-xl border border-white/5 bg-black/30 px-4 py-2.5 text-sm ${className}`}
      >
        <span className="chip">MISSION · NO TELEMETRY</span>
        <span className="text-slate-400">
          Nothing has arrived on{" "}
          <span className="font-mono">{thing ? `mission:${thing}` : "the mission channel"}</span> —
          the executor may not be running. This is not the same as “idle”.
        </span>
      </div>
    );
  }

  const running = !!m?.running;
  const tone = f.blind
    ? "border-rose-500/50 bg-rose-950/40"
    : running
      ? "border-ember-500/40 bg-ember-500/10"
      : "border-white/5 bg-black/30";

  return (
    <div className={`rounded-xl border-2 px-4 py-2.5 text-sm ${tone} ${className}`}>
      <div className="flex flex-wrap items-center gap-x-4 gap-y-1.5">
        <span className="flex items-center gap-2">
          {(running || f.blind) && (
            <span
              className={`inline-block h-2.5 w-2.5 shrink-0 rounded-full pulse-dot ${
                f.blind ? "bg-rose-400 text-rose-400" : "bg-ember-400 text-ember-400"
              }`}
            />
          )}
          <span
            className={`font-semibold tracking-wide ${
              f.blind ? "text-rose-100" : running ? "text-ember-200" : "text-slate-400"
            }`}
          >
            {f.blind ? "MISSION — TELEMETRY LOST WHILE DRIVING" : running ? "MISSION RUNNING" : "MISSION IDLE"}
          </span>
        </span>

        <span className="font-mono text-xs text-slate-300">
          {m?.mission ?? "--"}
          {m?.backend ? <span className="text-slate-500"> · via {m.backend}</span> : null}
        </span>
        <span className="font-mono text-xs text-slate-400" title="Mission phase as reported by the executor">
          phase {m?.phase ?? "--"}
        </span>
        {/* Which CORNER, named the way an operator would say it. This is the
            field that stops "m1" being read as the zone it is not. */}
        {m?.legLabel || m?.leg ? (
          <span className="font-mono text-xs text-sky-200/90" title="Leg the executor is driving now">
            → {m.legLabel ?? m.leg}
            {m?.legI !== null && m?.legI !== undefined && m?.legsN
              ? ` (${m.legI}/${m.legsN})`
              : ""}
          </span>
        ) : null}
        <span className="font-mono text-xs text-slate-400">
          seg {m?.segmentI !== null && m?.segmentI !== undefined && m?.segmentsN
            ? `${m.segmentI}/${m.segmentsN}`
            : "--"}
        </span>
        <span className="font-mono text-xs text-slate-400">
          remaining {m?.remainingMm !== null && m?.remainingMm !== undefined
            ? `${Math.round(m.remainingMm)} mm`
            : "--"}
        </span>
        <span className="font-mono text-xs text-slate-400">
          pose{" "}
          {m?.poseX !== null && m?.poseX !== undefined && m?.poseY !== null && m?.poseY !== undefined
            ? `(${Math.round(m.poseX)}, ${Math.round(m.poseY)}) mm`
            : "--"}
        </span>

        {/* Staleness is only shouted about while running — the executor
            deliberately throttles idle telemetry, so a quiet idle feed is
            normal and flagging it would train the operator to ignore the chip
            that matters. */}
        {f.blind ? (
          <span className="chip-hot" title="Last known state was DRIVING and telemetry has since stopped">
            stale {Math.round((f.ageMs ?? 0) / 1000)}s
          </span>
        ) : f.stale && running ? (
          <span className="chip-warn">stale {Math.round((f.ageMs ?? 0) / 1000)}s</span>
        ) : null}

        {m?.linkOk === false ? <span className="chip-hot">micro-ROS link down</span> : null}
        {m?.lidarOk === false ? <span className="chip-hot">LiDAR stale — guard blind</span> : null}
        {m?.poseAssumed ? (
          <span className="chip-warn" title="Nothing localises this rover; the pose is dead-reckoned from an assumed start">
            pose assumed
          </span>
        ) : null}

        {onAbort && (running || f.blind) ? (
          <button className="btn-hot ml-auto" onClick={onAbort} title="Abort the running mission">
            ABORT MISSION
          </button>
        ) : null}
      </div>

      {f.blind ? (
        <p className="mt-1.5 text-xs text-rose-200/90">
          <b>The rover was driving when telemetry stopped.</b> Treat it as still
          moving until you can see otherwise. Stop it before approaching.
        </p>
      ) : null}
    </div>
  );
}

export default MissionStrip;

import { type MissionPlan, type ReplanDetail } from "../lib/mission";

/**
 * PLANNER STATE for the route currently drawn on a map.
 *
 * This lived inside pages/Lidar.tsx and moved here the moment a SECOND page
 * needed to draw the same arena map — the Rover 2 Mission Test tab, which puts
 * the map and the run buttons on one screen so the operator's hand can stay on
 * FULL STOP while they watch. Copying the band would have been the easy version
 * and the wrong one: the whole value of it is the occupancy-age arithmetic
 * below, and two copies of that would drift, leaving one page confidently
 * reporting a fresh grid while the other called the same grid stale.
 *
 * Renders nothing when there is no plan — this band must not add a row of
 * dashes to a card that is simply idle.
 */

/**
 * An occupancy grid older than this is describing a world that has moved on.
 *
 * The grid ages independently of the LiDAR channel the map draws: the rover
 * folds scans into it on the mission node, so the map on screen can be updating
 * at 5 Hz while the thing the PLANNER is routing against has not been refreshed
 * in a minute. Those two staleness states look identical from the map alone,
 * which is half of why "the plan isn't working" has been so hard to pin down.
 */
export const OCC_STALE_S = 20;

/** Compact "5s" / "2m 10s". */
export function ageText(s: number | null): string {
  if (s === null) return "--";
  if (s < 60) return `${s < 10 ? s.toFixed(1) : Math.round(s)}s`;
  const m = Math.floor(s / 60);
  return `${m}m ${Math.round(s % 60)}s`;
}

export function PlannerBand({
  plan,
  replan,
  replanCount,
  now,
  className = "",
}: {
  plan: MissionPlan | null;
  replan: ReplanDetail | null;
  replanCount: number;
  now: number;
  className?: string;
}) {
  if (!plan) return null;

  const occ = plan.occupancy;
  // The grid's age is reported by the rover as of the moment it published the
  // plan, so the time since we RECEIVED that plan has to be added back on.
  // Showing the rover's figure alone would freeze at whatever it was when the
  // plan landed and read as fresh forever — the exact failure this band exists
  // to make visible.
  const sincePlanS = plan.ts === null ? null : Math.max(0, (now - plan.ts * 1000) / 1000);
  const occAgeS =
    occ === null || occ.ageS === null ? null : occ.ageS + (sincePlanS ?? 0);
  const occStale = occAgeS !== null && occAgeS > OCC_STALE_S;
  const astar = (plan.planner ?? "").toLowerCase() === "astar";

  return (
    <div className={`flex flex-wrap items-center gap-2 text-[11px] ${className}`}>
      {plan.planner && (
        <span
          className={astar ? "chip-ok font-mono" : "chip font-mono"}
          title={
            astar
              ? "A* routed this line around cells the rover has actually observed"
              : `Planner reported by the rover: ${plan.planner}`
          }
        >
          planner · {plan.planner}
        </span>
      )}

      {occ && (
        <span
          className={occStale ? "chip-warn font-mono" : "chip font-mono"}
          title={
            occStale
              ? `The occupancy grid this route was planned against last saw a scan ` +
                `${ageText(occAgeS)} ago. Obstacles in it may no longer exist, and ` +
                `new ones will not be in it.`
              : `Occupancy grid: ${occ.cells ?? "?"} occupied of ${
                  occ.cellsTracked ?? "?"
                } tracked, ${occ.scans ?? "?"} scans folded in`
          }
        >
          grid · {occ.cells ?? "?"} cells · {ageText(occAgeS)}
          {occStale ? " STALE" : ""}
        </span>
      )}

      {replanCount > 0 && (
        <span
          className="chip-warn font-mono"
          title={
            replan
              ? `Last reroute on leg ${replan.leg ?? "?"} (${replan.replanI ?? "?"}/${
                  replan.replanMax ?? "?"
                }): ${replan.reason ?? "no reason given"}${
                  replan.viaN ? ` — ${replan.viaN} detour point(s)` : ""
                }`
              : "The executor has rerouted mid-leg"
          }
        >
          replans · {replanCount}
          {replan?.replanMax ? ` / ${replan.replanMax} per leg` : ""}
        </span>
      )}

      {/* The planner's own words. Truncated on screen, full text on hover — it
          can be several clauses long when more than one thing forced the
          detour, and this band must stay on one line. */}
      {plan.plannerNote && (
        <span className="max-w-full truncate text-slate-400" title={plan.plannerNote}>
          {plan.plannerNote}
        </span>
      )}

      {replan?.reason && (
        <span className="text-amber-300/80" title="Reason for the newest reroute">
          rerouted: {replan.reason}
        </span>
      )}
    </div>
  );
}

export default PlannerBand;

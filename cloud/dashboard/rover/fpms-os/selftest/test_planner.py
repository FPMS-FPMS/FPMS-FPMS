#!/usr/bin/env python3
"""Unit checks for the fpms_missions.py PLANNER. No Pi, no ROS, no LiDAR.

    python3 selftest/test_planner.py

WHY THIS FILE EXISTS
--------------------
The executor plans autonomously: Theta*/A* over a live LiDAR occupancy grid,
with a turn cost, a clearance gradient, hysteresis, a three-rung relaxation
ladder and an `ObstacleDetour` live reroute. All of that is gate-verified
END TO END at m2 = 744.0 mm by `test_stack.py` -- and that gate runs on a CLEAN
arena with `grid=None`, so it exercises none of it.

Nothing proved that A* finds the optimal path on a known grid, that Theta*
actually shortcuts where line of sight allows, that the ladder degrades in the
documented order, or that the clearance gradient keeps the rover off walls.

That gap matters because the planner is the part most likely to be "improved"
later, and A PLANNER REGRESSION DOES NOT CRASH. It drives a worse route, and on
a 1.2 m arena a worse route looks exactly like a calibration problem -- which is
the most expensive thing this project can be made to chase, because the rover
has several real calibration problems and they would absorb the blame.

HOW THIS FILE IMPORTS A MODULE THAT WANTS ROS
---------------------------------------------
It does not need to stub anything, and that is a property worth stating rather
than assuming. `fpms_missions.py` guards `rclpy`, `paho`, `nav2_msgs` and `tf2`
in try/except at lines 374-427, sets `Node = object` when rclpy is absent, and
only raises `SystemExit` inside `main()` (lines 6238-6241), which is never
reached on import. Its own docstring (line 355) states the contract: "This
module is importable WITHOUT rclpy or paho installed."

So the import is a plain `importlib.util.spec_from_file_location`, exactly as
`test_stack.py` already does it. Two consequences, both deliberate:

  * NO `sys.modules` STUBBING. A stub is a second implementation of somebody
    else's package, and a wrong one silently changes what is being tested. If
    the guards above are ever removed, this file must FAIL LOUDLY rather than
    quietly test a mock -- which is what `load_planner()` below does.
  * IF THE IMPORT FAILS, THIS FILE SKIPS AND SAYS WHY, and `main()` still
    returns non-zero. A planner test that silently passes because it could not
    load the planner is worse than no planner test.

WHICH CONSTANTS THE TESTS RUN AGAINST -- READ BEFORE JUDGING A NUMBER
---------------------------------------------------------------------
`fpms_missions.py` reads `/etc/fpms/config.env` at import. On the rover that
file exists; on a laptop it does not, and the module falls back to its own
defaults. The two differ in ways that reach the planner:

    FPMS_MISSION_CRUISE_MPS    shipped 0.18   default 0.08
    FPMS_MISSION_MAX_LEG_MM    shipped 70     default 300
    -> LEG_EFFECTIVE_MM_S      shipped 83.4   default 71.4
    -> turn_cost_mm(90 deg)    shipped 391    default 335    (mm of driving)

So EVERY EXPECTATION HERE IS DERIVED FROM THE MODULE'S OWN CONSTANTS, never
from a literal. A test that hard-coded 391 would pass on the rover and fail on
a laptop while the planner was identical, which teaches nothing. The one place
the shipped numbers are named is `test_16_docs_arithmetic`, which re-derives
them from `overlay/etc/fpms/config.env` through the module's own formula and
checks them against `docs/PLANNING.md` -- a documentation check, not a planner
check, and labelled as such.

THE TOLERANCES, AND WHY THEY ARE THESE NUMBERS
-----------------------------------------------
STRAIGHT_TOL_MM = 0.05   A straight line's length is exact arithmetic, not an
                  estimate, so this is float noise and nothing else. It is the
                  same 0.05 mm `test_stack.py:96` uses on the 744.0 mm gate,
                  deliberately: the two files are checking the same claim about
                  the same geometry from opposite ends. Anything above this
                  means the search inserted a vertex where line of sight
                  existed -- i.e. Theta* has degenerated into A*. Observed: 0.0
                  on all four targets.

THETA_OPT_FRAC = 0.05    Theta* is HONESTLY DOCUMENTED as not optimal
                  (fpms_missions.py:2677-2685): re-parenting to a visible
                  grandparent can miss a better ancestor, and the turn term
                  makes cost history-dependent on top of that. So the test asks
                  for near-optimal, not optimal, against an independently
                  computed true optimum (see `optimal_cost` below). 5 % is a bit
                  over twice the worst gap measured here (2.35 %), and well
                  under the gap the 45-degree-quantised search shows (7.3-8.3 %)
                  -- so a regression that turned Theta* back into A* fails this,
                  which is the specific regression worth catching.

ASTAR_OPT_FRAC = 0.15    The `astar` setting is quantised to 45 degrees by
                  construction, so it CANNOT be near-optimal on an any-angle
                  cost and it is not being asked to be. Measured worst gap
                  8.34 %; 15 % is loose enough not to be a tripwire for grid
                  quantisation and tight enough that a gap above it means the
                  string-puller (`shortcut_points`) has stopped working. The
                  raw, un-pulled A* route measures 50-64 % over optimal, so the
                  failure mode this bounds is real and large.

CLEAR_EPS_MM = 1e-6      NOT A TOLERANCE. The pad is a hard safety bound and
                  this is float slack on a comparison against zero. It is never
                  to be widened: `clear_mm` already returns the margin AFTER
                  subtracting the robot radius and the point pad, so a negative
                  value means the chassis is inside a measured surface, and no
                  amount of it is acceptable. If a clearance check here fails,
                  the finding is the planner, not the number.

sampling dip     DERIVED, NOT CHOSEN, and it is the one place a route may go
                  measurably negative. `segment_ok` samples the line at half a
                  cell and asks for `clear_mm >= 0` AT THE SAMPLES; between two
                  samples the line can cut the chord of the forbidden disc. The
                  deepest that chord can be is exact geometry:

                      dip = (step/2)^2 / (2 * reach)

                  = 0.40 mm at the live 25 mm step and 195 mm reach. So the
                  driven line is checked against -dip, computed from the
                  module's own constants at run time rather than typed, and the
                  observed dip is printed. Measured worst: 0.006 mm. If that
                  ever approaches 0.40 mm the answer is a finer `step_mm`, not
                  a bigger number here.

WP_MAX_ONE_OBSTACLE = 4  One convex obstacle needs at most two bends to get
                  round, so anything above ~4 waypoints means the route is
                  staircasing. 4 rather than 2 because `prune_via` may only drop
                  a vertex whose replacement segment is `segment_ok`, and it
                  keeps bends above BEARING_TOL_DEG -- both correct, both leave
                  a residue. Observed: theta 4, astar 2 on the reference scene.

WHAT THIS FILE FOUND, FIRST TIME IT WAS RUN (2026-08-13)
--------------------------------------------------------
Recorded here rather than in a commit message, because the next person to touch
the planner will read this file and not that. Every one of these is a WARNING
below, not a failure: none is a broken invariant, and none was worked around.

  1. `astar` IS SOMETIMES CHEAPER THAN THE `theta` DEFAULT, on the planner's own
     cost function, at the LIVE 50 mm resolution -- 2 of 5 routable scenes, by
     up to 4.5 %, with fewer waypoints (test_05b). At the coarser 100 mm
     resolution the ordering is the expected one and is not close (theta 0.9-1.0 %
     from optimal, astar 7.3-8.3 %). So the any-angle advantage SHRINKS as the
     cells get fine relative to the 195 mm reach, and at the shipped resolution
     it can invert. `theta_star`'s own docstring is explicit that basic Theta*
     is not optimal, so this is expected in kind -- the size and the direction
     are what nobody had measured.

  2. THE DEFAULT `FRONT_STOP_MM` IS INSIDE THE DISTANCE THE PLANNER CAN PLAN OUT
     OF (test_15). The guard stops at 120 mm by default; the search cannot
     produce a full-margin route until the nearest surface is ~185 mm away,
     because below that every neighbour of the rover's own cell is inflated and
     `free=(here,)` frees exactly one cell. The shipped `config.env` sets
     400 mm, so THE ROVER IS FINE -- but `_cfg_float` permits values down to
     60 mm and nothing couples the two numbers, so lowering the stop distance
     would silently turn every `ObstacleDetour` back into `ABORT_OBSTACLE`.

  3. ROUTES GRAZE THE INFLATION BOUNDARY (test_06). On the reference scenes the
     tightest margin on the driven line is 0.003-0.14 mm: legal, and exactly the
     "path that is legal by one millimetre" the clearance gradient was added to
     discourage. CLEARANCE_COST_W = 0.25 is correctly too small to buy a detour
     on a 1.2 m arena (test_01 checks that bound), so it can only break ties --
     and at a tangent point there is no tie to break. Worth knowing on a rover
     whose entire pose-error budget at a target is 58 mm.

  4. `inflate_blocked()` AND `simplify_cells()` HAVE NO CALL SITE anywhere in
     the repository (test_16). The search runs on `CostMap.blocked`. They are
     exercised here anyway so they do not rot, but `CostMap`'s docstring
     (fpms_missions.py:2286-2288) says of `inflate_blocked` "that is fine for a
     search, and it is still what the search uses", and that sentence is no
     longer true.

  5. `segment_ok` SAMPLES AT HALF A CELL, so the driven line may dip up to
     (step/2)^2 / (2*reach) = 0.40 mm inside the pad between two samples. Its
     docstring's claim -- that half-cell sampling "cannot step over an obstacle
     whose forbidden disc is >= 170 mm across" -- is true for MISSING a disc and
     not for clipping its edge. The bound is small, exact, and derived rather
     than assumed; `sampling_dip_mm()` computes it and the tests use it.

WHAT THIS FILE CANNOT PROVE
---------------------------
That the LiDAR bearings feeding `OccupancyGrid.integrate` are the bearings the
scan really has (`LIDAR_ROTATION_SIGN` is still UNVERIFIED, and a mirror is not
a rigid transform). That the pose the grid is folded through is where the rover
is -- it is dead reckoning from an ASSUMED origin. That 170 mm is the chassis
half-width; it has never been measured. Every obstacle in every test below is
placed by hand at a known arena coordinate, so this file proves the SEARCH is
right about the map it is given, and says nothing about the map.

no pytest, no numpy, no scipy -- none of the three is on the rover.
"""

import importlib.util
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                      # fpms-os/
SRC = os.path.dirname(ROOT)                       # rover/
MISSIONS = os.path.join(SRC, "fpms_missions.py")
SHIPPED_CFG = os.path.join(ROOT, "overlay/etc/fpms/config.env")
PLANNING_MD = os.path.join(ROOT, "docs/PLANNING.md")

# -- tolerances; every one justified in the module docstring above ----------
STRAIGHT_TOL_MM = 0.05
THETA_OPT_FRAC = 0.05
ASTAR_OPT_FRAC = 0.15
CLEAR_EPS_MM = 1e-6
WP_MAX_ONE_OBSTACLE = 4

PASSES, FAILURES, WARNINGS, SKIPS = [], [], [], []
M = None                                          # the planner module


def check(name, cond, detail=""):
    (PASSES if cond else FAILURES).append(name)
    print("  [%s] %s%s" % ("PASS" if cond else "FAIL", name,
                           ("  %s" % detail) if detail else ""))
    return bool(cond)


def warn(name, detail):
    WARNINGS.append(name)
    print("  [WARN] %s  %s" % (name, detail))


def skip(name, detail):
    SKIPS.append(name)
    print("  [SKIP] %s  %s" % (name, detail))


# =========================================================== IMPORT STRATEGY
def load_planner():
    """Import fpms_missions.py off-robot. Returns the module, or None LOUDLY.

    See the module docstring: no stubbing, because the file already guards its
    optional imports and a stub would be a second implementation of somebody
    else's package. The only thing this adds is a clear failure.
    """
    if not os.path.exists(MISSIONS):
        skip("import", "fpms_missions.py not found at %s -- NOTHING BELOW RAN"
             % MISSIONS)
        return None
    try:
        spec = importlib.util.spec_from_file_location("fpms_missions_uut",
                                                      MISSIONS)
        mod = importlib.util.module_from_spec(spec)
        sys.modules["fpms_missions_uut"] = mod
        spec.loader.exec_module(mod)
    except BaseException as exc:                            # SystemExit too
        skip("import",
             "fpms_missions.py could not be imported without ROS/MQTT (%r). "
             "Its docstring at line 355 promises it can be, and its optional "
             "imports are guarded at lines 374-427; if that guard was removed, "
             "the geometry can no longer be tested off-robot and THAT is the "
             "finding. NOTHING BELOW RAN." % exc)
        return None
    for need in ("theta_star", "astar_cells", "CostMap", "OccupancyGrid",
                 "plan_detour", "path_cost_mm", "RELAX_LADDER"):
        if not hasattr(mod, need):
            skip("import", "loaded module has no %r -- the planner has been "
                           "renamed or moved. NOTHING BELOW RAN." % need)
            return None
    return mod


# ================================================================== FIXTURES
NOW = 1000.0            # a fixed fake clock; nothing here may depend on wall time


def grid_with(points, now=NOW, hits=None):
    """An OccupancyGrid that BELIEVES `points`, built the way the rover does.

    Marked `OCC_MIN_HITS + 1` times, because one return is noise by design
    (OCC_MIN_HITS) and a test that marked once would be testing the threshold
    rather than the planner. The clock is passed explicitly so nothing here can
    expire mid-test.
    """
    g = M.OccupancyGrid()
    for _ in range(hits if hits is not None else (M.OCC_MIN_HITS + 1)):
        for (x, y) in points:
            g.mark(float(x), float(y), now=now)
    return g


def costmap(points, n=None, cell=None, arena=None, **kw):
    """A CostMap over hand-placed obstacle points, none of them arena wall.

    `is_wall=False` on every point on purpose: `points()` would otherwise
    absorb anything within WALL_BAND_MM of the boundary into the static layer,
    and a test whose obstacle silently vanished would pass for the wrong reason.
    """
    n = n if n is not None else int(math.ceil(M.ARENA_MM / M.GRID_MM))
    cell = cell if cell is not None else M.GRID_MM
    arena = arena if arena is not None else M.ARENA_MM
    pts = [(float(x), float(y), False) for (x, y) in points]
    return M.CostMap(pts, n, cell, arena_mm=arena, **kw)


def map_of(grid, free_at=None, **kw):
    """The costmap `plan_detour` would have built from this grid.

    THE VERIFICATION MAP MUST BE THE SAME MAP. `OccupancyGrid.points()` returns
    ONE CENTROID PER CELL, not the raw returns that were marked -- sixteen
    points on a 60 mm ring collapse to four. A test that re-derived a costmap
    from its own raw obstacle list would be a STRICTER map than the planner
    ever saw, and would then fail routes that are perfectly legal. Measured
    while writing this file: that mistake failed 19 of 106 fuzz trials, none of
    them a planner defect.
    """
    free = () if free_at is None else (grid.cell_of(free_at[0], free_at[1]),)
    return M.CostMap(grid.points(now=NOW), grid.n, grid.cell_mm,
                     arena_mm=grid.arena_mm, free=free, **kw)


def sampling_dip_mm(cmap):
    """How far inside the pad the DRIVEN line may legally dip. Derived.

    `segment_ok` checks `clear_mm >= 0` at samples half a cell apart; between
    two samples the line cuts a chord of the forbidden disc. Exact geometry:
    a chord whose half-length is step/2 on a circle of radius `reach` dips
    (step/2)^2 / (2*reach) below the surface. Nothing is chosen here.
    """
    step = cmap.cell_mm * 0.5
    return (step / 2.0) ** 2 / (2.0 * max(cmap.reach, 1e-9))


def ring(cx, cy, r=30.0, k=16):
    """A small convex obstacle, as a ring of measured surface points."""
    return [(cx + r * math.cos(2 * math.pi * i / k),
             cy + r * math.sin(2 * math.pi * i / k)) for i in range(k)]


def hwall(y, x0, x1, step=25.0):
    """A straight run of surface returns -- a wall segment the LiDAR saw."""
    out, x = [], float(x0)
    while x <= x1 + 1e-9:
        out.append((x, float(y)))
        x += step
    return out


def polyline(sx, sy, pts, tx, ty):
    return [(float(sx), float(sy))] + [tuple(p) for p in (pts or [])] + \
           [(float(tx), float(ty))]


def length_mm(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def total_turn_deg(pts, heading_deg):
    """Every heading change the follower would actually be commanded to make."""
    tot, hdg = 0.0, math.radians(float(heading_deg))
    px, py = pts[0]
    for qx, qy in pts[1:]:
        if math.hypot(qx - px, qy - py) <= 1e-9:
            continue
        b = math.atan2(qy - py, qx - px)
        tot += abs(math.degrees(M.wrap_pi(b - hdg)))
        hdg, px, py = b, qx, qy
    return tot


def worst_clearance(cmap, pts, step_mm=2.0):
    """The tightest margin anywhere on the DRIVEN LINE, not just at vertices.

    Sampled at 2 mm, far finer than `segment_ok`'s half-cell, because the
    question here is "how close did it really get", not "is it legal".
    Returns (margin_mm, (x, y)).
    """
    best = (float("inf"), None)
    for a, b in zip(pts, pts[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        k = max(1, int(math.ceil(d / step_mm)))
        for i in range(k + 1):
            t = float(i) / k
            x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            c = cmap.clear_mm(x, y)
            if c < best[0]:
                best = (c, (x, y))
    return best


def drivable(cmap, pts):
    return all(cmap.segment_ok(a[0], a[1], b[0], b[1])
               for a, b in zip(pts, pts[1:]))


def planned(sx, sy, hdg, tx, ty, grid, planner=None, committed=None):
    """`plan_detour` plus the full polyline, so callers can measure it."""
    r = M.plan_detour(float(sx), float(sy), float(tx), float(ty), grid,
                      now=NOW, heading_deg=float(hdg), committed=committed,
                      planner=planner)
    return r, (None if r.points is None else polyline(sx, sy, r.points, tx, ty))


# ========================================================= INDEPENDENT ORACLES
def brute_force_grid_cost(blocked, n, start, goal):
    """The TRUE 8-connected cost, by Bellman-Ford relaxation to a fixpoint.

    DELIBERATELY NOT A SECOND A*. No heuristic, no heap, no closed set, no
    tie-break -- so it shares no mechanism with `astar_cells` and cannot
    reproduce its bug. It only shares the MOVEMENT RULES, which are the thing
    being agreed on: sqrt(2) diagonals and no corner cutting between two
    blocked orthogonal neighbours.

    Returns inf when the goal is unreachable.
    """
    INF = float("inf")
    g = {}
    for ix in range(n):
        for iy in range(n):
            g[(ix, iy)] = INF
    g[start] = 0.0
    for _ in range(n * n):
        changed = False
        for cx in range(n):
            for cy in range(n):
                if (cx, cy) in blocked or g[(cx, cy)] == INF:
                    continue
                for dx, dy in M.NEIGHBOURS:
                    nx, ny = cx + dx, cy + dy
                    if not (0 <= nx < n and 0 <= ny < n):
                        continue
                    if (nx, ny) in blocked:
                        continue
                    if dx and dy and ((cx + dx, cy) in blocked
                                      or (cx, cy + dy) in blocked):
                        continue
                    w = M.DIAG_COST if (dx and dy) else 1.0
                    if g[(cx, cy)] + w < g[(nx, ny)] - 1e-12:
                        g[(nx, ny)] = g[(cx, cy)] + w
                        changed = True
        if not changed:
            break
    return g[goal]


def optimal_cost(cmap, start_xy, goal_xy, heading_deg):
    """The cheapest route Theta* COULD have returned, computed independently.

    THE COMPARISON CLASS IS THE POINT, so it is stated exactly. `theta_star`
    can only ever emit a polyline whose interior vertices are CELL CENTRES
    (`pos()`, fpms_missions.py:2706-2716), whose consecutive vertices have line
    of sight, and whose endpoints are the true pose and the true target. This
    computes the CHEAPEST member of that same class, under the same
    `segment_cost_mm` + `turn_cost_mm` the search minimises.

    Dijkstra over states (previous vertex, current vertex) -- the pair is the
    state because the turn charge at a vertex depends on how the route ARRIVED
    there, which is exactly the history-dependence `theta_star`'s own docstring
    admits it does not resolve. Successors are ANY visible vertex, not just the
    eight neighbours, so this is a genuine lower bound on what the search can
    achieve rather than a re-run of it.

    O(V^2) states, so it is only usable on the coarse worlds in
    `test_05_theta_near_optimal`. Returns inf when nothing is reachable.
    """
    sc, gc = cmap.cell_of(*start_xy), cmap.cell_of(*goal_xy)

    def pos(c):
        if c == sc:
            return (float(start_xy[0]), float(start_xy[1]))
        if c == gc:
            return (float(goal_xy[0]), float(goal_xy[1]))
        return cmap.centre_of(*c)

    verts = [(ix, iy) for ix in range(cmap.n) for iy in range(cmap.n)
             if (ix, iy) not in cmap.blocked or (ix, iy) in (sc, gc)]
    _los, _seg = {}, {}

    def los(a, b):
        k = (a, b) if a <= b else (b, a)
        v = _los.get(k)
        if v is None:
            ax, ay = pos(a)
            bx, by = pos(b)
            v = _los[k] = cmap.segment_ok(ax, ay, bx, by)
        return v

    def seg(a, b):
        k = (a, b) if a <= b else (b, a)
        v = _seg.get(k)
        if v is None:
            ax, ay = pos(a)
            bx, by = pos(b)
            v = _seg[k] = cmap.segment_cost_mm(ax, ay, bx, by)
        return v

    import heapq
    INF = float("inf")
    dist, heap = {}, []
    ax, ay = pos(sc)
    h0 = math.radians(float(heading_deg))
    for q in verts:
        if q == sc or not los(sc, q):
            continue
        bx, by = pos(q)
        if math.hypot(bx - ax, by - ay) <= 1e-9:
            continue
        c = seg(sc, q) + M.turn_cost_mm(
            M.wrap_pi(math.atan2(by - ay, bx - ax) - h0))
        if c < dist.get((sc, q), INF):
            dist[(sc, q)] = c
            heapq.heappush(heap, (c, sc, q))
    best = INF
    while heap:
        c, p, cur = heapq.heappop(heap)
        if c > dist.get((p, cur), INF) + 1e-9:
            continue
        if cur == gc:
            best = min(best, c)
            continue
        px, py = pos(p)
        cx, cy = pos(cur)
        hin = math.atan2(cy - py, cx - px)
        for q in verts:
            if q == cur or not los(cur, q):
                continue
            qx, qy = pos(q)
            if math.hypot(qx - cx, qy - cy) <= 1e-9:
                continue
            nc = c + seg(cur, q) + M.turn_cost_mm(
                M.wrap_pi(math.atan2(qy - cy, qx - cx) - hin))
            if nc < dist.get((cur, q), INF) - 1e-9:
                dist[(cur, q)] = nc
                heapq.heappush(heap, (nc, cur, q))
    return best


# ================================================================ THE TESTS
def test_01_constants_are_sane():
    """The numbers the rest of this file derives its expectations from.

    Guards against a planner "improvement" that quietly re-tunes a constant out
    of the range its own derivation assumes -- a change that would move every
    route without failing anything, because there is nothing else that reads
    these together.
    """
    print("\n1. planner constants")
    n = int(math.ceil(M.ARENA_MM / M.GRID_MM))
    check("grid is n x n with n = ceil(arena/cell)", n == 24,
          "n=%d (arena %.0f, cell %.0f)" % (n, M.ARENA_MM, M.GRID_MM))
    check("WALL_PAD_MM defaults to ROBOT_RADIUS_MM",
          abs(M.WALL_PAD_MM - M.ROBOT_RADIUS_MM) < 1e-9,
          "pad=%.1f radius=%.1f" % (M.WALL_PAD_MM, M.ROBOT_RADIUS_MM))
    # The reach is what turns a point path into one this chassis can occupy.
    # If it ever drops below one cell, a point obstacle stops blocking any cell
    # centre and the cell view of the map goes blind -- the search would then
    # plan straight through it and only `segment_ok` would object, which shows
    # up as a false "no route", not as a detour.
    reach = M.ROBOT_RADIUS_MM + M.OCC_POINT_PAD_MM
    check("reach (radius + point pad) exceeds one grid cell", reach > M.GRID_MM,
          "reach=%.1f cell=%.1f" % (reach, M.GRID_MM))
    check("turn cost is derived, not typed: a 90 deg turn costs real mm",
          M.turn_cost_mm(math.pi / 2) > 100.0,
          "%.1f mm" % M.turn_cost_mm(math.pi / 2))
    # A turn under the smallest the follower can command must be free, or the
    # planner prices a rounding error and prefers routes for no reason.
    check("a turn below BEARING_TOL_DEG is free",
          M.turn_cost_mm(math.radians(M.BEARING_TOL_DEG * 0.9)) == 0.0)
    check("a turn above BEARING_TOL_DEG is not free",
          M.turn_cost_mm(math.radians(M.BEARING_TOL_DEG * 1.1)) > 0.0)
    # CLEARANCE_COST_W is a bribe to leave the wall. Its own derivation says it
    # must be too small to buy a turn on a 1.2 m arena; if it ever gets big
    # enough, the planner starts taking longer turnier routes to avoid squeezes
    # it could safely drive, which is worse than the hugging it was fixing.
    turn90 = M.turn_cost_mm(math.pi / 2)
    hug_mm = turn90 / max(M.CLEARANCE_COST_W, 1e-9)
    check("clearance bribe cannot buy a 90 deg turn inside this arena",
          hug_mm > M.ARENA_MM,
          "would need %.0f mm of continuous hugging; arena is %.0f mm"
          % (hug_mm, M.ARENA_MM))
    check("PLANNER is one of the three documented settings",
          M.PLANNER in ("theta", "astar", "straight"), "= %r" % M.PLANNER)


def test_02_empty_arena_is_a_straight_line():
    """PROPERTY 1. On a clean arena every target is a straight line.

    This is the regression that matters most and costs least. Theta* exists to
    avoid staircasing, so where line of sight exists it must emit exactly two
    points -- the true pose and the true target -- and a path length equal to
    the Euclidean distance to within float noise. A staircase here would still
    reach the zone, so it would never be noticed on the field; it would just
    spend a 300-400 mm turn charge per invented vertex and inherit +/-1-4 deg of
    heading error at each, which reads as odometry drift in the log.

    THE TWO PLANNERS ARE HELD TO DIFFERENT STANDARDS HERE, ON PURPOSE. Theta*
    must emit the straight line FROM THE SEARCH -- that is the whole any-angle
    claim, and it is why `theta` is the default. `astar` is 8-connected and
    physically cannot: it returns sixteen cell centres up the column. It is
    therefore judged on the route that is actually PUBLISHED, after
    `shortcut_points` and `prune_via`, which is what `plan_detour` does with it.
    Asserting the raw A* route were straight would be asserting A* is not A*.
    """
    print("\n2. empty arena -> straight lines (property 1)")
    cmap = costmap([])
    start = (M.ROVER_START["x_mm"], M.ROVER_START["y_mm"])
    hdg = M.ROVER_START["heading_deg"]
    targets = [("m1/zone-a", M.zone_center("zone-a")),
               ("m2/zone-b", M.zone_center("zone-b")),
               ("water-station", M.zone_center("water-station"))]
    for name, (tx, ty) in targets:
        eu = math.hypot(tx - start[0], ty - start[1])
        raw = {}
        for mode in ("theta", "astar"):
            route = M._search(cmap, start[0], start[1], hdg, tx, ty, mode)
            if route is None:
                check("%s: %s returns a route on a clean arena" % (name, mode),
                      False, "got None")
                continue
            raw[mode] = route
            published = M.prune_via(M.shortcut_points(route, cmap, hdg), cmap)
            got = length_mm(published)
            check("%s: %s publishes the straight line" % (name, mode),
                  abs(got - eu) <= STRAIGHT_TOL_MM,
                  "got %.3f mm, Euclidean %.3f mm (tol %.2f)"
                  % (got, eu, STRAIGHT_TOL_MM))
            check("%s: %s publishes no intermediate waypoints" % (name, mode),
                  len(published) == 2, "got %d points" % len(published))
            check("%s: %s ends on the true target, not a cell centre"
                  % (name, mode),
                  abs(published[-1][0] - tx) < 1e-9
                  and abs(published[-1][1] - ty) < 1e-9,
                  "ends at (%.3f, %.3f)" % published[-1])
        if "theta" in raw:
            check("%s: Theta* is straight BEFORE any string-pulling" % name,
                  len(raw["theta"]) == 2
                  and abs(length_mm(raw["theta"]) - eu) <= STRAIGHT_TOL_MM,
                  "raw search returned %d points, %.3f mm"
                  % (len(raw["theta"]), length_mm(raw["theta"])))
        if "astar" in raw and len(raw["astar"]) <= 2:
            # Not a failure -- but if it ever happens, `astar` has stopped being
            # the 45-degree-quantised control this file compares theta against,
            # and test_05's comparison would silently become vacuous.
            warn("empty arena/%s" % name,
                 "raw A* returned %d points on a clear line; it is expected to "
                 "staircase and be straightened by shortcut_points."
                 % len(raw["astar"]))
    # And through the real front door: an empty grid must be a NO-OP, which is
    # the short-circuit the 744.0 mm m2 gate in test_stack.py rests on.
    r, _ = planned(start[0], start[1], hdg, 972.0, 972.0, M.OccupancyGrid())
    check("empty grid -> [] (straight line), and no search ran",
          r.points == [] and r.searched is False,
          "points=%r searched=%r" % (r.points, r.searched))


def test_03_wall_with_a_gap():
    """PROPERTY 2. The route goes THROUGH the gap, with few waypoints.

    The gap is deliberately OFF the straight line: a gap the rover is already
    pointing at is answered by the `segment_ok` short-circuit and proves
    nothing about the search. Here the wall runs from the left edge to x = 600
    at y = 600, so the only way past is round its right-hand end, and the
    geometry fixes the answer: the centre may cross y = 600 only where it is
    further than `reach` from the last wall point, and no further right than the
    wall pad allows.
    """
    print("\n3. wall with a gap (property 2)")
    wall = hwall(600.0, 0.0, 600.0)
    grid = grid_with(wall)
    reach = M.ROBOT_RADIUS_MM + M.OCC_POINT_PAD_MM
    gap_lo = 600.0 + reach                       # right of the wall's end
    gap_hi = M.ARENA_MM - M.WALL_PAD_MM          # left of the arena wall pad
    check("the gap this test relies on is real", gap_hi - gap_lo > M.GRID_MM,
          "passable band x in [%.0f, %.0f]" % (gap_lo, gap_hi))
    for mode in ("theta", "astar"):
        r, poly = planned(250.0, 250.0, 90.0, 250.0, 1000.0, grid, planner=mode)
        if poly is None:
            check("%s: finds a route round the wall" % mode, False, "got None")
            continue
        check("%s: needed a search (the straight line is blocked)" % mode,
              r.searched and len(r.points) > 0,
              "searched=%r waypoints=%d" % (r.searched, len(r.points)))
        crossings = [a[0] + (b[0] - a[0]) * ((600.0 - a[1]) / (b[1] - a[1]))
                     for a, b in zip(poly, poly[1:])
                     if (a[1] - 600.0) * (b[1] - 600.0) < 0]
        check("%s: crosses the wall line exactly once" % mode,
              len(crossings) == 1, "crossings=%s" % crossings)
        if crossings:
            x = crossings[0]
            check("%s: crosses INSIDE the gap" % mode, gap_lo <= x <= gap_hi,
                  "crossed at x=%.1f, gap is [%.0f, %.0f]" % (x, gap_lo, gap_hi))
        check("%s: waypoint count stays small" % mode,
              len(r.points) <= WP_MAX_ONE_OBSTACLE,
              "%d waypoints (bound %d)" % (len(r.points), WP_MAX_ONE_OBSTACLE))


def test_04_astar_is_optimal_on_known_grids():
    """PROPERTY 3a. `astar_cells` must find THE optimal 8-connected path.

    Unlike Theta*, this one has no excuse: octile is admissible and consistent,
    so the first pop of the goal is optimal and the answer is exactly the
    shortest path. Checked against `brute_force_grid_cost`, which shares no
    mechanism with it (see that function). Random grids rather than one hand-
    drawn case, because a hand-drawn case tests the grid the author imagined.
    """
    print("\n4. A* optimality against brute force (property 3a)")
    random.seed(20260813)                        # fixed: a flaky test is noise
    n, trials, worst, mismatches, unreachable = 8, 40, 0.0, 0, 0
    for _ in range(trials):
        blocked = set()
        while len(blocked) < 14:
            c = (random.randrange(n), random.randrange(n))
            if c not in ((0, 0), (n - 1, n - 1)):
                blocked.add(c)
        path = M.astar_cells(blocked, n, (0, 0), (n - 1, n - 1))
        opt = brute_force_grid_cost(blocked, n, (0, 0), (n - 1, n - 1))
        if path is None:
            if opt == float("inf"):
                unreachable += 1
            else:
                mismatches += 1              # refused a route that exists
            continue
        if opt == float("inf"):
            mismatches += 1                  # returned a route that cannot exist
            continue
        cost = sum(M.DIAG_COST if (b[0] - a[0] and b[1] - a[1]) else 1.0
                   for a, b in zip(path, path[1:]))
        worst = max(worst, abs(cost - opt))
    check("A* agrees with brute force on every random grid", mismatches == 0,
          "%d disagreements in %d grids" % (mismatches, trials))
    check("A* cost equals the true optimum exactly", worst < 1e-9,
          "worst |astar - bruteforce| = %.12f over %d grids (%d unreachable)"
          % (worst, trials, unreachable))
    # No corner cutting: slipping diagonally between two blocked cells is free
    # on a grid and impossible for a rover with width.
    blocked = {(1, 0), (0, 1)}
    p = M.astar_cells(blocked, 3, (0, 0), (1, 1))
    check("A* refuses to cut a blocked corner", p is None,
          "got %r" % (p,))


def test_05_theta_near_optimal_and_beats_the_grid():
    """PROPERTY 3b. Theta* must be near-optimal, and better than 45 deg steps.

    Compared against `optimal_cost`, an independently computed optimum over the
    SAME representation class (see that function). Two claims are being made:
    that any-angle search is close to the best route it could have chosen, and
    that it is meaningfully better than the grid-quantised `astar` it replaced
    -- because the second claim is the entire justification for `theta` being
    the default, and nothing else in the repo checks it.

    A coarse world (100 mm cells) rather than the live 50 mm one, because the
    oracle is O(V^2) in cells and 576 cells does not finish in a test.
    """
    print("\n5. Theta* near-optimality against an independent oracle (property 3b)")
    n, cell = 12, 100.0
    scenes = {
        "wall from the left": hwall(600.0, 0.0, 600.0),
        "pillar in the middle": ring(600.0, 600.0, r=25.0),
    }
    start, goal, hdg = (250.0, 250.0), (950.0, 950.0), 90.0
    for name, obs in scenes.items():
        cmap = costmap(obs, n=n, cell=cell)
        if not (cmap.safe(*start) and cmap.safe(*goal)):
            skip("oracle/%s" % name, "start or goal is not safe in this world")
            continue
        t0 = time.time()
        opt = optimal_cost(cmap, start, goal, hdg)
        secs = time.time() - t0
        if opt == float("inf"):
            check("%s: the oracle finds a route at all" % name, False)
            continue
        costs = {}
        for mode, frac in (("theta", THETA_OPT_FRAC), ("astar", ASTAR_OPT_FRAC)):
            route = M._search(cmap, start[0], start[1], hdg, goal[0], goal[1], mode)
            if route is None:
                check("%s: %s finds a route the oracle proved exists"
                      % (name, mode), False, "got None (optimal=%.1f)" % opt)
                continue
            final = M.prune_via(M.shortcut_points(route, cmap, hdg), cmap)
            costs[mode] = M.path_cost_mm(final, hdg, cmap)
            gap = (costs[mode] - opt) / opt
            check("%s: %s is within %.0f%% of optimal" % (name, mode, frac * 100),
                  gap <= frac,
                  "cost %.1f vs optimal %.1f = %+.2f%% (oracle %.1fs)"
                  % (costs[mode], opt, gap * 100.0, secs))
            check("%s: %s never beats the oracle (the oracle is a lower bound)"
                  % (name, mode), costs[mode] >= opt - 1e-6,
                  "cost %.3f vs optimal %.3f" % (costs[mode], opt))
        if "theta" in costs and "astar" in costs:
            check("%s: any-angle beats 45 deg steps, which is why theta is "
                  "the default" % name, costs["theta"] <= costs["astar"],
                  "theta %.1f vs astar %.1f" % (costs["theta"], costs["astar"]))


def test_05b_theta_vs_astar_at_the_live_resolution():
    """The same comparison as test_05, but on the grid the rover really runs.

    WHY THIS IS SEPARATE, AND WHY IT WARNS RATHER THAN FAILS. test_05 measures
    against a true optimum, which forces a coarse world (the oracle is O(V^2)
    in cells). This one runs the LIVE 24 x 24 / 50 mm grid with no oracle, and
    just asks the two settings to score each other -- because the finer the
    cells are relative to the 195 mm reach, the more the any-angle advantage
    shrinks, and at the live resolution it can invert.

    Measured here: `astar` produces a CHEAPER route on the planner's own cost
    function in 2 of 5 routable scenes, by up to 4.5 %, and with fewer
    waypoints. That is not a bug -- `theta_star`'s docstring is explicit that
    basic Theta* is not optimal and that the turn term makes cost
    history-dependent on top of that -- but it IS the opposite of what the
    default setting is justified by, so it is measured out loud rather than
    assumed away.

    The assertion is a bound, not a preference: whatever the default is, it must
    never be worse than the fallback by more than the fallback's own
    quantisation penalty (7.3-8.3 % over optimal, measured in test_05). Beyond
    that, `theta` has stopped being the better choice and the default is wrong.
    """
    print("\n5b. theta vs astar on the live 24 x 24 grid")
    scenes = {
        "wall from the left": (hwall(600.0, 0.0, 600.0), (250.0, 250.0), (250.0, 1000.0)),
        "wall from the right": (hwall(600.0, 600.0, 1200.0), (950.0, 250.0), (950.0, 1000.0)),
        "pillar": (ring(600.0, 600.0, r=30.0), (250.0, 250.0), (950.0, 950.0)),
        "two pillars": (ring(450.0, 600.0, r=25.0) + ring(850.0, 600.0, r=25.0),
                        (600.0, 250.0), (600.0, 1000.0)),
        "offset pillar": (ring(700.0, 700.0, r=35.0), (250.0, 250.0), (950.0, 950.0)),
    }
    BOUND = 0.08
    astar_wins, routable = 0, 0
    for name, (obs, s, g) in scenes.items():
        grid = grid_with(obs)
        got = {}
        for mode in ("theta", "astar"):
            r, _ = planned(s[0], s[1], 90.0, g[0], g[1], grid, planner=mode)
            got[mode] = r
        if got["theta"].points is None or got["astar"].points is None:
            skip("live/%s" % name, "at least one planner found no route")
            continue
        routable += 1
        ct, ca = got["theta"].cost_mm, got["astar"].cost_mm
        check("%s: theta is within %.0f%% of the astar fallback"
              % (name, BOUND * 100), ct <= ca * (1.0 + BOUND),
              "theta %.1f (%d wp) vs astar %.1f (%d wp) = %+.2f%%"
              % (ct, len(got["theta"].points), ca, len(got["astar"].points),
                 100.0 * (ct - ca) / ca))
        if ca < ct - 1e-6:
            astar_wins += 1
            warn("live/%s" % name,
                 "the `astar` fallback is CHEAPER than the `theta` default by "
                 "%.1f%% (%.1f vs %.1f) and uses %d waypoints instead of %d. "
                 "Theta* is documented as non-optimal, so this is expected in "
                 "kind; the size of it is worth knowing before anyone 'tunes' "
                 "the planner on the assumption theta always wins."
                 % (100.0 * (ct - ca) / ct, ca, ct,
                    len(got["astar"].points), len(got["theta"].points)))
    if routable:
        print("      astar was cheaper in %d of %d routable scenes"
              % (astar_wins, routable))


def test_06_clearance_keeps_the_rover_off_walls():
    """PROPERTY 4. No part of the route may be inside the pad. THE ONE THAT
    KEEPS THE ROVER OFF WALLS.

    `clear_mm` already subtracts the robot radius and the point pad, so it IS
    the margin: >= 0 means the centre may be there, negative means the chassis
    is inside a measured surface. Checked at the waypoints (the stated
    property) and then along the whole driven line at 2 mm, because a route
    whose vertices are legal can still graze between them -- and grazing is the
    failure this arena cannot afford: there are only 58 mm of pose-error budget
    at a zone centre.
    """
    print("\n6. clearance (property 4)")
    scenes = {
        "wall from the left": (hwall(600.0, 0.0, 600.0), (250.0, 250.0), (250.0, 1000.0)),
        "pillar": (ring(600.0, 600.0, r=30.0), (250.0, 250.0), (950.0, 950.0)),
        "two pillars": (ring(450.0, 600.0, r=25.0) + ring(850.0, 600.0, r=25.0),
                        (600.0, 250.0), (600.0, 1000.0)),
    }
    for name, (obs, s, g) in scenes.items():
        grid = grid_with(obs)
        for mode in ("theta", "astar"):
            r, poly = planned(s[0], s[1], 90.0, g[0], g[1], grid, planner=mode)
            if poly is None:
                skip("clearance/%s/%s" % (name, mode), "no route to measure")
                continue
            # The costmap the answer must be judged on is the one the rung that
            # produced it used -- judging a relaxed route at full margin would
            # fail it for being exactly what it says it is -- and it must be
            # built from the grid, not from the raw obstacle list (see `map_of`).
            wall_s, rad_s = [t[1:] for t in M.RELAX_LADDER if t[0] == r.relax][0]
            cmap = map_of(grid, free_at=s,
                          radius_mm=M.ROBOT_RADIUS_MM * rad_s,
                          wall_pad_mm=M.WALL_PAD_MM * wall_s)
            bad = [p for p in r.points if cmap.clear_mm(p[0], p[1]) < -CLEAR_EPS_MM]
            check("%s/%s: every waypoint is outside the pad" % (name, mode),
                  not bad, "inside: %s" % bad)
            margin, at = worst_clearance(cmap, poly)
            dip = sampling_dip_mm(cmap)
            check("%s/%s: the driven line stays inside the sampling bound"
                  % (name, mode), margin >= -dip,
                  "worst margin %.4f mm at (%.0f, %.0f); segment_ok's half-cell "
                  "sampling permits at most %.4f mm"
                  % (margin, at[0], at[1], dip) if at else "")
            if -dip <= margin < 1.0:
                # Legal, and worth saying out loud: a route that is legal by a
                # rounding error has spent the entire inflation budget before
                # the rover has moved, and the inflation radius IS the budget
                # for pose error on a rover with no localisation.
                warn("clearance/%s/%s" % (name, mode),
                     "route is drivable but GRAZES the inflation boundary "
                     "(margin %.4f mm at (%.0f, %.0f)). CLEARANCE_COST_W=%.2f "
                     "did not buy any daylight here, and the arena allows only "
                     "58 mm of pose error at a target."
                     % (margin, at[0], at[1], M.CLEARANCE_COST_W))
    # The pad must also be what stops a target being accepted at all: a goal
    # inside the wall pad is not reachable and must not be answered with a
    # route that ends inside it.
    cmap = costmap([])
    edge = M.WALL_PAD_MM - 1.0
    check("a goal inside the wall pad is refused, not approximated",
          M.theta_star(cmap, (600.0, 600.0), (edge, 600.0), 0.0) is None,
          "goal at x=%.0f, pad is %.0f" % (edge, M.WALL_PAD_MM))


def test_07_turn_cost_actually_costs():
    """PROPERTY 5. Fewer turns must beat shorter, at the configured weight.

    Two levels, because either alone is weak. The first is arithmetic: the
    break-even extra distance a turn is worth must be exactly `turn_cost_mm`,
    which proves the weight is applied and applied once. The second is
    behavioural: with a SYMMETRIC obstacle the only thing that can break the
    left/right tie is the start heading, and the start heading only reaches the
    search through the turn term -- so if setting TURN_COST_WEIGHT to 0 makes
    the heading stop mattering, the turn cost is genuinely steering the search
    and not merely decorating the answer.
    """
    print("\n7. turn cost (property 5)")
    turn90 = M.turn_cost_mm(math.pi / 2)
    straight = [(0.0, 0.0), (0.0, 1000.0)]
    check("a straight run costs its length and nothing else",
          abs(M.path_cost_mm(straight, 90.0) - 1000.0) < 1e-9,
          "%.3f mm" % M.path_cost_mm(straight, 90.0))
    # A dog-leg with two right angles, starting along the current heading so
    # the first vertex is free and only the two bends are charged.
    dog = [(0.0, 0.0), (0.0, 500.0), (500.0, 500.0), (500.0, 1000.0)]
    dog_len, dog_cost = length_mm(dog), M.path_cost_mm(dog, 90.0)
    extra = dog_cost - dog_len
    check("two right-angle turns cost exactly two turn charges",
          abs(extra - 2 * turn90) < 1e-6,
          "charged %.2f mm over %.0f mm of driving, 2 x turn_cost_mm(90) "
          "= %.2f mm" % (extra, dog_len, 2 * turn90))
    # BREAK-EVEN. A turn-free route may be exactly `2 * turn90` mm longer than
    # this two-turn route and still be preferred. One millimetre either side of
    # that pins the weight: too small and turns are being under-charged, too
    # large and the planner will refuse turns it should take.
    for delta, should_win in ((-1.0, True), (+1.0, False)):
        longer = [(0.0, 0.0), (0.0, dog_len + 2 * turn90 + delta)]
        wins = M.path_cost_mm(longer, 90.0) < dog_cost
        check("a turn-free route %.0f mm %s break-even %s the two-turn route"
              % (abs(delta), "under" if delta < 0 else "over",
                 "beats" if should_win else "loses to"),
              wins == should_win,
              "straight %.2f vs dog-leg %.2f (break-even at %.2f mm of length)"
              % (M.path_cost_mm(longer, 90.0), dog_cost, dog_len + 2 * turn90))
    # Behavioural: symmetric pillar, asymmetric heading.
    grid = grid_with(ring(600.0, 600.0, r=40.0))
    original = M.TURN_COST_WEIGHT
    try:
        sides = {}
        for w in (original, 0.0):
            M.TURN_COST_WEIGHT = w
            picked = {}
            for hdg in (140.0, 40.0):
                r, poly = planned(600.0, 250.0, hdg, 600.0, 1000.0, grid)
                if not r.points:
                    picked[hdg] = None
                    continue
                mid = sum(p[0] for p in r.points) / float(len(r.points))
                picked[hdg] = "left" if mid < 600.0 else "right"
            sides[w] = picked
        check("at the configured weight the search passes on the side it is "
              "already pointing at",
              sides[original][140.0] == "left" and sides[original][40.0] == "right",
              "heading 140 -> %s, heading 40 -> %s"
              % (sides[original][140.0], sides[original][40.0]))
        check("with the turn weight at zero the start heading stops mattering",
              sides[0.0][140.0] == sides[0.0][40.0],
              "heading 140 -> %s, heading 40 -> %s"
              % (sides[0.0][140.0], sides[0.0][40.0]))
        # ... and the zero-weight route should be the shorter, turnier one.
        M.TURN_COST_WEIGHT = 0.0
        r0, p0 = planned(600.0, 250.0, 90.0, 600.0, 1000.0, grid)
        M.TURN_COST_WEIGHT = original
        r1, p1 = planned(600.0, 250.0, 90.0, 600.0, 1000.0, grid)
        if p0 and p1:
            check("charging for turns buys a straighter route than not charging",
                  total_turn_deg(p1, 90.0) <= total_turn_deg(p0, 90.0) + 1e-6,
                  "w=%.1f: %.1f deg over %.0f mm | w=0: %.1f deg over %.0f mm"
                  % (original, total_turn_deg(p1, 90.0), length_mm(p1),
                     total_turn_deg(p0, 90.0), length_mm(p0)))
    finally:
        M.TURN_COST_WEIGHT = original


def test_08_relax_ladder_degrades_in_order():
    """PROPERTY 6. The ladder relaxes in the documented sequence, and says so.

    The scenes are built so the answer is forced by arithmetic, not by taste.
    The wall stub runs from the left edge to x = XMAX at y = 600; the rover must
    pass to its right, which needs a FREE CELL CENTRE further right than
    XMAX + reach and no further right than the wall pad allows. Moving XMAX
    alone therefore steps the required rung, one rung at a time:

        stub short          -> "none"          (full margin is enough)
        stub longer         -> "wall"          (only the halved wall pad fits)
        stub longer still   -> "wall+radius"   (the chassis radius must give too)
        stub to the wall    -> None            (the ladder cannot reach zero)

    The last one is the property that matters most: there is no rung that
    removes padding, so "no route at any margin" stays an honest refusal rather
    than becoming "a route if the chassis is allowed to collide".
    """
    print("\n8. the fallback ladder (property 6)")
    check("the ladder is exactly the three documented rungs, in order",
          [t[0] for t in M.RELAX_LADDER] == ["none", "wall", "wall+radius"],
          "%r" % (M.RELAX_LADDER,))
    check("the wall pad is relaxed before the chassis radius",
          M.RELAX_LADDER[1][1] < 1.0 and M.RELAX_LADDER[1][2] == 1.0,
          "rung 2 = %r" % (M.RELAX_LADDER[1],))
    check("the chassis radius is relaxed last and least",
          M.RELAX_LADDER[2][2] >= M.RELAX_RADIUS_FRAC > 0.5,
          "radius scale %.2f, floor %.2f"
          % (M.RELAX_LADDER[2][2], M.RELAX_RADIUS_FRAC))
    check("no rung removes padding entirely",
          all(w > 0.0 and r > 0.0 for _, w, r in M.RELAX_LADDER))

    expect = [(700.0, "none"), (880.0, "wall"), (900.0, "wall+radius"),
              (1000.0, None)]
    seen = []
    for xmax, want in expect:
        grid = grid_with(hwall(600.0, 0.0, xmax, step=20.0))
        r, _ = planned(1000.0, 300.0, 90.0, 1000.0, 900.0, grid)
        got = None if r.points is None else r.relax
        seen.append(got)
        check("stub to x=%.0f uses rung %r" % (xmax, want), got == want,
              "got %r (points=%s)"
              % (got, "None" if r.points is None else len(r.points)))
        if r.points is not None and r.relax != "none":
            # Every rung is REPORTED, never silent. A path that only exists
            # because a margin was cut is not the same answer as one that
            # exists at full margin, and drawing it silently would tell the
            # operator the route is ordinary when it is the one case they most
            # need to be told about.
            check("rung %r explains itself in the note" % r.relax,
                  bool(r.note) and r.relax.split("+")[0] in (r.note or ""),
                  "note=%r" % (r.note,))
    check("the ladder degrades one rung at a time, never jumping to the loosest",
          seen == ["none", "wall", "wall+radius", None],
          "sequence %r" % (seen,))

    # A relaxed rung that returns a DIRECT route must still emit a note --
    # otherwise the quiet case and the one case the operator must be told about
    # look identical on the dashboard.
    grid = grid_with(hwall(600.0, 0.0, 700.0, step=20.0))
    r, _ = planned(100.0, 300.0, 90.0, 100.0, 900.0, grid)
    if r.points is not None and r.relax != "none" and not r.points:
        segs, note = M.plan_grid_route(100.0, 300.0, 90.0, 100.0, 900.0,
                                       grid=grid, now=NOW)
        check("a relaxed rung with no waypoints still publishes a note",
              bool(note), "note=%r" % (note,))

    # With relaxation switched off the ladder must not exist at all.
    original = M.PLAN_RELAX
    try:
        M.PLAN_RELAX = False
        grid = grid_with(hwall(600.0, 0.0, 880.0, step=20.0))
        r, _ = planned(1000.0, 300.0, 90.0, 1000.0, 900.0, grid)
        check("PLAN_RELAX off -> refuse rather than silently relax",
              r.points is None and r.relax == "none",
              "points=%s relax=%r"
              % ("None" if r.points is None else len(r.points), r.relax))
        segs, note = M.plan_grid_route(1000.0, 300.0, 90.0, 1000.0, 900.0,
                                       grid=grid, now=NOW)
        check("and the note says relaxation was switched off",
              note is not None and "relaxation is switched off" in note,
              "note=%r" % ((note or "")[:90],))
    finally:
        M.PLAN_RELAX = original


def test_09_unreachable_refuses_cleanly():
    """PROPERTY 7. No route means None, with a reason -- never a partial path.

    `grid_detour_points` has a three-answer contract: [] clear, [...] detour,
    None no route. Returning a partial path as though it were complete is the
    one failure that would drive the rover confidently at a wall, because the
    follower has no way to tell a short route from a truncated one.
    """
    print("\n9. unreachable goal (property 7)")
    box = []
    for t in [i * 20.0 for i in range(0, 26)]:
        box += [(400.0 + t, 400.0), (400.0 + t, 900.0),
                (400.0, 400.0 + t), (900.0, 400.0 + t)]
    grid = grid_with(box)
    for mode in ("theta", "astar"):
        r, _ = planned(250.0, 250.0, 45.0, 650.0, 650.0, grid, planner=mode)
        check("%s: a walled-in goal returns None, not a short path" % mode,
              r.points is None, "points=%r" % (r.points,))
        check("%s: it tried before refusing" % mode, r.searched,
              "searched=%r" % r.searched)
        segs, note = M.plan_grid_route(250.0, 250.0, 45.0, 650.0, 650.0,
                                       grid=grid, now=NOW, planner=mode)
        check("%s: the refusal names the planner and the target" % mode,
              note is not None and mode in note and "no clear route" in note,
              "note=%r" % ((note or "")[:100],))
        # The straight line is still drawn, on purpose: refusing to publish
        # would blank the operator's map at the moment it matters. What must
        # never happen is publishing it as though it were a plan.
        check("%s: a failed search still draws something for the operator" % mode,
              bool(segs), "%d segments" % len(segs))
    check("grid_detour_points passes the None through unchanged",
          M.grid_detour_points(250.0, 250.0, 650.0, 650.0, grid, now=NOW,
                               heading_deg=45.0) is None)


def test_10_zones_reachable_and_the_58mm_is_real():
    """PROPERTY 8. All three zone centres clear full padding, by exactly 58 mm.

    `docs/PLANNING.md` and `overlay/etc/fpms/zones.json` both claim 58.0 mm of
    centre-margin at every zone. That number is quoted in four places and
    measured in none, so this re-derives it from the code's real constants:
    the nearest wall is `min(cx, cy, arena-cx, arena-cy)` = 228 mm and the pad
    is ROBOT_RADIUS_MM = 170 mm.

    It is the entire pose-error budget at a target. If a rescale, a re-measured
    chassis or a changed pad ever makes it smaller, three documents keep saying
    58 and nothing else notices.
    """
    print("\n10. zone reachability and the 58 mm claim (property 8)")
    cmap = costmap([])
    claimed = 58.0
    targets = [("zone-a", M.zone_center("zone-a")),
               ("zone-b", M.zone_center("zone-b")),
               ("water-station", M.zone_center("water-station")),
               ("start pose", (M.ROVER_START["x_mm"], M.ROVER_START["y_mm"]))]
    for name, (cx, cy) in targets:
        margin = cmap.clear_mm(cx, cy)
        check("%s is reachable at FULL wall padding" % name, cmap.safe(cx, cy),
              "centre (%.0f, %.0f), margin %.2f mm" % (cx, cy, margin))
        nearest = min(cx, cy, M.ARENA_MM - cx, M.ARENA_MM - cy)
        check("%s margin equals nearest-wall minus robot radius" % name,
              abs(margin - (nearest - M.ROBOT_RADIUS_MM)) < 1e-9,
              "%.2f = %.0f - %.0f" % (margin, nearest, M.ROBOT_RADIUS_MM))
        check("%s margin is the 58.0 mm the docs claim" % name,
              abs(margin - claimed) < STRAIGHT_TOL_MM,
              "got %.2f mm, docs say %.1f mm" % (margin, claimed))
    # And a full route to each, on a clean arena, needing no rung of the ladder.
    sx, sy = M.ROVER_START["x_mm"], M.ROVER_START["y_mm"]
    grid = grid_with(ring(600.0, 600.0, r=20.0))   # something believed, off-route
    for name in ("zone-a", "zone-b", "water-station"):
        tx, ty = M.zone_center(name)
        r, poly = planned(sx, sy, M.ROVER_START["heading_deg"], tx, ty, grid)
        check("%s: routable with an obstacle on the map, at full margin" % name,
              r.points is not None and r.relax == "none",
              "relax=%r points=%s"
              % (r.relax, "None" if r.points is None else len(r.points)))


def test_11_occupancy_edge_cases():
    """Noise and occupancy edge cases. Each is a real thing a LiDAR does.

    The two that must never be answered with a route are a goal inside an
    obstacle and a fully occupied grid; the two that must never be answered
    with a DETOUR are a grid that believes nothing and a grid that has gone
    stale, because inventing a route round nothing drives the rover somewhere
    arbitrary.
    """
    print("\n11. occupancy edge cases")
    tx, ty = M.zone_center("zone-b")
    sx, sy = M.ROVER_START["x_mm"], M.ROVER_START["y_mm"]

    for mode in ("theta", "astar"):
        r, _ = planned(sx, sy, 90.0, tx, ty, grid_with([(tx, ty)]), planner=mode)
        check("%s: a goal cell that is itself occupied is refused" % mode,
              r.points is None, "points=%r" % (r.points,))
    check("astar_cells answers a blocked goal directly, without searching",
          M.astar_cells({(5, 5)}, 10, (0, 0), (5, 5)) is None)

    everything = [(ix * M.GRID_MM + M.GRID_MM / 2.0,
                   iy * M.GRID_MM + M.GRID_MM / 2.0)
                  for ix in range(24) for iy in range(24)]
    grid = grid_with(everything)
    for mode in ("theta", "astar"):
        r, _ = planned(sx, sy, 90.0, tx, ty, grid, planner=mode)
        check("%s: an all-occupied grid refuses at every rung" % mode,
              r.points is None, "points=%r relax=%r" % (r.points, r.relax))

    # A rover standing inside the wall pad must still be able to plan the route
    # OUT of the trouble it is in -- that is what the `free` escape hatch on the
    # start cell exists for, and it is only reachable through a relaxed rung.
    grid = grid_with(ring(600.0, 600.0, r=30.0))
    inside = M.WALL_PAD_MM - 70.0
    r, _ = planned(inside, 600.0, 45.0, 972.0, 972.0, grid)
    check("a rover parked inside the wall pad can still plan its way out",
          r.points is not None,
          "parked at x=%.0f (pad %.0f), relax=%r" % (inside, M.WALL_PAD_MM, r.relax))

    # A start with an obstacle close enough to swallow it. The chassis radius is
    # 170 mm, so a surface nearer than that is already touching -- refusing is
    # correct there. What is measured here is where refusal STOPS, because that
    # number has to sit below the distance the obstacle guard stops at.
    stand_off = None
    for d in range(120, 321, 5):
        g = grid_with([(600.0, 600.0 + d)])
        r, _ = planned(600.0, 600.0, 90.0, 600.0, 1000.0, g)
        if r.points is not None and r.relax == "none":
            stand_off = d
            break
    check("a route exists at full margin once an obstacle is a reach away",
          stand_off is not None and stand_off <= M.ROBOT_RADIUS_MM + M.OCC_POINT_PAD_MM,
          "first full-margin route at %s mm; reach is %.0f mm"
          % (stand_off, M.ROBOT_RADIUS_MM + M.OCC_POINT_PAD_MM))

    # Nothing believed -> no detour, three different ways.
    r, _ = planned(sx, sy, 90.0, tx, ty, M.OccupancyGrid())
    check("an empty grid is a no-op", r.points == [] and not r.searched)
    g = grid_with([(600.0, 600.0)], now=NOW)
    r = M.plan_detour(sx, sy, tx, ty, g, now=NOW + M.OCC_TTL_S + 1.0,
                      heading_deg=90.0)
    check("a stale grid is a no-op, not a wall", r.points == [] and not r.searched,
          "TTL %.1f s" % M.OCC_TTL_S)
    g = grid_with([(600.0, 600.0)], hits=1)
    check("one return is noise, not a surface (OCC_MIN_HITS=%d)" % M.OCC_MIN_HITS,
          M.plan_detour(sx, sy, tx, ty, g, now=NOW, heading_deg=90.0).points == [])
    # Zeros and non-finite bins mark nothing: a zero written in would put a wall
    # underneath the rover and the search would refuse to move at all.
    g = M.OccupancyGrid()
    marked = g.integrate([0.0] * 360, (600.0, 600.0, 0.0), now=NOW)
    check("a scan of zeros marks nothing", marked == 0 and not g.occupied(now=NOW),
          "marked %d" % marked)
    g = M.OccupancyGrid()
    marked = g.integrate([float("inf"), float("nan")] * 180,
                         (600.0, 600.0, 0.0), now=NOW)
    check("non-finite bins mark nothing", marked == 0, "marked %d" % marked)


def test_12_determinism():
    """The same arena must plan the same route, every time.

    Half the determinism guarantee is a fixed neighbour order and half is a heap
    key of (f, h, ix, iy) that depends only on WHICH cell. If either slips, two
    detours round opposite sides of one obstacle differ by a millimetre of cost
    and the rover takes the left one, then the right one, then the left one --
    spending a turn each time it changes its mind and making no progress.
    """
    print("\n12. determinism")
    obs = ring(600.0, 600.0, r=35.0) + hwall(400.0, 700.0, 1000.0)
    for mode in ("theta", "astar"):
        answers = set()
        for _ in range(8):
            grid = grid_with(obs)               # rebuilt each time: fresh dict order
            r, _ = planned(250.0, 250.0, 90.0, 950.0, 950.0, grid, planner=mode)
            answers.add(tuple(r.points) if r.points is not None else None)
        check("%s: 8 identical inputs give 1 answer" % mode, len(answers) == 1,
              "%d distinct answers" % len(answers))


def test_13_never_publishes_an_undrivable_route():
    """FUZZ. A route reported at full margin must be drivable at full margin.

    `plan_detour` promises a path is only returned after `segment_ok` has
    confirmed every leg on the costmap that produced it. This checks the promise
    from outside, on scenes nobody chose: the invariant is

        relax == "none"  =>  every leg clears the FULL pad

    and separately that the answer is only ever one of the three contract
    values. A relaxed route is expected to fail a full-margin check -- that is
    what the rung means, and counting them keeps this test honest about how
    often the ladder is being used.
    """
    print("\n13. fuzz: no undrivable route is ever published")
    random.seed(20260813)
    trials = 120
    ran = none_ct = relaxed = unsafe_at_full = bad_contract = 0
    for _ in range(trials):
        obs = []
        for _ in range(random.randrange(1, 7)):
            obs += ring(random.uniform(250.0, 950.0),
                        random.uniform(250.0, 950.0), r=30.0, k=12)
        grid = grid_with(obs)
        sx, sy = random.choice([(972.0, 228.0), (228.0, 228.0), (600.0, 250.0)])
        tx, ty = random.choice([(972.0, 972.0), (228.0, 972.0), (228.0, 228.0)])
        if (sx, sy) == (tx, ty):
            continue
        hdg = random.choice([0.0, 45.0, 90.0, 180.0, 270.0])
        mode = random.choice(["theta", "astar"])
        r, poly = planned(sx, sy, hdg, tx, ty, grid, planner=mode)
        ran += 1
        if r.points is None:
            none_ct += 1
            continue
        if not isinstance(r.points, list):
            bad_contract += 1
            continue
        if r.relax != "none":
            relaxed += 1
            continue
        if not drivable(map_of(grid, free_at=(sx, sy)), poly):
            unsafe_at_full += 1
    check("no route reported at full margin is undrivable at full margin",
          unsafe_at_full == 0,
          "%d of %d trials (%d refused, %d used a relaxed rung)"
          % (unsafe_at_full, ran, none_ct, relaxed))
    check("every answer is one of the three contract values (None / [] / [...])",
          bad_contract == 0, "%d violations" % bad_contract)
    if none_ct > ran * 0.5:
        warn("fuzz", "%d of %d random scenes had NO route at any rung. Each one "
                     "is an ABORT_OBSTACLE on the field, so this ratio is worth "
                     "watching if the obstacle density here is realistic."
                     % (none_ct, ran))


def test_14_hysteresis_is_sticky_but_not_stubborn():
    """The committed path is kept unless it is unsafe or clearly beaten.

    Three cases, and the middle one is the safety argument: eligibility is
    re-checked against the map AS IT IS NOW, so the common case at a real
    obstacle event -- where the committed route runs through the thing that just
    stopped the rover -- cannot keep itself alive.
    """
    print("\n14. hysteresis")
    obs = hwall(600.0, 0.0, 600.0)
    grid = grid_with(obs)
    fresh, _ = planned(250.0, 200.0, 90.0, 250.0, 1000.0, grid)
    if fresh.points is None:
        skip("hysteresis", "the reference scene has no route to commit to")
        return
    again, _ = planned(250.0, 200.0, 90.0, 250.0, 1000.0, grid,
                       committed=fresh.points)
    check("an unchanged committed path is kept, not re-decided",
          again.kept and again.points == fresh.points,
          "kept=%r" % again.kept)
    check("and the reason is reported, not silent",
          bool(again.note) and "committed" in (again.note or ""),
          "note=%r" % ((again.note or "")[:70],))
    # A committed path that now runs through the obstacle must lose.
    through = [(250.0, 600.0)]
    b, _ = planned(250.0, 200.0, 90.0, 250.0, 1000.0, grid, committed=through)
    check("a committed path that has become unsafe is dropped",
          not b.kept and b.points == fresh.points, "kept=%r" % b.kept)
    # A committed path that is drivable but far worse must also lose.
    worse = [(950.0, 300.0), (950.0, 900.0)]
    w, _ = planned(250.0, 200.0, 90.0, 250.0, 1000.0, grid, committed=worse)
    check("a committed path far worse than the fresh one is dropped",
          not w.kept, "kept=%r cost=%.1f (fresh %.1f)"
          % (w.kept, w.cost_mm, fresh.cost_mm))
    check("the hysteresis margin is a fraction, not a magic distance",
          0.0 <= M.PLAN_HYSTERESIS_FRAC < 1.0,
          "%.2f" % M.PLAN_HYSTERESIS_FRAC)


def test_15_guard_and_planner_agree_on_standoff():
    """The distance the guard STOPS at must exceed the distance the planner
    can PLAN OUT of. If it does not, every reroute becomes an abort.

    `_reroute` is only reached because the obstacle cone guard fired, which
    happens at FRONT_STOP_MM. From that pose the planner must be able to find a
    route, and it cannot until the nearest surface is roughly
    ROBOT_RADIUS_MM + OCC_POINT_PAD_MM away -- below that every neighbour of the
    rover's own cell is inside the inflation and the search has nowhere to
    expand to. The `free=(here,)` escape hatch frees ONE cell, and one cell is
    not an inflation radius.

    Nothing in the code couples these two numbers, so this is where they are
    compared.
    """
    print("\n15. obstacle guard vs the planner's minimum standoff")
    reach = M.ROBOT_RADIUS_MM + M.OCC_POINT_PAD_MM
    shipped = None
    if os.path.exists(SHIPPED_CFG):
        cfg = M.load_config(SHIPPED_CFG)
        try:
            shipped = float(cfg["FPMS_MISSION_FRONT_STOP_MM"])
        except (KeyError, ValueError):
            shipped = None
    if shipped is None:
        skip("standoff", "overlay config.env has no FPMS_MISSION_FRONT_STOP_MM")
    else:
        check("the SHIPPED FRONT_STOP_MM stops further out than the planner "
              "needs to replan", shipped > reach,
              "config.env %.0f mm vs reach %.0f mm" % (shipped, reach))
    if M.FRONT_STOP_MM <= reach:
        warn("standoff",
             "the FRONT_STOP_MM this module actually loaded is %.0f mm, which "
             "is INSIDE the planner's %.0f mm minimum standoff. From a pose "
             "that close, every neighbour of the rover's own cell is inflated "
             "and the search returns None -- so every ObstacleDetour degrades "
             "to ABORT_OBSTACLE. The shipped config.env sets %s, which is "
             "fine; the DEFAULT is not, and _cfg_float permits values down to "
             "60 mm with nothing to object."
             % (M.FRONT_STOP_MM, reach,
                "%.0f mm" % shipped if shipped else "a safe value"))


def test_16_docs_arithmetic():
    """Documentation checks, labelled as such: PLANNING.md's numbers must
    follow from the code and the shipped config, not from memory.

    These do not test the planner. They test that the three documents an
    operator will actually read still describe it.
    """
    print("\n16. docs vs code arithmetic")
    if not os.path.exists(SHIPPED_CFG):
        skip("docs", "overlay/etc/fpms/config.env not found")
        return
    cfg = M.load_config(SHIPPED_CFG)
    try:
        cruise = float(cfg["FPMS_MISSION_CRUISE_MPS"])
        leg = float(cfg["FPMS_MISSION_MAX_LEG_MM"])
    except (KeyError, ValueError) as exc:
        skip("docs", "config.env lacks the speed constants (%r)" % (exc,))
        return
    # The module's OWN derivation, re-run on the shipped numbers.
    eff = leg / (leg / 1000.0 / cruise + M.STOP_SETTLE_S)
    turn90 = M.TURN_COST_WEIGHT * ((math.pi / 2) * (eff / M.TURN_RADPS)
                                   + M.TURN_SETTLE_S * eff)
    check("PLANNING.md's 391 mm per 90 deg turn follows from config.env",
          abs(turn90 - 391.0) < 1.0,
          "derived %.1f mm from cruise=%.2f m/s, leg=%.0f mm (effective "
          "%.1f mm/s)" % (turn90, cruise, leg, eff))
    if abs(M.turn_cost_mm(math.pi / 2) - turn90) > 1.0:
        warn("docs", "this run used turn_cost_mm(90 deg) = %.1f mm, not the "
                     "shipped %.1f mm, because /etc/fpms/config.env is absent "
                     "here. Every expectation in this file is derived from the "
                     "loaded constants, so that is correct -- but a number "
                     "quoted from a laptop run is not the rover's number."
                     % (M.turn_cost_mm(math.pi / 2), turn90))
    if os.path.exists(PLANNING_MD):
        body = open(PLANNING_MD, encoding="utf-8").read()
        check("PLANNING.md still documents the ladder this code implements",
              all(r in body for r, _, _ in M.RELAX_LADDER),
              "missing: %s" % [r for r, _, _ in M.RELAX_LADDER if r not in body])
        check("PLANNING.md's 58 mm centre-margin claim is still in the doc",
              "58" in body)
    else:
        skip("docs", "docs/PLANNING.md not found")

    # Legacy helpers: `inflate_blocked` and `simplify_cells` have no call site
    # anywhere in the repo -- the search runs on `CostMap.blocked`. They are
    # kept exercised here so that if anybody reaches for them again they are
    # not silently broken, and flagged so nobody trusts the docstring that says
    # the search still uses them.
    # reach = radius + cell/2 = 85 mm, so with 50 mm cells exactly the 3x3
    # block around the obstacle is grown: (5,6) is 50 mm away and inside,
    # (5,7) is 100 mm away and outside.
    n = 10
    blocked = M.inflate_blocked({(5, 5)}, n, 50.0, radius_mm=60.0,
                                wall_pad_mm=0.0, arena_mm=500.0)
    check("inflate_blocked still grows an obstacle by radius + half a cell",
          (5, 5) in blocked and (5, 6) in blocked and (5, 7) not in blocked
          and len(blocked) == 9,
          "%d cells blocked: %s" % (len(blocked), sorted(blocked)))
    check("inflate_blocked's `free` escape hatch still clears a cell",
          (5, 5) not in M.inflate_blocked({(5, 5)}, n, 50.0, radius_mm=60.0,
                                          wall_pad_mm=0.0, arena_mm=500.0,
                                          free=((5, 5),)))
    warn("dead code",
         "inflate_blocked() and simplify_cells() have NO call site in the "
         "repository; the search runs on CostMap.blocked. CostMap's docstring "
         "(fpms_missions.py:2286-2288) says of inflate_blocked 'That is fine "
         "for a search, and it is still what the search uses', which is no "
         "longer true.")


# ==========================================================================
def main():
    global M
    print("=" * 72)
    print(" FPMS mission-planner unit checks (offline)")
    print("=" * 72)
    M = load_planner()
    if M is None:
        print("-" * 72)
        print("  SKIPPED: the planner could not be imported. This is NOT a pass.")
        print("=" * 72)
        return 2

    print("  module: %s" % MISSIONS)
    print("  planner=%s  arena=%.0f mm  cell=%.0f mm  radius=%.0f mm  "
          "wall pad=%.0f mm" % (M.PLANNER, M.ARENA_MM, M.GRID_MM,
                                M.ROBOT_RADIUS_MM, M.WALL_PAD_MM))
    print("  turn_cost_mm(90 deg)=%.1f mm  clearance w=%.2f  hysteresis=%.2f"
          % (M.turn_cost_mm(math.pi / 2), M.CLEARANCE_COST_W,
             M.PLAN_HYSTERESIS_FRAC))
    if M.CFG_NOTES:
        for note in M.CFG_NOTES:
            # The module's notes contain en/em dashes; a Windows console in a
            # legacy code page raises on them, and a test harness must not die
            # of its own banner.
            print("  cfg: %s" % note.encode("ascii", "replace").decode("ascii"))

    tests = [v for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v)]
    t0 = time.time()
    for t in tests:
        try:
            t()
        except Exception as exc:                                  # noqa: BLE001
            FAILURES.append(t.__name__)
            print("  [FAIL] %s: the test itself raised: %r" % (t.__name__, exc))

    print("\n" + "-" * 72)
    print("  %d test groups, %d assertions in %.1f s"
          % (len(tests), len(PASSES) + len(FAILURES), time.time() - t0))
    if WARNINGS:
        print("  %d warning(s) -- read them, they are findings not noise"
              % len(WARNINGS))
    if SKIPS:
        print("  %d skipped" % len(SKIPS))
    if FAILURES:
        print("  %d PASSED, %d FAILED" % (len(PASSES), len(FAILURES)))
        for name in FAILURES:
            print("    FAILED: %s" % name)
    else:
        print("  all %d assertions passed" % len(PASSES))
    print("=" * 72)
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())

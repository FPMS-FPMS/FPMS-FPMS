#!/usr/bin/env python3
"""FPMS Rover 2 route planner -- A*, Theta*, D* Lite, and the learned prior.

SELF-CONTAINED. STANDARD LIBRARY ONLY. NO ROS, NO MQTT, NO NUMPY, NO HARDWARE.
Import it on a laptop, unit-test it on a laptop. Nothing in here opens a port,
a serial device or a topic, and nothing in here commands motion: it answers the
question "which way" and hands back a list of turns and distances.

WHERE EVERY PIECE CAME FROM
===========================
Two files were read before a line of this was written, and both are named at the
functions that carry their behaviour:

  * `/home/ubuntu/fpms_phase6_M1_M2_WORKING.py`  ("golden", the B6/B8B code that
    actually completed missions). Its LIVE planner is lines 525-775:
        xy_cell / cell_xy / grid_size      the robot-relative 100 mm grid
        planning_obstacles                 self-filter, arena clip, target exclude
        build_costmap                      flat ROUTE_RADIUS=175 mm disc inflation
        nearest_free                       ring search for a free start/goal cell
        astar                              8-connected, Euclidean h, edge penalty
        route_goal                         stop short of the marker by TARGET_STOP_RAW
        simplify_path                      Douglas-Peucker, epsilon 150 mm
        plan_route                         straight-if-clear, else A* + DP
    and its segment builder is `_p5_navdrive` 1403-1412, which is where
    `if sd < 30: continue` -- the minimum-leg drop -- lives.
    Everything in golden after `app.run` (~2395) is DEAD CODE and was not read.
    `GoldenB6` below is a faithful, parameterised port of that live block, and
    `fpms_planner_test.py` proves it against a verbatim copy.

  * `/home/ubuntu/fpms_missions.py` (read-only; another agent owns it). Its
    planner is `OccupancyGrid` (2253), `inflate_blocked` (2510), `CostMap`
    (2560), `astar_cells` (2795), `theta_star` (2913), `prune_via` (3121),
    `shortcut_points` (3170), `plan_detour` (3335), `plan_grid_route` (3479).
    This module reproduces that behaviour in the arena frame so it can replace
    it, and adds the three things it does not have: D* Lite, a rectangular
    arena, and the learned prior.

WHAT IS NEW HERE, AND WHY
=========================
  1. THE ARENA IS RECTANGULAR. 1500 x 1200 mm as of 2026-08-14 (the y axis was
     shortened 200 mm). `fpms_missions.py` carries a single scalar `ARENA_MM =
     max(W, H)` and a square n x n grid, which puts 300 mm of phantom floor
     along +y. Nothing here is square and nothing here hardcodes 1400.
  2. D* LITE. True incremental repair from the rover's CURRENT position when the
     grid changes mid-leg, instead of throwing the search away and starting
     again. See `DStarLite`.
  3. REAL-FOOTPRINT INFLATION. A hard forbidden radius of robot_radius (150 mm)
     plus the sensor pad, and a SOFT cost that decays out to inflation_mm
     (200 mm) -- Nav2's layered idea. Golden used one flat 205 mm disc, which is
     both too generous close in and completely blind further out
     (AGENT_DESIGN.md, "Genuine upgrades over B8B", item 2).
  4. THE LEARNED PRIOR. `/var/lib/fpms/learned_grid.json` may ADD cost. It may
     never block, and it may never be required. See `LearnedPrior`.

THREE FILTERS THAT ARE NOT NEGOTIABLE (they are what made docking work)
=======================================================================
  TARGET_EXCLUDE_MM = 220  returns within 220 mm of the leg's OWN target are not
                           obstacles. Without this the rover treats the thing it
                           is docking against as a thing to avoid, the goal cell
                           goes occupied, and the planner refuses to route at
                           all. Golden `planning_obstacles` P5_TARGET_EXCLUDE_MM.
  SELF_FILTER_MM   = 140   returns closer than this are the rover's own bodywork
                           in the beam. Golden P5_ROBOT_FOOTPRINT_MM.
  ARENA CLIP               returns outside the arena box are room clutter and are
                           discarded. Golden's `wx < -20 or wx > 670 ...`.

DETERMINISM IS A REQUIREMENT, NOT A PROPERTY
============================================
Same inputs, same route, byte for byte, every time. There is no randomness here,
no `time.time()` in any decision, no dependence on dict insertion order, and
every priority-queue key ends in the cell index so ties break on WHICH cell and
never on when it was pushed. The one clock reading in the file is the optional
staleness check on the learned prior, and it is OFF by default for this reason.

The planner is also not allowed to be a source of unplanned turning:
  * routes are simplified (Douglas-Peucker, then line-of-sight shortcutting), so
    needless waypoints -- and therefore needless turns -- are gone;
  * every emitted segment carries its EXPLICIT planned rotation, and a straight
    run carries exactly 0.0, because the executor is given a rotation budget
    that aborts on unplanned turning;
  * any leg under MIN_LEG_MM (30 mm, golden's number) is merged or dropped, as a
    motion the chassis cannot express is an oscillation source;
  * a route that leaves the arena, or that doubles back through the start box,
    is refused rather than published.
"""

import heapq
import json
import math
import os
import threading
import time
from dataclasses import dataclass, field

__all__ = [
    "ARENA_W_MM", "ARENA_H_MM", "CELL_MM",
    "TARGET_EXCLUDE_MM", "SELF_FILTER_MM",
    "ROBOT_RADIUS_MM", "INFLATION_MM", "MIN_LEG_MM",
    "LearnedPrior", "load_learned_prior",
    "OccupancyGrid", "CostMap",
    "astar_cells", "theta_star", "DStarLite",
    "douglas_peucker", "shortcut_points", "prune_via",
    "Segment", "Plan", "Planner", "IncrementalPlanner",
    "GoldenB6",
]

# ============================================================= THE ARENA
# Operator-measured 2026-08-14: the arena was shortened 200 mm on the y axis.
# These are DEFAULTS. Every class takes arena_w_mm / arena_h_mm, nothing in this
# file assumes a square, and the number 1400 does not appear anywhere.
ARENA_W_MM = 1500.0          # x extent, left-right
ARENA_H_MM = 1200.0          # y extent, near-far
CELL_MM = 50.0               # search resolution; configurable everywhere

# ============================================================= THE FILTERS
TARGET_EXCLUDE_MM = 220.0    # golden P5_TARGET_EXCLUDE_MM -- docking depends on it
SELF_FILTER_MM = 140.0       # golden P5_ROBOT_FOOTPRINT_MM -- the chassis in the beam
ARENA_CLIP_MARGIN_MM = 20.0  # golden clipped at -20 .. +20 outside the box

# ============================================================= THE FOOTPRINT
# AGENT_DESIGN.md item 2: real-footprint inflation instead of golden's flat disc.
ROBOT_RADIUS_MM = 150.0      # HARD. The centre may never be this close to a surface.
INFLATION_MM = 200.0         # SOFT. Cost decays to ~nothing at this clearance.
OCC_POINT_PAD_MM = 25.0      # what a single range measurement is worth
WALL_PAD_MM = ROBOT_RADIUS_MM
START_CLEAR_MM = 150.0       # golden START_CLEAR_RADIUS: the box the rover starts in

# ============================================================= COST MODEL
# All costs are MILLIMETRES OF EQUIVALENT DRIVING. One unit, so the weights stay
# arguable; a search that adds millimetres to degrees to 0-254 costmap units is
# minimising something nobody chose.
DIAG_COST = math.sqrt(2.0)
# Fixed order => equal-cost successors are always generated the same way.
NEIGHBOURS = ((1, 0), (0, 1), (-1, 0), (0, -1),
              (1, 1), (1, -1), (-1, 1), (-1, -1))
TURN_COST_MM_PER_RAD = 240.0   # ~375 mm for a right angle, as measured on this chassis
TURN_COST_MM_FIXED = 60.0      # the settle that happens once per turn, however small
TURN_COST_DEADBAND_DEG = 2.0
CLEARANCE_COST_W = 0.25        # how much hugging a surface costs, per mm driven
PRIOR_COST_W = 1.5             # what a fully-confident learned cell costs, per mm

# ============================================================= THE CHASSIS
MIN_LEG_MM = 30.0              # golden `if sd < 30: continue`
MIN_TURN_DEG = 4.0             # under this the follower emits no turn at all
DP_EPSILON_MM = 150.0          # golden simplify_path(epsilon=150)
VIA_TOL_MM = 50.0              # how close counts as "passed through" a waypoint

LEARNED_GRID_PATH = "/var/lib/fpms/learned_grid.json"


# ================================================================== GEOMETRY
def wrap_pi(rad):
    r = (float(rad) + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if r == -math.pi else r


def wrap180(deg):
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def bearing_deg(dx_mm, dy_mm):
    """World bearing of a displacement, degrees CCW from +x."""
    return math.degrees(math.atan2(dy_mm, dx_mm))


def heading_error_deg(target_deg, actual_deg):
    return wrap180(target_deg - actual_deg)


def turn_cost_mm(delta_rad):
    """What changing heading by `delta_rad` costs, in millimetres of driving.

    Both terms are real: the rotation, and the fixed settle that happens once per
    turn however small it is. The fixed term is what stops the planner preferring
    five 18 degree turns to one 90.
    """
    d = abs(float(delta_rad))
    if not math.isfinite(d) or d < math.radians(TURN_COST_DEADBAND_DEG):
        return 0.0
    return d * TURN_COST_MM_PER_RAD + TURN_COST_MM_FIXED


def point_segment_dist(px, py, ax, ay, bx, by):
    """Golden's `point_segment_dist` (fpms_phase6:539), verbatim behaviour."""
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    denom = vx * vx + vy * vy
    if denom < 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
    cx, cy = ax + t * vx, ay + t * vy
    return math.hypot(px - cx, py - cy)


def douglas_peucker(pts, epsilon=DP_EPSILON_MM):
    """Golden's `simplify_path` (fpms_phase6:720). Removes redundant waypoints.

    Kept because it is the cheapest possible answer to "why does the rover turn
    fifteen times to drive one straight-ish line": an 8-connected search answers
    in staircases, and every step of a staircase is a real turn on a chassis
    where a right angle costs 375 mm of driving. Accepts [(x, y), ...] or
    [{"x":, "y":}, ...] exactly as golden's does.
    """
    if len(pts) <= 2:
        return list(pts)

    def _gx(p):
        return p["x"] if isinstance(p, dict) else p[0]

    def _gy(p):
        return p["y"] if isinstance(p, dict) else p[1]

    s, e = pts[0], pts[-1]
    dx, dy = _gx(e) - _gx(s), _gy(e) - _gy(s)
    ll = math.hypot(dx, dy)
    mx_d, mx_i = 0.0, 0
    for i in range(1, len(pts) - 1):
        if ll < 1:
            d = math.hypot(_gx(pts[i]) - _gx(s), _gy(pts[i]) - _gy(s))
        else:
            d = abs(dx * (_gy(s) - _gy(pts[i]))
                    - (_gx(s) - _gx(pts[i])) * dy) / ll
        if d > mx_d:
            mx_d, mx_i = d, i
    if mx_d > epsilon:
        left = douglas_peucker(pts[:mx_i + 1], epsilon)
        right = douglas_peucker(pts[mx_i:], epsilon)
        return left[:-1] + right
    return [s, e]


# ============================================================ LEARNED PRIOR
class LearnedPrior:
    """An ADDITIVE cost hint from `/var/lib/fpms/learned_grid.json`. Never a wall.

    THE CONTRACT, and it is the whole reason this class is so defensive:

      * The prior may only ADD cost. There is no code path in this file by which
        a learned cell becomes impassable, and there is no code path by which a
        missing, stale or malformed file changes the route at all. A rover that
        cannot plan because a JSON file is corrupt is a worse rover than one that
        never had the file.
      * Every failure is SILENT and total: `load_learned_prior` returns None and
        the planner proceeds exactly as if the path had never been configured.
        Nothing here raises, and nothing here logs to stderr -- the caller owns
        whether to mention it.

    SCHEMA (fixed; another agent writes exactly this):
        {"version": 1, "arena_mm": [1500, 1200], "cell_mm": <int>,
         "updated": <unix>, "notes": "...",
         "cells": [{"x": <mm>, "y": <mm>, "hits": <int>, "misses": <int>,
                    "conf": <0..1>}]}

    Confidence is `conf` when it is a usable number in [0, 1], else it is derived
    as hits / (hits + misses), else the cell is dropped. Cells are indexed at the
    PLANNER's resolution, not the file's, so a prior written on a different
    cell_mm still lands in the right place -- the file gives millimetres and
    millimetres are unambiguous.
    """

    def __init__(self, cells=None, cell_mm=CELL_MM, weight=PRIOR_COST_W,
                 source=None, updated=None, notes=""):
        self.cell_mm = float(cell_mm)
        self.weight = max(0.0, float(weight))
        self.source = source
        self.updated = updated
        self.notes = notes or ""
        # (ix, iy) -> confidence in [0, 1]. Highest confidence wins on collision,
        # deterministically, because two file cells can fall in one planner cell.
        self._cells = dict(cells or {})

    def __len__(self):
        return len(self._cells)

    def __bool__(self):
        return bool(self._cells) and self.weight > 0.0

    def confidence(self, x_mm, y_mm):
        return self._cells.get((int(math.floor(float(x_mm) / self.cell_mm)),
                                int(math.floor(float(y_mm) / self.cell_mm))),
                               0.0)

    def cost_frac(self, x_mm, y_mm):
        """Dimensionless multiplier ADDED to the per-mm driving cost. >= 0."""
        return self.weight * self.confidence(x_mm, y_mm)

    def rescale(self, cell_mm):
        """Re-index onto a different planner resolution. Pure; returns a new one."""
        cell_mm = float(cell_mm)
        if abs(cell_mm - self.cell_mm) < 1e-9:
            return self
        out = {}
        for (ix, iy), c in self._cells.items():
            x = (ix + 0.5) * self.cell_mm
            y = (iy + 0.5) * self.cell_mm
            k = (int(math.floor(x / cell_mm)), int(math.floor(y / cell_mm)))
            if c > out.get(k, -1.0):
                out[k] = c
        return LearnedPrior(out, cell_mm, self.weight, self.source,
                            self.updated, self.notes)


def load_learned_prior(path=LEARNED_GRID_PATH, cell_mm=CELL_MM,
                       arena_w_mm=ARENA_W_MM, arena_h_mm=ARENA_H_MM,
                       weight=PRIOR_COST_W, max_age_s=None, now=None):
    """Read the learned grid, or return None. NEVER raises. NEVER logs.

    `max_age_s` defaults to None -- staleness checking is OFF -- because it is
    the only thing in this module that would read a clock, and a planner whose
    route depends on what time it is cannot be deterministic. Pass a number only
    if the caller has decided it wants that trade.
    """
    try:
        if not path or not os.path.isfile(path):
            return None
        if os.path.getsize(path) > 8 * 1024 * 1024:
            return None                      # a plausible file is kilobytes
        with open(path, "r") as fh:
            doc = json.load(fh)
        if not isinstance(doc, dict):
            return None
        if int(doc.get("version", 0)) != 1:
            return None
        if max_age_s is not None:
            upd = float(doc.get("updated", 0))
            ref = time.time() if now is None else float(now)
            if not math.isfinite(upd) or (ref - upd) > float(max_age_s):
                return None
        raw = doc.get("cells")
        if not isinstance(raw, list):
            return None
        cell_mm = float(cell_mm)
        cells = {}
        for c in raw:
            if not isinstance(c, dict):
                continue
            try:
                x = float(c["x"])
                y = float(c["y"])
            except Exception:
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            # The arena clip applies to the prior too: a learned cell outside the
            # box describes floor that is not ours.
            if x < 0.0 or y < 0.0 or x > arena_w_mm or y > arena_h_mm:
                continue
            conf = None
            try:
                v = float(c.get("conf"))
                if math.isfinite(v) and 0.0 <= v <= 1.0:
                    conf = v
            except Exception:
                conf = None
            if conf is None:
                try:
                    hits = float(c.get("hits", 0))
                    miss = float(c.get("misses", 0))
                    tot = hits + miss
                    if tot > 0 and hits >= 0 and miss >= 0:
                        conf = max(0.0, min(1.0, hits / tot))
                except Exception:
                    conf = None
            if conf is None or conf <= 0.0:
                continue
            k = (int(math.floor(x / cell_mm)), int(math.floor(y / cell_mm)))
            if conf > cells.get(k, -1.0):
                cells[k] = conf
        if not cells:
            return None
        return LearnedPrior(cells, cell_mm, weight, path,
                            doc.get("updated"), doc.get("notes", ""))
    except Exception:
        # EVERY failure mode lands here on purpose: unreadable, truncated, not
        # JSON, a list where a dict was promised, a permission error. The answer
        # is always the same one -- plan as if the file did not exist.
        return None


# =========================================================== OCCUPANCY GRID
class OccupancyGrid:
    """What the LiDAR has seen, in ARENA millimetres, on a RECTANGULAR grid.

    Ported from `fpms_missions.OccupancyGrid` (2253) with two changes:
      * nx x ny, not n x n. The arena is 1500 x 1200 and a square grid puts
        300 mm of floor along +y that does not exist.
      * the three filters are applied HERE, in one place, where they can be
        tested: self-filter by range, arena clip by position, target exclusion by
        distance to the leg's own target.

    Three things it deliberately does NOT do, each inherited for its own reason:
      * a ZERO RANGE IS NOT AN OBSTACLE -- it is "no return", and writing it in
        would place a wall underneath the rover and make the goal unreachable;
      * it does NOT clear cells by ray-casting -- one badly-posed scan would
        erase a real obstacle. Cells expire by TIME instead;
      * it does NOT believe one return -- `min_hits` scans must agree.
    """

    MEAN_CAP = 16

    def __init__(self, cell_mm=CELL_MM, arena_w_mm=ARENA_W_MM,
                 arena_h_mm=ARENA_H_MM, ttl_s=6.0, min_hits=2,
                 max_range_mm=1800.0, self_filter_mm=SELF_FILTER_MM,
                 clip_margin_mm=ARENA_CLIP_MARGIN_MM, wall_band_mm=80.0):
        self.cell_mm = float(cell_mm)
        self.arena_w_mm = float(arena_w_mm)
        self.arena_h_mm = float(arena_h_mm)
        self.nx = max(1, int(math.ceil(self.arena_w_mm / self.cell_mm)))
        self.ny = max(1, int(math.ceil(self.arena_h_mm / self.cell_mm)))
        self.ttl_s = float(ttl_s)
        self.min_hits = int(min_hits)
        self.max_range_mm = float(max_range_mm)
        self.self_filter_mm = float(self_filter_mm)
        self.clip_margin_mm = float(clip_margin_mm)
        self.wall_band_mm = float(wall_band_mm)
        # (ix, iy) -> [hits, last_seen, mean_x, mean_y, n_mean]
        self._cells = {}
        self._lock = threading.Lock()
        self.scans = 0
        self.last_scan_t = 0.0
        # (x, y, r) or None -- golden's P5_TARGET_EXCLUDE_MM. Assigned whole,
        # never mutated, so it reads without the lock.
        self.exclude = None
        # Bumped on every change that can move a route. `IncrementalPlanner`
        # watches it to decide whether D* Lite has anything to repair.
        self.revision = 0

    # -- the target exclusion ---------------------------------------------
    def set_exclusion(self, x_mm, y_mm, r_mm=None):
        """Returns within r_mm of this arena point are THE THING BEING DOCKED
        AGAINST and are never folded in. Golden P5_TARGET_EXCLUDE_MM = 220.

        Without this the rover aborts on the thing it is docking against: the
        goal cell goes occupied, every search reports "no route", and the leg
        that was supposed to end in a dock ends in an abort instead.
        """
        r = TARGET_EXCLUDE_MM if r_mm is None else float(r_mm)
        if not math.isfinite(r) or r <= 0.0:
            self.exclude = None
            return
        self.exclude = (float(x_mm), float(y_mm), r)

    def clear_exclusion(self):
        self.exclude = None

    # -- geometry ----------------------------------------------------------
    def cell_of(self, x_mm, y_mm):
        return (int(math.floor(float(x_mm) / self.cell_mm)),
                int(math.floor(float(y_mm) / self.cell_mm)))

    def centre_of(self, ix, iy):
        return ((ix + 0.5) * self.cell_mm, (iy + 0.5) * self.cell_mm)

    def in_grid(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def in_arena(self, x_mm, y_mm, margin_mm=0.0):
        return (margin_mm <= x_mm <= self.arena_w_mm - margin_mm
                and margin_mm <= y_mm <= self.arena_h_mm - margin_mm)

    # -- writing -----------------------------------------------------------
    def mark(self, x_mm, y_mm, now=None):
        """Record ONE observed surface point in arena mm. Returns its cell or None.

        THE ARENA CLIP IS HERE (`in_grid`): a return outside the box is room
        clutter -- a chair leg beyond the tape -- and golden discarded it for the
        same reason.
        """
        if not (math.isfinite(x_mm) and math.isfinite(y_mm)):
            return None
        c = self.cell_of(x_mm, y_mm)
        if not self.in_grid(c[0], c[1]):
            return None
        now = 0.0 if now is None else float(now)
        x_mm, y_mm = float(x_mm), float(y_mm)
        with self._lock:
            e = self._cells.get(c)
            if e is None or (now - e[1]) > self.ttl_s:
                # An expired cell is a NEW observation, not a continuation. A
                # count that carried across the gap would let a cell touched once
                # an hour block a route for ever.
                self._cells[c] = [1, now, x_mm, y_mm, 1]
                self.revision += 1
            else:
                was = e[0] >= self.min_hits
                e[0] = min(e[0] + 1, self.min_hits + 8)
                e[1] = now
                k = min(e[4] + 1, self.MEAN_CAP)
                e[2] += (x_mm - e[2]) / k
                e[3] += (y_mm - e[3]) / k
                e[4] = k
                if not was and e[0] >= self.min_hits:
                    self.revision += 1
        return c

    def mark_obstacle(self, x_mm, y_mm, r_mm=0.0, now=None):
        """Believe a disc immediately -- a keepout, or the thing that just stopped
        the rover. Golden's KeepoutFilter idea (`planning_obstacles`, 582-609),
        which sampled a user-placed disk as virtual LiDAR returns so the search
        could not route through what the operator had marked.

        `min_hits` is bypassed on purpose: this is not a sensor return, it is a
        statement, and requiring two of them would mean the cell that stopped the
        rover is not in the map when the reroute is planned.
        """
        now = 0.0 if now is None else float(now)
        r = max(0.0, float(r_mm))
        step = max(self.cell_mm * 0.5, 1.0)
        k = int(math.floor(r / step))
        pts = [(0.0, 0.0)] if k <= 0 else [
            (dx * step, dy * step)
            for dx in range(-k, k + 1) for dy in range(-k, k + 1)
            if (dx * step) ** 2 + (dy * step) ** 2 <= r * r]
        n = 0
        for dx, dy in pts:
            c = self.cell_of(x_mm + dx, y_mm + dy)
            if not self.in_grid(*c):
                continue
            with self._lock:
                self._cells[c] = [self.min_hits + 8, now,
                                  x_mm + dx, y_mm + dy, self.MEAN_CAP]
            n += 1
        if n:
            self.revision += 1
        return n

    def integrate(self, ranges_m, pose, range_max_m=6.0, now=None,
                  rotation_sign=-1.0, zero_offset_deg=0.0):
        """Fold one 360-bin scan into the arena frame. Returns how many landed.

        THE THREE FILTERS, IN THE ORDER GOLDEN APPLIED THEM:
          1. self-filter   -- `mm < SELF_FILTER_MM` is the chassis, not the world
          2. target exclude -- inside `self.exclude` is the docking target
          3. arena clip    -- `mark` refuses anything outside the box

        Bin i is `rotation_sign * i * 360/n + zero_offset` degrees from the nose,
        which is exactly how the cone guard measures. If the mount yaw is wrong
        this grid is wrong by the same angle in the same direction as the guard
        that has been stopping the rover all along: one calibration, one failure
        mode.
        """
        if not ranges_m or pose is None:
            return 0
        px, py, ph = float(pose[0]), float(pose[1]), float(pose[2])
        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(ph)):
            return 0
        n = len(ranges_m)
        step = 360.0 / n
        sat = float(range_max_m) * 0.995
        now = 0.0 if now is None else float(now)
        marked = 0
        ex = self.exclude                   # read ONCE
        for i, r in enumerate(ranges_m):
            try:
                rv = float(r)
            except Exception:
                continue
            if not math.isfinite(rv) or rv <= 0.0 or rv >= sat:
                continue                    # UNKNOWN, not free and not occupied
            mm = rv * 1000.0
            if mm > self.max_range_mm:
                continue
            if mm < self.self_filter_mm:
                continue                    # (1) the chassis is not an obstacle
            b = math.radians(ph + rotation_sign * (i * step) + zero_offset_deg)
            hx = px + mm * math.cos(b)
            hy = py + mm * math.sin(b)
            if ex is not None and math.hypot(hx - ex[0], hy - ex[1]) <= ex[2]:
                continue                    # (2) neither is the docking target
            if self.mark(hx, hy, now):      # (3) arena clip lives in `mark`
                marked += 1
        with self._lock:
            self.scans += 1
            self.last_scan_t = now
        return marked

    # -- reading -----------------------------------------------------------
    def occupied(self, now=None):
        """Cells seen often enough, recently enough, to be believed. A SNAPSHOT:
        an A* whose obstacles move mid-search can return a path through a cell it
        has already rejected."""
        now = 0.0 if now is None else float(now)
        with self._lock:
            items = list(self._cells.items())
        return set(c for c, e in items
                   if e[0] >= self.min_hits and (now - e[1]) <= self.ttl_s)

    def points(self, now=None):
        """Believed surfaces as MEASURED CENTROIDS: [(x, y, is_wall), ...].

        SORTED, because everything downstream has to be deterministic and dict
        iteration order is not something to bet a replan on.

        `is_wall` is decided here, once: a centroid within `wall_band_mm` of the
        boundary IS the arena wall, which the static wall pad already models from
        a number we know exactly. Re-deriving it from a noisy range would
        double-count a boundary that was never in doubt.
        """
        now = 0.0 if now is None else float(now)
        with self._lock:
            items = list(self._cells.items())
        out = []
        for c, e in sorted(items):
            if e[0] < self.min_hits or (now - e[1]) > self.ttl_s:
                continue
            x, y = e[2], e[3]
            edge = min(x, y, self.arena_w_mm - x, self.arena_h_mm - y)
            out.append((x, y, edge <= self.wall_band_mm))
        return out

    def forget_stale(self, now=None):
        now = 0.0 if now is None else float(now)
        with self._lock:
            dead = [c for c, e in self._cells.items()
                    if (now - e[1]) > self.ttl_s]
            for c in dead:
                del self._cells[c]
        if dead:
            self.revision += 1
        return len(dead)

    def stats(self, now=None):
        now = 0.0 if now is None else float(now)
        with self._lock:
            total = len(self._cells)
            scans, last = self.scans, self.last_scan_t
        pts = self.points(now=now)
        walls = sum(1 for p in pts if p[2])
        return {"cells": len(pts), "cells_tracked": total, "scans": scans,
                "wall_cells": walls, "obstacle_cells": len(pts) - walls,
                "nx": self.nx, "ny": self.ny, "cell_mm": self.cell_mm,
                "arena_mm": [self.arena_w_mm, self.arena_h_mm],
                "revision": self.revision,
                "age_s": None if not last else round(now - last, 1)}


# ================================================================== COSTMAP
class CostMap:
    """The three layers the rover cares about, built once per replan. PURE.

    Nav2's LAYERED COSTMAP idea and nothing else from Nav2: a static layer (the
    arena boundary, known exactly), an obstacle layer (what the LiDAR believes),
    and an inflation layer (a soft cost that decays with distance so routes
    prefer the middle of free space). Plus, here, a fourth: the learned prior,
    which is additive and can never block.

    SAFETY IS CONTINUOUS, NOT CELLULAR. `clear_mm` measures a real distance from
    a real point to the nearest measured surface. The grid survives only as an
    index for finding candidate points quickly, which is all a grid was ever good
    for. Rounding a wall to a cell centre and then padding by half a cell to
    cover the error cost 30 mm of a 58 mm corridor on this arena and turned a
    60 mm measurement error into a total refusal.

    REAL-FOOTPRINT INFLATION (AGENT_DESIGN.md item 2), replacing golden's flat
    205 mm disc:
        HARD  centre must clear every surface by  robot_radius + point_pad
        SOFT  cost decays as exp(-clearance / decay) out to `inflation_mm`,
              and is exactly zero beyond it
    so close quarters are refused outright and merely-tight quarters are merely
    expensive -- which is the distinction a flat disc cannot make.
    """

    def __init__(self, points, nx, ny, cell_mm, arena_w_mm=ARENA_W_MM,
                 arena_h_mm=ARENA_H_MM, radius_mm=None, wall_pad_mm=None,
                 point_pad_mm=None, inflation_mm=None, free=(),
                 absorb_walls=True, prior=None):
        self.nx, self.ny = int(nx), int(ny)
        self.cell_mm = float(cell_mm)
        self.arena_w_mm = float(arena_w_mm)
        self.arena_h_mm = float(arena_h_mm)
        self.radius_mm = ROBOT_RADIUS_MM if radius_mm is None else float(radius_mm)
        self.wall_pad_mm = WALL_PAD_MM if wall_pad_mm is None else float(wall_pad_mm)
        self.point_pad_mm = (OCC_POINT_PAD_MM if point_pad_mm is None
                             else float(point_pad_mm))
        self.inflation_mm = (INFLATION_MM if inflation_mm is None
                             else float(inflation_mm))
        # exp(-3) ~= 5% left at the inflation boundary, so zeroing it there is a
        # step of 5% of one weight rather than a cliff.
        self.decay_mm = max(1.0, self.inflation_mm / 3.0)
        self.reach = self.radius_mm + self.point_pad_mm
        self.free = set(tuple(c) for c in free)
        self.prior = prior.rescale(self.cell_mm) if prior else None

        self.walls_absorbed = 0
        obs = []
        for p in points:
            x, y = float(p[0]), float(p[1])
            is_wall = bool(p[2]) if len(p) > 2 else False
            if is_wall and absorb_walls:
                self.walls_absorbed += 1
                continue
            obs.append((x, y))
        obs.sort()                          # determinism, not tidiness
        self.obstacles = obs

        # SPATIAL INDEX, not a second copy of the map. The stamp radius is
        # `reach + inflation + one cell diagonal`, because a query may sit
        # anywhere in its cell -- getting that wrong makes the index MISS a
        # hazard, the one error direction that is not allowed here.
        span = int(math.ceil((self.reach + self.inflation_mm
                              + self.cell_mm * DIAG_COST) / self.cell_mm))
        self._near = {}
        for (ox, oy) in obs:
            cx0 = int(math.floor(ox / self.cell_mm))
            cy0 = int(math.floor(oy / self.cell_mm))
            for dx in range(-span, span + 1):
                for dy in range(-span, span + 1):
                    ix, iy = cx0 + dx, cy0 + dy
                    if 0 <= ix < self.nx and 0 <= iy < self.ny:
                        self._near.setdefault((ix, iy), []).append((ox, oy))

        # The cell view, for the searches. A cell is blocked when its CENTRE is
        # unsafe: deliberately the coarse test, because every candidate the
        # search produces is re-checked continuously by `segment_ok` before it
        # becomes a route.
        self.blocked = set()
        self._prox = {}
        for ix in range(self.nx):
            for iy in range(self.ny):
                cx, cy = self.centre_of(ix, iy)
                m = self.clear_mm(cx, cy)
                if (ix, iy) not in self.free and m < 0.0:
                    self.blocked.add((ix, iy))
                self._prox[(ix, iy)] = (
                    0.0 if m >= self.inflation_mm
                    else math.exp(-max(0.0, m) / self.decay_mm))

    # -- geometry ----------------------------------------------------------
    def cell_of(self, x, y):
        return (int(math.floor(float(x) / self.cell_mm)),
                int(math.floor(float(y) / self.cell_mm)))

    def centre_of(self, ix, iy):
        return ((ix + 0.5) * self.cell_mm, (iy + 0.5) * self.cell_mm)

    def in_grid(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    # -- the safety question, asked continuously ---------------------------
    def clear_mm(self, x, y):
        """Margin in mm at an arena point: >= 0 means the rover centre may be here.

        Negative says how far INSIDE the forbidden region it is, which is more
        useful than a bool when relaxing pads down a ladder.
        """
        c = self.cell_of(x, y)
        if c in self.free:
            # The cell the rover is standing in is ALWAYS passable, whatever the
            # map believes. A rover parked inside the wall pad, or with one scan
            # artefact under itself, must still be able to plan the route out of
            # the trouble it is in.
            return float(self.reach)
        m = min(x - self.wall_pad_mm, y - self.wall_pad_mm,
                (self.arena_w_mm - self.wall_pad_mm) - x,
                (self.arena_h_mm - self.wall_pad_mm) - y)
        for (ox, oy) in self._near.get(c, ()):
            m = min(m, math.hypot(x - ox, y - oy) - self.reach)
            if m < 0.0:
                break
        return m

    def safe(self, x, y):
        return self.clear_mm(x, y) >= 0.0

    def segment_ok(self, x0, y0, x1, y1, step_mm=None):
        """Can the rover drive straight from one arena point to another?

        Sampled in CONTINUOUS space at half a cell, which cannot step over an
        obstacle whose forbidden disc is `reach` (>= 175 mm) across. This is the
        ONE predicate used for Theta*'s line of sight, for the "is the straight
        line already clear" short circuit, and for the final check on the
        published route -- so the search, the shortcut and the plan can never
        disagree about what "clear" means.
        """
        step = float(step_mm) if step_mm else self.cell_mm * 0.5
        d = math.hypot(x1 - x0, y1 - y0)
        k = max(1, int(math.ceil(d / step)))
        for i in range(k + 1):
            t = float(i) / k
            if not self.safe(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t):
                return False
        return True

    # -- the soft layers ---------------------------------------------------
    def proximity(self, x, y):
        """How squeezed a point is: ->1 against a surface, 0 beyond inflation_mm."""
        return self._prox.get(self.cell_of(x, y), 0.0)

    def prior_frac(self, x, y):
        """Learned extra cost at a point, dimensionless and >= 0. Never blocks."""
        return self.prior.cost_frac(x, y) if self.prior else 0.0

    def segment_cost_mm(self, x0, y0, x1, y1):
        """Length of a straight run, plus what driving it THERE costs.

            cost = d * (1 + w * mean_proximity + mean_prior)

        Per millimetre travelled, so a long run through open space is not
        penalised for being long and a long run through a squeeze is penalised in
        proportion to how much of it is squeezed. AVERAGED over the run and not
        sampled at one end, because a leg that passes an obstacle half way along
        is exactly as squeezed as one that starts beside it -- and a planner that
        only looked at its endpoints would thread the gap in the middle for free,
        which is precisely what this exists to stop.
        """
        d = math.hypot(x1 - x0, y1 - y0)
        if d <= 0.0:
            return 0.0
        if CLEARANCE_COST_W <= 0.0 and self.prior is None:
            return d
        k = max(1, int(math.ceil(d / self.cell_mm)))
        acc_p = 0.0
        acc_l = 0.0
        for i in range(k + 1):
            t = float(i) / k
            sx, sy = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            acc_p += self.proximity(sx, sy)
            if self.prior is not None:
                acc_l += self.prior_frac(sx, sy)
        n = float(k + 1)
        return d * (1.0 + CLEARANCE_COST_W * acc_p / n + acc_l / n)

    def cell_step_cost(self, a, b):
        """Cost of the 8-connected edge a->b in millimetres. Symmetric.

        Used by `astar_cells` and by D* Lite, which needs an UNDIRECTED graph:
        the prior is charged as the mean of the two endpoints so c(a,b) ==
        c(b,a), and the corner-cut rule is symmetric by construction.
        """
        step = (DIAG_COST if (a[0] != b[0] and a[1] != b[1]) else 1.0) * self.cell_mm
        if self.prior is None and CLEARANCE_COST_W <= 0.0:
            return step
        pa, pb = self.centre_of(*a), self.centre_of(*b)
        soft = 0.5 * (self.proximity(*pa) + self.proximity(*pb))
        lrn = 0.5 * (self.prior_frac(*pa) + self.prior_frac(*pb))
        return step * (1.0 + CLEARANCE_COST_W * soft + lrn)


# ==================================================================== A*
def _astar_core(is_blocked, nx, ny, start, goal, moves, heuristic,
                step_cost, allow_corner_cut, edge_penalty=None):
    """The one search engine. Golden's `astar` (fpms_phase6:670) generalised.

    Golden's loop, structurally unchanged: a binary heap keyed on
    (f, g, cell) so ties break on WHICH cell and never on when a node was pushed;
    a `closed` set; `came` written only on a strict improvement, so when two
    routes tie the one generated FIRST by `moves` wins. Both of those are why the
    same arena plans the same route on the second replan as on the first.

    Everything golden hardcoded is a parameter here, and the two callers differ
    only in these:
      GoldenB6      moves = golden's order, Euclidean h, unit steps,
                    corner cutting ALLOWED, golden's x-band edge penalty
      arena planner NEIGHBOURS, octile h, millimetre steps from the costmap,
                    corner cutting REFUSED (a rover with width cannot slip
                    diagonally between two blocked cells), no edge penalty
    """
    if not (0 <= start[0] < nx and 0 <= start[1] < ny):
        return None
    if not (0 <= goal[0] < nx and 0 <= goal[1] < ny):
        return None
    if is_blocked(goal[0], goal[1]):
        # A blocked GOAL is answered here rather than by an exhaustive search
        # that can only fail. "the target is inside an obstacle (or its
        # inflation)" is a real answer, and the caller turns it into an abort.
        return None
    if start == goal:
        return [start]
    h0 = heuristic(start, goal)
    q = [(h0, 0.0, start)]
    came = {}
    gscore = {start: 0.0}
    closed = set()
    while q:
        _f, g, cur = heapq.heappop(q)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return path
        cx, cy = cur
        for dc, dr in moves:
            nb = (cx + dc, cy + dr)
            c, r = nb
            if c < 0 or c >= nx or r < 0 or r >= ny or is_blocked(c, r):
                continue
            if dc and dr and not allow_corner_cut:
                if is_blocked(cx + dc, cy) or is_blocked(cx, cy + dr):
                    continue
            ng = g + step_cost(cur, nb)
            if edge_penalty is not None:
                ng += edge_penalty(nb)
            if ng < gscore.get(nb, float("inf")):
                gscore[nb] = ng
                came[nb] = cur
                heapq.heappush(q, (ng + heuristic(nb, goal), ng, nb))
    return None


def _octile(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (DIAG_COST - 2.0) * min(dx, dy)


def astar_cells(cmap, start_cell, goal_cell, free=()):
    """8-connected A* over the costmap's CELL view. [(ix, iy), ...] or None.

    THE BASELINE AND THE FALLBACK, exactly as the operator asked: this is
    golden's B6 search, on golden's engine, with golden's tie-breaks -- only the
    grid is the arena rather than the robot-relative box, the step cost comes
    from the costmap (so it inherits the real-footprint inflation and the learned
    prior), and corner cutting is refused because a 340 mm chassis cannot slip
    between two blocked cells however cheap the grid says it is.

    The heuristic is OCTILE, the exact cost of an unobstructed 8-connected walk,
    scaled to millimetres. Admissible and consistent against a step cost that can
    only be >= its geometric length, so the first time the goal is popped it is
    optimal and no closed cell is ever reopened.
    """
    free = set(tuple(c) for c in free)
    blocked = cmap.blocked - free

    def is_blocked(ix, iy):
        return (ix, iy) in blocked

    return _astar_core(
        is_blocked, cmap.nx, cmap.ny, tuple(start_cell), tuple(goal_cell),
        NEIGHBOURS,
        lambda a, b: _octile(a, b) * cmap.cell_mm,
        cmap.cell_step_cost,
        allow_corner_cut=False)


def astar(cmap, start_xy, goal_xy):
    """A* in ARENA MILLIMETRES. [(x, y), ...] from the true pose to the true
    target, or None.

    THE ENDPOINTS ARE THE TRUE POSE AND THE TRUE TARGET, never cell centres. The
    grid is a search space, not a coordinate system: docking on a cell centre
    would leave the rover up to 35 mm from the zone it was asked for, and that
    error reads as odometry drift in every log.
    """
    s = cmap.cell_of(*start_xy)
    g = cmap.cell_of(*goal_xy)
    if not (cmap.in_grid(*s) and cmap.in_grid(*g)):
        return None
    if not cmap.safe(*goal_xy):
        return None
    if s == g:
        return [tuple(start_xy), tuple(goal_xy)]
    cells = astar_cells(cmap, s, g, free=(s, g))
    if cells is None:
        return None
    pts = [(float(start_xy[0]), float(start_xy[1]))]
    for c in cells[1:-1]:
        pts.append(cmap.centre_of(*c))
    pts.append((float(goal_xy[0]), float(goal_xy[1])))
    return pts


# ================================================================= THETA*
def theta_star(cmap, start_xy, goal_xy, heading_deg):
    """ANY-ANGLE search. [(x, y), ...] from start to goal, or None.

    WHY ANY-ANGLE IS NOT A LUXURY ON THIS ROVER. An 8-connected A* can only leave
    a cell in one of eight directions, so a route that really wants to run at 20
    degrees comes back as a staircase of 45 degree steps. Douglas-Peucker then
    pulls that staircase straight again -- which works, but it is repairing
    damage the search did not have to do, and it can only remove turns the search
    already committed to, never choose a better angle in the first place. On a
    chassis where one right angle costs ~375 mm of driving, that is the whole
    ball game. Theta* (Daniel, Nash, Koenig & Felner 2010) is A* with one extra
    question at every relaxation: can the successor see this node's PARENT? If
    it can, hang it off the parent and skip the intermediate vertex entirely.

    STRAIGHT LINE WHEN NOTHING BLOCKS: the very first thing this does is ask
    `segment_ok(start, goal)`, and on a clean arena it returns a two-point route
    without searching at all.

    BOTH PATHS ARE COSTED AND THE CHEAPER WINS -- the one place this must NOT
    follow the published algorithm. Basic Theta* takes the parent shortcut
    unconditionally whenever line of sight allows, and under uniform cost the
    triangle inequality entitles it to. THAT GUARANTEE IS GONE HERE: the cost is
    length plus a turn charge plus a proximity charge plus the learned prior, and
    a straight shortcut past an obstacle can genuinely cost more than going via
    the bend because it spends the whole run hugging the thing. Taking the
    shortcut on sight alone would cut straight through the inflation gradient the
    costmap exists to provide.

    HONEST ABOUT OPTIMALITY: basic Theta* is not guaranteed optimal even on
    length alone, and the turn term makes cost history-dependent on top of that,
    so a closed node is not provably final. Accepted -- this is a 30 x 24 grid
    re-derived at every leg boundary. The heuristic stays ADMISSIBLE (straight
    line distance, which turn and proximity costs can only add to), so the search
    stays directed and still terminates.
    """
    s_cell = cmap.cell_of(*start_xy)
    g_cell = cmap.cell_of(*goal_xy)
    if not (cmap.in_grid(*s_cell) and cmap.in_grid(*g_cell)):
        return None
    if not cmap.safe(*goal_xy):
        return None
    if cmap.segment_ok(start_xy[0], start_xy[1], goal_xy[0], goal_xy[1]):
        return [(float(start_xy[0]), float(start_xy[1])),
                (float(goal_xy[0]), float(goal_xy[1]))]
    if s_cell == g_cell:
        return None

    def pos(c):
        if c == s_cell:
            return (float(start_xy[0]), float(start_xy[1]))
        if c == g_cell:
            return (float(goal_xy[0]), float(goal_xy[1]))
        return cmap.centre_of(*c)

    gx, gy = pos(g_cell)

    def h(c):
        px, py = pos(c)
        return math.hypot(gx - px, gy - py)

    # MEMOISED, because the same question is asked thousands of times and both
    # answers depend only on the PAIR OF CELLS. Symmetric key: a line and its
    # reverse are the same line, and both predicates sample the same points
    # either way round.
    _los, _cost = {}, {}

    def los(a, b):
        k = (a, b) if a <= b else (b, a)
        v = _los.get(k)
        if v is None:
            ax, ay = pos(a)
            bx, by = pos(b)
            v = _los[k] = cmap.segment_ok(ax, ay, bx, by)
        return v

    def seg_cost(a, b):
        k = (a, b) if a <= b else (b, a)
        v = _cost.get(k)
        if v is None:
            ax, ay = pos(a)
            bx, by = pos(b)
            v = _cost[k] = cmap.segment_cost_mm(ax, ay, bx, by)
        return v

    parent = {s_cell: None}
    g = {s_cell: 0.0}
    hdg = {s_cell: math.radians(float(heading_deg))}
    closed = set()
    h0 = h(s_cell)
    heap = [(h0, h0, s_cell[0], s_cell[1])]

    while heap:
        _f, _h, cx, cy = heapq.heappop(heap)
        cur = (cx, cy)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == g_cell:
            out, c = [], cur
            while c is not None:
                out.append(pos(c))
                c = parent[c]
            out.reverse()
            return out
        for dx, dy in NEIGHBOURS:
            nb = (cx + dx, cy + dy)
            if not cmap.in_grid(*nb) or nb in closed:
                continue
            if nb in cmap.blocked and nb != g_cell:
                continue
            gp = parent.get(cur)
            cands = []
            bx, by = pos(nb)
            if los(cur, nb):
                ax, ay = pos(cur)
                if math.hypot(bx - ax, by - ay) > 1e-9:
                    b1 = math.atan2(by - ay, bx - ax)
                    cands.append((g[cur] + seg_cost(cur, nb)
                                  + turn_cost_mm(wrap_pi(b1 - hdg[cur])),
                                  cur, b1))
            if gp is not None and los(gp, nb):
                px, py = pos(gp)
                if math.hypot(bx - px, by - py) > 1e-9:
                    b2 = math.atan2(by - py, bx - px)
                    cands.append((g[gp] + seg_cost(gp, nb)
                                  + turn_cost_mm(wrap_pi(b2 - hdg[gp])),
                                  gp, b2))
            if not cands:
                continue
            best = min(cands, key=lambda t: (t[0], t[1]))
            if best[0] < g.get(nb, float("inf")) - 1e-9:
                g[nb] = best[0]
                parent[nb] = best[1]
                hdg[nb] = best[2]
                hb = h(nb)
                heapq.heappush(heap, (best[0] + hb, hb, nb[0], nb[1]))
    return None


# ================================================================ D* LITE
class DStarLite:
    """TRUE incremental replanning. Koenig & Likhachev 2002, optimized version.

    WHY IT IS HERE AND WHAT IT ACTUALLY BUYS. A* and Theta* answer "what is the
    route from here", and when an obstacle appears half way down a leg the only
    thing they can do is throw the answer away and search the whole arena again
    from the rover's new position. D* Lite REPAIRS: it keeps the search tree, and
    when cells change it re-expands only the states whose cost-to-goal the change
    could possibly have altered. On a leg where the rover has already driven most
    of the way, that is a handful of states instead of the whole grid, and -- the
    part that matters more than the speed -- the route it returns is CONTINUOUS
    with the one it was already driving instead of being a fresh opinion.

    THE STANDARD ALGORITHM, NOT AN APPROXIMATION OF IT. Everything the paper
    specifies is here and is named after it:
      * the search runs BACKWARD from the goal, so g/rhs are costs TO the goal
        and the rover moving does not invalidate them;
      * `rhs` one-step-lookahead values alongside `g`, and a state is
        INCONSISTENT when they differ;
      * the TWO-PART KEY  k(s) = [min(g,rhs) + h(s_start, s) + km, min(g,rhs)],
        compared lexicographically;
      * the KM OFFSET. When the rover moves, every key in the queue is stale by
        the heuristic difference. Re-keying the whole queue would cost more than
        the search; `km += h(s_last, s_start)` makes the old keys comparable to
        the new ones instead. This is the entire trick of D* Lite over D*.
      * `compute_shortest_path` with the three-way test on `k_old`, the
        over-consistent (g > rhs) and under-consistent (g < rhs) branches.

    THE GRAPH IS UNDIRECTED, which is required for `pred == succ`: the step cost
    is symmetric (`CostMap.cell_step_cost` charges the mean of the two
    endpoints), and the no-corner-cutting rule is symmetric by construction.

    WHAT IT DOES NOT DO, said plainly: D* Lite searches a state space of CELLS,
    so it cannot carry the turn cost that Theta* carries -- a turn cost is
    history-dependent and would break the consistency the repair rests on. So D*
    Lite answers in cells on length-plus-clearance-plus-prior, and the answer is
    then handed to the SAME line-of-sight shortcutter and Douglas-Peucker that
    Theta* output goes through. The route that comes out is any-angle; the search
    underneath it is not.

    DETERMINISM: the heap key carries the cell index, so ties break on which cell
    and never on insertion order; successor iteration is over the fixed
    NEIGHBOURS tuple; and the greedy path extraction breaks its own ties the same
    way. Two identical grids give one identical route.
    """

    INF = float("inf")

    def __init__(self, cmap, start_cell, goal_cell, free=()):
        self.cmap = cmap
        self.free = set(tuple(c) for c in free)
        self.s_start = tuple(start_cell)
        self.s_goal = tuple(goal_cell)
        self.s_last = self.s_start
        self.km = 0.0
        self.g = {}
        self.rhs = {}
        self._heap = []
        self._keys = {}          # cell -> the key it is CURRENTLY queued with
        self.expansions = 0      # states popped, for the "did it repair" evidence
        self.rhs[self.s_goal] = 0.0
        self._push(self.s_goal, self._key(self.s_goal))
        self.compute_shortest_path()

    # -- the graph ---------------------------------------------------------
    def blocked(self, c):
        if c in self.free:
            return False
        return c in self.cmap.blocked

    def _neighbours(self, c):
        cx, cy = c
        out = []
        for dx, dy in NEIGHBOURS:
            n = (cx + dx, cy + dy)
            if not self.cmap.in_grid(*n):
                continue
            if self.blocked(n):
                continue
            if dx and dy:
                # NO CORNER CUTTING, and it is symmetric so pred == succ holds.
                if self.blocked((cx + dx, cy)) or self.blocked((cx, cy + dy)):
                    continue
            out.append(n)
        return out

    def cost(self, a, b):
        if self.blocked(a) or self.blocked(b):
            return self.INF
        return self.cmap.cell_step_cost(a, b)

    def _h(self, a, b):
        # Octile in millimetres. Admissible against a step cost that is never
        # below its geometric length, which is what makes the repair sound.
        return _octile(a, b) * self.cmap.cell_mm

    def _g(self, s):
        return self.g.get(s, self.INF)

    def _rhs(self, s):
        return self.rhs.get(s, self.INF)

    # -- the two-part key --------------------------------------------------
    def _key(self, s):
        k2 = min(self._g(s), self._rhs(s))
        if k2 == self.INF:
            return (self.INF, self.INF)
        return (k2 + self._h(self.s_start, s) + self.km, k2)

    # -- the queue, with lazy deletion -------------------------------------
    def _push(self, s, key):
        self._keys[s] = key
        heapq.heappush(self._heap, (key[0], key[1], s[0], s[1]))

    def _remove(self, s):
        self._keys.pop(s, None)      # the heap entry is skipped when popped

    def _prune(self):
        while self._heap:
            k0, k1, ix, iy = self._heap[0]
            s = (ix, iy)
            cur = self._keys.get(s)
            if cur is not None and cur[0] == k0 and cur[1] == k1:
                return s, (k0, k1)
            heapq.heappop(self._heap)
        return None, None

    def _top_key(self):
        s, k = self._prune()
        return (self.INF, self.INF) if s is None else k

    def _pop(self):
        s, k = self._prune()
        if s is None:
            return None, None
        heapq.heappop(self._heap)
        self._keys.pop(s, None)
        return s, k

    # -- the algorithm -----------------------------------------------------
    def update_vertex(self, u):
        if u != self.s_goal:
            best = self.INF
            for s2 in self._neighbours(u):
                c = self.cost(u, s2)
                if c == self.INF:
                    continue
                v = c + self._g(s2)
                if v < best:
                    best = v
            self.rhs[u] = best
        self._remove(u)
        if self._g(u) != self._rhs(u):
            self._push(u, self._key(u))

    def compute_shortest_path(self, max_expansions=400000):
        n = 0
        while True:
            top = self._top_key()
            ks = self._key(self.s_start)
            if not (top < ks or self._rhs(self.s_start) > self._g(self.s_start)):
                break
            u, k_old = self._pop()
            if u is None:
                break
            n += 1
            if n > max_expansions:
                break
            k_new = self._key(u)
            if k_old < k_new:
                # The key was stale (km moved under it). Re-queue at its real key
                # rather than expanding it now -- this is how the km offset stays
                # sound without touching the rest of the queue.
                self._push(u, k_new)
            elif self._g(u) > self._rhs(u):
                # OVER-CONSISTENT: the cheap case. Make it consistent and tell
                # its predecessors.
                self.g[u] = self._rhs(u)
                for s in self._neighbours(u):
                    self.update_vertex(s)
            else:
                # UNDER-CONSISTENT: something got more expensive. Invalidate and
                # re-evaluate it AND its predecessors.
                self.g[u] = self.INF
                self.update_vertex(u)
                for s in self._neighbours(u):
                    self.update_vertex(s)
        self.expansions += n
        return n

    # -- the two things the executor calls ---------------------------------
    def move_start(self, new_start):
        """The rover has driven somewhere. Re-key with km, do not re-search.

        `km += h(s_last, s_start)` is the whole point of D* Lite: the keys of
        everything already in the queue were computed against the OLD start, and
        this offset makes them comparable to keys computed against the new one
        without touching a single entry.
        """
        new_start = tuple(new_start)
        if new_start == self.s_start:
            return 0
        self.km += self._h(self.s_last, new_start)
        self.s_start = new_start
        self.s_last = new_start
        return self.compute_shortest_path()

    def update_cells(self, changed):
        """Some cells changed occupancy. Repair only what that can have moved.

        Every edge incident on a changed cell has changed cost, so the changed
        cell and each of its 8-neighbours are the states whose `rhs` can have
        moved -- and `compute_shortest_path` propagates outward from there for
        exactly as far as the change actually reaches. That bound is the reason
        this is cheaper than replanning, and it is also the reason the answer is
        the same one a full replan would give: nothing outside the affected
        region needed to change.
        """
        touched = set()
        for c in changed:
            c = tuple(c)
            if not self.cmap.in_grid(*c):
                continue
            touched.add(c)
            cx, cy = c
            for dx, dy in NEIGHBOURS:
                n = (cx + dx, cy + dy)
                if self.cmap.in_grid(*n):
                    touched.add(n)
        for s in sorted(touched):
            self.update_vertex(s)
        return self.compute_shortest_path()

    def path_cells(self, max_len=100000):
        """The current best route as cells, start to goal, or None.

        Greedy descent on `c(s, s') + g(s')`, which is what D* Lite guarantees is
        correct after `compute_shortest_path` -- the g-values off the optimal
        region may be stale, but they are never the minimum, so they are never
        chosen. Ties break on the cell index, deterministically.
        """
        if self._g(self.s_start) == self.INF and self._rhs(self.s_start) == self.INF:
            return None
        out = [self.s_start]
        s = self.s_start
        seen = {s}
        while s != self.s_goal:
            best, best_c = None, self.INF
            for s2 in self._neighbours(s):
                c = self.cost(s, s2)
                if c == self.INF:
                    continue
                v = c + self._g(s2)
                if v == self.INF:
                    continue
                if v < best_c - 1e-9 or (best is not None
                                         and abs(v - best_c) <= 1e-9
                                         and s2 < best):
                    best, best_c = s2, v
            if best is None or best_c == self.INF:
                return None
            if best in seen:
                return None                  # a cycle means the values are junk
            seen.add(best)
            out.append(best)
            s = best
            if len(out) > max_len:
                return None
        return out

    def path_cost(self):
        """Cost from the current start to the goal, or None if there is none.

        `min(g, rhs)`, NOT `g`. `compute_shortest_path` is entitled to stop with
        the start merely locally consistent -- its termination test is
        `rhs(s_start) <= g(s_start)`, which an over-consistent start satisfies
        with `g` still at infinity. Reading `g` alone reports "no route" for a
        route that exists, which is the same class of error as an honest abort
        and much harder to see. `min` is also exactly what `_key` reads.
        """
        v = min(self._g(self.s_start), self._rhs(self.s_start))
        return None if v == self.INF else v


def dstar_points(dstar, cmap, start_xy, goal_xy):
    """A D* Lite cell path as arena millimetres, with the TRUE endpoints."""
    cells = dstar.path_cells()
    if cells is None:
        return None
    pts = [(float(start_xy[0]), float(start_xy[1]))]
    for c in cells[1:-1]:
        pts.append(cmap.centre_of(*c))
    pts.append((float(goal_xy[0]), float(goal_xy[1])))
    return pts


# ======================================================== ROUTE TIDYING
def path_cost_mm(pts, heading_deg, cmap=None):
    """What a whole route costs THIS rover. One scorer, used everywhere.

    Distance, plus a turn charged at every vertex INCLUDING the first, plus the
    proximity and prior bribes when a costmap is supplied. The first turn is as
    real as any other, and a planner that ignored it would happily propose a
    route that starts by spinning 170 degrees to save 20 mm.
    """
    if not pts:
        return 0.0
    total = 0.0
    hdg = math.radians(float(heading_deg))
    px, py = pts[0]
    for (qx, qy) in pts[1:]:
        d = math.hypot(qx - px, qy - py)
        if d <= 1e-9:
            continue
        b = math.atan2(qy - py, qx - px)
        total += turn_cost_mm(wrap_pi(b - hdg))
        total += (cmap.segment_cost_mm(px, py, qx, qy) if cmap is not None else d)
        hdg = b
        px, py = qx, qy
    return total


def shortcut_points(pts, cmap, heading_deg):
    """Greedy line-of-sight string-pull. Drop any vertex the route does not need.

    Theta* only ever compares a node against its immediate parent, so it can
    leave a bend in that a longer look-back would have removed; A* and D* Lite
    leave whole staircases. Every removal is JUSTIFIED against `path_cost_mm`
    rather than against length alone, because on this chassis a shortcut that
    removes a bend but adds an awkward angle can genuinely be worse -- and every
    removal is GUARDED by `segment_ok`, because the straight line that replaces a
    vertex was never checked by the search.
    """
    if len(pts) <= 2:
        return [tuple(p) for p in pts]
    out = [tuple(pts[0])]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1:
            if cmap.segment_ok(pts[i][0], pts[i][1], pts[j][0], pts[j][1]):
                direct = path_cost_mm([pts[i], pts[j]], heading_deg
                                      if i == 0 else
                                      bearing_deg(pts[i][0] - pts[i - 1][0],
                                                  pts[i][1] - pts[i - 1][1]),
                                      cmap)
                via = path_cost_mm(list(pts[i:j + 1]), heading_deg
                                   if i == 0 else
                                   bearing_deg(pts[i][0] - pts[i - 1][0],
                                               pts[i][1] - pts[i - 1][1]),
                                   cmap)
                if direct <= via + 1e-9:
                    break
            j -= 1
        out.append(tuple(pts[j]))
        i = j
    return out


def prune_via(route, cmap, min_leg_mm=MIN_LEG_MM, min_turn_deg=MIN_TURN_DEG,
              via_tol_mm=VIA_TOL_MM):
    """Drop waypoints this chassis cannot act on. Every removal LOS-guarded.

    A SEARCH ANSWERS IN GEOMETRY; A ROVER ANSWERS IN MOTIONS IT CAN PERFORM.
    Three kinds of vertex are geometrically real and physically fictional:

      * A BEND SMALLER THAN THE SMALLEST TURN THIS CHASSIS CAN MAKE. This one
        actually bit, and it fails SILENTLY: the follower emits no turn under
        `min_turn_deg`, so the rover drives the following leg along the OLD
        heading and the route ends somewhere that looks like odometry drift.
      * ONE CLOSER TO THE TARGET THAN THE FOLLOWER'S RETIREMENT TOLERANCE, which
        the follower discards on proximity anyway.
      * ONE CLOSER TO ITS PREDECESSOR THAN THE SHORTEST MOTION THE CHASSIS WILL
        EXPRESS (`min_leg_mm`, golden's 30 mm) -- a leg that cannot move is an
        oscillation source, not a route.

    Pruning must never be the thing that puts the chassis into an obstacle, so a
    vertex stays if the straight line replacing it is not clear.
    """
    if len(route) <= 2:
        return [tuple(p) for p in route]
    out = [tuple(route[0])]
    for i in range(1, len(route) - 1):
        p, v, nxt = out[-1], tuple(route[i]), tuple(route[i + 1])
        bend = abs(math.degrees(wrap_pi(
            math.atan2(nxt[1] - v[1], nxt[0] - v[0])
            - math.atan2(v[1] - p[1], v[0] - p[0]))))
        drop = (bend < min_turn_deg
                or math.hypot(v[0] - route[-1][0], v[1] - route[-1][1]) < via_tol_mm
                or math.hypot(v[0] - p[0], v[1] - p[1]) < min_leg_mm)
        if drop and cmap.segment_ok(p[0], p[1], nxt[0], nxt[1]):
            continue
        out.append(v)
    out.append(tuple(route[-1]))
    return out


# ================================================================ SEGMENTS
@dataclass
class Segment:
    """One commanded motion: turn `turn_deg`, then drive `drive_mm`.

    `turn_deg` IS THE PLANNED ROTATION AND IT IS HONEST. The executor is given a
    rotation budget that aborts on unplanned turning, so a straight run carries
    EXACTLY 0.0 here -- not a small number, not a rounding residue. If this field
    says 0.0 and the rover rotates, that is a fault and the budget should catch
    it.
    """
    turn_deg: float = 0.0
    drive_mm: float = 0.0
    x0_mm: float = 0.0
    y0_mm: float = 0.0
    x1_mm: float = 0.0
    y1_mm: float = 0.0
    heading_deg: float = 0.0     # heading DURING the drive, after the turn
    index: int = 0

    def as_dict(self):
        return {"turn_deg": round(self.turn_deg, 3),
                "drive_mm": round(self.drive_mm, 2),
                "heading_deg": round(self.heading_deg, 3),
                "from": [round(self.x0_mm, 2), round(self.y0_mm, 2)],
                "to": [round(self.x1_mm, 2), round(self.y1_mm, 2)],
                "index": self.index}


def route_to_segments(route, heading_deg, min_leg_mm=MIN_LEG_MM,
                      min_turn_deg=MIN_TURN_DEG):
    """[(x, y), ...] -> [Segment, ...]. Golden `_p5_navdrive` 1403-1412.

    Golden's rule verbatim: `if sd < 30: continue` -- a leg the chassis cannot
    express is dropped rather than commanded, because commanding it produces an
    oscillation that looks like a random turn.

    ONE CORRECTION TO GOLDEN, and it is a correction not a rewrite. Golden
    measured every leg from `path[i-1]` whether or not the previous leg was
    dropped, so a dropped 20 mm hop silently deleted 20 mm of route. Here the
    origin does NOT advance when a leg is dropped, so the next leg absorbs the
    displacement and the geometry is preserved. A short leg at the very END has
    nothing to merge into and is dropped, which moves the endpoint by less than
    `min_leg_mm` -- that residue is the dock's job, not the planner's.

    A turn under `min_turn_deg` is emitted as EXACTLY 0.0 and the heading is left
    where it was, because the follower will not command it either: recording a
    turn the rover will not make is how a plan and a drive drift apart.
    """
    segs = []
    if not route or len(route) < 2:
        return segs
    hdg = float(heading_deg)
    px, py = float(route[0][0]), float(route[0][1])
    last_bearing = None
    for k in range(1, len(route)):
        qx, qy = float(route[k][0]), float(route[k][1])
        d = math.hypot(qx - px, qy - py)
        if d < min_leg_mm:
            continue                        # merge forward; do NOT advance p
        b = bearing_deg(qx - px, qy - py)
        # EXACTLY COLLINEAR CONTINUATIONS ARE ONE MOTION, not two. A search can
        # emit three vertices along one straight line; commanding them as three
        # bursts means two extra stop-settle-stops for no change of direction.
        # The test is EXACT equality of bearing, not "within the turn deadband":
        # merging two runs that differ by three degrees would quietly move the
        # endpoint, which is the opposite of what this function is for.
        if (last_bearing is not None and segs
                and abs(wrap180(b - last_bearing)) <= 1e-9):
            s = segs[-1]
            s.drive_mm += d
            s.x1_mm, s.y1_mm = qx, qy
            px, py = qx, qy
            continue
        turn = heading_error_deg(b, hdg)
        if abs(turn) < min_turn_deg:
            turn = 0.0                      # honest: the follower emits nothing
        else:
            hdg = wrap180(hdg + turn)
        segs.append(Segment(turn_deg=turn, drive_mm=d, x0_mm=px, y0_mm=py,
                            x1_mm=qx, y1_mm=qy, heading_deg=hdg,
                            index=len(segs)))
        px, py = qx, qy
        last_bearing = b
    return segs


# ==================================================================== PLAN
@dataclass
class Plan:
    """What the planner decided, and enough about HOW to explain it.

    `points` follows the three-answer contract every caller already depends on:
        []      the straight line is clear -- no waypoints needed
        [...]   a detour, via these intermediate waypoints
        None    NO ROUTE. The caller's abort hangs off this and nothing else.
    """
    ok: bool = False
    points: list = None                  # intermediate waypoints only
    route: list = field(default_factory=list)   # full polyline incl. both ends
    segments: list = field(default_factory=list)
    planner: str = "straight"
    cost_mm: float = 0.0
    distance_mm: float = 0.0
    rotation_deg: float = 0.0            # total PLANNED rotation, for the budget
    searched: bool = False
    repaired: bool = False               # D* Lite repaired rather than replanned
    expansions: int = 0
    obstacle_cells: int = 0
    wall_cells: int = 0
    prior_cells: int = 0
    note: str = None
    reason: str = None                   # why there is no route, when there is none

    def as_dict(self):
        return {"ok": self.ok, "planner": self.planner,
                "waypoints": [[round(p[0], 1), round(p[1], 1)]
                              for p in (self.points or [])],
                "route": [[round(p[0], 1), round(p[1], 1)] for p in self.route],
                "segments": [s.as_dict() for s in self.segments],
                "cost_mm": round(self.cost_mm, 1),
                "distance_mm": round(self.distance_mm, 1),
                "rotation_deg": round(self.rotation_deg, 1),
                "searched": self.searched, "repaired": self.repaired,
                "expansions": self.expansions,
                "obstacle_cells": self.obstacle_cells,
                "wall_cells": self.wall_cells, "prior_cells": self.prior_cells,
                "note": self.note, "reason": self.reason}


# THE RELAXATION LADDER, top rung first: (name, wall pad scale, radius scale).
# The wall pad is a policy about a boundary that is known and static, so it is
# relaxed FIRST; the obstacle radius is the rover's actual half-width and is
# relaxed LAST and LEAST. THE LADDER CANNOT REACH ZERO -- there is no rung that
# removes padding, because "no route at any margin" and "a route only if the
# chassis is allowed to collide" are the same answer as far as the rover is
# concerned.
RELAX_LADDER = (("none", 1.0, 1.0),
                ("wall", 0.5, 1.0),
                ("wall+radius", 0.5, 0.8))


class Planner:
    """The whole route decision for one leg. PURE, deterministic, no hardware.

    ORDER OF BUSINESS, and every step is a refusal to do something clever:
      1. NO GRID, OR NOTHING BELIEVED -> the straight line, untouched. On a clean
         arena this class must be a no-op, and that is checked first.
      2. THE STRAIGHT LINE IS STILL CLEAR -> the straight line. Checked against
         the CONTINUOUS costmap, so "clear" means the line the rover would really
         drive, not the cells it would pass through.
      3. SEARCH, down the relaxation ladder until something is found.
      4. SIMPLIFY: Douglas-Peucker (golden), then line-of-sight shortcutting,
         then prune the vertices the chassis cannot act on.
      5. VALIDATE, and refuse rather than publish a route that fails.

    A route is only ever returned after `segment_ok` has confirmed every one of
    its legs on the costmap that produced it. A search that returns a route its
    own map calls undrivable is a bug, and this is where it is caught rather than
    at the wheels.
    """

    def __init__(self, arena_w_mm=ARENA_W_MM, arena_h_mm=ARENA_H_MM,
                 cell_mm=CELL_MM, robot_radius_mm=ROBOT_RADIUS_MM,
                 inflation_mm=INFLATION_MM, wall_pad_mm=None,
                 point_pad_mm=OCC_POINT_PAD_MM, min_leg_mm=MIN_LEG_MM,
                 min_turn_deg=MIN_TURN_DEG, dp_epsilon_mm=DP_EPSILON_MM,
                 start_clear_mm=START_CLEAR_MM, planner="theta",
                 relax=True, prior=None):
        self.arena_w_mm = float(arena_w_mm)
        self.arena_h_mm = float(arena_h_mm)
        self.cell_mm = float(cell_mm)
        self.robot_radius_mm = float(robot_radius_mm)
        self.inflation_mm = float(inflation_mm)
        self.wall_pad_mm = (self.robot_radius_mm if wall_pad_mm is None
                            else float(wall_pad_mm))
        self.point_pad_mm = float(point_pad_mm)
        self.min_leg_mm = float(min_leg_mm)
        self.min_turn_deg = float(min_turn_deg)
        self.dp_epsilon_mm = float(dp_epsilon_mm)
        self.start_clear_mm = float(start_clear_mm)
        self.planner = str(planner).strip().lower()
        if self.planner not in ("theta", "astar", "dstar", "straight"):
            self.planner = "theta"
        self.relax = bool(relax)
        self.prior = prior
        self.nx = max(1, int(math.ceil(self.arena_w_mm / self.cell_mm)))
        self.ny = max(1, int(math.ceil(self.arena_h_mm / self.cell_mm)))

    # -- construction ------------------------------------------------------
    def costmap(self, points, free=(), wall_scale=1.0, radius_scale=1.0):
        return CostMap(points, self.nx, self.ny, self.cell_mm,
                       arena_w_mm=self.arena_w_mm, arena_h_mm=self.arena_h_mm,
                       radius_mm=self.robot_radius_mm * radius_scale,
                       wall_pad_mm=self.wall_pad_mm * wall_scale,
                       point_pad_mm=self.point_pad_mm,
                       inflation_mm=self.inflation_mm,
                       free=free, prior=self.prior)

    def grid(self, **kw):
        """A matching OccupancyGrid. Same arena, same cell size, same filters."""
        kw.setdefault("cell_mm", self.cell_mm)
        kw.setdefault("arena_w_mm", self.arena_w_mm)
        kw.setdefault("arena_h_mm", self.arena_h_mm)
        return OccupancyGrid(**kw)

    # -- validation --------------------------------------------------------
    def validate(self, route, cmap, start_xy):
        """(ok, reason). The last gate before a route is published.

        Three refusals, and each one is a failure mode that has cost a session:
          * a route that LEAVES THE ARENA. Off the field is not an escape route.
          * a route whose intermediate waypoints double back through THE START
            BOX. The start cell is deliberately forced passable so the rover can
            always plan its way out of wherever it is parked -- that escape hatch
            must not become a licence to route back through it.
          * a leg the costmap that produced it will not certify.
        """
        if not route or len(route) < 2:
            return False, "route has no legs"
        for (x, y) in route:
            if not (0.0 <= x <= self.arena_w_mm and 0.0 <= y <= self.arena_h_mm):
                return False, ("route leaves the arena at (%.0f, %.0f); the box "
                               "is %.0f x %.0f" % (x, y, self.arena_w_mm,
                                                   self.arena_h_mm))
        sx, sy = float(start_xy[0]), float(start_xy[1])
        for (x, y) in route[1:-1]:
            if math.hypot(x - sx, y - sy) < self.start_clear_mm:
                return False, ("route doubles back through the start box "
                               "(within %.0f mm of the start pose)"
                               % self.start_clear_mm)
        for a, b in zip(route, route[1:]):
            if not cmap.segment_ok(a[0], a[1], b[0], b[1]):
                return False, ("a leg of the route is not drivable on the map "
                               "that produced it: (%.0f, %.0f) -> (%.0f, %.0f)"
                               % (a[0], a[1], b[0], b[1]))
        return True, None

    def tidy(self, route, cmap, heading_deg):
        """Douglas-Peucker (golden), then LOS shortcut, then prune. In that order.

        DP FIRST because it is cheap and removes the staircase bulk; the LOS pass
        then only has to consider the few vertices that survived, and it is the
        expensive one. Pruning last, because it is the only pass that reasons
        about what the CHASSIS can do rather than about geometry.
        """
        route = [tuple(p) for p in route]
        if len(route) > 2:
            # DP works on perpendicular distance and knows NOTHING about
            # obstacles, so a corner it straightens can cut straight through
            # one. Every straightened run is re-checked and the simplification
            # is taken ONLY IF ALL of it is drivable -- otherwise the
            # un-simplified route goes on to the shortcutter, which removes the
            # same redundancy with a line-of-sight guard on every removal.
            #
            # (Getting this wrong is not theoretical: the first version returned
            # the DP output on failure instead of the original, which published
            # a route through the obstacle -- caught by `validate`, which then
            # relaxed the padding for a route that never needed relaxing.)
            simple = douglas_peucker(route, self.dp_epsilon_mm)
            if all(cmap.segment_ok(a[0], a[1], b[0], b[1])
                   for a, b in zip(simple, simple[1:])):
                route = simple
        route = shortcut_points(route, cmap, heading_deg)
        return prune_via(route, cmap, self.min_leg_mm, self.min_turn_deg)

    # -- the search --------------------------------------------------------
    def _search(self, cmap, start_xy, goal_xy, heading_deg, which, dstar=None):
        if which == "astar":
            return astar(cmap, start_xy, goal_xy), None
        if which == "dstar":
            s = cmap.cell_of(*start_xy)
            g = cmap.cell_of(*goal_xy)
            if not (cmap.in_grid(*s) and cmap.in_grid(*g)):
                return None, None
            if not cmap.safe(*goal_xy):
                return None, None
            d = dstar if dstar is not None else DStarLite(cmap, s, g, free=(s, g))
            return dstar_points(d, cmap, start_xy, goal_xy), d
        return theta_star(cmap, start_xy, goal_xy, heading_deg), None

    def plan(self, x_mm, y_mm, heading_deg, tx_mm, ty_mm, grid=None, now=None,
             planner=None, points=None, dstar=None):
        """Plan one leg. Returns a `Plan`. Never raises on bad map data.

        `points` lets a caller supply the believed surfaces directly (a test, or
        a caller that already has them); otherwise they come from `grid`.
        `dstar` lets `IncrementalPlanner` hand in a live D* Lite instance so the
        search is REPAIRED rather than rebuilt.
        """
        use = (planner or self.planner)
        out = Plan(ok=True, points=[], planner="straight")
        sx, sy = float(x_mm), float(y_mm)
        gx, gy = float(tx_mm), float(ty_mm)
        heading_deg = float(heading_deg)

        straight = [(sx, sy), (gx, gy)]
        out.route = straight
        out.segments = route_to_segments(straight, heading_deg, self.min_leg_mm,
                                         self.min_turn_deg)
        out.distance_mm = math.hypot(gx - sx, gy - sy)
        out.cost_mm = path_cost_mm(straight, heading_deg)
        out.rotation_deg = sum(abs(s.turn_deg) for s in out.segments)
        out.prior_cells = len(self.prior) if self.prior else 0

        if use == "straight":
            return out
        if points is None:
            points = grid.points(now=now) if grid is not None else None
        if not points:
            return out                        # (1) nothing believed: no-op
        out.wall_cells = sum(1 for p in points if len(p) > 2 and p[2])
        out.obstacle_cells = len(points) - out.wall_cells

        here = (int(math.floor(sx / self.cell_mm)),
                int(math.floor(sy / self.cell_mm)))
        goal_cell = (int(math.floor(gx / self.cell_mm)),
                     int(math.floor(gy / self.cell_mm)))
        base = None
        chosen = None
        for (name, wall_s, rad_s) in (RELAX_LADDER if self.relax
                                      else RELAX_LADDER[:1]):
            if dstar is not None and name == "none":
                # THE INCREMENTAL SEARCH AND THE VALIDATION MUST SHARE ONE MAP.
                # D* Lite's g-values were computed against `dstar.cmap`; building
                # a second, nominally identical costmap here and validating
                # against that one is how a repaired route gets rejected for
                # disagreeing with a map it was never planned on.
                cmap = dstar.cmap
            else:
                cmap = self.costmap(points, free=(here,), wall_scale=wall_s,
                                    radius_scale=rad_s)
            if base is None:
                base = cmap
                if cmap.segment_ok(sx, sy, gx, gy):
                    return out                # (2) the line is still clear
            out.searched = True
            route, used_d = self._search(cmap, (sx, sy), (gx, gy), heading_deg,
                                         use, dstar=dstar if name == "none"
                                         else None)
            if route is None:
                continue
            route = self.tidy(route, cmap, heading_deg)
            ok, why = self.validate(route, cmap, (sx, sy))
            if not ok:
                out.reason = why
                continue
            chosen = (name, cmap, route, used_d)
            break

        if chosen is None:
            out.ok = False
            out.points = None                 # NO ROUTE: the abort hangs here
            out.planner = use
            out.reason = out.reason or (
                "%s found no clear route to (%.0f, %.0f) around what the LiDAR "
                "has seen%s" % (use, gx, gy,
                                ", at any padding on the relaxation ladder"
                                if self.relax else ""))
            return out

        name, cmap, route, used_d = chosen
        out.planner = use
        out.route = route
        out.points = [(round(p[0], 3), round(p[1], 3)) for p in route[1:-1]]
        out.segments = route_to_segments(route, heading_deg, self.min_leg_mm,
                                         self.min_turn_deg)
        out.rotation_deg = sum(abs(s.turn_deg) for s in out.segments)
        out.distance_mm = sum(s.drive_mm for s in out.segments)
        out.cost_mm = path_cost_mm(route, heading_deg, cmap)
        if used_d is not None:
            out.expansions = used_d.expansions
        if name != "none":
            out.note = ("no route existed at full margin; this one needed the "
                        "%s padding relaxed and is drawn on that basis"
                        % name.replace("+", " and the "))
        elif out.points:
            out.note = ("%s detour via %d waypoint(s) around %d obstacle cell(s)"
                        % (use, len(out.points), out.obstacle_cells))
            if out.wall_cells:
                out.note += ("; %d further cell(s) were the arena wall and are "
                             "handled by the wall pad, not routed around"
                             % out.wall_cells)
        return out


class IncrementalPlanner:
    """One leg, driven, with D* LITE REPAIRING THE PLAN AS THE MAP CHANGES.

    THIS IS THE THING D* IS NAMED FOR. `Planner.plan` answers once. This holds
    the search alive for the whole leg, so when an obstacle appears while the
    rover is driving the plan is REPAIRED from where the rover is NOW -- the
    km-offset re-key plus a bounded re-expansion around the change -- instead of
    a fresh whole-arena search that has no memory of what it already decided.

    USE:
        inc = IncrementalPlanner(planner, grid, (x, y, hdg), (tx, ty))
        plan = inc.plan()                       # the first, full plan
        ...rover drives...
        plan = inc.update((x, y, hdg))          # cheap: nothing changed
        ...LiDAR marks an obstacle...
        plan = inc.update((x, y, hdg))          # REPAIRED, not replanned

    WHEN IT REBUILDS INSTEAD OF REPAIRING, and it says so in `plan.repaired`:
      * the target changed (a different goal is a different search);
      * the believed surfaces changed enough to move which CELLS are blocked, in
        a way that also changed the costmap's inflation -- the costmap is rebuilt
        for that, and D* Lite is told exactly which cells flipped;
      * the relaxation ladder had to be used, because a rung is a different map.
    A rebuild is always correct and never silent; a repair is the fast path.
    """

    def __init__(self, planner, grid, pose, target, now=None):
        self.p = planner
        self.grid = grid
        self.target = (float(target[0]), float(target[1]))
        self.pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        self._now = now
        self._d = None
        self._cmap = None
        self._blocked = set()
        self._revision = None
        self.rebuilds = 0
        self.repairs = 0

    def _points(self):
        return self.grid.points(now=self._now) if self.grid is not None else []

    def _rebuild(self):
        pts = self._points()
        here = (int(math.floor(self.pose[0] / self.p.cell_mm)),
                int(math.floor(self.pose[1] / self.p.cell_mm)))
        goal = (int(math.floor(self.target[0] / self.p.cell_mm)),
                int(math.floor(self.target[1] / self.p.cell_mm)))
        self._cmap = self.p.costmap(pts, free=(here,))
        self._blocked = set(self._cmap.blocked)
        self._prox = dict(self._cmap._prox)
        if (self._cmap.in_grid(*here) and self._cmap.in_grid(*goal)
                and self._cmap.safe(*self.target)):
            self._d = DStarLite(self._cmap, here, goal,
                                free=set(self._cmap.free) | {here, goal})
        else:
            self._d = None
        self._revision = getattr(self.grid, "revision", None)
        self.rebuilds += 1

    def plan(self, planner=None):
        """The first plan, or a full one. Always correct, never incremental."""
        self._rebuild()
        pl = self.p.plan(self.pose[0], self.pose[1], self.pose[2],
                         self.target[0], self.target[1], grid=self.grid,
                         now=self._now, planner=planner, dstar=self._d)
        pl.repaired = False
        return pl

    def update(self, pose, now=None, planner=None):
        """The rover moved and/or the map changed. Repair if we can, rebuild if
        we must. Returns a fresh `Plan` either way."""
        self.pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        if now is not None:
            self._now = now
        rev = getattr(self.grid, "revision", None)
        here = (int(math.floor(self.pose[0] / self.p.cell_mm)),
                int(math.floor(self.pose[1] / self.p.cell_mm)))
        repaired = False
        if self._d is None or self._cmap is None:
            self._rebuild()
        elif rev != self._revision or here not in self._cmap.free:
            # Either the believed surfaces moved, or the rover has driven into a
            # different cell -- and the second one changes the map too, because
            # the "cell the rover is standing in is always passable" hatch moves
            # with it. Rebuild the COSTMAP (inflation is continuous and cannot be
            # patched cell by cell), then tell D* Lite only which CELLS changed,
            # which is the incremental part and the only part that costs
            # anything.
            pts = self._points()
            cmap = self.p.costmap(pts, free=(here,))
            # THE CHANGE SET MUST COVER EVERY EDGE WHOSE COST MOVED, not only
            # the cells that flipped blocked. `cell_step_cost` also charges the
            # soft proximity layer, and rebuilding the costmap moves that layer
            # wherever the new surface is within `inflation_mm` -- and wherever
            # the "cell the rover is standing in is always free" hatch has moved
            # with the rover. A change set that missed those would leave D* Lite
            # holding g-values from a graph that no longer exists, and its repair
            # would silently stop matching a full replan. That is exactly the
            # bug the repair-equals-replan test exists to catch.
            changed = set(cmap.blocked ^ self._blocked)
            for c, v in cmap._prox.items():
                if v != self._prox.get(c):
                    changed.add(c)
            self._cmap = cmap
            self._blocked = set(cmap.blocked)
            self._prox = dict(cmap._prox)
            self._revision = rev
            self._d.cmap = cmap
            self._d.free = set(cmap.free) | {self._d.s_goal}
            # ORDER MATTERS. `move_start` re-keys the queue with km against the
            # NEW start; `update_cells` then repairs against those keys. Doing it
            # the other way round would repair against keys measured from where
            # the rover used to be.
            self._d.move_start(here)
            if changed:
                self._d.update_cells(changed)
            self.repairs += 1
            repaired = True
        else:
            self._d.move_start(here)
            self.repairs += 1
            repaired = True
        pl = self.p.plan(self.pose[0], self.pose[1], self.pose[2],
                         self.target[0], self.target[1], grid=self.grid,
                         now=self._now, planner=planner, dstar=self._d)
        pl.repaired = repaired and pl.searched
        return pl


# ============================================== GOLDEN B6, FAITHFULLY PORTED
class GoldenB6:
    """Golden's LIVE planner (fpms_phase6_M1_M2_WORKING.py:525-775), ported.

    THIS IS THE FIDELITY REFERENCE, not the live path. It exists so "exactly
    B8B's and golden's A*" is a claim that can be TESTED rather than asserted:
    `fpms_planner_test.py` runs a verbatim copy of golden's own functions
    side by side with this one over a deterministic battery of scans and targets
    and requires identical output.

    Everything golden hardcoded as a module global is a constructor argument
    here, with golden's value as the default, so the class can also be pointed at
    the 1500 x 1200 arena. Nothing else is changed: the same robot-relative
    frame, the same 100 mm cells, the same flat ROUTE_RADIUS disc, the same
    corner-cutting 8-connected A* with the same Euclidean heuristic and the same
    x-band edge penalty, the same `nearest_free` ring search, the same
    Douglas-Peucker epsilon, and the same straight-if-nothing-blocks decision.

    The x-band edge penalty (`x_mm > 500 -> +5.0`, `x_mm < 100 -> +0.5` per cell
    entered) is golden's, and it is ARENA-SPECIFIC: it biases routes towards the
    left of a 650 mm-wide robot-relative box. It is ported because the fidelity
    claim requires it, and it is OFF by default in the live arena planner because
    the arena it describes is not the arena we have.
    """

    def __init__(self, world_x_min=-800.0, world_x_max=800.0,
                 world_y_min=-400.0, world_y_max=1400.0, grid_res=100.0,
                 route_radius=175.0, start_clear_radius=150.0,
                 robot_footprint_mm=140.0, target_exclude_mm=220.0,
                 target_stop_raw=300.0, dp_epsilon=150.0,
                 arena_x=(0.0, 650.0), arena_y=(0.0, 1000.0),
                 clip_x=(-20.0, 670.0), clip_y=(-20.0, 1020.0),
                 edge_penalty=True):
        self.WORLD_X_MIN = float(world_x_min)
        self.WORLD_X_MAX = float(world_x_max)
        self.WORLD_Y_MIN = float(world_y_min)
        self.WORLD_Y_MAX = float(world_y_max)
        self.WORLD_W = self.WORLD_X_MAX - self.WORLD_X_MIN
        self.WORLD_H = self.WORLD_Y_MAX - self.WORLD_Y_MIN
        self.GRID_RES = float(grid_res)
        self.ROUTE_RADIUS = float(route_radius)
        self.START_CLEAR_RADIUS = float(start_clear_radius)
        self.P5_ROBOT_FOOTPRINT_MM = float(robot_footprint_mm)
        self.P5_TARGET_EXCLUDE_MM = float(target_exclude_mm)
        self.TARGET_STOP_RAW = float(target_stop_raw)
        self.DP_EPSILON = float(dp_epsilon)
        self.arena_x = (float(arena_x[0]), float(arena_x[1]))
        self.arena_y = (float(arena_y[0]), float(arena_y[1]))
        self.clip_x = (float(clip_x[0]), float(clip_x[1]))
        self.clip_y = (float(clip_y[0]), float(clip_y[1]))
        self.edge_penalty_on = bool(edge_penalty)

    # -- golden 525-536 ----------------------------------------------------
    def xy_cell(self, x, y):
        return (int(round((x - self.WORLD_X_MIN) / self.GRID_RES)),
                int(round((y - self.WORLD_Y_MIN) / self.GRID_RES)))

    def cell_xy(self, c, r):
        return (c * self.GRID_RES + self.WORLD_X_MIN,
                r * self.GRID_RES + self.WORLD_Y_MIN)

    def grid_size(self):
        return (int(self.WORLD_W / self.GRID_RES) + 1,
                int(self.WORLD_H / self.GRID_RES) + 1)

    # -- golden 550-610 ----------------------------------------------------
    def planning_obstacles(self, pts, pose, target_marker=None, keepouts=()):
        """Self-filter (circle), arena clip, target exclusion, keepout injection.

        `pose` is (x, y, theta) -- golden read it from `_p5_odom` under a lock;
        it is a parameter here so the function is pure. The world transform is
        golden's, INCLUDING its sign convention:
            wx = px + x*cos(t) + y*sin(t)
            wy = py - x*sin(t) + y*cos(t)
        """
        obs = []
        fp_sq = self.P5_ROBOT_FOOTPRINT_MM * self.P5_ROBOT_FOOTPRINT_MM
        prx, pry, prt = float(pose[0]), float(pose[1]), float(pose[2])
        for p in pts:
            x, y = p["x"], p["y"]
            if x * x + y * y < fp_sq:
                continue                      # self-filter
            wx = prx + x * math.cos(prt) + y * math.sin(prt)
            wy = pry - x * math.sin(prt) + y * math.cos(prt)
            if (wx < self.clip_x[0] or wx > self.clip_x[1]
                    or wy < self.clip_y[0] or wy > self.clip_y[1]):
                continue                      # arena clip
            if target_marker:
                dx = x - target_marker["x"]
                dy = y - target_marker["y"]
                if (dx * dx + dy * dy
                        < self.P5_TARGET_EXCLUDE_MM * self.P5_TARGET_EXCLUDE_MM):
                    continue                  # TARGET EXCLUSION -- docking
            obs.append(p)
        for ko in keepouts:
            kwx = ko.get("x", ko.get("wx", 0))
            kwy = ko.get("y", ko.get("wy", 0))
            kr = ko.get("r", 150)
            step = 60
            for ddx in range(-int(kr), int(kr) + 1, step):
                for ddy in range(-int(kr), int(kr) + 1, step):
                    if ddx * ddx + ddy * ddy <= kr * kr:
                        wx, wy = kwx + ddx, kwy + ddy
                        dwx, dwy = wx - prx, wy - pry
                        rx = dwx * math.cos(-prt) - dwy * math.sin(-prt)
                        ry = dwx * math.sin(-prt) + dwy * math.cos(-prt)
                        obs.append({"x": rx, "y": ry,
                                    "d": math.hypot(rx, ry), "deg": 0, "i": 0,
                                    "keepout": True})
        return obs

    # -- golden 613-651 ----------------------------------------------------
    def build_costmap(self, obs, pose):
        cols, rows = self.grid_size()
        occ = [[False] * cols for _ in range(rows)]
        rad = int(math.ceil(self.ROUTE_RADIUS / self.GRID_RES))
        for p in obs:
            c0, r0 = self.xy_cell(p["x"], p["y"])
            for dr in range(-rad, rad + 1):
                rr = r0 + dr
                if rr < 0 or rr >= rows:
                    continue
                for dc in range(-rad, rad + 1):
                    cc = c0 + dc
                    if cc < 0 or cc >= cols:
                        continue
                    if math.hypot(dc * self.GRID_RES,
                                  dr * self.GRID_RES) <= self.ROUTE_RADIUS:
                        occ[rr][cc] = True
        bx, by, bt = float(pose[0]), float(pose[1]), float(pose[2])
        for r in range(rows):
            for c in range(cols):
                rx = c * self.GRID_RES + self.WORLD_X_MIN
                ry = r * self.GRID_RES + self.WORLD_Y_MIN
                wx = bx + rx * math.cos(bt) + ry * math.sin(bt)
                wy = by - rx * math.sin(bt) + ry * math.cos(bt)
                if (wx < self.arena_x[0] or wx > self.arena_x[1]
                        or wy < self.arena_y[0] or wy > self.arena_y[1]):
                    occ[r][c] = True
        sc, sr = self.xy_cell(0, 0)
        clear = int(math.ceil(self.START_CLEAR_RADIUS / self.GRID_RES))
        for dr in range(-clear, clear + 1):
            for dc in range(-clear, clear + 1):
                rr, cc = sr + dr, sc + dc
                if (0 <= rr < rows and 0 <= cc < cols
                        and math.hypot(dc * self.GRID_RES, dr * self.GRID_RES)
                        <= self.START_CLEAR_RADIUS):
                    occ[rr][cc] = False
        return occ

    # -- golden 654-667 ----------------------------------------------------
    def nearest_free(self, occ, cell, max_rad=8):
        cols, rows = self.grid_size()
        c0, r0 = cell
        if 0 <= c0 < cols and 0 <= r0 < rows and not occ[r0][c0]:
            return cell
        for rad in range(1, max_rad + 1):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    if abs(dc) != rad and abs(dr) != rad:
                        continue
                    c, r = c0 + dc, r0 + dr
                    if 0 <= c < cols and 0 <= r < rows and not occ[r][c]:
                        return c, r
        return None

    # -- golden 670-709 ----------------------------------------------------
    def edge_penalty(self, cell):
        if not self.edge_penalty_on:
            return 0.0
        x_mm = cell[0] * self.GRID_RES + self.WORLD_X_MIN
        if x_mm > 500:
            return 5.0
        if x_mm < 100:
            return 0.5
        return 0.0

    def astar(self, occ, start_xy, goal_xy):
        cols, rows = self.grid_size()
        start = self.nearest_free(occ, self.xy_cell(*start_xy), 10)
        goal = self.nearest_free(occ, self.xy_cell(*goal_xy), 10)
        if not start or not goal:
            return None
        # Golden's moves order. It is load-bearing: `came` is written only on a
        # STRICT improvement, so when two routes tie the successor generated
        # first wins, and a different order gives a different (equal-cost) path.
        moves = ((-1, 0), (1, 0), (0, -1), (0, 1),
                 (-1, -1), (-1, 1), (1, -1), (1, 1))
        path = _astar_core(
            lambda c, r: occ[r][c], cols, rows, tuple(start), tuple(goal),
            moves,
            lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1]),
            lambda a, b: DIAG_COST if (a[0] != b[0] and a[1] != b[1]) else 1.0,
            allow_corner_cut=True,
            edge_penalty=self.edge_penalty)
        if path is None:
            return None
        return [self.cell_xy(c, r) for c, r in path]

    # -- golden 712-717 ----------------------------------------------------
    def route_goal(self, marker):
        d = math.hypot(marker["x"], marker["y"])
        if d <= self.TARGET_STOP_RAW + 20:
            return 0, 0
        scale = (d - self.TARGET_STOP_RAW) / d
        return marker["x"] * scale, marker["y"] * scale

    # -- golden 743-775 ----------------------------------------------------
    def plan_route(self, pts, marker, pose, keepouts=()):
        """Golden's decision, returned as a dict with the same keys it published."""
        goal = self.route_goal(marker)
        obs = self.planning_obstacles(pts, pose, marker, keepouts)
        blocked = []
        for p in obs:
            if point_segment_dist(p["x"], p["y"], 0, 0,
                                  goal[0], goal[1]) <= self.ROUTE_RADIUS:
                blocked.append({"x": p["x"], "y": p["y"]})
                if len(blocked) >= 40:
                    break
        if not blocked:
            return {"ok": True, "msg": "STRAIGHT CLEAR",
                    "path": [(0, 0), goal], "blocked": blocked}
        occ = self.build_costmap(obs, pose)
        raw = self.astar(occ, (0, 0), goal)
        if not raw:
            return {"ok": False, "msg": "BLOCKED: no safe A* route",
                    "path": [], "blocked": blocked}
        path = douglas_peucker(raw, epsilon=self.DP_EPSILON)
        if path[-1] != raw[-1]:
            path.append(raw[-1])
        return {"ok": True, "msg": "REROUTE PLANNED", "path": path,
                "blocked": blocked}

    # -- golden `_p5_navdrive` 1403-1412 ----------------------------------
    def segments(self, path, min_leg_mm=30.0):
        """Golden's segment builder, verbatim, INCLUDING its `continue`.

        Note the difference from `route_to_segments`: golden re-measured from
        `path[i-1]` even when the previous leg was dropped, so a dropped hop
        deleted that displacement from the route. Reproduced here for fidelity;
        the live path uses the merging version and says why.
        """
        segs = []
        heading = 0.0
        for i in range(1, len(path)):
            a, b = path[i - 1], path[i]
            ax = a["x"] if isinstance(a, dict) else a[0]
            ay = a["y"] if isinstance(a, dict) else a[1]
            bx = b["x"] if isinstance(b, dict) else b[0]
            by = b["y"] if isinstance(b, dict) else b[1]
            dx, dy = bx - ax, by - ay
            sd = math.hypot(dx, dy)
            if sd < min_leg_mm:
                continue
            sb = math.atan2(dx, dy)           # golden's atan2(dx, dy): from +y
            # Golden's own wrap, NOT `wrap_pi`. They agree mathematically and
            # disagree in the last bit of the mantissa, and the fidelity test
            # compares exact floats -- so this is golden's loop, verbatim.
            tr = sb - heading
            while tr > math.pi:
                tr -= 2 * math.pi
            while tr < -math.pi:
                tr += 2 * math.pi
            segs.append((math.degrees(tr), sd))
            heading = sb
        return segs


INTEGRATION = """
HOW TO WIRE THIS INTO fpms_missions.py -- WRITTEN, NOT APPLIED
==============================================================
Nothing in this file is imported by anything on the Pi. Deploying it is inert.
Apply the following only after the agent that owns fpms_missions.py has released
it, and apply them in this order -- each step is independently revertable and
each leaves the rover in a working state.

STEP 1 (zero risk, do it first and alone). Fix the arena.
    fpms_missions.py:526   ARENA_MM = max(ARENA_W_MM, ARENA_H_MM)
  The whole planner stack downstream is SQUARE: OccupancyGrid.n, CostMap.n,
  inflate_blocked's `arena_mm`, and `points()`'s wall-band test all take one
  scalar. With W=1500 and H=1200 that is 1500, so the grid models 300 mm of +y
  floor that does not exist and the wall pad sits 300 mm past the real wall.
  RISK: the planner currently believes it may drive to y=1350. Until this is
  fixed, do not trust any detour that bends toward +y.
  MINIMAL FIX (no new module): make OccupancyGrid/CostMap take (nx, ny) and
  (arena_w_mm, arena_h_mm) as this module's classes do.

STEP 2 (low risk, additive). Import the module and load the prior.
    at the end of fpms_missions.py's config block (~line 2220):
        try:
            import fpms_planner as _fplan
            PLANNER_PRIOR = _fplan.load_learned_prior(
                "/var/lib/fpms/learned_grid.json", cell_mm=GRID_MM,
                arena_w_mm=ARENA_W_MM, arena_h_mm=ARENA_H_MM)
        except Exception:
            _fplan, PLANNER_PRIOR = None, None
  RISK: none that can reach the wheels. `load_learned_prior` cannot raise, and a
  None prior plans identically to no prior. The `try` is only for the import.

STEP 3 (the substitution). Replace the search, keep the call sites.
  Three functions in fpms_missions.py are the entire seam, and their signatures
  do not need to change:
    fpms_missions.py:3315  _search(cmap, x, y, heading, tx, ty, planner)
    fpms_missions.py:3335  plan_detour(x, y, tx, ty, grid, now, heading_deg,
                                       committed, planner) -> PlanOutcome
    fpms_missions.py:3479  plan_grid_route(...) -> (segments, note)
  `Planner.plan(...) -> Plan` returns the SAME three-answer contract
  PlanOutcome.points has: [] clear, [...] detour, None no route. So:
    * in `_search`, add   if planner == "dstar": return dstar branch
    * in `plan_detour`, replace the RELAX_LADDER loop body with
          pl = PLANNER_OBJ.plan(x, y, heading_deg, tx, ty, points=pts)
      and map pl -> PlanOutcome field by field (points, relax, cost_mm, note).
  RISK: MEDIUM. `plan_detour`'s hysteresis block (3403-3430) and the `committed`
  path are NOT reproduced in this module and must stay where they are -- they sit
  AFTER the search and only need `pl.points`. Do not delete them.
  RISK: `PLANNER` validation at 1990 must gain "dstar" or the setting falls back
  to "theta" silently.

STEP 4 (the actual win). Make the mid-leg reroute incremental.
    fpms_missions.py:6585  MissionWorker._reroute(tx, ty, det, replans, committed)
    fpms_missions.py:6620  res = plan_detour(pose[0], pose[1], tx, ty, grid, ...)
  Hold ONE IncrementalPlanner per leg instead of calling plan_detour fresh:
      self._inc = IncrementalPlanner(PLANNER_OBJ, self.node.occ,
                                     pose, (tx, ty))          # at leg start
      pl = self._inc.update(pose)                             # in _reroute
  and read `pl.points` exactly where `res.points` is read now.
  RISK: MEDIUM-HIGH, and it is the one to test on a stand. The IncrementalPlanner
  holds state across a leg; if the target changes, or the leg restarts, it MUST
  be discarded and rebuilt or it will repair toward the previous goal. Rebuild it
  wherever `replans` is reset to 0.
  MITIGATION: `plan.repaired` is False on any rebuild, so the journal shows which
  answers were incremental. Ship with PLANNER=theta and only switch to dstar once
  the repair has been watched on the field.

STEP 5 (segments). `route_to_segments` emits an explicit per-segment
  `turn_deg`, and 0.0 exactly on a straight run, which is what the executor's
  rotation budget needs. fpms_missions.py builds segments in `plan_route` (1506)
  and `plan_via_route`; those already floor turns at BEARING_TOL_DEG. Either
  keep theirs or take this one, but NOT both -- two segment builders will
  eventually disagree, and a drawn route that disagrees with the driven one is
  worse than no route at all.
  RISK: LOW if swapped wholesale, HIGH if half-swapped.

WHAT THIS MODULE DELIBERATELY DOES NOT DO, so nothing is expected of it:
  no MQTT, no telemetry publishing, no `Segment` dataclass compatible with
  fpms_missions.Segment (theirs carries dock/crawl flags this one has no opinion
  about), no hysteresis, no REPLAN_MAX counting, and no abort. Those stay in
  fpms_missions.py where they already work.
"""


if __name__ == "__main__":            # pragma: no cover -- a smoke run, not a test
    p = Planner()
    g = p.grid()
    print("arena %.0f x %.0f mm, grid %d x %d @ %.0f mm"
          % (p.arena_w_mm, p.arena_h_mm, p.nx, p.ny, p.cell_mm))
    pl = p.plan(200, 200, 0.0, 1300, 1000, grid=g)
    print("empty arena:", pl.planner, pl.segments)
    g.mark_obstacle(750, 600, 80.0)
    g.set_exclusion(1300, 1000)
    pl = p.plan(200, 200, 0.0, 1300, 1000, grid=g)
    print("with obstacle:", pl.planner, pl.note)
    for s in pl.segments:
        print("   ", s.as_dict())

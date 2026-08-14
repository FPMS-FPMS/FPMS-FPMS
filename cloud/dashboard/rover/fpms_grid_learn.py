#!/usr/bin/env python3
"""
FPMS LEARNED GRID PRIOR -- what the arena has actually contained, across runs.

    python3 fpms_grid_learn.py --show
    python3 fpms_grid_learn.py --rebuild /var/lib/fpms/learn --out /tmp/g.json
    python3 fpms_grid_learn.py --self-test

WHAT THIS IS
    `fpms_missions.OccupancyGrid` is a LIVE grid with a 6 second TTL. It is
    correct for its job -- deciding, right now, whether to drive at something --
    and it is deliberately amnesiac: nothing it has ever seen survives the run,
    so the rover starts every mission believing the arena is empty.

    This file is the other half. It keeps a SLOW, CROSS-RUN belief about where
    obstacles have really been, so a box that has stood in the same place for
    ten runs is known before the LiDAR sees it, and a single phantom return
    fades on its own.

WHAT THIS IS NOT
    It is not a model, it is not trained, and nothing here generalises to
    another arena. It is a counter per 50 mm cell with an exponential decay --
    the same shape as any occupancy filter, written out in a form the planner
    can read. README_LEARN.md's first paragraph applies unchanged.

    It is also ADVISORY ONLY. It never blocks a route by itself: the planner
    treats it as a COST prior, the live grid still owns every stop decision,
    and if this file is missing or malformed the planner ignores it and
    behaves exactly as it does today. That is the whole safety argument for
    letting a recorded belief influence anything.

THE COUNTING UNIT IS AN EPOCH, NOT A SCAN
    A rover parked in front of a wall sees that wall at 9.5 Hz. If every scan
    were a "hit", sixty seconds of standing still would bank 570 of them and
    the cell could never be talked out of again by anything short of a week of
    contrary evidence. So evidence is committed at most ONCE PER CELL PER
    EPOCH (default 5 s). What the counts then measure is how many independent
    LOOKS agreed, which is the thing a confidence is supposed to mean.

HITS AND MISSES ARE NOT SYMMETRIC, AND THE ASYMMETRY IS ON PURPOSE
    A hit is a believed surface: the live-grid rule (>= OCC_MIN_HITS returns in
    the cell inside the TTL), mirrored here so this file and the thing it is
    describing agree about what "occupied" means.

    A miss is FREE SPACE PROVEN BY A RAY THAT WENT PAST. That is ray-casting,
    which the live grid refuses to do -- and refuses for a good reason: one
    badly-posed scan can erase a real obstacle, and erasing an obstacle from a
    live safety grid is how a rover drives into it. The trade is different
    here. This grid stops nothing, its output is a cost, and without free-space
    evidence a phantom can only fade with TIME, which means a phantom seen once
    outranks a cell proven empty a hundred times. So misses are counted -- and
    a cell is only ever DOWN-weighted by them, never marked passable, because
    nothing downstream reads this file for permission.

DECAY
    Both counts decay exponentially with a half-life (default 14 days) applied
    on load and on every commit. An obstacle that is really there is re-hit
    every run and stays sharp; one that was moved away last month falls under
    the noise floor and is pruned. Without this the file only ever accumulates,
    and a prior that cannot forget is a prior that gets worse for ever.

THE FILE FORMAT IS A CONTRACT
    /var/lib/fpms/learned_grid.json, written atomically (tmp + fsync + rename)
    so a power cut during a write leaves the previous version intact rather
    than a half-file. A separate agent's `fpms_planner.py` reads exactly this:

      {"version":1, "arena_mm":[1500,1200], "cell_mm":50, "updated":<unix>,
       "cells":[{"x":<mm>,"y":<mm>,"hits":<int>,"misses":<int>,"conf":<0..1>}],
       "notes":"..."}

    `x`/`y` are the CELL CENTRE in arena millimetres, integers. `conf` is
    hits/(hits+misses+2) -- Laplace-smoothed, so one hit and no contradiction
    is 0.33 rather than a confident 1.0, which is what makes a one-off phantom
    cheap to ignore and a repeated obstacle expensive.

ARENA SIZE IS READ, NOT HARDCODED
    The arena was shortened 200 mm on 2026-08-14 (y: 1400 -> 1200) and there is
    every reason to think it will change again. `arena_dims()` takes the
    environment first, then PARSES `fpms_missions.py` -- the file that actually
    defines it -- and only then falls back to a literal. A constant copied here
    would be a second source of truth that silently goes stale.
"""
import os
import re
import sys
import json
import math
import time
import glob
import threading
import tempfile

VERSION = 1

PRIOR_PATH = os.environ.get("FPMS_LEARNED_GRID",
                            "/var/lib/fpms/learned_grid.json")
MISSIONS_SRC = os.environ.get("FPMS_MISSIONS_SRC",
                              "/home/ubuntu/fpms_missions.py")

# Laplace smoothing. 2.0 means one uncontradicted hit reads 0.33, which is
# deliberately unimpressive: a cost prior should not swing a route on a single
# return, and the LIVE grid is what handles a real obstacle in real time.
CONF_PRIOR_N = float(os.environ.get("FPMS_GRID_PRIOR_N", "2.0"))
HALF_LIFE_S = float(os.environ.get("FPMS_GRID_HALF_LIFE_S", str(14 * 86400)))
# Below this, a cell's evidence is indistinguishable from nothing and keeping
# it only grows the file.
PRUNE_BELOW = float(os.environ.get("FPMS_GRID_PRUNE_BELOW", "0.05"))
EPOCH_S = float(os.environ.get("FPMS_GRID_EPOCH_S", "5.0"))
MAX_CELLS = int(os.environ.get("FPMS_GRID_MAX_CELLS", "4000"))
# A CEILING ON EVIDENCE, so a rover left parked for an hour staring at one
# thing cannot bank 700 hits on it. Past this, `conf` is saturated (60 hits and
# no misses reads 0.97) and further agreement adds nothing except the ability
# to ignore future contradiction -- which is exactly the property a prior must
# not have. Clamped, so a cell that really is wrong can still be talked down in
# a few dozen contrary looks rather than a few thousand.
EVIDENCE_CAP = float(os.environ.get("FPMS_GRID_EVIDENCE_CAP", "60"))

# Mirrors of fpms_missions' live-grid constants. Read from the environment the
# same way it reads them, so /etc/fpms/config.env moves both together.
def _cfgf(name, default, lo, hi):
    """The DEFAULT on out-of-range, never a clamp -- fpms_missions._cfg_float
    behaves this way and a mirror that clamped would disagree with the thing it
    mirrors at exactly the values somebody had to think about."""
    try:
        v = float(os.environ[name])
    except Exception:
        return float(default)
    return v if lo <= v <= hi else float(default)


GRID_MM = _cfgf("FPMS_MISSION_GRID_MM", 50.0, 20.0, 200.0)
OCC_MIN_HITS = int(_cfgf("FPMS_MISSION_OCC_MIN_HITS", 2, 1, 20))
OCC_TTL_S = _cfgf("FPMS_MISSION_OCC_TTL_S", 6.0, 0.5, 120.0)
OCC_MAX_RANGE_MM = _cfgf("FPMS_MISSION_OCC_MAX_RANGE_MM", 1800.0, 200.0, 20000.0)
SELF_FILTER_MM = _cfgf("FPMS_MISSION_SELF_FILTER_MM", 140.0, 0.0, 400.0)
LIDAR_ZERO_OFFSET_DEG = _cfgf("FPMS_LIDAR_ZERO_OFFSET_DEG", 0.0, -180.0, 180.0)
LIDAR_ROTATION_SIGN = _cfgf("FPMS_LIDAR_ROTATION_SIGN", -1.0, -1.0, 1.0)
# THE ARENA WALL IS NOT A LEARNED OBSTACLE. A parked rover sees the boundary
# from every bearing, so without this the prior fills with a hundred cells of
# perimeter within a minute and the actual obstacles are lost in it. The
# boundary is also the one thing here known EXACTLY -- `inflate_blocked` already
# pads it from WALL_PAD_MM, a number nobody had to measure -- so learning it
# adds nothing and costs the file's whole signal. Same band the executor's
# `points()` uses to make the same wall/obstacle split, from the same config
# key, so the two agree about what counts as the wall.
WALL_BAND_MM = _cfgf("FPMS_MISSION_WALL_BAND_MM", 80.0, 0.0, 400.0)
# How far short of a return the free-space evidence stops. One cell, because a
# range is only good to about that and marking free right up to the surface
# would put a "proven empty" on the obstacle itself.
CLEAR_MARGIN_MM = float(os.environ.get("FPMS_GRID_CLEAR_MARGIN_MM",
                                       str(GRID_MM)))


def arena_dims():
    """(w_mm, h_mm). Environment, then fpms_missions.py, then the fallback.

    Returns the source too, because a prior built against the wrong arena is
    wrong everywhere and it must be possible to see which number was used."""
    ew = os.environ.get("FPMS_ARENA_W_MM")
    eh = os.environ.get("FPMS_ARENA_H_MM")
    if ew and eh:
        try:
            return float(ew), float(eh), "environment"
        except Exception:
            pass
    try:
        with open(MISSIONS_SRC, encoding="utf-8", errors="replace") as fh:
            head = fh.read(200000)
        w = re.search(r"^ARENA_W_MM\s*=\s*([0-9.]+)", head, re.M)
        h = re.search(r"^ARENA_H_MM\s*=\s*([0-9.]+)", head, re.M)
        if w and h:
            return float(w.group(1)), float(h.group(1)), MISSIONS_SRC
    except Exception:
        pass
    # 1500 x 1200 as of 2026-08-14. NOT 1400: the arena was shortened 200 mm on
    # the y axis that day and the old value is still written down in several
    # documents.
    return 1500.0, 1200.0, "fallback default (fpms_missions.py unreadable)"


# =========================================================== SCAN -> CELLS
class ScanWitness:
    """One scan, in arena millimetres. A MIRROR of OccupancyGrid.integrate.

    Deliberately a mirror and not an import: this runs in a subscribe-only
    process that must not import the mission executor, whose module body
    connects to MQTT and builds a node. The transform is 8 lines and the
    constants come from the same environment, so the two stay together; what
    would NOT stay together is a copy that quietly used its own bearing
    convention. If `LIDAR_ROTATION_SIGN` or the mount yaw is ever corrected in
    config.env, both move at once.
    """

    def __init__(self, cell_mm=None, arena=None):
        self.cell_mm = float(GRID_MM if cell_mm is None else cell_mm)
        if arena:
            w, h, src = float(arena[0]), float(arena[1]), "caller"
        else:
            w, h, src = arena_dims()
        self.w_mm, self.h_mm, self.arena_src = float(w), float(h), src
        self.nx = max(1, int(math.ceil(self.w_mm / self.cell_mm)))
        self.ny = max(1, int(math.ceil(self.h_mm / self.cell_mm)))
        # (ix,iy) -> [returns_in_ttl, last_seen]
        self._live = {}
        self.scans = 0
        self.last_pose = None

    def cell_of(self, x_mm, y_mm):
        return (int(math.floor(x_mm / self.cell_mm)),
                int(math.floor(y_mm / self.cell_mm)))

    def centre_of(self, ix, iy):
        return ((ix + 0.5) * self.cell_mm, (iy + 0.5) * self.cell_mm)

    def in_grid(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def integrate(self, ranges_m, pose, range_max_m=6.0, now=None,
                  free=True, exclude=None):
        """Fold one scan. Returns (hit_cells, free_cells) as sets of (ix,iy).

        `pose` is (x_mm, y_mm, heading_deg) in the arena frame -- the same
        `MissionNode.pose()` the live grid uses, taken off telemetry/mission so
        there is no second pose derivation anywhere in this process.
        """
        if not ranges_m or pose is None:
            return set(), set()
        px, py, ph = float(pose[0]), float(pose[1]), float(pose[2])
        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(ph)):
            return set(), set()
        now = time.monotonic() if now is None else now
        n = len(ranges_m)
        step = 360.0 / n
        sat = float(range_max_m) * 0.995
        hits, frees = set(), set()
        for i, r in enumerate(ranges_m):
            try:
                rv = float(r)
            except Exception:
                continue
            # A ZERO IS "NO RETURN", NOT A SURFACE AT THE SENSOR. Same three
            # skips as front_clearance_mm and the live grid, for the same
            # reason: a zero written in puts a wall underneath the rover.
            if not math.isfinite(rv) or rv <= 0.0 or rv >= sat:
                continue
            mm = rv * 1000.0
            if mm > OCC_MAX_RANGE_MM:
                continue
            if mm < SELF_FILTER_MM:
                continue          # the rover's own bodywork in the beam
            b = math.radians(ph + LIDAR_ROTATION_SIGN * (i * step)
                             + LIDAR_ZERO_OFFSET_DEG)
            cb, sb = math.cos(b), math.sin(b)
            hx, hy = px + mm * cb, py + mm * sb
            if exclude is not None and math.hypot(
                    hx - exclude[0], hy - exclude[1]) <= exclude[2]:
                continue          # the thing being docked against, not an obstacle
            c = self.cell_of(hx, hy)
            if self.in_grid(*c):
                e = self._live.get(c)
                if e is None or (now - e[1]) > OCC_TTL_S:
                    self._live[c] = [1, now]
                else:
                    e[0] = min(e[0] + 1, OCC_MIN_HITS + 8)
                    e[1] = now
                if self._live[c][0] >= OCC_MIN_HITS:
                    hits.add(c)
            if free:
                # Ray-cast the clear span. Stepping by a whole cell is enough:
                # a finer step revisits cells it has already added and costs
                # CPU on a Pi that is running a mission at the same time.
                d = SELF_FILTER_MM
                stop = mm - CLEAR_MARGIN_MM
                while d < stop:
                    fc = self.cell_of(px + d * cb, py + d * sb)
                    if self.in_grid(*fc):
                        frees.add(fc)
                    d += self.cell_mm
        self.scans += 1
        self.last_pose = (px, py, ph)
        # A cell cannot be both this scan; the surface wins, because the ray
        # that proved it free was a DIFFERENT bearing and the surface is the
        # more expensive thing to be wrong about.
        frees -= hits
        # Housekeeping so a node running for weeks does not grow a dict of
        # every cell it ever glimpsed.
        if len(self._live) > 4 * (self.nx * self.ny):
            for c in [c for c, e in self._live.items()
                      if (now - e[1]) > OCC_TTL_S]:
                del self._live[c]
        return hits, frees

    def believed(self, now=None):
        """Cells the LIVE rule believes right now: >= OCC_MIN_HITS, in TTL."""
        now = time.monotonic() if now is None else now
        return set(c for c, e in self._live.items()
                   if e[0] >= OCC_MIN_HITS and (now - e[1]) <= OCC_TTL_S)


# ============================================================== THE PRIOR
class GridPrior:
    """The cross-run belief. Load, accumulate, decay, write atomically."""

    def __init__(self, path=None, cell_mm=None, arena=None):
        self.path = path or PRIOR_PATH
        self.cell_mm = float(GRID_MM if cell_mm is None else cell_mm)
        if arena:
            self.w_mm, self.h_mm, self.arena_src = (
                float(arena[0]), float(arena[1]), "caller")
        else:
            self.w_mm, self.h_mm, self.arena_src = arena_dims()
        self.cells = {}                 # (ix,iy) -> [hits, misses]
        # `observe` is called from the ROS executor thread and `commit`/`save`
        # from paho's network thread when a mission ends. Its own lock, not the
        # recorder's: this must never be able to delay a sensor callback.
        self._lock = threading.RLock()
        self.updated = 0.0
        self.runs = 0
        self.episodes = 0
        self._epoch_t = 0.0
        self._epoch_hits = set()
        self._epoch_free = set()
        self.load()

    # -- geometry ---------------------------------------------------------
    def centre_of(self, ix, iy):
        return ((ix + 0.5) * self.cell_mm, (iy + 0.5) * self.cell_mm)

    def _in(self, ix, iy):
        return (0 <= ix < int(math.ceil(self.w_mm / self.cell_mm))
                and 0 <= iy < int(math.ceil(self.h_mm / self.cell_mm)))

    def _learnable(self, ix, iy):
        """Inside the arena AND not the arena wall. See WALL_BAND_MM."""
        if not self._in(ix, iy):
            return False
        cx, cy = self.centre_of(ix, iy)
        edge = min(cx, cy, self.w_mm - cx, self.h_mm - cy)
        return edge > WALL_BAND_MM

    # -- persistence -------------------------------------------------------
    def load(self):
        """Read the file if it is there. A malformed one is IGNORED, not
        repaired: the planner is required to tolerate a missing prior, so the
        safe response to a file nobody can parse is to behave as if it did not
        exist and start again."""
        try:
            with open(self.path, encoding="utf-8") as fh:
                d = json.load(fh)
        except FileNotFoundError:
            return False
        except Exception as e:
            sys.stderr.write(f"learned_grid: unreadable ({e}); starting fresh\n")
            return False
        try:
            if int(d.get("version", 0)) != VERSION:
                return False
            cell = float(d.get("cell_mm") or self.cell_mm)
            if abs(cell - self.cell_mm) > 1e-6:
                # A grid re-sized under a stored prior cannot be re-indexed
                # honestly -- the counts describe squares that no longer exist.
                sys.stderr.write(
                    f"learned_grid: cell_mm changed {cell} -> {self.cell_mm}; "
                    "the stored counts describe cells that no longer exist, so "
                    "they are discarded rather than re-indexed\n")
                return False
            am = d.get("arena_mm") or []
            if len(am) == 2 and (abs(float(am[0]) - self.w_mm) > 1.0
                                 or abs(float(am[1]) - self.h_mm) > 1.0):
                sys.stderr.write(
                    f"learned_grid: arena changed {am} -> "
                    f"[{self.w_mm:.0f},{self.h_mm:.0f}]; cells outside the new "
                    "arena are dropped, the rest are kept\n")
            self.updated = float(d.get("updated") or 0.0)
            meta = d.get("meta") or {}
            self.runs = int(meta.get("runs") or 0)
            self.episodes = int(meta.get("episodes") or 0)
            for c in d.get("cells") or []:
                ix, iy = (int(math.floor(float(c["x"]) / self.cell_mm)),
                          int(math.floor(float(c["y"]) / self.cell_mm)))
                # `_learnable`, not `_in`: a stored file may predate the
                # wall-band rule or the arena resize, and reloading cells this
                # version would never have written is how a retired mistake
                # comes back for ever.
                if not self._learnable(ix, iy):
                    continue
                self.cells[(ix, iy)] = [float(c.get("hits") or 0.0),
                                        float(c.get("misses") or 0.0)]
            self._decay(time.time())
            return True
        except Exception as e:
            sys.stderr.write(f"learned_grid: malformed ({e}); starting fresh\n")
            self.cells = {}
            return False

    def _decay(self, now):
        """Exponential forgetting, applied from `updated` to `now`.

        Applied to BOTH counts at the same rate, so the RATIO of evidence is
        preserved while its WEIGHT falls. Because the Laplace prior
        (CONF_PRIOR_N) does not decay with them, `conf` slides toward 0 --
        toward "no evidence" -- rather than toward 0.5 or staying pinned where
        it was. That is the behaviour wanted: an obstacle nobody has seen for a
        month should read as unknown, not as a confident memory, and new
        evidence should move it easily."""
        if not self.updated or HALF_LIFE_S <= 0:
            return
        dt = max(0.0, now - self.updated)
        if dt < 60.0:
            return
        k = 0.5 ** (dt / HALF_LIFE_S)
        if k >= 0.999:
            return
        for c, v in list(self.cells.items()):
            v[0] *= k
            v[1] *= k
            if v[0] + v[1] < PRUNE_BELOW:
                del self.cells[c]

    def conf(self, ix, iy):
        v = self.cells.get((ix, iy))
        if not v:
            return 0.0
        return v[0] / (v[0] + v[1] + CONF_PRIOR_N)

    # -- accumulation ------------------------------------------------------
    def observe(self, hits, frees, now=None):
        """Bank one scan's worth of evidence into the current epoch.

        Nothing is committed here. A cell that appears in fifty scans inside
        one epoch is still one look, and counting it fifty times is how a
        parked rover would convince itself of a wall it can never unlearn."""
        now = time.time() if now is None else now
        with self._lock:
            if not self._epoch_t:
                self._epoch_t = now
            self._epoch_hits |= set(hits)
            self._epoch_free |= set(frees)
            due = (now - self._epoch_t) >= EPOCH_S
        if due:
            self.commit(now)

    def commit(self, now=None):
        """Turn the epoch into at most +1 hit or +1 miss per cell."""
        now = time.time() if now is None else now
        with self._lock:
            return self._commit_locked(now)

    def _commit_locked(self, now):
        if not self._epoch_hits and not self._epoch_free:
            self._epoch_t = now
            return 0
        self._decay(now)
        n = 0
        for c in self._epoch_hits:
            if not self._learnable(*c):
                continue
            v = self.cells.setdefault(c, [0.0, 0.0])
            v[0] += 1.0
            n += 1
        for c in self._epoch_free - self._epoch_hits:
            if not self._in(*c):
                continue
            v = self.cells.get(c)
            if v is None:
                # A cell nobody has ever seen occupied does not need a record
                # to say so -- absence already means "no evidence". Free-space
                # evidence is only banked for cells with a HIT to contradict,
                # which is what keeps this file the size of the obstacles
                # rather than the size of the arena.
                continue
            v[1] += 1.0
            n += 1
        self._epoch_hits, self._epoch_free = set(), set()
        self._epoch_t = now
        self.updated = now
        for c, v in list(self.cells.items()):
            if v[0] + v[1] < PRUNE_BELOW or not self._learnable(*c):
                del self.cells[c]
                continue
            v[0] = min(v[0], EVIDENCE_CAP)
            v[1] = min(v[1], EVIDENCE_CAP)
        return n

    # -- output ------------------------------------------------------------
    def to_dict(self, notes=None):
        items = sorted(self.cells.items(),
                       key=lambda kv: -(kv[1][0] + kv[1][1]))[:MAX_CELLS]
        cells = []
        for (ix, iy), v in sorted(items):
            cx, cy = self.centre_of(ix, iy)
            cells.append({"x": int(round(cx)), "y": int(round(cy)),
                          "hits": int(round(v[0])), "misses": int(round(v[1])),
                          "conf": round(v[0] / (v[0] + v[1] + CONF_PRIOR_N), 3)})
        return {
            "version": VERSION,
            "arena_mm": [int(round(self.w_mm)), int(round(self.h_mm))],
            "cell_mm": int(round(self.cell_mm)),
            "updated": int(round(self.updated or time.time())),
            "cells": cells,
            "notes": notes or (
                "Cross-run obstacle prior from fpms_learn_recorder. x,y are the "
                "CELL CENTRE in arena mm. conf = hits/(hits+misses+%g), both "
                "counts decayed with a %.0f day half-life, at most one count "
                "per cell per %.0fs look. ADVISORY: a cost prior only -- the "
                "live occupancy grid in fpms_missions still owns every stop "
                "decision, and a consumer must behave normally when this file "
                "is absent or malformed. Arena read from %s. runs=%d "
                "episodes=%d." % (CONF_PRIOR_N, HALF_LIFE_S / 86400.0, EPOCH_S,
                                  self.arena_src, self.runs, self.episodes)),
            "meta": {"runs": self.runs, "episodes": self.episodes,
                     "cells_tracked": len(self.cells),
                     "half_life_s": HALF_LIFE_S, "epoch_s": EPOCH_S,
                     "conf_prior_n": CONF_PRIOR_N,
                     "occ_min_hits": OCC_MIN_HITS,
                     "producer": "fpms_grid_learn.py"},
        }

    def save(self, notes=None):
        """Atomic: temp file in the SAME directory, fsync, rename.

        Same directory because rename is only atomic within a filesystem, and
        fsync before the rename because a rename that lands before the data
        does leaves a valid name pointing at nothing. This rover loses power by
        having its battery pulled; that is not a theoretical failure here."""
        with self._lock:
            d = self.to_dict(notes)
        dirn = os.path.dirname(self.path) or "."
        try:
            os.makedirs(dirn, exist_ok=True)
        except Exception:
            pass
        fd, tmp = tempfile.mkstemp(dir=dirn, prefix=".learned_grid.",
                                   suffix=".tmp")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                json.dump(d, fh, separators=(",", ":"))
                fh.flush()
                os.fsync(fh.fileno())
            # 0644, not mkstemp's 0600: the consumer is a separate process
            # written by somebody else and may not run as this user. A prior
            # nobody can read is the same as no prior, except harder to
            # diagnose.
            os.chmod(tmp, 0o644)
            os.replace(tmp, self.path)
            try:
                dfd = os.open(dirn, os.O_RDONLY)
                try:
                    os.fsync(dfd)
                finally:
                    os.close(dfd)
            except Exception:
                pass
            return True
        except Exception as e:
            sys.stderr.write(f"learned_grid: write failed: {e}\n")
            try:
                os.unlink(tmp)
            except Exception:
                pass
            return False


# ================================================================ OFFLINE
def rebuild(paths, out_path, cell_mm=None):
    """Rebuild a prior from the recorder's NDJSON `grid` observations.

    The recorder keeps, per episode, the cells it BELIEVED and the cells it
    proved free. Replaying those is not the same as replaying the raw scans --
    it is coarser -- but it is enough to rebuild the file after it is lost, and
    it means the dataset, not this process's memory, is the thing of record."""
    files = []
    for p in paths:
        if os.path.isdir(p):
            files += sorted(glob.glob(os.path.join(p, "*.ndjson")))
        else:
            files.append(p)
    gp = GridPrior(path=out_path, cell_mm=cell_mm)
    gp.cells = {}
    gp.updated = 0.0
    gp.runs = gp.episodes = 0
    n_obs = 0
    for f in sorted(set(files)):
        try:
            fh = open(f, encoding="utf-8")
        except OSError:
            continue
        with fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    r = json.loads(line)
                except Exception:
                    continue
                if r.get("rec") == "mission_done":
                    gp.runs += 1
                g = r.get("grid") if r.get("rec") == "segment" else None
                if not g:
                    continue
                gp.episodes += 1
                ts = float(r.get("ts_end") or r.get("ts") or time.time())
                hits = set(tuple(c) for c in (g.get("hit_cells") or []))
                free = set(tuple(c) for c in (g.get("free_cells") or []))
                gp._epoch_hits, gp._epoch_free = hits, free
                gp.commit(ts)
                n_obs += 1
    gp.save(notes=None)
    return gp, n_obs, len(files)


def _self_test():
    """Runs anywhere, needs no rover. Checks the three properties this file
    exists to have: a phantom fades, a real obstacle hardens, and the written
    schema is exactly what the planner was told to expect."""
    import tempfile as _tf
    ok = True
    d = _tf.mkdtemp()
    p = os.path.join(d, "learned_grid.json")
    gp = GridPrior(path=p, cell_mm=50.0, arena=(1500.0, 1200.0))
    t = time.time()

    # A phantom: seen once, then proven free ten times.
    for i in range(1):
        gp._epoch_hits = {(4, 4)}
        gp.commit(t + i)
    c_once = gp.conf(4, 4)
    for i in range(10):
        gp._epoch_free = {(4, 4)}
        gp.commit(t + 10 + i)
    c_faded = gp.conf(4, 4)

    # A real obstacle: hit on twelve separate looks.
    for i in range(12):
        gp._epoch_hits = {(9, 9)}
        gp.commit(t + 100 + i)
    c_hard = gp.conf(9, 9)

    print(f"  phantom after 1 hit      conf={c_once:.3f}")
    print(f"  phantom after 10 misses  conf={c_faded:.3f}")
    print(f"  obstacle after 12 hits   conf={c_hard:.3f}")
    if not (c_once < 0.4):
        print("  FAIL: a single hit should not be confident"); ok = False
    if not (c_faded < c_once / 3):
        print("  FAIL: free-space evidence did not fade the phantom"); ok = False
    if not (c_hard > 0.8):
        print("  FAIL: repeated hits did not harden"); ok = False

    gp.save()
    with open(p, encoding="utf-8") as fh:
        got = json.load(fh)
    for k in ("version", "arena_mm", "cell_mm", "updated", "cells", "notes"):
        if k not in got:
            print(f"  FAIL: schema is missing {k!r}"); ok = False
    if got.get("arena_mm") != [1500, 1200]:
        print(f"  FAIL: arena_mm is {got.get('arena_mm')}"); ok = False
    cell = (got.get("cells") or [{}])[0]
    for k in ("x", "y", "hits", "misses", "conf"):
        if k not in cell:
            print(f"  FAIL: a cell is missing {k!r}"); ok = False
    if not all(isinstance(cell.get(k), int) for k in ("x", "y", "hits", "misses")):
        print("  FAIL: x/y/hits/misses must be integers"); ok = False

    # Decay: the same counts, a year later, must be weaker.
    gp.updated = t - 365 * 86400
    gp._decay(t)
    print(f"  obstacle one year later  conf={gp.conf(9, 9):.3f}")
    if gp.conf(9, 9) >= c_hard:
        print("  FAIL: nothing decayed"); ok = False

    print("SELF TEST: " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--self-test" in argv:
        return _self_test()
    if "--rebuild" in argv:
        i = argv.index("--rebuild")
        src = argv[i + 1:i + 2] or [os.environ.get("FPMS_LEARN_DIR",
                                                   "/var/lib/fpms/learn")]
        out = PRIOR_PATH
        if "--out" in argv:
            j = argv.index("--out")
            out = argv[j + 1] if j + 1 < len(argv) else out
        gp, n, nf = rebuild(src, out)
        print(f"rebuilt from {nf} file(s), {n} episode observation(s) -> {out}")
        print(f"{len(gp.cells)} cell(s), {gp.runs} run(s)")
        return 0
    # --show, the default
    path = PRIOR_PATH
    if "--path" in argv:
        i = argv.index("--path")
        path = argv[i + 1] if i + 1 < len(argv) else path
    w, h, src = arena_dims()
    print(f"arena {w:.0f} x {h:.0f} mm (from {src}), cell {GRID_MM:.0f} mm")
    print(f"prior: {path}")
    try:
        with open(path, encoding="utf-8") as fh:
            d = json.load(fh)
    except FileNotFoundError:
        print("  ABSENT -- nothing has been learned yet. This is the expected "
              "state until the rover drives with the recorder running.")
        return 0
    except Exception as e:
        print(f"  MALFORMED ({e}). A consumer must ignore it and carry on.")
        return 0
    cells = d.get("cells") or []
    print(f"  version {d.get('version')}  arena {d.get('arena_mm')}  "
          f"cell {d.get('cell_mm')}mm  updated "
          f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(d.get('updated') or 0))}")
    print(f"  {len(cells)} cell(s) with evidence")
    for c in sorted(cells, key=lambda c: -c.get("conf", 0))[:20]:
        print(f"    ({c['x']:>5},{c['y']:>5})  hits {c['hits']:>3}  "
              f"misses {c['misses']:>3}  conf {c['conf']}")
    if len(cells) > 20:
        print(f"    ... {len(cells) - 20} more")
    print(f"  notes: {d.get('notes')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

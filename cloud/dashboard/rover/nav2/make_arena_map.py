#!/usr/bin/env python3
"""
make_arena_map.py - generate the static Nav2 occupancy map for the FPMS arena.

WHY THIS EXISTS
================
Nav2's static costmap layer and AMCL both need a real map.pgm + map.yaml pair.
The arena is a known, fixed 1200x1200mm walled square (NAV2_BRIEF.md section 6)
so there is no reason to SLAM it or hand-edit an image in GIMP: every pixel can
be derived from arithmetic, and deriving it means the map can never drift out
of sync with its own stated dimensions the way a hand-edited PNG could.

SINGLE SOURCE OF TRUTH
=======================
Arena geometry (ARENA_MM, zone size/margin fractions, zone corners, start
pose) is defined authoritatively in `frontend/src/lib/arena.ts`. This script
MIRRORS those constants (ARENA_MM=1200, zone side = 0.30*ARENA_MM, margin =
0.04*ARENA_MM, zone corners, ROVER_START) rather than owning them. If arena.ts
ever changes, update ARENA_MM / Z_FRAC / M_FRAC below to match, and re-run this
script and the frontend will still agree, because the JSON sidecar this script
emits is derived from the same numbers, not typed in twice.

WORLD FRAME (matches arena.ts exactly)
========================================
Origin at the arena's BOTTOM-LEFT floor corner. +x right, +y up, metres.
The drivable floor occupies world [0, 1.2] x [0, 1.2] (ARENA_MM/1000).

THE ORIGIN ARITHMETIC - READ THIS BEFORE TOUCHING BORDER_PX OR RESOLUTION
============================================================================
A ROS/Nav2 map.yaml `origin: [x, y, yaw]` field is the world-frame pose of the
LOWER-LEFT pixel of the image (map_server / nav2_map_server convention). "Lower
left" means: column 0, and the pixel row that map_server treats as minimum-y
after it un-flips the file's top-to-bottom row order into the bottom-to-top
OccupancyGrid row order.

This script builds the grid as `free_interior` pixels (120x120, the arena
floor) surrounded by a ring of occupied `BORDER_PX` pixels standing in for the
arena walls, so the total image is (120 + 2*BORDER_PX) square. The interior's
first free column/row is not pixel (0,0) - it is offset inward by BORDER_PX to
make room for the wall ring on the low side. So the lower-left pixel of the
WHOLE image (row/col 0) sits BORDER_PX pixels below-and-left of world (0,0),
i.e. at world (-BORDER_PX*RESOLUTION, -BORDER_PX*RESOLUTION).

    origin_x = origin_y = -(BORDER_PX * RESOLUTION)

With the defaults below (BORDER_PX=10, RESOLUTION=0.01) that is exactly
-0.10 m on both axes. Get this sign wrong (e.g. write +0.10, or forget the
border term and write 0.0) and every waypoint Nav2 plans will land 10cm-plus
off from where the dashboard's world-frame math (arena.ts) says it should be -
silently, because nothing errors, it just quietly drives to the wrong spot.

PIXEL <-> WORLD, worked example
================================
Pixel column `c` (0-indexed, image-file order) maps to world x by:
    x = origin_x + c * RESOLUTION
Column c = BORDER_PX (the first interior column) therefore maps to
    x = -(BORDER_PX*RES) + BORDER_PX*RES = 0.0   <- arena's left wall face
Column c = BORDER_PX + 119 (the last interior column) maps to
    x = -(BORDER_PX*RES) + (BORDER_PX+119)*RES = 1.19 m
which is the CENTRE of the last 10mm-wide free cell, correctly short of the
1.20 m right wall face by half a cell - exactly what a centred-pixel occupancy
grid should do. Rows work identically for y, modulo the vertical flip handled
in `write_pgm` (see its docstring).

WALLS AND BORDER
=================
BORDER_PX=10 (0.10 m at this resolution) was chosen as a generous, round
stand-in for the physical arena wall thickness plus a little slack so the
occupied ring is comfortably wide in the costmap and never a knife-edge one
pixel from disappearing under inflation rounding. It is NOT a measurement of
the real wall (unmeasured) - it only needs to be "clearly a wall" to Nav2's
static layer, which it is at 10 full cells.

Everything inside the wall ring is free (value 254 / white). There is no
"unknown" (205 / grey) region in this map: the whole arena floor is bounded
and known ahead of time, so leaving any of it unknown would just make AMCL's
particle filter and the costmap inflation do unnecessary work over territory
that is, in fact, fully characterised.

USAGE
=====
    python make_arena_map.py --out arena
        -> arena.pgm, arena.yaml, arena_zones.json

This module is plain Python (no ROS, no PIL) so it runs anywhere, including
this Windows dev machine, with zero extra dependencies beyond PyYAML.
"""

import argparse
import json
import math
import os

import yaml


# ============================================================================
# ARENA GEOMETRY - mirrors frontend/src/lib/arena.ts. See that file's header
# comment for the frame definitions (world = bottom-left origin, +x right,
# +y up, millimetres there / metres here). DO NOT let these drift from
# arena.ts; if you change one, change the other and re-run this script.
# ============================================================================
ARENA_MM = 1200.0
ARENA_M = ARENA_MM / 1000.0

Z_FRAC = 0.30   # zone side, as a fraction of ARENA_MM (arena.ts: `Z`)
M_FRAC = 0.04   # zone margin, as a fraction of ARENA_MM (arena.ts: `M`)

Z_MM = Z_FRAC * ARENA_MM   # 360 mm zone side
M_MM = M_FRAC * ARENA_MM   # 48 mm margin from the arena wall

FORWARD_HEADING_DEG = 90.0  # arena.ts FORWARD_HEADING_DEG: +y, up the arena

# ============================================================================
# MAP RASTER PARAMETERS
# ============================================================================
RESOLUTION_M = 0.01          # metres/pixel -> 120x120 px covers the 1.2x1.2m floor
INTERIOR_PX = int(round(ARENA_M / RESOLUTION_M))  # 120
BORDER_PX = 10                # wall ring thickness in pixels; see module docstring
IMAGE_SIZE_PX = INTERIOR_PX + 2 * BORDER_PX        # 140

OCCUPIED_VALUE = 0     # PGM grey value for occupied (walls) - black
FREE_VALUE = 254       # PGM grey value for free space - white
# (no UNKNOWN_VALUE / 205 used - see "WALLS AND BORDER" above)

OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196
NEGATE = 0


def zone_rect_mm(x_mm, y_mm, w_mm, h_mm):
    """Bottom-left-anchored rect -> (x_mm, y_mm, w_mm, h_mm, cx_mm, cy_mm)."""
    return {
        "x_mm": x_mm, "y_mm": y_mm, "w_mm": w_mm, "h_mm": h_mm,
        "cx_mm": x_mm + w_mm / 2.0, "cy_mm": y_mm + h_mm / 2.0,
    }


def compute_zones():
    """Zone A (top-left), Zone B (top-right), water (bottom-left).

    Coordinates copied verbatim from the arithmetic in arena.ts's ZONES array
    (M and Z above stand in for that file's `M` and `Z` constants).
    """
    zones = {
        "zone-a": zone_rect_mm(M_MM, ARENA_MM - M_MM - Z_MM, Z_MM, Z_MM),
        "zone-b": zone_rect_mm(ARENA_MM - M_MM - Z_MM, ARENA_MM - M_MM - Z_MM, Z_MM, Z_MM),
        "water-station": zone_rect_mm(M_MM, M_MM, Z_MM, Z_MM),
    }
    return zones


def compute_start_pose():
    """Bottom-right start box, nose FORWARD (+y). Matches arena.ts ROVER_START."""
    x_mm = ARENA_MM - M_MM - Z_MM / 2.0
    y_mm = M_MM + Z_MM / 2.0
    return {
        "x_mm": x_mm, "y_mm": y_mm,
        "x_m": x_mm / 1000.0, "y_m": y_mm / 1000.0,
        "heading_deg": FORWARD_HEADING_DEG,
        "heading_rad": math.radians(FORWARD_HEADING_DEG),
    }


def build_grid():
    """Return a list-of-lists `grid[row][col]` in WORLD row order: row 0 = min
    y (bottom), row IMAGE_SIZE_PX-1 = max y (top). This is the natural order to
    reason about world coordinates in; `write_pgm` flips it to file order.
    """
    grid = [[FREE_VALUE] * IMAGE_SIZE_PX for _ in range(IMAGE_SIZE_PX)]
    last = IMAGE_SIZE_PX - 1
    for row in range(IMAGE_SIZE_PX):
        for col in range(IMAGE_SIZE_PX):
            on_wall = (row < BORDER_PX or row > last - BORDER_PX
                       or col < BORDER_PX or col > last - BORDER_PX)
            if on_wall:
                grid[row][col] = OCCUPIED_VALUE
    return grid


def write_pgm(path, grid):
    """Write a binary (P5) PGM from a WORLD-ordered grid (row 0 = min y).

    PGM files store rows TOP-of-image first. map_server/nav2_map_server's
    convention is the opposite: the occupancy grid's row 0 is the LOWEST y
    (bottom of the map) and `origin` names that row's world position. So the
    world-ordered grid built by `build_grid` (row 0 = bottom) must be written
    to the file with rows in REVERSED order - file row 0 (top of the PNG/PGM
    when viewed in an image tool) is world row IMAGE_SIZE_PX-1 (top of the
    arena, max y).

    Skipping this flip is a classic way to hand Nav2 a map that is upside
    down: it would still load, still be 140x140, still "look like" a walled
    square (this map is symmetric so you would not even see it by eye) - but
    with any interior asymmetry it silently mirrors y, and everything Nav2
    thinks it saw would be reflected top-to-bottom.
    """
    h = len(grid)
    w = len(grid[0])
    with open(path, "wb") as f:
        header = "P5\n{} {}\n255\n".format(w, h)
        f.write(header.encode("ascii"))
        for file_row in range(h):
            world_row = h - 1 - file_row
            f.write(bytes(grid[world_row]))


def write_yaml(path, pgm_filename):
    """map.yaml alongside the pgm. See module docstring for the origin math."""
    origin_x = -(BORDER_PX * RESOLUTION_M)
    origin_y = -(BORDER_PX * RESOLUTION_M)
    doc = {
        "image": pgm_filename,
        "mode": "trinary",
        "resolution": RESOLUTION_M,
        "origin": [origin_x, origin_y, 0.0],
        "negate": NEGATE,
        "occupied_thresh": OCCUPIED_THRESH,
        "free_thresh": FREE_THRESH,
    }
    with open(path, "w") as f:
        # default_flow_style=None lets the origin list stay on one line
        # ([x, y, yaw]) which is how every ROS map.yaml in the wild looks,
        # while resolution/thresh stay one-per-line and diffable.
        yaml.safe_dump(doc, f, default_flow_style=None, sort_keys=False)
    return origin_x, origin_y


def write_zones_sidecar(path, zones, start_pose):
    """JSON sidecar so waypoint code can read zone centres/start pose without
    re-deriving arena.ts's arithmetic a third time (frontend, this script, and
    whatever consumes this JSON would otherwise be three independent copies
    that can silently disagree).
    """
    doc = {
        "_comment": (
            "Derived from the same ARENA_MM/Z_FRAC/M_FRAC constants as the "
            "map in this directory and frontend/src/lib/arena.ts. World frame: "
            "origin bottom-left, +x right, +y up, arena.ts is the source of "
            "truth for these numbers."
        ),
        "arena_mm": ARENA_MM,
        "zones": zones,
        "start_pose": start_pose,
    }
    with open(path, "w") as f:
        json.dump(doc, f, indent=2, sort_keys=False)
        f.write("\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--out", default="arena",
                     help="output path/stem (default: arena) -> "
                          "<stem>.pgm, <stem>.yaml, <stem>_zones.json")
    args = ap.parse_args()

    stem = args.out
    if stem.endswith((".pgm", ".yaml", ".yml")):
        stem = os.path.splitext(stem)[0]

    pgm_path = stem + ".pgm"
    yaml_path = stem + ".yaml"
    zones_path = stem + "_zones.json"

    grid = build_grid()
    write_pgm(pgm_path, grid)
    origin_x, origin_y = write_yaml(yaml_path, os.path.basename(pgm_path))
    zones = compute_zones()
    start_pose = compute_start_pose()
    write_zones_sidecar(zones_path, zones, start_pose)

    print("wrote %s (%dx%d px, %.3fm x %.3fm)"
          % (pgm_path, IMAGE_SIZE_PX, IMAGE_SIZE_PX,
             IMAGE_SIZE_PX * RESOLUTION_M, IMAGE_SIZE_PX * RESOLUTION_M))
    print("wrote %s (origin=[%.3f, %.3f, 0.0], resolution=%.3f, border=%dpx)"
          % (yaml_path, origin_x, origin_y, RESOLUTION_M, BORDER_PX))
    print("wrote %s" % zones_path)
    print()
    print("zone centres (mm):")
    for zid, z in zones.items():
        print("  %-14s cx=%.1f cy=%.1f  (x=%.1f y=%.1f w=%.1f h=%.1f)"
              % (zid, z["cx_mm"], z["cy_mm"], z["x_mm"], z["y_mm"], z["w_mm"], z["h_mm"]))
    print("start pose: x=%.1fmm y=%.1fmm (%.3fm, %.3fm) heading=%.1fdeg"
          % (start_pose["x_mm"], start_pose["y_mm"],
             start_pose["x_m"], start_pose["y_m"], start_pose["heading_deg"]))


if __name__ == "__main__":
    main()

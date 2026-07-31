/**
 * Arena geometry, coordinate frames and the world<->canvas transform.
 *
 * ---------------------------------------------------------------------------
 * FRAMES
 * ---------------------------------------------------------------------------
 * world   Origin at the BOTTOM-LEFT corner of the arena floor.
 *         +x runs right, +y runs UP. Units are millimetres.
 *         The arena occupies [0, ARENA_MM] x [0, ARENA_MM].
 *
 * canvas  Origin TOP-LEFT, +x right, +y DOWN, units are CSS pixels.
 *         This is the frame the 2D context draws in.
 *
 * body    Rover-fixed. Origin at the LiDAR, +x out the nose, +y to port.
 *         Millimetres. Produced by lidarCluster.ts from the raw range bins.
 *
 * heading psi is in radians, measured CCW from world +x. A rover at psi = 0
 *         points along +x (east); psi = PI/2 points along +y (north).
 *
 * ---------------------------------------------------------------------------
 * THE Y FLIP
 * ---------------------------------------------------------------------------
 * world +y is up, canvas +y is down. `worldToCanvasY` below is the ONE place
 * in the entire frontend where that flip happens. Every other module — the
 * clusterer, the renderer, the zone table — works in world millimetres and
 * never thinks about screen orientation. If the map ever renders mirrored,
 * this file is the only place to look.
 *
 * Likewise, canvas rotation is clockwise-positive while psi is CCW-positive,
 * so anything calling ctx.rotate() with a heading must negate it: rotate(-psi).
 *
 * ---------------------------------------------------------------------------
 * THE MAP IS WORLD-FIXED
 * ---------------------------------------------------------------------------
 * This is a bird's-eye POSE map, not a rover-centric radar sweep. The arena,
 * its grid and its zones occupy one fixed orientation on screen and NEVER
 * rotate — no transform in this frontend takes rover heading and applies it to
 * the scene. Only the rover glyph rotates, and it does so about its own centre
 * inside a save()/restore() pair (ArenaMap.drawRover). Point-cloud and object
 * coordinates arrive already resolved into world millimetres, so they are
 * drawn with the same fixed transform as the grid.
 *
 * The corollary is that "which way is forward" must be stated, not inferred
 * from a spinning picture: FORWARD_HEADING_DEG below is that statement, and
 * the static layer paints a matching FORWARD marker at the top edge.
 */

/**
 * Side length of the (square) arena in millimetres — 120 cm x 120 cm.
 *
 * This is the single source of truth. Zones, grid spacing, wall-detection
 * thresholds and the rover start pose are all expressed as fractions of it,
 * so changing this one number rescales the entire map coherently.
 */
export const ARENA_MM = 1200;

/** Rover chassis footprint, millimetres. Nose-to-tail x beam. */
export const ROVER_LEN_MM = 240;
export const ROVER_WID_MM = 180;

/**
 * Mounting correction for the scanner: degrees added to the raw bin index
 * before it becomes a body-frame bearing. 0 = bin 0 points out the nose.
 */
export const LIDAR_ZERO_OFFSET_DEG = 0;

/**
 * The LD06 reports angles increasing CLOCKWISE, while the body frame is
 * CCW-positive. Negating the bin index is what puts a wall on the rover's
 * left in the scan on the rover's left on the map.
 */
export const LIDAR_ANGLE_SIGN = -1;

/**
 * Bins at (or fractionally under) range_max are the driver's saturation
 * value, not a real surface. The agent clamps with min(mm/1000, 6.0), so a
 * literal 6.000 means "nothing out there" — treating it as a wall paints a
 * phantom ring at max range and swamps the clusterer.
 */
export const RANGE_SATURATION_FRAC = 0.995;

/** Grid pitch, millimetres. Minor = 50 mm, major = 300 mm at ARENA_MM=1200. */
export const GRID_MINOR_MM = ARENA_MM / 24;
export const GRID_MAJOR_MM = ARENA_MM / 4;

/** Pixels per millimetre for a square viewport of `sizePx` with `padPx` margin. */
export function scaleFor(sizePx: number, padPx: number): number {
  const usable = sizePx - 2 * padPx;
  return usable > 0 ? usable / ARENA_MM : 0;
}

/** world x (mm) -> canvas x (px). */
export function worldToCanvasX(xw: number, s: number, pad: number): number {
  return pad + xw * s;
}

/** world y (mm) -> canvas y (px). THE ONLY Y FLIP IN THE FRONTEND. */
export function worldToCanvasY(yw: number, s: number, pad: number): number {
  return pad + (ARENA_MM - yw) * s;
}

/**
 * Object-returning convenience wrapper. Allocates, so it is for setup code
 * (static layer, one-off glyph placement) only — the per-point hot loops in
 * ArenaMap call the scalar forms above.
 */
export function worldToCanvas(
  xw: number,
  yw: number,
  s: number,
  pad: number,
): { px: number; py: number } {
  return { px: worldToCanvasX(xw, s, pad), py: worldToCanvasY(yw, s, pad) };
}

/** A named region painted on the static layer. Rect anchored BOTTOM-LEFT. */
export type Zone = {
  id: string;
  label: string;
  /** world mm, bottom-left corner */
  x_mm: number;
  y_mm: number;
  w_mm: number;
  h_mm: number;
  stroke: string;
  fill: string;
  hatch?: boolean;
};

/** Zone side and margin, as fractions of the arena so a rescale is free. */
const Z = 0.3 * ARENA_MM;
const M = 0.04 * ARENA_MM;

export const ZONES: readonly Zone[] = [
  {
    id: "zone-a",
    label: "ZONE A",
    x_mm: M,
    y_mm: ARENA_MM - M - Z, // top-left
    w_mm: Z,
    h_mm: Z,
    stroke: "#38bdf8",
    fill: "rgba(56,189,248,0.10)",
  },
  {
    id: "zone-b",
    label: "ZONE B",
    x_mm: ARENA_MM - M - Z,
    y_mm: ARENA_MM - M - Z, // top-right
    w_mm: Z,
    h_mm: Z,
    stroke: "#a3e635",
    fill: "rgba(163,230,53,0.10)",
  },
  {
    id: "water-station",
    label: "WATER",
    x_mm: M,
    y_mm: M, // bottom-left
    w_mm: Z,
    h_mm: Z,
    stroke: "#22d3ee",
    fill: "rgba(34,211,238,0.12)",
    hatch: true,
  },
];

/**
 * "FORWARD" — the heading that points straight UP the arena.
 *
 * psi is measured CCW from world +x and world +y is up, so +y (up the arena,
 * towards the top of the screen after the single Y flip in `worldToCanvasY`)
 * is exactly 90 degrees. This is a named constant rather than a literal so
 * that "the rover starts facing forward" is legible at the call site: the
 * operator's requirement is that the map opens facing forward and stays that
 * way, and 90 is the only value that satisfies it in this frame.
 *
 * The static layer draws a matching FORWARD marker at the top edge of the map
 * (see ArenaMap.drawStatic) so the convention is visible, not just documented.
 */
export const FORWARD_HEADING_DEG = 90;

/**
 * Where the rover is assumed to be when no odometry is published: the
 * bottom-right quadrant, nose pointed FORWARD — straight up the arena.
 *
 * The position is unchanged (bottom-right start box). Only the heading is
 * pinned to FORWARD_HEADING_DEG. The previous 135 deg diagonal was a
 * presentation choice — it fanned the scan across the map — but it made the
 * map open at a slant, which reads as a bug to an operator who expects the
 * bird's-eye view to start and stay facing forward.
 */
export const ROVER_START = {
  x_mm: ARENA_MM - M - Z / 2,
  y_mm: M + Z / 2,
  heading_deg: FORWARD_HEADING_DEG,
} as const;

export type Pose = {
  /** world mm */
  x_mm: number;
  y_mm: number;
  /** radians CCW from world +x */
  heading_rad: number;
  /**
   * True when any component fell back to ROVER_START — which, for this fleet,
   * is always, because the heartbeat publishes no x_m/y_m/heading_deg.
   *
   * When this is set the heading is the ASSUMED start heading
   * (FORWARD_HEADING_DEG), never a measured one. The HUD must say so: an
   * operator who reads a bearing off this map while `simulated` is true is
   * reading a constant the rover has never reported.
   */
  simulated: boolean;
};

function isNum(v: unknown): v is number {
  return typeof v === "number" && Number.isFinite(v);
}

/** Pull `key` out of an unknown object without throwing on null/primitives. */
function pick(obj: unknown, key: string): unknown {
  if (obj === null || typeof obj !== "object") return undefined;
  return (obj as Record<string, unknown>)[key];
}

/**
 * Resolve a drawable pose from whatever telemetry happens to exist.
 *
 * THIS FLEET DOES NOT PUBLISH POSE. The rover agent's heartbeat() sends
 * telemetry/pose containing only uptime_s / camera_ok / lidar_ok / frames /
 * scans / streaming — there is no x_m, y_m, heading_deg or battery_pct
 * anywhere in it. Reading `pose.data.data.x_m.toFixed()` unguarded is the
 * exact crash recorded in HANDOFF.md: a TypeError with no error boundary
 * above it, which blanked the whole dashboard.
 *
 * So this function never throws and never returns null. Every field is
 * finite-checked and anything missing falls back to ROVER_START with
 * `simulated: true`. Callers must NOT branch their rendering math on pose
 * presence — always draw with what comes back and let the badge tell the
 * operator the position is assumed.
 */
export function readPose(envelope: unknown, lidarData?: unknown): Pose {
  // Accept either the envelope {thing, data:{...}} or a bare data object.
  const d = pick(envelope, "data") ?? envelope;

  const x = pick(d, "x_m");
  const y = pick(d, "y_m");
  const hasXY = isNum(x) && isNum(y);

  let x_mm = ROVER_START.x_mm;
  let y_mm = ROVER_START.y_mm;
  if (hasXY) {
    x_mm = (x as number) * 1000;
    y_mm = (y as number) * 1000;
  }

  // HEADING IS ONLY READ FROM TELEMETRY WHEN THE POSITION IS REAL.
  //
  // While the pose is simulated the heading stays pinned to
  // FORWARD_HEADING_DEG (90 deg, straight up the arena) and NOTHING may
  // override it. That guard exists specifically because the LiDAR payload
  // hard-codes `heading_deg: 0`: honouring a firmware constant while the
  // position is assumed would swing the glyph round to face +x (right across
  // the screen) and read as a rendering bug, when in fact the rover never
  // reported a heading at all.
  //
  // The `if (hasXY)` gate is the whole mechanism — the telemetry heading
  // sources are not even looked at outside it, so there is no path by which a
  // simulated pose renders at anything other than FORWARD.
  let heading_deg: number = ROVER_START.heading_deg; // = FORWARD_HEADING_DEG
  if (hasXY) {
    const hPose = pick(d, "heading_deg");
    const hLidar = pick(pick(lidarData, "data") ?? lidarData, "heading_deg");
    if (isNum(hPose)) heading_deg = hPose;
    else if (isNum(hLidar)) heading_deg = hLidar;
  }

  const heading_rad = (heading_deg * Math.PI) / 180;

  // A NaN slipping through here would poison every downstream coordinate, so
  // clamp one last time rather than trusting the checks above. The fallback is
  // FORWARD, not 0 — 0 rad is "facing +x", which is a real orientation and
  // would be indistinguishable from a measured heading pointing right.
  const FORWARD_RAD = (FORWARD_HEADING_DEG * Math.PI) / 180;
  return {
    x_mm: Number.isFinite(x_mm) ? x_mm : ROVER_START.x_mm,
    y_mm: Number.isFinite(y_mm) ? y_mm : ROVER_START.y_mm,
    heading_rad: Number.isFinite(heading_rad) ? heading_rad : FORWARD_RAD,
    simulated: !hasXY,
  };
}

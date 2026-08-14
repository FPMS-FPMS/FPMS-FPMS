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
 *
 * ---------------------------------------------------------------------------
 * THE ZONE NAMING TRAP
 * ---------------------------------------------------------------------------
 * There are two numbering schemes in play and they DO NOT AGREE.
 *
 *   the code    calls the top-LEFT zone `m1` and the top-RIGHT zone `m2`.
 *   the operator calls the zone DIRECTLY IN FRONT OF THE START BOX "Zone 1".
 *
 * The start box is bottom-RIGHT and the rover starts facing 90 deg (up the
 * arena), so the zone in front of it is the top-RIGHT one — `zone-b`, which
 * the mission code calls `m2`. The operator's "Zone 1" is therefore the code's
 * `m2`, not its `m1`.
 *
 * Nothing here renumbers anything: renaming one scheme to match the other just
 * moves the confusion. The fix is that NO LABEL ON THE MAP IS EVER A BARE
 * NUMBER. Every region carries the physical corner it occupies (`corner`
 * below), because the corner is the one description the operator standing at
 * the arena and the code running the mission cannot disagree about. The start
 * box additionally names the zone it faces (`ZONE_AHEAD_OF_START`), which is
 * derived from the geometry rather than typed in, so the map cannot claim a
 * relationship the coordinates do not support.
 */

/**
 * Arena extents in millimetres — 150 cm wide x 120 cm tall (RECTANGULAR).
 *
 * These mirror fpms_missions.py ARENA_W_MM / ARENA_H_MM exactly and are the
 * single source of truth here. Zones, grid spacing, wall-detection thresholds
 * and the rover start pose are all expressed as fractions of them, so changing
 * these numbers rescales the entire map coherently.
 *
 * OPERATOR-MEASURED 2026-08-14: the y (drive) extent was SHORTENED by 200 mm.
 * The start box is measured from the BOTTOM edge, so ROVER_START is unchanged
 * at (1322, 178); only the two top zones moved down with the far wall.
 */
export const ARENA_W_MM = 1500;
export const ARENA_H_MM = 1200;

/**
 * The SQUARE extent everything sized off one number still uses: the viewport,
 * the grid pitch, the y flip and the in-bounds checks. It must be the LARGER
 * of the two extents, or the far side of the arena falls outside the drawing
 * window and a legitimate pose is refused as "outside the arena" — which is
 * exactly the bug that drew the rover off the grid when this was 1200.
 */
export const ARENA_MM = Math.max(ARENA_W_MM, ARENA_H_MM);

/** Rover chassis footprint, millimetres. Nose-to-tail x beam. */
export const ROVER_LEN_MM = 235;
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

/**
 * The physical corner of the arena a region sits in.
 *
 * This is the label the operator reads off the map, and it is deliberately a
 * direction rather than a number — see THE ZONE NAMING TRAP at the top of this
 * file. "TOP" means the far side of the arena from the start box, which is the
 * side the rover faces at FORWARD_HEADING_DEG.
 */
export type ArenaCorner = "TOP-LEFT" | "TOP-RIGHT" | "BOTTOM-LEFT" | "BOTTOM-RIGHT";

/**
 * The same corner, abbreviated, for when the full name will not fit.
 *
 * Only BOTTOM is shortened, and only to BTM — the half that carries the
 * meaning (LEFT / RIGHT) is never touched, and neither name can be confused
 * with the other three. This exists so that a narrow card DEGRADES the corner
 * label rather than DROPPING it: on a phone-width map the region name plus
 * "BTM-LEFT" still fits where "BOTTOM-LEFT" does not, and the corner is the
 * one piece of text on this map that must not go missing.
 */
export function shortCorner(corner: ArenaCorner): string {
  return corner.startsWith("BOTTOM-") ? `BTM-${corner.slice(7)}` : corner;
}

/**
 * What a region is FOR, which is not a cosmetic distinction.
 *
 *   fire   a mission destination. The rover drives to it and discharges.
 *   water  the refill point. The rover drives to it to take water ON BOARD.
 *
 * An operator who mistakes the refill point for a target sends the rover to
 * spray the water station, so the two kinds are drawn as different objects
 * (hatched fill, droplet glyph, REFILL role text) and not merely in different
 * hues — hue alone fails for a colour-blind operator and fails again on a
 * washed-out screen in daylight.
 */
export type ZoneKind = "fire" | "water";

/** A named region painted on the static layer. Rect anchored BOTTOM-LEFT. */
export type Zone = {
  id: string;
  /** Short name, e.g. "ZONE A". Never a bare number. */
  label: string;
  /** Physical corner, always drawn directly under `label`. */
  corner: ArenaCorner;
  /** One-word purpose, drawn when the region is large enough to take it. */
  role: string;
  kind: ZoneKind;
  /** world mm, bottom-left corner */
  x_mm: number;
  y_mm: number;
  w_mm: number;
  h_mm: number;
  /**
   * Hue that identifies this region. The renderer derives the wash, the edge,
   * the corner brackets and the label ink from this ONE value, so a zone can
   * never end up with an edge in one colour and a fill in another — which is
   * how a region ends up drawn but invisible.
   */
  stroke: string;
};

/** Zone side and margin, as fractions of the arena so a rescale is free. */
export const ZONE_SIDE_MM = 0.1 * ARENA_W_MM;
export const ZONE_MARGIN_MM = 0.04 * ARENA_W_MM;
const Z = ZONE_SIDE_MM;
const M = ZONE_MARGIN_MM;

/**
 * The three named regions, in the three corners that are not the start box.
 *
 * HUES ARE ALLOCATED, NOT CHOSEN. The map already spends orange on the arena
 * border and the measured rover, amber on ASSUMED pose and OBSTACLE tracks,
 * emerald on MEASURED pose AND on the actual driven path, green on TREE tracks,
 * rose on danger, blue on the planned route, and slate on walls. A zone painted
 * in any of those reads as one of those. Violet, lime and cyan are what is
 * left, and they are far enough apart on the wheel to survive a 17 % wash.
 *
 * `zone-a` used to be sky #38bdf8 — one hue step from the water station's cyan
 * and the same hue as the measured trail. Three regions the operator has to
 * tell apart at a glance had two nearly identical colours between them.
 *
 * The driven path used to be that same sky, one step from the planned route's
 * sky-300 and a third step from the water station's cyan, which made the
 * plan-versus-actual comparison a contrast between two near-identical blues and
 * hid the plan wherever it entered the refill point. The driven path now takes
 * emerald — the MEASURED ink, which is what it is — and the plan takes a true
 * blue. Neither is ever the ONLY cue: planned is dashed and thin, driven is
 * solid and heavier.
 */
export const ZONES: readonly Zone[] = [
  {
    id: "zone-a",
    label: "ZONE A",
    corner: "TOP-LEFT",
    role: "FIRE ZONE",
    kind: "fire",
    x_mm: M,
    y_mm: ARENA_H_MM - M - Z, // top-left
    w_mm: Z,
    h_mm: Z,
    stroke: "#c084fc",
  },
  {
    id: "zone-b",
    label: "ZONE B",
    corner: "TOP-RIGHT",
    role: "FIRE ZONE",
    kind: "fire",
    x_mm: ARENA_W_MM - M - Z,
    y_mm: ARENA_H_MM - M - Z, // top-right
    w_mm: Z,
    h_mm: Z,
    stroke: "#a3e635",
  },
  {
    id: "water-station",
    label: "WATER",
    corner: "BOTTOM-LEFT",
    role: "REFILL",
    kind: "water",
    x_mm: M,
    y_mm: M, // bottom-left
    w_mm: Z,
    h_mm: Z,
    stroke: "#22d3ee",
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
 * The START BOX: bottom-right, mirroring the three named zones in size and
 * inset so the four corners of the arena read as one family of regions.
 *
 * It is deliberately NOT a member of ZONES. Zones are mission destinations the
 * rover drives to; this is the one region whose position is an ASSUMPTION —
 * where the operator is asked to place the rover before a run, and where the
 * map draws it when nothing publishes a position. The static layer paints it
 * in its own dashed style for exactly that reason, and `ROVER_START` below is
 * derived from it so the box and the glyph can never disagree.
 */
export const START_BOX = {
  x_mm: ARENA_W_MM - M - ROVER_LEN_MM,
  y_mm: M,
  w_mm: ROVER_LEN_MM,
  h_mm: ROVER_LEN_MM,
  label: "START",
  corner: "BOTTOM-RIGHT" as ArenaCorner,
} as const;

/**
 * The zone the rover is looking at when it sits in the start box, or null if
 * no zone is in front of it.
 *
 * DERIVED FROM THE GEOMETRY, never typed in. Forward is +y (see
 * FORWARD_HEADING_DEG), so "ahead" means: centred in the same vertical lane as
 * the start box, and further up the arena. Whichever zone satisfies that is
 * the operator's "Zone 1" — the trap documented at the top of this file — and
 * the map prints its name inside the start box so nobody has to hold the two
 * numbering schemes in their head.
 *
 * Deriving it means the caption cannot go stale: move a zone or move the start
 * box and the arrow follows, or disappears if nothing is ahead any more. A
 * hardcoded "AHEAD: ZONE B" would keep claiming a relationship the coordinates
 * had stopped supporting, which is exactly the class of bug that produced the
 * naming trap in the first place.
 */
export const ZONE_AHEAD_OF_START: Zone | null = (() => {
  const cx = START_BOX.x_mm + START_BOX.w_mm / 2;
  const cy = START_BOX.y_mm + START_BOX.h_mm / 2;
  let best: Zone | null = null;
  let bestDy = Infinity;
  for (const z of ZONES) {
    const zcx = z.x_mm + z.w_mm / 2;
    const zcy = z.y_mm + z.h_mm / 2;
    const dy = zcy - cy;
    if (dy <= 0) continue; // behind or alongside: not ahead at heading 90
    if (Math.abs(zcx - cx) > z.w_mm / 2) continue; // out of the forward lane
    if (dy < bestDy) {
      bestDy = dy;
      best = z;
    }
  }
  return best;
})();

/**
 * Where the rover is assumed to be when no odometry is published: the centre
 * of the start box, nose pointed FORWARD — straight up the arena.
 *
 * The position is unchanged (bottom-right start box). Only the heading is
 * pinned to FORWARD_HEADING_DEG. The previous 135 deg diagonal was a
 * presentation choice — it fanned the scan across the map — but it made the
 * map open at a slant, which reads as a bug to an operator who expects the
 * bird's-eye view to start and stay facing forward.
 */
export const ROVER_START = {
  x_mm: START_BOX.x_mm + START_BOX.w_mm / 2,
  y_mm: START_BOX.y_mm + START_BOX.h_mm / 2,
  heading_deg: FORWARD_HEADING_DEG,
} as const;

/** Wrap any angle into [0, 360). Returns null for anything non-finite. */
export function normDeg(deg: number): number | null {
  if (!Number.isFinite(deg)) return null;
  return ((deg % 360) + 360) % 360;
}

/**
 * Provenance of one pose component.
 *
 *   measured  the rover reported this number. It is dead-reckoned and it
 *             drifts, but it came off the robot.
 *   assumed   NOBODY reported it. The value is a constant from ROVER_START
 *             and means "we put it here", not "it is here".
 *
 * The renderer draws these two differently on purpose. Nothing localises this
 * rover, so the difference between "the odometry says 840 mm" and "we assumed
 * 1020 mm because there is no odometry" is the single most important thing on
 * the map, and it must be visible without reading any text.
 */
export type PoseSource = "measured" | "assumed";

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
  /** Provenance of x_mm/y_mm. `simulated` is exactly `position === "assumed"`. */
  position: PoseSource;
  /**
   * Provenance of heading_rad, tracked SEPARATELY from the position because
   * the two really do arrive apart: `poseEnvelopeFromMm` omits heading_deg
   * when the bridge has a position but no yaw. That case draws a rover whose
   * dot is measured and whose nose is a guess, and the only way to say so is
   * to keep the two flags distinct.
   */
  heading: PoseSource;
  /** Drawn heading in degrees, wrapped to [0,360). Never null — see below. */
  heading_deg: number;
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
  let headingMeasured = false;
  if (hasXY) {
    const hPose = pick(d, "heading_deg");
    const hLidar = pick(pick(lidarData, "data") ?? lidarData, "heading_deg");
    if (isNum(hPose)) {
      heading_deg = hPose;
      headingMeasured = true;
    } else if (isNum(hLidar)) {
      // Read, but NOT promoted to "measured".
      //
      // The LiDAR payload hard-codes `heading_deg: 0`. That zero is a struct
      // field the firmware fills in, not a bearing anything on this rover
      // observed — nothing here has a compass and the yaw that does exist is
      // integrated gyro, which arrives on the pose envelope above, not this
      // one. Drawing it is harmless because the value is kept and the glyph
      // renders in the ASSUMED style; labelling it MEASURED would put a
      // firmware constant on screen as an observation, which is the same
      // mistake as inventing a coordinate.
      heading_deg = hLidar;
    }
  }

  const heading_rad = (heading_deg * Math.PI) / 180;

  // A NaN slipping through here would poison every downstream coordinate, so
  // clamp one last time rather than trusting the checks above. The fallback is
  // FORWARD, not 0 — 0 rad is "facing +x", which is a real orientation and
  // would be indistinguishable from a measured heading pointing right.
  const FORWARD_RAD = (FORWARD_HEADING_DEG * Math.PI) / 180;
  const okHdg = Number.isFinite(heading_rad);
  const drawnDeg = normDeg(okHdg ? heading_deg : FORWARD_HEADING_DEG);

  return {
    x_mm: Number.isFinite(x_mm) ? x_mm : ROVER_START.x_mm,
    y_mm: Number.isFinite(y_mm) ? y_mm : ROVER_START.y_mm,
    heading_rad: okHdg ? heading_rad : FORWARD_RAD,
    // `simulated` stays derived from hasXY ALONE. It is the flag the badge
    // hangs off, and the only way to clear it is for real x_m/y_m to arrive —
    // there is deliberately no argument, option or fallback that can set it
    // false while the position is still a constant from ROVER_START.
    simulated: !hasXY,
    position: hasXY ? "measured" : "assumed",
    // A heading that failed the finite check is an assumption whatever the
    // position did: we drew FORWARD, and FORWARD is our choice, not the
    // rover's.
    heading: headingMeasured && okHdg ? "measured" : "assumed",
    heading_deg: drawnDeg === null ? FORWARD_HEADING_DEG : drawnDeg,
  };
}

// ------------------------------------------------------------------ trail ---

/** Ring capacity. At the 2 Hz telemetry rate this is ~4 minutes of history. */
const TRAIL_CAP = 480;

/**
 * Minimum movement before a new trail sample is kept, millimetres.
 *
 * Dead-reckoned position jitters by a few mm while parked. Without this gate a
 * stationary rover would fill the whole ring with a fuzzy dot and evict the
 * route it actually drove — the history would be destroyed by standing still.
 */
const TRAIL_MIN_STEP_MM = 8;

/**
 * A position change larger than this is a TELEPORT, not a drive.
 *
 * `set_coordinate` re-zeros the origin, and an origin re-zero moved this rover
 * from (-9182,-11926) to a sane pose in one message. Joining those two samples
 * with a line would draw a metre-long path the rover never took — a fabricated
 * history, which is the same sin as a fabricated position. The sample is kept
 * (it is where the rover is now) but flagged as a BREAK so the renderer lifts
 * the pen.
 */
const TRAIL_JUMP_MM = 250;

export const TRAIL_MEASURED = 1;
export const TRAIL_BREAK = 2;

/**
 * Bounded history of where the rover has been.
 *
 * Fixed-capacity ring over two Float32Arrays and a flag byte, all allocated
 * once. `push` does no allocation and takes no closures, so the rAF loop above
 * can call it every time a pose arrives without producing GC sawtooth.
 *
 * Samples carry their own provenance. A trail drawn from assumed poses is not
 * a record of travel — it is a record of the map's guess — so the renderer
 * needs the flag per point, not per trail.
 */
export class PoseTrail {
  readonly cap: number;
  readonly xs: Float32Array;
  readonly ys: Float32Array;
  /** bit 0 = TRAIL_MEASURED, bit 1 = TRAIL_BREAK (do not join to previous). */
  readonly flags: Uint8Array;
  count = 0;
  /**
   * Path length of the CURRENT unbroken stroke, millimetres. Reset to 0 by a
   * BREAK.
   *
   * This is the odometer, and it is the honest input to the drift picture. A
   * break is the last moment the tracker was re-anchored to something — a
   * re-zero, or the flip from an assumed pose to a reported one — so distance
   * since the break is exactly "how far this rover has dead-reckoned without
   * anything correcting it". Nothing localises this rover, so that number only
   * ever goes up, and the map is supposed to say so.
   *
   * Accumulated from the SAMPLES that were kept, so the TRAIL_MIN_STEP_MM gate
   * means it slightly under-counts a wandering path. It under-reports rather
   * than over-reports, which is the correct direction for a number the operator
   * reads as "at least this far since anyone knew where I was".
   */
  drivenMm = 0;
  /** Path length over the whole surviving history, breaks excluded. */
  totalMm = 0;
  private head = 0;

  constructor(cap: number = TRAIL_CAP) {
    this.cap = cap > 1 ? cap | 0 : 2;
    this.xs = new Float32Array(this.cap);
    this.ys = new Float32Array(this.cap);
    this.flags = new Uint8Array(this.cap);
  }

  /** Oldest-first ordering: `at(0)` is the eldest surviving sample. */
  at(i: number): number {
    return (this.head - this.count + i + this.cap * 2) % this.cap;
  }

  clear(): void {
    this.count = 0;
    this.head = 0;
    this.drivenMm = 0;
    this.totalMm = 0;
  }

  /**
   * Record a pose. Returns true when the sample opened a new stroke — a jump,
   * a provenance flip, or the first point — which is the caller's cue that the
   * world it had been tracking is no longer the world it is looking at.
   */
  push(x: number, y: number, measured: boolean): boolean {
    if (!Number.isFinite(x) || !Number.isFinite(y)) return false;

    let brk = true;
    let stepMm = 0;
    if (this.count > 0) {
      const last = (this.head - 1 + this.cap) % this.cap;
      const wasMeasured = (this.flags[last] & TRAIL_MEASURED) !== 0;
      const dx = x - this.xs[last];
      const dy = y - this.ys[last];
      const d2 = dx * dx + dy * dy;
      const flipped = wasMeasured !== measured;
      if (!flipped && d2 < TRAIL_MIN_STEP_MM * TRAIL_MIN_STEP_MM) return false;
      brk = flipped || d2 > TRAIL_JUMP_MM * TRAIL_JUMP_MM;
      // A break is a teleport, and the gap across it is not distance the rover
      // drove. Counting it would inflate the odometer with a jump the whole
      // BREAK mechanism exists to refuse to draw.
      if (!brk) stepMm = Math.sqrt(d2);
    }
    if (brk) this.drivenMm = 0;
    else {
      this.drivenMm += stepMm;
      this.totalMm += stepMm;
    }

    const h = this.head;
    this.xs[h] = x;
    this.ys[h] = y;
    this.flags[h] = (measured ? TRAIL_MEASURED : 0) | (brk ? TRAIL_BREAK : 0);
    this.head = (h + 1) % this.cap;
    if (this.count < this.cap) this.count++;
    return brk;
  }
}

export function createPoseTrail(cap?: number): PoseTrail {
  return new PoseTrail(cap);
}

// -------------------------------------------------------------- uncertainty ---

/**
 * How far the uncertainty rings are drawn — and, deliberately, NOT an error bar.
 *
 * ---------------------------------------------------------------------------
 * READ THIS BEFORE PUTTING A NUMBER ON SCREEN NEXT TO THE RINGS
 * ---------------------------------------------------------------------------
 * Nobody has measured this rover's positional error. There is no localisation,
 * no ground-truth run, no repeatability figure — the pose is integrated wheel
 * odometry from a start the operator eyeballed into a 36 cm box. Any millimetre
 * figure this file returned would be invented, and an invented error bar is
 * worse than none: it is the one number an operator would plan around.
 *
 * So the contract is one-directional. This function turns a MEASURED input (the
 * odometer, `PoseTrail.drivenMm`) into a PRESENTATION radius in millimetres,
 * used for nothing but sizing a pulsing ring family that has no hard edge. The
 * renderer never prints it, and no caller may. What the HUD prints is the
 * input — "DRIVEN 1240 mm" — which is a real quantity the rover reported.
 *
 * The shape is linear because uncorrected differential-drive odometry error
 * grows with path length, not with wall-clock time: a parked rover stops
 * getting more lost, a driving one does not. `DRIFT_SPREAD_FRAC` is a drawing
 * constant chosen so the ring is legible on a 120 cm arena, not a calibration.
 */
export const DRIFT_SPREAD_FRAC = 0.06;

/**
 * Ring spread for an ASSUMED position, millimetres.
 *
 * An assumed pose has driven nowhere — the glyph is pinned to ROVER_START — so
 * a distance-driven model would size its ring at zero, which would state the
 * exact opposite of the truth. The truth is that the rover is somewhere in the
 * start box, so the ring opens at the box's half-width and the operator reads
 * "somewhere in there", which is all anybody actually knows.
 */
export const ASSUMED_SPREAD_MM = START_BOX.w_mm / 2;

/** Rendering-only. See DRIFT_SPREAD_FRAC — never label the result as an error. */
export function driftSpreadMm(drivenMm: number): number {
  if (!Number.isFinite(drivenMm) || drivenMm <= 0) return 0;
  return drivenMm * DRIFT_SPREAD_FRAC;
}

// ------------------------------------------------------- plan versus actual ---

/** Anything with world-millimetre coordinates: a waypoint, a pose, a track. */
export type PathPoint = { readonly x_mm: number; readonly y_mm: number };

/**
 * Result of `nearestOnPath`, filled in place.
 *
 * An out-parameter rather than a return value because this is called from the
 * render path's 2 Hz edge and a fresh object per call is the allocation pattern
 * the whole renderer is written to avoid.
 */
export type NearestOnPath = {
  /** Distance from the query point to the path, mm. Infinity when no path. */
  dist_mm: number;
  /** The closest point ON the path, world mm. NaN when there is no path. */
  x_mm: number;
  y_mm: number;
  /** Index of the segment's first waypoint, or -1. */
  seg: number;
};

export function createNearestOnPath(): NearestOnPath {
  return { dist_mm: Infinity, x_mm: NaN, y_mm: NaN, seg: -1 };
}

/**
 * Closest point on a planned polyline to (x, y) — the cross-track deviation.
 *
 * WHY PERPENDICULAR AND NOT "DISTANCE TO THE NEXT WAYPOINT". A route-following
 * test asks whether the rover stayed ON the planned line, and distance to the
 * next waypoint answers a different question: it is large at the start of every
 * leg and small at the end of it, so it drops to nearly zero on a rover that
 * cut a corner badly and swung back. Projecting onto the nearest SEGMENT gives
 * the number an operator actually wants — how far off the intended track the
 * rover is right now — and it is comparable between legs.
 *
 * Clamped to each segment so a rover past the final waypoint measures to the
 * end of the route rather than to its infinite extension.
 *
 * O(n) over a route of a handful of waypoints, allocation-free, and called on
 * the telemetry edge rather than per frame.
 */
export function nearestOnPath(
  path: readonly PathPoint[] | null | undefined,
  x: number,
  y: number,
  out: NearestOnPath,
): void {
  out.dist_mm = Infinity;
  out.x_mm = NaN;
  out.y_mm = NaN;
  out.seg = -1;
  if (!path || path.length < 2 || !Number.isFinite(x) || !Number.isFinite(y)) return;

  let best = Infinity;
  for (let i = 0; i < path.length - 1; i++) {
    const ax = path[i].x_mm;
    const ay = path[i].y_mm;
    const bx = path[i + 1].x_mm;
    const by = path[i + 1].y_mm;
    if (!Number.isFinite(ax) || !Number.isFinite(ay)) continue;
    if (!Number.isFinite(bx) || !Number.isFinite(by)) continue;

    const vx = bx - ax;
    const vy = by - ay;
    const len2 = vx * vx + vy * vy;
    // A zero-length segment (two identical waypoints) projects to its own
    // endpoint rather than dividing by zero.
    let t = len2 > 0 ? ((x - ax) * vx + (y - ay) * vy) / len2 : 0;
    if (t < 0) t = 0;
    else if (t > 1) t = 1;

    const px = ax + vx * t;
    const py = ay + vy * t;
    const dx = x - px;
    const dy = y - py;
    const d2 = dx * dx + dy * dy;
    if (d2 < best) {
      best = d2;
      out.x_mm = px;
      out.y_mm = py;
      out.seg = i;
    }
  }
  out.dist_mm = best === Infinity ? Infinity : Math.sqrt(best);
}

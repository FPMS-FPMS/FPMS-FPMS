import { memo, useEffect, useRef } from "react";
import {
  ARENA_MM,
  ASSUMED_SPREAD_MM,
  FORWARD_HEADING_DEG,
  GRID_MAJOR_MM,
  GRID_MINOR_MM,
  ROVER_LEN_MM,
  ROVER_WID_MM,
  START_BOX,
  TRAIL_BREAK,
  TRAIL_MEASURED,
  ZONES,
  ZONE_AHEAD_OF_START,
  createNearestOnPath,
  createPoseTrail,
  driftSpreadMm,
  nearestOnPath,
  shortCorner,
  type ArenaCorner,
  type NearestOnPath,
  readPose,
  scaleFor,
  worldToCanvasX,
  worldToCanvasY,
  type Pose,
  type PoseTrail,
} from "../lib/arena";
import {
  CLASS_COLORS_DARK,
  CLASS_COLORS_LIGHT,
  CLASS_NAMES,
  CLS_OBSTACLE,
  CLS_TREE,
  createClusterEngine,
  isLabelable,
  type ArenaFrame,
  type ClusterEngine,
} from "../lib/lidarCluster";
import { useChannelRef } from "../lib/ws";

/**
 * World-fixed bird's-eye view of the arena.
 *
 * TWO SEPARATE GUARANTEES, both of which the operator asked for by name:
 *
 *   1. THE MAP STARTS FORWARD AND STAYS FORWARD. The world layer is fixed.
 *      Nothing in this file ever rotates the arena, the grid, the zones, the
 *      axis ticks or the scale bar by rover heading — there is exactly one
 *      ctx.rotate() in the whole module, inside drawRover's save()/restore(),
 *      and it turns the ROVER GLYPH ONLY. The old polar view was
 *      rover-centric, so a turning rover made the entire world appear to
 *      spin, which is unreadable when you are trying to tell where an
 *      obstacle actually is. Cloud points and object boxes arrive in world
 *      millimetres and go through the same fixed transform as the grid.
 *
 *   2. THE ROVER STARTS FACING FORWARD. "Forward" is up the arena
 *      (world +y, towards the top of the screen) = FORWARD_HEADING_DEG,
 *      90 deg CCW from +x. The static layer paints a FORWARD arrow above the
 *      top edge so that convention is on screen, not just in a comment.
 *
 * Because the map never rotates, the FORWARD marker is a permanent, honest
 * reference: it belongs to the static layer precisely so it can never be
 * mistaken for something that tracks the rover.
 *
 * ---------------------------------------------------------------------------
 * THE COORDINATE SYSTEM IS THE POINT
 * ---------------------------------------------------------------------------
 * This is a metric map, not a picture of a robot, and the requirement is that
 * an operator can read the rover's position TO THE CENTIMETRE without leaving
 * the map. Four things carry that, and they are all on the STATIC layer except
 * the last:
 *
 *   the ladder   a tick every 5 cm down both gutters, labelled every 30 cm, so
 *                a position can be read off the edge by counting ticks. The
 *                grid behind it is the same ladder drawn across the floor —
 *                minor lines ARE the 5 cm ticks, so eye and edge agree.
 *   the origin   world (0,0) is stated, not implied: a ringed dot in the
 *                bottom-left corner with the axis senses (+X right, +Y up)
 *                drawn as arrows off it. A map whose origin is a guess is a map
 *                whose every coordinate is a guess.
 *   the key      a scale bar with a mid-division, plus the two path styles.
 *                Bottom-centre, in the one strip of floor no region claims.
 *   the cursor   the rover's x and y printed IN THE GUTTER, on the axis, where
 *                the reticle lines meet it. This is the part that makes the
 *                centimetre readable without a lookup: follow the dashed line
 *                out to the edge and the number is already there. Dynamic, but
 *                the strings are rebuilt on the 2 Hz telemetry edge only.
 *
 * ---------------------------------------------------------------------------
 * PLANNED VERSUS ACTUAL
 * ---------------------------------------------------------------------------
 * A route-following test exists to compare what was intended with what was
 * achieved, so the two tracks must be separable at a glance and must never be
 * confused for one another:
 *
 *   planned  blue, DASHED, thin, with hollow waypoint nodes. Intent.
 *   actual   emerald when the pose is measured, amber when it is assumed;
 *            SOLID, thicker, fading with age. Achieved.
 *
 * The two used to be sky-300 and sky-400 — the same hue one step apart, for the
 * one comparison this map exists to support, with the cyan water station making
 * it a three-way collision (the plan against the water measured dE 5.9 for
 * NORMAL colour vision). The driven path now takes `theme.measured`, the same
 * emerald as the POSE MEASURED badge, so the ink itself says where the line
 * came from, and the plan took a true blue. Hue is never the only cue: dashed
 * versus solid and 1.5 px versus 2 px separate them in greyscale and for a
 * colour-blind operator, and the key at the bottom names both.
 *
 * The comparison is also made NUMERIC. `nearestOnPath` projects the rover onto
 * the nearest route SEGMENT — cross-track deviation, not distance-to-waypoint,
 * which is a different and much less useful number (see arena.ts) — and the HUD
 * prints it as PLAN DEV alongside a link drawn from the rover to that point.
 * The link is the answer to "am I on the line", visible without reading digits.
 *
 * ---------------------------------------------------------------------------
 * MEASURED VERSUS ASSUMED
 * ---------------------------------------------------------------------------
 * Nothing localises this rover. Its best case is a dead-reckoned pose that
 * drifts; its normal case is no pose at all, drawn at the assumed start. Those
 * two cases must not look alike, so the glyph itself changes:
 *
 *   measured  solid accent chassis, solid nose, solid heading ray.
 *   assumed   amber, dashed outline, hollow nose, and a pulsing uncertainty
 *             halo that says "somewhere around here" without naming a radius
 *             nobody has measured.
 *
 * The badge is the backstop, not the mechanism — an operator who never reads
 * the text still cannot mistake the amber ghost for a fix. `Pose.simulated`
 * comes straight out of `readPose` and can only be cleared by real x_m/y_m
 * arriving, so no code path here can silence the warning with a coordinate it
 * made up.
 *
 * DRIFT GROWS, AND THE RINGS GROW WITH IT. A measured pose here is still dead
 * reckoning from a start somebody eyeballed, with nothing correcting it, so its
 * uncertainty is not a constant — it increases with every millimetre driven.
 * The ring family is therefore sized from `PoseTrail.drivenMm`, the odometer
 * since the last break, and it opens as the rover works. Two rules keep that
 * honest and keep it from collapsing the measured/assumed distinction:
 *
 *   - IT IS STILL NOT AN ERROR BAR. Concentric, pulsing, no hard edge, and the
 *     millimetre figure is never printed — see DRIFT_SPREAD_FRAC in arena.ts.
 *     What the HUD prints is DRIVEN, the measured input, not a fabricated bound.
 *   - AN ASSUMED POSE OPENS AT THE START BOX and does not grow, because it has
 *     driven nowhere; its rings are amber and sit around a glyph that is
 *     dashed and hollow. A measured pose opens at nothing and grows, in the
 *     measured ink, around a solid glyph. Same visual family, opposite claims,
 *     still impossible to confuse.
 *
 * ---------------------------------------------------------------------------
 * THE ARENA MUST BE FINDABLE
 * ---------------------------------------------------------------------------
 * An operator opened this dashboard and asked where the two zones and the
 * water station were. They were being drawn the whole time — a 10 % wash and a
 * 9 px name on a near-black floor — which is the same outcome as not drawing
 * them. A region that is technically painted and practically invisible is a
 * bug, not a style choice.
 *
 * So each of the four corner regions is now stated four independent ways
 * (wash, corner brackets, a label plate, and a shape glyph) — see drawRegion.
 * Two rules hold across all of them:
 *
 *   - EVERY LABEL NAMES ITS PHYSICAL CORNER. The mission code's m1/m2 and the
 *     operator's "Zone 1" number the same two zones differently (arena.ts,
 *     THE ZONE NAMING TRAP), so no label on this map is ever a bare number.
 *     The start box additionally prints the zone it faces.
 *   - THE WATER STATION IS NOT A TARGET. It is hatched, double-edged and
 *     marked with a droplet, because it is where the rover refills, and an
 *     operator who reads it as a third fire zone sends the rover to spray it.
 *
 * ---------------------------------------------------------------------------
 * LAYERS AND THE FRAME BUDGET
 * ---------------------------------------------------------------------------
 *   static   arena border, grid, zones, start box, the axis ladder and its
 *            labels, the origin marker, the key (scale bar + path styles),
 *            heading legend, FORWARD marker. Redrawn on mount, on resize and
 *            on a theme change.
 *   dynamic  route, driven path, plan-deviation link, cloud, objects, rover,
 *            gutter cursor, HUD. Redrawn per animation frame.
 *
 * React renders this component approximately once. The LiDAR socket writes to
 * refs (useChannelRef), a rAF loop polls the sequence number, and clustering
 * runs only when the number changes — the stream is 2 Hz, so re-segmenting at
 * 60 Hz would be thirty times the work for the same picture. The HUD strings
 * are rebuilt on the same 2 Hz edge for the same reason: template literals in
 * a rAF loop are per-frame allocation, and this file has none. Dash patterns
 * are module constants because `setLineDash([4,4])` allocates an array every
 * time it is called.
 */

/**
 * Gutter around the arena, px. This is where the coordinate system lives.
 *
 * Sized by what has to fit, not by taste: a 5 px major tick, a 9 px centimetre
 * label under it, and the live cursor plate that prints the rover's coordinate
 * on the ladder. At 34 px the cursor plate did not fit beside a three-digit
 * label in the left gutter and would have been clipped by the card edge, which
 * is the one failure mode a readout must not have. The six pixels come out of
 * the arena, which is a ~2 % smaller floor for a coordinate system that can
 * actually be read.
 */
const PAD_PX = 40;

const EMBER = "#f97316";
const MONO = '10px "JetBrains Mono", ui-monospace, monospace';
const MONO_SM = '9px "JetBrains Mono", ui-monospace, monospace';

/**
 * Region titles, in three fixed sizes rather than one composed per draw.
 *
 * A region on this map is between about 50 px (a narrow phone card) and about
 * 150 px (the full-width panel) across, and 9 px text that is comfortable at
 * the top of that range is what made the zones unreadable at the bottom of it.
 * Semi-bold because these names sit on a busy floor and have to win.
 */
const ZONE_TITLE_LG = '600 12px "JetBrains Mono", ui-monospace, monospace';
const ZONE_TITLE_MD = '600 11px "JetBrains Mono", ui-monospace, monospace';
const ZONE_TITLE_SM = '600 10px "JetBrains Mono", ui-monospace, monospace';

/** Hoisted so the per-frame path setup allocates nothing. */
const DASH_NONE: number[] = [];
const DASH_RETICLE = [2, 4];
const DASH_TRAIL = [3, 3];
const DASH_ROVER = [3, 2.5];
const DASH_HALO = [2, 5];
const DASH_ROUTE = [5, 4];
const DASH_START = [4, 3];
const DASH_HEADING = [4, 4];
/** The rover-to-plan deviation link. Tight dots: a measurement, not a path. */
const DASH_DEV = [1, 3];

// ================================================================== theme ===

/**
 * Every colour the map uses, in one table per background.
 *
 * The dashboard ships dark-only today (`:root { color-scheme: dark }`, a
 * hard-coded dark body), so `THEME_DARK` is what renders. The light table is
 * not speculative decoration: a canvas paints its own pixels and inherits
 * nothing, so a map built against one background is invisible on the other,
 * and the failure only shows up after somebody adds a light theme and this
 * card turns into a black square. Both tables are the cost of that not
 * happening.
 */
type ArenaTheme = {
  id: "dark" | "light";
  surround: string;
  floor: string;
  gridMinor: string;
  gridMajor: string;
  border: string;
  text: string;
  textDim: string;
  forward: string;
  /** Dashed outline of the start box. */
  startInk: string;
  /** Start box label. Brighter than `startInk` — an outline may whisper, a name may not. */
  startText: string;
  /** HUD panel wash and the outline behind on-map labels. */
  scrim: string;
  halo: string;
  /**
   * The ACTUAL DRIVEN PATH, and deliberately the same ink as `measured`.
   *
   * This was sky (#38bdf8 dark / #0369a1 light) — one hue step from the planned
   * route's sky-300, which made the single comparison this map exists to
   * support, intended versus achieved, a contrast between two nearly identical
   * blues. Emerald is `theme.measured`, the POSE MEASURED badge's colour, so
   * the driven line and the badge that vouches for it now match, and the plan
   * moved out of the blue family it was sharing with the water station.
   * Validated against the planned route on both surfaces — see ROUTE_INK_DARK
   * for the runs — at dE 27.2 normal / 26.1 deutan on this floor; dashed versus
   * solid and 1.5 px versus 2 px carry it with no colour at all.
   */
  trailMeasured: string;
  trailAssumed: string;
  /** Provenance inks. Amber = assumed, emerald = measured. */
  assumed: string;
  measured: string;
  danger: string;
  cloudNoise: number;
  classColors: readonly string[];
  /** Zone strokes are mixed this far toward this colour for legibility. */
  zoneMix: string;
  zoneMixT: number;
  /**
   * Alpha of the zone wash.
   *
   * This is the number that decides whether the arena has visible zones at
   * all. It was 0.10 on a #0b0f16 floor, which is a wash of about three RGB
   * steps — present in the buffer, absent to the eye, and the reason an
   * operator opened the dashboard and asked where the zones were. The floor is
   * near-black, so the wash needs enough alpha to lift the region clear of it.
   */
  zoneFill: number;
};

const THEME_DARK: ArenaTheme = {
  id: "dark",
  surround: "#07090d",
  floor: "#0b0f16",
  gridMinor: "rgba(148,163,184,0.055)",
  gridMajor: "rgba(148,163,184,0.14)",
  border: "rgba(249,115,22,0.55)",
  text: "#e2e8f0",
  textDim: "rgba(148,163,184,0.55)",
  forward: "rgba(226,232,240,0.72)",
  startInk: "rgba(148,163,184,0.65)",
  startText: "#cbd5e1",
  scrim: "rgba(7,9,13,0.72)",
  halo: "rgba(7,9,13,0.85)",
  trailMeasured: "#34d399",
  trailAssumed: "#fbbf24",
  assumed: "#fbbf24",
  measured: "#34d399",
  danger: "#fb7185",
  cloudNoise: 0.55,
  classColors: CLASS_COLORS_DARK,
  zoneMix: "#e2e8f0",
  zoneMixT: 0,
  zoneFill: 0.17,
};

const THEME_LIGHT: ArenaTheme = {
  id: "light",
  surround: "#eef1f6",
  floor: "#ffffff",
  gridMinor: "rgba(51,65,85,0.07)",
  gridMajor: "rgba(51,65,85,0.18)",
  border: "rgba(194,65,12,0.65)",
  text: "#0f172a",
  textDim: "rgba(51,65,85,0.7)",
  forward: "rgba(15,23,42,0.75)",
  startInk: "rgba(71,85,105,0.65)",
  startText: "#334155",
  scrim: "rgba(255,255,255,0.82)",
  halo: "rgba(255,255,255,0.9)",
  trailMeasured: "#047857",
  trailAssumed: "#b45309",
  assumed: "#b45309",
  measured: "#047857",
  danger: "#be123c",
  cloudNoise: 0.7,
  classColors: CLASS_COLORS_LIGHT,
  zoneMix: "#0f172a",
  zoneMixT: 0.45,
  // Lower than dark: the same alpha over white is a much larger perceptual
  // step, and the ink has already been darkened by zoneMixT.
  zoneFill: 0.14,
};

/**
 * Pick a theme from what is actually behind the canvas.
 *
 * Deliberately NOT `prefers-color-scheme`: this app forces dark chrome
 * regardless of the OS setting, so honouring the media query would paint a
 * white map into a black dashboard for every operator whose laptop is in light
 * mode. Walking up for the first opaque-enough background and measuring its
 * luminance asks the only question that matters — what will this card be sat
 * on — and it keeps working however a future theme is implemented (a class, a
 * CSS variable, a data attribute).
 *
 * Runs on mount, on resize and on a colour-scheme change. Never per frame.
 */
function resolveTheme(el: HTMLElement | null): ArenaTheme {
  if (!el || typeof window === "undefined" || !window.getComputedStyle) return THEME_DARK;
  let node: HTMLElement | null = el;
  for (let hops = 0; node && hops < 12; hops++) {
    const bg = window.getComputedStyle(node).backgroundColor;
    const lum = luminanceOf(bg);
    if (lum !== null) return lum > 0.5 ? THEME_LIGHT : THEME_DARK;
    node = node.parentElement;
  }
  return THEME_DARK;
}

/** Relative luminance of an `rgb()/rgba()` string; null if too transparent. */
function luminanceOf(css: string): number | null {
  const open = css.indexOf("(");
  if (open < 0) return null;
  const parts = css.slice(open + 1, css.indexOf(")")).split(",");
  if (parts.length < 3) return null;
  const r = Number(parts[0]);
  const g = Number(parts[1]);
  const b = Number(parts[2]);
  const a = parts.length > 3 ? Number(parts[3]) : 1;
  if (!Number.isFinite(r) || !Number.isFinite(g) || !Number.isFinite(b)) return null;
  // A nearly transparent layer tells us nothing about what shows through it.
  if (!Number.isFinite(a) || a < 0.25) return null;
  return (0.2126 * r + 0.7152 * g + 0.0722 * b) / 255;
}

/** One waypoint of a planned route, in world millimetres. */
export type RoutePoint = {
  x_mm: number;
  y_mm: number;
  kind?: string;
  dock?: boolean;
};

type Props = {
  thing: string;
  accent?: string;
  /** Raw pose envelope. Heartbeat-rate (0.2 Hz), so passing it as a prop is free. */
  poseEnvelope?: unknown;
  /**
   * Nominal planned route from the mission executor's preview, world mm.
   *
   * NOMINAL is the operative word and the reason this is drawn as a dashed
   * line rather than a solid one. Execution re-measures the bearing after every
   * leg and inserts correction turns, so the driven path deviates from this.
   * Drawing it as a confident solid track would overstate what the rover
   * actually promises to do.
   */
  route?: readonly RoutePoint[] | null;
};

/** Snap to a half-pixel so a 1px stroke lands on one device row, not two. */
function hair(v: number): number {
  return Math.round(v) + 0.5;
}

/**
 * Cached HUD text.
 *
 * Rebuilt only when the scan or the pose changes, which is at most 2 Hz, and
 * read by the 60 Hz draw. Mutated in place — a fresh object per rebuild would
 * put an allocation back on a path that exists to avoid them.
 */
type Hud = {
  status: string;
  statusInk: string;
  counts: string;
  near: string;
  perf: string;
  oob: string;
  oobInk: string;
  badge: string;
  badgeInk: string;
  sub: string;
  posX: string;
  posY: string;
  hdg: string;
  /** Odometer since the last break. The MEASURED input behind the drift rings. */
  driven: string;
  /** Cross-track deviation from the planned route, or `--` when either is absent. */
  dev: string;
  devInk: string;
  /**
   * The rover's coordinate, in centimetres, printed in the gutter ON the axis.
   *
   * One decimal place and no unit: the axis is already labelled "X cm", the
   * plate sits between two centimetre ticks, and a unit repeated at every
   * sample is noise. A millimetre resolution readout would be false precision
   * on a pose this fleet dead-reckons — the HUD block carries the raw mm for
   * anyone who wants it.
   */
  curX: string;
  curY: string;
  /** False when the pose is off the arena, in which case no plate is drawn. */
  curOn: boolean;
  /**
   * Plate widths for the two cursor labels.
   *
   * Measured on the rebuild, not in the draw. `measureText` returns a
   * TextMetrics object, so calling it per frame is per-frame allocation — the
   * exact thing the HUD cache and the module-level dash arrays exist to avoid,
   * and it would be four of them a frame here.
   */
  curXW: number;
  curYW: number;
  leftW: number;
  rightW: number;
};

const STALE_MS = 3000;

/**
 * Derived inks, rebuilt only when the theme, the accent or the stale flag
 * changes.
 *
 * `withAlpha` builds a string, and a string built inside the rAF loop is a
 * per-frame allocation — eight of them a frame here, sixty times a second, for
 * values that change perhaps twice in a session. The three scalars below are
 * compared by identity so the guard itself allocates nothing either (a
 * concatenated cache key would put the allocation straight back).
 */
const INK = {
  cloud: ["", "", "", ""],
  trackFill: ["", "", "", ""],
  nearest: "",
  bodyFillAssumed: "",
  bodyFillMeasured: "",
  rayAssumed: "",
  rayMeasured: "",
};
let inkTheme: ArenaTheme | null = null;
let inkAccent = "";
let inkFade = -1;

function ensureInk(theme: ArenaTheme, accent: string, fade: number): void {
  if (inkTheme === theme && inkAccent === accent && inkFade === fade) return;
  inkTheme = theme;
  inkAccent = accent;
  inkFade = fade;
  INK.cloud[0] = withAlpha(accent, theme.cloudNoise * fade);
  INK.trackFill[0] = "rgba(0,0,0,0)";
  for (let c = 1; c < 4; c++) {
    INK.cloud[c] = withAlpha(theme.classColors[c], 0.85 * fade);
    // TREE reads as a solid post, OBSTACLE as a footprint; a wall is a line
    // and is never filled.
    INK.trackFill[c] = withAlpha(theme.classColors[c], c === CLS_TREE ? 0.18 : 0.14);
  }
  INK.nearest = withAlpha(theme.text, 0.55);
  INK.bodyFillAssumed = withAlpha(theme.assumed, 0.07);
  INK.bodyFillMeasured = withAlpha(accent, 0.16);
  INK.rayAssumed = withAlpha(theme.assumed, 0.6);
  INK.rayMeasured = withAlpha(accent, 0.6);
}

function ArenaMapImpl({ thing, accent = EMBER, poseEnvelope, route }: Props) {
  const wrapRef = useRef<HTMLDivElement>(null);
  const staticRef = useRef<HTMLCanvasElement>(null);
  const dynRef = useRef<HTMLCanvasElement>(null);

  const lidar = useChannelRef<any>(`lidar:${thing}`);

  // Pose arrives as a prop but is read from inside the rAF loop, so it lives
  // in a ref; writing it during render keeps the loop from closing over a
  // stale value without adding a dependency that would restart the loop.
  const poseEnvRef = useRef<unknown>(poseEnvelope);
  poseEnvRef.current = poseEnvelope;

  // Same treatment as the pose: read inside the rAF loop, so it must not be a
  // loop dependency or every new plan would tear down and restart rendering.
  const routeRef = useRef<readonly RoutePoint[] | null | undefined>(route);
  routeRef.current = route;

  const engineRef = useRef<ClusterEngine | null>(null);
  if (engineRef.current === null) engineRef.current = createClusterEngine();

  const trailRef = useRef<PoseTrail | null>(null);
  if (trailRef.current === null) trailRef.current = createPoseTrail();

  const frameRef = useRef<ArenaFrame | null>(null);
  const poseRef = useRef<Pose>(readPose(null, null));
  // Cross-track deviation from the planned route, recomputed on the telemetry
  // edge and mutated in place. One object for the life of the card.
  const devRef = useRef<NearestOnPath | null>(null);
  if (devRef.current === null) devRef.current = createNearestOnPath();
  const sizeRef = useRef(0);
  const drawMsRef = useRef(0);
  /**
   * True when the operator has asked their OS for reduced motion.
   *
   * Read here rather than left to CSS because CSS cannot reach a canvas: the
   * site stylesheet disables animation globally and the uncertainty rings, drawn
   * pixel by pixel in the rAF loop, sail straight through it. Kept in a ref and
   * updated by a listener so the loop reads it without restarting.
   */
  const stillRef = useRef(false);
  const themeRef = useRef<ArenaTheme>(THEME_DARK);
  const scanAtRef = useRef(0);
  const hudRef = useRef<Hud>({
    status: "AWAITING SCAN",
    statusInk: THEME_DARK.textDim,
    counts: "",
    near: "",
    perf: "",
    oob: "",
    oobInk: THEME_DARK.danger,
    badge: "POSE ASSUMED",
    badgeInk: THEME_DARK.assumed,
    sub: "NOT MEASURED",
    posX: "X   -- mm",
    posY: "Y   -- mm",
    hdg: "HDG  --",
    driven: "DRIVEN   -- mm",
    dev: "PLAN DEV   -- mm",
    devInk: THEME_DARK.textDim,
    curX: "",
    curY: "",
    curOn: false,
    curXW: 0,
    curYW: 0,
    leftW: 0,
    rightW: 0,
  });

  // ---------------------------------------------------------------- setup ---
  useEffect(() => {
    const wrap = wrapRef.current;
    const sc = staticRef.current;
    const dc = dynRef.current;
    if (!wrap || !sc || !dc) return;

    const sctx = sc.getContext("2d");
    const dctx = dc.getContext("2d");
    if (!sctx || !dctx) return;

    const engine = engineRef.current!;
    const trail = trailRef.current!;
    const dev = devRef.current!;
    let raf = 0;
    let lastSeq = -1;
    let lastPoseEnv: unknown = Symbol("unset");
    // The route is a prop that changes when a mission is planned, which is far
    // rarer than the pose edge; tracking it separately means a new plan
    // recomputes the deviation without waiting for the next telemetry frame.
    let lastRoute: unknown = Symbol("unset");
    let disposed = false;

    const repaintStatic = (size: number) => {
      sc.style.background = themeRef.current.surround;
      drawStatic(sctx, size, themeRef.current, accent);
    };

    const resize = () => {
      const size = Math.max(120, Math.floor(wrap.clientWidth));
      const dpr = window.devicePixelRatio || 1;
      const themeNow = resolveTheme(wrap);
      const themed = themeNow !== themeRef.current;
      themeRef.current = themeNow;
      if (!themed && size === sizeRef.current && sc.width === Math.round(size * dpr)) return;
      sizeRef.current = size;
      for (const c of [sc, dc]) {
        c.width = Math.round(size * dpr);
        c.height = Math.round(size * dpr);
        c.style.width = `${size}px`;
        c.style.height = `${size}px`;
      }
      // setTransform (not scale) so repeated resizes do not compound.
      sctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      dctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      repaintStatic(size);
    };

    const loop = () => {
      if (disposed) return;
      raf = requestAnimationFrame(loop);

      const seq = lidar.seqRef.current;
      const env = lidar.dataRef.current;
      const poseEnv = poseEnvRef.current;
      const scanChanged = seq !== lastSeq;
      const poseChanged = poseEnv !== lastPoseEnv;

      // The pose is a prop and the scan is a socket; either can move without
      // the other, and both must be able to update the glyph. Identity
      // comparison keeps this on the message edge rather than the frame edge.
      if (scanChanged || poseChanged) {
        lastSeq = seq;
        lastPoseEnv = poseEnv;
        const prev = poseRef.current;
        const next = readPose(poseEnv, env);
        poseRef.current = next;

        // A break means the world moved under the tracker: a teleport, or the
        // first real fix after an assumed one. Segments carried over from
        // before it are anchored to a pose that no longer exists, so they are
        // dropped rather than left to float.
        if (trail.push(next.x_mm, next.y_mm, next.position === "measured")) {
          if (prev.position !== next.position) engine.reset();
        }
      }

      if (scanChanged) {
        scanAtRef.current = Date.now();
        const data = env && typeof env === "object" ? (env as any).data : null;
        frameRef.current = engine.update(
          data ? data.ranges_m : null,
          data && typeof data.range_max_m === "number" ? data.range_max_m : 6.0,
          poseRef.current,
        );
      }

      // A plan and a pose are compared against each other, so either changing
      // invalidates the comparison. Recomputed here rather than in the draw:
      // this is O(waypoints) and belongs on the message edge like everything
      // else that produces a number the HUD prints.
      const routeNow = routeRef.current;
      const routeChanged = routeNow !== lastRoute;
      if (poseChanged || routeChanged) {
        lastRoute = routeNow;
        const p = poseRef.current;
        // An ASSUMED position has nothing to compare: measuring a constant we
        // invented against the plan yields a deviation the rover never had.
        if (p.position === "measured") nearestOnPath(routeNow, p.x_mm, p.y_mm, dev);
        else nearestOnPath(null, NaN, NaN, dev);
      }

      const ageMs = scanAtRef.current > 0 ? Date.now() - scanAtRef.current : Infinity;
      if (scanChanged || poseChanged || routeChanged) {
        buildHud(
          hudRef.current,
          dctx,
          frameRef.current,
          poseRef.current,
          themeRef.current,
          lidar.metaRef.current.connected,
          drawMsRef.current,
          trail,
          dev,
        );
      }

      const t0 = performance.now();
      drawDynamic(
        dctx,
        sizeRef.current,
        frameRef.current,
        poseRef.current,
        accent,
        themeRef.current,
        trail,
        hudRef.current,
        ageMs,
        t0,
        routeNow,
        dev,
        stillRef.current,
      );
      drawMsRef.current = performance.now() - t0;
    };

    resize();
    raf = requestAnimationFrame(loop);

    const ro = new ResizeObserver(resize);
    ro.observe(wrap);

    // A theme swap does not resize anything, so the observer above would never
    // fire for it and the static layer would keep the old palette forever.
    const mq = window.matchMedia ? window.matchMedia("(prefers-color-scheme: light)") : null;
    const onScheme = () => {
      themeRef.current = resolveTheme(wrap);
      repaintStatic(sizeRef.current);
    };
    mq?.addEventListener?.("change", onScheme);

    // Reduced motion. Only the ring style depends on it and the loop reads the
    // ref every frame, so the listener writes the flag and nothing repaints.
    const rm = window.matchMedia ? window.matchMedia("(prefers-reduced-motion: reduce)") : null;
    stillRef.current = rm ? rm.matches : false;
    const onMotion = () => {
      stillRef.current = rm ? rm.matches : false;
    };
    rm?.addEventListener?.("change", onMotion);

    return () => {
      disposed = true;
      cancelAnimationFrame(raf);
      ro.disconnect();
      mq?.removeEventListener?.("change", onScheme);
      rm?.removeEventListener?.("change", onMotion);
    };
    // `accent` and `thing` are stable for the life of a card; the refs above
    // carry everything that actually changes.
  }, [accent, lidar]);

  return (
    <div ref={wrapRef} className="relative aspect-square w-full">
      <canvas ref={staticRef} className="absolute inset-0 rounded-xl ring-1 ring-white/5" />
      <canvas ref={dynRef} className="pointer-events-none absolute inset-0 rounded-xl" />
    </div>
  );
}

/**
 * Props are primitives plus a pose envelope that only changes on the 5 s
 * heartbeat, so memo absorbs the parent's 2 Hz re-render entirely.
 */
export const ArenaMap = memo(ArenaMapImpl);
export default ArenaMap;

// ============================================================ static layer ===

function drawStatic(
  ctx: CanvasRenderingContext2D,
  size: number,
  theme: ArenaTheme,
  accent: string,
): void {
  if (size <= 0) return;
  const s = scaleFor(size, PAD_PX);
  const pad = PAD_PX;

  ctx.clearRect(0, 0, size, size);
  ctx.fillStyle = theme.surround;
  ctx.fillRect(0, 0, size, size);

  const x0 = worldToCanvasX(0, s, pad);
  const y0 = worldToCanvasY(ARENA_MM, s, pad); // world top -> canvas top
  const wpx = ARENA_MM * s;

  // Arena floor, one step off the surround so the playable area reads as a
  // distinct object rather than a grid floating on a panel.
  ctx.fillStyle = theme.floor;
  ctx.fillRect(x0, y0, wpx, wpx);

  // ---- grid ---------------------------------------------------------------
  ctx.lineWidth = 1;
  ctx.strokeStyle = theme.gridMinor;
  ctx.beginPath();
  for (let v = GRID_MINOR_MM; v < ARENA_MM; v += GRID_MINOR_MM) {
    if (Math.abs(v % GRID_MAJOR_MM) < 1e-6) continue;
    const px = hair(worldToCanvasX(v, s, pad));
    const py = hair(worldToCanvasY(v, s, pad));
    ctx.moveTo(px, y0);
    ctx.lineTo(px, y0 + wpx);
    ctx.moveTo(x0, py);
    ctx.lineTo(x0 + wpx, py);
  }
  ctx.stroke();

  ctx.strokeStyle = theme.gridMajor;
  ctx.beginPath();
  for (let v = GRID_MAJOR_MM; v < ARENA_MM; v += GRID_MAJOR_MM) {
    const px = hair(worldToCanvasX(v, s, pad));
    const py = hair(worldToCanvasY(v, s, pad));
    ctx.moveTo(px, y0);
    ctx.lineTo(px, y0 + wpx);
    ctx.moveTo(x0, py);
    ctx.lineTo(x0 + wpx, py);
  }
  ctx.stroke();

  // ---- regions: the three zones and the start box --------------------------
  // All four corners go through one routine so they read as one family and so
  // the start box cannot quietly drift into looking like a destination.
  for (const z of ZONES) {
    drawRegion(ctx, theme, {
      x: worldToCanvasX(z.x_mm, s, pad),
      y: worldToCanvasY(z.y_mm + z.h_mm, s, pad), // top edge in canvas
      w: z.w_mm * s,
      h: z.h_mm * s,
      upper: z.y_mm + z.h_mm / 2 > ARENA_MM / 2,
      // The zone inks are picked for a dark floor; on white they wash out, so
      // they are pulled toward the text colour by the theme's mix factor. Hue
      // is preserved — the hue is how the operator tells the zones apart.
      ink: mixHex(z.stroke, theme.zoneMix, theme.zoneMixT),
      fillA: theme.zoneFill,
      kind: z.kind,
      dashed: false,
      title: z.label,
      corner: z.corner,
      role: z.role,
    });
  }

  // The start box is drawn dashed, unfilled and in neutral ink because it is
  // not a destination: it is where the operator is asked to put the rover, and
  // where the glyph is drawn when nothing reports a position. Filling it in a
  // zone colour would imply the rover navigates to it.
  //
  // Its role line is the antidote to THE ZONE NAMING TRAP (see arena.ts): it
  // names the zone that is physically in front of the rover as it sits here,
  // derived from the coordinates, so the operator never has to translate
  // between their "Zone 1" and the mission code's m1/m2.
  drawRegion(ctx, theme, {
    x: worldToCanvasX(START_BOX.x_mm, s, pad),
    y: worldToCanvasY(START_BOX.y_mm + START_BOX.h_mm, s, pad),
    w: START_BOX.w_mm * s,
    h: START_BOX.h_mm * s,
    upper: false,
    ink: theme.startText,
    edgeInk: theme.startInk,
    fillA: 0,
    kind: "start",
    dashed: true,
    title: START_BOX.label,
    corner: START_BOX.corner,
    role: ZONE_AHEAD_OF_START ? `AHEAD ${ZONE_AHEAD_OF_START.label}` : "",
  });

  // ---- border -------------------------------------------------------------
  ctx.strokeStyle = theme.id === "dark" ? withAlpha(accent, 0.55) : theme.border;
  ctx.lineWidth = 1;
  ctx.strokeRect(hair(x0), hair(y0), Math.round(wpx), Math.round(wpx));

  // ---- axis ladder --------------------------------------------------------
  // A tick every 5 cm, a longer labelled tick every 30 cm. Labelled in
  // centimetres: the arena is 120 cm and operators measure it with a tape, not
  // in millimetres.
  //
  // The minor ticks are the reason a centimetre is readable at all. With 30 cm
  // labels alone the operator has to interpolate a third of a metre by eye;
  // with a 5 cm ladder they count ticks, and the ladder pitch is GRID_MINOR_MM
  // — the same lines already drawn across the floor — so a tick in the gutter
  // and a grid line under the rover are guaranteed to be the same coordinate.
  ctx.strokeStyle = theme.gridMajor;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let v = 0; v <= ARENA_MM; v += GRID_MINOR_MM) {
    if (Math.abs(v % GRID_MAJOR_MM) < 1e-6) continue;
    const px = hair(worldToCanvasX(v, s, pad));
    const py = hair(worldToCanvasY(v, s, pad));
    ctx.moveTo(px, y0 + wpx);
    ctx.lineTo(px, y0 + wpx + 3);
    ctx.moveTo(x0 - 3, py);
    ctx.lineTo(x0, py);
  }
  ctx.stroke();

  ctx.strokeStyle = theme.textDim;
  ctx.beginPath();
  for (let v = 0; v <= ARENA_MM; v += GRID_MAJOR_MM) {
    const px = hair(worldToCanvasX(v, s, pad));
    const py = hair(worldToCanvasY(v, s, pad));
    ctx.moveTo(px, y0 + wpx);
    ctx.lineTo(px, y0 + wpx + 5);
    ctx.moveTo(x0 - 5, py);
    ctx.lineTo(x0, py);
  }
  ctx.stroke();

  ctx.fillStyle = theme.textDim;
  ctx.font = MONO_SM;
  for (let v = 0; v <= ARENA_MM; v += GRID_MAJOR_MM) {
    // Both axes skip zero: the origin marker below prints "0,0" in the corner
    // between them, which says the same thing once instead of twice. Drawn as
    // well, the x "0" and the y "0" and the "0,0" all land within a few pixels
    // of each other and overlap into an unreadable smear — the one place on the
    // ladder where a label is guaranteed to collide.
    if (v === 0) continue;
    const px = worldToCanvasX(v, s, pad);
    const py = worldToCanvasY(v, s, pad);
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    ctx.fillText(`${Math.round(v / 10)}`, px, y0 + wpx + 7);
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    ctx.fillText(`${Math.round(v / 10)}`, x0 - 7, py);
  }
  // Axis names, so "0..120" is not left as a bare number sequence, and the
  // SENSE of each axis with it. Origin is bottom-left, so the captions sit at
  // the far end of their own axis pointing away from it.
  ctx.textAlign = "left";
  ctx.textBaseline = "top";
  ctx.fillText("X cm >", x0 + wpx + 2, y0 + wpx + 7);
  ctx.textAlign = "right";
  ctx.textBaseline = "bottom";
  ctx.fillText("^ Y cm", x0 - 2, y0 - 3);

  // ---- origin -------------------------------------------------------------
  drawOrigin(ctx, x0, y0, wpx, s, theme);

  // ---- heading legend -----------------------------------------------------
  // The convention, painted where it is used: heading is CCW from +x, so the
  // right edge is 0 and the left edge is 180. With FORWARD = 90 above, an
  // operator can turn any HDG readout into a direction without a diagram.
  ctx.fillStyle = theme.textDim;
  ctx.font = MONO_SM;
  ctx.textBaseline = "middle";
  ctx.textAlign = "right";
  ctx.fillText("0°", x0 + wpx - 6, y0 + wpx / 2);
  ctx.textAlign = "left";
  ctx.fillText("180°", x0 + 6, y0 + wpx / 2);

  // ---- key: scale bar and the two path styles ------------------------------
  drawKey(ctx, x0, y0, wpx, s, theme);

  // ---- FORWARD marker -----------------------------------------------------
  drawForwardMarker(ctx, x0, y0, wpx, theme);
}

/**
 * World (0,0), stated rather than implied.
 *
 * Every coordinate on this map, every zone rectangle and every waypoint the
 * mission executor emits is measured from this one corner, and until now it was
 * the only thing on the map you had to infer — a "0" at each end of the axis
 * ladder and nothing marking the point where they meet. An operator who assumes
 * the origin is the centre (which is where most bird's-eye views put it) reads
 * every number on this dashboard wrong by 600 mm in both axes.
 *
 * So the corner gets a ringed dot on the floor, a short heavy run of each axis
 * out of it, and the coordinate written in the diagonal gutter where no tick
 * label can collide with it. The two little runs also carry the axis SENSE:
 * they leave the origin in the +x and +y directions, which on screen is right
 * and UP, and up-is-+y is the one thing a canvas will silently invert.
 */
function drawOrigin(
  ctx: CanvasRenderingContext2D,
  x0: number,
  y0: number,
  wpx: number,
  s: number,
  theme: ArenaTheme,
): void {
  const oy = y0 + wpx; // world y = 0 is the BOTTOM of the arena
  // One minor grid cell (5 cm) of each axis, so the run is itself a legend for
  // what one grid square is worth.
  const run = Math.max(8, Math.min(26, GRID_MINOR_MM * s));

  ctx.strokeStyle = theme.text;
  ctx.globalAlpha = 0.75;
  ctx.lineWidth = 2;
  ctx.lineCap = "round";
  ctx.beginPath();
  ctx.moveTo(hair(x0) + run, hair(oy));
  ctx.lineTo(hair(x0), hair(oy));
  ctx.lineTo(hair(x0), hair(oy) - run);
  ctx.stroke();
  ctx.lineWidth = 1;
  ctx.lineCap = "butt";

  ctx.fillStyle = theme.floor;
  ctx.beginPath();
  ctx.arc(x0, oy, 3, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = theme.text;
  ctx.stroke();
  ctx.fillStyle = theme.text;
  ctx.beginPath();
  ctx.arc(x0, oy, 1.3, 0, Math.PI * 2);
  ctx.fill();
  ctx.globalAlpha = 1;

  // Diagonally outside the corner: the x ladder runs along the bottom and the
  // y ladder down the left side, so the only space guaranteed free of tick
  // labels is the square between them.
  ctx.font = MONO_SM;
  ctx.fillStyle = theme.textDim;
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  ctx.fillText("0,0", x0 - 15, oy + 7);
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
}

// The key's two path samples, as text. Module constants so the static layer
// composes nothing, and so the words on the map and the styles in drawTrail /
// drawRoute are edited in one place when either changes.
const KEY_ACTUAL = "DRIVEN";
const KEY_PLANNED = "PLANNED";

/**
 * Width of the unclaimed strip along the bottom of the arena, world mm.
 *
 * DERIVED, not measured off the current layout. The key and the scale bar sit
 * between the bottom-left region and the start box, and that gap is only wide
 * enough by arithmetic: move a zone in arena.ts, or change ZONE_SIDE_MM, and
 * this shrinks with it and the key gates itself out rather than being drawn
 * across a region. A hardcoded pixel budget would silently start overlapping.
 */
const BOTTOM_GAP_MM = (() => {
  let right = 0;
  for (const z of ZONES) {
    // Bottom half only: a top-corner zone does not bound this strip.
    if (z.y_mm + z.h_mm / 2 > ARENA_MM / 2) continue;
    if (z.x_mm >= START_BOX.x_mm) continue;
    right = Math.max(right, z.x_mm + z.w_mm);
  }
  return Math.max(0, START_BOX.x_mm - right);
})();

/**
 * Scale bar plus the plan-versus-actual key, bottom-centre.
 *
 * PLACEMENT. The strip of floor between the water station (bottom-left) and the
 * start box (bottom-right) is the only part of the arena no region claims, and
 * the HUD blocks are anchored to the top corners, so this is the one place a
 * persistent block can sit without covering something.
 *
 * WHY THE KEY IS STATIC. The two path styles are a property of the map, like
 * the FORWARD marker: they mean the same thing whether or not a route is
 * currently planned or the rover has moved. Drawing it only when data exists
 * would remove the legend at exactly the moment an operator is trying to work
 * out which line is which, and it would put measureText in the rAF loop.
 *
 * DEGRADATION. Everything here is width-gated, in the order things become
 * worth less: the samples' words go first, then the key entirely, and the scale
 * bar is last because a map with no scale is not a metric map at all.
 */
function drawKey(
  ctx: CanvasRenderingContext2D,
  x0: number,
  y0: number,
  wpx: number,
  s: number,
  theme: ArenaTheme,
): void {
  const cx = x0 + wpx / 2;
  const barMm = GRID_MAJOR_MM;
  const barPx = barMm * s;
  if (!(barPx > 12)) return;

  // ---- scale bar -----------------------------------------------------------
  // With a mid-division, so the bar reads as two 15 cm halves and can be halved
  // again by eye. A bar with only two ends can be compared to a distance; a
  // divided one can be used to measure it.
  const bx = cx - barPx / 2;
  const by = y0 + wpx - 12;
  ctx.strokeStyle = theme.text;
  ctx.globalAlpha = 0.6;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(hair(bx), hair(by) - 3);
  ctx.lineTo(hair(bx), hair(by) + 3);
  ctx.moveTo(hair(bx), hair(by));
  ctx.lineTo(hair(bx + barPx), hair(by));
  ctx.moveTo(hair(bx + barPx), hair(by) - 3);
  ctx.lineTo(hair(bx + barPx), hair(by) + 3);
  ctx.moveTo(hair(bx + barPx / 2), hair(by) - 2);
  ctx.lineTo(hair(bx + barPx / 2), hair(by) + 2);
  ctx.stroke();
  ctx.globalAlpha = 0.7;
  ctx.fillStyle = theme.text;
  ctx.font = MONO_SM;
  ctx.textAlign = "center";
  ctx.textBaseline = "bottom";
  ctx.fillText(`${Math.round(barMm / 10)} cm`, cx, by - 4);
  ctx.globalAlpha = 1;
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";

  // ---- path key ------------------------------------------------------------
  // The band this may occupy is the gap between the two bottom regions. Derived
  // from the geometry rather than guessed, so moving a zone moves the gate.
  const band = BOTTOM_GAP_MM * s;

  const sampleW = 18;
  const gap = 5;
  const padX = 5;
  const rowH = 11;
  ctx.font = MONO_SM;
  const textW = Math.max(ctx.measureText(KEY_ACTUAL).width, ctx.measureText(KEY_PLANNED).width);
  const plateW = sampleW + gap + textW + padX * 2;
  const plateH = rowH * 2 + 6;
  if (plateW > band || wpx < 190) return;

  const plateX = cx - plateW / 2;
  const plateY = by - 18 - plateH;
  scrim(ctx, plateX, plateY, plateW, plateH, theme);

  const sx = plateX + padX;
  const tx = sx + sampleW + gap;
  ctx.textBaseline = "middle";
  ctx.textAlign = "left";
  ctx.lineWidth = 1.5;

  // Actual first: it is the line the operator is watching, and the plan is the
  // reference it is watched against.
  let ry = plateY + 3 + rowH / 2;
  ctx.strokeStyle = theme.trailMeasured;
  ctx.setLineDash(DASH_NONE);
  ctx.beginPath();
  ctx.moveTo(sx, hair(ry));
  ctx.lineTo(sx + sampleW, hair(ry));
  ctx.stroke();
  ctx.fillStyle = theme.text;
  ctx.fillText(KEY_ACTUAL, tx, ry);

  ry += rowH;
  ctx.strokeStyle = theme.id === "light" ? ROUTE_INK_LIGHT : ROUTE_INK_DARK;
  ctx.setLineDash(DASH_ROUTE);
  ctx.beginPath();
  ctx.moveTo(sx, hair(ry));
  ctx.lineTo(sx + sampleW, hair(ry));
  ctx.stroke();
  ctx.setLineDash(DASH_NONE);
  ctx.fillStyle = theme.textDim;
  ctx.fillText(KEY_PLANNED, tx, ry);

  ctx.lineWidth = 1;
  ctx.textBaseline = "alphabetic";
}

/** One corner region of the arena, as the static layer needs to draw it. */
type RegionSpec = {
  /** Canvas rect: top-left corner and size, px. */
  x: number;
  y: number;
  w: number;
  h: number;
  /**
   * True when the region sits in the UPPER half of the arena, which decides
   * which inside edge the label plate hugs.
   *
   * Upper regions are labelled along their BOTTOM edge and lower regions along
   * their TOP, so every plate ends up facing the middle of the arena. That
   * keeps all four names off the arena's outer corners, which is where the HUD
   * block and the pose badge are anchored and where a zone label used to end
   * up underneath the badge.
   */
  upper: boolean;
  /** Identity hue: wash, edge, brackets, glyph and title all come from this. */
  ink: string;
  /** Optional separate edge ink; defaults to `ink`. Used to keep START neutral. */
  edgeInk?: string;
  fillA: number;
  kind: "fire" | "water" | "start";
  dashed: boolean;
  title: string;
  /** The physical corner. Never a number — see THE ZONE NAMING TRAP. */
  corner: ArenaCorner;
  role: string;
};

/**
 * Paint one corner region: wash, texture, edge, corner brackets, label plate.
 *
 * ---------------------------------------------------------------------------
 * WHY THIS IS FOUR CUES AND NOT ONE
 * ---------------------------------------------------------------------------
 * The previous version drew each zone as a 10 % wash with a 1 px 75 %-alpha
 * outline and a 9 px unhaloed name in the zone's own colour. Every one of
 * those is a single point of failure on a near-black floor, and together they
 * produced a map on which an operator could not find the regions at all. So a
 * region now states itself four independent ways:
 *
 *   wash      lifts the whole rectangle off the floor.
 *   brackets  2.5 px L-shapes at the four corners, at full strength. These
 *             survive any wash that gets lost to contrast, brightness or a
 *             screen in sunlight, and they read at 50 px across.
 *   plate     a scrim with the NAME and the PHYSICAL CORNER on it, so the
 *             label never has to compete with the grid or the point cloud.
 *   glyph     shape, not hue: a target for a fire zone, a droplet for the
 *             water station, a chevron for the start box. This is the cue that
 *             still works for an operator who cannot tell violet from lime.
 *
 * Static layer only — measureText and the label logic here run on mount,
 * resize and theme change, never in the rAF loop.
 */
function drawRegion(ctx: CanvasRenderingContext2D, theme: ArenaTheme, r: RegionSpec): void {
  const { x, y, w, h } = r;
  if (!(w > 8) || !(h > 8)) return;
  const edge = r.edgeInk ?? r.ink;

  // ---- wash ----------------------------------------------------------------
  if (r.fillA > 0) {
    ctx.fillStyle = withAlpha(r.ink, r.fillA);
    ctx.fillRect(x, y, w, h);
  }

  // ---- texture -------------------------------------------------------------
  // Diagonal hatch marks the water station as a DIFFERENT KIND OF THING from
  // the two fire zones — a place the rover takes water on board, not a place
  // it discharges. A texture says that at any size and in any palette; a
  // slightly different blue does not.
  if (r.kind === "water") {
    ctx.save();
    ctx.beginPath();
    ctx.rect(x, y, w, h);
    ctx.clip();
    ctx.strokeStyle = r.ink;
    ctx.globalAlpha = 0.3;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let d = -h; d < w; d += 8) {
      ctx.moveTo(x + d, y + h);
      ctx.lineTo(x + d + h, y);
    }
    ctx.stroke();
    ctx.restore();
  }

  // ---- edge ----------------------------------------------------------------
  ctx.strokeStyle = edge;
  ctx.lineWidth = 1;
  if (r.dashed) ctx.setLineDash(DASH_START);
  ctx.strokeRect(hair(x), hair(y), Math.round(w), Math.round(h));
  ctx.setLineDash(DASH_NONE);

  // A second, inset edge gives the water station a doubled border: one more
  // difference that does not depend on colour.
  if (r.kind === "water" && w > 26 && h > 26) {
    ctx.globalAlpha = 0.45;
    ctx.strokeRect(hair(x + 4), hair(y + 4), Math.round(w - 8), Math.round(h - 8));
    ctx.globalAlpha = 1;
  }

  // ---- corner brackets -----------------------------------------------------
  const arm = Math.max(6, Math.min(22, w * 0.22));
  ctx.strokeStyle = edge;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  for (let k = 0; k < 4; k++) {
    const right = k === 1 || k === 2;
    const bottom = k >= 2;
    // Inset by half the stroke width so the bracket sits inside its region.
    const cx = right ? x + w - 1.25 : x + 1.25;
    const cy = bottom ? y + h - 1.25 : y + 1.25;
    const sx = right ? -1 : 1;
    const sy = bottom ? -1 : 1;
    ctx.moveTo(cx + sx * arm, cy);
    ctx.lineTo(cx, cy);
    ctx.lineTo(cx, cy + sy * arm);
  }
  ctx.stroke();
  ctx.lineWidth = 1;

  // ---- label plate ---------------------------------------------------------
  // Below this the region is a few dozen pixels across and a plate would cover
  // it entirely; the wash and the brackets still say a region is there.
  if (w < 30 || h < 30) return;

  const titleFont = w >= 108 ? ZONE_TITLE_LG : w >= 74 ? ZONE_TITLE_MD : ZONE_TITLE_SM;
  const titleH = w >= 108 ? 12 : w >= 74 ? 11 : 10;

  // The glyph is the first thing dropped when space runs out: a name and a
  // corner are worth more than an icon, and the wash and brackets are still
  // carrying the region's identity.
  const glyphOn = w >= 84;
  const glyphR = glyphOn ? Math.round(titleH * 0.46) : 0;
  const glyphW = glyphOn ? glyphR * 2 + 5 : 0;

  const padX = 5;
  const padY = 4;
  const lineH = 10;

  ctx.font = titleFont;
  const titleW = ctx.measureText(r.title).width;
  ctx.font = MONO_SM;
  const roleW = r.role ? ctx.measureText(r.role).width : 0;

  // Lines are included only if they FIT. A clipped "BOTTOM-RIG" is worse than
  // no sub-line, and this is the check that keeps the small-card rendering
  // honest instead of merely hopeful.
  //
  // The corner gets two chances before it is given up on: the full name, then
  // the abbreviation. Losing "BOTTOM-LEFT" off the water station on a phone is
  // exactly the ambiguity this whole change exists to remove, so it degrades
  // through "BTM-LEFT" first and is only dropped when even that will not fit.
  const avail = w - 6 - glyphW - padX * 2;
  let sub = r.corner as string;
  let subW = ctx.measureText(sub).width;
  if (subW > avail) {
    sub = shortCorner(r.corner);
    subW = ctx.measureText(sub).width;
  }
  const subOn = h >= 44 && subW <= avail;
  const roleOn = roleW > 0 && h >= 72 && roleW <= avail;

  const textW = Math.max(titleW, subOn ? subW : 0, roleOn ? roleW : 0);
  const plateW = Math.min(w - 4, textW + glyphW + padX * 2);
  const plateH = padY * 2 + titleH + 1 + (subOn ? lineH : 0) + (roleOn ? lineH : 0);
  const plateX = x + (w - plateW) / 2;
  const plateY = r.upper ? y + h - 5 - plateH : y + 5;

  scrim(ctx, plateX, plateY, plateW, plateH, theme);

  if (glyphOn) {
    const gx = plateX + padX + glyphR;
    const gy = plateY + padY + titleH / 2;
    if (r.kind === "water") drawDropGlyph(ctx, gx, gy, glyphR, r.ink);
    else if (r.kind === "fire") drawTargetGlyph(ctx, gx, gy, glyphR, r.ink);
    else drawStartGlyph(ctx, gx, gy, glyphR, r.ink);
  }

  const tx = plateX + padX + glyphW;
  let ty = plateY + padY;
  ctx.textAlign = "left";
  ctx.textBaseline = "top";

  ctx.font = titleFont;
  ctx.fillStyle = r.ink;
  ctx.fillText(r.title, tx, ty);
  ty += titleH + 1;

  if (subOn) {
    // The corner goes in the plain text colour, not the region's hue: it is
    // the part an operator has to be able to read, and it must not inherit
    // whatever contrast problem the identity colour has.
    ctx.font = MONO_SM;
    ctx.fillStyle = theme.text;
    ctx.fillText(sub, tx, ty);
    ty += lineH;
  }
  if (roleOn) {
    ctx.font = MONO_SM;
    ctx.fillStyle = theme.textDim;
    ctx.fillText(r.role, tx, ty);
  }

  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
}

/** Fire zone: a target. Something the rover aims at. */
function drawTargetGlyph(
  ctx: CanvasRenderingContext2D,
  cx: number,
  cy: number,
  r: number,
  ink: string,
): void {
  ctx.strokeStyle = ink;
  ctx.fillStyle = ink;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(cx, cy, r * 0.85, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx - r, cy);
  ctx.lineTo(cx + r, cy);
  ctx.moveTo(cx, cy - r);
  ctx.lineTo(cx, cy + r);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(cx, cy, r * 0.28, 0, Math.PI * 2);
  ctx.fill();
}

/** Water station: a droplet. Something the rover takes ON BOARD. */
function drawDropGlyph(
  ctx: CanvasRenderingContext2D,
  cx: number,
  cy: number,
  r: number,
  ink: string,
): void {
  ctx.beginPath();
  ctx.moveTo(cx, cy - r);
  ctx.quadraticCurveTo(cx + r, cy + r * 0.15, cx, cy + r * 0.9);
  ctx.quadraticCurveTo(cx - r, cy + r * 0.15, cx, cy - r);
  ctx.closePath();
  ctx.fillStyle = withAlpha(ink, 0.35);
  ctx.fill();
  ctx.strokeStyle = ink;
  ctx.lineWidth = 1;
  ctx.stroke();
}

/** Start box: a chevron pointing FORWARD, the way the rover is placed. */
function drawStartGlyph(
  ctx: CanvasRenderingContext2D,
  cx: number,
  cy: number,
  r: number,
  ink: string,
): void {
  ctx.strokeStyle = ink;
  ctx.lineWidth = 1.5;
  ctx.lineJoin = "round";
  ctx.beginPath();
  ctx.moveTo(cx - r * 0.8, cy + r * 0.25);
  ctx.lineTo(cx, cy - r * 0.6);
  ctx.lineTo(cx + r * 0.8, cy + r * 0.25);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx - r * 0.8, cy + r * 0.9);
  ctx.lineTo(cx + r * 0.8, cy + r * 0.9);
  ctx.stroke();
  ctx.lineWidth = 1;
  // Put the join back: the corner brackets of any region drawn after this one
  // would otherwise inherit round corners from a glyph.
  ctx.lineJoin = "miter";
}

/**
 * Fixed orientation reference: an arrow labelled FORWARD 90°, centred in the
 * gutter above the arena.
 *
 * This lives on the STATIC layer deliberately. It is a property of the map,
 * not of the rover: it says "up the screen is up the arena is world +y is
 * heading 90 deg" and it must keep saying that no matter where the rover
 * points. Putting it on the dynamic layer, or inside drawRover's rotated
 * transform, would turn it into a second heading indicator and destroy the
 * one thing it is for.
 *
 * It sits OUTSIDE the arena rather than just inside the top edge so it cannot
 * collide with the HUD or the pose badge, both of which are anchored to the
 * arena's top corners and both of which grow with their text.
 *
 * The direction is DERIVED from FORWARD_HEADING_DEG rather than hardcoded to
 * screen-up, so the marker and the rover's start heading cannot drift apart:
 * change the constant and this arrow follows. It is still static — the input
 * is a compile-time constant, never `pose.heading_rad`.
 *
 * Drawn once per mount/resize/theme change, so measureText and the trig here
 * are off the hot path entirely.
 */
function drawForwardMarker(
  ctx: CanvasRenderingContext2D,
  x0: number,
  y0: number,
  wpx: number,
  theme: ArenaTheme,
): void {
  const label = `FORWARD ${FORWARD_HEADING_DEG}°`;

  ctx.font = MONO_SM;
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";

  const textW = ctx.measureText(label).width;
  const arrowW = 9;
  const gap = 5;
  const midY = y0 - PAD_PX / 2;
  const left = x0 + wpx / 2 - (arrowW + gap + textW) / 2;
  const ax = left + arrowW / 2; // arrow centre, screen x
  const half = 7; // half-length of the shaft, px

  // World heading -> screen direction. cos/sin give the world unit vector;
  // the y component is negated because canvas +y points down, which is the
  // same single flip worldToCanvasY applies. At FORWARD_HEADING_DEG = 90 this
  // evaluates to (0, -1) — straight up the screen.
  const rad = (FORWARD_HEADING_DEG * Math.PI) / 180;
  const dx = Math.cos(rad);
  const dy = -Math.sin(rad);
  const perpX = -dy; // unit normal, for the arrowhead base
  const perpY = dx;

  const tipX = ax + dx * half;
  const tipY = midY + dy * half;
  const tailX = ax - dx * half;
  const tailY = midY - dy * half;
  // Snap a perfectly vertical or horizontal shaft to a half-pixel so the 1px
  // stroke lands on one device row; a diagonal gets no benefit from it.
  const snapX = Math.abs(dx) < 1e-6;
  const snapY = Math.abs(dy) < 1e-6;

  ctx.strokeStyle = theme.forward;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(snapX ? hair(tailX) : tailX, snapY ? hair(tailY) : tailY);
  ctx.lineTo(snapX ? hair(tipX) : tipX, snapY ? hair(tipY) : tipY);
  ctx.stroke();

  // Arrowhead: apex at the tip, base one head-length back along the shaft.
  const headLen = 7;
  const baseX = tipX - dx * headLen;
  const baseY = tipY - dy * headLen;
  ctx.fillStyle = theme.forward;
  ctx.beginPath();
  ctx.moveTo(tipX, tipY);
  ctx.lineTo(baseX + perpX * (arrowW / 2), baseY + perpY * (arrowW / 2));
  ctx.lineTo(baseX - perpX * (arrowW / 2), baseY - perpY * (arrowW / 2));
  ctx.closePath();
  ctx.fill();

  ctx.fillText(label, left + arrowW + gap, midY);

  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
}

// =========================================================== dynamic layer ===

function drawDynamic(
  ctx: CanvasRenderingContext2D,
  size: number,
  frame: ArenaFrame | null,
  pose: Pose,
  accent: string,
  theme: ArenaTheme,
  trail: PoseTrail,
  hud: Hud,
  scanAgeMs: number,
  tMs: number,
  route: readonly RoutePoint[] | null | undefined,
  dev: NearestOnPath,
  stillOnly: boolean,
): void {
  if (size <= 0) return;
  const s = scaleFor(size, PAD_PX);
  const pad = PAD_PX;

  ctx.clearRect(0, 0, size, size);

  // ---- planned route ------------------------------------------------------
  // First, so the scan, the tracked objects and the rover all draw OVER it. A
  // plan is intent; everything else on this canvas is measurement, and
  // measurement should never be obscured by intent.
  drawRoute(ctx, route, s, pad, theme);

  // ---- actual driven path -------------------------------------------------
  // Over the plan, deliberately: where the two coincide the operator should see
  // the driven line, because that is the one that happened.
  drawTrail(ctx, trail, s, pad, theme);

  // ---- plan versus actual -------------------------------------------------
  drawDeviation(ctx, dev, pose, s, pad, theme);

  // ---- point cloud --------------------------------------------------------
  // A scan older than STALE_MS is faded rather than removed. The rover's own
  // LiDAR bridge stops publishing instead of repeating the last scan for the
  // same reason this fades: a frozen picture that still looks live is worse
  // than a gap, because it invites planning against a world nobody is
  // observing any more.
  const stale = scanAgeMs > STALE_MS;
  const fade = stale ? 0.3 : 1;
  ensureInk(theme, accent, fade);

  if (frame && frame.ptCount > 0) {
    const pts = frame.pts;
    const cls = frame.ptCls;
    const n = frame.ptCount;
    // Four passes, one fillStyle each, instead of a style change per point.
    // Grey walls, green posts, amber obstacles, accent for anything the
    // segmenter did not resolve — a monochrome cloud hides exactly the
    // distinction the operator is looking for.
    for (let c = 0; c < 4; c++) {
      ctx.fillStyle = INK.cloud[c];
      const half = c === 0 ? 0.9 : 1.1;
      const side = half * 2;
      for (let i = 0; i < n; i++) {
        if (cls[i] !== c) continue;
        ctx.fillRect(
          worldToCanvasX(pts[i * 2], s, pad) - half,
          worldToCanvasY(pts[i * 2 + 1], s, pad) - half,
          side,
          side,
        );
      }
    }

    // Nearest return: the number the obstacle guard actually acts on, so it
    // gets a ring rather than being left as one pixel among three hundred.
    const ni = frame.nearestIdx;
    if (ni >= 0 && ni < n) {
      const nx = worldToCanvasX(pts[ni * 2], s, pad);
      const ny = worldToCanvasY(pts[ni * 2 + 1], s, pad);
      ctx.globalAlpha = fade;
      ctx.strokeStyle = frame.nearestMm < 150 ? theme.danger : INK.nearest;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(nx, ny, 5, 0, Math.PI * 2);
      ctx.stroke();
      ctx.globalAlpha = 1;
    }
  }

  // ---- objects ------------------------------------------------------------
  if (frame) {
    const tracks = frame.tracks;
    for (let i = 0; i < tracks.length; i++) {
      const tk = tracks[i];
      if (!tk.active) continue;
      const color = theme.classColors[tk.cls];
      const cxp = worldToCanvasX(tk.cx, s, pad);
      const cyp = worldToCanvasY(tk.cy, s, pad);

      ctx.globalAlpha = Math.max(0, Math.min(1, tk.alpha)) * fade;
      ctx.strokeStyle = color;
      ctx.lineWidth = 1;

      // PCA already gave us a centroid, a unit axis and the extents along and
      // across it, so the four corners of the oriented box are free.
      const hl = Math.max(tk.len, 8) / 2;
      const hw = Math.max(tk.wid, 8) / 2;
      let topPx: number;

      if (tk.cls === CLS_TREE) {
        // A post is round. Drawing it as an oriented rectangle implies the
        // scan resolved an orientation it cannot resolve at 1 deg spacing —
        // the measured minor extent of a 100 mm post is a sampling artefact,
        // not a shape.
        const r = Math.max(hl, hw) * s;
        ctx.beginPath();
        ctx.arc(cxp, cyp, r, 0, Math.PI * 2);
        ctx.fillStyle = INK.trackFill[tk.cls];
        ctx.fill();
        ctx.stroke();
        topPx = cyp - r;
      } else {
        const ax = tk.axX;
        const ay = tk.axY;
        const nx = -ay;
        const ny = ax;
        let minY = Infinity;
        ctx.beginPath();
        for (let k = 0; k < 4; k++) {
          const sl = k === 0 || k === 3 ? 1 : -1;
          const sw = k < 2 ? 1 : -1;
          const wx = tk.cx + ax * hl * sl + nx * hw * sw;
          const wy = tk.cy + ay * hl * sl + ny * hw * sw;
          const px = worldToCanvasX(wx, s, pad);
          const py = worldToCanvasY(wy, s, pad);
          if (py < minY) minY = py;
          if (k === 0) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
        }
        ctx.closePath();
        if (tk.cls === CLS_OBSTACLE) {
          ctx.fillStyle = INK.trackFill[tk.cls];
          ctx.fill();
        }
        ctx.stroke();
        topPx = minY;
      }

      // A label is a claim. Only make it once the majority vote has had time
      // to settle, otherwise every frame renames the same object. The range
      // rides along because "OBSTACLE" without a distance is not actionable.
      if (isLabelable(tk)) {
        ctx.font = MONO_SM;
        ctx.textAlign = "center";
        ctx.textBaseline = "bottom";
        haloText(ctx, labelFor(tk.cls, tk.minR), cxp, topPx - 3, color, theme.halo);
        ctx.textAlign = "left";
        ctx.textBaseline = "alphabetic";
      }
      ctx.globalAlpha = 1;
    }
  }

  // ---- rover --------------------------------------------------------------
  drawReticle(ctx, pose, s, pad, theme);
  drawCursor(ctx, pose, hud, s, pad, theme);
  drawRover(ctx, pose, s, pad, accent, theme, tMs, trail.drivenMm, stillOnly);

  // ---- HUD ----------------------------------------------------------------
  drawHud(ctx, size, hud, theme, stale, scanAgeMs);
}

/**
 * The link from the rover to the nearest point on the planned route.
 *
 * This is the whole plan-versus-actual comparison in one mark: a short link
 * means the rover is on the line, a long one means it is not, and the direction
 * says which side it drifted to. It is drawn as a dotted tether with a hollow
 * ring at the plan end rather than as a solid line, because the ring is a point
 * on the PLAN and must not be mistakable for a measurement — the rover was
 * never there.
 *
 * Suppressed below a few pixels: a link shorter than its own end markers is
 * visual noise reporting a deviation the operator does not need to act on, and
 * the HUD's PLAN DEV still carries the number.
 */
function drawDeviation(
  ctx: CanvasRenderingContext2D,
  dev: NearestOnPath,
  pose: Pose,
  s: number,
  pad: number,
  theme: ArenaTheme,
): void {
  if (!Number.isFinite(dev.dist_mm) || !Number.isFinite(dev.x_mm)) return;
  const rx = worldToCanvasX(pose.x_mm, s, pad);
  const ry = worldToCanvasY(pose.y_mm, s, pad);
  const px = worldToCanvasX(dev.x_mm, s, pad);
  const py = worldToCanvasY(dev.y_mm, s, pad);
  if (!Number.isFinite(rx) || !Number.isFinite(ry)) return;
  const dx = px - rx;
  const dy = py - ry;
  if (dx * dx + dy * dy < 25) return;

  const ink = theme.id === "light" ? ROUTE_INK_LIGHT : ROUTE_INK_DARK;
  ctx.strokeStyle = ink;
  ctx.lineWidth = 1;
  ctx.setLineDash(DASH_DEV);
  ctx.beginPath();
  ctx.moveTo(rx, ry);
  ctx.lineTo(px, py);
  ctx.stroke();
  ctx.setLineDash(DASH_NONE);

  ctx.beginPath();
  ctx.arc(px, py, 2.5, 0, Math.PI * 2);
  ctx.stroke();
}

/**
 * The rover's coordinate, printed in the gutter where the reticle meets the
 * axis ladder.
 *
 * "Read the position to the centimetre without leaving the map" is the
 * requirement, and this is the mark that delivers it. The reticle already draws
 * the eye from the rover out to each axis; this puts the number at the end of
 * that journey, sat on the ladder it belongs to, so reading a coordinate is a
 * glance rather than a lookup in the corner HUD.
 *
 * Inks by provenance, like everything else that states a position: an assumed
 * coordinate is amber and carries the same `?` the HUD does, so a number read
 * off the axis cannot be mistaken for a measured one.
 *
 * The strings come from the cached HUD (2 Hz edge). Per frame this is two
 * scrims and two fillText calls and allocates nothing.
 */
function drawCursor(
  ctx: CanvasRenderingContext2D,
  pose: Pose,
  hud: Hud,
  s: number,
  pad: number,
  theme: ArenaTheme,
): void {
  if (!hud.curOn) return;
  const cx = worldToCanvasX(pose.x_mm, s, pad);
  const cy = worldToCanvasY(pose.y_mm, s, pad);
  if (!Number.isFinite(cx) || !Number.isFinite(cy)) return;

  const x0 = worldToCanvasX(0, s, pad);
  const y0 = worldToCanvasY(ARENA_MM, s, pad);
  const wpx = ARENA_MM * s;
  // Provenance is carried by the INK alone here, not by a "?" suffix like the
  // HUD's. The gutter is only wide enough for "120.0" and a suffix would push
  // the plate into the arena or off the card; the plate is amber for an assumed
  // coordinate and the reticle line arriving at it is amber too, so nothing on
  // this path lets an assumption read as a fix.
  const ink = pose.position === "assumed" ? theme.assumed : theme.measured;

  ctx.font = MONO_SM;
  ctx.textBaseline = "middle";

  // The plate sits ON the ladder's label band and its scrim occludes whichever
  // centimetre label is behind it — deliberately. The number it hides is the
  // coarse version of the number it prints, so replacing "60" with "58.7"
  // locally is a gain, and it is how every axis crosshair worth using behaves.
  // The reticle's own bright tick is drawn inside the last 4 px against the
  // arena edge, which is left clear so the plate never covers it.
  const hgt = 13;

  // X, in the bottom gutter, centred on the rover's column. Clamped to the card
  // so a plate near either wall stays on screen rather than being clipped.
  const wx = hud.curXW + 6;
  let bx = cx - wx / 2;
  if (bx < 2) bx = 2;
  if (bx + wx > x0 + wpx + PAD_PX - 2) bx = x0 + wpx + PAD_PX - 2 - wx;
  const byy = y0 + wpx + 5;
  scrim(ctx, bx, byy, wx, hgt, theme);
  ctx.fillStyle = ink;
  ctx.textAlign = "center";
  ctx.fillText(hud.curX, bx + wx / 2, byy + hgt / 2);

  // Y, in the left gutter, on the rover's row. Right-aligned to the ladder so
  // it reads as part of the same column of numbers.
  const wy = hud.curYW + 6;
  let ly = cy - hgt / 2;
  if (ly < 2) ly = 2;
  if (ly + hgt > y0 + wpx + PAD_PX - 2) ly = y0 + wpx + PAD_PX - 2 - hgt;
  const lx = Math.max(2, x0 - 5 - wy);
  scrim(ctx, lx, ly, wy, hgt, theme);
  ctx.fillStyle = ink;
  ctx.fillText(hud.curY, lx + wy / 2, ly + hgt / 2);

  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
}

/**
 * Ink for the planned route. Cool, so it never competes with the ember rover
 * and the accent-coloured live scan — the plan is context, the scan is truth.
 *
 * ---------------------------------------------------------------------------
 * WHY THIS IS A TRUE BLUE AND NOT THE SKY IT USED TO BE
 * ---------------------------------------------------------------------------
 * The route was sky #7dd3fc, the driven trail was sky #38bdf8 and the water
 * station is cyan #22d3ee: three near-identical blues carrying three unrelated
 * meanings. Measured on this floor rather than argued about, the route against
 * the water station came out at dE 5.9 for NORMAL colour vision — a hard fail,
 * and it meant the planned route running into the refill point (which every
 * refill leg does) was very nearly invisible at the one place an operator most
 * needs to see it.
 *
 * Plan and driven path now sit in different hue families, which is also Nav2's
 * convention for global-plan versus executed path:
 *
 *   node scripts/validate_palette.js "#3987e5,#34d399" --mode dark \
 *     --surface "#0b0f16" --pairs all
 *     [PASS] CVD separation      dE 26.1 (deutan) / 15.2 (tritan)
 *     [PASS] Normal-vision floor dE 27.2
 *     [PASS] Contrast vs surface both >= 3:1
 *
 * Light was validated the same way against #ffffff ("#1d4ed8,#047857,#b45309":
 * normal-vision dE 21.9, CVD 7.9 — the 6-8 band, which is legal here because
 * the dash pattern and the line weight carry the distinction with no colour at
 * all). Hue is never the only cue on this map.
 */
const ROUTE_INK_DARK = "rgba(57,135,229,0.95)";
const ROUTE_FILL_DARK = "rgba(57,135,229,0.20)";
const ROUTE_INK_LIGHT = "rgba(29,78,216,0.9)";
const ROUTE_FILL_LIGHT = "rgba(29,78,216,0.16)";

/**
 * Draw the nominal planned route: dashed spine, a node per waypoint, and a
 * ringed target at the end.
 *
 * Dashed, not solid, and that is a deliberate honesty choice — see the `route`
 * prop. Docking legs are drawn as filled nodes because that is where the rover
 * deliberately slows, and an operator watching the map should be able to see
 * where the careful part of the route begins without reading a table.
 */
function drawRoute(
  ctx: CanvasRenderingContext2D,
  route: readonly RoutePoint[] | null | undefined,
  s: number,
  pad: number,
  theme: ArenaTheme,
): void {
  if (!route || route.length < 2) return;
  const ink = theme.id === "light" ? ROUTE_INK_LIGHT : ROUTE_INK_DARK;
  const fill = theme.id === "light" ? ROUTE_FILL_LIGHT : ROUTE_FILL_DARK;

  const px = (p: RoutePoint) => worldToCanvasX(p.x_mm, s, pad);
  const py = (p: RoutePoint) => worldToCanvasY(p.y_mm, s, pad);

  ctx.save();

  // Spine.
  ctx.strokeStyle = ink;
  ctx.lineWidth = 1.5;
  ctx.setLineDash(DASH_ROUTE);
  ctx.lineJoin = "round";
  ctx.beginPath();
  ctx.moveTo(px(route[0]), py(route[0]));
  for (let i = 1; i < route.length; i++) ctx.lineTo(px(route[i]), py(route[i]));
  ctx.stroke();
  ctx.setLineDash(DASH_NONE);

  // Waypoint nodes. The first is the start and already carries the rover, so
  // it is skipped — two markers on one spot reads as an error.
  for (let i = 1; i < route.length - 1; i++) {
    const p = route[i];
    ctx.beginPath();
    ctx.arc(px(p), py(p), p.dock ? 3 : 2, 0, Math.PI * 2);
    if (p.dock) {
      ctx.fillStyle = ink;
      ctx.fill();
    } else {
      ctx.strokeStyle = ink;
      ctx.lineWidth = 1;
      ctx.stroke();
    }
  }

  // Target.
  const end = route[route.length - 1];
  const ex = px(end);
  const ey = py(end);
  ctx.beginPath();
  ctx.arc(ex, ey, 7, 0, Math.PI * 2);
  ctx.fillStyle = fill;
  ctx.fill();
  ctx.strokeStyle = ink;
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(ex - 4, ey);
  ctx.lineTo(ex + 4, ey);
  ctx.moveTo(ex, ey - 4);
  ctx.lineTo(ex, ey + 4);
  ctx.lineWidth = 1;
  ctx.stroke();

  ctx.restore();
}

/** Alpha bands for the trail. Six strokes, not one per segment. */
const TRAIL_BANDS = 6;

/**
 * THE ACTUAL DRIVEN PATH — where the rover has really been, as opposed to where
 * the plan said it would go.
 *
 * This is one half of the comparison a route-following test exists to make, so
 * it is drawn to be told apart from `drawRoute`'s plan at a glance and in three
 * independent ways: SOLID against the plan's dashes, thicker (2 px against
 * 1.5), and in `theme.trailMeasured` — emerald, the ink of the POSE MEASURED
 * badge — against the plan's sky. Any one of those survives the loss of the
 * other two, which is the point: a colour-blind operator reads solid-versus-
 * dashed, and a greyscale print reads weight.
 *
 * Older samples fade, so the recent path reads as the current one without
 * throwing history away. Two further properties are load-bearing:
 *
 *   - a MEASURED run is a solid emerald line; an ASSUMED run is dashed amber. A
 *     dead-reckoned track and a track of where the map guessed the rover was
 *     are not the same claim and must not share a style.
 *   - a sample flagged TRAIL_BREAK lifts the pen. Joining across a teleport
 *     (an origin re-zero, or the first fix after an assumption) would draw a
 *     path the rover never drove, which is fabricating history.
 *
 * Alpha is stepped in bands rather than per segment so the whole trail costs a
 * dozen path builds a frame instead of five hundred.
 */
function drawTrail(
  ctx: CanvasRenderingContext2D,
  trail: PoseTrail,
  s: number,
  pad: number,
  theme: ArenaTheme,
): void {
  const n = trail.count;
  if (n < 2) return;

  const xs = trail.xs;
  const ys = trail.ys;
  const flags = trail.flags;

  // Heavier than the planned route's 1.5: the achieved path outranks the
  // intended one wherever they overlap, and weight is the cue that says so
  // without needing colour.
  ctx.lineWidth = 2;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";

  for (let b = 0; b < TRAIL_BANDS; b++) {
    const lo = Math.floor((b * n) / TRAIL_BANDS);
    const hi = Math.floor(((b + 1) * n) / TRAIL_BANDS);
    if (hi - lo < 1) continue;
    ctx.globalAlpha = 0.1 + 0.8 * ((b + 1) / TRAIL_BANDS);

    for (let kind = 0; kind < 2; kind++) {
      ctx.strokeStyle = kind === 1 ? theme.trailMeasured : theme.trailAssumed;
      ctx.setLineDash(kind === 1 ? DASH_NONE : DASH_TRAIL);
      ctx.beginPath();
      let open = false;
      for (let i = lo < 1 ? 1 : lo; i < hi; i++) {
        const idx = trail.at(i);
        const f = flags[idx];
        if ((f & TRAIL_BREAK) !== 0) {
          open = false;
          continue;
        }
        if (((f & TRAIL_MEASURED) !== 0 ? 1 : 0) !== kind) {
          open = false;
          continue;
        }
        if (!open) {
          const prev = trail.at(i - 1);
          ctx.moveTo(worldToCanvasX(xs[prev], s, pad), worldToCanvasY(ys[prev], s, pad));
          open = true;
        }
        ctx.lineTo(worldToCanvasX(xs[idx], s, pad), worldToCanvasY(ys[idx], s, pad));
      }
      ctx.stroke();
    }
  }

  ctx.setLineDash(DASH_NONE);
  ctx.globalAlpha = 1;
  ctx.lineWidth = 1;
  ctx.lineCap = "butt";
}

/**
 * Dropped lines from the rover to the two labelled axes.
 *
 * "Locate the rover in under a second" is the requirement, and a glyph on a
 * 120 cm grid is easy to miss. These give the eye a path to it from the edge
 * of the card, and they double as a readout: follow the line to the gutter and
 * the rover's x and y can be read straight off the centimetre ticks.
 */
function drawReticle(
  ctx: CanvasRenderingContext2D,
  pose: Pose,
  s: number,
  pad: number,
  theme: ArenaTheme,
): void {
  const cx = worldToCanvasX(pose.x_mm, s, pad);
  const cy = worldToCanvasY(pose.y_mm, s, pad);
  if (!Number.isFinite(cx) || !Number.isFinite(cy)) return;

  const x0 = worldToCanvasX(0, s, pad);
  const y0 = worldToCanvasY(ARENA_MM, s, pad);
  const wpx = ARENA_MM * s;
  const ink = pose.position === "assumed" ? theme.assumed : theme.trailMeasured;

  ctx.strokeStyle = ink;
  ctx.lineWidth = 1;
  ctx.globalAlpha = 0.3;
  ctx.setLineDash(DASH_RETICLE);
  ctx.beginPath();
  ctx.moveTo(x0, hair(cy));
  ctx.lineTo(cx, hair(cy));
  ctx.moveTo(hair(cx), y0 + wpx);
  ctx.lineTo(hair(cx), cy);
  ctx.stroke();

  // Ticks in the gutter, at full strength: this is the part that is read.
  // They stop short of the centimetre labels — the left ones are right-aligned
  // to x0-5, the bottom ones start at y0+wpx+5 — so the tick never lands on a
  // digit and make it unreadable.
  ctx.setLineDash(DASH_NONE);
  ctx.globalAlpha = 0.85;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(x0 - 4, hair(cy));
  ctx.lineTo(x0, hair(cy));
  ctx.moveTo(hair(cx), y0 + wpx);
  ctx.lineTo(hair(cx), y0 + wpx + 4);
  ctx.stroke();
  ctx.lineWidth = 1;
  ctx.globalAlpha = 1;
}

/** Pulse period for the uncertainty halo, ms. Slow — it is a state, not an alarm. */
const HALO_PERIOD_MS = 2600;

/** Concentric rings in the halo family. More than one is what makes it read as
 *  a gradient of belief rather than as a boundary. */
const HALO_RINGS = 3;

/**
 * Uncertainty rings, sized by how far the rover has dead-reckoned.
 *
 * ---------------------------------------------------------------------------
 * WHAT THIS IS AND IS NOT
 * ---------------------------------------------------------------------------
 * It is NOT an error bar, and nothing about it may be read as one — no radius
 * is printed, the rings pulse and fade so there is no hard edge to measure
 * against, and the scale factor is a drawing constant documented as such
 * (DRIFT_SPREAD_FRAC, arena.ts). It IS an honest statement of the one thing
 * that is definitely true about this rover's pose: nothing localises it, so the
 * further it drives the less the coordinate is worth, and the picture should
 * get vaguer as that happens rather than staying reassuringly crisp.
 *
 * The old halo was a single expanding ring at a FIXED radius, which said "not
 * pinned down" but said it identically at 5 cm driven and at 5 m. `spreadMm` is
 * the fix: it comes from `PoseTrail.drivenMm`, the odometer since the tracker
 * was last re-anchored, so the rings open as the run goes on and snap back the
 * moment a break re-anchors them.
 *
 * MEASURED AND ASSUMED STILL DIVERGE, which the header guarantees:
 *   assumed   amber, opens at ASSUMED_SPREAD_MM (half the start box) and does
 *             not grow, because an assumed pose has driven nowhere — the rover
 *             is somewhere in that box and that is the whole claim.
 *   measured  the measured ink, opens at nothing and grows with the odometer,
 *             around a glyph that is solid where the assumed one is dashed.
 */
function drawUncertainty(
  ctx: CanvasRenderingContext2D,
  cx: number,
  cy: number,
  spreadPx: number,
  /** Radius of the rover glyph, px. The rings start OUTSIDE it. */
  glyphPx: number,
  ink: string,
  tMs: number,
  stillOnly: boolean,
): void {
  if (!(spreadPx > 1.5)) return;

  ctx.strokeStyle = ink;
  ctx.lineWidth = 1.5;
  ctx.setLineDash(DASH_HALO);

  // ---- reduced motion ------------------------------------------------------
  // The site stylesheet kills CSS animation globally, and it cannot touch a
  // pixel this loop paints — a canvas is outside the reach of any media query.
  // So the rings have to opt out themselves, and the state has to SURVIVE the
  // opt-out: if the only thing saying "this position is uncertain" is a pulse,
  // then an operator who has asked for no motion is shown a map that quietly
  // stops mentioning it. Three static rings, stepped in alpha rather than in
  // time, carry the same gradient of belief with nothing moving.
  if (stillOnly) {
    for (let k = 0; k < HALO_RINGS; k++) {
      const f = (k + 1) / HALO_RINGS;
      ctx.globalAlpha = 0.34 * (1 - f * 0.6);
      ctx.beginPath();
      ctx.arc(cx, cy, glyphPx + spreadPx * (0.12 + f * 1.05), 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.setLineDash(DASH_NONE);
    ctx.globalAlpha = 1;
    ctx.lineWidth = 1;
    return;
  }

  const t = (tMs % HALO_PERIOD_MS) / HALO_PERIOD_MS;
  for (let k = 0; k < HALO_RINGS; k++) {
    // Each ring is offset a third of a cycle from the last, so one is always
    // emerging as another dies and the family never blinks out entirely.
    const phase = (t + k / HALO_RINGS) % 1;
    // Anchored to the chassis and growing outward from it. A ring drawn INSIDE
    // the glyph is invisible at best and reads as part of the rover at worst,
    // and it would mean a small spread showed nothing at all rather than
    // showing a tight one — the difference this whole mark exists to convey.
    const r = glyphPx + spreadPx * (0.12 + phase * 1.05);
    ctx.globalAlpha = 0.42 * (1 - phase);
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.setLineDash(DASH_NONE);
  ctx.globalAlpha = 1;
  ctx.lineWidth = 1;
}

function drawRover(
  ctx: CanvasRenderingContext2D,
  pose: Pose,
  s: number,
  pad: number,
  accent: string,
  theme: ArenaTheme,
  tMs: number,
  drivenMm: number,
  stillOnly: boolean,
): void {
  const cx = worldToCanvasX(pose.x_mm, s, pad);
  const cy = worldToCanvasY(pose.y_mm, s, pad);
  if (!Number.isFinite(cx) || !Number.isFinite(cy)) return;

  const L = ROVER_LEN_MM * s;
  const W = ROVER_WID_MM * s;
  const posAssumed = pose.position === "assumed";
  const hdgAssumed = pose.heading === "assumed";
  const bodyInk = posAssumed ? theme.assumed : accent;
  const noseInk = hdgAssumed ? theme.assumed : accent;

  // Uncertainty rings — see drawUncertainty. Expanding and concentric, never a
  // fixed circle, because a fixed circle states a radius and nobody has
  // measured this rover's positional error.
  //
  // An assumed pose opens at the start box and stays there; a measured one is
  // sized from the odometer and grows, because a measured pose here is still
  // dead reckoning with nothing correcting it. Both are drawn, in their own
  // ink, so "measured" is never allowed to look like "known".
  const spreadMm = posAssumed ? ASSUMED_SPREAD_MM : driftSpreadMm(drivenMm);
  drawUncertainty(
    ctx,
    cx,
    cy,
    spreadMm * s,
    Math.hypot(L, W) / 2,
    posAssumed ? theme.assumed : theme.measured,
    tMs,
    stillOnly,
  );

  ctx.save();
  ctx.translate(cx, cy);
  // NEGATIVE psi: canvas rotation is clockwise-positive because +y points
  // down, while heading is CCW-positive in the world frame. Without the sign
  // flip the rover turns the wrong way and the cloud detaches from the walls.
  ctx.rotate(-pose.heading_rad);

  // Heading ray with a head on it. Dashed when the bearing is assumed, solid
  // when the rover reported one — the ray is the largest thing on the glyph,
  // so it is where the distinction is most visible.
  const rayEnd = L * 2.2;
  ctx.strokeStyle = hdgAssumed ? INK.rayAssumed : INK.rayMeasured;
  ctx.lineWidth = 1;
  ctx.setLineDash(hdgAssumed ? DASH_HEADING : DASH_NONE);
  ctx.beginPath();
  ctx.moveTo(L / 2, 0);
  ctx.lineTo(rayEnd, 0);
  ctx.stroke();
  ctx.setLineDash(DASH_NONE);
  ctx.beginPath();
  ctx.moveTo(rayEnd, 0);
  ctx.lineTo(rayEnd - 5, -3);
  ctx.moveTo(rayEnd, 0);
  ctx.lineTo(rayEnd - 5, 3);
  ctx.stroke();

  // Chassis. Dashed and unfilled while the position is an assumption; solid
  // and filled once it is a measurement.
  ctx.fillStyle = posAssumed ? INK.bodyFillAssumed : INK.bodyFillMeasured;
  ctx.strokeStyle = bodyInk;
  ctx.lineWidth = 1.25;
  ctx.setLineDash(posAssumed ? DASH_ROVER : DASH_NONE);
  ctx.beginPath();
  ctx.rect(-L / 2, -W / 2, L, W);
  ctx.fill();
  ctx.stroke();
  ctx.setLineDash(DASH_NONE);

  // Nose. Filled = the rover told us which way it is pointing. Hollow = we
  // are pointing it FORWARD because that is where we asked it to start.
  ctx.beginPath();
  ctx.moveTo(L / 2 + Math.max(6, L * 0.28), 0);
  ctx.lineTo(L / 2, -W * 0.32);
  ctx.lineTo(L / 2, W * 0.32);
  ctx.closePath();
  if (hdgAssumed) {
    ctx.strokeStyle = noseInk;
    ctx.lineWidth = 1.25;
    ctx.stroke();
  } else {
    ctx.fillStyle = noseInk;
    ctx.fill();
  }

  ctx.restore();

  // Sensor origin: the point every range in the scan is measured from, and
  // the point the reticle lines converge on.
  ctx.fillStyle = theme.text;
  ctx.beginPath();
  ctx.arc(cx, cy, 1.6, 0, Math.PI * 2);
  ctx.fill();
}

// ==================================================================== HUD ===

/**
 * Rebuild the cached HUD strings. Called on the message edge, never per frame.
 *
 * Every numeric goes through `mm()` / `degText()`, which render an absent or
 * non-finite value as `--`. Not as 0: a zero here is a coordinate, and an
 * operator reading "X 0 mm" has been told the rover is against the left wall
 * when in fact nothing was reported at all.
 */
function buildHud(
  hud: Hud,
  ctx: CanvasRenderingContext2D,
  frame: ArenaFrame | null,
  pose: Pose,
  theme: ArenaTheme,
  connected: boolean,
  lastDrawMs: number,
  trail: PoseTrail,
  dev: NearestOnPath,
): void {
  const counts = frame ? frame.counts : null;

  if (!frame || frame.ptCount === 0) {
    hud.status = connected ? "AWAITING SCAN" : "LINK DOWN";
    hud.statusInk = connected ? theme.textDim : theme.danger;
    hud.counts = "";
    hud.near = "";
    hud.perf = "";
    hud.oob = "";
  } else {
    hud.status = "";
    hud.counts = `WALL ${pad2(counts![1])}  TREE ${pad2(counts![2])}  OBST ${pad2(counts![3])}`;
    const nm = frame.nearestMm;
    hud.near =
      nm >= 0
        ? `NEAR ${Math.round(nm).toString().padStart(4, " ")}mm @ ${frame.nearestBearingDeg
            .toString()
            .padStart(3, " ")}°`
        : "NEAR   -- mm";
    hud.perf = `PTS ${frame.ptCount}  ${(frame.ms + lastDrawMs).toFixed(1)}ms`;
    // An out-of-bounds count that is not ~0 means the scan does not fit the
    // arena: either ARENA_MM is wrong or the pose is. It is the cheapest
    // possible check and it catches the mistake that is hardest to see.
    hud.oob = frame.outOfBounds > 0 ? `OOB ${frame.outOfBounds}` : "";
    hud.oobInk = frame.outOfBounds > 8 ? theme.danger : theme.assumed;
  }

  // ---- pose provenance ----------------------------------------------------
  // This fleet publishes no pose: the heartbeat carries uptime/camera/lidar
  // flags and nothing else, so the rover is normally drawn at ROVER_START
  // facing FORWARD_HEADING_DEG. Even when a position DOES arrive it is
  // dead-reckoned off an assumed start with nothing correcting it, so
  // "MEASURED" is never promoted to "located" here.
  const posA = pose.position === "assumed";
  const hdgA = pose.heading === "assumed";
  if (posA) {
    hud.badge = "POSE ASSUMED";
    hud.badgeInk = theme.assumed;
    hud.sub = "NOTHING REPORTED A POSITION";
  } else if (hdgA) {
    hud.badge = "POS MEASURED · HDG ASSUMED";
    hud.badgeInk = theme.assumed;
    hud.sub = "DEAD-RECKONED · NOT LOCALISED";
  } else {
    hud.badge = "POSE MEASURED";
    hud.badgeInk = theme.measured;
    hud.sub = "DEAD-RECKONED · NOT LOCALISED";
  }
  // Both units on one line. Millimetres are what the rover reports and what the
  // mission executor plans in, so they are the primary figure; centimetres are
  // what the arena is marked in and what a tape measure reads, so an operator
  // checking the map against the floor does not have to divide by ten in their
  // head. The axis ladder is in cm and this is the bridge to it.
  hud.posX = `X ${mm(pose.x_mm)} ${cm(pose.x_mm)}${posA ? " ?" : ""}`;
  hud.posY = `Y ${mm(pose.y_mm)} ${cm(pose.y_mm)}${posA ? " ?" : ""}`;
  hud.hdg = `HDG ${degText(pose.heading_deg)}${hdgA ? " ?" : ""}`;

  // ---- drift, stated by its cause -----------------------------------------
  // The odometer, not an error figure. This is the MEASURED input the
  // uncertainty rings are sized from (see drawUncertainty); printing it instead
  // of the ring radius is what keeps the rings from being read as a bound
  // somebody measured. An assumed pose has driven nothing by definition, so it
  // renders `--` rather than a truthful-looking 0.
  hud.driven = posA ? "DRIVEN   -- mm" : `DRIVEN ${mm(trail.drivenMm)}`;

  // ---- plan versus actual, as a number ------------------------------------
  // Absent whenever either half of the comparison is missing: no route planned,
  // or a position nobody measured. `dev.dist_mm` is already Infinity in both
  // cases, so `mm()` renders `--` and the zero rule holds — "PLAN DEV 0 mm"
  // would tell an operator the rover is perfectly on the line at the exact
  // moment there is no line.
  hud.dev = `PLAN DEV ${mm(dev.dist_mm)}`;
  hud.devInk = Number.isFinite(dev.dist_mm) ? theme.text : theme.textDim;

  // ---- gutter cursor ------------------------------------------------------
  // Suppressed when the pose is outside the arena. The plates are clamped to
  // the card's edge, so an out-of-bounds coordinate would print a number at the
  // wall while the reticle pointed somewhere else — a readout that contradicts
  // the mark it belongs to is worse than no readout, and the block above still
  // carries the raw millimetres.
  const inX = Number.isFinite(pose.x_mm) && pose.x_mm >= 0 && pose.x_mm <= ARENA_MM;
  const inY = Number.isFinite(pose.y_mm) && pose.y_mm >= 0 && pose.y_mm <= ARENA_MM;
  hud.curOn = inX && inY;
  hud.curX = hud.curOn ? (pose.x_mm / 10).toFixed(1) : "";
  hud.curY = hud.curOn ? (pose.y_mm / 10).toFixed(1) : "";

  // measureText once per rebuild so the scrims can be sized without touching
  // text metrics inside the draw.
  ctx.font = MONO;
  let lw = 0;
  for (const t of [hud.status, hud.counts, hud.near, hud.perf, hud.oob]) {
    if (t) lw = Math.max(lw, ctx.measureText(t).width);
  }
  hud.leftW = lw;
  ctx.font = MONO_SM;
  let rw = 0;
  for (const t of [hud.badge, hud.sub, hud.posX, hud.posY, hud.hdg, hud.driven, hud.dev]) {
    rw = Math.max(rw, ctx.measureText(t).width);
  }
  hud.rightW = rw;
  hud.curXW = hud.curOn ? ctx.measureText(hud.curX).width : 0;
  hud.curYW = hud.curOn ? ctx.measureText(hud.curY).width : 0;
}

function drawHud(
  ctx: CanvasRenderingContext2D,
  size: number,
  hud: Hud,
  theme: ArenaTheme,
  stale: boolean,
  scanAgeMs: number,
): void {
  ctx.textBaseline = "alphabetic";

  // ---- left block: what the scan sees -------------------------------------
  const lineH = 12;
  if (hud.leftW > 0) {
    scrim(ctx, 4, 4, hud.leftW + 10, lineH * countNonEmpty(hud) + 8, theme);
  }
  ctx.font = MONO;
  ctx.textAlign = "left";
  let y = 16;
  const write = (text: string, color: string) => {
    if (!text) return;
    ctx.fillStyle = color;
    ctx.fillText(text, 9, y);
    y += lineH;
  };
  write(hud.status, hud.statusInk);
  write(hud.counts, theme.text);
  write(hud.near, theme.text);
  write(hud.perf, theme.textDim);
  write(hud.oob, hud.oobInk);

  // ---- staleness ----------------------------------------------------------
  // Only rendered once the feed has actually gone quiet, and the age is
  // bucketed to a tenth of a second so the string is rebuilt ten times a
  // second at worst rather than sixty.
  if (stale && Number.isFinite(scanAgeMs)) {
    const bucket = Math.floor(scanAgeMs / 100);
    if (bucket !== staleBucket) {
      staleBucket = bucket;
      staleText = `SCAN STALE ${(bucket / 10).toFixed(1)}s`;
    }
    ctx.font = MONO;
    ctx.fillStyle = theme.danger;
    ctx.fillText(staleText, 9, y);
  }

  // ---- right block: where we think the rover is ---------------------------
  const rx = size - 8;
  const rw = hud.rightW + 10;
  scrim(ctx, rx - rw + 5, 4, rw, 11 * 7 + 8, theme);
  ctx.font = MONO_SM;
  ctx.textAlign = "right";
  let ry = 15;
  const wr = (text: string, color: string) => {
    ctx.fillStyle = color;
    ctx.fillText(text, rx, ry);
    ry += 11;
  };
  wr(hud.badge, hud.badgeInk);
  wr(hud.sub, theme.textDim);
  wr(hud.posX, hud.badgeInk);
  wr(hud.posY, hud.badgeInk);
  wr(hud.hdg, hud.badgeInk);
  // The odometer takes the provenance ink too: it is derived from the same
  // positions, so it is exactly as trustworthy as they are and must not read as
  // an independent measurement sat under two caveated ones.
  wr(hud.driven, hud.badgeInk);
  wr(hud.dev, hud.devInk);
  ctx.textAlign = "left";
}

/** Module-level so the throttled stale string is not re-created per frame. */
let staleBucket = -1;
let staleText = "";

function countNonEmpty(hud: Hud): number {
  let n = 0;
  if (hud.status) n++;
  if (hud.counts) n++;
  if (hud.near) n++;
  if (hud.perf) n++;
  if (hud.oob) n++;
  return n;
}

/** Wash behind HUD text. Without it the grid runs straight through the digits. */
function scrim(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  w: number,
  h: number,
  theme: ArenaTheme,
): void {
  if (w <= 0 || h <= 0) return;
  // Hand-rolled rather than ctx.roundRect: this dashboard is opened from the
  // arena on whatever phone is nearest, and roundRect only reached Safari in
  // 16.4. A scrim that throws takes the whole rAF loop down with it.
  const r = Math.min(5, w / 2, h / 2);
  ctx.fillStyle = theme.scrim;
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
  ctx.fill();
}

/** Text with a knocked-out outline, so labels stay legible over the cloud. */
function haloText(
  ctx: CanvasRenderingContext2D,
  text: string,
  x: number,
  y: number,
  fill: string,
  halo: string,
): void {
  ctx.lineWidth = 3;
  ctx.lineJoin = "round";
  ctx.strokeStyle = halo;
  ctx.strokeText(text, x, y);
  ctx.fillStyle = fill;
  ctx.fillText(text, x, y);
}

// ------------------------------------------------------------------ utils ---

function pad2(v: number): string {
  return v.toString().padStart(2, " ");
}

/**
 * Millimetres for the HUD. A value that is not a finite number renders `--`.
 *
 * THE ZERO RULE. Formatting an absent field as 0 puts a real, readable
 * coordinate on screen for something nobody measured; an operator cannot tell
 * it apart from the rover genuinely sitting on the origin. Absent renders as
 * absent.
 */
function mm(v: number): string {
  return Number.isFinite(v) ? `${Math.round(v).toString().padStart(4, " ")} mm` : "  -- mm";
}

/**
 * The same value in centimetres, to one decimal — the unit the arena is marked
 * in and the axis ladder is labelled in.
 *
 * One decimal and no more. The requirement is centimetre readability, the pose
 * is dead-reckoned wheel odometry, and printing 97.24 cm would dress a drifting
 * estimate up as a tenth-of-a-millimetre measurement. Absent is `--`, never 0,
 * for the reason in `mm`.
 */
function cm(v: number): string {
  return Number.isFinite(v) ? `${(v / 10).toFixed(1).padStart(5, " ")} cm` : "   -- cm";
}

function degText(v: number): string {
  if (!Number.isFinite(v)) return "--";
  return `${Math.round(((v % 360) + 360) % 360)
    .toString()
    .padStart(3, "0")}°`;
}

function labelFor(cls: number, minR: number): string {
  const name = CLASS_NAMES[cls] ?? "?";
  // A range of 0 out of the clusterer means "no valid return was kept for this
  // track", not "it is touching the sensor".
  return minR > 0 ? `${name} ${Math.round(minR / 10)}cm` : `${name} --`;
}

/** Mix a #rrggbb toward another #rrggbb. t=0 returns `hex` untouched. */
function mixHex(hex: string, target: string, t: number): string {
  if (t <= 0) return hex;
  const a = parseHex(hex);
  const b = parseHex(target);
  if (!a || !b) return hex;
  const r = Math.round(a[0] + (b[0] - a[0]) * t);
  const g = Math.round(a[1] + (b[1] - a[1]) * t);
  const bl = Math.round(a[2] + (b[2] - a[2]) * t);
  return `rgb(${r},${g},${bl})`;
}

function parseHex(hex: string): [number, number, number] | null {
  if (hex.length !== 7 || hex[0] !== "#") return null;
  const r = parseInt(hex.slice(1, 3), 16);
  const g = parseInt(hex.slice(3, 5), 16);
  const b = parseInt(hex.slice(5, 7), 16);
  if (!Number.isFinite(r) || !Number.isFinite(g) || !Number.isFinite(b)) return null;
  return [r, g, b];
}

/**
 * Re-alpha a colour. Handles `#rrggbb` and `rgb()/rgba()`; anything else is
 * passed through untouched.
 *
 * The rgb() case is not hypothetical — `mixHex` returns one, so a zone ink on
 * a light background arrives here already mixed, and a hex-only version would
 * silently return it opaque and fill the zone solid.
 */
function withAlpha(color: string, a: number): string {
  const rgb = parseHex(color);
  if (rgb) return `rgba(${rgb[0]},${rgb[1]},${rgb[2]},${a})`;
  const open = color.indexOf("(");
  if (open > 0) {
    const parts = color.slice(open + 1, color.indexOf(")")).split(",");
    if (parts.length >= 3) {
      const r = Number(parts[0]);
      const g = Number(parts[1]);
      const b = Number(parts[2]);
      if (Number.isFinite(r) && Number.isFinite(g) && Number.isFinite(b)) {
        return `rgba(${r},${g},${b},${a})`;
      }
    }
  }
  return color;
}

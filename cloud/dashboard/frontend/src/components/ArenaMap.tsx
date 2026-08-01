import { memo, useEffect, useRef } from "react";
import {
  ARENA_MM,
  FORWARD_HEADING_DEG,
  GRID_MAJOR_MM,
  GRID_MINOR_MM,
  ROVER_LEN_MM,
  ROVER_WID_MM,
  START_BOX,
  TRAIL_BREAK,
  TRAIL_MEASURED,
  ZONES,
  createPoseTrail,
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
 * ---------------------------------------------------------------------------
 * LAYERS AND THE FRAME BUDGET
 * ---------------------------------------------------------------------------
 *   static   arena border, grid, zones, start box, axis labels, scale bar,
 *            heading legend, FORWARD marker. Redrawn on mount, on resize and
 *            on a theme change.
 *   dynamic  route, trail, cloud, objects, rover, HUD. Redrawn per animation
 *            frame.
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

const PAD_PX = 34;

const EMBER = "#f97316";
const MONO = '10px "JetBrains Mono", ui-monospace, monospace';
const MONO_SM = '9px "JetBrains Mono", ui-monospace, monospace';

/** Hoisted so the per-frame path setup allocates nothing. */
const DASH_NONE: number[] = [];
const DASH_RETICLE = [2, 4];
const DASH_TRAIL = [3, 3];
const DASH_ROVER = [3, 2.5];
const DASH_HALO = [2, 5];
const DASH_ROUTE = [5, 4];
const DASH_START = [4, 3];
const DASH_HEADING = [4, 4];

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
  startInk: string;
  /** HUD panel wash and the outline behind on-map labels. */
  scrim: string;
  halo: string;
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
  startInk: "rgba(148,163,184,0.45)",
  scrim: "rgba(7,9,13,0.72)",
  halo: "rgba(7,9,13,0.85)",
  trailMeasured: "#38bdf8",
  trailAssumed: "#fbbf24",
  assumed: "#fbbf24",
  measured: "#34d399",
  danger: "#fb7185",
  cloudNoise: 0.55,
  classColors: CLASS_COLORS_DARK,
  zoneMix: "#e2e8f0",
  zoneMixT: 0,
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
  startInk: "rgba(71,85,105,0.55)",
  scrim: "rgba(255,255,255,0.82)",
  halo: "rgba(255,255,255,0.9)",
  trailMeasured: "#0369a1",
  trailAssumed: "#b45309",
  assumed: "#b45309",
  measured: "#047857",
  danger: "#be123c",
  cloudNoise: 0.7,
  classColors: CLASS_COLORS_LIGHT,
  zoneMix: "#0f172a",
  zoneMixT: 0.45,
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
  const sizeRef = useRef(0);
  const drawMsRef = useRef(0);
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
    let raf = 0;
    let lastSeq = -1;
    let lastPoseEnv: unknown = Symbol("unset");
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

      const ageMs = scanAtRef.current > 0 ? Date.now() - scanAtRef.current : Infinity;
      if (scanChanged || poseChanged) {
        buildHud(
          hudRef.current,
          dctx,
          frameRef.current,
          poseRef.current,
          themeRef.current,
          lidar.metaRef.current.connected,
          drawMsRef.current,
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
        routeRef.current,
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

    return () => {
      disposed = true;
      cancelAnimationFrame(raf);
      ro.disconnect();
      mq?.removeEventListener?.("change", onScheme);
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

  // ---- zones --------------------------------------------------------------
  for (const z of ZONES) {
    const zx = worldToCanvasX(z.x_mm, s, pad);
    const zy = worldToCanvasY(z.y_mm + z.h_mm, s, pad); // top edge in canvas
    const zw = z.w_mm * s;
    const zh = z.h_mm * s;
    // The zone inks are picked for a dark floor; on white they wash out, so
    // they are pulled toward the text colour by the theme's mix factor. Hue is
    // preserved — the hue is how the operator tells the zones apart.
    const ink = mixHex(z.stroke, theme.zoneMix, theme.zoneMixT);

    ctx.fillStyle = theme.zoneMixT > 0 ? withAlpha(ink, 0.1) : z.fill;
    ctx.fillRect(zx, zy, zw, zh);

    if (z.hatch) {
      ctx.save();
      ctx.beginPath();
      ctx.rect(zx, zy, zw, zh);
      ctx.clip();
      ctx.strokeStyle = ink;
      ctx.globalAlpha = 0.22;
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let d = -zh; d < zw; d += 9) {
        ctx.moveTo(zx + d, zy + zh);
        ctx.lineTo(zx + d + zh, zy);
      }
      ctx.stroke();
      ctx.restore();
    }

    ctx.strokeStyle = ink;
    ctx.globalAlpha = 0.75;
    ctx.lineWidth = 1;
    ctx.strokeRect(hair(zx), hair(zy), Math.round(zw), Math.round(zh));
    ctx.globalAlpha = 1;

    // Label at the BOTTOM of the zone, not the top. The two upper zones reach
    // the top edge of the arena, which is where the HUD block and the pose
    // badge are anchored, and a top-left label put ZONE B directly under the
    // badge. Anchoring low keeps every zone name clear of both.
    ctx.fillStyle = ink;
    ctx.font = MONO_SM;
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    ctx.fillText(z.label, zx + 5, zy + zh - 4);
  }

  // ---- start box ----------------------------------------------------------
  // Dashed, unfilled and unlabelled in a zone colour, because it is not a
  // destination: it is where the operator is asked to put the rover, and where
  // the glyph is drawn when nothing reports a position. Making it look like
  // ZONE A or ZONE B would imply the rover navigates to it.
  {
    const bx = worldToCanvasX(START_BOX.x_mm, s, pad);
    const by = worldToCanvasY(START_BOX.y_mm + START_BOX.h_mm, s, pad);
    const bw = START_BOX.w_mm * s;
    const bh = START_BOX.h_mm * s;
    ctx.strokeStyle = theme.startInk;
    ctx.lineWidth = 1;
    ctx.setLineDash(DASH_START);
    ctx.strokeRect(hair(bx), hair(by), Math.round(bw), Math.round(bh));
    ctx.setLineDash(DASH_NONE);
    ctx.fillStyle = theme.startInk;
    ctx.font = MONO_SM;
    ctx.textAlign = "right";
    ctx.textBaseline = "bottom";
    ctx.fillText("START", bx + bw - 5, by + bh - 4);
  }

  // ---- border -------------------------------------------------------------
  ctx.strokeStyle = theme.id === "dark" ? withAlpha(accent, 0.55) : theme.border;
  ctx.lineWidth = 1;
  ctx.strokeRect(hair(x0), hair(y0), Math.round(wpx), Math.round(wpx));

  // ---- axis ticks ---------------------------------------------------------
  // Labelled in centimetres: the arena is 120 cm and operators measure it with
  // a tape, not in millimetres.
  ctx.fillStyle = theme.textDim;
  ctx.font = MONO_SM;
  for (let v = 0; v <= ARENA_MM; v += GRID_MAJOR_MM) {
    const px = worldToCanvasX(v, s, pad);
    const py = worldToCanvasY(v, s, pad);
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    ctx.fillText(`${Math.round(v / 10)}`, px, y0 + wpx + 5);
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    ctx.fillText(`${Math.round(v / 10)}`, x0 - 5, py);
  }
  // Axis names, so "0..120" is not left as a bare number sequence. Origin is
  // bottom-left; both captions sit at the far end of their own axis.
  ctx.textAlign = "left";
  ctx.textBaseline = "top";
  ctx.fillText("X cm", x0 + wpx + 3, y0 + wpx + 5);
  ctx.textAlign = "right";
  ctx.textBaseline = "bottom";
  ctx.fillText("Y cm", x0 - 3, y0 - 3);

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

  // ---- scale bar ----------------------------------------------------------
  // Bottom-centre: the gap between the water station and the start box is the
  // one strip of floor no zone claims.
  const barMm = GRID_MAJOR_MM;
  const barPx = barMm * s;
  const bx = x0 + wpx / 2 - barPx / 2;
  const by = y0 + wpx - 11;
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
  ctx.stroke();
  ctx.globalAlpha = 1;
  ctx.fillStyle = theme.text;
  ctx.globalAlpha = 0.7;
  ctx.font = MONO_SM;
  ctx.textAlign = "center";
  ctx.textBaseline = "bottom";
  ctx.fillText(`${Math.round(barMm / 10)} cm`, bx + barPx / 2, by - 4);
  ctx.globalAlpha = 1;
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";

  // ---- FORWARD marker -----------------------------------------------------
  drawForwardMarker(ctx, x0, y0, wpx, theme);
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
  route?: readonly RoutePoint[] | null,
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

  // ---- trail --------------------------------------------------------------
  drawTrail(ctx, trail, s, pad, theme);

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
  drawRover(ctx, pose, s, pad, accent, theme, tMs);

  // ---- HUD ----------------------------------------------------------------
  drawHud(ctx, size, hud, theme, stale, scanAgeMs);
}

/** Ink for the planned route. Cool and desaturated so it never competes with
 *  the accent-coloured live scan — the plan is context, the scan is truth. */
const ROUTE_INK_DARK = "rgba(125,211,252,0.85)";
const ROUTE_FILL_DARK = "rgba(125,211,252,0.16)";
const ROUTE_INK_LIGHT = "rgba(2,132,199,0.9)";
const ROUTE_FILL_LIGHT = "rgba(2,132,199,0.14)";

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
 * Where the rover has actually been.
 *
 * Older samples fade, so the recent path reads as the current one without
 * throwing history away. Two properties are load-bearing:
 *
 *   - a MEASURED run is a solid cyan line; an ASSUMED run is dashed amber. A
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

  ctx.lineWidth = 1.5;
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

function drawRover(
  ctx: CanvasRenderingContext2D,
  pose: Pose,
  s: number,
  pad: number,
  accent: string,
  theme: ArenaTheme,
  tMs: number,
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

  // Uncertainty halo. Expanding rings, not a fixed circle, because a fixed
  // circle states a radius and nobody has measured this rover's positional
  // error. The ring says "not pinned down" and declines to say how far.
  if (posAssumed) {
    const t = (tMs % HALO_PERIOD_MS) / HALO_PERIOD_MS;
    const r = L * 0.85 + t * L * 1.7;
    ctx.strokeStyle = theme.assumed;
    ctx.globalAlpha = 0.4 * (1 - t);
    ctx.lineWidth = 1.5;
    ctx.setLineDash(DASH_HALO);
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash(DASH_NONE);
    ctx.globalAlpha = 1;
  }

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
  hud.posX = `X ${mm(pose.x_mm)}${posA ? " ?" : ""}`;
  hud.posY = `Y ${mm(pose.y_mm)}${posA ? " ?" : ""}`;
  hud.hdg = `HDG ${degText(pose.heading_deg)}${hdgA ? " ?" : ""}`;

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
  for (const t of [hud.badge, hud.sub, hud.posX, hud.posY, hud.hdg]) {
    rw = Math.max(rw, ctx.measureText(t).width);
  }
  hud.rightW = rw;
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
  scrim(ctx, rx - rw + 5, 4, rw, 11 * 5 + 8, theme);
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

import { memo, useEffect, useRef } from "react";
import {
  ARENA_MM,
  FORWARD_HEADING_DEG,
  GRID_MAJOR_MM,
  GRID_MINOR_MM,
  ROVER_LEN_MM,
  ROVER_WID_MM,
  ZONES,
  readPose,
  scaleFor,
  worldToCanvasX,
  worldToCanvasY,
  type Pose,
} from "../lib/arena";
import {
  CLASS_COLORS,
  CLASS_NAMES,
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
 *      90 deg CCW from +x. The static layer paints a FORWARD arrow at the top
 *      edge so that convention is on screen, not just in a comment.
 *
 * Because the map never rotates, the FORWARD marker is a permanent, honest
 * reference: it belongs to the static layer precisely so it can never be
 * mistaken for something that tracks the rover.
 *
 * Two stacked canvases:
 *   static   arena border, grid, zones, axis labels, scale bar, FORWARD
 *            marker. Redrawn only on mount and on resize.
 *   dynamic  cloud, oriented bounding boxes, labels, rover, HUD. Redrawn per
 *            animation frame.
 *
 * React renders this component approximately once. The LiDAR socket writes to
 * refs (useChannelRef), a rAF loop polls the sequence number, and clustering
 * runs only when the number changes — the stream is 2 Hz, so re-segmenting at
 * 60 Hz would be thirty times the work for the same picture.
 */

const PAD_PX = 30;

const INK_950 = "#07090d";
const INK_900 = "#0b0f16";
const EMBER = "#f97316";
const GRID_MINOR = "rgba(148,163,184,0.055)";
const GRID_MAJOR = "rgba(148,163,184,0.14)";
const TEXT_DIM = "rgba(148,163,184,0.55)";
/** Orientation marker ink — brighter than the ticks; it is a legend, not chrome. */
const FORWARD_INK = "rgba(226,232,240,0.72)";
const MONO = '10px "JetBrains Mono", ui-monospace, monospace';
const MONO_SM = '9px "JetBrains Mono", ui-monospace, monospace';

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

  const frameRef = useRef<ArenaFrame | null>(null);
  const poseRef = useRef<Pose>(readPose(null, null));
  const sizeRef = useRef(0);
  const drawMsRef = useRef(0);

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
    let raf = 0;
    let lastSeq = -1;
    let disposed = false;

    const resize = () => {
      const size = Math.max(120, Math.floor(wrap.clientWidth));
      const dpr = window.devicePixelRatio || 1;
      if (size === sizeRef.current && sc.width === Math.round(size * dpr)) return;
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
      drawStatic(sctx, size);
    };

    const loop = () => {
      if (disposed) return;
      raf = requestAnimationFrame(loop);

      const seq = lidar.seqRef.current;
      if (seq !== lastSeq) {
        lastSeq = seq;
        const env = lidar.dataRef.current;
        const data = env && typeof env === "object" ? (env as any).data : null;
        poseRef.current = readPose(poseEnvRef.current, env);
        frameRef.current = engine.update(
          data ? data.ranges_m : null,
          data && typeof data.range_max_m === "number" ? data.range_max_m : 6.0,
          poseRef.current,
        );
      }

      const t0 = performance.now();
      drawDynamic(
        dctx,
        sizeRef.current,
        frameRef.current,
        poseRef.current,
        accent,
        lidar.metaRef.current.connected,
        drawMsRef.current,
        routeRef.current,
      );
      drawMsRef.current = performance.now() - t0;
    };

    resize();
    raf = requestAnimationFrame(loop);

    const ro = new ResizeObserver(resize);
    ro.observe(wrap);

    return () => {
      disposed = true;
      cancelAnimationFrame(raf);
      ro.disconnect();
    };
    // `accent` and `thing` are stable for the life of a card; the refs above
    // carry everything that actually changes.
  }, [accent, lidar]);

  return (
    <div ref={wrapRef} className="relative aspect-square w-full">
      <canvas
        ref={staticRef}
        className="absolute inset-0 rounded-xl bg-ink-950 ring-1 ring-white/5"
      />
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

function drawStatic(ctx: CanvasRenderingContext2D, size: number): void {
  const s = scaleFor(size, PAD_PX);
  const pad = PAD_PX;

  ctx.clearRect(0, 0, size, size);
  ctx.fillStyle = INK_950;
  ctx.fillRect(0, 0, size, size);

  const x0 = worldToCanvasX(0, s, pad);
  const y0 = worldToCanvasY(ARENA_MM, s, pad); // world top -> canvas top
  const wpx = ARENA_MM * s;

  // Arena floor, one step lighter than the surround so the playable area
  // reads as a distinct object rather than a grid floating on a panel.
  ctx.fillStyle = INK_900;
  ctx.fillRect(x0, y0, wpx, wpx);

  // ---- grid ---------------------------------------------------------------
  ctx.lineWidth = 1;
  ctx.strokeStyle = GRID_MINOR;
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

  ctx.strokeStyle = GRID_MAJOR;
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

    ctx.fillStyle = z.fill;
    ctx.fillRect(zx, zy, zw, zh);

    if (z.hatch) {
      ctx.save();
      ctx.beginPath();
      ctx.rect(zx, zy, zw, zh);
      ctx.clip();
      ctx.strokeStyle = z.stroke;
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

    ctx.strokeStyle = z.stroke;
    ctx.globalAlpha = 0.75;
    ctx.lineWidth = 1;
    ctx.strokeRect(hair(zx), hair(zy), Math.round(zw), Math.round(zh));
    ctx.globalAlpha = 1;

    ctx.fillStyle = z.stroke;
    ctx.font = MONO_SM;
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    ctx.fillText(z.label, zx + 5, zy + 5);
  }

  // ---- border -------------------------------------------------------------
  ctx.strokeStyle = accentAlpha(EMBER, 0.55);
  ctx.lineWidth = 1;
  ctx.strokeRect(hair(x0), hair(y0), Math.round(wpx), Math.round(wpx));

  // ---- axis ticks ---------------------------------------------------------
  // Labelled in centimetres: the arena is 120 cm and operators measure it with
  // a tape, not in millimetres.
  ctx.fillStyle = TEXT_DIM;
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
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
  ctx.fillText("cm", x0 + wpx + 4, y0 + wpx + 12);

  // ---- scale bar ----------------------------------------------------------
  const barMm = GRID_MAJOR_MM;
  const barPx = barMm * s;
  const bx = x0 + wpx - barPx - 8;
  const by = y0 + wpx - 12;
  ctx.strokeStyle = "rgba(226,232,240,0.6)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(hair(bx), hair(by) - 3);
  ctx.lineTo(hair(bx), hair(by) + 3);
  ctx.moveTo(hair(bx), hair(by));
  ctx.lineTo(hair(bx + barPx), hair(by));
  ctx.moveTo(hair(bx + barPx), hair(by) - 3);
  ctx.lineTo(hair(bx + barPx), hair(by) + 3);
  ctx.stroke();
  ctx.fillStyle = "rgba(226,232,240,0.7)";
  ctx.font = MONO_SM;
  ctx.textAlign = "center";
  ctx.textBaseline = "bottom";
  ctx.fillText(`${Math.round(barMm / 10)} cm`, bx + barPx / 2, by - 4);
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";

  // ---- FORWARD marker -----------------------------------------------------
  drawForwardMarker(ctx, x0, y0, wpx);
}

/**
 * Fixed orientation reference: an arrow labelled FORWARD, pinned just inside
 * the arena's top edge.
 *
 * This lives on the STATIC layer deliberately. It is a property of the map,
 * not of the rover: it says "up the screen is up the arena is world +y is
 * heading 90 deg" and it must keep saying that no matter where the rover
 * points. Putting it on the dynamic layer, or inside drawRover's rotated
 * transform, would turn it into a second heading indicator and destroy the
 * one thing it is for.
 *
 * The direction is DERIVED from FORWARD_HEADING_DEG rather than hardcoded to
 * screen-up, so the marker and the rover's start heading cannot drift apart:
 * change the constant and this arrow follows. It is still static — the input
 * is a compile-time constant, never `pose.heading_rad`.
 *
 * Drawn once per mount/resize, so measureText and the trig here are off the
 * hot path entirely.
 */
function drawForwardMarker(
  ctx: CanvasRenderingContext2D,
  x0: number,
  y0: number,
  wpx: number,
): void {
  // Label is deliberately just the word: the numeric convention
  // (FORWARD_HEADING_DEG = 90 deg = world +y) is reported in the HUD, and a
  // shorter string keeps the marker inside the gap between the two top zones
  // even on the narrowest card.
  const label = "FORWARD";

  ctx.font = MONO_SM;
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";

  const textW = ctx.measureText(label).width;
  const arrowW = 9;
  const gap = 5;
  // Just INSIDE the arena's top edge, horizontally centred between zones A and
  // B. The top margin is already occupied by the HUD's right-aligned pose
  // badge, and a marker that collides with a warning is a marker nobody reads.
  const midY = y0 + 14;
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

  ctx.strokeStyle = FORWARD_INK;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(snapX ? hair(tailX) : tailX, snapY ? hair(tailY) : tailY);
  ctx.lineTo(snapX ? hair(tipX) : tipX, snapY ? hair(tipY) : tipY);
  ctx.stroke();

  // Arrowhead: apex at the tip, base one head-length back along the shaft.
  const headLen = 7;
  const baseX = tipX - dx * headLen;
  const baseY = tipY - dy * headLen;
  ctx.fillStyle = FORWARD_INK;
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
  connected: boolean,
  lastDrawMs: number,
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
  drawRoute(ctx, route, s, pad);

  // ---- point cloud --------------------------------------------------------
  if (frame && frame.ptCount > 0) {
    const pts = frame.pts;
    const n = frame.ptCount;
    ctx.fillStyle = accentAlpha(accent, 0.55);
    for (let i = 0; i < n; i++) {
      const px = worldToCanvasX(pts[i * 2], s, pad);
      const py = worldToCanvasY(pts[i * 2 + 1], s, pad);
      ctx.fillRect(px - 0.9, py - 0.9, 1.8, 1.8);
    }
  }

  // ---- objects ------------------------------------------------------------
  if (frame) {
    const tracks = frame.tracks;
    for (let i = 0; i < tracks.length; i++) {
      const tk = tracks[i];
      if (!tk.active) continue;
      const color = CLASS_COLORS[tk.cls];

      // PCA already gave us a centroid, a unit axis and the extents along and
      // across it, so the four corners of the oriented box are free.
      const hl = Math.max(tk.len, 8) / 2;
      const hw = Math.max(tk.wid, 8) / 2;
      const ax = tk.axX;
      const ay = tk.axY;
      const nx = -ay;
      const ny = ax;

      ctx.globalAlpha = Math.max(0, Math.min(1, tk.alpha));
      ctx.strokeStyle = color;
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let k = 0; k < 4; k++) {
        const sl = k === 0 || k === 3 ? 1 : -1;
        const sw = k < 2 ? 1 : -1;
        const wx = tk.cx + ax * hl * sl + nx * hw * sw;
        const wy = tk.cy + ay * hl * sl + ny * hw * sw;
        const cxp = worldToCanvasX(wx, s, pad);
        const cyp = worldToCanvasY(wy, s, pad);
        if (k === 0) ctx.moveTo(cxp, cyp);
        else ctx.lineTo(cxp, cyp);
      }
      ctx.closePath();
      ctx.stroke();

      // A label is a claim. Only make it once the majority vote has had time
      // to settle, otherwise every frame renames the same object.
      if (isLabelable(tk)) {
        const lx = worldToCanvasX(tk.cx, s, pad);
        const ly = worldToCanvasY(tk.cy, s, pad);
        ctx.fillStyle = color;
        ctx.font = MONO_SM;
        ctx.textAlign = "center";
        ctx.textBaseline = "bottom";
        ctx.fillText(CLASS_NAMES[tk.cls], lx, ly - hw * s - 3);
        ctx.textAlign = "left";
        ctx.textBaseline = "alphabetic";
      }
      ctx.globalAlpha = 1;
    }
  }

  // ---- rover --------------------------------------------------------------
  drawRover(ctx, pose, s, pad, accent);

  // ---- HUD ----------------------------------------------------------------
  drawHud(ctx, size, frame, pose, connected, lastDrawMs);
}

/** Ink for the planned route. Cool and desaturated so it never competes with
 *  the accent-coloured live scan — the plan is context, the scan is truth. */
const ROUTE_INK = "rgba(125,211,252,0.85)";
const ROUTE_FILL = "rgba(125,211,252,0.16)";

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
): void {
  if (!route || route.length < 2) return;

  const px = (p: RoutePoint) => worldToCanvasX(p.x_mm, s, pad);
  const py = (p: RoutePoint) => worldToCanvasY(p.y_mm, s, pad);

  ctx.save();

  // Spine.
  ctx.strokeStyle = ROUTE_INK;
  ctx.lineWidth = 1.5;
  ctx.setLineDash([5, 4]);
  ctx.lineJoin = "round";
  ctx.beginPath();
  ctx.moveTo(px(route[0]), py(route[0]));
  for (let i = 1; i < route.length; i++) ctx.lineTo(px(route[i]), py(route[i]));
  ctx.stroke();
  ctx.setLineDash([]);

  // Waypoint nodes. The first is the start and already carries the rover, so
  // it is skipped — two markers on one spot reads as an error.
  for (let i = 1; i < route.length - 1; i++) {
    const p = route[i];
    ctx.beginPath();
    ctx.arc(px(p), py(p), p.dock ? 3 : 2, 0, Math.PI * 2);
    if (p.dock) {
      ctx.fillStyle = ROUTE_INK;
      ctx.fill();
    } else {
      ctx.strokeStyle = ROUTE_INK;
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
  ctx.fillStyle = ROUTE_FILL;
  ctx.fill();
  ctx.strokeStyle = ROUTE_INK;
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

function drawRover(
  ctx: CanvasRenderingContext2D,
  pose: Pose,
  s: number,
  pad: number,
  accent: string,
): void {
  const cx = worldToCanvasX(pose.x_mm, s, pad);
  const cy = worldToCanvasY(pose.y_mm, s, pad);
  const L = ROVER_LEN_MM * s;
  const W = ROVER_WID_MM * s;

  ctx.save();
  ctx.translate(cx, cy);
  // NEGATIVE psi: canvas rotation is clockwise-positive because +y points
  // down, while heading is CCW-positive in the world frame. Without the sign
  // flip the rover turns the wrong way and the cloud detaches from the walls.
  ctx.rotate(-pose.heading_rad);

  // heading ray
  ctx.strokeStyle = accentAlpha(accent, 0.5);
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(L / 2, 0);
  ctx.lineTo(L * 2.5, 0);
  ctx.stroke();
  ctx.setLineDash([]);

  // chassis
  ctx.fillStyle = accentAlpha(accent, 0.16);
  ctx.strokeStyle = accent;
  ctx.lineWidth = 1.25;
  ctx.beginPath();
  ctx.rect(-L / 2, -W / 2, L, W);
  ctx.fill();
  ctx.stroke();

  // nose
  ctx.fillStyle = accent;
  ctx.beginPath();
  ctx.moveTo(L / 2 + Math.max(6, L * 0.28), 0);
  ctx.lineTo(L / 2, -W * 0.32);
  ctx.lineTo(L / 2, W * 0.32);
  ctx.closePath();
  ctx.fill();

  ctx.restore();

  // sensor origin
  ctx.fillStyle = "#e2e8f0";
  ctx.beginPath();
  ctx.arc(cx, cy, 1.6, 0, Math.PI * 2);
  ctx.fill();
}

function drawHud(
  ctx: CanvasRenderingContext2D,
  size: number,
  frame: ArenaFrame | null,
  pose: Pose,
  connected: boolean,
  lastDrawMs: number,
): void {
  const x = 8;
  let y = 12;
  const line = 12;

  ctx.font = MONO;
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";

  const counts = frame ? frame.counts : null;
  const write = (text: string, color: string) => {
    ctx.fillStyle = color;
    ctx.fillText(text, x, y);
    y += line;
  };

  if (!frame || frame.ptCount === 0) {
    write(connected ? "AWAITING SCAN" : "LINK DOWN", connected ? TEXT_DIM : "#fb7185");
  } else {
    write(
      `WALL ${pad2(counts![1])}  TREE ${pad2(counts![2])}  OBST ${pad2(counts![3])}`,
      "#cbd5e1",
    );
    const nm = frame.nearestMm;
    write(
      nm >= 0
        ? `NEAR ${Math.round(nm).toString().padStart(4, " ")}mm @ ${frame.nearestBearingDeg
            .toString()
            .padStart(3, " ")}°`
        : "NEAR    —",
      "#cbd5e1",
    );
    write(`PTS ${frame.ptCount}  ${(frame.ms + lastDrawMs).toFixed(1)}ms`, TEXT_DIM);

    // An out-of-bounds count that is not ~0 means the scan does not fit the
    // arena: either ARENA_MM is wrong or the pose is. It is the cheapest
    // possible check and it catches the mistake that is hardest to see.
    if (frame.outOfBounds > 0) {
      write(`OOB ${frame.outOfBounds}`, frame.outOfBounds > 8 ? "#fb7185" : "#fbbf24");
    }
  }

  // ---- pose provenance ----------------------------------------------------
  // This fleet publishes no pose: the heartbeat carries uptime/camera/lidar
  // flags and nothing else. So the rover is drawn at ROVER_START facing
  // FORWARD_HEADING_DEG, and the operator has to be told that the bearing on
  // screen is an assumption rather than a measurement — otherwise they will
  // trust a number the rover never reported. Hence two lines, not one: the
  // existing SIMULATED badge plus the heading and its provenance.
  if (pose.simulated) {
    const deg = (pose.heading_rad * 180) / Math.PI;
    // Every numeric that reaches a formatter is finite-guarded; heading_rad
    // is already clamped in readPose, but this HUD must never be the thing
    // that throws on top of a degraded link.
    const degText = Number.isFinite(deg)
      ? `${Math.round(((deg % 360) + 360) % 360)}°`
      : "—";

    ctx.font = MONO_SM;
    ctx.fillStyle = "#fbbf24";
    ctx.textAlign = "right";
    ctx.fillText("POSE: SIMULATED", size - 8, 12);
    ctx.fillText(`HDG ${degText} ASSUMED — NOT MEASURED`, size - 8, 23);
    ctx.textAlign = "left";
  }
}

// ------------------------------------------------------------------ utils ---

function pad2(v: number): string {
  return v.toString().padStart(2, " ");
}

/** #rrggbb -> rgba(). Accepts any 6-digit hex; anything else passes through. */
function accentAlpha(hex: string, a: number): string {
  if (hex.length !== 7 || hex[0] !== "#") return hex;
  const r = parseInt(hex.slice(1, 3), 16);
  const g = parseInt(hex.slice(3, 5), 16);
  const b = parseInt(hex.slice(5, 7), 16);
  if (!Number.isFinite(r) || !Number.isFinite(g) || !Number.isFinite(b)) return hex;
  return `rgba(${r},${g},${b},${a})`;
}

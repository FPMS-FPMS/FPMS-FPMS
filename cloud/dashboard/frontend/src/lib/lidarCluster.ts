/**
 * LiDAR segmentation, PCA shape fitting, classification and temporal tracking.
 *
 * Pure TypeScript — no React, no DOM, no canvas. Import it from a worker, a
 * node script or a component; the only contract is `createClusterEngine()`.
 *
 * ---------------------------------------------------------------------------
 * PAYLOAD CONTRACT (verified against rover/fpms_rover_agent.py lidar_loop)
 * ---------------------------------------------------------------------------
 *   data.ranges_m   number[360], ARRAY INDEX IS THE DEGREE, metres.
 *                   BINS = 360 in the agent and the emit does
 *                   `round(min(mm / 1000.0, RANGE_MAX_M), 3) if mm else 0.0`.
 *   0.0             no return in that bin — NOT "zero distance". Skipping
 *                   these is what removes the starburst through the origin.
 *   range_max_m     6.0. Ranges are CLAMPED to it, so a bin sitting at 6.000
 *                   means "nothing there", not "surface at six metres".
 *                   RANGE_SATURATION_FRAC trims that saturated shell off.
 *
 * ---------------------------------------------------------------------------
 * ALLOCATION POLICY
 * ---------------------------------------------------------------------------
 * Every buffer, cluster record and track record is allocated once in the
 * constructor and reused forever. There is no .map/.filter/.slice, no object
 * or array literal, and no closure created inside update(). At 2 Hz this
 * hardly matters; it matters a lot that the renderer above can call into it
 * from a requestAnimationFrame loop without producing GC sawtooth.
 */

import {
  ARENA_MM,
  LIDAR_ANGLE_SIGN,
  LIDAR_ZERO_OFFSET_DEG,
  RANGE_SATURATION_FRAC,
  type Pose,
} from "./arena";

// --------------------------------------------------------------- classes ---

export const CLS_NOISE = 0;
export const CLS_WALL = 1;
export const CLS_TREE = 2;
export const CLS_OBSTACLE = 3;

export type ClassId = 0 | 1 | 2 | 3;

export const CLASS_NAMES: readonly string[] = ["NOISE", "WALL", "TREE", "OBSTACLE"];

/** slate / slate-light / green / amber — deliberately low-chroma except hits. */
export const CLASS_COLORS: readonly string[] = ["#475569", "#94a3b8", "#4ade80", "#fbbf24"];

// -------------------------------------------------------------- tunables ---

const N_BINS = 360;

/** Dietmayer adaptive segmentation: r-jump tolerance = C0 + C1 * min(r, r'). */
const C0_MM = 30;
const C1 = 0.017453;

/** Bins may be empty (0.0) mid-object; tolerate a short drop-out before cutting. */
const MAX_BIN_GAP = 3;

/**
 * Corner-split tolerance for the IEPF pass, millimetres.
 *
 * Range-discontinuity segmentation alone cannot separate the walls of a
 * closed arena: a corner is perfectly continuous in r, so a rover sitting
 * inside a box sees one unbroken contour and reports it as a single blob
 * that is neither straight nor small — an OBSTACLE swallowing the entire
 * room. Splitting each segment at its point of maximum deviation from the
 * end-to-end chord recovers the individual walls.
 *
 * The threshold is deliberately just above the largest radius a TREE can
 * have (TREE_MAX_SIDE_MM / 2 = 70 mm), so a round object is never split into
 * arcs, while a corner — which deviates by hundreds of millimetres — always
 * is.
 */
const SPLIT_TOL_MM = 80;

/** Pool sizes. 64 simultaneous objects is far beyond what a 1.2 m arena holds. */
const POOL = 64;

/** Temporal association radius, millimetres. */
const MATCH_DIST_MM = 120;
const MATCH_DIST_SQ = MATCH_DIST_MM * MATCH_DIST_MM;

/** Class-vote window and fade-out, in frames. */
const HIST = 5;
const FADE_FRAMES = 3;
const FADE_STEP = 1 / FADE_FRAMES;

/** Label suppression: a track must survive this many frames before it is named. */
const MIN_LABEL_AGE = 2;

/** Classification thresholds. WALL length scales with the arena. */
const WALL_MIN_LEN_MM = 0.25 * ARENA_MM;
const WALL_MAX_RESID_MM = 25;
const WALL_MIN_LINEARITY = 0.95;
const TREE_MAX_SIDE_MM = 140;

/**
 * Maximum L/W for a TREE.
 *
 * The intuitive value is 2.0 — "roughly as deep as it is wide" — but at 1 deg
 * angular resolution that makes the TREE class unreachable, which the bench
 * check caught. A 100 mm post is only sampled across the few degrees it
 * subtends, and depth along the ray rises almost vertically as the beam
 * approaches the tangent, so the last bin before the limb still lands near
 * the front of the cylinder. The measured minor extent is therefore far
 * smaller than the true 50 mm radius. Actual figures for a 100 mm post:
 *
 *     range    n    L      W      L/W
 *      300mm  19   87.3   25.6   3.41
 *      600mm   9   79.7   19.8   4.03
 *      900mm   7   92.3   30.8   3.00
 *     1200mm   5   81.8   21.2   3.85
 *
 * 4.5 clears the whole measured band with margin — 4.0 put the 600 mm case
 * at 4.03, on the wrong side of its own threshold — while TREE_MAX_SIDE_MM
 * keeps the class genuinely small: anything longer than 140 mm is an OBSTACLE
 * regardless of how thin it is.
 */
const TREE_MAX_ASPECT = 4.5;
const MIN_CLUSTER_POINTS = 3;
const TREE_MIN_POINTS = 4;

/**
 * Slack on the out-of-bounds counter, millimetres.
 *
 * The arena walls ARE the boundary, so returns off them land on x = 0 or
 * x = ARENA_MM to within measurement noise and a zero-tolerance test flags a
 * fifth of every scan — an alarm that is always on tells you nothing. The
 * counter exists to catch a wrong ARENA_MM or a bad pose, and both of those
 * throw points metres out, not millimetres.
 */
const OOB_TOL_MM = 25;

// ------------------------------------------------------------------ LUTs ---

/**
 * Body-frame direction per bin, built once at module load.
 *   theta = LIDAR_ANGLE_SIGN * (i + LIDAR_ZERO_OFFSET_DEG) * PI / 180
 * 720 doubles total. Removes two trig calls per point per frame.
 */
const SIN_LUT = new Float64Array(N_BINS);
const COS_LUT = new Float64Array(N_BINS);
for (let i = 0; i < N_BINS; i++) {
  const th = (LIDAR_ANGLE_SIGN * (i + LIDAR_ZERO_OFFSET_DEG) * Math.PI) / 180;
  SIN_LUT[i] = Math.sin(th);
  COS_LUT[i] = Math.cos(th);
}

// --------------------------------------------------------------- records ---

/** One segmented object in the current frame. Pooled and overwritten. */
export type Cluster = {
  n: number;
  /** centroid, world mm */
  cx: number;
  cy: number;
  /** unit principal axis, world frame */
  axX: number;
  axY: number;
  /** extent along / across the principal axis, mm */
  len: number;
  wid: number;
  linearity: number;
  rmsResid: number;
  cls: ClassId;
  /** closest point in the cluster: range mm and raw sensor bin (degrees) */
  minR: number;
  bearingDeg: number;
};

/**
 * A temporally smoothed object. `cls` is the MAJORITY of the last HIST
 * frames, not this frame's guess — a wall that momentarily fits a tree does
 * not make the label blink. Tracks persist for FADE_FRAMES after they stop
 * being seen so objects dissolve instead of popping.
 */
export type Track = {
  active: boolean;
  id: number;
  cx: number;
  cy: number;
  axX: number;
  axY: number;
  len: number;
  wid: number;
  n: number;
  cls: ClassId;
  /** frames matched in a row */
  age: number;
  /** 1 while seen, decaying to 0 over FADE_FRAMES once lost */
  alpha: number;
  /** true if this track was matched in the current frame */
  fresh: boolean;
  minR: number;
  bearingDeg: number;
};

/** Per-frame output. The SAME object every call — read it, do not retain it. */
export type ArenaFrame = {
  /** interleaved world-mm xy pairs, valid for [0, 2*ptCount) */
  pts: Float32Array;
  ptCount: number;
  /** points outside [0,ARENA_MM]^2 — early warning that ARENA_MM is wrong */
  outOfBounds: number;
  /** pool of length POOL; iterate and skip !active */
  tracks: Track[];
  /** live object count per ClassId */
  counts: Int32Array;
  /** nearest valid return this frame, mm; -1 when the scan is empty */
  nearestMm: number;
  nearestBearingDeg: number;
  clusterCount: number;
  /** clustering wall-clock, milliseconds */
  ms: number;
  frame: number;
};

// ------------------------------------------------------------- classifier ---

/**
 * First match wins — the order is the whole algorithm.
 *
 * A wall is long, straight and thin in residual. A tree (or post, or cone) is
 * small in both directions and roughly round. Everything else with enough
 * support is an unclassified obstacle. Anything with fewer than three points
 * cannot have a meaningful covariance and is noise by definition.
 */
export function classify(
  n: number,
  len: number,
  wid: number,
  linearity: number,
  rmsResid: number,
): ClassId {
  if (n < MIN_CLUSTER_POINTS) return CLS_NOISE;
  if (len >= WALL_MIN_LEN_MM && rmsResid <= WALL_MAX_RESID_MM && linearity >= WALL_MIN_LINEARITY) {
    return CLS_WALL;
  }
  if (
    len <= TREE_MAX_SIDE_MM &&
    wid <= TREE_MAX_SIDE_MM &&
    len / Math.max(wid, 1) <= TREE_MAX_ASPECT &&
    n >= TREE_MIN_POINTS
  ) {
    return CLS_TREE;
  }
  return CLS_OBSTACLE;
}

// ----------------------------------------------------------------- engine ---

class ClusterEngine {
  // --- scan-space scratch, indexed 0..m-1 over VALID bins only -------------
  private readonly px = new Float32Array(N_BINS); // world x, mm
  private readonly py = new Float32Array(N_BINS); // world y, mm
  private readonly pr = new Float32Array(N_BINS); // range, mm
  private readonly pb = new Int16Array(N_BINS); // originating sensor bin
  private readonly brk = new Uint8Array(N_BINS); // 1 = segment starts here
  private readonly ord = new Int16Array(N_BINS); // circular visit order

  /** Explicit IEPF work stack of (start, n) pairs — recursion without frames. */
  private readonly stack = new Int32Array(512);

  private readonly clusters: Cluster[] = [];
  private readonly tracks: Track[] = [];
  private readonly vote = new Int32Array(4);
  /** per-track ring buffer of raw class votes, POOL x HIST, flattened */
  private readonly hist = new Int8Array(POOL * HIST);
  private readonly histPos = new Int32Array(POOL);
  private readonly histLen = new Int32Array(POOL);

  private readonly out: ArenaFrame;
  private nextId = 1;
  private clusterCount = 0;

  constructor() {
    for (let i = 0; i < POOL; i++) {
      this.clusters.push({
        n: 0,
        cx: 0,
        cy: 0,
        axX: 1,
        axY: 0,
        len: 0,
        wid: 0,
        linearity: 0,
        rmsResid: 0,
        cls: CLS_NOISE,
        minR: 0,
        bearingDeg: 0,
      });
      this.tracks.push({
        active: false,
        id: 0,
        cx: 0,
        cy: 0,
        axX: 1,
        axY: 0,
        len: 0,
        wid: 0,
        n: 0,
        cls: CLS_NOISE,
        age: 0,
        alpha: 0,
        fresh: false,
        minR: 0,
        bearingDeg: 0,
      });
    }
    this.out = {
      pts: new Float32Array(N_BINS * 2),
      ptCount: 0,
      outOfBounds: 0,
      tracks: this.tracks,
      counts: new Int32Array(4),
      nearestMm: -1,
      nearestBearingDeg: 0,
      clusterCount: 0,
      ms: 0,
      frame: 0,
    };
  }

  /**
   * Consume one scan. `ranges` is the raw ranges_m array (metres, index =
   * degree); `pose` places it in the world. Safe to call with null/short/
   * garbage input — it degrades to an empty frame rather than throwing.
   */
  update(ranges: ArrayLike<number> | null | undefined, rangeMaxM: number, pose: Pose): ArenaFrame {
    const t0 = now();
    const out = this.out;
    out.frame++;
    out.outOfBounds = 0;
    out.nearestMm = -1;
    out.nearestBearingDeg = 0;
    this.clusterCount = 0;

    const m = this.project(ranges, rangeMaxM, pose);
    out.ptCount = m;

    if (m >= MIN_CLUSTER_POINTS) {
      this.segment(m);
    }
    out.clusterCount = this.clusterCount;

    this.associate();

    out.ms = now() - t0;
    return out;
  }

  /** Reset tracking state — call when the stream drops or the rover jumps. */
  reset(): void {
    for (let i = 0; i < POOL; i++) {
      this.tracks[i].active = false;
      this.histLen[i] = 0;
      this.histPos[i] = 0;
    }
    this.clusterCount = 0;
    this.out.ptCount = 0;
  }

  // ------------------------------------------------------------ stage 1 ---
  /**
   * Polar bins -> body frame -> world frame, dropping empty and saturated
   * returns. Returns the number of valid points written to the scratch
   * buffers (call it m; scratch indices 0..m-1 stay in ascending bin order).
   */
  private project(
    ranges: ArrayLike<number> | null | undefined,
    rangeMaxM: number,
    pose: Pose,
  ): number {
    if (!ranges) return 0;
    const nIn = ranges.length | 0;
    if (nIn <= 0) return 0;

    const rmaxM = Number.isFinite(rangeMaxM) && rangeMaxM > 0 ? rangeMaxM : 6.0;
    const satMm = rmaxM * 1000 * RANGE_SATURATION_FRAC;

    const psi = pose.heading_rad;
    const cp = Math.cos(psi);
    const sp = Math.sin(psi);
    const ox = pose.x_mm;
    const oy = pose.y_mm;

    const px = this.px;
    const py = this.py;
    const pr = this.pr;
    const pb = this.pb;
    const pts = this.out.pts;

    const lim = nIn < N_BINS ? nIn : N_BINS;
    let m = 0;
    let oob = 0;
    let best = Infinity;
    let bestBin = 0;

    for (let i = 0; i < lim; i++) {
      const rm = ranges[i];
      if (typeof rm !== "number" || !Number.isFinite(rm)) continue;
      const r = rm * 1000;
      // 0.0 = no return; >= saturation = the driver's clamp, not a surface.
      if (r <= 0 || r >= satMm) continue;

      const xb = r * COS_LUT[i];
      const yb = r * SIN_LUT[i];
      const xw = ox + xb * cp - yb * sp;
      const yw = oy + xb * sp + yb * cp;

      px[m] = xw;
      py[m] = yw;
      pr[m] = r;
      pb[m] = i;
      pts[m * 2] = xw;
      pts[m * 2 + 1] = yw;
      m++;

      if (r < best) {
        best = r;
        bestBin = i;
      }
      if (
        xw < -OOB_TOL_MM ||
        xw > ARENA_MM + OOB_TOL_MM ||
        yw < -OOB_TOL_MM ||
        yw > ARENA_MM + OOB_TOL_MM
      ) {
        oob++;
      }
    }

    this.out.outOfBounds = oob;
    if (m > 0) {
      this.out.nearestMm = best;
      this.out.nearestBearingDeg = bestBin;
    }
    return m;
  }

  // ------------------------------------------------------------ stage 2 ---
  /**
   * Sequential adaptive segmentation (Dietmayer). Cut between neighbouring
   * returns when either
   *   - the bin gap exceeds MAX_BIN_GAP (an angular hole), or
   *   - |r_k - r_{k-1}| exceeds C0 + C1 * min(r_k, r_{k-1}) — the range term
   *     widens the tolerance with distance so a far wall does not shatter
   *     into fragments purely from angular spacing.
   *
   * The 359/0 seam is a real neighbour, so the scan is walked circularly:
   * find any break, start there, and a run that straddles the seam comes out
   * as one cluster instead of two half-objects.
   */
  private segment(m: number): void {
    const pr = this.pr;
    const pb = this.pb;
    const brk = this.brk;

    for (let k = 0; k < m; k++) {
      const p = k === 0 ? m - 1 : k - 1;
      const gap = k === 0 ? pb[0] + N_BINS - pb[m - 1] : pb[k] - pb[p];
      const rk = pr[k];
      const rp = pr[p];
      const tol = C0_MM + C1 * (rk < rp ? rk : rp);
      brk[k] = gap > MAX_BIN_GAP || Math.abs(rk - rp) > tol ? 1 : 0;
    }

    let start = -1;
    for (let k = 0; k < m; k++) {
      if (brk[k]) {
        start = k;
        break;
      }
    }
    // No break anywhere: the whole scan is one closed contour — the rover
    // sitting inside an unbroken box, which is the normal case in this arena.
    // It goes to emit() as a single run and the IEPF pass cuts it at the
    // corners.
    const circular = start >= 0;
    if (!circular) start = 0;

    const ord = this.ord;
    let runStart = 0;
    let runN = 0;

    for (let t = 0; t < m; t++) {
      const k = start + t < m ? start + t : start + t - m;
      ord[t] = k;
      if (t > 0 && circular && brk[k]) {
        this.emit(runStart, runN);
        runStart = t;
        runN = 0;
      }
      runN++;
    }
    this.emit(runStart, runN);
  }

  // ----------------------------------------------------------- stage 2b ---
  /**
   * Iterative End Point Fit. Split a segment at its point of greatest
   * perpendicular deviation from the chord joining its endpoints, recurse,
   * and finalize whatever survives under SPLIT_TOL_MM.
   *
   * When the endpoints coincide — which is exactly what happens for the
   * unbroken contour of a closed room, where the run wraps all the way round
   * to its own start — the chord is degenerate and deviation falls back to
   * plain distance from the first point. That first cut lands on the far
   * corner and the recursion resolves the rest.
   */
  private emit(runStart: number, runN: number): void {
    if (runN <= 0) return;

    const st = this.stack;
    let sp = 0;
    st[sp++] = runStart;
    st[sp++] = runN;

    while (sp > 0) {
      const n = st[--sp];
      const a = st[--sp];
      if (this.clusterCount >= POOL) return;
      if (n < 3 || sp + 4 > st.length) {
        this.finalize(a, n);
        continue;
      }

      const i0 = this.ord[a];
      const i1 = this.ord[a + n - 1];
      const x0 = this.px[i0];
      const y0 = this.py[i0];
      const dx = this.px[i1] - x0;
      const dy = this.py[i1] - y0;
      const chord = Math.sqrt(dx * dx + dy * dy);
      const degenerate = chord < 1e-6;

      let best = -1;
      let bestT = -1;
      for (let t = a + 1; t < a + n - 1; t++) {
        const k = this.ord[t];
        const ex = this.px[k] - x0;
        const ey = this.py[k] - y0;
        const d = degenerate
          ? Math.sqrt(ex * ex + ey * ey)
          : Math.abs(dx * ey - dy * ex) / chord;
        if (d > best) {
          best = d;
          bestT = t;
        }
      }

      if (best > SPLIT_TOL_MM && bestT > a && bestT < a + n - 1) {
        st[sp++] = a;
        st[sp++] = bestT - a + 1;
        st[sp++] = bestT;
        st[sp++] = a + n - bestT;
      } else {
        this.finalize(a, n);
      }
    }
  }

  // ------------------------------------------------------------ stage 3 ---
  /**
   * Close out one run: closed-form 2x2 PCA from the accumulated moments,
   * then a short second pass over the run to measure extent along and across
   * the principal axis.
   *
   *   t  = (Sxx + Syy) / 2
   *   d  = sqrt(((Sxx - Syy) / 2)^2 + Sxy^2)
   *   l1 = t + d   (major)      l2 = max(0, t - d)   (minor)
   *
   * linearity = 1 - l2/l1 is 1 for a perfect line, 0 for an isotropic blob.
   * rmsResid  = sqrt(l2) is the RMS distance off the fitted axis, in mm —
   * directly comparable to a wall-flatness tolerance.
   */
  private finalize(runStart: number, runN: number): void {
    if (runN <= 0 || this.clusterCount >= POOL) return;

    // Pass 1: raw moments. Accumulated over the run in one sweep; the PCA
    // below is closed form, so no iterative solve is needed.
    let ax0 = 0;
    let ay0 = 0;
    let axx = 0;
    let ayy = 0;
    let axy = 0;
    const stop = runStart + runN;
    for (let t = runStart; t < stop; t++) {
      const k = this.ord[t];
      const x = this.px[k];
      const y = this.py[k];
      ax0 += x;
      ay0 += y;
      axx += x * x;
      ayy += y * y;
      axy += x * y;
    }

    const inv = 1 / runN;
    const cx = ax0 * inv;
    const cy = ay0 * inv;
    const sxx = axx * inv - cx * cx;
    const syy = ayy * inv - cy * cy;
    const sxy = axy * inv - cx * cy;

    const tr = (sxx + syy) / 2;
    const df = (sxx - syy) / 2;
    const d = Math.sqrt(df * df + sxy * sxy);
    const l1 = tr + d;
    const l2 = tr - d > 0 ? tr - d : 0;

    const linearity = l1 > 1e-9 ? 1 - l2 / l1 : 0;
    const rmsResid = Math.sqrt(l2);

    // Major eigenvector. Both expressions are valid; the longer one is the
    // numerically stable choice when the cluster is axis-aligned.
    let ax = sxy;
    let ay = l1 - sxx;
    const bx = l1 - syy;
    const by = sxy;
    if (bx * bx + by * by > ax * ax + ay * ay) {
      ax = bx;
      ay = by;
    }
    const mag = Math.sqrt(ax * ax + ay * ay);
    if (mag > 1e-9) {
      ax /= mag;
      ay /= mag;
    } else {
      ax = 1;
      ay = 0;
    }

    // Second pass: project onto the axis for L, onto its normal for W, and
    // pick up the nearest return while we are already touching the points.
    let minA = Infinity;
    let maxA = -Infinity;
    let minP = Infinity;
    let maxP = -Infinity;
    let minR = Infinity;
    let minBin = 0;
    const end = runStart + runN;
    for (let t = runStart; t < end; t++) {
      const k = this.ord[t];
      const dx = this.px[k] - cx;
      const dy = this.py[k] - cy;
      const a = dx * ax + dy * ay;
      const p = -dx * ay + dy * ax;
      if (a < minA) minA = a;
      if (a > maxA) maxA = a;
      if (p < minP) minP = p;
      if (p > maxP) maxP = p;
      const r = this.pr[k];
      if (r < minR) {
        minR = r;
        minBin = this.pb[k];
      }
    }

    const c = this.clusters[this.clusterCount];
    c.n = runN;
    c.cx = cx;
    c.cy = cy;
    c.axX = ax;
    c.axY = ay;
    c.len = maxA - minA;
    c.wid = maxP - minP;
    c.linearity = linearity;
    c.rmsResid = rmsResid;
    c.minR = minR === Infinity ? 0 : minR;
    c.bearingDeg = minBin;
    c.cls = classify(runN, c.len, c.wid, linearity, rmsResid);
    this.clusterCount++;
  }

  // ------------------------------------------------------------ stage 4 ---
  /**
   * Greedy nearest-centroid association against the previous frame, then a
   * majority vote over the last HIST class guesses.
   *
   * This is the entire anti-flicker mechanism. Frame-to-frame the segmenter
   * will happily call the same wall a WALL, then an OBSTACLE when a corner
   * clips it, then a WALL again. Voting over five frames and refusing to
   * label anything younger than MIN_LABEL_AGE turns that into a stable
   * annotation; the FADE_STEP decay keeps a briefly-occluded object on
   * screen instead of blinking out.
   */
  private associate(): void {
    const tracks = this.tracks;
    for (let i = 0; i < POOL; i++) tracks[i].fresh = false;

    for (let ci = 0; ci < this.clusterCount; ci++) {
      const c = this.clusters[ci];
      if (c.cls === CLS_NOISE) continue;

      let bi = -1;
      let bd = MATCH_DIST_SQ;
      for (let i = 0; i < POOL; i++) {
        const tk = tracks[i];
        if (!tk.active || tk.fresh) continue;
        const dx = tk.cx - c.cx;
        const dy = tk.cy - c.cy;
        const dd = dx * dx + dy * dy;
        if (dd < bd) {
          bd = dd;
          bi = i;
        }
      }

      if (bi < 0) {
        for (let i = 0; i < POOL; i++) {
          if (!tracks[i].active) {
            bi = i;
            tracks[i].active = true;
            tracks[i].id = this.nextId++;
            tracks[i].age = 0;
            this.histLen[i] = 0;
            this.histPos[i] = 0;
            break;
          }
        }
      }
      if (bi < 0) continue; // pool exhausted; drop the extra object

      const tk = tracks[bi];
      tk.cx = c.cx;
      tk.cy = c.cy;
      tk.axX = c.axX;
      tk.axY = c.axY;
      tk.len = c.len;
      tk.wid = c.wid;
      tk.n = c.n;
      tk.minR = c.minR;
      tk.bearingDeg = c.bearingDeg;
      tk.age++;
      tk.alpha = 1;
      tk.fresh = true;
      this.pushVote(bi, c.cls);
      tk.cls = this.majority(bi);
    }

    const counts = this.out.counts;
    counts[0] = counts[1] = counts[2] = counts[3] = 0;
    for (let i = 0; i < POOL; i++) {
      const tk = tracks[i];
      if (!tk.active) continue;
      if (!tk.fresh) {
        tk.alpha -= FADE_STEP;
        if (tk.alpha <= 0) {
          tk.active = false;
          tk.alpha = 0;
          continue;
        }
      } else {
        counts[tk.cls]++;
      }
    }
  }

  private pushVote(i: number, cls: ClassId): void {
    const base = i * HIST;
    this.hist[base + this.histPos[i]] = cls;
    this.histPos[i] = (this.histPos[i] + 1) % HIST;
    if (this.histLen[i] < HIST) this.histLen[i]++;
  }

  private majority(i: number): ClassId {
    const vote = this.vote;
    vote[0] = vote[1] = vote[2] = vote[3] = 0;
    const base = i * HIST;
    const n = this.histLen[i];
    for (let k = 0; k < n; k++) vote[this.hist[base + k]]++;
    let best: ClassId = CLS_OBSTACLE;
    let bv = -1;
    for (let c = 0; c < 4; c++) {
      if (vote[c] > bv) {
        bv = vote[c];
        best = c as ClassId;
      }
    }
    return best;
  }
}

export type { ClusterEngine };

export function createClusterEngine(): ClusterEngine {
  return new ClusterEngine();
}

/** True once a track has been seen long enough to deserve a printed label. */
export function isLabelable(tk: Track): boolean {
  return tk.active && tk.age >= MIN_LABEL_AGE && tk.cls !== CLS_NOISE;
}

function now(): number {
  return typeof performance !== "undefined" ? performance.now() : Date.now();
}

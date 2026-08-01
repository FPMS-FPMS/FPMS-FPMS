/**
 * FPMS public analytics + offline demo replay.
 *
 * WHAT THIS IS FOR
 * ----------------
 * The rover is offline most of the time. `/live` answers "is anything on fire?"
 * well, but when every rover is dark it is a page of empty tables — which is
 * exactly when a judge, a teacher or a parent is most likely to open it. This
 * module adds two things:
 *
 *   1. ANALYTICS over the archive the system already keeps: what the rovers
 *      detected, when they ran, how long they stayed up, how much of the LiDAR
 *      sweep came back, and what the pack voltage did. All of it drawn as
 *      self-contained inline SVG, because /live ships a strict CSP with
 *      `default-src 'none'` and no CDN can ever load into it.
 *
 *   2. A REPLAY of one representative run, so the page still demonstrates the
 *      system working when nothing is live.
 *
 * THE REPLAY IS A SIMULATION AND SAYS SO, LOUDLY
 * ----------------------------------------------
 * There is no captured run in this repository, so the demo dataset is SCRIPTED
 * — built by buildDemoRun() from a fixed scenario table. It is not measured
 * data and it must never be mistaken for measured data. Presenting it as live
 * telemetry to competition judges would be dishonest, so "simulated" is carried
 * on five independent channels that cannot be styled away one at a time:
 *
 *   - the transport:  a separate endpoint, /api/public/demo, that touches
 *                     neither D1 nor the Durable Object;
 *   - the payload:    `simulated: true`, `source: "simulated"` and a plain
 *                     English `disclaimer` on every response;
 *   - the timeline:   anchored to a FIXED past date (DEMO_ANCHOR_MS), never to
 *                     Date.now(), so it can never read as "just happened";
 *   - the page:       a persistent banner, an amber rail, a 45 degree hatch on
 *                     every chart card, a "SIM" badge on every chart title and
 *                     a rotated SIMULATED watermark inside every plot;
 *   - assistive tech: `aria-live` announcement on mode change and a "Simulated:"
 *                     prefix inside every <svg><title>, which is what a screen
 *                     reader reads out.
 *
 * Live analytics and demo analytics are never blended. The section renders one
 * dataset or the other, and the mode is stamped on the section element.
 *
 * ============================================================================
 * WIRING — the main agent must add exactly this. Nothing here edits worker.js,
 * public.js or chat.js.
 * ============================================================================
 *
 * In `cloud/cloudapp/src/public.js`:
 *
 * (W1) Import, at the top of the file:
 *
 *        import {
 *          handleAnalytics, ANALYTICS_STYLE, ANALYTICS_HTML, ANALYTICS_SCRIPT,
 *        } from "./analytics.js";
 *
 * (W2) Route, inside `handlePublic`, AFTER the `scrapeRetryAfter` burst guard
 *      and BEFORE the final `return publicJson({ error: "not found" }, 404)` —
 *      so analytics inherits the same rate limiting as every other public feed:
 *
 *        const an = await handleAnalytics(request, env, url, path, hubStub);
 *        if (an) return an;
 *
 * (W3) Page CSS — inside the existing <style> block of PUBLIC_PAGE, on the line
 *      before `</style>`:            ${ANALYTICS_STYLE}
 *
 * (W4) Page markup — inside PUBLIC_PAGE, immediately before `<footer>`:
 *                                    ${ANALYTICS_HTML}
 *
 * (W5) Page script — inside PUBLIC_PAGE, immediately before the closing
 *      script tag that follows the existing IIFE:
 *                                    ${ANALYTICS_SCRIPT}
 *      (ANALYTICS_SCRIPT is a complete <script> element. It contains no
 *      closing-script-tag sequence, so it is safe to interpolate.)
 *
 * (W6) OPTIONAL, one line. Inside the page's existing `render(d)` function add:
 *
 *        if (window.FPMS_ANALYTICS) window.FPMS_ANALYTICS.onSummary(d);
 *
 *      This lets the analytics section switch to the demo the moment the live
 *      poll reports zero rovers online, instead of waiting for its own fetch.
 *      It is genuinely optional: without it the module decides from its own
 *      /api/public/analytics response and behaves identically, just later.
 *
 * No CSP change is required: everything is inline style, inline script and
 * inline SVG, all of which the existing policy already allows. No new binding,
 * no new secret, no wrangler.jsonc change.
 *
 * ============================================================================
 * DATA — every field below is one the system actually produces. Nothing here
 * invents a metric, and a metric with no rows renders a designed empty state
 * rather than a zero.
 * ============================================================================
 *
 *   readings.kind='events'          subtype fire | obstacle | wildlife | fault
 *                                   (+ the *_cleared resolutions, counted
 *                                   separately as "resolved", never as
 *                                   detections)
 *   readings.subtype='pose'         data.uptime_s, data.camera_ok, data.lidar_ok
 *                                   — despite the name this is the rover
 *                                   agent's 5 s health heartbeat, not a pose
 *   readings.subtype='lidar'        data.sectors_mm (12 x 30 degree sectors,
 *                                   null where nothing came back),
 *                                   data.min_mm, data.points, data.range_max_m
 *   readings.subtype='drive'        data.battery_v, data.battery_stale
 *                                   (ESP32-S3 /battery, decivolts -> volts)
 *   readings.subtype='mission'      data.elapsed_s, data.distance_travelled_mm,
 *                                   data.batt_v (battery fallback)
 *   reports.severity                ok | warning | critical
 *
 * WHAT IS DELIBERATELY NOT EXPOSED
 * --------------------------------
 * public.js design rule 1 is "whitelist fields, never blacklist", and rule 3
 * extends that to generated text. Two consequences here:
 *
 *   - Every query names its columns through json_extract() instead of pulling
 *     `data` and picking fields in JS. A field that is never fetched cannot be
 *     leaked by a later refactor — and it also keeps 360-float `ranges_m`
 *     arrays and base64 `frame` blobs out of the response entirely.
 *   - No rover-authored string is echoed. Not the mission name, not the phase,
 *     not detection labels, not wildlife species. Runs are numbered, event
 *     types come from a closed vocabulary in this file, and anything outside it
 *     is folded into "other".
 *
 * TIME UNITS
 * ----------
 * `readings.ts` and `reports.ts` are unix SECONDS. Anything bound into SQL is
 * therefore seconds; anything handed to the browser goes through toMs() first.
 * Fields whose names end in `_s` that are DURATIONS — uptime_s, elapsed_s,
 * duration_s — are left alone: they are not instants and toMs() would be wrong.
 * Every conversion below is commented with which of the two it is, because a
 * seconds/milliseconds mixup in this system once made every rover read offline.
 */

/* ------------------------------------------------------------------ config */

/** Default analytics window. Bounded: the archive query never walks further. */
const DEFAULT_DAYS = 14;
const MAX_DAYS = 30;

/**
 * How many rovers get per-stream charts.
 *
 * Each rover costs four index-driven queries, and the categorical palette caps
 * comfortably at three series for the all-pairs chart forms. Both limits land
 * in the same place, so one constant serves both.
 */
const MAX_ROVERS = 3;

/** Newest-N sample per stream, per rover. */
const SAMPLE_POSE = 150;
const SAMPLE_LIDAR = 150;
const SAMPLE_DRIVE = 150;
const SAMPLE_MISSION = 120;

/**
 * Analytics is history, not a live reading, so it is cached far longer than the
 * 10 s the status feed uses. Workers Caching serves a hit without invoking the
 * Worker at all, which is what keeps a public, unauthenticated aggregate off
 * the D1 read quota.
 */
const ANALYTICS_CACHE_S = 300;

/** The demo is a constant. It can be cached for a day. */
const DEMO_CACHE_S = 86400;

/** A gap longer than this ends a run. Mission telemetry is relayed every 15 s. */
const RUN_GAP_S = 120;

/** Most recent runs to report. */
const MAX_RUNS = 8;

/** Rover names, same shape worker.js accepts. Validated before it is echoed. */
const THING_RE = /^[a-z0-9_.-]{1,32}$/i;

/** LiDAR sweep summary: 12 sectors of 30 degrees. From the rover's own encoder. */
const LIDAR_SECTORS = 12;

/**
 * Pack low-voltage threshold, from BATT_LOW_V in the rover's mission and teleop
 * services. Drawn as a reference line so a reader can see how close a run got
 * without the chart needing a zero baseline it would never use.
 */
const BATT_LOW_V = 11.1;

/** Arena side, from ARENA_MM. Used only as a scale hint on the LiDAR plot. */
const ARENA_MM = 1200;

/**
 * The closed vocabulary of detection types. A rover can invent a subtype; it
 * cannot invent a series. Order is fixed and IS the color assignment — slot 1
 * through 4 of the categorical palette, never cycled, never reassigned by rank.
 */
const DETECTION_TYPES = ["wildlife", "fire", "obstacle", "fault"];

/** Resolutions. Counted so "3 fires, 3 cleared" is legible, never as detections. */
const RESOLVED_TYPES = { fire_cleared: "fire", obstacle_cleared: "obstacle" };

/** reports.severity is already a closed vocabulary; restate it so it stays one. */
const SEVERITIES = ["ok", "warning", "critical"];

/* -------------------------------------------------------------- utilities */

/**
 * Seconds-or-milliseconds normaliser. Same implementation and same reasoning as
 * public.js: 1e12 is about 2001 in ms and year 33658 in seconds, so it splits
 * the two cleanly. Duplicated rather than imported because this module is meant
 * to be self-contained and public.js does not export it.
 */
function toMs(ts) {
  const n = Number(ts);
  if (!n || !Number.isFinite(n)) return null;
  return n > 1e12 ? n : n * 1000;
}

/**
 * Finite number, or null. Nulls survive to the client and render as a gap in
 * the line rather than as a value.
 *
 * The explicit null/undefined/"" guard is load-bearing, not defensive noise:
 * `Number(null)` is 0 and `Number("")` is 0, so without it a json_extract that
 * found no such key would come out of here as a real-looking zero — a 0 V pack,
 * a 0 mm obstacle, a rover with 0 s uptime. Every one of those is a fabricated
 * data point, and two of them would also wreck the axis of the chart they land
 * in. Absent must stay absent.
 */
function num(v) {
  if (v === null || v === undefined || v === "") return null;
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

function clamp(n, lo, hi) {
  return n < lo ? lo : n > hi ? hi : n;
}

function logError(message, err) {
  console.error(JSON.stringify({
    message,
    error: err instanceof Error ? err.message : String(err),
  }));
}

function analyticsJson(obj, status = 200, maxAge = ANALYTICS_CACHE_S) {
  return new Response(JSON.stringify(obj), {
    status,
    headers: {
      "Content-Type": "application/json",
      "Cache-Control": `public, max-age=${maxAge}`,
      "Access-Control-Allow-Origin": "*",
      "X-Content-Type-Options": "nosniff",
    },
  });
}

/* ------------------------------------------------------------ the queries */

/**
 * Roster, from the Durable Object rather than from D1.
 *
 * publicSummary already explains why: `SELECT thing, MAX(ts) ... GROUP BY thing`
 * cannot use idx_readings_lookup for the max, so it degenerates into a whole
 * table scan on every poll. The hub holds this map in memory and is the
 * authority on it. Most recently seen first, so a fleet larger than MAX_ROVERS
 * charts the rovers that actually ran.
 */
async function analyticsRoster(env, hubStub) {
  try {
    const snap = await hubStub(env).fetch("https://hub/snapshot").then((r) => r.json());
    const out = [];
    const seen = snap?.last_seen;

    if (seen && typeof seen === "object") {
      for (const [name, ts] of Object.entries(seen)) {
        // `last_seen` maps a thing to unix SECONDS; toMs for the client.
        if (THING_RE.test(name)) out.push({ thing: name, last_seen: toMs(ts) });
      }
    }
    if (!out.length) {
      for (const name of (snap?.things_seen || [])) {
        const s = String(name);
        if (THING_RE.test(s)) out.push({ thing: s, last_seen: null });
      }
    }

    out.sort((a, b) => (b.last_seen || 0) - (a.last_seen || 0));
    return out.slice(0, MAX_ROVERS);
  } catch (err) {
    // A cold or unreachable DO costs the per-rover charts, not the whole page.
    logError("analytics: hub snapshot failed", err);
    return [];
  }
}

/**
 * Every statement analytics needs, as one D1 batch.
 *
 * Each one is both index-driven and bounded:
 *   - the two aggregates lead with a column their index leads with
 *     (idx_readings_events on kind, idx_reports_ts on ts) and carry a `ts >= ?`
 *     floor, so they are range scans that stop at the cutoff rather than table
 *     scans that stop at LIMIT;
 *   - the per-rover samples match idx_readings_lookup (thing, subtype, ts DESC)
 *     exactly, so `ORDER BY ts DESC LIMIT n` reads n rows and no more.
 *
 * json_extract keeps `ranges_m` (360 floats) and `frame` (base64 JPEG) out of
 * the result set entirely — cheaper on the wire, and structurally unable to
 * leak a field nobody named.
 */
function analyticsStatements(env, things, sinceS) {
  const db = env.DB;
  const stmts = [];
  const plan = [];

  const add = (kind, thing, stmt) => {
    plan.push({ kind, thing });
    stmts.push(stmt);
  };

  add("events", null, db.prepare(
    "SELECT CAST(ts / 86400 AS INTEGER) AS day, subtype, COUNT(*) AS n " +
    "FROM readings WHERE kind = 'events' AND ts >= ? " +
    "GROUP BY day, subtype ORDER BY day DESC LIMIT 400",
  ).bind(sinceS));

  add("reports", null, db.prepare(
    "SELECT CAST(ts / 86400 AS INTEGER) AS day, severity, COUNT(*) AS n " +
    "FROM reports WHERE ts >= ? GROUP BY day, severity ORDER BY day DESC LIMIT 200",
  ).bind(sinceS));

  for (const { thing } of things) {
    add("pose", thing, db.prepare(
      "SELECT ts, json_extract(data, '$.uptime_s') AS uptime_s, " +
      "json_extract(data, '$.camera_ok') AS camera_ok, " +
      "json_extract(data, '$.lidar_ok') AS lidar_ok " +
      "FROM readings WHERE thing = ? AND subtype = 'pose' AND ts >= ? " +
      "ORDER BY ts DESC LIMIT ?",
    ).bind(thing, sinceS, SAMPLE_POSE));

    add("lidar", thing, db.prepare(
      "SELECT ts, json_extract(data, '$.sectors_mm') AS sectors_mm, " +
      "json_extract(data, '$.min_mm') AS min_mm, " +
      "json_extract(data, '$.points') AS points, " +
      "json_extract(data, '$.range_max_m') AS range_max_m " +
      "FROM readings WHERE thing = ? AND subtype = 'lidar' AND ts >= ? " +
      "ORDER BY ts DESC LIMIT ?",
    ).bind(thing, sinceS, SAMPLE_LIDAR));

    add("drive", thing, db.prepare(
      "SELECT ts, json_extract(data, '$.battery_v') AS v, " +
      "json_extract(data, '$.battery_stale') AS stale " +
      "FROM readings WHERE thing = ? AND subtype = 'drive' AND ts >= ? " +
      "ORDER BY ts DESC LIMIT ?",
    ).bind(thing, sinceS, SAMPLE_DRIVE));

    add("mission", thing, db.prepare(
      "SELECT ts, json_extract(data, '$.elapsed_s') AS elapsed_s, " +
      "json_extract(data, '$.distance_travelled_mm') AS distance_mm, " +
      "json_extract(data, '$.batt_v') AS batt_v " +
      "FROM readings WHERE thing = ? AND subtype = 'mission' AND ts >= ? " +
      "ORDER BY ts DESC LIMIT ?",
    ).bind(thing, sinceS, SAMPLE_MISSION));
  }

  return { stmts, plan };
}

/* ------------------------------------------------------------- shaping it */

/** day-number (days since epoch, from the SQL CAST) -> UTC midnight in ms. */
function dayToMs(day) {
  const n = Number(day);
  return Number.isFinite(n) ? n * 86400 * 1000 : null;
}

/**
 * Detections per UTC day, per type.
 *
 * The bucket key is a day NUMBER in the SQL and becomes milliseconds here; the
 * client is told `bucket_unit: "day"` so it never has to guess the spacing.
 */
function shapeEvents(rows, sinceS, nowS) {
  const byDay = new Map();
  const totals = {};
  const resolved = {};
  for (const t of DETECTION_TYPES) totals[t] = 0;
  totals.other = 0;

  const bucket = (dayMs) => {
    let b = byDay.get(dayMs);
    if (!b) {
      b = { t: dayMs };
      for (const t of DETECTION_TYPES) b[t] = 0;
      b.other = 0;
      byDay.set(dayMs, b);
    }
    return b;
  };

  // Pre-seed every day in the window so a quiet day is a visible zero-height
  // slot rather than a missing column. This is not fabricated data: the archive
  // genuinely recorded nothing that day, and the chart should say so.
  const firstDay = Math.floor(sinceS / 86400);
  const lastDay = Math.floor(nowS / 86400);
  for (let d = firstDay; d <= lastDay; d++) bucket(d * 86400 * 1000);

  for (const r of rows || []) {
    const dayMs = dayToMs(r.day);
    if (dayMs === null) continue;
    const n = Number(r.n) || 0;
    const sub = String(r.subtype || "");

    if (RESOLVED_TYPES[sub]) {
      const of = RESOLVED_TYPES[sub];
      resolved[of] = (resolved[of] || 0) + n;
      continue;
    }
    // Closed vocabulary. An unrecognised subtype is counted, never named.
    const key = DETECTION_TYPES.includes(sub) ? sub : "other";
    bucket(dayMs)[key] += n;
    totals[key] += n;
  }

  const buckets = [...byDay.values()].sort((a, b) => a.t - b.t);
  const total = Object.values(totals).reduce((s, n) => s + n, 0);
  return { bucket_unit: "day", buckets, totals, resolved, total };
}

/** Analyst check outcomes per UTC day. reports.severity is already closed. */
function shapeReports(rows) {
  const byDay = new Map();
  let checks = 0;
  let nominal = 0;

  for (const r of rows || []) {
    const dayMs = dayToMs(r.day);
    const sev = String(r.severity || "");
    if (dayMs === null || !SEVERITIES.includes(sev)) continue;
    const n = Number(r.n) || 0;

    let b = byDay.get(dayMs);
    if (!b) {
      b = { t: dayMs, ok: 0, warning: 0, critical: 0 };
      byDay.set(dayMs, b);
    }
    b[sev] += n;
    checks += n;
    if (sev === "ok") nominal += n;
  }

  return {
    bucket_unit: "day",
    buckets: [...byDay.values()].sort((a, b) => a.t - b.t),
    checks,
    nominal,
  };
}

/** Uptime heartbeat. uptime_s is a DURATION in seconds — not a timestamp. */
function shapeUptime(rows) {
  const points = [];
  let camOk = 0;
  let lidOk = 0;
  let n = 0;

  // Rows arrive newest-first from the index; charts want oldest-first.
  for (const r of [...(rows || [])].reverse()) {
    const t = toMs(r.ts);                  // readings.ts is SECONDS -> ms
    const up = num(r.uptime_s);            // DURATION, left in seconds
    if (t === null) continue;
    points.push({ t, uptime_s: up });
    n++;
    if (Number(r.camera_ok)) camOk++;
    if (Number(r.lidar_ok)) lidOk++;
  }

  return {
    points,
    samples: n,
    camera_ok_pct: n ? camOk / n : null,
    lidar_ok_pct: n ? lidOk / n : null,
    // Peak uptime in the window: the longest the rover stayed up without a
    // reboot. Reboots show as the sawtooth in the chart.
    peak_uptime_s: points.reduce((m, p) => (p.uptime_s > m ? p.uptime_s : m), 0) || null,
  };
}

/**
 * LiDAR sweep coverage.
 *
 * `sectors_mm` is the rover's own 12 x 30 degree summary of the 360-bin sweep,
 * with null where nothing came back. Coverage is therefore an exact, unarguable
 * count — sectors that returned a range, over twelve — rather than a guess at
 * what a raw point count is counting.
 */
function shapeLidar(rows) {
  const points = [];
  let latestSectors = null;
  let latestRangeMm = null;
  let latestPoints = null;

  const parseSectors = (raw) => {
    if (raw === null || raw === undefined) return null;
    try {
      const arr = typeof raw === "string" ? JSON.parse(raw) : raw;
      if (!Array.isArray(arr)) return null;
      return arr.slice(0, LIDAR_SECTORS).map((v) => num(v));
    } catch {
      return null;
    }
  };

  for (const r of [...(rows || [])].reverse()) {
    const t = toMs(r.ts);                  // SECONDS -> ms
    if (t === null) continue;
    const sectors = parseSectors(r.sectors_mm);
    const hit = sectors ? sectors.filter((v) => v !== null && v > 0).length : null;
    points.push({
      t,
      coverage: hit === null ? null : hit / LIDAR_SECTORS,
      min_mm: num(r.min_mm),
    });
    if (sectors) {
      latestSectors = sectors;
      // range_max_m is METRES on the wire; everything else on this chart is mm.
      const rmax = num(r.range_max_m);
      latestRangeMm = rmax === null ? null : rmax * 1000;
      latestPoints = num(r.points);
    }
  }

  return {
    points,
    samples: points.length,
    latest_sectors_mm: latestSectors,
    latest_range_max_mm: latestRangeMm,
    latest_returns: latestPoints,
    sector_count: LIDAR_SECTORS,
  };
}

/**
 * Pack voltage.
 *
 * Primary source is `telemetry/drive` (battery_v, 1 Hz from the board's
 * /battery topic). Fallback is `telemetry/mission` (batt_v). BOTH ride the hub
 * relay, never the rover's direct uplink — the direct uplink process has no ROS
 * access — so with the laptop off the cloud sees no battery data at all. That
 * is a real property of the system, and the empty state says so rather than
 * drawing a flat line at zero.
 */
function shapeBattery(driveRows, missionRows) {
  const build = (rows, field, source) => {
    const points = [];
    let stale = 0;
    for (const r of [...(rows || [])].reverse()) {
      const t = toMs(r.ts);                // SECONDS -> ms
      const v = num(r[field]);
      if (t === null || v === null) continue;
      points.push({ t, v });
      if (r.stale !== undefined && Number(r.stale)) stale++;
    }
    return { points, samples: points.length, stale_samples: stale, source };
  };

  const drive = build(driveRows, "v", "telemetry/drive");
  if (drive.samples) return { ...drive, low_v: BATT_LOW_V };

  const mission = build(missionRows, "batt_v", "telemetry/mission");
  return { ...mission, low_v: BATT_LOW_V };
}

/**
 * Runs, inferred.
 *
 * Nothing on the rover mints a run id — mission identity is a reused name and
 * `elapsed_s` is monotonic within a run. So a run boundary is either a gap
 * longer than RUN_GAP_S (the relay throttles mission telemetry to 15 s, so a
 * two minute hole means it stopped) or `elapsed_s` going backwards, which only
 * happens when a new mission starts. Runs are NUMBERED, not named: the mission
 * name is a rover-authored string and public.js design rule 1 keeps those off
 * this page.
 */
function shapeRuns(rows) {
  const asc = [...(rows || [])]
    .map((r) => ({
      t: toMs(r.ts),                       // SECONDS -> ms
      elapsed_s: num(r.elapsed_s),         // DURATION, stays seconds
      distance_mm: num(r.distance_mm),
    }))
    .filter((r) => r.t !== null)
    .reverse();

  const runs = [];
  let cur = null;

  for (const r of asc) {
    const gapS = cur ? (r.t - cur.end) / 1000 : Infinity;
    const wentBackwards =
      cur && r.elapsed_s !== null && cur.last_elapsed !== null &&
      r.elapsed_s + 1 < cur.last_elapsed;

    if (!cur || gapS > RUN_GAP_S || wentBackwards) {
      cur = {
        start: r.t, end: r.t, samples: 0,
        // distance starts null, not 0: a run whose telemetry never carried an
        // odometry reading has an UNKNOWN distance, and "0 mm travelled" would
        // be a number nothing measured.
        duration_s: 0, distance_mm: null, last_elapsed: null,
      };
      runs.push(cur);
    }

    cur.end = r.t;
    cur.samples++;
    if (r.elapsed_s !== null) {
      cur.duration_s = Math.max(cur.duration_s, r.elapsed_s);
      cur.last_elapsed = r.elapsed_s;
    }
    if (r.distance_mm !== null) {
      cur.distance_mm = cur.distance_mm === null
        ? r.distance_mm
        : Math.max(cur.distance_mm, r.distance_mm);
    }
  }

  return runs
    .map((r) => ({
      start: r.start,
      end: r.end,
      // elapsed_s is authoritative when present; wall clock is the fallback.
      duration_s: Math.round(r.duration_s || (r.end - r.start) / 1000),
      distance_mm: r.distance_mm === null ? null : Math.round(r.distance_mm),
      samples: r.samples,
    }))
    .filter((r) => r.samples > 1)
    .slice(-MAX_RUNS)
    .map((r, i) => ({ index: i + 1, ...r }));
}

/* ------------------------------------------------------- the live endpoint */

async function liveAnalytics(env, url, hubStub) {
  const nowMs = Date.now();
  const nowS = nowMs / 1000;

  const days = clamp(
    Math.round(Number(url.searchParams.get("days")) || DEFAULT_DAYS),
    1, MAX_DAYS,
  );
  // readings.ts / reports.ts are unix SECONDS, so the floor is seconds too.
  const sinceS = nowS - days * 86400;

  const empty = {
    generated_at: nowMs,
    simulated: false,
    source: "live-archive",
    window_days: days,
    window_from: sinceS * 1000,
    events: { bucket_unit: "day", buckets: [], totals: {}, resolved: {}, total: 0 },
    health: { bucket_unit: "day", buckets: [], checks: 0, nominal: 0 },
    rovers: [],
    arena_mm: ARENA_MM,
    notes: [],
  };

  if (!env.DB) return analyticsJson({ ...empty, notes: ["archive_unavailable"] });

  const things = await analyticsRoster(env, hubStub);
  const { stmts, plan } = analyticsStatements(env, things, sinceS);

  let results;
  try {
    // One batch, one round trip. Everything in it is index-driven and bounded.
    results = await env.DB.batch(stmts);
  } catch (err) {
    logError("analytics query failed", err);
    return analyticsJson({ ...empty, notes: ["archive_query_failed"] });
  }

  const pick = (kind, thing) => {
    const i = plan.findIndex((p) => p.kind === kind && p.thing === thing);
    return i < 0 ? [] : (results[i]?.results || []);
  };

  const events = shapeEvents(pick("events", null), sinceS, nowS);
  const health = shapeReports(pick("reports", null));

  const rovers = things.map(({ thing, last_seen }) => {
    const missionRows = pick("mission", thing);
    return {
      thing,
      last_seen,
      uptime: shapeUptime(pick("pose", thing)),
      lidar: shapeLidar(pick("lidar", thing)),
      battery: shapeBattery(pick("drive", thing), missionRows),
      runs: shapeRuns(missionRows),
    };
  });

  // Machine-readable reasons a panel is empty, so the page can explain itself
  // precisely instead of showing a generic "no data".
  const notes = [];
  if (!things.length) notes.push("no_rovers_known");
  if (!events.total) notes.push("no_detections_in_window");
  if (!health.checks) notes.push("no_analyst_reports");
  if (rovers.length && rovers.every((r) => !r.battery.samples)) notes.push("no_battery_telemetry");
  if (rovers.length && rovers.every((r) => !r.lidar.samples)) notes.push("no_lidar_in_window");
  if (rovers.length && rovers.every((r) => !r.runs.length)) notes.push("no_runs_in_window");

  const dataPoints = rovers.reduce(
    (n, r) => n + r.uptime.samples + r.lidar.samples + r.battery.samples,
    0,
  ) + events.total + health.checks;

  return analyticsJson({
    ...empty,
    events,
    health,
    rovers,
    // The page uses this to decide whether the archive is worth showing at all,
    // so it can offer the demo without needing the live summary to be wired.
    data_points: dataPoints,
    notes,
  });
}

/* --------------------------------------------------------- the demo replay */

/**
 * Fixed anchor for the scripted run: 2026-05-16 14:20 UTC.
 *
 * Deliberately NOT Date.now(). A demo timeline that tracks the clock reads as
 * "this just happened", which is the exact impression this dataset must never
 * give. Pinning it to a constant past date means every timestamp on screen is
 * visibly historical and identical for every visitor.
 */
const DEMO_ANCHOR_MS = Date.UTC(2026, 4, 16, 14, 20, 0);

/** Scenario length, and how the scripted samples are spaced. */
const DEMO_DURATION_S = 360;
const DEMO_STEP_S = 5;        // heartbeat + LiDAR cadence
const DEMO_SLOW_STEP_S = 15;  // relayed drive/mission cadence

/**
 * The scripted event script. Hand-authored, reviewable, and the sole source of
 * the demo's detections — there is no randomness in what happens, only in the
 * measurement wobble around it.
 */
const DEMO_EVENTS = [
  { at: 40, type: "wildlife" },
  { at: 65, type: "obstacle" },
  { at: 95, type: "wildlife" },
  { at: 150, type: "fire" },
  { at: 155, type: "obstacle" },
  { at: 205, type: "fire" },
  { at: 215, type: "fire_cleared" },
  { at: 250, type: "obstacle" },
  { at: 275, type: "fault" },
  { at: 300, type: "wildlife" },
  { at: 330, type: "obstacle" },
];

/** Scripted analyst checks, one per minute of the run. */
const DEMO_REPORTS = [
  { at: 30, severity: "ok" },
  { at: 90, severity: "ok" },
  { at: 150, severity: "critical" },
  { at: 210, severity: "warning" },
  { at: 270, severity: "ok" },
  { at: 330, severity: "ok" },
];

/**
 * Deterministic wobble in [0,1). Not Math.random: the demo must be byte-identical
 * on every request so it can be cached for a day and so two people looking at it
 * are looking at the same thing.
 */
function demoWobble(i, k) {
  const x = Math.sin(i * 12.9898 + k * 78.233) * 43758.5453;
  return x - Math.floor(x);
}

/**
 * Build the scripted run.
 *
 * Every value is shaped by a real system constant so the demonstration is
 * physically representative rather than arbitrary: the arena is ARENA_MM on a
 * side, the sweep is LIDAR_SECTORS sectors, the pack is a 3S pack whose low
 * threshold is BATT_LOW_V. It is still a simulation, and the response says so
 * three times over.
 */
function buildDemoRun() {
  const t0 = DEMO_ANCHOR_MS;
  const at = (s) => t0 + s * 1000;

  /* --- detections, bucketed by minute rather than by day ---------------- */
  const bucketS = 60;
  const nBuckets = Math.ceil(DEMO_DURATION_S / bucketS);
  const buckets = [];
  for (let i = 0; i < nBuckets; i++) {
    const b = { t: at(i * bucketS) };
    for (const type of DETECTION_TYPES) b[type] = 0;
    b.other = 0;
    buckets.push(b);
  }

  const totals = {};
  for (const type of DETECTION_TYPES) totals[type] = 0;
  totals.other = 0;
  const resolved = {};
  const timeline = [];

  for (const e of DEMO_EVENTS) {
    if (RESOLVED_TYPES[e.type]) {
      const of = RESOLVED_TYPES[e.type];
      resolved[of] = (resolved[of] || 0) + 1;
      timeline.push({ t: at(e.at), type: e.type, resolves: of });
      continue;
    }
    buckets[Math.min(nBuckets - 1, Math.floor(e.at / bucketS))][e.type]++;
    totals[e.type]++;
    timeline.push({ t: at(e.at), type: e.type });
  }

  /* --- analyst checks --------------------------------------------------- */
  const healthBuckets = [];
  for (let i = 0; i < nBuckets; i++) {
    healthBuckets.push({ t: at(i * bucketS), ok: 0, warning: 0, critical: 0 });
  }
  let checks = 0;
  let nominal = 0;
  for (const r of DEMO_REPORTS) {
    healthBuckets[Math.min(nBuckets - 1, Math.floor(r.at / bucketS))][r.severity]++;
    checks++;
    if (r.severity === "ok") nominal++;
  }

  /* --- heartbeat: uptime climbs from a cold boot ------------------------ */
  const uptimePoints = [];
  const bootOffsetS = 12;
  for (let s = 0, i = 0; s <= DEMO_DURATION_S; s += DEMO_STEP_S, i++) {
    uptimePoints.push({ t: at(s), uptime_s: bootOffsetS + s });
  }

  /* --- LiDAR: sectors fill in as the sweep settles, then a wall closes in */
  const lidarPoints = [];
  let latestSectors = null;
  for (let s = 0, i = 0; s <= DEMO_DURATION_S; s += DEMO_STEP_S, i++) {
    // Sectors start partially blind (the scanner is still spinning up) and are
    // fully populated within the first half minute.
    const blind = s < 10 ? 4 : s < 20 ? 2 : s < 30 ? 1 : 0;
    const sectors = [];
    for (let k = 0; k < LIDAR_SECTORS; k++) {
      if (k < blind) { sectors.push(null); continue; }
      // Rover roughly mid-arena: walls sit a little under half the arena away.
      const base = ARENA_MM * 0.42 + Math.sin((k / LIDAR_SECTORS) * Math.PI * 2 + s / 40) * 120;
      // The scripted obstacle approach: sector 3 closes to ~180 mm around t+65
      // and again at t+155, which is what raises the obstacle events above.
      const nearWindow = (s > 55 && s < 80) || (s > 145 && s < 175);
      const v = nearWindow && k === 3 ? 180 + demoWobble(i, k) * 25
        : base + demoWobble(i, k) * 40;
      sectors.push(Math.round(clamp(v, 90, 6000)));
    }
    const hit = sectors.filter((v) => v !== null).length;
    const minMm = Math.round(Math.min(...sectors.filter((v) => v !== null)));
    lidarPoints.push({ t: at(s), coverage: hit / LIDAR_SECTORS, min_mm: minMm });
    latestSectors = sectors;
  }

  /* --- pack voltage: a healthy 3S pack draining over six minutes -------- */
  const battPoints = [];
  for (let s = 0, i = 0; s <= DEMO_DURATION_S; s += DEMO_SLOW_STEP_S, i++) {
    // 12.42 V down to about 11.6 V, with a sag while the drive motors load up
    // during the mid-run legs. Never crosses BATT_LOW_V — this is a good run.
    const drift = 12.42 - (s / DEMO_DURATION_S) * 0.78;
    const sag = s > 120 && s < 240 ? -0.09 : 0;
    battPoints.push({
      t: at(s),
      v: Math.round((drift + sag + (demoWobble(i, 7) - 0.5) * 0.05) * 100) / 100,
    });
  }

  /* --- one run, four segments ------------------------------------------ */
  const run = {
    index: 1,
    start: t0,
    end: at(DEMO_DURATION_S),
    duration_s: DEMO_DURATION_S,
    distance_mm: 3240,
    samples: Math.floor(DEMO_DURATION_S / DEMO_SLOW_STEP_S) + 1,
  };

  return {
    generated_at: Date.now(),
    simulated: true,
    source: "simulated",
    disclaimer:
      "SIMULATED — this is a scripted demonstration run, not recorded rover " +
      "telemetry and not live data. It exists so the page still shows how the " +
      "system behaves while the rover is offline.",
    scenario: {
      title: "Scripted demonstration run",
      anchor: t0,
      duration_s: DEMO_DURATION_S,
      arena_mm: ARENA_MM,
      note:
        "Values are generated from a fixed script using the system's own " +
        "constants (1200 mm arena, 12 x 30 degree LiDAR sectors, " +
        BATT_LOW_V + " V pack low threshold). No measurement in it is real.",
    },
    window_days: null,
    events: { bucket_unit: "minute", origin: t0, buckets, totals, resolved, total: Object.values(totals).reduce((s, n) => s + n, 0) },
    health: { bucket_unit: "minute", origin: t0, buckets: healthBuckets, checks, nominal },
    timeline,
    rovers: [{
      thing: "demo-rover",
      last_seen: at(DEMO_DURATION_S),
      uptime: {
        points: uptimePoints,
        samples: uptimePoints.length,
        camera_ok_pct: 1,
        lidar_ok_pct: 0.96,
        peak_uptime_s: bootOffsetS + DEMO_DURATION_S,
      },
      lidar: {
        points: lidarPoints,
        samples: lidarPoints.length,
        latest_sectors_mm: latestSectors,
        latest_range_max_mm: 6000,
        latest_returns: LIDAR_SECTORS,
        sector_count: LIDAR_SECTORS,
      },
      battery: {
        points: battPoints,
        samples: battPoints.length,
        stale_samples: 0,
        source: "simulated",
        low_v: BATT_LOW_V,
      },
      runs: [run],
    }],
    arena_mm: ARENA_MM,
    notes: ["simulated_dataset"],
  };
}

/* ------------------------------------------------------------- the router */

/**
 * Public analytics routes. Returns a Response for a path it owns, or null so
 * public.js carries on to its own dispatch.
 *
 * Mount AFTER the burst guard in handlePublic so these inherit it — see (W2).
 */
export async function handleAnalytics(request, env, url, path, hubStub) {
  if (!path.startsWith("/api/public/")) return null;
  if (request.method !== "GET") return null;

  if (path === "/api/public/analytics") {
    try {
      return await liveAnalytics(env, url, hubStub);
    } catch (err) {
      // Never let the analytics section take the status page down with it.
      logError("analytics handler failed", err);
      return analyticsJson({
        generated_at: Date.now(),
        simulated: false,
        source: "live-archive",
        events: { bucket_unit: "day", buckets: [], totals: {}, resolved: {}, total: 0 },
        health: { bucket_unit: "day", buckets: [], checks: 0, nominal: 0 },
        rovers: [],
        data_points: 0,
        notes: ["archive_unavailable"],
      });
    }
  }

  if (path === "/api/public/demo") {
    // Constant output: no D1, no Durable Object, no per-request work worth
    // caching short. A day is fine.
    return analyticsJson(buildDemoRun(), 200, DEMO_CACHE_S);
  }

  return null;
}

/* ===========================================================================
 *                            THE PAGE FRAGMENTS
 * ===========================================================================
 *
 * Three strings, interpolated into PUBLIC_PAGE by the main agent — see (W3),
 * (W4), (W5) at the top of this file. They are written to sit alongside the
 * page's existing variables (--bg, --panel, --line, --fg, --dim) and to add
 * only names prefixed `--fx-` / `fx-`, so nothing collides.
 */

/**
 * Palette note.
 *
 * The categorical slots below are the validated default eight, in their fixed
 * order, stepped for each mode. They were re-run against THIS page's actual
 * surfaces rather than the palette's defaults, because contrast is only
 * meaningful against the surface a chart really renders on:
 *
 *   light, surface #ffffff : all checks pass. Worst adjacent CVD dE 9.1
 *                            (yellow/aqua, protan), worst normal-vision dE 22.9.
 *                            Aqua (2.82:1) and yellow (2.17:1) sit under 3:1 on
 *                            white, so the relief rule applies — hence the
 *                            always-present legend, the direct labels, and the
 *                            table view under every chart.
 *   dark,  surface #141b24 : all checks pass, including contrast.
 *
 * Series-to-slot assignment is fixed by DETECTION_TYPES and never reassigned by
 * rank, so filtering or an empty day cannot repaint a series a reader has
 * already learned.
 */
export const ANALYTICS_STYLE = `
  /* ---------- analytics: tokens ---------- */
  #fx {
    /* dark is the page default, matching the existing :root above */
    --fx-surface:#141b24; --fx-plane:#0b0f14;
    --fx-ink:#e6edf3; --fx-ink-2:#b3c0cd; --fx-muted:#93a1b0;
    --fx-grid:#232d3a; --fx-axis:#33404f; --fx-hairline:rgba(255,255,255,.10);
    --fx-s1:#3987e5; --fx-s2:#d95926; --fx-s3:#199e70; --fx-s4:#c98500;
    --fx-good:#0ca30c; --fx-warn:#fab219; --fx-serious:#ec835a; --fx-crit:#d03b3b;
    --fx-sim:#ec835a; --fx-sim-wash:rgba(236,131,90,.10);
  }
  @media (prefers-color-scheme: light) {
    :root:where(:not([data-theme="dark"])) #fx {
      --fx-surface:#ffffff; --fx-plane:#f6f8fa;
      --fx-ink:#1f2328; --fx-ink-2:#4a5158; --fx-muted:#6b747d;
      --fx-grid:#e4e8ec; --fx-axis:#c9d0d7; --fx-hairline:rgba(31,35,40,.12);
      --fx-s1:#2a78d6; --fx-s2:#eb6834; --fx-s3:#1baf7a; --fx-s4:#eda100;
      --fx-sim:#a34a17; --fx-sim-wash:rgba(235,104,52,.10);
    }
  }
  :root[data-theme="light"] #fx {
    --fx-surface:#ffffff; --fx-plane:#f6f8fa;
    --fx-ink:#1f2328; --fx-ink-2:#4a5158; --fx-muted:#6b747d;
    --fx-grid:#e4e8ec; --fx-axis:#c9d0d7; --fx-hairline:rgba(31,35,40,.12);
    --fx-s1:#2a78d6; --fx-s2:#eb6834; --fx-s3:#1baf7a; --fx-s4:#eda100;
    --fx-sim:#a34a17; --fx-sim-wash:rgba(235,104,52,.10);
  }

  /* ---------- analytics: layout (mobile first) ---------- */
  #fx { margin-top:34px; }
  .fx-sr { position:absolute; width:1px; height:1px; padding:0; margin:-1px;
           overflow:hidden; clip:rect(0 0 0 0); white-space:nowrap; border:0; }

  .fx-bar { display:flex; flex-wrap:wrap; gap:8px; align-items:center; margin-bottom:14px; }
  .fx-btn {
    font:inherit; font-size:13px; font-weight:600; color:var(--fx-ink);
    background:var(--fx-surface); border:1px solid var(--fx-axis); border-radius:8px;
    padding:0 14px; min-height:44px; cursor:pointer; display:inline-flex;
    align-items:center; gap:8px;
  }
  .fx-btn:hover { border-color:var(--fx-s1); }
  .fx-btn:focus-visible { outline:2px solid var(--fx-s1); outline-offset:2px; }
  .fx-btn[aria-pressed="true"] { border-color:var(--fx-s1); box-shadow:inset 0 0 0 1px var(--fx-s1); }
  .fx-btn.fx-btn-sim[aria-pressed="true"] { border-color:var(--fx-sim); box-shadow:inset 0 0 0 1px var(--fx-sim); }
  .fx-bar-note { font-size:12px; color:var(--fx-muted); }

  /* ---------- the SIMULATED treatment ---------- */
  .fx-simbar {
    display:flex; gap:12px; align-items:flex-start;
    border:1px solid var(--fx-sim); border-left-width:5px; border-radius:8px;
    background:var(--fx-sim-wash); padding:12px 14px; margin:0 0 16px;
  }
  .fx-simbar strong { display:block; font-size:14px; color:var(--fx-ink); }
  .fx-simbar p { margin:4px 0 0; font-size:13px; color:var(--fx-ink-2); }
  .fx-badge {
    flex:none; font-size:11px; font-weight:700; letter-spacing:.08em;
    text-transform:uppercase; color:var(--fx-ink);
    border:1px solid var(--fx-sim); border-radius:4px; padding:3px 7px;
    background:var(--fx-sim-wash); white-space:nowrap;
  }
  .fx-tag {
    display:inline-block; font-size:10px; font-weight:700; letter-spacing:.08em;
    text-transform:uppercase; color:var(--fx-ink); background:var(--fx-sim-wash);
    border:1px solid var(--fx-sim); border-radius:3px; padding:1px 5px;
    margin-right:6px; vertical-align:1px;
  }
  #fx[data-mode="live"] .fx-tag,
  #fx[data-mode="live"] .fx-simbar { display:none; }
  /* In demo mode every card wears a 45 degree hatch and an amber rail, so a
     screenshot of ONE card still carries the label. */
  #fx[data-mode="demo"] .fx-card {
    border-color:var(--fx-sim); border-left:4px solid var(--fx-sim);
    background-image:repeating-linear-gradient(45deg,
      var(--fx-sim-wash) 0 6px, transparent 6px 16px);
  }
  #fx[data-mode="demo"] .fx-plot { background:var(--fx-surface); border-radius:6px; }

  /* ---------- cards & charts ---------- */
  .fx-grid { display:grid; grid-template-columns:1fr; gap:12px; }
  @media (min-width:760px) { .fx-grid { grid-template-columns:1fr 1fr; } }
  .fx-card {
    background:var(--fx-surface); border:1px solid var(--fx-axis);
    border-radius:8px; padding:14px; margin:0; min-width:0;
  }
  .fx-card.fx-wide { grid-column:1 / -1; }
  .fx-card h4 { margin:0; font-size:14px; font-weight:600; color:var(--fx-ink); }
  .fx-card .fx-sub { margin:2px 0 10px; font-size:12px; color:var(--fx-muted); }
  /* Charts scroll inside their own box. The page itself never scrolls sideways. */
  .fx-scroll { overflow-x:auto; overflow-y:hidden; -webkit-overflow-scrolling:touch; }
  .fx-plot { display:block; }
  .fx-plot:focus-visible { outline:2px solid var(--fx-s1); outline-offset:2px; }
  .fx-plot .fx-hit:focus-visible { outline:2px solid var(--fx-s1); }

  .fx-legend { list-style:none; display:flex; flex-wrap:wrap; gap:6px 14px;
               margin:10px 0 0; padding:0; font-size:12px; color:var(--fx-ink-2); }
  .fx-legend li { display:flex; align-items:center; gap:6px; }
  .fx-key { width:10px; height:10px; border-radius:3px; flex:none; }
  .fx-key-line { width:14px; height:3px; border-radius:2px; flex:none; }
  .fx-key-none { width:10px; height:10px; border-radius:3px; flex:none;
                 border:1px solid var(--fx-axis); background:transparent; }

  .fx-kpis { display:grid; grid-template-columns:repeat(2,1fr); gap:10px; margin-bottom:12px; }
  @media (min-width:620px) { .fx-kpis { grid-template-columns:repeat(4,1fr); } }
  .fx-kpi { background:var(--fx-surface); border:1px solid var(--fx-axis);
            border-radius:8px; padding:12px 13px; }
  .fx-kpi .fx-k-label { font-size:11px; color:var(--fx-muted); text-transform:uppercase;
                        letter-spacing:.05em; font-weight:600; }
  .fx-kpi .fx-k-value { font-size:26px; font-weight:600; color:var(--fx-ink);
                        line-height:1.15; margin-top:4px; }
  .fx-kpi .fx-k-note { font-size:12px; color:var(--fx-ink-2); margin-top:2px; }
  #fx[data-mode="demo"] .fx-kpi { border-color:var(--fx-sim); }

  .fx-meter { height:8px; border-radius:99px; background:var(--fx-grid);
              margin-top:8px; overflow:hidden; }
  .fx-meter > i { display:block; height:100%; background:var(--fx-s1); border-radius:99px; }

  details.fx-table { margin-top:10px; }
  details.fx-table > summary {
    font-size:12px; color:var(--fx-ink-2); cursor:pointer; min-height:24px;
    display:flex; align-items:center; padding:2px 0;
  }
  details.fx-table > summary:focus-visible { outline:2px solid var(--fx-s1); outline-offset:2px; }
  details.fx-table table { width:100%; border-collapse:collapse; font-size:12px;
                           margin-top:6px; font-variant-numeric:tabular-nums; }
  details.fx-table th, details.fx-table td {
    text-align:left; padding:6px 8px; border-bottom:1px solid var(--fx-grid);
    white-space:nowrap;
  }
  details.fx-table th { color:var(--fx-muted); font-weight:600; font-size:11px;
                        text-transform:uppercase; letter-spacing:.04em; }

  .fx-empty { border:1px dashed var(--fx-axis); border-radius:8px; padding:18px 14px;
              text-align:center; color:var(--fx-ink-2); font-size:13px; }
  .fx-empty b { display:block; color:var(--fx-ink); font-size:14px; margin-bottom:4px;
                font-weight:600; }

  .fx-replay { display:flex; align-items:center; gap:10px; flex-wrap:wrap;
               margin-bottom:14px; }
  .fx-replay input[type=range] { flex:1 1 180px; min-width:140px; accent-color:var(--fx-sim); height:24px; }
  .fx-replay .fx-clock { font-size:13px; color:var(--fx-ink-2);
                         font-variant-numeric:tabular-nums; min-width:96px; }

  .fx-tip {
    position:fixed; z-index:50; pointer-events:none; opacity:0;
    transition:opacity .08s linear; max-width:240px;
    background:var(--fx-surface); color:var(--fx-ink); border:1px solid var(--fx-axis);
    border-radius:6px; padding:8px 10px; font-size:12px; line-height:1.45;
    box-shadow:0 4px 14px rgba(0,0,0,.28);
  }
  .fx-tip[data-show="1"] { opacity:1; }
  .fx-tip .fx-tip-h { font-weight:600; margin-bottom:4px; }
  .fx-tip .fx-tip-r { display:flex; align-items:center; gap:6px; }
  .fx-tip .fx-tip-v { margin-left:auto; font-variant-numeric:tabular-nums; }

  @media (prefers-reduced-motion: reduce) {
    .fx-tip { transition:none; }
  }
  @media print {
    #fx[data-mode="demo"] .fx-card { border-left-width:6px; }
  }
`;

/** The section markup. Semantic, and empty until the script fills it. */
export const ANALYTICS_HTML = `
  <section id="fx" data-mode="live" aria-labelledby="fx-h">
    <h3 id="fx-h">Analytics</h3>

    <div class="fx-bar" role="group" aria-label="Analytics data source">
      <button type="button" class="fx-btn" id="fx-live" aria-pressed="true">Live archive</button>
      <button type="button" class="fx-btn fx-btn-sim" id="fx-demo" aria-pressed="false">
        Play simulated run
      </button>
      <span class="fx-bar-note" id="fx-source">Loading&hellip;</span>
    </div>

    <div id="fx-announce" class="fx-sr" role="status" aria-live="polite"></div>

    <div class="fx-simbar" id="fx-simbar" hidden>
      <span class="fx-badge">Simulated</span>
      <div>
        <strong>This is a scripted demonstration, not live or recorded data.</strong>
        <p id="fx-simnote">
          The rover is offline, so the charts below are replaying a scripted run
          built from the system's own constants. No measurement in it is real.
        </p>
      </div>
    </div>

    <div class="fx-replay" id="fx-replay" hidden>
      <button type="button" class="fx-btn" id="fx-play" aria-label="Pause the simulated replay">Pause</button>
      <input type="range" id="fx-scrub" min="0" max="360" value="0" step="1"
             aria-label="Replay position, seconds into the simulated run">
      <span class="fx-clock" id="fx-clock">0:00 / 6:00</span>
    </div>

    <div class="fx-kpis" id="fx-kpis"></div>
    <div class="fx-grid" id="fx-charts"></div>

    <p class="sub" id="fx-prov" style="margin-top:12px;font-size:12px"></p>
  </section>
`;

/**
 * The page script.
 *
 * Written with string concatenation rather than template literals so it can be
 * interpolated into public.js's own template literal without escaping, and it
 * deliberately contains no closing-script-tag sequence.
 *
 * Everything is inside one IIFE and touches the DOM only under #fx, so it
 * cannot collide with the existing status-page script. The only global it
 * defines is window.FPMS_ANALYTICS, which exists purely as the optional (W6)
 * hook.
 */
export const ANALYTICS_SCRIPT = `<script>
(function () {
  "use strict";

  var SVGNS = "http://www.w3.org/2000/svg";
  var root = document.getElementById("fx");
  if (!root) return;

  var el = function (id) { return document.getElementById(id); };
  var TYPES = ["wildlife", "fire", "obstacle", "fault"];
  /* Fixed slot assignment. Colour follows the entity, never its rank. */
  var TYPE_VAR = { wildlife: "--fx-s1", fire: "--fx-s2", obstacle: "--fx-s3", fault: "--fx-s4" };
  var TYPE_LABEL = {
    wildlife: "Wildlife", fire: "Fire", obstacle: "Obstacle",
    fault: "Fault", other: "Other"
  };
  var state = {
    mode: "live",
    live: null,
    demo: null,
    playing: false,
    head: 0,          /* seconds into the scripted run */
    timer: null,
    liveOnline: null  /* set by the optional onSummary hook */
  };

  /* ------------------------------------------------------------ helpers */

  function css(name) {
    return getComputedStyle(root).getPropertyValue(name).trim() || "#888";
  }
  function mk(tag, attrs, text) {
    var n = document.createElementNS(SVGNS, tag), k;
    if (attrs) for (k in attrs) if (attrs[k] !== null && attrs[k] !== undefined) {
      n.setAttribute(k, String(attrs[k]));
    }
    if (text !== undefined && text !== null) n.textContent = String(text);
    return n;
  }
  function h(tag, cls, text) {
    var n = document.createElement(tag);
    if (cls) n.className = cls;
    if (text !== undefined && text !== null) n.textContent = String(text);
    return n;
  }
  function fmtInt(n) {
    if (n === null || n === undefined || !isFinite(n)) return "-";
    return Math.round(n).toLocaleString();
  }
  function fmtDur(s) {
    if (s === null || s === undefined || !isFinite(s)) return "-";
    s = Math.max(0, Math.round(s));
    var d = Math.floor(s / 86400), hh = Math.floor((s % 86400) / 3600);
    var mm = Math.floor((s % 3600) / 60), ss = s % 60;
    if (d) return d + "d " + hh + "h";
    if (hh) return hh + "h " + mm + "m";
    if (mm) return mm + "m " + (ss < 10 ? "0" : "") + ss + "s";
    return ss + "s";
  }
  function mmss(s) {
    s = Math.max(0, Math.round(s));
    var m = Math.floor(s / 60), r = s % 60;
    return m + ":" + (r < 10 ? "0" : "") + r;
  }
  function fmtDist(mm) {
    if (mm === null || mm === undefined || !isFinite(mm)) return "-";
    return mm >= 1000 ? (mm / 1000).toFixed(2) + " m" : Math.round(mm) + " mm";
  }
  function fmtDate(ms) {
    if (!ms) return "-";
    try { return new Date(ms).toLocaleString(); } catch (e) { return String(ms); }
  }
  /* Bucket labels. Day buckets are UTC midnights (the SQL bucketed on UTC);
     minute buckets are offsets from the scripted run's start. */
  function bucketLabel(t, unit, origin) {
    if (unit === "minute") return "+" + mmss((t - origin) / 1000);
    try {
      return new Date(t).toLocaleDateString(undefined, {
        month: "short", day: "numeric", timeZone: "UTC"
      });
    } catch (e) { return String(t); }
  }

  /* ---------------------------------------------------------- tooltip */

  var tip = h("div", "fx-tip");
  tip.setAttribute("role", "presentation");
  document.body.appendChild(tip);

  function showTip(x, y, title, rows) {
    tip.textContent = "";
    var head = h("div", "fx-tip-h", title);
    tip.appendChild(head);
    for (var i = 0; i < rows.length; i++) {
      var r = h("div", "fx-tip-r");
      if (rows[i].color) {
        var sw = h("span", "fx-key");
        sw.style.background = rows[i].color;
        r.appendChild(sw);
      }
      r.appendChild(h("span", null, rows[i].label));
      r.appendChild(h("span", "fx-tip-v", rows[i].value));
      tip.appendChild(r);
    }
    tip.setAttribute("data-show", "1");
    var w = tip.offsetWidth, hh = tip.offsetHeight;
    var left = Math.min(Math.max(8, x + 14), window.innerWidth - w - 8);
    var top = y - hh - 12;
    if (top < 8) top = y + 18;
    tip.style.left = left + "px";
    tip.style.top = top + "px";
  }
  function hideTip() { tip.setAttribute("data-show", "0"); }
  document.addEventListener("scroll", hideTip, true);

  /* ------------------------------------------------------ chart chrome */

  /* A card is a <figure>; the caption is its heading, so the chart is
     announced with a name rather than as an anonymous graphic. */
  function card(title, subtitle, wide) {
    var fig = h("figure", "fx-card" + (wide ? " fx-wide" : ""));
    var cap = h("figcaption");
    var head = h("h4");
    var tag = h("span", "fx-tag", "Sim");
    head.appendChild(tag);
    head.appendChild(document.createTextNode(title));
    cap.appendChild(head);
    if (subtitle) cap.appendChild(h("p", "fx-sub", subtitle));
    fig.appendChild(cap);
    return fig;
  }

  function svgFrame(fig, w, hgt, title, desc) {
    var box = h("div", "fx-scroll");
    var svg = mk("svg", {
      "class": "fx-plot", viewBox: "0 0 " + w + " " + hgt,
      width: w, height: hgt, role: "img", tabindex: "0"
    });
    /* The prefix is what a screen reader reads first — the "simulated" label
       reaches assistive tech, not only the eye. */
    var prefix = state.mode === "demo" ? "Simulated: " : "";
    svg.appendChild(mk("title", null, prefix + title));
    if (desc) svg.appendChild(mk("desc", null, prefix + desc));
    box.appendChild(svg);
    fig.appendChild(box);
    if (state.mode === "demo") watermark(svg, w, hgt);
    return svg;
  }

  /* A rotated SIMULATED watermark inside every plot, so a cropped screenshot
     of the chart alone still carries the label. */
  function watermark(svg, w, hgt) {
    var t = mk("text", {
      x: w / 2, y: hgt / 2, "text-anchor": "middle",
      transform: "rotate(-18 " + (w / 2) + " " + (hgt / 2) + ")",
      "font-size": Math.max(20, Math.min(46, w / 11)),
      "font-weight": "700", "letter-spacing": "0.18em",
      fill: css("--fx-sim"), "fill-opacity": ".16",
      "aria-hidden": "true", "pointer-events": "none"
    }, "SIMULATED");
    svg.appendChild(t);
  }

  function legend(fig, items) {
    var ul = h("ul", "fx-legend");
    for (var i = 0; i < items.length; i++) {
      var li = h("li");
      var k = h("span", items[i].line ? "fx-key-line" : (items[i].none ? "fx-key-none" : "fx-key"));
      if (!items[i].none) k.style.background = items[i].color;
      li.appendChild(k);
      li.appendChild(document.createTextNode(items[i].label));
      ul.appendChild(li);
    }
    fig.appendChild(ul);
  }

  /* Every chart ships a table twin. Two reasons: the light-mode aqua and
     yellow slots sit under 3:1 on white, which obliges the relief rule, and a
     tooltip must never be the only way to read a value. */
  function tableView(fig, cols, rows, caption) {
    if (!rows.length) return;
    var d = h("details", "fx-table");
    d.appendChild(h("summary", null, "Table view (" + rows.length + " rows)"));
    var t = h("table");
    var capEl = h("caption", "fx-sr", caption);
    t.appendChild(capEl);
    var thead = h("thead"), tr = h("tr"), i;
    for (i = 0; i < cols.length; i++) {
      var th = h("th", null, cols[i]);
      th.setAttribute("scope", "col");
      tr.appendChild(th);
    }
    thead.appendChild(tr);
    t.appendChild(thead);
    var tb = h("tbody");
    for (i = 0; i < rows.length; i++) {
      var r = h("tr");
      for (var j = 0; j < rows[i].length; j++) r.appendChild(h("td", null, rows[i][j]));
      tb.appendChild(r);
    }
    t.appendChild(tb);
    d.appendChild(t);
    fig.appendChild(d);
  }

  function emptyCard(title, headline, why) {
    var fig = card(title, null, false);
    var box = h("div", "fx-empty");
    box.appendChild(h("b", null, headline));
    box.appendChild(document.createTextNode(why));
    fig.appendChild(box);
    return fig;
  }

  /* ---- shared axis furniture -------------------------------------- */

  function niceTicks(max, count) {
    if (!(max > 0)) return [0, 1];
    var raw = max / count;
    var mag = Math.pow(10, Math.floor(Math.log10(raw)));
    var norm = raw / mag;
    var step = (norm <= 1 ? 1 : norm <= 2 ? 2 : norm <= 5 ? 5 : 10) * mag;
    var out = [], v = 0;
    while (v <= max + step * 0.001) { out.push(v); v += step; }
    if (out.length < 2) out.push(step);
    return out;
  }

  function gridY(svg, pad, plotW, plotH, ticks, scaleY, fmt) {
    for (var i = 0; i < ticks.length; i++) {
      var y = scaleY(ticks[i]);
      /* Solid hairlines, one step off the surface. Never dashed. */
      svg.appendChild(mk("line", {
        x1: pad.l, y1: y, x2: pad.l + plotW, y2: y,
        stroke: css("--fx-grid"), "stroke-width": 1, "shape-rendering": "crispEdges"
      }));
      svg.appendChild(mk("text", {
        x: pad.l - 8, y: y + 4, "text-anchor": "end", "font-size": 11,
        fill: css("--fx-muted"), "font-variant-numeric": "tabular-nums"
      }, fmt ? fmt(ticks[i]) : fmtInt(ticks[i])));
    }
  }

  function baseline(svg, pad, plotW, plotH) {
    svg.appendChild(mk("line", {
      x1: pad.l, y1: pad.t + plotH, x2: pad.l + plotW, y2: pad.t + plotH,
      stroke: css("--fx-axis"), "stroke-width": 1, "shape-rendering": "crispEdges"
    }));
  }

  function topRounded(x, y, w, hh, r) {
    r = Math.min(r, w / 2, hh);
    return "M" + x + "," + (y + hh) + " L" + x + "," + (y + r) +
      " Q" + x + "," + y + " " + (x + r) + "," + y +
      " L" + (x + w - r) + "," + y +
      " Q" + (x + w) + "," + y + " " + (x + w) + "," + (y + r) +
      " L" + (x + w) + "," + (y + hh) + " Z";
  }
  function rightRounded(x, y, w, hh, r) {
    r = Math.min(r, hh / 2, w);
    return "M" + x + "," + y + " L" + (x + w - r) + "," + y +
      " Q" + (x + w) + "," + y + " " + (x + w) + "," + (y + r) +
      " L" + (x + w) + "," + (y + hh - r) +
      " Q" + (x + w) + "," + (y + hh) + " " + (x + w - r) + "," + (y + hh) +
      " L" + x + "," + (y + hh) + " Z";
  }

  function availWidth() {
    var g = el("fx-charts");
    var w = g ? g.clientWidth : 0;
    /* One column on narrow screens, two above 760px. */
    if (w > 760) w = Math.floor((w - 12) / 2);
    return Math.max(260, w - 30);
  }

  /* ------------------------------------------- 1. detections over time */

  function chartDetections(d, wide) {
    var ev = d.events || {};
    var buckets = (ev.buckets || []);
    if (!buckets.length || !ev.total) {
      return emptyCard(
        "Detections over time", "Nothing detected yet",
        state.mode === "demo"
          ? "The replay has not reached the first detection."
          : "No fire, wildlife, obstacle or fault events are recorded in this window. " +
            "The rovers were not running, or they ran and saw nothing worth flagging."
      );
    }

    var unit = ev.bucket_unit, origin = ev.origin || buckets[0].t;
    var fig = card(
      "Detections over time",
      unit === "minute"
        ? "Scripted events per minute of the run"
        : "Rover-raised events per day (UTC)",
      wide
    );

    var pad = { t: 14, r: 14, b: 34, l: 46 };
    var minBand = 34;
    var w = Math.max(availWidth(), pad.l + pad.r + buckets.length * minBand);
    var hgt = 220;
    var plotW = w - pad.l - pad.r, plotH = hgt - pad.t - pad.b;

    var max = 0, i, j;
    for (i = 0; i < buckets.length; i++) {
      var s = 0;
      for (j = 0; j < TYPES.length; j++) s += buckets[i][TYPES[j]] || 0;
      s += buckets[i].other || 0;
      if (s > max) max = s;
    }
    var ticks = niceTicks(max, 4);
    var top = ticks[ticks.length - 1];
    var scaleY = function (v) { return pad.t + plotH - (v / top) * plotH; };

    var svg = svgFrame(fig, w, hgt,
      "Detections over time",
      buckets.length + " buckets, " + ev.total + " detections, tallest bucket " + max + ".");

    gridY(svg, pad, plotW, plotH, ticks, scaleY);
    baseline(svg, pad, plotW, plotH);

    var band = plotW / buckets.length;
    var barW = Math.min(24, band * 0.62);
    var GAP = 2; /* surface gap between stacked segments */
    var keys = TYPES.concat(["other"]);

    for (i = 0; i < buckets.length; i++) {
      var b = buckets[i];
      var x = pad.l + band * i + (band - barW) / 2;
      var acc = 0, drawn = 0, rows = [], total = 0;
      for (j = 0; j < keys.length; j++) total += b[keys[j]] || 0;

      /* Draw top-down so the topmost drawn segment gets the rounded cap. */
      for (j = keys.length - 1; j >= 0; j--) {
        var v = b[keys[j]] || 0;
        if (!v) continue;
        var col = keys[j] === "other" ? css("--fx-muted") : css(TYPE_VAR[keys[j]]);
        rows.push({ color: col, label: TYPE_LABEL[keys[j]], value: String(v) });
      }
      for (j = 0; j < keys.length; j++) {
        var val = b[keys[j]] || 0;
        if (!val) continue;
        var segH = (val / top) * plotH;
        var y = pad.t + plotH - acc - segH;
        var drawH = Math.max(1, segH - (drawn > 0 ? GAP : 0));
        var color = keys[j] === "other" ? css("--fx-muted") : css(TYPE_VAR[keys[j]]);
        var isTop = true;
        for (var k = j + 1; k < keys.length; k++) if (b[keys[k]]) isTop = false;
        if (isTop) {
          svg.appendChild(mk("path", { d: topRounded(x, y, barW, drawH, 4), fill: color }));
        } else {
          svg.appendChild(mk("rect", { x: x, y: y, width: barW, height: drawH, fill: color }));
        }
        acc += segH;
        drawn++;
      }

      /* Selective direct label: the column total on the cap, only when the
         column is tall enough that the number will not collide with the grid. */
      if (total && acc > 16) {
        svg.appendChild(mk("text", {
          x: x + barW / 2, y: pad.t + plotH - acc - 6, "text-anchor": "middle",
          "font-size": 11, fill: css("--fx-ink-2"),
          "font-variant-numeric": "tabular-nums"
        }, String(total)));
      }

      /* x label, thinned so labels never overlap */
      var every = Math.ceil(52 / band);
      if (i % every === 0 || i === buckets.length - 1) {
        svg.appendChild(mk("text", {
          x: pad.l + band * i + band / 2, y: hgt - 12, "text-anchor": "middle",
          "font-size": 11, fill: css("--fx-muted")
        }, bucketLabel(b.t, unit, origin)));
      }

      /* Hit target is the whole band, so it clears 24px even when bars are thin. */
      var hit = mk("rect", {
        "class": "fx-hit", x: pad.l + band * i, y: pad.t,
        width: band, height: plotH, fill: "transparent",
        tabindex: "0", role: "img",
        "aria-label": bucketLabel(b.t, unit, origin) + ": " + total + " detections" +
          (rows.length ? " — " + rows.map(function (r) { return r.label + " " + r.value; }).join(", ") : "")
      });
      (function (rr, label, tot) {
        var open = function (e) {
          var r = hit.getBoundingClientRect();
          showTip(
            e.clientX !== undefined ? e.clientX : r.left + r.width / 2,
            e.clientY !== undefined ? e.clientY : r.top,
            label + " — " + tot + " total", rr
          );
        };
        hit.addEventListener("mousemove", open);
        hit.addEventListener("focus", open);
        hit.addEventListener("mouseleave", hideTip);
        hit.addEventListener("blur", hideTip);
      })(rows, bucketLabel(b.t, unit, origin), total);
      svg.appendChild(hit);
    }

    var items = [];
    for (i = 0; i < TYPES.length; i++) {
      items.push({ color: css(TYPE_VAR[TYPES[i]]), label: TYPE_LABEL[TYPES[i]] });
    }
    if (ev.totals && ev.totals.other) items.push({ color: css("--fx-muted"), label: "Other" });
    legend(fig, items);

    var trows = buckets.map(function (b) {
      var row = [bucketLabel(b.t, unit, origin)];
      for (var q = 0; q < TYPES.length; q++) row.push(String(b[TYPES[q]] || 0));
      row.push(String(b.other || 0));
      return row;
    });
    tableView(fig, ["Bucket", "Wildlife", "Fire", "Obstacle", "Fault", "Other"],
      trows, "Detections per bucket by type");
    return fig;
  }

  /* -------------------------------------------- 2. detections by type */

  function chartTypes(d) {
    var ev = d.events || {}, totals = ev.totals || {};
    var rows = [];
    for (var i = 0; i < TYPES.length; i++) {
      rows.push({ key: TYPES[i], label: TYPE_LABEL[TYPES[i]], value: totals[TYPES[i]] || 0 });
    }
    if (totals.other) rows.push({ key: "other", label: "Other", value: totals.other });
    var max = rows.reduce(function (m, r) { return r.value > m ? r.value : m; }, 0);
    if (!max) {
      return emptyCard(
        "Detections by type", "No detections to break down",
        "This chart fills in as soon as the fleet raises its first event."
      );
    }

    var fig = card("Detections by type",
      "Totals for the window, with resolutions where the rover cleared the alert", false);

    var pad = { t: 8, r: 58, b: 8, l: 76 };
    var w = Math.max(availWidth(), 300);
    var bandH = 34;
    var hgt = pad.t + pad.b + rows.length * bandH;
    var plotW = w - pad.l - pad.r;
    var barH = Math.min(24, bandH * 0.6);

    var svg = svgFrame(fig, w, hgt, "Detections by type",
      rows.map(function (r) { return r.label + " " + r.value; }).join(", ") + ".");

    for (i = 0; i < rows.length; i++) {
      var r = rows[i];
      var y = pad.t + bandH * i + (bandH - barH) / 2;
      var bw = Math.max(r.value > 0 ? 3 : 0, (r.value / max) * plotW);
      var color = r.key === "other" ? css("--fx-muted") : css(TYPE_VAR[r.key]);

      /* category name in the gutter: identity is never colour-alone */
      svg.appendChild(mk("text", {
        x: pad.l - 10, y: y + barH / 2 + 4, "text-anchor": "end",
        "font-size": 12, fill: css("--fx-ink-2")
      }, r.label));

      if (bw > 0) {
        svg.appendChild(mk("path", {
          d: rightRounded(pad.l, y, bw, barH, 4), fill: color
        }));
      }

      /* Value at the tip, always outside the bar end so it can never be
         clipped by a short bar. */
      var lab = String(r.value);
      var res = (ev.resolved || {})[r.key];
      if (res) lab += "  (" + res + " cleared)";
      svg.appendChild(mk("text", {
        x: pad.l + bw + 8, y: y + barH / 2 + 4, "font-size": 12,
        fill: css("--fx-ink"), "font-variant-numeric": "tabular-nums"
      }, lab));

      var hit = mk("rect", {
        "class": "fx-hit", x: pad.l, y: pad.t + bandH * i,
        width: plotW + pad.r, height: bandH, fill: "transparent",
        tabindex: "0", role: "img",
        "aria-label": r.label + ": " + r.value + " detections" +
          (res ? ", " + res + " cleared" : "")
      });
      (function (rr, col) {
        var open = function (e) {
          var bb = hit.getBoundingClientRect();
          showTip(e.clientX !== undefined ? e.clientX : bb.left + 40,
            e.clientY !== undefined ? e.clientY : bb.top,
            rr.label, [{ color: col, label: "Detections", value: String(rr.value) }]);
        };
        hit.addEventListener("mousemove", open);
        hit.addEventListener("focus", open);
        hit.addEventListener("mouseleave", hideTip);
        hit.addEventListener("blur", hideTip);
      })(r, color);
      svg.appendChild(hit);
    }

    tableView(fig, ["Type", "Detections", "Cleared"],
      rows.map(function (r) {
        return [r.label, String(r.value), String((ev.resolved || {})[r.key] || 0)];
      }), "Detection totals by type");
    return fig;
  }

  /* ------------------------------------------------- 3. run / mission */

  function chartRuns(d) {
    var runs = [];
    var rovers = d.rovers || [];
    for (var i = 0; i < rovers.length; i++) {
      for (var j = 0; j < (rovers[i].runs || []).length; j++) {
        runs.push({ thing: rovers[i].thing, run: rovers[i].runs[j] });
      }
    }
    if (!runs.length) {
      return emptyCard(
        "Mission runs", "No runs recorded",
        "Runs are reconstructed from mission telemetry, which reaches the cloud " +
        "through the field laptop. None was relayed in this window."
      );
    }

    /* One run is a number, not a chart. The stat tile IS the right form. */
    if (runs.length === 1) {
      var r0 = runs[0].run;
      var fig1 = card("Mission run", "Reconstructed from mission telemetry", false);
      var wrap = h("div", "fx-kpis");
      wrap.style.marginBottom = "0";
      wrap.appendChild(kpi("Duration", fmtDur(r0.duration_s), fmtDate(r0.start)));
      wrap.appendChild(kpi("Distance", fmtDist(r0.distance_mm), runs[0].thing));
      wrap.appendChild(kpi("Mean speed",
        (r0.duration_s && r0.distance_mm !== null && r0.distance_mm !== undefined)
          ? (r0.distance_mm / r0.duration_s).toFixed(0) + " mm/s"
          : "no odometry",
        "over the whole run"));
      wrap.appendChild(kpi("Samples", fmtInt(r0.samples), "relayed telemetry rows"));
      fig1.appendChild(wrap);
      tableView(fig1, ["Run", "Rover", "Start", "Duration", "Distance"],
        [["1", runs[0].thing, fmtDate(r0.start), fmtDur(r0.duration_s), fmtDist(r0.distance_mm)]],
        "Mission run detail");
      return fig1;
    }

    var fig = card("Mission runs", "Duration per reconstructed run, most recent last", false);
    var pad = { t: 8, r: 76, b: 8, l: 66 };
    var w = Math.max(availWidth(), 300);
    var bandH = 32;
    var hgt = pad.t + pad.b + runs.length * bandH;
    var plotW = w - pad.l - pad.r;
    var barH = Math.min(24, bandH * 0.6);
    var max = runs.reduce(function (m, x) { return x.run.duration_s > m ? x.run.duration_s : m; }, 0) || 1;
    var color = css("--fx-s1");

    var svg = svgFrame(fig, w, hgt, "Mission run durations",
      runs.length + " runs, longest " + fmtDur(max) + ".");

    for (i = 0; i < runs.length; i++) {
      var run = runs[i].run;
      var y = pad.t + bandH * i + (bandH - barH) / 2;
      var bw = Math.max(3, (run.duration_s / max) * plotW);
      svg.appendChild(mk("text", {
        x: pad.l - 10, y: y + barH / 2 + 4, "text-anchor": "end",
        "font-size": 12, fill: css("--fx-ink-2")
      }, "Run " + run.index));
      svg.appendChild(mk("path", { d: rightRounded(pad.l, y, bw, barH, 4), fill: color }));
      svg.appendChild(mk("text", {
        x: pad.l + bw + 8, y: y + barH / 2 + 4, "font-size": 12,
        fill: css("--fx-ink"), "font-variant-numeric": "tabular-nums"
      }, fmtDur(run.duration_s)));

      var hit = mk("rect", {
        "class": "fx-hit", x: pad.l, y: pad.t + bandH * i,
        width: plotW + pad.r, height: bandH, fill: "transparent",
        tabindex: "0", role: "img",
        "aria-label": "Run " + run.index + " on " + runs[i].thing + ", " +
          fmtDur(run.duration_s) + ", " + fmtDist(run.distance_mm) +
          ", started " + fmtDate(run.start)
      });
      (function (rn, th) {
        var open = function (e) {
          var bb = hit.getBoundingClientRect();
          showTip(e.clientX !== undefined ? e.clientX : bb.left + 40,
            e.clientY !== undefined ? e.clientY : bb.top,
            "Run " + rn.index + " — " + th, [
              { color: color, label: "Duration", value: fmtDur(rn.duration_s) },
              { label: "Distance", value: fmtDist(rn.distance_mm) },
              { label: "Started", value: fmtDate(rn.start) }
            ]);
        };
        hit.addEventListener("mousemove", open);
        hit.addEventListener("focus", open);
        hit.addEventListener("mouseleave", hideTip);
        hit.addEventListener("blur", hideTip);
      })(run, runs[i].thing);
      svg.appendChild(hit);
    }

    tableView(fig, ["Run", "Rover", "Start", "Duration", "Distance"],
      runs.map(function (x) {
        return [String(x.run.index), x.thing, fmtDate(x.run.start),
          fmtDur(x.run.duration_s), fmtDist(x.run.distance_mm)];
      }), "Reconstructed mission runs");
    return fig;
  }

  /* ------------------------------------------- generic time-series line */

  /**
   * series: [{ key, label, color, points:[{t, v}] }]
   * opts:   { title, subtitle, unit, fmt, threshold:{v,label,color},
   *           yZero:bool, domain:[lo,hi] }
   */
  function chartLine(series, opts) {
    var live = series.filter(function (s) { return s.points.length; });
    if (!live.length) return null;

    var fig = card(opts.title, opts.subtitle, opts.wide);
    var pad = { t: 14, r: 16, b: 30, l: 52 };
    var w = Math.max(availWidth(), 300);
    var hgt = opts.height || 200;
    var plotW = w - pad.l - pad.r, plotH = hgt - pad.t - pad.b;

    var tmin = Infinity, tmax = -Infinity, vmin = Infinity, vmax = -Infinity, i, j;
    for (i = 0; i < live.length; i++) {
      for (j = 0; j < live[i].points.length; j++) {
        var p = live[i].points[j];
        if (p.t < tmin) tmin = p.t;
        if (p.t > tmax) tmax = p.t;
        if (p.v === null || p.v === undefined) continue;
        if (p.v < vmin) vmin = p.v;
        if (p.v > vmax) vmax = p.v;
      }
    }
    if (!isFinite(vmin)) return null;
    if (tmax === tmin) tmax = tmin + 1;

    var lo, hi;
    if (opts.domain) { lo = opts.domain[0]; hi = opts.domain[1]; }
    else if (opts.yZero) { lo = 0; hi = vmax || 1; }
    else {
      /* A physical measurement (volts) has no meaningful zero baseline, so the
         window is padded around the data AND the threshold is drawn and
         labelled, which is what keeps a non-zero axis honest. */
      var span = (vmax - vmin) || Math.abs(vmax) * 0.1 || 1;
      lo = vmin - span * 0.25; hi = vmax + span * 0.25;
      if (opts.threshold) {
        lo = Math.min(lo, opts.threshold.v - span * 0.15);
        hi = Math.max(hi, opts.threshold.v + span * 0.15);
      }
    }
    if (hi === lo) hi = lo + 1;

    var sx = function (t) { return pad.l + ((t - tmin) / (tmax - tmin)) * plotW; };
    var sy = function (v) { return pad.t + plotH - ((v - lo) / (hi - lo)) * plotH; };
    var fmt = opts.fmt || fmtInt;

    var svg = svgFrame(fig, w, hgt, opts.title,
      live.map(function (s) {
        return s.label + " from " + fmt(s.points[0].v) +
          " to " + fmt(s.points[s.points.length - 1].v);
      }).join("; ") + ".");

    var ticks = opts.yZero
      ? niceTicks(hi, 4)
      : [lo, lo + (hi - lo) / 2, hi].map(function (v) { return Math.round(v * 100) / 100; });
    for (i = 0; i < ticks.length; i++) {
      if (ticks[i] < lo - 1e-9 || ticks[i] > hi + 1e-9) continue;
      svg.appendChild(mk("line", {
        x1: pad.l, y1: sy(ticks[i]), x2: pad.l + plotW, y2: sy(ticks[i]),
        stroke: css("--fx-grid"), "stroke-width": 1, "shape-rendering": "crispEdges"
      }));
      svg.appendChild(mk("text", {
        x: pad.l - 8, y: sy(ticks[i]) + 4, "text-anchor": "end", "font-size": 11,
        fill: css("--fx-muted"), "font-variant-numeric": "tabular-nums"
      }, fmt(ticks[i])));
    }
    baseline(svg, pad, plotW, plotH);

    /* threshold reference */
    if (opts.threshold && opts.threshold.v >= lo && opts.threshold.v <= hi) {
      var ty = sy(opts.threshold.v);
      svg.appendChild(mk("line", {
        x1: pad.l, y1: ty, x2: pad.l + plotW, y2: ty,
        stroke: opts.threshold.color, "stroke-width": 1.5
      }));
      svg.appendChild(mk("text", {
        x: pad.l + plotW, y: ty - 6, "text-anchor": "end", "font-size": 11,
        fill: css("--fx-ink-2")
      }, opts.threshold.label));
    }

    /* lines */
    for (i = 0; i < live.length; i++) {
      var s = live[i], dpath = "", pen = false;
      for (j = 0; j < s.points.length; j++) {
        var pt = s.points[j];
        if (pt.v === null || pt.v === undefined) { pen = false; continue; }
        dpath += (pen ? " L" : " M") + sx(pt.t) + "," + sy(pt.v);
        pen = true;
      }
      if (live.length === 1 && opts.area !== false) {
        /* single-series wash at ~10% */
        var fp = "", started = false, lastX = null;
        for (j = 0; j < s.points.length; j++) {
          var q = s.points[j];
          if (q.v === null || q.v === undefined) continue;
          fp += (started ? " L" : "M") + sx(q.t) + "," + sy(q.v);
          started = true; lastX = sx(q.t);
        }
        if (started) {
          var firstX = sx(s.points.filter(function (z) { return z.v !== null; })[0].t);
          fp += " L" + lastX + "," + (pad.t + plotH) + " L" + firstX + "," + (pad.t + plotH) + " Z";
          svg.appendChild(mk("path", { d: fp, fill: s.color, "fill-opacity": ".10" }));
        }
      }
      svg.appendChild(mk("path", {
        d: dpath.trim(), fill: "none", stroke: s.color, "stroke-width": 2,
        "stroke-linejoin": "round", "stroke-linecap": "round"
      }));

      /* end marker with a 2px surface ring, plus the end value directly labelled */
      var lastPt = null;
      for (j = s.points.length - 1; j >= 0; j--) {
        if (s.points[j].v !== null && s.points[j].v !== undefined) { lastPt = s.points[j]; break; }
      }
      if (lastPt) {
        svg.appendChild(mk("circle", {
          cx: sx(lastPt.t), cy: sy(lastPt.v), r: 4.5, fill: s.color,
          stroke: css("--fx-surface"), "stroke-width": 2
        }));
        svg.appendChild(mk("text", {
          x: Math.min(sx(lastPt.t) + 8, pad.l + plotW),
          y: sy(lastPt.v) - 9, "text-anchor": sx(lastPt.t) > pad.l + plotW - 44 ? "end" : "start",
          "font-size": 11, fill: css("--fx-ink"),
          "font-variant-numeric": "tabular-nums"
        }, fmt(lastPt.v)));
      }
    }

    /* x labels: first and last only, so they can never collide */
    svg.appendChild(mk("text", {
      x: pad.l, y: hgt - 10, "font-size": 11, fill: css("--fx-muted")
    }, opts.xfmt ? opts.xfmt(tmin) : shortTime(tmin)));
    svg.appendChild(mk("text", {
      x: pad.l + plotW, y: hgt - 10, "text-anchor": "end", "font-size": 11,
      fill: css("--fx-muted")
    }, opts.xfmt ? opts.xfmt(tmax) : shortTime(tmax)));

    /* crosshair + tooltip, keyboard-navigable with the arrow keys */
    var cross = mk("line", {
      y1: pad.t, y2: pad.t + plotH, stroke: css("--fx-axis"),
      "stroke-width": 1, opacity: 0, "pointer-events": "none"
    });
    svg.appendChild(cross);
    var dots = mk("g", { "pointer-events": "none" });
    svg.appendChild(dots);

    var idx = -1;
    var base = live[0].points;
    function point(k, clientX, clientY) {
      if (k < 0 || k >= base.length) return;
      idx = k;
      var t = base[k].t, x = sx(t);
      cross.setAttribute("x1", x); cross.setAttribute("x2", x);
      cross.setAttribute("opacity", "1");
      dots.textContent = "";
      var rows = [];
      for (var q = 0; q < live.length; q++) {
        var near = nearest(live[q].points, t);
        if (!near || near.v === null) continue;
        dots.appendChild(mk("circle", {
          cx: sx(near.t), cy: sy(near.v), r: 4, fill: live[q].color,
          stroke: css("--fx-surface"), "stroke-width": 2
        }));
        rows.push({ color: live[q].color, label: live[q].label, value: fmt(near.v) + (opts.unit || "") });
      }
      var bb = svg.getBoundingClientRect();
      showTip(clientX !== undefined ? clientX : bb.left + x,
        clientY !== undefined ? clientY : bb.top + pad.t,
        opts.xfmt ? opts.xfmt(t) : fmtDate(t), rows);
    }
    function nearest(pts, t) {
      var best = null, bd = Infinity;
      for (var q = 0; q < pts.length; q++) {
        var dd = Math.abs(pts[q].t - t);
        if (dd < bd) { bd = dd; best = pts[q]; }
      }
      return best;
    }
    svg.addEventListener("mousemove", function (e) {
      var bb = svg.getBoundingClientRect();
      var rel = (e.clientX - bb.left) * (w / bb.width);
      var frac = (rel - pad.l) / plotW;
      var k = Math.round(frac * (base.length - 1));
      point(Math.max(0, Math.min(base.length - 1, k)), e.clientX, e.clientY);
    });
    svg.addEventListener("mouseleave", function () {
      cross.setAttribute("opacity", "0"); dots.textContent = ""; hideTip();
    });
    svg.addEventListener("blur", function () {
      cross.setAttribute("opacity", "0"); dots.textContent = ""; hideTip();
    });
    svg.addEventListener("keydown", function (e) {
      if (e.key === "ArrowRight") { point(Math.min(base.length - 1, (idx < 0 ? -1 : idx) + 1)); e.preventDefault(); }
      else if (e.key === "ArrowLeft") { point(Math.max(0, (idx < 0 ? 1 : idx) - 1)); e.preventDefault(); }
      else if (e.key === "Escape") { cross.setAttribute("opacity", "0"); dots.textContent = ""; hideTip(); }
    });

    /* A single series needs no legend box — the title names it. */
    if (live.length > 1) {
      legend(fig, live.map(function (s) {
        return { color: s.color, label: s.label, line: true };
      }));
    }

    var cols = ["Time"].concat(live.map(function (s) { return s.label; }));
    var trows = base.map(function (p) {
      var row = [opts.xfmt ? opts.xfmt(p.t) : fmtDate(p.t)];
      for (var q = 0; q < live.length; q++) {
        var n2 = nearest(live[q].points, p.t);
        row.push(n2 && n2.v !== null ? fmt(n2.v) + (opts.unit || "") : "-");
      }
      return row;
    });
    tableView(fig, cols, trows, opts.title);
    return fig;
  }

  function shortTime(ms) {
    try {
      return new Date(ms).toLocaleString(undefined, {
        month: "short", day: "numeric", hour: "2-digit", minute: "2-digit"
      });
    } catch (e) { return String(ms); }
  }

  /* ---------------------------------------- 6b. LiDAR sector radial */

  function chartSectors(rover) {
    var sectors = rover.lidar && rover.lidar.latest_sectors_mm;
    if (!sectors || !sectors.length) return null;

    var fig = card("LiDAR sweep, latest scan",
      "Twelve 30 degree sectors. Wedge length is the nearest return in that sector.", false);

    var w = Math.max(Math.min(availWidth(), 380), 260);
    var hgt = w;
    var cx = w / 2, cy = hgt / 2;
    var R = Math.min(cx, cy) - 30;
    var rangeMm = rover.lidar.latest_range_max_mm || 6000;
    /* Clamp the visual scale to a bit beyond the arena: at 6 m full scale every
       wedge inside a 1.2 m arena would be a stub. */
    var scaleMm = Math.min(rangeMm, 900);
    var hit = sectors.filter(function (v) { return v !== null && v > 0; }).length;

    var svg = svgFrame(fig, w, hgt, "LiDAR sector coverage",
      hit + " of " + sectors.length + " sectors returned a range; nearest " +
      fmtDist(Math.min.apply(null, sectors.filter(function (v) { return v !== null; }))) + ".");

    /* range rings, solid hairlines, labelled */
    for (var ring = 1; ring <= 3; ring++) {
      var rr = (R * ring) / 3;
      svg.appendChild(mk("circle", {
        cx: cx, cy: cy, r: rr, fill: "none",
        stroke: css("--fx-grid"), "stroke-width": 1
      }));
      svg.appendChild(mk("text", {
        x: cx + 4, y: cy - rr - 3, "font-size": 10, fill: css("--fx-muted"),
        "font-variant-numeric": "tabular-nums"
      }, Math.round((scaleMm * ring) / 3) + " mm"));
    }
    /* arena reference: half the arena side, as a scale anchor */
    var arenaR = (R * ((${ARENA_MM} / 2) / scaleMm));
    if (arenaR > 8 && arenaR < R) {
      svg.appendChild(mk("circle", {
        cx: cx, cy: cy, r: arenaR, fill: "none",
        stroke: css("--fx-axis"), "stroke-width": 1
      }));
    }

    var step = (Math.PI * 2) / sectors.length;
    var GAPRAD = 0.024; /* the 2px-equivalent surface gap, in radians */
    for (var i = 0; i < sectors.length; i++) {
      var a0 = -Math.PI / 2 + step * i + GAPRAD;
      var a1 = -Math.PI / 2 + step * (i + 1) - GAPRAD;
      var v = sectors[i];
      var rOut = v === null || v <= 0 ? R : Math.max(6, Math.min(R, (v / scaleMm) * R));
      var x0 = cx + Math.cos(a0) * rOut, y0 = cy + Math.sin(a0) * rOut;
      var x1 = cx + Math.cos(a1) * rOut, y1 = cy + Math.sin(a1) * rOut;
      var dd = "M" + cx + "," + cy + " L" + x0 + "," + y0 +
        " A" + rOut + "," + rOut + " 0 0 1 " + x1 + "," + y1 + " Z";

      var isNull = (v === null || v <= 0);
      var wedge = mk("path", {
        "class": "fx-hit", d: dd,
        fill: isNull ? "transparent" : css("--fx-s1"),
        "fill-opacity": isNull ? "0" : ".30",
        stroke: isNull ? css("--fx-axis") : css("--fx-s1"),
        "stroke-width": isNull ? 1 : 2,
        "stroke-dasharray": null,
        tabindex: "0", role: "img",
        "aria-label": "Sector " + (i * 30) + " to " + ((i + 1) * 30) + " degrees: " +
          (isNull ? "no return" : fmtDist(v))
      });
      (function (vv, ii, nul) {
        var open = function (e) {
          var bb = wedge.getBoundingClientRect();
          showTip(e.clientX !== undefined ? e.clientX : bb.left + bb.width / 2,
            e.clientY !== undefined ? e.clientY : bb.top,
            (ii * 30) + "\\u00b0 – " + ((ii + 1) * 30) + "\\u00b0",
            [{ color: nul ? css("--fx-muted") : css("--fx-s1"),
               label: nul ? "No return" : "Nearest", value: nul ? "-" : fmtDist(vv) }]);
        };
        wedge.addEventListener("mousemove", open);
        wedge.addEventListener("focus", open);
        wedge.addEventListener("mouseleave", hideTip);
        wedge.addEventListener("blur", hideTip);
      })(v, i, isNull);
      svg.appendChild(wedge);
    }

    /* rover at the origin */
    svg.appendChild(mk("circle", {
      cx: cx, cy: cy, r: 4, fill: css("--fx-ink-2"),
      stroke: css("--fx-surface"), "stroke-width": 2
    }));
    svg.appendChild(mk("text", {
      x: cx, y: 16, "text-anchor": "middle", "font-size": 11, fill: css("--fx-muted")
    }, "0\\u00b0 (front)"));

    legend(fig, [
      { color: css("--fx-s1"), label: "Range returned" },
      { none: true, label: "No return in that sector" }
    ]);
    tableView(fig, ["Sector", "Nearest return"],
      sectors.map(function (v, i2) {
        return [(i2 * 30) + "\\u00b0–" + ((i2 + 1) * 30) + "\\u00b0",
          (v === null || v <= 0) ? "no return" : fmtDist(v)];
      }), "Latest LiDAR sector ranges");
    return fig;
  }

  /* ------------------------------------------------------- stat tiles */

  function kpi(label, value, note, meter) {
    var d = h("div", "fx-kpi");
    d.appendChild(h("div", "fx-k-label", label));
    d.appendChild(h("div", "fx-k-value", value));
    if (note) d.appendChild(h("div", "fx-k-note", note));
    if (meter !== undefined && meter !== null) {
      var m = h("div", "fx-meter");
      var f = document.createElement("i");
      f.style.width = Math.round(Math.max(0, Math.min(1, meter)) * 100) + "%";
      m.appendChild(f);
      d.appendChild(m);
    }
    return d;
  }

  function renderKpis(d) {
    var box = el("fx-kpis");
    box.textContent = "";

    var ev = d.events || {};
    var rovers = d.rovers || [];
    var lidarRover = null, battRover = null, upRover = null, i;
    for (i = 0; i < rovers.length; i++) {
      if (!lidarRover && rovers[i].lidar && rovers[i].lidar.samples) lidarRover = rovers[i];
      if (!battRover && rovers[i].battery && rovers[i].battery.samples) battRover = rovers[i];
      if (!upRover && rovers[i].uptime && rovers[i].uptime.samples) upRover = rovers[i];
    }

    box.appendChild(kpi(
      "Detections", fmtInt(ev.total || 0),
      state.mode === "demo" ? "in the scripted run so far"
        : (d.window_days ? "in the last " + d.window_days + " days" : "in the window")
    ));

    var cov = null;
    if (lidarRover) {
      var pts = lidarRover.lidar.points;
      for (i = pts.length - 1; i >= 0; i--) {
        if (pts[i].coverage !== null) { cov = pts[i].coverage; break; }
      }
    }
    box.appendChild(kpi(
      "LiDAR coverage",
      cov === null ? "no data" : Math.round(cov * 100) + "%",
      cov === null ? "no scans in this window"
        : "of " + (lidarRover.lidar.sector_count || 12) + " sectors, latest scan",
      cov
    ));

    box.appendChild(kpi(
      "Longest uptime",
      upRover && upRover.uptime.peak_uptime_s ? fmtDur(upRover.uptime.peak_uptime_s) : "no data",
      upRover ? "without a reboot, " + upRover.thing : "no heartbeat recorded"
    ));

    var hl = d.health || {};
    box.appendChild(kpi(
      "Analyst checks",
      hl.checks ? hl.nominal + " / " + hl.checks : "none",
      hl.checks ? "reported all-nominal" : "no analysis has run in this window",
      hl.checks ? hl.nominal / hl.checks : null
    ));
  }

  /* -------------------------------------------------------- rendering */

  function renderCharts(d) {
    var grid = el("fx-charts");
    grid.textContent = "";
    var rovers = d.rovers || [], i;

    grid.appendChild(chartDetections(d, true));
    grid.appendChild(chartTypes(d));
    grid.appendChild(chartRuns(d));

    /* Uptime — one line per rover, capped at the categorical slot order. */
    var upSeries = [];
    var slots = ["--fx-s1", "--fx-s2", "--fx-s3"];
    for (i = 0; i < rovers.length && i < 3; i++) {
      var u = rovers[i].uptime;
      if (!u || !u.samples) continue;
      upSeries.push({
        key: rovers[i].thing, label: rovers[i].thing, color: css(slots[i]),
        points: u.points.map(function (p) { return { t: p.t, v: p.uptime_s }; })
      });
    }
    var upFig = chartLine(upSeries, {
      title: "Onboard uptime",
      subtitle: "From the rover's 5 s health heartbeat. A drop to zero is a reboot.",
      yZero: true, fmt: function (v) { return fmtDur(v); }, height: 190
    });
    grid.appendChild(upFig || emptyCard(
      "Onboard uptime", "No heartbeat in this window",
      "The rover publishes a health heartbeat every five seconds while it is " +
      "powered on. None reached the archive here."
    ));

    /* Battery — single series, volts, with the pack low threshold drawn. */
    var battFig = null;
    for (i = 0; i < rovers.length; i++) {
      var b = rovers[i].battery;
      if (!b || !b.samples) continue;
      battFig = chartLine([{
        key: rovers[i].thing, label: rovers[i].thing + " pack voltage",
        color: css("--fx-s1"),
        points: b.points.map(function (p) { return { t: p.t, v: p.v }; })
      }], {
        title: "Battery voltage",
        subtitle: "Measured at the drive board. The axis is windowed around the " +
          "readings, with the " + b.low_v + " V low threshold drawn.",
        unit: " V", fmt: function (v) { return v.toFixed(2); },
        threshold: { v: b.low_v, label: "low " + b.low_v + " V", color: css("--fx-crit") },
        height: 190
      });
      break;
    }
    grid.appendChild(battFig || emptyCard(
      "Battery voltage", "No battery telemetry",
      "Pack voltage reaches the cloud through the field laptop's relay, not the " +
      "rover's direct uplink. With the laptop off, none is recorded — so this " +
      "chart is empty rather than flat."
    ));

    /* LiDAR coverage over time */
    var lidSeries = [];
    for (i = 0; i < rovers.length && i < 3; i++) {
      var l = rovers[i].lidar;
      if (!l || !l.samples) continue;
      lidSeries.push({
        key: rovers[i].thing, label: rovers[i].thing, color: css(slots[i]),
        points: l.points.map(function (p) { return { t: p.t, v: p.coverage === null ? null : p.coverage * 100 }; })
      });
    }
    var lidFig = chartLine(lidSeries, {
      title: "LiDAR coverage",
      subtitle: "Share of the twelve 30 degree sectors returning a range on each scan.",
      yZero: true, domain: [0, 100], unit: "%",
      fmt: function (v) { return Math.round(v) + "%"; }, height: 190
    });
    grid.appendChild(lidFig || emptyCard(
      "LiDAR coverage", "No scans in this window",
      "Coverage is counted from the sweep summary the rover publishes with " +
      "every scan. None was archived here."
    ));

    for (i = 0; i < rovers.length; i++) {
      var sec = chartSectors(rovers[i]);
      if (sec) { grid.appendChild(sec); break; }
    }
  }

  function renderProvenance(d) {
    var p = el("fx-prov");
    if (state.mode === "demo") {
      p.textContent =
        "Simulated dataset. Scripted run anchored to " +
        fmtDate(d.scenario ? d.scenario.anchor : null) +
        " — a fixed date, not the current time. " +
        (d.scenario ? d.scenario.note : "");
    } else {
      p.textContent =
        "Live archive. Detections and analyst checks are bucketed by UTC day; " +
        "per-rover charts sample the most recent readings in the window. " +
        "Empty panels mean the archive holds nothing for that stream, not zero.";
    }
  }

  function render() {
    var d = current();
    if (!d) return;
    root.setAttribute("data-mode", state.mode);
    el("fx-live").setAttribute("aria-pressed", state.mode === "live" ? "true" : "false");
    el("fx-demo").setAttribute("aria-pressed", state.mode === "demo" ? "true" : "false");
    el("fx-simbar").hidden = state.mode !== "demo";
    el("fx-replay").hidden = state.mode !== "demo";
    el("fx-source").textContent = state.mode === "demo"
      ? "Showing a scripted simulation"
      : (state.live && state.live.window_days
        ? "Live archive, last " + state.live.window_days + " days"
        : "Live archive");
    renderKpis(d);
    renderCharts(d);
    renderProvenance(d);
  }

  /* -------------------------------------------------- the demo replay */

  /**
   * Slice the scripted dataset at the playhead. Nothing is generated here —
   * the demo is a fixed dataset and the replay only reveals more of it.
   */
  function demoAt(head) {
    var d = state.demo;
    if (!d) return null;
    var t0 = d.scenario.anchor;
    var cut = t0 + head * 1000;

    var clipPts = function (pts) {
      return pts.filter(function (p) { return p.t <= cut; });
    };
    var clipBuckets = function (bs, unit) {
      var span = unit === "minute" ? 60000 : 86400000;
      return bs.filter(function (b) { return b.t <= cut; }).map(function (b) {
        if (b.t + span <= cut) return b;
        /* Partial bucket: recount from the timeline so a bucket in progress
           never shows counts that have not been "reached" yet. */
        var out = { t: b.t };
        for (var k in b) if (k !== "t") out[k] = 0;
        for (var i = 0; i < (d.timeline || []).length; i++) {
          var e = d.timeline[i];
          if (e.t > cut || e.t < b.t || e.t >= b.t + span) continue;
          if (e.resolves) continue;
          if (out[e.type] !== undefined) out[e.type]++;
        }
        return out;
      });
    };

    var totals = {}, resolved = {}, total = 0;
    for (var i = 0; i < (d.timeline || []).length; i++) {
      var e = d.timeline[i];
      if (e.t > cut) continue;
      if (e.resolves) { resolved[e.resolves] = (resolved[e.resolves] || 0) + 1; continue; }
      totals[e.type] = (totals[e.type] || 0) + 1;
      total++;
    }

    var health = { bucket_unit: "minute", origin: t0, buckets: [], checks: 0, nominal: 0 };
    var hb = clipBuckets(d.health.buckets, "minute");
    health.buckets = hb;
    for (i = 0; i < hb.length; i++) {
      health.checks += hb[i].ok + hb[i].warning + hb[i].critical;
      health.nominal += hb[i].ok;
    }

    var rover = d.rovers[0];
    var lidPts = clipPts(rover.lidar.points);
    var battPts = clipPts(rover.battery.points);
    var upPts = clipPts(rover.uptime.points);
    /* The "latest scan" radial follows the playhead too. */
    var sectors = head >= d.scenario.duration_s ? rover.lidar.latest_sectors_mm
      : (lidPts.length ? rover.lidar.latest_sectors_mm : null);

    return {
      simulated: true,
      source: "simulated",
      scenario: d.scenario,
      window_days: null,
      events: {
        bucket_unit: "minute", origin: t0,
        buckets: clipBuckets(d.events.buckets, "minute"),
        totals: totals, resolved: resolved, total: total
      },
      health: health,
      rovers: [{
        thing: rover.thing,
        uptime: {
          points: upPts, samples: upPts.length,
          camera_ok_pct: rover.uptime.camera_ok_pct,
          lidar_ok_pct: rover.uptime.lidar_ok_pct,
          peak_uptime_s: upPts.length ? upPts[upPts.length - 1].uptime_s : null
        },
        lidar: {
          points: lidPts, samples: lidPts.length,
          latest_sectors_mm: lidPts.length ? sectors : null,
          latest_range_max_mm: rover.lidar.latest_range_max_mm,
          sector_count: rover.lidar.sector_count
        },
        battery: {
          points: battPts, samples: battPts.length,
          low_v: rover.battery.low_v, source: "simulated"
        },
        runs: head > 20 ? [{
          index: 1, start: t0, end: t0 + head * 1000,
          duration_s: Math.round(head),
          distance_mm: Math.round(rover.runs[0].distance_mm * (head / d.scenario.duration_s)),
          samples: Math.max(2, Math.floor(head / 15))
        }] : []
      }],
      arena_mm: d.arena_mm
    };
  }

  function current() {
    return state.mode === "demo" ? demoAt(state.head) : state.live;
  }

  var TICK_MS = 200;
  var TICK_SCENARIO_S = 4;   /* 20x real time: a 6 minute run replays in ~18 s */

  function tick() {
    if (!state.demo) return;
    state.head += TICK_SCENARIO_S;
    if (state.head > state.demo.scenario.duration_s) state.head = 0;
    el("fx-scrub").value = String(Math.round(state.head));
    updateClock();
    render();
  }
  function updateClock() {
    if (!state.demo) return;
    el("fx-clock").textContent =
      mmss(state.head) + " / " + mmss(state.demo.scenario.duration_s);
  }
  function play() {
    if (state.timer || !state.demo) return;
    state.playing = true;
    el("fx-play").textContent = "Pause";
    el("fx-play").setAttribute("aria-label", "Pause the simulated replay");
    state.timer = setInterval(tick, TICK_MS);
  }
  function pause() {
    state.playing = false;
    if (state.timer) { clearInterval(state.timer); state.timer = null; }
    if (el("fx-play")) {
      el("fx-play").textContent = "Play";
      el("fx-play").setAttribute("aria-label", "Play the simulated replay");
    }
  }

  function announce(msg) { el("fx-announce").textContent = msg; }

  function setMode(mode) {
    if (mode === state.mode) return;
    state.mode = mode;
    if (mode === "demo") {
      announce("Now showing a simulated demonstration run. This is not live data.");
      loadDemo().then(function () {
        el("fx-scrub").max = String(state.demo ? state.demo.scenario.duration_s : 360);
        updateClock();
        render();
        play();
      });
    } else {
      pause();
      announce("Now showing the live archive.");
      render();
    }
  }

  /* --------------------------------------------------------- fetching */

  function getJSON(path) {
    return fetch(path, { headers: { Accept: "application/json" } })
      .then(function (r) { return r.ok ? r.json() : null; })
      .catch(function () { return null; });
  }

  var demoPromise = null;
  function loadDemo() {
    if (state.demo) return Promise.resolve(state.demo);
    if (!demoPromise) {
      demoPromise = getJSON("/api/public/demo").then(function (d) {
        /* Refuse to render anything from this endpoint that does not declare
           itself simulated. Cheap, and it means a misconfigured route can never
           quietly promote scripted data into the live view. */
        if (d && d.simulated === true && d.source === "simulated") state.demo = d;
        return state.demo;
      });
    }
    return demoPromise;
  }

  function loadLive() {
    return getJSON("/api/public/analytics").then(function (d) {
      if (!d || d.simulated) return null;
      state.live = d;
      return d;
    });
  }

  /* --------------------------------------------------------- wiring up */

  el("fx-live").addEventListener("click", function () { setMode("live"); });
  el("fx-demo").addEventListener("click", function () {
    if (state.mode === "demo") { state.playing ? pause() : play(); return; }
    setMode("demo");
  });
  el("fx-play").addEventListener("click", function () {
    state.playing ? pause() : play();
  });
  el("fx-scrub").addEventListener("input", function (e) {
    pause();
    state.head = Number(e.target.value) || 0;
    updateClock();
    render();
  });

  /* Re-render on resize: the SVGs are sized in px against the container. */
  var rt = null;
  window.addEventListener("resize", function () {
    if (rt) clearTimeout(rt);
    rt = setTimeout(function () { render(); }, 200);
  });

  /* Stop the replay timer while the tab is hidden — nobody is looking. */
  document.addEventListener("visibilitychange", function () {
    if (document.hidden) { if (state.timer) { clearInterval(state.timer); state.timer = null; } }
    else if (state.playing && state.mode === "demo") { state.timer = setInterval(tick, TICK_MS); }
  });

  /**
   * Optional hook — wiring step (W6). If the page's own poll tells us nobody is
   * online we can switch to the demo immediately. Without it the decision falls
   * back to the analytics payload below, which is slower but identical.
   */
  window.FPMS_ANALYTICS = {
    onSummary: function (d) {
      if (!d) return;
      var online = (d.counts && d.counts.rovers_online) || 0;
      var was = state.liveOnline;
      state.liveOnline = online;
      /* Only auto-switch once, and never away from a mode the visitor chose. */
      if (was === null && online === 0 && state.mode === "live" && !state.userChose) {
        setMode("demo");
      }
    }
  };
  el("fx-live").addEventListener("click", function () { state.userChose = true; });
  el("fx-demo").addEventListener("click", function () { state.userChose = true; });

  /* Initial load. One request; analytics is cached for five minutes at the edge
     and refreshed no more often than that, so an open tab costs almost nothing. */
  loadLive().then(function (d) {
    render();
    /* Self-sufficient fallback for the auto-demo: if the archive has nothing
       worth charting, offer the replay without waiting for the summary hook. */
    if ((!d || !d.data_points) && state.mode === "live" && !state.userChose) {
      setMode("demo");
    }
  });

  setInterval(function () {
    if (document.hidden || state.mode !== "live") return;
    loadLive().then(function () { render(); });
  }, 300000);
})();
</` + `script>`;

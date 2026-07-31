/**
 * FPMS public view — the read-only face of the system, no login required.
 *
 * WHY THIS IS SEPARATE FROM THE AUTHENTICATED API
 * -----------------------------------------------
 * Everything under /api/ in worker.js sits behind a session because it can
 * disclose the fleet's internals: broker hosts, LAN addresses, raw LiDAR,
 * camera frames, analyst prompts. This module exposes a deliberately narrow,
 * whitelisted projection of that data for anonymous viewers.
 *
 *   GET /live                    the public status page (self-contained HTML)
 *   GET /api/public/summary      fleet status, rolled up
 *   GET /api/public/events       recent safety events, field-whitelisted
 *   GET /api/public/reports      recent analyst report summaries
 *   GET /api/public/camera       latest frame — OFF unless explicitly enabled
 *
 * DESIGN RULES FOR ANYTHING ADDED HERE
 * ------------------------------------
 * 1. Whitelist fields, never blacklist. Telemetry payloads carry base64 frames
 *    and LiDAR arrays; echoing `data` wholesale would publish them by accident.
 *    Every field a viewer sees is named explicitly in PUBLIC_EVENT_FIELDS.
 * 2. Never expose hostnames, IPs, ports, tokens, emails or file paths.
 * 3. Whitelisting applies to GENERATED TEXT too, not just to payload fields.
 *    Agent findings and model output are free-form prose: they interpolate
 *    detection labels, wildlife species and obstacle distances into a sentence.
 *    A sentence has no schema, so there is nothing to whitelist inside it — the
 *    only safe public projection of one is a fixed phrase chosen from a table
 *    this file owns. See publicReports, which used to break this rule.
 * 4. Assume this URL gets scraped, and be precise about what actually protects
 *    it. Three things do, and each covers a different failure:
 *      - Workers Caching, enabled by `"cache": { "enabled": true }` in
 *        wrangler.jsonc. On a HIT, Cloudflare returns the response WITHOUT
 *        running this Worker at all, so a burst costs no D1 reads, no Durable
 *        Object calls and no CPU. It also collapses simultaneous misses for the
 *        same key into a single invocation. It works on workers.dev: the cache
 *        belongs to the Worker, not to a zone.
 *      - Every query below is LIMIT-bounded AND time-bounded, so even a stream
 *        of genuine misses cannot walk the whole `readings` table.
 *      - A per-IP burst limiter (see scrapeRetryAfter) for the one case caching
 *        does not cover: a scraper that appends a cache-busting query string
 *        mints a fresh cache key every request and so always reaches the Worker.
 *
 *    What does NOT protect it — this block previously claimed otherwise, and the
 *    claim was false in both directions, so state it plainly:
 *      - Caching was never actually enabled. It is now.
 *      - Caching does not protect the free plan's 100k requests/day ceiling.
 *        Cloudflare bills a cache HIT at the same per-request rate as a miss; a
 *        hit only skips the CPU. Caching stops the *database* bill, not the
 *        request count. Nothing in a Worker can stop the request count, because
 *        even a 429 is a billed request — that ceiling is a plan decision, not
 *        a code decision.
 *      - `caches.default` (the Cache API) is deliberately unused here. It is
 *        documented as functional on custom domains and pages.dev; on a
 *        workers.dev subdomain its put/match calls silently do nothing, so
 *        building the defence on it would look right and protect nothing.
 *
 * The camera feed is gated behind FPMS_PUBLIC_CAMERA because a fire-watch
 * camera also points at a home, and the operator's own label file maps class 0
 * to a person's name. Publishing a live view of that to the open internet is a
 * decision for the operator to make explicitly, not a default.
 *   npx wrangler secret put FPMS_PUBLIC_CAMERA   # value: on
 */

/** Seconds before a rover that has stopped reporting is no longer "live". */
const ONLINE_WINDOW_S = 90;
/** Seconds before it is considered properly offline rather than just lagging. */
const STALE_WINDOW_S = 3600;

/** Edge cache lifetime. Long enough to absorb a spike, short enough to feel live. */
const PUBLIC_CACHE_S = 10;

/**
 * How far back the "is anything on fire?" lookup may search.
 *
 * A fire event older than a day is history, not a live alarm, and the banner it
 * would raise would be wrong. Bounding it is also what stops the query being
 * unbounded work — see publicSummary.
 */
const FIRE_LOOKBACK_S = 24 * 3600;

/**
 * Per-IP burst guard for /api/public/*.
 *
 * WHY IN-ISOLATE AND NOT IN THE DURABLE OBJECT
 * --------------------------------------------
 * A DO-backed counter would be globally accurate, but it costs a subrequest and
 * a DO invocation on EVERY request — i.e. it spends the exact resource it is
 * meant to conserve, and it would run on requests that currently touch no
 * storage at all. A module-scope Map lives in the isolate, costs nothing, and
 * needs no binding.
 *
 * THE TRADEOFF, STATED HONESTLY
 * -----------------------------
 * Counters are per isolate, so they are not a global rate limit. A client whose
 * requests land in several colos (or several isolates in one colo) gets that
 * multiple of the allowance, and an isolate eviction resets the window. What it
 * does reliably catch is the realistic threat: one script hammering one URL from
 * one place, which otherwise sustains 10 req/s indefinitely.
 *
 * It also only ever sees traffic that reaches the Worker. Repeat hits on the
 * same URL are absorbed by Workers Caching before this code runs, so in practice
 * this limiter exists for cache-busting scrapers — the only ones that cost
 * anything beyond the request itself.
 *
 * 30 requests / 10s is roughly 60x what the page's own 30s poll needs, so a
 * human with several tabs open, or a shared NAT egress, will not trip it.
 */
const SCRAPE_WINDOW_MS = 10_000;
const SCRAPE_BURST = 30;
const SCRAPE_CLIENTS_MAX = 1000;

/** ip -> { n, reset_at }. Module scope: one table per isolate, no storage. */
const scrapeWindows = new Map();

/**
 * Returns 0 when the request is within budget, otherwise the seconds a caller
 * should wait (the Retry-After value).
 */
function scrapeRetryAfter(request) {
  // CF-Connecting-IP is set by the edge and cannot be spoofed by the client,
  // unlike X-Forwarded-For. Missing only in local dev, where "unknown" is fine.
  const ip = request.headers.get("CF-Connecting-IP") || "unknown";
  const now = Date.now();

  let win = scrapeWindows.get(ip);
  if (!win || now >= win.reset_at) {
    win = { n: 0, reset_at: now + SCRAPE_WINDOW_MS };
    scrapeWindows.set(ip, win);
  }
  win.n++;

  // Bounded memory. Sweep expired windows first; if the table is still oversized
  // (a distributed scan hitting one isolate from many addresses) drop it whole.
  // Losing the counters fails OPEN for one window, which is the right direction
  // for a public status page: an unbounded Map in a long-lived isolate is a
  // worse outcome than a scraper getting one free window.
  if (scrapeWindows.size > SCRAPE_CLIENTS_MAX) {
    for (const [k, v] of scrapeWindows) if (now >= v.reset_at) scrapeWindows.delete(k);
    if (scrapeWindows.size > SCRAPE_CLIENTS_MAX) {
      scrapeWindows.clear();
      scrapeWindows.set(ip, win);
    }
  }

  return win.n > SCRAPE_BURST ? Math.max(1, Math.ceil((win.reset_at - now) / 1000)) : 0;
}

/**
 * The only event fields an anonymous viewer ever sees. Anything not listed is
 * dropped — including `frame`, `ranges_m`, `ip` and any field added later.
 */
const PUBLIC_EVENT_FIELDS = [
  "type", "severity", "label", "kind", "conf", "ratio", "alert", "component",
];

/** Event subtypes safe to surface publicly. */
const PUBLIC_EVENT_TYPES = new Set([
  "fire", "fire_cleared", "obstacle", "wildlife", "fault",
  "camera_recovered", "hazard",
]);

/**
 * The complete set of headlines /api/public/reports can ever emit, keyed by the
 * report's severity. Nothing outside this table reaches a viewer.
 *
 * WHY A FIXED TABLE INSTEAD OF THE STORED `summary`
 * -------------------------------------------------
 * `reports.summary` is written by runAgents as the first critical/warning
 * finding's `.detail`, and perRoverAgent builds that detail by interpolating
 * whatever the fleet saw: YOLO detection labels, wildlife species, nearest
 * obstacle distances, per-rover names. Today it usually reads "rover2
 * [critical] no telemetry at all", which is harmless — but that is a property of
 * the rovers being down, not a property of the code. With a live camera the same
 * field publishes what the camera saw, to anonymous viewers, with no auth.
 *
 * The header of this file gates the camera feed precisely because the operator's
 * own label file maps class 0 to a person's name. Publishing the detector's
 * *words* while withholding its *pixels* leaks the same fact and defeats that
 * gate. Design rule 3: prose has no schema, so it cannot be whitelisted
 * field-by-field — only replaced by a phrase this file chose in advance.
 *
 * Anyone entitled to the real findings reads them through the authenticated API.
 */
const PUBLIC_REPORT_HEADLINE = {
  ok: "All monitored systems nominal.",
  warning: "A condition needs attention.",
  critical: "A critical condition was detected.",
};

/** Severity itself is whitelisted: an unrecognised value must not pass through. */
const PUBLIC_REPORT_UNKNOWN = { severity: "unknown", summary: "Status reported." };

function publicJson(obj, status = 200, cacheControl = `public, max-age=${PUBLIC_CACHE_S}`) {
  return new Response(JSON.stringify(obj), {
    status,
    headers: {
      "Content-Type": "application/json",
      // Public and cacheable, unlike the authenticated API which is no-store.
      "Cache-Control": cacheControl,
      // A public read-only feed is meant to be embedded and scraped.
      "Access-Control-Allow-Origin": "*",
      // Defence in depth: these responses are pure data, never markup.
      "X-Content-Type-Options": "nosniff",
    },
  });
}

/** Keep only whitelisted keys, and only scalars — never nested objects. */
function scrub(data) {
  const out = {};
  if (!data || typeof data !== "object") return out;
  for (const key of PUBLIC_EVENT_FIELDS) {
    const v = data[key];
    if (v === undefined || v === null) continue;
    if (typeof v === "object") continue;
    out[key] = v;
  }
  return out;
}

/**
 * Timestamps are stored in unix SECONDS everywhere in this system — see the
 * schema comment on `readings.ts` and `Date.now() / 1000` in the hub's publish
 * path. Browsers want milliseconds. Normalise once, here, so nothing
 * downstream has to remember which unit it is holding.
 *
 * The magnitude check is deliberate: a rover that ever sends milliseconds
 * would otherwise be reported as permanently offline (a seconds value compared
 * against a millisecond clock is ~55 years in the past). 1e12 is about
 * 2001 in ms and year 33658 in seconds, so it separates the two cleanly.
 */
function toMs(ts) {
  const n = Number(ts);
  if (!n || !Number.isFinite(n)) return null;
  return n > 1e12 ? n : n * 1000;
}

function liveness(lastSeenMs, nowMs) {
  if (!lastSeenMs) return "offline";
  const age = (nowMs - lastSeenMs) / 1000;
  if (age <= ONLINE_WINDOW_S) return "online";
  if (age <= STALE_WINDOW_S) return "stale";
  return "offline";
}

/**
 * Fleet status.
 *
 * The roster and its freshness come entirely from the Durable Object, which is
 * the process that receives the telemetry and therefore already knows both.
 * Every step degrades rather than fails: an unreachable DO leaves an empty
 * roster, a DO without the per-thing `last_seen` map falls back to the fleet-
 * wide timestamp, and a failed fire lookup reports "no fire" rather than
 * guessing. The page still says something useful when every rover is down —
 * which is exactly when someone checks it.
 *
 * D1 is touched once here, for the fire banner, and that query is time-bounded.
 */
async function publicSummary(env, hubStub) {
  const now = Date.now();
  let snap = null;

  try {
    snap = await hubStub(env).fetch("https://hub/snapshot").then((r) => r.json());
  } catch (err) {
    // A cold or unreachable DO must not fail the whole page.
    console.error(JSON.stringify({
      message: "public summary: hub snapshot failed",
      error: err instanceof Error ? err.message : String(err),
    }));
  }

  const things = new Map();

  /** Only fills a gap — never overwrites a precise time with an approximate one. */
  const noteApprox = (name, ms) => {
    if (name && !things.has(name)) things.set(name, { thing: name, last_seen: ms });
  };

  // Per-thing freshness, straight from the hub. `last_seen` maps a thing to the
  // unix SECONDS of its most recent publish.
  //
  // WHY THIS IS NOT A D1 QUERY ANY MORE
  // -----------------------------------
  // This used to be `SELECT thing, MAX(ts) FROM readings GROUP BY thing`. That
  // reads worse than it looks: `ts` is the THIRD column of idx_readings_lookup
  // (thing, subtype, ts DESC), so SQLite cannot skip to the newest row per
  // thing — the leading columns it can seek on are thing and subtype, and the
  // max it wants lives past both. It must therefore scan every group in full.
  // On a table that grows by ~35k rows/day, that is a whole-table read on EVERY
  // public page poll, from an endpoint with no auth in front of it. It was the
  // single largest consumer of the D1 read quota.
  //
  // The hub already has this map in memory and is the authority on it; asking
  // the archive to recompute what the live process already knows was the bug.
  const lastSeen = snap?.last_seen;
  if (lastSeen && typeof lastSeen === "object") {
    for (const [name, ts] of Object.entries(lastSeen)) {
      const ms = toMs(ts);
      if (name && ms) things.set(name, { thing: name, last_seen: ms });
    }
  }

  // Graceful path for a deployment that predates `last_seen`, or a cold DO that
  // has not rebuilt it yet. Names still come from the roster, and the hub's
  // single fleet-wide `last_message_at` is the only freshness signal left.
  //
  // It is deliberately used only as a fallback: being fleet-wide, it can make a
  // silent rover look alive. Listing a rover with an approximate time still
  // beats omitting it, because a page that has quietly dropped a rover answers
  // "is anything on fire?" with silence rather than with a warning.
  const snapSeen = toMs(snap?.last_message_at);
  // Channels look like "camera:rover2" — the thing is the half after the colon.
  for (const ch of (snap?.channels || [])) noteApprox(String(ch).split(":")[1], snapSeen);
  for (const name of (snap?.things_seen || [])) noteApprox(String(name), snapSeen);

  const rovers = [...things.values()]
    .map((t) => ({
      thing: t.thing,
      status: liveness(t.last_seen, now),
      last_seen: t.last_seen || null,
    }))
    .sort((a, b) => a.thing.localeCompare(b.thing));

  // The most recent unresolved fire event, if any.
  //
  // The `ts >= ?` floor is not cosmetic. idx_readings_events is (kind, ts DESC),
  // so `subtype IN (...)` is not indexed — it is a filter applied to rows the
  // engine has already read. Walking kind='events' newest-first therefore reads
  // every event row until it finds a fire one, and when there is no fire event
  // at all (the normal, permanent case) it reads all of them. LIMIT 1 caps what
  // comes back, not what gets scanned. The floor turns that into a bounded range
  // scan: at worst one day of events, and it stops at the first row older than
  // the cutoff.
  let fire = { active: false, since: null };
  if (env.DB) {
    try {
      const row = await env.DB
        .prepare(
          "SELECT ts, subtype FROM readings WHERE kind = 'events' AND ts >= ? " +
          "AND subtype IN ('fire','fire_cleared') ORDER BY ts DESC LIMIT 1",
        )
        // Stored in unix seconds, same unit as the rovers send.
        .bind(now / 1000 - FIRE_LOOKBACK_S)
        .first();
      if (row?.subtype === "fire") fire = { active: true, since: toMs(row.ts) };
    } catch {
      // Leave fire as inactive rather than guessing.
    }
  }

  const online = rovers.filter((r) => r.status === "online").length;

  return publicJson({
    generated_at: now,
    system: "FPMS — Fire Prevention & Monitoring System",
    status: online > 0 ? "operational" : (rovers.length ? "no rovers reporting" : "no data"),
    rovers,
    counts: {
      rovers_total: rovers.length,
      rovers_online: online,
      messages_seen: snap?.messages_seen ?? null,
    },
    fire,
    camera_public: env.FPMS_PUBLIC_CAMERA === "on",
  });
}

/** Recent safety events, field-whitelisted. */
async function publicEvents(env, url) {
  if (!env.DB) return publicJson({ events: [] });
  const limit = Math.min(Math.max(Number(url.searchParams.get("limit") || 20), 1), 50);
  try {
    const { results } = await env.DB
      .prepare(
        "SELECT ts, thing, subtype, data FROM readings WHERE kind = 'events' " +
        "ORDER BY ts DESC LIMIT ?",
      )
      .bind(limit)
      .all();

    const events = [];
    for (const r of results || []) {
      if (!PUBLIC_EVENT_TYPES.has(r.subtype)) continue;
      let parsed = null;
      try {
        parsed = JSON.parse(r.data);
      } catch {
        parsed = null;
      }
      events.push({
        ts: toMs(r.ts), thing: r.thing, subtype: r.subtype, data: scrub(parsed),
      });
    }
    return publicJson({ events });
  } catch (err) {
    console.error(JSON.stringify({
      message: "public events query failed",
      error: err instanceof Error ? err.message : String(err),
    }));
    return publicJson({ events: [] });
  }
}

/**
 * Analyst report headlines — when a report was produced and how bad it was.
 *
 * Deliberately NOT the stored `summary`. That column is agent prose and cannot
 * be published; see PUBLIC_REPORT_HEADLINE for the full reasoning. `severity` is
 * a closed vocabulary (ok | warning | critical) and safe on its own, so the
 * public shape is severity plus the fixed phrase that severity maps to.
 *
 * `summary` is still the field name so existing consumers of this feed keep
 * working — it is now a derived label, not the analyst's sentence.
 *
 * Note the SELECT does not even read the column. A field that is never fetched
 * cannot be leaked by a later refactor that forgets why it was dropped.
 */
async function publicReports(env, url) {
  if (!env.DB) return publicJson({ reports: [] });
  const limit = Math.min(Math.max(Number(url.searchParams.get("limit") || 5), 1), 20);
  try {
    const { results } = await env.DB
      .prepare("SELECT ts, severity FROM reports ORDER BY ts DESC LIMIT ?")
      .bind(limit)
      .all();
    return publicJson({
      reports: (results || []).map((r) => {
        const headline = PUBLIC_REPORT_HEADLINE[r.severity];
        // Unrecognised severity falls back to the placeholder rather than
        // echoing the value: whitelist, never blacklist (design rule 1).
        return headline
          ? { ts: toMs(r.ts), severity: r.severity, summary: headline }
          : { ts: toMs(r.ts), ...PUBLIC_REPORT_UNKNOWN };
      }),
    });
  } catch (err) {
    console.error(JSON.stringify({
      message: "public reports query failed",
      error: err instanceof Error ? err.message : String(err),
    }));
    return publicJson({ reports: [] });
  }
}

/**
 * Latest camera frame. Disabled unless the operator opts in — see the note at
 * the top of this file.
 */
async function publicCamera(env, url, hubStub) {
  if (env.FPMS_PUBLIC_CAMERA !== "on") {
    return publicJson({
      error: "camera feed is not public",
      hint: "operator must set FPMS_PUBLIC_CAMERA=on",
    }, 403);
  }
  const thing = String(url.searchParams.get("thing") || "rover2");
  if (!/^[a-z0-9_.-]+$/i.test(thing)) {
    return publicJson({ error: "invalid thing" }, 400);
  }
  try {
    const latest = await hubStub(env)
      .fetch(`https://hub/latest?channel=camera:${encodeURIComponent(thing)}`)
      .then((r) => r.json());
    const frame = latest?.data?.frame;
    if (!frame) return publicJson({ error: "no frame available" }, 404);
    return publicJson({ thing, ts: toMs(latest?.ts), frame });
  } catch (err) {
    console.error(JSON.stringify({
      message: "public camera fetch failed",
      error: err instanceof Error ? err.message : String(err),
    }));
    return publicJson({ error: "no frame available" }, 404);
  }
}

/**
 * Everything the public page needs, in one response.
 *
 * Each part is independently fault-tolerant: if the archive query fails, the
 * page still gets fleet status rather than nothing. The individual endpoints
 * remain available for anyone consuming the feed programmatically.
 */
async function publicAll(env, url, hubStub) {
  const readJson = async (p) => {
    try {
      return await (await p).json();
    } catch {
      return null;
    }
  };

  const [summary, events, reports] = await Promise.all([
    readJson(publicSummary(env, hubStub)),
    readJson(publicEvents(env, url)),
    readJson(publicReports(env, url)),
  ]);

  return publicJson({
    ...(summary || { status: "unavailable", rovers: [], counts: {} }),
    events: events?.events || [],
    reports: reports?.reports || [],
  });
}

/**
 * Route dispatcher. Returns a Response for public paths, or null so worker.js
 * can carry on to its authenticated routing.
 */
export async function handlePublic(request, env, url, path, hubStub) {
  if (path === "/live" || path === "/live/") {
    return new Response(PUBLIC_PAGE, {
      headers: {
        "Content-Type": "text/html; charset=utf-8",
        "Cache-Control": `public, max-age=${PUBLIC_CACHE_S}`,
        "X-Content-Type-Options": "nosniff",
        // The page is self-contained; nothing external may run in it.
        "Content-Security-Policy":
          "default-src 'none'; style-src 'unsafe-inline'; script-src 'unsafe-inline'; " +
          "img-src 'self' data:; connect-src 'self'; base-uri 'none'; form-action 'none'",
        "Referrer-Policy": "no-referrer",
      },
    });
  }

  if (!path.startsWith("/api/public/")) return null;
  if (request.method !== "GET") return publicJson({ error: "method not allowed" }, 405);

  // Burst guard. Cheap by construction: a Map lookup, no storage, no DO call,
  // no binding. See scrapeRetryAfter for what it does and does not cover.
  //
  // The 429 is explicitly no-store. Everything else here is `public, max-age=10`
  // and Workers Caching honours that, so a cacheable rejection would be handed
  // to every other viewer arriving at the same colo for the next ten seconds —
  // one scraper would take the status page down for the neighbourhood, which is
  // the outcome this endpoint exists to prevent.
  const retryAfter = scrapeRetryAfter(request);
  if (retryAfter) {
    const res = publicJson(
      { error: "rate limited", hint: `retry in ${retryAfter}s` },
      429,
      "no-store",
    );
    res.headers.set("Retry-After", String(retryAfter));
    return res;
  }

  // One call for everything the page renders.
  //
  // This exists for a availability reason, not tidiness. The page used to fetch
  // summary + events + reports separately on every tick; at a 10s interval that
  // is 18 requests/minute per open tab, or ~26k/day from ONE tab left open
  // against a 100k/day free-tier ceiling. A handful of idle tabs would take the
  // dashboard down. Combining them cuts that by 3x, and the page's polling
  // changes cut it by another ~10x.
  if (path === "/api/public/all") return publicAll(env, url, hubStub);

  if (path === "/api/public/summary") return publicSummary(env, hubStub);
  if (path === "/api/public/events") return publicEvents(env, url);
  if (path === "/api/public/reports") return publicReports(env, url);
  if (path === "/api/public/camera") return publicCamera(env, url, hubStub);

  return publicJson({ error: "not found" }, 404);
}

/* --------------------------------------------------------------- the page */

// Self-contained on purpose: no CDN, no build step, no dependency on the React
// bundle. If the SPA fails to build or its assets go missing, this page still
// answers "is anything on fire?" — which is the one question that matters.
const PUBLIC_PAGE = `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>FPMS — Live Status</title>
<meta name="description" content="Public live status for the FPMS fire prevention and monitoring system.">
<style>
  :root {
    --bg:#0b0f14; --panel:#141b24; --line:#243040; --fg:#e6edf3; --dim:#8b9cb0;
    --ok:#3fb950; --warn:#d29922; --bad:#f85149; --idle:#6e7681; --accent:#58a6ff;
  }
  @media (prefers-color-scheme: light) {
    :root {
      --bg:#f6f8fa; --panel:#ffffff; --line:#d8dee4; --fg:#1f2328; --dim:#59636e;
      --ok:#1a7f37; --warn:#9a6700; --bad:#cf222e; --idle:#818b98; --accent:#0969da;
    }
  }
  * { box-sizing:border-box; }
  body {
    margin:0; background:var(--bg); color:var(--fg);
    font:15px/1.5 ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif;
    padding:24px 16px 48px;
  }
  .wrap { max-width:920px; margin:0 auto; }
  header { margin-bottom:20px; }
  h1 { font-size:20px; margin:0 0 4px; letter-spacing:-0.01em; }
  .sub { color:var(--dim); font-size:13px; }
  .banner {
    border:1px solid var(--line); border-left-width:4px; border-radius:8px;
    background:var(--panel); padding:16px 18px; margin:18px 0;
  }
  .banner h2 { margin:0 0 4px; font-size:17px; }
  .banner p  { margin:0; color:var(--dim); font-size:13px; }
  .b-ok   { border-left-color:var(--ok); }
  .b-warn { border-left-color:var(--warn); }
  .b-bad  { border-left-color:var(--bad); }
  .b-idle { border-left-color:var(--idle); }
  .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(200px,1fr)); gap:12px; }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:14px; }
  .card .name { font-weight:600; margin-bottom:6px; }
  .pill {
    display:inline-block; font-size:11px; font-weight:600; text-transform:uppercase;
    letter-spacing:.04em; padding:2px 8px; border-radius:99px; color:#fff;
  }
  .p-online{background:var(--ok);} .p-stale{background:var(--warn);}
  .p-offline{background:var(--idle);} .p-bad{background:var(--bad);}
  h3 { font-size:13px; text-transform:uppercase; letter-spacing:.05em;
       color:var(--dim); margin:28px 0 10px; font-weight:600; }
  table { width:100%; border-collapse:collapse; font-size:13px; }
  .scroll { overflow-x:auto; border:1px solid var(--line); border-radius:8px; background:var(--panel); }
  th,td { text-align:left; padding:9px 12px; border-bottom:1px solid var(--line); white-space:nowrap; }
  th { color:var(--dim); font-weight:600; font-size:11px; text-transform:uppercase; letter-spacing:.04em; }
  tr:last-child td { border-bottom:none; }
  .empty { color:var(--dim); font-size:13px; padding:14px; }
  footer { margin-top:36px; color:var(--dim); font-size:12px;
           border-top:1px solid var(--line); padding-top:14px; }
  code { background:var(--bg); border:1px solid var(--line); border-radius:4px; padding:1px 5px; font-size:12px; }
  #cam { max-width:100%; border-radius:8px; border:1px solid var(--line); display:block; }
</style>
</head>
<body>
<div class="wrap">
  <header>
    <h1>FPMS — Fire Prevention &amp; Monitoring System</h1>
    <div class="sub">Public live status &middot; read-only &middot; <span id="clock">connecting&hellip;</span></div>
  </header>

  <div id="banner" class="banner b-idle">
    <h2 id="b-title">Loading&hellip;</h2>
    <p id="b-note">Fetching current fleet status.</p>
  </div>

  <h3>Rovers</h3>
  <div id="rovers" class="grid"><div class="empty">Loading&hellip;</div></div>

  <div id="cam-wrap" style="display:none">
    <h3>Camera</h3>
    <img id="cam" alt="Latest camera frame from the rover">
  </div>

  <h3>Recent events</h3>
  <div class="scroll"><table>
    <thead><tr><th>Time</th><th>Rover</th><th>Event</th><th>Detail</th></tr></thead>
    <tbody id="events"><tr><td colspan="4" class="empty">Loading&hellip;</td></tr></tbody>
  </table></div>

  <h3>Analysis</h3>
  <div class="scroll"><table>
    <thead><tr><th>Time</th><th>Severity</th><th>Summary</th></tr></thead>
    <tbody id="reports"><tr><td colspan="3" class="empty">Loading&hellip;</td></tr></tbody>
  </table></div>

  <footer>
    This is a read-only public view. Rover control, terminal access and live
    telemetry require authentication and are not reachable from here.
    Machine-readable: <code>/api/public/summary</code>, <code>/api/public/events</code>.
  </footer>
</div>

<script>
(function () {
  var $ = function (id) { return document.getElementById(id); };

  function esc(s) {
    return String(s == null ? "" : s).replace(/[&<>"']/g, function (c) {
      return { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c];
    });
  }

  function ago(ts) {
    if (!ts) return "never";
    var s = Math.max(0, Math.floor((Date.now() - ts) / 1000));
    if (s < 60) return s + "s ago";
    if (s < 3600) return Math.floor(s / 60) + "m ago";
    if (s < 86400) return Math.floor(s / 3600) + "h ago";
    return Math.floor(s / 86400) + "d ago";
  }

  function timeOf(ts) {
    if (!ts) return "-";
    try { return new Date(ts).toLocaleString(); } catch (e) { return String(ts); }
  }

  function getJSON(path) {
    return fetch(path, { headers: { Accept: "application/json" } })
      .then(function (r) { return r.ok ? r.json() : null; })
      .catch(function () { return null; });
  }

  function renderSummary(d) {
    if (!d) {
      $("banner").className = "banner b-bad";
      $("b-title").textContent = "Status unavailable";
      $("b-note").textContent = "Could not reach the monitoring service.";
      return;
    }

    $("clock").textContent = "updated " + new Date(d.generated_at).toLocaleTimeString();

    var online = (d.counts && d.counts.rovers_online) || 0;
    var total = (d.counts && d.counts.rovers_total) || 0;

    if (d.fire && d.fire.active) {
      $("banner").className = "banner b-bad";
      $("b-title").textContent = "FIRE DETECTED";
      $("b-note").textContent = "Active since " + timeOf(d.fire.since) + ".";
    } else if (online > 0) {
      $("banner").className = "banner b-ok";
      $("b-title").textContent = "All clear";
      $("b-note").textContent = online + " of " + total + " rover(s) reporting. No active fire detection.";
    } else if (total > 0) {
      $("banner").className = "banner b-warn";
      $("b-title").textContent = "No rovers reporting";
      $("b-note").textContent =
        "The service is up, but no rover has sent telemetry recently. " +
        "The most recent data is shown below.";
    } else {
      $("banner").className = "banner b-idle";
      $("b-title").textContent = "Awaiting first telemetry";
      $("b-note").textContent = "The service is running. No rover has reported yet.";
    }

    var rovers = d.rovers || [];
    if (!rovers.length) {
      $("rovers").innerHTML = '<div class="empty">No rovers known yet.</div>';
    } else {
      $("rovers").innerHTML = rovers.map(function (r) {
        return '<div class="card"><div class="name">' + esc(r.thing) + "</div>" +
          '<span class="pill p-' + esc(r.status) + '">' + esc(r.status) + "</span>" +
          '<div class="sub" style="margin-top:8px">last seen ' + esc(ago(r.last_seen)) + "</div></div>";
      }).join("");
    }

    if (d.camera_public) {
      $("cam-wrap").style.display = "";
      getJSON("/api/public/camera").then(function (c) {
        if (c && c.frame) $("cam").src = "data:image/jpeg;base64," + c.frame;
      });
    }
  }

  function renderEvents(d) {
    var rows = (d && d.events) || [];
    if (!rows.length) {
      $("events").innerHTML = '<tr><td colspan="4" class="empty">No events recorded.</td></tr>';
      return;
    }
    $("events").innerHTML = rows.map(function (e) {
      var bits = [];
      for (var k in e.data) { if (e.data[k] !== "") bits.push(k + "=" + e.data[k]); }
      return "<tr><td>" + esc(timeOf(e.ts)) + "</td><td>" + esc(e.thing) +
        "</td><td>" + esc(e.subtype) + "</td><td>" + esc(bits.join(", ") || "-") + "</td></tr>";
    }).join("");
  }

  function renderReports(d) {
    var rows = (d && d.reports) || [];
    if (!rows.length) {
      $("reports").innerHTML = '<tr><td colspan="3" class="empty">No analysis reports yet.</td></tr>';
      return;
    }
    $("reports").innerHTML = rows.map(function (r) {
      return "<tr><td>" + esc(timeOf(r.ts)) + "</td><td>" + esc(r.severity) +
        "</td><td>" + esc(r.summary) + "</td></tr>";
    }).join("");
  }

  // Polling is deliberately conservative. This page is public and may sit open
  // on a wall display for days; at 3 requests every 10s a single forgotten tab
  // would burn ~26k requests/day against a 100k/day ceiling and eventually take
  // the dashboard down for everyone. One combined request every 30s, paused
  // entirely while the tab is hidden, is roughly 1/20th of that.
  var PERIOD_MS = 30000;
  var timer = null;
  var failures = 0;

  function render(d) {
    renderSummary(d);
    renderEvents(d);
    renderReports(d);
  }

  function refresh() {
    return getJSON("/api/public/all?limit=20").then(function (d) {
      if (d) {
        failures = 0;
        render(d);
      } else {
        // Back off on repeated failure rather than hammering a struggling
        // origin — but never slower than 5 minutes, so it recovers on its own.
        failures = Math.min(failures + 1, 4);
      }
      return d;
    });
  }

  function schedule() {
    if (timer) clearTimeout(timer);
    if (document.hidden) return;              // nobody is looking; stop asking
    var wait = PERIOD_MS * Math.pow(2, failures);
    timer = setTimeout(function () { refresh().then(schedule); }, Math.min(wait, 300000));
  }

  document.addEventListener("visibilitychange", function () {
    if (document.hidden) {
      if (timer) clearTimeout(timer);
      timer = null;
    } else {
      // Refresh immediately on return so the page is never showing stale data
      // to someone actually looking at it.
      refresh().then(schedule);
    }
  });

  refresh().then(schedule);
})();
</script>
</body>
</html>`;

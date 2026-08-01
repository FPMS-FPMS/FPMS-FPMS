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
 *   GET /api/public/all          everything the page renders, in one response
 *   GET /api/public/summary      fleet status, rolled up
 *   GET /api/public/events       recent safety events, field-whitelisted
 *   GET /api/public/reports      recent analyst report headlines
 *   GET /api/public/camera       latest frame — OFF unless explicitly enabled
 *
 * WHO THE AUDIENCE IS, AND WHAT THAT DEMANDS
 * ------------------------------------------
 * Judges, teachers and passers-by, on phones, worldwide, with no context and no
 * login. Three consequences run through every decision below:
 *
 *   1. A first-time visitor must understand what they are looking at in
 *      seconds. That is why the API emits a plain-English `headline` per event
 *      and a machine `state` per summary — the page never has to invent prose,
 *      and it never has to render a raw key=value dump at a human.
 *   2. The rover is OFF most of the time. Offline is the NORMAL case, not the
 *      error case, and the page must not look broken in it. Everything here
 *      carries an honest timestamp so the page can say "last seen 3 hours ago"
 *      instead of implying live data that does not exist.
 *   3. It has to be cheap and it has to survive being linked from a scoreboard.
 *      See the caching notes below.
 *
 * DESIGN RULES FOR ANYTHING ADDED HERE
 * ------------------------------------
 * 1. Whitelist fields, never blacklist. Telemetry payloads carry base64 frames
 *    and LiDAR arrays; echoing `data` wholesale would publish them by accident.
 *    Every field a viewer sees is named explicitly in PUBLIC_EVENT_DATA.
 * 2. Never expose hostnames, IPs, ports, tokens, emails or file paths.
 * 3. Whitelisting applies to GENERATED TEXT too, not just to payload fields.
 *    Agent findings and model output are free-form prose: they interpolate
 *    detection labels, wildlife species and obstacle distances into a sentence.
 *    A sentence has no schema, so there is nothing to whitelist inside it — the
 *    only safe public projection of one is a fixed phrase chosen from a table
 *    this file owns. See PUBLIC_REPORT_HEADLINE, which publicReports used to
 *    break, and PUBLIC_EVENT_HEADLINE, which applies the same rule to events.
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

// Page fragments only. The analytics/demo ROUTE is registered in worker.js
// ahead of handlePublic, because handlePublic owns the /api/public/* prefix
// and answers 405 to every non-GET, so a route added after it is
// unreachable. Importing handleAnalytics here too would be a second, dead
// registration of the same handler.
import {
  ANALYTICS_STYLE,
  ANALYTICS_HTML,
  ANALYTICS_SCRIPT,
} from "./analytics.js";


/** Seconds before a rover that has stopped reporting is no longer "live". */
const ONLINE_WINDOW_S = 90;
/** Seconds before it is considered properly offline rather than just lagging. */
const STALE_WINDOW_S = 3600;

/**
 * Edge cache lifetimes.
 *
 * TWO SEPARATE BUDGETS, AND WHY
 * -----------------------------
 * The page and the data have completely different change rates, so they get
 * completely different TTLs:
 *
 *   - PUBLIC_PAGE is a constant in this file. It changes only when this Worker
 *     is redeployed, and the deployed Worker VERSION is part of the Workers
 *     Cache key by default (`cross_version_cache` is not enabled in
 *     wrangler.jsonc), so a deploy starts from a cold cache and cannot serve
 *     the old page. That makes a long TTL safe: there is no "stale HTML after
 *     deploy" failure mode to protect against.
 *   - The API responses change as fast as the rovers report, so they stay
 *     short — but with a stale window in front, which is what actually absorbs
 *     a spike.
 *
 * WHY stale-while-revalidate AND NOT s-maxage
 * -------------------------------------------
 * `s-maxage`, `must-revalidate` and `proxy-revalidate` all DISABLE
 * stale-while-revalidate and stale-if-error (RFC 9111 4.2.4, and Cloudflare
 * implements it that way). So the edge freshness window has to be plain
 * `max-age`. Do not add `s-maxage` here to "tune the edge separately" — it
 * silently turns both stale behaviours off and every expiry becomes a blocking
 * revalidation again.
 *
 * What the stale window buys, concretely:
 *   - stale-while-revalidate: when the entry expires, the next viewer gets the
 *     cached body IMMEDIATELY and the refresh runs in the background. Nobody
 *     ever waits on D1. A crowd arriving at once (the realistic spike: a link
 *     read out at a competition) is served entirely from cache while exactly
 *     one background invocation refreshes it.
 *   - stale-if-error: if this Worker throws or times out while refreshing, the
 *     last good response is served instead of a 5xx. A public status page that
 *     shows honestly-dated old data is doing its job; one that shows an error
 *     page looks broken, which is the single outcome this page must avoid.
 *     It is bounded rather than left to the default (serve stale forever), so a
 *     genuinely dead backend eventually surfaces instead of being masked.
 */
const PUBLIC_CACHE_S = 15;
const PUBLIC_SWR_S = 60;
const PUBLIC_SIE_S = 6 * 3600;
const DATA_CACHE_CONTROL =
  `public, max-age=${PUBLIC_CACHE_S}, stale-while-revalidate=${PUBLIC_SWR_S}, ` +
  `stale-if-error=${PUBLIC_SIE_S}`;

/** The page itself: immutable between deploys, so cache it hard. */
const PAGE_CACHE_CONTROL =
  "public, max-age=120, stale-while-revalidate=600, stale-if-error=86400";

/**
 * The "camera is not public" refusal is a constant, and the commonest reply
 * this endpoint gives. Caching it for an hour means a scraper polling for a
 * frame that will never come costs one Worker invocation per hour, not one per
 * request.
 */
const DENY_CACHE_CONTROL = "public, max-age=3600, stale-while-revalidate=3600";

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
 *
 * This is request-scoped counting in module scope, which is normally the bug
 * ("no global request state"). It is deliberate and safe here for one reason:
 * nothing in the table is derived from a request BODY or identity beyond the
 * edge-supplied IP, and a wrong answer only ever costs one extra window. No
 * viewer can be shown another viewer's data through it.
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

/* ------------------------------------------------------- public projections */

/** Event subtypes safe to surface publicly. Anything else is dropped whole. */
const PUBLIC_EVENT_TYPES = new Set([
  "fire", "fire_cleared", "obstacle", "obstacle_cleared", "wildlife", "fault",
  "camera_recovered", "hazard",
]);

/**
 * The only event payload fields an anonymous viewer ever sees, each with the
 * validator that normalises it. A field absent from this table is dropped —
 * including `frame`, `ranges_m`, `ip`, and any field added later.
 *
 * WHY THIS IS A TABLE OF VALIDATORS AND NOT A LIST OF NAMES
 * ---------------------------------------------------------
 * The previous version was a name list plus a "scalars only" filter. Two things
 * were wrong with it, and both were invisible until you compared it against
 * what the rover actually publishes (dashboard/rover/fpms_rover_agent.py):
 *
 *   - It named six fields the rover never sends (`type`, `severity`, `label`,
 *     `kind`, `conf`, `ratio`), and none of the fields it DOES send apart from
 *     `alert` and `component`. So /api/public/events published, for every
 *     event, the single fact `alert: true`. The public feed was empty of
 *     meaning and the page rendered "alert=true" at a human being.
 *   - `label` was the dangerous one. It is the detector's class name, and the
 *     operator's custom_labels.json maps class 0 to a real person's first name
 *     (documented in cloud/HANDOFF.md). Whitelisting `label` is exactly the
 *     leak that gating the camera and fixing the report headlines was meant to
 *     prevent — the detector's WORDS give away the same fact as its PIXELS.
 *     It is gone, and it must not come back. The same reasoning applies to any
 *     future field that carries a class name.
 *
 * Also deliberately absent, and each for a stated reason:
 *   - `detail`   free-form prose assembled from labels and distances (rule 3).
 *   - `error`    an exception string; carries file paths and device nodes (rule 2).
 *   - `device`   literally "/dev/videoN" — a file path (rule 2).
 *   - `regions`  pixel bounding boxes; tells a reader where in the frame a
 *                person was standing, which is camera data by another name.
 *   - `source`   internal implementation name, of no use to a viewer.
 *   - `repeat`   always true in the producer; noise.
 */
const PUBLIC_EVENT_DATA = {
  alert: (v) => (typeof v === "boolean" ? v : undefined),
  // Fraction of the camera frame showing flame colours.
  coverage_ratio: (v) => num(v, 0, 1, 4),
  // Legacy/synthetic alias for the same quantity; kept so the uplink test
  // fixture and any older archived rows still render.
  ratio: (v) => num(v, 0, 1, 4),
  conf: (v) => num(v, 0, 1, 3),
  distance_mm: (v) => num(v, 0, 100_000, 0),
  bearing_deg: (v) => num(v, -360, 360, 0),
  count: (v) => num(v, 0, 10_000, 0),
  component: (v) => publicComponent(v),
  species: (v) => publicSpecies(v),
};

/**
 * Which subsystem faulted. A closed vocabulary because the producer sets it
 * from a thread name, and a thread name is code-shaped: publishing an
 * unrecognised one would leak an internal identifier for no benefit.
 */
const PUBLIC_COMPONENTS = new Set(["camera", "lidar", "heartbeat", "motor", "gps", "network"]);

/**
 * Wildlife classes safe to name in public.
 *
 * These are the ten COCO animal classes the rover treats as wildlife. The list
 * is duplicated here on purpose rather than trusted from the payload: the rover
 * applies operator label overrides BEFORE publishing, so a species string can
 * be an arbitrary operator-chosen word — including a person's name. Anything
 * off this list becomes "an unidentified animal", which is both safe and true.
 */
const PUBLIC_SPECIES = new Set([
  "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
]);
const PUBLIC_SPECIES_UNKNOWN = "unidentified animal";

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
 *
 * The phrases are written for someone who has never heard of this project, so
 * they say what the check IS as well as how it came out.
 */
const PUBLIC_REPORT_HEADLINE = {
  ok: "Automatic check passed — all monitored systems nominal.",
  warning: "Automatic check found a condition that needs attention.",
  critical: "Automatic check found a critical condition.",
};

/** Severity itself is whitelisted: an unrecognised value must not pass through. */
const PUBLIC_REPORT_UNKNOWN = { severity: "unknown", summary: "Automatic check reported a status." };

/**
 * Plain-English sentence for each event subtype.
 *
 * SAME MECHANISM AS PUBLIC_REPORT_HEADLINE, FOR THE SAME REASON
 * -------------------------------------------------------------
 * The rover already ships a ready-made English sentence per event, in
 * `data.detail`. Publishing it would be the obvious shortcut and it is exactly
 * what design rule 3 forbids: that sentence is assembled by interpolating
 * detection labels and species into a template, so it can contain the operator
 * label ("Aryan") that the whole camera gate exists to withhold.
 *
 * So the sentence a viewer reads is built HERE, from a fixed phrase this file
 * owns plus values that have already passed PUBLIC_EVENT_DATA. Every substring
 * is either a literal in this file or a number. There is no path by which rover
 * prose reaches a viewer.
 *
 * It is computed server-side rather than in the page so that the JSON feed —
 * which is the thing embedders and other students will actually consume — is
 * self-explanatory too, and so the page never has to invent wording.
 */
function eventHeadline(subtype, d) {
  switch (subtype) {
    case "fire": {
      const where = fraction(d.coverage_ratio ?? d.ratio);
      return "Flame-coloured light was detected by the rover's camera." +
        (where ? ` It covered about ${where} of the picture.` : "");
    }
    case "fire_cleared":
      return "The flame colours are no longer visible. The fire signal has cleared.";
    case "obstacle": {
      const how = distance(d.distance_mm);
      // A bearing of 0 means "straight ahead", which the sentence has already
      // said. Printing "Bearing 0°" adds nothing and reads like a placeholder.
      const at = d.bearing_deg ? ` Bearing ${d.bearing_deg}°.` : "";
      return "Something blocked the rover's path" + (how ? `, about ${how} ahead.` : ".") + at;
    }
    case "obstacle_cleared":
      return "The path ahead is clear again.";
    case "wildlife": {
      const list = Array.isArray(d.species) && d.species.length ? joinWords(d.species) : null;
      const n = d.count;
      return list
        ? `Wildlife was seen in the camera frame: ${list}.`
        : (n ? `${n} animal${n === 1 ? "" : "s"} were seen in the camera frame.`
             : "Wildlife was seen in the camera frame.");
    }
    case "fault":
      // "other" is what publicComponent returns for a component name it does
      // not recognise, so it must not be dropped into the sentence as if it
      // were the name of a part.
      return d.component && d.component !== "other"
        ? `The rover reported a fault in its ${d.component}.`
        : "The rover reported a fault in one of its systems.";
    case "camera_recovered":
      return "The rover's camera recovered and is working again.";
    case "hazard":
      return "The rover reported a hazard.";
    default:
      // Unreachable: PUBLIC_EVENT_TYPES gates the subtype before this runs.
      // Present so a type added to that set without a phrase here degrades to a
      // safe sentence rather than to `undefined` on the page.
      return "The rover reported an event.";
  }
}

/* ----------------------------------------------------------------- helpers */

/** Bounded, finite, rounded — or undefined. Rejects NaN, Infinity and strings. */
function num(v, min, max, dp) {
  const n = Number(v);
  if (!Number.isFinite(n) || n < min || n > max) return undefined;
  return Number(n.toFixed(dp));
}

function publicComponent(v) {
  if (typeof v !== "string") return undefined;
  const s = v.toLowerCase();
  return PUBLIC_COMPONENTS.has(s) ? s : "other";
}

/** Array of class names -> allowlisted species names, deduped and capped. */
function publicSpecies(v) {
  if (!Array.isArray(v)) return undefined;
  const out = [];
  for (const raw of v) {
    if (typeof raw !== "string") continue;
    const name = PUBLIC_SPECIES.has(raw.toLowerCase())
      ? raw.toLowerCase()
      : PUBLIC_SPECIES_UNKNOWN;
    if (!out.includes(name)) out.push(name);
    if (out.length >= 4) break;
  }
  return out.length ? out : undefined;
}

/** 0.081 -> "8%". Small but non-zero must not round to "0%" and read as nothing. */
function fraction(r) {
  if (typeof r !== "number") return null;
  const pct = r * 100;
  if (pct <= 0) return null;
  return pct < 1 ? "less than 1%" : `${Math.round(pct)}%`;
}

/** Millimetres -> something a person can picture. */
function distance(mm) {
  if (typeof mm !== "number") return null;
  return mm < 1000 ? `${Math.round(mm / 10)} cm` : `${(mm / 1000).toFixed(1)} m`;
}

function joinWords(list) {
  if (list.length === 1) return list[0];
  return `${list.slice(0, -1).join(", ")} and ${list[list.length - 1]}`;
}

/** Apply PUBLIC_EVENT_DATA. Everything not in the table is dropped. */
function scrub(data) {
  const out = {};
  if (!data || typeof data !== "object") return out;
  for (const [key, check] of Object.entries(PUBLIC_EVENT_DATA)) {
    const v = check(data[key]);
    if (v !== undefined) out[key] = v;
  }
  return out;
}

function publicJson(obj, status = 200, cacheControl = DATA_CACHE_CONTROL) {
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
      // Lets everything public be invalidated in one call if a projection here
      // is ever found to be leaking. Purging by tag is the only way to clear a
      // long-lived cached response before it expires on its own.
      "Cache-Tag": "fpms-public",
    },
  });
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
 *
 * EVERYTHING THIS MODULE EMITS IS IN MILLISECONDS. Every field named `*_at`,
 * `ts`, `last_seen` or `since` in a public response has been through here. If
 * you add a timestamp to a response, put it through here too — a seconds value
 * that escapes makes the whole fleet read as offline, which is the one bug this
 * page cannot afford.
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

  // The most recent fire OR fire-cleared event, if any.
  //
  // The `ts >= ?` floor is not cosmetic. idx_readings_events is (kind, ts DESC),
  // so `subtype IN (...)` is not indexed — it is a filter applied to rows the
  // engine has already read. Walking kind='events' newest-first therefore reads
  // every event row until it finds a fire one, and when there is no fire event
  // at all (the normal, permanent case) it reads all of them. LIMIT 1 caps what
  // comes back, not what gets scanned. The floor turns that into a bounded range
  // scan: at worst one day of events, and it stops at the first row older than
  // the cutoff.
  //
  // `cleared_at` is reported as well as `active`, because "no fire right now"
  // and "a fire was detected two hours ago and then cleared" are different
  // answers and a viewer deserves the second one when it is true.
  let fire = { active: false, since: null, cleared_at: null, lookback_s: FIRE_LOOKBACK_S };
  if (env.DB) {
    try {
      const row = await env.DB
        .prepare(
          "SELECT ts, subtype FROM readings WHERE kind = 'events' AND ts >= ? " +
          "AND subtype IN ('fire','fire_cleared') ORDER BY ts DESC LIMIT 1",
        )
        // Stored in unix seconds, same unit as the rovers send. `now` is
        // milliseconds, hence the /1000 — this line is the reason toMs exists.
        .bind(now / 1000 - FIRE_LOOKBACK_S)
        .first();
      if (row?.subtype === "fire") fire = { ...fire, active: true, since: toMs(row.ts) };
      else if (row?.subtype === "fire_cleared") fire = { ...fire, cleared_at: toMs(row.ts) };
    } catch {
      // Leave fire as inactive rather than guessing.
    }
  }

  const online = rovers.filter((r) => r.status === "online").length;

  // The single honest answer to "when did you last hear anything?". Null when
  // nothing has ever reported — which the page renders as "waiting for the
  // first report", not as a failure.
  const lastDataAt = rovers.reduce((max, r) => Math.max(max, r.last_seen || 0), 0) || null;

  // A machine-readable state, so the page never has to re-derive one by
  // pattern-matching on the human `status` string. Ordered by what a viewer
  // most needs to know first.
  let state;
  if (fire.active) state = "fire";
  else if (online > 0) state = "live";
  else if (rovers.some((r) => r.status === "stale")) state = "recently_offline";
  else if (rovers.length) state = "offline";
  else state = "no_data";

  return publicJson({
    generated_at: now,
    system: "FPMS — Fire Prevention & Monitoring System",
    // Kept verbatim for existing consumers of this feed. `state` is the field
    // to branch on; this one is prose and may be reworded.
    status: online > 0 ? "operational" : (rovers.length ? "no rovers reporting" : "no data"),
    state,
    rovers,
    last_data_at: lastDataAt,
    // Published so a consumer can explain the thresholds rather than guess
    // them, and so the page's wording and this file can never disagree.
    windows: { online_s: ONLINE_WINDOW_S, stale_s: STALE_WINDOW_S },
    counts: {
      rovers_total: rovers.length,
      rovers_online: online,
      messages_seen: snap?.messages_seen ?? null,
    },
    fire,
    camera_public: env.FPMS_PUBLIC_CAMERA === "on",
  });
}

/**
 * Recent safety events, field-whitelisted, each with a fixed-phrase headline.
 *
 * See PUBLIC_EVENT_DATA for what survives and why, and eventHeadline for why
 * the sentence is built here rather than taken from the rover's own `detail`.
 */
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
      const data = scrub(parsed);
      events.push({
        ts: toMs(r.ts),
        thing: r.thing,
        subtype: r.subtype,
        headline: eventHeadline(r.subtype, data),
        data,
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
    }, 403, DENY_CACHE_CONTROL);
  }
  const thing = String(url.searchParams.get("thing") || "rover2");
  if (!/^[a-z0-9_.-]+$/i.test(thing)) {
    return publicJson({ error: "invalid thing" }, 400, DENY_CACHE_CONTROL);
  }
  try {
    const latest = await hubStub(env)
      .fetch(`https://hub/latest?channel=camera:${encodeURIComponent(thing)}`)
      .then((r) => r.json());
    const frame = latest?.data?.frame;
    if (!frame) return publicJson({ error: "no frame available" }, 404);
    // `ts` is the frame's own capture time, in milliseconds after toMs. The
    // page shows it beside the image: a still from three hours ago must not be
    // mistaken for a live view.
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
    ...(summary || { status: "unavailable", state: "unknown", rovers: [], counts: {} }),
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
        "Cache-Control": PAGE_CACHE_CONTROL,
        "Cache-Tag": "fpms-public",
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

  // HEAD is allowed because uptime monitors use it, and the runtime strips the
  // body for us — it costs the same as the GET it stands in for and no more.
  if (request.method !== "GET" && request.method !== "HEAD" && request.method !== "OPTIONS") {
    return publicJson({ error: "method not allowed" }, 405, "no-store");
  }

  // Burst guard. Cheap by construction: a Map lookup, no storage, no DO call,
  // no binding. See scrapeRetryAfter for what it does and does not cover.
  //
  // The 429 is explicitly no-store. Everything else here is cacheable and
  // Workers Caching honours that, so a cacheable rejection would be handed to
  // every other viewer arriving at the same colo for the next TTL — one scraper
  // would take the status page down for the neighbourhood, which is the outcome
  // this endpoint exists to prevent.
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

  // The feed is CORS-open by design (see publicJson). Answering the preflight
  // means an embedder using a non-safelisted header gets data rather than a 405.
  //
  // Deliberately AFTER the burst guard, not before it. Workers Caching only
  // stores GET and HEAD, so an OPTIONS answered above the guard would be an
  // uncached, uncounted path straight to the Worker — a free hole through the
  // one defence that covers cache-busting callers.
  if (request.method === "OPTIONS") {
    return new Response(null, {
      status: 204,
      headers: {
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "GET, HEAD, OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type, Accept",
        "Access-Control-Max-Age": "86400",
        "Cache-Control": DENY_CACHE_CONTROL,
      },
    });
  }

  // One call for everything the page renders.
  //
  // This exists for an availability reason, not tidiness. The page used to fetch
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

  return publicJson({ error: "not found" }, 404, DENY_CACHE_CONTROL);
}

/* --------------------------------------------------------------- the page */

// Self-contained on purpose: no CDN, no build step, no dependency on the React
// bundle. If the SPA fails to build or its assets go missing, this page still
// answers "is anything on fire?" — which is the one question that matters.
//
// WHY IT IS CLIENT-RENDERED AND NOT SERVER-RENDERED
// ------------------------------------------------
// Server-rendering the status into the HTML would give a better first paint and
// would work without JavaScript. It would also make /live a dynamic page: every
// single view would cost a Durable Object call and a D1 read, and the page could
// then only be cached as briefly as the data it embeds. Splitting them lets the
// shell be cached for ten minutes and the data for fifteen seconds, which is
// what keeps a traffic spike free. The <noscript> block below is the honest
// mitigation: it links straight to the JSON, which is readable in any browser.
//
// EVERY STRING BELOW IS WRITTEN FOR SOMEONE WHO HAS NEVER HEARD OF THIS PROJECT.
// If you edit the copy, keep it that way: the audience is a judge or a parent on
// a phone, arriving from a QR code, with no idea what a rover or a Worker is.
const PUBLIC_PAGE = `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>FPMS — a student-built wildfire-watch rover, and everything it has found</title>
<meta name="description" content="FPMS is a self-driving wildfire-watch rover built by two Grade 8 students in Canada. See what it has detected, how far it has driven, recordings of its laser scans and camera, and ask its AI assistant anything — no login, and it all works while the rover is switched off.">
<meta name="color-scheme" content="light dark">
<meta name="theme-color" content="#0b0f14" media="(prefers-color-scheme: dark)">
<meta name="theme-color" content="#ffffff" media="(prefers-color-scheme: light)">
<meta property="og:title" content="FPMS — a student-built wildfire-watch rover">
<meta property="og:description" content="What the rover has detected, how far it has driven, recordings of its laser scans and camera, and an AI assistant that answers questions about it.">
<meta property="og:type" content="website">
<style>
  /* Dark first, because the page is read outdoors at night as often as not,
     then a high-contrast light theme for direct sunlight. Both themes are
     checked to at least 4.5:1 for body text and 3:1 for large text. */
  :root {
    color-scheme: dark light;
    --bg:#0b0f14; --panel:#141b24; --panel2:#1b2430; --line:#2b3644;
    --fg:#eaf1f8; --dim:#9fb0c3;
    --ok:#4ac26b; --warn:#e3a72c; --bad:#ff6b60; --idle:#8896a6; --accent:#79b8ff;
    --on-tone:#080c11;          /* text colour that sits on a tone chip */
    --shadow:0 1px 2px rgba(0,0,0,.4);
  }
  @media (prefers-color-scheme: light) {
    :root {
      --bg:#ffffff; --panel:#f6f8fa; --panel2:#eef2f6; --line:#c2ccd6;
      --fg:#11161c; --dim:#4b5563;
      --ok:#136c2e; --warn:#8a5a00; --bad:#b3251f; --idle:#4f5b68; --accent:#0b5cc4;
      --on-tone:#ffffff;
      --shadow:0 1px 2px rgba(16,22,28,.10);
    }
  }
  @media (prefers-contrast: more) {
    :root { --dim:var(--fg); --line:currentColor; }
  }
  * { box-sizing:border-box; }
  html { -webkit-text-size-adjust:100%; }
  body {
    margin:0; background:var(--bg); color:var(--fg);
    font:17px/1.55 ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
    padding:16px 16px 56px;
    padding-left:max(16px,env(safe-area-inset-left));
    padding-right:max(16px,env(safe-area-inset-right));
    overflow-wrap:break-word;
  }
  .wrap { max-width:760px; margin:0 auto; }
  [hidden] { display:none !important; }

  a { color:var(--accent); }
  a:focus-visible, button:focus-visible, summary:focus-visible, .skip:focus {
    outline:3px solid var(--accent); outline-offset:2px; border-radius:6px;
  }
  .skip {
    position:absolute; left:-9999px; top:8px; background:var(--panel);
    border:1px solid var(--line); padding:10px 14px; border-radius:8px;
  }
  .skip:focus { left:16px; z-index:10; }

  header { margin:8px 0 20px; }
  .eyebrow {
    margin:0 0 6px; font-size:13px; font-weight:700; letter-spacing:.08em;
    text-transform:uppercase; color:var(--dim);
  }
  h1 { font-size:clamp(22px,6vw,30px); line-height:1.2; margin:0 0 10px; letter-spacing:-0.02em; }
  .lede { margin:0; color:var(--fg); font-size:16px; }

  /* --- the one thing a first-time visitor must read --- */
  .banner {
    border:1px solid var(--line); border-left:6px solid var(--idle);
    border-radius:12px; background:var(--panel); box-shadow:var(--shadow);
    padding:16px 18px; margin:20px 0;
  }
  .banner h2 { margin:2px 0 6px; font-size:clamp(20px,5.5vw,26px); line-height:1.2; }
  .banner p { margin:0 0 6px; }
  .banner p:last-child { margin-bottom:0; }
  .b-ok { border-left-color:var(--ok); }
  .b-warn { border-left-color:var(--warn); }
  .b-bad { border-left-color:var(--bad); }
  .b-idle { border-left-color:var(--idle); }
  .kicker {
    display:flex; align-items:center; gap:8px;
    font-size:13px; font-weight:700; letter-spacing:.08em; text-transform:uppercase;
    color:var(--dim);
  }
  .dot { width:11px; height:11px; border-radius:50%; background:var(--idle); flex:none; }
  .d-ok { background:var(--ok); animation:pulse 2.4s ease-in-out infinite; }
  .d-warn { background:var(--warn); }
  .d-bad { background:var(--bad); animation:pulse 1.2s ease-in-out infinite; }
  @keyframes pulse { 50% { opacity:.35; } }
  @media (prefers-reduced-motion: reduce) { .dot { animation:none !important; } }
  .stamp { color:var(--dim); font-size:14px; }

  h2.sec {
    font-size:14px; text-transform:uppercase; letter-spacing:.07em;
    color:var(--dim); margin:32px 0 10px; font-weight:700;
  }
  section > p.note { margin:0 0 12px; color:var(--dim); font-size:14px; }

  ul.facts { margin:0; padding:0; list-style:none; display:grid; gap:10px; }
  ul.facts li {
    background:var(--panel); border:1px solid var(--line); border-radius:10px;
    padding:12px 14px; font-size:16px;
  }
  ul.facts strong { display:block; font-size:14px; color:var(--dim); text-transform:uppercase; letter-spacing:.05em; }

  .grid { display:grid; grid-template-columns:1fr; gap:10px; }
  @media (min-width:520px) { .grid { grid-template-columns:repeat(auto-fit,minmax(220px,1fr)); } }
  .card {
    background:var(--panel); border:1px solid var(--line); border-radius:10px;
    padding:14px; box-shadow:var(--shadow);
  }
  .card .name { font-weight:700; font-size:18px; margin-bottom:8px; }

  .tone {
    display:inline-block; font-size:12px; font-weight:800; text-transform:uppercase;
    letter-spacing:.06em; padding:3px 10px; border-radius:999px;
    background:var(--idle); color:var(--on-tone);
  }
  .t-online { background:var(--ok); }
  .t-stale { background:var(--warn); }
  .t-offline { background:var(--idle); }
  .t-critical, .t-bad { background:var(--bad); }
  .t-warning { background:var(--warn); }
  .t-ok { background:var(--ok); }
  .t-unknown { background:var(--idle); }

  ul.feed { margin:0; padding:0; list-style:none; display:grid; gap:10px; }
  ul.feed li {
    background:var(--panel); border:1px solid var(--line); border-radius:10px;
    padding:12px 14px;
  }
  .item-head { display:flex; flex-wrap:wrap; align-items:center; gap:8px; margin:0 0 6px; }
  .item-body { margin:0; font-size:16px; }
  .item-meta { margin:6px 0 0; color:var(--dim); font-size:14px; }
  .empty { color:var(--dim); font-size:15px; }
  time { white-space:nowrap; }

  figure { margin:0; }
  #cam { max-width:100%; height:auto; border-radius:10px; border:1px solid var(--line); display:block; background:var(--panel2); }
  figcaption { color:var(--dim); font-size:14px; margin-top:8px; }

  .controls { display:flex; flex-wrap:wrap; align-items:center; gap:12px; margin:28px 0 0; }
  button {
    font:inherit; font-weight:700; min-height:44px; padding:0 20px;
    background:var(--panel2); color:var(--fg); border:1px solid var(--line);
    border-radius:10px; cursor:pointer;
  }
  button:hover { border-color:var(--accent); }
  button[disabled] { opacity:.55; cursor:progress; }

  details {
    background:var(--panel); border:1px solid var(--line); border-radius:10px;
    padding:0 14px; margin-top:12px;
  }
  details summary { cursor:pointer; font-weight:700; padding:13px 0; min-height:44px; display:flex; align-items:center; }
  details[open] summary { border-bottom:1px solid var(--line); }
  details .inner { padding:12px 0; }
  details dl { margin:0; display:grid; gap:10px; }
  details dt { font-weight:700; }
  details dd { margin:2px 0 0; color:var(--dim); }

  footer {
    margin-top:40px; color:var(--dim); font-size:14px;
    border-top:1px solid var(--line); padding-top:16px;
  }
  footer p { margin:0 0 10px; }
  code {
    background:var(--panel2); border:1px solid var(--line); border-radius:5px;
    padding:1px 6px; font-size:13px;
  }
  .vh {
    position:absolute; width:1px; height:1px; overflow:hidden;
    clip:rect(0 0 0 0); clip-path:inset(50%); white-space:nowrap;
  }

  /* ==================================================================
     The compact status strip.

     WHY THE BANNER IS SMALL BY DEFAULT
     ----------------------------------
     The rover is off almost always, so the honest answer to the page's
     own question is almost always "not right now". Rendered as a full
     alarm-sized banner, that answer was the first and largest thing a
     first-time visitor saw, and it made a working project read as a
     broken one. The status is still the first thing on the page and it
     is still completely honest — it is just sized like a fact rather
     than like an emergency.

     It escalates back to full size for exactly one state: an active
     fire signal (.b-loud, set by the script). Nothing else is allowed
     to claim that much of the screen.
     ================================================================== */
  .banner.b-compact { padding:12px 14px; margin:16px 0 22px; border-left-width:5px; }
  .banner.b-compact h2 { font-size:clamp(16px,4vw,18px); margin:2px 0 4px; }
  .banner.b-compact p { font-size:15px; }
  .banner.b-compact .stamp { font-size:13px; }
  .banner.b-loud { padding:16px 18px; border-left-width:6px; }
  .banner.b-loud h2 { font-size:clamp(20px,5.5vw,26px); }
  .jump {
    display:inline-flex; align-items:center; gap:6px; margin-top:8px;
    font-size:14px; font-weight:600;
  }

  /* ---------------- the assistant ---------------- */
  #ai { margin-top:34px; }
  .ai-card {
    border:1px solid var(--line); border-radius:12px; background:var(--panel);
    box-shadow:var(--shadow); padding:16px; margin-top:10px;
  }
  .ai-lede { margin:0 0 12px; font-size:16px; }
  .ai-chips { display:flex; flex-wrap:wrap; gap:8px; margin:0 0 14px; padding:0; list-style:none; }
  .ai-chip {
    font:inherit; font-size:14px; font-weight:600; text-align:left;
    min-height:44px; padding:8px 14px; border-radius:999px;
    background:var(--panel2); color:var(--fg); border:1px solid var(--line);
    cursor:pointer;
  }
  .ai-chip:hover { border-color:var(--accent); }
  .ai-log { display:grid; gap:10px; margin:0 0 12px; }
  .ai-turn { border-radius:10px; padding:11px 13px; font-size:16px; }
  .ai-you {
    background:var(--panel2); border:1px solid var(--line);
    justify-self:end; max-width:88%;
  }
  .ai-bot { background:transparent; border:1px solid var(--line); border-left:4px solid var(--accent); }
  .ai-who {
    display:block; font-size:12px; font-weight:700; letter-spacing:.06em;
    text-transform:uppercase; color:var(--dim); margin-bottom:4px;
  }
  /* Model output is inserted with textContent and never as markup. Newlines are
     the only formatting it is allowed to carry. */
  .ai-text { margin:0; white-space:pre-wrap; overflow-wrap:break-word; }
  .ai-form { display:flex; flex-wrap:wrap; gap:8px; align-items:flex-start; }
  .ai-form label { flex:1 1 220px; min-width:0; }
  .ai-input {
    font:inherit; font-size:16px; width:100%; min-height:48px; resize:vertical;
    padding:12px; border-radius:10px; background:var(--bg); color:var(--fg);
    border:1px solid var(--line);
  }
  .ai-input:focus-visible { outline:3px solid var(--accent); outline-offset:1px; }
  .ai-send { flex:0 0 auto; }
  .ai-foot { margin:10px 0 0; color:var(--dim); font-size:13px; }
  .ai-busy { color:var(--dim); font-size:14px; }

  /* ---------------- recordings ----------------
     THREE PROVENANCES, THREE COLOURS, AND NONE OF THEM CARRIES MEANING ALONE.
       live       green, pulsing dot   (the existing --ok)
       recorded   violet              (--rec, new here)
       simulated  burnt orange        (--sim, the SAME steps analytics.js uses
                                       for --fx-sim, so one word means one
                                       colour everywhere on this page)
     Every one of them ships with the word beside it, exactly as a status
     colour must — the hue is the fast channel, the label is the true one.

     Validated with the data-viz validator against this page's own panel
     surfaces rather than against the palette's defaults, because contrast is
     only meaningful against the surface a mark really renders on:
       light, surface #f6f8fa : #4a3aa7 / #a34a17 — ALL CHECKS PASS
                                (CVD dE 24.5 protan, normal-vision dE 27.6,
                                 both >= 3:1)
       dark,  surface #141b24 : #9085e9 / #ec835a — chroma, CVD separation
                                (dE 22.2) and contrast all pass. The dark
                                simulated step sits just above the dark
                                CATEGORICAL lightness band because it is not a
                                categorical slot: it is the palette's fixed
                                status step, kept identical to analytics.js so
                                the two sections cannot disagree about what
                                "simulated" looks like. It never appears
                                without the word. */
  :root { --rec:#9085e9; --rec-wash:rgba(144,133,233,.14); --rec-ink:#0b0f14;
          --sim:#ec835a; --sim-ink:#0b0f14; }
  @media (prefers-color-scheme: light) {
    :root { --rec:#4a3aa7; --rec-wash:rgba(74,58,167,.10); --rec-ink:#ffffff;
            --sim:#a34a17; --sim-ink:#ffffff; }
  }
  #rec { margin-top:34px; }
  .rec-note { margin:0 0 14px; color:var(--dim); font-size:15px; }
  .rec-grid { display:grid; grid-template-columns:1fr; gap:12px; }
  @media (min-width:760px) { .rec-grid { grid-template-columns:1fr 1fr; } }
  .rec-card {
    border:1px solid var(--rec); border-left:5px solid var(--rec);
    border-radius:10px; padding:14px; min-width:0;
    background-color:var(--panel);
    background-image:repeating-linear-gradient(45deg,
      var(--rec-wash) 0 6px, transparent 6px 16px);
  }
  .rec-card h3 { margin:0 0 2px; font-size:16px; }
  .rec-card .rec-sub { margin:0 0 10px; font-size:13px; color:var(--dim); }
  .rec-badge {
    display:inline-block; font-size:11px; font-weight:800; letter-spacing:.09em;
    text-transform:uppercase; padding:3px 8px; border-radius:4px;
    background:var(--rec); color:var(--rec-ink); margin-right:8px;
    vertical-align:2px; white-space:nowrap;
  }
  .rec-badge.is-sim { background:var(--sim); color:var(--sim-ink); }
  .rec-stage { position:relative; background:var(--panel2); border-radius:8px;
               overflow:hidden; min-height:180px; display:flex;
               align-items:center; justify-content:center; }
  .rec-stage img { display:block; max-width:100%; height:auto; }
  .rec-stage svg { display:block; max-width:100%; height:auto; }
  /* Burnt into the frame, not beside it. */
  .rec-stamp {
    position:absolute; left:8px; top:8px; z-index:2;
    font-size:11px; font-weight:800; letter-spacing:.09em; text-transform:uppercase;
    background:var(--rec); color:var(--rec-ink); padding:3px 8px; border-radius:4px;
    pointer-events:none;
  }
  .rec-stamp.is-sim { background:var(--sim); color:var(--sim-ink); }
  .rec-when {
    position:absolute; right:8px; bottom:8px; z-index:2;
    font-size:12px; background:var(--bg); color:var(--fg);
    border:1px solid var(--line); padding:2px 7px; border-radius:4px;
    pointer-events:none; font-variant-numeric:tabular-nums;
  }
  .rec-scroll { overflow-x:auto; overflow-y:hidden; -webkit-overflow-scrolling:touch; }
  .rec-controls { display:flex; align-items:center; gap:10px; flex-wrap:wrap; margin-top:10px; }
  .rec-controls input[type=range] { flex:1 1 140px; min-width:120px; height:24px; accent-color:var(--rec); }
  .rec-controls button { min-height:44px; padding:0 16px; font-size:14px; }
  .rec-clock { font-size:13px; color:var(--dim); font-variant-numeric:tabular-nums; min-width:78px; }
  .rec-meta { margin:8px 0 0; font-size:13px; color:var(--dim); }
  .rec-empty { border:1px dashed var(--line); border-radius:8px; padding:18px 14px;
               text-align:center; color:var(--dim); font-size:14px; }
  .rec-empty b { display:block; color:var(--fg); font-size:15px; margin-bottom:4px; }
  @media (prefers-reduced-motion: reduce) { .rec-stage img { transition:none; } }
${ANALYTICS_STYLE}
</style>
</head>
<body>
<a class="skip" href="#status">Skip to the status</a>
<div class="wrap">
  <header>
    <p class="eyebrow">A student wildfire-watch robot &middot; public, no login needed</p>
    <h1>FPMS: a rover that watches for wildfires</h1>
    <p class="lede">
      FPMS drives itself on patrol. Its camera looks for flame colours, its
      spinning laser scanner measures what is in front of it, and it files a
      report to this website every few seconds over a mobile network. Built by
      two Grade&nbsp;8 students in Canada for World Robot Olympiad.
      <strong>The rover only runs during tests and demonstrations</strong> &mdash;
      so everything below is drawn from the record it has already built up, and
      it is all here whether or not the robot is switched on right now.
    </p>
  </header>

  <section id="status" class="banner b-idle b-compact" aria-labelledby="b-title">
    <p class="kicker"><span class="dot" id="b-dot" aria-hidden="true"></span><span id="b-kicker">Checking</span></p>
    <h2 id="b-title">Checking whether the rover is powered on&hellip;</h2>
    <p id="b-note">Fetching the most recent report.</p>
    <p class="stamp" id="b-stamp"></p>
  </section>

  <!--
    The announcement channel, kept separate from the banner on purpose. Marking
    the banner itself as a live region would re-read the whole thing every time
    a relative timestamp ticked over from "4 minutes ago" to "5 minutes ago",
    which is once a minute, forever. This is written to only when the status
    genuinely changes, so a screen-reader user hears "the rover is offline"
    once rather than sixty times an hour.
  -->
  <p class="vh" id="announce" role="status" aria-live="polite" aria-atomic="true"></p>

  <noscript>
    <div class="banner b-warn">
      <h2>JavaScript is switched off</h2>
      <p>This page loads its status live, so it needs JavaScript. The same
      information is available as plain data at
      <a href="/api/public/all">/api/public/all</a>.</p>
    </div>
  </noscript>

${ANALYTICS_HTML}

  <!--
    THE ASSISTANT.

    Placed high on purpose. It is the part of this system that works best when
    the rover is dark: it answers from the same stored facts and history the
    charts above are drawn from, so a judge who arrives at midnight can still
    interrogate the project. The suggested questions are the ones people
    actually ask, pre-loaded so nobody has to guess what it knows.
  -->
  <section id="ai" aria-labelledby="h-ai">
    <h2 class="sec" id="h-ai">Ask the FPMS assistant</h2>
    <div class="ai-card">
      <p class="ai-lede">
        An AI assistant that answers questions about this project from its stored
        facts and its recorded history. It works while the rover is switched off.
        It will say &ldquo;I don&rsquo;t know&rdquo; rather than guess, and it
        cannot drive or contact anything.
      </p>

      <p class="ai-lede" style="font-size:14px;color:var(--dim);margin-bottom:8px">
        Try one of these:
      </p>
      <ul class="ai-chips" id="ai-chips"></ul>

      <div class="ai-log" id="ai-log"></div>

      <form class="ai-form" id="ai-form">
        <label for="ai-input">
          <span class="vh">Your question about FPMS</span>
          <textarea id="ai-input" class="ai-input" rows="2" maxlength="400"
                    placeholder="Ask anything about the rover, what it found, or how it works"></textarea>
        </label>
        <button type="submit" class="ai-send" id="ai-send">Ask</button>
      </form>
      <p class="ai-foot" id="ai-foot">
        Answers are generated by a language model from a fixed set of public
        project facts and the rover&rsquo;s own recorded events. Nothing you type
        here is stored, and no private telemetry is available to it.
      </p>
      <p class="vh" id="ai-live" role="status" aria-live="polite"></p>
    </div>
  </section>

  <!--
    RECORDINGS.

    Saved material, and the page must never let it read as live. Three
    independent channels carry that, none of which can be styled away on its
    own: the RECORDED badge is burnt into the media frame itself (so a cropped
    screenshot still carries it), every card wears the recorded rail and hatch,
    and every frame is captioned with the date it was captured rather than a
    relative "just now". The alt text and the SVG <title> say it too, which is
    what a screen reader reads out.

    If the recordings API is not deployed yet, this degrades to the scripted
    demonstration run — which is labelled SIMULATED, in the analytics section's
    amber, and never as "recorded". Simulated and recorded are different claims
    and this page keeps them apart.
  -->
  <section id="rec" aria-labelledby="h-rec">
    <h2 class="sec" id="h-rec">Recordings from previous runs</h2>
    <p class="rec-note" id="rec-note">
      Saved material from patrols that have already finished. None of it is live.
    </p>
    <div class="rec-grid" id="rec-grid"></div>
  </section>

  <section aria-labelledby="h-rovers">
    <h2 class="sec" id="h-rovers">Rovers</h2>
    <div id="rovers" class="grid"><p class="empty">Loading&hellip;</p></div>
  </section>

  <section id="cam-wrap" aria-labelledby="h-cam" hidden>
    <h2 class="sec" id="h-cam">Latest camera picture</h2>
    <figure>
      <img id="cam" alt="The most recent still picture from the rover's forward-facing camera." decoding="async">
      <figcaption id="cam-note">Loading&hellip;</figcaption>
    </figure>
  </section>

  <section aria-labelledby="h-events">
    <h2 class="sec" id="h-events">What the rover has seen</h2>
    <p class="note">Newest first. Times are shown in your own timezone.</p>
    <ul id="events" class="feed"><li class="empty">Loading&hellip;</li></ul>
  </section>

  <section aria-labelledby="h-reports">
    <h2 class="sec" id="h-reports">Automatic system checks</h2>
    <ul id="reports" class="feed"><li class="empty">Loading&hellip;</li></ul>
  </section>

  <section aria-labelledby="h-what">
    <h2 class="sec" id="h-what">What am I looking at?</h2>
    <ul class="facts">
      <li><strong>The rover</strong> A small autonomous robot that drives itself
      around and keeps watch. It runs its own camera and its own obstacle
      detection on board, then sends short reports to this website over a
      mobile network.</li>
      <li><strong>What it looks for</strong> Flame colours in the camera picture,
      things blocking its path, and animals in the frame. Anything it finds is
      listed above in the order it happened.</li>
      <li><strong>This page</strong> A read-only public window on the system. It
      is not a live video feed, and it is not a fire service alert. The rover is
      switched on for demonstrations and tests, so most of the time you are
      reading its record rather than watching something happen.</li>
    </ul>

    <details>
      <summary>How to read the words on this page</summary>
      <div class="inner">
        <dl>
          <dt>Online</dt>
          <dd>The rover reported within the last 90 seconds. What you see is happening now.</dd>
          <dt>Recently offline</dt>
          <dd>Nothing in the last 90 seconds, but something within the last hour.</dd>
          <dt>Offline</dt>
          <dd>Nothing for over an hour. This is normal &mdash; the rover is usually switched off. The website itself is still working.</dd>
          <dt>Recorded</dt>
          <dd>Material saved during a run that has already finished. It is stamped on the picture itself and dated. It is never live.</dd>
          <dt>Simulated</dt>
          <dd>A scripted demonstration built from the system&rsquo;s own settings, so the page can still show how the system behaves when nothing has been recorded. No measurement in it is real.</dd>
          <dt>Fire signal</dt>
          <dd>The camera saw flame-like colours. It is a camera detecting colour, not a certified fire alarm.</dd>
          <dt>Automatic check</dt>
          <dd>Every 15 minutes the system reviews its own data and records how healthy it looks. The wording of those lines is fixed; the detailed findings stay private.</dd>
          <dt>&ndash;&ndash;</dt>
          <dd>Nothing was recorded for that number. It is deliberately not shown as zero, because &ldquo;we did not measure it&rdquo; and &ldquo;we measured none&rdquo; are different facts.</dd>
        </dl>
      </div>
    </details>
  </section>

  <p class="controls">
    <button id="refresh" type="button">Refresh now</button>
    <span class="stamp" id="poll-note">This page updates by itself every 30 seconds.</span>
  </p>

  <footer>
    <p>FPMS &mdash; Fire Prevention &amp; Monitoring System. A student project
    built for World Robot Olympiad Canada. This is a read-only public view:
    driving the rover, its terminal, its raw camera feed and its live telemetry
    all require a login and cannot be reached from this page.</p>
    <p>Prefer the raw data? It is open and needs no key:
    <a href="/api/public/all"><code>/api/public/all</code></a>,
    <a href="/api/public/summary"><code>/api/public/summary</code></a>,
    <a href="/api/public/events"><code>/api/public/events</code></a>,
    <a href="/api/public/analytics"><code>/api/public/analytics</code></a>,
    <a href="/api/public/chat"><code>/api/public/chat</code></a>.
    Anything marked <em>simulated</em> in those responses is scripted, and says
    so in its own payload.</p>
  </footer>
</div>

<script>
/* A page that fails silently is undiagnosable by the only person who can see it.
 *
 * This page is public, so its audience is on phones and laptops this project
 * will never have access to. When something throws mid-render the layout
 * collapses to empty grid containers, which looks like a broken site and tells
 * the visitor -- and us -- nothing. Reported from a phone as "it appears for a
 * second, then just becomes grid", which is exactly what an uncaught exception
 * between two render steps looks like from outside.
 *
 * So: catch it, show it, and keep it short enough to read on a phone. This is
 * the same lesson as a windowed executable with no console -- the mechanism
 * that would report the fault must not be the thing that is broken.
 */
(function () {
  var shown = false;
  function report(what, where) {
    if (shown) return;
    shown = true;
    try {
      var el = document.getElementById("jsfail");
      if (!el) {
        el = document.createElement("div");
        el.id = "jsfail";
        document.body.insertBefore(el, document.body.firstChild);
      }
      el.hidden = false;
      el.setAttribute("role", "alert");
      el.style.cssText =
        "margin:12px;padding:14px 16px;border:1px solid #b45309;border-radius:12px;" +
        "background:#fffbeb;color:#7c2d12;font:14px/1.5 system-ui,sans-serif";
      el.textContent =
        "This page hit a script error and stopped updating. The rest of the site " +
        "still works. Details: " + String(what) + (where ? "  [" + where + "]" : "");
    } catch (_) { /* nothing left to try */ }
  }
  window.addEventListener("error", function (e) {
    report(e && e.message ? e.message : "unknown error",
           e && e.filename ? (e.filename + ":" + e.lineno + ":" + e.colno) : "");
  });
  window.addEventListener("unhandledrejection", function (e) {
    var r = e && e.reason;
    report(r && r.message ? r.message : String(r), "promise");
  });
})();
</script>
<script>
(function () {
  var $ = function (id) { return document.getElementById(id); };

  function esc(s) {
    return String(s == null ? "" : s).replace(/[&<>"']/g, function (c) {
      return { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c];
    });
  }

  // --- time -----------------------------------------------------------------
  // Every timestamp the API sends is in MILLISECONDS (see toMs in public.js).
  // Nothing here divides or multiplies by 1000; if a value ever looks like it is
  // from 1970, the bug is upstream of this file, not in it.

  function ago(ts) {
    if (!ts) return null;
    var s = Math.round((Date.now() - ts) / 1000);
    if (s < 0) s = 0;                                  // clock skew, not the future
    if (s < 45) return "just now";
    if (s < 90) return "a minute ago";
    var m = Math.round(s / 60);
    if (m < 60) return m + " minutes ago";
    var h = Math.round(m / 60);
    if (h < 24) return h === 1 ? "an hour ago" : h + " hours ago";
    var d = Math.round(h / 24);
    return d === 1 ? "yesterday" : d + " days ago";
  }

  function absolute(ts) {
    try { return new Date(ts).toLocaleString(); } catch (e) { return ""; }
  }

  // A real <time> element: screen readers and search engines get the machine
  // timestamp, everyone gets "3 hours ago", and hovering shows the exact time.
  function timeHtml(ts) {
    var rel = ago(ts);
    if (!rel) return '<span class="stamp">time unknown</span>';
    var iso = "";
    try { iso = new Date(ts).toISOString(); } catch (e) { iso = ""; }
    return '<time datetime="' + esc(iso) + '" title="' + esc(absolute(ts)) + '">' +
      esc(rel) + "</time>";
  }

  // --- static vocabularies --------------------------------------------------
  // Closed sets, mirroring the server's. An unrecognised value falls back to a
  // neutral word rather than being printed raw.

  var EVENT_LABEL = {
    fire: "Fire signal",
    fire_cleared: "Fire signal cleared",
    obstacle: "Obstacle",
    obstacle_cleared: "Path clear",
    wildlife: "Wildlife",
    fault: "Fault",
    camera_recovered: "Camera recovered",
    hazard: "Hazard"
  };
  var EVENT_TONE = {
    fire: "bad", fire_cleared: "ok", obstacle: "warning", obstacle_cleared: "ok",
    wildlife: "online", fault: "warning", camera_recovered: "ok", hazard: "bad"
  };
  var ROVER_WORD = { online: "Online", stale: "Recently offline", offline: "Offline" };
  var SEVERITY_WORD = { ok: "Healthy", warning: "Attention", critical: "Critical", unknown: "Unknown" };

  function getJSON(path) {
    return fetch(path, { headers: { Accept: "application/json" } })
      .then(function (r) { return r.ok ? r.json() : null; })
      .catch(function () { return null; });
  }

  // --- rendering ------------------------------------------------------------

  var announced = "";

  // The "loud" flag promotes the strip back to a full-size banner. Exactly one
  // state uses it — an active fire signal. Everything else is a fact, not an
  // emergency, and is sized accordingly: see .b-compact in the stylesheet.
  function setBanner(tone, dot, kicker, title, note, loud) {
    $("status").className = "banner b-" + tone + (loud ? " b-loud" : " b-compact");
    $("b-dot").className = "dot" + (dot ? " d-" + dot : "");
    $("b-kicker").textContent = kicker;
    $("b-title").textContent = title;
    $("b-note").textContent = note;

    // Announce only on a real change of status — see the note on #announce.
    var key = tone + "|" + title;
    if (key !== announced) {
      announced = key;
      $("announce").textContent = title + ". " + note;
    }
  }

  function renderSummary(d, fresh) {
    var counts = d.counts || {};
    var online = counts.rovers_online || 0;
    var total = counts.rovers_total || 0;
    var seen = ago(d.last_data_at);
    var state = d.state || "unknown";

    // The honest headline. Note that nothing here says "live" unless a rover
    // genuinely reported inside the last 90 seconds.
    if (state === "fire") {
      setBanner("bad", "bad", "Alert",
        "A fire signal was detected",
        "The rover's camera saw flame-coloured light. Detected " +
          (ago(d.fire && d.fire.since) || "recently") + ".",
        true);
    } else if (state === "live") {
      // Deliberately "right now" and not "in the last 24 hours": a fire that
      // was detected and then cleared this morning is reported on the stamp
      // line below, and the two lines must not contradict each other.
      setBanner("ok", "ok", "Live now",
        "The rover is on patrol right now",
        online + " of " + total + " rover" + (total === 1 ? "" : "s") +
          " reporting live, and no fire signal is being detected.");
    } else if (state === "recently_offline") {
      // Every offline wording below says the same two things in the same order:
      // the honest status, then where the substance is. A visitor who reads only
      // this strip should still know the page is worth scrolling.
      setBanner("warn", "warn", "Not live",
        "The rover is not reporting at the moment",
        "It last reported " + (seen || "recently") + ". Its full record, the " +
          "charts, the recordings and the assistant below all keep working.");
    } else if (state === "offline") {
      setBanner("idle", "", "Not live",
        "The rover is switched off, which is normal",
        "It runs during tests and demonstrations, and was last heard from " +
          (seen || "some time ago") + ". Everything below is its stored record: " +
          "what it has found, how far it has driven, recordings of past runs, " +
          "and an assistant that can answer questions about all of it.");
    } else if (state === "no_data") {
      setBanner("idle", "", "Not live",
        "Waiting for the rover's first report",
        "The website is running and ready, but no rover has reported to it yet.");
    } else {
      setBanner("idle", "", "Unknown",
        "Status unavailable",
        "The service answered, but not with anything this page understands.");
    }

    // Separate line, always shown: when the DATA was produced, which after a
    // cache hit can be a little older than "now". Saying so is the point.
    var parts = [];
    var checked = ago(d.generated_at);
    if (checked) parts.push("Status checked " + checked + ".");
    if (d.fire && !d.fire.active && d.fire.cleared_at) {
      parts.push("A fire signal was detected and cleared " + ago(d.fire.cleared_at) + ".");
    }
    $("b-stamp").textContent = parts.join(" ");

    var rovers = d.rovers || [];
    if (!rovers.length) {
      $("rovers").innerHTML = '<p class="empty">No rover has reported to this system yet.</p>';
    } else {
      $("rovers").innerHTML = rovers.map(function (r) {
        var word = ROVER_WORD[r.status] || "Unknown";
        var when = r.last_seen
          ? "Last reported " + timeHtml(r.last_seen)
          : "Never reported";
        return '<div class="card"><p class="name">' + esc(r.thing) + "</p>" +
          '<p><span class="tone t-' + esc(r.status) + '">' + esc(word) + "</span></p>" +
          '<p class="item-meta">' + when + "</p></div>";
      }).join("");
    }

    // Only on a genuinely new payload. render() also runs once a minute purely
    // to refresh the relative timestamps, and fetching a fresh camera frame on
    // each of those would double this page's request rate for no new
    // information — a frame is only ever as new as the summary that announced it.
    if (fresh && d.camera_public) loadCamera();
  }

  function renderEvents(d) {
    var rows = (d && d.events) || [];
    if (!rows.length) {
      $("events").innerHTML =
        '<li class="empty">Nothing recorded yet. When the rover spots a fire, ' +
        'an obstacle or an animal, it will appear here.</li>';
      return;
    }
    $("events").innerHTML = rows.map(function (e) {
      var label = EVENT_LABEL[e.subtype] || "Event";
      var tone = EVENT_TONE[e.subtype] || "offline";
      // The sentence is built server-side from a fixed phrase table so that no
      // rover-generated prose can reach this page. Never replace it with a
      // field taken straight from the payload.
      var line = e.headline || (label + " reported.");
      return "<li>" +
        '<p class="item-head"><span class="tone t-' + esc(tone) + '">' + esc(label) +
          "</span> " + timeHtml(e.ts) + "</p>" +
        '<p class="item-body">' + esc(line) + "</p>" +
        '<p class="item-meta">Reported by ' + esc(e.thing || "a rover") + "</p></li>";
    }).join("");
  }

  function renderReports(d) {
    var rows = (d && d.reports) || [];
    if (!rows.length) {
      $("reports").innerHTML =
        '<li class="empty">No automatic checks have been recorded yet.</li>';
      return;
    }
    $("reports").innerHTML = rows.map(function (r) {
      var sev = r.severity || "unknown";
      return "<li>" +
        '<p class="item-head"><span class="tone t-' + esc(sev) + '">' +
          esc(SEVERITY_WORD[sev] || "Unknown") + "</span> " + timeHtml(r.ts) + "</p>" +
        '<p class="item-body">' + esc(r.summary || "") + "</p></li>";
    }).join("");
  }

  function loadCamera() {
    getJSON("/api/public/camera").then(function (c) {
      if (!c || !c.frame) return;
      $("cam-wrap").hidden = false;
      $("cam").src = "data:image/jpeg;base64," + c.frame;
      // A still from three hours ago must never be mistaken for a live view.
      var when = ago(c.ts);
      $("cam-note").textContent = when
        ? "Still picture, taken " + when + ". Not a live video feed."
        : "Still picture. Not a live video feed.";
    });
  }

  // --- polling --------------------------------------------------------------
  // Deliberately conservative. This page is public and may sit open on a wall
  // display for days; at 3 requests every 10s a single forgotten tab would burn
  // ~26k requests/day against a 100k/day ceiling and eventually take the
  // dashboard down for everyone. One combined request every 30s, paused
  // entirely while the tab is hidden, is roughly 1/20th of that.

  var PERIOD_MS = 30000;
  var MAX_WAIT_MS = 300000;
  var timer = null;
  var failures = 0;
  var lastGood = null;
  var lastGoodAt = 0;
  var inFlight = false;

  function render(d, fresh) {
    renderSummary(d, fresh);
    renderEvents(d);
    renderReports(d);
  }

  // A failed fetch must never blank the page. If we have shown data before,
  // keep it on screen and say plainly that it is what we last loaded.
  function renderFailure() {
    if (lastGood) {
      render(lastGood, false);
      $("b-stamp").textContent =
        "Could not reach the service just now. Showing what was loaded " +
        (ago(lastGoodAt) || "earlier") + ". Trying again shortly.";
      return;
    }
    setBanner("idle", "", "No connection",
      "Could not load the status",
      "This page could not reach the monitoring service. It may be a network " +
        "problem at your end. Use the Refresh button to try again.");
    $("b-stamp").textContent = "";
    $("rovers").innerHTML = '<p class="empty">Not loaded.</p>';
    $("events").innerHTML = '<li class="empty">Not loaded.</li>';
    $("reports").innerHTML = '<li class="empty">Not loaded.</li>';
  }

  function refresh() {
    if (inFlight) return Promise.resolve(null);
    inFlight = true;
    $("refresh").disabled = true;
    return getJSON("/api/public/all?limit=20").then(function (d) {
      inFlight = false;
      $("refresh").disabled = false;
      if (d) {
        failures = 0;
        lastGood = d;
        lastGoodAt = Date.now();
        render(d, true);
      } else {
        // Back off on repeated failure rather than hammering a struggling
        // origin — but never slower than 5 minutes, so it recovers on its own.
        failures = Math.min(failures + 1, 4);
        renderFailure();
      }
      return d;
    });
  }

  function schedule() {
    if (timer) clearTimeout(timer);
    timer = null;
    if (document.hidden) return;              // nobody is looking; stop asking
    var wait = Math.min(PERIOD_MS * Math.pow(2, failures), MAX_WAIT_MS);
    timer = setTimeout(function () { refresh().then(schedule); }, wait);
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

  $("refresh").addEventListener("click", function () {
    failures = 0;
    refresh().then(schedule);
  });

  // Relative times go stale on their own while the tab sits open between polls,
  // so re-render from the last good payload once a minute. No network cost.
  setInterval(function () {
    if (!document.hidden && lastGood && !inFlight) render(lastGood, false);
  }, 60000);

  refresh().then(schedule);
})();
</script>

<!--
  The assistant and the recordings player.

  A separate IIFE from the status poll above: it touches only #ai and #rec, it
  shares no state with the poll, and if either endpoint it depends on is missing
  the rest of the page is unaffected. Nothing here is on the critical path for
  answering "is anything on fire?".
-->
<script>
(function () {
  "use strict";
  var $ = function (id) { return document.getElementById(id); };

  /**
   * Seconds-or-milliseconds normaliser, the browser-side twin of toMs() in
   * public.js — same 1e12 split, same reasoning.
   *
   * It exists here because the recordings API is a separate handler and this
   * page must not assume which unit it chose. A seconds value treated as
   * milliseconds dates every recorded frame to January 1970; a milliseconds
   * value treated as seconds dates it to the year 33658. Either one turns an
   * honest capture date into nonsense, and the capture date is the main thing
   * stopping recorded material from reading as live.
   */
  function toMs(ts) {
    var n = Number(ts);
    if (!n || !isFinite(n)) return null;
    return n > 1e12 ? n : n * 1000;
  }

  /** Absolute, local, unambiguous. Recorded material never gets "just now". */
  function stampText(ms) {
    if (!ms) return null;
    try {
      return new Date(ms).toLocaleString(undefined, {
        year: "numeric", month: "short", day: "numeric",
        hour: "2-digit", minute: "2-digit", second: "2-digit"
      });
    } catch (e) { return null; }
  }

  function getJSON(path) {
    return fetch(path, { headers: { Accept: "application/json" } })
      .then(function (r) { return r.ok ? r.json() : null; })
      .catch(function () { return null; });
  }

  function elem(tag, cls, text) {
    var n = document.createElement(tag);
    if (cls) n.className = cls;
    if (text !== undefined && text !== null) n.textContent = String(text);
    return n;
  }

  /* ====================================================================
     THE ASSISTANT
     ====================================================================
     The one part of this system that is at its most useful when the rover
     is dark: it answers from stored project facts and recorded history, so
     it does not need anything to be powered on.

     Model output is written with textContent, never innerHTML. The server
     already constrains what the model may say; this side guarantees that
     whatever comes back is displayed as text and can never become markup,
     a link or a script in this page.
     ==================================================================== */

  (function assistant() {
    var form = $("ai-form");
    if (!form) return;

    /* Questions a judge, a teacher or a visitor actually asks. Loaded as
       buttons because a blank box with a cursor in it gets used by nobody. */
    var SUGGESTIONS = [
      "What is FPMS and what does it do?",
      "What has the rover detected so far?",
      "How does it spot a fire?",
      "How does the rover avoid obstacles?",
      "Why is the rover offline right now?",
      "What competition was this built for?"
    ];

    var MAX_CHARS = 400;      /* mirrors MAX_MESSAGE_CHARS on the server */
    var HISTORY_MSGS = 6;     /* three exchanges; the server trims further */
    var HISTORY_CHARS = 280;  /* mirrors MAX_HISTORY_CHARS, keeps the body small */

    var history = [];
    var busy = false;
    var probed = false;
    var disabled = false;

    function setDisabled(reason) {
      disabled = true;
      $("ai-input").disabled = true;
      $("ai-send").disabled = true;
      var chips = $("ai-chips").getElementsByTagName("button");
      for (var i = 0; i < chips.length; i++) chips[i].disabled = true;
      $("ai-foot").textContent = reason;
    }

    /* One cacheable GET, on first interaction rather than on load, so a visitor
       who never opens the assistant never costs a request for it. */
    function probe() {
      if (probed) return;
      probed = true;
      getJSON("/api/public/chat").then(function (doc) {
        if (doc && doc.available === false) {
          setDisabled(
            "The assistant is switched off for this deployment. The record, the " +
            "charts and the recordings on this page all still work."
          );
        }
      });
    }

    function addTurn(who, cls) {
      var box = elem("div", "ai-turn " + cls);
      box.appendChild(elem("span", "ai-who", who));
      var p = elem("p", "ai-text", "");
      box.appendChild(p);
      $("ai-log").appendChild(box);
      return p;
    }

    /* SSE reader. Each event is one JSON object: {"t":"..."} for a chunk of
       text, {"done":true} at the end. Anything else is ignored rather than
       displayed — an unrecognised frame is not something to show a visitor. */
    function readStream(body, out) {
      var reader = body.getReader();
      var decoder = new TextDecoder();
      var buf = "";
      var text = "";

      function handleLine(line) {
        if (line.indexOf("data:") !== 0) return;
        var payload = line.slice(5).replace(/^\\s+/, "");
        if (!payload) return;
        var obj = null;
        try { obj = JSON.parse(payload); } catch (e) { return; }
        if (obj && typeof obj.t === "string") {
          text += obj.t;
          out.textContent = text;
        }
      }
      function drain() {
        var i;
        while ((i = buf.indexOf("\\n")) >= 0) {
          handleLine(buf.slice(0, i).replace(/\\r$/, ""));
          buf = buf.slice(i + 1);
        }
      }
      function pump() {
        return reader.read().then(function (res) {
          if (res.done) {
            if (buf) { handleLine(buf.replace(/\\r$/, "")); buf = ""; }
            return text;
          }
          buf += decoder.decode(res.value, { stream: true });
          drain();
          return pump();
        });
      }
      return pump().catch(function () { return text; });
    }

    function finish(question, out, text) {
      busy = false;
      if (!disabled) $("ai-send").disabled = false;
      var answer = String(text || "").trim();
      if (!answer) {
        /* Never a status code, never an exception string. A visitor is told
           what happened and what still works. */
        answer =
          "I could not answer that one just now. The record, the charts and the " +
          "recordings on this page are all still available \\u2014 please try again " +
          "in a moment.";
      }
      out.textContent = answer;
      history.push({ role: "user", content: question.slice(0, HISTORY_CHARS) });
      history.push({ role: "assistant", content: answer.slice(0, HISTORY_CHARS) });
      if (history.length > HISTORY_MSGS) history = history.slice(-HISTORY_MSGS);
      $("ai-live").textContent = "The assistant answered.";
    }

    function ask(raw) {
      if (busy || disabled) return;
      var question = String(raw || "").trim();
      if (!question) return;
      if (question.length > MAX_CHARS) question = question.slice(0, MAX_CHARS);

      probe();
      busy = true;
      $("ai-send").disabled = true;
      $("ai-input").value = "";
      addTurn("You asked", "ai-you").textContent = question;
      var out = addTurn("FPMS assistant", "ai-bot");
      out.textContent = "Thinking\\u2026";
      $("ai-live").textContent = "Asking the assistant.";

      fetch("/api/public/chat", {
        method: "POST",
        headers: { "Content-Type": "application/json", "Accept": "text/event-stream" },
        body: JSON.stringify({ message: question, history: history })
      }).then(function (r) {
        var ct = r.headers.get("Content-Type") || "";
        /* Streaming is the documented default. Fall back to reading the whole
           body as JSON for any deployment or intermediary that does not stream
           — the endpoint answers both shapes and every failure path of it
           carries an honest sentence rather than an error code. */
        if (ct.indexOf("text/event-stream") >= 0 && r.body && r.body.getReader) {
          return readStream(r.body, out);
        }
        return r.json().then(function (j) {
          return j && typeof j.reply === "string" ? j.reply : "";
        }).catch(function () { return ""; });
      }).then(function (text) {
        finish(question, out, text);
      }).catch(function () {
        finish(question, out, "");
      });
    }

    /* chips */
    var ul = $("ai-chips");
    for (var i = 0; i < SUGGESTIONS.length; i++) {
      (function (q) {
        var li = document.createElement("li");
        var b = elem("button", "ai-chip", q);
        b.type = "button";
        b.addEventListener("click", function () { ask(q); });
        li.appendChild(b);
        ul.appendChild(li);
      })(SUGGESTIONS[i]);
    }

    form.addEventListener("submit", function (e) {
      e.preventDefault();
      ask($("ai-input").value);
    });
    $("ai-input").addEventListener("focus", probe);
    /* Enter sends, Shift+Enter makes a new line — what everyone expects. */
    $("ai-input").addEventListener("keydown", function (e) {
      if (e.key === "Enter" && !e.shiftKey) {
        e.preventDefault();
        ask($("ai-input").value);
      }
    });
  })();

  /* ====================================================================
     RECORDINGS
     ====================================================================
     Saved material from runs that have already finished.

     THE ONE RULE: recorded must never read as live.
       - a RECORDED badge is drawn INSIDE the media frame, so a cropped
         screenshot of a single frame still carries it;
       - every frame is captioned with the absolute date it was captured,
         never with a relative "just now";
       - the card wears a coloured rail and a 45 degree hatch that belongs
         to nothing else on the page;
       - the alt text and the SVG <title> both start with "Recorded", which
         is what a screen reader reads out;
       - playback starts PAUSED. An animation that begins on its own is the
         one thing that most looks like a live feed.

     SIMULATED IS A DIFFERENT CLAIM AND KEEPS A DIFFERENT LABEL. If the
     recordings API is not deployed, this falls back to /api/public/demo,
     which is a scripted dataset — it is labelled Simulated in the analytics
     section's amber and is never called a recording.
     ==================================================================== */

  (function recordings() {
    var grid = $("rec-grid");
    if (!grid) return;

    var SVGNS = "http://www.w3.org/2000/svg";
    var STEP_MS = 700;
    var MAX_FRAMES = 240;

    /* Only these two shapes are ever placed in an <img src>. A recording feed
       is data this page did not author, and "data:" URLs can carry markup. */
    var B64 = /^[A-Za-z0-9+/=]+$/;
    var DATA_URL = /^data:image\\/(jpeg|png|webp);base64,[A-Za-z0-9+/=]+$/;

    function imageSrc(v) {
      if (typeof v !== "string" || !v.length) return null;
      var s = v.replace(/\\s+/g, "");
      if (DATA_URL.test(s)) return s;
      if (B64.test(s) && s.length > 64) return "data:image/jpeg;base64," + s;
      return null;
    }

    /**
     * A sweep, normalised to an array of millimetres with null for "nothing
     * came back in that direction".
     *
     * TWO UNITS ARRIVE HERE AND THEY MUST NOT BE CONFUSED.
     *   - the recordings API publishes 'ranges_m' — METRES, one entry per five
     *     degree bin, with 0 meaning no return (it fills gaps with 0 and drops
     *     any scan that is all-zero);
     *   - the analytics demo publishes 'sectors_mm' — MILLIMETRES, twelve
     *     thirty degree sectors, with null meaning no return.
     * A metres array read as millimetres draws every wall 1000x too close,
     * which is the LiDAR equivalent of the seconds/milliseconds bug: it looks
     * like data rather than like an error. So the unit is decided by the FIELD
     * NAME, never guessed from the magnitude.
     */
    function sweep(raw) {
      if (!raw || typeof raw !== "object") return null;
      var src = null, toMm = null;
      if (Array.isArray(raw.ranges_m)) {
        src = raw.ranges_m;
        toMm = function (n) { return n * 1000; };
      } else if (Array.isArray(raw.sectors_mm) || Array.isArray(raw.sectors) ||
                 Array.isArray(raw.ranges_mm)) {
        src = raw.sectors_mm || raw.sectors || raw.ranges_mm;
        toMm = function (n) { return n; };
      }
      if (!src || !src.length || src.length > 1440) return null;

      var out = [], any = false;
      for (var i = 0; i < src.length; i++) {
        var n = Number(src[i]);
        /* 0 and null both mean "no return". Neither is a measured distance and
           neither may be drawn as one. */
        if (src[i] === null || src[i] === undefined || !isFinite(n) || n <= 0) {
          out.push(null);
        } else {
          var mm = toMm(n);
          if (mm > 100000) { out.push(null); continue; }
          out.push(mm);
          any = true;
        }
      }
      return any ? out : null;
    }

    /* The recordings API is written by a different module, so read it
       tolerantly: take the first array-shaped field that carries frames, and
       accept either name for a timestamp. Anything unrecognised is dropped,
       never guessed at. */
    function frameArray(doc) {
      var keys = ["frames", "scans", "samples", "items", "records", "points"];
      for (var i = 0; i < keys.length; i++) {
        var v = doc && doc[keys[i]];
        if (Array.isArray(v) && v.length) return v.slice(0, MAX_FRAMES);
      }
      return null;
    }

    function normalise(doc, kind) {
      var rows = frameArray(doc);
      if (!rows) return null;
      var frames = [];
      for (var i = 0; i < rows.length; i++) {
        var r = rows[i];
        if (!r || typeof r !== "object") continue;
        var t = toMs(r.ts !== undefined ? r.ts
          : (r.t !== undefined ? r.t : r.captured_at));
        if (kind === "camera") {
          var src = imageSrc(r.frame !== undefined ? r.frame
            : (r.image !== undefined ? r.image : r.jpeg));
          if (src) frames.push({ t: t, src: src });
        } else {
          var sec = sweep(r);
          if (sec) frames.push({ t: t, sectors: sec });
        }
      }
      if (!frames.length) return null;
      /* A response that calls itself simulated is labelled simulated, whatever
         endpoint it came from. The claim travels with the data. */
      var sim = doc.simulated === true || doc.source === "simulated";
      return { kind: kind, frames: frames, simulated: sim };
    }

    /* ---------------------------------------------------- the polar plot */

    function polarSvg(sectors, scaleMm, size, simulated) {
      var cx = size / 2, cy = size / 2, R = size / 2 - 26;
      var svg = document.createElementNS(SVGNS, "svg");
      svg.setAttribute("viewBox", "0 0 " + size + " " + size);
      svg.setAttribute("width", String(size));
      svg.setAttribute("height", String(size));
      svg.setAttribute("role", "img");

      var hit = 0, nearest = null, i;
      for (i = 0; i < sectors.length; i++) {
        if (sectors[i] === null) continue;
        hit++;
        if (nearest === null || sectors[i] < nearest) nearest = sectors[i];
      }
      var word = simulated ? "Simulated" : "Recorded";
      var title = document.createElementNS(SVGNS, "title");
      title.textContent = word + " laser sweep: " + hit + " of " + sectors.length +
        " sectors returned a range" +
        (nearest === null ? "." : ", nearest " + Math.round(nearest) + " millimetres.");
      svg.appendChild(title);
      svg.setAttribute("aria-label", title.textContent);

      var line = function (attrs) {
        var n = document.createElementNS(SVGNS, "circle");
        for (var k in attrs) n.setAttribute(k, String(attrs[k]));
        return n;
      };
      var ink = getComputedStyle(document.body).getPropertyValue("--dim").trim() || "#888";
      var gridC = getComputedStyle(document.body).getPropertyValue("--line").trim() || "#444";
      var hue = getComputedStyle(document.body).getPropertyValue("--rec").trim() || "#4a3aa7";
      if (simulated) hue = getComputedStyle(document.body).getPropertyValue("--sim").trim() || "#a34a17";

      for (var ring = 1; ring <= 3; ring++) {
        svg.appendChild(line({
          cx: cx, cy: cy, r: (R * ring) / 3, fill: "none", stroke: gridC, "stroke-width": 1
        }));
        var lab = document.createElementNS(SVGNS, "text");
        lab.setAttribute("x", String(cx + 4));
        lab.setAttribute("y", String(cy - (R * ring) / 3 - 3));
        lab.setAttribute("font-size", "10");
        lab.setAttribute("fill", ink);
        lab.textContent = Math.round((scaleMm * ring) / 3) + " mm";
        svg.appendChild(lab);
      }

      var step = (Math.PI * 2) / sectors.length;
      /* The 2px surface gap between adjacent fills, in radians — but never more
         than a fraction of the wedge itself. The recordings API publishes 72
         five degree bins, where a fixed 0.024 rad gap on each side would eat
         more than half of every wedge and the plot would read as a dotted ring
         rather than as a sweep. */
      var gap = Math.min(0.024, step * 0.12);

      if (sectors.length > 24) {
        /* Many narrow bins: draw the sweep as one silhouette. Individual
           wedges at this width are thinner than their own outline. A gap in
           the scan breaks the outline rather than being filled across, so a
           direction that returned nothing still reads as nothing. */
        var run = null;
        var flushRun = function () {
          if (!run || run.length < 2) { run = null; return; }
          var d = "M" + cx + "," + cy;
          for (var q = 0; q < run.length; q++) d += " L" + run[q][0] + "," + run[q][1];
          d += " Z";
          var poly = document.createElementNS(SVGNS, "path");
          poly.setAttribute("d", d);
          poly.setAttribute("fill", hue);
          poly.setAttribute("fill-opacity", "0.30");
          poly.setAttribute("stroke", hue);
          poly.setAttribute("stroke-width", "2");
          poly.setAttribute("stroke-linejoin", "round");
          svg.appendChild(poly);
          run = null;
        };
        for (i = 0; i < sectors.length; i++) {
          if (sectors[i] === null) { flushRun(); continue; }
          var rr = Math.max(4, Math.min(R, (sectors[i] / scaleMm) * R));
          var aa = -Math.PI / 2 + step * (i + 0.5);
          if (!run) run = [];
          run.push([cx + Math.cos(aa) * rr, cy + Math.sin(aa) * rr]);
        }
        flushRun();
      } else {
        for (i = 0; i < sectors.length; i++) {
          var a0 = -Math.PI / 2 + step * i + gap;
          var a1 = -Math.PI / 2 + step * (i + 1) - gap;
          var v = sectors[i];
          var rOut = v === null ? R : Math.max(6, Math.min(R, (v / scaleMm) * R));
          var p = document.createElementNS(SVGNS, "path");
          p.setAttribute("d",
            "M" + cx + "," + cy +
            " L" + (cx + Math.cos(a0) * rOut) + "," + (cy + Math.sin(a0) * rOut) +
            " A" + rOut + "," + rOut + " 0 0 1 " +
            (cx + Math.cos(a1) * rOut) + "," + (cy + Math.sin(a1) * rOut) + " Z");
          p.setAttribute("fill", v === null ? "transparent" : hue);
          p.setAttribute("fill-opacity", v === null ? "0" : "0.30");
          p.setAttribute("stroke", v === null ? gridC : hue);
          p.setAttribute("stroke-width", v === null ? "1" : "2");
          svg.appendChild(p);
        }
      }

      svg.appendChild(line({ cx: cx, cy: cy, r: 4, fill: ink }));
      var front = document.createElementNS(SVGNS, "text");
      front.setAttribute("x", String(cx));
      front.setAttribute("y", "13");
      front.setAttribute("text-anchor", "middle");
      front.setAttribute("font-size", "11");
      front.setAttribute("fill", ink);
      front.textContent = "0\\u00b0 (front)";
      svg.appendChild(front);

      /* The label is part of the picture, not part of the page around it. */
      var mark = document.createElementNS(SVGNS, "text");
      mark.setAttribute("x", String(cx));
      mark.setAttribute("y", String(cy + 6));
      mark.setAttribute("text-anchor", "middle");
      mark.setAttribute("font-size", "22");
      mark.setAttribute("font-weight", "800");
      mark.setAttribute("letter-spacing", "3");
      mark.setAttribute("fill", hue);
      mark.setAttribute("fill-opacity", "0.22");
      mark.setAttribute("transform", "rotate(-24 " + cx + " " + (cy + 6) + ")");
      mark.textContent = simulated ? "SIMULATED" : "RECORDED";
      svg.appendChild(mark);

      return svg;
    }

    /* ------------------------------------------------------- the player */

    function player(spec, clip, footnote) {
      var sim = clip.simulated;
      var word = sim ? "Simulated" : "Recorded";
      var card = elem("article", "rec-card");

      var head = elem("h3", null);
      head.appendChild(elem("span", "rec-badge" + (sim ? " is-sim" : ""), word));
      head.appendChild(document.createTextNode(spec.title));
      card.appendChild(head);
      card.appendChild(elem("p", "rec-sub", spec.sub));

      var scroll = elem("div", "rec-scroll");
      var stage = elem("div", "rec-stage");
      scroll.appendChild(stage);
      card.appendChild(scroll);

      var badge = elem("span", "rec-stamp" + (sim ? " is-sim" : ""), word + " \\u2014 not live");
      var when = elem("span", "rec-when", "");
      stage.appendChild(badge);
      stage.appendChild(when);

      var media = elem("div", null);
      media.style.width = "100%";
      media.style.display = "flex";
      media.style.justifyContent = "center";
      stage.appendChild(media);

      /* One scale for every frame, so a wedge growing means the world changed
         and not that the axis did. */
      var scaleMm = 900;
      if (spec.kind === "lidar") {
        var top = 0;
        for (var f = 0; f < clip.frames.length; f++) {
          var s = clip.frames[f].sectors;
          for (var k = 0; k < s.length; k++) if (s[k] !== null && s[k] > top) top = s[k];
        }
        scaleMm = Math.max(300, Math.min(Math.round(top * 1.1) || 900, 6000));
      }

      var idx = 0;
      var timer = null;

      var table = null, tbody = null;
      if (spec.kind === "lidar") {
        var det = document.createElement("details");
        det.className = "fx-table";
        var sum = document.createElement("summary");
        sum.textContent = "This sweep as a table";
        det.appendChild(sum);
        table = document.createElement("table");
        var thead = document.createElement("thead");
        var hr = document.createElement("tr");
        var bins = clip.frames[0].sectors.length;
        hr.appendChild(elem("th", null, "Direction"));
        hr.appendChild(elem("th", null,
          bins > 24 ? "Nearest return in sector" : "Nearest return"));
        thead.appendChild(hr);
        table.appendChild(thead);
        tbody = document.createElement("tbody");
        table.appendChild(tbody);
        det.appendChild(table);
        card.appendChild(det);
      }

      function paint() {
        var fr = clip.frames[idx];
        media.textContent = "";
        if (spec.kind === "camera") {
          var img = document.createElement("img");
          img.decoding = "async";
          img.loading = "lazy";
          img.src = fr.src;
          img.alt = word + " still picture from the rover's forward-facing camera" +
            (stampText(fr.t) ? ", captured " + stampText(fr.t) : "") +
            ". This is saved material, not a live view.";
          media.appendChild(img);
        } else {
          var size = Math.max(220, Math.min(340, (stage.clientWidth || 300) - 8));
          media.appendChild(polarSvg(fr.sectors, scaleMm, size, sim));
          if (tbody) {
            /* The table is the non-visual route to the same sweep. Seventy-two
               five degree rows is a wall of numbers nobody reads, so anything
               finer than fifteen degrees is grouped into twelve thirty degree
               sectors reporting the NEAREST return in each — which is the
               number a reader of this plot actually wants. The column heading
               says so, so the table never implies a resolution it lacks. */
            tbody.textContent = "";
            var n = fr.sectors.length;
            var groups = n > 24 ? 12 : n;
            var per = n / groups;
            var wide = 360 / groups;
            for (var g = 0; g < groups; g++) {
              var near = null;
              for (var k2 = Math.floor(g * per); k2 < Math.floor((g + 1) * per); k2++) {
                var val = fr.sectors[k2];
                if (val === null) continue;
                if (near === null || val < near) near = val;
              }
              var tr = document.createElement("tr");
              tr.appendChild(elem("td", null,
                Math.round(g * wide) + "\\u00b0\\u2013" + Math.round((g + 1) * wide) + "\\u00b0"));
              tr.appendChild(elem("td", null,
                near === null ? "no return" : Math.round(near) + " mm"));
              tbody.appendChild(tr);
            }
          }
        }
        var st = stampText(fr.t);
        when.textContent = st ? st : "capture time not recorded";
        clock.textContent = "Frame " + (idx + 1) + " of " + clip.frames.length;
        scrub.value = String(idx);
      }

      var controls = elem("div", "rec-controls");
      var play = elem("button", null, "Play");
      play.type = "button";
      play.setAttribute("aria-label", "Play the " + word.toLowerCase() + " " + spec.title.toLowerCase());
      var scrub = document.createElement("input");
      scrub.type = "range";
      scrub.min = "0";
      scrub.max = String(clip.frames.length - 1);
      scrub.value = "0";
      scrub.step = "1";
      scrub.setAttribute("aria-label", "Position in the " + word.toLowerCase() + " sequence");
      var clock = elem("span", "rec-clock", "");
      controls.appendChild(play);
      controls.appendChild(scrub);
      controls.appendChild(clock);
      if (clip.frames.length < 2) {
        play.disabled = true;
        scrub.disabled = true;
      }
      card.appendChild(controls);

      function stop() {
        if (timer) { clearInterval(timer); timer = null; }
        play.textContent = "Play";
        play.setAttribute("aria-label", "Play the " + word.toLowerCase() + " sequence");
      }
      function start() {
        if (timer || clip.frames.length < 2) return;
        play.textContent = "Pause";
        play.setAttribute("aria-label", "Pause the " + word.toLowerCase() + " sequence");
        timer = setInterval(function () {
          idx = (idx + 1) % clip.frames.length;
          paint();
        }, STEP_MS);
      }
      play.addEventListener("click", function () { timer ? stop() : start(); });
      scrub.addEventListener("input", function () {
        stop();
        idx = Math.max(0, Math.min(clip.frames.length - 1, Number(scrub.value) || 0));
        paint();
      });
      document.addEventListener("visibilitychange", function () {
        if (document.hidden) stop();
      });

      var first = stampText(clip.frames[0].t);
      var last = stampText(clip.frames[clip.frames.length - 1].t);
      var meta = clip.frames.length + " frame" + (clip.frames.length === 1 ? "" : "s");
      if (first) meta += ", captured " + first + (last && last !== first ? " to " + last : "");
      meta += ". " + (sim
        ? "Scripted demonstration data. No measurement in it is real."
        : "Saved during a run that has already finished. This is not a live feed.");
      if (footnote) meta += " " + footnote;
      card.appendChild(elem("p", "rec-meta", meta));

      paint();
      /* The polar plot is sized in pixels against its container, and the
         container has no width until the card is in the document. Repaint once
         it is, and again when the viewport changes. */
      card.repaintChart = paint;
      return card;
    }

    var repaintTimer = null;
    function repaintAll() {
      var cards = grid.children;
      for (var i = 0; i < cards.length; i++) {
        if (typeof cards[i].repaintChart === "function") cards[i].repaintChart();
      }
    }
    window.addEventListener("resize", function () {
      if (repaintTimer) clearTimeout(repaintTimer);
      repaintTimer = setTimeout(repaintAll, 200);
    });

    function emptyCard(spec, headline, why) {
      var card = elem("article", "rec-card");
      var head = elem("h3", null, spec.title);
      card.appendChild(head);
      var box = elem("div", "rec-empty");
      box.appendChild(elem("b", null, headline));
      box.appendChild(document.createTextNode(why));
      card.appendChild(box);
      return card;
    }

    var SPECS = [
      {
        kind: "lidar",
        title: "Laser scans",
        sub: "One frame is one full sweep of the spinning laser scanner. " +
             "Straight up is the direction the rover was facing, and how far " +
             "the shape reaches in any direction is how far away the nearest " +
             "thing was. A notch means nothing came back that way."
      },
      {
        kind: "camera",
        title: "Camera frames",
        sub: "Still pictures the rover saved while it was out on patrol."
      }
    ];

    /* Ask for the real thing first. The endpoint is added by a separate module
       and may not be deployed yet, so its absence is a normal outcome here and
       not an error: getJSON returns null for a 404 and the fallback runs. */
    function load() {
      return Promise.all([
        getJSON("/api/public/recording?kind=lidar"),
        getJSON("/api/public/recording?kind=camera")
      ]).then(function (res) {
        return {
          lidar: normalise(res[0] || {}, "lidar"),
          camera: normalise(res[1] || {}, "camera")
        };
      });
    }

    /* Fallback. The scripted run is NOT a recording and is never presented as
       one — it comes back flagged simulated, so player() gives it the amber
       treatment and the word "Simulated" everywhere the word "Recorded" would
       otherwise appear. */
    function demoClip() {
      return getJSON("/api/public/demo").then(function (d) {
        if (!d || d.simulated !== true || d.source !== "simulated") return null;
        var rover = (d.rovers || [])[0];
        var lid = rover && rover.lidar;
        var sec = lid && sweep({ sectors_mm: lid.latest_sectors_mm });
        if (!sec) return null;
        var pts = (lid.points || []);
        var t = pts.length ? toMs(pts[pts.length - 1].t) : toMs(d.scenario && d.scenario.anchor);
        return { kind: "lidar", simulated: true, frames: [{ t: t, sectors: sec }] };
      });
    }

    function render(clips) {
      grid.textContent = "";
      var haveReal = !!(clips.lidar || clips.camera);

      if (clips.lidar) {
        grid.appendChild(player(SPECS[0], clips.lidar, ""));
      }
      if (clips.camera) {
        grid.appendChild(player(SPECS[1], clips.camera, ""));
      } else {
        grid.appendChild(emptyCard(SPECS[1],
          "No camera frames are published yet",
          "The rover's camera also points at a home, so saved pictures are only " +
          "published when the operator switches that on. Nothing is shown here " +
          "rather than something stood in for it."));
      }

      if (typeof requestAnimationFrame === "function") requestAnimationFrame(repaintAll);
      else repaintAll();

      if (haveReal) {
        /* One of the two cards can come back flagged simulated while the other
           is genuinely recorded. The section note must not claim more than the
           cards below it do, so it names both when both are present. */
        var anySim = (clips.lidar && clips.lidar.simulated) ||
                     (clips.camera && clips.camera.simulated);
        var allSim = (!clips.lidar || clips.lidar.simulated) &&
                     (!clips.camera || clips.camera.simulated);
        $("rec-note").textContent = allSim
          ? "Everything below is scripted demonstration data, labelled as such on " +
            "every frame. No measurement in it is real, and none of it is live."
          : (anySim
            ? "Saved material from patrols that have already finished, plus one " +
              "scripted demonstration. Each panel says which it is, on the frame " +
              "itself. None of it is live, and playback starts paused."
            : "Saved material from patrols that have already finished. Every frame " +
              "is stamped and dated on the picture itself. None of it is live, and " +
              "playback starts paused.");
        return;
      }

      /* Nothing recorded is published. Offer the scripted run instead, clearly
         labelled, rather than an empty panel. */
      return demoClip().then(function (clip) {
        $("rec-note").textContent =
          "No recorded runs have been published yet. Shown below instead is the " +
          "system's scripted demonstration \\u2014 it is simulated, not measured, " +
          "and it is labelled that way wherever it appears.";
        if (!clip) {
          grid.insertBefore(emptyCard(SPECS[0],
            "No saved scans yet",
            "Recorded sweeps appear here once a run has been archived. Until then " +
            "the charts above show the laser scanner's coverage over time."), grid.firstChild);
          return;
        }
        grid.insertBefore(
          player(SPECS[0], clip,
            "Recorded sweeps will replace this automatically once a run is archived."),
          grid.firstChild
        );
        if (typeof requestAnimationFrame === "function") requestAnimationFrame(repaintAll);
        else repaintAll();
      });
    }

    load().then(render).catch(function () {
      grid.textContent = "";
      grid.appendChild(emptyCard(SPECS[0], "Recordings could not be loaded",
        "This page could not reach the recordings service. Everything else on " +
        "the page is unaffected."));
    });
  })();
})();
</script>
${ANALYTICS_SCRIPT}
</body>
</html>`;

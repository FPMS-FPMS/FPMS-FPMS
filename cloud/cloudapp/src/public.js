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
 * 3. Assume this URL gets scraped. Responses are cached at the edge and every
 *    query is LIMIT-bounded, so a traffic spike cannot run up D1 reads.
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

function publicJson(obj, status = 200) {
  return new Response(JSON.stringify(obj), {
    status,
    headers: {
      "Content-Type": "application/json",
      // Public and cacheable, unlike the authenticated API which is no-store.
      "Cache-Control": `public, max-age=${PUBLIC_CACHE_S}`,
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
 * Fleet status. Reads the Durable Object for live state and falls back to the
 * D1 archive for last-known values, so the page still says something useful
 * when every rover is down — which is exactly when someone checks it.
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

  // Channels look like "camera:rover2" — the thing is the half after the colon.
  const snapSeen = toMs(snap?.last_message_at);
  for (const ch of (snap?.channels || [])) {
    const name = String(ch).split(":")[1];
    if (name) things.set(name, { thing: name, last_seen: snapSeen });
  }

  // Fill in anything the archive knows about but the hub has forgotten.
  if (env.DB) {
    try {
      const { results } = await env.DB
        .prepare("SELECT thing, MAX(ts) AS ts FROM readings GROUP BY thing LIMIT 50")
        .all();
      for (const r of results || []) {
        if (!r.thing) continue;
        const prev = things.get(r.thing);
        const ts = toMs(r.ts);
        if (!prev || (ts && (!prev.last_seen || ts > prev.last_seen))) {
          things.set(r.thing, { thing: r.thing, last_seen: ts });
        }
      }
    } catch (err) {
      console.error(JSON.stringify({
        message: "public summary: archive query failed",
        error: err instanceof Error ? err.message : String(err),
      }));
    }
  }

  const rovers = [...things.values()]
    .map((t) => ({
      thing: t.thing,
      status: liveness(t.last_seen, now),
      last_seen: t.last_seen || null,
    }))
    .sort((a, b) => a.thing.localeCompare(b.thing));

  // The most recent unresolved fire event, if any.
  let fire = { active: false, since: null };
  if (env.DB) {
    try {
      const row = await env.DB
        .prepare(
          "SELECT ts, subtype FROM readings WHERE kind = 'events' " +
          "AND subtype IN ('fire','fire_cleared') ORDER BY ts DESC LIMIT 1",
        )
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

/** Analyst report headlines — severity and summary only, never the findings. */
async function publicReports(env, url) {
  if (!env.DB) return publicJson({ reports: [] });
  const limit = Math.min(Math.max(Number(url.searchParams.get("limit") || 5), 1), 20);
  try {
    const { results } = await env.DB
      .prepare("SELECT ts, severity, summary FROM reports ORDER BY ts DESC LIMIT ?")
      .bind(limit)
      .all();
    return publicJson({
      reports: (results || []).map((r) => ({
        ts: toMs(r.ts), severity: r.severity, summary: r.summary,
      })),
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

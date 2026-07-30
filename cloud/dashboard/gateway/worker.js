/**
 * FPMS Gateway — a permanent front door for an impermanent tunnel.
 *
 * Cloudflare quick tunnels mint a new random *.trycloudflare.com hostname every
 * time cloudflared restarts, so any URL you hand out goes dead. This Worker sits
 * on a stable workers.dev address and forwards to whatever tunnel is current.
 *
 * The HQ laptop registers its tunnel URL on startup and re-registers every few
 * minutes as a heartbeat, so a missing heartbeat means HQ is genuinely offline
 * and we can say so instead of bouncing visitors into a Cloudflare 1033 error.
 *
 * It *proxies* rather than redirects, deliberately: the permanent hostname stays
 * the origin the browser sees, so the dashboard can be installed as a real app
 * ("Add to Home Screen") and keeps working after the tunnel rotates. A redirect
 * would install the app against the throwaway hostname and break on rotation.
 *
 * Proxying is cheap here because the rovers are event-based — camera frames
 * arrive as occasional base64 JPEGs over the telemetry WebSocket, not as a
 * continuous video stream, so no media firehose transits this Worker.
 *
 *   *    /*         → proxied to the live tunnel (HTTP + WebSocket), or a 503 page
 *   GET  /_status   → liveness JSON (no auth; leaks nothing but an age)
 *   POST /_register → { url }  (bearer auth) record current tunnel + heartbeat
 *   POST /_offline  →          (bearer auth) clear it, e.g. on a clean shutdown
 */

const KV_KEY = "current";

// A registered URL is trusted only if it is a Cloudflare quick tunnel. Without
// this the endpoint would be an open redirect for anyone who got the secret.
const ALLOWED_URL = /^https:\/\/[a-z0-9-]+\.trycloudflare\.com$/;

// Heartbeat is every 5 min; allow ~2.4 missed beats before declaring HQ down.
const STALE_AFTER_MS = 12 * 60 * 1000;

// Statuses Cloudflare's own edge generates when it can't reach the tunnel.
// These come from Cloudflare, never from the dashboard, so it's safe to
// reinterpret them without masking a genuine application error.
// Observed in practice: a dead quick tunnel produces 530 sometimes and a
// bodyless 502 other times, so both must be covered.
const ORIGIN_DOWN_STATUSES = new Set([502, 504, 521, 522, 523, 524, 530]);

export default {
  async fetch(request, env) {
    const url = new URL(request.url);

    if (url.pathname === "/_register" && request.method === "POST") {
      return register(request, env);
    }
    if (url.pathname === "/_offline" && request.method === "POST") {
      return goOffline(request, env);
    }
    if (url.pathname === "/_status") {
      return status(env);
    }
    return forward(request, env, url);
  },
};

function unauthorized() {
  return json({ ok: false, error: "unauthorized" }, 401);
}

function authorize(request, env) {
  const secret = env.FPMS_GATEWAY_SECRET;
  // Fail closed: an unconfigured secret must never mean "no auth required".
  if (!secret) return false;
  const header = request.headers.get("Authorization") || "";
  const supplied = header.startsWith("Bearer ") ? header.slice(7) : "";
  return timingSafeEqual(supplied, secret);
}

/** Constant-time compare so the secret isn't recoverable byte-by-byte. */
function timingSafeEqual(a, b) {
  if (a.length !== b.length) return false;
  let diff = 0;
  for (let i = 0; i < a.length; i++) diff |= a.charCodeAt(i) ^ b.charCodeAt(i);
  return diff === 0;
}

async function register(request, env) {
  if (!authorize(request, env)) return unauthorized();

  let body;
  try {
    body = await request.json();
  } catch {
    return json({ ok: false, error: "body must be JSON" }, 400);
  }

  const candidate = String(body?.url || "").replace(/\/+$/, "");
  if (!ALLOWED_URL.test(candidate)) {
    return json(
      { ok: false, error: "url must be an https://<name>.trycloudflare.com origin" },
      400,
    );
  }

  const record = { url: candidate, updated_at: Date.now() };
  await env.FPMS_KV.put(KV_KEY, JSON.stringify(record));
  return json({ ok: true, ...record });
}

async function goOffline(request, env) {
  if (!authorize(request, env)) return unauthorized();
  await env.FPMS_KV.delete(KV_KEY);
  return json({ ok: true, offline: true });
}

async function readRecord(env) {
  const raw = await env.FPMS_KV.get(KV_KEY);
  if (!raw) return null;
  try {
    const rec = JSON.parse(raw);
    if (!ALLOWED_URL.test(String(rec.url || ""))) return null;
    return rec;
  } catch {
    return null;
  }
}

async function status(env) {
  const rec = await readRecord(env);
  if (!rec) return json({ online: false, reason: "no tunnel registered" });
  const age = Date.now() - (rec.updated_at || 0);
  return json({
    online: age <= STALE_AFTER_MS,
    age_seconds: Math.round(age / 1000),
    stale_after_seconds: STALE_AFTER_MS / 1000,
  });
}

async function forward(request, env, url) {
  const rec = await readRecord(env);
  if (!rec) return offlinePage("HQ has never registered a tunnel, or was shut down cleanly.");

  const age = Date.now() - (rec.updated_at || 0);
  if (age > STALE_AFTER_MS) {
    const mins = Math.round(age / 60000);
    return offlinePage(
      `HQ stopped checking in about ${mins} minute${mins === 1 ? "" : "s"} ago. ` +
        `The laptop is probably asleep, off, or off the internet.`,
    );
  }

  const target = new URL(url.pathname + url.search, rec.url);

  // Pass the request straight through, method, headers and body intact. Cloudflare
  // handles a 101 Upgrade here too, which is what keeps the telemetry and terminal
  // WebSockets working through the proxy.
  const proxied = new Request(target, request);
  // Let the dashboard build correct absolute URLs and log the real visitor.
  proxied.headers.set("X-Forwarded-Host", url.host);
  proxied.headers.set("X-Forwarded-Proto", "https");

  try {
    // redirect:"manual" so an app-issued 3xx reaches the browser as-is instead
    // of being chased here, which would rewrite it onto the tunnel hostname.
    const response = await fetch(proxied, { redirect: "manual" });

    // A dead tunnel doesn't make fetch throw — Cloudflare's edge answers with
    // its own 5xx (530/1033 "origin unreachable"). Passing that through shows
    // visitors a raw Cloudflare error page during the few seconds it takes the
    // supervisor to rebuild the tunnel, so translate it into our own page.
    if (ORIGIN_DOWN_STATUSES.has(response.status)) {
      return offlinePage(
        `The tunnel stopped answering (HTTP ${response.status}). HQ usually ` +
          `rebuilds it within a few seconds — try again shortly.`,
      );
    }
    return response;
  } catch (err) {
    // Registered and fresh, but unreachable — tunnel died between heartbeats.
    return offlinePage(
      `HQ registered a tunnel that isn't answering right now (${err}). ` +
        `It may be mid-restart — this usually clears within a minute.`,
    );
  }
}

function json(obj, status = 200) {
  return new Response(JSON.stringify(obj), {
    status,
    headers: { "Content-Type": "application/json", "Cache-Control": "no-store" },
  });
}

function offlinePage(detail) {
  const html = `<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FPMS · HQ offline</title>
<style>
  :root { color-scheme: dark; }
  body { margin:0; min-height:100vh; display:grid; place-items:center;
         background:#07090d; color:#e2e8f0;
         font:16px/1.6 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif; }
  .card { max-width:32rem; margin:1.5rem; padding:2rem;
          border:1px solid rgba(255,255,255,.08); border-radius:16px;
          background:#0b0f16; box-shadow:0 20px 60px rgba(0,0,0,.5); }
  .dot { display:inline-block; width:.6rem; height:.6rem; border-radius:50%;
         background:#f59e0b; margin-right:.5rem; }
  h1 { margin:.4rem 0 .2rem; font-size:1.4rem; letter-spacing:-.01em; }
  p { color:#94a3b8; margin:.6rem 0; }
  .detail { color:#cbd5e1; background:rgba(255,255,255,.04);
            border-left:2px solid #f59e0b; padding:.7rem .9rem;
            border-radius:0 8px 8px 0; font-size:.92rem; }
  ul { color:#94a3b8; font-size:.92rem; padding-left:1.1rem; }
  .foot { margin-top:1.4rem; padding-top:1rem; font-size:.8rem; color:#64748b;
          border-top:1px solid rgba(255,255,255,.06); }
</style>
<div class="card">
  <div><span class="dot"></span><strong>FPMS</strong></div>
  <h1>HQ is offline</h1>
  <p>This link is permanent and still correct &mdash; there's just nothing
     answering on the other end right now.</p>
  <p class="detail">${detail}</p>
  <p>To bring it back:</p>
  <ul>
    <li>Wake the HQ laptop and sign in &mdash; it republishes automatically.</li>
    <li>Or run <code>Publish-Public-Persistent.bat</code> on that laptop.</li>
  </ul>
  <div class="foot">Reload this page once HQ is up. The address never changes,
    so you can keep it bookmarked.</div>
</div>`;
  return new Response(html, {
    status: 503,
    headers: {
      "Content-Type": "text/html; charset=utf-8",
      "Cache-Control": "no-store",
      "Retry-After": "60",
    },
  });
}

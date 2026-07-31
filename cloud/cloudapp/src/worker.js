/**
 * FPMS Cloud app — the public streaming deployment, entirely on Cloudflare.
 *
 * There is deliberately no MQTT here. Cloudflare retired its MQTT broker and
 * Workers cannot listen on a TCP port, so rovers publish over authenticated
 * HTTPS instead — which works from any cellular network, needs no certificates
 * on the device, and traverses carrier NAT without a tunnel.
 *
 *   POST /ingest        rover -> here.   Bearer FPMS_INGEST_TOKEN.
 *   GET  /ws/<channel>  viewer <- here.  Live envelopes, fanned out by a
 *                                        Durable Object.
 *   GET  /api/health    same shape the edge backend returns
 *   GET  /api/auth-status
 *   POST /api/login
 *   everything else     the built React frontend (static assets)
 *
 * The API shape mirrors the Python backend on purpose: the frontend is reused
 * byte-for-byte rather than forked, so a UI change ships to both apps at once.
 *
 * This app is the streaming half of FPMS. It has no terminal, no SSH, no LAN
 * scan and no provisioning — not because they're disabled, but because they
 * were never implemented here. Hardware control lives only in the edge app.
 */

import { DurableObject } from "cloudflare:workers";
import { runAgents, renderReport, reasonOverFindings } from "./agents.js";
import { handlePublic } from "./public.js";
import FPMS_INFO from "./info.json";

const COOKIE = "fpms_auth";
const TOKEN_TTL_S = 30 * 24 * 3600;

// Channels the frontend subscribes to are "<subtype>:<thing>", e.g. lidar:rover1.
const CHANNEL_RE = /^[a-z0-9_-]+:[a-z0-9_.-]+$/i;
const SUBTYPE_RE = /^[a-z0-9_-]+$/i;
const THING_RE = /^[a-z0-9_.-]+$/i;

// Paths served without a session, so the login screen itself can load.
const OPEN_PATHS = new Set(["/api/login", "/api/auth-status"]);

// 256 KB. NOTE: a camera item is ~28 KB (27 KB base64 frame + fields), so this
// caps a camera batch at ~9 items — the 100-item cap in ingest() is unreachable
// for camera and only binds for small payloads like pose.
const MAX_INGEST_BYTES = 256 * 1024;

// How fresh the last ingest must be for the feed to count as live. Anything
// older and /api/health reports disconnected rather than showing a frozen
// frame behind a green pill.
const INGEST_LIVE_WINDOW_S = 30;

// Publisher WebSocket frames skip ingest() and therefore skip MAX_INGEST_BYTES,
// so the Durable Object enforces its own cap. The platform allows 32 MiB, but
// nothing legitimate here exceeds a handful of frames.
const MAX_WS_INGEST_BYTES = 512 * 1024;

// How long a direct publisher "owns" a thing. While it is live, telemetry for
// the same thing arriving over the HTTP relay is dropped, so the rover and the
// laptop forwarder cannot both publish and double the write volume. Events are
// always accepted from either path — losing an alert is worse than a duplicate.
const PUBLISHER_OWNS_MS = 30_000;

// Persisting `latest:<channel>` on every single publish is what makes streaming
// unaffordable: ctx.storage.put() bills as a row written, and the free tier
// allows 100k/day. At 15 fps the old code wrote ~5.2M rows/day — 52x over.
// Viewers get live data from the in-memory map instead; storage is only a
// cold-start fallback, so it needs refreshing occasionally, not continuously.
const PERSIST_MIN_MS = 10_000;

// Frames are the bulk of the archive and are never read back from D1 — the
// dashboard reads them from the Durable Object's `latest:` key. Archiving them
// cost ~155 MB/day (and would be ~35 GB/day at 15 fps) against a 5 GB limit,
// for data nothing queries. Stored as a byte count instead.
const ARCHIVE_FRAMES = false;

// NOTE (next step, not done here): row COUNT is still one insert per reading,
// which at stream rate exceeds the 100k/day D1 row quota (each insert also
// writes the two index rows from schema.sql, so ~3 billed rows per reading).
// Sampling telemetry needs per-(thing,subtype) state, and that must live in the
// Durable Object — module-level mutable state in a Worker leaks across requests
// because isolates are reused. Frame stripping below is stateless and safe.

export default {
  async fetch(request, env, ctx) {
    const url = new URL(request.url);
    const path = url.pathname;

    try {
      if (path === "/ingest") return ingest(request, env, ctx);

      // Direct rover uplink. Registered here — ahead of the session auth gate —
      // because a rover authenticates with FPMS_INGEST_TOKEN, not the operator's
      // browser cookie. Same credential as POST /ingest, one persistent socket
      // instead of a TLS handshake per batch.
      if (path === "/ingest/ws") return ingestSocket(request, env);
      if (path === "/api/login") return login(request, env);
      if (path === "/api/auth-status") return authStatus(request, env);

      // The public read-only view. Deliberately ahead of the auth gate below,
      // and deliberately a separate module: it serves a whitelisted projection
      // of the data, never the raw telemetry the authenticated API returns.
      const pub = await handlePublic(request, env, url, path, hubStub);
      if (pub) return pub;

      // Everything below needs a session when a password is configured.
      const needsAuth = path.startsWith("/api/") || path.startsWith("/ws/");
      if (needsAuth && !OPEN_PATHS.has(path) && !(await authed(request, env))) {
        return json({ error: "unauthorized", reason: "password required" }, 401);
      }

      if (path.startsWith("/ws/")) return openSocket(request, env, path);
      if (path === "/api/health") return health(env);
      if (path === "/api/history") return history(env, url);
      if (path === "/api/reports") return reports(env, url);
      if (path === "/api/forget" && request.method === "POST") {
        return hubStub(env).fetch(`https://hub/forget?things=${url.searchParams.get("things") || ""}`);
      }
      if (path === "/api/analyze" && request.method === "POST") {
        // Manual trigger — the same code path the cron runs, so testing it
        // proves the scheduled behaviour rather than a parallel one.
        return json(await analyzeAndReport(env, { force: url.searchParams.get("force") === "1" }));
      }
      if (path === "/api/vision") return json(await vision(env, url));
      if (path === "/api/info") return json(FPMS_INFO);
      if (path === "/api/events") return events(env, url);
      if (path === "/api/analyst/status") return json(analystStatus());
      if (path === "/api/analyst/report") return json(await analystReport(env, url));
      if (path === "/api/alerts/status") return json(alertsStatus(env));
      if (path === "/api/network") return networkStub(url);

      // Static frontend. Assets are public: the React app renders its own
      // login screen, exactly as the edge app does.
      return env.ASSETS.fetch(request);
    } catch (err) {
      console.error(JSON.stringify({
        message: "unhandled error", path,
        error: err instanceof Error ? err.message : String(err),
      }));
      // Don't return the raw error to the caller — it can disclose internals.
      return json({ error: "internal server error" }, 500);
    }
  },

  /**
   * Cron entrypoint. Runs the agents on a schedule so problems surface even
   * when nobody has the dashboard open — which is most of the time, and
   * exactly when a fire would be missed.
   */
  async scheduled(event, env, ctx) {
    ctx.waitUntil(
      analyzeAndReport(env)
        .then((r) => console.log(JSON.stringify({
          message: "scheduled analysis", severity: r.severity,
          emailed: r.emailed, sample_size: r.sample_size,
        })))
        .catch((err) => console.error(JSON.stringify({
          message: "scheduled analysis failed",
          error: err instanceof Error ? err.message : String(err),
        }))),
    );
  },
};

/* ---------------------------------------------------------------- AI models */

/**
 * Vision model, chosen by measurement rather than reputation.
 *
 * The previous default, `@cf/llava-hf/llava-1.5-7b-hf`, was benchmarked against
 * generated 640x480 test frames and FAILED THE ONE CASE THAT MATTERS: shown a
 * grey smoke plume rising from a treeline with flame at its base, it described
 * the smoke and then answered NO. A false negative on visible smoke is far
 * worse than the false-positive problem the old prompt was written to fix.
 *
 * llama-4-scout was correct on 13/13 trials (clear scene, an orange-sunset
 * false-positive trap, and the smoke frame), byte-identical across repeats,
 * at 0.7-1.5s versus LLaVA's 2.2-3.2s. It also supports a real
 * `response_format: json_schema`, so the verdict arrives as a parsed object.
 *
 * Models deliberately NOT used:
 *  - `@cf/meta/llama-3.2-11b-vision-instruct` — licence-gated (error 5016
 *    requires submitting "agree" to Meta's licence, and asserts you are not
 *    domiciled in the EU). Accepting a licence for the operator is not ours to
 *    do. To enable it, run from a Worker: env.AI.run(<id>, { prompt: "agree" }).
 *    Note the old `npx wrangler ai run` instruction is stale — no such command.
 *  - `@cf/mistralai/mistral-small-3.1-24b-instruct` — accepts `guided_json`
 *    and silently ignores it, inventing its own shape. Exactly the schema drift
 *    we are trying to escape.
 *  - Reasoning models (glm-4.7-flash, nemotron-3, qwen3-30b, gemma-4) — they
 *    spend the whole token budget in `reasoning` and return content: null.
 *  - `@cf/unum/uform-gen2-qwen-500m` — was the fallback here, but it has been
 *    removed from the Workers AI catalog, so it was dead code.
 */
const VISION_MODEL = "@cf/meta/llama-4-scout-17b-16e-instruct";

/** Ungated prose-only fallback. Kept because it needs no licence acceptance. */
const LEGACY_VISION_MODEL = "@cf/llava-hf/llava-1.5-7b-hf";

const VISION_MODELS = [VISION_MODEL, LEGACY_VISION_MODEL];

/**
 * Text reasoning model — the "edge LM". Deliberately the SAME model as vision.
 *
 * `@cf/meta/llama-3.3-70b-instruct-fp8-fast` reasons better on numbers, but its
 * output costs 204,805 neurons/M against scout's 77,273. Free tier is 10,000
 * neurons/day with no rollover; pairing 70b with scout measured at ~8,800/day
 * (88%) before any per-alert bursts, whereas scout for both jobs lands at
 * ~6,700/day and leaves headroom. One model ID, one code path.
 */
const TEXT_MODEL = VISION_MODEL;

const VISION_PROMPT =
  "You are a wildfire monitoring analyst reviewing a rover camera frame.\n" +
  "Set hazard=true ONLY if you can actually see smoke, flame, glowing embers, " +
  "or active burning. Haze, shadow, sunlight, sunset colours, red or orange " +
  "objects, and ordinary indoor scenes are hazard=false.\n" +
  "Set kind to one of: smoke, flame, embers, none.\n" +
  "Describe the scene in two or three sentences.";

/** JSON.parse that never throws — models sometimes wrap output in ``` fences. */
function safeParseJson(s) {
  if (typeof s !== "string") return null;
  const cleaned = s.replace(/^\s*```(?:json)?\s*/i, "").replace(/\s*```\s*$/, "").trim();
  try {
    return JSON.parse(cleaned);
  } catch {
    // Last resort: the first balanced-looking object in the text.
    const m = cleaned.match(/\{[\s\S]*\}/);
    if (!m) return null;
    try { return JSON.parse(m[0]); } catch { return null; }
  }
}

/**
 * Vision-language description of the newest camera frame.
 *
 * The rover's YOLO tells you *what objects* are present; it cannot tell you
 * "smoke rising behind the treeline" or "the ground is scorched". That
 * judgement is what a VLM adds, and it's why this sits alongside detection
 * rather than replacing it — YOLO stays the fast, deterministic trigger.
 */
async function vision(env, url) {
  if (!env.AI) return { error: "Workers AI binding not configured" };

  const thing = url.searchParams.get("thing") || "rover2";
  const latest = await hubStub(env)
    .fetch(`https://hub/latest?channel=camera:${encodeURIComponent(thing)}`)
    .then((r) => r.json())
    .catch(() => null);

  const frame = latest?.data?.frame;
  if (!frame) return { thing, error: "no camera frame available yet" };

  const prompt = url.searchParams.get("prompt") || VISION_PROMPT;

  try {
    let lastErr = null;
    for (const model of VISION_MODELS) {
      try {
        const started = Date.now();
        const structured = model !== LEGACY_VISION_MODEL;

        let raw = "";
        let obj = null;

        if (structured) {
          // Chat-style multimodal input with a real JSON schema. The model
          // returns an already-parsed object, which is why the old
          // /HAZARD:\s*(YES|NO)/ regex is gone — there is no prose to parse.
          const result = await env.AI.run(model, {
            messages: [{
              role: "user",
              content: [
                { type: "text", text: prompt },
                { type: "image_url", image_url: { url: `data:image/jpeg;base64,${frame}` } },
              ],
            }],
            response_format: {
              type: "json_schema",
              json_schema: {
                type: "object",
                properties: {
                  hazard: { type: "boolean" },
                  kind: { type: "string" },
                  description: { type: "string" },
                },
                required: ["hazard", "description"],
              },
            },
            max_tokens: 256,
          });
          const r = result?.response;
          obj = typeof r === "string" ? safeParseJson(r) : r;
          raw = typeof r === "string" ? r : JSON.stringify(r ?? "");
        } else {
          // Legacy prose path, kept only for the ungated fallback model, which
          // takes raw bytes rather than a data URI.
          const bytes = Uint8Array.from(atob(frame), (c) => c.charCodeAt(0));
          const result = await env.AI.run(model, {
            image: [...bytes],
            prompt: `${prompt}\nReply with "HAZARD: YES" or "HAZARD: NO" then a short description.`,
            max_tokens: 256,
          });
          raw = (result?.description ?? result?.response ?? "").trim();
          const m = raw.match(/HAZARD:\s*(YES|NO)/i);
          if (m) {
            obj = {
              hazard: m[1].toUpperCase() === "YES",
              description: raw.replace(/HAZARD:\s*(YES|NO)\s*/i, "").trim(),
            };
          }
        }

        if (!obj && !raw) { lastErr = `${model} returned nothing`; continue; }

        // json_schema shapes the output but does NOT enforce `required`, so
        // every field needs a defensive default. Defaulting hazard to false is
        // deliberate: a false alarm every 15 minutes trains the operator to
        // ignore the alert entirely, and the deterministic event agents remain
        // the real trigger.
        return {
          thing,
          hazard: obj?.hazard === true,
          kind: typeof obj?.kind === "string" ? obj.kind : null,
          verdict_parsed: Boolean(obj && typeof obj.hazard === "boolean"),
          description: typeof obj?.description === "string" && obj.description
            ? obj.description
            : raw.slice(0, 400),
          model,
          structured,
          latency_ms: Date.now() - started,
          frame_ts: latest.ts,
          detections: latest?.data?.detections ?? [],
        };
      } catch (e) {
        lastErr = `${model}: ${String(e).slice(0, 160)}`;
      }
    }
    return { thing, error: lastErr || "no vision model available" };
  } catch (err) {
    console.error(JSON.stringify({
      message: "vision failed",
      error: err instanceof Error ? err.message : String(err),
    }));
    return { thing, error: String(err).slice(0, 300) };
  }
}

/** Recent rover events, for the Overview feed. */
async function events(env, url) {
  if (!env.DB) return json({ events: [] });
  const limit = Math.min(Math.max(Number(url.searchParams.get("limit") || 25), 1), 200);
  try {
    const { results } = await env.DB
      .prepare("SELECT ts, thing, subtype, data FROM readings WHERE kind = 'events' ORDER BY ts DESC LIMIT ?")
      .bind(limit).all();
    return json({
      events: results.map((r) => ({
        ts: r.ts, thing: r.thing, subtype: r.subtype, data: JSON.parse(r.data),
      })),
    });
  } catch {
    return json({ events: [] });
  }
}

/**
 * Which analysis backends exist here. The cloud app runs the rule-based agents
 * only — Copilot and Bedrock are edge-app options that need credentials this
 * deployment deliberately does not hold.
 */
function analystStatus() {
  return {
    providers: { "local-rules": true, "github-copilot": false, bedrock: false },
    default_provider: "cloud-agents",
  };
}

/**
 * Adapt an agent run to the shape the Analyst page renders, so the cloud app
 * reuses the same UI rather than needing its own page.
 */
async function analystReport(env, url) {
  const thing = url.searchParams.get("thing") || "rover1";
  if (!env.DB) {
    return { thing, verdict: "NOMINAL", concerns: ["archive not configured"],
             report: "No archive bound.", provider: "cloud-agents",
             snapshot: { streams_live: {}, ages_s: {}, lidar: null, detections: [] } };
  }

  const snap = await hubStub(env).fetch("https://hub/snapshot").then((r) => r.json());
  const result = await runAgents(env.DB, snap.things_seen || []);

  const verdict = { ok: "NOMINAL", warning: "ELEVATED", critical: "CRITICAL" }[result.severity];
  const concerns = result.findings.filter((f) => f.severity !== "ok").map((f) => `${f.agent}: ${f.detail}`);

  // Stream liveness for this rover, derived from the newest reading per stream.
  const streams_live = {};
  const ages_s = {};
  const now = Date.now() / 1000;
  for (const s of ["lidar", "camera", "thermal", "pose"]) {
    const row = await env.DB
      .prepare("SELECT ts FROM readings WHERE thing = ? AND subtype = ? ORDER BY ts DESC LIMIT 1")
      .bind(thing, s).first();
    const age = row ? now - row.ts : null;
    ages_s[s] = age === null ? null : Math.round(age);
    streams_live[s] = age !== null && age < 120;
  }

  const detections = (result.findings.find((f) => f.agent === "events")?.alerts || [])
    .map((a) => ({ thing: a.thing, subtype: a.subtype, ...a.data }));

  return {
    thing, verdict, concerns,
    report: [result.summary, "", ...result.findings.map((f) => `[${f.severity}] ${f.agent}: ${f.detail}`)].join("\n"),
    provider: "cloud-agents",
    snapshot: { streams_live, ages_s, lidar: null, detections },
  };
}

/** Email configuration, in the shape the alerts card expects. */
function alertsStatus(env) {
  const ready = Boolean(env.RESEND_API_KEY && env.ALERT_EMAIL);
  return {
    configured: ready,
    provider: "resend",
    providers: {
      smtp: { ready: false, user: "" },
      resend: { ready, from: env.FPMS_MAIL_FROM || "onboarding@resend.dev" },
    },
    smtp_host: "", smtp_port: 0, smtp_user: "",
    alert_to: env.ALERT_EMAIL || "",
    cooldown_s: 0,
    settings_path: "cloudflare worker secrets",
    recent: [],
  };
}

/** Recent analysis reports: /api/reports?limit= */
async function reports(env, url) {
  if (!env.DB) return json({ error: "archive not configured" }, 503);
  const limit = Math.min(Math.max(Number(url.searchParams.get("limit") || 20), 1), 200);
  try {
    const { results } = await env.DB
      .prepare("SELECT ts, severity, summary, findings, emailed FROM reports ORDER BY ts DESC LIMIT ?")
      .bind(limit)
      .all();
    return json({
      count: results.length,
      reports: results.map((r) => ({
        ts: r.ts, severity: r.severity, summary: r.summary,
        emailed: !!r.emailed, findings: JSON.parse(r.findings),
      })),
    });
  } catch (err) {
    return json({ error: "query failed", detail: String(err) }, 500);
  }
}

/* ------------------------------------------------------------------ auth */

function hubStub(env) {
  // A single hub instance: the whole point is one fan-out point that every
  // viewer and every rover agrees on.
  return env.HUB.get(env.HUB.idFromName("global"));
}

async function hmac(env, value) {
  const key = await crypto.subtle.importKey(
    "raw",
    new TextEncoder().encode(env.FPMS_PASSWORD || "unset"),
    { name: "HMAC", hash: "SHA-256" },
    false,
    ["sign"],
  );
  const sig = await crypto.subtle.sign("HMAC", key, new TextEncoder().encode(value));
  return [...new Uint8Array(sig)].map((b) => b.toString(16).padStart(2, "0")).join("");
}

async function issueToken(env) {
  const exp = Math.floor(Date.now() / 1000) + TOKEN_TTL_S;
  return `${exp}.${await hmac(env, String(exp))}`;
}

async function validToken(env, token) {
  if (!token) return false;
  const [expStr, sig] = String(token).split(".");
  const exp = Number(expStr);
  if (!exp || !sig) return false;
  if (exp < Math.floor(Date.now() / 1000)) return false;
  return await timingSafeEqual(sig, await hmac(env, expStr));
}

/**
 * Constant-time secret comparison.
 *
 * Both values are hashed to a fixed 32 bytes first. Comparing the raw strings
 * would have to bail out early on a length mismatch, and that early return is
 * itself measurable — it leaks the length of the real token to anyone timing
 * the responses. Hashing removes the length signal entirely.
 */
async function timingSafeEqual(a, b) {
  const enc = new TextEncoder();
  const [x, y] = await Promise.all([
    crypto.subtle.digest("SHA-256", enc.encode(String(a ?? ""))),
    crypto.subtle.digest("SHA-256", enc.encode(String(b ?? ""))),
  ]);
  return crypto.subtle.timingSafeEqual(x, y);
}

function cookieValue(request, name) {
  const raw = request.headers.get("Cookie") || "";
  for (const part of raw.split(";")) {
    const [k, ...v] = part.trim().split("=");
    if (k === name) return v.join("=");
  }
  return null;
}

/**
 * Upgrade a rover into a publisher WebSocket on the hub.
 *
 * Why this exists: POST /ingest pays a TLS handshake per batch (80-150 ms on
 * WiFi, 250-600 ms on cellular) and burns one Worker request each time, which
 * caps the free tier at roughly 1 fps. A persistent socket pays the handshake
 * once and delivers straight into the Durable Object.
 *
 * Deliberate limit: this carries telemetry UP and rate hints DOWN. It is not a
 * command channel. The cloud app having no hardware control is a design
 * invariant (see the header comment and authStatus's controls_disabled), and
 * rate shaping is telemetry policy, not control.
 */
async function ingestSocket(request, env) {
  if (request.headers.get("Upgrade") !== "websocket") {
    return json({ error: "expected a websocket upgrade" }, 426);
  }
  if (!(await ingestAuthed(request, env))) {
    return json({ error: "unauthorized" }, 401);
  }

  const thing = String(new URL(request.url).searchParams.get("thing") || "");
  if (!THING_RE.test(thing)) {
    return json({ error: "a valid ?thing= is required" }, 400);
  }

  return hubStub(env).fetch(
    `https://hub/publisher?thing=${encodeURIComponent(thing)}`,
    request,
  );
}

/**
 * Rover authentication, shared by POST /ingest and the publisher WebSocket.
 *
 * Fails closed: an unset FPMS_INGEST_TOKEN must never mean "no auth required".
 * The token is only ever read from the Authorization header — never a query
 * string, which would land it in access logs and Workers traces.
 */
async function ingestAuthed(request, env) {
  const supplied = (request.headers.get("Authorization") || "").replace(/^Bearer\s+/i, "");
  if (!env.FPMS_INGEST_TOKEN) return false;
  return timingSafeEqual(supplied, env.FPMS_INGEST_TOKEN);
}

/**
 * Validate and normalise a batch of readings. Shared by the HTTP and WebSocket
 * ingest paths so there is exactly one definition of a well-formed item.
 * Throws on invalid input; callers turn that into a 400 or a socket error.
 */
function normalizeItems(items) {
  if (!Array.isArray(items)) throw new Error("expected an array of items");
  if (items.length > 100) throw new Error("batch too large (max 100)");
  return items.map((item) => {
    const thing = String(item?.thing ?? "");
    const subtype = String(item?.subtype ?? "");
    if (!THING_RE.test(thing) || !SUBTYPE_RE.test(subtype)) {
      throw new Error("each item needs a valid thing and subtype");
    }
    return {
      thing,
      subtype,
      kind: item?.kind === "events" ? "events" : "telemetry",
      data: item?.data ?? {},
    };
  });
}

async function authed(request, env) {
  if (!env.FPMS_PASSWORD) return true; // no password configured -> open
  const bearer = (request.headers.get("Authorization") || "").replace(/^Bearer\s+/i, "");
  // Boolean() matters: `bearer && ...` yields "" when the header is absent, and
  // that leaks into JSON as `"authenticated": ""` instead of false.
  return Boolean(
    (await validToken(env, cookieValue(request, COOKIE))) ||
    (bearer && (await validToken(env, bearer))),
  );
}

async function login(request, env) {
  if (request.method !== "POST") return json({ error: "method not allowed" }, 405);
  let body;
  try {
    body = await request.json();
  } catch {
    return json({ error: "body must be JSON" }, 400);
  }
  if (!env.FPMS_PASSWORD || !(await timingSafeEqual(String(body?.password ?? ""), env.FPMS_PASSWORD))) {
    return json({ error: "invalid password" }, 401);
  }
  const token = await issueToken(env);
  return new Response(JSON.stringify({ ok: true }), {
    status: 200,
    headers: {
      "Content-Type": "application/json",
      "Cache-Control": "no-store",
      // Secure is safe here: workers.dev is always HTTPS, unlike the edge app
      // which also has to work over plain HTTP on a LAN.
      "Set-Cookie": `${COOKIE}=${token}; Max-Age=${TOKEN_TTL_S}; Path=/; HttpOnly; Secure; SameSite=Lax`,
    },
  });
}

async function authStatus(request, env) {
  return json({
    auth_required: !!env.FPMS_PASSWORD,
    authenticated: await authed(request, env),
    is_lan: false,
    client_host: request.headers.get("CF-Connecting-IP") || null,
    is_remote: true,
    safe_mode: true,
    role: "cloud",
    // Always true: this app has no hardware-control routes at all.
    controls_disabled: true,
  });
}

/* --------------------------------------------------------------- ingest */

async function ingest(request, env, ctx) {
  if (request.method !== "POST") return json({ error: "method not allowed" }, 405);

  if (!(await ingestAuthed(request, env))) {
    return json({ error: "unauthorized" }, 401);
  }

  // Cap the body before reading it. A batch of 100 readings is a few KB; this
  // stops anyone making the Worker buffer megabytes toward its memory limit.
  const declared = Number(request.headers.get("Content-Length") || 0);
  if (declared > MAX_INGEST_BYTES) {
    return json({ error: `body too large (max ${MAX_INGEST_BYTES} bytes)` }, 413);
  }

  let body;
  try {
    const raw = await request.text();
    // Content-Length can be absent or lie under chunked encoding, so re-check
    // what actually arrived rather than trusting the header alone.
    if (raw.length > MAX_INGEST_BYTES) {
      return json({ error: `body too large (max ${MAX_INGEST_BYTES} bytes)` }, 413);
    }
    body = JSON.parse(raw);
  } catch {
    return json({ error: "body must be JSON" }, 400);
  }

  // Accept a single reading or a batch — batching is how a rover keeps its
  // request count (and radio time) down on a metered cellular link.
  const items = Array.isArray(body) ? body : [body];
  if (items.length > 100) return json({ error: "batch too large (max 100)" }, 413);

  let accepted;
  try {
    accepted = normalizeItems(items);
  } catch (e) {
    return json({ error: String(e.message || e) }, 400);
  }

  const res = await hubStub(env).fetch("https://hub/publish", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(accepted),
  });

  // Archive and alerting happen after the response is on its way back: a rover
  // on a metered cellular link should not wait on our bookkeeping, and a
  // database hiccup must never make it think the reading was rejected and
  // re-send it.
  ctx.waitUntil(archive(env, accepted));
  const alerts = accepted.filter((i) => i.kind === "events" && i.data && i.data.alert);
  if (alerts.length) ctx.waitUntil(notify(env, alerts));

  return new Response(res.body, { status: res.status, headers: { "Content-Type": "application/json" } });
}

/** Append readings to the D1 archive. Best-effort by design — see ingest(). */
/**
 * Strip the base64 frame before archiving.
 *
 * Verified safe: `data.frame` is read only from the Durable Object's
 * `latest:<channel>` key (the /api/vision and /api/public/camera paths).
 * Nothing reads a frame back out of D1 — agents.js only looks at `fire_like`,
 * `fire_ratio` and `detections` on camera rows. So dropping the blob loses no
 * capability and reclaims the overwhelming majority of archive volume.
 *
 * If a frame archive is ever wanted, it belongs in R2, not D1.
 */
function archivable(data) {
  if (!data || typeof data !== "object") return data;
  if (ARCHIVE_FRAMES || typeof data.frame !== "string") return data;
  const { frame, ...rest } = data;
  return { ...rest, frame_bytes: frame.length };
}

async function archive(env, items) {
  if (!env.DB) return;
  try {
    const now = Date.now() / 1000;
    const stmt = env.DB.prepare(
      "INSERT INTO readings (ts, thing, subtype, kind, data) VALUES (?, ?, ?, ?, ?)",
    );
    await env.DB.batch(
      items.map((i) =>
        stmt.bind(
          typeof i.data?.ts === "number" ? i.data.ts : now,
          i.thing, i.subtype, i.kind, JSON.stringify(archivable(i.data)),
        ),
      ),
    );
  } catch (err) {
    // Structured so it's queryable in Workers Logs, not just readable.
    console.error(JSON.stringify({
      message: "archive failed", error: err instanceof Error ? err.message : String(err),
      items: items.length,
    }));
  }
}

/**
 * Send email via Resend. Returns {ok, id|error} — never throws, because a mail
 * failure must not take down ingest.
 *
 * onboarding@resend.dev is Resend's shared sender, which works without owning a
 * verified domain but can only deliver to the address that owns the Resend
 * account. Set FPMS_MAIL_FROM to a sender on your own verified domain to email
 * anyone else.
 */
async function sendEmail(env, subject, html, text) {
  if (!env.RESEND_API_KEY || !env.ALERT_EMAIL) {
    return { ok: false, error: "email not configured" };
  }
  try {
    const res = await fetch("https://api.resend.com/emails", {
      method: "POST",
      headers: {
        Authorization: `Bearer ${env.RESEND_API_KEY}`,
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        from: env.FPMS_MAIL_FROM || "FPMS Alerts <onboarding@resend.dev>",
        to: [env.ALERT_EMAIL],
        subject,
        html,
        text,
      }),
    });
    const body = await res.json().catch(() => ({}));
    if (!res.ok) {
      console.error(JSON.stringify({ message: "resend rejected", status: res.status, body }));
      return { ok: false, error: body?.message || `HTTP ${res.status}` };
    }
    return { ok: true, id: body?.id };
  } catch (err) {
    console.error(JSON.stringify({
      message: "resend request failed",
      error: err instanceof Error ? err.message : String(err),
    }));
    return { ok: false, error: String(err) };
  }
}

/**
 * Immediate email for a rover-raised alert — does not wait for the next cron.
 *
 * Rate-limited hard. A stuck sensor produces hundreds of identical alerts a
 * minute; emailing each one burns the provider's daily quota within minutes and
 * then a *genuine* fire alert cannot send at all. Observed exactly that: 330
 * obstacle events exhausted the quota. Suppressed alerts are still archived and
 * still counted, and the next email that does go out reports how many it stands
 * in for.
 */
async function notify(env, alerts) {
  const gate = await hubStub(env)
    .fetch(`https://hub/alert-gate?count=${alerts.length}`, { method: "POST" })
    .then((r) => r.json())
    .catch(() => ({ allowed: true, suppressed: 0 }));

  if (!gate.allowed) {
    console.log(JSON.stringify({
      message: "alert email suppressed by cooldown",
      suppressed_total: gate.suppressed, next_allowed_in_s: gate.next_in_s,
    }));
    return { ok: false, error: "suppressed by cooldown", suppressed: gate.suppressed };
  }

  const lines = alerts.map((a) => `${a.thing}: ${a.subtype} — ${JSON.stringify(a.data)}`);
  if (gate.suppressed > 0) {
    lines.unshift(`(${gate.suppressed} further alerts suppressed since the last email)`);
  }
  const html = `<div style="font-family:system-ui,sans-serif">
      <h2 style="color:#b91c1c;margin:0 0 8px">FPMS — rover alert</h2>
      <p style="color:#475569">A rover raised an alert. This is sent immediately,
         ahead of the next scheduled analysis.</p>
      <pre style="background:#f1f5f9;padding:12px;border-radius:8px;font-size:13px">${
        lines.map((l) => l.replace(/[<>&]/g, "")).join("\n")
      }</pre></div>`;

  const result = await sendEmail(env, "FPMS ALERT — rover raised an alert", html, lines.join("\n"));

  // Webhook stays supported for Slack/Discord alongside email.
  if (env.FPMS_ALERT_WEBHOOK) {
    try {
      await fetch(env.FPMS_ALERT_WEBHOOK, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ text: `FPMS ALERT\n${lines.join("\n")}`, alerts, source: "fpms-cloud" }),
      });
    } catch (err) {
      console.error(JSON.stringify({
        message: "alert webhook failed",
        error: err instanceof Error ? err.message : String(err),
      }));
    }
  }
  return result;
}

/**
 * Run the analysis agents, store the report, and email when it is not "ok".
 * Called by the cron trigger and by POST /api/analyze.
 */
async function analyzeAndReport(env, { force = false } = {}) {
  if (!env.DB) return { error: "archive not configured" };

  const snap = await hubStub(env).fetch("https://hub/snapshot").then((r) => r.json());
  const report = await runAgents(env.DB, snap.things_seen || []);

  // Ask the VLM what the newest frame actually shows. Rule-based agents know
  // the numbers; this is the only part that can say "smoke behind the ridge".
  // Best-effort: a slow or unavailable model must not delay an alert.
  for (const thing of (snap.things_seen || []).slice(0, 2)) {
    try {
      const v = await vision(env, new URL(`https://x/?thing=${encodeURIComponent(thing)}`));
      if (v.description) {
        report.findings.push({
          agent: `vision:${thing}`,
          // Trust the model's declared verdict, not keywords in its prose.
          severity: v.hazard ? "critical" : "ok",
          detail: v.description,
          hazard: v.hazard,
          verdict_parsed: v.verdict_parsed,
          model: v.model,
          latency_ms: v.latency_ms,
        });
      }
    } catch { /* vision is an enrichment, never a gate */ }
  }
  // Recompute after adding vision findings so a VLM fire sighting can raise it.
  const { worstSeverity } = await import("./agents.js");
  report.severity = worstSeverity(report.findings);

  // Edge LM pass — LAST, and deliberately after severity is final.
  //
  // It explains what the findings mean together; it does not get a vote on how
  // bad they are. Attaching it here rather than pushing a finding is what keeps
  // it out of worstSeverity() and out of the email trigger below. Returning
  // null is normal (no AI binding, over quota, model down) and the report is
  // complete without it.
  report.reasoning = await reasonOverFindings(env, report, { model: TEXT_MODEL });

  const { html, text } = renderReport(report);

  let emailed = false;
  let email = null;
  let gate = null;

  // Which conditions are firing, not how many times we have noticed them. A
  // persisting fault keeps the same signature; a NEW failing agent changes it
  // and pages immediately.
  const signature = report.findings
    .filter((f) => f.severity !== "ok")
    .map((f) => `${f.agent}:${f.severity}`)
    .sort()
    .join("|");

  // Only bother a human when something is actually wrong — an inbox full of
  // "all clear" is an inbox nobody reads when it finally matters. And only when
  // it is NEWS: see reportGate. Without that, one offline rover sent an email
  // every 15 minutes until the daily quota was gone.
  // Fail OPEN if the gate is unreachable — a missed fire alert is far worse
  // than a duplicate one. But say so loudly: a silently swallowed error here
  // means the flood protection is off and nobody knows.
  try {
    const gateRes = await hubStub(env).fetch(
      `https://hub/report-gate?severity=${encodeURIComponent(report.severity)}` +
      `&sig=${encodeURIComponent(signature)}`,
    );
    if (!gateRes.ok) {
      const body = await gateRes.text().catch(() => "");
      throw new Error(`report-gate HTTP ${gateRes.status}: ${body.slice(0, 80)}`);
    }
    gate = await gateRes.json();
  } catch (err) {
    console.error(JSON.stringify({
      message: "report gate unavailable — failing open, duplicate alerts possible",
      error: err instanceof Error ? err.message : String(err),
    }));
    gate = { allowed: true, reason: "gate unavailable — failing open" };
  }

  if (force || gate.allowed) {
    const recovered = gate.reason === "recovered";
    const subject = recovered
      ? `FPMS RECOVERED — ${report.summary.slice(0, 80)}`
      : `FPMS ${report.severity.toUpperCase()} — ${report.summary.slice(0, 80)}`;
    const note = gate.suppressed
      ? `\n\n(${gate.suppressed} identical report(s) suppressed since the last email.)`
      : "";
    email = await sendEmail(env, subject, html, text + note);
    emailed = email.ok;
  }

  await env.DB.prepare(
    "INSERT INTO reports (ts, severity, summary, findings, emailed) VALUES (?, ?, ?, ?, ?)",
  ).bind(report.ts, report.severity, report.summary, JSON.stringify(report.findings), emailed ? 1 : 0).run();

  return { ...report, emailed, email, gate };
}

/** Recent readings from the archive: /api/history?thing=&subtype=&limit= */
async function history(env, url) {
  if (!env.DB) return json({ error: "archive not configured" }, 503);

  const thing = url.searchParams.get("thing");
  const subtype = url.searchParams.get("subtype");
  const kind = url.searchParams.get("kind");
  const limit = Math.min(Math.max(Number(url.searchParams.get("limit") || 100), 1), 1000);

  // Built with bound parameters throughout — never string-concatenated, so a
  // crafted `thing` value cannot alter the query.
  const where = [];
  const binds = [];
  if (thing) { where.push("thing = ?"); binds.push(thing); }
  if (subtype) { where.push("subtype = ?"); binds.push(subtype); }
  if (kind) { where.push("kind = ?"); binds.push(kind); }
  const sql =
    "SELECT ts, thing, subtype, kind, data FROM readings" +
    (where.length ? " WHERE " + where.join(" AND ") : "") +
    " ORDER BY ts DESC LIMIT ?";
  binds.push(limit);

  try {
    const { results } = await env.DB.prepare(sql).bind(...binds).all();
    return json({
      count: results.length,
      readings: results.map((r) => ({
        ts: r.ts, thing: r.thing, subtype: r.subtype, kind: r.kind,
        data: JSON.parse(r.data),
      })),
    });
  } catch (err) {
    return json({ error: "query failed", detail: String(err) }, 500);
  }
}

/* ---------------------------------------------------------------- read */

async function openSocket(request, env, path) {
  const channel = decodeURIComponent(path.slice("/ws/".length));
  if (!CHANNEL_RE.test(channel) && channel !== "events") {
    return json({ error: "bad channel" }, 400);
  }
  if (request.headers.get("Upgrade") !== "websocket") {
    return json({ error: "expected a websocket upgrade" }, 426);
  }
  return hubStub(env).fetch(
    new Request(`https://hub/ws?channel=${encodeURIComponent(channel)}`, request),
  );
}

async function health(env) {
  const res = await hubStub(env).fetch("https://hub/snapshot");
  const snap = await res.json();
  // Mirror the edge backend's /api/health so the frontend's status pills work
  // without knowing which app it is talking to.
  //
  // `connected` is DERIVED, never hardcoded. It used to be a literal `true`,
  // which meant that when the relay stopped feeding this app the status pill
  // stayed green while the panels showed an indefinitely frozen camera frame —
  // a stale image asserting it was live. For a fire watch that is the worst
  // possible failure mode, so liveness is now computed from the last message.
  const lastAt = Number(snap.last_message_at) || 0;
  const ageS = lastAt ? (Date.now() / 1000) - lastAt : Infinity;

  return json({
    ok: true,
    mqtt: {
      connected: ageS < INGEST_LIVE_WINDOW_S,
      last_message_age_s: Number.isFinite(ageS) ? Math.round(ageS) : null,
      publishers: snap.publishers ?? null,
      host: "cloudflare-ingest",
      port: 443,
      tls: true,
      mode: "cloud",
      messages_seen: snap.messages_seen,
      things_seen: snap.things_seen,
      last_message_at: snap.last_message_at,
    },
    channels: snap.channels,
    aws: { mode: "cloud", endpoint: "", region: "", reachable: false },
  });
}

function networkStub(url) {
  // The frontend's share panel asks for this. There is no LAN or tunnel here,
  // so report the one URL that exists and no firewall.
  return json({
    urls: [{
      kind: "public",
      url: `${url.protocol}//${url.host}`,
      description: "Cloud deployment — reachable worldwide",
      recommended: true,
    }],
    firewall: { supported: false, allowed: null, rule_present: false, platform: "cloudflare" },
    tunnel: {
      running: false, url: null, started_at: null, password_set: true,
      cloudflared_available: false, recent_output: [],
      gateway_url: `${url.protocol}//${url.host}`,
      gateway_configured: true, gateway_published: true, gateway_error: null,
    },
  });
}

function json(obj, status = 200) {
  return new Response(JSON.stringify(obj), {
    status,
    headers: { "Content-Type": "application/json", "Cache-Control": "no-store" },
  });
}

/* ------------------------------------------------- Durable Object: hub */

export class TelemetryHub extends DurableObject {
  // extends DurableObject, not a hand-rolled class: the base class wires up
  // this.ctx and this.env, and platform features are added to it over time.

  // Hot state, so the streaming path does not touch durable storage on every
  // frame. Safe here in a way it would NOT be in a Worker: a Durable Object is
  // one addressable instance, whereas Worker isolates are reused across
  // unrelated requests. Class fields rather than a constructor so there is no
  // super(ctx, env) to keep in sync with the base class.
  //
  // This is a cache, never the source of truth: it is empty after hibernation
  // or eviction, and every read below falls back to storage.
  mem = new Map();                 // channel -> latest envelope
  lastPersist = new Map();         // channel -> Date.now() of last durable write
  lastCounterPersist = 0;
  memSeen = null;                  // null = not yet loaded this lifetime
  memThings = new Set();
  memLastAt = null;
  pubSeenAt = new Map();           // thing -> Date.now() of last direct publish

  async fetch(request) {
    const url = new URL(request.url);
    if (url.pathname === "/publish") return this.publish(request);
    if (url.pathname === "/publisher") return this.acceptPublisher(url);
    if (url.pathname === "/ws") return this.subscribe(url);
    if (url.pathname === "/snapshot") return this.snapshot();
    if (url.pathname === "/latest") {
      const ch = url.searchParams.get("channel");
      // Memory first: it holds the true latest between persists. Storage is the
      // cold-start fallback and may be up to PERSIST_MIN_MS behind.
      return Response.json(
        this.mem.get(ch) || (await this.ctx.storage.get(`latest:${ch}`)) || null,
      );
    }
    if (url.pathname === "/alert-gate") return this.alertGate(url);
    if (url.pathname === "/report-gate") return this.reportGate(url);
    if (url.pathname === "/forget") return this.forget(url);
    return new Response("not found", { status: 404 });
  }

  async publish(request) {
    // Everything reaching /publish came through the Worker's HTTP ingest, i.e.
    // the relay path.
    return Response.json(
      await this.publishItems(await request.json(), { viaRelay: true }),
    );
  }

  /**
   * The one publish core, shared by POST /publish (the HTTP relay path) and the
   * publisher WebSocket. Returns a plain object; callers wrap it if they need a
   * Response.
   *
   * `viaRelay` marks items that arrived over HTTP. When a rover is publishing
   * directly, relayed TELEMETRY for that same thing is dropped — otherwise the
   * laptop forwarder and the rover both publish the same readings, doubling the
   * write volume and making the UI jitter between two slightly different frames.
   * Events are never dropped: a duplicate alert is harmless, a missing one is not.
   */
  async publishItems(items, { viaRelay = false } = {}) {
    if (viaRelay) {
      const cutoff = Date.now() - PUBLISHER_OWNS_MS;
      items = items.filter(
        (i) => i.kind === "events" || !((this.pubSeenAt.get(i.thing) || 0) > cutoff),
      );
      if (!items.length) {
        return { ok: true, accepted: 0, superseded_by_direct_publisher: true };
      }
    }
    const now = Date.now() / 1000;

    // Load counters from storage once per instance lifetime, then keep them in
    // memory. Reading them on every publish was a billed storage read per frame.
    if (this.memSeen === null) {
      this.memSeen = (await this.ctx.storage.get("messages_seen")) || 0;
      this.memThings = new Set((await this.ctx.storage.get("things_seen")) || []);
    }
    let seen = this.memSeen;
    const things = this.memThings;
    const toArchive = [];
    let notified = 0;
    let allSockets = 0;

    for (const item of items) {
      const channel =
        item.kind === "events" ? "events" : `${item.subtype}:${item.thing}`;
      const envelope = {
        thing: item.thing,
        subtype: item.subtype,
        ts: now,
        data: item.data,
      };

      // FAN OUT FIRST, PERSIST SECOND.
      //
      // This used to `await storage.put()` before sending, so every viewer's
      // frame waited on a durable write. Sending first removes that write from
      // the latency path entirely.
      //
      // Tagged sockets: getWebSockets(tag) is why no subscriber map is needed,
      // and it keeps working across hibernation.
      const payload = JSON.stringify(envelope);
      const sockets = this.ctx.getWebSockets(channel);
      notified += sockets.length;
      allSockets = this.ctx.getWebSockets().length;
      for (const ws of sockets) {
        try {
          ws.send(payload);
        } catch {
          // A dead socket must not abort the publish for everyone else.
        }
      }

      // Serve /latest and new-viewer bootstrap from memory. Instance state is
      // legitimate here — a Durable Object is a single addressable instance,
      // unlike a Worker isolate.
      this.mem.set(channel, envelope);

      // Persist only occasionally. storage.put() bills as a row written and the
      // free tier allows 100k/day; writing on every frame was ~5.2M/day at
      // 15 fps. Events are always persisted — they are the audit trail.
      const lastPersist = this.lastPersist.get(channel) || 0;
      const nowMs = Date.now();
      const due = item.kind === "events" || nowMs - lastPersist >= PERSIST_MIN_MS;
      if (due) {
        this.lastPersist.set(channel, nowMs);
        await this.ctx.storage.put(`latest:${channel}`, envelope);
        // Items arriving over the publisher socket never pass through ingest(),
        // so nothing else would archive them. Reuse the same cadence: every
        // event, and telemetry only when it was due a persist anyway — which
        // is what keeps the D1 row count off the stream rate.
        if (!viaRelay) toArchive.push(item);
      }

      seen += 1;
      things.add(item.thing);
    }

    // Awaited rather than fired-and-forgotten: this only runs for events and
    // for one telemetry item per channel per PERSIST_MIN_MS, so it is off the
    // per-frame path already, and a silently dropped archive write would be
    // worse than the few ms it costs here.
    if (toArchive.length) {
      try {
        await archive(this.env, toArchive);
      } catch (err) {
        console.error(JSON.stringify({
          message: "direct-path archive failed",
          error: err instanceof Error ? err.message : String(err),
        }));
      }
    }

    // One throttled multi-key write instead of three puts per publish call.
    // `last_message_at` is also held in memory so liveness stays accurate
    // between persists — otherwise health() would report a stale age and
    // wrongly declare the feed dead.
    this.memSeen = seen;
    this.memLastAt = now;
    const nowMs = Date.now();
    if (nowMs - (this.lastCounterPersist || 0) >= PERSIST_MIN_MS) {
      this.lastCounterPersist = nowMs;
      await this.ctx.storage.put({
        messages_seen: seen,
        things_seen: [...things],
        last_message_at: now,
      });
    }

    return {
      ok: true, accepted: items.length, messages_seen: seen,
      // Diagnostics: distinguishes "nobody listening" from "tag lookup broken".
      sockets_notified: notified, sockets_open: allSockets,
    };
  }

  /**
   * Accept a rover as a publisher.
   *
   * The tag is "pub" (plus "pub:<thing>"), which deliberately contains no colon
   * pattern that could collide with a channel tag like "camera:rover2". If a
   * publisher were ever returned by getWebSockets(channel), publish() would echo
   * every frame straight back to the rover it came from.
   */
  async acceptPublisher(url) {
    const thing = url.searchParams.get("thing");
    if (!thing) return new Response("thing required", { status: 400 });

    const pair = new WebSocketPair();
    const [client, server] = Object.values(pair);

    this.ctx.acceptWebSocket(server, ["pub", `pub:${thing}`]);
    // Survives hibernation, unlike an in-memory map.
    server.serializeAttachment({ role: "publisher", thing });

    // Answered by the runtime without waking the DO, so rover keepalives cost
    // nothing. The viewer path's manual ping handler bills a request each time.
    this.ctx.setWebSocketAutoResponse(
      new WebSocketRequestResponsePair("ping", "pong"),
    );

    // Tell it immediately whether anyone is watching, so a rover connecting to
    // an unwatched fleet starts in trickle mode instead of streaming at full
    // rate into an empty room.
    try {
      server.send(JSON.stringify(this.rateCommandFor(thing)));
    } catch { /* client vanished during upgrade */ }

    return new Response(null, { status: 101, webSocket: client });
  }

  /**
   * What rate this thing's camera should send at.
   *
   * Viewer-gated on purpose, and it is the load-bearing part of staying inside
   * the free tier: Durable Object DURATION is billed, and an object that never
   * sleeps costs ~85% of the daily allowance on its own. With no viewers the
   * rover trickles, the object goes idle, and it stops being billed.
   *
   * Trickle is never zero — /api/vision, /api/public/camera and the 15-minute
   * cron all need a reasonably fresh frame even with nobody watching.
   */
  rateCommandFor(thing, exclude = null) {
    // `exclude` is the socket that is currently closing. getWebSockets() still
    // returns it during webSocketClose, so counting naively would see the
    // departing viewer as present and leave the rover streaming at full rate
    // into an empty room — which is precisely the billed-duration case this
    // gating exists to avoid. Excluding it explicitly is deterministic;
    // deferring the recount to a later tick is not.
    const viewers = this.ctx
      .getWebSockets(`camera:${thing}`)
      .filter((w) => w !== exclude).length;
    return viewers > 0
      ? { cmd: "rate", camera_fps: 5, viewers }
      : { cmd: "rate", camera_fps: 0.1, viewers: 0 };
  }

  /** Push the current rate to a thing's publishers after viewers change. */
  notifyPublishers(thing, exclude = null) {
    const msg = JSON.stringify(this.rateCommandFor(thing, exclude));
    for (const ws of this.ctx.getWebSockets(`pub:${thing}`)) {
      try { ws.send(msg); } catch { /* dead socket */ }
    }
  }

  async subscribe(url) {
    const channel = url.searchParams.get("channel");
    if (!channel) return new Response("channel required", { status: 400 });

    const pair = new WebSocketPair();
    const [client, server] = Object.values(pair);

    // Hibernatable: the DO can be evicted while sockets stay open, so idle
    // viewers cost nothing. The tag is how publish() finds this socket again.
    this.ctx.acceptWebSocket(server, [channel]);
    // Recorded so webSocketClose knows which camera lost a viewer without
    // having to scan every tag.
    server.serializeAttachment({ role: "viewer", channel });

    // Someone is now watching: if this is a camera channel, tell the rover it
    // can leave trickle mode and stream properly.
    if (channel.startsWith("camera:")) {
      try { this.notifyPublishers(channel.slice("camera:".length)); } catch { /* none */ }
    }

    // Bootstrap from memory when this instance is warm — it holds the true
    // latest, whereas storage can be up to PERSIST_MIN_MS behind. Falls back to
    // storage after hibernation or eviction.
    const latest =
      this.mem.get(channel) || (await this.ctx.storage.get(`latest:${channel}`));
    if (latest) {
      try {
        server.send(JSON.stringify(latest));
      } catch {
        /* client vanished between upgrade and first send */
      }
    }

    return new Response(null, { status: 101, webSocket: client });
  }

  async webSocketMessage(ws, message) {
    // Publisher or viewer? The attachment survives hibernation, so this is
    // reliable even after the object has been evicted and revived.
    let att = null;
    try { att = ws.deserializeAttachment(); } catch { att = null; }

    if (att?.role !== "publisher") {
      // Viewers stay strictly read-only — unchanged behaviour.
      if (message === "ping") {
        try { ws.send("pong"); } catch { /* ignore */ }
      }
      return;
    }

    // ---- publisher path ----
    // Messages from a rover bypass ingest() entirely, so MAX_INGEST_BYTES does
    // not apply. Enforce a cap here or a single frame could push the isolate
    // toward its 128 MB limit.
    const raw = typeof message === "string" ? message : null;
    if (raw === null) {
      try { ws.send(JSON.stringify({ error: "text frames only" })); } catch { /* ignore */ }
      return;
    }
    if (raw.length > MAX_WS_INGEST_BYTES) {
      console.error(JSON.stringify({
        message: "publisher frame too large", thing: att.thing, bytes: raw.length,
      }));
      try { ws.send(JSON.stringify({ error: "frame too large" })); } catch { /* ignore */ }
      return;
    }

    let items;
    try {
      items = normalizeItems(JSON.parse(raw));
    } catch (e) {
      try { ws.send(JSON.stringify({ error: String(e.message || e).slice(0, 120) })); } catch { /* ignore */ }
      return;
    }

    // A publisher may only speak for the thing it authenticated as. Without this
    // one compromised rover token could inject telemetry for the whole fleet.
    const own = items.filter((i) => i.thing === att.thing);
    if (!own.length) return;

    // Marks this thing as live on the direct path, which is how ingest() knows
    // to ignore the laptop relay for the same thing and avoid double-publishing.
    this.pubSeenAt.set(att.thing, Date.now());

    // No per-message ack: acking would double the billed message count for zero
    // benefit, since TCP already guarantees delivery order on this socket.
    await this.publishItems(own);
  }

  async webSocketError(ws, error) {
    console.error(JSON.stringify({
      message: "websocket error",
      error: error instanceof Error ? error.message : String(error),
    }));
  }

  async webSocketClose(ws, code, reason, wasClean) {
    let att = null;
    try { att = ws.deserializeAttachment(); } catch { att = null; }

    try { ws.close(code === 1006 ? 1000 : code, reason); } catch { /* already gone */ }

    if (att?.role === "publisher") {
      this.pubSeenAt.delete(att.thing);
      return;
    }

    // A viewer left. If it was the last one watching a camera, tell the rover to
    // drop to trickle so this object can go idle and stop being billed.
    const channel = att?.channel;
    if (channel && channel.startsWith("camera:")) {
      const thing = channel.slice("camera:".length);
      try { this.notifyPublishers(thing, ws); } catch { /* nothing watching */ }
    }
  }

  /**
   * Token gate for alert emails. Allows one every ALERT_COOLDOWN_MS and counts
   * what it held back, so the next message can say how many it represents.
   */
  async alertGate(url) {
    const ALERT_COOLDOWN_MS = 10 * 60 * 1000;
    const incoming = Number(url.searchParams.get("count") || 1);
    const now = Date.now();
    const last = (await this.ctx.storage.get("last_alert_email")) || 0;
    const suppressed = (await this.ctx.storage.get("alerts_suppressed")) || 0;

    if (now - last < ALERT_COOLDOWN_MS) {
      await this.ctx.storage.put("alerts_suppressed", suppressed + incoming);
      return Response.json({
        allowed: false,
        suppressed: suppressed + incoming,
        next_in_s: Math.ceil((ALERT_COOLDOWN_MS - (now - last)) / 1000),
      });
    }

    await this.ctx.storage.put("last_alert_email", now);
    await this.ctx.storage.put("alerts_suppressed", 0);
    return Response.json({ allowed: true, suppressed });
  }

  /**
   * Gate for scheduled-report emails — the uptime alerting path.
   *
   * WHY THIS EXISTS
   * ---------------
   * The cron emailed on every run where severity != "ok". A rover that goes
   * offline stays offline, so one unchanging fault produced an email every 15
   * minutes: 170 identical "rover2 no telemetry" messages in two days, against
   * a ~100/day Resend allowance. The mailbox becomes noise and, far worse, the
   * daily quota is spent — so a REAL fire alert cannot send. That is the same
   * failure the ingest-side cooldown was added to fix; this path never had one.
   *
   * The rule is state-change, not level:
   *   - severity dropped back to ok  -> send ONE recovery notice
   *   - the set of failing agents changed, or severity got worse -> send now
   *     (a fire starting while a rover is already flagged silent must not be
   *     swallowed by a cooldown)
   *   - otherwise, the same condition persisting -> at most one reminder per
   *     RENOTIFY_MS
   *
   * Net effect: an unchanging fault sends ~4 emails/day instead of 96, a
   * genuinely new condition still pages immediately, and recovery is reported.
   */
  async reportGate(url) {
    const RENOTIFY_MS = 6 * 60 * 60 * 1000;
    const RANK = { ok: 0, warning: 1, critical: 2 };

    const sig = url.searchParams.get("sig") || "";
    const severity = url.searchParams.get("severity") || "ok";
    const now = Date.now();

    const lastSig = (await this.ctx.storage.get("last_report_sig")) || "";
    const lastSeverity = (await this.ctx.storage.get("last_report_severity")) || "ok";
    const lastAt = (await this.ctx.storage.get("last_report_email_at")) || 0;
    const suppressed = (await this.ctx.storage.get("reports_suppressed")) || 0;

    const commit = async (reason) => {
      await this.ctx.storage.put({
        last_report_sig: sig,
        last_report_severity: severity,
        last_report_email_at: now,
        reports_suppressed: 0,
      });
      return Response.json({ allowed: true, reason, suppressed });
    };

    if (severity === "ok") {
      // Only worth an email if we had previously reported a problem.
      if (RANK[lastSeverity] > 0) return commit("recovered");
      await this.ctx.storage.put({
        last_report_sig: sig, last_report_severity: severity,
      });
      return Response.json({ allowed: false, reason: "still ok", suppressed });
    }

    if (sig !== lastSig) return commit("condition changed");
    if (RANK[severity] > RANK[lastSeverity]) return commit("severity increased");
    if (now - lastAt >= RENOTIFY_MS) return commit("periodic reminder");

    await this.ctx.storage.put("reports_suppressed", suppressed + 1);
    return Response.json({
      allowed: false,
      reason: "unchanged condition within re-notify window",
      suppressed: suppressed + 1,
      next_in_s: Math.ceil((RENOTIFY_MS - (now - lastAt)) / 1000),
    });
  }

  /** Drop rovers from the roster — used to clear test fixtures. */
  async forget(url) {
    const things = (url.searchParams.get("things") || "").split(",").filter(Boolean);
    const known = new Set((await this.ctx.storage.get("things_seen")) || []);
    for (const t of things) known.delete(t);
    await this.ctx.storage.put("things_seen", [...known]);
    for (const t of things) {
      const list = await this.ctx.storage.list({ prefix: "latest:" });
      for (const key of list.keys()) {
        if (key.endsWith(`:${t}`)) await this.ctx.storage.delete(key);
      }
    }
    return Response.json({ ok: true, things_seen: [...known] });
  }

  async snapshot() {
    const list = await this.ctx.storage.list({ prefix: "latest:" });

    // Prefer in-memory counters: between persists they are the accurate ones,
    // and health() derives liveness from last_message_at. Reading only storage
    // here would under-report freshness by up to PERSIST_MIN_MS and could
    // declare a live feed dead.
    const storedLastAt = (await this.ctx.storage.get("last_message_at")) || null;
    const lastAt = Math.max(Number(this.memLastAt) || 0, Number(storedLastAt) || 0) || null;

    // Channels known from storage plus any seen only in memory this lifetime.
    const channels = new Set(
      [...list.keys()].map((k) => k.slice("latest:".length)),
    );
    for (const ch of this.mem.keys()) channels.add(ch);

    return Response.json({
      messages_seen:
        this.memSeen !== null
          ? this.memSeen
          : (await this.ctx.storage.get("messages_seen")) || 0,
      things_seen:
        this.memThings.size
          ? [...this.memThings]
          : (await this.ctx.storage.get("things_seen")) || [],
      last_message_at: lastAt,
      // Lets the dashboard distinguish "no data" from "nobody publishing".
      publishers: this.ctx.getWebSockets("pub").length,
      channels: [...channels].sort(),
    });
  }
}

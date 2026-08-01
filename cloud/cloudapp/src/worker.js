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
import { runAgents, renderReport, reasonOverFindings, worstSeverity } from "./agents.js";
import { handlePublic } from "./public.js";
import { handleChat, CHAT_PATH } from "./chat.js";
import { handleAnalytics } from "./analytics.js";
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

// How old the newest camera frame may be before vision() refuses to look at it.
//
// `latest:camera:<thing>` is a durable key with no TTL, so a rover that went
// offline in March leaves its last frame readable forever. Without this gate the
// 15-minute cron paid a real, billed AI inference on that same frozen image
// every run — and worse, if the model ever read smoke in it, the operator was
// paged about a months-old scene indefinitely. Refusing early costs nothing and
// tells the truth: there is no recent frame, not "no hazard".
const VISION_MAX_FRAME_AGE_S = 300;

// A JPEG small enough to hit this is a black frame, a truncated write or a
// camera that failed to expose. Base64 inflates by 4/3, so ~4 KB of base64 is
// ~3 KB of JPEG — below anything a real 640x480 scene produces. Refusing costs
// nothing and stops the model narrating noise, which is where hallucinated
// smoke comes from: shown near-black input it fills in plausible detail.
const VISION_MIN_FRAME_B64 = 4096;

// TWO-STAGE DETECTION — see visionForThing().
//
// Stage 1 (triage) is a small fast VLM asked one high-recall question. Stage 2
// (confirm) is the big model, and runs ONLY when triage flags something. On a
// fire watch the overwhelming majority of frames are clear, so the expensive
// model is idle almost always and the alert path still gets its full judgement
// on the frames that matter.
//
// Moondream 3.1: 9B mixture-of-experts, 2B active, ~770 ms for a `query` task
// against scout's 0.7-1.5 s, and materially cheaper per call. It is deliberately
// NOT trusted to clear a frame on its own subtlety — it is tuned to over-report
// (answer YES when unsure) and stage 2 supplies all the precision.
const VISION_TRIAGE_MODEL = "@cf/moondream/moondream3.1-9B-A2B";

// Confirmation must clear ALL of these, not just `hazard: true`. Each one has
// caught a different failure: a model that says true with confidence 0.2, one
// that says true and then writes kind "none", and one that says true with an
// empty evidence string because it was pattern-matching the prompt rather than
// the image.
const VISION_MIN_CONFIDENCE = 0.7;
const VISION_MIN_EVIDENCE_CHARS = 24;
const VISION_HAZARD_KINDS = new Set(["smoke", "flame", "embers"]);

// How many consecutive runs may triage flag a frame that confirmation then
// rejects before the report says "warning" instead of staying silent.
//
// Asymmetric on purpose, the same way the rover-side fire detection is: one
// disagreement is noise (a sunset, a cloud) and must not page anybody, but the
// same disagreement three cycles running is a scene that keeps looking like
// smoke to a screening model — worth an operator's eyes, still not a fire
// alert. The streak resets the moment triage comes back clear.
const VISION_UNCONFIRMED_STREAK_WARN = 3;

// Cost discipline. Workers AI is billed and the free tier is 10,000 neurons/day
// with no rollover, so an unbounded /api/vision would be a self-inflicted denial
// of the alerting path: spend the quota on curiosity and a real fire cannot be
// analysed. Per-thing minimum interval between billed triage runs; a caller
// inside the window gets the previous verdict back, labelled `cached`.
const VISION_MIN_INTERVAL_S = 90;

// Hard daily ceiling on TRIAGE inferences, UTC-day reset. Confirmation passes
// are deliberately exempt: the budget may never be the reason a flagged frame
// goes unexamined. Capping the cheap always-on stage bounds routine spend
// without ever standing between a possible fire and a verdict.
const VISION_DAILY_TRIAGE_BUDGET = 400;

// Every external call gets a deadline. A model that hangs must not hold the
// cron invocation open until the platform kills it — that loses the report, the
// email and the prune pass together. On timeout the caller degrades to "no
// analysis available", never to a fabricated verdict.
const AI_TRIAGE_TIMEOUT_MS = 8_000;
const AI_CONFIRM_TIMEOUT_MS = 15_000;
const AI_REASON_TIMEOUT_MS = 12_000;
const HUB_TIMEOUT_MS = 5_000;

// Roster caps.
//
// Anyone holding the ingest token can invent thing/subtype names, and every new
// channel becomes a PERMANENT `latest:<channel>` storage key (~27 KB when it
// holds a camera frame) that snapshot() then lists on every public poll. Nothing
// reclaims those keys, so growth is one-way. These caps sit far above any real
// fleet — they exist to bound a typo or an abuse, not to limit legitimate use,
// and a name already known is never rejected.
const MAX_THINGS = 64;
const MAX_SUBTYPES = 32;

// "pub" is the WebSocket tag acceptPublisher() puts on a rover's OWN socket. An
// item with subtype "pub" builds channel "pub:<thing>", getWebSockets() then
// returns that publisher socket, and publishItems() echoes the frame straight
// back to the rover that sent it. Reserved, and enforced in normalizeItems().
const RESERVED_SUBTYPES = new Set(["pub"]);

// Retention. Nothing else in this system deletes anything, so without this the
// archive only ever grows: at stream rate `readings` reaches the 5 GB D1 ceiling
// and then every INSERT fails — ingest, events and alerting all stop together.
// Reports are kept longer because they are small and are the operator's history.
const READINGS_RETENTION_DAYS = 30;
const REPORTS_RETENTION_DAYS = 90;

// At most one prune pass per this interval, gated in the Durable Object. The
// cron fires 96 times a day; pruning that often is pure billed writes for no
// benefit, since a day of retention drift is invisible at 30-day granularity.
const PRUNE_MIN_INTERVAL_MS = 6 * 60 * 60 * 1000;

// Rows deleted per table per pass. Bounded so a large first backlog is worked
// off gradually instead of one statement timing out — and a statement that
// always times out prunes nothing at all, forever.
const PRUNE_BATCH = 2000;

// How many consecutive unchanged analysis runs may pass before a report row is
// written anyway. The cron runs every 15 minutes, so 24 is roughly a 6-hourly
// heartbeat: enough for "the watchdog is alive" to be provable from the table,
// without the 96 near-identical rows/day the old unconditional insert wrote.
const REPORT_HEARTBEAT_RUNS = 24;

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

      // Chat and analytics MUST be routed BEFORE handlePublic. That function
      // owns /api/public/* and answers 405 to every non-GET, so a POST /chat
      // registered after it would be rejected before ever arriving here.
      //
      // Both are public by design and sit ahead of the auth gate, but each
      // enforces its own limits: chat.js reserves a daily budget in D1 before
      // it will call a model and fails closed if that budget cannot be read,
      // and both serve only the same whitelisted projection public.js does.
      if (path === CHAT_PATH) return handleChat(request, env, ctx, hubStub);

      const analytics = await handleAnalytics(request, env, url, path, hubStub);
      if (analytics) return analytics;

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
        // Encoded, not interpolated raw: an operator-supplied `things` value
        // containing & or # would otherwise truncate the list at the DO or
        // smuggle an extra query parameter into the hub request.
        const forget = url.searchParams.get("things") || "";
        return hubStub(env).fetch(`https://hub/forget?things=${encodeURIComponent(forget)}`);
      }
      if (path === "/api/analyze" && request.method === "POST") {
        // Manual trigger — the same code path the cron runs, so testing it
        // proves the scheduled behaviour rather than a parallel one.
        // `ctx` is passed, never destructured: `const { waitUntil } = ctx`
        // loses the `this` binding and throws "Illegal invocation" at runtime.
        return json(await analyzeAndReport(env, {
          force: url.searchParams.get("force") === "1",
          ctx,
        }));
      }
      if (path === "/api/vision") return json(await vision(env, url, ctx));
      if (path === "/api/info") return json(FPMS_INFO);
      if (path === "/api/events") return events(env, url);
      if (path === "/api/analyst/status") return json(analystStatus());
      if (path === "/api/analyst/report") return json(await analystReport(env, url));
      if (path === "/api/alerts/status") return json(alertsStatus(env));
      if (path === "/api/network") return networkStub(url);

      // An unmatched /api/ path is an API error, not a page. Falling through to
      // ASSETS.fetch answered every typo'd or removed endpoint with index.html
      // and status 200, so callers saw a "successful" response full of HTML and
      // failed at JSON.parse instead of reading a clean 404.
      if (path.startsWith("/api/")) return json({ error: "not found", path }, 404);

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
      analyzeAndReport(env, { ctx })
        .then((r) => console.log(JSON.stringify({
          message: "scheduled analysis", severity: r.severity,
          emailed: r.emailed, sample_size: r.sample_size,
          vision: r.vision_stats,
        })))
        .catch((err) => console.error(JSON.stringify({
          message: "scheduled analysis failed",
          error: err instanceof Error ? err.message : String(err),
        }))),
    );

    // Retention rides the same cron rather than getting its own trigger: this
    // is the only unattended code path in the app, and pruning has to happen
    // whether or not anyone ever opens the dashboard. Its own throttle lives in
    // the Durable Object, so 96 runs a day produce at most a handful of passes.
    // Independent of the analysis promise on purpose — a failing agent run must
    // not also stop the archive from being trimmed.
    ctx.waitUntil(
      pruneArchive(env).catch((err) => console.error(JSON.stringify({
        message: "scheduled prune failed",
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

/**
 * There is deliberately NO fallback model.
 *
 * `@cf/llava-hf/llava-1.5-7b-hf` used to sit here as an "ungated prose-only
 * fallback". Two independent reasons it is gone, both worth keeping written
 * down so nobody re-adds it:
 *
 *  1. It has been REMOVED from the Workers AI catalog (same fate as
 *     uform-gen2-qwen-500m above), so every attempt was a guaranteed 404. Each
 *     vision failure therefore paid a second doomed round-trip before returning
 *     the error it already had — latency on the alert path for nothing.
 *  2. Even when it worked it failed the one case that matters — see the
 *     benchmark note above, where it described a smoke plume and then answered
 *     NO. A fallback that returns a confident false negative on visible smoke is
 *     worse than no answer at all: "vision unavailable" is honest, "no hazard"
 *     is a lie the operator will act on.
 *
 * The list stays an array so a genuinely better second model can be added later
 * without reshaping the loop below.
 */
const VISION_MODELS = [VISION_MODEL];

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

/**
 * STAGE 1 — triage. Deliberately biased towards YES.
 *
 * This question is not trying to be right; it is trying to never miss. It is
 * the cheap stage, it runs on every sampled frame, and the only decision it is
 * trusted to make alone is the NEGATIVE one — "there is plainly nothing here",
 * which is true of almost every frame a patrolling rover ever takes. Anything
 * else is handed to stage 2. Tell it to answer YES when unsure, because a
 * wasted confirmation costs one inference and a missed plume costs a forest.
 */
const VISION_TRIAGE_QUESTION =
  "You are screening a wilderness camera frame for a wildfire watch.\n" +
  "Is there ANY visible smoke, plume, rising haze column, flame, glowing " +
  "ember, or actively burning material anywhere in this image?\n" +
  "Answer YES if there is anything that could plausibly be smoke or fire, " +
  "even if you are not sure. Answer NO only if the scene is clearly free of " +
  "smoke and fire.\n" +
  "Reply with exactly one word: YES or NO.";

/**
 * STAGE 2 — confirmation. Deliberately biased towards NO.
 *
 * Its job is to REJECT, and it is told so. The enumerated non-hazards are not
 * padding: every one is a false positive a VLM has actually produced on an
 * outdoor camera. `evidence` exists to make a positive expensive to assert —
 * a model that cannot name a specific visible feature and say where it is has
 * not seen a fire, it has agreed with the question, and the code below throws
 * that answer away.
 */
const VISION_CONFIRM_PROMPT =
  "You are the CONFIRMATION stage of a wildfire detection system. A fast " +
  "screening model has flagged this rover camera frame as possibly showing " +
  "smoke or fire. Your job is to reject false alarms.\n\n" +
  "A false wildfire alert is far more damaging than a missed frame: it sends " +
  "people to an empty field and it teaches the operator to ignore the next " +
  "alert, which may be real.\n\n" +
  "Set hazard=true ONLY if you can point to a specific visible region of THIS " +
  "image that contains smoke, flame, glowing embers, or actively burning " +
  "material, and describe that region in `evidence`.\n\n" +
  "The following are NOT hazards. If this is what you are looking at, " +
  "hazard=false:\n" +
  "- sunset, sunrise, orange or red sky, golden hour light, coloured cloud\n" +
  "- fog, mist, low cloud, overcast sky, rain, dust, sea spray, snow\n" +
  "- red or orange objects: vehicles, clothing, signs, flowers, autumn foliage\n" +
  "- lens flare, glare, overexposure, headlights, street lights, screens\n" +
  "- steam, vehicle exhaust, condensation, and any indoor scene\n" +
  "- a fire you infer from context or expect to be there but cannot see\n\n" +
  "If you are not certain, answer hazard=false. Uncertainty is not a hazard.\n\n" +
  "Fields:\n" +
  "  hazard: true only under the rule above.\n" +
  "  kind: one of smoke, flame, embers, none.\n" +
  "  confidence: 0.0 to 1.0 — how sure you are the hazard is real, not how " +
  "confident you feel in general.\n" +
  "  evidence: when hazard is true, the specific visible feature and where in " +
  "the frame it appears. Empty string when hazard is false.\n" +
  "  description: two or three factual sentences describing the whole scene.";

/**
 * Race a promise against a deadline, without leaving anything floating.
 *
 * Two details matter. The timer is cleared in `finally`, so a fast success does
 * not hold the invocation open waiting for a timeout that will never fire. And
 * a no-op `.catch` is attached to the raced promise itself: when the deadline
 * wins, the loser is still in flight and its later rejection would otherwise
 * surface as an unhandled rejection with no context.
 */
async function withTimeout(promise, ms, label) {
  promise.catch(() => {});
  let timer;
  try {
    return await Promise.race([
      promise,
      new Promise((_, reject) => {
        timer = setTimeout(() => reject(new Error(`${label} timed out after ${ms}ms`)), ms);
      }),
    ]);
  } finally {
    clearTimeout(timer);
  }
}

/**
 * Cheap non-cryptographic fingerprint of a frame, for "have I already paid to
 * look at exactly this image?".
 *
 * FNV-1a over a stride rather than every byte: a 27 KB base64 string is 27,000
 * iterations of CPU on a path that runs on every poll, and two genuinely
 * different scenes differ in far more than one byte in seven. Not security —
 * nothing is authorised on this value, it only decides whether to spend an
 * inference — so a fast hash is the right one. Length is folded in so a
 * truncated frame cannot collide with the full one.
 */
function frameFingerprint(b64) {
  let h = 0x811c9dc5;
  for (let i = 0; i < b64.length; i += 7) {
    h ^= b64.charCodeAt(i);
    h = Math.imul(h, 0x01000193);
  }
  h ^= b64.length;
  return (h >>> 0).toString(36);
}

/** Ask the hub something, with a deadline. A cold DO must not hang the cron. */
async function hubFetch(env, path, init) {
  return withTimeout(hubStub(env).fetch(`https://hub${path}`, init), HUB_TIMEOUT_MS, `hub ${path}`);
}

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

/** Rover-side detector labels that mean "look at this frame properly". */
const ROVER_FIRE_LABEL_RE = /\b(fire|smoke|flame|ember|burn)/i;

/** HTTP entry point for /api/vision. Query params only; the work is below. */
async function vision(env, url, ctx) {
  return visionForThing(env, url.searchParams.get("thing") || "rover2", {
    ctx,
    // Operator override: skip the sampling gate and the frame cache. Still
    // counted against the daily budget, so it cannot be used to drain it
    // faster than the ceiling allows.
    force: url.searchParams.get("force") === "1",
    prompt: url.searchParams.get("prompt") || null,
  });
}

/**
 * Two-stage vision analysis of the newest camera frame.
 *
 * The rover's YOLO tells you *what objects* are present; it cannot tell you
 * "smoke rising behind the treeline" or "the ground is scorched". That
 * judgement is what a VLM adds, and it's why this sits alongside detection
 * rather than replacing it — YOLO stays the fast, deterministic trigger.
 *
 * THE CASCADE, AND WHY IT IS THIS WAY ROUND
 * -----------------------------------------
 * Stage 1 (moondream, high recall, cheap) looks at every sampled frame and is
 * trusted with exactly one decision: "this is plainly nothing". That is the
 * true answer for essentially every frame a patrolling rover ever takes, so
 * the expensive model is idle almost all the time.
 *
 * Stage 2 (llama-4-scout, high precision, expensive) runs only on the frames
 * stage 1 could not clear, and is the ONLY thing that can assert a hazard. It
 * is prompted to reject, and its answer then has to survive four independent
 * checks below before `hazard` is true. Two different models with opposite
 * biases must agree before anyone is woken up.
 *
 * ON KEYWORD MATCHING (see CLAUDE.md: severity must come from a structured
 * field): moondream's `query` task returns prose, with no json_schema option.
 * The prose is read here — but ONLY to decide whether to spend stage 2, and
 * biased so that anything other than a clean "NO" escalates. A misread costs
 * one extra inference; it can never produce a hazard, because severity still
 * comes exclusively from stage 2's structured fields. The failure this rule
 * was written about — scoring "there is no visible smoke" as critical — is
 * structurally impossible here.
 *
 * EVERY exit from this function is either a real verdict or an `error` field.
 * It never throws and never fabricates: "no analysis available" is an honest
 * answer, "no hazard" from a call that did not happen is a lie.
 */
async function visionForThing(env, thing, opts = {}) {
  const { ctx = null, force = false, prompt = null, escalate = false } = opts;
  if (!env.AI) return { thing, error: "Workers AI binding not configured" };

  let latest = null;
  try {
    latest = await hubFetch(env, `/latest?channel=camera:${encodeURIComponent(thing)}`)
      .then((r) => r.json());
  } catch (err) {
    console.error(JSON.stringify({
      message: "vision: could not read the latest frame", thing,
      error: err instanceof Error ? err.message : String(err),
    }));
    return { thing, error: "camera frame unavailable" };
  }

  const frame = latest?.data?.frame;
  if (typeof frame !== "string" || !frame) {
    return { thing, error: "no camera frame available yet" };
  }

  // A near-empty frame is a failed exposure or a truncated write, and a model
  // shown near-black input invents detail to fill it — which is precisely the
  // hallucinated plume this system must never produce. Refuse before spending.
  if (frame.length < VISION_MIN_FRAME_B64) {
    return { thing, error: "camera frame too small to analyse", frame_b64_len: frame.length };
  }

  // FRESHNESS GATE — before any env.AI.run, because the AI call is the billed
  // part and a stale frame cannot produce a useful verdict at any price.
  //
  // Timestamps are unix SECONDS throughout this codebase (see the hub's publish
  // path, which stamps Date.now() / 1000), so both sides of this subtraction are
  // seconds. Reject rather than answer: the caller gets "no recent camera frame"
  // — which analyzeAndReport correctly treats as "no vision finding" — instead
  // of a hazard verdict about a scene that may be months old.
  const frameTs = Number(latest?.ts);
  const frameAgeS = Number.isFinite(frameTs) && frameTs > 0
    ? Math.round(Date.now() / 1000 - frameTs)
    : null;
  if (frameAgeS === null || frameAgeS > VISION_MAX_FRAME_AGE_S) {
    return { thing, error: "no recent camera frame", frame_age_s: frameAgeS };
  }

  const detections = Array.isArray(latest?.data?.detections) ? latest.data.detections : [];
  const fp = frameFingerprint(frame);

  // SAMPLING GATE — the cost control. Refuses a frame byte-identical to the one
  // already analysed (a frozen camera is the common case, and re-buying the same
  // verdict is pure waste), enforces a minimum interval per rover, and holds a
  // hard daily ceiling on stage-1 calls.
  //
  // Fails towards LOOKING if the hub is unreachable: detection beats thrift, and
  // we only got this far because the same object just served us a frame, so this
  // branch means something is genuinely wrong and should be loud.
  let gate = { allowed: true, reason: "sampling gate unavailable" };
  if (!force) {
    try {
      gate = await hubFetch(env, `/vision-gate?thing=${encodeURIComponent(thing)}&fp=${fp}`)
        .then((r) => r.json());
    } catch (err) {
      console.error(JSON.stringify({
        message: "vision sampling gate unavailable — analysing anyway", thing,
        error: err instanceof Error ? err.message : String(err),
      }));
    }
  }
  if (!gate.allowed) {
    // A cached verdict is returned verbatim apart from `detections`, which are
    // re-attached live: the verdict is about the stored frame, the detections
    // describe the current one, and the caller wants both current.
    if (gate.cached) {
      return {
        ...gate.cached,
        thing,
        cached: true,
        cache_reason: gate.reason,
        detections,
        // The verdict is reused; the freshness is NOT. `frame_ts` is restated
        // from the frame in hand so a consumer reading "3 s old" is told the
        // truth about the evidence, not the age it had when it was analysed.
        frame_ts: frameTs,
        frame_age_s: frameAgeS,
        // Carried through so a run served from cache does not silently reset a
        // streak that has already reached the warning threshold.
        unconfirmed_streak: Number(gate.unconfirmed_streak) || 0,
      };
    }
    return { thing, error: "no analysis available", reason: gate.reason, frame_age_s: frameAgeS };
  }

  /* ------------------------------------------------- stage 1: triage */

  const yoloFlag = detections.some((d) =>
    ROVER_FIRE_LABEL_RE.test(String(d?.label ?? d?.class ?? d?.name ?? "")));

  const triage = {
    model: VISION_TRIAGE_MODEL, flagged: null, answer: null, latency_ms: null,
    error: null, reason: null,
  };
  let triageDescription = "";

  if (escalate || yoloFlag) {
    // The rover's own detector already fired, or the caller demanded the full
    // pass. Skip straight to confirmation — spending stage 1 to re-ask a
    // question already answered affirmatively is money for nothing.
    triage.flagged = true;
    triage.reason = yoloFlag ? "rover detector flagged fire/smoke" : "escalated by caller";
  } else {
    const t0 = Date.now();
    try {
      const res = await withTimeout(
        env.AI.run(VISION_TRIAGE_MODEL, {
          task: "query",
          image: `data:image/jpeg;base64,${frame}`,
          question: VISION_TRIAGE_QUESTION,
          // The reasoning trace is tokens we pay for and never read.
          reasoning: false,
          // `stream` DEFAULTS TO TRUE on this model. Without this the binding
          // hands back a ReadableStream and every field below reads undefined —
          // which would silently look like "model said nothing".
          stream: false,
          temperature: 0,
          max_tokens: 64,
        }),
        AI_TRIAGE_TIMEOUT_MS,
        "vision triage",
      );
      triage.latency_ms = Date.now() - t0;

      const answer = String(res?.answer ?? res?.response ?? res?.caption ?? "").trim();
      triage.answer = answer.slice(0, 120);
      const [verdictPart, ...rest] = answer.split("|");
      triageDescription = rest.join("|").trim();

      if (!answer) {
        triage.flagged = true;
        triage.reason = "triage returned nothing — escalating";
      } else {
        // Only a clean leading NO ends the run. Everything else — "yes",
        // "maybe", a hedge, a refusal, punctuation soup — goes to stage 2.
        triage.flagged = !/^\s*no\b/i.test(verdictPart);
        triage.reason = triage.flagged ? "triage flagged" : "triage clear";
      }
    } catch (e) {
      // A broken or slow triage model must never be able to clear a frame.
      triage.latency_ms = Date.now() - t0;
      triage.error = String(e).slice(0, 160);
      triage.flagged = true;
      triage.reason = "triage failed — escalating to confirmation";
    }
  }

  const base = {
    thing,
    // Both true on either branch, and both mean the same thing they always did:
    // a definite verdict was obtained, rather than the code falling back to a
    // default because nothing parsed. `stage` is what tells a consumer which
    // model produced it.
    structured: true,
    verdict_parsed: true,
    frame_ts: frameTs,
    // Reported alongside the verdict so a caller can see how fresh the
    // evidence was without re-deriving it from frame_ts.
    frame_age_s: frameAgeS,
    triage,
    cached: false,
  };

  if (!triage.flagged) {
    // Cleared cheaply. The one-line scene description from the same call is
    // what the dashboard shows and what analyzeAndReport files as an "ok"
    // finding, so a clear frame still produces evidence that we looked.
    const verdict = {
      ...base,
      hazard: false,
      kind: "none",
      confidence: null,
      evidence: null,
      description: triageDescription || "No smoke or fire visible in the current frame.",
      model: VISION_TRIAGE_MODEL,
      stage: "triage",
      confirmed: false,
      latency_ms: triage.latency_ms,
    };
    const rec = await recordVision(env, thing, fp, verdict, { ctx, needStreak: false });
    return { ...verdict, unconfirmed_streak: rec?.unconfirmed_streak ?? 0, detections };
  }

  /* -------------------------------------------- stage 2: confirmation */

  const confirmPrompt = prompt || VISION_CONFIRM_PROMPT;
  let lastErr = null;

  for (const model of VISION_MODELS) {
    let obj = null;
    let raw = "";
    let latencyMs = null;
    try {
      const started = Date.now();
      // Chat-style multimodal input with a real JSON schema. The model returns
      // an already-parsed object, which is why the old /HAZARD:\s*(YES|NO)/
      // regex is gone — there is no prose to parse.
      const result = await withTimeout(
        env.AI.run(model, {
          messages: [{
            role: "user",
            content: [
              { type: "text", text: confirmPrompt },
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
                confidence: { type: "number" },
                evidence: { type: "string" },
                description: { type: "string" },
              },
              required: ["hazard", "confidence", "description"],
            },
          },
          // Same numbers in, same verdict out. A fire watch that answers
          // differently on a re-run of the identical frame cannot be argued
          // with, and the whole gating design below assumes stability.
          temperature: 0,
          max_tokens: 384,
        }),
        AI_CONFIRM_TIMEOUT_MS,
        `vision confirm ${model}`,
      );
      latencyMs = Date.now() - started;

      const r = result?.response;
      obj = typeof r === "string" ? safeParseJson(r) : r;
      raw = typeof r === "string" ? r : JSON.stringify(r ?? "");
      if (!obj && !raw) { lastErr = `${model} returned nothing`; continue; }
    } catch (e) {
      lastErr = `${model}: ${String(e).slice(0, 160)}`;
      continue;
    }

    // json_schema shapes the output but does NOT enforce `required`, so every
    // field is defaulted rather than trusted.
    const kind = typeof obj?.kind === "string" ? obj.kind.trim().toLowerCase() : "";
    const evidence = typeof obj?.evidence === "string" ? obj.evidence.trim() : "";
    const confidence = Number(obj?.confidence);
    const claimed = obj?.hazard === true;

    // FOUR INDEPENDENT CHECKS, ALL REQUIRED.
    //
    // Each has caught a different real failure: a model asserting true at
    // confidence 0.2; asserting true and then naming kind "none"; asserting
    // true with an empty `evidence` because it was agreeing with the question
    // rather than reading the image; and asserting true with no confidence
    // field at all. A missing confidence is treated as a failure on purpose —
    // if that ever becomes systematic it shows up as the unconfirmed streak
    // below escalating to a warning, which is visible, rather than as silence.
    const checks = {
      model_asserted_hazard: claimed,
      kind_is_a_hazard: VISION_HAZARD_KINDS.has(kind),
      confidence_at_least_threshold: Number.isFinite(confidence) && confidence >= VISION_MIN_CONFIDENCE,
      evidence_is_specific: evidence.length >= VISION_MIN_EVIDENCE_CHARS,
    };
    const hazard = Object.values(checks).every(Boolean);
    const rejected = claimed && !hazard
      ? Object.entries(checks).filter(([, ok]) => !ok).map(([k]) => k)
      : [];

    if (rejected.length) {
      console.log(JSON.stringify({
        message: "vision: positive rejected by confirmation checks",
        thing, model, kind, confidence, evidence_len: evidence.length, rejected,
      }));
    }

    const verdict = {
      ...base,
      hazard,
      kind: hazard ? kind : (kind || "none"),
      confidence: Number.isFinite(confidence) ? confidence : null,
      evidence: hazard ? evidence.slice(0, 300) : null,
      claimed_hazard: claimed,
      rejected_because: rejected,
      description: typeof obj?.description === "string" && obj.description
        ? obj.description.slice(0, 600)
        : raw.slice(0, 400),
      model,
      stage: "confirm",
      confirmed: hazard,
      latency_ms: latencyMs,
    };

    // Awaited, not deferred: the streak this returns decides whether the
    // report says "warning", so the answer is needed before returning.
    const rec = await recordVision(env, thing, fp, verdict, { ctx, needStreak: true });
    return { ...verdict, unconfirmed_streak: rec?.unconfirmed_streak ?? 0, detections };
  }

  // Every confirmation model failed. Say so. The one thing never returned here
  // is a verdict — a frame stage 1 could not clear must not be reported as
  // clear just because stage 2 was unreachable.
  console.error(JSON.stringify({ message: "vision confirmation failed", thing, error: lastErr }));
  return {
    thing,
    error: lastErr || "no vision model available",
    triage,
    frame_ts: frameTs,
    frame_age_s: frameAgeS,
    detections,
  };
}

/**
 * Persist the verdict and get back the unconfirmed-positive streak.
 *
 * When the caller does not need the streak (a frame triage cleared, which is
 * the common and latency-sensitive case) the write rides `ctx.waitUntil` so the
 * response is not held up by it — handed to the runtime rather than left
 * floating, and with its own catch so a failed write is logged, not lost.
 */
async function recordVision(env, thing, fp, verdict, { ctx = null, needStreak = false } = {}) {
  const pending = hubFetch(
    env,
    `/vision-record?thing=${encodeURIComponent(thing)}&fp=${fp}`,
    { method: "POST", body: JSON.stringify(verdict) },
  ).then((r) => r.json());

  const logFailure = (err) => console.error(JSON.stringify({
    message: "vision verdict not recorded", thing,
    error: err instanceof Error ? err.message : String(err),
  }));

  if (!needStreak && ctx) {
    ctx.waitUntil(pending.catch(logFailure));
    return null;
  }
  try {
    return await pending;
  } catch (err) {
    logFailure(err);
    return null;
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
    // The detail goes to the log, never to the caller — raw D1 text names
    // tables, columns and binding internals. Same rule the top-level handler
    // applies; this path used to contradict it.
    console.error(JSON.stringify({
      message: "reports query failed",
      error: err instanceof Error ? err.message : String(err),
    }));
    return json({ error: "query failed" }, 500);
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
    // SUBTYPE_RE happily accepts "pub", and publishItems() would then build the
    // channel "pub:<thing>" — byte-identical to the tag acceptPublisher() puts
    // on the rover's own publisher socket. getWebSockets("pub:rover2") would
    // return that socket and the rover would be sent its own frames back. This
    // is the enforcement point that makes acceptPublisher's "cannot collide"
    // claim actually true.
    if (RESERVED_SUBTYPES.has(subtype.toLowerCase())) {
      throw new Error(`subtype "${subtype}" is reserved`);
    }
    return {
      thing,
      subtype,
      kind: item?.kind === "events" ? "events" : "telemetry",
      data: item?.data ?? {},
    };
  });
}

/**
 * Operator session check. FAILS CLOSED, exactly like ingestAuthed().
 *
 * This used to `return true` when FPMS_PASSWORD was unset, on the reasoning that
 * an unconfigured app should be usable. That is the wrong default for a secret
 * that can go missing: a binding dropped during a redeploy, a secret deleted by
 * accident, or a `wrangler deploy` from a machine without it, and the ENTIRE
 * authenticated surface silently opens to the internet — raw telemetry and
 * camera frames, unmetered AI spend via /api/vision and /api/analyze, the
 * operator's alert address, and the personal names in info.json. Nothing about
 * that failure is visible from the outside; the app just keeps working.
 *
 * Closed is recoverable (set the secret) and loud. Anonymous visitors still have
 * the whitelisted public projection in public.js, which is what it is for.
 */
async function authed(request, env) {
  if (!env.FPMS_PASSWORD) {
    console.error(JSON.stringify({
      message: "FPMS_PASSWORD is not configured — denying every authenticated request",
      hint: "npx wrangler secret put FPMS_PASSWORD",
    }));
    return false;
  }
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
    // Always true, because authed() now fails closed: with no password
    // configured the API denies everything rather than opening up, so reporting
    // auth_required:false would tell the frontend to render a dashboard whose
    // every request 401s. A login screen is the honest thing to show.
    auth_required: true,
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
async function analyzeAndReport(env, { force = false, ctx = null } = {}) {
  if (!env.DB) return { error: "archive not configured" };

  // A cold or unreachable hub must not take the analysis down with it — the
  // agents can still read D1 without knowing the roster.
  const snap = await hubFetch(env, "/snapshot")
    .then((r) => r.json())
    .catch((err) => {
      console.error(JSON.stringify({
        message: "analysis: hub snapshot failed — continuing without the roster",
        error: err instanceof Error ? err.message : String(err),
      }));
      return {};
    });

  let report;
  try {
    report = await runAgents(env.DB, snap.things_seen || []);
  } catch (err) {
    // DEGRADED PATH — the archive read failed.
    //
    // This call had no try/catch, so a D1 error propagated straight out: no
    // report row, no email, one log line nobody is watching. The watchdog went
    // silent at precisely the moment it lost its data, and silence from a fire
    // watch is indistinguishable from "all clear". Worse, D1's read quota can be
    // spent by anonymous traffic on the public page, so an outside visitor could
    // switch fire alerting off without touching anything authenticated.
    //
    // So: synthesise a critical report saying the archive is unreachable. Being
    // blind IS the emergency to report. Everything downstream — vision, the edge
    // LM, the email gate — still runs on it, which is the whole point: the
    // operator hears something rather than nothing.
    console.error(JSON.stringify({
      message: "agent run failed — emitting degraded report",
      error: err instanceof Error ? err.message : String(err),
    }));
    report = {
      ts: Date.now() / 1000,
      severity: "critical",
      summary: "Telemetry archive unreachable — the fire watch is BLIND, no telemetry could be analysed.",
      findings: [{
        agent: "archive",
        severity: "critical",
        detail:
          "Could not read the telemetry archive (D1). No rover data was analysed " +
          "on this run, so nothing below should be read as an all-clear. Check the " +
          "D1 binding and the daily read quota. Error: " +
          (err instanceof Error ? err.message : String(err)).slice(0, 200),
      }],
      sample_size: 0,
      degraded: true,
    };
  }

  // Ask the VLM what the newest frame actually shows. Rule-based agents know
  // the numbers; this is the only part that can say "smoke behind the ridge".
  // Best-effort: a slow or unavailable model must not delay an alert.
  //
  // Run in parallel and settle rather than sequentially: one rover whose model
  // call is crawling used to push the second rover's analysis behind it, so a
  // fire on rover2 waited on a slow frame from rover1. Every call already
  // carries its own deadline, so the fan-out is bounded by the slowest timeout,
  // not by their sum.
  const visionThings = (snap.things_seen || []).slice(0, 2);
  const settled = await Promise.allSettled(
    visionThings.map((thing) => visionForThing(env, thing, { ctx })),
  );

  const visionStats = { looked: 0, cached: 0, confirmed: 0, unconfirmed: 0, unavailable: 0 };
  for (let i = 0; i < settled.length; i++) {
    const thing = visionThings[i];
    const s = settled[i];
    if (s.status !== "fulfilled") {
      // visionForThing does not throw, so this is a bug, not a model failure.
      visionStats.unavailable++;
      console.error(JSON.stringify({
        message: "vision threw — treated as no analysis", thing,
        error: s.reason instanceof Error ? s.reason.message : String(s.reason),
      }));
      continue;
    }
    const v = s.value;

    // A refusal (stale frame, budget, no frame, model down) comes back with
    // `error` and no `description`, so it adds no finding and asserts nothing.
    // Silence here means "no analysis available", never "no hazard".
    if (!v || v.error || !v.description) {
      visionStats.unavailable++;
      continue;
    }

    visionStats.looked++;
    if (v.cached) visionStats.cached++;

    // SEVERITY, AND WHY AN UNCONFIRMED POSITIVE IS NOT ONE.
    //
    // `hazard` is already the AND of two models with opposite biases plus four
    // structural checks — so critical here means confirmed, not suspected. A
    // frame that triage flagged and confirmation rejected stays "ok": one
    // disagreement is a sunset, and promoting it would page the operator on
    // every dusk, which is exactly how an alert channel gets ignored.
    //
    // But the same disagreement several cycles running is not noise, so the
    // streak escalates to "warning" — asymmetric, like the rover-side fire
    // detection, and it clears the moment triage comes back clean.
    const streak = Number(v.unconfirmed_streak) || 0;
    const persistent = !v.hazard && v.triage?.flagged === true
      && streak >= VISION_UNCONFIRMED_STREAK_WARN;

    if (v.hazard) visionStats.confirmed++;
    else if (v.triage?.flagged) visionStats.unconfirmed++;

    let detail = v.description;
    if (v.hazard && v.evidence) {
      detail = `${v.description} Evidence: ${v.evidence}`;
    } else if (persistent) {
      detail =
        `Screening has flagged possible smoke or fire on ${streak} consecutive runs and the ` +
        `confirmation pass rejected it each time — NOT a confirmed fire, but worth human eyes. ` +
        `Scene: ${v.description}`;
    } else if (v.triage?.flagged) {
      detail =
        `Screening flagged this frame; the confirmation pass did not agree, so it is treated ` +
        `as no hazard. Scene: ${v.description}`;
    }

    report.findings.push({
      agent: `vision:${thing}`,
      // Trust the structured verdict, never keywords in the prose.
      severity: v.hazard ? "critical" : persistent ? "warning" : "ok",
      detail,
      hazard: v.hazard,
      kind: v.kind ?? null,
      confidence: v.confidence ?? null,
      evidence: v.evidence ?? null,
      // Kept so a rejected positive is auditable from the stored report rather
      // than only from the logs.
      claimed_hazard: v.claimed_hazard ?? false,
      rejected_because: v.rejected_because ?? [],
      unconfirmed_streak: streak,
      verdict_parsed: v.verdict_parsed,
      stage: v.stage ?? null,
      cached: Boolean(v.cached),
      model: v.model,
      triage_model: v.triage?.model ?? null,
      latency_ms: v.latency_ms ?? null,
    });
  }
  // Recompute after adding vision findings so a VLM fire sighting can raise it.
  report.severity = worstSeverity(report.findings);

  // Edge LM pass — LAST, and deliberately after severity is final.
  //
  // It explains what the findings mean together; it does not get a vote on how
  // bad they are. Attaching it here rather than pushing a finding is what keeps
  // it out of worstSeverity() and out of the email trigger below. Returning
  // null is normal (no AI binding, over quota, model down) and the report is
  // complete without it.
  //
  // Deadlined like every other model call. reasonOverFindings already swallows
  // its own errors, but nothing inside it bounds a model that simply never
  // answers — and this runs on the cron, where a hang costs the report, the
  // email and the prune pass together.
  report.reasoning = await withTimeout(
    reasonOverFindings(env, report, { model: TEXT_MODEL }),
    AI_REASON_TIMEOUT_MS,
    "edge LM reasoning",
  ).catch((err) => {
    console.error(JSON.stringify({
      message: "edge LM reasoning skipped",
      error: err instanceof Error ? err.message : String(err),
    }));
    return null;
  });

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
    const gateRes = await hubFetch(
      env,
      `/report-gate?severity=${encodeURIComponent(report.severity)}` +
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

  // Store on state change, not on every run.
  //
  // The cron fires every 15 minutes and this used to insert unconditionally: 96
  // rows/day of near-identical "All clear — N readings analysed", each one also
  // writing the index rows declared in schema.sql, against a 100k billed
  // rows/day free tier. The signature already tells us whether anything
  // changed, so reuse it and keep a periodic heartbeat row so a reader can
  // still tell "nothing changed" from "the cron stopped running".
  //
  // Always stored regardless: a forced run (the operator asked for it and
  // expects to see it) and any run that sent an email (the row is the record
  // the email refers to).
  let stored = false;
  let storeGate = { store: true, reason: "forced" };
  if (!force && !emailed) {
    storeGate = await hubFetch(
      env,
      `/report-store-gate?severity=${encodeURIComponent(report.severity)}` +
      `&sig=${encodeURIComponent(signature)}`,
    )
      .then((r) => r.json())
      .catch((err) => {
        // Fail towards storing: a missing history row is unrecoverable, a
        // duplicate one is merely a row.
        console.error(JSON.stringify({
          message: "report store gate unavailable — storing anyway",
          error: err instanceof Error ? err.message : String(err),
        }));
        return { store: true, reason: "gate unavailable" };
      });
  }

  if (storeGate.store) {
    try {
      await env.DB.prepare(
        "INSERT INTO reports (ts, severity, summary, findings, emailed) VALUES (?, ?, ?, ?, ?)",
      ).bind(report.ts, report.severity, report.summary, JSON.stringify(report.findings), emailed ? 1 : 0).run();
      stored = true;
    } catch (err) {
      // Never throw from here. On the degraded path above D1 is already down,
      // and letting the insert fail the whole function would undo the email that
      // has just gone out — the caller would see an error for a run that did
      // in fact alert.
      console.error(JSON.stringify({
        message: "report insert failed",
        error: err instanceof Error ? err.message : String(err),
      }));
    }
  }

  return {
    ...report, emailed, email, gate, stored, store_reason: storeGate.reason,
    // How much AI was actually spent this run, so cost is observable from the
    // response and from the cron log rather than only from the billing page.
    vision_stats: visionStats,
  };
}

/**
 * Delete aged-out rows so that month six looks like week one.
 *
 * Nothing else in this codebase deletes anything. Left alone, `readings` grows
 * until it hits D1's 5 GB database limit, at which point every INSERT starts
 * failing and ingest, the event trail and alerting stop together — a storage
 * problem that presents as a fire-detection outage.
 *
 * NOTE: SQLite (and therefore D1) does NOT return freed pages to the file on
 * DELETE — the space is only marked reusable inside the database. Row count and
 * query cost come down immediately, but the reported database SIZE will not
 * shrink without a VACUUM, which D1 does not expose as a statement you can run
 * from a Worker. The point of this function is to bound growth, not to reclaim
 * what has already been written.
 */
async function pruneArchive(env) {
  if (!env.DB) return { skipped: "archive not configured" };

  // Throttled in the Durable Object because a Worker has nowhere durable to
  // keep "when did I last prune" — module state does not survive, and isolates
  // are shared between unrelated requests.
  let gate;
  try {
    gate = await hubStub(env)
      .fetch("https://hub/prune-gate", { method: "POST" })
      .then((r) => r.json());
  } catch (err) {
    console.error(JSON.stringify({
      message: "prune gate unavailable — skipping this run",
      error: err instanceof Error ? err.message : String(err),
    }));
    return { skipped: "gate unavailable" };
  }
  if (!gate.allowed) return { skipped: "not due", next_in_s: gate.next_in_s };

  const now = Date.now() / 1000;
  // Table names are literals from this list, never caller input — the cutoff and
  // the batch size are bound parameters, as everywhere else in this file.
  const plan = [
    ["readings", now - READINGS_RETENTION_DAYS * 86400],
    ["reports", now - REPORTS_RETENTION_DAYS * 86400],
  ];

  const deleted = {};
  for (const [table, cutoff] of plan) {
    try {
      // `DELETE ... LIMIT` needs SQLITE_ENABLE_UPDATE_DELETE_LIMIT, which D1's
      // build does not have, so bound it through the rowid instead. Bounding
      // matters: the first prune after this ships may face months of backlog,
      // and one unbounded DELETE would exceed the statement time limit, roll
      // back, and delete nothing on every run forever. A batch per pass drains
      // it gradually, which is the difference between slow and never.
      const res = await env.DB
        .prepare(`DELETE FROM ${table} WHERE rowid IN (SELECT rowid FROM ${table} WHERE ts < ? LIMIT ?)`)
        .bind(cutoff, PRUNE_BATCH)
        .run();
      deleted[table] = res?.meta?.changes ?? 0;
    } catch (err) {
      deleted[table] = null;
      console.error(JSON.stringify({
        message: "prune failed", table,
        error: err instanceof Error ? err.message : String(err),
      }));
    }
  }

  console.log(JSON.stringify({
    message: "archive pruned",
    readings_deleted: deleted.readings,
    reports_deleted: deleted.reports,
    readings_retention_days: READINGS_RETENTION_DAYS,
    reports_retention_days: REPORTS_RETENTION_DAYS,
    batch: PRUNE_BATCH,
    // True when a table filled its batch: there is more to delete and the next
    // scheduled pass will take another bite.
    more_pending: deleted.readings === PRUNE_BATCH || deleted.reports === PRUNE_BATCH,
  }));

  return deleted;
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
    // Logged, not returned — see reports(). Raw D1 error text discloses schema.
    console.error(JSON.stringify({
      message: "history query failed",
      error: err instanceof Error ? err.message : String(err),
    }));
    return json({ error: "query failed" }, 500);
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
  memSubtypes = new Set();         // distinct subtypes tracked, for the cap
  memLastAt = null;
  // thing -> unix SECONDS of its most recent publish. Published by snapshot()
  // so the public summary can answer "when was this rover last heard from?"
  // without the full-table scan it used to run per page poll.
  memLastSeen = new Map();
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
    if (url.pathname === "/vision-gate") return this.visionGate(url);
    if (url.pathname === "/vision-record") return this.visionRecord(request, url);
    if (url.pathname === "/report-gate") return this.reportGate(url);
    if (url.pathname === "/report-store-gate") return this.reportStoreGate(url);
    if (url.pathname === "/prune-gate") return this.pruneGate();
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
      this.memSubtypes = new Set((await this.ctx.storage.get("subtypes_seen")) || []);
      // Rehydrated like memSeen: without this a cold start would report every
      // rover as never-seen until its next publish, and the public page would
      // show a healthy fleet as offline.
      this.memLastSeen = new Map(
        Object.entries((await this.ctx.storage.get("last_seen")) || {}),
      );
    }
    let seen = this.memSeen;
    const things = this.memThings;
    const subtypes = this.memSubtypes;
    const toArchive = [];
    let notified = 0;
    let allSockets = 0;
    let dropped = 0;

    for (const item of items) {
      // ROSTER CAP. Every new thing or subtype mints a permanent
      // `latest:<channel>` key that nothing ever deletes and that snapshot()
      // lists on every public poll, so a single token holder could grow this
      // object without limit. Names already known always pass — the cap only
      // ever refuses a NEW one, so a real fleet never notices it. Dropped
      // loudly rather than silently: a legitimate rover that cannot get in
      // must be diagnosable from the logs.
      if (!things.has(item.thing) && things.size >= MAX_THINGS) {
        dropped++;
        console.error(JSON.stringify({
          message: "thing cap reached — item dropped",
          thing: item.thing, subtype: item.subtype, cap: MAX_THINGS,
        }));
        continue;
      }
      if (!subtypes.has(item.subtype) && subtypes.size >= MAX_SUBTYPES) {
        dropped++;
        console.error(JSON.stringify({
          message: "subtype cap reached — item dropped",
          thing: item.thing, subtype: item.subtype, cap: MAX_SUBTYPES,
        }));
        continue;
      }

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
      subtypes.add(item.subtype);
      // Unix SECONDS, matching `ts` on the envelope and on every D1 row.
      this.memLastSeen.set(item.thing, now);
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
        subtypes_seen: [...subtypes],
        last_message_at: now,
        // Rides the SAME throttled multi-key write as the counters — it is one
        // more value in an existing put(), not an extra billed write. A plain
        // object because storage cannot serialise a Map.
        last_seen: Object.fromEntries(this.memLastSeen),
      });
    }

    return {
      ok: true, accepted: items.length - dropped, dropped, messages_seen: seen,
      // Diagnostics: distinguishes "nobody listening" from "tag lookup broken".
      sockets_notified: notified, sockets_open: allSockets,
    };
  }

  /**
   * Accept a rover as a publisher.
   *
   * The tag is "pub" (plus "pub:<thing>"). That second form DOES have the same
   * "<x>:<thing>" shape as a channel tag such as "camera:rover2", so the safety
   * property is not structural — it is enforced. An item with subtype "pub"
   * would build the channel "pub:rover2", getWebSockets() would hand back this
   * very socket, and publishItems() would echo every frame straight back to the
   * rover it came from. normalizeItems() rejects the reserved subtype (see
   * RESERVED_SUBTYPES), which is what makes the collision impossible; an earlier
   * version of this comment claimed it could not happen, and it could.
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
   * Sampling gate for billed vision inference — the cost control.
   *
   * WHY IT LIVES HERE
   * -----------------
   * The decision needs memory ("when did I last look at this rover, and at
   * which frame?") and a Worker has none that survives: isolates are reused
   * across unrelated requests, so module state is both lost and leaky. This
   * object is already on the vision path — it is where the frame comes from —
   * so the state costs no new class of round trip.
   *
   * Three refusals, in order of how much they save:
   *  1. The frame is byte-identical to the one already analysed. A stalled
   *     camera or a parked rover produces this constantly, and re-buying an
   *     identical verdict is the purest waste there is. The previous verdict
   *     comes back instead, labelled so nobody mistakes it for fresh.
   *  2. Less than VISION_MIN_INTERVAL_S since the last billed look at this
   *     rover. Bounds an authenticated dashboard polling /api/vision.
   *  3. The daily stage-1 ceiling is spent.
   *
   * The ceiling deliberately covers TRIAGE only. Confirmation is never gated:
   * once a frame has been flagged, budget must not be the reason it goes
   * unexamined. Capping the cheap always-on stage bounds routine spend without
   * ever standing between a possible fire and a verdict.
   */
  async visionGate(url) {
    const thing = url.searchParams.get("thing") || "";
    const fp = url.searchParams.get("fp") || "";
    const key = `vision:${thing}`;
    const now = Date.now();

    const st = (await this.ctx.storage.get(key)) || {};
    // UTC day, matching how Cloudflare resets the Workers AI allowance. Derived
    // from the clock rather than a timer so there is nothing to schedule and
    // nothing to miss while the object is hibernating.
    const day = new Date(now).toISOString().slice(0, 10);
    const stored = (await this.ctx.storage.get("vision_budget")) || {};
    const used = stored.day === day ? Number(stored.used) || 0 : 0;

    if (fp && st.fp === fp && st.verdict) {
      return Response.json({
        allowed: false,
        reason: "frame unchanged since last analysis",
        cached: st.verdict,
        unconfirmed_streak: Number(st.unconfirmed_streak) || 0,
        analysed_ago_s: st.at ? Math.round((now - st.at) / 1000) : null,
      });
    }

    const sinceS = st.at ? Math.round((now - st.at) / 1000) : null;
    if (sinceS !== null && sinceS < VISION_MIN_INTERVAL_S) {
      return Response.json({
        allowed: false,
        reason: "sampled recently",
        next_in_s: VISION_MIN_INTERVAL_S - sinceS,
        cached: st.verdict || null,
        unconfirmed_streak: Number(st.unconfirmed_streak) || 0,
      });
    }

    if (used >= VISION_DAILY_TRIAGE_BUDGET) {
      // No cached verdict is offered here on purpose. The cache answers "this
      // exact frame" and "a moment ago"; a budget refusal can be many hours
      // stale, and a stale verdict presented as current is the failure this
      // whole file is written to avoid. "No analysis available" is the truth.
      console.error(JSON.stringify({
        message: "vision daily triage budget exhausted",
        thing, used, budget: VISION_DAILY_TRIAGE_BUDGET, day,
      }));
      return Response.json({
        allowed: false, reason: "daily vision budget exhausted",
        used, budget: VISION_DAILY_TRIAGE_BUDGET,
      });
    }

    // Both claimed BEFORE the inference runs, exactly as pruneGate does. If the
    // model call then fails or the request is cut short, the next attempt waits
    // one interval — whereas committing afterwards would let a model that fails
    // instantly be retried on every single poll, for real money.
    await this.ctx.storage.put({
      [key]: { ...st, at: now },
      vision_budget: { day, used: used + 1 },
    });
    return Response.json({
      allowed: true, reason: "due",
      used: used + 1, budget: VISION_DAILY_TRIAGE_BUDGET,
    });
  }

  /**
   * Store a vision verdict and return the unconfirmed-positive streak.
   *
   * The streak is the asymmetric-hysteresis counter: it increments only when
   * triage flagged a frame and confirmation rejected it, and resets to zero on
   * anything else — a clear frame, a confirmed hazard, or a run that produced
   * no verdict. Slow to raise concern, instant to drop it.
   */
  async visionRecord(request, url) {
    const thing = url.searchParams.get("thing") || "";
    const fp = url.searchParams.get("fp") || "";
    const key = `vision:${thing}`;

    let verdict = null;
    try { verdict = await request.json(); } catch { verdict = null; }

    const st = (await this.ctx.storage.get(key)) || {};
    const previous = Number(st.unconfirmed_streak) || 0;
    const flagged = verdict?.triage?.flagged === true;
    const confirmed = verdict?.hazard === true;
    const streak = flagged && !confirmed ? previous + 1 : 0;

    await this.ctx.storage.put(key, {
      at: Date.now(),
      fp,
      // Trimmed, not stored whole. A verdict carries the rover's detection list
      // and the model's prose; a Durable Object storage value is capped and
      // every byte is a billed write, and none of the dropped fields are ever
      // read back from the cache.
      verdict: verdict ? {
        hazard: verdict.hazard === true,
        kind: typeof verdict.kind === "string" ? verdict.kind : null,
        confidence: Number.isFinite(verdict.confidence) ? verdict.confidence : null,
        evidence: typeof verdict.evidence === "string" ? verdict.evidence.slice(0, 300) : null,
        claimed_hazard: verdict.claimed_hazard === true,
        rejected_because: Array.isArray(verdict.rejected_because) ? verdict.rejected_because : [],
        description: typeof verdict.description === "string" ? verdict.description.slice(0, 600) : "",
        model: typeof verdict.model === "string" ? verdict.model : null,
        stage: typeof verdict.stage === "string" ? verdict.stage : null,
        structured: true,
        verdict_parsed: verdict.verdict_parsed === true,
        confirmed: verdict.confirmed === true,
        frame_ts: Number(verdict.frame_ts) || null,
        frame_age_s: Number.isFinite(verdict.frame_age_s) ? verdict.frame_age_s : null,
        triage: verdict.triage ? {
          model: verdict.triage.model ?? null,
          flagged: verdict.triage.flagged === true,
          reason: typeof verdict.triage.reason === "string" ? verdict.triage.reason : null,
        } : null,
        latency_ms: Number.isFinite(verdict.latency_ms) ? verdict.latency_ms : null,
      } : null,
      unconfirmed_streak: streak,
    });

    return Response.json({ unconfirmed_streak: streak, previous });
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

  /**
   * Should this analysis report be written to D1?
   *
   * The decision needs memory of the previous run, and a Worker has none that
   * survives — so it lives here beside the email gate, which asks a similar
   * question for a different action. Deliberately SEPARATE from reportGate:
   * that one decides whether to wake a human and has its own re-notify window;
   * this one only decides whether to spend a row. Merging them would couple the
   * history table's completeness to an email cooldown.
   */
  async reportStoreGate(url) {
    const key =
      `${url.searchParams.get("severity") || "ok"}|${url.searchParams.get("sig") || ""}`;

    const lastKey = await this.ctx.storage.get("last_stored_report_key");
    const runs = ((await this.ctx.storage.get("runs_since_stored_report")) || 0) + 1;

    // undefined (never stored) counts as changed, so the very first run after
    // deploy always lands a row.
    const changed = lastKey === undefined || key !== lastKey;
    const heartbeat = runs >= REPORT_HEARTBEAT_RUNS;

    if (changed || heartbeat) {
      await this.ctx.storage.put({
        last_stored_report_key: key,
        runs_since_stored_report: 0,
      });
      return Response.json({
        store: true,
        reason: changed ? "state changed" : "heartbeat",
        skipped: runs - 1,
      });
    }

    await this.ctx.storage.put("runs_since_stored_report", runs);
    return Response.json({
      store: false,
      reason: "unchanged since last stored report",
      skipped: runs,
      // How many more unchanged runs before the heartbeat row.
      heartbeat_in: REPORT_HEARTBEAT_RUNS - runs,
    });
  }

  /**
   * Throttle for archive pruning. State has to be durable — the cron fires 96
   * times a day and a Worker cannot remember across invocations — and this
   * object is the only durable thing the cron path already talks to.
   */
  async pruneGate() {
    const now = Date.now();
    const last = (await this.ctx.storage.get("last_prune_at")) || 0;
    if (now - last < PRUNE_MIN_INTERVAL_MS) {
      return Response.json({
        allowed: false,
        next_in_s: Math.ceil((PRUNE_MIN_INTERVAL_MS - (now - last)) / 1000),
      });
    }
    // Claimed BEFORE the delete runs, not after. If the prune then fails or the
    // request is cut short, the next pass simply happens one interval later —
    // whereas committing afterwards would let a repeatedly failing prune retry
    // on every single cron run.
    await this.ctx.storage.put("last_prune_at", now);
    return Response.json({ allowed: true, last_prune_at: last || null });
  }

  /** Drop rovers from the roster — used to clear test fixtures. */
  async forget(url) {
    const things = (url.searchParams.get("things") || "").split(",").filter(Boolean);
    const known = new Set((await this.ctx.storage.get("things_seen")) || []);
    for (const t of things) known.delete(t);
    await this.ctx.storage.put("things_seen", [...known]);

    // Forget the last-seen stamps too, in both storage and memory. Left behind,
    // a forgotten rover would keep reappearing in snapshot()'s last_seen map —
    // and therefore on the public page — despite being off the roster. Also
    // frees the roster slot the cap counts.
    const lastSeen = (await this.ctx.storage.get("last_seen")) || {};
    for (const t of things) {
      delete lastSeen[t];
      this.memLastSeen.delete(t);
      this.memThings.delete(t);
    }
    await this.ctx.storage.put("last_seen", lastSeen);

    for (const t of things) {
      const list = await this.ctx.storage.list({ prefix: "latest:" });
      for (const key of list.keys()) {
        if (key.endsWith(`:${t}`)) await this.ctx.storage.delete(key);
      }
      // And its vision state. Left behind, a rover cleared as a test fixture
      // and later re-added would inherit a stale cached verdict and a stale
      // unconfirmed streak — the second of which could produce a "warning"
      // about a scene that no longer exists.
      await this.ctx.storage.delete(`vision:${t}`);
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
      // Most recent publish per thing, unix SECONDS — the same unit as every
      // `ts` in this system.
      //
      // This exists so the public summary can answer "when was this rover last
      // heard from?" without `SELECT thing, MAX(ts) FROM readings GROUP BY
      // thing`, a full-table scan that ran on EVERY anonymous page poll and is
      // the most expensive query in the app. The hub already knows the answer
      // — it is what wrote those rows.
      last_seen: await this.lastSeenMap(),
      // Lets the dashboard distinguish "no data" from "nobody publishing".
      publishers: this.ctx.getWebSockets("pub").length,
      channels: [...channels].sort(),
    });
  }

  /**
   * { thing: unix_seconds } merged from storage and memory.
   *
   * Memory wins on a tie because it can be up to PERSIST_MIN_MS ahead of the
   * last durable write; reporting the older value would show a rover that
   * published two seconds ago as stale. Storage is what survives hibernation,
   * so neither source alone is sufficient.
   */
  async lastSeenMap() {
    const out = { ...((await this.ctx.storage.get("last_seen")) || {}) };
    for (const [thing, ts] of this.memLastSeen) {
      const n = Number(ts);
      if (Number.isFinite(n) && !(Number(out[thing]) > n)) out[thing] = n;
    }
    return out;
  }
}

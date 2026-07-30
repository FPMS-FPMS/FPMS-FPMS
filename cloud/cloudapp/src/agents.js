/**
 * FPMS analysis agents.
 *
 * Each agent reads recent telemetry from D1 and returns findings. They are
 * deliberately deterministic rather than model-driven: a fire-detection system
 * has to give the same verdict for the same numbers every time, and has to
 * work when an inference endpoint is down. An LLM summarisation pass can sit on
 * top of these findings later — it should not sit underneath the decision.
 *
 * Severity ladder: ok < warning < critical. The highest finding wins the report.
 */

const SEVERITY_ORDER = { ok: 0, warning: 1, critical: 2 };

// Thresholds. Deliberately explicit — an operator has to be able to read these
// numbers off the page and argue with them.
const THERMAL_CRITICAL_C = 400;   // sustained hot spot: probable fire
const THERMAL_WARNING_C = 200;    // elevated: worth a look
const ROVER_SILENT_WARNING_S = 15 * 60;
const ROVER_SILENT_CRITICAL_S = 60 * 60;

export function worstSeverity(findings) {
  return findings.reduce(
    (worst, f) => (SEVERITY_ORDER[f.severity] > SEVERITY_ORDER[worst] ? f.severity : worst),
    "ok",
  );
}

/** Pull the recent window once; every agent reads from the same snapshot. */
export async function loadWindow(db, seconds = 3600, limit = 2000) {
  const since = Date.now() / 1000 - seconds;
  const { results } = await db
    .prepare(
      "SELECT ts, thing, subtype, kind, data FROM readings WHERE ts >= ? ORDER BY ts DESC LIMIT ?",
    )
    .bind(since, limit)
    .all();
  return results.map((r) => ({ ...r, data: safeParse(r.data) }));
}

function safeParse(s) {
  try {
    return JSON.parse(s);
  } catch {
    return {};
  }
}

/** Highest temperature seen, and whether it crosses a threshold. */
export function thermalAgent(rows) {
  const thermal = rows.filter((r) => r.subtype === "thermal");
  if (!thermal.length) {
    return { agent: "thermal", severity: "ok", detail: "No thermal readings in window." };
  }

  let peak = null;
  for (const r of thermal) {
    // Accept a few field spellings — rover firmware naming has drifted before.
    const c = r.data.max_c ?? r.data.maxC ?? r.data.max_temp_c ?? r.data.temperature_c;
    if (typeof c === "number" && (!peak || c > peak.c)) peak = { c, thing: r.thing, ts: r.ts };
  }
  if (!peak) {
    return { agent: "thermal", severity: "ok", detail: `${thermal.length} thermal readings, no temperature field recognised.` };
  }

  let severity = "ok";
  if (peak.c >= THERMAL_CRITICAL_C) severity = "critical";
  else if (peak.c >= THERMAL_WARNING_C) severity = "warning";

  return {
    agent: "thermal",
    severity,
    detail: `Peak ${peak.c.toFixed(1)}°C on ${peak.thing}` +
      (severity === "critical" ? ` — at or above the ${THERMAL_CRITICAL_C}°C fire threshold.`
        : severity === "warning" ? ` — above the ${THERMAL_WARNING_C}°C watch threshold.`
        : " — within normal range."),
    peak_c: peak.c,
    thing: peak.thing,
  };
}

/** Rovers that have stopped reporting. Silence is itself a finding. */
export function livenessAgent(rows, knownThings) {
  const now = Date.now() / 1000;
  const lastSeen = new Map();
  for (const r of rows) {
    if (!lastSeen.has(r.thing) || r.ts > lastSeen.get(r.thing)) lastSeen.set(r.thing, r.ts);
  }
  for (const t of knownThings) if (!lastSeen.has(t)) lastSeen.set(t, 0);

  if (!lastSeen.size) {
    return { agent: "liveness", severity: "ok", detail: "No rovers registered yet." };
  }

  const silent = [];
  for (const [thing, ts] of lastSeen) {
    const age = ts ? now - ts : Infinity;
    if (age >= ROVER_SILENT_WARNING_S) {
      silent.push({ thing, age_seconds: Number.isFinite(age) ? Math.round(age) : null });
    }
  }

  if (!silent.length) {
    return { agent: "liveness", severity: "ok", detail: `All ${lastSeen.size} rover(s) reporting.` };
  }
  const worst = Math.max(...silent.map((s) => s.age_seconds ?? Infinity));
  return {
    agent: "liveness",
    severity: worst >= ROVER_SILENT_CRITICAL_S ? "critical" : "warning",
    detail: silent
      .map((s) => `${s.thing} silent for ${s.age_seconds === null ? "ever" : Math.round(s.age_seconds / 60) + " min"}`)
      .join("; "),
    silent,
  };
}

/** Any event the rover itself flagged. The rover is closest to the fire. */
export function eventAgent(rows) {
  const alerts = rows.filter((r) => r.kind === "events" && r.data && r.data.alert);
  if (!alerts.length) {
    return { agent: "events", severity: "ok", detail: "No rover-raised alerts in window." };
  }
  return {
    agent: "events",
    severity: "critical",
    detail: `${alerts.length} rover-raised alert(s): ` +
      alerts.slice(0, 5).map((a) => `${a.thing}/${a.subtype}`).join(", "),
    alerts: alerts.slice(0, 20).map((a) => ({ thing: a.thing, subtype: a.subtype, ts: a.ts, data: a.data })),
  };
}

/**
 * Onboard fire screen results. The rover's HSV detector is intentionally
 * sensitive — it is a screen, not a verdict — so this agent reports it as a
 * warning on its own and only escalates when the rover raised an actual event.
 */
export function fireAgent(rows) {
  const cams = rows.filter((r) => r.subtype === "camera");
  const hits = cams.filter((r) => r.data && r.data.fire_like);
  const fireEvents = rows.filter((r) => r.kind === "events" && r.subtype === "fire");
  if (!hits.length && !fireEvents.length) {
    return { agent: "fire", severity: "ok",
             detail: `No flame colours in ${cams.length} analysed frames.` };
  }
  const peak = Math.max(0, ...cams.map((r) => r.data?.fire_ratio || 0));
  const byThing = [...new Set(hits.map((h) => h.thing))].join(", ");
  return {
    agent: "fire",
    severity: fireEvents.length ? "critical" : "warning",
    detail: `Flame-coloured regions on ${hits.length}/${cams.length} frames` +
            (byThing ? ` (${byThing})` : "") +
            `, peak ${(peak * 100).toFixed(2)}% of view, ${fireEvents.length} rover alert(s). ` +
            `Onboard HSV screen — corroborate with thermal and the vision model.`,
    frames_flagged: hits.length,
    peak_ratio: peak,
  };
}

/** Wildlife seen by YOLO. Animals leaving an area can precede a fire. */
export function wildlifeAgent(rows) {
  const seen = new Map();
  for (const r of rows.filter((x) => x.subtype === "camera")) {
    for (const w of r.data?.wildlife || []) seen.set(w, (seen.get(w) || 0) + 1);
    for (const d of r.data?.detections || []) {
      if (d.kind === "wildlife") seen.set(d.label, (seen.get(d.label) || 0) + 1);
    }
  }
  if (!seen.size) {
    return { agent: "wildlife", severity: "ok", detail: "No wildlife detected in window." };
  }
  const list = [...seen.entries()].sort((a, b) => b[1] - a[1])
    .map(([k, v]) => `${k} (${v} frames)`);
  // A bear near the rover is an operational hazard, not just an observation.
  const hazardous = [...seen.keys()].some((k) => ["bear", "elephant", "horse", "cow"].includes(k));
  return {
    agent: "wildlife",
    severity: hazardous ? "warning" : "ok",
    detail: `Wildlife present: ${list.join(", ")}.` +
            (hazardous ? " Large animal near the rover — approach with care." : ""),
    species: [...seen.keys()].sort(),
  };
}

/** Volume per stream — catches a camera or LiDAR that has quietly stopped. */
export function coverageAgent(rows) {
  const counts = {};
  for (const r of rows) counts[r.subtype] = (counts[r.subtype] || 0) + 1;
  const streams = ["lidar", "camera", "thermal"];
  const missing = streams.filter((s) => !counts[s]);
  const summary = Object.entries(counts).map(([k, v]) => `${k}=${v}`).join(" ") || "nothing";

  if (missing.length === streams.length) {
    return { agent: "coverage", severity: "warning", detail: "No lidar, camera or thermal data in window.", counts };
  }
  if (missing.length) {
    return { agent: "coverage", severity: "warning", detail: `No data from: ${missing.join(", ")} (have ${summary}).`, counts };
  }
  return { agent: "coverage", severity: "ok", detail: `All streams reporting (${summary}).`, counts };
}

/**
 * Per-rover breakdown. A fleet-wide summary hides the case that matters most:
 * one rover healthy and one in trouble averages out to "mostly fine".
 */
export function perRoverAgent(rows, knownThings) {
  const now = Date.now() / 1000;
  const things = [...new Set([...rows.map((r) => r.thing), ...knownThings])].sort();
  if (!things.length) {
    return { agent: "per-rover", severity: "ok", detail: "No rovers reporting.", rovers: [] };
  }

  const rovers = things.map((thing) => {
    const mine = rows.filter((r) => r.thing === thing);
    const streams = {};
    for (const s of ["camera", "lidar", "thermal", "pose"]) {
      const newest = mine.filter((r) => r.subtype === s).sort((a, b) => b.ts - a.ts)[0];
      streams[s] = newest
        ? { count: mine.filter((r) => r.subtype === s).length, age_s: Math.round(now - newest.ts) }
        : { count: 0, age_s: null };
    }

    // What the cameras actually saw, aggregated.
    const species = new Set();
    const objects = new Map();
    let fireFrames = 0, peakFireRatio = 0, frames = 0;
    for (const r of mine.filter((x) => x.subtype === "camera")) {
      frames++;
      if (r.data.fire_like) fireFrames++;
      if (typeof r.data.fire_ratio === "number") {
        peakFireRatio = Math.max(peakFireRatio, r.data.fire_ratio);
      }
      for (const w of r.data.wildlife || []) species.add(w);
      for (const d of r.data.detections || []) {
        if (d.kind === "wildlife") species.add(d.label);
        else if (d.label) objects.set(d.label, (objects.get(d.label) || 0) + 1);
      }
    }

    const nearest = mine
      .filter((r) => r.subtype === "lidar" && typeof r.data.min_mm === "number")
      .reduce((min, r) => (min === null || r.data.min_mm < min ? r.data.min_mm : min), null);

    const events = mine.filter((r) => r.kind === "events");
    const fireEvents = events.filter((e) => e.subtype === "fire").length;
    const obstacleEvents = events.filter((e) => e.subtype === "obstacle").length;

    let severity = "ok";
    const notes = [];
    if (fireFrames > 0 || fireEvents > 0) {
      severity = "critical";
      notes.push(`FIRE SCREEN: flame colours on ${fireFrames}/${frames} frames (peak ${(peakFireRatio * 100).toFixed(2)}% of view), ${fireEvents} alert(s)`);
    }
    if (species.size) {
      notes.push(`wildlife: ${[...species].sort().join(", ")}`);
    }
    if (nearest !== null && nearest < 200) {
      if (severity === "ok") severity = "warning";
      notes.push(`nearest obstacle ${nearest} mm (${obstacleEvents} alert(s))`);
    }
    const stale = Object.entries(streams).filter(([, v]) => v.count === 0).map(([k]) => k);
    if (stale.length === 4) {
      severity = "critical";
      notes.push("no telemetry at all");
    } else if (stale.length) {
      if (severity === "ok") severity = "warning";
      notes.push(`no ${stale.join("/")} data`);
    }
    const top = [...objects.entries()].sort((a, b) => b[1] - a[1]).slice(0, 5)
      .map(([k, v]) => `${k}×${v}`);
    if (top.length) notes.push(`objects: ${top.join(", ")}`);

    return {
      thing, severity, streams, frames,
      nearest_mm: nearest,
      fire_frames: fireFrames, peak_fire_ratio: peakFireRatio,
      wildlife: [...species].sort(),
      top_objects: top,
      events: { fire: fireEvents, obstacle: obstacleEvents, total: events.length },
      detail: notes.join(" · ") || "nominal",
    };
  });

  return {
    agent: "per-rover",
    severity: worstSeverity(rovers),
    detail: rovers.map((r) => `${r.thing} [${r.severity}] ${r.detail}`).join("  ||  "),
    rovers,
  };
}

export async function runAgents(db, knownThings = []) {
  const rows = await loadWindow(db);
  const findings = [
    eventAgent(rows),
    thermalAgent(rows),
    fireAgent(rows),
    wildlifeAgent(rows),
    perRoverAgent(rows, knownThings),
    livenessAgent(rows, knownThings),
    coverageAgent(rows),
  ];
  const severity = worstSeverity(findings);

  const headline =
    severity === "critical" ? findings.find((f) => f.severity === "critical").detail
    : severity === "warning" ? findings.find((f) => f.severity === "warning").detail
    : `All clear — ${rows.length} readings analysed.`;

  return { ts: Date.now() / 1000, severity, summary: headline, findings, sample_size: rows.length };
}

/**
 * The "edge LM" pass — reasoning over the findings, on Cloudflare's edge.
 *
 * READ THE CONTRACT BEFORE CHANGING THIS:
 *
 * This runs ON TOP of the deterministic agents, never underneath them. It is
 * handed the findings they already produced and asked to explain them; it does
 * NOT get to decide whether something is on fire. `severity`, `summary` and the
 * alert-email trigger all stay exactly as the rule-based agents computed them.
 *
 * That split is deliberate and load-bearing. A fire system must give the same
 * verdict for the same numbers every time, and must keep working when an
 * inference endpoint is down or over quota. The moment a model's opinion can
 * raise or lower severity, both properties are gone. This function returning
 * null is a normal outcome, not a failure — the report is complete without it.
 *
 * It is also why the model is never shown raw camera frames or asked "is this a
 * fire?" here. That judgement belongs to the vision path, which has its own
 * structured verdict and its own benchmarks.
 *
 * Returns { assessment, recommended_action, confidence } or null.
 */
export async function reasonOverFindings(env, report, opts = {}) {
  if (!env?.AI || !report) return null;

  const model = opts.model || "@cf/meta/llama-4-scout-17b-16e-instruct";
  // Output tokens dominate neuron cost, and the free tier is 10,000/day with no
  // rollover. This runs on a 15-minute cron (96 calls/day), so keep it tight.
  const maxTokens = opts.maxTokens || 200;

  // Send the findings as compact facts, not prose. The model reasons better over
  // structure, and it keeps the input token count (and therefore cost) down.
  const facts = (report.findings || []).map((f) => ({
    agent: f.agent, severity: f.severity, detail: f.detail,
  }));

  const prompt =
    "You are a wildfire monitoring analyst. Below are findings already " +
    "produced by deterministic rule-based agents watching a rover fleet.\n\n" +
    `Overall severity (already decided, do not change it): ${report.severity}\n` +
    `Readings analysed: ${report.sample_size}\n` +
    `Findings: ${JSON.stringify(facts)}\n\n` +
    "Write a short operator-facing assessment explaining what these findings " +
    "mean together, and one concrete recommended action. Do not invent " +
    "measurements that are not in the findings. If the findings are all clear, " +
    "say so plainly rather than manufacturing concern.";

  try {
    const result = await env.AI.run(model, {
      messages: [{ role: "user", content: prompt }],
      response_format: {
        type: "json_schema",
        json_schema: {
          type: "object",
          properties: {
            assessment: { type: "string" },
            recommended_action: { type: "string" },
            confidence: { type: "string" },
          },
          required: ["assessment", "recommended_action"],
        },
      },
      max_tokens: maxTokens,
    });

    const r = result?.response;
    const obj = typeof r === "string" ? llmJson(r) : r;
    if (!obj || typeof obj.assessment !== "string" || !obj.assessment.trim()) return null;

    // json_schema shapes output but does not enforce `required`, so default
    // every field rather than trusting it to be present.
    return {
      assessment: obj.assessment.trim().slice(0, 1200),
      recommended_action:
        typeof obj.recommended_action === "string" ? obj.recommended_action.trim().slice(0, 400) : null,
      confidence: typeof obj.confidence === "string" ? obj.confidence.trim().slice(0, 40) : null,
      model,
    };
  } catch (err) {
    // Never let this take down a report. The deterministic findings are the
    // thing that matters and they are already computed.
    console.error(JSON.stringify({
      message: "edge LM reasoning failed", model,
      error: err instanceof Error ? err.message : String(err),
    }));
    return null;
  }
}

/** JSON.parse tolerant of ``` fences, which models add unprompted. */
function llmJson(s) {
  if (typeof s !== "string") return null;
  const cleaned = s.replace(/^\s*```(?:json)?\s*/i, "").replace(/\s*```\s*$/, "").trim();
  try {
    return JSON.parse(cleaned);
  } catch {
    const m = cleaned.match(/\{[\s\S]*\}/);
    if (!m) return null;
    try { return JSON.parse(m[0]); } catch { return null; }
  }
}

/** Plain-text + HTML email body for a report. */
export function renderReport(report) {
  const icon = { ok: "✓", warning: "⚠", critical: "ὒ5" };
  const rows = report.findings
    .map(
      (f) => `<tr>
        <td style="padding:6px 10px;border-bottom:1px solid #eee;font-weight:600">${f.agent}</td>
        <td style="padding:6px 10px;border-bottom:1px solid #eee;text-transform:uppercase;color:${
          f.severity === "critical" ? "#b91c1c" : f.severity === "warning" ? "#b45309" : "#15803d"
        }">${f.severity}</td>
        <td style="padding:6px 10px;border-bottom:1px solid #eee">${escapeHtml(f.detail)}</td>
      </tr>`,
    )
    .join("");

  // The LM assessment is labelled as such, and sits BELOW the deterministic
  // findings table in the text version. An operator must always be able to see
  // which lines a rule produced and which a model wrote.
  const r = report.reasoning;
  const reasoningHtml = r
    ? `<div style="margin:16px 0;padding:12px 14px;background:#f8fafc;border-left:3px solid #94a3b8">
         <div style="font-size:11px;text-transform:uppercase;letter-spacing:.05em;color:#64748b;font-weight:600">
           AI assessment · ${escapeHtml(r.model || "model")}
         </div>
         <p style="margin:6px 0 0;color:#334155">${escapeHtml(r.assessment)}</p>
         ${r.recommended_action
            ? `<p style="margin:8px 0 0;color:#334155"><strong>Recommended:</strong> ${escapeHtml(r.recommended_action)}</p>`
            : ""}
         <p style="margin:8px 0 0;font-size:11px;color:#94a3b8">
           Generated commentary. Severity above is set by deterministic rules, not by this model.
         </p>
       </div>`
    : "";

  const html = `<div style="font-family:system-ui,-apple-system,Segoe UI,sans-serif;max-width:640px">
    <h2 style="margin:0 0 4px">FPMS report — ${report.severity.toUpperCase()}</h2>
    <p style="color:#475569;margin:0 0 16px">${escapeHtml(report.summary)}</p>
    ${reasoningHtml}
    <table style="border-collapse:collapse;width:100%;font-size:14px">
      <thead><tr>
        <th align="left" style="padding:6px 10px;border-bottom:2px solid #cbd5e1">Agent</th>
        <th align="left" style="padding:6px 10px;border-bottom:2px solid #cbd5e1">Severity</th>
        <th align="left" style="padding:6px 10px;border-bottom:2px solid #cbd5e1">Finding</th>
      </tr></thead>
      <tbody>${rows}</tbody>
    </table>
    <p style="color:#64748b;font-size:12px;margin-top:16px">
      ${report.sample_size} readings analysed over the last hour ·
      generated ${new Date(report.ts * 1000).toISOString()}
    </p>
  </div>`;

  const text = `FPMS report — ${report.severity.toUpperCase()}\n${report.summary}\n\n` +
    report.findings.map((f) => `[${f.severity}] ${f.agent}: ${f.detail}`).join("\n") +
    (r
      ? `\n\n--- AI assessment (${r.model || "model"}; commentary only, ` +
        `severity above is rule-based) ---\n${r.assessment}` +
        (r.recommended_action ? `\nRecommended: ${r.recommended_action}` : "")
      : "");

  return { html, text, icon: icon[report.severity] };
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) =>
    ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}

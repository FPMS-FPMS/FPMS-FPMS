/* =========================================================================
   verify_render.js — proves the STALENESS CONTRACT and the SILENCE ALARM.
   =========================================================================

     node verify_render.js

   OPTIONAL, like verify_link.js. Nothing here is loaded by the page.

   WHAT IT IS FOR

   Two promises this dashboard makes are the kind that a comment cannot keep:

     1. "Every panel ages forward from receipt and greys out when stale."
        An operator decides whether to believe a number on the strength of
        that. So this drives a fake clock forward WITHOUT delivering any new
        message and asserts that the panels turn stale on their own — which
        is the only way a frozen publisher is ever caught.

     2. "Connected but receiving nothing is made loud, and named."
        /etc/fpms/config.env warns that a consumer on the wrong topic root
        "connects, authenticates, stays connected and receives nothing
        forever ... the broker, the bridge and every unit look healthy". So
        this reproduces exactly that shape — /diagnostics arriving, the
        /fpms/* mirrors silent — and asserts the banner says CHECK THE THING
        NAME rather than showing empty panels.

   It loads index.html, feeds.js, arena.js and app.js into a minimal DOM shim
   and pushes real-shaped rosbridge frames through them.

   THE CLOCK IS FAKED, NOT SLEPT. performance.now() is the only time source
   feeds.js uses — deliberately, because it is monotonic and an NTP step on
   the operator's laptop therefore cannot make a stale value look fresh — so
   moving that one function forward runs the whole suite instantly.
   ========================================================================= */
"use strict";

const fs = require("fs");
const path = require("path");

/* ==================================================== minimal DOM shim === */
const VOID = new Set(["meta", "link", "input", "br", "img", "hr", "source"]);

class El {
  constructor(tag) {
    this.tagName = (tag || "div").toUpperCase();
    this.attrs = {}; this.children = []; this.parentNode = null;
    this._text = ""; this._html = ""; this.style = {}; this._on = {};
    this.value = "";
    const self = this;
    this.classList = {
      add: (c) => { self._cls().add(c); self._sync(); },
      remove: (c) => { self._cls().delete(c); self._sync(); },
      toggle: (c, on) => { const s = self._cls(); if (on) { s.add(c); } else { s.delete(c); } self._sync(); },
      contains: (c) => self._cls().has(c),
    };
  }
  _cls() {
    if (!this._clsSet) {
      this._clsSet = new Set((this.attrs["class"] || "").split(/\s+/).filter(Boolean));
    }
    return this._clsSet;
  }
  _sync() { this.attrs["class"] = [...this._clsSet].join(" "); }
  get className() { return this.attrs["class"] || ""; }
  set className(v) { this.attrs["class"] = v; this._clsSet = null; }
  setAttribute(k, v) { this.attrs[k] = String(v); if (k === "class") { this._clsSet = null; } }
  getAttribute(k) { return this.attrs[k] === undefined ? null : this.attrs[k]; }
  get textContent() { return this._text; }
  set textContent(v) { this._text = String(v); }
  get innerHTML() { return this._html; }
  set innerHTML(v) { this._html = String(v); }
  appendChild(c) { c.parentNode = this; this.children.push(c); return c; }
  removeChild(c) { this.children = this.children.filter((x) => x !== c); return c; }
  get firstChild() { return this.children[0]; }
  get childNodes() { return this.children; }
  set scrollTop(_) {} get scrollTop() { return 0; }
  get scrollHeight() { return 0; }
  addEventListener(type, fn) { (this._on[type] = this._on[type] || []).push(fn); }
  dispatch(type, ev) { (this._on[type] || []).forEach((f) => f(ev || { preventDefault() {} })); }
  focus() {}
  getBoundingClientRect() { return { width: 800, height: 600, top: 0, left: 0 }; }
  getContext() { return CANVAS_CTX; }
  hidden() { return this._cls().has("hidden"); }
}

/* Canvas is exercised for crashes only — arena.js's arithmetic runs for real,
   which is the part that can throw on a malformed scan. */
const CANVAS_CTX = new Proxy({}, {
  get: (t, k) => {
    if (k === "canvas") { return { width: 800, height: 600 }; }
    if (["fillStyle", "strokeStyle", "lineWidth", "font", "textAlign",
         "lineJoin", "lineCap"].includes(k)) { return t[k]; }
    return () => {};
  },
  set: (t, k, v) => { t[k] = v; return true; },
});

function parseHtml(src) {
  const byId = {}, all = [];
  const root = new El("body");
  const stack = [root];
  const re = /<(\/?)([a-zA-Z0-9]+)((?:\s+[^>]*?)?)(\/?)>/g;
  let m;
  while ((m = re.exec(src))) {
    const [, closing, tag, attrStr, selfClose] = m;
    const t = tag.toLowerCase();
    if (closing) { if (stack.length > 1) { stack.pop(); } continue; }
    const el = new El(t);
    const ar = /([a-zA-Z-]+)\s*=\s*"([^"]*)"/g;
    let a;
    while ((a = ar.exec(attrStr))) { el.setAttribute(a[1], a[2]); }
    stack[stack.length - 1].appendChild(el);
    all.push(el);
    if (el.attrs.id) { byId[el.attrs.id] = el; }
    if (!selfClose && !VOID.has(t)) { stack.push(el); }
  }
  return { byId, all, root };
}

const HERE = __dirname;
const dom = parseHtml(fs.readFileSync(path.join(HERE, "index.html"), "utf8"));

let NOW = 1000;                                    // the fake monotonic clock
const timers = [];
let timerId = 1;

/* An optional local zones.json, resolved the same way serve.py resolves it. */
const ZONES_PATH = [
  path.join(HERE, "..", "rover", "fpms-os", "overlay", "etc", "fpms", "zones.json"),
].find((p) => fs.existsSync(p));

const g = {
  document: {
    getElementById: (id) => dom.byId[id] || null,
    querySelectorAll: (sel) => {
      const m = /^\[([a-zA-Z-]+)\]$/.exec(sel);
      return m ? dom.all.filter((e) => e.attrs[m[1]] !== undefined) : [];
    },
    createElement: (t) => new El(t),
    addEventListener: (ev, fn) => { if (ev === "DOMContentLoaded") { g._ready = fn; } },
    readyState: "loading",
    hidden: false,
    activeElement: { tagName: "BODY" },
  },
  addEventListener: () => {},
  navigator: { onLine: true },
  location: { search: "", hostname: "localhost" },
  localStorage: { getItem: () => null, setItem: () => {} },
  fetch: (url) => {
    if (/zones\.json/.test(url) && ZONES_PATH) {
      const body = fs.readFileSync(ZONES_PATH, "utf8");
      return Promise.resolve({ json: () => Promise.resolve(JSON.parse(body)) });
    }
    /* config.json is deliberately unavailable, so the run exercises the
       BUILT-IN rover1 defaults — the path an operator gets by opening the
       file directly. */
    return Promise.reject(new Error("not served in this test"));
  },
  URLSearchParams,
  performance: { now: () => NOW },
  setInterval: (fn, ms) => { const id = timerId++; timers.push({ id, fn, due: NOW + ms, every: ms }); return id; },
  clearInterval: (id) => { const i = timers.findIndex((t) => t.id === id); if (i >= 0) { timers.splice(i, 1); } },
  setTimeout: (fn, ms) => { const id = timerId++; timers.push({ id, fn, due: NOW + (ms || 0), every: 0 }); return id; },
  clearTimeout: (id) => { const i = timers.findIndex((t) => t.id === id); if (i >= 0) { timers.splice(i, 1); } },
  Math, Date, JSON, console,
};
g.window = g;

let FAILED = 0, PASSED = 0;
function advance(ms) {
  const end = NOW + ms;
  for (;;) {
    const due = timers.filter((t) => t.due <= end).sort((a, b) => a.due - b.due)[0];
    if (!due) { break; }
    NOW = due.due;
    if (due.every) { due.due = NOW + due.every; } else { timers.splice(timers.indexOf(due), 1); }
    try { due.fn(); } catch (e) { console.log("  timer threw: " + e.stack); FAILED++; }
  }
  NOW = end;
}

/* ------------------------------------------------- fake rosbridge socket */
let liveSockets = [];
class FakeWS {
  constructor(url) {
    this.url = url; this.readyState = 0; this.sent = [];
    liveSockets.push(this);
    /* Every socket comes up on the next tick, including the ones the silence
       remedy opens by itself. Without this the test would keep publishing
       into a socket the code under test had already replaced. */
    g.setTimeout(() => this.accept(), 1);
  }
  send(s) {
    if (this.readyState !== 1) { throw new Error("not open"); }
    const o = JSON.parse(s);
    this.sent.push(o);
    /* Answer the heartbeat exactly as a stock rosbridge does, so the link
       stays up across long clock advances instead of tearing itself down
       mid-test. An unknown op comes back as a status error carrying the id. */
    if (o.op === "__fpms_ping__") {
      this.deliver({ op: "status", level: "error", id: o.id, msg: "Unknown operation" });
    }
  }
  close() { if (this.readyState === 3) { return; } this.readyState = 3; if (this.onclose) { this.onclose({ code: 1000 }); } }
  accept() { if (this.readyState !== 0) { return; } this.readyState = 1; if (this.onopen) { this.onopen(); } }
  deliver(o) { if (this.onmessage) { this.onmessage({ data: JSON.stringify(o) }); } }
  pub(topic, msg) { this.deliver({ op: "publish", topic, msg }); }
}
g.WebSocket = FakeWS;

/* ------------------------------------------------------- load the app --- */
function load(file) {
  const src = fs.readFileSync(path.join(HERE, "assets", file), "utf8");
  new Function("window", "document", "performance", "WebSocket", "setTimeout",
               "clearTimeout", "setInterval", "clearInterval", "navigator",
               "location", "localStorage", "fetch", "URLSearchParams", "console", src)
    (g, g.document, g.performance, FakeWS, g.setTimeout, g.clearTimeout,
     g.setInterval, g.clearInterval, g.navigator, g.location, g.localStorage,
     g.fetch, URLSearchParams, console);
}

/* ------------------------------------------------------------ assertions */
function ok(cond, msg) {
  console.log((cond ? "  PASS  " : "  FAIL  ") + msg);
  if (cond) { PASSED++; } else { FAILED++; }
}
const txt = (id) => (dom.byId[id] ? dom.byId[id].textContent : "<<no element " + id + ">>");
const shown = (id) => !!dom.byId[id] && !dom.byId[id].classList.contains("hidden");
const lvl = (feed) => {
  const el = dom.all.find((e) => e.attrs["data-feed"] === feed);
  return el ? el.attrs["data-level"] : "<<no card for " + feed + ">>";
};
const links = () => (dom.byId.links ? dom.byId.links.innerHTML : "");
/* One link tile, by name: its css class and its rendered body. renderLinks()
   emits `<div class="ltile CLS"><div class="lrow"><span class="lname">NAME…`,
   and the class is the whole point of these assertions — a tile that says the
   right words in the wrong colour is still a lie. */
const tile = (name) => {
  const re = new RegExp('<div class="ltile ([a-z]+)"><div class="lrow">' +
                        '<span class="lname">' + name + '</span>' +
                        '([\\s\\S]*?)(?=<div class="ltile |$)');
  const m = re.exec(links());
  return m ? { cls: m[1], html: m[2] } : { cls: "<<no " + name + " tile>>", html: "" };
};
/* Resolve the CURRENT socket of each link every time. Both links dial the
   same URL, so they are told apart by what they asked for: only the main
   link ever subscribes. The silence remedy replaces sockets underneath us,
   and a test that cached socket[1] would quietly stop testing anything. */
const isMain = (s) => s.sent.some((m) => m.op === "subscribe");
const open1 = () => liveSockets.filter((s) => s.readyState === 1);
const mainSock = () => { const a = open1().filter(isMain); return a[a.length - 1]; };
const stopSock = () => { const a = open1().filter((s) => !isMain(s)); return a[a.length - 1]; };
const flush = () => new Promise((r) => setImmediate(r));

(async function main() {

load("rosbridge.js");
load("feeds.js");
load("arena.js");
load("app.js");
if (g._ready) { g._ready(); }
await flush(); await flush(); await flush();   // let the config.json rejection settle
advance(20);                                   // let both sockets come up

console.log("\n1. defaults are the ROVER 1 identity, and it is visible");
ok(liveSockets.length === 2, "two sockets dialed (main + dedicated STOP)");
ok(mainSock().url === "ws://fpms-rover1.local:9090",
   "dials the rover1 default: " + mainSock().url);
ok(txt("thingChip") === "rover1", "thing name shown in the header: '" + txt("thingChip") + "'");
ok(txt("roverHost") === "fpms-rover1.local:9090", "host shown in the header: " + txt("roverHost"));

console.log("\n2. zones.json is CHECKED, not copied");
if (ZONES_PATH) {
  const z = JSON.parse(fs.readFileSync(ZONES_PATH, "utf8"));
  const v = g.ARENA.validateZones(z);
  ok(v.ok, "the image's zones.json agrees with the derivation" +
     (v.ok ? " (" + v.checked + " centres, 0.05 mm tolerance)" : ": " + v.problems.join("; ")));
  const bad = JSON.parse(JSON.stringify(z));
  bad.zones["zone-a"].cx_mm = 328.0;                 // 100 mm out
  const vb = g.ARENA.validateZones(bad);
  ok(!vb.ok && /zone-a/.test(vb.problems.join(" ")),
     "a zone centre 100 mm out is REFUSED rather than adopted: " + vb.problems[0]);
  ok(g.ARENA.ZONES.find((q) => q.id === "water-station") !== undefined,
     "zone ids match zones.json (water-station, not water)");
  const za = g.ARENA.ZONES.find((q) => q.id === "zone-a");
  ok(za.cx === 228 && za.cy === 972 && za.corner === "TOP-LEFT" && za.mission === "m1",
     "zone-a derives to 228,972 TOP-LEFT (m1) — the corner names the zone, not the mission id");
} else {
  ok(true, "no zones.json in this checkout — skipped (absent file is a no-op by design)");
}

console.log("\n3. every topic is inside topics_glob, and no drive topic is touched");
advance(50);
const GLOB = ["/estop", "/fpms/*", "/clicked_point", "/scan_lidar", "/odom_raw",
              "/odom", "/imu", "/battery", "/wheel_ticks", "/wheel_duty",
              "/fpms_health", "/diagnostics", "/rosout"];
const globOk = (t) => GLOB.some((p) => (p.endsWith("*") ? t.startsWith(p.slice(0, -1)) : t === p));
const asked = [...mainSock().sent, ...stopSock().sent]
  .filter((m) => ["subscribe", "advertise", "publish"].includes(m.op))
  .map((m) => m.topic);
const outside = [...new Set(asked)].filter((t) => !globOk(t));
ok(outside.length === 0,
   "all " + new Set(asked).size + " topics are inside topics_glob" +
   (outside.length ? " — OUTSIDE: " + outside.join(", ") : ""));
ok(!asked.some((t) => /cmd_vel|cmd_duty|cmd_enable/.test(t)),
   "no drive topic requested — this page cannot move the rover");
ok(![...mainSock().sent, ...stopSock().sent].some((m) => m.op === "call_service"),
   "no ROS service called at all, so services_glob cannot affect this page");
ok(stopSock().sent.filter((m) => m.op === "subscribe").length === 0,
   "the STOP socket subscribes to NOTHING — its send buffer is empty by construction");

/* The camera/NPU health mirror published by fpms-telemetry-ros. Every name is
   asserted individually rather than by prefix: "/fpms/* covers it" is the
   claim, and a typo'd topic would satisfy the prefix and never arrive. */
const MIRROR = [
  "/fpms/camera/state", "/fpms/camera/health", "/fpms/camera/stale",
  "/fpms/camera/frame_age_s", "/fpms/camera/fps", "/fpms/camera/resolution",
  "/fpms/npu/state", "/fpms/npu/health", "/fpms/npu/detection_available",
  "/fpms/npu/model_loaded", "/fpms/npu/model_sha_state", "/fpms/npu/p50_ms",
  "/fpms/npu/p90_ms", "/fpms/npu/infer_per_s", "/fpms/npu/drops",
  "/fpms/npu/consecutive_failures", "/fpms/npu/core_mask",
  "/fpms/npu/fault", "/fpms/agent/fault",
];
const subscribed = new Set(mainSock().sent.filter((m) => m.op === "subscribe").map((m) => m.topic));
const missing = MIRROR.filter((t) => !subscribed.has(t));
ok(missing.length === 0,
   "all " + MIRROR.length + " camera/NPU health topics are subscribed" +
   (missing.length ? " — MISSING: " + missing.join(", ") : ""));
ok(MIRROR.every(globOk),
   "and every one of them is inside topics_glob via /fpms/* — the whitelist did not move");
/* expectS is what feeds the silence banner. These topics are quiet on any
   rover whose image predates fpms-telemetry-ros, and accusing them there
   would be a false alarm on a working rover. */
ok(!MIRROR.some((t) => new RegExp("expect .*" + t.replace(/\//g, "\\/")).test(txt("linkDiag"))),
   "none of them declares expectS, so a rover without fpms-telemetry-ros never " +
   "trips the silence banner (a false alarm is how an alarm becomes worthless)");

console.log("\n4. a value renders with an age, then goes STALE with NO new message");
mainSock().pub("/scan_lidar", { ranges: [1, 2, 3], angle_min: 0, angle_increment: 0.01, range_max: 12 });
advance(300);
ok(lvl("scan") === "ok", "fresh /scan_lidar card is ok");
ok(/3 rays/.test(txt("scanVal")), "renders the ray count: " + txt("scanVal"));
advance(3000);
ok(lvl("scan") === "warn", "at 3s WARN, with no message having arrived");
advance(3000);
ok(lvl("scan") === "stale", "at 6s STALE — the clock alone caught a frozen publisher");
ok(/STALE/.test(txt("scanVal")), "and the value says so: " + txt("scanVal"));
ok(!/rays/.test(txt("scanVal")), "the stale frame is DROPPED, not held on screen");

console.log("\n5. pose comes from /fpms/mission/*, judged by its WORST component");
mainSock().pub("/fpms/mission/x_mm", { data: 972 });
mainSock().pub("/fpms/mission/y_mm", { data: 228 });
mainSock().pub("/fpms/mission/heading_deg", { data: 90 });
advance(300);
ok(/972, 228 mm  hdg 90/.test(txt("poseVal")), "arena mm + heading: " + txt("poseVal"));
ok(lvl("pose") === "ok", "pose ok while all three are fresh");
advance(4000);
mainSock().pub("/fpms/mission/x_mm", { data: 975 });     // refresh x only
advance(300);
ok(lvl("pose") === "warn",
   "x is fresh but y/hdg are ageing — the pose card follows the WORST of the three, not x");
advance(3000);
mainSock().pub("/fpms/mission/x_mm", { data: 980 });     // still only x
advance(300);
ok(lvl("pose") === "stale", "a fresh x with a stale heading is NOT a fresh pose");
ok(/STALE/.test(txt("poseVal")), "and the readout says STALE: " + txt("poseVal"));

console.log("\n6. battery falls back to the mission mirror only while /battery is stale");
mainSock().pub("/battery", { voltage: 11.84 });
advance(300);
ok(/11\.84 V/.test(txt("battVal")) && /\/battery/.test(txt("battAge")),
   "shows /battery and names the source: " + txt("battVal") + " | " + txt("battAge"));
advance(12000);
mainSock().pub("/fpms/mission/batt_v", { data: 11.2 });
advance(300);
ok(/11\.20 V/.test(txt("battVal")) && /fallback/.test(txt("battAge")),
   "falls back and LABELS the fallback: " + txt("battVal") + " | " + txt("battAge"));

console.log("\n7. link health: real evidence for LIDAR/ESP32");
mainSock().pub("/fpms_health", { data: [30001, (1 << 4) | (1 << 3), 1, 0, 0, 0, 1000, 15, 11800, 0, 120, 640] });
advance(300);
ok(/ESP32[\s\S]*?fw 30001/.test(links()), "ESP32 tile reads the build off /fpms_health");
ok(/uptime 640s/.test(links()), "and the board uptime");
mainSock().pub("/fpms_health", { data: [30001, 1 << 3, 0, 60000, 60000, 0, 0, 0, 11800, 0, 120, 700] });
advance(300);
ok(/ESP32[\s\S]*?AGENT NOT CONNECTED/.test(links()),
   "bit4 clear reads as AGENT NOT CONNECTED, not as a healthy board");

/* =====================================================================
   7b–7h. THE CAMERA AND NPU TILES.

   These are the tiles that can do the most damage. A green NPU tile over a
   rover that is not detecting is the single worst output this dashboard has,
   so every state word the mirror can emit is driven through the real render
   path here and its COLOUR asserted — not merely its text.
   ===================================================================== */
console.log("\n7b. NO MIRROR AT ALL: nothing has published /fpms/camera/state — NO ROS SOURCE");
ok(tile("CAMERA").cls === "nosrc" && /NO ROS SOURCE/.test(tile("CAMERA").html),
   "CAMERA is hatched grey and says NO ROS SOURCE before the mirror ever speaks");
ok(tile("NPU").cls === "nosrc" && /NO ROS SOURCE/.test(tile("NPU").html),
   "NPU likewise — an absent bridge is not evidence about the sensor");
ok(/fpms-telemetry-ros/.test(tile("NPU").html),
   "and it names the unit to check rather than saying 'no data'");

console.log("\n7c. \"unknown\" IS NOT HEALTHY, AND IS NOT A FAULT — the load-bearing case");
/* The mirror is running and telling us it cannot see its source. This is the
   assertion the whole file exists for: never green. */
const CAM_WHY_UNKNOWN = "no camera frame and no agent heartbeat have arrived; " +
                        "this says nothing about the camera";
const NPU_WHY_UNKNOWN = "no telemetry/npu has ever arrived - fpms-npud may not be running. " +
                        "This is NOT evidence that detection works, and it is not evidence " +
                        "that it is broken.";
mainSock().pub("/fpms/camera/state", { data: "unknown" });
mainSock().pub("/fpms/camera/health", { data: JSON.stringify({
  component: "camera", state: "unknown", why: CAM_WHY_UNKNOWN,
  mqtt: { state: "connected", refused: [] } }) });
mainSock().pub("/fpms/npu/state", { data: "unknown" });
mainSock().pub("/fpms/npu/health", { data: JSON.stringify({
  component: "npu", state: "unknown", why: NPU_WHY_UNKNOWN,
  mqtt: { state: "connected", refused: [] } }) });
advance(300);
ok(tile("CAMERA").cls === "nosrc",
   "CAMERA state=unknown renders nosrc, NOT ok (class was '" + tile("CAMERA").cls + "')");
ok(tile("NPU").cls === "nosrc",
   "NPU state=unknown renders nosrc, NOT ok (class was '" + tile("NPU").cls + "')");
ok(tile("NPU").cls !== "ok" && tile("CAMERA").cls !== "ok",
   "NEITHER is green — an 'unknown' painted green is a blind rover reported healthy");
ok(tile("NPU").cls !== "down" && tile("CAMERA").cls !== "down",
   "and neither is red — 'I cannot see my source' is not a declared fault");
ok(tile("NPU").html.includes(NPU_WHY_UNKNOWN.replace(/&/g, "&amp;")),
   "the mirror's own `why` is shown VERBATIM, not paraphrased");

console.log("\n7d. a working camera and a working NPU do go green");
mainSock().pub("/fpms/camera/state", { data: "ok" });
mainSock().pub("/fpms/camera/health", { data: JSON.stringify({
  state: "ok", why: "frames arriving", stale: false, fps: 5.9,
  resolution: "640x480", agent_fps: 5.9, agent_heartbeat: "ok",
  mqtt: { state: "connected", refused: [] } }) });
mainSock().pub("/fpms/camera/stale", { data: false });
mainSock().pub("/fpms/camera/fps", { data: 5.9 });
mainSock().pub("/fpms/camera/frame_age_s", { data: 0.2 });
mainSock().pub("/fpms/camera/resolution", { data: "640x480" });
mainSock().pub("/fpms/npu/state", { data: "ok" });
mainSock().pub("/fpms/npu/health", { data: JSON.stringify({
  state: "ok", why: "detecting", detection_available: true, model_loaded: true,
  model_sha_state: "verified", p50_ms: 12.4, mqtt: { state: "connected", refused: [] } }) });
mainSock().pub("/fpms/npu/detection_available", { data: true });
mainSock().pub("/fpms/npu/model_loaded", { data: true });
mainSock().pub("/fpms/npu/model_sha_state", { data: "verified" });
mainSock().pub("/fpms/npu/p50_ms", { data: 12.4 });
mainSock().pub("/fpms/npu/p90_ms", { data: 19.1 });
mainSock().pub("/fpms/npu/infer_per_s", { data: 5.8 });
mainSock().pub("/fpms/npu/core_mask", { data: "0x7" });
advance(300);
ok(tile("CAMERA").cls === "ok" && /5\.9 fps/.test(tile("CAMERA").html),
   "CAMERA state=ok is green and carries the measured rate");
ok(/640x480/.test(tile("CAMERA").html), "and the resolution off the mirror");
ok(tile("NPU").cls === "ok" && /p50 12 ms/.test(tile("NPU").html),
   "NPU state=ok is green and carries the latency");

console.log("\n7e. detection_available=false CANNOT BE GREEN, whatever word arrives with it");
/* The mirror already turns det=false into state=fault. This asserts the
   dashboard does not DEPEND on it having done so: a contradictory pair — a
   healthy-looking state beside detection_available=false — must resolve
   pessimistically, because the pessimistic reading is the one that cannot
   get somebody hurt. */
mainSock().pub("/fpms/npu/state", { data: "ok" });
mainSock().pub("/fpms/npu/health", { data: JSON.stringify({
  state: "ok", why: "detecting", detection_available: false,
  mqtt: { state: "connected", refused: [] } }) });
mainSock().pub("/fpms/npu/detection_available", { data: false });
advance(300);
ok(tile("NPU").cls !== "ok",
   "a 'healthy' NPU reporting detection_available=false is NOT green (class '" +
   tile("NPU").cls + "')");
ok(tile("NPU").cls === "down", "it is red — the rover is not detecting");
ok(/NOT DETECTING/.test(tile("NPU").html), "and it says so in words: " +
   (/(THE ROVER IS NOT DETECTING)/.exec(tile("NPU").html) || ["<<not said>>"])[0]);

console.log("\n7f. `off` is an OPERATOR ACTION — amber, never red");
mainSock().pub("/fpms/camera/state", { data: "off" });
mainSock().pub("/fpms/camera/health", { data: JSON.stringify({
  state: "off", why: "streaming is switched off; the agent is not sending frames",
  mqtt: { state: "connected", refused: [] } }) });
advance(300);
ok(tile("CAMERA").cls === "warn",
   "CAMERA state=off renders warn (class '" + tile("CAMERA").cls + "')");
ok(tile("CAMERA").cls !== "down",
   "and NOT down — red for a thing somebody chose is how an operator learns to ignore red");
ok(/switched off/.test(tile("CAMERA").html), "the reason is the mirror's own sentence");

console.log("\n7g. degraded -> warn; stale and fault -> down; an unknown WORD -> nosrc");
mainSock().pub("/fpms/npu/state", { data: "degraded" });
mainSock().pub("/fpms/npu/health", { data: JSON.stringify({
  state: "degraded", why: "the loaded model contradicts /etc/fpms/models.json",
  detection_available: true, model_sha_state: "mismatch",
  mqtt: { state: "connected", refused: [] } }) });
mainSock().pub("/fpms/npu/detection_available", { data: true });
mainSock().pub("/fpms/npu/model_sha_state", { data: "mismatch" });
advance(300);
ok(tile("NPU").cls === "warn", "degraded -> warn (serving, but not as configured)");
mainSock().pub("/fpms/npu/state", { data: "fault" });
advance(300);
ok(tile("NPU").cls === "down", "fault -> down");
mainSock().pub("/fpms/camera/state", { data: "stale" });
advance(300);
ok(tile("CAMERA").cls === "down", "stale -> down (it was alive and it stopped)");
mainSock().pub("/fpms/camera/state", { data: "sploorf" });
advance(300);
ok(tile("CAMERA").cls === "nosrc",
   "a state word this dashboard does not know is NOT coloured good or bad");
ok(/does not recognise/.test(tile("CAMERA").html), "and it says that it does not know it");

console.log("\n7h. a REFUSED MQTT subscription is named, not left looking like a dead sensor");
mainSock().pub("/fpms/camera/state", { data: "unknown" });
mainSock().pub("/fpms/camera/health", { data: JSON.stringify({
  state: "unknown", why: CAM_WHY_UNKNOWN,
  mqtt: { state: "connected", subscribed: 0, expected: 7,
          refused: [{ topic: "fpms/rover1/telemetry/camera", code: "128" }] } }) });
advance(300);
ok(/BROKER REFUSED/.test(tile("CAMERA").html),
   "the ACL refusal is called out by name");
ok(/fpms\.acl/.test(tile("CAMERA").html), "and the file to check is named");
ok(tile("CAMERA").cls === "nosrc",
   "still grey, not red: an ACL refusal is not evidence the camera failed");

console.log("\n7i. THE MIRROR ITSELF GOING QUIET reverts to NO ROS SOURCE");
/* /fpms/camera/state is published at 1 Hz unconditionally, carrying even the
   word "unknown". So silence on it means fpms-telemetry-ros is gone — which
   is a statement about the bridge and about nothing else. */
mainSock().pub("/fpms/npu/state", { data: "ok" });
mainSock().pub("/fpms/npu/health", { data: JSON.stringify({
  state: "ok", why: "detecting", detection_available: true, model_loaded: true,
  model_sha_state: "verified", mqtt: { state: "connected", refused: [] } }) });
mainSock().pub("/fpms/npu/detection_available", { data: true });
mainSock().pub("/fpms/npu/model_sha_state", { data: "verified" });
advance(300);
ok(tile("NPU").cls === "ok", "NPU green while the mirror is talking");
advance(12000);                       // no message of any kind: 12 s > stale 10 s
ok(tile("NPU").cls === "nosrc" && /NO ROS SOURCE/.test(tile("NPU").html),
   "a stale mirror reverts to NO ROS SOURCE — never holds the last green tile");
ok(tile("CAMERA").cls === "nosrc" && /NO ROS SOURCE/.test(tile("CAMERA").html),
   "same for CAMERA, from the clock alone, with no message having arrived");
ok(/down or unreachable/.test(tile("NPU").html),
   "and it blames the bridge rather than the sensor");

console.log("\n8. residual verdicts follow STACK.md's table, and refuse to guess early");
mainSock().pub("/fpms/residual/raw", { data: JSON.stringify({ kind: "drive", leg_i: 0, segment_i: 0, target: 100, measured: 41, residual: -59, ratio: 0.41 }) });
advance(300);
ok(/at least 3 drive segments/.test(txt("resVerdict")),
   "one segment is not a pattern: " + txt("resVerdict").slice(0, 55) + "…");
mainSock().pub("/fpms/residual/raw", { data: JSON.stringify({ kind: "drive", leg_i: 0, segment_i: 1, target: 200, measured: 81, residual: -119, ratio: 0.405 }) });
mainSock().pub("/fpms/residual/raw", { data: JSON.stringify({ kind: "drive", leg_i: 0, segment_i: 2, target: 70, measured: 28.5, residual: -41.5, ratio: 0.407 }) });
advance(300);
ok(/CONSTANT RATIO/.test(txt("resVerdict")) && /SCALE ERROR/.test(txt("resVerdict")),
   "three matching ratios -> SCALE ERROR: " + txt("resVerdict").slice(0, 80) + "…");
ok(/odom_scale/.test(txt("resVerdict")), "and it names the knob to turn");

console.log("\n9. IDENTITY MISMATCH is loud — hardware_id vs the dashboard's thing");
mainSock().pub("/diagnostics", {
  status: [{ name: "ros/scan_lidar", level: 0, message: "9.80 Hz",
             hardware_id: "rover2", values: [{ key: "hz", value: "9.80" }] }],
});
advance(400);
ok(shown("idBanner"), "the identity banner is shown when the rover says rover2");
ok(/rover2/.test(txt("idDetail")) && /rover1/.test(txt("idDetail")),
   "and it names BOTH sides: " + txt("idDetail").slice(0, 100) + "…");

console.log("\n10. matching identity is silent — no banner for the normal case");
mainSock().pub("/diagnostics", {
  status: [{ name: "ros/scan_lidar", level: 0, message: "9.80 Hz", hardware_id: "rover1", values: [] }],
});
advance(400);
ok(!shown("idBanner"), "agreement shows nothing at all");

console.log("\n11. THE WRONG-TOPIC-ROOT SIGNATURE: /diagnostics arriving, mirrors silent");
/* Exactly the shape config.env warns about: connected, authenticated, every
   unit healthy, and no mission telemetry — because the bridge is subscribed
   to fpms/<other-thing>/telemetry/#. Keep /diagnostics and /scan_lidar
   flowing so ONLY the mirrors are quiet. */
for (let i = 0; i < 18; i++) {
  mainSock().pub("/diagnostics", { status: [{ name: "ros/scan_lidar", level: 0, message: "ok", hardware_id: "rover1", values: [] }] });
  mainSock().pub("/scan_lidar", { ranges: [1], angle_min: 0, angle_increment: 0.01, range_max: 12 });
  advance(2000);
}
ok(shown("silenceBanner"), "the silence banner is shown");
ok(/CHECK THE THING NAME/.test(txt("silenceHead")),
   "and it names the most likely cause: " + txt("silenceHead"));
ok(/fpms\/mission\/state/.test(txt("silenceBody")),
   "naming the silent topic, not just 'no data'");
ok(/rover1/.test(txt("silenceBody")), "and the thing name currently selected");
ok(!/scan_lidar/.test(txt("silenceBody")),
   "the topics that ARE arriving are not accused");

console.log("\n12. the banner clears the moment the mirrors deliver");
mainSock().pub("/fpms/mission/state", { data: JSON.stringify({ state: "running", phase: "drive" }) });
mainSock().pub("/fpms/mission/phase", { data: "drive" });
advance(400);
ok(!shown("silenceBanner"), "silence banner gone once the mirrors arrive");
ok(txt("stateVal") === "running", "and the whole-payload JSON mirror is parsed: " + txt("stateVal"));

console.log("\n13. STOP publishes both verbs on the dedicated socket");
const before = stopSock().sent.length;
dom.byId.stopBtn.dispatch("click");
const after = stopSock().sent.slice(before);
ok(after.some((m) => m.op === "publish" && m.topic === "/estop" && m.msg.data === true),
   "/estop true published on the STOP socket");
ok(after.some((m) => m.op === "publish" && m.topic === "/fpms/cmd/stop"),
   "/fpms/cmd/stop published on the STOP socket");
ok(!mainSock().sent.slice(-4).some((m) => m.op === "publish" && m.topic === "/estop"),
   "and NOT on the main socket, which carries the 10 Hz scan");

console.log("\n14. the ticker survives a hostile scan without throwing");
mainSock().pub("/scan_lidar", { ranges: [0, Infinity, NaN, -1, 0.5, null, 99999],
                                angle_min: -3.14, angle_increment: 0.9, range_max: 12 });
mainSock().pub("/fpms/mission/x_mm", { data: 500 });
mainSock().pub("/fpms/mission/y_mm", { data: 500 });
mainSock().pub("/fpms/mission/heading_deg", { data: 45 });
advance(600);
ok(true, "zeros, infinities, NaN and nulls in ranges rendered without throwing");

console.log("\n" + PASSED + " passed, " + FAILED + " failed");
process.exit(FAILED ? 1 : 0);

})();

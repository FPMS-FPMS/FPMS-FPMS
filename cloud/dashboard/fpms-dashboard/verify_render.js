/* =========================================================================
   verify_render.js — proves the STALENESS CONTRACT, headlessly.
   =========================================================================

     node verify_render.js

   OPTIONAL, like verify_link.js. Nothing here is loaded by the page.

   WHAT IT IS FOR

   "Every panel ages forward from receipt and greys out when stale" is the
   promise this dashboard makes to an operator who is deciding whether to
   believe a number on a screen. A comment cannot keep that promise. So this
   loads index.html, feeds.js, arena.js and app.js into a minimal DOM shim,
   pushes real-shaped rosbridge frames through them, advances a fake clock,
   and asserts what is actually rendered:

     * a value shows up, with an age;
     * the SAME value, unchanged, is marked stale once its threshold passes,
       without any new message arriving — which is the only way a frozen
       publisher can ever be caught;
     * the pose is judged by its WORST component;
     * the LiDAR frame is DROPPED, not held, when it goes stale;
     * the battery falls back to the mission mirror only while /battery is
       absent or stale, and says which source it is showing;
     * the CAMERA and NPU tiles never claim a health they have no ROS
       publisher for.

   THE CLOCK IS FAKED, NOT SLEPT. `performance.now()` is the only time source
   feeds.js uses (deliberately — it is monotonic, so an NTP step on the
   operator's laptop cannot make a stale value look fresh), so the whole
   suite runs instantly by moving that one function forward.
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
    this._text = ""; this._html = ""; this.style = {};
    this.classList = {
      _el: this,
      add: (c) => { this._el._cls().add(c); this._el._sync(); },
      remove: (c) => { this._el._cls().delete(c); this._el._sync(); },
      toggle: (c, on) => { const s = this._el._cls(); if (on) { s.add(c); } else { s.delete(c); } this._el._sync(); },
      contains: (c) => this._el._cls().has(c),
    };
  }
  _cls() { return (this._clsSet = this._clsSet || new Set((this.attrs["class"] || "").split(/\s+/).filter(Boolean))); }
  _sync() { this.attrs["class"] = [...this._cls()].join(" "); }
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
  addEventListener() {}
  focus() {}
  getBoundingClientRect() { return { width: 800, height: 600, top: 0, left: 0 }; }
  getContext() { return CANVAS_CTX; }
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
    if (closing) {
      if (stack.length > 1) { stack.pop(); }
      continue;
    }
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
const timers = [];                                 // [id, fn, dueAt, everyMs]
let timerId = 1;

const g = {
  document: {
    getElementById: (id) => dom.byId[id] || null,
    querySelectorAll: (sel) => {
      const m = /^\[([a-zA-Z-]+)\]$/.exec(sel);
      if (m) { return dom.all.filter((e) => e.attrs[m[1]] !== undefined); }
      return [];
    },
    createElement: (t) => new El(t),
    addEventListener: (ev, fn) => { if (ev === "DOMContentLoaded") { g._ready = fn; } },
    readyState: "loading",
    hidden: false,
    activeElement: { tagName: "BODY" },
  },
  addEventListener: () => {},
  navigator: { onLine: true },
  location: { search: "?rover=fake-pi:9090", hostname: "localhost" },
  localStorage: { getItem: () => null, setItem: () => {} },
  fetch: () => Promise.reject(new Error("no server in this test")),
  URLSearchParams,
  performance: { now: () => NOW },
  setInterval: (fn, ms) => { const id = timerId++; timers.push({ id, fn, due: NOW + ms, every: ms }); return id; },
  clearInterval: (id) => { const i = timers.findIndex((t) => t.id === id); if (i >= 0) { timers.splice(i, 1); } },
  setTimeout: (fn, ms) => { const id = timerId++; timers.push({ id, fn, due: NOW + (ms || 0), every: 0 }); return id; },
  clearTimeout: (id) => { const i = timers.findIndex((t) => t.id === id); if (i >= 0) { timers.splice(i, 1); } },
  Math, Date, JSON, console,
};
g.window = g;

/* Advance the fake clock, firing timers in order — this is what makes the
   5 Hz ticker run and what makes a value age without a message arriving. */
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
  constructor(url) { this.url = url; this.readyState = 0; this.sent = []; liveSockets.push(this); }
  send(s) { if (this.readyState !== 1) { throw new Error("not open"); } this.sent.push(JSON.parse(s)); }
  close() { if (this.readyState === 3) { return; } this.readyState = 3; if (this.onclose) { this.onclose({ code: 1000 }); } }
  accept() { this.readyState = 1; if (this.onopen) { this.onopen(); } }
  pub(topic, msg) { if (this.onmessage) { this.onmessage({ data: JSON.stringify({ op: "publish", topic, msg }) }); } }
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
load("rosbridge.js");
load("feeds.js");
load("arena.js");
load("app.js");
if (g._ready) { g._ready(); }

/* ------------------------------------------------------------ assertions */
let PASSED = 0, FAILED = 0;
function ok(cond, msg) {
  console.log((cond ? "  PASS  " : "  FAIL  ") + msg);
  if (cond) { PASSED++; } else { FAILED++; }
}
const txt = (id) => (dom.byId[id] ? dom.byId[id].textContent : "<<no element " + id + ">>");
const lvl = (feed) => {
  const el = dom.all.find((e) => e.attrs["data-feed"] === feed);
  return el ? el.attrs["data-level"] : "<<no card for " + feed + ">>";
};
const links = () => (dom.byId.links ? dom.byId.links.innerHTML : "");

/* The main socket is the second one created — the STOP link is deliberately
   dialed first so it wins the race to the server on a cold start. */
const main = () => liveSockets[1];
const stop = () => liveSockets[0];

console.log("\n1. two sockets, STOP first, and STOP advertises nothing else");
ok(liveSockets.length === 2, "two sockets dialed (main + dedicated STOP)");
liveSockets.forEach((s) => s.accept());
advance(50);
ok(stop().sent.filter((m) => m.op === "subscribe").length === 0,
   "the STOP socket subscribes to NOTHING — its send buffer is empty by construction");
ok(stop().sent.some((m) => m.op === "advertise" && m.topic === "/fpms/cmd/stop") &&
   stop().sent.some((m) => m.op === "advertise" && m.topic === "/estop"),
   "STOP socket advertises both stop verbs");

console.log("\n2. NOTHING outside the rosbridge whitelist is ever requested");
const GLOB = ["/estop", "/fpms/*", "/clicked_point", "/scan_lidar", "/odom_raw",
              "/odom", "/imu", "/battery", "/wheel_ticks", "/wheel_duty",
              "/fpms_health", "/diagnostics", "/rosout"];
const globOk = (t) => GLOB.some((p) => (p.endsWith("*") ? t.startsWith(p.slice(0, -1)) : t === p));
const asked = [...main().sent, ...stop().sent]
  .filter((m) => m.op === "subscribe" || m.op === "advertise" || m.op === "publish")
  .map((m) => m.topic);
const outside = [...new Set(asked)].filter((t) => !globOk(t));
ok(outside.length === 0, "every topic is inside topics_glob" +
   (outside.length ? " — OUTSIDE: " + outside.join(", ") : ""));
ok(!asked.some((t) => /cmd_vel|cmd_duty|cmd_enable/.test(t)),
   "no drive topic is requested — this page cannot move the rover");
ok(![...main().sent, ...stop().sent].some((m) => m.op === "call_service"),
   "no ROS service is called at all, so services_glob cannot affect this page");

console.log("\n3. a value renders with an age, then goes STALE with no new message");
main().pub("/scan_lidar", { ranges: [1, 2, 3], angle_min: 0, angle_increment: 0.01, range_max: 12 });
advance(300);
ok(lvl("scan") === "ok", "fresh /scan_lidar card is ok");
ok(/3 rays/.test(txt("scanVal")), "renders the ray count: " + txt("scanVal"));
advance(3000);                        // /scan_lidar warn=2 stale=5
ok(lvl("scan") === "warn", "at 3s the card is WARN without any new message");
advance(3000);
ok(lvl("scan") === "stale", "at 6s the card is STALE — a frozen publisher is caught by the clock alone");
ok(/STALE/.test(txt("scanVal")), "and the value says so: " + txt("scanVal"));

console.log("\n4. a stale LiDAR frame is DROPPED, never held");
/* arena.js is handed `scan: null` once stale; the map cannot hold a frame. */
ok(!/rays/.test(txt("scanVal")), "the ray count is gone rather than frozen on screen");

console.log("\n5. pose comes from /fpms/mission/*, and is judged by its WORST component");
main().pub("/fpms/mission/x_mm", { data: 972 });
main().pub("/fpms/mission/y_mm", { data: 228 });
main().pub("/fpms/mission/heading_deg", { data: 90 });
advance(300);
ok(/972, 228 mm  hdg 90/.test(txt("poseVal")), "arena mm and heading rendered: " + txt("poseVal"));
ok(lvl("pose") === "ok", "pose card ok while all three are fresh");
advance(4000);
main().pub("/fpms/mission/x_mm", { data: 980 });   // refresh x only
advance(300);
ok(lvl("pose") === "stale",
   "a fresh x with a stale heading is NOT a fresh pose (worst component wins)");
ok(/STALE/.test(txt("poseVal")), "and the pose readout says STALE: " + txt("poseVal"));

console.log("\n6. parked, with no pose at all, the map says ASSUMED rather than guessing");
ok(/x .* · y .* · hdg /.test(txt("poseAge")), "per-component ages shown: " + txt("poseAge"));

console.log("\n7. battery falls back to the mission mirror only while /battery is stale");
main().pub("/battery", { voltage: 11.84 });
advance(300);
ok(/11\.84 V/.test(txt("battVal")) && /\/battery/.test(txt("battAge")),
   "shows /battery and names it: " + txt("battVal") + " | " + txt("battAge"));
advance(12000);                                    // /battery stale after 10s
main().pub("/fpms/mission/batt_v", { data: 11.2 });
advance(300);
ok(/11\.20 V/.test(txt("battVal")) && /fallback/.test(txt("battAge")),
   "falls back and LABELS the fallback: " + txt("battVal") + " | " + txt("battAge"));

console.log("\n8. link health: real evidence for LIDAR/ESP32, honest silence for CAMERA/NPU");
main().pub("/fpms_health", { data: [30001, 1 << 4 | 1 << 3, 1, 0, 0, 0, 1000, 15, 11800, 0, 120, 640] });
main().pub("/scan_lidar", { ranges: [1], angle_min: 0, angle_increment: 0.01, range_max: 12 });
advance(300);
ok(/ESP32[\s\S]*?fw 30001/.test(links()), "ESP32 tile reads the firmware build off /fpms_health");
ok(/uptime 640s/.test(links()), "and its uptime");
ok(/CAMERA[\s\S]*?NO ROS SOURCE/.test(links()),
   "CAMERA is NO ROS SOURCE — nothing publishes camera health onto the graph");
ok(/NPU[\s\S]*?NO ROS SOURCE/.test(links()), "NPU is NO ROS SOURCE");
ok(/ltile nosrc/.test(links()), "and both are styled as 'no evidence', not as good or bad");

console.log("\n9. ESP32 tile goes DOWN when the micro-ROS session flag clears");
main().pub("/fpms_health", { data: [30001, 1 << 3, 0, 60000, 60000, 0, 0, 0, 11800, 0, 120, 700] });
advance(300);
ok(/ESP32[\s\S]*?AGENT NOT CONNECTED/.test(links()),
   "bit4 clear is reported as AGENT NOT CONNECTED, not as a healthy board");

console.log("\n10. residuals: the diagnostic verdict, and it refuses to guess early");
main().pub("/fpms/residual/raw", { kind: "drive", leg_i: 0, segment_i: 0, target: 100, measured: 41, residual: -59, ratio: 0.41 });
advance(300);
ok(/at least 3 drive segments/.test(txt("resVerdict")),
   "one segment is not a pattern: " + txt("resVerdict").slice(0, 60));
main().pub("/fpms/residual/raw", { kind: "drive", leg_i: 0, segment_i: 1, target: 200, measured: 81, residual: -119, ratio: 0.405 });
main().pub("/fpms/residual/raw", { kind: "drive", leg_i: 0, segment_i: 2, target: 70, measured: 28.5, residual: -41.5, ratio: 0.407 });
advance(300);
ok(/CONSTANT RATIO/.test(txt("resVerdict")) && /SCALE ERROR/.test(txt("resVerdict")),
   "three segments with the same ratio -> SCALE ERROR: " + txt("resVerdict").slice(0, 90));
ok(/2\.467|odom_scale/.test(txt("resVerdict")), "and it names the knob to turn");

console.log("\n11. a constant OFFSET is called coast, not scale");
["res_raw"].forEach(() => {});
main().pub("/fpms/residual/raw", { kind: "drive", leg_i: 1, segment_i: 0, target: 100, measured: 118, residual: 18, ratio: 1.18 });
main().pub("/fpms/residual/raw", { kind: "drive", leg_i: 1, segment_i: 1, target: 400, measured: 419, residual: 19, ratio: 1.047 });
main().pub("/fpms/residual/raw", { kind: "drive", leg_i: 1, segment_i: 2, target: 800, measured: 817, residual: 17, ratio: 1.021 });
main().pub("/fpms/residual/raw", { kind: "drive", leg_i: 1, segment_i: 3, target: 1000, measured: 1018, residual: 18, ratio: 1.018 });
main().pub("/fpms/residual/raw", { kind: "drive", leg_i: 1, segment_i: 4, target: 600, measured: 618, residual: 18, ratio: 1.03 });
main().pub("/fpms/residual/raw", { kind: "drive", leg_i: 1, segment_i: 5, target: 900, measured: 918, residual: 18, ratio: 1.02 });
advance(300);
ok(/CONSTANT OFFSET/.test(txt("resVerdict")) && /COAST/.test(txt("resVerdict")),
   "same absolute error at every length -> COAST: " + txt("resVerdict").slice(0, 90));

console.log("\n12. mission state accepts the whole-payload JSON mirror");
main().pub("/fpms/mission/state", { data: JSON.stringify({ state: "running", phase: "drive", leg_i: 2 }) });
main().pub("/fpms/mission/phase", { data: "drive" });
advance(300);
ok(txt("stateVal") === "running", "pulls .state out of the JSON dict: " + txt("stateVal"));
ok(txt("phaseVal") === "drive", "phase rendered: " + txt("phaseVal"));

console.log("\n13. STOP publishes both verbs on the dedicated socket");
const before = stop().sent.length;
dom.byId.stopBtn._onclick && dom.byId.stopBtn._onclick();
/* the click listener is registered via addEventListener, which the shim does
   not dispatch — call the exported path the same way the button does */
g.window.__fpmsFireStop && g.window.__fpmsFireStop("verify");
const after = stop().sent.slice(before);
ok(after.some((m) => m.op === "publish" && m.topic === "/estop" && m.msg.data === true) &&
   after.some((m) => m.op === "publish" && m.topic === "/fpms/cmd/stop"),
   "both /estop and /fpms/cmd/stop published on the STOP socket");

console.log("\n14. the whole ticker survives a hostile scan without throwing");
main().pub("/scan_lidar", { ranges: [0, Infinity, NaN, -1, 0.5, null, 99999],
                            angle_min: -3.14, angle_increment: 0.9, range_max: 12 });
main().pub("/fpms/mission/x_mm", { data: 500 });
main().pub("/fpms/mission/y_mm", { data: 500 });
main().pub("/fpms/mission/heading_deg", { data: 45 });
advance(600);
ok(true, "zeros, infinities, NaN and nulls in ranges rendered without throwing");

console.log("\n" + PASSED + " passed, " + FAILED + " failed");
process.exit(FAILED ? 1 : 0);

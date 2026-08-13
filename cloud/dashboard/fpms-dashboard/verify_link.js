/* =========================================================================
   verify_link.js — proves the reconnect logic instead of asserting it.
   =========================================================================

     node verify_link.js

   OPTIONAL. The dashboard does not need node to run — this is a test, not a
   build step, and nothing here is loaded by the page. It exists because
   "it must never lose the connection" is the headline requirement of this
   dashboard and a comment claiming a socket recovers is worth nothing.

   It loads assets/rosbridge.js unmodified, hands it a FAKE WebSocket, and
   drives the failure modes that actually happen on this rover:

     * a socket that closes,
     * a socket that OPENS AND IMMEDIATELY FLAPS (the case where the naive
       "reset backoff in onopen" turns a backoff into a hammer),
     * a socket that hangs in CONNECTING and never fires an event,
     * a socket that is OPEN and silently dead — no close, no error,
     * rosbridge accepting a subscription and delivering nothing forever
       (the ROS_PORT.md wedge),
     * re-pointing at a different rover without leaving a zombie dialing the
       old one.

   Timings are compressed via the same options the page passes, so the whole
   suite runs in a few seconds.
   ========================================================================= */
"use strict";

const fs = require("fs");
const path = require("path");

const SRC = fs.readFileSync(path.join(__dirname, "assets", "rosbridge.js"), "utf8");

/* ------------------------------------------------------- browser stand-in */
const g = {
  addEventListener: () => {},
  document: { addEventListener: () => {}, hidden: false },
  navigator: { onLine: true },
};
g.window = g;

let sockets = [];
class FakeWS {
  constructor(url) {
    this.url = url; this.readyState = 0; this.sent = []; this._answered = {};
    sockets.push(this);
  }
  send(s) {
    if (this.readyState !== 1) { throw new Error("not open"); }
    this.sent.push(JSON.parse(s));
  }
  close() {
    if (this.readyState === 3) { return; }
    this.readyState = 3;
    if (this.onclose) { this.onclose({ code: 1006 }); }
  }
  /* test-side controls */
  accept() { this.readyState = 1; if (this.onopen) { this.onopen(); } }
  deliver(o) { if (this.onmessage) { this.onmessage({ data: JSON.stringify(o) }); } }
  /* answer any outstanding heartbeat probe exactly as a stock rosbridge does:
     an unknown op comes back as a status error carrying the request id. */
  answerProbe() {
    const p = this.sent.filter((m) => m.op === "__fpms_ping__").pop();
    if (p && !this._answered[p.id]) {
      this._answered[p.id] = 1;
      this.deliver({ op: "status", level: "error", id: p.id, msg: "Unknown operation" });
      return true;
    }
    return false;
  }
}
g.WebSocket = FakeWS;
g.performance = { now: () => Number(process.hrtime.bigint() / 1000000n) };

new Function("window", "WebSocket", "performance", "setTimeout", "clearTimeout",
             "setInterval", "clearInterval", "navigator", "document", SRC)
  (g, FakeWS, g.performance, setTimeout, clearTimeout, setInterval, clearInterval,
   g.navigator, g.document);

const RosLink = g.RosLink;

/* --------------------------------------------------------------- harness */
const logs = [];
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
let pass = 0, fail = 0;
function ok(cond, msg) {
  console.log((cond ? "  PASS  " : "  FAIL  ") + msg);
  if (cond) { pass++; } else { fail++; }
}
const forUrl = (u) => sockets.filter((s) => s.url === u);
const latest = (u) => forUrl(u)[forUrl(u).length - 1];
const FAST = {
  backoffBaseMs: 60, backoffFloorMs: 20, backoffFactor: 2, backoffCapMs: 4000,
  connectTimeoutMs: 400, stableMs: 200, quietS: 0.3, probeTimeoutS: 0.3,
  wedgeS: 0.6, superviseMs: 50,
};
const opts = (url, label, extra) =>
  Object.assign({ url, label, onlog: (m) => logs.push(m) }, FAST, extra || {});

(async function main() {
  /* ------------------------------------------------------------------- */
  console.log("\n1. connect, replay every advertise and subscribe, go LIVE on data");
  const U = "ws://rover-a:9090";
  const L = new RosLink(opts(U, "main"));
  let delivered = 0;
  /* expectS declared on both, as the page does for its always-on topics. */
  L.subscribe("/scan_lidar", "sensor_msgs/LaserScan", () => { delivered++; }, { expectS: 0.6 });
  L.subscribe("/diagnostics", "diagnostic_msgs/DiagnosticArray", () => {}, { expectS: 0.6 });
  L.subscribe("/fpms/residual/raw", "std_msgs/String", () => {});   // event-driven: never flagged
  L.advertise("/estop", "std_msgs/Bool");
  await sleep(30);
  ok(forUrl(U).length === 1, "exactly one socket dialed");
  latest(U).accept();
  await sleep(20);
  ok(latest(U).sent.some((m) => m.op === "advertise" && m.topic === "/estop"),
     "advertise replayed on open");
  ok(latest(U).sent.some((m) => m.op === "subscribe" && m.topic === "/scan_lidar"),
     "subscribe replayed on open");
  latest(U).deliver({ op: "publish", topic: "/scan_lidar", msg: {} });
  await sleep(20);
  ok(delivered === 1, "handler received the message");
  ok(L.snapshot().phase === "live", "phase is LIVE only once topic data arrived");

  /* ------------------------------------------------------------------- */
  console.log("\n2. heartbeat: a quiet socket is probed, and the probe is answerable");
  await sleep(450);
  ok(!!latest(U).sent.find((m) => m.op === "__fpms_ping__"),
     "unknown-op probe sent after the quiet period");
  ok(latest(U).answerProbe(), "probe answered as a stock rosbridge would");
  await sleep(120);
  ok(L.isOpen(), "socket kept once the probe came back");

  /* ------------------------------------------------------------------- */
  console.log("\n3. a SILENTLY DEAD socket (open, no close, no error) is detected");
  const before3 = forUrl(U).length;
  await sleep(1300);              // quiet -> probe -> nobody answers
  ok(forUrl(U).length > before3, "socket torn down and redialed");
  ok(logs.some((m) => /HEARTBEAT LOST/.test(m)), "heartbeat loss logged in plain words");

  /* ------------------------------------------------------------------- */
  console.log("\n4. every subscription is restored on the NEW socket");
  L.reconnectNow(); await sleep(20);
  latest(U).accept(); await sleep(30);
  ok(latest(U).sent.some((m) => m.op === "subscribe" && m.topic === "/scan_lidar"),
     "resubscribed after reconnect (rosbridge keeps no state across a socket)");

  /* ------------------------------------------------------------------- */
  console.log("\n5. TOTAL silence: accepted, subscribed, nothing delivered ever");
  const iv = setInterval(() => { const s = latest(U); if (s && s.readyState === 1) { s.answerProbe(); } }, 25);
  await sleep(900);
  ok(L.snapshot().phase === "wedged",
     "phase is WEDGED — link provably alive, every expected topic silent");
  ok(L.snapshot().silent.length === 2, "both expected topics named as silent");
  ok(!L.snapshot().silent.some((s) => s.topic === "/fpms/residual/raw"),
     "the event-driven topic is NOT flagged (no false alarm while parked)");
  ok(logs.some((m) => /silence remedy 1\/2/.test(m)), "remedy 1 resubscribed the silent topics");
  await sleep(900);
  ok(logs.some((m) => /silence remedy 2\/2|STILL SILENT/.test(m)),
     "remedies escalated, then stopped and named the real fixes");
  ok(logs.some((m) => /SILENT while the link is up: .*scan_lidar/.test(m)),
     "the silent topics are named in the log, not merely counted");
  clearInterval(iv);

  console.log("\n6. silence clears the instant real data arrives");
  L.reconnectNow(); await sleep(20);
  latest(U).accept(); await sleep(20);
  latest(U).deliver({ op: "publish", topic: "/scan_lidar", msg: {} });
  latest(U).deliver({ op: "publish", topic: "/diagnostics", msg: {} });
  await sleep(20);
  ok(L.snapshot().phase === "live", "back to LIVE on the first topic message");
  ok(L.snapshot().silent.length === 0, "nothing reported silent once both deliver");

  console.log("\n6b. PARTIAL silence — the wrong-topic-root signature");
  /* /diagnostics keeps arriving (the Pi-side bridge is alive and its 0.5 s
     timer is firing) while the mirrored mission topic never does. That is
     exactly what a wrong thing name looks like from here, and it must NOT be
     reported as a dead link. */
  const keep = setInterval(() => {
    const s = latest(U);
    if (s && s.readyState === 1) { s.deliver({ op: "publish", topic: "/diagnostics", msg: {} }); }
  }, 60);
  await sleep(900);
  const snap = L.snapshot();
  ok(snap.phase !== "wedged",
     "partial silence is NOT reported as a wedged link (phase=" + snap.phase + ")");
  ok(snap.silent.length === 1 && snap.silent[0].topic === "/scan_lidar",
     "exactly the silent topic is named, and the working one is not");
  clearInterval(keep);
  await sleep(100);

  /* ------------------------------------------------------------------- */
  console.log("\n6c. the camera/NPU health mirror — restored on reconnect, never accused");
  /* app.js subscribes these WITHOUT expectS, deliberately: they are published
     by fpms-telemetry-ros, and a rover whose image predates that unit is
     silent on all of them while being perfectly healthy. Accusing it would be
     a false alarm on a working rover, and a false alarm is how an alarm
     becomes worth nothing on the day it is right. The tiles report the
     absence themselves, quietly. This proves both halves: no alarm, and no
     lost subscription across a reconnect. */
  const M = "ws://rover-mirror:9090";
  const LM = new RosLink(opts(M, "mirror"));
  const MIRROR = ["/fpms/camera/state", "/fpms/camera/health",
                  "/fpms/camera/frame_age_s", "/fpms/npu/state",
                  "/fpms/npu/health", "/fpms/npu/detection_available",
                  "/fpms/npu/fault", "/fpms/agent/fault"];
  MIRROR.forEach((t) => LM.subscribe(t, "std_msgs/String", () => {}));
  LM.subscribe("/scan_lidar", "sensor_msgs/LaserScan", () => {}, { expectS: 0.6 });
  await sleep(30);
  latest(M).accept();
  await sleep(30);
  const subbed = (s) => MIRROR.every((t) => s.sent.some((m) => m.op === "subscribe" && m.topic === t));
  ok(subbed(latest(M)), "all " + MIRROR.length + " mirror topics subscribed on open");
  /* Keep the LiDAR flowing so the link is provably live while every mirror
     topic stays silent — exactly a rover without fpms-telemetry-ros. */
  const quiet = setInterval(() => {
    const s = latest(M);
    if (s && s.readyState === 1) { s.answerProbe(); s.deliver({ op: "publish", topic: "/scan_lidar", msg: {} }); }
  }, 40);
  await sleep(900);
  ok(LM.snapshot().phase === "live",
     "the link is provably live with every /fpms/camera and /fpms/npu topic silent");
  ok(LM.snapshot().silent.length === 0,
     "and NOTHING is reported silent — a rover with no health mirror raises no alarm");
  clearInterval(quiet);
  LM.reconnectNow();
  await sleep(20);
  latest(M).accept();
  await sleep(30);
  ok(subbed(latest(M)),
     "every mirror topic is resubscribed on the new socket (rosbridge keeps no state)");
  LM.dispose();

  /* ------------------------------------------------------------------- */
  console.log("\n7. a socket hung in CONNECTING is abandoned, not waited on");
  const V = "ws://rover-hung:9090";
  const L7 = new RosLink(opts(V, "hung", { expectsData: false, connectTimeoutMs: 200 }));
  await sleep(700);
  ok(forUrl(V).length > 1,
     "hung socket abandoned and redialed (" + forUrl(V).length + " attempts)");
  ok(logs.some((m) => /connect timed out/.test(m)), "connect timeout logged");
  L7.dispose();

  /* ------------------------------------------------------------------- */
  console.log("\n8. backoff GROWS across a flapping open — the classic bug, absent");
  const W = "ws://rover-flap:9090";
  const L8 = new RosLink(opts(W, "flap", { expectsData: false, connectTimeoutMs: 5000, stableMs: 5000 }));
  const base8 = L8.backoffMs;
  for (let i = 0; i < 4; i++) {
    await sleep(30);
    const s = latest(W);
    if (!s) { continue; }
    s.accept();            // reaches onopen...
    await sleep(5);
    s.close();             // ...and drops straight away
    await sleep(220);
  }
  ok(L8.backoffMs > base8,
     "backoff kept growing (" + base8 + " -> " + L8.backoffMs + " ms), so a flapping server is not hammered");
  ok(forUrl(W).length >= 4, "and it never gave up (" + forUrl(W).length + " attempts)");
  L8.dispose();

  /* ------------------------------------------------------------------- */
  console.log("\n9. backoff DOES reset once the link has been healthy for stableMs");
  const X = "ws://rover-stable:9090";
  const L9 = new RosLink(opts(X, "stable", { expectsData: false, connectTimeoutMs: 5000, stableMs: 250 }));
  await sleep(20); latest(X).accept();
  await sleep(20); latest(X).close();
  await sleep(220);
  const grown = L9.backoffMs;
  latest(X).accept();
  await sleep(500);
  ok(grown > FAST.backoffBaseMs && L9.backoffMs === FAST.backoffBaseMs,
     "backoff reset only after sustained health (" + grown + " -> " + L9.backoffMs + " ms)");
  L9.dispose();

  /* ------------------------------------------------------------------- */
  console.log("\n10. publish is honest, and dispose leaves no zombie");
  const Y = "ws://rover-gone:9090";
  const L10 = new RosLink(opts(Y, "gone", { expectsData: false, connectTimeoutMs: 60000 }));
  ok(L10.publish("/estop", { data: true }) === false,
     "publish returns false rather than pretending on a socket that is not open");
  L10.dispose();
  const n10 = forUrl(Y).length;
  await sleep(500);
  ok(forUrl(Y).length === n10,
     "a disposed link stops dialing (no zombie after re-pointing at another rover)");

  L.dispose();

  console.log("\n" + pass + " passed, " + fail + " failed");
  process.exit(fail ? 1 : 0);
})();

/* =========================================================================
   app.js — subscriptions, STOP, panels, and the one renderer.
   ========================================================================= */
(function (global) {
"use strict";

var F = global.Feeds;

/* ======================================================================
   0. IDENTITY — WHICH ROVER, AND WHICH TOPIC ROOT
   ----------------------------------------------------------------------
   THIS IMAGE IS ROVER 1. Changed from rover2 on 2026-08-13. The defaults
   below are that identity:

     host   fpms-rover1.local   (hostname fpms-rover1, from fpms-os.conf)
     port   9090                (fpms-rosbridge.service)
     thing  rover1              (FPMS_THING_NAME in /etc/fpms/config.env)

   WHY THE HOST HAS AN IP OVERRIDE, AND WHY IT IS NOT OPTIONAL
   mDNS resolution is PER-RESOLVER, NOT PER-MACHINE. The image's own
   FLASHING.md records a session where `fpms-rover1.local` resolved from
   .NET and failed from Python's getaddrinfo on the same box at the same
   time. "ping works" therefore does not prove the browser will resolve it.
   The picker takes a raw IP for exactly that case.

   WHY THE THING NAME IS HERE AT ALL, GIVEN THIS IS A ROS DASHBOARD
   Be precise about this, because getting it wrong wastes a session. The ROS
   topic names this page subscribes to contain NO thing name — /scan_lidar
   and /fpms/mission/x_mm are the same strings on every rover. The thing name
   is the MQTT topic root, and it matters here for two real reasons:

     1. A two-rover fleet is coming, and the operator must be able to see at
        a glance WHICH rover the numbers on screen belong to. It is in the
        header, always, never behind a settings panel.
     2. It is CHECKABLE. Every DiagnosticStatus the Pi-side bridge publishes
        carries hardware_id = FPMS_THING_NAME, so /diagnostics is live
        evidence of the rover's own idea of its identity. If it disagrees
        with the dashboard's, that is shown loudly — see checkIdentity().

   And the failure that makes all of this worth doing, in config.env's own
   words: a consumer on the wrong root "connects, authenticates, stays
   connected and receives nothing forever. The rover looks dead; the broker,
   the bridge and every unit look healthy."

   Resolution order for the target, most explicit first:
     1. ?rover=host[:port]&thing=name    one visit, overrides everything
     2. localStorage                     what the operator last chose
     3. config.json                      what serve.py was started with
     4. the built-in rover1 defaults     dialed, and shown, and checkable
   ====================================================================== */
var DEFAULT_HOST  = "fpms-rover1.local";
var DEFAULT_PORT  = 9090;
var DEFAULT_THING = "rover1";

var LS_KEY = "fpms.dashboard.rover";
var target = { host: DEFAULT_HOST, port: DEFAULT_PORT, thing: DEFAULT_THING };

function parseTarget(s, thing) {
  if (!s) { return null; }
  s = String(s).trim().replace(/^wss?:\/\//, "").replace(/\/+$/, "");
  if (!s) { return null; }
  var m = /^(\[[^\]]+\]|[^:]+)(?::(\d+))?$/.exec(s);
  if (!m) { return null; }
  return {
    host: m[1],
    port: m[2] ? parseInt(m[2], 10) : DEFAULT_PORT,
    thing: (thing || DEFAULT_THING).trim() || DEFAULT_THING
  };
}

function wsUrl() { return "ws://" + target.host + ":" + target.port; }

/* ======================================================================
   1. THE LINKS
   ----------------------------------------------------------------------
   TWO sockets, and the second one is not redundancy — it is the whole STOP
   guarantee.

   A browser WebSocket send() queues behind whatever is already in that
   socket's send buffer, and the main socket carries a 10 Hz LaserScan plus
   an event stream measured at ~330 Hz during a plan. A STOP published on
   that socket can sit behind a scan frame. A panic button that can be busy
   is not a panic button.

   The STOP socket subscribes to NOTHING, so its send buffer is empty by
   construction, and it is created FIRST so that it is the connection that
   wins the race to the server on a cold start.
   ====================================================================== */
var stopLink = null, mainLink = null;

function log(line) {
  var el = document.getElementById("events");
  if (!el) { return; }
  var d = document.createElement("div");
  d.textContent = ts() + " " + line;
  if (/wedge|LOST|DOWN|error|FAIL|refus|nack|abort/i.test(line)) { d.className = "bad"; }
  else if (/STOP|warn|stale|timeout|retry/i.test(line)) { d.className = "warn"; }
  el.appendChild(d);
  while (el.childNodes.length > 400) { el.removeChild(el.firstChild); }
  el.scrollTop = el.scrollHeight;
}

function ts() {
  var d = new Date();
  return ("0" + d.getHours()).slice(-2) + ":" + ("0" + d.getMinutes()).slice(-2) +
         ":" + ("0" + d.getSeconds()).slice(-2);
}

function connect() {
  /* Retire the previous pair BEFORE clearing state. A link that is merely
     dropped keeps its supervisor timer and goes on dialing the old rover
     forever, invisibly. */
  if (stopLink) { stopLink.dispose(); }
  if (mainLink) { mainLink.dispose(); }
  /* Every feed is cleared on a target change. Carrying rover2's last pose
     into rover1's map is precisely the confident-value-in-the-wrong-frame
     failure this dashboard exists to prevent. */
  F.reset();
  arena.clearTrail();
  resHistory = [];
  route = [];
  scan = null;

  stopLink = new global.RosLink({
    url: wsUrl(), label: "stop", expectsData: false, onlog: log
  });
  /* Both verbs. /estop is honoured by the firmware-v3 control task and by
     fpms-cored; /fpms/cmd/stop is fanned out by the Pi-side bridge to the
     MQTT stop AND estop verbs at QoS 1. Both are on the rosbridge whitelist
     (/estop by name, /fpms/cmd/stop via /fpms/*). Sending both costs one
     extra 60-byte publish and removes a whole class of "which consumer
     listens to which verb" doubt at the worst possible moment. */
  stopLink.advertise("/estop", "std_msgs/Bool");
  stopLink.advertise("/fpms/cmd/stop", "std_msgs/Empty");

  mainLink = new global.RosLink({ url: wsUrl(), label: "main", onlog: log });
  subscribeAll(mainLink);

  document.getElementById("roverHost").textContent = target.host + ":" + target.port;
  document.getElementById("thingChip").textContent = target.thing;
  log("target " + target.host + ":" + target.port + "  thing " + target.thing);
}

/* ======================================================================
   2. SUBSCRIPTIONS
   ----------------------------------------------------------------------
   EVERY topic below is inside rosbridge's topics_glob:

     "[/estop,/fpms/*,/clicked_point,/scan_lidar,/odom_raw,/odom,/imu,
       /battery,/wheel_ticks,/wheel_duty,/fpms_health,/diagnostics,/rosout]"

   Note what is NOT here and cannot be: /cmd_vel, /cmd_duty, /cmd_enable.
   They are absent from that whitelist by construction, so the server refuses
   both advertise and publish for them. This file asking politely is the
   second layer, not the only one.

   This dashboard calls NO ROS SERVICE, including no /rosapi call. Its
   heartbeat is an unknown-op echo (rosbridge.js) precisely so that
   services_glob — "[/fpms/*]" — cannot silently swallow the thing that
   proves the link is alive.

   THROTTLES ARE NOT COSMETIC. rosbridge serialises every subscription onto
   ONE protocol thread and ONE socket. /fpms/events was measured bursting to
   ~330 Hz during a plan; unthrottled it does not merely fill a log, it
   starves /scan_lidar on the same socket, and a frozen map beside a
   scrolling log is the exact failure this dashboard exists to prevent.

   `expectS` IS THE SILENCE ALARM, AND IT IS DELIBERATELY NARROW.
   Only FOUR topics declare one, because only four have publishers that run
   unconditionally whatever the rover is doing:

     /scan_lidar          the LiDAR node runs whether or not a mission does
     /diagnostics         fpms_foxglove_cmd's own 0.5 s timer. The strongest
                          canary on the graph: it is published by the SAME
                          node that mirrors every /fpms/* topic, so if it is
                          arriving and the mirrors are not, the bridge is
                          alive and its MQTT side is empty — which is the
                          wrong-topic-root signature exactly.
     /fpms/mission/state  `_mirror_mission` publishes it on EVERY inbound
                          telemetry/mission with no condition attached.
     /fpms/mission/phase  likewise, and it defaults to "idle" rather than
                          being omitted, so it is present when parked too.

   Everything else is left unflagged ON PURPOSE:
     * x_mm / y_mm / heading_deg are omitted by `_f32` when the executor has
       no pose to report, which is the normal parked state;
     * /fpms_health and /wheel_ticks are FIRMWARE V3 ONLY and legitimately
       absent on the factory firmware this rover runs today;
     * /battery, residuals, the plan and events are event-driven.
   A silence alarm that fires while the rover is parked and healthy is an
   alarm the operator learns to ignore, and then it is worth nothing on the
   day it is right.
   ====================================================================== */
var scan = null;         // LaserScan or null. NEVER a stale frame.
var route = [];
var resHistory = [];     // newest first, from /fpms/residual/raw
var diagStatus = {};     // name -> {level, message, age_s, hz} from /diagnostics
var healthWords = null;  // /fpms_health Int32MultiArray data
var reportedThing = null; // hardware_id off /diagnostics: the ROVER's own identity
var zonesCheck = null;   // result of ARENA.validateZones(), or null if no file

function f32(key, fx) {
  return function (m) {
    if (m && typeof m.data === "number") {
      F.mark(key, m.data.toFixed(fx === undefined ? 1 : fx), m.data);
    }
  };
}

function subscribeAll(L) {
  /* ---- sensors ------------------------------------------------------ */
  L.subscribe("/scan_lidar", "sensor_msgs/LaserScan", function (m) {
    scan = m; F.mark("scan", (m.ranges || []).length + " rays", m);
  }, { throttle: 100, expectS: 8 });

  /* No type asserted: BatteryState on firmware v3, UInt16 decivolts on the
     Yahboom stock image. Guessing wrong shows a plausible number in the
     wrong units, which is worse than showing none. */
  L.subscribe("/battery", null, function (m) {
    var v = null;
    if (m && typeof m.voltage === "number") { v = m.voltage; }
    else if (m && typeof m.data === "number") { v = m.data / 10.0; }
    if (v !== null) { F.mark("batt", v.toFixed(2) + " V", v); }
  }, { throttle: 500 });

  L.subscribe("/wheel_ticks", "std_msgs/Int32MultiArray", function (m) {
    F.mark("ticks", (m.data || []).join(" "), m.data || []);
  }, { throttle: 250 });

  /* 12 int32 fields; layout is the table in firmware_v3/fpms_main.cpp and
     MUST be kept in sync with it. Decoded in decodeHealth() below. */
  L.subscribe("/fpms_health", "std_msgs/Int32MultiArray", function (m) {
    healthWords = m.data || [];
    F.mark("health", healthWords.length + " fields", healthWords);
  }, { throttle: 500 });

  /* The Pi's own per-topic freshness verdicts. We render our OWN ages from
     our OWN arrivals — this is a second opinion from the other end of the
     link, and where the two disagree, the link is the suspect. */
  L.subscribe("/diagnostics", "diagnostic_msgs/DiagnosticArray", function (m) {
    var out = {};
    (m.status || []).forEach(function (s) {
      var kv = {};
      (s.values || []).forEach(function (p) { kv[p.key] = p.value; });
      out[s.name] = { level: s.level, message: s.message, kv: kv };
    });
    diagStatus = out;
    /* hardware_id is FPMS_THING_NAME on every status the bridge publishes —
       the rover's own statement of which rover it is. */
    (m.status || []).forEach(function (s) {
      if (s.hardware_id) { reportedThing = s.hardware_id; }
    });
    F.mark("diag", Object.keys(out).length + " statuses", out);
  }, { throttle: 1000, expectS: 8 });

  /* ---- mission mirror — THE POSE. Not /odom. See arena.js. ----------- */
  L.subscribe("/fpms/mission/x_mm", "std_msgs/Float32", f32("x"));
  L.subscribe("/fpms/mission/y_mm", "std_msgs/Float32", f32("y"));
  L.subscribe("/fpms/mission/heading_deg", "std_msgs/Float32", f32("hdg"));

  L.subscribe("/fpms/mission/front_mm", "std_msgs/Float32", f32("front", 0));
  L.subscribe("/fpms/mission/distance_remaining_mm", "std_msgs/Float32", f32("drem", 0));
  L.subscribe("/fpms/mission/distance_travelled_mm", "std_msgs/Float32", f32("dtrav", 0));
  L.subscribe("/fpms/mission/state", "std_msgs/String", function (m) {
    /* This is the ENTIRE MQTT telemetry/mission dict as JSON, not a word. */
    var v = m.data, j = null;
    try { j = JSON.parse(m.data); } catch (e) {}
    F.mark("state", j && j.state ? String(j.state) : String(v).slice(0, 40), j || v);
  }, { expectS: 25 });
  L.subscribe("/fpms/mission/phase", "std_msgs/String", function (m) {
    F.mark("phase", m.data, m.data);
  }, { expectS: 25 });
  L.subscribe("/fpms/mission/armed", "std_msgs/Bool", function (m) {
    F.mark("armed", m.data ? "ARMED" : "disarmed", !!m.data);
  });
  L.subscribe("/fpms/mission/link_ok", "std_msgs/Bool", function (m) {
    F.mark("linkok", m.data ? "ok" : "DOWN", !!m.data);
  });
  L.subscribe("/fpms/mission/lidar_ok", "std_msgs/Bool", function (m) {
    F.mark("lidarok", m.data ? "ok" : "NOT OK", !!m.data);
  });
  L.subscribe("/fpms/mission/leg_i", "std_msgs/Int32", function (m) {
    F.mark("leg", String(m.data), m.data);
  });
  L.subscribe("/fpms/mission/segment_i", "std_msgs/Int32", function (m) {
    F.mark("seg", String(m.data), m.data);
  });
  /* Fallback battery. Used when /battery has not arrived OR HAS GONE STALE —
     not merely when it has never arrived, because "we saw a voltage once, an
     hour ago" is exactly the frozen-but-plausible reading this dashboard
     exists to stop showing. */
  L.subscribe("/fpms/mission/batt_v", "std_msgs/Float32", function (m) {
    F.mark("battm", m.data.toFixed(2) + " V", m.data);
  });

  /* ---- residuals: the most diagnostic thing this rover produces ------ */
  L.subscribe("/fpms/residual/drive_mm", "std_msgs/Float32", f32("res_mm"));
  L.subscribe("/fpms/residual/turn_deg", "std_msgs/Float32", f32("res_deg"));
  L.subscribe("/fpms/residual/lateral_mm", "std_msgs/Float32", f32("res_lat"));
  L.subscribe("/fpms/residual/cumulative_drive_mm", "std_msgs/Float32", f32("res_cum_mm"));
  L.subscribe("/fpms/residual/cumulative_turn_deg", "std_msgs/Float32", f32("res_cum_deg"));
  /* The raw payload carries target/measured/ratio/kind, which the scalar
     topics do not. Residual alone tells you the error; target+measured tell
     you WHICH of the three failure patterns produced it. */
  L.subscribe("/fpms/residual/raw", "std_msgs/String", function (m) {
    var j; try { j = JSON.parse(m.data); } catch (e) { return; }
    j._rx = performance.now();
    resHistory.unshift(j);
    if (resHistory.length > 40) { resHistory.pop(); }
    F.mark("res_raw", (j.kind || "?") + " " + (j.residual), j);
  });

  /* ---- plan + events ------------------------------------------------- */
  L.subscribe("/fpms/plan/route", "std_msgs/String", function (m) {
    var j; try { j = JSON.parse(m.data); } catch (e) { return; }
    route = (j.waypoints || []).map(function (w) {
      return { x: w.x_mm, y: w.y_mm, wp: w.waypoint };
    });
    F.mark("plan", route.length + " waypoints", j);
  });
  L.subscribe("/fpms/events", "std_msgs/String", function (m) {
    F.mark("events", "1", m.data);
    var txt = m.data;
    try { txt = JSON.stringify(JSON.parse(m.data)); } catch (e) {}
    log(txt.slice(0, 300));
  }, { throttle: 100 });
}

/* ======================================================================
   3. STOP
   ----------------------------------------------------------------------
   A topic publish, never a service call: no request id, no pending state,
   no server needed to answer, no timeout to expire. It goes out on the
   dedicated socket; if that socket is down it falls back to the main one
   AND SAYS WHICH PATH CARRIED IT. It never silently does nothing.
   ====================================================================== */
var stopCount = 0;

function fireStop(why) {
  stopCount++;
  var a = stopLink.publish("/estop", { data: true });
  var b = stopLink.publish("/fpms/cmd/stop", {});
  var ok = a && b, path = "stop link";

  if (!ok) {
    var c = mainLink.publish("/estop", { data: true });
    var d = mainLink.publish("/fpms/cmd/stop", {});
    ok = ok || c || d;
    path = "MAIN link (stop link was down)";
  }

  var btn = document.getElementById("stopBtn");
  btn.classList.add("fired");
  setTimeout(function () { btn.classList.remove("fired"); }, 900);

  log("STOP #" + stopCount + " (" + why + ") via " + path +
      (ok ? "" : "  — PUBLISH FAILED ON BOTH SOCKETS. GET TO THE ROVER."));

  if (!ok) {
    /* Both sockets down is the one case where the UI must not look normal. */
    document.getElementById("chipStop").className = "chip down";
    document.getElementById("chipStop").textContent = "STOP NOT SENT";
    /* And retry the instant a socket comes back, once. */
    stopLink.reconnectNow();
  }
  return ok;
}

/* ======================================================================
   4. LINK HEALTH TILES
   ----------------------------------------------------------------------
   FOUR subsystems were asked for. Two of them have a ROS source and two do
   not, and this panel says which is which rather than inventing a colour.

     LIDAR  — real. /scan_lidar arrivals here + /fpms/mission/lidar_ok +
              the Pi's own ros/scan_lidar diagnostic.
     ESP32  — real. /fpms_health flag bit4 is the micro-ROS agent session,
              which IS the board link, corroborated by /wheel_ticks arriving.
     CAMERA — NO ROS PUBLISHER EXISTS. The rover agent sends frames straight
              to MQTT; nothing mirrors them onto the graph. The best evidence
              on the graph is that the agent's own MQTT heartbeat is fresh,
              which says the camera-owning PROCESS is alive and says nothing
              about the camera. The tile reports exactly that and no more.
     NPU    — NO ROS PUBLISHER EXISTS AT ALL. FPMS_NPU_REQUIRED=1 makes a
              blind rover a declared fault, but that fault is published on
              MQTT, not on any topic inside topics_glob. Widening the
              whitelist to reach it is not on the table, so the tile is
              permanently "no ROS source" with the reason attached.

   Guessing green here would be the worst possible failure of this dashboard:
   a blind fire-detection rover reported healthy.
   ====================================================================== */
function decodeHealth() {
  if (!healthWords || healthWords.length < 12 || F.isStale("health")) { return null; }
  var w = healthWords, flags = w[1];
  return {
    fw: w[0],
    estopLatched: !!(flags & 1),
    velArmed:     !!(flags & 2),
    deadman:      !!(flags & 4),
    imuOk:        !!(flags & 8),
    agentUp:      !!(flags & 16),
    path:         w[2],
    loopHz:       w[6] / 10.0,
    movedMask:    w[7],
    battMv:       w[8],
    heapKb:       w[10],
    uptimeS:      w[11]
  };
}

function linkTiles() {
  var h = decodeHealth();
  var tiles = [];

  /* ---- LiDAR ---- */
  (function () {
    var lv = F.level("scan"), st;
    if (lv === "never")      { st = ["down", "NEVER SEEN"]; }
    else if (lv === "stale") { st = ["down", "STALE " + F.ageText("scan")]; }
    else if (lv === "warn")  { st = ["warn", "SLOW " + F.hz("scan").toFixed(1) + " Hz"]; }
    else                     { st = ["ok", F.hz("scan").toFixed(1) + " Hz"]; }
    var extra = [];
    if (F.seen("lidarok") && !F.isStale("lidarok")) {
      extra.push("executor says " + (F.fresh("lidarok") ? "lidar_ok" : "LIDAR NOT OK"));
      if (F.fresh("lidarok") === false && st[0] === "ok") { st = ["warn", st[1]]; }
    }
    var dg = diagStatus["ros/scan_lidar"];
    if (dg && !F.isStale("diag")) { extra.push("Pi diag: " + dg.message); }
    tiles.push({ name: "LIDAR", cls: st[0], val: st[1], age: F.ageText("scan"),
                 src: "/scan_lidar", note: extra.join(" · ") });
  })();

  /* ---- ESP32 / drive board ---- */
  (function () {
    var st, note = [];
    if (!F.seen("health"))      { st = ["down", "NEVER SEEN"]; }
    else if (F.isStale("health")) { st = ["down", "STALE " + F.ageText("health")]; }
    else if (h && !h.agentUp)   { st = ["down", "AGENT NOT CONNECTED"]; }
    else if (h)                 { st = ["ok", "fw " + h.fw + " · loop " + h.loopHz.toFixed(0) + " Hz"]; }
    else                        { st = ["warn", "health message too short"]; }
    if (h) {
      note.push("uptime " + h.uptimeS + "s");
      note.push("heap " + h.heapKb + " kB");
      if (h.estopLatched) { note.push("ESTOP LATCHED"); }
      if (!h.imuOk)       { note.push("IMU init FAILED"); }
      note.push("wheels moved mask 0x" + (h.movedMask || 0).toString(16));
    }
    if (F.seen("linkok") && !F.isStale("linkok")) {
      note.push("executor link_ok=" + F.fresh("linkok"));
      if (F.fresh("linkok") === false && st[0] === "ok") { st = ["warn", st[1]]; }
    }
    if (F.seen("ticks")) {
      note.push("/wheel_ticks " + (F.isStale("ticks") ? "STALE" : F.hz("ticks").toFixed(1) + " Hz"));
    }
    tiles.push({ name: "ESP32", cls: st[0], val: st[1], age: F.ageText("health"),
                 src: "/fpms_health bit4 (micro-ROS session)", note: note.join(" · ") });
  })();

  /* ---- Camera ---- */
  (function () {
    var dg = diagStatus["mqtt/telemetry/pose"];
    var val = "NO ROS SOURCE", cls = "nosrc", note =
      "Nothing publishes camera frames or camera health onto the ROS graph; " +
      "the rover agent sends them straight to MQTT. Porting this needs a " +
      "rover-side publisher, not a dashboard change.";
    if (dg && !F.isStale("diag")) {
      note = "PROCESS only: the rover-agent heartbeat that owns the camera is " +
             dg.message + ". That is evidence the process is alive, and no " +
             "evidence at all about the camera. " + note;
    }
    tiles.push({ name: "CAMERA", cls: cls, val: val,
                 age: F.seen("diag") ? "via /diagnostics " + F.ageText("diag") : "never seen",
                 src: "none on the whitelist", note: note });
  })();

  /* ---- NPU ---- */
  tiles.push({
    name: "NPU", cls: "nosrc", val: "NO ROS SOURCE", age: "n/a",
    src: "none on the whitelist",
    note: "FPMS_NPU_REQUIRED=1 makes a blind rover a declared fault, but that " +
          "fault is published on MQTT and has no mirror inside rosbridge's " +
          "topics_glob. This dashboard will not colour a tile it cannot " +
          "justify — check the rover agent's log or the MQTT dashboard."
  });

  return tiles;
}

function renderLinks() {
  var host = document.getElementById("links");
  var tiles = linkTiles();
  var html = tiles.map(function (t) {
    return '<div class="ltile ' + t.cls + '">' +
      '<div class="lrow"><span class="lname">' + t.name + '</span>' +
      '<span class="lval">' + esc(t.val) + '</span></div>' +
      '<div class="age">' + esc(t.age) + ' &middot; <span class="src">' + esc(t.src) + '</span></div>' +
      (t.note ? '<div class="tiny">' + esc(t.note) + '</div>' : '') +
      '</div>';
  }).join("");
  if (host.innerHTML !== html) { host.innerHTML = html; }
}

function esc(s) {
  return String(s === undefined || s === null ? "" : s)
    .replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

/* ======================================================================
   5. RESIDUAL VERDICT
   ----------------------------------------------------------------------
   STACK.md's table, applied to whatever segments have actually arrived:

     constant RATIO   -> scale error (counts/mm).  2.467 = 74/30 is the known
                         candidate on this rover.
     constant OFFSET  -> coast after the burst is cut.
     grows on TURNS   -> gyro / heading.

   It refuses to guess from fewer than three drive segments, and it labels
   its own confidence, because a wrong calibration verdict costs a session.
   ====================================================================== */
function residualVerdict() {
  var drives = resHistory.filter(function (r) { return r.kind !== "turn" && typeof r.target === "number" && Math.abs(r.target) > 20; });
  var turns  = resHistory.filter(function (r) { return r.kind === "turn"; });

  if (drives.length < 3) {
    return { cls: "dim", text: "Needs at least 3 drive segments before it will guess at a pattern. " +
             "Have " + drives.length + " drive, " + turns.length + " turn." };
  }
  var ratios = drives.map(function (r) { return r.measured / r.target; });
  var offs   = drives.map(function (r) { return r.residual; });

  var spread = function (a) {
    var mu = a.reduce(function (s, v) { return s + v; }, 0) / a.length;
    var sd = Math.sqrt(a.reduce(function (s, v) { return s + (v - mu) * (v - mu); }, 0) / a.length);
    return { mu: mu, sd: sd };
  };
  var R = spread(ratios), O = spread(offs);

  /* Which hypothesis explains the data with less scatter? Normalise both to
     the same yardstick: relative spread. */
  var ratioTight  = R.sd < 0.06;
  var offsetTight = O.sd < Math.max(8, Math.abs(O.mu) * 0.25);

  if (ratioTight && Math.abs(R.mu - 1) > 0.05) {
    var hint = (Math.abs(R.mu - 2.467) < 0.15)
      ? "  That is 2.467 = 74/30, the KNOWN scale candidate on this rover."
      : "";
    return { cls: "bad", text: "CONSTANT RATIO " + R.mu.toFixed(3) + " (sd " + R.sd.toFixed(3) +
      ") over " + drives.length + " segments -> SCALE ERROR in counts/mm. Set odom_scale in " +
      "/etc/fpms/calibration.json." + hint };
  }
  if (offsetTight && Math.abs(O.mu) > 8) {
    return { cls: "warn", text: "CONSTANT OFFSET " + O.mu.toFixed(1) + " mm (sd " + O.sd.toFixed(1) +
      ") regardless of segment length over " + drives.length + " segments -> COAST after the burst " +
      "is cut. Set coast_mm." };
  }
  if (turns.length >= 3) {
    var tAbs = turns.map(function (r) { return Math.abs(r.residual); });
    var T = spread(tAbs);
    if (T.mu > 3 && Math.abs(O.mu) < 8) {
      return { cls: "warn", text: "Drive residuals are small but TURN residuals average " +
        T.mu.toFixed(1) + " deg over " + turns.length + " turns -> GYRO / HEADING. Set gyro_scale." };
    }
  }
  return { cls: "ok", text: "No single pattern dominates: ratio " + R.mu.toFixed(3) +
    " (sd " + R.sd.toFixed(3) + "), offset " + O.mu.toFixed(1) + " mm (sd " + O.sd.toFixed(1) +
    ") over " + drives.length + " drive segments. Residuals look unstructured." };
}

/* ======================================================================
   6. THE TICKER — the ONLY renderer, at 5 Hz.
   ----------------------------------------------------------------------
   Every visible value is re-derived here from the feed registry, so no
   number can survive on screen without its age being re-checked. Message
   handlers call Feeds.mark() and touch no DOM.
   ====================================================================== */
var arena = null;

function setVal(valId, ageId, key, fallback) {
  var f = F.get(key);
  var v = document.getElementById(valId);
  var a = document.getElementById(ageId);
  if (v) { v.textContent = f ? f.v : (fallback || "—"); }
  if (a) { a.textContent = F.ageText(key); }
}

function applyLevels() {
  /* One pass: any element with data-feed gets the feed's level as a class,
     so staleness styling is impossible to forget on a new panel. */
  var els = document.querySelectorAll("[data-feed]");
  for (var i = 0; i < els.length; i++) {
    var key = els[i].getAttribute("data-feed");
    var lv = F.level(key);
    /* The pose card is judged by its worst component: a fresh x with a stale
       heading is not a fresh pose. */
    if (key === "pose") {
      lv = worst(["x", "y", "hdg"]);
    }
    els[i].setAttribute("data-level", lv);
    els[i].classList.toggle("evented", !!F.spec(key).event);
  }
}

function worst(keys) {
  var order = { ok: 0, warn: 1, stale: 2, never: 3 }, best = "ok";
  keys.forEach(function (k) {
    if (order[F.level(k)] > order[best]) { best = F.level(k); }
  });
  return best;
}

function chip(id, snap) {
  var el = document.getElementById(id);
  var cls = "chip", txt;
  var P = global.RosLink.PHASE;
  switch (snap.phase) {
    case P.LIVE:
      cls += " up";
      txt = snap.label.toUpperCase() + " LIVE · " + snap.upS.toFixed(0) + "s";
      break;
    case P.OPEN:
      cls += (snap.label === "stop") ? " up" : " warn";
      txt = snap.label.toUpperCase() + (snap.label === "stop" ? " UP" : " OPEN, no data yet");
      break;
    case P.WEDGED:
      cls += " down";
      txt = snap.label.toUpperCase() + " WEDGED";
      break;
    case P.CONNECTING:
      cls += " warn";
      txt = snap.label.toUpperCase() + " CONNECTING…";
      break;
    case P.BACKOFF:
      cls += " down";
      txt = snap.label.toUpperCase() + " DOWN · retry " +
            (snap.retryInS === null ? "?" : snap.retryInS.toFixed(1)) + "s";
      break;
    case P.OFFLINE:
      cls += " down";
      txt = "BROWSER OFFLINE · retry " +
            (snap.retryInS === null ? "?" : snap.retryInS.toFixed(1)) + "s";
      break;
    default:
      cls += " warn";
      txt = snap.label.toUpperCase() + " …";
  }
  if (snap.probing) { txt += " ·probe"; }
  el.className = cls;
  el.textContent = txt;
}

function tick() {
  if (!mainLink) { return; }
  var P = global.RosLink.PHASE;
  var ms = mainLink.snapshot(), ss = stopLink.snapshot();

  chip("chipMain", ms);
  chip("chipStop", ss);

  /* Wedge banner. */
  var wedged = (ms.phase === P.WEDGED);
  document.getElementById("wedgeBanner").classList.toggle("hidden", !wedged);
  if (wedged) {
    document.getElementById("wedgeDetail").textContent =
      " — socket up " + ms.upS.toFixed(0) + "s, " + ms.counters.frames +
      " frames, " + ms.counters.data + " topic messages. Automatic remedy " +
      ms.wedgeRemedy + "/2 attempted.";
  }

  /* --- staleness classes first, so every value below is styled by them -- */
  applyLevels();

  /* --- LiDAR: the scan is DROPPED the moment it is stale --------------- */
  if (F.isStale("scan")) { scan = null; }

  setVal("scanVal", "scanAge", "scan");
  if (F.isStale("scan")) {
    document.getElementById("scanVal").textContent =
      F.seen("scan") ? "STALE — scan dropped" : "no scan";
  } else {
    document.getElementById("scanVal").textContent =
      (F.get("scan") ? F.get("scan").v : "—") + " @ " + F.hz("scan").toFixed(1) + " Hz";
  }

  /* --- pose: ONLY from fresh mission mirror values --------------------- */
  var px = F.fresh("x"), py = F.fresh("y"), ph = F.fresh("hdg");
  var pose = null, poseStale = false;
  if (px !== null && py !== null && ph !== null) {
    pose = { x: px, y: py, hdg: ph };
  } else if (F.seen("x") && F.seen("y") && F.seen("hdg")) {
    pose = { x: F.get("x").raw, y: F.get("y").raw, hdg: F.get("hdg").raw };
    poseStale = true;
  }
  document.getElementById("poseVal").textContent = pose
    ? (Math.round(pose.x) + ", " + Math.round(pose.y) + " mm  hdg " + pose.hdg.toFixed(1) + "°" +
       (poseStale ? "  (STALE)" : ""))
    : "no live pose";
  document.getElementById("poseAge").textContent =
    F.seen("x") ? ("x " + F.ageText("x") + " · y " + F.ageText("y") + " · hdg " + F.ageText("hdg"))
                : "never seen — parked, the mission mirror publishes no pose";

  var frontMm = F.fresh("front");
  setVal("frontVal", "frontAge", "front");
  if (frontMm !== null) {
    document.getElementById("frontVal").textContent = frontMm.toFixed(0) + " mm";
  }

  /* --- the map --------------------------------------------------------- */
  arena.drawDynamic({
    pose: pose, poseStale: poseStale,
    scan: scan, frontMm: frontMm, route: route
  });

  /* --- mission --------------------------------------------------------- */
  setVal("stateVal", "stateAge", "state", "no mission telemetry");
  setVal("phaseVal", "phaseAge", "phase");
  setVal("armedVal", "armedAge", "armed");
  setVal("dtravVal", "dtravAge", "dtrav");
  setVal("dremVal",  "dremAge",  "drem");
  document.getElementById("legVal").textContent =
    (F.seen("leg") ? F.get("leg").v : "—") + " / " + (F.seen("seg") ? F.get("seg").v : "—");
  document.getElementById("legAge").textContent = F.ageText("leg");
  setVal("planVal", "planAge", "plan", "no route");

  /* --- battery. /battery first; the mission mirror only while it is
         absent or stale, and the panel says which one it is showing. ---- */
  var v = F.fresh("batt"), src = "/battery";
  if (v === null) { v = F.fresh("battm"); src = "/fpms/mission/batt_v (fallback)"; }
  var battEl = document.getElementById("battVal");
  var battCard = battEl.parentNode;
  if (v === null) {
    battEl.textContent = "no voltage";
    battCard.setAttribute("data-feed", "batt");
    document.getElementById("battAge").textContent =
      "/battery " + F.ageText("batt") + " · mirror " + F.ageText("battm");
    document.getElementById("battBar").style.width = "0%";
  } else {
    battEl.textContent = v.toFixed(2) + " V";
    battCard.setAttribute("data-feed", src === "/battery" ? "batt" : "battm");
    document.getElementById("battAge").textContent =
      src + " · " + F.ageText(src === "/battery" ? "batt" : "battm");
    /* 3S li-ion: 9.0 V empty, 12.6 V full. A bar, never a percentage — a
       percentage implies a state-of-charge model nobody has calibrated. */
    var frac = Math.max(0, Math.min(1, (v - 9.0) / (12.6 - 9.0)));
    var bar = document.getElementById("battBar");
    bar.style.width = (frac * 100).toFixed(0) + "%";
    bar.className = frac < 0.15 ? "bad" : (frac < 0.35 ? "warn" : "ok");
  }

  /* --- link health ----------------------------------------------------- */
  renderLinks();

  /* --- residuals ------------------------------------------------------- */
  setVal("resMmVal",     "resMmAge",     "res_mm");
  setVal("resDegVal",    "resDegAge",    "res_deg");
  setVal("resCumMmVal",  "resCumMmAge",  "res_cum_mm");
  setVal("resCumDegVal", "resCumDegAge", "res_cum_deg");
  renderResiduals();

  /* --- link diagnostics ------------------------------------------------ */
  document.getElementById("linkDiag").textContent =
    "main   " + ms.phase + "  up " + ms.upS.toFixed(0) + "s  attempts " + ms.attempts +
      "  frames " + ms.counters.frames + "  topicmsgs " + ms.counters.data +
      "  opens " + ms.counters.opens + "  closes " + ms.counters.closes +
      "  probes " + ms.counters.probes + " (" + ms.counters.probeFails + " failed)" +
      "  wedges " + ms.counters.wedges + "\n" +
    "stop   " + ss.phase + "  up " + ss.upS.toFixed(0) + "s  attempts " + ss.attempts +
      "  opens " + ss.counters.opens + "  closes " + ss.counters.closes +
      "  probes " + ss.counters.probes + " (" + ss.counters.probeFails + " failed)" + "\n" +
    "url    " + ms.url + "\n" +
    "ages are measured in THIS browser from the moment each frame arrived; " +
    "no payload timestamp is used for freshness.";
}

var lastResHtml = "";
function renderResiduals() {
  var v = residualVerdict();
  var ve = document.getElementById("resVerdict");
  ve.textContent = v.text;
  ve.className = "verdict " + v.cls;

  var rows;
  if (!resHistory.length) {
    rows = '<tr><td colspan="7" class="dim">no segments yet — residuals are published once per SETTLED segment, with the chassis stationary</td></tr>';
  } else {
    rows = resHistory.slice(0, 12).map(function (r) {
      var ageS = (performance.now() - r._rx) / 1000;
      var cls = r.kind === "turn" ? "turn" : "";
      return "<tr class='" + cls + "'>" +
        "<td>" + esc(r.leg_i) + "." + esc(r.segment_i) + "</td>" +
        "<td>" + esc(r.kind) + "</td>" +
        "<td>" + fmt(r.target) + "</td>" +
        "<td>" + fmt(r.measured) + "</td>" +
        "<td class='" + (Math.abs(r.residual) > (r.kind === "turn" ? 5 : 25) ? "bad" : "") + "'>" +
          fmt(r.residual) + "</td>" +
        "<td>" + (r.ratio === null || r.ratio === undefined ? "—" : Number(r.ratio).toFixed(3)) + "</td>" +
        "<td>" + (ageS < 90 ? Math.round(ageS) + "s" : Math.round(ageS / 60) + "m") + "</td>" +
        "</tr>";
    }).join("");
  }
  if (rows !== lastResHtml) {
    document.getElementById("resBody").innerHTML = rows;
    lastResHtml = rows;
  }
}

function fmt(n) {
  return (typeof n === "number") ? n.toFixed(1) : "—";
}

/* ======================================================================
   7. BOOT
   ====================================================================== */
function openModal() {
  document.getElementById("hostInput").value = target.host || "";
  document.getElementById("portInput").value = target.port || 9090;
  document.getElementById("roverModal").classList.remove("hidden");
  document.getElementById("hostInput").focus();
}

function boot() {
  arena = new global.Arena(document.getElementById("gridCanvas"),
                           document.getElementById("dynCanvas"));
  global.addEventListener("resize", function () { arena.resize(); });

  document.getElementById("stopBtn").addEventListener("click", function () { fireStop("button"); });
  /* Spacebar, anywhere, no modifier, no confirmation dialog. A confirmation
     on a panic button is a second thing that can be busy. */
  global.addEventListener("keydown", function (e) {
    if (e.code === "Space" && !/^(INPUT|TEXTAREA)$/.test(document.activeElement.tagName)) {
      e.preventDefault(); fireStop("spacebar");
    }
  });

  document.getElementById("btnReconnect").addEventListener("click", function () {
    log("operator forced a reconnect");
    stopLink.reconnectNow(); mainLink.reconnectNow();
  });
  document.getElementById("btnRover").addEventListener("click", openModal);
  document.getElementById("roverCancel").addEventListener("click", function () {
    document.getElementById("roverModal").classList.add("hidden");
  });
  document.getElementById("roverSave").addEventListener("click", function () {
    var t = parseTarget(document.getElementById("hostInput").value + ":" +
                        document.getElementById("portInput").value);
    if (!t) { return; }
    target = t;
    try { localStorage.setItem(LS_KEY, t.host + ":" + t.port); } catch (e) {}
    document.getElementById("roverModal").classList.add("hidden");
    log("target changed to " + t.host + ":" + t.port + " — every feed cleared");
    connect();
  });

  setInterval(tick, 200);   // 5 Hz, the only renderer

  /* Resolve the target, then connect. */
  var q = parseTarget(new URLSearchParams(location.search).get("rover"));
  if (q) { target = q; connect(); return; }

  var saved = null;
  try { saved = parseTarget(localStorage.getItem(LS_KEY)); } catch (e) {}
  if (saved) { target = saved; connect(); return; }

  /* serve.py answers this with whatever FPMS_ROVER_HOST it was started with.
     A failure here is normal (the page also works opened straight from
     file://), so it falls through to asking rather than to an error. */
  fetch("config.json", { cache: "no-store" }).then(function (r) { return r.json(); })
    .then(function (j) {
      var t = parseTarget(j.rover_host ? (j.rover_host + ":" + (j.rosbridge_port || 9090)) : null);
      if (t) { target = t; connect(); } else { openModal(); }
    })
    .catch(function () { openModal(); });
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", boot);
} else {
  boot();
}

})(window);

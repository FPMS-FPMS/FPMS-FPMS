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
  diagStatus = {};
  healthWords = null;
  reportedThing = null;    // the new rover must re-state its own identity
  silenceSticky = null;    // and re-earn any accusation of silence

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

  /* ---- camera + NPU health ------------------------------------------
     Mirrored onto the graph by the rover's fpms-telemetry-ros, which reads
     the agent's and fpms-npud's MQTT and republishes the HEALTH of both
     under /fpms/ — inside topics_glob, which was NOT widened for it.

     It does not carry the picture, and this page does not ask for one: a
     40 kB base64 JPEG at 6 Hz on the socket that carries the LaserScan is a
     different engineering problem. The question here is whether the camera
     is ALIVE.

     NO `expectS` ON ANY OF THESE, AND THAT IS DELIBERATE. expectS feeds the
     silence banner, which accuses a topic of being wrongly quiet. These
     topics are quiet on any rover whose image predates fpms-telemetry-ros,
     and shouting "CONNECTED BUT NO DATA" at an operator whose rover simply
     does not run that unit yet is a false alarm — and a false alarm is how
     an alarm becomes worthless on the day it is right. Their absence is
     already reported, precisely and quietly, by the tiles themselves.

     Two classes, and the difference is the whole contract:
       state/health  ALWAYS published, from process start, carrying the word
                     "unknown" when the mirror has heard nothing. Silence on
                     these means THE MIRROR is gone.
       everything    published only WHILE ITS SOURCE IS FRESH, and stopped
       else          otherwise. Never zero-filled, never frozen. So a missing
                     scalar is a silent source, not a zero. ---------------- */
  L.subscribe("/fpms/camera/state", "std_msgs/String", function (m) {
    F.mark("cam_state", String(m.data), String(m.data));
  });
  L.subscribe("/fpms/camera/health", "std_msgs/String", function (m) {
    /* The whole snapshot, as JSON. `why` inside it is written for direct
       display and is shown verbatim — see healthTile(). */
    var j = null; try { j = JSON.parse(m.data); } catch (e) {}
    F.mark("cam_health", (j && j.state) ? String(j.state) : "unparsable", j);
  });
  L.subscribe("/fpms/camera/stale", "std_msgs/Bool", function (m) {
    F.mark("cam_stale", m.data ? "stale" : "fresh", !!m.data);
  });
  L.subscribe("/fpms/camera/frame_age_s", "std_msgs/Float32", f32("cam_age"));
  L.subscribe("/fpms/camera/fps", "std_msgs/Float32", f32("cam_fps"));
  L.subscribe("/fpms/camera/resolution", "std_msgs/String", function (m) {
    F.mark("cam_res", String(m.data), String(m.data));
  });

  L.subscribe("/fpms/npu/state", "std_msgs/String", function (m) {
    F.mark("npu_state", String(m.data), String(m.data));
  });
  L.subscribe("/fpms/npu/health", "std_msgs/String", function (m) {
    var j = null; try { j = JSON.parse(m.data); } catch (e) {}
    F.mark("npu_health", (j && j.state) ? String(j.state) : "unparsable", j);
  });
  /* The single most consequential boolean on this page. false is fpms-npud
     saying THE ROVER IS NOT DETECTING. */
  L.subscribe("/fpms/npu/detection_available", "std_msgs/Bool", function (m) {
    F.mark("npu_det", m.data ? "yes" : "NO", !!m.data);
  });
  L.subscribe("/fpms/npu/model_loaded", "std_msgs/Bool", function (m) {
    F.mark("npu_loaded", m.data ? "yes" : "NO", !!m.data);
  });
  L.subscribe("/fpms/npu/model_sha_state", "std_msgs/String", function (m) {
    F.mark("npu_sha", String(m.data), String(m.data));
  });
  L.subscribe("/fpms/npu/p50_ms", "std_msgs/Float32", f32("npu_p50"));
  L.subscribe("/fpms/npu/p90_ms", "std_msgs/Float32", f32("npu_p90"));
  L.subscribe("/fpms/npu/infer_per_s", "std_msgs/Float32", f32("npu_rate"));
  L.subscribe("/fpms/npu/drops", "std_msgs/Int32", function (m) {
    F.mark("npu_drops", String(m.data), m.data);
  });
  L.subscribe("/fpms/npu/consecutive_failures", "std_msgs/Int32", function (m) {
    F.mark("npu_consec", String(m.data), m.data);
  });
  L.subscribe("/fpms/npu/core_mask", "std_msgs/String", function (m) {
    F.mark("npu_cores", String(m.data), String(m.data));
  });

  /* ---- the two fault mirrors: EDGES, not cadence -------------------- */
  L.subscribe("/fpms/npu/fault", "std_msgs/String", function (m) {
    var j = null; try { j = JSON.parse(m.data); } catch (e) {}
    F.mark("npu_fault", faultWord(j, m.data), j || m.data);
    log("NPU " + (j && j._topic ? j._topic : "fault") + ": " + faultWord(j, m.data));
  });
  L.subscribe("/fpms/agent/fault", "std_msgs/String", function (m) {
    var j = null; try { j = JSON.parse(m.data); } catch (e) {}
    F.mark("agent_fault", faultWord(j, m.data), j || m.data);
    log("agent " + (j && j._topic ? j._topic : "fault") + ": " + faultWord(j, m.data));
  });
}

/* The one human-readable word out of a mirrored event, whatever shape the
   emitter used. Never throws, never returns empty: an event this page cannot
   name still has to be visible. */
function faultWord(j, raw) {
  if (!j || typeof j !== "object") { return String(raw || "").slice(0, 120); }
  var w = j.fault || j.error || j.component || j.cleared || j._topic || "event";
  if (j.detail || j.fault_detail) { w += " — " + (j.detail || j.fault_detail); }
  return String(w).slice(0, 160);
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
   3b. DIAGNOSING SILENCE — turning "no data" into a specific instruction.
   ----------------------------------------------------------------------
   The link layer reports WHICH topics have gone quiet. This turns that into
   the one sentence an operator can act on, using the fact that /diagnostics
   and the /fpms/* mirrors come from the SAME NODE:

     /diagnostics silent too      -> the link or rosbridge itself. Nothing
                                     from the bridge is reaching us at all.
     /diagnostics ARRIVING but
     the mirrors silent           -> the bridge is alive and its MQTT side is
                                     empty. That is the wrong-topic-root
                                     signature: the bridge subscribes
                                     fpms/<thing>/telemetry/#, so a thing-name
                                     mismatch produces exactly this and
                                     nothing errors anywhere.
     only /scan_lidar silent      -> the LiDAR publisher, not the link.

   Each verdict names the check to run, in order of how likely it is to be
   the answer. None of them says "unknown".
   ====================================================================== */
/* Which feed key carries each topic that can declare an expectS. Needed
   because the LINK's silence bookkeeping is per-socket and resets whenever
   the remedy ladder reconnects — so if the banner were driven straight off
   it, a wrong thing name would make the alarm flicker off for 25 s after
   every automatic reconnect, which is worse than not having it. The FEED
   registry survives reconnects (it ages from receipt, monotonically), so a
   diagnosis is only cleared when the named topics genuinely start arriving
   again — never merely because a socket was replaced. */
var TOPIC_FEED = {
  "/scan_lidar": "scan",
  "/diagnostics": "diag",
  "/fpms/mission/state": "state",
  "/fpms/mission/phase": "phase"
};

var silenceSticky = null;

function diagnoseSilence(snap) {
  var silent = snap.silent || [];

  if (!silent.length) {
    if (!silenceSticky) { return null; }
    /* Clear only once EVERY topic we accused is demonstrably delivering. */
    var recovered = silenceSticky.names.every(function (n) {
      var k = TOPIC_FEED[n];
      return k && !F.isStale(k);
    });
    if (recovered) { silenceSticky = null; return null; }
    return silenceSticky;      // hold the diagnosis across the reconnect
  }

  var names = silent.map(function (s) { return s.topic; });
  var worst = Math.max.apply(null, silent.map(function (s) {
    /* Prefer the FEED's age. It is measured from the last time this browser
       actually saw the topic, across every socket — which is what the
       operator means by "how long has it been quiet". The link's own figure
       restarts at each reconnect and would under-report. */
    var k = TOPIC_FEED[s.topic];
    return (k && F.seen(k)) ? F.age(k) : s.silentS;
  }));
  var diagSilent = names.indexOf("/diagnostics") >= 0;
  var mirrorsSilent = names.some(function (n) { return n.indexOf("/fpms/") === 0; });
  var lidarSilent = names.indexOf("/scan_lidar") >= 0;

  var head, body;
  if (diagSilent && mirrorsSilent && lidarSilent) {
    head = "CONNECTED BUT NO DATA AT ALL.";
    body = "The socket is up and rosbridge is answering, yet not one " +
           "subscribed topic has delivered a message in " + worst.toFixed(0) + "s. " +
           "Check, in this order: (1) is the rover's ROS_DOMAIN_ID still 20; " +
           "(2) did rosbridge start BEFORE the publishers — it only ever " +
           "delivers topics whose publisher already existed, so " +
           "sudo systemctl restart fpms-rosbridge on the Pi; " +
           "(3) is this the right rover at all.";
  } else if (!diagSilent && mirrorsSilent) {
    head = "CONNECTED BUT NO MISSION DATA — CHECK THE THING NAME.";
    body = "/diagnostics IS arriving, so the Pi-side bridge is alive and " +
           "this link is fine — but " + names.filter(function (n) { return n.indexOf("/fpms/") === 0; }).join(", ") +
           " has been silent for " + worst.toFixed(0) + "s. That bridge subscribes " +
           "fpms/<thing>/telemetry/# on MQTT, so a thing-name mismatch " +
           "produces exactly this and errors nowhere. This dashboard is set " +
           "to thing '" + target.thing + "'" +
           (reportedThing ? "; the rover reports '" + reportedThing + "'" : "") +
           ". Check FPMS_THING_NAME in /etc/fpms/config.env, then that " +
           "fpms-missions is actually running.";
  } else if (lidarSilent && !mirrorsSilent) {
    head = "CONNECTED, BUT NO LIDAR.";
    body = "/scan_lidar has delivered nothing for " + worst.toFixed(0) + "s while other " +
           "topics are arriving, so the link is fine and the publisher is not. " +
           "Check fpms-lidar-ros and fpms-rover-agent on the Pi. The map is " +
           "drawing no returns, which is the honest picture — not a clear arena.";
  } else {
    head = "CONNECTED BUT SILENT ON: " + names.join(", ");
    body = "Quiet for " + worst.toFixed(0) + "s while the link is up. Check the " +
           "publisher for each, the thing name (" + target.thing + "), and " +
           "whether rosbridge started before the publishers.";
  }
  silenceSticky = { head: head, body: body, names: names, worst: worst,
                    remedy: snap.wedgeRemedy };
  return silenceSticky;
}

/* The rover's own idea of its identity, from DiagnosticStatus.hardware_id,
   against the dashboard's. Silent agreement is the normal case and shows
   nothing; disagreement is loud, because every number on the screen then
   belongs to a different robot than the label says. */
function checkIdentity() {
  if (!reportedThing || F.isStale("diag")) { return null; }
  if (reportedThing === target.thing) { return null; }
  return "IDENTITY MISMATCH: this dashboard is set to '" + target.thing +
         "' but the rover on " + target.host + " reports '" + reportedThing +
         "'. Everything on this screen belongs to '" + reportedThing +
         "'. Fix the selection in the header, or you are watching the wrong robot.";
}

/* ======================================================================
   4. LINK HEALTH TILES
   ----------------------------------------------------------------------
   FOUR subsystems, and ALL FOUR now have a ROS source.

     LIDAR  — /scan_lidar arrivals here + /fpms/mission/lidar_ok + the Pi's
              own ros/scan_lidar diagnostic.
     ESP32  — /fpms_health flag bit4 is the micro-ROS agent session, which IS
              the board link, corroborated by /wheel_ticks arriving.
     CAMERA — /fpms/camera/state + /fpms/camera/health, and the scalars.
     NPU    — /fpms/npu/state + /fpms/npu/health, and the scalars.

   The last two used to read NO ROS SOURCE, hatched grey, because nothing
   published camera or NPU health onto the graph at all — the agent sent
   frames straight to MQTT and fpms-npud raised its faults there. The rover
   now runs fpms-telemetry-ros, which mirrors both onto /fpms/*, inside
   topics_glob, WITHOUT WIDENING IT. That was always the fix: a rover-side
   publisher, not a dashboard that guesses.

   THE RULE THE COLOUR MAPPING EXISTS TO ENFORCE
   Guessing green here would be the worst possible failure of this dashboard:
   a blind fire-detection rover reported healthy. So "unknown" — the word the
   mirror publishes when it is running and its own source is silent — is
   still rendered as NO ROS SOURCE, hatched grey. It is neither good news nor
   bad news, and it must never be painted as either. Likewise a mirror that
   has gone quiet, or never spoke: that is a missing BRIDGE, and "NO ROS
   SOURCE" remains the honest answer for it.

   And in the other direction: `off` is a deliberate operator action — the
   camera stream was switched off on purpose — so it is amber, never red. Red
   for a thing somebody chose is how an operator learns to ignore red.
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

/* THE STATE WORD -> TILE CLASS MAP. It lives in exactly one place so the
   camera and the NPU can never drift apart, and so the rule is readable:

     unknown   nosrc   the mirror is UP and says it cannot see its source.
                       Not green, not red. From the operator's chair that is
                       still "no source" — and an "unknown" painted green is
                       a blind fire-detection rover reported healthy.
     ok        ok
     warn      warn    arriving, slower than it should be
     off       warn    DELIBERATE. Streaming was switched off by an operator.
                       Not a fault, and colouring it like one teaches the
                       operator to ignore the colour that means fault.
     degraded  warn    serving, but not as configured
     stale     down    it was alive and it has stopped
     fault     down    named, declared, by the daemon that owns it

   Any word not in this table is treated as nosrc, not as ok. A dashboard
   that colours a word it does not understand is guessing. */
var STATE_CLS = {
  unknown: "nosrc",
  ok: "ok",
  warn: "warn", off: "warn", degraded: "warn",
  stale: "down", fault: "down"
};

/* The shared skeleton of the CAMERA and NPU tiles: bridge liveness first,
   then the state word, then the mirror's own `why` verbatim.

   FRESHNESS IS OURS, ALWAYS. Every level and every age below comes from
   F.*(stateKey) — measured in this browser from the moment the message
   arrived. The snapshot carries its own ages (frame_age_s, telemetry_age_s,
   ts); those are DATA ABOUT THE ROVER'S SOURCES and are displayed as such,
   never used to decide whether what is on this screen is fresh. */
function healthTile(o) {
  var lv = F.level(o.stateKey);
  var src = o.topic + " + " + o.healthTopic;

  /* 1. IS THE MIRROR THERE AT ALL? /fpms/<x>/state is published at 1 Hz from
        process start, unconditionally, carrying "unknown" when the mirror has
        heard nothing. So silence on it is a statement about fpms-telemetry-ros
        and about NOTHING ELSE. Grey, not red: a missing bridge is not
        evidence that the sensor is broken. */
  if (lv === "never") {
    return { name: o.name, cls: "nosrc", val: "NO ROS SOURCE",
             age: "never seen", src: src,
             note: "Nothing has ever published " + o.topic + " to this browser. " +
                   "fpms-telemetry-ros is what mirrors " + o.what + " health from " +
                   "MQTT onto the graph; if that unit is not running on the rover, " +
                   "or this image predates it, there is no source here. That is " +
                   "not evidence about the " + o.what + " in either direction — " +
                   "check systemctl status fpms-telemetry-ros on the Pi." };
  }
  if (lv === "stale") {
    return { name: o.name, cls: "nosrc", val: "NO ROS SOURCE",
             age: F.ageText(o.stateKey), src: src,
             note: o.topic + " was arriving and has stopped (" +
                   F.ageText(o.stateKey) + "). That mirror publishes at 1 Hz " +
                   "whatever it knows, including the word \"unknown\", so silence " +
                   "means fpms-telemetry-ros itself is down or unreachable — " +
                   "which says nothing about the " + o.what + "." };
  }

  var word = String((F.get(o.stateKey) || {}).v || "").toLowerCase();
  var cls = STATE_CLS[word];
  var notes = [];
  if (!cls) {
    cls = "nosrc";
    notes.push("the mirror reports a state this dashboard does not recognise (\"" +
               word + "\"). It will not colour a word it cannot interpret.");
  }

  /* 2. THE MIRROR'S OWN WORDS. `why` is written by fpms-telemetry-ros for
        direct display — it names the specific evidence behind the state, and
        it is shown VERBATIM. Composing our own sentence here would be a
        second opinion assembled from less information than the mirror had. */
  var health = F.fresh(o.healthKey);
  if (health && typeof health.why === "string" && health.why) {
    notes.push(health.why);
  } else if (F.seen(o.healthKey)) {
    notes.push("the health snapshot on " + o.healthTopic + " is stale (" +
               F.ageText(o.healthKey) + ") while the state word is fresh — " +
               "showing the state alone rather than an old explanation of it.");
  }

  /* 3. A REFUSED MQTT SUBSCRIPTION LOOKS EXACTLY LIKE A DEAD SENSOR, and it
        is the difference between "the camera failed" and "the broker ACL does
        not grant this bridge". The mirror checks the SUBACK rather than the
        return of subscribe() precisely so this can be said out loud. */
  if (health && health.mqtt) {
    var ref = health.mqtt.refused || [];
    if (ref.length) {
      notes.unshift("BROKER REFUSED THE SUBSCRIPTION for " +
        ref.map(function (r) { return r.topic + " (" + r.code + ")"; }).join(", ") +
        ". The mirror is connected and granted nothing, which from here is " +
        "indistinguishable from a dead sensor — but it is an ACL, not the " +
        o.what + ". Check /etc/mosquitto/fpms.acl against FPMS_THING_NAME.");
      if (cls === "ok") { cls = "warn"; }
    } else if (health.mqtt.state && health.mqtt.state !== "connected") {
      notes.unshift("the mirror's own MQTT link is \"" + health.mqtt.state +
        "\", so it is reporting on a source it currently cannot hear.");
      if (cls === "ok") { cls = "warn"; }
    }
  }

  return { name: o.name, cls: cls,
           val: (word === "unknown") ? "UNKNOWN — NO SOURCE" : word.toUpperCase(),
           age: F.ageText(o.stateKey), src: src, notes: notes, word: word };
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
    var t = healthTile({ name: "CAMERA", what: "camera",
                         stateKey: "cam_state", healthKey: "cam_health",
                         topic: "/fpms/camera/state",
                         healthTopic: "/fpms/camera/health" });
    if (t.notes) {
      var health = F.fresh("cam_health") || {};
      var fps = F.fresh("cam_fps");
      if (t.cls === "ok" && fps !== null) { t.val = "OK · " + fps.toFixed(1) + " fps"; }

      /* CROSS-CHECK, not decoration. The mirror computes `state` and `stale`
         in the same tick from the same snapshot, so they cannot honestly
         disagree — if they do, something between here and there is rewriting
         one of them, and the safe reading is the pessimistic one. */
      if (F.fresh("cam_stale") === true && t.cls === "ok") {
        t.cls = "warn";
        t.notes.push("the mirror says state=ok while /fpms/camera/stale is true; " +
                     "those two are computed in the same tick and cannot both be " +
                     "right — treating it as the worse of the two.");
      }

      var res = F.fresh("cam_res");
      if (res) { t.notes.push(res); }
      /* The mirror's own measurement of the frame's age AT THE MIRROR. It is a
         fact about the rover's camera, published as data — never this page's
         idea of whether the tile is fresh, which is the age line above. */
      var fa = F.fresh("cam_age");
      if (fa !== null) {
        t.notes.push("last frame reached the rover-side mirror " + fa.toFixed(1) +
                     "s before it sent this");
      }
      if (fps !== null) { t.notes.push(fps.toFixed(1) + " fps into the mirror"); }
      if (typeof health.agent_fps === "number") {
        t.notes.push("the agent's own encoder is at " + health.agent_fps.toFixed(1) +
                     " fps (if that climbs while the mirror's rate does not, the " +
                     "frames are being lost on the link, not at the camera)");
      }
      if (health.agent_heartbeat) {
        t.notes.push("agent heartbeat " + health.agent_heartbeat +
                     (health.agent_camera_ok === false ? ", and it reports camera_ok=FALSE" : ""));
      }
      if (health.camera_fault_active) {
        t.notes.push("agent fault standing: " + health.camera_fault_active);
      }
      if (F.seen("agent_fault")) {
        t.notes.push("last agent event: " + F.get("agent_fault").v + " (" +
                     F.ageText("agent_fault") + ")");
      }
      t.note = t.notes.join(" · ");
    }
    tiles.push(t);
  })();

  /* ---- NPU ---- */
  (function () {
    var t = healthTile({ name: "NPU", what: "NPU",
                         stateKey: "npu_state", healthKey: "npu_health",
                         topic: "/fpms/npu/state",
                         healthTopic: "/fpms/npu/health" });
    if (t.notes) {
      var health = F.fresh("npu_health") || {};
      var p50 = F.fresh("npu_p50");
      if (t.cls === "ok" && p50 !== null) { t.val = "OK · p50 " + p50.toFixed(0) + " ms"; }

      /* THE ONE THAT MATTERS. detection_available=false is fpms-npud stating
         that this rover is not detecting anything. The mirror already turns
         that into state=fault, and this check does not trust it to: a
         DETECTING=false tile must not be green no matter what word arrived
         alongside it. Only ok/warn are overridden — an "unknown" stays grey,
         because the rule that "unknown" is never coloured runs in both
         directions. */
      if (F.fresh("npu_det") === false) {
        if (t.cls === "ok" || t.cls === "warn") {
          t.cls = "down";
          t.val = "NOT DETECTING";
        }
        t.notes.unshift("detection_available=FALSE — THE ROVER IS NOT DETECTING. " +
                        "Fire detection is not running; nothing on this screen " +
                        "should be read as if it were.");
      }
      if (F.fresh("npu_loaded") === false) {
        t.notes.push("no model is loaded");
        if (t.cls === "ok") { t.cls = "warn"; }
      }
      var sha = F.fresh("npu_sha");
      if (sha === "mismatch") {
        t.notes.push("model SHA MISMATCH against /etc/fpms/models.json — the " +
                     "weights running are not the weights on record");
        if (t.cls === "ok") { t.cls = "warn"; }
      } else if (sha) {
        t.notes.push("model sha " + sha);
      }
      var lat = [];
      if (p50 !== null) { lat.push("p50 " + p50.toFixed(0) + " ms"); }
      var p90 = F.fresh("npu_p90");
      if (p90 !== null) { lat.push("p90 " + p90.toFixed(0) + " ms"); }
      var rate = F.fresh("npu_rate");
      if (rate !== null) { lat.push(rate.toFixed(1) + " infer/s"); }
      if (lat.length) { t.notes.push(lat.join(" / ")); }
      var drops = F.fresh("npu_drops");
      if (drops !== null && drops > 0) { t.notes.push(drops + " drops"); }
      var consec = F.fresh("npu_consec");
      if (consec !== null && consec > 0) {
        t.notes.push(consec + " consecutive failures");
        if (t.cls === "ok") { t.cls = "warn"; }
      }
      var cores = F.fresh("npu_cores");
      if (cores) { t.notes.push("cores " + cores + " (in force, not requested)"); }
      if (health.fault) {
        t.notes.push("fault: " + health.fault +
                     (health.fault_detail ? " — " + health.fault_detail : ""));
      }
      if (F.seen("npu_fault")) {
        t.notes.push("last NPU event: " + F.get("npu_fault").v + " (" +
                     F.ageText("npu_fault") + ")");
      }
      t.note = t.notes.join(" · ");
    }
    tiles.push(t);
  })();

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

  /* --- the two banners. Silence first: it is the failure that otherwise
         looks exactly like a healthy dashboard. ------------------------- */
  var idErr = checkIdentity();
  var idBan = document.getElementById("idBanner");
  idBan.classList.toggle("hidden", !idErr);
  if (idErr) { document.getElementById("idDetail").textContent = idErr; }

  var d = diagnoseSilence(ms);
  var ban = document.getElementById("silenceBanner");
  ban.classList.toggle("hidden", !d);
  if (d) {
    document.getElementById("silenceHead").textContent = d.head;
    document.getElementById("silenceBody").textContent = d.body;
    document.getElementById("silenceDetail").textContent =
      "socket up " + ms.upS.toFixed(0) + "s · " + ms.counters.frames + " frames · " +
      ms.counters.data + " topic messages · automatic remedy " +
      Math.min(d.remedy, 2) + "/2 attempted" +
      (d.remedy >= 3 ? " and stopped — retrying further would only drop the topics that DO work" : "");
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
    "thing  " + target.thing + " (dashboard)  vs  " +
      (reportedThing || "not yet reported") + " (rover /diagnostics hardware_id)\n" +
    "zones  " + (zonesCheck
      ? (zonesCheck.ok
          ? "zones.json cross-checked OK, " + zonesCheck.checked + " centres within 0.05 mm"
          : "zones.json REFUSED — " + zonesCheck.problems.join("; ") + " (using the built-in derivation)")
      : "no zones.json served — using the built-in derivation (no fault)") + "\n" +
    "expect " + (ms.expecting || []).join(" ") + "\n" +
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
  document.getElementById("hostInput").value = target.host || DEFAULT_HOST;
  document.getElementById("portInput").value = target.port || DEFAULT_PORT;
  document.getElementById("thingInput").value = target.thing || DEFAULT_THING;
  document.getElementById("ipInput").value = "";
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
    /* The IP override wins when it is filled in. mDNS is per-resolver, not
       per-machine — the name can work everywhere else on this laptop and
       still fail inside the browser — so the raw address is a first-class
       input, not a troubleshooting afterthought. */
    var ip = document.getElementById("ipInput").value.trim();
    var host = ip || document.getElementById("hostInput").value;
    var t = parseTarget(host + ":" + document.getElementById("portInput").value,
                        document.getElementById("thingInput").value);
    if (!t) { return; }
    target = t;
    try { localStorage.setItem(LS_KEY, JSON.stringify(t)); } catch (e) {}
    document.getElementById("roverModal").classList.add("hidden");
    log("target changed to " + t.host + ":" + t.port + " thing " + t.thing +
        " — every feed cleared" + (ip ? " (IP override in use)" : ""));
    reportedThing = null;
    connect();
  });

  setInterval(tick, 200);   // 5 Hz, the only renderer

  /* A copy of /etc/fpms/zones.json, if serve.py can see one. Optional by
     design: absent file -> built-in derivation, no note, no fault, exactly
     as zones.json itself specifies. Present file -> we recompute and REFUSE
     it on disagreement rather than adopting its numbers. */
  fetch("zones.json", { cache: "no-store" }).then(function (r) { return r.json(); })
    .then(function (j) {
      zonesCheck = global.ARENA.validateZones(j);
      if (zonesCheck.ok) {
        log("zones.json cross-check OK — " + zonesCheck.checked +
            " centres agree with the derivation to within 0.05 mm");
      } else {
        log("ZONES.JSON REFUSED: " + zonesCheck.problems.join("; ") +
            " — keeping the built-in derivation");
      }
    })
    .catch(function () { /* no file; nothing to say */ });

  /* Resolve the target, then connect. Unlike the host, the THING NAME never
     falls back silently: it is stamped in the header on every path. */
  var qs = new URLSearchParams(location.search);
  var q = parseTarget(qs.get("rover"), qs.get("thing"));
  if (q) { target = q; connect(); return; }

  var saved = null;
  try {
    var raw = localStorage.getItem(LS_KEY);
    if (raw && raw.charAt(0) === "{") {
      var o = JSON.parse(raw);
      saved = parseTarget(o.host + ":" + o.port, o.thing);
    } else if (raw) {
      saved = parseTarget(raw);          // pre-thing-name format
    }
  } catch (e) {}
  if (saved) { target = saved; connect(); return; }

  /* serve.py answers this with whatever it was started with. A failure here
     is normal — the page also works opened straight from file:// — and it
     falls through to the built-in rover1 defaults, NOT to a prompt: an
     operator who has just flashed this image should get a live dashboard by
     opening a URL, and a wrong guess is now loud rather than silent. */
  fetch("config.json", { cache: "no-store" }).then(function (r) { return r.json(); })
    .then(function (j) {
      var t = parseTarget(
        (j.rover_host || DEFAULT_HOST) + ":" + (j.rosbridge_port || DEFAULT_PORT),
        j.thing_name);
      target = t || target;
      connect();
    })
    .catch(function () { connect(); });
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", boot);
} else {
  boot();
}

})(window);

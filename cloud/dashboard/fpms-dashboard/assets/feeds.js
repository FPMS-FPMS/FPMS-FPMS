/* =========================================================================
   feeds.js — the freshness registry. This is the honesty mechanism.
   =========================================================================

   THE RULE, AND IT IS NOT NEGOTIABLE:

     A VALUE'S AGE IS MEASURED FROM THE MOMENT THIS BROWSER RECEIVED IT.
     NO TIMESTAMP INSIDE A PAYLOAD IS EVER USED FOR FRESHNESS.

   Two independent reasons, both of which have actually bitten this project:

     * A stuck publisher keeps re-sending a payload whose internal stamp is
       old but constant — or worse, one it re-stamps on every emit while the
       underlying sensor is dead. Either way the payload's own opinion of its
       age is worthless as evidence that the SENSOR is alive.
     * The Pi's clock and the laptop's clock do not agree, are not
       synchronised at a competition venue with no NTP route, and the sign of
       the disagreement is unknown. Subtracting one from the other produces a
       plausible number that is not an age.

   `performance.now()` is used rather than `Date.now()` because it is
   monotonic: an NTP step or a manual clock change on the operator's laptop
   cannot make a stale value look fresh.

   SECOND RULE: THE TICKER IS THE ONLY RENDERER.

   A message handler calls mark() and does nothing else. Every visible value
   is re-derived from the registry at 5 Hz, so there is no code path that can
   leave a number on screen without re-checking its age. If the render path
   ran from the message handler, a feed that STOPPED would simply keep its
   last painted pixels — which is the exact failure this file exists to make
   impossible.

   THIRD RULE: EVERY FEED DECLARES ITS OWN THRESHOLDS.

   One global STALE_S is wrong here. /scan_lidar runs at 10 Hz and is dead at
   5 s; `telemetry/mission` is a ~1 Hz mirror the Pi's own /diagnostics gives
   20 s before calling it an error; the residual stream is EVENT-DRIVEN — it
   fires once per settled segment, so a 40 s gap is a rover standing still,
   not a fault. Judging all three by one number either cries wolf on the
   residuals or hides a dead scanner. The thresholds below are taken from the
   Pi's own WATCH_ROS/WATCH_MQTT table in fpms_foxglove_cmd.py so that this
   dashboard and the rover's /diagnostics never disagree about the same feed.
   ========================================================================= */
(function (global) {
"use strict";

/* warn_s / stale_s per feed key. `event: true` means the feed is emitted on
   an event, not on a cadence — it is aged and labelled but a long age is
   reported as "last seen", never as a fault. */
var SPEC = {
  /* --- sensors, straight off the board / the LiDAR node ---------------- */
  scan:      { warn: 2,  stale: 5,   label: "/scan_lidar" },
  batt:      { warn: 3,  stale: 10,  label: "/battery" },
  ticks:     { warn: 2,  stale: 5,   label: "/wheel_ticks" },
  health:    { warn: 2,  stale: 5,   label: "/fpms_health" },
  diag:      { warn: 2,  stale: 5,   label: "/diagnostics" },

  /* --- mission mirror. ~1 Hz WHILE A MISSION RUNS; silent when parked. -- */
  x:         { warn: 3,  stale: 5,   label: "/fpms/mission/x_mm" },
  y:         { warn: 3,  stale: 5,   label: "/fpms/mission/y_mm" },
  hdg:       { warn: 3,  stale: 5,   label: "/fpms/mission/heading_deg" },
  state:     { warn: 5,  stale: 20,  label: "/fpms/mission/state" },
  phase:     { warn: 5,  stale: 20,  label: "/fpms/mission/phase" },
  armed:     { warn: 5,  stale: 20,  label: "/fpms/mission/armed" },
  linkok:    { warn: 5,  stale: 20,  label: "/fpms/mission/link_ok" },
  lidarok:   { warn: 5,  stale: 20,  label: "/fpms/mission/lidar_ok" },
  leg:       { warn: 5,  stale: 20,  label: "/fpms/mission/leg_i" },
  seg:       { warn: 5,  stale: 20,  label: "/fpms/mission/segment_i" },
  drem:      { warn: 5,  stale: 20,  label: "/fpms/mission/distance_remaining_mm" },
  dtrav:     { warn: 5,  stale: 20,  label: "/fpms/mission/distance_travelled_mm" },
  front:     { warn: 3,  stale: 8,   label: "/fpms/mission/front_mm" },
  battm:     { warn: 5,  stale: 20,  label: "/fpms/mission/batt_v" },

  /* --- camera + NPU health, mirrored onto the graph by the rover's
         fpms-telemetry-ros. 1 Hz (FPMS_TELEMETRY_ROS_HZ), so the same
         2/5-style bounds as `health` and `diag` would be too tight: at a 1 s
         cadence a single missed tick would read as a fault. 3/10 is three
         missed ticks before WARN and ten before STALE.

         THE ONE THING TO UNDERSTAND ABOUT THESE FEEDS: going stale here means
         THE MIRROR stopped, not that the camera or the NPU stopped. The
         mirror publishes state and health UNCONDITIONALLY from process start
         — carrying the literal word "unknown" when it has heard nothing — so
         silence on /fpms/camera/state is a statement about fpms-telemetry-ros
         and about nothing else. app.js renders that as NO ROS SOURCE, not as
         a dead sensor. The scalars below are the opposite: the mirror stops
         publishing them the moment their source goes quiet, deliberately, so
         a stale one is simply absent rather than frozen.

         The first seven are the set the mirror's author specified. The rest
         are the remaining scalars it publishes, registered at the identical
         bound so that no feed on this page falls through to DEFAULT_SPEC —
         feeds.js's third rule is that every feed declares its own. --------- */
  cam_state:  { warn: 3, stale: 10, label: "/fpms/camera/state" },
  cam_health: { warn: 3, stale: 10, label: "/fpms/camera/health" },
  cam_fps:    { warn: 3, stale: 10, label: "/fpms/camera/fps" },
  cam_age:    { warn: 3, stale: 10, label: "/fpms/camera/frame_age_s" },
  cam_stale:  { warn: 3, stale: 10, label: "/fpms/camera/stale" },
  cam_res:    { warn: 3, stale: 10, label: "/fpms/camera/resolution" },

  npu_state:  { warn: 3, stale: 10, label: "/fpms/npu/state" },
  npu_health: { warn: 3, stale: 10, label: "/fpms/npu/health" },
  npu_det:    { warn: 3, stale: 10, label: "/fpms/npu/detection_available" },
  npu_loaded: { warn: 3, stale: 10, label: "/fpms/npu/model_loaded" },
  npu_sha:    { warn: 3, stale: 10, label: "/fpms/npu/model_sha_state" },
  npu_p50:    { warn: 3, stale: 10, label: "/fpms/npu/p50_ms" },
  npu_p90:    { warn: 3, stale: 10, label: "/fpms/npu/p90_ms" },
  npu_rate:   { warn: 3, stale: 10, label: "/fpms/npu/infer_per_s" },
  npu_drops:  { warn: 3, stale: 10, label: "/fpms/npu/drops" },
  npu_consec: { warn: 3, stale: 10, label: "/fpms/npu/consecutive_failures" },
  npu_cores:  { warn: 3, stale: 10, label: "/fpms/npu/core_mask" },

  /* --- the two fault mirrors. EDGES: fpms-npud and the rover agent emit them
         once, on a transition. A long age here is a rover that has not
         faulted, which is the good case, so they are aged and labelled and
         never allowed to colour a tile by their age alone. -------------- */
  npu_fault:   { warn: 60, stale: 300, event: true, label: "/fpms/npu/fault" },
  agent_fault: { warn: 60, stale: 300, event: true, label: "/fpms/agent/fault" },

  /* --- residuals: one per SETTLED SEGMENT. Event-driven by nature. ----- */
  res_raw:     { warn: 60, stale: 300, event: true, label: "/fpms/residual/raw" },
  res_mm:      { warn: 60, stale: 300, event: true, label: "/fpms/residual/drive_mm" },
  res_deg:     { warn: 60, stale: 300, event: true, label: "/fpms/residual/turn_deg" },
  res_lat:     { warn: 60, stale: 300, event: true, label: "/fpms/residual/lateral_mm" },
  res_cum_mm:  { warn: 60, stale: 300, event: true, label: "/fpms/residual/cumulative_drive_mm" },
  res_cum_deg: { warn: 60, stale: 300, event: true, label: "/fpms/residual/cumulative_turn_deg" },

  /* --- log-ish feeds: aged for display, never gate anything ----------- */
  events:    { warn: 30, stale: 120, event: true, label: "/fpms/events" },
  plan:      { warn: 60, stale: 600, event: true, label: "/fpms/plan/route" }
};

var DEFAULT_SPEC = { warn: 3, stale: 10, label: "(unregistered)" };

var feeds = {};   // key -> {t, v, raw, n}
var rates = {};   // key -> {last, hz}

function spec(key) { return SPEC[key] || DEFAULT_SPEC; }

/* Record an arrival. `v` is the display string, `raw` the value panels
   compute from. Nothing else in this dashboard writes a feed. */
function mark(key, v, raw) {
  var t = performance.now();
  var r = rates[key] || (rates[key] = { last: 0, hz: 0 });
  if (r.last) {
    var dt = (t - r.last) / 1000;
    if (dt > 1e-4) {
      var inst = 1 / dt;
      /* Same rule the rover agent uses on its own hz: weight the newest
         sample hard so ONE long gap moves the number a lot. A smooth average
         keeps reporting a plausible rate long after the source dies, and a
         plausible-but-frozen number has fooled this project twice. */
      r.hz = (r.hz <= 0) ? inst : (0.6 * r.hz + 0.4 * inst);
    }
  }
  r.last = t;
  var prev = feeds[key];
  feeds[key] = { t: t, v: v, raw: raw, n: (prev ? prev.n : 0) + 1 };
}

function get(key)  { return feeds[key] || null; }
function seen(key) { return !!feeds[key]; }

/* Infinity when never seen. Callers must handle that explicitly rather than
   treating "never" as "0 seconds old". */
function age(key) {
  var f = feeds[key];
  return f ? (performance.now() - f.t) / 1000 : Infinity;
}

function hz(key) {
  /* A rate is only meaningful while the feed is live. Past its stale bound
     the honest rate is zero, not the last average. */
  if (isStale(key)) { return 0; }
  var r = rates[key];
  return (r && r.hz > 0) ? r.hz : 0;
}

function isStale(key) { return age(key) > spec(key).stale; }
function isWarn(key)  { var a = age(key); return a > spec(key).warn && a <= spec(key).stale; }

/* One of: "never" | "ok" | "warn" | "stale". Panels style off this and
   nothing else, so every panel ages identically. */
function level(key) {
  if (!seen(key)) { return "never"; }
  if (isStale(key)) { return "stale"; }
  if (isWarn(key))  { return "warn"; }
  return "ok";
}

/* Human age, always shown next to a value. "—" is never used for an age:
   an unknown age is the thing the operator most needs to see. */
function ageText(key) {
  if (!seen(key)) { return "never seen"; }
  var a = age(key);
  if (a < 1)   { return a.toFixed(1) + "s ago"; }
  if (a < 90)  { return Math.round(a) + "s ago"; }
  if (a < 5400) { return Math.round(a / 60) + "m ago"; }
  return Math.round(a / 3600) + "h ago";
}

/* The value, but ONLY if it is fresh enough to be believed. Every panel that
   renders a number goes through here; there is no accessor that returns a
   stale value by accident. */
function fresh(key) {
  return (seen(key) && !isStale(key)) ? feeds[key].raw : null;
}

/* Clear everything. Used when the operator re-points the dashboard at a
   different rover: carrying rover2's last pose into rover1's map would be
   the exact "confident value in the wrong frame" failure this project has
   already paid for once. */
function reset() { feeds = {}; rates = {}; }

global.Feeds = {
  SPEC: SPEC, mark: mark, get: get, seen: seen, age: age, hz: hz,
  isStale: isStale, isWarn: isWarn, level: level, ageText: ageText,
  fresh: fresh, reset: reset, spec: spec
};

})(window);

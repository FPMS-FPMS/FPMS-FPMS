/* =========================================================================
   arena.js — the bird's-eye arena map.
   =========================================================================

   FRAME, ONCE, PLAINLY:

     ARENA FRAME. Origin BOTTOM-LEFT. Units MILLIMETRES. 1200 x 1200.
     Map metres = arena millimetres / 1000. The start box is (972, 228),
     heading 90 deg (up the arena). Those are the same numbers in
     /etc/fpms/config.env as FPMS_ARENA_ANCHOR_X/Y/YAW = 0.972 / 0.228 /
     1.5707963.

   THE POSE ON THIS MAP COMES FROM /fpms/mission/{x_mm,y_mm,heading_deg}
   AND FROM NOTHING ELSE. NOT /odom. NOT /odom_raw.

   /odom's origin is wherever the encoders were last zeroed. Drawing it here
   produces coordinates that are plausible, stable, smooth, and wrong by
   however far the odom origin sits from the arena origin — a silent constant
   offset that looks exactly like a working map. cloud/dashboard/ROS_PORT.md
   records that this has already cost this project a session. /odom_raw is
   subscribed by this dashboard as a DIAGNOSTIC READOUT only; it is never fed
   to this file.

   THE HONEST CONSEQUENCE: /fpms/mission/* is published while a mission runs,
   not on an idle heartbeat. PARKED, THERE IS NO LIVE POSE. This map then
   draws a hollow ghost at the start box labelled ASSUMED and says so. An
   assumed pose that admits it beats a confident pose in the wrong frame.

   GEOMETRY IS DERIVED, NEVER COPIED. /etc/fpms/zones.json in the image ships
   the three zone centres, and states in its own words why a consumer must not
   simply read them:

     > A consumer that reads cx_mm/cy_mm and uses them directly has created
     > the fifth independent copy of these numbers, which is the exact failure
     > the derivation-not-output rule exists to prevent.
     >
     > A consumer must recompute the centres from arena_mm, zone_side_frac and
     > zone_margin_frac and REFUSE this whole file if any published
     > cx_mm/cy_mm disagrees by more than 0.05 mm.

   So this file derives everything from three numbers — arena_mm and the two
   fractions — exactly as fpms_missions.py, arena.ts and make_arena_map.py all
   do, and `validateZones()` below implements the refusal. A missing zones.json
   does nothing at all: built-in constants, no note, no fault, which is what
   the file itself asks for.

   ZONE NAMES ARE BY CORNER, DELIBERATELY. zones.json records the trap: the
   zone STRAIGHT AHEAD of the start box (top-right) is mission `m2`, and the
   FAR one (top-left) is `m1`, so an operator saying "Zone 1" means the code's
   `m2`. Nothing is renumbered — renaming ids would silently change what every
   stored command and log line means — so every operator-facing string here
   names the CORNER, which is the one description that cannot be read two ways.
   ========================================================================= */
(function (global) {
"use strict";

/* The three numbers everything else comes from. */
var ARENA_MM = 1200.0, ZONE_FRAC = 0.30, MARGIN_FRAC = 0.04;
var ZONE = ZONE_FRAC * ARENA_MM;        // 360
var MARGIN = MARGIN_FRAC * ARENA_MM;    // 48

/* Rects anchored bottom-left, in arena mm. Ids match zones.json exactly so
   the two can be diffed field for field. */
var ZONES = [
  { id: "zone-a", label: "ZONE A", corner: "TOP-LEFT", mission: "m1", role: "FIRE",
    x: MARGIN,                   y: ARENA_MM - MARGIN - ZONE, c: "#ff4b70" },
  { id: "zone-b", label: "ZONE B", corner: "TOP-RIGHT", mission: "m2", role: "FIRE",
    x: ARENA_MM - MARGIN - ZONE, y: ARENA_MM - MARGIN - ZONE, c: "#ffbf00" },
  { id: "water-station", label: "WATER", corner: "BOTTOM-LEFT", mission: "water", role: "REFILL",
    x: MARGIN,                   y: MARGIN,                   c: "#00b7ff" }
];
ZONES.forEach(function (z) { z.cx = z.x + ZONE / 2; z.cy = z.y + ZONE / 2; });

/* The start box. Bottom-right member of the same four-corner family, but NOT
   a zone: zones are destinations, this is where the operator is asked to
   PLACE the rover. 90 deg because +y is up the arena and psi is CCW from +x. */
var HOME = { x: ARENA_MM - MARGIN - ZONE / 2, y: MARGIN + ZONE / 2, hdg: 90 };  // 972, 228

/* The refusal zones.json asks for. Returns {ok, checked, problems[]}.
   Called by app.js when a copy of zones.json is reachable; a failure is shown
   to the operator and the BUILT-IN derivation is kept, because a zone 100 mm
   from where it should be is a perfectly plausible number that drives the
   rover to the wrong place, silently and forever. */
function validateZones(j) {
  var problems = [], checked = 0;
  var TOL = 0.05;   // the same tolerance test_stack.py uses on the 744 mm gate
  try {
    var a = j.arena || {};
    if (Math.abs((a.arena_mm || 0) - ARENA_MM) > TOL) {
      problems.push("arena_mm " + a.arena_mm + " != " + ARENA_MM);
    }
    if (Math.abs((a.zone_side_frac || 0) - ZONE_FRAC) > 1e-9) {
      problems.push("zone_side_frac " + a.zone_side_frac + " != " + ZONE_FRAC);
    }
    if (Math.abs((a.zone_margin_frac || 0) - MARGIN_FRAC) > 1e-9) {
      problems.push("zone_margin_frac " + a.zone_margin_frac + " != " + MARGIN_FRAC);
    }
    ZONES.forEach(function (z) {
      var p = (j.zones || {})[z.id];
      if (!p) { problems.push("zones." + z.id + " missing"); return; }
      checked++;
      if (Math.abs(p.cx_mm - z.cx) > TOL || Math.abs(p.cy_mm - z.cy) > TOL) {
        problems.push(z.id + " centre published (" + p.cx_mm + "," + p.cy_mm +
                      ") but derives to (" + z.cx + "," + z.cy + ")");
      }
    });
    var sp = j.start_pose || {};
    checked++;
    if (Math.abs(sp.x_mm - HOME.x) > TOL || Math.abs(sp.y_mm - HOME.y) > TOL ||
        Math.abs(sp.heading_deg - HOME.hdg) > TOL) {
      problems.push("start_pose published (" + sp.x_mm + "," + sp.y_mm + "," +
                    sp.heading_deg + ") but derives to (" + HOME.x + "," +
                    HOME.y + "," + HOME.hdg + ")");
    }
  } catch (e) {
    problems.push("unreadable: " + e);
  }
  return { ok: problems.length === 0, checked: checked, problems: problems };
}

var VIEW_MIN = -180, VIEW_MAX = ARENA_MM + 180;    // a little apron round the arena
var ROBOT_W = 230, ROBOT_NOSE = 160, ROBOT_TAIL = -80;
var FRONT_STOP_MM = 400;                           // config.env FPMS_MISSION_FRONT_STOP_MM

function Arena(gridCanvas, dynCanvas) {
  this.gc = gridCanvas; this.dc = dynCanvas;
  this.g = gridCanvas.getContext("2d");
  this.d = dynCanvas.getContext("2d");
  this.scale = 1; this.ox = 0; this.oy = 0;
  this.trail = [];        // [{x,y}] recent MEASURED poses only
  this.resize();
}

Arena.prototype.resize = function () {
  var box = this.gc.parentNode.getBoundingClientRect();
  var w = Math.max(1, Math.floor(box.width)), h = Math.max(1, Math.floor(box.height));
  this.gc.width = this.dc.width = w;
  this.gc.height = this.dc.height = h;
  var span = VIEW_MAX - VIEW_MIN;
  this.scale = Math.min(w / span, h / span) * 0.96;
  this.ox = w / 2 - (VIEW_MIN + VIEW_MAX) / 2 * this.scale;
  this.oy = h / 2 + (VIEW_MIN + VIEW_MAX) / 2 * this.scale;
  this.drawStatic();
};

/* arena mm -> canvas px. The y flip is the bottom-left origin, and it lives
   here and nowhere else. */
Arena.prototype.px = function (x, y) {
  return [this.ox + x * this.scale, this.oy - y * this.scale];
};

/* robot frame (forward, left) -> arena mm. The ONLY place a heading is
   applied to geometry, so there is exactly one place to be wrong. */
Arena.prototype.toWorld = function (pose, fwd, left) {
  var h = pose.hdg * Math.PI / 180, c = Math.cos(h), s = Math.sin(h);
  return [pose.x + fwd * c - left * s, pose.y + fwd * s + left * c];
};

Arena.prototype.drawStatic = function () {
  var g = this.g, self = this;
  g.clearRect(0, 0, this.gc.width, this.gc.height);

  var tl = this.px(VIEW_MIN, VIEW_MAX), br = this.px(VIEW_MAX, VIEW_MIN);

  /* 100 mm grid, 500 mm emphasised. */
  for (var v = Math.ceil(VIEW_MIN / 100) * 100; v <= VIEW_MAX; v += 100) {
    var pv = this.px(v, v);
    g.strokeStyle = (v % 500) ? "rgba(80,190,255,.10)" : "rgba(80,190,255,.26)";
    g.lineWidth = 1;
    g.beginPath(); g.moveTo(pv[0], tl[1]); g.lineTo(pv[0], br[1]); g.stroke();
    g.beginPath(); g.moveTo(tl[0], pv[1]); g.lineTo(br[0], pv[1]); g.stroke();
  }

  /* Arena boundary. */
  var a0 = this.px(0, ARENA_MM), a1 = this.px(ARENA_MM, 0);
  g.fillStyle = "rgba(0,255,136,.04)";
  g.fillRect(a0[0], a0[1], a1[0] - a0[0], a1[1] - a0[1]);
  g.strokeStyle = "#00ff88"; g.lineWidth = 3;
  g.strokeRect(a0[0], a0[1], a1[0] - a0[0], a1[1] - a0[1]);

  g.font = "bold 11px Consolas, monospace"; g.textAlign = "center";
  g.fillStyle = "#00ff88";
  g.fillText(ARENA_MM + " mm  (1.20 m)", (a0[0] + a1[0]) / 2, a1[1] + 16);

  /* Origin marker — the single most misread thing on any arena map. */
  var o = this.px(0, 0);
  g.strokeStyle = "#00ff88"; g.lineWidth = 2;
  g.beginPath(); g.moveTo(o[0], o[1]); g.lineTo(o[0] + 34, o[1]); g.stroke();
  g.beginPath(); g.moveTo(o[0], o[1]); g.lineTo(o[0], o[1] - 34); g.stroke();
  g.textAlign = "left"; g.fillStyle = "#7fe8b0"; g.font = "10px Consolas, monospace";
  g.fillText("0,0 (bottom-left)", o[0] + 6, o[1] + 14);
  g.fillText("+x", o[0] + 36, o[1] + 4);
  g.fillText("+y", o[0] - 4, o[1] - 38);

  /* The three zones. */
  g.font = "bold 10px Consolas, monospace"; g.textAlign = "center";
  ZONES.forEach(function (z) {
    var p0 = self.px(z.x, z.y + ZONE), p1 = self.px(z.x + ZONE, z.y);
    g.strokeStyle = z.c; g.lineWidth = 2; g.setLineDash([6, 5]);
    g.strokeRect(p0[0], p0[1], p1[0] - p0[0], p1[1] - p0[1]);
    g.setLineDash([]);
    g.fillStyle = z.c;
    /* CORNER first, mission id second and in brackets — see the header. An
       operator reads the corner and cannot misread it; the mission id is
       there only to connect what they see to what the logs say. */
    g.fillText(z.label + " · " + z.corner, (p0[0] + p1[0]) / 2, p0[1] - 6);
    g.font = "9px Consolas, monospace";
    g.fillText("(" + z.mission + ")  centre " + Math.round(z.cx) + "," + Math.round(z.cy),
               (p0[0] + p1[0]) / 2, p1[1] - 6);
    g.font = "bold 10px Consolas, monospace";
  });

  /* Start box / home. */
  var hp = this.px(HOME.x, HOME.y);
  g.strokeStyle = "#3af07a"; g.lineWidth = 2;
  g.beginPath(); g.arc(hp[0], hp[1], 13, 0, Math.PI * 2); g.stroke();
  g.fillStyle = "#3af07a";
  g.fillText("START · BOTTOM-RIGHT " + Math.round(HOME.x) + "," + Math.round(HOME.y) +
             " @" + HOME.hdg + "°", hp[0], hp[1] - 18);
  g.textAlign = "left";
};

/*  st = {
      pose:      {x,y,hdg} | null   FRESH mission pose, or null
      poseStale: bool               we have one but it aged out
      scan:      LaserScan | null   null the instant it is stale — never held
      frontMm:   number | null
      route:     [{x,y,wp}]
    }
    Nothing in here decides freshness; app.js has already applied the feed
    registry and hands this function only what may be drawn. */
Arena.prototype.drawDynamic = function (st) {
  var d = this.d, self = this;
  d.clearRect(0, 0, this.dc.width, this.dc.height);

  var measured = !!st.pose && !st.poseStale;
  var pose = st.pose || { x: HOME.x, y: HOME.y, hdg: HOME.hdg };

  /* --- planned route, under everything -------------------------------- */
  if (st.route && st.route.length > 1) {
    d.lineJoin = d.lineCap = "round";
    d.beginPath();
    st.route.forEach(function (p, i) {
      var q = self.px(p.x, p.y);
      if (i) { d.lineTo(q[0], q[1]); } else { d.moveTo(q[0], q[1]); }
    });
    d.strokeStyle = "rgba(58,240,122,.55)"; d.lineWidth = 2.5; d.stroke();
    st.route.forEach(function (p) {
      if (!p.wp) { return; }
      var q = self.px(p.x, p.y);
      d.strokeStyle = "#3af07a"; d.lineWidth = 2;
      d.beginPath(); d.arc(q[0], q[1], 6, 0, Math.PI * 2); d.stroke();
      d.fillStyle = "#3af07a"; d.font = "bold 10px Consolas, monospace";
      d.fillText(p.wp, q[0] + 9, q[1] - 7);
    });
  }

  /* --- trail of MEASURED poses. Never a assumed one. ------------------ */
  if (measured) {
    var last = this.trail[this.trail.length - 1];
    if (!last || Math.hypot(last.x - pose.x, last.y - pose.y) > 8) {
      this.trail.push({ x: pose.x, y: pose.y });
      if (this.trail.length > 600) { this.trail.shift(); }
    }
  }
  if (this.trail.length > 1) {
    d.beginPath();
    this.trail.forEach(function (p, i) {
      var q = self.px(p.x, p.y);
      if (i) { d.lineTo(q[0], q[1]); } else { d.moveTo(q[0], q[1]); }
    });
    d.strokeStyle = "rgba(0,183,255,.42)"; d.lineWidth = 1.5; d.stroke();
  }

  /* --- LiDAR returns --------------------------------------------------
     `st.scan` is null the moment the feed goes stale, so the map physically
     cannot hold a frame: no dots IS the honest picture of a dead scanner.
     Placing the returns needs a pose, and an ASSUMED pose would put real
     obstacle geometry at a made-up place — so with no measured pose we draw
     no returns at all and the panel says why. */
  if (st.scan && measured) {
    var R = st.scan.ranges || [];
    var a0 = st.scan.angle_min, ai = st.scan.angle_increment;
    var rmax = st.scan.range_max || 12.0;
    d.fillStyle = "rgb(240,210,255)";
    for (var i = 0; i < R.length; i++) {
      var r = R[i];
      /* 0 and inf are "no return", not a wall at the origin. */
      if (!(r > 0.02) || r > rmax || !isFinite(r)) { continue; }
      var th = a0 + i * ai;
      var w = this.toWorld(pose, r * 1000 * Math.cos(th), r * 1000 * Math.sin(th));
      var q = this.px(w[0], w[1]);
      d.fillRect(q[0] - 1.5, q[1] - 1.5, 3, 3);
    }
  }

  /* --- front clearance arcs: WHY it stopped --------------------------- */
  if (measured) {
    var drawArc = function (mm, col, dash, label) {
      var w = self.toWorld(pose, mm, 0);
      var c = self.px(pose.x, pose.y), e = self.px(w[0], w[1]);
      var h = pose.hdg * Math.PI / 180;
      d.strokeStyle = col; d.lineWidth = 2; d.setLineDash(dash);
      d.beginPath();
      d.arc(c[0], c[1], Math.hypot(e[0] - c[0], e[1] - c[1]), -h - 0.45, -h + 0.45);
      d.stroke(); d.setLineDash([]);
      d.fillStyle = col; d.font = "bold 10px Consolas, monospace";
      d.fillText(label, e[0] + 6, e[1]);
    };
    drawArc(FRONT_STOP_MM, "#ffbf00", [7, 6], "STOP @" + FRONT_STOP_MM);
    if (st.frontMm !== null && st.frontMm > 0 && st.frontMm < 4000) {
      drawArc(st.frontMm, st.frontMm <= FRONT_STOP_MM ? "#ff4b70" : "#00b7ff", [],
              "front " + st.frontMm.toFixed(0));
    }
  }

  /* --- the rover ------------------------------------------------------
     SOLID + cyan = a measured arena pose.
     HOLLOW + dashed + grey at the start box = ASSUMED. There is no third
     rendering, and the difference is visible from across a room. */
  var body = [[ROBOT_NOSE, ROBOT_W / 2], [ROBOT_NOSE, -ROBOT_W / 2],
              [ROBOT_TAIL, -ROBOT_W / 2], [ROBOT_TAIL, ROBOT_W / 2]];
  d.beginPath();
  body.forEach(function (v, i) {
    var w = self.toWorld(pose, v[0], v[1]), q = self.px(w[0], w[1]);
    if (i) { d.lineTo(q[0], q[1]); } else { d.moveTo(q[0], q[1]); }
  });
  d.closePath();
  if (measured) {
    d.fillStyle = "rgba(0,183,255,.30)"; d.fill();
    d.strokeStyle = "#00b7ff"; d.lineWidth = 2; d.stroke();
  } else {
    d.setLineDash([5, 4]);
    d.strokeStyle = "rgba(150,160,175,.85)"; d.lineWidth = 2; d.stroke();
    d.setLineDash([]);
  }

  /* Heading spike. */
  var nose = this.toWorld(pose, ROBOT_NOSE + 90, 0);
  var ctr = this.px(pose.x, pose.y), np = this.px(nose[0], nose[1]);
  d.strokeStyle = measured ? "#00b7ff" : "rgba(150,160,175,.85)";
  d.lineWidth = 2;
  d.beginPath(); d.moveTo(ctr[0], ctr[1]); d.lineTo(np[0], np[1]); d.stroke();

  /* The badge. It is on the map, not only in a side panel, because the map
     is what an operator looks at while the rover is moving. */
  d.font = "bold 11px Consolas, monospace"; d.textAlign = "center";
  if (measured) {
    d.fillStyle = "#00b7ff";
    d.fillText("MEASURED  " + Math.round(pose.x) + "," + Math.round(pose.y) +
               "  " + pose.hdg.toFixed(0) + "°", ctr[0], ctr[1] - 26);
  } else {
    d.fillStyle = "#ffbf00";
    d.fillText(st.pose ? "ASSUMED — pose STALE" : "ASSUMED — no live pose",
               ctr[0], ctr[1] - 26);
  }
  d.textAlign = "left";
};

Arena.prototype.clearTrail = function () { this.trail = []; };

global.Arena = Arena;
global.ARENA = {
  ARENA_MM: ARENA_MM, ZONE_FRAC: ZONE_FRAC, MARGIN_FRAC: MARGIN_FRAC,
  ZONE: ZONE, MARGIN: MARGIN, ZONES: ZONES,
  HOME: HOME, FRONT_STOP_MM: FRONT_STOP_MM,
  validateZones: validateZones
};

})(window);

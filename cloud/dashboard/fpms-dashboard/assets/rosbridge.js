/* =========================================================================
   rosbridge.js — the connection layer, and the reason this dashboard exists.
   =========================================================================

   THE HEADLINE REQUIREMENT IS "IT MUST NEVER LOSE THE CONNECTION."
   That is not one feature. It is six, and each covers a failure the other
   five do not:

     1. RECONNECT FOREVER, with exponential backoff + jitter.
        Backoff so a dead Pi is not hammered; jitter so two open tabs (or the
        main and STOP sockets in ONE tab) do not synchronise into a thundering
        pair that retry in lockstep forever.

     2. BACKOFF RESETS ONLY AFTER THE LINK HAS BEEN HEALTHY FOR `stableMs`.
        The obvious version — "reset backoff in onopen" — is wrong, and the
        existing console has that bug. A rosbridge that accepts a socket and
        drops it 200 ms later produces open/close/open/close at the base delay
        forever. Health is measured in seconds survived, not in one callback.

     3. A CONNECT TIMEOUT.
        A WebSocket to a host that has gone away (Pi rebooted, wifi roamed,
        route black-holed) sits in readyState CONNECTING with NO EVENT until
        the OS TCP stack gives up — which can be 75 s or more, and on some
        stacks longer. For that whole time the UI would say "connecting" and
        nothing would be retried. We give up after `connectTimeoutMs` and
        start a fresh attempt.

     4. A HEARTBEAT THAT DETECTS A *SILENTLY DEAD* SOCKET.
        readyState OPEN proves nothing: a NAT box that dropped the flow, a
        suspended laptop, a wedged server — all leave a socket that is OPEN
        and will never deliver another byte, and whose close event may never
        arrive. So: whenever the socket has been QUIET for `quietS`, we send a
        probe and require an answer within `probeTimeoutS`. No answer ->
        the socket is dead; close it ourselves and reconnect.

        THE PROBE IS DELIBERATELY AN UNKNOWN OP, `{op:"__fpms_ping__", id}`.
        rosbridge's Protocol.incoming() pulls `id` off every inbound message
        BEFORE dispatch and, for an op it does not recognise, replies
        `{op:"status", level:"error", id:<our id>}`. That means the probe:
          * is answered by a stock, unmodified rosbridge_server;
          * depends on NO topic and NO service, so it is unaffected by
            topics_glob and services_glob — this dashboard depends on nothing
            outside the whitelist, and that includes its own heartbeat;
          * has no side effect on the ROS graph whatsoever.
        A /rosapi/get_time call would have been the tidier-looking choice and
        is the wrong one: it depends on services_glob, which is "[/fpms/*]"
        here, and a probe that a safety whitelist can silently swallow is a
        probe that will report a healthy link as dead at a competition.

        We only probe WHEN THE LINK IS ALREADY QUIET. Under normal load
        (/scan_lidar at 10 Hz) no probe is ever sent, so the Pi's journal
        stays clean; the "Unknown operation" lines appear exactly when the
        link went quiet, which is when you want a trace of it.

     5. AUTOMATIC RE-ADVERTISE AND RE-SUBSCRIBE ON EVERY OPEN.
        rosbridge keeps NO state across a dropped socket. Anything not
        replayed on reconnect is gone, silently — the subscription simply
        never delivers again and nothing errors. Every advertise and every
        subscribe is recorded and replayed on every open, reconnect included.

     6. THE WEDGE DETECTOR — see `wedgeS` below. This one is FPMS-specific
        and is documented at length in cloud/dashboard/ROS_PORT.md:

          > rosbridge only delivers topics whose PUBLISHER already existed
          > when rosbridge started. A publisher created afterwards is never
          > discovered, and the client subscription asking for it is accepted
          > and then silent forever.

        Symptom: connected, subscribed, silent, no error anywhere. It is the
        worst failure mode in this stack because every layer reports healthy.
        A client cannot fix it — reconnecting the BROWSER does not help,
        because the fault is in the SERVER's discovery state. So we detect it,
        say so in plain words, and name the actual remedy (restart
        fpms-rosbridge on the Pi) instead of silently retrying forever.

   Everything above is implemented against the rosbridge v2 protocol directly.
   There is no roslibjs here for the same reason there is none in
   fpms_console/console.html: the protocol is five JSON messages, and the
   alternative is either a CDN (no internet at the venue) or a minified blob
   nobody can debug at 2am.
   ========================================================================= */
(function (global) {
"use strict";

var DEFAULTS = {
  backoffBaseMs:     300,    // first retry is fast — a blip should be invisible
  backoffFactor:     1.7,
  backoffCapMs:      10000,  // never wait longer than this to try again
  backoffFloorMs:    250,    // jitter must never produce a hammering retry
  connectTimeoutMs:  12000,  // see (3) above
  stableMs:          5000,   // survive this long before backoff resets
  quietS:            4.0,    // silence that earns a probe
  probeTimeoutS:     3.0,    // probe unanswered this long => socket is dead
  wedgeS:            12.0,   // socket alive, zero topic data => wedged
  superviseMs:       1000    // the invariant checker
};

/* Connection phases, in the order an operator meets them. `live` is the only
   one that means data is arriving; `open` means the socket is up but nothing
   has been delivered yet, and those two must never be shown as the same
   thing. */
var PHASE = {
  INIT:       "init",
  CONNECTING: "connecting",
  OPEN:       "open",        // socket up, no topic data seen yet
  LIVE:       "live",        // socket up AND topic data flowing
  WEDGED:     "wedged",      // socket provably alive, topic data absent
  BACKOFF:    "backoff",     // waiting to retry
  OFFLINE:    "offline"      // the browser says it has no network at all
};

function now() { return performance.now(); }

function RosLink(opts) {
  if (!(this instanceof RosLink)) { return new RosLink(opts); }
  opts = opts || {};

  var cfg = {};
  for (var k in DEFAULTS) { cfg[k] = (opts[k] !== undefined) ? opts[k] : DEFAULTS[k]; }

  this.url      = opts.url;
  this.label    = opts.label || "link";
  this.cfg      = cfg;
  this.onstate  = opts.onstate || function () {};
  this.onlog    = opts.onlog   || function () {};
  /* A link with no subscriptions (the STOP link) must never be judged
     "wedged" for having no topic data — it asked for none. */
  this.expectsData = opts.expectsData !== false;

  this.ws        = null;
  this.gen       = 0;        // invalidates callbacks from superseded sockets
  this.phase     = PHASE.INIT;
  this.phaseAt   = now();

  this.subs      = [];       // [{topic, type, queue_length, throttle_rate}]
  this.advs      = [];       // [{topic, type}]
  this.handlers  = {};       // topic -> cb

  this.attempts     = 0;
  this.backoffMs    = cfg.backoffBaseMs;
  this.retryTimer   = null;
  this.retryDueAt   = 0;
  this.connectDueAt = 0;

  this.openedAt     = 0;
  this.lastFrameAt  = 0;     // ANY inbound frame (incl. status)
  this.lastDataAt   = 0;     // op:"publish" only — real topic data
  this.probeId      = null;
  this.probeSentAt  = 0;
  this.wedgeRemedy  = 0;     // 0 none tried, 1 subs cycled, 2 socket forced
  this.counters     = { opens: 0, closes: 0, probes: 0, probeFails: 0,
                        frames: 0, data: 0, wedges: 0 };

  var self = this;
  this.superviseTimer = setInterval(function () { self._supervise(); }, cfg.superviseMs);

  /* Browser lifecycle. Timers in a hidden tab are throttled to as little as
     once per minute, so a backgrounded dashboard can sit on a 10 s backoff
     for far longer than 10 s. Every one of these events is a reason to stop
     waiting and try immediately. */
  function wake(why) {
    return function () {
      if (self.phase === PHASE.BACKOFF || self.phase === PHASE.OFFLINE) {
        self._log("wake (" + why + ") — retrying now");
        self.reconnectNow();
      }
    };
  }
  global.addEventListener("online", wake("browser online"));
  global.addEventListener("focus",  wake("window focus"));
  global.addEventListener("pageshow", wake("page shown"));
  global.document.addEventListener("visibilitychange", function () {
    if (!global.document.hidden) { wake("tab visible")(); }
  });
  global.addEventListener("offline", function () {
    /* navigator.onLine is a hint, not a fact — it is true on a wifi with no
       route. We surface it and keep retrying anyway. */
    self._log("browser reports OFFLINE (still retrying)");
  });

  this._connect();
}

/* --------------------------------------------------------------- internals */

RosLink.prototype._log = function (msg) {
  try { this.onlog("[" + this.label + "] " + msg); } catch (e) { /* never throw into the link */ }
};

RosLink.prototype._setPhase = function (p, why) {
  if (this.phase === p) { return; }
  this.phase = p;
  this.phaseAt = now();
  if (why) { this._log(p.toUpperCase() + " — " + why); }
  try { this.onstate(this.snapshot()); } catch (e) { /* a UI bug must not kill the link */ }
};

RosLink.prototype._send = function (obj) {
  var ws = this.ws;
  if (ws && ws.readyState === 1) {
    try { ws.send(JSON.stringify(obj)); return true; }
    catch (e) { return false; }
  }
  return false;
};

RosLink.prototype._connect = function () {
  var self = this;
  var gen = ++this.gen;

  if (this.retryTimer) { clearTimeout(this.retryTimer); this.retryTimer = null; }
  this.retryDueAt = 0;
  this.attempts++;

  var ws;
  try {
    ws = new WebSocket(this.url);
  } catch (e) {
    /* Constructor throws on a malformed URL or a mixed-content block. Both
       are permanent-ish, but retrying costs nothing and a typo corrected in
       the URL bar should recover without a page reload. */
    this._log("WebSocket ctor failed: " + e);
    this._scheduleRetry("constructor threw");
    return;
  }

  this.ws = ws;
  this.connectDueAt = now() + this.cfg.connectTimeoutMs;
  this._setPhase(PHASE.CONNECTING, "dialing " + this.url + " (attempt " + this.attempts + ")");

  ws.onopen = function () {
    if (gen !== self.gen) { try { ws.close(); } catch (e) {} return; }
    self.counters.opens++;
    self.openedAt = now();
    self.lastFrameAt = now();
    self.lastDataAt = 0;
    self.probeId = null;
    self.wedgeRemedy = 0;
    self.connectDueAt = 0;
    self._setPhase(PHASE.OPEN, "socket open");

    /* Replay EVERYTHING. rosbridge has no memory of the previous socket. */
    self.advs.forEach(function (a) {
      self._send({ op: "advertise", topic: a.topic, type: a.type });
    });
    self.subs.forEach(function (s) {
      self._send(assign({ op: "subscribe" }, s));
    });
    if (self.advs.length || self.subs.length) {
      self._log("replayed " + self.advs.length + " advertise / " +
                self.subs.length + " subscribe");
    }
  };

  ws.onmessage = function (ev) {
    if (gen !== self.gen) { return; }
    self.lastFrameAt = now();
    self.counters.frames++;

    var m;
    try { m = JSON.parse(ev.data); } catch (e) { return; }

    /* Any frame bearing our probe id proves the application layer is alive —
       whether it answered with a status error (the expected reply to an
       unknown op) or anything else. */
    if (self.probeId && m.id === self.probeId) {
      self.probeId = null;
      return;
    }

    if (m.op === "publish") {
      self.counters.data++;
      self.lastDataAt = now();
      if (self.phase === PHASE.OPEN || self.phase === PHASE.WEDGED) {
        self._setPhase(PHASE.LIVE, "topic data flowing");
      }
      var h = self.handlers[m.topic];
      if (h) {
        try { h(m.msg); }
        catch (e) { self._log("handler error on " + m.topic + ": " + e); }
      }
    } else if (m.op === "status") {
      if (m.level === "error" || m.level === "warning") {
        self._log("rosbridge " + m.level + ": " + m.msg);
      }
    }
  };

  ws.onerror = function () {
    if (gen !== self.gen) { return; }
    /* An error event is always followed by a close event in every browser
       that matters, but forcing the close makes that guaranteed rather than
       assumed — and a socket stuck without either is exactly the hang the
       connect timeout exists for. */
    try { ws.close(); } catch (e) {}
  };

  ws.onclose = function (ev) {
    if (gen !== self.gen) { return; }
    self.counters.closes++;
    self.ws = null;
    var heldMs = self.openedAt ? (now() - self.openedAt) : 0;
    self.openedAt = 0;
    self._scheduleRetry("socket closed" +
      (ev && ev.code ? " (code " + ev.code + ")" : "") +
      (heldMs ? " after " + (heldMs / 1000).toFixed(1) + "s up" : " before opening"));
  };
};

RosLink.prototype._scheduleRetry = function (why) {
  var self = this;
  if (this.retryTimer) { return; }   // exactly one retry in flight, ever

  /* HALF-JITTER, not full jitter. Full jitter (`random() * delay`) can draw
     near-zero repeatedly and turn a backoff into a hammer; half-jitter keeps
     the mean growth while still decorrelating two sockets. */
  var d = this.backoffMs;
  var wait = Math.max(this.cfg.backoffFloorMs, d / 2 + Math.random() * (d / 2));
  this.backoffMs = Math.min(this.cfg.backoffCapMs, d * this.cfg.backoffFactor);

  this.retryDueAt = now() + wait;
  this._setPhase(
    (global.navigator && global.navigator.onLine === false) ? PHASE.OFFLINE : PHASE.BACKOFF,
    why + " — retry in " + (wait / 1000).toFixed(1) + "s");

  this.retryTimer = setTimeout(function () {
    self.retryTimer = null;
    self._connect();
  }, wait);
};

/* The invariant checker. Everything here is a belt to some other braces: if
   any single mechanism above fails to fire, this notices within a second.
   A dashboard that must never lose its link cannot depend on one timer. */
RosLink.prototype._supervise = function () {
  var t = now(), cfg = this.cfg;

  /* (a) Stuck in CONNECTING with no event from the socket. */
  if (this.phase === PHASE.CONNECTING && this.connectDueAt && t > this.connectDueAt) {
    this._log("connect timed out after " + (cfg.connectTimeoutMs / 1000) + "s");
    this.gen++;                                  // orphan the hung socket
    try { if (this.ws) { this.ws.close(); } } catch (e) {}
    this.ws = null;
    this.connectDueAt = 0;
    this._scheduleRetry("connect timeout");
    return;
  }

  /* (b) We believe we are waiting, but no retry is pending. Cannot normally
         happen; if it ever does, the dashboard is dead forever, so check. */
  if ((this.phase === PHASE.BACKOFF || this.phase === PHASE.OFFLINE) && !this.retryTimer) {
    this._log("no retry was pending — scheduling one");
    this._scheduleRetry("supervisor rescue");
    return;
  }

  if (!this.ws || this.ws.readyState !== 1) { return; }

  /* (c) Backoff resets on sustained health, not on a single onopen. */
  if (this.openedAt && (t - this.openedAt) > cfg.stableMs && this.backoffMs !== cfg.backoffBaseMs) {
    this.backoffMs = cfg.backoffBaseMs;
    this.attempts = 0;
    this._log("link stable for " + (cfg.stableMs / 1000) + "s — backoff reset");
  }

  /* (d) HEARTBEAT. Quiet socket -> probe -> demand an answer. */
  var quietS = (t - this.lastFrameAt) / 1000;
  if (this.probeId) {
    if ((t - this.probeSentAt) / 1000 > cfg.probeTimeoutS) {
      this.counters.probeFails++;
      this._log("HEARTBEAT LOST — no reply in " + cfg.probeTimeoutS +
                "s. The socket is open and dead. Reconnecting.");
      this.probeId = null;
      this.gen++;
      try { this.ws.close(); } catch (e) {}
      this.ws = null;
      this.openedAt = 0;
      this._scheduleRetry("heartbeat timeout");
      return;
    }
  } else if (quietS > cfg.quietS) {
    this.counters.probes++;
    this.probeId = this.label + "-hb-" + this.counters.probes;
    this.probeSentAt = t;
    /* Unknown op on purpose — see the header. Guaranteed reply, no glob
       dependency, no effect on the ROS graph. */
    if (!this._send({ op: "__fpms_ping__", id: this.probeId })) {
      this.probeId = null;
    }
  }

  /* (e) WEDGE DETECTOR. Socket demonstrably alive (frames within quietS, or a
         probe answered) yet no topic data at all. This is the ROS_PORT.md
         failure: rosbridge accepted the subscription and will never deliver
         it, because the publisher appeared after rosbridge started. */
  if (this.expectsData && this.subs.length && this.openedAt) {
    var since = this.lastDataAt ? (t - this.lastDataAt) : (t - this.openedAt);
    if (since / 1000 > cfg.wedgeS) {
      if (this.phase !== PHASE.WEDGED) {
        this.counters.wedges++;
        this._setPhase(PHASE.WEDGED,
          "socket alive but ZERO topic data for " + (since / 1000).toFixed(0) + "s");
      }
      /* Remedy 1: cycle the subscriptions. Cheap, safe, and fixes the lesser
         version of this (a subscription rosbridge dropped on its side). */
      if (this.wedgeRemedy === 0) {
        this.wedgeRemedy = 1;
        this._log("wedge remedy 1/2 — cycling every subscription");
        this._cycleSubs();
      } else if (this.wedgeRemedy === 1 && since / 1000 > cfg.wedgeS * 2) {
        /* Remedy 2: a fresh socket. Unlikely to help — the fault is in the
           SERVER's discovery state, not ours — but it is free and it rules
           the client out. */
        this.wedgeRemedy = 2;
        this._log("wedge remedy 2/2 — forcing a fresh socket");
        this.reconnectNow();
      } else if (this.wedgeRemedy === 2 && since / 1000 > cfg.wedgeS * 3) {
        this.wedgeRemedy = 3;
        this._log("WEDGE PERSISTS. This is not fixable from the browser. " +
                  "On the Pi: sudo systemctl restart fpms-rosbridge");
      }
    }
  }
};

RosLink.prototype._cycleSubs = function () {
  var self = this;
  this.subs.forEach(function (s) { self._send({ op: "unsubscribe", topic: s.topic }); });
  setTimeout(function () {
    self.subs.forEach(function (s) { self._send(assign({ op: "subscribe" }, s)); });
  }, 300);
};

/* ------------------------------------------------------------------ public */

/* Record then send. The record is what makes reconnect work, so it happens
   whether or not the socket is up right now. */
RosLink.prototype.subscribe = function (topic, type, cb, opt) {
  opt = opt || {};
  var s = { topic: topic, queue_length: (opt.queue_length !== undefined ? opt.queue_length : 1) };
  if (type) { s.type = type; }
  if (opt.throttle) { s.throttle_rate = opt.throttle; }
  this.subs.push(s);
  this.handlers[topic] = cb;
  this._send(assign({ op: "subscribe" }, s));
  return this;
};

RosLink.prototype.advertise = function (topic, type) {
  this.advs.push({ topic: topic, type: type });
  this._send({ op: "advertise", topic: topic, type: type });
  return this;
};

/* Returns TRUE only if the bytes were handed to an OPEN socket. The STOP
   button keys off this to decide whether to fall back, so it must never
   optimistically report success. */
RosLink.prototype.publish = function (topic, msg) {
  return this._send({ op: "publish", topic: topic, msg: msg });
};

RosLink.prototype.reconnectNow = function () {
  if (this.retryTimer) { clearTimeout(this.retryTimer); this.retryTimer = null; }
  this.gen++;
  try { if (this.ws) { this.ws.close(); } } catch (e) {}
  this.ws = null;
  this.openedAt = 0;
  this.probeId = null;
  this._connect();
};

RosLink.prototype.isOpen = function () {
  return !!(this.ws && this.ws.readyState === 1);
};

RosLink.prototype.snapshot = function () {
  var t = now();
  return {
    label:        this.label,
    url:          this.url,
    phase:        this.phase,
    phaseAgeS:    (t - this.phaseAt) / 1000,
    open:         this.isOpen(),
    attempts:     this.attempts,
    retryInS:     this.retryDueAt ? Math.max(0, (this.retryDueAt - t) / 1000) : null,
    frameAgeS:    this.lastFrameAt ? (t - this.lastFrameAt) / 1000 : null,
    dataAgeS:     this.lastDataAt  ? (t - this.lastDataAt)  / 1000 : null,
    upS:          this.openedAt ? (t - this.openedAt) / 1000 : 0,
    probing:      !!this.probeId,
    wedgeRemedy:  this.wedgeRemedy,
    counters:     this.counters
  };
};

function assign(a, b) {
  for (var k in b) { if (Object.prototype.hasOwnProperty.call(b, k)) { a[k] = b[k]; } }
  return a;
}

global.RosLink = RosLink;
global.RosLink.PHASE = PHASE;

})(window);

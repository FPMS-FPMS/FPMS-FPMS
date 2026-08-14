#!/usr/bin/env python3
"""fpms-cored — the FPMS rover's command spine.

This process does three jobs that nothing else on the rover was doing, and it
deliberately does NOT do a fourth: it never actuates. There is no /cmd_vel
publisher and no /cmd_duty publisher in this file. The only motion-adjacent
thing it can emit is a STOP, which is safe by construction.

  1. STOP AUTHORITY.   An always-listening, never-blocked path from an operator
                       clicking STOP to the rover's wheels being at zero.
  2. COMMAND RECEIPTS. Every command that arrives gets an immediate, visible
                       receipt, so an operator never has to wonder whether a
                       click landed.
  3. HEALTH.           One aggregated verdict about whether the stack is up,
                       which topics are fresh, and which units are running.

-----------------------------------------------------------------------------
WHY THIS EXISTS: the three ways a click used to vanish
-----------------------------------------------------------------------------
1. `fpms_missions.py` answers ONE verb, `mission`. Its dispatch ends with
   `if action != "mission": return` — an unknown verb produced no reply of any
   kind. From the dashboard, a command nobody implemented and a command that
   worked look identical.
2. `stop`/`estop`/`auto_off` are handled with an explicit "ACT, DO NOT ANSWER"
   policy. Correct for the executor — a stop must never wait to be
   acknowledged — but it left the operator with no confirmation at all for the
   single most safety-critical button on the page.
3. Everything — acks, nacks, telemetry — is dropped on the floor when the
   broker link is down (`if not self.connected: return`), at any QoS.

-----------------------------------------------------------------------------
HOW "STOP IS ALWAYS HONOURED" IS ENFORCED (mechanism, not intention)
-----------------------------------------------------------------------------
STOP gets its OWN MQTT client, on its OWN socket, with its OWN network thread,
subscribed to NOTHING BUT the three stop verbs.

That is the whole trick, and it is worth being precise about why it works.
paho delivers every message for a client on a single network thread, in order.
In `fpms_missions.py` that same thread also runs `on_lidar` (a 360-bin
occupancy-grid integration) and `_cmd_mission` (which holds a lock across an
A* search). Neither is time-bounded. A stop arriving behind either of them
waits for it. The executor's own docstring acknowledges the risk.

Here, nothing else can ever be in front of a stop, because nothing else is
subscribed on that client. The callback itself does no work beyond setting a
`threading.Event` and appending to a deque — no locks, no JSON parsing that can
throw past the latch, no I/O. A pre-started worker thread does the fan-out.
So the latch is set in microseconds no matter what else the process is doing.

Then the stop is fanned out over FOUR independent paths, because the whole
point is to not depend on any single one surviving:

  a. Re-publish `commands/stop` at QoS 1. Covers the executor having missed
     the original (a QoS 0 dashboard publish, a reconnect race).
  b. Publish ROS `/estop` (std_msgs/Bool, latched RELIABLE + TRANSIENT_LOCAL).
     Firmware v3 cuts the motors on this in its control task, independently of
     the Pi's mission logic. Harmless on factory firmware: nobody subscribes,
     so it is a no-op rather than a hazard.
  c. Publish `events/stop_asserted` so the dashboard can show STOP LATCHED
     without waiting for the executor to agree.
  d. ESCALATION. If `telemetry/mission` still reports a driving phase
     STOP_ESCALATE_S after the stop, SIGTERM `fpms-missions`. Its signal
     handler aborts the mission and publishes ten zero Twists before exiting,
     and systemd restarts it. This is the answer to "the executor is wedged" —
     the one case a) through c) cannot cover, because all three assume the
     executor is still running its loop.

Escalation is the only destructive action in this file. It is bounded (once
per stop), it is logged loudly, and it is exactly the tool you want when a
rover is moving and the process that should have stopped it is not responding.

-----------------------------------------------------------------------------
WHAT THIS PROCESS MUST NEVER DO
-----------------------------------------------------------------------------
* Never publish /cmd_vel or /cmd_duty. Not even zeros. Zeros from a second
  writer are how the "stall-then-lurch" interleaving happened: 2 Hz zeros mixed
  into a 20 Hz setpoint. The executor owns the wire; this process owns the
  stop latch, and those are different things.
* Never arm. Never start a mission.
* Never weaken a guard. It adds a supervisor; it removes nothing.
* NEVER FEED ITSELF. Path (a) publishes to a topic this process is subscribed
  to. That ring is not a hypothetical: on the first run it produced 3394
  commands/estop, 3375 commands/stop, 6773 events/nack and 2671
  events/stop_asserted in ten seconds, and took the micro-ROS link down.
  Five layered guards now bound it — see "loop guards" in the config section.
  The rule they encode: A STOP IS AN EVENT, NOT A STREAM. Anything in this
  file that can publish in response to something it receives must be able to
  say what bounds it.
"""
from __future__ import annotations

import collections
import json
import os
import signal
import subprocess
import sys
import threading
import time
import uuid

from paho.mqtt.client import Client, CallbackAPIVersion

# ----------------------------------------------------------------- config ---


def load_config(path="/etc/fpms/config.env"):
    """Read /etc/fpms/config.env.

    Every other FPMS process reads config from this file and NOT from the
    environment, which has burned this project before: a `FPMS_*` set in a
    systemd unit looks like it should work and silently does nothing. This
    process honours BOTH, with the environment winning, so that a one-off
    override during a session is possible without editing a root-owned file.
    """
    cfg = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    cfg[k.strip()] = v.strip()
    except FileNotFoundError:
        pass
    for k, v in os.environ.items():
        if k.startswith("FPMS_"):
            cfg[k] = v
    return cfg


CFG = load_config()
THING = CFG.get("FPMS_THING_NAME", "rover2")
BROKER = CFG.get("FPMS_MQTT_HOST", "127.0.0.1")
PORT = int(CFG.get("FPMS_MQTT_PORT", "1883"))
USER = CFG.get("FPMS_MQTT_USER", "")
PASS = CFG.get("FPMS_MQTT_PASS", "")

STOP_VERBS = ("stop", "estop", "auto_off")

# How long the executor gets to visibly stop before we SIGTERM it. Long enough
# that a normal stop (which lands in <= ~50 ms and then settles) never trips it;
# short enough that a wedged executor does not get to keep driving.
STOP_ESCALATE_S = float(CFG.get("FPMS_CORE_STOP_ESCALATE_S", "3.0"))
STOP_ESCALATE_ENABLED = CFG.get("FPMS_CORE_STOP_ESCALATE", "1") not in ("0", "false", "no")
# How old `telemetry/mission` may be and still count as evidence that the rover
# is driving. Sized from the executor's publish cadence (TELEM_HZ = 2 Hz), NOT
# from STOP_ESCALATE_S — see _escalate_if_still_driving for why conflating the
# two made the escalation unreachable.
STOP_EVIDENCE_MAX_AGE_S = float(CFG.get("FPMS_CORE_STOP_EVIDENCE_MAX_AGE_S", "2.5"))
# Publishing ROS /estop is harmless on factory firmware (no subscriber) and is
# the strongest available stop on firmware v3. Default on.
ESTOP_ROS = CFG.get("FPMS_CORE_ESTOP_ROS", "1") not in ("0", "false", "no")

# ------------------------------------------------------------ loop guards ---
#
# WHY THESE EXIST. Path (a) re-publishes `commands/<verb>` — the exact topic
# the stop client subscribes to. On the first run of this daemon that closed a
# ring: relay -> own subscription -> latch -> relay, at network speed. Measured
# 3394 commands/estop + 3375 commands/stop + 6773 events/nack +
# 2671 events/stop_asserted in TEN SECONDS, which took the micro-ROS link down
# with it.
#
# The fix is layered on purpose. Each layer alone stops the storm; together
# they make it arithmetically impossible for any payload shape to produce an
# unbounded stream:
#
#   1. SELF-ORIGIN FILTER  — drop our own relay in the hot path (bytes, no JSON).
#   2. CMD_ID DEDUP        — a repeated cmd_id is not a new stop.
#   3. RELAY RATE LIMIT    — our own re-publication is spaced and capped.
#   4. LATCH CAP           — a hard ceiling on acted-on stops per window.
#   5. RECEIPT CAP         — replies can never become a cascade.
#
# A stop is an EVENT, not a stream. Every ceiling below is far above what any
# real operator action produces and far below anything that could be called a
# flood.

# Marker bytes our own relay always carries. Checked as a byte substring rather
# than parsed, so the hot path keeps its "nothing that can block or throw before
# the latch" property. Both markers are required so that a genuine payload that
# merely mentions this daemon by name is not mistaken for our own relay —
# a false positive here would DROP a real stop, which is the dangerous
# direction, so the test is deliberately conservative.
SELF_SOURCE = "fpms-cored"
SELF_MARK = b"fpms-cored"
RELAY_MARK = b'"relay"'

# Hard ceiling on stops acted upon per window. A human cannot click faster than
# this; a loop can.
STOP_MAX_PER_WINDOW = int(CFG.get("FPMS_CORE_STOP_MAX_PER_WINDOW", "12"))
STOP_WINDOW_S = float(CFG.get("FPMS_CORE_STOP_WINDOW_S", "10.0"))
# Minimum spacing between two relays of the SAME verb. The relay covers a lost
# message; repeating it faster than the executor's control loop adds traffic
# and nothing else.
RELAY_MIN_INTERVAL_S = float(CFG.get("FPMS_CORE_RELAY_MIN_INTERVAL_S", "1.0"))
RELAY_MAX_PER_WINDOW = int(CFG.get("FPMS_CORE_RELAY_MAX_PER_WINDOW", "8"))
RELAY_WINDOW_S = float(CFG.get("FPMS_CORE_RELAY_WINDOW_S", "10.0"))
# Receipts are evidence, not telemetry.
RECEIPT_MAX_PER_WINDOW = int(CFG.get("FPMS_CORE_RECEIPT_MAX_PER_WINDOW", "60"))
RECEIPT_WINDOW_S = float(CFG.get("FPMS_CORE_RECEIPT_WINDOW_S", "10.0"))

HEALTH_HZ = float(CFG.get("FPMS_CORE_HEALTH_HZ", "1.0"))
# A command with no owner is answered after this long. It has to be a timeout
# rather than an instant answer: the owning service may be mid-restart, and
# "nobody answered in 2s" is true where "nobody is subscribed right now" is a
# race the operator should not have to think about.
OWNER_TIMEOUT_S = float(CFG.get("FPMS_CORE_OWNER_TIMEOUT_S", "2.5"))

# Units the stack expects to be running. Health reports on exactly these.
UNITS = tuple(u for u in CFG.get(
    "FPMS_CORE_UNITS",
    "micro-ros-agent,fpms-rover-agent,fpms-lidar-ros,fpms-odom-tf,"
    "fpms-tf,fpms-missions,fpms-cored"
).split(",") if u.strip())

# Topics whose freshness is part of the stack verdict, and how old is too old.
FRESHNESS = {
    "telemetry/lidar": float(CFG.get("FPMS_CORE_FRESH_LIDAR_S", "3.0")),
    "telemetry/mission": float(CFG.get("FPMS_CORE_FRESH_MISSION_S", "5.0")),
    "telemetry/pose": float(CFG.get("FPMS_CORE_FRESH_POSE_S", "20.0")),
}

RUNNING = threading.Event()
RUNNING.set()


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


# ------------------------------------------------------------------- state ---

class RateLimit:
    """Fixed-window counter. O(1), allocation-free, safe on the hot path.

    A fixed window (rather than a token bucket or sliding window) is chosen so
    that `allow()` is a compare and two increments with no data structure to
    walk — it is called from the stop client's network thread, where anything
    that can take an unbounded amount of time is a defect.
    """

    def __init__(self, limit, window_s, name=""):
        self.limit = limit
        self.window_s = window_s
        self.name = name
        self._start = 0.0
        self._n = 0
        self.dropped = 0
        self.notified = False

    def allow(self):
        now = time.monotonic()
        if now - self._start >= self.window_s:
            self._start = now
            self._n = 0
            self.notified = False
        if self._n < self.limit:
            self._n += 1
            return True
        self.dropped += 1
        return False


class StopState:
    """The stop latch, and everything needed to prove it was honoured.

    Deliberately tiny and lock-free on the hot path. `assert_stop` is called
    from the stop client's network thread and must be safe to call at any time,
    from any state, including during shutdown.
    """

    def __init__(self):
        self.event = threading.Event()          # signals the fan-out worker
        self.pending = collections.deque(maxlen=64)
        self.last_stop_mono = 0.0
        self.count = 0
        self.escalations = 0
        self.latched = False
        self.suppressed = 0
        self.self_dropped = 0
        self.limit = RateLimit(STOP_MAX_PER_WINDOW, STOP_WINDOW_S, "latch")

    def assert_stop(self, verb, payload, topic):
        # Everything here is O(1) and cannot block. No JSON, no locks, no I/O.
        #
        # LAYER 4: the hard ceiling. Note what is and is not sacrificed when it
        # trips. `latched` is still set and `last_stop_mono` still advances, so
        # the rover stays STOPPED and the latch keeps reading as asserted — we
        # decline to re-run the FAN-OUT, never the stop itself. Dropping
        # fan-out work under a flood is safe; dropping the latch would not be.
        self.latched = True
        self.last_stop_mono = time.monotonic()
        if not self.limit.allow():
            self.suppressed += 1
            return False
        self.count += 1
        self.pending.append((verb, payload, topic, time.time()))
        self.event.set()
        return True


STOP = StopState()

# Loop guards, module level so `health` can report them and a test can inspect
# them without standing up a broker.
RELAY_LIMIT = RateLimit(RELAY_MAX_PER_WINDOW, RELAY_WINDOW_S, "relay")
RECEIPT_LIMIT = RateLimit(RECEIPT_MAX_PER_WINDOW, RECEIPT_WINDOW_S, "receipt")
LAST_RELAY = {}                    # verb -> monotonic of last relay published
RECENT_STOP_IDS = collections.OrderedDict()   # cmd_id -> True, bounded
RECENT_CMD_IDS = collections.OrderedDict()    # cmd_id -> True, bounded

# Freshness bookkeeping for the health verdict.
LAST_SEEN = {}                     # topic suffix -> monotonic
LAST_MISSION = {"phase": None, "driving": False, "ts": 0.0, "armed": None}
OWNERS = {}                        # verb -> {"svc":..., "seen": mono}
PENDING = collections.OrderedDict()  # cmd_id -> receipt dict
PENDING_LOCK = threading.Lock()


# -------------------------------------------------------------------- bus ---

class Bus:
    """One paho client. Deliberately one job each — see the module docstring."""

    def __init__(self, role, on_message=None, will=False):
        self.role = role
        self.client = Client(client_id=f"{THING}-core-{role}",
                             callback_api_version=CallbackAPIVersion.VERSION2)
        if USER:
            self.client.username_pw_set(USER, PASS or None)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        if on_message:
            self.client.on_message = on_message
        if will:
            self.client.will_set(
                f"fpms/{THING}/events/offline",
                json.dumps({"thing": THING, "svc": "cored", "status": "offline",
                            "reason": "unexpected disconnect"}),
                qos=1, retain=False)
        self.connected = False
        self.subs = []

    def subscribe(self, topic, qos=1):
        self.subs.append((topic, qos))

    def start(self):
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        try:
            self.client.connect_async(BROKER, PORT, keepalive=15)
        except Exception as e:
            log(f"[{self.role}] connect_async failed: {e}")
        self.client.loop_start()

    def _on_connect(self, client, _u, _f, rc, _p=None):
        self.connected = True
        log(f"[{self.role}] connected to {BROKER}:{PORT} ({rc})")
        for topic, qos in self.subs:
            client.subscribe(topic, qos=qos)

    def _on_disconnect(self, _c, _u, *a):
        self.connected = False
        log(f"[{self.role}] disconnected")

    def publish(self, suffix, payload, qos=0, retain=False):
        try:
            payload.setdefault("ts", time.time())
            payload.setdefault("thing", THING)
            self.client.publish(f"fpms/{THING}/{suffix}", json.dumps(payload),
                                qos=qos, retain=retain)
        except Exception as e:
            log(f"[{self.role}] publish {suffix} failed: {e}")


# ------------------------------------------------------------- ROS (estop) ---

class RosEstop:
    """Publishes /estop only. Optional; absence must never break the stop path.

    This is a separate, tiny class rather than a full ROS node because the ONE
    thing it must not do is make the stop path depend on rclpy being importable,
    on a DDS discovery race, or on the micro-ROS session being up. If any of
    that fails, `assert_stop` still works over MQTT and the escalation still
    fires — this is defence in depth, not the primary path.
    """

    def __init__(self):
        self.ok = False
        self.node = None
        self._pub = None
        self._rclpy = None
        if not ESTOP_ROS:
            log("ROS /estop publisher disabled by config")
            return
        try:
            import rclpy
            from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                                   DurabilityPolicy, HistoryPolicy)
            from std_msgs.msg import Bool
            rclpy.init(args=None)
            self._rclpy = rclpy
            self.node = rclpy.create_node("fpms_cored_estop")
            # TRANSIENT_LOCAL so a board or node that subscribes AFTER the stop
            # was asserted still receives it. A stop that only reaches whoever
            # happened to be listening at that instant is not a stop.
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
            self._pub = self.node.create_publisher(Bool, "/estop", qos)
            self._Bool = Bool
            self.ok = True
            log("ROS /estop publisher ready (harmless no-op on factory firmware)")
        except Exception as e:
            log(f"ROS /estop unavailable ({e}); MQTT stop path unaffected")

    def assert_stop(self):
        if not self.ok:
            return False
        try:
            m = self._Bool()
            m.data = True
            self._pub.publish(m)
            return True
        except Exception as e:
            log(f"/estop publish failed: {e}")
            return False

    def shutdown(self):
        try:
            if self.node is not None:
                self.node.destroy_node()
            if self._rclpy is not None:
                self._rclpy.shutdown()
        except Exception:
            pass


# --------------------------------------------------------------- receipts ---

def receipt(bus, cmd_id, verb, state, **extra):
    """Publish one command-receipt event.

    QoS 1 on purpose. A receipt is the operator's only evidence that a click
    was seen, so it is worth the extra round trip; the volume is a handful of
    messages per command, not a stream.

    LAYER 5. "A handful per command, not a stream" is an ASSERTION, so it is
    enforced rather than assumed. An unknown or duplicated verb must never be
    able to turn replies into a cascade — that is half of what the 6773
    events/nack in ten seconds actually were. Past the cap we publish exactly
    ONE suppression notice per window and then go quiet, because a flood of
    "you are flooding" messages is still a flood.
    """
    if not RECEIPT_LIMIT.allow():
        if not RECEIPT_LIMIT.notified:
            RECEIPT_LIMIT.notified = True
            try:
                bus.client.publish(
                    f"fpms/{THING}/events/command_receipt",
                    json.dumps({"svc": "cored", "state": "receipts_suppressed",
                                "reason": "receipt rate cap hit; replies "
                                          "suppressed to prevent a cascade",
                                "limit": RECEIPT_MAX_PER_WINDOW,
                                "window_s": RECEIPT_WINDOW_S,
                                "dropped": RECEIPT_LIMIT.dropped,
                                "ts": time.time(), "thing": THING}),
                    qos=1)
            except Exception:
                pass
        return
    p = {"cmd_id": cmd_id, "verb": verb, "state": state, "svc": "cored"}
    p.update(extra)
    bus.publish("events/command_receipt", p, qos=1)


def _cmd_id_for(payload):
    """Prefer a dashboard-supplied id so the UI can correlate its own click."""
    for key in ("cmd_id", "id", "request_id", "correlation_id"):
        v = payload.get(key) if isinstance(payload, dict) else None
        if isinstance(v, (str, int)) and str(v).strip():
            return str(v)
    return uuid.uuid4().hex[:12]


# -------------------------------------------------------- the STOP client ---

def on_stop_message(_c, _u, msg):
    """THE hot path. Must do the least work of any callback in this stack.

    No JSON parsing before the latch. No lock. No logging. If this function
    ever grows an operation that can block, the stop guarantee is gone, because
    paho runs it on the one thread that delivers stops.
    """
    raw = msg.payload or b""
    # LAYER 1 — THE LOOP GUARD, and the single most important line in the file.
    #
    # Path (a) re-publishes onto `commands/<verb>`, which is precisely what this
    # client is subscribed to. Without this test every relay comes straight back
    # in, is latched as a brand-new stop, and is relayed again — the ring that
    # produced 3394 commands/estop in ten seconds.
    #
    # A byte substring test, NOT json.loads, so this stays allocation-free and
    # cannot raise before the latch. That constraint is why the original
    # `source`/`relay` guard ended up on the command bus instead of here, where
    # it was actually needed.
    if SELF_MARK in raw and RELAY_MARK in raw:
        STOP.self_dropped += 1
        return
    verb = msg.topic.rsplit("/", 1)[-1]
    STOP.assert_stop(verb, raw, msg.topic)


def stop_worker(bus, estop):
    """Fan out every asserted stop. Started before the stop client connects."""
    while RUNNING.is_set():
        if not STOP.event.wait(timeout=0.25):
            continue
        STOP.event.clear()
        while STOP.pending:
            try:
                verb, raw, topic, wall = STOP.pending.popleft()
            except IndexError:
                break
            try:
                payload = json.loads((raw or b"").decode() or "{}")
                if not isinstance(payload, dict):
                    payload = {"value": payload}
            except Exception:
                payload = {}
            # LAYER 2 — belt and braces for layer 1. If a relay ever reaches
            # here (marker stripped by a broker rewrite, a payload we did not
            # author echoing our cmd_id back, a QoS 1 redelivery), the SAME
            # cmd_id is not a new stop and must not be fanned out again.
            #
            # Note a genuine operator stop with no cmd_id gets a fresh uuid from
            # _cmd_id_for, so real repeated clicks are still each honoured. That
            # is the intended asymmetry: identity means "already handled",
            # absence of identity means "assume it is new".
            payload_had_id = any(
                isinstance(payload.get(k), (str, int)) and str(payload.get(k)).strip()
                for k in ("cmd_id", "id", "request_id", "correlation_id"))
            cmd_id = _cmd_id_for(payload)
            if payload_had_id and cmd_id in RECENT_STOP_IDS:
                STOP.self_dropped += 1
                continue
            RECENT_STOP_IDS[cmd_id] = True
            while len(RECENT_STOP_IDS) > 256:
                RECENT_STOP_IDS.popitem(last=False)

            # Belt and braces for layer 1 again: if we somehow dequeued our own
            # relay, do not treat it as a new command.
            if payload.get("relay") and payload.get("source") == SELF_SOURCE:
                STOP.self_dropped += 1
                continue

            t0 = time.monotonic()

            # (c) tell the dashboard immediately — before any downstream agrees.
            receipt(bus, cmd_id, verb, "honoured",
                    detail="stop latched by fpms-cored",
                    latency_ms=round((time.monotonic() - t0) * 1000.0, 1),
                    received_ts=wall, topic=topic)
            bus.publish("events/stop_asserted", {
                "verb": verb, "cmd_id": cmd_id, "count": STOP.count,
                "received_ts": wall,
            }, qos=1)

            # (a) re-publish at QoS 1 so a missed QoS 0 original still lands.
            #     The executor treats a repeated stop as idempotent.
            #
            # LAYER 3 — spaced and capped. The relay exists to cover ONE lost
            # message; emitting it faster than the executor's control loop buys
            # nothing and is the raw material of a storm. Suppressing a relay
            # never weakens the stop: paths (b) /estop, (c) stop_asserted and
            # (d) escalation are untouched, and the executor has already been
            # told at least once.
            now_mono = time.monotonic()
            spaced = (now_mono - LAST_RELAY.get(verb, -1e9)) >= RELAY_MIN_INTERVAL_S
            if spaced and RELAY_LIMIT.allow():
                LAST_RELAY[verb] = now_mono
                bus.publish(f"commands/{verb}",
                            {"source": SELF_SOURCE, "relay": True,
                             "cmd_id": cmd_id, "origin_ts": wall},
                            qos=1)
            else:
                log(f"relay for {verb} suppressed (rate limit); stop itself "
                    f"is unaffected — /estop and escalation still ran")

            # (b) the independent hardware-adjacent path.
            sent = estop.assert_stop()

            log(f"STOP asserted (verb={verb} cmd_id={cmd_id} "
                f"estop_ros={'yes' if sent else 'no'})")

            # (d) escalation watchdog, one per stop.
            threading.Thread(target=_escalate_if_still_driving,
                             args=(bus, cmd_id, verb, time.monotonic()),
                             daemon=True).start()


def _escalate_if_still_driving(bus, cmd_id, verb, t_stop):
    """SIGTERM the executor if it is still driving after a stop.

    This is the only case paths (a)-(c) cannot cover: they all assume the
    executor is still running its control loop and will notice the latch. If it
    is wedged — stuck under a lock, in a blocking call, or spinning — the only
    remaining lever is the process itself.

    SIGTERM is the right signal and not a blunt one: fpms-missions installs a
    handler that aborts the running mission and publishes ten zero Twists
    before exiting, and its unit has Restart=always, so the escalation both
    stops the rover and brings the service back.
    """
    if not STOP_ESCALATE_ENABLED:
        return
    deadline = t_stop + STOP_ESCALATE_S
    while RUNNING.is_set() and time.monotonic() < deadline:
        time.sleep(0.1)
        # Cleared as soon as the executor reports a non-driving phase.
        if not LAST_MISSION.get("driving"):
            return
    if not LAST_MISSION.get("driving"):
        return
    # Only escalate on evidence that is itself fresh. If telemetry/mission has
    # gone quiet we do NOT know the rover is driving, and killing a service on
    # the strength of a stale reading is its own hazard.
    #
    # This bound is deliberately INDEPENDENT of STOP_ESCALATE_S. Tying the two
    # together was a bug: we wait STOP_ESCALATE_S before checking, so evidence
    # captured at stop time is already about that old when we look at it, and
    # comparing it against the same number made the check almost always fail —
    # the escalation could never fire. What matters is the telemetry CADENCE:
    # fpms_missions publishes telemetry/mission at TELEM_HZ = 2 Hz (0.5 s), and
    # throttles to IDLE_TELEM_S = 0.5 s when idle, so anything inside a couple
    # of seconds is current.
    age = time.time() - (LAST_MISSION.get("ts") or 0.0)
    if age > STOP_EVIDENCE_MAX_AGE_S:
        log(f"stop escalation skipped: mission telemetry is {age:.1f}s old, "
            "so 'still driving' is not established")
        receipt(bus, cmd_id, verb, "escalation_skipped",
                reason=f"mission telemetry stale ({age:.1f}s)")
        return

    STOP.escalations += 1
    log(f"!! STOP ESCALATION: fpms-missions still reports driving "
        f"{STOP_ESCALATE_S:.1f}s after stop — sending SIGTERM")
    receipt(bus, cmd_id, verb, "escalated",
            reason=f"still driving {STOP_ESCALATE_S:.1f}s after stop",
            action="SIGTERM fpms-missions")
    bus.publish("events/fault", {
        "component": "stop", "alert": True,
        "error": "executor did not stop; escalated to SIGTERM",
        "escalations": STOP.escalations,
    }, qos=1)
    # MUST match the sudoers rule deploy_stack.sh installs, byte for byte:
    #   <user> ALL=(root) NOPASSWD: /bin/systemctl kill -s SIGTERM fpms-missions.service
    # sudo matches on the literal argv, so "fpms-missions" and
    # "fpms-missions.service" are NOT the same rule, and a mismatch turns the
    # last-resort stop into a silent password prompt that never gets answered.
    # Running as root (no sudo needed) still works: sudo -n is a no-op there.
    cmd = ["sudo", "-n", "/bin/systemctl", "kill", "-s", "SIGTERM",
           "fpms-missions.service"]
    # getattr: os.geteuid does not exist on Windows, and test_stack.py runs
    # these paths on a laptop. Non-root is the correct assumption when we
    # cannot tell.
    if getattr(os, "geteuid", lambda: 1000)() == 0:
        cmd = cmd[2:]
    try:
        r = subprocess.run(cmd, timeout=5, check=False,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if r.returncode != 0:
            log(f"escalation SIGTERM failed rc={r.returncode}: "
                f"{(r.stderr or b'').decode(errors='replace').strip()}")
            log("  check /etc/sudoers.d/fpms-cored exists and matches this "
                "exact command (deploy_stack.sh installs it)")
    except Exception as e:
        log(f"escalation failed to signal fpms-missions: {e}")


# ------------------------------------------------ the command/receipt bus ---

def on_bus_message(_c, _u, msg):
    """Everything that is not a stop. Runs on the command client's thread.

    Note this thread is NOT the stop thread, so however long anything here
    takes, it cannot delay a stop.
    """
    try:
        suffix = msg.topic.split(f"fpms/{THING}/", 1)[-1]
    except Exception:
        return
    now_mono = time.monotonic()
    LAST_SEEN[suffix] = now_mono

    try:
        payload = json.loads(msg.payload.decode() or "{}")
        if not isinstance(payload, dict):
            payload = {"value": payload}
    except Exception:
        payload = {}

    if suffix == "telemetry/mission":
        _note_mission(payload)
        return
    if suffix.startswith("events/"):
        _note_event(suffix, payload)
        return
    if suffix.startswith("commands/"):
        _note_command(suffix, payload)
        return


def _note_mission(payload):
    phase = payload.get("phase")
    LAST_MISSION["phase"] = phase
    LAST_MISSION["armed"] = payload.get("armed")
    LAST_MISSION["ts"] = time.time()
    # "driving" is whatever the executor says it is. The set of idle-ish phases
    # is deliberately explicit: an UNKNOWN phase counts as driving, because the
    # failure we are guarding against is a rover that is moving while something
    # unexpected is going on.
    #
    # ABSENT is NOT unknown, and the difference is load-bearing. A missing or
    # empty `phase` is what idle telemetry looks like, so it must resolve to
    # NOT driving. Comparing `str(phase).lower()` against a set containing the
    # OBJECT None never matched, because the string is "none" — so a missing
    # phase counted as driving and could have escalated a stop into a spurious
    # SIGTERM of a perfectly healthy executor. Normalise first, then compare.
    if phase is None:
        norm = ""
    else:
        norm = str(phase).strip().lower()
    idle = {"", "none", "null", "idle", "done", "aborted", "abort", "error",
            "ready", "cooldown", "complete", "completed", "stopped", "failed",
            # STATIONARY phases the executor really publishes
            # (fpms_missions.py:4318). Without these a healthy executor
            # reads as "driving" and gets SIGTERMed 3s after a stop.
            "planning", "hold", "aborting"}
    LAST_MISSION["driving"] = norm not in idle


def _note_event(suffix, payload):
    """Learn who owns which verb, and close out pending receipts."""
    kind = suffix.rsplit("/", 1)[-1]

    if kind == "online":
        svc = payload.get("svc") or "unknown"
        for verb in list(payload.get("actions") or []):
            OWNERS[str(verb)] = {"svc": svc, "seen": time.monotonic()}
        for verb in list(payload.get("acts_silently_on") or []):
            OWNERS[str(verb)] = {"svc": svc, "seen": time.monotonic(),
                                 "silent": True}
        return

    if kind not in ("ack", "nack"):
        return

    # fpms_missions.py does not echo a cmd_id, so correlation is by verb and
    # recency. This is stated plainly rather than hidden: the receipt carries
    # `correlated: "by_verb_recency"` so nobody later mistakes it for an exact
    # match. It is right in every real single-operator case and can only be
    # ambiguous if two commands of the same verb are in flight at once.
    verb = str(payload.get("action") or "mission")
    with PENDING_LOCK:
        for cmd_id in reversed(list(PENDING)):
            rec = PENDING[cmd_id]
            if rec["verb"] != verb or rec.get("closed"):
                continue
            rec["closed"] = True
            rec["state"] = "acked" if kind == "ack" else "nacked"
            rec["reason"] = payload.get("error") or payload.get("detail") or ""
            rec["by"] = payload.get("svc") or "fpms-missions"
            rec["correlated"] = "by_verb_recency"
            bus = BUSES.get("cmd")
            if bus is not None:
                receipt(bus, cmd_id, verb, rec["state"],
                        reason=rec["reason"], by=rec["by"],
                        correlated=rec["correlated"],
                        round_trip_ms=round(
                            (time.monotonic() - rec["mono"]) * 1000.0, 1))
            break


def _note_command(suffix, payload):
    """Immediate receipt for EVERY command, owned or not."""
    verb = suffix.rsplit("/", 1)[-1]
    if verb in STOP_VERBS:
        return                      # the stop client already answered for these
    if payload.get("relay") and payload.get("source") == SELF_SOURCE:
        return                      # our own re-publish; do not receipt it twice

    cmd_id = _cmd_id_for(payload)
    # A cmd_id we have already answered is a duplicate delivery, not a new
    # command, and must not produce a second receipt. Without this, a client
    # retrying at QoS 1 gets an answer per retry.
    if cmd_id in RECENT_CMD_IDS:
        return
    RECENT_CMD_IDS[cmd_id] = True
    while len(RECENT_CMD_IDS) > 512:
        RECENT_CMD_IDS.popitem(last=False)

    owner = OWNERS.get(verb)
    rec = {"verb": verb, "mono": time.monotonic(), "closed": False,
           "state": "received", "owner": (owner or {}).get("svc")}
    with PENDING_LOCK:
        PENDING[cmd_id] = rec
        while len(PENDING) > 128:
            PENDING.popitem(last=False)

    receipt(BUSES["cmd"], cmd_id, verb, "received",
            owner=rec["owner"], known_verb=bool(owner),
            payload_echo={k: v for k, v in list(payload.items())[:12]})


def _close_out_reaper():
    """Answer for unowned and owned-but-silent commands, from ONE thread.

    This used to be `threading.Thread(...).start()` per command. Under the
    storm that meant thousands of live threads, each sleeping OWNER_TIMEOUT_S
    and then publishing — an unbounded reply cascade built out of an unbounded
    number of threads, and the other half of the 6773 events/nack.

    One reaper cannot grow, does the same job, and is bounded by the size of
    PENDING (128) no matter how much traffic arrives.
    """
    while RUNNING.is_set():
        time.sleep(0.25)
        now = time.monotonic()
        due = []
        with PENDING_LOCK:
            for cmd_id, rec in list(PENDING.items()):
                if rec.get("closed") or now - rec["mono"] < OWNER_TIMEOUT_S:
                    continue
                rec["closed"] = True
                verb = rec["verb"]
                owner = OWNERS.get(verb)
                if owner is None:
                    rec["state"] = "no_owner"
                    reason = (f"no service on this rover claims the verb "
                              f"{verb!r}. Known verbs: "
                              f"{sorted(OWNERS) or 'none announced yet'}")
                elif owner.get("silent"):
                    rec["state"] = "acted_silently"
                    reason = (f"{owner['svc']} acts on {verb!r} without "
                              "replying (by design)")
                else:
                    rec["state"] = "timeout"
                    reason = (f"{owner['svc']} claims {verb!r} but did not "
                              f"answer within {OWNER_TIMEOUT_S:.1f}s")
                due.append((cmd_id, verb, rec["state"], reason))
        bus = BUSES.get("cmd")
        if bus is None:
            continue
        for cmd_id, verb, state, reason in due:
            receipt(bus, cmd_id, verb, state, reason=reason)


# ----------------------------------------------------------------- health ---

def _unit_active(unit):
    try:
        r = subprocess.run(["systemctl", "is-active", unit],
                           capture_output=True, text=True, timeout=4)
        return r.stdout.strip()
    except Exception:
        return "unknown"


def health_loop(bus):
    """One aggregated verdict, on a slow cadence, on its own thread.

    Runs `systemctl is-active` per unit, which forks. That is exactly why this
    is not on the stop thread and not on the command thread: a fork storm under
    memory pressure can take hundreds of milliseconds, and neither of those
    threads may ever wait that long.
    """
    period = 1.0 / max(HEALTH_HZ, 0.05)
    unit_cache, unit_checked = {}, 0.0
    while RUNNING.is_set():
        now_mono, now_wall = time.monotonic(), time.time()

        # Units are polled at most every 5 s; the fork cost is not worth 1 Hz.
        if now_mono - unit_checked > 5.0:
            unit_cache = {u: _unit_active(u) for u in UNITS}
            unit_checked = now_mono

        feeds = {}
        for suffix, limit in FRESHNESS.items():
            seen = LAST_SEEN.get(suffix)
            age = (now_mono - seen) if seen else None
            feeds[suffix] = {
                "age_s": round(age, 2) if age is not None else None,
                "limit_s": limit,
                # `None` (never seen) is NOT fresh. The old code's habit of
                # treating "no data yet" as benign is how a feed that never
                # started looked identical to one that was fine.
                "fresh": bool(age is not None and age <= limit),
            }

        degraded = [u for u, s in unit_cache.items() if s != "active"]
        stale = [t for t, f in feeds.items() if not f["fresh"]]
        verdict = "ok" if not degraded and not stale else (
            "degraded" if not degraded else "faulted")

        bus.publish("telemetry/health", {
            "svc": "cored",
            "verdict": verdict,
            "units": unit_cache,
            "units_degraded": degraded,
            "feeds": feeds,
            "feeds_stale": stale,
            "stop": {
                "latched": STOP.latched,
                "count": STOP.count,
                "escalations": STOP.escalations,
                "since_last_s": (round(now_mono - STOP.last_stop_mono, 1)
                                 if STOP.last_stop_mono else None),
                "escalate_after_s": STOP_ESCALATE_S if STOP_ESCALATE_ENABLED else None,
            },
            # Loop-guard telemetry. If any of these climb while the rover is
            # idle, something is feeding the stop path and it is visible here
            # BEFORE it becomes a storm — which is exactly what was missing
            # the first time.
            "loop_guard": {
                "self_dropped": STOP.self_dropped,
                "latch_suppressed": STOP.suppressed,
                "relay_dropped": RELAY_LIMIT.dropped,
                "receipt_dropped": RECEIPT_LIMIT.dropped,
                "pending": len(PENDING),
            },
            "mission": dict(LAST_MISSION),
            "owners": {v: o.get("svc") for v, o in OWNERS.items()},
            "uptime_s": round(now_wall - STARTED, 1),
        }, qos=0)

        for _ in range(int(period * 10)):
            if not RUNNING.is_set():
                return
            time.sleep(0.1)


# ------------------------------------------------------------------- main ---

BUSES = {}
STARTED = time.time()


def main():
    def stop_sig(*_):
        log("shutting down")
        RUNNING.clear()
    signal.signal(signal.SIGTERM, stop_sig)
    signal.signal(signal.SIGINT, stop_sig)

    log(f"fpms-cored starting — thing={THING} broker={BROKER}:{PORT}")
    log(f"stop escalation: {'on' if STOP_ESCALATE_ENABLED else 'OFF'} "
        f"({STOP_ESCALATE_S:.1f}s)")

    estop = RosEstop()

    # ORDER MATTERS. The stop worker is running before the stop client is even
    # connected, so there is no window in which a stop can be received and have
    # nothing ready to act on it.
    cmd_bus = Bus("cmd", on_message=on_bus_message, will=True)
    stop_bus = Bus("stop", on_message=on_stop_message)
    BUSES["cmd"] = cmd_bus
    BUSES["stop"] = stop_bus

    threading.Thread(target=stop_worker, args=(cmd_bus, estop),
                     daemon=True, name="stop").start()

    # The stop client subscribes to NOTHING except the stop verbs. This is the
    # guarantee; do not add a subscription here for convenience.
    for verb in STOP_VERBS:
        stop_bus.subscribe(f"fpms/{THING}/commands/{verb}", qos=1)

    cmd_bus.subscribe(f"fpms/{THING}/commands/#", qos=1)
    cmd_bus.subscribe(f"fpms/{THING}/events/+", qos=1)
    cmd_bus.subscribe(f"fpms/{THING}/telemetry/mission", qos=0)
    cmd_bus.subscribe(f"fpms/{THING}/telemetry/lidar", qos=0)
    cmd_bus.subscribe(f"fpms/{THING}/telemetry/pose", qos=0)

    stop_bus.start()
    cmd_bus.start()

    # Statically seed the verbs we know the stack owns, so a command issued
    # before fpms-missions has announced itself still gets a truthful receipt
    # rather than "no owner".
    for verb in ("mission",):
        OWNERS.setdefault(verb, {"svc": "fpms-missions", "seen": 0.0})
    for verb in ("set_coordinate",):
        OWNERS.setdefault(verb, {"svc": "fpms-missions", "seen": 0.0,
                                 "silent": True})

    cmd_bus.publish("events/online", {
        "svc": "cored", "status": "online",
        "role": "stop authority + command receipts + health",
        "stop_verbs": list(STOP_VERBS),
        "receipts_on": "events/command_receipt",
        "health_on": "telemetry/health",
        "escalation": {"enabled": STOP_ESCALATE_ENABLED,
                       "after_s": STOP_ESCALATE_S,
                       "action": "SIGTERM fpms-missions"},
        "estop_ros": estop.ok,
        "never_publishes": ["/cmd_vel", "/cmd_duty"],
    }, qos=1)

    threading.Thread(target=health_loop, args=(cmd_bus,),
                     daemon=True, name="health").start()
    # One reaper for every pending command, instead of a thread per command.
    threading.Thread(target=_close_out_reaper,
                     daemon=True, name="closeout").start()

    while RUNNING.is_set():
        time.sleep(0.5)

    cmd_bus.publish("events/offline", {"svc": "cored", "status": "offline",
                                       "reason": "clean shutdown"}, qos=1)
    time.sleep(0.3)
    for b in (stop_bus, cmd_bus):
        try:
            b.client.loop_stop()
            b.client.disconnect()
        except Exception:
            pass
    estop.shutdown()
    log("stopped")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""FPMS Foxglove command bridge — ROS 2 <-> the rover's existing MQTT verbs.

Runs ON THE PI (systemd: fpms-foxglove-cmd.service). It exists so that Foxglove
Studio, which speaks only ROS topics and ROS services, can drive the command
surface the rover already has, WITHOUT changing one line of fpms_missions.py,
fpms_teleop.py, fpms-rover-agent or fpms_cored.py.

===========================================================================
THE ONE SAFETY PROPERTY THAT MATTERS: THIS NODE CANNOT MOVE THE ROVER
===========================================================================
It creates ZERO publishers on any actuation topic. There is no `/cmd_vel`
publisher, no `/cmd_duty` publisher, no `/estop` publisher, no `/cmd_enable`
publisher anywhere in this file, and `_assert_no_actuators()` fails the node at
startup if one is ever added. Its entire power is: publish a JSON string to a
`fpms/<thing>/commands/<verb>` MQTT topic on the LOCAL broker. Everything that
can actually turn a wheel is downstream of that, unchanged, with all of its own
refusals (arm, link, battery, LiDAR, arena bounds) intact.

`/estop` is SUBSCRIBED here, never published. fpms_cored.py already owns the ROS
`/estop` publisher and TF_TREE.md's one-writer-per-thing discipline is the rule
this file follows.

===========================================================================
STOP
===========================================================================
STOP is a TOPIC SUBSCRIPTION (`/estop`, and `/fpms/cmd/stop`), never a service.
That is deliberate and it is the whole design:

  * A service call is request/response. Foxglove disables the button while a
    call is in flight, the call needs a live server, and it can time out. A
    panic button that can be "busy" is not a panic button.
  * A topic publish is fire-and-forget down an already-open TCP WebSocket. It
    cannot be refused, cannot block, and needs no reply to have worked.

When Foxglove publishes `/estop {data: true}` it fans out to THREE independent
consumers, none of which depends on the other two:

  1. The ESP32 firmware v3 subscribes `/estop` directly and latches all four
     motors to zero. (NO-OP TODAY: the board is on Yahboom stock firmware,
     which has no /estop. See the README.)
  2. THIS node mirrors it to MQTT `commands/estop` + `commands/stop` at QoS 1,
     which fpms_missions aborts on and fpms_teleop zero-Twists on.
  3. fpms-cored's dedicated stop client sees those verbs, re-publishes at
     QoS 1, asserts ROS `/estop` itself, and SIGTERMs fpms-missions if the
     executor is still reporting a driving phase 3 s later.

Inside this node the stop path is hardened the same way fpms_cored's is:
  * a DEDICATED MQTT client (`stop`) that subscribes to NOTHING, so its
    outbound queue can never be sitting behind a telemetry mirror publish;
  * its own MutuallyExclusiveCallbackGroup under a MultiThreadedExecutor, so a
    slow service handler cannot serialise ahead of it (the default rclpy
    single-threaded executor WOULD have done exactly that);
  * no JSON parsing, no locking, no state check before the publish;
  * no gate on arm / link / battery / mission state. Ever.

===========================================================================
RUN REQUIRES A DELIBERATE SECOND ACTION
===========================================================================
Two independent interlocks, and neither one is this file being careful:

  1. ROVER-SIDE (the real one, not ours): fpms_missions REQUIRE_ARM refuses any
     mission until `{"arm": true}` has been sent, auto-disarms on any stop or
     abort and after ARM_TIMEOUT_S unused.
  2. BRIDGE-SIDE (defence in depth): /fpms/run_m2 refuses unless /fpms/arm was
     called with data:true within ARM_WINDOW_S (default 60 s) and not since
     consumed. This still holds even on a bench where somebody has set
     FPMS_MISSION_REQUIRE_ARM=0.

  3. TRANSPORT-SIDE (the strongest): foxglove_bridge is started with
     `client_topic_whitelist` limited to /estop and /fpms/cmd/*, so the
     WebSocket is PHYSICALLY INCAPABLE of publishing /cmd_vel or /cmd_duty. A
     stray click in Foxglove cannot drive the rover because there is no wire
     from Foxglove to a drive topic at all.

===========================================================================
TELEMETRY MIRROR — why this node also republishes MQTT into ROS
===========================================================================
Foxglove plots numbers off ROS topics. Mission phase, per-leg residual and the
arm latch only exist on MQTT. So the mirror turns them into small typed ROS
topics under /fpms/**. It is one-way and read-only; nothing here can be
commanded by mirroring.

===========================================================================
FRESHNESS — the "stale but plausible" bug, structurally
===========================================================================
This project has twice been fooled by telemetry that looked alive while the
link was dead: a frozen 11 Hz rate read as healthy, a LiDAR panel that held its
last frame for minutes. That failure has TWO distinct causes which the old
dashboard could not tell apart:

  (a) the SOURCE stopped producing on the Pi, or
  (b) the LINK stopped delivering to the laptop.

Foxglove answers (b) by itself and structurally: the WebSocket is TCP, so when
it dies Studio's connection state goes to disconnected and says so; and every
message carries its real receive time, so a Plot against the timestamp axis
draws a flat line the instant a source stops while the axis keeps scrolling.
There is no code path in Studio that invents a frame.

This node answers (a): it publishes `/diagnostics` (diagnostic_msgs/
DiagnosticArray) at 2 Hz with the age and measured rate of every source AS
SEEN ON THE PI. If /scan_lidar is 40 s old, the Pi says so, in a message whose
own arrival proves the link is fine. Age computed at the source plus arrival
time measured at the sink is the pair that makes a lie impossible.

===========================================================================
ROS_DOMAIN_ID
===========================================================================
The board publishes on domain 20 and nowhere else. Every ad-hoc `ros2` command
in this project's history that forgot that saw an empty topic list and read it
as a dead link. This node logs its effective domain at startup and screams if
it is not 20. So does the systemd unit, which sets it.
"""

import json
import os
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Empty, Float32, Int32, String
from std_srvs.srv import SetBool, Trigger

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion


# ===================================================================== CONFIG
def load_config(path="/etc/fpms/config.env"):
    """Same contract as fpms_cored.py: file first, environment wins.

    NOTE the file may not exist. PI_FILE_INVENTORY.md records that it did not
    at survey time while HANDOFF.md says it does; both are handled, because a
    missing config file must never stop the observability layer from starting.
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

# 127.0.0.1 on purpose, and it is worth saying why: the broker runs on the Pi,
# so this hop never touches WiFi. The MQTT drop problem that motivated the move
# to Foxglove was the Pi->LAPTOP hop at QoS 0 under saturation. Pi-local MQTT
# over loopback at QoS 1 is not that link and is not affected by it.
BROKER = CFG.get("FPMS_MQTT_HOST_LOCAL", "127.0.0.1")
PORT = int(CFG.get("FPMS_MQTT_PORT", "1883"))
USER = CFG.get("FPMS_MQTT_USER", "")
PASS = CFG.get("FPMS_MQTT_PASS", "")

# How long a bridge-side arm stays good for. Shorter than fpms_missions'
# ARM_TIMEOUT_S (120 s) on purpose: this is "the operator armed, then pressed
# run, as one deliberate action", not "the rover is armed".
ARM_WINDOW_S = float(CFG.get("FPMS_FOXGLOVE_ARM_WINDOW_S", "60"))

# The arena start pose, from arena_zones.json and the map->odom anchor in
# fpms-tf.service. Both say (972, 228) mm. Overridable, but do not, casually.
RESET_X_MM = float(CFG.get("FPMS_RESET_X_MM", "972"))
RESET_Y_MM = float(CFG.get("FPMS_RESET_Y_MM", "228"))

# Sources whose freshness is reported on /diagnostics. (topic, type, warn_s,
# error_s). warn/error are sized from each source's own cadence, not from one
# global number — /battery at 1 Hz and /imu at 25 Hz cannot share a threshold.
WATCH_ROS = [
    ("/scan_lidar", "sensor_msgs/msg/LaserScan", 2.0, 5.0),
    ("/odom_raw", "nav_msgs/msg/Odometry", 1.0, 3.0),
    ("/odom", "nav_msgs/msg/Odometry", 1.0, 3.0),
    ("/imu", "sensor_msgs/msg/Imu", 1.0, 3.0),
    ("/battery", None, 3.0, 10.0),
    ("/wheel_ticks", "std_msgs/msg/Int32MultiArray", 2.0, 5.0),
]
WATCH_MQTT = [
    ("telemetry/mission", 5.0, 20.0),
    ("telemetry/lidar", 2.0, 5.0),
    ("telemetry/pose", 5.0, 20.0),
]


def log(msg):
    sys.stdout.write(f"[fpms-foxglove-cmd] {msg}\n")
    sys.stdout.flush()


# ======================================================================== BUS
class Bus:
    """One MQTT connection. Never raises into the caller; reconnects forever."""

    def __init__(self, name, on_message=None):
        self.name = name
        self.client = mqtt.Client(client_id=f"{THING}-fox-{name}",
                                  callback_api_version=CallbackAPIVersion.VERSION2)
        if USER:
            self.client.username_pw_set(USER, PASS or None)
        self.client.reconnect_delay_set(min_delay=1, max_delay=15)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        if on_message:
            self.client.on_message = on_message
        self.connected = False
        self._subs = []

    def subscribe(self, topic, qos=0):
        self._subs.append((topic, qos))

    def start(self):
        try:
            self.client.connect_async(BROKER, PORT, keepalive=30)
            self.client.loop_start()
            log(f"mqtt[{self.name}]: connecting to {BROKER}:{PORT}")
        except Exception as e:
            log(f"mqtt[{self.name}]: start failed ({e}); continuing")

    def _on_connect(self, client, _u, _f, rc, _p=None):
        ok = (rc == 0 or getattr(rc, "value", 1) == 0)
        self.connected = bool(ok)
        log(f"mqtt[{self.name}]: connected rc={rc}")
        if not ok:
            return
        for topic, qos in self._subs:
            client.subscribe(topic, qos=qos)

    def _on_disconnect(self, _c, _u, *args):
        self.connected = False
        log(f"mqtt[{self.name}]: disconnected; paho will retry")

    def publish(self, suffix, payload, qos=1):
        """Publish to fpms/<thing>/<suffix>. Returns True if handed to paho."""
        try:
            topic = f"fpms/{THING}/{suffix}"
            body = json.dumps(payload)
            info = self.client.publish(topic, body, qos=qos)
            return info.rc == mqtt.MQTT_ERR_SUCCESS
        except Exception as e:
            log(f"mqtt[{self.name}]: publish {suffix} failed ({e})")
            return False


# ======================================================================= NODE
class FoxgloveCmdBridge(Node):

    def __init__(self):
        super().__init__("fpms_foxglove_cmd")

        # Two callback groups. The stop group holds ONLY the two stop
        # subscriptions, so under the MultiThreadedExecutor a stop is never
        # queued behind a service handler that is waiting on MQTT.
        self.cg_stop = MutuallyExclusiveCallbackGroup()
        self.cg_work = MutuallyExclusiveCallbackGroup()

        # --- MQTT: the stop client subscribes to NOTHING. Do not add one.
        self.stop_bus = Bus("stop")
        self.bus = Bus("cmd", on_message=self._on_mqtt)
        for suffix, _w, _e in WATCH_MQTT:
            self.bus.subscribe(f"fpms/{THING}/{suffix}", qos=0)
        self.bus.subscribe(f"fpms/{THING}/telemetry/residual", qos=0)
        self.bus.subscribe(f"fpms/{THING}/events/+", qos=1)
        self.stop_bus.start()
        self.bus.start()

        # --- freshness bookkeeping
        self._seen = {}          # key -> (last_monotonic, ewma_hz)
        self._seen_lock = threading.Lock()

        # --- arm latch (bridge-side second action)
        self._armed_at = 0.0
        self._arm_lock = threading.Lock()

        # === SUBSCRIPTIONS: STOP =========================================
        # RELIABLE + VOLATILE, and VOLATILE is load bearing.
        #
        # fpms_cored publishes /estop as RELIABLE + TRANSIENT_LOCAL. Foxglove
        # Studio's client publisher is RELIABLE + VOLATILE. A TRANSIENT_LOCAL
        # *reader* requires a TRANSIENT_LOCAL *writer*, so making this reader
        # TRANSIENT_LOCAL to "catch a latched stop" would make it INCOMPATIBLE
        # with Foxglove's publisher and silently receive nothing from the panic
        # button. VOLATILE matches both writers. Do not "improve" this.
        stop_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Bool, "/estop", self._on_estop, stop_qos,
                                 callback_group=self.cg_stop)
        self.create_subscription(Empty, "/fpms/cmd/stop", self._on_stop_empty,
                                 stop_qos, callback_group=self.cg_stop)

        # === SUBSCRIPTIONS: freshness watch ==============================
        # Best-effort + small depth: this node only needs to know that a
        # message arrived and when. It must never apply backpressure to a
        # sensor, and it must never hold a big queue.
        watch_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        self._watch_subs = []
        self._sub_generic(topic="/scan_lidar", qos=watch_qos)
        self._sub_generic(topic="/odom_raw", qos=watch_qos)
        self._sub_generic(topic="/odom", qos=watch_qos)
        self._sub_generic(topic="/imu", qos=watch_qos)
        self._sub_generic(topic="/battery", qos=watch_qos)
        self._sub_generic(topic="/wheel_ticks", qos=watch_qos)

        # === OPERATOR CLICK ECHO =========================================
        # Foxglove's 3D "Publish point" tool writes geometry_msgs/PointStamped
        # on /clicked_point. NOTHING ON THE ROVER CONSUMES THIS. It is echoed
        # to MQTT control/operator_point (control/, never commands/ —
        # fpms-rover-agent subscribes commands/# and NACKs every verb it does
        # not know, which is how a well-meaning new verb becomes a fake error
        # on the operator's screen) and logged. Click-to-place keep-out
        # obstacles are NOT restored by this; see the README.
        self.create_subscription(PointStamped, "/clicked_point",
                                 self._on_clicked_point, 10,
                                 callback_group=self.cg_work)

        # === MIRROR PUBLISHERS ===========================================
        # Every one of these is a READ-OUT. None is an actuator. The assertion
        # below enforces that.
        m = lambda t, ty: self.create_publisher(ty, t, 10)
        self.p_state = m("/fpms/mission/state", String)
        self.p_phase = m("/fpms/mission/phase", String)
        self.p_armed = m("/fpms/mission/armed", Bool)
        self.p_link = m("/fpms/mission/link_ok", Bool)
        self.p_lidar_ok = m("/fpms/mission/lidar_ok", Bool)
        self.p_dist_rem = m("/fpms/mission/distance_remaining_mm", Float32)
        self.p_dist_trav = m("/fpms/mission/distance_travelled_mm", Float32)
        self.p_front = m("/fpms/mission/front_mm", Float32)
        self.p_batt = m("/fpms/mission/batt_v", Float32)
        self.p_leg_i = m("/fpms/mission/leg_i", Int32)
        self.p_seg_i = m("/fpms/mission/segment_i", Int32)
        self.p_x = m("/fpms/mission/x_mm", Float32)
        self.p_y = m("/fpms/mission/y_mm", Float32)
        self.p_hdg = m("/fpms/mission/heading_deg", Float32)

        self.p_res_raw = m("/fpms/residual/raw", String)
        self.p_res_mm = m("/fpms/residual/drive_mm", Float32)
        self.p_res_deg = m("/fpms/residual/turn_deg", Float32)
        self.p_res_cum_mm = m("/fpms/residual/cumulative_drive_mm", Float32)
        self.p_res_cum_deg = m("/fpms/residual/cumulative_turn_deg", Float32)
        self.p_res_lat = m("/fpms/residual/lateral_mm", Float32)

        # Command receipts, nacks and acks, so the operator can SEE that the
        # button they pressed was honoured rather than inferring it from the
        # rover moving (or not).
        self.p_events = m("/fpms/events", String)

        self.p_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self._assert_no_actuators()

        # === SERVICES ====================================================
        # std_srvs only. A custom .srv would need a colcon package built on the
        # Pi, and Trigger/SetBool already carry exactly what a button needs:
        # a success flag and a message the operator can read in the panel.
        s = lambda n, t, cb: self.create_service(t, n, cb,
                                                 callback_group=self.cg_work)
        s("/fpms/plan_m1", Trigger, self.srv_plan_m1)
        s("/fpms/run_m1", Trigger, self.srv_run_m1)
        s("/fpms/plan_m2", Trigger, self.srv_plan_m2)
        s("/fpms/run_m2", Trigger, self.srv_run_m2)
        s("/fpms/arm", SetBool, self.srv_arm)
        s("/fpms/reset_pose", Trigger, self.srv_reset_pose)
        s("/fpms/read_encoders", Trigger, self.srv_read_encoders)
        # A service form of stop EXISTS but is not the panic button. It is here
        # so a script can stop the rover and get a receipt. The Foxglove STOP
        # panel is the topic, not this.
        s("/fpms/stop", Trigger, self.srv_stop)

        self.create_timer(0.5, self._diag_tick, callback_group=self.cg_work)

        dom = os.environ.get("ROS_DOMAIN_ID", "<unset>")
        log(f"ROS_DOMAIN_ID={dom}")
        if dom != "20":
            log("*** WARNING: ROS_DOMAIN_ID IS NOT 20. The drive board "
                "publishes ONLY on domain 20. Every topic will look absent and "
                "it will look exactly like a dead link. Fix the unit. ***")
        log(f"thing={THING} broker={BROKER}:{PORT} "
            f"arm_window={ARM_WINDOW_S:.0f}s reset_pose=({RESET_X_MM:.0f},"
            f"{RESET_Y_MM:.0f})mm")
        log("publishers on actuation topics: 0 (this node cannot move the rover)")

    # ------------------------------------------------------------ guardrail
    FORBIDDEN_PUBS = ("/cmd_vel", "/cmd_duty", "/cmd_enable", "/estop",
                      "/reset_encoders", "/beep", "/servo_s1", "/servo_s2")

    def _assert_no_actuators(self):
        """Fail loudly at startup if anyone ever adds an actuation publisher.

        A comment saying "this node cannot move the rover" decays. A check that
        aborts the process does not.
        """
        bad = []
        for pub in self.publishers:
            name = pub.topic_name
            if name in self.FORBIDDEN_PUBS:
                bad.append(name)
        if bad:
            raise SystemExit(
                f"REFUSING TO START: fpms_foxglove_cmd created publishers on "
                f"{bad}. This node is a command TRANSLATOR; the only thing it "
                f"is allowed to emit is an MQTT command verb. Whatever added "
                f"that publisher belongs in fpms_teleop or fpms_missions.")

    # ---------------------------------------------------------------- STOP
    def _stop_now(self, why):
        """The whole stop path. No parsing, no locks, no state check.

        Both verbs, both at QoS 1, on the dedicated stop client:
          * `estop` and `stop` are separate verbs downstream. fpms_missions
            aborts on either; fpms_teleop acks either; fpms-cored's stop client
            is subscribed to stop/estop/auto_off by name. Sending both costs
            one extra 60-byte publish and removes a whole class of "which verb
            does this consumer actually listen to" doubt at the worst possible
            moment.
        """
        payload = {"source": "foxglove", "why": why, "ts": time.time()}
        ok1 = self.stop_bus.publish("commands/estop", payload, qos=1)
        ok2 = self.stop_bus.publish("commands/stop", payload, qos=1)
        log(f"STOP ({why}) -> mqtt estop={'ok' if ok1 else 'FAIL'} "
            f"stop={'ok' if ok2 else 'FAIL'}")
        return ok1 or ok2

    def _on_estop(self, msg):
        if msg.data:
            self._stop_now("/estop true")
            self._disarm_local("estop")
        else:
            # There is no MQTT "un-stop" verb, and inventing one here would be
            # a way to un-stop a rover from a panel. Clearing the latch is the
            # firmware's business; this is logged so the operator sees it
            # happened.
            log("/estop false — firmware latch clear requested; no MQTT verb "
                "sent (there is deliberately no un-stop command)")

    def _on_stop_empty(self, _msg):
        self._stop_now("/fpms/cmd/stop")
        self._disarm_local("stop")

    def srv_stop(self, _req, resp):
        ok = self._stop_now("/fpms/stop service")
        self._disarm_local("stop")
        resp.success = bool(ok)
        resp.message = ("stop published to commands/stop + commands/estop at "
                        "QoS 1" if ok else
                        "MQTT publish FAILED — use the STOP topic button, and "
                        "get to the rover")
        return resp

    # ----------------------------------------------------------- arm latch
    def _disarm_local(self, why):
        with self._arm_lock:
            if self._armed_at:
                log(f"bridge arm window cleared ({why})")
            self._armed_at = 0.0

    def _arm_age(self):
        with self._arm_lock:
            if not self._armed_at:
                return None
            return time.monotonic() - self._armed_at

    def srv_arm(self, req, resp):
        want = bool(req.data)
        ok = self.bus.publish("commands/mission", {"arm": want}, qos=1)
        with self._arm_lock:
            self._armed_at = time.monotonic() if want else 0.0
        if want:
            resp.message = (
                f"ARM sent. Run is unlocked for {ARM_WINDOW_S:.0f}s. The rover "
                f"also has its own arm latch and its own timeout — this window "
                f"only governs the Foxglove Run button.")
        else:
            resp.message = "DISARM sent."
        if not ok:
            resp.message = "MQTT publish failed — nothing was sent."
        resp.success = bool(ok)
        return resp

    # ------------------------------------------------------------ missions
    # M1 and M2 differ ONLY in the mission name on the wire. They were written
    # out separately, which is why the console had no M1 button at all: adding
    # one meant duplicating the arm-window logic, and nobody did. One pair of
    # helpers now serves both, so a third mission is two lines and cannot
    # accidentally get a weaker refusal path than M2 has.
    def _plan_mission(self, name, resp):
        ok = self.bus.publish("commands/mission",
                              {"name": name, "preview": True}, qos=1)
        resp.success = bool(ok)
        resp.message = ("PLAN %s (preview) sent - this cannot move the rover. "
                        "Watch /fpms/events and the mission panel for the route."
                        % name.upper()
                        if ok else "MQTT publish failed - nothing was sent.")
        return resp

    def _run_mission(self, name, resp):
        age = self._arm_age()
        if age is None:
            resp.success = False
            resp.message = ("REFUSED: not armed. Press ARM first, then RUN. "
                            "Run is deliberately two actions so a stray click "
                            "cannot drive the rover.")
            return resp
        if age > ARM_WINDOW_S:
            self._disarm_local("window expired")
            resp.success = False
            resp.message = (f"REFUSED: armed {age:.0f}s ago, window is "
                            f"{ARM_WINDOW_S:.0f}s. Press ARM again, then RUN.")
            return resp
        # Consume the window: one ARM buys exactly one RUN.
        self._disarm_local("consumed by run")
        ok = self.bus.publish("commands/mission", {"name": name}, qos=1)
        resp.success = bool(ok)
        resp.message = ("RUN %s sent. The rover still applies its own refusals "
                        "(arm latch, odometry link, battery, LiDAR, arena "
                        "bounds) and will NACK on /fpms/events if any fails."
                        % name.upper()
                        if ok else "MQTT publish failed - nothing was sent.")
        return resp

    def srv_plan_m1(self, _req, resp):
        """PREVIEW ONLY, exactly as srv_plan_m2 - it cannot move the rover."""
        return self._plan_mission("m1", resp)

    def srv_run_m1(self, _req, resp):
        return self._run_mission("m1", resp)

    def srv_plan_m2(self, _req, resp):
        """PREVIEW ONLY. fpms_missions._cmd_mission returns before any of the
        motion path when `preview` is set: it plans, publishes the route on
        telemetry/mission_plan, and touches nothing that can move."""
        ok = self.bus.publish("commands/mission",
                              {"name": "m2", "preview": True}, qos=1)
        resp.success = bool(ok)
        resp.message = ("PLAN M2 (preview) sent — this cannot move the rover. "
                        "Watch /fpms/events and the mission panel for the route."
                        if ok else "MQTT publish failed — nothing was sent.")
        return resp

    def srv_run_m2(self, _req, resp):
        age = self._arm_age()
        if age is None:
            resp.success = False
            resp.message = ("REFUSED: not armed. Press ARM first, then RUN. "
                            "Run is deliberately two actions so a stray click "
                            "cannot drive the rover.")
            return resp
        if age > ARM_WINDOW_S:
            self._disarm_local("window expired")
            resp.success = False
            resp.message = (f"REFUSED: armed {age:.0f}s ago, window is "
                            f"{ARM_WINDOW_S:.0f}s. Press ARM again, then RUN.")
            return resp
        # Consume the window: one ARM buys exactly one RUN.
        self._disarm_local("consumed by run")
        ok = self.bus.publish("commands/mission", {"name": "m2"}, qos=1)
        resp.success = bool(ok)
        resp.message = ("RUN M2 sent. The rover still applies its own refusals "
                        "(arm latch, odometry link, battery, LiDAR, arena "
                        "bounds) and will NACK on /fpms/events if any fails."
                        if ok else "MQTT publish failed — nothing was sent.")
        return resp

    def srv_reset_pose(self, _req, resp):
        ok = self.bus.publish("commands/set_coordinate",
                              {"x_mm": RESET_X_MM, "y_mm": RESET_Y_MM}, qos=1)
        resp.success = bool(ok)
        resp.message = (
            f"set_coordinate ({RESET_X_MM:.0f}, {RESET_Y_MM:.0f}) mm sent — the "
            f"arena start pose. This declares where the rover IS; physically "
            f"place it there first. It does not move anything."
            if ok else "MQTT publish failed — nothing was sent.")
        return resp

    def srv_read_encoders(self, _req, resp):
        ok = self.bus.publish("commands/read_encoders", {}, qos=1)
        resp.success = bool(ok)
        resp.message = ("read_encoders sent — teleop answers on events/encoders,"
                        " mirrored to /fpms/events."
                        if ok else "MQTT publish failed — nothing was sent.")
        return resp

    # -------------------------------------------------------------- clicks
    def _on_clicked_point(self, msg):
        p = msg.point
        log(f"operator clicked ({p.x:.3f}, {p.y:.3f}) in "
            f"{msg.header.frame_id!r} — echoed to MQTT, NOT acted on")
        self.bus.publish("control/operator_point",
                         {"x_m": p.x, "y_m": p.y, "z_m": p.z,
                          "frame_id": msg.header.frame_id,
                          "source": "foxglove-3d-panel",
                          "note": "no consumer on the rover; display only"},
                         qos=0)

    # -------------------------------------------------------- MQTT -> ROS
    def _note(self, key):
        now = time.monotonic()
        with self._seen_lock:
            prev, hz = self._seen.get(key, (None, 0.0))
            if prev is not None:
                dt = now - prev
                if dt > 1e-6:
                    inst = 1.0 / dt
                    # EWMA. A single long gap should move the reported rate
                    # substantially, because a rate that stays plausible while
                    # the source has died is the exact bug being fixed here.
                    hz = inst if hz <= 0 else (0.7 * hz + 0.3 * inst)
            self._seen[key] = (now, hz)

    def _sub_generic(self, topic, qos):
        """Subscribe purely to timestamp arrivals, with the right message type.

        rclpy has no untyped subscription, so the handful of types actually on
        this rover are mapped here. An unknown topic is simply not watched
        rather than crashing the node — /wheel_ticks, for instance, does not
        exist until firmware v3 is flashed.
        """
        types = {
            "/scan_lidar": ("sensor_msgs.msg", "LaserScan"),
            "/odom_raw": ("nav_msgs.msg", "Odometry"),
            "/odom": ("nav_msgs.msg", "Odometry"),
            "/imu": ("sensor_msgs.msg", "Imu"),
            "/wheel_ticks": ("std_msgs.msg", "Int32MultiArray"),
        }
        try:
            if topic == "/battery":
                # /battery is UInt16 decivolts on Yahboom stock and
                # BatteryState in volts on firmware v3. Both are subscribed;
                # whichever exists is the one that ticks. This is the single
                # place in the stack where that fork is handled without a
                # config flag, because freshness does not care about units.
                from std_msgs.msg import UInt16
                from sensor_msgs.msg import BatteryState
                self._watch_subs.append(self.create_subscription(
                    UInt16, topic, lambda _m: self._note(topic), qos,
                    callback_group=self.cg_work))
                self._watch_subs.append(self.create_subscription(
                    BatteryState, topic, lambda _m: self._note(topic), qos,
                    callback_group=self.cg_work))
                return
            mod, cls = types[topic]
            msgcls = getattr(__import__(mod, fromlist=[cls]), cls)
            self._watch_subs.append(self.create_subscription(
                msgcls, topic, lambda _m, t=topic: self._note(t), qos,
                callback_group=self.cg_work))
        except Exception as e:
            log(f"freshness watch for {topic} not installed ({e})")

    def _on_mqtt(self, _c, _u, msg):
        """paho's network thread. Must return fast and must never raise."""
        try:
            suffix = msg.topic.split(f"fpms/{THING}/", 1)[-1]
            self._note(f"mqtt:{suffix}")
            try:
                payload = json.loads(msg.payload.decode() or "{}")
            except Exception:
                return
            if not isinstance(payload, dict):
                return
            if suffix == "telemetry/mission":
                self._mirror_mission(payload)
            elif suffix == "telemetry/residual":
                self._mirror_residual(payload)
            elif suffix.startswith("events/"):
                self._mirror_event(suffix, payload)
        except Exception as e:
            log(f"mqtt message handler error {e}")

    def _f32(self, pub, value):
        """Publish a float, OMITTING it when the source omitted it.

        fpms_missions deliberately omits pose fields rather than zero-filling
        them, because a consumer seeing 0,0 cannot tell it apart from a real
        corner of the arena. Zero-filling here would throw that away and put a
        lie on a plot.
        """
        if value is None:
            return
        try:
            m = Float32()
            m.data = float(value)
            pub.publish(m)
        except (TypeError, ValueError):
            pass

    def _i32(self, pub, value):
        if value is None:
            return
        try:
            m = Int32()
            m.data = int(value)
            pub.publish(m)
        except (TypeError, ValueError):
            pass

    def _b(self, pub, value):
        if value is None:
            return
        m = Bool()
        m.data = bool(value)
        pub.publish(m)

    def _s(self, pub, value):
        m = String()
        m.data = value if isinstance(value, str) else json.dumps(value)
        pub.publish(m)

    def _mirror_mission(self, p):
        self._s(self.p_state, p)
        self._s(self.p_phase, str(p.get("phase") or "idle"))
        self._b(self.p_armed, p.get("armed"))
        self._b(self.p_link, p.get("link_ok"))
        self._b(self.p_lidar_ok, p.get("lidar_ok"))
        self._f32(self.p_dist_rem, p.get("distance_remaining_mm"))
        self._f32(self.p_dist_trav, p.get("distance_travelled_mm"))
        self._f32(self.p_front, p.get("front_mm"))
        self._f32(self.p_batt, p.get("batt_v"))
        self._i32(self.p_leg_i, p.get("leg_i"))
        self._i32(self.p_seg_i, p.get("segment_i"))
        self._f32(self.p_x, p.get("x_mm"))
        self._f32(self.p_y, p.get("y_mm"))
        self._f32(self.p_hdg, p.get("heading_deg"))

    def _mirror_residual(self, p):
        self._s(self.p_res_raw, p)
        kind = p.get("kind")
        if kind == "turn":
            self._f32(self.p_res_deg, p.get("residual"))
            self._f32(self.p_res_cum_deg, p.get("cumulative_residual"))
        else:
            self._f32(self.p_res_mm, p.get("residual"))
            self._f32(self.p_res_cum_mm, p.get("cumulative_residual"))
        self._f32(self.p_res_lat, p.get("lateral_mm"))

    def _mirror_event(self, suffix, p):
        out = dict(p)
        out["_topic"] = suffix
        out["_rx_ts"] = round(time.time(), 3)
        self._s(self.p_events, out)

    # ------------------------------------------------------------- /diagnostics
    def _diag_tick(self):
        now = time.monotonic()
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        with self._seen_lock:
            snap = dict(self._seen)

        def status(name, key, warn_s, err_s, note=""):
            st = DiagnosticStatus()
            st.name = name
            st.hardware_id = THING
            rec = snap.get(key)
            if rec is None:
                st.level = DiagnosticStatus.ERROR
                st.message = "NEVER SEEN since this bridge started"
                st.values = [KeyValue(key="age_s", value="never"),
                             KeyValue(key="hz", value="0.00")]
                if note:
                    st.values.append(KeyValue(key="note", value=note))
                return st
            last, hz = rec
            age = now - last
            if age >= err_s:
                st.level = DiagnosticStatus.ERROR
                st.message = f"STALE {age:.1f}s (>= {err_s:.0f}s)"
            elif age >= warn_s:
                st.level = DiagnosticStatus.WARN
                st.message = f"slow: {age:.1f}s since last message"
            else:
                st.level = DiagnosticStatus.OK
                st.message = f"{hz:.2f} Hz"
            st.values = [KeyValue(key="age_s", value=f"{age:.2f}"),
                         KeyValue(key="hz", value=f"{hz:.2f}"),
                         KeyValue(key="warn_s", value=f"{warn_s:.0f}"),
                         KeyValue(key="error_s", value=f"{err_s:.0f}")]
            if note:
                st.values.append(KeyValue(key="note", value=note))
            return st

        notes = {
            "/scan_lidar": "the ONLY live LiDAR; the board's /scan is dead",
            "/wheel_ticks": "firmware v3 only — absent on Yahboom stock",
        }
        for topic, _typ, w, e in WATCH_ROS:
            arr.status.append(status(f"ros{topic}", topic, w, e,
                                     notes.get(topic, "")))
        for suffix, w, e in WATCH_MQTT:
            arr.status.append(status(f"mqtt/{suffix}", f"mqtt:{suffix}", w, e))

        # The bridge's own MQTT connection state. If this is ERROR, every
        # button on the CONTROL tab is a no-op and the operator must know that
        # BEFORE pressing STOP and believing it worked.
        st = DiagnosticStatus()
        st.name = "fpms_foxglove_cmd/mqtt"
        st.hardware_id = THING
        cmd_ok, stop_ok = self.bus.connected, self.stop_bus.connected
        if stop_ok and cmd_ok:
            st.level, st.message = DiagnosticStatus.OK, "both clients connected"
        elif stop_ok:
            st.level = DiagnosticStatus.WARN
            st.message = "telemetry client down; STOP path is UP"
        else:
            st.level = DiagnosticStatus.ERROR
            st.message = ("STOP PATH DOWN — the broker is unreachable from the "
                          "Pi. Foxglove buttons do nothing. Use the hardware.")
        st.values = [KeyValue(key="stop_client", value=str(stop_ok)),
                     KeyValue(key="cmd_client", value=str(cmd_ok)),
                     KeyValue(key="broker", value=f"{BROKER}:{PORT}")]
        arr.status.append(st)

        # The arm window, so a "why won't Run work" question answers itself.
        age = self._arm_age()
        st = DiagnosticStatus()
        st.name = "fpms_foxglove_cmd/run_interlock"
        st.hardware_id = THING
        if age is None:
            st.level, st.message = DiagnosticStatus.OK, "not armed — RUN locked"
        elif age > ARM_WINDOW_S:
            st.level, st.message = DiagnosticStatus.OK, "arm window expired"
        else:
            st.level = DiagnosticStatus.WARN
            st.message = f"ARMED — RUN unlocked for {ARM_WINDOW_S - age:.0f}s"
        arr.status.append(st)

        self.p_diag.publish(arr)


def main():
    rclpy.init(args=None)
    node = FoxgloveCmdBridge()
    # MultiThreadedExecutor, not the default. With the single-threaded
    # executor a /estop message that arrived while a service handler was
    # blocked in an MQTT publish would wait for that handler to return. That is
    # a panic button behind a queue.
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

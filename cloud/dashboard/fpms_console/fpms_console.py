#!/usr/bin/env python3
"""FPMS operator console — the Pi-side half.

Runs ON THE PI (systemd: fpms-console.service). Two jobs, one process:

  1. SERVE THE UI. A single static HTML file on :8090, from the Pi, exactly
     like the phase6 :8085 app the operator liked. One URL to point a browser
     at; nothing installed on the laptop; a phone or tablet works too.

  2. FILL THE TWO HOLES IN THE ROS TOPIC SURFACE that the console needs and
     that nothing else publishes:
       * `/fpms/plan/route`          the planned route + occupancy stats
       * `/fpms/console/keepout_state`  operator keep-out discs, latched

===========================================================================
THIS NODE CANNOT MOVE THE ROVER
===========================================================================
It creates ZERO publishers on any actuation topic, and `_assert_no_actuators`
aborts the process at startup if one is ever added. It has no MQTT PUBLISHER
at all — its MQTT client is subscribe-only, so it cannot even emit a command
verb. Its total power is: republish JSON that the rover already broadcast.

===========================================================================
WHY MQTT APPEARS HERE AT ALL, GIVEN THIS IS THE "ROS-TO-ROS" CONSOLE
===========================================================================
Be blunt about this. The operator's pipeline IS ROS-to-ROS: the browser talks
rosbridge, and every live value it renders arrives as a ROS message. But on
this rover the *planned route* has only ever existed on MQTT
(`telemetry/mission_plan`, published by fpms_missions.py, which must not be
edited). There is no ROS topic carrying it. So one Pi-side node translates it
once, into ROS, in the same way `fpms-lidar-ros.service` already translates
`telemetry/lidar` into `/scan_lidar`. That is a Pi-internal bridge, not a
translation layer between the rover and the operator: nothing in the live path
from Pi to laptop is MQTT.

If `fpms_missions.py` ever publishes the route on ROS directly, delete
`PlanMirror` and subscribe to that instead. Nothing else here changes.

===========================================================================
ROS_DOMAIN_ID
===========================================================================
20. It is set IN THE UNIT FILE and nowhere else — `/etc/fpms/config.env` does
not set it and never has. Every ad-hoc `ros2` command in this project's
history that forgot it saw an empty topic list and concluded the link was
dead. This node logs its effective domain at startup and screams if it is
wrong.
"""

import json
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE_PATH = os.environ.get("FPMS_CONSOLE_PAGE", os.path.join(HERE, "console.html"))


def log(msg):
    print(f"[console {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ===================================================================== CONFIG
def load_config(path="/etc/fpms/config.env"):
    """File first, environment wins. Same contract as fpms_cored.py.

    A missing file must never stop the observability layer from starting, so
    this returns {} rather than raising.
    """
    cfg = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                cfg[k.strip()] = v.strip().strip('"').strip("'")
    except OSError:
        pass
    cfg.update({k: v for k, v in os.environ.items() if k.startswith("FPMS_")})
    return cfg


CFG = load_config()
THING = CFG.get("FPMS_THING_NAME", "rover2")
BROKER = CFG.get("FPMS_MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(CFG.get("FPMS_MQTT_PORT", "1883"))
MQTT_USER = CFG.get("FPMS_MQTT_USER") or None
MQTT_PASS = CFG.get("FPMS_MQTT_PASS") or None
HTTP_PORT = int(CFG.get("FPMS_CONSOLE_PORT", "8090"))
ROSBRIDGE_PORT = int(CFG.get("FPMS_ROSBRIDGE_PORT", "9090"))


# ======================================================================= HTTP
class Handler(BaseHTTPRequestHandler):
    """Static server. Deliberately tiny — no framework, no templating.

    The page is re-read from disk on every request. That costs a few hundred
    microseconds and buys the thing that matters at a competition: editing
    console.html and pressing reload is the whole edit cycle, with no service
    restart to forget.
    """

    server_version = "fpms-console"

    def log_message(self, fmt, *args):
        pass  # journald does not need a line per GET at 1 Hz

    def _send(self, code, body, ctype="text/html; charset=utf-8"):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        # No caching, ever. A cached console after a redeploy is a console
        # showing yesterday's code with today's data.
        self.send_header("Cache-Control", "no-store, must-revalidate")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html", "/console.html"):
            try:
                with open(PAGE_PATH, "rb") as f:
                    return self._send(200, f.read())
            except OSError as e:
                return self._send(500, f"<h1>console.html missing</h1><pre>{e}</pre>")
        if path == "/healthz":
            return self._send(200, json.dumps({
                "ok": True, "thing": THING,
                "rosbridge_port": ROSBRIDGE_PORT,
                "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "<unset>"),
                "plan_seen": NODE.plan_seen if NODE else False,
                "mqtt_connected": NODE.mqtt_ok if NODE else False,
            }), "application/json")
        return self._send(404, "not found", "text/plain; charset=utf-8")


def serve_http():
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    srv.daemon_threads = True
    log(f"UI on http://{socket.gethostname()}.local:{HTTP_PORT}/  "
        f"(rosbridge expected on :{ROSBRIDGE_PORT})")
    srv.serve_forever()


# ======================================================================== ROS
class ConsoleNode(Node):
    # Anything on this list must never be published by this process.
    FORBIDDEN_PUBS = ("/cmd_vel", "/cmd_duty", "/cmd_enable", "/estop",
                      "/reset_encoders", "/beep", "/servo_s1", "/servo_s2",
                      "/fpms/cmd/stop")

    def __init__(self):
        super().__init__("fpms_console")
        self.plan_seen = False
        self.mqtt_ok = False
        self._keepouts = []
        self._keep_lock = threading.Lock()

        # TRANSIENT_LOCAL: a browser that connects AFTER the plan was made
        # must still get it. Without this the route vanishes on every page
        # reload and the operator has to re-press PLAN to see a line they
        # already asked for.
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.p_plan = self.create_publisher(String, "/fpms/plan/route", latched)
        self.p_keep_state = self.create_publisher(
            String, "/fpms/console/keepout_state", latched)

        # Operator keep-out discs. RECORDED AND REDISTRIBUTED ONLY.
        #
        # There is no keep-out verb anywhere in fpms_missions.py and its
        # occupancy grid is built from LiDAR returns alone, so nothing on this
        # rover plans around these. Publishing them here does two honest
        # things: every operator's browser sees the same set, and the Pi's
        # journal records what the operator believed was blocked at the time
        # a route was chosen. It does NOT make the planner avoid them, and the
        # console says so on screen in the same words.
        self.create_subscription(String, "/fpms/console/keepout",
                                 self._on_keepout, 10)
        self.create_timer(1.0, self._republish_keepouts)

        self._assert_no_actuators()

        dom = os.environ.get("ROS_DOMAIN_ID", "<unset>")
        log(f"ROS_DOMAIN_ID={dom}")
        if dom != "20":
            log("*** WARNING: ROS_DOMAIN_ID IS NOT 20. The drive board "
                "publishes ONLY on domain 20. Every topic will look absent "
                "and it will look exactly like a dead link. Fix the unit. ***")
        log("publishers on actuation topics: 0 (this node cannot move the rover)")

    def _assert_no_actuators(self):
        bad = [p.topic_name for p in self.publishers
               if p.topic_name in self.FORBIDDEN_PUBS]
        if bad:
            raise SystemExit(
                f"REFUSING TO START: fpms_console created publishers on {bad}. "
                f"This process serves a web page and mirrors read-only JSON. "
                f"Whatever added that publisher belongs in fpms_teleop or "
                f"fpms_missions, and STOP belongs to the BROWSER, which "
                f"publishes it directly over rosbridge on its own socket.")

    # ------------------------------------------------------------- keep-outs
    def _on_keepout(self, msg):
        try:
            payload = json.loads(msg.data or "{}")
            ks = payload.get("keepouts", [])
            if not isinstance(ks, list):
                return
        except Exception:
            return
        with self._keep_lock:
            self._keepouts = ks[:200]
        where = ", ".join(
            "({:.0f},{:.0f})".format(float(k.get("x_mm", 0)), float(k.get("y_mm", 0)))
            for k in ks[:6]) or "cleared"
        log("operator keep-out set: %d disc(s) — RECORDED, NOT PLANNED  %s"
            % (len(ks), where))
        self._republish_keepouts()

    def _republish_keepouts(self):
        with self._keep_lock:
            ks = list(self._keepouts)
        m = String()
        m.data = json.dumps({"keepouts": ks, "consumed_by_planner": False,
                             "ts": time.time()})
        self.p_keep_state.publish(m)

    # ------------------------------------------------------------ plan mirror
    def publish_plan(self, payload):
        m = String()
        m.data = json.dumps(payload)
        self.p_plan.publish(m)
        self.plan_seen = True


class PlanMirror(threading.Thread):
    """MQTT `telemetry/mission_plan` -> ROS `/fpms/plan/route`. SUBSCRIBE ONLY.

    No publish() call exists on this client. It is created without any
    publish path being used, so this process physically cannot emit an MQTT
    command verb even by accident.
    """

    daemon = True

    def __init__(self, node):
        super().__init__(name="plan-mirror")
        self.node = node

    def run(self):
        try:
            import paho.mqtt.client as mqtt
            try:
                from paho.mqtt.enums import CallbackAPIVersion
                cli = mqtt.Client(CallbackAPIVersion.VERSION1,
                                  client_id=f"fpms-console-{os.getpid()}")
            except Exception:
                cli = mqtt.Client(client_id=f"fpms-console-{os.getpid()}")
        except Exception as e:
            log(f"paho unavailable ({e}) — the planned route will not be drawn. "
                f"Everything else on the console is native ROS and unaffected.")
            return

        if MQTT_USER:
            cli.username_pw_set(MQTT_USER, MQTT_PASS)
        topic = f"fpms/{THING}/telemetry/mission_plan"

        def on_connect(_c, _u, _f, rc, *a):
            self.node.mqtt_ok = (rc == 0)
            cli.subscribe(topic, qos=1)
            log(f"mqtt {'connected' if rc == 0 else f'rc={rc}'}; watching {topic}")

        def on_disconnect(_c, _u, rc, *a):
            self.node.mqtt_ok = False
            log(f"mqtt disconnected rc={rc} — the route will stop updating; "
                f"the console will show it as STALE rather than as current.")

        def on_message(_c, _u, msg):
            try:
                payload = json.loads(msg.payload.decode() or "{}")
            except Exception:
                return
            if isinstance(payload, dict):
                self.node.publish_plan(payload)

        cli.on_connect = on_connect
        cli.on_disconnect = on_disconnect
        cli.on_message = on_message
        while True:
            try:
                cli.connect(BROKER, MQTT_PORT, keepalive=30)
                cli.loop_forever()
            except Exception as e:
                self.node.mqtt_ok = False
                log(f"mqtt connect failed ({e}); retrying in 3s")
                time.sleep(3)


NODE = None


def main():
    global NODE
    if not os.path.exists(PAGE_PATH):
        sys.exit(f"REFUSING TO START: {PAGE_PATH} does not exist. There is no "
                 f"point serving a console with no page; install_console.sh "
                 f"puts it there.")
    rclpy.init()
    NODE = ConsoleNode()
    PlanMirror(NODE).start()
    threading.Thread(target=serve_http, name="http", daemon=True).start()
    try:
        rclpy.spin(NODE)
    except KeyboardInterrupt:
        pass
    finally:
        NODE.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

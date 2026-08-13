#!/usr/bin/env python3
"""Static server for the FPMS operator dashboard.

WHAT THIS PROCESS IS, AND MORE IMPORTANTLY WHAT IT IS NOT
========================================================
It serves four files over HTTP. That is all it does.

It is NOT in the data path. It does not connect to the rover, it does not
import rclpy, it does not speak MQTT, it holds no state, and it cannot move
anything. The browser dials the Pi's rosbridge on :9090 directly; this server
is out of that conversation entirely and can be killed mid-mission without the
dashboard in an already-open tab noticing.

That matters most under WSL. The WebSocket goes Windows-browser -> Pi and
never enters the WSL network namespace, so WSL's NAT, its port forwarding and
its DNS are all irrelevant to whether live data flows. Only this page comes
from WSL.

WHY STDLIB ONLY
===============
No pip install, no venv, no network at run time. `python3 serve.py` on a
freshly imaged Ubuntu, on a laptop with no internet, at a venue, works. Adding
a dependency to a dashboard whose whole job is to be available is a bad trade.

WHY NO CACHING
==============
Every response is no-store. Editing app.js and pressing reload is the entire
edit cycle; a cached asset that survives a reload is a debugging session spent
on the wrong file at 2am.
"""

import argparse
import json
import os
import posixpath
import socket
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))

# THIS IMAGE IS ROVER 1 (changed from rover2 on 2026-08-13). The hostname is
# fpms-rover1 (fpms-os/config/fpms-os.conf FPMS_HOSTNAME) and the MQTT topic
# root is rover1 (/etc/fpms/config.env FPMS_THING_NAME).
#
# THE THING NAME IS NOT A LABEL. config.env puts it plainly: a consumer left on
# the wrong root "connects, authenticates, stays connected and receives nothing
# forever. The rover looks dead; the broker, the bridge and every unit look
# healthy." The page therefore shows this value at all times and cross-checks
# it against the hardware_id the rover publishes on /diagnostics.
ROVER_HOST = os.environ.get("FPMS_ROVER_HOST", "fpms-rover1.local")
ROSBRIDGE_PORT = int(os.environ.get("FPMS_ROSBRIDGE_PORT", "9090"))
THING_NAME = os.environ.get("FPMS_THING_NAME", "rover1")

# An OPTIONAL copy of /etc/fpms/zones.json. The page recomputes every zone
# centre from arena_mm and the two fractions and REFUSES the file if a
# published centre disagrees by more than 0.05 mm — which is what zones.json
# itself demands of a consumer, because reading cx_mm/cy_mm directly would
# create yet another independent copy of numbers that must not drift.
#
# Absent file: nothing happens. Built-in derivation, no note, no fault —
# again, exactly what zones.json specifies. A file that must exist for the
# dashboard to work would be a new way for the dashboard to stop working.
ZONES_FILE = os.environ.get("FPMS_ZONES_FILE", "") or next(
    (p for p in (
        "/etc/fpms/zones.json",
        os.path.join(HERE, "..", "rover", "fpms-os", "overlay", "etc", "fpms", "zones.json"),
    ) if os.path.isfile(p)), "")


class Handler(SimpleHTTPRequestHandler):
    server_version = "fpms-dashboard"

    def __init__(self, *a, **kw):
        super().__init__(*a, directory=HERE, **kw)

    def log_message(self, fmt, *args):
        # One line per asset load is noise; failures still surface as tracebacks.
        pass

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        super().end_headers()

    def _json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = posixpath.normpath(self.path.split("?", 1)[0])
        if path in ("/config.json", "/config.json/"):
            self._json({
                "rover_host": ROVER_HOST,
                "rosbridge_port": ROSBRIDGE_PORT,
                "thing_name": THING_NAME,
                "zones_file": ZONES_FILE,
            })
            return
        if path in ("/zones.json", "/zones.json/"):
            # 404 is the correct, expected answer when no file was found. The
            # page treats it as "use the built-in derivation" and says nothing.
            if not ZONES_FILE:
                self.send_error(404, "no zones.json configured")
                return
            try:
                with open(ZONES_FILE, "rb") as f:
                    body = f.read()
            except OSError as e:
                self.send_error(404, f"zones.json unreadable: {e}")
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()


def local_addresses():
    """Best-effort list of addresses this page will be reachable on.

    Printed at startup because the single most common way to lose ten minutes
    with a WSL-hosted page is not knowing which address Windows should use.
    """
    out = []
    try:
        out.append(socket.gethostbyname(socket.gethostname()))
    except OSError:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.0.2.1", 9))          # TEST-NET-1: routed nowhere, sends nothing
        out.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    seen, uniq = set(), []
    for a in out:
        if a and not a.startswith("127.") and a not in seen:
            seen.add(a)
            uniq.append(a)
    return uniq


def main():
    ap = argparse.ArgumentParser(description="Serve the FPMS operator dashboard.")
    ap.add_argument("--port", type=int,
                    default=int(os.environ.get("FPMS_DASHBOARD_PORT", "8099")))
    # 0.0.0.0 by default so a Windows browser can reach a WSL-hosted page even
    # on WSL builds without localhost forwarding, and so a phone on the same
    # wifi works. Pass --bind 127.0.0.1 to keep it to this machine.
    ap.add_argument("--bind", default=os.environ.get("FPMS_DASHBOARD_BIND", "0.0.0.0"))
    ap.add_argument("--rover", default=None,
                    help="rosbridge host[:port] the page should dial "
                         "(default fpms-rover1.local:9090; an IP is fine and is "
                         "the answer when mDNS does not resolve in the browser)")
    ap.add_argument("--thing", default=None,
                    help="FPMS_THING_NAME the page should display and check "
                         "against the rover's own /diagnostics hardware_id "
                         "(default rover1)")
    args = ap.parse_args()

    global ROVER_HOST, THING_NAME
    if args.rover:
        if ":" in args.rover and not args.rover.startswith("["):
            host, _, port = args.rover.rpartition(":")
            ROVER_HOST = host
            globals()["ROSBRIDGE_PORT"] = int(port)
        else:
            ROVER_HOST = args.rover
    if args.thing:
        THING_NAME = args.thing

    httpd = ThreadingHTTPServer((args.bind, args.port), Handler)
    httpd.daemon_threads = True

    print(f"FPMS dashboard serving {HERE}")
    print(f"  bind        {args.bind}:{args.port}")
    print(f"  rover       {ROVER_HOST}:{ROSBRIDGE_PORT}")
    print(f"  thing       {THING_NAME}   (checked against the rover's own "
          f"/diagnostics hardware_id)")
    print(f"  zones.json  {ZONES_FILE or '(none found — built-in derivation, no fault)'}")
    print()
    print(f"  http://localhost:{args.port}/")
    for a in local_addresses():
        print(f"  http://{a}:{args.port}/            <- use this from Windows if "
              f"localhost does not work")
    print()
    print("  The browser dials the rover's rosbridge directly. This server is")
    print("  not in the data path and can be restarted at any time.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())

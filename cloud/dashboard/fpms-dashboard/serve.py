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

# The rover the page should dial by default. NO DEFAULT HOST IS SUPPLIED on
# purpose: a dashboard that silently dials "localhost" and shows a dead link
# looks exactly like a dashboard pointed at a rover that is switched off. With
# nothing set, the page asks the operator.
ROVER_HOST = os.environ.get("FPMS_ROVER_HOST", "")
ROSBRIDGE_PORT = int(os.environ.get("FPMS_ROSBRIDGE_PORT", "9090"))


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

    def do_GET(self):
        path = posixpath.normpath(self.path.split("?", 1)[0])
        if path in ("/config.json", "/config.json/"):
            body = json.dumps({
                "rover_host": ROVER_HOST,
                "rosbridge_port": ROSBRIDGE_PORT,
            }).encode()
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
                    help="rosbridge host the page should dial (else FPMS_ROVER_HOST, "
                         "else the page asks)")
    args = ap.parse_args()

    global ROVER_HOST
    if args.rover:
        if ":" in args.rover and not args.rover.startswith("["):
            host, _, port = args.rover.rpartition(":")
            ROVER_HOST = host
            globals()["ROSBRIDGE_PORT"] = int(port)
        else:
            ROVER_HOST = args.rover

    httpd = ThreadingHTTPServer((args.bind, args.port), Handler)
    httpd.daemon_threads = True

    print(f"FPMS dashboard serving {HERE}")
    print(f"  bind        {args.bind}:{args.port}")
    print(f"  rover       {ROVER_HOST or '(unset — the page will ask)'}"
          f":{ROSBRIDGE_PORT}")
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

#!/usr/bin/env python3
"""Prove the console's safety and freshness claims against the RUNNING stack.

Run it ON THE PI:   python3 ~/fpms_console/verify_console.py

It is a rosbridge CLIENT, exactly like the browser, so what it measures is
what the browser gets — not what a config file claims. Nothing here can move
the rover: it advertises, it publishes STOP, it calls the PREVIEW plan, and it
subscribes. It never publishes a drive topic — it only ATTEMPTS to advertise
one, to prove the attempt is refused, and unadvertises immediately either way.

Checks, in order of how much they matter:

  1. THE WHITELIST IS IN FORCE. Advertising /cmd_vel, /cmd_duty and
     /cmd_enable must be REFUSED by the server. This is the check that decides
     whether a stray click in a browser tab can turn a wheel. If it fails, the
     glob did not load and rosbridge has NO restriction at all — which is
     rosbridge's default, so "it started fine" proves nothing.
  2. STOP GOES OUT, on both verbs.
  3. The command services answer, and PLAN M2 (preview) is callable.
  4. Every telemetry topic the console renders is subscribable, and what rate
     it is actually arriving at right now.
"""
import json
import sys

from tornado import gen
from tornado.ioloop import IOLoop
from tornado.websocket import websocket_connect

URL = sys.argv[1] if len(sys.argv) > 1 else "ws://127.0.0.1:9090"

READ_TOPICS = [
    ("/scan_lidar", "sensor_msgs/LaserScan"),
    ("/odom_raw", "nav_msgs/Odometry"),
    ("/imu", "sensor_msgs/Imu"),
    ("/battery", None),
    ("/wheel_ticks", "std_msgs/Int32MultiArray"),
    ("/wheel_duty", "std_msgs/Int32MultiArray"),
    ("/fpms_health", "std_msgs/Int32MultiArray"),
    ("/diagnostics", "diagnostic_msgs/DiagnosticArray"),
    ("/fpms/mission/state", "std_msgs/String"),
    ("/fpms/mission/x_mm", "std_msgs/Float32"),
    ("/fpms/mission/front_mm", "std_msgs/Float32"),
    ("/fpms/residual/drive_mm", "std_msgs/Float32"),
    ("/fpms/plan/route", "std_msgs/String"),
    ("/fpms/console/keepout_state", "std_msgs/String"),
    ("/fpms/events", "std_msgs/String"),
]
# NEVER published by this script. Only advertised, to prove refusal.
FORBIDDEN = [("/cmd_vel", "geometry_msgs/Twist"),
             ("/cmd_duty", "std_msgs/Int32MultiArray"),
             ("/cmd_enable", "std_msgs/Bool")]

seen, statuses, results = {}, [], []
PASS, FAIL, WARN = "\033[1;32mPASS\033[0m", "\033[1;31mFAIL\033[0m", "\033[1;33mWARN\033[0m"
bad = 0


def on_message(raw):
    if raw is None:
        return
    try:
        m = json.loads(raw)
    except Exception:
        return
    op = m.get("op")
    if op == "publish":
        seen.setdefault(m["topic"], []).append(IOLoop.current().time())
    elif op == "status":
        statuses.append(m)
    elif op == "service_response":
        results.append(m)


async def main():
    global bad
    ws = await websocket_connect(URL, connect_timeout=10,
                                 on_message_callback=on_message)
    print(f"connected {URL}\n")

    def send(o):
        ws.write_message(json.dumps(o))

    # --- 1. the whitelist -------------------------------------------------
    #
    # HOW REFUSAL LOOKS ON THE WIRE, because this is easy to get wrong and a
    # wrong test here reads as "safe" when it is not: rosbridge does NOT send
    # an error. capabilities/advertise.py does
    #     self.protocol.log("warn", "No match found for topic, cancelling
    #                                advertisement of: <topic>")
    #     return
    # so the refusal arrives as op:"status", level:"warning", and the ONLY
    # difference between refused and accepted is the presence of that line.
    # An earlier version of this script looked for level=="error", saw none,
    # and reported the whitelist as broken when it was working.
    print("1. WHITELIST — a browser must not be able to advertise a drive topic")

    def refused_for(topic):
        for s in statuses:
            m = str(s.get("msg", ""))
            if topic in m and ("cancelling" in m or "No match found" in m):
                return True
        return False

    for topic, typ in FORBIDDEN:
        statuses.clear()
        send({"op": "advertise", "topic": topic, "type": typ, "id": "probe"})
        await gen.sleep(1.5)
        if refused_for(topic):
            print(f"   advertise {topic:<14} -> {PASS} refused by rosbridge")
        else:
            bad += 1
            print(f"   advertise {topic:<14} -> {FAIL} NOT REFUSED. THE "
                  f"WHITELIST IS NOT IN FORCE.")
            print(f"        A browser tab could publish this topic. Fix "
                  f"/etc/fpms/rosbridge_params.yaml, restart fpms-rosbridge, "
                  f"and do not use the console until this passes.")
        send({"op": "unadvertise", "topic": topic})

    # A drive topic having no publisher is weak evidence — it could just mean
    # nobody tried. /tf is live at tens of Hz and is deliberately NOT in the
    # whitelist, so a silent /tf subscription is POSITIVE proof that the glob
    # is switched on, not merely that nothing happened to be publishing.
    statuses.clear()
    seen.pop("/tf", None)
    send({"op": "subscribe", "topic": "/tf", "type": "tf2_msgs/TFMessage",
          "queue_length": 1})
    await gen.sleep(3.0)
    if seen.get("/tf"):
        bad += 1
        print(f"   canary: subscribe /tf  -> {FAIL} delivered "
              f"{len(seen['/tf'])} msgs. /tf is not in the whitelist, so the "
              f"whitelist is NOT being applied to subscriptions either.")
    else:
        print(f"   canary: subscribe /tf  -> {PASS} silent (it is live but "
              f"not whitelisted — the glob is genuinely on)")
    send({"op": "unsubscribe", "topic": "/tf"})

    # --- 2. stop ----------------------------------------------------------
    print("\n2. STOP — must advertise and publish, on both verbs")
    statuses.clear()
    send({"op": "advertise", "topic": "/estop", "type": "std_msgs/Bool"})
    send({"op": "advertise", "topic": "/fpms/cmd/stop", "type": "std_msgs/Empty"})
    await gen.sleep(1.5)
    errs = [s.get("msg") for s in statuses if s.get("level") == "error"]
    if errs:
        bad += 1
        print(f"   advertise /estop + /fpms/cmd/stop -> {FAIL} {errs}")
    else:
        print(f"   advertise /estop + /fpms/cmd/stop -> {PASS} accepted")
    statuses.clear()
    send({"op": "publish", "topic": "/estop", "msg": {"data": True}})
    send({"op": "publish", "topic": "/fpms/cmd/stop", "msg": {}})
    await gen.sleep(2.0)
    errs = [s.get("msg") for s in statuses if s.get("level") == "error"]
    if errs:
        bad += 1
        print(f"   publish   STOP                    -> {FAIL} {errs}")
    else:
        print(f"   publish   STOP                    -> {PASS} sent "
              f"(confirm the fan-out: journalctl -u fpms-foxglove-cmd -n 5)")

    # --- 3. services ------------------------------------------------------
    print("\n3. COMMANDS — PLAN M2 is preview-only and cannot move the rover")
    results.clear()
    send({"op": "call_service", "service": "/fpms/plan_m2", "args": {},
          "id": "plan1"})
    await gen.sleep(10.0)
    if results:
        r = results[0]
        v = r.get("values") or {}
        mark = PASS if r.get("result") else FAIL
        if not r.get("result"):
            bad += 1
        print(f"   /fpms/plan_m2 -> {mark} {v.get('message', v)}")
    else:
        bad += 1
        print(f"   /fpms/plan_m2 -> {FAIL} no response in 10 s. "
              f"fpms-foxglove-cmd owns these services — is it running?")

    # --- 4. feeds ---------------------------------------------------------
    #
    # /fpms/events is throttled here for the same reason the console throttles
    # it: measured on the running rover it bursts to ~330 Hz, and rosbridge
    # serialises every subscription onto ONE protocol thread and ONE socket.
    # An unthrottled event storm starves /odom_raw and /scan_lidar on the same
    # connection — which looks exactly like "those topics are dead" and is not.
    print("\n4. FEEDS — what the console will actually render")
    statuses.clear()
    for topic, typ in READ_TOPICS:
        s = {"op": "subscribe", "topic": topic, "queue_length": 1}
        if typ:
            s["type"] = typ
        if topic == "/fpms/events":
            s["throttle_rate"] = 100
        send(s)
    await gen.sleep(6.0)
    for s in statuses:
        if s.get("level") in ("error", "warning", "warn"):
            print(f"   {WARN} server said: {str(s.get('msg'))[:180]}")
    for topic, _ in READ_TOPICS:
        ts = seen.get(topic, [])
        if not ts:
            print(f"   {topic:<34} {WARN} SILENT for 6 s — the console shows "
                  f"this as 'never seen', which is the honest answer")
        else:
            hz = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else 0.0
            print(f"   {topic:<34} {PASS} {len(ts):>3} msg  ~{hz:5.1f} Hz")

    ws.close()
    print(f"\n{'ALL SAFETY CHECKS PASSED' if bad == 0 else str(bad) + ' CHECK(S) FAILED — READ THEM'}")


IOLoop.current().run_sync(main)
sys.exit(1 if bad else 0)

#!/usr/bin/env python3
"""Proof that fpms-cored's stop path is BOUNDED. Runs on a laptop, no rover.

WHY THIS FILE EXISTS
--------------------
fpms-cored was deployed without ever having been run, and on its first contact
with a live broker it built a feedback ring:

    relay -> own subscription -> latch -> relay -> ...

Measured on the rover: 3394 commands/estop, 3375 commands/stop,
6773 events/nack and 2671 events/stop_asserted IN TEN SECONDS. It took the
micro-ROS link down with it.

The mechanism is simple: path (a) re-publishes `commands/<verb>`, which is
exactly the topic the stop client subscribes to, and `on_stop_message` parses
no JSON (by design, for latency) so it could not see the `source`/`relay`
markers the relay stamps.

The point of this harness is that the bug was only reproducible against a
broker that ECHOES SUBSCRIBED TOPICS BACK. So we build one. `FakeBroker`
delivers every publish to every matching subscriber on a separate thread, the
same way paho's network thread does. That is faithful enough to reproduce the
original storm, which means a pass here is real evidence and not a tautology.

WHAT IS ASSERTED
----------------
  T1  ONE estop, benign world   -> commands/estop is bounded (<= 2: the
                                   original plus at most one relay), total
                                   traffic is small, and the stop is HONOURED.
  T2  Adversarial echo          -> a hostile participant echoes every
                                   commands/* back with the markers STRIPPED,
                                   which is the worst case layer 1 cannot
                                   catch. Traffic must still be bounded by the
                                   cmd_id dedup and the latch/relay caps.
  T3  Unknown-verb flood        -> 200 unknown commands must NOT produce an
                                   unbounded reply cascade, and must not
                                   produce one thread per command.

Run:  python cloud/dashboard/rover/test_cored_loop.py
"""
from __future__ import annotations

import collections
import json
import os
import queue
import sys
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
STACK = os.path.join(HERE, "stack")
sys.path.insert(0, STACK)

# Keep the test hermetic: no ROS, no systemctl kill, no /etc/fpms.
os.environ["FPMS_CORE_ESTOP_ROS"] = "0"
os.environ["FPMS_CORE_STOP_ESCALATE"] = "0"
os.environ["FPMS_THING_NAME"] = "rover2"

RUN = threading.Event()
RUN.set()


# --------------------------------------------------------------- fake broker ---

def topic_matches(filt, topic):
    f, t = filt.split("/"), topic.split("/")
    for i, seg in enumerate(f):
        if seg == "#":
            return True
        if i >= len(t):
            return False
        if seg == "+":
            continue
        if seg != t[i]:
            return False
    return len(f) == len(t)


class FakeBroker:
    """Echoes to subscribers on their own thread, like a real broker + paho."""

    def __init__(self):
        self.subs = []                      # (client, filter)
        self.counts = collections.Counter()  # topic -> n published
        self.lock = threading.RLock()
        self.hooks = []                     # extra (filter, fn) taps

    def add_sub(self, client, filt):
        with self.lock:
            self.subs.append((client, filt))

    def add_hook(self, filt, fn):
        with self.lock:
            self.hooks.append((filt, fn))

    def publish(self, topic, payload):
        if isinstance(payload, str):
            payload = payload.encode()
        with self.lock:
            self.counts[topic] += 1
            targets = [c for c, f in self.subs if topic_matches(f, topic)]
            hooks = [fn for f, fn in self.hooks if topic_matches(f, topic)]
        for c in targets:
            c._deliver(topic, payload)
        for fn in hooks:
            fn(topic, payload)

    def total(self):
        return sum(self.counts.values())

    def report(self, title):
        print(f"\n  {title}")
        for topic, n in sorted(self.counts.items(), key=lambda kv: -kv[1]):
            print(f"    {n:6d}  {topic}")
        print(f"    {self.total():6d}  TOTAL")


BROKER = FakeBroker()


class FakeClient:
    def __init__(self, client_id=None, callback_api_version=None, **kw):
        self.client_id = client_id
        self.on_message = None
        self.on_connect = None
        self.on_disconnect = None
        self.inbox = queue.Queue()
        self._thread = None

    # --- paho API surface fpms_cored actually uses ---
    def username_pw_set(self, *a, **k):
        pass

    def will_set(self, *a, **k):
        pass

    def reconnect_delay_set(self, *a, **k):
        pass

    def connect_async(self, *a, **k):
        pass

    def disconnect(self, *a, **k):
        pass

    def subscribe(self, topic, qos=1):
        BROKER.add_sub(self, topic)

    def publish(self, topic, payload, qos=0, retain=False):
        BROKER.publish(topic, payload)

    def loop_start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if self.on_connect:
            self.on_connect(self, None, None, 0, None)

    def loop_stop(self):
        pass

    # --- delivery ---
    def _deliver(self, topic, payload):
        self.inbox.put((topic, payload))

    def _run(self):
        while RUN.is_set():
            try:
                topic, payload = self.inbox.get(timeout=0.1)
            except queue.Empty:
                continue
            if self.on_message:
                msg = types.SimpleNamespace(topic=topic, payload=payload)
                try:
                    self.on_message(self, None, msg)
                except Exception as e:            # a callback must never die
                    print(f"    !! callback raised: {type(e).__name__}: {e}")


fake_paho = types.ModuleType("paho.mqtt.client")
fake_paho.Client = FakeClient
fake_paho.CallbackAPIVersion = types.SimpleNamespace(VERSION2=2)
sys.modules["paho"] = types.ModuleType("paho")
sys.modules["paho.mqtt"] = types.ModuleType("paho.mqtt")
sys.modules["paho.mqtt.client"] = fake_paho

import fpms_cored  # noqa: E402  (must follow the paho stub)

T = fpms_cored.THING


# ------------------------------------------------------------------ fixtures ---

def start_cored():
    t = threading.Thread(target=fpms_cored.main, daemon=True, name="cored")
    t.start()
    time.sleep(1.0)          # let both buses connect and subscribe
    return t


class FakeTeleop:
    """Stands in for fpms-teleop: nacks every command it sees.

    This is what turned the command storm into a nack storm on the rover.
    """

    def __init__(self):
        self.seen = 0
        BROKER.add_hook(f"fpms/{T}/commands/#", self._on_cmd)

    def _on_cmd(self, topic, payload):
        self.seen += 1
        verb = topic.rsplit("/", 1)[-1]
        BROKER.publish(f"fpms/{T}/events/nack",
                       json.dumps({"action": verb, "svc": "fpms-teleop",
                                   "error": "unknown or duplicate verb"}))


class HostileEcho:
    """Worst case: echoes commands back with our origin markers STRIPPED.

    This is the adversary layer 1 cannot see, so it exercises layers 2-4.
    Bounded here means bounded by cmd_id dedup and the rate caps alone.
    """

    def __init__(self, limit=500):
        self.echoed = 0
        self.limit = limit
        BROKER.add_hook(f"fpms/{T}/commands/#", self._echo)

    def _echo(self, topic, payload):
        if self.echoed >= self.limit:
            return
        try:
            d = json.loads(payload.decode() or "{}")
        except Exception:
            return
        d.pop("source", None)
        d.pop("relay", None)                 # strip the markers
        self.echoed += 1
        BROKER.publish(topic, json.dumps(d))


def settle(seconds):
    time.sleep(seconds)


def counts_for(suffix):
    return BROKER.counts.get(f"fpms/{T}/{suffix}", 0)


# --------------------------------------------------------------------- tests ---

FAILURES = []


def check(name, ok, detail):
    mark = "PASS" if ok else "FAIL"
    print(f"  [{mark}] {name}: {detail}")
    if not ok:
        FAILURES.append(f"{name}: {detail}")


def main():
    print(__doc__.split("Run:")[0].strip()[:0] or "", end="")
    print("=" * 74)
    print("fpms-cored STOP-PATH BOUNDEDNESS PROOF")
    print("=" * 74)

    teleop = FakeTeleop()
    start_cored()

    # ---------------------------------------------------------------- T1 ---
    print("\nT1  one estop, benign world")
    BROKER.counts.clear()
    BROKER.publish(f"fpms/{T}/commands/estop",
                   json.dumps({"cmd_id": "op-click-1", "source": "dashboard"}))
    settle(6.0)

    estops = counts_for("commands/estop")
    asserted = counts_for("events/stop_asserted")
    nacks = counts_for("events/nack")
    BROKER.report("T1 traffic")

    check("T1 commands/estop bounded", estops <= 2,
          f"{estops} published (original + at most one relay)")
    check("T1 stop_asserted bounded", asserted <= 2, f"{asserted} published")
    check("T1 nack bounded", nacks <= 4, f"{nacks} published")
    check("T1 stop was actually honoured", fpms_cored.STOP.latched,
          f"latched={fpms_cored.STOP.latched} count={fpms_cored.STOP.count}")
    check("T1 self-relay was dropped", fpms_cored.STOP.self_dropped >= 1,
          f"self_dropped={fpms_cored.STOP.self_dropped} (layer 1 fired)")

    # ---------------------------------------------------------------- T2 ---
    print("\nT2  adversarial echo (origin markers stripped)")
    hostile = HostileEcho(limit=500)
    BROKER.counts.clear()
    BROKER.publish(f"fpms/{T}/commands/estop",
                   json.dumps({"cmd_id": "op-click-2", "source": "dashboard"}))
    settle(8.0)

    estops2 = counts_for("commands/estop")
    total2 = BROKER.total()
    BROKER.report("T2 traffic")
    check("T2 commands/estop bounded under hostile echo", estops2 <= 25,
          f"{estops2} published (storm was 3394 in 10s)")
    check("T2 total traffic bounded", total2 <= 200,
          f"{total2} messages total")
    check("T2 latch still asserted", fpms_cored.STOP.latched,
          "stop remained latched throughout")

    # ---------------------------------------------------------------- T3 ---
    print("\nT3  unknown-verb flood (200 commands, no owner)")
    threads_before = threading.active_count()
    BROKER.counts.clear()
    for i in range(200):
        BROKER.publish(f"fpms/{T}/commands/bogus_verb",
                       json.dumps({"cmd_id": f"bogus-{i}"}))
    settle(6.0)
    threads_after = threading.active_count()
    receipts = counts_for("events/command_receipt")
    BROKER.report("T3 traffic")

    check("T3 receipts bounded (no cascade)", receipts <= 150,
          f"{receipts} receipts for 200 unknown commands")
    check("T3 no thread-per-command", threads_after - threads_before <= 10,
          f"threads {threads_before} -> {threads_after} "
          "(was one per command before the fix)")

    # -------------------------------------------------------------- verdict ---
    print("\n" + "=" * 74)
    if FAILURES:
        print(f"FAILED ({len(FAILURES)}):")
        for f in FAILURES:
            print(f"  - {f}")
    else:
        print("ALL BOUNDED. fpms-cored does not feed itself.")
    print("=" * 74)

    fpms_cored.RUNNING.clear()
    RUN.clear()
    time.sleep(0.4)
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())

"""Verify the rover agent's command handlers without an Orange Pi.

WHAT CHANGED, AND WHY THIS TEST CHANGED WITH IT
-----------------------------------------------
This file used to assert that motion commands answered "no motor interface on
this rover", that a self-test clamped speed to 0.4, and that STOP aborted that
self-test within 5s. All of that is gone, because the code it was testing was
built on a false premise.

The old handlers drove an optional Rosmaster_Lib driver — an STM32 board
protocol. This rover has a Yahboom MicroROS Board V2.0 (ESP32-S3) speaking
micro-ROS, which Rosmaster_Lib can never address, so the driver never opened and
every motion command replied "no motor interface on this rover". True of that
process; dangerously false about the rover, which drives and stops perfectly
well through fpms_teleop.py on ROS_DOMAIN_ID=20. An operator pressing STOP and
reading "no motor interface" would conclude the rover could not be halted.

So the property under test is no longer "motion refuses honestly" — it is:

    a refusal from this agent must never imply the rover is immobile,
    and this agent must never answer a command that fpms-teleop answers.

The clamp/abort assertions were not deleted for being wrong; they were deleted
because their subject moved. That discipline is now owed by fpms_teleop.py and
is documented at the command section of fpms_rover_agent.py.
"""
import sys, os, types
from unittest.mock import MagicMock

ROVER = os.path.dirname(os.path.abspath(__file__))
for name in ("cv2", "serial"):
    sys.modules[name] = types.ModuleType(name)
np = types.ModuleType("numpy"); np.ndarray = object
sys.modules["numpy"] = np
paho = types.ModuleType("paho"); pm = types.ModuleType("paho.mqtt"); pc = types.ModuleType("paho.mqtt.client")
pc.Client = MagicMock; pc.CallbackAPIVersion = MagicMock(); pc.MQTTv311 = 4
paho.mqtt = pm; pm.client = pc
sys.modules.update({"paho": paho, "paho.mqtt": pm, "paho.mqtt.client": pc})

sys.path.insert(0, ROVER)
import fpms_rover_agent as agent

FAIL = []
def check(n, ok, x=""):
    print(("  PASS  " if ok else "  FAIL  ") + n + (("   " + str(x)) if x else ""))
    if not ok: FAIL.append(n)

class RecordingBus:
    def __init__(self): self.pubs = []
    def publish(self, suffix, payload, qos=0): self.pubs.append((suffix, payload))
    def last(self, suffix=None):
        for s, p in reversed(self.pubs):
            if suffix is None or s == suffix: return s, p
        return None, None


print("1. UNOWNED motion verbs — nack must NAME THE OWNER, not imply immobility")
for action in agent.MOTION_VERBS_UNOWNED:
    bus = RecordingBus()
    agent.handle_command(bus, action, {})
    s, p = bus.last()
    blob = str(p).lower()
    check(f"{action} -> nack", s == "events/nack", f"got {s} {p}")
    check(f"{action} names fpms-teleop", "fpms-teleop" in blob, p)
    check(f"{action} names the ROS domain", "20" in blob, p)
    # The exact string the old build emitted. The dashboard special-cases it
    # (Drive.tsx maps "no motor interface" to a "no motor hardware" chip), so
    # letting it survive here would keep painting a working rover as dead.
    check(f"{action} never says 'no motor interface'",
          "no motor interface" not in blob, p)
    check(f"{action} states the rover CAN move", p.get("rover_can_move") is True, p)
    check(f"{action} is not a bare 'unknown command'",
          "unknown command" not in blob, p)
    check(f"{action} points somewhere useful", bool(p.get("hint")), p)

print("\n2. TELEOP-OWNED verbs — this agent must stay SILENT, not race the ack")
# These land in BOTH processes: this agent subscribes commands/#, teleop
# subscribes them by name. The dashboard resolves a command on the FIRST reply
# for a (thing, action) pair, and nothing guarantees teleop wins that race, so
# any publish from here could show a stop that DID happen as "refused".
for action in agent.SILENT_VERBS:
    bus = RecordingBus()
    agent.handle_command(bus, action, {})
    check(f"{action} publishes nothing from this agent", bus.pubs == [], bus.pubs)
check("stopping verbs are in the silent set",
      {"stop", "estop", "auto_off"} <= set(agent.SILENT_VERBS), agent.SILENT_VERBS)

print("\n3. SILENT_VERBS is a faithful mirror of teleop's TELEOP_ACTIONS")
# The invariant behind section 2. Checked by reading the source rather than
# importing it — fpms_teleop needs rclpy, which this agent deliberately does not
# depend on and which is not installed here.
#
# This agent's OWN verbs are exempt from the mirror: teleop's map also lists
# ping/status/connect/disconnect, but those are sensors-and-status work that
# this agent must keep answering. (That overlap is a real double-answer on
# teleop's side; it is not this file's to fix.)
OWN_VERBS = {"ping", "status", "connect", "disconnect", "restart"}
try:
    teleop_src = open(os.path.join(ROVER, "fpms_teleop.py"), encoding="utf-8").read()
except OSError as e:
    print(f"  SKIP  fpms_teleop.py unreadable ({e})")
else:
    import re
    m = re.search(r"TELEOP_ACTIONS\s*=\s*\{(.*?)\n\}", teleop_src, re.S)
    if not m:
        print("  SKIP  could not locate TELEOP_ACTIONS (teleop may have moved it)")
    else:
        subscribed = set(re.findall(r'^\s*"([a-z_]+)"\s*:', m.group(1), re.M))
        silent = set(agent.SILENT_VERBS)
        unowned = set(agent.MOTION_VERBS_UNOWNED)
        print(f"        teleop subscribes: {sorted(subscribed)}")
        # The one that actually causes a wrong reply on screen.
        check("no verb is answered by BOTH processes",
              subscribed & unowned == set(),
              f"double-answered={sorted(subscribed & unowned)}")
        # A verb teleop claims that this agent neither silences nor owns falls
        # to the generic 'unknown command' nack — and races teleop's ack with it.
        leaked = subscribed - silent - OWN_VERBS
        check("every teleop verb is silenced here or owned here",
              not leaked, f"would nack 'unknown command' over teleop's ack: {sorted(leaked)}")
        check("silent set claims nothing teleop does not subscribe",
              silent <= subscribed, f"unsubscribed={sorted(silent - subscribed)}")

print("\n4. The dead Rosmaster state is really gone, not just unused")
for name in ("MOTORS", "AUTO_MODE", "MOTOR_TEST_ABORT",
             "MOTOR_TEST_MAX_SPEED", "MOTOR_TEST_MAX_S",
             "_motors_stop", "_motor_self_test"):
    check(f"agent has no {name}", not hasattr(agent, name))
check("agent never imported rclpy", "rclpy" not in sys.modules)

print("\n5. capabilities must not advertise motors from this service")
bus_calls = []
class FakeClient:
    def subscribe(self, *a, **k): bus_calls.append(("subscribe",) + a)
real_bus = agent.Bus.__new__(agent.Bus)
real_bus.connected = False
real_bus.publish = lambda suffix, payload, qos=0: bus_calls.append((suffix, payload))
agent.Bus._on_connect(real_bus, FakeClient(), None, None, "ok")
online = [p for s, p in bus_calls if s == "events/online"]
check("events/online was published", len(online) == 1, bus_calls)
if online:
    caps = online[0].get("capabilities")
    check("capabilities excludes motors", "motors" not in caps, caps)
    check("capabilities keeps the sensors this agent really owns",
          {"camera", "lidar", "yolo"} <= set(caps), caps)

print("\n6. The real job is untouched")
bus = RecordingBus()
agent.handle_command(bus, "ping", {"n": 1})
s, p = bus.last()
check("ping -> pong", s == "events/pong", f"got {s}")
check("pong echoes the payload", p.get("echo") == {"n": 1}, p)

bus = RecordingBus()
agent.handle_command(bus, "status", {})
s, p = bus.last()
check("status -> status", s == "events/status", f"got {s}")
check("status keeps its field names",
      {"camera_ok", "lidar_ok", "frames", "scans", "uptime_s"} <= set(p), sorted(p))

bus = RecordingBus()
agent.handle_command(bus, "disconnect", {})
s, p = bus.last()
check("disconnect -> ack", s == "events/ack", f"got {s}")
check("disconnect pauses streaming", not agent.STREAM_ENABLED.is_set())
bus = RecordingBus()
agent.handle_command(bus, "connect", {})
check("connect resumes streaming", agent.STREAM_ENABLED.is_set())

bus = RecordingBus()
agent.handle_command(bus, "totally_made_up", {})
s, p = bus.last()
check("unknown command still nacks", s == "events/nack", f"got {s}")
check("unknown says unknown", p.get("error") == "unknown command", p)

print("\n7. no event subtype can trigger a fire email")
subs = {s for s, _ in bus.pubs} | {"events/ack", "events/nack", "events/pong",
                                   "events/status", "events/online"}
bad = [s for s in subs if "fire" in s.lower() or "alert" in s.lower()]
check("no subtype contains fire/alert", not bad, f"offending={bad}")

print("\n" + ("ALL PASSED" if not FAIL else f"{len(FAIL)} FAILED: {FAIL}"))
sys.exit(1 if FAIL else 0)

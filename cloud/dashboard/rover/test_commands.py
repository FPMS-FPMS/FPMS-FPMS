"""Verify the new rover command handlers without an Orange Pi.

The property that matters most: with no motor driver present (the real state of
this repo), motion commands must REFUSE, never report a success. A green ack for
a motor that never turned is worse than no button at all.
"""
import sys, os, types, time
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

print("1. NO DRIVER (the real state) — motion must refuse, not fake success")
agent.MOTORS = None
agent.AUTO_MODE.clear()

bus = RecordingBus()
agent.handle_command(bus, "test_motors", {})
s, p = bus.last()
check("test_motors -> nack", s == "events/nack", f"got {s} {p}")
check("nack says no motor interface", "motor" in str(p).lower(), p)

bus = RecordingBus()
agent.handle_command(bus, "read_encoders", {})
s, p = bus.last()
check("read_encoders -> nack", s == "events/nack", f"got {s} {p}")

bus = RecordingBus()
agent.handle_command(bus, "stop", {})
s, p = bus.last()
check("stop still acks (never gated)", s == "events/ack", f"got {s}")
check("stop reports motors_stopped False honestly", p.get("motors_stopped") is False, p)
check("stop clears auto mode", not agent.AUTO_MODE.is_set())

print("\n2. AUTO MODE latch")
bus = RecordingBus()
agent.handle_command(bus, "auto_on", {})
check("auto_on sets the latch", agent.AUTO_MODE.is_set())
bus = RecordingBus()
agent.handle_command(bus, "auto_off", {})
check("auto_off clears it", not agent.AUTO_MODE.is_set())
agent.handle_command(bus, "auto_on", {})
bus = RecordingBus()
agent.handle_command(bus, "stop", {})
check("stop overrides auto mode", not agent.AUTO_MODE.is_set())

print("\n3. WITH a stub driver — motion works AND stays bounded")
class StubMotors:
    def __init__(self): self.calls = []; self.stopped = 0
    def stop(self): self.stopped += 1; self.calls.append(("stop",))
    def drive(self, l, r): self.calls.append(("drive", l, r))
    def read_encoders(self): return {"left_ticks": 12, "right_ticks": 13}

agent.MOTORS = StubMotors()
agent.AUTO_MODE.clear()
bus = RecordingBus()
agent.handle_command(bus, "read_encoders", {})
s, p = bus.last()
check("read_encoders -> encoders with a driver", s == "events/encoders", f"got {s} {p}")

print("   running a bounded self-test with absurd inputs...")
bus = RecordingBus()
agent.MOTOR_TEST_ABORT.clear()
agent.handle_command(bus, "test_motors", {"speed": 99, "duration_s": 9999})
t0 = time.time()
# STOP must abort it promptly
time.sleep(0.4)
agent.handle_command(bus, "stop", {})
deadline = time.time() + 5
while time.time() < deadline:
    if any(s == "events/motor_test" for s, _ in bus.pubs): break
    time.sleep(0.05)
elapsed = time.time() - t0
s, p = bus.last("events/motor_test")
check("self-test terminated after STOP", s == "events/motor_test", f"got {s}")
check("terminated quickly (<5s despite duration_s=9999)", elapsed < 5, f"{elapsed:.2f}s")
speeds = [abs(c[1]) for c in agent.MOTORS.calls if c[0] == "drive"]
check("speed clamped to <=0.4", all(v <= 0.4 + 1e-9 for v in speeds), f"speeds={sorted(set(speeds))}")
check("motors were stopped", agent.MOTORS.stopped > 0, f"stop calls={agent.MOTORS.stopped}")

print("\n4. auto mode blocks a motor test")
agent.MOTORS = StubMotors()
agent.AUTO_MODE.set()
bus = RecordingBus()
agent.handle_command(bus, "test_motors", {})
s, p = bus.last()
check("test_motors refused while auto is on", s == "events/nack", f"got {s} {p}")
agent.AUTO_MODE.clear()

print("\n5. no new event subtype can trigger a fire email")
subs = {s for s, _ in bus.pubs} | {"events/ack", "events/nack", "events/encoders", "events/motor_test"}
bad = [s for s in subs if "fire" in s.lower() or "alert" in s.lower()]
check("no subtype contains fire/alert", not bad, f"offending={bad}")

print("\n" + ("ALL PASSED" if not FAIL else f"{len(FAIL)} FAILED: {FAIL}"))
sys.exit(1 if FAIL else 0)

"""Verify the Bus.publish -> CloudUplink hook without an Orange Pi.

The property under test is the ordering one: telemetry must reach the cloud
uplink even when the LOCAL MQTT broker is disconnected. If the hook were placed
after `if not self.connected: return`, the cloud would go dark at exactly the
moment the relay does -- which is the failure the direct path exists to fix.
"""
import sys, os, types
from unittest.mock import MagicMock

ROVER = os.path.dirname(os.path.abspath(__file__))

# --- stub the hardware/broker deps so the agent imports on a laptop ---
for name in ("cv2", "serial"):
    sys.modules[name] = types.ModuleType(name)

np = types.ModuleType("numpy")
np.ndarray = object
sys.modules["numpy"] = np

paho = types.ModuleType("paho")
paho_mqtt = types.ModuleType("paho.mqtt")
paho_client = types.ModuleType("paho.mqtt.client")
paho_client.Client = MagicMock
paho_client.CallbackAPIVersion = MagicMock()
paho_client.MQTTv311 = 4
paho.mqtt = paho_mqtt
paho_mqtt.client = paho_client
sys.modules["paho"] = paho
sys.modules["paho.mqtt"] = paho_mqtt
sys.modules["paho.mqtt.client"] = paho_client

sys.path.insert(0, ROVER)
import fpms_rover_agent as agent

FAIL = []
def check(name, ok, extra=""):
    print(("  PASS  " if ok else "  FAIL  ") + name + (("   " + str(extra)) if extra else ""))
    if not ok:
        FAIL.append(name)

class RecordingUplink:
    """Stands in for CloudUplink.offer, recording what it was handed."""
    def __init__(self):
        self.calls = []
    def offer(self, subtype, payload, kind="telemetry"):
        self.calls.append((subtype, dict(payload), kind))

print("1. the ordering property: broker DOWN, cloud must still get data")
up = RecordingUplink()
agent.UPLINK = up
bus = agent.Bus.__new__(agent.Bus)      # bypass __init__ (would build a client)
bus.client = MagicMock()
bus.connected = False                    # <-- local broker unreachable

bus.publish("telemetry/camera", {"frame": "AAAA", "fps": 23.6})
check("uplink received the reading while disconnected", len(up.calls) == 1,
      f"calls={len(up.calls)}")
check("MQTT publish correctly skipped when disconnected",
      bus.client.publish.call_count == 0, f"mqtt calls={bus.client.publish.call_count}")
if up.calls:
    suffix, payload, _ = up.calls[0]
    check("suffix passed through", suffix == "telemetry/camera", suffix)
    check("ts stamped before the guard", isinstance(payload.get("ts"), float), payload.get("ts"))
    check("thing stamped before the guard", payload.get("thing") == agent.THING, payload.get("thing"))

print("\n2. broker UP: both paths get it, with an identical timestamp")
up2 = RecordingUplink()
agent.UPLINK = up2
bus.client = MagicMock()
bus.connected = True
bus.publish("events/fire", {"type": "fire", "alert": True}, qos=1)
check("uplink got it", len(up2.calls) == 1)
check("MQTT got it too", bus.client.publish.call_count == 1)
if up2.calls and bus.client.publish.call_count:
    import json as _json
    mqtt_payload = _json.loads(bus.client.publish.call_args[0][1])
    check("identical ts on both paths", mqtt_payload["ts"] == up2.calls[0][1]["ts"],
          f"mqtt={mqtt_payload['ts']} cloud={up2.calls[0][1]['ts']}")
    check("MQTT topic unchanged",
          bus.client.publish.call_args[0][0] == f"fpms/{agent.THING}/events/fire",
          bus.client.publish.call_args[0][0])

print("\n3. no uplink configured: behaviour identical to before")
agent.UPLINK = None
bus.client = MagicMock()
bus.connected = True
try:
    bus.publish("telemetry/pose", {"x_m": 1.0})
    check("publishes fine with UPLINK=None", bus.client.publish.call_count == 1)
except Exception as e:
    check("publishes fine with UPLINK=None", False, repr(e))

print("\n4. a throwing uplink must not break the sensor loop")
class ExplodingUplink:
    def offer(self, *a, **k):
        raise RuntimeError("uplink exploded")
agent.UPLINK = ExplodingUplink()
bus.client = MagicMock()
bus.connected = True
try:
    bus.publish("telemetry/camera", {"frame": "x"})
    raised = False
except Exception:
    raised = True
check("exception did not escape into the caller", not raised)
check("MQTT publish still happened despite uplink failure",
      bus.client.publish.call_count == 1, f"calls={bus.client.publish.call_count}")

print("\n" + ("ALL PASSED" if not FAIL else f"{len(FAIL)} FAILED: {FAIL}"))
sys.exit(1 if FAIL else 0)

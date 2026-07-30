"""End-to-end: real Bus.publish hook -> real CloudUplink -> real Worker.

Stubs only the hardware (cv2/serial/numpy) and the MQTT client. Everything from
Bus.publish onward is the actual shipping code path, and the far end is a real
wrangler dev Worker.
"""
import sys, os, types, time, json, base64, urllib.request
from unittest.mock import MagicMock

ROVER = os.path.dirname(os.path.abspath(__file__))
BASE = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8796"
# Local test fixture, not a credential. It must match FPMS_INGEST_TOKEN in
# cloud/cloudapp/.dev.vars for the `wrangler dev` you point this at. Never run
# this against the deployed Worker with a real token.
TOKEN = os.environ.get("FPMS_TEST_INGEST_TOKEN", "dev-only-fixture-token")

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
import fpms_cloud_uplink as up

FAIL = []
def check(n, ok, x=""):
    print(("  PASS  " if ok else "  FAIL  ") + n + (("   " + str(x)) if x else ""))
    if not ok: FAIL.append(n)

def get(path):
    try:
        with urllib.request.urlopen(BASE + path, timeout=15) as r:
            return json.load(r)
    except Exception as e:
        print("   (get failed:", e, ")")
        return None

THING = "rover7"
uplink = up.CloudUplink(
    THING,
    f"{BASE}/ingest",
    BASE.replace("http", "ws") + "/ingest/ws",
    TOKEN,
    mode="ws", fps=5.0,
    spool_path=os.path.join(os.environ.get("TEMP", "."), "chain-spool.jsonl"),
)
uplink.start()
time.sleep(3)
check("uplink connected over websocket", uplink.connected_as == "ws", uplink.connected_as)

# Wire it in exactly as main() does, then publish through the REAL Bus.publish
# with the local broker deliberately marked down.
agent.UPLINK = uplink
bus = agent.Bus.__new__(agent.Bus)
bus.client = MagicMock()
bus.connected = False          # local broker DOWN

frame = base64.b64encode(b"\x00" * 3000).decode()
print("\npublishing through Bus.publish with the local broker DOWN")
bus.publish("telemetry/camera", {"frame": frame, "fps": 23.6, "fire_like": False})
bus.publish("events/fire", {"type": "fire", "ratio": 0.081, "alert": True})
time.sleep(4)

check("MQTT was skipped (broker down)", bus.client.publish.call_count == 0)

cam = get(f"/api/public/camera?thing={THING}")
check("camera frame reached the cloud", bool(cam and cam.get("frame")),
      f"len={len((cam or {}).get('frame') or '')}")

summ = get("/api/public/summary")
rover = next((r for r in (summ or {}).get("rovers", []) if r["thing"] == THING), None)
check("rover appears online in the cloud", rover and rover["status"] == "online", rover)

ev = get("/api/public/events?limit=20")
fire = next((e for e in (ev or {}).get("events", [])
             if e["thing"] == THING and e["subtype"] == "fire"), None)
check("fire event reached the cloud", bool(fire), fire)
check("event whitelisted (no frame leaked)", bool(fire) and "frame" not in fire["data"],
      fire["data"] if fire else "n/a")

st = uplink.status()
check("uplink reports items sent", (st.get("sent") or 0) > 0, f"sent={st.get('sent')}")
check("no uplink errors", (st.get("errors") or 0) == 0, f"errors={st.get('errors')}")

uplink.stop()
print("\n" + ("ALL PASSED" if not FAIL else f"{len(FAIL)} FAILED: {FAIL}"))
sys.exit(1 if FAIL else 0)

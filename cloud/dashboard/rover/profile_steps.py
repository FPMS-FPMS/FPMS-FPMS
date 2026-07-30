#!/usr/bin/env python3
"""Read-only profiler for the FPMS rover agent camera pipeline.

Does NOT touch the running service. /dev/video1 is held by the agent, so we
obtain a REAL frame by subscribing to the agent's own MQTT telemetry topic
(gives real image content, real JPEG size, real detections list). Falls back
to a synthetic scene if no telemetry arrives.
"""
import base64, json, os, statistics, sys, time

import cv2
import numpy as np
import paho.mqtt.client as mqtt

# Read from the environment — never hardcode. This file is committed.
#   export FPMS_MQTT_PASS=...   (or set it in the shell before running)
BROKER = os.environ.get("FPMS_MQTT_HOST", "192.168.137.1")
PORT = int(os.environ.get("FPMS_MQTT_PORT", "1883"))
MUSER = os.environ.get("FPMS_MQTT_USER", "fpms")
MPASS = os.environ.get("FPMS_MQTT_PASS") or ""
if not MPASS:
    raise SystemExit(
        "FPMS_MQTT_PASS is not set. Export it before running this profiler; "
        "the password is deliberately not stored in the repo."
    )
THING = "rover2"
N = 200                      # iterations per step (>= 50 required)
FIRE_MIN_AREA_PX = 2500      # from /etc/fpms/config.env
FIRE_MIN_RATIO = 0.025
JPEG_QUALITY = 55

print("cv2", cv2.__version__, "| paho", mqtt.__version__ if hasattr(mqtt, "__version__") else "?",
      "| cv2 threads", cv2.getNumThreads())


# ---------------------------------------------------------------- helpers ---
def bench(label, fn, n=N, warmup=10):
    for _ in range(warmup):
        fn()
    ts = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        ts.append((time.perf_counter() - t0) * 1000.0)
    ts.sort()
    res = {
        "label": label, "n": n,
        "mean": statistics.mean(ts),
        "median": ts[len(ts) // 2],
        "p95": ts[int(len(ts) * 0.95)],
        "min": ts[0], "max": ts[-1],
        "stdev": statistics.pstdev(ts),
    }
    RESULTS.append(res)
    print("  %-42s mean %7.3f  med %7.3f  p95 %7.3f  min %7.3f  max %7.3f" %
          (label, res["mean"], res["median"], res["p95"], res["min"], res["max"]))
    return res


RESULTS = []


# ------------------------------------------------- exact agent detect_fire ---
def detect_fire(frame):
    """Verbatim copy of detect_fire() from /usr/local/bin/fpms-rover-agent."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = cv2.inRange(hsv, np.array([0, 120, 200]), np.array([25, 255, 255]))
    upper = cv2.inRange(hsv, np.array([160, 120, 200]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    total = frame.shape[0] * frame.shape[1]
    ratio = float(cv2.countNonZero(mask)) / max(total, 1)

    boxes = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in sorted(contours, key=cv2.contourArea, reverse=True)[:3]:
        area = cv2.contourArea(cnt)
        if area >= FIRE_MIN_AREA_PX:
            x, y, w, h = cv2.boundingRect(cnt)
            boxes.append({"box": [int(x), int(y), int(x + w), int(y + h)],
                          "area_px": int(area)})
    return (bool(boxes) and ratio >= FIRE_MIN_RATIO), round(ratio, 5), boxes


# ---------------------------------------------- acquire a real frame (MQTT) ---
GOT = {}


def grab_real_frame(timeout=20):
    try:
        c = mqtt.Client(client_id="fpms-profiler-sub",
                        callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        c = mqtt.Client(client_id="fpms-profiler-sub")

    def on_connect(cl, u, f, rc, p=None):
        cl.subscribe(f"fpms/{THING}/telemetry/camera", qos=0)

    def on_message(cl, u, msg):
        if GOT:
            return
        try:
            d = json.loads(msg.payload.decode())
            GOT["payload"] = d
            GOT["raw_len"] = len(msg.payload)
        except Exception as e:
            print("  decode failed:", e)

    c.username_pw_set(MUSER, MPASS)
    c.on_connect, c.on_message = on_connect, on_message
    c.connect(BROKER, PORT, 30)
    c.loop_start()
    t0 = time.time()
    while not GOT and time.time() - t0 < timeout:
        time.sleep(0.05)
    c.loop_stop()
    c.disconnect()
    return GOT.get("payload")


def synth_frame():
    rng = np.random.default_rng(7)
    f = rng.integers(20, 90, (480, 640, 3), dtype=np.uint8)
    for (cx, cy, r) in [(150, 200, 55), (400, 300, 45), (520, 130, 38)]:
        cv2.circle(f, (cx, cy), r, (30, 120, 250), -1)
        cv2.circle(f, (cx, cy), r // 2, (60, 190, 255), -1)
    return f


print("\n[1] Acquiring a real frame from the agent's MQTT telemetry ...")
payload = grab_real_frame()
if payload and "frame" in payload:
    jpeg_bytes = base64.b64decode(payload["frame"])
    frame = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
    detections = payload.get("detections", [])
    SRC = "REAL frame from fpms/%s/telemetry/camera" % THING
    print("  got real telemetry: payload %d B, jpeg %d B, frame %s, %d detections"
          % (GOT["raw_len"], len(jpeg_bytes), frame.shape, len(detections)))
else:
    frame = synth_frame()
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
    jpeg_bytes = buf.tobytes()
    detections = []
    SRC = "SYNTHETIC frame (no MQTT telemetry received)"
    print("  NO telemetry; using synthetic frame, jpeg %d B" % len(jpeg_bytes))

if not detections:
    detections = [
        {"cls": 0, "label": "person", "conf": 0.83, "box": [100, 120, 210, 380], "kind": "person"},
        {"cls": 21, "label": "bear", "conf": 0.61, "box": [320, 150, 470, 330], "kind": "wildlife"},
        {"cls": -1, "label": "fire-like", "conf": 0.44, "box": [480, 90, 600, 210], "kind": "fire"},
    ]
print("  frame source:", SRC)
print("  jpeg size: %.1f KB | detections used: %d" % (len(jpeg_bytes) / 1024.0, len(detections)))

# Encode a fresh JPEG from this frame at the agent's quality to keep sizes honest
ok, ebuf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
enc_bytes = ebuf.tobytes()
b64_str = base64.b64encode(enc_bytes).decode("ascii")

full_payload = {
    "format": "jpeg", "frame": b64_str,
    "width": int(frame.shape[1]), "height": int(frame.shape[0]),
    "detections": detections, "npu": True,
    "fire_like": False, "fire_ratio": 0.0031,
    "wildlife": ["bear"],
    "ts": time.time(), "thing": THING,
}
payload_json = json.dumps(full_payload)
print("  re-encoded jpeg %.1f KB | b64 %.1f KB | full json payload %.1f KB"
      % (len(enc_bytes) / 1024.0, len(b64_str) / 1024.0, len(payload_json) / 1024.0))

fl, fr, fb = detect_fire(frame)
print("  detect_fire on this frame -> fire_like=%s ratio=%s boxes=%d" % (fl, fr, len(fb)))

# ------------------------------------------------------ MQTT pub connection ---
try:
    pub = mqtt.Client(client_id="fpms-profiler-pub",
                      callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
except AttributeError:
    pub = mqtt.Client(client_id="fpms-profiler-pub")
pub.username_pw_set(MUSER, MPASS)
pub.connect(BROKER, PORT, 30)
pub.loop_start()
time.sleep(0.7)
print("  publisher connected:", pub.is_connected())

TOPIC = "profiling/scratch"   # outside fpms/# so the dashboard is unaffected

# ------------------------------------------------------------- benchmarks ---
print("\n[2] Per-step timings (n=%d, service still running -> real contention)\n" % N)

bench("1. detect_fire() full 640x480", lambda: detect_fire(frame))

# sub-steps of detect_fire
hsv_c = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
lo = cv2.inRange(hsv_c, np.array([0, 120, 200]), np.array([25, 255, 255]))
up = cv2.inRange(hsv_c, np.array([160, 120, 200]), np.array([180, 255, 255]))
mask_c = cv2.bitwise_or(lo, up)
K = np.ones((5, 5), np.uint8)
bench("   1a. cvtColor BGR2HSV", lambda: cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
bench("   1b. inRange x2 + bitwise_or",
      lambda: cv2.bitwise_or(cv2.inRange(hsv_c, np.array([0, 120, 200]), np.array([25, 255, 255])),
                             cv2.inRange(hsv_c, np.array([160, 120, 200]), np.array([180, 255, 255]))))
bench("   1c. morphologyEx MORPH_OPEN 5x5",
      lambda: cv2.morphologyEx(mask_c, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)))
bench("   1d. countNonZero", lambda: cv2.countNonZero(mask_c))
bench("   1e. findContours + sort",
      lambda: sorted(cv2.findContours(mask_c, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0],
                     key=cv2.contourArea, reverse=True)[:3])

bench("2. base64.b64encode(%.1fKB jpeg)" % (len(enc_bytes) / 1024.0),
      lambda: base64.b64encode(enc_bytes))
bench("   2b. b64encode + .decode('ascii')",
      lambda: base64.b64encode(enc_bytes).decode("ascii"))

bench("3. json.dumps(payload w/ b64 + dets)", lambda: json.dumps(full_payload))

bench("4. client.publish(%.1fKB, qos=0)" % (len(payload_json) / 1024.0),
      lambda: pub.publish(TOPIC, payload_json, qos=0), n=N)

bench("   4b. bus.publish() total (dumps+publish)",
      lambda: pub.publish(TOPIC, json.dumps(full_payload), qos=0))


def overlay():
    for d in detections[:3]:
        x1, y1, x2, y2 = d["box"]
        colour = {"wildlife": (0, 255, 255), "vegetation": (0, 220, 0),
                  "person": (255, 180, 0), "fire": (0, 0, 255)}.get(d["kind"], (0, 100, 255))
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        cv2.putText(frame, f"{d['label']} {d['conf']:.2f}",
                    (x1, max(12, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, colour, 1, cv2.LINE_AA)


bench("5. cv2.rectangle+putText x3 overlay", overlay)

# ---------------------------------------------------- cross-check / extras ---
print("")
bench("X. cv2.imencode jpeg q%d (cross-check)" % JPEG_QUALITY,
      lambda: cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]))

# candidate optimisation A: detect_fire on a half-res copy
small = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_NEAREST)


def detect_fire_half():
    s = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_NEAREST)
    return detect_fire(s)


bench("OPT-A. detect_fire @320x240 (incl resize)", detect_fire_half)

# candidate optimisation B: detect_fire only on inference frames (no cost change,
# measured as the per-2-frame amortised value) -> reported in analysis.

# candidate optimisation C: skip base64, publish raw jpeg bytes on its own topic
bench("OPT-C. publish raw jpeg bytes (no b64/json)",
      lambda: pub.publish(TOPIC, enc_bytes, qos=0))

# candidate optimisation D: lower jpeg quality
for q in (35, 45):
    bench("OPT-D. imencode jpeg q%d" % q,
          lambda q=q: cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), q]))
okq, bq = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 35])
print("   (jpeg q35 size %.1f KB vs q55 %.1f KB)" % (len(bq.tobytes()) / 1024.0,
                                                     len(enc_bytes) / 1024.0))

# what does the whole measured tail cost per frame?
print("\n[3] Summary (mean ms)")
by = {r["label"]: r["mean"] for r in RESULTS}
core = ["1. detect_fire() full 640x480",
        "2. base64.b64encode(%.1fKB jpeg)" % (len(enc_bytes) / 1024.0),
        "3. json.dumps(payload w/ b64 + dets)",
        "4. client.publish(%.1fKB, qos=0)" % (len(payload_json) / 1024.0),
        "5. cv2.rectangle+putText x3 overlay"]
tot = 0.0
for k in core:
    print("  %-42s %7.3f" % (k, by[k]))
    tot += by[k]
print("  %-42s %7.3f" % ("SUBTOTAL of the 5 unprofiled steps", tot))
print("  %-42s %7.3f" % ("known baseline (cap+pre+infer/2+decode/2+jpeg)",
                         1.4 + 1.6 + 20.1 / 2 + 10.8 / 2 + 1.6))
print("  %-42s %7.3f" % ("=> modelled frame time", tot + 1.4 + 1.6 + 20.1 / 2 + 10.8 / 2 + 1.6))

pub.loop_stop()
pub.disconnect()
print("\nDONE. frame source:", SRC)

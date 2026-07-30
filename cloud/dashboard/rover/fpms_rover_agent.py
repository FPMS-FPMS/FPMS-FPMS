#!/usr/bin/env python3
"""FPMS rover agent — streams camera + LiDAR to the FPMS dashboard over MQTT.

Runs on the Orange Pi 5B. Three threads:

  camera : YOLOv8 on the RK3588 NPU, publishes annotated detections + JPEG
  lidar  : LD-series serial scanner, publishes a sector summary
  main   : MQTT connection, command subscription, liveness

Reuses /home/ubuntu/yolo/fpms_yolo_npu.py for inference rather than
reimplementing it — that decode path (DFL + NMS, tuned thresholds) is already
proven on this hardware, and a second copy would drift from it.

Config lives in /etc/fpms/config.env.
"""
from __future__ import annotations

import base64
import json
import math
import os
import signal
import sys
import threading
import time

import cv2
import numpy as np
import serial
from paho.mqtt.client import Client, CallbackAPIVersion

# ---------------------------------------------------------------- config ---

def load_config(path="/etc/fpms/config.env"):
    cfg = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    cfg[k] = v
    except FileNotFoundError:
        pass
    return cfg


CFG = load_config()
THING = CFG.get("FPMS_THING_NAME", "rover2")

# Optional direct-to-cloud uplink, alongside MQTT rather than instead of it.
#
# Set at startup by main(); stays None unless FPMS_CLOUD_UPLINK=1 is in
# /etc/fpms/config.env AND the module is deployed next to this file. Both the
# import and the construction are allowed to fail quietly: this agent's job is
# to keep a camera and a LiDAR running, and a missing optional uplink must
# never be the reason it does not start.
UPLINK = None
BROKER = CFG.get("FPMS_MQTT_HOST", "192.168.137.1")
PORT = int(CFG.get("FPMS_MQTT_PORT", "1883"))
USER = CFG.get("FPMS_MQTT_USER", "")
PASS = CFG.get("FPMS_MQTT_PASS", "")

CAMERA_DEV = int(CFG.get("FPMS_CAMERA_INDEX", "0"))
CAMERA_FPS = float(CFG.get("FPMS_CAMERA_FPS", "2"))       # frames published/sec
JPEG_QUALITY = int(CFG.get("FPMS_JPEG_QUALITY", "60"))
LIDAR_PORT = CFG.get("FPMS_LIDAR_PORT", "/dev/ttyUSB0")
LIDAR_BAUD = int(CFG.get("FPMS_LIDAR_BAUD", "230400"))
LIDAR_HZ = float(CFG.get("FPMS_LIDAR_HZ", "2"))           # summaries/sec
YOLO_DIR = CFG.get("FPMS_YOLO_DIR", "/home/ubuntu/yolo")

# A detection this close in front is worth an event, not just telemetry.
OBSTACLE_ALERT_MM = int(CFG.get("FPMS_OBSTACLE_ALERT_MM", "400"))

RUNNING = threading.Event()
RUNNING.set()

COCO_TREE_IDS = {58, 50, 75}   # potted plant / broccoli / vase — stand-ins for foliage

# Full COCO-80 names. The model returns class indices; without this the UI shows
# "class_57" and an operator has to look up what the rover actually saw.
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]

# Animals COCO can genuinely detect. Wildlife presence matters for a fire
# patrol: animals fleeing an area is an early signal, and a bear near the
# rover is an operational hazard in its own right.
WILDLIFE_IDS = {14: "bird", 15: "cat", 16: "dog", 17: "horse", 18: "sheep",
                19: "cow", 20: "elephant", 21: "bear", 22: "zebra", 23: "giraffe"}

# Fire is NOT a COCO class, so detection cannot come from the model. This is a
# classical HSV heuristic: flame is bright, saturated and red/orange. It runs in
# ~2 ms on CPU and is deliberately a *screen*, not a verdict — a hit raises an
# event that the thermal sensor and the cloud VLM then corroborate.
FIRE_MIN_AREA_PX = int(CFG.get("FPMS_FIRE_MIN_AREA", "900"))
FIRE_MIN_RATIO = float(CFG.get("FPMS_FIRE_MIN_RATIO", "0.004"))


def detect_fire(frame):
    """Return (fire_like, ratio, boxes) for flame-coloured regions.

    Runs on a quarter-scale copy. The HSV convert, morphological open and
    contour pass are all O(pixels), so 320x240 costs a quarter of 640x480 while
    losing nothing that matters — a flame large enough to act on is still tens
    of pixels across at half scale. Boxes are scaled back to full resolution.
    """
    small = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2),
                       interpolation=cv2.INTER_NEAREST)
    scale = 2
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    # Two hue bands: red wraps around 0/180 in OpenCV's scale.
    lower = cv2.inRange(hsv, np.array([0, 120, 200]), np.array([25, 255, 255]))
    upper = cv2.inRange(hsv, np.array([160, 120, 200]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    total = small.shape[0] * small.shape[1]
    ratio = float(cv2.countNonZero(mask)) / max(total, 1)

    boxes = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Area threshold is expressed in full-resolution pixels, so compare against
    # the downscaled area multiplied back up.
    for cnt in sorted(contours, key=cv2.contourArea, reverse=True)[:3]:
        area = cv2.contourArea(cnt) * scale * scale
        if area >= FIRE_MIN_AREA_PX:
            x, y, w, h = cv2.boundingRect(cnt)
            boxes.append({"box": [int(x * scale), int(y * scale),
                                  int((x + w) * scale), int((y + h) * scale)],
                          "area_px": int(area)})

    return (bool(boxes) and ratio >= FIRE_MIN_RATIO), round(ratio, 5), boxes


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


# ------------------------------------------------------------------ MQTT ---

class Bus:
    """Thin MQTT wrapper. Publishing must never raise into a sensor loop."""

    def __init__(self):
        self.client = Client(client_id=f"{THING}-agent",
                             callback_api_version=CallbackAPIVersion.VERSION2)
        if USER:
            self.client.username_pw_set(USER, PASS or None)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        # Last will: if this agent dies, the dashboard finds out from the broker
        # rather than waiting for telemetry to go stale.
        self.client.will_set(f"fpms/{THING}/events/offline",
                             json.dumps({"thing": THING, "status": "offline",
                                         "reason": "unexpected disconnect"}),
                             qos=1, retain=False)
        self.connected = False

    def connect_forever(self):
        while RUNNING.is_set():
            try:
                self.client.connect(BROKER, PORT, keepalive=30)
                self.client.loop_start()
                return
            except Exception as e:
                log(f"MQTT connect failed ({e}); retrying in 5s")
                time.sleep(5)

    def _on_connect(self, client, _u, _f, reason_code, _p=None):
        self.connected = True
        log(f"MQTT connected to {BROKER}:{PORT} ({reason_code})")
        client.subscribe(f"fpms/{THING}/commands/#", qos=1)
        self.publish("events/online", {
            "thing": THING, "status": "online",
            "camera": CAMERA_DEV, "lidar": LIDAR_PORT,
            "capabilities": ["camera", "lidar", "yolo"],
        }, qos=1)

    def _on_message(self, _c, _u, msg):
        action = msg.topic.rsplit("/", 1)[-1]
        try:
            payload = json.loads(msg.payload.decode() or "{}")
        except Exception:
            payload = {}
        handle_command(self, action, payload)

    def publish(self, suffix, payload, qos=0):
        # Stamp defaults before either path reads the payload, so the MQTT copy
        # and the cloud copy carry an identical ts rather than two clock reads
        # a few milliseconds apart.
        try:
            payload.setdefault("ts", time.time())
            payload.setdefault("thing", THING)
        except Exception:
            # A non-dict payload is a caller bug, but it must not stop the
            # publish attempt below from reporting it properly.
            pass

        # Cloud uplink FIRST, and deliberately ABOVE the connectivity guard.
        #
        # This ordering is the whole point of the direct path. `if not
        # self.connected: return` short-circuits whenever the LOCAL broker is
        # unreachable, so offering after it would mean the cloud goes dark at
        # exactly the moment the laptop relay does — the failure this uplink
        # exists to fix. Hooking here also covers every existing call site
        # (camera, lidar, heartbeat and all events/*) with no change to the
        # sensor loops.
        #
        # offer() is a queue append and already swallows its own errors, but it
        # is wrapped again here on purpose: this runs in the camera hot loop, and
        # an OPTIONAL uplink must never be able to interrupt the primary MQTT
        # path — not even if a future change to offer() starts raising.
        if UPLINK is not None:
            try:
                UPLINK.offer(suffix, payload)
            except Exception as e:  # noqa: BLE001
                log(f"cloud uplink offer failed on {suffix}: {e}")

        if not self.connected:
            return
        try:
            self.client.publish(f"fpms/{THING}/{suffix}", json.dumps(payload), qos=qos)
        except Exception as e:
            log(f"publish failed on {suffix}: {e}")


def handle_command(bus, action, payload):
    """Commands arrive on fpms/<thing>/commands/<action>."""
    log(f"command: {action} {payload}")
    if action == "ping":
        bus.publish("events/pong", {"echo": payload}, qos=1)
    elif action == "status":
        bus.publish("events/status", {
            "camera_ok": CAMERA_STATE.get("ok"), "lidar_ok": LIDAR_STATE.get("ok"),
            "frames": CAMERA_STATE.get("frames"), "scans": LIDAR_STATE.get("scans"),
            "uptime_s": round(time.time() - STARTED, 1),
        }, qos=1)
    elif action in ("connect", "disconnect"):
        # The dashboard's connect/disconnect buttons. Streaming is the default
        # state; disconnect pauses publishing without killing the service, so a
        # reconnect does not have to wait for systemd and a fresh NPU load.
        STREAM_ENABLED.set() if action == "connect" else STREAM_ENABLED.clear()
        bus.publish("events/ack", {"action": action,
                                   "streaming": STREAM_ENABLED.is_set()}, qos=1)
    elif action == "restart":
        bus.publish("events/ack", {"action": "restart"}, qos=1)
        time.sleep(0.5)
        RUNNING.clear()
    else:
        bus.publish("events/nack", {"action": action, "error": "unknown command"}, qos=1)


STREAM_ENABLED = threading.Event()
STREAM_ENABLED.set()
STARTED = time.time()
CAMERA_STATE = {"ok": False, "frames": 0, "device": None}
LIDAR_STATE = {"ok": False, "scans": 0}
# Obstacle alerts are edge-triggered; this re-reminds at most every 5 minutes
# while the obstruction persists.
OBSTACLE_STATE = {"active": False, "last_sent": 0.0}
OBSTACLE_REPEAT_S = float(CFG.get("FPMS_OBSTACLE_REPEAT_S", "300"))
FIRE_STATE = {"active": False, "last_sent": 0.0, "hits": 0, "misses": 0}
FIRE_REPEAT_S = float(CFG.get("FPMS_FIRE_REPEAT_S", "120"))
# Fire must persist ~0.2s to trigger and ~2s to clear at 22 fps. Asymmetric on
# purpose: slow to raise an alarm, slower still to declare it over.
FIRE_ON_FRAMES = int(CFG.get("FPMS_FIRE_ON_FRAMES", "5"))
FIRE_OFF_FRAMES = int(CFG.get("FPMS_FIRE_OFF_FRAMES", "45"))

# Run YOLO on every Nth frame. 1 = every frame (video capped at ~28 fps by
# inference); 2 = video at the sensor's 30 fps with detections at ~15 Hz.
INFER_EVERY = max(1, int(CFG.get("FPMS_INFER_EVERY", "2")))
FIRE_EVERY = max(1, int(CFG.get("FPMS_FIRE_EVERY", "2")))

# Which detector to load. "v8" keeps the proven yolov8n path; "v26" switches to
# yolo26n-rk3588.rknn, whose head folds DFL into the graph and so needs its own
# decode. Default stays v8 until v26 is benchmarked on the board.
YOLO_VARIANT = CFG.get("FPMS_YOLO_VARIANT", "v8").strip().lower()
LAST_DETECTIONS: list = []
LAST_FIRE: list = [False, 0.0, []]


# ---------------------------------------------------------------- camera ---

def find_camera(preferred=None):
    """Open the first /dev/video* that actually yields a frame.

    A fixed index is not safe here: this camera drops off the USB bus and
    re-enumerates (dmesg shows `device not accepting address, error -71`), and
    when it comes back the kernel may hand it a different node — /dev/video0
    became /dev/video1 mid-run. Several nodes also exist that are metadata or
    hardware-codec devices and open successfully while never producing frames,
    so "opens" is not good enough; we require a real read.
    """
    candidates = []
    if preferred is not None:
        candidates.append(preferred)
    try:
        for name in sorted(os.listdir("/dev")):
            if name.startswith("video") and name[5:].isdigit():
                idx = int(name[5:])
                if idx not in candidates:
                    candidates.append(idx)
    except OSError:
        pass

    for idx in candidates:
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ok, frame = cap.read()
        if ok and frame is not None and frame.size:
            log(f"camera found on /dev/video{idx} ({frame.shape[1]}x{frame.shape[0]})")
            return cap, idx
        cap.release()
    return None, None


def camera_loop(bus):
    """YOLO on the NPU, published as JPEG + detections."""
    sys.path.insert(0, YOLO_DIR)
    rknn = None
    decode = None
    labels = {}

    try:
        if YOLO_VARIANT in ("v26", "26", "yolo26"):
            import fpms_yolo26_npu as ynpu    # NMS-free head, own decode
        else:
            import fpms_yolo_npu as ynpu      # their proven decode path
        from rknnlite.api import RKNNLite

        decode = ynpu.decode
        rknn = RKNNLite()
        if rknn.load_rknn(ynpu.MODEL) != 0:
            raise RuntimeError(f"load_rknn failed for {ynpu.MODEL}")
        if rknn.init_runtime() != 0:
            raise RuntimeError("init_runtime failed")
        log(f"NPU ready with {ynpu.MODEL} (variant {YOLO_VARIANT})")
        try:
            labels = json.load(open(os.path.join(YOLO_DIR, "custom_labels.json")))
        except Exception:
            labels = {}
    except Exception as e:
        # Degrade to plain streaming rather than losing the camera entirely —
        # a video feed with no boxes still lets an operator see the fire.
        log(f"NPU unavailable ({e}); streaming without detection")
        rknn = None

    cap, dev = find_camera(CAMERA_DEV)
    if cap is None:
        log("no working camera device found")
        bus.publish("events/fault", {"component": "camera", "alert": True,
                                     "error": "no working /dev/video* device"}, qos=1)
    else:
        CAMERA_STATE["ok"] = True
        CAMERA_STATE["device"] = dev

    # Ask the driver for MJPG at the target rate. Without MJPG the camera falls
    # back to raw YUYV, which cannot sustain 30 fps over USB 2.0 at 640x480.
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # always read the freshest frame
        log(f"camera negotiated: {cap.get(cv2.CAP_PROP_FPS)} fps, "
            f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    except Exception as e:
        log(f"could not set capture properties: {e}")

    interval = 1.0 / max(CAMERA_FPS, 0.1)
    consecutive_failures = 0
    reopen_backoff = 2.0
    frame_index = 0

    while RUNNING.is_set():
        t0 = time.time()

        # No camera (or it vanished): keep trying to find it again instead of
        # spinning silently. The old code hit `continue` forever and the panel
        # just said "waiting for feed" with nothing in the logs.
        if cap is None:
            time.sleep(reopen_backoff)
            cap, dev = find_camera(CAMERA_DEV)
            if cap is not None:
                CAMERA_STATE["ok"] = True
                CAMERA_STATE["device"] = dev
                consecutive_failures = 0
                reopen_backoff = 2.0
                bus.publish("events/camera_recovered", {"device": f"/dev/video{dev}"}, qos=1)
                log(f"camera recovered on /dev/video{dev}")
            else:
                reopen_backoff = min(reopen_backoff * 1.5, 30.0)
            continue

        ok, frame = cap.read()
        if not ok or frame is None:
            consecutive_failures += 1
            # Five straight failures means the device is gone, not a hiccup —
            # a USB re-enumeration can move it to a different node entirely.
            if consecutive_failures >= 5:
                log(f"camera read failed {consecutive_failures}x; reopening")
                CAMERA_STATE["ok"] = False
                bus.publish("events/fault", {"component": "camera", "alert": True,
                                             "error": "camera stopped delivering frames"}, qos=1)
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
            time.sleep(0.4)
            continue

        consecutive_failures = 0
        if not STREAM_ENABLED.is_set():
            time.sleep(interval)
            continue

        # Inference is decoupled from capture. The camera tops out at 30 fps
        # (v4l2 reports no faster interval at any resolution) but a full YOLO
        # pass costs ~31 ms, so running it on every frame caps the video at ~28.
        # Running it every Nth frame and reusing the most recent boxes lets the
        # footage run at the sensor's real limit while detections stay current
        # to within a frame or two.
        frame_index += 1
        run_inference = (frame_index % INFER_EVERY == 0)

        detections = list(LAST_DETECTIONS)
        if run_inference and rknn is not None and decode is not None:
            detections = []
            try:
                ih, iw = frame.shape[:2]
                img = cv2.resize(frame, (640, 640))
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                outputs = rknn.inference(inputs=[np.expand_dims(img, 0)])
                boxes, scores, classes = decode(outputs, ih, iw)
                for b, s, c in zip(boxes, scores, classes):
                    x1, y1, x2, y2 = [int(v) for v in b]
                    cid = int(c)
                    # Operator overrides first, then the real COCO name.
                    name = labels.get(str(cid)) or (
                        COCO_NAMES[cid] if 0 <= cid < len(COCO_NAMES) else f"class_{cid}")
                    if cid in WILDLIFE_IDS:
                        kind, colour = "wildlife", (0, 255, 255)
                    elif cid in COCO_TREE_IDS:
                        kind, colour = "vegetation", (0, 220, 0)
                    elif cid == 0:
                        kind, colour = "person", (255, 180, 0)
                    else:
                        kind, colour = "object", (0, 100, 255)
                    detections.append({
                        "cls": cid, "label": name, "conf": round(float(s), 3),
                        "box": [x1, y1, x2, y2], "kind": kind,
                    })
                    cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                    cv2.putText(frame, f"{name} {s:.2f}",
                                (x1, max(12, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.45, colour, 1, cv2.LINE_AA)
                LAST_DETECTIONS[:] = detections
            except Exception as e:
                log(f"inference failed: {e}")
        elif detections:
            # Redraw the carried-over boxes so the video still shows them.
            for d in detections:
                x1, y1, x2, y2 = d["box"]
                colour = {"wildlife": (0, 255, 255), "vegetation": (0, 220, 0),
                          "person": (255, 180, 0), "fire": (0, 0, 255)}.get(
                              d["kind"], (0, 100, 255))
                cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                cv2.putText(frame, f"{d['label']} {d['conf']:.2f}",
                            (x1, max(12, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, colour, 1, cv2.LINE_AA)

        # Fire screen is the only heavy CV step INFER_EVERY did not halve, and
        # it was running on every frame at full resolution. It now runs on the
        # same cadence as inference and reuses its last result in between; the
        # hysteresis counters below already require several consecutive frames,
        # so sampling costs no responsiveness that matters.
        if frame_index % FIRE_EVERY == 0:
            fire_like, fire_ratio, fire_boxes = detect_fire(frame)
            LAST_FIRE[:] = [fire_like, fire_ratio, fire_boxes]
        else:
            fire_like, fire_ratio, fire_boxes = LAST_FIRE
        for fb in fire_boxes:
            x1, y1, x2, y2 = fb["box"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.putText(frame, "FIRE?", (x1, max(14, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            detections.append({
                "cls": -1, "label": "fire-like", "conf": round(min(fire_ratio * 50, 0.99), 3),
                "box": fb["box"], "kind": "fire",
            })

        wildlife = [d for d in detections if d["kind"] == "wildlife"]
        if wildlife:
            bus.publish("events/wildlife", {
                "alert": False,
                "species": sorted({d["label"] for d in wildlife}),
                "count": len(wildlife),
                "detail": "wildlife in frame: " + ", ".join(sorted({d["label"] for d in wildlife})),
            }, qos=1)

        # Hysteresis. A borderline frame alternating true/false at 22 fps emitted
        # a fire + fire_cleared pair every cycle — 173 of each in one window.
        # Require several consecutive frames each way before changing state.
        if fire_like:
            FIRE_STATE["hits"] += 1
            FIRE_STATE["misses"] = 0
        else:
            FIRE_STATE["misses"] += 1
            FIRE_STATE["hits"] = 0
        fire_like = (FIRE_STATE["hits"] >= FIRE_ON_FRAMES) if not FIRE_STATE["active"] \
            else (FIRE_STATE["misses"] < FIRE_OFF_FRAMES)

        # Edge-triggered like the obstacle alert, so a sunset doesn't spam.
        now = time.time()
        if fire_like and (not FIRE_STATE["active"]
                          or now - FIRE_STATE["last_sent"] >= FIRE_REPEAT_S):
            FIRE_STATE["active"] = True
            FIRE_STATE["last_sent"] = now
            bus.publish("events/fire", {
                "alert": True,
                "source": "onboard-hsv-screen",
                "coverage_ratio": fire_ratio,
                "regions": fire_boxes,
                "detail": f"flame-coloured region covering {fire_ratio*100:.2f}% of frame — "
                          f"needs thermal/VLM corroboration",
            }, qos=1)
        elif not fire_like and FIRE_STATE["active"]:
            FIRE_STATE["active"] = False
            bus.publish("events/fire_cleared", {"alert": False, "detail": "no flame colours"}, qos=1)

        ok, buf = cv2.imencode(".jpg", frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ok:
            CAMERA_STATE["frames"] += 1
            bus.publish("telemetry/camera", {
                "format": "jpeg",
                "frame": base64.b64encode(buf.tobytes()).decode("ascii"),
                "width": int(frame.shape[1]), "height": int(frame.shape[0]),
                "detections": detections,
                "npu": rknn is not None,
                "fire_like": fire_like,
                "fire_ratio": fire_ratio,
                "wildlife": sorted({d["label"] for d in wildlife}),
            })

        time.sleep(max(0.0, interval - (time.time() - t0)))

    cap.release()
    if rknn is not None:
        try:
            rknn.release()
        except Exception:
            pass


# ----------------------------------------------------------------- lidar ---

LD_HEADER = 0x54
LD_VERLEN = 0x2C
LD_POINTS = 12
LD_FRAME = 47          # 1+1+2+2+12*3+2+2+1


def parse_ld_frame(buf):
    """Decode one LD06/LD19 frame. Returns (start_deg, end_deg, [(dist_mm, intensity)])."""
    speed = int.from_bytes(buf[2:4], "little")
    start = int.from_bytes(buf[4:6], "little") / 100.0
    points = []
    off = 6
    for _ in range(LD_POINTS):
        dist = int.from_bytes(buf[off:off + 2], "little")
        inten = buf[off + 2]
        points.append((dist, inten))
        off += 3
    end = int.from_bytes(buf[off:off + 2], "little") / 100.0
    return speed, start, end, points


def lidar_loop(bus):
    """Read the scanner and publish a 12-sector summary."""
    ser = None
    for baud in (LIDAR_BAUD, 115200, 460800):
        try:
            ser = serial.Serial(LIDAR_PORT, baud, timeout=1)
            time.sleep(0.3)
            probe = ser.read(2048)
            if probe.count(bytes([LD_HEADER, LD_VERLEN])) >= 3:
                log(f"lidar sync at {baud} baud on {LIDAR_PORT}")
                LIDAR_STATE["baud"] = baud
                break
            ser.close()
            ser = None
        except Exception as e:
            log(f"lidar open failed at {baud}: {e}")
            ser = None

    if ser is None:
        log("lidar: no recognisable LD frames at any baud")
        bus.publish("events/fault",
                    {"component": "lidar", "error": "no LD frames; check model/baud"}, qos=1)
        return

    LIDAR_STATE["ok"] = True
    # Two resolutions from the same scan:
    #   BINS (72 x 5°)   -> ranges_m, what the dashboard's radar canvas draws
    #   SECTORS (12x30°) -> coarse obstacle logic and alerting
    # The UI contract is ranges_m + range_max_m; sending only sectors_mm left
    # ranges_m undefined and the canvas threw on every scan.
    SECTORS = 12
    # One bin per degree. The dashboard canvas maps array index directly to an
    # angle (`(i - 90) * PI / 180`), so any other length silently draws a wedge
    # instead of a full scan — it looks like a broken sensor, not a unit bug.
    BINS = 360
    RANGE_MAX_M = 6.0
    bin_min = [None] * BINS
    sector_min = [None] * SECTORS
    points_seen = 0
    last_emit = time.time()
    interval = 1.0 / max(LIDAR_HZ, 0.1)
    buf = bytearray()

    while RUNNING.is_set():
        try:
            chunk = ser.read(512)
        except Exception as e:
            log(f"lidar read error: {e}")
            time.sleep(1)
            continue
        if chunk:
            buf.extend(chunk)

        # Frame sync: find header pairs and consume complete frames.
        while len(buf) >= LD_FRAME:
            i = buf.find(bytes([LD_HEADER, LD_VERLEN]))
            if i < 0:
                del buf[:-1]
                break
            if len(buf) - i < LD_FRAME:
                del buf[:i]
                break
            frame = bytes(buf[i:i + LD_FRAME])
            del buf[:i + LD_FRAME]
            try:
                _speed, start, end, pts = parse_ld_frame(frame)
            except Exception:
                continue

            span = (end - start) % 360.0
            step = span / max(len(pts) - 1, 1)
            for n, (dist, inten) in enumerate(pts):
                if dist <= 0:
                    continue
                ang = (start + step * n) % 360.0
                s = int(ang // (360 / SECTORS)) % SECTORS
                if sector_min[s] is None or dist < sector_min[s]:
                    sector_min[s] = dist
                b = int(ang // (360 / BINS)) % BINS
                if bin_min[b] is None or dist < bin_min[b]:
                    bin_min[b] = dist
                points_seen += 1

        if time.time() - last_emit >= interval and points_seen:
            valid = [(i, d) for i, d in enumerate(sector_min) if d]
            nearest = min(valid, key=lambda t: t[1]) if valid else None
            # ranges_m: one entry per 5° bin, metres, 0 where nothing returned.
            # The canvas walks this array by index, so it must always be the
            # full length rather than a sparse list.
            ranges_m = [
                round(min(mm / 1000.0, RANGE_MAX_M), 3) if mm else 0.0
                for mm in bin_min
            ]
            payload = {
                "ranges_m": ranges_m,
                "range_max_m": RANGE_MAX_M,
                "heading_deg": 0,
                "points": points_seen,
                "sectors_mm": sector_min,
                "sector_width_deg": 360 // SECTORS,
                "min_mm": nearest[1] if nearest else None,
                "min_bearing_deg": nearest[0] * (360 // SECTORS) if nearest else None,
                "baud": LIDAR_STATE.get("baud"),
            }
            bus.publish("telemetry/lidar", payload)
            LIDAR_STATE["scans"] += 1

            # Edge-triggered, not level-triggered. A rover parked against a wall
            # would otherwise raise an alert every scan — hundreds a minute —
            # which floods the alert path and buries any real event.
            now = time.time()
            close = bool(nearest and nearest[1] <= OBSTACLE_ALERT_MM)
            if close and (not OBSTACLE_STATE["active"]
                          or now - OBSTACLE_STATE["last_sent"] >= OBSTACLE_REPEAT_S):
                OBSTACLE_STATE["active"] = True
                OBSTACLE_STATE["last_sent"] = now
                bus.publish("events/obstacle", {
                    "alert": True,
                    "distance_mm": nearest[1],
                    "bearing_deg": nearest[0] * (360 // SECTORS),
                    "detail": f"obstacle {nearest[1]}mm at {nearest[0] * (360 // SECTORS)}deg",
                    "repeat": OBSTACLE_STATE["active"],
                }, qos=1)
            elif not close and OBSTACLE_STATE["active"]:
                OBSTACLE_STATE["active"] = False
                bus.publish("events/obstacle_cleared", {
                    "alert": False,
                    "detail": "path clear",
                }, qos=1)

            sector_min = [None] * SECTORS
            bin_min = [None] * BINS
            points_seen = 0
            last_emit = time.time()

    try:
        ser.close()
    except Exception:
        pass


# ------------------------------------------------------------------ main ---

def heartbeat(bus):
    while RUNNING.is_set():
        bus.publish("telemetry/pose", {
            "uptime_s": round(time.time() - STARTED, 1),
            "camera_ok": CAMERA_STATE["ok"], "lidar_ok": LIDAR_STATE["ok"],
            "frames": CAMERA_STATE["frames"], "scans": LIDAR_STATE["scans"],
            "streaming": STREAM_ENABLED.is_set(),
        })
        for _ in range(50):
            if not RUNNING.is_set():
                return
            time.sleep(0.1)


def main():
    def stop(*_):
        log("shutting down")
        RUNNING.clear()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    log(f"FPMS rover agent starting — thing={THING} broker={BROKER}:{PORT}")

    # Bring up the cloud uplink before the bus, so the very first publish (the
    # events/online in _on_connect) is already carried on both paths.
    global UPLINK
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import fpms_cloud_uplink
        UPLINK = fpms_cloud_uplink.from_config()
        if UPLINK is not None:
            UPLINK.start()
    except Exception as e:  # noqa: BLE001
        # Never fatal. Without it the agent behaves exactly as it did before.
        log(f"cloud uplink unavailable ({e}); continuing on MQTT only")
        UPLINK = None

    bus = Bus()
    bus.connect_forever()

    def supervised(fn, name):
        """Restart a sensor loop if it dies, and say so.

        A daemon thread that raises disappears without a word — that is exactly
        how the camera went quiet while the service still reported `active` and
        the dashboard showed an empty panel with nothing in the logs.
        """
        def wrapper():
            attempt = 0
            while RUNNING.is_set():
                try:
                    fn(bus)
                    if RUNNING.is_set():
                        log(f"{name} loop returned unexpectedly; restarting")
                except Exception as e:  # noqa: BLE001
                    import traceback
                    log(f"{name} loop crashed: {e}\n{traceback.format_exc()}")
                    bus.publish("events/fault", {
                        "component": name, "alert": True, "error": str(e)[:300],
                    }, qos=1)
                attempt += 1
                # Back off so a hard failure doesn't spin the CPU.
                for _ in range(int(min(5 * attempt, 30) * 10)):
                    if not RUNNING.is_set():
                        return
                    time.sleep(0.1)
        return threading.Thread(target=wrapper, daemon=True, name=name)

    threads = [
        supervised(camera_loop, "camera"),
        supervised(lidar_loop, "lidar"),
        supervised(heartbeat, "heartbeat"),
    ]
    for t in threads:
        t.start()

    while RUNNING.is_set():
        time.sleep(0.5)

    bus.publish("events/offline", {"status": "offline", "reason": "clean shutdown"}, qos=1)
    time.sleep(0.4)
    try:
        bus.client.loop_stop()
        bus.client.disconnect()
    except Exception:
        pass
    # After the offline event has been offered, so it gets a chance to go out or
    # be spooled for the next start rather than dying in the queue.
    if UPLINK is not None:
        try:
            UPLINK.stop()
        except Exception:
            pass
    log("stopped")


if __name__ == "__main__":
    main()

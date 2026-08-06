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
    """Return (fire_like, ratio, boxes) for flame-coloured regions."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Two hue bands: red wraps around 0/180 in OpenCV's scale.
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


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


# ------------------------------------------------------------------ MQTT ---

class Bus:
    """Thin MQTT wrapper. Publishing must never raise into a sensor loop.

    `role` names the client. Every Bus is a SEPARATE paho client with its own
    socket and its own network thread, and that separation is load-bearing
    rather than tidiness:

    paho serialises every publish from one client onto one network thread, in
    order. A 40 kB base64 camera frame queued just ahead of a 2.5 kB scan makes
    the scan wait for the frame to drain over WiFi. That is how the LiDAR feed
    went late-but-plausible under link saturation - the messages were not
    dropped, they arrived stale, which is the failure this stack exists to make
    impossible. Camera and LiDAR therefore never share a client.

    `max_queued` bounds the out-queue for QoS 0. paho's default is UNBOUNDED,
    so a saturated link grows the queue without limit and every message in it
    ages. For the camera we want the oldest frames DROPPED, not delivered late:
    a dropped frame is invisible, a late one is a lie about the present.
    """

    def __init__(self, role="agent", subscribe=True, max_queued=0, will=True):
        self.role = role
        self.subscribe = subscribe
        self.client = Client(client_id=f"{THING}-{role}",
                             callback_api_version=CallbackAPIVersion.VERSION2)
        if USER:
            self.client.username_pw_set(USER, PASS or None)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        if subscribe:
            self.client.on_message = self._on_message
        if max_queued:
            # Drop the OLDEST queued message when full rather than blocking or
            # growing. Only meaningful for QoS 0, which is all the camera uses.
            try:
                self.client.max_queued_messages_set(max_queued)
            except Exception:
                pass
        # Last will: if this agent dies, the dashboard finds out from the broker
        # rather than waiting for telemetry to go stale.
        if will:
            self.client.will_set(f"fpms/{THING}/events/offline",
                                 json.dumps({"thing": THING, "status": "offline",
                                             "reason": "unexpected disconnect"}),
                                 qos=1, retain=False)
        self.connected = False
        self.drops = 0

    def connect_forever(self):
        # connect_async + loop_start, not connect + loop_start. The blocking
        # form cannot come back on its own if the broker is not up yet at boot;
        # the async form lets paho's own reconnect state machine own it, which
        # is what makes a cold boot with a slow-starting mosquitto self-heal
        # instead of sitting in this retry loop.
        self.client.reconnect_delay_set(min_delay=1, max_delay=15)
        while RUNNING.is_set():
            try:
                self.client.connect_async(BROKER, PORT, keepalive=30)
                self.client.loop_start()
                return
            except Exception as e:
                log(f"[{self.role}] MQTT connect failed ({e}); retrying in 5s")
                time.sleep(5)

    def _on_connect(self, client, _u, _f, reason_code, _p=None):
        self.connected = True
        log(f"[{self.role}] MQTT connected to {BROKER}:{PORT} ({reason_code})")
        if not self.subscribe:
            return
        client.subscribe(f"fpms/{THING}/commands/#", qos=1)
        self.publish("events/online", {
            "thing": THING, "status": "online",
            "camera": CAMERA_DEV, "lidar": LIDAR_PORT,
            "capabilities": ["camera", "lidar", "yolo"],
        }, qos=1)

    def _on_disconnect(self, _c, _u, *a):
        # Without this, `connected` latched True forever after the first
        # connect. Every publish then went into paho's queue believing it had a
        # link, and the "is the feed alive" question got a confident wrong
        # answer from a process that had been talking to nobody for minutes.
        self.connected = False
        log(f"[{self.role}] MQTT disconnected")

    def _on_message(self, _c, _u, msg):
        action = msg.topic.rsplit("/", 1)[-1]
        try:
            payload = json.loads(msg.payload.decode() or "{}")
        except Exception:
            payload = {}
        handle_command(self, action, payload)

    def publish(self, suffix, payload, qos=0):
        if not self.connected:
            return
        try:
            payload.setdefault("ts", time.time())
            payload.setdefault("thing", THING)
            info = self.client.publish(f"fpms/{THING}/{suffix}",
                                       json.dumps(payload), qos=qos)
            # rc 1 is MQTT_ERR_NOMEM: the bounded queue rejected it. Count it so
            # "the camera is dropping frames" is a number in health rather than
            # something an operator has to infer from a stuttering picture.
            if getattr(info, "rc", 0) != 0:
                self.drops += 1
        except Exception as e:
            log(f"[{self.role}] publish failed on {suffix}: {e}")


class SplitBus:
    """Route the bulky frames one way and everything else the other.

    The camera thread publishes two very different things on one call surface:
    ~40 kB base64 JPEGs at QoS 0, which we are happy to DROP under load, and
    fire/wildlife/fault events at QoS 1, which we are not. Putting both on the
    bounded client would let a saturated link discard a fire alert; putting both
    on the unbounded one brings back the head-of-line blocking that starves the
    LiDAR. So the frames go to the bounded client and the events go to the main
    one, and the camera loop needs no knowledge of any of it.
    """

    def __init__(self, heavy, light, heavy_suffixes=("telemetry/camera",)):
        self._heavy = heavy
        self._light = light
        self._heavy_suffixes = tuple(heavy_suffixes)

    def publish(self, suffix, payload, qos=0):
        target = self._heavy if suffix in self._heavy_suffixes else self._light
        return target.publish(suffix, payload, qos=qos)

    @property
    def drops(self):
        return self._heavy.drops

    def __getattr__(self, item):
        return getattr(self._light, item)


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
LAST_DETECTIONS: list = []


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
        import fpms_yolo_npu as ynpu          # their proven decode path
        from rknnlite.api import RKNNLite

        decode = ynpu.decode
        rknn = RKNNLite()
        if rknn.load_rknn(ynpu.MODEL) != 0:
            raise RuntimeError(f"load_rknn failed for {ynpu.MODEL}")
        if rknn.init_runtime() != 0:
            raise RuntimeError("init_runtime failed")
        log(f"NPU ready with {ynpu.MODEL}")
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

        # Fire screen runs on every frame — it is far cheaper than inference and
        # a flame can appear between two YOLO frames.
        fire_like, fire_ratio, fire_boxes = detect_fire(frame)
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


# ---------------------------------------------------------------------------
# LiDAR: freshness thresholds and stable port resolution.
#
# A scan that is merely late is not the same thing as a scanner that has
# stopped, and the dashboard has to tell them apart, so there are two verdicts
# rather than one boolean.
# ---------------------------------------------------------------------------
LIDAR_STALE_S = float(CFG.get("FPMS_LIDAR_STALE_S", "0.5"))
LIDAR_DEAD_S = float(CFG.get("FPMS_LIDAR_DEAD_S", "2.0"))
# No bytes at all for this long means the descriptor is wedged, not just quiet.
LIDAR_REOPEN_S = float(CFG.get("FPMS_LIDAR_REOPEN_S", "3.0"))
# Bytes ARE arriving but none of them parse as LD frames for this long. That is
# a different fault from silence - a wrong baud, a half-open port that survived
# a re-enumeration, or another process talking on the same tty - and the
# byte-silence timer above can never fire for it, because bytes keep coming.
# Without this the loop reports "dead" forever while cheerfully reading noise.
LIDAR_STARVE_S = float(CFG.get("FPMS_LIDAR_STARVE_S", "4.0"))

# Both CP2102 adapters on this rover report the identical ID_SERIAL "0001", so
# udev can only ever create ONE /dev/serial/by-id link for the pair: by-id is
# unusable here and the USB topology path is the only stable discriminator.
#
# Port 1.3 is the ESP32-S3 micro-ROS board and MUST NEVER be opened by this
# process. Opening that tty raises the CP2102 handshake lines wired to the MCU's
# EN/IO0 pins and resets the board out from under micro-ros-agent, which costs a
# session and takes the rover's only pose source down with it. A bare
# /dev/ttyUSBn in config is exactly how that mis-binding happens, because
# enumeration order swaps across boots.
LIDAR_BY_PATH = "/dev/serial/by-path/platform-fc880000.usb-usb-0:1.2:1.0-port0"
UROS_BY_PATH = "/dev/serial/by-path/platform-fc880000.usb-usb-0:1.3:1.0-port0"

LIDAR_PORT_CFG = LIDAR_PORT


def _resolve_lidar_port():
    """Pick a stable device path for the scanner, never the micro-ROS board."""
    try:
        uros_real = os.path.realpath(UROS_BY_PATH)
    except Exception:
        uros_real = None

    candidates = []
    if os.path.exists(LIDAR_BY_PATH):
        candidates.append(LIDAR_BY_PATH)
    if LIDAR_PORT_CFG and LIDAR_PORT_CFG not in candidates:
        candidates.append(LIDAR_PORT_CFG)

    for dev in candidates:
        try:
            if uros_real and os.path.realpath(dev) == uros_real:
                log(f"lidar: refusing {dev} - it resolves to the micro-ROS board")
                continue
        except Exception:
            pass
        if os.path.exists(dev):
            if dev != LIDAR_PORT_CFG:
                log(f"lidar: using stable path {dev} instead of configured "
                    f"{LIDAR_PORT_CFG}")
            return dev
    return LIDAR_PORT_CFG


def _lidar_open():
    """Open the scanner and confirm it is really emitting LD frames.

    A short read timeout matters as much as the baud: with the old 1 s timeout
    the loop could sit blocked in read() long after the scanner died, delaying
    the dead verdict by up to a second and adding that much latency to every
    emit when data was flowing normally.
    """
    global LIDAR_PORT
    LIDAR_PORT = _resolve_lidar_port()
    last_err = None
    for baud in (LIDAR_BAUD, 115200, 460800):
        ser = None
        try:
            ser = serial.Serial(LIDAR_PORT, baud, timeout=0.05)
            time.sleep(0.3)
            probe = ser.read(4096)
            if probe.count(bytes([LD_HEADER, LD_VERLEN])) >= 3:
                if LIDAR_STATE.get("baud") != baud:
                    log(f"lidar sync at {baud} baud on {LIDAR_PORT}")
                LIDAR_STATE["baud"] = baud
                return ser
            ser.close()
        except Exception as e:
            last_err = e
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
    if last_err is not None:
        log(f"lidar open failed on {LIDAR_PORT}: {last_err}")
    return None


def lidar_loop(bus):
    """Read the scanner and publish a 12-sector summary.

    Freshness is a first-class part of the payload. The old loop only published
    when it had points (`if ... and points_seen`), so the instant the scanner
    stopped the feed simply went silent and the dashboard kept drawing its last
    frame forever - live-looking data with nothing behind it, and no way for an
    operator to tell the difference. This version publishes on the same cadence
    no matter what and states, every time, how old the data actually is.

    THE THREE MECHANISMS THAT MAKE "NEVER STALE" TRUE, not merely intended:

    1. The emit is on a TIMER, not on data. There is no `and points_seen` and
       no early `continue` that can skip it. The only way this loop stops
       publishing is if the process dies, which systemd and the LWT both make
       visible.
    2. `hz` is recomputed from scratch on every emit and FORCED to 0.0 unless
       the newest point is fresh right now. It is never an EMA and never a ring
       buffer average, because both keep returning a plausible number long
       after the source dies - this project has been fooled by exactly that.
    3. The port is reopened on BOTH failure shapes: byte silence (a wedged
       descriptor that read()s empty forever without ever raising) and frame
       starvation (bytes arriving that are not LD frames). Neither raises an
       exception, so neither can be caught by the supervisor above.
    """
    ser = _lidar_open()
    if ser is None:
        log("lidar: no recognisable LD frames at any baud")
        bus.publish("events/fault",
                    {"component": "lidar", "error": "no LD frames; check model/baud"},
                    qos=1)
        # Deliberately NOT `return`. Returning hands the thread to supervised()'s
        # backoff, and for those seconds the feed reports nothing at all - the
        # exact silence this rewrite exists to remove. Falling through keeps the
        # health verdict publishing on cadence while the port is retried.

    # Two resolutions from the same scan:
    #   BINS (360 x 1 deg) -> ranges_m, what the dashboard's radar canvas draws
    #   SECTORS (12 x 30 deg) -> coarse obstacle logic and alerting
    # The UI contract is ranges_m + range_max_m; sending only sectors_mm left
    # ranges_m undefined and the canvas threw on every scan.
    SECTORS = 12
    # One bin per degree. The dashboard canvas maps array index directly to an
    # angle (`(i - 90) * PI / 180`), so any other length silently draws a wedge
    # instead of a full scan - it looks like a broken sensor, not a unit bug.
    BINS = 360
    RANGE_MAX_M = 6.0
    bin_min = [None] * BINS
    sector_min = [None] * SECTORS
    points_seen = 0
    last_emit = time.time()
    interval = 1.0 / max(LIDAR_HZ, 0.1)
    buf = bytearray()

    # The ONLY source of truth for freshness: when the newest point was parsed.
    # Everything downstream is derived from this on each emit, never cached.
    last_point_ts = 0.0
    last_byte_ts = time.time()
    last_frame_ts = time.time()
    emit_stamps = []
    last_health = None
    reopen_backoff = 0.0
    seq = 0
    reopens = 0

    while RUNNING.is_set():
        now = time.time()

        if ser is None:
            # Retry the port on the reopen cadence, but keep falling through to
            # the emit block below so staleness keeps being reported meanwhile.
            if now >= reopen_backoff:
                ser = _lidar_open()
                if ser is not None:
                    log(f"lidar: port reopened on {LIDAR_PORT}")
                    reopens += 1
                    buf.clear()
                    last_byte_ts = time.time()
                    last_frame_ts = time.time()
                else:
                    reopen_backoff = now + LIDAR_REOPEN_S
        else:
            try:
                chunk = ser.read(512)
            except Exception as e:
                log(f"lidar read error: {e}; dropping port for reopen")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                reopen_backoff = now + LIDAR_REOPEN_S
                chunk = b""
            if chunk:
                buf.extend(chunk)
                last_byte_ts = now
            elif now - last_byte_ts >= LIDAR_REOPEN_S:
                # Bytes stopped entirely. A silent descriptor survives a USB
                # re-enumeration indefinitely - read() just returns empty for
                # the rest of the run - so drop it and re-probe rather than
                # politely reading a dead fd forever.
                log(f"lidar: no bytes for {now - last_byte_ts:.1f}s; reopening")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                buf.clear()
                reopen_backoff = now + LIDAR_REOPEN_S

            # Frame starvation. Bytes are flowing but nothing parses. The
            # byte-silence timer above can never catch this, and without it the
            # loop reports "dead" forever while reading noise at full rate.
            if ser is not None and now - last_frame_ts >= LIDAR_STARVE_S:
                log(f"lidar: bytes but no LD frames for {now - last_frame_ts:.1f}s; "
                    "reopening (wrong baud, or another reader on this tty?)")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                buf.clear()
                reopen_backoff = now + LIDAR_REOPEN_S

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
            last_frame_ts = time.time()

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
                last_point_ts = time.time()

        now = time.time()
        if now - last_emit < interval:
            continue

        # Recomputed from the newest arrival on EVERY emit and never cached.
        # A rate averaged over a ring buffer keeps returning a plausible number
        # long after the source dies; that false-alive reading is the whole
        # reason this block exists, so hz is forced to 0.0 unless the data is
        # genuinely fresh right now.
        age = (now - last_point_ts) if last_point_ts else None
        if age is None or age >= LIDAR_DEAD_S:
            health = "dead"
        elif age >= LIDAR_STALE_S:
            health = "stale"
        else:
            health = "ok"

        if health == "ok":
            emit_stamps.append(now)
            del emit_stamps[:-32]
        else:
            emit_stamps.clear()
        hz = 0.0
        if health == "ok" and len(emit_stamps) >= 2:
            span_s = emit_stamps[-1] - emit_stamps[0]
            if span_s > 0:
                hz = round((len(emit_stamps) - 1) / span_s, 2)

        if health == "dead":
            # Zeros, not the last good scan. Re-sending a stale frame under a
            # fresh ts is exactly what made a frozen feed look live; zeros plus
            # health="dead" render honestly as "no return".
            #
            # NOTE FOR CONSUMERS: zeros mean NO EVIDENCE, not "clear". Any
            # obstacle guard reading this payload must gate on `stale`/`health`
            # and refuse to treat a dead scan as a clear path. fpms_missions.py
            # and fpms_lidar_ros.py both do; see STACK.md "the stale contract".
            ranges_m = [0.0] * BINS
            sectors_out = [None] * SECTORS
            nearest = None
        else:
            valid = [(i, d) for i, d in enumerate(sector_min) if d]
            nearest = min(valid, key=lambda t: t[1]) if valid else None
            # ranges_m: one entry per 1 deg bin, metres, 0 where nothing
            # returned. The canvas walks this array by index, so it must always
            # be the full length rather than a sparse list. Two decimals is
            # centimetre resolution - below the scanner's own accuracy - and
            # trims roughly a sixth off the largest payload on the link.
            ranges_m = [
                round(min(mm / 1000.0, RANGE_MAX_M), 2) if mm else 0.0
                for mm in bin_min
            ]
            sectors_out = sector_min

        seq += 1
        payload = {
            "ranges_m": ranges_m,
            "range_max_m": RANGE_MAX_M,
            "heading_deg": 0,
            "points": points_seen,
            "sectors_mm": sectors_out,
            "sector_width_deg": 360 // SECTORS,
            "min_mm": nearest[1] if nearest else None,
            "min_bearing_deg": nearest[0] * (360 // SECTORS) if nearest else None,
            "baud": LIDAR_STATE.get("baud"),
            # --- freshness contract (all keys ADDITIVE; nothing above changed) ---
            # scan_age_s is the age of the DATA, not of the message. ts (added
            # by Bus.publish) always looks fresh because we always publish, so
            # age is the field a consumer must actually gate on.
            "scan_age_s": round(age, 3) if age is not None else None,
            "last_scan_ts": round(last_point_ts, 3) if last_point_ts else None,
            "health": health,
            "stale": health != "ok",
            "hz": hz,
            "port": LIDAR_PORT,
            # seq increments on every emit including dead ones. A consumer that
            # sees seq advancing while stale is true knows this loop is alive
            # and the SCANNER is the fault; a consumer that sees seq frozen
            # knows the PROCESS is the fault. Those need different fixes and
            # were previously indistinguishable.
            "seq": seq,
            "port_reopens": reopens,
        }
        bus.publish("telemetry/lidar", payload)

        LIDAR_STATE["ok"] = (health == "ok")
        LIDAR_STATE["health"] = health
        LIDAR_STATE["age_s"] = payload["scan_age_s"]
        LIDAR_STATE["hz"] = hz
        LIDAR_STATE["seq"] = seq
        if health == "ok":
            LIDAR_STATE["scans"] += 1

        if health != last_health:
            log(f"lidar health {last_health} -> {health} "
                f"(age={payload['scan_age_s']}s hz={hz})")
            if health == "dead":
                bus.publish("events/fault", {
                    "component": "lidar", "alert": True,
                    "error": "lidar feed dead - no points",
                    "scan_age_s": payload["scan_age_s"],
                }, qos=1)
            elif last_health == "dead":
                bus.publish("events/lidar_recovered", {
                    "component": "lidar", "alert": False,
                    "detail": "lidar feed live again", "hz": hz,
                }, qos=1)
            last_health = health

        # Obstacle logic runs on fresh data only. Alerting - or worse, clearing
        # an existing alert - from a dead feed would report a path as clear on
        # the strength of no evidence at all.
        if health == "ok":
            # Edge-triggered, not level-triggered. A rover parked against a wall
            # would otherwise raise an alert every scan - hundreds a minute -
            # which floods the alert path and buries any real event.
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
        last_emit = now

    try:
        if ser is not None:
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
            # Additive. lidar_ok alone was a latched True that never came back
            # down, so it kept reporting a healthy scanner long after the feed
            # died. These three are recomputed by the LiDAR loop on every emit.
            "lidar_health": LIDAR_STATE.get("health"),
            "lidar_age_s": LIDAR_STATE.get("age_s"),
            "lidar_hz": LIDAR_STATE.get("hz"),
            "lidar_seq": LIDAR_STATE.get("seq"),
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
    # THREE clients, not one, and the split is a safety property.
    #
    #   bus       control + events + heartbeat. Owns the LWT and the command
    #             subscription, so command handling never sits behind a queue
    #             of sensor data.
    #   lidar_bus scans ONLY. Unbounded queue on purpose: a scan is small and
    #             we would rather deliver every one of them. Nothing large ever
    #             shares this socket, so nothing can delay a scan.
    #   cam_bus   camera ONLY, with a SHORT bounded queue. When the link
    #             saturates the oldest frames are dropped instead of piling up.
    #             Previously all three shared one client and one network thread,
    #             so a 40 kB frame ahead of a scan delayed the scan by however
    #             long the frame took to drain - the LiDAR feed went stale
    #             without a single message being lost, which is precisely the
    #             failure mode that is hardest to see from the dashboard.
    bus = Bus(role="agent", subscribe=True)
    lidar_bus = Bus(role="lidar", subscribe=False, will=False)
    cam_bus = Bus(role="cam", subscribe=False, will=False, max_queued=2)
    for b in (bus, lidar_bus, cam_bus):
        b.connect_forever()

    def supervised(fn, name, feed=None):
        """Restart a sensor loop if it dies, and say so.

        A daemon thread that raises disappears without a word — that is exactly
        how the camera went quiet while the service still reported `active` and
        the dashboard showed an empty panel with nothing in the logs.
        """
        target_bus = feed if feed is not None else bus

        def wrapper():
            attempt = 0
            while RUNNING.is_set():
                try:
                    fn(target_bus)
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
        supervised(camera_loop, "camera", feed=SplitBus(heavy=cam_bus, light=bus)),
        supervised(lidar_loop, "lidar", feed=lidar_bus),
        supervised(heartbeat, "heartbeat"),
    ]
    for t in threads:
        t.start()

    while RUNNING.is_set():
        time.sleep(0.5)

    bus.publish("events/offline", {"status": "offline", "reason": "clean shutdown"}, qos=1)
    time.sleep(0.4)
    for b in (bus, lidar_bus, cam_bus):
        try:
            b.client.loop_stop()
            b.client.disconnect()
        except Exception:
            pass
    log("stopped")


if __name__ == "__main__":
    main()

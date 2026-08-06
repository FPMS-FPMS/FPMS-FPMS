#!/usr/bin/env python3


# ============================================================
# B6 SPEED / DISTANCE TUNING - 2026-05-06
# ============================================================
# Normal mission movement should be faster.
B6_FORWARD_POWER = 34

# Return should not be too slow when far from HOME.
B6_HOME_FAST_BACKWARD_POWER = 28

# But close to HOME it MUST crawl slowly to avoid crashing.
B6_HOME_CRAWL_BACKWARD_POWER = 16

# Start crawling this far from HOME.
# Increase this if it still hits HOME.
B6_HOME_CRAWL_DIST_CM = 45

# Stop at HOME earlier/safer.
# Increase this if M1 still crashes into HOME.
B6_HOME_STOP_DIST_CM = 24

# Target stop: slightly closer for M2.
# Decrease this if M2 still stops too far.
# Increase this if it bumps the marker/tree.
B6_TARGET_STOP_DIST_CM = 14
# ============================================================

"""
FPMS B8.4 LIDAR GAP
One webserver: D500 LiDAR + ESP-NOW receiver + marker tracking + route planner + run-to-M1-and-home mission.

Map coverage: 200 cm wide x 250 cm deep.
X: -1000..+1000 mm
Y: -500..+2000 mm

Tested assumptions from FPMS build:
- D500 LiDAR on CP2102 at 230400 baud
- ESP32 receiver on by-path platform-xhci-hcd.11.auto... at 115200 baud
- Yahboom ROS board on by-path platform-fc840000... using Rosmaster_Lib
- set_motor negative values drive forward on this chassis
- LiDAR is 200 mm behind robot front tip
- stop raw distance 250 mm = about 5 cm from front tip
"""

import glob
import heapq
import math
import os
import serial
import struct
import threading
import time
from flask import Flask, jsonify, request, render_template_string

# =====================
# Stable ports
# =====================
D500_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
ESP_PORT = "/dev/serial/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-port0"
YAHBOOM_PORT = "/dev/serial/by-path/platform-fc840000.usb-usb-0:1:1.0-port0"

D500_BAUD = 230400
ESP_BAUD = 115200
WEB_PORT = 8085

# =====================
# World/map geometry
# =====================
WORLD_X_MIN = -1000
WORLD_X_MAX = 1000
WORLD_Y_MIN = -1500
WORLD_Y_MAX = 1500
WORLD_W = WORLD_X_MAX - WORLD_X_MIN
WORLD_H = WORLD_Y_MAX - WORLD_Y_MIN

ROBOT_W = 230
ROBOT_RADIUS = ROBOT_W / 2
LIDAR_TO_FRONT = 200
TARGET_STOP_RAW = 300
FRONT_STOP_RAW = 300
MAX_LIDAR_MM = 2600

# =====================
# Anti-lag tuning
# =====================
UPDATE_S = 0.45
UI_POINT_LIMIT = 100
FRONT_DEG = 25

# =====================
# Marker lock tuning
# =====================
MARKER_SNAP_MM = 190
MARKER_RELOCK_MM = 420
MARKER_ANGLE_RELOCK_DEG = 18
MARKER_DIST_RELOCK_MM = 520
MARKER_LOST_LIMIT = 12
CLUSTER_ANGLE_DEG = 8
CLUSTER_DIST_MM = 190
CLUSTER_RADIUS_MM = 220

# =====================
# Planner tuning
# =====================
GRID_RES = 50
ROUTE_CLEARANCE = 70
ROUTE_RADIUS = ROBOT_RADIUS + ROUTE_CLEARANCE
TARGET_EXCLUDE_RADIUS = 260
START_CLEAR_RADIUS = 200

# =====================
# Motor tuning
# =====================
FORWARD_POWER = -24
BACKWARD_POWER = B6_HOME_FAST_BACKWARD_POWER
TURN_POWER = 34
FINE_FORWARD_POWER = -18
FINE_BACKWARD_POWER = 18
TURN_SIGN = 1       # if it turns away from marker, change to -1
FACE_TOL_DEG = 8
DRIVE_ANGLE_TOL_DEG = 17
MISSION_TIMEOUT_S = 55

# B4 safety/odometry tuning
SLOW_APPROACH_RAW = 390
CRITICAL_FRONT_RAW = 285
REPLAN_EVERY_S = 0.75
BACKWARD_HOME_ANGLE_DEG = 105
AUTO_COOLDOWN_S = 18

# Pump disabled until MOSFET/pump output is separately verified.
SPRAY_ENABLED = False
SPRAY_SECONDS = 5

# Pump/MOSFET on Yahboom PWM Servo 1 signal pin.
# If pump is reversed, swap ON/OFF angles.
SPRAY_SERVO_ID = 1
SPRAY_ON_ANGLE = 180
SPRAY_OFF_ANGLE = 0

# Always-face target mode.
# When ROBOT ON and no mission is running:
# - if ALERT2 and M2 locked, face M2
# - otherwise face M1 when M1 is locked
FACE_HOLD_ENABLED = True
FACE_HOLD_TOL_DEG = 6
FACE_HOLD_INTERVAL_S = 0.12
FACE_HOLD_MIN_DIST = TARGET_STOP_RAW + 80


CRC8 = [
0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
]


def crc8(data):
    c = 0
    for b in data:
        c = CRC8[(c ^ b) & 0xFF]
    return c


lock = threading.Lock()
work_scan = [None] * 360
disp_scan = [None] * 360
last_sa = -1.0
frame_count = 0
last_hz_t = time.time()
raw_bytes = 0
last_bps_t = time.time()

S = {
    "status": "BOOT",
    "raw_bps": 0,
    "headers": 0,
    "ok": 0,
    "bad": 0,
    "scan_hz": 0,
    "points": 0,
    "front_raw": 0,
    "front_gap": 0,
    "ui_scan": [],
    "markers": {"m1": None, "m2": None, "home": None},
    "route_path": [],
    "blocked_points": [],
    "route_msg": "No route yet.",
    "events": []
}

ESP = {
    "n1_online": False,
    "n2_online": False,
    "alert1": False,
    "alert2": False,
    "last_line": "",
    "last_alert": "",
    "last_update": 0,
    "age_s": 999
}

MISSION = {
    "state": "IDLE",
    "target": "NONE",
    "message": "Ready.",
    "last_error": "",
    "moved_mm": 0,
    "started_at": 0,
    "finished_at": 0
}

robot_enabled = False
auto_enabled = False
mission_active = False
stop_requested = False
auto_cooldown_until = 0


def event(msg, typ="i"):
    print(f"[{typ}] {msg}", flush=True)
    with lock:
        S["events"].append({"t": time.strftime("%H:%M:%S"), "msg": msg, "typ": typ})
        S["events"] = S["events"][-12:]
        MISSION["message"] = msg


def deg_xy(deg, dist):
    a = math.radians(deg)
    return math.sin(a) * dist, math.cos(a) * dist


def xy_deg_dist(x, y):
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0, math.hypot(x, y)


def signed_angle(a):
    return (float(a) + 180.0) % 360.0 - 180.0


def angle_gap(a, b):
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def in_world(x, y):
    return WORLD_X_MIN <= x <= WORLD_X_MAX and WORLD_Y_MIN <= y <= WORLD_Y_MAX


# =====================
# D500 LiDAR
# =====================
def parse_d500(pkt):
    global last_sa, frame_count, last_hz_t
    sa_raw = struct.unpack_from("<H", pkt, 4)[0]
    ea_raw = struct.unpack_from("<H", pkt, 42)[0]
    diff = ea_raw - sa_raw if sa_raw <= ea_raw else 36000 + ea_raw - sa_raw
    step = diff / 11.0
    sa = sa_raw / 100.0

    with lock:
        if last_sa > 270 and sa < 90:
            disp_scan[:] = work_scan[:]
            for i in range(360):
                work_scan[i] = None
            frame_count += 1
            now = time.time()
            if now - last_hz_t >= 1:
                S["scan_hz"] = round(frame_count / (now - last_hz_t), 1)
                frame_count = 0
                last_hz_t = now

        last_sa = sa

        for i in range(12):
            off = 6 + i * 3
            d = struct.unpack_from("<H", pkt, off)[0]
            inten = pkt[off + 2]
            if 0 < d <= MAX_LIDAR_MM:
                deg = int(((sa_raw + step * i) % 36000) / 100.0) % 360
                work_scan[deg] = [int(d), int(inten)]


def lidar_loop():
    global raw_bytes, last_bps_t
    while True:
        try:
            ser = serial.Serial(D500_PORT, D500_BAUD, timeout=0.25)
            event("D500 opened", "k")
            with lock:
                S["status"] = "D500 OPEN"
            while True:
                b = ser.read(1)
                if b:
                    raw_bytes += 1
                now = time.time()
                if now - last_bps_t >= 1:
                    with lock:
                        S["raw_bps"] = raw_bytes
                    raw_bytes = 0
                    last_bps_t = now
                if not b or b[0] != 0x54:
                    continue
                b2 = ser.read(1)
                if b2:
                    raw_bytes += 1
                if not b2 or b2[0] != 0x2C:
                    continue
                rest = ser.read(45)
                raw_bytes += len(rest)
                if len(rest) != 45:
                    continue
                pkt = bytes([0x54, 0x2C]) + rest
                with lock:
                    S["headers"] += 1
                if crc8(pkt[:46]) != pkt[46]:
                    with lock:
                        S["bad"] += 1
                        S["status"] = "BAD CRC"
                    continue
                parse_d500(pkt)
                with lock:
                    S["ok"] += 1
                    S["status"] = "D500 LIVE"
        except Exception as e:
            event("LiDAR error: " + str(e), "e")
            with lock:
                S["status"] = "LIDAR ERROR"
            time.sleep(2)


def scan_points(all_points=True):
    with lock:
        sc = list(disp_scan)
    pts = []
    for deg, v in enumerate(sc):
        if not v:
            continue
        d, inten = v
        x, y = deg_xy(deg, d)
        if in_world(x, y):
            pts.append({"deg": deg, "d": d, "i": inten, "x": round(x, 1), "y": round(y, 1)})
    if all_points or len(pts) <= UI_POINT_LIMIT:
        return pts
    pts.sort(key=lambda p: p["deg"])
    step = len(pts) / UI_POINT_LIMIT
    out, idx = [], 0.0
    while int(idx) < len(pts) and len(out) < UI_POINT_LIMIT:
        out.append(pts[int(idx)])
        idx += step
    return out


# =====================
# Marker tracking
# =====================
def nearest_point(x, y, pts, max_mm):
    best, bd = None, max_mm
    for p in pts:
        e = math.hypot(p["x"] - x, p["y"] - y)
        if e < bd:
            best, bd = p, e
    return best


def find_relock_point(marker, pts):
    if not marker:
        return None
    best = nearest_point(marker["x"], marker["y"], pts, MARKER_RELOCK_MM)
    if best:
        return best
    best_score = 1e9
    best_p = None
    for p in pts:
        da = angle_gap(p["deg"], marker["angle"])
        dd = abs(p["d"] - marker["dist"])
        if da <= MARKER_ANGLE_RELOCK_DEG and dd <= MARKER_DIST_RELOCK_MM:
            score = da * 18 + dd
            if score < best_score:
                best_score = score
                best_p = p
    return best_p


def make_cluster(seed, pts):
    members = []
    for p in pts:
        if angle_gap(p["deg"], seed["deg"]) <= CLUSTER_ANGLE_DEG and abs(p["d"] - seed["d"]) <= CLUSTER_DIST_MM:
            if math.hypot(p["x"] - seed["x"], p["y"] - seed["y"]) <= CLUSTER_RADIUS_MM:
                members.append(p)
    if not members:
        members = [seed]
    cx = sum(p["x"] for p in members) / len(members)
    cy = sum(p["y"] for p in members) / len(members)
    a, d = xy_deg_dist(cx, cy)
    return {
        "locked": True,
        "lost_count": 0,
        "x": round(cx, 1),
        "y": round(cy, 1),
        "angle": round(a, 1),
        "dist": round(d, 1),
        "front_gap": round(d - LIDAR_TO_FRONT, 1),
        "points": len(members),
        "last_seen": time.time()
    }


# =====================
# ESP-NOW receiver over serial
# =====================
def esp_line(line):
    now = time.time()
    with lock:
        ESP["last_line"] = line
        ESP["last_update"] = now
        if line.startswith("ALERT1"):
            ESP["alert1"] = True; ESP["n1_online"] = True; ESP["last_alert"] = "ALERT1"
        elif line.startswith("CLEAR1"):
            ESP["alert1"] = False; ESP["n1_online"] = True; ESP["last_alert"] = "CLEAR1"
        elif line.startswith("ALERT2"):
            ESP["alert2"] = True; ESP["n2_online"] = True; ESP["last_alert"] = "ALERT2"
        elif line.startswith("CLEAR2"):
            ESP["alert2"] = False; ESP["n2_online"] = True; ESP["last_alert"] = "CLEAR2"
        elif line.startswith("STATUS:"):
            for part in line.split(":"):
                if part == "N1=ONLINE": ESP["n1_online"] = True
                elif part == "N1=OFFLINE": ESP["n1_online"] = False
                elif part == "N2=ONLINE": ESP["n2_online"] = True
                elif part == "N2=OFFLINE": ESP["n2_online"] = False
                elif part == "A1=1": ESP["alert1"] = True
                elif part == "A1=0": ESP["alert1"] = False
                elif part == "A2=1": ESP["alert2"] = True
                elif part == "A2=0": ESP["alert2"] = False


def esp_loop():
    while True:
        try:
            ser = serial.Serial(ESP_PORT, ESP_BAUD, timeout=1)
            time.sleep(2)
            event("ESP receiver opened", "k")
            while True:
                line = ser.readline().decode("utf-8", "ignore").strip()
                if line:
                    esp_line(line)
        except Exception as e:
            with lock:
                ESP["last_line"] = "ESP_ERROR: " + str(e)
                ESP["last_update"] = time.time()
                ESP["n1_online"] = False
                ESP["n2_online"] = False
            time.sleep(2)


# =====================
# Planner/costmap
# =====================
def xy_cell(x, y):
    c = int(round((x - WORLD_X_MIN) / GRID_RES))
    r = int(round((y - WORLD_Y_MIN) / GRID_RES))
    return c, r


def cell_xy(c, r):
    return c * GRID_RES + WORLD_X_MIN, r * GRID_RES + WORLD_Y_MIN


def grid_size():
    return int(WORLD_W / GRID_RES) + 1, int(WORLD_H / GRID_RES) + 1


def point_segment_dist(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    denom = vx * vx + vy * vy
    if denom < 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0, min(1, (wx * vx + wy * vy) / denom))
    cx, cy = ax + t * vx, ay + t * vy
    return math.hypot(px - cx, py - cy)


def planning_obstacles(pts, target_marker=None):
    obs = []
    for p in pts:
        x, y = p["x"], p["y"]
        if math.hypot(x, y) < START_CLEAR_RADIUS:
            continue
        if -ROBOT_W/2 - 60 <= x <= ROBOT_W/2 + 60 and -150 <= y <= LIDAR_TO_FRONT + 120:
            continue
        if target_marker and math.hypot(x - target_marker["x"], y - target_marker["y"]) < TARGET_EXCLUDE_RADIUS:
            continue
        obs.append(p)
    return obs


def build_costmap(obs):
    cols, rows = grid_size()
    occ = [[False] * cols for _ in range(rows)]
    rad = int(math.ceil(ROUTE_RADIUS / GRID_RES))
    for p in obs:
        c0, r0 = xy_cell(p["x"], p["y"])
        for dr in range(-rad, rad + 1):
            rr = r0 + dr
            if rr < 0 or rr >= rows:
                continue
            for dc in range(-rad, rad + 1):
                cc = c0 + dc
                if cc < 0 or cc >= cols:
                    continue
                if math.hypot(dc * GRID_RES, dr * GRID_RES) <= ROUTE_RADIUS:
                    occ[rr][cc] = True
    sc, sr = xy_cell(0, 0)
    clear = int(math.ceil(START_CLEAR_RADIUS / GRID_RES))
    for dr in range(-clear, clear + 1):
        for dc in range(-clear, clear + 1):
            rr, cc = sr + dr, sc + dc
            if 0 <= rr < rows and 0 <= cc < cols and math.hypot(dc * GRID_RES, dr * GRID_RES) <= START_CLEAR_RADIUS:
                occ[rr][cc] = False
    return occ


def nearest_free(occ, cell, max_rad=8):
    cols, rows = grid_size()
    c0, r0 = cell
    if 0 <= c0 < cols and 0 <= r0 < rows and not occ[r0][c0]:
        return cell
    for rad in range(1, max_rad + 1):
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                if abs(dc) != rad and abs(dr) != rad:
                    continue
                c, r = c0 + dc, r0 + dr
                if 0 <= c < cols and 0 <= r < rows and not occ[r][c]:
                    return c, r
    return None


def astar(occ, start_xy, goal_xy):
    cols, rows = grid_size()
    start = nearest_free(occ, xy_cell(*start_xy), 10)
    goal = nearest_free(occ, xy_cell(*goal_xy), 10)
    if not start or not goal:
        return None
    def h(a, b): return math.hypot(a[0] - b[0], a[1] - b[1])
    q = [(h(start, goal), 0, start)]
    came = {}
    gscore = {start: 0}
    closed = set()
    moves = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    while q:
        _, g, cur = heapq.heappop(q)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return [cell_xy(c, r) for c, r in path]
        for dc, dr in moves:
            nb = (cur[0] + dc, cur[1] + dr)
            c, r = nb
            if c < 0 or c >= cols or r < 0 or r >= rows or occ[r][c]:
                continue
            ng = g + (math.sqrt(2) if dc and dr else 1)
            if ng < gscore.get(nb, 1e18):
                gscore[nb] = ng
                came[nb] = cur
                heapq.heappush(q, (ng + h(nb, goal), ng, nb))
    return None


def route_goal(marker):
    d = math.hypot(marker["x"], marker["y"])
    if d <= TARGET_STOP_RAW + 20:
        return 0, 0
    scale = (d - TARGET_STOP_RAW) / d
    return marker["x"] * scale, marker["y"] * scale


def plan_route(target):
    pts = scan_points(True)
    m = marker_copy(target)
    if not m or not m.get("locked"):
        return {"ok": False, "msg": f"{target.upper()} not locked", "path": [], "blocked": []}
    goal = route_goal(m)
    obs = planning_obstacles(pts, m)
    blocked = []
    for p in obs:
        if point_segment_dist(p["x"], p["y"], 0, 0, goal[0], goal[1]) <= ROUTE_RADIUS:
            blocked.append({"x": p["x"], "y": p["y"]})
            if len(blocked) >= 40:
                break
    if not blocked:
        path = [(0, 0), goal]
        msg = "STRAIGHT CLEAR"
    else:
        occ = build_costmap(obs)
        raw = astar(occ, (0, 0), goal)
        if not raw:
            return {"ok": False, "msg": "BLOCKED: no safe A* route", "path": [], "blocked": blocked}
        path = raw[::max(1, len(raw)//12)]
        if path[-1] != raw[-1]:
            path.append(raw[-1])
        msg = "REROUTE PLANNED"
    js_path = [{"x": round(x,1), "y": round(y,1)} for x,y in path]
    with lock:
        S["route_path"] = js_path
        S["blocked_points"] = blocked
        S["route_msg"] = f"{msg}: {len(js_path)} waypoint(s), corridor {int(ROUTE_RADIUS*2)}mm"
    return {"ok": True, "msg": S["route_msg"], "path": js_path, "blocked": blocked}


# =====================
# Metrics/tracking loop
# =====================
def metrics_loop():
    while True:
        pts_full = scan_points(True)
        pts_ui = scan_points(False)
        front = [p["d"] for p in pts_full if p["deg"] <= FRONT_DEG or p["deg"] >= 360 - FRONT_DEG]
        raw = min(front) if front else 0
        updates = {}
        with lock:
            markers_copy = dict(S["markers"])
        for name, m in markers_copy.items():
            if not m:
                continue
            near = find_relock_point(m, pts_full)
            if near:
                updates[name] = make_cluster(near, pts_full)
            else:
                lost = dict(m)
                lost["lost_count"] = int(lost.get("lost_count", 0)) + 1
                if lost["lost_count"] > MARKER_LOST_LIMIT:
                    lost["locked"] = False
                updates[name] = lost
        with lock:
            S["points"] = len(pts_full)
            S["ui_scan"] = [[p["x"], p["y"], p["i"]] for p in pts_ui]
            S["front_raw"] = int(raw)
            S["front_gap"] = int(raw - LIDAR_TO_FRONT) if raw else 0
            for k, v in updates.items():
                S["markers"][k] = v
            ESP["age_s"] = round(time.time() - float(ESP.get("last_update", 0)), 1)
            if ESP["age_s"] > 8:
                ESP["n1_online"] = False
                ESP["n2_online"] = False
        time.sleep(UPDATE_S)


# =====================
# Motors/mission
# =====================
def marker_copy(name):
    with lock:
        m = S["markers"].get(name)
        return dict(m) if m else None


def open_bot():
    from Rosmaster_Lib import Rosmaster
    bot = Rosmaster(com=YAHBOOM_PORT)
    bot.create_receive_threading()
    bot.set_auto_report_state(True, False)
    time.sleep(0.8)
    return bot


def stop_bot(bot=None):
    try:
        if bot:
            bot.set_motor(0, 0, 0, 0)
        else:
            from Rosmaster_Lib import Rosmaster
            b = Rosmaster(com=YAHBOOM_PORT)
            b.set_motor(0, 0, 0, 0)
    except Exception:
        pass


def drive_forward(bot, fine=False):
    p = FINE_FORWARD_POWER if fine else FORWARD_POWER
    bot.set_motor(p, p, p, p)


def drive_backward(bot, fine=False):
    p = FINE_BACKWARD_POWER if fine else BACKWARD_POWER
    bot.set_motor(p, p, p, p)


def spin(bot, direction):
    direction = (1 if direction > 0 else -1) * TURN_SIGN
    if direction > 0:
        bot.set_motor(TURN_POWER, TURN_POWER, -TURN_POWER, -TURN_POWER)
    else:
        bot.set_motor(-TURN_POWER, -TURN_POWER, TURN_POWER, TURN_POWER)


def face_marker(bot, target, timeout=11):
    global stop_requested
    event(f"Facing {target.upper()}", "i")
    t0 = time.time()
    last_abs = 999
    reverse = 1
    while time.time() - t0 < timeout:
        if stop_requested:
            stop_bot(bot); return False
        m = marker_copy(target)
        if not m or not m.get("locked"):
            event(f"{target.upper()} lost while facing", "e"); stop_bot(bot); return False
        a = signed_angle(m["angle"])
        with lock:
            MISSION["message"] = f"Facing {target.upper()}: angle={a:.1f}°"
        if abs(a) <= FACE_TOL_DEG:
            stop_bot(bot); return True
        if abs(a) > last_abs + 9:
            reverse *= -1
        last_abs = abs(a)
        spin(bot, (1 if a > 0 else -1) * reverse)
        time.sleep(0.07)
    stop_bot(bot); event(f"Face timeout {target.upper()}", "e"); return False


def mini_obstacle_check(target):
    """
    B4 hard safety:
    If front raw is too close, stop even if the marker distance still looks okay.
    This prevents bumping the obstacle due to delay/momentum/cluster offset.
    """
    m = marker_copy(target)
    with lock:
        front = S["front_raw"]
    if not m:
        return False

    # If target is basically reached, this is okay.
    if float(m["dist"]) <= TARGET_STOP_RAW + 20:
        return False

    # Otherwise a close front hit is dangerous.
    if front and front <= CRITICAL_FRONT_RAW:
        return True

    return False


def desired_drive_mode(target, marker_angle):
    """
    Return 'forward' or 'backward'.
    For HOME, if it is behind us, use the 360 LiDAR and drive backward.
    This avoids a full 180 turn.
    """
    a = abs(signed_angle(marker_angle))
    if target == "home" and a >= BACKWARD_HOME_ANGLE_DEG:
        return "backward"
    return "forward"


def heading_error_for_mode(marker_angle, mode):
    """
    Forward wants marker at 0 degrees.
    Backward wants marker at 180/-180 degrees.
    """
    if mode == "backward":
        return signed_angle(marker_angle - 180.0)
    return signed_angle(marker_angle)


def face_marker_for_mode(bot, target, mode="forward", timeout=8):
    global stop_requested

    event(f"Facing {target.upper()} for {mode} drive", "i")
    t0 = time.time()
    last_abs = 999
    reverse = 1

    while time.time() - t0 < timeout:
        if stop_requested:
            stop_bot(bot)
            return False

        m = marker_copy(target)
        if not m or not m.get("locked"):
            event(f"{target.upper()} lost while facing", "e")
            stop_bot(bot)
            return False

        err = heading_error_for_mode(float(m["angle"]), mode)

        with lock:
            MISSION["message"] = f"Facing {target.upper()} {mode}: err={err:.1f}°"

        if abs(err) <= FACE_TOL_DEG:
            stop_bot(bot)
            return True

        # If the error is getting worse, reverse the spin direction.
        if abs(err) > last_abs + 8:
            reverse *= -1
        last_abs = abs(err)

        spin(bot, (1 if err > 0 else -1) * reverse)
        time.sleep(0.06)

    stop_bot(bot)
    event(f"Face timeout {target.upper()}", "e")
    return False


def drive_to_marker(bot, target, timeout=MISSION_TIMEOUT_S):
    """
    B4 LiDAR odometry drive:
    - Uses live marker distance as odometry: moved = start_dist - current_dist for forward.
    - For HOME behind robot, drives backward while keeping HOME at ~180 degrees.
    - Slows near target.
    - Stops on marker distance OR front emergency raw.
    - Replans visually while moving.
    """
    global stop_requested

    m0 = marker_copy(target)
    if not m0:
        return False

    start_dist = float(m0["dist"])
    mode = desired_drive_mode(target, float(m0["angle"]))

    if not face_marker_for_mode(bot, target, mode=mode, timeout=8):
        return False

    event(f"Driving {mode} to {target.upper()} until {TARGET_STOP_RAW}mm raw", "i")

    t0 = time.time()
    last_replan = 0

    while time.time() - t0 < timeout:
        if stop_requested:
            stop_bot(bot)
            return False

        m = marker_copy(target)
        if not m or not m.get("locked"):
            stop_bot(bot)
            event(f"{target.upper()} lost while driving", "e")
            return False

        dist = float(m["dist"])
        angle = float(m["angle"])
        err = heading_error_for_mode(angle, mode)

        # LiDAR odometry estimate.
        if mode == "forward":
            moved = max(0, start_dist - dist)
        else:
            moved = max(0, dist - start_dist)

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = (
                f"{mode} to {target.upper()}: "
                f"dist={dist:.0f}mm moved={moved:.0f}mm err={err:.1f}°"
            )

        # Replan visual route while moving.
        if time.time() - last_replan >= REPLAN_EVERY_S:
            try:
                plan_route(target)
            except Exception:
                pass
            last_replan = time.time()

        # Stop rule: marker reached.
        if dist <= TARGET_STOP_RAW:
            stop_bot(bot)
            event(f"Reached {target.upper()} at marker dist={dist:.0f}mm", "k")
            return True

        # Stop rule: physical front too close.
        with lock:
            front = S["front_raw"]

        if mode == "forward" and front and front <= CRITICAL_FRONT_RAW:
            stop_bot(bot)
            event(f"Emergency front stop at raw={front}mm", "w")
            return True

        # If angle error grows, stop and re-face.
        if abs(err) > DRIVE_ANGLE_TOL_DEG:
            stop_bot(bot)
            if not face_marker_for_mode(bot, target, mode=mode, timeout=5):
                return False

        fine = dist <= SLOW_APPROACH_RAW

        if mode == "backward":
            drive_backward(bot, fine=fine)
        else:
            drive_forward(bot, fine=fine)

        time.sleep(0.055)

    stop_bot(bot)
    event(f"Drive timeout {target.upper()}", "e")
    return False



def pump_off(bot):
    try:
        if hasattr(bot, "set_pwm_servo"):
            bot.set_pwm_servo(SPRAY_SERVO_ID, SPRAY_OFF_ANGLE)
            time.sleep(0.12)
            return True
        event("Pump OFF failed: set_pwm_servo not found.", "e")
        return False
    except Exception as e:
        event("Pump OFF error: " + str(e), "e")
        return False


def pump_on(bot):
    try:
        if hasattr(bot, "set_pwm_servo"):
            bot.set_pwm_servo(SPRAY_SERVO_ID, SPRAY_ON_ANGLE)
            time.sleep(0.12)
            return True
        event("Pump ON failed: set_pwm_servo not found.", "e")
        return False
    except Exception as e:
        event("Pump ON error: " + str(e), "e")
        return False


def spray_step(bot):
    """
    B5 real spray step using Yahboom PWM Servo 1 signal pin.
    MOSFET signal should be on Servo 1 signal, with common GND.
    """
    if not SPRAY_ENABLED:
        event("Spray placeholder: disabled", "w")
        time.sleep(0.7)
        return True

    event(f"SPRAY ON: servo {SPRAY_SERVO_ID} angle {SPRAY_ON_ANGLE}", "w")
    ok = pump_on(bot)

    time.sleep(SPRAY_SECONDS)

    event("SPRAY OFF", "k")
    pump_off(bot)

    if not ok:
        event("Spray command attempted but pump API failed. Continuing mission.", "e")

    return True


def mission_worker(target):
    global mission_active, stop_requested, auto_cooldown_until
    mission_active = True
    stop_requested = False
    auto_cooldown_until = time.time() + AUTO_COOLDOWN_S
    with lock:
        MISSION.update({"state": "RUNNING", "target": target.upper(), "message": "Starting", "last_error": "", "started_at": time.time(), "finished_at": 0, "moved_mm": 0})
    bot = None
    try:
        if not robot_enabled:
            raise RuntimeError("ROBOT is OFF")
        if not marker_copy(target) or not marker_copy(target).get("locked"):
            raise RuntimeError(f"{target.upper()} marker not locked")
        if target != "home" and (not marker_copy("home") or not marker_copy("home").get("locked")):
            raise RuntimeError("HOME marker not locked")
        plan_route(target)
        bot = open_bot()
        stop_bot(bot)
        if not drive_to_marker(bot, target):
            raise RuntimeError("Could not reach target")
        if target != "home":
            spray_step(bot)
            plan_route("home")
            if not drive_to_marker(bot, "home"):
                raise RuntimeError("Could not reach home")
        stop_bot(bot)
        with lock:
            MISSION.update({"state": "DONE", "message": "Mission complete", "finished_at": time.time()})
        event("Mission complete", "k")
    except Exception as e:
        stop_bot(bot)
        with lock:
            MISSION.update({"state": "ERROR", "message": str(e), "last_error": str(e), "finished_at": time.time()})
        event("Mission error: " + str(e), "e")
    finally:
        try:
            if bot:
                pump_off(bot)
        except Exception:
            pass
        stop_bot(bot)
        mission_active = False
        auto_cooldown_until = time.time() + AUTO_COOLDOWN_S


def start_mission(target):
    global mission_active, stop_requested
    if mission_active:
        return False, "Mission already running"
    if target not in ["m1", "m2", "home"]:
        return False, "Bad target"
    stop_requested = False
    mission_active = True
    threading.Thread(target=mission_worker, args=(target,), daemon=True).start()
    return True, f"Started {target.upper()}"


def auto_loop():
    global auto_cooldown_until
    while True:
        with lock:
            a1 = ESP["alert1"]
            a2 = ESP["alert2"]
        if auto_enabled and robot_enabled and not mission_active and time.time() >= auto_cooldown_until:
            if a1:
                ok, _ = start_mission("m1")
                if ok:
                    event("AUTO ALERT1 -> M1", "w")
            elif a2:
                ok, _ = start_mission("m2")
                if ok:
                    event("AUTO ALERT2 -> M2", "w")
        time.sleep(0.25)



def preferred_face_target():
    """
    Priority:
    1. If ALERT2 is active and M2 is locked, face M2.
    2. Otherwise face M1 if M1 is locked.
    This matches FPMS demo behavior: default attention is M1, but M2 alert overrides.
    """
    with lock:
        a2 = bool(ESP.get("alert2"))
        a1 = bool(ESP.get("alert1"))

    m2 = marker_copy("m2")
    m1 = marker_copy("m1")

    if a2 and m2 and m2.get("locked"):
        return "m2"

    if m1 and m1.get("locked"):
        return "m1"

    if a1 and m1 and m1.get("locked"):
        return "m1"

    return None


def face_hold_loop():
    """
    B5 continuous target facing.
    Only active when:
    - ROBOT is ON
    - no mission is active
    - M1/M2 target is locked

    It keeps the chassis pointed at the selected marker using live LiDAR angle.
    This is not run during mission movement so it does not fight drive control.
    """
    global stop_requested

    bot = None
    last_target = None

    while True:
        try:
            if (not FACE_HOLD_ENABLED) or (not robot_enabled) or mission_active:
                if bot is not None:
                    stop_bot(bot)
                    pump_off(bot)
                    try:
                        del bot
                    except Exception:
                        pass
                    bot = None
                time.sleep(0.20)
                continue

            target = preferred_face_target()

            if not target:
                if bot is not None:
                    stop_bot(bot)
                time.sleep(0.20)
                continue

            m = marker_copy(target)
            if not m or not m.get("locked"):
                if bot is not None:
                    stop_bot(bot)
                time.sleep(0.20)
                continue

            # Do not spin when extremely close to the target.
            if float(m.get("dist", 99999)) < FACE_HOLD_MIN_DIST:
                if bot is not None:
                    stop_bot(bot)
                time.sleep(0.20)
                continue

            if bot is None:
                bot = open_bot()
                stop_bot(bot)
                last_target = target
                event(f"FACE-HOLD active: {target.upper()}", "i")

            if last_target != target:
                stop_bot(bot)
                last_target = target
                event(f"FACE-HOLD switched to {target.upper()}", "w")

            err = signed_angle(float(m["angle"]))

            with lock:
                if MISSION.get("state") in ["IDLE", "DONE", "ERROR", "STOPPING"]:
                    MISSION["state"] = "FACE-HOLD"
                    MISSION["target"] = target.upper()
                    MISSION["message"] = f"Facing {target.upper()}: err={err:.1f}°"

            if abs(err) <= FACE_HOLD_TOL_DEG:
                stop_bot(bot)
            else:
                spin(bot, 1 if err > 0 else -1)

            time.sleep(FACE_HOLD_INTERVAL_S)

        except Exception as e:
            event("Face-hold error: " + str(e), "e")
            try:
                if bot is not None:
                    stop_bot(bot)
            except Exception:
                pass
            try:
                del bot
            except Exception:
                pass
            bot = None
            time.sleep(0.5)


app = Flask(__name__)

PAGE = r'''
<!doctype html><html><head><meta charset="utf-8"><title>FPMS B8.4 LIDAR GAP</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#040911;color:#e8f7ff;font-family:Consolas,monospace;overflow:hidden}.top{height:58px;background:#071629;border-bottom:2px solid #ff4b70;padding:8px 14px;display:flex;justify-content:space-between;align-items:center}h1{margin:0;color:#ff4b70;letter-spacing:4px;font-size:22px}.sub{font-size:12px;color:#98d4ff}.main{height:calc(100vh - 58px);display:grid;grid-template-columns:1fr 360px;gap:8px;padding:8px}.panel{background:#071629;border:1px solid #28506f;border-radius:12px;padding:10px;min-height:0}.h{color:#00b7ff;letter-spacing:3px;font-weight:900;font-size:13px;border-bottom:1px solid #28506f;padding-bottom:6px;margin-bottom:8px}#mapbox{height:calc(100% - 28px);position:relative;background:#020711;border:1px solid #173450;border-radius:10px}canvas{position:absolute;left:0;top:0;width:100%;height:100%}.grid2{display:grid;grid-template-columns:1fr 1fr;gap:6px}button{background:#071427;border:1px solid #2e76aa;color:#e8f5ff;border-radius:9px;padding:8px;font-family:Consolas;font-weight:900}.green{border-color:#3af07a;color:#3af07a}.yellow{border-color:#ffbf00;color:#ffbf00}.danger{border-color:#ff4b70;color:#ff4b70}.active{background:#241900;border-color:#ffbf00;color:#ffbf00}.card{background:#020711;border:1px solid #1d4163;border-radius:9px;padding:8px;margin-top:6px}.big{font-size:18px;color:#00b7ff;font-weight:900}.small{font-size:11px;color:#98d4ff}.ok{color:#3af07a}.bad{color:#ff4b70}.warn{color:#ffbf00}#events{height:86px;overflow:auto;font-size:10px;white-space:pre-wrap}#leftDist{position:absolute;left:10px;top:10px;z-index:10;width:245px;background:rgba(2,7,17,.86);border:1px solid #1d4163;border-radius:10px;padding:9px;font-size:12px;line-height:1.45;color:#98d4ff}#leftDist .title{color:#00b7ff;letter-spacing:2px;font-weight:900;margin-bottom:5px}#leftDist b{color:#e8f7ff}#leftDist .ok{color:#3af07a}#leftDist .bad{color:#ff4b70}</style></head>
<body><div class="top"><div><h1>FPMS B8.4 LIDAR GAP</h1><div class="sub">200cm × 300cm centered map • one webserver • front_gap targets • M=15cm HOME=10cm • 2s spray</div></div><div id="clock"></div></div>
<div class="main"><div class="panel"><div class="h">LIVE MAP</div><div id="mapbox"><div id="leftDist"><div class="title">LIVE DISTANCES</div>Waiting...</div><canvas id="grid"></canvas><canvas id="dyn"></canvas></div></div>
<div class="panel"><div class="h">MARKERS</div><div class="grid2"><button id="b_m1" onclick="setMode('m1')">SET M1</button><button id="b_m2" onclick="setMode('m2')">SET M2</button><button id="b_home" onclick="setMode('home')">SET HOME</button><button class="danger" onclick="clearMarkers()">CLEAR</button></div>
<div class="h" style="margin-top:10px">MISSION</div><div class="grid2"><button id="robotBtn" class="danger" onclick="toggleRobot()">ROBOT OFF</button><button id="autoBtn" class="danger" onclick="toggleAuto()">AUTO OFF</button><button class="green" onclick="runMission('m1')">TEST FULL M1</button><button class="green" onclick="runMission('m2')">RUN M2</button><button class="yellow" onclick="runMission('home')">RUN HOME</button><button class="danger" onclick="stopMission()">STOP</button></div><div class="card"><div id="mission" class="big">--</div><div id="msg" class="small">--</div><div id="routeMsg" class="small">--</div></div>
<div class="h" style="margin-top:10px">ESP-NOW</div><div class="grid2"><div class="card"><div class="small">NODE1</div><div id="n1" class="big">--</div></div><div class="card"><div class="small">ALERT1</div><div id="a1" class="big">--</div></div><div class="card"><div class="small">NODE2</div><div id="n2" class="big">--</div></div><div class="card"><div class="small">ALERT2</div><div id="a2" class="big">--</div></div></div><div id="lastEsp" class="card small">--</div>
<div class="h" style="margin-top:10px">LIVE DISTANCES</div><div id="mread" class="card small">--</div>
<div class="h" style="margin-top:10px">DIAGNOSTICS</div><div class="grid2"><div class="card"><div class="small">STATUS</div><div id="status" class="big">--</div></div><div class="card"><div class="small">SCAN</div><div id="hz" class="big">0</div></div><div class="card"><div class="small">POINTS</div><div id="points" class="big">0</div></div><div class="card"><div class="small">FRONT</div><div id="front" class="big">--</div></div></div>
<div class="h" style="margin-top:10px">EVENTS</div><div id="events" class="card"></div></div></div>
<script>
const XMIN=-1000,XMAX=1000,YMIN=-1500,YMAX=1500,ROBOT_W=230,LIDAR_FRONT=200,TARGET_STOP=300,FRONT_STOP=300,ROUTE_RADIUS=185;let scale=1,ox=0,oy=0,mode=null,scan=[],markers={},route=[],blocked=[];const grid=document.getElementById('grid'),dyn=document.getElementById('dyn'),box=document.getElementById('mapbox'),g=grid.getContext('2d'),d=dyn.getContext('2d');function $(x){return document.getElementById(x)}function mmToPix(x,y){return[ox+x*scale,oy-y*scale]}function pixToMm(px,py){return[(px-ox)/scale,(oy-py)/scale]}function resize(){let r=box.getBoundingClientRect();grid.width=dyn.width=Math.floor(r.width);grid.height=dyn.height=Math.floor(r.height);scale=Math.min(grid.width/(XMAX-XMIN),grid.height/(YMAX-YMIN))*.96;ox=grid.width/2-(XMIN+XMAX)/2*scale;oy=grid.height/2+(YMIN+YMAX)/2*scale;drawGrid();drawDyn()}function drawGrid(){g.clearRect(0,0,grid.width,grid.height);let[x0,yt]=mmToPix(XMIN,YMAX),[x1,yb]=mmToPix(XMAX,YMIN);g.fillStyle='rgba(0,55,28,.25)';g.fillRect(x0,yt,x1-x0,yb-yt);g.strokeStyle='#00d66b';g.lineWidth=2;g.strokeRect(x0,yt,x1-x0,yb-yt);for(let x=XMIN;x<=XMAX;x+=100){let[px]=mmToPix(x,0);g.strokeStyle=x%500?'rgba(80,190,255,.16)':'rgba(80,190,255,.36)';g.beginPath();g.moveTo(px,yt);g.lineTo(px,yb);g.stroke()}for(let y=YMIN;y<=YMAX;y+=100){let[,py]=mmToPix(0,y);g.strokeStyle=y%500?'rgba(80,190,255,.16)':'rgba(80,190,255,.36)';g.beginPath();g.moveTo(x0,py);g.lineTo(x1,py);g.stroke()}let[lx,fy]=mmToPix(-ROBOT_W/2,LIDAR_FRONT),[rx,by]=mmToPix(ROBOT_W/2,-80);g.fillStyle='rgba(0,183,255,.16)';g.strokeStyle='#00b7ff';g.fillRect(lx,fy,rx-lx,by-fy);g.strokeRect(lx,fy,rx-lx,by-fy);let[cx,cy]=mmToPix(0,0);g.fillStyle='#00b7ff';g.beginPath();g.arc(cx,cy,6,0,Math.PI*2);g.fill();let[,s1]=mmToPix(0,FRONT_STOP),[,s2]=mmToPix(0,TARGET_STOP);g.setLineDash([7,6]);g.strokeStyle='#ffbf00';g.beginPath();g.moveTo(x0,s1);g.lineTo(x1,s1);g.stroke();g.strokeStyle='#ff4b70';g.beginPath();g.moveTo(x0,s2);g.lineTo(x1,s2);g.stroke();g.setLineDash([])}function mc(n){return n==='m1'?'#ff4b70':n==='m2'?'#ffbf00':'#3af07a'}function drawMarker(n){let m=markers[n];if(!m)return;let[px,py]=mmToPix(m.x,m.y),c=mc(n);d.strokeStyle=m.locked?c:'#888';d.lineWidth=3;d.beginPath();d.arc(px,py,14,0,Math.PI*2);d.stroke();d.fillStyle=c;d.font='bold 12px Consolas';d.textAlign='center';d.fillText(n.toUpperCase(),px,py-18);d.textAlign='left'}function drawRoute(){if(!route||route.length<2)return;d.lineJoin='round';d.lineCap='round';d.beginPath();route.forEach((p,i)=>{let[px,py]=mmToPix(p.x,p.y);if(i===0)d.moveTo(px,py);else d.lineTo(px,py)});d.strokeStyle='rgba(58,240,122,.18)';d.lineWidth=ROUTE_RADIUS*2*scale;d.stroke();d.beginPath();route.forEach((p,i)=>{let[px,py]=mmToPix(p.x,p.y);if(i===0)d.moveTo(px,py);else d.lineTo(px,py)});d.strokeStyle='#3af07a';d.lineWidth=4;d.stroke()}let needsDraw=false;function drawDyn(){d.clearRect(0,0,dyn.width,dyn.height);for(const p of scan){let[px,py]=mmToPix(p[0],p[1]);d.fillStyle='rgb(240,210,255)';d.fillRect(px-2.5,py-2.5,5,5)}for(const p of blocked){let[px,py]=mmToPix(p.x,p.y);d.fillStyle='#ff4b70';d.beginPath();d.arc(px,py,5,0,Math.PI*2);d.fill()}drawRoute();drawMarker('m1');drawMarker('m2');drawMarker('home')}function setMode(m){mode=m;['m1','m2','home'].forEach(x=>$('b_'+x).classList.remove('active'));$('b_'+m).classList.add('active')}dyn.addEventListener('click',async e=>{if(!mode){alert('Choose marker first');return}let r=dyn.getBoundingClientRect();let[x,y]=pixToMm(e.clientX-r.left,e.clientY-r.top);let js=await(await fetch('/api/set_marker',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name:mode,x:x,y:y})})).json();if(!js.ok)alert(js.msg)});async function clearMarkers(){await fetch('/api/clear_markers',{method:'POST'})}async function toggleRobot(){await fetch('/api/toggle_robot',{method:'POST'});poll()}async function toggleAuto(){await fetch('/api/toggle_auto',{method:'POST'});poll()}async function runMission(t){if(!confirm('Robot will move. Area clear?'))return;let js=await(await fetch('/api/run',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({target:t})})).json();if(!js.ok)alert(js.msg)}async function stopMission(){await fetch('/api/stop',{method:'POST'})}function line(n,m){return m?`${n.toUpperCase()}: ${m.locked?'LOCKED':'LOST'} dist=${Math.round(m.dist)}mm gap=${Math.round(m.front_gap)}mm angle=${Math.round(m.angle)}°`:`${n.toUpperCase()}: empty`}function leftLine(n,m){if(!m)return `<b>${n.toUpperCase()}</b>: <span class="bad">EMPTY</span>`;let cls=m.locked?'ok':'bad';return `<b>${n.toUpperCase()}</b>: <span class="${cls}">${m.locked?'LOCKED':'LOST'}</span><br>`+`&nbsp;dist ${Math.round(m.dist)}mm | gap ${Math.round(m.front_gap)}mm<br>`+`&nbsp;angle ${Math.round(m.angle)}° | pts ${m.points||0}`}async function poll(){try{let s=await(await fetch('/api/state?t='+Date.now())).json();scan=s.scan||[];markers=s.markers||{};route=s.route_path||[];blocked=s.blocked_points||[];$('status').textContent=s.status;$('hz').textContent=s.scan_hz;$('points').textContent=s.points;$('front').textContent=s.front_raw||'--';$('mission').textContent=s.mission.state+' → '+s.mission.target;$('msg').textContent=s.mission.message;$('routeMsg').textContent=s.route_msg;$('robotBtn').textContent=s.robot_enabled?'ROBOT ON':'ROBOT OFF';$('robotBtn').className=s.robot_enabled?'green':'danger';$('autoBtn').textContent=s.auto_enabled?'AUTO ON':'AUTO OFF';$('autoBtn').className=s.auto_enabled?'green':'danger';$('n1').textContent=s.esp.n1_online?'ONLINE':'OFFLINE';$('a1').textContent=s.esp.alert1?'ALERT':'CLEAR';$('n2').textContent=s.esp.n2_online?'ONLINE':'OFFLINE';$('a2').textContent=s.esp.alert2?'ALERT':'CLEAR';$('lastEsp').textContent='Last ESP: '+(s.esp.last_line||'none')+' | age '+s.esp.age_s+'s';$('mread').innerHTML=line('m1',markers.m1)+'<br>'+line('m2',markers.m2)+'<br>'+line('home',markers.home);if($('leftDist')){$('leftDist').innerHTML='<div class="title">LIVE DISTANCES</div>'+leftLine('m1',markers.m1)+'<hr style="border-color:#173450">'+leftLine('m2',markers.m2)+'<hr style="border-color:#173450">'+leftLine('home',markers.home)+'<hr style="border-color:#173450">'+`<b>FRONT</b>: ${s.front_raw||'--'}mm`;}$('events').textContent=(s.events||[]).slice().reverse().map(e=>'['+e.t+'] '+e.msg).join('\n');requestAnimationFrame(drawDyn)}catch(e){$('msg').textContent='Poll error '+e}setTimeout(poll,700)}window.addEventListener('resize',resize);resize();poll();setInterval(()=>{$('clock').textContent=new Date().toLocaleTimeString()},1000);
</script></body></html>
'''


@app.route('/')
def index():
    return render_template_string(PAGE)


@app.route('/api/state')
def api_state():
    with lock:
        return jsonify({
            "status": S["status"],
            "scan_hz": S["scan_hz"],
            "points": S["points"],
            "front_raw": S["front_raw"],
            "front_gap": S["front_gap"],
            "scan": list(S["ui_scan"]),
            "markers": dict(S["markers"]),
            "route_path": list(S["route_path"]),
            "blocked_points": list(S["blocked_points"]),
            "route_msg": S["route_msg"],
            "events": list(S["events"]),
            "esp": dict(ESP),
            "mission": dict(MISSION),
            "robot_enabled": robot_enabled,
            "auto_enabled": auto_enabled,
            "mission_active": mission_active
        })


@app.route('/api/set_marker', methods=['POST'])
def api_set_marker():
    data = request.get_json(force=True)
    name = str(data.get('name', '')).lower()
    if name not in ['m1', 'm2', 'home']:
        return jsonify(ok=False, msg='bad marker')
    pts = scan_points(True)
    p = nearest_point(float(data.get('x', 0)), float(data.get('y', 0)), pts, MARKER_SNAP_MM)
    if not p:
        return jsonify(ok=False, msg='No LiDAR dot close enough')
    m = make_cluster(p, pts)
    with lock:
        S['markers'][name] = m
    event(f'{name.upper()} locked dist={m["dist"]:.0f}mm angle={m["angle"]:.1f}', 'k')
    return jsonify(ok=True)


@app.route('/api/clear_markers', methods=['POST'])
def api_clear_markers():
    with lock:
        S['markers'] = {'m1': None, 'm2': None, 'home': None}
        S['route_path'] = []
        S['blocked_points'] = []
    event('Markers cleared', 'w')
    return jsonify(ok=True)


@app.route('/api/toggle_robot', methods=['POST'])
def api_toggle_robot():
    global robot_enabled
    robot_enabled = not robot_enabled
    if not robot_enabled:
        stop_bot()
    event('ROBOT ON' if robot_enabled else 'ROBOT OFF', 'w' if robot_enabled else 'k')
    return jsonify(ok=True, robot_enabled=robot_enabled)


@app.route('/api/toggle_auto', methods=['POST'])
def api_toggle_auto():
    global auto_enabled
    auto_enabled = not auto_enabled
    event('AUTO ON' if auto_enabled else 'AUTO OFF', 'w' if auto_enabled else 'k')
    return jsonify(ok=True, auto_enabled=auto_enabled)


@app.route('/api/run', methods=['POST'])
def api_run():
    data = request.get_json(force=True)
    target = str(data.get('target', '')).lower()
    ok, msg = start_mission(target)
    return jsonify(ok=ok, msg=msg)


@app.route('/api/stop', methods=['POST'])
def api_stop():
    global stop_requested
    stop_requested = True
    stop_bot()
    with lock:
        MISSION['state'] = 'STOPPING'
        MISSION['message'] = 'Stop requested'
    event('Stop requested', 'w')
    return jsonify(ok=True)


@app.route('/api/plan', methods=['POST'])
def api_plan():
    data = request.get_json(force=True)
    target = str(data.get('target', '')).lower()
    if target not in ['m1', 'm2', 'home']:
        return jsonify(ok=False, msg='bad target')
    return jsonify(plan_route(target))




# ============================================================
# FPMS B6 STABLE TURN / FACE / RETURN OVERRIDES
# ============================================================

# B6 philosophy:
# - Do not rely on weak continuous turns.
# - Use strong short tank-turn pulses.
# - After every pulse, stop and re-read the LiDAR marker angle.
# - If the error gets worse, flip runtime turn direction.
# - Home can be handled by driving backward if it is behind the robot.
# - Spray is disabled for now.

SPRAY_ENABLED = False

B6_TURN_POWER = 30
B6_TURN_PULSE_S = 0.18
B6_TURN_SETTLE_S = 0.10
B6_FACE_TOL_DEG = 7
B6_DRIVE_ANGLE_TOL_DEG = 16
B6_TARGET_STOP_RAW = 380
B6_HOME_STOP_RAW = 340
B6_CRITICAL_FRONT_RAW = 355
B6_SLOW_APPROACH_RAW = 560
B6_FINE_FORWARD_POWER = -14
B6_FINE_BACKWARD_POWER = 15
B6_FORWARD_POWER = -21
B6_BACKWARD_POWER = 24
B6_HOME_BACKWARD_ANGLE = 105
B6_FACEHOLD_INTERVAL_S = 0.42
B6_FACEHOLD_MIN_DIST = 430
B6_AUTO_FACEHOLD = True

# Runtime correction. If first turn direction is wrong, the code flips this.
B6_RUNTIME_TURN_SIGN = 1


def b6_motor_stop(bot):
    try:
        bot.set_motor(0, 0, 0, 0)
    except Exception:
        pass


def b6_forward(bot, fine=False):
    p = B6_FINE_FORWARD_POWER if fine else B6_FORWARD_POWER
    bot.set_motor(p, p, p, p)


def b6_backward(bot, fine=False):
    p = B6_FINE_BACKWARD_POWER if fine else B6_BACKWARD_POWER
    bot.set_motor(p, p, p, p)


def b6_tank_turn(bot, direction, power=None, pulse=True):
    """
    direction: +1 or -1.
    Pattern uses one side forward, one side backward:
    left forward/right backward or the reverse.
    On this robot, raw negative = forward, raw positive = backward.
    """
    global B6_RUNTIME_TURN_SIGN

    p = int(power or B6_TURN_POWER)
    d = 1 if direction > 0 else -1
    d *= TURN_SIGN
    d *= B6_RUNTIME_TURN_SIGN

    if d > 0:
        # left forward, right backward
        bot.set_motor(-p, -p, p, p)
    else:
        # left backward, right forward
        bot.set_motor(p, p, -p, -p)

    if pulse:
        time.sleep(B6_TURN_PULSE_S)
        b6_motor_stop(bot)
        time.sleep(B6_TURN_SETTLE_S)


def b6_marker_error(target, mode="forward"):
    m = marker_copy(target)
    if not m or not m.get("locked"):
        return None, None

    angle = float(m["angle"])
    dist = float(m["dist"])

    if mode == "backward":
        err = signed_angle(angle - 180.0)
    else:
        err = signed_angle(angle)

    return err, dist


def b6_choose_drive_mode(target):
    """
    B6.3:
    M1/M2 = forward.
    HOME = backward when HOME is behind or side-behind.
    This uses the 360° LiDAR as the odometry ruler instead of turning
    the front of the robot into the HOME marker.
    """
    if target != "home":
        return "forward"

    m = marker_copy("home")
    if not m:
        return "forward"

    a = abs(signed_angle(float(m["angle"])))

    # If HOME is anywhere mostly behind us, reverse into it.
    if a >= 70:
        return "backward"

    return "forward"


def b6_face_marker(bot, target, mode=None, timeout=12.0):
    """
    Stable LiDAR face routine.
    It turns in short tank pulses and re-checks the marker after each pulse.
    This is much better for a heavy 4-wheel skid-steer robot than weak continuous turning.
    """
    global B6_RUNTIME_TURN_SIGN, stop_requested

    if mode is None:
        mode = b6_choose_drive_mode(target)

    event(f"B6 facing {target.upper()} for {mode}", "i")

    t0 = time.time()
    last_abs = None
    bad_steps = 0

    while time.time() - t0 < timeout:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        err, dist = b6_marker_error(target, mode)
        if err is None:
            b6_motor_stop(bot)
            event(f"{target.upper()} marker lost during B6 face", "e")
            return False

        with lock:
            MISSION["message"] = f"B6 face {target.upper()} {mode}: err={err:.1f}° dist={dist:.0f}mm"

        if abs(err) <= B6_FACE_TOL_DEG:
            b6_motor_stop(bot)
            event(f"B6 face complete {target.upper()} err={err:.1f}°", "k")
            return True

        before = abs(err)
        direction = 1 if err > 0 else -1

        # Strong pulse turn.
        b6_tank_turn(bot, direction, pulse=True)

        err2, dist2 = b6_marker_error(target, mode)
        if err2 is None:
            continue

        after = abs(err2)

        # If turning made it worse, flip runtime sign.
        if last_abs is not None and after > before + 4:
            bad_steps += 1
        else:
            bad_steps = 0

        if bad_steps >= 2:
            B6_RUNTIME_TURN_SIGN *= -1
            bad_steps = 0
            event("B6 flipped runtime turn direction", "w")

        last_abs = after

    b6_motor_stop(bot)
    event(f"B6 face timeout {target.upper()}", "e")
    return False


# Override older face_marker name too.
def face_marker(bot, target, timeout=12.0):
    return b6_face_marker(bot, target, mode=None, timeout=timeout)


def face_marker_for_mode(bot, target, mode="forward", timeout=12.0):
    return b6_face_marker(bot, target, mode=mode, timeout=timeout)


def b6_front_too_close(target, dist):
    with lock:
        front = S.get("front_raw", 0)

    if not front:
        return False

    # When target is basically reached, close front is expected.
    if dist <= B6_TARGET_STOP_RAW + 25:
        return False

    return front <= B6_CRITICAL_FRONT_RAW


def drive_to_marker(bot, target, timeout=MISSION_TIMEOUT_S):
    """
    B6 LiDAR odometry:
    - distance to locked marker is the odometry ruler.
    - if start_dist = 700 and now = 450, moved about 250 mm.
    - forward to M1/M2; backward to HOME if HOME is behind.
    """
    global stop_requested

    m0 = marker_copy(target)
    if not m0 or not m0.get("locked"):
        event(f"B6 cannot drive: {target.upper()} not locked", "e")
        return False

    start_dist = float(m0["dist"])
    mode = b6_choose_drive_mode(target)
    stop_raw = B6_HOME_STOP_RAW if target == "home" else B6_TARGET_STOP_RAW

    if not b6_face_marker(bot, target, mode=mode, timeout=12):
        return False

    event(f"B6 driving {mode} to {target.upper()} until {stop_raw}mm", "i")

    t0 = time.time()
    last_reface = 0

    while time.time() - t0 < timeout:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        m = marker_copy(target)
        if not m or not m.get("locked"):
            b6_motor_stop(bot)
            event(f"{target.upper()} marker lost while B6 driving", "e")
            return False

        dist = float(m["dist"])
        err, _ = b6_marker_error(target, mode)

        if err is None:
            b6_motor_stop(bot)
            return False

        if mode == "backward":
            moved = abs(dist - start_dist)
        else:
            moved = max(0, start_dist - dist)

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = (
                f"B6 {mode} {target.upper()} stop={stop_raw:.0f}: "
                f"dist={dist:.0f}mm moved={moved:.0f}mm err={err:.1f}°"
            )

        # Stop at safe target distance.
        if dist <= stop_raw:
            b6_motor_stop(bot)
            event(f"B6 reached {target.upper()} dist={dist:.0f}mm", "k")
            return True

        # Emergency front stop only for forward driving.
        if mode == "forward" and b6_front_too_close(target, dist):
            b6_motor_stop(bot)
            event("B6 emergency front stop", "w")
            return True

        # Re-face if angle goes bad.
        if abs(err) > B6_DRIVE_ANGLE_TOL_DEG and time.time() - last_reface > 0.7:
            b6_motor_stop(bot)
            if not b6_face_marker(bot, target, mode=mode, timeout=6):
                return False
            last_reface = time.time()

        fine = dist <= B6_SLOW_APPROACH_RAW

        if mode == "backward":
            b6_backward(bot, fine=fine)
        else:
            b6_forward(bot, fine=fine)

        time.sleep(0.06)

    b6_motor_stop(bot)
    event(f"B6 drive timeout {target.upper()}", "e")
    return False


def spray_step(bot):
    event(f"B6 SPRAY ON: S1 angle {SPRAY_ON_ANGLE}", "w")
    try:
        bot.set_pwm_servo(SPRAY_SERVO_ID, SPRAY_ON_ANGLE)
        time.sleep(SPRAY_SECONDS)
        bot.set_pwm_servo(SPRAY_SERVO_ID, SPRAY_OFF_ANGLE)
        event("B6 SPRAY OFF", "k")
        time.sleep(0.2)
        return True
    except Exception as e:
        event("B6 spray error: " + str(e), "e")
        try:
            bot.set_pwm_servo(SPRAY_SERVO_ID, SPRAY_OFF_ANGLE)
        except Exception:
            pass
        return False


def b6_preferred_face_target():
    """
    Priority:
    - If ALERT2 and M2 locked: face M2
    - Else if ALERT1 and M1 locked: face M1
    - Else face M1 by default if locked
    """
    with lock:
        a1 = bool(ESP.get("alert1"))
        a2 = bool(ESP.get("alert2"))

    m1 = marker_copy("m1")
    m2 = marker_copy("m2")

    if a2 and m2 and m2.get("locked"):
        return "m2"

    if a1 and m1 and m1.get("locked"):
        return "m1"

    if m1 and m1.get("locked"):
        return "m1"

    return None


def face_hold_loop():
    """
    Always-face loop.
    It only turns when robot is ON and no mission is active.
    It uses B6 pulse turns so it should not stall like the old weak turn.
    """
    bot = None
    last_target = None

    while True:
        try:
            if (not B6_AUTO_FACEHOLD) or (not robot_enabled) or mission_active:
                if bot is not None:
                    b6_motor_stop(bot)
                    try:
                        del bot
                    except Exception:
                        pass
                    bot = None
                time.sleep(0.20)
                continue

            target = b6_preferred_face_target()
            if not target:
                if bot is not None:
                    b6_motor_stop(bot)
                time.sleep(0.20)
                continue

            m = marker_copy(target)
            if not m or not m.get("locked"):
                if bot is not None:
                    b6_motor_stop(bot)
                time.sleep(0.20)
                continue

            if float(m.get("dist", 99999)) < B6_FACEHOLD_MIN_DIST:
                if bot is not None:
                    b6_motor_stop(bot)
                time.sleep(0.20)
                continue

            if bot is None:
                bot = open_bot()
                b6_motor_stop(bot)
                last_target = target
                event(f"B6 FACE-HOLD active: {target.upper()}", "i")

            if target != last_target:
                b6_motor_stop(bot)
                last_target = target
                event(f"B6 FACE-HOLD switched to {target.upper()}", "w")

            err, dist = b6_marker_error(target, mode="forward")
            if err is None:
                time.sleep(0.20)
                continue

            with lock:
                if MISSION.get("state") in ["IDLE", "DONE", "ERROR", "STOPPING", "FACE-HOLD"]:
                    MISSION["state"] = "FACE-HOLD"
                    MISSION["target"] = target.upper()
                    MISSION["message"] = f"B6 face-hold {target.upper()}: err={err:.1f}°"

            if abs(err) <= B6_FACE_TOL_DEG:
                b6_motor_stop(bot)
            else:
                b6_tank_turn(bot, 1 if err > 0 else -1, pulse=True)

            time.sleep(B6_FACEHOLD_INTERVAL_S)

        except Exception as e:
            event("B6 face-hold error: " + str(e), "e")
            try:
                if bot is not None:
                    b6_motor_stop(bot)
            except Exception:
                pass
            bot = None
            time.sleep(0.5)




# ============================================================
# FPMS B8.4 LIDAR GAP OVERRIDES
# Dead-simple mission:
# - Face target ONCE.
# - Drive forward only until target LiDAR distance says stop.
# - Spray.
# - Drive backward only until HOME LiDAR distance says stop.
# - No turning during route.
# - No idle face-hold.
# ============================================================

SPRAY_ENABLED = False
STRAIGHT_TARGET_STOP_RAW = 390
STRAIGHT_HOME_STOP_RAW = 300

STRAIGHT_FORWARD_PULSE_S = 0.12
STRAIGHT_BACKWARD_PULSE_S = 0.14
STRAIGHT_SETTLE_S = 0.06

STRAIGHT_LOST_LIMIT = 6
STRAIGHT_TIMEOUT_S = 45

AUTO_POLL_S = 0.20
AUTO_SERVED = {"m1": False, "m2": False}


def force_safe_outputs(bot=None):
    try:
        if bot:
            bot.set_motor(0, 0, 0, 0)
            try:
                bot.set_pwm_servo(1, 0)
            except Exception:
                pass
        else:
            from Rosmaster_Lib import Rosmaster
            b = Rosmaster(com=YAHBOOM_PORT)
            b.set_motor(0, 0, 0, 0)
            try:
                b.set_pwm_servo(1, 0)
            except Exception:
                pass
    except Exception:
        pass


def open_bot():
    from Rosmaster_Lib import Rosmaster
    bot = Rosmaster(com=YAHBOOM_PORT)
    force_safe_outputs(bot)
    time.sleep(0.15)
    return bot


def spray_step(bot):
    force_safe_outputs(bot)
    event("SPRAY SKIPPED: disabled while pump/battery is tested", "w")
    time.sleep(0.4)
    return True


def face_hold_loop():
    """
    No idle face-hold.
    Robot stays still/facing forward until mission/alert.
    """
    while True:
        time.sleep(1.0)


def b6_preferred_face_target():
    return None


def preferred_face_target():
    return None


def marker_dist(name):
    m = marker_copy(name)
    if not m or not m.get("locked"):
        return None
    return float(m.get("dist", 999999))



def marker_gap(name):
    m = marker_copy(name)
    if not m or not m.get("locked"):
        return None
    try:
        return float(m.get("gap", 999999))
    except Exception:
        return None

def marker_angle(name):
    m = marker_copy(name)
    if not m or not m.get("locked"):
        return None
    return float(m.get("angle", 0))





STRAIGHT_TURN_SIGN = 1  # If turn direction is opposite, change to -1.




def slow_turn_pulse(bot, direction, *args, **kwargs):
    """
    B6 stronger face-correction tank turn.

    This is ONLY for face alignment:
    - short direct tank-turn pulse
    - stop
    - caller re-reads marker angle
    - no driving during face correction

    If robot turns away from marker, flip STRAIGHT_TURN_SIGN.
    """
    import time

    sign = int(globals().get("STRAIGHT_TURN_SIGN", 1))

    try:
        raw_dir = float(direction)
    except Exception:
        raw_dir = 0.0

    d = raw_dir * sign
    mag = abs(raw_dir)

    override_power = kwargs.get("power", kwargs.get("turn_power", None))
    override_ms = kwargs.get("pulse_ms", kwargs.get("ms", kwargs.get("duration_ms", None)))

    if len(args) >= 1 and override_power is None:
        override_power = args[0]
    if len(args) >= 2 and override_ms is None:
        override_ms = args[1]

    # STRONGER than previous B6.
    # Uses enough power to overcome friction, but still short pulses to prevent over-turn.
    if override_power is None:
        if mag >= 30:
            power = 46
        elif mag >= 18:
            power = 42
        elif mag >= 9:
            power = 38
        elif mag >= 4:
            power = 34
        else:
            power = 30
    else:
        power = int(override_power)

    if override_ms is None:
        if mag >= 30:
            pulse_ms = 125
        elif mag >= 18:
            pulse_ms = 105
        elif mag >= 9:
            pulse_ms = 85
        elif mag >= 4:
            pulse_ms = 65
        else:
            pulse_ms = 45
    else:
        pulse_ms = int(override_ms)

    # Safety clamps.
    power = max(28, min(50, power))
    pulse_ms = max(40, min(135, pulse_ms))

    if d == 0:
        bot.set_motor(0, 0, 0, 0)
        return

    # Direction:
    # d > 0 = left side backward, right side forward
    # d < 0 = left side forward, right side backward
    if d > 0:
        left_power = -power
        right_power = power
    else:
        left_power = power
        right_power = -power

    # Tiny kick to overcome static friction so BOTH sides actually move.
    kick_power = min(55, max(power + 8, 46))
    if d > 0:
        bot.set_motor(-kick_power, -kick_power, kick_power, kick_power)
    else:
        bot.set_motor(kick_power, kick_power, -kick_power, -kick_power)

    time.sleep(0.035)

    # Main controlled pulse.
    bot.set_motor(left_power, left_power, right_power, right_power)
    time.sleep(pulse_ms / 1000.0)

    # Hard stop after each pulse.
    bot.set_motor(0, 0, 0, 0)
    time.sleep(0.10)


def face_error_for_marker(target, mode="forward"):
    m = marker_copy(target)
    if not m or not m.get("locked"):
        return None, None

    # Need a real cluster, not just 1 random point.
    try:
        pts = int(m.get("points", 0))
    except Exception:
        pts = 0

    if pts < 3:
        return None, m

    angle = float(m.get("angle", 0))

    if mode == "backward":
        err = signed_angle(angle - 180.0)
    else:
        err = signed_angle(angle)

    return err, m


def straight_face_once_verified(bot, target, mode="forward", tolerance=6, attempts=1):
    """
    Precision face:
    - Does NOT drive until face is stable.
    - Uses tiny turn pulses.
    - If it overshoots, stops, waits, and corrects slowly.
    - Requires 3 stable readings inside tolerance.
    """
    event(f"SLOW FACE {target.upper()} mode={mode}", "i")

    stable_count = 0
    bad_count = 0
    last_err = None
    t0 = time.time()

    while time.time() - t0 < 18:
        err, m = face_error_for_marker(target, mode)

        if err is None:
            b6_motor_stop(bot)
            bad_count += 1
            event(f"SLOW FACE waiting: {target.upper()} marker weak/lost", "w")
            time.sleep(0.25)

            if bad_count >= 10:
                event(f"SLOW FACE failed: {target.upper()} marker not stable", "e")
                return False
            continue

        bad_count = 0
        abs_err = abs(err)

        with lock:
            MISSION["message"] = f"SLOW FACE {target.upper()}: err={err:.1f}°"

        # Stable enough: require multiple good readings before driving.
        if abs_err <= tolerance:
            b6_motor_stop(bot)
            stable_count += 1
            event(f"SLOW FACE {target.upper()} stable {stable_count}/3 err={err:.1f}°", "i")
            time.sleep(0.22)

            if stable_count >= 3:
                event(f"SLOW FACE {target.upper()} OK err={err:.1f}°", "k")
                b6_motor_stop(bot)
                time.sleep(0.75)   # pause before driving, prevents launch after overshoot
                return True

            continue

        stable_count = 0

        # Overshoot detection: sign changed while close-ish.
        if last_err is not None:
            if (err > 0 and last_err < 0) or (err < 0 and last_err > 0):
                if abs_err <= 28:
                    b6_motor_stop(bot)
                    event(f"SLOW FACE overshoot detected err={err:.1f}°, pausing", "w")
                    time.sleep(0.45)

        last_err = err

        # Direction and power selection.
        direction = 1 if err > 0 else -1

        if abs_err > 35:
            power = 34
        elif abs_err > 18:
            power = 30
        elif abs_err > 10:
            power = 27
        else:
            power = 24

        slow_turn_pulse(bot, direction, power=power)

    b6_motor_stop(bot)
    event(f"SLOW FACE timeout: {target.upper()}", "e")
    return False


def straight_forward_to_target(bot, target):
    """
    Face target once, verify angle, then drive forward only.
    No turning while driving.
    """
    global stop_requested

    m = marker_copy(target)
    if not m or not m.get("locked"):
        event(f"{target.upper()} not locked", "e")
        return False

    event(f"STRAIGHT: face {target.upper()} once, then forward only", "i")

    if not straight_face_once_verified(bot, target, mode="forward", tolerance=6, attempts=1):
        return False

    event(f"STRAIGHT: forward to {target.upper()} until {STRAIGHT_TARGET_STOP_RAW}mm", "i")

    t0 = time.time()
    lost_count = 0
    start_dist = marker_dist(target) or 9999
    last_dist = start_dist
    same_count = 0

    while time.time() - t0 < STRAIGHT_TIMEOUT_S:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        dist = marker_dist(target)
        gap = marker_gap(target)

        if dist is None:
            b6_motor_stop(bot)
            lost_count += 1
            event(f"{target.upper()} lost while driving straight", "w")
            time.sleep(0.15)

            if lost_count >= STRAIGHT_LOST_LIMIT:
                event(f"{target.upper()} lost too long", "e")
                return False
            continue

        lost_count = 0
        moved = max(0, start_dist - dist)

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = f"STRAIGHT forward {target.upper()}: dist={dist:.0f} gap={(gap if gap is not None else -999):.0f} moved={moved:.0f}"

        # Stop either by raw distance or close gap
        if gap is not None and gap <= 35:
            b6_motor_stop(bot)
            event(f"Reached {target.upper()} by gap={gap:.0f}mm dist={dist:.0f}mm", "k")
            return True

        if dist <= STRAIGHT_TARGET_STOP_RAW:
            b6_motor_stop(bot)
            event(f"Reached {target.upper()} by dist={dist:.0f}mm", "k")
            return True

        if abs(dist - last_dist) < 4:
            same_count += 1
        else:
            same_count = 0
        last_dist = dist

        fine = dist <= B6_SLOW_APPROACH_RAW

        if same_count >= 8:
            event(f"{target.upper()} progress weak: stronger forward pulse", "w")
            bot.set_motor(-26, -26, -26, -26)
            time.sleep(0.16)
            b6_motor_stop(bot)
            same_count = 0
        else:
            b6_forward(bot, fine=fine)
            time.sleep(STRAIGHT_FORWARD_PULSE_S)
            b6_motor_stop(bot)

        time.sleep(STRAIGHT_SETTLE_S)

    b6_motor_stop(bot)
    event(f"Timeout forward to {target.upper()}", "e")
    return False



def straight_backward_home(bot):
    """
    Face HOME once in backward mode, then drive backward only.
    No turning while backing up.
    """
    global stop_requested

    m = marker_copy("home")
    if not m or not m.get("locked"):
        event("HOME not locked", "e")
        return False

    event("STRAIGHT: face HOME once for backward return", "i")

    if not straight_face_once_verified(bot, "home", mode="backward", tolerance=8, attempts=1):
        return False

    event(f"STRAIGHT: backward HOME until raw<={STRAIGHT_HOME_STOP_RAW} or gap<=20mm", "i")

    t0 = time.time()
    lost_count = 0
    start_dist = marker_dist("home") or 9999
    last_dist = start_dist
    same_count = 0
    blind_reverse_budget = 8

    while time.time() - t0 < STRAIGHT_TIMEOUT_S:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        dist = marker_dist("home")
        gap = marker_gap("home")

        if dist is None:
            lost_count += 1
            event("HOME briefly lost while backing up", "w")

            if blind_reverse_budget > 0:
                bot.set_motor(20, 20, 20, 20)
                time.sleep(0.12)
                b6_motor_stop(bot)
                time.sleep(0.05)
                blind_reverse_budget -= 1
                continue

            b6_motor_stop(bot)
            time.sleep(0.12)
            if lost_count >= 14:
                event("HOME lost too long during reverse", "e")
                return False
            continue

        lost_count = 0
        moved = abs(start_dist - dist)

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = f"STRAIGHT backward HOME: dist={dist:.0f} gap={(gap if gap is not None else -999):.0f} moved={moved:.0f}"

        if gap is not None and gap <= 20:
            b6_motor_stop(bot)
            event(f"Reached HOME by gap={gap:.0f}mm dist={dist:.0f}mm", "k")
            return True

        if dist <= STRAIGHT_HOME_STOP_RAW:
            b6_motor_stop(bot)
            event(f"Reached HOME by dist={dist:.0f}mm", "k")
            return True

        if abs(dist - last_dist) < 4:
            same_count += 1
        else:
            same_count = 0

        last_dist = dist

        fine = False
        if gap is not None:
            fine = gap <= 140
        else:
            fine = dist <= B6_SLOW_APPROACH_RAW

        if same_count >= 6:
            event("HOME reverse progress weak: stronger backward pulse", "w")
            bot.set_motor(28, 28, 28, 28)
            time.sleep(0.18)
            b6_motor_stop(bot)
            same_count = 0
        else:
            b6_backward(bot, fine=fine)
            time.sleep(STRAIGHT_BACKWARD_PULSE_S)
            b6_motor_stop(bot)

        time.sleep(STRAIGHT_SETTLE_S)

    b6_motor_stop(bot)
    event("Timeout backward HOME", "e")
    return False


def drive_to_marker(bot, target, timeout=MISSION_TIMEOUT_S):
    """
    Override:
    M1/M2 = face once then forward only.
    HOME = backward only, no turn.
    """
    if target == "home":
        return straight_backward_home(bot)
    return straight_forward_to_target(bot, target)


def start_mission(target):
    global mission_active, stop_requested

    if mission_active:
        return False, "Mission already running"

    if target not in ["m1", "m2", "home"]:
        return False, "Bad target"

    if not marker_copy(target) or not marker_copy(target).get("locked"):
        return False, f"{target.upper()} marker not locked"

    if target != "home":
        if not marker_copy("home") or not marker_copy("home").get("locked"):
            return False, "HOME marker not locked"

    stop_requested = False
    mission_active = True

    threading.Thread(target=mission_worker, args=(target,), daemon=True).start()
    return True, f"Started {target.upper()}"


def mission_worker(target):
    global mission_active, auto_cooldown_until

    bot = None

    with lock:
        MISSION.update({
            "state": "RUNNING",
            "target": target.upper(),
            "message": f"STRAIGHT mission to {target.upper()}",
            "last_error": "",
            "started_at": time.time(),
            "finished_at": 0,
            "moved_mm": 0
        })

    try:
        if not robot_enabled:
            raise RuntimeError("ROBOT is OFF")

        bot = open_bot()

        if not drive_to_marker(bot, target):
            raise RuntimeError(f"Could not reach {target.upper()}")

        if target != "home":
            spray_step(bot)

            if not straight_backward_home(bot):
                raise RuntimeError("Could not reach HOME")

        force_safe_outputs(bot)

        with lock:
            MISSION.update({
                "state": "DONE",
                "message": "Mission complete",
                "finished_at": time.time()
            })

        event("Mission complete", "k")

    except Exception as e:
        force_safe_outputs(bot)
        with lock:
            MISSION.update({
                "state": "ERROR",
                "message": str(e),
                "last_error": str(e),
                "finished_at": time.time()
            })
        event("Mission error: " + str(e), "e")

    finally:
        force_safe_outputs(bot)
        mission_active = False
        auto_cooldown_until = time.time() + AUTO_COOLDOWN_S


def reset_auto_served_if_clear():
    with lock:
        a1 = bool(ESP.get("alert1"))
        a2 = bool(ESP.get("alert2"))

    if not a1:
        AUTO_SERVED["m1"] = False
    if not a2:
        AUTO_SERVED["m2"] = False


def choose_alert_target():
    with lock:
        a1 = bool(ESP.get("alert1"))
        a2 = bool(ESP.get("alert2"))

    choices = []

    if a1 and not AUTO_SERVED["m1"]:
        d = marker_dist("m1")
        if d is not None:
            choices.append(("m1", d))

    if a2 and not AUTO_SERVED["m2"]:
        d = marker_dist("m2")
        if d is not None:
            choices.append(("m2", d))

    if not choices:
        return None

    choices.sort(key=lambda x: x[1])
    return choices[0][0]


def auto_loop():
    global auto_cooldown_until

    while True:
        time.sleep(AUTO_POLL_S)

        try:
            reset_auto_served_if_clear()

            with lock:
                ready = bool(robot_enabled) and bool(auto_enabled) and not bool(mission_active)
                cooldown = float(auto_cooldown_until)

            if not ready:
                continue

            if time.time() < cooldown:
                continue

            target = choose_alert_target()
            if not target:
                continue

            AUTO_SERVED[target] = True
            event(f"AUTO ALERT -> {target.upper()} straight-only", "w")

            ok, msg = start_mission(target)
            if not ok:
                AUTO_SERVED[target] = False
                event("AUTO start failed: " + msg, "e")
                auto_cooldown_until = time.time() + 2.0

        except Exception as e:
            event("auto_loop error: " + str(e), "e")
            time.sleep(0.5)




# ============================================================
# FPMS B8 SAFE REBUILD OVERRIDES - 2026-05-06
# Built on Golden B6. No route-following. ESP32 controls spray.
# ============================================================

# Behavior:
# face once -> fast straight -> crawl -> stop -> spray -> face HOME -> fast reverse -> crawl/park -> wait

# Movement tuning. Negative is forward on this chassis; positive is backward.
B8_FORWARD_FAST_POWER = -36
B8_FORWARD_CRAWL_POWER = -18
B8_BACKWARD_FAST_POWER = 30
B8_BACKWARD_CRAWL_POWER = 15

# Stop/crawl tuning in mm.
B8_TARGET_STOP_RAW = 280
B8_TARGET_GAP_STOP = 80
B8_TARGET_CRAWL_RAW = 560

B8_HOME_STOP_RAW = 320
B8_HOME_GAP_STOP = 70
B8_HOME_CRAWL_RAW = 650

# Turn tuning.
B8_FACE_TOL_FWD = 7
B8_FACE_TOL_HOME = 9
B8_FACE_STABLE_READS = 2
B8_TURN_MAX_POWER = 42
B8_TURN_MIN_POWER = 24
B8_TURN_MAX_PULSE_MS = 80
B8_TURN_SETTLE_S = 0.075
B8_TURN_KICK_S = 0.012

# Pulse drive tuning.
STRAIGHT_FORWARD_PULSE_S = 0.20
STRAIGHT_BACKWARD_PULSE_S = 0.20
STRAIGHT_SETTLE_S = 0.035
STRAIGHT_TARGET_STOP_RAW = B8_TARGET_STOP_RAW
STRAIGHT_HOME_STOP_RAW = B8_HOME_STOP_RAW
B6_SLOW_APPROACH_RAW = B8_TARGET_CRAWL_RAW
STRAIGHT_LOST_LIMIT = 10
STRAIGHT_TIMEOUT_S = 55

# ESP32 spray gateway.
ESP32_SPRAY_MS = 3000
ESP32_SPRAY_ENABLED = True
ESP_SER = None
ESP_WRITE_LOCK = threading.Lock()

# Auto repeat: allow repeated missions while alert remains active.
AUTO_REPEAT_ALERTS = True
B8_AUTO_COOLDOWN_S = 8.0

B8_STICKY_MARKERS = {}


def marker_copy(name):
    """
    Sticky marker read: keeps last good locked marker briefly so one weak LiDAR frame
    does not kill a mission while driving.
    """
    now = time.time()
    with lock:
        m = S.get("markers", {}).get(name)
        d = dict(m) if m else None

    if d and d.get("locked"):
        B8_STICKY_MARKERS[name] = (now, dict(d))
        return d

    old = B8_STICKY_MARKERS.get(name)
    if old:
        t, old_d = old
        if now - t <= 1.25:
            d2 = dict(old_d)
            d2["locked"] = True
            d2["sticky"] = True
            return d2

    return d


def marker_gap(name):
    """
    Corrects the old bug: marker uses front_gap, not gap.
    """
    m = marker_copy(name)
    if not m or not m.get("locked"):
        return None

    for key in ("front_gap", "gap"):
        try:
            if key in m and m.get(key) is not None:
                return float(m.get(key))
        except Exception:
            pass

    try:
        return float(m.get("dist", 999999)) - float(LIDAR_TO_FRONT)
    except Exception:
        return None


def b6_forward(bot, fine=False):
    p = B8_FORWARD_CRAWL_POWER if fine else B8_FORWARD_FAST_POWER
    bot.set_motor(p, p, p, p)


def b6_backward(bot, fine=False):
    p = B8_BACKWARD_CRAWL_POWER if fine else B8_BACKWARD_FAST_POWER
    bot.set_motor(p, p, p, p)


def slow_turn_pulse(bot, direction, *args, **kwargs):
    """
    Smooth tank-turn pulse. B8 uses LiDAR marker angle as truth.
    Strong enough to move, short enough to avoid overshoot.
    """
    sign = int(globals().get("STRAIGHT_TURN_SIGN", 1))

    try:
        raw_dir = float(direction)
    except Exception:
        raw_dir = 0.0

    d = raw_dir * sign

    if d == 0:
        b6_motor_stop(bot)
        return

    mag = abs(raw_dir)

    override_power = kwargs.get("power", kwargs.get("turn_power", None))
    override_ms = kwargs.get("pulse_ms", kwargs.get("ms", kwargs.get("duration_ms", None)))

    if len(args) >= 1 and override_power is None:
        override_power = args[0]
    if len(args) >= 2 and override_ms is None:
        override_ms = args[1]

    if override_power is not None:
        power = int(abs(override_power))
    elif mag >= 35:
        power = 40
    elif mag >= 20:
        power = 36
    elif mag >= 10:
        power = 32
    elif mag >= 5:
        power = 28
    else:
        power = 24

    if override_ms is not None:
        pulse_ms = int(override_ms)
    elif mag >= 35:
        pulse_ms = 78
    elif mag >= 20:
        pulse_ms = 64
    elif mag >= 10:
        pulse_ms = 50
    elif mag >= 5:
        pulse_ms = 38
    else:
        pulse_ms = 28

    power = max(B8_TURN_MIN_POWER, min(B8_TURN_MAX_POWER, power))
    pulse_ms = max(25, min(B8_TURN_MAX_PULSE_MS, pulse_ms))

    left = -power if d > 0 else power
    right = power if d > 0 else -power

    # Tiny anti-stiction kick, not a jerk.
    kick = min(B8_TURN_MAX_POWER + 2, power + 4)

    if d > 0:
        bot.set_motor(-kick, -kick, kick, kick)
    else:
        bot.set_motor(kick, kick, -kick, -kick)

    time.sleep(B8_TURN_KICK_S)

    bot.set_motor(left, left, right, right)
    time.sleep(pulse_ms / 1000.0)

    b6_motor_stop(bot)
    time.sleep(B8_TURN_SETTLE_S)


def straight_face_once_verified(bot, target, mode="forward", tolerance=None, attempts=1):
    """
    Face once, smoothly. Smaller final pulses and only 2 stable reads before driving.
    """
    tol = B8_FACE_TOL_HOME if mode == "backward" else B8_FACE_TOL_FWD

    if tolerance is not None:
        tol = min(float(tolerance), float(tol))

    event(f"B8 FACE {target.upper()} mode={mode} tol={tol}°", "i")

    stable = 0
    last_err = None
    lost = 0
    t0 = time.time()

    while time.time() - t0 < 14:
        err, m = face_error_for_marker(target, mode)

        if err is None:
            b6_motor_stop(bot)
            lost += 1
            if lost > 12:
                event(f"B8 FACE failed: {target.upper()} marker weak", "e")
                return False
            time.sleep(0.12)
            continue

        lost = 0
        ae = abs(err)

        with lock:
            MISSION["message"] = f"B8 FACE {target.upper()}: err={err:.1f}°"

        if ae <= tol:
            b6_motor_stop(bot)
            stable += 1
            if stable >= B8_FACE_STABLE_READS:
                event(f"B8 FACE {target.upper()} OK err={err:.1f}°", "k")
                time.sleep(0.18)
                return True
            time.sleep(0.10)
            continue

        stable = 0

        if last_err is not None and ((err > 0 > last_err) or (err < 0 < last_err)) and ae <= 18:
            b6_motor_stop(bot)
            time.sleep(0.16)

        last_err = err

        direction = 1 if err > 0 else -1

        if ae > 30:
            p = 38
        elif ae > 16:
            p = 34
        elif ae > 8:
            p = 29
        else:
            p = 25

        slow_turn_pulse(bot, direction, power=p)

    b6_motor_stop(bot)
    event(f"B8 FACE timeout: {target.upper()}", "e")
    return False


def straight_forward_to_target(bot, target):
    global stop_requested

    m = marker_copy(target)

    if not m or not m.get("locked"):
        event(f"{target.upper()} not locked", "e")
        return False

    try:
        plan_route(target)  # UI display only
    except Exception as e:
        event(f"Route display skipped: {e}", "w")

    event(f"B8: face {target.upper()} once, then fast/crawl forward", "i")

    if not straight_face_once_verified(bot, target, mode="forward", tolerance=B8_FACE_TOL_FWD):
        return False

    t0 = time.time()
    lost_count = 0
    start_dist = marker_dist(target) or 9999

    while time.time() - t0 < STRAIGHT_TIMEOUT_S:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        dist = marker_dist(target)
        gap = marker_gap(target)

        if dist is None:
            b6_motor_stop(bot)
            lost_count += 1
            event(f"{target.upper()} briefly lost while driving", "w")

            if lost_count >= STRAIGHT_LOST_LIMIT:
                event(f"{target.upper()} lost too long", "e")
                return False

            time.sleep(0.08)
            continue

        lost_count = 0
        moved = max(0, start_dist - dist)

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = f"B8 forward {target.upper()}: dist={dist:.0f} gap={(gap if gap is not None else -999):.0f} moved={moved:.0f}"

        if gap is not None and gap <= B8_TARGET_GAP_STOP:
            b6_motor_stop(bot)
            event(f"B8 reached {target.upper()} by front_gap={gap:.0f}mm dist={dist:.0f}mm", "k")
            return True

        if dist <= B8_TARGET_STOP_RAW:
            b6_motor_stop(bot)
            event(f"B8 reached {target.upper()} by dist={dist:.0f}mm", "k")
            return True

        fine = (dist <= B8_TARGET_CRAWL_RAW) or (gap is not None and gap <= 260)

        b6_forward(bot, fine=fine)
        time.sleep(STRAIGHT_FORWARD_PULSE_S if not fine else 0.14)
        b6_motor_stop(bot)
        time.sleep(STRAIGHT_SETTLE_S)

    b6_motor_stop(bot)
    event(f"B8 timeout forward to {target.upper()}", "e")
    return False


def straight_backward_home(bot):
    global stop_requested

    m = marker_copy("home")

    if not m or not m.get("locked"):
        event("HOME not locked", "e")
        return False

    try:
        plan_route("home")  # UI display only
    except Exception as e:
        event(f"Home route display skipped: {e}", "w")

    event("B8: face HOME once, then fast/crawl backward", "i")

    if not straight_face_once_verified(bot, "home", mode="backward", tolerance=B8_FACE_TOL_HOME):
        return False

    t0 = time.time()
    lost_count = 0
    start_dist = marker_dist("home") or 9999
    blind_budget = 4

    while time.time() - t0 < STRAIGHT_TIMEOUT_S:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        dist = marker_dist("home")
        gap = marker_gap("home")

        if dist is None:
            lost_count += 1
            event("HOME briefly lost while backing", "w")

            if blind_budget > 0:
                b6_backward(bot, fine=True)
                time.sleep(0.08)
                b6_motor_stop(bot)
                blind_budget -= 1
                continue

            if lost_count >= STRAIGHT_LOST_LIMIT:
                event("HOME lost too long", "e")
                return False

            time.sleep(0.08)
            continue

        lost_count = 0
        moved = abs(start_dist - dist)

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = f"B8 backward HOME: dist={dist:.0f} gap={(gap if gap is not None else -999):.0f} moved={moved:.0f}"

        if dist <= B8_HOME_STOP_RAW:
            b6_motor_stop(bot)
            event(f"B8 HOME parked by dist={dist:.0f}mm", "k")
            return True

        if gap is not None and gap <= B8_HOME_GAP_STOP and dist <= B8_HOME_CRAWL_RAW:
            b6_motor_stop(bot)
            event(f"B8 HOME parked by gap={gap:.0f}mm dist={dist:.0f}mm", "k")
            return True

        fine = dist <= B8_HOME_CRAWL_RAW or (gap is not None and gap <= 260)

        b6_backward(bot, fine=fine)
        time.sleep(STRAIGHT_BACKWARD_PULSE_S if not fine else 0.14)
        b6_motor_stop(bot)
        time.sleep(STRAIGHT_SETTLE_S)

    b6_motor_stop(bot)
    event("B8 timeout backward HOME", "e")
    return False


def drive_to_marker(bot, target, timeout=MISSION_TIMEOUT_S):
    if target == "home":
        return straight_backward_home(bot)
    return straight_forward_to_target(bot, target)


def esp_loop():
    """
    Single owner of ESP32 serial. Mission writes spray command through this same port.
    """
    global ESP_SER

    while True:
        ser = None
        try:
            ser = serial.Serial(ESP_PORT, ESP_BAUD, timeout=0.25)

            with ESP_WRITE_LOCK:
                ESP_SER = ser

            time.sleep(1.6)
            event("ESP receiver opened for read/write", "k")

            while True:
                line = ser.readline().decode("utf-8", "ignore").strip()
                if line:
                    esp_line(line)

        except Exception as e:
            with ESP_WRITE_LOCK:
                if ESP_SER is ser:
                    ESP_SER = None

            try:
                if ser:
                    ser.close()
            except Exception:
                pass

            with lock:
                ESP["last_line"] = "ESP_ERROR: " + str(e)
                ESP["last_update"] = time.time()
                ESP["n1_online"] = False
                ESP["n2_online"] = False

            time.sleep(1.0)


def fpms_spray(ms=None):
    """
    Send spray to ESP32 using the already-open ESP serial port.
    """
    if ms is None:
        ms = ESP32_SPRAY_MS

    if not ESP32_SPRAY_ENABLED:
        event("B8 spray disabled", "w")
        return False

    deadline = time.time() + 3.0

    while time.time() < deadline:
        with ESP_WRITE_LOCK:
            ser = ESP_SER
            if ser is not None:
                try:
                    ser.write(f"SPRAY:{int(ms)}\n".encode("utf-8"))
                    ser.flush()
                    event(f"B8 sent ESP32 SPRAY:{int(ms)}", "w")
                    return True
                except Exception as e:
                    event("B8 ESP32 spray write failed: " + str(e), "e")
                    return False

        time.sleep(0.10)

    event("B8 ESP32 serial not ready; no spray", "e")
    return False


def fpms_pump_off():
    with ESP_WRITE_LOCK:
        ser = ESP_SER
        if ser is not None:
            try:
                ser.write(b"PUMP_OFF\n")
                ser.flush()
                return True
            except Exception:
                return False

    return False


def force_safe_outputs(bot):
    """
    Stop motors and force ESP32 pump off.
    """
    try:
        if bot:
            bot.set_motor(0, 0, 0, 0)
    except Exception:
        pass

    try:
        fpms_pump_off()
    except Exception:
        pass


def spray_step(bot):
    """
    B8 real spray: ESP32 GPIO26 -> MOSFET -> pump.
    """
    b6_motor_stop(bot)
    event("B8 spray: stopped motors, sending ESP32 command", "w")

    ok = fpms_spray(ESP32_SPRAY_MS)

    if ok:
        time.sleep((ESP32_SPRAY_MS / 1000.0) + 0.35)
        event("B8 spray complete", "k")
        return True

    event("B8 spray skipped/failed; continuing safely", "e")
    return True


def start_mission(target):
    global mission_active, stop_requested

    if mission_active:
        return False, "Mission already running"

    if target not in ["m1", "m2", "home"]:
        return False, "Bad target"

    if not marker_copy(target) or not marker_copy(target).get("locked"):
        return False, f"{target.upper()} marker not locked"

    if target != "home" and (not marker_copy("home") or not marker_copy("home").get("locked")):
        return False, "HOME marker not locked"

    stop_requested = False
    mission_active = True

    with lock:
        MISSION["state"] = "STARTING"
        MISSION["target"] = target.upper()
        MISSION["message"] = "Starting B8 mission"

    threading.Thread(target=mission_worker, args=(target,), daemon=True).start()
    return True, f"Started {target.upper()}"


def mission_worker(target):
    global mission_active, auto_cooldown_until

    bot = None

    with lock:
        MISSION.update({
            "state": "RUNNING",
            "target": target.upper(),
            "message": f"B8 mission to {target.upper()}",
            "last_error": "",
            "started_at": time.time(),
            "finished_at": 0,
            "moved_mm": 0,
        })

    try:
        if not robot_enabled:
            raise RuntimeError("ROBOT is OFF")

        bot = open_bot()

        try:
            plan_route(target)
        except Exception:
            pass

        if not drive_to_marker(bot, target):
            raise RuntimeError(f"Could not reach {target.upper()}")

        if target != "home":
            if not spray_step(bot):
                event("B8 spray returned false; still attempting home", "e")

            try:
                plan_route("home")
            except Exception:
                pass

            if not straight_backward_home(bot):
                raise RuntimeError("Could not reach HOME")

        force_safe_outputs(bot)

        with lock:
            MISSION.update({
                "state": "DONE",
                "message": "B8 mission complete",
                "finished_at": time.time()
            })

        event("B8 mission complete", "k")

    except Exception as e:
        force_safe_outputs(bot)

        with lock:
            MISSION.update({
                "state": "ERROR",
                "message": str(e),
                "last_error": str(e),
                "finished_at": time.time()
            })

        event("B8 mission error: " + str(e), "e")

    finally:
        force_safe_outputs(bot)
        mission_active = False
        auto_cooldown_until = time.time() + B8_AUTO_COOLDOWN_S


def choose_alert_target():
    """
    Allow repeat missions while alert remains active; cooldown prevents instant repeats.
    """
    with lock:
        a1 = bool(ESP.get("alert1"))
        a2 = bool(ESP.get("alert2"))

    choices = []

    if a1:
        d = marker_dist("m1")
        if d is not None:
            choices.append(("m1", d))

    if a2:
        d = marker_dist("m2")
        if d is not None:
            choices.append(("m2", d))

    if not choices:
        return None

    choices.sort(key=lambda x: x[1])
    return choices[0][0]


def auto_loop():
    global auto_cooldown_until

    while True:
        time.sleep(AUTO_POLL_S)

        try:
            with lock:
                ready = bool(robot_enabled) and bool(auto_enabled) and not bool(mission_active)
                cooldown = float(auto_cooldown_until)

            if not ready:
                continue

            if time.time() < cooldown:
                continue

            target = choose_alert_target()

            if not target:
                continue

            event(f"B8 AUTO ALERT -> {target.upper()}", "w")

            ok, msg = start_mission(target)

            if not ok:
                event("B8 AUTO start failed: " + msg, "e")
                auto_cooldown_until = time.time() + 2.0

        except Exception as e:
            event("B8 auto_loop error: " + str(e), "e")
            time.sleep(0.5)

# ============================================================
# END FPMS B8 SAFE REBUILD OVERRIDES
# ============================================================




# ============================================================
# FPMS B8.1 LIDAR-ODOM HOME FIX - 2026-05-06
# This overrides only the unstable parts from B8.
# ============================================================

# HOME return tuning:
# Previous B8 crawled too early at 650mm with power 15.
# That is why it stopped around 459mm and timed out.
B81_HOME_FAST_POWER = 35
B81_HOME_CRAWL_POWER = 23
B81_HOME_RECOVERY_POWER = 39

B81_HOME_CRAWL_RAW = 360       # only crawl once actually close
B81_HOME_STOP_RAW = 300        # park distance
B81_HOME_GAP_STOP = 120        # backup stop if gap is useful

B81_HOME_FAST_PULSE_S = 0.22
B81_HOME_CRAWL_PULSE_S = 0.13
B81_HOME_RECOVERY_PULSE_S = 0.24
B81_HOME_SETTLE_S = 0.040

B81_HOME_TIMEOUT_S = 45
B81_HOME_PROGRESS_MM = 8
B81_HOME_STUCK_SEC = 1.9

# If HOME angle drifts this much while backing, stop and re-face.
B81_HOME_REFACE_ERR_DEG = 17
B81_HOME_REFACE_COOLDOWN_S = 1.4

# Forward target accuracy tuning.
B81_TARGET_FAST_POWER = -36
B81_TARGET_CRAWL_POWER = -19
B81_TARGET_STOP_GAP = 70
B81_TARGET_STOP_RAW = 270
B81_TARGET_CRAWL_RAW = 520

# AUTO should not keep repeating the same alert forever.
B81_AUTO_SERVED = {"m1": False, "m2": False}


def b81_median(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    vals = sorted(vals)
    return vals[len(vals)//2]


def b81_marker_filtered(name, samples=5, delay_s=0.025):
    """
    Small LiDAR filter:
    reads the locked marker a few times and returns median distance/gap/angle.
    This reduces one-frame LiDAR jitter.
    """
    import time

    ds, gs, angs = [], [], []
    last = None

    for _ in range(samples):
        m = marker_copy(name)
        if m and m.get("locked"):
            last = m
            try:
                ds.append(float(m.get("dist")))
            except Exception:
                pass
            try:
                gs.append(float(m.get("front_gap")))
            except Exception:
                pass
            try:
                angs.append(float(m.get("angle")))
            except Exception:
                pass
        time.sleep(delay_s)

    if not last:
        return None

    out = dict(last)
    md = b81_median(ds)
    mg = b81_median(gs)
    ma = b81_median(angs)

    if md is not None:
        out["dist"] = md
    if mg is not None:
        out["front_gap"] = mg
        out["gap"] = mg
    if ma is not None:
        out["angle"] = ma

    return out


def b81_home_error_deg():
    """
    HOME backing wants marker at 180 degrees.
    """
    try:
        err, m = face_error_for_marker("home", "backward")
        return err
    except Exception:
        return None


def b81_backward_power(bot, power, pulse_s):
    """
    Positive power is backward on the current working chassis.
    """
    bot.set_motor(power, power, power, power)
    time.sleep(pulse_s)
    b6_motor_stop(bot)


def b81_forward_power(bot, power, pulse_s):
    """
    Negative power is forward on the current working chassis.
    """
    bot.set_motor(power, power, power, power)
    time.sleep(pulse_s)
    b6_motor_stop(bot)


def straight_forward_to_target(bot, target):
    """
    B8.1 LiDAR-filtered forward target approach.
    Face once, drive fast, crawl near target, stop, spray later.
    """
    import time

    global stop_requested

    m = b81_marker_filtered(target)
    if not m or not m.get("locked"):
        event(f"B8.1 {target.upper()} not locked", "e")
        return False

    try:
        plan_route(target)  # visual route only
    except Exception as e:
        event(f"B8.1 route display skipped: {e}", "w")

    event(f"B8.1: face {target.upper()} once, then LiDAR-odom forward", "i")

    if not straight_face_once_verified(bot, target, mode="forward", tolerance=6, attempts=1):
        return False

    start_dist = float(b81_marker_filtered(target).get("dist", 9999))
    best_dist = start_dist
    last_progress_t = time.monotonic()
    t0 = time.monotonic()
    lost = 0

    while time.monotonic() - t0 < STRAIGHT_TIMEOUT_S:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        m = b81_marker_filtered(target, samples=3, delay_s=0.015)
        if not m or not m.get("locked"):
            lost += 1
            b6_motor_stop(bot)
            if lost >= STRAIGHT_LOST_LIMIT:
                event(f"B8.1 {target.upper()} lost too long", "e")
                return False
            time.sleep(0.06)
            continue

        lost = 0

        dist = float(m.get("dist", 9999))
        gap = float(m.get("front_gap", dist))
        moved = max(0, start_dist - dist)

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = f"B8.1 forward {target.upper()}: dist={dist:.0f} gap={gap:.0f} moved={moved:.0f}"

        if gap <= B81_TARGET_STOP_GAP or dist <= B81_TARGET_STOP_RAW:
            b6_motor_stop(bot)
            event(f"B8.1 reached {target.upper()}: dist={dist:.0f} gap={gap:.0f}", "k")
            return True

        if dist < best_dist - B81_HOME_PROGRESS_MM:
            best_dist = dist
            last_progress_t = time.monotonic()

        fine = dist <= B81_TARGET_CRAWL_RAW or gap <= 230

        if fine:
            b81_forward_power(bot, B81_TARGET_CRAWL_POWER, 0.13)
        else:
            b81_forward_power(bot, B81_TARGET_FAST_POWER, 0.20)

        time.sleep(B81_HOME_SETTLE_S)

    b6_motor_stop(bot)
    event(f"B8.1 timeout forward to {target.upper()}", "e")
    return False


def straight_backward_home(bot):
    """
    B8.1 HOME return:
    LiDAR-filtered landmark odometry + angle re-face + progress watchdog.
    This is the real fix for the 459mm timeout.
    """
    import time

    global stop_requested

    m = b81_marker_filtered("home")
    if not m or not m.get("locked"):
        event("B8.1 HOME not locked", "e")
        return False

    try:
        plan_route("home")  # visual route only
    except Exception as e:
        event(f"B8.1 home route display skipped: {e}", "w")

    event("B8.1: face HOME once, then LiDAR-odom backward", "i")

    if not straight_face_once_verified(bot, "home", mode="backward", tolerance=8, attempts=1):
        return False

    start = b81_marker_filtered("home")
    start_dist = float(start.get("dist", 9999)) if start else 9999
    best_dist = start_dist
    last_progress_t = time.monotonic()
    last_reface_t = 0.0
    t0 = time.monotonic()
    lost = 0
    recovery_count = 0

    event(f"B8.1 HOME start dist={start_dist:.0f}mm", "i")

    while time.monotonic() - t0 < B81_HOME_TIMEOUT_S:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        m = b81_marker_filtered("home", samples=3, delay_s=0.015)

        if not m or not m.get("locked"):
            lost += 1
            event("B8.1 HOME briefly lost while backing", "w")

            if lost <= 6:
                b81_backward_power(bot, B81_HOME_CRAWL_POWER, 0.08)
                time.sleep(B81_HOME_SETTLE_S)
                continue

            b6_motor_stop(bot)
            if lost >= 14:
                event("B8.1 HOME lost too long", "e")
                return False
            time.sleep(0.08)
            continue

        lost = 0
        dist = float(m.get("dist", 9999))
        gap = float(m.get("front_gap", dist))
        moved = abs(start_dist - dist)

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = f"B8.1 HOME: dist={dist:.0f} gap={gap:.0f} best={best_dist:.0f} moved={moved:.0f}"

        # Stop rules.
        if dist <= B81_HOME_STOP_RAW:
            b6_motor_stop(bot)
            event(f"B8.1 HOME parked by dist={dist:.0f}mm gap={gap:.0f}", "k")
            return True

        if gap <= B81_HOME_GAP_STOP and dist <= (B81_HOME_CRAWL_RAW + 80):
            b6_motor_stop(bot)
            event(f"B8.1 HOME parked by gap={gap:.0f}mm dist={dist:.0f}", "k")
            return True

        # LiDAR angular correction. No continuous steering; stop, re-face, continue.
        err = b81_home_error_deg()
        now = time.monotonic()
        if err is not None and abs(err) >= B81_HOME_REFACE_ERR_DEG and now - last_reface_t >= B81_HOME_REFACE_COOLDOWN_S:
            b6_motor_stop(bot)
            event(f"B8.1 HOME angle drift {err:.1f}°, re-facing", "w")
            straight_face_once_verified(bot, "home", mode="backward", tolerance=8, attempts=1)
            last_reface_t = time.monotonic()
            continue

        # Progress watchdog.
        if dist < best_dist - B81_HOME_PROGRESS_MM:
            best_dist = dist
            last_progress_t = time.monotonic()
            recovery_count = 0

        stuck = (time.monotonic() - last_progress_t) >= B81_HOME_STUCK_SEC

        if stuck:
            recovery_count += 1
            event(f"B8.1 HOME progress weak: recovery pulse {recovery_count}", "w")
            b81_backward_power(bot, B81_HOME_RECOVERY_POWER, B81_HOME_RECOVERY_PULSE_S)
            last_progress_t = time.monotonic()
        else:
            if dist <= B81_HOME_CRAWL_RAW or gap <= 210:
                b81_backward_power(bot, B81_HOME_CRAWL_POWER, B81_HOME_CRAWL_PULSE_S)
            else:
                b81_backward_power(bot, B81_HOME_FAST_POWER, B81_HOME_FAST_PULSE_S)

        time.sleep(B81_HOME_SETTLE_S)

    b6_motor_stop(bot)
    event("B8.1 timeout backward HOME", "e")
    return False


def drive_to_marker(bot, target, timeout=MISSION_TIMEOUT_S):
    if str(target).lower() == "home":
        return straight_backward_home(bot)
    return straight_forward_to_target(bot, str(target).lower())


def reset_auto_served_if_clear():
    """
    Do not repeat the same active alert forever.
    Reset only after the alert clears.
    """
    with lock:
        a1 = bool(ESP.get("alert1"))
        a2 = bool(ESP.get("alert2"))

    if not a1:
        B81_AUTO_SERVED["m1"] = False
    if not a2:
        B81_AUTO_SERVED["m2"] = False


def choose_alert_target():
    """
    Pick the closest active unserved alert.
    """
    with lock:
        a1 = bool(ESP.get("alert1"))
        a2 = bool(ESP.get("alert2"))

    choices = []

    if a1 and not B81_AUTO_SERVED["m1"]:
        d = marker_dist("m1")
        if d is not None:
            choices.append(("m1", d))

    if a2 and not B81_AUTO_SERVED["m2"]:
        d = marker_dist("m2")
        if d is not None:
            choices.append(("m2", d))

    if not choices:
        return None

    choices.sort(key=lambda x: x[1])
    return choices[0][0]


def auto_loop():
    """
    B8.1 AUTO:
    - runs once per active alert
    - waits for alert clear before repeating
    - manual buttons still work after DONE/ERROR
    """
    global auto_cooldown_until

    while True:
        time.sleep(AUTO_POLL_S)

        try:
            reset_auto_served_if_clear()

            with lock:
                ready = bool(robot_enabled) and bool(auto_enabled) and not bool(mission_active)
                cooldown = float(auto_cooldown_until)

            if not ready:
                continue

            if time.time() < cooldown:
                continue

            target = choose_alert_target()
            if not target:
                continue

            B81_AUTO_SERVED[target] = True
            event(f"B8.1 AUTO ALERT -> {target.upper()}", "w")

            ok, msg = start_mission(target)
            if not ok:
                B81_AUTO_SERVED[target] = False
                event("B8.1 AUTO start failed: " + msg, "e")
                auto_cooldown_until = time.time() + 2.0

        except Exception as e:
            event("B8.1 auto_loop error: " + str(e), "e")
            time.sleep(0.5)

# ============================================================
# END FPMS B8.1 LIDAR-ODOM HOME FIX
# ============================================================




# ============================================================
# FPMS B8.4 LIDAR GAP MODE - 2026-05-06
# Slower mission + safer marker stop + safer HOME parking.
# Overrides B8/B8.1 values/functions.
# ============================================================

# Spray only 2 seconds.
ESP32_SPRAY_MS = 2000

# General slower motion.
B82_TARGET_FAST_POWER = -28
B82_TARGET_CRAWL_POWER = -15
B82_TARGET_FINAL_POWER = -11

B82_HOME_FAST_POWER = 26
B82_HOME_PARK_POWER = 17
B82_HOME_FINAL_POWER = 11
B82_HOME_RECOVERY_POWER = 28

# Safer target stopping.
# front_gap is distance from robot front to marker/object.
B82_TARGET_STOP_GAP = 130
B82_TARGET_STOP_RAW = 340
B82_TARGET_CRAWL_RAW = 600
B82_TARGET_FINAL_RAW = 430

# Safer HOME parking.
B82_HOME_STOP_RAW = 390
B82_HOME_GAP_STOP = 170
B82_HOME_PARK_RAW = 650
B82_HOME_FINAL_RAW = 500

# Pulse timing.
B82_FAST_PULSE_S = 0.16
B82_CRAWL_PULSE_S = 0.105
B82_FINAL_PULSE_S = 0.060
B82_SETTLE_S = 0.055

# Progress watchdog.
B82_PROGRESS_MM = 5
B82_STUCK_SEC = 2.1

# HOME re-face if angle drifts.
B82_HOME_REFACE_ERR_DEG = 14
B82_HOME_REFACE_COOLDOWN_S = 1.3

# Slightly calmer face.
B8_FACE_TOL_FWD = 6
B8_FACE_TOL_HOME = 7
B8_TURN_MAX_POWER = 38
B8_TURN_MIN_POWER = 22
B8_TURN_MAX_PULSE_MS = 68
B8_TURN_KICK_S = 0.010
B8_TURN_SETTLE_S = 0.090


def b82_drive_all(bot, power, pulse_s):
    bot.set_motor(power, power, power, power)
    time.sleep(pulse_s)
    b6_motor_stop(bot)
    time.sleep(B82_SETTLE_S)


def b82_safe_stop_target(dist, gap):
    if gap is not None and gap <= B82_TARGET_STOP_GAP:
        return True
    if dist is not None and dist <= B82_TARGET_STOP_RAW:
        return True
    return False


def b82_safe_stop_home(dist, gap):
    if dist is not None and dist <= B82_HOME_STOP_RAW:
        return True
    if gap is not None and gap <= B82_HOME_GAP_STOP and dist is not None and dist <= B82_HOME_PARK_RAW:
        return True
    return False


def straight_forward_to_target(bot, target):
    """
    B8.2 safer target approach:
    face once -> slower forward -> crawl -> tiny final creep -> stop earlier -> spray.
    """
    import time
    global stop_requested

    target = str(target).lower()

    m = b81_marker_filtered(target)
    if not m or not m.get("locked"):
        event(f"B8.2 {target.upper()} not locked", "e")
        return False

    try:
        plan_route(target)  # route display only
    except Exception as e:
        event(f"B8.2 route display skipped: {e}", "w")

    event(f"B8.2: safe slow approach to {target.upper()}", "i")

    if not straight_face_once_verified(bot, target, mode="forward", tolerance=6, attempts=1):
        return False

    start = b81_marker_filtered(target)
    start_dist = float(start.get("dist", 9999)) if start else 9999
    best_dist = start_dist
    last_progress_t = time.monotonic()
    t0 = time.monotonic()
    lost = 0

    while time.monotonic() - t0 < STRAIGHT_TIMEOUT_S:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        m = b81_marker_filtered(target, samples=3, delay_s=0.015)

        if not m or not m.get("locked"):
            lost += 1
            b6_motor_stop(bot)

            if lost >= STRAIGHT_LOST_LIMIT:
                event(f"B8.2 {target.upper()} lost too long", "e")
                return False

            time.sleep(0.07)
            continue

        lost = 0

        dist = float(m.get("dist", 9999))
        gap = float(m.get("front_gap", dist))
        moved = max(0, start_dist - dist)

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = f"B8.2 target {target.upper()}: dist={dist:.0f} gap={gap:.0f} moved={moved:.0f}"

        if b82_safe_stop_target(dist, gap):
            b6_motor_stop(bot)
            event(f"B8.2 {target.upper()} safe stop: dist={dist:.0f} gap={gap:.0f}", "k")
            return True

        if dist < best_dist - B82_PROGRESS_MM:
            best_dist = dist
            last_progress_t = time.monotonic()

        # Speed zones.
        if dist <= B82_TARGET_FINAL_RAW or gap <= 210:
            power = B82_TARGET_FINAL_POWER
            pulse = B82_FINAL_PULSE_S
            zone = "FINAL"
        elif dist <= B82_TARGET_CRAWL_RAW or gap <= 300:
            power = B82_TARGET_CRAWL_POWER
            pulse = B82_CRAWL_PULSE_S
            zone = "CRAWL"
        else:
            power = B82_TARGET_FAST_POWER
            pulse = B82_FAST_PULSE_S
            zone = "SLOW-MED"

        with lock:
            MISSION["message"] = f"B8.2 {zone} {target.upper()}: dist={dist:.0f} gap={gap:.0f}"

        b82_drive_all(bot, power, pulse)

    b6_motor_stop(bot)
    event(f"B8.2 timeout forward to {target.upper()}", "e")
    return False


def straight_backward_home(bot):
    """
    B8.2 safer HOME parking:
    face HOME once -> reverse slow-medium -> parking slow -> final tiny creep -> stop early.
    """
    import time
    global stop_requested

    m = b81_marker_filtered("home")
    if not m or not m.get("locked"):
        event("B8.2 HOME not locked", "e")
        return False

    try:
        plan_route("home")  # route display only
    except Exception as e:
        event(f"B8.2 home route display skipped: {e}", "w")

    event("B8.2: safe slow HOME parking", "i")

    if not straight_face_once_verified(bot, "home", mode="backward", tolerance=7, attempts=1):
        return False

    start = b81_marker_filtered("home")
    start_dist = float(start.get("dist", 9999)) if start else 9999
    best_dist = start_dist
    last_progress_t = time.monotonic()
    last_reface_t = 0.0
    t0 = time.monotonic()
    lost = 0
    recovery_count = 0

    event(f"B8.2 HOME start dist={start_dist:.0f}mm", "i")

    while time.monotonic() - t0 < B81_HOME_TIMEOUT_S:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        m = b81_marker_filtered("home", samples=3, delay_s=0.015)

        if not m or not m.get("locked"):
            lost += 1
            event("B8.2 HOME briefly lost while parking", "w")

            if lost <= 5:
                b82_drive_all(bot, B82_HOME_FINAL_POWER, 0.055)
                continue

            b6_motor_stop(bot)

            if lost >= 14:
                event("B8.2 HOME lost too long", "e")
                return False

            time.sleep(0.08)
            continue

        lost = 0

        dist = float(m.get("dist", 9999))
        gap = float(m.get("front_gap", dist))
        moved = abs(start_dist - dist)

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = f"B8.2 HOME parking: dist={dist:.0f} gap={gap:.0f} best={best_dist:.0f}"

        if b82_safe_stop_home(dist, gap):
            b6_motor_stop(bot)
            event(f"B8.2 HOME parked safely: dist={dist:.0f} gap={gap:.0f}", "k")
            return True

        # Re-face if backing angle drifts.
        err = b81_home_error_deg()
        now = time.monotonic()

        if err is not None and abs(err) >= B82_HOME_REFACE_ERR_DEG and now - last_reface_t >= B82_HOME_REFACE_COOLDOWN_S:
            b6_motor_stop(bot)
            event(f"B8.2 HOME drift {err:.1f}°, re-facing", "w")
            straight_face_once_verified(bot, "home", mode="backward", tolerance=7, attempts=1)
            last_reface_t = time.monotonic()
            continue

        # Progress watchdog.
        if dist < best_dist - B82_PROGRESS_MM:
            best_dist = dist
            last_progress_t = time.monotonic()
            recovery_count = 0

        stuck = (time.monotonic() - last_progress_t) >= B82_STUCK_SEC

        if stuck:
            recovery_count += 1
            event(f"B8.2 HOME weak progress: gentle recovery {recovery_count}", "w")
            b82_drive_all(bot, B82_HOME_RECOVERY_POWER, 0.14)
            last_progress_t = time.monotonic()
            continue

        # Parking speed zones.
        if dist <= B82_HOME_FINAL_RAW or gap <= 230:
            power = B82_HOME_FINAL_POWER
            pulse = B82_FINAL_PULSE_S
            zone = "FINAL PARK"
        elif dist <= B82_HOME_PARK_RAW or gap <= 330:
            power = B82_HOME_PARK_POWER
            pulse = B82_CRAWL_PULSE_S
            zone = "PARK"
        else:
            power = B82_HOME_FAST_POWER
            pulse = B82_FAST_PULSE_S
            zone = "SLOW-MED"

        with lock:
            MISSION["message"] = f"B8.2 HOME {zone}: dist={dist:.0f} gap={gap:.0f}"

        b82_drive_all(bot, power, pulse)

    b6_motor_stop(bot)
    event("B8.2 timeout backward HOME", "e")
    return False


def spray_step(bot):
    """
    B8.2 real spray: ESP32 GPIO26 -> MOSFET -> pump, 2 seconds.
    """
    b6_motor_stop(bot)
    event("B8.2 spray: 2 seconds", "w")

    ok = fpms_spray(ESP32_SPRAY_MS)

    if ok:
        time.sleep((ESP32_SPRAY_MS / 1000.0) + 0.35)
        event("B8.2 spray complete", "k")
        return True

    event("B8.2 spray skipped/failed; continuing safely", "e")
    return True

# ============================================================
# END FPMS B8.4 LIDAR GAP MODE
# ============================================================




# ============================================================
# FPMS B8.4 LIDAR GAP SAFE MODE - 2026-05-06
# Fixes B8.2 being too weak / never reaching spray threshold.
# ============================================================

ESP32_SPRAY_MS = 2000

# Balanced speeds: slower than B8.1, stronger than B8.2.
B83_TARGET_FAST_POWER = -31
B83_TARGET_CRAWL_POWER = -19
B83_TARGET_FINAL_POWER = -16

B83_HOME_FAST_POWER = 31
B83_HOME_PARK_POWER = 22
B83_HOME_FINAL_POWER = 17
B83_HOME_RECOVERY_POWER = 34

# Safer but reachable target stop.
# Your stuck point was around dist=405 gap=205, so stop before that stalls.
B83_TARGET_STOP_GAP = 220
B83_TARGET_STOP_RAW = 430
B83_TARGET_CRAWL_RAW = 620
B83_TARGET_FINAL_RAW = 500

# HOME parking: stop safely before collision.
B83_HOME_STOP_RAW = 420
B83_HOME_GAP_STOP = 210
B83_HOME_PARK_RAW = 720
B83_HOME_FINAL_RAW = 560

# Pulse timing.
B83_FAST_PULSE_S = 0.18
B83_CRAWL_PULSE_S = 0.13
B83_FINAL_PULSE_S = 0.085
B83_SETTLE_S = 0.045

# Progress watchdog.
B83_PROGRESS_MM = 5
B83_STUCK_SEC = 1.7

# HOME angle correction.
B83_HOME_REFACE_ERR_DEG = 15
B83_HOME_REFACE_COOLDOWN_S = 1.2

# Turn smoothness.
B8_FACE_TOL_FWD = 6
B8_FACE_TOL_HOME = 7
B8_TURN_MAX_POWER = 40
B8_TURN_MIN_POWER = 23
B8_TURN_MAX_PULSE_MS = 72
B8_TURN_KICK_S = 0.010
B8_TURN_SETTLE_S = 0.085


def b83_drive_all(bot, power, pulse_s):
    bot.set_motor(power, power, power, power)
    time.sleep(pulse_s)
    b6_motor_stop(bot)
    time.sleep(B83_SETTLE_S)


def b83_stop_target(dist, gap):
    if gap is not None and gap <= B83_TARGET_STOP_GAP:
        return True
    if dist is not None and dist <= B83_TARGET_STOP_RAW:
        return True
    return False


def b83_stop_home(dist, gap):
    if dist is not None and dist <= B83_HOME_STOP_RAW:
        return True
    if gap is not None and gap <= B83_HOME_GAP_STOP and dist is not None and dist <= B83_HOME_PARK_RAW:
        return True
    return False


def straight_forward_to_target(bot, target):
    """
    B8.3 target approach:
    face once -> slow-medium -> crawl -> final -> stop/spray.
    """
    import time
    global stop_requested

    target = str(target).lower()

    m = b81_marker_filtered(target)
    if not m or not m.get("locked"):
        event(f"B8.3 {target.upper()} not locked", "e")
        return False

    try:
        plan_route(target)
    except Exception as e:
        event(f"B8.3 route display skipped: {e}", "w")

    event(f"B8.3 approach {target.upper()}: balanced safe mode", "i")

    if not straight_face_once_verified(bot, target, mode="forward", tolerance=6, attempts=1):
        return False

    start = b81_marker_filtered(target)
    start_dist = float(start.get("dist", 9999)) if start else 9999
    best_dist = start_dist
    last_progress_t = time.monotonic()
    t0 = time.monotonic()
    lost = 0
    final_seen = False

    while time.monotonic() - t0 < STRAIGHT_TIMEOUT_S:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        m = b81_marker_filtered(target, samples=3, delay_s=0.015)

        if not m or not m.get("locked"):
            lost += 1
            b6_motor_stop(bot)
            if lost >= STRAIGHT_LOST_LIMIT:
                event(f"B8.3 {target.upper()} lost too long", "e")
                return False
            time.sleep(0.07)
            continue

        lost = 0

        dist = float(m.get("dist", 9999))
        gap = float(m.get("front_gap", dist))
        moved = max(0, start_dist - dist)

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = f"B8.3 target {target.upper()}: dist={dist:.0f} gap={gap:.0f}"

        if b83_stop_target(dist, gap):
            b6_motor_stop(bot)
            event(f"B8.3 {target.upper()} stop/spray point: dist={dist:.0f} gap={gap:.0f}", "k")
            return True

        if dist < best_dist - B83_PROGRESS_MM:
            best_dist = dist
            last_progress_t = time.monotonic()

        # If already in final zone and no progress, don't sit forever.
        if final_seen and (time.monotonic() - last_progress_t) >= B83_STUCK_SEC:
            b6_motor_stop(bot)
            event(f"B8.3 {target.upper()} final-zone safe stop: dist={dist:.0f} gap={gap:.0f}", "k")
            return True

        if dist <= B83_TARGET_FINAL_RAW or gap <= 270:
            final_seen = True
            zone = "FINAL"
            power = B83_TARGET_FINAL_POWER
            pulse = B83_FINAL_PULSE_S
        elif dist <= B83_TARGET_CRAWL_RAW or gap <= 360:
            zone = "CRAWL"
            power = B83_TARGET_CRAWL_POWER
            pulse = B83_CRAWL_PULSE_S
        else:
            zone = "SLOW-MED"
            power = B83_TARGET_FAST_POWER
            pulse = B83_FAST_PULSE_S

        with lock:
            MISSION["message"] = f"B8.3 {zone} {target.upper()}: dist={dist:.0f} gap={gap:.0f}"

        b83_drive_all(bot, power, pulse)

    b6_motor_stop(bot)
    event(f"B8.3 timeout forward to {target.upper()}", "e")
    return False


def straight_backward_home(bot):
    """
    B8.3 HOME return:
    stronger than B8.2, still safe LiDAR parking.
    """
    import time
    global stop_requested

    m = b81_marker_filtered("home")
    if not m or not m.get("locked"):
        event("B8.3 HOME not locked", "e")
        return False

    try:
        plan_route("home")
    except Exception as e:
        event(f"B8.3 home route display skipped: {e}", "w")

    event("B8.3 HOME return: balanced parking mode", "i")

    if not straight_face_once_verified(bot, "home", mode="backward", tolerance=7, attempts=1):
        return False

    start = b81_marker_filtered("home")
    start_dist = float(start.get("dist", 9999)) if start else 9999
    best_dist = start_dist
    last_progress_t = time.monotonic()
    last_reface_t = 0.0
    t0 = time.monotonic()
    lost = 0
    final_seen = False

    event(f"B8.3 HOME start dist={start_dist:.0f}mm", "i")

    while time.monotonic() - t0 < B81_HOME_TIMEOUT_S:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        m = b81_marker_filtered("home", samples=3, delay_s=0.015)

        if not m or not m.get("locked"):
            lost += 1
            event("B8.3 HOME briefly lost", "w")

            if lost <= 5:
                b83_drive_all(bot, B83_HOME_FINAL_POWER, 0.075)
                continue

            b6_motor_stop(bot)

            if lost >= 14:
                event("B8.3 HOME lost too long", "e")
                return False

            time.sleep(0.08)
            continue

        lost = 0

        dist = float(m.get("dist", 9999))
        gap = float(m.get("front_gap", dist))
        moved = abs(start_dist - dist)

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = f"B8.3 HOME: dist={dist:.0f} gap={gap:.0f} best={best_dist:.0f}"

        if b83_stop_home(dist, gap):
            b6_motor_stop(bot)
            event(f"B8.3 HOME parked safely: dist={dist:.0f} gap={gap:.0f}", "k")
            return True

        err = b81_home_error_deg()
        now = time.monotonic()

        if err is not None and abs(err) >= B83_HOME_REFACE_ERR_DEG and now - last_reface_t >= B83_HOME_REFACE_COOLDOWN_S:
            b6_motor_stop(bot)
            event(f"B8.3 HOME drift {err:.1f}°, re-facing", "w")
            straight_face_once_verified(bot, "home", mode="backward", tolerance=7, attempts=1)
            last_reface_t = time.monotonic()
            continue

        if dist < best_dist - B83_PROGRESS_MM:
            best_dist = dist
            last_progress_t = time.monotonic()

        stuck = (time.monotonic() - last_progress_t) >= B83_STUCK_SEC

        if stuck:
            event("B8.3 HOME progress weak: recovery pulse", "w")
            b83_drive_all(bot, B83_HOME_RECOVERY_POWER, 0.16)
            last_progress_t = time.monotonic()
            continue

        if dist <= B83_HOME_FINAL_RAW or gap <= 260:
            final_seen = True
            zone = "FINAL PARK"
            power = B83_HOME_FINAL_POWER
            pulse = B83_FINAL_PULSE_S
        elif dist <= B83_HOME_PARK_RAW or gap <= 390:
            zone = "PARK"
            power = B83_HOME_PARK_POWER
            pulse = B83_CRAWL_PULSE_S
        else:
            zone = "SLOW-MED"
            power = B83_HOME_FAST_POWER
            pulse = B83_FAST_PULSE_S

        with lock:
            MISSION["message"] = f"B8.3 HOME {zone}: dist={dist:.0f} gap={gap:.0f}"

        b83_drive_all(bot, power, pulse)

    b6_motor_stop(bot)
    event("B8.3 timeout backward HOME", "e")
    return False


def spray_step(bot):
    """
    B8.3 spray: 2 seconds through ESP32.
    """
    b6_motor_stop(bot)
    event("B8.3 spray: 2 seconds", "w")

    ok = fpms_spray(ESP32_SPRAY_MS)

    if ok:
        time.sleep((ESP32_SPRAY_MS / 1000.0) + 0.35)
        event("B8.3 spray complete", "k")
        return True

    event("B8.3 spray failed/skipped; continuing safely", "e")
    return True

# ============================================================
# END FPMS B8.4 LIDAR GAP SAFE MODE
# ============================================================




# ============================================================
# FPMS B8.4 LIDAR GAP PRECISION - 2026-05-06
# Uses front_gap as the PRIMARY truth.
#
# Required:
# - M1/M2: stop at 15cm from front/top of car
# - HOME: stop at 10cm from front/top of car
# - Spray: 2 seconds
# ============================================================

ESP32_SPRAY_MS = 2000

# EXACT GAP TARGETS FROM FRONT/TOP OF CAR
B84_TARGET_STOP_GAP_MM = 150
B84_HOME_STOP_GAP_MM = 100

# Backup dist stops only if front_gap is missing.
# Since LiDAR is about 200mm behind front, dist ~= gap + 200mm.
B84_TARGET_STOP_DIST_BACKUP_MM = 360
B84_HOME_STOP_DIST_BACKUP_MM = 310

# Speed zones based on front_gap.
B84_TARGET_SLOW_GAP_MM = 360
B84_TARGET_FINAL_GAP_MM = 240

B84_HOME_SLOW_GAP_MM = 380
B84_HOME_FINAL_GAP_MM = 220

# Slower but strong enough to actually move.
# Negative = forward on this chassis. Positive = backward.
B84_TARGET_APPROACH_POWER = -30
B84_TARGET_SLOW_POWER = -21
B84_TARGET_FINAL_POWER = -16
B84_TARGET_RECOVERY_POWER = -25

B84_HOME_APPROACH_POWER = 32
B84_HOME_SLOW_POWER = 24
B84_HOME_FINAL_POWER = 18
B84_HOME_RECOVERY_POWER = 34

# Pulse lengths.
B84_APPROACH_PULSE_S = 0.17
B84_SLOW_PULSE_S = 0.12
B84_FINAL_PULSE_S = 0.075
B84_SETTLE_S = 0.050

# Stop confidence: require 2 consecutive LiDAR-confirmed stops.
B84_STOP_CONFIRM_READS = 2

# Progress watchdog.
B84_PROGRESS_MM = 4
B84_STUCK_SEC = 1.8

# Re-face HOME if angle drifts too much.
B84_HOME_REFACE_ERR_DEG = 15
B84_HOME_REFACE_COOLDOWN_S = 1.1

# Face tuning: not jerky.
B8_FACE_TOL_FWD = 6
B8_FACE_TOL_HOME = 7
B8_TURN_MAX_POWER = 40
B8_TURN_MIN_POWER = 23
B8_TURN_MAX_PULSE_MS = 70
B8_TURN_KICK_S = 0.010
B8_TURN_SETTLE_S = 0.085


def b84_get_lidar_gap(name, samples=5):
    """
    Returns filtered (dist_mm, front_gap_mm, angle_deg).
    PRIMARY: front_gap from marker tracker.
    BACKUP: dist - LIDAR_TO_FRONT.
    """
    m = b81_marker_filtered(name, samples=samples, delay_s=0.012)
    if not m or not m.get("locked"):
        return None, None, None

    dist = None
    gap = None
    angle = None

    try:
        dist = float(m.get("dist"))
    except Exception:
        pass

    try:
        gap = float(m.get("front_gap"))
    except Exception:
        pass

    if gap is None and dist is not None:
        try:
            gap = dist - float(LIDAR_TO_FRONT)
        except Exception:
            gap = dist - 200.0

    try:
        angle = float(m.get("angle"))
    except Exception:
        pass

    return dist, gap, angle


def b84_drive_pulse(bot, power, pulse_s):
    bot.set_motor(power, power, power, power)
    time.sleep(pulse_s)
    b6_motor_stop(bot)
    time.sleep(B84_SETTLE_S)


def b84_target_stop_reached(dist, gap):
    if gap is not None:
        return gap <= B84_TARGET_STOP_GAP_MM
    if dist is not None:
        return dist <= B84_TARGET_STOP_DIST_BACKUP_MM
    return False


def b84_home_stop_reached(dist, gap):
    if gap is not None:
        return gap <= B84_HOME_STOP_GAP_MM
    if dist is not None:
        return dist <= B84_HOME_STOP_DIST_BACKUP_MM
    return False


def b84_wait_confirmed_stop(name, is_home=False):
    """
    Confirm the stop zone using repeated filtered LiDAR reads.
    """
    ok_count = 0

    for _ in range(5):
        dist, gap, angle = b84_get_lidar_gap(name, samples=3)

        if is_home:
            ok = b84_home_stop_reached(dist, gap)
        else:
            ok = b84_target_stop_reached(dist, gap)

        if ok:
            ok_count += 1
        else:
            ok_count = 0

        if ok_count >= B84_STOP_CONFIRM_READS:
            return True, dist, gap

        time.sleep(0.035)

    return False, dist, gap


def straight_forward_to_target(bot, target):
    """
    B8.4 M1/M2:
    Face once -> drive using LiDAR gap zones -> stop at 15cm front_gap -> spray.
    """
    import time
    global stop_requested

    target = str(target).lower()

    m = marker_copy(target)
    if not m or not m.get("locked"):
        event(f"B8.4 {target.upper()} not locked", "e")
        return False

    try:
        plan_route(target)  # visual only
    except Exception as e:
        event(f"B8.4 route display skipped: {e}", "w")

    event(f"B8.4 {target.upper()}: LiDAR front_gap stop at 150mm", "i")

    if not straight_face_once_verified(bot, target, mode="forward", tolerance=6, attempts=1):
        return False

    start_dist, start_gap, _ = b84_get_lidar_gap(target)
    if start_dist is None:
        start_dist = 9999

    best_gap = start_gap if start_gap is not None else 9999
    last_progress_t = time.monotonic()
    t0 = time.monotonic()
    lost = 0
    stop_seen_once = False

    while time.monotonic() - t0 < STRAIGHT_TIMEOUT_S:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        dist, gap, angle = b84_get_lidar_gap(target, samples=3)

        if dist is None and gap is None:
            lost += 1
            b6_motor_stop(bot)
            if lost >= STRAIGHT_LOST_LIMIT:
                event(f"B8.4 {target.upper()} LiDAR lost too long", "e")
                return False
            time.sleep(0.06)
            continue

        lost = 0

        if gap is None:
            gap = dist - 200.0 if dist is not None else 9999

        moved = max(0, start_dist - dist) if dist is not None else 0

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = f"B8.4 {target.upper()}: gap={gap:.0f}mm target=150mm dist={dist:.0f}"

        if b84_target_stop_reached(dist, gap):
            b6_motor_stop(bot)

            confirmed, cdist, cgap = b84_wait_confirmed_stop(target, is_home=False)
            if confirmed:
                event(f"B8.4 {target.upper()} STOP confirmed: gap={cgap:.0f}mm dist={cdist:.0f}mm", "k")
                return True

            # If one stop was seen and then jitter moved, accept if still close.
            if stop_seen_once:
                event(f"B8.4 {target.upper()} STOP accepted after jitter: gap={gap:.0f}mm dist={dist:.0f}mm", "k")
                return True

            stop_seen_once = True

        # Progress tracking from front_gap.
        if gap < best_gap - B84_PROGRESS_MM:
            best_gap = gap
            last_progress_t = time.monotonic()

        stuck = (time.monotonic() - last_progress_t) >= B84_STUCK_SEC

        if stuck:
            event(f"B8.4 {target.upper()} weak gap progress: recovery pulse", "w")
            b84_drive_pulse(bot, B84_TARGET_RECOVERY_POWER, 0.11)
            last_progress_t = time.monotonic()
            continue

        # Gap-based speed zones.
        if gap <= B84_TARGET_FINAL_GAP_MM:
            zone = "FINAL"
            power = B84_TARGET_FINAL_POWER
            pulse = B84_FINAL_PULSE_S
        elif gap <= B84_TARGET_SLOW_GAP_MM:
            zone = "SLOW"
            power = B84_TARGET_SLOW_POWER
            pulse = B84_SLOW_PULSE_S
        else:
            zone = "APPROACH"
            power = B84_TARGET_APPROACH_POWER
            pulse = B84_APPROACH_PULSE_S

        with lock:
            MISSION["message"] = f"B8.4 {zone} {target.upper()}: gap={gap:.0f}mm → stop 150mm"

        b84_drive_pulse(bot, power, pulse)

    b6_motor_stop(bot)
    event(f"B8.4 timeout to {target.upper()}", "e")
    return False


def straight_backward_home(bot):
    """
    B8.4 HOME:
    Face HOME once -> reverse using LiDAR front_gap -> park at 100mm.
    """
    import time
    global stop_requested

    m = marker_copy("home")
    if not m or not m.get("locked"):
        event("B8.4 HOME not locked", "e")
        return False

    try:
        plan_route("home")  # visual only
    except Exception as e:
        event(f"B8.4 home route display skipped: {e}", "w")

    event("B8.4 HOME: LiDAR front_gap park at 100mm", "i")

    if not straight_face_once_verified(bot, "home", mode="backward", tolerance=7, attempts=1):
        return False

    start_dist, start_gap, _ = b84_get_lidar_gap("home")
    if start_dist is None:
        start_dist = 9999

    best_gap = start_gap if start_gap is not None else 9999
    last_progress_t = time.monotonic()
    last_reface_t = 0.0
    t0 = time.monotonic()
    lost = 0
    stop_seen_once = False

    event(f"B8.4 HOME start: gap={best_gap:.0f}mm stop=100mm", "i")

    while time.monotonic() - t0 < B81_HOME_TIMEOUT_S:
        if stop_requested:
            b6_motor_stop(bot)
            return False

        dist, gap, angle = b84_get_lidar_gap("home", samples=3)

        if dist is None and gap is None:
            lost += 1
            b6_motor_stop(bot)
            event("B8.4 HOME LiDAR briefly lost", "w")

            if lost >= 14:
                event("B8.4 HOME LiDAR lost too long", "e")
                return False

            # Small safe backing nudge if briefly lost.
            if lost <= 5:
                b84_drive_pulse(bot, B84_HOME_FINAL_POWER, 0.055)

            time.sleep(0.05)
            continue

        lost = 0

        if gap is None:
            gap = dist - 200.0 if dist is not None else 9999

        moved = max(0, start_dist - dist) if dist is not None else 0

        with lock:
            MISSION["moved_mm"] = round(moved, 1)
            MISSION["message"] = f"B8.4 HOME: gap={gap:.0f}mm target=100mm dist={dist:.0f}"

        # Exact HOME park stop.
        if b84_home_stop_reached(dist, gap):
            b6_motor_stop(bot)

            confirmed, cdist, cgap = b84_wait_confirmed_stop("home", is_home=True)
            if confirmed:
                event(f"B8.4 HOME PARK confirmed: gap={cgap:.0f}mm dist={cdist:.0f}mm", "k")
                return True

            if stop_seen_once:
                event(f"B8.4 HOME PARK accepted after jitter: gap={gap:.0f}mm dist={dist:.0f}mm", "k")
                return True

            stop_seen_once = True

        # Re-face if HOME angle drifts too much.
        err = b81_home_error_deg()
        now = time.monotonic()

        if err is not None and abs(err) >= 15 and now - last_reface_t >= 1.1:
            b6_motor_stop(bot)
            event(f"B8.4 HOME angle drift {err:.1f}°, re-face", "w")
            straight_face_once_verified(bot, "home", mode="backward", tolerance=7, attempts=1)
            last_reface_t = time.monotonic()
            continue

        # Progress tracking from front_gap.
        if gap < best_gap - B84_PROGRESS_MM:
            best_gap = gap
            last_progress_t = time.monotonic()

        stuck = (time.monotonic() - last_progress_t) >= B84_STUCK_SEC

        if stuck:
            event("B8.4 HOME weak gap progress: recovery pulse", "w")
            b84_drive_pulse(bot, B84_HOME_RECOVERY_POWER, 0.12)
            last_progress_t = time.monotonic()
            continue

        # Gap-based parking zones.
        if gap <= B84_HOME_FINAL_GAP_MM:
            zone = "FINAL PARK"
            power = B84_HOME_FINAL_POWER
            pulse = B84_FINAL_PULSE_S
        elif gap <= B84_HOME_SLOW_GAP_MM:
            zone = "SLOW PARK"
            power = B84_HOME_SLOW_POWER
            pulse = B84_SLOW_PULSE_S
        else:
            zone = "APPROACH"
            power = B84_HOME_APPROACH_POWER
            pulse = B84_APPROACH_PULSE_S

        with lock:
            MISSION["message"] = f"B8.4 HOME {zone}: gap={gap:.0f}mm → park 100mm"

        b84_drive_pulse(bot, power, pulse)

    b6_motor_stop(bot)
    event("B8.4 HOME timeout", "e")
    return False


def drive_to_marker(bot, target, timeout=MISSION_TIMEOUT_S):
    target = str(target).lower()
    if target == "home":
        return straight_backward_home(bot)
    return straight_forward_to_target(bot, target)


def spray_step(bot):
    """
    B8.4 spray: 2 seconds through ESP32.
    """
    b6_motor_stop(bot)
    event("B8.4 spray: ESP32 2 seconds", "w")

    ok = fpms_spray(ESP32_SPRAY_MS)

    if ok:
        time.sleep((ESP32_SPRAY_MS / 1000.0) + 0.30)
        event("B8.4 spray complete", "k")
        return True

    event("B8.4 spray failed/skipped; continuing safely", "e")
    return True

# ============================================================
# END FPMS B8.4 LIDAR GAP PRECISION
# ============================================================


if __name__ == '__main__':
    event('FPMS B8.4 LIDAR GAP starting', 'i')
    threading.Thread(target=lidar_loop, daemon=True).start()
    threading.Thread(target=esp_loop, daemon=True).start()
    threading.Thread(target=metrics_loop, daemon=True).start()
    threading.Thread(target=auto_loop, daemon=True).start()
    threading.Thread(target=face_hold_loop, daemon=True).start()
    app.run(host='0.0.0.0', port=WEB_PORT, threaded=True, debug=False, use_reloader=False)

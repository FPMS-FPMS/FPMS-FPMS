#!/usr/bin/env python3
"""
FPMS B6 REAL WORKING
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
D500_PORT = "/dev/ttyUSB1"
ESP_PORT = "/dev/serial/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-port0"
YAHBOOM_PORT = "/dev/ttyUSB0"

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
UPDATE_S = 0.40
UI_POINT_LIMIT = 95
FRONT_DEG = 25

# =====================
# Marker lock tuning
# =====================
MARKER_SNAP_MM = 190
MARKER_RELOCK_MM = 600
MARKER_ANGLE_RELOCK_DEG = 30
MARKER_DIST_RELOCK_MM = 800
MARKER_LOST_LIMIT = 30
CLUSTER_ANGLE_DEG = 8
CLUSTER_DIST_MM = 190
CLUSTER_RADIUS_MM = 220

# =====================
# Planner tuning
# =====================
GRID_RES = 100
ROUTE_CLEARANCE = 100    # PHASE5: was 70 — too tight for 230mm chassis
ROUTE_RADIUS = ROBOT_RADIUS + ROUTE_CLEARANCE
TARGET_EXCLUDE_RADIUS = 260
START_CLEAR_RADIUS = 150

# =====================
# Motor tuning
# =====================
FORWARD_POWER = 24
BACKWARD_POWER = -24
TURN_POWER = 34
FINE_FORWARD_POWER = 18
FINE_BACKWARD_POWER = -18
TURN_SIGN = -1     # PHASE5: flipped — new chassis turns opposite
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
FACE_HOLD_ENABLED = False  # PHASE5: off during testing
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
    "obstacles": [],   # PHASE5 KEEPOUT ZONES: list of {x, y, r_mm} user-placed obstacle disks (Nav2 KeepoutFilter pattern)

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
    import os
    if not os.path.exists(ESP_PORT):
        event("ESP not attached — skipping", "w")
        with lock:
            ESP["last_line"] = "DISABLED"
            ESP["n1_online"] = False
            ESP["n2_online"] = False
        return
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
            time.sleep(5)


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
    """PHASE5: Nav2-style self-filter using a CIRCLE around robot center
    + KEEPOUT ZONES (user-placed obstacle markers, injected as virtual LiDAR
    points). Real Nav2 calls these the KeepoutFilter — manually marked no-go
    disks added on top of LiDAR-detected obstacles. Solves the 'lidar sees
    obstacle but planner routes through it anyway' problem by FORCING those
    cells into the costmap."""
    obs = []
    P5_ROBOT_FOOTPRINT_MM = 140   # circle radius around robot center (new chassis is ~280mm wide)
    P5_TARGET_EXCLUDE_MM  = 220   # tighter than B6's 260; we want to see obstacles closer to target
    fp_sq = P5_ROBOT_FOOTPRINT_MM * P5_ROBOT_FOOTPRINT_MM
    for p in pts:
        x, y = p["x"], p["y"]
        # Single-circle self-filter (Nav2 pattern)
        if x*x + y*y < fp_sq:
            continue
        # Don't let obstacle inflation block the target itself
        if target_marker:
            dx = x - target_marker["x"]
            dy = y - target_marker["y"]
            if dx*dx + dy*dy < P5_TARGET_EXCLUDE_MM * P5_TARGET_EXCLUDE_MM:
                continue
        obs.append(p)
    # === Inject user-placed keepout zones as virtual obstacle points (Nav2 KeepoutFilter) ===
    # Each keepout is a disk of radius r. We sample its boundary as virtual
    # LiDAR returns so the A* planner sees them as solid obstacles.
    with lock:
        keepouts = list(S.get("obstacles", []))
    import math as _m
    for ko in keepouts:
        kx = ko.get("x", ko.get("wx", 0))
        ky = ko.get("y", ko.get("wy", 0))
        kr=ko.get("r",150)
        # Fill the entire disk with virtual obstacle points on a 40mm grid,
        # so A* cannot slip through gaps in a boundary-only sample.
        step = 80
        for dx in range(-int(kr), int(kr) + 1, step):
            for dy in range(-int(kr), int(kr) + 1, step):
                if dx*dx + dy*dy <= kr*kr:
                    vx, vy = kx + dx, ky + dy
                    obs.append({"x": vx, "y": vy, "d": _m.hypot(vx, vy),
                                "deg": 0, "i": 0, "keepout": True})
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


def simplify_path(pts, epsilon=150):
    """Douglas-Peucker: remove redundant waypoints. 15 pts → 3-4 key turns."""
    import math as _spm
    if len(pts) <= 2: return pts
    def _gx(p): return p["x"] if isinstance(p,dict) else p[0]
    def _gy(p): return p["y"] if isinstance(p,dict) else p[1]
    s,e = pts[0],pts[-1]
    dx,dy = _gx(e)-_gx(s),_gy(e)-_gy(s)
    ll = _spm.hypot(dx,dy)
    mx_d,mx_i = 0,0
    for i in range(1,len(pts)-1):
        if ll<1:
            d=_spm.hypot(_gx(pts[i])-_gx(s),_gy(pts[i])-_gy(s))
        else:
            d=abs(dx*(_gy(s)-_gy(pts[i]))-(_gx(s)-_gx(pts[i]))*dy)/ll
        if d>mx_d: mx_d=d; mx_i=i
    if mx_d>epsilon:
        left=simplify_path(pts[:mx_i+1],epsilon)
        right=simplify_path(pts[mx_i:],epsilon)
        return left[:-1]+right
    else:
        return [s,e]

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
        path = simplify_path(raw, epsilon=150)
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
_active_markers = set()  # during mission, only track these markers

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
            # During missions, only track active target + home to prevent M2 jumping
            if mission_active and _active_markers and name not in _active_markers:
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



# World-frame dead-reckoning odometry (mm, radians)
# Updated by _p5_navdrive during missions.  x=right, y=forward, theta=heading(rad).
# Used ONLY for world-frame keepout zone tracking — not for navigation.


_p5_odom = {"x": 0.0, "y": 0.0, "theta": 0.0}
def _p5_odom_reset():
    _p5_odom.update({"x": 0.0, "y": 0.0, "theta": 0.0})


_p5_retrace = []  # outbound (turn_deg, mm_driven) for return trip

def _p5_navdrive(bot, target, timeout=55.0):
    import math as _m
    _TKMM=_m.pi*70.0/1320.0; _DRV=26; _TRN=36; _COAST=0.82; _HZ=20
    _FSTOP=120; _KPH=10
    _stop_at=B6_HOME_STOP_RAW if target=="home" else B6_TARGET_STOP_RAW
    def _enc():
        try: return bot.get_motor_encoder()
        except: return None
    def _gz():
        try:
            g=bot.get_gyroscope_data(); return float(g[2]) if g else 0.0
        except: return 0.0
    def _fr():
        with lock: f=S.get("front_raw",0)
        return int(f) if f else 9999
    def _stp():
        try: bot.set_motor(0,0,0,0)
        except: pass
    def _spin(deg):
        if abs(deg)<2.0: return
        rad=abs(_m.radians(deg))
        d=(1 if deg>0 else -1)*TURN_SIGN; pw=_TRN
        if d>0: bot.set_motor(-pw,-pw,+pw,+pw)
        else:   bot.set_motor(+pw,+pw,-pw,-pw)
        ig=0.0; lt=time.time()
        while ig<rad*_COAST:
            if stop_requested: _stp(); return
            now=time.time(); dt=now-lt; lt=now
            if 0<dt<0.5: ig+=abs(_gz())*dt
            time.sleep(1.0/_HZ)
        _stp(); time.sleep(0.12)
        event(f"  rotated {deg:+.0f}°","i")
    def _fwd(mm,tgt,rev=False):
        e0=_enc()
        if not e0: return 0.0,"NO_ENC"
        pwr=-_DRV if rev else _DRV
        hd=0.0; lt=time.time(); t0=lt
        while time.time()-t0<12.0:
            if stop_requested: _stp(); return 0.0,"STOP"
            e=_enc()
            dm=(sum(abs(e[i]-e0[i]) for i in range(4))/4.0*_TKMM) if e else 0.0
            mk=marker_copy(tgt)
            if mk and mk.get("locked") and float(mk["dist"])<=_stop_at:
                _stp(); return dm,"ARRIVED"
            if not rev and _fr()<_FSTOP: _stp(); return dm,"BLOCKED"
            if dm>=mm: _stp(); return dm,"DONE"
            now=time.time(); dt=now-lt; lt=now
            if 0<dt<0.5: hd+=_gz()*dt
            c=max(-6,min(6,int(hd*_KPH*(1 if not rev else -1))))
            bot.set_motor(pwr-c,pwr-c,pwr+c,pwr+c)
            with lock: MISSION["message"]=f"P5 {'BWD' if rev else 'FWD'} {tgt.upper()} {dm:.0f}/{mm:.0f}mm"
            time.sleep(1.0/_HZ)
        _stp(); return mm,"TIMEOUT"
    def _face_and_drive(tgt):
        for attempt in range(15):
            if stop_requested: _stp(); return False
            mk=marker_copy(tgt)
            if not mk or not mk.get("locked"):
                event(f"P5 LOST {tgt.upper()} final approach","e"); return False
            dist=float(mk["dist"])
            if dist<=_stop_at: _stp(); return True
            angle=float(mk["angle"]); err=signed_angle(angle)
            if abs(err)>15:
                _spin(err); time.sleep(0.3); continue
            chunk=min(500,dist-_stop_at+80)
            mm,reason=_fwd(chunk,tgt)
            if reason=="ARRIVED": return True
            if reason=="BLOCKED": event("P5 BLOCKED final approach","w"); return False
        return False
    mk0=marker_copy(target)
    if not mk0 or not mk0.get("locked"):
        event(f"P5: {target.upper()} not locked","e"); return False
    event(f"P5->{target.upper()} enc+gyro WB=170mm dist={mk0['dist']:.0f}mm","k")
    # ── HOME RETRACE: drive backward along stored outbound path ──
    if target=="home" and len(_p5_retrace)>0:
        ret = list(reversed(_p5_retrace))
        _p5_retrace.clear()
        event(f"P5 HOME RETRACE: {len(ret)} segments backward","k")
        for i,(td,dd) in enumerate(ret):
            event(f"  ret[{i}]: bwd {dd:.0f}mm then undo turn {-td:+.0f}°","i")
        for i,(td,dd) in enumerate(ret):
            if stop_requested: _stp(); return False
            mk=marker_copy("home")
            if mk and mk.get("locked") and float(mk["dist"])<=_stop_at:
                _stp(); event(f"P5 ARRIVED HOME during retrace","k")
                return True
            event(f"P5 ret[{i}/{len(ret)}]: bwd {dd:.0f}mm","i")
            mm_done,reason=_fwd(dd,"home",rev=True)
            event(f"P5 ret[{i}] bwd {mm_done:.0f}mm reason={reason}","i")
            if reason=="ARRIVED": return True
            if reason=="STOP": return False
            # Check if already home before undoing turn
            mk=marker_copy("home")
            if mk and mk.get("locked") and float(mk["dist"])<=_stop_at:
                _stp(); event(f"P5 ARRIVED HOME — skipping final turn","k"); return True
            if abs(td)>2:
                _spin(-td)
                time.sleep(0.4)
        _stp()
        # One LiDAR-guided correction to face AWAY from HOME (HOME should be behind)
        _hm = marker_copy("home")
        if _hm and _hm.get("locked"):
            _ha = signed_angle(float(_hm["angle"]))
            # HOME should be at 180° (behind). Current angle _ha.
            # Need to spin so HOME moves to 180°.
            _corr = signed_angle(float(_hm["angle"]) - 180.0)
            if abs(_corr) > 10:
                _spin(_corr)
                event(f"P5 HOME correction {_corr:+.0f}° (HOME now behind)","i")
        # Final LiDAR-guided approach to HOME
        mk = marker_copy("home")
        if mk and mk.get("locked") and float(mk["dist"]) > _stop_at:
            event(f"P5 retrace done — final approach HOME {mk['dist']:.0f}mm","k")
            return _face_and_drive("home")
        event(f"P5 retrace done — HOME","k")
        return True
    if target!="home": _p5_retrace.clear()
    plan_route(target); time.sleep(0.1)
    with lock:
        path=list(S.get("route_path",[])); route_msg=S.get("route_msg","")
    if ("STRAIGHT CLEAR" in route_msg or len(path)<=2) and target == "home":
        event(f"P5 STRAIGHT → face+drive HOME","k")
        return _face_and_drive(target)
    segments=[]; heading=0.0
    for i in range(1,len(path)):
        dx=path[i]["x"]-path[i-1]["x"]; dy=path[i]["y"]-path[i-1]["y"]
        sd=_m.hypot(dx,dy)
        if sd<30: continue
        sb=_m.atan2(dx,dy); tr=sb-heading
        while tr>_m.pi: tr-=2*_m.pi
        while tr<-_m.pi: tr+=2*_m.pi
        segments.append((_m.degrees(tr),sd)); heading=sb
    event(f"P5 route: {len(segments)} segments","k")
    for i,(td,dd) in enumerate(segments):
        event(f"  seg[{i}]: turn {td:+.0f}° drive {dd:.0f}mm","i")
    for i,(turn_deg,drive_mm) in enumerate(segments):
        if stop_requested: _stp(); return False
        mk=marker_copy(target)
        if mk and mk.get("locked") and float(mk["dist"])<=_stop_at:
            _stp(); event(f"P5 ARRIVED {target.upper()} at seg[{i}]","k"); return True
        event(f"P5 seg[{i}/{len(segments)}]: turn {turn_deg:+.0f}° drive {drive_mm:.0f}mm","i")
        if abs(turn_deg)>2: _spin(turn_deg)
        if abs(turn_deg)>45:
            time.sleep(0.6)
            mk=marker_copy(target)
            if not mk or not mk.get("locked"):
                event(f"P5 relock wait after {turn_deg:+.0f}° spin","w")
                time.sleep(1.0)
        mm_done,reason=_fwd(drive_mm,target)
        if target!="home": _p5_retrace.append((turn_deg, mm_done))
        event(f"P5 seg[{i}] drove {mm_done:.0f}mm reason={reason}","i")
        if reason=="ARRIVED": return True
        if reason=="BLOCKED":
            event(f"P5 BLOCKED at seg[{i}] — skipping","w"); continue
        if reason=="STOP": return False
        time.sleep(0.3)
    event(f"P5 segments done → final approach","k")
    _d0 = 0
    _mkb = marker_copy(target)
    if _mkb and _mkb.get("locked"): _d0 = float(_mkb["dist"])
    result = _face_and_drive(target)
    _mka = marker_copy(target)
    _d1 = float(_mka["dist"]) if _mka and _mka.get("locked") else _d0
    _gap = _d0 - _d1
    if target != "home" and _gap > 20:
        _p5_retrace.append((0, _gap))
        event(f"P5 approach {_gap:.0f}mm recorded (no turn)","i")
    return result


def mission_worker(target):
    # PHASE5 pre-flight: log front + marker state so we can see why mission fails fast
    with lock:
        f0 = S.get("front_raw", 0)
        m_state = dict(S.get("markers", {}))
    m_target = m_state.get(target if target != "test" else "m1")
    if m_target:
        event(f"PRE-FLIGHT {target}: front={f0}mm | target_dist={m_target.get('dist',0):.0f}mm "
              f"angle={m_target.get('angle',0):.0f}° | crit_front={B6_CRITICAL_FRONT_RAW}", "i")
    else:
        event(f"PRE-FLIGHT {target}: front={f0}mm | TARGET NOT SET", "e")

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
        if not _p5_navdrive(bot, target):
            raise RuntimeError("Could not reach target")
        if target != "home":
            spray_step(bot)
            plan_route("home")
            if not _p5_navdrive(bot, "home"):
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
<!doctype html><html><head><meta charset="utf-8"><title>FPMS B6 REAL WORKING</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#040911;color:#e8f7ff;font-family:Consolas,monospace;overflow:hidden}.top{height:58px;background:#071629;border-bottom:2px solid #ff4b70;padding:8px 14px;display:flex;justify-content:space-between;align-items:center}h1{margin:0;color:#ff4b70;letter-spacing:4px;font-size:22px}.sub{font-size:12px;color:#98d4ff}.main{height:calc(100vh - 58px);display:grid;grid-template-columns:1fr 360px;gap:8px;padding:8px}.panel{background:#071629;border:1px solid #28506f;border-radius:12px;padding:10px;min-height:0}.h{color:#00b7ff;letter-spacing:3px;font-weight:900;font-size:13px;border-bottom:1px solid #28506f;padding-bottom:6px;margin-bottom:8px}#mapbox{height:calc(100% - 28px);position:relative;background:#020711;border:1px solid #173450;border-radius:10px}canvas{position:absolute;left:0;top:0;width:100%;height:100%}.grid2{display:grid;grid-template-columns:1fr 1fr;gap:6px}button{background:#071427;border:1px solid #2e76aa;color:#e8f5ff;border-radius:9px;padding:8px;font-family:Consolas;font-weight:900}.green{border-color:#3af07a;color:#3af07a}.yellow{border-color:#ffbf00;color:#ffbf00}.danger{border-color:#ff4b70;color:#ff4b70}.active{background:#241900;border-color:#ffbf00;color:#ffbf00}.card{background:#020711;border:1px solid #1d4163;border-radius:9px;padding:8px;margin-top:6px}.big{font-size:18px;color:#00b7ff;font-weight:900}.small{font-size:11px;color:#98d4ff}.ok{color:#3af07a}.bad{color:#ff4b70}.warn{color:#ffbf00}#events{height:86px;overflow:auto;font-size:10px;white-space:pre-wrap}#leftDist{position:absolute;left:10px;top:10px;z-index:10;width:245px;background:rgba(2,7,17,.86);border:1px solid #1d4163;border-radius:10px;padding:9px;font-size:12px;line-height:1.45;color:#98d4ff}#leftDist .title{color:#00b7ff;letter-spacing:2px;font-weight:900;margin-bottom:5px}#leftDist b{color:#e8f7ff}#leftDist .ok{color:#3af07a}#leftDist .bad{color:#ff4b70}</style></head>
<body><div class="top"><div><h1>FPMS B6 REAL WORKING</h1><div class="sub">200cm × 250cm centered map • one webserver • strong lock • run M1/home</div></div><div id="clock"></div></div>
<div class="main"><div class="panel"><div class="h">LIVE MAP</div><div id="mapbox"><div id="leftDist"><div class="title">LIVE DISTANCES</div>Waiting...</div><canvas id="grid"></canvas><canvas id="dyn"></canvas></div></div>
<div class="panel"><div class="h">MARKERS</div><div class="grid2"><button id="b_m1" onclick="setMode('m1')">SET M1</button><button id="b_m2" onclick="setMode('m2')">SET M2</button><button id="b_home" onclick="setMode('home')">SET HOME</button><button id="b_obs" onclick="setMode('obstacle')" style="border-color:#ff4b70;color:#ff4b70">+ OBSTACLE</button><button class="danger" onclick="clearMarkers()">CLEAR ALL</button></div>
<div class="h" style="margin-top:10px">MISSION</div><div class="grid2"><button id="robotBtn" class="danger" onclick="toggleRobot()">ROBOT OFF</button><button id="autoBtn" class="danger" onclick="toggleAuto()">AUTO OFF</button><button class="green" onclick="runMission('m1')">TEST FULL M1</button><button class="green" onclick="runMission('m2')">RUN M2</button><button class="yellow" onclick="runMission('home')">RUN HOME</button><button class="danger" onclick="stopMission()">STOP</button></div><div class="grid2"><button class="green" onclick="queueTarget('m2')" id="queueM2Btn">QUEUE M2</button><button class="green" onclick="queueTarget('m1')" id="queueM1Btn">QUEUE M1</button></div><div class="card"><div id="mission" class="big">--</div><div id="msg" class="small">--</div><div id="routeMsg" class="small">--</div></div>
<div class="h" style="margin-top:10px">ESP-NOW</div><div class="grid2"><div class="card"><div class="small">NODE1</div><div id="n1" class="big">--</div></div><div class="card"><div class="small">ALERT1</div><div id="a1" class="big">--</div></div><div class="card"><div class="small">NODE2</div><div id="n2" class="big">--</div></div><div class="card"><div class="small">ALERT2</div><div id="a2" class="big">--</div></div></div><div id="lastEsp" class="card small">--</div>
<div class="h" style="margin-top:10px">LIVE DISTANCES</div><div id="mread" class="card small">--</div>
<div class="h" style="margin-top:10px">DIAGNOSTICS</div><div class="grid2"><div class="card"><div class="small">STATUS</div><div id="status" class="big">--</div></div><div class="card"><div class="small">SCAN</div><div id="hz" class="big">0</div></div><div class="card"><div class="small">POINTS</div><div id="points" class="big">0</div></div><div class="card"><div class="small">FRONT</div><div id="front" class="big">--</div></div></div>
<div class="h" style="margin-top:10px">EVENTS</div><div id="events" class="card"></div></div></div>
<script>
const XMIN=-1000,XMAX=1000,YMIN=-1500,YMAX=1500,ROBOT_W=230,LIDAR_FRONT=200,TARGET_STOP=300,FRONT_STOP=300,ROUTE_RADIUS=185;let scale=1,ox=0,oy=0,mode=null,scan=[],markers={},route=[],blocked=[];const grid=document.getElementById('grid'),dyn=document.getElementById('dyn'),box=document.getElementById('mapbox'),g=grid.getContext('2d'),d=dyn.getContext('2d');function $(x){return document.getElementById(x)}function mmToPix(x,y){return[ox+x*scale,oy-y*scale]}function pixToMm(px,py){return[(px-ox)/scale,(oy-py)/scale]}function resize(){let r=box.getBoundingClientRect();grid.width=dyn.width=Math.floor(r.width);grid.height=dyn.height=Math.floor(r.height);scale=Math.min(grid.width/(XMAX-XMIN),grid.height/(YMAX-YMIN))*.96;ox=grid.width/2-(XMIN+XMAX)/2*scale;oy=grid.height/2+(YMIN+YMAX)/2*scale;drawGrid();drawDyn()}function drawGrid(){g.clearRect(0,0,grid.width,grid.height);let[x0,yt]=mmToPix(XMIN,YMAX),[x1,yb]=mmToPix(XMAX,YMIN);g.fillStyle='rgba(0,55,28,.25)';g.fillRect(x0,yt,x1-x0,yb-yt);g.strokeStyle='#00d66b';g.lineWidth=2;g.strokeRect(x0,yt,x1-x0,yb-yt);for(let x=XMIN;x<=XMAX;x+=100){let[px]=mmToPix(x,0);g.strokeStyle=x%500?'rgba(80,190,255,.16)':'rgba(80,190,255,.36)';g.beginPath();g.moveTo(px,yt);g.lineTo(px,yb);g.stroke()}for(let y=YMIN;y<=YMAX;y+=100){let[,py]=mmToPix(0,y);g.strokeStyle=y%500?'rgba(80,190,255,.16)':'rgba(80,190,255,.36)';g.beginPath();g.moveTo(x0,py);g.lineTo(x1,py);g.stroke()}let[lx,fy]=mmToPix(-ROBOT_W/2,LIDAR_FRONT),[rx,by]=mmToPix(ROBOT_W/2,-80);g.fillStyle='rgba(0,183,255,.16)';g.strokeStyle='#00b7ff';g.fillRect(lx,fy,rx-lx,by-fy);g.strokeRect(lx,fy,rx-lx,by-fy);let[cx,cy]=mmToPix(0,0);g.fillStyle='#00b7ff';g.beginPath();g.arc(cx,cy,6,0,Math.PI*2);g.fill();let[,s1]=mmToPix(0,FRONT_STOP),[,s2]=mmToPix(0,TARGET_STOP);g.setLineDash([7,6]);g.strokeStyle='#ffbf00';g.beginPath();g.moveTo(x0,s1);g.lineTo(x1,s1);g.stroke();g.strokeStyle='#ff4b70';g.beginPath();g.moveTo(x0,s2);g.lineTo(x1,s2);g.stroke();g.setLineDash([])}function mc(n){return n==='m1'?'#ff4b70':n==='m2'?'#ffbf00':'#3af07a'}function drawMarker(n){let m=markers[n];if(!m)return;let[px,py]=mmToPix(m.x,m.y),c=mc(n);d.strokeStyle=m.locked?c:'#888';d.lineWidth=3;d.beginPath();d.arc(px,py,14,0,Math.PI*2);d.stroke();d.fillStyle=c;d.font='bold 12px Consolas';d.textAlign='center';d.fillText(n.toUpperCase(),px,py-18);d.textAlign='left'}function drawRoute(){if(!route||route.length<2)return;d.lineJoin='round';d.lineCap='round';d.beginPath();route.forEach((p,i)=>{let[px,py]=mmToPix(p.x,p.y);if(i===0)d.moveTo(px,py);else d.lineTo(px,py)});d.strokeStyle='rgba(58,240,122,.18)';d.lineWidth=ROUTE_RADIUS*2*scale;d.stroke();d.beginPath();route.forEach((p,i)=>{let[px,py]=mmToPix(p.x,p.y);if(i===0)d.moveTo(px,py);else d.lineTo(px,py)});d.strokeStyle='#3af07a';d.lineWidth=4;d.stroke()}let needsDraw=false;function drawDyn(){d.clearRect(0,0,dyn.width,dyn.height);for(const p of scan){let[px,py]=mmToPix(p[0],p[1]);d.fillStyle='rgb(240,210,255)';d.fillRect(px-2.5,py-2.5,5,5)}for(const p of blocked){let[px,py]=mmToPix(p.x,p.y);d.fillStyle='#ff4b70';d.beginPath();d.arc(px,py,5,0,Math.PI*2);d.fill()}if(window.obstacles){for(const ob of window.obstacles){let[px,py]=mmToPix(ob.x,ob.y);let rPx=(ob.r||150)*scale;d.fillStyle='rgba(255,75,112,0.35)';d.strokeStyle='#ff4b70';d.lineWidth=2;d.beginPath();d.arc(px,py,rPx,0,Math.PI*2);d.fill();d.stroke();d.fillStyle='#ff4b70';d.font='bold 11px Consolas';d.textAlign='center';d.fillText('NO-GO',px,py-rPx-4);d.textAlign='left'}}drawRoute();drawMarker('m1');drawMarker('m2');drawMarker('home')}function setMode(m){mode=m;['m1','m2','home'].forEach(x=>$('b_'+x).classList.remove('active'));$('b_'+m).classList.add('active')}dyn.addEventListener('click',async e=>{if(!mode){alert('Choose marker first');return}let r=dyn.getBoundingClientRect();let[x,y]=pixToMm(e.clientX-r.left,e.clientY-r.top);let url=(mode==='obstacle')?'/api/add_obstacle':'/api/set_marker';let body=(mode==='obstacle')?{x:x,y:y,r:100}:{name:mode,x:x,y:y};let js=await(await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})).json();if(!js.ok)alert(js.msg||'failed')});async function clearMarkers(){await fetch('/api/clear_markers',{method:'POST'});await fetch('/api/clear_obstacles',{method:'POST'})}async function toggleRobot(){await fetch('/api/toggle_robot',{method:'POST'});poll()}async function toggleAuto(){await fetch('/api/toggle_auto',{method:'POST'});poll()}async function runMission(t){if(!confirm('Robot will move. Area clear?'))return;let js=await(await fetch('/api/run',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({target:t})})).json();if(!js.ok)alert(js.msg)}async function queueTarget(t){let js=await(await fetch('/api/queue',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({target:t})})).json();if(js.ok){let b=document.getElementById('queue'+t.charAt(0).toUpperCase()+t.slice(1)+'Btn');if(b){b.textContent=t.toUpperCase()+' QUEUED';b.className='yellow'}}}
async function stopMission(){await fetch('/api/stop',{method:'POST'})}function line(n,m){return m?`${n.toUpperCase()}: ${m.locked?'LOCKED':'LOST'} dist=${Math.round(m.dist)}mm gap=${Math.round(m.front_gap)}mm angle=${Math.round(m.angle)}°`:`${n.toUpperCase()}: empty`}function leftLine(n,m){if(!m)return `<b>${n.toUpperCase()}</b>: <span class="bad">EMPTY</span>`;let cls=m.locked?'ok':'bad';return `<b>${n.toUpperCase()}</b>: <span class="${cls}">${m.locked?'LOCKED':'LOST'}</span><br>`+`&nbsp;dist ${Math.round(m.dist)}mm | gap ${Math.round(m.front_gap)}mm<br>`+`&nbsp;angle ${Math.round(m.angle)}° | pts ${m.points||0}`}async function poll(){try{let s=await(await fetch('/api/state?t='+Date.now())).json();scan=s.scan||[];markers=s.markers||{};route=s.route_path||[];blocked=s.blocked_points||[];window.obstacles=s.obstacles||[];$('status').textContent=s.status;$('hz').textContent=s.scan_hz;$('points').textContent=s.points;$('front').textContent=s.front_raw||'--';$('mission').textContent=s.mission.state+' → '+s.mission.target;$('msg').textContent=s.mission.message;$('routeMsg').textContent=s.route_msg;$('robotBtn').textContent=s.robot_enabled?'ROBOT ON':'ROBOT OFF';$('robotBtn').className=s.robot_enabled?'green':'danger';$('autoBtn').textContent=s.auto_enabled?'AUTO ON':'AUTO OFF';$('autoBtn').className=s.auto_enabled?'green':'danger';$('n1').textContent=s.esp.n1_online?'ONLINE':'OFFLINE';$('a1').textContent=s.esp.alert1?'ALERT':'CLEAR';$('n2').textContent=s.esp.n2_online?'ONLINE':'OFFLINE';$('a2').textContent=s.esp.alert2?'ALERT':'CLEAR';$('lastEsp').textContent='Last ESP: '+(s.esp.last_line||'none')+' | age '+s.esp.age_s+'s';$('mread').innerHTML=line('m1',markers.m1)+'<br>'+line('m2',markers.m2)+'<br>'+line('home',markers.home);if($('leftDist')){$('leftDist').innerHTML='<div class="title">LIVE DISTANCES</div>'+leftLine('m1',markers.m1)+'<hr style="border-color:#173450">'+leftLine('m2',markers.m2)+'<hr style="border-color:#173450">'+leftLine('home',markers.home)+'<hr style="border-color:#173450">'+`<b>FRONT</b>: ${s.front_raw||'--'}mm`;}$('events').textContent=(s.events||[]).slice().reverse().map(e=>'['+e.t+'] '+e.msg).join('\n');requestAnimationFrame(drawDyn)}catch(e){$('msg').textContent='Poll error '+e}setTimeout(poll,650)}window.addEventListener('resize',resize);resize();poll();setInterval(()=>{$('clock').textContent=new Date().toLocaleTimeString()},1000);
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
            "odom": dict(_p5_odom),
            "obstacles": list(S.get("obstacles", [])),
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
    _p5_odom_reset()
    _active_markers.clear()
    event('Markers + odom cleared', 'w')
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


@app.route('/api/queue', methods=['POST'])
def api_queue_target():
    data = request.json or {}
    t = data.get("target", "").lower()
    if t not in ["m1", "m2"]:
        return jsonify(ok=False, msg="Bad target")
    if not mission_active:
        return jsonify(ok=False, msg="No active mission — use RUN instead")
    _mission_queue.append(t)
    event(f"QUEUED {t.upper()} (queue: {[x.upper() for x in _mission_queue]})", "k")
    return jsonify(ok=True, msg=f"{t.upper()} queued")

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


B6_TURN_POWER = 36
B6_TURN_PULSE_S = 0.14
B6_TURN_SETTLE_S = 0.10
B6_FACE_TOL_DEG = 7
B6_DRIVE_ANGLE_TOL_DEG = 16
B6_TARGET_STOP_RAW = 350
B6_HOME_STOP_RAW = 340
B6_CRITICAL_FRONT_RAW = 120
B6_SLOW_APPROACH_RAW = 430
B6_FINE_FORWARD_POWER = 17
B6_FINE_BACKWARD_POWER = -17
B6_FORWARD_POWER = 26    # PHASE5: flipped — new chassis is opposite sign
B6_BACKWARD_POWER = -26  # PHASE5: flipped — new chassis is opposite sign
B6_HOME_BACKWARD_ANGLE = 105
B6_FACEHOLD_INTERVAL_S = 0.42
B6_FACEHOLD_MIN_DIST = 430
B6_AUTO_FACEHOLD = False   # PHASE5: off during testing

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
    Real working B6 behavior:
    M1/M2 = forward.
    HOME = backward when HOME is behind/side-behind.
    """
    if target != "home":
        return "forward"

    m = marker_copy("home")
    if not m:
        return "forward"

    a = abs(signed_angle(float(m["angle"])))

    if a >= 70:
        return "backward"

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
    # PHASE5 entry diagnostic
    _m0 = marker_copy(target)
    with lock:
        _f = S.get("front_raw", 0)
    if _m0:
        event(f"DRIVE→{target.upper()}: dist={_m0['dist']:.0f}mm angle={_m0['angle']:.1f}° front={_f}mm stop_at={(340 if target=='home' else 380)}mm crit={200}mm", "i")
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
            event(f"B6 emergency front stop (front={dist:.0f}mm gap, dist_to_target={dist:.0f}mm)", "e")
            return False   # PHASE5: was True; emergency stop is NOT success

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


@app.route('/api/debug_obstacles')
def api_debug_obstacles():
    """PHASE5 diagnostic: show what the planner sees as obstacles vs filters out.
    Helps verify the self-filter isn't eating real obstacles."""
    pts = scan_points(True)
    m1 = marker_copy("m1")
    obs = planning_obstacles(pts, m1)
    obs_set = set((round(o["x"],1), round(o["y"],1)) for o in obs)
    filtered = []
    kept = []
    for p in pts:
        key = (round(p["x"],1), round(p["y"],1))
        if key in obs_set:
            kept.append({"x": p["x"], "y": p["y"]})
        else:
            filtered.append({"x": p["x"], "y": p["y"]})
    return jsonify({
        "total_points": len(pts),
        "kept_as_obstacles": len(kept),
        "filtered_as_self_or_target": len(filtered),
        "kept": kept[:200],
        "filtered": filtered[:200],
        "footprint_radius_mm": 140,
    })


@app.route('/api/add_obstacle', methods=['POST'])
def api_add_obstacle():
    """PHASE5: place a keepout zone at clicked (x,y) in robot frame, radius default 150mm."""
    data = request.get_json(force=True)
    x = float(data.get('x', 0))
    y = float(data.get('y', 0))
    r = float(data.get('r', 150))
    with lock:
        S['obstacles'].append({"x": x, "y": y, "r": r})
        n = len(S['obstacles'])
    event(f"KEEPOUT zone #{n} placed at ({x:.0f},{y:.0f}) r={r:.0f}mm", "w")
    return jsonify(ok=True, count=n)


@app.route('/api/reset_odom', methods=['POST'])
def api_reset_odom():
    _p5_odom_reset()
    with lock: S['obstacles'] = []
    event("Odom + keepouts reset to (0,0,0°)", "w")
    return jsonify(ok=True)


@app.route('/api/clear_obstacles', methods=['POST'])
def api_clear_obstacles():
    with lock:
        n = len(S['obstacles'])
        S['obstacles'] = []
    event(f"Cleared {n} keepout zones", "k")
    return jsonify(ok=True)


if __name__ == '__main__':
    event('FPMS PHASE 5 (B6 + tuned) starting', 'i')
    threading.Thread(target=lidar_loop, daemon=True).start()
    threading.Thread(target=esp_loop, daemon=True).start()
    threading.Thread(target=metrics_loop, daemon=True).start()
    threading.Thread(target=auto_loop, daemon=True).start()
    threading.Thread(target=face_hold_loop, daemon=True).start()
    app.run(host='0.0.0.0', port=WEB_PORT, threaded=True, debug=False, use_reloader=False)


# ============================================================================
# PHASE 5: Continuous proportional driver (replaces pulse-based B6)
# ----------------------------------------------------------------------------
# Real ROS diff_drive_controller pattern: run at 20Hz, compute (v, ω) from
# heading error to marker, blend into left/right wheel power, send every 50ms.
# 
# Math (canonical ROS):
#   v_left  = v - ω * wheelbase / 2
#   v_right = v + ω * wheelbase / 2
# 
# Where v = forward power, ω = turn power (proportional to heading error).
# No more pulse-stop-read-pulse. Continuous closed-loop control.
# ============================================================================

P5_HZ                = 20      # control loop rate, Hz (was effectively 3Hz with B6 pulses)
P5_KP_HEADING        = 0.8     # how aggressively to turn for each degree of error
P5_MAX_TURN_POWER    = 30      # cap on turn component
P5_MIN_FORWARD_POWER = 14      # don't go slower than this when not turning hard
P5_MAX_FORWARD_POWER = 30      # cap on forward component
P5_SLOWDOWN_DIST_MM  = 500     # start ramping down forward power within this distance
P5_TURN_ONLY_ERR_DEG = 35      # if heading error > this, stop forward and just turn
P5_ARRIVED_BUFFER_MM = 15      # extra buffer above stop threshold


def p5_clamp(v, lo, hi):
    return max(lo, min(hi, v))


def p5_compute_wheel_powers(target, mode):
    """Returns (left_power, right_power, status_string) or (None, None, status).
    
    Closed-loop: read marker angle + distance, compute desired (v, ω), convert to wheels.
    Status: 'ARRIVED' / 'BLOCKED' / 'LOST' / 'DRIVING'
    """
    m = marker_copy(target)
    if not m or not m.get("locked"):
        return None, None, "LOST"
    
    dist  = float(m["dist"])
    angle = float(m["angle"])
    err   = heading_error_for_mode(angle, mode)  # signed degrees, 0 = facing target
    
    stop_at = B6_HOME_STOP_RAW if target == "home" else B6_TARGET_STOP_RAW
    if dist <= stop_at:
        return 0, 0, "ARRIVED"
    
    with lock:
        front_raw = S.get("front_raw", 0)
    # Emergency stop: real obstacle, not just "haven\'t arrived yet"
    if mode == "forward" and front_raw and front_raw <= B6_CRITICAL_FRONT_RAW:
        if dist > stop_at + 50:
            return 0, 0, "BLOCKED"
    
    # === Proportional controller ===
    # ω component: P-controller on heading error
    omega_power = p5_clamp(P5_KP_HEADING * err, -P5_MAX_TURN_POWER, P5_MAX_TURN_POWER)
    
    # v component: ramp down as we approach target; zero out if turning hard
    if abs(err) > P5_TURN_ONLY_ERR_DEG:
        v_power = 0   # spin in place when heading is way off
    else:
        # Scale forward by how aligned we are
        align = 1.0 - (abs(err) / P5_TURN_ONLY_ERR_DEG)   # 1.0 = perfectly aligned
        # Distance-based slowdown
        if dist < P5_SLOWDOWN_DIST_MM:
            dist_scale = max(0.3, (dist - stop_at) / max(1, P5_SLOWDOWN_DIST_MM - stop_at))
        else:
            dist_scale = 1.0
        v_power = align * dist_scale * P5_MAX_FORWARD_POWER
        if v_power < P5_MIN_FORWARD_POWER and abs(err) < 15:
            v_power = P5_MIN_FORWARD_POWER   # don\'t stall close to target
    
    # Backward mode: flip v
    if mode == "backward":
        v_power = -v_power
    
    # Apply chassis sign (B6_FORWARD_POWER is positive on this chassis)
    # Already correct: positive v = forward.
    
    # === Diff-drive kinematics ===
    # v_left  = v - ω
    # v_right = v + ω
    # (omega sign: positive ω = turn LEFT = right wheel faster)
    # On this chassis, TURN_SIGN handles flips.
    omega_signed = omega_power * TURN_SIGN
    
    left_pwr  = int(p5_clamp(v_power - omega_signed, -40, 40))
    right_pwr = int(p5_clamp(v_power + omega_signed, -40, 40))
    
    return left_pwr, right_pwr, f"DRIVING err={err:+.1f}° dist={dist:.0f}mm v={v_power:.0f} w={omega_signed:+.0f}"


def p5_drive_to_marker(bot, target, timeout=MISSION_TIMEOUT_S):
    """Continuous closed-loop drive to marker at P5_HZ. Replaces pulse-based B6."""
    global stop_requested
    
    m0 = marker_copy(target)
    if not m0 or not m0.get("locked"):
        event(f"P5: {target.upper()} not locked", "e")
        return False
    
    mode = desired_drive_mode(target, float(m0["angle"]))
    start_dist = float(m0["dist"])
    event(f"P5 DRIVE→{target.upper()} mode={mode} start_dist={start_dist:.0f}mm", "k")
    
    t0 = time.time()
    dt = 1.0 / P5_HZ
    last_status = ""
    
    while time.time() - t0 < timeout:
        if stop_requested:
            bot.set_motor(0,0,0,0)
            event("P5: stop requested", "w")
            return False
        
        l, r, status = p5_compute_wheel_powers(target, mode)
        
        if status == "ARRIVED":
            bot.set_motor(0,0,0,0)
            event(f"P5 ARRIVED {target.upper()}", "k")
            return True
        if status == "BLOCKED":
            bot.set_motor(0,0,0,0)
            event(f"P5 BLOCKED en route to {target.upper()} (front gap too small)", "e")
            return False
        if status == "LOST":
            bot.set_motor(0,0,0,0)
            event(f"P5 LOST marker {target.upper()}", "e")
            return False
        
        # Send wheel commands directly
        bot.set_motor(l, l, r, r)   # M1,M2=left; M3,M4=right
        
        # Update mission status (throttled)
        if status != last_status:
            with lock:
                MISSION["message"] = f"P5 {target.upper()}: {status}"
            last_status = status
        
        time.sleep(dt)
    
    bot.set_motor(0,0,0,0)
    event(f"P5 timeout {target.upper()}", "e")
    return False


# ============================================================================
# Hijack drive_to_marker to use the P5 controller
# ============================================================================
# DEDUPED: _b6_drive_to_marker_original = drive_to_marker
# DEDUPED: def drive_to_marker(bot, target, timeout=MISSION_TIMEOUT_S):
# DEDUPED:     return p5_drive_to_marker(bot, target, timeout)


# ============================================================================
# PHASE 5: PURE PURSUIT CONTROLLER + DIFF-DRIVE DRIVER (Nav2 architecture)
# ============================================================================
P5_WHEELBASE_MM        = 170
P5_WHEEL_DIAM_MM       = 70
P5_POWER_PER_MPS       = 136
P5_HZ                  = 20
P5_REPLAN_HZ           = 1.0
P5_LOOKAHEAD_MM        = 300
P5_LOOKAHEAD_MIN_MM    = 200
P5_LOOKAHEAD_MAX_MM    = 500
P5_MAX_LIN_MPS         = 0.22
P5_MIN_LIN_MPS         = 0.10
P5_MAX_ANG_RADPS       = 2.0
P5_APPROACH_DIST_MM    = 500
P5_ROTATE_FIRST_DEG    = 45

import math as _math

def p5_clamp(v, lo, hi):
    return max(lo, min(hi, v))

def p5_pick_lookahead(path_pts, lookahead_mm):
    if not path_pts:
        return None
    for pt in path_pts:
        if _math.hypot(pt["x"], pt["y"]) >= lookahead_mm:
            return pt
    return path_pts[-1]

def p5_compute_velocity_command(target_name, mode):
    m = marker_copy(target_name)
    if not m or not m.get("locked"):
        return 0.0, 0.0, "LOST"
    dist_to_goal = float(m["dist"])
    stop_at = B6_HOME_STOP_RAW if target_name == "home" else B6_TARGET_STOP_RAW
    if dist_to_goal <= stop_at:
        return 0.0, 0.0, "ARRIVED"
    with lock:
        front_raw = S.get("front_raw", 0)
    if mode == "forward" and front_raw and front_raw <= B6_CRITICAL_FRONT_RAW:
        if dist_to_goal > stop_at + 50:
            return 0.0, 0.0, "BLOCKED"
    with lock:
        path_pts = list(S.get("route_path", []))
    if not path_pts or len(path_pts) < 2:
        return 0.0, 0.0, "NO_PATH"
    L_d = p5_clamp(P5_LOOKAHEAD_MM, P5_LOOKAHEAD_MIN_MM, P5_LOOKAHEAD_MAX_MM)
    lookahead = p5_pick_lookahead(path_pts, L_d)
    if lookahead is None:
        return 0.0, 0.0, "NO_PATH"
    lx, ly = lookahead["x"], lookahead["y"]
    alpha_rad = _math.atan2(lx, ly)
    alpha_deg = _math.degrees(alpha_rad)
    if abs(alpha_deg) > P5_ROTATE_FIRST_DEG:
        omega = -p5_clamp(alpha_rad * 2.0, -P5_MAX_ANG_RADPS, P5_MAX_ANG_RADPS)
        return 0.0, omega, "TURNING"
    Ld_mm = _math.hypot(lx, ly)
    if Ld_mm < 1:
        return 0.0, 0.0, "ARRIVED"
    curvature = -2.0 * _math.sin(alpha_rad) / (Ld_mm / 1000.0)
    if dist_to_goal < P5_APPROACH_DIST_MM:
        v_scale = (dist_to_goal - stop_at) / max(1, P5_APPROACH_DIST_MM - stop_at)
        v = P5_MIN_LIN_MPS + (P5_MAX_LIN_MPS - P5_MIN_LIN_MPS) * v_scale
    else:
        v = P5_MAX_LIN_MPS
    v = v / (1.0 + abs(curvature) * 0.5)
    v = p5_clamp(v, P5_MIN_LIN_MPS, P5_MAX_LIN_MPS)
    omega = p5_clamp(curvature * v, -P5_MAX_ANG_RADPS, P5_MAX_ANG_RADPS)
    if mode == "backward":
        v = -v
    return v, omega, f"DRIVING a={alpha_deg:+.0f} v={v:.2f} w={omega:+.2f}"

def p5_velocity_to_motor_powers(v_mps, omega_radps):
    L_m = P5_WHEELBASE_MM / 1000.0
    v_left_mps  = v_mps - omega_radps * L_m / 2.0
    v_right_mps = v_mps + omega_radps * L_m / 2.0
    left_power  = int(round(v_left_mps  * P5_POWER_PER_MPS))
    right_power = int(round(v_right_mps * P5_POWER_PER_MPS))
    left_power  = p5_clamp(left_power,  -40, 40)
    right_power = p5_clamp(right_power, -40, 40)
    if TURN_SIGN < 0:
        left_power, right_power = right_power, left_power
    return left_power, right_power

def p5_replan_loop(target_name, stop_flag):
    interval = 1.0 / P5_REPLAN_HZ
    while not stop_flag.is_set():
        try:
            plan_route(target_name)
        except Exception as e:
            event(f"P5 replan error: {e}", "e")
        for _ in range(int(interval * 10)):
            if stop_flag.is_set():
                return
            time.sleep(0.1)

def p5_pure_pursuit_drive(bot, target_name, timeout=MISSION_TIMEOUT_S):
    global stop_requested
    m0 = marker_copy(target_name)
    if not m0 or not m0.get("locked"):
        event(f"P5: {target_name.upper()} not locked", "e")
        return False
    mode = desired_drive_mode(target_name, float(m0["angle"]))
    start_dist = float(m0["dist"])
    event(f"P5 DRIVE->{target_name.upper()} mode={mode} start={start_dist:.0f}mm WB={P5_WHEELBASE_MM}mm Hz={P5_HZ}", "k")
    plan_route(target_name)
    time.sleep(0.1)
    replan_stop = threading.Event()
    replan_thread = threading.Thread(target=p5_replan_loop, args=(target_name, replan_stop), daemon=True)
    replan_thread.start()
    t0 = time.time()
    dt = 1.0 / P5_HZ
    last_status_logged = ""
    last_log_t = 0
    blocked_streak = 0
    no_path_streak = 0
    try:
        while time.time() - t0 < timeout:
            if stop_requested:
                bot.set_motor(0, 0, 0, 0)
                event("P5: stop requested", "w")
                return False
            v, omega, status = p5_compute_velocity_command(target_name, mode)
            if status == "ARRIVED":
                bot.set_motor(0, 0, 0, 0)
                event(f"P5 ARRIVED {target_name.upper()}", "k")
                return True
            if status == "LOST":
                bot.set_motor(0, 0, 0, 0)
                event(f"P5 LOST marker {target_name.upper()}", "e")
                return False
            if status == "BLOCKED":
                blocked_streak += 1
                bot.set_motor(0, 0, 0, 0)
                if blocked_streak >= int(P5_HZ * 2):
                    event(f"P5 BLOCKED for 2s", "e")
                    return False
                time.sleep(dt)
                continue
            else:
                blocked_streak = 0
            if status == "NO_PATH":
                no_path_streak += 1
                bot.set_motor(0, 0, 0, 0)
                if no_path_streak >= int(P5_HZ * 3):
                    event(f"P5 NO_PATH for 3s", "e")
                    return False
                time.sleep(dt)
                continue
            else:
                no_path_streak = 0
            left_pwr, right_pwr = p5_velocity_to_motor_powers(v, omega)
            bot.set_motor(left_pwr, left_pwr, right_pwr, right_pwr)
            now = time.time()
            if now - last_log_t > 0.5 and status != last_status_logged:
                with lock:
                    MISSION["message"] = f"P5 {target_name.upper()}: {status} L={left_pwr} R={right_pwr}"
                last_status_logged = status
                last_log_t = now
            # PHASE5 debug log every ~500ms to see what controller is doing
            if int(time.time() * 2) != int((time.time() - dt) * 2):
                event(f"PP {status} L={left_pwr} R={right_pwr}", "i")
            time.sleep(dt)
        bot.set_motor(0, 0, 0, 0)
        event(f"P5 timeout {target_name.upper()}", "e")
        return False
    finally:
        replan_stop.set()
        bot.set_motor(0, 0, 0, 0)

_b6_drive_original = drive_to_marker
def drive_to_marker(bot, target, timeout=MISSION_TIMEOUT_S):
    return p5_pure_pursuit_drive(bot, target, timeout)

# ============================================================================
# FPMS PHASE 5 — COMPLETE ENCODER + IMU + LIDAR MISSION SYSTEM
# ============================================================================
import math as _p5m

P5_WHEELBASE_MM     = 170
P5_WHEEL_DIAM_MM    = 70
P5_TICKS_PER_REV    = 1320
P5_MM_PER_TICK      = _p5m.pi * P5_WHEEL_DIAM_MM / P5_TICKS_PER_REV
P5_DRIVE_PWR        = 26
P5_TURN_PWR         = 32
P5_FINE_PWR         = 16
P5_HZ               = 20
P5_COAST            = 0.87
P5_HEADING_GAIN     = 10
P5_FACE_TOL_DEG     = 8
P5_LOOKAHEAD_MM     = 280
P5_REPLAN_S         = 1.0
P5_FRONT_STOP_MM    = 120

def p5_enc(bot):
    try: return bot.get_motor_encoder()
    except: return None

def p5_gz(bot):
    try:
        g = bot.get_gyroscope_data()
        return float(g[2]) if g else 0.0
    except: return 0.0

def p5_front():
    with lock:
        f = S.get("front_raw", 0)
    return int(f) if f else 9999

def p5_stop(bot):
    try: bot.set_motor(0,0,0,0)
    except: pass

def p5_rotate(bot, deg_target, timeout=6.0):
    global stop_requested
    if abs(deg_target) < 1.5: return 0.0
    rad_target = abs(_p5m.radians(deg_target))
    raw_dir = 1 if deg_target > 0 else -1
    spin = raw_dir * TURN_SIGN
    p = P5_TURN_PWR
    if spin > 0: bot.set_motor(-p,-p,+p,+p)
    else:        bot.set_motor(+p,+p,-p,-p)
    integrated = 0.0
    last_t = time.time(); t0 = last_t
    while time.time()-t0 < timeout:
        if stop_requested: p5_stop(bot); return raw_dir*_p5m.degrees(integrated)
        gz = abs(p5_gz(bot))
        now = time.time(); dt = now-last_t; last_t = now
        if 0 < dt < 0.5: integrated += gz*dt
        if integrated >= rad_target*P5_COAST: break
        time.sleep(1.0/P5_HZ)
    p5_stop(bot); time.sleep(0.12)
    return raw_dir*_p5m.degrees(integrated)

def p5_drive_segment(bot, target_mm, target_name, timeout=10.0):
    global stop_requested
    e0 = p5_enc(bot)
    if not e0: return 0.0,'NO_ENCODER'
    stop_at = B6_HOME_STOP_RAW if target_name=="home" else B6_TARGET_STOP_RAW
    heading_integral = 0.0
    last_t = time.time(); t0 = last_t
    while time.time()-t0 < timeout:
        if stop_requested: p5_stop(bot); return 0.0,'STOP'
        e = p5_enc(bot)
        mm_driven = (sum(abs(e[i]-e0[i]) for i in range(4))/4.0*P5_MM_PER_TICK) if e else 0.0
        m = marker_copy(target_name)
        if m and m.get("locked") and float(m["dist"])<=stop_at:
            p5_stop(bot); return mm_driven,'ARRIVED'
        front = p5_front()
        if front < P5_FRONT_STOP_MM: p5_stop(bot); return mm_driven,'BLOCKED'
        if mm_driven >= target_mm: p5_stop(bot); return mm_driven,'DONE'
        now = time.time(); dt = now-last_t; last_t = now
        if 0 < dt < 0.5: heading_integral += p5_gz(bot)*dt
        corr = max(-6,min(6,int(heading_integral*P5_HEADING_GAIN*TURN_SIGN)))
        bot.set_motor(P5_DRIVE_PWR-corr, P5_DRIVE_PWR-corr, P5_DRIVE_PWR+corr, P5_DRIVE_PWR+corr)
        with lock: MISSION["message"] = f"P5 {target_name.upper()} {mm_driven:.0f}/{target_mm:.0f}mm front={front}"
        time.sleep(1.0/P5_HZ)
    p5_stop(bot); return mm_driven,'TIMEOUT'

def p5_follow_route(bot, target_name, timeout=55.0):
    global stop_requested
    event(f"P5 FOLLOW->{target_name.upper()} enc+gyro+lidar WB={P5_WHEELBASE_MM}mm","k")
    t0 = time.time(); last_replan = -999.0; blocked_streak = 0
    while time.time()-t0 < timeout:
        if stop_requested: p5_stop(bot); return False
        m = marker_copy(target_name)
        if not m or not m.get("locked"): event(f"P5 LOST {target_name.upper()}","e"); return False
        dist = float(m["dist"])
        stop_at = B6_HOME_STOP_RAW if target_name=="home" else B6_TARGET_STOP_RAW
        if dist <= stop_at: p5_stop(bot); event(f"P5 ARRIVED {target_name.upper()} {dist:.0f}mm","k"); return True
        if time.time()-last_replan >= P5_REPLAN_S:
            plan_route(target_name); last_replan = time.time(); time.sleep(0.08)
        with lock: path = list(S.get("route_path",[]))
        lookahead = None
        for pt in path:
            if _p5m.hypot(pt["x"],pt["y"]) >= P5_LOOKAHEAD_MM: lookahead=pt; break
        if lookahead is None:
            lookahead = path[-1] if path else {"x":_p5m.sin(_p5m.radians(float(m["angle"])))*dist,"y":_p5m.cos(_p5m.radians(float(m["angle"])))*dist}
        lx,ly = lookahead["x"],lookahead["y"]
        ld = _p5m.hypot(lx,ly)
        bearing = _p5m.degrees(_p5m.atan2(lx,ly))
        event(f"P5 WP bearing={bearing:+.0f}° ld={ld:.0f}mm marker={dist:.0f}mm","i")
        if abs(bearing) > P5_FACE_TOL_DEG:
            rot = p5_rotate(bot, -bearing)
            event(f"P5 rotated {rot:+.0f}°","i")
        seg = max(100, min(ld*0.80, 400))
        mm,reason = p5_drive_segment(bot, seg, target_name)
        event(f"P5 drove {mm:.0f}mm reason={reason}","i")
        if reason=='ARRIVED': return True
        if reason in ('BLOCKED','STOP'):
            blocked_streak+=1
            if blocked_streak>=3: event("P5 permanently blocked","e"); return False
            last_replan=-999.0; time.sleep(0.2); continue
        blocked_streak=0; time.sleep(0.05)
    p5_stop(bot); event(f"P5 timeout {target_name.upper()}","e"); return False

def p5_run_mission(target):
    global mission_active,stop_requested,auto_cooldown_until
    mission_active=True; stop_requested=False
    auto_cooldown_until=time.time()+AUTO_COOLDOWN_S
    with lock:
        MISSION.update({"state":"RUNNING","target":target.upper(),"message":"P5 starting",
                        "last_error":"","started_at":time.time(),"finished_at":0,"moved_mm":0})
    bot=None
    try:
        if not robot_enabled: raise RuntimeError("ROBOT is OFF")
        mt=marker_copy(target)
        if not mt or not mt.get("locked"): raise RuntimeError(f"{target.upper()} not locked")
        if target!="home":
            mh=marker_copy("home")
            if not mh or not mh.get("locked"): raise RuntimeError("HOME not locked")
        with lock: front=S.get("front_raw",0)
        _p5_odom_reset()
        event(f"P5 PRE-FLIGHT {target.upper()}: front={front}mm dist={mt['dist']:.0f}mm angle={mt['angle']:.0f}°","k")
        bot=open_bot(); p5_stop(bot); time.sleep(0.3)
        with lock: MISSION["state"]="DRIVING"
        if not p5_follow_route(bot,target): raise RuntimeError("Could not reach target")
        if target!="home":
            with lock: MISSION.update({"state":"AT_TARGET","message":f"Spraying {target.upper()}"})
            spray_step(bot)
            with lock: MISSION.update({"state":"RETURNING","message":"P5 returning HOME"})
            if not p5_follow_route(bot,"home"): raise RuntimeError("Could not reach HOME")
        p5_stop(bot)
        with lock: MISSION.update({"state":"DONE","message":"P5 mission complete","finished_at":time.time()})
        event("P5 mission complete","k")
    except Exception as ex:
        p5_stop(bot)
        with lock: MISSION.update({"state":"ERROR","message":str(ex),"last_error":str(ex),"finished_at":time.time()})
        event(f"P5 mission error: {ex}","e")
    finally:
        try:
            if bot: pump_off(bot)
        except: pass
        p5_stop(bot); mission_active=False
        auto_cooldown_until=time.time()+AUTO_COOLDOWN_S

def start_mission(target):
    global mission_active,stop_requested
    if mission_active: return False,"Mission already running"
    if target not in ["m1","m2","home"]: return False,"Bad target"
    stop_requested=False; mission_active=True
    threading.Thread(target=p5_run_mission,args=(target,),daemon=True).start()
    return True,f"P5 started {target.upper()}"


# ============================================================================
# LAYER 6: MULTI-TARGET MISSION QUEUE + RETRACE STACK
# ============================================================================
_retrace_stack = []
_mission_queue = []

def p5_run_mission_v2(initial_target):
    global mission_active, stop_requested, auto_cooldown_until
    mission_active = True; stop_requested = False
    auto_cooldown_until = time.time() + AUTO_COOLDOWN_S
    _retrace_stack.clear(); _mission_queue.clear(); _p5_retrace.clear()
    _p5_odom_reset()
    with lock: S["route_path"] = []; S["obstacles"] = []
    if initial_target != "home": _mission_queue.append(initial_target)
    with lock:
        MISSION.update({"state":"RUNNING","target":initial_target.upper(),
            "message":"P5 multi-target starting","last_error":"",
            "started_at":time.time(),"finished_at":0,"moved_mm":0})
    bot = None
    try:
        if not robot_enabled: raise RuntimeError("ROBOT is OFF")
        mh = marker_copy("home")
        if not mh or not mh.get("locked"): raise RuntimeError("HOME not locked")
        m1c = marker_copy("m1")
        m2c = marker_copy("m2")
        if not m1c or not m1c.get("locked"): event("WARNING: M1 not locked","w")
        if not m2c or not m2c.get("locked"): event("WARNING: M2 not locked","w")
        bot = open_bot()
        try: bot.set_motor(0,0,0,0)
        except: pass
        time.sleep(0.3)
        targets_sprayed = []
        while not stop_requested:
            if not _mission_queue: break
            target = _mission_queue.pop(0)
            if target == "home": continue
            mt = marker_copy(target)
            if not mt or not mt.get("locked"):
                event(f"P5: {target.upper()} not locked — skipping","w"); continue
            with lock: front = S.get("front_raw",0)
            event(f"P5 PRE-FLIGHT {target.upper()}: front={front}mm dist={mt['dist']:.0f}mm angle={mt['angle']:.0f}°","k")
            with lock: MISSION.update({"state":"DRIVING","target":target.upper(),"message":f"→ {target.upper()}"})
            _p5_retrace.clear()
            _active_markers.clear(); _active_markers.update({target, "home"})
            if not _p5_navdrive(bot, target):
                event(f"P5 could not reach {target.upper()}","e"); continue
            leg = list(_p5_retrace); _retrace_stack.append(leg); _p5_retrace.clear()
            event(f"P5 saved leg {len(_retrace_stack)} ({len(leg)} segs) for retrace","k")
            with lock: MISSION.update({"state":"AT_TARGET","message":f"Spraying {target.upper()}"})
            spray_step(bot)
            targets_sprayed.append(target)
            event(f"P5 sprayed {target.upper()} (done:{[t.upper() for t in targets_sprayed]} queue:{[t.upper() for t in _mission_queue]})","k")
            time.sleep(0.5)
        if stop_requested: raise RuntimeError("STOPPED by user")
        if _retrace_stack:
            with lock: MISSION.update({"state":"RETURNING","message":f"Returning HOME ({len(_retrace_stack)} legs)"})
            leg_count = len(_retrace_stack)
            while _retrace_stack and not stop_requested:
                leg = _retrace_stack.pop()
                leg_idx = leg_count - len(_retrace_stack)
                event(f"P5 RETRACE leg {leg_idx}/{leg_count} ({len(leg)} segs bwd)","k")
                _p5_retrace.clear(); _p5_retrace.extend(leg)
                _p5_navdrive(bot, "home")
            if stop_requested: raise RuntimeError("STOPPED during return")
        try: bot.set_motor(0,0,0,0)
        except: pass
        with lock: MISSION.update({"state":"DONE","message":f"Complete ({', '.join(t.upper() for t in targets_sprayed)})","finished_at":time.time()})
        event(f"P5 MISSION COMPLETE: {[t.upper() for t in targets_sprayed]}","k")
    except Exception as ex:
        try: bot.set_motor(0,0,0,0)
        except: pass
        with lock: MISSION.update({"state":"ERROR","message":str(ex),"last_error":str(ex),"finished_at":time.time()})
        event(f"P5 mission error: {ex}","e")
    finally:
        try:
            if bot: pump_off(bot)
        except: pass
        try:
            if bot: bot.set_motor(0,0,0,0)
        except: pass
        mission_active = False; _retrace_stack.clear(); _mission_queue.clear(); _p5_retrace.clear(); _active_markers.clear(); _active_markers.clear(); _active_markers.clear()
        auto_cooldown_until = time.time() + AUTO_COOLDOWN_S

def start_mission(target):
    global mission_active, stop_requested
    if mission_active: return False, "Mission already running"
    if target not in ["m1","m2","home"]: return False, "Bad target"
    stop_requested = False; mission_active = True
    threading.Thread(target=p5_run_mission_v2, args=(target,), daemon=True).start()
    return True, f"P5 started {target.upper()}"

#!/usr/bin/env python3
import math, os, glob, struct, threading, time, heapq, json
import serial
from flask import Flask, jsonify, request, render_template_string

PORT_FIXED = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
BAUD = 230400
WEB_PORT = 8085
ESP_STATUS_FILE = "/tmp/fpms_esp_status.json"

WORLD_W = 2000
WORLD_H = 2000

ROBOT_W = 230
ROBOT_RADIUS = ROBOT_W / 2.0
LIDAR_TO_FRONT = 200

FRONT_STOP_RAW = 300
TARGET_STOP_RAW = 250

GRID_RES = 25
SAFETY_CLEARANCE = 60
ROUTE_RADIUS = ROBOT_RADIUS + SAFETY_CLEARANCE

START_IGNORE_RADIUS = 190
TARGET_EXCLUDE_RADIUS = 230
MAX_LIDAR_MM = 2400

FRONT_DEG = 25
MARKER_CLICK_SNAP_MM = 180
MARKER_RELOCK_MM = 300

STATE_UPDATE_S = 0.12
UI_POINT_LIMIT = 360

YAHBOOM_TEST_SPEED = 0.12
YAHBOOM_TEST_SECONDS = 2.0

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

robot_mode = False
robot_busy = False
robot_text = "Not tested yet."
robot_error = ""

S = {
    "status": "BOOT",
    "port": "",
    "raw_bps": 0,
    "headers": 0,
    "ok": 0,
    "bad": 0,
    "scan_hz": 0,
    "points": 0,
    "front_raw": 0,
    "front_gap": 0,
    "temp": 0,
    "err": "",
    "markers": {"m1": None, "m2": None, "home": None},
    "marker_info": {"m1": None, "m2": None, "home": None},
    "ui_scan": [],
    "route": {
        "state": "IDLE",
        "message": "Set markers, then press MAP R1 / MAP R2 / MAP HOME.",
        "path": [],
        "blocked": [],
        "target": "",
        "type": ""
    },
    "events": []
}

def event(msg, typ="i"):
    print(f"[{typ}] {msg}", flush=True)
    with lock:
        S["events"].append({"t": time.strftime("%H:%M:%S"), "msg": msg, "typ": typ})
        S["events"] = S["events"][-20:]

def find_port():
    if os.path.exists(PORT_FIXED):
        return PORT_FIXED
    for p in ["/dev/serial/by-id/*CP210*", "/dev/serial/by-id/*USB*", "/dev/ttyUSB*", "/dev/ttyACM*"]:
        m = sorted(glob.glob(p))
        if m:
            return m[0]
    return PORT_FIXED

def temp_c():
    for p in ["/sys/class/thermal/thermal_zone0/temp", "/sys/class/thermal/thermal_zone1/temp"]:
        try:
            v = float(open(p).read().strip())
            return round(v / 1000.0 if v > 1000 else v, 1)
        except Exception:
            pass
    return 0

def deg_xy(deg, dist):
    a = math.radians(deg)
    return math.sin(a) * dist, math.cos(a) * dist

def xy_deg_dist(x, y):
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0, math.hypot(x, y)

def angle_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)

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
        port = find_port()

        with lock:
            S["port"] = port
            S["status"] = "OPENING"
            S["err"] = ""

        try:
            ser = serial.Serial(port, BAUD, timeout=0.25)
            event(f"D500 opened: {port}", "k")

            with lock:
                S["status"] = "SERIAL OPEN"

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

                with lock:
                    S["headers"] += 1

                rest = ser.read(45)
                raw_bytes += len(rest)

                if len(rest) != 45:
                    continue

                pkt = bytes([0x54, 0x2C]) + rest

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
            with lock:
                S["status"] = "ERROR"
                S["err"] = str(e)
            event(f"LiDAR error: {e}", "e")
            time.sleep(1)

def scan_points(all_points=True):
    with lock:
        sc = list(disp_scan)

    pts = []

    for deg, v in enumerate(sc):
        if not v:
            continue

        d, inten = v
        x, y = deg_xy(deg, d)

        if -WORLD_W / 2 <= x <= WORLD_W / 2 and -WORLD_H / 2 <= y <= WORLD_H / 2:
            pts.append({
                "deg": deg,
                "d": d,
                "i": inten,
                "x": round(x, 1),
                "y": round(y, 1)
            })

    if all_points or len(pts) <= UI_POINT_LIMIT:
        return pts

    pts = sorted(pts, key=lambda p: p["deg"])
    step = len(pts) / UI_POINT_LIMIT
    out, idx = [], 0.0

    while int(idx) < len(pts) and len(out) < UI_POINT_LIMIT:
        out.append(pts[int(idx)])
        idx += step

    return out

def nearest_point(x, y, pts, max_mm):
    best = None
    bd = max_mm

    for p in pts:
        e = math.hypot(p["x"] - x, p["y"] - y)
        if e < bd:
            best = p
            bd = e

    return best

def make_cluster(seed, pts):
    members = []

    for p in pts:
        if angle_diff(p["deg"], seed["deg"]) <= 8 and abs(p["d"] - seed["d"]) <= 180:
            if math.hypot(p["x"] - seed["x"], p["y"] - seed["y"]) <= 190:
                members.append(p)

    if not members:
        members = [seed]

    cx = sum(p["x"] for p in members) / len(members)
    cy = sum(p["y"] for p in members) / len(members)
    a, d = xy_deg_dist(cx, cy)

    return {
        "x": round(cx, 1),
        "y": round(cy, 1),
        "angle": round(a, 1),
        "dist": round(d, 1),
        "front_gap": round(d - LIDAR_TO_FRONT, 1),
        "points": len(members),
        "locked": True,
        "last_seen": time.time()
    }

def marker_info(m):
    if not m:
        return None

    return {
        "locked": bool(m.get("locked")),
        "x": m["x"],
        "y": m["y"],
        "angle": m["angle"],
        "dist": m["dist"],
        "front_gap": m["front_gap"],
        "points": m["points"]
    }

def xy_cell(x, y):
    return int(round((x + WORLD_W / 2) / GRID_RES)), int(round((y + WORLD_H / 2) / GRID_RES))

def cell_xy(c, r):
    return c * GRID_RES - WORLD_W / 2, r * GRID_RES - WORLD_H / 2

def grid_size():
    return int(WORLD_W / GRID_RES) + 1, int(WORLD_H / GRID_RES) + 1

def point_seg_dist(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    denom = vx * vx + vy * vy

    if denom < 1e-6:
        return math.hypot(px - ax, py - ay)

    t = max(0, min(1, (wx * vx + wy * vy) / denom))
    cx, cy = ax + t * vx, ay + t * vy
    return math.hypot(px - cx, py - cy)

def target_stop_goal(marker):
    d = math.hypot(marker["x"], marker["y"])

    if d <= TARGET_STOP_RAW + 30:
        return None

    scale = (d - TARGET_STOP_RAW) / d
    return marker["x"] * scale, marker["y"] * scale

def planning_obstacles(pts, target_marker):
    obs = []

    for p in pts:
        x, y = p["x"], p["y"]

        if math.hypot(x, y) < START_IGNORE_RADIUS:
            continue

        if -ROBOT_W / 2 - 60 <= x <= ROBOT_W / 2 + 60 and -140 <= y <= LIDAR_TO_FRONT + 90:
            continue

        if target_marker and math.hypot(x - target_marker["x"], y - target_marker["y"]) < TARGET_EXCLUDE_RADIUS:
            continue

        obs.append(p)

    return obs

def check_straight(start, goal, obstacles):
    ax, ay = start
    bx, by = goal
    blocked = []

    for p in obstacles:
        if point_seg_dist(p["x"], p["y"], ax, ay, bx, by) <= ROUTE_RADIUS:
            blocked.append({"x": p["x"], "y": p["y"]})
            if len(blocked) >= 40:
                break

    return len(blocked) > 0, blocked

def build_costmap(obstacles):
    cols, rows = grid_size()
    occ = [[False for _ in range(cols)] for _ in range(rows)]
    rad = int(math.ceil(ROUTE_RADIUS / GRID_RES))

    for p in obstacles:
        c0, r0 = xy_cell(p["x"], p["y"])

        for dr in range(-rad, rad + 1):
            r = r0 + dr
            if r < 0 or r >= rows:
                continue

            for dc in range(-rad, rad + 1):
                c = c0 + dc
                if c < 0 or c >= cols:
                    continue

                if math.hypot(dc * GRID_RES, dr * GRID_RES) <= ROUTE_RADIUS:
                    occ[r][c] = True

    sc, sr = xy_cell(0, 0)
    clear = int(math.ceil(START_IGNORE_RADIUS / GRID_RES))

    for dr in range(-clear, clear + 1):
        for dc in range(-clear, clear + 1):
            r, c = sr + dr, sc + dc
            if 0 <= r < rows and 0 <= c < cols and math.hypot(dc * GRID_RES, dr * GRID_RES) <= START_IGNORE_RADIUS:
                occ[r][c] = False

    return occ

def nearest_free(occ, cell, max_rad=10):
    cols, rows = grid_size()
    c0, r0 = cell

    if 0 <= c0 < cols and 0 <= r0 < rows and not occ[r0][c0]:
        return cell

    for rad in range(1, max_rad + 1):
        best = None
        bd = 1e9

        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                if abs(dc) != rad and abs(dr) != rad:
                    continue

                c, r = c0 + dc, r0 + dr

                if 0 <= c < cols and 0 <= r < rows and not occ[r][c]:
                    dd = dc * dc + dr * dr
                    if dd < bd:
                        best = (c, r)
                        bd = dd

        if best:
            return best

    return None

def astar(occ, start_xy, goal_xy):
    cols, rows = grid_size()
    start = nearest_free(occ, xy_cell(*start_xy), 10)
    goal = nearest_free(occ, xy_cell(*goal_xy), 12)

    if not start or not goal:
        return None

    def h(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    openq = [(h(start, goal), 0, start)]
    came = {}
    gscore = {start: 0}
    closed = set()
    moves = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while openq:
        _, g, cur = heapq.heappop(openq)

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

            step = math.sqrt(2) if dc and dr else 1
            ng = g + step

            if ng < gscore.get(nb, 1e18):
                gscore[nb] = ng
                came[nb] = cur
                heapq.heappush(openq, (ng + h(nb, goal), ng, nb))

    return None

def line_free(occ, a, b):
    ax, ay = a
    bx, by = b
    d = math.hypot(bx - ax, by - ay)
    steps = max(1, int(d / (GRID_RES / 2)))

    cols, rows = grid_size()

    for i in range(steps + 1):
        t = i / steps
        x = ax + (bx - ax) * t
        y = ay + (by - ay) * t
        c, r = xy_cell(x, y)

        if c < 0 or c >= cols or r < 0 or r >= rows or occ[r][c]:
            return False

    return True

def smooth_path(path, occ):
    if not path or len(path) <= 2:
        return path or []

    out = [path[0]]
    i = 0

    while i < len(path) - 1:
        j = len(path) - 1

        while j > i + 1:
            if line_free(occ, path[i], path[j]):
                break
            j -= 1

        out.append(path[j])
        i = j

    return out

def plan_to(marker_name):
    pts = scan_points(all_points=True)

    with lock:
        marker = S["markers"].get(marker_name)

    if not marker or not marker.get("locked"):
        return {"ok": False, "msg": f"{marker_name.upper()} is not locked.", "path": [], "blocked": []}

    goal = target_stop_goal(marker)

    if goal is None:
        return {
            "ok": True,
            "msg": f"{marker_name.upper()} already within target stop distance.",
            "path": [],
            "blocked": [],
            "type": "AT TARGET",
            "target": marker_name
        }

    obs = planning_obstacles(pts, marker)
    blocked, blocked_pts = check_straight((0, 0), goal, obs)

    if not blocked:
        path = [(0, 0), goal]
        typ = "STRAIGHT CLEAR"
    else:
        occ = build_costmap(obs)
        raw = astar(occ, (0, 0), goal)

        if not raw:
            return {
                "ok": False,
                "msg": "BLOCKED: no safe inflated A* path found.",
                "path": [],
                "blocked": blocked_pts,
                "type": "NO PATH",
                "target": marker_name
            }

        path = smooth_path(raw, occ)
        typ = "REROUTE READY"

    path_json = [{"x": round(x, 1), "y": round(y, 1)} for x, y in path]

    msg = f"{typ}: {len(path_json)} waypoint(s). Robot corridor width = {int(ROUTE_RADIUS*2)}mm."

    return {
        "ok": True,
        "msg": msg,
        "path": path_json,
        "blocked": blocked_pts,
        "type": typ,
        "target": marker_name
    }

def metrics_loop():
    while True:
        pts_full = scan_points(all_points=True)
        pts_ui = scan_points(all_points=False)

        front = [p["d"] for p in pts_full if p["deg"] <= FRONT_DEG or p["deg"] >= 360 - FRONT_DEG]
        raw = min(front) if front else 0

        with lock:
            S["points"] = len(pts_full)
            S["ui_scan"] = [[p["x"], p["y"], p["i"]] for p in pts_ui]
            S["front_raw"] = int(raw)
            S["front_gap"] = int(raw - LIDAR_TO_FRONT) if raw else 0
            S["temp"] = temp_c()
            marker_copy = dict(S["markers"])

        updates = {}

        for name, m in marker_copy.items():
            if not m:
                continue

            near = nearest_point(m["x"], m["y"], pts_full, MARKER_RELOCK_MM)

            if near:
                updates[name] = make_cluster(near, pts_full)
            else:
                lost = dict(m)
                lost["locked"] = False
                updates[name] = lost

        if updates:
            with lock:
                for k, v in updates.items():
                    S["markers"][k] = v

        with lock:
            S["marker_info"] = {k: marker_info(S["markers"].get(k)) for k in ["m1", "m2", "home"]}

        time.sleep(STATE_UPDATE_S)

def yahboom_worker():
    global robot_busy, robot_text, robot_error

    try:
        robot_text = "Starting Yahboom test..."
        robot_error = ""
        event("Yahboom test: low speed for 2 seconds.", "w")

        from Rosmaster_Lib import Rosmaster
        bot = Rosmaster()

        try:
            bot.set_car_motion(YAHBOOM_TEST_SPEED, 0, 0)
            time.sleep(YAHBOOM_TEST_SECONDS)
            bot.set_car_motion(0, 0, 0)
            robot_text = "Yahboom set_car_motion test complete."
            event(robot_text, "k")
            return
        except Exception as e:
            event(f"set_car_motion failed; trying set_motor. {e}", "w")

        if hasattr(bot, "set_motor"):
            bot.set_motor(20, 20, 20, 20)
            time.sleep(YAHBOOM_TEST_SECONDS)
            bot.set_motor(0, 0, 0, 0)
            robot_text = "Yahboom set_motor fallback test complete."
            event(robot_text, "k")
            return

        raise RuntimeError("No usable Rosmaster motor function found.")

    except Exception as e:
        robot_error = str(e)
        robot_text = "Yahboom test failed."
        event(f"Yahboom test failed: {e}", "e")

    finally:
        robot_busy = False


def read_esp_status():
    default = {
        "n1_online": False,
        "n2_online": False,
        "alert1": False,
        "alert2": False,
        "last_line": "",
        "last_alert": "",
        "last_update": 0,
        "age_s": 999
    }

    try:
        with open(ESP_STATUS_FILE, "r") as f:
            data = json.load(f)

        now = time.time()
        data["age_s"] = round(now - float(data.get("last_update", 0)), 1)

        # If bridge data is very old, mark nodes offline but keep last line visible.
        if data["age_s"] > 8:
            data["n1_online"] = False
            data["n2_online"] = False

        return {**default, **data}
    except Exception:
        return default

app = Flask(__name__)

PAGE = r"""
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>FPMS A3</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
*{box-sizing:border-box}
:root{--bg:#020711;--panel:#071629;--line:#25506f;--cyan:#00b7ff;--pink:#ff4b70;--yellow:#ffbf00;--green:#3af07a;--soft:#98d4ff}
body{margin:0;background:radial-gradient(circle at top,#0b2440,#020711 55%);color:#e8f7ff;font-family:Consolas,Menlo,monospace;height:100vh;overflow:hidden}
#top{height:62px;background:rgba(2,7,17,.96);border-bottom:2px solid var(--pink);display:flex;align-items:center;justify-content:space-between;padding:9px 16px}
#title{color:var(--pink);font-size:22px;font-weight:900;letter-spacing:5px}.sub{font-size:12px;color:var(--soft);margin-top:3px}#clock{font-size:18px;font-weight:900}
#main{height:calc(100vh - 62px);display:grid;grid-template-columns:minmax(820px,1fr) 390px;gap:8px;padding:8px}
.panel{background:rgba(7,22,41,.94);border:1px solid var(--line);border-radius:14px;padding:12px;min-height:0;box-shadow:0 0 24px rgba(0,183,255,.08)}
.h{color:var(--cyan);letter-spacing:3px;font-size:13px;font-weight:900;border-bottom:1px solid var(--line);padding-bottom:8px;margin-bottom:10px}
#mapbox{position:relative;height:calc(100% - 34px);width:100%;overflow:hidden;background:#020711;border:1px solid #173450;border-radius:12px}
canvas{position:absolute;left:0;top:0;width:100%;height:100%}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:7px}.grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:7px}
button{background:#071427;border:1px solid #2e76aa;color:#edf8ff;border-radius:10px;padding:9px 6px;font-family:Consolas;font-weight:900;cursor:pointer;min-height:38px}button:hover{background:#102846}button.active{border-color:var(--yellow);color:var(--yellow);background:#211802}button.danger{border-color:var(--pink);color:var(--pink)}button.green{border-color:var(--green);color:var(--green)}button.yellow{border-color:var(--yellow);color:var(--yellow)}
.card{background:#020711;border:1px solid #1d4163;border-radius:10px;text-align:center;padding:7px}.lbl{font-size:10px;color:#82b0d0}.val{font-size:18px;font-weight:900;color:var(--cyan);line-height:1.2}.unit{font-size:9px;color:#6f92ad}
.marker{background:#020711;border:1px solid #1d4163;border-radius:10px;padding:8px;margin-top:7px}.markerTop{display:flex;justify-content:space-between}.locked{color:var(--green)}.lost{color:var(--pink)}.empty{color:#8eabc7}.small{font-size:10px;color:#9ec7e5;margin-top:3px}
.msg{font-size:11px;color:#e8f7ff;background:#020711;border:1px solid #1d4163;border-radius:10px;padding:8px;margin-top:7px;min-height:46px}
#events{height:82px;overflow:auto;background:#020711;border:1px solid #1d4163;border-radius:10px;padding:6px;font-size:10px}.ei{color:var(--soft)}.ek{color:var(--green)}.ew{color:var(--yellow)}.ee{color:var(--pink)}#err{font-size:10px;color:var(--pink);overflow-wrap:anywhere}
</style>
</head>
<body>
<div id="top"><div><div id="title">FPMS A3 PLANNER</div><div class="sub">200cm × 200cm fixed LiDAR coverage • ROS-style costmap + inflation + A* reroute • no autozoom</div></div><div id="clock"></div></div>
<div id="main">
<div class="panel"><div class="h">LIVE MAP — 200cm × 200cm COVERAGE, ROBOT-WIDTH ROUTING</div><div id="mapbox"><canvas id="grid"></canvas><canvas id="dyn"></canvas></div></div>
<div class="panel">
<div class="h">MARKERS</div><div class="grid2"><button id="b_m1" onclick="setMode('m1')">SET M1</button><button id="b_m2" onclick="setMode('m2')">SET M2</button><button id="b_home" onclick="setMode('home')">SET HOME</button><button class="danger" onclick="clearAll()">CLEAR</button></div>
<div class="h" style="margin-top:12px">REAL ROUTE PLANNER</div><div class="grid2"><button onclick="planRoute('m1')">MAP R1</button><button onclick="planRoute('m2')">MAP R2</button><button class="yellow" onclick="planRoute('home')">MAP HOME</button><button class="danger" onclick="clearRoute()">CLEAR ROUTE</button></div><div id="routeMsg" class="msg">Waiting for route.</div>

<div class="h" style="margin-top:12px">ESP-NOW ALERTS</div>
<div class="grid2">
  <div class="card"><div class="lbl">NODE 1</div><div id="espn1" class="val">--</div></div>
  <div class="card"><div class="lbl">ALERT 1</div><div id="espa1" class="val">--</div></div>
  <div class="card"><div class="lbl">NODE 2</div><div id="espn2" class="val">--</div></div>
  <div class="card"><div class="lbl">ALERT 2</div><div id="espa2" class="val">--</div></div>
</div>
<div id="espLine" class="msg">Waiting for ESP bridge...</div>
<div class="h" style="margin-top:12px">YAHBOOM TEST</div><div class="grid2"><button id="robotBtn" class="danger" onclick="toggleRobot()">ROBOT MODE OFF</button><button class="green" onclick="testYahboom()">TEST 2s</button></div><div id="robotText" class="small">Optional. Wheels off ground.</div>
<div class="h" style="margin-top:12px">LIVE DISTANCES</div><div id="m1" class="marker"></div><div id="m2" class="marker"></div><div id="home" class="marker"></div>
<div class="h" style="margin-top:12px">DIAGNOSTICS</div><div class="grid2"><div class="card"><div class="lbl">STATUS</div><div id="status" class="val">--</div></div><div class="card"><div class="lbl">SCAN</div><div id="hz" class="val">0</div><div class="unit">Hz</div></div><div class="card"><div class="lbl">RAW BPS</div><div id="bps" class="val">0</div></div><div class="card"><div class="lbl">POINTS</div><div id="points" class="val">0</div></div><div class="card"><div class="lbl">FRONT RAW</div><div id="frontraw" class="val">--</div></div><div class="card"><div class="lbl">FRONT GAP</div><div id="frontgap" class="val">--</div></div><div class="card"><div class="lbl">OK</div><div id="ok" class="val">0</div></div><div class="card"><div class="lbl">BAD CRC</div><div id="bad" class="val">0</div></div></div>
<div class="h" style="margin-top:12px">EVENTS</div><div id="events"></div><div id="err"></div>
</div></div>
<script>
const WORLD_W=2000,WORLD_H=2000,ROBOT_W=230,LIDAR_FRONT=200,FRONT_STOP=300,TARGET_STOP=250,ROUTE_RADIUS=175;
let scale=1,ox=0,oy=0,mode=null,scan=[],markers={},routePath=[],blockedPts=[];
const box=document.getElementById('mapbox'),grid=document.getElementById('grid'),dyn=document.getElementById('dyn'),g=grid.getContext('2d'),d=dyn.getContext('2d');
function $(id){return document.getElementById(id)}
function resizeCanvas(){
  const r=box.getBoundingClientRect();
  grid.width=dyn.width=Math.floor(r.width);
  grid.height=dyn.height=Math.floor(r.height);
  scale=Math.min(grid.width/WORLD_W,grid.height/WORLD_H)*0.96;
  ox=grid.width/2;oy=grid.height/2;
  drawGrid();drawDyn();
}
function mmToPix(x,y){return [ox+x*scale,oy-y*scale]}
function pixToMm(px,py){return [(px-ox)/scale,(oy-py)/scale]}
function drawGrid(){
  g.clearRect(0,0,grid.width,grid.height);
  const [x0,yt]=mmToPix(-1000,1000),[x1,yb]=mmToPix(1000,-1000);
  g.fillStyle='rgba(0,55,28,.25)';g.fillRect(x0,yt,x1-x0,yb-yt);
  g.strokeStyle='rgba(0,230,110,.95)';g.lineWidth=2;g.strokeRect(x0,yt,x1-x0,yb-yt);
  for(let x=-1000;x<=1000;x+=100){let [px,_]=mmToPix(x,0);g.strokeStyle=x%500===0?'rgba(80,190,255,.36)':'rgba(80,190,255,.17)';g.lineWidth=x%500===0?1.5:1;g.beginPath();g.moveTo(px,yt);g.lineTo(px,yb);g.stroke();}
  for(let y=-1000;y<=1000;y+=100){let [_,py]=mmToPix(0,y);g.strokeStyle=y%500===0?'rgba(80,190,255,.36)':'rgba(80,190,255,.17)';g.lineWidth=y%500===0?1.5:1;g.beginPath();g.moveTo(x0,py);g.lineTo(x1,py);g.stroke();}
  let [lx,fy]=mmToPix(-ROBOT_W/2,LIDAR_FRONT),[rx,by]=mmToPix(ROBOT_W/2,-80);
  g.fillStyle='rgba(0,183,255,.16)';g.strokeStyle='#00b7ff';g.lineWidth=2;g.fillRect(lx,fy,rx-lx,by-fy);g.strokeRect(lx,fy,rx-lx,by-fy);
  let [cx,cy]=mmToPix(0,0);g.beginPath();g.arc(cx,cy,7,0,Math.PI*2);g.fillStyle='#00b7ff';g.fill();
  g.beginPath();g.moveTo(cx,cy);let [fx,fy2]=mmToPix(0,285);g.lineTo(fx,fy2);g.strokeStyle='#00b7ff';g.lineWidth=3;g.stroke();
  let [_,s1]=mmToPix(0,FRONT_STOP),[__,s2]=mmToPix(0,TARGET_STOP);
  g.setLineDash([8,6]);g.strokeStyle='#ffbf00';g.beginPath();g.moveTo(x0,s1);g.lineTo(x1,s1);g.stroke();g.strokeStyle='#ff4b70';g.beginPath();g.moveTo(x0,s2);g.lineTo(x1,s2);g.stroke();g.setLineDash([]);
  g.font='12px Consolas';g.fillStyle='#ffbf00';g.fillText('front stop raw 300mm',x0+8,s1-7);g.fillStyle='#ff4b70';g.fillText('target spray stop raw 250mm',x0+8,s2-7);
}
function markerColor(n){return n==='m1'?'#ff4b70':n==='m2'?'#ffbf00':'#3af07a'}
function drawRoute(){
  if(!routePath || routePath.length<2)return;
  d.lineJoin='round';d.lineCap='round';
  d.beginPath();routePath.forEach((p,i)=>{let [px,py]=mmToPix(p.x,p.y);if(i===0)d.moveTo(px,py);else d.lineTo(px,py)});
  d.strokeStyle='rgba(58,240,122,.22)';d.lineWidth=ROUTE_RADIUS*2*scale;d.stroke();
  d.beginPath();routePath.forEach((p,i)=>{let [px,py]=mmToPix(p.x,p.y);if(i===0)d.moveTo(px,py);else d.lineTo(px,py)});
  d.strokeStyle='#3af07a';d.lineWidth=4;d.stroke();
  for(const p of routePath){let [px,py]=mmToPix(p.x,p.y);d.beginPath();d.arc(px,py,5,0,Math.PI*2);d.fillStyle='white';d.fill();}
}
function drawMarker(n){
  const m=markers[n];if(!m)return;
  let [px,py]=mmToPix(m.x,m.y),c=markerColor(n);
  d.beginPath();d.arc(px,py,15,0,Math.PI*2);d.strokeStyle=m.locked?c:'#888';d.lineWidth=3;d.stroke();d.fillStyle='rgba(0,0,0,.36)';d.fill();
  d.font='bold 13px Consolas';d.textAlign='center';d.fillStyle=c;d.fillText(n.toUpperCase(),px,py-19);d.textAlign='left';
}
function drawDyn(){
  d.clearRect(0,0,dyn.width,dyn.height);
  for(const p of scan){let [px,py]=mmToPix(p[0],p[1]);let b=Math.max(120,Math.min(255,p[2]*3));d.fillStyle=`rgb(${b},${Math.min(255,90+b/2)},255)`;d.fillRect(px-2.5,py-2.5,5,5);}
  for(const p of blockedPts){let [px,py]=mmToPix(p.x,p.y);d.beginPath();d.arc(px,py,6,0,Math.PI*2);d.fillStyle='#ff4b70';d.fill();}
  drawRoute();drawMarker('m1');drawMarker('m2');drawMarker('home');
}
function setMode(m){mode=m;['m1','m2','home'].forEach(x=>$('b_'+x).classList.remove('active'));$('b_'+m).classList.add('active')}
dyn.addEventListener('click',async ev=>{if(!mode){alert('Choose SET M1 / SET M2 / SET HOME first.');return}const r=dyn.getBoundingClientRect();const [x,y]=pixToMm(ev.clientX-r.left,ev.clientY-r.top);const js=await(await fetch('/api/set_marker',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name:mode,x:x,y:y})})).json();if(!js.ok)alert(js.msg)});
async function clearAll(){await fetch('/api/clear',{method:'POST'});routePath=[];blockedPts=[]}
async function clearRoute(){await fetch('/api/clear_route',{method:'POST'});routePath=[];blockedPts=[];$('routeMsg').textContent='Route cleared.'}
async function planRoute(name){const js=await(await fetch('/api/plan',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({target:name})})).json();$('routeMsg').textContent=js.msg||'';routePath=js.path||[];blockedPts=js.blocked||[];if(!js.ok)alert(js.msg);drawDyn()}
async function toggleRobot(){const js=await(await fetch('/api/toggle_robot',{method:'POST'})).json();setRobot(js.robot_mode)}
function setRobot(on){$('robotBtn').textContent=on?'ROBOT MODE ON':'ROBOT MODE OFF';$('robotBtn').classList.toggle('green',on);$('robotBtn').classList.toggle('danger',!on)}
async function testYahboom(){if(!confirm('Wheels OFF GROUND. Run 2 second motor test?'))return;const js=await(await fetch('/api/test_yahboom',{method:'POST'})).json();if(!js.ok)alert(js.msg)}
function markerLine(id,m){const el=$(id);if(!m){el.innerHTML='<div class="markerTop"><b>'+id.toUpperCase()+'</b><span class="empty">EMPTY</span></div><div class="small">Click SET '+id.toUpperCase()+' then click a LiDAR cluster.</div>';return}let cls=m.locked?'locked':'lost',txt=m.locked?'LOCKED':'LOST';el.innerHTML='<div class="markerTop"><b>'+id.toUpperCase()+'</b><span class="'+cls+'">'+txt+'</span></div><div class="small">LiDAR distance: <b>'+Math.round(m.dist)+'mm</b></div><div class="small">Front gap: <b>'+Math.round(m.front_gap)+'mm</b></div><div class="small">Angle: <b>'+Math.round(m.angle)+'°</b> • cluster: '+m.points+' pts</div>'}
function events(lines){$('events').innerHTML='';for(const e of (lines||[]).slice().reverse()){const div=document.createElement('div');div.className='e'+e.typ;div.textContent='['+e.t+'] '+e.msg;$('events').appendChild(div)}}
async function poll(){
  try{
    const s=await(await fetch('/api/state?t='+Date.now())).json();
    scan=s.scan||[];markers=s.markers||{};routePath=s.route.path||routePath;blockedPts=s.route.blocked||blockedPts;
    $('status').textContent=s.status;$('hz').textContent=s.scan_hz;$('bps').textContent=s.raw_bps;$('ok').textContent=s.ok;$('bad').textContent=s.bad;$('points').textContent=s.points;$('frontraw').textContent=s.front_raw||'--';$('frontgap').textContent=s.front_gap||'--';$('err').textContent=s.err||'';$('robotText').textContent=s.robot_text||'';
    const esp=s.esp||{};
    $('espn1').textContent=esp.n1_online?'ONLINE':'OFFLINE';
    $('espa1').textContent=esp.alert1?'ALERT':'CLEAR';
    $('espn2').textContent=esp.n2_online?'ONLINE':'OFFLINE';
    $('espa2').textContent=esp.alert2?'ALERT':'CLEAR';
    $('espLine').textContent='Last ESP: '+(esp.last_line||'none')+' | age '+(esp.age_s??'--')+'s';

    setRobot(!!s.robot_mode);markerLine('m1',s.marker_info.m1);markerLine('m2',s.marker_info.m2);markerLine('home',s.marker_info.home);events(s.events);drawDyn();
  }catch(e){$('status').textContent='POLL ERR'}
  setTimeout(poll,160);
}
window.addEventListener('resize',resizeCanvas);resizeCanvas();poll();setInterval(()=>{$('clock').textContent=new Date().toLocaleTimeString()},1000);
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(PAGE)

@app.route("/api/state")
def api_state():
    with lock:
        out = dict(S)
        out["markers"] = dict(S["markers"])
        out["marker_info"] = dict(S["marker_info"])
        out["events"] = list(S["events"])
        out["scan"] = list(S["ui_scan"])
        out["route"] = dict(S["route"])

    out["robot_mode"] = robot_mode
    out["robot_busy"] = robot_busy
    out["robot_text"] = robot_text
    out["robot_error"] = robot_error
    out["esp"] = read_esp_status()
    return jsonify(out)

@app.route("/api/set_marker", methods=["POST"])
def api_set_marker():
    data = request.get_json(force=True)
    name = str(data.get("name", "")).lower()

    if name not in ["m1", "m2", "home"]:
        return jsonify(ok=False, msg="Bad marker name.")

    pts = scan_points(all_points=True)
    p = nearest_point(float(data.get("x", 0)), float(data.get("y", 0)), pts, MARKER_CLICK_SNAP_MM)

    if not p:
        return jsonify(ok=False, msg="No LiDAR dot close enough. Click directly on a visible dot/cluster.")

    m = make_cluster(p, pts)

    with lock:
        S["markers"][name] = m
        S["marker_info"][name] = marker_info(m)

    event(f"{name.upper()} locked at {int(m['dist'])}mm, {int(m['angle'])}°.", "k")
    return jsonify(ok=True, msg=f"{name.upper()} locked.")

@app.route("/api/clear", methods=["POST"])
def api_clear():
    with lock:
        S["markers"] = {"m1": None, "m2": None, "home": None}
        S["marker_info"] = {"m1": None, "m2": None, "home": None}
        S["route"] = {"state": "IDLE", "message": "Cleared.", "path": [], "blocked": [], "target": "", "type": ""}

    event("Markers and route cleared.", "w")
    return jsonify(ok=True)

@app.route("/api/clear_route", methods=["POST"])
def api_clear_route():
    with lock:
        S["route"] = {"state": "IDLE", "message": "Route cleared.", "path": [], "blocked": [], "target": "", "type": ""}
    return jsonify(ok=True)

@app.route("/api/plan", methods=["POST"])
def api_plan():
    target = str(request.get_json(force=True).get("target", "")).lower()

    if target not in ["m1", "m2", "home"]:
        return jsonify(ok=False, msg="Bad target.")

    res = plan_to(target)

    with lock:
        S["route"] = {
            "state": res.get("type", "NO PATH"),
            "message": res.get("msg", ""),
            "path": res.get("path", []),
            "blocked": res.get("blocked", []),
            "target": target,
            "type": res.get("type", "")
        }

    event(res.get("msg", "route result"), "k" if res.get("ok") else "e")
    return jsonify(res)

@app.route("/api/toggle_robot", methods=["POST"])
def api_toggle_robot():
    global robot_mode
    robot_mode = not robot_mode
    event("ROBOT MODE ENABLED. Wheels off ground." if robot_mode else "ROBOT MODE DISABLED.", "w" if robot_mode else "k")
    return jsonify(ok=True, robot_mode=robot_mode)

@app.route("/api/test_yahboom", methods=["POST"])
def api_test_yahboom():
    global robot_busy

    if not robot_mode:
        return jsonify(ok=False, msg="Robot mode is OFF.")

    if robot_busy:
        return jsonify(ok=False, msg="Robot test already running.")

    robot_busy = True
    threading.Thread(target=yahboom_worker, daemon=True).start()
    return jsonify(ok=True, msg="Yahboom test started.")

if __name__ == "__main__":
    print("FPMS A3 Planner")
    print(f"Open http://0.0.0.0:{WEB_PORT}")
    event("FPMS A3 starting: fixed 200cm × 200cm coverage, costmap inflation, A* reroute.", "i")
    threading.Thread(target=lidar_loop, daemon=True).start()
    threading.Thread(target=metrics_loop, daemon=True).start()
    app.run(host="0.0.0.0", port=WEB_PORT, threaded=True, debug=False)

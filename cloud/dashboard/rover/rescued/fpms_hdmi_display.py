#!/usr/bin/env python3
"""FPMS bird's eye display - direct framebuffer + touch"""
import json, time, math, urllib.request, threading, struct, glob, os
from PIL import Image, ImageDraw, ImageFont

FB_W, FB_H = 1920, 1080
ARENA_W, ARENA_H = 600.0, 1200.0

MX, MY = 40, 30
MW, MH = 480, 960
SX = MW / ARENA_W
SY = MH / ARENA_H

# All colors RGB
BLACK = (0, 0, 0)
DARK = (8, 15, 25)
GRID_C = (30, 70, 100)
ARENA_C = (0, 255, 136)
ARENA_F = (5, 20, 12)
ROBOT_C = (0, 183, 255)
M1_C = (255, 75, 112)
M2_C = (255, 191, 0)
HOME_C = (58, 240, 122)
LIDAR_C = (220, 200, 255)
OBS_C = (255, 75, 112)
WHITE = (255, 255, 255)
CYAN = (0, 200, 255)
RED = (255, 60, 90)
GREEN = (50, 255, 120)
YELLOW = (255, 200, 0)
GRAY = (160, 170, 180)

try:
    FT = lambda s: ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", s)
    FR = lambda s: ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", s)
    f_title = FT(44)
    f_big = FT(34)
    f_med = FT(28)
    f_sm = FR(22)
    f_ev = FR(18)
except:
    f_title = f_big = f_med = f_sm = f_ev = ImageFont.load_default()

WAYPOINTS = [("HOME", 500, 50, HOME_C), ("M1", 100, 1100, M1_C), ("M2", 500, 1100, M2_C)]

def mm2px(x, y):
    return int(MX + x * SX), int(MY + MH - y * SY)

def r2w(rx, ry, ox, oy, ot):
    ct, st = math.cos(ot), math.sin(ot)
    return ox + rx*ct + ry*st, oy - rx*st + ry*ct

# === TOUCH INPUT ===
_tx, _ty, _tapped = 0, 0, False

def find_touch():
    for dev in sorted(glob.glob("/dev/input/event*")):
        try:
            with open(dev, "rb") as f:
                name_bytes = bytearray(256)
                import fcntl
                fcntl.ioctl(f, 0x80FF4506, name_bytes)  # EVIOCGNAME
                name = name_bytes.split(b'\x00')[0].decode()
                if "touch" in name.lower() or "goodix" in name.lower() or "usb" in name.lower():
                    return dev
        except:
            continue
    # Fallback: try last event device
    devs = sorted(glob.glob("/dev/input/event*"))
    return devs[-1] if devs else None

def touch_thread():
    global _tx, _ty, _tapped
    dev = find_touch()
    if not dev:
        return
    f = open(dev, "rb")
    ax, ay, down = 0, 0, False
    while True:
        try:
            data = f.read(24)
            if len(data) < 24: continue
            _, _, typ, code, val = struct.unpack("llHHi", data)
            if typ == 3:  # EV_ABS
                if code == 0: ax = val
                elif code == 1: ay = val
            elif typ == 1 and code == 330:  # BTN_TOUCH
                if val == 0 and down:
                    _tx = int(ax * FB_W / 800)
                    _ty = int(ay * FB_H / 480)
                    _tapped = True
                down = val == 1
        except:
            time.sleep(0.1)

threading.Thread(target=touch_thread, daemon=True).start()

BUTTONS = [
    (580, 870, 320, 65, "RUN M1", RED, "/api/run", {"target": "m1"}),
    (920, 870, 320, 65, "RUN M2", YELLOW, "/api/run", {"target": "m2"}),
    (1260, 870, 320, 65, "ROBOT ON/OFF", GREEN, "/api/toggle_robot", {}),
    (580, 950, 320, 65, "QUEUE M2", CYAN, "/api/queue", {"target": "m2"}),
    (920, 950, 320, 65, "STOP", (255, 0, 0), "/api/stop", {}),
    (1260, 950, 320, 65, "RESET ODOM", GRAY, "/api/clear_markers", {}),
]

def handle_touch():
    global _tapped
    if not _tapped: return
    _tapped = False
    for bx, by, bw, bh, lbl, col, url, body in BUTTONS:
        if bx <= _tx <= bx+bw and by <= _ty <= by+bh:
            try:
                req = urllib.request.Request(
                    f"http://localhost:8085{url}",
                    data=json.dumps(body).encode() if body else b"{}",
                    headers={"Content-Type": "application/json"}, method="POST")
                urllib.request.urlopen(req, timeout=2)
            except: pass
            return

while True:
    try:
        raw = urllib.request.urlopen("http://localhost:8085/api/state?t=" + str(int(time.time()*1000)), timeout=1).read()
        data = json.loads(raw)
    except:
        data = {}

    odom = data.get("odom", {"x":0,"y":0,"theta":0})
    mission = data.get("mission", {"state":"--","target":"--","message":"--"})
    esp = data.get("esp", {})
    scan = data.get("scan", [])
    obstacles = data.get("obstacles", [])
    front = data.get("front_raw", "--")
    batt = data.get("battery_v", 0)
    ox, oy, ot = odom["x"], odom["y"], odom["theta"]

    img = Image.new('RGB', (FB_W, FB_H), BLACK)
    d = ImageDraw.Draw(img)

    # Arena
    d.rectangle([mm2px(0, ARENA_H), mm2px(ARENA_W, 0)], fill=ARENA_F, outline=ARENA_C, width=3)

    # Grid
    for x in range(0, 601, 100):
        d.line([mm2px(x, 0), mm2px(x, ARENA_H)], fill=GRID_C, width=2)
    for y in range(0, 1201, 100):
        d.line([mm2px(0, y), mm2px(ARENA_W, y)], fill=GRID_C, width=2)

    # Waypoints
    for name, wx, wy, col in WAYPOINTS:
        px, py = mm2px(wx, wy)
        d.ellipse([px-14, py-14, px+14, py+14], outline=col, width=3)
        d.text((px-25, py-35), name, fill=col, font=f_med)

    # Obstacles
    for ob in obstacles:
        px, py = mm2px(ob.get("x",0), ob.get("y",0))
        r = max(int(ob.get("r",80) * SX), 5)
        d.ellipse([px-r, py-r, px+r, py+r], outline=OBS_C, width=2)

    # LiDAR
    for p in scan:
        wx, wy = r2w(p[0], p[1], ox, oy, ot)
        if 0 <= wx <= 600 and 0 <= wy <= 1200:
            px, py = mm2px(wx, wy)
            d.rectangle([px-2, py-2, px+2, py+2], fill=LIDAR_C)

    # Robot
    corners = [(-115, 160), (115, 160), (115, -80), (-115, -80)]
    pts = [mm2px(*r2w(rx, ry, ox, oy, ot)) for rx, ry in corners]
    d.polygon(pts, outline=ROBOT_C, width=3)
    d.line([mm2px(*r2w(0, 0, ox, oy, ot)), mm2px(*r2w(0, 220, ox, oy, ot))], fill=RED, width=3)

    # === RIGHT PANEL ===
    TX, TY = 580, 30

    d.text((TX, TY), "SCORCH SENTINEL", fill=RED, font=f_title); TY += 55
    d.text((TX, TY), "Fire Prevention System", fill=CYAN, font=f_sm); TY += 35
    d.line([(TX, TY), (1880, TY)], fill=GRID_C, width=2); TY += 15

    # Position
    d.text((TX, TY), f"X:{ox:.0f}  Y:{oy:.0f}  HDG:{math.degrees(ot):.1f}°", fill=WHITE, font=f_med); TY += 38
    d.text((TX, TY), f"FRONT: {front}mm", fill=WHITE, font=f_med)
    batt_col = GREEN if batt and batt > 11 else YELLOW if batt and batt > 10 else RED
    d.text((TX + 400, TY), f"BATT: {batt}V", fill=batt_col, font=f_med); TY += 38
    d.line([(TX, TY), (1880, TY)], fill=GRID_C, width=2); TY += 15

    # Mission
    state = mission.get("state", "--")
    sc = GREEN if state == "DONE" else RED if "ERR" in state else YELLOW
    d.text((TX, TY), f"{state} > {mission.get('target','--')}", fill=sc, font=f_big); TY += 42
    d.text((TX, TY), mission.get("message", "--")[:45], fill=GRAY, font=f_sm); TY += 32
    d.line([(TX, TY), (1880, TY)], fill=GRID_C, width=2); TY += 15

    # ESP-NOW
    d.text((TX, TY), "ESP-NOW SENSORS", fill=CYAN, font=f_med); TY += 35
    n1c = GREEN if esp.get("n1_online") else RED
    a1c = RED if esp.get("alert1") else GREEN
    d.text((TX, TY), "ZONE 1:", fill=WHITE, font=f_sm)
    d.text((TX+120, TY), "ON" if esp.get("n1_online") else "OFF", fill=n1c, font=f_sm)
    d.text((TX+220, TY), "ALERT" if esp.get("alert1") else "CLEAR", fill=a1c, font=f_sm)
    n2c = GREEN if esp.get("n2_online") else RED
    a2c = RED if esp.get("alert2") else GREEN
    d.text((TX+450, TY), "ZONE 2:", fill=WHITE, font=f_sm)
    d.text((TX+570, TY), "ON" if esp.get("n2_online") else "OFF", fill=n2c, font=f_sm)
    d.text((TX+670, TY), "ALERT" if esp.get("alert2") else "CLEAR", fill=a2c, font=f_sm)
    TY += 35
    d.line([(TX, TY), (1880, TY)], fill=GRID_C, width=2); TY += 15

    # Events log
    d.text((TX, TY), "LIVE LOG", fill=CYAN, font=f_med); TY += 32
    for ev in reversed(data.get("events", [])[-10:]):
        txt = f"[{ev.get('t','')}] {ev.get('msg','')[:50]}"
        d.text((TX, TY), txt, fill=GRAY, font=f_ev)
        TY += 22
        if TY > 855: break

    # Buttons
    for bx, by, bw, bh, lbl, col, url, body in BUTTONS:
        d.rectangle([bx, by, bx+bw, by+bh], outline=col, width=3)
        tw = len(lbl) * 15
        d.text((bx + (bw-tw)//2, by+18), lbl, fill=col, font=f_med)

    handle_touch()

    # Write RGB to framebuffer as BGRA
    rgba = img.convert("RGBA")
    with open("/dev/fb0", "wb") as fb:
        fb.write(rgba.tobytes("raw", "BGRA"))

    time.sleep(0.25)

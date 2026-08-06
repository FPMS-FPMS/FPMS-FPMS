#!/usr/bin/env python3
import os, json, time, math, glob, atexit, signal, threading
from pathlib import Path
from flask import Flask, jsonify, render_template_string

APP_PORT = 8089
DATA_FILE = Path.home() / "fpms_teachin" / "routes_FINAL.json"

WHEEL_DIAMETER_MM = 70.0
MD520_RPM = os.environ.get("FPMS_MD520_RPM", "333").strip()
GEAR_RATIO_BY_RPM = {"550": 19, "333": 30, "205": 56}
if MD520_RPM not in GEAR_RATIO_BY_RPM:
    raise SystemExit("FPMS_MD520_RPM must be 205, 333, or 550")

GEAR_RATIO = GEAR_RATIO_BY_RPM[MD520_RPM]
MAGNETIC_RING_LINES = 11
QUAD_MULT = int(os.environ.get("FPMS_QUAD_MULT", "4"))
COUNTS_PER_WHEEL_REV = MAGNETIC_RING_LINES * GEAR_RATIO * QUAD_MULT
COUNTS_PER_MM = COUNTS_PER_WHEEL_REV / (math.pi * WHEEL_DIAMETER_MM)
DISTANCE_SCALE = float(os.environ.get("FPMS_DISTANCE_SCALE", "1.0"))

CAR_TYPE = int(os.environ.get("ROBOT_CAR_TYPE", "1"))
FORCED_PORT = os.environ.get("ROBOT_PORT", "").strip()
DRY_RUN = os.environ.get("DRY_RUN", "0") == "1"

DRIVE_FAST = int(os.environ.get("FPMS_DRIVE_FAST", "55"))
DRIVE_SLOW = int(os.environ.get("FPMS_DRIVE_SLOW", "32"))
DRIVE_TOL_COUNTS = float(os.environ.get("FPMS_DRIVE_TOL_COUNTS", "14"))
DRIVE_SLOW_ZONE = float(os.environ.get("FPMS_DRIVE_SLOW_ZONE", "170"))
MAX_DRIVE_SECONDS = float(os.environ.get("FPMS_MAX_DRIVE_SECONDS", "14"))

TURN_POWER = int(os.environ.get("FPMS_TURN_POWER", "30"))
TURN_PULSES = int(os.environ.get("FPMS_TURN_PULSES", "6"))
TURN_PULSE_SECONDS = float(os.environ.get("FPMS_TURN_PULSE_SECONDS", "0.055"))
TURN_PAUSE_SECONDS = float(os.environ.get("FPMS_TURN_PAUSE_SECONDS", "0.080"))
REPLAY_SETTLE_SECONDS = float(os.environ.get("FPMS_REPLAY_SETTLE_SECONDS", "0.05"))

app = Flask(__name__)
bot = None
bot_port = None
bot_error = None
last_status = "Booting FPMS FINAL teach-in server..."
move_lock = threading.RLock()
route_lock = threading.RLock()
recording_route = None
routes = {"1": [], "2": []}

def set_status(msg):
    global last_status
    last_status = msg
    print(msg, flush=True)

def distance_cm_to_counts(cm):
    return cm * 10.0 * COUNTS_PER_MM * DISTANCE_SCALE

def load_routes():
    global routes
    try:
        if DATA_FILE.exists():
            data = json.loads(DATA_FILE.read_text())
            routes = {"1": data.get("1", []), "2": data.get("2", [])}
    except Exception as e:
        print("Route load error:", e, flush=True)

def save_routes():
    DATA_FILE.parent.mkdir(parents=True, exist_ok=True)
    DATA_FILE.write_text(json.dumps(routes, indent=2))

def possible_ports():
    ports = []
    if FORCED_PORT:
        ports.append(FORCED_PORT)
    ports += ["/dev/myserial", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]
    ports += sorted(glob.glob("/dev/serial/by-id/*"))
    out, seen = [], set()
    for p in ports:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out

def safe_set_car_run(state, speed, adjust=False):
    if DRY_RUN or bot is None:
        return
    try:
        bot.set_car_run(int(state), int(speed), bool(adjust))
    except TypeError:
        bot.set_car_run(int(state), int(speed))


def direct_motor(m1, m2, m3, m4):
    if DRY_RUN or bot is None:
        return
    bot.set_motor(int(m1), int(m2), int(m3), int(m4))


def safe_set_car_motion(vx, vy, vz):
    if DRY_RUN or bot is None:
        return
    try:
        bot.set_car_motion(vx, vy, vz)
    except Exception:
        pass

def stop_robot():
    if DRY_RUN or bot is None:
        return
    try:
        safe_set_car_run(0, 0, False)
        safe_set_car_motion(0, 0, 0)
    except Exception as e:
        print("Stop error:", e, flush=True)

def read_encoders():
    if DRY_RUN or bot is None:
        return [0.0, 0.0, 0.0, 0.0]
    e = bot.get_motor_encoder()
    return [float(e[0]), float(e[1]), float(e[2]), float(e[3])]

def encoder_delta_counts(start, now):
    deltas = [abs(now[i] - start[i]) for i in range(4)]
    active = [d for d in deltas if d > 2]
    return 0.0 if not active else sum(active) / len(active)

def connect_bot():
    global bot, bot_port, bot_error
    if DRY_RUN:
        bot_port = "DRY_RUN"
        bot_error = None
        set_status("DRY_RUN mode: server runs, robot movement disabled.")
        return
    try:
        from Rosmaster_Lib import Rosmaster
    except Exception as e:
        bot_error = "Rosmaster_Lib import failed: " + str(e)
        set_status("ERROR: " + bot_error)
        return

    last_err = None
    for port in possible_ports():
        try:
            set_status("Trying Yahboom STM32 board on " + port + "...")
            b = Rosmaster(car_type=CAR_TYPE, com=port, debug=False)
            try:
                b.create_receive_threading()
            except Exception:
                pass
            time.sleep(0.4)
            try:
                b.set_auto_report_state(True, False)
            except Exception:
                pass
            time.sleep(0.3)
            bot = b
            bot_port = port
            bot_error = None
            stop_robot()
            enc = read_encoders()
            set_status("Connected on " + port + "\nEncoders: " + str(enc))
            return
        except Exception as e:
            last_err = e
            print("Port failed " + port + ": " + str(e), flush=True)

    bot_error = "Could not connect to Yahboom board. Last error: " + str(last_err)
    set_status("ERROR: " + bot_error)

def add_step(step):
    if recording_route in ("1", "2"):
        with route_lock:
            routes[recording_route].append(step)
            save_routes()
        return True
    return False

def drive_step(label, state, cm, record=True):
    with move_lock:
        if bot_error:
            return False, bot_error
        target = distance_cm_to_counts(cm)
        start = read_encoders()
        set_status("\n".join([
            "Running: " + label,
            "Target counts: " + str(round(target, 1)),
            "Start encoders: " + str(start),
        ]))
        t0 = time.time()
        final_delta = 0.0
        try:
            while True:
                now = read_encoders()
                delta = encoder_delta_counts(start, now)
                final_delta = delta
                remaining = target - delta
                if remaining <= DRIVE_TOL_COUNTS:
                    break
                if time.time() - t0 > MAX_DRIVE_SECONDS:
                    set_status("Drive timeout: " + label + "\nMoved counts: " + str(round(delta, 1)) + "/" + str(round(target, 1)))
                    break
                speed = DRIVE_SLOW if remaining < DRIVE_SLOW_ZONE else DRIVE_FAST
                safe_set_car_run(state, speed, True)
                time.sleep(0.03)
        finally:
            stop_robot()
            time.sleep(0.12)

        end = read_encoders()
        recorded = add_step({"kind": "drive", "label": label, "state": state, "cm": cm}) if record else False
        msg = "\n".join([
            "Done: " + label,
            "Moved counts: " + str(round(final_delta, 1)) + "/" + str(round(target, 1)),
            "End encoders: " + str(end),
            "Recorded to route #" + str(recording_route) if recorded else "Not recording",
        ])
        set_status(msg)
        return True, msg

def turn_step(label, state, record=True):
    with move_lock:
        if bot_error:
            return False, bot_error

        # DIRECT MOTOR TURN MODE.
        # This bypasses Yahboom set_car_run spin states.
        # From previous FPMS working turn logic:
        # right turn = left side forward, right side backward.
        # set_motor order used here: M1, M2, M3, M4.
        power = int(os.environ.get("FPMS_DIRECT_TURN_POWER", "42"))
        pulses = int(os.environ.get("FPMS_DIRECT_TURN_PULSES", "9"))
        pulse_s = float(os.environ.get("FPMS_DIRECT_TURN_PULSE_SECONDS", "0.070"))
        pause_s = float(os.environ.get("FPMS_DIRECT_TURN_PAUSE_SECONDS", "0.060"))

        # state 6 = right turn, state 5 = left turn
        if int(state) == 6:
            motor_tuple = (-power, -power, power, power)
        else:
            motor_tuple = (power, power, -power, -power)

        set_status("\n".join([
            "Running: " + label,
            "DIRECT set_motor pulsed turn mode",
            "Motors: " + str(motor_tuple),
            "Power: " + str(power),
            "Pulses: " + str(pulses),
            "Pulse seconds: " + str(pulse_s),
            "Pause seconds: " + str(pause_s),
        ]))

        start = read_encoders()
        try:
            for _ in range(pulses):
                direct_motor(*motor_tuple)
                time.sleep(pulse_s)
                stop_robot()
                time.sleep(pause_s)
        finally:
            stop_robot()
            time.sleep(0.12)

        end = read_encoders()
        delta = encoder_delta_counts(start, end)
        recorded = add_step({"kind": "turn", "label": label, "state": state}) if record else False
        msg = "\n".join([
            "Done: " + label,
            "Direct motor turn encoder delta: " + str(round(delta, 1)),
            "Recorded to route #" + str(recording_route) if recorded else "Not recording",
        ])
        set_status(msg)
        return True, msg

def toggle_record(route_id):
    global recording_route
    with route_lock:
        if recording_route == route_id:
            recording_route = None
            save_routes()
            set_status("Saved route #" + route_id + ". Idle gaps were not recorded.")
            return True, "Saved route #" + route_id
        recording_route = route_id
        routes[route_id] = []
        save_routes()
        set_status("Recording route #" + route_id + "\nOnly button movement steps are saved. Idle time is ignored.")
        return True, "Recording route #" + route_id

def execute_step(step):
    if step.get("kind") == "drive":
        return drive_step(step.get("label", "drive"), int(step["state"]), float(step["cm"]), record=False)
    if step.get("kind") == "turn":
        return turn_step(step.get("label", "turn"), int(step["state"]), record=False)
    return False, "Unknown route step: " + str(step)

def replay(route_id):
    global recording_route
    with route_lock:
        steps = list(routes.get(route_id, []))
    if not steps:
        return False, "Route #" + route_id + " is empty."
    old = recording_route
    recording_route = None
    try:
        set_status("Replaying route #" + route_id + " with " + str(len(steps)) + " steps. No idle gaps.")
        for i, step in enumerate(steps, 1):
            set_status("Replay route #" + route_id + " step " + str(i) + "/" + str(len(steps)) + ": " + step.get("label", step.get("kind", "step")))
            ok, msg = execute_step(step)
            if not ok:
                return False, msg
            time.sleep(REPLAY_SETTLE_SECONDS)
        set_status("Finished replay route #" + route_id)
        return True, "Finished replay route #" + route_id
    finally:
        recording_route = old

def wipe():
    global routes, recording_route
    with route_lock:
        routes = {"1": [], "2": []}
        recording_route = None
        save_routes()
    stop_robot()
    set_status("Wiped all route data.")
    return True, "Wiped all route data."

HTML = """
<!doctype html><html><head><title>FPMS FINAL Teach-In</title><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body{font-family:Arial,sans-serif;background:#0f172a;color:#e5e7eb;margin:0;padding:18px}.card{max-width:850px;margin:auto;background:#111827;border:1px solid #334155;border-radius:16px;padding:16px}button{font-size:16px;padding:16px;border:0;border-radius:14px;background:#2563eb;color:white;font-weight:700;cursor:pointer}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px;margin-top:16px}.status{white-space:pre-wrap;background:#020617;border:1px solid #475569;border-radius:12px;padding:12px;margin-top:14px;min-height:120px}.record{background:#dc2626}.replay{background:#16a34a}.wipe{background:#7c2d12}.long{background:#4f46e5}.small{color:#94a3b8;font-size:14px;line-height:1.45}
</style></head><body><div class="card"><h1>FPMS FINAL Teach-In Server</h1><div class="small">Open: <b>http://fpms-pi.local:8089</b><br>Encoder distance. Slow strong pulsed turns. Recording saves movements only, not idle gaps.</div><div id="status" class="status">Loading...</div><div class="grid">
<button onclick="sendAction('forward10')">Forward 10cm</button><button onclick="sendAction('backward10')">Backwards 10cm</button><button onclick="sendAction('left10')">Left 10cm</button><button onclick="sendAction('right10')">Right 10 cm</button><button class="long" onclick="sendAction('forward40')">Forward 40cm</button><button class="long" onclick="sendAction('backward40')">Backwards 40cm</button><button onclick="sendAction('turn_right45')">Turn Right 45 degrees</button><button onclick="sendAction('turn_left45')">Turn left 45 degrees</button><button class="record" onclick="sendAction('record1')">Record route #1</button><button class="record" onclick="sendAction('record2')">Record route #2</button><button class="replay" onclick="sendAction('replay1')">Replay route #1</button><button class="replay" onclick="sendAction('replay2')">Replay route #2</button><button class="wipe" onclick="sendAction('wipe')">Wipe all route data.</button>
</div></div><script>
async function refreshStatus(){const r=await fetch('/api/status');const d=await r.json();document.getElementById('status').textContent=d.status+'\\n\\nPort: '+d.port+'\\nRecording: '+d.recording+'\\nRoute #1 steps: '+d.route1_steps+'\\nRoute #2 steps: '+d.route2_steps+'\\nWheel: '+d.wheel_diameter_mm+' mm\\nMotor RPM: '+d.md520_rpm+'\\nGear: 1:'+d.gear_ratio+'\\nCounts/10cm: '+d.counts_per_10cm+'\\nCounts/40cm: '+d.counts_per_40cm+'\\nTurn power: '+d.turn_power+'\\nTurn pulses: '+d.turn_pulses+'\\nPulse/pause: '+d.turn_pulse_seconds+'/'+d.turn_pause_seconds;}
async function sendAction(a){document.getElementById('status').textContent='Sending '+a+'...';await fetch('/api/action/'+a,{method:'POST'});await refreshStatus();}
refreshStatus();setInterval(refreshStatus,1200);
</script></body></html>
"""

@app.route("/")
def index():
    return render_template_string(HTML)

@app.route("/api/status")
def api_status():
    return jsonify({
        "status": last_status,
        "port": bot_port if bot_port else "not connected",
        "error": bot_error,
        "recording": recording_route if recording_route else "none",
        "route1_steps": len(routes.get("1", [])),
        "route2_steps": len(routes.get("2", [])),
        "wheel_diameter_mm": WHEEL_DIAMETER_MM,
        "md520_rpm": MD520_RPM,
        "gear_ratio": GEAR_RATIO,
        "counts_per_10cm": round(distance_cm_to_counts(10), 1),
        "counts_per_40cm": round(distance_cm_to_counts(40), 1),
        "turn_power": TURN_POWER,
        "turn_pulses": TURN_PULSES,
        "turn_pulse_seconds": TURN_PULSE_SECONDS,
        "turn_pause_seconds": TURN_PAUSE_SECONDS,
    })

@app.route("/api/action/<action>", methods=["POST"])
def api_action(action):
    actions = {
        "forward10": lambda: drive_step("Forward 10cm", 1, 10.0),
        "backward10": lambda: drive_step("Backwards 10cm", 2, 10.0),
        "left10": lambda: drive_step("Left 10cm", 3, 10.0),
        "right10": lambda: drive_step("Right 10 cm", 4, 10.0),
        "forward40": lambda: drive_step("Forward 40cm", 1, 40.0),
        "backward40": lambda: drive_step("Backwards 40cm", 2, 40.0),
        "turn_right45": lambda: turn_step("Turn Right 45 degrees", 6),
        "turn_left45": lambda: turn_step("Turn left 45 degrees", 5),
        "record1": lambda: toggle_record("1"),
        "record2": lambda: toggle_record("2"),
        "replay1": lambda: replay("1"),
        "replay2": lambda: replay("2"),
        "wipe": wipe,
    }
    if action not in actions:
        return jsonify({"ok": False, "message": "Unknown action"}), 404
    ok, msg = actions[action]()
    return jsonify({"ok": ok, "message": msg})

def shutdown_handler(signum, frame):
    stop_robot()
    raise SystemExit

if __name__ == "__main__":
    load_routes()
    connect_bot()
    atexit.register(stop_robot)
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    set_status("FPMS FINAL ready. Open http://fpms-pi.local:8089")
    app.run(host="0.0.0.0", port=APP_PORT, debug=False, threaded=True)
